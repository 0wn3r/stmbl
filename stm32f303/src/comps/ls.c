#include "ls_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "periph.h"
#include "common.h"
#include "f3hw.h"
#include "ringbuf.h"


HAL_COMP(ls);

//process data from LS
HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(pos);
HAL_PIN(vel);
HAL_PIN(en);

// config data from LS
HAL_PIN(cmd_mode);
HAL_PIN(phase_mode);
HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(lq);  // q axis inductance for curpid0.lq: config lq, or l when that is 0
HAL_PIN(psi);
HAL_PIN(cur_bw);
HAL_PIN(cur_ff);
HAL_PIN(cur_ind);
HAL_PIN(max_y);
HAL_PIN(max_cur);
HAL_PIN(dac);
HAL_PIN(drop);
HAL_PIN(drop_k);

// process data to LS
HAL_PIN(dc_volt);
HAL_PIN(id_fb);
HAL_PIN(iq_fb);
HAL_PIN(ud_fb);
HAL_PIN(uq_fb);

// state data to LS
HAL_PIN(hv_temp);
HAL_PIN(mot_temp);
HAL_PIN(core_temp);
HAL_PIN(fault_in);  //fault code send to f4
HAL_PIN(ignore_fault_pin);
HAL_PIN(y);
HAL_PIN(u_fb);
HAL_PIN(v_fb);
HAL_PIN(w_fb);

// misc
HAL_PIN(pwm_volt);
HAL_PIN(crc_error);
HAL_PIN(crc_ok);
HAL_PIN(timeout);
HAL_PIN(dma_pos);
HAL_PIN(idle);
HAL_PIN(fault);  //communication fault output

HAL_PIN(dma_pos2);
HAL_PIN(arr);
HAL_PIN(dma_pos_cmd);
HAL_PIN(inc);
HAL_PIN(window);

struct ls_ctx_t {
  uint32_t timeout;
  uint32_t tx_addr;
  uint8_t send;
  volatile packet_to_hv_t packet_to_hv;
  volatile packet_from_hv_t packet_from_hv;
};

//TODO: move to ctx
// volatile packet_to_hv_t packet_to_hv;
// volatile packet_from_hv_t packet_from_hv;

f3_config_data_t config;
f3_state_data_t state;

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx = (struct ls_ctx_t *)ctx_ptr;
  // struct ls_pin_ctx_t * pins = (struct ls_pin_ctx_t *)pin_ptr;

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  // 8N1, 8x oversampling, clocked from SYSCLK; DMA requests and overrun
  // disable are set first and kept by the init, as with HAL_UART_Init()
  USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS;
  LL_USART_Disable(USART3);
  LL_USART_Init(USART3, &(LL_USART_InitTypeDef){
                            .BaudRate            = DATABAUD,
                            .DataWidth           = LL_USART_DATAWIDTH_8B,
                            .StopBits            = LL_USART_STOPBITS_1,
                            .Parity              = LL_USART_PARITY_NONE,
                            .TransferDirection   = LL_USART_DIRECTION_TX_RX,
                            .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
                            .OverSampling        = LL_USART_OVERSAMPLING_8,
                        });
  LL_USART_DisableOneBitSamp(USART3);
  USART3->CR2 &= ~(USART_CR2_LINEN | USART_CR2_CLKEN);
  USART3->CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
  LL_USART_Enable(USART3);
  while(!LL_USART_IsActiveFlag_TEACK(USART3) || !LL_USART_IsActiveFlag_REACK(USART3)) {
  }

  /**USART3 GPIO Configuration    
   PB10     ------> USART3_TX
   PB11     ------> USART3_RX 
   */
  LL_GPIO_Init(GPIOB, &(LL_GPIO_InitTypeDef){
                          .Pin        = LL_GPIO_PIN_10 | LL_GPIO_PIN_11,
                          .Mode       = LL_GPIO_MODE_ALTERNATE,
                          .Speed      = LL_GPIO_SPEED_FREQ_HIGH,
                          .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                          .Pull       = LL_GPIO_PULL_UP,
                          .Alternate  = LL_GPIO_AF_7,
                      });

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  //TX DMA
  DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR_EN);
  DMA1_Channel2->CPAR  = (uint32_t) & (USART3->TDR);
  DMA1_Channel2->CMAR  = (uint32_t) & (ctx->packet_from_hv);
  DMA1_Channel2->CNDTR = sizeof(packet_from_hv_t);
  DMA1_Channel2->CCR   = DMA_CCR_MINC | DMA_CCR_DIR;  // | DMA_CCR_PL_0 | DMA_CCR_PL_1
  DMA1->IFCR           = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CGIF2;

  //RX DMA
  DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR_EN);
  DMA1_Channel3->CPAR  = (uint32_t) & (USART3->RDR);
  DMA1_Channel3->CMAR  = (uint32_t) & (ctx->packet_to_hv);
  DMA1_Channel3->CNDTR = sizeof(packet_to_hv_t);
  DMA1_Channel3->CCR   = DMA_CCR_MINC;  // | DMA_CCR_PL_0 | DMA_CCR_PL_1
  DMA1->IFCR           = DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CGIF3;
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  config.pins.r       = 0.0;
  config.pins.l       = 0.0;
  config.pins.psi     = 0.0;
  config.pins.cur_bw  = 0.0;
  config.pins.cur_ff  = 0.0;
  config.pins.cur_ind = 0.0;
  config.pins.max_y   = 0.0;
  config.pins.max_cur = 0.0;
  config.pins.dac     = 0.0;
  config.pins.drop    = 0.0;
  config.pins.drop_k  = 0.0;
  config.pins.lq      = 0.0;

  USART3->RTOR = 16;               // 16 bits timeout
  USART3->CR2 |= USART_CR2_RTOEN;  // timeout en
  USART3->ICR |= USART_ICR_RTOCF;  // timeout clear flag

  ctx->packet_from_hv.header.len        = (sizeof(packet_from_hv_t) - sizeof(stmbl_talk_header_t)) / 4;
  ctx->packet_from_hv.header.flags.cmd  = WRITE_CONF;
  ctx->packet_from_hv.header.slave_addr = 0;
}

static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx      = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  ctx->timeout     = 0;
  ctx->tx_addr     = 0;
  ctx->send        = 0;
  PIN(crc_error)   = 0.0;
  PIN(crc_ok)      = 0.0;
  PIN(timeout)     = 0.0;
  PIN(idle)        = 0.0;
  PIN(dma_pos_cmd) = 4;
  PIN(inc)         = 5;
  PIN(window)      = 1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ls_ctx_t *ctx      = (struct ls_ctx_t *)ctx_ptr;
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  uint32_t dma_pos = sizeof(packet_to_hv_t) - DMA1_Channel3->CNDTR;

  PIN(dma_pos2) = dma_pos;
  PIN(arr)      = PWM_RES;

  if(dma_pos > PIN(window) && dma_pos < sizeof(packet_to_hv_t) - PIN(window)) {
    if(PIN(dma_pos_cmd) < dma_pos) {
      PIN(arr) = PWM_RES - PIN(inc);
    } else if(PIN(dma_pos_cmd) > dma_pos) {
      PIN(arr) = PWM_RES + PIN(inc);
    }
  }

  uint32_t fault = 0;

  if(dma_pos == sizeof(packet_to_hv_t)) {
    uint32_t crc = crc_calc((uint32_t *)&(ctx->packet_to_hv.header.slave_addr), sizeof(packet_to_hv_t) / 4 - 1);
    if(ctx->packet_to_hv.header.slave_addr == 0 && ctx->packet_to_hv.header.len == (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4 && crc == ctx->packet_to_hv.header.crc) {
      //
      // An out of range address means the f4 was flashed with a newer config
      // layout than this image knows, which is the window between updating the
      // f4 and letting it push the matching f3 image over. Drop those words
      // instead of folding them onto the last entry, where a foreign value
      // would land in whatever field happens to sit at the end.
      uint8_t a     = ctx->packet_to_hv.header.conf_addr;
      uint8_t valid = a < sizeof(config) / 4;
      a             = CLAMP(a, 0, sizeof(config) / 4 - 1);

      switch(ctx->packet_to_hv.header.flags.cmd) {
        case NO_CMD:
          break;
        case WRITE_CONF:
          if(valid) {
            config.data[a] = ctx->packet_to_hv.header.config.f32;  // TODO: first enable after complete update
          }
          break;
        case READ_CONF:
          // same reason as the write: answering an address this image does not
          // have would return the last word labelled as the one asked for.
          if(valid) {
            ctx->tx_addr = a;
          }
          break;
        case DO_RESET:
          NVIC_SystemReset();
          break;
        case BOOTLOADER:
          RTC->BKP0R = 0xDEADBEEF;
          NVIC_SystemReset();
          break;
      }

      PIN(en)               = ctx->packet_to_hv.flags.enable;
      PIN(phase_mode)       = ctx->packet_to_hv.flags.phase_type;
      PIN(cmd_mode)         = ctx->packet_to_hv.flags.cmd_type;
      PIN(ignore_fault_pin) = ctx->packet_to_hv.flags.ignore_fault_pin;
      PIN(d_cmd)            = ctx->packet_to_hv.d_cmd;
      PIN(q_cmd)            = ctx->packet_to_hv.q_cmd;
      PIN(pos)              = ctx->packet_to_hv.pos;
      PIN(vel)              = ctx->packet_to_hv.vel;

      if(ctx->packet_to_hv.flags.buf != 0x0) {
        extern struct ringbuf rx_buf;
        rb_write(&rx_buf, (void *)&(ctx->packet_to_hv.flags.buf), 1);
      }

      PIN(r)       = config.pins.r;
      PIN(l)       = config.pins.l;
      PIN(psi)     = config.pins.psi;
      PIN(cur_bw)  = config.pins.cur_bw;
      PIN(cur_ff)  = config.pins.cur_ff;
      PIN(cur_ind) = config.pins.cur_ind;
      PIN(max_y)   = config.pins.max_y;
      PIN(max_cur) = config.pins.max_cur;
      PIN(dac)     = config.pins.dac;
      PIN(drop)    = config.pins.drop;
      PIN(drop_k)  = config.pins.drop_k;
      // an interior magnet rotor has lq above ld (1.31x on X), and curpid
      // uses each for its axis' gain and decoupling. 0 keeps the old single l.
      PIN(lq) = config.pins.lq > 0.0 ? config.pins.lq : config.pins.l;
      ctx->timeout = 0;
      PIN(crc_ok)
      ++;
      if(ctx->send == 0) {
        ctx->send = 1;
      }
    } else {
      PIN(crc_error)
      ++;
      fault = 3;
    }
  } else if(ctx->timeout <= 5) {  // if no packet and no timeout, advance pos by velovity
    PIN(pos) = PIN(pos) + PIN(vel) * period;
  }


  if(USART3->ISR & USART_ISR_RTOF) {                                    // idle line
    USART3->ICR |= USART_ICR_RTOCF | USART_ICR_FECF | USART_ICR_ORECF;  // timeout clear flag
    GPIOA->BSRR |= LL_GPIO_PIN_10;

    PIN(idle)
    ++;
    if(dma_pos != sizeof(packet_to_hv_t)) {
      PIN(dma_pos) = dma_pos;
    }

    // reset rx DMA
    DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR_EN);
    DMA1_Channel3->CNDTR = sizeof(packet_to_hv_t);
    DMA1_Channel3->CCR |= DMA_CCR_EN;
    dma_pos = 0;
    GPIOA->BSRR |= LL_GPIO_PIN_10 << 16;

    //ctx->send = 1;
  }

  if(ctx->send == 2) {
    ctx->send = 0;
  }
  if(ctx->send == 1 && dma_pos != 0) {
    ctx->send = 2;
    //packet_to_hv.d_cmd = -99.0;
    state.pins.u_fb      = PIN(u_fb);
    state.pins.v_fb      = PIN(v_fb);
    state.pins.w_fb      = PIN(w_fb);
    state.pins.hv_temp   = PIN(hv_temp);
    state.pins.mot_temp  = PIN(mot_temp);
    state.pins.core_temp = PIN(core_temp);
    state.pins.y         = PIN(y);
    state.pins.dc_volt   = PIN(dc_volt);
    state.pins.pwm_volt  = PIN(pwm_volt);

    // fill tx struct
    ctx->packet_from_hv.fault             = (uint8_t)PIN(fault_in);
    ctx->packet_from_hv.id_fb             = PIN(id_fb);
    ctx->packet_from_hv.iq_fb             = PIN(iq_fb);
    ctx->packet_from_hv.ud_fb             = PIN(ud_fb);
    ctx->packet_from_hv.uq_fb             = PIN(uq_fb);
    ctx->packet_from_hv.header.conf_addr  = ctx->tx_addr;
    ctx->packet_from_hv.header.config.f32 = state.data[ctx->tx_addr++];
    ctx->tx_addr %= sizeof(state) / 4;

    extern struct ringbuf tx_buf;
    uint8_t buf[1];
    if(rb_read(&tx_buf, buf, 1)) {
      ctx->packet_from_hv.buf = buf[0];
    } else {
      ctx->packet_from_hv.buf = 0x0;
    }
    ctx->packet_from_hv.header.crc = crc_calc((uint32_t *)&(ctx->packet_from_hv.header.slave_addr), sizeof(packet_from_hv_t) / 4 - 1);

    // start tx DMA
    DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR_EN);
    DMA1_Channel2->CNDTR = sizeof(packet_from_hv_t);
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    //ctx->send = 0;
  }

  if(ctx->timeout > 5) {  //disable driver
    PIN(en)  = 0.0;
    PIN(vel) = 0.0;
    PIN(timeout)
    ++;
    fault = 1;
  }
  ctx->timeout++;

  PIN(fault) = MAX(fault, PIN(fault_in));


  // TODO: sin = 0.5
  switch((uint16_t)PIN(phase_mode)) {
    case PHASE_90_3PH:  // 90°
      PIN(pwm_volt) = PIN(dc_volt) * M_SQRT1_2 * 0.95;
      break;

    case PHASE_90_4PH:  // 90°
      PIN(pwm_volt) = PIN(dc_volt) * 0.95;
      break;

    case PHASE_120_3PH:  // 120°
      PIN(pwm_volt) = PIN(dc_volt) * M_SQRT1_3 * 0.95;
      break;

    case PHASE_180_2PH:  // 180°
    case PHASE_180_3PH:  // 180°
      PIN(pwm_volt) = PIN(dc_volt) * 0.95;
      break;

    default:
      PIN(pwm_volt) = 0.0;
  }
}

hal_comp_t ls_comp_struct = {
    .name      = "ls",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .hw_init   = hw_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ls_ctx_t),
    .pin_count = sizeof(struct ls_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
