#include "ls_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f3xx_hal.h"
#include "common.h"
#include "f3hw.h"
#include "ringbuf.h"

extern CRC_HandleTypeDef hcrc;

/**
* ## Brief
* `ls` is the F3 (HV board) end of the serial link to the F4 (3 Mbaud UART, USART3 with DMA, CRC checked packets). It receives the current or voltage command, rotor angle, velocity, enable, mode and braking flags, plus one word of configuration per packet, and puts them on pins for the current loop. It answers with the measured d/q currents, output voltages, the fault code and one word of state per packet. It also locks the F3's PWM period to the F4's packet timing, carries the F3 terminal (one byte each way per packet), holds the enable off until the whole configuration has arrived, disables the bridge when packets stop and can then ask `io0` for a short-circuit brake. It is loaded by `stm32f303/src/main.c` as `ls0` (rt_prio 0.6, the first in the chain). The F4 end of the link is the `hv` component.
*
* ## Component Explanation
*
* 1. **Wiring on the F3** (fixed in main.c):
* - Commands out: `d_cmd`/`q_cmd` to `curpid0.id_cmd/iq_cmd` (and to `hv0` for the dead time compensation), `pos` to `dq0.pos` and `emf0.pos`, `pos_v` to `idq0.pos`, `vel` to `curpid0.vel`, `en` to `curpid0.en`, `io0.hv_en` and `emf0.en`, `cmd_mode` to `curpid0` and `hv0`, `phase_mode` to `dq0`, `idq0` and `hv0`, `sbrake` to `io0.sbrake`.
* - Configuration out: `r`, `l` (as `ld`), `lq`, `psi`, `cur_bw`, `cur_ff`, `cur_ind`, `max_cur` and `pwm_volt` to `curpid0`, `max_cur`, `dac` and `ignore_fault_pin` to `io0`, `drop_k`, `drop_knee` and `arr` to `hv0`, `emf_run`/`emf_sel`/`emf_pp` to `emf0.run/sel/pp`, `fault` to `io0.led`.
* - Feedback in: `id_fb`/`iq_fb`/`y` from `dq0`, `ud_fb`/`uq_fb` from `curpid0`, `dc_volt` (`io0.udc`), `hv_temp`, `mot_temp`, `u_fb`/`v_fb`/`w_fb` and `fault_in` from `io0`, `emf_val` from `emf0.val`, `duty_max` from `hv0.duty_max`.
*
* 2. **Receiving** (rt):
* - The RX DMA writes each 32 byte `packet_to_hv_t` into the context. When all 32 bytes have arrived, the packet is accepted if `slave_addr` is 0, `len` matches and the CRC is right.
* - The header command is handled: `WRITE_CONF` stores the float into `f3_config_data_t` at `conf_addr`, `READ_CONF` sets the address of the next state word to send, `DO_RESET` resets the F3, `BOOTLOADER` sets a backup register flag and resets into the bootloader. Addresses past this image's config are ignored (the F4 may be newer than the F3 image; the layout is append only).
* - The process data goes to `phase_mode`, `cmd_mode`, `ignore_fault_pin`, `sbrake`, `sbrake_arm`, `d_cmd`, `q_cmd`, `pos` and `vel`, and all configuration words go to their pins (`r`, `l`, `psi`, `cur_bw`, `cur_ff`, `cur_ind`, `max_y`, `max_cur`, `dac`, `drop_k`, `lq`, `emf_run`, `emf_sel`, `emf_pp`, `drop_knee`). `lq` falls back to `l` when the F4 sent 0. A nonzero terminal byte is pushed into the F3's terminal RX buffer (read by `term0`).
* - `crc_ok` counts good packets, `crc_error` bad ones (this sets the communication fault to 3 for that tick).
* - The F4 sends at 5 kHz and the F3 rt runs at 15 kHz, so most ticks have no new packet. In those ticks `pos` is extrapolated with `pos += vel * period` (not wrapped), for up to 5 ticks.
*
* 3. **Enable only after a full configuration** (rt):
* - The F4 sends the 15 configuration words one per packet, so right after an F3 reset `r`, `l`, `max_cur`, `dac` etc. are still 0 for a while. `ls` keeps one bit per word written, and `conf_ok` becomes 1 once every word has been written at least once (the bits are cleared in rt_start). Until then `en` stays 0 whatever the F4 asks; after that `en` follows the F4's enable flag.
* - This assumes the F4 and F3 images come from the same build (same config layout): an older F4 that sends fewer words never gets `conf_ok`.
*
* 4. **Voltage angle** (rt):
* - `dq0` transforms the currents at `pos`, the angle at the current sample. The voltage computed from them is preloaded and applied over the next PWM period, so on average it lands 1.5 periods later. `ls` therefore outputs a second angle for `idq0` (and, through `idq0.si_out/co_out`, for `hv0`'s dead time reference):
* ```c
* pos_v = pos + vel * v_lead * period;
* ```
* - `v_lead` is in PWM periods, 1.5 by default (set in hw_init); 0 gives the old behaviour of one angle for both. `pos_v` is not wrapped (the sin/cos routine does not need it). With this, the F4's `hv0.adv` only has to cover the encoder to sample latency.
*
* 5. **Answering** (rt):
* - After each good packet, one `packet_from_hv_t` is sent by DMA: `fault_in`, `id_fb`, `iq_fb`, `ud_fb`, `uq_fb`, one byte from the terminal TX buffer, and one word of `f3_state_data_t` (in order `u_fb`, `v_fb`, `w_fb`, `hv_temp`, `mot_temp`, `core_temp`, `dc_volt`, `pwm_volt`, `y`, `emf_val`), cycling through the 10 words one per packet.
* - A UART receive timeout (16 bit times of idle line) resets the RX DMA so the next packet starts at byte 0. `idle` counts these, `dma_pos` records how many bytes a packet had if it was cut short.
*
* 6. **PWM phase lock** (rt):
* - `dma_pos2` is the number of bytes of the current packet already received when the rt tick runs. While a packet is being received (more than `window` bytes from either end), `arr` is set to `PWM_RES - inc` if more than `dma_pos_cmd` bytes have arrived and to `PWM_RES + inc` if fewer, otherwise `PWM_RES` (4800). `hv0` writes it to the PWM timer, so the F3's PWM and rt tick slide until they are in a fixed phase to the F4's packets.
* - rt_start sets `dma_pos_cmd` = 4, `inc` = 5, `window` = 1.
*
* 7. **Timeout and braking on link loss** (rt):
* - Without a good packet for more than 5 rt ticks (about 0.4 ms), `en` and `vel` are set to 0, `timeout` is counted up every tick and the communication fault is 1. The bridge is thus switched off by `io0` and `curpid0`. The internal tick counter saturates, so a long loss cannot wrap it and re-enable the bridge.
* - On the first timeout tick `ls` decides whether to brake: only if the F4's last packet had `sbrake_arm` set and the motor was driven (`en`) or already braking (`sbrake`). If so, `sbrake` is held at 1 for `sbrake_time` (1 s, set in rt_start) from the start of the timeout, and `io0` short-circuit brakes the motor on its own; otherwise `sbrake` is 0.
* - While packets arrive, `sbrake` simply follows the F4's flag (the F4 only sets it while `en` is 0).
*
* 8. **Faults and voltage limit** (rt):
* - `fault` = MAX(communication fault, `fault_in`). It only drives the LED blink count (`io0.led`). The F4 gets `fault_in` (the `io0` fault code) and detects CRC errors and timeouts on its own side.
* - `pwm_volt`, the largest voltage vector `curpid0` may output, depends on `phase_mode`: `dc_volt / sqrt(3) * duty` for 120 deg 3 phase, `dc_volt / sqrt(2) * duty` for 90 deg 3 phase, `dc_volt * duty` for 90 deg 4 phase and the 180 deg modes, else 0. `duty` is `duty_max` from `hv0` (0.91 with the default 3 us minimum on and off times), or 0.95 when `duty_max` is 0 (not wired).
*
* {{% hint warning %}}
* - The communication fault codes 1 (timeout) and 3 (CRC) are not `fault_t` codes (they would read as `CMD_ERROR` and `COM_FB_ERROR`). They are only used for the LED blink count and never reach the F4.
* - `max_y` is received but not used by anything on the F3, and `core_temp` is not wired, so it is always sent as 0.
* - A config word still takes effect as soon as it arrives, one per packet; `conf_ok` only guards the first enable after a reset, not later changes while enabled.
* - `sbrake_time` is reset to 1 s by rt_start and is not an F4 config word, so the F4's own braking time does not apply to a link loss.
* {{% /hint %}}
*/

HAL_COMP(ls);

//process data from LS
HAL_PIN(d_cmd);  // *output*, D-axis command from the F4, current (A) or voltage (V) depending on cmd_mode
HAL_PIN(q_cmd);  // *output*, Q-axis command from the F4, current (A) or voltage (V) depending on cmd_mode
HAL_PIN(pos);    // *output*, Electrical rotor angle at the current sample (rad), from the F4, extrapolated between packets
HAL_PIN(vel);    // *output*, Electrical velocity from the F4 (rad/s), 0 on timeout
// The angle the voltage computed this tick lands at, on average: the
// compares are preloaded, so it is applied over the next period, 1.5
// periods after the current sample that dq0 transforms at pos. idq0 and
// hv0's dead-time reference use it; hv0.adv on the f4 is then the encoder
// to sample latency alone. v_lead in periods, default 1.5, 0 = old
// behaviour (one angle for both).
HAL_PIN(pos_v);    // *output*, Voltage angle (rad), pos + vel * v_lead * period, not wrapped, to idq0.pos
HAL_PIN(conf_ok);  // *output*, 1 once every config word has been received since rt_start, en stays 0 before
HAL_PIN(v_lead);   // *parameter*, Voltage angle lead in PWM periods, default 1.5, 0 = same angle as pos
HAL_PIN(en);       // *output*, Enable from the F4, 0 until conf_ok and on timeout

// config data from LS
HAL_PIN(cmd_mode);    // *output*, Command mode from the F4, 0 = voltage, 1 = current
HAL_PIN(phase_mode);  // *output*, Phase mode from the F4, 0 = 90 deg 3ph, 1 = 90 deg 4ph, 2 = 120 deg 3ph, 3 = 180 deg 2ph, 4 = 180 deg 3ph
HAL_PIN(r);           // *output*, Motor resistance from the F4 config (Ohm)
HAL_PIN(l);           // *output*, Motor inductance, d-axis, from the F4 config (H)
HAL_PIN(lq);          // *output*, Q-axis inductance for curpid0.lq (H), config lq, or l when that is 0
HAL_PIN(psi);         // *output*, Magnet flux linkage from the F4 config (Vs/rad)
HAL_PIN(cur_bw);      // *output*, Current loop bandwidth from the F4 config (rad/s)
HAL_PIN(cur_ff);      // *output*, Resistance feed forward factor from the F4 config
HAL_PIN(cur_ind);     // *output*, Back EMF and cross coupling feed forward factor from the F4 config
HAL_PIN(max_y);       // *output*, From the F4 config, not used on the F3
HAL_PIN(max_cur);     // *output*, Maximum current from the F4 config (A), for curpid0 and io0
HAL_PIN(dac);         // *output*, Overcurrent comparator DAC value from the F4 config, for io0
HAL_PIN(drop_k);      // *output*, Dead time compensation factor from the F4 config, for hv0
HAL_PIN(emf_run);     // *output*, emf0 control from the F4 config, 1 = sum, 0 = hold, -1 = clear
HAL_PIN(emf_sel);     // *output*, emf0 result number to return in emf_val, from the F4 config
HAL_PIN(emf_pp);      // *output*, Pole pairs for emf0's per pole bins, from the F4 config
HAL_PIN(drop_knee);   // *output*, Dead time compensation curve knee (A) from the F4 config, 0 = latched sign, for hv0

// process data to LS
HAL_PIN(dc_volt);  // *input*, DC link voltage (V), from io0.udc, sent to the F4 and used for pwm_volt
HAL_PIN(id_fb);    // *input*, Measured d-axis current (A), from dq0.d, sent to the F4
HAL_PIN(iq_fb);    // *input*, Measured q-axis current (A), from dq0.q, sent to the F4
HAL_PIN(ud_fb);    // *input*, D-axis output voltage (V), from curpid0.ud, sent to the F4
HAL_PIN(uq_fb);    // *input*, Q-axis output voltage (V), from curpid0.uq, sent to the F4

// state data to LS
HAL_PIN(hv_temp);    // *input*, Power stage temperature (degC), from io0, sent to the F4
HAL_PIN(mot_temp);   // *input*, Motor temperature (degC), from io0, sent to the F4
HAL_PIN(core_temp);  // *input*, Core temperature, not wired, sent to the F4 as 0
HAL_PIN(fault_in);   // *input*, HV fault code from io0.fault, sent to the F4

// short-circuit braking request for io0: follows the f4's flag, and on a
// link loss brakes for sbrake_time if the f4 last sent sbrake_arm
HAL_PIN(sbrake);            // *output*, Short-circuit braking request for io0.sbrake, the F4's flag, or 1 for sbrake_time after a link loss
HAL_PIN(sbrake_arm);        // *output*, Flag from the F4, > 0 allows braking on a link loss
HAL_PIN(sbrake_time);       // *parameter*, Braking time after a link loss (s), set to 1 in rt_start
HAL_PIN(ignore_fault_pin);  // *output*, Ignore the driver fault pin, flag from the F4, for io0
HAL_PIN(y);                 // *input*, Zero sequence current, mean of the phase currents (A), from dq0.y, sent to the F4
HAL_PIN(u_fb);              // *input*, U phase voltage (V), from io0.u, sent to the F4
HAL_PIN(v_fb);              // *input*, V phase voltage (V), from io0.v, sent to the F4
HAL_PIN(w_fb);              // *input*, W phase voltage (V), from io0.w, sent to the F4
HAL_PIN(emf_val);           // *input*, emf0 result number emf_sel, from emf0.val, sent to the F4

// misc
HAL_PIN(pwm_volt);   // *output*, Maximum output voltage vector for curpid0 (V), from dc_volt, phase_mode and duty_max
HAL_PIN(duty_max);   // *input*, Usable duty from hv0.duty_max, 0 = not wired (0.95 is used)
HAL_PIN(crc_error);  // *output*, Counter of packets with a bad CRC, address or length
HAL_PIN(crc_ok);     // *output*, Counter of good packets
HAL_PIN(timeout);    // *output*, Counter of rt ticks spent in timeout
HAL_PIN(dma_pos);    // *output*, Bytes received of the last packet cut short by an idle line (debug)
HAL_PIN(idle);       // *output*, Counter of UART idle line events
HAL_PIN(fault);      // *output*, Max of the communication fault (1 = timeout, 3 = CRC) and fault_in, drives the LED

HAL_PIN(dma_pos2);     // *output*, Bytes of the current packet received at this rt tick (debug)
HAL_PIN(arr);          // *output*, PWM timer reload value for hv0.arr, PWM_RES +- inc for the phase lock
HAL_PIN(dma_pos_cmd);  // *parameter*, Phase lock target, bytes received at the rt tick, default 4
HAL_PIN(inc);          // *parameter*, Phase lock step on arr (timer ticks), default 5
HAL_PIN(window);       // *parameter*, Phase lock only acts more than this many bytes from either end of a packet, default 1

struct ls_ctx_t {
  uint32_t timeout;
  uint32_t sbrake_loss;  // brake for this link loss
  uint32_t tx_addr;
  uint32_t conf_seen;  // bit per config word written since boot
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
  struct ls_pin_ctx_t *pins = (struct ls_pin_ctx_t *)pin_ptr;

  GPIO_InitTypeDef GPIO_InitStruct;

  /* Peripheral clock enable */
  __HAL_RCC_USART3_CLK_ENABLE();

  UART_HandleTypeDef huart3;
  huart3.Instance                    = USART3;
  huart3.Init.BaudRate               = DATABAUD;
  huart3.Init.WordLength             = UART_WORDLENGTH_8B;
  huart3.Init.StopBits               = UART_STOPBITS_1;
  huart3.Init.Parity                 = UART_PARITY_NONE;
  huart3.Init.Mode                   = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling           = UART_OVERSAMPLING_8;
  huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR | USART_CR3_OVRDIS;
  HAL_UART_Init(&huart3);

  /*USART3 GPIO Configuration    
   PB10     ------> USART3_TX
   PB11     ------> USART3_RX 
   */
  GPIO_InitStruct.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  __HAL_RCC_DMA1_CLK_ENABLE();

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
  config.pins.drop_k  = 0.0;
  config.pins.lq      = 0.0;
  config.pins.emf_run = 0.0;
  config.pins.emf_sel = 0.0;
  config.pins.emf_pp  = 0.0;
  config.pins.drop_knee = 0.0;
  PIN(v_lead) = 1.5;

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
  ctx->sbrake_loss = 0;
  ctx->tx_addr     = 0;
  ctx->conf_seen   = 0;
  ctx->send        = 0;
  PIN(crc_error)   = 0.0;
  PIN(crc_ok)      = 0.0;
  PIN(timeout)     = 0.0;
  PIN(sbrake)      = 0.0;
  PIN(sbrake_arm)  = 0.0;
  PIN(sbrake_time) = 1.0;
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
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&(ctx->packet_to_hv.header.slave_addr), sizeof(packet_to_hv_t) / 4 - 1);
    if(ctx->packet_to_hv.header.slave_addr == 0 && ctx->packet_to_hv.header.len == (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4 && crc == ctx->packet_to_hv.header.crc) {
      //
      // an address past this image's config is a newer f4's word: ignore it
      uint8_t a     = ctx->packet_to_hv.header.conf_addr;
      uint8_t valid = a < sizeof(config) / 4;
      a             = CLAMP(a, 0, sizeof(config) / 4 - 1);

      switch(ctx->packet_to_hv.header.flags.cmd) {
        case NO_CMD:
          break;
        case WRITE_CONF:
          if(valid) {
            config.data[a] = ctx->packet_to_hv.header.config.f32;
            ctx->conf_seen |= 1u << a;
          }
          break;
        case READ_CONF:
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

      // no enable before every config word has arrived once since boot: the
      // f4 cycles through them one per packet, and until then r, l, max_cur,
      // dac etc. are still 0
      PIN(conf_ok) = ctx->conf_seen == (1u << (sizeof(config) / 4)) - 1u;
      PIN(en)      = PIN(conf_ok) > 0.0 ? ctx->packet_to_hv.flags.enable : 0;
      PIN(phase_mode)       = ctx->packet_to_hv.flags.phase_type;
      PIN(cmd_mode)         = ctx->packet_to_hv.flags.cmd_type;
      PIN(ignore_fault_pin) = ctx->packet_to_hv.flags.ignore_fault_pin;
      PIN(sbrake)           = ctx->packet_to_hv.flags.sbrake;
      PIN(sbrake_arm)       = ctx->packet_to_hv.flags.sbrake_arm;
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
      PIN(drop_k)  = config.pins.drop_k;
      PIN(lq)      = config.pins.lq > 0.0 ? config.pins.lq : config.pins.l;
      PIN(emf_run) = config.pins.emf_run;
      PIN(emf_sel) = config.pins.emf_sel;
      PIN(emf_pp)  = config.pins.emf_pp;
      PIN(drop_knee) = config.pins.drop_knee;
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
  PIN(pos_v) = PIN(pos) + PIN(vel) * PIN(v_lead) * period;  // sincos_fast wraps


  if(USART3->ISR & USART_ISR_RTOF) {                                    // idle line
    USART3->ICR |= USART_ICR_RTOCF | USART_ICR_FECF | USART_ICR_ORECF;  // timeout clear flag
    GPIOA->BSRR |= GPIO_PIN_10;

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
    GPIOA->BSRR |= GPIO_PIN_10 << 16;

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
    state.pins.emf_val   = PIN(emf_val);

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
    ctx->packet_from_hv.header.crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&(ctx->packet_from_hv.header.slave_addr), sizeof(packet_from_hv_t) / 4 - 1);

    // start tx DMA
    DMA1_Channel2->CCR &= (uint16_t)(~DMA_CCR_EN);
    DMA1_Channel2->CNDTR = sizeof(packet_from_hv_t);
    DMA1_Channel2->CCR |= DMA_CCR_EN;
    //ctx->send = 0;
  }

  if(ctx->timeout == 6) {
    // brake on the loss only if the motor was driven or already braking
    ctx->sbrake_loss = PIN(sbrake_arm) > 0.0 && (PIN(en) > 0.0 || PIN(sbrake) > 0.0);
  }
  if(ctx->timeout > 5) {  //disable driver
    PIN(en)     = 0.0;
    PIN(vel)    = 0.0;
    PIN(sbrake) = ctx->sbrake_loss && (float)(ctx->timeout - 5) * period < PIN(sbrake_time);
    PIN(timeout)
    ++;
    fault = 1;
  }
  if(ctx->timeout < 0x7FFFFFFF) {  // saturate: a wrap would re-enable for a few ticks
    ctx->timeout++;
  }

  PIN(fault) = MAX(fault, PIN(fault_in));


  // The ceiling curpid may ask for. hv.c reserves min_on at one end of the
  // period and min_off at the other, so the usable link is duty_max of it (0.91
  // at the 3 us defaults, where the old fixed 0.95 promised 4% more than the
  // clamp would pass, and the loop wound up against the clamp instead of its
  // own limit). Falls back to 0.95 when the pin is not wired.
  float duty = PIN(duty_max) > 0.0 ? PIN(duty_max) : 0.95;

  // TODO: sin = 0.5
  switch((uint16_t)PIN(phase_mode)) {
    case PHASE_90_3PH:  // 90°
      PIN(pwm_volt) = PIN(dc_volt) * M_SQRT1_2 * duty;
      break;

    case PHASE_90_4PH:  // 90°
      PIN(pwm_volt) = PIN(dc_volt) * duty;
      break;

    case PHASE_120_3PH:  // 120°
      PIN(pwm_volt) = PIN(dc_volt) * M_SQRT1_3 * duty;
      break;

    case PHASE_180_2PH:  // 180°
    case PHASE_180_3PH:  // 180°
      PIN(pwm_volt) = PIN(dc_volt) * duty;
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
