#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "dma_util.h"
#include "hw/hw.h"
#include "common.h"
#include "main.h"
#include "ringbuf.h"

/**
* ## Brief
* `hv` runs on the F4 logic board and is its end of the UART link to the F3 power (HV) board. Every rt period it sends the current/voltage command, the commutation angle and velocity, the short brake flags and one word of current loop configuration to the F3, and unpacks the F3 reply (dq feedback, fault code and one word of state such as DC link voltage and temperatures). It also carries the `hv` terminal pass-through and the `hv_update` F3 firmware update. The F3 side of the link is the F3 `ls` component (stm32f303/src/comps/ls.c); the F3 bridge driver that shares this name is stm32f303/src/comps/hv.c, documented as page hv_f3.
*
* Typical wiring (conf/template/pid.txt + pmsm.txt): `hv0.en = fault0.en_out`, `hv0.sbrake = fault0.sbrake`, `hv0.sbrake_arm = fault0.sbrake_en`, `hv0.pos = vel2.pos_out`, `hv0.vel = vel2.vel`, `hv0.q_cmd = pmsm_ttc0.cur`, `hv0.scale = fault0.scale`, the motor constants from `conf0`, and `fault0.hv_error = hv0.fault`, `fault0.dc_volt = hv0.dc_volt`, `fault0.dc_cur = hv0.dc_cur`, `iit0.cur = hv0.abs_cur`. conf/template/id_pmsm.txt links the `emf_*` pins to `idpmsm0`, conf/template/id_tune.txt drives `drop_k` from `idtune0`.
*
* ## Component Explanation
*
* 1. **Link and packets**:
* - USART `UART_DRV` (USART2) at `DATABAUD` = 3 Mbaud, 8N1, TX and RX both by DMA; packets use the stmbl_talk header with an STM32 hardware CRC32 over everything after the CRC field.
* - F4 to F3 (`packet_to_hv_t`, 32 bytes): `d_cmd`, `q_cmd`, `pos`, `vel`, the flags `enable`, `cmd_type` (from `cmd_mode`), `phase_type` (from `phase_mode`), `ignore_fault_pin`, `sbrake`, `sbrake_arm`, one terminal byte, plus one config word (`conf_addr` + float).
* - F3 to F4 (`packet_from_hv_t`, 32 bytes): `id_fb`, `iq_fb`, `ud_fb`, `uq_fb`, `fault`, one terminal byte, plus one state word.
* - The exchange is one request/answer per rt period: the answer to the packet sent in period n is read at the start of period n+1.
*
* 2. **Commands sent (rt)**:
* - `pos` is advanced by `vel * adv` and wrapped with `mod()` before sending: `pos_sent = mod(pos + vel * adv)`. `vel` is sent too; the F3 uses it to extrapolate `pos` between packets and for the BEMF decoupling of its current loop.
* - If `en > 0`, `d_cmd`/`q_cmd` are sent and the enable flag is set; otherwise both are sent as 0 and the F3 is disabled.
* - `cmd_mode`: 0 = voltage mode (`d_cmd`/`q_cmd` in V), 1 = current mode (`d_cmd`/`q_cmd` in A peak, closed by the F3 current loop).
* - `phase_mode`: 0 = 90 deg 3 phase, 1 = 90 deg 4 phase, 2 = 120 deg 3 phase (normal 3 phase motor), 3 = 180 deg 2 phase (DC motor), 4 = 180 deg 3 phase.
* - `rev > 0` negates `q_cmd`, `pos` and `vel` on the way out and `iq_fb`, `uq_fb` on the way in, to reverse the output direction.
*
* 3. **Short-circuit braking flags**:
* - The `sbrake` flag is sent only while `en <= 0` and `sbrake > 0`: the F3 (io.c) then turns all three low sides on and chops the phase current at its `io0.sbrake_cur` (default `max_cur`, at most 0.8 x the overcurrent limit). It refuses after any trip since the last enable and stops for good on a trip while braking; that trip is reported in `fault` until the request goes away.
* - `sbrake_arm > 0` lets the F3 brake on its own (for its `ls0.sbrake_time`, default 1 s) when packets from the F4 stop while the motor was driven or already braking.
* - Both are wired from `fault0` (see the fault page for when it asks for braking). During an F3 update both flags are sent as 0.
*
* 4. **Config words (round robin)**:
* - The F3 current loop parameters are sent one word per packet, cycling through `r`, `l`, `psi`, `cur_bw`, `cur_ff`, `cur_ind`, `max_y`, `max_cur * scale`, `dac`, `drop_k`, `lq`, `emf_run`, `emf_sel`, `emf_pp`, `drop_knee` (the layout of `f3_config_data_t` in shared/common.h, append only). A full update takes 15 rt periods.
* - `max_cur` is multiplied by `scale` (the fault component's derating, 0..1) before sending; on the F3 it is the current loop command limit and sets the software overcurrent trip.
* - `dac` is the raw 12 bit value (0..4095) of the F3 DAC that sets the hardware overcurrent comparator threshold.
* - `lq = 0` makes the F3 use `l` for both axes.
* - Dead time compensation (F3 hv, current mode and 120 deg 3 phase only): `drop_k` scales the ideal dead time loss (dead time / pwm period x DC link voltage), 0 = off; the sign per phase comes from the commanded phase currents. With `drop_knee = 0` the sign latches outside a current band; `drop_knee > 0` (A) instead scales each phase by `k(i) = 1 - 1/(1 + |i|/drop_knee)^2`, which fades out at low current and cannot stick.
* - `emf_run` (1 sum, 0 hold, -1 clear), `emf_sel` and `emf_pp` (pole pairs) drive the F3 `emf0` back EMF map, which sums the unfiltered phase voltages while the bridge is off and the rotor coasts; the result selected by `emf_sel` comes back in `emf_val`. Used by `idpmsm0`.
*
* 5. **Feedback (rt)**:
* - A received packet is only used if it is complete, its CRC matches, `slave_addr` is 0 and its length is that of `packet_from_hv_t`. Then `id_fb`, `iq_fb`, `ud_fb`, `uq_fb`, `fault` are copied, `abs_cur = sqrt(id_fb^2 + iq_fb^2)`, `abs_volt = sqrt(ud_fb^2 + uq_fb^2)` and `duty = abs_volt / pwm_volt` (only while `pwm_volt > 0`).
* - One state word per packet fills `u_fb`, `v_fb`, `w_fb`, `hv_temp`, `mot_temp`, `core_temp`, `dc_volt`, `pwm_volt`, `y`, `emf_val` (10 words, so each is refreshed every 10 periods).
* - `power` and `dc_cur` are not measured, they are estimated from the commanded dq voltages, so they leave out inverter losses and read low:
* ```c
* power  = 0.5 * power + 0.5 * 1.5 * (ud_fb * id_fb + uq_fb * iq_fb);
* dc_cur = power / dc_volt;   // only updated while dc_volt > 1 V
* ```
* - Every failed CRC increments `crc_error` (never reset); it does not raise a fault.
*
* 6. **Timeout**:
* - A counter is reset by every valid packet; after more than 2 rt periods without one, `fault` is forced to `HV_TIMEOUT_ERROR` (9). All other codes in `fault` come from the F3 (`fault_t` in shared/common.h, e.g. `HV_TEMP_ERROR` 10, `HV_VOLT_ERROR` 11, `HV_FAULT_ERROR` 12, `HV_OVERCURRENT_*` 14..16).
*
* 7. **Terminal pass-through**:
* - The terminal command `hv <text>` queues the text plus newline into a 128 byte ring buffer; one byte is sent per packet to the F3 terminal. Bytes coming back from the F3 are printed by `nrt`.
*
* 8. **F3 firmware update (`hv_update`)**:
* - The F4 image contains the F3 application (obj_hvf3/hvf3.bin). The terminal command `hv_update` starts a state machine in `rt`, reported on `state`:
* - 0 `SLAVE_IN_APP`: normal operation.
* - 1 `SEND_TO_BOOTLOADER`: sends the BOOTLOADER command with the output disabled for 10 periods, the F3 resets into its bootloader.
* - 2 `ERASE_FLASH`: page erase request to slave address 255, waits for an OK (gives up after 20000 periods).
* - 3 `SEND_APP`: writes the image word by word to 0x08004000, each word must be acknowledged with the same address and value (10 period timeout per word). `nrt` prints the progress in percent.
* - 4 `CRC_CHECK`: asks the bootloader to verify the app CRC (2000 period timeout).
* - 5 `SEND_TO_APP`: sends DO_RESET for 2000 periods, then back to 0.
* - 6 `FLASH_FAILED`: after 10 periods back to 0. The F3 then usually still sits in its bootloader, so `fault` shows the timeout until the update is retried or the drive is power cycled.
* - While updating, every third rt period sends nothing, to give the bootloader time to answer.
*
* {{% hint info %}}
* `flash_state` and the terminal buffers are global, so only one `hv` instance works. `max_y` is transmitted but not used by the current F3 firmware. `value`, `uart_sr` and `uart_dr` are debug outputs.
* {{% /hint %}}
*/

HAL_COMP(hv);

//process data from LS
HAL_PIN(d_cmd);             // *input*, d axis command: current (A peak) in current mode, voltage (V) in voltage mode
HAL_PIN(q_cmd);             // *input*, q axis command: current (A peak) in current mode, voltage (V) in voltage mode
HAL_PIN(pos);               // *input*, commutation angle (rad)
HAL_PIN(vel);               // *input*, velocity of pos (rad/s), sent to the F3 and used for the advance
HAL_PIN(adv);               // *parameter*, commutation advance (s), pos is sent as pos + vel * adv, default 0
HAL_PIN(en);                // *input*, enable the F3 power stage (> 0), commands are sent as 0 while disabled

// config data from LS
HAL_PIN(phase_mode);        // *parameter*, 0 = 90 deg 3ph, 1 = 90 deg 4ph, 2 = 120 deg 3ph, 3 = 180 deg 2ph (DC), 4 = 180 deg 3ph
HAL_PIN(cmd_mode);          // *parameter*, 0 = voltage mode, 1 = current mode
HAL_PIN(r);                 // *parameter*, phase resistance (Ohm), for the F3 current loop
HAL_PIN(l);                 // *parameter*, d axis inductance (H), and q too while lq is 0
HAL_PIN(lq);                // *parameter*, q axis inductance (H), 0 = same as l, default 0
HAL_PIN(psi);               // *parameter*, flux linkage / torque constant (V*s/rad), for BEMF decoupling
HAL_PIN(cur_bw);            // *parameter*, current loop bandwidth (rad/s)
HAL_PIN(cur_ff);            // *parameter*, current loop resistance feed forward gain
HAL_PIN(cur_ind);           // *parameter*, current loop BEMF / cross coupling feed forward gain
HAL_PIN(max_y);             // *parameter*, sent to the F3 but unused there
HAL_PIN(max_cur);           // *parameter*, max phase current (A peak), sent multiplied by scale
HAL_PIN(dac);               // *parameter*, F3 overcurrent comparator DAC value (0..4095), default 2500
HAL_PIN(drop_k);            // *parameter*, dead time compensation, fraction of the ideal loss, 0 = off, default 0
HAL_PIN(drop_knee);         // *parameter*, dead time compensation curve knee (A peak), 0 = latched sign
HAL_PIN(emf_run);           // *input*, F3 emf0 back EMF map: 1 sum, 0 hold, -1 clear
HAL_PIN(emf_sel);           // *input*, which emf0 result comes back in emf_val
HAL_PIN(emf_pp);            // *parameter*, pole pairs, for emf0's per pole bins

// process data to LS
HAL_PIN(dc_volt);           // *output*, DC link voltage (V)
HAL_PIN(id_fb);             // *output*, measured d axis current (A peak)
HAL_PIN(iq_fb);             // *output*, measured q axis current (A peak), negated when rev > 0
HAL_PIN(ud_fb);             // *output*, d axis output voltage (V)
HAL_PIN(uq_fb);             // *output*, q axis output voltage (V), negated when rev > 0
HAL_PIN(abs_cur);           // *output*, current magnitude sqrt(id^2 + iq^2) (A peak)
HAL_PIN(abs_volt);          // *output*, voltage magnitude sqrt(ud^2 + uq^2) (V)
HAL_PIN(duty);              // *output*, abs_volt / pwm_volt
HAL_PIN(power);             // *output*, electrical power into the motor (W), estimated, negative when braking
HAL_PIN(dc_cur);            // *output*, DC link current (A), estimated from power balance, negative when braking

// state data to LS
HAL_PIN(hv_temp);           // *output*, power stage temperature (deg C)
HAL_PIN(mot_temp);          // *output*, motor temperature input of the F3 board
HAL_PIN(core_temp);         // *output*, F3 chip temperature (deg C)
HAL_PIN(fault);             // *output*, fault code from the F3, or HV_TIMEOUT_ERROR (9) when the link is lost
HAL_PIN(ignore_fault_pin);  // *parameter*, > 0 makes the F3 ignore the gate driver fault pin, default 1
HAL_PIN(sbrake);            // *input*, short-circuit braking request, sent only while en <= 0, usually fault0.sbrake
HAL_PIN(sbrake_arm);        // *input*, > 0 lets the F3 brake on its own on link loss, usually fault0.sbrake_en
HAL_PIN(y);                 // *output*, zero sequence component of the phase currents (A)
HAL_PIN(u_fb);              // *output*, phase U voltage (V)
HAL_PIN(v_fb);              // *output*, phase V voltage (V)
HAL_PIN(w_fb);              // *output*, phase W voltage (V)
HAL_PIN(emf_val);           // *output*, emf0 result number emf_sel, from the F3

// misc
HAL_PIN(rev);               // *parameter*, > 0 reverses the output direction (q_cmd, pos, vel, iq_fb, uq_fb)
HAL_PIN(pwm_volt);          // *output*, max output voltage the F3 can produce (V)
HAL_PIN(uart_sr);           // *output*, debug, USART status register, read to clear errors
HAL_PIN(uart_dr);           // *output*, debug, USART data register, read to clear errors
HAL_PIN(crc_error);         // *output*, total number of crc errors, never reset
HAL_PIN(scale);             // *input*, current limit scale (0..1), usually fault0.scale

HAL_PIN(state);             // *output*, F3 update state, 0 = normal operation, 1..6 see above
HAL_PIN(value);             // *output*, debug, 0 = no packet, 0.75 = packet not accepted, 1 = valid packet, 3 = bad bootloader reply

struct hv_ctx_t {
  union {
    volatile packet_to_hv_t packet_to_hv;
    volatile packet_bootloader_t packet_to_hv_bootloader;
  } to_hv;
  union {
    volatile packet_from_hv_t packet_from_hv;
    volatile packet_bootloader_t packet_from_hv_bootloader;
  } from_hv;
  f3_config_data_t config;
  f3_state_data_t state;
  uint32_t addr;
  uint16_t timeout;
  uint8_t conf_addr;
  uint8_t send_state;
};

typedef enum {
  SLAVE_IN_APP,
  SEND_TO_BOOTLOADER,
  ERASE_FLASH,
  SEND_APP,
  CRC_CHECK,
  SEND_TO_APP,
  FLASH_FAILED,
} flash_state_t;

flash_state_t flash_state;

uint32_t send_to_bootloader;

extern uint8_t _binary_obj_hvf3_hvf3_bin_start;
extern uint8_t _binary_obj_hvf3_hvf3_bin_size;
extern uint8_t _binary_obj_hvf3_hvf3_bin_end;

struct ringbuf hv_rx_buf = RINGBUF(128);
struct ringbuf hv_tx_buf = RINGBUF(128);

void hv_send(char *ptr) {
  if(ptr) {  //TODO: check connection status
    rb_write(&hv_tx_buf, ptr, strlen(ptr));
    rb_write(&hv_tx_buf, "\n", 1);
  }
}
COMMAND("hv", hv_send, "send command to hv board");

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  //setup uart to f1. uses DMA to transfer to_hv struct.
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
  DMA_InitTypeDef DMA_InitStructure;

  UART_DRV_CLOCK_COMMAND(UART_DRV_RCC, ENABLE);

  //USART TX
  GPIO_PinAFConfig(UART_DRV_TX_PORT, UART_DRV_TX_PIN_SOURCE, UART_DRV_TX_AF_SOURCE);
  GPIO_InitStruct.GPIO_Pin   = UART_DRV_TX_PIN;
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(UART_DRV_TX_PORT, &GPIO_InitStruct);

  //USART RX
  GPIO_PinAFConfig(UART_DRV_RX_PORT, UART_DRV_RX_PIN_SOURCE, UART_DRV_RX_AF_SOURCE);
  GPIO_InitStruct.GPIO_Pin = UART_DRV_RX_PIN;
  GPIO_Init(UART_DRV_RX_PORT, &GPIO_InitStruct);

  USART_OverSampling8Cmd(UART_DRV, ENABLE);
  USART_InitStruct.USART_BaudRate            = DATABAUD;
  USART_InitStruct.USART_WordLength          = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits            = USART_StopBits_1;
  USART_InitStruct.USART_Parity              = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART_DRV, &USART_InitStruct);

  /* Enable the USART */
  USART_Cmd(UART_DRV, ENABLE);

  // DMA-Disable
  dma_stream_stop(UART_DRV_TX_DMA);
  DMA_DeInit(UART_DRV_TX_DMA);

  // DMA2-Config
  DMA_InitStructure.DMA_Channel            = UART_DRV_TX_DMA_CHAN;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART_DRV->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) & (ctx->to_hv.packet_to_hv);
  DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize         = MAX(sizeof(packet_to_hv_t), sizeof(packet_bootloader_t));
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(UART_DRV_TX_DMA, &DMA_InitStructure);

  //DMA_Cmd(UART_DRV_TX_DMA, ENABLE);

  USART_DMACmd(UART_DRV, USART_DMAReq_Tx, ENABLE);


  // DMA-Disable
  dma_stream_stop(UART_DRV_RX_DMA);
  DMA_DeInit(UART_DRV_RX_DMA);

  // DMA2-Config
  DMA_InitStructure.DMA_Channel            = UART_DRV_RX_DMA_CHAN;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART_DRV->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t) & (ctx->from_hv.packet_from_hv);
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize         = MAX(sizeof(packet_from_hv_t), sizeof(packet_bootloader_t));
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
  DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(UART_DRV_RX_DMA, &DMA_InitStructure);


  USART_DMACmd(UART_DRV, USART_DMAReq_Rx, ENABLE);
  dma_stream_stop(UART_DRV_RX_DMA);
  DMA_Cmd(UART_DRV_RX_DMA, ENABLE);

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
  ctx->timeout          = 0;
  PIN(dac)              = 2500;
  PIN(drop_k)           = 0;
  PIN(lq)               = 0;
  PIN(adv)              = 0;
  send_to_bootloader    = 0;
  flash_state           = SLAVE_IN_APP;
  ctx->send_state       = 0;
  PIN(ignore_fault_pin) = 1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;
  float e                   = PIN(en);
  float pos                 = PIN(pos);
  float vel                 = PIN(vel);
  pos                       = mod(pos + vel * PIN(adv));

  ctx->config.pins.r       = PIN(r);
  ctx->config.pins.l       = PIN(l);
  ctx->config.pins.psi     = PIN(psi);
  ctx->config.pins.cur_bw  = PIN(cur_bw);
  ctx->config.pins.cur_ff  = PIN(cur_ff);
  ctx->config.pins.cur_ind = PIN(cur_ind);
  ctx->config.pins.max_y   = PIN(max_y);
  ctx->config.pins.max_cur = PIN(max_cur) * PIN(scale);
  ctx->config.pins.dac     = PIN(dac);
  ctx->config.pins.drop_k  = PIN(drop_k);
  ctx->config.pins.lq      = PIN(lq);
  ctx->config.pins.emf_run = PIN(emf_run);
  ctx->config.pins.emf_sel = PIN(emf_sel);
  ctx->config.pins.emf_pp  = PIN(emf_pp);
  ctx->config.pins.drop_knee = PIN(drop_knee);

  uint32_t dma_count = MAX(sizeof(packet_from_hv_t), sizeof(packet_bootloader_t)) - DMA_GetCurrDataCounter(UART_DRV_RX_DMA);

  PIN(value) = 0.0;

  if(dma_count >= sizeof(stmbl_talk_header_t)) {
    // PIN(value) = 0.5;
    if(dma_count >= sizeof(stmbl_talk_header_t) + ctx->from_hv.packet_from_hv.header.len * 4) {
      PIN(value) = 0.75;

      CRC_ResetDR();
      uint32_t crc = CRC_CalcBlockCRC((uint32_t *)&(ctx->from_hv.packet_from_hv.header.slave_addr), sizeof(stmbl_talk_header_t) / 4 + ctx->from_hv.packet_from_hv.header.len - 1);
      if(ctx->from_hv.packet_from_hv.header.crc == crc) {
        switch(flash_state) {
          case SLAVE_IN_APP:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 0 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_from_hv_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 app
              PIN(id_fb) = ctx->from_hv.packet_from_hv.id_fb;
              PIN(iq_fb) = ctx->from_hv.packet_from_hv.iq_fb;
              PIN(ud_fb) = ctx->from_hv.packet_from_hv.ud_fb;
              PIN(uq_fb) = ctx->from_hv.packet_from_hv.uq_fb;
              if(PIN(rev) > 0.0) {
                PIN(uq_fb) *= -1.0;
                PIN(iq_fb) *= -1.0;
              }
              PIN(fault)    = ctx->from_hv.packet_from_hv.fault;
              PIN(abs_cur)  = sqrtf(PIN(id_fb) * PIN(id_fb) + PIN(iq_fb) * PIN(iq_fb));
              PIN(abs_volt) = sqrtf(PIN(ud_fb) * PIN(ud_fb) + PIN(uq_fb) * PIN(uq_fb));
              if(PIN(pwm_volt) > 0.0) {
                PIN(duty) = PIN(abs_volt) / PIN(pwm_volt);
              }

              uint16_t a         = ctx->from_hv.packet_from_hv.header.conf_addr;
              a                  = CLAMP(a, 0, sizeof(f3_state_data_t) / 4 - 1);
              ctx->state.data[a] = ctx->from_hv.packet_from_hv.header.config.f32;

              PIN(dc_volt)   = ctx->state.pins.dc_volt;
              PIN(pwm_volt)  = ctx->state.pins.pwm_volt;
              PIN(u_fb)      = ctx->state.pins.u_fb;
              PIN(v_fb)      = ctx->state.pins.v_fb;
              PIN(w_fb)      = ctx->state.pins.w_fb;
              PIN(hv_temp)   = ctx->state.pins.hv_temp;
              PIN(mot_temp)  = ctx->state.pins.mot_temp;
              PIN(core_temp) = ctx->state.pins.core_temp;
              PIN(y)         = ctx->state.pins.y;
              PIN(emf_val)   = ctx->state.pins.emf_val;

              // not measured: P = 3/2 (ud id + uq iq) from the commanded
              // voltages, so inverter losses are left out
              PIN(power) = 1.5 * (PIN(ud_fb) * PIN(id_fb) + PIN(uq_fb) * PIN(iq_fb)) * 0.5 + PIN(power) * 0.5;
              if(PIN(dc_volt) > 1.0) {
                PIN(dc_cur) = PIN(power) / PIN(dc_volt);
              }

              PIN(value) = 1.0;

              ctx->timeout = 0;

              if(ctx->from_hv.packet_from_hv.buf != 0x0) {
                rb_write(&hv_rx_buf, (void *)&(ctx->from_hv.packet_from_hv.buf), 1);
              }
            } else {
              // wrong packet len or slave addr
            }
            break;
          case SEND_TO_BOOTLOADER:

            break;
          case ERASE_FLASH:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 255 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 bootloader
              if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_OK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_PAGEERASE) {
                ctx->timeout = 0;
                flash_state  = SEND_APP;
              }
            } else {
              // wrong packet len or slave addr
              PIN(value) = 3.0;
            }
            break;
          case SEND_APP:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 255 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 bootloader
              if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_OK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_WRITE && ctx->from_hv.packet_from_hv_bootloader.addr == 0x08004000 + ctx->addr * 4 && ctx->from_hv.packet_from_hv_bootloader.value == ((uint32_t *)&(_binary_obj_hvf3_hvf3_bin_start))[ctx->addr]) {
                ctx->timeout = 0;
                ctx->addr++;
              }
              if(ctx->addr > ((uint32_t) & (_binary_obj_hvf3_hvf3_bin_size)) / 4) {
                flash_state = CRC_CHECK;
                // flash_state = SEND_TO_APP;
              }
            } else {
              // wrong packet len or slave addr
              PIN(value) = 3.0;
            }
            break;
          case CRC_CHECK:
            if(ctx->from_hv.packet_from_hv.header.slave_addr == 255 && ctx->from_hv.packet_from_hv.header.len == (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4) {
              // from f3 bootloader
              if(ctx->from_hv.packet_from_hv_bootloader.state == BOOTLOADER_STATE_OK && ctx->from_hv.packet_from_hv_bootloader.cmd == BOOTLOADER_OPCODE_CRCCHECK) {
                ctx->timeout = 0;
                flash_state  = SEND_TO_APP;
              }
            } else {
              // wrong packet len or slave addr
              PIN(value) = 3.0;
            }
            break;
          case SEND_TO_APP:

            break;
          case FLASH_FAILED:

            break;
        }


      } else {
        // CRC fault
        PIN(crc_error)
        ++;
        // PIN(fault) = HV_CRC_ERROR;
        // PIN(value) = 4.0;
      }
    }
  }

  if(ctx->timeout > 2) {
    PIN(fault) = HV_TIMEOUT_ERROR;
  }
  ctx->timeout++;

  float d_cmd = PIN(d_cmd);
  float q_cmd = PIN(q_cmd);

  if(PIN(rev) > 0.0) {
    q_cmd *= -1.0;
    pos = minus(0, pos);
    vel *= -1.0;  // the f3 extrapolates pos and decouples with vel
  }


  uint32_t tx_size = 0;

  switch(flash_state) {
    case SLAVE_IN_APP:
      if(e > 0.0) {
        ctx->to_hv.packet_to_hv.d_cmd        = d_cmd;
        ctx->to_hv.packet_to_hv.q_cmd        = q_cmd;
        ctx->to_hv.packet_to_hv.flags.enable = 1;
      } else {
        ctx->to_hv.packet_to_hv.d_cmd        = 0.0;
        ctx->to_hv.packet_to_hv.q_cmd        = 0.0;
        ctx->to_hv.packet_to_hv.flags.enable = 0;
      }
      ctx->to_hv.packet_to_hv.flags.ignore_fault_pin = PIN(ignore_fault_pin) > 0.0;
      ctx->to_hv.packet_to_hv.flags.sbrake           = e <= 0.0 && PIN(sbrake) > 0.0;
      ctx->to_hv.packet_to_hv.flags.sbrake_arm       = PIN(sbrake_arm) > 0.0;
      ctx->to_hv.packet_to_hv.flags.cmd_type         = PIN(cmd_mode);
      ctx->to_hv.packet_to_hv.flags.phase_type       = PIN(phase_mode);
      ctx->to_hv.packet_to_hv.pos                    = pos;
      ctx->to_hv.packet_to_hv.vel                    = vel;

      ctx->to_hv.packet_to_hv.header.slave_addr = 0;
      ctx->to_hv.packet_to_hv.header.flags.cmd  = WRITE_CONF;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_to_hv_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv.header.conf_addr  = ctx->conf_addr;
      ctx->to_hv.packet_to_hv.header.config.f32 = ctx->config.data[ctx->conf_addr++];

      uint8_t buf[1];
      if(rb_read(&hv_tx_buf, buf, 1)) {
        ctx->to_hv.packet_to_hv.flags.buf = buf[0];
      } else {
        ctx->to_hv.packet_to_hv.flags.buf = 0x0;
      }

      tx_size = sizeof(packet_to_hv_t);

      ctx->conf_addr %= sizeof(f3_config_data_t) / 4;

      if(send_to_bootloader) {
        send_to_bootloader = 0;
        flash_state        = SEND_TO_BOOTLOADER;
        ctx->timeout       = 0;
        // TODO: check f3 crc, size, ...
      }
      break;

    case SEND_TO_BOOTLOADER:  // fix
      ctx->to_hv.packet_to_hv.header.flags.cmd = BOOTLOADER;
      ctx->to_hv.packet_to_hv.flags.buf        = 0x0;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.d_cmd        = 0.0;
      ctx->to_hv.packet_to_hv.q_cmd        = 0.0;
      ctx->to_hv.packet_to_hv.flags.enable = 0;
      ctx->to_hv.packet_to_hv.flags.sbrake     = 0;
      ctx->to_hv.packet_to_hv.flags.sbrake_arm = 0;

      tx_size = sizeof(packet_to_hv_t);

      if(ctx->timeout > 10) {
        ctx->timeout = 0;
        flash_state  = ERASE_FLASH;
      }
      break;

    case ERASE_FLASH:
      ctx->to_hv.packet_to_hv.header.slave_addr = 255;
      ctx->to_hv.packet_to_hv.header.flags.cmd  = NO_CMD;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len        = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv.header.conf_addr  = 0;
      ctx->to_hv.packet_to_hv.header.config.f32 = 0;
      ctx->to_hv.packet_to_hv_bootloader.addr   = 0;
      ctx->to_hv.packet_to_hv_bootloader.value  = 0;
      ctx->to_hv.packet_to_hv_bootloader.cmd    = BOOTLOADER_OPCODE_PAGEERASE;

      tx_size = sizeof(packet_bootloader_t);

      ctx->addr = 0;

      // flash_state = SLAVE_IN_APP;

      if(ctx->timeout > 20000) {
        ctx->timeout = 0;
        flash_state  = FLASH_FAILED;
      }
      break;

    case SEND_APP:
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len       = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv_bootloader.addr  = 0x08004000 + ctx->addr * 4;
      ctx->to_hv.packet_to_hv_bootloader.value = ((uint32_t *)&_binary_obj_hvf3_hvf3_bin_start)[ctx->addr];
      ctx->to_hv.packet_to_hv_bootloader.cmd   = BOOTLOADER_OPCODE_WRITE;

      tx_size = sizeof(packet_bootloader_t);

      if(ctx->timeout > 10) {
        ctx->timeout = 0;
        flash_state  = FLASH_FAILED;
      }
      break;

    case CRC_CHECK:
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len     = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv_bootloader.cmd = BOOTLOADER_OPCODE_CRCCHECK;

      tx_size = sizeof(packet_bootloader_t);

      if(ctx->timeout > 2000) {
        ctx->timeout = 0;
        flash_state  = FLASH_FAILED;
      }
      break;
    case SEND_TO_APP:
      ctx->to_hv.packet_to_hv.header.flags.cmd = DO_RESET;
      ctx->to_hv.packet_to_hv.header.flags.counter++;
      ctx->to_hv.packet_to_hv.header.len     = (sizeof(packet_bootloader_t) - sizeof(stmbl_talk_header_t)) / 4;
      ctx->to_hv.packet_to_hv_bootloader.cmd = BOOTLOADER_OPCODE_NOP;

      tx_size = sizeof(packet_bootloader_t);

      if(ctx->timeout > 2000) {
        ctx->timeout = 0;
        flash_state  = SLAVE_IN_APP;
      }
      break;
    case FLASH_FAILED:
      if(ctx->timeout > 10) {
        ctx->timeout = 0;
        flash_state  = SLAVE_IN_APP;
      }
      break;
  }

  if(ctx->send_state > 1) {
    if(flash_state != SLAVE_IN_APP) {
      tx_size = 0;
    }
    ctx->send_state = 0;
  }
  ctx->send_state++;

  if(tx_size) {
    CRC_ResetDR();
    ctx->to_hv.packet_to_hv.header.crc = CRC_CalcBlockCRC((uint32_t *)&(ctx->to_hv.packet_to_hv.header.slave_addr), tx_size / 4 - 1);

    //start DMA TX transfer
    dma_stream_stop(UART_DRV_TX_DMA);
    UART_DRV_TX_DMA->NDTR = tx_size;
    DMA_Cmd(UART_DRV_TX_DMA, ENABLE);

    // clear uart faults
    PIN(uart_sr) = UART_DRV->SR;
    PIN(uart_dr) = UART_DRV->DR;

    //start DMA RX transfer. The stream is usually stopped mid-transfer
    //(sized for the larger bootloader packet), so set NDTR again rather than
    //rely on the reload (RM0090 10.3.17 step 5)
    dma_stream_stop(UART_DRV_RX_DMA);
    UART_DRV_RX_DMA->NDTR = MAX(sizeof(packet_from_hv_t), sizeof(packet_bootloader_t));
    DMA_Cmd(UART_DRV_RX_DMA, ENABLE);
  }


  PIN(state) = flash_state;
}

void send_boot(char *ptr) {
  send_to_bootloader = 1;
}
COMMAND("hv_update", send_boot, "try hv update");

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx = (struct hv_ctx_t *)ctx_ptr;
  // struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;
  char c;
  while(rb_getc(&hv_rx_buf, &c)) {
    printf("%c", c);
  }

  static flash_state_t last_flash_state = SLAVE_IN_APP;
  static uint32_t last_addr             = 0;

  if(ctx->addr >= last_addr + 1024) {
    printf("hv_update: status: %i%%\n", (int)(100.0 * ctx->addr * 4. / (float)((uint32_t) & (_binary_obj_hvf3_hvf3_bin_size))));
    last_addr = ctx->addr;
  }

  if(last_flash_state != flash_state) {
    switch(flash_state) {
      case SLAVE_IN_APP:
        printf("hv_update: SLAVE_IN_APP\n");
        break;
      case SEND_TO_BOOTLOADER:
        printf("hv_update: SEND_TO_BOOTLOADER\n");
        last_addr = 0;
        break;
      case ERASE_FLASH:
        printf("hv_update: ERASE_FLASH\n");
        last_addr = 0;
        break;
      case SEND_APP:
        printf("hv_update: SEND_APP\n");
        break;
      case CRC_CHECK:
        printf("hv_update: CRC_CHECK\n");
        last_addr = 0;
        break;
      case SEND_TO_APP:
        printf("hv_update: SEND_TO_APP\n");
        last_addr = 0;
        break;
      case FLASH_FAILED:
        printf("hv_update: FLASH_FAILED\n");
        last_addr = 0;
        break;
    }
    last_flash_state = flash_state;
  }
}

hal_comp_t hv_comp_struct = {
    .name      = "hv",
    .nrt       = nrt_func,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct hv_ctx_t),
    .pin_count = sizeof(struct hv_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
