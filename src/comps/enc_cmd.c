#include "enc_cmd_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f4xx_conf.h"
#include "hw/hw.h"

/**
* ## Brief
* `enc_cmd` reads an incremental position command (quadrature or step/dir style A/B signals) with an STM32 hardware timer on the F4 board and outputs it as an angle. It is loaded by `conf/template/enc_cmd.txt` (`enc_cmd0.res = conf0.cmd_res`, `rev0.in = enc_cmd0.pos`), so the command position goes through `rev0` to build a multiturn command. It can also drive a fault/ready line back to the controller.
*
* ## Component Explanation
*
* 1. **Connector and timer selection (`remap`, read once in hw_init)**:
* - `remap = 0` (default): CMD connector, A = PA15, B = PB3 on TIM2 (32 bit). Fault line = CMD C (PB5), its line driver enable PB9 is switched on.
* - `remap = 1`: FB0 connector, A = PD12, B = PD13 on TIM4. Fault line = FB0 Z (PD14), enable PD15.
* - `remap = 2`: FB1 connector, A = PE9, B = PE11 on TIM1. Fault line = FB1 Z (PE13), enable PE14.
* - `remap = 3` ("bene style", used by `conf/bene_sanyo.txt` and `conf/spindle_slip_uf.txt`): A = PA8, B = PA9 on TIM1. Fault line = CMD D (PB8), enable PB2.
* - Any other value returns from hw_init without configuring anything, `pos` then reads a timer that was never set up.
* - A and B get the timer alternate function with pull-ups, the fault pin and its enable are push-pull outputs.
*
* 2. **Timer setup (hw_init)**:
* - The timer is put into encoder mode (TI1 and TI2 edges), auto-reload = `2 * res - 1`.
* - The two input channels are then re-initialised: CH1 on both edges (commented as the step clock), CH2 on rising edges (commented as direction). The digital input filter of both channels is `input_filter`, clamped to 0..15 (default 3).
* - `res` is copied at init and clamped to >= 1 (default 4096).
*
* 3. **Position output (rt)**:
* - Each rt cycle the counter is read and converted:
* ```c
* pos = mod(counter * 2 * pi / res);   // rad, wrapped to +-pi
* ```
* - Because the counter range is `2 * res` counts, `pos` wraps twice per counter period; the multiturn tracking is done downstream (e.g. `rev0`). The bene config comments `cmd_res` as "cmd counts / rev * 2".
* - If `res` changes at runtime the auto-reload register is updated.
* - `a` and `b` show the raw logic levels of the A and B inputs.
*
* 4. **Fault output (rt)**:
* - `fault > 0` drives the fault line high, otherwise it is driven low (e.g. `enc_cmd0.fault = fault0.fault` or an inverted enable signal).
*
* {{% hint warning %}}
* The `mode` pin (documented as 0 = quad, 1 = step/dir, 2 = dir/step, 3 = up/down, and set by several configs) is never read: the timer mode is fixed by the setup in hw_init. That setup mixes encoder mode with input-capture settings that look copied from a step/dir experiment (CH1 mapped to the other input, `TIM_ICPrescaler = 1` instead of `TIM_ICPSC_DIV1`), so check the counting behaviour on real hardware. With TIM1/TIM4 (`remap` 1..3) the counter is 16 bit, so `res` must stay <= 32768. The pin/timer definitions exist only for the V4 board.
* {{% /hint %}}
*/

HAL_COMP(enc_cmd);

HAL_PIN(res);           // *parameter*, Counts per revolution, auto-reload = 2 * res - 1 (default 4096)
HAL_PIN(pos);           // *output*, Command position (rad, +-pi)
HAL_PIN(a);             // *output*, Raw level of the A input (0/1)
HAL_PIN(b);             // *output*, Raw level of the B input (0/1)
HAL_PIN(fault);         // *input*, > 0 drives the fault/ready line high
HAL_PIN(mode);          // *parameter*, Intended 0 = quad, 1 = step/dir, 2 = dir/step, 3 = up/down; currently not used by the code
HAL_PIN(remap);         // *parameter*, Input connector: 0 = CMD (TIM2), 1 = FB0 (TIM4), 2 = FB1 (TIM1), 3 = PA8/PA9 bene style (TIM1)
HAL_PIN(input_filter);  // *parameter*, Timer input filter setting 0..15 (default 3)

struct enc_cmd_ctx_t {
  int e_res;
  uint32_t a_pin, b_pin, c_pin, c_en_pin, a_pin_source, b_pin_source, tim_af, tim_rcc;
  GPIO_TypeDef *a_port, *b_port, *c_port, *c_en_port;
  TIM_TypeDef *tim;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_cmd_ctx_t *ctx      = (struct enc_cmd_ctx_t *)ctx_ptr;
  struct enc_cmd_pin_ctx_t *pins = (struct enc_cmd_pin_ctx_t *)pin_ptr;
  ctx->e_res                     = 0;
  PIN(res)                       = 4096.0;
  PIN(input_filter)              = 3;
}

static void hw_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_cmd_ctx_t *ctx      = (struct enc_cmd_ctx_t *)ctx_ptr;
  struct enc_cmd_pin_ctx_t *pins = (struct enc_cmd_pin_ctx_t *)pin_ptr;

  ctx->e_res = (int)PIN(res);
  if(ctx->e_res < 1) {
    ctx->e_res = 1;
  }

  //RCC_AHB1PeriphClockCmd(FB1_|ENC0_B_IO_RCC, ENABLE);    //Enable needed Clocks for IOs
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;


  switch((int)PIN(remap)) {
    case 0:
      ctx->a_pin        = CMD_A_PIN;
      ctx->b_pin        = CMD_B_PIN;
      ctx->c_pin        = CMD_C_PIN;
      ctx->c_en_pin     = CMD_C_EN_PIN;
      ctx->a_port       = CMD_A_PORT;
      ctx->b_port       = CMD_B_PORT;
      ctx->c_port       = CMD_C_PORT;
      ctx->c_en_port    = CMD_C_EN_PORT;
      ctx->a_pin_source = CMD_A_PIN_SOURCE;
      ctx->b_pin_source = CMD_B_PIN_SOURCE;
      ctx->tim_af       = CMD_ENC_TIM_AF;
      ctx->tim_rcc      = CMD_ENC_TIM_RCC;
      ctx->tim          = CMD_ENC_TIM;
      RCC_APB1PeriphClockCmd(ctx->tim_rcc, ENABLE);
      break;
    case 1:
      ctx->a_pin        = FB0_A_PIN;
      ctx->b_pin        = FB0_B_PIN;
      ctx->c_pin        = FB0_Z_PIN;
      ctx->c_en_pin     = FB0_Z_TXEN_PIN;
      ctx->a_port       = FB0_A_PORT;
      ctx->b_port       = FB0_B_PORT;
      ctx->c_port       = FB0_Z_PORT;
      ctx->c_en_port    = FB0_Z_TXEN_PORT;
      ctx->a_pin_source = FB0_A_PIN_SOURCE;
      ctx->b_pin_source = FB0_B_PIN_SOURCE;
      ctx->tim_af       = FB0_ENC_TIM_AF;
      ctx->tim_rcc      = FB0_ENC_TIM_RCC;
      ctx->tim          = FB0_ENC_TIM;
      RCC_APB1PeriphClockCmd(ctx->tim_rcc, ENABLE);
      break;
    case 2:
      ctx->a_pin        = FB1_A_PIN;
      ctx->b_pin        = FB1_B_PIN;
      ctx->c_pin        = FB1_Z_PIN;
      ctx->c_en_pin     = FB1_Z_TXEN_PIN;
      ctx->a_port       = FB1_A_PORT;
      ctx->b_port       = FB1_B_PORT;
      ctx->c_port       = FB1_Z_PORT;
      ctx->c_en_port    = FB1_Z_TXEN_PORT;
      ctx->a_pin_source = FB1_A_PIN_SOURCE;
      ctx->b_pin_source = FB1_B_PIN_SOURCE;
      ctx->tim_af       = FB1_ENC_TIM_AF;
      ctx->tim_rcc      = FB1_ENC_TIM_RCC;
      ctx->tim          = FB1_ENC_TIM;
      RCC_APB2PeriphClockCmd(ctx->tim_rcc, ENABLE);
      break;
    case 3:
      ctx->a_pin        = GPIO_Pin_8;
      ctx->b_pin        = GPIO_Pin_9;
      ctx->c_pin        = GPIO_Pin_8;
      ctx->c_en_pin     = GPIO_Pin_2;
      ctx->a_port       = GPIOA;
      ctx->b_port       = GPIOA;
      ctx->c_port       = GPIOB;
      ctx->c_en_port    = GPIOB;
      ctx->a_pin_source = GPIO_PinSource8;
      ctx->b_pin_source = GPIO_PinSource9;
      ctx->tim_af       = FB1_ENC_TIM_AF;
      ctx->tim_rcc      = FB1_ENC_TIM_RCC;
      ctx->tim          = FB1_ENC_TIM;
      RCC_APB2PeriphClockCmd(ctx->tim_rcc, ENABLE);
      break;
    default:
      return;
  }
  GPIO_InitStruct.GPIO_Pin = ctx->a_pin;
  GPIO_Init(ctx->a_port, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Pin = ctx->b_pin;
  GPIO_Init(ctx->b_port, &GPIO_InitStruct);

  GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Pin   = ctx->c_pin;
  GPIO_Init(ctx->c_port, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = ctx->c_en_pin;
  GPIO_Init(ctx->c_en_port, &GPIO_InitStruct);

  GPIO_SetBits(ctx->c_en_port, ctx->c_en_pin);

  //Bind pins to Timer
  GPIO_PinAFConfig(ctx->a_port, ctx->a_pin_source, ctx->tim_af);
  GPIO_PinAFConfig(ctx->b_port, ctx->b_pin_source, ctx->tim_af);

  TIM_SetAutoreload(ctx->tim, ctx->e_res * 2 - 1);
  // quad
  TIM_Cmd(ctx->tim, DISABLE);
  TIM_EncoderInterfaceConfig(ctx->tim, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICInitTypeDef TIM_ICInitStruct;
  TIM_ICInitStruct.TIM_Channel     = TIM_Channel_1;
  TIM_ICInitStruct.TIM_ICFilter    = MAX(MIN(PIN(input_filter), 15), 0);  //Digital filtering @ 1/32 fDTS
  TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_BothEdge;             //Just trigger at the rising edge, because its the  clock
  TIM_ICInitStruct.TIM_ICPrescaler = 1;                                   //no prescaler, capture is done each time an edge is detected on the capture input
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_IndirectTI;          //IC1 mapped to TI1
  TIM_ICInit(ctx->tim, &TIM_ICInitStruct);

  TIM_ICInitStruct.TIM_Channel     = TIM_Channel_2;
  TIM_ICInitStruct.TIM_ICFilter    = MAX(MIN(PIN(input_filter), 15), 0);  //Digital filtering @ 1/32 fDTS
  TIM_ICInitStruct.TIM_ICPolarity  = TIM_ICPolarity_Rising;               //Trigger at every edge, because its the direction
  TIM_ICInitStruct.TIM_ICPrescaler = 1;                                   //no prescaler, capture is done each time an edge is detected on the capture input
  TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;            //IC2 mapped to TI1
  TIM_ICInit(ctx->tim, &TIM_ICInitStruct);
  TIM_Cmd(ctx->tim, ENABLE);
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct enc_cmd_ctx_t *ctx      = (struct enc_cmd_ctx_t *)ctx_ptr;
  struct enc_cmd_pin_ctx_t *pins = (struct enc_cmd_pin_ctx_t *)pin_ptr;

  int32_t tim = TIM_GetCounter(ctx->tim);

  PIN(a) = GPIO_ReadInputDataBit(ctx->a_port, ctx->a_pin);
  PIN(b) = GPIO_ReadInputDataBit(ctx->b_port, ctx->b_pin);

  float p = 0.0;
  p       = mod(tim * 2.0f * M_PI / (float)ctx->e_res);

  PIN(pos) = p;

  int r = (int)PIN(res);
  if(r < 1) {
    r = 1;
  }

  if(ctx->e_res != r) {
    ctx->e_res = r;
    TIM_SetAutoreload(ctx->tim, ctx->e_res * 2 - 1);
  }

  if(PIN(fault) > 0.0) {
    GPIO_SetBits(ctx->c_port, ctx->c_pin);
  } else {
    GPIO_ResetBits(ctx->c_port, ctx->c_pin);
  }
}

const hal_comp_t enc_cmd_comp_struct = {
    .name      = "enc_cmd",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = hw_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct enc_cmd_ctx_t),
    .pin_count = sizeof(struct enc_cmd_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
