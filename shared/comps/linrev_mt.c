#include "linrev_mt_comp.h"
//Calculate motor angle from position in machine units, version for multiturn (16 bit) absolute encoder

/*

      |
  Q2  |  Q1
      |
-------------
      |
  Q3  |  Q4
      |

*/


#include "hal.h"
#include "angle.h"
#include "defines.h"
#include <math.h>

#define TWOPOWER15 32768U

HAL_COMP(linrev_mt);

HAL_PIN(scale);

HAL_PIN(cmd_in);
HAL_PIN(cmd_out);

HAL_PIN(cmd_d_in);
HAL_PIN(cmd_d_out);

HAL_PIN(fb_in);
HAL_PIN(fb_out);
HAL_PIN(fb_d_in);
HAL_PIN(fb_d_out);

HAL_PIN(turns);


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct linrev_mt_pin_ctx_t *pins = (struct linrev_mt_pin_ctx_t *)pin_ptr;

  float scale = PIN(scale);
  if(ABS(scale) > 0.01) {
    PIN(cmd_out)   = mod((PIN(cmd_in) / scale) * 2.0 * M_PI);
    PIN(cmd_d_out) = PIN(cmd_d_in) / scale * 2.0 * M_PI;
  }

  uint32_t turns = lround(PIN(turns));
  int32_t shifted_turns;
  if (turns > TWOPOWER15 - 1) {
    shifted_turns = turns % TWOPOWER15 - TWOPOWER15;
  } else {
    shifted_turns = turns;
  }

  PIN(fb_out)   = ((PIN(fb_in) + shifted_turns * M_PI * 2.0) * scale) / (2.0 * M_PI);
  PIN(fb_d_out) = (PIN(fb_d_in) * scale) / (2.0 * M_PI);
}

const hal_comp_t linrev_mt_comp_struct = {
    .name      = "linrev_mt",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct linrev_mt_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
