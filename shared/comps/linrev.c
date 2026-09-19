#include "linrev_comp.h"
//Calculate motor angle from position in machine units

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

HAL_COMP(linrev);

HAL_PIN(scale);

HAL_PIN(cmd_in);
HAL_PIN(cmd_out);

HAL_PIN(cmd_d_in);
HAL_PIN(cmd_d_out);

HAL_PIN(fb_in);
HAL_PIN(fb_out);
HAL_PIN(fb_d_in);
HAL_PIN(fb_d_out);

HAL_PIN(rev_clear);
HAL_PIN(rev);

HAL_PIN(abs_en);
HAL_PIN(abs_rev);

HAL_PIN(pos_offset);

struct linrev_ctx_t {
  int lastq;    //last quadrant
  int32_t rev;  //current multiturn
  int synced;   //rev has been taken from abs_rev at least once
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct linrev_ctx_t *ctx      = (struct linrev_ctx_t *)ctx_ptr;
  struct linrev_pin_ctx_t *pins = (struct linrev_pin_ctx_t *)pin_ptr;

  PIN(pos_offset) = 0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct linrev_ctx_t *ctx      = (struct linrev_ctx_t *)ctx_ptr;
  struct linrev_pin_ctx_t *pins = (struct linrev_pin_ctx_t *)pin_ptr;

  float scale = PIN(scale);
  if(ABS(scale) > 0.01) {
    PIN(cmd_out)   = mod((PIN(cmd_in) / scale) * 2.0 * M_PI);
    PIN(cmd_d_out) = PIN(cmd_d_in) / scale * 2.0 * M_PI;
  }

  int q = quadrant(PIN(fb_in));

  //rev cancels fb_in's wrap, so it has to change exactly at that wrap - which is
  //also where the encoder's own turn counter changes, to within a count. an
  //absolute count read there is ambiguous by a whole revolution and nothing
  //downstream can tell the two apart. so keep counting the wrap, which is
  //self consistent, and use abs_rev only to correct that count in quadrants 1 and
  //4, half a turn away from the ambiguity. a slip is taken out within a quarter
  //turn of motion and cannot accumulate.
  if(q == 3 && ctx->lastq == 2) {
    ctx->rev++;
  }
  if(q == 2 && ctx->lastq == 3) {
    ctx->rev--;
  }

  if(PIN(abs_en) > 0 && (!ctx->synced || q == 1 || q == 4)) {
    ctx->rev    = PIN(abs_rev);
    ctx->synced = 1;
  }

  ctx->lastq = q;

  //with an absolute turn count the homed zero cannot live in rev, it would be
  //overwritten on the next cycle. it belongs in pos_offset.
  if(PIN(rev_clear) > 0 && !(PIN(abs_en) > 0)) {
    ctx->rev = 0;
  }
  PIN(rev)      = ctx->rev;
  PIN(fb_out)   = ((PIN(fb_in) + ctx->rev * M_PI * 2.0) * scale) / (2.0 * M_PI) + PIN(pos_offset) * scale;
  PIN(fb_d_out) = (PIN(fb_d_in) * scale) / (2.0 * M_PI);
}

const hal_comp_t linrev_comp_struct = {
    .name      = "linrev",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct linrev_ctx_t),
    .pin_count = sizeof(struct linrev_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
