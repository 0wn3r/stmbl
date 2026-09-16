#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "tim.h"
#include "f3hw.h"

HAL_COMP(hv);

HAL_PIN(drop);

//IU IV IW input in amp
HAL_PIN(iu);
HAL_PIN(iv);
HAL_PIN(iw);

//U V W input in Volt
HAL_PIN(u);
HAL_PIN(v);
HAL_PIN(w);

//dclink in, to scale pwm
HAL_PIN(udc);

//TODO: half bridge enable in
HAL_PIN(enu);
HAL_PIN(env);
HAL_PIN(enw);

HAL_PIN(min_on);   // min on time [s]
HAL_PIN(min_off);  // min off time [s]

HAL_PIN(arr);

struct hv_ctx_t {
  int32_t pwm_res;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct hv_ctx_t * ctx = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  PIN(enu)     = 1.0;
  PIN(env)     = 1.0;
  PIN(enw)     = 1.0;
  PIN(min_on)  = 0.000003;
  PIN(min_off) = 0.000003;
  PIN(arr)     = PWM_RES;
  PIN(drop)    = 0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  ctx->pwm_res = (int32_t)CLAMP(PIN(arr), PWM_RES * 0.9, PWM_RES * 1.1);
  TIM8->ARR    = ctx->pwm_res;

  float udc = MAX(PIN(udc), 0.1);

  // drop compensation
  float uu = PIN(u) + PIN(drop) * SIGN2(PIN(iu), 0.1);
  float uv = PIN(v) + PIN(drop) * SIGN2(PIN(iv), 0.1);
  float uw = PIN(w) + PIN(drop) * SIGN2(PIN(iw), 0.1);
  //convert voltages to PWM output compare values
  int32_t u = (int32_t)(CLAMP(uu, 0.0, udc) / udc * (float)(ctx->pwm_res));
  int32_t v = (int32_t)(CLAMP(uv, 0.0, udc) / udc * (float)(ctx->pwm_res));
  int32_t w = (int32_t)(CLAMP(uw, 0.0, udc) / udc * (float)(ctx->pwm_res));
  //convert on and off times to PWM output compare values.
  //TIM8 is center aligned, so a PWM period spans 2*ARR timer ticks and one
  //compare unit is worth 2 ticks of on time -- the on time resolution is a
  //constant PWM_TIM_CLK/2, independent of ARR (which ls.c varies to phase
  //lock the loop to the master).
  int32_t min_on  = (int32_t)(PWM_TIM_CLK / 2.0 * PIN(min_on) + 0.5);
  int32_t min_off = (int32_t)(PWM_TIM_CLK / 2.0 * PIN(min_off) + 0.5);

  // if the commanded phase spread is wider than what min_on/min_off leave
  // available, no common-mode shift can satisfy both boundaries at once
  // (a double violation). Scale the spread down around its center so the
  // shift below always has enough room, instead of relying on the
  // per-phase clamp further down to silently distort the output.
  int32_t max_range = ctx->pwm_res - min_on - min_off;
  int32_t range      = MAX3(u, v, w) - MIN3(u, v, w);
  if(max_range > 0 && range > max_range) {
    int32_t center = (MAX3(u, v, w) + MIN3(u, v, w)) / 2;
    u              = center + (int32_t)(((int64_t)(u - center) * max_range) / range);
    v              = center + (int32_t)(((int64_t)(v - center) * max_range) / range);
    w              = center + (int32_t)(((int64_t)(w - center) * max_range) / range);
  }

  // evaluate both checks against the pre-correction values so the min_off
  // shift can't be triggered by (and undo) the min_on shift, or vice versa
  int32_t u0 = u, v0 = v, w0 = w;

  if((u0 > 0 && u0 < min_on) || (v0 > 0 && v0 < min_on) || (w0 > 0 && w0 < min_on)) {
    u += min_on;
    v += min_on;
    w += min_on;
  }

  if((u0 > ctx->pwm_res - min_off) || (v0 > ctx->pwm_res - min_off) || (w0 > ctx->pwm_res - min_off)) {
    u -= min_off;
    v -= min_off;
    w -= min_off;
  }

  // final per-phase safety net: guarantees every phase individually clears
  // both min_on and min_off even when a single common-mode shift above
  // can't satisfy both boundaries at once (e.g. one phase near 0 and
  // another near full scale in the same cycle -- a double violation)
  if(u > 0 && u < min_on) u = min_on;
  if(v > 0 && v < min_on) v = min_on;
  if(w > 0 && w < min_on) w = min_on;

  u = CLAMP(u, 0, ctx->pwm_res - min_off);
  v = CLAMP(v, 0, ctx->pwm_res - min_off);
  w = CLAMP(w, 0, ctx->pwm_res - min_off);

#ifdef PWM_INVERT
  PWM_U = ctx->pwm_res - u;
  PWM_V = ctx->pwm_res - v;
  PWM_W = ctx->pwm_res - w;
#else
  PWM_U = u;
  PWM_V = v;
  PWM_W = w;
#endif
}

hal_comp_t hv_comp_struct = {
    .name      = "hv",
    .nrt       = 0,
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
