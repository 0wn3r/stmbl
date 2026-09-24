#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "tim.h"
#include "f3hw.h"
#include "common.h"

HAL_COMP(hv);

HAL_PIN(drop);    // dead time compensation, fixed volts
HAL_PIN(drop_k);  // dead time compensation, scaled by dc link and pwm period

// Sign source for the dead time compensation: the COMMANDED current, never
// the measured one. d/q come from the f4, si/co are dq0's sin and cos of the
// same angle, so the reference phase currents cost a few multiplies here
// instead of a second idq in the rt (see main.c).
HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(si);
HAL_PIN(co);
HAL_PIN(phase_mode);  // the compensation runs in 120 deg 3ph mode only
HAL_PIN(iu);          // reference phase currents, outputs for the scope
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

HAL_PIN(cmd_mode);   // the compensation runs in current mode only
HAL_PIN(drop_band);  // current at which the compensation's sign latches [A]

struct hv_ctx_t {
  int32_t pwm_res;
  int8_t drop_su;
  int8_t drop_sv;
  int8_t drop_sw;
};

// Latching sign for the dead time compensation.
//
// The obvious thing here is SIGN2(i, band), and it is wrong: SIGN2 is
// CLAMP(i / band, -1, 1), a linear ramp through zero rather than a sign. Used
// on the compensation it puts dt_drop / band volts per amp *in phase* with the
// current -- 83 V/A at a 300 V link and a 0.1 A band -- which on the d axis is
// 4/3 of that against a winding of well under an ohm. That is positive
// feedback with a loop gain in the hundreds and a time constant of L over it,
// about one pwm period: the current runs away from zero command to the trip.
// Measured on a 0.685 ohm axis it reached 21.8 A and 128 V before the bridge
// stopped talking.
//
// So the sign must have no incremental gain anywhere. It holds its last value
// inside the band and flips only outside it, which is a schmitt trigger: no
// gain in the band, none out of it, and nothing to amplify. It starts at 0, so
// no compensation is applied until a phase has carried real current once.
//
// The cost is the usual one: through a zero crossing the sign is briefly stale
// and the compensation is applied backwards for as long as the current takes
// to cross the band. That is a bounded distortion of 2 * dt_drop, and a small
// band keeps it short -- unlike the ramp, which was unbounded.
static float drop_sign(int8_t *s, float i, float band) {
  if(i > band) {
    *s = 1;
  } else if(i < -band) {
    *s = -1;
  }
  return (float)(*s);
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  ctx->drop_su = 0;
  ctx->drop_sv = 0;
  ctx->drop_sw = 0;

  PIN(enu)       = 1.0;
  PIN(env)       = 1.0;
  PIN(enw)       = 1.0;
  PIN(min_on)    = 0.000003;
  PIN(min_off)   = 0.000003;
  PIN(arr)       = PWM_RES;
  PIN(drop)      = 0;
  PIN(drop_k)    = 0;
  PIN(drop_band) = 0.5;
}

// The latches survive a stop, so clear them here too: coming back up with a
// sign left over from whatever the current was doing when the loop stopped
// would apply the compensation backwards for as long as it takes that phase to
// cross the band.
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx = (struct hv_ctx_t *)ctx_ptr;

  ctx->drop_su = 0;
  ctx->drop_sv = 0;
  ctx->drop_sw = 0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  ctx->pwm_res = (int32_t)CLAMP(PIN(arr), PWM_RES * 0.9, PWM_RES * 1.1);
  TIM8->ARR    = ctx->pwm_res;

  float udc = MAX(PIN(udc), 0.1);

  // Dead time compensation. While a phase current keeps its sign the bridge
  // loses a fixed slice of volt seconds every cycle, worth the dead time as a
  // fraction of the pwm period times the link voltage. Both times are counted
  // in the same timer ticks, so the clock cancels and only the tick ratio is
  // needed -- and it has to be against the live pwm_res, because ls.c walks
  // ARR to phase lock the loop and a constant would drift with the lock.
  //
  // drop_k scales that ideal figure. The switches' own turn off minus turn on
  // delay eats into the programmed dead time, by 28 ticks of 288 on the IPM
  // this was characterised on, so 0.9 is a reasonable starting point; 0 keeps
  // the pre-existing behaviour. drop is a plain volt offset on top, for a
  // board where a fixed number is preferred or to trim what the model misses.
  //
  // Over-compensating is worse than under-compensating: past the real drop the
  // residual changes sign and sits in phase with the current instead of
  // opposing it. That only becomes self excitation if the sign is read from
  // the current it pushes, which is why iu/iv/iw carry the command.
  float dt_drop = PIN(drop) + PIN(drop_k) * (float)PWM_DEADTIME_TICKS / (2.0 * (float)ctx->pwm_res) * udc;

  // Current mode only. The sign is the commanded current, and in volt mode
  // there is none: the command is a voltage, whose sign leads the current by
  // the load's power factor angle -- up to most of a quarter cycle on a lightly
  // loaded induction motor under uf, which would put the compensation
  // backwards for much of every cycle. Falling back to measured current there
  // would restore the self excitation, with no current loop to null it. The
  // volt mode users are the id comps' l and pp tests, which do not need the
  // compensation (the l test measures a time constant, which a constant drop
  // does not touch), and open loop running (uf, acim_ttc mode 2), which loses
  // the dead time volts at low speed as it always has -- acim_ttc's u_boost
  // is the knob for that.
  if(PIN(cmd_mode) == VOLT_MODE || PIN(phase_mode) != PHASE_120_3PH) {
    dt_drop = 0.0;
  }

  // reference phase currents, inverse park and clarke of the command
  float a  = PIN(d_cmd) * PIN(co) - PIN(q_cmd) * PIN(si);
  float b  = PIN(d_cmd) * PIN(si) + PIN(q_cmd) * PIN(co);
  PIN(iu)  = a;
  PIN(iv)  = -a / 2.0 + b / 2.0 * M_SQRT3;
  PIN(iw)  = -a / 2.0 - b / 2.0 * M_SQRT3;

  // A backstop on both pins. The compensation only ever adds volts, so a
  // negative one would invert it, and the honest figure is a few percent of
  // the link -- 3% here. Cap it at 10% so a mistyped drop or drop_k costs a
  // distorted waveform rather than a bridge.
  dt_drop = CLAMP(dt_drop, 0.0, udc * 0.1);

  float band = MAX(PIN(drop_band), 0.05);

  float uu = PIN(u) + dt_drop * drop_sign(&(ctx->drop_su), PIN(iu), band);
  float uv = PIN(v) + dt_drop * drop_sign(&(ctx->drop_sv), PIN(iv), band);
  float uw = PIN(w) + dt_drop * drop_sign(&(ctx->drop_sw), PIN(iw), band);
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
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct hv_ctx_t),
    .pin_count = sizeof(struct hv_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
