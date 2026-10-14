#include "vel_int_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(vel_int);

HAL_PIN(pos_in);
HAL_PIN(pos_out);

HAL_PIN(vel_in);
HAL_PIN(vel_out);

HAL_PIN(cmd_freq);
HAL_PIN(real_cmd_freq);

HAL_PIN(wd);
HAL_PIN(error);
// vel_in is the host's velocity at the frame; held flat it runs half a frame
// late against pos_out. vel_out is extrapolated through the frame by the
// change of vel_in since the previous frame (vel_ip 1; default 0 = hold as
// before, set 1 for a position axis on sserial).
HAL_PIN(vel_ip);
// acceleration feedforward for pid0.acc_ext_cmd (the sserial template links
// it; vel0 is off there, so pid.txt's vel0.acc is 0): the same change of
// vel_in per frame, held until the next frame, 0 while the watchdog holds.
// acc_k scales it (1 = as commanded, 0 = off), max_acc clamps it (0 = none).
HAL_PIN(acc_out);
HAL_PIN(acc_k);
HAL_PIN(max_acc);

struct vel_int_ctx_t {
  float pos;
  float counter;
  float cmd_freq;
  float last_pos_in;
  float v_edge;     // vel_in at the last frame
  float acc;        // change of vel_in per second over the last frame
  float since;      // time since the last frame [s]
  uint32_t have_v;  // v_edge is from a frame inside the watchdog window
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vel_int_ctx_t *ctx      = (struct vel_int_ctx_t *)ctx_ptr;
  struct vel_int_pin_ctx_t *pins = (struct vel_int_pin_ctx_t *)pin_ptr;

  ctx->pos     = 0.0;
  ctx->counter = 0.0;
  ctx->last_pos_in = 0.0;
  PIN(pos_in)  = 0.0;
  PIN(pos_out) = 0.0;

  PIN(vel_in)  = 0.0;
  PIN(vel_out) = 0.0;

  PIN(wd)    = 0.002;
  PIN(error) = 0.0;

  PIN(cmd_freq) = 1000.0;
  ctx->cmd_freq = PIN(cmd_freq);
  PIN(vel_ip)   = 0.0;
  PIN(acc_out)  = 0.0;
  PIN(acc_k)    = 1.0;
  PIN(max_acc)  = 0.0;
  ctx->v_edge   = 0.0;
  ctx->acc      = 0.0;
  ctx->since    = 0.0;
  ctx->have_v   = 0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vel_int_ctx_t *ctx      = (struct vel_int_ctx_t *)ctx_ptr;
  struct vel_int_pin_ctx_t *pins = (struct vel_int_pin_ctx_t *)pin_ptr;

  float p = PIN(pos_in);
  float v = PIN(vel_in);

  if(ctx->counter > PIN(wd) && v != 0.0) {
    v          = 0;
    PIN(error) = 1.0;
  } else {
    PIN(error) = 0.0;
    ctx->counter += period;
  }

  // per instance: EDGE() keeps one static for every instance
  int edge         = p != ctx->last_pos_in;
  ctx->last_pos_in = p;

  ctx->since += period;
  if(edge) {
    ctx->counter = 0.0;
    ctx->pos     = p;
    ctx->cmd_freq += 1.0;
    // nominal host frame rate (real_cmd_freq sags at standstill); frames
    // since the last one, rounded, at least 1, so a dropped frame does not
    // double the step
    float f     = MAX(PIN(cmd_freq), 100.0);
    float n     = MAX(floorf(ctx->since * f + 0.5), 1.0);
    ctx->acc    = ctx->have_v ? (PIN(vel_in) - ctx->v_edge) * f / n : 0.0;
    ctx->v_edge = PIN(vel_in);
    ctx->since  = 0.0;
    ctx->have_v = 1;
  }
  // no frames: velocity is forced to 0 above, and the next frame starts a
  // fresh difference instead of one across the gap
  if(PIN(error) > 0.0 || ctx->since > PIN(wd)) {
    ctx->acc    = 0.0;
    ctx->have_v = 0;
  }
  if(PIN(vel_ip) > 0.0) {
    // extrapolate at most one frame, so a late frame does not run away
    v += ctx->acc * MIN(ctx->since, 1.0 / MAX(PIN(cmd_freq), 100.0));
  }
  if(!edge) {
    ctx->pos += v * period;
  }

  ctx->cmd_freq -= ctx->cmd_freq * period;

  PIN(real_cmd_freq) = ctx->cmd_freq * 0.001 + PIN(real_cmd_freq) * 0.999;

  ctx->pos = mod(ctx->pos);

  float acc = ctx->acc * PIN(acc_k);
  if(PIN(max_acc) > 0.0) {
    acc = CLAMP(acc, -PIN(max_acc), PIN(max_acc));
  }
  PIN(acc_out) = acc;

  PIN(pos_out) = ctx->pos;
  PIN(vel_out) = v;
}

hal_comp_t vel_int_comp_struct = {
    .name      = "vel_int",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct vel_int_ctx_t),
    .pin_count = sizeof(struct vel_int_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
