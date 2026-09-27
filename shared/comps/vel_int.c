#include "vel_int_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `vel_int` interpolates a position command that arrives at a lower or irregular rate (e.g. from LinuxCNC via smart serial) by integrating the commanded velocity between updates. F4 component, loaded by `conf/template/sserial.txt`: `vel_int0.pos_in = linrev0.cmd_out`, `vel_int0.vel_in = linrev0.cmd_d_out`, `rev0.in = vel_int0.pos_out`.
*
* ## Component Explanation
* All work is done in `rt`. Defaults: `wd` = 0.002 s, `cmd_freq` = 1000.
*
* 1. **Interpolation**:
* - When `pos_in` changes, the internal position is set to `pos_in` and the watchdog counter is reset.
* - Otherwise the internal position is advanced by `vel_in * period`. `pos_out` is wrapped with `mod()`, `vel_out` is the (possibly zeroed) `vel_in`.
*
* 2. **Watchdog**:
* - If `pos_in` has not changed for more than `wd` seconds while `vel_in` is not 0, the velocity is forced to 0 (no more extrapolation) and `error` is set to 1. Otherwise `error` is 0.
*
* 3. **Command rate**:
* - `real_cmd_freq` estimates how often `pos_in` changes (Hz): every change adds 1 to a counter that decays with a 1 s time constant; the result is additionally low pass filtered (factor 0.001 per period).
*
* {{% hint warning %}}
* Change detection uses the `EDGE()` macro, which keeps its state in a static variable, so all instances of `vel_int` share it. The `cmd_freq` pin only seeds the internal counter at init and is otherwise unused.
* {{% /hint %}}
*/

HAL_COMP(vel_int);

HAL_PIN(pos_in);   // *input*, Position command, sampled at the command rate (rad)
HAL_PIN(pos_out);  // *output*, Interpolated position command (rad, +-pi)

HAL_PIN(vel_in);   // *input*, Velocity command used for interpolation (rad/s)
HAL_PIN(vel_out);  // *output*, Velocity command, 0 after watchdog timeout (rad/s)

HAL_PIN(cmd_freq);       // *parameter*, Initial value of the command rate estimate (Hz), default 1000
HAL_PIN(real_cmd_freq);  // *output*, Measured rate of pos_in updates (Hz)

HAL_PIN(wd);     // *parameter*, Watchdog time without pos_in change (s), default 0.002
HAL_PIN(error);  // *output*, 1 = watchdog timed out while vel_in != 0

struct vel_int_ctx_t {
  float pos;
  float counter;
  float cmd_freq;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vel_int_ctx_t *ctx      = (struct vel_int_ctx_t *)ctx_ptr;
  struct vel_int_pin_ctx_t *pins = (struct vel_int_pin_ctx_t *)pin_ptr;

  ctx->pos     = 0.0;
  ctx->counter = 0.0;
  PIN(pos_in)  = 0.0;
  PIN(pos_out) = 0.0;

  PIN(vel_in)  = 0.0;
  PIN(vel_out) = 0.0;

  PIN(wd)    = 0.002;
  PIN(error) = 0.0;

  PIN(cmd_freq) = 1000.0;
  ctx->cmd_freq = PIN(cmd_freq);
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

  if(EDGE(p)) {
    ctx->counter = 0.0;
    ctx->pos     = p;
    ctx->cmd_freq += 1.0;
  } else {
    ctx->pos += v * period;
  }

  ctx->cmd_freq -= ctx->cmd_freq * period;

  PIN(real_cmd_freq) = ctx->cmd_freq * 0.001 + PIN(real_cmd_freq) * 0.999;

  ctx->pos = mod(ctx->pos);

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
