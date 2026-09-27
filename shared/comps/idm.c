#include "idm_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `idm` identifies the mechanical parameters of the axis: inertia, viscous damping, coulomb friction and a constant torque offset (e.g. gravity on a vertical axis). It runs on the F4 board and is loaded by the `id_mot` template (bare motor, results for `conf0.j`, `conf0.d`, `conf0.f`, `conf0.o`) and, through `id_sys` (which links `id_mot` and sets `idm0.sys = 1`), for the inertia of the coupled load (`conf0.j_sys`). It moves the axis back and forth through `pid0` and adapts its estimates until the feedback torque of the position/velocity loop no longer correlates with acceleration, velocity, direction or a constant.
*
* ## Component Explanation
*
* 1. **Before you start**:
* - The motor must already run with a working current loop and commutation (run `id_pmsm`, `id_dc` or `id_acim` first).
* - The axis travels between `min_pos` and `max_pos` (default -10 and +10 rad, about 1.6 turns each way) around position 0 of the motor feedback, at up to `max_vel` (25 rad/s) and `max_acc` (250 rad/s^2). Make sure there is room, or reduce these pins before enabling.
* - At the console, `link id_mot` (or `link id_sys` for the load inertia, which needs `conf0.j` from the motor run already in the config). The template wires `idm0.en = fault0.en_out`, the trajectory into `pid0.pos_ext_cmd` / `vel_ext_cmd` / `acc_ext_cmd`, the estimates back into `pid0.j_mot`, `pid0.d`, `pid0.f`, `pid0.o` (with `id_sys`: `pid0.j_sys = idm0.inertia`), and `pid0.torque_cmd` / `pid0.fb_torque_cmd` into `torque` / `fb_torque`. It also sets `pid0.j_sys = 0`, `conf0.max_pos_error = 0`, `conf0.max_sat = 10`, `idm0.li = 0.005` and uses the soft loop gains `pos_bw`, `vel_bw`, `vel_d` (5, 40, 4) of this component for `pid0` during the test.
*
* 2. **Procedure (state machine, `state` pin)**:
* - `0`: idle. When `en` goes high the rt function resets the `*_sum` / `*_time` pins and goes to `1.0`. `en` low returns to `0` from any state.
* - `1.0` -> `1.1` (nrt): with `auto_step >= 1` (default 1.4) it goes straight to `1.2`; otherwise it prints a prompt and waits in `1.1` for `idm0.state = 1.2`.
* - `1.2`: five ramp-up sweeps (`sub_state` 1..5). Sweep k uses `max_acc * k / 5` and `max_vel * k / 5` and lasts `2 * (|max_pos - min_pos| / v + 2 * v / a)`, going to `max_pos` for the first half and back to `min_pos` for the second. With the defaults this takes about 20 s.
* - `1.3`: 90 s of full speed / full acceleration moves, reversing whenever the position is within 0.1 rad of an end point.
* - `1.4` (nrt): prints `conf0.j` (or `conf0.j_sys` when `sys > 0`), `conf0.o`, `conf0.d` and `conf0.f` as lines to append to the config, then goes to `1.5` (done, the axis is held at its last position).
* - The whole run takes about 110 s with the defaults. Afterwards continue with `id_sys` or `id_pid`.
*
* 3. **Trajectory**:
* - `pos` is integrated from `vel_cmd` and `acc_cmd`; `pos_cmd` is `mod(pos)` (wrapped to +-pi). The generator is a bang-bang profile: `vel = LIMIT(acc * sqrt(2 * |to_go| / max_acc), max_vel)`, `acc_cmd = LIMIT((vel - vel_cmd) / period, max_acc)`.
* - `pos` is never re-anchored to the actual position: after boot it starts at 0, on a rerun at wherever the last run stopped. `pid0.pos_ext_cmd` is an absolute command, so if the motor is not near that position when `en` rises the first move starts with a position step.
*
* 4. **Estimation (rt, states 1.2 and 1.3)**:
* - Each estimate is a gradient loop on the feedback torque of `pid0`; because the estimates are fed back into the pid feedforward, each one stops moving once the feedback torque no longer contains its regressor:
* ```c
* inertia  += period / ji * fb_torque * acc_cmd * period;
* damping  += period / di * fb_torque * vel_cmd * period;
* friction += period / fi * fb_torque * SIGN(vel_cmd) * period;
* offset   += period / li * fb_torque * period;
* ```
* - `ji`, `di`, `fi`, `li` are divisors: larger means slower and smoother (defaults 10, 1.0, 0.001, 0.01). The offset loop has a time constant of `li / period`, i.e. 50 s at the default 0.01 and the 5 kHz rt rate, which is why the template lowers it to 0.005 (25 s).
* - Clamps: `inertia` 5e-6..50 kgm^2 (0..50 in state 1.2 when `sys > 0`), `damping` 0..100 Nm/(rad/s), `friction` 0..100 Nm, `offset` -100..100 Nm. `inertia` starts at 0.0002 from nrt_init; the others start at 0 (or at their last value on a rerun).
* - In state 1.2 it also accumulates open-loop averages of the total torque (`inertia_sum`, `damping_sum`, `friction_sum`, `offset_sum`, divided by `acc_time`, `vel_time` and `time` at the end of 1.2). These are diagnostics only and are not used for the results.
*
* {{% hint warning %}}
* In state 1.3 the inertia is always clamped to at least 5e-6, even when `sys > 0` (state 1.2 allows 0 there). The pins `freq` and `amp` are unused leftovers (`amp` is only ever cleared).
* {{% /hint %}}
*/

HAL_COMP(idm);

HAL_PIN(en);  // *input*, enable; high starts the identification, low aborts it (state -> 0)

HAL_PIN(state);      // *input/output*, state machine: 0 off, 1.1 wait for start, 1.2 ramp-up sweeps, 1.3 90 s run, 1.4 print results, 1.5 done
HAL_PIN(sub_state);  // *output*, sweep 1..5 in state 1.2, acc/vel scaled by sub_state/5
HAL_PIN(timer);      // *output*, time in the current sweep / in state 1.3 (s)
HAL_PIN(acc_time);   // *output*, time spent accelerating in state 1.2 (s)
HAL_PIN(vel_time);   // *output*, time spent moving in state 1.2 (s)
HAL_PIN(time);       // *output*, total time of state 1.2 (s)

HAL_PIN(freq);     // *parameter*, unused
HAL_PIN(amp);      // *output*, unused, only cleared to 0
HAL_PIN(min_pos);  // *parameter*, lower end of the travel (rad, around feedback position 0), default -10
HAL_PIN(max_pos);  // *parameter*, upper end of the travel (rad), default 10
HAL_PIN(max_vel);  // *parameter*, test velocity (rad/s), default 25
HAL_PIN(max_acc);  // *parameter*, test acceleration (rad/s^2), default 250

HAL_PIN(pos);      // *output*, unwrapped trajectory position (rad)
HAL_PIN(pos_cmd);  // *output*, position command, pos wrapped to +-pi (rad), to pid0.pos_ext_cmd
HAL_PIN(vel_cmd);  // *output*, velocity command (rad/s), to pid0.vel_ext_cmd
HAL_PIN(acc_cmd);  // *output*, acceleration command (rad/s^2), to pid0.acc_ext_cmd

HAL_PIN(ji);  // *parameter*, inertia adaptation divisor, larger = slower, default 10
HAL_PIN(fi);  // *parameter*, friction adaptation divisor, default 0.001
HAL_PIN(di);  // *parameter*, damping adaptation divisor, default 1.0
HAL_PIN(li);  // *parameter*, offset adaptation divisor, time constant li/period, default 0.01

HAL_PIN(torque);     // *input*, total torque command (Nm), from pid0.torque_cmd, only used for the _sum diagnostics
HAL_PIN(fb_torque);  // *input*, feedback torque of the pid (Nm), from pid0.fb_torque_cmd, drives the estimation

HAL_PIN(inertia_sum);   // *output*, diagnostic, mean of acc_cmd * torque while accelerating in state 1.2
HAL_PIN(friction_sum);  // *output*, diagnostic, mean of sign(vel_cmd) * torque while moving in state 1.2 (Nm)
HAL_PIN(damping_sum);   // *output*, diagnostic, mean of vel_cmd * torque while moving in state 1.2
HAL_PIN(offset_sum);    // *output*, diagnostic, mean torque over state 1.2 (Nm)

HAL_PIN(inertia);   // *output*, estimated inertia (kgm^2), result for conf0.j or conf0.j_sys, default 0.0002
HAL_PIN(damping);   // *output*, estimated viscous damping (Nm/(rad/s)), result for conf0.d
HAL_PIN(friction);  // *output*, estimated coulomb friction (Nm), result for conf0.f
HAL_PIN(offset);    // *output*, estimated constant torque, e.g. gravity (Nm), result for conf0.o

HAL_PIN(pos_bw);  // *parameter*, position bandwidth used by pid0 during the test, default 5
HAL_PIN(vel_bw);  // *parameter*, velocity bandwidth used by pid0 during the test, default 40
HAL_PIN(vel_d);   // *parameter*, velocity loop damping used by pid0 during the test, default 4

HAL_PIN(sys);  // *parameter*, 0 = bare motor (prints conf0.j), >0 = load inertia (prints conf0.j_sys), set by id_sys

HAL_PIN(target);     // *output*, end point the trajectory is moving to (rad)
HAL_PIN(auto_step);  // *parameter*, >= 1 starts without waiting for state = 1.2, default 1.4

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idm_ctx_t * ctx = (struct idm_ctx_t *)ctx_ptr;
  struct idm_pin_ctx_t *pins = (struct idm_pin_ctx_t *)pin_ptr;
  PIN(fi)                    = 0.001;
  PIN(li)                    = 0.01;
  PIN(di)                    = 1.0;
  PIN(ji)                    = 10.0;
  PIN(pos_bw)                = 5.0;
  PIN(vel_bw)                = 40.0;
  PIN(vel_d)                 = 4.0;
  PIN(max_vel)               = 25.0;
  PIN(max_acc)               = 250.0;
  PIN(min_pos)               = -10.0;
  PIN(max_pos)               = 10.0;
  PIN(auto_step)             = 1.4;
  PIN(inertia)               = 0.0002;
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idm_ctx_t * ctx = (struct idm_ctx_t *)ctx_ptr;
  struct idm_pin_ctx_t *pins = (struct idm_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      //PIN(inertia) = 0.0001;
      break;

    case 10:
      PIN(state)  = 1.1;
      PIN(timer)  = 0.0;
      PIN(target) = PIN(min_pos);

      if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Measure friction, damping and inertia\n");
        printf("the motor will move\n");
        printf("idm0.state = 1.2 <font color='green'>to start</font>\n");
      }
      break;

    case 14:
      if(PIN(sys) > 0.0) {
        printf("conf0.j_sys = %f <font color='green'># append to config</font>\n", PIN(inertia));
      } else {
        printf("conf0.j = %f <font color='green'># append to config</font>\n", PIN(inertia));
      }
      printf("conf0.o = %f <font color='green'># append to config</font>\n", PIN(offset));
      printf("conf0.d = %f <font color='green'># append to config</font>\n", PIN(damping));
      printf("conf0.f = %f <font color='green'># append to config</font>\n", PIN(friction));
      printf("done\n");
      if(PIN(sys) > 0.0) {
        printf("continue with id_pid\n");
      } else {
        printf("continue with id_sys or id_pid\n");
      }
      PIN(state) = 1.5;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idm_ctx_t * ctx = (struct idm_ctx_t *)ctx_ptr;
  struct idm_pin_ctx_t *pins = (struct idm_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  float to_go;
  float time_to_go;
  float acc;
  float vel;
  float max_acc;
  float max_vel;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(timer)   = 0.0;
      PIN(acc_cmd) = 0.0;
      PIN(vel_cmd) = 0.0;
      PIN(amp)     = 0.0;

      if(PIN(en) > 0.0) {
        PIN(state)     = 1.0;
        PIN(sub_state) = 1.0;
        PIN(timer)     = 0.0;

        PIN(inertia_sum)  = 0.0;
        PIN(friction_sum) = 0.0;
        PIN(damping_sum)  = 0.0;
        PIN(offset_sum)   = 0.0;
        PIN(acc_time)     = 0.0;
        PIN(vel_time)     = 0.0;
        PIN(time)         = 0.0;
      }
      break;

    case 12:
      max_acc = PIN(max_acc) / 5.0 * PIN(sub_state);
      max_vel = PIN(max_vel) / 5.0 * PIN(sub_state);

      PIN(pos) += PIN(vel_cmd) * period + PIN(acc_cmd) * period * period / 2.0;
      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) += PIN(acc_cmd) * period;
      to_go        = PIN(target) - PIN(pos);
      time_to_go   = sqrtf(2.0 * ABS(to_go) / max_acc);
      acc          = max_acc * SIGN(to_go);
      vel          = acc * time_to_go;
      vel          = LIMIT(vel, max_vel);
      acc          = (vel - PIN(vel_cmd)) / period;
      PIN(acc_cmd) = LIMIT(acc, max_acc);

      if(time_to_go < period) {
        time_to_go   = 0.0;
        to_go        = 0.0;
        PIN(acc_cmd) = 0.0;
        PIN(pos)     = PIN(target);
        PIN(vel_cmd) = 0.0;
      }

      if(ABS(PIN(acc_cmd)) > 0.0) {
        PIN(acc_time) += period;
        PIN(inertia_sum) += PIN(acc_cmd) * PIN(torque) * period;
      }

      if(ABS(PIN(vel_cmd)) > 0.0) {
        PIN(vel_time) += period;
        PIN(damping_sum) += PIN(vel_cmd) * PIN(torque) * period;
        PIN(friction_sum) += SIGN(PIN(vel_cmd)) * PIN(torque) * period;
      }

      PIN(time) += period;
      PIN(offset_sum) += PIN(torque) * period;

      PIN(inertia) += period / PIN(ji) * PIN(fb_torque) * PIN(acc_cmd) * period;
      PIN(damping) += period / PIN(di) * PIN(fb_torque) * PIN(vel_cmd) * period;
      PIN(friction) += period / PIN(fi) * PIN(fb_torque) * SIGN(PIN(vel_cmd)) * period;
      PIN(offset) += period / PIN(li) * PIN(fb_torque) * period;

      if(PIN(sys) > 0.0) {
        PIN(inertia) = CLAMP(PIN(inertia), 0.0, 50.0);
      } else {
        PIN(inertia) = CLAMP(PIN(inertia), 0.000005, 50.0);
      }
      PIN(damping)  = CLAMP(PIN(damping), 0.0, 100.0);
      PIN(friction) = CLAMP(PIN(friction), 0.0, 100.0);
      PIN(offset)   = CLAMP(PIN(offset), -100.0, 100.0);

      PIN(timer) += period;
      if(PIN(timer) < (ABS(PIN(max_pos) - PIN(min_pos)) / max_vel + 2.0 * max_vel / max_acc)) {
        PIN(target) = PIN(max_pos);
      } else {
        PIN(target) = PIN(min_pos);
      }
      if(PIN(timer) > 2.0 * (ABS(PIN(max_pos) - PIN(min_pos)) / max_vel + 2.0 * max_vel / max_acc)) {
        PIN(timer) = 0.0;
        PIN(sub_state)
        ++;
      }

      if(PIN(sub_state) > 5.0) {
        PIN(state) = 1.3;

        if(PIN(acc_time) > period) {
          PIN(inertia_sum) /= PIN(acc_time);
        }

        if(PIN(vel_time) > period) {
          PIN(friction_sum) /= PIN(vel_time);
          PIN(damping_sum) /= PIN(vel_time);
        }

        if(PIN(time) > period) {
          PIN(offset_sum) /= PIN(time);
        }
      }
      break;

    case 13:  // j, d, f
      PIN(pos) += PIN(vel_cmd) * period + PIN(acc_cmd) * period * period / 2.0;
      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) += PIN(acc_cmd) * period;
      to_go        = PIN(target) - PIN(pos);
      time_to_go   = sqrtf(2.0 * ABS(to_go) / PIN(max_acc));
      acc          = PIN(max_acc) * SIGN(to_go);
      vel          = acc * time_to_go;
      vel          = LIMIT(vel, PIN(max_vel));
      acc          = (vel - PIN(vel_cmd)) / period;
      PIN(acc_cmd) = LIMIT(acc, PIN(max_acc));

      if(ABS(PIN(max_pos) - PIN(pos)) < 0.1) {
        PIN(target) = PIN(min_pos);
      } else if(ABS(PIN(min_pos) - PIN(pos)) < 0.1) {
        PIN(target) = PIN(max_pos);
      }

      PIN(inertia) += period / PIN(ji) * PIN(fb_torque) * PIN(acc_cmd) * period;
      PIN(damping) += period / PIN(di) * PIN(fb_torque) * PIN(vel_cmd) * period;
      PIN(friction) += period / PIN(fi) * PIN(fb_torque) * SIGN(PIN(vel_cmd)) * period;
      PIN(offset) += period / PIN(li) * PIN(fb_torque) * period;

      PIN(inertia)  = CLAMP(PIN(inertia), 0.000005, 50.0);
      PIN(damping)  = CLAMP(PIN(damping), 0.0, 100.0);
      PIN(friction) = CLAMP(PIN(friction), 0.0, 100.0);
      PIN(offset)   = CLAMP(PIN(offset), -100.0, 100.0);


      PIN(timer) += period;
      if(PIN(timer) > 90.0) {
        PIN(timer)   = 0.0;
        PIN(acc_cmd) = 0.0;
        PIN(vel_cmd) = 0.0;
        PIN(amp)     = 0.0;

        PIN(state) = 1.4;
      }
      break;
  }
}


hal_comp_t idm_comp_struct = {
    .name      = "idm",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,  //sizeof(struct idm_ctx_t),
    .pin_count = sizeof(struct idm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
