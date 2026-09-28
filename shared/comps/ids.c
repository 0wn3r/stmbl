#include "ids_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `ids` tunes the gains of the `pid` position/velocity loop (`pos_bw`, `vel_bw`, `vel_d`) by moving the axis back and forth between two positions and searching, one gain at a time, for the lowest tracking-error cost. It runs on the F4 board and is loaded by the `id_pid` template, normally as the last identification step after `id_mot` (and `id_sys` if a load is coupled).
*
* ## Component Explanation
*
* 1. **Before you start**:
* - `id_mot` (and `id_sys`) should be done and their results (`conf0.j`, `conf0.j_sys`, `conf0.d`, `conf0.f`, `conf0.o`) in the config, since the pid feedforward and gain scaling use them. `conf0.cur_bw` must be set: it limits the velocity bandwidth.
* - The axis travels between `min_pos` and `max_pos` (default -10 and +10 rad around feedback position 0) at up to `max_vel` (100 rad/s) and `max_acc` (1000 rad/s^2). These defaults are aggressive; reduce them if the machine cannot do that.
*
* 2. **How to run it**:
* - At the console, `link id_pid`. The template loads `ids` and wires `ids0.en = fault0.en_out`, the trajectory into `pid0.pos_ext_cmd` / `vel_ext_cmd` / `acc_ext_cmd`, `pid0.pos_bw` / `vel_bw` / `vel_d` from this component, `pid0.pos_error` / `vel_error` back into it and `ids0.cur_bw = conf0.cur_bw`. It sets `conf0.max_pos_error = 0`, `conf0.max_sat = 10` and `conf0.vel_g = 1`, and puts `pos_cmd`, `vel_cmd`, `min_cost` and the three gains on the scope waves.
* - Enable the drive. With `auto_step >= 1` (default) the search starts at once; with `auto_step = 0` the console asks for `ids0.state = 1.2` first.
* - Watch `min_cost` and the gains on the scope. When it is done the console prints `conf0.pos_bw`, `conf0.vel_bw` and `conf0.vel_d`: append them to the config and save.
*
* 3. **Gain search (`state`)**:
* - `0`: idle. While here the rt function resets `pos_bw` 10, `vel_bw` 100, `vel_d` 10, `param` 0, `step` 0.1, `rep` 1, the limits `max_params[0] = cur_bw / 2`, `max_params[1] = 1`, and seeds `min_cost` and `cost` with `(max_pos - min_pos)^2 / max_vel * 100`. When `en` goes high it goes to `1.0`; `en` low returns to `0` from any state.
* - `1.0` -> `1.1` (nrt): sets `target = max_pos`; with `auto_step >= 1` it goes straight to `1.2`, otherwise it waits in `1.1`.
* - `1.2` (rt): a trapezoidal trajectory (`pos`, `vel`, `acc`) moves to `max_pos` and back to `min_pos` in cycles of `2 * (|max_pos - min_pos| / max_vel + 2 * max_vel / max_acc)` (0.8 s with the defaults). `pos_cmd` is `pos` wrapped to +-pi, `vel_cmd` / `acc_cmd` are scaled by `ff`. Over each cycle a cost is integrated:
* ```c
* cost += (kp * |pos_error| + ks * pos_error^2 + kv * vel_error^2) * period;
* ```
* - At the end of each cycle `min_cost = min(cost, min_cost)` and one gain, selected by `param`, is changed (coordinate search). `params[0]` is `vel_bw`, `params[1]` is `1 / vel_d`, `params[2]` is `pos_bw`:
* - if the gain is above its limit it is clamped and the search moves to the next gain. Limits: `vel_bw <= cur_bw / 2`, `1 / vel_d <= 1` (so `vel_d >= 1`), `pos_bw <= 2 * vel_bw`.
* - else if `cost > min_cost * kt` (kt default 1.2) the gain is multiplied by `kd` (0.7) and the search moves to the next gain.
* - else the gain is multiplied by `1 + step` and the cycle repeats.
* - Moving to the next gain sets `min_cost` to 10 times the last cost, so the next gain's first step is always accepted.
* - When all three gains are done the feedforward outputs are zeroed and the state goes to `1.3`; the nrt function prints the results and goes to `1.4` (done). The trajectory stays where it stopped until `en` goes low.
*
* {{% hint warning %}}
* The rt function overwrites `step` (to 0.1, nrt_init sets 0.25) and `rep` while the state is 0, so `step` can only be changed after enabling. `rep`, `freq` and `amp` are unused. The search starts `pos` from wherever the previous run left it (0 after boot) rather than from the actual position, so the first move may start with a position step. The first cycle's cost starts at the seed value instead of 0.
* {{% /hint %}}
*/

HAL_COMP(ids);

HAL_PIN(en);  // *input*, enable; high starts the gain search, low aborts (state -> 0)

HAL_PIN(state);  // *input/output*, 0 off, 1.0/1.1 start/wait, 1.2 gain search, 1.3 print, 1.4 done
HAL_PIN(param);  // *output*, gain currently searched: 0 vel_bw, 1 1/vel_d, 2 pos_bw
HAL_PIN(step);   // *parameter*, relative gain increase per cycle, reset to 0.1 while disabled
HAL_PIN(rep);    // *output*, unused, reset to 1 while disabled

HAL_PIN(freq);     // *parameter*, unused
HAL_PIN(amp);      // *parameter*, unused
HAL_PIN(min_pos);  // *parameter*, lower end of the travel (rad), default -10
HAL_PIN(max_pos);  // *parameter*, upper end of the travel (rad), default 10
HAL_PIN(max_vel);  // *parameter*, test velocity (rad/s), default 100
HAL_PIN(max_acc);  // *parameter*, test acceleration (rad/s^2), default 1000

HAL_PIN(pos);      // *output*, unwrapped trajectory position (rad)
HAL_PIN(vel);      // *output*, trajectory velocity (rad/s)
HAL_PIN(acc);      // *output*, trajectory acceleration (rad/s^2)
HAL_PIN(pos_cmd);  // *output*, position command, pos wrapped to +-pi (rad), to pid0.pos_ext_cmd
HAL_PIN(vel_cmd);  // *output*, velocity feedforward vel * ff (rad/s), to pid0.vel_ext_cmd
HAL_PIN(acc_cmd);  // *output*, acceleration feedforward acc * ff (rad/s^2), to pid0.acc_ext_cmd

HAL_PIN(pos_error);  // *input*, position error from pid0.pos_error (rad)
HAL_PIN(vel_error);  // *input*, velocity error from pid0.vel_error (rad/s)

HAL_PIN(pos_bw);  // *output*, position bandwidth under test, to pid0.pos_bw, result for conf0.pos_bw
HAL_PIN(vel_bw);  // *output*, velocity bandwidth under test, to pid0.vel_bw, result for conf0.vel_bw
HAL_PIN(vel_d);   // *output*, velocity loop damping under test, to pid0.vel_d, result for conf0.vel_d
HAL_PIN(cur_bw);  // *input*, current loop bandwidth, conf0.cur_bw, limits vel_bw to cur_bw / 2

HAL_PIN(ff);  // *parameter*, feedforward scale for vel_cmd and acc_cmd, default 1
HAL_PIN(kp);  // *parameter*, cost weight of abs(pos_error), default 1
HAL_PIN(ks);  // *parameter*, cost weight of pos_error^2, default 0
HAL_PIN(kv);  // *parameter*, cost weight of vel_error^2, default 1
HAL_PIN(kt);  // *parameter*, cost ratio above which a gain step is rejected, default 1.2
HAL_PIN(kd);  // *parameter*, factor applied to a rejected gain, default 0.7

HAL_PINA(params, 3);      // *output*, gains searched: 0 vel_bw, 1 1/vel_d, 2 pos_bw
HAL_PINA(max_params, 3);  // *output*, limits: cur_bw/2, 1.0, 2 * vel_bw

HAL_PIN(target);     // *output*, end point the trajectory is moving to (rad)
HAL_PIN(cost);       // *output*, cost of the current cycle
HAL_PIN(min_cost);   // *output*, lowest cost of the current gain
HAL_PIN(auto_step);  // *parameter*, >= 1 starts the search without waiting for state = 1.2, default 1

HAL_PIN(timer);  // *output*, time in the current cycle (s)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct ids_ctx_t * ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;
  PIN(pos_bw)                = 10.0;
  PIN(vel_bw)                = 100.0;
  PIN(vel_d)                 = 10.0;
  PINA(params, 0)            = PIN(vel_bw);
  PINA(params, 1)            = 1.0 / PIN(vel_d);
  PINA(params, 2)            = PIN(pos_bw);

  PIN(kp) = 1.0;
  PIN(ks) = 0.0;
  PIN(kv) = 1.0;
  PIN(kt) = 1.2;
  PIN(kd) = 0.7;

  PIN(ff) = 1.0;

  PIN(max_vel) = 100.0;
  PIN(max_acc) = 1000.0;
  PIN(min_pos) = -10.0;
  PIN(max_pos) = 10.0;

  PIN(param) = 0.0;
  PIN(step)  = 0.25;
  PIN(rep)   = 2.0;

  PIN(auto_step) = 1.0;
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct ids_ctx_t * ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      break;

    case 10:
      PIN(state)  = 1.1;
      PIN(target) = PIN(max_pos);

      if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Tune PPI bandwidth and damping\n");
        printf("the motor will move\n");
        printf("ids0.state = 1.2 <font color='green'>to start</font>\n");
      }
      break;

    case 13:
      printf("conf0.pos_bw = %f <font color='green'># append to config</font>\n", PIN(pos_bw));
      printf("conf0.vel_bw = %f <font color='green'># append to config</font>\n", PIN(vel_bw));
      printf("conf0.vel_d = %f <font color='green'># append to config</font>\n", PIN(vel_d));
      printf("done\n");
      PIN(state) = 1.4;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct ids_ctx_t * ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(acc_cmd) = 0.0;
      PIN(vel_cmd) = 0.0;
      PIN(acc)     = 0.0;
      PIN(vel)     = 0.0;

      PIN(param) = 0.0;
      PIN(step)  = 0.1;
      PIN(rep)   = 1.0;

      PIN(pos_bw)     = 10.0;
      PIN(vel_bw)     = 100.0;
      PIN(vel_d)      = 10.0;
      PINA(params, 0) = PIN(vel_bw);
      PINA(params, 1) = 1.0 / PIN(vel_d);
      PINA(params, 2) = PIN(pos_bw);

      PINA(max_params, 0) = PIN(cur_bw) / 2.0;
      PINA(max_params, 1) = 1.0;

      PIN(min_cost) = (PIN(max_pos) - PIN(min_pos)) * (PIN(max_pos) - PIN(min_pos)) / PIN(max_vel) * 100.0;
      PIN(cost)     = PIN(min_cost);

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:
      PIN(pos) += PIN(vel) * period + PIN(acc) * period * period / 2.0;
      PIN(vel) += PIN(acc) * period;
      float to_go      = PIN(target) - PIN(pos);
      float time_to_go = sqrtf(2.0 * ABS(to_go) / PIN(max_acc));
      float acc        = PIN(max_acc) * SIGN(to_go);
      float vel        = acc * time_to_go;
      vel              = LIMIT(vel, PIN(max_vel));
      acc              = (vel - PIN(vel)) / period;

      if(time_to_go < period) {
        time_to_go = 0.0;
        to_go      = 0.0;
        vel        = 0.0;
        acc        = 0.0;
        PIN(pos)   = PIN(target);
        PIN(vel)   = 0.0;
        PIN(acc)   = 0.0;
      }

      PIN(acc) = LIMIT(acc, PIN(max_acc));

      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) = PIN(vel) * PIN(ff);
      PIN(acc_cmd) = PIN(acc) * PIN(ff);

      PIN(cost) += ABS(PIN(pos_error)) * PIN(kp) * period;
      PIN(cost) += PIN(pos_error) * PIN(pos_error) * PIN(ks) * period;
      PIN(cost) += PIN(vel_error) * PIN(vel_error) * PIN(kv) * period;


      PIN(timer) += period;
      if(PIN(timer) < (ABS(PIN(max_pos) - PIN(min_pos)) / PIN(max_vel) + 2.0 * PIN(max_vel) / PIN(max_acc))) {
        PIN(target) = PIN(max_pos);
      } else {
        PIN(target) = PIN(min_pos);
      }
      if(PIN(timer) > 2.0 * (ABS(PIN(max_pos) - PIN(min_pos)) / PIN(max_vel) + 2.0 * PIN(max_vel) / PIN(max_acc))) {
        PIN(timer) = 0.0;
      }

      if(PIN(timer) == 0.0) {
        PIN(min_cost) = MIN(PIN(cost), PIN(min_cost));

        PINA(max_params, 2) = PINA(params, 0) * 2.0;

        if(PINA(params, (int)PIN(param)) > PINA(max_params, (int)PIN(param))) {
          PINA(params, (int)PIN(param)) = PINA(max_params, (int)PIN(param));
          PIN(min_cost)                 = PIN(cost) * 10.0;
          PIN(param)
          ++;
        } else if(PIN(cost) > PIN(min_cost) * PIN(kt)) {
          PINA(params, (unsigned int)PIN(param)) *= PIN(kd);
          PIN(min_cost) = PIN(cost) * 10.0;
          PIN(param)
          ++;
        } else {
          PINA(params, (unsigned int)PIN(param)) *= 1.0 + PIN(step);
        }

        PIN(cost) = 0.0;
      }

      PIN(pos_bw) = PINA(params, 2);
      PIN(vel_bw) = PINA(params, 0);
      PIN(vel_d)  = 1.0 / PINA(params, 1);

      if(PIN(param) > 2.0) {
        PIN(acc_cmd) = 0.0;
        PIN(vel_cmd) = 0.0;
        PIN(param)   = 0.0;

        PIN(state) = 1.3;
      }
      break;
  }
}


hal_comp_t ids_comp_struct = {
    .name      = "ids",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,  //sizeof(struct ids_ctx_t),
    .pin_count = sizeof(struct ids_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};