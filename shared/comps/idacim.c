#include "idacim_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `idacim` identifies an AC induction motor: stator resistance `r` (with the inverter dead time voltage `drop` separated from it), inductance `l` from the current time constant, pole pairs and direction. It runs on the F4 board, drives `hv0` directly and is loaded by the `id_acim` template. Slip, rated values and the other `acim_ttc` parameters are not measured.
*
* ## Component Explanation
*
* 1. **Before you start**:
* - `test_cur` (peak A, default 3) must be under `conf0.max_ac_cur`; the chord r fit wants at least about 5 A (below that the dead time bias is understated). `test_vel` (electrical rad/s, default 50) sets the field speed of the pole pair test.
* - If you have a four wire measurement of the winding resistance, set `idacim0.r_known` to it: r is then taken as given and only the dead time drop is read.
* - The template does not touch `hv0.drop_k`; keep it at its default 0 (no dead time compensation) while identifying.
* - While the state is 0 the nrt function resets `r` 0.1 ohm, `l` 1 mH, `l_half` 0.1 s, `drop` 0, `out_rev` 0, `cur_bw` 1, so set `l_half` only after enabling. `test_cur`, `test_vel`, `r_known` and `drop_slope` keep their values.
*
* 2. **How to run it**:
* - At the console, `link id_acim`. The template loads `idacim` and wires `idacim0.en = fault0.en_out`, sets `fault0.pos_error = 0` and `pid0.en = 0`, connects `hv0.en`, `cur_bw`, `cmd_mode`, `d_cmd`, `q_cmd`, `pos`, `rev`, `r`, `l` to this component (`hv0.lq = 0`) and `hv0.ud_fb`, `id_fb`, `pwm_volt`, `dc_volt` and `vel1.vel` back into it. `hv0.r` / `hv0.l` are this component's `r` / `l`, so they are the current loop's plant model during the tests.
* - Block the rotor and enable the drive. The state goes `0` -> `1.0` -> `1.1` and the console asks for `idacim0.state = 1.2`. `en` low returns to `0` from any state.
* - The r and l tests run (6 s) and state `1.4` prints `conf0.r` and `conf0.l` (or why l failed) and goes to `2.0`, which asks to unblock the rotor. Set `idacim0.state = 2.2`; the rotor turns for 3 s and state `2.4` prints `conf0.polecount` and, if reversed, `conf0.out_rev = 1`. Append the printed lines to the config and save.
*
* 3. **Resistance (`state` 1.2), rotor blocked**:
* - Current mode, `cur_bw` 1, current on the d axis at `com_pos = 0`: 2 s at `test_cur`, then 2 s at `test_cur / 2`. `id_fb` / `ud_fb` of each dwell are low-passed (0.001 per tick) into `tmp2` / `tmp3` (top) and `tmp0` / `tmp1` (half). Meanwhile `r` tracks `ud_fb / id_fb` so the current loop can build up voltage at all.
* - `ud_fb` contains `r * id + 4/3 * drop` (dead time; at angle 0 the phase currents are id, -id/2, -id/2). After 4 s `fit_di = tmp2 - tmp0` and the chord `r_2p = (tmp3 - tmp1) / fit_di` (only if `fit_di > 0.01` A) are computed, and `r` is taken from one of two modes:
* - `r_known > 0`: `r = r_known`, `drop = 0.75 * (tmp3 - r * tmp2)`. Needs the top dwell to reach half of `test_cur`.
* - default: `r = r_2p - r_bias` with `r_bias = drop_slope * dc_volt / test_cur` (0 if `dc_volt` is not wired), `drop` as above. The dead time drop rises roughly as ln(i), which biases the chord; `drop_slope` (0.0039 ohm A per volt) was fitted on one bridge.
* - `r_ok` is set if the chosen mode produced a value. Then `avg_test_volt = r * test_cur + 4/3 * drop` (limited to `pwm_volt / 2`) and the state goes to `1.3`, or straight to `1.4` if `r_ok` is 0.
*
* 4. **Inductance (`state` 1.3)**:
* - Voltage mode, `cur_bw` 1: `d_cmd` is a square wave between `avg_test_volt` and `avg_test_volt - r * test_cur / 2` with half period `l_half`, for 2 s. Both levels keep the current on the same side of zero, so the dead time drop cancels.
* - Edges are detected on the returned `ud_fb` (same packet delay as `id_fb`). For each high half the current is integrated (trapezoid) and the time constant is obtained by the area method:
* ```c
* tau = (i_b * T - integral(i dt)) / (i_b - i_a);
* ```
* - `i_a` / `i_b` are the settled currents of the last quarter of the previous low and this high half; the step must move the current by more than 5 % of `test_cur`. The first cycle is discarded.
* - With at least 3 cycles and `10 * period <= tau <= l_half / 8`: `l = tau * r`, `l_ok = 1`; otherwise `l = 0`.
* - `1.4` (nrt): if `r_ok`, prints `conf0.r`, `conf0.l` (or why l failed: no transient, check that `idacim0.ud_fb` is wired; too slow, raise `l_half` above `8 * tau`; too fast, use an LCR meter) and the measured `drop` for information only, then goes to `2.0`. If not, prints why the r test failed and returns to state 0 (with `en` still high, straight back to the 1.1 prompt).
*
* 5. **Pole pairs and direction (`state` 2.2), rotor free**:
* - For 3 s: current mode, `cur_bw` 100, `d_cmd = test_cur`, and `com_pos` advances open loop at `test_vel` electrical rad/s, dragging the rotor along. While `|vel_fb| > 0.1` rad/s, `pp` is low-passed (0.005 per tick) from `test_vel / vel_fb`. At the end a negative `pp` sets `out_rev = 1`, and `pp` is rounded to an integer.
* - `2.4` (nrt) prints the result and goes to `3.0` (done).
*
* {{% hint warning %}}
* `drop` is a voltage per phase for information only; it is not a value for `hv0.drop_k` (a fraction). The induction rotor slips behind the rotating field in the pole pair test, so `test_vel / vel_fb` reads slightly high before rounding, and `pp` is not reset between runs, so its filter starts from the previous result. The l test's detector state (cycle count, tau sum) is only cleared by `rt_start`, not when state 1.3 is entered, so a second r/l run without restarting the rt system averages in the previous run's cycles.
* {{% /hint %}}
*/

HAL_COMP(idacim);

HAL_PIN(d_cmd);     // *output*, d axis command to hv0.d_cmd, current (A) or voltage (V) depending on cmd_mode
HAL_PIN(q_cmd);     // *output*, q axis command to hv0.q_cmd, always 0
HAL_PIN(com_pos);   // *output*, commutation angle to hv0.pos (rad), 0 except in the pole pair test
HAL_PIN(cmd_mode);  // *output*, to hv0.cmd_mode, 0 = voltage, 1 = current
HAL_PIN(en);        // *input*, enable, from fault0.en_out; low aborts (state -> 0)
HAL_PIN(en_out);    // *output*, enables hv0 while a test runs

HAL_PIN(id_fb);  // *input*, d axis current from hv0.id_fb (A)
HAL_PIN(ud_fb);  // *input*, d axis voltage from hv0.ud_fb (V)

HAL_PIN(state);  // *input/output*, 0 off, 1.1 wait, 1.2 r test, 1.3 l test, 1.4 print, 2.1 wait, 2.2 pp test, 2.4 print, 3 done
HAL_PIN(timer);  // *output*, time in the current test (s)

HAL_PIN(r);           // *output*, measured resistance (ohm), to hv0.r, result for conf0.r
HAL_PIN(l);           // *output*, measured inductance tau * r (H), to hv0.l, 0 = not measured, result for conf0.l
HAL_PIN(l_half);      // *parameter*, l test half period (s), reset to 0.1 while disabled
HAL_PIN(tau);         // *output*, measured current time constant l/r (s)
HAL_PIN(l_ok);        // *output*, 1 = l is a measurement, 0 = it is not
HAL_PIN(drop);        // *output*, measured dead time voltage per phase at the top dwell (V), information only
HAL_PIN(r_known);     // *parameter*, known winding resistance (ohm), 0 = fit it, default 0
HAL_PIN(fit_di);      // *output*, current difference of the two dwells (A)
HAL_PIN(r_2p);        // *output*, two dwell chord slope (ohm), 0 = the fit did not run
HAL_PIN(drop_slope);  // *parameter*, chord dead time bias (ohm A per volt of dc link), default 0.0039
HAL_PIN(r_bias);      // *output*, bias subtracted from the chord to get r (ohm)
HAL_PIN(r_ok);        // *output*, 1 = this run produced a resistance

HAL_PIN(pp);       // *output*, measured pole pairs, result for conf0.polecount
HAL_PIN(out_rev);  // *output*, 1 = motor turns backwards, to hv0.rev, result for conf0.out_rev

HAL_PIN(test_cur);  // *parameter*, test current (A peak), default 3
HAL_PIN(test_vel);  // *parameter*, field speed of the pole pair test (electrical rad/s), default 50

HAL_PIN(vel_fb);  // *input*, mechanical velocity from vel1.vel (rad/s)

HAL_PIN(pwm_volt);  // *input*, usable voltage from hv0.pwm_volt (V), limits avg_test_volt
HAL_PIN(dc_volt);   // *input*, dc link voltage from hv0.dc_volt (V), scales r_bias

HAL_PIN(cur_bw);  // *output*, current loop bandwidth to hv0.cur_bw

HAL_PIN(tmp0);           // *output*, filtered id_fb of the half current dwell (A)
HAL_PIN(tmp1);           // *output*, filtered ud_fb of the half current dwell (V)
HAL_PIN(tmp2);           // *output*, filtered id_fb of the full current dwell (A)
HAL_PIN(tmp3);           // *output*, filtered ud_fb of the full current dwell (V)
HAL_PIN(avg_test_volt);  // *output*, voltage that holds test_cur incl. dead time, high level of the l test (V)

// l test state
struct idacim_ctx_t {
  uint8_t was_high;  // last observed level of ud_fb
  uint8_t have_a;    // a settled low value has been captured
  uint16_t end_n;    // samples in the settled tail of this half
  uint16_t cyc;      // observed cycles, the first is discarded
  uint16_t tau_n;    // cycles that yielded a time constant
  float t_cmd;       // phase of the commanded square wave
  float t_in;        // time since the last observed edge
  float t_hi;        // measured length of the high half
  float area;        // integral of id_fb across the high half
  float prev;        // previous id_fb, for the trapezoid
  float end_sum;     // settled tail accumulator
  float i_a;         // settled low current
  float tau_sum;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;
  PIN(r_known)                  = 0.0;
  PIN(drop_slope)               = 0.0039;
  PIN(test_cur)                 = 3.0;
  PIN(test_vel)                 = 50.0;
  PIN(cur_bw)                   = 1.0;
}

// ctx survives a stop
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx = (struct idacim_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idacim_ctx_t));
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(r)       = 0.1;
      PIN(l)       = 0.001;
      PIN(l_half)  = 0.1;
      PIN(drop)    = 0.0;
      PIN(out_rev) = 0.0;
      PIN(cur_bw)  = 1.0;
      break;

    case 10:  // r, l
      PIN(state)    = 1.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(tmp0)     = 0.0;
      PIN(tmp1)     = 0.0;
      PIN(tmp2)     = 0.0;
      PIN(tmp3)     = 0.0;

      printf("Measure r, l\n");
      printf("<font color='green'>block the rotor</font>\n");
      printf("idacim0.state = 1.2 <font color='green'>to start</font>\n");
      break;

    case 14:
      if(PIN(r_ok) > 0.0) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        if(PIN(l_ok) > 0.0) {
          printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(l));
          printf("<font color='green'># from tau = %f ms against the r above.</font>\n", PIN(tau) * 1000.0);
        } else if(PIN(tau) <= 0.0) {
          printf("<font color='red'>l not measured</font>: no usable transient\n");
          printf("check that the bridge is enabled and idacim0.ud_fb is wired\n");
        } else if(PIN(tau) > PIN(l_half) / 8.0) {
          printf("<font color='red'>l not measured</font>: tau = %f ms needs a longer half period\n", PIN(tau) * 1000.0);
          printf("raise idacim0.l_half above %f s and rerun\n", PIN(tau) * 8.0);
        } else {
          printf("<font color='red'>l not measured</font>: tau = %f ms is too fast to time here\n", PIN(tau) * 1000.0);
          printf("use an LCR meter: line to line at 1 kHz, halved\n");
        }
        printf("<font color='green'># dead time %f V per phase at the %f A dwell, %f V link\n", PIN(drop), PIN(test_cur), PIN(dc_volt));
        if(PIN(r_known) > 0.0) {
          printf("# read against the r you gave.</font>\n");
        } else {
          printf("# r is the chord %f less its dead time bias %f (idacim0.drop_slope).\n", PIN(r_2p), PIN(r_bias));
          printf("# below about 5 A the bias is understated; a four wire r in\n");
          printf("# idacim0.r_known is better.</font>\n");
        }
      } else {
        if(PIN(r_known) > 0.0) {
          printf("<font color='red'>r read failed</font>: the top dwell did not reach half of %f A\n", PIN(test_cur));
        } else {
          printf("<font color='red'>r fit failed</font>: the two dwells differ by %f A\n", PIN(fit_di));
        }
        printf("nothing below is measured, do not append it\n");
        printf("check that idacim0.test_cur (%f) is under conf0.max_ac_cur\n", PIN(test_cur));
      }
      // walk on only if r worked: hv0.r is this pin
      PIN(state) = PIN(r_ok) > 0.0 ? 2.0 : 0.0;
      break;

    case 20:  // pp
      PIN(state)    = 2.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;

      printf("Measure polepairs\n");
      printf("<font color='green'>unblock the rotor, it will move</font>\n");
      printf("idacim0.state = 2.2 <font color='green'>to start</font>\n");
      break;

    case 24:
      printf("conf0.polecount = %f <font color='green'># append to config</font>\n", PIN(pp));
      if(PIN(out_rev) > 0.0) {
        printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
      }
      printf("done\n");
      PIN(state) = 3.0;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx        = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:  // r -- rotor blocked, current control: dwell at two currents and
              // fit the voltage the current loop needs to sustain them.
              // com_pos is held at 0 throughout -- an induction motor has no rotor
              // flux to align to, so the injection axis is arbitrary.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 1.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;

      // ud = r * id + 4/3 * drop: at angle 0 the phase currents are id,
      // -id/2, -id/2, and the three per phase dead time drops land 4/3 on d.
      // Two dwells split r (the slope) from the drop. Full current first, so
      // the first dwell aligns what the second holds.
      if(PIN(timer) < 2.0) {
        PIN(d_cmd) = PIN(test_cur);
        PIN(tmp2)  = PIN(tmp2) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp3)  = PIN(tmp3) * 0.999 + PIN(ud_fb) * 0.001;
      } else {
        PIN(d_cmd) = PIN(test_cur) * 0.5;
        PIN(tmp0)  = PIN(tmp0) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp1)  = PIN(tmp1) * 0.999 + PIN(ud_fb) * 0.001;
      }

      // hv0.r is this pin, the loop's plant model: the ud/id ratio bootstraps
      // it so current flows at cur_bw 1; the fit replaces it
      PIN(r) = PIN(r) * 0.99 + PIN(ud_fb) / MAX(PIN(id_fb), 0.01) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 4.0) {
        // the filters are linear and alike, so the fit holds on the filtered
        // pair even where neither dwell reaches its command
        float di    = PIN(tmp2) - PIN(tmp0);
        PIN(fit_di) = di;
        PIN(r_2p) = di > 0.01 ? MAX((PIN(tmp3) - PIN(tmp1)) / di, 0.001) : 0.0;

        float r_ok = 0.0;

        if(PIN(r_known) > 0.0) {
          // the drop is read at the top dwell, which has to be near its command
          if(PIN(tmp2) > PIN(test_cur) * 0.5) {
            PIN(r)    = PIN(r_known);
            PIN(drop) = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
            r_ok      = 1.0;
          }
        } else if(di > 0.01) {
          // The dead time drop rises as K ln(i), which biases the chord by
          // 8/3 K ln2 / test_cur. drop_slope = 8/3 ln2 K / Vdc, 0.0039 ohm A
          // per volt on the bridge it was fitted on. Skipped without dc_volt.
          PIN(r_bias) = PIN(dc_volt) > 1.0 ? PIN(drop_slope) * PIN(dc_volt) / MAX(PIN(test_cur), 0.1) : 0.0;
          PIN(r)      = MAX(PIN(r_2p) - PIN(r_bias), 0.001);
          PIN(drop)   = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
          r_ok        = 1.0;
        }

        PIN(r_ok) = r_ok;

        // the l test needs the voltage that holds test_cur, dead time
        // included, and never one built from a failed measurement
        if(r_ok > 0.0) {
          PIN(avg_test_volt) = PIN(r) * PIN(test_cur) + 4.0 / 3.0 * PIN(drop);
          PIN(avg_test_volt) = LIMIT(PIN(avg_test_volt), PIN(pwm_volt) / 2.0);
        } else {
          PIN(avg_test_volt) = 0.0;
        }

        PIN(timer)  = 0.0;
        PIN(state)  = r_ok > 0.0 ? 1.3 : 1.4;
        PIN(d_cmd)  = 0.0;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;
        PIN(tmp2)   = 0.0;
        PIN(tmp3)   = 0.0;
      }
      break;

    case 13: {  // l, from the current's relaxation time constant
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      // Step between two currents of the same sign and take the relaxation
      // time constant by the area method:
      //
      //   i(t) = i_f + (i_a - i_f) exp(-t/tau),  tau = l/r
      //   => tau = (i_b * T - integral i dt) / (i_b - i_a)     for T >> tau
      //
      // The dead time drop is a constant offset while the current keeps its
      // sign, so it cancels out of a time constant.
      float half = MAX(PIN(l_half), 0.01);
      float v_hi = PIN(avg_test_volt);
      float v_lo = PIN(avg_test_volt) - PIN(r) * PIN(test_cur) * 0.5;
      float mid  = (v_hi + v_lo) * 0.5;

      // time the window off the step as it comes back in ud_fb: it shares
      // id_fb's packet, so the pipeline delay cancels
      uint8_t hi = PIN(ud_fb) > mid;

      if(hi && !ctx->was_high) {  // observed rising edge
        if(ctx->end_n > 0) {
          ctx->i_a    = ctx->end_sum / (float)ctx->end_n;
          ctx->have_a = 1;
        }
        ctx->end_sum = 0.0;
        ctx->end_n   = 0;
        ctx->area    = 0.0;
        ctx->t_hi    = 0.0;
        ctx->t_in    = 0.0;
        ctx->prev    = PIN(id_fb);
      } else if(!hi && ctx->was_high) {  // observed falling edge
        if(ctx->end_n > 0 && ctx->have_a) {
          float i_b = ctx->end_sum / (float)ctx->end_n;
          float di  = i_b - ctx->i_a;
          // the step has to have moved the current, and one whole cycle has to
          // have gone by, before any of it means anything
          if(di > PIN(test_cur) * 0.05 && ctx->t_hi > 0.0 && ctx->cyc > 0) {
            float tau = (i_b * ctx->t_hi - ctx->area) / di;
            if(tau > 0.0) {
              ctx->tau_sum += tau;
              ctx->tau_n++;
            }
          }
        }
        ctx->end_sum = 0.0;
        ctx->end_n   = 0;
        ctx->t_in    = 0.0;
        ctx->cyc++;
      }

      if(hi) {
        // trapezoid: a left hand sum is about a percent of tau off
        ctx->area += (PIN(id_fb) + ctx->prev) * 0.5 * period;
        ctx->t_hi += period;
      }

      ctx->t_in += period;
      if(ctx->t_in > half * 0.75) {  // settled tail of whichever half we are in
        ctx->end_sum += PIN(id_fb);
        ctx->end_n++;
      }

      ctx->prev     = PIN(id_fb);
      ctx->was_high = hi;

      // the commanded square wave runs on its own clock; what we measure
      // against is the echo, not this
      ctx->t_cmd += period;
      if(ctx->t_cmd >= half * 2.0) {
        ctx->t_cmd = 0.0;
      }
      PIN(d_cmd) = ctx->t_cmd < half ? v_lo : v_hi;

      PIN(timer) += period;
      if(PIN(timer) >= 2.0) {
        float tau = ctx->tau_n > 0 ? ctx->tau_sum / (float)ctx->tau_n : 0.0;
        float l_ok = 0.0;

        PIN(tau) = tau;

        // too slow for the half period truncates the tail, too fast leaves a
        // handful of samples
        if(ctx->tau_n >= 3 && tau >= 10.0 * period && tau <= half / 8.0) {
          PIN(l) = tau * PIN(r);
          l_ok   = 1.0;
        } else {
          PIN(l) = 0.0;
        }

        PIN(l_ok)   = l_ok;
        PIN(timer)  = 0.0;
        PIN(state)  = 1.4;
        PIN(d_cmd)  = PIN(avg_test_volt);
        PIN(en_out) = 0.0;
      }
      break;
    }

    case 22:  // pp -- rotor free to turn: hold q_cmd at 0 and ramp com_pos open-loop
              // at test_vel while injecting d_cmd, forcing the rotor to follow the
              // rotating stator field (same technique as a sensorless PMSM I/F
              // startup). Comparing the commanded electrical rate against the
              // measured mechanical rate gives pole pairs directly.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) += PIN(test_vel) * period;
      PIN(com_pos) = mod(PIN(com_pos));

      if(ABS(PIN(vel_fb)) > 0.1) {
        PIN(pp) = PIN(pp) * 0.995 + PIN(test_vel) / PIN(vel_fb) * 0.005;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 3.0) {
        PIN(timer) = 0.0;

        if(PIN(pp) < 0.0) {
          PIN(out_rev) = 1.0;
          PIN(pp) *= -1.0;
        }
        PIN(pp) = (int)(PIN(pp) + 0.5);

        PIN(en_out)   = 0.0;
        PIN(d_cmd)    = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 2.4;
      }
      break;
  }
}

hal_comp_t idacim_comp_struct = {
    .name      = "idacim",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idacim_ctx_t),
    .pin_count = sizeof(struct idacim_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
