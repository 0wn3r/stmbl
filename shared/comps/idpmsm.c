#include "idpmsm_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(idpmsm);

HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(com_pos);
HAL_PIN(cmd_mode);
HAL_PIN(en);
HAL_PIN(en_out);

HAL_PIN(id_fb);
HAL_PIN(iq_fb);
HAL_PIN(ud_fb);
HAL_PIN(uq_fb);
HAL_PIN(pos_fb);
HAL_PIN(vel_fb);

HAL_PIN(state);
HAL_PIN(timer);

HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(drop);
HAL_PIN(r_known);       // *parameter*, measured winding resistance, 0 = try to fit it
HAL_PIN(single_dwell);  // *parameter*, take r from the top dwell alone
HAL_PIN(fit_di);        // dwell current separation, 0 = r/drop fit did not run
HAL_PIN(r_1p);          // top dwell ratio, 0 = that dwell never reached its command
HAL_PIN(r_2p);          // two dwell chord slope, 0 = the fit did not run
HAL_PIN(drop_slope);    // *parameter*, the chord's dead time bias, ohm A per volt
HAL_PIN(r_bias);        // what was subtracted from the chord to get r
HAL_PIN(r_ok);          // this run produced a resistance

HAL_PIN(pp);
HAL_PIN(com_offset);
HAL_PIN(out_rev);

HAL_PIN(test_cur);
HAL_PIN(test_vel);
HAL_PIN(ki);
HAL_PIN(vel_bw);

HAL_PIN(pi);

HAL_PIN(pwm_volt);
HAL_PIN(dc_volt);

HAL_PIN(psi);

HAL_PIN(cur_bw);
HAL_PIN(cur_sum);
HAL_PIN(auto_step);

HAL_PIN(tmp0);
HAL_PIN(tmp1);
HAL_PIN(tmp2);
HAL_PIN(tmp3);
HAL_PIN(avg_test_volt);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idpmsm_ctx_t * ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;
  PIN(r_known)                  = 0.0;
  PIN(single_dwell)             = 0.0;
  PIN(drop_slope)               = 0.0039;
  PIN(test_cur)                 = 3.0;
  PIN(test_vel)                 = 50.0;
  PIN(ki)                       = 1.0;
  PIN(pi)                       = 1.0;
  PIN(vel_bw)                   = 20.0;
  PIN(cur_bw)                   = 1.0;
  PIN(auto_step)                = 4.2;
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idpmsm_ctx_t * ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(r)          = 0.1;
      PIN(l)          = 0.001;
      PIN(drop)       = 0.0;
      PIN(psi)        = 0.055;
      PIN(pp)         = 3.0;
      PIN(com_offset) = 0.0;
      PIN(out_rev)    = 0.0;
      PIN(cur_bw)     = 1.0;
      break;

    case 10:  // r
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

      if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Measure r, l\n");
        printf("the motor can move a bit\n");
        printf("idpmsm0.state = 1.2 <font color='green'>to start</font>\n");
      }
      break;

    case 14:
      // r_ok is set by whichever branch of the r test actually produced a
      // resistance, so the report, the l test and the state advance cannot
      // disagree about whether this run measured anything.
      if(PIN(r_ok) > 0.0) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(l));
        // Measurement, not a config line. hv0.drop and hv0.drop_k both feed the
        // same dt_drop in the f3's hv.c, and that compensation is not yet safe
        // to switch on: it is keyed on measured current, so once it exceeds the
        // real drop it drives the current it is reading. Setting either one in
        // a config has taken a drive to its overcurrent trip. Read the number,
        // do not append it.
        printf("<font color='red'>measured</font> drop = %f V at the %f A dwell\n", PIN(drop), PIN(test_cur));
        printf("<font color='green'># scales with the dc link. do NOT put hv0.drop or hv0.drop_k\n");
        printf("# in a config -- the compensation is not stable yet.</font>\n");
        if(PIN(r_known) > 0.0) {
          printf("<font color='green'># drop read at the %f A dwell against the r you gave.\n", PIN(test_cur));
          printf("# it scales with the dc link, remeasure if that changes.</font>\n");
        } else if(PIN(single_dwell) > 0.0) {
          printf("<font color='green'># r is the %f A dwell's ratio, so it is a resistance\n", PIN(test_cur));
          printf("# only as far as hv0.drop_k cancels the dead time. what\n");
          printf("# drop_k misses shows up here as 4/3 * residual / %f\n", PIN(test_cur));
          printf("# ohms -- raise idpmsm0.test_cur to shrink it.\n");
          printf("# hv0.drop is not measured in this mode and reads 0.\n");
          printf("# the chord fit on this same run gave %f. it should read\n", PIN(r_2p));
          printf("# high by roughly 1.2 / %f ohms, which is the dead time's\n", PIN(test_cur));
          printf("# rise with current leaking into the slope. a much wider\n");
          printf("# gap than that means drop_k is over compensating.</font>\n");
        } else {
          printf("<font color='green'># chord %f, less its dead time bias %f.\n", PIN(r_2p), PIN(r_bias));
          printf("# the bias is 8/3 * ln2 * K / test_cur and needs only K, the\n");
          printf("# drop's slope with current -- not its magnitude, which the\n");
          printf("# chord cancels. idpmsm0.drop_slope carries it.\n");
          if(!(PIN(dc_volt) > 1.0)) {
            printf("</font><font color='red'># no bias was subtracted: idpmsm0.dc_volt reads %f.\n", PIN(dc_volt));
            printf("# add \"idpmsm0.dc_volt = hv0.dc_volt\" to the config -- without\n");
            printf("# it r is the raw chord and reads high.</font><font color='green'>\n");
          }
          if(PIN(test_cur) < 5.0) {
            printf("</font><font color='red'># test_cur %f is too low for this correction:\n", PIN(test_cur));
            printf("# below about 5 A the drop stops following ln(i) and the\n");
            printf("# bias is understated. rerun higher.</font><font color='green'>\n");
          }
          printf("# cross check: measure r four wire, halve the phase to\n");
          printf("# phase reading, and set idpmsm0.r_known.</font>\n");
        }
      } else {
        // no usable estimate: either the two dwells read the same current so
        // there is no line to fit, or the top dwell never got near its
        // command so its ratio is not a resistance. Either way r, drop and
        // everything downstream of them are still at their init values. Say
        // so -- printing those as a result is worse than failing.
        if(PIN(r_known) > 0.0 || PIN(single_dwell) > 0.0) {
          printf("<font color='red'>r read failed</font>: the top dwell did not reach half of %f A\n", PIN(test_cur));
        } else {
          printf("<font color='red'>r fit failed</font>: the two dwells differ by %f A\n", PIN(fit_di));
        }
        printf("nothing below is measured, do not append it\n");
        printf("check that idpmsm0.test_cur (%f) is under conf0.max_ac_cur\n", PIN(test_cur));
      }
      // only walk on to the next test if this one worked. hv0.r is wired to
      // PIN(r), so carrying a rejected value forward would hand the next test
      // its plant model at cur_bw 100.
      PIN(state) = PIN(r_ok) > 0.0 ? 2.0 : 0.0;
      break;

    case 20:  // pp, out_rev, com_offset
      PIN(state) = 2.1;
      PIN(timer) = 0.0;
      //PIN(d_cmd) = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;

      if(PIN(auto_step) >= 2) {
        PIN(state) = 2.2;
      } else {
        printf("Measure com_offset, polepairs, out_rev\n");
        printf("the motor will move\n");
        printf("idpmsm0.state = 2.2 <font color='green'>to start</font>\n");
      }
      break;

    case 25:  // pp, out_rev, com_offset
      printf("conf0.polecount = %f <font color='green'># append to config</font>\n", PIN(pp));
      printf("conf0.mot_fb_offset = %f <font color='green'># append to config</font>\n", PIN(com_offset));
      if(PIN(out_rev) > 0.0) {
        printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
      }
      PIN(state) = 3.0;
      break;

    case 30:
      PIN(state)    = 3.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_sum)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(cur_bw)   = 250.0;

      if(PIN(auto_step) >= 3) {
        PIN(state) = 3.2;
      } else {
        printf("Measure torque constant\n");
        printf("the motor will move\n");
        printf("id0.state = 3.2 to start\n");
      }
      break;

    case 33:
      printf("conf0.psi = %f <font color='green'># append to config</font>\n", PIN(psi));
      printf("done\n");
      printf("continue with id_mot\n");

      PIN(state) = 3.4;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idpmsm_ctx_t * ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;

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

    case 12:  // r
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 1.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;

      // ud_fb is the current loop's voltage command, so it carries the
      // inverter's dead time error on top of the winding drop:
      //
      //   ud = r * id + 4/3 * drop
      //
      // The 4/3 is the injection pattern: at electrical angle 0 the phase
      // currents are iu = +id, iv = iw = -id/2, so the per phase dead time
      // corrections are +drop, -drop, -drop and (2*drop + drop + drop)/3
      // lands on the d axis. That term does not shrink with current -- at
      // 3 A with 2us of dead time it is an order of magnitude larger than
      // the winding drop -- so a single ud/id reading is mostly dead time
      // and raising test_cur barely helps. Dwelling at two currents splits
      // them: the slope is r, the intercept is what hv0.drop wants.
      //
      // The filter is a tenth the speed of the one the l test below uses.
      // That one tracks a per sample square wave, this one holds a steady
      // dwell: 0.001 at the 5 kHz rt rate is 200 ms, still settling ten
      // times over inside a 2 s dwell while averaging ten times as many
      // samples. r is the small difference of two large voltages, so it is
      // the one number here that wants every sample it can get.
      // full current first, reduced second. The other order parks the rotor
      // at a current that may be too weak to pull a loaded axis into
      // alignment, then doubles the torque and breaks it loose -- and at
      // cur_bw 1.0 the loop cannot reject the back emf of that swing, so the
      // current goes wherever the inductance takes it. Starting at test_cur
      // reproduces the single dwell test's startup, which is known to be
      // survivable, and stepping down afterwards leaves an aligned rotor
      // where it is.
      if(PIN(timer) < 2.0) {
        PIN(d_cmd) = PIN(test_cur);
        PIN(tmp2)  = PIN(tmp2) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp3)  = PIN(tmp3) * 0.999 + PIN(ud_fb) * 0.001;
      } else {
        PIN(d_cmd) = PIN(test_cur) * 0.5;
        PIN(tmp0)  = PIN(tmp0) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp1)  = PIN(tmp1) * 0.999 + PIN(ud_fb) * 0.001;
      }

      // keep the single point ratio running through both dwells. hv0.r is
      // wired to this pin, so it is the current loop's plant model, not just
      // an output: at cur_bw 1.0 the integral gain is cur_bw * r, which from
      // the 0.1 init is 0.1 V per A s and needs half a minute to reach the
      // ~11 V that clears the dead time. Nothing would flow inside a dwell.
      // The ratio bootstraps it -- with id near zero it reads high, which
      // lifts the feedforward, which starts the current, which settles the
      // ratio. The fit below then replaces it with the real resistance.
      PIN(r) = PIN(r) * 0.99 + PIN(ud_fb) / MAX(PIN(id_fb), 0.01) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 4.0) {
        // the filters are linear and both dwells use the same one, so the
        // fit holds on the filtered pair even though the loop is slow
        // enough that neither dwell reaches its commanded current.
        float di   = PIN(tmp2) - PIN(tmp0);
        PIN(fit_di) = di;

        // Both estimates, every run, so the report can show the spread even
        // though only one of them is used. The ratio needs a dwell that got
        // near its command: at a fraction of test_cur it divides a voltage
        // that is mostly dead time by a current that is mostly nothing.
        PIN(r_1p) = PIN(tmp2) > PIN(test_cur) * 0.5 ? MAX(PIN(tmp3) / PIN(tmp2), 0.001) : 0.0;
        PIN(r_2p) = di > 0.01 ? MAX((PIN(tmp3) - PIN(tmp1)) / di, 0.001) : 0.0;

        float r_ok = 0.0;

        if(PIN(r_known) > 0.0) {
          // Resistance supplied from a four wire measurement. Skip the fit:
          // separating r from the dead time needs curvature in u(i), and over
          // the current range the trip limit allows there is not enough of it
          // -- on one motor the fitted r came out 1.20, 0.81, 0.73 and 0.57
          // at four test currents against a meter's 0.685, sweeping through
          // the answer rather than settling on it. The dead time itself is
          // solid: read at the top dwell against a known r it reproduced to
          // 0.2% across two firmware builds and four runs.
          // the supplied r needs no chord, but the drop read against it is
          // still the top dwell's, so that dwell has to have got near its
          // command -- the same condition r_1p is gated on.
          if(PIN(tmp2) > PIN(test_cur) * 0.5) {
            PIN(r)    = PIN(r_known);
            PIN(drop) = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
            r_ok      = 1.0;
          }
        } else if(PIN(single_dwell) > 0.0) {
          // Top dwell alone. This is a resistance only once hv0.drop_k
          // cancels the dead time in hardware -- whatever drop_k misses
          // lands here as 4/3 * residual / test_cur ohms, so it wants the
          // highest test_cur the trip allows and means nothing below the
          // current drop_k was trimmed at. What it buys is the absence of
          // the one error the chord can never shed: the chord's slope picks
          // up 4/3 * K * ln(i2/i1) / (i2 - i1) from the dead time's rise
          // with current, and that term is a difference, so the compensation
          // subtracts out of it exactly and leaves the bias untouched. No
          // choice of drop_k improves the fit. This sidesteps it instead.
          if(PIN(r_1p) > 0.0) {
            PIN(r)    = PIN(r_1p);
            PIN(drop) = 0.0;  // it lives in hv0.drop_k now, not here
            r_ok      = 1.0;
          }
        } else if(di > 0.01) {
          // Two dwells, with the chord's bias subtracted rather than ignored.
          //
          // The bias is not noise, it has a closed form. ud carries 4/3 of the
          // dead time voltage, that voltage rises as K*ln(i), and the test's
          // own geometry is i2 = test_cur, i1 = test_cur/2, so
          //
          //   chord = r + 8/3 * K * ln2 / test_cur
          //
          // and computing it per phase at each phase's own current gives the
          // same number to four figures -- the log turns the ratio into a
          // difference and both dwells scale alike.
          //
          // What matters is which constant this needs. Not the dead time's
          // magnitude, which is the part that varies between power stages with
          // the switches' turn off delay, and which the chord cancels by being
          // a difference. Only K, its slope with current, which is a much more
          // transferable device property. Express it against the dc link, as
          // the whole drop scales that way: drop_slope = 8/3 * ln2 * K / Vdc,
          // 0.0039 ohm A per volt on the bridge this was characterised on.
          //
          // Against a four wire 0.685 ohm, correcting the chords measured on
          // that axis gives 0.682 at 6 A and 0.673 at 8 A. The same correction
          // at 3 A gives 0.986, because the log stops describing the drop once
          // the current is too small to commutate the devices cleanly -- hence
          // the warning below rather than a silent answer.
          // dc_volt, not pwm_volt: ls.c scales pwm_volt by a factor that
          // depends on phase_mode (1/sqrt3 for 120 deg 3 phase, 1/sqrt2 for
          // 90 deg, 1 for the 180 deg modes, 0 for none of them), so inverting
          // it with one constant is right for one mode and 22% out on another.
          // If dc_volt is not wired the bias is skipped rather than guessed.
          PIN(r_bias) = PIN(dc_volt) > 1.0 ? PIN(drop_slope) * PIN(dc_volt) / MAX(PIN(test_cur), 0.1) : 0.0;

          PIN(r) = MAX(PIN(r_2p) - PIN(r_bias), 0.001);

          // and read the drop at the top dwell against that r, the same form
          // the r_known path uses. The chord's own intercept would do, but it
          // is the worse conditioned half of the same fit.
          PIN(drop) = MAX(0.75 * (PIN(tmp3) - PIN(r) * PIN(tmp2)), 0.0);
          r_ok      = 1.0;
        }

        PIN(r_ok) = r_ok;

        // the l test needs the voltage that holds test_cur, dead time
        // included -- deriving it from r alone falls short by the dead time
        // and pushes almost no current. Build it from the two terms just
        // measured rather than from a ud/id ratio: that ratio is divided by
        // a current, so a dwell that reads low inflates it without bound and
        // the LIMIT below hands the l test half the dc link in voltage mode.
        //
        // In single dwell mode drop is 0 and this reduces to r * test_cur,
        // which is right for the same reason that mode is: hv0.drop_k adds
        // the dead time volts downstream of the command, so the command does
        // not have to carry them. Run that mode with drop_k unset and r
        // absorbs the dead time instead, which lands this in much the same
        // place by a worse route.
        // and it must never be built from a measurement that did not happen.
        // With no branch taken PIN(r) is still the bootstrap ud/id ratio,
        // divided by a current that never flowed, so it is arbitrarily large;
        // the LIMIT caps it at half the dc link, which into a sub ohm winding
        // is an overcurrent trip rather than a test. Skip the l test instead
        // and go straight to the report, which already says nothing was
        // measured.
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

    case 13:  // l
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      //PIN(l) = PIN(l) * 0.995 + ABS(PIN(ud_fb) - avg_test_volt / 2.0) / MAX(ABS(PIN(id_fb) - PIN(test_cur)), 0.001) * period * 0.005;
      if(PIN(d_cmd) < PIN(avg_test_volt)) {
        PIN(tmp0)  = PIN(tmp0) * 0.99 + PIN(id_fb) * 0.01;
        PIN(tmp1)  = PIN(tmp1) * 0.99 + PIN(ud_fb) * 0.01;
        PIN(d_cmd) = PIN(avg_test_volt) * 1.5;
      } else {
        PIN(tmp2)  = PIN(tmp2) * 0.99 + PIN(id_fb) * 0.01;
        PIN(tmp3)  = PIN(tmp3) * 0.99 + PIN(ud_fb) * 0.01;
        PIN(d_cmd) = PIN(avg_test_volt) * 0.5;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 1.0) {
        PIN(l)      = ABS(PIN(tmp1) - PIN(tmp3)) / MAX(ABS(PIN(tmp0) - PIN(tmp2)), 0.001) * period;
        PIN(timer)  = 0.0;
        PIN(state)  = 1.4;
        PIN(d_cmd)  = PIN(avg_test_volt);
        PIN(en_out) = 0.0;
        // PIN(tmp0) = 0.0;
        // PIN(tmp1) = 0.0;
      }
      break;

    case 22:  // pp, out_rev
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
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

        PIN(state) = 2.3;
      }
      break;

    case 23:  // mot_fb_offset
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) = 0.0;

      PIN(com_offset) = PIN(com_offset) * 0.99 - PIN(pos_fb) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 2.0) {
        PIN(en)    = 0.0;
        PIN(d_cmd) = 0.0;
        PIN(state) = 2.5;
        PIN(timer) = 0.0;
      }
      break;

    case 32:  // psi
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 250.0;
      PIN(d_cmd)    = 0.0;
      PIN(com_pos)  = mod((PIN(pos_fb) + PIN(com_offset)) * PIN(pp));

      float vel_error = PIN(test_vel) - PIN(vel_fb);
      vel_error       = LIMIT(vel_error, PIN(test_vel) / 100.0);
      PIN(cur_sum) += PIN(ki) * vel_error * period;

      PIN(q_cmd) = PIN(vel_bw) * period * vel_error + PIN(cur_sum);

      if(ABS(PIN(vel_fb)) > 0.1) {
        float psi = (PIN(uq_fb) - PIN(iq_fb) * PIN(r)) / (PIN(vel_fb) * PIN(pp));
        PIN(psi)  = PIN(psi) * (1.0 - period / PIN(pi)) + psi * period / PIN(pi);
      }

      PIN(psi) = CLAMP(PIN(psi), 0.001, 1.0);

      PIN(timer) += period;
      if(PIN(timer) >= 5.0) {
        PIN(timer)    = 0.0;
        PIN(en_out)   = 0.0;
        PIN(d_cmd)    = 0.0;
        PIN(q_cmd)    = 0.0;
        PIN(cur_sum)  = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 3.3;
      }
      break;

    case 100:
      PIN(com_pos) = mod((PIN(pos_fb) + PIN(com_offset)) * PIN(pp));
  }
}


hal_comp_t idpmsm_comp_struct = {
    .name      = "idpmsm",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,  //sizeof(struct idpmsm_ctx_t),
    .pin_count = sizeof(struct idpmsm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};