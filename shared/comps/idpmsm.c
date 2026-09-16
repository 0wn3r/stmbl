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
HAL_PIN(fit_di);   // smaller chord current span, 0 = r/drop fit did not run
HAL_PIN(fit_sa);   // high chord slope
HAL_PIN(fit_sb);   // low chord slope

HAL_PIN(pp);
HAL_PIN(com_offset);
HAL_PIN(out_rev);

HAL_PIN(test_cur);
HAL_PIN(test_vel);
HAL_PIN(ki);
HAL_PIN(vel_bw);

HAL_PIN(pi);

HAL_PIN(pwm_volt);

HAL_PIN(psi);

HAL_PIN(cur_bw);
HAL_PIN(cur_sum);
HAL_PIN(auto_step);

HAL_PIN(tmp0);
HAL_PIN(tmp1);
HAL_PIN(tmp2);
HAL_PIN(tmp3);
HAL_PIN(tmp4);
HAL_PIN(tmp5);
HAL_PIN(avg_test_volt);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idpmsm_ctx_t * ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;
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
      PIN(tmp4)     = 0.0;
      PIN(tmp5)     = 0.0;

      if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Measure r, l\n");
        printf("the motor can move a bit\n");
        printf("idpmsm0.state = 1.2 <font color='green'>to start</font>\n");
      }
      break;

    case 14:
      if(PIN(fit_di) > 0.01) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(l));
        printf("hv0.drop = %f <font color='green'># dead time, scales with dc link</font>\n", PIN(drop));
        // r and drop now come from two chords in one run, so no rerun is
        // needed. What still matters is that the lowest dwell draws real
        // current: below a machine specific level the dead time voltage
        // stops shrinking with current and the Richardson step has nothing
        // to remove, which is where a single chord read 1.38 on a 0.685 ohm
        // winding. Report the chord slopes so that is visible.
        printf("<font color='green'># chord slopes %f and %f ohm.\n", PIN(fit_sa), PIN(fit_sb));
        printf("# if they are nearly equal, test_cur is too low for the\n");
        printf("# fit to separate r from the dead time -- raise it.</font>\n");
      } else {
        // the two dwells read the same current, so there is no line to fit and
        // r, drop and everything downstream of them are still at their init
        // values. Say so -- printing those as a result is worse than failing.
        printf("<font color='red'>r fit failed</font>: the two dwells differ by %f A\n", PIN(fit_di));
        printf("nothing below is measured, do not append it\n");
        printf("check that idpmsm0.test_cur (%f) is under conf0.max_ac_cur\n", PIN(test_cur));
      }
      PIN(state) = 2.0;
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
        PIN(tmp0)  = PIN(tmp0) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp1)  = PIN(tmp1) * 0.999 + PIN(ud_fb) * 0.001;
      } else if(PIN(timer) < 4.0) {
        PIN(d_cmd) = PIN(test_cur) * 0.5;
        PIN(tmp2)  = PIN(tmp2) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp3)  = PIN(tmp3) * 0.999 + PIN(ud_fb) * 0.001;
      } else {
        PIN(d_cmd) = PIN(test_cur) * 0.25;
        PIN(tmp4)  = PIN(tmp4) * 0.999 + PIN(id_fb) * 0.001;
        PIN(tmp5)  = PIN(tmp5) * 0.999 + PIN(ud_fb) * 0.001;
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
      if(PIN(timer) >= 6.0) {
        // the filters are linear and every dwell uses the same one, so the
        // fits hold on the filtered pairs even though the loop is slow
        // enough that no dwell reaches its commanded current.
        //
        // One chord is not enough. Its slope is r plus 4/3 of the local
        // slope of the dead time voltage, and that term is still large at
        // any current the trip limit allows, so a single chord reads high
        // by tens of percent. It shrinks as the chord moves up in current,
        // so a second chord lower down measures the shrinkage and a
        // Richardson step removes it. Chords at 0.75 and 0.375 of test_cur
        // sharing the middle dwell give the widest lever the current range
        // allows; simulated against a dV(i) fitted to bench data the bias
        // is about 3% where one chord alone is 20 to 100%.
        float dia  = PIN(tmp0) - PIN(tmp2);
        float dib  = PIN(tmp2) - PIN(tmp4);
        PIN(fit_di) = MIN(dia, dib);
        if(dia > 0.01 && dib > 0.01) {
          float sa = (PIN(tmp1) - PIN(tmp3)) / dia;
          float sb = (PIN(tmp3) - PIN(tmp5)) / dib;
          PIN(fit_sa) = sa;
          PIN(fit_sb) = sb;
          float ia = 0.5 * (PIN(tmp0) + PIN(tmp2));
          float ib = 0.5 * (PIN(tmp2) + PIN(tmp4));
          PIN(r)    = MAX((ia * sa - ib * sb) / MAX(ia - ib, 0.01), 0.001);
          // evaluate the dead time at the top dwell, where it is closest to
          // its asymptote and so closest to what hv0.drop should hold
          PIN(drop) = MAX(0.75 * (PIN(tmp1) - PIN(r) * PIN(tmp0)), 0.0);
        }

        // the l test needs the voltage that holds test_cur, dead time
        // included -- deriving it from r alone falls short by the dead time
        // and pushes almost no current. Build it from the two terms just
        // measured rather than from a ud/id ratio: that ratio is divided by
        // a current, so a dwell that reads low inflates it without bound and
        // the LIMIT below hands the l test half the dc link in voltage mode.
        PIN(avg_test_volt) = PIN(r) * PIN(test_cur) + 4.0 / 3.0 * PIN(drop);
        PIN(avg_test_volt) = LIMIT(PIN(avg_test_volt), PIN(pwm_volt) / 2.0);

        PIN(timer)  = 0.0;
        PIN(state)  = 1.3;
        PIN(d_cmd)  = 0.0;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;
        PIN(tmp2)   = 0.0;
        PIN(tmp3)   = 0.0;
        PIN(tmp4)   = 0.0;
        PIN(tmp5)   = 0.0;
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
        PIN(l)      = ABS(PIN(tmp1) - PIN(tmp3)) / ABS(PIN(tmp0) - PIN(tmp2)) * period;
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