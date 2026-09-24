#include "idpmsm_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"
#include <math.h>

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
HAL_PIN(l_half);  // *parameter*, l test half period [s]
HAL_PIN(tau);     // measured current time constant l/r [s]
HAL_PIN(l_ok);    // 1 = l is a measurement, 0 = it is not
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
HAL_PIN(pp_raw);  // pole pair ratio before rounding, sign carries out_rev
HAL_PIN(pp_ok);   // the pp test tracked the field and landed near an integer
HAL_PIN(com_offset);
HAL_PIN(out_rev);

// The commutation offset is a circular mean, not an average. pos_fb is a rotor
// angle: averaging it linearly is only right while it never crosses the wrap,
// and when the alignment position lands inside the ring down swing of the wrap
// it fails hard (a park at 180 degrees, 6 Hz ring at zeta 0.05, came out 0.77
// rad wrong -- 3.1 rad electrical at four pole pairs). Accumulate the unit
// vector and take its argument. The resultant's length is 1 when the rotor sat
// still for the window and falls toward 0 if it wandered.
HAL_PIN(off_sin);
HAL_PIN(off_cos);
HAL_PIN(off_mag);  // resultant length, 1 = the rotor held station
HAL_PIN(off_n);
HAL_PIN(com_ok);   // the offset dwell drew its current and the rotor held still

HAL_PIN(test_cur);
HAL_PIN(test_vel);
HAL_PIN(ki);
HAL_PIN(vel_bw);

HAL_PIN(pwm_volt);
HAL_PIN(dc_volt);

HAL_PIN(psi);
HAL_PIN(psi_ok);     // this run produced a psi
HAL_PIN(u_fb);       // phase u voltage to ground from hv0, read while coasting
HAL_PIN(v_fb);       // phase v voltage to ground from hv0
HAL_PIN(psi_hi);     // back emf psi from the fast half of the coast
HAL_PIN(psi_lo);     // back emf psi from the slow half of the coast
HAL_PIN(emf_angle);  // back emf phase against the commutation angle, extrapolated to standstill [deg el]
HAL_PIN(emf_delay);  // how much later the phase voltage reads than the rotor angle [s]
HAL_PIN(vel_lo);     // mean speed of the low dwell [rad/s]
HAL_PIN(vel_hi);     // mean speed of the high dwell [rad/s]
HAL_PIN(udt_lo);     // uq - r iq - pp vel psi in the low dwell: dead time volts on q [V]
HAL_PIN(idt_lo);     // mean iq in the low dwell [A]
HAL_PIN(udt_hi);     // the same for the high dwell [V]
HAL_PIN(idt_hi);     // [A]

HAL_PIN(cur_bw);
HAL_PIN(cur_sum);
HAL_PIN(auto_step);

HAL_PIN(tmp0);
HAL_PIN(tmp1);
HAL_PIN(tmp2);
HAL_PIN(tmp3);
HAL_PIN(avg_test_volt);


// The l test's state. tau is integrated over thousands of ticks and the window
// is timed off what comes back in ud_fb, so none of this can be pins without
// making the state machine's scratch pins mean two different things at once.
struct idpmsm_ctx_t {
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

  // pp test
  uint32_t pp_n;     // ticks in the measure window where the rotor was turning
  uint32_t pp_w;     // ticks in the measure window
  float pp_field;    // field angle turned across the window [rad el]
  float pp_rotor;    // rotor angle turned across the window [rad]

  // psi test: dwells at test_vel / 2 and test_vel, then a coast from the top
  uint8_t psi_stage;  // 0 spin up low, 1 dwell low, 2 spin up high, 3 dwell high, 4 coast
  uint8_t psi_fail;   // 0 none, 1 stalled, 2 never settled, 3 coast too short
  uint32_t psi_n;     // samples in this dwell
  float st_t;         // time in this stage
  float settle_t;     // time the filtered speed has been inside the band
  float vel_lp;       // filtered vel_fb, for settling and stall detection
  float sx;           // sum of pp * vel_fb across the dwell
  float sy;           // sum of uq_fb - r * iq_fb across the dwell
  float si;           // sum of iq_fb across the dwell
  float x_lo, y_lo, i_lo;
  float x_hi, y_hi, i_hi;
  // coast, per band (0 fast, 1 slow): the line to line voltage demodulated
  // against the commutation angle, see the psi case in rt_func
  uint32_t cn[2];
  float cz_re[2], cz_im[2];  // sum of (u - v) * g
  float cg_re[2], cg_im[2];  // sum of g, to take out the dc level of u - v
  float cu[2];               // sum of u - v
  float cw[2];               // sum of the electrical speed
};

// hv0.u_fb/v_fb come from io.c on the f3, which filters each phase with
// u = 0.05 * adc + 0.95 * u at its 15 kHz rt rate. At 400 rad/s electrical
// that is 11% of amplitude and 27 degrees of phase, so the coast undoes it.
#define HV_IO_ALPHA 0.05
#define HV_IO_PERIOD (1.0 / 15000.0)

#define PP_RAMP 1.0      // pp test: time to ramp the field up to test_vel [s]
#define PP_SETTLE 0.5    // pp test: wait after the ramp before measuring [s]
#define PP_TIME 4.0      // pp test: total [s]
#define PSI_SETTLE 0.5   // psi test: speed inside the band this long [s]
#define PSI_BAND 0.15    // psi test: settled band, fraction of the dwell speed
#define PSI_DWELL 2.0    // psi test: averaging time per dwell [s]
#define PSI_SPINUP 8.0   // psi test: give up reaching a dwell speed after [s]
#define PSI_STALL 0.5    // psi test: q current, as a fraction of test_cur, that has to turn the rotor
#define PSI_COAST_MIN 0.2   // psi coast: ignore speeds under this fraction of test_vel
#define PSI_COAST_SPLIT 0.6 // psi coast: the fast band is above this fraction of test_vel
#define PSI_COAST_TIME 4.0  // psi coast: longest it may run [s]
#define PSI_COAST_N 200     // psi coast: fewest samples a band needs

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idpmsm_ctx_t * ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;
  PIN(r_known)                  = 0.0;
  PIN(single_dwell)             = 0.0;
  PIN(drop_slope)               = 0.0039;
  // Swept on X (alpha3/3000) with the load off, 2026-09-23. test_cur 3, 4, 5,
  // 6 and 8 A gave r 0.984, 0.871, 0.707, 0.679 and 0.657 against a meter's
  // 0.685: below 5 A the dead time drop stops following ln(i) and the chord's
  // bias correction understates, and 6 A lands within 1% (three more runs at
  // 6 A: 0.680, 0.683, 0.687). psi, pp and the offset did not care.
  PIN(test_cur) = 6.0;
  // test_vel 50, 75, 100 and 150 gave psi 0.0346, 0.0354, 0.0348 and 0.0346
  // from the coast, with its two halves within a few percent. At 40 the halves
  // split (0.0343 / 0.0377), at 35 psi and the emf angle drift (0.0337, 50
  // deg against 58-62), and at 25 the halves split again. 50 is the lowest
  // that holds, and faster buys nothing. The pp test's field runs at this
  // many electrical rad/s, 12.5 mechanical at four pole pairs.
  PIN(test_vel) = 50.0;
  PIN(ki)                       = 1.0;
  PIN(vel_bw)                   = 20.0;
  PIN(cur_bw)                   = 1.0;
  PIN(auto_step)                = 4.2;
}


// ctx survives a stop, so come back to a cleared detector rather than half a
// measurement taken against whatever the bridge was doing when the loop stopped.
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idpmsm_ctx_t *ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idpmsm_ctx_t));
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idpmsm_ctx_t *ctx = (struct idpmsm_ctx_t *)ctx_ptr;
  struct idpmsm_pin_ctx_t *pins = (struct idpmsm_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(r)          = 0.1;
      PIN(l)          = 0.001;
      PIN(l_half)     = 0.1;
      PIN(drop)       = 0.0;
      PIN(psi)        = 0.055;
      PIN(pp)         = 3.0;
      PIN(com_offset) = 0.0;
      PIN(out_rev)    = 0.0;
      PIN(cur_bw)     = 1.0;
      PIN(pp_ok)      = 0.0;
      PIN(com_ok)     = 0.0;
      PIN(psi_ok)     = 0.0;
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
        if(PIN(l_ok) > 0.0) {
          printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(l));
          printf("<font color='green'># from tau = %f ms against the r above, not from a voltage slope.\n", PIN(tau) * 1000.0);
          printf("# an LCR still beats it: measure line to line at 1 kHz and halve.</font>\n");
        } else if(PIN(tau) <= 0.0) {
          printf("<font color='red'>l not measured</font>: no usable transient\n");
          printf("check that the bridge is enabled and idpmsm0.ud_fb is wired\n");
        } else if(PIN(tau) > PIN(l_half) / 8.0) {
          printf("<font color='red'>l not measured</font>: tau = %f ms needs a longer half period\n", PIN(tau) * 1000.0);
          printf("raise idpmsm0.l_half above %f s and rerun\n", PIN(tau) * 8.0);
        } else {
          printf("<font color='red'>l not measured</font>: tau = %f ms is too fast to time here\n", PIN(tau) * 1000.0);
          printf("use an LCR meter: line to line at 1 kHz, halved\n");
        }
        // Measurement, not a config line. hv0.drop and hv0.drop_k both feed the
        // same dt_drop in the f3's hv.c, and that compensation is not yet safe
        // to switch on: it is keyed on measured current, so once it exceeds the
        // real drop it drives the current it is reading. Setting either one in
        // a config has taken a drive to its overcurrent trip. Read the number,
        // do not append it.
        printf("<font color='red'>measured</font> drop = %f V at the %f A dwell, %f V link\n", PIN(drop), PIN(test_cur), PIN(dc_volt));
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
      // its plant model at cur_bw 100. Not back to 0 on a failure either: with
      // the drive still enabled, rt's state 0 walks straight into 1.0 and
      // auto_step reruns the r test forever. 9.0 has no rt case, so en_out
      // stays where the test left it, off, until the drive is disabled.
      PIN(state) = PIN(r_ok) > 0.0 ? 2.0 : 9.0;
      break;

    case 20:  // pp, out_rev, com_offset
      PIN(state) = 2.1;
      PIN(timer) = 0.0;
      //PIN(d_cmd) = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(pp_ok)    = 0.0;
      ctx->pp_n     = 0;
      ctx->pp_w     = 0;
      ctx->pp_field = 0.0;
      ctx->pp_rotor = 0.0;

      if(PIN(auto_step) >= 2) {
        PIN(state) = 2.2;
      } else {
        printf("Measure com_offset, polepairs, out_rev\n");
        printf("the motor will move\n");
        printf("idpmsm0.state = 2.2 <font color='green'>to start</font>\n");
      }
      break;

    case 24:  // pp failed, nothing downstream of it can run
      printf("<font color='red'>pp read failed</font>: ratio %f", PIN(pp_raw));
      printf(", the rotor turned for %lu of %lu ticks\n", (unsigned long)ctx->pp_n, (unsigned long)ctx->pp_w);
      printf("the rotor did not follow the field. lower idpmsm0.test_vel\n");
      printf("(%f el. rad/s) or raise idpmsm0.test_cur and rerun\n", PIN(test_vel));
      printf("nothing below is measured, do not append it\n");
      PIN(state) = 9.0;
      break;

    case 26:  // com_offset failed
      if(PIN(off_mag) <= 0.98) {
        printf("<font color='red'>mot_fb_offset not measured</font>: the rotor moved during the dwell\n");
        printf("(resultant %f, 1.0 means it held still). raise idpmsm0.test_cur.\n", PIN(off_mag));
      } else {
        printf("<font color='red'>mot_fb_offset not measured</font>: the dwell drew %f A of %f\n", PIN(id_fb), PIN(test_cur));
      }
      printf("nothing below is measured, do not append it\n");
      PIN(state) = 9.0;
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
      PIN(psi_ok)    = 0.0;
      PIN(psi_hi)    = 0.0;
      PIN(psi_lo)    = 0.0;
      PIN(emf_angle) = 0.0;
      PIN(emf_delay) = 0.0;
      PIN(vel_lo)    = 0.0;
      PIN(vel_hi)    = 0.0;
      PIN(udt_lo)    = 0.0;
      PIN(idt_lo)    = 0.0;
      PIN(udt_hi)    = 0.0;
      PIN(idt_hi)    = 0.0;
      ctx->psi_stage = 0;
      ctx->psi_fail  = 0;
      ctx->psi_n     = 0;
      ctx->st_t      = 0.0;
      ctx->settle_t  = 0.0;
      ctx->vel_lp    = 0.0;
      ctx->sx        = 0.0;
      ctx->sy        = 0.0;
      ctx->si        = 0.0;
      for(int b = 0; b < 2; b++) {
        ctx->cn[b]    = 0;
        ctx->cz_re[b] = 0.0;
        ctx->cz_im[b] = 0.0;
        ctx->cg_re[b] = 0.0;
        ctx->cg_im[b] = 0.0;
        ctx->cu[b]    = 0.0;
        ctx->cw[b]    = 0.0;
      }

      if(PIN(auto_step) >= 3) {
        PIN(state) = 3.2;
      } else {
        printf("Measure torque constant\n");
        printf("the motor will move\n");
        printf("id0.state = 3.2 to start\n");
      }
      break;

    case 33:
      if(PIN(psi_ok) > 0.0) {
        printf("conf0.psi = %f <font color='green'># append to config</font>\n", PIN(psi));
        printf("<font color='green'># back emf with the bridge off, coasting down from %f rad/s:\n", PIN(vel_hi));
        printf("# %f in the fast half, %f in the slow half. no dead time or\n", PIN(psi_hi), PIN(psi_lo));
        printf("# current loop in it -- it is what a scope on the phases reads.</font>\n");
        printf("<font color='green'># back emf leads the commutation angle by %f deg el at standstill,\n", PIN(emf_angle));
        printf("# and reads %f ms later than the rotor angle. with conf0.mot_fb_offset\n", PIN(emf_delay) * 1000.0);
        printf("# right the angle repeats from run to run; a shift of it is an offset\n");
        printf("# error of that many electrical degrees.</font>\n");
        printf("<font color='green'># dead time on q while turning, uq - r iq - pp vel psi:\n");
        printf("# %f V at %f A (%f rad/s), %f V at %f A (%f rad/s).\n", PIN(udt_lo), PIN(idt_lo), PIN(vel_lo), PIN(udt_hi), PIN(idt_hi), PIN(vel_hi));
        printf("# the r test's drop above is the same thing at standstill on d.</font>\n");
        printf("done\n");
        printf("continue with id_mot\n");
      } else {
        if(ctx->psi_fail == 1) {
          printf("<font color='red'>psi read failed</font>: the rotor stalled\n");
          printf("check conf0.polecount, conf0.mot_fb_offset and out_rev above\n");
        } else if(ctx->psi_fail == 2) {
          printf("<font color='red'>psi read failed</font>: the speed never settled inside %i%%\n", (int)(PSI_BAND * 100.0));
          printf("of the dwell. check the load is off, or lower idpmsm0.test_vel\n");
        } else {
          printf("<font color='red'>psi read failed</font>: the coast gave %lu and %lu usable samples\n", (unsigned long)ctx->cn[0], (unsigned long)ctx->cn[1]);
          printf("check idpmsm0.u_fb/v_fb are wired to hv0.u_fb/v_fb, or raise idpmsm0.test_vel\n");
        }
        printf("nothing here is measured, do not append it\n");
      }

      PIN(state) = 3.4;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idpmsm_ctx_t *ctx        = (struct idpmsm_ctx_t *)ctx_ptr;
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

    case 13: {  // l, from the current's relaxation time constant
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      // The old test swung the commanded voltage 1.5x / 0.5x about
      // avg_test_volt every rt tick and read l off dV/dI * period. That is
      // wrong twice over. The voltage DEVIATION driving the ramp is half the
      // swing, not the swing, so the formula returns 2l. And at 5 kHz against
      // a 15 kHz current loop the sample that comes back covers some unknown
      // fraction of a ramp: simulating the f3 sub tick the packet happens to
      // carry moves the answer between 2x and 6x. On this bench it read
      // 15.9 mH against an LCR's 3.30 mH.
      //
      // So measure a time instead of a slope. Step between two currents of the
      // same sign and take the relaxation time constant by the area method:
      //
      //   i(t) = i_f + (i_a - i_f) exp(-t/tau),  tau = l/r
      //   integral of (i_f - i) dt = (i_f - i_a) * tau     for T >> tau
      //   => tau = (i_b * T - integral i dt) / (i_b - i_a)
      //
      // A time constant does not care about the amplitude, so the dead time
      // drop -- a constant volt offset for as long as the current keeps its
      // sign -- cancels out of it exactly. That is also why both levels have to
      // sit on the same side of zero: the old test's 0.5x level is under the
      // drop at these dead times, which collapses the current and the
      // assumption with it.
      float half = MAX(PIN(l_half), 0.01);
      float v_hi = PIN(avg_test_volt);
      float v_lo = PIN(avg_test_volt) - PIN(r) * PIN(test_cur) * 0.5;
      float mid  = (v_hi + v_lo) * 0.5;

      // Time the window off the step as it comes BACK in ud_fb rather than off
      // the tick we commanded it. ls.c packs ud_fb and id_fb into the same
      // packet, so they carry identical delay and it cancels instead of having
      // to be guessed at. Simulated, that takes the spread across zero, one and
      // two ticks of pipeline delay from 3x to nothing.
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
        // trapezoid: the half tick a left hand sum leaves behind is worth about
        // a percent of tau on these motors
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

        // Two ways this reads nothing. Too slow for the half period and the
        // truncated tail drags the area down -- 5% low already at T = 5 tau.
        // Too fast and the transient is a handful of samples, where noise runs
        // the answer and a meter beats this outright.
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

    case 22:  // pp, out_rev
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      // Ramp the field instead of stepping it to test_vel. A step asks the
      // rotor to jump to full speed from rest, and at test_vel 100 it slipped:
      // the field ran on alone, vel_fb stayed near 0 and the ratio below never
      // updated off its init value, or updated on the slip and read 10.
      {
        float field_vel = PIN(test_vel) * MIN(PIN(timer) / PP_RAMP, 1.0);
        PIN(com_pos) += field_vel * period;
        PIN(com_pos) = mod(PIN(com_pos));

        // only once the rotor has had time to lock on at full speed. The
        // ratio is of the angles turned over the whole window, not a filtered
        // ratio of speeds: that was a 40 ms snapshot of field_vel / vel_fb,
        // and at test_vel 25 the rotor's cogging moved it 5% (3.80 and 3.84
        // for a rotor that tracked every tick).
        if(PIN(timer) >= PP_RAMP + PP_SETTLE) {
          ctx->pp_w++;
          ctx->pp_field += field_vel * period;
          ctx->pp_rotor += PIN(vel_fb) * period;
          if(ABS(PIN(vel_fb)) > 0.1) {
            ctx->pp_n++;
          }
        }
      }

      PIN(timer) += period;
      if(PIN(timer) >= PP_TIME) {
        PIN(timer)  = 0.0;
        PIN(pp)     = ABS(ctx->pp_rotor) > 0.01 ? ctx->pp_field / ctx->pp_rotor : 0.0;
        PIN(pp_raw) = PIN(pp);

        if(PIN(pp) < 0.0) {
          PIN(out_rev) = 1.0;
          PIN(pp) *= -1.0;
        }
        float pp_int = (int)(PIN(pp) + 0.5);

        // A rotor that followed the field turns at test_vel / pp for the whole
        // window and gives a ratio within a few percent of an integer. One that
        // slipped turns for part of it, or not at all, at whatever speed the
        // slip gives -- the ratio that leaves is not a pole pair count, and
        // rounding it hands the offset and psi tests a wrong commutation.
        PIN(pp_ok) = ctx->pp_n > ctx->pp_w * 9 / 10 && pp_int >= 1.0 && pp_int <= 24.0 && ABS(PIN(pp) - pp_int) < 0.15;
        PIN(pp)    = pp_int;

        PIN(off_sin) = 0.0;
        PIN(off_cos) = 0.0;
        PIN(off_n)   = 0.0;

        if(PIN(pp_ok) > 0.0) {
          PIN(state) = 2.3;
        } else {
          PIN(en_out) = 0.0;
          PIN(d_cmd)  = 0.0;
          PIN(state)  = 2.4;
        }
      }
      break;

    case 23:  // mot_fb_offset
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) = 0.0;

      // Sum over the back half of the dwell only: the rotor is still ringing
      // into alignment through the front half, and a plain sum over a settled
      // window is what makes the resultant's length mean something. An ema
      // would track the motion and report a full length whatever it did.
      PIN(timer) += period;
      if(PIN(timer) > 1.0) {
        PIN(off_sin) += sinf(PIN(pos_fb));
        PIN(off_cos) += cosf(PIN(pos_fb));
        PIN(off_n) += 1.0;
      }

      if(PIN(timer) >= 2.0) {
        float n       = MAX(PIN(off_n), 1.0);
        PIN(off_mag)  = sqrtf(PIN(off_sin) * PIN(off_sin) + PIN(off_cos) * PIN(off_cos)) / n;
        float com_off = -atan2f(PIN(off_sin), PIN(off_cos));

        // Fold it into one period. com_pos is mod((pos_fb + com_offset) * pp),
        // so adding 2 pi / pp to the offset changes nothing, and the rotor parks
        // on whichever of the pp alignment positions is nearest where the pole
        // pair test left it. On the bench three runs printed -0.5945, -0.5931
        // and 0.9805: the same axis, pi / 2 apart. Folding to [0, 2 pi / pp)
        // prints the same number every run, so one that really differs shows.
        if(PIN(pp) >= 1.0) {
          float fold = 2.0 * M_PI / PIN(pp);
          com_off    = com_off - fold * floorf(com_off / fold);
        }
        PIN(com_offset) = com_off;
        PIN(com_ok)     = (PIN(off_mag) > 0.98 && PIN(id_fb) > PIN(test_cur) * 0.5) ? 1.0 : 0.0;

        PIN(d_cmd) = 0.0;
        PIN(timer) = 0.0;
        // the psi test commutates from this offset, so a bad one stops here
        // rather than driving current at the wrong angle
        if(PIN(com_ok) > 0.0) {
          PIN(state) = 2.5;
        } else {
          PIN(en_out) = 0.0;
          PIN(state)  = 2.6;
        }
      }
      break;

    case 32:  // psi
      {
        float v_t = ctx->psi_stage < 2 ? PIN(test_vel) * 0.5 : PIN(test_vel);
        ctx->vel_lp += (PIN(vel_fb) - ctx->vel_lp) * period / 0.05;
        ctx->st_t += period;

        if(ctx->psi_stage < 4) {
          PIN(en_out)   = 1.0;
          PIN(cmd_mode) = 1.0;
          PIN(cur_bw)   = 250.0;
          PIN(d_cmd)    = 0.0;
          PIN(com_pos)  = mod((PIN(pos_fb) + PIN(com_offset)) * PIN(pp));

          float vel_error = v_t - PIN(vel_fb);
          vel_error       = LIMIT(vel_error, PIN(test_vel) / 100.0);
          PIN(cur_sum) += PIN(ki) * vel_error * period;
          PIN(q_cmd) = PIN(vel_bw) * period * vel_error + PIN(cur_sum);
        }

        // wrong commutation from a bad pp or offset puts current in without
        // torque; on the bench that was 4.5 A into a locked rotor for the full
        // 5 s. Key it on the current rather than a time: the speed loop's
        // integrator climbs at ki * test_vel / 100 A/s, so at test_vel 25 a
        // healthy rotor needed 2 s just to break away at 0.4 A, while at
        // test_vel 100 a locked one reaches 3 A in 3 s.
        if(ctx->psi_stage < 4 && ABS(ctx->vel_lp) < v_t * 0.1 && ABS(PIN(q_cmd)) > PIN(test_cur) * PSI_STALL) {
          ctx->psi_fail = 1;
        } else if(ctx->psi_stage == 0 || ctx->psi_stage == 2) {  // spin up
          if(ABS(ctx->vel_lp - v_t) < v_t * PSI_BAND) {
            ctx->settle_t += period;
          } else {
            ctx->settle_t = 0.0;
          }
          if(ctx->settle_t >= PSI_SETTLE) {
            ctx->psi_stage++;
            ctx->st_t  = 0.0;
            ctx->psi_n = 0;
            ctx->sx    = 0.0;
            ctx->sy    = 0.0;
            ctx->si    = 0.0;
          } else if(ctx->st_t > PSI_SPINUP) {
            ctx->psi_fail = 2;
          }
        } else if(ctx->psi_stage == 1 || ctx->psi_stage == 3) {  // dwell
          // uq is the current loop's voltage command, so it carries the
          // inverter's dead time on top of the back emf and r iq:
          //
          //   uq - r iq = pp vel psi + u_dt(iq)
          //
          // Dividing it by the speed, as this test used to, reported u_dt as
          // flux: 0.089 at 25 rad/s, 0.070 at 50, 0.052 at 100 on an axis a
          // scope on the phases puts at 0.036. A chord between two speeds
          // does not cancel it either, because friction draws more current at
          // the higher speed and u_dt is still rising there (0.049 and 0.036
          // for the 25-50 and 50-100 chords). So psi comes from the coast
          // below, and these dwells keep what is left over: u_dt at two
          // currents, which is what the dead time compensation needs.
          ctx->sx += PIN(vel_fb) * PIN(pp);
          ctx->sy += PIN(uq_fb) - PIN(iq_fb) * PIN(r);
          ctx->si += PIN(iq_fb);
          ctx->psi_n++;
          if(ctx->st_t >= PSI_DWELL) {
            float n = (float)ctx->psi_n;
            if(ctx->psi_stage == 1) {
              ctx->x_lo = ctx->sx / n;
              ctx->y_lo = ctx->sy / n;
              ctx->i_lo = ctx->si / n;
            } else {
              ctx->x_hi = ctx->sx / n;
              ctx->y_hi = ctx->sy / n;
              ctx->i_hi = ctx->si / n;
            }
            ctx->settle_t = 0.0;
            ctx->psi_stage++;
            ctx->st_t = 0.0;
          }
        } else {  // 4: coast
          // Bridge off and let it run down. The terminals then carry the back
          // emf alone -- no dead time, no current, no loop -- and io.c on the
          // f3 reads each phase to ground. Their difference is
          //
          //   u - v = sqrt3 w psi cos(th + phi)       w = pp vel, th = commutation angle
          //
          // seen through the f3's filter H(w). Multiplying by
          // g = exp(-j th) / (sqrt3 w H(w)) and averaging leaves psi exp(j phi) / 2,
          // plus the dc level of u - v times the mean of g, which the sums of g
          // and of u - v take back out. psi is the magnitude, phi the angle
          // between the back emf and the angle the drive commutates with.
          // Done offline on a coast from 90 rad/s this gave 0.0351 to 0.0368
          // across 22 to 90 rad/s, against 0.0367 from a scope with the motor
          // turned by a drill.
          PIN(en_out)   = 0.0;
          PIN(d_cmd)    = 0.0;
          PIN(q_cmd)    = 0.0;
          PIN(cur_sum)  = 0.0;
          PIN(cmd_mode) = 0.0;

          float vel  = PIN(vel_fb);
          float avel = ABS(vel);
          if(avel > PIN(test_vel) * PSI_COAST_MIN) {
            int b   = avel > PIN(test_vel) * PSI_COAST_SPLIT ? 0 : 1;
            float w = vel * PIN(pp);
            float s_th, c_th, s_wt, c_wt;
            sincos_fast((PIN(pos_fb) + PIN(com_offset)) * PIN(pp), &s_th, &c_th);
            sincos_fast(w * HV_IO_PERIOD, &s_wt, &c_wt);
            // 1 / H = (1 - (1 - a) exp(-j w T)) / a
            float hi_re = (1.0 - (1.0 - HV_IO_ALPHA) * c_wt) / HV_IO_ALPHA;
            float hi_im = ((1.0 - HV_IO_ALPHA) * s_wt) / HV_IO_ALPHA;
            // exp(-j th) / H, over sqrt3 w (signed, so turning backwards does
            // not add half a turn to phi)
            float k    = 1.0 / (1.7320508 * w);
            float g_re = (c_th * hi_re + s_th * hi_im) * k;
            float g_im = (c_th * hi_im - s_th * hi_re) * k;
            float uv   = PIN(u_fb) - PIN(v_fb);
            ctx->cz_re[b] += uv * g_re;
            ctx->cz_im[b] += uv * g_im;
            ctx->cg_re[b] += g_re;
            ctx->cg_im[b] += g_im;
            ctx->cu[b] += uv;
            ctx->cw[b] += w;
            ctx->cn[b]++;
          }

          if(avel < PIN(test_vel) * PSI_COAST_MIN || ctx->st_t > PSI_COAST_TIME) {
            float z_re[3], z_im[3], w_mean[2];
            uint32_t n_all = ctx->cn[0] + ctx->cn[1];
            for(int b = 0; b < 3; b++) {  // 0 fast, 1 slow, 2 both
              float n  = b < 2 ? (float)ctx->cn[b] : (float)n_all;
              float zr = b < 2 ? ctx->cz_re[b] : ctx->cz_re[0] + ctx->cz_re[1];
              float zi = b < 2 ? ctx->cz_im[b] : ctx->cz_im[0] + ctx->cz_im[1];
              float gr = b < 2 ? ctx->cg_re[b] : ctx->cg_re[0] + ctx->cg_re[1];
              float gi = b < 2 ? ctx->cg_im[b] : ctx->cg_im[0] + ctx->cg_im[1];
              float u  = b < 2 ? ctx->cu[b] : ctx->cu[0] + ctx->cu[1];
              n        = MAX(n, 1.0);
              z_re[b]  = zr / n - (u / n) * (gr / n);
              z_im[b]  = zi / n - (u / n) * (gi / n);
              if(b < 2) {
                w_mean[b] = ctx->cw[b] / n;
              }
            }
            if(ctx->cn[0] >= PSI_COAST_N && ctx->cn[1] >= PSI_COAST_N) {
              PIN(psi)    = CLAMP(2.0 * sqrtf(z_re[2] * z_re[2] + z_im[2] * z_im[2]), 0.001, 1.0);
              PIN(psi_hi) = 2.0 * sqrtf(z_re[0] * z_re[0] + z_im[0] * z_im[0]);
              PIN(psi_lo) = 2.0 * sqrtf(z_re[1] * z_re[1] + z_im[1] * z_im[1]);
              // the angle drifts with speed by the reading delay between the
              // two paths; two bands give the slope and the standstill value
              float ph_hi = atan2f(z_im[0], z_re[0]);
              float ph_lo = atan2f(z_im[1], z_re[1]);
              float slope = ABS(w_mean[0] - w_mean[1]) > 1.0 ? mod(ph_hi - ph_lo) / (w_mean[0] - w_mean[1]) : 0.0;
              PIN(emf_angle) = mod(ph_lo - slope * w_mean[1]) * 180.0 / M_PI;
              PIN(emf_delay) = -slope;
              // what the dwells saw beyond back emf and r iq
              PIN(vel_lo) = ctx->x_lo / PIN(pp);
              PIN(vel_hi) = ctx->x_hi / PIN(pp);
              PIN(udt_lo) = ctx->y_lo - ctx->x_lo * PIN(psi);
              PIN(idt_lo) = ctx->i_lo;
              PIN(udt_hi) = ctx->y_hi - ctx->x_hi * PIN(psi);
              PIN(idt_hi) = ctx->i_hi;
              PIN(psi_ok) = 1.0;
            } else {
              ctx->psi_fail = 3;
            }
            ctx->psi_stage = 5;
          }
        }
      }

      if(ctx->psi_fail || ctx->psi_stage >= 5) {
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
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idpmsm_ctx_t),
    .pin_count = sizeof(struct idpmsm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};