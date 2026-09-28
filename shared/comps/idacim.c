#include "idacim_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(idacim);

HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(com_pos);
HAL_PIN(cmd_mode);
HAL_PIN(en);
HAL_PIN(en_out);

HAL_PIN(id_fb);
HAL_PIN(ud_fb);

HAL_PIN(state);
HAL_PIN(timer);

HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(l_ok);    // 1 = l is a measurement, 0 = it is not
HAL_PIN(l_freq_a);  // *parameter*, leakage test, lower injection frequency [Hz]
HAL_PIN(l_freq_b);  // *parameter*, leakage test, upper injection frequency [Hz]
HAL_PIN(l_ripple);  // *parameter*, injected current, fraction of test_cur
HAL_PIN(l_za);      // |Z| at l_freq_a [ohm]
HAL_PIN(l_zb);      // |Z| at l_freq_b [ohm]
HAL_PIN(l_ia);      // injected current amplitude reached at l_freq_a [A]
HAL_PIN(l_ib);      // injected current amplitude reached at l_freq_b [A]
HAL_PIN(l_res);     // resistance the two frequencies imply, stator plus cage [ohm]

HAL_PIN(rot_half);    // *parameter*, rotor test, time at each current level [s]
HAL_PIN(rot_cycles);  // *parameter*, rotor test, measured cycles (two edges each)
HAL_PIN(rot_bw);      // *parameter*, rotor test, current loop bandwidth [rad/s]
HAL_PIN(rot_t0);      // *parameter*, rotor test, skip this long after each edge [s]
HAL_PIN(tr);          // rotor time constant Lr/Rr [s]
HAL_PIN(slip_n);      // 1/tr, acim_ttc's slip constant [rad/s electrical]
HAL_PIN(lmr);         // rotor side magnetizing inductance Lm^2/Lr [H]
HAL_PIN(ls);          // stator inductance l + lmr [H]
HAL_PIN(rot_n);       // edges that went into tr and lmr
HAL_PIN(rot_dip);     // largest current error at rot_t0, fraction of the step
HAL_PIN(tr_ok);       // 1 = tr, slip_n and lmr are measurements
HAL_PIN(tr_spread);   // (max - min) / mean of tr across edges
HAL_PIN(drop);          // dead time volts per phase at the top dwell [V]
HAL_PIN(r_known);       // *parameter*, measured winding resistance, 0 = fit it
HAL_PIN(fit_di);        // dwell current separation, 0 = r/drop fit did not run
HAL_PIN(r_2p);          // two dwell chord slope, 0 = the fit did not run
HAL_PIN(drop_slope);    // *parameter*, the chord's dead time bias, ohm A per volt
HAL_PIN(r_bias);        // what was subtracted from the chord to get r
HAL_PIN(r_ok);          // this run produced a resistance

HAL_PIN(pp);
HAL_PIN(out_rev);

HAL_PIN(test_cur);
HAL_PIN(test_vel);

HAL_PIN(vel_fb);

HAL_PIN(pwm_volt);
HAL_PIN(dc_volt);

HAL_PIN(cur_bw);

HAL_PIN(tmp0);
HAL_PIN(tmp1);
HAL_PIN(tmp2);
HAL_PIN(tmp3);
HAL_PIN(avg_test_volt);

// State of the leakage and rotor tests. They integrate over thousands of
// ticks, so none of this can be pins without making the state machine's
// scratch pins mean two different things at once.
struct idacim_ctx_t {
  // leakage injection
  uint8_t l_fi;     // 0 = l_freq_a, 1 = l_freq_b
  uint8_t l_stage;  // 0 settle, 1 size the amplitude, 2 measure
  uint16_t l_block; // sizing blocks done
  uint32_t l_n;     // samples in this block or window
  float l_t;        // time in this stage or block
  float l_th;       // injection phase
  float l_amp;      // injection voltage amplitude
  float v_re, v_im, i_re, i_im;
  // rotor step
  uint16_t r_edge;      // edges seen, the first magnetizes from zero
  uint16_t r_n;         // edges that produced a fit
  uint8_t have_lvl[2];  // a settled value is known for level 0 (test_cur), 1 (half)
  uint8_t act;          // this edge is being fitted
  uint8_t have_t0;      // i_t0 captured
  uint32_t e_n;         // samples in this level's settled tail
  float lvl_u[2];       // settled ud_fb at each level
  float lvl_i[2];       // settled id_fb at each level
  float r_t;            // time since the last commanded edge
  float eu, ei;         // settled tail sums
  float uinf, iinf;     // where this edge settles, from the last visit to its level
  float di;             // the step, settled to settled
  float i0;             // settled current before the step
  float i_t0;           // id_fb at rot_t0
  float lam_raw;        // integral of ud - uinf - R (id - iinf) since the edge
  float ii;             // integral of id - iinf since the edge
  float jj;             // integral of lam since the edge
  float aa, ab, bb, ay, by;  // normal equations of this edge's fit
  float tr_sum, lm_sum, tr_min, tr_max;
  float dip;            // largest |current error at rot_t0| / step
};

#define L_BIAS_SETTLE 1.0  // leakage test: first settle, the dc bias rides the slow rotor pole [s]
#define L_SETTLE 0.2       // leakage test: settle after changing frequency [s]
#define L_BLOCK 0.05       // leakage test: one amplitude sizing block [s]
#define L_BLOCKS 4         // leakage test: sizing blocks per frequency
#define L_MEASURE 0.4      // leakage test: demodulation window per frequency [s]
#define ROT_TAIL 0.75      // rotor test: the settled tail starts here, fraction of rot_half

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;
  PIN(r_known)                  = 0.0;
  PIN(drop_slope)               = 0.0039;
  // The r chord's dead time correction needs every phase above about 2 A in
  // its lower dwell (test_cur/2 on d is -test_cur/4 on v and w), and the
  // rotor test reuses the same two levels.
  PIN(test_cur)                 = 8.0;
  PIN(test_vel)                 = 50.0;
  // A pair either side of the current loop's crossover (cur_bw / 2 pi, about
  // 160 Hz). The leakage has no plateau on a cage rotor, so this is the band
  // conf0.l has to describe, not the kHz an LCR meter defaults to.
  PIN(l_freq_a)                 = 120.0;
  PIN(l_freq_b)                 = 240.0;
  PIN(l_ripple)                 = 0.15;
  // tr is 50 to 150 ms on a few kW motor. Each level's settled value is
  // read over its last quarter, so 1.5 s puts that 7.6 tr out even at
  // 150 ms; at 1.0 s the leftover tail reads tr 2% short there (simulated).
  PIN(rot_half)                 = 1.5;
  PIN(rot_cycles)               = 4.0;
  PIN(rot_bw)                   = 1500.0;
  PIN(rot_t0)                   = 0.015;
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

      printf("Measure r, leakage l, rotor time constant\n");
      printf("<font color='green'>the rotor may stay free: a dc field makes no torque on a cage</font>\n");
      printf("idacim0.state = 1.2 <font color='green'>to start</font>\n");
      break;

    case 15:
      if(PIN(r_ok) > 0.0) {
        printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
        if(PIN(l_ok) > 0.0) {
          printf("conf0.l = %f <font color='green'># leakage, sigma*Ls</font>\n", PIN(l));
          printf("<font color='green'># from |Z| %f / %f ohm at %f / %f Hz, which\n", PIN(l_za), PIN(l_zb), PIN(l_freq_a), PIN(l_freq_b));
          printf("# also imply %f ohm of stator plus cage resistance there.\n", PIN(l_res));
          printf("# right for acim_foc (hv0.psi carries the rotor flux); acim_ttc\n");
          printf("# has no flux term, so there it makes iq fall short at speed.</font>\n");
        } else {
          printf("<font color='red'>l not measured</font>: injected %f / %f A of %f A asked\n", PIN(l_ia), PIN(l_ib), PIN(l_ripple) * PIN(test_cur));
          printf("use an LCR meter: line to line near 150 Hz, halved\n");
        }
        if(PIN(tr_ok) > 0.0) {
          printf("acim_flux0.tr = %f <font color='green'># append to config</font>\n", PIN(tr));
          printf("acim_flux0.lmr = %f <font color='green'># append to config</font>\n", PIN(lmr));
          printf("<font color='green'># rotor time constant tr = %f ms from %f edges\n", PIN(tr) * 1000.0, PIN(rot_n));
          printf("# slip_n = 1/tr = %f rad/s. for acim_ttc, which derives it from vel_n:\n", PIN(slip_n));
          printf("# acim_ttc0.vel_n = (2 pi acim_ttc0.freq_n - %f) / conf0.polecount\n", PIN(slip_n));
          printf("# edges agree within %f of tr.\n", PIN(tr_spread));
          printf("# lmr = Lm^2/Lr = %f mH, ls = %f mH, at %f..%f A on d.\n", PIN(lmr) * 1000.0, PIN(ls) * 1000.0, PIN(test_cur) * 0.5, PIN(test_cur));
          printf("# tr is the rotor's at this temperature and flux: a hot cage\n");
          printf("# reads shorter, and rated flux saturates it a little shorter.</font>\n");
          if(PIN(rot_dip) > 0.1) {
            printf("<font color='red'># the current was %f of the step off at rot_t0:\n", PIN(rot_dip));
            printf("# the loop is slow. the fit uses the current that flowed, but\n");
            printf("# raise idacim0.rot_bw and rerun to confirm.</font>\n");
          }
        } else if(PIN(rot_n) >= 2.0) {
          printf("<font color='red'>tr not measured</font>: the fit gave %f ms, outside what this test can see\n", PIN(tr) * 1000.0);
        } else {
          printf("<font color='red'>tr not measured</font>: fewer than two edges fitted\n");
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

        memset(ctx, 0, sizeof(struct idacim_ctx_t));
        ctx->l_amp  = 1.0;
        PIN(timer)  = 0.0;
        PIN(state)  = r_ok > 0.0 ? 1.3 : 1.5;
        PIN(d_cmd)  = 0.0;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;
        PIN(tmp2)   = 0.0;
        PIN(tmp3)   = 0.0;
      }
      break;

    case 13: {  // leakage l, by injection at two frequencies
      // The time constant test that was here read l = tau * r. On an induction
      // motor a voltage step's current has two poles, about 2 ms and 100-250 ms,
      // and the area method returns the slow one's Ls / r: the full stator
      // inductance, some twenty times the leakage the current loop wants,
      // printed as a conf0.l to append. So inject instead, as idpmsm does, but
      // on d only (a cage rotor is round) and at two frequencies:
      //
      //   |Z|^2 = R^2 + w^2 l^2   at both  =>  l^2 = (|Zb|^2 - |Za|^2) / (wb^2 - wa^2)
      //
      // so no resistance goes in. That matters here: the cage adds a
      // frequency dependent resistance on top of the stator's (0.2 ohm at
      // 100 Hz on the spindle, 0.8 at 1 kHz), and subtracting the dc r from
      // one |Z| gets l 10-13% wrong near the loop's crossover. The pair
      // assumes R and l are equal at both frequencies; the cage's rise between
      // 120 and 240 Hz costs a few percent of l, not more.
      //
      // The dc bias holds test_cur on d, so every phase current stays on one
      // side of zero and the dead time is an offset, not a nonlinearity. In
      // volt mode that bias settles on the slow rotor pole, hence the longer
      // first settle. A single pulsating axis makes no torque at standstill.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(cur_bw)   = 1.0;
      PIN(com_pos)  = 0.0;
      PIN(q_cmd)    = 0.0;

      float fa = CLAMP(PIN(l_freq_a), 20.0, 0.2 / period);
      float fb = CLAMP(PIN(l_freq_b), 20.0, 0.2 / period);
      float f  = ctx->l_fi == 0 ? fa : fb;
      float w  = 2.0 * M_PI * f;
      ctx->l_th += w * period;
      if(ctx->l_th > 2.0 * M_PI) {
        ctx->l_th -= 2.0 * M_PI;
      }
      float sn, cs;
      sincos_fast(ctx->l_th, &sn, &cs);
      PIN(d_cmd) = PIN(avg_test_volt) + ctx->l_amp * sn;

      float v = PIN(ud_fb);
      float i = PIN(id_fb);
      ctx->l_t += period;
      float target = PIN(l_ripple) * PIN(test_cur);

      if(ctx->l_stage == 0) {
        if(ctx->l_t >= (ctx->l_fi == 0 ? L_BIAS_SETTLE : L_SETTLE)) {
          ctx->l_stage = 1;
          ctx->l_t     = 0.0;
          ctx->l_n     = 0;
          ctx->v_re = ctx->v_im = ctx->i_re = ctx->i_im = 0.0;
        }
      } else {
        ctx->v_re += v * sn;
        ctx->v_im += v * cs;
        ctx->i_re += i * sn;
        ctx->i_im += i * cs;
        ctx->l_n++;
        float n = MAX((float)ctx->l_n, 1.0);
        if(ctx->l_stage == 1 && ctx->l_t >= L_BLOCK) {
          // size the amplitude to the ripple asked for: enough signal, and
          // never so much that a phase current crosses zero
          float i1 = 2.0 / n * sqrtf(ctx->i_re * ctx->i_re + ctx->i_im * ctx->i_im);
          float k  = i1 > 0.001 ? target / i1 : 4.0;
          ctx->l_amp *= CLAMP(k, 0.25, 4.0);
          ctx->l_amp = CLAMP(ctx->l_amp, 0.2, PIN(pwm_volt) / 4.0);
          ctx->l_block++;
          ctx->l_t = 0.0;
          ctx->l_n = 0;
          ctx->v_re = ctx->v_im = ctx->i_re = ctx->i_im = 0.0;
          if(ctx->l_block >= L_BLOCKS) {
            ctx->l_stage = 2;
          }
        } else if(ctx->l_stage == 2 && ctx->l_t >= L_MEASURE) {
          // the f3 holds each 5 kHz command for a whole tick, which scales the
          // applied fundamental by sin(pi f T) / (pi f T); take it back out
          float v1  = 2.0 / n * sqrtf(ctx->v_re * ctx->v_re + ctx->v_im * ctx->v_im);
          float i1  = 2.0 / n * sqrtf(ctx->i_re * ctx->i_re + ctx->i_im * ctx->i_im);
          float x   = M_PI * f * period;
          float zoh = sinf(x) / x;
          float z   = i1 > 0.001 ? v1 * zoh / i1 : 0.0;
          if(ctx->l_fi == 0) {  // on to the upper frequency, from this amplitude
            PIN(l_za)    = z;
            PIN(l_ia)    = i1;
            ctx->l_fi    = 1;
            ctx->l_stage = 0;
            ctx->l_block = 0;
            ctx->l_t     = 0.0;
            ctx->l_n     = 0;
          } else {
            PIN(l_zb)  = z;
            PIN(l_ib)  = i1;
            float wa   = 2.0 * M_PI * fa;
            float wb   = 2.0 * M_PI * fb;
            float za   = PIN(l_za);
            float den  = wb * wb - wa * wa;
            float l2   = den > 0.0 ? (z * z - za * za) / den : 0.0;
            float r2   = den > 0.0 ? (wb * wb * za * za - wa * wa * z * z) / den : 0.0;
            int ok     = l2 > 0.0 && PIN(l_ia) > target * 0.5 && PIN(l_ia) < target * 2.0 && i1 > target * 0.5 && i1 < target * 2.0;
            PIN(l_ok)  = ok ? 1.0 : 0.0;
            PIN(l_res) = r2 > 0.0 ? sqrtf(r2) : 0.0;
            // hv0.l = idacim0.l, so the rotor test's current loop runs on this.
            // A failed read leaves the 1 mH init, a sane leakage for a few kW.
            PIN(l) = ok ? sqrtf(l2) : 0.001;

            memset(ctx, 0, sizeof(struct idacim_ctx_t));
            PIN(timer)  = 0.0;
            PIN(state)  = 1.4;
            PIN(d_cmd)  = 0.0;
            PIN(en_out) = 0.0;
          }
        }
      }
      break;
    }

    case 14: {  // rotor time constant and magnetizing inductance, by d current steps
      // Hold com_pos and step the d current between test_cur and test_cur/2
      // with a fast current loop. The rotor flux follows the stator current
      // with tr = Lr/Rr, and while it moves the stator sees its emf
      //
      //   e = Lmr * d(i_mr)/dt,   tr * d(i_mr)/dt = id - i_mr,   Lmr = Lm^2/Lr
      //
      // on top of r*id, the dead time and l*d(id)/dt. The first two settle with
      // the current; the chord from the r test is their dc slope over exactly
      // these two levels, so it takes them out of the part that has not.
      //
      // The current is not a clean step: the tail pushes against the loop and
      // id sags for as long as the integrator takes to catch it, and that sag
      // moves the flux too. Fitting an exponential to ud alone came out 3% short
      // on tr with a 1500 rad/s loop and 10% at 300 (simulated). So fit the
      // model to the current that actually flowed. With everything integrated
      // from the edge, where i_mr still sits at the old level:
      //
      //   lam(t) = integral(ud - uinf - R (id - iinf)) - l (id - i0) = Lmr (i_mr - i0)
      //   integral(lam) = Lmr * (integral(id - iinf) + di * t) - tr * lam
      //
      // which is linear in Lmr and tr, fitted by least squares over rot_t0 to
      // the settled tail. Simulated against a T model with dead time, noise,
      // a 300 to 1500 rad/s loop and 20% error in R, it lands within 1% on
      // 49 ms and 147 ms; an l 50% wrong costs 2-4%, and a feedback delay of
      // two extra ticks between ud_fb and id_fb about 2%.
      //
      // uinf and iinf come from the previous visit to the same level, so the
      // integrals can run from the edge. The first edge magnetizes from zero
      // and the second has no earlier visit to its level: both are dropped.
      // slip_n in acim_ttc is 1/tr in electrical rad/s.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = MAX(PIN(rot_bw), 1.0);
      PIN(com_pos)  = 0.0;
      PIN(q_cmd)    = 0.0;

      float half = MAX(PIN(rot_half), 0.1);
      float t0   = CLAMP(PIN(rot_t0), period, half * 0.25);
      float t1   = half * ROT_TAIL;
      int lv     = ctx->r_edge & 1;  // 0 at test_cur, 1 at half of it
      float rin  = PIN(r_2p) > 0.0 ? PIN(r_2p) : PIN(r);

      PIN(d_cmd) = lv ? PIN(test_cur) * 0.5 : PIN(test_cur);

      float ud = PIN(ud_fb);
      float id = PIN(id_fb);

      if(ctx->r_t == 0.0) {  // first tick of this level
        ctx->act = ctx->have_lvl[0] && ctx->have_lvl[1];
        if(ctx->act) {
          ctx->uinf    = ctx->lvl_u[lv];
          ctx->iinf    = ctx->lvl_i[lv];
          ctx->i0      = ctx->lvl_i[1 - lv];
          ctx->di      = ctx->iinf - ctx->i0;
          ctx->act     = ABS(ctx->di) > 0.2 * PIN(test_cur) * 0.5;
          ctx->have_t0 = 0;
          ctx->lam_raw = ctx->ii = ctx->jj = 0.0;
          ctx->aa = ctx->ab = ctx->bb = ctx->ay = ctx->by = 0.0;
        }
      }
      ctx->r_t += period;
      float t = ctx->r_t;

      if(ctx->act) {
        ctx->lam_raw += (ud - ctx->uinf - rin * (id - ctx->iinf)) * period;
        ctx->ii += (id - ctx->iinf) * period;
        float lam = ctx->lam_raw - PIN(l) * (id - ctx->i0);
        ctx->jj += lam * period;
        if(t >= t0 && t < t1) {
          if(!ctx->have_t0) {
            ctx->i_t0    = id;
            ctx->have_t0 = 1;
          }
          // scaled by the step, so rising and falling edges fit alike and
          // the sums stay near unity
          float xa = ctx->ii / ctx->di + t;
          float xb = -lam / ctx->di;
          float y  = ctx->jj / ctx->di;
          ctx->aa += xa * xa;
          ctx->ab += xa * xb;
          ctx->bb += xb * xb;
          ctx->ay += xa * y;
          ctx->by += xb * y;
        }
      }
      if(t >= t1) {
        ctx->eu += ud;
        ctx->ei += id;
        ctx->e_n++;
      }

      if(t >= half) {  // end of this level
        if(ctx->act) {
          float det = ctx->aa * ctx->bb - ctx->ab * ctx->ab;
          if(det > 0.0) {
            float lm = (ctx->ay * ctx->bb - ctx->by * ctx->ab) / det;
            float tr = (ctx->by * ctx->aa - ctx->ay * ctx->ab) / det;
            if(tr > 0.0 && lm > 0.0) {
              ctx->tr_sum += tr;
              ctx->lm_sum += lm;
              ctx->tr_min = ctx->r_n ? MIN(ctx->tr_min, tr) : tr;
              ctx->tr_max = ctx->r_n ? MAX(ctx->tr_max, tr) : tr;
              ctx->dip    = MAX(ctx->dip, ABS((ctx->i_t0 - ctx->iinf) / ctx->di));
              ctx->r_n++;
            }
          }
        }
        if(ctx->e_n > 0) {
          ctx->lvl_u[lv]    = ctx->eu / (float)ctx->e_n;
          ctx->lvl_i[lv]    = ctx->ei / (float)ctx->e_n;
          ctx->have_lvl[lv] = 1;
        }
        ctx->eu = ctx->ei = 0.0;
        ctx->e_n = 0;
        ctx->r_edge++;
        ctx->r_t = 0.0;
      }

      if(ctx->r_edge >= 1 + 2 * (int)MAX(PIN(rot_cycles), 1.0)) {
        float tr_ok  = 0.0;
        PIN(rot_n)   = ctx->r_n;
        PIN(rot_dip) = ctx->dip;
        PIN(tr)      = 0.0;
        PIN(slip_n)  = 0.0;
        PIN(lmr)     = 0.0;
        PIN(ls)      = 0.0;
        PIN(tr_spread) = 0.0;
        if(ctx->r_n >= 2) {
          float n  = (float)ctx->r_n;
          float tr = ctx->tr_sum / n;
          PIN(tr)  = tr;  // reported either way, so a rejected fit still says what it was
          // a fit that lands outside the window it was taken over is not an
          // exponential this test can see
          if(tr > 5.0 * period && tr < t1) {
            PIN(tr)        = tr;
            PIN(slip_n)    = 1.0 / tr;
            PIN(lmr)       = ctx->lm_sum / n;
            PIN(ls)        = PIN(l) + PIN(lmr);
            PIN(tr_spread) = (ctx->tr_max - ctx->tr_min) / tr;
            tr_ok          = 1.0;
          }
        }
        PIN(tr_ok)  = tr_ok;
        PIN(timer)  = 0.0;
        PIN(state)  = 1.5;
        PIN(d_cmd)  = 0.0;
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
