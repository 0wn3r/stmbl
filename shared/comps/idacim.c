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
HAL_PIN(tr);          // rotor time constant Lr/Rr, mean of tr_rise and tr_fall [s]
HAL_PIN(slip_n);      // 1/tr, acim_ttc's slip constant [rad/s electrical]
HAL_PIN(lmr);         // rotor side magnetizing inductance Lm^2/Lr, median over edges [H]
HAL_PIN(ls);          // stator inductance l + lmr [H]
HAL_PIN(rot_n);       // edges that went into tr and lmr
HAL_PIN(rot_dip);     // largest current error at rot_t0, fraction of the step
HAL_PIN(tr_ok);       // 1 = tr, slip_n and lmr are measurements
HAL_PIN(tr_spread);   // (max - min) / median of tr, the worse of the two directions
HAL_PIN(tr_rise);     // median tr of the edges up to test_cur [s]
HAL_PIN(tr_fall);     // median tr of the edges down to test_cur/2 [s]
HAL_PIN(tr_min);      // shortest edge [s]
HAL_PIN(tr_max);      // longest edge [s]

HAL_PIN(lad_top);     // *parameter*, offset ladder, top current [A], 0 = no ladder
HAL_PIN(lad_n);       // *parameter*, offset ladder, rungs (at most 4), rung k steps between top k/n and half of it
HAL_PINA(lad_i, 4);   // rung's upper current [A]
HAL_PINA(lad_lm, 4);  // rung's lmr, the slope of rotor flux over the rung [H]
HAL_PINA(lad_tr, 4);  // rung's tr [s]
HAL_PINA(lad_sp, 4);  // rung's tr spread
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

#define ROT_BLK 8     // rotor test: the settled tail is read as the median of this many blocks
#define ROT_EDGES 32  // rotor test: edges kept per rung, so rot_cycles is at most 16
#define ROT_DEC 8     // rotor test: the fit sums take every this many ticks

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
  uint16_t r_edge;      // edges seen in this rung, the first comes from another level
  uint16_t r_n;         // edges that produced a fit (nrt)
  uint16_t r_seen;      // edges handed to nrt, fitted or not
  uint16_t dec;         // ticks since the fit sums were last fed
  uint8_t rung;         // 0 = the test at test_cur, 1.. = ladder rungs
  uint8_t act;          // this edge is being fitted
  uint8_t have_prev;    // the previous edge's settled values are known
  uint8_t have_t0;      // i_t0 captured
  float r_t;            // time since the last commanded edge
  float prev_u, prev_i; // where the previous edge settled
  float c0;             // offset taken out while integrating, ud - R id where the last edge settled
  float i0;             // settled current before the step
  float i_t0;           // id_fb at rot_t0
  float p;              // integral of ud - R id - c0 since the edge
  float q;              // integral of id - i0 since the edge
  float jp;             // integral of p
  float u1, u2, i1, i2; // the two samples before this one, for a median of three
  float s[14];          // fit sums, kept as a polynomial in this edge's own offset
  float blk_u[ROT_BLK], blk_i[ROT_BLK];  // settled tail, sums per block
  uint16_t blk_n[ROT_BLK];
  // rt hands each finished edge and rung to nrt, which does the solving and
  // the medians: in rt they cost the f4 more than its slack (bench, 5 Oct)
  float ps[14];         // the edge's sums
  float pd;             // its offset correction c - c0, from the tail
  float pdip;           // its |current error at rot_t0| / step
  uint8_t pup;          // 1 = it went up to the upper level
  float pdi;            // its step, settled to settled [A]
  float ptail;          // max - min of its tail's block means of ud [V]
  volatile uint8_t pend;   // an edge waits for nrt
  volatile uint8_t rdone;  // a rung waits for nrt
  uint8_t rd_rung;      // which rung
  float rd_hi;          // its upper current
  float rd_t1;          // where its settled tail starts, longest tr it can see
  float rd_per;         // rt period
  // nrt's
  float tr_e[ROT_EDGES], lm_e[ROT_EDGES];  // per edge fits
  uint8_t up_e[ROT_EDGES];                 // 1 = the edge went up to the upper level
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
  PIN(lad_top)                  = 0.0;
  PIN(lad_n)                    = 4.0;
  PIN(cur_bw)                   = 1.0;
}

static float median(float *v, int n) {  // sorts v
  for(int i = 1; i < n; i++) {
    float x = v[i];
    int j   = i - 1;
    while(j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  if(n <= 0) {
    return 0.0;
  }
  return n & 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

static float med3(float a, float b, float c) {
  return MAX(MIN(a, b), MIN(MAX(a, b), c));
}

// ctx survives a stop
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx = (struct idacim_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idacim_ctx_t));
}

// x' G y over the basis (a, bk, t, sg, yk) the fit sums are kept in
static double rot_dot(const double g[5][5], const double *x, const double *y) {
  double r = 0.0;
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      r += x[i] * g[i][j] * y[j];
    }
  }
  return r;
}

// The rotor test's arithmetic, out of rt. Each edge's fit is solved here in
// double, from the sums rt collected; at the end of a rung the edges are
// reduced to medians.
//
// The edge's offset c is fitted with Lmr and tr, not read off the tail: an
// error e in c grows as e t in lam, and at 16 A one mV of it moved lmr 2% and
// tr with it (bench, 5 Oct; the tail's median carried a few mV). With
// y = yk + d sg and b = bk + d t the model y = Lmr a + tr b is linear in Lmr
// and tr but not in d, so Gauss-Newton from the tail's d, which is close.
// The settled part pins d down through the curvature d t^2/2 it leaves in the
// integral, over the whole edge instead of its last quarter.
static void rot_nrt(struct idacim_ctx_t *ctx, struct idacim_pin_ctx_t *pins) {
  if(ctx->pend) {
    float tr = 0.0, lm = 0.0, dfit = 0.0;
    int ok   = 0;
    if(ctx->r_n < ROT_EDGES) {
      float *s       = ctx->ps;
      double g[5][5] = {
          {s[0], s[1], s[2], s[7], s[6]},
          {s[1], s[3], s[4], s[9], s[8]},
          {s[2], s[4], s[5], s[11], s[10]},
          {s[7], s[9], s[11], s[12], s[13]},
          {s[6], s[8], s[10], s[13], 0.0},  // yk yk is not needed
      };
      // start: Lmr and tr at the tail's d
      double d   = ctx->pd;
      double ea[5] = {1, 0, 0, 0, 0};
      double eb[5] = {0, 1, d, 0, 0};
      double ey[5] = {0, 0, 0, d, 1};
      double aa  = rot_dot(g, ea, ea);
      double ab  = rot_dot(g, ea, eb);
      double bb  = rot_dot(g, eb, eb);
      double ay  = rot_dot(g, ea, ey);
      double by  = rot_dot(g, eb, ey);
      double det = aa * bb - ab * ab;
      ok         = det > 0.0;
      double x0  = ok ? (ay * bb - by * ab) / det : 0.0;
      double x1  = ok ? (by * aa - ay * ab) / det : 0.0;
      for(int it = 0; it < 6 && ok; it++) {
        // residual y - model and the model's derivatives in (Lmr, tr, d)
        double r[5]    = {-x0, -x1, -x1 * d, d, 1};
        double j[3][5] = {
            {1, 0, 0, 0, 0},
            {0, 1, d, 0, 0},
            {0, 0, x1, -1, 0},
        };
        double m[3][3], v[3];
        for(int k = 0; k < 3; k++) {
          v[k] = rot_dot(g, j[k], r);
          for(int l = 0; l < 3; l++) {
            m[k][l] = rot_dot(g, j[k], j[l]);
          }
        }
        double dm = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if(!(dm > 0.0)) {
          ok = 0;
          break;
        }
        double s0 = (v[0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (v[1] * m[2][2] - m[1][2] * v[2]) + m[0][2] * (v[1] * m[2][1] - m[1][1] * v[2])) / dm;
        double s1 = (m[0][0] * (v[1] * m[2][2] - m[1][2] * v[2]) - v[0] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * v[2] - v[1] * m[2][0])) / dm;
        double s2 = (m[0][0] * (m[1][1] * v[2] - v[1] * m[2][1]) - m[0][1] * (m[1][0] * v[2] - v[1] * m[2][0]) + v[0] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / dm;
        x0 += s0;
        x1 += s1;
        d += s2;
      }
      if(ok) {
        lm   = (float)x0;
        tr   = (float)x1;
        dfit = (float)d;
        ok = tr > 0.0 && lm > 0.0;
        if(ok) {
          ctx->tr_e[ctx->r_n] = tr;
          ctx->lm_e[ctx->r_n] = lm;
          ctx->up_e[ctx->r_n] = ctx->pup;
          ctx->dip            = MAX(ctx->dip, ctx->pdip);
          ctx->r_n++;
        }
      }
    }
    // one line per edge, to see whether the scatter between edges is noise
    // or follows something: direction, step, offset correction, tail noise
    ctx->r_seen++;
    printf("# edge %f %s tr %f ms lmr %f mH d %f V (tail %f) di %f A dip %f tail %f V%s\n", (float)ctx->r_seen, ctx->pup ? "up" : "dn", tr * 1000.0, lm * 1000.0, dfit, ctx->pd, ctx->pdi, ctx->pdip, ctx->ptail, ok ? "" : " rejected");
    ctx->pend = 0;
  }
  if(ctx->rdone && !ctx->pend) {
    int n    = ctx->r_n;
    float tr = 0.0, lm = 0.0, sp = 0.0, tmin = 0.0, tmax = 0.0, trise = 0.0, tfall = 0.0;
    int ok   = 0;
    if(n >= 2) {
      // Rising and falling edges are taken apart: over a step that
      // reaches into saturation the flux is not linear in i_mr, and the
      // fit then reads one direction long and the other short by about
      // the same amount (simulated, 8 <-> 16 A on a knee at 12 A: 82 and
      // 123 ms on 100). The mean of the two medians cancels that, and the
      // spread is the worse of the two directions' own, so it says how
      // repeatable the edges are, not how saturated the step is.
      float v[ROT_EDGES];
      float med[2] = {0.0, 0.0};
      int cnt[2]   = {0, 0};
      for(int up = 0; up < 2; up++) {
        int m = 0;
        for(int k = 0; k < n; k++) {
          if(ctx->up_e[k] == up) {
            v[m++] = ctx->tr_e[k];
          }
        }
        med[up] = median(v, m);  // sorted now
        cnt[up] = m;
        if(m > 0) {
          tmin = (cnt[0] + cnt[1] == m) ? v[0] : MIN(tmin, v[0]);
          tmax = (cnt[0] + cnt[1] == m) ? v[m - 1] : MAX(tmax, v[m - 1]);
          if(med[up] > 0.0) {
            sp = MAX(sp, (v[m - 1] - v[0]) / med[up]);
          }
        }
      }
      tfall = med[0];
      trise = med[1];
      tr    = cnt[0] && cnt[1] ? 0.5 * (trise + tfall) : (cnt[1] ? trise : tfall);
      // lmr from the falling edges. Rising ones read it 20% under them at
      // 12 and 16 A, and with 3 s edges instead of 1.5 their lmr spread
      // 7.5-11.6 mH and two of four fits failed, while falling ones held
      // 13.2-13.8, the at-speed 13.2 (bench, 5 Oct): something slow at the
      // upper level (heating, or flux creeping near saturation) bends the
      // edges that end there. A falling edge ends at the lower level, where
      // both are smallest. Rising edges only when no falling one fitted.
      int lup = cnt[0] ? 0 : 1;
      int m   = 0;
      for(int k = 0; k < n; k++) {
        if(ctx->up_e[k] == lup) {
          v[m++] = ctx->lm_e[k];
        }
      }
      lm = median(v, m);
      // a fit that lands outside the window it was taken over is not an
      // exponential this test can see
      ok = tr > 5.0 * ctx->rd_per && tr < ctx->rd_t1;
    }
    if(ctx->rd_rung == 0) {
      PIN(rot_n)     = n;
      PIN(rot_dip)   = ctx->dip;
      PIN(tr)        = tr;  // reported either way, so a rejected fit still says what it was
      PIN(tr_rise)   = trise;
      PIN(tr_fall)   = tfall;
      PIN(tr_min)    = tmin;
      PIN(tr_max)    = tmax;
      PIN(tr_spread) = sp;
      PIN(slip_n)    = ok ? 1.0 / tr : 0.0;
      PIN(lmr)       = ok ? lm : 0.0;
      PIN(ls)        = ok ? PIN(l) + lm : 0.0;
      PIN(tr_ok)     = ok ? 1.0 : 0.0;
      for(int k = 0; k < 4; k++) {
        PINA(lad_i, k)  = 0.0;
        PINA(lad_lm, k) = 0.0;
        PINA(lad_tr, k) = 0.0;
        PINA(lad_sp, k) = 0.0;
      }
    } else {
      PINA(lad_i, ctx->rd_rung - 1)  = ctx->rd_hi;
      PINA(lad_lm, ctx->rd_rung - 1) = ok ? lm : 0.0;
      PINA(lad_tr, ctx->rd_rung - 1) = tr;
      PINA(lad_sp, ctx->rd_rung - 1) = sp;
    }

    ctx->r_n    = 0;
    ctx->r_seen = 0;
    ctx->dip    = 0.0;
    ctx->rdone = 0;
  }
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idacim_ctx_t *ctx       = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  rot_nrt(ctx, pins);

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
      if(ctx->pend || ctx->rdone) {  // the last rung is still being reduced
        break;
      }
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
          printf("# from %f edges: up %f ms, down %f ms (median each), min %f max %f ms,\n", PIN(rot_n), PIN(tr_rise) * 1000.0, PIN(tr_fall) * 1000.0, PIN(tr_min) * 1000.0, PIN(tr_max) * 1000.0);
          printf("# spread %f of tr within a direction. up and down apart means\n", PIN(tr_spread));
          printf("# the step reaches into saturation; tr is their mean.</font>\n");
          if(PIN(tr_spread) > 0.25) {
            printf("<font color='red'># edges disagree: check ud_fb noise, or rerun with more rot_cycles.</font>\n");
          }
          printf("<font color='green'>");
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
        if(PIN(lad_top) > 0.0 && PIN(lad_n) >= 1.0) {
          // each rung's lmr is the slope of psi_r over its range; taken at the
          // range's middle and integrated from zero, it gives psi_r and the
          // secant psi_r / id that acim_flux's lmr is. Below the first rung
          // the slope is extended along the line through the first two.
          printf("<font color='green'># offset ladder: rung, id range [A], lmr slope [mH], tr [ms], spread, psi_r [Vs], secant lmr [mH]\n");
          int nr   = (int)MIN(PIN(lad_n), 4.0);
          float l0 = 0.0;
          for(int k = 0, j = -1; k < nr; k++) {
            if(PINA(lad_lm, k) > 0.0 && PINA(lad_i, k) > 0.0) {
              if(j < 0) {
                j  = k;
                l0 = PINA(lad_lm, k);
              } else {
                float ma = 0.75 * PINA(lad_i, j), mb = 0.75 * PINA(lad_i, k);
                l0       = PINA(lad_lm, j) + (PINA(lad_lm, j) - PINA(lad_lm, k)) * ma / (mb - ma);
                l0       = MAX(l0, PINA(lad_lm, j));
                break;
              }
            }
          }
          float psi = 0.0, mp = 0.0, lp = l0;
          for(int k = 0; k < nr; k++) {
            float lm = PINA(lad_lm, k);
            float m  = 0.75 * PINA(lad_i, k);
            if(lm <= 0.0 || m <= 0.0) {
              printf("# %f %f..%f no fit (tr %f ms, spread %f)\n", (float)(k + 1), PINA(lad_i, k) * 0.5, PINA(lad_i, k), PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k));
              continue;
            }
            psi += 0.5 * (lp + lm) * (m - mp);
            mp = m;
            lp = lm;
            printf("# %f %f..%f %f %f %f %f %f\n", (float)(k + 1), PINA(lad_i, k) * 0.5, PINA(lad_i, k), lm * 1000.0, PINA(lad_tr, k) * 1000.0, PINA(lad_sp, k), psi, psi / m * 1000.0);
          }
          printf("# psi_r and secant are at each range's middle, 0.75 of its top.</font>\n");
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
      // from the edge, where i_mr still sits at the old level i0, and c the
      // offset ud - R id where this edge settles:
      //
      //   lam(t) = integral(ud - R id - c) - l (id - i0) = Lmr (i_mr - i0)
      //   integral(lam) = Lmr * integral(id - i0) - tr * lam
      //
      // which is linear in Lmr and tr, fitted by least squares from rot_t0 to
      // the end of the edge.
      //
      // c is this edge's own, fitted with them (rot_nrt). Taken from an
      // earlier visit to the level, as it once was, one bad sample or a cage
      // warming between visits put an error in c that grows as t in lam and
      // t^2 in its integral, and edges spread 100-300% on the spindle (2 Oct);
      // read off the edge's last quarter, a few mV of it still spread them
      // 30% (5 Oct). The integration takes out c0, the previous edge's offset
      // (the chord makes it the same at both levels), and the sums are kept as
      // a polynomial in the correction d = c - c0. The median of ROT_BLK block
      // means over the last quarter starts the fit and settles the level for
      // the next edge. A median of three
      // past rot_t0 keeps single bad samples out of the integrals. Each edge
      // is fitted on its own and the median is reported.
      //
      // Ladder rungs (lad_n > 0, lad_top > 0) repeat the same steps between
      // lad_top k/n and half of it after the main test: each rung's lmr is the
      // slope of rotor flux over its range, so together they give psi_r(id).
      // slip_n in acim_ttc is 1/tr in electrical rad/s.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = MAX(PIN(rot_bw), 1.0);
      PIN(com_pos)  = 0.0;
      PIN(q_cmd)    = 0.0;

      int nrung  = (int)CLAMP(PIN(lad_n), 0.0, 4.0);
      float hi   = ctx->rung ? PIN(lad_top) * (float)ctx->rung / (float)MAX(nrung, 1) : PIN(test_cur);
      float half = MAX(PIN(rot_half), 0.1);
      float t0   = CLAMP(PIN(rot_t0), period, half * 0.25);
      float t1   = half * ROT_TAIL;
      int lv     = ctx->r_edge & 1;  // 0 at hi, 1 at half of it
      float rin  = PIN(r_2p) > 0.0 ? PIN(r_2p) : PIN(r);

      PIN(d_cmd) = lv ? hi * 0.5 : hi;

      float ud = PIN(ud_fb);
      float id = PIN(id_fb);

      if(ctx->r_t == 0.0) {  // first tick of this level
        // the first edge of a rung comes from somewhere else: magnetizes only
        ctx->act = ctx->r_edge >= 1 && ctx->have_prev;
        if(ctx->act) {
          ctx->i0      = ctx->prev_i;
          ctx->c0      = ctx->prev_u - rin * ctx->prev_i;
          ctx->have_t0 = 0;
          ctx->p = ctx->q = ctx->jp = 0.0;
          ctx->dec = 0;
          for(int k = 0; k < 14; k++) {
            ctx->s[k] = 0.0;
          }
        }
        for(int k = 0; k < ROT_BLK; k++) {
          ctx->blk_u[k] = ctx->blk_i[k] = 0.0;
          ctx->blk_n[k] = 0;
        }
      }
      ctx->r_t += period;
      float t = ctx->r_t;

      // single bad samples: a median of three once the edge's fast part is over
      float um = ud;
      float im = id;
      if(t > t0) {
        um = med3(ud, ctx->u1, ctx->u2);
        im = med3(id, ctx->i1, ctx->i2);
      }
      ctx->u2 = ctx->u1;
      ctx->u1 = ud;
      ctx->i2 = ctx->i1;
      ctx->i1 = id;

      if(ctx->act) {
        ctx->p += (um - rin * im - ctx->c0) * period;
        ctx->q += (im - ctx->i0) * period;
        ctx->jp += ctx->p * period;
        if(t >= t0) {
          if(!ctx->have_t0) {
            ctx->i_t0    = im;
            ctx->have_t0 = 1;
          }
          if(++ctx->dec >= ROT_DEC) {
            ctx->dec = 0;
            // y = Lmr a + tr b, with b = bk + d t and y = yk - d t^2/2
            float a  = ctx->q;
            float bk = -(ctx->p - PIN(l) * (im - ctx->i0));
            float yk = ctx->jp - PIN(l) * ctx->q;
            float tt = t;
            float sg = -0.5 * t * t;
            ctx->s[0] += a * a;
            ctx->s[1] += a * bk;
            ctx->s[2] += a * tt;
            ctx->s[3] += bk * bk;
            ctx->s[4] += bk * tt;
            ctx->s[5] += tt * tt;
            ctx->s[6] += a * yk;
            ctx->s[7] += a * sg;
            ctx->s[8] += bk * yk;
            ctx->s[9] += bk * sg;
            ctx->s[10] += tt * yk;
            ctx->s[11] += tt * sg;
            ctx->s[12] += sg * sg;
            ctx->s[13] += sg * yk;
          }
        }
      }
      if(t >= t1) {
        int k = (int)((t - t1) / (half - t1) * ROT_BLK);
        k     = CLAMP(k, 0, ROT_BLK - 1);
        ctx->blk_u[k] += um;
        ctx->blk_i[k] += im;
        ctx->blk_n[k]++;
      }

      if(t >= half) {  // end of this level
        float bu[ROT_BLK], bi[ROT_BLK];
        int nb = 0;
        for(int k = 0; k < ROT_BLK; k++) {
          if(ctx->blk_n[k] > 0) {
            bu[nb] = ctx->blk_u[k] / (float)ctx->blk_n[k];
            bi[nb] = ctx->blk_i[k] / (float)ctx->blk_n[k];
            nb++;
          }
        }
        if(nb > 0) {
          float uinf = median(bu, nb);
          float iinf = median(bi, nb);
          float di   = iinf - ctx->i0;
          if(ctx->act && ABS(di) > 0.1 * hi && !ctx->pend) {
            for(int k = 0; k < 14; k++) {
              ctx->ps[k] = ctx->s[k];
            }
            ctx->pd   = uinf - rin * iinf - ctx->c0;
            ctx->pdip = ABS((ctx->i_t0 - iinf) / di);
            ctx->pup  = lv == 0;
            ctx->pdi  = di;
            ctx->ptail = bu[nb - 1] - bu[0];  // median() sorted bu
            ctx->pend = 1;
          }
          ctx->prev_u    = uinf;
          ctx->prev_i    = iinf;
          ctx->have_prev = 1;
        }
        ctx->r_edge++;
        ctx->r_t = 0.0;
      }

      if(ctx->r_edge >= 1 + 2 * (int)CLAMP(PIN(rot_cycles), 1.0, ROT_EDGES / 2)) {
        ctx->rd_rung = ctx->rung;
        ctx->rd_hi   = hi;
        ctx->rd_t1   = t1;
        ctx->rd_per  = period;
        ctx->rdone   = 1;

        if(PIN(lad_top) > 0.0 && ctx->rung < nrung) {  // next rung
          ctx->rung++;
          ctx->r_edge = 0;
          ctx->r_t    = 0.0;
        } else {
          PIN(timer)  = 0.0;
          PIN(state)  = 1.5;
          PIN(d_cmd)  = 0.0;
          PIN(en_out) = 0.0;
        }
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
