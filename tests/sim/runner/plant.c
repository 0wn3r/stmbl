// The motor, inverter and load the firmware drives.
//
// Electrical: a PMSM in its own dq frame (r, ld, lq, psi, pp). The inverter
// turns the F3's compares into phase voltages averaged over each PWM period
// (switching ripple is not modelled), less the dead-time loss: per phase
// udc * dt_eff / T_pwm * tanh(i / dt_i0), the sign of the phase current with
// the ripple's softening at zero, plus an optional device drop. With the
// gates off (MOE clear) the phases freewheel through the diodes, or float
// once the currents have died and the back emf is below the link.
//
// Mechanical: motor inertia jm and load inertia jl, joined by a spring k and
// damper c (k = 0: one rigid body jm + jl). Coulomb f and viscous d act on
// the load, fm and dm on the motor; load is an external torque on the load.
//
// The firmware's electrical angle is sigma * pp * (encoder + offset), with
// sigma = +-1 from out_rev and mot_fb_rev and the offset from
// conf0.mot_fb_offset (runner sets both with plant_frame), so a correctly
// configured drive is correctly phased; off_err [rad el] mis-phases it.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_hw.h"
#include "f3hw.h"
#include "runner.h"

sim_f3_adc_t sim_f3_adc;
sim_tim_t sim_tim8;

typedef struct {
  const char *key;
  double v;
  int set;
  const char *doc;
} param_t;

static param_t P[] = {
    {"r", 0.75, 0, "phase resistance [ohm]"},
    {"ld", 0.0015, 0, "d inductance [H]"},
    {"lq", 0.0015, 0, "q inductance [H]"},
    {"psi", 0.05, 0, "flux linkage [Vs]"},
    {"pp", 4, 0, "pole pairs"},
    {"udc", 295, 0, "link voltage [V]"},
    {"udc_r", 0, 0, "link source resistance [ohm]"},
    {"dt_eff", 2.0e-6 * 0.94, 0, "effective dead time [s]"},
    {"dt_i0", 0.3, 0, "phase current where the dead-time loss is 76% of full [A]"},
    {"vdrop", 0, 0, "device drop [V]"},
    {"adc_lsb", 0.0176, 0, "current ADC step [A] (3 mOhm shunt, gain 16)"},
    {"adc_noise", 0, 0, "current ADC noise per sample [A rms]"},
    {"adc_off_u", 0, 0, "current offset u [A] (the F3 learns it at boot)"},
    {"adc_off_v", 0, 0, "current offset v [A]"},
    {"adc_off_w", 0, 0, "current offset w [A]"},
    {"oc_dac0", 144, 0, "comparator dac word at 0 A"},
    {"oc_k", 6.53, 0, "comparator dac words per A, 0 = no comparator"},
    {"jm", 0.000025, 0, "motor inertia [kg m2]"},
    {"jl", 0, 0, "load inertia [kg m2]"},
    {"k", 0, 0, "coupling stiffness [Nm/rad], 0 = rigid"},
    {"c", 0, 0, "coupling damping [Nm s/rad]"},
    {"ring_hz", 0, 0, "sets k for this two-mass ring frequency [Hz]"},
    {"ring_zeta", 0.2, 0, "sets c for this ring damping, with ring_hz"},
    {"f", 0, 0, "coulomb friction on the load [Nm]"},
    {"d", 0, 0, "viscous friction on the load [Nm s/rad]"},
    {"fm", 0, 0, "coulomb friction on the motor [Nm]"},
    {"dm", 0, 0, "viscous friction on the motor [Nm s/rad]"},
    {"w_s", 0.01, 0, "below this speed coulomb friction sticks [rad/s]"},
    {"load", 0, 0, "external torque on the load [Nm]"},
    {"lead", 10, 0, "screw lead for pos_mm [mm/rev]"},
    {"off_err", 0, 0, "commutation error [rad el]"},
    {"enc_delay", 0, 0, "encoder sample age at the F4 rt [s]"},
    {"enc_bits", 22, 0, "encoder bits per turn"},
    {"hv_temp", 40, 0, "IPM case temperature [C]"},
    {"substeps", 16, 0, "integration steps per PWM period"},
};
#define NP (sizeof(P) / sizeof(P[0]))

static double *pp_(const char *k) {
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, k)) {
      return &P[i].v;
    }
  }
  fprintf(stderr, "plant: no parameter %s\n", k);
  exit(2);
}
// each use looks its parameter up once
#define PV(k) (*({              \
  static double *p_;            \
  if(!p_) p_ = pp_(#k);         \
  p_;                           \
}))

double plant_enc_delay;
double plant_enc_bits;

// state
static double id, iq;      // plant dq frame [A]
static double thm, wm;     // motor [rad, rad/s], multi-turn
static double thl, wl;     // load
static double tm, ts;      // motor torque, shaft torque [Nm]
static double vu, vv, vw;  // phase voltages to ground, last step [V]
static double ud, uq;      // applied dq voltage [V]
static double udc_now;
static double sigma = 1.0, mech_off = 0.0;
static uint32_t cu, cv, cw, carr = PWM_RES;  // compares in force
static uint32_t nu, nv, nw, narr = PWM_RES;  // compares for the next period
static double t_plant;

#define HIST 4096
static double hist_t[HIST], hist_th[HIST];
static int hist_n;

void plant_init(void) {
  id = iq = thm = wm = thl = wl = tm = ts = 0.0;
  vu = vv = vw = ud = uq = 0.0;
  cu = cv = cw = nu = nv = nw = 0;
  carr = narr = PWM_RES;
  hist_n      = 0;
  t_plant     = 0.0;
  memset(&sim_tim8, 0, sizeof(sim_tim8));
  sim_tim8.ARR = PWM_RES;
}

int plant_set(const char *key, double v) {
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, key)) {
      P[i].v   = v;
      P[i].set = 1;
      return 1;
    }
  }
  if(!strcmp(key, "pos")) {  // place the shaft
    thm = thl = v;
    return 1;
  }
  return 0;
}

int plant_was_set(const char *key) {
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, key)) {
      return P[i].set;
    }
  }
  return 0;
}

void plant_print_params(void) {
  for(unsigned i = 0; i < NP; i++) {
    printf("  plant.%-10s %-12g %s%s\n", P[i].key, P[i].v, P[i].doc, P[i].set ? "" : " (default)");
  }
}

void plant_frame(double s, double off) {
  sigma    = s;
  mech_off = off;
}

void plant_pwm(uint32_t u, uint32_t v, uint32_t w, uint32_t arr) {
  nu   = u;
  nv   = v;
  nw   = w;
  narr = arr;
}

static double th_el(void) {
  return sigma * PV(pp) * (thm + mech_off) + PV(off_err);
}

static void abc(double th, double *iu, double *iv, double *iw) {
  double s = sin(th), c = cos(th);
  double ia = id * c - iq * s;
  double ib = id * s + iq * c;
  *iu       = ia;
  *iv       = -0.5 * ia + 0.5 * sqrt(3.0) * ib;
  *iw       = -0.5 * ia - 0.5 * sqrt(3.0) * ib;
}

static double gauss(void) {
  double a = (rand() + 1.0) / (RAND_MAX + 2.0), b = (rand() + 1.0) / (RAND_MAX + 2.0);
  return sqrt(-2.0 * log(a)) * cos(2.0 * M_PI * b);
}

// one phase leg's average voltage to ground over a PWM period
static double leg(uint32_t cmp, uint32_t arr, double i, double udc) {
  double soft = tanh(i / fmax(PV(dt_i0), 1e-6));
  double v;
  if(cmp == 0) {
    v = 0.0;
  } else if(cmp >= arr) {
    v = udc;
  } else {
    double t_pwm = 2.0 * arr / PWM_TIM_CLK;
    v            = udc * (double)cmp / arr - udc * PV(dt_eff) / t_pwm * soft;
  }
  return v - PV(vdrop) * soft;
}

// Coulomb friction f on a body at speed v with the other torques tn on it.
// Moving (|v| > w_s): f against v. Inside w_s it is stiction (Karnopp): it
// cancels tn up to f and takes out the remaining speed within one step h, so
// a body held by friction stays put instead of creeping.
static double coulomb(double f, double v, double tn, double j, double h) {
  if(f <= 0.0) {
    return 0.0;
  }
  if(fabs(v) > PV(w_s)) {
    return v > 0.0 ? f : -f;
  }
  double need = tn + j * v / h;
  return need > f ? f : need < -f ? -f : need;
}

static void deriv(const double *x, double *dx, double h) {
  // x: id iq thm wm thl wl, in the plant's dq frame
  double _id = x[0], _iq = x[1];
  double r = PV(r), ld = PV(ld), lq = PV(lq), psi = PV(psi), pp = PV(pp);
  double we = sigma * pp * x[3];
  dx[0]     = (ud - r * _id + we * lq * _iq) / ld;
  dx[1]     = (uq - r * _iq - we * (ld * _id + psi)) / lq;
  double te = 1.5 * pp * (psi * _iq + (ld - lq) * _id * _iq);
  double k = PV(k), c = PV(c), jm = PV(jm), jl = PV(jl);
  double tq = sigma * te;
  if(k > 0.0 && jl > 0.0) {
    double sh = k * (x[2] - x[4]) + c * (x[3] - x[5]);
    double tm_n = tq - sh - PV(dm) * x[3];
    double tl_n = sh + PV(load) - PV(d) * x[5];
    dx[2]     = x[3];
    dx[3]     = (tm_n - coulomb(PV(fm), x[3], tm_n, jm, h)) / jm;
    dx[4]     = x[5];
    dx[5]     = (tl_n - coulomb(PV(f), x[5], tl_n, jl, h)) / jl;
    ts        = sh;
  } else {
    double j  = jm + jl;
    double tn = tq + PV(load) - (PV(dm) + PV(d)) * x[3];
    dx[2]     = x[3];
    dx[3]     = (tn - coulomb(PV(fm) + PV(f), x[3], tn, j, h)) / j;
    dx[4]     = dx[2];
    dx[5]     = dx[3];
    ts        = tq - jm * dx[3];
  }
  tm = tq;
}

static void step(double dt) {
  double th = th_el();
  double we = sigma * PV(pp) * wm;
  double iu, iv, iw;
  abc(th, &iu, &iv, &iw);

  // link sag from the drive's own draw
  double udc = PV(udc);
  double p   = vu * iu + vv * iv + vw * iw;
  if(PV(udc_r) > 0.0 && udc > 1.0) {
    udc -= PV(udc_r) * p / udc;
  }
  udc_now = udc;

  // comparators: instantaneous, on any phase
  if(sim_tim8.moe && PV(oc_k) > 0.0) {
    double thr  = (sim_tim8.dac - PV(oc_dac0)) / PV(oc_k);
    uint32_t cm = (fabs(iu) > thr) | (fabs(iv) > thr) << 1 | (fabs(iw) > thr) << 2;
    sim_tim8.cmp = cm;
    if(cm) {
      sim_tim8.moe = 0;
      sim_tim8.brk = 1;
    }
  }

  double s = sin(th), c = cos(th);
  if(sim_tim8.moe) {
    vu = leg(cu, carr, iu, udc);
    vv = leg(cv, carr, iv, udc);
    vw = leg(cw, carr, iw, udc);
  } else {
    // back emf per phase, to see whether the diodes conduct
    double ea = -we * PV(psi) * s, eb = we * PV(psi) * c;
    double eu = ea, ev = -0.5 * ea + 0.5 * sqrt(3.0) * eb, ew = -0.5 * ea - 0.5 * sqrt(3.0) * eb;
    double ell = fmax(fabs(eu - ev), fmax(fabs(ev - ew), fabs(ew - eu)));
    if(fabs(iu) < 0.01 && fabs(iv) < 0.01 && fabs(iw) < 0.01 && ell < udc) {
      id = iq = 0.0;  // open: the phases float on the back emf
      vu      = eu + udc / 2.0;
      vv      = ev + udc / 2.0;
      vw      = ew + udc / 2.0;
      ud = uq = 0.0;
      goto mech;
    }
    vu = iu > 0.0 ? 0.0 : udc;
    vv = iv > 0.0 ? 0.0 : udc;
    vw = iw > 0.0 ? 0.0 : udc;
  }
  {
    double va = (2.0 / 3.0) * (vu - 0.5 * (vv + vw));
    double vb = (vv - vw) / sqrt(3.0);
    ud        = va * c + vb * s;
    uq        = -va * s + vb * c;
  }
mech:;
  // RK2 (midpoint), voltages held
  double x[6] = {id, iq, thm, wm, thl, wl}, k1[6], k2[6], xm[6];
  deriv(x, k1, dt);
  for(int i = 0; i < 6; i++) {
    xm[i] = x[i] + 0.5 * dt * k1[i];
  }
  deriv(xm, k2, dt);
  if(sim_tim8.moe || ud != 0.0 || uq != 0.0 || id != 0.0 || iq != 0.0) {
    id += dt * k2[0];
    iq += dt * k2[1];
  }
  thm += dt * k2[2];
  wm += dt * k2[3];
  thl += dt * k2[4];
  wl += dt * k2[5];
  // a rigid pair moves as one
  if(!(PV(k) > 0.0 && PV(jl) > 0.0)) {
    thl = thm;
    wl  = wm;
  }
}

static void apply_ring(void) {
  // two-mass resonance f = sqrt(k (jm + jl) / (jm jl)) / 2 pi
  if(PV(ring_hz) > 0.0 && PV(jl) > 0.0) {
    double jr = PV(jm) * PV(jl) / (PV(jm) + PV(jl));
    double w  = 2.0 * M_PI * PV(ring_hz);
    PV(k)     = w * w * jr;
    PV(c)     = 2.0 * PV(ring_zeta) * w * jr;
  }
}

void plant_advance(double dt) {
  if(dt <= 0.0) {
    return;
  }
  apply_ring();
  double h = 1.0 / PWM_FREQ / fmax(PV(substeps), 1.0);
  int n    = (int)ceil(dt / h - 1e-9);
  double d = dt / n;
  for(int i = 0; i < n; i++) {
    step(d);
    t_plant += d;
    int j      = hist_n++ % HIST;
    hist_t[j]  = t_plant;
    hist_th[j] = thm;
  }
}

void plant_sample_adc(void) {
  // the compares the F3 wrote last tick take over at this update event
  cu   = nu;
  cv   = nv;
  cw   = nw;
  carr = narr;

  double iu, iv, iw;
  abc(th_el(), &iu, &iv, &iw);
  double lsb = PV(adc_lsb), nz = PV(adc_noise);
  double in[3]  = {iu + PV(adc_off_u), iv + PV(adc_off_v), iw + PV(adc_off_w)};
  double out[3] = {0, 0, 0};
  for(int p = 0; p < 3; p++) {
    for(int k = 0; k < ADC_CUR_SAMPLES; k++) {  // averaged like io.c's sum
      double x = in[p] + (nz > 0.0 ? nz * gauss() : 0.0);
      out[p] += (lsb > 0.0 ? lsb * floor(x / lsb + 0.5) : x) / ADC_CUR_SAMPLES;
    }
  }
  sim_f3_adc.iu      = out[0];
  sim_f3_adc.iv      = out[1];
  sim_f3_adc.iw      = out[2];
  sim_f3_adc.ur      = vu;
  sim_f3_adc.vr      = vv;
  sim_f3_adc.wr      = vw;
  sim_f3_adc.udc     = udc_now > 0.0 ? udc_now : PV(udc);
  sim_f3_adc.hv_temp = PV(hv_temp);
}

void plant_encoder(double delay, double *mech) {
  if(hist_n == 0 || delay <= 0.0) {
    *mech = thm;
    return;
  }
  double t = t_plant - delay;
  int last = hist_n - 1, first = hist_n > HIST ? hist_n - HIST : 0;
  for(int i = last; i > first; i--) {
    int a = (i - 1) % HIST, b = i % HIST;
    if(hist_t[a] <= t) {
      double f = (t - hist_t[a]) / fmax(hist_t[b] - hist_t[a], 1e-12);
      *mech    = hist_th[a] + f * (hist_th[b] - hist_th[a]);
      return;
    }
  }
  *mech = hist_th[first % HIST];
}

double plant_get(const char *key, int *ok) {
  *ok = 1;
  double iu, iv, iw;
  if(!strcmp(key, "id")) return id;
  if(!strcmp(key, "iq")) return iq;
  if(!strcmp(key, "ud")) return ud;
  if(!strcmp(key, "uq")) return uq;
  if(!strcmp(key, "iu") || !strcmp(key, "iv") || !strcmp(key, "iw")) {
    abc(th_el(), &iu, &iv, &iw);
    return key[1] == 'u' ? iu : key[1] == 'v' ? iv : iw;
  }
  if(!strcmp(key, "pos")) return thm;
  if(!strcmp(key, "vel")) return wm;
  if(!strcmp(key, "load_pos")) return thl;
  if(!strcmp(key, "load_vel")) return wl;
  if(!strcmp(key, "twist")) return thm - thl;
  if(!strcmp(key, "pos_mm")) return thl / (2.0 * M_PI) * PV(lead);
  if(!strcmp(key, "vel_mm")) return wl / (2.0 * M_PI) * PV(lead);
  if(!strcmp(key, "torque")) return tm;
  if(!strcmp(key, "shaft")) return ts;
  if(!strcmp(key, "udc")) return udc_now;
  if(!strcmp(key, "moe")) return sim_tim8.moe;
  if(!strcmp(key, "brk")) return sim_tim8.brk;
  if(!strcmp(key, "th_el")) return remainder(th_el(), 2.0 * M_PI);
  for(unsigned i = 0; i < NP; i++) {
    if(!strcmp(P[i].key, key)) {
      return P[i].v;
    }
  }
  *ok = 0;
  return 0.0;
}
