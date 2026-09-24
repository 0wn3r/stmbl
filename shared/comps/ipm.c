#include "ipm_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

HAL_COMP(ipm);

/*
The module's NTC sits on the inner PCB, not on the dies, so it lags the
junction by seconds. io.c filters it again, but only lightly: 0.01/0.99 on a
nrt loop running at roughly 600 Hz is about 0.17 s, small against the die's
own time constants. That leaves a hole between the two protections the
bridge already has: an instantaneous 30 A trip that cannot see a sustained
overload below it, and an NTC derate that cannot see a transient above it. A three second deceleration at a
hundred and fifty watts of bridge loss passes through both untouched.

iit.c does this job for the motor, from current alone. Here the case
temperature is measured, so only the rise above it has to be modelled: the
measurement carries the slow part, where it is good, and the model carries
the fast part, where the measurement is blind.

The loss follows the CIPOS Mini application note, per device rather than per
module. Each leg's transistor and its antiparallel diode see the same peak
current but opposite duty: the transistor conducts when the leg is driving
power out, the diode when the load is pushing it back. MI*cos(phi) is what
splits them, and it changes sign under regeneration -- which is why the
diode is modelled at all. Both run their own thermal impedance to the case,
because the note's own heatsink equation puts each junction at the case plus
that one device's dissipation times that one device's Rth(J-C), not at some
average of the six.

About rth_i and rth_d. The IKCM30F60GD fitted to v5.0 is a DCB part,
datasheet Rth(J-C) 1.58 K/W per IGBT and 2.05 K/W per diode, and its own
Zth(J-C) curve, drawn with all six IGBTs heating, settles at 1.74 - only a
tenth above the single chip figure. It has no SOA chart. The IM535-U6D, the
nearest part that has one - also DCB, same package, a little more capable -
is the calibration reference: reproducing its chart at 300 V, MI 0.8, PF 0.8,
Tj 150 with this model and its own datasheet (1.49 and 2.18 K/W) needs 1.40
to 1.60 times Rth over eight points across 5 and 15 kHz, and 1.5 lands every
point within 4 K. The defaults below are the GD's datasheet Rth times 1.5.
The gap between that and the GD's own 1.1 is not explained by chips heating
each other; it is whatever Infineon's chart carries that these mean value
equations do not. With the 1.1 the GD would come out stronger than the
IM535, which it is not. This is the number to trim against the real heatsink,
and the one that decides whether the thresholds mean anything.

Feeding hv_temp in as case_temp is not obviously a source of error. The
note's figure 34 measures the NTC a few kelvin above the datasheet's Tc point
under forced cooling and level with it under convection. That is for a
screwed module on paste; with the heatsink soldered straight to the DCB it
has not been checked.

The Foster shape is fitted to the GD's own Zth(J-C) curve, three poles to
within 0.06 K/W. The GD has no diode curve. The IM535-U6D's pair in AN2022-06
puts its diode ahead of its IGBT by 2.0 times at the fastest pole and 1.45
times at the middle one, and the diode shares here are the GD's IGBT shares
scaled by the same.
Almost all of it has settled within a second; the NTC carries the rest.

Note that f_sw is not free on this hardware. ls.c servos the F3's timer to the
F4's packet rate and hv.c clamps it to a tenth either side, so the 5 kHz
column of the SOA chart is not a remedy that can be reached from a conf file.

Defaults are for the IKCM30F60GD fitted to v5.0.
*/

// in
HAL_PIN(id);         // *input*, hv0.id_fb
HAL_PIN(iq);         // *input*, hv0.iq_fb
HAL_PIN(ud);         // *input*, hv0.ud_fb
HAL_PIN(uq);         // *input*, hv0.uq_fb
HAL_PIN(dc_volt);    // *input*, hv0.dc_volt
HAL_PIN(case_temp);  // *input*, hv0.hv_temp, the module NTC

// device, common
HAL_PIN(f_sw);    // *parameter*, switching frequency (Hz)
HAL_PIN(e_volt);  // *parameter*, link voltage the switching energies were measured at (V)
HAL_PIN(v_tc);    // *parameter*, rise of the conduction drops with junction temperature (1/K)

// transistor
HAL_PIN(vi);     // *parameter*, on state knee voltage at 25 C (V)
HAL_PIN(ri);     // *parameter*, on state slope resistance at 25 C (ohm)
HAL_PIN(ei);     // *parameter*, (Eon + Eoff) per amp switched at e_volt, 25 C (J/A)
HAL_PIN(ei_tc);  // *parameter*, rise of that energy with junction temperature (1/K)
HAL_PIN(rth_i);  // *parameter*, junction to case, see the note above (K/W)

// diode
HAL_PIN(vd);     // *parameter*, forward knee voltage at 25 C (V)
HAL_PIN(rd);     // *parameter*, forward slope resistance at 25 C (ohm)
HAL_PIN(ed);     // *parameter*, Erec per amp recovered at e_volt, 25 C (J/A)
HAL_PIN(ed_tc);  // *parameter*, rise of that energy with junction temperature (1/K)
HAL_PIN(rth_d);  // *parameter*, junction to case (K/W)

// the shape of the junction to case impedance, as a three pole Foster network.
// the time constants are shared and the splits are separate pins, so a module
// whose diode curve differs from its transistor's can say so. the last share
// is whatever the first two leave over.
HAL_PIN(tau1);
HAL_PIN(tau2);
HAL_PIN(tau3);
HAL_PIN(fi1);
HAL_PIN(fi2);
HAL_PIN(fd1);
HAL_PIN(fd2);

// out
HAL_PIN(mi_cos);   // modulation index times power factor, negative regenerating
HAL_PIN(p_igbt);   // one transistor (W)
HAL_PIN(p_diode);  // one diode (W)
HAL_PIN(power);    // all twelve dies, what the heatsink sees (W)
HAL_PIN(t_igbt);   // (C)
HAL_PIN(t_diode);  // (C)
HAL_PIN(temp);     // the hotter of the two, what a derate should watch (C)

struct ipm_ctx_t {
  float ti[3];
  float td[3];
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ipm_pin_ctx_t *pins = (struct ipm_pin_ctx_t *)pin_ptr;

  PIN(f_sw)   = 15000.0;  // PWM_TIM_CLK / (2 * PWM_RES), centre aligned
  PIN(e_volt) = 300.0;
  PIN(v_tc)   = 0.00155;  // 1.55 V to 1.85 V between 25 C and 150 C

  PIN(vi)    = 0.667;  // 1.55 V at 20 A, split 0.43 knee to 0.57 slope
  PIN(ri)    = 0.0442;
  PIN(ei)    = 56.65e-6;  // (698 + 435) uJ at 300 V, 20 A, 25 C
  PIN(ei_tc) = 0.00316;   // 1133 uJ to 1580 uJ between 25 C and 150 C
  PIN(rth_i) = 2.37;      // 1.58 K/W times 1.5

  // the diode's own drop barely moves with temperature, 1.55 V to 1.6 V at
  // 20 A, but it shares v_tc. so its knee and slope are the datasheet's
  // curve fitted at 150 C and divided back down: right hot, a sixth low cold
  PIN(vd)    = 0.729;
  PIN(rd)    = 0.0306;
  PIN(ed)    = 4.75e-6;  // 95 uJ at 300 V, 20 A, 25 C
  PIN(ed_tc) = 0.00665;  // 95 uJ to 174 uJ between 25 C and 150 C
  PIN(rth_d) = 3.08;     // 2.05 K/W times 1.5

  PIN(tau1) = 0.00041;
  PIN(tau2) = 0.0143;
  PIN(tau3) = 0.229;
  PIN(fi1)  = 0.054;
  PIN(fi2)  = 0.170;
  PIN(fd1)  = 0.110;
  PIN(fd2)  = 0.247;

  PIN(t_igbt)  = 25.0;
  PIN(t_diode) = 25.0;
  PIN(temp)    = 25.0;
}

// one first order lag per pole, driven by the same power. the coefficient is
// clamped at one so a tau shorter than the tick cannot make the filter ring.
static float foster(float *state, float power, float rth, float f1, float f2, float t1, float t2, float t3, float period) {
  const float f3 = MAX(1.0 - f1 - f2, 0.0);

  state[0] += (power * rth * f1 - state[0]) * period / MAX(t1, period);
  state[1] += (power * rth * f2 - state[1]) * period / MAX(t2, period);
  state[2] += (power * rth * f3 - state[2]) * period / MAX(t3, period);

  return state[0] + state[1] + state[2];
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ipm_ctx_t *ctx      = (struct ipm_ctx_t *)ctx_ptr;
  struct ipm_pin_ctx_t *pins = (struct ipm_pin_ctx_t *)pin_ptr;

  // stmbl's transforms are amplitude invariant, so this is the peak of the
  // phase current and of the phase voltage, which is what the note wants
  const float i   = sqrtf(PIN(id) * PIN(id) + PIN(iq) * PIN(iq));
  const float udc = MAX(PIN(dc_volt), 1.0);

  // the modulation index is the peak phase voltage over half the link, and
  // the power factor is the voltage projected on the current, so the product
  // the loss equations want is just the real power scaled by the link
  float m = 0.0;
  if(i > 0.01) {
    m = 2.0 * (PIN(ud) * PIN(id) + PIN(uq) * PIN(iq)) / (udc * i);
  }
  m           = CLAMP(m, -1.2, 1.2);
  PIN(mi_cos) = m;

  // the previous cycle's junction estimate feeds the temperature terms. at
  // these time constants it is the same number, and it keeps the loop
  // explicit; the clamp stops a bad parameter set from running away.
  const float kvi = CLAMP(1.0 + PIN(v_tc) * (PIN(t_igbt) - 25.0), 0.5, 3.0);
  const float kvd = CLAMP(1.0 + PIN(v_tc) * (PIN(t_diode) - 25.0), 0.5, 3.0);
  const float kei = CLAMP(1.0 + PIN(ei_tc) * (PIN(t_igbt) - 25.0), 0.5, 3.0);
  const float ked = CLAMP(1.0 + PIN(ed_tc) * (PIN(t_diode) - 25.0), 0.5, 3.0);

  // conduction, the note's equations 9 and 10. the duty is (1 + MI cos)/2 for
  // the transistor and the rest for the diode, which is the whole of the sign
  // difference between these two lines.
  const float kv = M_1_PI / 2.0;
  const float kr = 1.0 / 8.0;
  const float dv = m / 8.0;
  const float dr = m * M_1_PI / 3.0;

  float p_i = kvi * (PIN(vi) * i * (kv + dv) + PIN(ri) * i * i * (kr + dr));
  float p_d = kvd * (PIN(vd) * i * (kv - dv) + PIN(rd) * i * i * (kr - dr));

  // switching, the note's equation 16 split over the two dies. the energies
  // are close to linear in the current interrupted and in the link voltage.
  const float k = PIN(f_sw) * i * M_1_PI * udc / MAX(PIN(e_volt), 1.0);

  p_i += PIN(ei) * k * kei;
  p_d += PIN(ed) * k * ked;

  PIN(p_igbt) = p_i = MAX(p_i, 0.0);
  PIN(p_diode) = p_d = MAX(p_d, 0.0);
  PIN(power)         = 6.0 * (p_i + p_d);

  const float rise_i = foster(ctx->ti, p_i, PIN(rth_i), PIN(fi1), PIN(fi2), PIN(tau1), PIN(tau2), PIN(tau3), period);
  const float rise_d = foster(ctx->td, p_d, PIN(rth_d), PIN(fd1), PIN(fd2), PIN(tau1), PIN(tau2), PIN(tau3), period);

  // deliberately no rt_start: a module does not cool because the drive was
  // disabled, so the state has to survive a stop
  PIN(t_igbt)  = PIN(case_temp) + rise_i;
  PIN(t_diode) = PIN(case_temp) + rise_d;
  PIN(temp)    = MAX(PIN(t_igbt), PIN(t_diode));
}

hal_comp_t ipm_comp_struct = {
    .name      = "ipm",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ipm_ctx_t),
    .pin_count = sizeof(struct ipm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
