#include "acim_flux_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* The `acim_flux` component models the rotor flux of an induction motor in the
* rotor flux frame and gives the slip that keeps that frame oriented.
*
* ## Model
* With i_mr the magnetizing current behind the rotor flux (psi_r = Lm * i_mr):
*
*     tr * d(i_mr)/dt = id - i_mr
*     slip            = iq / (tr * i_mr)             [rad/s electrical]
*     torque          = 3/2 * pp * lmr * i_mr * iq    lmr = Lm^2 / Lr
*
* `tr` (Lr/Rr) and `lmr` come from idacim. `id`/`iq` are normally the measured
* currents, so a current limit in curpid scales both and the slip stays right.
*
* `psi` = lmr * i_mr is the rotor flux seen from the stator. Sent as hv0.psi,
* with hv0.l = hv0.lq = sigma*Ls (leakage), it turns curpid's PMSM decoupling
* psi_d = l * id + psi into the induction motor's stator flux, so the F3 needs
* no ACIM code.
*
* `en` = 0 clears the flux, so every enable starts from a demagnetized rotor.
*
* ## tr adaptation (reactive power MRAS), off by default
* tr changes with rotor temperature. With `tr_ki` > 0 the model runs on
* `tr_est`, which integrates the reactive power residual
*
*     Q   = uq id - ud iq
*     Q_m = w (id (l id + psi) + l iq^2) - r_w |w| id iq
*     tr_est -= tr_ki * (Q - Q_m) * sign(w) * period
*
* Q does not depend on r, so neither does the estimate. A positive residual
* (times sign(w)) means tr_est is too high; the plant gain is a few thousand VA per second of
* tr, so tr_ki 0.0001 settles in a few seconds. It adapts only while
* |iq| > `tr_iq_min` and |w| > `tr_vel_min` (slip must be large enough to
* see), holds otherwise, and stays within 0.5 to 2 times `tr`. With
* tr_ki 0, tr_est = tr.
*/

HAL_COMP(acim_flux);

HAL_PIN(tr);         // *parameter*, rotor time constant Lr/Rr [s]
HAL_PIN(lmr);        // *parameter*, rotor side magnetizing inductance Lm^2/Lr [H]
HAL_PIN(polecount);  // *parameter*, pole pairs
HAL_PIN(i_min);      // *parameter*, i_mr floor for the slip division [A]
HAL_PIN(slip_max);   // *parameter*, slip clamp [rad/s electrical]

HAL_PIN(en);         // *input*, 0 = rotor demagnetized
HAL_PIN(id);         // *input*, d current [A]
HAL_PIN(iq);         // *input*, q current [A]

HAL_PIN(i_mr);       // *output*, magnetizing current [A]
HAL_PIN(psi);        // *output*, lmr * i_mr [V s], to hv0.psi
HAL_PIN(slip);       // *output*, slip [rad/s electrical]
HAL_PIN(torque);     // *output*, estimated torque [Nm]

HAL_PIN(tr_ki);      // *parameter*, tr adaptation gain [1/(VA s)], 0 = off
HAL_PIN(tr_iq_min);  // *parameter*, adapt only above this |iq| [A]
HAL_PIN(tr_vel_min); // *parameter*, adapt only above this |vel| [rad/s electrical]
HAL_PIN(l);          // *parameter*, leakage sigma*Ls [H], conf0.l
HAL_PIN(r_w);        // *parameter*, speed dependent d loss [ohm per rad/s electrical], as obs0.r_w
HAL_PIN(ud);         // *input*, hv0.ud_fb
HAL_PIN(uq);         // *input*, hv0.uq_fb
HAL_PIN(vel);        // *input*, synchronous speed [rad/s electrical], angle0.vel
HAL_PIN(tr_est);     // *output*, the tr the model runs on
HAL_PIN(q_res);      // *output*, reactive power residual Q - Q_m [VA]

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_flux_pin_ctx_t *pins = (struct acim_flux_pin_ctx_t *)pin_ptr;

  PIN(tr)        = 0.06;
  PIN(lmr)       = 0.0165;
  PIN(polecount) = 2.0;
  PIN(i_min)     = 0.5;
  PIN(slip_max)  = 100.0;
  PIN(tr_iq_min) = 3.0;
  PIN(tr_vel_min) = 100.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_flux_pin_ctx_t *pins = (struct acim_flux_pin_ctx_t *)pin_ptr;

  float tr_n = MAX(PIN(tr), 0.001);
  float lmr  = MAX(PIN(lmr), 0.0);
  float i_mr = PIN(i_mr);

  float tr = tr_n;
  if(PIN(tr_ki) > 0.0) {
    tr = PIN(tr_est) > 0.0 ? PIN(tr_est) : tr_n;

    float id  = PIN(id);
    float iq  = PIN(iq);
    float w   = PIN(vel);
    float l   = MAX(PIN(l), 0.0);
    float q   = PIN(uq) * id - PIN(ud) * iq;
    float q_m = w * (id * (l * id + lmr * i_mr) + l * iq * iq) - PIN(r_w) * ABS(w) * id * iq;
    PIN(q_res) = q - q_m;
    if(PIN(en) > 0.0 && ABS(iq) > PIN(tr_iq_min) && ABS(w) > PIN(tr_vel_min)) {
      // Q and Q_m both follow the direction of rotation; the detuning term
      // goes with iq^2, so only the sign of w matters
      tr -= PIN(tr_ki) * PIN(q_res) * (w > 0.0 ? 1.0 : -1.0) * period;
    }
    tr = CLAMP(tr, 0.5 * tr_n, 2.0 * tr_n);
  } else {
    PIN(q_res) = 0.0;
  }
  PIN(tr_est) = tr;

  if(PIN(en) > 0.0) {
    // backward Euler step of the first order lag, stable for any period
    i_mr += (PIN(id) - i_mr) * period / (tr + period);
  } else {
    i_mr = 0.0;
  }

  // the sign of i_mr follows id; divide by its magnitude with a floor, so a
  // demagnetized rotor asks for a bounded slip instead of an infinite one
  float i_div = MAX(ABS(i_mr), MAX(PIN(i_min), 0.01));
  float slip  = PIN(iq) / (tr * i_div);
  if(i_mr < 0.0) {
    slip = -slip;
  }
  if(PIN(slip_max) > 0.0) {
    slip = LIMIT(slip, PIN(slip_max));
  }

  PIN(i_mr)   = i_mr;
  PIN(psi)    = lmr * i_mr;
  PIN(slip)   = slip;
  PIN(torque) = 1.5 * MAX(PIN(polecount), 1.0) * lmr * i_mr * PIN(iq);
}

hal_comp_t acim_flux_comp_struct = {
    .name      = "acim_flux",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct acim_flux_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
