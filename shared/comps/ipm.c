#include "ipm_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `ipm` estimates the junction temperatures of the power module (IPM) on the F4 board, so the drive can derate and trip on die temperature between the fast overcurrent trips and the slow module NTC. It adds a modelled temperature rise of each of the twelve dies (per leg: high and low IGBT, high and low diode) to the NTC reading. Loaded by conf/template/ipm.txt, which the motor templates (pid, mpid, uf and vf) link at their end, so a config does not link it again (a second `link ipm` loads an `ipm1` nothing reads): its inputs come from `hv0` (`ipm0.id = hv0.id_fb`, ..., `ipm0.case_temp = hv0.hv_temp`, `ipm0.pos = hv0.pos`) and `fault0.ipm_temp = ipm0.temp` feeds the derate (`conf0.high_ipm_temp`) and the IPM_TEMP_ERROR trip (`conf0.max_ipm_temp`) in `fault`. The defaults are for the IRAM256-2067A fitted to the v4.0 board.
*
* ## Component Explanation
* Everything runs in `rt`. There is no `rt_start`, so the die temperatures survive a disable (the module does not cool because the drive stopped).
*
* 1. **Leg currents and duties**:
* - The angle is read from the pin that finally drives `pos`: `ipm0.pos = hv0.pos` links an input to an input, so the component follows the link chain (up to 4 hops) to the driving pin, e.g. `vel2.pos_out`.
* - `id`, `iq`, `ud`, `uq` are turned into the three phase currents and voltages with an amplitude invariant inverse Park and Clarke transform at that angle.
* - Leg duty = `CLAMP(0.5 + u_phase / dc_volt, 0, 1)`; the common mode of the modulation is ignored. `dc_volt` is taken as at least 1 V.
*
* 2. **Losses per die** (the equations of Infineon's CIPOS Mini application note, AN2016-10, which hold for any three phase IGBT module):
* - Current out of the leg flows through the high IGBT for the duty and the low diode for the rest; current into the leg through the low IGBT for the rest and the high diode for the duty. Dies not carrying the current get no loss.
* - Conduction: `duty_share * kvt * (v0 * |i| + r0 * i^2)` with `vi`/`ri` for the IGBT and `vd`/`rd` for the diode, `kvt = 1 + v_tc * (Tj - 25)`.
* - Switching: `e * f_sw * dc_volt / e_volt * |i| * ket` with `ei` (IGBT Eon + Eoff per amp) or `ed` (diode Erec per amp), `ket = 1 + ei_tc (or ed_tc) * (Tj - 25)`. Both temperature factors are clamped to 0.5..3 and use each die's own last junction temperature.
*
* 3. **Thermal model**:
* - Each die has a three pole Foster network, one first order lag per pole with the shared time constants `tau1..tau3` and the splits `fi1`, `fi2` (IGBT) or `fd1`, `fd2` (diode). The static rise is `P * rth_i` or `P * rth_d`. Time constants shorter than the rt period are clamped to the period.
* ```c
* Tj = case_temp + sum over k of lag(P * rth * f_k, tau_k)
* ```
* - At speed the networks average over the electrical cycle; at standstill the leg carrying DC heats on its own, which a mean value model over a sine would understate about 3.3x.
*
* {{% hint warning %}}
* The IRAM256-2067A defaults come from its datasheet (May 2014) and are not yet checked on the bench: `rth_i` and `rth_d` are the maximum Rth(J-C) (3.6 and 5.1 K/W), the conduction drops are split from the one point the table gives (7.5 A, 25 C), `ed` is an estimate (the datasheet folds the diode's recovery into the IGBT's Eon), and `v_tc`, `ei_tc` and `ed_tc` are fitted so the total loss matches the datasheet's loss curves at Tj 150 C (Fig. 6 and 7, within 4 %). The Foster shape is borrowed, as the datasheet has no Zth curve. At standstill one IGBT carries the full phase current: about 18 A DC from 40 C reaches the 125 C derate in a fraction of a second. Check `temp` against the datasheet's Fig. 9 (Tj = 1.26 * T_therm + 27 degC at full load) and trim on the hardware. They do not include the heatsink, which is covered by `case_temp` from the NTC. `pos` is used as linked, without `hv0.adv`.
* {{% /hint %}}
*/

HAL_COMP(ipm);

/*
Bridge junction temperature estimate, for a derate and trip between the
instantaneous overcurrent trip and the NTC derate. The module NTC lags the
dies by seconds, so it carries the slow part (case_temp) and this models the
rise above it.

Each of the twelve dies (per leg: high and low IGBT, high and low diode) runs
its own three pole Foster network on its instantaneous loss: conduction from
the leg current and duty, switching from the current and link voltage, after
Infineon's CIPOS Mini application note. At speed the networks average the cycle; at
standstill the one leg carrying dc heats on its own, which a mean value model
over a sine understates about 3.3x.

Defaults are first datasheet values for the IRAM256-2067A on the v4.0
board; trim them against the real heatsink.
*/

// in
HAL_PIN(id);         // *input*, d axis current (A peak), hv0.id_fb
HAL_PIN(iq);         // *input*, q axis current (A peak), hv0.iq_fb
HAL_PIN(ud);         // *input*, d axis voltage (V), hv0.ud_fb
HAL_PIN(uq);         // *input*, q axis voltage (V), hv0.uq_fb
HAL_PIN(pos);        // *input*, electrical angle hv0 commutates with (rad), link chain is followed
HAL_PIN(dc_volt);    // *input*, DC link voltage (V), hv0.dc_volt
HAL_PIN(case_temp);  // *input*, module NTC temperature (deg C), hv0.hv_temp

// device, common
HAL_PIN(f_sw);    // *parameter*, switching frequency (Hz), default 15000, the module templates link hv0.pwm_freq
HAL_PIN(e_volt);  // *parameter*, link voltage the switching energies were measured at (V), default 300
HAL_PIN(v_tc);    // *parameter*, rise of the conduction drops with junction temperature (1/K)

// transistor
HAL_PIN(vi);     // *parameter*, IGBT on state knee voltage at 25 C (V)
HAL_PIN(ri);     // *parameter*, IGBT on state slope resistance at 25 C (Ohm)
HAL_PIN(ei);     // *parameter*, IGBT (Eon + Eoff) per amp switched at e_volt, 25 C (J/A)
HAL_PIN(ei_tc);  // *parameter*, rise of that energy with junction temperature (1/K)
HAL_PIN(rth_i);  // *parameter*, IGBT junction to case thermal resistance (K/W)

// diode
HAL_PIN(vd);     // *parameter*, diode forward knee voltage at 25 C (V)
HAL_PIN(rd);     // *parameter*, diode forward slope resistance at 25 C (Ohm)
HAL_PIN(ed);     // *parameter*, diode Erec per amp recovered at e_volt, 25 C (J/A)
HAL_PIN(ed_tc);  // *parameter*, rise of that energy with junction temperature (1/K)
HAL_PIN(rth_d);  // *parameter*, diode junction to case thermal resistance (K/W)

// junction to case impedance shape, three pole Foster network: shared time
// constants, separate splits; the last share is what the first two leave
HAL_PIN(tau1);   // *parameter*, Foster pole 1 time constant (s), shared by IGBT and diode
HAL_PIN(tau2);   // *parameter*, Foster pole 2 time constant (s)
HAL_PIN(tau3);   // *parameter*, Foster pole 3 time constant (s)
HAL_PIN(fi1);    // *parameter*, IGBT share of rth_i in pole 1
HAL_PIN(fi2);    // *parameter*, IGBT share of rth_i in pole 2, pole 3 gets the rest
HAL_PIN(fd1);    // *parameter*, diode share of rth_d in pole 1
HAL_PIN(fd2);    // *parameter*, diode share of rth_d in pole 2, pole 3 gets the rest

// out
HAL_PIN(p_igbt);   // *output*, mean loss per IGBT (W)
HAL_PIN(p_diode);  // *output*, mean loss per diode (W)
HAL_PIN(power);    // *output*, total loss of all twelve dies, what the heatsink sees (W)
HAL_PIN(t_igbt);   // *output*, hottest IGBT junction (deg C)
HAL_PIN(t_diode);  // *output*, hottest diode junction (deg C)
HAL_PIN(temp);     // *output*, the hotter of t_igbt and t_diode (deg C), to fault0.ipm_temp

struct ipm_ctx_t {
  float die[12][3];  // per leg: high igbt, low igbt, high diode, low diode
  float tdie[12];    // their junctions, for the temperature coefficients
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ipm_pin_ctx_t *pins = (struct ipm_pin_ctx_t *)pin_ptr;

  PIN(f_sw)   = 15000.0;  // the f3 PWM rate, the templates link hv0.pwm_freq
  PIN(e_volt) = 300.0;
  // the tcs are fitted to the datasheet's loss curves at Tj 150 C (Fig. 6 and 7,
  // 400 V, PF 0.6, 2-20 kHz, 1-12 A rms), which sit 27 % (conduction) and 67 %
  // (switching) above the table values; the 25 C values are the table's
  PIN(v_tc)   = 0.00349;

  PIN(vi)    = 0.645;  // 1.5 V at 7.5 A, split 0.43 knee to 0.57 slope
  PIN(ri)    = 0.114;
  PIN(ei)    = 39.5e-6;  // 395 uJ at 400 V, 7.5 A, 25 C, scaled to 300 V
  PIN(ei_tc) = 0.0113;
  PIN(rth_i) = 3.6;      // maximum Rth(J-C) per IGBT

  // 1.24 V at 7.5 A and 25 C, so 1.77 V at 150 C with the shared v_tc
  PIN(vd)    = 0.531;
  PIN(rd)    = 0.0939;
  PIN(ed)    = 3.3e-6;  // not in the datasheet: an Erec share of ei
  PIN(ed_tc) = 0.016;  // the 3x clamp at 150 C
  PIN(rth_d) = 5.1;  // maximum Rth(J-C) per diode

  // no Zth curve in the datasheet: a module Foster shape to start from
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

// one first order lag per pole, clamped so a tau under the tick cannot ring
static float foster(float *state, float power, float rth, float f1, float f2, float t1, float t2, float t3, float period) {
  const float f3 = MAX(1.0 - f1 - f2, 0.0);

  state[0] += (power * rth * f1 - state[0]) * period / MAX(t1, period);
  state[1] += (power * rth * f2 - state[1]) * period / MAX(t2, period);
  state[2] += (power * rth * f3 - state[2]) * period / MAX(t3, period);

  return state[0] + state[1] + state[2];
}

// deliberately no rt_start: a module does not cool because the drive was
// disabled, so the state survives a stop
static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ipm_ctx_t *ctx      = (struct ipm_ctx_t *)ctx_ptr;
  struct ipm_pin_ctx_t *pins = (struct ipm_pin_ctx_t *)pin_ptr;

  const float udc = MAX(PIN(dc_volt), 1.0);

  // ipm0.pos = hv0.pos links input to input and PIN() reads one level, so
  // walk the chain to the pin that is driven
  volatile hal_pin_inst_t *src = pins->pos.source;
  for(int hop = 0; hop < 4 && src->source != src; hop++) {
    src = src->source;
  }
  const float th = src->value;

  // phase currents and voltages, amplitude invariant inverse park and clarke
  float c, s;
  sincos_fast(th, &s, &c);
  float ia = PIN(id) * c - PIN(iq) * s, ib = PIN(id) * s + PIN(iq) * c;
  float ua = PIN(ud) * c - PIN(uq) * s, ub = PIN(ud) * s + PIN(uq) * c;
  float il[3] = {ia, -0.5 * ia + 0.5 * M_SQRT3 * ib, -0.5 * ia - 0.5 * M_SQRT3 * ib};
  float ul[3] = {ua, -0.5 * ua + 0.5 * M_SQRT3 * ub, -0.5 * ua - 0.5 * M_SQRT3 * ub};

  const float ksw = PIN(f_sw) * udc / MAX(PIN(e_volt), 1.0);
  float sum_i = 0.0, sum_d = 0.0, t_i = -300.0, t_d = -300.0;
  for(int l = 0; l < 3; l++) {
    float a    = ABS(il[l]);
    float duty = CLAMP(0.5 + ul[l] / udc, 0.0, 1.0);  // leg duty, common mode ignored
    for(int k = 0; k < 4; k++) {  // high igbt, low igbt, high diode, low diode
      float tj  = ctx->tdie[l * 4 + k];
      int diode = k >= 2;
      float kvt = CLAMP(1.0 + PIN(v_tc) * (tj - 25.0), 0.5, 3.0);
      float ket = CLAMP(1.0 + (diode ? PIN(ed_tc) : PIN(ei_tc)) * (tj - 25.0), 0.5, 3.0);
      // current out of the leg: high igbt for the duty, low diode for the
      // rest; into the leg: low igbt for the rest, high diode for the duty
      int on_hi  = (k == 0 || k == 2);
      int active = il[l] >= 0.0 ? (k == 0 || k == 3) : (k == 1 || k == 2);
      float d    = on_hi ? duty : 1.0 - duty;
      float cond = diode ? kvt * (PIN(vd) * a + PIN(rd) * a * a) : kvt * (PIN(vi) * a + PIN(ri) * a * a);
      float sw   = (diode ? PIN(ed) : PIN(ei)) * ksw * a * ket;
      float pw   = active ? d * cond + sw : 0.0;
      float rise = foster(ctx->die[l * 4 + k], pw, diode ? PIN(rth_d) : PIN(rth_i), diode ? PIN(fd1) : PIN(fi1), diode ? PIN(fd2) : PIN(fi2), PIN(tau1), PIN(tau2), PIN(tau3), period);
      ctx->tdie[l * 4 + k] = PIN(case_temp) + rise;
      if(diode) {
        sum_d += pw;
        t_d = MAX(t_d, ctx->tdie[l * 4 + k]);
      } else {
        sum_i += pw;
        t_i = MAX(t_i, ctx->tdie[l * 4 + k]);
      }
    }
  }
  PIN(p_igbt)  = sum_i / 6.0;
  PIN(p_diode) = sum_d / 6.0;
  PIN(power)   = sum_i + sum_d;
  PIN(t_igbt)  = t_i;
  PIN(t_diode) = t_d;
  PIN(temp)    = MAX(t_i, t_d);
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
