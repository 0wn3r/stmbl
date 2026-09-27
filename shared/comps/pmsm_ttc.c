#include "pmsm_ttc_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `pmsm_ttc` (torque to current) converts a torque command (Nm) into a q-axis current command (peak A) for a permanent magnet synchronous motor, with optional cogging / torque ripple compensation and an experimental block commutation mode. It runs on the F4 board and is loaded by `conf/template/pmsm.txt`: `pmsm_ttc0.torque = pid0.torque_cmd`, `pmsm_ttc0.pos_in = vel2.pos_out`, `hv0.q_cmd = pmsm_ttc0.cur`, `hv0.pos = pmsm_ttc0.pos_out`, with `psi` and `polecount` from `conf0`.
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Torque to current**:
* ```c
* cur = (torque + g * (tc + te)) * 2 / 3 / polecount / psi;
* ```
* - This inverts `torque = 3/2 * polecount * psi * iq`. `polecount` is clamped to at least 1 and `psi` (flux linkage, Vs) to at least 0.01.
*
* 2. **Torque ripple model** (all amplitudes 0 by default, so nothing is added):
* - Cogging torque, independent of load: `tc = ac * sin(pc + pos_in * nc)`.
* - Electrical ripple, proportional to the torque command: `te = torque * ae * sin(pe + pos_in * ne)`.
* - `t = tc + te` is always output; it is only added to the current command when the gain `g` is non zero.
* - `nc` and `ne` are the number of ripple periods per turn of `pos_in` (default 1).
*
* 3. **Block commutation** (experimental):
* - `pos_out` is `pos_in` quantized to six 60 degree sectors, blended with the unmodified position by `block_gain`:
* ```c
* pos_out = pos_in * (1 - block_gain) + sector_pos(pos_in) * block_gain;
* ```
* - `block_gain` defaults to 0, so `pos_out = pos_in`.
*/

HAL_COMP(pmsm_ttc);

// motor values
HAL_PIN(psi);        // *parameter*, Flux linkage (Vs), min 0.01
HAL_PIN(polecount);  // *parameter*, Pole pairs, min 1

// cogging torque ripple, constant
HAL_PIN(ac);  // *parameter*, Cogging torque amplitude (Nm)
HAL_PIN(pc);  // *parameter*, Cogging torque phase (rad)
HAL_PIN(nc);  // *parameter*, Cogging periods per turn of pos_in (default 1)

// electrical torque ripple, linear with current
HAL_PIN(ae);  // *parameter*, Electrical ripple amplitude, relative to torque
HAL_PIN(pe);  // *parameter*, Electrical ripple phase (rad)
HAL_PIN(ne);  // *parameter*, Electrical ripple periods per turn of pos_in (default 1)

HAL_PIN(pos_in);      // *input*, Rotor position (rad)
HAL_PIN(pos_out);     // *output*, Commutation position, pos_in blended with block commutation (rad)
HAL_PIN(t);           // *output*, Modelled ripple torque tc + te (Nm)
HAL_PIN(g);           // *parameter*, Ripple compensation gain, 0 = off
HAL_PIN(block_gain);  // *parameter*, Block commutation blend 0..1 (default 0, experimental)


// torque cmd in
HAL_PIN(torque);  // *input*, Torque command (Nm)

// cur cmd out
HAL_PIN(cur);  // *output*, q-axis current command (A)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct sim_ctx_t * ctx = (struct sim_ctx_t *)ctx_ptr;
  struct pmsm_ttc_pin_ctx_t *pins = (struct pmsm_ttc_pin_ctx_t *)pin_ptr;

  PIN(nc)         = 1.0;
  PIN(ne)         = 1.0;
  PIN(block_gain) = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pmsm_ttc_ctx_t * ctx = (struct pmsm_ttc_ctx_t *)ctx_ptr;
  struct pmsm_ttc_pin_ctx_t *pins = (struct pmsm_ttc_pin_ctx_t *)pin_ptr;

  float p      = MAX(PIN(polecount), 1.0);
  float psi_m  = MAX(PIN(psi), 0.01);
  float torque = PIN(torque);
  float pos    = PIN(pos_in);

  float tc = PIN(ac) * sinf(PIN(pc) + pos * PIN(nc));
  float te = torque * PIN(ae) * sinf(PIN(pe) + pos * PIN(ne));

  PIN(pos_out) = pos * (1.0 - PIN(block_gain)) + (((int)((pos / 2.0 * M_1_PI + 0.5) * 6 + 0.5)) / 6.0 * 2.0 * M_PI - M_PI) * PIN(block_gain);
  PIN(t)       = tc + te;
  PIN(cur)     = (torque + PIN(g) * (tc + te)) / 3.0 * 2.0 / p / psi_m;
}

hal_comp_t pmsm_ttc_comp_struct = {
    .name      = "pmsm_ttc",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct pmsm_ttc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
