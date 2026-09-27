#include "idq_comp.h"
#include "commands.h"
#include "common.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `idq` is the inverse of `dq`: it turns d/q values (normally the voltages from the current controller) at the rotor angle `pos` into the alpha/beta frame (inverse Park) and then into three phase values (inverse Clarke). It runs on the F3 (HV board) as `idq0`, loaded by `stm32f303/src/main.c` (rt_prio 4): `idq0.d/q = curpid0.ud/uq`, `idq0.pos = ls0.pos`, `idq0.mode = ls0.phase_mode`, and `u`/`v`/`w` go to `svm0`.
*
* ## Component Explanation
*
* 1. **Inverse Park transform** (rt):
* - The electrical angle is `pos * polecount` (`polecount` is truncated to an integer, min 1).
* ```c
* a = d * cos - q * sin;
* b = d * sin + q * cos;
* ```
*
* 2. **Inverse Clarke transform** (rt), selected by `mode`:
* - 0 (90 deg 3 phase): `u = a`, `v = 0`, `w = b`.
* - 2 (120 deg 3 phase): `u = a`, `v = -a/2 + b * sqrt(3)/2`, `w = -a/2 - b * sqrt(3)/2`. The phase amplitude equals the vector length.
* - 3 (180 deg 2 phase): `u = b/2`, `v = 0`, `w = -b/2`.
* - 4 (180 deg 3 phase): `u = b/2`, `v = a`, `w = -b/2`.
* - Any other mode (including 1, 90 deg 4 phase) gives 0.
* - The outputs are centred on 0. `svm` adds the offset that puts them into 0..udc.
*/

HAL_COMP(idq);

HAL_PIN(mode);  // *input*, Phase mode, 0 = 90 deg 3ph, 2 = 120 deg 3ph, 3 = 180 deg 2ph, 4 = 180 deg 3ph, others give 0

//d,q inputs
HAL_PIN(d);  // *input*, D-axis value, e.g. voltage (V)
HAL_PIN(q);  // *input*, Q-axis value, e.g. voltage (V)

//rotor position
HAL_PIN(pos);        // *input*, Rotor angle (rad)
HAL_PIN(polecount);  // *parameter*, Pole pairs, pos is multiplied by it, min 1, default 0 (used as 1)

// sin/cos of the same angle from elsewhere (dq0 on the f3, which transforms
// the currents with it a moment earlier), used instead of a second
// sincos_fast when ext_sc > 0
HAL_PIN(si);
HAL_PIN(co);
HAL_PIN(ext_sc);
// the sin/cos this tick used, for hv0's dead-time reference currents
HAL_PIN(si_out);
HAL_PIN(co_out);

//a,b output
HAL_PIN(a);  // *output*, Alpha component
HAL_PIN(b);  // *output*, Beta component

//U V W output
HAL_PIN(u);  // *output*, U phase value
HAL_PIN(v);  // *output*, V phase value
HAL_PIN(w);  // *output*, W phase value

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct idq_ctx_t * ctx = (struct idq_ctx_t *)ctx_ptr;
  struct idq_pin_ctx_t *pins = (struct idq_pin_ctx_t *)pin_ptr;

  float d = PIN(d);
  float q = PIN(q);

  float p   = (int)MAX(PIN(polecount), 1.0);
  float pos = PIN(pos) * p;

  float si = PIN(si);
  float co = PIN(co);
  if(PIN(ext_sc) <= 0.0) {
    sincos_fast(pos, &si, &co);
  }
  PIN(si_out) = si;
  PIN(co_out) = co;

  //inverse park transformation
  float a = d * co - q * si;
  float b = d * si + q * co;

  //inverse clarke transformation
  float u, v, w;

  switch((int)PIN(mode)) {
    case PHASE_90_3PH:  // 90°
      u = a;
      v = 0.0;
      w = b;
      break;

    case PHASE_120_3PH:  // 120°
      u = a;
      v = -a / 2.0 + b / 2.0 * M_SQRT3;
      w = -a / 2.0 - b / 2.0 * M_SQRT3;
      break;

    case PHASE_180_2PH:  // 180°
      u = b / 2.0;
      v = 0.0;
      w = -b / 2.0;
      break;

    case PHASE_180_3PH:  // 180°
      u = b / 2.0;
      v = a;
      w = -b / 2.0;
      break;

    default:
      u = 0.0;
      v = 0.0;
      w = 0.0;
  }


  PIN(a) = a;
  PIN(b) = b;

  PIN(u) = u;
  PIN(v) = v;
  PIN(w) = w;
}

hal_comp_t idq_comp_struct = {
    .name      = "idq",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct idq_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
