#include "ocdac_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `ocdac` sets the F3 hardware overcurrent comparator threshold `hv0.dac` from a trip current in A peak, using a linear map fitted per board. It runs on the F4 board and is loaded by `conf/template/ocdac.txt` with `hv0.dac = ocdac0.dac` and `ocdac0.cur = 28`. Use `iddac` (`id_dac` template) to find the dac for one current on a given board.
*
* ## Component Explanation
*
* 1. **Dac map (nrt)**:
* ```c
* k       = MAX(k, 0.1);
* dac     = CLAMP(roundf(dac0 + k * MAX(cur, 0)), dac_min, dac_max);
* cur_out = (dac - dac0) / k;
* ```
* - `cur_out` shows when `dac_min` or `dac_max` limited the result.
*
* 2. **Calibration**:
* - The comparators see the raw shunt node (2.9 mV/A, about 3.6 counts/A in theory); the measured slope and offset depend on the TIM8 break filter, because the shorter filters trip on switching ringing on top of the current.
* - The defaults are that theory for the v4.0 board (3 mOhm shunt into the 15k/470 bias, AREF 3.338 V): `dac0` 124, `k` 3.57. On other boards built the same way the measured slope came out near twice that with filter 0xC (the default, periph.c), so the theory map trips well below `cur`: on the safe side, but it may stop the drive under load.
* - Fit `dac0` and `k` on the board from a few trips (`iddac` finds the dac for one current directly, take the highest trip of the 60/180/300 deg angles) and save them in the config.
*
* {{% hint warning %}}
* The map is a bench fit for one board and break filter setting; check it on your hardware before relying on it as overcurrent protection.
* {{% /hint %}}
*/
HAL_COMP(ocdac);

HAL_PIN(cur);      // *input*, wanted trip current (A peak), default 28
HAL_PIN(dac0);     // *parameter*, dac at 0 A from the fit, default 124 (theory)
HAL_PIN(k);        // *parameter*, dac counts per A, at least 0.1, default 3.57 (theory)
HAL_PIN(dac_min);  // *parameter*, floor: lowest dac with no ringing trip at 0 A, default 150
HAL_PIN(dac_max);  // *parameter*, ceiling, default 4095
HAL_PIN(dac);      // *output*, comparator dac value (0..4095), to hv0.dac
HAL_PIN(cur_out);  // *output*, trip current the clamped dac stands for (A peak)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ocdac_pin_ctx_t *pins = (struct ocdac_pin_ctx_t *)pin_ptr;
  PIN(cur)                     = 28.0;
  PIN(dac0)                    = 124.0;
  PIN(k)                       = 3.57;
  PIN(dac_min) = 150.0;
  PIN(dac_max) = 4095.0;
}

static void nrt_func(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ocdac_pin_ctx_t *pins = (struct ocdac_pin_ctx_t *)pin_ptr;

  float k      = MAX(PIN(k), 0.1);
  float dac    = CLAMP(roundf(PIN(dac0) + k * MAX(PIN(cur), 0.0)), PIN(dac_min), PIN(dac_max));
  PIN(dac)     = dac;
  PIN(cur_out) = (dac - PIN(dac0)) / k;
}

hal_comp_t ocdac_comp_struct = {
    .name      = "ocdac",
    .nrt       = nrt_func,
    .rt        = 0,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct ocdac_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
