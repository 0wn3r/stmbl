#include "vf_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* The `vf` component runs an induction motor open loop, V/f, and replaces uf
* (uf2.c) for new configs. Its frequency goes to angle0 (src 3) and its
* voltage to hv0.d_cmd in volt mode.
*
* ## Behaviour
* - Frequency `vel` ramps toward `vel_cmd` (mechanical rad/s) at `acc` and
*   `dec`; vf is its own ramp, so no ramp0 is needed in front. Accel stall prevention scales the ramp with the active current (low
*   passed at damp_hz): full `acc` up to `i_stall`, down to 0 at 1.1 *
*   i_stall, then back toward zero, reaching `stall_dec` at 1.3 * i_stall.
*   Proportional, so the frequency never steps. The magnetizing current doesn't count, so
*   i_stall is a load limit and works near base speed too. It stops
*   falling while the dc link is above `dc_hold` (decel stall prevention, the
*   motor regenerating into the link). `scale` (fault0.scale) lowers i_stall,
*   so a derate slows the motor instead of cutting its flux.
* - Voltage: u_n * |vel| / vel_n plus a boost u_boost that fades linearly to 0
*   at boost_vel. Phase peak volts, like every volt mode d_cmd.
* - Slip compensation from the ACTIVE current, signed: the current along the
*   voltage vector, which is torque producing and goes negative when the motor
*   brakes, minus nothing for magnetizing. slip_n is the rated slip in
*   mechanical rad/s at active current cur_n.
* - `vel_e` = (vel + slip) * polecount, synchronous electrical speed for
*   angle0.vel_cmd, so hv0.vel is right and the f3 extrapolates between packets.
* - Damping (optional): open loop V/f on a lightly loaded motor hunts, the
*   rotor swinging against the field. `damp` = k_damp * the active current
*   band passed between damp_hz and damp_lp_hz moves the field frequency with
*   the swing and damps it. The high pass keeps steady slip and the ramp's
*   acceleration current out, the low pass the pwm and current loop ripple.
*   Frequency only, the voltage stays on vel + slip. Clamped to 5 % of vel_n.
*   k_damp 0 = off, positive damps.
*/

HAL_COMP(vf);

HAL_PIN(en);         // *input*, 0 = frequency back to 0 at once
HAL_PIN(vel_cmd);    // *input*, speed command [rad/s mech]
HAL_PIN(polecount);  // *parameter*, pole pairs
HAL_PIN(u_n);        // *parameter*, phase peak volts at vel_n
HAL_PIN(vel_n);      // *parameter*, base speed [rad/s mech]
HAL_PIN(u_boost);    // *parameter*, low speed boost at standstill [V]
HAL_PIN(boost_vel);  // *parameter*, speed where the boost has faded out [rad/s mech]
HAL_PIN(slip_n);     // *parameter*, rated slip [rad/s mech] at active current cur_n, 0 = off
HAL_PIN(cur_n);      // *parameter*, rated active current [A peak]
HAL_PIN(acc);        // *parameter*, acceleration [rad/s^2 mech]
HAL_PIN(dec);        // *parameter*, deceleration [rad/s^2 mech]
HAL_PIN(i_stall);    // *parameter*, accel stall active current [A peak], 0 = off
HAL_PIN(stall_dec);  // *parameter*, deceleration while stalled [rad/s^2 mech]
HAL_PIN(dc_hold);    // *parameter*, dc link volts that hold deceleration, 0 = off
HAL_PIN(scale);      // *input*, derate, 1 = none
HAL_PIN(k_damp);     // *parameter*, damping gain [rad/s mech per A], 0 = off
HAL_PIN(damp_hz);    // *parameter*, damping high pass corner [Hz]
HAL_PIN(damp_lp_hz); // *parameter*, damping low pass corner [Hz]

HAL_PIN(id);         // *input*, hv0.id_fb
HAL_PIN(iq);         // *input*, hv0.iq_fb
HAL_PIN(ud);         // *input*, hv0.ud_fb
HAL_PIN(uq);         // *input*, hv0.uq_fb
HAL_PIN(dc_volt);    // *input*, hv0.dc_volt

HAL_PIN(vel);        // *output*, stator frequency, less slip [rad/s mech]
HAL_PIN(vel_e);      // *output*, synchronous electrical speed [rad/s], to angle0.vel_cmd
HAL_PIN(u_cmd);      // *output*, phase peak volts, to hv0.d_cmd
HAL_PIN(i_act);      // *output*, active current [A], negative when braking
HAL_PIN(slip);       // *output*, slip compensation [rad/s mech]
HAL_PIN(stall);      // *output*, 1 accel held, 2 accel reversed, -1 decel held
HAL_PIN(damp);       // *output*, damping frequency offset [rad/s mech]

struct vf_ctx_t {
  float i_lp;  // active current low pass at damp_hz, damping high pass
  float d_lp;  // high passed current, low passed at damp_lp_hz
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vf_pin_ctx_t *pins = (struct vf_pin_ctx_t *)pin_ptr;

  PIN(polecount) = 2.0;
  PIN(u_n)       = 90.0;
  PIN(vel_n)     = 246.3;
  PIN(u_boost)   = 0.0;
  PIN(boost_vel) = 25.0;
  PIN(slip_n)    = 0.0;
  PIN(cur_n)     = 10.0;
  PIN(acc)       = 50.0;
  PIN(dec)       = 50.0;
  PIN(i_stall)   = 0.0;
  PIN(stall_dec) = 50.0;
  PIN(dc_hold)   = 0.0;
  PIN(scale)     = 1.0;
  PIN(k_damp)    = 0.0;
  PIN(damp_hz)    = 3.0;
  PIN(damp_lp_hz) = 20.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vf_ctx_t *ctx      = (struct vf_ctx_t *)ctx_ptr;
  struct vf_pin_ctx_t *pins = (struct vf_pin_ctx_t *)pin_ptr;

  float vel = PIN(vel);
  float cmd = PIN(vel_cmd);
  float id  = PIN(id);
  float iq  = PIN(iq);
  float ud  = PIN(ud);
  float uq  = PIN(uq);

  float u     = sqrtf(ud * ud + uq * uq);
  float i_act = u > 0.1 ? (ud * id + uq * iq) / u : 0.0;

  // active current low pass: stall prevention, and the damping high pass
  if(PIN(en) > 0.0) {
    ctx->i_lp += (i_act - ctx->i_lp) * CLAMP(2.0 * M_PI * PIN(damp_hz) * period, 0.0, 1.0);
  } else {
    ctx->i_lp = 0.0;
  }

  float stall = 0.0;
  float i_st  = PIN(i_stall) * CLAMP(PIN(scale), 0.0, 1.0);
  int accel   = ABS(cmd) > ABS(vel) && cmd * vel >= 0.0;
  int decel   = !accel && cmd != vel;

  if(PIN(en) <= 0.0) {
    vel = 0.0;
  } else if(accel) {
    // over: active current above i_stall, as a fraction of i_stall
    float over = PIN(i_stall) > 0.0 ? (ctx->i_lp - i_st) / MAX(i_st, 0.1) : -1.0;
    if(over > 0.1) {  // back off toward zero
      stall = 2.0;
      float dec = MAX(PIN(stall_dec), 0.0) * MIN((over - 0.1) / 0.2, 1.0);
      vel -= SIGN(vel) * MIN(dec * period, ABS(vel));
    } else {
      float acc = MAX(PIN(acc), 0.0);
      if(over > 0.0) {  // slow down toward a hold
        stall = 1.0;
        acc *= 1.0 - over / 0.1;
      }
      vel += LIMIT(cmd - vel, acc * period);
    }
  } else if(decel) {
    if(PIN(dc_hold) > 0.0 && PIN(dc_volt) > PIN(dc_hold)) {
      stall = -1.0;
    } else {
      vel += LIMIT(cmd - vel, MAX(PIN(dec), 0.0) * period);
    }
  }

  float slip = 0.0;
  if(PIN(slip_n) > 0.0 && PIN(cur_n) > 0.0) {
    // i_act follows the power: positive motoring in either direction, so the
    // slip adds in the direction of rotation when motoring and takes away
    // when braking
    slip = LIMIT(PIN(slip_n) * i_act / PIN(cur_n), 2.0 * PIN(slip_n)) * SIGN(vel);
  }

  float boost = 0.0;
  if(PIN(boost_vel) > 0.0) {
    boost = PIN(u_boost) * MAX(1.0 - ABS(vel) / PIN(boost_vel), 0.0);
  }
  float u_cmd = PIN(u_n) * ABS(vel + slip) / MAX(PIN(vel_n), 0.1) + boost;

  float damp = 0.0;
  if(PIN(en) > 0.0) {
    ctx->d_lp += (i_act - ctx->i_lp - ctx->d_lp) * CLAMP(2.0 * M_PI * PIN(damp_lp_hz) * period, 0.0, 1.0);
    damp = LIMIT(PIN(k_damp) * ctx->d_lp, 0.05 * MAX(PIN(vel_n), 0.1));
  } else {
    ctx->d_lp = 0.0;
  }

  PIN(vel)   = vel;
  PIN(slip)  = slip;
  PIN(i_act) = i_act;
  PIN(stall) = stall;
  PIN(damp)  = damp;
  PIN(vel_e) = (vel + slip + damp) * MAX(PIN(polecount), 1.0);
  PIN(u_cmd) = u_cmd;
}

hal_comp_t vf_comp_struct = {
    .name      = "vf",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct vf_ctx_t),
    .pin_count = sizeof(struct vf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
