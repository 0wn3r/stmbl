#include "ids_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

/**
 * id_pid (comp ids): tunes pos_bw, vel_bw and vel_d one at a time on a
 * trapezoid profile between min_pos and max_pos. Each step raises the
 * parameter by step while each raise cuts the cycle cost (tracking error)
 * by at least kg. A raise that gains less, or goes over the noise limit, is
 * taken back and the search moves to the next parameter, so it stops where
 * more gain stops paying. Noisy already at its start value, a parameter is
 * cut by kd instead.
 *
 * The first cycle after enable is a warm-up and is not scored: it holds the
 * enable, an ACIM's field build and the integrators settling, and runs on
 * while pid saturates (pid0.sat, or the torque at conf0.max_force), up to
 * WARM_MAX cycles. A saturated cycle after that is not scored either: it
 * is repeated with the same gains. SAT_MAX saturated cycles in a row
 * count as a failed step; at a parameter's start value they cut it by kd
 * and retry, up to CUT_MAX times. The start values are conf0.pos_bw, vel_bw
 * and vel_d (10, 100 and 10 when they are 0). The profile
 * starts at pos_fb, so there is no jump. Each score averages rep cycles.
 *
 * With good feedforward the tracking cost goes to almost nothing and stops
 * limiting the gains, so a step also fails when the feedback torque's noise
 * (fb_torque above NOISE_HZ, rms) goes over fb_max x max_torque. max_torque
 * is conf0.max_force; 0 turns the limit off. The torque peak is only
 * reported: it is set by the profile (J x max_acc at the reversals), not by
 * the gains, so it would fail every step alike. The limit is printed at the
 * start, the last noise and peak with the result.
 */
HAL_COMP(ids);

HAL_PIN(en);

HAL_PIN(state);
HAL_PIN(param);
HAL_PIN(step);
HAL_PIN(rep);

HAL_PIN(freq);
HAL_PIN(amp);
HAL_PIN(min_pos);
HAL_PIN(max_pos);
HAL_PIN(max_vel);
HAL_PIN(max_acc);
HAL_PIN(acc_lim);  // conf0.max_acc: the profile's max_acc is capped at it
HAL_PIN(vel_lim);  // conf0.max_vel

HAL_PIN(pos);
HAL_PIN(pos_fb);  // fb_switch0.pos_fb: the profile starts where the rotor is
HAL_PIN(vel);
HAL_PIN(acc);
HAL_PIN(pos_cmd);
HAL_PIN(vel_cmd);
HAL_PIN(acc_cmd);

HAL_PIN(pos_error);
HAL_PIN(vel_error);

HAL_PIN(pos_bw);
HAL_PIN(vel_bw);
HAL_PIN(vel_d);
HAL_PIN(cur_bw);

HAL_PIN(ff);
HAL_PIN(kp);
HAL_PIN(ks);
HAL_PIN(kv);
HAL_PIN(kg);  // least cost cut a raise has to give
HAL_PIN(kd);

HAL_PINA(params, 3);
HAL_PINA(max_params, 3);

HAL_PIN(torque);      // pid0.torque_cmd
HAL_PIN(fb_torque);   // pid0.fb_torque_cmd
HAL_PIN(max_torque);  // conf0.max_force
HAL_PIN(fb_max);      // noise limit, fraction of max_torque
HAL_PIN(noise);       // last score: fb_torque noise, rms [Nm]
HAL_PIN(peak);        // last score: peak |torque| [Nm]

HAL_PIN(target);
HAL_PIN(cost);
HAL_PIN(min_cost);
HAL_PIN(auto_step);

HAL_PIN(timer);
HAL_PIN(sat);      // pid0.sat
HAL_PIN(pos_bw0);  // conf0.pos_bw: start value, 10 when 0
HAL_PIN(vel_bw0);  // conf0.vel_bw: start value, 100 when 0
HAL_PIN(vel_d0);   // conf0.vel_d: start value, 10 when 0
HAL_PIN(skipped);  // saturated cycles not scored, for the whole run

#define NOISE_HZ 50.0  // fb_torque above this counts as noise [Hz]
#define SAT_MAX 5      // saturated cycles in a row before a step fails
#define WARM_MAX 25    // warm-up cycles at most while pid saturates
#define CUT_MAX 5      // kd cuts of a saturating start value before moving on

struct ids_ctx_t {
  int warm;        // the warm-up cycle is over
  int n;           // cycles in the current score
  float t;         // time in the current score [s]
  float fb_lp;     // fb_torque low pass
  float noise_sq;  // integral of the noise squared
  float peak;
  int first;      // the next score is the first of this parameter
  int cuts;       // kd cuts of this parameter's start value
  int sat_cycle;  // this cycle saturated
  int sat_n;      // saturated cycles in a row
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct ids_ctx_t * ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;
  PIN(pos_bw)                = 10.0;
  PIN(vel_bw)                = 100.0;
  PIN(vel_d)                 = 10.0;
  PINA(params, 0)            = PIN(vel_bw);
  PINA(params, 1)            = 1.0 / PIN(vel_d);
  PINA(params, 2)            = PIN(pos_bw);

  PIN(kp) = 1.0;
  PIN(ks) = 0.0;
  PIN(kv) = 1.0;
  PIN(kg) = 0.05;
  PIN(kd) = 0.7;

  PIN(ff) = 1.0;

  PIN(max_vel) = 100.0;
  PIN(max_acc) = 1000.0;
  PIN(min_pos) = -10.0;
  PIN(max_pos) = 10.0;

  PIN(param) = 0.0;
  PIN(step)  = 0.1;
  PIN(rep)   = 2.0;
  PIN(fb_max) = 0.05;

  PIN(auto_step) = 1.0;
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct ids_ctx_t * ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      break;

    case 10:
      PIN(state)  = 1.1;
      PIN(target) = PIN(max_pos);

      if(PIN(auto_step) >= 1) {
        PIN(state) = 1.2;
      } else {
        printf("Tune PPI bandwidth and damping\n");
        printf("the motor will move\n");
        printf("ids0.state = 1.2 <font color='green'>to start</font>\n");
      }
      if(PIN(max_torque) > 0.0) {
        printf("<font color='green'># noise limit %f Nm rms (ids0.fb_max %f x conf0.max_force %f Nm)</font>\n", PIN(fb_max) * PIN(max_torque), PIN(fb_max), PIN(max_torque));
      } else {
        printf("<font color='red'>noise limit off</font>: ids0.max_torque reads 0, set conf0.max_force\n");
      }
      break;

    case 13:
      printf("conf0.pos_bw = %f <font color='green'># append to config</font>\n", PIN(pos_bw));
      printf("conf0.vel_bw = %f <font color='green'># append to config</font>\n", PIN(vel_bw));
      printf("conf0.vel_d = %f <font color='green'># append to config</font>\n", PIN(vel_d));
      printf("<font color='green'># last score: noise %f Nm rms, peak %f Nm, of conf0.max_force %f Nm</font>\n", PIN(noise), PIN(peak), PIN(max_torque));
      if(PIN(skipped) > 0.0) {
        printf("<font color='green'># %i saturated cycles were not scored</font>\n", (int)PIN(skipped));
      }
      if(PIN(max_torque) > 0.0 && PIN(peak) > PIN(max_torque)) {
        printf("<font color='red'>the profile peaks over conf0.max_force</font>: raise it to the drive's real torque or lower ids0.max_acc\n");
      }
      printf("done\n");
      PIN(state) = 1.4;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ids_ctx_t *ctx = (struct ids_ctx_t *)ctx_ptr;
  struct ids_pin_ctx_t *pins = (struct ids_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(acc_cmd) = 0.0;
      PIN(vel_cmd) = 0.0;
      PIN(acc)     = 0.0;
      PIN(vel)     = 0.0;

      PIN(param) = 0.0;
      PIN(timer) = 0.0;
      // follow the rotor while idle, so the profile starts without a step
      PIN(pos)     = PIN(pos_fb);
      PIN(pos_cmd) = mod(PIN(pos));
      PIN(target)  = PIN(max_pos);

      PIN(pos_bw)     = PIN(pos_bw0) > 0.0 ? PIN(pos_bw0) : 10.0;
      PIN(vel_bw)     = PIN(vel_bw0) > 0.0 ? PIN(vel_bw0) : 100.0;
      PIN(vel_d)      = PIN(vel_d0) > 0.0 ? PIN(vel_d0) : 10.0;
      PINA(params, 0) = PIN(vel_bw);
      PINA(params, 1) = 1.0 / PIN(vel_d);
      PINA(params, 2) = PIN(pos_bw);

      PINA(max_params, 0) = PIN(cur_bw) / 2.0;
      PINA(max_params, 1) = 1.0;

      PIN(min_cost) = 0.0;  // the cost to beat: the last kept score
      ctx->first    = 1;
      ctx->cuts     = 0;
      PIN(cost)     = 0.0;
      ctx->warm     = 0;
      ctx->n        = 0;
      ctx->t        = 0.0;
      ctx->fb_lp    = 0.0;
      ctx->noise_sq = 0.0;
      ctx->sat_cycle = 0;
      ctx->sat_n     = 0;
      PIN(skipped)   = 0.0;
      ctx->peak     = 0.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:
      // the profile stays inside the drive's own limits, conf0.max_acc and max_vel
      float p_acc = PIN(acc_lim) > 0.0 ? MIN(PIN(max_acc), PIN(acc_lim)) : PIN(max_acc);
      float p_vel = PIN(vel_lim) > 0.0 ? MIN(PIN(max_vel), PIN(vel_lim)) : PIN(max_vel);
      PIN(pos) += PIN(vel) * period + PIN(acc) * period * period / 2.0;
      PIN(vel) += PIN(acc) * period;
      float to_go      = PIN(target) - PIN(pos);
      float time_to_go = sqrtf(2.0 * ABS(to_go) / p_acc);
      float acc        = p_acc * SIGN(to_go);
      float vel        = acc * time_to_go;
      vel              = LIMIT(vel, p_vel);
      acc              = (vel - PIN(vel)) / period;

      if(time_to_go < period) {
        time_to_go = 0.0;
        to_go      = 0.0;
        vel        = 0.0;
        acc        = 0.0;
        PIN(pos)   = PIN(target);
        PIN(vel)   = 0.0;
        PIN(acc)   = 0.0;
      }

      PIN(acc) = LIMIT(acc, p_acc);

      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) = PIN(vel) * PIN(ff);
      PIN(acc_cmd) = PIN(acc) * PIN(ff);

      PIN(cost) += ABS(PIN(pos_error)) * PIN(kp) * period;
      PIN(cost) += PIN(pos_error) * PIN(pos_error) * PIN(ks) * period;
      PIN(cost) += PIN(vel_error) * PIN(vel_error) * PIN(kv) * period;

      ctx->fb_lp += (PIN(fb_torque) - ctx->fb_lp) * LIMIT(2.0 * M_PI * NOISE_HZ * period, 1.0);
      ctx->noise_sq += (PIN(fb_torque) - ctx->fb_lp) * (PIN(fb_torque) - ctx->fb_lp) * period;
      ctx->peak = MAX(ctx->peak, ABS(PIN(torque)));
      ctx->t += period;
      if(PIN(sat) > 0.0 || (PIN(max_torque) > 0.0 && ABS(PIN(torque)) >= 0.99 * PIN(max_torque))) {
        ctx->sat_cycle = 1;
      }

      PIN(timer) += period;
      if(PIN(timer) < (ABS(PIN(max_pos) - PIN(min_pos)) / p_vel + 2.0 * p_vel / p_acc)) {
        PIN(target) = PIN(max_pos);
      } else {
        PIN(target) = PIN(min_pos);
      }
      if(PIN(timer) > 2.0 * (ABS(PIN(max_pos) - PIN(min_pos)) / p_vel + 2.0 * p_vel / p_acc)) {
        PIN(timer) = 0.0;
      }

      int score = 0, sat_fail = 0;
      if(PIN(timer) == 0.0) {
        int sat = ctx->sat_cycle;
        ctx->sat_cycle = 0;
        ctx->sat_n     = sat ? ctx->sat_n + 1 : 0;
        if(sat) {
          PIN(skipped)++;
        }
        if(!ctx->warm) {  // warm-up: not scored, runs on while pid saturates
          if(!sat || ctx->sat_n >= WARM_MAX) {
            ctx->warm  = 1;
            ctx->sat_n = 0;
          }
          PIN(cost) = 0.0;
          ctx->t = ctx->noise_sq = ctx->peak = 0.0;
        } else if(sat && ctx->sat_n < SAT_MAX) {  // drop it, same gains again
          ctx->n    = 0;
          PIN(cost) = 0.0;
          ctx->t = ctx->noise_sq = ctx->peak = 0.0;
        } else if(sat) {  // saturates every time: these gains fail
          ctx->sat_n = 0;
          sat_fail   = 1;
          score      = 1;
          ctx->n     = MAX(ctx->n, 1);
        } else if(++ctx->n >= MAX(PIN(rep), 1.0)) {
          score = 1;
        }
      }

      if(score) {
        PIN(cost) /= ctx->n;
        PIN(noise) = ctx->t > 0.0 ? sqrtf(ctx->noise_sq / ctx->t) : 0.0;
        PIN(peak)  = ctx->peak;
        int noisy  = sat_fail || (PIN(max_torque) > 0.0 && PIN(noise) > PIN(fb_max) * PIN(max_torque));
        ctx->n = 0;
        ctx->t = ctx->noise_sq = ctx->peak = 0.0;

        PINA(max_params, 2) = PINA(params, 0) * 2.0;

        int k = (int)PIN(param);
        if(PINA(params, k) > PINA(max_params, k)) {
          PINA(params, k) = PINA(max_params, k);
          ctx->first      = 1;
          PIN(param)++;
        } else if(ctx->first && sat_fail && ++ctx->cuts < CUT_MAX) {
          PINA(params, k) *= PIN(kd);  // the start value saturates: cut it, try again
        } else if(ctx->first && noisy) {  // noisy at the start value: cut it
          PINA(params, k) *= PIN(kd);
          ctx->cuts = 0;
          PIN(param)++;
        } else if(!ctx->first && (noisy || PIN(cost) > PIN(min_cost) * (1.0 - PIN(kg)))) {
          PINA(params, k) /= 1.0 + PIN(step);  // back to the last kept value
          ctx->first = 1;
          ctx->cuts  = 0;
          PIN(param)++;
        } else {
          PIN(min_cost) = PIN(cost);
          ctx->first    = 0;
          PINA(params, k) *= 1.0 + PIN(step);
        }

        PIN(cost) = 0.0;
      }

      PIN(pos_bw) = PINA(params, 2);
      PIN(vel_bw) = PINA(params, 0);
      PIN(vel_d)  = 1.0 / PINA(params, 1);

      if(PIN(param) > 2.0) {
        PIN(acc_cmd) = 0.0;
        PIN(vel_cmd) = 0.0;
        PIN(param)   = 0.0;

        PIN(state) = 1.3;
      }
      break;
  }
}


hal_comp_t ids_comp_struct = {
    .name      = "ids",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ids_ctx_t),
    .pin_count = sizeof(struct ids_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};