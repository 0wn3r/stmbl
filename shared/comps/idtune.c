#include "idtune_comp.h"
#include "hal.h"
#include "string.h"
#include "defines.h"
#include "angle.h"
#include <math.h>

// Tunes the two running compensations that id_pmsm leaves at 0: hv0.drop_k,
// the dead time compensation, and hv0.adv, the commutation advance. Run it
// after id_pmsm, with r, l, lq, psi, polecount and mot_fb_offset in the config,
// the load off, and commutation on the normal running path (hv0.pos and hv0.vel
// are not touched here, so what gets tuned is what runs).
//
// drop_k: square steps on d through zero, no torque, the rotor held by nothing
// but its own cogging. Undercompensated, the current creeps toward the command
// for tens of ms after each crossing; overcompensated, it overshoots. The
// signed mean of (i - cmd) * sign(cmd) after the edge crosses zero between
// the two, so bisect on it. On X by hand: 0 -> 42 ms to 90%, 0.6 -> 9.7 ms,
// 0.75 -> 1.8 ms and 6% overshoot, 0.92 -> 60% overshoot.
//
// adv: a speed dwell, then compare ud with what the motor alone asks for,
//   ud = r id - w lq iq           w = electrical speed
// An angle lag d of the commutation turns part of uq into ud,
//   ud - ud_expected = -uq sin(d)
// and d = w * (delay - adv), so each dwell moves adv by -(ud - ud_exp) / uq / w.
// By hand on X at 100 rad/s: 0 -> -6.07 V, 0.5 ms -> -3.46 V, 0.9 ms -> -1.60 V
// against -1.63 V expected.

HAL_COMP(idtune);

HAL_PIN(en);
HAL_PIN(en_out);
HAL_PIN(state);
HAL_PIN(cmd_mode);
HAL_PIN(d_cmd);
HAL_PIN(q_cmd);

HAL_PIN(adv);     // out, to hv0.adv [s]
HAL_PIN(drop_k);  // out, to hv0.drop_k

HAL_PIN(id_fb);
HAL_PIN(iq_fb);
HAL_PIN(ud_fb);
HAL_PIN(uq_fb);
HAL_PIN(vel_fb);  // mechanical speed, for the speed loop [rad/s]
HAL_PIN(vel_e);   // electrical speed, what hv0.vel sees [rad/s]
HAL_PIN(dc_volt);

HAL_PIN(r);
HAL_PIN(l);
HAL_PIN(lq);  // 0 = same as l

HAL_PIN(step_cur);   // *parameter*, d step amplitude [A]
HAL_PIN(step_freq);  // *parameter*, d step frequency [Hz]
HAL_PIN(test_cur);   // *parameter*, q current limit of the speed loop [A]
HAL_PIN(test_vel);   // *parameter*, dwell speed, mechanical [rad/s]
HAL_PIN(ki);
HAL_PIN(vel_bw);
HAL_PIN(cur_sum);

HAL_PIN(dk_err);   // signed mean step error of the last drop_k try, fraction of step_cur
HAL_PIN(ud_err);   // ud - ud expected in the last dwell [V]
HAL_PIN(ud_exp);   // ud expected in the last dwell [V]
HAL_PIN(uq_mean);  // uq in the last dwell [V]
HAL_PIN(iq_mean);  // iq in the last dwell [A]
HAL_PIN(w_mean);   // electrical speed in the last dwell [rad/s]
HAL_PIN(fail);     // 0 none, 1 stalled, 2 never settled, 3 adv did not converge

#define DK_MAX 1.2     // drop_k search range 0..DK_MAX
#define DK_ITER 7      // bisection steps, DK_MAX / 2^7 = 0.01
#define DK_CYCLES 5    // step cycles per try, the first one is not measured
#define DK_SKIP 0.002  // ignore this long after each edge: the loop's own rise [s]

#define ADV_MAX 0.003    // [s]
#define ADV_ITER 12      // dwells before giving up
#define ADV_TOL 0.00001  // done when a dwell moves adv less than this [s]
#define ADV_DWELL 0.5    // averaging time per dwell [s]
#define VEL_SETTLE 0.5   // speed inside the band this long before the first dwell [s]
#define VEL_BAND 0.15    // settled band, fraction of test_vel
#define VEL_SPINUP 8.0   // give up reaching test_vel after [s]
#define VEL_STALL 0.5    // q current, fraction of test_cur, that has to turn the rotor

struct idtune_ctx_t {
  float t;      // time in this try or stage
  float lo, hi; // drop_k bracket
  uint8_t it;   // tries or dwells done
  uint8_t stage;  // adv: 0 spin up, 1 dwell
  float se;     // sum of the signed step error
  uint32_t n;   // samples in the sums
  float vel_lp;
  float settle_t;
  float sud, suq, sid, siq, sw;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_pin_ctx_t *pins = (struct idtune_pin_ctx_t *)pin_ptr;

  PIN(step_cur)  = 1.0;
  PIN(step_freq) = 10.0;
  PIN(test_cur)  = 3.0;
  PIN(test_vel)  = 100.0;
  PIN(ki)        = 1.0;
  PIN(vel_bw)    = 20.0;
}

static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_ctx_t *ctx = (struct idtune_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct idtune_ctx_t));
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_pin_ctx_t *pins = (struct idtune_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 13:
      printf("hv0.drop_k = %f <font color='green'># append to config</font>\n", PIN(drop_k));
      printf("hv0.adv = %f <font color='green'># append to config</font>\n", PIN(adv));
      printf("<font color='green'># drop_k from +-%f A steps on d at %f V link.\n", PIN(step_cur), PIN(dc_volt));
      printf("# adv from ud at %f el. rad/s: %f V, expected %f V,\n", PIN(w_mean), PIN(ud_exp) + PIN(ud_err), PIN(ud_exp));
      printf("# uq %f V, iq %f A. keep drop_k at 0 while running id_pmsm.</font>\n", PIN(uq_mean), PIN(iq_mean));
      printf("done\n");
      PIN(state) = 1.5;
      break;

    case 14:
      if(PIN(fail) == 1.0) {
        printf("<font color='red'>adv failed</font>: the rotor stalled. check the load is off and\n");
        printf("conf0.polecount, conf0.mot_fb_offset and out_rev\n");
      } else if(PIN(fail) == 2.0) {
        printf("<font color='red'>adv failed</font>: the speed never settled inside %i%% of %f rad/s\n", (int)(VEL_BAND * 100.0), PIN(test_vel));
      } else {
        printf("<font color='red'>adv failed</font>: it did not converge in %i dwells, last %f s,\n", ADV_ITER, PIN(adv));
        printf("ud off by %f V. check conf0.lq and conf0.r\n", PIN(ud_err));
      }
      printf("hv0.drop_k = %f <font color='green'># this part is measured</font>\n", PIN(drop_k));
      PIN(state) = 1.5;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idtune_ctx_t *ctx      = (struct idtune_ctx_t *)ctx_ptr;
  struct idtune_pin_ctx_t *pins = (struct idtune_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 1.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_sum)  = 0.0;
      PIN(fail)     = 0.0;
      if(PIN(en) > 0.0) {
        memset(ctx, 0, sizeof(struct idtune_ctx_t));
        ctx->hi       = DK_MAX;
        PIN(adv)      = 0.0;
        PIN(drop_k)   = DK_MAX / 2.0;
        PIN(state)    = 1.1;
      }
      break;

    case 11: {  // drop_k
      PIN(en_out) = 1.0;
      PIN(q_cmd)  = 0.0;
      float half  = 0.5 / MAX(PIN(step_freq), 1.0);
      float tc    = fmodf(ctx->t, 2.0 * half);
      float cmd   = tc < half ? PIN(step_cur) : -PIN(step_cur);
      float since = tc < half ? tc : tc - half;
      PIN(d_cmd)  = cmd;

      if(ctx->t >= 2.0 * half && since >= DK_SKIP) {
        ctx->se += (PIN(id_fb) - cmd) / cmd;
        ctx->n++;
      }
      ctx->t += period;

      if(ctx->t >= DK_CYCLES * 2.0 * half) {
        PIN(dk_err) = ctx->n ? ctx->se / (float)ctx->n : 0.0;
        if(PIN(dk_err) < 0.0) {
          ctx->lo = PIN(drop_k);  // current short of the command: under
        } else {
          ctx->hi = PIN(drop_k);
        }
        PIN(drop_k) = (ctx->lo + ctx->hi) / 2.0;
        ctx->t      = 0.0;
        ctx->se     = 0.0;
        ctx->n      = 0;
        if(++ctx->it >= DK_ITER) {
          PIN(d_cmd) = 0.0;
          ctx->it    = 0;
          ctx->stage = 0;
          PIN(state) = 1.2;
        }
      }
    } break;

    case 12: {  // adv
      float v_t = PIN(test_vel);
      ctx->vel_lp += (PIN(vel_fb) - ctx->vel_lp) * period / 0.05;
      ctx->t += period;

      PIN(en_out) = 1.0;
      PIN(d_cmd)  = 0.0;
      float vel_error = LIMIT(v_t - PIN(vel_fb), v_t / 100.0);
      PIN(cur_sum) += PIN(ki) * vel_error * period;
      PIN(cur_sum) = LIMIT(PIN(cur_sum), PIN(test_cur));
      PIN(q_cmd)   = LIMIT(PIN(vel_bw) * period * vel_error + PIN(cur_sum), PIN(test_cur));

      if(ABS(ctx->vel_lp) < v_t * 0.1 && ABS(PIN(q_cmd)) > PIN(test_cur) * VEL_STALL) {
        PIN(fail) = 1.0;
      } else if(ctx->stage == 0) {  // spin up
        if(ABS(ctx->vel_lp - v_t) < v_t * VEL_BAND) {
          ctx->settle_t += period;
        } else {
          ctx->settle_t = 0.0;
        }
        if(ctx->settle_t >= VEL_SETTLE) {
          ctx->stage = 1;
          ctx->t     = 0.0;
        } else if(ctx->t > VEL_SPINUP) {
          PIN(fail) = 2.0;
        }
      } else {  // dwell
        ctx->sud += PIN(ud_fb);
        ctx->suq += PIN(uq_fb);
        ctx->sid += PIN(id_fb);
        ctx->siq += PIN(iq_fb);
        ctx->sw += PIN(vel_e);
        ctx->n++;
        if(ctx->t >= ADV_DWELL) {
          float n      = (float)ctx->n;
          float lq     = PIN(lq) > 0.0 ? PIN(lq) : PIN(l);
          PIN(w_mean)  = ctx->sw / n;
          PIN(uq_mean) = ctx->suq / n;
          PIN(iq_mean) = ctx->siq / n;
          PIN(ud_exp)  = PIN(r) * ctx->sid / n - PIN(w_mean) * lq * PIN(iq_mean);
          PIN(ud_err)  = ctx->sud / n - PIN(ud_exp);
          float step   = 0.0;
          if(ABS(PIN(uq_mean)) > 1.0 && ABS(PIN(w_mean)) > 1.0) {
            step = -PIN(ud_err) / PIN(uq_mean) / PIN(w_mean);
          }
          PIN(adv) = CLAMP(PIN(adv) + step, 0.0, ADV_MAX);
          ctx->sud = ctx->suq = ctx->sid = ctx->siq = ctx->sw = 0.0;
          ctx->n   = 0;
          ctx->t   = 0.0;
          ctx->it++;
          if(ABS(step) < ADV_TOL) {
            PIN(state) = 1.3;
          } else if(ctx->it >= ADV_ITER) {
            PIN(fail) = 3.0;
          }
        }
      }
      if(PIN(fail) > 0.0) {
        PIN(state) = 1.4;
      }
    } break;

    case 13:
    case 14:
    case 15:
      PIN(en_out)  = 0.0;
      PIN(d_cmd)   = 0.0;
      PIN(q_cmd)   = 0.0;
      PIN(cur_sum) = 0.0;
      break;
  }
}

hal_comp_t idtune_comp_struct = {
    .name      = "idtune",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idtune_ctx_t),
    .pin_count = sizeof(struct idtune_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
