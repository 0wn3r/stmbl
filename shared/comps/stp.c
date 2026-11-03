#include "stp_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `stp` is a simple point to point trajectory planner with velocity and acceleration limits, soft limits and a jog input. F4 component, used in `conf/template/jog_cmd.txt` (`stp0.jog = jog0.jog`, `rev0.in = stp0.mpos`) and `conf/festo.txt`.
*
* ## Component Explanation
* All work is done in `rt`. Positions are not wrapped, only `mpos` is.
*
* 1. **Target generation**:
* - `target` is moved by `vel_ext_cmd + jog * max_vel` (limited to 0.99 * `max_vel`) and `acc_ext_cmd` (limited to 0.99 * `max_acc`). `vel_ext_cmd` itself is integrated with `acc_ext_cmd` and written back.
* - `target` is clamped to `min_pos`..`max_pos`.
*
* 2. **Planning** (every period):
* - `dtg = target - pos`. The time to stop at the target from standstill with `max_acc` is `sqrt(2 |dtg| / max_acc)`, rounded up to whole periods (`ttg`).
* - The velocity for that time is limited to `max_vel`; the acceleration needed to reach it in one period is limited to `max_acc` and output as `acc_cmd`.
* - `pos` and `vel_cmd` are integrated with `acc_cmd`.
*
* 3. **Target reached**:
* - When less than one period to go and `|vel_cmd|` < `max_acc * period`, `pos` is set to `target` and `vel_cmd` to 0.
* - `at_target` = 1 when at most one period to go and `|vel_cmd|` < `max_vel / 10000`.
*
* 4. **Jerk limit** (`max_jerk` > 0):
* - The planner above runs unchanged on internal state, and `pos`, `vel_cmd` and `acc_cmd` are its moving average over `tj = 2 * max_acc / max_jerk` (rounded to whole periods). A box average of a trapezoid is an S-curve: the peak acceleration and velocity stay, the move ends exactly at `target` and takes `tj` longer, and the command is delayed by `tj / 2`. Where the trapezoid's acceleration flips from +`max_acc` to -`max_acc` (a move too short to reach `max_vel`) the jerk is `max_jerk`; where it starts from or ends at 0 it is half that.
* - The planner then also brakes with the exact discrete stopping distance: the velocity it ends a period with is the one from which braking at `max_acc` stops at the target (`v^2 / (2 max_acc) + v * period / 2 = dtg - vel * period / 2`). The plain trapezoid instead overshoots the target by a few mrad and flips the acceleration between +-`max_acc` there for some periods.
* - The three outputs are averages of one consistent state (`pos` += `vel_cmd` * period + `acc_cmd` * period^2 / 2 holds for them too), so `vel_cmd` and `acc_cmd` can feed `rev0.in_d` / `in_dd` as feed forward.
* - `dtg` and `ttg` are those of the averaged output; `at_target` also waits for it.
* - The average holds at most 256 periods (51.2 ms at 5 kHz): a lower `max_jerk` gets `2 * max_acc / (256 * period)`.
* - A change of the average length (`max_jerk` or `max_acc`) or a write to `pos` from outside restarts the planner at the current `pos`, at rest: change them only at standstill. `max_jerk` 0 is the plain trapezoid (default).
*/

HAL_COMP(stp);

HAL_PIN(target);       // *input/output*, Target position, clamped and moved by jog (rad)
HAL_PIN(vel_ext_cmd);  // *input/output*, External velocity, integrated from acc_ext_cmd (rad/s)
HAL_PIN(acc_ext_cmd);  // *input*, External acceleration (rad/s^2)
HAL_PIN(jog);          // *input*, Jog -1..1, multiplied with max_vel

HAL_PIN(pos);      // *output*, Planned position, not wrapped (rad)
HAL_PIN(mpos);     // *output*, Planned position, wrapped (rad, +-pi)
HAL_PIN(vel_cmd);  // *output*, Planned velocity (rad/s)
HAL_PIN(acc_cmd);  // *output*, Planned acceleration (rad/s^2)

HAL_PIN(max_pos);  // *parameter*, Upper soft limit (rad), default 20pi
HAL_PIN(min_pos);  // *parameter*, Lower soft limit (rad), default -20pi
HAL_PIN(max_vel);  // *parameter*, Maximum velocity (rad/s), default 2pi
HAL_PIN(max_acc);  // *parameter*, Maximum acceleration (rad/s^2), default 20pi
HAL_PIN(max_jerk);  // *parameter*, Maximum jerk (rad/s^3), 0 = trapezoid (default), see the jerk limit section for the lowest value

HAL_PIN(dtg);        // *output*, Distance to go (rad)
HAL_PIN(ttg);        // *output*, Time to go (s)
HAL_PIN(at_target);  // *output*, 1 = target reached

#define STP_JN 256  // longest jerk average, periods (2 KB of ctx)

struct stp_ctx_t {
  uint32_t n, idx;     // average length (0 = not running), ring index
  float p, v, a;       // planner state while jerk limited
  float pb[STP_JN];    // planner position and velocity at the end of the last n periods
  float vb[STP_JN];
  double psum;         // sum of pb, double: it is added to and taken from every period
  float pos_out;       // pos as written last period, to see a write from outside
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct stp_ctx_t *ctx      = (struct stp_ctx_t *)ctx_ptr;
  struct stp_pin_ctx_t *pins = (struct stp_pin_ctx_t *)pin_ptr;
  PIN(target)                = 0.0;
  PIN(max_vel)               = 1.0 * 2.0 * M_PI;
  PIN(max_acc)               = 10.0 * 2.0 * M_PI;
  PIN(max_pos)               = 10.0 * 2.0 * M_PI;
  PIN(min_pos)               = -10.0 * 2.0 * M_PI;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct stp_ctx_t *ctx      = (struct stp_ctx_t *)ctx_ptr;
  struct stp_pin_ctx_t *pins = (struct stp_pin_ctx_t *)pin_ptr;

  float max_acc = MAX(PIN(max_acc), 0.01);
  float max_vel = MAX(PIN(max_vel), 0.01);

  // jog input
  float acc_ext_cmd = LIMIT(PIN(acc_ext_cmd), PIN(max_acc) * 0.99);
  float vel_ext_cmd = LIMIT(PIN(vel_ext_cmd) + PIN(jog) * max_vel, PIN(max_vel) * 0.99);
  PIN(target) += vel_ext_cmd * period + acc_ext_cmd * period * period / 2.0;
  PIN(vel_ext_cmd) += acc_ext_cmd * period;
  PIN(vel_ext_cmd) = LIMIT(PIN(vel_ext_cmd), max_vel * 0.99);

  // pos input
  float target = CLAMP(PIN(target), PIN(min_pos), PIN(max_pos));
  PIN(target)  = target;

  // planner state: the pins, or with a jerk limit the internal one, whose
  // moving average goes to the pins
  uint32_t n = 0;
  if(PIN(max_jerk) > 0.0) {
    n = CLAMP((uint32_t)(2.0 * max_acc / (PIN(max_jerk) * period) + 0.5), 1, STP_JN);
  }
  if(n != ctx->n || (n && PIN(pos) != ctx->pos_out)) {
    // (re)start at the current output, at rest
    ctx->p = PIN(pos);
    ctx->v = 0.0;
    ctx->a = 0.0;
    if(!n) {
      PIN(vel_cmd) = 0.0;
      PIN(acc_cmd) = 0.0;
    }
    for(int i = 0; i < STP_JN; i++) {
      ctx->pb[i] = ctx->p;
      ctx->vb[i] = 0.0;
    }
    ctx->psum    = (double)ctx->p * n;
    ctx->idx     = 0;
    ctx->n       = n;
    ctx->pos_out = ctx->p;
  }
  float *pp = n ? &ctx->p : &PIN(pos);
  float *vp = n ? &ctx->v : &PIN(vel_cmd);
  float *ap = n ? &ctx->a : &PIN(acc_cmd);

  // update
  *pp += *vp * period + *ap * period * period / 2.0;
  *vp += *ap * period;

  // distance to go
  float to_go = target - *pp;

  // time to go
  float time_to_go = sqrtf(2.0 * ABS(to_go) / max_acc);

  // real time to go
  int periods_to_go = ceilf(time_to_go / period);

  // calc new acc
  float acc = 0.0;
  if(periods_to_go) {
    acc = 2.0 * to_go / (periods_to_go * periods_to_go * period * period);
  }

  float vel = acc * periods_to_go * period;
  if(n) {
    // The velocity to end this period with so that braking at max_acc from
    // there stops exactly at the target: with pos stepped by the mean of
    // two velocities, braking from v takes v^2 / (2 max_acc), so
    // v^2 / (2 max_acc) + v * period / 2 = to_go - vel_cmd * period / 2.
    // Near the target it asks for the rest in one period instead of
    // flipping the acceleration every period.
    float e  = to_go - *vp * period / 2.0;
    float vb = max_acc * (sqrtf(period * period / 4.0 + 2.0 * ABS(e) / max_acc) - period / 2.0);
    vel      = e > 0.0 ? vb : -vb;
  }
  vel = LIMIT(vel, max_vel);
  acc       = (vel - *vp) / period;
  acc       = LIMIT(acc, max_acc);

  if(time_to_go < period && ABS(*vp) < max_acc * period) {
    acc = 0.0;
    *vp = 0.0;
    *pp = PIN(target);
  }

  *ap = acc;

  if(n) {
    // Moving average of the planner state over the last n periods. The ring
    // holds the state each period ends in (p1, v1), so the oldest entry is
    // the one n periods back: the window's acc is its velocity change, its
    // vel the position change less the acc term, and the average position
    // is the ring's sum before this period's entry goes in.
    float p1          = *pp + *vp * period + acc * period * period / 2.0;
    float v1          = *vp + acc * period;
    float pold        = ctx->pb[ctx->idx];
    float vold        = ctx->vb[ctx->idx];
    float acc_f       = (v1 - vold) / (n * period);
    PIN(acc_cmd)      = acc_f;
    PIN(vel_cmd)      = (p1 - pold) / (n * period) - acc_f * period / 2.0;
    PIN(pos)          = (float)(ctx->psum / n);
    ctx->psum += (double)p1 - (double)pold;
    ctx->pb[ctx->idx] = p1;
    ctx->vb[ctx->idx] = v1;
    ctx->idx          = (ctx->idx + 1) % n;
    ctx->pos_out      = PIN(pos);
  }

  PIN(dtg)     = to_go;
  PIN(ttg)     = periods_to_go * period;
  PIN(mpos)    = mod(PIN(pos));

  int settled = 1;
  if(n) {  // the average arrives n periods after the planner
    PIN(dtg) = target - PIN(pos);
    PIN(ttg) += n * period;
    settled  = ABS(PIN(dtg)) <= 1e-6 * (1.0 + ABS(target));
  }

  if((periods_to_go <= 1) & (ABS(PIN(vel_cmd)) < max_vel / 10000.0) & settled) {
    PIN(at_target) = 1.0;
  } else {
    PIN(at_target) = 0.0;
  }
}

const hal_comp_t stp_comp_struct = {
    .name      = "stp",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct stp_ctx_t),
    .pin_count = sizeof(struct stp_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
