#include "ids_comp.h"
#include "hal.h"
#include "string.h"
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


// ---------------------------------------------------------------------------
// ring down estimator for pid0.j_lpf
//
// pid.c models a compliant coupling by believing it drives j_mot + j_sys below
// j_lpf and only j_mot above it. Collapsing its acc_cmd_lp / acc_cmd_hp split,
// the acc_cmd -> fb_torque transfer is
//
//   j_mot + j_sys / (1 + s / wc)
//
// a lag network with its pole at wc and its zero at wc * (1 + j_sys / j_mot).
// The corner belongs at the two mass anti resonance, sqrt(K / j_sys): that is
// where the load stops following and the inertia the motor actually sees starts
// falling toward j_mot.
//
// K is the one quantity the id sequence never measures, and searching for j_lpf
// against the cost below does not work -- the cost is dominated by the low
// frequency tracking error, which is exactly the band j_lpf does not act in. So
// measure it instead. Kick the axis, time the ring, and let the stiffness drop
// out: the resonance and the anti resonance differ only by the inertia ratio,
// which id_mot and id_sys already give us.
//
//   f_res / f_anti = sqrt(1 + j_sys / j_mot)   =>   j_lpf = f_res / sqrt(...)
//
// Frequency is the right thing to measure here because it survives a messy
// excitation: several torque edges superposed at the same mode are still that
// mode's period, even though the amplitude and phase are a mess.
//
// This lives on its own branch of the state machine (2.x) and touches nothing
// the gain search uses, so it is safe to run on a machine that is already tuned.
HAL_PIN(j_mot);  // *input*, motor inertia [kgm^2], conf0.j
HAL_PIN(j_sys);  // *input*, load inertia [kgm^2], conf0.j_sys

HAL_PIN(ring_pos);      // *parameter*, excitation move size [rad]
HAL_PIN(ring_acc);      // *parameter*, excitation acceleration [rad/s^2], 0 = max_acc
HAL_PIN(ring_dwell);    // *parameter*, measuring window after each kick [s]
HAL_PIN(ring_reps);     // *parameter*, kick + dwell pairs to average
HAL_PIN(ring_hp_hz);    // *parameter*, detector high pass corner [Hz]
HAL_PIN(ring_min_amp);  // *parameter*, ring amplitude below which a dwell is noise [rad/s]

HAL_PIN(ring_sig);   // high passed vel_error, what the detector actually sees
HAL_PIN(ring_amp);   // peak of that in the current dwell [rad/s]
HAL_PIN(ring_n);     // half periods accepted, 0 = nothing was measured
HAL_PIN(ring_ok);    // 1 = f_ring and j_lpf are usable
HAL_PIN(f_ring);     // measured resonance [Hz]
HAL_PIN(zeta_ring);  // measured damping ratio
HAL_PIN(j_lpf);      // computed pid0.j_lpf corner [Hz]

struct ring_t {
  uint8_t armed;      // the 2.2 entry hook has run
  uint8_t spent;      // this dwell's ring has decayed into the noise
  int8_t sign;        // last polarity the schmitt trigger latched
  uint16_t n_cross;   // trigger events in this dwell
  uint16_t n_dwell;   // half periods accepted in this dwell
  uint16_t n_half;    // half periods accepted over all dwells
  uint16_t n_ln;      // dwells that yielded a log decrement
  uint16_t rep;
  float lp;           // high pass state
  float t;            // time inside the current kick + dwell cycle
  float t_last;       // when the last trigger fired
  float env;          // largest peak seen so far in this dwell
  float pk;           // peak since the last trigger
  float pk_prev;      // peak of the half cycle before it, which sets the band
  float a_first;      // first and last accepted peak, for the log decrement
  float a_last;
  float pos0;         // the axis parks here between kicks
  float sum_half;
  float sum_ln;
};

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
  struct ring_t r;  // ring down estimator, state 2.x
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

  PIN(ring_pos)     = 0.2;
  PIN(ring_acc)     = 0.0;  // 0 = fall back to max_acc
  PIN(ring_dwell)   = 1.0;
  PIN(ring_reps)    = 4.0;
  PIN(ring_hp_hz)   = 5.0;
  PIN(ring_min_amp) = 0.02;
}

// The latches and the accumulators live in ctx, and ctx survives a stop, so a
// restart has to come back to a cleared detector rather than half a measurement
// taken against whatever the axis was doing when the loop stopped.
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct ids_ctx_t *ctx = (struct ids_ctx_t *)ctx_ptr;
  memset(ctx, 0, sizeof(struct ids_ctx_t));
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

    case 20:
      PIN(state) = 2.1;
      printf("Measure the coupling resonance for pid0.j_lpf\n");
      printf("the motor will twitch %f rad, %f times\n", PIN(ring_pos), PIN(ring_reps));
      if(PIN(auto_step) >= 1) {
        PIN(state) = 2.2;
      } else {
        printf("ids0.state = 2.2 <font color='green'>to start</font>\n");
      }
      break;

    case 23:
      if(PIN(ring_ok) > 0.0) {
        printf("resonance = %f Hz, damping = %f, from %f half periods\n", PIN(f_ring), PIN(zeta_ring), PIN(ring_n));
        printf("conf0.j_lpf = %f <font color='green'># append to config</font>\n", PIN(j_lpf));
        printf("<font color='green'># j_lpf is the anti resonance, f_ring / sqrt(1 + j_sys / j_mot).\n");
        printf("# above it pid.c drops the load out of the torque model.\n");
        printf("# set it BEFORE id_pid. it changes the plant the gains are tuned\n");
        printf("# against, so a j_lpf set afterwards invalidates the tune.\n");
        printf("# it only pays if the velocity loop crosses near this frequency.\n");
        printf("# with conf0.vel_bw well under %f rad/s there is nothing to correct\n", 2.0 * M_PI * PIN(j_lpf));
        printf("# and j_lpf only costs phase, so leave it at 0.</font>\n");
        if(PIN(zeta_ring) > 0.0) {
          printf("<font color='green'># and for zv_ip, if you ever wire it up:\n");
          printf("# zv_ip0.natural_frequency = %f\n", PIN(f_ring));
          printf("# zv_ip0.damping_ratio = %f</font>\n", PIN(zeta_ring));
        }
      } else if(PIN(f_ring) > 0.0) {
        printf("<font color='red'>resonance = %f Hz found, but j_lpf needs the inertia ratio</font>\n", PIN(f_ring));
        printf("j_mot = %f, j_sys = %f -- run id_mot and id_sys, and check that\n", PIN(j_mot), PIN(j_sys));
        printf("ids0.j_mot and ids0.j_sys are wired\n");
      } else if(PIN(ring_n) > 0.0 && PIN(ring_amp) > PIN(ring_min_amp) * 3.0) {
        // it rang, but it died before there was anything to time. That is a
        // well damped coupling, which is the good case: nothing to compensate.
        printf("<font color='green'>ring died too fast to time</font>: %f half periods at %f rad/s\n", PIN(ring_n), PIN(ring_amp));
        printf("# the coupling is well damped. there is no resonance worth\n");
        printf("# modelling, so leave conf0.j_lpf at 0.\n");
      } else {
        printf("<font color='red'>no ring found</font>: %f half periods, peak %f rad/s\n", PIN(ring_n), PIN(ring_amp));
        printf("either the coupling is stiff enough that there is nothing here,\n");
        printf("or the mode is faster than this loop rate can time (a period\n");
        printf("measurement needs about 20 samples per cycle),\n");
        printf("or the kick was too small: raise ids0.ring_acc or ids0.ring_pos.\n");
        printf("if the mode is slower than ids0.ring_hp_hz (%f Hz) the high pass\n", PIN(ring_hp_hz));
        printf("ate it -- drop that. put ids0.ring_sig on a scope wave to see\n");
        printf("what the detector is working with.\n");
      }
      printf("done\n");
      PIN(state) = 2.4;
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

  // Anything other than the measuring state disarms it, so a second run always
  // re-enters through the hook below with cleared accumulators. Doing it here
  // rather than in nrt keeps ctx to a single writer.
  if((int)(PIN(state) * 10.0 + 0.5) != 22) {
    ctx->r.armed = 0;
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

    case 22: {
      // the same conf0 caps as the gain search
      float r_lim    = PIN(acc_lim) > 0.0 ? MIN(PIN(max_acc), PIN(acc_lim)) : PIN(max_acc);
      float ring_acc = PIN(ring_acc) > 0.0 ? PIN(ring_acc) : r_lim;
      ring_acc       = MAX(PIN(acc_lim) > 0.0 ? MIN(ring_acc, PIN(acc_lim)) : ring_acc, 1.0);
      float r_vmax   = PIN(vel_lim) > 0.0 ? MIN(PIN(max_vel), PIN(vel_lim)) : PIN(max_vel);
      float ring_pos = MAX(PIN(ring_pos), 0.001);
      float dwell    = MAX(PIN(ring_dwell), 0.1);
      // what the generator needs to get there and stop again, with room to spare
      float move_t = 2.0 * sqrtf(2.0 * ring_pos / ring_acc) + 2.0 * r_vmax / ring_acc + 0.05;
      float f_max  = MIN(1.0 / period / 20.0, 500.0);
      float f_min  = MAX(2.0 * PIN(ring_hp_hz), 3.0 / dwell);

      if(!ctx->r.armed) {
        // start where the rotor is, so the first kick is not a lurch
        ctx->r.pos0      = PIN(pos_fb);
        ctx->r.armed     = 1;
        ctx->r.sign      = 0;
        ctx->r.n_cross   = 0;
        ctx->r.n_dwell   = 0;
        ctx->r.n_half    = 0;
        ctx->r.n_ln      = 0;
        ctx->r.rep       = 0;
        ctx->r.t         = 0.0;
        ctx->r.t_last    = 0.0;
        ctx->r.env       = 0.0;
        ctx->r.pk        = 0.0;
        ctx->r.pk_prev   = 0.0;
        ctx->r.spent     = 0;
        ctx->r.a_first   = 0.0;
        ctx->r.a_last    = 0.0;
        ctx->r.sum_half  = 0.0;
        ctx->r.sum_ln    = 0.0;
        PIN(pos)       = ctx->r.pos0;
        PIN(vel)       = 0.0;
        PIN(acc)       = 0.0;
        PIN(target)    = ctx->r.pos0 + ring_pos;
        PIN(f_ring)    = 0.0;
        PIN(zeta_ring) = 0.0;
        PIN(j_lpf)     = 0.0;
        PIN(ring_ok)   = 0.0;
        PIN(ring_n)    = 0.0;
        PIN(ring_amp)  = 0.0;
      }

      // trajectory, the same generator the gain search uses
      PIN(pos) += PIN(vel) * period + PIN(acc) * period * period / 2.0;
      PIN(vel) += PIN(acc) * period;
      float r_to_go = PIN(target) - PIN(pos);
      float r_ttg   = sqrtf(2.0 * ABS(r_to_go) / ring_acc);
      float r_acc   = ring_acc * SIGN(r_to_go);
      float r_vel   = r_acc * r_ttg;
      r_vel         = LIMIT(r_vel, r_vmax);
      r_acc         = (r_vel - PIN(vel)) / period;

      if(r_ttg < period) {
        r_vel    = 0.0;
        r_acc    = 0.0;
        PIN(pos) = PIN(target);
        PIN(vel) = 0.0;
      }

      PIN(acc)     = LIMIT(r_acc, ring_acc);
      PIN(pos_cmd) = mod(PIN(pos));
      PIN(vel_cmd) = r_vel * PIN(ff);
      PIN(acc_cmd) = PIN(acc) * PIN(ff);

      // The high pass runs the whole time so it is settled by the time a dwell
      // starts; only the trigger is gated. What is left of vel_error once the
      // loop's own bandwidth is filtered out is the mechanical mode.
      float hp = LP_HZ(MAX(PIN(ring_hp_hz), 0.5));
      ctx->r.lp  = PIN(vel_error) * hp + ctx->r.lp * (1.0 - hp);
      float v  = PIN(vel_error) - ctx->r.lp;

      PIN(ring_sig) = v;

      ctx->r.t += period;
      float dt = ctx->r.t - move_t;

      if(dt < 0.0) {
        // still moving: the move's own transient is not the mode
        ctx->r.sign    = 0;
        ctx->r.spent   = 0;
        ctx->r.n_cross = 0;
        ctx->r.n_dwell = 0;
        ctx->r.env     = 0.0;
        ctx->r.pk      = 0.0;
        ctx->r.pk_prev = 0.0;
        ctx->r.a_first = 0.0;
        ctx->r.a_last  = 0.0;
      } else {
        ctx->r.pk  = MAX(ctx->r.pk, ABS(v));
        ctx->r.env = MAX(ctx->r.env, ABS(v));

        // Schmitt trigger whose level comes from the previous half cycle's own
        // peak, so it tracks the decay without anyone having to know the rate
        // in advance. A fixed band cannot: by the time a lightly damped 145 Hz
        // mode has been observed long enough to measure its amplitude it has
        // already fallen an order of magnitude below it. The env and min_amp
        // terms are floors, to stop the band chasing the signal into the noise.
        float band = MAX(MAX(0.25 * ctx->r.pk_prev, 0.1 * ctx->r.env), MAX(PIN(ring_min_amp), 0.001));
        int cross  = 0;

        if(ctx->r.sign >= 0 && v < -band) {
          ctx->r.sign = -1;
          cross     = 1;
        } else if(ctx->r.sign <= 0 && v > band) {
          ctx->r.sign = 1;
          cross     = 1;
        }

        if(cross) {
          if(ctx->r.n_cross > 0) {
            float half = dt - ctx->r.t_last;
            float mean = ctx->r.n_half > 0 ? ctx->r.sum_half / (float)ctx->r.n_half : 0.0;

            // Once the ring has decayed past the gate, latch the count off for
            // the rest of this dwell. Letting it back on lets the noise floor
            // keep triggering, and noise produces far more edges than the mode
            // does -- that is what poisons the mean.
            if(ctx->r.n_dwell > 0 && ctx->r.pk_prev < 0.15 * ctx->r.env) {
              ctx->r.spent = 1;
            }

            // Three filters, in order of how much they matter: the half period
            // has to be resolvable at this rate and to have survived the high
            // pass; it has to agree with the ones already accepted; and the
            // cycle that produced it has to be a real one. The first four seed
            // the mean, so they are held to the top of the ring where nothing
            // else is big enough to trigger.
            float gate = (ctx->r.n_half < 4 ? 0.5 : 0.15) * ctx->r.env;

            if(!ctx->r.spent && half > 0.5 / f_max && half < 0.5 / f_min && ctx->r.pk_prev >= gate &&
               (ctx->r.n_half < 4 || (half > 0.75 * mean && half < 1.35 * mean))) {
              ctx->r.sum_half += half;
              ctx->r.n_half++;
              ctx->r.n_dwell++;
              ctx->r.a_last = ctx->r.pk;
              if(ctx->r.n_dwell == 1) {
                ctx->r.a_first = ctx->r.pk;
              }
            }
          }
          ctx->r.t_last  = dt;
          ctx->r.n_cross++;
          ctx->r.pk_prev = ctx->r.pk;
          ctx->r.pk      = 0.0;
        }

        PIN(ring_amp) = ctx->r.env;
      }

      if(dt >= dwell) {
        // log decrement for this dwell: the envelope falls as exp(-zeta*w*t),
        // so ln(first / last) across n half periods is pi * n * zeta
        if(ctx->r.n_dwell >= 3 && ctx->r.a_last > 0.0 && ctx->r.a_first > ctx->r.a_last) {
          ctx->r.sum_ln += logf(ctx->r.a_first / ctx->r.a_last) / (M_PI * (float)(ctx->r.n_dwell - 1));
          ctx->r.n_ln++;
        }

        ctx->r.rep++;
        ctx->r.t       = 0.0;
        ctx->r.t_last  = 0.0;
        ctx->r.sign    = 0;
        ctx->r.spent   = 0;
        ctx->r.pk_prev = 0.0;
        ctx->r.n_cross = 0;
        ctx->r.n_dwell = 0;
        ctx->r.env     = 0.0;
        ctx->r.pk      = 0.0;
        ctx->r.a_first = 0.0;
        ctx->r.a_last  = 0.0;
        // alternate, so the axis rocks about where it started instead of
        // walking away one kick at a time
        PIN(target) = ctx->r.pos0 + ((ctx->r.rep & 1) ? -ring_pos : ring_pos);

        if((float)ctx->r.rep >= MAX(PIN(ring_reps), 1.0)) {
          PIN(acc_cmd) = 0.0;
          PIN(vel_cmd) = 0.0;
          PIN(ring_n)  = (float)ctx->r.n_half;

          if(ctx->r.n_half >= 6 && ctx->r.sum_half > 0.0) {
            PIN(f_ring) = 0.5 * (float)ctx->r.n_half / ctx->r.sum_half;
          }
          if(ctx->r.n_ln > 0) {
            PIN(zeta_ring) = ctx->r.sum_ln / (float)ctx->r.n_ln;
          }

          if(PIN(f_ring) < f_min || PIN(f_ring) > f_max) {
            // outside what this rate and this high pass can resolve: nothing
            // was measured, and half an answer here is worse than none
            PIN(f_ring) = 0.0;
          } else if(PIN(j_mot) > 0.0 && PIN(j_sys) > 0.0) {
            PIN(j_lpf)   = PIN(f_ring) / sqrtf(1.0 + PIN(j_sys) / PIN(j_mot));
            PIN(ring_ok) = 1.0;
          }

          PIN(state) = 2.3;
        }
      }
      break;
    }

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
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct ids_ctx_t),
    .pin_count = sizeof(struct ids_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};