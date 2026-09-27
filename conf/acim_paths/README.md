# ACIM paths on the α2 spindle: test plan

Configs in this folder exercise every ACIM control path stmbl has, on the Fanuc
α2 spindle (A06B-0852-B190#3000, delta, 4 poles, pp 2, 19 A rms / 27 A rms S2,
base 78.4 Hz, 293 V link). They sit on the idacim branch
(`claude/project-thread-5aj2ef`, b97df08), so T1 uses the new leakage and
rotor tests. All stmbl currents are **peak** and all volt-mode voltages are
**phase-peak, equivalent Y**.

Nothing here has run on hardware. Every pin was checked against the built
component sources with `template-crosscheck/check_templates.py` (no unknown
pins in these files; the templates' own known faults such as
`pid0.min_torque` still print "not found" at load, harmless).

## Common to every config

| Setting | Value | Why |
|---|---|---|
| `conf0.max_ac_cur` | 13.8 (T5: 8) | bridge rule: no IPM above ~13 A until P-N is scoped and the comparators checked. Raise only on Eduard's word |
| `conf0.max_vel` | 100–400 rad/s per test | spindle max 838; raise in steps |
| `conf0.r` | 0.18 | DC reading (live config had 0.615, the old dead-time bias). Halves curpid's integral gain vs live; revert to 0.615 if the current loop is sluggish |
| `conf0.l` | 3.539 mH (unchanged) | σLs 0.84 mH waits on the curpid flux term |
| `conf0.psi` | 0 | ACIM: no PMSM back-EMF feedforward |
| `acim_ttc0.cur_n` / `torque_n` | 16 / 9.9 | id_n = 11.3 A fits under 13.8; torque_n scaled by (16/19)² |
| `pid0.neg_min_torque` | `acim_ttc0.t_max` | acim.txt links a nonexistent `pid0.min_torque` |
| `hv0.vel` | `acim_ttc0.vel_e` | F3 decoupling wants synchronous speed, pid.txt gives rotor speed |
| sserial | not linked | bench only. **Enable: `ramp0.en = 1`, stop: `ramp0.en = 0`. Speed: `ramp0.vel_ext_cmd = <rad/s mech>`** |

Every test:

- Scope P-N at the IPM and one phase current probe on the **first enable** of
  each config (open item from the spindle bridge failure).
- Abort (`ramp0.en = 0`, then contactor) on any fault, `hv0.abs_cur` sitting
  at the limit for more than a second, `hv0.dc_volt` above ~350 V, or noise.
- Record: the servoterm waves (each config sets term0 for its test), and
  `show` of the result pins named below. Save as `acim_paths/<test>_<date>.txt`
  in project files.

## Order

T1 → (T2 if T1's tr looks odd) → put r and τr into the config → T3 → T8 → T4
→ T6 → T7 → T5. Stop after any test that fails and send the record.

## T1 `t1_idacim.txt`: identification

Standstill tests, then the pp step, which turns the spindle open loop (belt on
is fine, keep hands and tools clear).

1. Load, `ramp0.en = 1`. idacim runs through its states and prints results.
2. Read: `drop` (with `r_known` 0.18 it reports the bridge drop), `l`,
   `l_ok`, `tr`, `slip_n`, `lmr`, `ls`, `tr_ok`, `tr_spread`, `rot_dip`, `pp`.
3. **Pass:** `l` 0.7–1.1 mH (LCR 0.84), `tr` inside 40–170 ms with
   `tr_spread` < 10%, `lmr` 14–20 mH, `pp` 2.
4. **Do not append its `conf0.l` line** until the curpid flux term exists.
5. Put τr into acim_ttc through `vel_n` (slip_n is derived, review item 4):
   `acim_ttc0.vel_n = (492.6 - 1/tr) / 2`. Examples: tr 49 ms → 236.1; 60 ms →
   238.0 (today's value); 147 ms → 242.9.

test_cur is 10 so injection ripple stays under the 13.8 A limit.

## T2 `t2_dstep.txt`: manual standstill d step (cross-check)

Config-only version of idacim case 14 (acim-identification.md §4.1).

1. `ramp0.en = 1`. d current squares between 6 and 12 A at 1 Hz, rotor at rest.
2. Scope `hv0.ud_fb` and `hv0.id_fb` for 10+ cycles.
3. Fit the tail after each edge: `ud = a + b·e^(−t/τr)`; area/Δi = Lmr.
4. **Pass:** within 10% of T1. Expected tail peak 0.7–2 V, area ~0.1 V·s.

## T3 `t3_slip_low.txt`: acim_ttc mode 0 with encoder, below base

The production path, no field weakening (`duty` unlinked on purpose here).

1. With T1's `vel_n`: speeds 20, 50, 100, 200 rad/s, both directions.
2. At each: steady current, `hv0.iq_fb` near 0 at constant speed (friction
   only), `hv0.id_fb` ≈ 11.3 A.
3. Accelerate/decelerate between 50 and 200 with `conf0.max_acc` 100 and 300.
   Watch `ud_fb` during the ramps (acim-identification §4.2): with correct
   slip, `ud` changes little between constant speed and ramp. Try `vel_n`
   ±1.5 rad/s around the T1 value and keep the one with the least `ud` step.
4. **Pass:** speed follows the ramp, no oscillation, current never at limit
   except during the 300 rad/s² ramp.

## T8 `t8_emf_check.txt`: slip check with the observer as a meter

`sensorless0` runs frozen (`ki = 0`, velocity tied to `acim_ttc0.vel_e`), so it
does not steer anything; its `ed`/`eq` are the back-EMF in the drive's own
d/q frame. In a correctly oriented frame `ed ≈ 0` and `eq ≈ ωe·Lmr·i_mr`.

1. Same speeds and ramps as T3.
2. Record `ed`, `eq` at constant speed and during ramps. `ed/eq` is the
   orientation error in radians.
3. Sweep `vel_n` as in T3; the right value keeps `ed` flat across the ramp.
4. **Pass:** `|ed/eq|` < 0.05 at 100 and 200 rad/s under ramp torque.
   Below ~20 rad/s the reading is dominated by the dead time and Rs, ignore it.

This is also the first real-hardware check of the observer's EMF math, needed
before any closed-loop sensorless work.

## T4 `t4_slip_fw.txt`: mode 0 with field weakening

`acim_ttc0.duty = hv0.duty`, setpoint 0.8.

1. `conf0.max_vel` 400 first. Ramp 200 → 400 at 100 rad/s².
2. Watch `hv0.duty` (should hold ~0.8), `acim_ttc0.scale` (falls from 1 above
   the corner), `acim_ttc0.t_max`, `hv0.id_fb`.
3. Then 600, then 838 (only after 400 is clean, and on Eduard's word).
4. **Pass:** duty regulated, no current runaway, speed reached, decel clean
   (watch `dc_volt`, the PSM should clamp).

## T6 `t6_mode2_uf_enc.txt`: acim_ttc mode 2 (V/f with encoder)

Volt mode: no dead-time compensation (by design) and curpid's volt-mode
current limit is close to bang-bang at this current (review item 19). Keep
ramps gentle.

1. `u_n` starts at 90 V phase-peak at 78.4 Hz, about 10 A magnetizing.
2. Hold 100 rad/s: raise `acim_ttc0.u_n` in steps of 5 until no-load
   `hv0.abs_cur` is ~11 A. **Stop** if current rises steeply per step
   (saturation).
3. Speeds 20, 50, 100, 200; try `u_boost` 5, 9, 12 at 20 rad/s.
4. **Pass:** speed held by the pid, no limit chatter at constant speed.

## T7 `t7_uf_open.txt`: open-loop V/f (uf template)

The only fully sensorless path today. Encoder read only (`vel0.vel`) to see
the slip.

1. `u_n` 90 as T6, tune the same way. `max_acc` 30.
2. Speeds 20, 50, 100, 200 rad/s forward, then reverse.
3. Record `ramp0.vel_cmd − vel0.vel` (slip) and `abs_cur`.
4. If 20 rad/s stalls: `hv0.q_cmd` 3, then 6 (fixed boost volts).
5. Slip comp stays off (`uf0.slip_n = 0`): it is unsigned (review item 14).
   Optional forward-only check with `uf0.slip_n = 0.03`: slip should shrink.
6. **Pass:** speed within slip of command, no trip on accel/decel at 30 rad/s².

## T5 `t5_mode1.txt`: mode 1 (constant slip)

For completeness. d = 0 and fixed slip, so torque goes with I², not I (review
item 6). Limit 8 A.

1. 20, 50, 100 rad/s. Expect weak, nonlinear speed control.
2. **Pass:** runs without trip. Not a mode to use.

## Not testable yet (need code first)

- `acim_ttc0.sensorless = 1`: vel_m scaled by poles instead of divided, pos
  integrates rotor speed, writes into the linked `vel1.vel` (review items 1–3).
- Closed-loop observer (`hv0.pos = sensorless0.pos`): no proportional term,
  undamped at no load (item 10); needs an I/f start (item 12).
- High-speed 2-pole V/f: needs F3 angle extrapolation every PWM cycle.

Review: `reviews/acim-sensorless-review-2026-09-27.md` in project files.
