# Host simulation harness

Runs the real F4 and F3 firmware (hal.c, shared comps, conf templates, board configs) on a PC against
a PMSM + two-mass load model. Scripted scenarios, CSV traces, pass/fail exit code.

## Use

```
make -C tests/sim            # build build/stmbl_sim
make -C tests/sim test       # run all scenarios, CSV/log in build/out/
tests/sim/build/stmbl_sim [-v] [-o out.csv] [-d decim] [-e "line"] scenario.txt
tests/sim/build/stmbl_sim -p # list plant parameters
python3 tests/sim/plot.py build/out/y_move.csv
```

## Scenario language

One command per line, `#` comments, optional `at T` prefix (seconds):

- `config FILE`: board config fed through hal_parse (path relative to scenario dir, walking upward)
- `f4 LINE` / `f3 LINE`: hal command on that side (`f4 en0.en = 1`, `f4 hv_pause 5`)
- `plant k=v ...`, `link k=v ...` (latency, jitter, ...)
- `lcnc enable|scale|latency|jitter|period|move DIST VEL ACC|jog VEL ACC|dwell T`: 1 kHz sserial source
- `trace SIG ...`, `sample f3`, `f3_phase`, `strict` (parse errors fail), `title`, `run T`
- `expect STAT SIG OP VAL [from] [to]`, STAT = max min absmax mean rms std p2p first final
- Signals: `comp0.pin` (F4), `f3:comp0.pin`, `plant.x`, `lcnc.x`

Plant defaults come from the config (conf0.r, l, lq, psi, polecount, j, j_sys, f, d). Two-mass via
`k`/`c` or `ring_hz`/`ring_zeta`; Karnopp friction; `enc_delay` defaults to hv0.adv minus the
packet-to-F3 time.

## Limits

No rt CPU time, no ISR races, no recon/PLL/fault pin, averaged PWM (dead time as tanh voltage loss),
real hardware drivers replaced by pin-compatible stand-ins (comps_f4/hv.c, comps_f3/ls.c, io.c,
generated shells). The coupling ring is a plant input, not validated against bench captures.
