# Closed-loop position control (phase A: autonomous profile)

An encoder -> PID -> PWM position-control loop running entirely in the
timer ISR (`docs/timer.md`), driving a step profile (target 0 -> +400 ->
-200 counts) with no host/CPU-external involvement. This is phase A of the
roadmap's closed-loop demo: autonomous only, no new peripherals - the
"plant" (motor + encoder) is a behavioral model in `sim/tb_soc.v` for
verification; there is no real actuator in this phase.

RTL/firmware involved: `rtl/timer.v` (ISR_CYCLES), `rtl/soc_top.v`
(ENABLE_MUL/ENABLE_FAST_MUL), `fw/gen_firmware.py` and `fw/main.c` (the PID),
`sim/tb_soc.v` (the plant + pass/fail assertions), `fw/control_model.py`
(the Python cross-check).

## PID design

Fixed-point, Q8 (`SHIFT = 8`, i.e. every gain is stored as `gain * 256`):

    error      = target - position                (position = ENC.COUNT)
    integral  += error                             (conditional - see below)
    derivative = error - prev_error
    correction = (KP*error + KI*integral + KD*derivative) >>> SHIFT
    duty       = clamp(CENTER_DUTY + correction, DUTY_MIN, DUTY_MAX)

`correction` uses an *arithmetic* right shift (`srai` in the hand-assembled
ISR, plain `>>` on a signed `int32_t` in `main.c` - arithmetic on every
compiler/platform that matters here, though technically implementation-
defined in C - and `sra()` in `fw/control_model.py`, which floors toward
-infinity to match). A logical shift here would corrupt the sign of a
negative correction.

Gains: `KP=800, KI=1, KD=800`, `CENTER_DUTY=1500`, `DUTY_MIN=1000`,
`DUTY_MAX=2000` (the same 1000-2000 servo-pulse range PWM already used).

### Anti-windup: conditional integration

The tentative integral update (`integral + error`) is only committed if the
*unclamped* duty isn't already saturated in the direction `error` is
pushing:

    skip = (duty_unclamped > DUTY_MAX && error > 0) ||
           (duty_unclamped < DUTY_MIN && error < 0)

Otherwise the integrator would keep accumulating while the actuator command
is already pinned at its limit, building up a large value that then has to
"unwind" once the error reverses sign - classic integrator windup. Verified
in simulation: during the +400 step, `duty` sits at the `DUTY_MAX` (2000)
clamp for the first ~11 ticks while the integrator visibly holds at 0 (not
accumulating) until the duty comes off the clamp - see the gain-tuning
notes below for the actual traced numbers.

## Why PERIOD is live but DUTY is double-buffered (and why that's safe here)

This is a deliberate asymmetry, not an oversight. PWM's `DUTY` is
double-buffered (`docs/pwm.md`) because it feeds an external actuator (a
servo) through a physical wire - a value change landing mid-pulse produces
a torn, physically visible glitch that software can't undo after the fact.
The timer's `PERIOD` is a plain, immediately-effective register
(`docs/timer.md`) because its only consumer is our own ISR, and the `>=`
tick compare (not `==`) is exactly what makes that safe: even the worst-case
reconfiguration (shrinking PERIOD out from under a running count) still
produces a single, bounded, on-time-or-early tick, never a silently skipped
one. There's no physical output to glitch - only a control-loop tick that
either fires correctly or fires very slightly early, and "fires early" is
never a hazard the way "silently misses a step" would be for the actuator
path.

## `ISR_CYCLES`: measure, don't guess

`rtl/soc_top.v` sets `ENABLE_MUL(1)` *and* `ENABLE_FAST_MUL(1)`. Enabling
`ENABLE_MUL` alone selects picorv32's vendored bit-serial multiplier
(`picorv32_pcpi_mul`, ~32+ cycle latency) - checked directly in
`rtl/picorv32.v` before relying on it. `ENABLE_FAST_MUL` selects
`picorv32_pcpi_fast_mul`, a combinational `rs1 * rs2` behind a couple of
pipeline stages (~2-cycle latency from issue to `pcpi_ready`) - genuinely
single-cycle-class, appropriate for per-tick ISR math. Both flags are set:
`ENABLE_FAST_MUL` wins in `picorv32.v`'s generate-block priority regardless
of `ENABLE_MUL`, but `ENABLE_MUL` also gates other MUL-related plumbing, so
both are needed together.

Rather than assume the resulting ISR is "fast enough," `timer.v` exposes
`ISR_CYCLES` (offset `0x10`): a free-running cycle counter, read-only,
incrementing every clock unconditionally. Both firmwares sample it at ISR
entry and exit and report the delta as part of telemetry (`C=` field, hex
cycles). Measured in the `sim/tb_soc.v` run this design was verified
against: **168-184 cycles per ISR call** (varies with which anti-windup/
clamp branch is taken), against a 2000-cycle tick period - under 10%
utilization, with three `mul` instructions accounting for a small fraction
of that (~2 cycles each). This is real measured data from the actual
simulation, not an assumed budget.

## Profile and telemetry

Target sequence: `0 -> +400 -> -200` counts, one step change every
`SEG_TICKS = 400` ISR ticks. Each 2000-clk tick is this codebase's
established "1kHz-style" sim-accelerated convention (`docs/timer.md`): 1
ISR tick is treated as 1ms of conceptual control-loop time, matching the
real-hardware `main.c` setting (`TIMER_PERIOD=50000` at 50MHz) proportionally
- so "settle within 300ms" is checked as "settle within 300 ISR ticks" in
`sim/tb_soc.v`, not 300ms of simulated wall-clock time.

Telemetry prints roughly every 100 ticks over UART, hex-only (no `/` or
`%`- neither firmware has hardware division; `ENABLE_DIV` stays 0):

    P=xxxxxxxx T=xxxxxxxx D=xxxxxxxx C=xxxxxxxx\n

`P`=position (ENC.COUNT, raw 32-bit two's complement), `T`=target,
`D`=commanded PWM duty, `C`=last ISR's measured duration in cycles.

## System assertions (`sim/tb_soc.v`)

A behavioral velocity plant stands in for the real motor+encoder: reads the
PID's commanded duty (`dut.u_pwm.duty_shadow` - the value the PID just
wrote, not gated behind PWM's own frame-boundary double-buffering, since
that buffering protects the physical pin, not the plant's notion of
commanded intent), maps it to a target velocity proportional about center,
runs it through a first-order lag (simulated inertia), integrates the
result into a fixed-point position, and - whenever that position crosses an
integer boundary - drives one real Gray-code quadrature transition on
`enc_a`/`enc_b`, exactly like a real encoder on a real motor shaft:

    duty_err     = duty - CENTER_DUTY
    vel_target   = (duty_err * POS_FRAC_ONE) >>> VEL_GAIN_SHIFT
    vel         += (vel_target - vel) >>> PLANT_SHIFT
    pos_frac    += vel
    // pos_frac crossing +-POS_FRAC_ONE increments/decrements position
    // and emits one quadrature transition

`PLANT_SHIFT=4` (lag time constant), `VEL_GAIN_SHIFT=7` (duty-error-to-
velocity scale), `POS_FRAC_ONE=256` (fixed-point: 256 units = 1 count).

Pass criteria, gating `RESULT: PASS`/`FAIL`: each profile step settles to
within **+-8 counts** of target within **300 ISR ticks**, with **overshoot
under 25%** of the step size. "Settled" means the *true* settle point - the
earliest tick from which every later sample (through the end of the
400-tick segment) stays in tolerance, not just the first tick that happens
to touch the band in passing (a naive first-touch check can trigger while
the system is still swinging through the target on its way to overshoot).

## Cross-check procedure (`fw/control_model.py`)

`fw/control_model.py` re-implements the exact same PID and plant arithmetic
(same constants, same profile, same settle/overshoot definitions) in plain
Python, as an independent, sub-second re-derivation of the same numbers -
run it directly (`python3 fw/control_model.py`) any time a gain or plant
constant changes, before touching the (much slower) RTL simulation.

It is not a substitute for `sim/tb_soc.v` - it doesn't run the hand-
assembled ISR or the RTL at all, so it can't catch a hand-assembly bug (an
encoding mistake, a clobbered register) the way running the real firmware
does. Its job is: (1) fast gain-tuning iteration, and (2) an independent
check that the RTL run's numbers are in the right ballpark.

Comparing the two (from the run this design was verified against):

| | settle (seg 1, target 400) | overshoot | settle (seg 2, target -200) | overshoot |
|---|---|---|---|---|
| `sim/tb_soc.v` (RTL+firmware) | 143 | 1.5% | 195 | 0.8% |
| `fw/control_model.py` (Python) | 141 | 1.5% | 193 | 0.8% |

The small (~2-tick, ~1-count) differences are expected, not a bug in either
model: the Python model computes PID and plant in perfect lockstep within
one iteration (fresh duty applied to the plant immediately), while in the
real hardware the plant (in `tb_soc.v`) samples `duty_shadow` at the exact
instant `irq_pulse` fires - which is *before* that tick's ISR has run and
written a fresh duty - so the plant is always responding to the *previous*
tick's command, a consistent one-tick lag the Python model doesn't
represent. Agreement within a few ticks/counts is the expected signature of
a faithful implementation; exact match is not the bar (and would actually
suggest the two models were accidentally doing the same simplification).

## Gain-tuning notes

Starting point and process, for whoever retunes this next:

1. **Characterize the plant open-loop first.** Drive it at max duty (a
   constant 2000) and see how fast position moves. The first attempt at
   `VEL_GAIN_SHIFT` was too aggressive (shift=6 gave ~8 counts/tick at full
   duty - reaching a 400-count step in ~50 ticks even with a naive
   controller, well past any interesting closed-loop dynamics) and then
   overcorrected (shift=12 made the plant essentially not move at all - the
   per-tick lag increment `(vel_target - vel) >>> PLANT_SHIFT` rounded to
   zero because `vel_target` itself was too small relative to `2^PLANT_SHIFT`,
   a real integer-truncation stall worth watching for). `VEL_GAIN_SHIFT=7`
   (max ~3.8-7.8 counts/tick depending on `PLANT_SHIFT`) was the first
   value that let a 400-count step plausibly settle inside a few hundred
   ticks.
2. **Tune P(D) with KI=0 first.** Integral action isn't needed for zero
   steady-state error here (the plant's velocity naturally returns to 0
   when duty returns to `CENTER_DUTY`, i.e. it already has its own
   integrator - position accumulates velocity). Sweeping KP alone (with a
   fixed KD) showed monotonic, non-oscillating convergence for a wide gain
   range, converging *faster* as KP increased (opposite of the naive
   expectation, because the initial low-gain sweeps were so weak the system
   barely moved in the whole window) - `KP=800` was the first value settling
   comfortably inside 300 ticks with room to spare (~150-200 tick actual
   settle, roughly half the budget).
3. **Add a small KI last, only to exercise anti-windup.** `KI=0` already
   satisfied the settle/overshoot bar; `KI=1` was added specifically so the
   anti-windup logic has something real to do (verified: the integrator
   visibly holds at 0 while duty is saturated, then starts accumulating the
   instant duty comes off the clamp - see the anti-windup section above).
   Larger `KI` values (tested up to 16) degrade settle time and overshoot
   monotonically - this plant doesn't need much integral action, and it's
   easy to give it too much.
4. **Re-run `fw/control_model.py` after every gain change**, then confirm
   with `sim/tb_soc.v` before considering it done - the Python model is
   fast enough to sweep gains in a loop; the RTL sim is the ground truth.
