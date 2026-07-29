#!/usr/bin/env python3
"""
control_model.py - Python golden model for the closed-loop position-control
demo: the same fixed-point PID and the same behavioral plant as the real
hardware, run against the same profile.

This is NOT a replacement for the RTL/firmware simulation (sim/tb_soc.v) -
it's an independent, fast (<1s) re-derivation of the same numbers, used to
(a) pick and sanity-check gains/plant constants before committing them to
hand-assembled RTL-adjacent firmware, and (b) give a second, RTL-independent
answer to compare tb_soc.v's [CTRL] output against. See docs/control.md for
the full comparison procedure and gain-tuning notes.

Every constant here is mirrored exactly in:
  - fw/gen_firmware.py  (PID: SHIFT/KP/KI/KD/CENTER_DUTY/DUTY_MIN/DUTY_MAX;
                         profile: PROFILE/SEG_TICKS/TELEMETRY_PERIOD)
  - fw/main.c           (same)
  - sim/tb_soc.v         (plant: PLANT_SHIFT/VEL_GAIN_SHIFT/POS_FRAC_ONE/
                         CENTER_DUTY; profile: profile_targets/SEG_TICKS)
If you change a constant, change it in all four places.
"""

# ---- PID (Q8 fixed point) ---------------------------------------------------
SHIFT       = 8
KP          = 800
KI          = 1
KD          = 800
CENTER_DUTY = 1500
DUTY_MIN    = 1000
DUTY_MAX    = 2000

# ---- plant: duty -> velocity (first-order lag) -> position (integrated) ----
PLANT_SHIFT    = 4    # lag time-constant shift
VEL_GAIN_SHIFT = 7    # duty error (+-500 max) -> velocity_target scale
POS_FRAC_ONE   = 256  # fixed-point: 256 fractional units = 1 count

# ---- profile -----------------------------------------------------------
PROFILE   = [0, 400, -200]
SEG_TICKS = 400
SETTLE_TOLERANCE = 8
SETTLE_BUDGET    = 300
OVERSHOOT_LIMIT_PCT = 25.0


def sra(x, n):
    """Arithmetic right shift matching RV32I's srai / Verilog's signed >>>:
    floor division by 2^n (rounds toward -infinity, not toward 0)."""
    return x >> n if x >= 0 else -((-x - 1) >> n) - 1


def pid_step(state, target, position):
    """One ISR tick's worth of PID. state is a dict with 'integral' and
    'prev_error', mutated in place - matches the persistent-register/memory
    state gen_firmware.py's ISR keeps across calls."""
    error = target - position
    integral_tentative = state['integral'] + error
    p_term = KP * error
    i_term = KI * integral_tentative
    d_term = KD * (error - state['prev_error'])
    correction = sra(p_term + i_term + d_term, SHIFT)
    duty_unclamped = CENTER_DUTY + correction

    # conditional-integration anti-windup: only commit the integral update
    # if not already saturated in the direction error is pushing
    skip = (duty_unclamped > DUTY_MAX and error > 0) or \
           (duty_unclamped < DUTY_MIN and error < 0)
    if not skip:
        state['integral'] = integral_tentative
    state['prev_error'] = error

    duty = max(DUTY_MIN, min(DUTY_MAX, duty_unclamped))
    return duty, error


def plant_step(pstate, duty):
    """One ISR tick's worth of plant. pstate is a dict with 'vel',
    'pos_frac', 'position', mutated in place."""
    duty_err = duty - CENTER_DUTY
    vel_target = sra(duty_err * POS_FRAC_ONE, VEL_GAIN_SHIFT)
    pstate['vel'] += sra(vel_target - pstate['vel'], PLANT_SHIFT)
    pstate['pos_frac'] += pstate['vel']
    while pstate['pos_frac'] >= POS_FRAC_ONE:
        pstate['pos_frac'] -= POS_FRAC_ONE
        pstate['position'] += 1
    while pstate['pos_frac'] <= -POS_FRAC_ONE:
        pstate['pos_frac'] += POS_FRAC_ONE
        pstate['position'] -= 1
    return pstate['position']


def true_settle_tick(positions, target, tolerance=SETTLE_TOLERANCE):
    """Earliest index from which every later sample (through the end of the
    list) stays within `tolerance` of target - same backward-scan definition
    tb_soc.v's closed_loop_mon uses. Returns len(positions) if it never
    settles for good within the window."""
    settle = len(positions)
    for k in range(len(positions) - 1, -1, -1):
        if abs(positions[k] - target) <= tolerance:
            settle = k
        else:
            break
    return settle


def run_profile(profile=PROFILE, seg_ticks=SEG_TICKS, verbose=False):
    """Runs the full profile, returns a list of per-segment result dicts:
    {seg, target, settle_tick, overshoot_pct, final_pos, positions}."""
    pid_state = {'integral': 0, 'prev_error': 0}
    plant_state = {'vel': 0, 'pos_frac': 0, 'position': 0}
    results = []
    for seg, target in enumerate(profile):
        seg_start_pos = plant_state['position']
        step_size = target - seg_start_pos
        positions = []
        for _ in range(seg_ticks):
            duty, _ = pid_step(pid_state, target, plant_state['position'])
            pos = plant_step(plant_state, duty)
            positions.append(pos)

        settle_tick = true_settle_tick(positions, target)

        if step_size > 0:
            max_pos = max(positions)
            overshoot_pct = max(0.0, (max_pos - target)) / step_size * 100.0
        elif step_size < 0:
            min_pos = min(positions)
            overshoot_pct = max(0.0, (target - min_pos)) / (-step_size) * 100.0
        else:
            overshoot_pct = 0.0

        result = dict(seg=seg, target=target, settle_tick=settle_tick,
                      overshoot_pct=overshoot_pct, final_pos=positions[-1],
                      positions=positions)
        results.append(result)
        if verbose:
            print(f"[CTRL] segment {seg}: target={target} settle_tick={settle_tick} "
                  f"overshoot={overshoot_pct:.1f}% final_pos={result['final_pos']}")
    return results


def check(results):
    """Applies the same pass/fail gate as tb_soc.v's RESULT: PASS/FAIL.
    Returns (ok, list-of-failure-strings)."""
    failures = []
    for r in results:
        if r['settle_tick'] > SETTLE_BUDGET:
            failures.append(f"segment {r['seg']}: settle_tick={r['settle_tick']} "
                             f"exceeds {SETTLE_BUDGET}-tick budget")
        if r['overshoot_pct'] >= OVERSHOOT_LIMIT_PCT:
            failures.append(f"segment {r['seg']}: overshoot={r['overshoot_pct']:.1f}% "
                             f">= {OVERSHOOT_LIMIT_PCT}%")
    return (len(failures) == 0), failures


if __name__ == "__main__":
    results = run_profile(verbose=True)
    ok, failures = check(results)
    if ok:
        print("RESULT: PASS")
    else:
        for f in failures:
            print(f"[CTRL] MISMATCH: {f}")
        print("RESULT: FAIL")
