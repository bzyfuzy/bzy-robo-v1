# CLAUDE.md — robot-soc

Minimal robot SoC: PicoRV32 (RV32I) + peripherals, building toward a
perception (NPU) + real-time control SoC for a pan/tilt robot. Currently in
the peripheral-building + verification-buildout phase.

## Architecture

- CPU: `rtl/picorv32.v` (vendored, do not modify). Native memory bus,
  reset vector 0x0, stack top 0x2000. IRQ enabled (`ENABLE_IRQ(1)`),
  `PROGADDR_IRQ = 0x1000` (fixed firmware-layout convention — see
  `docs/timer.md`). PicoRV32's IRQ interface is non-standard custom-0
  opcodes (getq/setq/retirq/maskirq/waitirq), not CSRs — hand-encoded in
  both `fw/gen_firmware.py` and `fw/start.S`.
- `ENABLE_MUL(1)` + `ENABLE_FAST_MUL(1)`: real hardware multiply for
  deterministic PID math in the timer ISR (`docs/control.md`).
  `ENABLE_MUL` alone selects the vendored bit-serial multiplier (~32+
  cycles); `ENABLE_FAST_MUL` is the actual single-cycle-class one
  (measured ~2-cycle pcpi latency) and wins in `picorv32.v`'s generate
  priority regardless of `ENABLE_MUL` — both are set because `ENABLE_MUL`
  also gates other MUL-related plumbing. `ENABLE_DIV` stays 0; firmware
  that needs the CPU's multiply/no-divide split (`fw/main.c`,
  `fw/gen_firmware.py`) never uses `/` or `%`.
- SoC top: `rtl/soc_top.v` — address decode on `mem_addr[31:24]`,
  registered one-wait-state ready for every access:
  `mem_ready <= mem_valid && !mem_ready;`
  Writes are honored only on the ready cycle (`wstrb_eff`).

## Directory layout

Current reality: flat `rtl/`, `sim/`, `fw/`, `docs/`. Target layout below —
each directory is created only when its first real file lands ("create
when"), and any move is its own commit with zero functional changes.

    rtl/
      core/           # create when: 2nd core-side file (now: picorv32.v stays put)
      bus/            # create when: interconnect/mailbox leaves soc_top.v
      peripherals/    # create at next restructure: pwm, uart_tx, quad_enc, timer
      memory/         # create when: RAM/scratchpads leave soc_top.v
      top/            # create at next restructure: soc_top.v
    tb/
      unit/           # exists: tb_quad_enc.v, tb_timer.v; add per-peripheral as written
      integration/    # create at next restructure: tb_soc.v
    fw/
      include/        # create when: >1 header (register defs split out)
      drivers/        # create when: per-peripheral C drivers split from main.c
      tests/          # create when: firmware-level test programs exist
    fpga/
      <board>/        # create when: board chosen (real name, not "board_name")
        constraints/  #   pin/timing constraints for that board
        top.v         #   board wrapper around soc_top (clocks, reset, IO)
    asic/
      constraints/    # create when: first synthesis/PnR run targets silicon
      config/
      macros/
    scripts/          # create when: 2nd helper script (CI, lint, regression)
    docs/             # exists

## Memory map

| Base        | Block              | Registers (word offsets)                         |
| ----------- | ------------------ | ------------------------------------------------ |
| 0x0000_0000 | RAM 8KB            | firmware via $readmemh, word index addr[12:2]    |
| 0x0200_0000 | PWM                | 0x0 CTRL(en), 0x4 PRESCALE, 0x8 PERIOD, 0xC DUTY |
| 0x0300_0000 | UART TX            | 0x0 DATA, 0x4 STATUS (bit0 busy)                 |
| 0x0400_0000 | GPIO               | 0x0 LEDs (8-bit)                                 |
| 0x0500_0000 | Quadrature encoder | 0x0 COUNT (ro, signed 32), 0x4 CTRL (bit0 clear) |
| 0x0600_0000 | Timer (IRQ)        | 0x0 CTRL, 0x4 PERIOD, 0x8 COUNT, 0xC STATUS, 0x10 ISR_CYCLES (ro, free-running) |
| 0x0700_0000 | (next: UART RX)    |                                                  |

Timer is the first peripheral with a 5th register (`ISR_CYCLES` @ 0x10), so
its `addr` port is 5 bits, not the usual 4 — see the Peripheral bus
convention below and `docs/timer.md`.

Firmware layout convention: reset code at 0x0, IRQ handler at 0x1000 —
`gen_firmware.py` and `sections.lds` must both respect this.

Assign new peripherals the next free 0xNN00_0000 slot; update this table,
the decoder, and the read mux in `soc_top.v` together, in the same commit.

## Peripheral bus convention

Every peripheral uses the same interface as `rtl/pwm.v` (the template):

    input  sel;            // address-decoded select from soc_top
    input  [3:0] wstrb;    // byte write strobes; 0000 = read
    input  [3:0] addr;     // byte offset within the peripheral (word-aligned:
                           // only bits [3:2] vary, so 4 bits reaches exactly
                           // 4 registers — 0x0/0x4/0x8/0xC)
    input  [31:0] wdata;
    output [31:0] rdata;   // combinational read mux inside the peripheral

A peripheral needing a 5th register widens `addr` for its own instantiation
only (`timer.v` is the first: 5 bits, reaching 0x10 — see the memory map
above). This is a per-peripheral exception, not a convention change.

Async active-low `rst_n` — fed from `rst_sync.v`'s synchronized output, not
the raw external pin (see `docs/reset.md`). External async inputs (encoder phases, future
sensor lines) pass through 2FF synchronizers before any logic. Outputs
that touch actuators must be glitch-free (see PWM's double-buffered DUTY).
Determinism is a feature: bus accesses are fixed-latency; no
variable-latency slaves on the control path. IRQ-related peripheral status
bits are write-1-to-clear from the ISR, never auto-clearing on read.

## Build, lint, test

Lint (run before every commit; zero warnings is the bar):

    verilator --lint-only -Wall verilator.vlt rtl/soc_top.v rtl/rst_sync.v \
              rtl/pwm.v rtl/uart_tx.v rtl/quad_enc.v rtl/timer.v rtl/picorv32.v

`rtl/picorv32.v` must be included (soc_top.v instantiates it). `verilator.vlt`
(repo root) waives picorv32.v's pre-existing vendored-style warnings by file
path only - it has zero effect on any file we own, so a new warning in our
own RTL still fails this command. See the header of that file for exactly
which categories are waived and why.

Full-SoC simulation (no RISC-V toolchain needed):

    cd sim
    python3 ../fw/gen_firmware.py
    iverilog -g2005-sv -o tb_soc.vvp tb_soc.v ../rtl/soc_top.v \
             ../rtl/rst_sync.v ../rtl/pwm.v ../rtl/uart_tx.v \
             ../rtl/quad_enc.v ../rtl/timer.v ../rtl/picorv32.v
    vvp tb_soc.vvp            # add +trace for tb_soc.vcd

Add every new RTL file to both command lines above (and keep README in
sync). C firmware for real hardware: `fw/Makefile` (riscv32 gcc).

Planned restructure (own commit, zero functional changes, update all paths
here when done): `rtl/peripherals/`, `tb/integration/` (tb_soc.v still lives
in `sim/`; `tb/unit/` has already started early with per-peripheral tbs),
`tests/` for CI scripts. Directories follow files, not ambitions — `fpga/`
only when a board is chosen, `asic/` only when an ASIC artifact exists.

## Rules

1. Every peripheral has (a) a unit testbench covering edge cases (for
   quad_enc: all 8 legal transitions, illegal skips, clear, signed
   overflow, bounce; for timer: period accuracy, IRQ assertion timing,
   status clear, disable mid-count) and (b) a monitor in the integration
   tb that verifies waveform-level behavior — not just register readback.
2. Integration coverage must include the program-visible path: firmware
   reads the peripheral over the bus and reports via UART/GPIO.
   Hierarchical peeks (`dut.u_enc.count`) are allowed in unit tbs, not as
   the only system-level check.
3. Every simulation ends with an explicit `RESULT: PASS` / `RESULT: FAIL`
   line computed from checks; PASS is required before commit.
4. Exercise each new peripheral in `fw/gen_firmware.py` and mirror in
   `fw/main.c`.
5. One peripheral (or one restructure, or one verification item) per
   commit.
6. Never claim a test passed without running it. If the environment cannot
   run the tools, state so explicitly, mark the item "verification
   pending" below, and hand the exact commands to the user.

## Verification status

| Item                          | RTL     | Simulated & PASS confirmed |
| ----------------------------- | ------- | -------------------------- |
| Baseline (PWM+UART+GPIO boot) | done    | yes (original baseline; `sim/tb_soc.v`'s firmware no longer touches GPIO as of the closed-loop demo below, so its LED-mirror check was retired along with it — GPIO/LEDs themselves are unchanged and untested by the current integration tb) |
| Quadrature encoder            | written | yes (unit tb `tb/unit/tb_quad_enc.v`, 24 checks; also driven live by the closed-loop plant in `sim/tb_soc.v` now — see below) |
| Timer + IRQ                   | written | yes (unit tb `tb/unit/tb_timer.v`, 55 checks, incl. the `>=` fix + `ISR_CYCLES`; integration: `sim/tb_soc.v` `period_errors=0` over 1200 ticks) |
| Closed-loop position control (phase A) | written | yes (2026-07-29, `sim/tb_soc.v`: both profile steps settle well inside the 300-tick budget with low overshoot — see below and `docs/control.md`) |

Update this table the moment a sim actually runs. "Hand-traced" is not a
verification state.

## Verification ladder (build in this order)

0. Verilator lint clean — done (2026-07-29). `rtl/picorv32.v` is now in the
   documented command; `verilator.vlt` waives its pre-existing vendored-style
   warnings (BLKSEQ, DECLFILENAME, GENUNNAMED, MULTITOP, UNUSEDSIGNAL) by
   file path only. Three real issues in our own files were fixed instead of
   waived: a dead `mem_instr` wire and dead `ram_rdata` reg in soc_top.v, and
   a dead `frame_start` wire in pwm.v. Three more got scoped, commented
   waivers for expected-not-buggy port-width mismatches inherent to the
   uniform 32-bit bus convention (soc_top.v's `mem_addr[23:13]` sparse-decode
   gap, uart_tx.v's `wstrb[3:1]`/`wdata[31:8]`, quad_enc.v's `wdata[31:1]`).
   One cross-boundary SYNCASYNCNET (`rst_n` async in soc_top.v/peripherals,
   sync inside picorv32.v) was initially waived after a first attempt at
   fixing it (converting soc_top.v's own regs to sync reset) just relocated
   the same warning to the peripherals. Root-caused and actually fixed
   (2026-07-29, no waiver anymore): added `rtl/rst_sync.v`, a standard
   reset synchronizer (async assert, 2FF sync de-assert — see
   `docs/reset.md`) that the raw external `rst_n` now feeds exclusively;
   every other register (soc_top's own, every peripheral, the CPU) resets
   from its output. picorv32 still consumes resetn purely synchronously
   internally (vendored, unchangeable) while everything else consumes the
   same synchronized value asynchronously — verilator's SYNCASYNCNET check
   flags same-*net* sync/async mixing, not same-*value* mixing, so
   `soc_top.v` feeds picorv32 through a distinctly-named alias wire
   (`cpu_resetn`, `= rst_n_sync`) purely to keep it a separate net for that
   one consumer; confirmed empirically (a plain continuous-assign alias is
   sufficient, verified in isolation before applying it here) and confirmed
   the full lint command now exits with zero warnings.
1. CI workflow: lint + all sims on every push (GitHub Actions) — done
   (`.github/workflows/ci.yml`: verilator lint gated on zero warnings, plus
   tb_soc integration + tb_quad_enc unit + tb_timer unit, each gated on
   `RESULT: PASS`).
2. Run the existing integration tb — confirm encoder + timer PASS — done
   (2026-07-29, see Verification status table)
3. Self-checking UART test — exact byte sequence
4. Self-checking PWM test — pulse width, period, enable/disable,
   mid-frame DUTY write (must not tear a pulse)
5. Unit tb for quad_enc — done (`tb/unit/tb_quad_enc.v`: all 8 legal
   transitions, all 4 illegal skips, clear, signed overflow/underflow at
   the 32-bit boundary, contact bounce, reset). Unit tb for timer — done
   (2026-07-29, `tb/unit/tb_timer.v`, 52 checks: period accuracy over 11
   consecutive gaps with cycle-exact spacing, IRQ pulse width/timing,
   STATUS write-1-to-clear (write-0 no-op, survives unrelated accesses),
   disable mid-count (holds COUNT at 0, no ticks), re-enable restarts the
   count from 0 rather than resuming, PERIOD change while running in both
   directions (growing extends the current period live; shrinking below
   the current COUNT now fires the tick on the very next clock instead of
   being missed - see below), reset state. Bounded-wait timeout guard
   (`fork`/`join_any`) on that last check so a regression to the old
   missed-tick behavior fails cleanly instead of hanging the sim/CI.
   Mutation-tested twice: against an off-by-one period compare (`== period`
   instead of `== period - 1`, caught with 14 mismatches) and, after the
   `>=` change below, against reverting `>=` back to `==` (caught with 3
   mismatches, no hang, thanks to the timeout guard). Both times confirmed
   restored to the real RTL after.

   **`timer.v` tick compare changed from `==` to `>=`** (2026-07-29):
   `counter == period - 1` meant shrinking PERIOD to a value at or below
   the running COUNT missed that tick until COUNT wrapped a 32-bit range -
   effectively never, for a control-loop heartbeat. Changed to
   `counter >= period - 1`, matching PERIOD's already-live (non-double-
   buffered) semantics: the tick now always fires within one clock of any
   PERIOD write, at latest. Rationale (a heartbeat must fail loud or fire
   early, never skip silently) and why this is safe *because* PERIOD is
   live-updated while PWM's DUTY is deliberately double-buffered (actuator
   vs. our-own-ISR consumer) are both written up in `docs/timer.md`.
6. Bus tests — byte strobes, invalid addresses, back-to-back transactions
7. Firmware-level test — program-visible results over UART — done
   (2026-07-29): the closed-loop control demo's hex telemetry
   (`P=/T=/D=/C=` — position/target/duty/ISR-cycles) is exactly this,
   decoded live from the UART pin by `sim/tb_soc.v`'s existing `uart_mon`.
   See item 9 below and `docs/control.md`.
8. Yosys synthesis check (before the FPGA step)
9. Closed-loop position control, phase A (autonomous profile, no new
   actuator) — done (2026-07-29, `docs/control.md`): fixed-point PID
   (Q8, `KP=800 KI=1 KD=800`) in the timer ISR, conditional-integration
   anti-windup, duty clamped to PWM's existing [1000,2000] range. Profile
   `0 -> +400 -> -200` counts, `sim/tb_soc.v` closes the loop with a
   behavioral velocity plant (duty -> lag -> velocity -> integrated
   position -> real quadrature transitions back into the encoder inputs).
   Both steps settle within the 300-ISR-tick budget with room to spare
   (143, 195 ticks) and low overshoot (1.5%, 0.8%) — `RESULT: PASS` gates
   on both. Independently cross-checked against `fw/control_model.py` (a
   Python re-implementation of the same fixed-point PID + plant), which
   agrees within a few ticks/counts (see `docs/control.md` for why exact
   match isn't expected). `rtl/timer.v`'s new `ISR_CYCLES` register
   measured real ISR duration at 168-184 cycles against a 2000-cycle
   budget — "measure, don't guess," not an assumed number. `fw/main.c`
   carries the same PID logic for real hardware, but the CI environment
   has no riscv32 toolchain, so it's written and reasoned through but not
   compiled here — hand off `cd fw && make` to whoever has the toolchain.
   Next (phase B, not yet): a real actuator/plant instead of the
   behavioral model, host-commanded setpoints instead of the fixed
   profile.

## Peripheral roadmap

1. ~~Quadrature encoder (0x0500_0000)~~ — RTL done (2FF sync, 4-state
   decoder, signed COUNT, clear); sim verified (unit tb + integration,
   see Verification status table)
2. ~~Timer + interrupt (0x0600_0000)~~ — RTL done (irq[3] every PERIOD
   clks, `ISR_CYCLES` @ 0x10, `docs/timer.md`); sim verified. ~~Closed-loop
   position demo (encoder -> PID -> PWM in the ISR)~~ — phase A done
   (autonomous profile, behavioral plant; see ladder item 9 and
   `docs/control.md`). Next: phase B (real actuator/plant, host-commanded
   setpoints)
3. UART RX — start-bit detect, mid-bit sampling, small FIFO
4. I2C master — MPU-6050 IMU
5. SPI master
6. Watchdog — unfed => PWM forced safe in hardware

Later phases (not yet): camera capture, tiny-NPU integration (perception
domain), shared-memory mailbox between perception and control domains.