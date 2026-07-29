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
      unit/           # create at next restructure: per-peripheral testbenches
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
| 0x0600_0000 | Timer (IRQ)        | 0x0 CTRL, 0x4 PERIOD, 0x8 COUNT, 0xC STATUS      |
| 0x0700_0000 | (next: UART RX)    |                                                  |

Firmware layout convention: reset code at 0x0, IRQ handler at 0x1000 —
`gen_firmware.py` and `sections.lds` must both respect this.

Assign new peripherals the next free 0xNN00_0000 slot; update this table,
the decoder, and the read mux in `soc_top.v` together, in the same commit.

## Peripheral bus convention

Every peripheral uses the same interface as `rtl/pwm.v` (the template):

    input  sel;            // address-decoded select from soc_top
    input  [3:0] wstrb;    // byte write strobes; 0000 = read
    input  [3:0] addr;     // word-aligned offset within the peripheral
    input  [31:0] wdata;
    output [31:0] rdata;   // combinational read mux inside the peripheral

Async active-low `rst_n`. External async inputs (encoder phases, future
sensor lines) pass through 2FF synchronizers before any logic. Outputs
that touch actuators must be glitch-free (see PWM's double-buffered DUTY).
Determinism is a feature: bus accesses are fixed-latency; no
variable-latency slaves on the control path. IRQ-related peripheral status
bits are write-1-to-clear from the ISR, never auto-clearing on read.

## Build, lint, test

Lint (run before every commit; zero warnings is the bar):

    verilator --lint-only -Wall rtl/soc_top.v rtl/pwm.v rtl/uart_tx.v \
              rtl/quad_enc.v rtl/timer.v

Full-SoC simulation (no RISC-V toolchain needed):

    cd sim
    python3 ../fw/gen_firmware.py
    iverilog -g2005-sv -o tb_soc.vvp tb_soc.v ../rtl/soc_top.v \
             ../rtl/pwm.v ../rtl/uart_tx.v ../rtl/quad_enc.v \
             ../rtl/timer.v ../rtl/picorv32.v
    vvp tb_soc.vvp            # add +trace for tb_soc.vcd

Add every new RTL file to both command lines above (and keep README in
sync). C firmware for real hardware: `fw/Makefile` (riscv32 gcc).

Planned restructure (own commit, zero functional changes, update all paths
here when done): `rtl/peripherals/`, `tb/unit/` + `tb/integration/`,
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
| Baseline (PWM+UART+GPIO boot) | done    | yes (original baseline)    |
| Quadrature encoder            | written | pending                    |
| Timer + IRQ                   | written | pending (hand-traced only) |

Update this table the moment a sim actually runs. "Hand-traced" is not a
verification state.

## Verification ladder (build in this order)

0. Verilator lint clean (now) — then keep it clean
1. CI workflow: lint + all sims on every push (GitHub Actions)
2. Run the existing integration tb — confirm encoder + timer PASS
3. Self-checking UART test — exact byte sequence
4. Self-checking PWM test — pulse width, period, enable/disable,
   mid-frame DUTY write (must not tear a pulse)
5. Unit tb for quad_enc, then for timer (see Rule 1 lists)
6. Bus tests — byte strobes, invalid addresses, back-to-back transactions
7. Firmware-level test — program-visible results over UART, including an
   ISR-counted timer check (ISR increments, main loop reports)
8. Yosys synthesis check (before the FPGA step)

## Peripheral roadmap

1. ~~Quadrature encoder (0x0500_0000)~~ — RTL done (2FF sync, 4-state
   decoder, signed COUNT, clear); sim verification pending
2. ~~Timer + interrupt (0x0600_0000)~~ — RTL done (irq[3] every PERIOD
   clks, ISR increments counter -> LEDs, `docs/timer.md`); sim
   verification pending; next: closed-loop position demo
   (encoder -> PID -> PWM in the 1 kHz ISR)
3. UART RX — start-bit detect, mid-bit sampling, small FIFO
4. I2C master — MPU-6050 IMU
5. SPI master
6. Watchdog — unfed => PWM forced safe in hardware

Later phases (not yet): camera capture, tiny-NPU integration (perception
domain), shared-memory mailbox between perception and control domains.