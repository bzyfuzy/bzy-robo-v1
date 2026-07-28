# CLAUDE.md — robot-soc

Minimal robot SoC: PicoRV32 (RV32I) + peripherals, building toward a
perception (NPU) + real-time control SoC for a pan/tilt robot. Currently in
the peripheral-building phase.

## Architecture

- CPU: `rtl/picorv32.v` (vendored, do not modify). Native memory bus,
  reset vector 0x0, stack top 0x2000. IRQ currently disabled
  (`ENABLE_IRQ(0)`) — will be enabled in the timer step.
- SoC top: `rtl/soc_top.v` — address decode on `mem_addr[31:24]`,
  registered one-wait-state ready for every access:
  `mem_ready <= mem_valid && !mem_ready;`
  Writes are honored only on the ready cycle (`wstrb_eff`).

## Memory map

| Base        | Block                      | Registers (word offsets)                         |
| ----------- | -------------------------- | ------------------------------------------------ |
| 0x0000_0000 | RAM 8KB                    | firmware via $readmemh, word index addr[12:2]    |
| 0x0200_0000 | PWM                        | 0x0 CTRL(en), 0x4 PRESCALE, 0x8 PERIOD, 0xC DUTY |
| 0x0300_0000 | UART TX                    | 0x0 DATA, 0x4 STATUS (bit0 busy)                 |
| 0x0400_0000 | GPIO                       | 0x0 LEDs (8-bit)                                 |
| 0x0500_0000 | Quadrature encoder         | 0x0 COUNT (signed), 0x4 CTRL (bit0 clear)        |
| 0x0600_0000 | (next: timer)              |

Assign new peripherals the next free 0xNN00_0000 slot; update this table,
the decoder, and the read mux in `soc_top.v` together.

## Peripheral bus convention

Every peripheral uses the same interface as `rtl/pwm.v` (use it as the
template):

    input  sel;            // address-decoded select from soc_top
    input  [3:0] wstrb;    // byte write strobes; 0000 = read
    input  [3:0] addr;     // word-aligned offset within the peripheral
    input  [31:0] wdata;
    output [31:0] rdata;   // combinational read mux inside the peripheral

Registers reset with async active-low `rst_n`. Control outputs that touch
physical actuators must be glitch-free (see PWM's double-buffered DUTY —
shadow register loaded at frame boundary).

## Build & test

Simulation (no RISC-V toolchain needed):

    cd sim
    python3 ../fw/gen_firmware.py
    iverilog -g2005-sv -o tb_soc.vvp tb_soc.v ../rtl/soc_top.v \
             ../rtl/pwm.v ../rtl/uart_tx.v ../rtl/quad_enc.v ../rtl/picorv32.v
    vvp tb_soc.vvp            # add +trace for tb_soc.vcd

Add every new RTL file to the iverilog command line (and keep README in
sync). C firmware for real hardware: `fw/Makefile` (riscv32 gcc).

## Rules

1. Every new peripheral gets a testbench monitor that verifies its actual
   output behavior (waveform-level, like the UART decoder and PWM
   pulse-width monitor in `sim/tb_soc.v`) — not just register readback.
2. The simulation must end with `RESULT: PASS`. A change that breaks an
   existing monitor is not done.
3. Extend `fw/gen_firmware.py` (hand-assembled RV32I) to exercise each new
   peripheral in the boot firmware; mirror it in `fw/main.c`.
4. One peripheral per commit. Baseline (PWM+UART+GPIO, passing) is the
   first commit.
5. Determinism is a feature: keep bus accesses fixed-latency; no
   variable-latency slaves on the control path.

## Roadmap (build in this order)

1. ~~Quadrature encoder decoder (0x0500_0000)~~ — done: 2FF synchronizers per
   input, 4-state transition decoder, 32-bit signed COUNT register, clear bit
2. Timer + interrupt — enable PicoRV32 IRQ, 1 kHz tick, first ISR;
   then closed-loop position control demo (encoder -> PID -> PWM)
3. UART RX — start-bit detect, mid-bit sampling, small FIFO
4. I2C master — for MPU-6050 IMU
5. SPI master
6. Watchdog — firmware stops petting it => PWM forced to safe state in
   hardware

Later phases (not yet): camera capture, tiny-NPU integration (perception
domain), shared-memory mailbox between perception and control.
