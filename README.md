# robot-soc — Step 1: RISC-V + PWM minimal SoC

The first increment of a robot SoC: a PicoRV32 (RV32I) core, 8 KB RAM,
a glitch-free servo PWM peripheral, a TX UART for debug prints, and a
GPIO/LED register — all on PicoRV32's simple native memory bus.

**Status: verified in simulation.** The CPU boots, prints over a decoded
serial line, then runs a closed-loop position-control demo entirely in the
timer ISR: a fixed-point PID reads the quadrature encoder and drives the
servo PWM duty. It starts on an autonomous step profile (0 -> +400 -> -200
counts) but hands control to a host the moment one arrives: type
`T+300\n` / `T-150\n` down the UART RX line to set the target interactively,
with hex telemetry (now sign-and-magnitude) streaming back over UART TX.
See [docs/control.md](docs/control.md).

## Memory map

| Base        | Block              | Registers                                    |
| ----------- | ------------------ | -------------------------------------------- |
| 0x0000_0000 | RAM 8 KB           | firmware, reset vector = 0                   |
| 0x0200_0000 | PWM                | 0x0 CTRL, 0x4 PRESCALE, 0x8 PERIOD, 0xC DUTY |
| 0x0300_0000 | UART               | 0x0 DATA, 0x4 STATUS(bit0 busy)              |
| 0x0400_0000 | GPIO               | 0x0 LEDs                                     |
| 0x0500_0000 | Quadrature encoder | 0x0 COUNT (signed), 0x4 CTRL(bit0 clear)     |
| 0x0600_0000 | Timer (IRQ)        | 0x0 CTRL, 0x4 PERIOD, 0x8 COUNT, 0xC STATUS, 0x10 ISR_CYCLES (ro) |
| 0x0700_0000 | UART RX            | 0x0 DATA (pops FIFO), 0x4 STATUS(bit0 avail, bit1 overflow, bit2 framing) |

Per-peripheral register reference, bit fields, and design notes: [docs/](docs/).

## Run the simulation (no RISC-V toolchain needed)

```bash
cd sim
python3 ../fw/gen_firmware.py      # hand-assembled boot firmware -> firmware.hex
iverilog -g2005-sv -o tb_soc.vvp tb_soc.v ../rtl/soc_top.v ../rtl/rst_sync.v \
         ../rtl/pwm.v ../rtl/uart_tx.v ../rtl/uart_rx.v ../rtl/quad_enc.v \
         ../rtl/timer.v ../rtl/picorv32.v
vvp tb_soc.vvp                     # add: vvp tb_soc.vvp +trace  for a VCD
```

Expected: `[UART] 'O' 'K'`, then hex telemetry lines (`P=... T=... D=... C=...`
- position/target/duty/last-ISR-cycles, `P=`/`T=` sign-and-magnitude)
roughly every 100 ticks, a 1 kHz-style timer interrupt ticking every 2000
clks in sim with zero period errors. The testbench then "becomes the
user": it types `T+300\n`, `T-150\n`, and one malformed `Tx9\n` down the
UART RX line at realistic baud, and `[CTRL]`/`[CMD]` lines confirm the
loop chases each accepted setpoint within the 300-tick/25%-overshoot
budget while the malformed command is rejected (`?`) without moving the
target. Final line `RESULT: PASS`. See [docs/control.md](docs/control.md)
for the full design and a Python cross-check (`fw/control_model.py`).

## Real hardware

`fw/main.c` + `fw/Makefile` build the same program with a riscv32 gcc
(`-march=rv32im` - real hardware multiply, see `docs/control.md`;
PRESCALE=49 for true 1 us ticks, TIMER_PERIOD=50000 for a true 1 kHz tick
at 50 MHz). Constrain `pwm_out` to a 3.3 V pin -> servo signal wire (servo
power from a separate 5 V supply, common ground). `uart_txd`/`uart_rxd` ->
USB-serial adapter at clk/UART_DIV baud - a terminal typing `T+300\n` etc.
drives the position setpoint interactively (`docs/control.md`).

## Design notes

- PWM DUTY is double-buffered: software can write any time; the value
  loads only at frame boundaries, so the servo never sees a torn pulse.
- Every bus access completes in exactly one wait state — deterministic
  timing, the property the control domain of the robot SoC is built on.
- The raw external `rst_n` feeds only `rtl/rst_sync.v`; every register in
  the design resets from its synchronized output instead - see
  [docs/reset.md](docs/reset.md).
- `fw/gen_firmware.py` is a readable mini-assembler: every RV32I encoding
  (LUI/ADDI/LW/SW/BNE/BLT/JAL), plus PicoRV32's custom IRQ opcodes
  (maskirq/retirq), written out by hand - see [docs/timer.md](docs/timer.md)
  for the encoding derivation and a cycle-level hand-trace of the first
  interrupt.

## Roadmap

1. **(done)** CPU + PWM + UART: servo sweep
2. **(done)** Quadrature encoder input
3. **(done)** Timer interrupt (1 kHz tick, first ISR)
4. **(done)** Encoder -> PID -> PWM closed-loop position control: phase A
   (autonomous profile, behavioral plant) + phase B (UART RX, interactive
   host-commanded setpoints) - `docs/control.md`; next (phase C, not yet):
   a real actuator/plant
5. NPU integration (perception domain) + camera capture
6. Mailbox contract: vision detections -> control setpoints
7. Replace the SBC in the pan/tilt tracker rig; measure latency and jitter

## License

Copyright (C) 2026 BzY\*FuZy

Licensed under the GNU General Public License v3.0 — see [LICENSE](LICENSE).
`rtl/picorv32.v` is vendored from the PicoRV32 project and retains its own
ISC license (see the header in that file); ISC is permissive and compatible
as an inbound license into this GPL-3.0 project.
