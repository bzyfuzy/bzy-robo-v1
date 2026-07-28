# robot-soc — Step 1: RISC-V + PWM minimal SoC

The first increment of a robot SoC: a PicoRV32 (RV32I) core, 8 KB RAM,
a glitch-free servo PWM peripheral, a TX UART for debug prints, and a
GPIO/LED register — all on PicoRV32's simple native memory bus.

**Status: verified in simulation.** The CPU boots, prints over a decoded
serial line, sweeps the servo PWM duty in clean 20 ms frames, and tracks a
quadrature encoder's position.

## Memory map

| Base        | Block              | Registers                                    |
| ----------- | ------------------ | -------------------------------------------- |
| 0x0000_0000 | RAM 8 KB           | firmware, reset vector = 0                   |
| 0x0200_0000 | PWM                | 0x0 CTRL, 0x4 PRESCALE, 0x8 PERIOD, 0xC DUTY |
| 0x0300_0000 | UART               | 0x0 DATA, 0x4 STATUS(bit0 busy)              |
| 0x0400_0000 | GPIO               | 0x0 LEDs                                     |
| 0x0500_0000 | Quadrature encoder | 0x0 COUNT (signed), 0x4 CTRL(bit0 clear)     |

Per-peripheral register reference, bit fields, and design notes: [docs/](docs/).

## Run the simulation (no RISC-V toolchain needed)

```bash
cd sim
python3 ../fw/gen_firmware.py      # hand-assembled boot firmware -> firmware.hex
iverilog -g2005-sv -o tb_soc.vvp tb_soc.v ../rtl/soc_top.v \
         ../rtl/pwm.v ../rtl/uart_tx.v ../rtl/quad_enc.v ../rtl/picorv32.v
vvp tb_soc.vvp                     # add: vvp tb_soc.vvp +trace  for a VCD
```

Expected: `[UART] 'O' 'K'`, PWM pulses stepping 1000 -> 1100 -> ... ticks,
encoder COUNT tracking a driven 24-step-forward/15-step-reverse waveform
with zero mismatches, final line `RESULT: PASS`.

## Real hardware

`fw/main.c` + `fw/Makefile` build the same program with a riscv32 gcc
(PRESCALE=49 for true 1 us ticks at 50 MHz). Constrain `pwm_out` to a
3.3 V pin -> servo signal wire (servo power from a separate 5 V supply,
common ground). `uart_txd` -> USB-serial adapter at clk/UART_DIV baud.

## Design notes

- PWM DUTY is double-buffered: software can write any time; the value
  loads only at frame boundaries, so the servo never sees a torn pulse.
- Every bus access completes in exactly one wait state — deterministic
  timing, the property the control domain of the robot SoC is built on.
- `fw/gen_firmware.py` is a readable mini-assembler: every RV32I encoding
  (LUI/ADDI/LW/SW/BNE/BLT/JAL) written out by hand.

## Roadmap

1. **(done)** CPU + PWM + UART: servo sweep
2. **(this)** Quadrature encoder input; next: timer interrupt -> 1 kHz closed-loop position control
3. NPU integration (perception domain) + camera capture
4. Mailbox contract: vision detections -> control setpoints
5. Replace the SBC in the pan/tilt tracker rig; measure latency and jitter

## License

Copyright (C) 2026 BzY\*FuZy

Licensed under the GNU General Public License v3.0 — see [LICENSE](LICENSE).
`rtl/picorv32.v` is vendored from the PicoRV32 project and retains its own
ISC license (see the header in that file); ISC is permissive and compatible
as an inbound license into this GPL-3.0 project.
