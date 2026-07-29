# Peripheral reference

Register-level documentation for each peripheral on the bus. For the
address decode, bus protocol, and overall SoC layout, see the top-level
[README](../README.md) and [CLAUDE.md](../CLAUDE.md).

| Base        | Peripheral | Doc |
|-------------|------------|-----|
| 0x0200_0000 | PWM                | [pwm.md](pwm.md) |
| 0x0300_0000 | UART TX            | [uart_tx.md](uart_tx.md) |
| 0x0400_0000 | GPIO               | [gpio.md](gpio.md) |
| 0x0500_0000 | Quadrature encoder | [quad_enc.md](quad_enc.md) |
| 0x0600_0000 | Timer (IRQ)        | [timer.md](timer.md) |

SoC infrastructure (not memory-mapped):

| Block                | Doc |
|----------------------|-----|
| Reset synchronizer   | [reset.md](reset.md) |

Applications built on top of the peripherals above:

| Demo                              | Doc |
|-----------------------------------|-----|
| Closed-loop position control (PID)| [control.md](control.md) |
