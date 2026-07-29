# Reset synchronizer

RTL: [`rtl/rst_sync.v`](../rtl/rst_sync.v), instantiated once in `soc_top.v`.
The raw external `rst_n` feeds only this module; every other register in the
design (soc_top's own bus logic, every peripheral, and the CPU) resets from
its synchronized output instead.

The synchronizer asserts asynchronously and de-asserts synchronously: the
output drops the instant the raw reset drops, with no clock edge required,
so actuator-facing outputs (PWM, etc.) are driven to a safe state immediately
regardless of clock activity - the same reasoning behind the peripheral bus
convention's async-reset rule. The output only rises again after two clean
`clk` edges once the raw reset returns high, which absorbs any metastability
from the raw reset's asynchronous release edge before it reaches the rest of
the design.
