# UART RX — receive-only debug UART (8N1), 16x-oversampled

Base address: `0x0700_0000`
RTL: [`rtl/uart_rx.v`](../rtl/uart_rx.v)

## Registers

| Offset | Name   | Access | Reset | Description |
|--------|--------|--------|-------|--------------|
| 0x0    | DATA   | R      | —     | Reading pops the oldest queued byte (0 if the FIFO is empty; never underflows). Writes are ignored. |
| 0x4    | STATUS | R/W    | 0x0   | bit0 = data-available (FIFO non-empty, live, not sticky). bit1 = FIFO overflow, sticky, write-1-to-clear (W1C). bit2 = framing error, sticky, write-1-to-clear (W1C). |

Same W1C convention as the timer's `STATUS.IRQ` (see
[`docs/timer.md`](timer.md)), including set-wins-over-a-simultaneous-clear:
if a new sticky event and a W1C write land on the same cycle, the set wins.

## Framing and sampling

8 data bits, no parity, 1 stop bit (8N1), LSB first - same framing as
`uart_tx`, opposite direction. `rx` is a real external pin (asynchronous to
`clk`), so it passes through a 2-flop synchronizer before any logic, per
this codebase's bus convention for async inputs (see CLAUDE.md's
Peripheral bus convention - the same rule quadrature encoder phases
follow).

Receiving is 16x-oversampled via the `OVERSAMPLE_DIV` parameter (bit period
= `OVERSAMPLE_DIV * 16` clk cycles - `DIV` on `uart_tx` is clk-cycles-per-
*bit*, `OVERSAMPLE_DIV` here is clk-cycles-per-*sample-tick*, 16 of which
make a bit). A free-running prescaler generates a sample tick throughout,
independent of receiver state - it is never reset to a start bit's edge, so
there's an inherent +-1 sample-tick quantization between the real edge and
the tick grid, same as real 16x-oversampling UART hardware.

On a falling edge (start bit), the receiver waits 8 sample ticks (half a
bit) to reach the start bit's center, then re-checks the line:

* still low -> a real start bit; sample each of the 8 data bits (LSB
  first), then the stop bit, each 16 sample ticks after the last
* already back high -> discard as a glitch. This is exactly what rejects a
  glitch shorter than half a bit: by the 8th sample tick a genuine short
  glitch has already ended, so the center check reads high and no byte is
  ever produced.

Framing: the stop bit is sampled the same way (16 ticks after the last data
bit). Sampled low -> sticky FERR, the byte is discarded, never queued.

## Overflow policy: drop-newest

If a good frame (valid stop bit) completes while the 8-deep FIFO already
holds 8 bytes, the *arriving* byte is dropped and sticky OVERFLOW is set -
the 8 already-queued bytes are never touched, reordered, or overwritten.

This was a deliberate choice over the alternatives:

* **Overwrite the oldest queued byte** would silently corrupt whatever the
  CPU is in the middle of reading - the byte at the front of the queue
  (the very next one to be popped) is the worst possible byte to touch,
  since firmware has no way to know its read is about to race a write.
* **Drop-newest**, by contrast, only ever discards a byte the CPU hasn't
  seen yet and has no expectation about - the queue's existing order and
  contents are guaranteed stable. Firmware finds out via the sticky
  OVERFLOW bit and can decide how to react (log it, resync a line-based
  protocol, etc.) without ever having silently lost or reordered a byte it
  already believed was queued.

A byte whose stop bit completes in the exact same cycle firmware pops
DATA is accepted if that pop frees the slot the new byte needs - fullness
for the push decision is evaluated *after* the concurrent pop, not before
(see `rtl/uart_rx.v` and its unit testbench's "byte arriving while the CPU
pops mid-stream" case).

## Why DATA's read needs a ready-qualified strobe

Every other register in this codebase is a plain combinational read mux -
timing doesn't matter, since reading has no side effect. DATA is the first
exception: reading it pops the FIFO, and `soc_top.v`'s bus holds a
peripheral's `sel`/`addr` asserted for **two** clock cycles per access (the
setup cycle and the ready cycle - see `rtl/soc_top.v`'s one-wait-state
design), not one. Popping on the raw `sel && addr==0` condition would fire
twice per logical read. `uart_rx.v` takes an additional `bus_ready` input
(wired to `soc_top.v`'s `mem_ready`) specifically to qualify the pop to the
one cycle that's actually the read's completion.

## Usage (C)

```c
#define UART_RX_BASE 0x07000000u
#define REG(a) (*(volatile uint32_t *)(a))
#define UART_RX_DATA   REG(UART_RX_BASE + 0x0)
#define UART_RX_STATUS REG(UART_RX_BASE + 0x4)

// non-blocking: returns 0 if nothing is queued
int try_getc(void) {
    if (!(UART_RX_STATUS & 1)) return -1;
    return (int)(uint8_t)UART_RX_DATA;
}
```

## Verification

`tb/unit/tb_uart_rx.v`: reset state; single bytes (all-zeros, all-ones,
alternating); back-to-back bytes at full line rate; FIFO fill to exactly
8 then a 9th overflows (sticky bit set, 9th dropped, first 8 intact and in
order, W1C clears it); framing error (low stop bit); a glitch shorter than
half a bit (must never produce a byte); a byte arriving while the CPU
concurrently pops queued bytes; mid-run reset. Mutation-tested: breaking
the start-bit center check (sampling at tick 0 instead of 8) is caught
directly by the glitch-rejection test.

`sim/tb_soc.v` exercises it at the system level: the testbench "becomes
the user," typing real `T+NNN`/`T-NNN` setpoint commands down the RX line
at realistic baud and verifying the firmware's command parser and the
closed control loop both respond correctly - see
[`docs/control.md`](control.md)'s command protocol section.
