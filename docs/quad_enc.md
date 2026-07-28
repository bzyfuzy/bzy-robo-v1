# Quadrature encoder decoder

Base address: `0x0500_0000`
RTL: [`rtl/quad_enc.v`](../rtl/quad_enc.v)

## Registers

| Offset | Name  | Access | Reset | Description |
|--------|-------|--------|-------|--------------|
| 0x0    | COUNT | R      | 0x0   | 32-bit signed position count. |
| 0x4    | CTRL  | W      | —     | bit0 = CLEAR: write 1 to reset COUNT to 0 on that cycle. Reads back 0. |

## Inputs

| Signal | Description |
|--------|--------------|
| `enc_a` | Raw encoder A phase (asynchronous to `clk`) |
| `enc_b` | Raw encoder B phase (asynchronous to `clk`) |

## Decode behavior

* Each of `enc_a`/`enc_b` passes through an independent 2-flop synchronizer
  before use, since they're real encoder pins with no timing relationship
  to `clk`.
* Standard 4x (A+B edge) quadrature decode: the synchronized `{A,B}` pair
  forms a 2-bit Gray-coded state. Every valid single-step transition
  (`00↔01↔11↔10↔00`) advances COUNT by ±1 depending on direction.
* Any other transition (e.g. a state skip from noise or a signal glitch) is
  ignored — COUNT holds its value rather than corrupting.
* COUNT is a free-running signed 32-bit counter; it wraps on overflow
  (no saturation).

## Design notes

* CLEAR takes effect combinationally with the write cycle: `count <= 0` on
  the same edge, overriding the decoded delta for that cycle.
* Read of CTRL (offset 0x4) always returns 0 — it is not a stored register.

## Usage (C)

```c
#define ENC_BASE 0x05000000u
#define REG(a) (*(volatile uint32_t *)(a))
#define ENC_COUNT REG(ENC_BASE + 0x0)
#define ENC_CTRL  REG(ENC_BASE + 0x4)

ENC_CTRL = 1;              // zero the position count at boot
int32_t pos = (int32_t)ENC_COUNT;
```

## Verification

`sim/tb_soc.v` drives a waveform-level A/B generator (24 steps forward,
15 steps reverse) directly on `enc_a`/`enc_b` and checks COUNT against the
expected running total at every step, independent of firmware — this
catches decode errors that a register-readback-only test would miss.
