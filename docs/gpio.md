# GPIO — LED output register

Base address: `0x0400_0000`
RTL: inline in [`rtl/soc_top.v`](../rtl/soc_top.v) (no standalone module — small
enough to live directly in the top level).

## Registers

| Offset | Name | Access | Reset | Description |
|--------|------|--------|-------|--------------|
| 0x0    | LEDS | R/W    | 0x0   | bits [7:0] drive `leds[7:0]`. Bits [31:8] read 0. |

Writes update the register only on byte lane 0 (`wstrb_eff[0]`); the upper
three byte lanes of a write are ignored, so word writes with any value in
bits [31:8] are safe.

## Design notes

* Purely combinational passthrough to the top-level `leds` output — no
  internal timing, no side effects. Useful as a cheap debug/telemetry
  output (e.g. mirroring live state like the encoder count, as `fw/main.c`
  does).

## Usage (C)

```c
#define GPIO_BASE 0x04000000u
#define REG(a) (*(volatile uint32_t *)(a))
#define LEDS REG(GPIO_BASE + 0x0)

LEDS = 0xAA;              // static pattern
LEDS = ENC_COUNT & 0xFFu; // mirror live encoder count
```

## Verification

Covered indirectly in `sim/tb_soc.v` via the encoder-mirroring firmware
path; there is no dedicated GPIO monitor since the register is a direct
combinational passthrough.
