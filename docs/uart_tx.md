# UART TX — transmit-only debug UART (8N1)

Base address: `0x0300_0000`
RTL: [`rtl/uart_tx.v`](../rtl/uart_tx.v)

## Registers

| Offset | Name   | Access | Reset | Description |
|--------|--------|--------|-------|--------------|
| 0x0    | DATA   | W      | —     | Write a byte to send it. Write is ignored while `STATUS.busy` is set. |
| 0x4    | STATUS | R      | 0x0   | bit0 = busy. Poll until 0 before writing DATA. |

Reading DATA (or any offset other than 0x4) returns 0.

## Framing

8 data bits, no parity, 1 stop bit (8N1), LSB first. Baud rate is fixed at
build time via the `DIV` parameter on `uart_tx`:

```
DIV = clk_freq / baud
```

The simulation default and the real-hardware default are both `DIV = 434`
(50 MHz / 115200 baud), set via `soc_top`'s `UART_DIV` parameter.

## Design notes

* One in-flight byte at a time — there is no FIFO. Firmware must poll
  `STATUS.busy` before every write to DATA.
* The shifter holds `{stop, data[7:0], start}`; `bits_left` counts down from
  10 as bits are shifted out at the `DIV`-derived baud rate.
* This is TX-only. UART RX (start-bit detect, mid-bit sampling, small FIFO)
  is a separate future peripheral — see the roadmap in the project
  [`CLAUDE.md`](../CLAUDE.md).

## Usage (C)

```c
#define UART_BASE 0x03000000u
#define REG(a) (*(volatile uint32_t *)(a))
#define UART_DATA   REG(UART_BASE + 0x0)
#define UART_STATUS REG(UART_BASE + 0x4)

static void putc(char c) {
    while (UART_STATUS & 1) ;   // wait while busy
    UART_DATA = (uint32_t)c;
}
```

## Verification

`sim/tb_soc.v` includes a bit-level UART decoder that samples `uart_txd` at
the expected baud rate and reconstructs transmitted bytes, checked against
the expected boot string.
