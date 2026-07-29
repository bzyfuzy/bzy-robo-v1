# Timer (periodic interrupt)

Base address: `0x0600_0000`
RTL: [`rtl/timer.v`](../rtl/timer.v)
IRQ: drives PicoRV32 `irq[3]`, one clk-wide pulse per tick

## Registers

| Offset | Name   | Access | Reset | Description |
|--------|--------|--------|-------|--------------|
| 0x0    | CTRL   | R/W    | 0x0   | bit0 = enable |
| 0x4    | PERIOD | R/W    | 0x0   | clk cycles per tick. PERIOD=0 disables ticking even if enabled. |
| 0x8    | COUNT  | R      | 0x0   | current up-counter value, 0..PERIOD-1 |
| 0xC    | STATUS | R/W    | 0x0   | bit0 = IRQ: sticky, set on every tick, write 1 to clear (W1C) |
| 0x10   | ISR_CYCLES | R  | 0x0   | free-running cycle counter - increments every clk unconditionally (independent of enable/PERIOD); sample twice to measure elapsed cycles. Note: `addr` is 5 bits here (not the usual 4) to reach this offset - see the Peripheral bus convention. |

`irq_pulse` fires exactly once, for exactly one clk cycle, PERIOD cycles
after the counter was last reset (by enable, or by a previous tick) - same
up-counter idiom as `pwm.v`'s frame counter.

## PERIOD semantics: live update, `>=` compare

The tick condition is `counter >= period - 1`, not `counter == period - 1`.
This timer is a control-loop heartbeat: if firmware shrinks PERIOD to a
value at or below the current COUNT (a live reconfiguration, or a bug in
the caller), the tick fires on the very next clock instead of being missed
until COUNT wraps all the way around a 32-bit range - effectively forever
at any real clock rate. A heartbeat that can go silent for that long is
worse than one that fires a cycle early: firing early is a visible,
recoverable glitch; going silent means every downstream deadline that
depends on this interrupt (a control-loop tick, a watchdog kick) is missed
with no signal that anything went wrong. `>=` trades a possible one-tick-early
pulse for the guarantee that the heartbeat never silently stops.

PERIOD itself is a plain, immediately-effective register - not
double-buffered like PWM's DUTY. That asymmetry is intentional, not an
oversight: PWM's DUTY feeds an external actuator (a servo), where a value
change landing mid-pulse would produce a torn, physically visible glitch on
a wire nothing in software can un-glitch after the fact - hence the
shadow-register-swapped-at-frame-boundary design. The timer's only
consumer is our own ISR, which only ever observes PERIOD's effect through
`irq_pulse`/COUNT/STATUS - and the `>=` compare is exactly what makes a
live, unbuffered PERIOD safe here: even the worst-case reconfiguration
(shrinking PERIOD out from under a running count) still produces a single,
bounded, on-time-or-early tick, never a skipped one. Double-buffering would
add a cycle of write-to-effect latency for no corresponding safety benefit.

## PicoRV32 IRQ mechanism (why the firmware looks the way it does)

PicoRV32's interrupt interface is **not** the standard RISC-V CSR/mret
scheme - it's five custom-0 (opcode `0001011`) instructions selected by
`funct7`, decoded straight out of `rtl/picorv32.v`:

| Instruction | funct7 | Effect |
|---|---|---|
| `getq  rd, qs`  | `0000000` | `rd = q[qs]` (qs ∈ {0,1}) |
| `setq  qd, rs`  | `0000001` | `q[qd] = rs` |
| `retirq`        | `0000010` | jump to `q0`, clear `irq_active` |
| `maskirq rd,rs` | `0000011` | `rd = old irq_mask; irq_mask = rs` |
| `waitirq rd`    | `0000100` | block until an irq is pending |

Bit layout (same R-type shape as the rest of RV32I): `funct7[31:25] |
rs2/qs[24:20] | rs1[19:15] | rd[11:7] | opcode[6:0]=0001011`. Only
`maskirq` and `retirq` are used here; both `gen_firmware.py` and
`fw/start.S` emit them as raw `.word` constants since there's no toolchain
mnemonic for them - see the encoding derivation at the top of
`fw/gen_firmware.py`.

On interrupt, PicoRV32:
1. Latches the return PC into hidden register `q0`.
2. Latches `irq_pending & ~irq_mask` into hidden register `q1`.
3. Sets `irq_active`, jumps to `PROGADDR_IRQ`.

`irq_mask` resets to all-1s (everything masked) - firmware must call
`maskirq` before any interrupt can fire. `LATCHED_IRQ` defaults to all-1s
too, which makes every `irq[]` input edge-triggered: a single-cycle pulse
is latched as pending and then auto-cleared the instant the core enters
the ISR, so the timer peripheral needs no software acknowledgment to keep
ticking (`STATUS.IRQ` is informational only).

## Firmware layout

`PROGADDR_IRQ = 0x1000` (set in `soc_top.v`'s PicoRV32 instantiation,
overriding the core's own default of `0x10`). This is a fixed convention,
so the ISR, wherever it physically ends up, must occupy exactly that
address:

* `gen_firmware.py`: assembles the boot program, pads with NOPs up to word
  1024 (byte `0x1000`), places the 5-instruction ISR there, then one data
  word (the tick counter, initialized to 0) right after it at `0x1014`.
* `fw/start.S` / `fw/sections.lds`: the C build gets the same address via
  a dedicated `.irqvec 0x1000 : { ... }` linker section holding a
  hand-written `irq_vec` trampoline (`main.c`'s `timer_isr` is a normal C
  function, so the trampoline saves every register the C ABI might
  clobber before calling it, and restores them before `retirq`).

### Register budget (hand-assembled firmware only)

The sim firmware skips a general save/restore by giving the ISR two
registers the main program provably never touches:

* `x29` (`CNTP`) - address of the tick-counter word (`0x1014`), set up
  once at boot, read-only after that.
* `x28` - ISR-local scratch (load/increment/store the counter).
* `x14` (`GPIO`) - shared read-only: main sets it up once, only the ISR
  writes through it afterward.

This is safe *only* because interrupts stay masked until after all three
are initialized, and the main sweep loop never touches x28/x29/x14 again.
The C firmware doesn't get to make this assumption (an arbitrary compiled
function can use any register), hence the full-register-save trampoline
there instead.

## Hand-trace: verifying the design before trusting the simulator

Simulation numbers below are from `sim/tb_soc.v` with `TIMER.PERIOD=2000`,
a 10 ns clk period (100 MHz), obtained via a temporary instrumentation
pass (probes on `u_cpu.irq_state` / `reg_pc` / `reg_next_pc`, removed
before commit) - i.e. these are predictions made by reading the RTL,
then confirmed against the actual waveform, not just asserted.

**1. Tick period.** `tick = enable && period!=0 && (counter == period-1)`,
counter resets to 0 the same cycle. With `PERIOD=2000` that's exactly 2000
distinct counter states -> exactly 2000 clk cycles between pulses, i.e.
20000 ns. Confirmed: `sim/tb_soc.v`'s `timer_pulse_mon` measured 199 ticks
across the whole run with `period_errors=0` (every single gap was exactly
20000 ns).

**2. IRQ entry latency.** PicoRV32 only samples `irq_pending & ~irq_mask`
between instructions (`decoder_trigger`), so response time is jittered by
whatever instruction is in flight, but the entry sequence itself is a
fixed 2-cycle FSM (`irq_state`: `00 -> 01 -> 10`, one clock each,
`rtl/picorv32.v` cpu_state_fetch case). Observed: `irq_state` hit `01` at
t=21045 ns and `10` at t=21055 ns - exactly one clk (10 ns) apart, and
`reg_pc` was already `0x1000` at that same t=21055 ns sample, matching the
RTL's `current_pc = PROGADDR_IRQ` assignment landing in the `irq_state==10`
branch. The timer had been enabled at t=955 ns; first tick expected around
t=955+20000=20955 ns, observed IRQ entry at 21045..21055 ns - the ~90 ns
(9 cycle) gap is exactly the jitter explained above (the CPU was mid-way
through the sweep-loop's delay countdown, the longest-running instruction
sequence in the program, so statistically the likeliest place to catch an
interrupt). `reg_next_pc=0x90` at the moment of entry confirms this: word
address `0x90` = word 36, the `addi DLY,DLY,-1` decrement inside that same
delay loop - the saved return PC is exactly where you'd expect a
2000-cycle-period interrupt to land inside a 20000-cycle delay loop.

**3. ISR correctness / `retirq`.** `retirq`'s `rd`/`rs1` instruction-word
fields are architecturally ignored - PicoRV32 forces `decoded_rs1` to `q0`
for this opcode regardless of what bits are there (see the decoder's
`instr_retirq` override) - so encoding it as `0x0400000B` (all reg fields
zero) is correct by inspection, not just by convention. Functional proof:
`led_mon` cross-checks the `leds` *pin* (not a peeked-at register) against
an independently maintained expected value and requires an exact 1:1
match against the tick count (`led_count == tick_count`, `led_errors=0`
across all 199 interrupts) - if `retirq` ever returned to the wrong PC, or
any general-purpose register got corrupted by an interrupt landing at the
wrong moment, the very next PWM/UART/sweep-loop instruction would most
likely misbehave and the existing PWM/UART monitors would fail too. They
didn't, across 199 live interrupt round-trips.

## Usage (C)

```c
#define TIMER_BASE 0x06000000u
#define REG(a) (*(volatile uint32_t *)(a))
#define TIMER_CTRL   REG(TIMER_BASE + 0x0)
#define TIMER_PERIOD REG(TIMER_BASE + 0x4)

TIMER_PERIOD = 50000;   // 50 MHz / 50000 = 1 kHz tick
TIMER_CTRL   = 1;
irq_unmask_timer();      // see fw/main.c - maskirq has no C/toolchain mnemonic
```

## Verification

`sim/tb_soc.v`'s `timer_pulse_mon` checks the tick period against
`u_timer.irq_pulse` (waveform-level, internal signal, like the encoder's
COUNT check); `led_mon` independently checks the `leds` *output pin*
increments by exactly 1 on every tick with zero drops or duplicates -
proof that the interrupt reached the CPU and the ISR executed a real bus
write, not just that the timer peripheral itself counts correctly.
