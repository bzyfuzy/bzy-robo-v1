# PWM — Servo PWM peripheral

Base address: `0x0200_0000`
RTL: [`rtl/pwm.v`](../rtl/pwm.v)

## Registers

| Offset | Name     | Access | Reset  | Description |
|--------|----------|--------|--------|--------------|
| 0x0    | CTRL     | R/W    | 0x0    | bit0 = enable. All other bits read 0. |
| 0x4    | PRESCALE | R/W    | 0x0    | Tick rate divider: `tick = clk / (PRESCALE+1)`. |
| 0x8    | PERIOD   | R/W    | 20000  | Frame length in ticks. |
| 0xC    | DUTY     | R/W    | 0x0    | High time in ticks. Writes go to a shadow register (see below). |

`pwm_out` is high while the frame counter is less than the active duty value,
low otherwise.

## Timing example

50 MHz clock, PRESCALE=49 → 1 MHz tick (1 us resolution). A standard servo
wants a 20 ms frame (50 Hz) with a 1–2 ms pulse:

```
PRESCALE = 49       // 1 us/tick
PERIOD   = 20000     // 20 ms frame
DUTY     = 1000..2000  // 1..2 ms pulse, 1500 = center
```

## Design notes

* **DUTY is double-buffered.** Software writes land in `duty_shadow` and are
  copied into `duty_active` only at the start of the next frame
  (`counter >= period - 1` on a tick). This guarantees a servo never sees a
  torn pulse, even if firmware updates DUTY mid-frame.
* Setting `CTRL.enable = 0` resets the frame counter and drives `pwm_out`
  low immediately; the prescaler also holds at 0 while disabled.
* `PERIOD` and `PRESCALE` are *not* double-buffered — only change them while
  disabled (or accept a torn frame on the change).

## Usage (C)

```c
#define PWM_BASE 0x02000000u
#define REG(a) (*(volatile uint32_t *)(a))
#define PWM_CTRL     REG(PWM_BASE + 0x0)
#define PWM_PRESCALE REG(PWM_BASE + 0x4)
#define PWM_PERIOD   REG(PWM_BASE + 0x8)
#define PWM_DUTY     REG(PWM_BASE + 0xC)

PWM_PRESCALE = 49;      // 1 MHz tick at 50 MHz clk
PWM_PERIOD   = 20000;   // 20 ms frame
PWM_DUTY     = 1500;    // center position
PWM_CTRL     = 1;       // enable
```

## Verification

`sim/tb_soc.v` includes a pulse-width monitor that measures `pwm_out` high
time against the expected DUTY value every frame and flags any mismatch.
