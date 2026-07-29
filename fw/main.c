// main.c - robot-soc closed-loop position control demo (C version, real HW)
// Build with the riscv32 toolchain via the provided Makefile (-march=rv32im:
// real hardware MUL is required and available - ENABLE_MUL+ENABLE_FAST_MUL
// in soc_top.v - but this file never uses '/' or '%', so the "im" arch
// string never triggers an actual DIV/REM emission even though the ISA
// string nominally implies both; ENABLE_DIV stays 0 in hardware).
//
// Same fixed-point PID (SHIFT/KP/KI/KD), plant-facing constants, and profile
// as fw/gen_firmware.py and fw/control_model.py - see docs/control.md for
// the cross-check procedure and gain-tuning notes.

#include <stdint.h>

#define PWM_BASE   0x02000000u
#define UART_BASE  0x03000000u
#define ENC_BASE   0x05000000u
#define TIMER_BASE 0x06000000u

#define REG(a) (*(volatile uint32_t *)(a))
#define PWM_CTRL       REG(PWM_BASE + 0x0)
#define PWM_PRESCALE   REG(PWM_BASE + 0x4)
#define PWM_PERIOD     REG(PWM_BASE + 0x8)
#define PWM_DUTY       REG(PWM_BASE + 0xC)
#define UART_DATA      REG(UART_BASE + 0x0)
#define UART_STATUS    REG(UART_BASE + 0x4)
#define ENC_COUNT      ((int32_t)REG(ENC_BASE + 0x0))
#define TIMER_CTRL     REG(TIMER_BASE + 0x0)
#define TIMER_PERIOD   REG(TIMER_BASE + 0x4)
#define TIMER_ISR_CYCLES REG(TIMER_BASE + 0x10)

// ---- PID / plant-facing constants (Q8 fixed point) -------------------------
#define SHIFT       8
#define KP          800
#define KI          1
#define KD          800
#define CENTER_DUTY 1500
#define DUTY_MIN    1000
#define DUTY_MAX    2000

// ---- profile (mirrored in gen_firmware.py / control_model.py / tb_soc.v) --
static const int32_t PROFILE[3] = {0, 400, -200};
#define SEG_TICKS 400
#define TELEMETRY_PERIOD 100

// PID-persistent state - touched only by timer_isr()
static volatile int32_t integral   = 0;
static volatile int32_t prev_error = 0;
// shared with main(): target is a single 32-bit-aligned store/load, which
// on RV32 is atomic with respect to an interrupt landing between
// instructions - same reasoning as gen_firmware.py's single-ADDI updates.
static volatile int32_t target = 0;
static volatile uint32_t tick_count = 0;
static volatile uint32_t isr_duration = 0;

// Entered via irq_vec (start.S), which has already saved every register
// this function could clobber and will retirq on return.
void timer_isr(void) {
    uint32_t entry_cyc = TIMER_ISR_CYCLES;   // measure, don't guess (docs/control.md)

    int32_t position = ENC_COUNT;
    int32_t error = target - position;
    int32_t integral_tentative = integral + error;
    int32_t derivative = error - prev_error;

    // correction uses an arithmetic (sign-extending) right shift on a
    // signed value - GCC on RV32 emits SRAI for this, matching
    // fw/gen_firmware.py's explicit srai and fw/control_model.py's sra().
    int32_t correction = (KP * error + KI * integral_tentative + KD * derivative) >> SHIFT;
    int32_t duty_unclamped = CENTER_DUTY + correction;

    // anti-windup (conditional integration): only commit the tentative
    // integral update if the unclamped duty isn't already saturated in the
    // direction error is pushing (that would only wind further into clamp)
    int skip_integration =
        (duty_unclamped > DUTY_MAX && error > 0) ||
        (duty_unclamped < DUTY_MIN && error < 0);
    if (!skip_integration) integral = integral_tentative;
    prev_error = error;

    int32_t duty = duty_unclamped;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    if (duty < DUTY_MIN) duty = DUTY_MIN;
    PWM_DUTY = (uint32_t)duty;

    tick_count++;
    isr_duration = TIMER_ISR_CYCLES - entry_cyc;
}

// irq_mask defaults to all-1s (everything masked) at reset. maskirq is a
// PicoRV32 custom-0 instruction, not RV32I, so there's no GCC mnemonic for
// it; pin the operand to a fixed register with a local register variable
// and emit the encoding as a raw word (same derivation as gen_firmware.py:
// funct7=0000011, rd=x0 (discard old mask), rs1=x1 -> 0x0600800b).
static inline void irq_unmask_timer(void) {
    register uint32_t mask asm("x1") = ~(1u << 3);
    asm volatile (".word 0x0600800b" :: "r"(mask));
}

static void putc(char c) {
    while (UART_STATUS & 1) ;
    UART_DATA = (uint32_t)c;
}
static void puts(const char *s) { while (*s) putc(*s++); }

// Prints v as 8 uppercase hex ASCII digits, MSB first. No '/' or '%' - same
// constraint and same output format as gen_firmware.py's hex telemetry, so
// both firmwares' UART output is byte-for-byte comparable.
static void print_hex32(uint32_t v) {
    int shamt;
    for (shamt = 28; shamt >= 0; shamt -= 4) {
        uint32_t nibble = (v >> shamt) & 0xFu;
        putc(nibble < 10 ? (char)('0' + nibble) : (char)('A' + (nibble - 10)));
    }
}

int main(void) {
    PWM_PRESCALE = 49;      // 50 MHz / (49+1) = 1 MHz tick -> 1 us resolution
    PWM_PERIOD   = 20000;   // 20 ms frame = 50 Hz servo rate
    PWM_DUTY     = CENTER_DUTY;
    PWM_CTRL     = 1;

    TIMER_PERIOD = 50000;   // 50 MHz / 50000 = 1 kHz tick (real-hardware rate;
                            // sim uses PERIOD=2000 for the same "1kHz-style"
                            // sim-accelerated convention - see docs/timer.md)
    TIMER_CTRL   = 1;
    irq_unmask_timer();

    puts("OK\n");

    uint32_t seg = 0;
    uint32_t next_telemetry = TELEMETRY_PERIOD;
    for (;;) {
        uint32_t tc = tick_count;

        if (seg == 0 && tc >= SEG_TICKS) {
            target = PROFILE[1];
            seg = 1;
        } else if (seg == 1 && tc >= 2 * SEG_TICKS) {
            target = PROFILE[2];
            seg = 2;
        }

        if (tc >= next_telemetry) {
            next_telemetry += TELEMETRY_PERIOD;
            puts("P="); print_hex32((uint32_t)ENC_COUNT);
            puts(" T="); print_hex32((uint32_t)target);
            puts(" D="); print_hex32(PWM_DUTY);
            puts(" C="); print_hex32(isr_duration);
            putc('\n');
        }
    }
}
