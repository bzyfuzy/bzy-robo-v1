// main.c - robot-soc step 1 firmware (C version, for real hardware)
// Build with the riscv32 toolchain via the provided Makefile.

#include <stdint.h>

#define PWM_BASE   0x02000000u
#define UART_BASE  0x03000000u
#define GPIO_BASE  0x04000000u
#define ENC_BASE   0x05000000u
#define TIMER_BASE 0x06000000u

#define REG(a) (*(volatile uint32_t *)(a))
#define PWM_CTRL     REG(PWM_BASE + 0x0)
#define PWM_PRESCALE REG(PWM_BASE + 0x4)
#define PWM_PERIOD   REG(PWM_BASE + 0x8)
#define PWM_DUTY     REG(PWM_BASE + 0xC)
#define UART_DATA    REG(UART_BASE + 0x0)
#define UART_STATUS  REG(UART_BASE + 0x4)
#define LEDS         REG(GPIO_BASE + 0x0)
#define ENC_COUNT    REG(ENC_BASE  + 0x0)
#define ENC_CTRL     REG(ENC_BASE  + 0x4)
#define TIMER_CTRL   REG(TIMER_BASE + 0x0)
#define TIMER_PERIOD REG(TIMER_BASE + 0x4)

// GPIO/LEDs are owned exclusively by timer_isr() below - main() never
// writes them, so there's no race between the mainline code and the ISR.
static volatile uint32_t tick_count = 0;

// Entered via irq_vec (start.S), which has already saved every register
// this function could clobber and will retirq on return.
void timer_isr(void) {
    tick_count++;
    LEDS = tick_count & 0xFFu;
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

static void delay(volatile uint32_t n) { while (n--) ; }

int main(void) {
    // 50 MHz clock / (49+1) = 1 MHz tick -> 1 us resolution
    PWM_PRESCALE = 49;
    PWM_PERIOD   = 20000;   // 20 ms frame = 50 Hz servo rate
    PWM_DUTY     = 1500;    // center position (1.5 ms)
    PWM_CTRL     = 1;

    TIMER_PERIOD = 50000;   // 50 MHz / 50000 = 1 kHz tick
    TIMER_CTRL   = 1;
    irq_unmask_timer();

    puts("robot-soc step1 boot\n");

    ENC_CTRL = 1;   // clear encoder COUNT at boot

    uint32_t duty = 1000, dir = 1;
    for (;;) {
        PWM_DUTY = duty;
        delay(200000);
        if (dir) { duty += 10; if (duty >= 2000) dir = 0; }
        else     { duty -= 10; if (duty <= 1000) dir = 1; }
    }
}
