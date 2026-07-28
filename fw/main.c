// main.c - robot-soc step 1 firmware (C version, for real hardware)
// Build with the riscv32 toolchain via the provided Makefile.

#include <stdint.h>

#define PWM_BASE  0x02000000u
#define UART_BASE 0x03000000u
#define GPIO_BASE 0x04000000u
#define ENC_BASE  0x05000000u

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

    puts("robot-soc step1 boot\n");

    ENC_CTRL = 1;   // clear encoder COUNT at boot

    uint32_t duty = 1000, dir = 1;
    for (;;) {
        PWM_DUTY = duty;
        LEDS = ENC_COUNT & 0xFFu;   // live position count
        delay(200000);
        if (dir) { duty += 10; if (duty >= 2000) dir = 0; }
        else     { duty -= 10; if (duty <= 1000) dir = 1; }
    }
}
