# start.S - minimal reset entry: set stack, call main; plus the PicoRV32
# IRQ vector (custom-0 opcode 0x0B instructions - not RV32I, no toolchain
# mnemonics for them, so retirq is emitted as a raw .word; see gen_firmware.py
# for the from-the-RTL derivation of the encoding).
    .section .init
    .globl _start
_start:
    la sp, _stack_top
    call main
1:  j 1b

# PicoRV32 jumps to PROGADDR_IRQ (0x1000, fixed by sections.lds) directly on
# interrupt - unlike a normal call, nothing saved the caller's registers, so
# every register the C handler might touch has to be saved here first.
    .section .irqvec
    .globl irq_vec
irq_vec:
    addi sp, sp, -128
    sw   x1,   0(sp)
    sw   x3,   4(sp)
    sw   x4,   8(sp)
    sw   x5,  12(sp)
    sw   x6,  16(sp)
    sw   x7,  20(sp)
    sw   x8,  24(sp)
    sw   x9,  28(sp)
    sw   x10, 32(sp)
    sw   x11, 36(sp)
    sw   x12, 40(sp)
    sw   x13, 44(sp)
    sw   x14, 48(sp)
    sw   x15, 52(sp)
    sw   x16, 56(sp)
    sw   x17, 60(sp)
    sw   x18, 64(sp)
    sw   x19, 68(sp)
    sw   x20, 72(sp)
    sw   x21, 76(sp)
    sw   x22, 80(sp)
    sw   x23, 84(sp)
    sw   x24, 88(sp)
    sw   x25, 92(sp)
    sw   x26, 96(sp)
    sw   x27, 100(sp)
    sw   x28, 104(sp)
    sw   x29, 108(sp)
    sw   x30, 112(sp)
    sw   x31, 116(sp)

    call timer_isr

    lw   x1,   0(sp)
    lw   x3,   4(sp)
    lw   x4,   8(sp)
    lw   x5,  12(sp)
    lw   x6,  16(sp)
    lw   x7,  20(sp)
    lw   x8,  24(sp)
    lw   x9,  28(sp)
    lw   x10, 32(sp)
    lw   x11, 36(sp)
    lw   x12, 40(sp)
    lw   x13, 44(sp)
    lw   x14, 48(sp)
    lw   x15, 52(sp)
    lw   x16, 56(sp)
    lw   x17, 60(sp)
    lw   x18, 64(sp)
    lw   x19, 68(sp)
    lw   x20, 72(sp)
    lw   x21, 76(sp)
    lw   x22, 80(sp)
    lw   x23, 84(sp)
    lw   x24, 88(sp)
    lw   x25, 92(sp)
    lw   x26, 96(sp)
    lw   x27, 100(sp)
    lw   x28, 104(sp)
    lw   x29, 108(sp)
    lw   x30, 112(sp)
    lw   x31, 116(sp)
    addi sp, sp, 128

    .word 0x0400000b   # retirq: return to the interrupted PC, clear irq_active
