MEMORY { RAM : ORIGIN = 0x00000000, LENGTH = 8K }
SECTIONS {
    .text : { *(.init) *(.text*) } > RAM
    /* Must match PROGADDR_IRQ in rtl/soc_top.v - PicoRV32 jumps here on
       interrupt, so irq_vec (fw/start.S) has to live at this exact address. */
    .irqvec 0x1000 : { KEEP(*(.irqvec)) } > RAM
    .data : { *(.data*) *(.rodata*) } > RAM
    .bss  : { *(.bss*) } > RAM
    _stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
