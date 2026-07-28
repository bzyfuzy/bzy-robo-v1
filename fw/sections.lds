MEMORY { RAM : ORIGIN = 0x00000000, LENGTH = 8K }
SECTIONS {
    .text : { *(.init) *(.text*) } > RAM
    .data : { *(.data*) *(.rodata*) } > RAM
    .bss  : { *(.bss*) } > RAM
    _stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
