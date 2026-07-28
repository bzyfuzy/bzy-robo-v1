# start.S - minimal reset entry: set stack, call main
    .section .init
    .globl _start
_start:
    la sp, _stack_top
    call main
1:  j 1b
