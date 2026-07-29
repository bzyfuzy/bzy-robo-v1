# Firmware build (needs riscv32 gcc: apt install gcc-riscv64-unknown-elf works
# with -march=rv32im -mabi=ilp32, or use xpack riscv-none-elf-gcc)
#
# rv32im (not rv32i): the PID in main.c's timer_isr() needs real hardware
# multiply (soc_top.v: ENABLE_MUL+ENABLE_FAST_MUL) rather than a -nostdlib
# build silently failing to link libgcc's software __mulsi3. The "im" arch
# string nominally implies hardware divide too, but ENABLE_DIV stays 0 in
# hardware - safe only because main.c never uses '/' or '%', so the compiler
# never actually emits a div/rem instruction for this specific source file.
CROSS   ?= riscv64-unknown-elf-
CFLAGS   = -march=rv32im -mabi=ilp32 -Os -ffreestanding -nostdlib

firmware.hex: firmware.elf
	$(CROSS)objcopy -O verilog --verilog-data-width 4 $< $@.tmp
	grep -v '^@' $@.tmp > $@ && rm $@.tmp

firmware.elf: start.S main.c sections.lds
	$(CROSS)gcc $(CFLAGS) -Wl,-T,sections.lds -o $@ start.S main.c

sim-hex:
	python3 gen_firmware.py

clean:
	rm -f firmware.elf firmware.hex
