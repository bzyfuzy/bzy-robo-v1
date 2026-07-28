# Firmware build (needs riscv32 gcc: apt install gcc-riscv64-unknown-elf works
# with -march=rv32i -mabi=ilp32, or use xpack riscv-none-elf-gcc)
CROSS   ?= riscv64-unknown-elf-
CFLAGS   = -march=rv32i -mabi=ilp32 -Os -ffreestanding -nostdlib

firmware.hex: firmware.elf
	$(CROSS)objcopy -O verilog --verilog-data-width 4 $< $@.tmp
	grep -v '^@' $@.tmp > $@ && rm $@.tmp

firmware.elf: start.S main.c sections.lds
	$(CROSS)gcc $(CFLAGS) -Wl,-T,sections.lds -o $@ start.S main.c

sim-hex:
	python3 gen_firmware.py

clean:
	rm -f firmware.elf firmware.hex
