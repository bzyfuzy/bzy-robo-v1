#!/usr/bin/env python3
"""
gen_firmware.py - hand-assembled RV32I boot firmware for robot-soc step 1.

What the program does:
  1. PWM: PRESCALE=0, PERIOD=20000, DUTY=1500, enable.
     (with PRESCALE=0 one tick = one clk, so a "frame" is 20000 cycles -
      fast to simulate; on a real 50 MHz board the C firmware uses
      PRESCALE=49 for true 1 us ticks.)
  2. UART: print "OK\n".
  3. Loop forever sweeping DUTY 1000 -> 2000 in steps of 100 per frame delay,
     and each iteration copy the encoder COUNT (low byte) out to the LEDs so
     the live position count is observable.

Every instruction below is encoded by the helper functions - readable
RV32I encoding practice. Output: firmware.hex (one 32-bit word per line),
loadable by $readmemh.
"""

def reg(n): return n & 0x1F

def lui(rd, imm20):
    return ((imm20 & 0xFFFFF) << 12) | (reg(rd) << 7) | 0x37

def addi(rd, rs1, imm):
    return ((imm & 0xFFF) << 20) | (reg(rs1) << 15) | (0 << 12) | (reg(rd) << 7) | 0x13

def lw(rd, rs1, imm):
    return ((imm & 0xFFF) << 20) | (reg(rs1) << 15) | (2 << 12) | (reg(rd) << 7) | 0x03

def sw(rs2, rs1, imm):
    imm &= 0xFFF
    return ((imm >> 5) << 25) | (reg(rs2) << 20) | (reg(rs1) << 15) | (2 << 12) | ((imm & 0x1F) << 7) | 0x23

def _b_type(rs1, rs2, off, funct3):
    off &= 0x1FFF
    imm12   = (off >> 12) & 1
    imm10_5 = (off >> 5) & 0x3F
    imm4_1  = (off >> 1) & 0xF
    imm11   = (off >> 11) & 1
    return (imm12 << 31) | (imm10_5 << 25) | (reg(rs2) << 20) | (reg(rs1) << 15) | \
           (funct3 << 12) | (imm4_1 << 8) | (imm11 << 7) | 0x63

def bne(rs1, rs2, off): return _b_type(rs1, rs2, off, 0b001)
def blt(rs1, rs2, off): return _b_type(rs1, rs2, off, 0b100)

def jal(rd, off):
    off &= 0x1FFFFF
    imm20    = (off >> 20) & 1
    imm10_1  = (off >> 1) & 0x3FF
    imm11    = (off >> 11) & 1
    imm19_12 = (off >> 12) & 0xFF
    return (imm20 << 31) | (imm10_1 << 21) | (imm11 << 20) | (imm19_12 << 12) | (reg(rd) << 7) | 0x6F

# registers used:
X0 = 0
PWM  = 5   # x5 = 0x0200_0000
UART = 6   # x6 = 0x0300_0000
TMP  = 7
DUTY = 8
STAT = 10
CHR  = 11
DLY  = 12
LIM  = 13
GPIO = 14  # x14 = 0x0400_0000
ENC  = 15  # x15 = 0x0500_0000
CNT  = 16  # scratch: last-read encoder COUNT

prog = []
def emit(word): prog.append(word)

# --- setup ------------------------------------------------------------------
emit(lui(PWM,  0x02000))          #  0: x5 = PWM base
emit(lui(UART, 0x03000))          #  4: x6 = UART base
emit(lui(GPIO, 0x04000))          #    x14 = GPIO base
emit(lui(ENC,  0x05000))          #    x15 = ENC base
emit(sw(X0, PWM, 4))              #  8: PRESCALE = 0 (1 tick per clk, sim-fast)
emit(lui(TMP, 5))                 #  c: x7 = 20480
emit(addi(TMP, TMP, -480))        # 10: x7 = 20000
emit(sw(TMP, PWM, 8))             # 14: PERIOD = 20000
emit(addi(DUTY, X0, 1500))        # 18: x8 = 1500
emit(sw(DUTY, PWM, 12))           # 1c: DUTY = 1500
emit(addi(TMP, X0, 1))            # 20:
emit(sw(TMP, PWM, 0))             # 24: CTRL = 1 (enable)

# --- print "OK\n" (poll STATUS busy bit, then write DATA) --------------------
for ch in (ord('O'), ord('K'), 10):
    emit(addi(CHR, X0, ch))       # CHR = char
    emit(lw(STAT, UART, 4))       # poll: STAT = STATUS
    emit(bne(STAT, X0, -4))       #       loop back to the lw while busy
    emit(sw(CHR, UART, 0))        # DATA = char

# --- sweep loop: DUTY 1000..2000 step 100, one delay per step ----------------
emit(addi(DUTY, X0, 1000))        # DUTY = 1000
sweep_top = len(prog) * 4
emit(sw(DUTY, PWM, 12))           # write duty
emit(lw(CNT, ENC, 0))             # CNT = encoder COUNT
emit(sw(CNT, GPIO, 0))            # LEDs = COUNT (low byte latched by hw)
emit(lui(DLY, 5))                 # delay counter = 20000 (~ one frame)
emit(addi(DLY, DLY, -480))
emit(addi(DLY, DLY, -1))          # DLY--
emit(bne(DLY, X0, -4))            # while DLY != 0
emit(addi(DUTY, DUTY, 100))       # DUTY += 100
emit(addi(LIM, X0, 2001))
here = len(prog) * 4
emit(blt(DUTY, LIM, sweep_top - here))   # if DUTY < 2001 goto sweep_top
emit(addi(DUTY, X0, 1000))               # wrap to 1000
here = len(prog) * 4
emit(jal(X0, sweep_top - here))          # goto sweep_top

with open("firmware.hex", "w") as f:
    for w in prog:
        f.write(f"{w:08x}\n")

print(f"firmware.hex written: {len(prog)} instructions ({len(prog)*4} bytes)")
