#!/usr/bin/env python3
"""
gen_firmware.py - hand-assembled RV32I(+PicoRV32 IRQ custom-ops) boot
firmware for robot-soc step 2 (timer interrupt).

What the program does:
  1. PWM: PRESCALE=0, PERIOD=20000, DUTY=1500, enable.
     (with PRESCALE=0 one tick = one clk, so a "frame" is 20000 cycles -
      fast to simulate; on a real 50 MHz board the C firmware uses
      PRESCALE=49 for true 1 us ticks.)
  2. TIMER: PERIOD=2000 cycles, enable -> fires irq[3] every 2000 clks.
  3. Unmask irq bit 3 (maskirq) so the timer can actually interrupt.
  4. UART: print "OK\n".
  5. Loop forever sweeping PWM DUTY 1000 -> 2000 in steps of 100 per frame
     delay. GPIO/LEDs are owned exclusively by the timer ISR (see below) -
     the main loop never touches them.

Interrupt vector layout (fixed convention, see soc_top.v PROGADDR_IRQ):
  Reset vector is 0x0 as always. PicoRV32's IRQ vector is hardwired here to
  0x1000 (word 1024) - comfortably past any boot code, before the RAM's own
  8 KB (2048-word) limit. The main program is padded with NOPs up to word
  1024, then the ISR (5 words) is placed at 0x1000, followed immediately by
  a single data word (the tick counter) at 0x1014.

PicoRV32's IRQ entry/exit uses non-standard "custom-0" (opcode 0x0B)
instructions rather than RISC-V CSRs - not in RV32I, so hand-encoded here
just like every other instruction:
  maskirq rd, rs : rd = old irq_mask; irq_mask = rs   (funct7 0000011)
  retirq         : jump to the return PC PicoRV32 saved in its hidden q0
                   register on IRQ entry, clear irq_active (funct7 0000010,
                   rs1/rd fields ignored - the core forces the source)
Bit layout for both: funct7[31:25] | rs1[19:15] | rd[11:7] | opcode[6:0]=0001011.

Every RV32I instruction below is encoded by the helper functions - readable
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

# PicoRV32 custom-0 (opcode 0x0B) IRQ instructions - not RV32I, hand-encoded
# straight from rtl/picorv32.v's decoder (search "instr_maskirq"/"instr_retirq").
def maskirq(rd, rs):
    return (0b0000011 << 25) | (reg(rs) << 15) | (reg(rd) << 7) | 0x0B

RETIRQ = (0b0000010 << 25) | 0x0B   # rs1/rd fields unused - core forces q0 as source

# registers used:
X0 = 0
PWM   = 5   # x5  = 0x0200_0000
UART  = 6   # x6  = 0x0300_0000
TMP   = 7
DUTY  = 8
STAT  = 10
CHR   = 11
DLY   = 12
LIM   = 13
GPIO  = 14  # x14 = 0x0400_0000 - owned by the timer ISR only, boot sets it up
TIMER = 17  # x17 = 0x0600_0000, boot-only scratch
MASK  = 1   # x1,  boot-only scratch (new irq_mask value)
CNTP  = 29  # x29 = address of the tick-counter word, set up once at boot,
            #       read by both boot code (never) and the ISR (every tick)
ISR_TMP = 28  # x28, used only inside the ISR - never touched by main code,
              #       so no context save/restore is needed on IRQ entry/exit

PROGADDR_IRQ = 0x1000          # must match soc_top.v's PROGADDR_IRQ parameter
ISR_LEN_WORDS = 5               # lw, addi, sw, sw, retirq
CNTR_ADDR = PROGADDR_IRQ + ISR_LEN_WORDS * 4   # 0x1014: data word right after the ISR

prog = []
def emit(word): prog.append(word)

# --- setup ------------------------------------------------------------------
emit(lui(PWM,  0x02000))          # x5  = PWM base
emit(lui(UART, 0x03000))          # x6  = UART base
emit(lui(GPIO, 0x04000))          # x14 = GPIO base (ISR-owned from here on)
emit(sw(X0, PWM, 4))              # PRESCALE = 0 (1 tick per clk, sim-fast)
emit(lui(TMP, 5))                 # x7 = 20480
emit(addi(TMP, TMP, -480))        # x7 = 20000
emit(sw(TMP, PWM, 8))             # PWM PERIOD = 20000
emit(addi(DUTY, X0, 1500))        # x8 = 1500
emit(sw(DUTY, PWM, 12))           # PWM DUTY = 1500
emit(addi(TMP, X0, 1))
emit(sw(TMP, PWM, 0))             # PWM CTRL = 1 (enable)

# --- timer setup: PERIOD=2000 clk cycles/tick, enable ------------------------
emit(lui(TIMER, 0x06000))         # x17 = TIMER base
emit(addi(TMP, X0, 2000))         # x7 = 2000 (fits the 12-bit addi immediate)
emit(sw(TMP, TIMER, 4))           # TIMER PERIOD = 2000
emit(addi(TMP, X0, 1))
emit(sw(TMP, TIMER, 0))           # TIMER CTRL = 1 (enable) -> irq[3] every 2000 clks

# --- counter-word pointer, set up once, read-only from here on --------------
emit(lui(CNTP, CNTR_ADDR >> 12))            # x29 = 0x1000
emit(addi(CNTP, CNTP, CNTR_ADDR & 0xFFF))   # x29 = CNTR_ADDR (0x1014)

# --- unmask irq bit 3 (timer) - all others (incl. reserved 0/1/2) stay masked
emit(addi(MASK, X0, ~(1 << 3) & 0xFFFFFFFF))   # x1 = 0xFFFFFFF7 (sign-extends to -9)
emit(maskirq(X0, MASK))                        # irq_mask = x1; old mask discarded

# --- print "OK\n" (poll STATUS busy bit, then write DATA) --------------------
for ch in (ord('O'), ord('K'), 10):
    emit(addi(CHR, X0, ch))       # CHR = char
    emit(lw(STAT, UART, 4))       # poll: STAT = STATUS
    emit(bne(STAT, X0, -4))       #       loop back to the lw while busy
    emit(sw(CHR, UART, 0))        # DATA = char

# --- sweep loop: DUTY 1000..2000 step 100, one delay per step ----------------
# (LEDs are no longer touched here - the timer ISR owns GPIO exclusively.)
emit(addi(DUTY, X0, 1000))        # DUTY = 1000
sweep_top = len(prog) * 4
emit(sw(DUTY, PWM, 12))           # write duty
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

# --- pad with NOPs up to the fixed IRQ vector address ------------------------
assert len(prog) * 4 <= PROGADDR_IRQ, "boot program grew past the ISR vector"
NOP = addi(X0, X0, 0)
while len(prog) * 4 < PROGADDR_IRQ:
    emit(NOP)

# --- ISR: increment the tick counter, mirror it to the LEDs, return ---------
assert len(prog) * 4 == PROGADDR_IRQ
emit(lw(ISR_TMP, CNTP, 0))        # x28 = *CNTP
emit(addi(ISR_TMP, ISR_TMP, 1))   # x28++
emit(sw(ISR_TMP, CNTP, 0))        # *CNTP = x28
emit(sw(ISR_TMP, GPIO, 0))        # LEDs = x28 (low byte latched by hw)
emit(RETIRQ)                      # return to the interrupted PC, clear irq_active
assert len(prog) == PROGADDR_IRQ // 4 + ISR_LEN_WORDS

# --- tick-counter data word, right after the ISR (PC never falls into it) --
assert len(prog) * 4 == CNTR_ADDR
emit(0)                           # counter starts at 0

with open("firmware.hex", "w") as f:
    for w in prog:
        f.write(f"{w:08x}\n")

print(f"firmware.hex written: {len(prog)} words ({len(prog)*4} bytes)")
