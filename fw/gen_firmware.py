#!/usr/bin/env python3
"""
gen_firmware.py - hand-assembled RV32I(+M mul, +PicoRV32 IRQ custom-ops) boot
firmware for robot-soc: closed-loop position control demo (phase A).

What the program does:
  1. PWM: PRESCALE=0, PERIOD=20000, DUTY=1500 (center), enable. The PID loop
     (below) owns DUTY exclusively from here on.
  2. TIMER: PERIOD=2000 cycles, enable -> fires irq[3] every 2000 clks. This
     is the control loop's tick, at the same sim-accelerated "1kHz-style"
     rate already established for this codebase (docs/timer.md): 1 ISR tick
     is treated as 1 ms of conceptual control-loop time, matching the real
     50 MHz-board setting (PERIOD=50000) proportionally.
  3. Unmask irq bit 3 (maskirq) so the timer can interrupt.
  4. UART: print "OK\\n".
  5. Zero-initialize the shared data block (target/integral/prev_error).
  6. Main loop: step the target profile (0 -> +400 -> -200 counts) at fixed
     tick-count thresholds, and print hex telemetry (position, target, duty,
     last ISR duration in cycles) roughly every 100 ticks.

Timer ISR (fixed-point PID, Q8 = SHIFT 8):
  error      = target - position (position = quadrature encoder COUNT)
  integral  += error                          (conditional - see anti-windup)
  derivative = error - prev_error
  correction = (KP*error + KI*integral + KD*derivative) >>> SHIFT   (srai)
  duty       = clamp(CENTER_DUTY + correction, DUTY_MIN, DUTY_MAX)
  Anti-windup (conditional integration): the tentative integral update is
  only committed if the *unclamped* duty is not saturated in the direction
  the error is already pushing - i.e. skip integrating when it would only
  wind the integrator further into a clamp it's already hit.
  ISR_CYCLES (timer offset 0x10, a free-running cycle counter) is sampled
  at entry and exit to measure real ISR duration - "measure, don't guess"
  (see docs/control.md) - stored for telemetry, not assumed.

Same fixed constants (SHIFT/KP/KI/KD, plant profile, telemetry cadence) are
mirrored in fw/main.c and the Python golden model (fw/control_model.py) -
see docs/control.md for the cross-check procedure and gain-tuning notes.

Interrupt vector layout (fixed convention, see soc_top.v PROGADDR_IRQ):
  Reset vector is 0x0. PicoRV32's IRQ vector is hardwired to 0x1000 (word
  1024). Boot + main-loop code is padded with NOPs up to word 1024, then
  the ISR, then a small shared data block (word-addressed, fixed offsets
  from DATA_BASE) immediately after the ISR.

PicoRV32's IRQ entry/exit uses non-standard "custom-0" (opcode 0x0B)
instructions rather than RISC-V CSRs - hand-encoded here just like every
other instruction:
  maskirq rd, rs : rd = old irq_mask; irq_mask = rs   (funct7 0000011)
  retirq         : jump to the return PC PicoRV32 saved in its hidden q0
                   register on IRQ entry, clear irq_active (funct7 0000010,
                   rs1/rd fields ignored - the core forces the source)
Bit layout for both: funct7[31:25] | rs1[19:15] | rd[11:7] | opcode[6:0]=0001011.

Every RV32I/M instruction below is encoded by the helper functions below -
readable RV32I encoding practice. A small two-pass assembler (labels +
patched branch/jump/load-address-of-label) replaces manual offset counting,
since this program has far more control flow than the original PWM demo.
Output: firmware.hex (one 32-bit word per line), loadable by $readmemh.
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

def beq(rs1, rs2, off): return _b_type(rs1, rs2, off, 0b000)
def bne(rs1, rs2, off): return _b_type(rs1, rs2, off, 0b001)
def blt(rs1, rs2, off): return _b_type(rs1, rs2, off, 0b100)

def jal(rd, off):
    off &= 0x1FFFFF
    imm20    = (off >> 20) & 1
    imm10_1  = (off >> 1) & 0x3FF
    imm11    = (off >> 11) & 1
    imm19_12 = (off >> 12) & 0xFF
    return (imm20 << 31) | (imm10_1 << 21) | (imm11 << 20) | (imm19_12 << 12) | (reg(rd) << 7) | 0x6F

def _r_type(rd, rs1, rs2, funct3, funct7):
    return (funct7 << 25) | (reg(rs2) << 20) | (reg(rs1) << 15) | (funct3 << 12) | (reg(rd) << 7) | 0x33

def add(rd, rs1, rs2): return _r_type(rd, rs1, rs2, 0b000, 0b0000000)
def sub(rd, rs1, rs2): return _r_type(rd, rs1, rs2, 0b000, 0b0100000)
def mul(rd, rs1, rs2): return _r_type(rd, rs1, rs2, 0b000, 0b0000001)   # RV32M

def andi(rd, rs1, imm):
    return ((imm & 0xFFF) << 20) | (reg(rs1) << 15) | (0b111 << 12) | (reg(rd) << 7) | 0x13

def srai(rd, rs1, shamt):
    return (0b0100000 << 25) | ((shamt & 0x1F) << 20) | (reg(rs1) << 15) | (0b101 << 12) | (reg(rd) << 7) | 0x13

def srli(rd, rs1, shamt):
    return (0b0000000 << 25) | ((shamt & 0x1F) << 20) | (reg(rs1) << 15) | (0b101 << 12) | (reg(rd) << 7) | 0x13

# PicoRV32 custom-0 (opcode 0x0B) IRQ instructions - not RV32I, hand-encoded
# straight from rtl/picorv32.v's decoder (search "instr_maskirq"/"instr_retirq").
def maskirq(rd, rs):
    return (0b0000011 << 25) | (reg(rs) << 15) | (reg(rd) << 7) | 0x0B

RETIRQ = (0b0000010 << 25) | 0x0B   # rs1/rd fields unused - core forces q0 as source

# =============================================================================
# Mini two-pass assembler: labels + patched branch/jump/load-address-of-label.
# Manual offset counting (as the original single-loop demo used) doesn't scale
# to this program's control-flow density; this removes that whole class of
# hand-arithmetic bugs.
# =============================================================================
prog = []
labels = {}
_patches = []   # list of callables invoked at resolve() time

def here():
    return len(prog)

def emit(word):
    prog.append(word)
    return len(prog) - 1

def label(name):
    assert name not in labels, f"duplicate label {name!r}"
    labels[name] = here()

def emit_jal(rd, target_label):
    idx = emit(0)
    def patch():
        off = (labels[target_label] - idx) * 4
        prog[idx] = jal(rd, off)
    _patches.append(patch)

def emit_branch(kind, rs1, rs2, target_label):
    idx = emit(0)
    enc = {'beq': beq, 'bne': bne, 'blt': blt}[kind]
    def patch():
        off = (labels[target_label] - idx) * 4
        prog[idx] = enc(rs1, rs2, off)
    _patches.append(patch)

def emit_load_const(rd, target_label):
    """LUI+ADDI pair loading the absolute byte address of target_label into
    rd, patched once the label's final word index is known. Handles the
    ADDI-sign-extension carry into the LUI's upper bits generally (not just
    for the lucky case where the low 12 bits happen to be < 0x800)."""
    lui_idx = emit(0)
    addi_idx = emit(0)
    def patch():
        addr = labels[target_label] * 4
        lo = addr & 0xFFF
        if lo & 0x800:
            lo -= 0x1000
        hi = (addr - lo) & 0xFFFFFFFF
        imm20 = (hi >> 12) & 0xFFFFF
        prog[lui_idx] = lui(rd, imm20)
        prog[addi_idx] = addi(rd, rd, lo)
    _patches.append(patch)

def resolve():
    for p in _patches:
        p()

def emit_li(rd, value):
    """LUI+ADDI pair loading a known 32-bit constant (e.g. a peripheral base
    address) into rd immediately - no patching needed, unlike
    emit_load_const (which is for the address of a *code label*, not known
    until assembly finishes)."""
    value &= 0xFFFFFFFF
    lo = value & 0xFFF
    if lo & 0x800:
        lo -= 0x1000
    hi = (value - lo) & 0xFFFFFFFF
    imm20 = (hi >> 12) & 0xFFFFF
    emit(lui(rd, imm20))
    emit(addi(rd, rd, lo))

_uniq = [0]
def uniq(base):
    _uniq[0] += 1
    return f"{base}_{_uniq[0]}"

def emit_wait_uart_send(char_reg, uart_base_reg, tmp_reg):
    """Poll STATUS busy bit, then write DATA = char_reg."""
    poll = uniq("uart_poll")
    label(poll)
    emit(lw(tmp_reg, uart_base_reg, 4))
    emit_branch('bne', tmp_reg, X0, poll)
    emit(sw(char_reg, uart_base_reg, 0))

# =============================================================================
# Register allocation
# =============================================================================
X0 = 0

# boot-only scratch (never touched again once interrupts are unmasked)
MASK = 1

# boot + main-loop shared scratch/pointers (ISR never touches these; an
# interrupt can land between any two of these instructions, so anything the
# main loop is mid-use of here would otherwise get clobbered)
UART   = 6    # UART TX base, 0x0300_0000
TMP    = 7    # general scratch
TEN    = 9    # constant 10, hoisted once for hex-nibble ASCII conversion and
              # for the command parser's decimal-digit range check
CHR    = 11   # character-to-send scratch
SEG    = 15   # main-persistent: which profile segment we're in (0,1,2)
NEXT_T = 16   # main-persistent: next telemetry tick_count threshold
MTMP   = 5    # general scratch #2
MTMP2  = 8    # general scratch #3

# command-parser state (main-loop-only, persists across loop iterations
# exactly like SEG/NEXT_T above)
PARSE_STATE  = 3    # 0=IDLE 1=SIGN 2=DIGITS 3=ERROR_SKIP
PARSE_VALUE  = 4    # decimal accumulator (multiply-by-10, no division needed)
PARSE_SIGN   = 10   # +1 or -1
CMD_RECEIVED = 12   # latches to 1 on the first accepted command, forever
                     # after disabling the autonomous profile fallback

# boot-loaded constant pointers: set once before interrupts are unmasked,
# read (never reassigned) by both the ISR and the main loop afterward
ENC_BASE     = 24   # 0x0500_0000
PWM_BASE     = 25   # 0x0200_0000
TIMER_BASE   = 26   # 0x0600_0000
UART_RX_BASE = 2    # 0x0700_0000 - main-loop-only (the ISR never touches
                     # UART RX), but still boot-loaded once alongside the
                     # other base pointers for consistency
DATA_BASE  = 29   # shared data block, right after the ISR (see below)

# ISR-only scratch (freely reused within one ISR call; never touched by
# main, so no save/restore is needed on IRQ entry/exit)
ISR_TMP  = 28
ISR_TMP2 = 31
POSITION = 23
TARGETR  = 22
ERROR    = 21
INTOLD   = 20
INTTENT  = 19
PERROLD  = 18
DERIV    = 17
DTERM    = 27
ITERM    = 14
PTERM    = 30
ENTRYCYC = 13   # entry ISR_CYCLES snapshot - dedicated for the whole ISR
                # body (ISR_TMP2 gets reused for DUTY_MAX/DUTY_MIN staging
                # partway through, so it can't also hold this)

# ---- shared data block offsets (from DATA_BASE) ----------------------------
D_TICK   = 0    # tick_count: ISR increments every call; main polls it
D_TARGET = 4    # target: main writes (single ADDI, atomic); ISR reads
D_INTEG  = 8    # integral: ISR-persistent PID state
D_PERR   = 12   # prev_error: ISR-persistent PID state
D_DUR    = 16   # isr_duration: ISR writes every call (cycles); main reads

PROGADDR_IRQ = 0x1000

# ---- PID / plant-facing constants (Q8 fixed point; mirrored in main.c and
#      fw/control_model.py - see docs/control.md) ----------------------------
SHIFT       = 8
KP          = 800
KI          = 1
KD          = 800
CENTER_DUTY = 1500
DUTY_MIN    = 1000
DUTY_MAX    = 2000

# ---- profile (mirrored in main.c / fw/control_model.py / tb_soc.v) --------
PROFILE = [0, 400, -200]
SEG_TICKS = 400          # ticks (== ISR periods) per profile segment
TELEMETRY_PERIOD = 100   # ticks between telemetry prints

# =============================================================================
# print helpers (main-loop-only: MTMP/MTMP2/TEN/UART/CHR, never touched by
# the ISR). Defined here, called from both the command parser's ok/? echo
# and the telemetry block below.
# =============================================================================
def emit_print_hex_word(val_reg):
    """Prints val_reg as 8 uppercase hex ASCII digits, MSB first."""
    for shamt in (28, 24, 20, 16, 12, 8, 4, 0):
        emit(srli(MTMP, val_reg, shamt))
        emit(andi(MTMP, MTMP, 0xF))
        after = uniq("hexdigit_after")
        emit_branch('blt', MTMP, TEN, after)
        emit(addi(MTMP, MTMP, 7))        # 'A'-'9'-1 adjustment
        label(after)
        emit(addi(MTMP, MTMP, 0x30))     # '0' ascii base
        emit_wait_uart_send(MTMP, UART, MTMP2)

def emit_print_signed_hex_word(val_reg):
    """Sign-and-magnitude, not raw two's complement: '-' then the hex
    magnitude if negative, otherwise identical to emit_print_hex_word.
    Mutates val_reg in place (negates it) - callers only ever pass a
    freshly-loaded scratch register (TMP), never a persistent one."""
    neg = uniq("signed_hex_neg")
    pos = uniq("signed_hex_pos")
    emit_branch('blt', val_reg, X0, neg)   # val_reg < 0 -> print '-' + magnitude
    emit_jal(X0, pos)
    label(neg)
    emit_print_char(ord('-'))
    emit(sub(val_reg, X0, val_reg))         # val_reg = -val_reg
    label(pos)
    emit_print_hex_word(val_reg)

def emit_print_char(c):
    emit(addi(CHR, X0, c))
    emit_wait_uart_send(CHR, UART, MTMP2)

def emit_print_ok():
    emit_print_char(ord('o')); emit_print_char(ord('k')); emit_print_char(10)

def emit_print_bad_response():
    emit_print_char(ord('?')); emit_print_char(10)

# =============================================================================
# boot: peripheral setup
# =============================================================================
emit_li(PWM_BASE, 0x02000000)
emit_li(UART, 0x03000000)
emit(sw(X0, PWM_BASE, 4))              # PRESCALE = 0 (1 tick per clk, sim-fast)
emit(lui(TMP, 5)); emit(addi(TMP, TMP, -480))   # x7 = 20000
emit(sw(TMP, PWM_BASE, 8))             # PWM PERIOD = 20000
emit(addi(TMP, X0, CENTER_DUTY))
emit(sw(TMP, PWM_BASE, 12))            # PWM DUTY = 1500 (center)
emit(addi(TMP, X0, 1))
emit(sw(TMP, PWM_BASE, 0))             # PWM CTRL = 1 (enable)

emit_li(TIMER_BASE, 0x06000000)
emit(addi(TMP, X0, 2000))
emit(sw(TMP, TIMER_BASE, 4))           # TIMER PERIOD = 2000
emit(addi(TMP, X0, 1))
emit(sw(TMP, TIMER_BASE, 0))           # TIMER CTRL = 1 (enable)

emit_li(ENC_BASE, 0x05000000)
emit_li(UART_RX_BASE, 0x07000000)
emit_load_const(DATA_BASE, "data_block")   # data_block's address isn't known
                                            # until the ISR (right before it)
                                            # is fully assembled - deferred

# zero-init the shared data block (RAM words are only guaranteed to be the
# value we explicitly emit(0) for, further down at data_block - this just
# establishes known starting register-free state before the ISR ever runs)
emit(sw(X0, DATA_BASE, D_TARGET))
emit(sw(X0, DATA_BASE, D_INTEG))
emit(sw(X0, DATA_BASE, D_PERR))
emit(sw(X0, DATA_BASE, D_DUR))

# unmask irq bit 3 (timer) - all others (incl. reserved 0/1/2) stay masked
emit(addi(MASK, X0, ~(1 << 3) & 0xFFFFFFFF))
emit(maskirq(X0, MASK))

# print "OK\n"
for ch in (ord('O'), ord('K'), 10):
    emit(addi(CHR, X0, ch))
    emit_wait_uart_send(CHR, UART, TMP)

# ---- main loop: command parsing + profile stepping + hex telemetry --------
emit(addi(SEG, X0, 0))                       # SEG = 0 (segment 0 == target 0, already set)
emit(addi(NEXT_T, X0, TELEMETRY_PERIOD))     # NEXT_T = 100
emit(addi(TEN, X0, 10))                      # TEN = 10, hoisted for hex-nibble compares
emit(addi(PARSE_STATE, X0, 0))               # PARSE_STATE = IDLE
emit(addi(CMD_RECEIVED, X0, 0))              # no command yet -> autonomous profile active

label("main_loop")
emit(lw(TMP, DATA_BASE, D_TICK))             # TMP = tick_count

# -- command parser: accept "T+NNN\n" / "T-NNN\n" over UART RX -----------
# Processes at most one received byte per loop iteration - plenty, since
# the main loop iterates far faster than bytes arrive at any real baud.
# Malformed lines are discarded (through the next '\n') and echo '?'; a
# valid line echoes "ok" and sets CMD_RECEIVED, permanently disabling the
# autonomous profile fallback below. See docs/control.md's command
# protocol section.
emit(lw(MTMP, UART_RX_BASE, 4))              # STATUS
emit(andi(MTMP, MTMP, 1))                    # data_available
uart_rx_skip = uniq("uart_rx_skip")
emit_branch('beq', MTMP, X0, uart_rx_skip)   # no byte waiting -> skip parser
emit(lw(MTMP, UART_RX_BASE, 0))               # pop the byte

ps_done = uniq("ps_done")

# --- state 0: IDLE - wait for 'T' ---------------------------------------
emit(addi(MTMP2, X0, 0))
ps_not_idle = uniq("ps_not_idle")
emit_branch('bne', PARSE_STATE, MTMP2, ps_not_idle)
emit(addi(MTMP2, X0, ord('T')))
ps_not_T = uniq("ps_not_T")
emit_branch('bne', MTMP, MTMP2, ps_not_T)
emit(addi(PARSE_STATE, X0, 1))               # -> SIGN
emit(addi(PARSE_VALUE, X0, 0))
label(ps_not_T)                              # not 'T': ignored, stay IDLE
emit_jal(X0, ps_done)
label(ps_not_idle)

# --- state 1: SIGN - expect '+' or '-' -----------------------------------
emit(addi(MTMP2, X0, 1))
ps_not_sign = uniq("ps_not_sign")
emit_branch('bne', PARSE_STATE, MTMP2, ps_not_sign)
emit(addi(MTMP2, X0, ord('+')))
ps_not_plus = uniq("ps_not_plus")
emit_branch('bne', MTMP, MTMP2, ps_not_plus)
emit(addi(PARSE_SIGN, X0, 1))
emit(addi(PARSE_STATE, X0, 2))               # -> DIGITS
emit_jal(X0, ps_done)
label(ps_not_plus)
emit(addi(MTMP2, X0, ord('-')))
ps_not_minus = uniq("ps_not_minus")
emit_branch('bne', MTMP, MTMP2, ps_not_minus)
emit(addi(PARSE_SIGN, X0, -1))
emit(addi(PARSE_STATE, X0, 2))
emit_jal(X0, ps_done)
label(ps_not_minus)                          # neither '+' nor '-' -> malformed
emit(addi(PARSE_STATE, X0, 3))               # -> ERROR_SKIP
emit_jal(X0, ps_done)
label(ps_not_sign)

# --- state 2: DIGITS - accumulate decimal digits, '\n' finalizes --------
emit(addi(MTMP2, X0, 2))
ps_not_digits = uniq("ps_not_digits")
emit_branch('bne', PARSE_STATE, MTMP2, ps_not_digits)
emit(addi(MTMP2, X0, 10))                    # '\n'
ps_not_nl = uniq("ps_not_nl")
emit_branch('bne', MTMP, MTMP2, ps_not_nl)
emit_branch('blt', PARSE_SIGN, X0, "cmd_negate_" + ps_not_nl)
emit_jal(X0, "cmd_store_" + ps_not_nl)
label("cmd_negate_" + ps_not_nl)
emit(sub(PARSE_VALUE, X0, PARSE_VALUE))      # apply '-' sign
label("cmd_store_" + ps_not_nl)
emit(sw(PARSE_VALUE, DATA_BASE, D_TARGET))   # target = parsed value (atomic store)
emit(addi(CMD_RECEIVED, X0, 1))
emit_print_ok()
emit(addi(PARSE_STATE, X0, 0))               # -> IDLE
emit_jal(X0, ps_done)
label(ps_not_nl)
emit(addi(MTMP2, X0, ord('0')))
emit(sub(MTMP2, MTMP, MTMP2))                # MTMP2 = digit = byte - '0'
ps_digit_invalid = uniq("ps_digit_invalid")
ps_digit_valid = uniq("ps_digit_valid")
emit_branch('blt', MTMP2, X0, ps_digit_invalid)   # digit < 0 -> not a digit
emit_branch('blt', MTMP2, TEN, ps_digit_valid)    # digit < 10 -> valid digit
label(ps_digit_invalid)
emit(addi(PARSE_STATE, X0, 3))               # -> ERROR_SKIP
emit_jal(X0, ps_done)
label(ps_digit_valid)
emit(mul(PARSE_VALUE, PARSE_VALUE, TEN))     # value = value*10 + digit
emit(add(PARSE_VALUE, PARSE_VALUE, MTMP2))
emit_jal(X0, ps_done)
label(ps_not_digits)

# --- state 3: ERROR_SKIP - discard until '\n', then echo '?' ------------
emit(addi(MTMP2, X0, 10))
ps_not_nl2 = uniq("ps_not_nl2")
emit_branch('bne', MTMP, MTMP2, ps_not_nl2)
emit_print_bad_response()
emit(addi(PARSE_STATE, X0, 0))               # -> IDLE
label(ps_not_nl2)

label(ps_done)
label(uart_rx_skip)

emit_branch('bne', CMD_RECEIVED, X0, "check_telemetry")  # a command already
                                              # arrived -> autonomous profile
                                              # fallback is retired for good

# -- segment 1 transition: tick_count >= SEG_TICKS && SEG==0 --------------
emit(addi(MTMP, X0, SEG_TICKS))
emit_branch('blt', TMP, MTMP, "check_seg2")      # not yet time -> skip
emit_branch('bne', SEG, X0, "check_seg2")        # already past segment 0 -> skip
emit(addi(MTMP, X0, PROFILE[1]))
emit(sw(MTMP, DATA_BASE, D_TARGET))          # target = PROFILE[1] (single-insn, atomic)
emit(addi(SEG, X0, 1))

# -- segment 2 transition: tick_count >= 2*SEG_TICKS && SEG==1 ------------
label("check_seg2")
emit(lw(TMP, DATA_BASE, D_TICK))
emit(addi(MTMP, X0, 2 * SEG_TICKS))
emit_branch('blt', TMP, MTMP, "check_telemetry") # not yet time -> skip
emit(addi(MTMP, X0, 1))
emit_branch('bne', SEG, MTMP, "check_telemetry") # not currently in segment 1 -> skip
emit(addi(MTMP, X0, PROFILE[2]))
emit(sw(MTMP, DATA_BASE, D_TARGET))
emit(addi(SEG, X0, 2))

label("check_telemetry")
emit(lw(TMP, DATA_BASE, D_TICK))
emit_branch('blt', TMP, NEXT_T, "main_loop_end")
emit(addi(NEXT_T, NEXT_T, TELEMETRY_PERIOD))

# ---- telemetry: "P=xxxxxxxx T=xxxxxxxx D=xxxxxxxx C=xxxxxxxx\n" -----------
# P=/T= (position/target) use sign-and-magnitude (print_signed_hex_word):
# a leading '-' then the hex magnitude, not raw two's complement - much
# more readable when interactively typing setpoints. D=/C= (duty, cycles)
# are always non-negative, so they keep plain hex.
emit_print_char(ord('P')); emit_print_char(ord('='))
emit(lw(TMP, ENC_BASE, 0))
emit_print_signed_hex_word(TMP)
emit_print_char(ord(' ')); emit_print_char(ord('T')); emit_print_char(ord('='))
emit(lw(TMP, DATA_BASE, D_TARGET))
emit_print_signed_hex_word(TMP)
emit_print_char(ord(' ')); emit_print_char(ord('D')); emit_print_char(ord('='))
emit(lw(TMP, PWM_BASE, 12))
emit_print_hex_word(TMP)
emit_print_char(ord(' ')); emit_print_char(ord('C')); emit_print_char(ord('='))
emit(lw(TMP, DATA_BASE, D_DUR))
emit_print_hex_word(TMP)
emit_print_char(10)

label("main_loop_end")
emit_jal(X0, "main_loop")

# --- pad with NOPs up to the fixed IRQ vector address ------------------------
assert here() * 4 <= PROGADDR_IRQ, "boot+main code grew past the ISR vector"
NOP = addi(X0, X0, 0)
while here() * 4 < PROGADDR_IRQ:
    emit(NOP)

# =============================================================================
# ISR: PID controller, run once per timer tick
# =============================================================================
assert here() * 4 == PROGADDR_IRQ
isr_start = here()

# ISR_CYCLES entry timestamp (measure, don't guess - see docs/control.md)
emit(lw(ENTRYCYC, TIMER_BASE, 0x10))           # entry cycle count

emit(lw(POSITION, ENC_BASE, 0))               # position = ENC.COUNT
emit(lw(TARGETR, DATA_BASE, D_TARGET))        # target
emit(sub(ERROR, TARGETR, POSITION))           # error = target - position

emit(lw(INTOLD, DATA_BASE, D_INTEG))          # integral (old)
emit(add(INTTENT, INTOLD, ERROR))             # integral_tentative = old + error

emit(lw(PERROLD, DATA_BASE, D_PERR))          # prev_error (old)
emit(sub(DERIV, ERROR, PERROLD))              # derivative = error - prev_error

emit(addi(ISR_TMP, X0, KP))
emit(mul(PTERM, ISR_TMP, ERROR))              # p_term = KP * error
emit(addi(ISR_TMP, X0, KI))
emit(mul(ITERM, ISR_TMP, INTTENT))            # i_term = KI * integral_tentative
emit(addi(ISR_TMP, X0, KD))
emit(mul(DTERM, ISR_TMP, DERIV))              # d_term = KD * derivative

emit(add(ISR_TMP, PTERM, ITERM))
emit(add(ISR_TMP, ISR_TMP, DTERM))            # sum = p+i+d
emit(srai(ISR_TMP, ISR_TMP, SHIFT))           # correction = sum >>> SHIFT
emit(addi(ISR_TMP2, X0, CENTER_DUTY))
emit(add(ISR_TMP, ISR_TMP2, ISR_TMP))         # duty_unclamped (ISR_TMP)

# ---- anti-windup: conditional integration -------------------------------
# commit integral_tentative UNLESS the unclamped duty is already saturated
# in the direction error is pushing (would only wind further into clamp)
emit(addi(ISR_TMP2, X0, DUTY_MAX))
emit_branch('blt', ISR_TMP2, ISR_TMP, "aw_check_high_err")   # duty_unclamped > MAX?
emit_jal(X0, "aw_check_low")
label("aw_check_high_err")
emit_branch('blt', X0, ERROR, "aw_skip")      # error > 0 -> skip integration
emit_jal(X0, "aw_commit")
label("aw_check_low")
emit(addi(ISR_TMP2, X0, DUTY_MIN))
emit_branch('blt', ISR_TMP, ISR_TMP2, "aw_check_low_err")    # duty_unclamped < MIN?
emit_jal(X0, "aw_commit")
label("aw_check_low_err")
emit_branch('blt', ERROR, X0, "aw_skip")      # error < 0 -> skip integration
label("aw_commit")
emit(sw(INTTENT, DATA_BASE, D_INTEG))         # integral = integral_tentative
label("aw_skip")
# (else: integral in memory is left unchanged - simply never overwritten)

emit(sw(ERROR, DATA_BASE, D_PERR))            # prev_error = error (always)

# ---- clamp duty to [DUTY_MIN, DUTY_MAX] ---------------------------------
emit(addi(ISR_TMP2, X0, DUTY_MAX))
emit_branch('blt', ISR_TMP2, ISR_TMP, "clamp_hi")
emit(addi(ISR_TMP2, X0, DUTY_MIN))
emit_branch('blt', ISR_TMP, ISR_TMP2, "clamp_lo")
emit_jal(X0, "clamp_done")
label("clamp_hi")
emit(addi(ISR_TMP, X0, DUTY_MAX))
emit_jal(X0, "clamp_done")
label("clamp_lo")
emit(addi(ISR_TMP, X0, DUTY_MIN))
label("clamp_done")
emit(sw(ISR_TMP, PWM_BASE, 12))               # PWM.DUTY = duty

# tick_count++
emit(lw(ISR_TMP, DATA_BASE, D_TICK))
emit(addi(ISR_TMP, ISR_TMP, 1))
emit(sw(ISR_TMP, DATA_BASE, D_TICK))

# ISR_CYCLES exit timestamp; isr_duration = exit - entry
emit(lw(ISR_TMP, TIMER_BASE, 0x10))
emit(sub(ISR_TMP, ISR_TMP, ENTRYCYC))
emit(sw(ISR_TMP, DATA_BASE, D_DUR))

emit(RETIRQ)
isr_len = here() - isr_start

# =============================================================================
# shared data block, right after the ISR
# =============================================================================
label("data_block")
emit(0)   # D_TICK
emit(0)   # D_TARGET
emit(0)   # D_INTEG
emit(0)   # D_PERR
emit(0)   # D_DUR

resolve()

with open("firmware.hex", "w") as f:
    for w in prog:
        f.write(f"{w:08x}\n")

print(f"firmware.hex written: {len(prog)} words ({len(prog)*4} bytes)")
print(f"ISR: {isr_len} words at 0x{isr_start*4:04x}; data_block at 0x{labels['data_block']*4:04x}")
