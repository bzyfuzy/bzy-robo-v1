// =============================================================================
// timer.v - periodic interrupt-request timer
//
// Register map (word offsets from base):
//   0x0  CTRL    bit0 = enable
//   0x4  PERIOD  tick period in clk cycles (irq_pulse fires once every
//                PERIOD clk cycles while enabled; PERIOD=0 disables ticking)
//   0x8  COUNT   current up-counter value, 0..PERIOD-1 (read-only, debug)
//   0xC  STATUS  bit0 = IRQ: sticky flag set on every tick; write 1 to bit0
//                clears it (W1C). Informational only - see design notes.
//   0x10 ISR_CYCLES  free-running cycle counter (read-only). Increments
//                every clk unconditionally (independent of enable/PERIOD),
//                only reset by rst_n. Firmware samples it at ISR entry and
//                exit to measure real ISR duration in cycles - "measure,
//                don't guess" (see docs/control.md) - rather than a
//                hierarchical peek, since it's genuinely bus-visible.
//
// Design notes:
//   * Same up-counter idiom as pwm.v: counter runs 0..period-1 and wraps,
//     giving exactly PERIOD clk cycles between ticks.
//   * irq_pulse is exactly one clk wide - matches PicoRV32's edge-triggered
//     (LATCHED_IRQ) external irq input. The core clears its own pending bit
//     the instant it enters the ISR, so firmware never has to ack the core
//     to keep ticking; STATUS.IRQ is a software-visible mirror for polling
//     or for cross-checking that the ISR actually ran.
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module timer (
    input  wire        clk,
    input  wire        rst_n,

    // simple register bus (one write/read per cycle, from soc_top)
    input  wire        sel,          // this peripheral is addressed
    input  wire [3:0]  wstrb,        // byte write strobes (0 = read)
    // 5 bits, not the usual 4 (see the Peripheral bus convention) - this is
    // the first peripheral with a 5th register (ISR_CYCLES @ 0x10), which
    // doesn't fit the standard 4-bit/4-word sub-address space.
    input  wire [4:0]  addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    output reg          irq_pulse    // one-cycle pulse, PERIOD clk cycles apart
);

    reg        enable;
    reg [31:0] period;
    reg [31:0] counter;
    reg        status_irq;
    reg [31:0] cycle_ctr;

    // >= (not ==): if PERIOD is written to a value at or below the current
    // COUNT, the heartbeat must fire on the very next clock, not be missed
    // until COUNT wraps 2^32 - see docs/timer.md for the rationale.
    wire tick = enable && (period != 0) && (counter >= period - 1);

    // ---- register writes ------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable <= 1'b0;
            period <= 32'd0;
        end else if (sel && wstrb != 4'b0000) begin
            case (addr)
                5'h00: enable <= wdata[0];
                5'h04: period <= wdata;
                default: ;
            endcase
        end
    end

    // ---- register reads ---------------------------------------------------
    always @(*) begin
        case (addr)
            5'h00:   rdata = {31'd0, enable};
            5'h04:   rdata = period;
            5'h08:   rdata = counter;
            5'h0C:   rdata = {31'd0, status_irq};
            5'h10:   rdata = cycle_ctr;
            default: rdata = 32'd0;
        endcase
    end

    // ---- up-counter + tick pulse ------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter   <= 32'd0;
            irq_pulse <= 1'b0;
        end else if (!enable) begin
            counter   <= 32'd0;
            irq_pulse <= 1'b0;
        end else if (tick) begin
            counter   <= 32'd0;
            irq_pulse <= 1'b1;
        end else begin
            counter   <= counter + 1;
            irq_pulse <= 1'b0;
        end
    end

    // ---- sticky STATUS.IRQ (software-visible, write-1-to-clear) -----------
    wire w1c = sel && (wstrb != 4'b0000) && (addr == 5'h0C) && wdata[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)    status_irq <= 1'b0;
        else if (tick) status_irq <= 1'b1;
        else if (w1c)  status_irq <= 1'b0;
    end

    // ---- ISR_CYCLES: free-running cycle counter (read-only, 0x10) --------
    // Runs unconditionally - independent of enable/PERIOD/reads/writes -
    // so two reads bracketing a code region give its true elapsed cycle
    // count. Wraps silently at 2^32, same as the CPU's own rdcycle.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cycle_ctr <= 32'd0;
        else        cycle_ctr <= cycle_ctr + 1;
    end

endmodule

`default_nettype wire
