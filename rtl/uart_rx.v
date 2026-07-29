// =============================================================================
// uart_rx.v - receive-only UART (8N1), 16x-oversampled, 8-deep FIFO
//
// Register map (word offsets):
//   0x0  DATA    read pops the oldest queued byte (0 if empty, does not
//                underflow); writes ignored
//   0x4  STATUS  bit0 = data-available (fifo non-empty, live, not sticky)
//                bit1 = FIFO overflow: sticky, write-1-to-clear (W1C)
//                bit2 = framing error: sticky, write-1-to-clear (W1C)
//                (same W1C convention as timer.v's STATUS.IRQ - see
//                docs/timer.md - including set-wins-over-simultaneous-clear)
//
// rx is asynchronous (a real external pin), so it passes through a 2-flop
// synchronizer before any logic - same rule as the encoder phase inputs
// (see the Peripheral bus convention in CLAUDE.md).
//
// 16x oversampling: a free-running prescaler (period OVERSAMPLE_DIV clocks)
// generates a sample_tick pulse throughout, independent of RX state - it is
// never reset to a start bit's edge, so there's inherent +-1 sample_tick
// quantization between the real edge and the tick grid, same as real
// 16x-oversampling UART hardware. On a falling edge (start bit), the state
// machine waits 8 sample_tick pulses (half a bit) to reach the start bit's
// center and re-checks the line: still low -> real start bit, continue;
// back high already -> discard as a glitch (this is what rejects a glitch
// shorter than half a bit - by the 8th tick a real glitch has already
// ended, so the center check reads high). Each of the 8 data bits (LSB
// first) is then sampled every 16 sample_tick pulses at its own center,
// followed by the stop bit at the same cadence.
//
// Framing: stop bit sampled low -> sticky FERR, byte discarded (never
// queued). Overflow: a good frame arrives while the FIFO already holds 8
// bytes -> drop the newest (arriving) byte, sticky OVERFLOW, queued bytes
// are never touched - see docs/uart_rx.md for why drop-newest was chosen
// over drop-oldest/overwrite. A byte finishing its stop bit in the same
// cycle firmware pops DATA is accepted if that pop frees the slot the new
// byte needs (i.e. fullness is evaluated post-pop, not pre-pop).
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_rx #(
    parameter OVERSAMPLE_DIV = 2   // clk cycles per sample tick; bit period
                                    // = OVERSAMPLE_DIV * 16 clk cycles
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        sel,          // this peripheral is addressed
    input  wire [3:0]  wstrb,        // byte write strobes (0 = read)
    input  wire [3:0]  addr,         // word-aligned offset within peripheral
    // DATA/STATUS only use wdata[2:0] (STATUS W1C bits) - the rest of the
    // standard 32-bit-wdata bus is never read here, expected, not a bug.
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0] wdata,
    /* verilator lint_on UNUSEDSIGNAL */
    output reg  [31:0] rdata,

    // Bus reads are otherwise side-effect-free in this codebase (a plain
    // combinational mux), but DATA's read pops the FIFO - a genuine
    // exception needing the ready-qualified bus cycle (mem_ready in
    // soc_top.v), not just sel/addr, which stay asserted for two cycles
    // per access (see docs/uart_rx.md) and would otherwise double-pop.
    input  wire        bus_ready,

    input  wire        rx             // raw async RX line (idle = high)
);

    // ---- 2FF synchronizer ----------------------------------------------------
    reg rx_ff1, rx_ff2, rx_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_ff1 <= 1'b1; rx_ff2 <= 1'b1; rx_prev <= 1'b1;
        end else begin
            rx_ff1  <= rx;
            rx_ff2  <= rx_ff1;
            rx_prev <= rx_ff2;
        end
    end

    wire falling_edge = rx_prev && !rx_ff2;

    // ---- 16x oversample tick generator (free-running) -------------------------
    reg [31:0] samp_prescaler;
    wire sample_tick = (samp_prescaler == OVERSAMPLE_DIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)          samp_prescaler <= 32'd0;
        else if (sample_tick) samp_prescaler <= 32'd0;
        else                  samp_prescaler <= samp_prescaler + 1;
    end

    // ---- receive state machine -------------------------------------------------
    localparam ST_IDLE  = 2'd0;
    localparam ST_START = 2'd1;
    localparam ST_DATA  = 2'd2;
    localparam ST_STOP  = 2'd3;

    reg [1:0] state;
    reg [3:0] tick_cnt;   // sample_tick pulses since entering this phase
    reg [2:0] bit_idx;    // which data bit (0 = LSB, first bit after start)
    reg [7:0] shift_reg;

    wire stop_check  = (state == ST_STOP) && sample_tick && (tick_cnt == 4'd15);
    wire frame_ok    = stop_check && rx_ff2;
    wire frame_error = stop_check && !rx_ff2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            tick_cnt <= 4'd0;
            bit_idx  <= 3'd0;
            shift_reg <= 8'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (falling_edge) begin
                        state    <= ST_START;
                        tick_cnt <= 4'd0;
                    end
                end
                ST_START: begin
                    if (sample_tick) begin
                        if (tick_cnt == 4'd7) begin
                            if (!rx_ff2) begin   // still low at half-bit: real start bit
                                state    <= ST_DATA;
                                tick_cnt <= 4'd0;
                                bit_idx  <= 3'd0;
                            end else begin       // back high already: glitch, discard
                                state <= ST_IDLE;
                            end
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end
                ST_DATA: begin
                    if (sample_tick) begin
                        if (tick_cnt == 4'd15) begin
                            shift_reg[bit_idx] <= rx_ff2;
                            tick_cnt <= 4'd0;
                            if (bit_idx == 3'd7) state <= ST_STOP;
                            else                 bit_idx <= bit_idx + 1'b1;
                        end else begin
                            tick_cnt <= tick_cnt + 1'b1;
                        end
                    end
                end
                ST_STOP: begin
                    if (sample_tick) begin
                        if (tick_cnt == 4'd15) state <= ST_IDLE;
                        else                    tick_cnt <= tick_cnt + 1'b1;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    // ---- 8-deep FIFO ------------------------------------------------------------
    reg [7:0] fifo_mem [0:7];
    reg [2:0] wr_ptr, rd_ptr;
    reg [3:0] fifo_count;   // 0..8

    wire fifo_empty = (fifo_count == 4'd0);
    wire fifo_full  = (fifo_count == 4'd8);

    wire pop          = sel && (wstrb == 4'b0000) && (addr == 4'h0) && bus_ready;
    wire pop_effective = pop && !fifo_empty;

    // fullness for the push decision is evaluated post-pop: a byte whose
    // stop bit lands the same cycle firmware pops DATA is accepted if that
    // pop just freed the slot it needs.
    wire push_room   = !fifo_full || pop_effective;
    wire fifo_push   = frame_ok && push_room;
    wire fifo_overflow_event = frame_ok && !push_room;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr     <= 3'd0;
            rd_ptr     <= 3'd0;
            fifo_count <= 4'd0;
        end else begin
            case ({fifo_push, pop_effective})
                2'b10: begin
                    fifo_mem[wr_ptr] <= shift_reg;
                    wr_ptr     <= wr_ptr + 1'b1;
                    fifo_count <= fifo_count + 1'b1;
                end
                2'b01: begin
                    rd_ptr     <= rd_ptr + 1'b1;
                    fifo_count <= fifo_count - 1'b1;
                end
                2'b11: begin   // simultaneous push+pop: net count unchanged
                    fifo_mem[wr_ptr] <= shift_reg;
                    wr_ptr <= wr_ptr + 1'b1;
                    rd_ptr <= rd_ptr + 1'b1;
                end
                default: ;
            endcase
        end
    end

    // ---- sticky STATUS bits (write-1-to-clear) ---------------------------------
    reg overflow_sticky, ferr_sticky;
    wire w1c_overflow = sel && (wstrb != 4'b0000) && (addr == 4'h4) && wdata[1];
    wire w1c_ferr     = sel && (wstrb != 4'b0000) && (addr == 4'h4) && wdata[2];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                       overflow_sticky <= 1'b0;
        else if (fifo_overflow_event)     overflow_sticky <= 1'b1;
        else if (w1c_overflow)            overflow_sticky <= 1'b0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)               ferr_sticky <= 1'b0;
        else if (frame_error)     ferr_sticky <= 1'b1;
        else if (w1c_ferr)        ferr_sticky <= 1'b0;
    end

    // ---- register reads ---------------------------------------------------
    always @(*) begin
        case (addr)
            4'h0:    rdata = fifo_empty ? 32'd0 : {24'd0, fifo_mem[rd_ptr]};
            4'h4:    rdata = {29'd0, ferr_sticky, overflow_sticky, !fifo_empty};
            default: rdata = 32'd0;
        endcase
    end

endmodule

`default_nettype wire
