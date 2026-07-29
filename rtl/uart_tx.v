// =============================================================================
// uart_tx.v - transmit-only UART (8N1)
//
// Register map (word offsets):
//   0x0  DATA    write a byte here to send it (write ignored while busy)
//   0x4  STATUS  bit0 = busy (poll until 0 before writing DATA)
//
// DIV = clk_freq / baud. e.g. 50 MHz / 115200 = 434.
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module uart_tx #(
    parameter DIV = 434
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        sel,
    // DATA is a single byte on the standard 4-bit-wstrb/32-bit-wdata bus:
    // only wstrb[0] gates the write and only wdata[7:0] is the byte, so
    // wstrb[3:1] and wdata[31:8] are never read - expected, not a bug.
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [3:0]  wstrb,
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire [3:0]  addr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [31:0] wdata,
    /* verilator lint_on UNUSEDSIGNAL */
    output reg  [31:0] rdata,

    output reg         tx
);

    reg  [9:0] shifter;     // {stop, data[7:0], start}
    reg  [3:0] bits_left;
    reg [31:0] baud_cnt;
    wire       busy = (bits_left != 0);

    always @(*) begin
        case (addr)
            4'h4:    rdata = {31'd0, busy};
            default: rdata = 32'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx        <= 1'b1;
            shifter   <= 10'h3FF;
            bits_left <= 4'd0;
            baud_cnt  <= 32'd0;
        end else begin
            if (!busy) begin
                if (sel && wstrb[0] && addr == 4'h0) begin
                    shifter   <= {1'b1, wdata[7:0], 1'b0};  // stop, data, start
                    bits_left <= 4'd10;
                    baud_cnt  <= 32'd0;
                end
            end else begin
                if (baud_cnt == DIV - 1) begin
                    baud_cnt  <= 32'd0;
                    tx        <= shifter[0];
                    shifter   <= {1'b1, shifter[9:1]};
                    bits_left <= bits_left - 1;
                end else begin
                    baud_cnt <= baud_cnt + 1;
                end
            end
        end
    end

endmodule

`default_nettype wire
