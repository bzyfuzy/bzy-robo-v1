// =============================================================================
// rst_sync.v - reset synchronizer
//
// Async assert, 2FF synchronous de-assert: rst_n_out drops the instant
// rst_n_in drops (no clock needed - safe for actuator outputs downstream),
// and only rises again after two clean clk edges once rst_n_in returns
// high, filtering out any metastability from the external reset's
// asynchronous release edge.
//
// The raw external reset should feed only this module; every other
// register in the design (soc_top's own bus logic, every peripheral, and
// the CPU) resets from rst_n_out instead.
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module rst_sync (
    input  wire clk,
    input  wire rst_n_in,    // raw, asynchronous external reset (active low)
    output reg  rst_n_out     // synchronized reset (active low)
);

    reg meta;

    always @(posedge clk or negedge rst_n_in) begin
        if (!rst_n_in) begin
            meta      <= 1'b0;
            rst_n_out <= 1'b0;
        end else begin
            meta      <= 1'b1;
            rst_n_out <= meta;
        end
    end

endmodule

`default_nettype wire
