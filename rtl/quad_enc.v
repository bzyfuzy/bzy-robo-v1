// =============================================================================
// quad_enc.v - Quadrature encoder decoder
//
// Register map (word offsets from base):
//   0x0  COUNT  32-bit signed position count (read-only)
//   0x4  CTRL   bit0 = CLEAR (write 1 to reset COUNT to 0 on that cycle;
//                             reads back 0)
//
// Design notes:
//   * enc_a / enc_b are asynchronous to clk (real encoder pins), so each is
//     passed through a 2-flop synchronizer before use.
//   * Standard 4x (A+B edge) quadrature decode: the synchronized {A,B} pair
//     forms a 2-bit Gray-coded state; every valid single-step transition
//     (00<->01<->11<->10<->00) advances COUNT by +1 or -1 depending on
//     direction. Any other (e.g. a skipped state from noise) is ignored -
//     COUNT holds rather than corrupts.
// =============================================================================
`default_nettype none

module quad_enc (
    input  wire        clk,
    input  wire        rst_n,

    // simple register bus (one write/read per cycle, from soc_top)
    input  wire        sel,          // this peripheral is addressed
    input  wire [3:0]  wstrb,        // byte write strobes (0 = read)
    input  wire [3:0]  addr,         // word-aligned offset within peripheral
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    input  wire        enc_a,        // raw encoder A phase (async)
    input  wire        enc_b         // raw encoder B phase (async)
);

    // ---- 2FF synchronizers ---------------------------------------------------
    reg a_ff1, a_ff2;
    reg b_ff1, b_ff2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_ff1 <= 1'b0; a_ff2 <= 1'b0;
            b_ff1 <= 1'b0; b_ff2 <= 1'b0;
        end else begin
            a_ff1 <= enc_a; a_ff2 <= a_ff1;
            b_ff1 <= enc_b; b_ff2 <= b_ff1;
        end
    end

    wire [1:0] ab_curr = {a_ff2, b_ff2};

    // ---- transition decode ----------------------------------------------------
    reg [1:0] ab_prev;
    reg signed [31:0] delta;

    always @(*) begin
        case ({ab_prev, ab_curr})
            4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: delta = 32'sd1;   // forward
            4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: delta = -32'sd1;  // reverse
            default:                                delta = 32'sd0;  // no change / skip
        endcase
    end

    // ---- COUNT register --------------------------------------------------------
    reg signed [31:0] count;
    wire clear = sel && (wstrb != 4'b0000) && (addr == 4'h4) && wdata[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ab_prev <= 2'b00;
            count   <= 32'sd0;
        end else begin
            ab_prev <= ab_curr;
            if (clear) count <= 32'sd0;
            else       count <= count + delta;
        end
    end

    // ---- register reads ---------------------------------------------------
    always @(*) begin
        case (addr)
            4'h0:    rdata = count;
            4'h4:    rdata = 32'd0;
            default: rdata = 32'd0;
        endcase
    end

endmodule

`default_nettype wire
