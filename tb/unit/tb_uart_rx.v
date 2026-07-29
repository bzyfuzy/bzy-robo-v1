// =============================================================================
// tb_uart_rx.v - standalone unit testbench for uart_rx.v (no CPU, no soc_top).
//
// Coverage:
//   - reset state (STATUS==0, DATA read returns 0)
//   - single byte: all-zeros, all-ones, alternating pattern
//   - back-to-back bytes at full line rate (zero idle gap between frames)
//   - FIFO fill to exactly 8, then a 9th overflows: sticky OVERFLOW set, the
//     9th byte dropped, the first 8 bytes intact and in order, W1C clears it
//   - framing error (stop bit sampled low): sticky FERR set, byte discarded
//   - glitch shorter than half a bit: must never produce a queued byte
//   - a byte arriving while the CPU pops already-queued bytes concurrently
//   - mid-run reset clears everything regardless of prior state
//
// Mutation-tested: breaking the start-bit center check (sampling at tick 0
// instead of 8) is caught by the glitch-rejection test - see the bottom of
// this file's companion mutation run (not included here; see CLAUDE.md).
// =============================================================================
`timescale 1ns / 1ps

module tb_uart_rx;

    localparam OVERSAMPLE_DIV = 2;
    localparam BIT_PERIOD = OVERSAMPLE_DIV * 16;   // 32 clk cycles/bit

    reg clk = 0;
    always #5 clk = ~clk;

    reg        rst_n;
    reg        sel;
    reg [3:0]  wstrb;
    reg [3:0]  addr;
    reg [31:0] wdata;
    wire [31:0] rdata;
    reg        bus_ready;
    reg        rx;

    uart_rx #(.OVERSAMPLE_DIV(OVERSAMPLE_DIV)) dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .sel       (sel),
        .wstrb     (wstrb),
        .addr      (addr),
        .wdata     (wdata),
        .rdata     (rdata),
        .bus_ready (bus_ready),
        .rx        (rx)
    );

    integer errors = 0;
    integer checks = 0;

    task check(input integer got, input integer exp, input [799:0] label);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("[URX ] MISMATCH (%0s): got=%0d expected=%0d", label, got, exp);
            end
        end
    endtask

    // ---- serial line driver ---------------------------------------------------
    task send_byte(input [7:0] b);
        integer bi;
        begin
            rx = 1'b0;                       // start bit
            repeat (BIT_PERIOD) @(posedge clk);
            for (bi = 0; bi < 8; bi = bi + 1) begin
                rx = b[bi];                   // LSB first
                repeat (BIT_PERIOD) @(posedge clk);
            end
            rx = 1'b1;                       // stop bit
            repeat (BIT_PERIOD) @(posedge clk);
        end
    endtask

    task send_byte_bad_stop(input [7:0] b);
        integer bi;
        begin
            rx = 1'b0;
            repeat (BIT_PERIOD) @(posedge clk);
            for (bi = 0; bi < 8; bi = bi + 1) begin
                rx = b[bi];
                repeat (BIT_PERIOD) @(posedge clk);
            end
            rx = 1'b0;                       // bad stop bit (framing error)
            repeat (BIT_PERIOD) @(posedge clk);
            rx = 1'b1;                       // return to idle
            repeat (BIT_PERIOD) @(posedge clk);
        end
    endtask

    // ---- register bus helpers -------------------------------------------------
    task read_data(output [31:0] val);
        begin
            @(negedge clk);
            sel = 1'b1; wstrb = 4'b0; addr = 4'h0; bus_ready = 1'b1;
            #1;
            val = rdata;
            @(posedge clk);      // pop (if any) takes effect here
            @(negedge clk);
            sel = 1'b1; wstrb = 4'b0; addr = 4'h0; bus_ready = 1'b0;
        end
    endtask

    task read_status(output [31:0] val);
        begin
            @(negedge clk);
            sel = 1'b1; wstrb = 4'b0; addr = 4'h4; bus_ready = 1'b0;
            #1;
            val = rdata;
        end
    endtask

    task write_status(input [31:0] val);
        begin
            @(negedge clk);
            sel = 1'b1; wstrb = 4'hF; addr = 4'h4; wdata = val; bus_ready = 1'b0;
            @(posedge clk);
            @(negedge clk);
            sel = 1'b1; wstrb = 4'b0; addr = 4'h0; wdata = 32'h0;
        end
    endtask

    reg [31:0] rv;
    integer i;

    initial begin
        if ($test$plusargs("trace")) begin
            $dumpfile("tb_uart_rx.vcd");
            $dumpvars(0, tb_uart_rx);
        end

        sel = 1'b1; wstrb = 4'b0; addr = 4'h0; wdata = 32'h0;
        bus_ready = 1'b0; rx = 1'b1;

        // ---- reset ---------------------------------------------------
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("[TEST] reset state");
        read_status(rv); check(rv, 0, "STATUS after reset");
        read_data(rv);   check(rv, 0, "DATA after reset (empty)");

        // ---- single byte: all-zeros, all-ones, alternating -----------------
        $display("[TEST] single byte: 0x00, 0xFF, 0x55, 0xAA");
        send_byte(8'h00);
        read_status(rv); check(rv[0], 1, "data_available after 0x00");
        read_data(rv);   check(rv, 'h00, "popped byte == 0x00");
        read_status(rv); check(rv[0], 0, "data_available empty after pop");

        send_byte(8'hFF);
        read_status(rv); check(rv[0], 1, "data_available after 0xFF");
        read_data(rv);   check(rv, 'hFF, "popped byte == 0xFF");

        send_byte(8'h55);
        read_data(rv);   check(rv, 'h55, "popped byte == 0x55");

        send_byte(8'hAA);
        read_data(rv);   check(rv, 'hAA, "popped byte == 0xAA");
        read_status(rv); check(rv[0], 0, "data_available empty after all pops");

        // ---- back-to-back bytes at full line rate --------------------------
        $display("[TEST] back-to-back bytes, zero idle gap");
        send_byte(8'h11);
        send_byte(8'h22);
        send_byte(8'h33);
        send_byte(8'h44);
        read_status(rv); check(rv[0], 1, "data_available after 4 back-to-back bytes");
        read_data(rv); check(rv, 'h11, "back-to-back byte 1");
        read_data(rv); check(rv, 'h22, "back-to-back byte 2");
        read_data(rv); check(rv, 'h33, "back-to-back byte 3");
        read_data(rv); check(rv, 'h44, "back-to-back byte 4");
        read_status(rv); check(rv[0], 0, "data_available empty after draining back-to-back bytes");

        // ---- FIFO fill to exactly 8, then overflow -------------------------
        $display("[TEST] FIFO fill to 8, 9th overflows");
        for (i = 0; i < 8; i = i + 1) send_byte(i[7:0]);
        read_status(rv);
        check(rv[0], 1, "data_available with FIFO full");
        check(rv[1], 0, "overflow not yet set at exactly 8 bytes");
        send_byte(8'h63);   // 9th byte - must be dropped
        read_status(rv);
        check(rv[1], 1, "overflow sticky set after 9th byte");
        for (i = 0; i < 8; i = i + 1) begin
            read_data(rv);
            check(rv, i, "overflow test: original queued byte, in order, undisturbed");
        end
        read_status(rv); check(rv[0], 0, "FIFO empty after draining all 8 (9th truly dropped)");
        write_status(32'h2);   // W1C bit1 (overflow)
        read_status(rv); check(rv[1], 0, "overflow clears after W1C");

        // ---- framing error --------------------------------------------------
        $display("[TEST] framing error (low stop bit)");
        send_byte_bad_stop(8'h77);
        read_status(rv);
        check(rv[2], 1, "FERR sticky set after bad stop bit");
        check(rv[0], 0, "byte with framing error is discarded, not queued");
        write_status(32'h4);   // W1C bit2 (FERR)
        read_status(rv); check(rv[2], 0, "FERR clears after W1C");

        // ---- glitch shorter than half a bit ----------------------------------
        $display("[TEST] glitch shorter than half a bit must not produce a byte");
        rx = 1'b0;
        repeat (4 * OVERSAMPLE_DIV) @(posedge clk);   // 4 sample ticks < 8 (half bit)
        rx = 1'b1;
        // Wait past a full 10-bit-period frame (not just a couple of bit
        // periods): a buggy implementation that mis-times the start-bit
        // center check can still go on to construct a spurious byte from
        // the glitch, and that construction takes a full frame to finish -
        // checking too early would miss it (caught the hard way: an
        // earlier, too-short wait here let a real mutation slip past this
        // test and only get caught by cross-test contamination downstream).
        repeat (12 * BIT_PERIOD) @(posedge clk);
        read_status(rv);
        check(rv[0], 0, "no byte produced by a sub-half-bit glitch");
        check(rv[2], 0, "glitch does not raise a framing error either");

        // ---- byte arriving while the CPU pops mid-stream ---------------------
        $display("[TEST] byte arrives while CPU concurrently pops queued bytes");
        send_byte(8'hA1);
        send_byte(8'hA2);
        send_byte(8'hA3);
        fork
            begin
                send_byte(8'hA4);   // 4th byte, arriving concurrently with the pops below
            end
            begin
                read_data(rv); check(rv, 'hA1, "concurrent: byte 1 popped correctly while byte 4 arrives");
                read_data(rv); check(rv, 'hA2, "concurrent: byte 2 popped correctly while byte 4 arrives");
            end
        join
        read_data(rv); check(rv, 'hA3, "concurrent: byte 3 (queued before) popped after the join");
        read_data(rv); check(rv, 'hA4, "concurrent: byte 4 (arrived during pops) queued correctly");
        read_status(rv); check(rv[0], 0, "FIFO empty after draining the concurrent test");

        // ---- mid-run reset clears everything regardless of prior state ------
        $display("[TEST] mid-run reset clears FIFO/STATUS regardless of prior state");
        send_byte(8'h5A);
        read_status(rv); check(rv[0], 1, "data_available set just before reset");
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);
        read_status(rv); check(rv, 0, "STATUS clears on mid-run reset");
        read_data(rv);   check(rv, 0, "DATA reads 0, FIFO empty after mid-run reset");

        $display("");
        $display("[URX ] checks=%0d errors=%0d", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS  (checks=%0d, errors=%0d)", checks, errors);
        else
            $display("RESULT: FAIL  (checks=%0d, errors=%0d)", checks, errors);
        $finish;
    end

endmodule
