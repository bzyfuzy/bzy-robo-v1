// =============================================================================
// tb_quad_enc.v - standalone unit testbench for quad_enc.v (no CPU, no soc_top).
//
// Coverage:
//   - COUNT == 0 immediately out of reset, and stays 0 while idle
//   - contact bounce: enc_a toggled every single clock (faster than the 2FF
//     synchronizer's settle time) with enc_b held fixed; the running count
//     must never drift outside the true physical excursion and must return
//     to its pre-bounce value once the bounce settles
//   - all 8 legal single-step Gray transitions, both directions
//   - all 4 illegal two-step ("skip") transitions - count must hold
//   - CTRL clear
//   - signed overflow/underflow at the 0x7FFFFFFF / 0x80000000 boundary
//   - mid-run reset clears count regardless of prior state
// =============================================================================
`timescale 1ns / 1ps

module tb_quad_enc;

    reg clk = 0;
    always #5 clk = ~clk;

    reg        rst_n;
    reg        sel;
    reg [3:0]  wstrb;
    reg [3:0]  addr;
    reg [31:0] wdata;
    wire [31:0] rdata;
    reg        enc_a;
    reg        enc_b;

    quad_enc dut (
        .clk   (clk),
        .rst_n (rst_n),
        .sel   (sel),
        .wstrb (wstrb),
        .addr  (addr),
        .wdata (wdata),
        .rdata (rdata),
        .enc_a (enc_a),
        .enc_b (enc_b)
    );

    integer errors        = 0;
    integer checks         = 0;
    integer expected_count = 0;
    integer base_count;

    // ---- checks COUNT (word 0) against a running expected total -------------
    task check_count(input integer exp);
        begin
            checks = checks + 1;
            if ($signed(rdata) !== exp) begin
                errors = errors + 1;
                $display("[ENC ] MISMATCH: count=%0d expected=%0d", $signed(rdata), exp);
            end
        end
    endtask

    // ---- drives one Gray-code input change, waits full 2FF+decode settle,
    //      then checks COUNT against the running expected total. Used for
    //      both legal steps (delta = +-1) and illegal skips (delta = 0). ----
    task step(input a, input b, input integer delta);
        begin
            enc_a = a; enc_b = b;
            repeat (8) @(posedge clk);   // >>3 cycles needed for sync+decode
            expected_count = expected_count + delta;
            check_count(expected_count);
        end
    endtask

    // ---- one clock-cycle bounce of enc_a; count may only move within
    //      [baseline-1, baseline] - it must never run away. ------------------
    task bounce_toggle(input a, input integer baseline);
        begin
            enc_a = a;
            @(posedge clk);
            #1;
            if (($signed(rdata) > baseline) || ($signed(rdata) < baseline - 1)) begin
                errors = errors + 1;
                $display("[ENC ] MISMATCH: bounce runaway count=%0d baseline=%0d",
                          $signed(rdata), baseline);
            end
        end
    endtask

    // ---- pulses CTRL bit0 for exactly one clock -------------------------------
    task pulse_clear;
        begin
            @(negedge clk);
            sel = 1'b1; wstrb = 4'b1111; addr = 4'h4; wdata = 32'h1;
            @(posedge clk);
            @(negedge clk);
            sel = 1'b1; wstrb = 4'b0000; addr = 4'h0; wdata = 32'h0;
        end
    endtask

    // ---- whitebox poke of the internal COUNT reg, only to reach the signed
    //      overflow boundary without 2^31 real transitions. Legal in a unit tb
    //      (CLAUDE.md Rule 2 allows hierarchical peeks/pokes here). -----------
    task force_count(input integer val);
        begin
            @(negedge clk);
            dut.count = val;
        end
    endtask

    initial begin
        if ($test$plusargs("trace")) begin
            $dumpfile("tb_quad_enc.vcd");
            $dumpvars(0, tb_quad_enc);
        end

        sel = 1'b1; wstrb = 4'b0000; addr = 4'h0; wdata = 32'h0;
        enc_a = 1'b0; enc_b = 1'b0;

        // ---- reset -------------------------------------------------------
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("[TEST] reset: count out of reset");
        expected_count = 0;
        check_count(0);
        repeat (8) @(posedge clk);
        check_count(0);   // stays 0 while idle - no spurious counts out of reset

        // ---- contact bounce (state 00, enc_b held low) --------------------
        $display("[TEST] contact bounce (rapid single-cycle A oscillation)");
        base_count = expected_count;
        bounce_toggle(1'b1, base_count);
        bounce_toggle(1'b0, base_count);
        bounce_toggle(1'b1, base_count);
        bounce_toggle(1'b0, base_count);
        bounce_toggle(1'b1, base_count);
        bounce_toggle(1'b0, base_count);
        repeat (8) @(posedge clk);        // let the pipeline fully settle
        check_count(base_count);          // net movement must be zero
        $display("[ENC ] bounce settled: count=%0d baseline=%0d", $signed(rdata), base_count);

        // ---- all 8 legal Gray transitions, both directions ----------------
        $display("[TEST] all 8 legal Gray-code transitions (forward then reverse)");
        step(1'b0, 1'b1,  1);   // 00 -> 01  forward
        step(1'b1, 1'b1,  1);   // 01 -> 11  forward
        step(1'b1, 1'b0,  1);   // 11 -> 10  forward
        step(1'b0, 1'b0,  1);   // 10 -> 00  forward
        step(1'b1, 1'b0, -1);   // 00 -> 10  reverse
        step(1'b1, 1'b1, -1);   // 10 -> 11  reverse
        step(1'b0, 1'b1, -1);   // 11 -> 01  reverse
        step(1'b0, 1'b0, -1);   // 01 -> 00  reverse

        // ---- illegal double-step (skip) transitions -----------------------
        $display("[TEST] illegal double-step (skip) transitions - count must hold");
        step(1'b1, 1'b1, 0);    // 00 -> 11  skip
        step(1'b0, 1'b0, 0);    // 11 -> 00  skip
        step(1'b0, 1'b1, 1);    // 00 -> 01  legal (detour to reach 01)
        step(1'b1, 1'b0, 0);    // 01 -> 10  skip
        step(1'b0, 1'b1, 0);    // 10 -> 01  skip
        step(1'b0, 1'b0, -1);   // 01 -> 00  legal (return to 00)

        // ---- CTRL clear ----------------------------------------------------
        $display("[TEST] clear via CTRL");
        step(1'b0, 1'b1, 1);    // 00 -> 01
        step(1'b1, 1'b1, 1);    // 01 -> 11   (count now 2)
        pulse_clear;
        expected_count = 0;
        repeat (2) @(posedge clk);
        check_count(0);

        // ---- signed overflow / underflow at the 32-bit boundary -----------
        $display("[TEST] signed overflow at 0x7FFFFFFF / 0x80000000 boundary");
        force_count(32'sh7fffffff);
        expected_count = 32'sh7fffffff;
        repeat (2) @(posedge clk);
        check_count(expected_count);
        step(1'b1, 1'b0,  1);   // 11 -> 10 forward: 0x7FFFFFFF + 1 wraps to 0x80000000
        step(1'b1, 1'b1, -1);   // 10 -> 11 reverse: 0x80000000 - 1 wraps to 0x7FFFFFFF

        // ---- mid-run reset clears count regardless of prior state ---------
        $display("[TEST] mid-run reset clears count regardless of prior state");
        enc_a = 1'b0; enc_b = 1'b0;
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);
        expected_count = 0;
        check_count(0);

        $display("");
        $display("[ENC ] checks=%0d errors=%0d", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS  (checks=%0d, errors=%0d)", checks, errors);
        else
            $display("RESULT: FAIL  (checks=%0d, errors=%0d)", checks, errors);
        $finish;
    end

endmodule
