// =============================================================================
// tb_timer.v - standalone unit testbench for timer.v (no CPU, no soc_top).
//
// Coverage:
//   - reset state (CTRL/PERIOD/COUNT/STATUS all 0, irq_pulse low)
//   - period accuracy: irq_pulse fires exactly every PERIOD clocks, checked
//     over 11 consecutive gaps (12 pulses)
//   - IRQ pulse width: exactly one clk wide, every time
//   - STATUS write-1-to-clear: write-0 doesn't clear, write-1 does, and the
//     cleared/set value survives unrelated bus accesses until changed again
//   - disable mid-count: CTRL=0 stops and holds COUNT at 0, no ticks while
//     disabled; re-enable restarts the count from 0 (not a resume)
//   - PERIOD change while running: growing PERIOD extends the current
//     period to the new value; shrinking PERIOD below the current COUNT
//     fires the tick on the very next clock (>= compare, no silent miss),
//     checked with cycle-exact timing
// =============================================================================
`timescale 1ns / 1ps

module tb_timer;

    reg clk = 0;
    always #5 clk = ~clk;

    reg        rst_n;
    reg        sel;
    reg [3:0]  wstrb;
    reg [3:0]  addr;
    reg [31:0] wdata;
    wire [31:0] rdata;
    wire        irq_pulse;

    timer dut (
        .clk       (clk),
        .rst_n     (rst_n),
        .sel       (sel),
        .wstrb     (wstrb),
        .addr      (addr),
        .wdata     (wdata),
        .rdata     (rdata),
        .irq_pulse (irq_pulse)
    );

    integer errors = 0;
    integer checks = 0;

    task check(input integer got, input integer exp, input [799:0] label);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("[TMR ] MISMATCH (%0s): got=%0d expected=%0d", label, got, exp);
            end
        end
    endtask

    // ---- register bus helpers -------------------------------------------
    task write_reg(input [3:0] a, input [31:0] d);
        begin
            @(negedge clk);
            sel = 1'b1; wstrb = 4'hF; addr = a; wdata = d;
            @(posedge clk);            // register write takes effect here
            @(negedge clk);
            sel = 1'b1; wstrb = 4'h0; addr = 4'h0; wdata = 32'h0;
        end
    endtask

    task read_reg(input [3:0] a, output [31:0] d);
        begin
            addr = a;
            #1;
            d = rdata;
        end
    endtask

    reg [31:0] rv;

    // ---- one tick: capture rise time, check pulse width, return rise time
    task capture_tick(output integer t_rise);
        reg [31:0] t_fall;
        begin
            @(posedge irq_pulse);
            t_rise = $time;
            @(negedge irq_pulse);
            if (($time - t_rise) !== 10) begin
                errors = errors + 1;
                $display("[TMR ] MISMATCH (pulse width): width=%0d ns expected=10 ns", $time - t_rise);
            end
            checks = checks + 1;
        end
    endtask

    // ---- same as capture_tick, but bounded: a regression that brings back
    //      the missed-tick behavior must fail cleanly, not hang the sim/CI
    //      forever waiting for an irq_pulse that never comes.
    task capture_tick_bounded(output integer t_rise, input integer max_cycles);
        reg [31:0] t_fall;
        integer timed_out;
        begin
            timed_out = 0;
            fork
                begin
                    @(posedge irq_pulse);
                    t_rise = $time;
                    @(negedge irq_pulse);
                    if (($time - t_rise) !== 10) begin
                        errors = errors + 1;
                        $display("[TMR ] MISMATCH (pulse width): width=%0d ns expected=10 ns", $time - t_rise);
                    end
                    checks = checks + 1;
                end
                begin
                    repeat (max_cycles) @(posedge clk);
                    timed_out = 1;
                end
            join_any
            disable fork;
            if (timed_out) begin
                errors = errors + 1;
                checks = checks + 1;
                t_rise = -999999999;
                $display("[TMR ] MISMATCH (tick timeout): no irq_pulse within %0d cycles", max_cycles);
            end
        end
    endtask

    integer t_prev, t_now, i;
    integer enable_edge_time;

    initial begin
        if ($test$plusargs("trace")) begin
            $dumpfile("tb_timer.vcd");
            $dumpvars(0, tb_timer);
        end

        sel = 1'b1; wstrb = 4'b0; addr = 4'h0; wdata = 32'h0;

        // ---- reset ---------------------------------------------------
        rst_n = 1'b0;
        repeat (4) @(posedge clk);
        @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("[TEST] reset state");
        read_reg(4'h0, rv); check(rv, 0, "CTRL after reset");
        read_reg(4'h4, rv); check(rv, 0, "PERIOD after reset");
        read_reg(4'h8, rv); check(rv, 0, "COUNT after reset");
        read_reg(4'hC, rv); check(rv, 0, "STATUS after reset");
        check(irq_pulse, 0, "irq_pulse after reset");
        repeat (20) @(posedge clk);
        check(irq_pulse, 0, "irq_pulse stays low, idle+disabled");
        read_reg(4'h8, rv); check(rv, 0, "COUNT stays 0, idle+disabled");

        // ---- period accuracy + pulse width, 12 pulses / 11 gaps -------
        $display("[TEST] period accuracy (PERIOD=20, 11 consecutive gaps) + pulse width");
        write_reg(4'h4, 32'd20);
        write_reg(4'h0, 32'd1);
        enable_edge_time = $time - 5;   // write_reg returns 5ns after the effective posedge

        capture_tick(t_prev);
        check(t_prev - enable_edge_time, 20 * 10, "first tick offset from enable");
        for (i = 0; i < 11; i = i + 1) begin
            capture_tick(t_now);
            check(t_now - t_prev, 20 * 10, "inter-tick period spacing");
            t_prev = t_now;
        end

        // ---- STATUS write-1-to-clear -----------------------------------
        $display("[TEST] STATUS write-1-to-clear semantics");
        read_reg(4'hC, rv); check(rv, 1, "STATUS set after a tick");
        write_reg(4'hC, 32'h0);                 // write 0: must NOT clear
        read_reg(4'hC, rv); check(rv, 1, "STATUS unaffected by write-0");
        read_reg(4'h8, rv);                      // unrelated read
        read_reg(4'h0, rv);                      // unrelated read
        read_reg(4'hC, rv); check(rv, 1, "STATUS survives unrelated accesses");
        write_reg(4'hC, 32'h1);                 // write 1: clears
        read_reg(4'hC, rv); check(rv, 0, "STATUS cleared by write-1");
        // cleared value holds until the next tick re-sets it (well under
        // one period away, so no tick can have snuck in)
        repeat (5) @(posedge clk);
        read_reg(4'hC, rv); check(rv, 0, "STATUS stays cleared, no spontaneous re-set");
        capture_tick(t_now);                     // let the next natural tick land
        read_reg(4'hC, rv); check(rv, 1, "STATUS re-set by the next tick");
        write_reg(4'hC, 32'h1);                 // clear again, tidy state for next section

        // ---- disable mid-count + re-enable -----------------------------
        $display("[TEST] disable mid-count, then re-enable restarts from 0");
        write_reg(4'h0, 32'd0);                 // disable
        repeat (2) @(posedge clk);
        read_reg(4'h8, rv); check(rv, 0, "COUNT clears on disable");

        write_reg(4'h4, 32'd30);
        write_reg(4'h0, 32'd1);                 // enable, fresh PERIOD=30
        enable_edge_time = $time - 5;

        // let it count partway, then disable mid-count
        repeat (10) @(posedge clk);
        read_reg(4'h8, rv); check(rv, 10, "COUNT mid-run before disable");
        write_reg(4'h0, 32'd0);                 // disable mid-count
        repeat (2) @(posedge clk);
        read_reg(4'h8, rv); check(rv, 0, "COUNT clears immediately on mid-count disable");

        // stay disabled well past where the original period would have
        // ticked - must not fire
        repeat (100) @(posedge clk);
        check(irq_pulse, 0, "no tick while disabled, even past the old period");
        read_reg(4'h8, rv); check(rv, 0, "COUNT held at 0 throughout disable");

        // re-enable: must restart from 0, i.e. the next tick is a full
        // PERIOD away, not "resume" from the count=10 it was at
        write_reg(4'h0, 32'd1);
        enable_edge_time = $time - 5;
        capture_tick(t_now);
        check(t_now - enable_edge_time, 30 * 10, "re-enable restarts count from 0 (full period, not a resume)");

        // ---- PERIOD change while running -------------------------------
        $display("[TEST] PERIOD change while running: grow");
        write_reg(4'h0, 32'd0);                 // clean disable/re-enable
        repeat (2) @(posedge clk);
        write_reg(4'h4, 32'd40);
        write_reg(4'h0, 32'd1);
        enable_edge_time = $time - 5;
        repeat (10) @(posedge clk);
        read_reg(4'h8, rv); check(rv, 10, "COUNT before growing PERIOD");
        write_reg(4'h4, 32'd60);                // grow PERIOD mid-count
        capture_tick(t_now);
        check(t_now - enable_edge_time, 60 * 10,
              "growing PERIOD mid-count extends to the new value (live compare, not double-buffered)");

        $display("[TEST] PERIOD change while running: shrink below current COUNT fires immediately");
        write_reg(4'h0, 32'd0);
        repeat (2) @(posedge clk);
        write_reg(4'h4, 32'd40);
        write_reg(4'h0, 32'd1);
        repeat (20) @(posedge clk);
        read_reg(4'h8, rv); check(rv, 20, "COUNT before shrinking PERIOD below it");
        write_reg(4'hC, 32'h1);                  // clear any STATUS left from earlier sections'
                                                  // ticks, so the check below proves a fresh baseline
        // period-1=9 <= COUNT (21 by this write's own effective edge - the
        // counter keeps advancing under the OLD period for that one edge,
        // same CTRL/PERIOD-write visibility lag measured earlier in this
        // file) - the >= compare must fire the tick on the very next clock,
        // not miss it.
        write_reg(4'h4, 32'd10);
        enable_edge_time = $time - 5;
        capture_tick_bounded(t_now, 20);   // bounded: a regression to the old
                                            // missed-tick behavior must fail,
                                            // not hang waiting forever
        check(t_now - enable_edge_time, 1 * 10,
              "shrinking PERIOD below COUNT fires the tick on the very next clock (>= compare, no silent miss)");
        read_reg(4'hC, rv); check(rv, 1, "STATUS set - the tick really fired, not silently skipped");
        write_reg(4'h0, 32'd0);                  // disable, clean state

        $display("");
        $display("[TMR ] checks=%0d errors=%0d", checks, errors);
        if (errors == 0)
            $display("RESULT: PASS  (checks=%0d, errors=%0d)", checks, errors);
        else
            $display("RESULT: FAIL  (checks=%0d, errors=%0d)", checks, errors);
        $finish;
    end

endmodule
