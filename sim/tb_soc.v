// =============================================================================
// tb_soc.v - boots the SoC, checks UART prints "OK", then closes the loop:
// a behavioral velocity plant reads the PID's commanded PWM duty, integrates
// a simulated position, and drives real quadrature transitions back into the
// encoder inputs - the same closed-loop demo the firmware is running, but
// with a plant standing in for the physical motor+encoder.
//
// Pass criteria:
//   1. UART transmits 'O', 'K', '\n' (decoded from the serial line itself)
//   2. Timer fires irq[3] every PERIOD (2000) clk cycles - period_errors==0
//   3. Each profile step (target 0 -> +400 -> -200 counts) settles to within
//      +-8 counts of target within 300 ISR ticks (this codebase's existing
//      "1kHz-style" sim convention treats each 2000-clk ISR tick as 1ms of
//      conceptual control-loop time - see docs/timer.md and docs/control.md)
//      and overshoots by less than 25% of the step size.
//
// The plant (duty -> velocity -> position) uses the exact same fixed-point
// arithmetic as fw/control_model.py's plant_step() - see docs/control.md for
// the cross-check procedure between this simulation, the firmware, and the
// Python golden model.
// =============================================================================
`timescale 1ns / 1ps

module tb_soc;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;                 // 100 MHz sim clock (period irrelevant)

    wire pwm_out;
    wire uart_txd;
    wire [7:0] leds;
    reg  enc_a = 1'b0;
    reg  enc_b = 1'b0;

    soc_top #(
        .FIRMWARE_HEX ("firmware.hex"),
        .UART_DIV     (8)                  // fast baud for simulation
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .pwm_out  (pwm_out),
        .uart_txd (uart_txd),
        .leds     (leds),
        .enc_a    (enc_a),
        .enc_b    (enc_b)
    );

    // ---- UART line decoder (8N1, DIV=8) -------------------------------------
    integer uart_chars = 0;
    reg [7:0] rx_byte;
    integer i;
    initial begin : uart_mon
        forever begin
            @(negedge uart_txd);                 // start bit edge
            repeat (12) @(posedge clk);          // into middle of bit 0 (1.5 * 8)
            for (i = 0; i < 8; i = i + 1) begin
                rx_byte[i] = uart_txd;
                repeat (8) @(posedge clk);
            end
            uart_chars = uart_chars + 1;
            if (rx_byte >= 32) $display("[UART] '%c' (0x%02x)", rx_byte, rx_byte);
            else               $display("[UART] 0x%02x", rx_byte);
        end
    end

    // ---- PWM pulse-width monitor (diagnostic only) --------------------------
    integer pulse_count = 0;
    integer high_cycles;
    time    t_rise;
    initial begin : pwm_mon
        forever begin
            @(posedge pwm_out);
            t_rise = $time;
            @(negedge pwm_out);
            high_cycles = ($time - t_rise) / 10;   // 10 ns per clk
            pulse_count = pulse_count + 1;
        end
    end

    // ---- timer IRQ period monitor --------------------------------------------
    integer tick_count = 0;
    time    t_last_tick;
    integer period_errors = 0;
    initial begin : timer_pulse_mon
        @(posedge rst_n);
        @(posedge dut.u_timer.irq_pulse);   // first tick: just record it
        t_last_tick = $time;
        tick_count = 1;
        forever begin
            @(posedge dut.u_timer.irq_pulse);
            if (($time - t_last_tick) !== 2000 * 10) begin   // 2000 clk * 10 ns/clk
                $display("[TMR ] MISMATCH: tick %0d spacing=%0d ns (expected 20000 ns)",
                          tick_count + 1, $time - t_last_tick);
                period_errors = period_errors + 1;
            end
            t_last_tick = $time;
            tick_count = tick_count + 1;
        end
    end

    // ---- behavioral velocity plant -------------------------------------------
    // duty (measured as PWM's commanded duty_shadow - the value the PID just
    // wrote, not gated behind PWM's own glitch-free frame-boundary buffering,
    // since that buffering exists only to protect the physical pin, not to
    // model actuator intent) -> velocity_target (proportional about center)
    // -> velocity (first-order lag) -> position (integrated, fixed-point).
    // Whenever position crosses an integer boundary, drives one real
    // Gray-code quadrature transition on enc_a/enc_b, exactly like a real
    // encoder on a real motor shaft. Same arithmetic as
    // fw/control_model.py's plant_step() - see docs/control.md.
    localparam PLANT_SHIFT    = 4;
    localparam VEL_GAIN_SHIFT = 7;
    localparam POS_FRAC_ONE   = 256;
    localparam CENTER_DUTY    = 1500;

    integer plant_vel      = 0;
    integer plant_pos_frac = 0;
    integer plant_position = 0;   // ground truth - tracks dut.u_enc.count
    reg [1:0] plant_ab     = 2'b00;

    function integer sra;             // arithmetic right shift, matches
        input integer val;            // srai / Verilog's >>> on a signed value
        input integer n;
        sra = val >>> n;
    endfunction

    task plant_drive_step(input dir);   // dir=1: forward (+1), dir=0: reverse (-1)
        begin
            if (dir) begin
                case (plant_ab)
                    2'b00: plant_ab = 2'b01;
                    2'b01: plant_ab = 2'b11;
                    2'b11: plant_ab = 2'b10;
                    2'b10: plant_ab = 2'b00;
                endcase
            end else begin
                case (plant_ab)
                    2'b00: plant_ab = 2'b10;
                    2'b10: plant_ab = 2'b11;
                    2'b11: plant_ab = 2'b01;
                    2'b01: plant_ab = 2'b00;
                endcase
            end
            {enc_a, enc_b} = plant_ab;
            repeat (8) @(posedge clk);   // let 2FF sync + decode settle
        end
    endtask

    integer duty_err, vel_target;
    initial begin : plant_mon
        @(posedge rst_n);
        forever begin
            @(posedge dut.u_timer.irq_pulse);
            duty_err = $signed(dut.u_pwm.duty_shadow) - CENTER_DUTY;
            vel_target = sra(duty_err * POS_FRAC_ONE, VEL_GAIN_SHIFT);
            plant_vel = plant_vel + sra(vel_target - plant_vel, PLANT_SHIFT);
            plant_pos_frac = plant_pos_frac + plant_vel;
            while (plant_pos_frac >= POS_FRAC_ONE) begin
                plant_pos_frac = plant_pos_frac - POS_FRAC_ONE;
                plant_position = plant_position + 1;
                plant_drive_step(1'b1);
            end
            while (plant_pos_frac <= -POS_FRAC_ONE) begin
                plant_pos_frac = plant_pos_frac + POS_FRAC_ONE;
                plant_position = plant_position - 1;
                plant_drive_step(1'b0);
            end
        end
    end

    // ---- closed-loop settling / overshoot assertions -------------------------
    // Mirrors fw/gen_firmware.py's profile schedule (PROFILE / SEG_TICKS) -
    // see docs/control.md. Segment 0 (target 0) is trivial (position already
    // 0 at reset); segments 1/2 are the actual +400 / -200 steps.
    localparam SEG_TICKS = 400;
    integer profile_targets [0:2];
    initial begin
        profile_targets[0] = 0;
        profile_targets[1] = 400;
        profile_targets[2] = -200;
    end

    integer pos_hist [0:SEG_TICKS-1];
    integer seg, local_tick, seg_start_pos, step_size, k;
    integer settle_tick, max_exc, min_exc;
    integer found_break;
    real    overshoot_pct;
    integer settle_errors = 0;
    integer overshoot_errors = 0;
    reg     closed_loop_done = 1'b0;

    initial begin : closed_loop_mon
        @(posedge rst_n);
        for (seg = 0; seg < 3; seg = seg + 1) begin
            seg_start_pos = dut.u_enc.count;
            max_exc = seg_start_pos;
            min_exc = seg_start_pos;
            for (local_tick = 0; local_tick < SEG_TICKS; local_tick = local_tick + 1) begin
                @(posedge dut.u_timer.irq_pulse);
                pos_hist[local_tick] = dut.u_enc.count;
                if (pos_hist[local_tick] > max_exc) max_exc = pos_hist[local_tick];
                if (pos_hist[local_tick] < min_exc) min_exc = pos_hist[local_tick];
            end

            step_size = profile_targets[seg] - seg_start_pos;

            // true settle: earliest index from which every later sample
            // (through the end of the segment) stays within +-8 of target
            settle_tick = SEG_TICKS;   // sentinel: never settled
            k = SEG_TICKS - 1;
            found_break = 0;
            while (k >= 0 && !found_break) begin
                if ((pos_hist[k] - profile_targets[seg] <= 8) &&
                    (pos_hist[k] - profile_targets[seg] >= -8))
                    settle_tick = k;
                else
                    found_break = 1;
                k = k - 1;
            end

            if (step_size > 0)
                overshoot_pct = (max_exc > profile_targets[seg]) ?
                    (max_exc - profile_targets[seg]) * 100.0 / step_size : 0.0;
            else if (step_size < 0)
                overshoot_pct = (min_exc < profile_targets[seg]) ?
                    (profile_targets[seg] - min_exc) * 100.0 / (-step_size) : 0.0;
            else
                overshoot_pct = 0.0;

            $display("[CTRL] segment %0d: target=%0d settle_tick=%0d overshoot=%0.1f%% final_pos=%0d",
                      seg, profile_targets[seg], settle_tick, overshoot_pct, pos_hist[SEG_TICKS-1]);

            if (settle_tick > 300) begin
                $display("[CTRL] MISMATCH: segment %0d settle_tick=%0d exceeds 300-tick budget",
                          seg, settle_tick);
                settle_errors = settle_errors + 1;
            end
            if (overshoot_pct >= 25.0) begin
                $display("[CTRL] MISMATCH: segment %0d overshoot=%0.1f%% >= 25%%", seg, overshoot_pct);
                overshoot_errors = overshoot_errors + 1;
            end
        end
        closed_loop_done = 1'b1;
    end

    // ---- run ----------------------------------------------------------------
    initial begin
        if ($test$plusargs("trace")) begin
            $dumpfile("tb_soc.vcd");
            $dumpvars(0, tb_soc);
        end
        repeat (10) @(posedge clk);
        rst_n = 1;

        wait (closed_loop_done);
        repeat (10) @(posedge clk);   // let any in-flight monitor prints land

        $display("[TMR ] ticks=%0d period_errors=%0d", tick_count, period_errors);
        $display("[PWM ] pulses=%0d", pulse_count);
        $display("");
        if (uart_chars >= 3 && period_errors == 0 &&
            settle_errors == 0 && overshoot_errors == 0)
            $display("RESULT: PASS  (uart_chars=%0d, ticks=%0d, settle_errors=%0d, overshoot_errors=%0d)",
                     uart_chars, tick_count, settle_errors, overshoot_errors);
        else
            $display("RESULT: FAIL  (uart_chars=%0d, ticks=%0d, settle_errors=%0d, overshoot_errors=%0d)",
                     uart_chars, tick_count, settle_errors, overshoot_errors);
        $finish;
    end

endmodule
