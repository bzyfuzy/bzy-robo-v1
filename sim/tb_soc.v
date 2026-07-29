// =============================================================================
// tb_soc.v - boots the SoC, checks UART prints "OK", then becomes the user:
// a behavioral velocity plant closes the position loop (duty -> lag ->
// velocity -> integrated position -> real quadrature transitions back into
// the encoder inputs), and once the autonomous profile's first segment has
// settled, this testbench types real serial commands down the RX line at
// realistic baud - "T+300\n", "T-150\n", and one malformed "Tx9\n" - and
// verifies the firmware's command parser (docs/control.md) does the right
// thing in each case.
//
// Pass criteria:
//   1. UART transmits 'O', 'K', '\n' (decoded from the serial line itself)
//   2. Timer fires irq[3] every PERIOD (2000) clk cycles - period_errors==0
//   3. "T+300\n" is accepted ("ok" echoed) and the loop chases to +300 within
//      the 300-ISR-tick settle budget (+-8 counts) and <25% overshoot - same
//      budgets as the closed-loop demo's autonomous phase (docs/control.md).
//   4. "T-150\n" likewise, chasing from +300 to -150.
//   5. "Tx9\n" is rejected ("?" echoed) and the target does NOT change - the
//      loop stays settled at -150, it doesn't move.
//
// The plant (duty -> velocity -> position) uses the exact same fixed-point
// arithmetic as fw/control_model.py's plant_step() - see docs/control.md for
// the cross-check procedure between this simulation, the firmware, and the
// Python golden model.
// =============================================================================
`timescale 1ns / 1ps

module tb_soc;

    localparam UART_DIV = 32;                  // shared "baud" for TX and RX
    localparam BIT_PERIOD = UART_DIV;           // clk cycles/bit (10ns clk)

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;                 // 100 MHz sim clock (period irrelevant)

    wire pwm_out;
    wire uart_txd;
    wire [7:0] leds;
    reg  enc_a = 1'b0;
    reg  enc_b = 1'b0;
    reg  uart_rxd = 1'b1;                 // idle high

    soc_top #(
        .FIRMWARE_HEX ("firmware.hex"),
        .UART_DIV     (UART_DIV),
        .UART_RX_OVERSAMPLE_DIV (UART_DIV / 16)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .pwm_out  (pwm_out),
        .uart_txd (uart_txd),
        .leds     (leds),
        .enc_a    (enc_a),
        .enc_b    (enc_b),
        .uart_rxd (uart_rxd)
    );

    // ---- UART TX line decoder (8N1) + command-response line tracker --------
    integer uart_chars = 0;
    reg [7:0] rx_byte;
    integer i;
    reg [7:0] line_buf [0:63];
    integer line_len = 0;
    integer ok_count = 0;
    integer q_count = 0;
    initial begin : uart_mon
        forever begin
            @(negedge uart_txd);                        // start bit edge
            repeat (BIT_PERIOD + BIT_PERIOD/2) @(posedge clk);   // into bit 0 center
            for (i = 0; i < 8; i = i + 1) begin
                rx_byte[i] = uart_txd;
                repeat (BIT_PERIOD) @(posedge clk);
            end
            uart_chars = uart_chars + 1;
            if (rx_byte >= 32) $display("[UART] '%c' (0x%02x)", rx_byte, rx_byte);
            else               $display("[UART] 0x%02x", rx_byte);
            if (rx_byte == 8'h0a) begin
                if (line_len == 2 && line_buf[0] == "o" && line_buf[1] == "k")
                    ok_count = ok_count + 1;
                else if (line_len == 1 && line_buf[0] == "?")
                    q_count = q_count + 1;
                line_len = 0;
            end else if (line_len < 64) begin
                line_buf[line_len] = rx_byte;
                line_len = line_len + 1;
            end
        end
    end

    task wait_for_ok;
        integer target_count;
        begin
            target_count = ok_count + 1;
            wait (ok_count >= target_count);
        end
    endtask

    task wait_for_q;
        integer target_count;
        begin
            target_count = q_count + 1;
            wait (q_count >= target_count);
        end
    endtask

    // ---- serial command driver (types down uart_rxd at realistic baud) -----
    task send_uart_byte(input [7:0] b);
        integer bi;
        begin
            uart_rxd = 1'b0;                     // start bit
            repeat (BIT_PERIOD) @(posedge clk);
            for (bi = 0; bi < 8; bi = bi + 1) begin
                uart_rxd = b[bi];                 // LSB first
                repeat (BIT_PERIOD) @(posedge clk);
            end
            uart_rxd = 1'b1;                     // stop bit
            repeat (BIT_PERIOD) @(posedge clk);
        end
    endtask

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

    // ---- settle / overshoot check, reused for each command -------------------
    localparam WINDOW_TICKS = 400;   // matches SEG_TICKS's monitoring window
    integer pos_hist [0:WINDOW_TICKS-1];
    integer settle_errors = 0;
    integer overshoot_errors = 0;

    task check_settle_and_overshoot(input integer target, input [255:0] label);
        integer seg_start_pos, step_size, k, local_tick;
        integer settle_tick, max_exc, min_exc, found_break;
        real    overshoot_pct;
        begin
            seg_start_pos = dut.u_enc.count;
            max_exc = seg_start_pos;
            min_exc = seg_start_pos;
            for (local_tick = 0; local_tick < WINDOW_TICKS; local_tick = local_tick + 1) begin
                @(posedge dut.u_timer.irq_pulse);
                pos_hist[local_tick] = dut.u_enc.count;
                if (pos_hist[local_tick] > max_exc) max_exc = pos_hist[local_tick];
                if (pos_hist[local_tick] < min_exc) min_exc = pos_hist[local_tick];
            end

            step_size = target - seg_start_pos;

            settle_tick = WINDOW_TICKS;   // sentinel: never settled
            k = WINDOW_TICKS - 1;
            found_break = 0;
            while (k >= 0 && !found_break) begin
                if ((pos_hist[k] - target <= 8) && (pos_hist[k] - target >= -8))
                    settle_tick = k;
                else
                    found_break = 1;
                k = k - 1;
            end

            if (step_size > 0)
                overshoot_pct = (max_exc > target) ? (max_exc - target) * 100.0 / step_size : 0.0;
            else if (step_size < 0)
                overshoot_pct = (min_exc < target) ? (target - min_exc) * 100.0 / (-step_size) : 0.0;
            else
                overshoot_pct = 0.0;

            $display("[CTRL] %0s: target=%0d settle_tick=%0d overshoot=%0.1f%% final_pos=%0d",
                      label, target, settle_tick, overshoot_pct, pos_hist[WINDOW_TICKS-1]);

            if (settle_tick > 300) begin
                $display("[CTRL] MISMATCH: %0s settle_tick=%0d exceeds 300-tick budget", label, settle_tick);
                settle_errors = settle_errors + 1;
            end
            if (overshoot_pct >= 25.0) begin
                $display("[CTRL] MISMATCH: %0s overshoot=%0.1f%% >= 25%%", label, overshoot_pct);
                overshoot_errors = overshoot_errors + 1;
            end
        end
    endtask

    // ---- become the user: type commands, verify the loop obeys them --------
    integer cmd_errors = 0;
    reg     scenario_done = 1'b0;

    initial begin : user_scenario
        @(posedge rst_n);

        // let segment 0 (target=0, already there at reset) nominally settle
        // before the first command arrives - well before the autonomous
        // profile's tick-400 transition, so it never fires.
        repeat (50) @(posedge dut.u_timer.irq_pulse);

        $display("[USER] typing T+300");
        send_uart_byte("T"); send_uart_byte("+"); send_uart_byte("3");
        send_uart_byte("0"); send_uart_byte("0"); send_uart_byte(8'h0a);
        wait_for_ok;
        check_settle_and_overshoot(300, "T+300");

        $display("[USER] typing T-150");
        send_uart_byte("T"); send_uart_byte("-"); send_uart_byte("1");
        send_uart_byte("5"); send_uart_byte("0"); send_uart_byte(8'h0a);
        wait_for_ok;
        check_settle_and_overshoot(-150, "T-150");

        $display("[USER] typing malformed Tx9");
        send_uart_byte("T"); send_uart_byte("x"); send_uart_byte("9");
        send_uart_byte(8'h0a);
        wait_for_q;
        // target must NOT have changed: position stays settled at -150
        repeat (60) @(posedge dut.u_timer.irq_pulse);
        if (dut.u_enc.count - (-150) > 8 || dut.u_enc.count - (-150) < -8) begin
            $display("[CTRL] MISMATCH: malformed command changed the target - position=%0d, expected ~-150",
                      dut.u_enc.count);
            cmd_errors = cmd_errors + 1;
        end else begin
            $display("[CTRL] malformed command correctly ignored - position=%0d, still ~-150", dut.u_enc.count);
        end

        scenario_done = 1'b1;
    end

    // ---- run ----------------------------------------------------------------
    initial begin
        if ($test$plusargs("trace")) begin
            $dumpfile("tb_soc.vcd");
            $dumpvars(0, tb_soc);
        end
        repeat (10) @(posedge clk);
        rst_n = 1;

        wait (scenario_done);
        repeat (10) @(posedge clk);   // let any in-flight monitor prints land

        $display("[TMR ] ticks=%0d period_errors=%0d", tick_count, period_errors);
        $display("[PWM ] pulses=%0d", pulse_count);
        $display("[CMD ] ok_count=%0d q_count=%0d", ok_count, q_count);
        $display("");
        if (uart_chars >= 3 && period_errors == 0 &&
            settle_errors == 0 && overshoot_errors == 0 && cmd_errors == 0 &&
            ok_count == 2 && q_count == 1)
            $display("RESULT: PASS  (uart_chars=%0d, ticks=%0d, ok=%0d, q=%0d)",
                     uart_chars, tick_count, ok_count, q_count);
        else
            $display("RESULT: FAIL  (uart_chars=%0d, ticks=%0d, ok=%0d, q=%0d)",
                     uart_chars, tick_count, ok_count, q_count);
        $finish;
    end

endmodule
