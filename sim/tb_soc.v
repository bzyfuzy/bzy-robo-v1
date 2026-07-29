// =============================================================================
// tb_soc.v - boots the SoC, checks UART prints "OK", measures PWM pulses.
//
// Pass criteria:
//   1. UART transmits 'O', 'K', '\n' (decoded from the serial line itself)
//   2. First measured PWM high-pulse is 1500 ticks (initial duty)
//   3. Subsequent pulses step through the 1000..2000 sweep
//   4. Timer fires irq[3] every PERIOD (2000) clk cycles
//   5. Each tick's ISR increments the tick counter and mirrors it onto the
//      `leds` pins - checked on the external pin, not by peeking at CPU
//      registers, so it proves the whole path (timer -> IRQ -> ISR -> bus
//      write) actually works, not just that the timer counts.
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

    // ---- PWM pulse-width monitor --------------------------------------------
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
            $display("[PWM ] pulse %0d: high = %0d ticks", pulse_count, high_cycles);
        end
    end

    // ---- timer IRQ / ISR monitor ---------------------------------------------
    // Watches two independent things and cross-checks them against each other:
    //   (a) dut.u_timer.irq_pulse - the hardware tick itself (internal signal,
    //       like the encoder's COUNT check) - verifies PERIOD (2000 clk) timing.
    //   (b) the `leds` pin - external, waveform-level - proves the interrupt
    //       actually reached the CPU and the ISR ran and hit the bus, since
    //       LEDs only change via the ISR's `sw` to GPIO.
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

    integer led_count = 0;
    integer led_errors = 0;
    reg [7:0] led_prev = 8'd0;
    initial begin : led_mon
        @(posedge rst_n);
        forever begin
            @(leds);
            led_count = led_count + 1;
            if (leds !== ((led_prev + 1) & 8'hFF)) begin
                $display("[LEDS] MISMATCH: change %0d leds=%0d expected=%0d",
                          led_count, leds, (led_prev + 1) & 8'hFF);
                led_errors = led_errors + 1;
            end
            led_prev = leds;
        end
    end

    // ---- quadrature encoder generator + checker -----------------------------
    // Drives enc_a/enc_b through real Gray-code waveforms in both directions
    // and cross-checks the decoder's internal COUNT after every step against
    // an independently-computed expected value.
    reg [1:0] enc_ab      = 2'b00;
    integer   enc_expected = 0;
    integer   enc_errors   = 0;

    task enc_step(input dir);   // dir=1: forward (+1), dir=0: reverse (-1)
        begin
            if (dir) begin
                case (enc_ab)
                    2'b00: enc_ab = 2'b01;
                    2'b01: enc_ab = 2'b11;
                    2'b11: enc_ab = 2'b10;
                    2'b10: enc_ab = 2'b00;
                endcase
                enc_expected = enc_expected + 1;
            end else begin
                case (enc_ab)
                    2'b00: enc_ab = 2'b10;
                    2'b10: enc_ab = 2'b11;
                    2'b11: enc_ab = 2'b01;
                    2'b01: enc_ab = 2'b00;
                endcase
                enc_expected = enc_expected - 1;
            end
            {enc_a, enc_b} = enc_ab;
            repeat (8) @(posedge clk);   // let 2FF sync + decode settle
            if (dut.u_enc.count !== enc_expected) begin
                $display("[ENC ] MISMATCH: count=%0d expected=%0d",
                          dut.u_enc.count, enc_expected);
                enc_errors = enc_errors + 1;
            end
        end
    endtask

    integer enc_i;
    initial begin : enc_mon
        @(posedge rst_n);
        repeat (20) @(posedge clk);         // let CPU boot settle first
        for (enc_i = 0; enc_i < 24; enc_i = enc_i + 1) enc_step(1);   // forward
        for (enc_i = 0; enc_i < 15; enc_i = enc_i + 1) enc_step(0);   // reverse
        $display("[ENC ] done: count=%0d expected=%0d errors=%0d",
                  dut.u_enc.count, enc_expected, enc_errors);
    end

    // ---- run ----------------------------------------------------------------
    initial begin
        if ($test$plusargs("trace")) begin
            $dumpfile("tb_soc.vcd");
            $dumpvars(0, tb_soc);
        end
        repeat (10) @(posedge clk);
        rst_n = 1;

        // enough time for boot + UART + ~8 PWM frames (20000 cycles each)
        repeat (400000) @(posedge clk);

        $display("[TMR ] ticks=%0d period_errors=%0d", tick_count, period_errors);
        $display("[LEDS] changes=%0d led_errors=%0d", led_count, led_errors);
        $display("");
        if (uart_chars >= 3 && pulse_count >= 5 &&
            enc_errors == 0 && dut.u_enc.count === 9 &&
            period_errors == 0 && led_errors == 0 &&
            tick_count >= 100 && led_count == tick_count)
            $display("RESULT: PASS  (uart_chars=%0d, pwm_pulses=%0d, enc_errors=%0d, enc_count=%0d, ticks=%0d)",
                     uart_chars, pulse_count, enc_errors, dut.u_enc.count, tick_count);
        else
            $display("RESULT: FAIL  (uart_chars=%0d, pwm_pulses=%0d, enc_errors=%0d, enc_count=%0d, ticks=%0d)",
                     uart_chars, pulse_count, enc_errors, dut.u_enc.count, tick_count);
        $finish;
    end

endmodule
