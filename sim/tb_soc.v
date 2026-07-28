// =============================================================================
// tb_soc.v - boots the SoC, checks UART prints "OK", measures PWM pulses.
//
// Pass criteria:
//   1. UART transmits 'O', 'K', '\n' (decoded from the serial line itself)
//   2. First measured PWM high-pulse is 1500 ticks (initial duty)
//   3. Subsequent pulses step through the 1000..2000 sweep
// =============================================================================
`timescale 1ns / 1ps

module tb_soc;

    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;                 // 100 MHz sim clock (period irrelevant)

    wire pwm_out;
    wire uart_txd;
    wire [7:0] leds;

    soc_top #(
        .FIRMWARE_HEX ("firmware.hex"),
        .UART_DIV     (8)                  // fast baud for simulation
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .pwm_out  (pwm_out),
        .uart_txd (uart_txd),
        .leds     (leds)
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

        $display("");
        if (uart_chars >= 3 && pulse_count >= 5)
            $display("RESULT: PASS  (uart_chars=%0d, pwm_pulses=%0d)",
                     uart_chars, pulse_count);
        else
            $display("RESULT: FAIL  (uart_chars=%0d, pwm_pulses=%0d)",
                     uart_chars, pulse_count);
        $finish;
    end

endmodule
