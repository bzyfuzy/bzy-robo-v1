// =============================================================================
// soc_top.v - Minimal robot-SoC step 1
//
//   PicoRV32 (RV32I) --- native memory bus --- address decoder
//        |                                          |
//        |             +------------+---------------+--------------+
//        |             |            |               |              |
//      (core)         RAM 8KB     PWM x1        UART TX         GPIO/LED
//
// Memory map (decoded on addr[31:24]):
//   0x0000_0000  RAM   (8 KB, firmware loaded here, reset vector = 0)
//   0x0200_0000  PWM   (CTRL / PRESCALE / PERIOD / DUTY)
//   0x0300_0000  UART  (DATA / STATUS)
//   0x0400_0000  GPIO  (bit per LED)
//   0x0500_0000  ENC   (COUNT / CTRL)
//   0x0600_0000  TIMER (CTRL / PERIOD / COUNT / STATUS) -> irq[3]
//
// The bus is PicoRV32's native interface: the core raises mem_valid with an
// address; the slave answers with mem_ready + mem_rdata one cycle later.
// Every access completes in exactly one wait state - simple and deterministic.
// =============================================================================
`timescale 1ns / 1ps
`default_nettype none

module soc_top #(
    parameter FIRMWARE_HEX = "firmware.hex",
    parameter UART_DIV     = 434
)(
    input  wire clk,
    input  wire rst_n,

    output wire pwm_out,
    output wire uart_txd,
    output wire [7:0] leds,

    input  wire enc_a,
    input  wire enc_b
);

    // ---- reset synchronizer --------------------------------------------------
    // The raw external rst_n feeds only this synchronizer; every register in
    // the design below (soc_top's own, every peripheral, and the CPU) resets
    // from rst_n_sync instead. See docs/reset.md for the rationale.
    wire rst_n_sync;
    rst_sync u_rst_sync (
        .clk       (clk),
        .rst_n_in  (rst_n),
        .rst_n_out (rst_n_sync)
    );

    // picorv32 is vendored and treats resetn purely synchronously inside
    // (no `negedge` in its always blocks), while everything else here uses
    // rst_n_sync asynchronously (see the Peripheral bus convention). Same
    // value, but a distinct net for the CPU keeps that intentional style
    // difference from reading as a cross-wired reset bug.
    wire cpu_resetn = rst_n_sync;

    // ---- PicoRV32 native memory interface -----------------------------------
    wire        mem_valid;
    reg         mem_ready;
    // mem_addr[23:13] is genuinely unread: the decoder only looks at
    // mem_addr[31:24] (peripheral select) and mem_addr[12:2]/[3:0] (RAM
    // index / peripheral word offset) - the gap is sparse-address-map
    // slack, not a bug.
    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] mem_addr;
    /* verilator lint_on UNUSEDSIGNAL */
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    reg  [31:0] mem_rdata;

    // ---- IRQ bus --------------------------------------------------------
    // bits 0/1/2 are reserved by PicoRV32 itself (internal timer/ebreak/
    // buserror); external sources start at bit 3. Our timer peripheral
    // drives bit 3 with a single-cycle pulse per tick (LATCHED_IRQ default
    // makes every irq input edge-triggered, so a 1-cycle pulse is enough -
    // the core latches it and auto-clears its pending bit on ISR entry).
    wire [31:0] irq_bus = {28'd0, timer_irq_pulse, 3'd0};

    picorv32 #(
        .ENABLE_COUNTERS (1),
        .ENABLE_MUL      (0),
        .ENABLE_DIV      (0),
        .ENABLE_IRQ      (1),
        .PROGADDR_RESET  (32'h0000_0000),
        .PROGADDR_IRQ    (32'h0000_1000),    // fixed firmware layout convention:
                                              // main program before 0x1000, ISR at/after
        .STACKADDR       (32'h0000_2000)     // top of 8 KB RAM
    ) u_cpu (
        .clk       (clk),
        .resetn    (cpu_resetn),
        .mem_valid (mem_valid),
        .mem_ready (mem_ready),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_wstrb (mem_wstrb),
        .mem_rdata (mem_rdata),
        // unused interfaces tied off: no split instr/data bus logic here,
        // and lookahead/pcpi/trace are optional PicoRV32 features this
        // minimal SoC doesn't use
        /* verilator lint_off PINCONNECTEMPTY */
        .mem_instr (),
        .mem_la_read  (), .mem_la_write (), .mem_la_addr (),
        .mem_la_wdata (), .mem_la_wstrb (),
        .pcpi_valid (), .pcpi_insn (), .pcpi_rs1 (), .pcpi_rs2 (),
        .pcpi_wr (1'b0), .pcpi_rd (32'd0), .pcpi_wait (1'b0), .pcpi_ready (1'b0),
        .irq (irq_bus), .eoi (),
        .trace_valid (), .trace_data (),
        .trap ()
        /* verilator lint_on PINCONNECTEMPTY */
    );

    // ---- address decode -----------------------------------------------------
    wire sel_ram  = mem_valid && (mem_addr[31:24] == 8'h00);
    wire sel_pwm  = mem_valid && (mem_addr[31:24] == 8'h02);
    wire sel_uart = mem_valid && (mem_addr[31:24] == 8'h03);
    wire sel_gpio = mem_valid && (mem_addr[31:24] == 8'h04);
    wire sel_enc  = mem_valid && (mem_addr[31:24] == 8'h05);
    wire sel_timer= mem_valid && (mem_addr[31:24] == 8'h06);

    // one-cycle ready for every access
    always @(posedge clk or negedge rst_n_sync) begin
        if (!rst_n_sync) mem_ready <= 1'b0;
        else             mem_ready <= mem_valid && !mem_ready;
    end

    // write strobes are only honored on the ready cycle (single write)
    wire [3:0] wstrb_eff = (mem_ready) ? mem_wstrb : 4'b0000;

    // ---- RAM: 8 KB = 2048 words, initialized from firmware hex --------------
    reg [31:0] ram [0:2047];
    initial $readmemh(FIRMWARE_HEX, ram);

    wire [10:0] ram_idx = mem_addr[12:2];

    always @(posedge clk) begin
        if (sel_ram) begin
            if (wstrb_eff[0]) ram[ram_idx][ 7: 0] <= mem_wdata[ 7: 0];
            if (wstrb_eff[1]) ram[ram_idx][15: 8] <= mem_wdata[15: 8];
            if (wstrb_eff[2]) ram[ram_idx][23:16] <= mem_wdata[23:16];
            if (wstrb_eff[3]) ram[ram_idx][31:24] <= mem_wdata[31:24];
        end
    end

    // ---- peripherals --------------------------------------------------------
    wire [31:0] pwm_rdata;
    pwm u_pwm (
        .clk   (clk),
        .rst_n (rst_n_sync),
        .sel   (sel_pwm),
        .wstrb (wstrb_eff),
        .addr  (mem_addr[3:0]),
        .wdata (mem_wdata),
        .rdata (pwm_rdata),
        .pwm_out (pwm_out)
    );

    wire [31:0] uart_rdata;
    uart_tx #(.DIV(UART_DIV)) u_uart (
        .clk   (clk),
        .rst_n (rst_n_sync),
        .sel   (sel_uart),
        .wstrb (wstrb_eff),
        .addr  (mem_addr[3:0]),
        .wdata (mem_wdata),
        .rdata (uart_rdata),
        .tx    (uart_txd)
    );

    reg [7:0] gpio_reg;
    always @(posedge clk or negedge rst_n_sync) begin
        if (!rst_n_sync)                     gpio_reg <= 8'd0;
        else if (sel_gpio && wstrb_eff[0])   gpio_reg <= mem_wdata[7:0];
    end
    assign leds = gpio_reg;

    wire [31:0] enc_rdata;
    quad_enc u_enc (
        .clk   (clk),
        .rst_n (rst_n_sync),
        .sel   (sel_enc),
        .wstrb (wstrb_eff),
        .addr  (mem_addr[3:0]),
        .wdata (mem_wdata),
        .rdata (enc_rdata),
        .enc_a (enc_a),
        .enc_b (enc_b)
    );

    wire [31:0] timer_rdata;
    wire        timer_irq_pulse;
    timer u_timer (
        .clk       (clk),
        .rst_n     (rst_n_sync),
        .sel       (sel_timer),
        .wstrb     (wstrb_eff),
        .addr      (mem_addr[3:0]),
        .wdata     (mem_wdata),
        .rdata     (timer_rdata),
        .irq_pulse (timer_irq_pulse)
    );

    // ---- read mux (registered, valid on the ready cycle) --------------------
    always @(posedge clk) begin
        case (mem_addr[31:24])
            8'h00:   mem_rdata <= ram[ram_idx];
            8'h02:   mem_rdata <= pwm_rdata;
            8'h03:   mem_rdata <= uart_rdata;
            8'h04:   mem_rdata <= {24'd0, gpio_reg};
            8'h05:   mem_rdata <= enc_rdata;
            8'h06:   mem_rdata <= timer_rdata;
            default: mem_rdata <= 32'h0000_0000;
        endcase
    end

endmodule

`default_nettype wire
