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
//
// The bus is PicoRV32's native interface: the core raises mem_valid with an
// address; the slave answers with mem_ready + mem_rdata one cycle later.
// Every access completes in exactly one wait state - simple and deterministic.
// =============================================================================
`default_nettype none

module soc_top #(
    parameter FIRMWARE_HEX = "firmware.hex",
    parameter UART_DIV     = 434
)(
    input  wire clk,
    input  wire rst_n,

    output wire pwm_out,
    output wire uart_txd,
    output wire [7:0] leds
);

    // ---- PicoRV32 native memory interface -----------------------------------
    wire        mem_valid;
    wire        mem_instr;
    reg         mem_ready;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire [3:0]  mem_wstrb;
    reg  [31:0] mem_rdata;

    picorv32 #(
        .ENABLE_COUNTERS (1),
        .ENABLE_MUL      (0),
        .ENABLE_DIV      (0),
        .ENABLE_IRQ      (0),
        .PROGADDR_RESET  (32'h0000_0000),
        .STACKADDR       (32'h0000_2000)     // top of 8 KB RAM
    ) u_cpu (
        .clk       (clk),
        .resetn    (rst_n),
        .mem_valid (mem_valid),
        .mem_instr (mem_instr),
        .mem_ready (mem_ready),
        .mem_addr  (mem_addr),
        .mem_wdata (mem_wdata),
        .mem_wstrb (mem_wstrb),
        .mem_rdata (mem_rdata),
        // unused interfaces tied off
        .mem_la_read  (), .mem_la_write (), .mem_la_addr (),
        .mem_la_wdata (), .mem_la_wstrb (),
        .pcpi_valid (), .pcpi_insn (), .pcpi_rs1 (), .pcpi_rs2 (),
        .pcpi_wr (1'b0), .pcpi_rd (32'd0), .pcpi_wait (1'b0), .pcpi_ready (1'b0),
        .irq (32'd0), .eoi (),
        .trace_valid (), .trace_data (),
        .trap ()
    );

    // ---- address decode -----------------------------------------------------
    wire sel_ram  = mem_valid && (mem_addr[31:24] == 8'h00);
    wire sel_pwm  = mem_valid && (mem_addr[31:24] == 8'h02);
    wire sel_uart = mem_valid && (mem_addr[31:24] == 8'h03);
    wire sel_gpio = mem_valid && (mem_addr[31:24] == 8'h04);

    // one-cycle ready for every access
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) mem_ready <= 1'b0;
        else        mem_ready <= mem_valid && !mem_ready;
    end

    // write strobes are only honored on the ready cycle (single write)
    wire [3:0] wstrb_eff = (mem_ready) ? mem_wstrb : 4'b0000;

    // ---- RAM: 8 KB = 2048 words, initialized from firmware hex --------------
    reg [31:0] ram [0:2047];
    initial $readmemh(FIRMWARE_HEX, ram);

    wire [10:0] ram_idx = mem_addr[12:2];
    reg  [31:0] ram_rdata;

    always @(posedge clk) begin
        if (sel_ram) begin
            if (wstrb_eff[0]) ram[ram_idx][ 7: 0] <= mem_wdata[ 7: 0];
            if (wstrb_eff[1]) ram[ram_idx][15: 8] <= mem_wdata[15: 8];
            if (wstrb_eff[2]) ram[ram_idx][23:16] <= mem_wdata[23:16];
            if (wstrb_eff[3]) ram[ram_idx][31:24] <= mem_wdata[31:24];
            ram_rdata <= ram[ram_idx];
        end
    end

    // ---- peripherals --------------------------------------------------------
    wire [31:0] pwm_rdata;
    pwm u_pwm (
        .clk   (clk),
        .rst_n (rst_n),
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
        .rst_n (rst_n),
        .sel   (sel_uart),
        .wstrb (wstrb_eff),
        .addr  (mem_addr[3:0]),
        .wdata (mem_wdata),
        .rdata (uart_rdata),
        .tx    (uart_txd)
    );

    reg [7:0] gpio_reg;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                          gpio_reg <= 8'd0;
        else if (sel_gpio && wstrb_eff[0])   gpio_reg <= mem_wdata[7:0];
    end
    assign leds = gpio_reg;

    // ---- read mux (registered, valid on the ready cycle) --------------------
    always @(posedge clk) begin
        case (mem_addr[31:24])
            8'h00:   mem_rdata <= ram[ram_idx];
            8'h02:   mem_rdata <= pwm_rdata;
            8'h03:   mem_rdata <= uart_rdata;
            8'h04:   mem_rdata <= {24'd0, gpio_reg};
            default: mem_rdata <= 32'h0000_0000;
        endcase
    end

endmodule

`default_nettype wire
