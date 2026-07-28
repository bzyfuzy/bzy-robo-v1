// =============================================================================
// pwm.v - Servo PWM peripheral
//
// Register map (word offsets from base):
//   0x0  CTRL      bit0 = enable
//   0x4  PRESCALE  tick = clk / (PRESCALE+1). e.g. 50 MHz clk, PRESCALE=49
//                  gives a 1 MHz tick (1 us resolution).
//   0x8  PERIOD    frame length in ticks (servo: 20000 ticks = 20 ms = 50 Hz)
//   0xC  DUTY      high time in ticks   (servo: 1000..2000 = 1..2 ms)
//
// Design notes:
//   * DUTY is double-buffered: writes land in a shadow register and are
//     transferred into the active register only at the start of a frame.
//     This guarantees glitch-free pulses - a servo never sees a torn pulse
//     even if software updates DUTY mid-frame.
//   * out is high while counter < duty_active.
// =============================================================================
`default_nettype none

module pwm (
    input  wire        clk,
    input  wire        rst_n,

    // simple register bus (one write/read per cycle, from soc_top)
    input  wire        sel,          // this peripheral is addressed
    input  wire [3:0]  wstrb,        // byte write strobes (0 = read)
    input  wire [3:0]  addr,         // word-aligned offset within peripheral
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    output reg         pwm_out
);

    reg        enable;
    reg [31:0] prescale;
    reg [31:0] period;
    reg [31:0] duty_shadow;   // written by software
    reg [31:0] duty_active;   // used by the counter, loaded at frame start

    reg [31:0] tick_cnt;
    reg [31:0] counter;
    wire       tick = (tick_cnt == prescale);
    wire       frame_start = tick && (counter >= period - 1);

    // ---- register writes ----------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            enable      <= 1'b0;
            prescale    <= 32'd0;
            period      <= 32'd20000;
            duty_shadow <= 32'd0;
        end else if (sel && wstrb != 4'b0000) begin
            case (addr)
                4'h0: enable      <= wdata[0];
                4'h4: prescale    <= wdata;
                4'h8: period      <= wdata;
                4'hC: duty_shadow <= wdata;
                default: ;
            endcase
        end
    end

    // ---- register reads -----------------------------------------------------
    always @(*) begin
        case (addr)
            4'h0:    rdata = {31'd0, enable};
            4'h4:    rdata = prescale;
            4'h8:    rdata = period;
            4'hC:    rdata = duty_shadow;
            default: rdata = 32'd0;
        endcase
    end

    // ---- prescaler: generates one tick every (prescale+1) clk cycles --------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tick_cnt <= 32'd0;
        else if (!enable)
            tick_cnt <= 32'd0;
        else if (tick)
            tick_cnt <= 32'd0;
        else
            tick_cnt <= tick_cnt + 1;
    end

    // ---- frame counter + glitch-free duty load ------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter     <= 32'd0;
            duty_active <= 32'd0;
            pwm_out     <= 1'b0;
        end else if (!enable) begin
            counter     <= 32'd0;
            pwm_out     <= 1'b0;
        end else if (tick) begin
            if (counter >= period - 1) begin
                counter     <= 32'd0;
                duty_active <= duty_shadow;          // safe update point
                pwm_out     <= (duty_shadow != 0);   // new frame starts high
            end else begin
                counter <= counter + 1;
                pwm_out <= ((counter + 1) < duty_active);
            end
        end
    end

endmodule

`default_nettype wire
