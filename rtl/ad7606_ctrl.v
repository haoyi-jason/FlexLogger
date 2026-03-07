// ad7606_ctrl.v
// AD7606 ADC capture controller (clk50 domain, 50 MHz).
//
// The AD7606 in 16-bit parallel mode (CONVST A/B tied together):
//   - Assert CONVST for ≥25 ns → conversion starts (~4 µs typical)
//   - Wait for BUSY to go low
//   - Read 8 channels by toggling RD_N low→high; DB[15:0] valid while RD_N low
//
// This controller operates at 50 MHz (20 ns per cycle).
// CONVST pulse ≥ 2 cycles (40 ns).  BUSY deassert wait loop.
// RD_N hold ≥ 4 cycles (80 ns ≥ t_RDLW_MIN=40 ns).
//
// Output: 8 x 16-bit channel words, valid pulse.
// A conversion is triggered at the configured sample rate (default ~10 kSPS,
// but the parent module should call trig_conv periodically).

`timescale 1ns/1ps

module ad7606_ctrl (
    input  wire        clk50,
    input  wire        rst_n,

    // Trigger from rate generator (clk50 domain pulse)
    input  wire        trig_conv,

    // AD7606 control pins
    output reg         adc_convst,   // CONVST A and B tied together
    input  wire        adc_busy,     // BUSY
    output reg         adc_rd_n,     // RD_N (read strobe, active low)
    output reg         adc_cs_n,     // CS_N (active low chip select)
    output reg         adc_reset,    // RESET (active high, at startup)
    output reg  [2:0]  adc_os,       // Over-sampling ratio
    output reg         adc_range,    // Range select (0=±5V, 1=±10V)
    output reg         adc_stby_n,   // Standby (active low to engage standby)

    // Data bus (16-bit parallel)
    input  wire [15:0] adc_db,

    // Output frame (8 channels x 16-bit, packed as 128-bit vector)
    // ch0 = frame[127:112], ch7 = frame[15:0]
    output reg [127:0] frame_data,
    output reg         frame_valid,  // pulse: new frame in frame_data
    output wire        busy          // high during conversion
);

    // ------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------
    localparam ST_RESET      = 4'd0;
    localparam ST_IDLE       = 4'd1;
    localparam ST_CONVST     = 4'd2;
    localparam ST_WAIT_BUSY  = 4'd3;
    localparam ST_RD_ASSERT  = 4'd4;
    localparam ST_RD_HOLD    = 4'd5;
    localparam ST_RD_RELEASE = 4'd6;
    localparam ST_DONE       = 4'd7;

    reg [3:0]  state;
    reg [7:0]  delay_cnt;
    reg [2:0]  ch_idx;

    assign busy = (state != ST_IDLE);

    // Config registers (driven by regfile externally; for now use defaults)
    // These can be updated via WRITE_REG; parent module wires them in.
    // (Driven by top.v from regfile_core outputs)

    always @(posedge clk50 or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_RESET;
            adc_convst  <= 1'b0;
            adc_rd_n    <= 1'b1;
            adc_cs_n    <= 1'b1;
            adc_reset   <= 1'b1;
            adc_os      <= 3'b000;
            adc_range   <= 1'b0;
            adc_stby_n  <= 1'b1;
            delay_cnt   <= 8'd0;
            ch_idx      <= 3'd0;
            frame_data  <= 128'd0;
            frame_valid <= 1'b0;
        end else begin
            frame_valid <= 1'b0;

            case (state)
                // Hold RESET high for 8 cycles (~160 ns, min 50 ns required)
                ST_RESET: begin
                    adc_reset <= 1'b1;
                    if (delay_cnt == 8'd7) begin
                        adc_reset <= 1'b0;
                        delay_cnt <= 8'd0;
                        state     <= ST_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_IDLE: begin
                    adc_convst <= 1'b0;
                    adc_rd_n   <= 1'b1;
                    adc_cs_n   <= 1'b1;
                    if (trig_conv) begin
                        state <= ST_CONVST;
                    end
                end

                // Assert CONVST for 3 cycles (~60 ns)
                ST_CONVST: begin
                    adc_convst <= 1'b1;
                    if (delay_cnt == 8'd2) begin
                        adc_convst <= 1'b0;
                        delay_cnt  <= 8'd0;
                        state      <= ST_WAIT_BUSY;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // Wait for BUSY to go low (conversion complete)
                // Timeout after 512 cycles (~10.24 µs) to avoid hang
                ST_WAIT_BUSY: begin
                    if (!adc_busy) begin
                        ch_idx    <= 3'd0;
                        adc_cs_n  <= 1'b0;
                        state     <= ST_RD_ASSERT;
                    end else if (delay_cnt == 8'd255) begin
                        // Timeout: reset and return to idle
                        delay_cnt <= 8'd0;
                        state     <= ST_IDLE;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // Assert RD_N (active low)
                ST_RD_ASSERT: begin
                    adc_rd_n  <= 1'b0;
                    delay_cnt <= 8'd0;
                    state     <= ST_RD_HOLD;
                end

                // Hold RD_N low for 4 cycles, sample data
                ST_RD_HOLD: begin
                    if (delay_cnt == 8'd3) begin
                        // Store channel data in packed vector
                        // ch0 in bits[127:112], ch7 in bits[15:0]
                        frame_data[(7-ch_idx)*16 +: 16] <= adc_db;
                        state      <= ST_RD_RELEASE;
                        delay_cnt  <= 8'd0;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // Release RD_N, then move to next channel or finish
                ST_RD_RELEASE: begin
                    adc_rd_n <= 1'b1;
                    if (delay_cnt == 8'd1) begin
                        delay_cnt <= 8'd0;
                        if (ch_idx == 3'd7) begin
                            state <= ST_DONE;
                        end else begin
                            ch_idx <= ch_idx + 1'b1;
                            state  <= ST_RD_ASSERT;
                        end
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_DONE: begin
                    adc_cs_n    <= 1'b1;
                    frame_valid <= 1'b1;
                    state       <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
