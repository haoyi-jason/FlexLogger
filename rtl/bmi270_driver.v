// bmi270_driver.v
// BMI270 IMU driver: initializes the sensor and reads 6-axis data
// at the configured output data rate via I2C.
//
// This is a simplified driver that:
//   1. Waits for PSRAM init_calib then performs BMI270 init sequence
//   2. Polls DRDY or uses a timer to read accel+gyro registers
//   3. Outputs a 96-bit frame: gx,gy,gz,ax,ay,az (each 16-bit, signed)
//
// I2C address: 0x68 (SDO tied low per schematic)
//
// TODO: Full BMI270 FIFO init sequence for production use.
// The init sequence below writes the minimum config to enable 6-axis mode:
//   - PWR_CTRL: enable accel + gyro
//   - ACC_CONF: ODR=100Hz, BW=normal
//   - GYR_CONF: ODR=100Hz, BW=normal
//   - PWR_CONF: disable advanced power save
// After init, reads DATA registers periodically.
//
// BMI270 registers used:
//   0x03  STATUS   (bit 6 = drdy_acc, bit 5 = drdy_gyr)
//   0x0C  DATA_8   GX_LSB
//   0x7C  PWR_CONF
//   0x7D  PWR_CTRL
//   0x40  ACC_CONF
//   0x42  GYR_CONF

`timescale 1ns/1ps

module bmi270_driver #(
    parameter CLKDIV = 31,     // 50 MHz / (4 * 400 kHz) ≈ 31
    parameter DEV_ADDR = 7'h68
) (
    input  wire        clk,       // 50 MHz
    input  wire        rst_n,

    // I2C pins (open-drain; connect to dedicated FPGA pins per instance)
    output wire        scl_oe,
    output wire        sda_oe,
    input  wire        sda_in,

    // Output: 6-axis IMU frame (valid when frame_valid=1)
    // [95:80]=gx, [79:64]=gy, [63:48]=gz, [47:32]=ax, [31:16]=ay, [15:0]=az
    output reg  [95:0] frame_data,
    output reg         frame_valid,
    output reg         init_done,
    output wire        busy
);

    // -----------------------------------------------------------------------
    // I2C master instance
    // -----------------------------------------------------------------------
    reg         i2c_start;
    reg         i2c_rw;
    reg  [6:0]  i2c_dev_addr;
    reg  [7:0]  i2c_reg_addr;
    reg  [7:0]  i2c_wr_data;
    wire [7:0]  i2c_rd_data;
    wire        i2c_done;
    wire        i2c_ack_err;
    wire        i2c_busy;

    i2c_master #(.CLKDIV(CLKDIV)) u_i2c (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (i2c_start),
        .rw       (i2c_rw),
        .dev_addr (i2c_dev_addr),
        .reg_addr (i2c_reg_addr),
        .wr_data  (i2c_wr_data),
        .rd_data  (i2c_rd_data),
        .done     (i2c_done),
        .ack_err  (i2c_ack_err),
        .busy     (i2c_busy),
        .scl_oe   (scl_oe),
        .sda_oe   (sda_oe),
        .sda_in   (sda_in)
    );

    assign busy = !init_done || (state != ST_IDLE);

    // -----------------------------------------------------------------------
    // Init sequence table
    // -----------------------------------------------------------------------
    // [reg_addr, wr_data] pairs
    localparam INIT_LEN = 4;
    reg [15:0] init_seq [0:INIT_LEN-1];
    initial begin
        init_seq[0] = {8'h7C, 8'h00}; // PWR_CONF: disable adv_power_save
        init_seq[1] = {8'h7D, 8'h0E}; // PWR_CTRL: enable accel+gyro+temp
        init_seq[2] = {8'h40, 8'hA8}; // ACC_CONF: ODR=100Hz, BW=norm, perf=CIC
        init_seq[3] = {8'h42, 8'hA9}; // GYR_CONF: ODR=100Hz, BW=norm, noise=perf
    end

    // Data register offsets for burst read of 12 bytes starting at 0x0C
    // GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H, AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H
    // We do 6 individual reads (no burst support in this simple I2C master).

    localparam [7:0] REG_DATA_START = 8'h0C; // DATA_8 (GX_LSB)
    localparam READ_REGS = 6'd12;

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    localparam ST_IDLE       = 4'd0;
    localparam ST_INIT       = 4'd1;
    localparam ST_INIT_WAIT  = 4'd2;
    localparam ST_INIT_NEXT  = 4'd3;
    localparam ST_WAIT_ODR   = 4'd4;   // 50 MHz / 400 Hz = 125000 cycles
    localparam ST_READ_REQ   = 4'd5;
    localparam ST_READ_WAIT  = 4'd6;
    localparam ST_READ_NEXT  = 4'd7;
    localparam ST_EMIT       = 4'd8;

    reg [3:0]  state;
    reg [2:0]  init_idx;
    reg [5:0]  rd_idx;
    reg [16:0] odr_cnt;    // 125000 max → 17 bits
    reg [95:0] tmp_frame;

    // 400 Hz at 50 MHz: 50_000_000 / 400 = 125_000 cycles
    localparam ODR_PERIOD = 17'd124999;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_INIT;
            init_done   <= 1'b0;
            frame_valid <= 1'b0;
            frame_data  <= 96'd0;
            i2c_start   <= 1'b0;
            i2c_rw      <= 1'b0;
            i2c_dev_addr<= DEV_ADDR;
            i2c_reg_addr<= 8'h00;
            i2c_wr_data <= 8'h00;
            init_idx    <= 3'd0;
            rd_idx      <= 6'd0;
            odr_cnt     <= 17'd0;
            tmp_frame   <= 96'd0;
        end else begin
            i2c_start   <= 1'b0;
            frame_valid <= 1'b0;

            case (state)
                ST_IDLE: begin
                    // Should not stay here; transitions to WAIT_ODR after init
                    state <= ST_WAIT_ODR;
                end

                // ---- Init sequence ----
                ST_INIT: begin
                    if (!i2c_busy) begin
                        i2c_rw      <= 1'b0;
                        i2c_dev_addr<= DEV_ADDR;
                        i2c_reg_addr<= init_seq[init_idx][15:8];
                        i2c_wr_data <= init_seq[init_idx][7:0];
                        i2c_start   <= 1'b1;
                        state       <= ST_INIT_WAIT;
                    end
                end

                ST_INIT_WAIT: begin
                    if (i2c_done) state <= ST_INIT_NEXT;
                end

                ST_INIT_NEXT: begin
                    if (init_idx == INIT_LEN - 1) begin
                        init_done <= 1'b1;
                        odr_cnt   <= 17'd0;
                        state     <= ST_WAIT_ODR;
                    end else begin
                        init_idx <= init_idx + 1'b1;
                        state    <= ST_INIT;
                    end
                end

                // ---- Wait for ODR period ----
                ST_WAIT_ODR: begin
                    if (odr_cnt == ODR_PERIOD) begin
                        odr_cnt <= 17'd0;
                        rd_idx  <= 6'd0;
                        state   <= ST_READ_REQ;
                    end else begin
                        odr_cnt <= odr_cnt + 1'b1;
                    end
                end

                // ---- Read 12 data bytes (6 registers x 2 bytes each) ----
                ST_READ_REQ: begin
                    if (!i2c_busy) begin
                        i2c_rw      <= 1'b1;
                        i2c_dev_addr<= DEV_ADDR;
                        i2c_reg_addr<= REG_DATA_START + rd_idx;
                        i2c_start   <= 1'b1;
                        state       <= ST_READ_WAIT;
                    end
                end

                ST_READ_WAIT: begin
                    if (i2c_done) state <= ST_READ_NEXT;
                end

                ST_READ_NEXT: begin
                    // Pack byte into tmp_frame (MSB = rd_idx 0)
                    // Layout: gx[15:8]=byte0, gx[7:0]=byte1, ..., az[7:0]=byte11
                    tmp_frame <= {tmp_frame[87:0], i2c_rd_data};
                    if (rd_idx == 6'd11) begin
                        state <= ST_EMIT;
                    end else begin
                        rd_idx <= rd_idx + 1'b1;
                        state  <= ST_READ_REQ;
                    end
                end

                ST_EMIT: begin
                    frame_data  <= tmp_frame;
                    frame_valid <= 1'b1;
                    state       <= ST_WAIT_ODR;
                end

                default: state <= ST_INIT;
            endcase
        end
    end

endmodule
