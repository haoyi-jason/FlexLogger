// regfile_core.v
// Simple 8-entry x 16-bit register file.
// Synchronous write, asynchronous read.
// Operates on psram_clk domain.
//
// Register map (addresses 0..7):
//   0x00  ADC watermark  (default 1)
//   0x01  IMU watermark  (default 1)
//   0x02  IRQ enable mask  bit[0]=ADC, bit[1]=IMU
//   0x03  AD7606 config   bits[2:0]=OS, bit[3]=RANGE
//   0x04  IMU sample rate divider (0 = 400 Hz native)
//   0x05  reserved
//   0x06  reserved
//   0x07  reserved

`timescale 1ns/1ps

module regfile_core (
    input  wire        clk,
    input  wire        rst_n,

    // Write port
    input  wire [2:0]  wr_addr,
    input  wire [15:0] wr_data,
    input  wire        wr_en,

    // Read port (combinational)
    input  wire [2:0]  rd_addr,
    output wire [15:0] rd_data,

    // Decoded register outputs
    output wire [15:0] reg_adc_watermark,
    output wire [15:0] reg_imu_watermark,
    output wire [1:0]  reg_irq_mask,
    output wire [3:0]  reg_adc_cfg,
    output wire [15:0] reg_imu_rate_div
);

    reg [15:0] regs [0:7];
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            regs[0] <= 16'd1;    // ADC watermark
            regs[1] <= 16'd1;    // IMU watermark
            regs[2] <= 16'h0003; // IRQ mask: both enabled
            regs[3] <= 16'h0000; // AD7606: OS=0, RANGE=0
            regs[4] <= 16'd0;    // IMU rate divider
            regs[5] <= 16'd0;
            regs[6] <= 16'd0;
            regs[7] <= 16'd0;
        end else if (wr_en) begin
            regs[wr_addr] <= wr_data;
        end
    end

    assign rd_data           = regs[rd_addr];
    assign reg_adc_watermark = regs[0];
    assign reg_imu_watermark = regs[1];
    assign reg_irq_mask      = regs[2][1:0];
    assign reg_adc_cfg       = regs[3][3:0];
    assign reg_imu_rate_div  = regs[4];

endmodule
