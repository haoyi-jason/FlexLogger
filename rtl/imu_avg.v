// imu_avg.v
// Averages 4 BMI270 outputs into a single 6-axis stream at 400 Hz.
// Operates on clk50 domain.
//
// When all 4 sensors have produced a new frame, this module:
//   - Averages gx, gy, gz, ax, ay, az (signed 16-bit → signed 18-bit sum / 4)
//   - Outputs the averaged 96-bit frame
//
// The driver controls the ODR; we use a simple "collect and average when all
// four sources have posted at least one new frame" policy.
// frame_valid from each bmi270_driver is used to latch data and set a pending bit.

`timescale 1ns/1ps

module imu_avg (
    input  wire        clk,
    input  wire        rst_n,

    // 4 IMU inputs (each 96-bit: gx,gy,gz,ax,ay,az × 16-bit)
    input  wire [95:0] imu0_data,
    input  wire        imu0_valid,
    input  wire [95:0] imu1_data,
    input  wire        imu1_valid,
    input  wire [95:0] imu2_data,
    input  wire        imu2_valid,
    input  wire [95:0] imu3_data,
    input  wire        imu3_valid,

    // Averaged output
    output reg  [95:0] avg_data,
    output reg         avg_valid
);

    // Latch each sensor's latest frame and a "new data available" flag
    reg [95:0] latch [0:3];
    reg [3:0]  ready;   // bit set when new frame received since last average

    integer i;

    // Latch incoming frames
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            latch[0] <= 96'd0; latch[1] <= 96'd0;
            latch[2] <= 96'd0; latch[3] <= 96'd0;
            ready    <= 4'b0000;
        end else begin
            if (imu0_valid) begin latch[0] <= imu0_data; ready[0] <= 1'b1; end
            if (imu1_valid) begin latch[1] <= imu1_data; ready[1] <= 1'b1; end
            if (imu2_valid) begin latch[2] <= imu2_data; ready[2] <= 1'b1; end
            if (imu3_valid) begin latch[3] <= imu3_data; ready[3] <= 1'b1; end

            avg_valid <= 1'b0;

            // When all four have new data, compute average
            if (ready == 4'b1111) begin
                ready <= 4'b0000;

                // Average each axis (signed arithmetic; extend to 18 bits)
                // gx
                avg_data[95:80] <= $signed(
                    ($signed(latch[0][95:80]) +
                     $signed(latch[1][95:80]) +
                     $signed(latch[2][95:80]) +
                     $signed(latch[3][95:80])) >>> 2);
                // gy
                avg_data[79:64] <= $signed(
                    ($signed(latch[0][79:64]) +
                     $signed(latch[1][79:64]) +
                     $signed(latch[2][79:64]) +
                     $signed(latch[3][79:64])) >>> 2);
                // gz
                avg_data[63:48] <= $signed(
                    ($signed(latch[0][63:48]) +
                     $signed(latch[1][63:48]) +
                     $signed(latch[2][63:48]) +
                     $signed(latch[3][63:48])) >>> 2);
                // ax
                avg_data[47:32] <= $signed(
                    ($signed(latch[0][47:32]) +
                     $signed(latch[1][47:32]) +
                     $signed(latch[2][47:32]) +
                     $signed(latch[3][47:32])) >>> 2);
                // ay
                avg_data[31:16] <= $signed(
                    ($signed(latch[0][31:16]) +
                     $signed(latch[1][31:16]) +
                     $signed(latch[2][31:16]) +
                     $signed(latch[3][31:16])) >>> 2);
                // az
                avg_data[15:0]  <= $signed(
                    ($signed(latch[0][15:0])  +
                     $signed(latch[1][15:0])  +
                     $signed(latch[2][15:0])  +
                     $signed(latch[3][15:0])) >>> 2);

                avg_valid <= 1'b1;
            end
        end
    end

endmodule
