// psram_addr_map.v
// Compile-time parameters defining the PSRAM layout.
//
// PSRAM word = 64 bits (8 bytes).
// Each record (ADC or IMU) is 16 bytes = 2 PSRAM words.
// 1000 records → 2000 PSRAM words per buffer.
//
// Layout:
//   ADC buffer: words [ADC_BASE .. ADC_BASE + ADC_WORDS - 1]
//   IMU buffer: words [IMU_BASE .. IMU_BASE + IMU_WORDS - 1]

`timescale 1ns/1ps

module psram_addr_map #(
    // ADC buffer
    parameter ADC_BASE        = 21'd0,       // word address of first ADC record word-0
    parameter ADC_DEPTH       = 10'd1000,    // number of records
    parameter ADC_WORDS_PER   = 2,           // 64-bit words per record (16 bytes)
    // IMU buffer
    parameter IMU_BASE        = 21'd2000,    // word address after ADC buffer (1000*2=2000)
    parameter IMU_DEPTH       = 10'd1000,
    parameter IMU_WORDS_PER   = 2            // 64-bit words per padded IMU record (16 bytes)
) (
    // Expose parameters as output wires for use by other modules.
    // In Verilog-2001, parameters are accessed via defparam or port connections;
    // here we provide output wires so simulation can observe them.
    output wire [20:0] o_adc_base,
    output wire [9:0]  o_adc_depth,
    output wire [20:0] o_imu_base,
    output wire [9:0]  o_imu_depth
);

    assign o_adc_base  = ADC_BASE[20:0];
    assign o_adc_depth = ADC_DEPTH[9:0];
    assign o_imu_base  = IMU_BASE[20:0];
    assign o_imu_depth = IMU_DEPTH[9:0];

endmodule
