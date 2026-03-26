// gowin_psram_stub.v
// Simulation stub for the Gowin-generated PSRAM IP.
// Module name: PSRAM_Memory_Interface_HS_V2_Top
//
// Behaviour modelled for simulation:
//   - clk_out is driven from memory_clk (gives psram_clk to the system).
//   - init_calib is asserted after ~200 clk_d rising edges post-reset.
//   - A small behavioural memory array (2048 × 64-bit words) responds to
//     cmd_en pulses: writes are committed immediately; reads return rd_data
//     with rd_data_valid asserted 4 clk_d cycles later.
//   - Physical PSRAM pins are driven to idle / high-Z.
//
// Guard: only compiled when SYNTHESIS is NOT defined.

`ifndef SYNTHESIS

`timescale 1ns/1ps

module PSRAM_Memory_Interface_HS_V2_Top (
    // Clocking / reset
    input  wire        clk_d,
    input  wire        memory_clk,
    input  wire        memory_clk_p,
    input  wire        pll_lock,
    input  wire        rst_n,

    // Physical PSRAM pins
    output wire [1:0]  O_psram_ck,
    output wire [1:0]  O_psram_ck_n,
    inout  wire [15:0] IO_psram_dq,
    inout  wire [1:0]  IO_psram_rwds,
    output wire [1:0]  O_psram_cs_n,
    output wire [1:0]  O_psram_reset_n,

    // User data interface (clk_d domain)
    input  wire [63:0] wr_data,
    output reg  [63:0] rd_data,
    output reg         rd_data_valid,
    input  wire [20:0] addr,
    input  wire        cmd,          // 1 = write, 0 = read
    input  wire        cmd_en,
    output reg         init_calib,
    output wire        clk_out,
    input  wire [7:0]  data_mask
);

    // -----------------------------------------------------------------------
    // clk_out: forward memory_clk so the system gets a usable psram_clk
    // -----------------------------------------------------------------------
    assign clk_out = memory_clk;

    // -----------------------------------------------------------------------
    // Physical pins: idle state
    // -----------------------------------------------------------------------
    assign O_psram_ck      = 2'b00;
    assign O_psram_ck_n    = 2'b11;
    assign O_psram_cs_n    = 2'b11;
    assign O_psram_reset_n = 2'b11;
    assign IO_psram_dq     = 16'hz;
    assign IO_psram_rwds   = 2'hz;

    // -----------------------------------------------------------------------
    // Calibration counter – assert init_calib after 200 clk_d cycles
    // -----------------------------------------------------------------------
    integer calib_cnt;
    initial begin
        init_calib = 1'b0;
        calib_cnt  = 0;
    end

    always @(posedge clk_d or negedge rst_n) begin
        if (!rst_n) begin
            init_calib <= 1'b0;
            calib_cnt  <= 0;
        end else if (!init_calib) begin
            if (calib_cnt >= 199)
                init_calib <= 1'b1;
            else
                calib_cnt  <= calib_cnt + 1;
        end
    end

    // -----------------------------------------------------------------------
    // Behavioural memory – 2048 × 64-bit words (lowest 2048 addresses)
    // -----------------------------------------------------------------------
    reg [63:0] mem [0:2047];
    integer i;
    initial begin
        for (i = 0; i < 2048; i = i + 1)
            mem[i] = 64'd0;
    end

    // -----------------------------------------------------------------------
    // Command handler
    // -----------------------------------------------------------------------
    reg        pending_read;
    reg [10:0] read_addr_lat;
    reg [3:0]  read_delay;

    initial begin
        rd_data       = 64'd0;
        rd_data_valid = 1'b0;
        pending_read  = 1'b0;
        read_addr_lat = 11'd0;
        read_delay    = 4'd0;
    end

    always @(posedge clk_d or negedge rst_n) begin
        if (!rst_n) begin
            rd_data       <= 64'd0;
            rd_data_valid <= 1'b0;
            pending_read  <= 1'b0;
            read_addr_lat <= 11'd0;
            read_delay    <= 4'd0;
        end else begin
            rd_data_valid <= 1'b0;

            // Accept a new command only after calibration
            if (cmd_en && init_calib) begin
                if (cmd) begin
                    // Write: apply data_mask (0 = write byte, 1 = mask byte)
                    mem[addr[10:0]] <=
                        (wr_data & ~{{8{data_mask[7]}},{8{data_mask[6]}},
                                     {8{data_mask[5]}},{8{data_mask[4]}},
                                     {8{data_mask[3]}},{8{data_mask[2]}},
                                     {8{data_mask[1]}},{8{data_mask[0]}}}) |
                        (mem[addr[10:0]] & {{8{data_mask[7]}},{8{data_mask[6]}},
                                           {8{data_mask[5]}},{8{data_mask[4]}},
                                           {8{data_mask[3]}},{8{data_mask[2]}},
                                           {8{data_mask[1]}},{8{data_mask[0]}}});
                end else begin
                    // Read: latch address, start delay counter
                    read_addr_lat <= addr[10:0];
                    read_delay    <= 4'd4;
                    pending_read  <= 1'b1;
                end
            end

            // Read pipeline: return data after 4-cycle latency
            if (pending_read) begin
                if (read_delay == 4'd1) begin
                    rd_data       <= mem[read_addr_lat];
                    rd_data_valid <= 1'b1;
                    pending_read  <= 1'b0;
                    read_delay    <= 4'd0;
                end else begin
                    read_delay <= read_delay - 4'd1;
                end
            end
        end
    end

endmodule

`endif // SYNTHESIS
