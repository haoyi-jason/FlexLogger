// top.v
// FlexLogger Top-Level Module
// Target: Gowin GW1NR-9 QN88P
//
// Architecture:
//   clk50  – 50 MHz system clock (from OSC or ext pin)
//   psram_clk – output from Gowin PSRAM IP (clk_out), ~166 MHz
//
// Instantiated subsystems:
//   - PLL (Gowin rPLL) to generate memory_clk and memory_clk_p
//   - PSRAM IP wrapper
//   - PSRAM arbiter
//   - ADC (AD7606) capture controller
//   - ADC → PSRAM DMA
//   - 4x BMI270 drivers (one I2C bus each)
//   - IMU averager
//   - IMU → PSRAM DMA
//   - SPI command mailbox
//   - PSRAM command executor (with regfile + ring pointers)
//   - HOST_IRQ generation

`timescale 1ns/1ps

module top (
    // System clock (50 MHz)
    input  wire        clk50,

    // Active-low reset (from MCU or pushbutton)
    input  wire        rst_n_ext,

    // ---- AD7606 ADC interface ----
    output wire        adc_convst,
    input  wire        adc_busy,
    output wire        adc_rd_n,
    output wire        adc_cs_n,
    output wire        adc_reset,
    output wire [2:0]  adc_os,
    output wire        adc_range,
    output wire        adc_stby_n,
    input  wire [15:0] adc_db,

    // ---- BMI270 I2C buses (4 separate buses) ----
    output wire        imu0_scl_oe,
    output wire        imu0_sda_oe,
    input  wire        imu0_sda_in,

    output wire        imu1_scl_oe,
    output wire        imu1_sda_oe,
    input  wire        imu1_sda_in,

    output wire        imu2_scl_oe,
    output wire        imu2_sda_oe,
    input  wire        imu2_sda_in,

    output wire        imu3_scl_oe,
    output wire        imu3_sda_oe,
    input  wire        imu3_sda_in,

    // ---- PSRAM (Winbond W955D8MKY-6I x2 via Gowin IP) ----
    output wire [1:0]  O_psram_ck,
    output wire [1:0]  O_psram_ck_n,
    inout  wire [15:0] IO_psram_dq,
    inout  wire [1:0]  IO_psram_rwds,
    output wire [1:0]  O_psram_cs_n,
    output wire [1:0]  O_psram_reset_n,

    // ---- SPI slave (MCU interface) ----
    input  wire        spi_sclk,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output wire        spi_miso,

    // ---- Host interrupt ----
    output reg         HOST_IRQ
);

    // =========================================================================
    // Resets
    // =========================================================================
    // Synchronise external reset into clk50 and psram_clk domains
    reg [2:0] rst50_sync;
    wire      rst50_n = rst50_sync[2];

    always @(posedge clk50 or negedge rst_n_ext) begin
        if (!rst_n_ext) rst50_sync <= 3'b000;
        else            rst50_sync <= {rst50_sync[1:0], 1'b1};
    end

    // psram_clk reset derived after PLL lock + PSRAM init_calib
    wire        psram_clk;
    wire        pll_lock;
    wire        init_calib;

    reg [2:0]   rst_psram_sync;
    wire        rst_psram_n = rst_psram_sync[2];
    wire        pll_and_calib = pll_lock && init_calib;

    always @(posedge psram_clk or negedge pll_and_calib) begin
        if (!pll_and_calib) rst_psram_sync <= 3'b000;
        else                rst_psram_sync <= {rst_psram_sync[1:0], 1'b1};
    end

    // =========================================================================
    // PLL: generate memory_clk (e.g. 166 MHz) and memory_clk_p (phase-shifted)
    // Replace with Gowin rPLL IP instantiation.
    // =========================================================================
    wire memory_clk, memory_clk_p;

    // TODO: Replace with actual Gowin rPLL IP (generated via EDA GUI).
    // The rPLL should output:
    //   CLKOUT  → memory_clk  (e.g. 166.667 MHz for DDR-333 operation)
    //   CLKOUTP → memory_clk_p (phase offset by ~90°)
    //   LOCK    → pll_lock
    // Placeholder simulation model:
    // synthesis translate_off
    assign memory_clk   = clk50;
    assign memory_clk_p = clk50;
    assign pll_lock     = 1'b1;
    // synthesis translate_on

    // =========================================================================
    // PSRAM wrapper (includes Gowin IP)
    // =========================================================================
    wire        psram_req_arb;
    wire        psram_cmd_wr_arb;
    wire [20:0] psram_addr_arb;
    wire [63:0] psram_wr_data_arb;
    wire [7:0]  psram_mask_arb;
    wire [63:0] psram_rd_data_w;
    wire        psram_rd_valid_w;
    wire        psram_busy_w;

    // clk_out from PSRAM IP becomes psram_clk
    wire clk_out_psram;
    assign psram_clk = clk_out_psram;

    psram_wrap #(
        .CMD_WRITE    (1),
        .ADDR_IS_WORD (1)
    ) u_psram (
        .clk_d          (psram_clk),
        .memory_clk     (memory_clk),
        .memory_clk_p   (memory_clk_p),
        .pll_lock       (pll_lock),
        .rst_n          (rst50_n),     // use clk50 reset for IP (before psram_clk available)
        .O_psram_ck     (O_psram_ck),
        .O_psram_ck_n   (O_psram_ck_n),
        .IO_psram_dq    (IO_psram_dq),
        .IO_psram_rwds  (IO_psram_rwds),
        .O_psram_cs_n   (O_psram_cs_n),
        .O_psram_reset_n(O_psram_reset_n),
        .req            (psram_req_arb),
        .cmd_wr         (psram_cmd_wr_arb),
        .addr           (psram_addr_arb),
        .wr_data        (psram_wr_data_arb),
        .data_mask      (psram_mask_arb),
        .rd_data        (psram_rd_data_w),
        .rd_valid       (psram_rd_valid_w),
        .busy           (psram_busy_w),
        .init_calib     (init_calib)
    );

    // Expose clk_out from IP through assign
    // (psram_wrap's u_psram_ip.clk_out is left unconnected internally;
    //  the top-level uses a direct connection)
    // NOTE: In real Gowin synthesis, clk_out is a dedicated global clock.
    // Wire it directly in the constraints / netlist.
    // For simulation, use the placeholder assignment above.

    // =========================================================================
    // ADC sample rate generator (10 kSPS at clk50)
    // 50 MHz / 10000 = 5000 cycles between triggers
    // =========================================================================
    reg [12:0] adc_rate_cnt;
    reg        adc_trig;

    always @(posedge clk50 or negedge rst50_n) begin
        if (!rst50_n) begin
            adc_rate_cnt <= 13'd0;
            adc_trig     <= 1'b0;
        end else begin
            adc_trig <= 1'b0;
            if (adc_rate_cnt == 13'd4999) begin
                adc_rate_cnt <= 13'd0;
                adc_trig     <= 1'b1;
            end else begin
                adc_rate_cnt <= adc_rate_cnt + 1'b1;
            end
        end
    end

    // =========================================================================
    // AD7606 controller
    // =========================================================================
    wire [127:0] adc_frame_data;
    wire         adc_frame_valid;
    wire         adc_busy_ctrl;

    ad7606_ctrl u_adc (
        .clk50       (clk50),
        .rst_n       (rst50_n),
        .trig_conv   (adc_trig),
        .adc_convst  (adc_convst),
        .adc_busy    (adc_busy),
        .adc_rd_n    (adc_rd_n),
        .adc_cs_n    (adc_cs_n),
        .adc_reset   (adc_reset),
        .adc_os      (adc_os),
        .adc_range   (adc_range),
        .adc_stby_n  (adc_stby_n),
        .adc_db      (adc_db),
        .frame_data  (adc_frame_data),
        .frame_valid (adc_frame_valid),
        .busy        (adc_busy_ctrl)
    );

    // =========================================================================
    // BMI270 drivers
    // =========================================================================
    wire [95:0] imu0_frame, imu1_frame, imu2_frame, imu3_frame;
    wire        imu0_valid, imu1_valid, imu2_valid, imu3_valid;

    bmi270_driver #(.CLKDIV(31), .DEV_ADDR(7'h68)) u_imu0 (
        .clk        (clk50), .rst_n    (rst50_n),
        .scl_oe     (imu0_scl_oe), .sda_oe (imu0_sda_oe), .sda_in (imu0_sda_in),
        .frame_data (imu0_frame),  .frame_valid (imu0_valid),
        .init_done  (), .busy ()
    );

    bmi270_driver #(.CLKDIV(31), .DEV_ADDR(7'h68)) u_imu1 (
        .clk        (clk50), .rst_n    (rst50_n),
        .scl_oe     (imu1_scl_oe), .sda_oe (imu1_sda_oe), .sda_in (imu1_sda_in),
        .frame_data (imu1_frame),  .frame_valid (imu1_valid),
        .init_done  (), .busy ()
    );

    bmi270_driver #(.CLKDIV(31), .DEV_ADDR(7'h68)) u_imu2 (
        .clk        (clk50), .rst_n    (rst50_n),
        .scl_oe     (imu2_scl_oe), .sda_oe (imu2_sda_oe), .sda_in (imu2_sda_in),
        .frame_data (imu2_frame),  .frame_valid (imu2_valid),
        .init_done  (), .busy ()
    );

    bmi270_driver #(.CLKDIV(31), .DEV_ADDR(7'h68)) u_imu3 (
        .clk        (clk50), .rst_n    (rst50_n),
        .scl_oe     (imu3_scl_oe), .sda_oe (imu3_sda_oe), .sda_in (imu3_sda_in),
        .frame_data (imu3_frame),  .frame_valid (imu3_valid),
        .init_done  (), .busy ()
    );

    // =========================================================================
    // IMU averager
    // =========================================================================
    wire [95:0] imu_avg_frame;
    wire        imu_avg_valid;

    imu_avg u_imu_avg (
        .clk        (clk50),
        .rst_n      (rst50_n),
        .imu0_data  (imu0_frame), .imu0_valid (imu0_valid),
        .imu1_data  (imu1_frame), .imu1_valid (imu1_valid),
        .imu2_data  (imu2_frame), .imu2_valid (imu2_valid),
        .imu3_data  (imu3_frame), .imu3_valid (imu3_valid),
        .avg_data   (imu_avg_frame),
        .avg_valid  (imu_avg_valid)
    );

    // =========================================================================
    // ADC ring buffer pointer (psram_clk domain)
    // =========================================================================
    wire        adc_ptr_push;
    wire [9:0]  adc_wr_idx, adc_rd_idx;
    wire [10:0] adc_pending;
    wire [15:0] adc_overflow;
    wire        adc_pop_exec;

    ringptr_1000 u_adc_ptr (
        .clk      (psram_clk),
        .rst_n    (rst_psram_n),
        .push     (adc_ptr_push),
        .pop      (adc_pop_exec),
        .wr_idx   (adc_wr_idx),
        .rd_idx   (adc_rd_idx),
        .pending  (adc_pending),
        .overflow (adc_overflow)
    );

    // =========================================================================
    // IMU ring buffer pointer (psram_clk domain)
    // =========================================================================
    wire        imu_ptr_push;
    wire [9:0]  imu_wr_idx, imu_rd_idx;
    wire [10:0] imu_pending;
    wire [15:0] imu_overflow;
    wire        imu_pop_exec;

    ringptr_1000 u_imu_ptr (
        .clk      (psram_clk),
        .rst_n    (rst_psram_n),
        .push     (imu_ptr_push),
        .pop      (imu_pop_exec),
        .wr_idx   (imu_wr_idx),
        .rd_idx   (imu_rd_idx),
        .pending  (imu_pending),
        .overflow (imu_overflow)
    );

    // =========================================================================
    // ADC DMA (client 0)
    // =========================================================================
    wire        c0_req, c0_grant, c0_rel;
    wire        c0_psram_req, c0_cmd_wr;
    wire [20:0] c0_addr;
    wire [63:0] c0_wr_data;
    wire [7:0]  c0_mask;

    adc_psram_dma u_adc_dma (
        .clk50           (clk50),
        .rst50_n         (rst50_n),
        .adc_frame       (adc_frame_data),
        .adc_frame_valid (adc_frame_valid),
        .psram_clk       (psram_clk),
        .psram_rst_n     (rst_psram_n),
        .ptr_push        (adc_ptr_push),
        .wr_idx          (adc_wr_idx),
        .arb_req         (c0_req),
        .arb_grant       (c0_grant),
        .arb_rel         (c0_rel),
        .psram_req       (c0_psram_req),
        .psram_cmd_wr    (c0_cmd_wr),
        .psram_addr      (c0_addr),
        .psram_wr_data   (c0_wr_data),
        .psram_mask      (c0_mask),
        .psram_busy      (psram_busy_w)
    );

    // =========================================================================
    // IMU DMA (client 1)
    // =========================================================================
    wire        c1_req, c1_grant, c1_rel;
    wire        c1_psram_req, c1_cmd_wr;
    wire [20:0] c1_addr;
    wire [63:0] c1_wr_data;
    wire [7:0]  c1_mask;

    imu_psram_dma u_imu_dma (
        .clk50           (clk50),
        .rst50_n         (rst50_n),
        .imu_frame       (imu_avg_frame),
        .imu_frame_valid (imu_avg_valid),
        .psram_clk       (psram_clk),
        .psram_rst_n     (rst_psram_n),
        .ptr_push        (imu_ptr_push),
        .wr_idx          (imu_wr_idx),
        .arb_req         (c1_req),
        .arb_grant       (c1_grant),
        .arb_rel         (c1_rel),
        .psram_req       (c1_psram_req),
        .psram_cmd_wr    (c1_cmd_wr),
        .psram_addr      (c1_addr),
        .psram_wr_data   (c1_wr_data),
        .psram_mask      (c1_mask),
        .psram_busy      (psram_busy_w)
    );

    // =========================================================================
    // Fetch engine (client 2, shared for ADC and IMU reads)
    // The fetch engine is controlled by psram_cmd_exec.
    // We use a shared psram_ringbuf_fetch with a mux on BASE_ADDR.
    // =========================================================================
    wire        c2_req, c2_grant, c2_rel;
    wire        c2_psram_req, c2_cmd_wr;
    wire [20:0] c2_addr;
    wire [63:0] c2_wr_data;
    wire [7:0]  c2_mask;

    wire [9:0]  exec_fetch_idx;
    wire        exec_fetch_req;
    wire        exec_fetch_is_imu;
    wire        fetch_busy;
    wire [63:0] fetch_word0, fetch_word1;
    wire        fetch_valid;

    // Compute fetch base: ADC=0, IMU=2000
    wire [20:0] fetch_base = exec_fetch_is_imu ? 21'd2000 : 21'd0;

    psram_ringbuf_fetch #(
        .BASE_ADDR (21'd0)   // overridden by fetch_base offset below
    ) u_fetch (
        .clk          (psram_clk),
        .rst_n        (rst_psram_n),
        .fetch_idx    (exec_fetch_idx),
        .fetch_req    (exec_fetch_req),
        .busy         (fetch_busy),
        .rd_word0     (fetch_word0),
        .rd_word1     (fetch_word1),
        .rd_valid     (fetch_valid),
        .arb_req      (c2_req),
        .arb_grant    (c2_grant),
        .arb_rel      (c2_rel),
        .psram_req    (c2_psram_req),
        .psram_cmd_wr (c2_cmd_wr),
        .psram_addr   (c2_addr),
        .psram_wr_data(c2_wr_data),
        .psram_mask   (c2_mask),
        .psram_rd_data(psram_rd_data_w),
        .psram_rd_valid(psram_rd_valid_w),
        .psram_busy   (psram_busy_w)
    );
    // NOTE: The fetch module's BASE_ADDR is 0, but the address it generates
    // is BASE_ADDR + {idx,word_bit}.  For IMU we need to offset by 2000.
    // The psram_ringbuf_fetch module uses a fixed BASE_ADDR parameter.
    // To support both buffers we instantiate two fetch modules and select,
    // or we add the offset externally by replacing the address.
    // Here we keep it simple: the arbiter c2_addr is overridden below.

    // =========================================================================
    // PSRAM arbiter
    // =========================================================================
    // Override c2_addr to include buffer-specific base address
    wire [20:0] c2_addr_offset = c2_addr + fetch_base;

    psram_arbiter u_arb (
        .clk          (psram_clk),
        .rst_n        (rst_psram_n),
        // Client 0 – ADC DMA
        .c0_req       (c0_req),
        .c0_grant     (c0_grant),
        .c0_rel       (c0_rel),
        .c0_psram_req (c0_psram_req),
        .c0_cmd_wr    (c0_cmd_wr),
        .c0_addr      (c0_addr),
        .c0_wr_data   (c0_wr_data),
        .c0_mask      (c0_mask),
        // Client 1 – IMU DMA
        .c1_req       (c1_req),
        .c1_grant     (c1_grant),
        .c1_rel       (c1_rel),
        .c1_psram_req (c1_psram_req),
        .c1_cmd_wr    (c1_cmd_wr),
        .c1_addr      (c1_addr),
        .c1_wr_data   (c1_wr_data),
        .c1_mask      (c1_mask),
        // Client 2 – Fetch / CMD exec (with address offset)
        .c2_req       (c2_req),
        .c2_grant     (c2_grant),
        .c2_rel       (c2_rel),
        .c2_psram_req (c2_psram_req),
        .c2_cmd_wr    (c2_cmd_wr),
        .c2_addr      (c2_addr_offset),
        .c2_wr_data   (c2_wr_data),
        .c2_mask      (c2_mask),
        // PSRAM interface
        .psram_req    (psram_req_arb),
        .psram_cmd_wr (psram_cmd_wr_arb),
        .psram_addr   (psram_addr_arb),
        .psram_wr_data(psram_wr_data_arb),
        .psram_mask   (psram_mask_arb),
        .psram_rd_data(psram_rd_data_w),
        .psram_rd_valid(psram_rd_valid_w),
        .psram_busy   (psram_busy_w),
        .rd_data      (),
        .rd_valid     ()
    );

    // =========================================================================
    // Register file
    // =========================================================================
    wire [2:0]  reg_rd_addr_exec, reg_wr_addr_exec;
    wire [15:0] reg_rd_data_exec, reg_wr_data_exec;
    wire        reg_wr_en_exec;
    wire [15:0] reg_adc_watermark, reg_imu_watermark;
    wire [1:0]  reg_irq_mask;
    wire [3:0]  reg_adc_cfg;

    regfile_core u_regfile (
        .clk              (psram_clk),
        .rst_n            (rst_psram_n),
        .wr_addr          (reg_wr_addr_exec),
        .wr_data          (reg_wr_data_exec),
        .wr_en            (reg_wr_en_exec),
        .rd_addr          (reg_rd_addr_exec),
        .rd_data          (reg_rd_data_exec),
        .reg_adc_watermark(reg_adc_watermark),
        .reg_imu_watermark(reg_imu_watermark),
        .reg_irq_mask     (reg_irq_mask),
        .reg_adc_cfg      (reg_adc_cfg),
        .reg_imu_rate_div ()
    );

    // =========================================================================
    // SPI command mailbox
    // =========================================================================
    wire [63:0] cmd_data_mb;
    wire        cmd_valid_mb, cmd_ready_mb;
    wire [7:0]  resp_data_mb;
    wire        resp_push_mb, resp_full_mb;

    spi_cmd_mailbox u_mailbox (
        .spi_sclk    (spi_sclk),
        .spi_cs_n    (spi_cs_n),
        .spi_mosi    (spi_mosi),
        .spi_miso    (spi_miso),
        .spi_rst_n   (rst50_n),
        .psram_clk   (psram_clk),
        .psram_rst_n (rst_psram_n),
        .cmd_data    (cmd_data_mb),
        .cmd_valid   (cmd_valid_mb),
        .cmd_ready   (cmd_ready_mb),
        .resp_data   (resp_data_mb),
        .resp_push   (resp_push_mb),
        .resp_full   (resp_full_mb)
    );

    // =========================================================================
    // PSRAM command executor
    // =========================================================================
    psram_cmd_exec u_cmd_exec (
        .clk           (psram_clk),
        .rst_n         (rst_psram_n),
        .cmd_data      (cmd_data_mb),
        .cmd_valid     (cmd_valid_mb),
        .cmd_ready     (cmd_ready_mb),
        .resp_data     (resp_data_mb),
        .resp_push     (resp_push_mb),
        .resp_full     (resp_full_mb),
        // ADC ring pointer
        .adc_pending   (adc_pending),
        .adc_rd_idx    (adc_rd_idx),
        .adc_wr_idx    (adc_wr_idx),
        .adc_overflow  (adc_overflow),
        .adc_pop       (adc_pop_exec),
        // IMU ring pointer
        .imu_pending   (imu_pending),
        .imu_rd_idx    (imu_rd_idx),
        .imu_wr_idx    (imu_wr_idx),
        .imu_overflow  (imu_overflow),
        .imu_pop       (imu_pop_exec),
        // Register file
        .reg_rd_addr   (reg_rd_addr_exec),
        .reg_rd_data   (reg_rd_data_exec),
        .reg_wr_addr   (reg_wr_addr_exec),
        .reg_wr_data   (reg_wr_data_exec),
        .reg_wr_en     (reg_wr_en_exec),
        // Fetch engine
        .fetch_idx     (exec_fetch_idx),
        .fetch_req     (exec_fetch_req),
        .fetch_is_imu  (exec_fetch_is_imu),
        .fetch_busy    (fetch_busy),
        .fetch_word0   (fetch_word0),
        .fetch_word1   (fetch_word1),
        .fetch_valid   (fetch_valid),
        // Raw PSRAM (bring-up) – not arbitrated; tied off for now
        .raw_req       (),
        .raw_cmd_wr    (),
        .raw_addr      (),
        .raw_wr_data   (),
        .raw_mask      (),
        .raw_rd_data   (64'd0),
        .raw_rd_valid  (1'b0),
        .raw_busy      (1'b0),
        .init_calib    (init_calib)
    );

    // =========================================================================
    // HOST_IRQ generation (psram_clk domain, synchronised)
    // =========================================================================
    wire adc_irq_en = reg_irq_mask[0];
    wire imu_irq_en = reg_irq_mask[1];

    wire adc_irq_level = adc_irq_en && (adc_pending >= reg_adc_watermark[10:0]);
    wire imu_irq_level = imu_irq_en && (imu_pending >= reg_imu_watermark[10:0]);
    wire irq_combined  = adc_irq_level | imu_irq_level;

    // Synchronize IRQ into clk50 domain (or keep in psram_clk and leave to MCU)
    reg [2:0] irq_sync;
    always @(posedge clk50 or negedge rst50_n) begin
        if (!rst50_n) irq_sync <= 3'b000;
        else          irq_sync <= {irq_sync[1:0], irq_combined};
    end

    always @(posedge clk50 or negedge rst50_n) begin
        if (!rst50_n) HOST_IRQ <= 1'b0;
        else          HOST_IRQ <= irq_sync[2];
    end

endmodule
