// imu_psram_dma.v
// Receives averaged IMU frames (clk50 domain) via async FIFO,
// writes them to PSRAM via ring-buffer store engine (psram_clk domain).
//
// IMU frame: 6 axes x 16-bit = 96 bits = 12 bytes.
// Padded to 16 bytes in PSRAM: word0 = axes 0..3 lower 64 bits,
//                               word1 = axes 4..5 upper 32 bits + 32-bit zero pad.
// Mapping:
//   word0[63:48] = gx, word0[47:32] = gy, word0[31:16] = gz, word0[15:0] = ax
//   word1[63:48] = ay, word1[47:32] = az, word1[31:0]  = 0 (pad)

`timescale 1ns/1ps

module imu_psram_dma (
    // clk50 side
    input  wire        clk50,
    input  wire        rst50_n,
    input  wire [95:0] imu_frame,
    input  wire        imu_frame_valid,

    // psram_clk side
    input  wire        psram_clk,
    input  wire        psram_rst_n,

    // Ring pointer
    output wire        ptr_push,
    input  wire [9:0]  wr_idx,

    // PSRAM arbiter (client 1)
    output wire        arb_req,
    input  wire        arb_grant,
    output wire        arb_rel,
    output wire        psram_req,
    output wire        psram_cmd_wr,
    output wire [20:0] psram_addr,
    output wire [63:0] psram_wr_data,
    output wire [7:0]  psram_mask,
    input  wire        psram_busy
);

    // IMU base address in PSRAM (after ADC buffer: 1000*2=2000 words)
    localparam IMU_BASE = 21'd2000;

    // ------------------------------------------------------------------
    // CDC FIFO: 96-bit, depth 8
    // ------------------------------------------------------------------
    wire        fifo_full;
    wire        fifo_empty;
    wire [95:0] fifo_rd_data;
    reg         fifo_rd_en;

    async_fifo_simple #(.DATA_WIDTH(96), .DEPTH_LOG2(3)) u_cdc_fifo (
        .wr_clk   (clk50),
        .wr_rst_n (rst50_n),
        .wr_en    (imu_frame_valid && !fifo_full),
        .wr_data  (imu_frame),
        .full     (fifo_full),

        .rd_clk   (psram_clk),
        .rd_rst_n (psram_rst_n),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_rd_data),
        .empty    (fifo_empty)
    );

    // ------------------------------------------------------------------
    // psram_clk state machine
    // ------------------------------------------------------------------
    localparam ST_IDLE  = 2'd0;
    localparam ST_LATCH = 2'd1;
    localparam ST_STORE = 2'd2;

    reg [1:0]  state;
    reg [63:0] latch_w0, latch_w1;
    reg        store_push;
    wire       store_busy;

    psram_ringbuf_store #(
        .BASE_ADDR  (IMU_BASE),
        .WORDS_PER  (2)
    ) u_store (
        .clk          (psram_clk),
        .rst_n        (psram_rst_n),
        .din_word0    (latch_w0),
        .din_word1    (latch_w1),
        .push         (store_push),
        .busy         (store_busy),
        .ptr_push     (ptr_push),
        .wr_idx       (wr_idx),
        .arb_req      (arb_req),
        .arb_grant    (arb_grant),
        .arb_rel      (arb_rel),
        .psram_req    (psram_req),
        .psram_cmd_wr (psram_cmd_wr),
        .psram_addr   (psram_addr),
        .psram_wr_data(psram_wr_data),
        .psram_mask   (psram_mask),
        .psram_busy   (psram_busy)
    );

    always @(posedge psram_clk or negedge psram_rst_n) begin
        if (!psram_rst_n) begin
            state      <= ST_IDLE;
            fifo_rd_en <= 1'b0;
            store_push <= 1'b0;
            latch_w0   <= 64'd0;
            latch_w1   <= 64'd0;
        end else begin
            fifo_rd_en <= 1'b0;
            store_push <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (!fifo_empty && !store_busy) begin
                        fifo_rd_en <= 1'b1;
                        state      <= ST_LATCH;
                    end
                end

                ST_LATCH: begin
                    // Pack 96-bit IMU frame into two 64-bit words
                    // word0: gx(16), gy(16), gz(16), ax(16)
                    // word1: ay(16), az(16), pad(32)
                    latch_w0 <= fifo_rd_data[95:32];
                    latch_w1 <= {fifo_rd_data[31:0], 32'd0};
                    state    <= ST_STORE;
                end

                ST_STORE: begin
                    if (!store_busy) begin
                        store_push <= 1'b1;
                        state      <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
