// adc_psram_dma.v
// Receives ADC frames from ad7606_ctrl (clk50 domain) via an async FIFO,
// then writes them to PSRAM via the ring-buffer store engine (psram_clk domain).
//
// Each ADC frame is 8 x 16-bit = 128 bits = 16 bytes = 2 PSRAM words.
//
// CDC: clk50 → psram_clk via async_fifo_simple (128-bit wide).

`timescale 1ns/1ps

module adc_psram_dma (
    // clk50 side (write side of FIFO)
    input  wire         clk50,
    input  wire         rst50_n,
    input  wire [127:0] adc_frame,     // from ad7606_ctrl
    input  wire         adc_frame_valid,

    // psram_clk side
    input  wire         psram_clk,
    input  wire         psram_rst_n,

    // Ring buffer pointer interface
    output wire         ptr_push,
    input  wire [9:0]   wr_idx,

    // PSRAM arbiter (client 0)
    output wire         arb_req,
    input  wire         arb_grant,
    output wire         arb_rel,
    output wire         psram_req,
    output wire         psram_cmd_wr,
    output wire [20:0]  psram_addr,
    output wire [63:0]  psram_wr_data,
    output wire [7:0]   psram_mask,
    input  wire         psram_busy
);

    // ------------------------------------------------------------------
    // CDC FIFO: 128-bit, depth 8
    // ------------------------------------------------------------------
    wire        fifo_full;
    wire        fifo_empty;
    wire [127:0] fifo_rd_data;
    reg         fifo_rd_en;

    async_fifo_simple #(.DATA_WIDTH(128), .DEPTH_LOG2(3)) u_cdc_fifo (
        .wr_clk   (clk50),
        .wr_rst_n (rst50_n),
        .wr_en    (adc_frame_valid && !fifo_full),
        .wr_data  (adc_frame),
        .full     (fifo_full),

        .rd_clk   (psram_clk),
        .rd_rst_n (psram_rst_n),
        .rd_en    (fifo_rd_en),
        .rd_data  (fifo_rd_data),
        .empty    (fifo_empty)
    );

    // ------------------------------------------------------------------
    // State machine (psram_clk domain)
    // ------------------------------------------------------------------
    localparam ST_IDLE  = 2'd0;
    localparam ST_LATCH = 2'd1;
    localparam ST_STORE = 2'd2;

    reg [1:0]  state;
    reg [63:0] latch_w0, latch_w1;

    // Connect to psram_ringbuf_store
    reg  store_push;
    wire store_busy;

    psram_ringbuf_store #(
        .BASE_ADDR  (21'd0),
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
                    // One cycle after rd_en; data is valid now
                    latch_w0 <= fifo_rd_data[127:64];
                    latch_w1 <= fifo_rd_data[63:0];
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
