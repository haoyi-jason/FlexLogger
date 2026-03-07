// psram_ringbuf_fetch.v
// Reads one 16-byte record (2 x 64-bit words) from a circular buffer in PSRAM.
// Operates on psram_clk domain.
//
// Usage:
//   1. Caller places a 10-bit logical index on fetch_idx and asserts fetch_req.
//   2. Module acquires arbiter, reads word0 then word1 from PSRAM.
//   3. When done, rd_valid is pulsed and rd_word0/rd_word1 hold the data.

`timescale 1ns/1ps

module psram_ringbuf_fetch #(
    parameter BASE_ADDR = 21'd0    // PSRAM word-address base of buffer
) (
    input  wire        clk,
    input  wire        rst_n,

    // Fetch request
    input  wire [9:0]  fetch_idx,  // logical record index (0..999)
    input  wire        fetch_req,  // pulse: start fetch
    output wire        busy,

    // Result
    output reg  [63:0] rd_word0,
    output reg  [63:0] rd_word1,
    output reg         rd_valid,   // pulse: data ready

    // PSRAM arbiter
    output reg         arb_req,
    input  wire        arb_grant,
    output reg         arb_rel,

    // PSRAM read port
    output reg         psram_req,
    output reg         psram_cmd_wr,  // 0 = read
    output reg  [20:0] psram_addr,
    output wire [63:0] psram_wr_data, // unused for reads
    output wire [7:0]  psram_mask,
    input  wire [63:0] psram_rd_data,
    input  wire        psram_rd_valid,
    input  wire        psram_busy
);

    assign psram_wr_data = 64'd0;
    assign psram_mask    = 8'hFF;  // all bytes masked for read (mask unused)

    localparam ST_IDLE  = 3'd0;
    localparam ST_ARB   = 3'd1;
    localparam ST_RD0   = 3'd2;
    localparam ST_RD0W  = 3'd3;
    localparam ST_RD1   = 3'd4;
    localparam ST_RD1W  = 3'd5;
    localparam ST_DONE  = 3'd6;

    reg [2:0] state;
    reg [9:0] latch_idx;

    assign busy = (state != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            arb_req      <= 1'b0;
            arb_rel      <= 1'b0;
            psram_req    <= 1'b0;
            psram_cmd_wr <= 1'b0;
            psram_addr   <= 21'd0;
            rd_word0     <= 64'd0;
            rd_word1     <= 64'd0;
            rd_valid     <= 1'b0;
            latch_idx    <= 10'd0;
        end else begin
            arb_rel   <= 1'b0;
            psram_req <= 1'b0;
            rd_valid  <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (fetch_req) begin
                        latch_idx <= fetch_idx;
                        arb_req   <= 1'b1;
                        state     <= ST_ARB;
                    end
                end

                ST_ARB: begin
                    if (arb_grant) state <= ST_RD0;
                end

                ST_RD0: begin
                    if (!psram_busy) begin
                        psram_req    <= 1'b1;
                        psram_cmd_wr <= 1'b0;
                        psram_addr   <= BASE_ADDR[20:0] + {latch_idx, 1'b0};
                        state        <= ST_RD0W;
                    end
                end

                ST_RD0W: begin
                    if (psram_rd_valid) begin
                        rd_word0 <= psram_rd_data;
                        state    <= ST_RD1;
                    end
                end

                ST_RD1: begin
                    if (!psram_busy) begin
                        psram_req    <= 1'b1;
                        psram_cmd_wr <= 1'b0;
                        psram_addr   <= BASE_ADDR[20:0] + {latch_idx, 1'b1};
                        state        <= ST_RD1W;
                    end
                end

                ST_RD1W: begin
                    if (psram_rd_valid) begin
                        rd_word1 <= psram_rd_data;
                        state    <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    rd_valid <= 1'b1;
                    arb_req  <= 1'b0;
                    arb_rel  <= 1'b1;
                    state    <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
