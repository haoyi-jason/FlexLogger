// psram_ringbuf_store.v
// Writes one 16-byte record (2 x 64-bit PSRAM words) to a circular buffer.
// Operates on psram_clk domain; uses psram_wrap via the arbiter request bus.
//
// Handshake with arbiter (psram_arbiter):
//   arb_req    – assert to request bus
//   arb_grant  – arbiter grants access (single-cycle)
//   arb_rel    – pulse to release bus after all writes done
//
// Usage:
//   1. Caller places word0/word1 on din_* and asserts push.
//   2. Module acquires PSRAM arbiter, writes two consecutive words,
//      advances wr pointer via ringptr push output, releases arbiter.

`timescale 1ns/1ps

module psram_ringbuf_store #(
    parameter BASE_ADDR    = 21'd0,   // PSRAM word-address base of buffer
    parameter WORDS_PER    = 2        // PSRAM 64-bit words per record
) (
    input  wire        clk,
    input  wire        rst_n,

    // Data input (one record = 2 words)
    input  wire [63:0] din_word0,
    input  wire [63:0] din_word1,
    input  wire        push,          // pulse: new record to store
    output wire        busy,          // high while storing

    // Ring pointer interface
    output reg         ptr_push,      // pulse to ringptr_1000
    input  wire [9:0]  wr_idx,        // current write index from ringptr

    // PSRAM arbiter request bus
    output reg         arb_req,
    input  wire        arb_grant,
    output reg         arb_rel,

    // PSRAM write port (to arbiter, then to psram_wrap)
    output reg         psram_req,
    output reg         psram_cmd_wr,
    output reg  [20:0] psram_addr,
    output reg  [63:0] psram_wr_data,
    output reg  [7:0]  psram_mask,    // 0x00 = write all bytes
    input  wire        psram_busy
);

    localparam ST_IDLE    = 3'd0;
    localparam ST_ARB     = 3'd1;
    localparam ST_WR0     = 3'd2;
    localparam ST_WR0W    = 3'd3;
    localparam ST_WR1     = 3'd4;
    localparam ST_WR1W    = 3'd5;
    localparam ST_DONE    = 3'd6;

    reg [2:0]  state;
    reg [63:0] latch0, latch1;
    reg [9:0]  latch_idx;
    reg [20:0] base_addr;

    assign busy = (state != ST_IDLE);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            arb_req      <= 1'b0;
            arb_rel      <= 1'b0;
            psram_req    <= 1'b0;
            psram_cmd_wr <= 1'b1;
            psram_addr   <= 21'd0;
            psram_wr_data<= 64'd0;
            psram_mask   <= 8'h00;
            ptr_push     <= 1'b0;
            latch0       <= 64'd0;
            latch1       <= 64'd0;
            latch_idx    <= 10'd0;
        end else begin
            arb_rel   <= 1'b0;
            psram_req <= 1'b0;
            ptr_push  <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (push && !busy) begin
                        latch0    <= din_word0;
                        latch1    <= din_word1;
                        latch_idx <= wr_idx;
                        arb_req   <= 1'b1;
                        state     <= ST_ARB;
                    end
                end

                ST_ARB: begin
                    if (arb_grant) begin
                        state <= ST_WR0;
                    end
                end

                ST_WR0: begin
                    if (!psram_busy) begin
                        psram_req    <= 1'b1;
                        psram_cmd_wr <= 1'b1;
                        psram_addr   <= BASE_ADDR[20:0] +
                                        {latch_idx, 1'b0};  // *2 words
                        psram_wr_data<= latch0;
                        psram_mask   <= 8'h00;
                        state        <= ST_WR0W;
                    end
                end

                ST_WR0W: begin
                    if (!psram_busy && !psram_req) state <= ST_WR1;
                end

                ST_WR1: begin
                    if (!psram_busy) begin
                        psram_req    <= 1'b1;
                        psram_cmd_wr <= 1'b1;
                        psram_addr   <= BASE_ADDR[20:0] +
                                        {latch_idx, 1'b1};  // +1
                        psram_wr_data<= latch1;
                        psram_mask   <= 8'h00;
                        state        <= ST_WR1W;
                    end
                end

                ST_WR1W: begin
                    if (!psram_busy && !psram_req) state <= ST_DONE;
                end

                ST_DONE: begin
                    ptr_push <= 1'b1;   // advance ring pointer
                    arb_req  <= 1'b0;
                    arb_rel  <= 1'b1;
                    state    <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
