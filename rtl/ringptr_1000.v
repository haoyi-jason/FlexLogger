// ringptr_1000.v
// Circular-buffer pointer management for a fixed 1000-entry ring.
// All signals synchronous to clk (psram_clk domain).
//
// The module tracks:
//   wr_idx   – next slot to write (0..999)
//   rd_idx   – next slot to read  (0..999)
//   pending  – number of unread entries (0..1000)
//   overflow – cumulative overflow counter (saturates at 16-bit max)
//
// Control:
//   push   – write one entry (always succeeds; overwrites oldest when full)
//   pop    – read one entry  (ignored when pending==0)

`timescale 1ns/1ps

module ringptr_1000 (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        push,
    input  wire        pop,

    output reg  [9:0]  wr_idx,      // current write index (before push)
    output reg  [9:0]  rd_idx,      // current read  index (before pop)
    output reg  [10:0] pending,     // 0..1000
    output reg  [15:0] overflow     // cumulative overwrite count
);

    localparam DEPTH = 10'd999;     // max index (depth-1)

    wire full  = (pending == 11'd1000);
    wire empty = (pending == 11'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_idx   <= 10'd0;
            rd_idx   <= 10'd0;
            pending  <= 11'd0;
            overflow <= 16'd0;
        end else begin
            case ({push, pop})
                2'b10: begin // push only
                    wr_idx  <= (wr_idx == DEPTH) ? 10'd0 : wr_idx + 1'b1;
                    if (full) begin
                        // Overwrite oldest: advance rd_idx as well
                        rd_idx   <= (rd_idx == DEPTH) ? 10'd0 : rd_idx + 1'b1;
                        overflow <= (overflow == 16'hFFFF) ? overflow : overflow + 1'b1;
                    end else begin
                        pending <= pending + 1'b1;
                    end
                end
                2'b01: begin // pop only
                    if (!empty) begin
                        rd_idx  <= (rd_idx == DEPTH) ? 10'd0 : rd_idx + 1'b1;
                        pending <= pending - 1'b1;
                    end
                end
                2'b11: begin // push + pop simultaneously
                    wr_idx <= (wr_idx == DEPTH) ? 10'd0 : wr_idx + 1'b1;
                    if (!empty)
                        rd_idx <= (rd_idx == DEPTH) ? 10'd0 : rd_idx + 1'b1;
                    else
                        pending <= pending + 1'b1; // was empty, pop has no effect
                end
                default: ; // no-op
            endcase
        end
    end

endmodule
