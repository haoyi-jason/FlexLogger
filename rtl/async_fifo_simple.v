// async_fifo_simple.v
// Asynchronous FIFO using Gray-code pointers for CDC.
// Verilog-2001, synthesizable for Gowin EDA.
//
// Parameters:
//   DATA_WIDTH  – width of each data word
//   DEPTH_LOG2  – log2 of FIFO depth (depth = 2**DEPTH_LOG2)
//
// Write side: wr_clk domain
//   wr_en, wr_data → full
// Read side: rd_clk domain
//   rd_en → rd_data, empty

`timescale 1ns/1ps

module async_fifo_simple #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH_LOG2 = 4          // depth = 16
) (
    input  wire                  wr_clk,
    input  wire                  wr_rst_n,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire                  full,

    input  wire                  rd_clk,
    input  wire                  rd_rst_n,
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  empty
);

    localparam DEPTH = 1 << DEPTH_LOG2;

    // ---------------------------------------------------------------------------
    // Memory
    // ---------------------------------------------------------------------------
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // ---------------------------------------------------------------------------
    // Write pointer (binary + gray) – wr_clk domain
    // ---------------------------------------------------------------------------
    reg [DEPTH_LOG2:0] wr_ptr_bin;
    reg [DEPTH_LOG2:0] wr_ptr_gray;

    wire [DEPTH_LOG2:0] wr_ptr_bin_next = wr_ptr_bin + 1'b1;
    wire [DEPTH_LOG2:0] wr_ptr_gray_next = (wr_ptr_bin_next >> 1) ^ wr_ptr_bin_next;

    // Synchronize rd_ptr_gray into wr_clk domain
    reg [DEPTH_LOG2:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            rd_ptr_gray_sync1 <= {(DEPTH_LOG2+1){1'b0}};
            rd_ptr_gray_sync2 <= {(DEPTH_LOG2+1){1'b0}};
        end else begin
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end
    end

    assign full = (wr_ptr_gray_next == {~rd_ptr_gray_sync2[DEPTH_LOG2:DEPTH_LOG2-1],
                                         rd_ptr_gray_sync2[DEPTH_LOG2-2:0]});

    always @(posedge wr_clk or negedge wr_rst_n) begin
        if (!wr_rst_n) begin
            wr_ptr_bin  <= {(DEPTH_LOG2+1){1'b0}};
            wr_ptr_gray <= {(DEPTH_LOG2+1){1'b0}};
        end else if (wr_en && !full) begin
            mem[wr_ptr_bin[DEPTH_LOG2-1:0]] <= wr_data;
            wr_ptr_bin  <= wr_ptr_bin_next;
            wr_ptr_gray <= wr_ptr_gray_next;
        end
    end

    // ---------------------------------------------------------------------------
    // Read pointer (binary + gray) – rd_clk domain
    // ---------------------------------------------------------------------------
    reg [DEPTH_LOG2:0] rd_ptr_bin;
    reg [DEPTH_LOG2:0] rd_ptr_gray;

    wire [DEPTH_LOG2:0] rd_ptr_bin_next  = rd_ptr_bin + 1'b1;
    wire [DEPTH_LOG2:0] rd_ptr_gray_next = (rd_ptr_bin_next >> 1) ^ rd_ptr_bin_next;

    // Synchronize wr_ptr_gray into rd_clk domain
    reg [DEPTH_LOG2:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            wr_ptr_gray_sync1 <= {(DEPTH_LOG2+1){1'b0}};
            wr_ptr_gray_sync2 <= {(DEPTH_LOG2+1){1'b0}};
        end else begin
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end
    end

    assign empty = (rd_ptr_gray == wr_ptr_gray_sync2);

    always @(posedge rd_clk or negedge rd_rst_n) begin
        if (!rd_rst_n) begin
            rd_ptr_bin  <= {(DEPTH_LOG2+1){1'b0}};
            rd_ptr_gray <= {(DEPTH_LOG2+1){1'b0}};
        end else if (rd_en && !empty) begin
            rd_ptr_bin  <= rd_ptr_bin_next;
            rd_ptr_gray <= rd_ptr_gray_next;
        end
    end

    assign rd_data = mem[rd_ptr_bin[DEPTH_LOG2-1:0]];

endmodule
