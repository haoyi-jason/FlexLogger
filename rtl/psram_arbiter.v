// psram_arbiter.v
// Simple priority arbiter for shared PSRAM access.
// Three clients (ADC DMA, IMU DMA, Fetch engine) are arbitrated
// with fixed priority: Fetch > IMU DMA > ADC DMA (Fetch is latency-critical).
//
// Each client has a req/grant/rel handshake:
//   req  – client requests the bus
//   grant – arbiter grants the bus (single-cycle pulse); client may begin
//   rel  – client releases the bus (single-cycle pulse)
//
// Only one client is active at a time. A grant is issued when the bus is idle
// and the client has raised req.
//
// The arbiter also muxes the PSRAM control signals to psram_wrap.

`timescale 1ns/1ps

module psram_arbiter (
    input  wire        clk,
    input  wire        rst_n,

    // --- Client 0: ADC DMA (lowest priority) ---
    input  wire        c0_req,
    output reg         c0_grant,
    input  wire        c0_rel,
    input  wire        c0_psram_req,
    input  wire        c0_cmd_wr,
    input  wire [20:0] c0_addr,
    input  wire [63:0] c0_wr_data,
    input  wire [7:0]  c0_mask,

    // --- Client 1: IMU DMA ---
    input  wire        c1_req,
    output reg         c1_grant,
    input  wire        c1_rel,
    input  wire        c1_psram_req,
    input  wire        c1_cmd_wr,
    input  wire [20:0] c1_addr,
    input  wire [63:0] c1_wr_data,
    input  wire [7:0]  c1_mask,

    // --- Client 2: Fetch engine (highest priority) ---
    input  wire        c2_req,
    output reg         c2_grant,
    input  wire        c2_rel,
    input  wire        c2_psram_req,
    input  wire        c2_cmd_wr,
    input  wire [20:0] c2_addr,
    input  wire [63:0] c2_wr_data,
    input  wire [7:0]  c2_mask,

    // --- PSRAM wrap interface ---
    output reg         psram_req,
    output reg         psram_cmd_wr,
    output reg  [20:0] psram_addr,
    output reg  [63:0] psram_wr_data,
    output reg  [7:0]  psram_mask,
    input  wire [63:0] psram_rd_data,
    input  wire        psram_rd_valid,
    input  wire        psram_busy,

    // Read data returned to all clients (they know who is active)
    output wire [63:0] rd_data,
    output wire        rd_valid
);

    assign rd_data  = psram_rd_data;
    assign rd_valid = psram_rd_valid;

    // Who holds the bus (one-hot or none)
    localparam OWNER_NONE = 2'd3;
    localparam OWNER_0    = 2'd0;
    localparam OWNER_1    = 2'd1;
    localparam OWNER_2    = 2'd2;

    reg [1:0] owner;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            owner    <= OWNER_NONE;
            c0_grant <= 1'b0;
            c1_grant <= 1'b0;
            c2_grant <= 1'b0;
        end else begin
            c0_grant <= 1'b0;
            c1_grant <= 1'b0;
            c2_grant <= 1'b0;

            // Release
            if ((owner == OWNER_0 && c0_rel) ||
                (owner == OWNER_1 && c1_rel) ||
                (owner == OWNER_2 && c2_rel)) begin
                owner <= OWNER_NONE;
            end

            // Grant (priority: 2 > 1 > 0)
            if (owner == OWNER_NONE) begin
                if (c2_req) begin
                    owner    <= OWNER_2;
                    c2_grant <= 1'b1;
                end else if (c1_req) begin
                    owner    <= OWNER_1;
                    c1_grant <= 1'b1;
                end else if (c0_req) begin
                    owner    <= OWNER_0;
                    c0_grant <= 1'b1;
                end
            end
        end
    end

    // Mux PSRAM control signals based on owner
    always @(*) begin
        case (owner)
            OWNER_0: begin
                psram_req    = c0_psram_req;
                psram_cmd_wr = c0_cmd_wr;
                psram_addr   = c0_addr;
                psram_wr_data= c0_wr_data;
                psram_mask   = c0_mask;
            end
            OWNER_1: begin
                psram_req    = c1_psram_req;
                psram_cmd_wr = c1_cmd_wr;
                psram_addr   = c1_addr;
                psram_wr_data= c1_wr_data;
                psram_mask   = c1_mask;
            end
            OWNER_2: begin
                psram_req    = c2_psram_req;
                psram_cmd_wr = c2_cmd_wr;
                psram_addr   = c2_addr;
                psram_wr_data= c2_wr_data;
                psram_mask   = c2_mask;
            end
            default: begin
                psram_req    = 1'b0;
                psram_cmd_wr = 1'b0;
                psram_addr   = 21'd0;
                psram_wr_data= 64'd0;
                psram_mask   = 8'hFF;
            end
        endcase
    end

endmodule
