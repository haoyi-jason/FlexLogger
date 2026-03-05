// psram_wrap.v
// Wrapper around Gowin "PSRAM Memory Interface HS V2.0" IP.
// Presents a simple single-transaction interface on psram_clk domain.
//
// Parameters
//   CMD_WRITE    – value of 'cmd' that means WRITE (default 1)
//   ADDR_IS_WORD – when 1, 'addr' is in 64-bit word units (default 1)
//
// Interface (psram_clk domain):
//   req         – pulse: start a transaction
//   cmd_wr      – 1=write, 0=read
//   addr        – 21-bit word address
//   wr_data     – 64-bit write data
//   data_mask   – 8-bit byte-enable mask (active HIGH bytes to MASK, i.e. 0=write)
//   rd_data     – 64-bit read data (valid when rd_valid=1)
//   rd_valid    – read data valid pulse
//   busy        – transaction in progress
//   init_calib  – PSRAM IP calibration done

`timescale 1ns/1ps

module psram_wrap #(
    parameter CMD_WRITE    = 1,    // cmd value that triggers a write
    parameter ADDR_IS_WORD = 1     // 1 = addr indexes 64-bit words
) (
    // PSRAM IP clocking / reset
    input  wire        clk_d,          // domain clock (= psram_clk / clk_out)
    input  wire        memory_clk,     // high-speed PLL clock for PSRAM IO
    input  wire        memory_clk_p,   // phase-shifted memory_clk (90 or 180 deg)
    input  wire        pll_lock,
    input  wire        rst_n,

    // PSRAM physical pins (connect to top-level ports)
    output wire [1:0]  O_psram_ck,
    output wire [1:0]  O_psram_ck_n,
    inout  wire [15:0] IO_psram_dq,
    inout  wire [1:0]  IO_psram_rwds,
    output wire [1:0]  O_psram_cs_n,
    output wire [1:0]  O_psram_reset_n,

    // User interface (clk_d domain)
    input  wire        req,
    input  wire        cmd_wr,
    input  wire [20:0] addr,
    input  wire [63:0] wr_data,
    input  wire [7:0]  data_mask,
    output reg  [63:0] rd_data,
    output reg         rd_valid,
    output wire        busy,
    output wire        init_calib
);

    // -----------------------------------------------------------------------
    // Internal signals
    // -----------------------------------------------------------------------
    reg         ip_cmd_en;
    reg         ip_cmd;
    reg  [20:0] ip_addr;
    reg  [63:0] ip_wr_data;
    reg  [7:0]  ip_data_mask;

    wire [63:0] ip_rd_data;
    wire        ip_rd_valid;

    // -----------------------------------------------------------------------
    // State machine
    // -----------------------------------------------------------------------
    localparam ST_IDLE   = 2'd0;
    localparam ST_CMD    = 2'd1;
    localparam ST_WAIT   = 2'd2;

    reg [1:0] state;
    reg       pending_read;

    assign busy = (state != ST_IDLE);

    always @(posedge clk_d or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            ip_cmd_en    <= 1'b0;
            ip_cmd       <= 1'b0;
            ip_addr      <= 21'd0;
            ip_wr_data   <= 64'd0;
            ip_data_mask <= 8'd0;
            pending_read <= 1'b0;
            rd_valid     <= 1'b0;
            rd_data      <= 64'd0;
        end else begin
            rd_valid  <= 1'b0;
            ip_cmd_en <= 1'b0;

            case (state)
                ST_IDLE: begin
                    if (req && init_calib) begin
                        ip_cmd       <= cmd_wr ? CMD_WRITE[0] : ~CMD_WRITE[0];
                        ip_addr      <= ADDR_IS_WORD ? addr : {addr[20:0]};
                        ip_wr_data   <= wr_data;
                        ip_data_mask <= data_mask;
                        ip_cmd_en    <= 1'b1;
                        pending_read <= ~cmd_wr;
                        state        <= ST_CMD;
                    end
                end

                ST_CMD: begin
                    // cmd_en was pulsed last cycle; now wait for response
                    state <= ST_WAIT;
                end

                ST_WAIT: begin
                    if (pending_read) begin
                        if (ip_rd_valid) begin
                            rd_data  <= ip_rd_data;
                            rd_valid <= 1'b1;
                            state    <= ST_IDLE;
                        end
                    end else begin
                        // Write: IP acknowledges quickly; one cycle is enough
                        state <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // Gowin PSRAM IP instantiation
    // -----------------------------------------------------------------------
    PSRAM_Memory_Interface_HS_V2_Top u_psram_ip (
        .clk_d          (clk_d),
        .memory_clk     (memory_clk),
        .memory_clk_p   (memory_clk_p),
        .pll_lock       (pll_lock),
        .rst_n          (rst_n),
        .O_psram_ck     (O_psram_ck),
        .O_psram_ck_n   (O_psram_ck_n),
        .IO_psram_dq    (IO_psram_dq),
        .IO_psram_rwds  (IO_psram_rwds),
        .O_psram_cs_n   (O_psram_cs_n),
        .O_psram_reset_n(O_psram_reset_n),
        .wr_data        (ip_wr_data),
        .rd_data        (ip_rd_data),
        .rd_data_valid  (ip_rd_valid),
        .addr           (ip_addr),
        .cmd            (ip_cmd),
        .cmd_en         (ip_cmd_en),
        .init_calib     (init_calib),
        .clk_out        (),             // connected externally via clk_d
        .data_mask      (ip_data_mask)
    );

endmodule
