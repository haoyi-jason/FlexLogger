// spi_cmd_mailbox.v
// Bridges the SPI-slave byte stream (spi_sclk domain) to the
// PSRAM command executor (psram_clk domain).
//
// Protocol:
//   MCU sends fixed 8-byte command frames with CS_N low.
//   After the 8th byte is received the full command is pushed into an
//   async FIFO (cmd_fifo) for the psram_clk side to consume.
//
//   Responses are pushed by the psram_clk side into resp_fifo (1 byte per
//   entry).  The SPI master clocks out dummy bytes to receive the response.
//   The first tx_byte pre-loaded is 0x00 (busy indicator); the mailbox
//   switches to actual response bytes as they arrive.
//
// CMD frame layout (8 bytes):
//   [0]   opcode
//   [1]   payload byte 0
//   [2]   payload byte 1
//   [3]   payload byte 2
//   [4]   payload byte 3
//   [5]   payload byte 4
//   [6]   payload byte 5
//   [7]   payload byte 6
//
// Async FIFOs:
//   cmd_fifo:  64-bit wide, depth 4 (one 8-byte command per entry, low-latency)
//   resp_fifo: 8-bit wide, depth 32 (up to 32 response bytes queued)

`timescale 1ns/1ps

module spi_cmd_mailbox (
    // SPI slave interface (spi_sclk domain)
    input  wire        spi_sclk,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output wire        spi_miso,

    // spi_sclk domain reset (active low)
    input  wire        spi_rst_n,

    // psram_clk domain
    input  wire        psram_clk,
    input  wire        psram_rst_n,

    // CMD output (psram_clk domain) — pop one 64-bit word = 8 bytes
    output wire [63:0] cmd_data,
    output wire        cmd_valid,
    input  wire        cmd_ready,   // consumer ACK

    // Response input (psram_clk domain) — push one byte at a time
    input  wire [7:0]  resp_data,
    input  wire        resp_push,
    output wire        resp_full
);

    // -----------------------------------------------------------------------
    // SPI slave byte engine
    // -----------------------------------------------------------------------
    wire [7:0] rx_byte;
    wire       rx_valid;
    wire [7:0] tx_byte;
    wire       tx_ready;

    spi_slave_mode0_sclk #(.MSB_FIRST(1)) u_spi (
        .spi_sclk (spi_sclk),
        .spi_cs_n (spi_cs_n),
        .spi_mosi (spi_mosi),
        .spi_miso (spi_miso),
        .rx_byte  (rx_byte),
        .rx_valid (rx_valid),
        .tx_byte  (tx_byte),
        .tx_ready (tx_ready)
    );

    // -----------------------------------------------------------------------
    // Receive 8 bytes and pack into a 64-bit word
    // -----------------------------------------------------------------------
    reg [63:0] cmd_shift;
    reg [2:0]  byte_cnt;

    // CMD async FIFO: 64-bit, depth 4
    wire        cmd_wr_full;
    wire        cmd_rd_empty;
    reg         cmd_wr_en;
    reg  [63:0] cmd_wr_data;

    async_fifo_simple #(.DATA_WIDTH(64), .DEPTH_LOG2(2)) u_cmd_fifo (
        .wr_clk   (spi_sclk),
        .wr_rst_n (spi_rst_n),
        .wr_en    (cmd_wr_en),
        .wr_data  (cmd_wr_data),
        .full     (cmd_wr_full),

        .rd_clk   (psram_clk),
        .rd_rst_n (psram_rst_n),
        .rd_en    (cmd_ready && !cmd_rd_empty),
        .rd_data  (cmd_data),
        .empty    (cmd_rd_empty)
    );

    assign cmd_valid = !cmd_rd_empty;

    always @(posedge spi_sclk or posedge spi_cs_n) begin
        if (spi_cs_n) begin
            byte_cnt    <= 3'd0;
            cmd_shift   <= 64'd0;
            cmd_wr_en   <= 1'b0;
            cmd_wr_data <= 64'd0;
        end else begin
            cmd_wr_en <= 1'b0;
            if (rx_valid) begin
                // Pack bytes: byte 0 in bits[63:56], byte 7 in bits[7:0]
                cmd_shift <= {cmd_shift[55:0], rx_byte};
                byte_cnt  <= byte_cnt + 1'b1;
                if (byte_cnt == 3'd7) begin
                    if (!cmd_wr_full) begin
                        cmd_wr_data <= {cmd_shift[55:0], rx_byte};
                        cmd_wr_en   <= 1'b1;
                    end
                    byte_cnt <= 3'd0;
                end
            end
        end
    end

    // -----------------------------------------------------------------------
    // Response FIFO: 8-bit, depth 32 (psram_clk → spi_sclk)
    // -----------------------------------------------------------------------
    wire        resp_rd_empty;
    wire [7:0]  resp_rd_data;
    reg         resp_rd_en;

    async_fifo_simple #(.DATA_WIDTH(8), .DEPTH_LOG2(5)) u_resp_fifo (
        .wr_clk   (psram_clk),
        .wr_rst_n (psram_rst_n),
        .wr_en    (resp_push),
        .wr_data  (resp_data),
        .full     (resp_full),

        .rd_clk   (spi_sclk),
        .rd_rst_n (spi_rst_n),
        .rd_en    (resp_rd_en),
        .rd_data  (resp_rd_data),
        .empty    (resp_rd_empty)
    );

    // Drive tx_byte from resp_fifo; 0x00 when FIFO is empty
    reg [7:0] tx_hold;

    assign tx_byte = tx_hold;

    always @(posedge spi_sclk or posedge spi_cs_n) begin
        if (spi_cs_n) begin
            tx_hold    <= 8'h00;
            resp_rd_en <= 1'b0;
        end else begin
            resp_rd_en <= 1'b0;
            if (tx_ready) begin
                if (!resp_rd_empty) begin
                    tx_hold    <= resp_rd_data;
                    resp_rd_en <= 1'b1;
                end else begin
                    tx_hold <= 8'h00;
                end
            end
        end
    end

endmodule
