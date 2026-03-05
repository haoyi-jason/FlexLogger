// spi_slave_mode0_sclk.v
// SPI Slave, Mode 0 (CPOL=0, CPHA=0).
// Data shifted on the SCLK domain; output registered on falling SCLK.
//
// The module shifts in MOSI LSB-first or MSB-first (parameter).
// A byte is complete after 8 SCLK cycles.  rx_valid pulses for one SCLK cycle
// when a full byte has been received.  tx_data must be loaded before or exactly
// when the previous byte's last bit is being shifted out.
//
// CS_N must be low for the transfer to be active.

`timescale 1ns/1ps

module spi_slave_mode0_sclk #(
    parameter MSB_FIRST = 1   // 1 = MSB first, 0 = LSB first
) (
    // SPI pins (external)
    input  wire       spi_sclk,
    input  wire       spi_cs_n,
    input  wire       spi_mosi,
    output reg        spi_miso,

    // SCLK-domain byte interface
    output reg  [7:0] rx_byte,
    output reg        rx_valid,    // 1 SCLK cycle pulse: new byte in rx_byte
    input  wire [7:0] tx_byte,     // byte to send; sampled after each byte boundary
    output reg        tx_ready     // 1 SCLK cycle pulse: tx_byte has been consumed
);

    reg [2:0] bit_cnt;
    reg [7:0] shift_rx;
    reg [7:0] shift_tx;

    always @(posedge spi_sclk or posedge spi_cs_n) begin
        if (spi_cs_n) begin
            bit_cnt  <= 3'd0;
            shift_rx <= 8'd0;
            rx_valid <= 1'b0;
            tx_ready <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            tx_ready <= 1'b0;

            // Shift in MOSI
            if (MSB_FIRST)
                shift_rx <= {shift_rx[6:0], spi_mosi};
            else
                shift_rx <= {spi_mosi, shift_rx[7:1]};

            bit_cnt <= bit_cnt + 1'b1;

            if (bit_cnt == 3'd7) begin
                // Byte complete
                if (MSB_FIRST)
                    rx_byte <= {shift_rx[6:0], spi_mosi};
                else
                    rx_byte <= {spi_mosi, shift_rx[7:1]};
                rx_valid <= 1'b1;
                tx_ready <= 1'b1;
                shift_tx <= tx_byte;   // load next byte to send
            end
        end
    end

    // MISO: shift out on falling SCLK
    // On CS de-assertion, pre-load the first byte so bit 7 (MSB) is ready.
    always @(negedge spi_sclk or posedge spi_cs_n) begin
        if (spi_cs_n) begin
            shift_tx <= tx_byte;
            spi_miso <= MSB_FIRST ? tx_byte[7] : tx_byte[0];
        end else begin
            if (MSB_FIRST) begin
                // Shift first, then present next bit on MISO
                shift_tx <= {shift_tx[6:0], 1'b0};
                spi_miso <= shift_tx[6];
            end else begin
                shift_tx <= {1'b0, shift_tx[7:1]};
                spi_miso <= shift_tx[1];
            end
        end
    end

endmodule
