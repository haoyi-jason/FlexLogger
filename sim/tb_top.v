// tb_top.v
// Top-level simulation testbench for FlexLogger.
//
// Exercises:
//   - 50 MHz clk50 generation
//   - Reset sequence
//   - SPI command transactions (STATUS, READ_REG, WRITE_REG,
//     POP_ADC_FRAME, POP_IMU_FRAME)
//   - HOST_IRQ and init_calib monitoring
//
// SPI is mode 0 (CPOL=0, CPHA=0), 8-byte frames, ~5 MHz (100 ns half-period).
//
// Compile with:
//   iverilog -g2005 -DSIMULATION -o sim/sim_top \
//     rtl/async_fifo_simple.v rtl/psram_addr_map.v rtl/ringptr_1000.v \
//     rtl/regfile_core.v rtl/psram_wrap.v rtl/psram_arbiter.v \
//     rtl/psram_ringbuf_store.v rtl/psram_ringbuf_fetch.v \
//     rtl/psram_cmd_exec.v rtl/ad7606_ctrl.v rtl/adc_psram_dma.v \
//     rtl/i2c_master.v rtl/bmi270_driver.v rtl/imu_avg.v \
//     rtl/imu_psram_dma.v rtl/spi_slave_mode0_sclk.v \
//     rtl/spi_cmd_mailbox.v rtl/gowin_rpll_stub.v \
//     rtl/gowin_psram_stub.v rtl/top.v sim/tb_top.v
//   vvp sim/sim_top

`timescale 1ns/1ps

module tb_top;

    // =========================================================================
    // DUT ports
    // =========================================================================
    reg         clk50;
    reg         rst_n_ext;

    // ADC
    wire        adc_convst;
    reg         adc_busy;
    wire        adc_rd_n;
    wire        adc_cs_n;
    wire        adc_reset;
    wire [2:0]  adc_os;
    wire        adc_range;
    wire        adc_stby_n;
    reg  [15:0] adc_db;

    // IMU I2C (open-drain emulation – pull high, OE pulls low)
    wire        imu0_scl_oe, imu0_sda_oe; reg imu0_sda_in;
    wire        imu1_scl_oe, imu1_sda_oe; reg imu1_sda_in;
    wire        imu2_scl_oe, imu2_sda_oe; reg imu2_sda_in;
    wire        imu3_scl_oe, imu3_sda_oe; reg imu3_sda_in;

    // PSRAM pins
    wire [1:0]  O_psram_ck;
    wire [1:0]  O_psram_ck_n;
    wire [15:0] IO_psram_dq;
    wire [1:0]  IO_psram_rwds;
    wire [1:0]  O_psram_cs_n;
    wire [1:0]  O_psram_reset_n;

    // Bidirectional PSRAM pins: driven by pullup when not driven by DUT
    reg  [15:0] psram_dq_drive;
    reg  [1:0]  psram_rwds_drive;
    reg         psram_dq_oe;
    reg         psram_rwds_oe;

    assign IO_psram_dq   = psram_dq_oe   ? psram_dq_drive   : 16'hz;
    assign IO_psram_rwds = psram_rwds_oe ? psram_rwds_drive  : 2'hz;

    // SPI
    reg         spi_sclk;
    reg         spi_cs_n;
    reg         spi_mosi;
    wire        spi_miso;

    // IRQ
    wire        HOST_IRQ;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    top u_top (
        .clk50          (clk50),
        .rst_n_ext      (rst_n_ext),
        .adc_convst     (adc_convst),
        .adc_busy       (adc_busy),
        .adc_rd_n       (adc_rd_n),
        .adc_cs_n       (adc_cs_n),
        .adc_reset      (adc_reset),
        .adc_os         (adc_os),
        .adc_range      (adc_range),
        .adc_stby_n     (adc_stby_n),
        .adc_db         (adc_db),
        .imu0_scl_oe    (imu0_scl_oe),
        .imu0_sda_oe    (imu0_sda_oe),
        .imu0_sda_in    (imu0_sda_in),
        .imu1_scl_oe    (imu1_scl_oe),
        .imu1_sda_oe    (imu1_sda_oe),
        .imu1_sda_in    (imu1_sda_in),
        .imu2_scl_oe    (imu2_scl_oe),
        .imu2_sda_oe    (imu2_sda_oe),
        .imu2_sda_in    (imu2_sda_in),
        .imu3_scl_oe    (imu3_scl_oe),
        .imu3_sda_oe    (imu3_sda_oe),
        .imu3_sda_in    (imu3_sda_in),
        .O_psram_ck     (O_psram_ck),
        .O_psram_ck_n   (O_psram_ck_n),
        .IO_psram_dq    (IO_psram_dq),
        .IO_psram_rwds  (IO_psram_rwds),
        .O_psram_cs_n   (O_psram_cs_n),
        .O_psram_reset_n(O_psram_reset_n),
        .spi_sclk       (spi_sclk),
        .spi_cs_n       (spi_cs_n),
        .spi_mosi       (spi_mosi),
        .spi_miso       (spi_miso),
        .HOST_IRQ       (HOST_IRQ)
    );

    // =========================================================================
    // Clock generation – 50 MHz (20 ns period)
    // =========================================================================
    initial clk50 = 1'b0;
    always #10 clk50 = ~clk50;

    // =========================================================================
    // SPI idle state
    // =========================================================================
    initial begin
        spi_sclk  = 1'b0;
        spi_cs_n  = 1'b1;
        spi_mosi  = 1'b0;
    end

    // =========================================================================
    // ADC / IMU stimulus defaults
    // =========================================================================
    initial begin
        adc_busy       = 1'b0;
        adc_db         = 16'h0000;
        imu0_sda_in    = 1'b1;
        imu1_sda_in    = 1'b1;
        imu2_sda_in    = 1'b1;
        imu3_sda_in    = 1'b1;
        psram_dq_drive = 16'h0000;
        psram_rwds_drive = 2'b00;
        psram_dq_oe    = 1'b0;
        psram_rwds_oe  = 1'b0;
    end

    // =========================================================================
    // Monitor
    // =========================================================================
    initial $monitor("[%0t ns] HOST_IRQ=%b init_calib=%b",
                     $time, HOST_IRQ,
                     u_top.init_calib);

    // =========================================================================
    // SPI helper task – transmit one 8-bit byte (MSB first, mode 0)
    // Returns received byte in rx_byte.
    // Half-period: 100 ns → ~5 MHz.
    // =========================================================================
    reg [7:0] spi_rx_byte;

    task spi_send_byte;
        input [7:0] tx;
        integer     b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                spi_mosi = tx[b];
                #100;                        // setup before rising edge
                spi_sclk = 1'b1;
                spi_rx_byte = {spi_rx_byte[6:0], spi_miso}; // sample MISO
                #100;
                spi_sclk = 1'b0;
            end
        end
    endtask

    // =========================================================================
    // SPI helper task – send one complete 8-byte command frame.
    // cmd[63:56] = byte 0 (opcode), cmd[55:0] = payload bytes 1-7.
    // =========================================================================
    task spi_send_frame;
        input [63:0] cmd;
        integer      n;
        begin
            spi_cs_n = 1'b0;
            #200;
            for (n = 7; n >= 0; n = n - 1)
                spi_send_byte(cmd[n*8 +: 8]);
            #200;
            spi_cs_n = 1'b1;
            #500;
        end
    endtask

    // =========================================================================
    // SPI helper task – read N response bytes (send dummy 0x00 frames)
    // =========================================================================
    task spi_read_resp;
        input integer num_bytes;
        integer k;
        begin
            spi_cs_n = 1'b0;
            #200;
            for (k = 0; k < num_bytes; k = k + 1) begin
                spi_send_byte(8'h00);
                $display("[%0t ns] resp[%0d] = 0x%02h", $time, k, spi_rx_byte);
            end
            #200;
            spi_cs_n = 1'b1;
            #500;
        end
    endtask

    // =========================================================================
    // Main stimulus
    // =========================================================================
    integer timeout_cnt;

    initial begin
        $dumpfile("tb_top.vcd");
        $dumpvars(0, tb_top);

        // -------------------------------------------------------------------
        // 1. Reset sequence
        // -------------------------------------------------------------------
        rst_n_ext = 1'b0;
        repeat (10) @(posedge clk50);
        rst_n_ext = 1'b1;
        $display("[%0t ns] Reset released", $time);

        // -------------------------------------------------------------------
        // 2. Wait for PSRAM init_calib
        // -------------------------------------------------------------------
        $display("[%0t ns] Waiting for init_calib...", $time);
        timeout_cnt = 0;
        while (!u_top.init_calib && timeout_cnt < 100000) begin
            @(posedge clk50);
            timeout_cnt = timeout_cnt + 1;
        end
        if (!u_top.init_calib) begin
            $display("TIMEOUT waiting for init_calib");
            $finish;
        end
        $display("[%0t ns] init_calib asserted after %0d clk50 cycles",
                 $time, timeout_cnt);

        // Allow the psram_clk reset to propagate
        repeat (20) @(posedge clk50);

        // -------------------------------------------------------------------
        // 3. SPI: STATUS (opcode 0x00) – returns 16 response bytes
        // -------------------------------------------------------------------
        $display("[%0t ns] Sending STATUS command (0x00)", $time);
        spi_send_frame(64'h0000000000000000);
        // Give executor time to process
        repeat (200) @(posedge clk50);
        $display("[%0t ns] Reading STATUS response (16 bytes)", $time);
        spi_read_resp(16);

        // -------------------------------------------------------------------
        // 4. SPI: WRITE_REG (opcode 0x20) – addr=0, data=0x0001
        //    Frame: [0x20][0x00][0x01][0x00][0x00][0x00][0x00][0x00]
        // -------------------------------------------------------------------
        $display("[%0t ns] Sending WRITE_REG command (0x20)", $time);
        spi_send_frame(64'h2000_0100_0000_0000);
        repeat (100) @(posedge clk50);

        // -------------------------------------------------------------------
        // 5. SPI: READ_REG (opcode 0x21) – addr=0
        //    Frame: [0x21][0x00][0x00][0x00][0x00][0x00][0x00][0x00]
        // -------------------------------------------------------------------
        $display("[%0t ns] Sending READ_REG command (0x21)", $time);
        spi_send_frame(64'h2100_0000_0000_0000);
        repeat (100) @(posedge clk50);
        $display("[%0t ns] Reading READ_REG response (2 bytes)", $time);
        spi_read_resp(2);

        // -------------------------------------------------------------------
        // 6. SPI: POP_ADC_FRAME (opcode 0x42)
        //    Frame: [0x42][0x00][0x00][0x00][0x00][0x00][0x00][0x00]
        // -------------------------------------------------------------------
        $display("[%0t ns] Sending POP_ADC_FRAME command (0x42)", $time);
        spi_send_frame(64'h4200_0000_0000_0000);
        repeat (200) @(posedge clk50);
        $display("[%0t ns] Reading POP_ADC_FRAME response (16 bytes)", $time);
        spi_read_resp(16);

        // -------------------------------------------------------------------
        // 7. SPI: POP_IMU_FRAME (opcode 0x43)
        //    Frame: [0x43][0x00][0x00][0x00][0x00][0x00][0x00][0x00]
        // -------------------------------------------------------------------
        $display("[%0t ns] Sending POP_IMU_FRAME command (0x43)", $time);
        spi_send_frame(64'h4300_0000_0000_0000);
        repeat (200) @(posedge clk50);
        $display("[%0t ns] Reading POP_IMU_FRAME response (12 bytes)", $time);
        spi_read_resp(12);

        // -------------------------------------------------------------------
        // 8. Check HOST_IRQ (should be 0 since no frames in ring buffers)
        // -------------------------------------------------------------------
        repeat (50) @(posedge clk50);
        if (HOST_IRQ)
            $display("[%0t ns] HOST_IRQ is asserted", $time);
        else
            $display("[%0t ns] HOST_IRQ is de-asserted (expected)", $time);

        // -------------------------------------------------------------------
        // 9. Done
        // -------------------------------------------------------------------
        $display("[%0t ns] Simulation complete", $time);
        #1000;
        $finish;
    end

    // Safety watchdog: end simulation after 10 ms
    initial begin
        #10_000_000;
        $display("WATCHDOG: simulation timed out at 10 ms");
        $finish;
    end

endmodule
