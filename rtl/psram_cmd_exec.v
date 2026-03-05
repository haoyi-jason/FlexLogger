// psram_cmd_exec.v
// SPI command executor – psram_clk domain.
//
// Receives 8-byte command words from spi_cmd_mailbox, executes them,
// and pushes response bytes back into spi_cmd_mailbox's response FIFO.
//
// Supported commands:
//   0x00  STATUS       → 16 bytes
//   0x20  WRITE_REG    payload: [addr(1B) data(2B)]
//   0x21  READ_REG     payload: [addr(1B)] → 2 bytes
//   0x40  READ_ADC_FRAME  payload: [idx(2B)] → 16 bytes (no ptr change)
//   0x41  READ_IMU_FRAME  payload: [idx(2B)] → 12 bytes (no ptr change)
//   0x42  POP_ADC_FRAME   → 16 bytes, advance rd ptr if pending>0
//   0x43  POP_IMU_FRAME   → 12 bytes, advance rd ptr if pending>0
//   0xF0  PSRAM_WRITE_WORD payload: [addr(3B) data(8B)] (bring-up)
//   0xF1  PSRAM_READ_WORD  payload: [addr(3B)] → 8 bytes (bring-up)
//
// CMD frame byte layout (8 bytes MSB-first from mailbox):
//   bits[63:56] opcode
//   bits[55:48] payload[0]
//   bits[47:40] payload[1]
//   bits[39:32] payload[2]
//   bits[31:24] payload[3]
//   bits[23:16] payload[4]
//   bits[15: 8] payload[5]
//   bits[ 7: 0] payload[6]

`timescale 1ns/1ps

module psram_cmd_exec (
    input  wire        clk,
    input  wire        rst_n,

    // From spi_cmd_mailbox
    input  wire [63:0] cmd_data,
    input  wire        cmd_valid,
    output reg         cmd_ready,

    // To spi_cmd_mailbox response FIFO
    output reg  [7:0]  resp_data,
    output reg         resp_push,
    input  wire        resp_full,

    // ADC ring-pointer status (read from ringptr_1000)
    input  wire [10:0] adc_pending,
    input  wire [9:0]  adc_rd_idx,
    input  wire [9:0]  adc_wr_idx,
    input  wire [15:0] adc_overflow,
    output reg         adc_pop,

    // IMU ring-pointer status
    input  wire [10:0] imu_pending,
    input  wire [9:0]  imu_rd_idx,
    input  wire [9:0]  imu_wr_idx,
    input  wire [15:0] imu_overflow,
    output reg         imu_pop,

    // Register file
    output reg  [2:0]  reg_rd_addr,
    input  wire [15:0] reg_rd_data,
    output reg  [2:0]  reg_wr_addr,
    output reg  [15:0] reg_wr_data,
    output reg         reg_wr_en,

    // PSRAM fetch interface (shared via arbiter, client 2)
    output reg  [9:0]  fetch_idx,
    output reg         fetch_req,
    output reg         fetch_is_imu,  // 0=ADC buffer, 1=IMU buffer
    input  wire        fetch_busy,
    input  wire [63:0] fetch_word0,
    input  wire [63:0] fetch_word1,
    input  wire        fetch_valid,

    // Direct PSRAM access for bring-up (raw arbiter client 2 bypass)
    output reg         raw_req,
    output reg         raw_cmd_wr,
    output reg  [20:0] raw_addr,
    output reg  [63:0] raw_wr_data,
    output reg  [7:0]  raw_mask,
    input  wire [63:0] raw_rd_data,
    input  wire        raw_rd_valid,
    input  wire        raw_busy,

    // init_calib from PSRAM IP
    input  wire        init_calib
);

    // ------------------------------------------------------------------
    // Opcodes
    // ------------------------------------------------------------------
    localparam OP_STATUS         = 8'h00;
    localparam OP_WRITE_REG      = 8'h20;
    localparam OP_READ_REG       = 8'h21;
    localparam OP_READ_ADC_FRAME = 8'h40;
    localparam OP_READ_IMU_FRAME = 8'h41;
    localparam OP_POP_ADC_FRAME  = 8'h42;
    localparam OP_POP_IMU_FRAME  = 8'h43;
    localparam OP_PSRAM_WRITE    = 8'hF0;
    localparam OP_PSRAM_READ     = 8'hF1;

    // ------------------------------------------------------------------
    // State machine
    // ------------------------------------------------------------------
    localparam ST_IDLE        = 5'd0;
    localparam ST_DECODE      = 5'd1;
    localparam ST_STATUS      = 5'd2;
    localparam ST_SEND_BYTES  = 5'd3;
    localparam ST_WRITE_REG   = 5'd4;
    localparam ST_READ_REG    = 5'd5;
    localparam ST_FETCH_REQ   = 5'd6;
    localparam ST_FETCH_WAIT  = 5'd7;
    localparam ST_SEND_FRAME  = 5'd8;
    localparam ST_RAW_WR      = 5'd9;
    localparam ST_RAW_WR_WAIT = 5'd10;
    localparam ST_RAW_RD      = 5'd11;
    localparam ST_RAW_RD_WAIT = 5'd12;
    localparam ST_RAW_SEND    = 5'd13;
    localparam ST_POP         = 5'd14;

    reg [4:0]  state;
    reg [7:0]  opcode;
    reg [55:0] payload;
    reg [7:0]  resp_buf [0:15];
    reg [4:0]  resp_cnt;   // total bytes to send
    reg [4:0]  resp_ptr;   // current byte being sent
    reg        is_imu_cmd; // distinguishes ADC vs IMU fetch/pop

    integer j;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            cmd_ready    <= 1'b0;
            resp_push    <= 1'b0;
            resp_data    <= 8'h00;
            adc_pop      <= 1'b0;
            imu_pop      <= 1'b0;
            reg_wr_en    <= 1'b0;
            fetch_req    <= 1'b0;
            raw_req      <= 1'b0;
            raw_cmd_wr   <= 1'b0;
            raw_addr     <= 21'd0;
            raw_wr_data  <= 64'd0;
            raw_mask     <= 8'h00;
            resp_cnt     <= 5'd0;
            resp_ptr     <= 5'd0;
            is_imu_cmd   <= 1'b0;
            for (j = 0; j < 16; j = j + 1) resp_buf[j] <= 8'h00;
        end else begin
            cmd_ready  <= 1'b0;
            resp_push  <= 1'b0;
            adc_pop    <= 1'b0;
            imu_pop    <= 1'b0;
            reg_wr_en  <= 1'b0;
            fetch_req  <= 1'b0;
            raw_req    <= 1'b0;

            case (state)

                // ---- Wait for command ----
                ST_IDLE: begin
                    if (cmd_valid) begin
                        opcode  <= cmd_data[63:56];
                        payload <= cmd_data[55:0];
                        cmd_ready <= 1'b1;
                        state   <= ST_DECODE;
                    end
                end

                // ---- Decode ----
                ST_DECODE: begin
                    case (opcode)
                        OP_STATUS: begin
                            // 16 bytes: adc_pending(2) adc_rd(2) adc_wr(2) adc_ovf(2)
                            //           imu_pending(2) imu_rd(2) imu_wr(2) imu_ovf(2)
                            //           + init_calib(1) pad(1)
                            resp_buf[0]  <= adc_pending[10:8] == 0 ?
                                              adc_pending[7:0] : 8'hFF; // clamp
                            resp_buf[1]  <= adc_pending[7:0];
                            resp_buf[2]  <= adc_rd_idx[9:8];
                            resp_buf[3]  <= adc_rd_idx[7:0];
                            resp_buf[4]  <= adc_wr_idx[9:8];
                            resp_buf[5]  <= adc_wr_idx[7:0];
                            resp_buf[6]  <= adc_overflow[15:8];
                            resp_buf[7]  <= adc_overflow[7:0];
                            resp_buf[8]  <= imu_pending[10:8] == 0 ?
                                              imu_pending[7:0] : 8'hFF;
                            resp_buf[9]  <= imu_pending[7:0];
                            resp_buf[10] <= imu_rd_idx[9:8];
                            resp_buf[11] <= imu_rd_idx[7:0];
                            resp_buf[12] <= imu_wr_idx[9:8];
                            resp_buf[13] <= imu_wr_idx[7:0];
                            resp_buf[14] <= imu_overflow[15:8];
                            resp_buf[15] <= imu_overflow[7:0];
                            resp_cnt     <= 5'd16;
                            resp_ptr     <= 5'd0;
                            state        <= ST_SEND_BYTES;
                        end

                        OP_WRITE_REG: begin
                            reg_wr_addr <= payload[55:48][2:0];
                            reg_wr_data <= {payload[47:40], payload[39:32]};
                            reg_wr_en   <= 1'b1;
                            state       <= ST_IDLE;
                        end

                        OP_READ_REG: begin
                            reg_rd_addr <= payload[55:48][2:0];
                            state       <= ST_READ_REG;
                        end

                        OP_READ_ADC_FRAME: begin
                            fetch_idx    <= {payload[55:48][1:0], payload[47:40]};
                            fetch_is_imu <= 1'b0;
                            is_imu_cmd   <= 1'b0;
                            fetch_req    <= 1'b1;
                            state        <= ST_FETCH_WAIT;
                        end

                        OP_READ_IMU_FRAME: begin
                            fetch_idx    <= {payload[55:48][1:0], payload[47:40]};
                            fetch_is_imu <= 1'b1;
                            is_imu_cmd   <= 1'b1;
                            fetch_req    <= 1'b1;
                            state        <= ST_FETCH_WAIT;
                        end

                        OP_POP_ADC_FRAME: begin
                            is_imu_cmd   <= 1'b0;
                            fetch_is_imu <= 1'b0;
                            state        <= ST_POP;
                        end

                        OP_POP_IMU_FRAME: begin
                            is_imu_cmd   <= 1'b1;
                            fetch_is_imu <= 1'b1;
                            state        <= ST_POP;
                        end

                        OP_PSRAM_WRITE: begin
                            raw_addr    <= {payload[55:48][4:0], payload[47:40],
                                            payload[39:32]};
                            raw_wr_data <= {payload[31:24], payload[23:16],
                                            payload[15: 8], payload[ 7: 0],
                                            32'd0};  // top 32 bits from cmd
                            raw_mask    <= 8'h00;
                            raw_cmd_wr  <= 1'b1;
                            state       <= ST_RAW_WR;
                        end

                        OP_PSRAM_READ: begin
                            raw_addr   <= {payload[55:48][4:0], payload[47:40],
                                           payload[39:32]};
                            raw_cmd_wr <= 1'b0;
                            state      <= ST_RAW_RD;
                        end

                        default: state <= ST_IDLE;
                    endcase
                end

                // ---- STATUS: stream resp_buf bytes ----
                ST_SEND_BYTES: begin
                    if (!resp_full) begin
                        resp_data <= resp_buf[resp_ptr];
                        resp_push <= 1'b1;
                        resp_ptr  <= resp_ptr + 1'b1;
                        if (resp_ptr == resp_cnt - 1'b1)
                            state <= ST_IDLE;
                    end
                end

                // ---- READ_REG: wait one cycle for combinational read ----
                ST_READ_REG: begin
                    if (!resp_full) begin
                        resp_buf[0] <= reg_rd_data[15:8];
                        resp_buf[1] <= reg_rd_data[7:0];
                        resp_cnt    <= 5'd2;
                        resp_ptr    <= 5'd0;
                        state       <= ST_SEND_BYTES;
                    end
                end

                // ---- FETCH: wait for fetch engine ----
                ST_FETCH_WAIT: begin
                    if (fetch_valid) begin
                        // Pack word0 and word1 into resp_buf (16 bytes)
                        resp_buf[0]  <= fetch_word0[63:56];
                        resp_buf[1]  <= fetch_word0[55:48];
                        resp_buf[2]  <= fetch_word0[47:40];
                        resp_buf[3]  <= fetch_word0[39:32];
                        resp_buf[4]  <= fetch_word0[31:24];
                        resp_buf[5]  <= fetch_word0[23:16];
                        resp_buf[6]  <= fetch_word0[15: 8];
                        resp_buf[7]  <= fetch_word0[ 7: 0];
                        resp_buf[8]  <= fetch_word1[63:56];
                        resp_buf[9]  <= fetch_word1[55:48];
                        resp_buf[10] <= fetch_word1[47:40];
                        resp_buf[11] <= fetch_word1[39:32];
                        resp_buf[12] <= fetch_word1[31:24];
                        resp_buf[13] <= fetch_word1[23:16];
                        resp_buf[14] <= fetch_word1[15: 8];
                        resp_buf[15] <= fetch_word1[ 7: 0];
                        // IMU frame is 12 bytes, ADC is 16
                        resp_cnt     <= is_imu_cmd ? 5'd12 : 5'd16;
                        resp_ptr     <= 5'd0;
                        state        <= ST_SEND_BYTES;
                    end
                end

                // ---- POP: fetch current rd_idx, then advance pointer ----
                ST_POP: begin
                    if (!is_imu_cmd) begin
                        if (adc_pending > 11'd0) begin
                            fetch_idx <= adc_rd_idx;
                            fetch_req <= 1'b1;
                            adc_pop   <= 1'b1;
                            state     <= ST_FETCH_WAIT;
                        end else begin
                            // No data: send 16 zero bytes
                            for (j = 0; j < 16; j = j + 1) resp_buf[j] <= 8'h00;
                            resp_cnt <= 5'd16;
                            resp_ptr <= 5'd0;
                            state    <= ST_SEND_BYTES;
                        end
                    end else begin
                        if (imu_pending > 11'd0) begin
                            fetch_idx <= imu_rd_idx;
                            fetch_req <= 1'b1;
                            imu_pop   <= 1'b1;
                            state     <= ST_FETCH_WAIT;
                        end else begin
                            for (j = 0; j < 12; j = j + 1) resp_buf[j] <= 8'h00;
                            resp_cnt <= 5'd12;
                            resp_ptr <= 5'd0;
                            state    <= ST_SEND_BYTES;
                        end
                    end
                end

                // ---- Raw PSRAM write (bring-up) ----
                ST_RAW_WR: begin
                    if (!raw_busy) begin
                        raw_req <= 1'b1;
                        state   <= ST_RAW_WR_WAIT;
                    end
                end

                ST_RAW_WR_WAIT: begin
                    if (!raw_busy && !raw_req) state <= ST_IDLE;
                end

                // ---- Raw PSRAM read (bring-up) ----
                ST_RAW_RD: begin
                    if (!raw_busy) begin
                        raw_req <= 1'b1;
                        state   <= ST_RAW_RD_WAIT;
                    end
                end

                ST_RAW_RD_WAIT: begin
                    if (raw_rd_valid) begin
                        resp_buf[0] <= raw_rd_data[63:56];
                        resp_buf[1] <= raw_rd_data[55:48];
                        resp_buf[2] <= raw_rd_data[47:40];
                        resp_buf[3] <= raw_rd_data[39:32];
                        resp_buf[4] <= raw_rd_data[31:24];
                        resp_buf[5] <= raw_rd_data[23:16];
                        resp_buf[6] <= raw_rd_data[15: 8];
                        resp_buf[7] <= raw_rd_data[ 7: 0];
                        resp_cnt    <= 5'd8;
                        resp_ptr    <= 5'd0;
                        state       <= ST_SEND_BYTES;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
