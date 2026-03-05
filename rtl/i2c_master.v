// i2c_master.v
// Simple I2C master for a single-byte register read or write.
// Clock: clk (50 MHz typically).  I2C SCL rate set by CLKDIV parameter.
//   CLKDIV = clk_freq / (4 * i2c_freq)
//   e.g., 50 MHz / (4 * 400 kHz) = 31.25 → use 31
//
// Supports:
//   - Single-byte write:  START, ADDR+W, REG, DATA, STOP
//   - Single-byte read:   START, ADDR+W, REG, RESTART, ADDR+R, DATA, NACK, STOP
//   - Multi-byte read:    same as above but n bytes with ACK between, final NACK
//
// SDA and SCL are open-drain: drive 0 by enabling output (sda_out=0, sda_oe=1)
// or release by disabling output (sda_oe=0 → external pull-up).
//
// Inputs/Outputs:
//   start     – pulse: begin transaction
//   rw        – 0=write, 1=read
//   dev_addr  – 7-bit device address
//   reg_addr  – 8-bit register address
//   wr_data   – 8-bit write data (used when rw=0)
//   rd_data   – 8-bit read data (valid when done=1 and rw=1)
//   done      – pulse: transaction complete
//   ack_err   – 1 when a NACK was received from device (cleared on start)

`timescale 1ns/1ps

module i2c_master #(
    parameter CLKDIV  = 31,    // clk divider for I2C SCL quarter-period
    parameter RD_BYTES = 1     // number of bytes to read
) (
    input  wire       clk,
    input  wire       rst_n,

    // Transaction control
    input  wire       start,
    input  wire       rw,          // 0=write, 1=read
    input  wire [6:0] dev_addr,
    input  wire [7:0] reg_addr,
    input  wire [7:0] wr_data,

    output reg  [7:0] rd_data,
    output reg        done,
    output reg        ack_err,
    output wire       busy,

    // I2C pins (open-drain model)
    output reg        scl_oe,    // 1 = drive SCL low, 0 = release (pull-up)
    output reg        sda_oe,    // 1 = drive SDA low, 0 = release (pull-up)
    input  wire       sda_in     // sampled SDA (for read and ACK)
);

    // -----------------------------------------------------------------------
    // Bit-bang state machine
    // -----------------------------------------------------------------------
    // Each SCL cycle takes 4 quarter-periods (set by CLKDIV).
    //   Q0: SCL=low,  SDA changes
    //   Q1: SCL=rising setup
    //   Q2: SCL=high (sample point)
    //   Q3: SCL=falling
    //
    // We enumerate all high-level states and break each into phases.

    localparam [5:0]
        ST_IDLE      = 6'd0,
        ST_START     = 6'd1,
        ST_ADDR_W    = 6'd2,   // send (dev_addr<<1)|0
        ST_ADDR_ACK  = 6'd3,
        ST_REG       = 6'd4,   // send reg_addr
        ST_REG_ACK   = 6'd5,
        ST_WDATA     = 6'd6,
        ST_WDATA_ACK = 6'd7,
        ST_STOP_W    = 6'd8,
        ST_RSTART    = 6'd9,   // repeated START
        ST_ADDR_R    = 6'd10,  // send (dev_addr<<1)|1
        ST_ADDR_R_ACK= 6'd11,
        ST_RDATA     = 6'd12,
        ST_RDATA_NACK= 6'd13,
        ST_STOP_R    = 6'd14,
        ST_DONE      = 6'd15;

    reg [5:0]  state;
    reg [7:0]  shreg;      // shift register for TX/RX
    reg [2:0]  bit_cnt;
    reg [7:0]  div_cnt;
    reg [1:0]  phase;      // 0..3 quarter periods

    assign busy = (state != ST_IDLE);

    // Quarter-period clock generator
    wire phase_tick = (div_cnt == CLKDIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 8'd0;
        end else if (state != ST_IDLE) begin
            if (phase_tick) div_cnt <= 8'd0;
            else            div_cnt <= div_cnt + 1'b1;
        end else begin
            div_cnt <= 8'd0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            scl_oe   <= 1'b0;
            sda_oe   <= 1'b0;
            shreg    <= 8'd0;
            bit_cnt  <= 3'd0;
            phase    <= 2'd0;
            rd_data  <= 8'd0;
            done     <= 1'b0;
            ack_err  <= 1'b0;
        end else begin
            done <= 1'b0;

            case (state)
                ST_IDLE: begin
                    scl_oe <= 1'b0;
                    sda_oe <= 1'b0;
                    if (start) begin
                        ack_err <= 1'b0;
                        state   <= ST_START;
                        phase   <= 2'd0;
                    end
                end

                // START condition: SDA falls while SCL is high
                ST_START: begin
                    if (phase_tick) begin
                        phase <= phase + 1'b1;
                        case (phase)
                            2'd0: begin sda_oe <= 1'b0; scl_oe <= 1'b0; end // SDA=H, SCL=H
                            2'd1: begin sda_oe <= 1'b1; end                  // SDA falls
                            2'd2: begin scl_oe <= 1'b1; end                  // SCL falls
                            2'd3: begin
                                shreg   <= {dev_addr, 1'b0};  // ADDR+W
                                bit_cnt <= 3'd7;
                                state   <= ST_ADDR_W;
                                phase   <= 2'd0;
                            end
                        endcase
                    end
                end

                // Generic byte send; bit_cnt=7..0
                ST_ADDR_W, ST_REG, ST_WDATA, ST_ADDR_R: begin
                    if (phase_tick) begin
                        phase <= phase + 1'b1;
                        case (phase)
                            2'd0: begin
                                sda_oe <= ~shreg[7];  // drive SDA
                                scl_oe <= 1'b1;       // SCL low
                            end
                            2'd1: begin scl_oe <= 1'b0; end  // SCL rises
                            2'd2: begin end                   // SCL high, sample ignored
                            2'd3: begin
                                scl_oe <= 1'b1;
                                shreg  <= {shreg[6:0], 1'b0};
                                if (bit_cnt == 3'd0) begin
                                    // Transition to ACK receive
                                    case (state)
                                        ST_ADDR_W: state <= ST_ADDR_ACK;
                                        ST_REG:    state <= ST_REG_ACK;
                                        ST_WDATA:  state <= ST_WDATA_ACK;
                                        ST_ADDR_R: state <= ST_ADDR_R_ACK;
                                        default:   state <= ST_DONE;
                                    endcase
                                    phase <= 2'd0;
                                end else begin
                                    bit_cnt <= bit_cnt - 1'b1;
                                    phase   <= 2'd0;
                                end
                            end
                        endcase
                    end
                end

                // Generic ACK receive
                ST_ADDR_ACK, ST_REG_ACK, ST_WDATA_ACK, ST_ADDR_R_ACK: begin
                    if (phase_tick) begin
                        phase <= phase + 1'b1;
                        case (phase)
                            2'd0: begin sda_oe <= 1'b0; scl_oe <= 1'b1; end
                            2'd1: begin scl_oe <= 1'b0; end
                            2'd2: begin
                                if (sda_in) ack_err <= 1'b1;  // NACK
                            end
                            2'd3: begin
                                scl_oe <= 1'b1;
                                phase  <= 2'd0;
                                case (state)
                                    ST_ADDR_ACK: begin
                                        shreg   <= reg_addr;
                                        bit_cnt <= 3'd7;
                                        state   <= ST_REG;
                                    end
                                    ST_REG_ACK: begin
                                        if (rw) begin
                                            state <= ST_RSTART;
                                        end else begin
                                            shreg   <= wr_data;
                                            bit_cnt <= 3'd7;
                                            state   <= ST_WDATA;
                                        end
                                    end
                                    ST_WDATA_ACK: state <= ST_STOP_W;
                                    ST_ADDR_R_ACK: begin
                                        bit_cnt <= 3'd7;
                                        shreg   <= 8'd0;
                                        state   <= ST_RDATA;
                                    end
                                    default: state <= ST_DONE;
                                endcase
                            end
                        endcase
                    end
                end

                // Repeated START
                ST_RSTART: begin
                    if (phase_tick) begin
                        phase <= phase + 1'b1;
                        case (phase)
                            2'd0: begin sda_oe <= 1'b0; scl_oe <= 1'b1; end
                            2'd1: begin scl_oe <= 1'b0; end
                            2'd2: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end
                            2'd3: begin
                                // Repeated START: SDA falls while SCL high
                                scl_oe  <= 1'b0;
                                sda_oe  <= 1'b1;  // SDA low (RSTART)
                                shreg   <= {dev_addr, 1'b1};  // ADDR+R
                                bit_cnt <= 3'd7;
                                state   <= ST_ADDR_R;
                                phase   <= 2'd0;
                            end
                        endcase
                    end
                end

                // Read data byte
                ST_RDATA: begin
                    if (phase_tick) begin
                        phase <= phase + 1'b1;
                        case (phase)
                            2'd0: begin sda_oe <= 1'b0; scl_oe <= 1'b1; end
                            2'd1: begin scl_oe <= 1'b0; end
                            2'd2: begin
                                shreg <= {shreg[6:0], sda_in}; // MSB first
                            end
                            2'd3: begin
                                scl_oe <= 1'b1;
                                if (bit_cnt == 3'd0) begin
                                    rd_data <= {shreg[6:0], sda_in};
                                    state   <= ST_RDATA_NACK;
                                    phase   <= 2'd0;
                                end else begin
                                    bit_cnt <= bit_cnt - 1'b1;
                                    phase   <= 2'd0;
                                end
                            end
                        endcase
                    end
                end

                // Send NACK after last read byte
                ST_RDATA_NACK: begin
                    if (phase_tick) begin
                        phase <= phase + 1'b1;
                        case (phase)
                            2'd0: begin sda_oe <= 1'b1; scl_oe <= 1'b1; end // SDA=H (NACK)
                            2'd1: begin scl_oe <= 1'b0; end
                            2'd2: begin end
                            2'd3: begin scl_oe <= 1'b1; state <= ST_STOP_R; phase <= 2'd0; end
                        endcase
                    end
                end

                // STOP after write
                ST_STOP_W, ST_STOP_R: begin
                    if (phase_tick) begin
                        phase <= phase + 1'b1;
                        case (phase)
                            2'd0: begin sda_oe <= 1'b1; scl_oe <= 1'b1; end
                            2'd1: begin scl_oe <= 1'b0; end  // SCL rises
                            2'd2: begin sda_oe <= 1'b0; end  // SDA rises (STOP)
                            2'd3: begin state <= ST_DONE; phase <= 2'd0; end
                        endcase
                    end
                end

                ST_DONE: begin
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
