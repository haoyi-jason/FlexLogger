# FlexLogger

High-speed ADC + IMU data logger on Gowin GW1NR-9 (QN88P).

---

## Overview

FlexLogger captures 8-channel ADC data (ADI AD7606) and averaged 6-axis IMU
data (4× Bosch BMI270) into a 16 MB HyperRAM (Winbond W955D8MKY-6I × 2),
and exposes a simple SPI command interface for an MCU host to read the data.

```
                           ┌──────────────────────────────────┐
  AD7606 (16-bit parallel)─►  ad7606_ctrl ──► adc_psram_dma  │
                           │                        │          │
  BMI270 × 4 (I2C 400kHz)─►  bmi270_driver         │          │
                           │    ↓ imu_avg ──► imu_psram_dma  │
                           │                        │          │
                           │              psram_arbiter       │
                           │                  │               │
                           │              psram_wrap          │
                           │                  │               │
                           │          PSRAM IP (Gowin)        │
                           │                  │               │
                           │         W955D8MKY-6I × 2         │
                           │                                  │
  MCU SPI (≤20 MHz) ───────►  spi_cmd_mailbox                │
                           │  psram_cmd_exec                  │
                           └──────────────────────────────────┘
```

---

## Feature Highlights

| Feature                    | Detail                                           |
|----------------------------|--------------------------------------------------|
| ADC                        | AD7606, 8 ch × 16-bit, up to 200 kSPS parallel  |
| IMU                        | 4× BMI270 on independent I2C buses at 400 kHz   |
| IMU averaging              | 4 sensors → 1 averaged 6-axis output at 400 Hz  |
| PSRAM                      | Winbond W955D8MKY-6I × 2 = 16 MB via Gowin IP   |
| Circular buffers           | 1000 frames each for ADC and IMU with overflow   |
| MCU interface              | SPI slave mode 0, ≤20 MHz, 8-byte command frames |
| Interrupt                  | `HOST_IRQ` configurable watermark per buffer     |

---

## Repository Structure

```
rtl/
  async_fifo_simple.v       Gray-code async FIFO (CDC)
  psram_wrap.v              Gowin PSRAM IP wrapper
  spi_slave_mode0_sclk.v    SPI slave (mode 0)
  spi_cmd_mailbox.v         SPI <-> psram_clk command bridge
  psram_addr_map.v          PSRAM layout parameters
  ringptr_1000.v            1000-entry circular buffer pointer
  psram_ringbuf_store.v     Ring buffer write engine
  psram_ringbuf_fetch.v     Ring buffer read engine
  regfile_core.v            8×16-bit register file
  psram_cmd_exec.v          SPI command executor (full command set)
  psram_arbiter.v           3-client PSRAM bus arbiter
  ad7606_ctrl.v             AD7606 capture state machine
  adc_psram_dma.v           ADC → PSRAM DMA (with CDC)
  i2c_master.v              Bit-bang I2C master (400 kHz)
  bmi270_driver.v           BMI270 init + data polling
  imu_avg.v                 4-sensor IMU averager
  imu_psram_dma.v           IMU → PSRAM DMA (with CDC)
  top.v                     Top-level module

docs/
  spi_command_set_full.md         SPI protocol and command reference
  psram_bringup.md                PSRAM verification procedure
  psram_clocking_recommendation.md PLL and timing constraints
  integration_notes.md            Architecture notes and TODO list
```

---

## Clock Architecture

| Domain       | Frequency | Description                           |
|--------------|-----------|---------------------------------------|
| `clk50`      | 50 MHz    | System clock; ADC and IMU drivers     |
| `psram_clk`  | ~83 MHz   | From PSRAM IP `clk_out`; PSRAM and DMA|
| `spi_sclk`   | ≤20 MHz   | MCU-provided SPI clock                |

All clock-domain crossings use `async_fifo_simple` (Gray-code pointers) or
3-flip-flop synchronizers. See `docs/psram_clocking_recommendation.md`.

---

## PSRAM Memory Map

| Word Address       | Content                          |
|--------------------|----------------------------------|
| 0x00000 – 0x007CF  | ADC circular buffer (1000 frames)|
| 0x007D0 – 0x00F9F  | IMU circular buffer (1000 frames)|

Each frame occupies 2 × 64-bit words (16 bytes).  An ADC frame holds 8 channels;
an IMU frame holds gx, gy, gz, ax, ay, az (12 bytes + 4 bytes padding).

---

## SPI Command Quick Reference

| Opcode | Command          | Response   | Description                        |
|--------|------------------|------------|------------------------------------|
| 0x00   | STATUS           | 16 bytes   | Pending counts, indices, overflow  |
| 0x20   | WRITE_REG        | none       | Write FPGA register                |
| 0x21   | READ_REG         | 2 bytes    | Read FPGA register                 |
| 0x40   | READ_ADC_FRAME   | 16 bytes   | ADC frame by index                 |
| 0x41   | READ_IMU_FRAME   | 12 bytes   | IMU frame by index                 |
| 0x42   | POP_ADC_FRAME    | 16 bytes   | Pop oldest ADC frame               |
| 0x43   | POP_IMU_FRAME    | 12 bytes   | Pop oldest IMU frame               |
| 0xF0   | PSRAM_WRITE_WORD | none       | Bring-up: raw PSRAM write          |
| 0xF1   | PSRAM_READ_WORD  | 8 bytes    | Bring-up: raw PSRAM read           |

Full protocol documentation: `docs/spi_command_set_full.md`

---

## Build Instructions (Gowin EDA)

1. Install **Gowin EDA** v1.9.9 or later.
2. Create a new project targeting **GW1NR-9 QN88P**.
3. Add all `rtl/*.v` files to the project.
4. Generate required IPs via **IP Core Generator**:
   - *PSRAM Memory Interface HS V2.0* – configure for W955D8MKY-6I, 2 chips, 166 MHz
   - *rPLL* – 50 MHz input → 166.667 MHz CLKOUT + 83.333 MHz CLKOUTD
5. Replace the simulation placeholders in `top.v` (~line 105) with real IP instances.
6. Add pin constraints (`.cst`) based on your PCB schematic.
7. Add timing constraints per `docs/psram_clocking_recommendation.md`.
8. Run **Synthesize → Place & Route → Generate Bitstream**.
9. Flash via JTAG.
10. Verify PSRAM operation per `docs/psram_bringup.md`.

---

## MCU Usage

### Typical Polling Loop

```c
void poll_flexlogger(void) {
    uint8_t status[16];
    spi_cmd(0x00, NULL, status, 16);   // STATUS
    uint16_t adc_pending = (status[0] << 8) | status[1];

    while (adc_pending > 0) {
        uint8_t frame[16];
        spi_cmd(0x42, NULL, frame, 16);  // POP_ADC_FRAME
        process_adc_frame(frame);
        adc_pending--;
    }
}
```

### Interrupt-Driven

1. Configure watermarks via `WRITE_REG` (address 0x00 / 0x01).
2. Enable IRQs via `WRITE_REG` address 0x02 (default: both enabled).
3. On `HOST_IRQ` assertion, issue `STATUS` then pop frames as above.

---

## Hardware Notes

- **AD7606**: CONVST_A and CONVST_B must be tied together on the PCB.
  OS[2:0] and RANGE pins are driven by the FPGA.
  BUSY output requires a direct FPGA input.
- **BMI270**: Each sensor on its own I2C bus (4 buses total).
  Address = 0x68 (SDO tied to GND). Pull-ups 4.7 kΩ per bus.
- **PSRAM**: 1.8 V device; add level shifters if FPGA I/O bank is 3.3 V.
  Differential PSRAM clock pair must be routed as a matched-length pair.
- **SPI**: Max 20 MHz. SCK idle low (mode 0). CS_N active low.

---

## License

See [LICENSE](LICENSE) for details.
