# SPI Command Set – Full Reference

## Transport Layer

- **SPI Mode 0** (CPOL=0, CPHA=0), MSB first
- **Max clock**: 20 MHz
- **Frame size**: 8 bytes (MCU → FPGA), response is variable length streamed back
- **CS_N**: must be held low for the entire command frame; can be released between
  the command frame and response clocking, but continuous CS_N is simpler

### Command Frame Layout (8 bytes, MCU → FPGA)

| Byte | Field       | Description                              |
|------|-------------|------------------------------------------|
| 0    | opcode      | Command identifier (see table below)     |
| 1    | payload[0]  | Opcode-specific                          |
| 2    | payload[1]  | Opcode-specific                          |
| 3    | payload[2]  | Opcode-specific                          |
| 4    | payload[3]  | Opcode-specific                          |
| 5    | payload[4]  | Opcode-specific                          |
| 6    | payload[5]  | Opcode-specific                          |
| 7    | payload[6]  | Opcode-specific                          |

### Response Stream (FPGA → MCU)

After sending the command frame, the MCU clocks dummy bytes to receive the
response.  The FPGA pushes response bytes into an 8-entry async FIFO
(`resp_fifo`).  The SPI slave drives `MISO = 0x00` when the FIFO is empty.
The MCU should clock as many bytes as defined for each command (see table).

---

## Command Table

### 0x00 – STATUS

Returns 16 bytes of system status.

**Payload**: (none, bytes 1–7 ignored)

**Response** (16 bytes):

| Byte  | Content                          |
|-------|----------------------------------|
| 0–1   | `adc_pending` (uint16, big-endian) |
| 2–3   | `adc_rd_idx`  (uint16)           |
| 4–5   | `adc_wr_idx`  (uint16)           |
| 6–7   | `adc_overflow` (uint16)          |
| 8–9   | `imu_pending` (uint16)           |
| 10–11 | `imu_rd_idx`  (uint16)           |
| 12–13 | `imu_wr_idx`  (uint16)           |
| 14–15 | `imu_overflow` (uint16)          |

---

### 0x20 – WRITE_REG

Writes a 16-bit value to a register in the FPGA register file.

**Payload**:

| Byte | Field    | Description              |
|------|----------|--------------------------|
| 1    | reg_addr | Register address (0–7)   |
| 2    | data_hi  | Data[15:8]               |
| 3    | data_lo  | Data[7:0]                |
| 4–7  | (unused) |                          |

**Response**: (none)

#### Register Map

| Addr | Name              | Default  | Description                          |
|------|-------------------|----------|--------------------------------------|
| 0x00 | ADC_WATERMARK     | 1        | IRQ threshold for ADC pending count  |
| 0x01 | IMU_WATERMARK     | 1        | IRQ threshold for IMU pending count  |
| 0x02 | IRQ_MASK          | 0x0003   | bit[0]=ADC_EN, bit[1]=IMU_EN         |
| 0x03 | ADC_CFG           | 0x0000   | bits[2:0]=OS[2:0], bit[3]=RANGE      |
| 0x04 | IMU_RATE_DIV      | 0        | Future: IMU ODR divider              |
| 0x05 | (reserved)        | 0        |                                      |
| 0x06 | (reserved)        | 0        |                                      |
| 0x07 | (reserved)        | 0        |                                      |

---

### 0x21 – READ_REG

Reads a 16-bit register value.

**Payload**:

| Byte | Field    | Description              |
|------|----------|--------------------------|
| 1    | reg_addr | Register address (0–7)   |
| 2–7  | (unused) |                          |

**Response** (2 bytes): `[data_hi, data_lo]` big-endian

---

### 0x40 – READ_ADC_FRAME

Reads one ADC frame by logical index (does **not** advance the read pointer).

**Payload**:

| Byte | Field    | Description                            |
|------|----------|----------------------------------------|
| 1    | idx_hi   | Frame index [9:8] (upper 2 bits)       |
| 2    | idx_lo   | Frame index [7:0]                      |
| 3–7  | (unused) |                                        |

**Response** (16 bytes):

Each ADC frame is 8 channels × 16-bit = 128 bits, stored as two 64-bit PSRAM
words.  Byte order is big-endian within each channel:

| Byte  | Content     |
|-------|-------------|
| 0–1   | Channel 0   |
| 2–3   | Channel 1   |
| 4–5   | Channel 2   |
| 6–7   | Channel 3   |
| 8–9   | Channel 4   |
| 10–11 | Channel 5   |
| 12–13 | Channel 6   |
| 14–15 | Channel 7   |

---

### 0x41 – READ_IMU_FRAME

Reads one IMU frame by logical index (does **not** advance the read pointer).

**Payload**: same as `READ_ADC_FRAME`

**Response** (12 bytes):

| Byte  | Content     |
|-------|-------------|
| 0–1   | gx (int16)  |
| 2–3   | gy (int16)  |
| 4–5   | gz (int16)  |
| 6–7   | ax (int16)  |
| 8–9   | ay (int16)  |
| 10–11 | az (int16)  |

(Bytes 12–15 are the alignment padding stored in PSRAM but **not** returned.)

---

### 0x42 – POP_ADC_FRAME

Returns the oldest unread ADC frame and advances the read pointer.
If `pending == 0`, returns 16 zero bytes.

**Payload**: (none)

**Response** (16 bytes): same format as `READ_ADC_FRAME`

---

### 0x43 – POP_IMU_FRAME

Returns the oldest unread IMU frame and advances the read pointer.
If `pending == 0`, returns 12 zero bytes.

**Payload**: (none)

**Response** (12 bytes): same format as `READ_IMU_FRAME`

---

### 0xF0 – PSRAM_WRITE_WORD *(bring-up)*

Writes 8 bytes directly to a raw PSRAM word address.  Useful for testing the
PSRAM interface.

**Payload**:

| Byte | Field    | Description                        |
|------|----------|------------------------------------|
| 1    | addr[20:16] | Upper 5 bits of 21-bit word addr |
| 2    | addr[15:8]  | Mid byte                         |
| 3    | addr[7:0]   | Low byte                         |
| 4–7  | data[63:32] | Upper 32 bits of 64-bit word     |

> **Note**: The remaining 32 bits of the 64-bit write are zero.  To write a
> full 64-bit word, use two consecutive writes or extend the protocol.

**Response**: (none)

---

### 0xF1 – PSRAM_READ_WORD *(bring-up)*

Reads 8 bytes from a raw PSRAM word address.

**Payload**:

| Byte | Field       | Description                      |
|------|-------------|----------------------------------|
| 1    | addr[20:16] | Upper 5 bits of 21-bit word addr |
| 2    | addr[15:8]  | Mid byte                         |
| 3    | addr[7:0]   | Low byte                         |
| 4–7  | (unused)    |                                  |

**Response** (8 bytes): the 64-bit word, big-endian

---

## HOST_IRQ Behaviour

`HOST_IRQ` is asserted (active high) when:
- ADC IRQ is enabled (`IRQ_MASK[0]=1`) **and** `adc_pending >= ADC_WATERMARK`
- **or** IMU IRQ is enabled (`IRQ_MASK[1]=1`) **and** `imu_pending >= IMU_WATERMARK`

The MCU should respond to the interrupt by issuing `STATUS` to check which
buffer triggered the interrupt, then issue `POP_ADC_FRAME` / `POP_IMU_FRAME`
until `pending` drops below the watermark.

---

## Timing Notes

- The FPGA response FIFO holds up to 32 bytes.  For long responses (16 bytes),
  begin clocking MISO immediately after the 8th MOSI byte.
- The command executor typically responds within a few psram_clk cycles
  (~6–10 ns each) for register operations, and within ~200 ns for a PSRAM
  read fetch (2× PSRAM round-trips at ~100 ns each).
- The MCU may insert a small delay (e.g. 2–4 µs) between command and response
  clocking to ensure data is ready, or monitor MISO for non-zero bytes.
