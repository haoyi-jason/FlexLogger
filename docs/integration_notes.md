# Integration Notes and TODO List

## Architecture Summary

```
clk50 domain
  ├── ad7606_ctrl    → 128-bit ADC frames (8ch × 16-bit)
  ├── bmi270_driver × 4  → 96-bit IMU frames (6-axis × 16-bit), one per 2.5ms
  └── imu_avg        → averaged 96-bit IMU frame at 400 Hz

psram_clk domain (from PSRAM IP clk_out, ~83 MHz)
  ├── adc_psram_dma  (CDC FIFO from clk50)  → arbiter client 0
  ├── imu_psram_dma  (CDC FIFO from clk50)  → arbiter client 1
  ├── psram_arbiter  → psram_wrap → Gowin PSRAM IP
  ├── ringptr_1000 × 2 (ADC + IMU)
  ├── psram_ringbuf_store  (inside each DMA)
  ├── psram_ringbuf_fetch  (client 2, driven by cmd_exec)
  ├── regfile_core
  └── psram_cmd_exec ← spi_cmd_mailbox ← SPI slave (spi_sclk domain)

spi_sclk domain
  └── spi_slave_mode0_sclk  (inside spi_cmd_mailbox)
```

---

## Known Limitations / TODOs

### High Priority

1. **Gowin rPLL instantiation** (`top.v` line ~105)
   - Replace the simulation-only `assign memory_clk = clk50` placeholder
   - Instantiate the actual `rPLL` primitive from Gowin IP generator
   - Set: CLKOUT=166.667 MHz, CLKOUTP=90°, CLKOUTD=83.333 MHz (for psram_clk)

2. **`psram_wrap.v` – clk_out connection**
   - The Gowin IP outputs `clk_out` which must be routed as a global clock net
   - In `top.v`, `psram_clk` is currently assigned from a wire; ensure this
     is implemented as a clock buffer (BUFG) in the constraints

3. **PSRAM bring-up parameter tuning**
   - Verify `CMD_WRITE` and `ADDR_IS_WORD` parameters match Gowin IP behavior
   - See `docs/psram_bringup.md` for test procedure

4. **BMI270 init sequence** (`bmi270_driver.v`)
   - The current init writes 4 registers to enable basic 100 Hz mode
   - For production, implement the full BMI270 initialization sequence:
     - Load built-in configuration file (MCU or FPGA must upload ~8 KB init blob)
     - Enable FIFO in header mode for batch reads
   - FIFO mode significantly reduces I2C bus time per ODR period

5. **AD7606 config wiring**
   - `ad7606_ctrl.v` has `adc_os`, `adc_range`, `adc_stby_n` as output regs
   - These are currently initialized to fixed defaults (OS=0, RANGE=0)
   - Wire them to `regfile_core.reg_adc_cfg` in `top.v` for MCU-controllable config

### Medium Priority

6. **psram_cmd_exec – raw PSRAM ports**
   - `raw_req`, `raw_cmd_wr`, etc. are currently left unconnected in `top.v`
   - Wire these to a dedicated arbiter client (or time-multiplex with fetch client 2)
   - Required for 0xF0/0xF1 bring-up commands to function in hardware

7. **Fetch engine BASE_ADDR limitation**
   - `psram_ringbuf_fetch` uses a fixed `BASE_ADDR` parameter
   - In `top.v`, we apply an offset via `c2_addr_offset = c2_addr + fetch_base`
   - This works because the fetch module uses `BASE_ADDR=0`; but if BASE_ADDR
     is non-zero, the offset arithmetic would be wrong
   - Consider adding a runtime `base_addr` input port to `psram_ringbuf_fetch`

8. **IRQ watermark comparison**
   - `adc_pending >= reg_adc_watermark[10:0]` — verify bit widths match
   - `reg_adc_watermark` is 16-bit; `adc_pending` is 11-bit (0..1000)
   - Currently safe as long as watermark ≤ 1000

9. **SPI response latency**
   - The MCU must account for PSRAM read latency (~200 ns) when fetching frames
   - Recommend: insert 4–8 µs delay between command TX and response RX
   - Or: send dummy bytes until MISO returns non-zero as a "ready" indicator

### Low Priority

10. **Constraint file**
    - Create `impl/flexlogger.sdc` with timing constraints per
      `docs/psram_clocking_recommendation.md`

11. **Simulation testbench**
    - No testbench exists yet; a basic `tb_top.v` would help validate CDC FIFOs
    - At minimum, test `ringptr_1000` push/pop/overflow in isolation

12. **BMI270 I²C address**
    - All four BMI270s are configured at address 0x68 (SDO=GND)
    - Each has its own dedicated I2C bus, so address collision is not an issue
    - If future hardware puts multiple BMIs on the same bus, change address pin wiring

13. **ADC STBY_N wiring**
    - Currently `adc_stby_n = 1` (not in standby) from reset
    - Could be controlled via a register bit if power saving is needed

---

## File Inventory

### RTL (`rtl/`)

| File                       | Description                                    | Status       |
|----------------------------|------------------------------------------------|--------------|
| `async_fifo_simple.v`      | Gray-code async FIFO                           | Complete     |
| `psram_wrap.v`             | Gowin PSRAM IP wrapper                         | Complete     |
| `spi_slave_mode0_sclk.v`   | SPI slave, mode 0                              | Complete     |
| `spi_cmd_mailbox.v`        | SPI ↔ psram_clk command bridge                 | Complete     |
| `psram_addr_map.v`         | Address map parameters                         | Complete     |
| `ringptr_1000.v`           | 1000-entry circular buffer pointer             | Complete     |
| `psram_ringbuf_store.v`    | Ring buffer write engine                       | Complete     |
| `psram_ringbuf_fetch.v`    | Ring buffer read engine                        | Complete     |
| `regfile_core.v`           | 8×16-bit register file                         | Complete     |
| `psram_cmd_exec.v`         | SPI command executor                           | Complete     |
| `psram_arbiter.v`          | 3-client PSRAM arbiter                         | Complete     |
| `ad7606_ctrl.v`            | AD7606 ADC controller                          | Complete     |
| `adc_psram_dma.v`          | ADC → PSRAM DMA                                | Complete     |
| `i2c_master.v`             | I2C master (400 kHz)                           | Complete     |
| `bmi270_driver.v`          | BMI270 init + poll (stub init)                 | Partial      |
| `imu_avg.v`                | 4-IMU averager                                 | Complete     |
| `imu_psram_dma.v`          | IMU → PSRAM DMA                                | Complete     |
| `top.v`                    | Top-level integration                          | Complete*    |

\* Requires rPLL instantiation and bring-up parameter verification.

### Documentation (`docs/`)

| File                              | Description                            |
|-----------------------------------|----------------------------------------|
| `spi_command_set_full.md`         | Full SPI command protocol reference    |
| `psram_bringup.md`                | PSRAM verification procedure           |
| `psram_clocking_recommendation.md`| PLL config and timing constraints      |
| `integration_notes.md`           | This file                              |

---

## Build Instructions (Gowin EDA)

1. Open Gowin EDA (v1.9.9 or later recommended for GW1NR-9)
2. Create a new project, select device: **GW1NR-9 QN88P**
3. Add all files from `rtl/` to the project
4. Generate required IPs via IP Core Generator:
   - **PSRAM Memory Interface HS V2.0** → configure for W955D8MKY-6I, 2 chips
   - **rPLL** → 50 MHz in, 166.667 MHz CLKOUT, 83.333 MHz CLKOUTD
5. Replace placeholder code in `top.v` with generated IP instances
6. Add pin constraints (`.cst` file) per hardware schematic
7. Add timing constraints per `docs/psram_clocking_recommendation.md`
8. Run synthesis → P&R → bitstream generation
9. Flash via JTAG and perform bring-up per `docs/psram_bringup.md`
