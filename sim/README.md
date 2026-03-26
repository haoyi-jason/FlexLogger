# FlexLogger Simulation

This directory contains the top-level simulation testbench for FlexLogger.

## Prerequisites

* [Icarus Verilog](https://github.com/steveicarus/iverilog) (`iverilog` / `vvp`)
* Optional: [GTKWave](https://gtkwave.sourceforge.net/) for waveform viewing

## Running the Simulation

Run the following commands from the **repository root**:

```bash
iverilog -g2005 \
  -DSIMULATION \
  -o sim/sim_top \
  rtl/async_fifo_simple.v \
  rtl/psram_addr_map.v \
  rtl/ringptr_1000.v \
  rtl/regfile_core.v \
  rtl/psram_wrap.v \
  rtl/psram_arbiter.v \
  rtl/psram_ringbuf_store.v \
  rtl/psram_ringbuf_fetch.v \
  rtl/psram_cmd_exec.v \
  rtl/ad7606_ctrl.v \
  rtl/adc_psram_dma.v \
  rtl/i2c_master.v \
  rtl/bmi270_driver.v \
  rtl/imu_avg.v \
  rtl/imu_psram_dma.v \
  rtl/spi_slave_mode0_sclk.v \
  rtl/spi_cmd_mailbox.v \
  rtl/gowin_rpll_stub.v \
  rtl/gowin_psram_stub.v \
  rtl/top.v \
  sim/tb_top.v

vvp sim/sim_top
```

This produces a `tb_top.vcd` waveform dump in the working directory.

## Viewing Waveforms

```bash
gtkwave tb_top.vcd
```

## What the Testbench Does

1. Applies a 10-cycle reset, then releases `rst_n_ext`.
2. Waits for `init_calib` from the PSRAM stub (asserted after ~200 `psram_clk` cycles).
3. Sends five SPI command frames and reads back the responses:

| Step | Opcode | Command | Response |
|------|--------|---------|----------|
| 1 | `0x00` | `STATUS` | 16 bytes |
| 2 | `0x20` | `WRITE_REG` addr=0 data=0x0001 | — |
| 3 | `0x21` | `READ_REG` addr=0 | 2 bytes |
| 4 | `0x42` | `POP_ADC_FRAME` | 16 bytes |
| 5 | `0x43` | `POP_IMU_FRAME` | 12 bytes |

4. Checks `HOST_IRQ` level.
5. Ends simulation.

A 10 ms watchdog terminates the simulation automatically if it hangs.

## Gowin EDA Synthesis Notes

When targeting Gowin EDA for synthesis, **do not** include the stub files:

* `rtl/gowin_rpll_stub.v` – replaced by the IP generated via `Tools → IP Core Generator → rPLL`
* `rtl/gowin_psram_stub.v` – replaced by the IP generated via `Tools → IP Core Generator → PSRAM Memory Interface HS V2`

The `// synthesis translate_off / translate_on` blocks in `rtl/top.v` are
automatically ignored by the Gowin synthesizer, so the real `Gowin_rPLL`
instantiation is used for synthesis while the simulation assigns remain
available for Gowin's own functional simulator.
