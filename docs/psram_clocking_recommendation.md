# PSRAM Clocking Recommendations

## Overview

The Gowin PSRAM Memory Interface HS V2.0 IP requires two clocks:

| Signal          | Source         | Typical Frequency | Notes                          |
|-----------------|----------------|-------------------|--------------------------------|
| `memory_clk`    | rPLL CLKOUT    | 166.667 MHz       | Core I/O clock for PSRAM       |
| `memory_clk_p`  | rPLL CLKOUTP   | 166.667 MHz       | Phase-shifted version (90°)    |
| `clk_d` / `clk_out` | rPLL or IP | 83.333 MHz       | User-domain clock (clk_out from IP) |

The W955D8MKY-6I (HyperRAM) is rated at up to 200 MHz (6 ns cycle).
166 MHz provides margin for layout.

---

## Gowin rPLL Configuration

Use the Gowin EDA IP core generator to instantiate an rPLL.
Recommended settings for a 50 MHz input:

```
Input frequency:   50 MHz
Output (CLKOUT):   166.667 MHz   (FBDIV=9, IDIV=2, ODIV=1 — verify in IP GUI)
Output (CLKOUTP):  166.667 MHz   90° phase shift
Output (CLKOUTD):  83.333 MHz    divided-by-2 for user domain
LOCK:              pll_lock output
```

### rPLL Instantiation Template

```verilog
// Replace with actual Gowin rPLL primitive name for GW1NR-9
rPLL u_pll (
    .CLKIN    (clk50),
    .CLKOUT   (memory_clk),     // 166.667 MHz
    .CLKOUTP  (memory_clk_p),   // 166.667 MHz, 90° phase
    .CLKOUTD  (clk_user),       // 83.333 MHz for clk_d
    .LOCK     (pll_lock),
    .RESET    (1'b0),
    .RESET_P  (1'b0),
    .FBDSEL   (6'd0),
    .IDSEL    (6'd0),
    .ODSEL    (6'd0),
    .DUTYDA   (8'd0),
    .PSDA     (4'd0),
    .FDLY     (4'd0)
);
```

> Exact primitive ports vary by Gowin device family.  Use the IP generator
> for the GW1NR-9 to get the correct template.

---

## Clock Domain Summary

| Domain       | Clock Source          | Frequency   | Modules                           |
|--------------|-----------------------|-------------|-----------------------------------|
| `clk50`      | External / OSC        | 50 MHz      | ad7606_ctrl, bmi270_driver, imu_avg, rst sync |
| `psram_clk`  | IP `clk_out`          | ~83 MHz     | psram_wrap, arbiter, DMAs, ringptr, cmd_exec, regfile |
| `spi_sclk`   | MCU-provided SCLK     | ≤ 20 MHz    | spi_slave (shift register), mailbox FIFO write side |

---

## CDC (Clock Domain Crossing) Strategy

| Crossing                   | Method                                      |
|----------------------------|---------------------------------------------|
| clk50 → psram_clk (data)   | `async_fifo_simple` (Gray-code pointers)    |
| psram_clk → spi_sclk (resp)| `async_fifo_simple`                         |
| spi_sclk → psram_clk (cmd) | `async_fifo_simple`                         |
| psram_clk → clk50 (IRQ)    | 3-FF synchronizer                           |
| Reset synchronization      | 3-FF synchronizer per domain                |

---

## Timing Constraints Guidance

Add the following to your Gowin `.sdc` / Timing Constraints file:

```tcl
# Primary clocks
create_clock -name clk50      -period 20.000  [get_ports clk50]
create_clock -name spi_sclk   -period 50.000  [get_ports spi_sclk]
# Derived clocks (if not auto-detected by EDA)
create_generated_clock -name memory_clk  -source [get_ports clk50] -multiply_by 10 -divide_by 3 [get_nets memory_clk]
create_generated_clock -name psram_clk   -source [get_ports clk50] -multiply_by 5  -divide_by 3 [get_nets psram_clk]

# Async FIFO crossings — set as asynchronous
set_clock_groups -asynchronous \
    -group [get_clocks clk50] \
    -group [get_clocks psram_clk] \
    -group [get_clocks spi_sclk]
```

---

## PSRAM IP Bring-Up Checklist

- [ ] Confirm `pll_lock = 1` before releasing PSRAM reset
- [ ] Confirm `init_calib = 1` within 500 µs of reset release
- [ ] Verify `clk_out` from IP is connected to `clk_d` **and** to design registers
- [ ] Add `memory_clk` and `memory_clk_p` to clock constraints
- [ ] Run timing analysis: no setup violations on `psram_clk` paths

---

## Notes on W955D8MKY-6I

- **Density**: 64 Mb per device × 2 devices = 128 Mb = 16 MB total
- **Interface**: HyperBus (differential clock, 8-bit DQ + RWDS)
- **Access latency**: 6CK initial latency at 166 MHz ≈ 36 ns
- **Burst length**: Configurable; Gowin IP handles wrapping bursts
- **Operating voltage**: 1.8 V (check level shifters if FPGA I/O is 3.3 V)
