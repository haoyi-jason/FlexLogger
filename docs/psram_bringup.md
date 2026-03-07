# PSRAM Bring-Up Guide

This document describes how to verify that the Gowin PSRAM Memory Interface HS V2.0
IP and the `psram_wrap.v` module are working correctly before enabling the full
data path.

---

## 1. Hardware Prerequisites

| Signal           | Connection                                |
|------------------|-------------------------------------------|
| `IO_psram_dq`    | 16-bit data bus to W955D8MKY-6I (both chips share the bus) |
| `IO_psram_rwds`  | DQS per chip (2-bit)                      |
| `O_psram_ck/ck_n`| Differential clock output (one pair per chip) |
| `O_psram_cs_n`   | Chip select (one per chip)                |
| `O_psram_reset_n`| Active-low reset                          |
| Pull-ups         | 10 kΩ on SDA lines; PSRAM has internal termination |

---

## 2. `psram_wrap` Parameters to Adjust

The Gowin IP's `cmd` and `addr` semantics are not publicly documented.
`psram_wrap.v` has two adjustable parameters:

| Parameter      | Default | Description                                        |
|----------------|---------|----------------------------------------------------|
| `CMD_WRITE`    | `1`     | Value of `cmd` that triggers a **write** operation |
| `ADDR_IS_WORD` | `1`     | `1` = `addr` is in 64-bit word units; `0` = bytes  |

If bring-up fails, try toggling `CMD_WRITE` to `0` first, then try `ADDR_IS_WORD=0`.

---

## 3. Step-by-Step Bring-Up via SPI Commands

### Step 1 – Power-On and PLL Lock

1. Apply power; verify 3.3 V on PSRAM VCC.
2. Assert `rst_n_ext = 0` for ≥ 1 ms, then release.
3. Issue `STATUS` (0x00) command and check that `init_calib` status bit is set
   (currently `init_calib` is embedded in the PSRAM IP's `init_calib` output;
   observable via STATUS byte 0 being non-zero only after calibration).

> **Tip**: Monitor `HOST_IRQ` – it will not assert until the PSRAM is calibrated
> and data is written.

### Step 2 – Write a Known Pattern

Use SPI command **0xF0 PSRAM_WRITE_WORD**:

```
TX: F0  00  00  00  DE AD BE EF  (write 0xDEADBEEF00000000 to word addr 0x000000)
```

- Byte 1–3: word address = 0x000000
- Bytes 4–7: upper 32 bits of data = 0xDEADBEEF

### Step 3 – Read Back

Use SPI command **0xF1 PSRAM_READ_WORD**:

```
TX: F1  00  00  00  00 00 00 00
RX: (8 dummy bytes) → should return DE AD BE EF 00 00 00 00
```

If the readback matches, the PSRAM is operational.

### Step 4 – Test Multiple Addresses

Write to addresses 0, 1, 2, 0x7CF, 0x7D0, 0xF9F (boundary of each buffer).
Read back and verify.

### Step 5 – Stress Write/Read

Write 1000 sequential words with incrementing data, read them back and compare.

---

## 4. Troubleshooting

### All-zeros readback

- Check PLL lock signal.  If `pll_lock=0`, the IP will not function.
- Verify `init_calib=1` from the IP.  If not, check memory_clk frequency
  (should be ≥ 100 MHz; recommended 166 MHz).

### Readback returns wrong data

- Try `CMD_WRITE=0` (flip write/read command polarity).
- Check that the address is within range: maximum word address = 2^21 - 1 = 2,097,151.

### Readback has bit errors

- Check PSRAM DQ signal integrity with an oscilloscope.
- Reduce memory_clk frequency.
- Verify `memory_clk_p` phase offset is correct (90° or 180° per Gowin recommendations).

### `init_calib` never goes high

- Check `rst_n` polarity (active low).
- Ensure `pll_lock` is asserted before `rst_n` is de-asserted.
- Verify `clk_d` = `clk_out` from the IP is properly connected.

---

## 5. Expected PSRAM Layout After Bring-Up

```
Word Address   Content
0x00000        ADC frame 0, word 0  (ch0–ch3)
0x00001        ADC frame 0, word 1  (ch4–ch7)
0x00002        ADC frame 1, word 0
...
0x007CF        ADC frame 999, word 1
0x007D0        IMU frame 0, word 0  (gx,gy,gz,ax)
0x007D1        IMU frame 0, word 1  (ay,az,pad,pad)
...
0x00F9F        IMU frame 999, word 1
```

Total used: 4000 × 8 bytes = 32 KB out of 16 MB available.

---

## 6. Updating Parameters in Gowin EDA

To change `CMD_WRITE` or `ADDR_IS_WORD`:

1. Open `rtl/psram_wrap.v`.
2. Locate the `parameter` declarations near the top of the module.
3. Modify the defaults, or use `defparam` / parameter override in `top.v`:

```verilog
psram_wrap #(
    .CMD_WRITE    (0),   // flip if reads/writes appear swapped
    .ADDR_IS_WORD (0)    // flip if byte addressing is required
) u_psram ( ... );
```

4. Re-synthesize and re-test.
