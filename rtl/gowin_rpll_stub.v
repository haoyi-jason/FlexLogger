// gowin_rpll_stub.v
// Simulation stub for the Gowin-generated rPLL IP (Gowin_rPLL).
//
// In real synthesis the Gowin EDA tool provides the actual Gowin_rPLL
// primitive.  For iverilog simulation this stub is compiled instead.
// It mirrors the translate_off assignments in top.v so that both drivers
// resolve to the same value and there is no multi-driver conflict:
//   clkout  = clkin   (50 MHz in simulation, stands in for 166.667 MHz)
//   clkoutp = clkin   (same; phase shift not modelled in simulation)
//   lock    = 1'b1    (PLL immediately locked)
//
// Guard: only compiled when SYNTHESIS is NOT defined.

`ifndef SYNTHESIS

`timescale 1ns/1ps

module Gowin_rPLL (
    input  wire clkin,
    output wire clkout,
    output wire clkoutp,
    output wire lock
);
    // Forward clkin unchanged; lock is immediately asserted.
    assign clkout  = clkin;
    assign clkoutp = clkin;
    assign lock    = 1'b1;
endmodule

`endif // SYNTHESIS
