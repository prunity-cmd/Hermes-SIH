## ============================================================================
## EDGE Artix-7 constraints for edge_top (v3: R2R DAC + XADC pots)
## Pins from the allaboutfpga master XDC.
## ============================================================================

# Clock: 100 MHz (matches CLK_HZ = 100_000_000)
set_property -dict { PACKAGE_PIN N11 IOSTANDARD LVCMOS33 } [get_ports { clk }];
create_clock -period 10.000 -name sys_clk -waveform {0.000 5.000} [get_ports { clk }];

# Reset: centre push-button, active-high
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 PULLDOWN true } [get_ports { btn_rst }];

# LEDs: {tx_active, active_mode[2:0]}
set_property -dict { PACKAGE_PIN J3 IOSTANDARD LVCMOS33 } [get_ports { led[0] }];
set_property -dict { PACKAGE_PIN H3 IOSTANDARD LVCMOS33 } [get_ports { led[1] }];
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports { led[2] }];
set_property -dict { PACKAGE_PIN K1 IOSTANDARD LVCMOS33 } [get_ports { led[3] }];

# ---- R2R ladder on the 20-pin expansion connector, pins 5..12 ----
# (connector pin 3 = 3V3, pin 4 = GND)
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[0] }];  # pin 5  LSB
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[1] }];  # pin 6
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[2] }];  # pin 7
set_property -dict { PACKAGE_PIN P16 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[3] }];  # pin 8
set_property -dict { PACKAGE_PIN R15 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[4] }];  # pin 9
set_property -dict { PACKAGE_PIN R16 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[5] }];  # pin 10
set_property -dict { PACKAGE_PIN T14 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[6] }];  # pin 11
set_property -dict { PACKAGE_PIN T15 IOSTANDARD LVCMOS33 DRIVE 12 SLEW FAST } [get_ports { r2r[7] }];  # pin 12 MSB

# ---- Analog power enable + scope trigger: expansion pins 13, 14 ----
set_property -dict { PACKAGE_PIN N13 IOSTANDARD LVCMOS33 } [get_ports { amp_en }];      # pin 13
set_property -dict { PACKAGE_PIN P13 IOSTANDARD LVCMOS33 } [get_ports { scope_trig }];  # pin 14

# ---- Pots -> XADC aux inputs on J13 (0..1 V only!) ----
# Channel numbers must match edge_top: AUX_DEPTH=6, AUX_TURB=14, AUX_TEMP=7, AUX_SAL=15
set_property -dict { PACKAGE_PIN C3 IOSTANDARD LVCMOS33 } [get_ports { vaux_p[0] }];  # depth       AD6P
set_property -dict { PACKAGE_PIN C2 IOSTANDARD LVCMOS33 } [get_ports { vaux_n[0] }];  #             AD6N
set_property -dict { PACKAGE_PIN B2 IOSTANDARD LVCMOS33 } [get_ports { vaux_p[1] }];  # turbidity   AD14P
set_property -dict { PACKAGE_PIN A2 IOSTANDARD LVCMOS33 } [get_ports { vaux_n[1] }];  #             AD14N
set_property -dict { PACKAGE_PIN C1 IOSTANDARD LVCMOS33 } [get_ports { vaux_p[2] }];  # temperature AD7P
set_property -dict { PACKAGE_PIN B1 IOSTANDARD LVCMOS33 } [get_ports { vaux_n[2] }];  #             AD7N
set_property -dict { PACKAGE_PIN E2 IOSTANDARD LVCMOS33 } [get_ports { vaux_p[3] }];  # salinity    AD15P
set_property -dict { PACKAGE_PIN D1 IOSTANDARD LVCMOS33 } [get_ports { vaux_n[3] }];  #             AD15N

# Outputs are IOB-registered; no timing constraints needed on the pins.
set_false_path -to   [get_ports { r2r[*] amp_en scope_trig led[*] }];
set_false_path -from [get_ports { btn_rst }];
