# Adaptive Software-Defined Sonar Transmitter (FPGA)

**Smart India Hackathon 2026 · Problem Statement SIH26058 · Team Hermes**

A low-power, real-time adaptive sonar transmitter payload for Autonomous Underwater Vehicles (AUVs), built entirely in Verilog on an Artix-7 FPGA with a custom R-2R DAC and analog front end.

> Demo video: `<YouTube link>`  ·  Project report: `<Word document link>`

---

## 1. What Our Project Does

This system reshapes every sonar ping in real time based on the water the AUV is in. Four potentiometers stand in for environmental sensors (depth, turbidity, temperature and salinity). The FPGA reads them through its built-in XADC and uses ocean-acoustics models to pick the best centre frequency, bandwidth, pulse length and transmit power. It then synthesises the pulse (LFM chirp, geometric sweep or Barker-13 phase code) and streams it at 10 MS/s to an 8-bit R-2R DAC and an NE5532 analog filter.

There is no processor in the signal path. Every sample is produced by dedicated hardware, so nothing can stall and the output is deterministic.

---

## 2. Problem Statement & Solution

### Problem
Traditional sonar sends fixed-frequency pulses. The underwater environment changes constantly:
- A high-frequency ping (≈500 kHz) gives sharp images but is absorbed quickly in long or muddy paths.
- A low-frequency ping (≈100 kHz) travels far but gives blurry images.
- A fixed maximum-energy ping wastes the AUV's limited battery in easy conditions.

The problem statement asks for a physical transmitter that adapts its analog waveform in real time, like a software-defined radio for sound, while staying low-power.

### Solution
- **Physics-driven adaptation:** Francois–Garrison absorption (salinity-aware) and Medwin sound speed are stored in on-chip ROMs. Every dial position produces a new waveform, not one of a few presets.
- **Three modulations, switched on the fly:** LFM chirp, true exponential geometric sweep, and Barker-13 phase coding with phase flips placed on carrier zero crossings.
- **Digital windowing:** full-length Hann, Hamming, Blackman and Tukey windows to remove edge transients and sidelobes.
- **Hardware timer + streaming DDS**, the FPGA equivalent of the problem statement's "hardware timers and DMA".
- **Low-power sequencing:** the DDS runs only while transmitting, and `amp_en` powers the analog stage only around each pulse.

---

## 3. Features

- 100–500 kHz band (fc 112–450 kHz, bandwidth = 20 % of fc)
- Pulse length 1–8 ms, amplitude 50–100 %, range 10–150 m
- Listen window = 2R / c, with c corrected for temperature and salinity
- Automatic modulation choice:
  - deep + turbid → Barker-13
  - turbid → geometric sweep
  - otherwise → LFM chirp
- Manual override of modulation and window from Vivado VIO
- Parameters swap atomically at ping boundaries, so no pulse is ever changed mid-flight
- Adaptation latency ≤ 1 ping
- 8-bit R-2R output with all bits registered in the I/O blocks on one clock edge (minimal glitches)
- VIO fallback sliders for bring-up without pots
- Self-checking testbench: **159/159 checks pass** across 9 scenarios

### Measured / simulated results

| Metric | Value | Source |
|---|---|---|
| Range resolution (clear reef) | 0.89 cm (84 kHz sweep) | RTL simulation |
| Range sidelobes, Hann LFM | −43.8 dB (−13.9 dB unwindowed) | RTL output, matched filter |
| Barker-13 sidelobes | −20.3 dB | RTL output, matched filter |
| Energy per ping, clear reef vs fixed max | 94 % less | RTL parameters |
| Average energy saving, 3 scenarios | 54 % | RTL parameters |
| Salinity effect, same depth | 133 kHz (sea) vs 364 kHz (fresh) | RTL simulation |
| Timing at 100 MHz | Met, WNS +0.41 ns | Vivado implementation |
| FPGA on-chip power | 110 mW | Vivado `report_power` |
| Utilisation, XC7A35T | 2,538 LUT (12.2 %), 2,914 FF (7.0 %), 0.5 BRAM, 13 DSP (14.4 %) | Vivado, incl. VIO |

---

## 4. Technologies Used

### Hardware
- EDGE Artix-7 FPGA board (Xilinx XC7A35T, 100 MHz oscillator)
- Built-in XADC (12-bit) for the 4 potentiometer inputs
- 8-bit R-2R resistor-ladder DAC (hand-matched 1 kΩ resistors)
- NE5532 dual op-amp: unity-gain buffer + 2nd-order MFB low-pass (fc ≈ 760 kHz)
- ±9 V supply (two 9 V batteries or a dual bench supply)
- 4 × 10 kΩ linear potentiometers (B10K) with 22 kΩ series resistors
- Oscilloscope with FFT for validation

### Software & Tools
- Verilog (RTL design)
- Xilinx Vivado (synthesis, implementation, XSIM simulation, VIO IP)
- Icarus Verilog (optional, fast command-line simulation)
- Python 3 with NumPy, SciPy, Matplotlib (ROM tables and waveform analysis)

---

## 5. Repository Structure

```
├── rtl/
│   ├── edge_top.v             # board wrapper: XADC, VIO, R-2R outputs
│   ├── xadc_pots.v            # XADC reader for the 4 pots
│   ├── adaptive_sonar_top.v   # ping sequencer (PREP → TX → LISTEN), sample timer
│   ├── param_controller.v     # physics-driven adaptation engine
│   └── dds_waveform_gen.v     # DDS, sine/window ROMs, modulations
├── mem/
│   ├── sine_quarter.mem       # 1024-point quarter-wave sine
│   ├── window.mem             # Hann / Hamming / Blackman / Tukey tables
│   ├── soundspeed.mem         # Medwin sound-speed correction vs temperature
│   └── salinity.mem           # absorption + sound-speed correction vs salinity
├── constraints/
│   └── edge_top.xdc           # EDGE Artix-7 pin assignments
├── sim/
│   ├── tb_adaptive_sonar_v3.v # self-checking testbench (159 checks)
│   └── analyze_tx.py          # spectrograms, sidelobe and SFDR analysis
├── scripts/
│   └── gen_tables.py          # regenerates all .mem files
└── docs/                      # schematic, architecture diagram, RTL waveforms, slides
```

---

## 6. Steps to Install and Run

### A. Generate the ROM tables (optional; the .mem files are included)
```bash
pip install numpy scipy matplotlib
python3 scripts/gen_tables.py
```

### B. Create the Vivado project
1. Create a new RTL project for part **xc7a35tftg256-1** (EDGE Artix-7).
2. **Add design sources:** all files in `rtl/` and all four files in `mem/`. Set the file filter to "All Files" so the `.mem` files show up. They must be added as design sources so synthesis can initialise the block RAMs.
3. **Add constraints:** `constraints/edge_top.xdc`.
4. Set **`edge_top`** as the top module.

### C. Generate the VIO IP
IP Catalog → **VIO (Virtual Input/Output)**, component name **`vio_0`**, with 6 input probes and 8 output probes:

| Probe | Width | Initial value | Function |
|---|---|---|---|
| probe_in0 | 5 | — | {amp_en, tx_active, active_mode[2:0]} |
| probe_in1 | 10 | — | centre frequency (kHz) |
| probe_in2 | 32 | — | {depth, turbidity, temperature, salinity} |
| probe_in3 | 24 | — | pulse width (samples) |
| probe_in4 | 8 | — | amplitude |
| probe_in5 | 12 | — | XADC die temperature |
| probe_out0 | 1 | 0x0 | mode override enable |
| probe_out1 | 3 | 0x1 | override mode (0 CW, 1 LFM up, 2 LFM down, 3 Barker, 4 geo) |
| probe_out2 | 3 | 0x1 | window (0 rect, 1 Hann, 2 Hamming, 3 Blackman, 4 Tukey) |
| probe_out3 | 8 | 0x14 | VIO depth |
| probe_out4 | 8 | 0x0A | VIO turbidity |
| probe_out5 | 8 | 0x80 | VIO temperature |
| probe_out6 | 1 | 0x0 | 1 = use VIO sliders instead of pots |
| probe_out7 | 8 | 0xDF | VIO salinity |

### D. Simulate
1. Add `sim/tb_adaptive_sonar_v3.v` as a simulation source and **Set as Top**.
2. No path edits are needed: with `MEM_DIR` undefined, the testbench loads the `.mem` files by name, which is how Vivado finds them.
3. Settings → Simulation: `xsim.simulate.runtime` = `all`, and untick `log_all_signals`.
4. Run Behavioral Simulation. The Tcl console should end with:
   ```
   ==== 159 checks, 0 failures -> ALL PASS ====
   ```
5. The run writes `tx_samples.csv`. Analyse it with:
   ```bash
   python3 sim/analyze_tx.py
   ```

Command-line alternative with Icarus Verilog (run from `sim/`):
```bash
iverilog -g2012 -DMEM_DIR=\"../mem/\" -o sim tb_adaptive_sonar_v3.v ../rtl/adaptive_sonar_top.v ../rtl/param_controller.v ../rtl/dds_waveform_gen.v
vvp -n sim
```

### E. Build and program
1. Generate Bitstream, then check the timing summary shows **positive WNS**.
2. Hardware Manager → Open Target → Auto Connect → Program Device.
3. Open `hw_vio_1`. Set `use_vio` = 1 to test with sliders, or 0 to use the pots.

---

## 7. Hardware Connections

### FPGA pins (EDGE Artix-7)

| Signal | FPGA pin | Board connector |
|---|---|---|
| r2r[0] (LSB) … r2r[7] (MSB) | M16, N16, P15, P16, R15, R16, T14, T15 | 20-pin expansion, pins 5–12 |
| amp_en | N13 | expansion pin 13 |
| scope_trig | P13 | expansion pin 14 → scope EXT TRIG |
| 3.3 V / GND | — | expansion pin 3 / pin 4 |
| Depth pot | C3 / C2 (AD6P/N) | J13 |
| Turbidity pot | B2 / A2 (AD14P/N) | J13 |
| Temperature pot | C1 / B1 (AD7P/N) | J13 |
| Salinity pot | E2 / D1 (AD15P/N) | J13 |

### Analog front end
```
r2r[7:0] → R-2R ladder (R = 1 kΩ, 2R = 2 × 1 kΩ) → 150 pF to GND
         → 1 µF → 100 kΩ to GND → NE5532 A (unity buffer)
         → NE5532 B MFB low-pass (R1 = R3 = 2 kΩ, R2 = 470 Ω, 470 pF, 100 pF)
         → 100 Ω → transducer / oscilloscope
```

**Potentiometers** (×4): 3.3 V → 22 kΩ → 10 kΩ pot → GND. The wiper goes to VAUXP with 10 nF to GND, and VAUXN goes to GND. The wiper must stay within 0–1 V.

**Ground:** the FPGA GND, breadboard GND, ±9 V midpoint and scope ground must all be connected together.

---

## 8. Configuration Parameters

No environment variables are needed. The design is configured through Verilog parameters:

| Parameter | File | Default | Meaning |
|---|---|---|---|
| `CLK_HZ` | edge_top.v | 100 000 000 | board clock; must match the `.xdc` clock period |
| `FS_HZ` | adaptive_sonar_top.v | 10 000 000 | DAC sample rate (CLK_HZ / FS_HZ ≥ 4) |
| `AUX_DEPTH/TURB/TEMP/SAL` | edge_top.v | 6, 14, 7, 15 | XADC channel for each pot |
| `F_MIN_HZ / F_MAX_HZ` | param_controller.v | 112 000 / 450 000 | centre-frequency range |
| `PW_MIN_US / PW_MAX_US` | param_controller.v | 1000 / 8000 | pulse-length range |
| `R_MIN_M / R_MAX_M` | param_controller.v | 10 / 150 | range (sets the listen window) |
| `AMP_LEAD_US / AMP_TAIL_US` | adaptive_sonar_top.v | 200 / 50 | analog power-up lead and ring-down tail |

---

**Team Hermes** · SIH 2026 · PS SIH26058
