#!/usr/bin/env python3
"""
Generates the ROM init files used by the RTL ($readmemh). Run once, then add
the three .mem files to the Vivado project (Add Sources -> design sources) so
synthesis and simulation find them.

  sine_quarter.mem  1024 x 15-bit  quarter-wave sine magnitude, half-sample offset
  window.mem        4096 x 16-bit  8 window slots x 512 (first half, mirrored in RTL)
                                   0 rect, 1 Hann, 2 Hamming, 3 Blackman, 4 Tukey(0.25),
                                   5 Tukey(0.05) edge taper (auto for Barker), 6-7 rect
  soundspeed.mem     256 x 16-bit  Q1.15 of 1500/c(T), T = code*35/255 degC  (Medwin 1975, S=35, z=0)
  salinity.mem       256 x 32-bit  S = code*40/255 ppt, packed {7'b0, k[8:0], spd[15:0]}
                                   k   Q1.8  absorption at 300 kHz relative to S=35 (Francois-Garrison)
                                   spd Q1.15 c(15C,35)/c(15C,S): salinity sound-speed correction

Also prints LN_R_Q48 for param_controller (geometric-sweep constant).
"""
import math, os
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mem")

def w(path, vals, digits):
    with open(os.path.join(OUT, path), "w") as f:
        for v in vals:
            f.write(f"{v:0{digits}X}\n")

# ---- sine: round(32767 * sin((k+0.5) * pi/2 / 1024)) ----
sine = [round(32767 * math.sin((k + 0.5) * math.pi / 2 / 1024)) for k in range(1024)]
w("sine_quarter.mem", sine, 4)

# ---- windows, full-length N=1024 with half-sample offset, first half stored ----
def win(kind, x):              # x in (0, 0.5)
    c1, c2 = math.cos(2 * math.pi * x), math.cos(4 * math.pi * x)
    if kind == 1: return 0.5 - 0.5 * c1                       # Hann
    if kind == 2: return 0.54 - 0.46 * c1                     # Hamming
    if kind == 3: return 0.42 - 0.5 * c1 + 0.08 * c2          # Blackman
    if kind in (4, 5):                                        # Tukey, alpha 0.25 / 0.05
        a = 0.25 if kind == 4 else 0.05
        return 0.5 * (1 - math.cos(2 * math.pi * x / a)) if x < a / 2 else 1.0
    return 1.0                                                # rectangular

wins = []
for kind in range(8):
    for n in range(512):
        x = (n + 0.5) / 1024
        wins.append(min(65535, max(0, round(65535 * win(kind, x)))))
w("window.mem", wins, 4)

# ---- sound speed correction: 1500 / c(T), Medwin, S = 35 ppt, z = 0 ----
def medwin(T, S=35.0, z=0.0):
    return (1449.2 + 4.6 * T - 0.055 * T**2 + 0.00029 * T**3
            + (1.34 - 0.01 * T) * (S - 35) + 0.016 * z)

ss = [round(32768 * 1500.0 / medwin(code * 35.0 / 255)) for code in range(256)]
w("soundspeed.mem", ss, 4)

# ---- salinity: absorption scale (Francois & Garrison 1982) + sound speed ----
def fg(f, T, S, z=0.0, pH=8.0):          # dB/km, f in kHz
    c  = 1412 + 3.21 * T + 1.19 * S + 0.0167 * z
    A1 = 8.86 / c * 10 ** (0.78 * pH - 5)
    f1 = 2.8 * math.sqrt(max(S, 0.01) / 35) * 10 ** (4 - 1245 / (T + 273))
    A2 = 21.44 * S / c * (1 + 0.025 * T); P2 = 1 - 1.37e-4 * z + 6.2e-9 * z * z
    f2 = 8.17 * 10 ** (8 - 1990 / (T + 273)) / (1 + 0.0018 * (S - 35))
    A3 = 4.937e-4 - 2.59e-5 * T + 9.11e-7 * T * T - 1.50e-8 * T ** 3
    P3 = 1 - 3.83e-5 * z + 4.9e-10 * z * z
    return (A1 * f1 * f * f / (f1 * f1 + f * f) + A2 * P2 * f2 * f * f / (f2 * f2 + f * f)
            + A3 * P3 * f * f)

sal = []
for code in range(256):
    S = code * 40.0 / 255
    k = min(511, round(256 * fg(300, 15, S) / fg(300, 15, 35)))
    spd = round(32768 * medwin(15, 35) / medwin(15, S))
    sal.append((k << 16) | spd)
w("salinity.mem", sal, 8)
print("absorption @300 kHz: fresh %.1f, brackish(15) %.1f, sea(35) %.1f dB/km"
      % (fg(300, 15, 0), fg(300, 15, 15), fg(300, 15, 35)))

# ---- geometric sweep constant for BW_FRAC_Q8 = 51 (~20 % fractional bandwidth) ----
f = 51 / 256
r = (1 + f / 2) / (1 - f / 2)
print("LN_R_Q48 =", round(math.log(r) * 2**48))
print("c(0C) = %.1f m/s, c(35C) = %.1f m/s" % (medwin(0), medwin(35)))
