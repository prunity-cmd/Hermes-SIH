#!/usr/bin/env python3
"""
Analyses tx_samples.csv written by tb_adaptive_sonar_v3 (the real RTL output):
  * spectrogram of one ping per scenario  -> tx_spectrograms.png
  * pulse-compression (matched filter) range sidelobes, Hann vs rect LFM
  * SFDR of the CW pulse at the 8-bit R2R resolution
Numbers printed here are the ones to quote on the slides.
"""
import numpy as np, csv, collections
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.signal import spectrogram, correlate, hilbert

FS = 10e6
MODES = {0: "CW", 1: "LFM up", 2: "LFM down", 3: "Barker-13", 4: "Geo sweep"}
pings = collections.OrderedDict()
with open("tx_samples.csv") as f:
    for r in csv.DictReader(f):
        k = int(r["ping"])
        pings.setdefault(k, {"scen": int(r["scenario"]), "mode": int(r["mode"]), "x": []})
        pings[k]["x"].append(int(r["sample"]))

per_scen = collections.OrderedDict()
for k, p in pings.items():
    per_scen[p["scen"]] = p                      # keep the last (checked) ping
names = {1: "Clear reef", 2: "Muddy estuary", 3: "Deep sea", 4: "Deep fresh",
         5: "Deep+muddy", 6: "CW Blackman", 7: "Down Tukey", 8: "Barker reef", 9: "Reef rect"}

def q8(x):                                       # what the 8-bit R2R actually outputs
    return np.clip(np.round(np.asarray(x) / 256.0), -127, 127)

fig, ax = plt.subplots(3, 3, figsize=(14, 10))
for a, (s, p) in zip(ax.flat, per_scen.items()):
    x = q8(p["x"])
    f, t, S = spectrogram(x, FS, window="blackmanharris", nperseg=512, noverlap=480, nfft=2048)
    S_db = 10 * np.log10(S / S.max() + 1e-12)
    a.pcolormesh(t * 1e3, f / 1e3, S_db, shading="auto", vmin=-70, vmax=0)
    a.set_ylim(0, 600); a.set_title(f"{names.get(s, s)}: {MODES[p['mode']]}")
    a.set_xlabel("ms"); a.set_ylabel("kHz")
plt.tight_layout(); plt.savefig("tx_spectrograms.png", dpi=110)

def psl_db(x):                                   # peak range sidelobe after matched filter
    x = np.asarray(x, float)
    c = np.abs(hilbert(correlate(x, x, mode="full")))   # envelope, not the carrier
    c /= c.max()
    m = len(x) - 1
    i = m                                        # walk out of the main lobe to 1st null
    while i + 1 < len(c) and c[i + 1] < c[i]: i += 1
    return 20 * np.log10(c[i:].max())

def sfdr_db(x):
    x = np.asarray(x, float) * np.blackman(len(x))
    X = np.abs(np.fft.rfft(x, 1 << 16)); X /= X.max()
    k = X.argmax(); X[max(0, k - 250):k + 250] = 0   # +/-38 kHz main lobe
    return -20 * np.log10(X.max())

reef_hann, reef_rect = q8(per_scen[1]["x"]), q8(per_scen[9]["x"])
print("Range sidelobes (matched filter, 8-bit output):")
print("  LFM, Hann window : %.1f dB" % psl_db(reef_hann))
print("  LFM, rectangular : %.1f dB" % psl_db(reef_rect))
print("  Barker-13        : %.1f dB" % psl_db(q8(per_scen[5]["x"])))
print("SFDR, CW pulse     : %.1f dBc at 8 bits, %.1f dBc at 16 bits" %
      (sfdr_db(q8(per_scen[6]["x"])), sfdr_db(per_scen[6]["x"])))
B = 423.6e3 * 51 / 256
print("Reef range resolution c/2B: %.2f cm (B = %.1f kHz)" % (1500 / (2 * B) * 100, B / 1e3))
