"""docs/img/carry.png from print_plan CSV (50 base seeds). print_plan gives no
paths or joint angles, only per-seed numbers plus the scripted carry's own
value in the '# ' summary lines, so the figure compares those.
Usage: carry.py IN.csv OUT.png"""
import re
import sys
import numpy as np
from common import *

rows, scripted = [], {}
for line in open(sys.argv[1]):
    if line.startswith("#"):
        m = re.search(r"length m of motion.*scripted ([\d.]+)", line)
        if m: scripted["length"] = float(m.group(1))
        m = re.search(r"smallest clearance.*scripted ([\d.]+)", line)
        if m: scripted["clear"] = float(m.group(1))
        m = re.search(r"trajectory s.*scripted moves.*?([\d.]+) *\(?stopping|scripted moves \(stopping at each waypoint\) ([\d.]+)", line)
        if m: scripted["dur"] = float(m.group(1) or m.group(2))
    elif not line.startswith("seed"):
        rows.append(line.strip().split(","))
seed = np.array([int(r[0]) for r in rows])
length = np.array([float(r[6]) for r in rows])
clear = np.array([float(r[7]) for r in rows])
dur = np.array([float(r[9]) for r in rows])
assert len(scripted) == 3, scripted

fig, ax = plt.subplots(1, 3, figsize=(14, 5.4))
panels = [(dur, "dur", "Carry duration (s)", "Duration"),
          (length, "length", "Length of motion (m)", "Length of motion"),
          (clear, "clear", "Smallest clearance (mm)", "Smallest clearance")]
for a, (y, k, lab, ttl) in zip(ax, panels):
    a.scatter(seed, y, s=26, color=BLUE, zorder=3,
              label=f"Planned, 50 seeds (median {np.median(y):.2f})")
    a.axhline(scripted[k], color=ORANGE, lw=2.4,
              label=f"Hand-chosen ({scripted[k]:.2f})")
    a.set_xlabel("Base seed")
    a.set_ylabel(lab)
    a.set_title(ttl, loc="left")
    lo = min(y.min(), scripted[k]); hi = max(y.max(), scripted[k])
    a.set_ylim(lo - 0.12 * (hi - lo), hi + 0.42 * (hi - lo))
    a.legend(loc="upper center", fontsize=10.5, handlelength=1.2)
fig.suptitle("Planned carry vs hand-chosen carry: shorter and faster, but nearer the "
             "obstacles (planning margin 45 mm)", x=0.01, ha="left", fontsize=14)
fig.tight_layout()
save(fig, sys.argv[2])
