"""docs/img/profile.png from print_profile CSV. Usage: profile.py IN.csv OUT.png"""
import sys
import numpy as np
from common import *

d = np.genfromtxt(sys.argv[1], delimiter=",", names=True)
t = d["time"]
fig, ax = plt.subplots(3, 1, figsize=(14, 8.6), sharex=True)
rows = [("position", "Position (rad)"), ("velocity", "Velocity (rad/s)"),
        ("acceleration", "Acceleration (rad/s$^2$)")]
for a, (k, lab) in zip(ax, rows):
    a.plot(t, d["trapezoid_" + k], color=ORANGE, lw=2.2, label="Trapezoid")
    a.plot(t, d["scurve_" + k], color=BLUE, lw=2.2, label="S-curve")
    a.set_ylabel(lab)
ax[-1].set_xlabel("Time (s)")
ax[0].legend(loc="lower right", ncols=2)
ax[0].set_title("A quarter turn of one joint: the trapezoid takes 1.18 s, the S-curve 1.29 s "
                "but never jumps in acceleration", loc="left")
fig.tight_layout()
save(fig, sys.argv[2])
