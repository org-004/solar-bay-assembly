"""docs/img/check_depth.png: the upward check camera's noisy depth image of the
held module's underside, with the outline and centre the finder found.

Data (one trial, same seed, same picture):
  print_check -- --noise --first-seed N --pgm DEPTH.pgm 2> /dev/null > OUTLINE.csv
DEPTH.pgm is the image; OUTLINE.csv has the finder's four corners and centre
projected with the calibrated camera (corner0..3, centre: u_px, v_px), and the
measured and true offsets from the design grip.
Usage: check_depth.py DEPTH.pgm OUTLINE.csv OUT.png"""
import sys
import numpy as np
from matplotlib.patches import Polygon
from common import *

img = read_pgm16(sys.argv[1])
rows = [l.strip().split(",") for l in open(sys.argv[2]) if l.strip()][1:]
pts = {r[0]: np.array(r[1:], float) for r in rows}
# Project's (u, v) puts pixel (c, r)'s centre at (c + 0.5, r + 0.5); imshow
# puts it at (c, r).
corners = np.array([pts[f"corner{k}"][:2] for k in range(4)]) - 0.5
centre = pts["centre"][:2] - 0.5
meas, true = pts["measured_offset"], pts["true_offset"]

shown = np.where(img > 0, img / 1000.0, np.nan)
lo, hi = np.nanpercentile(shown, [0.5, 99.5])
fig, ax = plt.subplots(figsize=(14, 7.2))
im = ax.imshow(shown, cmap=depth_cmap(), interpolation="nearest", vmin=lo, vmax=hi)
ax.add_patch(Polygon(corners, closed=True, fill=False, ec=ORANGE, lw=2.4))
ax.plot(*centre, "+", color=ORANGE, ms=18, mew=2.8)
ax.grid(False)
ax.set_xlabel("Pixel column"); ax.set_ylabel("Pixel row")
cb = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.02)
cb.set_label("Depth from camera (m)")
txt = (f"measured offset from the design grip\n"
       f"x {meas[0]:+.1f}  y {meas[1]:+.1f}  z {meas[2]:+.1f} mm\n"
       f"turns {meas[3]:+.2f}, {meas[4]:+.2f}, {meas[5]:+.2f} deg\n"
       f"(true: {true[0]:+.1f}, {true[1]:+.1f}, {true[2]:+.1f} mm)")
ax.text(0.015, 0.975, txt, transform=ax.transAxes, fontsize=12, color=INK,
        va="top", bbox=dict(fc=SURFACE, ec="none", alpha=0.9))
ax.text(0.985, 0.03, "box: outline the finder found (orange), + its centre",
        transform=ax.transAxes, fontsize=12, color=INK, ha="right", va="bottom",
        bbox=dict(fc=SURFACE, ec="none", alpha=0.9))
ax.set_title("Check camera: the held module's underside, depth noise on, with the outline the finder found",
             loc="left", fontsize=14)
fig.tight_layout()
save(fig, sys.argv[3])
print("depth range m", lo, hi, "centre px", centre)
