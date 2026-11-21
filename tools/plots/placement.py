"""docs/img/placement.png: where modules landed relative to the slot.
Usage: placement.py OUT.png CAMERA.csv MISCAL.csv CHECK.csv CHECK_MISCAL.csv
(run_cell --trials 20 CSVs; columns along_mm, across_mm, outcome)."""
import sys
import numpy as np
from matplotlib.patches import Circle
from common import *

names = ["Camera only", "Pallet camera off\n(1 mm, 0.05 deg)",
         "Same, with the check", "Check camera also off\n(1 mm, 0.05 deg)"]
cols = [BLUE, ORANGE, AQUA, VIOLET]
fig, axs = plt.subplots(1, 4, figsize=(14, 4.9), sharex=True, sharey=True)
for ax, nm, col, f in zip(axs, names, cols, sys.argv[2:6]):
    rows = [l.strip().split(",") for l in open(f) if l.strip() and not l.startswith(("seed", "#"))]
    h = open(f).readline().strip().split(",")
    ia, ic, io = h.index("along_mm"), h.index("across_mm"), h.index("outcome")
    ok = [r for r in rows if r[io] == "placed"]
    x = np.array([float(r[ia]) for r in ok]); y = np.array([float(r[ic]) for r in ok])
    r = np.hypot(x, y)
    ax.add_patch(Circle((0, 0), 3, fill=False, ec=INK2, lw=1.6, ls="--", zorder=2))
    ax.axhline(0, color=GRID, lw=1); ax.axvline(0, color=GRID, lw=1)
    ax.scatter(x, y, s=38, color=col, zorder=3, edgecolor=SURFACE, linewidth=0.8)
    ax.set_aspect("equal")
    ax.set_title(nm, fontsize=12.5, loc="left")
    ax.text(0.03, 0.03, f"{len(ok)} of {len(rows)} placed\nworst {r.max():.1f} mm, rms {np.sqrt((r**2).mean()):.1f} mm",
            transform=ax.transAxes, fontsize=11, va="bottom")
    ax.set_xlabel("Along the tube (mm)")
    print(nm.replace("\n", " "), len(ok), len(rows), "worst %.2f rms %.2f" % (r.max(), np.sqrt((r**2).mean())))
axs[0].set_ylabel("Across the tube (mm)")
axs[0].set_xlim(-6, 6); axs[0].set_ylim(-6, 6)
fig.suptitle("Placement error against the slot, 20 pallets each; dashed circle: 3 mm tolerance. "
             "The check pulls the cloud back to the centre", x=0.01, ha="left", fontsize=13.5)
fig.tight_layout()
save(fig, sys.argv[1])
