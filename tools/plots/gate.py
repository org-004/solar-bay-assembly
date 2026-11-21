"""docs/img/gate.png: position error of every module of whole pallets against
the 3 mm tolerance, one strip per seed.
Usage: gate.py OUT.png SEED1.csv SEED2.csv ...   (run_cell --modules 30 CSVs;
position error = sqrt(along_mm^2 + across_mm^2), as src/trial_runner.h)."""
import sys
import numpy as np
from common import *

rng = np.random.default_rng(0)
fig, (a, b) = plt.subplots(1, 2, figsize=(14, 5.4), gridspec_kw={"width_ratios": [1.5, 1]})
allerr, fails, placed_n, n_expected = [], 0, 0, 0
for i, f in enumerate(sys.argv[2:]):
    lines = [l.strip().split(",") for l in open(f) if l.strip() and not l.startswith("#")]
    h, rows = lines[0], lines[1:]
    seed = int(rows[0][h.index("seed")])
    ok = [r for r in rows if r[h.index("outcome")] == "placed"]
    e = np.hypot([float(r[h.index("along_mm")]) for r in ok],
                 [float(r[h.index("across_mm")]) for r in ok])
    allerr.append(e); placed_n += len(ok); n_expected += 30
    a.scatter(i + rng.uniform(-0.28, 0.28, len(e)), e, s=30, color=BLUE, alpha=0.8,
              edgecolor=SURFACE, linewidth=0.6, zorder=3)
    a.text(i, 3.45, f"{len(ok)}/30", ha="center", fontsize=11, color=INK2)
    print("seed", seed, "placed", len(ok), "worst %.2f" % e.max())
a.axhline(3, color=RED, lw=2, ls="--")
a.text(len(allerr) - 0.55, 3.0, "3 mm tolerance", color=INK, ha="right", va="bottom", fontsize=11.5)
a.set_xticks(range(len(allerr))); a.set_xticklabels([f"{s}" for s in range(1, len(allerr) + 1)])
a.set_xlabel("Seed (one whole pallet of 30 modules each)")
a.set_ylabel("Position error in the rails' plane (mm)")
a.set_ylim(0, 3.8)
E = np.concatenate(allerr)
b.hist(E, bins=np.arange(0, 3.01, 0.1), color=BLUE, edgecolor=SURFACE, linewidth=1)
b.axvline(3, color=RED, lw=2, ls="--")
b.set_xlim(0, 3.2)
b.set_xlabel("Position error (mm)"); b.set_ylabel("Modules")
fig.suptitle(f"Gate settings, {len(allerr)} whole pallets: {placed_n} of {n_expected} modules placed, "
             f"worst {E.max():.2f} mm, median {np.median(E):.2f} mm", x=0.01, ha="left", fontsize=14)
fig.tight_layout()
save(fig, sys.argv[1])
print("n", len(E), "worst", E.max(), "median", np.median(E), "over3", (E > 3).sum())
