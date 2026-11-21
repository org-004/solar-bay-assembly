"""docs/img/pallet_depth.png: the pallet camera's noisy depth image with the
glass outline and centre, and the glass points flattened onto their plane.

Data (same seed, same picture):
  print_depth -- --noise --pgm DEPTH.pgm        (image)
  print_depth -- --noise --points               (x,y,z,u,v per pixel, CSV)
No program prints the finder's fitted outline, so the outline here is made
from those points by this script for display only: the glass points are
picked by a trimmed plane fit and boxed along their principal axes.
Usage: pallet_depth.py DEPTH.pgm POINTS.csv OUT.png"""
import sys
import numpy as np
from matplotlib.patches import Polygon
from common import *

img = read_pgm16(sys.argv[1])
P = np.loadtxt(sys.argv[2], delimiter=",", skiprows=1)
X, uv = P[:, :3], P[:, 3:5]

# --- pick the front glass: design plane (lean 10 deg at (1.95, 0, 0.75)),
# then a trimmed least-squares plane, 5 passes.
a = np.radians(10)
n = np.array([-np.cos(a), 0, np.sin(a)]); c = np.array([1.95, 0, 0.75])
sel = (np.abs((X - c) @ n) < 0.04) & (X[:, 2] > 0.3) & (np.abs(X[:, 1]) < 1.5)
for _ in range(5):
    Q = X[sel]; c = Q.mean(0)
    n = np.linalg.svd(Q - c, full_matrices=False)[2][2]
    if n[0] > 0: n = -n
    d = (X - c) @ n
    sel = (np.abs(d) < 0.008) & (X[:, 2] > 0.3) & (np.abs(X[:, 1]) < 1.5)
Q, G = X[sel], uv[sel]
# in-plane axes: long (along the glass's horizontal side), up
u_ax = np.linalg.svd(Q - Q.mean(0), full_matrices=False)[2][0]
if u_ax[1] < 0: u_ax = -u_ax
v_ax = np.cross(n, u_ax)
if v_ax[2] < 0: v_ax = -v_ax
cen = Q.mean(0)
s = (Q - cen) @ u_ax; t = (Q - cen) @ v_ax; z = (Q - cen) @ n
lo_s, hi_s = np.percentile(s, [0.05, 99.95]); lo_t, hi_t = np.percentile(t, [0.05, 99.95])
corners2 = np.array([[lo_s, lo_t], [hi_s, lo_t], [hi_s, hi_t], [lo_s, hi_t]])
mid2 = np.array([(lo_s + hi_s) / 2, (lo_t + hi_t) / 2])
# homography plane(s,t) -> pixel(u,v), least squares on the glass points
idx = np.random.default_rng(0).choice(len(s), 4000, replace=False)
A = []
for (x, y), (uu, vv) in zip(np.c_[s, t][idx], G[idx]):
    A.append([x, y, 1, 0, 0, 0, -uu * x, -uu * y, -uu])
    A.append([0, 0, 0, x, y, 1, -vv * x, -vv * y, -vv])
H = np.linalg.svd(np.array(A), full_matrices=False)[2][-1].reshape(3, 3)
def to_px(pts):
    q = np.c_[pts, np.ones(len(pts))] @ H.T
    return q[:, :2] / q[:, 2:]
cpx, mpx = to_px(corners2), to_px(mid2[None])[0]

fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 5.6),
                             gridspec_kw={"width_ratios": [1, 1.25]})
shown = np.where(img > 0, img / 1000.0, np.nan)
im = a1.imshow(shown, cmap=depth_cmap(), interpolation="nearest",
                vmin=3.3, vmax=5.2)
a1.add_patch(Polygon(cpx, closed=True, fill=False, ec=ORANGE, lw=2.4))
a1.plot(*mpx, "+", color=ORANGE, ms=16, mew=2.6)
a1.set_xlabel("Pixel column"); a1.set_ylabel("Pixel row")
a1.grid(False)
a1.set_title("Depth image, noise on", loc="left")
cb = fig.colorbar(im, ax=a1, fraction=0.046, pad=0.03)
cb.set_label("Depth from camera (m)")
a1.text(0.02, 0.03, "box: glass outline (orange), + its centre", color=INK,
        transform=a1.transAxes, fontsize=11,
        bbox=dict(fc=SURFACE, ec="none", alpha=0.85))

a2.scatter(s * 1000, t * 1000, s=1.2, color=BLUE, alpha=0.35, rasterized=True)
a2.add_patch(Polygon(corners2 * 1000, closed=True, fill=False, ec=ORANGE, lw=2.4))
a2.plot(mid2[0] * 1000, mid2[1] * 1000, "+", color=ORANGE, ms=16, mew=2.6)
a2.set_aspect("equal")
a2.set_xlabel("Along the glass's long side (mm)")
a2.set_ylabel("Up the glass (mm)")
a2.set_title(f"Glass points flattened onto their plane ({len(s):,} points)", loc="left")
a2.text(0.5, 0.02, f"box {(hi_s-lo_s)*1000:.0f} x {(hi_t-lo_t)*1000:.0f} mm", ha="center",
        transform=a2.transAxes, fontsize=11, bbox=dict(fc=SURFACE, ec="none", alpha=0.85))
fig.suptitle(f"Pallet camera: the front module's glass, found through depth noise ({z.std()*1000:.1f} mm rms off the plane)", x=0.01, ha="left", fontsize=14)
fig.tight_layout()
save(fig, sys.argv[3])
print("long", (hi_s-lo_s)*1000, "short", (hi_t-lo_t)*1000, "rms mm", z.std()*1000)
