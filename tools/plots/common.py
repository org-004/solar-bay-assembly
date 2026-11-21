"""Shared style for the README figures. Palette: the dataviz skill's default
categorical slots 1, 2, 3, 7 (blue, orange, aqua, violet), light surface."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
SURFACE = "#fcfcfb"
BLUE, ORANGE, AQUA, VIOLET = "#2a78d6", "#eb6834", "#1baf7a", "#6250d6"
RED = "#d03b3b"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": INK2, "axes.labelcolor": INK, "text.color": INK,
    "xtick.color": INK2, "ytick.color": INK2,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.axisbelow": True,
    "font.size": 13, "axes.titlesize": 14, "axes.labelsize": 13,
    "legend.fontsize": 12, "legend.frameon": False,
    "figure.dpi": 100, "savefig.dpi": 100,
})


def save(fig, path):
    fig.savefig(path)  # figures are sized 14 in x dpi 100 = 1400 px wide
    print("wrote", path)


def read_pgm16(path):
    """16-bit binary PGM as an int array (print_depth --pgm)."""
    import numpy as np
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        w, h = map(int, f.readline().split())
        f.readline()
        a = np.frombuffer(f.read(), dtype=">u2")
    return a.reshape(h, w).astype(float)


def depth_cmap():
    """One-hue sequential (the palette's blue ramp), near = dark."""
    from matplotlib.colors import LinearSegmentedColormap
    return LinearSegmentedColormap.from_list(
        "depth", ["#0d366b", "#256abf", "#6da7ec", "#cde2fb"])
