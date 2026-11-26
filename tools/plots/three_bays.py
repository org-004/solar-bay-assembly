#!/usr/bin/env python3
"""Three bays back to back in a committed plan: a picture and a table.

Usage (from tools/plots/):
  python3 -I three_bays.py --table PLAN.json
      prints, for each arm and each boundary between bays, the time from the
      arm's last step of bay b to its first step of bay b+1, split by what
      the cell is doing meanwhile, and each boundary's events with the
      instant the holder is empty (from layouts/<name>.json beside plans/;
      standard library only)
  $PY three_bays.py PLAN.json OUT.png
      draws one row per arm and one row for the bay's events, coloured by
      segment kind, with the bays' boundaries marked (needs matplotlib)

A plan of several bays is src/layout_bays.h's: each step number n of bay b is
n + b N, and the events carry their bay. An arm's "step" here is any segment
but a wait: its time between bays is waiting only.
"""
import json
import math
import os
import sys

KINDS = ["wait", "move", "approach", "work", "retreat"]
COLOURS = {"wait": "#e8e8e8", "move": "#4c78a8", "approach": "#f58518", "work": "#54a24b",
           "retreat": "#b279a2"}
EVENT_COLOURS = {"tube_in": "#72b7b2", "clamp": "#eeca3b", "release": "#ff9da6",
                 "bay_out": "#9d755d"}
# The parts of an arm's time between bays, in time order.
PARTS = ["last steps", "release", "back to wait", "bay out", "gap", "tube in", "clamp",
         "after clamp"]


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def bay_of_step(plan):
    return {s["number"]: s.get("bay", 0) for s in plan["steps"]}


def events(plan):
    out = {}
    for e in plan["events"]:
        out[(e.get("bay", 0), e["kind"])] = e
    return out


def overlap(a0, a1, b0, b1):
    return max(0.0, min(a1, b1) - max(a0, b0))


def between(plan):
    """Rows of (arm, b, last_end, first_start, {part: seconds}, last_what, first_what)."""
    bays = plan.get("bays", 1)
    of = bay_of_step(plan)
    what = {s["number"]: s["what"] for s in plan["steps"]}
    ev = events(plan)
    rows = []
    for arm in plan["arms"]:
        work = [g for g in arm["segments"] if g["kind"] != "wait"]
        for b in range(bays - 1):
            last = max((g for g in work if of.get(g["step"]) == b), key=lambda g: g["t1"])
            first = min((g for g in work if of.get(g["step"]) == b + 1), key=lambda g: g["t0"])
            a0, a1 = last["t1"], first["t0"]
            rel, out = ev[(b, "release")], ev[(b, "bay_out")]
            tin, clamp = ev[(b + 1, "tube_in")], ev[(b + 1, "clamp")]
            spans = {
                "last steps": (-1e18, rel["t0"]),
                "release": (rel["t0"], rel["t1"]),
                "back to wait": (rel["t1"], out["t0"]),
                "bay out": (out["t0"], out["t1"]),
                "gap": (out["t1"], tin["t0"]),
                "tube in": (tin["t0"], tin["t1"]),
                "clamp": (clamp["t0"], clamp["t1"]),
                "after clamp": (clamp["t1"], 1e18),
            }
            parts = {p: overlap(a0, a1, *spans[p]) for p in PARTS}
            rows.append((arm["name"], b, a0, a1, parts, what[last["step"]], what[first["step"]]))
    return rows


def holder_empty(layout, bay_out):
    """src/layout_bays.h, HolderEmptyAt: the end of bay_out.path's first leg."""
    path = layout["bay_out"]["path"]
    legs = [math.dist(path[i - 1], path[i]) for i in range(1, len(path))]
    if not legs or sum(legs) <= 0.0:
        return bay_out["t1"], 0.0, 0.0
    t = bay_out["t0"] + (bay_out["t1"] - bay_out["t0"]) * legs[0] / sum(legs)
    return t, legs[0], sum(legs)


def table(plan, layout):
    print("# Between bays, %s (plans/%s.json): for each arm and boundary, the arm's last "
          "step of bay b ends, its first step of bay b+1 starts, and the time between, split "
          "by what the cell does meanwhile; seconds" % (plan["layout"], plan["layout"]))
    print("arm,boundary,last_step,ends_s,first_step,starts_s,between_s," + ",".join(
        p.replace(" ", "_") + "_s" for p in PARTS))
    for arm, b, a0, a1, parts, lw, fw in between(plan):
        total = a1 - a0
        assert abs(sum(parts.values()) - max(0.0, total)) < 1e-6, (arm, b)
        print("%s,%d-%d,%s,%.3f,%s,%.3f,%.3f,%s" % (
            arm, b, b + 1, lw, a0, fw, a1, total, ",".join("%.3f" % parts[p] for p in PARTS)))
    ev = events(plan)
    rows = between(plan)
    for b in range(plan.get("bays", 1) - 1):
        out, tin = ev[(b, "bay_out")], ev[(b + 1, "tube_in")]
        last = max((r for r in rows if r[1] == b), key=lambda r: r[2])
        print("# boundary %d-%d: the last arm to finish bay %d is %s, whose %s ends at %.3f s; "
              "bay_out starts at %.3f s" % (b, b + 1, b, last[0], last[5], last[2], out["t0"]))
        clamp = ev[(b + 1, "clamp")]
        rel = ev[(b, "release")]
        empty, lift, way = holder_empty(layout, out)
        print("# boundary %d-%d: release %.3f-%.3f, bay_out %.3f-%.3f, tube_in %.3f-%.3f, "
              "clamp %.3f-%.3f; from release to the clamp's end %.3f s; the holder is empty "
              "at %.3f s (bay_out's first leg, %.3f m of its %.3f m), %.3f s before bay_out ends"
              % (b, b + 1, rel["t0"], rel["t1"], out["t0"], out["t1"], tin["t0"], tin["t1"],
                 clamp["t0"], clamp["t1"], clamp["t1"] - rel["t0"], empty, lift, way,
                 out["t1"] - empty))


def picture(plan, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    arms = plan["arms"]
    rows = [a["name"] for a in arms] + ["bay events"]
    end = max(g["t1"] for a in arms for g in a["segments"])
    fig, ax = plt.subplots(figsize=(16, 4.6), dpi=130)
    h = 0.7
    for i, a in enumerate(arms):
        y = len(rows) - 1 - i
        for kind in KINDS:
            bars = [(g["t0"], g["t1"] - g["t0"]) for g in a["segments"] if g["kind"] == kind]
            ax.broken_barh(bars, (y - h / 2, h), facecolors=COLOURS[kind], edgecolor="none")
    for e in plan["events"]:
        ax.broken_barh([(e["t0"], e["t1"] - e["t0"])], (-h / 2, h),
                       facecolors=EVENT_COLOURS[e["kind"]], edgecolor="#333333", linewidth=0.4)
    bays = plan.get("bays", 1)
    ev = events(plan)
    for b in range(bays):
        tin, out = ev[(b, "tube_in")], ev[(b, "bay_out")]
        ax.axvline(tin["t0"], color="#222222", linewidth=1.2, linestyle="--")
        ax.axvline(out["t1"], color="#222222", linewidth=1.2, linestyle="--")
        if b % 2 == 1:
            ax.axvspan(tin["t0"], out["t1"], color="#1f5fa8", alpha=0.05)
        ax.text((tin["t0"] + out["t1"]) / 2, len(rows) - 0.35, "bay %d: %.2f to %.2f s"
                % (b, tin["t0"], out["t1"]), ha="center", va="bottom", fontsize=10)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels(list(reversed(rows)))
    ax.set_ylim(-0.6, len(rows) - 0.05)
    ax.set_xlim(0, end)
    ax.set_xlabel("plan time (s); dashed lines: each bay's tube_in start and bay_out end")
    ax.set_title("%s: three bays back to back (plans/%s.json)" % (plan["layout"], plan["layout"]),
                 fontsize=12, pad=18)
    handles = [Patch(color=COLOURS[k], label=k) for k in KINDS]
    handles += [Patch(facecolor=EVENT_COLOURS[k], edgecolor="#333333", label=k)
                for k in ["tube_in", "clamp", "release", "bay_out"]]
    ax.legend(handles=handles, ncol=9, loc="upper center", bbox_to_anchor=(0.5, -0.17),
              frameon=False, fontsize=9)
    fig.tight_layout()
    fig.savefig(out_png)


def main(argv):
    if len(argv) == 2 and argv[0] == "--table":
        plan = load(argv[1])
        # The layout beside the plan: plans/<name>.json -> layouts/<name>.json.
        root = os.path.dirname(os.path.dirname(os.path.abspath(argv[1])))
        table(plan, load(os.path.join(root, "layouts", plan["layout"] + ".json")))
        return 0
    if len(argv) == 2:
        picture(load(argv[0]), argv[1])
        return 0
    sys.stderr.write(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
