#!/usr/bin/env python3
"""Where a bay's planned time goes, from a committed plan (run 032).

    python3 -B tools/time/account.py plans/<name>.json [--bay N]

Reads a plan written by //:plan_layout (src/layout_plan.h, "Plan") and
prints, for one bay (bay 0 by default):

  1. Per arm, the bay's time from its tube_in to its bay_out split by what
     the arm does: moving (planned, settle included) against the time
     model's figure for the same moves (Segment::model_s, settle included),
     on its approach and retreat legs (not timed by the model), at work
     (process times, tool changes apart), and waiting, by whom it waits for.
  2. The critical path: the chain of segments that sets the bay's time,
     walked back from bay_out. Where an arm waits on the path, the walk
     goes to what it waits for (the arm the wait's reason names, the tube's
     transfer, the tube coming in). Each stage's end and the arm whose chain
     ends it.
  3. The bay's excess over its time model (bay_planned_s - bay_model_s)
     split by cause along that path: moves slower than modelled, legs the
     model does not time, waits on the path, and the rest, which is work the
     model lets overlap and the plan does one after another (stage
     barriers, one pin arm with two modules of a stage).

It plans nothing and builds nothing; it needs only python3.
"""

import argparse
import collections
import json
import re
import sys

EPS = 1e-6


def wait_on(reason):
    """Whom or what a wait waits for, from its reason."""
    m = re.search(r"by (\w+)$", reason)
    if m:
        return m.group(1)
    m = re.match(r"for (\w+) to pass", reason)
    if m:
        return m.group(1) + " (passing)"
    if "move to stage" in reason:
        return "stage end"
    if "while the tube moves" in reason:
        return "transfer"
    if "come in and be clamped" in reason:
        return "tube in"
    if reason.startswith("bay out"):
        return "bay out"
    return reason


def work_kind(what):
    if "tool" in what and "change" in what:
        return "tool"
    return "process"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("plan")
    ap.add_argument("--bay", type=int, default=0)
    args = ap.parse_args()
    p = json.load(open(args.plan))
    bay = args.bay
    events = [e for e in p["events"] if e["bay"] == bay]
    ev = {e["kind"]: e for e in events if e["kind"] != "transfer"}
    transfers = [e for e in events if e["kind"] == "transfer"]
    b0, b1 = ev["tube_in"]["t0"], ev["bay_out"]["t1"]
    steps = {s["number"]: s for s in p["steps"] if s["bay"] == bay}
    phases = {ph["name"]: ph for ph in p["phases"] if ph["bay"] == bay}
    planned = b1 - b0
    model = sum(ph["model_s"] for ph in phases.values())
    print(f"# {p['layout']}, bay {bay}: planned {planned:.2f} s, "
          f"model {model:.2f} s, excess {planned - model:.2f} s")
    print("# phases (planned / model s): " + ", ".join(
        f"{n} {ph['planned_s']:.2f} / {ph['model_s']:.2f}"
        for n, ph in phases.items()))

    # 1. Per arm.
    arms = {}
    for a in p["arms"]:
        segs = [s for s in a["segments"] if s["t1"] > b0 + EPS and s["t0"] < b1 - EPS]
        arms[a["name"]] = segs
    print("\n## Per arm (s, bay window)")
    print("| arm | moves | moving planned | model, same moves | excess | legs (approach+retreat) "
          "| process | tool changes | waiting | busy % |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    waits = {}
    for name, segs in arms.items():
        c = collections.defaultdict(float)
        w = collections.defaultdict(float)
        nmov = 0
        mmodel = 0.0
        for s in segs:
            d = min(s["t1"], b1) - max(s["t0"], b0)
            if s["kind"] == "move":
                nmov += 1
                mmodel += s["model_s"]
                c["move"] += d
            elif s["kind"] in ("approach", "retreat"):
                c["legs"] += d
            elif s["kind"] == "work":
                c[work_kind(steps.get(s["step"], {}).get("what", ""))] += d
            else:
                c["wait"] += d
                w[wait_on(s["reason"])] += d
        waits[name] = w
        busy = c["move"] + c["legs"] + c["process"] + c["tool"]
        print(f"| {name} | {nmov} | {c['move']:.1f} | {mmodel:.1f} | {c['move'] - mmodel:+.1f} "
              f"| {c['legs']:.1f} | {c['process']:.1f} | {c['tool']:.1f} | {c['wait']:.1f} "
              f"| {100 * busy / planned:.0f} |")
    print("\n## Waiting, by whom or what (s)")
    keys = sorted({k for w in waits.values() for k in w})
    print("| arm | " + " | ".join(keys) + " |")
    print("|---|" + "---:|" * len(keys))
    for name, w in waits.items():
        print(f"| {name} | " + " | ".join(f"{w.get(k, 0.0):.1f}" for k in keys) + " |")

    # 2. Critical path, walked back from bay_out.
    def last_ending(arm, t):
        """The index of the arm's segment that ends at t (or the latest
        ending before it)."""
        best = None
        for i, s in enumerate(arms[arm]):
            if s["t1"] <= t + EPS and (best is None or s["t1"] > arms[arm][best]["t1"] + EPS):
                best = i
        return best

    def latest_work_before(t):
        """The arm whose latest non-wait segment ends last at or before t."""
        best, bt = None, -1.0
        for name, segs in arms.items():
            for i, s in enumerate(segs):
                if s["kind"] != "wait" and s["t1"] <= t + EPS and s["t1"] > bt + EPS:
                    best, bt = (name, i), s["t1"]
        return best

    path = []  # (arm or event, kind, t0, t1, segment or None)
    t = ev["bay_out"]["t1"]
    path.append(("bay out", "event", ev["bay_out"]["t0"], ev["bay_out"]["t1"], None))
    path.append(("release", "event", ev["release"]["t0"], ev["release"]["t1"], None))
    t = ev["release"]["t0"]
    cur = latest_work_before(t)
    guard = 0
    while cur is not None and guard < 100000:
        guard += 1
        arm, i = cur
        s = arms[arm][i]
        if s["t1"] < t - EPS:
            path.append((arm, "gap", s["t1"], t, None))
        if s["kind"] != "wait":
            path.append((arm, s["kind"], max(s["t0"], b0), s["t1"], s))
            t = s["t0"]
            cur = (arm, i - 1) if i > 0 else None
            continue
        who = wait_on(s["reason"])
        if who in arms:
            # The wait ends when what it waits for has happened; go to that
            # arm's segment ending at or before the wait's end. Only the part
            # of the wait after it is on the path.
            j = last_ending(who, s["t1"])
            gap_from = min(arms[who][j]["t1"], s["t1"])
            path.append((arm, "wait", gap_from, s["t1"], s))
            t = gap_from
            cur = (who, j)
            continue
        if who.endswith("(passing)"):
            path.append((arm, "wait", s["t0"], s["t1"], s))
            t = s["t0"]
            cur = (arm, i - 1) if i > 0 else None
            continue
        if who in ("stage end", "transfer"):
            tr = [e for e in transfers if e["t0"] <= s["t1"] + EPS and e["t1"] >= s["t1"] - EPS]
            tr = [e for e in transfers if abs(e["t1"] - s["t1"]) < 1e-3] or tr
            e = tr[0]
            path.append(("transfer", "event", e["t0"], e["t1"], None))
            t = e["t0"]
            cur = latest_work_before(t)
            continue
        if who == "tube in":
            path.append(("clamp", "event", ev["clamp"]["t0"], ev["clamp"]["t1"], None))
            path.append(("tube in", "event", ev["tube_in"]["t0"], ev["tube_in"]["t1"], None))
            t = b0
            break
        path.append((arm, "wait", s["t0"], s["t1"], s))
        t = s["t0"]
        cur = (arm, i - 1) if i > 0 else None
    path.reverse()

    print("\n## Critical path (s)")
    tot = collections.defaultdict(lambda: collections.defaultdict(float))
    mv_model = collections.defaultdict(float)
    for who, kind, t0, t1, s in path:
        d = t1 - t0
        if kind == "work":
            kind = work_kind(steps.get(s["step"], {}).get("what", ""))
        if kind in ("approach", "retreat"):
            kind = "legs"
        tot[who][kind] += d
        if kind == "move":
            mv_model[who] += s["model_s"]
    span = sum(t1 - t0 for _, _, t0, t1, _ in path)
    print(f"# path covers {span:.2f} s of {planned:.2f} s")
    print("| on the path | moving planned | model, same moves | legs | process | tool changes "
          "| waiting/gaps | events | total |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    sums = collections.defaultdict(float)
    for who in list(arms) + ["tube in", "clamp", "transfer", "release", "bay out"]:
        if who not in tot:
            continue
        k = tot[who]
        row = [k["move"], mv_model[who], k["legs"], k["process"], k["tool"],
               k["wait"] + k["gap"], k["event"]]
        for name, v in zip(["move", "model", "legs", "process", "tool", "wait", "event"], row):
            sums[name] += v
        total = row[0] + sum(row[2:])
        print(f"| {who} | " + " | ".join(f"{v:.1f}" for v in row) + f" | {total:.1f} |")
    print("| all | " + " | ".join(f"{sums[n]:.1f}" for n in
                                  ["move", "model", "legs", "process", "tool", "wait", "event"])
          + f" | {span:.1f} |")

    print("\n## On the path, by kind of step (s)")
    print("| kind of step | steps | moving planned | model, same moves | legs | process |")
    print("|---|---:|---:|---:|---:|---:|")
    by = collections.defaultdict(lambda: collections.defaultdict(float))
    seen = collections.defaultdict(set)
    for who, kind, t0, t1, s in path:
        if s is None or kind in ("wait", "gap"):
            continue
        what = re.sub(r" [0-9/]+$", "", steps.get(s["step"], {}).get("what", ""))
        key = f"{who}: {what}"
        seen[key].add(s["step"])
        k = "legs" if kind in ("approach", "retreat") else kind
        by[key][k] += t1 - t0
        if kind == "move":
            by[key]["model"] += s["model_s"]
    for key, k in sorted(by.items(), key=lambda kv: -sum(kv[1].values())):
        print(f"| {key} | {len(seen[key])} | {k['move']:.1f} | {k['model']:.1f} | {k['legs']:.1f} "
              f"| {k['work']:.1f} |")

    # Stages: which chain ends each stage.
    print("\n## The path, step by step (arm: steps, s)")
    runs = []
    for who, kind, t0, t1, s in path:
        label = who
        what = steps.get(s["step"], {}).get("what", "") if s else kind
        if runs and runs[-1][0] == label:
            runs[-1][2] = t1
            if what and what not in runs[-1][3] and kind != "wait":
                runs[-1][3].append(what)
        else:
            runs.append([label, t0, t1, [what] if kind != "wait" else ["wait"]])
    for label, t0, t1, whats in runs:
        shown = whats if len(whats) <= 4 else whats[:2] + ["..."] + whats[-1:]
        print(f"- {t0:7.2f} -> {t1:7.2f} ({t1 - t0:6.2f}) {label}: {', '.join(shown)}")

    # 3. The excess by cause.
    work_model = sums["model"] + sums["process"] + sums["tool"] + sums["event"]
    print("\n## The excess over the time model, by cause along the path (s)")
    print("| cause | s |")
    print("|---|---:|")
    print(f"| moves on the path slower than modelled (planned - model) | {sums['move'] - sums['model']:+.2f} |")
    print(f"| approach and retreat legs on the path (not timed by the model) | {sums['legs']:+.2f} |")
    print(f"| waits on the path | {sums['wait']:+.2f} |")
    print(f"| serialized work: the path's modelled moves, process and events less the model's bay "
          f"({work_model:.2f} - {model:.2f}) | {work_model - model:+.2f} |")
    print(f"| total (planned - model) | {planned - model:+.2f} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
