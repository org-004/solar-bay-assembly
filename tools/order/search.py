#!/usr/bin/env python3
"""Tries orders of work for a layout on the flat rack by planning them.

A variant is the base layout with an "order" field (src/layout.h, WorkOrder)
and, optionally, the hole and strap-bolt points divided among the pin arms
in another way. Each variant is written as DIR/<name>.json and planned with
the built //:plan_layout (bazel build --config=opt //:plan_layout first),
several at a time, outside Bazel; the table gives each one's planned bay,
whether the plan is complete (every move found, the verifier content), the
least clearance over the schedule and the waits.

Usage:
  search.py try BASE --out DIR [--jobs N] [--threads N] SPEC...
      SPEC is name:modules:start:tools[:holes[:bolts]], where
        modules  the bay's modules in the order they are set, as "0-8" or
                 "0,4,7,1,5,8,2,6,3";
        start    "all" (after_all_bolts) or "rails" (after_their_rails);
        tools    "jit" (just_in_time) or "early";
        holes    the modules of each pin arm, as "0-3/4-6/7-8" (a, b, c);
        bolts    the strap bolts of each pin arm, as "0-4/5-7/8-9".
  search.py orders --arms 0-3/4-6/7-8 [--top N]
      ranks module orders by a rough schedule (see Rough below) and prints
      the best as "modules" strings for try.

Python 3 standard library only; run it from the repository's root with -I.
"""
import argparse
import concurrent.futures
import itertools
import json
import os
import re
import subprocess
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
RUNFILES = os.path.join(ROOT, "bazel-bin", "plan_layout.runfiles")
PIN_ARMS = ["pin_arm_a", "pin_arm_b", "pin_arm_c"]


def numbers(text):
    """"0-3" or "0,4,7" as a list of ints, in the order given."""
    out = []
    for part in text.split(","):
        if "-" in part:
            a, b = part.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return out


def variant(base, name, modules, start, tools, holes=None, bolts=None):
    layout = json.loads(json.dumps(base))
    layout["name"] = name
    layout["order"] = {
        "modules": modules,
        "modules_start": {"all": "after_all_bolts", "rails": "after_their_rails"}[start],
        "pin_tools": {"jit": "just_in_time", "early": "early"}[tools],
        "note": "Tried by tools/order/search.py.",
        "basis": "ours",
    }
    if holes:
        mods = [numbers(x) for x in holes.split("/")]
        for w in layout["work"]:
            if w["kind"] in ("locate", "pin", "swage"):
                m = mods[PIN_ARMS.index(w["arm"])]
                w["points"] = "%d-%d" % (4 * min(m), 4 * max(m) + 3)
    if bolts:
        rails = [numbers(x) for x in bolts.split("/")]
        for w in layout["work"]:
            if w["kind"] == "strap_bolt":
                r = rails[PIN_ARMS.index(w["arm"])]
                w["points"] = "%d-%d" % (min(r), max(r))
    return layout


def plan(directory, name, threads):
    main = os.path.join(RUNFILES, "_main")
    env = dict(os.environ, LAYOUT_DIR=directory, MODELS_DIR="models",
               UR10E_XML="../+menagerie_model+menagerie_ur10e/ur10e.xml",
               VERSIONS_JSON="versions/versions.json")
    p = subprocess.run([os.path.join(main, "plan_layout"), name, "--summary", "--threads", str(threads),
                        "--out", os.path.join(directory, name + ".plan.json")],
                       cwd=main, env=env, capture_output=True, text=True)
    row = {"name": name, "exit": p.returncode, "faults": 0}
    for line in p.stdout.splitlines():
        m = re.match(r"(\w+): (.*)", line)
        if not m:
            continue
        k, v = m.groups()
        if k == "fault":
            row["faults"] += 1
            row.setdefault("first_fault", v)
        else:
            row[k] = v
    if p.returncode == 2:
        row["first_fault"] = p.stderr.strip().splitlines()[-1] if p.stderr.strip() else "usage"
    return row


def do_try(args):
    base = json.load(open(args.base))
    os.makedirs(args.out, exist_ok=True)
    names = []
    for spec in args.spec:
        parts = spec.split(":")
        name, modules, start, tools = parts[:4]
        holes = parts[4] if len(parts) > 4 and parts[4] else None
        bolts = parts[5] if len(parts) > 5 and parts[5] else None
        v = variant(base, name, numbers(modules), start, tools, holes, bolts)
        with open(os.path.join(args.out, name + ".json"), "w") as f:
            json.dump(v, f, indent=2)
            f.write("\n")
        names.append(name)
    with concurrent.futures.ThreadPoolExecutor(args.jobs) as ex:
        rows = list(ex.map(lambda n: plan(os.path.abspath(args.out), n, args.threads), names))
    print("name,bay_planned_s,complete,least_clearance_m,least_arm_to_arm_m,waits,wait_s,faults,first_fault")
    for r in sorted(rows, key=lambda r: float(r.get("bay_planned_s", 1e9))):
        print(",".join(str(r.get(k, "")) for k in
                       ("name", "bay_planned_s", "complete", "least_clearance_m", "least_arm_to_arm_m",
                        "waits", "wait_s", "faults")) + ',"%s"' % r.get("first_fault", "")[:160])


# Rough: a schedule of the module phase alone, from figures read off the
# committed plan of flat_rack_3_pin_arms (in seconds): the module arm's take
# and set of one module, one pin arm's four holes, its tool change and magazine
# before its first hole, and the end. It knows no collisions and no waits for
# another arm; it only ranks orders, and the planner decides.
TAKE_SET = 24.0       # The module arm, per module.
FIRST_SET_END = 251.7  # The first set's end with today's start.
HOLES = 63.0          # A pin arm, per module.
FIRST_LAG = 17.0      # From a set to the start of its arm's first hole.
NEXT_LAG = 2.0        # From a set to the start of a later hole of an idle arm.
END = 13.8 + 5.0 + 40.0


def rough(order, owner):
    free = {a: None for a in set(owner.values())}
    t_set = FIRST_SET_END - TAKE_SET
    end = 0.0
    for m in order:
        t_set += TAKE_SET
        a = owner[m]
        start = t_set + (FIRST_LAG if free[a] is None else NEXT_LAG)
        if free[a] is not None:
            start = max(start, free[a])
        free[a] = start + HOLES
        end = max(end, free[a])
    return end + END


def do_orders(args):
    mods = [numbers(x) for x in args.arms.split("/")]
    owner = {m: a for a, ms in enumerate(mods) for m in ms}
    best = sorted((rough(o, owner), o) for o in itertools.permutations(range(9)))
    for s, o in best[:args.top]:
        print("%.1f %s" % (s, ",".join(map(str, o))))


def main(argv):
    p = argparse.ArgumentParser()
    sub = p.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("try")
    t.add_argument("base")
    t.add_argument("--out", required=True)
    t.add_argument("--jobs", type=int, default=3)
    t.add_argument("--threads", type=int, default=2)
    t.add_argument("spec", nargs="+")
    o = sub.add_parser("orders")
    o.add_argument("--arms", default="0-3/4-6/7-8")
    o.add_argument("--top", type=int, default=10)
    args = p.parse_args(argv)
    if args.cmd == "try":
        do_try(args)
    else:
        do_orders(args)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
