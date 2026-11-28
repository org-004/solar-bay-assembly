"""Builds run 067's hole-reassignment candidates from _pallet_near.

A candidate is a list of moves (kind, hole, arm): that step of that hole goes
to that pin arm, which gets the giving arm's track stop for it (all three pin
arms share one beam, base and orientation, so the same stop gives the same
reach), or a stop override (kind, hole, arm, stop) changes only a stop.
The planner serializes a module's hole steps across arms in the order
locate/pin 0,2,3,1 then swage 1,3,2,0, and each arm works its modules in set
order, so only a module's last steps move without making an arm wait.
Usage: make.py <layouts dir> [tag...]; writes <base>_<tag>[_s2|_s3].json.
"""
import json, sys, os

BASE = ("flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_"
        "torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near")
A, B, C = "pin_arm_a", "pin_arm_b", "pin_arm_c"
HOLE = ("locate", "pin", "swage")
CANDS = {
    # First round (whole holes; all three deadlock in bay 0, see the report).
    "holes": [(k, h, A) for k in HOLE for h in (12, 13)] +
             [(k, h, B) for k in HOLE for h in (24, 25)],
    "holes_k2": [(k, h, A) for k in HOLE for h in (12, 13)] +
                [(k, 24, B) for k in HOLE],
    "holes_k3": [(k, 12, A) for k in HOLE] + [(k, 24, B) for k in HOLE],
    # Second round: last steps only.
    "sw30": [("swage", 12, A)],                      # A swages 3/0
    "sw30_sw60": [("swage", 12, A), ("swage", 24, B)],  # + B swages 6/0
    "sw30_sw32": [("swage", 12, A), ("swage", 14, A)],  # A swages 3/2 and 3/0
}
# tag: [(kind, hole, arm, stop)]; B's swage 3/0 from -2.0 to -1.6 m, 0.4 m
# farther from A at holes 1/2 and 1/3 (-3.1, -3.5), where it waited 18 s.
STOPS = {"st30": [("swage", 12, B, -1.6)]}

def span(s):
    out = []
    for part in s.split(","):
        lo, _, hi = part.partition("-")
        out += list(range(int(lo), int(hi or lo) + 1))
    return out

def text(holes):
    holes = sorted(holes)
    runs, i = [], 0
    while i < len(holes):
        j = i
        while j + 1 < len(holes) and holes[j + 1] == holes[j] + 1:
            j += 1
        runs.append(str(holes[i]) if i == j else "%d-%d" % (holes[i], holes[j]))
        i = j + 1
    return ",".join(runs)

def make(d, tag, seed):
    arms = {a["name"]: a for a in d["arms"]}
    owner = {}
    for w in d["work"]:
        if w["kind"] in HOLE:
            for h in span(w["points"]):
                owner[(w["kind"], h)] = w["arm"]
    for kind, h, arm in CANDS.get(tag, []):
        arms[arm]["stops"][kind][h] = arms[owner[(kind, h)]]["stops"][kind][h]
        owner[(kind, h)] = arm
    for kind, h, arm, stop in STOPS.get(tag, []):
        arms[arm]["stops"][kind][h] = stop
    for w in d["work"]:
        if w["kind"] in HOLE:
            w["points"] = text([h for (k, h), a in owner.items()
                                if k == w["kind"] and a == w["arm"]])
    d["name"] = BASE + "_" + tag + ("_s%d" % seed if seed > 1 else "")
    if seed > 1:
        d["plan_seed"] = seed
    return d

if __name__ == "__main__":
    out = sys.argv[1]
    for tag in sys.argv[2:] or list(CANDS):
        for seed in (1, 2, 3):
            d = json.load(open(os.path.join(out, BASE + ".json")))
            d = make(d, tag, seed)
            with open(os.path.join(out, d["name"] + ".json"), "w") as f:
                json.dump(d, f, indent=2)
                f.write("\n")
