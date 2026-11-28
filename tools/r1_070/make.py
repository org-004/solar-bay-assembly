"""Builds run 070's candidates from _pallet_near (v24), each with swage_after_pins.

A candidate is a list of moves (hole, arm): that hole's swage goes to that pin
arm, which gets the giving arm's track stop for it (all three pin arms share
one beam, base and orientation, so the same stop gives the same reach). With
swage_after_pins a module's swages wait for its last pin, not for each other,
so two arms may swage one module at once.
Usage: make.py <layouts dir> [tag...]; writes <base>_<tag>[_s2|_s3].json.
"""
import json, sys, os

BASE = ("flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_"
        "torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near")
A, B, C = "pin_arm_a", "pin_arm_b", "pin_arm_c"
HOLE = ("locate", "pin", "swage")
CANDS = {
    "ctl": [],                                        # the option only
    "a30_b61": [(12, A), (24, B), (25, B)],           # A swages 3/0; B 6/0, 6/1
    "a31_b61": [(12, A), (13, A), (24, B), (25, B)],  # A swages 3/0, 3/1; B 6/0, 6/1
    "a31": [(12, A), (13, A)],                        # A swages 3/0, 3/1
    "a30_b60": [(12, A), (24, B)],                    # run 067's sw30_sw60
    "c52": [(22, C)],                                 # C swages 5/2 (5/3 fails: C carriage into B carriage)
    "a31_c52": [(12, A), (13, A), (22, C)],           # A 3/0, 3/1; C 5/2
}

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
    for h, arm in CANDS[tag]:
        arms[arm]["stops"]["swage"][h] = arms[owner[("swage", h)]]["stops"]["swage"][h]
        owner[("swage", h)] = arm
    for w in d["work"]:
        if w["kind"] in HOLE:
            w["points"] = text([h for (k, h), a in owner.items()
                                if k == w["kind"] and a == w["arm"]])
    d["name"] = BASE + "_" + tag + ("_s%d" % seed if seed > 1 else "")
    d["swage_after_pins"] = True
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
