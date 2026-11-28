"""Writes run 073's landed layout and its committed plan from a candidate:
land.py <repo> <candidate layout file> <candidate plan file> <what> <seeds text>
The layout is the candidate with the name ..._pallet_near_borrowed (plan_seed
kept), the title and summary extended; the plan is the candidate's plan with
its layout name and layout_sha256 set to the landed file's (the plan's moves
and schedule are the candidate's own, and the verifier checks them whole)."""
import hashlib, json, os, sys

repo, lay, plan, what, seeds = sys.argv[1:6]
NAME = ("flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_"
        "infeed_rail_mag_early_next_fill_pallet_near_borrowed")
d = json.load(open(lay))
d["name"] = NAME
d["title"] = d["title"] + "; and pin arms that swage modules of another arm (run 073)."
d["summary"] = (d["summary"] + " Run 073: v24's layout with " + what + " (swage_after_pins, "
                "borrowed_last, borrowed_after; tools/r1_073/make.py on branch "
                "org/073-borrowed-after). " + seeds)
path = os.path.join(repo, "layouts", NAME + ".json")
with open(path, "w") as f:
    json.dump(d, f, indent=2)
    f.write("\n")
text = open(plan).read()
old = json.load(open(plan))
sha = hashlib.sha256(open(path, "rb").read()).hexdigest()
head = '"layout": "%s",\n"layout_sha256": "%s",' % (old["layout"], old["layout_sha256"])
assert head in text[:1000], "unexpected plan head"
text = text.replace(head, '"layout": "%s",\n"layout_sha256": "%s",' % (NAME, sha), 1)
with open(os.path.join(repo, "plans", NAME + ".json"), "w") as f:
    f.write(text)
