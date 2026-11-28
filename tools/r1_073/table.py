"""Prints run 073's R2 table from the plan logs: table.py <out dir> <tag>...
Each tag's seed 1 is <tag>.log, seeds 2 and 3 <tag>_s2.log and <tag>_s3.log."""
import os, re, sys

def figs(path):
    if not os.path.exists(path):
        return None
    t = open(path).read()
    g = lambda k: (re.search(r"^%s: ([0-9.]+)" % k, t, re.M) or [None, None])[1]
    if g("cycle_s") is None:
        return None
    ok = re.search(r"^complete: true", t, re.M) is not None
    return {"cycle": g("cycle_s"), "interval": g("interval_s"), "one": g("one_bay_s"), "ok": ok}

out = sys.argv[1]
print("| candidate | s1 cycle / interval / one bay | s2 | s3 | mean (seeds) | verifies s1/s2/s3 |")
print("|---|---|---|---|---|---|")
for tag in sys.argv[2:]:
    fs = [figs(os.path.join(out, tag + s + ".log")) for s in ("", "_s2", "_s3")]
    cell = lambda f: "-" if not f else "%s / %s / %s" % (f["cycle"], f["interval"], f["one"])
    got = [float(f["cycle"]) for f in fs if f]
    mean = "%.2f (%d)" % (sum(got) / len(got), len(got)) if got else "-"
    v = "/".join("-" if not f else ("yes" if f["ok"] else "no") for f in fs)
    print("| %s | %s | %s | %s | %s | %s |" % (tag, cell(fs[0]), cell(fs[1]), cell(fs[2]), mean, v))
