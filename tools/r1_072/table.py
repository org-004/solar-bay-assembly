"""Prints run 072's R2 table from the plan logs: table.py <out dir> <tag>..."""
import os, re, sys

def figs(path):
    if not os.path.exists(path):
        return None
    t = open(path).read()
    g = lambda k: (re.search(r"^%s: ([0-9.]+)" % k, t, re.M) or [None, None])[1]
    ok = re.search(r"^complete: true", t, re.M) is not None
    return {"cycle": g("cycle_s"), "interval": g("interval_s"), "one": g("one_bay_s"), "ok": ok}

out = sys.argv[1]
print("| candidate | s1 cycle / interval / one bay | s2 cycle / interval / one bay | mean cycle | verifies s1/s2 |")
print("|---|---|---|---|---|")
for tag in sys.argv[2:]:
    f1, f2 = figs(os.path.join(out, tag + ".log")), figs(os.path.join(out, tag + "_s2.log"))
    cell = lambda f: "-" if not f else "%s / %s / %s" % (f["cycle"], f["interval"], f["one"])
    mean = "%.2f" % ((float(f1["cycle"]) + float(f2["cycle"])) / 2) if f1 and f2 and f1["cycle"] and f2["cycle"] else "-"
    v = "/".join("-" if not f else ("yes" if f["ok"] else "no") for f in (f1, f2))
    print("| %s | %s | %s | %s | %s |" % (tag, cell(f1), cell(f2), mean, v))
