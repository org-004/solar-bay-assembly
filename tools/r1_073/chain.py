"""Prints each bay's critical chain owner, its waits over 1 s and the second chain's gap
from a plan_layout --margins file: chain.py <margins file>."""
import sys, csv
rows = [r for r in csv.reader(open(sys.argv[1])) if r and r[0] in ("critical", "critical_next")]
for bay in "012":
    ch = [r for r in rows if r[0] == "critical" and r[1] == bay]
    arms = []
    for r in ch:
        if r[5] not in ("events",) and (not arms or arms[-1] != r[5]): arms.append(r[5])
    waits = ["%s %s %.1fs %s" % (r[5][-1], r[4], float(r[9]), r[11]) for r in ch if float(r[9]) > 1]
    nxt = [r for r in rows if r[0] == "critical_next" and r[1] == bay]
    print("bay %s: %s | waits: %s | next: %s" % (bay, ">".join(a.replace("pin_arm_", "") for a in arms),
          "; ".join(waits) or "-", ", ".join("%s %s gap %s" % (r[2], r[4], r[7]) for r in nxt[:1])))
