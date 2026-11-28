#!/bin/bash
# Prints each bay's critical chain owner, its wait links and the second chain's gap
# for a planned candidate: chain.sh <run dir> <tag>.
cd "$1"
B=flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near
cp out/${B}_$2.json plans/
timeout 900 /root/bin/plan_layout ${B}_$2 --margins > out/$2.margins 2>&1; echo "margins exit=$?"
python3 - out/$2.margins <<'PY'
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
PY
