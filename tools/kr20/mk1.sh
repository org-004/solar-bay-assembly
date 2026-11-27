#!/bin/bash
# mk1.sh name [gen args...] : the one-arm layout from 031's candidate
L=/root/work/034-a-kr-20-40-ft-layout-that-r1/repo/layouts/flat_rack_3_kr20_pin_arms_ordered.json
n=$1; shift
cd /root/scratch && python3 /root/scripts/var.py $L v_shift1 shift=1.0 mhi=5.5 && python3 /root/scripts/endbar.py v_shift1.json v_cell 0.7 -1.185 0.06 && python3 /root/scripts/gen.py v_cell.json pm_cell.feas $n split=1 s1=-5.7 "$@" && python3 - $n <<'P'
import json, sys
f = '/root/scratch/%s.json' % sys.argv[1]; d = json.load(open(f))
d['stocks'] = [s for s in d['stocks'] if s['name'] not in ('pins_1', 'pins_2')]
for it in d['items']:
    if it['name'] == 'pin_lane': it['boxes'] = [b for b in it['boxes'] if b['name'] not in ('nest_1', 'nest_2')]
json.dump(d, open(f, 'w'), indent=1)
P
[ -n "$POST" ] && eval "$POST"
cd /root/work/034-a-kr-20-40-ft-layout-that-r1/repo/bazel-bin/check_layout.runfiles/_main && LAYOUT_DIR=/root/scratch timeout 900 ../../check_layout $n > /root/scratch/$n.csv 2>/root/scratch/$n.err; python3 /root/scripts/sumfail.py /root/scratch/$n.csv 10; grep '^time,bay' /root/scratch/$n.csv | cut -c1-80
cd /root/work/034-a-kr-20-40-ft-layout-that-r1/repo/bazel-bin/plan_layout.runfiles/_main && LAYOUT_DIR=/root/scratch timeout 1200 ../../plan_layout $n --bays 1 --summary --out /root/scratch/plan_$n.json > /root/scratch/plan_$n.log 2>&1; grep 'moves\|bay_planned_s\|complete\|fault\|least\|wait_s' /root/scratch/plan_$n.log
