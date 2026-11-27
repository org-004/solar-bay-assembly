#!/bin/bash
# dg.sh map_variant.json tag track : check pin_arm_a's locate 0/0 and 0/1 at one track
python3 - "$1" "$2" "$3" <<'P'
import json, sys
d = json.load(open(sys.argv[1])); d['name'] = 'dg_' + sys.argv[2]; a = d['arms'][1]; t = float(sys.argv[3])
for k in ('locate', 'pin', 'swage'): a['stops'][k] = [t] * 36
json.dump(d, open('/root/scratch/dg_' + sys.argv[2] + '.json', 'w'), indent=1)
P
cd /root/work/034-a-kr-20-40-ft-layout-that-r1/repo/bazel-bin/check_layout.runfiles/_main
LAYOUT_DIR=/root/scratch timeout 600 ../../check_layout dg_$2 2>/dev/null | grep '^clearance,locate 0/0\|^approach,locate 0/0\|^hold\|^ways' | cut -c1-200
