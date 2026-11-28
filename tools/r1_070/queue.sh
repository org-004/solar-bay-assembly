#!/bin/bash
# Plans run 070's candidates three bays, one at a time, from copied binaries.
# Usage: queue.sh <run dir with models/ layouts/ plans/ versions/> <tag>...
# (a tag may end in _s2 or _s3 for its plan_seed copy).
cd "$1"; shift
B=flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near
for t in "$@"; do
  s=$(date +%s)
  timeout 1800 stdbuf -oL /root/bin/plan_layout ${B}_$t --bays 3 --summary \
    --budgets 5000,40000,160000,640000 --out out/${B}_$t.json > out/$t.log 2>&1
  echo "$t exit=$? secs=$(( $(date +%s) - s ))" >> out/queue.done
done
