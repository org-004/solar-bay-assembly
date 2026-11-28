#!/bin/bash
# Waits for the running plan, screens each candidate one bay (120 s), plans the passing ones three bays at seed 1.
cd /root/rf/_main
export LAYOUT_DIR=layouts MODELS_DIR=models VERSIONS_JSON=versions/versions.json UR10E_XML=../+menagerie_model+menagerie_ur10e/ur10e.xml
B=flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near
O=/root/out
while ! grep -q '^a31_b61_s3 ' $O/queue.done 2>/dev/null; do sleep 10; done
pass=()
for t in "$@"; do
  timeout 120 stdbuf -oL taskset -c 3 /root/bin2/plan_layout ${B}_$t --summary --budgets 5000,40000,160000,640000 > $O/screen_$t.log 2>&1
  e=$?
  echo "screen $t exit=$e blocked=$(grep -c 'is blocked' $O/screen_$t.log)" >> $O/queue.done
  grep -q 'is blocked' $O/screen_$t.log || pass+=("$t")
done
for t in "${pass[@]}"; do
  st=$(date +%s)
  timeout 1800 stdbuf -oL taskset -c 3 /root/bin2/plan_layout ${B}_$t --bays 3 --summary --budgets 5000,40000,160000,640000 --out $O/${B}_$t.json > $O/$t.log 2>&1
  echo "$t exit=$? secs=$(( $(date +%s) - st ))" >> $O/queue.done
done
echo ALLDONE >> $O/queue.done
