#!/bin/bash
# plan1.sh <tag> : three-bay plan of candidate <tag> into /root/out
cd /root/rf/_main
export LAYOUT_DIR=layouts MODELS_DIR=models VERSIONS_JSON=versions/versions.json UR10E_XML=../+menagerie_model+menagerie_ur10e/ur10e.xml
B=flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near
O=/root/out
for t in "$@"; do
st=$(date +%s)
timeout 1800 stdbuf -oL taskset -c 3 ${PLAN:-/root/bin2/plan_layout} ${B}_$t --bays 3 --summary --budgets 5000,40000,160000,640000 --out $O/${B}_$t.json > $O/$t.log 2>&1
echo "$t exit=$? secs=$(( $(date +%s) - st ))" >> $O/queue.done
done
