#!/bin/bash
# Screens each candidate (one bay, 120 s), then plans the passing ones three bays at seeds 1 and 2.
cd "${RUN_DIR:-/root/work/rf/_main}"
export LAYOUT_DIR=layouts MODELS_DIR=models VERSIONS_JSON=versions/versions.json UR10E_XML=../+menagerie_model+menagerie_ur10e/ur10e.xml
B=flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near
O=${OUT:-/root/work/out}
pass=()
for t in "$@"; do
  timeout 120 stdbuf -oL /root/bin/plan_layout ${B}_$t --summary --budgets 5000,40000,160000,640000 > $O/screen_$t.log 2>&1
  e=$?
  echo "screen $t exit=$e blocked=$(grep -c 'is blocked' $O/screen_$t.log)" >> $O/queue.done
  grep -q 'is blocked' $O/screen_$t.log || pass+=("$t")
done
for t in "${pass[@]}"; do
  for s in "" _s2; do
    st=$(date +%s)
    timeout 1800 stdbuf -oL /root/bin/plan_layout ${B}_$t$s --bays 3 --summary \
      --budgets 5000,40000,160000,640000 --out $O/${B}_$t$s.json > $O/$t$s.log 2>&1
    echo "$t$s exit=$? secs=$(( $(date +%s) - st ))" >> $O/queue.done
  done
done
echo ALLDONE >> $O/queue.done
