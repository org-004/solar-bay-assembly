#!/bin/bash
# Prints each bay's critical chain owner, its wait links and the second chain's gap
# for a planned candidate: chain.sh <run dir> <tag>.
SELF=$(cd "$(dirname "$0")" && pwd)
cd "$1"; O=${OUT:-/root/out}
B=flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_rail_mag_early_next_fill_pallet_near
cp $O/${B}_$2.json plans/
timeout 900 /root/bin2/plan_layout ${B}_$2 --margins > $O/$2.margins 2>&1; echo "margins exit=$?"
python3 "$SELF/chain.py" $O/$2.margins
