#!/bin/bash
# probe.sh src.json tag [shift lo hi]
src=$1; tag=$2; S=${3:-0}; lo=${4:--5.3}; hi=${5:-5.3}
python3 /root/scripts/mapvar.py $src map_$tag $lo $hi
cd /root/work/034-a-kr-20-40-ft-layout-that-r1/repo/bazel-bin/check_layout.runfiles/_main
LAYOUT_CHECK_PROBE=pin_arm_a LAYOUT_CHECK_PROBE_WIDE=0 LAYOUT_DIR=/root/scratch timeout 900 ../../check_layout map_$tag 2>/root/scratch/pm_$tag.txt >/root/scratch/map_$tag.csv
python3 /root/scripts/feas.py /root/scratch/pm_$tag.txt > /root/scratch/pm_$tag.feas
python3 /root/scripts/sides.py /root/scratch/pm_$tag.feas $S > /root/scratch/pm_$tag.sides
tail -1 /root/scratch/pm_$tag.sides
