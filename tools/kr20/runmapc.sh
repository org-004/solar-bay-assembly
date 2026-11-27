#!/bin/bash
tag=$1; shift
python3 /root/scripts/cand.py cm_$tag "$@" >/dev/null && python3 /root/scripts/mapvar.py /root/scratch/cm_$tag.json map_$tag
cd /root/work/031-kuka-kr-20-pin-arms-for-the-r1/solar-bay-assembly/bazel-bin/check_layout.runfiles/_main
LAYOUT_CHECK_PROBE=pin_arm_a LAYOUT_CHECK_PROBE_WIDE=0 LAYOUT_DIR=/root/scratch timeout 900 ../../check_layout map_$tag 2>/root/scratch/pm_$tag.txt >/root/scratch/map_$tag.csv
python3 /root/scripts/feas.py /root/scratch/pm_$tag.txt > /root/scratch/pm_$tag.feas
