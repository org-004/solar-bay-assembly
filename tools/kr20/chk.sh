#!/bin/bash
# chk.sh name -> /root/scratch/name.csv and summary
R=/root/work/031-kuka-kr-20-pin-arms-for-the-r1/solar-bay-assembly/bazel-bin
cd $R/check_layout.runfiles/_main && LAYOUT_DIR=/root/scratch $R/check_layout "$1" > /root/scratch/$1.csv 2>/root/scratch/$1.err
python3 /root/scripts/sumfail.py /root/scratch/$1.csv ${2:-6}
