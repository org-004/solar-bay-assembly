#!/bin/sh
cd /root/work/013-design-principles-and-r2/out/counts
python3 runt.py P1 P1.txt tidy18_8 > tidy18_8/runt.log 2>&1
python3 runt.py P2 P2.txt tidy18_8 >> tidy18_8/runt.log 2>&1
mkdir -p tidy18_3
./runtd.sh > tidy18_3/runt.log 2>&1
