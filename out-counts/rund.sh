#!/bin/sh
cd /root/work/013-design-principles-and-r2/out/counts
for t in A1 A2 B; do
python3 runw.py $t set$t.txt gcc13 docker exec -w /root/work/013-design-principles-and-r2/repo -e "BAZEL=bazel --output_base=/root/.cache/bazel/ob-ubuntu" sba-ci
done
