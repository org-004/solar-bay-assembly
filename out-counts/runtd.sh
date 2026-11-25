#!/bin/sh
cd /root/work/013-design-principles-and-r2/out/counts
for p in P1 P2; do
python3 runt.py $p $p.txt tidy18_3 docker exec -w /root/work/013-design-principles-and-r2/repo -e CLANG_TIDY=clang-tidy-18 -e "BAZEL=bazel --output_base=/root/.cache/bazel/ob-ubuntu" sba-ci
done
