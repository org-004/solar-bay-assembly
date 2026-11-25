#!/bin/sh
# Compile every own target in the container, without MuJoCo, under the lock.
cd /root/work/013-design-principles-and-r2/repo
T=$(docker exec -w /root/work/013-design-principles-and-r2/repo sba-ci bazel --output_base=/root/.cache/bazel/ob-ubuntu query 'kind("cc_.*", //...)' 2>/dev/null | tr '\n' ' ')
S=$(date +%s)
flock /root/work/013-design-principles-and-r2/heavy.lock docker exec -w /root/work/013-design-principles-and-r2/repo sba-ci bazel --output_base=/root/.cache/bazel/ob-ubuntu build --config=opt --symlink_prefix=/ --jobs=3 --local_resources=cpu=3 --output_groups=compilation_outputs $T
echo rc=$?
