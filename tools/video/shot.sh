#!/bin/bash
# shot.sh OUT.png SIZE(WxH) viewer-args...   records the last frame of a run that ends by --quit-at
set -e
V=${V:-/tmp/m2-video}; mkdir -p $V
out=$(realpath -m "$1"); size=$2; shift 2
raw=$V/shot_$$.raw
cd "$(dirname "$0")/../.."
DISPLAY=${REC_DISPLAY:-:7} timeout 900 bazel run --config=opt //:view_cell -- --size $size --record $raw --fps 1 "$@" > $V/shot_$$.log 2>&1 || { tail -5 $V/shot_$$.log; exit 1; }
w=${size%x*}; h=${size#*x}
n=$(( $(stat -c %s $raw) / (w*h*3) ))
ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -video_size $size -i $raw -vf "select=eq(n\,$((n-1)))" -frames:v 1 $out
rm -f $raw
echo "$out frames=$n"
