#!/bin/bash
# play_gif.sh NAME OUT.gif SIZE(WxH) SPEED FROM TO COLORS [viewer camera options...]
#
# A GIF of the committed plan of a layout, from schedule time FROM to TO (in
# seconds), played SPEED schedule seconds per second of the GIF. It runs
# //:viewer_layout NAME --play with --no-overlay and the camera options that
# follow COLORS (--lookat, --distance, --azimuth, --elevation), records the
# window at SIZE to a raw file at 10 frames a second of viewer time, and
# stops after (TO - FROM) / SPEED seconds. tools/video/gif.sh turns the raw
# file into a GIF 720 px wide with at most COLORS colours (for example 64).
# The raw file is removed. Prints the GIF's duration and size, and its size
# in bytes, for example
#
#   tools/video/play_gif.sh flat_rack_3_pin_arms docs/img/bay.gif 960x600 8 0 644.28 64 \
#       --lookat 0,0,1.3 --distance 13 --azimuth 215 --elevation -22
#
# V is the directory for the raw file and the log (default $HOME/video; never
# a memory-backed /tmp). REC_DISPLAY is the X display (default :7, an Xvfb
# started with: Xvfb :7 -screen 0 1920x1200x24 +extension GLX &).
set -e
if [ $# -lt 7 ]; then
  echo "usage: $0 NAME OUT.gif WxH SPEED FROM TO COLORS [viewer camera options...]" >&2
  exit 2
fi
V=${V:-$HOME/video}; mkdir -p "$V"
name=$1; out=$(realpath -m "$2"); size=$3; speed=$4; from=$5; to=$6; colors=$7; shift 7
raw=$V/play_$$.raw
cd "$(dirname "$0")/../.."
export PATH=$HOME/.local/bin:$PATH
quit=$(awk -v f="$from" -v t="$to" -v s="$speed" 'BEGIN { printf "%.4f", (t - f) / s }')
DISPLAY=${REC_DISPLAY:-:7} timeout 3000 bazel run --config=opt --config=agent //:viewer_layout -- \
  "$name" --play --speed "$speed" --from-time "$from" --to-time "$to" --no-overlay \
  --size $size --record $raw --fps 10 --quit-at "$quit" "$@" > "$V/play_$$.log" 2>&1 \
  || { tail -5 "$V/play_$$.log"; rm -f $raw; exit 1; }
mkdir -p "$(dirname "$out")"
tools/video/gif.sh $raw $size 10 "$out" $colors
rm -f $raw
stat -c "%s bytes" "$out"
