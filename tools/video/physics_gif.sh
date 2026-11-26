#!/bin/bash
# physics_gif.sh NAME OUT.gif [SIZE(WxH) SPEED COLORS] [viewer camera options...]
#
# A GIF of one module's take and set of the committed plan of a layout, run in
# physics: //:viewer_layout NAME --physics (src/view_layout.cc) with
# --no-overlay, recorded to a raw file at 10 frames a second of viewer time,
# each frame SPEED / 10 seconds of physics. The run is about 25 s of physics
# (1 s hold, the plan's take and set step, 2 s of settling) and the recording
# ends with it. tools/video/gif.sh turns the raw file into a GIF 720 px wide
# with at most COLORS colours. The raw file is removed. Prints the GIF's
# duration and size, and its size in bytes.
#
# The defaults (960x600, speed 2, 64 colours, and a camera on the module arm
# of flat_rack_3_pin_arms_ordered) give a GIF of about 12 s; with any
# argument after OUT.gif, SIZE SPEED COLORS must all be given, and the camera
# options that follow them replace the default camera. For example
#
#   tools/video/physics_gif.sh flat_rack_3_pin_arms_ordered $HOME/video/v10-physics.gif
#   tools/evolution/caption.sh $HOME/video/v10-physics.gif docs/evolution/v10-physics.gif \
#       LINE1 LINE2 LINE3
#
# V is the directory for the raw file and the log (default $HOME/video; never
# a memory-backed /tmp). REC_DISPLAY is the X display (default :7, an Xvfb
# started with: Xvfb :7 -screen 0 1920x1200x24 +extension GLX &).
set -e
if [ $# -lt 2 ] || [ $# -eq 3 ] || [ $# -eq 4 ]; then
  echo "usage: $0 NAME OUT.gif [WxH SPEED COLORS [viewer camera options...]]" >&2
  exit 2
fi
V=${V:-$HOME/video}; mkdir -p "$V"
name=$1; out=$(realpath -m "$2"); shift 2
if [ $# -ge 3 ]; then
  size=$1; speed=$2; colors=$3; shift 3
else
  size=960x600; speed=2; colors=64
  set -- --lookat 0.7,0.5,1.6 --distance 7 --azimuth 215 --elevation -20
fi
raw=$V/physics_$$.raw
cd "$(dirname "$0")/../.."
export PATH=$HOME/.local/bin:$PATH
DISPLAY=${REC_DISPLAY:-:7} timeout 3000 bazel run --config=opt --config=agent //:viewer_layout -- \
  "$name" --physics --speed "$speed" --no-overlay \
  --size $size --record $raw --fps 10 --quit-at 60 "$@" > "$V/physics_$$.log" 2>&1 \
  || { tail -5 "$V/physics_$$.log"; rm -f $raw; exit 1; }
mkdir -p "$(dirname "$out")"
tools/video/gif.sh $raw $size 10 "$out" $colors
rm -f $raw
stat -c "%s bytes" "$out"
