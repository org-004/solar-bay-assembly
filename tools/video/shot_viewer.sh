#!/bin/bash
# shot_viewer.sh TARGET OUT.png SIZE(WxH) [viewer options...]
#
# A PNG of any target built from src/viewer.cc (//:viewer, //:viewer_bay,
# //:viewer_container, ...), the sibling of shot.sh, which does the same for
# //:view_cell. It runs the target with --record to a raw file at one frame a
# second, and converts the last frame with ffmpeg. The viewer options follow
# SIZE and are those of src/viewer.cc; end them with --quit-at so the run
# stops (--quit-at 0 gives one frame of the model as loaded), for example
#
#   tools/video/shot_viewer.sh //:viewer_bay docs/img/bay.png 1200x900 \
#       --lookat 0,0,1.2 --distance 9 --azimuth 140 --elevation -25 \
#       --no-overlay --quit-at 0
#
# V is the directory for the raw file and the log (default $HOME/video; never
# a memory-backed /tmp). REC_DISPLAY is the X display (default :7, an Xvfb
# started with: Xvfb :7 -screen 0 1920x1200x24 +extension GLX &). Prints the
# output path and the number of frames recorded.
set -e
if [ $# -lt 3 ]; then
  echo "usage: $0 TARGET OUT.png WxH [viewer options...]" >&2
  exit 2
fi
V=${V:-$HOME/video}; mkdir -p "$V"
target=$1; out=$(realpath -m "$2"); size=$3; shift 3
raw=$V/shot_$$.raw
cd "$(dirname "$0")/../.."
export PATH=$HOME/.local/bin:$PATH
DISPLAY=${REC_DISPLAY:-:7} timeout 900 bazel run --config=opt --config=agent "$target" -- --size $size --record $raw --fps 1 "$@" > "$V/shot_$$.log" 2>&1 || { tail -5 "$V/shot_$$.log"; rm -f $raw; exit 1; }
w=${size%x*}; h=${size#*x}
n=$(( $(stat -c %s $raw) / (w*h*3) ))
mkdir -p "$(dirname "$out")"
ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -video_size $size -i $raw -vf "select=eq(n\,$((n-1)))" -frames:v 1 "$out"
rm -f $raw
echo "$out frames=$n"
