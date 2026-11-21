#!/bin/bash
# gif.sh RAW WxH FPS OUT.gif COLORS [PRE_FILTERS]   raw RGB24 frames -> 720 px GIF at 10 fps
# PRE_FILTERS (optional) run first, e.g. "trim=start=3:end=20,setpts=PTS-STARTPTS"
set -e
raw=$1; size=$2; fps=$3; out=$4; colors=$5; pre=${6:-null}
ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -video_size $size -framerate $fps -i $raw \
  -vf "$pre,fps=10,scale=720:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=$colors:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
  -loop 0 $out
ffprobe -v error -show_entries format=duration,size:stream=width,height,nb_frames -of compact $out
