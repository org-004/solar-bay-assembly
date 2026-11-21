#!/bin/bash
# Remakes docs/*.gif and docs/img/{cell,tube_full,check_pose}.png. Needs a display in REC_DISPLAY (default :7).
# rec.sh NAME ARGS records raw RGB24 frames to $V/NAME.raw via `bazel run //:view_cell -- --record ...`;
# gif.sh RAW WxH FPS OUT COLORS PREFILTER -> 720 px, 10 fps, 64-colour GIF; shot.sh OUT WxH ARGS -> last frame as PNG.
set -e
V=${V:-/tmp/m2-video}; mkdir -p $V; D=$(cd "$(dirname "$0")/../.." && pwd)/docs; B=$(cd "$(dirname "$0")" && pwd); cd $V
$B/rec.sh cycle --seed 3 --check --calibration 1,0.05 --view cycle --speed 2 --fps 10 --quit-after-done 1.5
$B/gif.sh cycle.raw 1200x900 10 $D/cycle.gif 64 "tpad=stop_mode=clone:stop_duration=1.5"
$B/rec.sh pallet --seed 3 --modules 10 --check --view pallet --overlay short --font-scale 250 --size 1280x720 --speed 7.5 --fps 10 --quit-at 259
$B/gif.sh pallet.raw 1280x720 10 $D/pallet.gif 64 "trim=end_frame=341,tpad=stop_mode=clone:stop_duration=2"   # frame 341 = tube swap
$B/rec.sh pick --seed 11 --check --view pick --overlay short --font-scale 250 --caption "pallet off by 2.9 deg, 32 mm" --speed 1.5 --fps 10 --quit-at 20
$B/gif.sh pick.raw 1200x900 10 $D/pick.gif 64 "trim=start_frame=52:end_frame=130,setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration=1"
$B/rec.sh check --seed 3 --modules 2 --check --view check --overlay short --font-scale 250 --speed 1.5 --fps 10 --quit-at 57
$B/gif.sh check.raw 1200x900 10 $D/check.gif 64 "trim=start_frame=286:end_frame=374,setpts=PTS-STARTPTS,tpad=stop_mode=clone:stop_duration=1"
S="--lookat 1.65,0,1.0 --distance 3.3 --azimuth 20 --elevation -60 --size 640x600 --no-overlay --font-scale 250 --speed 1.5 --fps 10"
$B/rec.sh bvc_blind --seed 11 --check --blind $S --caption $'blind (no camera)\nstops: tool on the glass 9 mm early' --quit-at 19
$B/rec.sh bvc_cam --seed 11 --check $S --caption $'camera\ntool turned to the glass, picks' --quit-at 21
ffmpeg -loglevel error -y -f rawvideo -pix_fmt rgb24 -video_size 640x600 -framerate 10 -i bvc_blind.raw -f rawvideo -pix_fmt rgb24 -video_size 640x600 -framerate 10 -i bvc_cam.raw -filter_complex "[0]trim=start_frame=18:end_frame=99,setpts=PTS-STARTPTS[a];[1]trim=start_frame=45:end_frame=126,setpts=PTS-STARTPTS[b];[a][b]hstack,tpad=stop_mode=clone:stop_duration=1.5,scale=720:-1:flags=lanczos,split[x][y];[x]palettegen=max_colors=64:stats_mode=diff[p];[y][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" -loop 0 $D/blind_vs_camera.gif
$B/shot.sh $D/img/cell.png 1200x900 --seed 3 --no-overlay --view overview --quit-at 0
$B/shot.sh $D/img/tube_full.png 1200x900 --seed 3 --modules 10 --check --no-overlay --lookat -1.5,-4.5,0.6 --distance 12 --azimuth 50 --elevation -35 --quit-at 255.6
$B/shot.sh $D/img/check_pose.png 1200x900 --seed 3 --check --no-overlay --lookat 0.1,1.3,1.0 --distance 5.2 --azimuth 235 --elevation -24 --quit-at 21
rm -f $V/*.raw
