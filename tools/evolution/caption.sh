#!/bin/bash
# caption.sh IN OUT LINE1 [LINE2 [LINE3]]
#
# Burns up to three lines of text into a white band added above the picture IN
# (a GIF or a PNG) and writes OUT (the same kind). The lines come from
# `tools/evolution/pages.py caption <id>`. Each line goes through a text file
# (drawtext textfile=), so no character needs escaping. Font sizes are 22 px
# for line 1 and 19 px for the others at 720 px width, scaled to the input's
# width. A GIF keeps its frame rate and loops forever, with at most 64 colours
# (32 on a second try). Prints the output's size in bytes; exits 1 when it is
# above 2,500,000 bytes. Temporary files go under ${V:-$HOME/video}.
set -e
if [ $# -lt 3 ] || [ $# -gt 5 ]; then
  echo "usage: $0 IN OUT LINE1 [LINE2 [LINE3]]" >&2
  exit 2
fi
in=$1; out=$2; shift 2
font=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
[ -f "$font" ] || { echo "caption.sh: font $font is missing (apt-get install fonts-dejavu-core)" >&2; exit 1; }
[ -f "$in" ] || { echo "caption.sh: $in does not exist" >&2; exit 1; }
V=${V:-$HOME/video}; mkdir -p "$V"
work=$(mktemp -d "$V/caption.XXXXXX")
trap 'rm -rf "$work"' EXIT
width=$(ffprobe -v error -select_streams v:0 -show_entries stream=width -of csv=p=0 "$in")
size1=$(awk -v w="$width" 'BEGIN { printf "%d", 22 * w / 720 + 0.5 }')
size2=$(awk -v w="$width" 'BEGIN { printf "%d", 19 * w / 720 + 0.5 }')
pad=$(awk -v w="$width" 'BEGIN { printf "%d", 8 * w / 720 + 0.5 }')
lines=$#
band=$(( pad * 2 + size1 + (lines - 1) * (size2 + pad / 2 + 2) + 2 ))
band=$(( band + band % 2 ))
vf="pad=iw:ih+$band:0:$band:white"
i=0
y=$pad
for line in "$@"; do
  i=$((i + 1))
  printf '%s' "$line" > "$work/line$i.txt"
  if [ $i -eq 1 ]; then s=$size1; else s=$size2; fi
  vf="$vf,drawtext=fontfile=$font:textfile=$work/line$i.txt:fontsize=$s:fontcolor=#222222:x=$pad:y=$y"
  y=$(( y + s + pad / 2 + 2 ))
done
mkdir -p "$(dirname "$out")"
case "$in" in
  *.gif|*.GIF)
    fps=$(ffprobe -v error -select_streams v:0 -show_entries stream=r_frame_rate -of csv=p=0 "$in")
    for colors in 64 32; do
      ffmpeg -v error -y -i "$in" -vf "$vf,fps=$fps,split[a][b];[a]palettegen=max_colors=$colors:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" \
        -loop 0 "$out"
      bytes=$(stat -c %s "$out")
      [ "$bytes" -le 2500000 ] && break
    done ;;
  *.png|*.PNG)
    ffmpeg -v error -y -i "$in" -vf "$vf" -frames:v 1 -update 1 "$out"
    bytes=$(stat -c %s "$out") ;;
  *) echo "caption.sh: $in is neither a GIF nor a PNG" >&2; exit 1 ;;
esac
echo "$bytes bytes"
if [ "$bytes" -gt 2500000 ]; then
  echo "caption.sh: $out is $bytes bytes, above 2,500,000" >&2
  exit 1
fi
