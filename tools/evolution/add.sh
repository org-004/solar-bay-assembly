#!/bin/bash
# add.sh NAME "TITLE" "CHANGED" "WHY" [--status chosen|candidate] [--commit HASH|this] [--competed-with vNN]
#
# Registers a layout that has a committed plan (plans/NAME.json) as the next
# version in versions/versions.json:
#   1. writes scores/NAME.json (bazel run //:score_layout -- NAME --write);
#   2. records the whole bay of the plan as a raw GIF under ${V:-$HOME/video}
#      (tools/video/play_gif.sh; starts Xvfb :7 if no display answers there and
#      stops it at the end);
#   3. appends the version with tools/evolution/pages.py add (the direction
#      comes from the layout; a chosen version supersedes the direction's
#      previous chosen one);
#   4. burns the caption of pages.py caption into docs/evolution/<id>.gif;
#   5. runs pages.py write (the direction pages, the charts, the README results).
# Defaults: --status chosen, --commit this. Refuses (exit 2) when the layout has
# no committed plan or is already registered. Run it from anywhere; it works in
# the repository that holds it. Bazel commands go through flock on $HOME/bazel.lock.
set -e
if [ $# -lt 4 ]; then
  sed -n 2,3p "$0" | sed 's/^# \{0,1\}//' >&2
  exit 2
fi
name=$1; title=$2; changed=$3; why=$4; shift 4
status=chosen; commit=this; competed=""
while [ $# -gt 0 ]; do
  case "$1" in
    --status) status=$2; shift 2 ;;
    --commit) commit=$2; shift 2 ;;
    --competed-with) competed=$2; shift 2 ;;
    *) echo "add.sh: unknown argument $1" >&2; exit 2 ;;
  esac
done
case "$status" in chosen|candidate) ;; *) echo "add.sh: --status is chosen or candidate" >&2; exit 2 ;; esac
cd "$(dirname "$0")/../.."
export PATH=$HOME/.local/bin:$PATH
V=${V:-$HOME/video}; export V; mkdir -p "$V"
pages="python3 -I -B tools/evolution/pages.py"

[ -f "layouts/$name.json" ] || { echo "add.sh: layouts/$name.json does not exist" >&2; exit 2; }
if [ ! -f "plans/$name.json" ]; then
  echo "add.sh: layouts/$name.json has no committed plan (plans/$name.json); the scorecard needs one" >&2
  exit 2
fi
id=$(python3 -I -B -c 'import json,sys
v = json.load(open("versions/versions.json"))["versions"]
print("v%02d" % (max(int(x["id"][1:]) for x in v) + 1))')
planned=$(python3 -I -B -c 'import json,sys; print(json.load(open(sys.argv[1]))["bay_planned_s"])' "plans/$name.json")
raw=$V/$id-raw.gif

echo "add.sh: 1/5 scoring $name"
flock -o "$HOME/bazel.lock" bazel run --config=opt --config=agent //:score_layout -- "$name" --write

echo "add.sh: 2/5 recording the bay of $name ($planned s)"
xvfb_pid=""
if ! xdpyinfo -display :7 > /dev/null 2>&1; then
  Xvfb :7 -screen 0 1920x1200x24 +extension GLX > "$V/xvfb.log" 2>&1 &
  xvfb_pid=$!
  trap '[ -n "$xvfb_pid" ] && kill "$xvfb_pid" 2> /dev/null || true' EXIT
  sleep 3
fi
tools/video/play_gif.sh "$name" "$raw" 960x600 16 0 "$planned" 64 \
  --lookat 0,0,0.9 --distance 16 --azimuth 215 --elevation -30

echo "add.sh: 3/5 registering $id"
args=(add --layout "$name" --title "$title" --changed "$changed" --why "$why" --status "$status"
      --commit "$commit" --recording "docs/evolution/$id.gif")
[ -n "$competed" ] && args+=(--competed-with "$competed")
$pages "${args[@]}"

echo "add.sh: 4/5 captioning docs/evolution/$id.gif"
mapfile -t lines < <($pages caption "$id")
tools/evolution/caption.sh "$raw" "docs/evolution/$id.gif" "${lines[@]}"
rm -f "$raw"

echo "add.sh: 5/5 writing the pages"
$pages write
echo "add.sh: $id ($status) added for $name: versions/versions.json, scores/$name.json, docs/evolution/$id.gif, docs/directions/, README.md changed"
