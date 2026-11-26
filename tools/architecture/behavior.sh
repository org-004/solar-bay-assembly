#!/bin/sh
# behavior.sh: records what this checkout does, and compares two records.
#
# A refactoring must not change what the programs print, which tests exist or
# whether they pass. This script writes that down ("capture") and tells two
# records apart ("compare"), so that an agent can show a change altered none
# of it. What counts as behavior is in docs/architecture/behavior.md.
#
#   behavior.sh capture DIR [--no-tests] [--gate]
#   behavior.sh compare A B
#
# capture writes into DIR (made if absent; it must be empty):
#   - one file for each line of programs.txt (for a line that names `@layout`
#     or `@plan`, one for each layout that `check_layout --list` prints at
#     capture time, or each plans/*.json). The file holds `exit: N`, then
#     `--- stdout` and what the program printed, then `--- stderr` and what
#     it wrote there. Both streams are kept: the programs' stderr carries
#     their summaries (and wall-clock fields, which programs.txt cuts), and no
#     Bazel message reaches it because the programs are not started by Bazel
#     but by the script `bazel run --script_path` writes, which does what
#     `bazel run` does (changes into the runfiles directory, passes the
#     arguments of BUILD.bazel) and then runs the binary alone.
#   - tests.txt: the test targets, from `bazel query 'tests(//...)'`, sorted.
#   - unless --no-tests, results.txt: one line `<target> <RESULT>` for each
#     test target that `bazel test --config=opt --jobs=3 --local_resources=cpu=3
#     --test_tag_filters=-gate //...` ran, from Bazel's own summary (with
#     --gate, the gate is in it: no tag filter). A result served from Bazel's
#     cache is the real result of a test whose inputs did not change; Bazel
#     runs a test again when any input changed.
#   A program's own exit code is data, not an error: several check_layout
#   runs exit 1 by design, and capture records that and goes on. One line of
#   progress per program (id, exit code, seconds, bytes) goes to stderr.
#
# compare prints `same` and exits 0 when the two captures hold the same, or
# one line for each difference and exits 1. A line names a program (its file,
# the number of the first differing line and that line from each side, cut to
# 200 characters), a test target present on one side only, or a test whose
# result differs. A file present on one side only is a difference. When only
# one capture has results.txt it says so (the other was made with --no-tests).
#
# Everything uses --config=opt, at most three jobs and three cores, no
# display. Nothing is written into the checkout (Bazel keeps its own output
# base elsewhere); the script works from any current directory. The programs
# are built in one `bazel build` of every target in programs.txt, so a capture
# from a warm build starts at once. The list of programs, their arguments and
# the fields cut because they are wall-clock times are in programs.txt. The
# programs that need a display (viewer*, view_cell) are not in the list.
# Several captures must not run at once in one checkout (Bazel serializes
# them, which is fine) and the machine-wide memory lock, if there is one, is
# the caller's to take.
#
# The environment variable BAZEL, if set, is the Bazel command (default:
# bazel). Python 3 is needed (standard library only): behavior_lib.py, run with
# -I because other scripts in this directory are named like standard modules.
#
# Exit status: 0 capture done, or compare found no difference; 1 compare
# found a difference; 2 usage error, a build that failed, or a capture that
# could not be made (a message says why).

here=$(cd "$(dirname "$0")" && pwd) || exit 2
root=$(cd "$here/../.." && pwd) || exit 2
lib="$here/behavior_lib.py"
programs="$here/programs.txt"
bazel=${BAZEL:-bazel}

usage() {
  echo "usage: behavior.sh capture DIR [--no-tests] [--gate]" >&2
  echo "       behavior.sh compare A B" >&2
  exit 2
}

[ $# -ge 1 ] || usage
mode=$1
shift

case $mode in
compare)
  [ $# -eq 2 ] || usage
  [ -d "$1" ] && [ -d "$2" ] || {
    echo "behavior: compare wants two capture directories" >&2
    exit 2
  }
  exec python3 -I "$lib" compare "$1" "$2"
  ;;
capture) ;;
*) usage ;;
esac

[ $# -ge 1 ] || usage
dir=$1
shift
tests=yes
gate=no
for a in "$@"; do
  case $a in
  --no-tests) tests=no ;;
  --gate) gate=yes ;;
  *) usage ;;
  esac
done

mkdir -p "$dir" || exit 2
dir=$(cd "$dir" && pwd) || exit 2
if [ -n "$(ls -A "$dir")" ]; then
  echo "behavior: $dir is not empty" >&2
  exit 2
fi

flags="--config=opt --jobs=3 --local_resources=cpu=3"
quiet="--noshow_progress --ui_event_filters=-info,-warning,-debug"
scratch=$(mktemp -d) || exit 2
trap 'rm -rf "$scratch"' EXIT INT TERM

cd "$root" || exit 2

targets=$(python3 -I "$lib" targets "$programs") || exit 2

# Build every program of the list once; the checks below then start at once.
# shellcheck disable=SC2086
if ! $bazel build $flags $quiet $targets >"$scratch/build.log" 2>&1; then
  cat "$scratch/build.log" >&2
  echo "behavior: the build failed" >&2
  exit 2
fi

# One script for each target: it starts the binary the way `bazel run` does.
for t in $targets; do
  name=${t#//:}
  # shellcheck disable=SC2086
  $bazel run $flags $quiet --script_path="$scratch/$name.sh" "$t" \
    >"$scratch/run.log" 2>&1 || {
    cat "$scratch/run.log" >&2
    echo "behavior: cannot make the start script of $t" >&2
    exit 2
  }
done

python3 -I "$lib" run "$programs" "$scratch" "$dir" "$root" || exit 2

# shellcheck disable=SC2086
$bazel query $quiet 'tests(//...)' >"$scratch/tests.raw" 2>"$scratch/query.err" || {
  cat "$scratch/query.err" >&2
  echo "behavior: bazel query failed" >&2
  exit 2
}
LC_ALL=C sort "$scratch/tests.raw" >"$dir/tests.txt"

if [ "$tests" = yes ]; then
  filter="--test_tag_filters=-gate"
  [ "$gate" = yes ] && filter=""
  # Test failures make Bazel exit 3 and are results, not errors. Other
  # failures (a build error is exit 1; a bad command line, 2) leave a test
  # without a result, and compare shows that, but an exit of 2 is stopped.
  # shellcheck disable=SC2086
  $bazel test $flags $quiet --keep_going --test_summary=short $filter //... \
    >"$scratch/test.log" 2>&1
  rc=$?
  if [ $rc -ge 4 ] || [ $rc -eq 2 ]; then
    tail -n 30 "$scratch/test.log" >&2
    echo "behavior: bazel test stopped with exit $rc" >&2
    exit 2
  fi
  python3 -I "$lib" results "$scratch/test.log" >"$dir/results.txt" || exit 2
  if [ ! -s "$dir/results.txt" ]; then
    tail -n 30 "$scratch/test.log" >&2
    echo "behavior: no test result in Bazel's output" >&2
    exit 2
  fi
fi
exit 0
