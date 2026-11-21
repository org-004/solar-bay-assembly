#!/bin/bash
# rec.sh NAME viewer-args...   records raw frames to $V/NAME.raw, log to $V/NAME.log
V=${V:-/tmp/m2-video}; mkdir -p $V
name=$1; shift
cd "$(dirname "$0")/../.."
start=$(date +%s)
DISPLAY=${REC_DISPLAY:-:7} timeout 3000 bazel run --config=opt //:view_cell -- --record $V/$name.raw "$@" > $V/$name.log 2>&1
echo "rc=$? wall=$(( $(date +%s)-start ))s"
grep -v "^INFO\|^Loading\|^Analyzing\|^\[" $V/$name.log | tail -3
ls -la $V/$name.raw
