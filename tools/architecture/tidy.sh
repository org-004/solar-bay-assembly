#!/bin/sh
# tidy.sh [OPTIONS] [FILE.cc ...]
#
# Runs clang-tidy over every translation unit of src/ and tests/ (or over the
# files given) and counts what it finds. The work is in tidy.py beside this
# file; this wrapper only finds Python 3. Everything is described in
# tidy.py's opening comment, which `tidy.sh --help` prints.
#
# Needs a build done once (bazel build --config=opt //...), so that the
# headers of the outside libraries are in Bazel's execution root. The
# environment variables CLANG_TIDY (default clang-tidy-18) and BAZEL (Bazel's
# command with its startup options) are passed on. Nothing is written into the
# checkout.
exec python3 "$(dirname "$0")/tidy.py" "$@"
