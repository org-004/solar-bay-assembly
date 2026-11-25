#!/bin/sh
# measure.sh [--list P<n>[.<part>] | --check [--strict-items]]
#
# Prints how far the repository departs from its design principles
# (docs/architecture/principles.md): one line "P<n> <figure> <what>" per
# principle (P1 to P11), "P<n>.<part> <figure> <what>" for its parts, and
# "P<n>.frozen" for the part of a figure that lies in the stand-in cell's
# files. Every figure counts departures; lower is better. A line that
# needs `bazel query` ends in "(bazel)" and is left out, with a note on
# stderr, when bazel is not on PATH.
#
#   --list P<n>   prints the places one line counts, one per line.
#   --check       exits 0 when tools/architecture/deps.py passes, when the
#                 output equals the compared part of
#                 docs/architecture/baseline.txt, and when every source,
#                 principle, backlog item, record, path and link the
#                 architecture documents cite exists; otherwise prints what
#                 does not, one line each, and exits 1. With --strict-items,
#                 a departure of deps.json whose backlog item is still "B?"
#                 fails as well.
#
# This is a thin wrapper: measure.py does the work and its opening comment
# says the rest, the one-liner that refreshes the baseline among it. Python
# runs isolated (-I), so that no file of this directory or of the caller's
# environment stands in for a module of the standard library. Needs git, a
# POSIX shell and Python 3. Exit codes: 0 ok, 1 --check found something,
# 2 usage error.
here=$(dirname "$0")
exec python3 -I "$here/measure.py" "$@"
