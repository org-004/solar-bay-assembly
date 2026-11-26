#!/bin/sh
# ci.sh
#
# The one entry point of the repository's architecture checks, for CI and for
# an agent before it hands work over. It runs, in this order,
#
#   1. tools/architecture/measure.sh --check   the documents, the dependency
#                                              rules and the figures against
#                                              docs/architecture/baseline.txt;
#   2. tools/architecture/tidy.sh              clang-tidy over every file of
#                                              src/ and tests/ against the
#                                              same baseline.
#   3. tools/evolution/pages.py check          the pages of the directions,
#                                              their charts and the README's
#                                              results are what pages.py writes
#                                              from versions/versions.json and
#                                              scores/ (ADR 0023).
#
# Before the second step it makes sure that the headers clang-tidy reads are
# in Bazel's output base: the outside libraries' headers (Eigen, MuJoCo,
# GLFW, Boost, OMPL, ...) and the headers that a rule generates (OMPL's
# config.h). The command is
#   bazel build --config=opt --output_groups=compilation_prerequisites_INTERNAL_ //...
# which fetches the libraries and makes those files but compiles nothing, so
# MuJoCo and OMPL are not built. Its failure is reported and the checks run
# anyway; a header it did not provide then shows up as a unit that does not
# parse.
#
# Every step runs even when an earlier one fails, so that one run shows everything.
# Each step's name is printed on standard output as "== step N: ..." before
# it, and "== N: ok" or "== N: FAILED (exit C)" after it; the last line is
# "ci.sh: ok" or "ci.sh: FAILED: <the steps>".
#
# Needs: git, Python 3, Bazel through Bazelisk (the repository's
# .bazelversion names the version), and clang-tidy 18 (Debian: clang-tidy;
# Ubuntu: clang-tidy-18). The environment variables BAZEL (Bazel's command,
# with its startup options, default bazel) and CLANG_TIDY (default
# clang-tidy-18) are passed on to the tools. Nothing is written into the
# checkout.
#
# Exit codes: 0 every step passed; 1 a step failed.
cd "$(dirname "$0")/../.." || exit 2
here=tools/architecture
failed=

echo "== step 1: measure.sh --check (documents, dependency rules, figures)"
"$here/measure.sh" --check
rc=$?
if [ $rc -eq 0 ]; then
    echo "== 1: ok"
else
    echo "== 1: FAILED (exit $rc)"
    failed="$failed measure.sh"
fi

echo "== step 2: tidy.sh (headers of the outside libraries, then clang-tidy)"
# $BAZEL is not quoted on purpose: it may carry startup options.
${BAZEL:-bazel} build --config=opt \
    --output_groups=compilation_prerequisites_INTERNAL_ //...
rc=$?
if [ $rc -ne 0 ]; then
    echo "== 2: the headers of the outside libraries could not be made (exit $rc)"
fi
"$here/tidy.sh"
rc=$?
if [ $rc -eq 0 ]; then
    echo "== 2: ok"
else
    echo "== 2: FAILED (exit $rc)"
    failed="$failed tidy.sh"
fi

echo "== step 3: pages.py check (direction pages, charts, the README's results)"
python3 -B tools/evolution/pages.py check
rc=$?
if [ $rc -eq 0 ]; then
    echo "== 3: ok"
else
    echo "== 3: FAILED (exit $rc)"
    failed="$failed pages.py"
fi

if [ -z "$failed" ]; then
    echo "ci.sh: ok"
    exit 0
fi
echo "ci.sh: FAILED:$failed"
exit 1
