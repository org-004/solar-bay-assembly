#!/usr/bin/env python3
"""tidy.sh [OPTIONS] [FILE.cc ...]

Runs clang-tidy over every translation unit of src/ and tests/ (or over the
files given, paths relative to the top of the repository) and counts what it
finds, in PLACES: distinct (file, line, column, check) with the file under
src/ or tests/. A header that many units include is counted once. Where
clang-tidy names two checks for one finding (an alias, "[a,b]"), the place is
counted under each name, and the pairs seen are listed on stderr as
"alias A B", so that only one of each pair need be switched on.

The compile database is made with compdb.py --for-clang into a temporary
directory of this script's own (removed at exit). The build is GCC's; clang-tidy
reads GCC's commands and GCC's standard library headers. The repository must
have been built once (bazel build --config=opt //...) so that the outside
libraries' headers are in Bazel's execution root; if a unit does not parse
(clang-diagnostic-error) it is printed and the script fails.

Options:
  --jobs N               units at a time (default 3)
  --checks LIST          clang-tidy's --checks, overriding .clang-tidy (this is
                         how candidates are counted before a .clang-tidy exists)
  --warnings-as-errors L clang-tidy's --warnings-as-errors, for trying the exit
                         status; .clang-tidy's WarningsAsErrors is used otherwise
  --baseline FILE        the baseline (default docs/architecture/baseline.txt at
                         the top of the repository; ignored if absent)
  --list CHECK           also print the places of CHECK with their messages
  --by-file              print "check file count" lines instead of the counts
  --raw DIR              keep each unit's clang-tidy output in DIR
Environment: CLANG_TIDY (default clang-tidy-18), BAZEL (as for compdb.py).

The checks come from .clang-tidy at the top of the repository (passed as
--config-file) unless --checks is given; with neither, the script stops.

Prints on stdout: a first line with clang-tidy's version, then
"T <check> <places>" for every check with at least one place, sorted by name,
then "T total <places>". On stderr: aliases, the five slowest units, and the
wall time (monotonic clock).

Exit status: 0 when nothing is wrong; 1 when clang-tidy reported an error (a
check under WarningsAsErrors fired, or a unit did not parse; the lines are
printed) or when a check's count is above its baseline: the baseline's lines
"T <check> <n>" after the line starting "# not compared" (a check absent from
it has baseline 0), each check above it is named on stderr; 2 for bad usage.
"""
import concurrent.futures
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
DIAG = re.compile(r"^(.+?):(\d+):(\d+): (warning|error): (.*?)(?: \[([A-Za-z0-9_.,-]+)\])?$")


def usage_error(msg):
    sys.stderr.write("tidy.sh: %s\n" % msg)
    sys.exit(2)


def parse_args(argv):
    o = {"jobs": 3, "checks": None, "wae": None, "baseline": None, "list": None,
         "by_file": False, "raw": None, "files": []}
    i = 0
    while i < len(argv):
        a = argv[i]
        name, eq, val = a.partition("=")
        if name in ("--jobs", "--checks", "--warnings-as-errors", "--baseline", "--list", "--raw"):
            if not eq:
                i += 1
                if i >= len(argv):
                    usage_error("%s needs a value" % name)
                val = argv[i]
            o[{"--jobs": "jobs", "--checks": "checks", "--warnings-as-errors": "wae",
               "--baseline": "baseline", "--list": "list", "--raw": "raw"}[name]] = val
        elif a == "--by-file":
            o["by_file"] = True
        elif a in ("-h", "--help"):
            print(__doc__)
            sys.exit(0)
        elif a.startswith("-"):
            usage_error("unknown option " + a)
        else:
            o["files"].append(os.path.relpath(os.path.abspath(a), ROOT))
        i += 1
    try:
        o["jobs"] = max(1, int(o["jobs"]))
    except ValueError:
        usage_error("--jobs needs a number")
    return o


def repo_path(path, execroot):
    """The path relative to the repository if it is under src/ or tests/, else None."""
    cands = [path]
    if not os.path.isabs(path):
        cands.append(os.path.join(execroot, path))
    for c in cands:
        for p in (os.path.normpath(c), os.path.realpath(c)):
            if p.startswith(ROOT + "/"):
                p = p[len(ROOT) + 1:]
            elif os.path.isabs(p):
                continue
            if re.match(r"(src|tests)/[^/]+$", p):
                return p
    return None


def read_baseline(path):
    base, on = {}, False
    with open(path) as f:
        for line in f:
            if line.startswith("# not compared"):
                on = True
                continue
            m = re.match(r"T (\S+) (\d+)\s*$", line)
            if on and m:
                base[m.group(1)] = int(m.group(2))
    return base


def main(argv):
    start = time.monotonic()
    o = parse_args(argv)
    tidy = shlex.split(os.environ.get("CLANG_TIDY", "clang-tidy-18"))
    config = os.path.join(ROOT, ".clang-tidy")
    if o["checks"] is None and not os.path.exists(config):
        usage_error("there is no .clang-tidy at the top of the repository; give --checks")
    version = subprocess.run(tidy + ["--version"], stdout=subprocess.PIPE, text=True).stdout
    vline = next((l.strip() for l in version.splitlines() if "version" in l), "clang-tidy (unknown version)")
    print(vline)
    sys.stdout.flush()

    tmp = tempfile.TemporaryDirectory(prefix="tidy-")
    with tmp:
        r = subprocess.run([sys.executable, os.path.join(HERE, "compdb.py"), "--for-clang", tmp.name],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if r.returncode != 0:
            sys.stderr.write(r.stderr)
            sys.exit("tidy.sh: compdb.py failed")
        db = json.load(open(os.path.join(tmp.name, "compile_commands.json")))
        execroot = db[0]["directory"]
        # Bazel keeps the outside libraries in <output base>/external and links
        # them into the execution root only for the repositories its latest
        # command used, so another Bazel command can take a link away between
        # two runs here. The database is therefore pointed at the output
        # base's own copies.
        ext = os.path.join(os.path.dirname(os.path.dirname(execroot)), "external")
        for e in db:
            e["arguments"] = [re.sub(r"^(-I|-isystem|-iquote|-idirafter)?external/",
                                     lambda m: (m.group(1) or "") + ext + "/", x)
                              for x in e["arguments"]]
        with open(os.path.join(tmp.name, "compile_commands.json"), "w") as f:
            json.dump(db, f)
        units = [e["file"] for e in db]
        if o["files"]:
            for f in o["files"]:
                if f not in units:
                    usage_error("%s is not a translation unit of src/ or tests/" % f)
            units = o["files"]
        # The biggest files first, so that the last units to run are small.
        units.sort(key=lambda u: -os.path.getsize(os.path.join(ROOT, u)))
        if o["raw"]:
            os.makedirs(o["raw"], exist_ok=True)

        base_cmd = tidy + ["-p", tmp.name, "--quiet",
                           "--header-filter=(^|/)(src|tests)/[^/]+$"]
        if o["checks"] is not None:
            base_cmd.append("--checks=" + o["checks"])
        if os.path.exists(config):
            base_cmd.append("--config-file=" + config)
        if o["wae"] is not None:
            base_cmd.append("--warnings-as-errors=" + o["wae"])

        def run(unit):
            t0 = time.monotonic()
            p = subprocess.run(base_cmd + [unit], cwd=ROOT, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, errors="replace")
            dt = time.monotonic() - t0
            if o["raw"]:
                with open(os.path.join(o["raw"], unit.replace("/", "_") + ".txt"), "w") as f:
                    f.write(p.stdout)
                    f.write("# exit %d, %.1f s\n" % (p.returncode, dt))
            return unit, p.returncode, p.stdout, dt

        places = {}   # (file, line, col, check) -> message
        aliases = set()
        errors, parse_fail, times = [], [], []
        with concurrent.futures.ThreadPoolExecutor(o["jobs"]) as ex:
            for unit, rc, out, dt in ex.map(run, units):
                times.append((dt, unit))
                seen_error = False
                for line in out.splitlines():
                    m = DIAG.match(line)
                    if not m:
                        continue
                    path, ln, col, sev, msg, checks = m.groups()
                    names = [c for c in (checks or "").split(",") if c != "-warnings-as-errors"]
                    if sev == "error":
                        seen_error = True
                        if "clang-diagnostic-error" in names or not names:
                            parse_fail.append("%s: %s" % (unit, line))
                        else:
                            errors.append(line)
                    f = repo_path(path, execroot)
                    if f is None or not names:
                        continue
                    if sev == "error" and "clang-diagnostic-error" in names:
                        continue
                    if len(names) > 1:
                        aliases.add(tuple(sorted(names)))
                    for c in names:
                        places.setdefault((f, int(ln), int(col), c), msg)
                if rc != 0 and not seen_error:
                    parse_fail.append("%s: clang-tidy exited %d without a diagnostic; its last lines: %s"
                                      % (unit, rc, " | ".join(out.strip().splitlines()[-3:])))

    status = 0
    if parse_fail:
        status = 1
        sys.stdout.write("\n".join(sorted(set(parse_fail))) + "\n")
        sys.stderr.write("tidy.sh: a unit did not parse. If a header is missing, run "
                         "`bazel build --config=opt //...` first, so that the outside libraries "
                         "are in Bazel's execution root.\n")
    if errors:
        status = 1
        sys.stdout.write("\n".join(sorted(set(errors))) + "\n")
        sys.stderr.write("tidy.sh: error from a check under WarningsAsErrors: %s\n" % ", ".join(
            sorted(set(re.search(r"\[([^\],]+)", e).group(1) for e in errors if "[" in e))))

    counts, byfile = {}, {}
    for (f, ln, col, c) in places:
        counts[c] = counts.get(c, 0) + 1
        byfile[(c, f)] = byfile.get((c, f), 0) + 1
    if o["by_file"]:
        for (c, f), n in sorted(byfile.items()):
            print(c, f, n)
    else:
        for c in sorted(counts):
            print("T %s %d" % (c, counts[c]))
        print("T total %d" % len(places))
    if o["list"]:
        for (f, ln, col, c), msg in sorted(places.items()):
            if c == o["list"]:
                print("%s:%d:%d: %s [%s]" % (f, ln, col, msg, c))
    for a in sorted(aliases):
        sys.stderr.write("alias %s\n" % " ".join(a))

    bpath = o["baseline"] or os.path.join(ROOT, "docs", "architecture", "baseline.txt")
    if os.path.exists(bpath):
        base = read_baseline(bpath)
        above = [(c, n, base.get(c, 0)) for c, n in sorted(counts.items()) if n > base.get(c, 0)]
        for c, n, b in above:
            sys.stderr.write("tidy.sh: %s has %d places, above its baseline %d\n" % (c, n, b))
        if above:
            status = 1
    elif o["baseline"]:
        usage_error("no baseline file " + o["baseline"])

    times.sort(reverse=True)
    sys.stderr.write("slowest: " + ", ".join("%s %.1f s" % (u, t) for t, u in times[:5]) + "\n")
    sys.stderr.write("wall_s=%.1f\n" % (time.monotonic() - start))
    sys.exit(status)


if __name__ == "__main__":
    main(sys.argv[1:])
