#!/bin/sh
# warnings.sh [OPTIONS]
#
# Counts the compiler warnings that a set of -W flags finds in this
# repository's own files (src/ and tests/), for choosing the flags of
# .bazelrc and for keeping their counts from growing. In short: it takes
# Bazel's own compile commands (tools/architecture/compdb.py), runs the
# compiler itself on every translation unit with the output sent to
# /dev/null, reads the diagnostics, and prints one line per flag with the
# number of distinct places (file, line, column, diagnostic) in src/ and
# tests/ and the number of places elsewhere (Eigen, MuJoCo, the system).
# The Python program below the `exec` line has the full description.
#
# Options:
#   (none)         the commands of `--config strict` and every -W flag that
#                  config adds to the plain `--config=opt` commands
#   --flags A,B    the commands of the plain --config=opt build with these
#                  flags appended (how candidates are counted)
#   --cc COMPILER  replace the compiler in every command
#   --extra FLAG   append a flag that is not counted (repeatable)
#   --jobs N       compilers at a time (default 3)
#   --raw DIR      keep each unit's diagnostics in DIR
#   --list FLAG    print the places of one flag (file:line:col: message)
#   --by-file      print "flag file count" lines
#   --check        exit 1 if a count is above docs/architecture/baseline.txt
#   --baseline F   with --check, read the baseline from F (a file in the
#                  format of docs/architecture/baseline.txt) instead
#   --probe        with --flags: say which flags the compiler knows, and stop
#
# The environment variable BAZEL, if set, is the Bazel command, startup
# options included (default: bazel). Exit codes: 0 done, 1 a count above its
# baseline or a compiler error, 2 a usage error, an unknown flag or a failure
# of Bazel (for example no `strict` config yet).
#
# The program is in this file, not in a warnings.py beside it, because a file of
# that name in the script's directory would shadow Python's own `warnings`
# module for every Python script run from here (compdb.py among them).
WARNINGS_DIR=$(cd "$(dirname "$0")" && pwd) || exit 2
export WARNINGS_DIR
exec python3 -I -c "$(sed -n '/^: <<.PYEOF.$/,$p' "$0" | sed '1d;$d')" "$0" "$@"
: <<'PYEOF'
"""The work of warnings.sh (the options are in its first comment).

What it does. It asks tools/architecture/compdb.py for the commands Bazel
compiles every translation unit of src/ and tests/ with, and runs each one in
Bazel's execution root with the output sent to /dev/null, the dependency
file options removed, `-fdiagnostics-color=never` and
`-fno-diagnostics-show-caret` added, and the optimization level of the
command kept (some warnings come from the optimizer). It reads stderr. It
does not read a Bazel log, because Bazel does not print a cached action's
warnings again and a second count would read zero.

Counting. A place is a distinct (file, line, column, diagnostic name), so a
header seen from many units counts once. A place whose file is under src/ or
tests/ is "inside"; a place anywhere else (the headers of Eigen and of the
other dependencies, the system's) is "outside": a flag that fires outside
cannot be made an error. The diagnostic name is the one GCC prints in
brackets, which can differ from the command line: -Wformat=2 reports
[-Wformat-nonliteral], -Woverloaded-virtual reports [-Woverloaded-virtual=].
Which diagnostic names an asked flag stands for is read from the compiler
(`-Q --help=warnings` with and without the flag), plus the flag's own name;
a name that belongs to several asked flags (-Wextra and -Wsign-compare) is
counted for each of them. A diagnostic with no name is called "(none)".

Output (standard output):
  line 1:  COMPILER FULLVERSION TARGET, from -dumpfullversion, -dumpmachine
  then:    W FLAG INSIDE OUTSIDE        for every asked flag, sorted by flag,
                                         zero counts included
           D NAME INSIDE OUTSIDE FLAGS  for every diagnostic name produced
           # FLAG produced: NAMES       the names each asked flag produced
           # asked, nothing produced: FLAGS
  With --list FLAG (an asked flag or a diagnostic name) only line 1 and the
  places of that flag; with --by-file only line 1 and `FLAG FILE COUNT` lines
  (inside places, by asked flag). Notes about the run go to standard error.

--check reads, in docs/architecture/baseline.txt of this checkout (or in the
file named by --baseline, which must exist), the lines
`W FLAG N` after the line that starts with `# not compared`, and exits 1,
naming each flag whose count is above its N.

Exit codes: 0 done; 1 a compiler error in a unit (printed) or a count above its
baseline; 2 usage, an unknown flag (the compiler's message is printed), or
Bazel failing.
"""
import collections
import concurrent.futures
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.environ["WARNINGS_DIR"]
ROOT = os.path.dirname(os.path.dirname(HERE))
ENV = dict(os.environ, LC_ALL="C", LANG="C")
DIAG = re.compile(r"^(\S[^:]*):(\d+):(\d+): (warning|error|fatal error): (.*)$")
NAMED = re.compile(r"^(.*?) \[(-[A-Za-z0-9=+_-]+)\]$")


def die(msg, code=2):
    sys.stderr.write(msg.rstrip("\n") + "\n")
    sys.exit(code)


def parse_args(argv):
    argv = ["x"] + argv
    o = dict(flags=None, cc=None, extra=[], jobs=3, raw=None, list=None,
             baseline=None, by_file=False, check=False, probe=False)
    i = 1
    while i < len(argv):
        a = argv[i]
        if a in ("--flags", "--cc", "--extra", "--jobs", "--raw", "--list", "--baseline"):
            i += 1
            if i >= len(argv):
                die(__doc__)
            v = argv[i]
            if a == "--flags":
                o["flags"] = [f for f in v.split(",") if f]
            elif a == "--extra":
                o["extra"].append(v)
            elif a == "--jobs":
                o["jobs"] = int(v)
            else:
                o[a[2:]] = v
        elif a in ("--by-file", "--check", "--probe"):
            o[a[2:].replace("-", "_")] = True
        else:
            die(__doc__)
        i += 1
    if o["probe"] and o["flags"] is None:
        die("--probe needs --flags")
    return o


def compdb(configs, outdir):
    cmd = [sys.executable, "-I", os.path.join(HERE, "compdb.py")]
    for c in configs:
        cmd += ["--config", c]
    sub = os.path.join(outdir, "cdb-" + ("-".join(configs) or "opt"))
    r = subprocess.run(cmd + [sub], cwd=ROOT, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, text=True)
    if r.returncode != 0:
        die("compdb.py %s failed (is there a `build:%s` in .bazelrc, and has the "
            "repository been built?):\n%s" % (" ".join(cmd[2:]), configs[-1] if configs else "opt",
                                              ((r.stderr or r.stdout).strip().splitlines() or ["no output"])[-1]))
    with open(os.path.join(sub, "compile_commands.json")) as f:
        return json.load(f)


def warning_flags(args):
    return {a for a in args if re.match(r"-W[a-z]", a) and not a.startswith(("-Wno-", "-Werror"))}


def prepare(args, cc, extra, flags, execroot):
    """The argument list for one unit: dependency files removed, output to /dev/null."""
    out, skip = [], False
    for a in args:
        if skip:
            skip = False
        elif a in ("-MF", "-MT", "-MQ"):
            skip = True
        elif a in ("-MD", "-MMD", "-MP"):
            pass
        elif a.startswith("-fdiagnostics-color"):
            pass
        else:
            out.append(a)
    # Replace the output file; the command has `-o X`.
    for k, a in enumerate(out):
        if a == "-o" and k + 1 < len(out):
            out[k + 1] = "/dev/null"
    # Bazel links an outside repository into the execution root only while a
    # command uses it, so another Bazel command can take the link away; the
    # output base's own copy stays. Point at that.
    ext = os.path.join(os.path.dirname(os.path.dirname(execroot)), "external")
    out = [re.sub(r"^(-I|-isystem|-iquote|-idirafter)?external/",
                  lambda m: (m.group(1) or "") + ext + "/", a) for a in out]
    if cc:
        out[0] = cc
    return out + flags + ["-fdiagnostics-color=never", "-fno-diagnostics-show-caret"] + extra


def compiler_info(cc):
    def ask(opt):
        r = subprocess.run([cc, opt], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           text=True, env=ENV)
        if r.returncode != 0:
            die("%s %s failed: %s" % (cc, opt, r.stderr))
        return r.stdout.strip()
    return "%s %s %s" % (os.path.basename(cc), ask("-dumpfullversion"), ask("-dumpmachine"))


def help_state(cc, flags):
    """{option name: state text} from -Q --help=warnings with the given flags."""
    r = subprocess.run([cc, "-x", "c++", "-std=c++20", "-Q", "--help=warnings"] + flags,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=ENV)
    if r.returncode != 0:
        die(r.stderr)
    st = {}
    for line in r.stdout.splitlines():
        m = re.match(r"^\s+(-W[^\s]*)\s+(.*)$", line)
        if m:
            name = re.sub(r"=<.*>$", "", m.group(1)).rstrip("=")
            if name not in st or m.group(2).strip().startswith("-W"):
                st[name] = m.group(2).strip()
    return st


def norm(name):
    return name.rstrip("=")


def names_of(cc, flag, base):
    """The diagnostic names an asked flag stands for: its own and what it turns on."""
    got = {norm(flag), norm(flag.split("=")[0])}
    st = help_state(cc, [flag])
    for k, v in st.items():
        if v != base.get(k) and v != "[disabled]":
            got.add(k)
    return got


def run_unit(entry, cc, extra, flags, rawdir):
    argv = prepare(entry["arguments"], cc, extra, flags, entry["directory"])
    r = subprocess.run(argv, cwd=entry["directory"], stdout=subprocess.DEVNULL,
                       stderr=subprocess.PIPE, text=True, errors="replace", env=ENV)
    if rawdir:
        with open(os.path.join(rawdir, entry["file"].replace("/", "__") + ".txt"), "w") as f:
            f.write(r.stderr)
    return entry["file"], r.returncode, r.stderr


def relpath(path, execroot):
    p = os.path.normpath(path)
    if os.path.isabs(p):
        if p.startswith(execroot + os.sep):
            p = p[len(execroot) + 1:]
    return p


def main():
    o = parse_args(sys.argv[2:])
    tmp = tempfile.mkdtemp(prefix="warnings-")
    try:
        if o["raw"]:
            os.makedirs(o["raw"], exist_ok=True)
        run(o, tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def run(o, tmp):
    if o["flags"] is not None:
        entries = compdb([], tmp)
        asked, append = sorted(set(o["flags"])), sorted(set(o["flags"]))
    else:
        entries = compdb(["strict"], tmp)
        plain = compdb([], tmp)
        have = set()
        for e in plain:
            have |= warning_flags(e["arguments"])
        now = set()
        for e in entries:
            now |= warning_flags(e["arguments"])
        asked, append = sorted(now - have), []
    if not entries:
        die("no translation units")
    cc = o["cc"] or entries[0]["arguments"][0]
    execroot = entries[0]["directory"]
    head = compiler_info(cc)

    # An unknown flag fails here with the compiler's own message.
    if o["probe"]:
        for f in asked:
            r = subprocess.run([cc, "-x", "c++", "-std=c++20", "-fsyntax-only", f, "/dev/null"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=ENV)
            print("known %s %s" % (f, "yes" if r.returncode == 0 else "no"))
        return
    if append:
        r = subprocess.run([cc, "-x", "c++", "-std=c++20", "-fsyntax-only"] + append + ["/dev/null"],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=ENV)
        if r.returncode != 0:
            die("the compiler refuses the flags:\n" + r.stderr)

    base = help_state(cc, [])
    groups = {f: names_of(cc, f, base) for f in asked}

    inside = collections.defaultdict(dict)    # name -> {(file, line, col): message}
    outside = collections.defaultdict(set)
    errors = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=o["jobs"]) as ex:
        futs = [ex.submit(run_unit, e, o["cc"], o["extra"], append, o["raw"]) for e in entries]
        for fu in futs:
            unit, rc, err = fu.result()
            bad = rc != 0
            for line in err.splitlines():
                m = DIAG.match(line)
                if not m:
                    continue
                path, ln, col, kind, msg = m.groups()
                if kind != "warning":
                    errors.append("%s: %s" % (unit, line))
                    bad = True
                    continue
                nm = NAMED.match(msg)
                name, text = (norm(nm.group(2)), nm.group(1)) if nm else ("(none)", msg)
                f = relpath(path, execroot)
                key = (f, int(ln), int(col))
                if f.startswith(("src/", "tests/")):
                    inside[name].setdefault(key, text)
                else:
                    outside[name].add(key)
            if bad and not any(e.startswith(unit + ":") for e in errors):
                errors.append("%s: the compiler failed (exit %d):\n%s" % (unit, rc, err[-1500:]))
    if errors:
        die("compiler errors:\n" + "\n".join(errors[:40]), 1)

    names = sorted(set(inside) | set(outside))
    result = {}
    for f in asked:
        ins, outs, used = {}, set(), set()
        for n in groups[f]:
            for k, t in inside.get(n, {}).items():
                ins[(n,) + k] = t
            outs |= {(n,) + k for k in outside.get(n, ())}
            if n in inside or n in outside:
                used.add(n)
        result[f] = (ins, outs, sorted(used))

    print(head)
    sys.stderr.write("# %d units, extra flags: %s, appended: %s\n" %
                     (len(entries), " ".join(o["extra"]) or "-", " ".join(append) or "-"))
    if o["list"]:
        target = o["list"]
        ins = {}
        if target in result:
            ins = result[target][0]
        else:
            for k, t in inside.get(norm(target), {}).items():
                ins[(norm(target),) + k] = t
        for k in sorted(ins, key=lambda k: (k[1], k[2], k[3], k[0])):
            print("%s:%d:%d: %s [%s]" % (k[1], k[2], k[3], ins[k], k[0]))
        return
    if o["by_file"]:
        for f in asked:
            c = collections.Counter(k[1] for k in result[f][0])
            for fl, n in sorted(c.items()):
                print("%s %s %d" % (f, fl, n))
        return
    for f in asked:
        print("W %s %d %d" % (f, len(result[f][0]), len(result[f][1])))
    for n in names:
        fl = [f for f in asked if n in groups[f]]
        print("D %s %d %d %s" % (n, len(inside.get(n, ())), len(outside.get(n, ())), ",".join(fl) or "-"))
    for f in asked:
        if result[f][2]:
            print("# %s produced: %s" % (f, " ".join(result[f][2])))
    none = [f for f in asked if not result[f][2]]
    print("# asked, nothing produced: %s" % (" ".join(none) or "-"))

    if o["check"]:
        path = o["baseline"] or os.path.join(ROOT, "docs", "architecture", "baseline.txt")
        if o["baseline"] and not os.path.exists(path):
            die("no baseline file " + path)
        if os.path.exists(path):
            limits, on = {}, False
            with open(path) as fh:
                for line in fh:
                    if line.startswith("# not compared"):
                        on = True
                    elif on:
                        p = line.split()
                        if len(p) == 3 and p[0] == "W":
                            limits[p[1]] = int(p[2])
            over = [f for f in asked if f in limits and len(result[f][0]) > limits[f]]
            for f in over:
                sys.stderr.write("above baseline: %s has %d places, baseline %d\n" %
                                 (f, len(result[f][0]), limits[f]))
            if over:
                sys.exit(1)


if __name__ == "__main__":
    main()
PYEOF
