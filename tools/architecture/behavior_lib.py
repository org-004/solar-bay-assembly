#!/usr/bin/env python3
"""behavior_lib.py: the parts of behavior.sh that are easier in Python.

Not run by hand; behavior.sh calls it. Sub-commands:

  targets PROGRAMS
      Prints the Bazel targets (//:name) that the program list names, once
      each, one per line.

  run PROGRAMS SCRIPTDIR OUTDIR ROOT
      Runs every line of the program list and writes one file for each into
      OUTDIR. SCRIPTDIR holds one <target>.sh for each target, written by
      `bazel run --script_path`: the script changes into the runfiles
      directory, sets the environment Bazel would and starts the binary with
      the arguments BUILD.bazel gives it, then the arguments of the call. The
      names of the layouts (for `@layout`) come from running check_layout
      with --list; the names of the committed plans (for `@plan`) are the
      files plans/*.json under ROOT. A line of progress for each program goes
      to stderr: id, exit code, seconds, bytes. Exit status 0.

  results LOGFILE
      Reads the summary of a `bazel test` run and prints one line for each
      test target, `<target> <RESULT>`, sorted. A result served from Bazel's
      cache is the same word as one that was just run: an unchanged test that
      passed is a true result. The times are left out.

  compare A B
      Compares two captures; see behavior.sh.

The program list (tools/architecture/programs.txt) has one line for each
capture: `id | target | arguments | cuts`. The arguments are split like a
shell would. `@layout` in the id and the arguments stands for each layout
name, `@plan` for each committed plan. The cuts are separated by ` ;; `, or
a single `-` for none:

  col:NAME      in a CSV table (a line whose comma-separated fields include
                NAME), every non-empty value in that column of the rows
                below it is replaced by the token <wall>. This is for a
                column that holds a wall-clock time.
  tab:NAME[#K]  the same for an aligned table (the header is a line without a
                digit that has the word NAME K+1 times; K counts from 0): in
                the rows below it, up to a blank line, the value that ends
                where the word ends.
  re:PATTERN    in each line of the output, the text of every group of
                PATTERN (a Python regular expression, applied line by line)
                is replaced by <wall>. For a wall-clock time in a sentence.
  digest        the output is replaced by its line count, byte count and
                SHA-256: for an output too large to keep, which still shows
                any change.

The cuts apply to both streams. What a cut replaces is a wall-clock time or
a figure derived from one; programs.txt's notes (and behavior.md) give the
source line that shows it.
"""
import hashlib
import os
import re
import shlex
import subprocess
import sys
import time

TOKEN = "<wall>"
LIMIT = 200


def read_programs(path):
    """Returns [(id, target, args, cuts)] for the lines of the list."""
    out = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.rstrip("\n")
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            parts = [p.strip() for p in line.split(" | ", 3)]
            if len(parts) != 4:
                sys.exit("%s:%d: want 'id | target | args | cuts'" % (path, n))
            pid, target, args, cuts = parts
            cuts = [] if cuts == "-" else [c.strip() for c in cuts.split(" ;; ")]
            out.append((pid, target, args, cuts))
    return out


def field_spans(line):
    """Spans (start, end) of the fields of one CSV line, quotes respected."""
    spans, start, quoted = [], 0, False
    for i, ch in enumerate(line):
        if ch == '"':
            quoted = not quoted
        elif ch == "," and not quoted:
            spans.append((start, i))
            start = i + 1
    spans.append((start, len(line)))
    return spans


def cut_col(text, name):
    lines = text.split("\n")
    width = index = None
    for i, line in enumerate(lines):
        if line.startswith("#") or not line:
            continue
        spans = field_spans(line)
        fields = [line[a:b] for a, b in spans]
        if name in fields:  # a header row (a new table may start anywhere)
            width, index = len(fields), fields.index(name)
            continue
        if width is not None and len(spans) == width:
            a, b = spans[index]
            if b > a:
                lines[i] = line[:a] + TOKEN + line[b:]
    return "\n".join(lines)


def cut_tab(text, spec):
    name, _, k = spec.partition("#")
    k = int(k) if k else 0
    lines = text.split("\n")
    edge = None
    word = re.compile(r"(?<!\S)" + re.escape(name) + r"(?!\S)")
    for i, line in enumerate(lines):
        if not line.strip():
            edge = None  # a table ends at a blank line
            continue
        if line.startswith("#"):
            continue
        found = list(word.finditer(line))
        if len(found) > k and not re.search(r"\d", line):
            edge = found[k].end()  # a header row has no digit
            continue
        if edge is None or len(line) < edge or line[edge - 1] == " ":
            continue
        if len(line) > edge and line[edge] not in " *":
            continue
        start = edge
        while start > 0 and line[start - 1] != " ":
            start -= 1
        lines[i] = line[:start] + TOKEN + line[edge:]
    return "\n".join(lines)


def cut_re(text, pattern):
    rx = re.compile(pattern)
    out = []
    for line in text.split("\n"):
        m = rx.search(line)
        if m:
            spans = [m.span(g) for g in range(1, (rx.groups or 0) + 1)
                     if m.span(g)[0] >= 0]
            for a, b in sorted(spans, reverse=True):
                line = line[:a] + TOKEN + line[b:]
        out.append(line)
    return "\n".join(out)


def apply_cuts(text, cuts):
    for c in cuts:
        if c == "digest":
            b = text.encode("utf-8", "replace")
            return "digest: %d lines, %d bytes, sha256 %s\n" % (
                text.count("\n"), len(b), hashlib.sha256(b).hexdigest())
        kind, _, arg = c.partition(":")
        if kind == "col":
            text = cut_col(text, arg)
        elif kind == "tab":
            text = cut_tab(text, arg)
        elif kind == "re":
            text = cut_re(text, arg)
        else:
            sys.exit("unknown cut: " + c)
    return text


def run_one(script, args):
    p = subprocess.run(["sh", script] + args, stdin=subprocess.DEVNULL,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return (p.returncode, p.stdout.decode("utf-8", "replace"),
            p.stderr.decode("utf-8", "replace"))


def cmd_targets(programs):
    seen = []
    for _, target, _, _ in read_programs(programs):
        if target not in seen:
            seen.append(target)
    print("\n".join(seen))


def cmd_run(programs, scripts, outdir, root):
    entries = read_programs(programs)
    layouts = None
    plans = sorted(f[:-5] for f in os.listdir(os.path.join(root, "plans"))
                   if f.endswith(".json"))
    for pid, target, args, cuts in entries:
        names = [None]
        if "@layout" in pid + args:
            if layouts is None:
                rc, out, _ = run_one(os.path.join(scripts, "check_layout.sh"),
                                     ["--list"])
                layouts = [l.split("\t")[0] for l in out.split("\n")
                           if l.strip()]
                if rc != 0 or not layouts:
                    sys.exit("behavior: check_layout --list gave no layouts")
            names = layouts
        elif "@plan" in pid + args:
            names = plans
        for name in names:
            sub = (lambda s: s) if name is None else (
                lambda s: s.replace("@layout", name).replace("@plan", name))
            argv = [] if args == "-" else shlex.split(sub(args))
            script = os.path.join(scripts, target.lstrip("/:") + ".sh")
            t0 = time.monotonic()
            rc, out, err = run_one(script, argv)
            dt = time.monotonic() - t0
            body = "exit: %d\n--- stdout\n%s--- stderr\n%s" % (
                rc, apply_cuts(out, cuts), apply_cuts(err, cuts))
            if not body.endswith("\n"):
                body += "\n"
            with open(os.path.join(outdir, sub(pid)), "w") as f:
                f.write(body)
            sys.stderr.write("%s exit=%d %.1fs bytes=%d\n"
                             % (sub(pid), rc, dt, len(out) + len(err)))


STATUS = re.compile(
    r"^(//\S+)\s+(?:\(cached\)\s+)?"
    r"(PASSED|FAILED TO BUILD|FAILED|TIMEOUT|FLAKY|NO STATUS|SKIPPED|"
    r"INCOMPLETE|CANCELLED)\b")


def cmd_results(log):
    found = {}
    with open(log, errors="replace") as f:
        for line in f:
            m = STATUS.match(line.strip())
            if m:
                found[m.group(1)] = m.group(2)
    for t in sorted(found):
        print(t, found[t])


def clip(s):
    return s[:LIMIT]


def first_diff(a, b):
    la, lb = a.split("\n"), b.split("\n")
    for i in range(max(len(la), len(lb))):
        x = la[i] if i < len(la) else "<end of file>"
        y = lb[i] if i < len(lb) else "<end of file>"
        if x != y:
            return i + 1, x, y
    return 0, "", ""


def read(path):
    with open(path, errors="replace") as f:
        return f.read()


def pairs(path):
    out = {}
    for line in read(path).split("\n"):
        if line.strip():
            k, _, v = line.partition(" ")
            out[k] = v
    return out


def cmd_compare(a, b):
    diffs = []
    fa, fb = set(os.listdir(a)), set(os.listdir(b))
    for name in sorted(fa | fb):
        if name in ("tests.txt", "results.txt"):
            continue
        if name not in fb:
            diffs.append("%s: only in %s" % (name, a))
        elif name not in fa:
            diffs.append("%s: only in %s" % (name, b))
        else:
            x, y = read(os.path.join(a, name)), read(os.path.join(b, name))
            if x != y:
                n, p, q = first_diff(x, y)
                diffs.append("%s: line %d: a: %s | b: %s"
                             % (name, n, clip(p), clip(q)))
    if "tests.txt" in fa and "tests.txt" in fb:
        ta = set(read(os.path.join(a, "tests.txt")).split())
        tb = set(read(os.path.join(b, "tests.txt")).split())
        for t in sorted(ta - tb):
            diffs.append("tests.txt: %s only in %s" % (t, a))
        for t in sorted(tb - ta):
            diffs.append("tests.txt: %s only in %s" % (t, b))
    elif "tests.txt" in fa or "tests.txt" in fb:
        diffs.append("tests.txt: only in %s" % (a if "tests.txt" in fa else b))
    if "results.txt" in fa and "results.txt" in fb:
        ra, rb = pairs(os.path.join(a, "results.txt")), pairs(
            os.path.join(b, "results.txt"))
        for t in sorted(set(ra) | set(rb)):
            if t not in rb:
                diffs.append("%s: result only in %s (%s)" % (t, a, ra[t]))
            elif t not in ra:
                diffs.append("%s: result only in %s (%s)" % (t, b, rb[t]))
            elif ra[t] != rb[t]:
                diffs.append("%s: a: %s | b: %s" % (t, ra[t], rb[t]))
    elif "results.txt" in fa or "results.txt" in fb:
        have, lack = (a, b) if "results.txt" in fa else (b, a)
        diffs.append("results.txt: only %s has test results; %s was captured "
                     "with --no-tests" % (have, lack))
    if diffs:
        print("\n".join(diffs))
        return 1
    print("same")
    return 0


def main(argv):
    cmd = argv[1] if len(argv) > 1 else ""
    if cmd == "targets" and len(argv) == 3:
        cmd_targets(argv[2])
    elif cmd == "run" and len(argv) == 6:
        cmd_run(*argv[2:])
    elif cmd == "results" and len(argv) == 3:
        cmd_results(argv[2])
    elif cmd == "compare" and len(argv) == 4:
        return cmd_compare(argv[2], argv[3])
    else:
        sys.stderr.write(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
