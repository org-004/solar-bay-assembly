#!/usr/bin/env python3
r"""measure.py [--list P<n>[.<part>] | --check [--strict-items]]   (run it as tools/architecture/measure.sh)

Measures how far the repository departs from its design principles
(docs/architecture/principles.md), one figure per principle. Every figure is
a count of departures: lower is better, and the target is 0 or "may not
grow". The rules and the data the counts need are in
tools/architecture/deps.json; the dependency check itself is
tools/architecture/deps.py, which this script imports.

Without arguments it prints one headline line per principle (P1 to P11),
    P<n> <figure> <what is counted>
and, where a principle has parts, detail lines
    P<n>.<part> <figure> <what is counted>
A line that needs `bazel query` ends in "(bazel)"; when bazel is not on PATH
it is left out and a note goes to stderr. The lines, in the order printed:

  P1  dependencies point one way: file-to-header pairs against the groups'
      allowed direction. P1.cycles, P1.namespace (src/ files, .cc included),
      and the guards P1.allowed, P1.listed, P1.frozen_files.
  P2  nothing moves outside the net: files under src/ that no test and no
      captured program depends on (bazel). P2.untested (bazel), P2.unrun.
  P3  sensor data only: files that read the simulation's truth. P3.allowed.
  P4  outside libraries: their number (bazel_dep, fetches, use_repo names and
      overrides) and who lists them. P4.unpinned, P4.mujoco ..., P4.group
      (also through project headers), P4.allowed.
  P5  a target lists what its files include. P5.unused_project,
      P5.unused_outside, P5.missing, P5.files. The headline is their sum.
  P6  the same on every platform and in every run. P6.clock (steady_clock
      read for more than a print), P6.allowed.
  P7  a fact has one home (src/ only). One detail line per fact.
  P8  committed outputs agree with their source. P8.unmatched, P8.no_test.
  P9  a finding is cured, not silenced. P9.allowed, P9.held.
  P10 documents point at what exists. P10.left_out.
  P11 a file is small enough to read and says what it is. P11.cc, P11.h,
      P11.comment.

A guard line (P1.allowed, P1.listed, P1.frozen_files, P3.allowed, P4.allowed,
P6.allowed, P8.no_test, P9.allowed, P10.left_out) counts what the rules of
deps.json excuse, and ends in "(a rule of deps.json; may not grow)": a rule
that is widened shows there. P9.held counts what is switched on (a guard that
may not fall).

For P1, P2, P3, P5, P6, P7, P9 and P11 a further detail line
    P<n>.frozen <k> of them in the stand-in cell's files, which a refactoring does not edit
counts the places of the headline whose file is in a group that deps.json
marks frozen (for a target, one all of whose files are frozen).

  --list P<n>          prints the places counted by the headline of P<n>,
  --list P<n>.<part>   or by one detail line, one per line.
  --check              exits 0 only when all of these hold, and otherwise
                       prints what does not hold, one line each, and exits 1:
     1. tools/architecture/deps.py passes (with --strict-items, a departure
        whose backlog item is still "B?" fails too; otherwise "B?" is
        accepted, and so is B<n> when docs/architecture/backlog.md has
        "## B<n>");
     2. the output equals the compared part of docs/architecture/baseline.txt:
        its lines before the first line that starts with "# not compared",
        blank lines and lines starting with "#" left out. Without Bazel the
        "(bazel)" lines are skipped on both sides. The lines after the mark
        (compiler warnings, clang-tidy) are never compared;
     3. every [S<n>] cited in docs/architecture/*.md and docs/adr/*.md is
        defined in docs/architecture/research.md by a line matching
        ^(- |#+ )?\[S[0-9]+\] ;
     4. every P<n> cited there is a "## P<n>" heading of
        docs/architecture/principles.md, every B<n> a "## B<n>" heading of
        docs/architecture/backlog.md, and every record cited as "ADR NNNN"
        or in "superseded by NNNN" a file docs/adr/NNNN-*.md;
     5. every path in backticks and every relative Markdown link in those
        documents exists (and, in AGENTS.md and README.md, see
        AGENTS_README_ALL_LINES below). A backticked token is a path when it
        starts with a tracked top-level directory or is a tracked top-level
        file; a glob must match a file; path:LINE and path:LINE-LINE must be
        within the file; a path followed by " (new)" or " (gone)" is not
        checked; a token with a placeholder (<, >, $, {) is not a path.
     A document that does not exist yet is reported as missing.

Refreshing the baseline. Nothing here writes into the checkout. To replace
the compared part of the baseline and keep its "# not compared" tail, from
the repository's root, with Bazel installed:

    { tools/architecture/measure.sh && sed -n '/^# not compared/,$p' docs/architecture/baseline.txt; } > "${TMPDIR:-/tmp}/baseline.txt" && mv "${TMPDIR:-/tmp}/baseline.txt" docs/architecture/baseline.txt

Needs git, a POSIX shell and Python 3's standard library; takes a few
seconds without Bazel, and about twice the time of two `bazel query` runs
with it. Exit codes: 0 (printed, or --check passed), 1 (--check found
something), 2 (usage error, or a metric that cannot be computed).

THE TABLE. METRICS below is the one place that says which line a figure is.
To drop a metric delete its row; to add one write a function that returns the
list of places counted and add a row. A row whose label ends in ".*" stands
for several detail lines: its function returns {part: (words, places)}, and
"{}" in its text is replaced by the words. A P<n>.frozen row follows the last
line of each principle listed in FROZEN_LINES.
"""
import sys

# Run as a script, Python puts this directory first on the module path, where
# a sibling script named like a standard module (warnings.py is one) would be
# imported in its place. Take it off before importing anything else.
if __name__ == "__main__" and sys.path and sys.path[0] == __import__("os").path.dirname(
        __import__("os").path.abspath(__file__)):
    del sys.path[0]

import ast
import fnmatch
import glob
import importlib.util
import json
import os
import re
import shutil
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

_spec = importlib.util.spec_from_file_location("deps", os.path.join(HERE, "deps.py"))
deps = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(deps)

# Whether the path check of --check (5) reads every line of AGENTS.md and
# README.md (True), or only their lines that mention docs/architecture,
# docs/adr or tools/architecture (False). True holds at the base commit: all
# their backticked paths and relative links exist there.
AGENTS_README_ALL_LINES = True

# The principles that get a P<n>.frozen line.
FROZEN_LINES = (1, 2, 3, 5, 6, 7, 9, 11)
FROZEN_TEXT = "of them in the stand-in cell's files, which a refactoring does not edit"


# ---------------------------------------------------------------- context


class Context:
    """What the metrics read, read once."""

    def __init__(self, root):
        self.root = root
        self.rules = deps.load_rules(root)
        self.result = deps.analyse(root, self.rules)
        self.files = self.result.files
        self.tracked = set(self.files)
        self._code = {}
        self._query = {}
        frozen_groups = {g["name"] for g in self.rules["groups"] if g.get("frozen")}
        self.frozen_files = {f for f, g in self.result.group_of.items() if g in frozen_groups}

    def code(self, path):
        """The file's text with comments and string literals blanked out."""
        if path not in self._code:
            self._code[path] = deps.strip_comments(deps.read(self.root, path))
        return self._code[path]

    def sources(self):
        return [f for f in self.files
                if f.startswith(("src/", "tests/")) and f.endswith((".h", ".cc"))]

    def source_files_of(self, expression):
        """The files, as repository paths, of the source files that a bazel
        query expression reaches. Run once per expression."""
        if expression not in self._query:
            out = subprocess.run(
                ["bazel", "query", "--noshow_progress", "--output=label",
                 'kind("source file", %s)' % expression],
                cwd=self.root, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            if out.returncode != 0:
                raise SystemExit("measure: bazel query failed (exit %d): %s"
                                 % (out.returncode, expression[:80]))
            self._query[expression] = {l[3:] for l in out.stdout.split() if l.startswith("//:")}
        return self._query[expression]

    def program_targets(self):
        """The targets that tools/architecture/programs.txt captures."""
        found = []
        for line in deps.read(self.root, "tools/architecture/programs.txt").split("\n"):
            if line.strip() and not line.startswith("#"):
                t = line.split("|")[1].strip()
                if t not in found:
                    found.append(t)
        return found

    def is_frozen(self, place):
        """Whether the file that a refactoring would edit for this place is
        frozen. The place's first word is a file (with :line or :line
        after it) or a target (//:name, all of whose files must be frozen)."""
        word = place.split(" ", 1)[0]
        if word.startswith("//:"):
            t = self.result.targets.get(word)
            files = (t["srcs"] + t["hdrs"]) if t else []
            return bool(files) and all(f in self.frozen_files for f in files)
        word = re.sub(r":[0-9]+(-[0-9]+)?$", "", word)
        return word in self.frozen_files


def matches_any(path, patterns):
    return any(fnmatch.fnmatchcase(path, p) for p in patterns)


# ---------------------------------------------------------------- metrics
# Each returns the sorted list of places counted, or None when the line is
# not to be printed at all.


def m_layering(c):
    return [deps.place("include", k) for k in sorted(c.result.found["include"])]


def m_cycles(c):
    return [deps.place("cycle", k) for k in sorted(c.result.found["cycle"])]


def opened_namespaces(code):
    """The namespaces a file opens, each by its full name (cell::layout
    when `namespace layout` is inside `namespace cell`); "(anonymous)" for
    an unnamed one."""
    opened, stack = [], []
    for m in re.finditer(r"\bnamespace(?:\s+([A-Za-z_][\w:]*))?\s*\{|\{|\}", code):
        t = m.group(0)
        if t == "}":
            if stack:
                stack.pop()
        elif t == "{":
            stack.append(None)
        else:
            outer = [x for x in stack if x]
            if m.group(1) is None:
                stack.append(outer[-1] if outer else None)
                opened.append("(anonymous)")
                continue
            name = m.group(1)
            full = "::".join(outer[-1:] + [name]) if outer else name
            stack.append(full)
            opened.append(full)
    return opened


def m_namespace(c):
    """Files under src/ that open a namespace their group does not allow
    (headers and .cc files; an unnamed namespace is allowed in a .cc file),
    and #define lines that carry the word namespace."""
    allowed = {g["name"]: set(g["namespaces"]) for g in c.rules["groups"]}
    out = []
    for f in c.sources():
        if not f.startswith("src/"):
            continue
        ok = allowed.get(c.result.group_of.get(f), set())
        if f.endswith(".cc"):
            ok = ok | {"(anonymous)"}
        bad = sorted({o for o in opened_namespaces(c.code(f)) if o not in ok})
        if bad:
            out.append("%s opens %s, which group %s does not allow"
                       % (f, ", ".join(bad), c.result.group_of.get(f)))
        for n, line in enumerate(c.code(f).split("\n"), 1):
            if re.match(r"\s*#\s*define\b.*\bnamespace\b", line):
                out.append("%s:%d a #define that holds the word namespace" % (f, n))
    return out


def m_untested(c):
    tested = c.source_files_of("deps(tests(//...))")
    return sorted(f for f in c.files if f.startswith("src/") and f not in tested)


def m_unheld(c):
    held = c.source_files_of("deps(tests(//...))") | c.source_files_of(
        "deps(set(%s))" % " ".join(c.program_targets()))
    return sorted(f for f in c.files if f.startswith("src/") and f not in held)


def m_unrun(c):
    captured = set(c.program_targets())
    return sorted(n for n, t in c.result.targets.items()
                  if t["kind"] == "cc_binary" and n not in captured)


def m_truth(c):
    t = c.rules["truth"]
    regexes = [re.compile(p["regex"], re.M) for p in t["patterns"]]
    allowed = [f for a in t["allowed"] for f in a["files"]]
    return files_matching(c, regexes, allowed)


def pinned(kw):
    """Whether a repository rule's or override's keywords pin what it fetches:
    an integrity (sha256- and base64), a sha256 (64 hex digits) or a commit
    (40 hex digits)."""
    return bool(re.fullmatch(r"sha256-[A-Za-z0-9+/]{43}=", str(kw.get("integrity") or ""))
                or re.fullmatch(r"[0-9a-f]{64}", str(kw.get("sha256") or ""))
                or re.fullmatch(r"[0-9a-f]{40}", str(kw.get("commit") or "")))


def module_bazel(c):
    """[(kind, name, pinned)] read from MODULE.bazel: bazel_dep (pinned by a
    version), fetch (a repository rule), override (a *_override) and use_repo
    (a name taken from a module extension)."""
    tree = ast.parse(deps.read(c.root, "MODULE.bazel"), "MODULE.bazel")
    rules, found = set(), []
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Call) \
                and getattr(node.value.func, "id", None) == "use_repo_rule":
            rules.add(node.targets[0].id)
            continue
        if not (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)):
            continue
        call = node.value
        fn = getattr(call.func, "id", None)
        kw = {}
        for k in call.keywords:
            try:
                kw[k.arg] = ast.literal_eval(k.value)
            except ValueError:
                kw[k.arg] = None
        if fn == "bazel_dep":
            found.append(("bazel_dep", kw.get("name"), bool(kw.get("version"))))
        elif fn in rules:
            found.append(("fetch", kw.get("name"), pinned(kw)))
        elif fn == "use_repo":
            names = [a.value for a in call.args[1:] if isinstance(a, ast.Constant)]
            names += [v for v in kw.values() if isinstance(v, str)]
            found += [("use_repo", n, True) for n in names]
        elif fn and fn.endswith("_override"):
            found.append(("override", "%s %s" % (fn, kw.get("module_name")),
                          pinned(kw) or bool(kw.get("version"))))
    return found


def m_outside(c):
    return sorted("%s %s" % (k, n) for k, n, _ in module_bazel(c))


def m_outside_unpinned(c):
    return sorted("%s %s" % (k, n) for k, n, ok in module_bazel(c) if not ok)


def targets_listing(lib):
    def m(c):
        out = []
        for name, t in sorted(c.result.targets.items()):
            if t["kind"] == "alias":
                continue
            if any(deps.library_of_label(c.rules, c.result.targets, d) == ("outside", lib)
                   for d in t["deps"]):
                out.append(name)
        return out
    return m


def m_outside_mujoco_files(c):
    return sorted(f for f, incs in c.result.file_includes.items()
                  if any(k == "<" and n.startswith("mujoco/") for _, k, n in incs))


def m_outside_group(c):
    """The #include lines of an outside library from a group that may not,
    and the files that reach such a library through project headers."""
    out = []
    for f, lib in sorted(c.result.found["outside_include"]):
        lines = [line for line, kind, name in c.result.file_includes[f]
                 if kind == "<" and deps.outside_library(c.rules, name) == lib]
        for line in lines:
            out.append("%s:%d %s" % (f, line, lib))
        if not lines:
            out.append("%s %s (through its project headers)" % (f, lib))
    return out


def m_unused_project(c):
    return [deps.place("unused_dep", k) for k in sorted(c.result.found["unused_dep"])]


def m_unused_outside(c):
    return [deps.place("unused_outside_dep", k)
            for k in sorted(c.result.found["unused_outside_dep"])]


def m_missing(c):
    return [deps.place("missing_dep", k) for k in sorted(c.result.found["missing_dep"])]


def m_target_files(c):
    """Files under src/ and tests/ that are in the srcs and hdrs of no target
    or of more than one. The targets that one comprehension of BUILD.bazel
    makes from one source count as one."""
    homes = {}
    for name, t in c.result.targets.items():
        if t["kind"] == "alias":
            continue
        for f in set(t["srcs"]) | set(t["hdrs"]):
            homes.setdefault(f, {}).setdefault(t["family"], []).append(name)
    out = []
    for f in c.sources():
        fams = homes.get(f, {})
        if len(fams) == 0:
            out.append("%s is in the srcs and hdrs of no target" % f)
        elif len(fams) > 1:
            names = sorted(n for ns in fams.values() for n in ns)
            out.append("%s is in the srcs or hdrs of %d targets: %s" % (f, len(fams), " ".join(names)))
    return out


def m_dep_places(c):
    return m_unused_project(c) + m_unused_outside(c) + m_missing(c) + m_target_files(c)


def files_matching(c, regexes, allowed, scope=("src/*", "tests/*")):
    out = []
    for f in c.sources():
        if not matches_any(f, scope) or matches_any(f, allowed):
            continue
        code = c.code(f)
        if any(r.search(code) for r in regexes):
            out.append(f)
    return sorted(out)


def clock_printed_only(c):
    return [e["file"] for e in c.rules["portable"]["clock_printed_only"]]


def m_clock(c):
    """Files that read steady_clock for more than a print."""
    r = re.compile(r"\bsteady_clock\b")
    return files_matching(c, [r], clock_printed_only(c))


def m_portable(c):
    found = set(m_clock(c))
    for p in c.rules["portable"]["patterns"]:
        r = re.compile(p["regex"], re.M)
        found |= set(files_matching(c, [r], [a["file"] for a in p["allowed"]]))
    return sorted(found)


def m_single_home_parts(c):
    """{key: places} for each entry of deps.json single_home."""
    parts = {}
    for e in c.rules["single_home"]["entries"]:
        r = re.compile(e["pattern"], re.M)
        parts[e["key"]] = (e["fact"], files_matching(c, [r], e["home"], scope=("src/*",)))
    return parts


def m_single_home(c):
    return ["%s (%s)" % (f, key) for key, (_, files) in m_single_home_parts(c).items()
            for f in files]


def expand(pattern):
    """Brace expansion ({a,b}) of one pattern, for generated's globs."""
    m = re.search(r"\{([^{}]*)\}", pattern)
    if not m:
        return [pattern]
    return [x for alt in m.group(1).split(",")
            for x in expand(pattern[:m.start()] + alt + pattern[m.end():])]


def test_compares(c, e):
    """Whether the entry's test is a file under tests/ that names the
    committed file's base name, or whose target's data covers it."""
    t = e.get("test")
    if not t or not t.startswith("tests/") or t not in c.tracked:
        return False
    pats = expand(e["committed"])
    text = deps.read(c.root, t)
    if any(os.path.basename(p) in text for p in pats):
        return True
    for tgt in c.result.targets.values():
        if t in tgt["srcs"]:
            for d in tgt["data"] if isinstance(tgt["data"], list) else []:
                if any(fnmatch.fnmatchcase(d, p) for p in pats):
                    return True
    return False


def m_generated(c):
    return sorted(e["committed"] for e in c.rules["generated"]["entries"]
                  if not test_compares(c, e) and not e.get("no_test"))


def m_no_test(c):
    return sorted(e["committed"] for e in c.rules["generated"]["entries"] if e.get("no_test"))


def m_generated_unmatched(c):
    """Entries whose committed pattern matches no tracked file (the data is
    stale), tracked files under plans/ that no entry matches, and the
    models/*.xml that a generator writes and no entry matches."""
    entries = c.rules["generated"]["entries"]
    pats = [p for e in entries for p in expand(e["committed"])]
    out = []
    for e in entries:
        if not any(any(fnmatch.fnmatchcase(f, p) for f in c.files) for p in expand(e["committed"])):
            out.append(e["committed"])
    for f in c.files:
        if f.startswith("plans/") and not any(fnmatch.fnmatchcase(f, p) for p in pats):
            out.append("%s matches no entry of generated" % f)
    for e in entries:
        for m in re.finditer(r"models/[A-Za-z0-9_./-]+\.xml", e["generator"]):
            if not any(fnmatch.fnmatchcase(m.group(0), p) for p in pats):
                out.append("%s, written by %s, matches no entry of generated" % (m.group(0), e["generator"]))
    return out


_SILENCE_CODE = [
    re.compile(r"^[ \t]*#[ \t]*pragma[ \t]+(?:GCC|clang)[ \t]+diagnostic\b", re.M),
    re.compile(r"\b_Pragma\s*\("),
    re.compile(r"^[ \t]*#[ \t]*pragma[ \t]+(?:GCC|clang)[ \t]+system_header\b", re.M),
    re.compile(r"__attribute__\s*\(\(\s*no_sanitize"),
    re.compile(r"\[\[\s*(?:gnu|clang)::no_sanitize"),
]
# NOLINT is itself a comment, and clang-tidy honours it anywhere in one.
_COMMENT = re.compile(r"//[^\n]*|/\*.*?\*/", re.S)
_NOLINT = re.compile(r"NOLINT(?:NEXTLINE|BEGIN)?\b")
_BAZELRC_FLAG = re.compile(r"(?<![\w-])(-Wno-[\w=+-]*|-w|-isystem)(?![\w-])")


def m_silenced(c):
    out = []
    for f in c.sources():
        raw = deps.read(c.root, f)
        for cm in _COMMENT.finditer(raw):
            for m in _NOLINT.finditer(cm.group(0)):
                out.append("%s:%d NOLINT" % (f, raw.count("\n", 0, cm.start() + m.start()) + 1))
        code = c.code(f)
        for r in _SILENCE_CODE:
            for m in r.finditer(code):
                out.append("%s:%d %s" % (f, code.count("\n", 0, m.start()) + 1,
                                         m.group(0).strip()))
    for name, t in sorted(c.result.targets.items()):
        for o in t["copts"]:
            if o == "-w" or o.startswith(("-Wno-", "-isystem")):
                out.append("BUILD.bazel:%d %s copts %s" % (t["line"], name, o))
    budget = {}
    for a in c.rules["silenced"]["allowed"]:
        budget[(a["file"], a["token"])] = budget.get((a["file"], a["token"]), 0) + 1
    if os.path.exists(os.path.join(c.root, ".bazelrc")):
        for n, line in enumerate(deps.read(c.root, ".bazelrc").split("\n"), 1):
            if line.lstrip().startswith("#"):
                continue
            for m in _BAZELRC_FLAG.finditer(line):
                key = (".bazelrc", m.group(1))
                if budget.get(key, 0) > 0:
                    budget[key] -= 1
                else:
                    out.append(".bazelrc:%d %s" % (n, m.group(1)))
    return sorted(out, key=lambda p: (p.split(":")[0], int(re.match(r"[^:]*:(\d+)", p).group(1)), p))


def line_count(c, f):
    return deps.read(c.root, f).count("\n")


def m_size_cc(c):
    n = c.rules["size"]["cc_lines"]
    return ["%s %d" % (f, line_count(c, f)) for f in c.files
            if f.startswith("src/") and f.endswith(".cc") and line_count(c, f) > n]


def m_size_h(c):
    n = c.rules["size"]["h_lines"]
    return ["%s %d" % (f, line_count(c, f)) for f in c.files
            if f.startswith("src/") and f.endswith(".h") and line_count(c, f) > n]


def opening_comment_lines(text):
    """Lines of the comment block before a header's first line of code."""
    count, in_block = 0, False
    for line in text.split("\n"):
        s = line.strip()
        if in_block:
            count += 1
            if "*/" in s:
                in_block = False
            continue
        if not s or s.startswith("#pragma once") or re.match(r"#\s*(ifndef|define)\s+\w+_H_?\w*\s*$", s):
            continue
        if s.startswith("//"):
            count += 1
            continue
        if s.startswith("/*"):
            count += 1
            in_block = "*/" not in s
            continue
        break
    return count


def m_size_comment(c):
    n = c.rules["size"]["opening_comment_lines"]
    out = []
    for f in c.files:
        if f.startswith("src/") and f.endswith(".h"):
            k = opening_comment_lines(deps.read(c.root, f))
            if k < n:
                out.append("%s %d" % (f, k))
    return out


def m_size(c):
    return m_size_cc(c) + m_size_h(c) + m_size_comment(c)


def doc_files(root, left_out=()):
    """AGENTS.md, README.md and every docs/**/*.md that exists, less the
    documents deps.json leaves out."""
    found = [d for d in ("AGENTS.md", "README.md") if os.path.exists(os.path.join(root, d))]
    for base, dirs, names in os.walk(os.path.join(root, "docs")):
        dirs.sort()
        for n in sorted(names):
            if n.endswith(".md"):
                found.append(os.path.relpath(os.path.join(base, n), root))
    return [d for d in found if d not in left_out]


_LABEL = re.compile(r"(?<![\w/@])//:([A-Za-z0-9_][A-Za-z0-9_./+-]*[A-Za-z0-9_])")


_STARTER = re.compile(r"\s*(?:[-*+]\s|\d+[.)]\s|\||#|>)")
_GONE = re.compile(r"\(gone\)|no longer exists?", re.I)


def doc_units(text):
    """(line, text, fenced) for each paragraph (its lines joined, with a
    list of the offset at which each line starts) outside fenced blocks, and
    for each line inside one. A new paragraph starts at a blank line, a
    table row, a heading, a quotation or a list item."""
    lines = text.split("\n")
    fence, para, starts = None, [], []
    units = []

    def flush():
        if para:
            units.append((starts[0][1], " ".join(para), False, [(o, n) for o, n in starts]))
        para.clear()
        starts.clear()

    for n, line in enumerate(lines, 1):
        m = re.match(r"^\s*(```|~~~)", line)
        if m:
            flush()
            fence = None if fence == m.group(1) else (fence or m.group(1))
            continue
        if fence:
            units.append((n, line, True, [(0, n)]))
            continue
        if not line.strip():
            flush()
            continue
        if _STARTER.match(line):
            flush()
        offset = len(" ".join(para)) + (1 if para else 0)
        starts.append((offset, n))
        para.append(line.strip())
    flush()
    return units


def line_at(starts, offset):
    n = starts[0][1]
    for o, ln in starts:
        if o <= offset:
            n = ln
    return n


def sentence_around(text, start, end):
    """The sentence of text that holds [start, end)."""
    lo = 0
    hi = len(text)
    for m in re.finditer(r"(?<=[.!?])\s+", text):
        if m.end() <= start:
            lo = m.end()
        elif m.start() >= end:
            hi = m.start()
            break
    return text[lo:hi]


def token_departures(c, token, tops):
    """(token, reason) for each path word and Bazel label of the text."""
    out = []
    for word in token.split():
        w = word.strip("\"'(),;").rstrip(".:")
        why = check_path(c.root, w, tops)
        if why:
            out.append((w, why))
    for lm in _LABEL.finditer(token):
        name = lm.group(1)
        if "//:" + name not in c.result.rule_names and name not in c.tracked:
            out.append(("//:" + name, "is no target of BUILD.bazel"))
    return out


def doc_departures(c, doc, tops):
    """(line, token, reason) for each backticked path or Bazel label of a
    document that does not exist, in paragraphs, inside multi-word spans and
    in fenced blocks, and each relative link to a file that does not exist.
    A token is skipped when " (new)" or " (gone)" follows its span, or when
    its sentence says "(gone)" or "no longer exists"."""
    out = []
    for first, text, fenced, starts in doc_units(deps.read(c.root, doc)):
        if fenced:
            if _GONE.search(text):
                continue
            out += [(first, t, why) for t, why in token_departures(c, text, tops)]
            continue
        for m in _BACKTICK.finditer(text):
            if text[m.end():m.end() + 7].startswith((" (new)", " (gone)")):
                continue
            if _GONE.search(sentence_around(text, m.start(), m.end())):
                continue
            line = line_at(starts, m.start())
            out += [(line, t, why) for t, why in token_departures(c, m.group(2).strip(), tops)]
        bare = _BACKTICK.sub(lambda x: " " * len(x.group(0)), text)
        for m in _LINK.finditer(bare):
            target = m.group(1)
            if re.match(r"^[a-z][a-z0-9+.-]*:", target, re.I) or target.startswith("#"):
                continue
            target = target.split("#", 1)[0]
            if not target or _GONE.search(sentence_around(text, m.start(), m.end())):
                continue
            full = os.path.normpath(os.path.join(c.root, os.path.dirname(doc), target))
            if not os.path.exists(full):
                out.append((line_at(starts, m.start()), target, "link target does not exist"))
    return sorted(set(out))


def m_documents(c):
    left = {d["file"] for d in c.rules["docs_left_out"]["list"]}
    tops = top_level(c.root)
    return ["%s:%d %s" % (doc, n, token)
            for doc in doc_files(c.root, left)
            for n, token, _ in doc_departures(c, doc, tops)]


def documents_by_file(root):
    """{document: count} for every document, those left out included."""
    c = Context(root)
    tops = top_level(root)
    return {doc: len(doc_departures(c, doc, tops)) for doc in doc_files(root)}


def m_allowed_pairs(c):
    return ["%s may include %s" % (g["name"], m) for g in c.rules["groups"] for m in g["may_include"]]


def m_listed(c):
    return [deps.place(d["kind"], deps.departure_key(d)) for d in c.rules["departures"]["list"]]


def m_frozen_files(c):
    return sorted(c.frozen_files)


def m_truth_allowed(c):
    names = set()
    for a in c.rules["truth"]["allowed"]:
        for pat in a["files"]:
            names |= {f for f in c.files if fnmatch.fnmatchcase(f, pat)}
    return sorted(names)


def m_outside_pairs(c):
    out = ["%s may include %s" % (g["name"], lib) for g in c.rules["groups"] for lib in g["outside"]]
    return out + ["tests may include %s" % lib for lib in c.rules["tests_outside"]]


def m_portable_allowed(c):
    out = ["%s (%s)" % (a["file"], p["name"]) for p in c.rules["portable"]["patterns"]
           for a in p["allowed"]]
    return out + ["%s (steady_clock only printed)" % f for f in clock_printed_only(c)]


def m_silenced_allowed(c):
    return ["%s %s" % (a["file"], a["token"]) for a in c.rules["silenced"]["allowed"]]


def m_left_out(c):
    return [d["file"] for d in c.rules["docs_left_out"]["list"]]


def clang_tidy_entries(text, key):
    """The comma-separated entries of a key of .clang-tidy (a plain, quoted
    or folded value, over one or several lines)."""
    m = re.search(r"^%s:[ \t]*([^\n]*)((?:\n[ \t]+[^\n]*)*)" % key, text, re.M)
    if not m:
        return []
    value = (m.group(1) + " " + m.group(2)).strip().lstrip(">|-+").strip().strip("'\"")
    return [e.strip().strip("'\"") for e in re.split(r"[,\n]", value) if e.strip().strip("'\"")]


def m_held(c):
    out = []
    rc = os.path.join(c.root, ".bazelrc")
    if os.path.exists(rc):
        for n, line in enumerate(deps.read(c.root, ".bazelrc").split("\n"), 1):
            if line.startswith("build --per_file_copt="):
                out += [".bazelrc:%d %s" % (n, f) for f in re.findall(r"-Werror=[\w+-]+", line)]
    ct = os.path.join(c.root, ".clang-tidy")
    if os.path.exists(ct):
        text = deps.read(c.root, ".clang-tidy")
        out += [".clang-tidy Checks %s" % e for e in clang_tidy_entries(text, "Checks")
                if not e.startswith("-")]
        out += [".clang-tidy WarningsAsErrors %s" % e
                for e in clang_tidy_entries(text, "WarningsAsErrors")]
    return out


GUARD = " (a rule of deps.json; may not grow)"

# The table. label, what is counted, the function that lists the places,
# whether it needs `bazel query`.
METRICS = [
    ("P1", "file-to-header pairs (a file and a project header it includes) against the groups' allowed direction (deps.json)", m_layering, False),
    ("P1.cycles", "header cycles", m_cycles, False),
    ("P1.namespace", "files under src/ that open a namespace their group does not allow (deps.json namespaces), and #define lines with the word namespace", m_namespace, False),
    ("P1.allowed", "pairs (group, group it may include)" + GUARD, m_allowed_pairs, False),
    ("P1.listed", "departures on the list of deps.json" + GUARD, m_listed, False),
    ("P1.frozen_files", "files in frozen groups" + GUARD, m_frozen_files, False),
    ("P2", "files under src/ that no test and no captured program (programs.txt) depends on (bazel)", m_unheld, True),
    ("P2.untested", "files under src/ that no test target depends on (bazel)", m_untested, True),
    ("P2.unrun", "cc_binary targets that programs.txt does not capture", m_unrun, False),
    ("P3", "files that read the simulation's truth and are not allowed to (deps.json truth)", m_truth, False),
    ("P3.allowed", "files that may read the truth, by exact name" + GUARD, m_truth_allowed, False),
    ("P4", "outside dependencies in MODULE.bazel (bazel_dep, fetches, use_repo names, overrides; may not grow)", m_outside, False),
    ("P4.unpinned", "of them not pinned by a version, a commit, an integrity or a sha256", m_outside_unpinned, False),
    ("P4.mujoco", "targets that list @mujoco directly (may not grow)", targets_listing("mujoco"), False),
    ("P4.eigen", "targets that list @eigen directly (may not grow)", targets_listing("eigen"), False),
    ("P4.googletest", "targets that list @googletest directly (may not grow)", targets_listing("googletest"), False),
    ("P4.nlohmann_json", "targets that list @nlohmann_json directly (may not grow)", targets_listing("nlohmann_json"), False),
    ("P4.ompl", "targets that list @ompl directly (may not grow)", targets_listing("ompl"), False),
    ("P4.glfw", "targets that list @glfw directly (may not grow)", targets_listing("glfw"), False),
    ("P4.mujoco_files", "files that include mujoco/ (may not grow)", m_outside_mujoco_files, False),
    ("P4.group", "#include lines of an outside library from a group that may not include it, and files that reach one through project headers", m_outside_group, False),
    ("P4.allowed", "pairs (group, outside library)" + GUARD, m_outside_pairs, False),
    ("P5", "unused and missing deps of targets, and files in the srcs and hdrs of no target or of several", m_dep_places, False),
    ("P5.unused_project", "targets' deps on project libraries none of whose headers the target includes", m_unused_project, False),
    ("P5.unused_outside", "targets' deps on outside libraries none of whose headers the target includes", m_unused_outside, False),
    ("P5.missing", "targets that include a header of a library they do not list", m_missing, False),
    ("P5.files", "files under src/ and tests/ in the srcs and hdrs of no target or of more than one", m_target_files, False),
    ("P6", "files that use what differs between platforms or runs outside src/portable_random.h, or that read steady_clock for more than a print", m_portable, False),
    ("P6.clock", "files that read steady_clock and are not in clock_printed_only of deps.json", m_clock, False),
    ("P6.allowed", "files excused from a pattern or from the clock reading" + GUARD, m_portable_allowed, False),
    ("P7", "places in src/ where a fact that has a home is written again (file and fact; deps.json single_home)", m_single_home, False),
    ("P7.*", "files under src/ outside its home that write {}", m_single_home_parts, False),
    ("P8", "committed generated files with no test that compares them to their source and no reason why none can", m_generated, False),
    ("P8.unmatched", "generated entries that match no tracked file, and plans and models that match no entry", m_generated_unmatched, False),
    ("P8.no_test", "entries excused from a test" + GUARD, m_no_test, False),
    ("P9", "places that silence a compiler's or analyzer's finding (deps.json silenced)", m_silenced, False),
    ("P9.allowed", "silencers excused" + GUARD, m_silenced_allowed, False),
    ("P9.held", "-Werror= flags in .bazelrc plus enabled entries of Checks and WarningsAsErrors in .clang-tidy (a guard; may not fall)", m_held, False),
    ("P10", "backticked paths and Bazel labels, and relative links, in AGENTS.md, README.md and docs/**/*.md that do not exist", m_documents, False),
    ("P10.left_out", "documents left out of P10" + GUARD, m_left_out, False),
    ("P11", "files under src/ too long, and headers whose opening comment is too short", m_size, False),
    ("P11.cc", ".cc files under src/ over the line limit of deps.json size", m_size_cc, False),
    ("P11.h", "headers under src/ over the line limit of deps.json size", m_size_h, False),
    ("P11.comment", "headers whose opening comment is shorter than deps.json size says", m_size_comment, False),
]


def principle_of(label):
    return int(re.match(r"P(\d+)", label).group(1))


def validate_table():
    seen = set()
    for label, _, _, _ in METRICS:
        if label in seen:
            raise SystemExit("measure.py: %s is printed twice" % label)
        seen.add(label)


def measure(c, only=None, notes=True):
    """[(label, figure, text, places)] for the lines to print."""
    have_bazel = shutil.which("bazel") is not None
    rows = []
    skipped = set()
    for label, text, fn, bazel in METRICS:
        if only is not None and label not in only:
            continue
        if bazel and not have_bazel:
            skipped.add(label)
            if notes:
                sys.stderr.write("measure: %s left out: bazel is not on PATH\n" % label)
            continue
        places = fn(c)
        if places is None:
            continue
        if label.endswith(".*"):
            for part, (words, ps) in places.items():
                rows.append((label[:-1] + part, len(ps), text.replace("{}", words), ps))
            continue
        rows.append((label, len(places), text, places))
    # P<n>.frozen after the last line of each principle.
    out = []
    for i, row in enumerate(rows):
        out.append(row)
        p = principle_of(row[0])
        last = i + 1 == len(rows) or principle_of(rows[i + 1][0]) != p
        if p in FROZEN_LINES and last:
            head = [r for r in rows if r[0] == "P%d" % p]
            if head:
                ps = [x for x in head[0][3] if c.is_frozen(x)]
                bz = " (bazel)" if head[0][2].endswith("(bazel)") else ""
                out.append(("P%d.frozen" % p, len(ps), FROZEN_TEXT + bz, ps))
    return out


def format_row(row):
    return "%s %d %s" % (row[0], row[1], row[2])


# ---------------------------------------------------------------- --check

DOC_GLOBS = ["docs/architecture/*.md", "docs/adr/*.md"]
NEEDED = ["docs/architecture/baseline.txt", "docs/architecture/research.md",
          "docs/architecture/principles.md", "docs/architecture/backlog.md"]
MARK = "# not compared"


def compared_part(text):
    out = []
    for line in text.split("\n"):
        if line.startswith(MARK):
            break
        s = line.rstrip()
        if s and not s.startswith("#"):
            out.append(s)
    return out


def markdown_lines(text):
    """(line number, line) outside fenced code blocks."""
    fence = None
    for i, line in enumerate(text.split("\n"), 1):
        m = re.match(r"^\s*(```|~~~)", line)
        if m:
            fence = None if fence == m.group(1) else (fence or m.group(1))
            continue
        if fence is None:
            yield i, line


def top_level(root):
    """Tracked top-level directories (with '/') and files."""
    names = set()
    for f in deps.tracked_files(root):
        names.add(f.split("/", 1)[0] + "/" if "/" in f else f)
    return names


def check_path(root, token, tops):
    """None when the backticked token is fine or no path; else a reason."""
    if any(ch in token for ch in "<>${} "):
        return None
    if not (any(token.startswith(t) for t in tops if t.endswith("/")) or token in tops):
        return None
    m = re.match(r"^(.*?):(\d+)(?:-(\d+))?$", token)
    path, first, last = (m.group(1), int(m.group(2)), int(m.group(3) or m.group(2))) if m else (token, None, None)
    full = os.path.join(root, path)
    if any(ch in path for ch in "*?["):
        if not glob.glob(full, recursive=True):
            return "matches no file"
        return None
    if not os.path.exists(full):
        return "does not exist"
    if first is not None:
        if os.path.isdir(full):
            return "is a directory, not a file with lines"
        with open(full, encoding="utf-8", errors="replace") as f:
            n = sum(1 for _ in f)
        if not 1 <= first <= last <= n:
            return "lines %d-%d are not within its %d lines" % (first, last, n)
    return None


_BACKTICK = re.compile(r"(`+)(.+?)\1")
_LINK = re.compile(r"\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")


def path_problems(root, doc, tops, only_lines_with=None):
    out = []
    text = deps.read(root, doc)
    for n, line in markdown_lines(text):
        if only_lines_with and not any(w in line for w in only_lines_with):
            continue
        for m in _BACKTICK.finditer(line):
            token = m.group(2).strip()
            after = line[m.end():m.end() + 7]
            if after.startswith((" (new)", " (gone)")):
                continue
            why = check_path(root, token, tops)
            if why:
                out.append("%s:%d: `%s` %s" % (doc, n, token, why))
        for m in _LINK.finditer(_BACKTICK.sub(lambda x: " " * len(x.group(0)), line)):
            target = m.group(1)
            if re.match(r"^[a-z][a-z0-9+.-]*:", target, re.I) or target.startswith("#"):
                continue
            target = target.split("#", 1)[0]
            if not target:
                continue
            full = os.path.normpath(os.path.join(root, os.path.dirname(doc), target))
            if not os.path.exists(full):
                out.append("%s:%d: link (%s) does not exist" % (doc, n, m.group(1)))
    return out


def docs_problems(root):
    out = []
    for need in NEEDED:
        if not os.path.exists(os.path.join(root, need)):
            out.append("%s: does not exist yet" % need)
    if not glob.glob(os.path.join(root, "docs/adr/*.md")):
        out.append("docs/adr/: holds no decision record yet")
    docs = sorted({os.path.relpath(p, root) for g in DOC_GLOBS
                   for p in glob.glob(os.path.join(root, g))})

    def read_or_empty(path):
        full = os.path.join(root, path)
        return deps.read(root, path) if os.path.exists(full) else ""

    research = read_or_empty("docs/architecture/research.md")
    defined_s = set(re.findall(r"^(?:- |#+ )?\[(S[0-9]+)\]", research, re.M))
    principles = set(re.findall(r"^## (P[0-9]+)\b", read_or_empty("docs/architecture/principles.md"), re.M))
    backlog = set(re.findall(r"^## (B[0-9]+)\b", read_or_empty("docs/architecture/backlog.md"), re.M))
    records = {os.path.basename(p)[:4] for p in glob.glob(os.path.join(root, "docs/adr/[0-9][0-9][0-9][0-9]-*.md"))}
    have = {"research": os.path.exists(os.path.join(root, "docs/architecture/research.md")),
            "principles": os.path.exists(os.path.join(root, "docs/architecture/principles.md")),
            "backlog": os.path.exists(os.path.join(root, "docs/architecture/backlog.md"))}
    tops = top_level(root)
    for doc in docs:
        text = deps.read(root, doc)
        for n, line in enumerate(text.split("\n"), 1):
            if have["research"]:
                for s in re.findall(r"\[(S[0-9]+)\]", line):
                    if s not in defined_s:
                        out.append("%s:%d: cites [%s], which docs/architecture/research.md does "
                                   "not define" % (doc, n, s))
            if have["principles"]:
                for p in re.findall(r"\b(P[0-9]+)\b", line):
                    if p not in principles:
                        out.append("%s:%d: cites %s, which is no '## %s' heading of "
                                   "docs/architecture/principles.md" % (doc, n, p, p))
            if have["backlog"]:
                for b in re.findall(r"\b(B[0-9]+)\b", line):
                    if b not in backlog:
                        out.append("%s:%d: cites %s, which is no '## %s' heading of "
                                   "docs/architecture/backlog.md" % (doc, n, b, b))
            for r in re.findall(r"\bADR[ -]?([0-9]{4})\b", line) + \
                    re.findall(r"[Ss]uperseded by (?:ADR[ -]?)?([0-9]{4})\b", line):
                if r not in records:
                    out.append("%s:%d: cites record %s, but there is no docs/adr/%s-*.md"
                               % (doc, n, r, r))
        out += path_problems(root, doc, tops)
    words = None if AGENTS_README_ALL_LINES else ["docs/architecture", "docs/adr", "tools/architecture"]
    for doc in ("AGENTS.md", "README.md"):
        if os.path.exists(os.path.join(root, doc)):
            out += path_problems(root, doc, tops, words)
    return out


def run_check(root, strict_items=False):
    problems = []
    dep_problems, _ = deps.check(root, strict_items)
    problems += ["deps.py: " + p for p in dep_problems]
    c = Context(root)
    have_bazel = shutil.which("bazel") is not None
    now = [format_row(r) for r in measure(c)]
    base = os.path.join(root, "docs/architecture/baseline.txt")
    if os.path.exists(base):
        want = compared_part(deps.read(root, "docs/architecture/baseline.txt"))
        if not have_bazel:
            want = [l for l in want if "(bazel)" not in l]
            now = [l for l in now if "(bazel)" not in l]
        if want != now:
            key = lambda l: l.split(" ", 1)[0]
            wmap, nmap = {key(l): l for l in want}, {key(l): l for l in now}
            for k in sorted(set(wmap) | set(nmap), key=lambda k: (want + now).index(wmap.get(k) or nmap[k])):
                if wmap.get(k) != nmap.get(k):
                    problems.append("baseline.txt: %s; now: %s"
                                    % (wmap.get(k, "(no line)"), nmap.get(k, "(no line)")))
            if not problems or all(not p.startswith("baseline.txt") for p in problems):
                problems.append("baseline.txt: the compared lines are in another order than the output")
    problems += docs_problems(root)
    backlog_path = os.path.join(root, "docs/architecture/backlog.md")
    if os.path.exists(backlog_path):
        backlog = set(re.findall(r"^## (B[0-9]+)\b", deps.read(root, "docs/architecture/backlog.md"), re.M))
        for d in c.rules["departures"]["list"]:
            if re.match(r"^B[0-9]+$", d.get("item", "")) and d["item"] not in backlog:
                problems.append("deps.json: the departure %s names %s, which is no '## %s' heading "
                                "of docs/architecture/backlog.md"
                                % (deps.place(d["kind"], deps.departure_key(d)), d["item"], d["item"]))
    return problems


# ---------------------------------------------------------------- main


def main(argv):
    validate_table()
    if len(argv) == 1:
        c = Context(ROOT)
        for row in measure(c):
            print(format_row(row))
        return 0
    if argv[1] == "--list" and len(argv) == 3:
        want = argv[2]
        c = Context(ROOT)
        rows = [r for r in measure(c, notes=False) if r[0] == want]
        if not rows:
            sys.stderr.write("measure: no line %s; the lines are: %s\n"
                             % (want, " ".join(r[0] for r in measure(c, notes=False))))
            return 2
        for place in rows[0][3]:
            print(place)
        return 0
    if argv[1] == "--check" and len(argv) in (2, 3) and (len(argv) == 2 or argv[2] == "--strict-items"):
        problems = run_check(ROOT, len(argv) == 3)
        for p in problems:
            print(p)
        if not problems:
            print("ok")
        return 1 if problems else 0
    sys.stderr.write(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
