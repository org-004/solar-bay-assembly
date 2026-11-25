#!/usr/bin/env python3
r"""measure.py [--list P<n>[.<part>] | --check]   (run it as tools/architecture/measure.sh)

Measures how far the repository departs from its design principles
(docs/architecture/principles.md), one figure per principle that a command
can check. Every figure is a count of departures: lower is better, and the
target is 0 or "may not grow". The rules and the data the counts need are in
tools/architecture/deps.json; the dependency check itself is
tools/architecture/deps.py, which this script imports.

Without arguments it prints one headline line per principle,
    P<n> <figure> <what is counted>
and, where a principle has parts, detail lines
    P<n>.<part> <figure> <what is counted>
A line that needs `bazel query` ends in "(bazel)"; when bazel is not on PATH
it is left out and a note goes to stderr.

  --list P<n>          prints the places counted by the headline of P<n>,
  --list P<n>.<part>   or by one detail line, one per line (a metric's key,
                       such as declared.project, works as well).
  --check              exits 0 only when all of these hold, and otherwise
                       prints what does not hold, one line each, and exits 1:
     1. tools/architecture/deps.py passes;
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
seconds without Bazel. Exit codes: 0 (printed, or --check passed), 1
(--check found something), 2 (usage error, or a metric that cannot be
computed).

THE TABLE. METRICS below is the one place that says which principle a figure
belongs to. To renumber a principle change its number there; to drop a
metric delete its row; to add one write a function that returns the list of
places counted and add a row. A row whose key has a dot is a detail line of
the headline row with the key before the dot. A row whose key ends in ".*"
stands for several detail lines: its function returns {part: (words,
places)}, and "{}" in its text is replaced by the words.
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


# ---------------------------------------------------------------- context


class Context:
    """What the metrics read, read once."""

    def __init__(self, root):
        self.root = root
        self.rules = deps.load_rules(root)
        self.result = deps.analyse(root, self.rules)
        self.files = self.result.files
        self._code = {}

    def code(self, path):
        """The file's text with comments and string literals blanked out."""
        if path not in self._code:
            self._code[path] = deps.strip_comments(deps.read(self.root, path))
        return self._code[path]

    def sources(self):
        return [f for f in self.files
                if f.startswith(("src/", "tests/")) and f.endswith((".h", ".cc"))]


def matches_any(path, patterns):
    return any(fnmatch.fnmatchcase(path, p) for p in patterns)


# ---------------------------------------------------------------- metrics
# Each returns the sorted list of places counted, or None when the line is
# not to be printed at all.


def m_layering(c):
    return [deps.place("include", k) for k in sorted(c.result.found["include"])]


def m_declared(c):
    return m_declared_project(c) + m_declared_outside(c)


def m_declared_project(c):
    return [deps.place("unused_dep", k) for k in sorted(c.result.found["unused_dep"])]


def m_declared_outside(c):
    return [deps.place("unused_outside_dep", k)
            for k in sorted(c.result.found["unused_outside_dep"])]


def module_bazel(c):
    """(bazel_deps, fetches): [(name, pinned by)] read from MODULE.bazel."""
    tree = ast.parse(deps.read(c.root, "MODULE.bazel"), "MODULE.bazel")
    rules, bazel_deps, fetches = set(), [], []
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
            bazel_deps.append((kw.get("name"), kw.get("version") or ""))
        elif fn in rules:
            pin = kw.get("sha256") or kw.get("integrity") or kw.get("commit") or ""
            fetches.append((kw.get("name"), pin))
    return bazel_deps, fetches


def m_outside(c):
    bazel_deps, fetches = module_bazel(c)
    return sorted(["bazel_dep " + n for n, _ in bazel_deps] +
                  ["fetch " + n for n, _ in fetches])


def m_outside_unpinned(c):
    bazel_deps, fetches = module_bazel(c)
    return sorted(["bazel_dep " + n for n, v in bazel_deps if not v] +
                  ["fetch " + n for n, p in fetches if not p])


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


def files_matching(c, regexes, allowed, scope=("src/*", "tests/*")):
    out = []
    for f in c.sources():
        if not matches_any(f, scope) or matches_any(f, allowed):
            continue
        code = c.code(f)
        if any(r.search(code) for r in regexes):
            out.append(f)
    return sorted(out)


def m_truth(c):
    t = c.rules["truth"]
    regexes = [re.compile(p["regex"], re.M) for p in t["patterns"]]
    allowed = [f for a in t["allowed"] for f in a["files"]]
    return files_matching(c, regexes, allowed)


def m_single_home_parts(c):
    """{key: places} for each entry of deps.json single_home."""
    parts = {}
    for e in c.rules["single_home"]["entries"]:
        r = re.compile(e["pattern"], re.M)
        parts[e["key"]] = (e["fact"], files_matching(c, [r], e["home"]))
    return parts


def m_single_home(c):
    return ["%s (%s)" % (f, key) for key, (_, files) in m_single_home_parts(c).items()
            for f in files]


def m_untested(c):
    if not shutil.which("bazel"):
        return None
    out = subprocess.run(
        ["bazel", "query", "--noshow_progress", "--output=label",
         'kind("source file", deps(tests(//...)))'],
        cwd=c.root, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    if out.returncode != 0:
        raise SystemExit("measure: bazel query failed (exit %d)" % out.returncode)
    tested = {l[3:] for l in out.stdout.split() if l.startswith("//:")}
    return sorted(f for f in c.files if f.startswith("src/") and f not in tested)


def m_untested_headers(c):
    tracked = set(c.files)
    return sorted(f for f in c.files
                  if f.startswith("src/") and f.endswith(".h")
                  and "tests/%s_test.cc" % os.path.basename(f)[:-2] not in tracked)


def expand(pattern):
    """Brace expansion ({a,b}) of one pattern, for generated's globs."""
    m = re.search(r"\{([^{}]*)\}", pattern)
    if not m:
        return [pattern]
    return [x for alt in m.group(1).split(",")
            for x in expand(pattern[:m.start()] + alt + pattern[m.end():])]


def m_generated(c):
    return sorted(e["committed"] for e in c.rules["generated"]["entries"] if not e.get("test"))


def m_generated_unmatched(c):
    """Entries whose committed pattern matches no tracked file: the data is stale."""
    out = []
    for e in c.rules["generated"]["entries"]:
        pats = expand(e["committed"])
        if not any(any(fnmatch.fnmatchcase(f, p) for f in c.files) for p in pats):
            out.append(e["committed"])
    return out or None


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


def m_portable(c):
    p = c.rules["portable"]
    regexes = [re.compile(x["regex"], re.M) for x in p["patterns"]]
    return files_matching(c, regexes, p["allowed"])


def m_namespace(c):
    groups = [g for g in c.rules["groups"] if g.get("namespace")]
    if not groups:
        return None
    want = {g["name"]: g["namespace"] for g in groups}
    out = []
    for f in c.sources():
        ns = want.get(c.result.group_of.get(f))
        if not ns:
            continue
        opened = re.findall(r"^\s*namespace\s+([A-Za-z_][\w:]*)\s*\{", c.code(f), re.M)
        if not any(o == ns or o.startswith(ns + "::") for o in opened) or \
                any(not (o == ns or o.startswith(ns + "::") or ns.startswith(o + "::"))
                    for o in opened):
            out.append(f)
    return out


# The table. key, provisional principle number, what is counted, the
# function that lists the places, whether it needs `bazel query`.
METRICS = [
    ("layering", 1, "#include lines against the groups' allowed direction (deps.json)", m_layering, False),
    ("declared", 2, "targets' deps none of whose headers the target includes", m_declared, False),
    ("declared.project", 2, "of them on project libraries", m_declared_project, False),
    ("declared.outside", 2, "of them on outside libraries", m_declared_outside, False),
    ("outside", 3, "outside dependencies in MODULE.bazel (bazel_dep and fetches; may not grow)", m_outside, False),
    ("outside.unpinned", 3, "fetches or bazel_deps not pinned by a version, commit or hash", m_outside_unpinned, False),
    ("outside.mujoco", 3, "targets that list @mujoco directly (may not grow)", targets_listing("mujoco"), False),
    ("outside.eigen", 3, "targets that list @eigen directly (may not grow)", targets_listing("eigen"), False),
    ("outside.googletest", 3, "targets that list @googletest directly (may not grow)", targets_listing("googletest"), False),
    ("outside.nlohmann_json", 3, "targets that list @nlohmann_json directly (may not grow)", targets_listing("nlohmann_json"), False),
    ("outside.ompl", 3, "targets that list @ompl directly (may not grow)", targets_listing("ompl"), False),
    ("outside.glfw", 3, "targets that list @glfw directly (may not grow)", targets_listing("glfw"), False),
    ("outside.mujoco_files", 3, "files that include mujoco/ (may not grow)", m_outside_mujoco_files, False),
    ("truth", 4, "files that read the simulation's truth and are not allowed to (deps.json truth)", m_truth, False),
    ("single_home", 5, "places where a fact that has a home is written again (file and fact; deps.json single_home)", m_single_home, False),
    ("single_home.*", 5, "files outside its home that write {}", m_single_home_parts, False),
    ("untested", 6, "files under src/ on which no test target depends (bazel)", m_untested, True),
    ("untested.headers", 6, "headers src/x.h with no tests/x_test.cc", m_untested_headers, False),
    ("generated", 7, "committed generated files with no test that compares them to their source", m_generated, False),
    ("generated.unmatched", 7, "deps.json generated entries that match no tracked file", m_generated_unmatched, False),
    ("size", 8, "files under src/ too long, and headers whose opening comment is too short", m_size, False),
    ("size.cc", 8, ".cc files under src/ over the line limit of deps.json size", m_size_cc, False),
    ("size.h", 8, "headers under src/ over the line limit of deps.json size", m_size_h, False),
    ("size.comment", 8, "headers whose opening comment is shorter than deps.json size says", m_size_comment, False),
    ("portable", 9, "files that use what differs between platforms or runs outside src/portable_random.h", m_portable, False),
    ("namespace", 10, "files whose namespace is not their group's", m_namespace, False),
]


def label_of(key, p):
    return "P%d" % p if "." not in key else "P%d.%s" % (p, key.split(".", 1)[1])


def validate_table():
    seen = {}
    for key, p, _, _, _ in METRICS:
        lab = label_of(key, p)
        if lab in seen:
            raise SystemExit("measure.py: %s is printed by both %s and %s; give each its own "
                             "principle or part" % (lab, seen[lab], key))
        seen[lab] = key


def measure(c, only=None, notes=True):
    """[(label, key, figure, text, places)] for the lines to print."""
    have_bazel = shutil.which("bazel") is not None
    rows = []
    for key, p, text, fn, bazel in METRICS:
        if only is not None and key not in only:
            continue
        if bazel and not have_bazel:
            if notes:
                sys.stderr.write("measure: %s (%s) left out: bazel is not on PATH\n"
                                 % (label_of(key, p), key))
            continue
        places = fn(c)
        if places is None:
            continue
        if key.endswith(".*"):
            for part, (words, ps) in places.items():
                k = key[:-1] + part
                rows.append((label_of(k, p), k, len(ps), text.replace("{}", words), ps))
            continue
        rows.append((label_of(key, p), key, len(places), text, places))
    return rows


def format_row(row):
    return "%s %d %s" % (row[0], row[2], row[3])


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


def run_check(root):
    problems = []
    dep_problems, _ = deps.check(root)
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
        rows = [r for r in measure(c, notes=False) if want in (r[0], r[1])]
        if not rows:
            sys.stderr.write("measure: no line %s; the lines are: %s\n"
                             % (want, " ".join(r[0] for r in measure(c, notes=False))))
            return 2
        for row in rows:
            for place in row[4]:
                print(place)
        return 0
    if argv[1] == "--check" and len(argv) == 2:
        problems = run_check(ROOT)
        for p in problems:
            print(p)
        if not problems:
            print("ok")
        return 1 if problems else 0
    sys.stderr.write(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
