#!/usr/bin/env python3
"""deps.py [--list KIND | --graph | --kinds]

Checks the dependency rules of tools/architecture/deps.json against the
repository as it is: the #include lines of every tracked file under src/ and
tests/, and the targets of BUILD.bazel. It needs no Bazel, only git and
Python 3's standard library, and takes about a second.

The rules, and the places where the repository departs from them today, are
data in deps.json; that file's "about" strings explain each part. This
script fails, with one line per problem that names the place (a file or a
target) and the rule broken, when:

  (a) a file under src/ or tests/ is in no group, or in more than one;
  (b) an #include "src/..." goes from a group to one that the first may not
      include (may_include), and is not a listed departure;
  (c) a file includes an outside library that its group may not (outside),
      and that is not a listed departure;
  (d) the project's headers include each other in a cycle (the cycle is
      printed);
  (e) a target's files include a header of a library that is not among the
      target's deps (direct deps only; a header in the target's own srcs or
      hdrs is fine), or a project header that no target offers;
  (f) a target lists in deps a project library none of whose headers its
      files include, and the pair is not a listed departure;
  (g) a listed departure no longer occurs, so that the list can only shrink;
  (h) a target lists an outside library in deps and none of its files
      include it, and the pair is not a listed departure.

BUILD.bazel is read with Python's ast module (Starlark is Python syntax):
cc_library, cc_binary, cc_test and alias are understood; glob, select,
list concatenation, "%" formatting and names assigned at the top of the
file are evaluated; other rules (genrule, filegroup) are skipped. #include
lines are read with a regular expression after comments and string
literals (raw strings too) are blanked out, so an #include inside a comment
does not count.

Arguments:
  (none)        Run the check. Prints "ok" and one summary line and exits 0,
                or prints one line per problem and exits 1.
  --list KIND   Print the places of one kind as they are found now, one per
                line, whether listed as departures or not. KIND is one of
                include, outside_include, unused_dep, unused_outside_dep,
                missing_dep, cycle. Exits 0.
  --kinds       Print the kinds that --list knows. Exits 0.
  --graph       Print the group-to-group include counts as a table: one row
                per including group, one column per included group, the
                number of #include lines. Exits 0.

Exit codes: 0 when the check passes (or after --list, --kinds, --graph),
1 when it finds a problem, 2 on a usage error or a file that cannot be read.

measure.py imports this module: analyse() returns everything found, and
check() the problem lines.
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
import json
import os
import re
import subprocess

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RULES = os.path.join(ROOT, "tools", "architecture", "deps.json")

KINDS = ["include", "outside_include", "unused_dep", "unused_outside_dep",
         "missing_dep", "cycle"]

# The C and C++ standard library: an #include <...> that is none of the
# outside libraries is taken for it when its name has no "/" and either no
# extension or the ".h" of a C header. Anything else in angle brackets that
# no outside library claims is reported, so that a new library cannot come
# in unnoticed.
_STD = re.compile(r"^[a-z_0-9]+(\.h)?$")

_INCLUDE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*([<"])([^>"]+)[>"]', re.M)


# ---------------------------------------------------------------- files


def tracked_files(root=ROOT):
    """Every file git tracks, as paths relative to the repository's root."""
    out = subprocess.run(["git", "ls-files", "-z"], cwd=root, check=True,
                         stdout=subprocess.PIPE).stdout.decode()
    return sorted(p for p in out.split("\0") if p)


def read(root, path):
    with open(os.path.join(root, path), encoding="utf-8", errors="replace") as f:
        return f.read()


def strip_comments(text):
    """Text with comments, string and character literals blanked out.

    Newlines are kept, so line numbers stay as they were. Preprocessor
    lines keep their quoted file name: #include "x" is not a string here.
    """
    out = []
    i, n = 0, len(text)
    line_start = True
    include_line = False
    while i < n:
        c = text[i]
        if c == "\n":
            out.append(c)
            i += 1
            line_start, include_line = True, False
            continue
        if line_start and c in " \t":
            out.append(c)
            i += 1
            continue
        if line_start and c == "#":
            include_line = re.match(r"#[ \t]*include\b", text[i:i + 40]) is not None
        line_start = False
        if text.startswith("//", i):
            j = text.find("\n", i)
            j = n if j < 0 else j
            i = j
            continue
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join("\n" if ch == "\n" else " " for ch in text[i:j]))
            i = j
            continue
        if include_line and c in '"<':
            # The file name of an #include: keep it as it is.
            close = '"' if c == '"' else ">"
            j = text.find(close, i + 1)
            j = n if j < 0 else j + 1
            out.append(text[i:j])
            i = j
            include_line = False
            continue
        m = re.match(r'(?:u8|u|U|L)?R"([^()\\ \n]{0,16})\(', text[i:i + 24])
        if m and (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_")):
            end = ")" + m.group(1) + '"'
            j = text.find(end, i + m.end())
            j = n if j < 0 else j + len(end)
            out.append("".join("\n" if ch == "\n" else " " for ch in text[i:j]))
            i = j
            continue
        if c in "\"'":
            # A digit separator (1'000) is not a character literal.
            if c == "'" and i > 0 and text[i - 1].isalnum() and i + 1 < n and text[i + 1].isalnum() \
                    and re.match(r"[0-9a-fA-F]", text[i - 1]):
                out.append(c)
                i += 1
                continue
            j = i + 1
            while j < n and text[j] != c and text[j] != "\n":
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(c + " " * max(0, j - i - 2) + (c if j - i >= 2 else ""))
            i = j
            continue
        out.append(c)
        i += 1
    return "".join(out)


def includes(text):
    """(line number, '"' or '<', name) for each #include outside comments."""
    code = strip_comments(text)
    found = []
    for m in _INCLUDE.finditer(code):
        line = code.count("\n", 0, m.start()) + 1
        found.append((line, m.group(1), m.group(2).strip()))
    return found


# ---------------------------------------------------------------- BUILD.bazel


class Unknown(Exception):
    """A Starlark expression this reader does not evaluate."""


def _glob_regex(pattern):
    """Bazel's glob: ** crosses directories, * and ? do not."""
    out, i = [], 0
    while i < len(pattern):
        if pattern.startswith("**/", i):
            out.append("(?:.*/)?")
            i += 3
        elif pattern.startswith("**", i):
            out.append(".*")
            i += 2
        elif pattern[i] == "*":
            out.append("[^/]*")
            i += 1
        elif pattern[i] == "?":
            out.append("[^/]")
            i += 1
        else:
            out.append(re.escape(pattern[i]))
            i += 1
    return re.compile("^" + "".join(out) + "$")


class Starlark:
    """Evaluates the few expressions BUILD.bazel uses for its attributes."""

    def __init__(self, files):
        self.files = files
        self.env = {}

    def glob(self, include, exclude=(), allow_empty=True, **_):
        inc = [_glob_regex(p) for p in include]
        exc = [_glob_regex(p) for p in exclude]
        return [f for f in self.files
                if any(r.match(f) for r in inc) and not any(r.match(f) for r in exc)]

    def eval(self, node, local=None):
        local = local or {}
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, (ast.List, ast.Tuple)):
            return [self.eval(e, local) for e in node.elts]
        if isinstance(node, ast.Dict):
            return {self.eval(k, local): self.eval(v, local) for k, v in zip(node.keys, node.values)}
        if isinstance(node, ast.Name):
            if node.id in local:
                return local[node.id]
            if node.id in self.env:
                v = self.env[node.id]
                if isinstance(v, Unknown):
                    raise v
                return v
            if node.id in ("True", "False", "None"):
                return {"True": True, "False": False, "None": None}[node.id]
            raise Unknown("name " + node.id)
        if isinstance(node, ast.BinOp):
            a, b = self.eval(node.left, local), self.eval(node.right, local)
            if isinstance(node.op, ast.Add):
                return a + b
            if isinstance(node.op, ast.Mod):
                return a % b
            if isinstance(node.op, ast.BitOr) and isinstance(a, dict):
                return {**a, **b}
            raise Unknown(ast.unparse(node))
        if isinstance(node, ast.ListComp) and len(node.generators) == 1:
            g = node.generators[0]
            if not isinstance(g.target, ast.Name) or g.ifs:
                raise Unknown(ast.unparse(node))
            return [self.eval(node.elt, {**local, g.target.id: x})
                    for x in self.eval(g.iter, local)]
        if isinstance(node, ast.Call):
            kwargs = {k.arg: self.eval(k.value, local) for k in node.keywords if k.arg}
            args = [self.eval(a, local) for a in node.args]
            if isinstance(node.func, ast.Name) and node.func.id == "glob":
                return self.glob(*args, **kwargs)
            if isinstance(node.func, ast.Name) and node.func.id == "select":
                # Every branch counts: a file or a dep in any of them is a
                # file or a dep of the target on some platform.
                merged = []
                for v in args[0].values():
                    merged += [x for x in v if x not in merged]
                return merged
            if isinstance(node.func, ast.Attribute):
                obj = self.eval(node.func.value, local)
                if isinstance(obj, str) and node.func.attr in ("format", "join"):
                    return getattr(obj, node.func.attr)(*args, **kwargs)
            raise Unknown(ast.unparse(node))
        raise Unknown(ast.unparse(node))


def build_targets(root, files):
    """name -> {kind, srcs, hdrs, deps, actual, line} for BUILD.bazel."""
    tree = ast.parse(read(root, "BUILD.bazel"), "BUILD.bazel")
    star = Starlark(files)
    targets = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                star.env[node.targets[0].id] = star.eval(node.value)
            except Unknown as e:
                star.env[node.targets[0].id] = e
            continue
        if not (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
                and isinstance(node.value.func, ast.Name)):
            continue
        kind = node.value.func.id
        if kind not in ("cc_library", "cc_binary", "cc_test", "alias"):
            continue
        attrs = {}
        for k in node.value.keywords:
            if k.arg in ("name", "srcs", "hdrs", "deps", "actual"):
                try:
                    attrs[k.arg] = star.eval(k.value)
                except Unknown as e:
                    raise SystemExit("BUILD.bazel:%d: cannot read %s of a %s: %s"
                                     % (node.lineno, k.arg, kind, e))
        name = attrs.get("name")
        targets["//:" + name] = {
            "kind": kind,
            "srcs": [s for s in attrs.get("srcs", []) if not s.startswith((":", "@", "//"))],
            "hdrs": [s for s in attrs.get("hdrs", []) if not s.startswith((":", "@", "//"))],
            "deps": attrs.get("deps", []),
            "actual": attrs.get("actual"),
            "line": node.lineno,
        }
    return targets


def label(dep):
    """A dep as written, in the form //:name for this package's targets."""
    if dep.startswith(":"):
        return "//" + dep
    if dep.startswith("//:"):
        return dep
    return dep


# ---------------------------------------------------------------- analysis


def load_rules(root=ROOT):
    with open(os.path.join(root, "tools", "architecture", "deps.json"), encoding="utf-8") as f:
        return json.load(f)


def outside_library(rules, name):
    """The outside library an #include <name> belongs to, or None."""
    for lib, spec in rules["outside_libraries"].items():
        if lib == "about":
            continue
        if any(name.startswith(p) for p in spec["include"]):
            return lib
    return None


def library_of_label(rules, targets, dep):
    """('project', '//:x') or ('outside', 'eigen') for a dep label."""
    dep = label(dep)
    seen = set()
    while dep in targets and targets[dep]["kind"] == "alias" and dep not in seen:
        seen.add(dep)
        dep = label(targets[dep]["actual"])
    if dep in targets:
        return ("project", dep)
    for lib, spec in rules["outside_libraries"].items():
        if lib == "about":
            continue
        for p in spec["labels"]:
            if fnmatch.fnmatchcase(dep, p) or fnmatch.fnmatchcase(dep, p + "//*"):
                return ("outside", lib)
    return ("unknown", dep)


class Result:
    """Everything analyse() finds; found[kind] is a set of tuples."""

    def __init__(self):
        self.problems = []
        self.found = {k: set() for k in KINDS}
        self.group_of = {}
        self.edges = []           # (from file, to file, from group, to group)
        self.files = []
        self.targets = {}
        self.file_includes = {}   # path -> [(line, kind, name)]


def group_files(rules, files, r):
    groups = rules["groups"]
    for path in files:
        if not path.startswith(("src/", "tests/")):
            continue
        hits = [g["name"] for g in groups
                if any(fnmatch.fnmatchcase(path, p) for p in g["files"])]
        if not hits:
            r.problems.append("%s: in no group (deps.json groups)" % path)
        elif len(hits) > 1:
            r.problems.append("%s: in more than one group: %s (deps.json groups)"
                              % (path, ", ".join(hits)))
        else:
            r.group_of[path] = hits[0]


def find_cycles(graph):
    """Each strongly connected component of more than one node (or a self
    loop), as one cycle through it, smallest node first."""
    index, low, stack, on, comps, counter = {}, {}, [], set(), [], [0]

    def visit(v):
        index[v] = low[v] = counter[0]
        counter[0] += 1
        stack.append(v)
        on.add(v)
        for w in sorted(graph.get(v, ())):
            if w not in index:
                visit(w)
                low[v] = min(low[v], low[w])
            elif w in on:
                low[v] = min(low[v], index[w])
        if low[v] == index[v]:
            comp = []
            while True:
                w = stack.pop()
                on.discard(w)
                comp.append(w)
                if w == v:
                    break
            if len(comp) > 1 or v in graph.get(v, ()):
                comps.append(sorted(comp))

    sys.setrecursionlimit(10000)
    for v in sorted(graph):
        if v not in index:
            visit(v)
    cycles = []
    for comp in comps:
        members = set(comp)
        start = comp[0]
        # A shortest way back to the start inside the component.
        prev, frontier, found = {start: None}, [start], None
        while frontier and found is None:
            nxt = []
            for v in frontier:
                for w in sorted(graph.get(v, ())):
                    if w == start:
                        found = v
                        break
                    if w in members and w not in prev:
                        prev[w] = v
                        nxt.append(w)
                if found is not None:
                    break
            frontier = nxt
        path, v = [], found
        while v is not None:
            path.append(v)
            v = prev[v]
        cycles.append(tuple(reversed(path)) + (start,))
    return cycles


def analyse(root=ROOT, rules=None):
    rules = rules or load_rules(root)
    r = Result()
    r.files = tracked_files(root)
    tracked = set(r.files)
    group_files(rules, r.files, r)
    groups = {g["name"]: g for g in rules["groups"]}
    tests_outside = set(rules.get("tests_outside", []))

    # Files: includes, direction, outside libraries.
    header_graph = {}
    for path in r.files:
        if not path.startswith(("src/", "tests/")) or not path.endswith((".h", ".cc")):
            continue
        incs = includes(read(root, path))
        r.file_includes[path] = incs
        g = r.group_of.get(path)
        for line, kind, name in incs:
            if kind == '"':
                if name not in tracked:
                    r.problems.append("%s:%d: includes %s, which is not a tracked file"
                                      % (path, line, name))
                    continue
                if path.endswith(".h") and name.endswith(".h"):
                    header_graph.setdefault(path, set()).add(name)
                h = r.group_of.get(name)
                if g and h:
                    r.edges.append((path, name, g, h))
                    if h != g and h not in groups[g]["may_include"]:
                        r.found["include"].add((path, name))
                continue
            lib = outside_library(rules, name)
            if lib is None:
                if not _STD.match(name):
                    r.problems.append("%s:%d: includes <%s>, which is no outside library of "
                                      "deps.json and not the standard library" % (path, line, name))
                continue
            allowed = set(groups[g]["outside"]) if g else set()
            if path.startswith("tests/"):
                allowed |= tests_outside
            if g and lib not in allowed:
                r.found["outside_include"].add((path, lib))

    for cyc in find_cycles(header_graph):
        r.found["cycle"].add(cyc)

    # Targets.
    targets = build_targets(root, r.files)
    r.targets = targets
    offered = {}   # header -> project libraries that list it in hdrs
    for name, t in targets.items():
        if t["kind"] == "cc_library":
            for h in t["hdrs"]:
                offered.setdefault(h, set()).add(name)
    for name, t in targets.items():
        if t["kind"] == "alias":
            continue
        own = set(t["srcs"]) | set(t["hdrs"])
        for f in own:
            if f not in tracked:
                r.problems.append("%s: lists %s, which is not a tracked file (BUILD.bazel:%d)"
                                  % (name, f, t["line"]))
        deps = {}
        for d in t["deps"]:
            deps[label(d)] = library_of_label(rules, targets, d)
        project_deps = {v[1]: d for d, v in deps.items() if v[0] == "project"}
        outside_deps = {}
        for d, v in deps.items():
            if v[0] == "outside":
                outside_deps.setdefault(v[1], d)
            elif v[0] == "unknown":
                r.problems.append("%s: deps lists %s, which is neither a target of BUILD.bazel "
                                  "nor an outside library of deps.json" % (name, d))
        used_project, used_outside = set(), set()
        for f in sorted(own):
            for line, kind, inc in r.file_includes.get(f, []):
                if kind == '"':
                    if inc in own or inc not in tracked:
                        continue
                    libs = offered.get(inc, set())
                    if not libs:
                        r.problems.append("%s: %s includes %s, which no cc_library offers in hdrs "
                                          "(rule e)" % (name, f, inc))
                        continue
                    hit = libs & set(project_deps)
                    if hit:
                        used_project |= hit
                    else:
                        r.found["missing_dep"].add((name, sorted(libs)[0]))
                else:
                    lib = outside_library(rules, inc)
                    if lib is None:
                        continue
                    if lib in outside_deps:
                        used_outside.add(lib)
                    else:
                        r.found["missing_dep"].add((name, "@" + lib if not lib.startswith("@") else lib))
        for lib in project_deps:
            if lib not in used_project:
                r.found["unused_dep"].add((name, lib))
        for lib in outside_deps:
            if lib not in used_outside:
                r.found["unused_outside_dep"].add((name, lib))

    # Compare with the listed departures.
    listed = {k: set() for k in KINDS}
    for d in rules["departures"].get("list", []):
        listed.setdefault(d["kind"], set()).add(departure_key(d))
    r.listed = listed
    explain = {
        "include": lambda k: "%s: includes %s; group %s may not include group %s "
                             "(deps.json may_include, rule b)"
                             % (k[0], k[1], r.group_of.get(k[0]), r.group_of.get(k[1])),
        "outside_include": lambda k: "%s: includes outside library %s, which group %s may not "
                                     "include (deps.json outside, rule c)"
                                     % (k[0], k[1], r.group_of.get(k[0])),
        "unused_dep": lambda k: "%s: deps lists %s, none of whose headers its files include "
                                "(rule f)" % k,
        "unused_outside_dep": lambda k: "%s: deps lists outside library %s, which none of its "
                                        "files include (rule h)" % k,
        "missing_dep": lambda k: "%s: its files include a header of %s, which is not in its "
                                 "deps (rule e)" % k,
        "cycle": lambda k: "%s: the project's headers include each other in a cycle: %s (rule d)"
                           % (k[0], " -> ".join(k)),
    }
    for kind in KINDS:
        for k in sorted(r.found[kind] - listed.get(kind, set())):
            r.problems.append(explain[kind](k))
    for kind in sorted(listed):
        for k in sorted(listed[kind] - r.found.get(kind, set())):
            r.problems.append("%s: listed in deps.json departures (%s) but no longer occurs; "
                              "remove the entry (rule g)" % (place(kind, k), kind))
    return r


def departure_key(d):
    kind = d["kind"]
    if kind == "include":
        return (d["from"], d["to"])
    if kind == "outside_include":
        return (d["file"], d["library"])
    if kind in ("unused_dep", "unused_outside_dep", "missing_dep"):
        return (d["target"], d["dep"])
    if kind == "cycle":
        return tuple(d["cycle"])
    raise SystemExit("deps.json: unknown departure kind %r" % kind)


def place(kind, key):
    """One line naming a found place of a kind, for --list."""
    if kind == "include":
        return "%s -> %s" % key
    if kind == "cycle":
        return " -> ".join(key)
    return "%s %s" % key


def check(root=ROOT):
    """(problem lines, Result)."""
    r = analyse(root)
    return r.problems, r


def graph_table(r, rules):
    """The group-to-group include counts: rows include columns' headers."""
    names = [g["name"] for g in rules["groups"]]
    groups = {g["name"]: g for g in rules["groups"]}
    counts = {}
    for _, _, g, h in r.edges:
        counts[(g, h)] = counts.get((g, h), 0) + 1
    width = max(len(n) for n in names) + 3
    lines = ["#include lines from the files of a row's group to the headers of a column's",
             "group; * marks a count against may_include (listed departures).",
             "",
             " " * width + "".join("%6d" % (i + 1) for i in range(len(names)))]
    for i, g in enumerate(names):
        cells = []
        for h in names:
            n = counts.get((g, h), 0)
            bad = n and h != g and h not in groups[g]["may_include"]
            cells.append("%6s" % (("%d*" % n) if bad else (str(n) if n else ".")))
        lines.append("%-*s%s" % (width, "%d %s" % (i + 1, g), "".join(cells)))
    return "\n".join(lines)


def main(argv):
    if len(argv) == 1:
        problems, r = check()
        if problems:
            for p in problems:
                print(p)
            return 1
        n_dep = sum(len(v) for v in r.listed.values())
        print("ok")
        print("%d files in %d groups, %d targets, %d #include lines between project files, "
              "%d listed departures" % (len(r.group_of), len(load_rules()["groups"]),
                                        len([t for t in r.targets.values() if t["kind"] != "alias"]),
                                        len(r.edges), n_dep))
        return 0
    if argv[1] == "--list" and len(argv) == 3 and argv[2] in KINDS:
        r = analyse()
        for k in sorted(r.found[argv[2]]):
            print(place(argv[2], k))
        return 0
    if argv[1] == "--kinds" and len(argv) == 2:
        print("\n".join(KINDS))
        return 0
    if argv[1] == "--graph" and len(argv) == 2:
        r = analyse()
        print(graph_table(r, load_rules()))
        return 0
    sys.stderr.write(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
