#!/usr/bin/env python3
"""deps.py [--strict-items] [--list KIND | --graph | --kinds]

Checks the dependency rules of tools/architecture/deps.json against the
repository as it is: the #include lines of every tracked file under src/ and
tests/, and the targets of BUILD.bazel. It needs no Bazel, only git and
Python 3's standard library, and takes about a second.

The rules, and the places where the repository departs from them today, are
data in deps.json; that file's "about" strings explain each part. This
script fails, with one line per problem that names the place (a file or a
target) and the rule broken, when:

  (a) a file under src/ or tests/ is in no group, or in more than one, or
      is neither .h nor .cc; or a directive cannot be followed: an
      #include whose operand is not a literal "..." or <...> (a macro), an
      #import, an #include_next. Backslash-newline is removed first and the
      digraph %: is read as #;
  (b) an #include "src/..." goes from a group to one that the first may not
      include (may_include), and is not a listed departure;
  (c) a file includes an outside library that its group may not (outside),
      directly or through project headers, and that is not a listed
      departure; an #include <...> must name an outside library of deps.json
      or be in STD_HEADERS, the fixed list of C++20 and C library headers;
  (d) the project's headers include each other in a cycle (the cycle is
      printed);
  (e) a target's files include a header of a library that is not among the
      target's deps (direct deps only; a header in the target's own srcs or
      hdrs is fine), or a project header that no target offers; BUILD.bazel
      uses an attribute on a cc_library, cc_binary, cc_test or alias that
      deps.json build_attributes does not list, a rule of another kind lists
      a file under src/ or tests/, or a comprehension cannot be evaluated;
  (f) a target lists in deps a project library none of whose headers its
      files include, and the pair is not a listed departure;
  (g) a listed departure no longer occurs, so that the list can only shrink;
  (h) a target lists an outside library in deps and none of its files
      include it, and the pair is not a listed departure;
  (i) deps.json is malformed: a group without a boolean frozen or a list
      namespaces, a may_include that names no group, or a departure whose
      item is not B<n> (the backlog item that removes it), "left" (the
      backlog's "Left as it is" names it: it is in a frozen file) or "B?"
      (no item yet). With --strict-items, "B?" is a problem too.

BUILD.bazel is read with Python's ast module (Starlark is Python syntax):
cc_library, cc_binary, cc_test and alias are understood, also when a list
comprehension makes several of them (the targets of one comprehension share
a "family"); glob, select, list concatenation, "%" formatting and names
assigned at the top of the file are evaluated. Other rules (genrule,
filegroup) are only counted and named: the summary line says how many rules
were read as targets, of how many, and which kinds were not. #include
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

# The C and C++ standard library, by its names (C++20 and the C headers, in
# both spellings). Anything else in <...> must be an outside library of
# deps.json.
STD_HEADERS = frozenset("""
algorithm any array atomic barrier bit bitset charconv chrono codecvt compare complex concepts
condition_variable coroutine deque exception execution filesystem format forward_list fstream
functional future initializer_list iomanip ios iosfwd iostream istream iterator latch limits list
locale map memory memory_resource mutex new numbers numeric optional ostream queue random ranges
ratio regex scoped_allocator stdexcept semaphore set shared_mutex source_location span sstream stack
stop_token streambuf string string_view strstream syncstream system_error thread tuple
type_traits typeindex typeinfo unordered_map unordered_set utility valarray variant vector version
cassert cctype cerrno cfenv cfloat cinttypes ciso646 climits clocale cmath csetjmp csignal
cstdalign cstdarg cstdbool cstddef cstdint cstdio cstdlib cstring ctgmath ctime cuchar cwchar
cwctype ccomplex
assert.h complex.h ctype.h errno.h fenv.h float.h inttypes.h iso646.h limits.h locale.h math.h
setjmp.h signal.h stdalign.h stdarg.h stdatomic.h stdbool.h stddef.h stdint.h stdio.h stdlib.h
string.h tgmath.h threads.h time.h uchar.h wchar.h wctype.h
""".split())

_DIRECTIVE = re.compile(r"^[ \t]*(?:#|%:)[ \t]*(include_next|include|import)\b[ \t]*(.*?)[ \t]*$", re.M)
_LITERAL = re.compile(r'^(?:"([^">]+)"|<([^<>"]+)>)$')


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
        if line_start and (c == "#" or text.startswith("%:", i)):
            include_line = re.match(r"(?:#|%:)[ \t]*(?:include_next|include|import)\b",
                                    text[i:i + 40]) is not None
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


def splice_lines(text):
    """The text with each backslash-newline removed, as the preprocessor does
    before it reads a directive, and a function that gives the line of the
    original that an offset of the new text lies on."""
    pieces, cuts, pos = [], [], 0
    for m in re.finditer(r"\\\r?\n", text):
        pieces.append(text[pos:m.start()])
        pos = m.end()
        cuts.append((sum(len(p) for p in pieces), len(cuts) + 1))
    pieces.append(text[pos:])
    joined = "".join(pieces)

    def line_of(offset):
        extra = 0
        for at, k in cuts:
            if at <= offset:
                extra = k
            else:
                break
        return joined.count("\n", 0, offset) + 1 + extra
    return joined, line_of


def includes(text):
    """(line number, kind, name) for each #include outside comments. kind is
    '"' or '<' for a literal file name; '!' for a directive that this check
    cannot follow (#import, #include_next, or an operand that is not a
    literal "..." or <...>, such as a macro), where name says why."""
    joined, line_of = splice_lines(text)
    code = strip_comments(joined)
    found = []
    for m in _DIRECTIVE.finditer(code):
        line = line_of(m.start())
        directive, operand = m.group(1), m.group(2)
        if directive != "include":
            found.append((line, "!", "#%s is not followed (use #include)" % directive))
            continue
        lit = _LITERAL.match(operand)
        if not lit:
            found.append((line, "!", "#include %s: the operand is not a literal \"...\" or <...>"
                          % operand))
        elif lit.group(1) is not None:
            found.append((line, '"', lit.group(1).strip()))
        else:
            found.append((line, "<", lit.group(2).strip()))
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
        if isinstance(node, ast.List):
            return [self.eval(e, local) for e in node.elts]
        if isinstance(node, ast.Tuple):
            return tuple(self.eval(e, local) for e in node.elts)
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
        if isinstance(node, ast.ListComp):
            return [self.eval(node.elt, env) for env in _bindings(self, node, local)]
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


# The rules of BUILD.bazel that are read as targets (they have files and
# deps); every other rule is only counted and named.
TARGET_KINDS = ("cc_library", "cc_binary", "cc_test")


def _bindings(star, comp, local):
    """The local names of each pass of a list comprehension's loops."""
    passes = [dict(local)]
    for g in comp.generators:
        if g.ifs:
            raise Unknown(ast.unparse(comp))
        nxt = []
        for env in passes:
            for x in star.eval(g.iter, env):
                new = dict(env)
                if isinstance(g.target, ast.Name):
                    new[g.target.id] = x
                elif isinstance(g.target, ast.Tuple):
                    for t, v in zip(g.target.elts, x):
                        new[t.id] = v
                else:
                    raise Unknown(ast.unparse(comp))
                nxt.append(new)
        passes = nxt
    return passes


def build_targets(root, files, info=None, allowed_attrs=None):
    """name -> {kind, srcs, hdrs, deps, actual, copts, line, family} for
    BUILD.bazel. The targets that one comprehension makes share a family (the
    line of the statement). When info is a dict, it receives "rules" (kind ->
    names of every rule, those it does not read as targets included) and
    "problems" (what in BUILD.bazel this reader refuses: an attribute of a
    cc_* rule that allowed_attrs, a dict of kind -> attribute names, does
    not list; a rule of another kind whose srcs or hdrs name a file under
    src/ or tests/; an expression it cannot evaluate)."""
    tree = ast.parse(read(root, "BUILD.bazel"), "BUILD.bazel")
    star = Starlark(files)
    targets = {}
    rules = {}
    problems = []

    def one(call, local, line):
        kind = call.func.id
        attrs = {}
        for k in call.keywords:
            if k.arg is None:
                problems.append("BUILD.bazel:%d: a %s with ** arguments, which this check does "
                                "not read (rule e)" % (line, kind))
                continue
            if kind in TARGET_KINDS or kind == "alias":
                if allowed_attrs is not None and k.arg not in allowed_attrs.get(kind, []):
                    problems.append("BUILD.bazel:%d: a %s with the attribute %s, which deps.json "
                                    "build_attributes does not list (rule e)" % (line, kind, k.arg))
            if k.arg in ("name", "srcs", "hdrs", "deps", "actual", "copts", "data"):
                try:
                    attrs[k.arg] = star.eval(k.value, local)
                except Unknown as e:
                    problems.append("BUILD.bazel:%d: cannot read %s of a %s: %s (rule e)"
                                    % (line, k.arg, kind, e))
        name = attrs.get("name")
        if kind not in TARGET_KINDS and kind != "alias":
            for attr in ("srcs", "hdrs"):
                for f in attrs.get(attr, []) if isinstance(attrs.get(attr), list) else []:
                    if isinstance(f, str) and f.startswith(("src/", "tests/")):
                        problems.append("BUILD.bazel:%d: a %s lists %s in %s, but this check reads "
                                        "only cc_library, cc_binary and cc_test (rule e)"
                                        % (line, kind, f, attr))
        if name is None:
            return
        rules.setdefault(kind, []).append(name)
        if kind not in TARGET_KINDS and kind != "alias":
            return
        targets["//:" + name] = {
            "kind": kind,
            "srcs": [s for s in attrs.get("srcs", []) if not s.startswith((":", "@", "//"))],
            "hdrs": [s for s in attrs.get("hdrs", []) if not s.startswith((":", "@", "//"))],
            "deps": attrs.get("deps", []),
            "actual": attrs.get("actual"),
            "copts": attrs.get("copts", []),
            "data": attrs.get("data", []),
            "line": line,
            "family": line,
        }

    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                star.env[node.targets[0].id] = star.eval(node.value)
            except Unknown as e:
                star.env[node.targets[0].id] = e
            continue
        if not isinstance(node, ast.Expr):
            continue
        v = node.value
        if isinstance(v, ast.ListComp) and isinstance(v.elt, ast.Call) \
                and isinstance(v.elt.func, ast.Name):
            try:
                passes = _bindings(star, v, {})
            except Unknown as e:
                problems.append("BUILD.bazel:%d: cannot evaluate this list comprehension: %s "
                                "(rule e)" % (node.lineno, e))
                continue
            for env in passes:
                one(v.elt, env, node.lineno)
        elif isinstance(v, ast.Call) and isinstance(v.func, ast.Name):
            one(v, {}, node.lineno)
    if info is not None:
        info["rules"] = rules
        info["problems"] = problems
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
        self.rules = {}           # rule kind -> names, every rule of BUILD.bazel
        self.rule_names = set()   # //:name of every rule


def group_files(rules, files, r):
    groups = rules["groups"]
    for path in files:
        if not path.startswith(("src/", "tests/")):
            continue
        if not path.endswith((".h", ".cc")):
            r.problems.append("%s: a file under src/ or tests/ that is neither .h nor .cc, "
                              "which this check does not read (rule a)" % path)
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


def analyse(root=ROOT, rules=None, strict_items=False):
    """Everything found. With strict_items a departure whose item is "B?"
    (no backlog item yet) is a problem too."""
    rules = rules or load_rules(root)
    r = Result()
    for g in rules["groups"]:
        if not isinstance(g.get("frozen"), bool):
            r.problems.append("deps.json: group %s has no boolean frozen" % g["name"])
        if not isinstance(g.get("namespaces"), list):
            r.problems.append("deps.json: group %s has no list namespaces" % g["name"])
        for m in g["may_include"]:
            if m not in {x["name"] for x in rules["groups"]}:
                r.problems.append("deps.json: group %s may include %s, which is no group"
                                  % (g["name"], m))
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
            if kind == "!":
                r.problems.append("%s:%d: %s (rule a)" % (path, line, name))
                continue
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
                if name not in STD_HEADERS:
                    r.problems.append("%s:%d: includes <%s>, which is no outside library of "
                                      "deps.json and not in the list of standard headers "
                                      "(deps.py STD_HEADERS)" % (path, line, name))
                continue
            allowed = set(groups[g]["outside"]) if g else set()
            if path.startswith("tests/"):
                allowed |= tests_outside
            if g and lib not in allowed:
                r.found["outside_include"].add((path, lib))

    # Outside libraries through project headers: a file may not reach, by
    # any chain of #include "src/...", a library its group may not include.
    memo = {}

    def reach(f, seen=()):
        if f in memo:
            return memo[f]
        if f in seen:
            return set()
        out = set()
        for line, kind, name in r.file_includes.get(f, []):
            if kind == "<":
                lib = outside_library(rules, name)
                if lib:
                    out.add(lib)
            elif kind == '"' and name in r.file_includes:
                out |= reach(name, seen + (f,))
        memo[f] = out
        return out

    for path in r.file_includes:
        g = r.group_of.get(path)
        if not g:
            continue
        allowed = set(groups[g]["outside"]) | (tests_outside if path.startswith("tests/") else set())
        for lib in reach(path) - allowed:
            r.found["outside_include"].add((path, lib))

    for cyc in find_cycles(header_graph):
        r.found["cycle"].add(cyc)

    # Targets.
    info = {}
    targets = build_targets(root, r.files, info, rules.get("build_attributes", {}))
    r.problems += info["problems"]
    r.targets = targets
    r.rules = info["rules"]
    r.rule_names = {"//:" + n for names in r.rules.values() for n in names}
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
        item = d.get("item")
        if not isinstance(item, str) or not re.match(r"^(B(\?|[0-9]+)|left)$", item):
            r.problems.append("deps.json: the departure %s (%s) has item %r, which is not B<n>, left or B?"
                              % (place(d["kind"], departure_key(d)), d["kind"], item))
        elif strict_items and item == "B?":
            r.problems.append("deps.json: the departure %s (%s) has item B?; name its backlog item "
                              "(--strict-items)" % (place(d["kind"], departure_key(d)), d["kind"]))
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


def check(root=ROOT, strict_items=False):
    """(problem lines, Result)."""
    r = analyse(root, strict_items=strict_items)
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
    strict = "--strict-items" in argv
    argv = [a for a in argv if a != "--strict-items"]
    if len(argv) == 1:
        r = analyse(strict_items=strict)
        problems = r.problems
        if problems:
            for p in problems:
                print(p)
            return 1
        n_dep = sum(len(v) for v in r.listed.values())
        print("ok")
        skipped = ", ".join("%s %d" % (k, len(v)) for k, v in sorted(r.rules.items())
                            if k not in TARGET_KINDS)
        n_rules = sum(len(v) for v in r.rules.values())
        n_read = sum(len(r.rules.get(k, [])) for k in TARGET_KINDS)
        print("%d files in %d groups, %d targets read of %d rules (not read as targets: %s), "
              "%d #include lines between project files, %d listed departures"
              % (len(r.group_of), len(load_rules()["groups"]), n_read, n_rules, skipped,
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
