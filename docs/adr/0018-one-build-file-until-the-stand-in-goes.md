# 0018 The build stays one BUILD.bazel and src/ stays flat until the stand-in cell is removed

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R12)

**Context.** Every target of the repository is in one package: `BUILD.bazel` at the
top, 2,133 lines, 110 rules in 29 sections marked `# BEGIN x` / `# END x`, and
`grep -c visibility BUILD.bazel` prints 0. `src/` holds 99 files in one directory and
`tests/` 41. The usual advice for a Bazel project is a package for each directory with
narrow `visibility`, so that Bazel itself refuses a dependency in the wrong direction
([S4], [S5] of `docs/architecture/research.md`), and Drake is built that way ([S48],
[S49]). Four facts of this repository weigh against doing that now. `AGENTS.md`,
"Dividing the work", divides work among agents by the sections of the one file, and
features are being built in it while any restructuring runs. Dividing the package
changes every label and every include path: 44 distinct labels `//:x` are named in
`AGENTS.md`, `README.md`, `docs/`, `tools/` and CI
(`grep -o -h '//:[a-z_0-9]*' AGENTS.md README.md docs/*.md tools/video/* tools/plots/* .github/workflows/ci.yml | sort -u | wc -l`),
and 463 `#include "src/..."` lines would be rewritten. About half of the files belong
to the stand-in cell, whose contents a refactoring does not edit (ADR 0020), and an
include path is contents. And `layering_check`, the other way Bazel can hold a
layering, needs Clang as the compiler ([S6]), which would change how everything here
is built.

**Decision.** The build stays one `BUILD.bazel` with a section for each piece, and
`src/` and `tests/` stay flat. No `visibility` is added and the compiler stays GCC. The
layering that packages would hold is held by `tools/architecture/deps.py` instead: it
reads the `#include` lines and the `deps` of every target against the groups of
`tools/architecture/deps.json` (ADR 0017). A new target goes into the section of its
piece; a new piece gets a new section. This is decided again, by a new record that
supersedes this one, when the stand-in cell is removed: then no file is frozen, and the
groups of ADR 0017 are the directories and packages to divide into.

**Alternatives.**
- A package for each group now, with `visibility`. Lost: it edits the include lines of
  the stand-in's files, moves every label that documents and CI name, and collides with
  every feature under way in `BUILD.bazel`; what it would buy, a refused dependency, the
  include check gives today in about a second.
- Packages for the new cell only (`layout`, `hardware`), the stand-in left at the top.
  Lost: two conventions at once, and the new cell's files would move twice.
- Clang and `layering_check`. Lost: a change of compiler for every figure in the
  documents, to check what `deps.py` already checks.
- `depend_on_what_you_use` as an aspect. Lost: it touches `MODULE.bazel` while
  `bazel mod deps` is broken, and it cannot see groups inside one package ([S7]).

**Consequences.** Nothing moves, so no document, label or include path changes, and
agents keep dividing work by sections. The price: Bazel does not refuse a wrong
dependency by itself; the check is a script that must be run (`measure.sh --check`,
which `tools/architecture/ci.sh` and CI run). `BUILD.bazel` keeps growing, and one
agent at a time still edits each section.

**Check.** P5 and P1 of `docs/architecture/principles.md`: `measure.sh --check` fails
on an include against the direction or a `deps` entry that is not used.
`grep -c visibility BUILD.bazel` prints 0 and `ls BUILD.bazel src/BUILD.bazel` shows
only the first.

**Migration.** None: the code already follows this.
