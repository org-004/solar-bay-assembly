# 0015 clang-tidy 18 over a compile database finds defects; sanitizers run by hand

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R29, R34)

**Context.** The tool is clang-tidy 18: 18.1.8 on Debian 13, 18.1.3 on Ubuntu 24.04,
the image CI's runner uses. The chosen checks give the same 13 counts and 128 places
under both (`T` lines of `docs/architecture/baseline.txt`).
`tools/architecture/tidy.sh` reads GCC's compile commands from `bazel aquery` through
`tools/architecture/compdb.py`, not an aspect: `bazel_clang_tidy` [S8] would add a
module to `MODULE.bazel` while `bazel mod deps` is broken here. The headers come from
`bazel build --output_groups=compilation_prerequisites_INTERNAL_`, which compiles
nothing. One run over the 98 units takes 502 s at three jobs (`wall_s` of `tidy.sh`).
Findings are reported for `src/` and `tests/` only (`HeaderFilterRegex`) [S36].

**Decision.** `.clang-tidy` turns on these groups, each for a reason of correctness.
- `bugprone-*`: code that compiles and does the wrong thing (suspicious comparisons,
  moves, casts and macros).
- Eight `cert-*`: `cert-dcl58-cpp`, `-err34-c`, `-err52-cpp`, `-flp30-c`, `-mem57-cpp`,
  `-msc50-cpp`, `-oop57-cpp`, `-oop58-cpp`: unchecked number parsing, `rand()`, a
  float loop counter, copies that modify their source.
- `clang-analyzer-*`: null dereference, use after move, uninitialized read, division
  by zero, found by following paths through a function.
- Seven `cppcoreguidelines-*`: uninitialized members and variables, slicing, special
  member functions, virtual destructors, forwarding. Eight `misc-*`, of which
  `misc-definitions-in-headers` guards the one definition rule.
- `portability-*`, because `AGENTS.md` asks for the same behaviour on every platform.

A check that finds nothing is under `WarningsAsErrors`. One that finds something is
counted in `docs/architecture/baseline.txt`, may not grow, has a backlog item, and
moves under `WarningsAsErrors` in the commit that brings it to 0. Today 13 are counted:
`cert-err34-c` 60, `cppcoreguidelines-pro-type-member-init` 23, `-init-variables` 12,
`cert-flp30-c` 11, `cppcoreguidelines-special-member-functions` 5, five `bugprone-*`
(3, 3, 2, 2, 1), three `clang-analyzer-*` (2, 3, 1). There is no `NOLINT` (P9).

**Alternatives.** Left out of `.clang-tidy`, with places from `notes/counts-tidy.md`.
- `bugprone-unchecked-optional-access`, 879: 810 under `tests/` (30 files), 69 under
  `src/` (13). GoogleTest's `ASSERT_TRUE(x)` before `*x` is not understood by the
  check. Switching it off for `tests/` alone needs a `.clang-tidy` under `tests/`,
  which this run may not add; the backlog holds that as an item.
- `bugprone-easily-swappable-parameters` 68, `bugprone-implicit-widening-of-
  multiplication-result` 188, `bugprone-narrowing-conversions` 52 (held once by
  `-Wconversion` under `--config=strict`, ADR 0014).
- `bugprone-assignment-in-if-condition` 1, `-reserved-identifier`: style, naming.
- `cert-err33-c` 280 (ignored results of C library calls) and `cert-err58-cpp` 566
  (statics built at start-up): not defects of this code.
- `cppcoreguidelines-pro-bounds-*` and `-pro-type-vararg`, hundreds each (1,311 pointer
  arithmetic, 437 constant index, 248 array decay, 1,074 vararg): MuJoCo's C arrays
  and `printf`.
- `misc-non-private-member-variables-in-classes` 212: a style of class.
  `performance-*` and `concurrency-mt-unsafe` 65: not defects here.
- `clang-analyzer-optin.performance.Padding` 6; `clang-analyzer-security.
  FloatLoopCounter`: the places of `cert-flp30-c`, which is counted.
- GCC's `-fanalyzer`: its own notes say it is unlikely to help on C++ [S38].

The sanitizers are `--config=asan` and `--config=ubsan` in `.bazelrc`, for this
repository's files only. They stop a test at an out-of-bounds access, a use after free
or signed overflow and print the stack [S39]. They are run by hand
(`bazel test --config=opt --config=agent --config=ubsan //...`), not in CI: the slow
tests are slower still.

**Consequences.** A new defect of a clean check fails `tidy.sh`, naming the check. A
run costs 502 s, so it is run before handing work over (`ci.sh`), not in a loop; it
needs clang-tidy 18 and one build of the headers.

**Check.** P9 of `docs/architecture/principles.md`: `tools/architecture/tidy.sh`
(exit 1 on a fired `WarningsAsErrors` check or a count above baseline) and
`tools/architecture/ci.sh`. Append to `src/profile.cc`
```
int NullDeref() {
  int* p = nullptr;
  return *p;
}
```
then `tools/architecture/tidy.sh` exits 1 and prints `error: Dereference of null
pointer (loaded from variable 'p')
[clang-analyzer-core.NullDereference,-warnings-as-errors]`.

**Migration.** The backlog items that serve P9 in `docs/architecture/backlog.md`.
