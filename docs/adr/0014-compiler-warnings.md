# 0014 Warnings for correctness are errors where the front end decides and code is clean

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R28)

**Context.** Bazel's toolchain passes `-Wall`, so its warnings were printed, never
errors. Each flag was counted at `940a01b` over the 98 translation units of `src/` and
`tests/` under GCC 14.2 (Debian 13) and GCC 13.3 (Ubuntu 24.04, Docker), in places
inside those directories and outside (Eigen, MuJoCo, system); inside, both compilers
find the same places. The 29 flags of the default line are decided by the front end
and find nothing inside, nor in the headers included, under both compilers. All 98
units parse with Ubuntu's x86-64 cross compiler (`x86_64-linux-gnu-g++
-fsyntax-only`) and no flag fires: the nearest proof of CI's platform. With the flags
on, the object files are the same bytes and Bazel reran no test. [S34] advises a
selective `-Werror=<flag>` and a counted baseline.

**Decision.** `.bazelrc` makes these 29 flags errors in every build, for files
matching `^(src|tests)/.*\.cc$` through `--per_file_copt`; no dependency is compiled
again. Each line is a group and what it catches, all as `-Werror=`:
- A path without a return, a missed enumerator, a fall-through, a misleading block:
  `return-type`, `switch`, `implicit-fallthrough`, `misleading-indentation`,
  `empty-body`, `sequence-point`, `logical-not-parentheses`.
- A test that is always true or false, or compares the wrong thing: `sign-compare`,
  `address`, `bool-compare`, `tautological-compare`, `int-in-bool-context`,
  `logical-op`, `duplicated-cond`.
- A result thrown away: `unused-value`, `unused-result`. A misspelled macro read as 0:
  `undef`. A format that does not match its arguments: `format`, `format-security`.
- A class that is destroyed or initialized wrongly: `reorder`,
  `delete-non-virtual-dtor`, `non-virtual-dtor`, `suggest-override`.
- A wrong size or raw write: `sizeof-pointer-memaccess`, `memset-transposed-args`,
  `sizeof-array-div`, `class-memaccess`, `vla`, `cast-qual`.

`--config=strict` adds twelve flags as warnings. `tools/architecture/warnings.sh`
prints `W <flag> <places>`; the counts are lines of `docs/architecture/baseline.txt`
and may not grow. Today: `-Wconversion` 61 (not sign), `-Wmissing-field-initializers`
11, `-Wshadow` 8, `-Wparentheses` 4, `-Wcomment` 3: they find something.
`-Wnull-dereference` 2: the optimizer decides it. `-Wcast-align` 0 and `-Wtype-limits`
0: the platform decides, since alignment rules and the sign of `char` differ between
arm64 and x86-64. `-Wold-style-cast` 0, `-Woverloaded-virtual` 0 and
`-Wduplicated-branches` 0 fire in outside headers (113, 3, and 5 under 14.2 or 4 under
13.3 places). `-Wdangling-reference` 0: a heuristic that changed between GCC versions.
A flag whose count is 0 and that the front end decides moves to the default line as
`-Werror=` in the commit that brings it to 0. Nothing is silenced in source (P9).

**Alternatives.** Weighed and left out; places inside `src/`+`tests/` (outside).
- `-Wsign-conversion`, 1,253 (109): index arithmetic between `int` and `size_t` for
  Eigen and MuJoCo. A backlog that large is one nobody should work through, and a
  sign error at run time is what `--config=ubsan` and `--config=asan` are for.
- `-Wfloat-equal`, 53 (8). Five opened are exact-zero guards: `src/profile.cc:57`, `:87`
  (`d == 0.0`, an empty move), `src/joint_move.cc:22` (`delta[i] != 0.0` before a
  division), `src/cell.cc:48` (`range == 0.0`), `src/portable_random.h:90`
  (`r2 == 0.0` before `std::log`).
- `-Wdouble-promotion`, 27 (1): no figure is lost by widening. `-Wswitch-enum`, 4,
  and `-Wswitch-default`, 22: `-Werror=switch` already refuses a missed enumerator
  where there is no `default`.
- `-Wformat=2`, 48 `-Wformat-nonliteral` in wrappers of `vsnprintf`;
  `-Wformat-truncation=2`, 68, decided by the optimizer; `-Wuseless-cast`, 4 (45);
  `-Wzero-as-null-pointer-constant`, 0 (27); `-Wpedantic` finds nothing: it judges
  conformance, not defects.
- `-Wmaybe-uninitialized`, `-Warray-bounds` and the other optimizer warnings of
  `-Wall` stay warnings: their answer moves with the compiler's version and the
  optimization level [S34].
- A blanket `-Werror`: it would stop on outside headers [S34]. Silencing in source:
  it lowers a count and cures nothing.

**Consequences.** A missed return or enumerator fails the build. A flag enters only
with a count; one that an outside header makes fire moves to `strict` by a new record.

**Check.** P9 of `docs/architecture/principles.md`: `tools/architecture/warnings.sh
--check` exits 1 when a count is above the baseline. Append to `src/profile.cc`
```
int NoReturn(int x) {
  if (x > 0) return 1;
}
```
then `bazel build --config=opt //:profile` fails with `error: control reaches end of
non-void function [-Werror=return-type]`; `git checkout src/profile.cc` restores it.

**Migration.** The backlog items that serve P9 in `docs/architecture/backlog.md`.
