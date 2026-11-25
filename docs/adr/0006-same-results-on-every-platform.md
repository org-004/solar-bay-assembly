# 0006 A bound on random scenes takes its draws from `src/portable_random.h`

Status: accepted
Date: 2026-11-20
Decided by: the first engineer, commit 5600068 (the header); `AGENTS.md` took the rule
up in 4c5bf74 (2026-11-23)

**Context.** Development machines are 64-bit ARM and CI is x86-64. Two things made a
random scene differ between them. The standard fixes the sequence of
`std::mt19937` but leaves the distributions to each library (`[S12]`): libstdc++,
libc++ and Microsoft's give different numbers for the same seed
(`src/portable_random.h:3-7`). And in `Vector2d(unit(rng), unit(rng))` the standard
leaves the order of the two draws to the compiler: GCC evaluates right to left on
x86-64 and left to right on ARM, so the same seed gives x and y swapped
(`src/portable_random.h:14-19`). The body of `5600068` records the effect: "seed 1" was
a different scene on CI, "where two bounds set from three local scenes failed". A
second failure was a time: `ae3bb74` records that the median planning time was
asserted under 1 s and a shared CI runner measured 1.05 s.

**Decision.** `src/portable_random.h` replaces the library distributions
(`std::uniform_real_distribution`, `std::normal_distribution`,
`std::uniform_int_distribution`) with draws written out on the generator's raw
outputs, each bit for bit what libstdc++ (GCC 11 to 16) gives, so code moved onto them
draws what it drew before. Each draw is in a statement of its own. A bound that rests
on particular random scenes takes its draws from it (`AGENTS.md`, "The same on every
platform"); a test bounds a count, not a wall-clock time. Users today are
`src/rectangle_fit.cc:13`, `src/layout_plan.cc:25`, `src/layout_check.cc:32`,
`tests/rectangle_fit_test.cc:24` and `tests/ompl_search_test.cc:36`.
`src/layout_plan.h:112-118` plans with iteration budgets only, "no time limit". The
header came in `5600068` (2026-11-20), the count bound in `ae3bb74` (the same day).

**Alternatives.** The repository records no alternative that was weighed.

**Consequences.** One draw is not bit-identical everywhere: `Normal()` calls `std::log`,
which may differ in the last bit between math libraries; the header says that changes a
scene by a part in 10^16, not which scene it is. The code still departs from the
decision. `tools/architecture/measure.sh --list P6` prints 13 files, among them
`src/rrt.cc` (a `std::uniform_real_distribution` at line 442 and a search stopped on
wall-clock time at line 451), `src/point_generator.cc`, `src/conveyor.cc`,
`src/plane_fit.cc` and `tests/rrt_test.cc` (`EXPECT_LT(seconds, 2.0)` at line 373).
Until they move, a plan made on one machine is not the plan made on another
(`docs/flat-rack-cell.md:295-296`), and a comparison of captured output is made on
one machine.

**Check.** P6 of `docs/architecture/principles.md`: `measure.sh` line `P6`. Today
`tools/architecture/measure.sh | grep '^P6 '` prints `P6 13 files that use what differs
between platforms or runs outside src/portable_random.h, or that read steady_clock for
more than a print`, and `P6.frozen` counts seven of them in frozen files.

**Migration.** The backlog items that serve P6 in `docs/architecture/backlog.md`.
