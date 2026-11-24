# OMPL against the hand-written planner

The cell's planner (`src/motion_planner.h`) can search with its own RRT-Connect
(`src/rrt.h`) or with OMPL 2.0.2 (`src/ompl_search.h`): `MotionPlanOptions::search` in
code, `--planner own|ompl` on `run_cell` and `print_plan`, the own search being the
default. This document compares the two through that switch on the four moves
`print_plan` knows, and over whole pallets. The verdict follows a rule that was fixed
before any of these numbers existed (it is quoted under Verdict). The document states
what was measured; whether OMPL becomes the default is the owner's decision and nothing
here recommends for or against it.

Every figure below was printed by `print_plan` or `run_cell`; the ratios and percentages
were computed from those figures. How, where and when is under
[How this was measured](#how-this-was-measured).

## Verdict

The rule. The reference is the row `own`. The candidates are the rows `ompl`, `bitrrt`,
`bitstar-1x`, `aorrtc-1x` and `informed-1x`. A candidate qualifies on a move when (a) its
`found` is at least `own`'s; (b) its `clearance_min_mm` is at least the planning margin
less half the edge resolution and its `self_clearance_min_mm` at least the planning self
margin less half the edge resolution (30 mm and 10 mm here: `print_plan` prints the
margins, 45.0 and 25.0 mm, in its first line, and the resolution is 30 mm); (c) its
`queries_median` is at most 1.5 times `own`'s and its `ms_median` at most 3 times
`own`'s. Then:

- `OMPL better`: one and the same candidate qualifies on all four moves, its
  `duration_median_s` is at most 0.95 of `own`'s on at least one move and at most 1.01
  of it on every move, and the pallets run with OMPL place no fewer modules within
  tolerance than those run with the own search.
- `hand-written better`: no candidate qualifies on all four moves with a
  `duration_median_s` of at most 1.05 of `own`'s on every move.
- `no material difference`: otherwise.

`own` itself meets bound (b) on every move: its smallest clearance is 44.44, 44.60,
45.17 and 211.19 mm on the four moves against the 30 mm bound, and its smallest self
clearance 81.05, 81.05, 81.05 and 85.34 mm against 10 mm.

Verdict: hand-written better

The figures that decided it, from the tables under Results. "q" means the candidate
qualifies on that move; the last column is its `duration_median_s` over `own`'s on the
four moves in the tables' order.

| candidate | default | `--module 29 --slot 9` | `--other-way` | `--return` | qualifies on | duration over `own`'s |
|---|---|---|---|---|---|---|
| `ompl` | q | q | q | fails (c): `queries_median` 2735.0 against 664.0 (4.12 times), `ms_median` 24.19 against 5.80 (4.17 times) | 3 of 4 | 1.000, 1.000, 1.000, 1.000 |
| `bitrrt` | q | fails (c): `queries_median` 28916.0 against 18985.5 (1.52 times) | q | fails (c): 6624.5 against 664.0 (9.98 times), 58.85 ms against 5.80 (10.1 times) | 2 of 4 | 1.013, 0.994, 0.976, 1.000 |
| `bitstar-1x` | q | q | q | q | 4 of 4 | 0.977, 1.000, 1.053, 1.000 |
| `aorrtc-1x` | q | q | q | fails (c): 5290.5 against 664.0 (7.97 times), 45.82 ms against 5.80 (7.90 times) | 3 of 4 | 1.004, 1.008, 0.985, 1.000 |
| `informed-1x` | fails (a) and (c): found 45 of 50 | fails (a) and (c): found 35 | fails (a) and (c): found 41 | fails (a) and (c): found 3 | 0 of 4 | not applicable |

Only `bitstar-1x` qualifies on all four moves, and on `--other-way` its
`duration_median_s` is 6.037 s against `own`'s 5.734 s, 1.053 of it, which is above
1.05. So no candidate qualifies on all four moves within 1.05 of `own`'s duration on
every move, and the rule gives `hand-written better`. The pallets clause belongs to
`OMPL better` only and was not reached; the pallets are reported all the same.

Three things a reader should know about how that verdict came about. None of them is
part of the rule.

1. **The return decided against three candidates, by the straight line.** On `--return`
   the straight line from start to goal is free. The own search tries that line before
   it grows a tree (`RrtOptions::try_direct_first`) and is done after 664 collision
   queries, shortening included. OMPL's tree planners do not try it: `ompl` needs a
   median of 2735.0 queries and 24.19 ms where `own` needs 664.0 and 5.80 ms, and after
   shortening returns the same straight line (same length, clearance and duration in
   all 50 plans). That is 4.1 times the queries, above the rule's 1.5; in absolute
   terms it is 18 ms more for a move that takes 2.97 s. `BITstar` finds the direct
   edge first and matches `own` there, query for query.
2. **The deciding figure comes from a planner that does not repeat a plan.**
   `BITstar` draws part of its samples from a generator OMPL seeds from the clock, so
   the row `bitstar-1x` is one sample. The same command line run three more times
   printed a `duration_median_s` of 6.070, 6.048 and 5.984 s on `--other-way`: 1.059,
   1.055 and 1.044 of `own`'s. With the last of these in the table the rule would have
   given `no material difference`. The table holds the first run, as decided before
   the repeats were made. No run came near what `OMPL better` asks (at most 0.95 on one
   move and at most 1.01 on all four).
3. **On the three carries the switch as it ships changes nothing but the count of
   queries.** With the problem made equal, OMPL's `RRTConnect` returns the path the own
   search returns: in all 150 plans of the three carries (50 base seeds each, four
   seeds a plan) the waypoints, length, clearances and duration are the same to the
   last printed digit, and the queries differ by -95 to +101 a plan of some 19,000 to
   25,000. See "What is left unequal" for why it comes out so.

## Results

One table for each move. A row is what
`bazel run --config=opt //:print_plan -- <the row's flags> --summary` printed in its
two lines that do not start with `#`; the flags are listed under each table. The
columns: `plans` is the number of base seeds (50, or 10 in the `-10x` rows), each of
them one `PlanMove`, which plans four seeds and keeps the shortest unless
`--plan-seeds 1` is given; `found` how many of them found a path; `queries_*` the
collision queries of a plan, search and shortening together, and `ms_*` their computing
time, over all plans; `length_*` (metres of motion), `clearance_min_mm`,
`self_clearance_min_mm` and `duration_*_s` (the timed trajectory) over the plans found.

The rows:

- `own`: no new flag. What the cell uses, and the reference of the verdict.
- `own-1`: one seed a plan instead of four.
- `ompl`: the switch as it ships: OMPL's `RRTConnect`, range 3 m, 200,000 queries.
- `ompl-1`: the same with one seed a plan. Beside `own-1` it shows the two searches
  alone.
- `ompl-range`: OMPL's own choice of range in place of the planner's 3 m step.
- `bitrrt`: OMPL's `BiTRRT`.
- `bitstar-1x`: OMPL's `BITstar`, one seed a plan, with a budget of Q collision queries
  for the search, Q being `own`'s `queries_median` on that move to the nearest whole
  query. `bitstar-10x`: ten times Q, over 10 base seeds.
- `aorrtc-1x`, `aorrtc-10x`: OMPL's `AORRTC` by the same rule.
- `informed-1x`, `informed-10x`: OMPL's `InformedRRTstar` by the same rule.

The optimizing planners (`BITstar`, `AORRTC`, `InformedRRTstar`) run until their budget
is spent, so their `queries_median` is the budget plus the shortening that follows (and,
for `AORRTC`, see "What is left unequal"). A plan that has no path when the budget ends
is tried again with twice the budget, up to three rounds, as the own search is tried
again with twice the iterations; that is what the `queries_worst` of those rows shows.
Their rows do not repeat from run to run (see "What is left unequal"); `own`, `own-1`,
`ompl`, `ompl-1`, `ompl-range` and `bitrrt` do, in every column but the two `ms` ones.

### The default: the carry of module 0 to slot 0

| row | plans | found | queries_median | queries_worst | ms_median | ms_worst | length_median | length_worst | clearance_min_mm | self_clearance_min_mm | duration_median_s | duration_worst_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `own` | 50 | 50 | 25354.5 | 35166 | 241.85 | 338.46 | 18.569 | 20.533 | 44.44 | 81.05 | 6.167 | 7.350 |
| `own-1` | 50 | 50 | 6664.0 | 9534 | 63.29 | 94.14 | 19.563 | 30.280 | 44.44 | 81.05 | 6.779 | 9.705 |
| `ompl` | 50 | 50 | 25365.0 | 35196 | 243.90 | 343.12 | 18.569 | 20.533 | 44.44 | 81.05 | 6.167 | 7.350 |
| `ompl-1` | 50 | 50 | 6669.0 | 9544 | 63.21 | 93.86 | 19.563 | 30.280 | 44.44 | 81.05 | 6.779 | 9.705 |
| `ompl-range` | 50 | 50 | 29732.0 | 39492 | 287.45 | 387.95 | 18.541 | 19.636 | 45.13 | 81.05 | 6.248 | 7.441 |
| `bitrrt` | 50 | 50 | 36670.5 | 50233 | 357.60 | 487.09 | 18.623 | 19.937 | 44.64 | 81.05 | 6.245 | 7.521 |
| `bitstar-1x` | 50 | 50 | 31073.0 | 83629 | 311.75 | 831.05 | 18.588 | 19.624 | 45.11 | 81.05 | 6.028 | 7.584 |
| `bitstar-10x` | 10 | 10 | 258802.5 | 262676 | 3051.45 | 4788.74 | 18.449 | 19.512 | 45.00 | 81.05 | 6.120 | 6.922 |
| `aorrtc-1x` | 50 | 50 | 33143.5 | 41487 | 320.85 | 395.84 | 18.224 | 18.879 | 45.02 | 81.05 | 6.195 | 7.599 |
| `aorrtc-10x` | 10 | 10 | 254809.0 | 254896 | 2815.48 | 2963.88 | 17.997 | 18.022 | 45.02 | 81.05 | 5.860 | 6.523 |
| `informed-1x` | 50 | 45 | 80892.5 | 184610 | 809.75 | 1882.28 | 19.766 | 23.043 | 45.05 | 81.05 | 6.963 | 8.596 |
| `informed-10x` | 10 | 10 | 260660.0 | 955656 | 2706.41 | 10093.88 | 19.162 | 31.071 | 45.55 | 81.06 | 6.984 | 7.927 |

The command line of each row, after `bazel run --config=opt //:print_plan --`:

- `own`: `--summary`
- `own-1`: `--plan-seeds 1 --summary`
- `ompl`: `--planner ompl --summary`
- `ompl-1`: `--planner ompl --plan-seeds 1 --summary`
- `ompl-range`: `--planner ompl --ompl-range 0 --summary`
- `bitrrt`: `--planner ompl --ompl-planner BiTRRT --summary`
- `bitstar-1x`: `--planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 25355 --summary`
- `bitstar-10x`: `--planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 253550 --seeds 10 --summary`
- `aorrtc-1x`: `--planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 25355 --summary`
- `aorrtc-10x`: `--planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 253550 --seeds 10 --summary`
- `informed-1x`: `--planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 25355 --summary`
- `informed-10x`: `--planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 253550 --seeds 10 --summary`

### `--module 29 --slot 9`: the carry of module 29 to slot 9

| row | plans | found | queries_median | queries_worst | ms_median | ms_worst | length_median | length_worst | clearance_min_mm | self_clearance_min_mm | duration_median_s | duration_worst_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `own` | 50 | 50 | 18985.5 | 25919 | 160.13 | 217.55 | 16.833 | 17.018 | 44.60 | 81.05 | 5.138 | 5.873 |
| `own-1` | 50 | 50 | 4777.5 | 9111 | 40.10 | 76.15 | 16.946 | 18.315 | 45.23 | 81.05 | 5.201 | 6.548 |
| `ompl` | 50 | 50 | 18998.5 | 25945 | 160.88 | 217.88 | 16.833 | 17.018 | 44.60 | 81.05 | 5.138 | 5.873 |
| `ompl-1` | 50 | 50 | 4776.5 | 9123 | 40.09 | 75.26 | 16.946 | 18.315 | 45.23 | 81.05 | 5.201 | 6.548 |
| `ompl-range` | 50 | 50 | 22794.0 | 32044 | 192.20 | 275.09 | 16.844 | 17.026 | 45.01 | 81.05 | 5.240 | 6.592 |
| `bitrrt` | 50 | 50 | 28916.0 | 37858 | 243.11 | 315.30 | 16.829 | 17.127 | 44.19 | 81.05 | 5.108 | 5.961 |
| `bitstar-1x` | 50 | 50 | 24364.0 | 63704 | 209.88 | 539.10 | 17.126 | 19.210 | 44.57 | 81.05 | 5.140 | 7.110 |
| `bitstar-10x` | 10 | 10 | 194327.0 | 195849 | 1846.33 | 2783.28 | 17.065 | 17.769 | 44.72 | 81.05 | 5.492 | 6.221 |
| `aorrtc-1x` | 50 | 50 | 28054.0 | 34461 | 237.81 | 299.38 | 16.730 | 16.980 | 40.63 | 81.05 | 5.180 | 6.112 |
| `aorrtc-10x` | 10 | 10 | 190828.0 | 194692 | 1902.54 | 1982.10 | 16.655 | 16.678 | 39.91 | 81.15 | 5.015 | 5.821 |
| `informed-1x` | 50 | 35 | 133148.0 | 141997 | 1149.05 | 1222.10 | 17.192 | 18.060 | 44.01 | 81.05 | 5.004 | 5.795 |
| `informed-10x` | 10 | 10 | 386755.5 | 578989 | 3358.97 | 5197.86 | 17.427 | 17.656 | 45.32 | 81.06 | 5.230 | 5.705 |

The command line of each row, after `bazel run --config=opt //:print_plan --`:

- `own`: `--module 29 --slot 9 --summary`
- `own-1`: `--module 29 --slot 9 --plan-seeds 1 --summary`
- `ompl`: `--module 29 --slot 9 --planner ompl --summary`
- `ompl-1`: `--module 29 --slot 9 --planner ompl --plan-seeds 1 --summary`
- `ompl-range`: `--module 29 --slot 9 --planner ompl --ompl-range 0 --summary`
- `bitrrt`: `--module 29 --slot 9 --planner ompl --ompl-planner BiTRRT --summary`
- `bitstar-1x`: `--module 29 --slot 9 --planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 18986 --summary`
- `bitstar-10x`: `--module 29 --slot 9 --planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 189860 --seeds 10 --summary`
- `aorrtc-1x`: `--module 29 --slot 9 --planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 18986 --summary`
- `aorrtc-10x`: `--module 29 --slot 9 --planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 189860 --seeds 10 --summary`
- `informed-1x`: `--module 29 --slot 9 --planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 18986 --summary`
- `informed-10x`: `--module 29 --slot 9 --planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 189860 --seeds 10 --summary`

### `--other-way`: the carry of module 0 round the other side

| row | plans | found | queries_median | queries_worst | ms_median | ms_worst | length_median | length_worst | clearance_min_mm | self_clearance_min_mm | duration_median_s | duration_worst_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `own` | 50 | 50 | 25461.0 | 33788 | 261.55 | 353.37 | 18.347 | 19.205 | 45.17 | 81.05 | 5.734 | 6.599 |
| `own-1` | 50 | 50 | 5969.0 | 10293 | 65.07 | 110.97 | 18.670 | 20.383 | 45.15 | 81.05 | 5.874 | 6.954 |
| `ompl` | 50 | 50 | 25458.5 | 33828 | 298.77 | 420.91 | 18.347 | 19.205 | 45.17 | 81.05 | 5.734 | 6.599 |
| `ompl-1` | 50 | 50 | 5975.0 | 10300 | 62.70 | 106.79 | 18.670 | 20.383 | 45.15 | 81.05 | 5.874 | 6.954 |
| `ompl-range` | 50 | 50 | 29421.0 | 42728 | 302.67 | 446.70 | 18.284 | 18.766 | 45.21 | 81.05 | 5.692 | 7.063 |
| `bitrrt` | 50 | 50 | 35426.0 | 44382 | 363.53 | 454.14 | 18.320 | 18.985 | 45.05 | 81.05 | 5.597 | 6.426 |
| `bitstar-1x` | 50 | 50 | 31275.0 | 82903 | 335.53 | 883.73 | 18.535 | 19.645 | 39.79 | 81.05 | 6.037 | 7.509 |
| `bitstar-10x` | 10 | 10 | 260483.5 | 262204 | 3159.34 | 6933.83 | 18.504 | 19.223 | 46.88 | 81.05 | 5.962 | 6.641 |
| `aorrtc-1x` | 50 | 50 | 33699.0 | 41112 | 352.06 | 436.58 | 18.119 | 18.493 | 45.06 | 81.05 | 5.646 | 6.439 |
| `aorrtc-10x` | 10 | 10 | 256259.0 | 259195 | 3014.63 | 3180.18 | 17.991 | 18.044 | 45.07 | 81.05 | 5.930 | 6.471 |
| `informed-1x` | 50 | 41 | 82137.0 | 185995 | 1167.96 | 2670.67 | 18.553 | 20.644 | 45.01 | 81.05 | 5.873 | 7.818 |
| `informed-10x` | 10 | 9 | 769831.0 | 971547 | 8040.22 | 10058.70 | 18.640 | 21.642 | 45.12 | 81.05 | 5.572 | 6.007 |

The command line of each row, after `bazel run --config=opt //:print_plan --`:

- `own`: `--other-way --summary`
- `own-1`: `--other-way --plan-seeds 1 --summary`
- `ompl`: `--other-way --planner ompl --summary`
- `ompl-1`: `--other-way --planner ompl --plan-seeds 1 --summary`
- `ompl-range`: `--other-way --planner ompl --ompl-range 0 --summary`
- `bitrrt`: `--other-way --planner ompl --ompl-planner BiTRRT --summary`
- `bitstar-1x`: `--other-way --planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 25461 --summary`
- `bitstar-10x`: `--other-way --planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 254610 --seeds 10 --summary`
- `aorrtc-1x`: `--other-way --planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 25461 --summary`
- `aorrtc-10x`: `--other-way --planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 254610 --seeds 10 --summary`
- `informed-1x`: `--other-way --planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 25461 --summary`
- `informed-10x`: `--other-way --planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 254610 --seeds 10 --summary`

### `--return`: the bare arm's return

| row | plans | found | queries_median | queries_worst | ms_median | ms_worst | length_median | length_worst | clearance_min_mm | self_clearance_min_mm | duration_median_s | duration_worst_s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `own` | 50 | 50 | 664.0 | 664 | 5.80 | 6.03 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `own-1` | 50 | 50 | 664.0 | 664 | 5.79 | 6.06 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `ompl` | 50 | 50 | 2735.0 | 5255 | 24.19 | 46.54 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `ompl-1` | 50 | 50 | 2735.0 | 5255 | 24.68 | 47.20 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `ompl-range` | 50 | 50 | 4294.5 | 7681 | 37.82 | 67.91 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `bitrrt` | 50 | 50 | 6624.5 | 10590 | 58.85 | 96.50 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `bitstar-1x` | 50 | 50 | 664.0 | 664 | 5.93 | 6.06 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `bitstar-10x` | 10 | 10 | 664.0 | 664 | 5.82 | 5.93 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `aorrtc-1x` | 50 | 50 | 5290.5 | 9083 | 45.82 | 78.18 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `aorrtc-10x` | 10 | 10 | 5521.5 | 8170 | 47.95 | 70.67 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `informed-1x` | 50 | 3 | 4929.5 | 11402 | 45.23 | 102.08 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |
| `informed-10x` | 10 | 10 | 24772.0 | 26722 | 223.92 | 243.36 | 14.118 | 14.118 | 211.19 | 85.34 | 2.966 | 2.966 |

The command line of each row, after `bazel run --config=opt //:print_plan --`:

- `own`: `--return --summary`
- `own-1`: `--return --plan-seeds 1 --summary`
- `ompl`: `--return --planner ompl --summary`
- `ompl-1`: `--return --planner ompl --plan-seeds 1 --summary`
- `ompl-range`: `--return --planner ompl --ompl-range 0 --summary`
- `bitrrt`: `--return --planner ompl --ompl-planner BiTRRT --summary`
- `bitstar-1x`: `--return --planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 664 --summary`
- `bitstar-10x`: `--return --planner ompl --ompl-planner BITstar --plan-seeds 1 --ompl-queries 6640 --seeds 10 --summary`
- `aorrtc-1x`: `--return --planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 664 --summary`
- `aorrtc-10x`: `--return --planner ompl --ompl-planner AORRTC --plan-seeds 1 --ompl-queries 6640 --seeds 10 --summary`
- `informed-1x`: `--return --planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 664 --summary`
- `informed-10x`: `--return --planner ompl --ompl-planner InformedRRTstar --plan-seeds 1 --ompl-queries 6640 --seeds 10 --summary`

## Headroom

How much shorter a path anybody found: for each move the shortest `duration_median_s`
and the shortest `length_median` of any row, the `-10x` rows included, beside `own`'s.
Rows that found no path at all would not count; there is none.

| move | `own` duration_median_s | shortest of any row | row | shorter by | `own` length_median | shortest of any row | row | shorter by |
|---|---:|---:|---|---:|---:|---:|---|---:|
| default | 6.167 | 5.860 | `aorrtc-10x` | 5.0% | 18.569 | 17.997 | `aorrtc-10x` | 3.1% |
| `--module 29 --slot 9` | 5.138 | 5.004 | `informed-1x` | 2.6% | 16.833 | 16.655 | `aorrtc-10x` | 1.1% |
| `--other-way` | 5.734 | 5.572 | `informed-10x` | 2.8% | 18.347 | 17.991 | `aorrtc-10x` | 1.9% |
| `--return` | 2.966 | 2.966 | `own` | 0.0% | 14.118 | 14.118 | `own` | 0.0% |

The two columns are not won by the same row everywhere, and a shorter path is not
always a quicker one: the duration is set by the joints' speed limits along the path,
not by its length in metres of motion. With ten times the queries of `own`, `AORRTC`
shortens the median path by 1.1 to 3.1% on the three carries; the best median duration
of any row is 2.6 to 5.0% under `own`'s, and all three of those are from planners that
do not repeat a plan, over the 10, 35 and 9 plans they found. On the return every row
that found a path found the straight line.

## Whole pallets

`bazel run --config=opt //:run_cell -- --pretty --modules 30 --carry planned --trials 3`
with `--planner own` and with `--planner ompl`: three pallets of 30 modules each, the
pallet set down at random, the camera with noisy pictures, every carry planned.

| | `--planner own` | `--planner ompl` | from |
|---|---:|---:|---|
| modules placed | 90 of 90 | 90 of 90 | the summary's first line |
| modules within tolerance | 90 of 90 | 90 of 90 | not printed as a count; see below |
| carry, mean | 5.75 s | 5.75 s | the summary's `carry: mean` |
| cycle, mean | 23.81 s | 23.81 s | the summary's `cycle: mean` |
| planner's computing time for a carry, median | 263.0 ms | 263.5 ms | the median of the rows' `plan` column |
| planner's computing time for a carry, worst | 333 ms | 336 ms | the summary's `planning (wall): ... worst` |
| planned in time (during lift, pull back and clear) | 90 of 90 | 90 of 90 | the summary's `fits in ...` |
| smallest clearance of a planned path | 45.0 mm | 45.0 mm | the summary |

Missing: `run_cell` prints no count of modules within tolerance (the gate judges that,
`run_cell` does not). The figure above is derived from what it does print: for each of
the 90 rows of each run, the in-plane error (from the `along` and `across` columns), the
`yaw`, the `tilt` and the `height` are inside `JudgeCycle`'s default tolerance (3 mm,
0.15 degrees, 0.15 degrees, 2 mm) and the `rest` column says yes. The worst in either
run are 0.62 mm in the plane, 0.019 degrees of yaw, 0.000 degrees of tilt and 0.00 mm of
height. `run_cell` prints the planner's computing time for each carry and its mean and
worst, not its median; the median above is taken over the 90 rows.

The two runs differ in nothing the cell does: the carries are the same paths, and the
placement errors of the 90 rows agree to within 0.01 mm. The computing times are the
only difference, and they differ by less than this machine's noise.

## How the two were made equal

With OMPL chosen, `PlanMove` does what it does with its own search, except that the one
call of `PlanRrtConnect` is replaced by `PlanOmpl` (`src/ompl_search.h`) with the same
arguments. What OMPL would otherwise decide for itself is decided for it, each point
checked by a test of `//:ompl_search_test` (`tests/ompl_search_test.cc`):

| What | How it was done | Test |
|---|---|---|
| The box (R4) | OMPL plans in a `RealVectorStateSpace` whose bounds are the box `PlanMove` computed; every joint is an interval, none a circle. Every sampler OMPL allocates for the space samples in those bounds. | `OmplSearch.SamplesStayInTheBoxAndFillIt` |
| The metric (R5) | OMPL plans in the scaled coordinates `x_i = w_i * q_i`, `w` being the planner's weights (`CollisionChecker::LeverArms()`). Its Euclidean distance is then `cell::Distance(a, b, weights)`, and its range and path lengths are in metres of motion. | `OmplSearch.DistanceIsThePlannersMetric` (200 pairs, to 1e-9) |
| Edges (R6) | A motion validator of the file's own answers each of OMPL's edge questions by calling `cell::EdgeIsFree` with the planner's edge resolution (30 mm), weights and order: the same points, so the same verdict and the same number of collision queries. Both `checkMotion` functions are implemented. The margin budget of `src/motion_planner.h` therefore holds for a path of OMPL's. | `OmplSearch.EdgesCheckedLikeEdgeIsFree` (400 edges; on arm64 129 of them free with 20,226 queries, 271 blocked) |
| Settings (R7) | `OmplSettings`: the planner by name (`RRTConnect` the default, `BiTRRT`, `BITstar`, `AORRTC`, `InformedRRTstar`), the range (by default the planner's step of 3 m; 0 for OMPL's own choice), and a budget of collision queries for one search (200,000 by default). An unknown name is refused with a reason. | `OmplSearch.UnknownPlannerIsRefused`, `OmplSearch.RefusesWhatPlanRrtConnectRefuses` |
| Solutions and stopping (R8) | Only exact solutions are returned. A search ends when a planner that only seeks a path has one, when the budget of queries is spent, or when the time left to `PlanMove` has passed on `std::chrono::steady_clock`; the limits are asked for where OMPL asks whether to stop, without a timer thread (`solve(double)` is not used). The report says which ended the search. OMPL's messages are turned off. | `OmplSearch.EveryPlannerReturnsAnExactPathAndSaysWhatStoppedIt`, `OmplSearch.NoPathEndsAtTheQueryLimit` |
| Seeds (R9) | Every state sampler of the space is seeded with the seed `PlanMove` gives the k-th plan, the one the own search gets. OMPL's process-wide seed is not touched. `RRTConnect` and `BiTRRT` then repeat a single plan; the other three do not (below). | `OmplSearch.SameSeedSamePath`; `print_plan --planner ompl` twice (the acceptance criteria) |
| Threads (R10) | A search runs in the calling thread and starts none; each call builds its own OMPL objects. | `OmplSearch.ThreadsPlanningAtOnceGiveTheSamePaths` |
| The switch (R11) | `MotionPlanOptions::search` chooses the call; the box, the margins, the start and goal checks, the seeds, the rounds (OMPL's budget doubled each round as the iterations are), `ShortenPath`, the choice of the shortest, the resampling, the clearance measurement and the report are one code for both. | `OmplSearch.PlanMovePlansTheCarry`, `OmplSearch.OneCycleWithOmplPlaces` |

That the result is an equal problem shows in the tables: `own` and `ompl` are the same
algorithm given the same samples, and they return the same paths.

## What is left unequal

Each remaining difference, and which way it can push the figures.

- **The straight line first (`try_direct_first`).** The own search checks the straight
  line from start to goal before it grows a tree; OMPL's planners are not given that
  step, because it is part of `PlanRrtConnect` and not of the problem. Where the line is
  blocked (the three carries) it costs `own` a few queries. Where it is free (the
  return) `own` is done at once and OMPL's tree planners are not: against OMPL by 684
  to 4,591 queries a plan on the return (median 2,076), and that is what clause (c)
  failed on for `ompl`, `bitrrt` and `aorrtc-1x`. It does not change the path that
  comes out after shortening.
- **One more query for each step of the goal tree.** OMPL's `RRTConnect` (and `BiTRRT`)
  grows its goal tree by asking whether the new configuration is free and then whether
  the edge from it back to the tree is, and that edge's check ends on the tree's node,
  which is known to be free. The own search checks the edge from the node outward and
  never asks about the node again. Each accepted step of the goal tree so costs OMPL
  one query more; a blocked edge, checked from the other end, can cost either search
  more or fewer queries than the other. On the three carries the sum is -95 to +101
  queries a plan, with medians of +12 to +17: against OMPL, by under a tenth of a per
  cent.
- **The nearest-neighbour search.** A linear scan in the own search, a GNAT tree in
  OMPL. It changes the computing time, not the queries or the path. With trees of the
  size these moves need it does not show: `own-1` and `ompl-1` take 63.29 and 63.21 ms
  at the median on the default carry for the same work. For much larger trees it would
  favour OMPL.
- **How each planner spends its iterations or its budget.** The own search is limited
  by iterations (2,000, doubled each round), OMPL by collision queries (200,000,
  doubled each round). OMPL's limit was not reached by `ompl`, `ompl-range` or `bitrrt`
  on these moves: their worst plan spent 50,233 queries on four searches and their
  shortening. And since `own` and `ompl` returned the same paths from the same seeds,
  the difference in the kind of limit did not act here. The optimizing
  planners always spend their whole budget, which is why theirs was set from `own`'s
  queries: Q is what `own` spends on a whole plan of four seeds, shortening included,
  and they get it for one search, then are shortened like every other path. Their
  `queries_median` is therefore above Q by the shortening, which counts against them in
  clause (c) but within its factor of 1.5. `InformedRRTstar` spends its queries on
  rewiring its tree and often has no path when Q is spent: it found 45, 35, 41 and 3
  of 50, and its `queries_worst` shows the second and third rounds.
- **`AORRTC` overruns its budget and is checked twice.** It looks at its limit between
  whole searches of its inner planner, not within one: its `queries_median` is 31 to
  48% above Q on the carries and, on the return, where Q is only 664, eight times Q.
  It also runs OMPL's path simplifier inside its own loop, whose last step can
  leave a configuration where no check passed; `PlanOmpl` therefore checks a path of
  `AORRTC`'s once more with `EdgeIsFree` and counts those queries. Both push its
  queries up. Its simplifier is part of that planner; OMPL's simplifier is not run on
  any other planner's result.
- **Which OMPL planners repeat a single plan.** `RRTConnect` and `BiTRRT` draw from the
  seeded sampler only and repeat: the same seed, problem and settings give the same
  path in any process. `BITstar`, `AORRTC` and `InformedRRTstar` also draw from
  generators that OMPL 2.0.2 keeps private and seeds from the clock (an informed
  sampler, cost samples, a goal bias), so the seed fixes only their samples up to the
  first path, and their rows are a sample that a second run does not reproduce. This
  can push their figures either way; for the row that decided the verdict the spread
  over four runs is given under Verdict.
- **Time ended some searches of `InformedRRTstar`.** `PlanMove` has 10 s for a plan.
  The `ms_worst` of `informed-10x` is at that limit on the default carry and on
  `--other-way` (10,093.88 and 10,058.70 ms): those plans were ended by the clock, not
  by the budget, and on a slower or faster machine would end elsewhere. No other row
  reached it (the next longest plan is 6,933.83 ms, in `bitstar-10x` on `--other-way`).
- **OMPL's own range.** `ompl-range` lets OMPL choose its range, a fifth of the box's
  diagonal in the metric, in place of the planner's 3 m. It is in the tables as a row
  of its own: 16 to 20% more queries on the carries, median lengths and durations
  within 2% of `ompl`'s.
- **The compiler's settings.** OMPL is compiled like the rest of this repository, as
  C++20 at `-O2 -DNDEBUG`; its maintainers build and test it as C++17 at `-O3`. This
  can only make OMPL's computing times here longer than a build of its maintainers'
  kind would give; it does not touch queries, lengths or durations. With a collision
  query at about 9 microseconds, the queries are nearly all of the time for both
  searches.
- **The last bit.** OMPL's states are the joint angles times the weights, turned back
  by a division, which can move the last bit of a configuration; the ends of a path
  are set back to the start and goal given. The start and the goal are checked once by
  `PlanOmpl`, and OMPL's own repeat of that question is answered from memory without
  another query.
- **The objective.** Every OMPL planner is given the path's length in the planner's
  metric as what to minimize. `BiTRRT` is built for a cost that varies from place to
  place and is given none here.

## If OMPL became the default

Facts for that decision.

**What of this repository's planning code would still be needed.** `PlanMove` and
everything round the search (`src/motion_planner.h` and `.cc`, 369 and 558 lines): the
box, the margin budget, the seeds and rounds, the shortening, the clearance measured
afterwards. The collision checker and the timing (`src/collision.h`,
`src/path_timing.h`). `src/ompl_search.h` and `.cc` (270 and 563 lines) and the build
file for OMPL (`bazel/ompl.BUILD.bazel`, 126 lines). Of `src/rrt.h` and `.cc`: the
metric (`Distance`, `PathLength`), `Resample`, `EdgeIsFree`, which OMPL's edges are
checked with, `ShortenPath`, which shortens OMPL's paths too, and the types the search
is called with (`IsFree`, `JointPath`, `RrtOptions`, `RrtReport`, `RrtStatus`).

**What could be deleted.** The RRT-Connect itself: in `src/rrt.cc` the tree, the
planner and `PlanRrtConnect` (lines 87 to 224 and 370 to 485, 254 of the file's 565
lines); in `src/rrt.h` the declarations of `PlanRrtConnect` and `RrtTrees` (lines 290 to
315, 26 of 397 lines) with the parts of the opening comment and of `RrtOptions` that
describe RRT-Connect only (`goal_bias`, `max_iterations`, `try_direct_first`). Of the 21
tests of `tests/rrt_test.cc` (703 lines) 4 call `PlanRrtConnect`; `src/print_rrt.cc`
(332 lines) prints tables of it. The toy worlds (`src/rrt_worlds.h`) are used by
`tests/ompl_search_test.cc` as well.

**What would change in behaviour.** On the three carries nothing: the same paths (150
of 150 plans). On the return, and on any move whose straight line is free, the own
search's first step is missing: a median of 2735.0 queries and 24.19 ms in place of
664.0 and 5.80 ms, for the same path. Over three whole pallets the two placed the same
90 of 90 within the same errors, with the same carry and cycle times.

**What the build pays for OMPL.**

- The module graph gains 82 modules, all of them Boost (`boost` and 81 `boost.*`; the
  lock file names 150 modules where it named 68), and no module that was there before
  changes version.
- OMPL's release archive is 149,922,968 bytes. With Boost's modules the earlier run
  measured 253 MB downloaded for OMPL in all (`docs/ompl-integration.md`,
  Measurements, O1 native).
- Every target that plans links OMPL: `print_plan` grows from 4,397,784 to 6,017,240
  bytes.
- Build and test time on the engineer's machine (six arm64 cores, of which
  `--config=agent` uses three): a new clone with an empty disk cache builds
  and passes everything but the gate (`bazel test --config=opt --config=agent //...`)
  in 611.7 s, where `AGENTS.md` said about 6 minutes before OMPL; a second new clone
  that takes the build from the disk cache the first one filled, every test run again,
  in 148.8 s, about the 2.5 minutes it was; a third, with the tests' results from the
  cache too, in 9.0 s. The downloads were in Bazel's repository cache each time. The
  library alone, with the toy program of the earlier run, was a cold build of 147.5 s
  on six cores (`docs/ompl-integration.md`).
- CI has six jobs with a disk cache each, on two cores; each compiles OMPL once for its
  cache.

**What was and was not verified on x86-64.** Nothing of this was built or run on
x86-64 before it landed: not OMPL, not Boost's modules, not the search, not the test.
All figures here are from arm64 with GCC 14.2. OMPL and `//:ompl_search_test` were also
built and run with GCC 13.3 in an Ubuntu 24.04 container, the compiler and distribution
of CI's runners, but on arm64; that passed. The first build on x86-64 is the CI run
started by the push that lands this; its six jobs' results are in the check runs of that
commit. The tests assert no figure that depends on the platform's floating point; the
tables above do depend on it, and on x86-64 the paths of both searches may differ from
these in their digits.

## How this was measured

- **The code.** The code as it lands with this document: `src/`, `tests/`, `bazel/` and
  the build files as they are in this commit, measured on the branch
  `org/004-ompl-on-main-and-the` at commit `29d7b7b`. Only documents were changed
  after the measurement.
- **The date.** 2026-11-24.
- **The machine.** An Incus container on a shared host: Debian 13, aarch64, 6 cores,
  about 11 GB of memory shared with other containers; GCC 14.2.0, Bazel 9.3.0,
  `--config=opt` (`-O2 -DNDEBUG`, `-std=c++20`, for OMPL as for everything else).
- **The times** (`ms_*`, the planner's computing time over the pallets, the build
  times) are from that one shared arm64 machine, read from a monotonic clock, and are
  good to a factor, not to a few per cent: the same work was seen to take 261.55 and
  298.77 ms in two rows above. **The query counts, lengths, clearances and durations
  are exact for that machine**: for `own`, `own-1`, `ompl`, `ompl-1`, `ompl-range` and
  `bitrrt` the same command prints them again; for the rows of `BITstar`, `AORRTC` and
  `InformedRRTstar` they are what one run printed.
- **One at a time.** Each measured command ran alone, with no build and no other
  measured run going on the machine, one after the other from a script.

The commands, from the repository's top:

```
P="bazel run --config=opt //:print_plan --"

# A table's rows; MOVE is nothing, "--module 29 --slot 9", "--other-way" or "--return".
$P $MOVE --summary                                        # own
$P $MOVE --plan-seeds 1 --summary                         # own-1
$P $MOVE --planner ompl --summary                         # ompl
$P $MOVE --planner ompl --plan-seeds 1 --summary          # ompl-1
$P $MOVE --planner ompl --ompl-range 0 --summary          # ompl-range
$P $MOVE --planner ompl --ompl-planner BiTRRT --summary   # bitrrt
# Q is own's queries_median on the move, to the nearest whole query (a half goes up):
# 25355, 18986, 25461 and 664. NAME is BITstar, AORRTC or InformedRRTstar.
$P $MOVE --planner ompl --ompl-planner NAME --plan-seeds 1 --ompl-queries Q --summary               # -1x
$P $MOVE --planner ompl --ompl-planner NAME --plan-seeds 1 --ompl-queries 10Q --seeds 10 --summary  # -10x

# Plan against plan (Verdict, point 3; What is left unequal): the rows for each seed.
$P $MOVE
$P $MOVE --planner ompl

# Whole pallets.
bazel run --config=opt //:run_cell -- --pretty --modules 30 --carry planned --trials 3 --planner own
bazel run --config=opt //:run_cell -- --pretty --modules 30 --carry planned --trials 3 --planner ompl

# Line counts and sizes (If OMPL became the default).
wc -l src/rrt.h src/rrt.cc src/ompl_search.h src/ompl_search.cc src/motion_planner.h \
      src/motion_planner.cc src/print_rrt.cc tests/rrt_test.cc bazel/ompl.BUILD.bazel
grep -o '/modules/[^/"]*/' MODULE.bazel.lock | sort -u | wc -l    # 150; 68 at a091cc2
ls -lL bazel-bin/print_plan                                       # at this commit and at a091cc2

# Build and test time: a new clone, a disk cache that is empty, then filled.
bazel test --config=opt --config=agent --disk_cache=<empty directory> //...
bazel test --config=opt --config=agent --disk_cache=<the same> --nocache_test_results //...   # in a second new clone
bazel test --config=opt --config=agent --disk_cache=<the same> //...                           # in a third
```
