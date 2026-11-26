# 0008 OMPL is a second search behind a switch; the own RRT-Connect is the default

Status: accepted
Date: 2026-11-24
Decided by: the engineer of run 004, commit 9397bdd

**Context.** The cell's planner (`src/motion_planner.h`) searched for a path with the
RRT-Connect of `src/rrt.h`, written for this cell. OMPL, the Open Motion Planning
Library, offers that planner and others, and the project wanted to know whether any of
them does better on the cell's moves. OMPL is not in the Bazel Central Registry, so
bringing it in needed a decision of its own: `docs/ompl-integration.md` built four ways
of doing it (added in 13678be, 2026-11-24) and assessed four on paper. The planner is
shared: `TrialSettings` in `src/trial_runner.h:79` carries the choice of search to whole
pallets, and it is `kOwn` there too.

**Decision.** `MotionPlanOptions::search` (`src/motion_planner.h:194`) chooses between
`PlanSearch::kOwn`, the own RRT-Connect, and `PlanSearch::kOmpl`, OMPL's planner set up
by `src/ompl_search.h`. It is `kOwn` unless set: `run_cell` and `print_plan` start with
`kOwn` (`src/run_cell.cc:432`) and take `--planner own|ompl`. Everything round the one
call, the margins, the shortening, the rounds and the report, is the same code for both
(`src/motion_planner.h:98-111`), and with the switch left alone nothing of OMPL runs.
OMPL is built from the release archive fetched by hash, with its own build file
`bazel/ompl.BUILD.bazel`, and reached through one target, `//:ompl_search`
(`BUILD.bazel`, section `# BEGIN ompl`). The comparison is `docs/ompl-comparison.md`;
its verdict, by a rule fixed before any figure existed, is "hand-written better".
Whether OMPL becomes the default is left to the owner (`docs/ompl-comparison.md:6-10`)
and the repository records no such decision: the default stays as it was.

**Alternatives.** `docs/ompl-integration.md` weighs eight ways to bring OMPL in. O1
builds it natively from an archive by hash with a build file of the repository's own; O2
builds it through `rules_foreign_cc`; O3 takes it from the ROS registry; O4 makes a
module of it in a registry of the project's; O5 puts that module in the central
registry; O6 copies the source into the tree; O7 uses a system package; O8 stays outside
Bazel. O1 was recommended (`docs/ompl-integration.md:16-46`): with O4 it left the
existing build exactly as it was and needed no patch, and of the two it has no registry
to keep. O2 moved three module versions and compiled OMPL at `-O3`; O3 moved sixteen and
rebuilt the viewer's libraries. O4 is the runner-up, better on the day a second project
needs OMPL. For the default, the comparison's rule (`docs/ompl-comparison.md`,
"Verdict") names OMPL's RRT-Connect, BiTRRT, BIT*, AORRTC and Informed RRT* as
candidates, and the own search won because no candidate qualified on all four moves with
a median duration within 1.05 of the own search's. The repository records no other
reason for keeping the own search as the default.

**Consequences.** Nothing about the stand-in cell or its gate changed: no existing test
was edited and the rows `print_plan` printed for each seed are the bytes they were. The
price is that every target that plans links OMPL and builds it: no target is tagged
manual (`BUILD.bazel`, section `# BEGIN ompl`), so a fresh build is longer. Two of the
report's recommendations were not done: a CI query that only manual targets depend on
OMPL, and a CI job for the OMPL files (decisions 5 and 6,
`docs/ompl-integration.md:1387-1398`).

**Check.** P4, in `tools/architecture/measure.sh`: `P4.ompl` counts the targets that
list `@ompl` and may not grow, and `P4.group` counts includes of OMPL from a group that
may not. `tools/architecture/measure.sh --list P4.ompl` prints `//:ompl_search`, one
target. `grep -n 'PlanSearch search' src/motion_planner.h` prints `194:  PlanSearch
search = PlanSearch::kOwn;`.

**Migration.** None: the code already follows this.