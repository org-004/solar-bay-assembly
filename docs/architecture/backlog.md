# Backlog: where the code departs from the principles

Each item below is one commit's work for one agent: a change that brings the code
closer to a principle of [`principles.md`](principles.md) or a decision under
[`docs/adr/`](../adr/README.md) without changing what any program prints or any test's
result. Take the first item whose **State** is `open` and whose **After** items are
`done`, and do it by the procedure of [`README.md`](README.md); what "behavior" means is
[`behavior.md`](behavior.md). The figures are those of `tools/architecture/measure.sh`
at the commit this file lands with; `tools/architecture/warnings.sh` prints the `W`
lines and `tools/architecture/tidy.sh` the `T` lines. A **Done when** figure assumes
that the items above it have landed. So do the line numbers: an earlier item that edits
the same file can move them by a few lines, and the text quoted finds the place.

Every item also changes its own **State** line in this file and
`docs/architecture/baseline.txt`, and most change a **Now** line in `principles.md`
(every one that holds a figure that moved); the last two are listed in **Touches**, and
two items that share no other file can run side by side and merge those two by running
the command of step 7 of `README.md` again. A `BUILD.bazel` entry names its section
(`# BEGIN x`). When an item removes a departure that `tools/architecture/deps.json`
lists, its entry there goes in the same commit. No item edits a frozen file
(`tools/architecture/measure.sh --list P1.frozen_files`, ADR 0020).

An item that adds a test target or a line of `tools/architecture/programs.txt` widens
the net: `behavior.sh compare` of the captures before and after then differs by exactly
that target in `tests.txt` or that program's file, and the proof is instead two
captures after the change that compare `same`. Such an item says so under **Done when**.

## B1 Remove the four unused deps of the layout's, the rows' and the bay's targets

**Principles.** P5
**Decisions.** ADR 0018
**Where.** `//:layout_model`, `//:layout_rows_test`, `//:bay_model_test`,
`//:plan_replay_test` in `BUILD.bazel`.
**Evidence.** `measure.sh --list P5` lists `//:layout_model //:container`,
`//:layout_rows_test //:container`, `//:bay_model_test eigen`,
`//:plan_replay_test nlohmann_json`. Opened: `src/layout_model.cc` and `.h` name nothing
of `src/container.h` (`grep -n 'container::' src/layout_model.*` prints nothing);
`tests/layout_rows_test.cc` includes `src/flat_rack.h`, not `src/container.h`;
`tests/bay_model_test.cc` names no `Eigen` (`grep -c Eigen` prints 0);
`tests/plan_replay_test.cc` includes no `nlohmann/` header. None includes through a
macro (`grep -n '#define'` in the four files finds the include guard of
`src/layout_model.h` and `NEED_COMMITTED` in `tests/plan_replay_test.cc:470`, which
names no header).
**Change.** In `BUILD.bazel` delete the line `":container",` from the `deps` of
`layout_model` (section `layout`) and of `layout_rows_test` (section `layout_rows`),
`"@eigen",` from `bay_model_test` (section `bay`) and `"@nlohmann_json//:json",` from
`plan_replay_test` (section `layout_plan`). Delete the four matching entries of
`departures` in `tools/architecture/deps.json`.
**Done when.** `measure.sh` prints `P5 12`, `P5.unused_project 8`,
`P5.unused_outside 2`, `P4.eigen 77`, `P4.nlohmann_json 3`, `P1.listed 16`;
`python3 tools/architecture/deps.py` prints ok; `bazel build --config=opt //...`
passes; `behavior.sh compare` prints `same`.
**Net.** The four test targets themselves (their results are in the capture);
`//:layout_model` is in the deps of `layout_check_test`, `layout_model_test`,
`layout_plan_test`, `layout_rows_test`, `layout_scene_test`, `plan_replay_test` and of
the captures `check_layout.*` and `plan_layout.*`. Nothing to pin first.
**Size.** 2 files, 4 lines of `BUILD.bazel` and 4 of `deps.json` deleted.
**Touches.** `BUILD.bazel` (sections `layout`, `layout_rows`, `bay`, `layout_plan`),
`tools/architecture/deps.json`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P4 and P5 **Now**).
**After.** None.
**State.** open

## B2 Include src/mj_handles.h where DataPtr is named, and drop it from viewer_lib

**Principles.** P5
**Decisions.** ADR 0018
**Where.** `//:layout_plan`, `//:layout_scene_test`, `//:viewer_lib`.
**Evidence.** `measure.sh --list P5` lists `//:layout_plan //:mj_handles`,
`//:layout_scene_test //:mj_handles`, `//:viewer_lib //:mj_handles`.
`src/layout_plan.cc` (lines 712, 1043, 1356), `src/layout_replay.cc` (1637, 1651)
and `tests/layout_scene_test.cc` (91, 115, 164, 174) name `DataPtr` of
`src/mj_handles.h` but reach it only through `src/layout_scene.h:136`.
`src/viewer_lib.cc` and `.h` name nothing of it
(`grep -n 'DataPtr\|ModelPtr\|MakeData\|LoadModel' src/viewer_lib.*` prints nothing).
**Change.** Add `#include "src/mj_handles.h"` to the project includes of
`src/layout_plan.cc`, `src/layout_replay.cc` and `tests/layout_scene_test.cc`, in
alphabetical place. In `BUILD.bazel`, delete `":mj_handles",` from the `deps` of
`viewer_lib` (line 656, between `# END cell` and `# BEGIN pick_place`, in no
section). Delete the three entries of `departures` in `deps.json`.
**Done when.** `measure.sh` prints `P5 9`, `P5.unused_project 5`, `P1.listed 13`;
`deps.py` prints ok; `bazel build --config=opt //...` passes; `behavior.sh compare`
prints `same`.
**Net.** `layout_plan_test`, `plan_replay_test`, `layout_scene_test`; capture
`plan_layout.*`. `src/viewer_lib.cc` is outside the net (`P2`), but only its deps
change and the build proves them.
**Size.** 4 files, 3 lines added, 4 deleted.
**Touches.** `src/layout_plan.cc`, `src/layout_replay.cc`,
`tests/layout_scene_test.cc`, `BUILD.bazel` (`viewer_lib`, in no section),
`tools/architecture/deps.json`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P5 **Now**).
**After.** B1
**State.** open

## B3 List @mujoco in the deps of layout_plan_test

**Principles.** P5, P4
**Decisions.** ADR 0018
**Where.** `//:layout_plan_test`.
**Evidence.** `measure.sh --list P5.missing` prints `//:layout_plan_test @mujoco`;
`tests/layout_plan_test.cc:11` includes `<mujoco/mujoco.h>` and calls `mj_kinematics`
(line 92), and its target lists no `@mujoco`.
**Change.** In `BUILD.bazel`, section `layout_plan`, add `"@mujoco",` after
`"@googletest//:gtest_main",` in the `deps` of `layout_plan_test`; delete the
`missing_dep` entry of `departures` in `deps.json`. `P4.mujoco` counts the targets
that list MuJoCo and rises by one: this target already used it through
`//:layout_plan`, so no new dependency comes in, and the commit message says so.
**Done when.** `measure.sh` prints `P5 8`, `P5.missing 0`, `P4.mujoco 68`,
`P1.listed 12`; `deps.py` prints ok; `behavior.sh compare` prints `same`.
**Net.** `layout_plan_test` itself.
**Size.** 2 files, 1 line added, 1 deleted.
**Touches.** `BUILD.bazel` (section `layout_plan`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P4 and P5
**Now**).
**After.** B2
**State.** open

## B4 Mark the gone program in docs/layout.md as gone

**Principles.** P10
**Decisions.** ADR 0001
**Where.** `docs/layout.md`, lines 667 and 668.
**Evidence.** `measure.sh --list P10` prints `docs/layout.md:667 //:print_layout_moves`;
`docs/layout.md:11` already says the program no longer exists (it left in run 012).
**Change.** Replace the sentence "The command is `bazel run --config=opt
//:print_layout_moves -- below_rails_by_module_arm --iterations 40000`; it runs with a
fixed seed (1) and prints the same rows every run." by "The program was
`//:print_layout_moves` (gone), run with `below_rails_by_module_arm --iterations 40000`
and a fixed seed (1); it printed the same rows every run." Keep each backtick span on
one line. Nothing else in the file changes.
**Done when.** `measure.sh --list P10` no longer lists `docs/layout.md:667` and `P10`
is one lower than before. A change to documents alone needs no capture.
**Net.** None needed: no code.
**Size.** 1 file, 2 lines.
**Touches.** `docs/layout.md`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P10 **Now**).
**After.** None.
**State.** open

## B5 Let src/collision.cc find the pallet's bodies itself, not through src/pallet.h

**Principles.** P1
**Decisions.** ADR 0017, ADR 0020
**Where.** `src/collision.cc`, `//:collision`.
**Evidence.** `measure.sh --list P1` prints `src/collision.cc -> src/pallet.h`, the one
departure outside frozen files. `src/collision.cc:17` includes it for one call:
`FindPallet(m)` at line 102, of which it keeps `pallet->modules` (the bodies
`module_<i>`) and `pallet->pallet_mocap`. `FindPallet` (`src/pallet.cc:52`) returns
nothing unless body `pallet` exists and is a mocap body, bodies `module_0` to
`module_<n-1>` exist with no gap, and each has exactly one free joint.
**Change.** In the anonymous namespace of `src/collision.cc` add
`struct PalletIds { int mocap = -1; std::vector<int> modules; };` and
`std::optional<PalletIds> FindPalletIds(const mjModel& m)`, which applies exactly the
tests of `FindPallet` listed above, in its order (copy the name parsing of
`ModuleIndex`, `src/pallet.cc`, into it), and returns the two fields; it reads no
`body_pos`, `body_quat` or `qpos0`. Call it at line 102 in place of `FindPallet`, delete
`#include "src/pallet.h"`, delete `":pallet",` from the `deps` of `collision`
(`BUILD.bazel`, section `collision`) and the `include` departure in `deps.json`. A
comment above the function says that it mirrors `FindPallet` so that the arm's stack
does not depend on the stand-in cell, and goes when the stand-in goes.
**Done when.** `measure.sh` prints `P1 4` (equal to `P1.frozen`), `P1.listed 11`;
`deps.py` prints ok; `behavior.sh compare` (with the tests) prints `same`.
**Net.** `collision_test`, `motion_planner_test`, `ompl_search_test`, `pick_place_test`,
`planned_carry_test`, `camera_cycle_test`, `check_cycle_test`, `trial_runner_test`;
captures `print_plan.*`, `run_cell.*`, `print_check.*`. Capture with the tests.
**Size.** 3 files, about 40 lines added, 3 deleted.
**Touches.** `src/collision.cc`, `BUILD.bazel` (section `collision`),
`tools/architecture/deps.json`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P1 **Now**).
**After.** None.
**State.** open

## B6 Draw the RRT's samples through src/portable_random.h

**Principles.** P6
**Decisions.** ADR 0006
**Where.** `src/rrt.cc`, lines 442, 460, 462, 522 to 524.
**Evidence.** `measure.sh --list P6` lists `src/rrt.cc`; `src/rrt.cc:442` builds
`std::uniform_real_distribution<double> unit(0.0, 1.0)` and `:522` one over
`(0.0, total)`. `src/portable_random.h:1` to `:12` states that `UniformUnit` and
`UniformReal` give, bit for bit, what libstdc++ (GCC 11 to 16) gives for the same call
on the same generator, so on this toolchain the draws do not change.
**Change.** Include `src/portable_random.h` in `src/rrt.cc` and add
`":portable_random",` to the `deps` of `rrt` (`BUILD.bazel`, section `rrt`). Delete line
442; at 460 write `sample[j] = lower[j] + UniformUnit(rng) * (upper[j] - lower[j]);` and
at 462 `UniformUnit(rng) < options.goal_bias`; delete line 522 and write
`double s1 = UniformReal(rng, 0.0, total);` and `double s2 = UniformReal(rng, 0.0,
total);` on two lines. Keep one draw per statement. Leave the clock of line 451 alone
(it is for the owner, "Needs the owner").
**Done when.** `grep -c _distribution src/rrt.cc` prints 0; `measure.sh` still lists
`src/rrt.cc` under `P6.clock` and prints `P6 13`; `behavior.sh compare` (with the tests)
prints `same`. If it differs, the draws were not identical: stop and report.
**Net.** `rrt_test`, `motion_planner_test`, `ompl_search_test`, `layout_plan_test`,
`plan_replay_test` and the cycle tests; captures `print_rrt.*`, `print_plan.*`,
`run_cell.*`, `plan_layout.*`.
**Size.** 2 files, about 6 lines.
**Touches.** `src/rrt.cc`, `BUILD.bazel` (section `rrt`),
`docs/architecture/baseline.txt` (no figure moves; nothing to change unless a line
did).
**After.** None.
**State.** open

## B7 Draw the random scenes of three tests through src/portable_random.h

**Principles.** P6
**Decisions.** ADR 0006
**Where.** `tests/ik_test.cc` (189, 195, 952), `tests/path_timing_test.cc` (681),
`tests/rrt_test.cc` (209, 212).
**Evidence.** `measure.sh --list P6` lists the three files. `tests/ik_test.cc:189`
draws the angles behind `EXPECT_GE(rate, 0.85)` with `std::uniform_real_distribution`,
against `AGENTS.md`, "The same on every platform"; `tests/rrt_test.cc:212` takes two
draws in one expression, `Vec({coordinate(rng), coordinate(rng)})`.
**Change.** Include `src/portable_random.h` in the three tests and add
`":portable_random",` to the `deps` of `ik_test` (section `ik`), `path_timing_test`
(section `path_timing`) and `rrt_test` (section `rrt`). Replace each
`std::uniform_real_distribution<double>(a, b)` object or call by `cell::UniformReal(rng,
a, b)` at the place of the call, and the `std::normal_distribution<double> normal` of
`path_timing_test.cc:681` by `cell::NormalDraw normal;` (same call syntax,
`normal(generator)`). At `rrt_test.cc:212` take `const double x =` and
`const double y =` in two statements, x first, and build the vector from them: this
keeps the order that GCC on 64-bit ARM uses today.
**Done when.** `measure.sh` prints `P6 11` (`tests/ik_test.cc` and
`tests/path_timing_test.cc` leave the list; `tests/rrt_test.cc` stays for its clock);
the three tests pass; `behavior.sh compare` (with the tests) prints `same`.
**Net.** The three tests themselves: their results are the net. Should one fail, the
draws differed: stop and report.
**Size.** 4 files, about 15 lines.
**Touches.** `tests/ik_test.cc`, `tests/path_timing_test.cc`, `tests/rrt_test.cc`,
`BUILD.bazel` (sections `ik`, `path_timing`, `rrt`), `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P6 **Now**).
**After.** B6
**State.** open

## B8 Pin the viewer's argument parsing with a test

**Principles.** P2
**Decisions.** ADR 0021
**Where.** `src/viewer_lib.cc` (`ParseViewerArgs`, `ViewerArgsUsage`), a new test.
**Evidence.** `measure.sh --list P2` lists `src/viewer_lib.cc` and `src/viewer_lib.h`;
`behavior.md`, "Outside the net", says the parsing can be tested without a display.
`src/viewer_lib.cc` 214 to 326 parse `--no-overlay`, `--lookat`, `--distance`,
`--azimuth`, `--elevation`, `--speed`, `--fps`, `--overlay`, `--overlay-at`,
`--caption`, `--font-scale`, `--size`, `--record`, `--quit-at`, `--key`, `--groups`.
**Change.** Write `tests/viewer_lib_test.cc` (new): for each option one case that it is
taken (the field of `ViewerOptions` or `ViewerRun` holds the value) and one that a bad
value is refused with the message `ParseViewerArgs` gives today (read it from the
source, do not change it), and one case for an unknown option. Add `cc_test(name =
"viewer_lib_test", size = "small", srcs = ["tests/viewer_lib_test.cc"], deps =
[":viewer_lib", "@googletest//:gtest_main"])` after `viewer_lib` (in no section), and
`tests/viewer_lib_test.cc` (new) to the files of group `viewer` in `deps.json`. Then
change one parsed value by hand in `src/viewer_lib.cc`, see the test fail, and put it
back.
**Done when.** `//:viewer_lib_test` (new) passes; `measure.sh` prints `P2 4`,
`P2.untested 22`, `P4.googletest 43` (a new test target lists GoogleTest); two
captures after the change compare `same`, and against the capture before only
`tests.txt` differs, by the new target.
**Net.** This item is the net for `src/viewer_lib.*`.
**Size.** 3 files, about 150 lines added.
**Touches.** `tests/viewer_lib_test.cc` (new), `BUILD.bazel` (after `viewer_lib`),
`tools/architecture/deps.json`, `docs/architecture/behavior.md` (section 6a, the row of
`src/viewer_lib.cc`), `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P2 **Now**).
**After.** None.
**State.** open

## B9 Capture run_gate with one short trial and print_check --study

**Principles.** P2
**Decisions.** ADR 0021
**Where.** `tools/architecture/programs.txt`.
**Evidence.** `measure.sh --list P2` lists `src/run_gate.cc`, and `--list P2.unrun`
lists `//:run_gate`. `behavior.md`, section 6a: `src/check_placement.cc` and `.h` are
reached only by `print_check --study` (`src/print_check.cc:198`), which no capture
passes. Both files are frozen; this item adds no line to them.
**Change.** Add two lines to `programs.txt`, each with the comment above it that names
the source lines of the times it cuts:
`run_gate.one_trial | //:run_gate | --trials 1 --modules 2 --threads 1 | <cuts>` and
`print_check.study | //:print_check | --study | <cuts>`. The cuts are the wall-clock
fields only: for `run_gate`, "in N s wall" of `src/trial_runner.cc:745`, "in N s of
trial wall time" of `:767` and "wall time N s" of `src/run_gate.cc:147`; for `--study`,
every field computed from the render time of `src/check_placement.cc:634` to `:638`. Run
each twice and find every other field that differs; if one is not a time, stop and
report. Add both ids to the table of section 3 of `behavior.md` and drop `run_gate` and
`print_check --study` from section 4b.
**Done when.** `measure.sh` prints `P2 3`, `P2.unrun 10`, `P2.frozen 1`; two captures
after the change compare `same`; against the capture before only the two new files
differ.
**Net.** This item is the net for `src/run_gate.cc` and `src/check_placement.*`.
**Size.** 2 files, about 12 lines.
**Touches.** `tools/architecture/programs.txt`, `docs/architecture/behavior.md`
(sections 3, 4b, 6a), `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P2 **Now**).
**After.** None.
**State.** open

## B10 Verify the committed plan of flat_rack_2_pin_arms in plan_replay_test

**Principles.** P8
**Decisions.** ADR 0010
**Where.** `tests/plan_replay_test.cc`, `plans/flat_rack_2_pin_arms.json`.
**Evidence.** `measure.sh --list P8` prints `plans/flat_rack_2_pin_arms.json`;
`deps.json`, `generated`: "nothing: plan_replay_test reads only the chosen layout's
plan" (`TheCommitted()`, `tests/plan_replay_test.cc:411`, takes the layout with
`"chosen": true` only; since run 018 the layout `versions/versions.json` names as chosen).
**Change.** Add one case, `TEST(OtherPlans, FlatRack2PinArmsPassesTheVerifier)`, at
the end of the section "the committed plan": it loads
`layouts/flat_rack_2_pin_arms.json` with `LoadLayout`, reads
`plans/flat_rack_2_pin_arms.json` with `ReadFile` and `PlanFromJson`, takes
`Sha256OfFile` of the layout file, and asserts what `Committed.PassesTheVerifier`
asserts except the count of joint margins: `r.ok`, no fault, both least distances at or
above the plan's clearance less the tolerance. In `deps.json`, entry
`plans/flat_rack_2_pin_arms.json`, set `test` to `tests/plan_replay_test.cc` and
`compares` to what it compares. If the verifier finds a fault in the committed plan,
stop: that is a finding for the owner, not a test to loosen. Then change one waypoint of
the plan in memory and see the case fail.
**Done when.** `plan_replay_test` passes; `measure.sh` prints `P8 1`; two captures after
the change compare `same` (the test target is the same, its result passes).
**Net.** This item is the net for the plan.
**Size.** 2 files, about 30 lines.
**Touches.** `tests/plan_replay_test.cc`, `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P8 **Now**).
**After.** None.
**State.** open

## B11 Fail ci.sh when MODULE.bazel.lock is not current

**Principles.** P8
**Decisions.** ADR 0010, ADR 0003
**Where.** `tools/architecture/ci.sh`.
**Evidence.** `measure.sh --list P8` prints `MODULE.bazel.lock`; `deps.json`,
`generated`: "no build runs with --lockfile_mode=error". `docs/ompl-integration.md:1572`
records that `bazel build --nobuild --lockfile_mode=error --config=opt //...` (there
with a smoke target added) exits 0
when the lock file is current (`bazel mod deps` cannot be used: it fails on this
repository). The lock file holds no entry for one platform (`grep -c '"os:'
MODULE.bazel.lock` prints 0), so the check passes on x86-64 as on arm64.
**Change.** In `tools/architecture/ci.sh` add a step before the others, "== step 0: the
lock file", that runs `${BAZEL:-bazel} build --nobuild --lockfile_mode=error
--config=opt //...` and, on a non-zero exit, adds `lock file` to `failed`; describe it
in the opening comment. In `deps.json`, entry `MODULE.bazel.lock`, set `test` to
`tools/architecture/ci.sh` and `compares` to "bazel refuses to build when the lock file
does not match MODULE.bazel (--lockfile_mode=error), in CI's architecture job". Add a
row to the guardrails table of `docs/architecture/README.md`. Check it: add a
`bazel_dep` to a scratch copy, see the step fail, put it back.
**Done when.** `tools/architecture/ci.sh` prints `ci.sh: ok`, its step 0 included;
`measure.sh` prints `P8 0`.
No code changes, so no capture.
**Net.** None needed: no code.
**Size.** 3 files, about 20 lines.
**Touches.** `tools/architecture/ci.sh`, `tools/architecture/deps.json`,
`docs/architecture/README.md`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P8 **Now**).
**After.** B10
**State.** open

## B12 Give every initializer its members: -Wmissing-field-initializers to 0, an error

**Principles.** P9
**Decisions.** ADR 0014
**Where.** `src/layout_check.cc:1695`; `tests/layout_model_test.cc:495` to `:509`.
**Evidence.** `W -Wmissing-field-initializers 11`
(`warnings.sh --list -Wmissing-field-initializers` prints them): the
`PostureChanges` of `src/layout_check.cc:1695` (`{Use(i).name}`) and ten `Thing`
initializers in `Things()` of the test. No place is in a frozen file.
**Change.** Write every member: `postures_.push_back({Use(i).name, 0.0, "", "", 0});`
(`PostureChanges`, `src/layout_check.h:395`), and each `Thing` as
`{kind, arm, index, name}` with the defaults it has today (`""` and `-1`). Move
`-Wmissing-field-initializers` from the `build:strict` line of `.bazelrc` to the default
line as `-Werror=missing-field-initializers`, and delete its `W` line.
**Done when.** `warnings.sh` prints `W -Wmissing-field-initializers 0` before the move,
and the same commit moves the flag to the default set as an error; `bazel build
--config=opt //...` passes; `measure.sh` prints `P9.held 62` (one higher: a new
`-Werror=` flag); `behavior.sh compare` (with the tests) prints `same`.
**Net.** `check_layout.*`, `plan_layout.*`; `layout_check_test`, `layout_model_test`.
**Size.** 3 files, about 12 lines.
**Touches.** `src/layout_check.cc`, `tests/layout_model_test.cc`, `.bazelrc`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** None.
**State.** open

## B13 Check the nullptr of FindArmModel in the test: -Wnull-dereference to 0

**Principles.** P9
**Decisions.** ADR 0014
**Where.** `tests/layout_model_test.cc` (300, 446, 557, 570); `tests/ik_test.cc:412`.
**Evidence.** `W -Wnull-dereference 2`: `src/arm_catalog.h:116` inlined into
`TouchesByDesign` at `tests/layout_model_test.cc:570` ("potential null pointer
dereference"), and `tests/ik_test.cc:414` in `FlippedWrist`.
**Change.** In `tests/layout_model_test.cc` test each `FindArmModel` result before use:
in a `TEST` body `ASSERT_NE(model, nullptr) << arm.model;`, in `TouchesByDesign` (a
function that returns text) `if (model == nullptr) { ADD_FAILURE() << arm->model;
return ""; }`, and at 557 bind the result to a name first. In `FlippedWrist` of
`tests/ik_test.cc` start with `if (q.size() != Arm::kJoints) return q;`. If GCC still
warns at `ik_test.cc:414`, stop and report the message: the place is then for the lead.
**Done when.** `warnings.sh` prints `W -Wnull-dereference 0`; the flag stays under
`--config=strict` as a warning with its `W` line at 0, because the optimizer decides it
(ADR 0014); `layout_model_test` and `ik_test` pass; `behavior.sh compare` (with the
tests) prints `same`.
**Net.** The two tests themselves.
**Size.** 2 files, about 12 lines.
**Touches.** `tests/layout_model_test.cc`, `tests/ik_test.cc`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B12
**State.** open

## B14 Rename the six shadowing names outside frozen files: -Wshadow to 2

**Principles.** P9
**Decisions.** ADR 0014
**Where.** `src/path_timing.cc:732`, `tests/arm_follower_test.cc:509`,
`tests/joint_move_test.cc:228`, `tests/path_follow_test.cc:384`,
`tests/path_timing_test.cc:564`, `tests/plan_replay_test.cc:413`.
**Evidence.** `W -Wshadow 8`; the other two places, `src/view_cell.cc:352` and
`tests/motion_planner_test.cc:927`, are frozen.
**Change.** Rename the inner name and its uses inside its scope only: the lambda
parameter `limits` of `change` in `src/path_timing.cc:732` to `step_limits`; the name
generators' parameter `info` of the three `INSTANTIATE_TEST_SUITE_P` to `param`; the
lambda parameter `limits` of `plan` at `tests/path_timing_test.cc:564` to `with`; the
inner `auto* c` of `TheCommitted()` at `tests/plan_replay_test.cc:413` to `made`.
**Done when.** `warnings.sh` prints `W -Wshadow 2`; the flag stays under
`--config=strict` until the stand-in cell's files go; `behavior.sh compare` (with the
tests) prints `same`.
**Net.** `path_timing_test`, `arm_follower_test`, `path_follow_test`, the cycle tests,
captures `print_plan.*`, `run_cell.*`, `plan_layout.*`; the tests themselves.
**Size.** 6 files, about 20 lines.
**Touches.** `src/path_timing.cc`, `tests/arm_follower_test.cc`,
`tests/joint_move_test.cc`, `tests/path_follow_test.cc`, `tests/path_timing_test.cc`,
`tests/plan_replay_test.cc`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P9 **Now**).
**After.** B7, B10
**State.** open

## B15 Brace the three ifs around EXPECT_EQ outside frozen files: -Wparentheses to 1

**Principles.** P9
**Decisions.** ADR 0014
**Where.** `tests/ik_test.cc:971`, `tests/layout_check_test.cc:242`, `:879`.
**Evidence.** `W -Wparentheses 4` ("suggest explicit braces to avoid ambiguous
'else'"); the fourth, `tests/planned_carry_test.cc:997`, is frozen.
**Change.** Put braces round the body of each of the three `if`, as
`if (!is_arm[i]) { EXPECT_EQ(...) << ...; }`.
**Done when.** `warnings.sh` prints `W -Wparentheses 1`; the flag stays under
`--config=strict` until the stand-in cell's files go; `behavior.sh compare` (with the
tests) prints `same`.
**Net.** `ik_test`, `layout_check_test`.
**Size.** 2 files, 6 lines.
**Touches.** `tests/ik_test.cc`, `tests/layout_check_test.cc`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B13
**State.** open

## B16 End no comment line of view_layout.cc with a backslash: -Wcomment to 2

**Principles.** P9
**Decisions.** ADR 0014
**Where.** `src/view_layout.cc:9`.
**Evidence.** `W -Wcomment 3` ("multi-line comment"); `src/view_layout.cc:9` ends a
`//` line of the usage example with ` \`. The other two, `tests/gate_test.cc:18` and
`:25`, are frozen.
**Change.** Delete the trailing ` \` of line 9 and of any following comment line of the
same example, and add the words "(one command)" after it, so that the example still
reads as one command. A comment alone changes.
**Done when.** `warnings.sh` prints `W -Wcomment 2`; the flag stays under
`--config=strict` until the stand-in cell's files go; `bazel build --config=opt
//:viewer_layout` passes. Outside the net, but only a comment changes: no capture.
**Net.** None needed: a comment.
**Size.** 1 file, 3 lines.
**Touches.** `src/view_layout.cc`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P9 **Now**).
**After.** None.
**State.** open

## B17 Make the conversions of the arm, base and hardware explicit: -Wconversion to 53

**Principles.** P9
**Decisions.** ADR 0014
**Where.** `src/collision.cc` (205, 206, 281, 307), `src/ik.cc:78`,
`src/motion_planner.cc:447`, `tests/bay_model_test.cc:76`, `tests/profile_test.cc:330`.
**Evidence.** `W -Wconversion 61`, of which 43 in frozen files. These eight are
`mjtSize` or `long` counts into `int` (`3 * m.nmocap`, `model_->nbody`, `model.nv`,
`m.nbody`), a `size_t` into the `int` of `PlanSeed(..., 1000 + leg)` and a 32-bit
generator output (held in `unsigned long`) into `double`. Each value is far below
2^31, so no figure changes.
**Change.** Write each conversion as a `static_cast` to the type it goes into today
(`static_cast<int>(...)`, `static_cast<double>(...)`), at the place of the warning and
nothing else.
**Done when.** `warnings.sh` prints `W -Wconversion 53`; the flag stays under
`--config=strict`; `behavior.sh compare` (with the tests) prints `same`.
**Net.** `collision_test`, `ik_test`, `motion_planner_test`, the cycle tests,
`bay_model_test`, `profile_test`; captures `print_plan.*`, `run_cell.*`,
`print_check.*`.
**Size.** 5 files, 8 lines.
**Touches.** `src/collision.cc`, `src/ik.cc`, `src/motion_planner.cc`,
`tests/bay_model_test.cc`, `tests/profile_test.cc`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P9 **Now**).
**After.** B5
**State.** open

## B18 Make the conversions of planning and the layout explicit: -Wconversion to 44

**Principles.** P9
**Decisions.** ADR 0014
**Where.** `src/ompl_search.cc:447`, `src/print_rrt.cc` (60, 235, 298),
`src/layout_replay.cc:1659`, `tests/path_timing_test.cc` (676, 697).
**Evidence.** `W -Wconversion` nine places here: a `size_t` index into OMPL's
`unsigned int` (`getState(i)`), `size_t` and `long long` counts into `double`, the
`long` counter `k` of `k * sample`, and generator outputs into `double` and
`unsigned`. All exact.
**Change.** As in B17: a `static_cast` to today's target type at each place.
**Done when.** `warnings.sh` prints `W -Wconversion 44`; the flag stays under
`--config=strict`; `behavior.sh compare` (with the tests) prints `same`.
**Net.** `ompl_search_test`, `motion_planner_test`, `path_timing_test`,
`plan_replay_test`, `layout_plan_test`; captures `print_rrt.*`, `plan_layout.*`.
**Size.** 4 files, 9 lines.
**Touches.** `src/ompl_search.cc`, `src/print_rrt.cc`, `src/layout_replay.cc`,
`tests/path_timing_test.cc`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P9 **Now**).
**After.** B2, B7, B17
**State.** open

## B19 Remove the three cloned branches: bugprone-branch-clone to 0, an error

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `src/layout_plan.cc:972`, `src/path_timing.cc:438`, `src/plan_layout.cc:303`.
**Evidence.** `T bugprone-branch-clone 3`. At `src/layout_plan.cc:972`
`e.config = s0.kind == "move" ? s0.path.front() : s0.path.front();` has two equal arms
and no side effect, so both give the start of the first segment, which the comment
above it asks for. `src/path_timing.cc:438` to `:444` gives `kInfinity` in its first and
third branch; `src/plan_layout.cc:303` to `:308` returns `Usage()` in two.
**Change.** Write `e.config = s0.path.front();`. In `src/path_timing.cc` join the first
and third branch as `if (line.straight_on[c] || (!stop[c] && sine[c] < kStraightOn))`
and keep the `stop` branch and the last one as they are. In `src/plan_layout.cc` write
`} else if (name.empty() && (a.empty() || a[0] != '-')) { name = a; } else { return
Usage(); }` in place of the last three branches. Delete `-bugprone-branch-clone,` under
`WarningsAsErrors` in `.clang-tidy` and its `T` line.
**Done when.** `tidy.sh` prints no place of `bugprone-branch-clone` and exits 0, and the
same commit moves the check to the errors; `P9.held` is unchanged (the
exclusion it deletes is not counted); `behavior.sh compare` (with the tests) prints
`same`.
**Net.** `layout_plan_test`, `plan_replay_test`, `path_timing_test` and the cycle tests;
captures `plan_layout.*`, `print_plan.*`, `run_cell.*`.
**Size.** 4 files, about 10 lines.
**Touches.** `src/layout_plan.cc`, `src/path_timing.cc`, `src/plan_layout.cc`,
`.clang-tidy`, `docs/architecture/baseline.txt`, `docs/architecture/principles.md`
(P9 **Now**).
**After.** B2, B14
**State.** open

## B20 Parenthesize NEED_COMMITTED's argument: bugprone-macro-parentheses to 0

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `tests/plan_replay_test.cc:470` to `:472`.
**Evidence.** `T bugprone-macro-parentheses 2`, both at line 472 ("macro argument
should be enclosed in parentheses").
**Change.** On line 472 write the two uses of `c` as `(c)`: `if (!(c).error.empty())
FAIL() << (c).error;`; line 471, where `c` is declared, stays. Delete
`-bugprone-macro-parentheses,` under `WarningsAsErrors` and its `T` line.
**Done when.** `tidy.sh` prints no place of the check and exits 0, and the same commit
moves the check to the errors; `P9.held` unchanged; `plan_replay_test` passes;
`behavior.sh compare` (with the tests) prints `same`.
**Net.** `plan_replay_test`.
**Size.** 2 files, 3 lines.
**Touches.** `tests/plan_replay_test.cc`, `.clang-tidy`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B10, B14
**State.** open

## B21 Check getenv before making a string of it: StringChecker to 1

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `tests/arm_follower_test.cc:718`, `tests/layout_model_test.cc:1135`.
**Evidence.** `T clang-analyzer-cplusplus.StringChecker 3` ("The parameter must not be
null"): `std::string` built from `std::getenv(...)`, undefined behavior when the
variable is unset (Bazel sets it; a binary run by hand does not). The third,
`tests/motion_planner_test.cc:852`, is frozen.
**Change.** At `arm_follower_test.cc:718` write `const char* env =
std::getenv("MODEL_PATH"); ASSERT_NE(env, nullptr) << "MODEL_PATH is not set";
const std::string path = env;`. At `layout_model_test.cc:1135` use the file's
`LayoutPath("flat_rack_3_pin_arms")` (line 52).
**Done when.** `tidy.sh` prints `T clang-analyzer-cplusplus.StringChecker 1` and exits
0; the check stays counted until the stand-in cell's files go; both tests pass;
`behavior.sh compare` (with the tests) prints `same`.
**Net.** `arm_follower_test`, `layout_model_test`.
**Size.** 2 files, 4 lines.
**Touches.** `tests/arm_follower_test.cc`, `tests/layout_model_test.cc`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B13, B14
**State.** open

## B22 Index the two ends of the container and the flat rack: CallAndMessage to 0

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `src/container.h:160` to `:170`, `src/flat_rack.h:130` to `:136`.
**Evidence.** `T clang-analyzer-core.CallAndMessage 2` ("2nd function call argument is
an uninitialized value"): `ends[end]` with a counter `end` beside a loop over
`{-1.0, 1.0}`; the analyzer loses that `end` is 0 or 1. A false positive, but the loop
reads better by index.
**Change.** In both, write `for (int end = 0; end < 2; ++end) { const double sy = end
== 0 ? -1.0 : 1.0; ... }` with the same body, names and order of `push_back`, and
delete the separate counter. Delete `-clang-analyzer-core.CallAndMessage,` under
`WarningsAsErrors` and its `T` line. No figure of the hardware changes.
**Done when.** `tidy.sh` prints no place of the check and exits 0, and the same commit
moves the check to the errors; `P9.held` unchanged; `behavior.sh compare` (with the
tests) prints `same`.
**Net.** `container_test`, `flat_rack_test`, `bay_model_test`, the layout tests;
captures `check_layout.*`, `plan_layout.*`.
**Size.** 3 files, about 10 lines.
**Touches.** `src/container.h`, `src/flat_rack.h`, `.clang-tidy`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** None.
**State.** open

## B23 Initialize the twelve locals: cppcoreguidelines-init-variables to 0, an error

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `src/layout_check.cc` (359, 705, 865, 2106, 2158), `src/portable_random.h`
(80, 85).
**Evidence.** `T cppcoreguidelines-init-variables 12`. Each is set before it is read:
`perr` and its kin by `eval` on line 360, `margin` by `Checker::Margin` (line 475 writes
`*margin = 1e9` first), `standard`, `x`, `y`, `r2` in `NormalDraw`.
**Change.** Give each its value at the declaration: `0.0` for every `double` (`double
perr = 0.0, aerr = 0.0, perrn = 0.0, aerrn = 0.0;`, `double margin = 0.0;`,
`double standard = 0.0;`, `double x = 0.0, y = 0.0, r2 = 0.0;`). Delete
`-cppcoreguidelines-init-variables,` under `WarningsAsErrors` and its `T` line.
**Done when.** `tidy.sh` prints no place of the check and exits 0, and the same commit
moves the check to the errors; `P9.held` unchanged; `behavior.sh compare` (with the
tests) prints `same`.
**Net.** `layout_check_test`, `layout_plan_test`, `plan_replay_test`,
`rectangle_fit_test`, the cycle tests; captures `check_layout.*`, `plan_layout.*`,
`print_perception.*`, `run_cell.*`.
**Size.** 3 files, 7 lines.
**Touches.** `src/layout_check.cc`, `src/portable_random.h`, `.clang-tidy`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B12
**State.** open

## B24 Value-initialize the members and arrays outside the viewer: member-init to 14

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `src/ik.cc:58`, `src/layout_check.cc` (113, 1642), `src/layout_plan.cc:853`,
`tests/arm_test.cc:274`, `tests/ik_test.cc:107`, `tests/locate_physics_test.cc` (292,
362, 388).
**Evidence.** `T cppcoreguidelines-pro-type-member-init 23`, of which 12 in frozen files
and 2 in `src/viewer_lib.cc` (B27). Each of these nine is written before it is read
(the loops, `StateAt`, the six columns of the Jacobian).
**Change.** Add `{}` to each declaration (`std::array<double, 6> deg{};`, `Other x{};`,
`Glass g{};`, and so on) and `Matrix6d jacobian = Matrix6d::Zero();` in the struct of
`src/ik.cc:58`. Nothing else.
**Done when.** `tidy.sh` prints `T cppcoreguidelines-pro-type-member-init 14` and exits
0; the check stays counted; `behavior.sh compare` (with the tests) prints `same`.
**Net.** `ik_test`, `arm_test`, `locate_physics_test`, the layout tests, the cycle
tests; captures `check_layout.*`, `plan_layout.*`, `print_plan.*`, `run_cell.*`.
**Size.** 6 files, 9 lines.
**Touches.** `src/ik.cc`, `src/layout_check.cc`, `src/layout_plan.cc`,
`tests/arm_test.cc`, `tests/ik_test.cc`, `tests/locate_physics_test.cc`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B15, B17, B19, B23
**State.** open

## B25 Declare the special members of five classes: special-member-functions to 0

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `src/layout_model.cc:684` (`VfsGuard`), `src/layout_plan.cc:579` (`Report`),
`src/layout_scene.h:242` (`LayoutScene`), `src/motion_planner.cc:60` (`MarginGuard`),
`src/ompl_search.h:228` (`OmplSpace`).
**Evidence.** `T cppcoreguidelines-special-member-functions 5`. `VfsGuard` calls
`mj_deleteVFS` in its destructor and can be copied: a copy would delete the VFS twice.
Nothing copies either guard today. The other three already delete copying.
**Change.** For `VfsGuard` and `Report` add a constructor that takes the members
(`explicit VfsGuard(mjVFS* v) : vfs(v) {}`; `Report(const MoveJob& j,
std::chrono::steady_clock::time_point b) : job(j), begun(b) {}`), keep the brace
initializations that call them, and declare copy and move constructor and assignment
`= delete`. For `LayoutScene`, `MarginGuard` and `OmplSpace` declare the move
constructor and move assignment `= delete` (they are not movable today either), and
for `LayoutScene` `~LayoutScene() = default;`. Delete
`-cppcoreguidelines-special-member-functions,` under `WarningsAsErrors` and its `T`
line.
**Done when.** `tidy.sh` prints no place of the check and exits 0, and the same commit
moves the check to the errors; `P9.held` unchanged; `behavior.sh compare` (with the
tests) prints `same`.
**Net.** The layout tests, `motion_planner_test`, `ompl_search_test`, the cycle tests;
captures `check_layout.*`, `plan_layout.*`, `print_plan.*`, `run_cell.*`.
**Size.** 6 files, about 25 lines.
**Touches.** `src/layout_model.cc`, `src/layout_plan.cc`, `src/layout_scene.h`,
`src/motion_planner.cc`, `src/ompl_search.h`, `.clang-tidy`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B17, B24
**State.** open

## B26 Count the three test loops with an integer: cert-flp30-c to 8

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `tests/layout_plan_test.cc:86`, `:87`, `tests/ik_test.cc:273`.
**Evidence.** `T cert-flp30-c 11` ("loop induction expression should not have
floating-point type"): 7 in frozen files, 1 at `src/layout_plan.cc:825` whose cure would
move the instants a plan is checked at ("Needs the owner"), and these three.
`layout_plan_test.cc:86` and `:87` step by 10.0 from whole numbers, which is exact.
**Change.** `for (int k = 0; k <= 14 && !low; ++k) { const double j2 = -140.0 + 10.0 *
k;` and `for (int k = 0; k <= 25 && !low; ++k) { const double j3 = 150.0 - 10.0 * k;`
in `layout_plan_test.cc`; `for (int k = 0; k <= 6000; ++k) { const double q3 = -0.3 +
1e-4 * k;` in `ik_test.cc`. The first two give the same values as today; the third
tries `0.3` itself and its values differ in the last bits; the test's bounds do not
change.
**Done when.** `tidy.sh` prints `T cert-flp30-c 8` and exits 0; the check stays counted;
`layout_plan_test` and `ik_test` pass; `behavior.sh compare` (with the tests) prints
`same`. If `ik_test` fails, stop and report.
**Net.** The two tests.
**Size.** 2 files, 8 lines.
**Touches.** `tests/layout_plan_test.cc`, `tests/ik_test.cc`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P9 **Now**).
**After.** B3, B24
**State.** open

## B27 Clear the viewer library's findings, now that a test holds it

**Principles.** P9
**Decisions.** ADR 0014, ADR 0015
**Where.** `src/viewer_lib.cc` (40, 127, 241).
**Evidence.** `W -Wconversion` at `src/viewer_lib.cc:127` (`const int ncam =
viewer->model->ncam;`, `mjtSize` to `int`), and
`T cppcoreguidelines-pro-type-member-init` at `:40` (`mjvCamera camera;` and three
more, set by `mjv_default*` at 398 to 413) and `:241` (`std::array<double, 3> at;`,
filled by `sscanf` before use).
**Change.** `const int ncam = static_cast<int>(viewer->model->ncam);`;
`mjvCamera camera{};`, `mjvOption options{};`, `mjvScene scene{};`,
`mjrContext context{};`; `std::array<double, 3> at{};`.
**Done when.** `warnings.sh` prints `W -Wconversion 43` (all in frozen files) and
`tidy.sh` `T cppcoreguidelines-pro-type-member-init 12` (all in frozen files); both
stay counted; `viewer_lib_test` passes; `behavior.sh compare` (with the tests) prints
`same`. The window is held by nothing: run `//:viewer` once by hand if a display is
there, and say so.
**Net.** `viewer_lib_test` (B8).
**Size.** 1 file, 6 lines.
**Touches.** `src/viewer_lib.cc`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P9 **Now**).
**After.** B2, B8, B18, B24
**State.** open

## B28 Switch bugprone-unchecked-optional-access on for src/, off for tests/

**Principles.** P9
**Decisions.** ADR 0015
**Where.** `.clang-tidy`, `tests/.clang-tidy` (new), `src/joint_move.cc:55`,
`src/layout_plan.cc:687`, `:690`, `src/print_profile.cc:80`, `:81`.
**Evidence.** ADR 0015, "Alternatives": 879 places, 810 under `tests/` where
GoogleTest's `ASSERT_TRUE(x)` before `*x` is not understood, 69 under `src/`; switching
it off for `tests/` alone "needs a `.clang-tidy` under `tests/`". Of the 69, 64 are in
frozen files and five are here, each an
optional known to hold a value that the check cannot follow.
**Change.** Write `tests/.clang-tidy` (new) with `InheritParentConfig: true` and
`Checks: '-bugprone-unchecked-optional-access'` and a comment that gives the reason.
In `.clang-tidy` delete `-bugprone-unchecked-optional-access,` from `Checks`, so that
the check runs, and add `-bugprone-unchecked-optional-access,` under
`WarningsAsErrors`, so that it is counted and not an error. Cure the five: at
`joint_move.cc:55` `if (!still) return
std::nullopt;` before `*still`; at `layout_plan.cc:687` bind `const JointPath& chosen =
shortened ? *shortened : *path;` and use `chosen` at 690; at `print_profile.cc:80`
check `trapezoid` and `s_curve` as the function's other failures do. Add the line
`T bugprone-unchecked-optional-access <n>` that `tidy.sh` prints, and change ADR 0015's
sentence by a superseding record only if the lead asks; this item edits no record.
**Done when.** `tidy.sh` prints `T bugprone-unchecked-optional-access` with only places
in frozen files (64 expected) and exits 0; `P9.held` unchanged (61: only
exclusions change, which it does not count); `behavior.sh compare`
(with the tests) prints `same`.
**Net.** `joint_move_test` and every cycle test, `layout_plan_test`,
`plan_replay_test`; captures `print_profile.*`, `plan_layout.*`.
**Size.** 5 files, about 20 lines.
**Touches.** `.clang-tidy`, `tests/.clang-tidy` (new), `src/joint_move.cc`,
`src/layout_plan.cc`, `src/print_profile.cc`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P9 **Now**).
**After.** B19, B25
**State.** open

## B29 Give the geom distance one home in base: src/geom_distance.h

**Principles.** P7, P1
**Decisions.** ADR 0017, ADR 0009
**Where.** `src/geom_distance.h` (new), `src/collision.cc:526`,
`src/layout_scene.cc:575`, `:586`.
**Evidence.** `measure.sh --list P7` prints `src/collision.cc (geom_distance)`; the
home in `deps.json` is `src/layout_scene.cc` (group `layout`), which `arm` may not
include, so `src/collision.cc` cannot take it from there. The fact to keep in one place
is in `src/layout_model.cc:717` to `:725`: MuJoCo 4.0.0's `mj_geomDistance` measures a
capsule whose axis passes through a box as clear.
**Change.** Write `src/geom_distance.h` (new), group `base`, namespace `cell`, with an
opening comment that states that fact and `inline double GeomDistance(const mjModel& m,
mjData* d, int g, int h, double far) { return mj_geomDistance(&m, d, g, h, far,
nullptr); }`. Add `cc_library(name = "geom_distance", hdrs = ["src/geom_distance.h"],
deps = ["@mujoco"])` beside `mj_handles` near the top of `BUILD.bazel`. Call it at the
three places, include it, and add `":geom_distance",` to the `deps` of `collision`
(section `collision`) and `layout_check` (section `layout_check`). In `deps.json` add
the file to group `base` and set the `home` of `geom_distance` to
`["src/geom_distance.h"]`.
**Done when.** `measure.sh` prints `P7.geom_distance 2` (both frozen), `P7 16`,
`P4.mujoco 69` and `P4.mujoco_files 71` (one library and one header for MuJoCo; the
commit message says why); `behavior.sh compare` (with the tests) prints `same`.
**Net.** `collision_test`, `motion_planner_test`, the cycle tests, the layout tests;
captures `print_plan.*`, `run_cell.*`, `check_layout.*`, `plan_layout.*`.
**Size.** 5 files, about 35 lines.
**Touches.** `src/geom_distance.h` (new), `src/collision.cc`, `src/layout_scene.cc`,
`BUILD.bazel` (top, sections `collision`, `layout_check`),
`tools/architecture/deps.json`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P4 and P7 **Now**).
**After.** B5, B17
**State.** open

## B30 Take the container's base figures in src/flat_rack.h from src/container.h

**Principles.** P7
**Decisions.** ADR 0009
**Where.** `src/flat_rack.h:62`, `:63`, `:74`, `:78`; `//:flat_rack`.
**Evidence.** `measure.sh --list P7` prints `src/flat_rack.h (container_base)`:
`kFittingSpacingLength 11.985`, `kFittingSpacingWidth 2.259`,
`kTransportGrossMass 23800.0` ("As src/container.h") and `kChassisDeckHeight 1.219`
are written again, with the same values as `src/container.h:52`, `:53`, `:94`, `:98`.
Both files are in group `hardware`.
**Change.** In `src/flat_rack.h` include `src/container.h` and replace the four
definitions by `using container::kFittingSpacingLength;` and the same for the other
three, inside `namespace cell::flat_rack`, so that every user's name stays. Keep
`kTareMass` and `kCellMassBudget`, which differ. Add `":container",` to the `deps` of
`flat_rack` (section `flat_rack`). No value changes.
**Done when.** `measure.sh` prints `P7.container_base 0`, `P7 15`; `deps.py` prints ok;
`behavior.sh compare` (with the tests) prints `same`.
**Net.** `flat_rack_test`, `container_test`, the layout tests; captures
`check_layout.*`, `plan_layout.*`.
**Size.** 2 files, about 8 lines.
**Touches.** `src/flat_rack.h`, `BUILD.bazel` (section `flat_rack`),
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P7 **Now**).
**After.** B22
**State.** open

## B31 Write pi as std::numbers::pi in the five files that stay

**Principles.** P7
**Decisions.** ADR 0009
**Where.** `src/bay_hardware.h` (233, 503, 712), `src/layout_model.cc:75`,
`src/layout_plan.cc:32`, `src/layout_replay.cc:28`, `src/print_bay.cc` (77, 136, 203,
212).
**Evidence.** `measure.sh --list P7` prints these five under `(pi)`; the other five
are frozen. Every literal is `3.14159265358979323846`, 21 digits, which rounds to the
same `double` as `std::numbers::pi`, so no figure of the hardware moves.
**Change.** Include `<numbers>` and write `std::numbers::pi` for each literal
(`constexpr double kPi = std::numbers::pi;` where a file names it). Change no other
digit. A literal with fewer digits, if found, is not this item's: report it.
**Done when.** `measure.sh` prints `P7.pi 5` (all frozen), `P7 10`; `behavior.sh
compare` (with the tests) prints `same`, `gen_bay.default` and `print_bay.*` included.
**Net.** `bay_hardware_test`, `bay_model_test`, `locate_physics_test`, the layout tests;
captures `print_bay.*`, `gen_bay.default`, `check_layout.*`, `plan_layout.*`.
**Size.** 5 files, about 15 lines.
**Touches.** `src/bay_hardware.h`, `src/layout_model.cc`, `src/layout_plan.cc`,
`src/layout_replay.cc`, `src/print_bay.cc`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P7 **Now**).
**After.** B2, B18, B24, B25, B28
**State.** open

## B32 Remove boost.config, which OMPL reaches through the other Boost modules

**Principles.** P4
**Decisions.** ADR 0016, ADR 0008
**Where.** `MODULE.bazel:124`, `bazel/ompl.BUILD.bazel:110`, `MODULE.bazel.lock`.
**Evidence.** [`dependencies.md`](dependencies.md), "Not needed, by the query": of the
17 Boost modules only `boost.config` is never included by name by OMPL's sources and is
reached without its own edge, every other Boost module asking for it at the same
version; the query is evidence, not proof.
**Change.** Follow "Updating, replacing or removing an outside dependency" of
`dependencies.md`: delete the `bazel_dep` of `boost.config` from `MODULE.bazel` and
`"@boost.config",` from `bazel/ompl.BUILD.bazel`, let Bazel rewrite
`MODULE.bazel.lock` on the next build, and change the rows of `dependencies.md` that
count the modules. If the build fails, or `bazel mod`'s resolved version of
`boost.config` differs from `1.90.0.bcr.1`, put everything back and report.
**Done when.** `measure.sh` prints `P4 25`; `bazel build --config=opt //...` passes;
`behavior.sh compare` (with the tests) prints `same`.
**Net.** `ompl_search_test`, `motion_planner_test`; captures `print_plan.*`,
`run_cell.*`.
**Size.** 4 files, about 5 lines and the lock file.
**Touches.** `MODULE.bazel`, `MODULE.bazel.lock`, `bazel/ompl.BUILD.bazel`,
`docs/architecture/dependencies.md`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P4 **Now**).
**After.** B11
**State.** open

## B33 Move the planner's class of src/layout_plan.cc into a private header

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_plan.cc` (30 to 153, 182 to 272),
`src/layout_plan_internal.h` (new).
**Evidence.** `measure.sh --list P11` prints `src/layout_plan.cc 1462`. Its three
stages (the comment at lines 1 to 9: assemble the blocks, plan the moves, place them
on one clock) are methods of one class `Planner` in an anonymous namespace, so no stage
can leave the file before the class is declared where two files can see it.
**Change.** Write `src/layout_plan_internal.h` (new), group `layout`, namespace
`cell::layout`, with an opening comment of at least five lines that says it holds what
the files of `//:layout_plan` share and that no other target includes it. Move into it,
unchanged, the helpers of lines 30 to 152 (`Fmt` with its 2048-byte buffer, `ToConfig`,
`ToPose`, `SameConfig`, `ConfigText`, `Union`, `AddLeadersParts`, `IsHoleKind`, as
`inline`), the structs `MoveJob`, `Block`, `Need`, `ArmEnd`, the declaration of class
`Planner` (184 to 264) and `FindPoint` (265, `inline`). Include it in
`src/layout_plan.cc`; list it in the `srcs` of `layout_plan` (section `layout_plan`),
and in group `layout` of `deps.json`. It includes no `<mujoco/...>` header directly.
**Done when.** `measure.sh` prints `P11 7` still, `P11.comment 0`, `P4.mujoco_files 70`
(or what B29 left); `wc -l src/layout_plan.cc` prints about 1,250; `deps.py` prints ok;
`behavior.sh compare` (with the tests) prints `same`.
**Net.** `layout_plan_test`, `plan_replay_test`; capture `plan_layout.*`.
**Size.** 4 files, about 210 lines moved.
**Touches.** `src/layout_plan.cc`, `src/layout_plan_internal.h` (new), `BUILD.bazel`
(section `layout_plan`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`.
**After.** B2, B19, B24, B25, B28, B31
**State.** open

## B34 Move the assembling of the arms' blocks into src/layout_plan_assemble.cc

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_plan.cc` (`Planner::Assemble`, lines 273 to 576 at the base
commit), `src/layout_plan_assemble.cc` (new).
**Evidence.** `measure.sh --list P11` prints `src/layout_plan.cc` over 1,000 lines
after B33. `Planner::Assemble` builds every arm's row of blocks from the checker's
steps and poses, the first stage of the file's comment, and reads no clock.
**Change.** Move `Planner::Assemble` unchanged into `src/layout_plan_assemble.cc` (new),
which opens with a comment of what the stage does (from the checker's steps to
each arm's blocks, with their durations and paths) and includes
`src/layout_plan_internal.h` (new) and the standard headers it uses. The clock readings
(`PlanOne`, `Run`) stay in `src/layout_plan.cc`, which `deps.json`
`clock_printed_only` names. Add the file to the `srcs` of `layout_plan` and to group
`layout` of `deps.json`.
**Done when.** `measure.sh` prints `P11.cc 4`, `P11 6`; `P6`, `P3` and
`P4.mujoco_files` unchanged; `wc -l` of both files under 1,000; `behavior.sh compare`
(with the tests) prints `same`.
**Net.** `layout_plan_test`, `plan_replay_test`; capture `plan_layout.*`.
**Size.** 4 files, about 310 lines moved.
**Touches.** `src/layout_plan.cc`, `src/layout_plan_assemble.cc` (new), `BUILD.bazel`
(section `layout_plan`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P11 **Now**).
**After.** B33
**State.** open

## B35 Move the plan's file format out of src/layout_replay.cc

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_replay.cc` (sections "json" and "sha-256", lines 189 to 518),
`src/layout_plan_file.cc` (new).
**Evidence.** `measure.sh --list P11` prints `src/layout_replay.cc 1812`. Its opening
comment names five parts: motion, the plan as a file, the player, the distances, the
verifier. The file part uses only its own helpers and `Fmt`.
**Change.** Move lines 189 to 518 (`G9`, `F6`, `Fig`, `Q`, `B`, `Ints`, `ConfigText`,
`Lines`, `SegmentText`, `FiguresText`, `Get`, `ConfigFrom`, `SegmentFrom`,
`PlanToJson`, `PlanFromJson`, class `Sha256`, `Sha256OfFile`) unchanged into
`src/layout_plan_file.cc` (new), with an opening comment on the plan's file format and
its hash. `Fmt` (lines 193 to 202) is used by both files: put a copy of it, with its
1024-byte buffer, in the new file's anonymous namespace and keep the original, so that
no message is cut differently. Add the file to the `srcs` of `layout_plan` and to group
`layout` of `deps.json`; move `@nlohmann_json` use with it (both files keep the
include they need).
**Done when.** `wc -l src/layout_replay.cc` prints about 1,495; `P11` unchanged;
`deps.py` prints ok; `behavior.sh compare` (with the tests) prints `same`.
**Net.** `plan_replay_test` (`Json.*`, `Committed.*`), `layout_plan_test`; captures
`plan_layout.*`.
**Size.** 4 files, about 330 lines moved.
**Touches.** `src/layout_replay.cc`, `src/layout_plan_file.cc` (new), `BUILD.bazel`
(section `layout_plan`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`.
**After.** B33
**State.** open

## B36 Move the plan's verifier into src/layout_verify.cc

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_replay.cc` (section "verify": `StepName` to class `Verifier`,
lines 835 to 1644 at the base commit, and `VerifyPlan`, 1734 to 1810),
`src/layout_verify.cc` (new).
**Evidence.** After B35 `src/layout_replay.cc` is still about 1,495 lines; the verifier
is the largest part. `Schedule` (1646 to 1731) uses `Measurer` of the distances part.
**Change.** Move the lines named unchanged into `src/layout_verify.cc` (new), opening
with a comment on what the verifier checks of a whole plan. `Schedule` stays in
`src/layout_replay.cc` beside `Measurer`, leaves the anonymous namespace and is
declared in `src/layout_plan_internal.h` (new) as `void VerifySchedule(...)` with its
parameters; the call in `VerifyPlan` uses that name. Copy `Fmt` (1024 bytes) and
`ToVector` (lines 30 to 35) into the new file's anonymous namespace. Add the file to the
`srcs` of `layout_plan` and to group `layout` of `deps.json`.
**Done when.** `measure.sh` prints `P11.cc 3`, `P11 5`; both files under 1,000 lines;
`P4.mujoco_files` unchanged; `behavior.sh compare` (with the tests) prints `same`.
**Net.** `plan_replay_test` (`Committed.PassesTheVerifier`, `BreakagesAreFound`,
`TamperingIsFound`), `layout_plan_test`; captures `plan_layout.*`.
**Size.** 5 files, about 890 lines moved.
**Touches.** `src/layout_replay.cc`, `src/layout_verify.cc` (new),
`src/layout_plan_internal.h` (new), `BUILD.bazel` (section `layout_plan`),
`tools/architecture/deps.json`, `docs/architecture/baseline.txt`,
`docs/architecture/principles.md` (P11 **Now**).
**After.** B35
**State.** open

## B37 Move the checker's class of src/layout_check.cc into a private header

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_check.cc` (35 to 274), `src/layout_check_internal.h` (new).
**Evidence.** `measure.sh --list P11` prints `src/layout_check.cc 2610`. Every check is
a method of class `Checker` in an anonymous namespace.
**Change.** As B33 for `//:layout_check`: write `src/layout_check_internal.h` (new),
group `layout`, namespace `cell::layout`, opening comment of at least five lines; move
into it unchanged the helpers `Fmt` (4096 bytes), `V`, `Wanted`, `ExactExtent`, `ToDeg`
(as `inline`), the structs `ArmCtx`, `Target`, `IkResult`, `PointResult`, `LegResult`,
the declaration of class `Checker` (169 to 274), and every helper of the anonymous
namespace that B38 to B40 will need in two files (`Row`, `ByDesign`, `InsideTrack`,
`Phase`, `PhaseTime`, `ArmsNote`; the compiler names any missed). No `xpos` read moves
(P3 allows `src/layout_check.cc` by name), and the header includes no `<mujoco/...>`
directly. List it in the `srcs` of `layout_check` (section `layout_check`) and in
group `layout` of `deps.json`.
**Done when.** `P11 5` still, `P3 5`, `P11.comment 0`; `deps.py` prints ok;
`behavior.sh compare` (with the tests) prints `same`.
**Net.** `layout_check_test`, `layout_rows_test`, `layout_scene_test`,
`layout_plan_test`, `plan_replay_test`; captures `check_layout.*`, `plan_layout.*`.
**Size.** 4 files, about 250 lines moved.
**Touches.** `src/layout_check.cc`, `src/layout_check_internal.h` (new), `BUILD.bazel`
(section `layout_check`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`.
**After.** B12, B23, B24, B29
**State.** open

## B38 Move the solving of the arms' poses into src/layout_check_solve.cc

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_check.cc` (`Checker::Init` to `Checker::SolveAll`, lines 275 to
1039 at the base commit), `src/layout_check_solve.cc` (new).
**Evidence.** After B37 `src/layout_check.cc` is about 2,370 lines; inverse kinematics,
the track candidates, the margins, the postures, the walk of each leg and the order
of the steps are one part (the `Solve*`, `WalkLeg`, `StepOrder` methods).
**Change.** Move the methods named unchanged into `src/layout_check_solve.cc` (new),
opening with a comment on how the checker finds each arm's poses and walks its legs;
include `src/layout_check_internal.h` (new). Add the file to the `srcs` of
`layout_check` and to group `layout` of `deps.json`.
**Done when.** `wc -l src/layout_check.cc` about 1,600; `P3 5`, `P4.mujoco_files`
unchanged; `behavior.sh compare` (with the tests) prints `same`.
**Net.** As B37.
**Size.** 4 files, about 765 lines moved.
**Touches.** `src/layout_check.cc`, `src/layout_check_solve.cc` (new), `BUILD.bazel`
(section `layout_check`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`.
**After.** B37
**State.** open

## B39 Move the rows of reach, clearance, transport, hold, load and ways out

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_check.cc` (`Checker::ReachRows` to `Checker::WaysRows`, lines
1053 to 1554 at the base commit), `src/layout_check_rows.cc` (new).
**Evidence.** After B38 `src/layout_check.cc` is about 1,600 lines; these six kinds of
`AGENTS.md`'s map ("transport, hold, reach, clearance, load, ways") are one part.
**Change.** Move `ReachRows`, `FreeSpace`, `ClearanceRows`, `ToolInFreeSpace`,
`TransportRows`, `HoldRows`, `LoadRows`, `WaysRows` unchanged into
`src/layout_check_rows.cc` (new), opening with a comment on what those six rows check;
include `src/layout_check_internal.h` (new). Add it to the `srcs` of `layout_check` and
to group `layout` of `deps.json`.
**Done when.** `wc -l src/layout_check.cc` about 1,090; `P3 5`, `P4.mujoco_files`
unchanged; `behavior.sh compare` (with the tests) prints `same`.
**Net.** As B37.
**Size.** 4 files, about 500 lines moved.
**Touches.** `src/layout_check.cc`, `src/layout_check_rows.cc` (new), `BUILD.bazel`
(section `layout_check`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`.
**After.** B38
**State.** open

## B40 Move the time model and the time rows into src/layout_check_time.cc

**Principles.** P11
**Decisions.** ADR 0017
**Where.** `src/layout_check.cc` (`Checker::MoveTime` to `Checker::TimeRows`, lines
1555 to 1948, and `JointMoveTime` to `TimeModelText`, 2441 to 2513, at the base
commit), `src/layout_check_time.cc` (new).
**Evidence.** After B39 `src/layout_check.cc` is about 1,090 lines; the time model
(`ModelMoveSeconds`, `PhaseEnd`, `JointMoveTime`, `SideBySide`, `TrackMoveTime`,
`TimeModelText`, declared in `src/layout_check.h`) and the rows of kind `time` are one
part.
**Change.** Move them unchanged into `src/layout_check_time.cc` (new), opening with a
comment on how a bay's time is reckoned; include `src/layout_check_internal.h` (new).
Add it to the `srcs` of `layout_check` and to group `layout` of `deps.json`.
**Done when.** `measure.sh` prints `P11.cc 2`, `P11 4` (= `P11.frozen` plus
`src/bay_hardware.h`, the target of P11); every file of `//:layout_check` under 1,000
lines; `behavior.sh compare` (with the tests) prints `same`.
**Net.** As B37.
**Size.** 4 files, about 470 lines moved.
**Touches.** `src/layout_check.cc`, `src/layout_check_time.cc` (new), `BUILD.bazel`
(section `layout_check`), `tools/architecture/deps.json`,
`docs/architecture/baseline.txt`, `docs/architecture/principles.md` (P11 **Now**).
**After.** B36, B39
**State.** open

## Left as it is

By principle, with the figure `measure.sh` (or `warnings.sh`, `tidy.sh`) prints today.

- **Everything in the stand-in cell's files** (ADR 0020): they are to be deleted, not
  converted, and the gate runs on them. The eleven departures of `deps.json` among
  them carry `"item": "left"`. P1: `P1.frozen 4` (`src/held_module_finder.h`,
  `src/module_finder.cc`, `src/module_finder.h` include `src/pallet.h`;
  `tests/locating_gate_test.cc` includes `src/bay_hardware.h`). P2: `P2.frozen 2`
  (`src/view_cell.cc`, and `src/run_gate.cc` until B9). P3: `P3.frozen 5`, which is
  P3's target. P5: `P5.frozen 7` (`//:check_camera_cell_test //:depth_camera`,
  `//:pick_place //:path_timing`, `//:trial_runner_test` with `//:cell`, `//:pick_place`
  and `//:score`, `@eigen` of
  `//:print_plane_fit` and `//:print_points`): their targets build frozen files, and
  `//:pick_place` is the bottleneck of `AGENTS.md`. P6: `P6.frozen 7`. P7: `P7.frozen 9`
  (`mj_geomDistance` in `src/check_placement.cc` and `src/cycle_check.cc`, the module's
  size in `src/conveyor.h` and `src/suction.h`, pi in five). P11: `P11.frozen 3`
  (`src/pick_place.cc` 1,911, `src/rectangle_fit.cc` 1,252, `src/pick_place.h` 1,173).
  The places of strict flags and checks in frozen files: `-Wconversion` 43, `-Wshadow`
  2, `-Wcomment` 2, `-Wparentheses` 1; `cert-err34-c` 43 (and "Needs the owner"),
  `cppcoreguidelines-pro-type-member-init` 12, `cert-flp30-c` 7,
  `bugprone-integer-division` 3, `bugprone-misplaced-widening-cast` 2,
  `bugprone-suspicious-memory-comparison` 1, `clang-analyzer-deadcode.DeadStores` 1,
  `clang-analyzer-cplusplus.StringChecker` 1, and 64 places of
  `bugprone-unchecked-optional-access`. Of these, the loops of
  `tests/suction_test.cc:302` and `src/check_placement.cc:180` may skip their last
  sample, which matters only as long as the stand-in lives.
- **`src/viewer.cc` in eight programs** (P5, `P5.files 1`). The honest cure is one
  library holding the `main` of `src/viewer.cc`, linked by eight binaries with no
  `srcs`; `tools/architecture/deps.py` would then count the library as an unused dep
  of each binary, since it offers no header. One comprehension over the eight would
  lower the figure while the file is still compiled eight times, which P5's Check calls
  gaming. The file is 96 lines. Left until `deps.py` learns that a binary without
  `srcs` uses its one library.
- **`src/bay_hardware.h`, 976 lines** (P11, `P11.h`): a table of the hardware's figures
  with their sources, read as one table; P11's target counts it.
- **The gate's figures in `src/bay_hardware.h:502`** (P7, `P7.gate 1`): the home is
  `src/trial_runner.h`, frozen, which `hardware` may not include;
  `tests/locating_gate_test.cc` holds the two copies equal.
- **One build file and flat directories** (ADR 0018) and **namespaces as they are**
  (ADR 0019): decided, with the reasons there; decided again when the stand-in goes.
- **The pictures and recordings** (P8, `P8.no_test 6`): they need a display or
  matplotlib and are judged by eye; each entry of `deps.json` names its commands.
- **The `boost.*` modules other than `boost.config`** (P4): `dependencies.md`, "Not
  needed, by the query", finds OMPL's sources include each of them by name.
- **`docs/ompl-integration.md`** (P10, `P10.left_out 1`): the record of four trial
  builds whose paths were never on `main`.

## Needs the owner

- **Malformed numbers on the command line** (P9, `T cert-err34-c 60`, 17 places outside
  frozen files: `src/plan_layout.cc`, `src/print_profile.cc`, `src/print_rrt.cc`,
  `src/viewer_lib.cc`, `tests/layout_check_test.cc`, `tests/plan_replay_test.cc`).
  `atoi`, `atof` and `sscanf` take `--seed 3x` as 3 and `abc` as 0 without a word, and
  overflow is undefined behavior. The cure, one parser in `base` that refuses what is
  not wholly a number, makes such input print the usage and exit 1: a program's output
  and exit code change (`behavior.md`, section 1). The owner decides whether that is
  wanted; then it is an item per group, and the 43 frozen places stay.
- **The clock-bounded searches** (P6, `P6.clock 6`): `src/rrt.cc:451` stops RRT-Connect
  after `RrtOptions::max_seconds` (5 s by default, `src/rrt.h:240`), `src/rrt.cc:517`
  path shortening, `src/ompl_search.cc:417` OMPL, and `src/motion_planner.cc:343`,
  `:372` hand the time left on. On a slow machine a search can end in another state, so
  a plan depends on the machine's speed. The cure, a budget of queries in place of
  seconds, changes when searches stop and so what `print_plan`, `run_cell` and
  `plan_layout --summary` print; and `src/pick_place.cc` (frozen) passes such budgets.
  The owner decides the budgets.
- **`tests/rrt_test.cc:373`** (P6): `EXPECT_LT(seconds, 2.0)` bounds a wall-clock time
  in a test, which `AGENTS.md` forbids; the test already asserts the count
  (`report.iterations`). Removing the bound changes a test's bound. Also
  `tests/planned_carry_test.cc` (frozen) reads the clock for more than a print.
- **The sample times of `src/layout_plan.cc:825`** (P9, `cert-flp30-c`): `t += sample_s`
  accumulates rounding; an integer counter changes the instants at which two arms are
  checked against each other, and possibly a plan and `plans/*.json`.
- **The four `Fmt` buffers of the layout code**: 4096 bytes in `src/layout_check.cc:51`
  and `src/layout_rows.cc:39`, 2048 in `src/layout_plan.cc:38`, 1024 in
  `src/layout_replay.cc:194`; a longer message is cut without a word. One `Fmt` would
  change where messages are cut.
