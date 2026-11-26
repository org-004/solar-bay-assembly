# What "behavior" means for a refactoring here

For an agent that is about to restructure code. It says what you may not change, how
to show that you did not, and what the proof does not reach. The decision is ADR 0021,
the principle P2 of [`principles.md`](principles.md). Every command below was run at
the commit this file lands with, on the engineer's machine (6 cores, 7.4 GB, three
jobs). Put Bazel on the path first: `export PATH=$HOME/.local/bin:$PATH`.

## 1. Three classes

A refactoring changes how the code is built and arranged and nothing that is in the
first class. The second class changes only when every reference to the item changes in
the same commit. The third is not checked.

| Item | Class | What it means for you | Held by |
|---|---|---|---|
| The set of test targets and their results | must stay | Do not add, remove or rename a test target, and none that passed may fail. A new test is its own commit. | `tests.txt` and `results.txt` of a capture |
| The gate | must stay | `//:gate_test` (tag `gate`, `docs/gate.md`) passes as before. It runs once, alone, before code lands on `main` (`AGENTS.md`). | `capture --gate`; `tests/gate_test.cc` |
| What each program prints, its flags, its exit codes | must stay | The bytes on stdout and stderr, the options and the exit status of every program in section 3 do not change, a failure that exits 1 by design included. | the capture files |
| Committed files a program writes or a test compares | must stay | `models/` (`models/bay.xml` is what `//:gen_bay` prints), `layouts/`, `plans/`, `docs/img/` and the GIFs under `docs/`. You do not regenerate one to make a test pass. | `tests/bay_model_test.cc`, `tests/plan_replay_test.cc`; `gen_bay.default` and `plan_layout.@plan.*`; the pictures by nothing (section 6) |
| Formats of the layout and plan files | must stay | Keys, units and meaning in `layouts/*.json` and `plans/*.json` (`docs/layout.md`). A reader of the old file keeps working. | `tests/layout_model_test.cc`, `tests/layout_plan_test.cc`, `tests/plan_replay_test.cc` |
| Environment variables a program reads | must stay | `MODEL_PATH`, `LAYOUT_DIR`, `PLAN_DIR`, `MODELS_DIR`, `UR10E_XML`, `PLAN_DEBUG`, `PLAN_SKIP_VERIFY`, `BUILD_WORKING_DIRECTORY` and the others `grep -rhno 'getenv("[A-Z_0-9]*")' src tests` prints keep their names and meaning; `BUILD.bazel` sets some of them. | the programs' captures; the tests that set them |
| Bazel labels named in `AGENTS.md`, `README.md`, `docs/`, `tools/` and CI | may change with every reference | Change the label and every place that names it in one commit (`grep -rn '//:<old>' .`). Two labels are also held: those of the test targets (`tests.txt`) and of the programs (`programs.txt`); renaming one makes `compare` differ, so that commit is reported, not shown `same`. | `grep`; `.github/workflows/ci.yml` |
| Include paths | may change with every reference | Moving a header changes every `#include` of it and its `BUILD.bazel` entry in the same commit. | the build |
| Namespaces, names of functions and types | may change with every reference | Rename with all callers in the same commit. Never in a frozen file, nor its include paths (ADR 0020, `P2.frozen`). | the build and the tests |
| How long things take, how much memory they need | free, within reason | A change may make a program slower or leaner. A test's `size` in `BUILD.bazel` is not raised, and no timeout is added, to let a slower test pass. `AGENTS.md` forbids bounding wall-clock time in a test: bound a count. Memory is a hard limit on the machine (7,445 MiB) and a change that needs more is reported. | nothing |

Why. The tests and the gate are the owner's definition of correct (`AGENTS.md`, "What
correct means is the owner's"); the programs print every figure the documents state.
A name or a path only lets one piece of code refer to another, and a compiler finds a
stale one; a stale label in a document it does not, hence the `grep`.

## 2. How a change is shown not to have altered it

On the same machine, before and after the change:

    tools/architecture/behavior.sh capture <dir-before> --no-tests    # before the change
    tools/architecture/behavior.sh capture <dir-after> --no-tests     # after it
    tools/architecture/behavior.sh compare <dir-before> <dir-after>

`<dir>` is empty or absent. `--no-tests` leaves out the results of the tests, without
it the whole suite runs; `--gate` adds `//:gate_test`. A change that can alter a test's
result is captured without `--no-tests`, both times. A capture uses `--config=opt`,
three jobs and no display. It is a Bazel command like any other: one at a time in a
checkout (`AGENTS.md`, "Caches, locks and worktrees").

A capture holds: one file for each line of `tools/architecture/programs.txt` (for
`@layout` one for each name `check_layout --list` prints, for `@plan` one for each
`plans/*.json`), each with `exit: N`, then `--- stdout`, then `--- stderr`;
`tests.txt`, the test targets from `bazel query 'tests(//...)'`; and, without
`--no-tests`, `results.txt`, one line `<target> <RESULT>` from Bazel's summary. A
program is run as `bazel run` runs it, with its arguments from `BUILD.bazel`, but
started by the script `bazel run --script_path` writes, so that no Bazel message
reaches the output. The wall-clock fields are replaced by `<wall>` (section 4).

Both captures are made on one machine because the planner's draws come from
`std::uniform_real_distribution` (`src/rrt.cc` 442), which differs between standard
libraries: a capture from CI is not the engineer's (P6).

`same` means: `compare` printed `same` and exited 0. Every program file is the same
byte for byte, `tests.txt` is the same, and the results are the same when both sides
have them. Anything else is a line naming the program (first differing line from each
side), the test target, or the result, and exit 1. When it differs, stop. Do not read
the difference as noise, loosen a cut, edit `programs.txt` or re-run until it passes:
the change is not a refactoring (ADR 0021). Report what `compare` printed, and either
revert or ask the owner to decide that the new output is meant.

Two captures of one commit are the same bytes. On 2026-11-25 three captures were made,
two in one checkout and one in a second clone, and `behavior.sh compare` printed `same`
for each of the three pairs. A change of one figure in a layout file makes `compare`
name `check_layout` for that layout (R26 of the spec, shown in the run's report).

## 3. The programs captured

All targets are `bazel run --config=opt`. "-" is no arguments. Each line is one file in
the capture; `@layout` is each of the 8 names of `layouts/*.json` and `@plan` each of
the 2 files of `plans/`, so a capture holds 58 program files (the 42 lines of
`programs.txt`, of which 4 expand: 8 + 8 + 2 + 2 files) and `tests.txt` (42 lines).

| Capture id | Target | Arguments |
|---|---|---|
| `print_bay.default` | `//:print_bay` | - |
| `print_bay.summary` | `//:print_bay` | `--summary` |
| `print_bay.capture` | `//:print_bay` | `--capture` |
| `print_cell.default` | `//:print_cell` | - |
| `print_cell.seed_3_2s` | `//:print_cell` | `3 2` |
| `print_check.csv` | `//:print_check` | `--trials 2` |
| `print_check.pretty` | `//:print_check` | `--trials 2 --pretty` |
| `print_check.noise_calibrated` | `//:print_check` | `--trials 2 --first-seed 5 --noise --calibration 1,0.05` |
| `print_check.no_roof` | `//:print_check` | `--trials 2 --no-roof --position 3 --angle 0.5` |
| `print_depth.nominal` | `//:print_depth` | `--nominal` |
| `print_depth.noise_last` | `//:print_depth` | `--seed 2 --noise --last` |
| `print_depth.points` | `//:print_depth` | `--nominal --points` (digest) |
| `print_pallet.default` | `//:print_pallet` | - |
| `print_perception.csv` | `//:print_perception` | `--trials 2` |
| `print_perception.pretty` | `//:print_perception` | `--trials 2 --pretty` |
| `print_perception.noise_depths` | `//:print_perception` | `--trials 2 --first-seed 3 --noise --calibration-error 1,0.05 --depths 0,29 --no-creases --arm-home` |
| `print_plan.pretty` | `//:print_plan` | `--pretty --seeds 5` |
| `print_plan.module_29_slot_9` | `//:print_plan` | `--pretty --module 29 --slot 9 --seeds 3` |
| `print_plan.return` | `//:print_plan` | `--pretty --return --seeds 3` |
| `print_plan.csv_summary` | `//:print_plan` | `--seeds 5 --summary` |
| `print_plane_fit.csv`, `.pretty` | `//:print_plane_fit` | -, `--pretty` |
| `print_points.default`, `.seed_7` | `//:print_points` | -, `7` (digest) |
| `print_profile.csv`, `.pretty` | `//:print_profile` | -, `--pretty` |
| `print_rrt.circle` | `//:print_rrt` | - |
| `print_rrt.gap_seed_2` | `//:print_rrt` | `gap 2` |
| `run_cell.nominal_2_modules` | `//:run_cell` | `--nominal --modules 2` |
| `run_cell.nominal_2_modules_pretty` | `//:run_cell` | `--nominal --modules 2 --pretty` |
| `run_cell.trials_2` | `//:run_cell` | `--trials 2` |
| `run_cell.phases` | `//:run_cell` | `--nominal --phases` |
| `run_cell.planned` | `//:run_cell` | `--nominal --carry planned` |
| `run_cell.check` | `//:run_cell` | `--nominal --check` |
| `step_arm.default` | `//:step_arm` | - |
| `gen_bay.default` | `//:gen_bay` | - (prints `models/bay.xml`) |
| `check_layout.list` | `//:check_layout` | `--list` |
| `check_layout.@layout` | `//:check_layout` | `<layout>` (all checks) |
| `check_layout.@layout.steps` | `//:check_layout` | `<layout> --steps` |
| `check_layout.flat_rack_3_pin_arms.checks` | `//:check_layout` | `flat_rack_3_pin_arms --checks port,approach` |
| `plan_layout.@plan.margins` | `//:plan_layout` | `<plan> --margins` (reads the plan) |
| `plan_layout.@plan.summary` | `//:plan_layout` | `<plan> --summary` (plans afresh) |

`digest` means the output is replaced by its line count, byte count and SHA-256, for
a table too large to keep; any change still shows.

## 4. Not compared

### 4a. Fields cut from a capture because they are wall-clock times

Each is replaced by `<wall>` (and the blanks before it by one), and only the field
named. The line that shows it is a time is given; open it before you rely on the cut.
Lines are of the commit this file lands with; `tools/architecture/programs.txt` names
the same lines in its comments.

| Program (capture ids) | Field | Source line that shows it is a time |
|---|---|---|
| `print_cell.*` | "settled .. in X s wall" | `src/print_cell.cc` 102; the clock is `steady_clock` at 95 to 100 |
| `print_cell.*` | "in X s wall: steps/s, x real time" (three figures of one division by the wall time) | `src/print_cell.cc` 119; clock at 111 to 116 |
| `print_check.*` | columns `render_ms`, `fit_ms` | `src/print_check.cc` 364 and 365 (`render_seconds`, `seconds_total`) |
| `print_check.*` | stderr lines `render ms`, `fit ms` (worst, rms, mean) | `src/print_check.cc` 411 and 412, summed at 274 and 275 |
| `print_depth.*` | "cast in X ms" | `src/print_depth.cc` 195; clock at 154 to 158 |
| `print_pallet.default` | "N steps in X s wall: steps/s, x real time" | `src/print_pallet.cc` 116; clock at 73 and 111 |
| `print_perception.*` | columns `render_ms`, `fit_ms` | `src/print_perception.cc` 307 and 308; render clock at 241 to 246 |
| `print_perception.*` | stderr lines `render ms`, `fit ms` | `src/print_perception.cc` 368 and 369, summed at 264 and 265 |
| `print_plan.*` | columns `search_ms`, `shorten_ms` | `src/print_plan.cc` 440 (`search_seconds`, `shorten_seconds`, from `Clock` in `src/motion_planner.cc` 25) |
| `print_plan.*` | line "# planning ms, median / worst" | `src/print_plan.cc` 478 to 481, summed at 446 to 448 |
| `print_plan.csv_summary` | columns `ms_median`, `ms_worst` | `src/print_plan.cc` 464 to 471 (`Median(total)`, `Max(total)`) |
| `run_cell.*` | columns `finder_ms`, `plan_ms` | `src/run_cell.cc` 265 and 278 (`finder_seconds`, `plan_seconds`; clock at `src/pick_place.cc` 756 to 817) |
| `run_cell.*` | columns `check_finder_ms`, `replan_ms` | `src/run_cell.cc` 242 and 243; `--pretty` column `ms` at 220; clock at `src/pick_place.cc` 909 to 921 |
| `run_cell.*` | summary "planning (wall): mean, worst", "finder N ms at most", "rechecked .. mean, worst ms; replanned: mean, worst ms" | `src/run_cell.cc` 744, 761, 771, 782 to 786 |
| `run_cell.*` | stderr "simulated S s in W s of wall time: Nx real time" | `src/run_cell.cc` 791; clock at 573 and 615 |
| `run_cell.phases` | "(finder N ms)" at the end of a look | `src/pick_place.cc` 1576 and 1892 (`seconds_total` of the finder, `src/module_finder.cc` 982) |
| `plan_layout.@plan.summary` | "planning_s:" | `src/plan_layout.cc` 401, from `result_.seconds` at `src/layout_plan.cc` 1325 |

One time is in neither a cut nor a capture. `src/pick_place.cc` 1077 prints "finder N
ms" in the note of a check, which `run_cell --phases --check` shows; no capture passes
both flags, and `--phases` alone has no check. Do not add that pair without a cut.

### 4b. Programs and flags not in the list

`bazel query 'kind(cc_binary, //...)'` prints 27 targets. 16 are in section 3, one
for each program named there. The 11 others are what
`tools/architecture/measure.sh --list P2.unrun` prints, and are here:

| Program | Why it is not captured | What stands in for it |
|---|---|---|
| `//:viewer`, `viewer_arm`, `viewer_bay`, `viewer_cell`, `viewer_container`, `viewer_conveyor`, `viewer_pallet`, `viewer_suction` (all `src/viewer.cc`), `//:viewer_layout` (`src/view_layout.cc`), `//:view_cell` (`src/view_cell.cc`) | They open a window and need a display. | `tests/layout_model_test.cc` loads the layouts as models; the window is held by nothing. |
| `//:run_gate` | With its default 8 trials of 30 modules it runs for minutes. | `//:gate_test`, which runs the same trials (`capture --gate`). |

Flags of captured programs that are left out:

| Program and flag | Why | What stands in for it |
|---|---|---|
| `run_cell --planner ompl`, `print_plan --planner ompl` | OMPL's planners are seeded from the clock: two runs differ. | The same program with the default planner (above), and `tests/ompl_search_test.cc`. |
| `print_rrt --sweep`, `--shorten-sweep`, `--nn` | Minutes of timing tables. | `print_rrt` default and `gap 2`, which print the search itself. |
| `print_check --study` | Not run by any capture; see section 6. | Nothing. |
| `plan_layout <name> --out PATH` | Writes a file; the committed `plans/` are read by `--margins` and `tests/plan_replay_test.cc`. | `plan_layout.@plan.margins`. |

## 5. Measured at this commit

Machine: 6 cores, 7.4 GB, three jobs (`--jobs=3 --local_resources=cpu=3`). Each figure
was measured by `/root/work/013-design-principles-and-r2/logs/m3_heavy.py` under the
machine's lock, with Python's `time.monotonic()`; the memory is the largest reading,
once a second, of `memory.current` of the whole container, so it includes Bazel's
server and the page cache and is more than the programs need.

| Capture | Wall time | Largest `memory.current` |
|---|---|---|
| `capture <dir> --no-tests`, warm build | 182.2 s | 5,745 MiB |
| `capture <dir>`, warm build, every test result cached | 189.3 s | 5,438 MiB |

The container already held 4,202 and 4,436 MiB before each capture started, so a
capture added about 1,500 and 1,000 MiB to what was there. The time includes waiting
for the lock, if another agent held it. In the capture with tests all 41 results (42
test targets less the gate; `wc -l results.txt` prints 41, each a line such as
`//:arm_test PASSED`) came from Bazel's cache because nothing had changed (the lead who ran it says so;
`results.txt` does not record it). So 189 s is a lower bound: a capture after a change
runs again every test whose inputs changed, up to the whole suite without the gate,
which `AGENTS.md` puts at 417 s on three cores ("Where the time goes"). Nothing here
was measured with `--gate`.

## 6. Outside the net

This is what neither a capture nor a test holds. Before you touch any of it, write a
test that pins what it does now (right or wrong) in a commit of its own that changes
nothing else, or leave it as it is.

### 6a. Files under `src/` on which no test target depends

`tools/architecture/measure.sh --list P2.untested` prints 24 files. Sixteen of them are
the programs' own `main` files and are held by the capture named; the other eight are
held by nothing.

| File | Held by |
|---|---|
| `src/check_layout.cc` | `check_layout.*` |
| `src/plan_layout.cc` | `plan_layout.*` |
| `src/gen_bay.cc`, `src/step_arm.cc` | `gen_bay.default`, `step_arm.default` |
| `src/print_<name>.cc` for `bay`, `cell`, `depth`, `pallet`, `perception`, `plan`, `plane_fit`, `points`, `profile`, `rrt` | the `print_<name>.*` captures |
| `src/print_check.cc` | `print_check.*`, except `--study` |
| `src/run_cell.cc` | `run_cell.*` |
| `src/check_placement.cc`, `src/check_placement.h` | nothing: only `print_check --study` calls them (`src/print_check.cc` 198), and no capture passes it |
| `src/run_gate.cc` | nothing (`//:gate_test` runs the same trials by other code) |
| `src/view_cell.cc`, `src/view_layout.cc`, `src/viewer.cc` | nothing: they need a display |
| `src/viewer_lib.cc`, `src/viewer_lib.h` | nothing: only the viewers use them |

`tools/architecture/measure.sh --list P2` prints 6 (the last three rows); it counts the
two `check_placement` files as reached because a captured binary is linked with them,
which says less than the table above. A capture of a program holds the program's flags
in section 3 and nothing else of the file: another flag of `print_check` is as
unpinned as a file that no capture reaches. To pin a file, run its program with the
flags that use it and put the result in `programs.txt`, or write a test.

### 6b. Programs that no capture and no test runs

`tools/architecture/measure.sh --list P2.unrun` prints 11: the ten viewers and
`run_gate`, in section 4b. No test runs any program either
(`bazel query 'kind(cc_binary, deps(tests(//...))) intersect //:*'` prints nothing), so
the programs of section 3 are held by captures alone: without a capture, no test
notices a changed table. Do not restructure `src/viewer.cc`, `src/view_layout.cc`,
`src/view_cell.cc`, `src/viewer_lib.*` or `src/run_gate.cc` before a test pins the part
you touch; a viewer's drawing can be left alone, its argument parsing
(`ParseViewerArgs`) can be tested.

### 6c. What a viewer or a recording shows that nothing compares

The pictures under `docs/img/` (19 files), the GIFs under `docs/` (`bay_planned.gif`,
`blind_vs_camera.gif`, `check.gif`, `cycle.gif`, `module_planned.gif`, `pallet.gif`,
`pick.gif`, `pins_planned.gif`) and the scripts that make them (`tools/video/`,
`tools/plots/`) are held by no test and no capture. They need a display or the viewers
and show what the code did on the day they were made. A refactoring leaves them as
they are, and does not regenerate them to look the same; one that changes what they
show is not a refactoring.

### 6d. What a capture does not hold even for a captured program

Flags and arguments not in section 3, and the programs that take a layout name or a
seed: only the seeds and names listed are run. The behavior on another platform: a
capture is of one machine, and says nothing about another, CI's included (P6). Time and memory
(section 1), and a test's own output (only its result). A `same` is a statement about
these 58 files and the results, not about the code.
