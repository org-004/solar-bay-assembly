# Audit of the structure at 940a01b (agent `audit`)

Checkout `/root/work/013-design-principles-and-r1/repo`, branch tip `142db79` (one commit, `tools/architecture`, on `940a01b`; `git diff --stat 940a01b 142db79 -- src tests BUILD.bazel` is empty, so every figure below is true of `940a01b`).
Every figure has the command that printed it. Commands run from the repo root after `export PATH=$HOME/.local/bin:$PATH`. Scratch (include table, rule dump) is in `/root/work/013-design-principles-and-r1/out/audit/`.
"Context said X; at 940a01b it is Y" marks a correction. `src/X.h:N` lines are at 940a01b.

## 0. The corrections that matter most

1. Context said "two inverse-kinematics solvers ... `Checker::Solve` at `src/layout_check.cc:469`"; at 940a01b it is `src/layout_check.cc:299` (declared :220), and there is no third solver (`grep -n 'SolveIk\|Solve(' src/layout_plan.cc src/layout_scene.cc src/layout_replay.cc src/layout_rows.cc` prints nothing).
2. Context said `src/print_layout_moves.cc` keeps a private copy of the moving and obstacle sets; at 940a01b the file is gone (`ls src/print_layout_moves.cc` fails; `git diff --stat --diff-filter=D 68a1199 940a01b -- src tests` lists only it, 644 lines), the one copy is `LayoutScene::Moving/Obstacles/WayMoving/WayObstacles` (`src/layout_scene.cc:178,221,483,497`) and `LeastDistance` (`:531`). But `docs/layout.md:667` still tells the reader to run `//:print_layout_moves` (a stale command; `:11` says it no longer exists).
3. Context said "every header is in `cell`; four add `cell::layout`, two `cell::bay`, one `cell::container`"; at 940a01b 31 of 42 headers open `namespace cell {` and 11 open only a nested one, with a new fourth family `cell::flat_rack` (`src/flat_rack.h:50`): `cell::layout` in 7 headers, `cell::bay` in 2, `cell::container` 1, `cell::flat_rack` 1 (section 2).
4. Context said `src/collision.h` includes `arm.h` and `pallet.h` and `src/motion_planner.h` includes `arm.h`; at 940a01b `src/collision.h:119-120` includes `src/arm.h` and `src/mj_handles.h` (no `pallet.h`), and `src/motion_planner.h:141-145` includes no `arm.h` directly (it reaches it through `collision.h`). The new cell now reaches the stand-in's code through headers: `src/layout_plan.h:137` includes `src/joint_move.h` (which includes `src/profile.h`), and `layout_replay.cc:21-22` uses `path_timing.h` and `rrt.h` (section 3).
5. Context said `mj_geomDistance` is called from five files incl. `print_layout_moves.cc` and `layout_check.cc`; at 940a01b it is four files in `src/` (`collision.cc:526`, `cycle_check.cc:306`, `check_placement.cc:490`, `layout_scene.cc:575,586`); the capsule workaround is not in a distance function but in the model loader `src/layout_model.cc:717-731`, so every stand-in caller (`collision`, `cycle_check`, `check_placement`) measures against the UR10e's capsules unmended (section 4b).
Others: sizes (99 src files, 41 tests, 110 rules, 42 tests; section 1); `@eigen` 78 / `@mujoco` 67 / `@googletest` 42 / `@nlohmann_json` 4 / `@ompl` 1 / `@glfw` 1 (section 11); `src/check_placement.*` is reached by no test (section 6); `src/rrt.cc` draws with `std::uniform_real_distribution`, whose output the standard does not fix (section 8); `bazel mod graph` and `mod deps` still fail (section 11).

## 1. Size

| What | Command | Context (68a1199) | Now (940a01b) |
|---|---|---|---|
| files in `src/` | `ls src \| wc -l` | 91 | 99 (all `.h` or `.cc`: `ls src \| grep -v '\.\(h\|cc\)$'` prints nothing) |
| headers | `ls src/*.h \| wc -l` | 38 | 42 |
| `.cc` | `ls src/*.cc \| wc -l` | 53 | 57 |
| lines, headers | `cat src/*.h \| wc -l` | | 10,945 |
| lines, `.cc` | `cat src/*.cc \| wc -l` | | 28,808 |
| lines, `src/` | `cat src/*.h src/*.cc \| wc -l` | 32,476 | 39,753 |
| files in `tests/` | `ls tests \| wc -l` | 36 | 41 (all `*_test.cc`) |
| lines in `tests/` | `cat tests/* \| wc -l` | 22,929 | 25,898 |
| `BUILD.bazel` lines | `wc -l BUILD.bazel` | 1,989 | 2,133 |
| `visibility` | `grep -c visibility BUILD.bazel` | 0 | 0 |
| `copts`/`features` | `grep -c 'copts\|features' BUILD.bazel` | 0 | 0 |
| test targets | `bazel query 'tests(//...)' \| wc -l` | 37 | 42 |
| all rules | `bazel query 'kind(rule, //...)' \| wc -l` | 98 | 110 |

Rules by kind (`bazel query "kind($k, //...)" | grep -c '^//'`): `cc_library` 38 (Context 36), `cc_binary` 27 (27), `cc_test` 42 (35), `genrule` 1, `filegroup` 1, `alias` 1 (the `ompl` section). A grep for rules at the start of a line (`grep -o '^[a-z_]*(' BUILD.bazel | sort | uniq -c`) finds only 40 `cc_test(`: two more are made by the comprehension `[cc_test(` at `BUILD.bazel:1469` (`check_cycle_test`, `check_cycle_batch_test`). The 27 binaries are 19 distinct main files plus `src/viewer.cc` built eight times (`viewer`, `viewer_arm`, `_bay`, `_cell`, `_container`, `_conveyor`, `_pallet`, `_suction`).

Ten largest files (`wc -l src/* tests/* | sort -rn | sed -n 2,11p`):
`src/layout_check.cc` 2,610 | `src/pick_place.cc` 1,911 | `src/layout_replay.cc` 1,812 | `tests/check_cycle_test.cc` 1,654 | `src/layout_plan.cc` 1,462 | `src/rectangle_fit.cc` 1,252 | `tests/layout_model_test.cc` 1,247 | `tests/rectangle_fit_test.cc` 1,235 | `tests/layout_check_test.cc` 1,185 | `src/pick_place.h` 1,173.

`# BEGIN`/`# END` sections (command: python scan of `BUILD.bazel` between the markers, `grep -n '^# BEGIN\|^# END' BUILD.bazel` gives the lines; the count is of `kind(` rules whose line lies between them; `[cc_test(` counted as 2): 28 sections, 93 rules of 110; 17 rules (lines 19-169, 445, 655, 668: arm/model helper libraries, `mj_handles`, `viewer_lib`, `viewer`) are outside any section.

| section | rules | section | rules | section | rules |
|---|---|---|---|---|---|
| arm | 4 | rrt | 4 | container | 3 |
| pallet | 4 | path_timing | 3 | flat_rack | 2 |
| conveyor | 3 | motion_planner | 3 | layout | 4 |
| suction | 3 | held_module_finder | 4 | layout_check | 4 |
| follower | 2 | check_camera_cell | 1 | layout_rows | 1 |
| ik | 2 | planned_carry | 1 | locate | 1 |
| cell | 5 | check_cycle | 2 (comprehension) | layout_plan | 4 |
| pick_place | 6 | gate | 4 | | |
| depth_camera | 4 | ompl | 3 | | |
| rectangle_fit | 3 | bay | 8 | | |
| module_finder | 3 | collision | 2 | | |

New sections since 68a1199 (`git diff 68a1199 940a01b -- BUILD.bazel | grep '^+# BEGIN'`): `flat_rack`, `layout_rows`, `layout_plan`, and edits to `layout`, `layout_check`. `AGENTS.md` is 124 lines (`wc -l AGENTS.md`; Context 115) with the sections Build and test, Caches/locks/worktrees, Dividing the work, Rules, Frames and units, Map (`grep -n '^## ' AGENTS.md`). `MODULE.bazel`, `bazel/`, `.bazelrc` unchanged since 68a1199 (`git diff --stat 68a1199 940a01b -- MODULE.bazel bazel .bazelrc | wc -l` prints 0).

Diff `68a1199..940a01b` under `src/` and `tests/`:
- added (`git diff --stat --diff-filter=A 68a1199 940a01b -- src tests`): 14 files, 8,004 lines: `src/flat_rack.h` (145), `layout_plan.cc` (1,462), `layout_plan.h` (468), `layout_replay.cc` (1,812; implements `layout_plan.h`), `layout_rows.cc` (766), `layout_rows.h` (87), `layout_scene.cc` (613), `layout_scene.h` (321), `plan_layout.cc` (500), and tests `flat_rack_test.cc` (173), `layout_plan_test.cc` (170), `layout_rows_test.cc` (445), `layout_scene_test.cc` (220), `plan_replay_test.cc` (822).
- removed (`--diff-filter=D`): `src/print_layout_moves.cc` (644).
- renamed (`git diff -M --summary --diff-filter=R 68a1199 940a01b -- src tests`): none.
- modified (`--diff-filter=M`): 13 files, +3,978 -1,092: `arm_catalog.h`, `bay_model.{cc,h}`, `check_layout.cc`, `layout.{cc,h}`, `layout_check.{cc,h}`, `layout_model.{cc,h}`, `view_layout.cc`, `tests/layout_check_test.cc`, `tests/layout_model_test.cc`. No stand-in file changed (the 13 are all new-cell files).
- whole commit: `git diff --shortstat 68a1199 940a01b` = 50 files, 19,074 insertions, 1,785 deletions.

## 2. Namespaces

Command: `grep -n '^ *namespace\b' src/*.h | grep -v ':namespace cell {$'` (11 lines) and `grep -c '^namespace cell {' src/*.h | grep -c ':1'` (31).

| namespace opened by a header | headers |
|---|---|
| `cell` only | 31 of 42 |
| `cell::layout` only (no plain `cell`) | 7: `arm_catalog.h:27`, `layout.h:98`, `layout_check.h:291`, `layout_model.h:74`, `layout_plan.h:143`, `layout_rows.h:71`, `layout_scene.h:138` |
| `cell::bay` | 2: `bay_hardware.h:87`, `bay_model.h:58` |
| `cell::container` | 1: `container.h:42` |
| `cell::flat_rack` | 1: `flat_rack.h:50` |
| anonymous namespace in a header | 0 (`grep -c '^namespace {' src/*.h` all 0) |
| `using namespace` in `src/` | 5, all in `.cc` of programs: `print_bay.cc:60,201`, `plan_layout.cc:50`, `view_layout.cc:103,291` |

So Context's "every header is in `cell`" is true as a prefix; the 11 new-cell headers are in `cell::<sub>` and never in plain `cell`. `.cc` files: `grep -n '^ *namespace\b' src/*.cc | awk -F: '{print $3}' | sed 's/{.*//' | sort | uniq -c` gives 60 anonymous, 29 `namespace cell`, 8 `namespace cell::...`, and two aliases (`ob = ompl::base`, `og = ompl::geometric`).

## 3. The two families and what joins them

Method: a Python scan of every `#include` line in `src/` and `tests/` (table in `out/audit/inc.json`); families by file name as the brief lists them. `grep -c` equivalent for one header: `grep -l '#include "src/mj_handles.h"' src/* tests/*`.

Headers that include no other project header (16 of 42): `arm_catalog.h`, `bay_hardware.h`, `camera_model.h`, `check_placement.h`, `container.h`, `flat_rack.h`, `layout.h`, `mj_handles.h`, `pallet.h`, `plane_fit.h`, `portable_random.h`, `profile.h`, `rectangle_fit.h`, `rrt.h`, `suction.h`, `viewer_lib.h`.

Most included project headers (files that include them; `.cc` and `.h` together):

| header | `src/` files | `tests/` files |
|---|---|---|
| `mj_handles.h` | 17 | 19 |
| `arm.h` | 13 | 12 |
| `cell.h` | 14 | 11 |
| `pallet.h` | 12 | 10 |
| `camera_model.h` | 16 | 5 |
| `depth_camera.h` | 14 | 4 |
| `layout.h` | 9 | 6 |
| `suction.h` | 10 | 4 |
| `layout_model.h` | 8 | 6 |
| `rrt.h` | 11 | 3 |
| `pick_place.h` | 7 | 7 |
| `joint_move.h` | 8 | 5 |
| (13th, 14th) `bay_hardware.h` | 6 | 7 |
| `conveyor.h` | 5 | 8 |

Context said `mj_handles` is included from 32 files; at 940a01b it is 36 (17 + 19).

Outside libraries (include lines of `src/` headers, `src/` `.cc`, `tests/`): `mujoco/mujoco.h` in 17 headers + 24 `.cc` = 41 of 99 `src/` files (Context: 39 of 91, 15 headers) and 29 test files (0 headers); `Eigen/` in 64 `src/` files and 31 tests; `ompl/` in `src/ompl_search.cc` only (headers: none; `ompl_search.h` hides it); `nlohmann/` in `src/layout.cc`, `src/layout_replay.cc`, `tests/layout_model_test.cc`; `GLFW/` in one `src/` file; `gtest/` in 41 tests. (Python scan; `grep -l '#include <GLFW' src/*` names the file.)

Includes from new-cell code to stand-in code (direct, by the brief's lists): exactly one, `src/layout_plan.h:137` -> `src/joint_move.h`. By transitive reach through headers (`src/joint_move.h:20` includes `src/profile.h`): `layout_plan.cc`, `layout_replay.cc`, `plan_layout.cc`, `view_layout.cc` reach `joint_move.h` and `profile.h`; `layout_replay.cc:21-22` also uses `path_timing.h` and `rrt.h`, `layout_plan.cc:26` `rrt.h`. `layout_check.cc`, `layout_scene.cc`, `layout_model.cc`, `layout.cc` reach none of them. From stand-in to new: none (no stand-in file includes a new-cell header). Includes of `portable_random.h` (`src/layout_check.cc:32`, `src/layout_plan.cc:25`) and `mj_handles.h` (`layout_*`: `layout_check.cc:31`, `layout_model.h:72`, `layout_rows.cc:24`, `layout_scene.h:136`, `view_layout.cc:68`) are the shared neutral headers.

Tests that mix families: only `tests/locating_gate_test.cc` (`bay_hardware.h` and `trial_runner.h:13`).

Planner headers (`grep -n '#include' file`):
- `src/collision.h:115-120`: `Eigen/Core`, `Eigen/Geometry`, `mujoco/mujoco.h`, `src/arm.h`, `src/mj_handles.h`.
- `src/motion_planner.h:137-145`: Eigen, `mujoco/mujoco.h`, `src/collision.h`, `src/joint_move.h`, `src/ompl_search.h`, `src/path_timing.h`, `src/rrt.h`.
- `src/rrt.h:66-71`: only std headers and `Eigen/Core` (no project header; confirmed).
- `src/path_timing.h:128-134`: `Eigen/Core`, `src/joint_move.h`, `src/profile.h`.
- `src/ompl_search.h:117-125`: std, `Eigen/Core`, `src/rrt.h` (no `ompl/` header).

No cycle among headers was found by the scan (not a formal check).

## 4. Things written twice

(a) Inverse kinematics.
- `SolveIk` `src/ik.cc:133` (declared `src/ik.h:162`; `SolveIkPath` `ik.cc:248`): takes `cell::Arm` (`src/arm.h`), a full 6-D target `Isometry3d`, a seed, `IkOptions`; damped least squares with step limit and joint clamping (`ik.cc:166-200`), reports `IkStatus` {converged, stalled, iteration limit, bad input} through `IkReport`; no track; `max_distance_from_seed` option; `SolveIkPath` keeps one branch along a path. Callers: `src/pick_place.cc` and `src/check_placement.cc:302,716,720`.
- `Checker::Solve` `src/layout_check.cc:299` (declared :220): works on a layout's `SceneArm` (joint limits `a.lo/a.hi`), sets the arm's carriage position `track` first (`:307`), solves five or six dimensions (a target with `has_x` false constrains only the tool axis `z`, `:333-345`, so the tool may spin about its axis), weights orientation by `kOrientWeight`, adaptive damping (`lambda` halves on success, x5 on failure), 300 iterations, stall test, returns `perr/aerr` in an `IkResult`. Calls: `layout_check.cc:536,678,863,2084,2119`. Neither handles the other's case: `SolveIk` cannot take a track or an axis-only target; `Solve` has no seed-distance limit, no status, no path version, and takes no `cell::Arm`.
- Third solver: none (grep above). Same damped-least-squares normal equations `J^T (J J^T + l^2 I)^-1 e`, `ik.cc:178-184` and `layout_check.cc:381-383`.

(b) Collision and distance.
- `src/collision.h` (class `CollisionChecker`, `collision.cc`): one arm and its held module, contact bits and, for clearance, `mj_geomDistance` over precomputed pairs (`collision.cc:526`), bounding-sphere pre-test `:515-523`. The layout code's single function is `LayoutScene::LeastDistance` (`src/layout_scene.cc:531`, declared `layout_scene.h:294`), pairs by design and bounding boxes first (`:555-560`). Two distance functions, with different pair sets and conventions.
- Every call (`grep -n 'mj_geomDistance' src/* tests/*`): `src/collision.cc:526`; `src/cycle_check.cc:306`; `src/check_placement.cc:490`; `src/layout_scene.cc:575` and `:586`; tests `tests/collision_test.cc:642`, `tests/layout_model_test.cc:644`, `tests/cell_test.cc:149,606`, `tests/camera_cycle_test.cc:149,435`, `tests/motion_planner_test.cc:601`, `tests/pick_place_test.cc:150,350`. Comments that name it: `layout_model.h:31`, `layout_scene.h:21`, `layout_check.h:41`, `collision.h:52,154`, `cycle_check.cc:22,304`, `cycle_check.h:24`.
- The workaround: `src/layout_model.cc:717-731` (`LoadLayoutModel`): every capsule with contact bits is rewritten as a cylinder of the same radius, half-length + radius (`:727-731`). Rationale `:719-723`: radius 0.05 capsule through a 5 mm plate measured +0.1975 m against -0.2525 m for the cylinder. Calls that go through it: `layout_scene.cc:575,586` (the scene loads `LoadLayoutModel`'s model, `layout_scene.h:11-13`) and `tests/layout_model_test.cc:644`. Calls that do NOT: `collision.cc:526`, `cycle_check.cc:306`, `check_placement.cc:490` and the stand-in tests, which load `models/cell.xml` (includes Menagerie's UR10e capsules; `grep -c capsule models/*.xml` is non-zero only for `suction_scene.xml`, the capsules are in the fetched `@menagerie_ur10e`). Whether those calls are wrong in practice is not measured here.

(c) `src/print_layout_moves.cc`: gone (section 0 item 2). The moving and obstacle sets live in `LayoutScene` (`layout_scene.cc:178` `Moving`, `:221` `Obstacles`, `:483` `WayMoving`, `:497` `WayObstacles`; enums `layout_scene.h:146-160`). Users: `layout_check.cc:451,456,829,1158`, `layout_replay.cc:735-736` (keeps per-arm copies of the lists for speed, built from the scene, `:775-776`). Small second list: `layout_check.cc:1145-1163` builds a probe set of its own (`MovingGeom{g, kProbe}`, `link_geoms`, `carriage_geoms` as obstacles) for the tool-in-free-space row: not a copy of `Moving`, but a hand-written set beside it.

(d) Hardware literals.
- Stand-in module 2.3 x 1.1 x 0.035 m: `src/suction.h:45` (`module_size{2.3, 1.1, 0.035}`), `src/pallet.h:27-29` (`kModuleLength/Width/Thickness`), `src/conveyor.h:69-71` (same three constants again, class members), `src/point_generator.h:64-66`; in prose `src/rectangle_fit.h:5`, `src/module_finder.h:39`, `src/held_module_finder.h:51`, `src/rrt.h:4`, `src/cell.cc:29`; in models `models/pallet.xml:77`, `models/check_scene.xml:51`, `models/conveyor_scene.xml:20,25,30`, `models/suction_scene.xml:40`. Command: `grep -n '2\.3\b\|1\.1\b\|0\.035' src/*.h src/*.cc models/*.xml`. So four code copies of the three numbers, none derived from another.
- Real module (`src/bay_hardware.h:103-106`: 2.382 x 1.134 x 0.030 m, 32.4 kg; tube radius 0.062 `:90`; rail pitch from `kModulePitch :111`): `grep -n '2\.382\|1\.134\|1\.144\|0\.062\|0\.0625\|10\.516\|32\.4' src/*.cc src/*.h tests/*.cc | grep -v '^src/bay_hardware.h'` finds only comments (`src/layout_check.h:199` "32.4 kg", `tests/locate_physics_test.cc:11,15,73`, `tests/check_cycle_test.cc:119`), no constant in code. Literals of those numbers also sit in the generated `models/bay.xml` (by design, `tests/bay_model_test.cc:97` checks it byte for byte) and in `layouts/below_rails_by_module_arm.json` (rail offsets 1.144/-1.144). Verdict: the real hardware has one code home; the stand-in module has four.
- `kPi` as a local constant in 8 `.cc` (`grep -c 'kPi = ' src/*.cc | grep -v ':0'`: `camera_model.cc`, `check_placement.cc`, `layout_model.cc:75`, `pick_place.cc:50`, `depth_camera.cc:20`, `layout_plan.cc:32`, `layout_replay.cc:28`, `score.cc:17`); the digits `3.14159265358979323846` appear in `bay_hardware.h:233,503,712`, `print_bay.cc:77,136,203,212`, `print_cell.cc:31`, `view_cell.cc:298,339,340,444` and `M_PI` in `print_pallet.cc:139`. `std::numbers::pi` is used in `check_placement.cc:40`, `pick_place.cc:50`, `point_generator.cc:11`.

(e) Argument parsing. No shared helper except the viewer's: `ParseViewerArgs` (`src/viewer_lib.cc:214`, used by `viewer.cc:65` and `view_layout.cc:354` only). Each of the 20 `main`s hand-rolls its loop (`grep -ln '^int main' src/*.cc`): 8 loops of the form `std::strcmp(argv[i], "--x") == 0 && has_value` (`run_cell.cc:439`, `run_gate.cc:63`, `print_depth.cc:110`, `print_check.cc:158`, `print_perception.cc:143`, `view_cell.cc:116`, `print_profile.cc:30`, `print_plane_fit.cc:29`), 3 loops of `const std::string a = argv[i]; a == "--x"` with a `value` lambda (`print_plan.cc:167` (exits with 2 on a missing value), `plan_layout.cc:253`, `check_layout.cc:149`), the rest positional `argc`/`argv[n]` with `atoi`/`atof` (`print_cell.cc`, `print_pallet.cc`, `print_points.cc`, `step_arm.cc`, `print_bay.cc`, `print_rrt.cc`). Numbers: `grep -l '"--pretty"' src/*.cc | wc -l` = 8 copies of `--pretty` (`run_cell`, `print_plan`, `print_check`, `print_perception`, `print_profile`, `print_plane_fit`, `plan_layout`, `check_layout`); `--seed` in 4 (`run_cell`, `run_gate`, `print_depth`, `view_cell`; `--first-seed` in 3: `print_plan`, `print_check`, `print_perception`). Errors differ (return 1, return 2, `std::exit(2)`). `FromWorkingDirectory` (turns a path relative to `BUILD_WORKING_DIRECTORY`) is written twice: `run_cell.cc:407`, `run_gate.cc:40`. `LayoutDir()` (`LAYOUT_DIR` or `layouts`) twice: `check_layout.cc:37`, `plan_layout.cc:52`; the same getenv again in `view_layout.cc:107,360`; `PLAN_DIR` in `plan_layout.cc:133`, `view_layout.cc:104`.

(f) Table printing. No shared helper. CSV and pretty are chosen by `if (pretty)` inside each program with printf formats (`print_check.cc:221`, `print_perception.cc:199`, `print_plan.cc:388,416`, `print_profile.cc:66`, `print_plane_fit.cc:52`, `run_cell.cc:147` `PrintHeader` and `:302` `PrintRow`). The two new programs each have a `PrintTable(rows, pretty)`: `check_layout.cc:61` and `plan_layout.cc:86`, doing the same job with differently written bodies (width padding vs. cell resize; `diff` of the two bodies differs in 8 lines). Also `Csv` (`check_layout.cc:43`, `plan_layout.cc:57`), `Num` four times with three behaviours (`check_layout.cc:53` "%.4f"; `bay_model.cc:46` and `layout_model.cc:81` "%.12g", identical; `plan_layout.cc:67` "%.*f"), `PrintMargins` (`check_layout.cc:128`, `plan_layout.cc:130`: 8 vs 110 lines, same name, different bodies).

(g) JSON. Layouts read: `LoadLayout` `src/layout.cc:807` (nlohmann, `layout.cc:28`). Plans written and read: `PlanToJson` `src/layout_replay.cc:321` and `PlanFromJson` `:359` (declared in `src/layout_plan.h:311-312`; implemented in a file whose name does not match the header), with `Q()` at `layout_replay.cc:207`. Nothing else uses nlohmann except `tests/layout_model_test.cc`. File reading is written five times: `layout.cc:812`, `layout_model.cc:112` (`ReadFile`, whole-file), `layout_replay.cc:507`, `plan_layout.cc:137`, `view_layout.cc:110`; `trial_runner.cc:612,624` has its own reader/writer for failure files.

(h) Same-name helpers in two or more files (a script over function definitions at column 0 in `src/*.cc` and `tests/*.cc`; body checked by eye where marked `=`):
- `SecondsSince(Clock::time_point)` x5 `=`: `held_module_finder.cc:31`, `module_finder.cc:32`, `motion_planner.cc:33`, `ompl_search.cc:43`, `rrt.cc:15`.
- `Fmt(const char*, ...)` x4, vsnprintf into a string `=`: `layout_check.cc:51`, `layout_plan.cc:38`, `layout_rows.cc:39`, `layout_replay.cc:194`.
- `Num` x4 (above), `Csv` x2, `PrintTable` x2, `PrintMargins` x2, `FromWorkingDirectory` x2 `=`, `LayoutDir` x2 `=`.
- `ExactExtent` (32 lines, near-identical, a comment differs) `layout_check.cc:68`, `layout_rows.cc:73`; geom pose reading from `geom_xpos/xmat` three times: `layout_check.cc:70-72`, `layout_scene.cc:40-42`, `layout_rows.cc:75-77`.
- `InBox` x3: `rrt.cc:218`, `ompl_search.cc:272` `=`, `module_finder.cc:200` (different). `ToConfig` x2 (different signatures): `layout_plan.cc:47`, `layout_replay.cc:36`.
- `Advance` `=` `path_timing.cc:65`, `profile.cc:12`; `AllPositiveAndFinite` `=` `joint_move.cc:10`, `path_timing.cc:60`; `SideSigma` `=` `held_module_finder.cc:69`, `module_finder.cc:141`; `WritePgm` `print_check.cc:128`, `print_depth.cc:60`; `AngleBetween` x4 (`held_module_finder.cc:53`, `plane_fit.cc:91`, `print_pallet.cc:32`, `tests/suction_test.cc:86`); `Median` x4 (`module_finder.cc:51`, `print_plan.cc:131`, `tests/motion_planner_test.cc:91` `=`, `tests/rrt_test.cc:119`); `Collides` x2 different (`collision.cc:39`, `cycle_check.cc:24`).
- `Degrees`: `check_placement.cc:65` (vector of six degrees to radians) and `point_generator.cc:11` (one angle): different. `Deg`, `Rad`, `Norm`, `Usage` as function names: none in 2+ files (`Usage` only `plan_layout.cc:77`, `ViewerArgsUsage` `viewer_lib.cc:327`; `check_layout.cc:139` `kUsage`; the rest print their own `usage:` string, 12 files).
- Tests: `BodyDistance`, `RunScript`, `RunUntilConveyorStops` `=` in `tests/camera_cycle_test.cc` and `tests/pick_place_test.cc` (`:140/:140`, `:95/:86`, `:107/:99`); `Angle`, `Distance`, `PlanningMargin`, `PlanningSelfMargin`, `HistoryFor`, `Apply` in `tests/check_cycle_test.cc` and `tests/planned_carry_test.cc`; `DistanceToSegment` x2 `=` (`arm_follower_test.cc:157`, `path_follow_test.cc:134`); `Scaled` (`arm_follower_test.cc:139`, `path_timing_test.cc:61`); `WithNoise` `=` (`held_module_finder_test.cc:121`, `module_finder_test.cc:133`); `Named` `=` (`layout_check_test.cc:516`, `layout_scene_test.cc:32`); `Load` (`layout_plan_test.cc:30`, `layout_rows_test.cc:29`).

## 5. Sensor data only

`grep -n 'True[A-Z][A-Za-z]*(' src/*` finds 10 files (same count as Context):

| file | role | lines |
|---|---|---|
| `src/cell.h`, `src/cell.cc` | defines `TrueModulePose`, `TrueModuleSpeedOnTube`, `TruePlacedSlot` (the stand-in's cell, which also hosts control's accessors) | `cell.h:312,317,321`; `cell.cc:341,387,391,413-414` |
| `src/conveyor.h`, `src/conveyor.cc` | defines `TrueSlidePosition`, `TrueTubePose`, `TrueSlotPose` | `conveyor.h:177,180,184`; `conveyor.cc:192,196,208` |
| `src/score.cc` | scorer | `:24,25,45,53,64` |
| `src/trial_runner.cc:179` | harness: finder error vs truth (not control; it runs the script and compares) | |
| `src/print_cell.cc:162,163,171`, `print_perception.cc:263`, `print_plan.cc:287` | programs that print | |
| `src/view_cell.cc:387` | viewer (caption) | |

No `src/` file outside these calls a `True...` accessor; no scene generator calls them (the "scene generator" is `Cell::Reset`/`Randomize`, which writes truth and does not read it through `True...`). 11 test files call them (`grep -c 'True[A-Z][A-Za-z]*(' tests/* | grep -v ':0' | wc -l`).

Other truth reads: counted by this regex over `src/*`: `d(ata)?(->|\.)(xpos|xquat|xmat|site_xpos|site_xmat|geom_xpos|geom_xmat|qpos|qvel|sensordata|contact|ncon|cvel|xipos|subtree_com|efc_force|qfrc_*|actuator_force)` or the bare field followed by `[`: 21 files with matches, counts: `collision.cc` 32, `layout_check.cc` 18, `cell.cc` 13, `pallet.cc` 11, `cycle_check.cc` 8, `check_rig.cc` 8, `conveyor.cc` 7, `suction.cc` 6, `layout_scene.cc` 6, `print_pallet.cc` 5, `arm.cc` 5, `print_cell.cc` 4, `check_placement.cc` 4, `layout_rows.cc` 3, `layout_replay.cc` 3, `layout_model.cc` 2, and 1 each in `suction.h`, `view_layout.cc`, `step_arm.cc`, `ik.cc`, `arm_follower.cc`. Also `camera_model.cc:49,53` reads `cam_xpos/cam_xmat` (not in the regex) and `pallet.cc:140-151` `PalletPose`/`ModulePose` return true body poses.

What each is (read in context):
- Legitimate sensor reading (joint angles): `arm.cc:76` `JointAngles` (`qpos[arm.qpos[j]]`), `arm.cc:82` velocities, `step_arm.cc:45`, `cycle_check.cc:211` (angle), `conveyor.cc:185,189,193` (`sensordata` encoder position and velocity, `SensorPosition`/`SensorVelocity`), `arm.cc:91-93` tip pose is forward kinematics of the arm's own joints.
- Writes, not reads: `cell.cc:69-81,237,322`, `pallet.cc:122-132`, `check_rig.cc:153-158`, `check_placement.cc:403,407,598,606` (placing bodies/joints in a scratch `mjData`), `ik.cc:38`, `layout_scene.cc:150-151`, `collision.cc:203,218,242,401,422-424`, `layout_check.cc:307,318`.
- Scratch kinematics, not simulation truth: reads of `site_xpos`/`geom_xpos` after the code set `qpos` itself in a private `mjData` (`collision.cc:413-416`, `layout_check.cc:70-72,321-324,808-812,1493`, `layout_scene.cc:40-42`, `layout_rows.cc:75-77`, `layout_replay.cc:976-979`). The layout code has no simulation at all; all of it is design-time.
- Truth of bodies: `suction.cc:19-35` (module body `xpos/xquat` for the weld: the suction cup's physics, plant-side), `pallet.cc:139-151` (`PalletPose`, `ModulePose`: called by `cell.cc:388` (`TrueModulePose`), `print_cell.cc:150`, `print_pallet.cc:77-156`), `conveyor.cc:201-204` (`TrueTubePose`), `cycle_check.cc:180-181,203,220-221,297-298` (contacts, motor force, finite checks, geom distance: the check on a run, a scorer in the brief's sense), `print_pallet.cc:118-132`, `print_cell.cc:42-51` (print programs), `camera_model.cc:49-53` (`CameraModelFromMujoco` reads the camera's true pose from `mjData`; the calibrated model is made from it elsewhere, so this is the one place a "calibration" starts from truth; flag for the lead).
- `arm_follower.cc:204` reads `qfrc_bias` (the plant's gravity and Coriolis torque for feed-forward: model data, not a pose).
- `pick_place.cc` (control code) has no raw field read; it uses `cell_.data()->time`, `JointAngles`, `conveyor.SensorPosition` and `conveyor.Moving(*cell_.data())` (`pick_place.cc:739-742,771`; `Moving` is `conveyor.cc`'s, not read here). `motion_planner.cc`, `module_finder.cc`, `held_module_finder.cc`, `depth_camera.cc` have none.

Control code in the rule's sense (files that must not call `True...` or read poses): `pick_place.*`, `motion_planner.*`, `collision.*` (its reads are scratch writes), `rrt.*`, `ompl_search.*`, `path_timing.*`, `joint_move.*`, `profile.*`, `arm_follower.*`, `ik.*`, `arm.*`, `module_finder.*`, `held_module_finder.*`, `rectangle_fit.*`, `plane_fit.*`, `depth_camera.*` (renders a depth image; sensor), `camera_model.*`, `suction.*` (plant-side, borderline), `check_placement.*`, `trial_runner.*` (harness, owns the truth comparisons).

Proposed smallest honest rule for a command: grep `\bTrue[A-Z][A-Za-z]*\(` under `src/`, fail if the file is not on this allowlist: `src/cell.h src/cell.cc src/conveyor.h src/conveyor.cc src/score.cc src/trial_runner.cc src/view_cell.cc src/print_*.cc` (plus anything under `tests/`). Today's offenders: none. It misses: (1) truth reached without a `True` name: `ModulePose`/`PalletPose` (`pallet.h:69,73`) and raw `data->xpos|xquat|site_xpos|geom_xpos|cam_xpos` reads, the first of which a control file could call tomorrow; (2) accessors added later without the prefix; (3) reaching truth through `Cell::data()` (every `.cc` that includes `cell.h` can read any `mjData` field). A second command closes (1) and part of (3): grep the raw-field regex above over the control list and allow only `arm.cc` (joint angles and tip), `ik.cc` and `collision.cc` (scratch data), `suction.cc`, `arm_follower.cc` (`qfrc_bias`). It still cannot tell scratch data from the simulation's data by name; that distinction is a comment today, not checkable.

## 6. Tests and what they reach

Command: `bazel query "kind('source file', deps(tests(//...)))" --output=label | grep '^//:src/'` gives 75 of 99 `src/` files. The 24 reached by no test: 
- the 19 mains: `check_layout.cc`, `gen_bay.cc`, `plan_layout.cc`, `print_bay.cc`, `print_cell.cc`, `print_check.cc`, `print_depth.cc`, `print_pallet.cc`, `print_perception.cc`, `print_plan.cc`, `print_plane_fit.cc`, `print_points.cc`, `print_profile.cc`, `print_rrt.cc`, `run_cell.cc`, `run_gate.cc`, `step_arm.cc`, `view_cell.cc`, `view_layout.cc`, plus `viewer.cc`;
- `viewer_lib.cc`, `viewer_lib.h` (only binaries use them);
- `check_placement.cc`, `check_placement.h`: belong to no library, only to `print_check` (`BUILD.bazel:1357-1358`); `src/pick_place.h:420` only mentions it in a comment. This is stand-in code (the check camera's placement study) that no test compiles.

Binaries run by a test or script: none. `bazel query 'kind(cc_binary, deps(tests(//...))) intersect //:*'` prints nothing; no test has a binary in `data` (`grep -n 'data = \[' -A4 BUILD.bazel` shows only models/layouts/plans files) and `grep -n 'popen\|system(' tests/*.cc` is empty. Tests mention binaries only in failure messages (`tests/bay_model_test.cc:114`: "bazel run //:gen_bay > models/bay.xml"). CI (`.github/workflows/ci.yml:67-78`) runs only `bazel test`. So no program's output (including `--pretty`) is compared with anything; `gen_bay` and `plan_layout` are indirectly checked through the library they call, `run_cell` through `trial_runner`.

Headers with no `tests/<name>_test.cc` (`for h in src/*.h; do [ -f tests/$(basename $h .h)_test.cc ] || echo $h; done`): `arm_catalog`, `camera_model`, `check_placement`, `check_rig`, `cycle_check`, `layout`, `mj_handles`, `portable_random`, `rrt_worlds`, `score`, `viewer_lib` (11). Several are covered by other tests: `layout` by `layout_check_test`, `layout_model_test` etc., `score` and `cycle_check` by `check_cycle_test`/`gate_test`. Tests without a same-name header: `camera_cycle`, `check_camera_cell`, `check_cycle`, `gate`, `locate_physics`, `locating_gate`, `path_follow`, `plan_replay`, `planned_carry`, `step`.

Tests that include headers from both families: one, `tests/locating_gate_test.cc` (section 3).

Test targets with attributes (query `--output=streamed_jsonproto`, explicitly set attributes only; 42 targets):

| size | targets |
|---|---|
| small (22) | arm_follower, arm, bay_hardware, bay_model, container, flat_rack, ik, joint_move, layout_check, layout_model, layout_rows, layout_scene, locating_gate, path_follow, path_timing, plane_fit, point_generator, profile, rectangle_fit, rrt, step, suction |
| medium (16) | cell, check_camera_cell, check_cycle_batch, check_cycle, collision, conveyor, depth_camera, held_module_finder, layout_plan, locate_physics, ompl_search, pallet, pick_place, plan_replay, planned_carry, trial_runner |
| large (4) | camera_cycle, gate (`shard_count = 4`, `tags = ["gate"]`), module_finder, motion_planner (`shard_count = 4`) |

(The full table with every target is in `out/audit/tests_attrs.txt`.) No target sets `timeout`, `flaky` or `local`; `tags` is set only on `gate_test`. `check_cycle_test` and `check_cycle_batch_test` come from one comprehension with `args = ["--gtest_filter=..."]` (`BUILD.bazel:1469-1472`). `AGENTS.md` times are in `AGENTS.md:19-24`.

Tests that compare with committed files:
- `tests/bay_model_test.cc:97` `CommittedFileIsWhatTheGeneratorPrints`: `models/bay.xml` against `BayModelXml()`: byte equality (`committed == generated`), reports first differing line.
- `tests/plan_replay_test.cc:474-686` (`Committed.*`): the committed `plans/flat_rack_3_pin_arms.json` (found by `PlanDir()`, `:36`) is loaded, verified without planning (`VerifyPlan`, `:477`) against the layout file's SHA-256 (`:446`), `JsonRoundTripIsByteIdentical` (`:502-504`: `PlanToJson(plan) == committed text`, byte equality), and tamper/breakage tests. Tolerances only inside the verifier (`pose_tolerance_deg = 0.2`, `src/layout_plan.h:392`).
- Tests that read `layouts/*.json` and check numbers computed from them (not file equality): `layout_check_test.cc` (lines 40-1143), `layout_rows_test.cc:32`, `layout_scene_test.cc:35`, `layout_plan_test.cc:27`, `layout_model_test.cc:53`.
- `models/*.xml` are loaded by all model tests through `MODEL_PATH` env; only `bay.xml` is compared byte for byte. No golden data of other kinds.

## 7. Headers' opening comments and file sizes

Counted: lines starting `//` before the first line that is neither blank nor `//` (script over `src/*.h`; full per-header table in the working output). Minimum 6, median 39, maximum 275. Headers with fewer than 5: none. Fewest: `mj_handles.h` 6, `arm.h` 9, `point_generator.h` 9, `joint_move.h` 11, `check_rig.h` 12, `pallet.h` 12, `score.h` 12, `arm_catalog.h` 18, `suction.h` 19. Most: `layout_check.h` 275, `pick_place.h` 240, `motion_planner.h` 124, `path_timing.h` 123, `layout_plan.h` 121, `layout_scene.h` 119, `ompl_search.h` 112, `collision.h` 105.
`.cc` over 1,000 lines (`wc -l src/*.cc | awk '$1>1000'`): `layout_check.cc` 2,610; `pick_place.cc` 1,911; `layout_replay.cc` 1,812; `layout_plan.cc` 1,462; `rectangle_fit.cc` 1,252 (5; Context: 3). Headers over 600: `pick_place.h` 1,173; `bay_hardware.h` 976 (2). Tests over 1,000: `check_cycle_test` 1,654, `layout_model_test` 1,247, `rectangle_fit_test` 1,235, `layout_check_test` 1,185, `ik_test` 1,141, `planned_carry_test` 1,082, `collision_test` 1,016, `camera_cycle_test` 1,004, `motion_planner_test` 1,004.

## 8. Determinism and platform

Command: `grep -nE 'std::rand|\bsrand|random_device|mt19937|<random>|_distribution|std::chrono|\btime\(|getenv|std::thread|std::async|omp |hardware_concurrency' src/* tests/*`.
- `std::rand`, `srand`, `random_device`, `std::async`, OpenMP, `time(`: none. Clocks: only `std::chrono::steady_clock`.
- `std::mt19937` with a seed given by the caller: `rrt.cc:441,514`, `plane_fit.cc:132`, `rectangle_fit.cc:860`, `depth_camera.cc:50,172`, `cell.cc:187`, `check_rig.cc:57`, `conveyor.cc:59/h:190`, `layout_plan.cc:570` (one draw per move, `UniformIndex`), `layout_check.cc:279` (fixed 20261124), `print_points.cc:30`, `print_plane_fit.cc:67`, `point_generator.cc` (passed in). Tests seed their own: fixed seeds everywhere (`grep -c 'mt19937' tests/*`).
- Draws that go through `src/portable_random.h` (`UniformUnit`, `UniformReal`, `UniformIndex`, the normal): `layout_check.cc:288`, `layout_plan.cc:572`, `rectangle_fit.cc`, `depth_camera.cc` (own Box-Muller, `:35`), `cell.cc:44-47` (own, says std's distribution is avoided).
- Draws that go through a `std::` distribution, whose algorithm the standard leaves to the library (libstdc++ vs libc++ give different numbers; the same library on ARM and x86-64 gives the same ones as the arithmetic is plain double): `src/rrt.cc:442` (`uniform_real_distribution<double> unit`, used `:460,462` for samples and the goal bias) and `:522-524` (shortening, `along`); `src/plane_fit.cc:133` (`uniform_int_distribution`); `src/point_generator.cc:21-22,37,52,60-63` (uniform and normal); `src/conveyor.cc:123` (`normal_distribution`, the stop error). This is what "samples via std" means: RRT-Connect's samples and shortening in `rrt.cc` are portable only for one standard library. Committed plans depend on it: `layout_plan.cc:26` includes `rrt.h` and plans with `ro.max_seconds = 1e12` (`layout_plan.cc:644,683`), which also removes the wall-clock stop (`rrt.cc:451,517`) for the layout planner. Tests drawing through std distributions (not portable draws): `ik_test.cc:189,195,952`, `collision_test.cc:206...`, `rrt_test.cc:209`, `module_finder_test.cc:374`, `cell_test.cc:593`.
- Wall clock that can change an output: `rrt.cc:451,517` (`max_seconds`, default 5 s and 1 s, `rrt.h:240,348`: a slow machine ends the search or the shortening earlier); `motion_planner.cc:334,373` (`time_budget` split across tries); `ompl_search.cc:417` (`options.max_seconds`); OMPL is seeded by `SeededSampler` (`ompl_search.cc:83-111`) from the file's own seed, and `ompl_search.cc:48-51` has a table of which planners repeat from their seed; the brief's clock-seeded ones (`BITstar`, `AORRTC`, `InformedRRTstar`) are the ones that do not repeat (not re-verified here). Reports of seconds only (`pick_place.cc:756-953,1544`, `module_finder.cc:825-982`, `held_module_finder.cc`, `check_placement.cc:634`, `check_rig.cc:185`, `trial_runner.cc:107,235`, `run_cell.cc:573,614`, `print_*.cc`, `run_gate.cc:102,143`) enter printed columns of time; they change the output of those programs byte for byte (so such columns cannot be compared) but not any decision.
- Threads: `src/layout_plan.cc:21,722-724` (a pool plans moves in parallel; each move has its own seed, `:570-572`, and results go "into the blocks, in order", `:726`; thread count `plan_layout --threads`; the result is meant not to depend on it, not verified here); `src/trial_runner.cc:14,276-278` (trials in parallel, `run_gate.cc:60` default `hardware_concurrency()`); tests: `gate_test.cc:87`, `ompl_search_test.cc:289-293`.
- `getenv`: `LAYOUT_DIR`, `PLAN_DIR`, `MODELS_DIR`, `UR10E_XML` (`layout_model.cc:447-448`), `BUILD_WORKING_DIRECTORY`, and `PLAN_DEBUG`, `PLAN_SKIP_VERIFY` (`layout_plan.cc:583,659,1334`: a variable that skips the verifier, an input to a program's output); tests read `MODEL_PATH` and Bazel's `TEST_*`.

## 9. Units and frames

- Suffix count: `grep -ohE '\b[a-zA-Z_0-9]*(_deg|_mm)\b' src/*.h src/*.cc | wc -l` = 450 uses of 160 distinct identifiers, in 25 files (top: `print_bay.cc` 73, `run_cell.cc` 56, `trial_runner.cc` 52, `layout_check.cc` 47, `print_check.cc` 32; `grep -oE ... | awk -F: '{print $1}' | sort | uniq -c`). In headers: `layout.h` (`base_rpy_deg:195`, `wait_joints_deg:203`, `stow_joints_deg:205`, `yaw_deg:224`, `frame_yaw_deg:296`, `joint_margin_deg:128`, `holds_within_deg:151`), `layout_check.h` (`work_deg:361`, `approach_leg_deg:372`, `joint_margin_deg:328,409`), `layout_plan.h` (`pose_tolerance_deg:392`, `margin_deg:414`), `arm_catalog.h:44-45,74-75` (`joint_lo_deg`), `layout_model.h:93` (`joints_deg`), `layout_scene.h:248,309`, `trial_runner.h:70-81` (`xy_mm`, `yaw_deg`, `calibration_mm/_deg`), `bay_hardware.h:728` (`direction_deg`). So degrees and millimetres are an I/O convention of the layout JSON and of the programs' flags, not a violation inside the stand-in's code, but `AGENTS.md` says metres and radians: the layout structs carry degrees all the way into the checker (`layout_check.cc:112` `ToDeg`).
- Metre/second suffixes: `_m` 21 distinct (187 uses), `_s` 32 (275), `_seconds` 14 (111) (`grep -ohE '\b[a-zA-Z_0-9]*_s\b' src/*.h src/*.cc | sort -u | wc -l` etc.); `_rad` 0, `_mps` 0. So seconds and metres are sometimes marked, radians never (radians are the unmarked default).
- Frames: marked only by comments and by names (`world`, `tool`, `tip`, `slot`); `AGENTS.md:84-91` "Frames and units" has the frame list; `docs/layout.md`, `docs/bay-hardware.md` define the bay's frames; no type distinguishes frames (all `Eigen::Isometry3d`/`Vector3d`). `Config = std::array<double, 7>` (`layout_plan.h:146`, comment "Six joints (rad), then the track's value (m)").
- `path_timing.h` claim confirmed: `PathTimingOptions::max_deviation` is documented in radians as the Euclidean distance in joint space (`src/path_timing.h:138-141`), "Units are radians and seconds" (`:123`); the layout planner passes seven-dimensional waypoints (six radians then a track in metres, `layout_plan.h:145-146`, `layout_replay.cc:36-40,86-92`) with `options.max_deviation = f.max_deviation` (`layout_replay.cc:91`) to the same `Trajectory::Plan`, so for an arm with a track the one bound mixes the metres of the carriage with the radians of the joints (the planner weights the track in the search, `layout_plan.cc:643` `ro.weights = m.weights.head(dims)`, but the bound in `path_timing.h` has no weights). Whether the weights are applied inside the deviation test is not read here.

## 10. Programs

Command: `bazel query 'kind(cc_binary, //...)'` (27); main files `grep -ln '^int main' src/*.cc` (20 incl. `viewer.cc`). Lines `wc -l`. "Display" means a window (GLFW).

| binary | source (lines) | prints | flags (from usage/comment) | display |
|---|---|---|---|---|
| `run_cell` | `run_cell.cc` (804) | per-trial scores, CSV or table | `--pretty --seed --trials --modules --first --nominal --camera --blind --noise --no-noise --calibration a,b --check-calibration --carry --check --planner --xy --yaw --phases --save-failures --replay --verbose` | no |
| `run_gate` | `run_gate.cc` (156) | pallet gate batch | `--trials --seed --threads --modules --save-failures` | no |
| `print_plan` | `print_plan.cc` (495) | planner statistics | `--pretty --module --slot --seeds --first-seed --return --planner --ompl-* --summary --resample --corner ...` | no |
| `print_check` | `print_check.cc` (432) | check-after-pickup table | `--pretty --trials --first-seed --noise --cell --study --angle --position --pgm --no-roof --verbose` | no |
| `print_perception` | `print_perception.cc` (389) | finder vs truth | `--pretty --trials --first-seed --noise --depths --calibration-error --arm-home --no-creases --verbose` | no |
| `print_depth` | `print_depth.cc` (225) | one depth image, PGM | `--seed --nominal --noise --points --last --pgm --pgm8` | no |
| `print_rrt` | `print_rrt.cc` (332) | toy-world planner runs | `--sweep --shorten-sweep --nn --stats` | no |
| `print_pallet` | `print_pallet.cc` (183) | stack stillness | positional seconds | no |
| `print_cell` | `print_cell.cc` (184) | cell contents, contacts | positional | no |
| `print_points`, `print_plane_fit`, `print_profile` | `print_points.cc` (64), `print_plane_fit.cc` (103), `print_profile.cc` (86) | CSV or `--pretty` table | `--pretty`, positional | no |
| `step_arm` | `step_arm.cc` (50) | joint angles after 1 s | model path | no |
| `print_bay` | `print_bay.cc` (283) | bay hardware figures | `--capture --summary` | no |
| `gen_bay` | `gen_bay.cc` (15) | `models/bay.xml` to stdout | none | no |
| `check_layout` | `check_layout.cc` (276) | a layout's check rows, CSV or table | `--list --checks --steps --summary --pretty` | no |
| `plan_layout` | `plan_layout.cc` (500) | one row per move, plan JSON | `--summary --margins --out --pretty --threads --budgets --first-step --last-step --why` | no |
| `view_cell` | `view_cell.cc` (530) | window | ~25 flags (`--seed`, camera, overlay, ...) | yes |
| `viewer_layout` | `view_layout.cc` (431) | window | `--state --play --at --from-time --to-time --speed --rails --modules --no-overlay` | yes |
| `viewer`, `viewer_arm`, `_bay`, `_cell`, `_container`, `_conveyor`, `_pallet`, `_suction` | `viewer.cc` (96), shared | window | `ParseViewerArgs` flags | yes |

Mains that carry logic rather than only calling a library: `run_cell.cc` 804, `print_plan.cc` 495, `plan_layout.cc` 500, `view_cell.cc` 530, `print_check.cc` 432, `view_layout.cc` 431, `print_perception.cc` 389, `print_rrt.cc` 332 (its toy worlds are in `rrt_worlds.h`), `print_bay.cc` 283 (geometry arithmetic in `main`). Thin: `gen_bay` 15, `step_arm` 50, `viewer` 96, `print_points` 64, `print_profile` 86, `print_plane_fit` 103. Sharing through a library: `run_cell`, `run_gate` and `view_cell` use `trial_runner`/`pick_place`; `check_layout`, `plan_layout` and `view_layout` use `layout*`; `print_check` uniquely carries `check_placement.*` in its own `srcs` (no library, section 6).

## 11. Outside libraries

`MODULE.bazel` (unchanged since 68a1199):
- `bazel_dep`: `rules_cc` 0.2.28; `mujoco` 4.0.0; `eigen` 5.0.1.bcr.2; `glfw` 3.4.0.bcr.4; `nlohmann_json` 3.12.0.bcr.2; `googletest` 1.18.0.bcr.1 (`dev_dependency = True`); 17 `boost.*` at 1.90.0.bcr.1 (`boost.graph` at 1.90.0.bcr.2): `algorithm assert concept_check config dynamic_bitset foreach functional graph math numeric_odeint property_map random range serialization smart_ptr thread type_traits`.
- Repository rules: `menagerie_model` `menagerie_ur10e` at commit `370c43965323c0944b433b202aac5058b5f907a9` with a sha256 for each of 25 files (`bazel/menagerie.bzl`); `kuka_description` `kuka_kr210_r2700_2` at commit `d2fa4377b77b7370cdcf6e52b430fc514a0eef19` with sha256 for 15 files (`bazel/kuka.bzl`); `http_archive` `ompl` 2.0.2, sha256 `b9155e243e555b04b01f8e4ff7e21a48582edbab9c300a611289655778bcd29d`, `bazel/ompl.BUILD.bazel`.

Targets listing each directly (`bazel query "attr(deps, '$l', //...)" | grep -c '^//'`), and files that include it (include scan):

| label | targets listing (Context) | files including (src / tests) | listing but no include in own srcs/hdrs |
|---|---|---|---|
| `@eigen` | 78 (75) | 95 (64 / 31) | 3: `bay_model_test`, `print_plane_fit`, `print_points` |
| `@mujoco` | 67 (64) | 70 (41 / 29) | none |
| `@googletest` | 42 (37) | 41 (0 / 41) | none (42 targets, 41 files: one file builds two) |
| `@nlohmann_json` | 4 (2) | 3 (2 / 1) | `plan_replay_test` (targets: `layout`, `layout_model_test`, `layout_plan`, `plan_replay_test`) |
| `@ompl` | 1 (1) | 1 (`ompl_search.cc`) | none |
| `@glfw` | 1 (1) | 1 | none |

`bazel mod graph 2>&1 | tail -3` still ends "ERROR: error evaluating module extension @@rules_python+//python/extensions:pip.bzl%pip ... ERROR: Results may be incomplete as 1 extension failed."; `bazel mod deps 2>&1 | tail -3` prints the same: both still fail (the `rules_python` pip extension).

Boost: `bazel query 'deps(@ompl//:ompl)' --output=package 2>/dev/null | grep -i boost | sort -u` prints 81 packages: the 17 apparent names (`@boost.algorithm//`...) and 64 canonical `@@boost.<name>+//` ones, i.e. 64 further modules (align, any, array, atomic, bimap, bind, chrono, compute, container, ..., xpressive) that Bazel pulls in through the 17 and that need no `bazel_dep` of ours. `bazel query 'deps(@ompl//:ompl, 1)' | grep boost` prints exactly the 17: all 17 are direct deps of OMPL's target (`bazel/ompl.BUILD.bazel:107-123`), none is transitive only. Which are needed: `boost/` includes in OMPL's source tree (`grep -rhoE '#\s*include\s*[<"]boost/[a-z_0-9]+' <ompl>/src/ompl | sort | uniq -c`, over all of `src/ompl`, not only the compiled `_DIRS`): graph 59, math 40, foreach 12, serialization 9, property_map 8, concept_check 8, range 7, archive 7 (serialization), scoped_ptr 6 (smart_ptr), random 6, pending 6, numeric 6 (`numeric/odeint`), functional 3, version 1 (config), type_traits 1, thread 1, dynamic_bitset 1, assert 1, algorithm 1: every one of the 17 is included by OMPL's own sources. Redundancy by closure: for each of the 17, a query `deps(set(the other 16))` finds the module again for 15 of them (so, in a minimal list, those 15 could be left to the transitive closure); only `graph` and `numeric_odeint` are not reachable from the other sixteen. Naming them all is the safe choice; the list could be 2 to 17 entries.
