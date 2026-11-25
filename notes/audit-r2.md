# Re-run of the structure audit at 940a01b (agent `audit`, run r2)

Checkout `/root/work/013-design-principles-and-r2/repo`, HEAD `f01fea9`. `git diff --stat 940a01b HEAD -- src tests BUILD.bazel MODULE.bazel .bazelrc bazel docs ':!docs/architecture'` prints nothing, so every figure is of `940a01b`; `tools/architecture/` and `docs/architecture/research.md` are in no count (`ls src`, `ls tests`, `BUILD.bazel` do not hold them).
Commands run from the repo root after `export PATH=$HOME/.local/bin:$PATH`. Nothing built; only `bazel query`, `bazel mod`, `bazel info`, grep, python3 (`-I`).
Scratch: `/root/work/013-design-principles-and-r2/out/audit/` (`inc.py` include scanner, `inc.json`, `rules.jsonl` = `bazel query 'kind(rule,//...)' --output=streamed_jsonproto`, `rules_loc.txt` = same with `--output=location`, `reached.txt`, `det_src.txt`).
Verdict column: `SAME`, or `DIFFERS (old: X)`. "old" = `notes/r1-audit.md`. Context column of the old section 1 (commit 68a1199) was re-run too (git archive).

Two helper commands used below (family scan and include table):
- `python3 -I /root/work/013-design-principles-and-r2/out/audit/inc.py` prints include counts and every cross-family edge by file:line; with an argument `x` it also prints transitive reach, mixed tests, header cycles. Families as the AGENTS.md map groups them: L (new cell) = `layout*`, `arm_catalog`, `flat_rack`, `check_layout`, `plan_layout`, `view_layout`, `plan_replay_test`; B (bay and container, "beside the cell") = `bay_hardware`, `bay_model`, `container`, `print_bay`, `gen_bay`; N (neutral) = `mj_handles`, `portable_random`; S (stand-in) = the rest.

## 0. The corrections that matter most

| # | Old statement | Command | I got | Verdict |
|---|---|---|---|---|
| 1a | `Checker::Solve` at `layout_check.cc:299`, declared :220 | `grep -n 'Solve(' src/layout_check.cc` | 220 (decl), 299 (def), calls 536, 678, 863, 2084, 2119 | SAME |
| 1b | no third solver | `grep -n 'SolveIk\|Solve(' src/layout_plan.cc src/layout_scene.cc src/layout_replay.cc src/layout_rows.cc` | prints nothing | SAME |
| 2a | `src/print_layout_moves.cc` gone | `ls src/print_layout_moves.cc` | No such file | SAME |
| 2b | removed file was 644 lines, only deletion | `git diff --stat --diff-filter=D 68a1199 940a01b -- src tests` | `src/print_layout_moves.cc \| 644 ---`, 1 file | SAME |
| 2c | one copy of moving/obstacle sets, `layout_scene.cc:178,221,483,497`, `LeastDistance` :531 | `grep -nE 'LayoutScene::(Moving\|Obstacles\|WayMoving\|WayObstacles\|LeastDistance)\(' src/layout_scene.cc` | 178, 221, 483, 497, 531 | SAME |
| 2d | `docs/layout.md:667` tells to run `//:print_layout_moves`, `:11` says it no longer exists | `grep -n 'print_layout_moves' docs/layout.md` | 11 and 667 | SAME |
| 3 | 31 of 42 headers open `namespace cell {`, 11 only a nested one | see section 2 | 31 / 11 | SAME |
| 4a | `collision.h:119-120` includes `arm.h`, `mj_handles.h`, no `pallet.h` | `grep -n '#include "src/' src/collision.h` | 119 `arm.h`, 120 `mj_handles.h` | SAME |
| 4b | `motion_planner.h:141-145` no direct `arm.h` | `grep -n '#include "src/' src/motion_planner.h` | 141 collision, 142 joint_move, 143 ompl_search, 144 path_timing, 145 rrt | SAME |
| 4c | `layout_plan.h:137` includes `joint_move.h`; `joint_move.h:20` includes `profile.h`; `layout_replay.cc:21-22` path_timing, rrt | `grep -n '#include "src/' src/layout_plan.h src/joint_move.h src/layout_replay.cc \| grep -E 'joint_move\|profile\|path_timing\|rrt'` | all four lines as stated | SAME |
| 5a | `mj_geomDistance` called in four `src/` files: `collision.cc:526`, `cycle_check.cc:306`, `check_placement.cc:490`, `layout_scene.cc:575,586` | `grep -n 'mj_geomDistance' src/*.cc` | exactly those 5 call lines | SAME |
| 5b | workaround in `layout_model.cc:717-731` | `grep -n 'mjGEOM_CAPSULE\|mjGEOM_CYLINDER' src/layout_model.cc` | comment from 717, loop 727-732 | DIFFERS (old: 717-731; code ends :732, trivial) |
| 5c | "every stand-in caller measures against the UR10e's capsules unmended" | `grep -n 'include' models/cell.xml`; `grep -o 'type="[a-z]*"' models/arm.xml \| sort \| uniq -c`; `grep -c 'type="capsule"' models/*.xml \| grep -v ':0'` | `cell.xml` includes `arm.xml`, a KUKA KR210 model whose only geom type is `mesh`; the only capsule in `models/` is `suction_scene.xml:57` (`rig_post`). The UR10e (capsules, Menagerie `ur10e.xml` has 1 `capsule` line, class default :32) is loaded only by the new cell (`layout_model.cc:388`) and by `step_test` (`BUILD.bazel:48`) | DIFFERS (old: stand-in callers see UR10e capsules; false: the stand-in's arm is the KUKA with meshes, capsules reach it nowhere but `suction_scene`'s rig post) |
| 6 | counts: 99 src, 41 tests, 110 rules, 42 tests | section 1 | same | SAME |
| 6b | `@eigen` 78 / `@mujoco` 67 / `@googletest` 42 / `@nlohmann_json` 4 / `@ompl` 1 / `@glfw` 1 | section 11 | same | SAME |
| 6c | `check_placement.*` reached by no test | section 6 | confirmed | SAME |
| 6d | `rrt.cc` draws via `std::uniform_real_distribution` | `grep -n 'uniform_real_distribution' src/rrt.cc` | :442, :522 | SAME |
| 6e | `bazel mod graph`, `mod deps` fail | `bazel mod graph 2>&1 \| grep ERROR` | rules_python pip extension error + "Results may be incomplete as 1 extension failed" (also `mod deps`) | SAME |

## 1. Size

All commands below; "ctx" = at 68a1199 re-run with `git archive 68a1199 src tests` into scratch and `git ls-tree`.

| What | Command | Got | Verdict |
|---|---|---|---|
| files in `src/` | `ls src \| wc -l` | 99 | SAME |
| non `.h`/`.cc` files in `src/` | `ls src \| grep -v '\.\(h\|cc\)$'` | nothing | SAME |
| headers / `.cc` | `ls src/*.h \| wc -l`; `ls src/*.cc \| wc -l` | 42 / 57 | SAME |
| lines, headers / `.cc` / both | `cat src/*.h \| wc -l`; `cat src/*.cc \| wc -l`; `cat src/*.h src/*.cc \| wc -l` | 10,945 / 28,808 / 39,753 | SAME |
| files in `tests/`; not `*_test.cc` | `ls tests \| wc -l`; `ls tests \| grep -vc '_test.cc$'` | 41; 0 | SAME |
| lines in `tests/` | `cat tests/* \| wc -l` | 25,898 | SAME |
| `BUILD.bazel` lines | `wc -l BUILD.bazel` | 2,133 | SAME |
| `visibility`, `copts`/`features` | `grep -c visibility BUILD.bazel`; `grep -c 'copts\|features' BUILD.bazel` | 0; 0 | SAME |
| test targets | `bazel query 'tests(//...)' \| wc -l` | 42 | SAME |
| all rules | `bazel query 'kind(rule, //...)' \| wc -l` | 110 | SAME |
| `cc_library` | `bazel query 'kind(cc_library, //...)' \| grep -c '^//'` | 38 | SAME |
| `cc_binary` | same with `cc_binary` | 27 | SAME |
| `cc_test` | same with `cc_test` | 42 | SAME |
| `genrule`, `filegroup`, `alias` | same | 1, 1, 1 | SAME |
| rules at line start | `grep -o '^[a-z_]*(' BUILD.bazel \| sort \| uniq -c` | alias 1, cc_binary 27, cc_library 38, cc_test 40, filegroup 1, genrule 1, load 3 | SAME |
| comprehension | `grep -n '\[cc_test(' BUILD.bazel` | :1469 (makes `check_cycle_test`, `check_cycle_batch_test`) | SAME |
| 27 binaries = 19 distinct mains + `viewer.cc` x 8 | `bazel query 'kind(cc_binary,//...)'` + srcs from `rules.jsonl` | `viewer`, `viewer_arm`, `_bay`, `_cell`, `_container`, `_conveyor`, `_pallet`, `_suction` all `src/viewer.cc`; 19 other binaries each with own main (`viewer_layout` = `view_layout.cc`) | SAME |
| ctx: src 91 / .h 38 / .cc 53 / lines 32,476 / tests 36 / lines 22,929 / BUILD 1,989 / AGENTS 115 | `git ls-tree --name-only 68a1199 src/ \| grep -c '\.h$'` etc., `git show 68a1199:AGENTS.md \| wc -l` | 91, 38, 53, 32,476, 36, 22,929, 1,989, 115 | SAME |
| ctx: 98 rules, 37 tests, 36 libs, 35 tests-kind | not re-run (needs a Bazel query of the old commit's package) | | see Could not decide |
| AGENTS.md lines, sections | `wc -l AGENTS.md`; `grep -n '^## ' AGENTS.md` | 124; Build and test :7, Caches :33, Dividing :53, Rules :64, Frames :84, Map :93 | SAME |

Ten largest files, `wc -l src/* tests/* | sort -rn | sed -n 2,11p`: `src/layout_check.cc` 2610, `src/pick_place.cc` 1911, `src/layout_replay.cc` 1812, `tests/check_cycle_test.cc` 1654, `src/layout_plan.cc` 1462, `src/rectangle_fit.cc` 1252, `tests/layout_model_test.cc` 1247, `tests/rectangle_fit_test.cc` 1235, `tests/layout_check_test.cc` 1185, `src/pick_place.h` 1173. SAME.

`# BEGIN` sections: `grep -c '^# BEGIN' BUILD.bazel` = 29, `grep -c '^# END' BUILD.bazel` = 29. Old text says "28 sections"; its own table lists 29 rows. DIFFERS (old: 28).
Rules per section: python over `rules_loc.txt` (rule line between `# BEGIN x` and `# END x`, `grep -n '^# BEGIN\|^# END' BUILD.bazel` for the markers), total 93 inside, 17 outside. SAME for 93 / 17.

| section | rules | section | rules | section | rules |
|---|---|---|---|---|---|
| arm | 4 | rrt | 4 | container | 3 |
| pallet | 4 | path_timing | 3 | flat_rack | 2 |
| conveyor | 3 | motion_planner | 3 | layout | 4 |
| suction | 3 | held_module_finder | 4 | layout_check | 4 |
| follower | 2 | check_camera_cell | 1 | layout_rows | 1 |
| ik | 2 | planned_carry | 1 | locate | 1 |
| cell | 5 | check_cycle | 2 | layout_plan | 4 |
| pick_place | 6 | gate | 4 | | |
| depth_camera | 4 | ompl | 3 | | |
| rectangle_fit | 3 | bay | 8 | | |
| module_finder | 3 | collision | 2 | | |

All 29 per-section counts SAME as the old table. The 17 outside: `//:arm` (:445), `joint_move`(:76), `joint_move_test`(:86), `mj_handles`(:19), `plane_fit`(:109), `plane_fit_test`(:128), `point_generator`(:118), `point_generator_test`(:140), `print_plane_fit`(:156), `print_points`(:169), `print_profile`(:101), `profile`(:58), `profile_test`(:64), `step_arm`(:29), `step_test`(:43), `viewer_lib`(:655), `viewer`(:668). SAME (old: lines 19-169, 445, 655, 668).

Diff 68a1199..940a01b:
| Claim | Command | Got | Verdict |
|---|---|---|---|
| new BUILD sections | `git diff 68a1199 940a01b -- BUILD.bazel \| grep '^[+-]# BEGIN'` | `+flat_rack`, `+layout_rows`, `-layout_moves`, `+layout_plan` | DIFFERS (old omits that section `layout_moves` was removed) |
| `MODULE.bazel`, `bazel/`, `.bazelrc` unchanged | `git diff --stat 68a1199 940a01b -- MODULE.bazel bazel .bazelrc \| wc -l` | 0 | SAME |
| added 14 files, 8,004 lines | `git diff --stat --diff-filter=A 68a1199 940a01b -- src tests` | 14 files, 8004 insertions; per-file lines flat_rack.h 145, layout_plan.cc 1462, layout_plan.h 468, layout_replay.cc 1812, layout_rows.cc 766, layout_rows.h 87, layout_scene.cc 613, layout_scene.h 321, plan_layout.cc 500, flat_rack_test 173, layout_plan_test 170, layout_rows_test 445, layout_scene_test 220, plan_replay_test 822 | SAME |
| removed | `--diff-filter=D` | print_layout_moves.cc 644 | SAME |
| renamed none | `git diff -M --summary --diff-filter=R 68a1199 940a01b -- src tests` | nothing | SAME |
| modified 13, +3,978 -1,092 | `--diff-filter=M --stat` | 13 files, 3978 insertions, 1092 deletions; names as old | SAME |
| whole commit | `git diff --shortstat 68a1199 940a01b` | 50 files, 19,074 insertions, 1,785 deletions | SAME |

## 2. Namespaces

Commands: `grep -n '^ *namespace\b' src/*.h | grep -v ':namespace cell {$'` (11 lines); `grep -c '^namespace cell {' src/*.h | grep -c ':1'` (31).

| namespace opened by a header | headers | Verdict |
|---|---|---|
| `cell` only | 31 of 42 | SAME |
| `cell::layout` | 7: `arm_catalog.h:27`, `layout.h:98`, `layout_check.h:291`, `layout_model.h:74`, `layout_plan.h:143`, `layout_rows.h:71`, `layout_scene.h:138` | SAME |
| `cell::bay` | 2: `bay_hardware.h:87`, `bay_model.h:58` | SAME |
| `cell::container` | 1: `container.h:42` | SAME |
| `cell::flat_rack` | 1: `flat_rack.h:50` | SAME |
| anonymous in a header | `grep -c '^namespace {' src/*.h \| grep -v ':0'` prints nothing: 0 | SAME |
| `using namespace` in `src/` | `grep -n 'using namespace' src/* tests/*`: `print_bay.cc:60,201` (`cell::bay`), `plan_layout.cc:50`, `view_layout.cc:103,291` (`cell::layout`); none in tests | SAME |
| `.cc` namespaces | `grep -h '^ *namespace\b' src/*.cc \| sed 's/ *{.*//' \| sort \| uniq -c` | 60 `namespace`, 29 `namespace cell`, 1 `cell::bay`, 7 `cell::layout` (= 8 nested), `ob = ompl::base`, `og = ompl::geometric` | SAME |

Header-to-namespace in full (11 non-plain): above table; every other header (31) is plain `cell`.

## 3. The two families and what joins them

Method: `inc.py` (regex on `#include` lines of every file in `src/` and `tests/`). Families as defined at the top.
Members (src): L = `arm_catalog.h check_layout.cc flat_rack.h layout.cc layout.h layout_check.cc layout_check.h layout_model.cc layout_model.h layout_plan.cc layout_plan.h layout_replay.cc layout_rows.cc layout_rows.h layout_scene.cc layout_scene.h plan_layout.cc view_layout.cc` (18); B = `bay_hardware.h bay_model.cc bay_model.h container.h gen_bay.cc print_bay.cc` (6); N = `mj_handles.h portable_random.h`; S = the other 73.

| Claim | Command | Got | Verdict |
|---|---|---|---|
| 16 of 42 headers include no project header | `inc.py` (headers with no `#include "src/..."`) | `arm_catalog bay_hardware camera_model check_placement container flat_rack layout mj_handles pallet plane_fit portable_random profile rectangle_fit rrt suction viewer_lib` = 16 | SAME |
| include counts, src / tests (files that include) | `inc.py` | `mj_handles` 17/19; `arm` 13/12; `cell` 14/11; `pallet` 12/10; `camera_model` 16/5; `depth_camera` 14/4; `layout` 9/6; `suction` 10/4; `layout_model` 8/6; `rrt` 11/3; `pick_place` 7/7; `joint_move` 8/5; `bay_hardware` 6/7; `conveyor` 5/8 (also `held_module_finder` 9/4, `plane_fit` 9/4) | SAME (all 14 rows) |
| `mj_handles` from 36 files | `grep -l '#include "src/mj_handles.h"' src/* tests/* \| wc -l` | 36 | SAME |
| `mujoco/mujoco.h` 17 headers + 24 `.cc` = 41 src files, 29 tests | `grep -l '#include <mujoco/' src/*.h \| wc -l` etc. (inc.py) | 17 + 24 = 41; tests 29 (0 headers) | SAME |
| `Eigen/` 64 src (30 h + 34 cc), 31 tests | inc.py | 64, 31 | SAME |
| `ompl/` only `ompl_search.cc` | `grep -l '#include <ompl/' src/* tests/*` | `src/ompl_search.cc` | SAME |
| `nlohmann/` | `grep -l '#include <nlohmann/' src/* tests/*` | `src/layout.cc`, `src/layout_replay.cc`, `tests/layout_model_test.cc` | SAME |
| `GLFW/` one src file | `grep -l '#include <GLFW' src/*` | `src/viewer_lib.cc` | SAME |
| `gtest/` in 41 tests | `grep -l '#include <gtest/' tests/* \| wc -l` | 41 | SAME |

Includes that join the families, direct, by file:line (`inc.py`; equivalently `grep -n '#include "src/' <file>`):

New cell (L) to stand-in (S), 5 lines:
| line | includes | Verdict |
|---|---|---|
| `src/layout_plan.h:137` | `src/joint_move.h` | SAME |
| `src/layout_plan.cc:26` | `src/rrt.h` | old lists it only as a use; DIFFERS (old: "exactly one" direct include) |
| `src/layout_replay.cc:21` | `src/path_timing.h` | same |
| `src/layout_replay.cc:22` | `src/rrt.h` | same |
| `src/view_layout.cc:69` | `src/viewer_lib.h` | not in old; DIFFERS |

Transitive (S headers reached): `layout_plan.cc` joint_move, profile, rrt; `layout_plan.h` joint_move, profile; `layout_replay.cc` joint_move, path_timing, profile, rrt; `plan_layout.cc` joint_move, profile; `view_layout.cc` joint_move, profile, viewer_lib. `layout_check.cc`, `layout_scene.cc`, `layout_model.cc`, `layout.cc`, `layout_rows.cc`, `check_layout.cc` reach no S header. SAME as old.
S to L, and S to B, direct or transitive, in `src/`: none (`inc.py x`, "S src files reaching L or B" prints nothing). SAME.
B to S or L in `src/`: none. (not in old)

New cell (L) to bay and container (B), 11 lines in `src/` and 4 in tests (not in old):
`src/layout_check.cc:27` bay_hardware.h, `:28` container.h; `src/layout_model.cc:70` bay_model.h; `src/layout_model.h:70` bay_hardware.h; `src/layout_plan.cc:24` bay_hardware.h; `src/layout_rows.cc:22` container.h; `src/layout_scene.cc:16` bay_hardware.h; tests `layout_check_test.cc:23,24`, `layout_model_test.cc:36,37`.

Neutral headers:
| Claim | Command | Got | Verdict |
|---|---|---|---|
| `portable_random.h` from `layout_check.cc:32`, `layout_plan.cc:25` | `grep -n '#include "src/portable_random.h"' src/* tests/*` | those two plus `src/rectangle_fit.cc:13` (stand-in) | DIFFERS (old omits `rectangle_fit.cc:13`, it said "(L)" only) |
| `mj_handles.h` from `layout_check.cc:31`, `layout_model.h:72`, `layout_rows.cc:24`, `layout_scene.h:136`, `view_layout.cc:68` | `grep -n '#include "src/mj_handles.h"' src/*` | those 5, plus 12 stand-in files: `cell.cc:21 cell.h:57 check_placement.cc:26 check_rig.cc:19 check_rig.h:28 collision.cc:16 collision.h:120 pick_place.cc:28 pick_place.h:260 print_pallet.cc:24 step_arm.cc:14 viewer.cc:50` | SAME |

Tests that mix families (`inc.py x`, "tests with >1 family"):
| test | families | Verdict |
|---|---|---|
| `tests/locating_gate_test.cc` | B `bay_hardware.h:12` + S `trial_runner.h:13` | SAME (old's only one, if bay counts with S) |
| `tests/layout_check_test.cc` | L + B (`bay_hardware.h:23`, `container.h:24`) | DIFFERS (old: only `locating_gate_test` mixes; true only if bay is counted with the new cell) |
| `tests/layout_model_test.cc` | L + B (`bay_hardware.h:36`, `container.h:37`) | same |
| `tests/plan_replay_test.cc:19-23` | includes only L headers (layout, layout_check, layout_model, layout_plan, layout_scene) | no mix |

Planner headers (`grep -n '#include' file`):
- `collision.h`: Eigen/Core, Eigen/Geometry, mujoco (115-117), `arm.h` 119, `mj_handles.h` 120. SAME.
- `motion_planner.h`: Eigen/Core, Eigen/Geometry, mujoco (137-139), 141-145 as in section 0. SAME.
- `rrt.h`: 66-69 std (`cstdint functional optional vector`), 71 `Eigen/Core`; no project header. SAME.
- `path_timing.h`: 128-134: optional, vector, Eigen/Core, `joint_move.h` 133, `profile.h` 134. SAME.
- `ompl_search.h`: 117-125: std, Eigen/Core, `rrt.h` 125; no `ompl/`. SAME.
Header cycles: none (`inc.py x`, DFS over project headers: `header cycles: []`). SAME ("not formal" in the old; this is a DFS).

## 4. Things written twice

(a) Inverse kinematics
| Claim | Command | Got | Verdict |
|---|---|---|---|
| `SolveIk` `ik.cc:133`, decl `ik.h:162`, `SolveIkPath` `ik.cc:248` | `grep -n 'SolveIk' src/ik.h src/ik.cc` | ik.h:162, ik.h:198 (path decl), ik.cc:133, 248 (and 272 internal call) | SAME |
| callers of `SolveIk*` | `grep -n 'SolveIk' src/*.cc \| grep -v '^src/ik\.'` | `pick_place.cc:331` (SolveIk), `:353` (SolveIkPath); `check_placement.cc:302,716,720` | SAME |
| `SolveIk`: status enum, `max_distance_from_seed`, step limit | `grep -n 'IkStatus\|max_distance_from_seed\|max_step' src/ik.h src/ik.cc` | `IkStatus` ik.h:114, option ik.h:110, `max_step` clamp ik.cc ~196 | SAME |
| `Checker::Solve` track at `:307` | `sed -n 307p src/layout_check.cc` | `if (a.h->track_qpos >= 0) d->qpos[a.h->track_qpos] = track;` | SAME |
| axis-only target when `has_x` false | `grep -n 'has_x' src/layout_check.cc` | branch at :330-347 | SAME |
| `kOrientWeight`, adaptive lambda, 300 iterations | `grep -n 'kOrientWeight\|lambda\|< 300' src/layout_check.cc` | `kOrientWeight = 0.5` :46; lambda 0.01 :362, `*0.5` :386, `*5` :388; loop `it < 300` :365 | SAME |
| same normal equations at `ik.cc:178-184` and `layout_check.cc:381-383` | `grep -n 'ldlt().solve' src/ik.cc src/layout_check.cc` | `ik.cc:191`, `layout_check.cc:370` (ik.cc:178-184 is the comment above the step; layout_check.cc:381-383 is the acceptance branch) | DIFFERS (old: ik.cc:178-184, layout_check.cc:381-383; actual solve lines 191 and 370) |

(b) Collision and distance
| Claim | Command | Got | Verdict |
|---|---|---|---|
| `collision.cc:526` call, bounding-sphere pre-test `:515-523` | `sed -n 505,526p src/collision.cc` | `geom_rbound` test from ~:512 to :524, call :526 | SAME (pre-test begins a few lines before 515) |
| `LeastDistance` `layout_scene.cc:531`, decl `layout_scene.h:294`, bounding boxes `:555-560` | `grep -n 'LeastDistance\|AabbGap' src/layout_scene.*` | 531, h:294, `AabbGap` use 553-561 | SAME |
| all `mj_geomDistance` calls, src | `grep -n 'mj_geomDistance' src/*` | calls: `collision.cc:526`, `cycle_check.cc:306`, `check_placement.cc:490`, `layout_scene.cc:575,586`. Comments: `layout_model.h:31`, `layout_scene.h:21`, `layout_check.h:41`, `collision.h:52,154`, `cycle_check.cc:22,304`, `cycle_check.h:24` | SAME |
| tests | `grep -n 'mj_geomDistance' tests/*` | `collision_test.cc:642`, `layout_model_test.cc:644` (comment :12), `cell_test.cc:149,606`, `camera_cycle_test.cc:149,435`, `motion_planner_test.cc:601` (comment :595), `pick_place_test.cc:150,350` | SAME |
| workaround rationale `:719-723` (+0.1975 vs -0.2525) | `sed -n 717,732p src/layout_model.cc` | 717 start, figures at 721-723, rewrite loop 727-732 | SAME |
| calls through it: `layout_scene.cc:575,586`, `layout_model_test.cc:644` | `grep -n 'LoadLayoutModel' src/layout_scene.cc` | :91 `model_ = LoadLayoutModel(...)` | SAME |
| calls NOT through it: `collision.cc`, `cycle_check.cc`, `check_placement.cc` "see UR10e's capsules" | see section 0 item 5c | those call sites measure the KUKA mesh model of `models/cell.xml` | DIFFERS (old: UR10e capsules unmended) |
| `grep -c capsule models/*.xml` non-zero only `suction_scene.xml` | `grep -c capsule models/*.xml \| grep -v ':0'` | `models/suction_scene.xml:1` | SAME |

(c) Moving and obstacle sets
| Claim | Command | Got | Verdict |
|---|---|---|---|
| definitions `layout_scene.cc:178,221,483,497`; enums `layout_scene.h:146-160` | `grep -n 'enum' src/layout_scene.h` | defs as in section 0; enums present in that range | SAME |
| users `layout_check.cc:451,456,829,1158`, `layout_replay.cc:735-736`, per-arm cache `:775-776` | `grep -nE '\b(Moving\|Obstacles\|WayMoving\|WayObstacles)\(' src/layout_check.cc src/layout_replay.cc src/layout_plan.cc src/layout_rows.cc` | `layout_check.cc:451,457,829,840,841,846,1158,1487,1488,2060,2061,2169,2171,2173`; `layout_plan.cc:161,167,774`; `layout_replay.cc:735,736` (`:773-776` is the `ArmCache` struct, not the build); `layout_rows.cc` none | DIFFERS (old: incomplete; also uses in `layout_plan.cc`, 13 `layout_check.cc` sites; `:456` is `:457`) |
| hand-written probe set `layout_check.cc:1145-1163` | `grep -n 'kProbe\|link_geoms\|carriage_geoms' src/layout_check.cc` | 1147, 1162, 1163, 1166 | SAME |

(d) Hardware literals
| Claim | Command | Got | Verdict |
|---|---|---|---|
| stand-in module 2.3 x 1.1 x 0.035: four code copies | `grep -n '2\.3\b\|1\.1\b\|0\.035' src/*.h src/*.cc models/*.xml` | `suction.h:45`, `pallet.h:27-29`, `conveyor.h:69-71`, `point_generator.h:64-66` (four, none derived) | SAME |
| prose in `rectangle_fit.h:5`, `module_finder.h:39`, `held_module_finder.h:51`, `rrt.h:4`, `cell.cc:29` | same grep | same lines | SAME |
| in models: `pallet.xml:77`, `check_scene.xml:51`, `conveyor_scene.xml:20,25,30`, `suction_scene.xml:40` | same grep | `pallet.xml:74` (also :40, :108, :119), `check_scene.xml:14`, `conveyor_scene.xml:9`, `suction_scene.xml:14` (also `conveyor.xml:11,15`, `tool.xml:17`) | DIFFERS (old line numbers wrong; actual as listed) |
| real module literals only in `bay_hardware.h:103-106`, tube 0.062 `:90`, `kModulePitch :111` | `grep -n 'kModuleLength\|kModuleWidth\|kModuleThickness\|kModuleMass\|kTubeRadius\|kModulePitch' src/bay_hardware.h` | 90, 103-106, 111 | SAME |
| other src/tests hits of those numbers are comments | `grep -n '2\.382\|1\.134\|1\.144\|0\.062\|0\.0625\|10\.516\|32\.4' src/*.cc src/*.h tests/*.cc \| grep -v '^src/bay_hardware.h'` | `layout_check.h:199`, `locate_physics_test.cc:11,15` only | DIFFERS (old also lists `locate_physics_test.cc:73` and `check_cycle_test.cc:119`; neither matches now) |
| `layouts/below_rails_by_module_arm.json` rail offsets 1.144 | `grep -c '1\.144' layouts/below_rails_by_module_arm.json` | 10 (lines 109, 122, 135, ...) | SAME |
| `kPi` local constant in 8 `.cc` | `grep -n 'kPi = ' src/*.cc` | `camera_model.cc:14`, `depth_camera.cc:20`, `layout_replay.cc:28`, `score.cc:17`, `layout_model.cc:75`, `layout_plan.cc:32` (digits); `check_placement.cc:40`, `pick_place.cc:50` (`std::numbers::pi`) = 8 | SAME |
| digits `3.14159265358979323846` elsewhere | `grep -n '3\.14159265358979323846' src/*.h src/*.cc \| grep -v 'kPi = '` | `bay_hardware.h:233,503,712`, `print_bay.cc:77,136,203,212`, `print_cell.cc:31` | DIFFERS (old also named `view_cell.cc:298,339,340,444`: no such digits in `view_cell.cc`) |
| `M_PI` | `grep -n 'M_PI' src/*` | `print_pallet.cc:139` | SAME |
| `std::numbers::pi` used in `check_placement.cc:40`, `pick_place.cc:50`, `point_generator.cc:11` | `grep -n 'numbers::pi' src/*` | 40 lines in 25 files (e.g. `held_module_finder.h:113`, `ik.h:60,245`, `module_finder.h:118,268,479`, `rectangle_fit.cc:394,432,816`, `layout_check.cc:40,625,2046`, `layout_scene.cc:24`, `layout_rows.cc:329,666`, `run_cell.cc:136`, `trial_runner.h:182,183`, `trial_runner.cc:27`...) | DIFFERS (old: three places; it is the common form, 25 files) |

(e) Argument parsing
| Claim | Command | Got | Verdict |
|---|---|---|---|
| `ParseViewerArgs` `viewer_lib.cc:214`, used by `viewer.cc:65`, `view_layout.cc:354` only | `grep -n 'ParseViewerArgs' src/*.cc` | 214, 65, 354 (`view_cell.cc` does not use it) | SAME |
| 20 `main`s | `grep -ln '^int main' src/*.cc \| wc -l` | 20 (incl. `viewer.cc`) | SAME |
| 8 `strcmp(argv[i], "--x") == 0` loops, lines | `sed -n <n>p` of each | `run_cell.cc:439`, `run_gate.cc:63`, `print_depth.cc:110`, `print_check.cc:158`, `print_perception.cc:143`, `view_cell.cc:116`, `print_profile.cc:30`, `print_plane_fit.cc:29` are `for (int i...argc...)` loops; `strcmp(argv[i]` count run_cell 21, run_gate 5, print_depth 7, print_check 12, print_perception 9, view_cell 27, print_profile 1, print_plane_fit 1 | SAME |
| 3 `const std::string a = argv[i]` loops | `grep -n 'std::string a = argv\[i\]' src/*.cc` | `check_layout.cc:150`, `plan_layout.cc:254`, `print_plan.cc:168` (old said 149, 253, 167: the `for` line) | SAME |
| `print_plan.cc` exits 2 on a missing value | `grep -n 'exit(2)\|return 2' src/print_plan.cc` | :164, :172 (`std::exit(2)`), :209 | SAME |
| `--pretty` in 8 programs | `grep -l '"--pretty"' src/*.cc \| wc -l` | 8: `check_layout plan_layout print_plane_fit print_plan print_perception run_cell print_check print_profile` | SAME |
| `--seed` in 4, `--first-seed` in 3 | `grep -l '"--seed"' src/*.cc`; `grep -l '"--first-seed"' src/*.cc` | `run_gate view_cell print_depth run_cell`; `print_perception print_check print_plan` | SAME |
| `FromWorkingDirectory` x2 | `grep -n 'FromWorkingDirectory' src/*.cc` | `run_cell.cc:407`, `run_gate.cc:40` | SAME |
| `LayoutDir()` x2, getenv again in `view_layout.cc:107,360`, `PLAN_DIR` `plan_layout.cc:133`, `view_layout.cc:104` | `grep -n 'LayoutDir\|LAYOUT_DIR\|PLAN_DIR' src/*.cc` | `check_layout.cc:37`, `plan_layout.cc:52`; `view_layout.cc:107,360`; `plan_layout.cc:133`, `view_layout.cc:104` | SAME |
| 12 files print their own `usage:` | `grep -ln 'usage:' src/*.cc \| wc -l` | 19 files contain "usage:" (includes comments) | not decided: old figure 12 counted by hand; command gives 19 |

(f) Table printing
| Claim | Command | Got | Verdict |
|---|---|---|---|
| `if (pretty)` printf formats at `print_check.cc:221`, `print_perception.cc:199`, `print_plan.cc:388,416`, `print_profile.cc:66`, `print_plane_fit.cc:52`, `run_cell.cc:147` (`PrintHeader`), `:302` (`PrintRow`) | `sed -n <n>p` each | all there (`print_check.cc:221` is `} else if (pretty) {`) | SAME |
| `PrintTable` `check_layout.cc:61`, `plan_layout.cc:86`; "diff of the two bodies differs in 8 lines" | `diff <(sed -n 61,84p src/check_layout.cc) <(sed -n 86,107p src/plan_layout.cc) \| grep -c '^[<>]'` | defs at 61 and 86; 10 differing lines (body ranges 61-84 and 86-107) | DIFFERS (old: 8) |
| `Csv` `check_layout.cc:43`, `plan_layout.cc:57` | `grep -n '^std::string Csv' src/*.cc` | 43, 57 | SAME |
| `Num` x4: `check_layout.cc:53` "%.4f", `bay_model.cc:46`, `layout_model.cc:81`, `plan_layout.cc:67` | `grep -n '^std::string Num' src/*.cc` | 53, 46, 81, 67 (a fifth in `tests/locate_physics_test.cc:93`) | SAME for src |
| `PrintMargins` `check_layout.cc:128`, `plan_layout.cc:130`: 8 vs 110 lines | python over `^}` after each definition | `check_layout.cc:128-136` (9 lines), `plan_layout.cc:130-243` (114 lines) | DIFFERS (old: 8 vs 110) |

(g) JSON and file reading
| Claim | Command | Got | Verdict |
|---|---|---|---|
| `LoadLayout` `layout.cc:807` | `grep -n 'LoadLayout' src/layout.cc` | 807 | SAME |
| `PlanToJson` `layout_replay.cc:321`, `PlanFromJson` `:359`, declared `layout_plan.h:311-312`, `Q()` `:207` | `grep -n 'PlanToJson\|PlanFromJson\|^std::string Q(' src/layout_replay.cc src/layout_plan.h` | 321, 359; 311, 312; 207 | SAME |
| nlohmann used only by those and `layout_model_test.cc` | `grep -ln nlohmann src/* tests/*` | `layout.cc`, `layout_replay.cc`, `layout_model_test.cc` | SAME |
| file reading written five times | `grep -n 'ifstream\|ReadFile' src/*.cc` | `layout.cc:812`, `layout_model.cc:112`, `layout_replay.cc:507`, `plan_layout.cc:137`, `view_layout.cc:110`; `trial_runner.cc:624` (+630) | SAME |

(h) Same-name helpers: python scan of column-0 function definitions with capitalised names in `src/*.cc` and `tests/*.cc` defined in two or more files (script inline in this run; output below is the full list).
All of the old list re-found at the old lines, verdict SAME: `SecondsSince` x5 (`held_module_finder.cc:31`, `module_finder.cc:32`, `motion_planner.cc:33`, `ompl_search.cc:43`, `rrt.cc:15`); `Fmt` x4 (`layout_check.cc:51`, `layout_plan.cc:38`, `layout_rows.cc:39`, `layout_replay.cc:194`); `Csv`, `PrintTable`, `FromWorkingDirectory`, `LayoutDir` (src x2, plus `tests/layout_model_test.cc:47`, `tests/plan_replay_test.cc:30`); `ExactExtent` (`layout_check.cc:68`, `layout_rows.cc:73`); `InBox` x3 (`rrt.cc:218`, `ompl_search.cc:272`, `module_finder.cc:200`); `ToConfig` (`layout_plan.cc:47`, `layout_replay.cc:36`); `Advance` (`path_timing.cc:65`, `profile.cc:12`); `AllPositiveAndFinite` (`joint_move.cc:10`, `path_timing.cc:60`); `SideSigma` (`held_module_finder.cc:69`, `module_finder.cc:141`); `WritePgm` (`print_check.cc:128`, `print_depth.cc:60`); `AngleBetween` x4 (`held_module_finder.cc:53`, `plane_fit.cc:91`, `print_pallet.cc:32`, `tests/suction_test.cc:86`); `Median` x4 (`module_finder.cc:51`, `print_plan.cc:131`, `tests/motion_planner_test.cc:91`, `tests/rrt_test.cc:119`); `Collides` (`collision.cc:39`, `cycle_check.cc:24`); `Degrees` (`check_placement.cc:65`, `point_generator.cc:11`, and `pick_place.cc:53`, `tests/collision_test.cc:62`, `tests/ik_test.cc:170`); tests: `BodyDistance`, `RunScript`, `RunUntilConveyorStops` (`camera_cycle_test.cc:140/95/107`, `pick_place_test.cc:140/86/99`); `Angle`, `Distance`, `PlanningMargin`, `PlanningSelfMargin`, `HistoryFor`, `Apply` (`check_cycle_test.cc` and `planned_carry_test.cc`); `DistanceToSegment` (`arm_follower_test.cc:157`, `path_follow_test.cc:134`, also `path_timing.cc:516`); `Scaled` (`arm_follower_test.cc:139`, `path_timing_test.cc:61`, also `check_placement.cc:127`); `WithNoise` (`held_module_finder_test.cc:121`, `module_finder_test.cc:133`); `Named` (`layout_check_test.cc:516`, `layout_scene_test.cc:32`); `Load` (`layout_plan_test.cc:30`, `layout_rows_test.cc:29`).
Not in the old list (DIFFERS: old incomplete), also defined in 2+ src files: `PositiveFinite` x3 (`motion_planner.cc:37`, `ompl_search.cc:47`, `rrt.cc:40`), `IsPositiveAndFinite` (`conveyor.cc:15`, `profile.cc:9`), `ConfigText` (`layout_plan.cc:68`, `layout_replay.cc:219`), `Quat` (`bay_model.cc:74`, `layout_model.cc:96`), `Join` x3 (`check_placement.cc:154`, `layout_rows.cc:56`, `path_timing.cc:212`), `WeightsFor` (`ompl_search.cc:73`, `rrt.cc:22`), `PathLength` (`layout_replay.cc:49`, `rrt.cc:317`), `Merge`, `Lean`, `Project`, `Evaluate`, `Frame`, `Format` x5 in src, `Number`/`ParseNumber`, `Lines`, `ReadFile`, `StartsWith`, `V`, `Vec3`, `Row` (full list: output of the scan, 80 names; `Usage`/`Deg`/`Rad`/`Norm` are in 1 file each, SAME).

## 5. Sensor data only

`grep -n 'True[A-Z][A-Za-z]*(' src/*` (file: count: lines):
| file | n | lines | Verdict |
|---|---|---|---|
| `src/cell.h` | 3 | 312, 317, 321 | SAME |
| `src/cell.cc` | 5 | 341, 387, 391, 413, 414 | SAME |
| `src/conveyor.h` | 3 | 177, 180, 184 | SAME |
| `src/conveyor.cc` | 4 | 192, 196, 208, 209 | DIFFERS (old: 192, 196, 208) |
| `src/score.cc` | 5 | 24, 25, 45, 53, 64 | SAME |
| `src/trial_runner.cc` | 1 | 179 | SAME |
| `src/print_cell.cc` | 3 | 162, 163, 171 | SAME |
| `src/print_perception.cc` | 1 | 263 | SAME |
| `src/print_plan.cc` | 1 | 287 | SAME |
| `src/view_cell.cc` | 1 | 387 | SAME |
10 files; no other `src/` file. SAME. `grep -c 'True[A-Z][A-Za-z]*(' tests/* | grep -v ':0' | wc -l` = 11. SAME.

Truth that the `True...(` pattern misses (not in old): `Cell::true_scene()` returning `TrueScene` (`cell.h:308`); callers `grep -n 'true_scene()' src/*`: `print_cell.cc:101`, `trial_runner.cc:128`, `print_perception.cc:258`, `view_cell.cc:434`. None is a control file.

Raw body-pose reads (the three fields of the brief), `grep -nE '(->|\.)(xpos|xquat|xmat|xipos|ximat)\b|\b(xpos|xquat|xmat)\[' src/*` (file: n: lines): `collision.cc` 7: 431-435, 438, 445; `pallet.cc` 6: 142-144, 149-151; `suction.cc` 3: 19, 20, 24; `layout_check.cc` 3: 2024, 2026, 2027; `conveyor.cc` 2: 201, 204; `check_rig.cc` 2: 124, 127. 6 files, 23 lines. (New figure, the old counted wider fields.)
With the site/geom/cam fields as well (`xpos|xquat|xmat|site_xpos|site_xmat|geom_xpos|geom_xmat|cam_xpos|cam_xmat` as whole words, comments excluded): `layout_check.cc` 16, `collision.cc` 15, `suction.cc` 6, `pallet.cc` 6, `check_rig.cc` 4, `layout_scene.cc` 3 (40-42), `layout_rows.cc` 3 (75-77), `layout_replay.cc` 3 (976, 977, 979), `cycle_check.cc` 2 (297, 298), `conveyor.cc` 2, `camera_model.cc` 2 (49, 53), `arm.cc` 2 (91, 93), `view_layout.cc` 1 (246): 13 files. `pick_place.cc`, `motion_planner.cc`, `module_finder.cc`, `held_module_finder.cc`, `depth_camera.cc`: none (same grep). The old claim, SAME.

The old's wider regex re-run: `grep -nE 'd(ata)?(->|\.)(xpos|xquat|xmat|site_xpos|site_xmat|geom_xpos|geom_xmat|qpos|qvel|sensordata|contact|ncon|cvel|xipos|subtree_com|efc_force|qfrc_[a-z]*|actuator_force)|\b(xpos|...|xipos)\[' src/*` per file: `collision.cc` 32, `layout_check.cc` 18, `cell.cc` 13, `pallet.cc` 11, `cycle_check.cc` 8, `check_rig.cc` 8, `conveyor.cc` 7, `suction.cc` 6, `layout_scene.cc` 6, `print_pallet.cc` 5, `arm.cc` 5, `print_cell.cc` 4, `check_placement.cc` 4, `layout_rows.cc` 3, `layout_replay.cc` 3, `layout_model.cc` 2, 1 each: `suction.h`, `view_layout.cc`, `step_arm.cc`, `ik.cc`, `arm_follower.cc` = 21 files. SAME (all 21 counts).
Other claims:
| Claim | Command | Got | Verdict |
|---|---|---|---|
| `pick_place.cc` reads via `cell_.data()->time`, `JointAngles`, `conveyor.SensorPosition`, `Moving` at 739-742, 771 | `grep -n 'SensorPosition\|conveyor.*Moving(\|data()->time' src/pick_place.cc` | 619, 688, 740, 742, 771 (and many `->time`) | SAME |
| `conveyor.cc:185,189` encoder via `sensordata` | `grep -n sensordata src/*.cc` | `conveyor.cc:185,189` only | SAME |
| `arm_follower.cc:204` reads `qfrc_bias` | `grep -n qfrc_bias src/*.cc` | :192 (comment), :204 | SAME |
| `camera_model.cc:49,53` reads `cam_xpos/cam_xmat`; callers of `CameraModelFromMujoco` | `grep -n CameraModelFromMujoco src/*.cc` | `cell.cc:160,166,347,363`, `check_placement.cc:422,628`, `check_rig.cc:135` | SAME |
| "no command that checks it" (implied by "Proposed rule") | `grep -n 'NoTruth' -B3 -A22 tests/camera_cycle_test.cc` | `camera_cycle_test.cc:974-1002`: test `NoTruth.TheScriptsSourceReadsOnlyWhatACellKnows` reads `src/pick_place.cc` (`SCRIPT_SOURCE`, `BUILD.bazel` data `//:src/pick_place.cc`) and fails on `True`, `true_`, `xpos`, `xquat`, `xmat`, `ModulePose`, `PalletPose`, `ModuleGlass`, `suction()`, `HeldPose`, `contact`, `DepthCamera()`, `module_state`, `qpos`, `TipPose(` | DIFFERS (old: nothing checks the rule; one file, `pick_place.cc`, is checked by a test) |
| offenders today by the allowlist rule: none | `grep -ln 'True[A-Z][A-Za-z]*(' src/* \| grep -v 'cell\.\|conveyor\.\|score\.cc\|trial_runner\.cc\|view_cell\.cc\|print_'` | prints nothing | SAME |
| role of each raw read (legit / write / scratch / truth) | judgement by reading in context | not re-read line by line in this run | see Could not decide |

## 6. Tests and what they reach

| Claim | Command | Got | Verdict |
|---|---|---|---|
| `src/` files reached by some test | `bazel query "kind('source file', deps(tests(//...)))" --output=label \| grep '^//:src/' \| wc -l` | 75 of 99 | SAME |
| the 24 not reached | `ls src \| sed 's\|^\|//:src/\|' \| sort \| comm -13 reached.txt -` | `check_layout.cc check_placement.cc check_placement.h gen_bay.cc plan_layout.cc print_bay.cc print_cell.cc print_check.cc print_depth.cc print_pallet.cc print_perception.cc print_plan.cc print_plane_fit.cc print_points.cc print_profile.cc print_rrt.cc run_cell.cc run_gate.cc step_arm.cc view_cell.cc view_layout.cc viewer.cc viewer_lib.cc viewer_lib.h` = 24 (20 mains incl. `viewer.cc`, `viewer_lib.{cc,h}`, `check_placement.{cc,h}`) | SAME |
| `check_placement.*` belongs to no library, only `print_check` (`BUILD.bazel:1357-1358`); `pick_place.h:420` mentions in comment | `grep -n 'check_placement' BUILD.bazel src/pick_place.h` | `BUILD.bazel:1349` (comment), 1357, 1358; `pick_place.h:420` | SAME |
| no `cc_binary` reachable from a test | `bazel query 'kind(cc_binary, deps(tests(//...))) intersect //:*' \| wc -l` | 0 | SAME |
| no binary in a test's `data` | explicit `data`, `args`, `env` of every `cc_test` from `rules.jsonl` | `data` holds `//:arm_model`, `//:arm_meshes`, Menagerie UR10e files, `models/ layouts/ plans/` and, for `camera_cycle_test`, the source file `//:src/pick_place.cc`; no binary | DIFFERS (old: data only models/layouts/plans; also source file and external files) |
| no `popen`/`system(` | `grep -n 'popen\|system(' tests/*.cc` | nothing | SAME |
| tests name binaries only in messages | `grep -n 'gen_bay' tests/*.cc` | `bay_model_test.cc:114` | SAME |
| CI runs only `bazel test` | `grep -n 'bazel' .github/workflows/ci.yml` | `bazel test` at :67, :72, :78 (+ setup-bazel :52) | SAME |

Every `cc_binary`, who runs it (`bazel query "rdeps(//..., $b, 1) except $b"` for each of the 27): all 27 give an empty set, so no rule (test, genrule, other binary) depends on any binary; nothing runs them under Bazel. Main source of each: section 10. Of the 27, 8 are `viewer*` (need a display; `viewer_cell` and `view_cell` too). Verdict SAME as the old ("none run by a test"), now checked for all 27 by a command.
`genrule` count: 1 (`arm_meshes`, `BUILD.bazel:195`), a copy of KUKA meshes; no genrule runs a binary.

| Claim | Command | Got | Verdict |
|---|---|---|---|
| headers with no `tests/<name>_test.cc` | `for h in src/*.h; do [ -f tests/$(basename $h .h)_test.cc ] \|\| echo $h; done` | `arm_catalog camera_model check_placement check_rig cycle_check layout mj_handles portable_random rrt_worlds score viewer_lib` (11) | SAME |
| tests with no same-name header | `for t in tests/*_test.cc; do [ -f src/$(basename $t _test.cc).h ] \|\| echo $t; done` | `camera_cycle check_camera_cell check_cycle gate locate_physics locating_gate path_follow plan_replay planned_carry step` (10) | SAME |
| sizes of the 42 tests | python over `rules.jsonl` (`size` attr, explicit) | small 22, medium 16, large 4; lists exactly as old (large: `camera_cycle gate module_finder motion_planner`) | SAME |
| `gate_test` `shard_count = 4`, `tags = ["gate"]`, `motion_planner_test` `shard_count = 4`; no `timeout`, `flaky`, `local` | same | only those (check_cycle tests carry `args = --gtest_filter=...`) | SAME |
| committed-file tests: `bay_model_test.cc:97` | `grep -n 'CommittedFileIsWhatTheGeneratorPrints' tests/bay_model_test.cc` | :97 | SAME |
| `plan_replay_test.cc` `Committed.*` 474-686, `PlanDir()` :36 (old: :36), sha :446, round trip :502-504, `VerifyPlan` :477, tamper :686 | `grep -n 'PlanDir\|Sha256OfFile(c\|JsonRoundTrip\|VerifyPlan(\|TamperingIsFound' tests/plan_replay_test.cc` | `PlanDir` defined :35, :446 sha, :477, :502-504, :686 | SAME (:35 for :36) |
| `pose_tolerance_deg = 0.2` `layout_plan.h:392` | `grep -n pose_tolerance_deg src/layout_plan.h` | :392 | SAME |
| tests reading layouts: `layout_rows_test.cc:32`, `layout_scene_test.cc:35`, `layout_plan_test.cc:27`, `layout_model_test.cc:53`, `layout_check_test.cc` 40-1143 | `grep -n 'LoadLayout\|LAYOUT_DIR' tests/*.cc` | :32, :35, :27 (path), :52-53 (`LayoutPath`), `layout_check_test.cc:40` and :1143 | SAME |
| only `bay.xml` compared byte for byte | `grep -n 'committed' tests/*.cc` | `bay_model_test.cc` and `plan_replay_test.cc` (plans) | SAME |

## 7. Headers' opening comments and file sizes

| Claim | Command | Got | Verdict |
|---|---|---|---|
| opening-comment lines per header: min 6, median 39, max 275, none < 5 | python: count `//` lines before the first non-blank non-`//` line of each `src/*.h` | 6, 39.0, 275, none < 5 | SAME |
| fewest | same | `mj_handles` 6, `arm` 9, `point_generator` 9, `joint_move` 11, `check_rig` 12, `pallet` 12, `score` 12, `arm_catalog` 18, `suction` 19 | SAME |
| most | same | `layout_check` 275, `pick_place` 240, `motion_planner` 124, `path_timing` 123, `layout_plan` 121, `layout_scene` 119, `ompl_search` 112, `collision` 105 | SAME |
| `.cc` over 1,000 | `wc -l src/*.cc \| awk '$1>1000'` | `layout_check` 2610, `pick_place` 1911, `layout_replay` 1812, `layout_plan` 1462, `rectangle_fit` 1252 (5) | SAME |
| headers over 600 | `wc -l src/*.h \| awk '$1>600'` | `pick_place.h` 1173, `bay_hardware.h` 976 | SAME |
| tests over 1,000 | `wc -l tests/* \| awk '$1>1000'` | `check_cycle` 1654, `layout_model` 1247, `rectangle_fit` 1235, `layout_check` 1185, `ik` 1141, `planned_carry` 1082, `collision` 1016, `camera_cycle` 1004, `motion_planner` 1004 (9) | SAME |
| AGENTS says 240 of 1,173 lines of `pick_place.h` | above | 240 / 1173 | SAME |

## 8. Determinism and platform

Command (src, then tests): `grep -nE 'std::rand|\bsrand|random_device|mt19937|<random>|_distribution|std::chrono|\btime\(|getenv|std::thread|std::async|omp |hardware_concurrency|steady_clock|system_clock|high_resolution|clock_gettime|gettimeofday' src/* tests/*` (src output in `out/audit/det_src.txt`, 155 lines).

| Claim | Got | Verdict |
|---|---|---|
| `std::rand`, `srand`, `random_device`, `std::async`, OpenMP, `time(`, `system_clock`, `high_resolution_clock` | none in `src/` and `tests/` | SAME |
| clocks only `std::chrono::steady_clock` | yes in `src/` and `tests/` | SAME |
| `std::mt19937` seeded by caller, src | `rrt.cc:441,514`, `plane_fit.cc:132`, `rectangle_fit.cc:860`, `depth_camera.cc:50,172`, `cell.cc:187`, `check_rig.cc:57`, `conveyor.cc:59`/`conveyor.h:190`, `layout_plan.cc:570`, `layout_check.cc:279` (`20261124u`), `print_points.cc:30`, `print_plane_fit.cc:67`, `point_generator.cc` (passed in) | SAME |
| tests seed their own with fixed seeds | `grep -n 'mt19937' tests/*`: 11 files (`cell_test ik_test collision_test module_finder_test plane_fit_test path_timing_test point_generator_test profile_test rectangle_fit_test ompl_search_test rrt_test`), every seed a literal or loop variable `seed` | SAME |
| via `portable_random.h` (`UniformUnit`, `UniformReal`, `UniformIndex`, normal) | `grep -n 'portable_random' src/*`: `layout_check.cc:32`, `layout_plan.cc:25`, `rectangle_fit.cc:13`; `depth_camera.cc:23` own Box-Muller; `cell.cc:45-47` own `Uniform` | SAME |
| std distributions in src: `rrt.cc:442` (used 460, 462), `:522-524`; `plane_fit.cc:133`; `point_generator.cc:21,22,37,52,60-63`; `conveyor.cc:123` | `grep -n '_distribution' src/*.cc` | exactly those | SAME |
| tests drawing through std distributions: `ik_test.cc:189,195,952`, `collision_test.cc:206...`, `rrt_test.cc:209`, `module_finder_test.cc:374`, `cell_test.cc:593` | `grep -n '_distribution' tests/*.cc` | `ik_test.cc:189,195,952`; `collision_test.cc:206,724,737,891,893,941`; `rrt_test.cc:209`; `module_finder_test.cc:374`; `path_timing_test.cc:681`. `cell_test.cc:593` is `std::mt19937 random(5)` used raw (`random()`, :597): no distribution | DIFFERS (old: `cell_test.cc:593` is a std distribution - it is not; `path_timing_test.cc:681` missing) |
| `layout_plan.cc` plans with `max_seconds = 1e12` at `:644,683` | `grep -n 'max_seconds' src/layout_plan.cc` | 644, 683 | SAME |
| wall clock in decisions: `rrt.cc:451,517` | `grep -n 'SecondsSince(began)' src/rrt.cc` | 451 (`> max_seconds`), 517 | SAME |
| defaults 5 s / 1 s `rrt.h:240,348` | `grep -n 'max_seconds = ' src/rrt.h` | 240 = 5.0, 348 = 1.0 | SAME |
| `motion_planner.cc:334,373`; `ompl_search.cc:417` | grep | 334 (`left = time_budget - ...`), 373; `ompl_search.cc:417` | SAME |
| OMPL `SeededSampler` `ompl_search.cc:83-111`; planner table 48-51; clock-seeded `BITstar`, `AORRTC`, `InformedRRTstar` | `grep -n 'SeededSampler\|BITstar\|AORRTC\|InformedRRTstar' src/ompl_search.cc` | class at :85; table rows :60-61 (`{"BITstar", false, false}`, `{"AORRTC", false, true}`, `{"InformedRRTstar", false, false}`), comment :51 | SAME in substance (table is :56-61, not 48-51); "do not repeat" not re-verified, see Could not decide |
| threads `layout_plan.cc:21,722-724`, results in order `:726-727` | `grep -n 'std::thread' src/layout_plan.cc`; `sed -n 726,728p` | 21, 722, 724; "Into the blocks, in order" at :726 | SAME |
| `trial_runner.cc:14,276-278`, `run_gate.cc:60` `hardware_concurrency()`, `gate_test.cc:87`, `ompl_search_test.cc:289-293` | grep | 14, 276, 278; 60; 87; 289, 293 | SAME |
| `getenv` names | `grep -n getenv src/*.cc` | `LAYOUT_DIR` (`check_layout.cc:38`, `plan_layout.cc:53`, `view_layout.cc:107,360`), `PLAN_DIR` (`plan_layout.cc:133`, `view_layout.cc:104`), `MODELS_DIR` `layout_model.cc:447`, `UR10E_XML` :448, `BUILD_WORKING_DIRECTORY` (`run_cell.cc:408`, `run_gate.cc:41`), `PLAN_DEBUG` (`layout_plan.cc:583,659`), `PLAN_SKIP_VERIFY` :1334 | SAME |
| "planned output independent of thread count" | `grep -n 'threads' src/layout_plan.cc`, header comment :6-8 claims it | claim only in a comment; no test of 1 thread vs N found by `grep -n 'threads' tests/layout_plan_test.cc tests/plan_replay_test.cc` (not searched in depth) | not decided |
| wall-clock bounds in tests (AGENTS: "Do not bound a wall-clock time in a test") | `grep -n 'EXPECT_LT(seconds\|report.seconds' tests/rrt_test.cc` | `rrt_test.cc:373` `EXPECT_LT(seconds, 2.0)` (wall), `:391` `EXPECT_GE(report.seconds, 0.05)`, `:392` `EXPECT_LT(report.seconds, 0.5)` | new finding, not in old |

## 9. Units and frames

| Claim | Command | Got | Verdict |
|---|---|---|---|
| `_deg`/`_mm` uses | `grep -ohE '\b[a-zA-Z_0-9]*(_deg\|_mm)\b' src/*.h src/*.cc \| wc -l` | 450 | SAME |
| distinct identifiers | same `\| sort -u \| wc -l` | 160 | SAME |
| in 25 files | `grep -cE '\b[a-zA-Z_0-9]*(_deg\|_mm)\b' src/*.h src/*.cc \| grep -v ':0' \| wc -l` | 28 | DIFFERS (old: 25) |
| top files | `grep -oE ... \| awk -F: '{print $1}' \| sort \| uniq -c \| sort -rn` | `print_bay.cc` 73, `run_cell.cc` 56, `trial_runner.cc` 52, `layout_check.cc` 47, `print_check.cc` 32 (next `layout_replay.cc` 24) | SAME |
| header lines `layout.h:195,203,205,224,296,128,151` | `grep -n 'base_rpy_deg\|wait_joints_deg\|stow_joints_deg\|yaw_deg\|frame_yaw_deg\|joint_margin_deg\|holds_within_deg' src/layout.h` | 195, 203, 205, 224, 296, 128, 151 | SAME |
| `layout_check.h` `work_deg:361`, `approach_leg_deg:372`, `joint_margin_deg:328,409` | grep | same | SAME |
| `layout_plan.h` `pose_tolerance_deg:392`, `margin_deg:414` | grep | same | SAME |
| `arm_catalog.h:44-45,74-75` `joint_lo_deg`; `layout_model.h:93` `joints_deg`; `layout_scene.h:248,309`; `trial_runner.h:70-81`; `bay_hardware.h:728` | grep | `arm_catalog.h:44, 74, 95` (joint_lo_deg, joint_hi_deg adjacent), `layout_model.h:93`, `layout_scene.h:248,309`, `trial_runner.h:70,71,74,75,80,81`, `bay_hardware.h:728` | SAME |
| `layout_check.cc:112` `ToDeg` | `grep -n 'ToDeg' src/layout_check.cc` | :112 | SAME |
| `_m` 21 distinct (187 uses), `_s` 32 (275), `_seconds` 14 (111), `_rad` 0, `_mps` 0 | `grep -ohE "\b[a-zA-Z_0-9]*_m\b" src/*.h src/*.cc \| wc -l` and `\| sort -u \| wc -l` (same with `_s`, `_seconds`, `_rad`, `_mps`) | 187/21, 275/32, 111/14, 0, 0 | SAME |
| `Config = std::array<double, 7>` `layout_plan.h:146` | `grep -n 'using Config' src/layout_plan.h` | :146 | SAME |
| `path_timing.h:123` "Units are radians and seconds"; `max_deviation` doc `:138-141` | `grep -n 'Units are radians' src/path_timing.h` | :123; the `max_deviation` comment sits at :136-141 | SAME |
| `layout_replay.cc:36-40,86-92`, `options.max_deviation = f.max_deviation` `:91` | `grep -n 'max_deviation' src/layout_replay.cc` | :91 (inside :86-92) and `ToConfig` :36-40 | SAME |
| "whether the weights are applied inside the deviation test is not read here" | `grep -n 'weights' src/path_timing.h src/path_timing.cc \| wc -l` | 0: `path_timing.*` has no `weights`, so the deviation bound is the plain Euclidean norm of the 7-vector (metres of the track mixed with radians) | DECIDED: unweighted (old: undecided) |

## 10. Programs

`bazel query 'kind(cc_binary, //...)'`: 27. `grep -ln '^int main' src/*.cc | wc -l`: 20. SAME. Line counts `wc -l src/<main>.cc`: `run_cell` 804, `run_gate` 156, `print_plan` 495, `print_check` 432, `print_perception` 389, `print_depth` 225, `print_rrt` 332, `print_pallet` 183, `print_cell` 184, `print_points` 64, `print_plane_fit` 103, `print_profile` 86, `step_arm` 50, `print_bay` 283, `gen_bay` 15, `check_layout` 276, `plan_layout` 500, `view_cell` 530, `view_layout` 431, `viewer` 96: all SAME.
Flags: `grep -o '"--[a-z0-9-]*"' src/<main>.cc | sort -u` per program: `run_cell` 21 flags as old, `run_gate` (`modules save-failures seed threads trials`), `print_plan` 18 (`both-ways corner first-seed module ompl-planner ompl-queries ompl-range other-way over-slot plan-seeds planner pretty resample resolution return seeds slot summary`), `print_check` 12 (old omits `--calibration`), `print_perception` 9, `print_depth` 7, `print_rrt` 4 (`nn shorten-sweep stats sweep`), `print_bay` (`capture summary`), `check_layout` (`checks list pretty steps summary`), `plan_layout` (`budgets first-step last-step margins out pretty summary threads why`), `view_layout` (`at from-time modules no-overlay play rails speed state to-time`), `view_cell` 28 (old: "~25"), `print_points` none, `print_profile`/`print_plane_fit` `--pretty`. SAME in substance; DIFFERS only: `print_check --calibration` not in old list, `view_cell` 28 flags.
Display: `grep -n GLFW` -> only `viewer_lib.cc`; binaries that link `viewer_lib` (`BUILD.bazel:247,321,378,437,646,675,773,1738`): `viewer*`, `view_cell`, `viewer_layout`. SAME.
`print_check` carries `check_placement.cc/.h` in own srcs: `rules.jsonl` srcs `['//:src/check_placement.cc','//:src/check_placement.h','//:src/print_check.cc']`. SAME.
Logic-carrying mains (`wc -l` above), thin mains: SAME (judgement; sizes given).

## 11. Outside libraries

`MODULE.bazel` (`grep -n 'bazel_dep' MODULE.bazel`; 23 lines):
| dep | line | version | Verdict |
|---|---|---|---|
| `rules_cc` | 9 | 0.2.28 | SAME |
| `mujoco` | 12 | 4.0.0 | SAME |
| `eigen` | 16 | 5.0.1.bcr.2 | SAME |
| `glfw` | 19 | 3.4.0.bcr.4 | SAME |
| `nlohmann_json` | 22 | 3.12.0.bcr.2 | SAME |
| `googletest` | 25 | 1.18.0.bcr.1, `dev_dependency = True` | SAME |
| 17 `boost.*` | 121-137 | 1.90.0.bcr.1, `boost.graph` 1.90.0.bcr.2 (`grep -c 'bazel_dep(name = "boost\.' MODULE.bazel` = 17): algorithm assert concept_check config dynamic_bitset foreach functional graph math numeric_odeint property_map random range serialization smart_ptr thread type_traits | SAME |

Repository rules (`grep -n 'use_repo_rule\|http_archive\|menagerie_model(\|kuka_description(' MODULE.bazel bazel/*.bzl`; no `archive_override`, `git_override`, `local_path_override`, `use_extension`):
| rule | where | what | Verdict |
|---|---|---|---|
| `menagerie_model` `menagerie_ur10e` | `MODULE.bazel:31-33`, rule in `bazel/menagerie.bzl` (`ctx.download` :22, base `https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{commit}/{model}` :50) | commit `370c43965323c0944b433b202aac5058b5f907a9`, sha256 for each file: `sed -n 33,70p MODULE.bazel \| grep -c '": "[0-9a-f]\{64\}"'` = 23 | DIFFERS (old: 25 files) |
| `kuka_description` `kuka_kr210_r2700_2` | `MODULE.bazel:71-73`, `bazel/kuka.bzl` (`ctx.download` :30, base `https://raw.githubusercontent.com/kroshu/kuka_robot_descriptions/{commit}` :58) | commit `d2fa4377b77b7370cdcf6e52b430fc514a0eef19`, 15 files with sha256 (`sed -n 73,95p MODULE.bazel \| grep -c ...` = 15) | SAME |
| `http_archive` `ompl` | `MODULE.bazel:107-115` | `https://github.com/ompl/ompl/releases/download/2.0.2/ompl-2.0.2.tar.gz`, sha256 `b9155e243e555b04b01f8e4ff7e21a48582edbab9c300a611289655778bcd29d`, `strip_prefix ompl-2.0.2`, build file `bazel/ompl.BUILD.bazel` | SAME |

Targets listing a library directly in `deps` (python over `rules.jsonl` explicit `deps`; the old command `bazel query "attr(deps, '@ompl', //...)"` returns 0 because Bazel's canonical name is `@@ompl+`; use `@ompl//:ompl`) and files including (section 3):
| label | targets listing | files including (src / tests) | targets with no include in own srcs/hdrs | Verdict |
|---|---|---|---|---|
| `@eigen//:eigen` | 78 (`bazel query "attr(deps, '@eigen', //...)" \| grep -c '^//'`) | 95 (64 / 31) | 3: `bay_model_test`, `print_plane_fit`, `print_points` | SAME |
| `@mujoco//:mujoco` | 67 | 70 (41 / 29) | none | SAME |
| `@googletest//:gtest_main` 41 + `//:gtest` 1 | 42 | 41 (0 / 41) | none | SAME |
| `@nlohmann_json//:json` | 4 (`layout`, `layout_model_test`, `layout_plan`, `plan_replay_test`) | 3 (2 / 1) | `plan_replay_test` | SAME |
| `@ompl//:ompl` | 1 (`ompl_search`; `:ompl` alias at `BUILD.bazel:1585`) | 1 (`ompl_search.cc`) | none | SAME |
| `@glfw//:glfw` | 1 (`viewer_lib`) | 1 (`viewer_lib.cc`) | none | SAME |
Also in explicit `deps` of 69 targets, though `grep -c link_extra_lib BUILD.bazel` = 0: `@rules_cc//:link_extra_lib` (a rules_cc default; not ours). New note.

| Claim | Command | Got | Verdict |
|---|---|---|---|
| `bazel mod graph`, `mod deps` fail (pip extension) | `bazel mod graph 2>&1 \| grep -n ERROR` | `rules_python+/python/private/pypi/hub_builder.bzl:405:17` traceback, "error evaluating module extension @@rules_python+//python/extensions:pip.bzl%pip", "Results may be incomplete as 1 extension failed" | SAME |
| Boost packages under OMPL | `bazel query 'deps(@ompl//:ompl)' --output=package 2>/dev/null \| grep -i boost \| sort -u \| wc -l` | 81 (17 apparent + 64 `@@boost.*+`) | SAME |
| 17 direct deps of OMPL's target | `bazel query 'deps(@ompl//:ompl, 1)' \| grep -c boost`; `bazel/ompl.BUILD.bazel:107-123` | 17; deps lines 107-123 | SAME |
| `boost/` includes in OMPL source | `O=$(bazel info output_base)/external/+http_archive+ompl; grep -rhoE '#\s*include\s*[<"]boost/[a-z_0-9]+' $O/src/ompl \| sed 's/.*boost\///' \| sort \| uniq -c \| sort -rn` | graph 59, math 40, foreach 12, serialization 9, property_map 8, concept_check 8, range 7, archive 7, scoped_ptr 6, random 6, pending 6, numeric 6, functional 3, version 1, type_traits 1, thread 1, dynamic_bitset 1, assert 1, algorithm 1 | SAME |
| 15 of the 17 reachable from the other 16; not `graph`, `numeric_odeint` | for each m: `bazel query "deps(set(<other 16 as @boost.X>))" --output=package \| grep -c "^@@boost.$m+//"` | 1 for 15; 0 for `graph` and `numeric_odeint` | SAME |

## Differences (all figures that differ from the old audit)

1. Section 1: `# BEGIN` sections are 29, not 28 (the old table itself lists 29). Per-section counts, 93 inside, 17 outside: SAME.
2. Section 1: BUILD diff since 68a1199 also removed the `layout_moves` section (old only lists additions).
3. Section 0 / 4(b) / 4(d): stand-in arm geometry. The old said every stand-in caller of `mj_geomDistance` measures UR10e capsules unmended. `models/cell.xml` includes `models/arm.xml` (KUKA KR210, meshes only); the only capsule in `models/` is `suction_scene.xml:57`. UR10e capsules are in the new cell's model and `step_test`.
4. Section 3: new-cell code includes stand-in headers directly in 5 lines (`layout_plan.h:137`, `layout_plan.cc:26`, `layout_replay.cc:21,22`, `view_layout.cc:69`), not one. No S to L and no B to S/L include.
5. Section 3: L to B includes (11 src lines, 4 test lines) listed; `layout_check_test`, `layout_model_test` mix L and B; `locating_gate_test` mixes B and S. `portable_random.h` is also included by `rectangle_fit.cc:13`.
6. Section 4(a): the shared normal-equation lines are `ik.cc:191` and `layout_check.cc:370`, not 178-184 and 381-383.
7. Section 4(b): `layout_model.cc` rewrite loop ends at :732 (old :731).
8. Section 4(c): `Moving/Obstacles` users are more: `layout_check.cc` 13 sites (451, 457, 829, 840, 841, 846, 1158, 1487, 1488, 2060, 2061, 2169, 2171, 2173), `layout_plan.cc:161,167,774`; `layout_replay.cc:775-776` is the `ArmCache` struct (the builds are 735-736).
9. Section 4(d): model line numbers are `pallet.xml:74`, `check_scene.xml:14`, `conveyor_scene.xml:9`, `suction_scene.xml:14`; `tests/check_cycle_test.cc:119` and `locate_physics_test.cc:73` no longer match the real-module numbers; `view_cell.cc` has no pi digits; `std::numbers::pi` is in 25 `src/` files (40 lines), not 3.
10. Section 4(f): `PrintMargins` is 9 vs 114 lines (old 8 vs 110); `PrintTable` bodies differ in 10 lines by my diff of 61-84 vs 86-107 (old 8; range-dependent).
11. Section 4(h): the old list is a subset: more same-name helpers (`PositiveFinite` x3, `IsPositiveAndFinite`, `ConfigText`, `Quat`, `Join` x3, `WeightsFor`, `PathLength`, ...).
12. Section 5: `conveyor.cc` has a fourth `True...(` line (:209); `Cell::true_scene()` (`cell.h:308`, callers `print_cell.cc:101`, `trial_runner.cc:128`, `print_perception.cc:258`, `view_cell.cc:434`) exposes truth without a `True` name pattern.
13. Section 5 / 6: a test already enforces the "sensor data only" rule for one file: `tests/camera_cycle_test.cc:974-1002` over `src/pick_place.cc` (`BUILD.bazel` data `//:src/pick_place.cc`). The old says no command checks it, and that test data is only models, layouts and plans.
14. Section 8: `tests/cell_test.cc:593` draws raw `mt19937` output, not a std distribution; `tests/path_timing_test.cc:681` (`std::normal_distribution`) is missing from the old list. Tests bound wall-clock time in `tests/rrt_test.cc:373,391,392`, against AGENTS.md.
15. Section 9: `_deg`/`_mm` identifiers occur in 28 files, not 25. The planner's `max_deviation` bound has no weights (`grep weights src/path_timing.*` = 0): decided (old: undecided).
16. Section 11: `menagerie_ur10e` pins 23 files, not 25.
17. Section 4(e): the old "12 files print their own usage: string" is 19 files by `grep -ln 'usage:' src/*.cc` (includes comments; figure not decided).

Figures re-run: about 330 rows/entries; 17 differences (each may cover several figures; about 40 single figures differ), the rest SAME.

## Could not decide

- Section 1: context-column rule counts at 68a1199 (98 rules, 37 tests, 36 `cc_library`, 35 `cc_test`): needs a Bazel query of the old commit; file, line and BUILD line counts at 68a1199 were re-run (SAME).
- Section 4(e): the figure "12 files print their own `usage:` string": a by-hand figure; the command counts 19 files with the word, mixing comments and strings.
- Section 5: the old's classification of each raw read as legitimate / write / scratch / truth (arm.cc:76-93 etc.) rests on reading code in context; I re-ran every grep that gives the lines but did not re-read each site. Whether `camera_model.cc:49-53` "starts calibration from truth" stands: its callers are listed (`cell.cc:160,166,347,363`, `check_placement.cc:422,628`, `check_rig.cc:135`); whether the camera's pose is perturbed afterwards is not decided here.
- Section 8: that OMPL's `BITstar`, `AORRTC`, `InformedRRTstar` do not repeat from a seed (`ompl_search.cc:56-61` table says so; no build or run); that `plan_layout` output does not depend on `--threads` (a comment claims it, no run).
- Section 4(b): whether the capsule-versus-box error matters in practice for the layout checks: no run.
- Nothing was built or run; no figure rests on a test result.
