# History and reasons for the 17 decisions at 940a01b: checked (history, run 013-r2)

Everything below was read in an export of `940a01b` (`git archive 940a01b`) and in `git log 940a01b`.
Dates are `git show -s --format='%h %ad' --date=short`. "Old" is `notes/r1-history.md`.
`project.md` is the project brief the old note calls "the brief"; its path in the old note
(`/root/org-runs/013-design-principles-and-r1/project.md`) does not exist; the file is
`/root/org-runs/013-design-principles-and-r2/project.md` (330 lines). It is outside the repository.
A quotation that spans two source lines is given with both line numbers; `grep -F` on one line cannot find it.
`git rev-list --count 940a01b` = 28 commits (old note says 29: DIFFERS), 2026-11-19 to 2026-11-25.

Commit table (all checked): 8adf3bd 11-19 Toolchain; 83970cc 11-19 Cell; 12d1127 11-20 AGENTS.md; 5600068 11-20
portable random; ae3bb74 11-20 no wall-clock; dd03149 11-20 Gate; 5d8fb21 11-22 agent config; 1d2c098 11-22 CI six jobs;
014ba3a 11-22 CI by hand; 4c5bf74 11-23 AGENTS.md rewritten; 13678be and fa4d595 11-24 OMPL report; a091cc2 11-23 merge
of the report (docs/ompl-integration.md only, 1 file); 9397bdd 11-24 OMPL on main; 1213319 11-24 bay parts; 68a1199 11-25
run 011; 940a01b 11-25 run 012. All SAME as old.

---

## 1. Bazel with bzlmod and one pinned version

**True today.**
- `.bazelversion:1` = `9.3.0`. `MODULE.bazel:3-6` `module(name = "solar_bay_assembly", version = "0.0.0")` (old said 3-4: DIFFERS slightly).
- `MODULE.bazel:9,12,16,19,22,25` `bazel_dep` rules_cc 0.2.28, mujoco 4.0.0, eigen, glfw, nlohmann_json (`:22`), googletest (`:25`, dev). Seventeen `boost.*` `bazel_dep`s at `MODULE.bazel:121-137` (old: "from about line 118": DIFFERS, 121).
- `MODULE.bazel.lock` committed, 709 lines at `940a01b` (SAME). Lines per commit: 463 at 8adf3bd, 708 at 9397bdd, 709 at 68a1199 (all SAME; `git log --follow` shows exactly these three commits).
- `README.md:40-41`: "The build tool is Bazel, run through Bazelisk, which fetches the version named / in `.bazelversion`." (spans two lines). `.github/workflows/ci.yml:47` "Installs Bazelisk, which reads .bazelversion" (SAME).
- project.md:10-11 "built with Bazel 9.3.0 (bzlmod, / `.bazelversion`)"; project.md:167-170 `bazel mod deps`/`graph` exit 2, "a flaw of the `mujoco` module", check command `bazel build --nobuild --lockfile_mode=error --config=opt //...` (SAME).

**When.** `8adf3bd 2026-11-19 Toolchain: Bazel build, MuJoCo stepping a UR10e from C++, viewer, tests and CI` added `.bazelversion`, `MODULE.bazel`, `MODULE.bazel.lock`, `BUILD.bazel`, `bazel/BUILD.bazel`, `.bazelrc`, `ci.yml` (`git log --diff-filter=A`). SAME. Lock changed at `9397bdd` and `68a1199`. The "not chosen yet" sentence is `dd65a4d:README.md:16` ("Empty repository. Language, planning stack, and robot model are not chosen yet."), not the commit message (`dd65a4d 2026-11-19 Initial commit: README and .gitignore`): old note attributes it to the commit (DIFFERS).

**Reason.** no reason recorded in the repository for Bazel, for bzlmod or for the pin (body of 8adf3bd is its subject only). For dependencies only: `docs/ompl-integration.md:130-131` "the Boost modules are pinned by the hashes of their / registry files in `MODULE.bazel.lock` and by the integrity values in the registry [S8]" (old said `:132`, with "..." joining line 130: DIFFERS in line); properties table row `docs/ompl-integration.md:1230` "The same OMPL on every machine and on CI | numbers that can be compared between machines" (SAME). "Likely: one version so results compare" is the old note's guess; not in the repository.

**Alternatives mentioned.** None for Bazel. For bringing a non-registry library in: options O1 to O8 (`docs/ompl-integration.md:47` "## Options", O1 at `:76`, O8 at `:644`); approaches A to **H** (not A to F: old DIFFERS) at `docs/ompl-integration.md:1245-1252` (table), section `:1241` "### The approaches".

**Elsewhere.** `docs/ompl-integration.md:1449` "| Bazel | 9.3.0, chosen by `.bazelversion`, through Bazelisk |" (SAME). `AGENTS.md:41` "wait for the lock" is Bazel's command lock; `AGENTS.md` never mentions `MODULE.bazel.lock` (confirmed by grep). "+245 lines" at `docs/ompl-integration.md:733` is the spike branch's glue table (`| MODULE.bazel.lock | 245 |`); 708-463 = 245 happens to agree.

---

## 2. External code fetched by hash, with `base_url`; registry modules otherwise

**True today.**
- `bazel/menagerie.bzl:3-4` "Menagerie as a whole is several hundred megabytes, so only the files of the / one model are downloaded, each checked against a SHA-256 hash." (spans 3-4); `:20` `ctx.attr.base_url.format(...)`; `:21-26` download with `sha256`; `:49-52` attr `base_url`, doc "Where the files are served from. Point it at another host to self-host." (SAME).
- `bazel/kuka.bzl:3-6` same idea; `:22` base_url; `:26-34` download; `:57-60` attr (old `:26-33`, `:57`: close).
- `MODULE.bazel:27-31` (menagerie, "downloaded file by file at a pinned commit. Not a Bazel / module ... To self-host, copy the files elsewhere and set base_url."), `:65-71` KUKA; each lists every file with SHA-256 and one `commit` (SAME).
- OMPL: `MODULE.bazel:97-115` (old: 95-114: DIFFERS), comment and `http_archive(name = "ompl", sha256 = "b9155e24...", urls = [...])` at `:109-115`; "To self-host, copy the archive elsewhere and put that / address first in `urls`." at `:105-106` (SAME). Build file `bazel/ompl.BUILD.bazel`.
- Registry: `MODULE.bazel:9-25`, Boost `:121-137`.
- The convention is only in project.md:194-198 ("`bazel_dep` from the Bazel Central Registry where the library is there; otherwise a repository rule in `bazel/` that downloads pinned files by SHA-256 and has a `base_url` attribute for self-hosting (`bazel/kuka.bzl`, `bazel/menagerie.bzl`)"); not in `AGENTS.md` or `README.md` (grep for base_url, SHA-256, registry finds nothing there). SAME.

**When.** `menagerie.bzl` 8adf3bd 2026-11-19 SAME. `kuka.bzl` 83970cc 2026-11-19 SAME. `ompl.BUILD.bazel` and the `http_archive` on main: 9397bdd 2026-11-24 SAME (`git log -S'http_archive' -- MODULE.bazel`). Spike: `b90da7c 2026-11-23 Build: OMPL 2.0.2 as a library built by Bazel, for comparison only` (only on `origin/spike/ompl-native`); `1d68640` same subject, 11-23, on `origin/org/004-ompl-on-main-and-the`, and its body says "//:ompl is an alias for it, tagged manual, so //... does not build it"; `158cc23 2026-11-24 Build: OMPL is the planner's second search, no longer a yardstick` rewrites the comments (SAME). `nlohmann_json` from the registry: 68a1199 2026-11-25 (`git log -S nlohmann_json -- MODULE.bazel`) SAME.

**Reason.** `bazel/menagerie.bzl:3-4` and `bazel/kuka.bzl:4-5` "The repository holds every KUKA robot ROS supports and is far larger than one robot's meshes" (SAME; old cites `menagerie.bzl:3`, KUKA `:4-5`). `base_url`: "to self-host" (`MODULE.bazel:30,70`, `:106`). Why a hash: no reason recorded in the repository beyond "checked against a SHA-256 hash". OMPL's archive: `docs/ompl-integration.md:1388` (decision 2; old said 1385: DIFFERS) "Its hash can be checked against upstream's own; it is a file that was uploaded, not one generated on request"; `:1389` (decision 3; old 1386: DIFFERS) "`http_archive` already does what `base_url` is for: `urls` takes a list and tries each in turn. The repository's own rules exist because their sources are loose files in large repositories, which an archive is not." and "The repository's written convention for dependencies outside the registry would need a sentence saying archives use `http_archive`." (verified, and no such sentence was added to `AGENTS.md`, `README.md` or `docs/`). Also `MODULE.bazel:102-105` "The archive is the one OMPL's / maintainers upload with each release (it has the git submodules inside, / which GitHub's own archive of the tag lacks), and the hash is the one they / publish beside it."

**Alternatives mentioned.** `docs/ompl-integration.md:1388` (release asset 150 MB vs GitHub tag archive 77 MB), `:1389` (`http_archive` vs an own rule with `base_url`), `:1396` decision 10 (own registry), `:1397` decision 11 (contribute to the central registry); O1 to O8 (`:47-644`); approaches A to H (`:1245-1252`). The decisions table has header at `:1380`, rows 1 to 12 at `:1387-1398` (old said `:1380-1393`, and row lines 4 too low: DIFFERS).

---

## 3. MuJoCo as the simulator; hand-written perception/planning/control; C++20; `--config=opt`

**True today.**
- `MODULE.bazel:11-12` "Physics engine. Its C API is the target @mujoco//:mujoco." and `bazel_dep(name = "mujoco", version = "4.0.0")` (SAME).
- `README.md:4-6` (old 5-7): "Perception, motion planning and control are / hand-written in C++ against MuJoCo." Same sentence at `8adf3bd:README.md:4-5`.
- `.bazelrc:1-3` "C++20 for this project's code and for everything it builds from source." `build --cxxopt=-std=c++20`, `build --host_cxxopt=-std=c++20`.
- `.bazelrc:21-27`: "With optimization on, GCC warns that a variable in one of MuJoCo's files may / be used before it is set, and MuJoCo's build turns warnings into errors, so / plain `-c opt` fails. This config turns that one error back into a warning." (lines 23-25) with `build:opt -c opt` and `build:opt --per_file_copt=^external/mujoco.*@-Wno-error=maybe-uninitialized` (`:26-27`).
- Hand-written pieces: `AGENTS.md:100-121` map (rrt.h, motion_planner.h, ik.h, module_finder.h...); OMPL is the one library (`src/ompl_search.h`, `AGENTS.md:107`).

**When.** MuJoCo, C++20: 8adf3bd 2026-11-19 SAME (`git log -S'std=c++20'`). `--config=opt` and the maybe-uninitialized flag: 83970cc 2026-11-19, body "- .bazelrc: --config=opt, an optimized build that gets past a warning in / MuJoCo's own code" SAME. `-Wno-psabi`: 3fb9514 2026-11-19 SAME. `--output_filter=^//`: f8d4b94 2026-11-19 SAME. "5 to 70 times longer": `12d1127:AGENTS.md:11` ("unoptimized tests take 5 to 70 times longer"), `4c5bf74:AGENTS.md:9`, now `AGENTS.md:9`; `ci.yml:58-59` (old said 56-57: DIFFERS) "Optimized: the tests that run whole cycles take 5 to 70 times longer / unoptimized (motion_planner_test alone about 4 minutes)." (`ci.yml:58-59`, added by dd03149).

**Reason.** no reason recorded in the repository for MuJoCo, for hand-writing perception/planning/control, or for C++20 (the `.bazelrc:1` comment gives scope only). For `--config=opt`: the `.bazelrc:23-25` comment above. For one standard across dependencies: project.md:199-200 "A flag change there recompiles MuJoCo; scope new flags with `--per_file_copt` or per target." and `docs/ompl-integration.md:1390` (old said 1389: DIFFERS) "C++17 for OMPL alone would go against `.bazelrc`'s rule and gains nothing known."

**Alternatives mentioned.** None for simulator or language. For one planner against another: decision 7.

---

## 4. Control code reads sensor data only

**True today.**
- `AGENTS.md:66-69` (the quote starts at 66, ends 69): "**Sensor data only.** Control code reads depth images with the calibrated camera, / joint angles, the conveyor's encoder, and what the script did itself. `True...` / accessors and body poses out of the simulation are for the scene generator, the / scorer (`src/score.h`) and tests."
- `src/pick_place.h:22-25` "It never reads where a module or the pallet truly is. / pick_place.cc does not call any True... accessor and reads no body pose / out of the simulation; that is what makes the score of a run (src/score.h) / a fair measure of what each way of sensing costs." (old said 18-25: the quote is 22-25; 18-21 is the list of what the script may read.) Also `src/pick_place.cc:40-41` ("Nothing here reads a module's or the / pallet's pose out of the simulation, or calls a True... accessor") and `src/trial_runner.h:13-15` ("it is a scorer, not control code. The script it runs / never reads the truth.").
- Accessors `src/cell.h:304-321` (section "truth", `:305` "For setting up scenes, tests and scoring only."): `true_scene()` `:308`, `TrueModulePose` `:312`, `TrueModuleSpeedOnTube` `:317`, `TruePlacedSlot` `:321`; `src/cell.h:103` "What Reset drew for one seed. Truth: control code must not read it."; `src/conveyor.h:177,180,184`.
- Callers (grep `True[A-Z]` over `src/`): `cell.cc`, `conveyor.cc`, `score.cc:24-64`, `trial_runner.cc:128,179`, `print_cell.cc:101,162-171`, `print_perception.cc:37,258-263`, `print_plan.cc:287`, `view_cell.cc:387,434`; comment-only: `collision.h:232`, `motion_planner.h:331`, `module_finder.h:559`, `score.h:26,69`. Tests (11): cell, conveyor, module_finder, pick_place, camera_cycle, check_cycle, planned_carry, collision, motion_planner, depth_camera, ompl_search. Zero hits in `src/layout*`, `bay*`, `container*`, `flat_rack*`, `arm_catalog*`, `ompl_search*`. All SAME as old.

**When.** Rule and accessors: 83970cc 2026-11-19 (`git log -S` on `pick_place.h` and `cell.h`) SAME. In `AGENTS.md`: `12d1127 2026-11-20` as "**The rule.** Control code reads sensor data only: ..." (`12d1127:AGENTS.md:27`); as "Sensor data only": `4c5bf74 2026-11-23` (body: "Kept: the build rules, the Bazel lock, BUILD.bazel's sections, sensor / data only, ..."). SAME.

**Reason.** `src/pick_place.h:24-25` as quoted: the score is "a fair measure of what each way of sensing costs". project.md:33-35 "No fixed tooling that relies on tight tolerances: the cell corrects itself from what it senses, because it works outdoors, is moved and jostled, / loses its calibration, and meets wind and glare." (a reason for sensing, not stated as the reason for this rule).

**Alternatives mentioned.** No written alternative to the rule. The blind run is the contrast: `README.md:164` "none of 20 trials placed a module" with `--blind`, `docs/blind_vs_camera.gif` (`README.md:184`); the words "blind 0 of 20" are in the body of `a2f4e85` ("a whole pallet 30 of 30; blind 0 of 20"), not in the README (old note says README and a2f4e85: DIFFERS).

---

## 5. The same results on every platform; `src/portable_random.h`

**True today.**
- `AGENTS.md:73-76` quote verified word for word.
- `src/portable_random.h:3-7` "the library distributions (...) are not: each / standard library chooses its own algorithm, and libstdc++, libc++ and / Microsoft's give different numbers for the same seed." (old wrote "are not [fixed]": the bracket is the old note's insertion). `:14-19` "GCC evaluates them right to left on x86-64 / and left to right on 64-bit ARM: the same seed gives x and y swapped. Take / each draw in a statement of its own." `:22-23` Normal() calls std::log (`:91`).
- Users: `src/rectangle_fit.cc:13`, `tests/rectangle_fit_test.cc:24,119`, `tests/ompl_search_test.cc:10,36`, `src/layout_plan.h:112-118` ("iteration budgets only (no time limit)"), `:121-124` ("it never compares a / freshly planned path with the stored one"), `src/layout_plan.cc:25`, `src/layout_check.h:94`, `src/layout_check.cc:32`.
- Not covered (consequences): `std::uniform_real_distribution`/`normal_distribution`/`uniform_int_distribution` still in `src/point_generator.cc:21-63`, `src/rrt.cc:442,522` (old listed 442 only), `src/conveyor.cc:123`, `src/plane_fit.cc:133` (not in old note). Own draws on mt19937: `src/cell.cc:45-47`, `src/depth_camera.cc:23`, `src/check_rig.h:53` (SAME). Wall-clock bounds still in tests: `tests/rrt_test.cc:373` (`EXPECT_LT(seconds, 2.0)`), `:390-391` (`EXPECT_GE(report.seconds, 0.05)`, `EXPECT_LT(report.seconds, 0.5)`) SAME.

**When.** `5600068 2026-11-20 Tests: rectangle fit's random scenes are the same on every platform` created `src/portable_random.h` (`git log --diff-filter=A`) SAME. `ae3bb74 2026-11-20 Tests: no wall-clock bound on the planner's median time; CI may take an hour` (touched `tests/motion_planner_test.cc` and `ci.yml`) SAME. In `AGENTS.md`: first at `4c5bf74 2026-11-23` (`git log -S'The same on every platform' -- AGENTS.md`; absent from `12d1127`) SAME.

**Reason.** `5600068` body: "Two draws inside one call are evaluated right to left by GCC on x86-64 / and left to right on aarch64, so "seed 1" was a different scene on CI, / where two bounds set from three local scenes failed". `ae3bb74` body: "The median planning time was asserted under 1 s; a shared CI runner / measured 1.05 s. The query counts are the machine-independent bound". SAME.

**Alternatives mentioned.** `5600068`: bounds "restated from 10,000 scenes and tested on 500"; no alternative generator mentioned. `docs/flat-rack-cell.md:295-296` "the plan repeats byte for byte on one machine for any number / of threads, but another platform's standard library may find other paths." (old `:296`: close). project.md:237-240 OMPL planners seeded from the clock (SAME).

---

## 6. A figure comes from a program's run

**True today.**
- `AGENTS.md:77-80` quote verified word for word.
- Programs in `BUILD.bazel`: `run_cell` 738, `print_profile` 102, `print_plane_fit` 157, `print_points` 170, `print_pallet` 300, `print_cell` 622, `print_depth` 902, `print_perception` 1011, `print_rrt` 1124, `print_plan` 1252, `print_check` 1355, `run_gate` 1570, `gen_bay` 1670, `print_bay` 1703, `check_layout` 1966, `plan_layout` 2087 (SAME list).
- `docs/ompl-comparison.md:12-13` (old said 13-14: DIFFERS) "Every figure below was printed by `print_plan` or `run_cell`; the ratios and percentages / were computed from those figures."; `docs/layout.md:20-22` (old 20-21) "... every result below is printed by these commands:"; `docs/flat-rack-cell.md:9` "Every result below is printed by these commands:".

**When.** Habit: `12d1127 2026-11-20`, `AGENTS.md:73-76` there: "**Verify before committing a stage,** in the lead session: read the diff; rerun the full / suite; reproduce the stage's table with `run_cell` or the `print_*` program and compare / with the agent's figures; run one cycle in the viewer." Present wording: `4c5bf74 2026-11-23` (`git log -S'A number comes from a run'`; its body does not list it) SAME. First commit of each program (`git log -S'name = "<p>"' -- BUILD.bazel`): print_profile f8d4b94, print_plane_fit/print_points 3fb9514, run_cell/print_pallet/print_cell 83970cc, print_depth/print_perception/print_rrt/print_plan c97c947, print_check a2f4e85, run_gate dd03149, print_bay 1213319, check_layout/gen_bay 68a1199, plan_layout 940a01b (old listed only the 11-19 commits: INCOMPLETE).

**Reason.** no reason recorded in the repository for the rule itself (not in `AGENTS.md`). Context only: project.md:244-247 "an agent that / wrote none of it checks the document's prose against the raw outputs (it found four / wrong figures)" and project.md:298-299 "the review found real gaps in the / checker and every layout, picture and figure had to be made again" (SAME; both are run reports, not a statement of why the rule exists).

**Alternatives mentioned.** None.

---

## 7. A second planner (OMPL) behind a switch, own the default

**True today.**
- `src/motion_planner.h:98-111` "The search. ..." block, last sentence "With `search` left alone nothing of OMPL runs."; `:151-154` `enum class PlanSearch { kOwn, kOmpl }` (old 149-158).
- `src/run_cell.cc:432` `cell::PlanSearch planner = cell::PlanSearch::kOwn;`; `--planner own|ompl` at `src/run_cell.cc:33,422`, `src/print_plan.cc:17,40,202`. `README.md:222-224` "...; the hand-written one is the default." Files `src/ompl_search.h`, `bazel/ompl.BUILD.bazel`, `tests/ompl_search_test.cc`.
- `docs/ompl-comparison.md:16-39` rule and `:39` "Verdict: hand-written better"; `:7-8` "The verdict follows a rule that was fixed / before any of these numbers existed"; `:8-10` "whether OMPL becomes the default is the owner's decision and nothing / here recommends for or against it"; `:391-393` "## If OMPL became the default" / "Facts for that decision." SAME.
- `BUILD.bazel:1578-1585` (`# BEGIN ompl`; every planning target links OMPL; no manual tag). 9397bdd body "No target is tagged manual: the planner depends on it." SAME.
- `9397bdd` also edits `src/trial_runner.{h,cc}` and `src/run_cell.cc` (`git show --stat`), as the old note says; `git diff --stat 9397bdd 940a01b -- models/cell.xml src/cell.* src/pick_place.* src/pallet.* src/conveyor.* src/check_rig.* src/trial_runner.* tests/gate_test.cc docs/gate.md` prints nothing.

**When.** `docs/ompl-integration.md`: `13678be 2026-11-24` (added; `git log --diff-filter=A`) and `fa4d595 2026-11-24`; merged by `a091cc2 2026-11-23`. `158cc23 2026-11-24 Build: OMPL is the planner's second search, no longer a yardstick`, earlier `1d68640 2026-11-23 ... for comparison only`. Landed on main `9397bdd 2026-11-24 Planning: OMPL as a second search behind a switch, and the comparison with the own planner` (added `ompl_search.h`, `ompl-comparison.md`, `ompl.BUILD.bazel`). SAME.

**Reason.** 9397bdd body: "With the switch left alone nothing changes: no existing test is edited, / print_plan's rows for each seed are the bytes they were, and src/rrt.*, / src/collision.*, ... are untouched." and "By the rule fixed beforehand: "Verdict: hand-written better". OMPL's RRT-Connect returns the own planner's paths on the three carries (150 of 150 plans) but needs 2735.0 collision queries at the median against 664.0 on the return ... The document does not decide whether OMPL becomes the default." `docs/ompl-integration.md:3-5` "This report is for deciding how OMPL (...) should be brought into this repository, and what the comparison of its planners with the hand-written one in `src/rrt.h` will need." No owner decision to keep own as default is recorded; project.md:227 says only "own being the default". SAME.

**Alternatives mentioned.** O1 to O8 (`docs/ompl-integration.md:47-644`); recommendation O1 (`:1387`; old said `:1383`: DIFFERS) "one of the two built options that changed nothing in the existing build, needed no patch, and compile OMPL with the repository's own flags; of the two, the one without a registry to keep". Twelve decisions for the owner `:1387-1398`. Recommended and not done: decision 5 (`:1391`, a CI query that nothing but manual targets depends on OMPL; in fact `BUILD.bazel` makes `motion_planner` depend on it) and decision 6 (`:1392`, a CI job of its own; `ci.yml` has none and no `ompl`/`rdeps` text). Planners compared: RRTConnect, BiTRRT, BITstar, AORRTC, InformedRRTstar (`docs/ompl-comparison.md:18-19`, rule text).
**Elsewhere.** `README.md:222-224`, `:306-320` (old: 306-325); `AGENTS.md:107`; project.md:223-229; the OMPL comment is at `MODULE.bazel:97-106` (old said `:87-93`: DIFFERS).

---

## 8. A cell's layout as data, checked by a program

**True today.**
- `layouts/*.json`: 8 files (above, below, below_each_rail, below_rails_by_module_arm, beside; flat_rack_1/2/3_pin_arms). SAME.
- `src/layout.h:6-10` "One JSON file under layouts/ is one layout, and / nothing about a layout is written anywhere else: the model that is looked / at (src/layout_model.h) and the checks (src/layout_check.h) are both made / from the file."; `:13-16` "LoadLayout reads a file and refuses one with a / missing or unknown field, a wrong type, an arm model that is not in / src/arm_catalog.h, or a name that is used twice, and says which." SAME words (old lines 1-14: DIFFERS slightly).
- `//:check_layout` (`BUILD.bazel:1966`, `src/check_layout.cc`), tests `layout_check_test`, `layout_scene_test`, `layout_rows_test`; commands `docs/layout.md:20-34`; `AGENTS.md:116-118`. `docs/layout.md:20-21` "Five layouts were written as data (`layouts/*.json`, in the form of `src/layout.h`), / checked by one program (`src/layout_check.h`) and compared" SAME.

**When.** `68a1199 2026-11-25` added `layouts/`, `src/layout.h`, `src/layout_check.h` (`git log --diff-filter=A`); body: "A layout of the new cell is one JSON file under layouts/; src/layout_model.h builds its MuJoCo model, //:check_layout checks one whole bay for transport, reach, clearance, load, the ways in and out, time and how the tube is held, and //:viewer_layout shows it." (the old note's elision of the list is fine). `940a01b 2026-11-25`: "Run 011's five layouts load in the new form". `nlohmann_json` `MODULE.bazel:21-22`, added in 68a1199. SAME.

**Reason.** project.md:26-27 "Layouts will be compared, so a quick way to describe and check a layout is part of / the work." (owner, 2026-11-24, project.md:19-20 and :40). `src/layout.h:6-10` one file as the single source for model and checks. project.md:140-141 `check_layout` "checks a layout in about four seconds". For JSON as the format: no reason recorded in the repository (`MODULE.bazel:21` "JSON for the layouts of the cell" says what, not why).

**Alternatives mentioned.** Five layouts compared: `docs/layout.md:126` "## The layouts", `:177` "## The comparison", `:237` `above` fails, `:256` `beside` passes and not chosen, `:219` three ways of putting the rails on; `docs/layout.md:720-794` fifteen decisions. No alternative to data-plus-checker. SAME.

---

## 9. A generated model is committed with its generator and a test that they agree

**True today.**
- `src/bay_model.h:5-11` "It was once written out by a script that is / lost. BayModelXml() is that script again: every position, size and mass in / the text follows from the header's constants, so that a change of the / header (the number of modules, say) is a change of the model by running / //:gen_bay and nothing else. tests/bay_model_test.cc fails when the / committed file differs from this text by a single byte." (old said 1-17; command `bazel run --config=opt //:gen_bay > models/bay.xml` is at `src/bay_model.h:13`, old said 15: DIFFERS); also `BUILD.bazel:1667-1670`, `AGENTS.md:114`.
- `tests/bay_model_test.cc:2-5` "the committed models/bay.xml is, byte for byte, what BayModelXml() / returns; when it is not, the message shows the first line that differs / and the command that writes the file again".
- No other model is generated: `models/cell.xml`, `container.xml`, `flat_rack.xml`, `arm.xml` etc. are hand-written (headers of `container.xml`, `flat_rack.xml`, `cell.xml` are prose); `tests/container_test.cc:11,58`, `tests/flat_rack_test.cc:11,37` read `MODEL_PATH` and check against the header. SAME.
- Layout models are built at run time: `src/layout_model.cc:26-30` "<actuator> and <keyframe> are removed: the model is for looking and / measuring, ..." and `:29-30` "so nothing is written / to disk" (old said 26-34: DIFFERS slightly).
- `plans/*.json` (2 files) are written by `plan_layout --out PATH` (`src/plan_layout.cc:3,265`; `docs/flat-rack-cell.md:15-17`). No byte test: `tests/plan_replay_test.cc` verifies the committed plan without planning against `layout_sha256` (`src/layout_plan.h:231`, `Sha256OfFile` `:313`); `src/layout_plan.h:121-124` "it never compares a / freshly planned path with the stored one". SAME.

**When.** `1213319 2026-11-24 Bay: the real parts of one bay and the container, from public sources, as models` added `models/bay.xml` (and `container.xml`). `68a1199 2026-11-25` added `src/bay_model.h`, `src/gen_bay.cc`, `tests/bay_model_test.cc`; body: "models/bay.xml is generated by //:gen_bay from src/bay_hardware.h, and a test fails when the committed model differs; with the count at ten the generator printed the model of 1213319 byte for byte." `plans/`: `940a01b`. SAME.

**Reason.** `src/bay_model.h:5-6` "It was once written out by a script that is lost." (the reason the generator exists). project.md:55 and 61-63 "Accepted from run 007's report, each to be done when a spec asks for it: ... generated models get a committed generator". 68a1199 body: "A bay is nine modules and ten rails, from one constant". Why nine, not ten: project.md:55-57 ("decided 2026-11-24, after run 007 showed that ten leave 35 mm at each end of a 40 ft container"). The old note's causal chain is consistent; `docs/bay-hardware.md` has no sentence on generation (grep "generat" finds nothing). SAME.

**Alternatives mentioned.** None.

---

## 10. Programs that print tables print CSV; `--pretty` for an aligned table

**True today.**
- `src/print_profile.cc:2-3` "The output is CSV, for a plotting tool. With --pretty it is a table of / aligned columns, for reading in a terminal." (old `:1-3`).
- `--pretty` in `check_layout.cc`, `plan_layout.cc`, `print_plane_fit.cc`, `print_profile.cc`, `print_plan.cc`, `print_perception.cc`, `print_check.cc`, `run_cell.cc` (+ `trial_runner.cc`) SAME. `AGENTS.md:109` "| Tables | `run_cell`, `print_perception`, `print_plan`, `print_check`, each with `-- --pretty` |"; `README.md:56` "`--pretty` (aligned table; CSV without it)"; `README.md:77-78` "Add `-- --pretty` to `print_profile`, `print_plane_fit`, `print_perception` / and `print_plan`" (old: 72-85).
- Not uniform: no `--pretty` in `print_bay` (CSV, `print_bay.cc:9,20`), `print_rrt` (CSV), `print_points` (CSV), `print_depth` (prints a text summary, CSV only with `--points`, `print_depth.cc:18`: old note says CSV: DIFFERS), `print_cell`, `print_pallet`, `run_gate`, `gen_bay` (no table). `check_layout`/`plan_layout`: CSV then `#` lines (`docs/layout.md:26-28`; old said 22-26: DIFFERS).

**When.** `f8d4b94 2026-11-19` (`print_profile`; body "prints both shapes for one move as CSV, or as an aligned / table with --pretty"); also 3fb9514 (`print_plane_fit`), 83970cc (`run_cell`), c97c947 (`print_perception`, `print_plan`), a2f4e85 (`print_check`), 68a1199 (`check_layout`), 940a01b (`plan_layout`) (`git log --diff-filter=A -- src/<p>.cc`). SAME.

**Reason.** `src/print_profile.cc:2-3` only. Nothing else recorded; the old note's "likely" is a guess. **Alternatives.** None.

---

## 11. Kinematic checking and planning before physics

**True today.**
- `docs/flat-rack-cell.md:7-9` "The cell is designed, not built, and kinematic: arms, tools, parts / and envelopes are checked at poses and along planned motion, and nothing steps the / physics."; `:32-40` "What is allowed", "The owner set these limits on 2026-11-24 for the first cell." ... "The work is kinematic only." (`:40`); `:633` "- Physics, sensing and controllers; ..." SAME.
- `src/layout_plan.h:12-13` "Kinematic only: nothing here steps the physics." (the sentence is on `:13`); `src/layout_model.cc:26-28` actuator/keyframe removal SAME.
- project.md:157 "The layout's model has no actuators and its viewer does not step physics." and :130-131 "Nothing in the new cell moves yet: it is checked at poses." `README.md:114-120` (old 116-119: DIFFERS; the quote "It has no / controllers, no cameras and no gate: it is kinematic." is at `:119-120`). `docs/layout.md:14-18` "the / checks are made at poses"; "Paths between poses. Only poses are checked, as the brief scopes it." is at `docs/layout.md:798` (old `:796`: DIFFERS; `:796` is the heading "## What was not considered").
- Physics test of the new parts: `tests/locate_physics_test.cc` (`BUILD.bazel:2039-2041`). SAME.

**When.** Poses only: 68a1199 2026-11-25. Planned motion, still kinematic: 940a01b 2026-11-25 (`//:plan_layout`, `tests/plan_replay_test.cc`; "Kinematic only" first appears there per `git log -S` on `layout_plan.h`). SAME.

**Reason.** Scope, set by the owner: `docs/flat-rack-cell.md:32-40` and project.md:40 "A working system comes first, then iteration (the owner, 2026-11-24, after run 011)." No cause for the order recorded in the repository beyond that. Fact bearing on it: the time model against the planner, `docs/layout.md:661-719`, table `:673-684`: planned / model 1.731 to 3.488 on seven moves (`:685`); "1.7 to 3.5 times" is in the body of 68a1199 ("they take 1.7 to 3.5 times what the time model gives") and project.md:143-145 ("low by 1.7 to 3.5 times"), not in `docs/layout.md` in those words.

**Alternatives mentioned.** Time model vs planner (`docs/layout.md:661`); moves planned one arm at a time, others at wait, not jointly (`docs/flat-rack-cell.md:264-298`, "### The method and its limits" `:264`, "Decoupled: ..." in its list).

---

## 12. The stand-in cell is kept untouched until it is replaced

**True today.**
- project.md:58-60 "the stand-in cell / (`models/cell.xml`, `src/pick_place.*`, the gate) still has ten slots and a pallet of / thirty, on purpose: it is not being converted, it is being replaced." and :160-163 (carries over: `src/rrt.h`, `path_timing.h`, `viewer_lib.h`, `mj_handles.h`; needs changes: `ik.h`, `collision.h`, `motion_planner.h`; does not: `pick_place.*`, `pallet.*`, `conveyor.*`, `check_rig.*`, `models/cell.xml`). SAME.
- Bodies: 1213319 "The cell does not change: models/cell.xml loads neither part and still carries its old square tube. README.md and AGENTS.md point to the new pieces."; 68a1199 "The stand-in cell is unchanged; README.md and AGENTS.md name the new pieces."; 9397bdd "...src/rrt.*, src/collision.*, src/path_timing.*, src/pick_place.* and models/ are untouched." SAME.
- `AGENTS.md:111,116` "(beside the cell, not in it)"; `README.md:107-108` "exist as models beside the cell, / which still uses its old square tube"; `docs/bay-hardware.md:769-772` "What the cell's models and headers assume now, beside the reference. Nothing here is / changed by this document." SAME.
- `AGENTS.md` has no rule "do not edit the stand-in cell". The phrase "stand-in cell" is in project.md, body of 68a1199, **and `src/layout_model.cc:14`** ("the stand-in cell's suction"): old note says brief, spec and 68a1199 only (DIFFERS). `docs/bay-hardware.md:823` uses "stand-in modules" (a different thing).
- Verified: `git diff --stat 9397bdd 940a01b -- models/cell.xml src/cell.* src/pick_place.* src/pallet.* src/conveyor.* src/check_rig.* src/trial_runner.* tests/gate_test.cc docs/gate.md` prints nothing. SAME.

**When.** "Beside, not in": 1213319 2026-11-24 (SAME). "Replaced, not converted": project.md (not dated line by line; the list is "the owner, 2026-11-24" and "Since `68a1199` (run 011)" at :57-58).

**Reason.** project.md:64-68 "The cell is to be rebuilt on the real parts of `docs/bay-hardware.md` (...), and that work includes / its layout (decided 2026-11-24)."; gate's tolerance is the owner's: `docs/gate.md:34` "This is an assumption chosen by the project owner." (old: `:35`: DIFFERS); `AGENTS.md:70-73` "What "correct" means is the owner's" (old: `:62-65`: DIFFERS). No cause recorded for keeping the stand-in unchanged beyond these.

**Alternatives mentioned.** None.

---

## 13. One `BUILD.bazel` with `# BEGIN x` / `# END x` sections

**True today.** `grep -c '^# BEGIN' BUILD.bazel` = 29, `^# END` = 29; first `# BEGIN arm` `BUILD.bazel:183`, last `# BEGIN layout_plan` `:2055`; lines 1-182 have none. `bazel/BUILD.bazel:1-2` "Holds .bzl files only. A directory needs a BUILD file before Bazel can load / anything from it." Only two BUILD files exist (`./BUILD.bazel`, `./bazel/BUILD.bazel`). `AGENTS.md:55-56` quote verified. project.md:194-195 "`BUILD.bazel` at the top is the only build file for the code." SAME.

**When.** One file since 8adf3bd 2026-11-19; first `# BEGIN` in `83970cc 2026-11-19` (`git log -S'# BEGIN'`, oldest; `83970cc:BUILD.bazel:183 # BEGIN arm`). In `AGENTS.md` at `12d1127 2026-11-20` (`:17` "`BUILD.bazel` is one file with a `# BEGIN x` / `# END x` section per piece. Edit only / your own section."); still in 4c5bf74. SAME.

**Reason.** `AGENTS.md:53-57` ("Dividing the work", "Two agents never own the same file"). No reason recorded for the single file itself.

**Alternatives mentioned.** `docs/ompl-integration.md:1391` "A separate package with `visibility` would also do it, against the rule of one build file." (SAME). The old note's `:1375` reference is wrong: that line is about the OMPL build file's size, not the one-file rule (DIFFERS).

---

## 14. Flat `src/` and `tests/`; namespace `cell` and nested ones

**True today.** `src/` 99 files (42 headers, 57 `.cc`); `tests/` 41 files; no subdirectories (`find src tests -mindepth 1 -type d` empty). `namespace cell {` in 60 files of `src/` (+2 in `tests/`; 17 `src/*.cc` programs have no namespace). Nested: `cell::layout` `src/layout.h:98`, `layout_check.h:291`, `layout_model.h:74`, `layout_scene.h:138`, `layout_plan.h:143`, `layout_rows.h:71`, `arm_catalog.h:27`; `cell::bay` `bay_hardware.h:87`, `bay_model.h:58`; `cell::container` `container.h:42`; `cell::flat_rack` `flat_rack.h:50`. No document states a rule on namespaces or layout. SAME.

**When.** Flat since 8adf3bd. `namespace cell {` first appears in **8adf3bd 2026-11-19** (old: "f8d4b94/83970cc": DIFFERS). `cell::bay` and `cell::container`: 1213319 2026-11-24; `cell::layout`: 68a1199; `cell::flat_rack`: 940a01b (`git log -S'namespace cell::...'`). SAME.

**Reason.** no reason recorded in the repository. **Alternatives.** None.

---

## 15. Each header opens with a long comment

**True today.** All 42 headers begin with `//`; comment length of the first block: 6 lines `src/mj_handles.h`, 9 `arm.h` and `point_generator.h`, 11 `joint_move.h`, 12 `check_rig.h`, `pallet.h`, `score.h`; `src/pick_place.h` 240 of 1,173 lines (all SAME). `AGENTS.md:95-96` "Each header opens with a long comment that explains the piece (240 of the 1,173 lines of / `src/pick_place.h`). Read that before the body." project.md:201-202 "Comments are plain, complete sentences; headers open with a long comment that explains / the piece."

**When.** In code: `src/mj_handles.h` at 8adf3bd 2026-11-19. In `AGENTS.md`: `12d1127 2026-11-20` (`:32` "**Map.** Read the top-of-file comment of a header before its body; they are long."); present wording `4c5bf74 2026-11-23` (`git log -S'long comment'`). SAME.

**Reason.** no reason recorded beyond "Read that before the body" (`AGENTS.md:96`). **Alternatives.** None.

---

## 16. Tests: one `tests/<piece>_test.cc` per piece, GoogleTest, the gate tagged

**True today.**
- 41 `tests/*_test.cc`, all 41 with one source each; **42 targets**, not 41: 40 literal `cc_test(` blocks plus the loop `[cc_test(` at `BUILD.bazel:1469-1498` that makes both `check_cycle_test` and `check_cycle_batch_test` from `tests/check_cycle_test.cc` (old: "40 literal blocks plus check_cycle_batch_test = 41": DIFFERS). Sizes of the 42 targets: 22 small, 16 medium (14 literal + the 2 generated), 4 large (old: 22/15/4: DIFFERS). Large: `camera_cycle_test` `BUILD.bazel:815`, `module_finder_test` `:986`, `motion_planner_test` `:1220`, `gate_test` `:1553` (name lines; `size` on the next). No `timeout =`. `shard_count = 4`: `motion_planner_test` `:1224` ("The statistics, the execution and the rest in separate shards, run in / parallel"), `gate_test` `:1558`. SAME.
- Gate: `BUILD.bazel:1559` `tags = ["gate"]`; `.bazelrc:37` `test:agent --test_tag_filters=-gate`; `ci.yml:72` (old said 67: DIFFERS) `--test_tag_filters=-gate`.
- `.bazelrc:31-34` "One full run of the suite already fills this machine's cores and, unlimited, / three at once ran it out of memory (measured 2026-11-22). The limit keeps / each worktree to three cores. The gate (about 4 minutes on 4 cores) is left / out: the lead runs it once, alone, before a commit, and CI runs it." (old `:29-37`: block is 29-37; text 31-34).
- `MODULE.bazel:24-25` googletest dev_dependency ("Only this repository's tests need it."). The old note says every test links `@googletest//:gtest`: DIFFERS: 40 blocks link `@googletest//:gtest_main`, only `gate_test` links `:gtest`.
- `AGENTS.md:14-15` "While iterating, only your piece's tests:" (old said `:11-13`: DIFFERS; 11-12 are the `--config=agent` sentence).

**When.** GoogleTest `MODULE.bazel:18` at 8adf3bd 2026-11-19; first test `tests/step_test.cc` at 8adf3bd; test per piece: 8adf3bd, f8d4b94 ("Tests assert the limits, the end state and hand-worked durations."), 3fb9514 ("Tests assert the face's distance within 0.5 mm...") SAME. Gate and `tags = ["gate"]`: `dd03149 2026-11-20` (`git log -S'tags = ["gate"]'`). `--config=agent` and the batch split: `5d8fb21 2026-11-22` (body: "BUILD.bazel: the CheckCycles suite of check_cycle_test runs as / check_cycle_batch_test, from the same source."; "AGENTS.md: named tests while iterating, the suite without the gate once / at the end, the gate and the unlimited suite for the lead alone"). SAME.

**Reason.** Per piece: `AGENTS.md:14` only your piece's tests, the reason being time (`AGENTS.md:25-31`). Gate out of `--config=agent`: `.bazelrc:31-34` above. `docs/gate.md:4-5` "It is part of `bazel test --config=opt //...` and / is tagged `gate`." Batch split: `BUILD.bazel:1466-1468` "One source, two targets, so that the two halves run side by side: the whole / camera cycles (the CheckCycles suite, whose shared setup runs nine cycles / and is about two thirds of the time) ..."; sizes: no reason recorded in the repository.

**Alternatives mentioned.** `BUILD.bazel:1467-1468` "Sharding instead would repeat that setup in every shard." (not in old note). Otherwise none.

---

## 17. CI: six jobs dividing the suite, one disk cache per job

**True today.** `.github/workflows/ci.yml` (line numbers at `940a01b`; old ones were off by 3 to 4): `on` push to `main`, `pull_request`, `workflow_dispatch` with `rerun_tests` `:3-14`; `CYCLE_TESTS` `:24-30`; comment "a runner has two cores, and in one job the suite took 33 minutes." `:32-33` (old 31-34); `runs-on: ubuntu-latest` `:37`; `timeout-minutes: 30` `:38` (old 35); matrix `part: [cycle tests, other tests, gate 1, gate 2, gate 3, gate 4]` `:43` (old 36-40); comment "Each part keeps its own, since / each builds and runs different tests." `:50-51` (old 47-51), `disk-cache: ${{ github.workflow }}-${{ matrix.part }}` `:55` (old 44-56); `GATE_TOTAL_SHARDS=4`, `GATE_SHARD_INDEX`, `GATE_THREADS=2` `:80-82` (old 76-80). project.md:211-212 the owner is moving the six jobs to arm64, `ci.yml` still x86-64. (Later commits 60f8efc, 30b015d touch `ci.yml` but are not in `940a01b`.)

**When.** `8adf3bd 2026-11-19` one job; `dd03149 2026-11-20` runs optimized (`ci.yml:58-59`); `ae3bb74 2026-11-20` timeout 30 to 60 (diff verified); `1d2c098 2026-11-22 CI: the suite divided over six jobs that run side by side` (also 60 back to 30, diff verified); `014ba3a 2026-11-22 CI: a run by hand can run every test again, cached or not`. SAME.

**Reason.** `1d2c098` body: "the cycle tests, the other tests, and the gate in four jobs of / two pallets each, every one with its own disk cache. In one job on a / two-core runner the suite took 33 minutes." and "gate_test: GATE_TOTAL_SHARDS and GATE_SHARD_INDEX name one shard from / outside, since Bazel does not let --test_env set its own TEST_ variables." `ae3bb74` body "a cold build with the gate took / 25.5 minutes once and over 30 the next time". project.md:208-209 "Org does not use pull requests, so CI runs when work lands on `main`; do not start runs / by hand, and do not wait for CI" (owner, 2026-11-24, `:209`). SAME. `docs/gate.md:65-80` is "Running it" and "Replaying a failure": it does not describe CI (old note cites it for CI: DIFFERS); `AGENTS.md` has no CI workflow text (only `:73` "CI is x86-64", and `docs/gate.md` for the gate).

**Alternatives mentioned.** `docs/ompl-integration.md:1392` decision 6: "never; on every pull request; in a job of its own when the OMPL files change and on a schedule", recommended "a job of its own", reason includes "The six existing jobs should not pay for it." Recommended, not made. SAME.

---

## The three decisions whose reason I could not find

Checked by grep over `README.md`, `AGENTS.md`, `docs/*.md`, `MODULE.bazel`, `.bazelrc`, `bazel/*.bzl`, `BUILD.bazel`, headers, and `git log --grep`; nothing found.
1. MuJoCo, hand-written perception/planning/control, C++20 (decision 3). Confirmed: no reason in the repository. `dd65a4d:README.md:16` says only they were "not chosen yet" (old note calls it the commit message: DIFFERS); 8adf3bd has a subject-only message.
2. Bazel with bzlmod and the pin to 9.3.0 (decision 1), and JSON as the layout format (decision 8). Confirmed. (Also none for: nested namespaces, decision 14; why own is the default planner beyond the 9397bdd statements and the verdict, decision 7; test sizes, decision 16.)
3. Flat `src/` and `tests/` with `cell::layout`, `cell::bay`, `cell::container`, `cell::flat_rack` (decision 14), and why kinematic work comes first (decision 11, beyond "The work is kinematic only", `docs/flat-rack-cell.md:40`, and the owner's "A working system comes first, then iteration", project.md:40). Confirmed.
Also without any recorded reason (not in the old list): decision 6 (a figure comes from a run), decision 9 (only "script is lost", `src/bay_model.h:5-6`, and project.md:61-63), decision 10, 12 (only the replacement plan), 13, 15.

---

## Where the decisions about the cell's design are written (corrected)

Headings and lines at `940a01b` (`grep -n '^## \|^### '`). Changes against the old table are marked.

| Place | Heading | Line | Decisions there |
|---|---|---|---|
| `docs/layout.md` | "## Decisions" ("Every decision the brief left open, with its reason.") | 720 | 15 numbered (`:724` to `:794`); "## What was not considered" at 796 (SAME) |
| `docs/layout.md` | "## The comparison" 177; "### Three ways of putting the rails on, by time" 219; "### What makes `above` fail" 237; "### Why `beside` was not chosen, though it passes" 256 | 177, 219, 237, 256 | choice among five layouts (chosen `below_rails_by_module_arm`; `above` failed; `beside` passed, not chosen); 3 ways of putting rails on (SAME; "## The layouts" at 126 introduces them) |
| `docs/layout.md` | "## The constraints" 46, "## What had to be possible at all" 84 | 46, 84 | feasibility findings; no numbered list (SAME) |
| `docs/layout.md` | notice at top | 3-5 | "This document stands as the / record of the closed container (run 011)" (quote spans 4-5; old said line 3) |
| `docs/flat-rack-cell.md` | "## Decisions" | 546 | 15 numbered (`:548` to `:624`, ends about `:629`); "## What was not considered" at 631 (SAME) |
| `docs/flat-rack-cell.md` | "## What is allowed" | 32 | the owner's limits "set ... on 2026-11-24 for the first cell" (`:34`); 5 `allowed` rows ("the chosen layout passes all five") (SAME) |
| `docs/flat-rack-cell.md` | "### How many pin arms (R12)" | 130 | 1 (three pin arms vs one and two) (SAME) |
| `docs/flat-rack-cell.md` | "## Margins" 406, "### The values used and why" 408 | 406, 408 | margin values; decision 12 of "Decisions" (`:603`) repeats the changes (SAME) |
| `docs/bay-hardware.md` | "## The reference" | 39 | 1: "Three things decided / it: it is the most shipped tracker, its torque tube is round, and no other round-tube / tracker was found that is better documented." (`:40-42`; the quote spans lines; the old paraphrase is right) |
| `docs/bay-hardware.md` | "## Torque tube" 70, "## Module" 118, "## Rail" 161, "## Fastenings" 201, "## The bay" 256 | 70, 118, 161, 201, 256 | `ours` rows 6, 5, 7, 8, 3 (SAME, counted with `awk '/^## /{sec=$0} /\| ours \|$/{c[sec]++}'`) |
| `docs/bay-hardware.md` | "## Holding the tube" | 286 | 5 `ours` rows (SAME) |
| `docs/bay-hardware.md` | "## Locating a module before pinning" | 385 | 25 `ours` rows; "This section is designed here; no source was read for any of it." (`:387`); "It rests on the owner's / decision of 2026-11-24" (`:387-388`) (SAME, spans lines) |
| `docs/bay-hardware.md` | "## The container" (also the flat rack's table) | 599 | 19 `ours` rows (SAME) |
| `docs/bay-hardware.md` | "## Against the cell today" | 769 | 6 `ours` rows; comparison, not a decision (SAME) |
| `docs/ompl-comparison.md` | "## Verdict" 16; "## If OMPL became the default" 391 | 16, 391 | 1 verdict rule fixed beforehand (`:39` "Verdict: hand-written better"); 391 holds "Facts for that decision." (`:393`), not a decision (SAME) |
| `docs/gate.md` | "## The tolerance" | 33 | 1: 3 mm, 0.15 degree, at rest; "This is an assumption chosen by the project owner." (`:34`); "The 2 mm is the / runner's own choice" (`:40-41`) (SAME) |
| `docs/ompl-integration.md` | "## Decisions for the owner" | 1380 | 12 rows, `:1387-1398` (old gave `:1380-1393`); recommended, not recorded as decided (SAME) |

Counts: `ours` rows total **84** (6+5+7+8+3+5+25+19+6), not 79 (old: DIFFERS). Overlap: flat-rack decision 9 (`:589-591`, "The holder is run 011's") and decision 11 (`:597`, "The order of work is run 011's") point back to `docs/layout.md` decisions 11 (`:775`) and 13 (`:783`) (SAME).

---

## Differences from the old note

Header: 28 commits at 940a01b, not 29. The project brief path does not exist (`r1` for `r2`).
Line numbers or quotation locations that were wrong:
- D1: `module(` is `MODULE.bazel:3-6`; Boost deps start at `:121`; `docs/ompl-integration.md:130-131` not `:132`; approaches are A to H (`:1245-1252`), not A to F; "not chosen yet" is `dd65a4d:README.md:16`, not the commit message.
- D2: OMPL comment/`http_archive` is `MODULE.bazel:97-115`; `docs/ompl-integration.md` decisions rows are `:1387-1398` (decision 1 `:1387`, 2 `:1388`, 3 `:1389`, 4 `:1390`; old 1383, 1385, 1386, 1389).
- D3: `ci.yml:58-59` not 56-57; `README.md:4-6` not 5-7; `docs/ompl-integration.md:1390` not 1389.
- D4: quote at `pick_place.h:22-25` (not 18-25); `AGENTS.md:66-69`; "blind 0 of 20" is in `a2f4e85`'s body, not the README.
- D5: quote "[fixed]" is the old note's insertion; extra unlisted standard-distribution uses at `src/plane_fit.cc:133`, `src/rrt.cc:522`.
- D6: `docs/ompl-comparison.md:12-13` not 13-14; program dates listed incompletely (dd03149, a2f4e85, 1213319, 68a1199, 940a01b missing).
- D7: OMPL comment in `MODULE.bazel:97-106` not 87-93; decision 1 row `:1387` not `:1383`; README planner paragraph `:306-320`.
- D9: command at `bay_model.h:13` not 15; text `1-17` is really 5-11; `layout_model.cc:26-30`.
- D10: `print_depth` is not a CSV program; `docs/layout.md:26-28` for CSV and `#` lines; `README.md:77-78` lists only four programs.
- D11: `README.md:114-120`; `docs/layout.md:798` not 796; "1.7 to 3.5" comes from 68a1199's body and project.md, the document's table gives 1.731 to 3.488.
- D12: `docs/gate.md:34` not 35; `AGENTS.md:70-73` not 62-65; "stand-in cell" also at `src/layout_model.cc:14`.
- D13: `docs/ompl-integration.md:1375` is not about the one-file rule.
- D14: `namespace cell {` first in 8adf3bd (not f8d4b94/83970cc); the 60 files are `src/` only.
- D16: 42 test targets (not 41), 16 medium (not 15); `ci.yml:72` not 67; `AGENTS.md:14-15` not 11-13; 40 targets link `gtest_main`, not `gtest`; `docs/gate.md:65-80` is not about CI.
- D17: all `ci.yml` line numbers (`:32-33`, `:38`, `:43`, `:50-55`, `:80-82`); `docs/gate.md:65-80` does not describe CI.
- R13: `ours` rows total 84 not 79; quotations in the table that span two lines (reference `:40-41`, locating `:387-388`, gate `:40-41`, layout notice `:4-5`); decisions table of `docs/ompl-integration.md` is `:1387-1398`.
Everything else (every commit hash, every date, the other counts, and the quotations not named above) was checked and SAME.

## Could not verify

- `origin/docs/new-hire` was not read (the owner's branch; outside the base commit).
- The ground for "likely" statements in the old note (guesses on Bazel pin, JSON, CSV, BUILD file, test sizes): there is nothing in the repository to check them against.
- `docs/ompl-integration.md:1230` and the other `[S..]` source claims were not followed to the external sources.
- project.md is not part of the repository; its quotations were checked against `/root/org-runs/013-design-principles-and-r2/project.md`, which may differ from the file the old note's author read.
- The "run 007/011 report" statements, the 2026-11-24 dates of owner decisions, and "the first cell's spec" exist only in project.md and commit bodies; no spec file was available.
- Dates of `b90da7c` and `1d68640` come from `git show`; their branches are remote spikes not in `940a01b`'s history.
