# History and reasons for the decisions already in force at 940a01b

Evidence for the lead's decision records. Everything was read at `940a01b` (or at the commit
named). `main` has 29 commits, 2026-11-19 to 2026-11-25; most commit bodies are short, so a
reason is often only in a code comment or a document. "The project brief" is
`/root/org-runs/013-design-principles-and-r1/project.md`. `origin/docs/new-hire` was not read.

Short names: `8adf3bd` Toolchain (11-19), `83970cc` Cell (11-19), `12d1127` AGENTS.md (11-20),
`5600068` portable random (11-20), `ae3bb74` no wall-clock bound (11-20), `5d8fb21` agent config
(11-22), `1d2c098` CI six jobs (11-22), `4c5bf74` AGENTS.md rewritten (11-23), `9397bdd` OMPL on
main (11-24), `1213319` bay parts (11-24), `68a1199` run 011 layout (log date 11-25),
`940a01b` run 012 flat rack (log date 11-25). Dates in the brief for run 011's decisions are 11-24.

The three decisions whose reason I could not find are at the end of the decisions, before the R13 table.

---

## 1. Bazel with bzlmod and one pinned version

**What is true now.**
- `.bazelversion:1` is `9.3.0`.
- `MODULE.bazel:3-4` `module(name = "solar_bay_assembly")`; `MODULE.bazel:9-25` `bazel_dep` for rules_cc, mujoco 4.0.0, eigen, glfw, nlohmann_json, googletest (dev); from about line 118 seventeen `boost.*` `bazel_dep`s.
- `MODULE.bazel.lock` is committed (709 lines; 463 at `8adf3bd`, 708 at `9397bdd`).
- `README.md:40-41`: "The build tool is Bazel, run through Bazelisk, which fetches the version named in `.bazelversion`." `.github/workflows/ci.yml:47`: "Installs Bazelisk, which reads .bazelversion".
- The brief: "built with Bazel 9.3.0 (bzlmod, `.bazelversion`)"; known flaw: `bazel mod deps` and `bazel mod graph` exit 2 (a flaw of the `mujoco` module); to check the lock file is current: `bazel build --nobuild --lockfile_mode=error --config=opt //...`.

**When.** `8adf3bd` 2026-11-19 "Toolchain: Bazel build, MuJoCo stepping a UR10e from C++, viewer, tests and CI" created `.bazelversion`, `MODULE.bazel`, `MODULE.bazel.lock`, `BUILD.bazel`, `bazel/`, CI. The lock changed at `9397bdd` (OMPL's Boost modules: +245 lines, `docs/ompl-integration.md:733`) and `68a1199` (nlohmann_json). The first commit `dd65a4d` says "Language, planning stack, and robot model are not chosen yet."

**Why.** No reason recorded for Bazel or for bzlmod: the body of `8adf3bd` is its subject only. The pin is described, not justified. For dependencies the benefit is stated: `docs/ompl-integration.md:132` "Pinned, self-hosted, offline ... the Boost modules are pinned by the hashes of their registry files in `MODULE.bazel.lock`", and the property "The same OMPL on every machine and on CI | numbers that can be compared between machines" (properties table before `:1234`). Likely: one Bazel version on ARM machines and x86-64 CI so results compare (guess).

**What was weighed.** For Bazel itself, nothing. For how a non-registry library enters a bzlmod build: `docs/ompl-integration.md` options O1 to O8 and the approaches A to F at `:1241-1256`.

**Where else.** `README.md:40`; `docs/ompl-integration.md:1449` ("Bazel | 9.3.0, chosen by `.bazelversion`, through Bazelisk"); the brief. Beware: "the Bazel lock" in the body of `4c5bf74` and `AGENTS.md:41` is Bazel's per-checkout command lock, not `MODULE.bazel.lock`; `AGENTS.md` never mentions the lock file.

---

## 2. External code fetched by hash, with a `base_url`; registry modules otherwise

**What is true now.**
- `bazel/menagerie.bzl:1-4` "only the files of the one model are downloaded, each checked against a SHA-256 hash"; `:20` `ctx.attr.base_url.format(...)`; `:21-25` download with `sha256`; `:49-52` attr `base_url` "Where the files are served from. Point it at another host to self-host."
- `bazel/kuka.bzl:1-6` the same idea; `:22` `base_url`; `:26-33` per-file `sha256`; `:57` attr.
- `MODULE.bazel:27-31` ("downloaded file by file at a pinned commit. Not a Bazel module ... To self-host, copy the files elsewhere and set base_url"), `:65-71` KUKA; both list every file with its SHA-256 and one `commit`.
- OMPL: `MODULE.bazel:95-114` `http_archive(name = "ompl", sha256 = "b9155e24...", urls = [...])` with the comment "To self-host, copy the archive elsewhere and put that address first in `urls`" (`:105-106`); build file `bazel/ompl.BUILD.bazel`.
- Registry modules otherwise: `MODULE.bazel:9-25`, the Boost modules.
- The convention is written only in the brief ("Conventions beyond AGENTS.md"): "`bazel_dep` from the Bazel Central Registry where the library is there; otherwise a repository rule in `bazel/` that downloads pinned files by SHA-256 and has a `base_url` attribute for self-hosting (`bazel/kuka.bzl`, `bazel/menagerie.bzl`)." It is not in `AGENTS.md`.

**When.** `menagerie.bzl`: `8adf3bd`. `kuka.bzl`: `83970cc`. OMPL `http_archive`: on spike branches `b90da7c`/`1d68640` (2026-11-23, "for comparison only", tagged manual), on `main` at `9397bdd` (2026-11-24; wording `158cc23`). `nlohmann_json` from the registry: `68a1199`.

**Why.** Menagerie: "Menagerie as a whole is several hundred megabytes, so only the files of the one model are downloaded" (`menagerie.bzl:3`). KUKA: "The repository holds every KUKA robot ROS supports and is far larger than one robot's meshes" (`kuka.bzl:4-5`). `base_url`: "to self-host". Why a hash: no sentence beyond "checked against a SHA-256 hash" (likely: a reproducible fetch). OMPL's archive: `docs/ompl-integration.md` decision 2 (`:1385`) "Its hash can be checked against upstream's own; it is a file that was uploaded, not one generated on request"; decision 3 (`:1386`) "`http_archive` already does what `base_url` is for: `urls` takes a list and tries each in turn. The repository's own rules exist because their sources are loose files in large repositories, which an archive is not." That decision 3 also notes "The repository's written convention for dependencies outside the registry would need a sentence saying archives use `http_archive`" (not added to any document in the repo).

**What was weighed.** `docs/ompl-integration.md:1380-1393` decisions 2 (release asset 150 MB vs GitHub's tag archive 77 MB), 3 (`http_archive` vs an own rule with `base_url`), 10 (a registry of the owner's: later), 11 (contribute to the central registry: later); options O1 to O8; approaches A to F (`:1241`).

**Where else.** The brief ("Conventions beyond AGENTS.md"; "Planning code"); `docs/ompl-integration.md:132`; body of `9397bdd`.

---

## 3. MuJoCo as the simulator; what is built on it; C++20; `--config=opt`

**What is true now.**
- `MODULE.bazel:11-12` "Physics engine. Its C API is the target @mujoco//:mujoco." `bazel_dep(name = "mujoco", version = "4.0.0")`.
- `README.md:5-7`: "Perception, motion planning and control are hand-written in C++ against MuJoCo." (the same sentence in `8adf3bd:README.md`.)
- `.bazelrc:1-3` "C++20 for this project's code and for everything it builds from source." `build --cxxopt=-std=c++20`, `build --host_cxxopt=-std=c++20`.
- `.bazelrc:19-27` the `opt` config: "With optimization on, GCC warns that a variable in one of MuJoCo's files may be used before it is set, and MuJoCo's build turns warnings into errors, so plain `-c opt` fails. This config turns that one error back into a warning." `build:opt -c opt`; `build:opt --per_file_copt=^external/mujoco.*@-Wno-error=maybe-uninitialized`.
- Hand-written pieces (map, `AGENTS.md:98-121`): `src/rrt.h`, `src/motion_planner.h`, `src/ik.h`, `src/module_finder.h`, `src/rectangle_fit.h`, `src/plane_fit.h`, `src/profile.h`, `src/arm_follower.h`; the only library for planning is OMPL (`src/ompl_search.h`).

**When.** MuJoCo, C++20: `8adf3bd` 2026-11-19. `--config=opt`: `83970cc` ("`.bazelrc`: --config=opt, an optimized build that gets past a warning in MuJoCo's own code"). `-Wno-psabi` for `src|tests`: `3fb9514`. `--output_filter=^//`: `f8d4b94`. "Always `--config=opt`: unoptimized tests take 5 to 70 times longer": `AGENTS.md` at `12d1127`, kept at `4c5bf74`. CI comment `ci.yml:56-57` gives the same figure with "motion_planner_test alone about 4 minutes".

**Why.** No reason recorded for MuJoCo over another simulator, for hand-writing perception, planning and control, or for C++20 (the `.bazelrc` comment gives scope, "for everything it builds from source", not a cause). For `--config=opt`: the `.bazelrc` comment. For "everything" including dependencies: the brief "A flag change there recompiles MuJoCo; scope new flags with `--per_file_copt` or per target." and `docs/ompl-integration.md:1389` decision 4: "C++17 for OMPL alone would go against `.bazelrc`'s rule and gains nothing known."

**What was weighed.** Nothing for simulator or language. For one planner against another: decision 7.

**Where else.** `README.md:5-7`; `AGENTS.md:7-9` ("Always `--config=opt`"); the brief (Conventions); `.bazelrc`.

---

## 4. Control code reads sensor data only

**What is true now.**
- `AGENTS.md:66-68`: "**Sensor data only.** Control code reads depth images with the calibrated camera, joint angles, the conveyor's encoder, and what the script did itself. `True...` accessors and body poses out of the simulation are for the scene generator, the scorer (`src/score.h`) and tests."
- `src/pick_place.h:18-25`: "It never reads where a module or the pallet truly is. pick_place.cc does not call any True... accessor and reads no body pose out of the simulation; that is what makes the score of a run (src/score.h) a fair measure of what each way of sensing costs." Also `src/pick_place.cc:41`; `src/trial_runner.h:13-16` ("it is a scorer, not control code. The script it runs never reads the truth").
- The accessors: `src/cell.h:304-321` (section "truth": "For setting up scenes, tests and scoring only.") `true_scene()`, `TrueModulePose`, `TrueModuleSpeedOnTube`, `TruePlacedSlot`; `src/cell.h:103` "What Reset drew for one seed. Truth: control code must not read it."; `src/conveyor.h:177-184` `TrueSlidePosition`, `TrueTubePose`, `TrueSlotPose`.
- Who calls them (grep `True[A-Z]` over `src/`): `src/cell.cc` (scene and baseline), `src/conveyor.cc` (definitions), `src/score.cc:24-64` (scorer), `src/trial_runner.cc:128,179` (scorer), `src/print_cell.cc`, `src/print_perception.cc:37,258-263` ("The truth ... is read here, to score"), `src/print_plan.cc:287`, `src/view_cell.cc:387,434`. Tests: `cell_test`, `conveyor_test`, `module_finder_test`, `pick_place_test`, `camera_cycle_test`, `check_cycle_test`, `planned_carry_test`, `collision_test`, `motion_planner_test`, `depth_camera_test`, `ompl_search_test`. `src/collision.h:232`, `src/motion_planner.h:331`, `src/module_finder.h:559`, `src/score.h:26,69` only name them in comments. The new cell's code has none of these.

**When.** Rule and accessors: `83970cc` 2026-11-19. In `AGENTS.md` as "The rule": `12d1127` 2026-11-20; as "Sensor data only": `4c5bf74` 2026-11-23 (its body: "Kept: ... sensor data only").

**Why.** The repository's reason is `src/pick_place.h:24-25`: the score is "a fair measure of what each way of sensing costs". The brief supports it: "No fixed tooling that relies on tight tolerances: the cell corrects itself from what it senses, because it works outdoors, is moved and jostled, loses its calibration, and meets wind and glare."

**What was weighed.** The README and `a2f4e85` compare blind with camera runs ("blind 0 of 20"; `docs/blind_vs_camera.gif`); no written alternative to the rule.

**Where else.** `AGENTS.md:66`; `src/pick_place.h:18`; `src/trial_runner.h:13`; `src/cell.h:304`; the brief.

---

## 5. The same results on every platform; `src/portable_random.h`

**What is true now.**
- `AGENTS.md:73-76`: "**The same on every platform.** Development machines are 64-bit ARM and CI is x86-64. A bound that rests on particular random scenes takes its draws from `src/portable_random.h`, one draw per statement. Do not bound a wall-clock time in a test; bound a count."
- `src/portable_random.h:1-23`: the library distributions "are not [fixed]: each standard library chooses its own algorithm"; "GCC evaluates them right to left on x86-64 and left to right on 64-bit ARM: the same seed gives x and y swapped. Take each draw in a statement of its own." `Normal()` still calls `std::log` (last-bit differences possible).
- Users: `src/rectangle_fit.cc`, `tests/rectangle_fit_test.cc:24,119`, `tests/ompl_search_test.cc:8-11,36`, `src/layout_plan.h:111-124` and `.cc` ("iteration budgets only (no time limit)"; "never compares a freshly planned path with the stored one"), `src/layout_check.*`.
- Gaps to state as consequences: `std::uniform_real_distribution` and `std::normal_distribution` are still used in `src/point_generator.cc:21-63`, `src/rrt.cc:442`, `src/conveyor.cc:123`; `src/cell.cc:45-47`, `src/depth_camera.cc:23` and `src/check_rig.h:53` write their own draws on `mt19937` outputs. Two tests still bound a wall-clock time: `tests/rrt_test.cc:373` (`EXPECT_LT(seconds, 2.0)`) and `:391` (`report.seconds < 0.5` after a 0.05 s limit).

**When.** `5600068` 2026-11-20 created `src/portable_random.h` with a golden-value test. `ae3bb74` 2026-11-20 removed a wall-clock bound. Into `AGENTS.md`: `4c5bf74` 2026-11-23 ("New: ... tests that behave the same on ARM and x86-64").

**Why.** `5600068`: "Two draws inside one call are evaluated right to left by GCC on x86-64 and left to right on aarch64, so 'seed 1' was a different scene on CI, where two bounds set from three local scenes failed". `ae3bb74`: "The median planning time was asserted under 1 s; a shared CI runner measured 1.05 s. The query counts are the machine-independent bound". The header: libstdc++, libc++ and Microsoft's give different numbers for the same seed (`:3-6`).

**What was weighed.** `5600068`: bounds "restated from 10,000 scenes and tested on 500". No alternative generator is mentioned.

**Where else.** `AGENTS.md:73`; the headers above; `docs/flat-rack-cell.md:296` ("another platform's standard library may find other paths"); the brief on OMPL planners seeded from the clock.

---

## 6. A figure comes from a program's run

**What is true now.**
- `AGENTS.md:77-80`: "**A number comes from a run.** A figure in the README, a report or a commit message is reproduced with `run_cell` or a `print_*` program at that commit, not copied from another agent's summary. A change that moves a number in the README's Status updates it there."
- Programs: `run_cell`, `print_profile`, `print_plane_fit`, `print_points`, `print_pallet`, `print_cell`, `print_depth`, `print_perception`, `print_rrt`, `print_plan`, `print_check`, `print_bay`, `check_layout`, `plan_layout`, `gen_bay`, `run_gate`.
- Same habit in documents: `docs/ompl-comparison.md:13-14` "Every figure below was printed by `print_plan` or `run_cell`; the ratios and percentages were computed from those figures"; `docs/layout.md:20-21`; `docs/flat-rack-cell.md:9-10`.

**When.** The habit first appears in `12d1127` 2026-11-20 ("Verify before committing a stage ... reproduce the stage's table with `run_cell` or the `print_*` program and compare with the agent's figures"); present wording in `4c5bf74` 2026-11-23 (its body does not list it; compare `git show 4c5bf74 -- AGENTS.md`). The programs date from `f8d4b94`, `3fb9514`, `83970cc`, `c97c947` (11-19).

**Why.** The old text: the lead compares the stage's table "with the agent's figures". The brief: in run 004 "an agent that wrote none of it checks the document's prose against the raw outputs (it found four wrong figures)"; run 011's "review found real gaps in the checker and every layout, picture and figure had to be made again" (Sizing). No reason is stated in `AGENTS.md` itself.

**What was weighed.** Nothing written.

**Where else.** `AGENTS.md:77`; `docs/ompl-comparison.md:13`; `docs/layout.md:20`; `docs/flat-rack-cell.md:9`.

---

## 7. A second planner (OMPL) behind a switch, the project's own the default

**What is true now.**
- `src/motion_planner.h:149-158` `enum class PlanSearch { kOwn, kOmpl }`; `MotionPlanOptions::search`; `:98-111` "With `search` left alone nothing of OMPL runs."
- `src/run_cell.cc:432` `PlanSearch planner = PlanSearch::kOwn;`; `--planner own|ompl` at `src/run_cell.cc:33`, `src/print_plan.cc:40,202`. `src/ompl_search.h`, `bazel/ompl.BUILD.bazel`, `tests/ompl_search_test.cc`. `README.md:222-224` "the hand-written one is the default".
- `docs/ompl-comparison.md:16-39`: the rule ("The reference is the row `own`. A candidate qualifies on a move when (a) its `found` is at least `own`'s; (b) its clearances ...; (c) its `queries_median` is at most 1.5 times `own`'s and its `ms_median` at most 3 times"), three outcomes, "Verdict: hand-written better" at `:39`. `:7-8` "The verdict follows a rule that was fixed before any of these numbers existed"; `:8-10` "whether OMPL becomes the default is the owner's decision and nothing here recommends for or against it"; `:391` "If OMPL became the default: Facts for that decision."
- Every planning target links OMPL: `BUILD.bazel:1578-1585`; body of `9397bdd` "No target is tagged manual: the planner depends on it."

**When.** `docs/ompl-integration.md` (13678be, fa4d595; 2026-11-23/24; spike branches 11-23); `158cc23` 2026-11-24 "Build: OMPL is the planner's second search, no longer a yardstick" (before: `1d68640` "for comparison only"); landed `9397bdd` 2026-11-24 (merged line `a091cc2` is the report only).

**Why.** Why a switch: body of `9397bdd`: "With the switch left alone nothing changes: no existing test is edited, print_plan's rows for each seed are the bytes they were, and src/rrt.*, src/collision.*, src/path_timing.*, src/pick_place.* and models/ are untouched." Why own is the default: the verdict in the same body ("OMPL's RRT-Connect returns the own planner's paths on the three carries (150 of 150 plans) but needs 2735.0 collision queries at the median against 664.0 on the return, where the own search tries the straight line first; BIT*, ... had 1.053 of the own planner's median duration on one carry, in a row that does not repeat from run to run. The document does not decide whether OMPL becomes the default."). No owner decision to keep own as the default is recorded; the brief says only "own being the default". Why OMPL at all: `docs/ompl-integration.md:3-5` "for deciding how OMPL ... should be brought into this repository, and what the comparison of its planners with the hand-written one in `src/rrt.h` will need."

**What was weighed.** How OMPL comes in: `docs/ompl-integration.md` O1 to O8, recommendation O1 native (`:1383` decision 1: "one of the two built options that changed nothing in the existing build, needed no patch, and compile OMPL with the repository's own flags; of the two, the one without a registry to keep"). The twelve decisions for the owner (`:1380-1393`). Two of those were recommended and not done: decision 5 (a CI query that no control target depends on OMPL; today `motion_planner` depends on it by design) and decision 6 (a CI job of its own for OMPL files; `ci.yml` has six jobs and none). Planners compared: `RRTConnect`, `BiTRRT`, `BITstar`, `AORRTC`, `InformedRRTstar` (`docs/ompl-comparison.md:17-21`).

**Where else.** `README.md:222`, `:306-325`; `AGENTS.md:107`; the brief ("Planning code"); `MODULE.bazel:87-93`.

---

## 8. A cell's layout as data, checked by a program

**What is true now.**
- `layouts/*.json`: 8 files (run 011's five, the flat rack's three). `src/layout.h:1-14`: "One JSON file under layouts/ is one layout, and nothing about a layout is written anywhere else: the model that is looked at (src/layout_model.h) and the checks (src/layout_check.h) are both made from the file."; `LoadLayout` "refuses one with a missing or unknown field, a wrong type, an arm model that is not in src/arm_catalog.h, or a name that is used twice, and says which."
- `//:check_layout` (`src/check_layout.cc`), `tests/layout_check_test.cc`, `layout_scene_test.cc`, `layout_rows_test.cc`; commands at `docs/layout.md:20-35`; `AGENTS.md:116-118`.
- `docs/layout.md:20`: "Five layouts were written as data (`layouts/*.json`, in the form of `src/layout.h`), checked by one program (`src/layout_check.h`) and compared".

**When.** `68a1199` (run 011): "A layout of the new cell is one JSON file under layouts/; src/layout_model.h builds its MuJoCo model, //:check_layout checks one whole bay ... and //:viewer_layout shows it." Form extended at `940a01b` ("Run 011's five layouts load in the new form"). `nlohmann_json` entered for it (`MODULE.bazel:21-22`).

**Why.** The brief (the owner, 2026-11-24): "Layouts will be compared, so a quick way to describe and check a layout is part of the work." `src/layout.h:5-8`: one file is the single source for model and checks. The brief: `check_layout` "checks a layout in about four seconds". No reason recorded for JSON as the format (likely: the registry has `nlohmann_json`).

**What was weighed.** Five layouts compared (`docs/layout.md:126-267`): `below`, `below_each_rail`, `below_rails_by_module_arm` (chosen), `beside` (passes, not better), `above` (fails clearance, kept as the failing case). No alternative to data-plus-checker is recorded.

**Where else.** `AGENTS.md:116,118-119`; `docs/layout.md`; `docs/flat-rack-cell.md:12-20`; `README.md:112-119`; the brief ("The new cell, as designed").

---

## 9. A generated model is committed with its generator and a test that they agree

**What is true now.**
- `models/bay.xml`: `src/bay_model.h:1-17`: "It was once written out by a script that is lost. BayModelXml() is that script again: every position, size and mass in the text follows from the header's constants, so that a change of the header ... is a change of the model by running //:gen_bay and nothing else. tests/bay_model_test.cc fails when the committed file differs from this text by a single byte." Command `bazel run --config=opt //:gen_bay > models/bay.xml` (`bay_model.h:15`, `BUILD.bazel:1668`, `AGENTS.md` map row "The bay's model is generated").
- `tests/bay_model_test.cc:1-8`: "the committed models/bay.xml is, byte for byte, what BayModelXml() returns; when it is not, the message shows the first line that differs and the command that writes the file again".
- No other model is generated: `models/container.xml`, `models/flat_rack.xml`, `models/cell.xml`, `arm.xml` and the rest are written by hand; `tests/container_test.cc`, `tests/flat_rack_test.cc` load the file named in `MODEL_PATH` and check what it holds against the header, with no generator.
- A layout's model is built at run time and never committed (`src/layout_model.cc:26-34`: "nothing is written to disk").
- `plans/*.json` (two files) are written by `plan_layout --out PATH` (`src/plan_layout.cc:3,265`; `docs/flat-rack-cell.md:13-18`). The agreement test is a different kind: `tests/plan_replay_test.cc` verifies the committed plan without planning (`VerifyPlan`) against the layout file's SHA-256 recorded in the plan (`src/layout_plan.h:313`), and `src/layout_plan.h:118-121`: it "never compares a freshly planned path with the stored one". So a plan has a generator and no byte-for-byte test, on purpose (another platform's library may find other paths: `docs/flat-rack-cell.md:296`).

**When.** `1213319` 2026-11-24 committed `models/bay.xml`; `68a1199` added `src/bay_model.*`, `//:gen_bay`, `tests/bay_model_test.cc` ("models/bay.xml is generated by //:gen_bay from src/bay_hardware.h, and a test fails when the committed model differs; with the count at ten the generator printed the model of 1213319 byte for byte."). Plans: `940a01b`.

**Why.** The bay went from ten modules to nine (brief: decided 2026-11-24), so the model had to change and its writer was lost (`bay_model.h:6-7`). The brief: "Accepted from run 007's report, each to be done when a spec asks for it: ... generated models get a committed generator". `68a1199`: "A bay is nine modules and ten rails, from one constant".

**What was weighed.** Nothing written.

**Where else.** `AGENTS.md` map row "The bay's model is generated: change the header, then regenerate"; the brief; `src/bay_model.h`; `tests/bay_model_test.cc`. `docs/bay-hardware.md` has no sentence on generation.

---

## 10. Programs that print tables print CSV; `--pretty` for an aligned table

**What is true now.**
- `src/print_profile.cc:1-3`: "The output is CSV, for a plotting tool. With --pretty it is a table of aligned columns, for reading in a terminal."
- `--pretty` is in `run_cell`, `print_plan`, `print_perception`, `print_plane_fit`, `print_profile`, `print_check`, `check_layout`, `plan_layout`. `AGENTS.md:109` "Tables | `run_cell`, `print_perception`, `print_plan`, `print_check`, each with `-- --pretty`"; `README.md:56` "`--pretty` (aligned table; CSV without it)"; `README.md:72-85`.
- Not uniform: `print_bay`, `print_depth`, `print_points`, `print_rrt` print CSV and have no `--pretty`; `print_cell`, `print_pallet`, `run_gate`, `gen_bay` print no table. `check_layout` and `plan_layout` print CSV followed by `#` lines (`docs/layout.md:22-26`).

**When.** `f8d4b94` 2026-11-19 (`print_profile`: "prints both shapes for one move as CSV, or as an aligned table with --pretty"), `3fb9514` (`print_plane_fit`), `83970cc` (`run_cell`), `c97c947`, `a2f4e85`; layout programs `68a1199`, `940a01b`.

**Why.** Only the program comment: CSV "for a plotting tool", aligned "for reading in a terminal". Likely (guess): a figure is reproduced from a program (decision 6), so the output has to be machine-readable and diffable; `9397bdd` says "print_plan's rows for each seed are the bytes they were".

**What was weighed.** Nothing.

**Where else.** `README.md:56,72-85`; `AGENTS.md:109`; each program's header comment; `docs/layout.md:22`.

---

## 11. Kinematic checking and planning before physics (spec 012)

**What is true now.**
- `docs/flat-rack-cell.md:7-9`: "The cell is designed, not built, and kinematic: arms, tools, parts and envelopes are checked at poses and along planned motion, and nothing steps the physics."; `:32-40` under "What is allowed" ("The owner set these limits on 2026-11-24 for the first cell"): "The work is kinematic only."; `:633` "Physics, sensing and controllers" under "What was not considered".
- `src/layout_plan.h:12-13` "Kinematic only: nothing here steps the physics."; `src/layout_model.cc:26-28` "<actuator> and <keyframe> are removed: the model is for looking and measuring, its poses are set in qpos".
- The brief: "The layout's model has no actuators and its viewer does not step physics." and "Nothing in the new cell moves yet: it is checked at poses." `README.md:116-119`: "It has no controllers, no cameras and no gate: it is kinematic." `docs/layout.md:15-18` "the checks are made at poses"; `:796` "Paths between poses. Only poses are checked, as the brief scopes it."
- The one physics test of the new parts: `tests/locate_physics_test.cc` (the locating nose).

**When.** Poses only: `68a1199`. Planned motion, still kinematic: `940a01b` ("builds one whole bay as planned motion"; `//:plan_layout`, `tests/plan_replay_test.cc`).

**Why.** Scope, as the owner set it: `docs/flat-rack-cell.md:32,40`; the brief ("A working system comes first, then iteration", owner, 2026-11-24). No cause of the order (cost, speed, an alternative) is recorded. Facts that bear on it: `check_layout` takes about four seconds (brief); the time model was "low by 1.7 to 3.5 times on seven moves" against the planner (brief; `docs/layout.md:661-719`), which is why `940a01b` plans every move.

**What was weighed.** Time model against planner (`docs/layout.md:661`); moves planned one arm at a time with the others at wait, not jointly (`docs/flat-rack-cell.md:271-291`).

**Where else.** `README.md:116`; `docs/flat-rack-cell.md:7,40,633`; `docs/layout.md:12-18,796`; `src/layout_plan.h:13`; the brief.

---

## 12. The stand-in cell is kept untouched until it is replaced

**What is true now.**
- The brief: "the stand-in cell (`models/cell.xml`, `src/pick_place.*`, the gate) still has ten slots and a pallet of thirty, on purpose: it is not being converted, it is being replaced." It also lists what carries over (`src/rrt.h`, `src/path_timing.h`, `src/viewer_lib.h`, `src/mj_handles.h`), what needs changes (`src/ik.h`, `src/collision.h`, `src/motion_planner.h`, tied to `cell::Arm`) and what does not (`src/pick_place.*`, `src/pallet.*`, `src/conveyor.*`, `src/check_rig.*`, `models/cell.xml`).
- Commit bodies: `1213319` "The cell does not change: models/cell.xml loads neither part and still carries its old square tube."; `68a1199` "The stand-in cell is unchanged; README.md and AGENTS.md name the new pieces."; `9397bdd` "src/rrt.*, src/collision.*, src/path_timing.*, src/pick_place.* and models/ are untouched" (it did edit `src/trial_runner.{h,cc}` and `src/run_cell.cc` to carry the planner choice).
- Documents: `AGENTS.md:111,116` "beside the cell, not in it"; `README.md:107-108` "beside the cell, which still uses its old square tube"; `docs/bay-hardware.md:769-771` "Against the cell today ... Nothing here is changed by this document."
- `AGENTS.md` does not say "do not edit the stand-in cell". The phrase "stand-in cell" is in the brief, the spec and the body of `68a1199` only; the documents call it "the cell".
- Verified: `git diff --stat 9397bdd 940a01b -- models/cell.xml src/cell.* src/pick_place.* src/pallet.* src/conveyor.* src/check_rig.* src/trial_runner.* tests/gate_test.cc docs/gate.md` prints nothing.

**When.** "Beside, not in": `1213319` 2026-11-24. "Replaced, not converted": the brief, of 2026-11-24 or 25 (the brief names `68a1199`, run 011).

**Why.** The brief: "The cell is to be rebuilt on the real parts of `docs/bay-hardware.md` ... and that work includes its layout (decided 2026-11-24)"; the stand-in "is not being converted, it is being replaced". The gate runs on it and its tolerance is the owner's (`docs/gate.md:35` "an assumption chosen by the project owner"; `AGENTS.md:62-65` "What 'correct' means is the owner's"). The spec says no spec has yet said when it is removed.

**What was weighed.** Nothing written.

**Where else.** The brief ("Where the project is going"; "A bay is nine modules"; "The new cell, as designed"); commit bodies of `1213319`, `68a1199`, `9397bdd`; `README.md:107`; `AGENTS.md:111,116`; `docs/bay-hardware.md:769`.

---

## 13. One `BUILD.bazel` with `# BEGIN x` / `# END x` sections

**What is true now.** `BUILD.bazel` at the top is the one build file for code; `bazel/BUILD.bazel:1-2` "Holds .bzl files only. A directory needs a BUILD file before Bazel can load anything from it." 29 `# BEGIN` sections, from `# BEGIN arm` (`BUILD.bazel:183`) to `# BEGIN layout_plan` (`:2055`); lines 1-182 (`mj_handles`, `step_arm`, `profile`, `plane_fit`, early `print_*`) have none. `AGENTS.md:55-56`: "Two agents never own the same file. `BUILD.bazel` is the one shared file: it has a `# BEGIN x` / `# END x` section per piece, and each agent edits only its own." The brief: "`BUILD.bazel` at the top is the only build file for the code. Repository rules live in `bazel/*.bzl`."

**When.** One file since `8adf3bd` 2026-11-19; the first `# BEGIN` (by `git log -S'# BEGIN'`) in `83970cc` 2026-11-19. Written in `AGENTS.md` at `12d1127` 2026-11-20 ("Docs: AGENTS.md, build rules, code map and how a stage is split among agents"): "`BUILD.bazel` is one file with a `# BEGIN x` / `# END x` section per piece. Edit only your own section."

**Why.** Dividing the work among agents (`AGENTS.md:53-57`). Nothing recorded on visibility or build performance. Likely (guess): one file shows every dependency to every agent.

**What was weighed.** Only `docs/ompl-integration.md:1391` decision 5: "A separate package with `visibility` would also do it, against the rule of one build file."

**Where else.** `AGENTS.md:55`; the brief ("Conventions beyond AGENTS.md"); `docs/ompl-integration.md:1375,1391`.

---

## 14. Flat `src/` and `tests/`; namespace `cell` and nested ones

**What is true now.** `src/` has 42 headers and 57 `.cc`; `tests/` 41 files; no subdirectories; includes are `"src/x.h"`. `namespace cell` in 60 files; `cell::layout` in `src/layout.h:98`, `layout_check.h`, `layout_model.h`, `layout_scene.h`, `layout_plan.h:143`, `layout_rows.h:71`, `arm_catalog.h:27` and their `.cc`; `cell::bay` in `src/bay_hardware.h:87` and `src/bay_model.h:58`; `cell::container` in `src/container.h:42`; `cell::flat_rack` in `src/flat_rack.h:50`. No document states a rule on namespaces or on the flat layout.

**When.** Flat since `8adf3bd`; `cell` since `f8d4b94`/`83970cc` 2026-11-19. `cell::bay` and `cell::container`: `1213319` 2026-11-24. `cell::layout`: `68a1199`. `cell::flat_rack`: `940a01b` (by `git log -S'namespace cell::...'`).

**Why.** No reason recorded. Observed: the nested ones are the new cell's pieces; the stand-in's code is all in `cell`.

**What was weighed.** Nothing.

**Where else.** `AGENTS.md:98-121` (the map groups files by piece, not by directory).

---

## 15. Each header opens with a long comment

**What is true now.** All 42 headers begin with a `//` comment; shortest: `src/mj_handles.h` 6 lines, `arm.h` and `point_generator.h` 9, `joint_move.h` 11, `check_rig.h`, `pallet.h`, `score.h` 12; `src/pick_place.h` has 240 of 1,173 lines (BRIEF.md and `AGENTS.md:95-96`: "Each header opens with a long comment that explains the piece"). The brief: "Comments are plain, complete sentences; headers open with a long comment that explains the piece."

**When.** In code since `8adf3bd` (`src/mj_handles.h`); in `AGENTS.md` at `12d1127` 2026-11-20 ("Map. Read the top-of-file comment of a header before its body; they are long.") and in its present wording from `4c5bf74`.

**Why.** "Read the top-of-file comment of a header before its body" (`12d1127:AGENTS.md:32`); no further reason.

**What was weighed.** Nothing.

**Where else.** `AGENTS.md:95`; the brief.

---

## 16. Tests: one `tests/<piece>_test.cc` per piece, GoogleTest, the gate tagged

**What is true now.**
- 41 test sources in `tests/`; 41 targets: 40 literal `cc_test(` blocks plus `check_cycle_batch_test`, generated from `check_cycle_test.cc` by a loop (`BUILD.bazel:1466-1498`). Sizes: 22 `small`, 15 `medium`, 4 `large` (`camera_cycle_test` `:815`, `module_finder_test` `:986`, `motion_planner_test` `:1220`, `gate_test` `:1553`). No `timeout =` anywhere; `shard_count = 4` on `motion_planner_test` (`:1224`, "The statistics, the execution and the rest in separate shards") and `gate_test` (`:1558`).
- The gate: `BUILD.bazel:1552-1564` `tags = ["gate"]`; `.bazelrc:37` `test:agent --test_tag_filters=-gate`; `ci.yml:67` `--test_tag_filters=-gate` for "other tests".
- `.bazelrc:29-37`: "One full run of the suite already fills this machine's cores and, unlimited, three at once ran it out of memory (measured 2026-11-22). The limit keeps each worktree to three cores. The gate (about 4 minutes on 4 cores) is left out: the lead runs it once, alone, before a commit, and CI runs it."
- `MODULE.bazel:24-25` googletest, `dev_dependency = True` ("Only this repository's tests need it"); every test links `@googletest//:gtest`.
- `AGENTS.md:11-13`: "While iterating, only your piece's tests".

**When.** GoogleTest and a test file per piece from `8adf3bd`, `f8d4b94`, `3fb9514` (their bodies: "Tests assert ..."). The gate and its tag: `dd03149` 2026-11-20. `--config=agent` and the batch split: `5d8fb21` 2026-11-22 ("BUILD.bazel: the CheckCycles suite of check_cycle_test runs as check_cycle_batch_test, from the same source"; "AGENTS.md: named tests while iterating, the suite without the gate once at the end, the gate and the unlimited suite for the lead alone").

**Why.** Per piece: so an agent runs only its own (`AGENTS.md:11-13`). The gate out of `--config=agent`: the `.bazelrc` comment above. The gate tagged: `docs/gate.md:3-5` "It is part of `bazel test --config=opt //...` and is tagged `gate`". Sizes: no reason recorded (likely: `large` for tests that run whole cycles; guess).

**What was weighed.** Nothing.

**Where else.** `AGENTS.md:9-31`; `docs/gate.md:1-5,65-80`; `.bazelrc:29-37`; `ci.yml`.

---

## 17. CI: six jobs dividing the suite, one disk cache per job

**What is true now.** `.github/workflows/ci.yml:36-40` matrix `part: [cycle tests, other tests, gate 1, gate 2, gate 3, gate 4]`; `:44-56` `setup-bazel` with `disk-cache: ${{ github.workflow }}-${{ matrix.part }}`, comment `:47-51` "Each part keeps its own, since each builds and runs different tests."; `CYCLE_TESTS` at `:22-28`; gate shards by `GATE_TOTAL_SHARDS=4`, `GATE_SHARD_INDEX`, `GATE_THREADS=2` (`:76-80`); `timeout-minutes: 30` (`:35`); `ubuntu-latest` (x86-64, two cores); on push to `main`, pull request, and `workflow_dispatch` with `rerun_tests` (`:3-11`). The brief: the owner is moving the six jobs to arm64 (`ubuntu-24.04-arm`), not yet done.

**When.** `8adf3bd` 2026-11-19 (one job); `dd03149` (tests run optimized); `ae3bb74` 2026-11-20 (limit 30 to 60 minutes: "a cold build with the gate took 25.5 minutes once and over 30 the next time"; now 30 per job again); `1d2c098` 2026-11-22 "CI: the suite divided over six jobs that run side by side"; `014ba3a` 2026-11-22 (a run by hand can run every test again).

**Why.** `1d2c098`: "the cycle tests, the other tests, and the gate in four jobs of two pallets each, every one with its own disk cache. In one job on a two-core runner the suite took 33 minutes." and "gate_test: GATE_TOTAL_SHARDS and GATE_SHARD_INDEX name one shard from outside, since Bazel does not let --test_env set its own TEST_ variables." `ci.yml:31-34`: "a runner has two cores, and in one job the suite took 33 minutes." The brief: "Org does not use pull requests, so CI runs when work lands on `main`; do not start runs by hand, and do not wait for CI" (owner, 2026-11-24).

**What was weighed.** Only `docs/ompl-integration.md:1392` decision 6 (a job of its own for OMPL files: "The six existing jobs should not pay for it"), recommended and not made.

**Where else.** The brief ("CI"); `docs/gate.md:65-80`; `.bazelrc:29-37`; `AGENTS.md` mentions the gate's CI run only through `docs/gate.md`.

---

## The three decisions whose reason I could not find

1. MuJoCo as the simulator, with perception, planning and control hand-written in C++20 (decision 3): the repository says what was done and never why; the first commit says "not chosen yet", the choice came in `8adf3bd` with a subject-only message. Reasons exist only for `--config=opt` and the OMPL flags.
2. Bazel with bzlmod and the pinned 9.3.0 (decision 1), and JSON as the layout format (decision 8): no sentence says why these over the alternatives. (Also unrecorded: why the namespaces nest, decision 14; why own is the default planner beyond "nothing changes" and the verdict, decision 7; why each test has its size, decision 16.)
3. Flat `src/` and `tests/` with `cell::layout`, `cell::bay`, `cell::container`, `cell::flat_rack` (decision 14), and why the kinematic order (decision 11) beyond "as the owner scopes it": no reason anywhere.

---

## Where the decisions about the cell's design are written (R13)

Line numbers at `940a01b`. The sections' `ours` counts come from `awk '/^## /{sec=$0} /\| ours \|$/{c[sec]++}' docs/bay-hardware.md` (79 rows in all).

| Place | Heading | Starts at line | Decisions there |
|---|---|---|---|
| `docs/layout.md` | "## Decisions" ("Every decision the brief left open, with its reason.") | 720 | 15 numbered, to line 794; "What was not considered" follows at 796 |
| `docs/layout.md` | "## The comparison", "### What makes `above` fail", "### Why `beside` was not chosen, though it passes", "### Three ways of putting the rails on, by time" | 177, 237, 256, 219 | the choice among five layouts: 1 chosen, 1 failed, 1 passed and not chosen; 3 ways of putting rails on compared |
| `docs/layout.md` | "## The constraints", "## What had to be possible at all" | 46, 84 | feasibility findings behind its decisions 1 and 2; no numbered list |
| `docs/layout.md` | notice at top | 3 | says `docs/flat-rack-cell.md` is current and this document "stands as the record of the closed container (run 011)" |
| `docs/flat-rack-cell.md` | "## Decisions" | 546 | 15 numbered, to line 629; "What was not considered" at 631 |
| `docs/flat-rack-cell.md` | "## What is allowed" ("The owner set these limits on 2026-11-24") | 32 | the owner's limits, 5 `allowed` rows; the owner's, not the engineer's |
| `docs/flat-rack-cell.md` | "### How many pin arms (R12)" | 130 | 1 (three pin arms, compared with one and two) |
| `docs/flat-rack-cell.md` | "## Margins", "### The values used and why" | 406, 408 | the margin values (decision 12 of "Decisions" repeats the changes) |
| `docs/bay-hardware.md` | "## The reference" | 39 | 1 ("Three things decided it": most shipped tracker, round tube, best documented) |
| `docs/bay-hardware.md` | "## Torque tube", "## Module", "## Rail", "## Fastenings", "## The bay" | 70, 118, 161, 201, 256 | `ours` (designed) rows: 6, 5, 7, 8, 3 |
| `docs/bay-hardware.md` | "## Holding the tube" | 286 | 5 `ours` rows |
| `docs/bay-hardware.md` | "## Locating a module before pinning" | 385 | 25 `ours` rows; "designed here; no source was read for any of it"; rests on "the owner's decision of 2026-11-24" |
| `docs/bay-hardware.md` | "## The container" (holds the flat rack's table) | 599 | 19 `ours` rows |
| `docs/bay-hardware.md` | "## Against the cell today" | 769 | 6 `ours` rows; a comparison with the stand-in, not a decision |
| `docs/ompl-comparison.md` | "## Verdict" (the rule) | 16 | 1 (the verdict rule, fixed beforehand); "## If OMPL became the default" (391) holds facts for the owner, not a decision |
| `docs/gate.md` | "## The tolerance" | 33 | 1 (3 mm, 0.15 degree, at rest: "an assumption chosen by the project owner"; the 2 mm height check is "the runner's own choice") |
| `docs/ompl-integration.md` | "## Decisions for the owner" (a build decision, not the cell's design) | 1380 | 12 rows, recommended and not recorded as decided by the owner |

Overlap: flat-rack decision 9 and 11 say "run 011's" (the holder, the order of work), so those two are written once in `docs/layout.md` and pointed to.
