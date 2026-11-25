# Attack on the checks, the measures and the principles (run 013-r2, agent `attack`)

Done in the scratch clone `clone3` (HEAD `242f9ea`). Before the work I copied in the lead's
`tools/architecture/`, `docs/architecture/*.md`, `.bazelrc` and `.clang-tidy` from `repo`.
I copied them a second time after the lead changed `deps.json`: `src/arm_catalog.h` moved to
`layout`, which brought `P1.namespace` from 1 to 0. Every figure below was taken after the
second copy. Unless a line ends in `(bazel)`, the figures come from
`PATH=/usr/bin:/bin tools/architecture/measure.sh`, with Bazel off the PATH, diffed against
the same command on the untouched tree.

Helper scripts are in `out/attack/`: `mdiff.sh` (measure plus deps.py, diffed), `restore.sh`,
`battery.sh` (the deps.py bypasses, can be run again), `patterns.py` (snippets through the
deps.json patterns) and `where.py`. After each attempt I put `src`, `tests`, `BUILD.bazel` and
`deps.json` back. When I finished, `git status --short` in clone3 showed only the copied tools
and documents.

## Table, most serious first

| Check or line | Verdict | The attack that worked | Proposed cure |
|---|---|---|---|
| deps.py: what counts as an `#include` | **bad** | A forbidden include got through deps.py (exit 0) in five spellings: a macro include (`#define H "src/pick_place.h"` / `#include H`), a line splice (`#inc\` + newline + `lude`), the digraph `%:include`, `#import` and `#include_next`. `g++ -E` honours the first three. Example: `src/motion_planner.cc` (arm) included standin's `src/pallet.h` through a macro with no BUILD change. It compiles because `//:collision` brings pallet.h in transitively and there is no layering_check. | Splice backslash-newlines before scanning. Treat `%:` as `#`. Fail on any `#`/`%:` directive whose name begins with `include` or `import` unless its operand is a literal `"..."` or `<...>`. |
| deps.py: which files are read | **bad** | Only `.h` and `.cc` are scanned. A tracked `src/rrt.cpp` in `//:rrt`'s srcs that includes pick_place.h passed. So did a tracked `src/rrt.inc` (in `planning` by the pattern `src/rrt.*`, listed in hdrs, included by `src/rrt.h`). | Read every tracked file under src/ and tests/, or fail (rule a) on any extension other than .h/.cc. |
| deps.py: BUILD attributes it does not read | **bad** | These all passed: `copts = ["-include", "src/pick_place.h"]` (a forced include), `defines = ["ATTACK_H=\"src/pick_place.h\""]` together with `#include ATTACK_H`, and `implementation_deps = [":pick_place", "@mujoco"]` (unused, unseen). | Allow an explicit list of attributes on cc_* rules. Fail on `includes`, `textual_hdrs`, `implementation_deps`, `defines`/`local_defines` and `include_prefix`/`strip_include_prefix`, and on copts other than a listed few. Or read implementation_deps as deps. |
| deps.py: the standard library | **bad** | `_STD = ^[a-z_0-9]+(\.h)?$` takes `<tinyxml2.h>` and `<lodepng.h>` for the standard library. These are MuJoCo's own outside dependencies: `@tinyxml2` is a `deps` of MuJoCo's `src/xml` targets, so its headers reach every target that lists @mujoco. Added to `src/layout_plan.cc`, deps.py printed ok and P4 did not move. | Replace the regex with the fixed list of C++20 and C standard headers. |
| deps.json itself (may_include, groups, frozen, allowed, departures) | **bad** | I widened planning's `may_include` to `standin` and added `#include "src/pick_place.h"` to rrt.cc. deps.py printed ok and no measure line moved. I set `"frozen": true` on `arm` and `layout`: P1.frozen went from 4 to 5, so P1 = P1.frozen and the P1 target is met. P5.frozen went from 7 to 13, P7.frozen from 18 to 29, P11.frozen from 3 to 6. A new departure entry is accepted. Only P1 moved (5 to 6), and a baseline refresh hides that. | `--check` reads deps.json at the merge base (`git show $(git merge-base HEAD main):tools/architecture/deps.json`). It fails when departures gains an entry, when may_include, outside, truth/portable allowed or silenced allowed widens, when a group or frozen flag is added, or when a single_home entry or pattern is removed. Simplest form: measure prints a line for each of these sizes, e.g. `P1.rules <allowed group pairs>`, `P3.allowed <files>`, `frozen <files>`. Each "may not grow" and shows in the baseline diff. |
| P2 / P2.unrun / P2.untested | **bad** | Eleven lines `<t>.help \| //:<t> \| --help \| -` in programs.txt took P2 from 6 to 0 and P2.unrun from 11 to 0 (bazel). An empty `TEST(ViewerLib, Links) {}` with deps on :viewer_lib took P2 from 6 to 4 and P2.untested from 24 to 22. Deleting all 138 EXPECT/ASSERT of tests/rrt_test.cc moved nothing. A cut `re:^(.*)$` in programs.txt would blank an output and nothing would count it. | Add a second count, `P2.weak`, that may not grow: test files with no EXPECT/ASSERT, programs.txt lines whose arguments are `--help`, lines whose capture is under 3 lines or exits non-zero, and cuts whose pattern matches a whole line. List the display programs by name in deps.json as excepted; do not count them as captured. The principle should say that how much a test asserts is held by review. |
| P11 / P11.h / P11.cc / P11.comment | **bad** | I stripped src/pick_place.h down to a 5-line opening comment with no other comments (1173 lines to 374). P11.h went from 2 to 1, yet this is the 240-line comment that AGENTS.md says to read first. Cutting layout_check.cc at lines 900 and 1800 into `src/layout_check.part2.cc` and `.part3.cc` took P11.cc from 5 to 4. The `src/layout_check.*` pattern puts both pieces in `layout` with no deps.json edit. Five empty `//` lines satisfy P11.comment, and 3,000 lines added to layout_plan.cc moved nothing. | Add `P11.over`, the sum of lines over the limits, so growth shows. Count comment words, not lines: an opening comment of at least 40 words, plus a total of comment lines under src/ that may not fall. Count .cc files with no header of the same stem and no `main` (the random-split piece). Make the bay_hardware.h exception data. |
| P5 | **bad** | Three pointless `#include`s (cell.h, pick_place.h, score.h) in tests/trial_runner_test.cc, plus removing their departures, took P5 from 16 to 13. When tests/path_follow_test.cc dropped `#include "src/arm.h"` and `:arm` and still used `cell::Arm` through ik.h, nothing moved. | Add clang-tidy 18's `misc-include-cleaner` as a T line that may not grow. It flags both an unused include and a symbol used through someone else's include. |
| P7 and P7.* | **bad** | Calls written `(&mj_geomDistance)(...)` in three files and pi written `0.314159...e1` in three took P7 from 36 to 30 (geom_distance 9 to 6, pi 21 to 18). The forms `kModuleLength{2.3}`, `{2.30, 1.10, 0.035}`, `kGateOffset{0.003}` and `position = 0.003` are not counted. 18 of the 36 places are in tests/, which the Exception allows. 8 of the 9 geom_distance places (stand-in and arm) and the gate's place (hardware) cannot take the fact from its home without an include P1 forbids. | Leave tests/ out of P7, or drop the Exception. Scope geom_distance to the layout groups, or give it a home in `base`. Patterns: `\bmj_geomDistance\b` with no parenthesis; `3\.14159\|M_PI\|acos\s*\(\s*-1`; value forms `\b2\.30*\b`, `\b0?\.0030*\b\|3e-3`, with `=` or `{`. |
| P6 | **bad** | One `using UniformReal = std::uniform_real_distribution<double>;` in the allowed src/portable_random.h, used by rrt.cc, took P6 from 9 to 8 with the same draws. `std::shuffle`, a `steady_clock` deadline, `high_resolution_clock`, `clock()`, `drand48`/`random()`/`rand_r` and `generate_canonical` added to layout_plan.cc moved nothing. src/rrt.cc:451, the clock-stopped search that the Why names, is not counted, because deps.json exempts steady_clock. | Add patterns for `shuffle\|sample\|high_resolution_clock\|\bclock\s*\(\|drand48\|\brandom\s*\(\|rand_r\|generate_canonical`. Count `steady_clock` outside an explicit list of files where it is only printed, with rrt.cc as a counted departure. Count `_distribution` in portable_random.h as a line that may not grow. |
| P9 | **bad** | These moved nothing (P9 stayed 0): `_Pragma("GCC diagnostic ignored ...")`, `// reason: kept. NOLINT` (clang-tidy 18 honours NOLINT anywhere in the comment: "Suppressed 2 warnings (2 NOLINT)", out/attack/tidy/t.cc), `#pragma GCC system_header`, `copts = ["-isystem", "."]` and `features = ["-treat_warnings_as_errors"]`. Three `(void)x;` silencings exist today (src/layout_plan.cc:1187, tests/layout_model_test.cc:513-514). Removing a check from .clang-tidy lowers T lines unseen. | Add patterns: `_Pragma\s*\(`, `NOLINT` anywhere in a comment, `#\s*pragma\s+GCC\s+system_header`, copts `-isystem`, a feature string starting with `-`, `\(void\)\s*\w+\s*;`. Add a line `T.checks <n>` for enabled checks that may not fall. Either the Rule allows `[[maybe_unused]]`/`(void)` or they are counted. |
| P10 | **bad** | The stale command at docs/layout.md:667, the Why's own example, is not counted because its backtick span crosses a line break. The truthful sentence at layout.md:11 ("no longer exists") is counted, and taking its backticks off took P10 from 5 to 4. Stale labels and paths in a fenced block, in a wrapped span, in a Markdown link to `../src/print_layout_moves.cc`, as a bare file name, or marked `(gone)` are not counted. principles.md itself is counted (lines 303 and 304). | Join each paragraph before matching backticks. Scan fenced blocks for `//:` labels and paths. Check relative links in every doc, not only in docs/architecture and docs/adr. Mark gone names in principles.md with ` (gone)`. |
| P3 | **bad** (the pattern is weaker than the existing test) | Two added helpers in the allowed src/cell.h (`ModulePoseOf(i)` returns TrueModulePose, `scene()` returns scene_), used by trial_runner.cc, took P3 from 5 to 4. P3 does not count `d->geom_xpos[`, `site_xpos`, `xipos`, `sensordata`, `p = d->xpos; p[i]`, `std::span(d->xpos, ...)` or `&Cell::TrueModulePose`. Of the 5 places counted today, 4 are scoring or reporting (trial_runner, print_perception, print_cell, view_cell); only print_plan.cc:287 uses truth as a planning input. A new file named `src/cell.x.cc` or `src/score.x.cc` inherits the allowance. Splitting layout_check.cc raised P3 (exact-name allowance). | Use the forbidden list of tests/camera_cycle_test.cc:993 (xpos as a substring, qpos, contact, ModulePose, HeldPose, ...) plus `xipos`/`sensordata`. Write allowed as exact file names. Put the scoring programs under allowed as "the scorer", so that P3 counts control code. Rule: an allowed file offers truth only under a `True...`/`true_` name. Review holds the rest. |
| P4, P4.group, P4.mujoco* | **limited** | MODULE.bazel with `use_extension(...)`/`use_repo(extra, "fmt")` and `git_override(... branch="master")`: P4 counted neither. An `http_archive` from a branch URL with `integrity = "x"` counts as pinned. tests/arm_test.cc dropped `<mujoco/mujoco.h>` and `@mujoco` while keeping 9 `mj_` uses: P4.mujoco went from 67 to 66 and P4.mujoco_files from 70 to 69. `src/rrt.h` including `src/mj_handles.h` puts MuJoCo into `planning` while P4.group stays 0. Splitting a file raises P4.mujoco_files. | Count `use_repo` names and `*_override`. Treat a pin as a 64-hex sha256, a `sha256-` integrity or a 40-hex commit. Take P4.group over the transitive closure of project includes. Count P4.mujoco_files by target, not file. |
| P1 (headline) | **limited** | See the include rows above (a macro include of a header available transitively). Forward declarations and an `extern` declaration of score.cc's `MeasureStackDisturbance` in planning's rrt.cc move nothing; that is a real dependency no include scan can see. The unit is (file, header) pairs, not "#include lines". | The include cures above. Change "lines" to "file-to-header pairs". The principle should say that forward declarations of another group's names are dependencies held by review. |
| P1.namespace | **limited** | `#define NS_LAYOUT namespace layout` / `NS_LAYOUT {` in src/rrt.h, or the same in rrt.cc (.cc files are not read): 0 stays 0. | Read .cc files too; count `#define` bodies that contain `namespace`. |
| P8 | **bad** | `"test": "tests/plan_replay_test.cc"` on the flat_rack_2 plan (that test does not read it) and a `no_test` for MODULE.bazel.lock took P8 from 2 to 0. `if (true \|\| committed == generated)` in tests/bay_model_test.cc:100 moved nothing. Deleting the bay.xml entry, or committing a new generated file, moves nothing. | The `test` field names a test target whose `data` lists the committed file (deps.py would read `data`). `P8.no_test` may not grow. Every tracked file under `plans/` (and the `models/` that are generated) must be matched by an entry. |
| deps.py: BUILD shapes | limited | A target made by a macro from a .bzl (`my_cc_test`) has unchecked deps: deps.py printed ok, the summary named the rule kind, and P5.files rose by 1. A comprehension with `if` crashes with a traceback (exit 1, fails closed). `cc_test(**{...})` is refused by Bazel 9 itself. | Fail when a rule of an unknown kind has srcs or hdrs. Catch Unknown in comprehensions and report the line. |
| deps.py: spellings, forwarding, subdirectories, alias, data, labels | sound | `"../src/x.h"`, `<src/x.h>` and `"./src/x.h"` fail. Spaces, a trailing comment and `/* */` before the name fail. `#if 0` fails (over-counts, fails closed). A raw string passes, rightly. A forwarding header (planning or subdirectory, tracked or not) fails. `:x`, `//:x` and an alias resolve. `profile` and `@//:profile` fail as unknown. `data` carries no headers. Rules a, b, c, d, e, f and g each fire and name the place. | (Only the message for `../src` should name the direction rule.) |
| P1.cycles | sound with a stated limit | Only .h-to-.h edges; a cycle through an `.inc` is not seen. | Covered by "read every file". |
| P8.unmatched, P4.unpinned (bazel_dep) | sound | none | |

## Acceptance cases

1. `sed -i '1i #include "src/pick_place.h"' src/rrt.h` followed by `python3 tools/architecture/deps.py`:
   ```
   src/rrt.h: includes src/pick_place.h; group planning may not include group standin (deps.json may_include, rule b)
   //:rrt: its files include a header of //:pick_place, which is not in its deps (rule e)
   src/motion_planner.h: the project's headers include each other in a cycle: src/motion_planner.h -> src/rrt.h -> src/pick_place.h -> src/motion_planner.h (rule d)
   ```
   exit 1. Pass.
2. `deps = ["@eigen", ":profile"]` on `//:rrt` (BUILD.bazel:1083) gave `//:rrt: deps lists //:profile, none of whose headers its files include (rule f)`, exit 1. Pass.
   With `@mujoco` (and with `@mujoco//:mujoco`): `//:rrt: deps lists outside library mujoco, which none of its files include (rule h)`, exit 1. Pass.

## 1. deps.py, attempt by attempt

Each attempt was inserted after line 1 of `src/rrt.cc` (group planning), then deps.py was run.

| Edit | deps.py printed | Gets through? | Should it? |
|---|---|---|---|
| `#include "../src/pick_place.h"` | `src/rrt.cc:2: includes ../src/pick_place.h, which is not a tracked file`, exit 1 | no | no |
| `#include <src/pick_place.h>` | `... which is no outside library of deps.json and not the standard library`, exit 1 | no | no |
| `#include "./src/pick_place.h"` | `... which is not a tracked file`, exit 1 | no | no |
| `  #  include   "src/pick_place.h"   // trailing` | rule b, exit 1 | no | no |
| `#include /* x */ "src/pick_place.h"` | rule b, exit 1 | no | no |
| `#if 0` / include / `#endif` | rule b, exit 1 | no (over-counts) | acceptable: fails closed |
| `R"(` newline `#include "src/pick_place.h"` newline `)"` | ok, exit 0 | yes | yes, it is not an include |
| `#define ATTACK_H "src/pick_place.h"` / `#include ATTACK_H` | **ok, exit 0** | **yes** | no |
| `#inc\` newline `lude "src/pick_place.h"` | **ok, exit 0** | **yes** | no |
| `%:include "src/pick_place.h"` | **ok, exit 0** | **yes** | no |
| `#import "src/pick_place.h"` | **ok, exit 0** | **yes** | no |
| `#include_next "src/pick_place.h"` | **ok, exit 0** | **yes** | no |
| function-like macro building the name `ATTACK_H(pick_place)` | **ok, exit 0** | **yes** | no |

A preprocess-only check, `g++ -E -P -I. a.cc` on a 5-line file (out/attack/pp/), printed
`int from_target;` three times. So the macro, splice and digraph forms really do include.

Macro include of a transitively available header, with no BUILD edit. Two lines at the top of
src/motion_planner.cc: `#define MP_PALLET_H "src/pallet.h"` / `#include MP_PALLET_H`. deps.py
printed `ok`, exit 0, and no measure line moved. BUILD.bazel, .bazelrc and MODULE.bazel never
mention `layering_check`, and `//:motion_planner` depends on `//:collision`, which depends on
`//:pallet`. For an existing departure, the same macro trick on collision.cc took P1 from 5 to
4, but rule f caught it: `//:collision: deps lists //:pallet, none of whose headers its files
include`. The way round is to list that pair as an unused_dep departure, which nets P1 -1 and
P5 +1.

Other files and extensions, all with `git add -N` so that git tracks them:

- `src/rrt_fwd.h` (forwarding header): untracked, `includes src/rrt_fwd.h, which is not a
  tracked file`; tracked, `src/rrt_fwd.h: in no group`. Caught.
- `src/sub/x.h`: untracked and tracked, both caught the same way.
- `src/rrt.inc`, included from rrt.h and not in hdrs: `... which no cc_library offers in hdrs
  (rule e)`. Caught by accident.
- `src/rrt.inc` added to `hdrs = ["src/rrt.h", "src/rrt.inc"]`: **ok, exit 0**. The .inc is
  never scanned.
- `src/rrt.inc` as `textual_hdrs`: rule e. Caught as a false positive of a legal construct.
- `src/rrt_impl.hpp`: `in no group`. `src/rrt.hpp` (in planning by `src/rrt.*`) and not in
  hdrs: rule e.
- `src/rrt.cpp` containing `#include "src/pick_place.h"`, added to `srcs`: **ok, exit 0**.

BUILD attributes, all on `//:rrt`:

- `copts = ["-include", "src/pick_place.h"]` with no deps entry: **ok**.
- The same with `":pick_place"` in deps: rule f (unused).
- `includes = ["src"]` with `#include "pick_place.h"`: `not a tracked file`, caught.
- `defines = ["ATTACK_H=\"src/pick_place.h\""]` with `#include ATTACK_H`: **ok**.
- `implementation_deps = [":pick_place", "@mujoco"]`: **ok**. Unused deps are unseen, and a
  used one would give a false missing_dep.
- `data = [":pick_place"]`: ok. Data gives no headers, so this is no way in.
- `alias(name = "pp", actual = ":pick_place")` with `":pp"` in deps: rule f naming
  `//:pick_place`. Caught.
- `"profile"` and `"@//:profile"`: `neither a target of BUILD.bazel nor an outside library`.
  Caught.
- `cc_test(**{...})`: Bazel 9 refuses it (`**kwargs arguments are not allowed in BUILD files`),
  so it is no way in. deps.py silently skips that call (a nameless call is dropped) and
  P5.files rose to 2.
- `my_cc_test(...)` loaded from a .bzl, with `:pick_place` and `@mujoco` added to its deps:
  deps.py ok, summary `not read as targets: ... my_cc_test 1`, P5.files +1. Not caught by the
  hard check.
- A comprehension with `if`: `Unknown: [cc_test(...) for _x in [1] if _x]` traceback, exit 1.

Declarations. In src/rrt.cc I wrote `namespace cell { class Cell; struct StackDisturbance;
StackDisturbance MeasureStackDisturbance(const Cell& cell); }`. deps.py printed ok and P1
stayed 5. This is a real dependency that the check cannot see; only a link of a binary
without `//:score` exposes it. Say so in P1.

deps.json edits, each reverted afterwards:

- Planning `may_include` widened to `["base", "standin"]`, plus the include and the dep:
  deps.py ok, and measure without Bazel was identical to the base (only the absent (bazel)
  lines differed). **Invisible to --check apart from the deps.json diff in review.**
- A new departure `{"kind": "include", "from": "src/rrt.cc", "to": "src/pick_place.h"}`:
  deps.py ok and P1 went from 5 to 6. Visible only as a baseline line.
- `"frozen": true` on `arm` and `layout`: P1.frozen 4 to 5 (so P1 = P1.frozen, the P1 target
  met), P5.frozen 7 to 13, P6.frozen 5 to 6, P7.frozen 18 to 29, P11.frozen 3 to 6.
- Departures may also be of kind `cycle` and `outside_include`, so deps.json can accept a cycle.

What would make this visible to `--check`: compare against deps.json at the merge base, as in
the table. As a minimum, add measured lines for the size of each permission (allowed group
pairs, files allowed truth, frozen files, departures, silenced allowed, single_home entries).
Each such line may not grow (or, for single_home, may not fall), so that a baseline refresh
in the same commit shows the widening as a number.

Outside libraries:

- `#include <mujoco/mujoco.h>` in rrt.cc with `@mujoco` in deps gave
  `src/rrt.cc: includes outside library mujoco, which group planning may not include (deps.json outside, rule c)`.
  Caught.
- `<fmt/core.h>`: caught as unknown.
- `#include <tinyxml2.h>` and `#include <lodepng.h>` in src/layout_plan.cc: **ok, exit 0**.
  `grep -n tinyxml2 external/mujoco+/src/xml/BUILD.bazel` shows `"@tinyxml2"` in `deps`, and
  `external/tinyxml2+/BUILD.bazel` uses `strip_include_prefix = "/"`.

Other rules:

- Removing `#include "src/pallet.h"` from collision.cc gave rule g
  (`... listed in deps.json departures (include) but no longer occurs; remove the entry`) and
  rule f.
- `src/rrt.h` including `src/rrt_worlds.h` gave rule d (`src/rrt.h -> src/rrt_worlds.h -> src/rrt.h`)
  and P1.cycles went from 0 to 1.
- A tracked `tests/new_test.cc` gave `tests/new_test.cc: in no group`.

## 2. Each counted line, before and after

The figures before the attacks, from `tools/architecture/measure.sh` with Bazel:

- P1 5, P1.cycles 0, P1.namespace 0, P1.frozen 4
- P2 6, P2.untested 24, P2.unrun 11, P2.frozen 2
- P3 5 (all frozen)
- P4 26, P4.unpinned 0, P4.mujoco 67, P4.eigen 78, P4.googletest 42, P4.nlohmann_json 4,
  P4.ompl 1, P4.glfw 1, P4.mujoco_files 70, P4.group 0
- P5 16 (unused_project 10, unused_outside 4, missing 1, files 1, frozen 7)
- P6 9 (frozen 5)
- P7 36 (geom_distance 9, standin_module 4, gate 1, container_base 1, pi 21, frozen 18)
- P8 2, P8.unmatched 0
- P9 0
- P10 5
- P11 7 (cc 5, h 2, comment 0, frozen 3)

**P1.** (i) A macro include took the existing collision.cc -> pallet.h departure from P1 5 to
4. Rule f caught it as an unused dep, so the agent must also list that pair, which gives P5
+1. (ii) A new forbidden include of a transitively available header, through a macro in
motion_planner.cc: nothing moved, deps.py ok. Forward and extern declarations: nothing moved.
Verdict: limited.

**P1.cycles.** Sound. It sees only header-to-header edges.

**P1.namespace.** (ii) A namespace opened through an object-like macro, or opened in a .cc:
the figure stays 0. The control, `namespace layout { ... }` written directly in rrt.h, gives 1.
(i) Not applicable at 0. Verdict: limited.

**P2 (bazel).** (i) The `--help` lines in programs.txt: P2 6 to 0, P2.unrun 11 to 0, P2.frozen
2 to 0. The empty test: P2 6 to 4, P2.untested 24 to 22. (ii) All 138 assertions of
rrt_test.cc deleted: P2 6, P2.untested 24, unchanged. Verdict: bad.

**P3.** (i) Truth behind a cell.h helper: P3 5 to 4, P3.frozen 5 to 4. (ii) In
src/pick_place.cc, a function reading `geom_xpos`, `site_xpos`, `xipos`, `sensordata` and a
pointer alias of xpos: no change. The one form caught was `d->xquat + 4 * body`, which gave P3
6. Note that tests/camera_cycle_test.cc:993's substring list would catch geom_xpos and
site_xpos in pick_place.cc, but not xipos or sensordata. Verdict: bad.

**P4.** (i) arm_test through mj_handles.h: P4.mujoco 67 to 66, P4.mujoco_files 70 to 69. (ii)
rrt.h including mj_handles.h: P4.group stays 0, deps.py ok. For MODULE.bazel I did not edit
clone3; I ran measure's `module_bazel` on a copy, out/attack/mod/MODULE.bazel, with
`use_extension`/`use_repo(extra, "fmt")`, `git_override(... branch = "master")` and an
`http_archive` named spdlog with `integrity = "x"`. P4 went from 26 to 27 (spdlog only),
P4.unpinned stayed `[]`, and fmt and the override were unseen. `bazel_dep(name = FMT, ...)`
with a variable crashes measure (TypeError). Verdict: limited.

**P5.** (i) Pointless includes: P5 16 to 13, unused_project 10 to 7, P5.frozen 7 to 4. (ii) A
symbol used transitively after its include and dep were removed: no change. P5.files would also
fall by 1 if the eight viewer targets became one comprehension (`family` = the statement's
line). That is not tried, read from m_target_files. Verdict: bad.

**P6.** (i) The typedef in portable_random.h: P6 9 to 8. (ii) shuffle, a steady_clock
deadline, high_resolution_clock and clock() in layout_plan.cc: no change. Verdict: bad.

**P7.** (i) Function pointer and respelled pi: P7 36 to 30 (geom_distance 9 to 6, pi 21 to
18, frozen 18 to 17). (ii) In collision.cc I added `kAttackModuleLength{2.3}`,
`{2.30, 1.10, 0.035}`, `kAttackGateOffset = 0.003`, `kAttackTubeStep = 1.12` and
`kAttackTwoPi = 6.283185307179586`: no change. A value-level detector (a constexpr literal in
more than one file under src/) finds 29 values today and is mostly noise (0.05, 180.0, 0.02),
so the honest cure is the narrower patterns plus a statement that review holds facts not on
the list. Verdict: bad.

**P8.** (i) A test that compares nothing, plus a no_test reason: P8 2 to 0. (ii) The
byte-for-byte test made to always pass (`if (true || committed == generated)`): no change.
The bay.xml entry deleted: no change. Verdict: bad.

**P9.** (i) Not applicable at 0. The control, `#pragma GCC diagnostic ignored` plus
`// NOLINT(bugprone-*)` in rrt.cc, gives P9 2. (ii) `_Pragma`, mid-comment NOLINT,
`#pragma GCC system_header` in rrt.h after the guard, and copts `-isystem` with
`features = ["-treat_warnings_as_errors"]`: P9 0, unchanged. `"-Wno" + "-error"` is counted,
because the reader evaluates `+`. Verdict: bad.

**P10.** (i) The backticks taken off layout.md:11: P10 5 to 4. (ii) A fenced
`bazel run //:print_layout_moves`, a span wrapped over a line break, a Markdown link to
../src/print_layout_moves.cc, a bare `print_layout_moves.cc`, and `src/layout_moves.h` (gone):
P10 unchanged, and `--list P10` still shows only docs/layout.md:11 from that file. Verdict: bad.

**P11.** (i) The comment strip of pick_place.h: P11.h 2 to 1, P11.frozen 3 to 2. The
three-way split: P11.cc 5 to 4, while P3 went 5 to 6 (the piece leaves the exact-name truth
allowance) and P4.mujoco_files 70 to 72 (a legitimate split is punished there). If a part is
over the limit P11.cc grows; the first try gave a 1334-line part and P11.cc 6. (ii) 3,000 lines
added and the opening comment of rrt.h replaced by five `//`: no change. A strip of comments
alone gets rectangle_fit.cc to 1,003 code lines and bay_hardware.h to 612. Verdict: bad.

## 3. The patterns (`out/attack/patterns.out`, made by `python3 out/attack/patterns.py` in clone3)

P3 (truth):

- Counted: `d.xpos[3*body]` after `auto& d = *cell.data()`, and `Eigen::Map<...>(d->xpos + 3*body)`.
- Missed: `p = d->xpos; p[3*body]`, `d->geom_xpos[...]`, `d->xipos[...] + d->site_xpos[0]`,
  `d->sensordata[adr]`, `&Cell::TrueModulePose` called through a pointer, and
  `std::span(d->xpos, ...)`.
- Today's 5 places, opened (`python3 out/attack/where.py truth ...`):
  - Innocent (scorer or report): src/trial_runner.cc:128,179 (pallet offsets and the finder's
    error for the result row), src/print_perception.cc:258,263 (the estimate compared with the
    truth), src/view_cell.cc:387,434 ("off the truth" text) and src/print_cell.cc:101,162-171
    (a table of the scene).
  - Real: src/print_plan.cc:287 (the grip "as designed" computed from TrueModulePose and fed
    to the planner).

P6 (platform and run):

- Counted: `using Dist = std::uniform_real_distribution<double>;` outside the allowed file.
- Missed: `std::shuffle`, `steady_clock` deadlines, `high_resolution_clock`, `drand48`,
  `random()`, `rand_r`, `clock()`, `generate_canonical`, and a `uniform_##k##_distribution`
  macro.
- Today's places opened: src/conveyor.cc:123 (`normal_distribution` stop error),
  src/plane_fit.cc:133 (`uniform_int_distribution` RANSAC pick), src/point_generator.cc:21-63,
  tests/ik_test.cc:189,195,952 and tests/collision_test.cc:206-941. All are real by the Rule.

P7 (one home):

- Missed: `&mj_geomDistance` through a pointer, `(mj_geomDistance)(...)`, pi as `0.3141...e1`,
  `M_PI` and `6.2831...`, `kModuleLength{2.3}`, `{2.30, 1.10, 0.035}`,
  `kLength = 2.3, kWidth = 1.1`, `kGateOffset{0.003}`, `position = 0.003` and `3e-3` under
  another name.
- Today's places:
  - Real: src/conveyor.h:69-71, src/suction.h:45, src/flat_rack.h.
  - Innocent by P7's own Exception: tests/arm_follower_test.cc:53-55 and
    tests/path_follow_test.cc:52-54.
  - Cannot use the home without breaking P1: src/bay_hardware.h:502 (gate; hardware may not
    include standin's trial_runner.h, and deps.json's note says this copy is deliberate and
    tested), and src/check_placement.cc:490, tests/cell_test.cc:149,606 and
    tests/pick_place_test.cc:150,350 (geom_distance in the stand-in; standin may not include
    layout).

P9 (silencing):

- Counted: `// NOLINTNEXTLINE(...)` and `#pragma GCC diagnostic push`.
- Missed: `_Pragma(...)`, `// reason: kept. NOLINT` (which clang-tidy honours), `#pragma GCC
  system_header`, `[[maybe_unused]]`, `(void)x;` and `static_cast<void>(f())`.
- Today `--list P9` is empty. `grep -rnE "\(void\)\s*[a-zA-Z_]" src tests` finds 3 places.

## 4. The prose of principles.md (repo copy as of this run)

Statements shown false by a command:

1. **Line 215-216** (P7): "The stand-in module's size and the gate's 3 mm are spelled out as
   literals in files that include the header that names them." False for every counted place.
   `grep -n '#include "src' src/conveyor.h src/suction.h tests/arm_follower_test.cc tests/path_follow_test.cc`
   shows that none includes src/pallet.h. src/bay_hardware.h:79-85 includes only standard
   headers and Eigen, not src/trial_runner.h.
2. **Line 258-259** (P8 Exceptions): "docs/layout.md lists those commands" (for docs/img/ and
   docs/*.gif). docs/layout.md:389-391 lists only the layout_*.png commands. deps.json itself
   says the commands of bay_fastening.png and container.png "are written nowhere". The gifs'
   commands are in tools/video/make_all.sh and docs/flat-rack-cell.md.
3. **Line 310-312** (P10 Check) against line 302: the Check is said to count stale paths and
   labels, but it does not count docs/layout.md:667, the Why's own example (`--list P10` shows
   docs/layout.md:11 only). What it counts at :11 is a true sentence.
4. **Line 184 against 190-192 and 198** (P6): rrt.cc:451's clock-stopped search is named as the
   problem, but the Check does not count it (deps.json: "steady_clock, used for time budgets,
   is not counted"). The Exception allows steady_clock only "for a time that is only printed",
   which line 451 is not.
5. **Line 346 against 349-350** (P11): "Target: P11 equals P11.frozen", while
   src/bay_hardware.h (976 lines, group hardware, not frozen) is excepted as "left whole". It
   stays in P11.h, so the target is unreachable without breaking the exception.
6. **Line 228** (P7 Target "P7 equals P7.frozen, then 0"): unreachable while P1 holds. See the
   P7 places above (collision.cc in arm, bay_hardware.h in hardware).
7. **Lines 16, 66 and 128**: links to `../adr/README.md`, `behavior.md` and `dependencies.md`.
   `ls` shows that none exists (docs/adr/ in repo holds 0018 and 0019 only). The same goes for
   `backlog.md` (lines 17 and 350) and ADRs 0001-0017 and 0020-0021. `--check` reports these,
   so they may simply be not written yet.
8. **Line 230-231** (P7 Exceptions: tests may state figures) against the Check: 18 of the 36
   P7 places are in tests/ (`--list P7`).
9. **Line 9-10**: "--check fails when one of them changes without the baseline changing in
   the same commit". --check compares the working tree with baseline.txt and knows nothing of
   commits.

Correct as written (checked): 99 files under src/; `grep -c visibility BUILD.bazel` gives 0;
the P2 query prints "Empty results"; 24 files are untested; mj_geomDistance is called from 4
files under src/ counting the home; OMPL, GLFW and nlohmann_json are included by 1, 1 and 3
files; planning includes no MuJoCo, even transitively; rrt.cc:442 and :451 are as described;
bay_model_test.cc:97 is the byte-compare TEST; src/ and tests/ hold 65,651 lines of .h/.cc;
.bazelrc at base had exactly `-Wno-psabi` (line 19) and `-Wno-error=maybe-uninitialized` (line
27); header opening comments run 6 (mj_handles.h) to 275 (layout_check.h) lines; layout.md:667
is still stale; the three file sizes in P11 are right.

Wording defects that are not false:

- P1 Rule (21-23): its first half tells the agent to edit deps.json. Moving a file to a more
  permissive group, or widening may_include, satisfies the Rule. It should say that groups,
  may_include and frozen change only with a decision record, as P3's Rule does for truth.
- P1 Check (37): "#include lines". The unit is (file, header) pairs.
- P1 (31-32): the groups are listed in another order than deps.json's "lowest first" (layout
  comes after standin).
- P3 (94): camera_cycle_test.cc:974 is the comment; the TEST is at 979. It fails on 15
  substrings, not on "a True name". It is stronger than P3's own patterns, which the text
  should admit.
- P3 Exceptions (108-110): leave out suction.* and check_rig.*, which deps.json allows. The
  allowance is by glob, so new files `src/cell.x.cc` and `src/score.x.cc` inherit it.
- P3 Target (106): already met today (P3 5 = P3.frozen 5), and 4 of the 5 places are scoring,
  so the line exerts no pressure on control code.
- P4 Check (133-135): does not mention P4.mujoco_files. "fetches without a version, commit or
  hash" also covers bazel_deps.
- P9 Rule (265-267): "an attribute" in practice means `no_sanitize` only. `[[maybe_unused]]`
  and `(void)` are neither forbidden by name nor counted.
- Sentences that name nothing of this repository:
  - line 92-93: "a cell that corrects itself ... only a simulator knows"
  - line 246-247: "A committed file that has drifted from its source is a wrong figure
    waiting to be read."
  - line 270-271: "A count only means something if it cannot be lowered by hiding"
  - line 157-158: "The deps lists are also the only statement of structure Bazel has"
  - the P10 Rule (298-299) as a whole is generic.
  They are harmless but could go.
- P6 Rule (176-177): "let no result depend on a clock" contradicts the OMPL Exception, which
  lets OMPL seed itself from the clock.
- P2 Target (76): "0 but for the programs that need a display". src/viewer_lib.* are not
  programs, and run_gate.cc does not need a display. Name the six files.
