# Working in this repository

Read by Claude Code at the start of a session here, and by each subagent first. The plan
of record for the stages is the M2 note (`~/Documents/root/notes/robotics/
m2-module-pick-and-place.md`); the working state between sessions is
`m2-session-handoff.md` beside it.

## For every agent

**Build.** `bazel test --config=opt //... --test_output=errors`. Always `--config=opt`
(plain `-c opt` fails inside MuJoCo; unoptimized tests take 5 to 70 times longer). One
Bazel command runs at a time per checkout: wait for the lock, never start a second.
`bazel run` releases the lock before the program starts. Do not read under
`bazel-testlogs` or `bazel-out`; they resolve outside the checkout and can stall on a
permission prompt.

**`BUILD.bazel`** is one file with a `# BEGIN x` / `# END x` section per piece. Edit only
your own section.

**Frames and units.** Metres, radians, simulated seconds. World origin on the floor under
the arm's base, +Z up, +X toward the pallet, conveyor on -X. Front module's glass at
(1.95, 0, 0.75), lean 10 degrees. Tube along Y at x = -2.0, rails' top at 0.8 m, step
1.12 m, 10 slots. Pallet camera at (0, 0, 4.0), its post at (0, -3.8). Check camera on the
floor at (0.1307, 1.4943), looking up. The arm swings round +Y; joint 1 is at 180 degrees
when placing, 5 degrees from its limit.

**The rule.** Control code reads sensor data only: depth images with the calibrated
camera, joint angles, the conveyor's encoder, and what the script did itself. `True...`
accessors and body poses out of the simulation are for the scene generator, the scorer
(`src/score.h`) and tests.

**Map.** Read the top-of-file comment of a header before its body; they are long.

| Piece | Files |
|---|---|
| The cell: load, reset, randomize, cameras, grip | `src/cell.h`, `models/cell.xml` |
| The cycle, step by step | `src/pick_place.h` |
| Checks on a run: contacts, torques | `src/cycle_check.h`, `src/score.h` |
| Pallet camera to module pose | `src/depth_camera.h`, `src/module_finder.h`, `src/rectangle_fit.h`, `src/plane_fit.h` |
| Check camera to grip offset | `src/held_module_finder.h`, `src/check_placement.h`, `src/check_rig.h` |
| Planning | `src/motion_planner.h` (`PlanArmMove`, `PlanScene`), `src/collision.h`, `src/rrt.h`, `src/path_timing.h` |
| Following | `src/arm_follower.h`, `src/joint_move.h`, `src/profile.h`, `src/ik.h` |
| Tables | `run_cell`, `print_perception`, `print_plan`, `print_check`, each with `-- --pretty` |
| Viewer | `DISPLAY=:1 bazel run --config=opt //:view_cell -- --seed 3` |

## For the lead session

The lead plans, briefs, verifies and commits. It writes code itself only when the change
is smaller than its brief would be.

**Before fanning out,** settle top-level feasibility yourself: reach, layout, whether a
pose clears the planning margin. Rework on 2026-11-19 came from skipping this.

**Break a stage into pieces by file ownership.** Two agents never own the same file.
`src/pick_place.*` is the bottleneck: anything that changes the cycle is one piece, and
pieces that feed it (a model change, a `Cell` accessor, a library, a table) run beside it.
Launch independent pieces in one message.

| Piece | Agent |
|---|---|
| Design inside a fixed goal: wiring into the cycle, a new library, debugging a phase | `m2-implementer` (Opus) |
| Exact spec: additive API, model snippet, flag, a stated test, refactor without behaviour change, CI | `m2-mechanic` (Sonnet) |
| Run commands and bring back the summary lines | `m2-runner` (Haiku) |
| Read a stage's diff before the commit | `m2-reviewer` (Opus), for stages that change the cycle or a tolerance |

A fresh agent per piece; do not resume one past about 40 turns. If a cheaper agent's
result is wrong or thin, move the piece up one tier with a new brief.

**A brief states:** the goal and done-when (commands that must run, the table wanted);
the files owned and the files forbidden; what other agents are changing at the same
time; what is already decided or ruled out; a scratch directory; the report's form.

**Verify before committing a stage,** in the lead session: read the diff; rerun the full
suite; reproduce the stage's table with `run_cell` or the `print_*` program and compare
with the agent's figures; run one cycle in the viewer. Then README status, the M2 note's
stage paragraph, the daily note.

**Commit** at the end of each stage, to `main`, by path, message `Area: what changed`
with bullets, no attribution line. Do not push unless asked. Note edits stay uncommitted
in the notes repo unless asked.

**Ask the user** for anything that sets what "correct" means: a tolerance, dropping a
planned item, a layout change. Carry on with the rest meanwhile.
