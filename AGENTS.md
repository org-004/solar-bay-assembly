# Working in this repository

For every agent that builds or edits here, on any machine. What to deliver, and where,
is the task's to say. When you brief a subagent that will build or edit here, tell it to
read this file first.

## Build and test

Always `--config=opt`: unoptimized tests take 5 to 70 times longer, and plain `-c opt`
fails inside MuJoCo. Add `--config=agent` whenever another agent may be building on the
machine: it keeps Bazel to three cores and leaves the gate out. Without the limit, three
suites at once ran a machine out of memory.

- While iterating, only your piece's tests:
  `bazel test --config=opt --config=agent //:cell_test //:ik_test`
- Once, when the piece is done, everything but the gate:
  `bazel test --config=opt --config=agent //...`
- Once, alone on the machine, before code lands on `main`, the gate as well
  (`docs/gate.md`): `bazel test --config=opt //...`
- A change to documents alone needs no build and no test.

Where the time goes (three cores, 2026-11-23): eight tests take 370 of the suite's 417
seconds. `check_cycle_batch_test` 76 s, `planned_carry_test` 74, `motion_planner_test`
58, `trial_runner_test` 45, `camera_cycle_test` 41, `check_cycle_test` 37,
`pick_place_test` 23, `collision_test` 17. The other 21 take under 10 s each. Keep the
eight out of the loop unless your change is theirs to check. A fresh checkout builds and
passes everything but the gate in about 10 minutes (about 6 before OMPL was added); the
gate adds about 4 on four cores.

## Caches, locks and worktrees

- Before the first build, give every checkout on the machine one disk cache, unless
  `~/.bazelrc` already names one:
  `echo "build --disk_cache=$HOME/.cache/bazel-disk" >> ~/.bazelrc`.
  A new clone or worktree then takes MuJoCo and the rest from it (build and all tests in
  about 2.5 minutes in place of 10), and a test none of whose inputs changed is not run
  again anywhere.
- One Bazel command at a time per checkout: wait for the lock, never start a second.
  `bazel run` releases the lock before the program starts. A git worktree
  (`git worktree add ../sba-wt-<piece>`) has a lock of its own.
- Suites side by side gain little: three at once finish about a quarter sooner than one
  after another. The saving is in each agent running few tests, not in more agents
  testing.
- Before removing a worktree, `bazel clean --expunge` in it; otherwise its output base
  and server stay behind.
- A failing test's output is printed in the terminal. Do not read under `bazel-testlogs`
  or `bazel-out`: they resolve outside the checkout, and a session that asks permission
  for that stalls.

## Dividing the work

- Two agents never own the same file. `BUILD.bazel` is the one shared file: it has a
  `# BEGIN x` / `# END x` section per piece, and each agent edits only its own.
- `src/pick_place.*` is the bottleneck. Everything that changes the cycle is one piece
  for one agent; pieces that feed it (a model change, a `Cell` accessor, a library, a
  table) run beside it.
- Before dividing the work, settle top-level feasibility in one place: reach, layout,
  whether a pose clears the planning margin. Rework on 2026-11-19 came from skipping
  this.

## Rules

- **Sensor data only.** Control code reads depth images with the calibrated camera,
  joint angles, the conveyor's encoder, and what the script did itself. `True...`
  accessors and body poses out of the simulation are for the scene generator, the
  scorer (`src/score.h`) and tests.
- **What "correct" means is the owner's.** Do not loosen a tolerance or a test's bound,
  change the cell's layout or drop a planned item to make something pass. Leave it
  failing and say what you found.
- **The same on every platform.** Development machines are 64-bit ARM and CI is x86-64.
  A bound that rests on particular random scenes takes its draws from
  `src/portable_random.h`, one draw per statement. Do not bound a wall-clock time in a
  test; bound a count.
- **A number comes from a run.** A figure in the README, a report or a commit message
  is reproduced with `run_cell` or a `print_*` program at that commit, not copied from
  another agent's summary. A change that moves a number in the README's Status updates
  it there.
- **Commits.** `Area: what changed`, then bullets in plain, complete sentences. No
  attribution line.

## Frames and units

Metres, radians, simulated seconds. World origin on the floor under the arm's base, +Z
up, +X toward the pallet, conveyor on -X. Front module's glass at (1.95, 0, 0.75), lean
10 degrees. Tube along Y at x = -2.0, rails' top at 0.8 m, step 1.12 m, 10 slots. Pallet
camera at (0, 0, 4.0), its post at (0, -3.8). Check camera on the floor at
(0.1307, 1.4943), looking up. The arm swings round +Y; joint 1 is at 180 degrees when
placing, 5 degrees from its limit.

## Map

Each header opens with a long comment that explains the piece (240 of the 1,173 lines of
`src/pick_place.h`). Read that before the body.

| Piece | Files |
|---|---|
| The cell: load, reset, randomize, cameras, grip | `src/cell.h`, `models/cell.xml` |
| The cycle, step by step | `src/pick_place.h` |
| Checks on a run: contacts, torques | `src/cycle_check.h`, `src/score.h` |
| Whole pallets and the gate | `src/trial_runner.h`, `tests/gate_test.cc`, `docs/gate.md` |
| Pallet camera to module pose | `src/depth_camera.h`, `src/module_finder.h`, `src/rectangle_fit.h`, `src/plane_fit.h` |
| Check camera to grip offset | `src/held_module_finder.h`, `src/check_placement.h`, `src/check_rig.h` |
| Planning | `src/motion_planner.h` (`PlanArmMove`, `PlanScene`), `src/collision.h`, `src/rrt.h`, `src/path_timing.h` |
| OMPL as the planner's second search | `src/ompl_search.h`, switched by `MotionPlanOptions::search` and `--planner ompl` on `run_cell` and `print_plan`; `bazel/ompl.BUILD.bazel`; `docs/ompl-comparison.md` |
| Following | `src/arm_follower.h`, `src/joint_move.h`, `src/profile.h`, `src/ik.h` |
| Tables | `run_cell`, `print_perception`, `print_plan`, `print_check`, each with `-- --pretty` |
| Viewer (needs a display) | `bazel run --config=opt //:view_cell -- --seed 3` |
