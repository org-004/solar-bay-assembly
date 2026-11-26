# 0024 A layout's plan is run in physics by a group of its own, which uses the arm's follower and the stand-in cell's suction

Status: accepted
Date: 2026-11-26
Decided by: spec 019 (the first step in physics: one module carried and set)

**Context.** Every figure of the flat rack cell was kinematic: the plan
(`src/layout_plan.h`) says where every arm is at every instant and checks it for
clearance, but nothing drove it. Spec 019 asks that the module arm, on its position
servos and with its real payload, follow the committed plan for one module's cycle in
MuJoCo, that the run be measured as the checker measures, and that a committed record of
it count towards `physics_share` (`docs/scorecard.md`). The spec names the pieces to use
as they are: the follower of the one-arm stack (`src/arm_follower.h`, group `arm`) and
the stand-in cell's suction weld (`src/suction.h`, group `standin`, frozen). The layout
code (group `layout`) may include neither, and the stand-in cell is to be replaced, not
converted.

**Decision.**
- **A new group `physics`** (`tools/architecture/deps.json`) holds `src/physics_run.*`,
  `src/run_physics.cc` and `tests/physics_run_test.cc`. It may include `base`,
  `planning`, `hardware`, `layout`, `arm` and `standin`, and the outside libraries
  `eigen` and `mujoco`. This adds six pairs to P1.allowed (24 to 30) and two to
  P4.allowed (22 to 24).
- **Nothing in `layout` includes `physics`.** The record a run writes
  (`src/physics_record.h`, `physics/<layout>.json`) is in `layout`, because the
  scorecard reads it, and it includes only `layout_plan`.
- **`src/suction.*` is used and not edited**, so the stand-in group stays frozen.
- **`src/physics_run.cc` may read the simulation's truth** (P3.allowed 55 to 56). It is
  the scorer of the run, as `src/score.cc` is the stand-in cell's: it compares the
  carried module's true pose with the plan and with its designed place on the rails.
  Its control reads only joint angles and the track's position, which are sensor data.
- **Two small changes to existing pieces carry the run:**
  - `ArmFollower` follows a `TimedPath` (a motion given as a function of time) and
    takes a carrier's planned acceleration into its model feedforward.
  - `FindArm` takes the names of an arm attached with a prefix.
  - Both keep every existing behaviour, and the existing tests show it.
- **One builder still makes both models.** `BuildLayoutXml` makes the physics model,
  switched on by `LayoutState::physics_arm`. Without it the text is what it always was.
- **One distance function still measures both.** `InstantDistances` measures the run,
  through a hook that moves the carried module's geoms to where the physics has it.

**Consequences.**
- The counts that follow from the new targets grow, and `baseline.txt` records them:
  - P4.mujoco, P4.eigen, P4.googletest, P4.nlohmann_json and P4.mujoco_files each grow
    by one or two;
  - the new program is captured in `programs.txt`.
- When the stand-in cell is removed, the suction weld moves to a group that `physics`
  may include. That removal is the time to give `physics` its own suction.
- A run of more arms, or of the pins, extends this group and does not widen `layout`.

**Alternatives.**
- *Put the run in `layout`, with `arm` and `standin` in its `may_include`.* Every layout
  file would then be free to reach the stand-in cell, and only the run needs it.
- *Copy the suction weld and the follower into `layout`.* This would write the same
  facts twice. The spec asks for the carried-over pieces as they are.
