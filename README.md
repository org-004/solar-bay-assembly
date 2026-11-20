# solar-panel-pick-place

A simulated factory cell in which a 6-axis KUKA arm with a suction tool moves
solar modules from a pallet onto the rails of a torque tube. Perception,
motion planning and control are hand-written in C++ against MuJoCo.

## Task

1. **Pick.** Modules stand on edge on the pallet, side by side, with their
   weight on their frames. The pallet is tilted slightly so they lean back.
   The arm finds the front module from a depth image and grips its face. The
   pallet's pose is uncertain and the front face moves back after every pick.
2. **Transfer.** The arm carries the module to the other side of itself,
   turning it from near-vertical to flat, along a path that keeps the arm and
   the module clear of the stack, the pallet and the conveyor.
3. **Place.** The arm sets the module flat on rails already mounted on a
   torque tube and releases it.
4. **Index.** The torque tube lies on a conveyor, which moves it along so the
   next module goes down in the same spot.

The result is a placement error and a cycle time over randomized trials.

## Building

The build tool is Bazel, run through Bazelisk, which fetches the version named
in `.bazelversion`. Dependencies are fetched on the first build, which compiles
MuJoCo from source and takes several minutes.

```
bazel test //...             # build everything and run the tests
bazel run //:run_cell -- --pretty --nominal --modules 2
                             # pick and place two modules, print the scores
bazel run //:view_cell -- --nominal
                             # the same in a window; needs a display
bazel run //:viewer_cell     # the cell at rest in a window
```

`run_cell` takes `--seed N`, `--trials T`, `--modules M`, `--nominal` (pallet
exactly where the script assumes), `--xy MM` and `--yaw DEG` (how far off the
pallet may be set down), `--phases` (time and result of every step) and
`--pretty` (aligned table; CSV without it).

In a viewer window: left drag rotates, right drag moves, scroll zooms, Space
pauses, S steps while paused, `[` and `]` change camera, Backspace starts
again, Escape quits.

Smaller programs, each for one part:

```
bazel run //:print_profile     # a trapezoid and an S-curve as CSV
bazel run //:print_plane_fit   # plane-fit error over made-up pallet views
bazel run //:print_points      # one made-up pallet view as CSV
bazel run //:print_cell        # what is in the cell, contacts, speed
bazel run //:print_pallet      # how still the untouched stack stands
bazel run //:viewer_arm        # also _pallet, _conveyor, _suction
```

Add `-- --pretty` to the first two for an aligned table. For speed
measurements build optimized with `--config=opt`; plain `-c opt` fails inside
MuJoCo's own code.

## Layout

| Path | What it holds |
|---|---|
| `MODULE.bazel` | External dependencies |
| `BUILD.bazel` | Build targets |
| `bazel/` | Build rules written for this repository |
| `models/` | The cell as MuJoCo model files: arm, tool, pallet, conveyor, camera |
| `src/` | Programs and libraries |
| `tests/` | Tests |

The world frame has its origin on the floor under the arm's base, +Z up, +X
toward the pallet and the conveyor on the -X side. Lengths are metres, angles
radians.

## Status

The cell exists and one cycle runs in it without a camera.

**The cell** (`models/cell.xml`). A KUKA KR 210 R2700-2, converted from the
vendor's robot description, with a suction tool. A pallet tilted 10 degrees
holding 30 modules standing on edge, the front face 1.95 m from the arm. A
conveyor carrying a torque tube with rails for 10 modules, which moves one
step after each placement and is swapped for an empty one when full. A depth
camera 4 m up, looking down at the stack. Suction is a weld switched on at
the moment of gripping, which holds the module at whatever pose it had.
Left alone for 10 simulated seconds, the stack moves less than 0.01 mm.

**The cycle** (`src/pick_place.h`). The arm goes to the front module, touches
the glass, grips, lifts along the glass, pulls back, turns the module flat,
swings it to the other side, turns it across the tube, sets it on the rails
and lets go; the conveyor then steps. Tool poses become joint angles by
inverse kinematics (`src/ik.h`), and the motors follow S-curve joint moves
(`src/arm_follower.h`) at 90% of KUKA's speed limits. A cycle takes about 25
simulated seconds.

The script is blind: it assumes the pallet is exactly where the drawing says.
With the pallet there, a module lands within 0.1 mm of its slot, for all 30
modules of a pallet. With the pallet set down up to 30 mm and 3 degrees off,
none of 10 trials placed a module: the cups meet the glass early, or the
glass is out of the grip's 10 mm reach. With 5 mm and 0.3 degrees, 8 of 10
placed, each about as far off as the pallet was. Finding the module with the
camera is the next stage.

**Built before the cell.** `src/profile.h` and `src/joint_move.h` time moves
within velocity, acceleration and jerk limits. `src/plane_fit.h` finds the
plane most of a cloud of 3D points lies on, by RANSAC; on made-up clouds with
2 mm of noise it recovers a module face's distance within 0.5 mm and its lean
within 0.05 degrees.

**Where the simulation is kinder than a real cell.** The motors' rotor
inertia and gearbox friction are estimates; KUKA publishes neither. A welded
grip never slips. The module is rigid. The follower's model of the arm is
the simulator's own.

Not written yet: depth images and finding the module in them, planning a
path around obstacles, and the gate that runs randomized trials in CI.
