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
bazel test //...          # build everything and run the tests
bazel run //:step_arm     # simulate one second and print the joint angles
bazel run //:viewer       # draw the arm in a window; needs a display
bazel run //:print_profile  # print a trapezoid and an S-curve as CSV;
                            # add `-- --pretty` for an aligned table
bazel run //:print_plane_fit  # fit planes to made-up pallet views and print
                              # each one's error as CSV; `-- --pretty` as above
bazel run //:print_points   # print one made-up pallet view as CSV, a row
                            # per point, for plotting
```

## Layout

| Path | What it holds |
|---|---|
| `MODULE.bazel` | External dependencies |
| `BUILD.bazel` | Build targets |
| `bazel/` | Build rules written for this repository |
| `src/` | Programs and libraries |
| `tests/` | Tests |

## Status

The toolchain works: a C++ program loads a robot arm into MuJoCo, steps it and
draws it, and CI runs the tests on every push. The arm is a stand-in, a
Universal Robots UR10e from MuJoCo Menagerie.

Timing is written: `src/profile.h` gives the shortest rest-to-rest move along
one axis within velocity and acceleration limits (trapezoid) or velocity,
acceleration and jerk limits (S-curve), and `src/joint_move.h` uses one such
profile to move all joints together within each joint's own limits. Nothing
drives the simulated arm with them yet.

The plane fit is written: `src/plane_fit.h` finds the plane that most of a
cloud of 3D points lies on, by RANSAC with a least-squares refinement, at any
orientation. `src/point_generator.h` makes clouds with known answers to test
it on: a module face with depth noise, the module behind it, a floor and
clutter. With 2 mm of noise and 2,000 points on the face, the face's distance
is recovered within 0.5 mm and its lean within 0.05 degrees. No camera
supplies real clouds yet.

The cell, the KUKA arm, the rest of perception, planning and control are not
written yet.
