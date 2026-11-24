# Stop 2: Where things are

Frames and poses: how the code says where the tool, a module and a camera are, and how
it turns one description into another. Nearly every bug you will chase in a robot cell
is, underneath, a pose that is wrong, stale, inverted or in the wrong frame. This is the
most important stop of the first week. About a day.

## Read

1. `README.md`, the paragraph on the world frame; `AGENTS.md`, "Frames and units".
2. `src/pallet.h`: the comments on `kModuleLength` and its neighbours and on
   `WorldPose`. A module's own frame has +x along its long side, +y along its short
   side and +z out of the glass.
3. `src/arm.h` and `src/arm.cc`: `TipPose`. Ten lines that turn the simulator's raw
   numbers into a pose.
4. `src/held_module_finder.h`, from "the frame algebra" to the end, then the three
   functions in `src/held_module_finder.cc` it declares. Each is one line.
5. `src/pick_place.cc`: the first thirty lines of `PickPlace::PlanPlace`, down to where
   `release_tip_` is set.
6. `src/camera_model.h`: the paragraph headed "Frame".

## The idea

**A frame** is a place to stand and three directions to measure along: an origin and
three axes at right angles, called x, y and z. The world frame of this cell has its
origin on the floor under the arm's base, +X toward the pallet and +Z up. The tool has a
frame of its own, the *tool tip*, where the cups meet the glass, with +Z pointing out of
the tool. Each module has one, at its centre. Each camera has one.

The same point has different coordinates in different frames. The centre of the front
module's glass is at about (1.95, 0, 0.75) in the world, and at about (0, 0, 0.0175) in
that module's own frame: half a thickness along its +z.

**A pose** says where one frame is relative to another. It has two parts:

- a rotation `R`, a 3 by 3 table of numbers whose three columns are the child frame's
  x, y and z axes written in the parent's coordinates;
- a translation `t`, the child's origin in the parent's coordinates.

To convert a point from the child's coordinates to the parent's: `p_parent = R * p_child
+ t`. The code's type for a pose is `Eigen::Isometry3d`, which holds `R` and `t` and
does that sum when you write `pose * point`. `pose.linear()` is `R`, `pose.translation()`
is `t`, and `pose.linear().col(2)` is "the child's z axis, in the parent's coordinates".

**Naming.** Call a pose "B in A": frame B as seen from frame A. It takes B's coordinates
to A's. The code's names follow this: `module_in_tip`, `tool_tip_pose_world`.

**Chaining.** If you know the tip in the world and the module in the tip, then

```
module in world = tip in world * module in tip
```

Read it like the words: the inner names (tip, tip) meet and cancel. `*` between two
poses means "do the right one first, then the left". The order matters: `a * b` and
`b * a` are different poses, and both compile.

**Inverting.** The inverse of "B in A" is "A in B": `pose.inverse()`. With chaining and
inverting, any one pose in a chain can be solved for from the others, as in algebra,
except that you must multiply on the correct side.

**The three functions.** `src/held_module_finder.h` is that one line solved three ways.

- `DesignModuleInTip`: where the module should sit on the tool if the pick went exactly
  as drawn. Half a turn about x (the tool's +Z points into the glass, the module's +z
  out of it) and a short step along the tool's +Z.
- `GripOffsetFromCheck`: the check camera measures the module in the world, and the
  joint angles give the tip in the world. So `module in tip = (tip in world)^-1 * module
  in world`.
- `CorrectedTipTarget`: the module should land on a known pose in the world, and it sits
  on the tool as measured. So `tip in world = module in world * (module in tip)^-1`.

That last line is the whole of "the check after pickup corrects the placement".

**Rotations in other clothes.** A rotation matrix is easy to chain and wasteful to
store. You will meet three other forms of the same thing:

- a *quaternion*, four numbers, which is how MuJoCo stores orientations and what
  `WorldPose` in `src/pallet.h` holds;
- a *rotation vector*, three numbers: an arrow along the axis to turn about, whose
  length is the angle. The solver in stop 5 measures orientation error this way;
- single angles with names, such as the scorer's *yaw* (turned about the vertical) and
  *tilt* (out of flat).

There are no Euler angles (three turns about three axes in sequence) in `src/`. They
are what a robot's teach pendant shows, and they are easy to get wrong.

**Angle times distance.** A small turn of `a` radians moves a point at distance `L`
from the axis by about `L * a`. The pallet camera is 3.8 to 4.5 m from the front glass,
depending on how far into the stack the pick is, so one milliradian of error in where
the camera is thought to point moves the answer by about 4 mm (the README measures
4.4). This one multiplication explains most of the numbers in the README.

**Units.** Metres and radians everywhere inside the code. Millimetres and degrees
appear only where something is printed.

## C++ you meet here

**Small types that are values.** `Eigen::Vector3d` is three `double`s stored in place.
Assigning one copies three numbers; there is no hidden reference. `Vector3d`,
`Matrix3d` and `Isometry3d` have their size fixed at compile time. `VectorXd` (the X is
for "any size") holds its numbers on the heap and is what the code uses for a set of
joint angles.

**Operators that mean what the maths means.** C++ lets a library define `*`, `+` and
the rest for its own types. `pose * pose` chains, `pose * point` converts a point,
`matrix * vector` multiplies, `2.0 * vector` scales. The types decide which, at
compile time. Named operations are ordinary calls: `a.dot(b)`, `a.cross(b)`,
`v.norm()`, `v.normalized()`.

**Passing without copying.** A parameter written `const Eigen::Isometry3d& pose` is a
reference: the function reads the caller's pose in place. Returning a pose by value,
as `TipPose` does, is fine and cheap.

**Views.** `pose.translation() = ...` in `TipPose` assigns through a function call.
`translation()` returns a reference to the three numbers inside `pose`, so the
assignment writes into the pose. `Eigen::Map` goes further: it makes a block of memory
that belongs to somebody else (here MuJoCo's `site_xmat`, nine `double`s in a row)
usable as an Eigen matrix without copying it. `RowMajor` says the nine numbers are
stored a row at a time, which is MuJoCo's way; Eigen's own default is a column at a
time.

**`auto` and Eigen.** `auto x = ...` asks the compiler to work out the type. With Eigen
that can go wrong: `a * b` does not produce a matrix but a recipe for one, which may
refer to temporaries that are gone by the next line. This code writes Eigen types out
in full. Do the same.

**A dereferenced optional.** In `PlanPlace`, `*GripPose(...)` has a star in front.
`GripPose` returns a `std::optional`, a box that may be empty, and `*` takes the
contents out without looking. That is only right where the inputs cannot fail, as
there. Stop 3 covers `optional` properly.

## Lab

1. **Geometry by hand.** `bazel run --config=opt //:print_cell` and look at "where
   things are". Using only module_0's glass centre and its normal:
   - How far does the glass lean back from upright?
   - Where is the point 0.35 m in front of the glass, where the arm's approach starts?
   - A module is 0.035 m thick. Predict module_29's glass centre, then compare with
     the line below.
   - The camera looks along its own -Z. How far below the horizontal?
2. **What the correction is worth.** One pallet, the pallet camera's calibration off by
   1 mm and 0.05 degrees, three ways. Note "along" and "across" each time.
   - No check:
     `bazel run --config=opt //:run_cell -- --pretty --seed 3 --modules 1 --calibration 1,0.05 --carry planned`
   - With the check: the same with `--check` in place of `--carry planned`. Add
     `--phases` and find the measured grip offset in the carry's line.
   - In a worktree, apply the correction the wrong way round. In
     `PickPlace::PlanPlace`, replace `CorrectedTipTarget(*release_module_,
     *held_in_tip_)` with

     ```
     design_release * (*held_in_tip_ *
                       DesignModuleInTip(kModuleThickness, options_.grip_gap)
                           .inverse())
     ```

     Before you run it, predict "along" and "across" from the first two runs. Then
     work out on paper why.
3. **The compiler does not know about frames.** Undo that edit. In
   `GripOffsetFromCheck` swap the two factors, so it returns `measured_module_pose_world
   * tool_tip_pose_world.inverse()`. It builds. Predict what the cycle does, then run
   the `--check --phases` command again. What stopped it, and what does the message
   blame?
4. **How well the offset is measured.**
   `bazel run --config=opt //:print_check -- --pretty --trials 5 --noise`. The module
   is gripped up to 10 mm and 1.5 degrees off on purpose. How close is the recovered
   offset?

Remove the worktree when you are done.

## Pitfalls

- **The order of a product.** `a * b` against `b * a`. Both compile.
- **A missing or extra inverse.** "B in A" used where "A in B" was meant. Lab run 2
  shows the size of it: twice the error you were trying to remove.
- **Turning about the wrong point.** A correction of half a degree about the tool tip
  is not the same motion as half a degree about the module's centre. See the comment
  on `CorrectedTipTarget`.
- **Quaternion order.** MuJoCo stores w, x, y, z. `Eigen::Quaterniond`'s constructor
  takes w, x, y, z, but its `coeffs()` are x, y, z, w. ROS messages are x, y, z, w.
- **Camera frames.** A MuJoCo camera looks along its -Z with +Y up. OpenCV and ROS
  "optical" frames look along +Z with +Y down.
- **Degrees.** Everything is radians, except that MuJoCo reads a camera's `fovy` in
  degrees whatever the file's setting (see the comment in `models/cell.xml`).
- **Stale poses.** A pose worked out before the arm moved describes the past. MuJoCo
  updates body poses only when asked (`mj_forward`, `mj_kinematics`).

## In the MoveIt cells

ROS 2 keeps poses in a shared tree called tf2. Every frame has a name (`world`,
`base_link`, `tool0`, one per camera), nodes broadcast the transform from each frame
to its parent, and any node can ask for any frame in any other:
`lookupTransform("world", "tool0", time)` is "tool0 in world". The robot's own links
come from the URDF and the joint angles, published by `robot_state_publisher`; a
camera's calibration is a transform published once.

What changes for you:

- The chaining and inverting are done for you, which removes one class of mistake and
  hides another: a wrong transform anywhere in the tree is wrong for everybody, and it
  is not in the code you are reading.
- Transforms carry a time. Asking for "now" on a moving robot can give a pose from a
  few milliseconds ago or fail outright. This repository has no such problem because
  nothing moves between the simulation's steps.
- Conventions are written down. REP 103 fixes x forward, y left, z up, metres and
  radians; this cell's world frame agrees with it.
- On the robot's pendant you will see Euler angles: KUKA shows a pose as X, Y, Z, A, B,
  C, with A about Z, B about Y and C about X.

The algebra is the same. When a production cell places a part 4 mm off, the first
question is still which pose in the chain is wrong.

## Reading

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 3, "Rigid-Body Motions". Free
  at <http://modernrobotics.org>, with short videos for each section. Sections 3.1 to
  3.3 are what this stop uses; leave twists and wrenches for later.
- 3Blue1Brown, *Essence of linear algebra*,
  <https://www.3blue1brown.com/topics/linear-algebra>: chapters 3 and 4 (matrices as
  transformations, and multiplying them), if matrices are a distant memory.
- Eigen, "Space transformations",
  <https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html>: the reference for
  `Isometry3d`, `AngleAxisd` and `Quaterniond`.
- ROS REP 103, <https://www.ros.org/reps/rep-0103.html>: one page of conventions.
- tf2 in the ROS 2 documentation,
  <https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Tf2.html>.

## Office hours

Be ready to explain, with the chapter closed:

- what "module in tip" means, and what its inverse is called;
- why `tip in world = module in world * (module in tip)^-1`, in words;
- why applying the correction the wrong way round doubles the error rather than
  leaving it unchanged;
- why 0.05 degrees of camera error is worth worrying about.

Questions worth asking your teacher:

- Why does the grip's offset show up as "across" for one of its parts and "along" for
  another?
- The message in lab run 3 blames the camera. How would you tell a software fault from
  a hardware fault on a real cell?
