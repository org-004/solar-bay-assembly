# Stop 5: Pose to joint angles

The cycle thinks in tool poses: "cups square to the glass, 2 mm in front of it". Motors
take joint angles. Inverse kinematics is the translation between the two, and the places
where it struggles (the edge of reach, a joint at its limit, a wrist lined up with
itself) are the places where robot cells stop with an error. About a day.

## Read

1. `src/ik.h`: the comment at the top, all of it. It is the best single page on the
   subject in this repository.
2. In `src/ik.h`: the comment on `IkOptions::damping`, then `IkStatus` and `IkReport`.
3. `src/ik.cc`: `Evaluate` and `PoseError`, then the loop in `SolveIk`. Read the
   comments; the arithmetic is five lines.
4. `SolveIkPath`, `StraightLine` and `GripPose`, declarations with their comments
   first, then the bodies.
5. `PickPlace::SolveLine` in `src/pick_place.cc`: how the cycle uses them.
6. `tests/ik_test.cc`: the comment above `RoundTripsFromNearbySeeds`, which names the
   arm's three singularities, and `FlippedWrist`.

## The idea

**Forward is easy.** Given six joint angles, where is the tool tip? Chain six poses,
base to link 1 to link 2 and on to the tip (stop 2, six times over). There is always
exactly one answer. MuJoCo does it (`mj_kinematics`), and `TipPoseAt` wraps it.

**Inverse is not.** Given a pose for the tip, which angles put it there? Six unknowns
and six conditions (three for position, three for orientation), tangled together
through sines and cosines. There may be no answer, one, or several. So the solver
searches. It starts from a guess called the *seed*, asks forward kinematics where the
tip is for that guess, measures how far that is from the target, turns the joints a
little to close the gap, and repeats.

**The Jacobian says which way to turn.** At the current guess, ask of each joint in
turn: if this joint alone turned by a small angle, how would the tip move and turn?
With the arm at home the tip is 1.55 m out from joint 1's axis, so one milliradian
of joint 1 swings it 1.55 mm sideways. Six such answers, one per joint, each with six
numbers (three of movement, three of turning), make a 6 by 6 table: the Jacobian. For
small changes it is accurate enough to say

```
change of tip  =  Jacobian * change of joints
```

and that is a set of six linear equations. Put the gap you want to close on the left
(`PoseError`: the position difference, and the orientation difference as a rotation
vector from stop 2), solve for the change of joints, take the step, and work the table
out afresh at the new guess. From a seed close by, three iterations is typical.

**Singularities.** In some poses the arm loses a direction it can move in. Stretched
out straight, it cannot move the tip further out. With joint 5 at zero, joints 4 and 6
turn about the same line, and nothing turns the tool about the axis square to both.
With the wrist straight above joint 1's axis, joint 1 no longer moves it. At such a
pose two columns of the table say the same thing, and near one they almost do. Solve
the equations there and the answer asks for enormous joint turns to gain a hair in
the lost direction. The next guess lands somewhere wild.

**Damping.** The cure is to ask for less. In place of "close the gap exactly", the
step minimises

```
(gap left after the step)^2  +  damping^2 * (size of the joint step)^2
```

so a direction that costs huge joint motion is mostly given up for this step. Far
from a singularity the penalty changes almost nothing. The solver also adapts: a step
that makes things worse is undone and retried with ten times the damping, and no
joint turns more than `max_step` in one go. The comment on `IkOptions::damping` gives
the numbers the default was chosen on.

**Several answers: branches.** One tool pose usually has several sets of joint
angles. The wrist can be flipped (joint 4 half a turn on, joint 5 bent the other way,
joint 6 half a turn on). The elbow can be above or below the line to the wrist. Joint
1 can face the target or face away and reach back over. A search that takes small
steps from its seed ends at the answer nearest the seed. That is what you want: solve
a path waypoint by waypoint, each seeded from the one before, and it stays on one
branch. A jump to another branch would be a large, fast swing of the joints for a
tiny motion of the tool.

**Guarding the path.** `SolveIkPath` refuses a path in which any joint changes by more
than `max_joint_step` between neighbouring waypoints, reporting `kTooFarFromSeed`. The
cycle prints that as "jumped branch". The limit has to suit the spacing of the
waypoints: it is a bound on angle per step, not a detector of branches.

**Limits and reach.** Every guess is clamped into the joint ranges. If the answer near
the seed lies beyond a limit, the search presses against it, stops improving and
reports `kStalled`. A target out of reach ends the same way; the report says how close.

**Straight lines for the tool.** Stop 3 showed a joint move bows the tool's path. Where
it must be straight, the cycle makes one: `StraightLine` lists tool poses a centimetre
apart (position along the line; orientation turning steadily about one fixed axis,
called slerp), and `SolveIkPath` turns the list into joint waypoints. Between two
waypoints a centimetre apart the bow is a few hundredths of a millimetre.

**What it does not do.** The solver knows the arm's geometry and its joint ranges and
nothing else. Its answer may put the arm through the pallet. Collisions are stop 8.

## C++ you meet here

**File-private helpers and constants.** In `ik.cc` everything between `namespace {`
and its closing brace (`Evaluate`, `PoseError`, `Converged`, `Clamp`) can be used only
in that file. An unnamed namespace is C++'s "not exported", like a lowercase name in
Go: the header is the public face and this holds the workings. The tuning constants
live there too. `constexpr double kDampingUp = 10.0;` is fixed at compile time, and
each carries a comment saying what was tried. A bare number is a question for the
next reader.

**Names for fixed-size matrices.** `using Vector6d = Eigen::Matrix<double, 6, 1>;`
gives a type whose size the compiler knows, so it lives on the stack with no
allocation. `error.head<3>()` is a view of its first three entries and `tail<3>()` of
its last three; the number in angle brackets is a compile-time size.

**An answer, and a report on the side.** `SolveIk` returns
`std::optional<Eigen::VectorXd>`: the angles or nothing. Why it failed goes into an
`IkReport`, if the caller passed one: `IkReport* report = nullptr`. Inside, two lines
make that painless:

```
IkReport local;
IkReport& r = report != nullptr ? *report : local;
```

From there the code writes to `r` without caring whose it is. Go would return
`(value, err)`. Here failure is ordinary, and the caller decides how much to hear.

**A status and a `switch`.** `enum class IkStatus` lists every way a solve can end.
`IkStatusName` in `src/pick_place.cc` turns one into words with a `switch` that has a
`case` for each and no `default`. Add a status and forget the `switch`, and the
compiler warns you. That is a reason to prefer it to a chain of `if`s.

**A loop with exits, and solving without inverting.** The solve is `while (true)`
with a `break` for each way out; the status set before the `break` says which. The
step itself is `a.ldlt().solve(error)`: `ldlt()` factorises the 6 by 6 matrix and
`solve` uses the factors. Code that solves equations by computing an inverse is
slower and less accurate, and you will not find it here.

## Lab

Runs 2 and 3 edit a default: use a throwaway worktree (`new-hire/AGENTS.md`, "Labs"),
and restore each edit before the next.

1. **Read the solver's own report.**
   `bazel test --config=opt //:ik_test --test_output=all`. Find and explain:
   - "round trips": how many iterations from a seed 0.1 rad off? Is "ended on other
     angles near a singularity" a failure?
   - "from home": one seed for every target. Why are a tenth not solved?
   - under "cell poses", a pose and its "wrist flipped" line. Which joints differ?
   - "place, X -X": "from home itself: stalled", yet a pose is printed. How?
   - the "unreachable" and "singular" lines.
2. **More damping, then almost none.** In `IkOptions` set `damping` to 0.05. Predict
   which kind of target suffers, then run the test and read the trial that fails: look
   at its joint 5. Then set it to `1e-6`. A different test fails. What is the "largest
   joint move from the seed" now, and what would that be on a real arm?
3. **The path guard.** Put damping back. In `PickPlaceOptions` set `max_joint_step` to
   0.005 and run
   `bazel run --config=opt //:run_cell -- --pretty --nominal --modules 1 --phases`.
   Which step fails, which move does the reason name, and what does it say happened?
   Restore it, set `line_spacing` to 0.10, and run again. Did a branch jump either time?
4. **One column by hand.** `bazel run --config=opt //:print_cell` gives the tool tip's
   position at home. Joint 2's axis is 0.33 m forward of joint 1's and 0.645 m up
   (`models/arm.xml`). If joint 2 alone turned by one milliradian, how far would the
   tip move, and roughly in which direction?

## Pitfalls

- **"No IK solution" does not mean out of reach.** It can be the seed, a branch, a
  joint limit, or waypoints spaced too far apart for the guard. Read the status.
- **The same pose on another branch is another arm.** Elbow down may be through the
  conveyor; a flipped wrist may run joint 6 into its limit ten centimetres later.
- **Passing near a singularity.** The path exists, but some joint must swing fast to
  follow it, and the timing (stop 10) has to slow everything down for it.
- **Winding.** Joints 4 and 6 turn 350 degrees each way. Two answers a whole turn
  apart put the tool in the same place and the cables in different ones.
- **"Converged" means within tolerance.** Here 0.1 mm and 0.01 degrees. Near a
  singularity the pose can be that close while the angles differ by degrees.
- **Solving in the live state.** Each guess overwrites body poses; hence the scratch
  `mjData` every function here takes.

## In the MoveIt cells

- **IK is a plugin.** Each group of joints has a kinematics solver named in a
  configuration file. The default is numeric, like this one. TRAC-IK is a sturdier
  numeric solver that handles joint limits better. IKFast generates closed-form code
  for one robot: for an arm whose last three axes meet at a point, as this KUKA's do,
  it returns every solution, up to eight, in microseconds, and you choose the branch.
- **Seeds are still there.** A numeric plugin starts from the robot's current state,
  with a timeout in place of an iteration count. "IK failed" in a log needs the same
  questions as here.
- **Straight tool paths are computed the same way.** MoveIt solves IK at small steps
  along the line and reports what fraction of the line it achieved. A jump threshold
  rejects configuration jumps between steps: the cousin of `max_joint_step`, with the
  same sensitivity to step size. "Only 63% of the path achieved" is a classic symptom.
- **The robot names its branches.** A Cartesian target on a KUKA controller carries
  two extra numbers, Status and Turn, that say which branch and which turn of each
  axis is meant. A straight-line motion that passes through a singularity ends in a
  fault or a crawl, because some axis would have to turn faster than it can.

A question for your team: which IK solver do the production cells use, and how is the
branch chosen for the pick pose and the place pose?

## Reading

- Kevin Lynch and Frank Park, *Modern Robotics*: chapter 4 for forward kinematics,
  section 5.3 for singularities, section 6.2 for numerical inverse kinematics.
  <http://modernrobotics.org>. Chapter 5's Jacobian is the one built in `Evaluate`.
- Samuel Buss, "Introduction to Inverse Kinematics with Jacobian Transpose,
  Pseudoinverse and Damped Least Squares Methods" (2004, revised 2009). Twenty pages on
  why damping works. Search for the title; it is on the author's university page.
- Patrick Beeson and Barrett Ames, "TRAC-IK: An Open-Source Library for Improved
  Solving of Generic Inverse Kinematics" (IEEE-RAS Humanoids, 2015). What a production
  numeric solver adds. Read the introduction and the results table.
- MoveIt's documentation on kinematics configuration and on IKFast,
  <https://moveit.picknik.ai>: how a solver is chosen per group, and its timeout.
- Alexander Elias and John Wen, "IK-Geo: Unified Robot Inverse Kinematics Using
  Subproblem Decomposition" (2022). Current work on closed forms for six-axis arms.

## Office hours

Be ready to explain, with the chapter closed:

- what one column of the Jacobian means, with an example from this arm;
- what goes wrong at a singularity, and what damping gives up to avoid it;
- why the seed decides which answer you get, and why that is useful along a path;
- the four ways a solve can fail (`IkStatus`) and what each tells you to look at.

Questions worth asking your teacher:

- This arm has a closed-form solution. Why does the repository search instead?
- The cycle solved the same pick pose for thirty modules. What changes between them,
  and where is the arm closest to a limit?
