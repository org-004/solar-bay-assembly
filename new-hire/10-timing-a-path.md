# Stop 10: Timing a path

The planner of stop 9 hands back a list of joint angles. It says where the arm should
pass, and nothing about when. This stop is the piece that puts a clock on it: how to
go through a list of waypoints in one motion, as fast as the motors' limits allow,
without leaving the corridor that was checked for collisions. It is also where cycle
time is won or lost. About a day.

## Read

1. `src/path_timing.h`: the comment at the top. Read parts 1 and 2 twice; they are the
   method. Then the comment on `PathTimingOptions::max_deviation`, which ties this
   stop to stops 8 and 9, and the public half of `Trajectory`.
2. `src/arm_follower.h` again: the comments on `MoveThrough`, `MoveAlong` and
   `Follow`. Three ways to go through waypoints; this stop is the second and third.
3. `src/path_timing.cc`: the constants at the top, then `Trajectory::Plan` from its
   first line to its last. When it calls something, read that: `Time` (the two
   passes), then `Reach` and `Peak`, then `StretchLimits`. Leave `BuildShape` and
   `Simplify` for a second pass.
4. `tests/path_timing_test.cc`: the names of the tests are a list of what is promised.
   Read `TimeSavedAtOneCornerByItsAngle` and `RandomPolylinesKeepEveryGuarantee`.
5. `tests/path_follow_test.cc`: `StraightToolLineAsOneMotion`.
6. `src/motion_planner.cc`: the last fifteen lines of `PlanArmMove`, where a planned
   path is timed.

## The idea

**A path and a trajectory.** A *path* is a shape: these joint angles, then these, with
straight lines between, in joint space. A *trajectory* is a path with a clock: where
to be at every instant, and so how fast and with what acceleration. The same road can
be driven at many speeds. Stop 3's `JointMove` is the simplest trajectory: one
straight line, from rest to rest.

**Why not stop at every waypoint.** That is what running one `JointMove` after another
does (`MoveThrough`). It is safe and exact, and slow: every stop costs the time to
brake and to get going again. A 0.3 m straight line of the tool tip, solved every
centimetre, is thirty-one waypoints. The header gives the bill for a curve like that:
9.7 s stopping at each, against 1.63 s in one motion.

**Why not just keep going.** At a corner the direction of travel changes. Changing it
in an instant would take infinite acceleration. So at each corner there are two honest
choices: come to rest there, or round it.

**Shape first: a blend.** To round a corner the path leaves the incoming line a
distance `l` before the corner and joins the outgoing line `l` after it, turning
gradually in between. The rounded path no longer passes through the corner, and how
far it strays is known exactly: at most `(3/16) * l * sin(turn)`. So a limit on
straying, `max_deviation`, fixes the largest `l` allowed. Try the numbers: a right
angle and a `max_deviation` of 0.002 rad allow `l` of about 0.011 rad. That is a very
tight bend, and a tight bend must be taken slowly. A gentle corner gets a long blend.

**Then speed.** With the shape fixed, call `s` the distance travelled along it. The
only thing left to choose is how `s` grows with time: one speedometer for the whole
arm. Each joint's speed is the path's direction for that joint times the path speed.
Acceleration has two parts, exactly as in a car:

```
joint acceleration  =  direction * (change of path speed)   speeding up or braking
                     + bend * (path speed)^2                cornering
```

Both draw on the same motor. On a straight stretch the bend is zero and everything is
available for speeding up. On a blend, the faster you go the more cornering takes,
which caps the speed through it. Jerk has one more term; the header writes all three.

**Two passes.** The path is cut into stretches, each with its own caps. How fast may
the arm be at each boundary between stretches? Two sweeps answer it.

- *Backward, from the end:* how fast may I be here and still slow down in time for
  everything that follows? The end is at rest, so start there and work back.
- *Forward, from the start:* how fast can I have got here? The start is at rest too.

The speed at each boundary is the smaller of the two, and inside a stretch the arm
speeds up as far as it can while still able to slow to its exit speed. In the code
that is `Time`, with `Reach` for one boundary and `Peak` for one stretch.

**Choices, tried.** Some decisions have no formula, so the code tries each and keeps
the quickest: stop or blend at each corner; drop waypoints that lie almost on the line
between their neighbours; how much of a blend's acceleration to keep for speeding up.
One candidate is always "stop at every corner", so it is never slower than
`MoveThrough`.

**What is promised.** Read the list in the header: every joint within its velocity,
acceleration and jerk limits everywhere; never further than `max_deviation` from the
polyline through the waypoints; at rest exactly on the first and the last. Not
promised: the fastest possible motion. It is close, and the header says how close.

**Where `max_deviation` comes from.** It is an angle, in joint space. Collisions
happen in metres. The bridge is stop 8's lever arms: a path within `d` radians of the
checked polyline keeps every point of the arm within `|r| * d` metres of where it was
checked, with `|r|` the length of the vector of lever arms. Stop 9's budget set aside
5 mm for corners, and `PlanMove` divides that by `|r|` to get the 0.60 mrad you saw on
`print_plan`'s first line. The cycle's straight lines use two fixed values instead,
`deviation_near` and `deviation_far` in `PickPlaceOptions`.

## C++ you meet here

**How a long numerical file is laid out.** `path_timing.cc` is 800 lines and has two
public functions, both at the bottom. Everything above them is inside an unnamed
namespace: constants first, each with the reasoning for its value; then small structs
that exist only in this file (`Stretch`, `Timing`, `Shape`, `Polyline`); then short
functions that each do one piece of the maths, with the derivation in the comment.
Read such a file from the public function down, not from the top.

**A type inside a type.** `Trajectory::Piece` is a struct declared inside the class.
Nesting only scopes the name (outside the class it is `Trajectory::Piece`); unlike an
inner class in Java it has no link to any `Trajectory`. `Segment` is nested and private.

**Tolerances with names.** Computed floating-point numbers are never compared with
`==`. The file's first lines are its tolerances, `kSamePoint`, `kStraightOn`,
`kSameTime` and the rest, each saying what it is too small to matter for. Note also
tests shaped like `if (!(duration > 0.0)) return;`. That is not `duration <= 0.0`: a
NaN fails every comparison, so the first form catches a NaN and the second lets it by.

**Try, compare, keep.** In `Plan`, the lambda `consider` times one candidate and
overwrites `best_time`, `best_shape` and `best_stretches` if it wins; the loops round
it run over the small constant arrays `kSimplify` and `kShares`. `best_shape` is a
`std::optional` because before the first candidate there is no best yet.

**Searching for a number.** Where a quantity has no formula but a yes-or-no test that
flips once, the code halves an interval 60 times (`kBisections`): see `Reach`. The line
`(test ? low : high) = mid;` assigns to whichever of the two the test picks; a
conditional expression can name a variable, not only a value.

**References into a vector that grows.** In `Plan`, the lambda `append` takes `const
Segment& last = segments.back();`, uses it, and only then calls `push_back`. The order
matters. A vector keeps its elements in one block of memory and may move the whole
block when it grows, after which `last` would refer to memory that is gone. A Python
or Java list holds references to objects that stay put; a vector holds the objects.

## Lab

Write your prediction down before each run. Runs 1 and 2 are tests that print what
they measure; `--test_output=all` shows it.

1. **One corner.** `bazel test --config=opt //:path_timing_test --test_output=all` and
   find the table headed "one corner between two 0.5 rad legs". Predict first: for a
   right-angle corner and a deviation of 0.002 rad, how much quicker is rounding than
   stopping? At what turn does rounding start to pay? Then find the line for "31
   waypoints on a gentle curve".
2. **One straight line, four ways.** The tool tip is to move 0.3 m in a straight
   line, solved every centimetre. It can stop at every centimetre, blend within 0.002
   rad, blend within 0.0002 rad, or ignore the line and make one joint move from end
   to end. Rank the four by time, and separately by how far the tip strays. Then
   `bazel test --config=opt //:path_follow_test --test_output=all` and find the block
   "pull back, tool level, no module".
3. **Blending in the cycle.** Note the cycle time of
   `bazel run --config=opt //:run_cell -- --pretty --nominal --modules 1 --phases`.
   In a worktree set `PickPlaceOptions::blend_lines` to `false` in `src/pick_place.h`.
   Predict whether the cycle gets slower or quicker, then run it. Explain the result
   with the fourth row of run 2.
4. **A looser corridor.** Put `blend_lines` back and change `deviation_near` from
   `2e-4` to `2e-3`. Which steps change, by how much, and what happens to the
   placement? Would you ship it?
5. **Corners on the carry.**
   `bazel run --config=opt //:print_plan -- --pretty --seeds 5`, then with `--corner
   0`, which forbids rounding altogether. Predict the cost in trajectory time. Why is
   it so different from run 1's gentle curve? (Careful: the flag also changes the
   planning margin, so the paths are not quite the same paths.)

Remove the worktree when you are done.

## Pitfalls

- **Calling a path a trajectory.** A list of waypoints has no times. Something must
  add them, and that something has settings.
- **Rounding corners nobody budgeted for.** The collision check looked at the
  polyline. A blend leaves it. If the deviation is not covered by the margin, the
  timed motion was never checked.
- **More waypoints for "accuracy".** Stopping at each costs time per waypoint.
  Blending many tiny corners costs jerk; the header's "Fewer waypoints" explains.
- **Slowing a motion by scaling only velocity.** Played at a fraction `k` of full
  speed, velocity scales by `k`, acceleration by `k^2` and jerk by `k^3`
  (`PathTimingOptions::speed`).
- **Comparing durations of different paths** and crediting the timing. Lab 5 warns you.
- **Ignoring acceleration and jerk limits.** The trajectory looks fine on paper, the
  motors cannot follow it, and the error eats stop 9's tracking allowance.

## In the MoveIt cells

In MoveIt this stop is a separate step after planning, called *time
parameterization*, run as an adapter in the planning pipeline.

- The planner returns waypoints without times. The usual adapter is time-optimal
  trajectory generation, after Kunz and Stilman: it rounds each corner with a circular
  arc inside a path tolerance, then finds the fastest speed along the result under
  velocity and acceleration limits, by the same forward and backward reasoning as
  here. It has no jerk limit. A second adapter based on Ruckig can smooth the result
  to respect jerk.
- The path tolerance is that adapter's `max_deviation`: an angle in joint space, set
  in configuration. Nothing connects it to the collision padding. With a long part on
  the wrist, work out what it means in millimetres before trusting it.
- Velocity limits come from the URDF. Acceleration limits are not in a URDF at all;
  they come from a separate `joint_limits.yaml`, and jerk limits too if they are
  given. Missing or guessed acceleration limits are a common reason a real arm runs
  slower than it could, or faults on following error. A request can also scale
  velocity and acceleration by two separate factors; compare the `k`, `k^2`, `k^3`
  rule above.
- TOPP-RA is the well-known general method for the same problem and can take torque
  limits as well. It is a library, not part of MoveIt.
- Then the trajectory leaves MoveIt. A trajectory controller interpolates between its
  points and streams set-points to the robot's own controller, which closes the loop.
  When the robot instead runs a native program, its controller does the timing and the
  rounding itself. KUKA calls rounding *approximate positioning*: a point marked for it
  is passed near, not through, within a programmed distance or percentage. It is
  `max_deviation`'s cousin, measured in the controller's own terms.

For your team: which time parameterization do the real cells use, what is its path
tolerance in millimetres at the part's edge, where do the acceleration limits come
from, and does anything check the rounded path for collisions?

## Reading

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 9, "Trajectory Generation",
  free at <http://modernrobotics.org>: 9.1 for path against trajectory, 9.4 for
  time-optimal timing along a path, with the picture every paper on the subject uses.
- Tobias Kunz and Mike Stilman, "Time-Optimal Trajectory Generation for Path Following
  with Bounded Acceleration and Velocity" (Robotics: Science and Systems, 2012): the
  method MoveIt ships. Read the sections on blending corners and on the limit curve.
- Hung Pham and Quang-Cuong Pham, "A New Approach to Time-Optimal Path
  Parameterization Based on Reachability Analysis" (IEEE Transactions on Robotics,
  2018), and <https://github.com/hungpham2511/toppra>: TOPP-RA. The backward and
  forward passes of this stop, made general. Read the introduction and the
  description of the two passes.
- Lars Berscheid and Torsten Kröger, "Jerk-limited Real-time Trajectory Generation
  with Arbitrary Target States" (Robotics: Science and Systems, 2021), and
  <https://github.com/pantor/ruckig>: Ruckig, the current standard for jerk-limited
  motion computed on the fly. `docs/ompl-integration.md` has a note on what bringing
  it into this repository would take ("Applied to Ruckig").

## Office hours

Be ready to explain, with the chapter closed:

- the difference between a path and a trajectory, with an example of each from the
  code;
- why a sharp corner inside a tight deviation is taken almost at rest;
- the backward and the forward pass, in the words you would use for driving a car;
- how a corner allowance in millimetres becomes a `max_deviation` in radians;
- what the trajectory guarantees and one thing it does not.

Questions worth asking your teacher:

- Lab 3 came out the opposite way from what most people expect. What was really being
  compared?
- If a library such as Ruckig or TOPP-RA replaced this file, what would have to be
  checked before the gate could be trusted again?
