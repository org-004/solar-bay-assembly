# Stop 3: Moving one axis

How a joint gets from one angle to another: not at once, and not any way it likes, but
as fast as limits on speed, acceleration and jerk allow. Then six joints together. This
is what fills the `to standoff`, `swing` and `return` rows of the table from stop 0, and
by the end you will predict one of those times by hand. About a day.

## Read

1. `src/profile.h`: the comment at the top, then the class. 93 lines.
2. `src/profile.cc`: `Advance`, then `Profile::Trapezoid` (short), then
   `Profile::SCurve` with its comments, then `Profile::At`.
3. `src/print_profile.cc`: a whole program that uses the class.
4. `src/joint_move.h`, the comment at the top; in `src/joint_move.cc`, `FractionLimit`
   and `JointMove::Plan`.
5. The first twenty lines of `src/arm.cc`: the arm's real limits.
6. `tests/profile_test.cc`: the comment above the checking helper, on the two ways a
   limit is checked, and `ShortMoveIsATriangle`. `docs/img/profile.png` is the picture.

## The idea

**Four quantities.** A joint has a position (its angle). Speed is how fast the position
changes, acceleration how fast the speed changes, and jerk how fast the acceleration
changes. In a car: where you are, the speedometer, how hard the seat pushes your back,
and how suddenly that push comes on.

**Why each is limited.** A motor has a top speed. Its torque is limited, and torque is
what accelerates the arm's mass, so acceleration is limited too. Jerk is different:
nothing in a rigid arm limits it. But a real arm is slightly springy in its gearboxes
and links, and a 2.3 m sheet of glass on rubber cups more so. A force that comes on
suddenly makes springy things ring. Limiting jerk makes forces come on gradually. The
comment in `src/arm.h` is frank about it: speed and acceleration limits are KUKA's
figures, and the jerk limit "is ours to change".

**The trapezoid.** The quickest way from rest to rest within a speed limit `v` and an
acceleration limit `a`: accelerate as hard as allowed, cruise at top speed, brake as
hard as allowed. Speed against time is a trapezoid.

```
 speed                              speed
   v |    ____________                 |
     |   /            \                |      /\        a short move never
     |  /              \               |     /  \       reaches top speed
     | /                \              |    /    \
     +---------------------> time      +------------> time
```

Reaching top speed takes `v / a` seconds, so a move of distance `d` takes
`d / v + v / a`. A move too short to reach top speed turns round at its midpoint and
takes `2 * sqrt(d / a)`.

**The S-curve.** The trapezoid's acceleration jumps between `+a`, `0` and `-a`: its jerk
is unbounded at the corners. The S-curve gives the acceleration ramps instead, at a
jerk limit `j`. That makes up to seven segments: acceleration ramps up, holds, ramps
down; cruise; then the mirror image to stop. Each ramp takes `a / j` seconds, and a
long move ends up exactly that much slower than the trapezoid: `d / v + v / a + a / j`.
Shorter moves drop segments, which is what the `std::min` of three candidates in
`Profile::SCurve` decides.

**The arm's numbers.** Joint 1 may turn at 2.04 rad/s and accelerate at 1.98 rad/s².
The jerk limit is ten times the acceleration limit (`kJerkPerAcceleration`), so every
ramp takes a tenth of a second.

**Asking a profile.** A `Profile` is planned once and then asked, as often as you like,
"where should the joint be at time `t`?" (`Profile::At`). It has no clock of its own.
The follower in stop 4 asks every 2 ms.

**Six joints: joint space.** Think of the arm's six angles as one point in a space
with six directions. A `JointMove` goes from the start point to the goal point along
the straight line between them:

```
q(t) = start + (goal - start) * fraction(t)
```

with one `Profile` running `fraction` from 0 to 1. All six joints therefore start
together and stop together. The profile's limits are set so that no joint exceeds its
own: each moving joint allows `limit / |distance|`, and the smallest wins
(`FractionLimit`). The joint that needs longest sets the pace and the rest go slower
than they could.

**Straight for the joints is not straight for the tool.** Every joint turns about an
axis, so the tool tip moves on arcs. Along a joint move the tip strays from the
straight line between its ends by an amount that grows with the square of the length:
the comment at the top of `src/pick_place.h` gives about `2.2e-4 * L^2` mm for a leg
of `L` mm. That is nothing over a centimetre and 2 cm over 30 cm. So the cycle uses
plain joint moves where there is room, and many short ones where the tool must travel
straight (stop 5).

**Playing a move slower.** To run the same move at a fraction `s` of full speed, time
is stretched by `1 / s`. Speeds then scale by `s`, accelerations by `s^2` and jerks by
`s^3`. That is what `MoveOptions::speed` does, and why 0.9 costs about a tenth more
time and saves about a fifth of the acceleration.

## C++ you meet here

**A class.** `Profile` bundles data with the functions that work on it. What follows
`public:` is its face; what follows `private:` only its own functions may touch. Data
members end in an underscore (`duration_`). A member function marked `const`, as in
`double duration() const`, promises not to change the object, and the compiler holds it
to that. Python has no private; Java's `private` and `final` are the nearest.

**Declared here, defined there.** The header declares `Sample At(double t) const;`.
The `.cc` defines it as `Sample Profile::At(double t) const { ... }`, where `Profile::`
says whose `At` this is. One-liners such as `duration()` are defined in the header.

**A factory instead of a constructor.** `Profile`'s constructor is private
(`Profile() = default;`), so outside code cannot make a blank one. The only way in is
`Profile::Trapezoid(...)` or `Profile::SCurve(...)`: `static` member functions, called
on the type and not on an object. A `Profile` that exists is therefore always a valid
one. Compare a `@classmethod` constructor, or Go's `NewThing() (Thing, error)`.

**`std::optional`.** The factories return `std::optional<Profile>`: a box holding
either a `Profile` or nothing. Inside, `return std::nullopt;` gives back the empty box
and `return profile;` a full one. The caller tests it, `if (!trapezoid)`, and reaches
in with `trapezoid->duration()` or `*trapezoid`. This is the repository's way of
saying "this can fail" without exceptions, and you will see it several hundred times.
The trap: reaching into an empty box is not an exception, it is undefined behaviour.
Test first. Java's `Optional` and Python's `T | None` are the same idea with a softer
landing.

**Plain structs.** `Limits` and `Sample` are `struct`s: every member public, each with
a default written beside it (`double velocity = 0.0;`), so `Sample{}` is all zeros. Use
a struct for plain data and a class when there are rules to keep. `Profile::Segment` is
a struct declared inside the class's private part: a helper type nobody else can name.

**`enum class` and `std::vector`.** `enum class ProfileShape { kTrapezoid, kSCurve }`
is a closed set of names. You must write `ProfileShape::kSCurve` in full, and it will
not silently turn into an integer. `std::vector<Segment>` is a growable array:
`push_back` appends, `back()` is the last element, `empty()` asks, and
`for (const Segment& candidate : segments_)` visits each in turn without copying it.

## Lab

Write your prediction down before each run. Run 5 edits code: use a throwaway worktree
(`new-hire/AGENTS.md`, "Labs").

1. **By hand, then by program.** A quarter turn (1.5708 rad) with limits 2 rad/s,
   5 rad/s² and 50 rad/s³. Work out how long the trapezoid and the S-curve take. Then
   `bazel run --config=opt //:print_profile -- --pretty`; the two durations are on the
   line it prints before the table. Read down the acceleration and jerk columns of
   each shape.
2. **A short move.** The same limits over 0.05 rad:
   `bazel run --config=opt //:print_profile -- --pretty 0.05 2 5 50`. Predict first:
   does either shape reach 2 rad/s? Does the S-curve reach 5 rad/s²? Which shape loses
   more time compared with run 1, and why?
3. **What jerk costs.** The quarter turn again with the jerk limit at 500, 5000 and 5
   (the last number on the command line). Predict each S-curve's duration. One of your
   three predictions will be wrong if you use the long-move formula; find out which
   and why.
4. **A row of the cycle.** In the `swing` step joint 1 turns half a turn (pi rad)
   at 0.9 of full speed, and it sets the pace. From the limits in `src/arm.cc`,
   predict how long `swing` takes. Check with
   `bazel run --config=opt //:run_cell -- --pretty --nominal --modules 1 --phases`,
   and note the cycle time and the "peak torque fraction" line for the next run.
5. **Change the jerk limit.** In the worktree set `kJerkPerAcceleration` in
   `src/arm.cc` to 100, predict `swing`, and run the cycle again. Then set it to 2.
   What happened to the cycle time, the placement error and the peak torque each
   time? The tenfold jerk looks like a free gain here. Is it?

## Pitfalls

- **Slowing down by scaling everything alike.** Half speed is half the speed limit, a
  quarter of the acceleration limit and an eighth of the jerk limit. Scale them all by
  a half and you get a different, more abrupt move.
- **Trusting a sampled peak.** A table with a row every few milliseconds can miss the
  instant of top speed. Run 2's printed peak is a little under the true one.
- **Thinking a joint move is straight.** It is straight for the joints. Over 30 cm
  the tool bows by centimetres.
- **Degrees per second.** Robot data sheets give speeds in degrees; the code is in
  radians. 2.04 rad/s is 117 degrees per second.
- **Concluding from the simulation that jerk does not matter.** The simulated arm and
  module are rigid, so nothing rings. Run 5 measures what the limit costs, not what it
  buys.
- **Using an optional without testing it.** `*Profile::SCurve(d, limits)` with a zero
  limit does not throw. It does something undefined.

## In the MoveIt cells

- **Timing is a separate step.** MoveIt's planners return a path: joint angles with no
  times. A time-parameterization step then assigns the times within each joint's speed
  and acceleration limits, and optionally a smoothing step adds jerk limits. The limits
  live in a configuration file per robot, not in code, and a pair of scaling factors
  between 0 and 1 slows a motion down. Check how those factors scale acceleration
  against the `s^2` rule above before you trust a "half speed" run.
- **Somebody else samples the plan.** The result is a list of points with times, sent
  to a trajectory controller, which interpolates between the points at its own rate.
  `Profile::At` is that interpolation, written out.
- **The robot's own controller has motion types.** On a KUKA, PTP is a synchronized
  joint move, the same thing as `JointMove`: all axes start and stop together and the
  slowest leads. LIN is a straight line of the tool. The controller plans their speed
  profiles itself, with its own limits. MoveIt's Pilz planner offers the same two
  motion types from the ROS side.

A question for your team: on the production cells, which of these three owns the speed
profile of the carry, and where are its limits set?

## Reading

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 9, "Trajectory Generation",
  sections 9.1 and 9.2: point-to-point moves, time scaling, the trapezoid and the
  S-curve. <http://modernrobotics.org>, with videos.
- Luigi Biagiotti and Claudio Melchiorri, *Trajectory Planning for Automatic Machines
  and Robots* (Springer, 2008), chapter 3: the reference for the trapezoid and the
  seven-segment "double S" profile, every case worked out. For when you need to change
  `profile.cc`, not before.
- Lars Berscheid and Torsten Kröger, "Jerk-limited Real-time Trajectory Generation
  with Arbitrary Target States" (Robotics: Science and Systems, 2021), and the library
  it describes, Ruckig, <https://github.com/pantor/ruckig>. Current practice: the same
  jerk-limited profiles, but from any moving state to any other. Read the README;
  `docs/ompl-integration.md` in this repository discusses bringing it in.
- cppreference on `std::optional`, <https://en.cppreference.com/w/cpp/utility/optional>:
  the first screen, and the list of ways to look inside.

## Office hours

Be ready to explain, with the chapter closed:

- position, speed, acceleration and jerk, and what limits each on a real arm;
- why the S-curve of a long move is exactly `a / j` slower than the trapezoid;
- which joint sets the pace of a joint move, and what the others do;
- why a factory that returns `std::optional` is used here in place of a constructor.

Questions worth asking your teacher:

- The conveyor uses a `Profile` too. What sets its acceleration limit?
- If the jerk limit is "ours to change", how would you choose it for a real cell?
