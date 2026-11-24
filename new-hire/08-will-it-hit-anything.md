# Stop 8: Will it hit anything

The collision check: one question, asked tens of thousands of times a plan. "With the
arm at these six joint angles, holding this module, is anything closer to anything
else than it should be?" The planner of stop 9 is built on nothing but that question,
so what the answer does and does not promise decides what a planned path is worth.
About a day.

## Read

1. `src/collision.h`: the comment at the top, all of it. It is long because every
   paragraph is a decision. Then the options and report structs, and the public half
   of `CollisionChecker`.
2. `models/arm.xml`: the `arm_collision` class near the top, and one link's pair of
   geoms (`arm_link_2_visual` and `arm_link_2_collision`).
3. `src/collision.cc`: the four bit constants and `Collides` at the top; then
   `ApplyFilters`, `Counts`, `Query` and `SegmentIsFree`. Leave `Create` and `Pose`
   for a second pass.
4. `tests/collision_test.cc`: read, do not run (it is one of the slow tests). Read
   `MarginFlipsAtTheMeasuredClearance`, `FastModelGivesThePlainModelsAnswers` and
   `LeverArmsBoundHowFarPointsMove`, and say in a sentence what each one proves.
5. `src/motion_planner.h`: the struct `PlanScene`, and in `src/pick_place.h` the
   comment on `CarryScene`.

## The idea

**Shapes, not the robot.** The check does not know about steel and glass. It knows
*geoms*: simple shapes fixed to bodies. The arm has two sets. The drawn meshes are for
your eyes and the camera. The collision meshes are what the check uses, and MuJoCo
collides the *convex hull* of a mesh: the shape you would get by stretching cling film
round it. A hull has no hollows, so it is a little fatter than the link. That errs on
the safe side, and it has a cost you will meet in the lab: two hulls can be close when
the links are not.

**Which pairs count.** Thirty modules lean on each other and on the pallet; `print_cell`
counts 240 contacts among them with the arm nowhere near. None of that is a collision.
The header lists what counts and what does not. In short: a pair counts when at least
one of the two belongs to the arm, the tool or the module being carried. The checker
gives every body one of four roles (`Role` in the class) and decides by role.

**The held module is part of the arm.** Once a module is on the tool it moves when the
joints move. `SetHeldModule(i, module_in_tip)` tells the checker so: at every query the
module is put where the tool tip is at those angles, times the grip offset. That is
stop 2's chain, `module in world = tip in world * module in tip`, run for every
question. The arm is now 2.3 m wider at the wrist, and the module can hit the arm's own
links: that is *self-collision*, checked with a margin of its own.

**Touching on purpose.** A pick touches the glass, and a set-down touches the rails.
`AllowContact` names pairs of bodies that do not count for now. A carry allows none.

**A margin.** "Not touching" is not enough to plan with, because the real arm will not
be exactly where the plan says. So a configuration is *free* only if every counted
pair is at least `margin` apart: 20 mm to the world by default. The checker gets this
almost for nothing. MuJoCo already reports a contact when two geoms are closer than
the sum of their two `geom_margin` values, so the checker gives every geom a share
such that each kind of pair adds up to the margin wanted. Work the shares out for a
margin of 45 mm and a self margin of 25 mm before you read `ApplyFilters`.

**Three phases.** A query runs two MuJoCo calls, `mj_kinematics` then `mj_collision`.
The second works coarse to fine: a *broad* phase throws out bodies whose boxes are far
apart, a *mid* phase does the same per geom, and a *narrow* phase computes the exact
distance for the few pairs left. The header's "Speed" paragraph has the details.

**Yes or no is cheap; how far is dear.** `IsFree` only asks whether anything is inside
the margin. `Check` with `clearance` measures how far apart things are, which the
header puts at four to five times the cost. The planner asks the cheap question while
searching and the dear one once, along the path it kept.

**Only where you ask.** The check answers for one configuration. Between two checked
configurations it has not looked. To check a move, sample it: but how finely? Here is
the argument, and it carries the whole of stop 9. Turn joint 1 by a small angle and
every point of the arm moves along a circle about joint 1's axis, by at most the angle
times its distance from that axis. The header of `src/path_timing.h` estimates the
farthest point, a corner of the held module, at about 4.5 m from joint 1's axis: so
0.01 rad at joint 1 moves nothing more than 45 mm. Each joint has such a *lever arm*
(`LeverArms`), and if all six turn at once their effects at worst add up:

```
farthest any point can move  <=  r1 |dq1| + r2 |dq2| + ... + r6 |dq6|
```

That sum is `SweepBound`. Sample a move so that it is at most 30 mm between
neighbours, and every pose in between is within 15 mm of one that was checked. Check
with a margin 15 mm larger than you need, and the gaps are covered. Angle times
distance, again.

**What the checker believes.** The checker has its own picture of the cell, and
somebody has to put the obstacles in it. `SetScene` copies the simulator's truth,
right for things fixed by design and for tests. A real cell does not know where its
pallet truly is, so the cycle builds the scene from what it sensed and did
(`CarryScene`): the stack where the camera saw it, the tube where the encoder says,
each placed module where the script put it. A path is only as clear as that picture.

## C++ you meet here

**A private copy to scribble on.** `CollisionChecker::Create` copies the model with
`mj_copyModel` and makes its own `mjData`. The class then rewrites that copy freely
(margins, contact bits) and writes joint angles into its own state at every query. The
simulation's model and state are never touched; `SetScene` takes a `const mjData&`.
Owning a copy is how a class gets the right to change something.

**Bit masks.** MuJoCo tests a pair of geoms only if `contype` of one shares a bit with
`conaffinity` of the other. An `int` here is 32 separate yes-or-no flags. `kLinkBit =
1`, `kToolBit = 2`, `kHeldBit = 4`, `kObstacleBit = 8` are one flag each; `|` joins
flags (`kLinkBit | kObstacleBit`), `&` keeps the flags two values share, and `!= 0`
asks whether any are left. `Collides` is that test in two lines. In `Create`, `&= ~(...)`
switches named flags off: `~` flips every bit. Python and Java have the same operators;
C++ code that talks to C libraries uses them far more.

**Small types.** `enum class Role : unsigned char` says each value is stored in one
byte; there is one per body. `std::pair<int, int>` is two values with no names but
`first` and `second`, and `for (const auto& [a, b] : pairs)` unpacks each pair into two
names, as Python's `for a, b in pairs` does.

**Sorting and searching with a lambda.** In `Query`, `std::find_if` takes a lambda
that says what to look for and `std::sort` one that says which of two comes first. The
sort breaks ties by geom number, so the report's order does not depend on the order
MuJoCo happened to find the contacts in. Small decisions like that make a run
repeatable.

**A query that is not `const`.** `IsFree` reads like a question, but it is not marked
`const`: it writes the angles into the checker's scratch state and counts the query.
Two threads sharing one checker would overwrite each other's angles halfway through.
Hence the header's rule, one checker per thread. When a method that sounds read-only
is not `const`, ask what it writes.

**Counting, not timing.** `queries()` counts every configuration checked. A count is
the same on every machine; a time is not. The repository's tests bound counts and only
report times (`std::chrono` appears properly at stop 9).

## Lab

Write your prediction down before each run.

1. **What does not count.** `bazel run --config=opt //:print_cell` and find the
   contacts "at arm_home after settling". Between which bodies are they, and how many
   of them would the checker call a collision?
2. **How close the scripted cycle comes.** The scorer's side measures clearances from
   the truth during a real cycle:

   ```
   bazel run --config=opt //:run_cell -- --pretty --nominal --modules 1 --verbose \
       2>&1 | grep "mm  in "
   ```

   Each line is one group of pairs: the smallest distance seen over the whole cycle,
   the step it was seen in, and between what. (1000.0 means nothing came within a
   metre.) Predict first: in which step is the held module nearest to something it
   must not touch, and to what? Then compare the smallest figure with the checker's
   default margin, and with the 45 mm the planner of stop 9 plans with.
3. **Reading the planner's margins.**
   `bazel run --config=opt //:print_plan -- --pretty --seeds 5`. Its first line gives
   the two margins it planned with. Why is every `clear_mm` above the first, and what
   might the two things be that are always about 81 mm apart in `self_mm`? The same
   line says a 5 mm corner allowance became a "max deviation" of 0.60 mrad and a 5 mm
   tracking allowance became 0.26 mrad. Read in `src/motion_planner.cc` how `PlanMove`
   computes the two, and work back to the sum of the six lever arms. Run it again with
   `--return` (the bare arm) and do the same. How much does a held module add?
4. **Changing no answer, only the time.** In a worktree, set
   `CollisionOptions::skip_obstacle_pairs` to `false` in `src/collision.h` and run the
   command of run 3 again. Predict which columns change and by roughly what factor.

Remove the worktree when you are done.

## Pitfalls

- **"Free" taken to mean "not touching".** It means at least the margin away. A pose
  can be refused with nothing in contact.
- **Trusting a segment because its ends are free.** `SegmentFindsThePostBetweenFreeEnds`
  in the test is that mistake, caught.
- **A stale scene.** The checker's obstacles are where somebody last put them. A
  module placed on the tube and not reported to the checker is not there for it.
- **Forgetting the held module.** The bare arm is free in many poses where the module
  is deep in the stack (`HeldModuleIntoTheStackWhileTheBareArmIsFree`).
- **Allowed contacts left on.** A pair allowed for the grip and never cleared stays
  invisible for the carry.
- **Hulls are not the links.** A self margin of 80 mm sounds harmless and leaves the
  wrist no room; the comment on `CollisionOptions::self_margin` gives the measurement.

## In the MoveIt cells

MoveIt keeps the same things under other names.

- The checker's picture of the cell is the **planning scene**: the robot from the
  URDF, plus *collision objects* that nodes add and move. It has to be kept current
  there too; a stale or missing object is the classic cause of a planned path that
  hits something.
- The geometry is done by a library, FCL by default, with Bullet as an alternative.
  Links have separate visual and collision geometry in the URDF, as here.
- Which pairs are never checked is the **allowed collision matrix**. It starts from
  the SRDF, generated once by sampling many poses and noting which links always or
  never touch. Here MuJoCo's parent-child rule and two explicit excludes do that.
- `SetHeldModule` is an **attached collision object**: an object fixed to a link, with
  a list of links it may touch. `AllowContact` is an entry in the matrix made for one
  move. Margins exist as **padding** on the robot's links, set in configuration.
- The spacing of checks along a move is where the two differ most. In MoveIt's OMPL
  configuration it is `longest_valid_segment_fraction`, a fraction of the size of the
  whole joint space. It is not a distance in metres, and nothing ties it to the
  padding. Here the spacing is in metres of worst-case motion and the margin is sized
  to cover it. With a 2.3 m module on the wrist, a spacing that looks small in radians
  is not small at the module's corner.

For your team: how is the planning scene kept current on the real cells, how much
padding do they use, and has anybody worked out what the check spacing means in
millimetres at the edge of the part?

## Reading

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 10, section 10.2: obstacles
  in configuration space and distance to obstacles, in a few pages. Free at
  <http://modernrobotics.org>.
- Christer Ericson, *Real-Time Collision Detection* (2005): the standard book. Dip
  into the chapters on bounding volumes and on GJK; the three phases are made of them.
- MuJoCo, "Computation", <https://mujoco.readthedocs.io/en/stable/computation/index.html>:
  the section on collision detection, for what `contype`, `conaffinity` and `margin`
  mean in the engine this code calls.
- Jia Pan, Sachin Chitta and Dinesh Manocha, "FCL: A General Purpose Library for
  Collision and Proximity Queries" (ICRA 2012): the library under MoveIt. Read the
  introduction and the list of query types.
- Balakumar Sundaralingam and others, "cuRobo: Parallelized Collision-Free Robot
  Motion Generation" (2023), <https://curobo.org>: the current direction. The robot is
  a set of spheres and the world a distance field, so thousands of checks run at once
  on a graphics card. Read the collision section only.

## Office hours

Be ready to explain, with the chapter closed:

- which pairs of shapes count as a collision here, and three kinds that do not;
- how the checker gets a 20 mm margin out of MuJoCo without measuring distances;
- why a move must be sampled, and how the lever arms say how finely;
- what `SetHeldModule` changes, in the geometry and in the lever arms;
- where the checker's picture of the cell comes from in a real cycle.

Questions worth asking your teacher:

- The scripted cycle brings the module within 30 mm of the pallet's deck, closer than
  the planner's 45 mm. How can a planned carry follow straight on from it?
- What would a checker that could look at a whole move at once, not at samples, save?
