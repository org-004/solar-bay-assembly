# Stop 9: Finding a path

The motion planner: given where the arm is and where it should end up, find joint
angles in between along which nothing hits anything. This is the part people mean when
they call this repository "the planner", and the part most likely to be measured
against the production stack. About a day and a half; the lab is the longer half.

## Read

1. `src/rrt.h`: the comment at the top, slowly. It is a short account of RRT-Connect
   written for this arm. Then the comment on the metric, `RrtOptions` (every option
   carries a measurement) and the comment above `ShortenPath`.
2. `src/rrt_worlds.h`: the made-up worlds the planner is tested on.
3. `src/rrt.cc`: `CheckEdge`, then `Planner::Extend` and `Planner::Connect`, then the
   loop at the bottom of `PlanRrtConnect`. About eighty lines carry the algorithm.
4. `src/motion_planner.h`: the comment at the top, which is what had to be decided for
   *this* arm. Then `PlanMove` and `PlanArmMove` in `src/motion_planner.cc`.
5. `docs/img/carry.png` and the README's "Planning" paragraph.
6. `docs/ompl-integration.md`: only "What the comparison needs" and "A proposed
   design": what it would take to compare this planner fairly with OMPL, the library
   MoveIt uses.

## The idea

**One point is one pose of the whole arm.** Six joint angles fix where every part of
the arm and the module is. Treat the six numbers as one point in a six-dimensional
space, the *configuration space*, and a motion of the arm is a curve in it. The post,
the stack and the conveyor each become a forbidden region of that space: every
combination of angles that would put something inside them. Those regions have no
formula. All we have is stop 8's question, asked of one point at a time.

**Why not try every point.** The header does the sum: a grid fine enough to be useful
has about 10^15 cells. So the planner *samples*: it tries points, keeps the free ones
and joins them with short straight lines, in joint space, whose points it also tries.
Its cost is counted in questions asked.

**RRT-Connect.** Grow two trees of free points, one from the start and one from the
goal.

```
   start tree                               goal tree

     S---o---o                                o---G
          \                                  /
           o---o===N   <- - - - - - - - - -  o
                (1)             (2)
```

Each turn: draw a random point in the joint box. (1) Find the node of one tree
nearest to it and take one step from there toward the point, if that step is free;
call the new node `N` (`Extend`). (2) The other tree then tries to reach `N`, step
after step in a straight line, until it arrives or is blocked (`Connect`). If it
arrives, the trees have met and the path runs up one and down the other. Otherwise
they swap roles. The header explains why random pulls spread a tree outward quickly,
and why two trees meet sooner than one tree finds the goal.

**What the promise is.** "Probabilistically complete" means: if a path exists, the
chance of having found one tends to 1 as the planner is given more turns. It does
*not* say how many turns. It cannot tell "there is no path" from "not found yet". It
does not promise a short path. And it says nothing about the gaps between checked
points, which is stop 8's problem and comes back below.

**Jagged, then taut.** A raw path is a chain of steps toward random points, so it
wanders. `ShortenPath` picks two points on it and, if the straight line between them is
free, replaces the stretch between. Repeat, and the path is pulled toward a taut
string. What shortcutting cannot do is change which side of an obstacle the string
passes. So `PlanMove` plans four times from different seeds and keeps the shortest.

**How far is far.** Nearest node, step length and path length all need a distance
between configurations. A radian at joint 1 swings the module's far corner some four
metres; a radian at joint 6 turns it on the spot. So each joint's difference is
weighted by its lever arm from stop 8, and a distance comes out as roughly "metres of
motion of the farthest point". The step is 3 m of motion; a carry is about 18.

**Intervals, not circles.** Joint 6 can turn nearly a full turn either way. To the
planner 170 and -190 degrees are different points, 6.3 rad apart, as they are for the
cables. Two consequences are decided in `PlanMove`: the box for joints 4 and 6 is
narrowed to the span the move needs plus some slack, so a path cannot wind the wrist
up; and joint 1's goal angle decides which side of the cell the carry goes round.

**The margin budget.** This is the argument that makes a planned path trustworthy,
and it is only addition. The real arm must stay 20 mm from everything. What it really
does differs from the checked points in three ways, each bounded in millimetres by a
lever-arm argument:

```
   20 mm   required from every obstacle
 + 15 mm   between checked points: half of the 30 mm edge resolution
 +  5 mm   corners rounded when the path is timed (stop 10)
 +  5 mm   the arm trailing its plan (stop 4)
 = 45 mm   what every checked point must keep
```

If every checked point keeps 45 mm, the moving arm keeps 20. Among the arm's own parts
the required margin is zero, so the same three allowances give 25 mm. The header
calls this *soundness*; `MotionPlanReport` carries every figure the planner used.

**The same answer twice.** Every random draw comes from a seed, so the same question
gives the same path. One thing can break that: a limit in seconds, since whether a
hard search finishes inside ten seconds depends on the machine. The header says which
limit should bind (iterations); stop 12 shows why the repository cares.

## C++ you meet here

**A function as a value.** `using IsFree = std::function<bool(const Eigen::VectorXd&)>`
names the type "anything callable with a configuration that returns yes or no".
`PlanRrtConnect` takes one and knows nothing else about the world: not MuJoCo, not the
arm. That is why it can be tested on a point among circles (`src/rrt_worlds.h`) and
why the OMPL report can hand OMPL the same checker. In `PlanMove` the value is a
lambda, `[&checker](const Eigen::VectorXd& q) { return checker.IsFree(q); }`; the
`[&checker]` means it holds a reference to the checker, so it must not outlive it. A
template parameter would do the same job a little faster and move the code into the
header; `std::function` keeps `rrt.cc` an ordinary source file, and the check it wraps
takes microseconds, beside which the wrapping does not show. (`using` only gives a
type a second name: `JointPath` is a vector of vectors, not a new type.)

**Integers of a stated size.** An `int` is 32 bits on these machines and tops out near
two billion. Counts that could pass that are `std::int64_t` (`collision_queries`);
seeds are `std::uint32_t`, which is what the generator takes. `PlanSeed` mixes a base
seed and a plan number in 64 bits so that neighbouring base seeds share no plan.

**A clock for budgets.** `std::chrono::steady_clock` never jumps when the system's
time is corrected, so it is the clock for measuring how long something took
(`SecondsSince`). It is used for two things: reporting, which is harmless, and
stopping a search, the one place where the machine's speed can change the outcome.

**A lambda that ends the function.** `auto fail = [&](RrtStatus status) ->
std::optional<JointPath> { ... }` near the top of `PlanRrtConnect` fills in the report
and returns "nothing", so every early exit is the one line `return fail(...)`. The `->`
part states the return type; left to itself the compiler would take the type of
`std::nullopt`, which is not an optional path.

**A guard that puts things back.** `PlanMove` raises the checker's margins while it
plans. `MarginGuard` saves the old margins in its constructor and restores them in
its destructor, so they are put back on every one of the function's many returns:
stop 1's ownership idea, used for a setting instead of memory. `= delete` on its copy
operations stops a second guard restoring twice, and holding it in a `std::optional`
lets `PlanMove` end it early with `guard.reset()`. Python's `with`; Go's `defer`.

## Lab

Write your prediction down before each run. Runs 1 and 2 take seconds; the planner
there is working on a point in a square, 10 units on a side.

1. **One plan, drawn.** `bazel run --config=opt //:print_rrt -- circle 1 2>/dev/null`
   prints the trees and the path as text (the format is at the top of
   `src/print_rrt.cc`). Count the `raw` and the `short` rows, and sketch both paths on
   paper from their coordinates. Then run `enclosed_goal 1` with `>/dev/null` in
   place of `2>/dev/null`, which hides the rows and keeps the summary line. What did
   the planner conclude, and what could it not conclude?
2. **Many plans.** `bazel run --config=opt //:print_rrt -- --stats`. Which worlds are
   not solved every time, and why those? How much longer than the shortest path is a
   raw path, and a shortened one? Then `--sweep`: predict whether a smaller step helps
   or hurts in `gap_0.2`, and in `arm_like`. Is there one best setting?
3. **The budget, in the cell.**
   `bazel run --config=opt //:print_plan -- --pretty --seeds 5`. Add up the 45 mm on
   the first line from the options in `src/motion_planner.h`. Now predict the planning
   margin, and roughly the number of queries, for `--resolution 15`, and run it. Last:
   the `self_mm` column says two parts of the arm are always about 81 mm apart. Work
   out the resolution at which the planner must refuse to start, try one a little
   above it, and read the reason.
4. **Seeds and sides.** Add `--plan-seeds 1` to run 3's first command: what does
   planning four times buy, and what does it cost? Then `--other-way` and
   `--both-ways`: which side of the cell does the planner prefer when left to choose?
   Find in the header why the cycle does not let it.
5. **In the cycle.**
   `bazel run --config=opt //:run_cell -- --pretty --nominal --modules 1 --phases`,
   then again with `--carry planned`. Compare the carry and the cycle. In the carry's
   line, find how long planning took and what it ran alongside.

## Pitfalls

- **"No path" read as "impossible".** It means the limits ran out.
- **Tuning on one case.** Run 2's sweep: the step that is best in one world is worst
  in another. A setting is judged over many seeds and several problems, and in
  queries before wall time.
- **Planning to a margin and following with less care.** The 45 mm assumes the path is
  timed with the reported `max_deviation` and followed within the tracking allowance.
  Change one without the others and the sum no longer holds.
- **A goal chosen without thinking about joint 1 or joint 6.** The same tool pose with
  a joint a turn away is a different goal and a different path.
- **Shortening the whole of a path that must pass through a point.** `PlanThrough`
  shortens each leg separately for that reason.
- **Planning from the truth.** `PlanScene::world` is the simulator's state; the cycle
  passes estimates instead (stop 8).

## In the MoveIt cells

MoveIt plans through plug-ins. The usual one wraps OMPL, a library of sampling
planners, and the usual planner from it is RRTConnect: the algorithm you have just
read. What surrounds it differs.

- **A pipeline.** A request goes through adapters, then the planner, then more
  adapters: fixing a start state that is slightly out of bounds, adding timing
  (stop 10). Each is configuration, not code you see.
- **Other planners in the same socket.** Pilz for plain industrial moves (joint-space
  point to point, straight lines, arcs), with no search at all. CHOMP and STOMP, which
  start from a guess and push it out of collision by optimization. Newer work plans on
  a graphics card (cuRobo) or with vector instructions (VAMP), in milliseconds.
- **Defaults stand in for decisions.** The metric is plain joint distance unless
  somebody changes it, the step is a fraction of the space's size, and the check
  spacing is the fraction from stop 8. Nothing like the margin budget is built in:
  padding, check spacing, corner rounding and tracking error are four settings in
  four places.
- **Joints and seeds.** A joint the URDF calls `continuous` wraps round; a `revolute`
  joint with limits is an interval, as here. OMPL seeds its random numbers once per
  process, so a single plan cannot be replayed by itself.

`docs/ompl-integration.md` is this repository's own work on the question "is the
hand-written planner any good?". Its table "What makes a naive comparison unfair" is
worth keeping: every row is a way to fool yourself when comparing two planners.

For your team: which planner and which settings do the real cells use, how often does
planning fail, and what happens to the cycle when it does?

## Reading

- Steven LaValle, *Planning Algorithms* (2006), chapters 4 and 5, free at
  <https://lavalle.pl/planning/>: configuration space and sampling-based planning, by
  the inventor of the RRT. In chapter 5, the sections on distance, on collision
  detection and on rapidly exploring trees are enough.
- James Kuffner and Steven LaValle, "RRT-Connect: An Efficient Approach to
  Single-Query Path Planning" (ICRA 2000): short, with `Extend` and `Connect` by name.
- Ioan Sucan, Mark Moll and Lydia Kavraki, "The Open Motion Planning Library" (IEEE
  Robotics and Automation Magazine, 2012), and <https://ompl.kavrakilab.org>: what
  MoveIt plans with. Read the article; use the site as reference.
- Sertac Karaman and Emilio Frazzoli, "Sampling-based Algorithms for Optimal Motion
  Planning" (International Journal of Robotics Research, 2011): RRT*, the planner that
  keeps improving its path, and what "asymptotically optimal" in the OMPL report
  refers to. Read the introduction and look at the pictures.
- Wil Thomason, Zachary Kingston and Lydia Kavraki, "Motions in Microseconds via
  Vectorized Sampling-Based Planning" (ICRA 2024): the current direction. The same
  RRT-Connect, with the collision check rewritten so that a plan takes microseconds.

## Office hours

Be ready to explain, with the chapter closed:

- what a point in configuration space is, and why the obstacles there have no formula;
- one turn of RRT-Connect, with a sketch;
- three things "probabilistically complete" does not promise;
- why the planner runs four seeds, and what shortening cannot fix;
- where each of the 45 mm comes from.

Questions worth asking your teacher:

- The cell plans nearly the same carry thirty times a pallet. Why plan afresh each
  time?
- What would you have to hold equal to compare this planner fairly with OMPL's?
