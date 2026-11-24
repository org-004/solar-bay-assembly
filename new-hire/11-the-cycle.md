# Stop 11: The cycle

Everything from stops 2 to 10 is called from one place: the script that takes a module
from the pallet to the rails, step by step. This stop is about how those pieces are put
in order, how the script knows when something has gone wrong, and what it does then.
When a cell misbehaves, this is the layer you will read first. About a day.

## Read

1. The comment at the top of `src/pick_place.h`, in two passes. First: the opening,
   the list headed "One cycle, in the order the steps run", and "Use, once per physics
   step" near the end. Second, after the lab's first run: "The planned carry", "The
   check after pickup", "The plausibility test on the measured grip" and "A refusal by
   the finder".
2. In `PickPlaceOptions`, the comments on `grip_gap`, `lift_back`, `turn_outboard`,
   `blocked_tracking_error` and `touch_tolerance`. Each number comes with the
   measurement that chose it.
3. `enum class PickPlaceStep`, then in `src/pick_place.cc`, in this order:
   `PickPlace::Update`, `Advance`, `Finish`, `Next`, `BeginStep`, `BeginMove`,
   `MoveFinished` and `Fail`. Those eight are the machine; the rest of the file is
   what the steps do.
4. `PickPlace::PlanCarry`, `CarryFinished` and `CarryScene`. Skim `RunCheck` for its
   shape: one picture, then a chain of "if this, fail with that reason".
5. The loop in `RunTrial` (`src/trial_runner.cc`) that calls `script.Update()`,
   `cell.Step()` and `check.Observe(...)`, and the comment at the top of
   `src/cycle_check.h`, the observer that watches a cycle from the truth.

## The idea

**A script of steps.** A cycle is a fixed list of steps, and each leans on an earlier
stop.

| Steps | What happens | Stop |
|---|---|---|
| look, look ahead | a depth picture becomes the front module's pose | 6 |
| to standoff, flatten, swing, turn, return | joint moves between chosen poses | 3, 4, 5 |
| approach, touch glass, lift, pull back, clear, lower, set down, retreat | the tool tip along straight lines | 2, 5, 10 |
| grip, release | the suction switched on and off | 7 |
| carry | one planned move in place of flatten, swing and turn | 8, 9, 10 |
| the check, inside carry | the grip measured from below, the place corrected | 2, 6 |
| wait conveyor, conveyor | the tube's encoder read, the tube stepped on | 7 |

**One call per physics step.** The script never waits. `Update` is called once before
every `Cell::Step`, 500 times a simulated second, and returns at once. Each call asks
"has the current step finished?", and if so begins the next. Whatever a step is waiting
for (a move to end, a dwell to pass, the tube to stop) is remembered in member
variables between calls. A game loop's tick has the same shape, for the same reason:
the world advances whether the script is ready or not, and a script that looped until
the arm arrived would stop the world it was waiting on. `BeginStep` says what a step
does when it starts; `Advance` says when it is over.

**Numbers with reasons.** The options are not tuned by feel. Three to read closely,
each from its own comment:

- `grip_gap`, 2 mm. The tool stops short of the glass and lets the suction close the
  gap, because the modules stand free: cups that touch first push the stack.
- `lift_back`, 4 mm. When the module's weight comes onto the tool the arm sags a
  millimetre or two, and on glass leaning back 10 degrees the sag presses the module
  into whatever is behind it. Behind the last module that is the pallet's back rest,
  which does not give way. Backing off while lifting takes the press away.
- `turn_outboard`, 0.35 m. The module is turned across the tube beyond the slot,
  because over the slot its edge sweeps through the upper arm.

**Straight lines.** A joint move curves the tool's path, which is fine in open space
and not at the glass or the rails. For a straight line the script lists tool poses a
centimetre apart, solves each for joint angles starting from the one before
(`SolveLine`, on stop 5's `SolveIkPath`), and hands the list to the follower as one
blended motion (stop 10).

**Knowing it has hit something.** The cell has no force sensor. The script has the
joint angles and its own plan. Following a plan, a joint stays within a fraction of a
milliradian of it; a tool pressing on glass bends the arm by tens of milliradians
within a fraction of a second. So a joint more than `blocked_tracking_error` (3 mrad)
behind its plan means the arm is held up. In two steps that is expected: in touch glass
the cups may meet the glass a little early, and in set down the module may meet the
rails. Anywhere else it fails the cycle. `Fail` then tells the follower to hold the
measured angles, so an arm that has stopped against something stops pushing.

**Thinking while moving.** Planning the carry takes a few tenths of a second of
computing. The script starts it the moment the grip holds, when the carry's start (the
end of clear) is already known, so on a real cell the plan is ready before the arm has
lifted the module off the stack. The simulation waits for the planner, and the carry's
line says whether the computing "fits" inside those moves. The look-ahead picture is
the same idea: the next pick's picture is taken while the tube moves.

The planner's obstacles come from what the script knows, never from the simulator.
`CarryScene` puts the stack where the camera saw it, each module the script laid down
in its slot at the encoder's reading (`KnownModule`), and the held module at the
design grip. If that picture is wrong, the plan is confidently wrong.

**The check, and believing it.** With `--check` the carry has two legs, with a short
stop between them over the check camera. The picture gives the grip as measured (stop
2's `GripOffsetFromCheck`). The second leg, planned for the design grip, is checked
again with the module where it really hangs and replanned only if that fails; then the
set-down is corrected. But a measurement can be wrong too. The cups hold the glass flat
and close to the tool whatever else went wrong, so a measured grip with the module
tilted on the cups, or far from the tool tip, cannot be true, and the script refuses
it. That test has one blind spot, which the header names and the lab shows.

**Failing well.** Nothing is placed uncorrected, and there is no fallback: if the
planner finds no path the cycle stops rather than use the scripted carry, and if the
check is refused twice the cycle stops with the module on the tool. Every failure
carries its step and a reason in words with the numbers, and the vacuum switch tells
"nothing was picked" from "the camera is at fault". A fallback that quietly works is
how a cell comes to place modules 4 mm off for a week before anyone notices.

## C++ you meet here

**A large class.** `PickPlace` is the biggest class in the repository, and its shape is
the usual one: a small public part (a constructor, `Start`, `Update`, `Done`, read-only
accessors), a long private part of helpers, and the state as member variables at the
bottom, each with a trailing underscore. The header shows the private members too,
which looks odd from Java or Python: the compiler needs every member to know how big
the object is. The bodies are in the `.cc`, each written `PickPlace::Name`.

**A state machine as an enum and a switch.** The steps are an `enum class`; the current
one is `step_`. In `Advance`:

- several `case` labels share one body, as the eleven plain moves do;
- a case that declares a variable is wrapped in braces, because a `case` label is only
  a place to jump to and starts no scope of its own;
- `return` means "not finished, ask again next tick"; `break` means "finished", and
  control falls out to `Finish(result)`;
- there is no `default:`. Every step is listed, so the compiler warns when someone adds
  a step and forgets this switch;
- `Next` gets the following step by turning the enum into an integer and adding one,
  so the order of the enum's lines is the order of the cycle.

**Optional members for "not known yet".** `estimate_`, `release_tip_` and
`held_in_tip_` are `std::optional`: empty until the look step, the place planning and
the check have produced them. `Start` empties them with `.reset()`, so nothing measured
on one module leaks into the next cycle. A plain member with "zero means unset" would
compile as well, and fail quietly.

**A reference as a member.** `Cell& cell_` is not a copy of the cell and does not keep
it alive. The constructor's comment states the contract: the cell "must outlive this
object". With a garbage collector that sentence is unnecessary; here it is all that
stands between you and a crash somewhere else, later. The lambdas `RunTrial` hands over
as hooks capture local variables by reference (`[&]`) under the same contract.

**Stored callbacks.** `PlacedHook` is a `std::function` (stop 9) kept as a member.
`set_placed_hook` takes it by value and `std::move`s it into place, and the script
calls it, if one was set, when the module has been let go. The hook's body belongs to
the caller, which is why it may read the truth: the script only says "now".

**Small things in passing.** `mutable` on `scratch_` marks a workspace that functions
declared `const` may still write. `check_pose` is initialized by a lambda written and
called on the spot (`[] { ... }()`), a way to run statements where only an expression
is allowed. Messages are built with `std::snprintf`, C's `printf` writing into a
fixed `char` buffer and never past `sizeof(text)`.

## Lab

Write each prediction down first. Runs 2 and 4 edit code: use a throwaway worktree
(`new-hire/AGENTS.md`, "Labs"), and undo each edit before the next.

1. **Three carries, one pallet.**
   `bazel run --config=opt //:run_cell -- --pretty --seed 3 --modules 1 --phases`,
   then the same with `--carry planned`, then with `--check`. Which step lines change?
   What happens to the carry's time and the cycle's? In the planned carry's line, find
   how long the planning took, what it ran beside, and what the path came closest to.
2. **The last module.**
   `bazel run --config=opt //:run_cell -- --pretty --nominal --first 29 --modules 1 --verbose`.
   The cycle check's report comes out on stderr. In its table "peak torque / limit",
   which joint works hardest, and in which step? Then set `lift_back` to 0.0 in
   `src/pick_place.h`, predict which row changes, and run it again. Also: what did the
   look ahead say after the last module, and was the cycle right to carry on?
3. **A check camera that is turned.**
   `bazel run --config=opt //:run_cell -- --pretty --seed 1 --trials 6 --check --check-calibration 0,0.5`.
   The check camera is half a degree off, in a different direction for each seed.
   Predict: all refused, none, or some? For any that was placed, look at "yaw d". For
   the others, read the reason and the step.
4. **Pressing on.** Set `blocked_tracking_error` to 1.0 in `src/pick_place.h`, which
   switches the early warning off, and run stop 0's blind pallet:
   `bazel run --config=opt //:run_cell -- --pretty --seed 3 --modules 1 --phases --blind`.
   Compare touch glass's time, the peak torque and the stack's disturbance with the
   unedited run.
5. **The README's table.** Five pallets each, `--pretty --trials 5`, with: `--carry
   planned`; `--check`; `--carry planned --calibration 1,0.05`; `--check --calibration
   1,0.05`; and that last plus `--check-calibration 1,0.05`. Read each "placement,
   worst / rms" line. Which error does the check remove, and which does it leave?

With a display, `view_cell` takes the same flags and shows the step in the corner.

## Pitfalls

- **Waiting inside `Update`.** A loop that waits for the arm stops the world. State
  goes in members; the function returns.
- **A fallback that hides a fault.** "If the planner fails, use the scripted carry"
  makes the failure leave the table and stay in the cell.
- **Adding a step.** The enum's order is the cycle's order, and `BeginStep`, `Advance`,
  `Skipped`, `StepName` and the cycle check's allowed contacts all know the steps.
- **A measurement outliving its moment.** The measured grip belongs to one module, the
  look-ahead picture to one scene. Both are dropped when that moment has passed.
- **Changing an option without its measurement.** Each default records what it was
  measured with. Change one, and rerun what measured it.

## In the MoveIt cells

MoveIt plans a motion when asked; it does not decide what to ask for. The cycle lives
in an application above it, in one of three forms. MoveIt Task Constructor describes a
pick and place as a sequence of stages (move to a pose, move along a direction, attach
the object, connect two states with a planned path) and plans the whole sequence before
anything moves. A behaviour tree does what `Advance` does, ticked at a fixed rate. Or a
PLC sequences the whole cell, and the robot is one of its devices.

What differs when you debug one:

- A motion is an action: a goal is sent, feedback comes back while it runs, a result
  arrives at the end, and it can be cancelled. A step finishes in a callback, not in a
  function you can read from top to bottom.
- Detecting a collision is usually the robot controller's own work, and reaches you
  as a fault.
- Planning while moving is possible on the same condition as here: the plan must start
  where the arm will be. MoveIt refuses to run a trajectory that does not start at the
  arm's present state.
- Recovery involves a person. A real cell that stops with a module on the tool needs
  somebody to decide what happens to it. This script only refuses to start a pick
  while the vacuum still holds.

For your notebook: what sequences the cycle in the production cells, and what happens,
step by step, when one stops while holding a module?

## Reading

- Michele Colledanchise and Petter Ögren, *Behavior Trees in Robotics and AI: An
  Introduction* (2018; arXiv 1709.00084). Chapters 1 and 2: why state machines grow
  hard to change, and what a tick is.
- Michael Görner and others, "MoveIt! Task Constructor for Task-Level Motion Planning"
  (ICRA 2019), with the pick-and-place tutorial in MoveIt's documentation. Read the
  paper for what a stage is.
- Nikolaus Correll and others, "Analysis and Observations from the First Amazon Picking
  Challenge" (IEEE Transactions on Automation Science and Engineering, 2018; arXiv
  1601.05484). What went wrong, and why, across the 26 teams' pick systems.
- Caelan Garrett and others, "Integrated Task and Motion Planning" (Annual Review of
  Control, Robotics, and Autonomous Systems, 2021; arXiv 2010.01083). Where research
  is on deciding the steps and the motions together. The introduction is enough.

## Office hours

Be ready to explain, with the chapter closed:

- what `Update` does in one call, and why it must return at once;
- how the script knows the arm has met something, with no force sensor;
- why the carry is planned at the start of lift, and what "fits" means;
- what the plausibility test checks, and the one error it cannot see;
- why a refused check stops the cycle rather than placing uncorrected.

Questions worth asking your teacher:

- Why is the carry's start the end of clear and not the end of pull back?
- What would it take for the cycle to recover from a refused check by itself, and
  should it?
