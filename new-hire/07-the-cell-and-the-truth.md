# Stop 7: The cell and the truth

What the simulated cell is made of, what in it is a model of physics and what is a
stand-in, and the line the code draws between what the robot may know and what only the
scorer may know. Every number in the README rests on that line being kept. About a day.

## Read

1. `src/cell.h`, the comment at the top, then `Randomization`, `TrueScene` and the
   section comments inside `Cell` ("the suction", "the camera", "truth").
2. The model files' top comments, in this order: `models/common.xml`,
   `models/cell.xml`, `models/pallet.xml`, `models/conveyor.xml` (and the comment on
   its motor), `models/suction.xml`, `models/tool.xml`.
3. `src/suction.h`, the top comment, and `Suction::Grip` in `src/suction.cc` from
   "Bind the weld" to the end.
4. `src/conveyor.h`, the top comment, and `Conveyor::StartStep` in `src/conveyor.cc`.
5. `src/score.h`, and `MeasurePlacement` in `src/score.cc`: stop 2's algebra in three
   lines.
6. `src/cell.cc`: `Cell::Load`, `Cell::DrawScene` and `Cell::Reset`.
7. `README.md`, "Where the simulation is kinder than a real cell".

## The idea

**What a model file holds.** MuJoCo's XML describes the world as a tree. The words you
need:

- a *body* is a rigid thing; a *joint* says how it may move relative to its parent: a
  hinge turns, a slide slides, a free joint lets it go anywhere (six freedoms);
- a *geom* is a shape fixed to a body, used for contact and for drawing;
- a *site* is a marked point with a frame: `tool_tip` is one;
- an *actuator* is a motor on a joint, a *sensor* reads something out;
- an *equality constraint* ties two things together; a *keyframe* is a named state,
  such as `arm_home`.

The cell is built from part files that `models/cell.xml` includes. Run `print_cell` and
you can account for every number on its first line.

**Contacts.** Nothing in the file says the modules rest on each other. At every step
the engine finds the geoms that touch and pushes them apart with forces: contacts. In
MuJoCo a contact is slightly soft, like a very stiff spring that settles over a few
milliseconds, and friction under a steady sideways load creeps. Both shape this cell:
`models/pallet.xml` explains why the whole pallet is tilted (the stack stands on
pushing forces, not friction), and `Cell::kSettleTime` is the second that `Reset`
gives the stack to take up its own weight.

**Suction is a weld.** MuJoCo has no suction cup. `Suction::Grip` checks that the tool
tip is on a module's glass (within 10 mm, within 3 degrees of square, inside the
outline), then switches on one weld constraint between tool and module, holding the
pose the two have *at that instant*. Nothing is snapped square: if the tool arrived
2 mm off, the module rides 2 mm off, and that offset reaches the placement. That part
is honest. What the weld hides is everything a real vacuum does: cups that leak or
peel, a module that slips or sags, and the time the vacuum takes to build.

**The pallet is a mocap body.** Its pose is not in the model but in the state, so
`PlacePallet` can set it down somewhere new for each trial without reloading anything.
The modules are free bodies placed on it.

**The conveyor is a servo with an encoder.** A slide joint carries the tube, a position
servo drives it, and two sensors read its position and speed. Each step is an S-curve
(stop 3) to an *absolute* target: after n steps it aims for n times 1.12 m, wherever
the last stop really ended. A random stopping error is added to each target, so the
tube really does stop a little off; because the targets are absolute, the errors do
not add up. Control code reads the encoder (`SensorPosition`), not the step count.

**Reset and the seed.** `Cell::Reset(seed)` clears the state, puts the arm at home,
draws the scene (`DrawScene`), places the pallet, makes a fresh conveyor and lets
everything settle. The draws are taken in a fixed order from one generator, so the
same seed gives the same scene, and a new kind of randomization added later does not
change what old seeds produce. `Randomization` says how far each thing may be off:
the design's tolerance, which perception's crop is built from too.

**The truth line.** A simulator knows where everything is. A real cell knows its joint
angles, its encoder, its vacuum switch and its pictures. The code draws the line by
name:

- *Allowed*: `JointAngles`, `conveyor().Sensor...`, `TakeDepthImage`,
  `CalibratedCamera`, `VacuumHolds`, and what the script did itself.
- *Truth*: anything named `True...` or `true_...`, and any body's pose read straight
  out of `mjData`. For building scenes, for tests and for the scorer.

Nothing but convention enforces it. `data()` has to be writable, because control code
writes motor commands into it; the same pointer could read a module's pose. The rule is
kept by the names, by review, and by the fact that anyone can search for it.

**Scoring.** `MeasurePlacement` asks the simulator where the slot truly is and where
the module truly is, and expresses the second in the first's frame:
`slot_pose.inverse() * module_pose`. Along, across, height, yaw and tilt are read off
that one pose. The scorer sets no tolerance; it reports numbers.

**Kinder than a real cell.** The README lists it plainly: motor inertia and friction
are estimates, the weld never slips, the module is rigid, the arm's controller uses
the simulator's own model of the arm, and the camera's noise is assumed. Keep that
list in mind when a number looks very good.

## C++ you meet here

**A class that owns its parts.** `Cell` holds the model, the state, the arm's indices,
the conveyor and the suction as members. They live exactly as long as the `Cell` does
and are destroyed with it, last declared first: the state before the model it belongs
to. Two members are `std::optional`: a `Conveyor` cannot be made until the model is
loaded, so the member starts empty and `Load` fills it, and `Reset` replaces it with a
fresh one. `conveyor_->Moving(...)` and `*conveyor_` reach inside.

**Move-only types.** Because `Cell` holds a `unique_ptr` (stop 1), it cannot be
copied; the compiler refuses. It can be *moved*: its contents handed to a new `Cell`,
the old one left empty. `Cell::Load` builds a local `Cell` and ends `return cell;`,
which moves it into the `std::optional<Cell>` the caller receives. `std::move(x)`
asks for a move where the compiler would otherwise copy, as in `inliers =
std::move(next)` in `src/plane_fit.cc`. Moving a `Cell` moves the pointer to the
model, not the model, which is why the `Conveyor`'s own plain pointer to the model
stays good. A `PickPlace` keeps a reference to its `Cell`, so that `Cell` must stay
where it is and outlive it; the comments on both constructors say so, and nothing else
checks.

**`const` and non-`const` pairs.** `Cell` has `mjData* data()` and `const mjData*
data() const`. Which one you get depends on whether your `Cell` is `const`. The scorer
takes `const Cell&` and so can only read; the script takes `Cell&` and can write
commands. `suction()` exists only in the `const` form: gripping goes through
`Cell::Grip`, which keeps the bookkeeping.

**Overloading.** Two functions may share a name if their parameters differ.
`MeasurePlacement(cell, i, slot)` returns an error; `MeasurePlacement(cell, i)` looks
the slot up and returns an `optional`. The compiler picks by the arguments.

**Constants in headers.** `inline constexpr double kModuleLength = 2.3;` in
`src/pallet.h` is a compile-time constant that any file including the header may use,
with one definition program-wide. It is the same thing as `static constexpr` inside a
class (stop 4) and `constexpr` in a `.cc` file's unnamed namespace (stop 5), at a
wider scope.

**`.at(i)` against `[i]`.** `state_.at(i)` checks the index and stops the program if
it is out of range. `state_[i]` does not check; out of range, it reads or writes
whatever memory is there. This code uses `.at` where the index comes from a caller
(`Cell::module_state`) and `[]` where it has just been checked.

## Lab

Write your prediction down before each run.

1. **Count the cell.** `bazel run --config=opt //:print_cell`. Before you look: from
   what is in the cell, how many joints, how many degrees of freedom and how many
   actuators should the first line report? Then: with nothing moving, what touches
   what?
2. **Settling.** `bazel run --config=opt //:print_pallet`. Find the table headed
   "settling". When has the stack finished moving, and what does that say about
   `Cell::kSettleTime`?
3. **A tolerance is a contract.** The pallet may be set down 30 mm off. Give it more:
   `bazel run --config=opt //:run_cell -- --pretty --seed 4 --modules 1 --xy 180`,
   then `--xy 240`, then `--xy 300`. For each, note the pallet's true offset, the
   result, "across", and the finder's "long" error. Is any of the three worse than a
   failure? If so, why did nothing refuse? (Think about stop 6's crop and size check.)
   Then try `--seed 2 --yaw 9` and `--yaw 12`.
4. **Who knows where the tube is.** In a worktree (`../sppp-wt-lab`, removed when you
   are done), with `bazel run --config=opt //:run_cell -- --pretty --seed 3 --modules 3`
   each time:
   - Run it unchanged and note "along" for the three modules.
   - In `src/conveyor.h` make `ConveyorSettings::stop_error_stddev` ten times larger.
     Predict "along", run, and explain what you see.
   - Put that back. In `src/cell.h` make `Randomization::conveyor_stop_error` ten
     times larger. Predict, run.
   - Keep that edit. In `PickPlace::PlanPlace` replace
     `conveyor.SensorPosition(*cell_.data())` with `conveyor.nominal_position()`.
     Predict "along" for each module, run, and look at the third module's "height"
     and "tilt" as well.
5. **Search for the line.** `grep -c "True[A-Z]" src/pick_place.cc src/score.cc`.
   Are the counts what the rule says they should be?

## Pitfalls

- **Changing a default that something else overrides.** Lab 4 shows one. Before you
  conclude a setting has no effect, find who writes it last.
- **Reading a pose out of `mjData` straight after `mj_step`.** It is from before the
  step's last integration. `mj_kinematics` brings it up to date (`Suction::Grip`).
- **Believing a stand-in.** The weld never slips and the module never bends. A result
  that depends on either is a result about the simulation.
- **Inputs outside the designed tolerance.** Between "works" and "refuses" there can
  be a band of "wrong without knowing". Lab 3 finds one.
- **Reaching for the truth "just to debug".** That is what the scorer, the tests and
  the `print_*` programs are for. Control code that reads it stops being evidence.

## In the MoveIt cells

- **Simulators.** A ROS 2 cell is usually simulated in Gazebo, sometimes in Isaac Sim
  or in MuJoCo, with the robot's description in URDF. The same code then drives the
  simulated robot and the real one through ros2_control.
- **No physics at all.** The most common "simulation" in MoveIt work is ros2_control's
  mock hardware, which reports every commanded joint position straight back as the
  measured one. Plans always execute perfectly in it: no lag, no contact, no sag. Know
  which kind you are looking at before you trust a result.
- **What a real cell adds.** A vacuum generator with a switch or sensor per group of
  cups, and a blow-off to release. A PLC that runs the conveyor, the interlocks and
  the safety devices, talking to the robot over a fieldbus. Fences, light curtains and
  a safety controller that can stop everything, governed by standards such as
  ISO 10218. None of that is in this repository.
- **No reset and no truth.** A real cell cannot be put back to a seed, and nothing can
  print where the module truly landed. Placement is measured with separate equipment
  or judged by whether the next process accepts the part.

Ask your team how the real cells know a module is held, who commands the conveyor and
how its position is known, and which simulator, if any, their tests run in.

## Reading

- MuJoCo documentation, "Modeling",
  <https://mujoco.readthedocs.io/en/stable/modeling.html>: the first sections, for
  bodies, joints, geoms and defaults. Keep the XML reference open beside the models.
- Emanuel Todorov, Tom Erez and Yuval Tassa, "MuJoCo: A physics engine for model-based
  control", IROS 2012. Why contacts here are soft, from the people who chose it. Read
  the introduction and the section on contact.
- ros2_control documentation, <https://control.ros.org>: the page on mock components,
  for the "simulation" with no physics.
- Josh Tobin and others, "Domain Randomization for Transferring Deep Neural Networks
  from Simulation to the Real World", IROS 2017. Written for learning, but the idea is
  this cell's `Randomization`: vary what you are unsure of and require success across
  the range.

## Office hours

Be ready to explain, with the chapter closed:

- what a body, a joint, a geom and a site are, with an example of each from this cell;
- what the weld captures honestly and what it hides;
- why the conveyor's stopping errors neither add up nor reach the placement;
- which of `Cell`'s functions the script may call, and how you would check;
- why a `Cell` cannot be copied, and what `Cell::Load` returns.

Questions worth asking your teacher:

- What would have to change in this code if the suction could slip?
- The truth line is kept by naming. Could the compiler be made to keep it?
