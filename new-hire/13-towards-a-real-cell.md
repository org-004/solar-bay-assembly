# Stop 13: Towards a real cell

This repository is the in-house planner, and it has only ever driven a simulation. The
cells in production run on ROS 2 and MoveIt. This stop puts the two side by side: what
each piece here is called there, where the simulation is kinder than a factory floor,
and what this planner would still need before it could move a real arm. About a day,
much of it talking to your team.

## Read

1. `README.md`: "Where the simulation is kinder than a real cell" and "The planner
   against OMPL".
2. `docs/ompl-comparison.md`: "Verdict" again, now with "What is left unequal" in
   full and "If OMPL became the default". It is the one place where this planner has
   been measured against the library the production cells plan with.
3. `docs/ompl-integration.md`: "Summary and recommendation" only, for how a C++ library
   gets into this build at all. Then the OMPL part of `MODULE.bazel`, and a glance at
   `bazel/ompl.BUILD.bazel`.
4. `src/ompl_search.cc`: the three small classes near the top (`SeededSampler`,
   `SeededBox`, `EdgeValidator`), `MakeProblem` and `MakePlanner`. And at the bottom
   of `src/ompl_search.h`, the class `OmplSpace`.
5. Your own notebook: "Questions for my team" and the answers you have collected.
6. The "In the MoveIt cells" section of each earlier stop, once more, in order.

## The idea

**The map.** The same problem is solved in both places, cut into pieces differently.

| Here | In a ROS 2 and MoveIt cell | What changes for you |
|---|---|---|
| `models/*.xml` | URDF and SRDF | the robot's description is shared by every node |
| `Cell` and MuJoCo | the real arm behind ros2_control, or a simulator | no step to call; the clock runs by itself |
| poses passed as `Isometry3d` (stop 2) | the tf2 tree | lookups by name and time, not arguments |
| `profile`, `joint_move`, `path_timing` (3, 10) | time parameterization, then the controller's own interpolation | two layers shape the motion, and you own one |
| `arm_follower` (4) | a trajectory controller and the vendor's servo | no feedforward of yours; payload data set on the controller |
| `ik` (5) | a kinematics plugin | chosen in configuration, not called directly |
| `depth_camera`, `module_finder` (6) | a camera driver, a perception node, a calibration transform | real noise, real glass, a calibration somebody measured |
| `collision` (8) | the planning scene and its collision library | a shared world model that can be stale |
| `rrt`, `motion_planner` (9) | OMPL through a planning pipeline | many planners behind one interface, tuned by parameters |
| `ompl_search` (9) | the same OMPL, on its defaults | here it is handed this planner's box, metric and edge check; there it decides them itself |
| `pick_place` (11) | a task layer: stages, a behaviour tree, or a PLC | sequencing across processes, with cancel and recovery |
| `trial_runner`, the gate (12) | launch tests, recorded bags, acceptance tests | no truth, and runs that do not repeat exactly |

Two differences are not in any one row. Here everything is one process stepping one
clock, so a run is a function of its seed. There, a dozen processes share a wall clock.
And here every part can be read in an afternoon; there, most of the code that moves the
arm is in libraries and in the vendor's controller.

**Where the simulation is kind.** The README lists what it admits to. Reading the code
adds more. Each is a place where a number in this repository is better than a real
cell's would be.

- *The arm is exactly its drawing.* Joint angles through the model give the true tool
  pose. A real arm's links, gearboxes and mounting differ a little from the drawing,
  so it returns to a taught pose far better than it reaches a computed one.
- *The follower's model is the simulator's own* (README). Its feedforward is exact. A
  real arm's inertia and friction have to be identified, or left to the vendor.
- *Calibration is a number, not a procedure.* A camera's error here is a flag
  (`--calibration`). Nothing in the repository finds a camera's pose; on a real cell
  somebody measures it, and it drifts.
- *The grip is a weld* (`src/suction.h`). It never slips, leaks or half holds. The code
  has a branch for a slipped module; the simulation cannot exercise it.
- *The module is rigid.* A real 2.3 m module held near its middle bends.
- *The pictures' noise is an assumption* (`src/depth_camera.h` says so), and glass is
  hard for real depth cameras.
- *Computing takes no time.* The carry is planned "while the arm lifts", but the
  simulation waits for the planner (`src/pick_place.h`). `run_cell` reports whether the
  plan would have fitted; a real cell has to make it fit.
- *Commands arrive at once.* `ArmFollower` writes a setpoint every 2 ms with no delay.
  A real controller interface has a cycle time, a delay and its own smoothing.
- *Stopping is free.* `ArmFollower::Hold` on a moving arm "stops dead". A real arm's
  stops are the controller's and the safety system's business.
- *Nobody is in the cell.* There is no safety system, no operator and no recovery: a
  failed cycle ends the trial.
- *The tolerance is assumed.* 3 mm and 0.15 degrees are the owner's assumption, not a
  tracker maker's figure (`docs/gate.md`).

**What the planner would need.** Not a roadmap, which is the owner's to draw; a list of
questions that have to be answered before this code moves steel.

1. *How does it reach the arm?* As a library inside a ROS 2 node that sends a
   trajectory to the existing controller, as a planner plugged into MoveIt, or by
   streaming setpoints to the vendor's controller itself? Each answer decides who
   interpolates, who checks limits and who stops the arm.
2. *What must finish in bounded time?* Planning has a 10 s budget and usually takes a
   third of a second. Which parts need a guarantee, and what happens on the day it
   takes two seconds?
3. *Where do the calibrations come from?* Pallet camera to arm, check camera to arm,
   tool tip to flange. How are they measured, how often, and how is drift noticed? The
   check after pickup removes the pallet camera's error and not its own.
4. *What do real pictures look like?* Noise from a datasheet or a measurement, glass,
   frames, sunlight.
5. *What can the grip do wrong?* Per-cup vacuum sensing, a module that peels or
   shifts, and what the cycle does then.
6. *How flexible is the module, and does it matter at 3 mm?*
7. *How does a cycle recover?* Put the module back, set it aside, call a person?
8. *How is placement measured without truth?*
9. *Is it better than what runs today?* Half of this has an answer. Against OMPL's
   planners inside this repository, on four moves of this simulated cell:
   "hand-written better", by a rule fixed beforehand (`docs/ompl-comparison.md`).
   Whether OMPL should become the default search all the same is left to the owner on
   purpose. Against a MoveIt cell on the same task, in cycle time and placement,
   nothing has been measured.

**Comparing fairly.** `docs/ompl-comparison.md` is worth studying as a method, whatever
you make of its result, because every "planner A beats planner B" claim you will ever
read is open to the faults it guards against.

- *The rule came first.* What would count as "OMPL better", "hand-written better" or
  "no material difference" was written down before any number existed. One candidate,
  BIT*, missed by 1.053 against a limit of 1.05. A rule written afterwards could have
  been made to say either.
- *The problem was made identical.* "How the two were made equal" lists what that
  took: the box, the metric, the edge check, the seeds, the limits, exact solutions
  only. Each has a test.
- *What could not be made equal is listed*, with the way each difference pushes. One
  of them decided the verdict: the hand-written search tries the straight line first.
- *Counts before milliseconds.* Collision queries are the same on every machine; its
  times are "good to a factor, not to a few per cent".
- *Many plans, and honesty about repeats.* Medians and worst cases over fifty seeds,
  and the rows that do not repeat from run to run are marked as samples.

`docs/ompl-integration.md`, "What makes a naive comparison unfair", is the same list
written before the work; the comparison is that list carried out.

## C++ you meet here

**Bringing in a library.** If you come from `pip`, `npm` or `cargo`, this is the part of
C++ that will surprise you most. There is no one package manager. A C++ library is
headers plus compiled code, and both must agree with your own code about the compiler,
its flags, the language version and the version of every library you both use.

`docs/ompl-integration.md` weighed eight ways of bringing OMPL in and built four of
them. The one it recommended is now in the build, and you can read what it took:

- In `MODULE.bazel`, an `http_archive`: an address, a hash of the file, and a build
  file. OMPL has no Bazel files of its own, so `bazel/ompl.BUILD.bazel` is this
  repository's build file for somebody else's source.
- Under it, seventeen Boost libraries named one by one, because OMPL includes their
  headers. They bring others with them: 82 modules in all, by the comparison's count.
- The price, from `AGENTS.md` and the comparison: a fresh build takes about ten
  minutes where it took six, and every program that plans is larger.

Three things in the report are general:

- *One of each library in a program.* OMPL's headers include Eigen, and so does this
  code. If the two saw different Eigens, the program would hold two definitions of the
  same types, which C++ forbids without diagnosing (the "one definition rule"). The
  report's row "One Eigen in the binary" is about that.
- *The same flags.* A library compiled at another optimisation level is still correct
  but no longer comparable: timings mean something else.
- *Pinned.* `MODULE.bazel` names an exact version of every dependency, and the files
  fetched outside the registry carry a hash. The same source builds the same program
  next year.

**Where two dialects meet.** `src/ompl_search.cc` is the one file in `src/` that looks
like most of the C++ you will meet outside this repository, because it has to speak
OMPL's language.

- *Inheritance.* OMPL is extended by deriving from its classes. `class EdgeValidator :
  public ob::MotionValidator` is a motion validator of this file's own; `override` on
  `checkMotion` says "this replaces the base class's function", and the compiler
  checks that there is one to replace. OMPL then calls it without knowing its type.
  That is the ordinary method call of Java or Python, which C++ makes you ask for.
- *Shared ownership.* OMPL passes its objects round as `std::shared_ptr` (its `...Ptr`
  types): several owners, and the object is freed when the last lets go.
  `std::make_shared<SeededBox>(...)` makes one. Compare stop 1's single owner.
- *A short name for a long namespace.* `namespace ob = ompl::base;`.
- *Hiding all of it.* `OmplSpace` in the header has one data member,
  `std::unique_ptr<Impl> impl_`, and `Impl` is only declared there. Its contents are
  in the `.cc`. So the header includes nothing of OMPL's, and the files that include
  it never see OMPL's or Boost's headers; remember from stop 1 that an include pastes
  text. The idiom is called "pimpl". Its price: the destructor has to be defined in
  the `.cc`, where `Impl` is known, and copying is switched off with `= delete`.

ROS 2 code is written in this second dialect throughout.

## Lab

Make the simulation less kind, one flag at a time, and find where the cell stops
coping. For each run, predict first: will it place, will it be in tolerance, and will
the script know if it is not?

1. A baseline, five pallets with the planned carry:
   `bazel run --config=opt //:run_cell -- --pretty --trials 5 --carry planned`.
   Find the planner's computing time and the line that says whether it fits inside
   the motion it runs beside. Then once more with `--planner ompl`: what changes in
   the summary, and what does not?
2. A pallet set down twice as far off as the cell was designed for: add
   `--xy 60 --yaw 6` (drop `--carry planned` if you like).
3. The pallet camera's calibration three times worse than the gate assumes, with the
   check on: `--trials 5 --check --calibration 3,0.15 --check-calibration 1,0.05`.
   Which step fails? Could the check after pickup have helped?
4. The check camera's calibration three times worse instead:
   `--trials 5 --check --calibration 1,0.05 --check-calibration 3,0.15`. How many are
   placed? How many are within 3 mm? What did the script report about them?
5. On paper: take one fault from a production cell that your team has told you about
   and say which row of the map it lives in.

## Pitfalls

- **"It works in simulation."** It works on the simulation's assumptions. Say which.
- **Trusting a result the cell did not question.** Run 4 is the dangerous kind of
  failure: nothing refused, nothing stopped.
- **Fixing the wrong layer.** In a ROS 2 cell the same symptom (the arm is 4 mm off)
  can come from the calibration, the tf tree, the planner's scene or the controller.
  The map is a checklist of where to look.
- **Comparing two planners on two problems.** See "Comparing fairly".
- **Reading this list as a plan.** Which gap matters first depends on the real cells,
  which this repository cannot see.

## In the MoveIt cells

This whole stop is that section. What it cannot contain is what is particular to your
company's cells: which arm and controller, which interface to it, which cameras, how
calibration is done, what the cycle time and tolerance really are, what fails most
often. Every one of those is a question for your team, and your notebook is where the
answers go. When you have them, the "Keeping this material true" section of
`new-hire/README.md` says where they belong.

## Reading

- `docs/ompl-integration.md` and `docs/ompl-comparison.md` in this repository. The
  best examples here of how a technical decision is prepared and how a comparison is
  reported: the options, a rule fixed beforehand, the measurements, and what was not
  verified.
- Steven LaValle, *Planning Algorithms*, chapter 5, "Sampling-Based Motion Planning".
  Free at <https://lavalle.pl/planning/>. The standard account of what stop 9 built.
- Ioan Sucan, Mark Moll and Lydia Kavraki, "The Open Motion Planning Library", IEEE
  Robotics and Automation Magazine, 2012, and <https://ompl.kavrakilab.org>: what the
  library is, and how its pieces (state space, motion validator, planner) map onto
  `src/ompl_search.cc`.
- Balakumar Sundaralingam and others, "cuRobo: Parallelized Collision-Free
  Robot Motion Generation" (2023), <https://curobo.org>: planning as optimisation on a
  GPU, the current direction for cycle time.
- Wil Thomason, Zachary Kingston and Lydia Kavraki, "Motions in Microseconds via
  Vectorized Sampling-Based Planning", ICRA 2024: the same planners as stop 9, made
  very fast by checking many configurations at once on a CPU.
- ISO 10218 (industrial robots, safety requirements): read a summary, not the
  standard, so that the words mean something when your team uses them.

## Office hours

Be ready to explain, with the chapter closed:

- for three rows of the map, what the piece is called on each side and what changes;
- three ways the simulation is kinder than a real cell, each with the place in the
  code or the README that says so;
- why run 4 is worse than run 3;
- what has to be equal before two planners' timings can be compared, and why the rule
  for the verdict was written first;
- what `src/ompl_search.cc` does that no other file in `src/` does, and how the
  header keeps it from spreading.

Questions worth asking your teacher:

- If the planner became a MoveIt plugin tomorrow, which parts of this repository would
  come with it and which would be left behind?
- What would a test like the gate look like for a cell with no truth?
