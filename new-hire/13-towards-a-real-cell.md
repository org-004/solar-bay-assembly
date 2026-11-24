# Stop 13: Towards a real cell

This repository is the in-house planner, and it has only ever driven a simulation. The
cells in production run on ROS 2 and MoveIt. This stop puts the two side by side: what
each piece here is called there, where the simulation is kinder than a factory floor,
and what this planner would still need before it could move a real arm. About a day,
much of it talking to your team.

## Read

1. `README.md`: "Where the simulation is kinder than a real cell" and "Not done".
2. `docs/ompl-integration.md`: "Summary and recommendation", then "What the comparison
   needs" down to the end of "A proposed design", then "Decisions for the owner". Skip
   the build detail in between unless it interests you.
3. Your own notebook: "Questions for my team" and the answers you have collected.
4. The "In the MoveIt cells" section of each earlier stop, once more, in order.

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
9. *Is it better than what runs today?* On the same task, against the MoveIt cell, in
   cycle time and placement. And against OMPL's planners inside this repository, which
   `docs/ompl-integration.md` prepares.

**Comparing fairly.** The OMPL report's table "What makes a naive comparison unfair" is
worth knowing by heart, because every "planner A beats planner B" claim you will ever
read is open to the same faults: different spacing of the collision checks along an
edge, different step lengths, one path shortened and the other not, one seed against
the best of four, wall time on different compile flags. Its rule: make the problem
identical first, count collision queries before milliseconds, and compare distributions
over many plans, never one plan against one plan.

## C++ you meet here

**Bringing in a library.** If you come from `pip`, `npm` or `cargo`, this is the part of
C++ that will surprise you most. There is no one package manager. A C++ library is
headers plus compiled code, and both must agree with your own code about the compiler,
its flags, the language version and the version of every library you both use.

`docs/ompl-integration.md` is a worked example: eight ways of bringing one library in,
four of them built and measured. Three things in it are general:

- *One of each library in a program.* OMPL's headers include Eigen, and so does this
  code. If the two saw different Eigens, the program would hold two definitions of the
  same types, which C++ forbids without diagnosing (the "one definition rule"). The
  report's row "One Eigen in the binary" is about that.
- *The same flags.* A library compiled at another optimisation level is still correct
  but no longer comparable: timings mean something else.
- *Pinned.* `MODULE.bazel` names an exact version of every dependency, and the files
  fetched outside the registry carry a hash. The same source builds the same program
  next year.

## Lab

Make the simulation less kind, one flag at a time, and find where the cell stops
coping. For each run, predict first: will it place, will it be in tolerance, and will
the script know if it is not?

1. A baseline, five pallets with the planned carry:
   `bazel run --config=opt //:run_cell -- --pretty --trials 5 --carry planned`.
   Find the planner's computing time and the line that says whether it fits inside
   the motion it runs beside.
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

- `docs/ompl-integration.md` in this repository. The best example here of how a
  technical decision is prepared: options, measurements, what was not verified.
- Steven LaValle, *Planning Algorithms*, chapter 5, "Sampling-Based Motion Planning".
  Free at <https://lavalle.pl/planning/>. The standard account of what stop 9 built.
- Ioan Sucan, Mark Moll and Lydia Kavraki, "The Open Motion Planning Library", IEEE
  Robotics and Automation Magazine, 2012, and <https://ompl.kavrakilab.org>: what the
  library is and how its pieces map onto the adapter the report describes.
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
- what has to be equal before two planners' timings can be compared.

Questions worth asking your teacher:

- If the planner became a MoveIt plugin tomorrow, which parts of this repository would
  come with it and which would be left behind?
- What would a test like the gate look like for a cell with no truth?
