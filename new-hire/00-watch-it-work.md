# Stop 0: Watch it work

Before any code: what the cell does, how to run it, and how to read what it prints. Every
later stop explains one part of the table you will produce here. About half a day,
most of it the first build.

## Set up

1. Install Bazelisk and make it answer to the name `bazel`
   (<https://github.com/bazelbuild/bazelisk>). It reads `.bazelversion` and fetches that
   Bazel. Check with `bazel --version`.
2. Give every checkout on the machine one disk cache, unless `~/.bazelrc` already names
   one: `echo "build --disk_cache=$HOME/.cache/bazel-disk" >> ~/.bazelrc`.
3. Build everything once: `bazel build --config=opt //...`. The first build compiles the
   physics engine and the planning library OMPL from source and takes about ten
   minutes. Later builds take seconds.

Always pass `--config=opt`. Without it the programs are built unoptimized and run 5 to
70 times slower.

## Read

1. `README.md`, all of it, and the recordings it shows (`docs/cycle.gif`, `docs/pick.gif`,
   `docs/blind_vs_camera.gif`). Do not try to follow every term yet.
2. `AGENTS.md`: "Rules", "Frames and units" and "Map". The rules are for people too.
3. The comment at the top of `src/run_cell.cc`, which documents every flag and column.

## The idea

**The task.** Solar modules (2.3 m by 1.1 m, 30 kg) arrive standing on edge on a pallet.
A six-jointed industrial arm with a suction tool takes the front one, carries it to the
other side of itself, and lays it flat on rails that are already mounted on a steel tube
(the torque tube of a solar tracker). A conveyor then moves the tube along by one
module's width, and the arm goes back for the next. One module from pallet to rails is a
**cycle**; how long it takes is the **cycle time**; how far the module lands from where
it should is the **placement error**. Those two numbers are what the cell is judged by.

**The layout.** Seen from above, with the arm's base at the origin of the world frame:

```
                          +Y
                           ^
       tube                |    x  check camera, on the floor, looking up
        | |                |       (0.13, 1.49)
        | |                |
   -----|-|----------------A--------------[ pallet ]-----> +X
        | |             arm base          front glass at x = 1.95
        | |      (pallet camera 4 m       modules lean back 10 degrees
        | |        above the base)
     x = -2.0              |
                           o  camera post (0, -3.8)
```

Lengths are metres and +Z is up. The arm swings round the +Y side, away from the post.

**Who does what.** The code is a chain from sensing to motion:

```
 pallet camera -> module_finder -> where the front module is
                                          |
                                     pick_place   (decides every step of the cycle)
                                     |    |    \
                           ik <------+    |     +-----> motion_planner
     (tool pose to joint angles)          |             (rrt + collision + path_timing:
                                          |              a path for the carry that hits
        check camera -> held_module_finder               nothing, and its timing)
        (where the module sits on the tool)
                                          |
                                     arm_follower -> motor commands -> the simulated cell
```

`Cell` (`src/cell.h`) owns the simulated world. `PickPlace` (`src/pick_place.h`) is the
script that runs a cycle step by step. `run_cell` runs cycles and prints the scores.

**Simulated time and wall time.** The cell is simulated: a physics engine (MuJoCo) works
out where everything is 500 times per simulated second. "Simulated 33.4 s in 4.9 s of
wall time" means 33.4 seconds passed for the robot while 4.9 passed for you. Cycle times
are always simulated seconds.

**Sensor data and truth.** A simulator knows exactly where everything is. A real cell
does not: it knows its joint angles, its conveyor's encoder and what its cameras show.
The rule that makes this repository's results mean something is that the control code
reads only what a real cell could sense. Things named `True...` are for the code that
sets up a scene and the code that scores it. In the table, the pallet's offset and
every error column are truth, printed for you; the script never saw them.

**Seeds.** Each trial sets the pallet down a little off where the drawing says, by up to
30 mm and 3 degrees, drawn from a random seed. The same seed always gives the same
trial, down to the last digit. `--nominal` puts the pallet exactly on the drawing.

**The table.** `run_cell --pretty` prints one row per module. Its column groups:

| Columns | Meaning | Truth? |
|---|---|---|
| `seed mod` | the trial's seed and the module's number (0 is the front one) | |
| `dx mm, dy mm, yaw d` | how far off the pallet was set down | truth |
| `result slot` | placed or failed, and which pair of rails | |
| `along, across, height, yaw d, tilt d, rest` | placement error against the slot, and whether the module lies still | truth |
| `stack, turn d` | how far the pick disturbed the modules still on the pallet | truth |
| `cycle` | cycle time, simulated seconds | |
| `seen n, long, up, face d, rot d, ms` | the camera's error in finding the module, and its computing time | truth |
| `carry, plan, clr, ctorq` | the carry's time, the planner's computing time and clearance, peak motor torque as a fraction of the limit | |
| `reason` | why a cycle failed, in words | |

With `--phases` each row is followed by one line per step of the cycle.

## C++ you meet here

No language yet, only the build.

- **Compiled.** C++ source is translated to machine code before it runs. That is what
  the build does, and why there is a wait before the first run.
- **Targets.** `BUILD.bazel` lists what can be built. A `cc_library` is code other
  targets use (`:profile`), a `cc_binary` is a program (`:run_cell`), a `cc_test` is a
  program whose exit status says pass or fail (`:profile_test`). `//:run_cell` means
  "the target `run_cell` in the top directory".
- **Running.** `bazel run --config=opt //:run_cell -- --pretty` builds the target if
  anything it depends on changed, then starts it. Everything after the lone `--` goes
  to the program, not to Bazel.
- **Dependencies.** `MODULE.bazel` names the outside libraries: MuJoCo (physics), Eigen
  (vectors and matrices), OMPL (a second motion planner, stop 9, with the Boost
  libraries it needs), GLFW (the viewer's window), GoogleTest. Bazel downloads and
  compiles them; nothing is installed on the machine.

If you know `cargo`, `go build` or `npm run`, Bazel is that, for a language with no
standard build tool of its own.

## Lab

Write your prediction down before each run.

1. Two modules from a pallet exactly on the drawing:
   `bazel run --config=opt //:run_cell -- --pretty --nominal --modules 2`.
   Find the cycle time and the placement error of each. Why might the second cycle be
   quicker? Add `--phases` and find out.
2. One module from a pallet set down off the drawing:
   `bazel run --config=opt //:run_cell -- --pretty --seed 3 --modules 1 --phases`.
   How far off was the pallet? How far off did the module land? Which three steps take
   longest?
3. Predict what happens to the same pallet with the camera switched off, then add
   `--blind`. Which step fails, and by how much? What happened to the peak torque and to
   the stack?
4. Now `--nominal --modules 1 --blind`. Why does blind work here?
5. With a display: `bazel run --config=opt //:view_cell -- --seed 3`. Space pauses, `[`
   and `]` change the camera. Without one, watch `docs/blind_vs_camera.gif` again and
   match it to what runs 2 and 3 printed.

## Pitfalls

- Leaving out `--config=opt` and concluding the simulation is slow.
- Starting a second Bazel command in the same checkout while one runs. It waits for the
  first; it has not hung.
- Reading a truth column as something the robot knew. The robot never saw `dx mm`.
- Comparing a cycle time with a wall-clock time.
- Expecting different numbers on a second run. Same seed, same numbers; if yours differ
  from a colleague's, something in the code or the settings differs.

## In the MoveIt cells

A production cell on ROS 2 is not one program but many, started together by a launch
file: a driver that talks to the robot's controller, MoveIt's `move_group` for planning,
a perception node per camera, and an application that sequences the cycle. They exchange
messages over topics, services and actions, and run on the wall clock. You watch them in
RViz and record them with rosbag.

Two differences matter from the first day:

- There is no truth. Nothing on a real cell can print the `dx mm` column. Placement
  error has to be measured some other way, and how it is measured decides what you can
  trust.
- Runs do not repeat exactly. Many processes on a wall clock give slightly different
  timing each time, so "it failed once" is a harder report to act on than a seed.

What is the same is the cycle: look, pick, carry, place, index. Ask your team how the
real cells measure cycle time and placement error, and write it in your notebook.

## Reading

- `docs/gate.md` in this repository: how a change is judged. Skim it now; stop 12
  returns to it.
- MuJoCo's overview, <https://mujoco.readthedocs.io/en/stable/overview.html>: the first
  two sections, for what a physics simulator does.
- ROS 2 concepts, <https://docs.ros.org/en/rolling/Concepts/Basic.html>: nodes, topics,
  services and actions, a page each.
- MoveIt concepts, <https://moveit.picknik.ai/main/doc/concepts/concepts.html>: the
  diagram of `move_group` is enough for now.

## Office hours

Be ready to explain, with the chapter closed:

- the four parts of the task, and which piece of code does each;
- why the blind cycle fails on an offset pallet and succeeds on a nominal one;
- one column of the table that is truth and one that is sensor data, and why the
  difference matters;
- where the time goes in a cycle.

Questions worth asking your teacher:

- Why is the camera 4 m up rather than in front of the pallet?
- What would a real cell use in place of the "at rest" column?
