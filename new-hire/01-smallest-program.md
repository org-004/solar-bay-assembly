# Stop 1: The smallest program

Fifty lines of C++ that load a robot and simulate it for one second. This stop is where
the language starts: how C++ source is organised, who owns memory, and what the physics
engine's two main objects are. About a day.

## Read

1. `src/step_arm.cc`, top to bottom. It is the whole program.
2. `src/mj_handles.h`, the 49 lines it includes.
3. In `BUILD.bazel`, the three targets `mj_handles`, `step_arm` and `step_test`, with
   their comments.
4. `tests/step_test.cc`: the same load and step, with assertions.

## The idea

**Model and state.** MuJoCo keeps a simulation in two objects, and everything in this
repository is built on that split.

- `mjModel` is the description: which bodies exist, how they are joined, their masses,
  the motors, the time step. It is loaded from an XML file and does not change while
  simulating.
- `mjData` is the state: the time, every joint's position and speed, the motors'
  commands, and everything worked out from those. It changes at every step.

One model can have several states. Stop 5 uses that: a second, scratch `mjData` to ask
"where would the tool be at these joint angles" without disturbing the simulation.

**A step.** `mj_step(model, data)` moves the state forward by the model's time step,
here 0.002 s. Inside, the engine works out the forces (gravity, motors, contacts), turns
them into accelerations, and moves the positions and speeds on by that small time. Five
hundred steps make one simulated second.

**Flat arrays.** MuJoCo is a C library. It has no objects for "joint 3"; it has long
arrays, and tables that say where in them each thing lives. `data->qpos` holds every
position in the model, `data->qvel` every speed, `data->ctrl` every motor command. A
joint's angle is `data->qpos[model->jnt_qposadr[joint]]`. Names are looked up once and
kept as indices; `src/arm.h` (stop 4) does that for the arm.

**What the run shows.** `step_arm` loads a UR10e, a smaller arm than the cell's, used
here only to check the toolchain. It sends no commands, so the motors hold the angles
they start at. After a second the shoulder has sagged by 0.024 rad: gravity pulls, the
motor pushes back in proportion to how far the joint is from its command, and they
balance a little way off. That small sag is the first appearance of a recurring
character, the difference between where a joint was told to be and where it is.

**Radians.** Angles in the code are radians: 0.024 rad is 1.4 degrees, and 1 rad is
about 57.3 degrees. A milliradian (mrad) at 1 m is 1 mm.

## C++ you meet here

**Source files and headers.** A `.cc` file is compiled by itself into machine code.
What several `.cc` files need to share (types, function declarations) goes in a `.h`
file, and `#include "src/mj_handles.h"` pastes that file's text in at that line. That
is all `#include` does; there is no import system. The `#ifndef SRC_MJ_HANDLES_H_ ...
#endif` wrapper stops the text being pasted twice. Angle brackets (`<cstdio>`) are for
the standard library and outside libraries.

**Namespaces.** `namespace cell { ... }` puts names inside `cell`, so the handle type is
`cell::ModelPtr`. It plays the part of a package or module name.

**Values, pointers and references.** In Python or Java a variable of an object type is
always a reference to an object somewhere else. In C++ you choose:

- `mjModel m` would be the object itself, stored right there, copied when assigned.
- `mjModel* p` is a pointer: the address of an object, which may be null. Reach through
  it with `p->opt` or `*p`.
- `mjModel& r` is a reference: another name for an object that must exist. Use it like
  the object itself.

`const` in front means "I will not change it through this name". This repository uses
the three to say something: a function that only reads takes `const T&`, and a function
that writes takes `T*`. So `MakeData(const mjModel& model)` promises to leave the model
alone, and at the call you write `MakeData(*model)`, where `*` turns the handle into
the object it points at.

**Ownership.** C++ has no garbage collector. `mj_loadXML` returns memory that somebody
must hand back to `mj_deleteModel`, exactly once, on every path out of the function,
including the early `return 1`. `mj_handles.h` solves that with `std::unique_ptr`: an
object that holds the pointer and calls the delete function when it goes out of scope.
`ModelDeleter` tells it which function to call. This pattern, tying a resource's
release to the end of a scope, is the central habit of C++ (its name is RAII). Nothing
in `src/` calls `mj_deleteModel` by hand.

**One owner.** A `unique_ptr` cannot be copied, because then two owners would both
delete. It can only be moved, handing ownership on. `model.get()` lends the raw pointer
to a C function without giving ownership away.

**Small things in passing.** `using ModelPtr = ...` gives a long type a short name.
`inline` on a function in a header lets every `.cc` that includes it carry a copy
without a clash. `if (!model)` asks the handle whether it is empty. `std::string*
error = nullptr` is an optional output: pass the address of a string to receive the
message, or nothing.

## Lab

Runs 2 to 4 edit code, so make a throwaway worktree first (see `new-hire/AGENTS.md`,
"Labs"), and remove it afterwards.

1. `bazel run --config=opt //:step_arm`. How many steps, and which joints moved? Why
   those?
2. Change `duration` to 5.0. Predict two things before running: how many steps it will
   report, and whether the sag will be five times larger, the same, or something else.
3. Put the duration back. After the line that makes `data`, add
   `cell::ModelPtr copy = model;` and build with `bazel build --config=opt //:step_arm`.
   Read the first error. Which words in it say what you did wrong?
4. Remove that line, and take the `*` out of `cell::MakeData(*model)`. Build. Read the
   error and the note under it. What did the compiler want, and what did it get?
5. Back in the main checkout: `bazel test --config=opt //:step_test`. Then read one
   `TEST_F` and say what it would take to make it fail.

## Pitfalls

- Reading `Type* x` as "x is a Type". It is an address, and it may be null.
- Forgetting that `=` copies. `Eigen::VectorXd a = b;` makes a second vector. Python's
  `a = b` does not.
- Comparing a sum of floating-point steps with a round number. Run 2 shows what
  happens.
- Believing the first line of a compiler error and not reading the notes under it. The
  note usually names the declaration that did not match.
- Editing in the main checkout "just to try". Use the worktree.

## In the MoveIt cells

- The description of the robot is a URDF file, with an SRDF beside it for MoveIt's
  extras, in place of MuJoCo's XML. `models/arm.xml` was converted from the vendor's
  URDF by hand.
- The state arrives as messages. A driver publishes the joint angles on a topic,
  usually `/joint_states`, and anything that needs them subscribes.
- Nobody calls a step. The real arm moves in real time, and a control loop (in
  ros2_control, the controller manager) reads, computes and writes at a fixed rate
  whether your code is ready or not.
- C++ there looks different. ROS 2 nodes are shared between owners with
  `std::shared_ptr`, work is done in callbacks, and errors are often exceptions. This
  repository has single owners, plain function calls and no exceptions. Neither is
  wrong; know which you are reading.

## Reading

- MuJoCo, "Simulation" in the programming guide,
  <https://mujoco.readthedocs.io/en/stable/programming/simulation.html>: the first
  part, on `mjModel`, `mjData` and the simulation loop.
- Bjarne Stroustrup, *A Tour of C++* (3rd edition): the first three chapters now (the
  basics, user-defined types, modularity), and the chapters on classes and on essential
  operations (copy, move, resource management) when the ownership section above has
  sunk in. Written for programmers who already know another language.
- C++ Core Guidelines, the section on resource management,
  <https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-resource>: rules R.1,
  R.3, R.20 and R.21 are the ones this code lives by.
- Google C++ Style Guide, <https://google.github.io/styleguide/cppguide.html>: the
  naming section. This repository follows it (`kConstant`, `member_`, `FunctionName`).

## Office hours

Be ready to explain, with the chapter closed:

- what is in `mjModel` and what is in `mjData`, with one example of each;
- what one call to `mj_step` does;
- the difference between `mjModel`, `mjModel*` and `const mjModel&`;
- who frees the model in `step_arm.cc`, and when, on each way out of `main`.

Questions worth asking your teacher:

- Why does this repository pass things it will change as pointers rather than
  references?
- What would go wrong, and when would you notice, if `ModelPtr` were a plain pointer?
