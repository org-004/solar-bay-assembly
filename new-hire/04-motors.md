# Stop 4: Motors

Stop 3 produced a plan: where each joint should be at every instant. This stop is about
what happens between that plan and the joint actually being there. The gap between the
two, the tracking error, is how the cell knows it has arrived, how it notices that it
has run into something, and one of the reasons a module lands where it does. About a
day.

## Read

1. `models/arm.xml`: the comment above `<actuator>`, including "Two things a
   controller has to know", and the comment on armature and damping at joint 1.
2. `src/arm.h`, all of it, and `FindArm` in `src/arm.cc`.
3. `src/arm_follower.h`: the comment at the top, `MoveOptions`, `FollowerSettings`,
   and the public half of the class.
4. `src/arm_follower.cc`: `ArmFollower::Update` (thirty lines, the heart of it), then
   `Done`, `Planned`, `TrackingError` and `MoveThrough`.
5. `tests/arm_follower_test.cc`: `LagCompensationMatters`,
   `ModelFeedforwardRemovesMostTrackingErrorAndTheSag` and
   `FullSpeedCanSaturateJoint2WithAModuleAtFullReach`.
6. `PickPlace::MoveFinished` in `src/pick_place.cc`.

## The idea

**The motor is a spring and a brake.** Each of the arm's six motors is a position
servo. You give it a wanted angle (its `ctrl`), and it produces a torque

```
torque = kp * (wanted angle - angle)  -  kv * speed
```

never more than the motor's limit. The first term is a spring pulling the joint toward
the wanted angle. The second is a brake that stops it swinging past. The spring is
stiff: joint 2's `kp` is ten million newton metres per radian, so an error of one
milliradian already asks for more torque than the motor has.

**Never command a jump.** Set the wanted angle a radian away and the spring demands a
thousand times the limit. The motor gives its limit, the joint flies, and it overshoots
by a long way (the comment in `arm.xml` has the numbers). So the wanted angle is moved
smoothly, every 2 ms, along the plan from stop 3. That is the job of `ArmFollower`:
call `Update` once per step and it writes the six commands for that instant.

**Lag.** Suppose the wanted angle moves at a steady speed and the joint keeps pace. No
torque is needed to hold a steady speed, so spring and brake must cancel:
`kp * error = kv * speed`. The joint therefore trails by `error = (kv / kp) * speed`,
which is a fixed *time* behind: `kv / kp` seconds, 0.035 s for joint 1 (`Arm::lag`,
which counts the joint's own friction as part of the brake). At joint 1's full 2 rad/s
that is 0.07 rad, which at a tool 2.7 m out is about 19 cm. The cure costs one line:
command the planned angle plus `lag * planned speed`, and the trailing cancels
(`FollowerSettings::lag_compensation`).

**Feedforward.** What is left: a servo makes torque only out of error, and moving an
arm needs torque. Accelerating its mass needs some; holding it up against gravity
needs a lot. Without help the joint must run a little behind, and sag a little at
rest, to earn that torque. The help is to work out in advance the torque the planned
motion needs,

```
torque needed = M * planned acceleration  +  gravity and velocity terms
```

and add `torque needed / kp` to the command, so the spring is pre-stretched by exactly
the right amount (`FollowerSettings::model_feedforward`). `M` is the arm's mass matrix:
how much torque at each joint one unit of acceleration at each joint costs, in the
pose the arm is in now. MuJoCo supplies both pieces (`mj_mulM`, `qfrc_bias`). Working
torques out from a model of the arm is called inverse dynamics.

**The model is too good.** The follower's model of the arm is the simulator's own, so
here it is exact. On a real arm it never is, and the README lists this among the ways
the simulation is kinder. One error is modelled honestly: the held module is a
separate body welded to the tool, the follower's model does not include it, and it
sags the arm by up to half a milliradian (the comment on `model_feedforward`).

**Saturation.** When the torque wanted exceeds the motor's limit, the motor gives the
limit and the joint falls behind the plan: the plan is no longer being followed. The
test with a 30 kg module at full reach finds joint 2 at its limit at full speed, which
is why moves run at 0.9 by default (`MoveOptions::kDefaultSpeed`): a tenth of the
speed buys back about a tenth of the torque, kept in hand for what the model gets
wrong.

**Planned and measured.** The follower keeps two ideas apart: where the arm *should*
be (the plan, a function of simulation time) and where it *is* (the joint angles).
`TrackingError` is the difference. A new move starts from the end of the old plan, not
from the measured angles, so the command never jumps. `Done` needs the plan to have
ended *and* every joint to be within a milliradian of the goal and nearly still.

**The motors as a sense of touch.** Following a plan, the arm stays within about a
tenth of a milliradian. If any joint falls 3 mrad behind
(`PickPlaceOptions::blocked_tracking_error`), something is holding it. That is how
`PickPlace::MoveFinished` knew, in stop 0's blind run, that the cups had met the glass
early: "joint 3 3.9 mrad off".

## C++ you meet here

**`std::array` and a constant that belongs to a type.** `Arm` holds
`std::array<int, kJoints> qpos{}`: exactly six integers stored in the struct itself.
A `std::vector` could grow and lives on the heap; an array's size is part of its type.
`static constexpr int kJoints = 6;` is a constant fixed at compile time that belongs
to `Arm` as a whole, written `Arm::kJoints` from outside.

**Indices into flat arrays.** `FindArm` looks each joint and motor up by name, once,
and stores where its numbers live. After that the code is index arithmetic:
`model.jnt_range[2 * joint + 1]` is a joint's upper limit because MuJoCo stores two
numbers per joint, one after the other. Nothing checks these indices. Get one wrong
and you read a neighbour's number, with no error.

**Constructors.** `ArmFollower(const Arm& arm, const FollowerSettings& settings)` ends
with `: arm_(arm), settings_(settings) {}`. The part after the colon is the member
initializer list: it builds each member directly from the arguments. `explicit` in the
header stops the compiler from quietly turning an `Arm` into an `ArmFollower` where
one was expected.

**Options with defaults.** Functions take a struct of settings with a default:
`const MoveOptions& options = {}`. Call `MoveTo(goal, data)` and you get every
default. To change one, name it: `follower.MoveTo(goal, data, {.speed = 1.0})`, as the
tests do. The `{.name = value}` form is a designated initializer, the nearest C++ has
to Python's keyword arguments.

**One of two types, and the first lambda.** A leg of the plan is either a `JointMove`
or a `Trajectory` (stop 10). `std::variant<JointMove, Trajectory>` holds exactly one
of them and knows which. To use it:

```
std::visit([t](const auto& m) { return m.At(t); }, move);
```

The first argument is a lambda, a function written in place. `[t]` lists what it
carries in from outside (a copy of `t`), `(const auto& m)` is its parameter, and
`auto` there means "whatever type it is called with". `std::visit` calls it with
whichever the variant holds, and it works for both because both have an `At`. In Java
you would give the two classes a common interface; here they share nothing but a
function name, and the compiler checks it.

**No allocation in the loop.** `acceleration_` and `torque_` are members only so that
`Update`, which runs 500 times a second, does not ask the heap for memory each time.
On a real controller a late answer is a wrong answer, and memory allocation is one of
the things that can be late.

## Lab

Runs 2 to 4 edit a default: use a throwaway worktree (`new-hire/AGENTS.md`, "Labs"),
and restore each edit before the next.

1. **What each term is worth.**
   `bazel test --config=opt //:arm_follower_test --test_output=all` prints what it
   measures. Find "lag, s", the two "tracking ... lag compensation" lines, the two
   "tracking ... model feedforward" lines and "joint 2 torque/limit". Before you read
   the line without lag compensation, predict joint 1's figure: in that move joint 1
   peaks at 0.61 of its 2.04 rad/s.
2. **Lag compensation off.** In `FollowerSettings` set `lag_compensation = false`.
   Predict what the cycle does, then
   `bazel run --config=opt //:run_cell -- --pretty --nominal --modules 1 --phases`.
   Which step fails, and how soon? What does the message say stopped it? Did anything
   touch the arm?
3. **Feedforward off.** Set `model_feedforward = false` instead. Does the module still
   get placed? Which placement column moved, and by how much? Now explain why that
   column, using stop 2. (Hint: where was the arm when the sag mattered?) Then add
   `--check` and read the carry's line.
4. **Full speed.** Set `MoveOptions::kDefaultSpeed` to 1.0 and
   `PickPlaceOptions::speed_carrying` to 1.0. Predict the `swing` time from stop 3,
   then run. What did the cycle time and the peak torque do? The cycle did not
   saturate. So why is the default 0.9?

## Pitfalls

- **Calling a control fault a collision.** Run 2 reports "blocked" when nothing was
  touched. A message says what the script concluded, not what happened.
- **Starting a move from where the arm is.** The measured angles differ from the plan
  by a little; starting there makes the command jump. Start from the plan's end.
- **Forgetting the payload.** A model without the module is wrong by 30 kg at the end
  of a 2.7 m lever.
- **Tuning against a perfect model.** Feedforward that is exact in simulation hides
  how much the feedback has to do on a real arm.
- **Commands near a joint's limit.** MuJoCo clamps a command to the joint's range, so
  the lead terms are cut off there and the joint trails more (the comment on `Update`).
- **Reusing a follower after a reset.** Its plan is in simulation time; when the clock
  goes back to zero the plan means nothing. Make a new one.
- **An unchecked index.** `data->qpos[i]` with the wrong `i` is somebody else's joint.

## In the MoveIt cells

- **You do not own the servo.** The robot's own controller closes the position loop,
  at a rate of its own, with the maker's model of the arm and the maker's feedforward.
  Nothing like `ArmFollower::Update` is yours to edit.
- **You send a trajectory.** MoveIt hands a list of timed joint positions to a
  trajectory controller in ros2_control, which interpolates between them and passes
  setpoints to the robot's driver at a fixed rate. It also watches tolerances: stray
  too far from the trajectory on the way, or fail to settle at the end, and it aborts.
  That is `blocked_tracking_error` and `Done` under other names.
- **The payload has to be declared.** The maker's model needs the mass, centre of mass
  and inertia of the tool and of whatever it holds. With the wrong load data the arm
  tracks worse and its collision detection misfires. It is the same problem as the
  held module here, with a setting in place of a sag.
- **Time is not free.** Commands take milliseconds to reach the robot and the state
  takes milliseconds to come back. Here both take none.

A question for your team: when the gripper picks a module and when it lets go, who
tells the robot's controller that its load has changed?

## Reading

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 11, "Robot Control", sections
  11.1 to 11.4: feedback, feedforward and the computed-torque idea that
  `model_feedforward` is a form of. <http://modernrobotics.org>.
- Karl Åström and Richard Murray, *Feedback Systems: An Introduction for Scientists
  and Engineers*, the chapter on PID control. Free at <https://fbswiki.org>. The
  spring-and-brake servo here is its P and D; read it for what an I term would add and
  what it would cost.
- MuJoCo, "Computation", the section on the actuation model,
  <https://mujoco.readthedocs.io/en/stable/computation/index.html>: what `gainprm` and
  `biasprm` in `FindArm` mean.
- ros2_control, the joint trajectory controller,
  <https://control.ros.org/rolling/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html>:
  what it accepts, how it interpolates and what its tolerances do.

## Office hours

Be ready to explain, with the chapter closed:

- why a joint following a moving command trails it by a fixed time, and what the
  cure is;
- what model feedforward adds, and one thing its model leaves out;
- the difference between the plan ending and the arm being `Done`;
- how the cell tells that the arm has met something, without a touch sensor.

Questions worth asking your teacher:

- Why is there no integral term in the servo?
- Run 2 failed with "blocked". How would you tell that apart from a real collision
  on a real cell?
