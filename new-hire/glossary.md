# Glossary

The robotics, cell and C++ terms as this path uses them, in alphabetical order. Each
entry names the stop that introduces the term, and any later stops that take it up
again. The definitions are short on purpose; the chapter has the explanation.

## Notation

- **"B in A"** (stop 2): the pose of frame B seen from frame A; it converts B's
  coordinates to A's.

## A

- **acceptance test (FAT, SAT)** (stop 12): the agreed run a cell must pass before it
  ships (factory) and after it is installed (site).
- **action (ROS 2)** (stop 11): a long-running request with a goal, feedback while it
  runs, a result, and cancellation.
- **actuator** (stop 7): a motor acting on a joint; here every one is a position servo.
- **allowed collision matrix (ACM)** (stop 8): MoveIt's table of link pairs that are never
  checked.
- **allowed contact** (stop 8): a pair of bodies whose contact does not count for one
  move, because the move is meant to touch.
- **approximate positioning** (stop 10): a robot controller's own rounding of programmed
  points (KUKA's term).
- **armature** (stop 4): the inertia of a motor's rotor as seen through its gearbox.
- **attached collision object** (stop 8): MoveIt's name for an object fixed to a link that
  moves with it, such as a held part.

## B

- **backward pass / forward pass** (stop 10): the two sweeps that find the fastest allowed
  speed at each boundary: back from the end for braking in time, forward from the start
  for what can be reached.
- **baseline** (stop 14): the unchanged system run on the same seeds, which every result
  is a difference from.
- **behaviour tree** (stop 11): a tree of tasks ticked at a fixed rate, each node
  answering success, failure or running.
- **blend** (stop 10): the rounded piece of path that replaces a corner.
- **blind** (stop 0): picking from the drawing's poses, with no camera.
- **blocked (move)** (stop 11): a move that cannot reach its goal because the arm or its
  load has met something.
- **body** (stop 7): a rigid thing in a MuJoCo model; bodies form a tree under the world.
- **branch (configuration)** (stop 5): one of several sets of joint angles for the same
  tool pose: elbow up or down, wrist flipped or not, shoulder front or back.
- **broad, mid and narrow phase** (stop 8): the three stages of collision detection, from
  cheap box tests to exact distances.

## C

- **calibration** (stop 2): where a sensor is believed to be relative to the arm.
- **calibration (camera)** (stop 6): where the control code believes the camera is (its
  pose in the robot's frame); an error in it moves every point rigidly and cannot be seen
  in one picture.
- **cell** (stop 0): one robot workstation: the arm, its tool, what it works on, its
  sensors and its conveyor.
- **check after pickup** (stop 11): measuring where the module sits on the tool from a
  second camera, and correcting the place for it.
- **CI (continuous integration)** (stop 12): a service that builds and tests every change
  on a clean machine.
- **clearance** (stop 8): the smallest measured distance between the arm (with what it
  holds) and anything else.
- **collision check** (stop 8): the question "at these joint angles, is any counted pair
  of shapes closer than the margin?".
- **configuration** (stop 9): one set of joint angles: one pose of the whole arm and what
  it holds.
- **configuration space** (stop 9): the space of all configurations; six-dimensional for
  this arm.
- **contact** (stop 7): the forces the engine applies where two geoms touch; in MuJoCo
  slightly soft, with a settling time of a few milliseconds.
- **control tick** (stop 4): one pass of read, compute, write; here every 2 ms simulation
  step.
- **convex hull** (stop 8): the smallest shape with no hollows that contains a mesh; what
  MuJoCo collides for a mesh geom.
- **corner allowance** (stop 10): the share of the planning margin set aside for blends
  leaving the checked polyline.
- **crease** (stop 6): an edge where two surfaces meet without a step in depth, only a
  bend; harder to locate than a step.
- **crop** (stop 6): keeping only the points inside a box where the design says the object
  can be.
- **cycle** (stops 0, 11): one module from pallet to rails; its duration is the cycle
  time.
- **cycle check** (stop 12): the observer that watches every physics step for forbidden
  contacts, close approaches and motor torque.

## D

- **damped least squares** (stop 5): the IK step that closes the gap while paying a
  penalty for large joint motion.
- **data race** (stop 12): unsynchronized access to one variable from two threads, at
  least one writing.
- **degree of freedom** (stops 1, 7): one independent way a mechanism can move; this arm
  has six.
- **depth and range** (stop 6): depth is distance along the camera's viewing axis; range
  is distance along the pixel's own ray, longer by 1 / cos(angle off axis).
- **depth image** (stop 6): a picture whose pixels hold distances instead of colours; here
  640 by 480 `float`s in metres, NaN where there is no reading.
- **depth noise** (stop 6): the scatter of a depth camera's readings; here modelled as
  growing with the square of the depth.
- **deterministic** (stop 12): giving the same output for the same input, here to the last
  bit.
- **dwell** (stop 11): a planned pause with the arm holding still, for the vacuum to build
  or a picture to be taken.

## E

- **edge resolution** (stop 9): the largest gap between checked points along a straight
  joint-space segment.
- **encoder** (stop 7): the device that reads a drive's actual position; what
  `Conveyor::SensorPosition` stands for.
- **equality constraint, weld** (stop 7): a constraint tying two bodies together; a weld
  holds one at a fixed pose relative to the other. The suction grip here is one weld
  switched on and off.
- **Euler angles** (stop 2): an orientation as three turns in sequence; what a pendant
  shows, absent from `src/`.

## F

- **factory** (stop 3): a function that builds and returns an object in place of a public
  constructor.
- **failure record** (stop 12): the small text file a failed trial leaves: its settings,
  its first bad cycle and the command that reruns it.
- **fallback** (stop 11): a second way of doing something, used when the first fails; this
  script has none on purpose.
- **feedforward** (stop 4): adding to the command what a model says the motion will need,
  so feedback has little left to correct.
- **fieldbus** (stop 7): the industrial network between a PLC, the robot controller and
  I/O devices.
- **fixture** (stop 12): in GoogleTest, a class that sets up what several tests share.
- **flange** (stop 2): the arm's last link, where the tool is bolted on; 0.30 m behind the
  tool tip here.
- **flying pixel** (stop 6): a reading at a depth edge that lands between the two
  surfaces, on neither.
- **focal length (in pixels)** (stop 6): the pinhole model's scale; one pixel covers depth
  / focal metres (about 6 mm at 4 m for this camera's 659 pixels).
- **forbidden contact** (stop 12): a touch between the arm, tool or held module and
  something the cycle is not meant to touch at that step.
- **forward kinematics** (stop 5): from joint angles to the tool's pose; always one
  answer.
- **frame** (stop 2): an origin and three axes at right angles to measure positions along.
- **free** (stop 8): a configuration at which every counted pair is at least the margin
  apart.
- **free space** (stop 9): the configurations at which nothing is in collision.

## G

- **gain (kp, kv)** (stop 4): how hard the servo pushes per radian of error, and brakes
  per radian per second.
- **gate** (stop 12): the test that runs whole pallets and fails the build if any
  placement is out of tolerance.
- **geom** (stops 7, 8): a shape fixed to a body, used for contact and for drawing.
- **grip offset** (stop 2): how far the module sits on the tool from where the design
  says.

## H

- **hand-eye calibration** (stops 6, 13): the procedure that measures the camera-to-robot
  transform on a real cell, by showing the camera a target from many arm poses.
- **hook (callback)** (stop 11): a function the caller hands over to be called at a chosen
  moment.

## I

- **inlier / outlier** (stop 6): a point within the allowed distance of a fitted model, or
  not.
- **intrinsics** (stop 6): the numbers that describe the camera itself: focal length,
  principal point and, on a real lens, distortion.
- **inverse dynamics** (stop 4): working out the torques a given motion needs.
- **inverse kinematics (IK)** (stop 5): from a tool pose to joint angles; none, one or
  several answers.

## J

- **Jacobian** (stop 5): a table, valid at one pose, of how the tool tip moves and turns
  per small turn of each joint.
- **joint** (stop 7): how a body may move relative to its parent: a hinge turns, a slide
  slides, a free joint allows all six motions.
- **joint box** (stop 9): the range of each joint the planner may sample in.
- **joint move** (stop 3): a straight line in joint space, all joints starting and
  stopping together.
- **joint space** (stop 3): the space whose coordinates are the joint angles; one point is
  one configuration of the arm.

## K

- **keyframe** (stop 7): a named state stored in the model, such as `arm_home` and
  `arm_clear`.

## L

- **lag** (stop 4): the fixed time a joint trails a steadily moving command: brake gain
  over push gain.
- **lambda** (stop 6): a function written in place, with a capture list saying what it may
  use from its surroundings.
- **launch test, bag** (stop 12): in ROS 2, a test that starts real nodes, and a recording
  of messages that can be played back.
- **least squares** (stop 6): the fit with the smallest sum of squared distances to all
  the points given; best under small noise, dragged by strays.
- **lever arm** (stops 2, 8): the distance from an axis to a point; a small turn moves the
  point by angle times lever arm. The collision checker (stop 8) keeps, for each joint,
  the largest lever arm of any point of the arm and its load.
- **look ahead** (stop 11): taking the picture for the next pick at the end of this cycle,
  while the tube moves.

## M

- **margin** (stop 8): the distance the arm, tool and held module must keep from obstacles
  for a configuration to count as free.
- **margin budget** (stop 9): the sum that turns a required clearance into a planning
  margin: required margin, half the edge resolution, corner allowance, tracking allowance.
- **mass matrix** (stop 4): how much torque at each joint one unit of acceleration at each
  joint costs, in the current pose.
- **max_deviation** (stop 10): the most a blended path may stray from the polyline through
  its waypoints, in radians of joint-space distance.
- **metric** (stop 9): the rule for distance between two configurations; here weighted by
  the lever arms, so roughly metres of motion.
- **mjData** (stop 1): MuJoCo's state: time, positions, speeds, commands and everything
  worked out from them.
- **mjModel** (stop 1): MuJoCo's description of the world: bodies, joints, masses, motors,
  the time step. Constant while simulating.
- **mocap body** (stop 7): a body the physics holds fixed but whose pose lives in the
  state, so it can be moved between trials without changing the model; the pallet.
- **mock hardware** (stop 7): in ros2_control, a stand-in robot that reports each
  commanded position back as measured: no physics at all.
- **module** (stop 0): a solar panel, 2.3 m by 1.1 m, 30 kg here.
- **move (C++)** (stop 7): handing an object's contents to another object instead of
  copying them.
- **MoveIt** (stop 13): the motion planning framework of ROS: planning scene, planners,
  kinematics, execution.
- **MoveIt Task Constructor** (stop 11): a MoveIt library that describes a task as a
  sequence of stages and plans them together.

## N

- **naN** (stop 6): "not a number", a floating-point value used here for "no reading";
  every comparison with it is false.
- **narrow passage** (stop 9): a thin corridor of free space that random samples seldom
  land in.
- **nominal** (stops 0, 7): exactly as the drawing says.

## O

- **OMPL** (stops 9, 13): the Open Motion Planning Library, the collection of sampling
  planners MoveIt usually plans with.

## P

- **padding** (stop 8): MoveIt's name for a margin added round the robot's links.
- **pallet** (stop 0): what the modules arrive on, standing on edge.
- **path** (stop 10): a shape in joint space: waypoints and the lines between them, with
  no times.
- **path parameter (s)** (stop 10): how far along the path the arm is; the one quantity
  the timing chooses over time.
- **path speed** (stop 10): the rate of change of s; the arm's single speedometer.
- **payload (load data)** (stop 4): the mass, centre of mass and inertia of what the arm
  carries, which a controller's model must be told.
- **pinhole model** (stop 6): the camera as one point that every ray passes through; a
  point at (x, y) and depth z lands at pixel (cx + f x / z, cy - f y / z).
- **placement error** (stop 0): how far a module lands from its slot, in millimetres and
  degrees.
- **planning margin** (stop 9): the margin every checked configuration must keep so that
  the moving arm keeps the required one (45 mm here).
- **planning pipeline** (stop 9): MoveIt's chain of adapters round a planner plug-in.
- **planning scene** (stops 8, 11, 13): the planner's picture of where the obstacles are;
  in this repository `PlanScene`, in MoveIt the name of the same thing.
- **plausibility test** (stop 11): refusing a measurement that the hardware could not have
  produced (a module tilted on the cups, or far from the tool tip).
- **PLC** (stops 7, 11, 13): programmable logic controller: the industrial computer that
  runs a real cell's conveyors, interlocks and safety devices.
- **point cloud** (stop 6): the set of 3D points made from a depth image, one per pixel
  with a reading.
- **pose** (stop 2): where one frame is relative to another: a rotation and a translation.
- **position servo** (stop 4): a motor controller given a wanted angle; pushes in
  proportion to the error and brakes in proportion to speed.
- **principal point** (stop 6): where the camera's viewing axis meets the image; `cx`,
  `cy`.
- **prior** (stop 6): what perception knows before it looks: the design poses and sizes
  and how far off a pallet may be set down (`PalletPrior`).
- **probabilistically complete** (stop 9): if a path exists, the chance of finding one
  tends to 1 with more iterations; says nothing about how many, or about path length.
- **profile (velocity profile)** (stop 3): the timing of one move from rest to rest along
  one axis, within limits.

## Q

- **qpos, qvel, ctrl** (stop 1): the arrays in `mjData` holding every position, every
  speed and every motor command.
- **quantisation** (stop 6): readings rounded to a fixed step, here whole millimetres.
- **quaternion** (stop 2): four numbers encoding an orientation; MuJoCo's storage form.

## R

- **radian** (stop 1): the angle unit of the code; 1 rad is about 57.3 degrees, and 1 mrad
  at 1 m is 1 mm.
- **randomization** (stop 7): setting each trial's scene a little off nominal, within
  stated ranges, from a seed (`Randomization`, `Cell::Reset`).
- **RANSAC** (stop 6): random sample consensus: fit to a few random points, count the
  points that agree, keep the best supported, then refine on its supporters.
- **reach** (stop 5): how far from its base the arm can put the wrist.
- **refusal** (stop 6): the finder returning nothing with a reason (`FinderFailure`)
  instead of a doubtful pose.
- **regression** (stop 12): something that used to work and no longer does after a change.
- **repeatability, accuracy** (stop 13): how closely an arm returns to a taught pose,
  against how closely it reaches a computed one; the first is much better.
- **replay** (stop 12): running a saved failure again and comparing every recorded value
  with the original.
- **rest-to-rest** (stop 3): starting and ending with zero speed.
- **root mean square (rms)** (stop 14): the square root of the mean of the squares; a
  typical size that punishes large errors.
- **ROS 2** (stop 13): a framework of communicating processes (nodes) that most research
  and many production robot systems are built from.
- **ros2_control** (stop 13): the ROS 2 layer between controllers and robot hardware.
- **rotation matrix** (stop 2): a 3 by 3 table whose columns are the child frame's axes in
  the parent's coordinates.
- **rotation vector** (stop 2): an arrow along the axis of a turn whose length is the
  angle.
- **RRT** (stop 9): rapidly-exploring random tree: a tree of free configurations grown
  toward random samples.
- **RRT-Connect** (stop 9): two such trees, one from the start and one from the goal, each
  trying to reach the other's newest node.
- **Ruckig** (stop 10): a library for jerk-limited trajectories computed on the fly.
- **rule of three** (stop 12): after n tries with no failure, the failure rate is below
  about 3/n with 95% confidence.

## S

- **S-curve** (stop 3): the profile whose acceleration ramps at a jerk limit; up to seven
  segments.
- **sag** (stop 4): the steady error a joint needs under gravity when nothing compensates.
- **sampling-based planner** (stop 9): a planner that learns about free space only by
  testing individual configurations.
- **saturation** (stop 4): the wanted torque exceeds the motor's limit, so the motor gives
  its limit and the plan is no longer followed.
- **scorer** (stops 7, 12): the code that judges a run from the truth (`src/score.h`); it
  reports errors and sets no tolerance.
- **scratch data** (stop 5): a second `mjData` used to try joint angles without disturbing
  the simulation.
- **seed** (stops 0, 5, 6, 9, 12): the number a trial's random draws start from; the same
  seed gives the same trial. In a numeric solver (stop 5), the guess it starts from, which
  decides which answer it finds.
- **self-collision** (stop 8): the arm, tool or held module too close to another part of
  themselves.
- **sensor (MuJoCo)** (stop 7): a value the model reports, such as the conveyor slide's
  position and speed (its encoder).
- **sensor data** (stops 0, 7): what a real cell could know: joint angles, an encoder, a
  picture.
- **servo** (stop 1): a motor with a controller that drives a joint toward a commanded
  angle.
- **settling** (stop 7): the short time after a scene is set up in which resting bodies
  take up their weight; `Cell::kSettleTime`.
- **shard** (stop 12): one of several separate processes into which a slow test is split.
- **shortcutting** (stop 9): shortening a path by replacing a stretch with a straight line
  wherever that line is free.
- **sim-to-real gap** (stop 13): everything a simulation assumes that the real machine
  does not honour.
- **simulated time, wall time** (stop 0): seconds that pass for the simulated robot,
  against seconds that pass for you.
- **singularity** (stop 5): a pose where the arm cannot move the tool in some direction,
  so nearby poses need very fast joint motion.
- **site** (stop 7): a marked point with a frame on a body, such as `tool_tip`.
- **slerp** (stop 5): turning between two orientations about one fixed axis at a steady
  rate.
- **slot** (stop 0): one pair of rails on the tube, where one module goes.
- **speed, acceleration, jerk** (stop 3): how fast position changes, how fast speed
  changes, how fast acceleration changes.
- **stall** (stop 5): a solve that has stopped improving: out of reach, or held at a joint
  limit.
- **standard deviation (sigma)** (stop 6): the typical size of a random error; averaging N
  independent errors shrinks it by the square root of N.
- **standoff** (stop 11): the pose a set distance in front of the glass from which the
  tool moves straight in.
- **state machine** (stop 11): a program that is always in one of a fixed set of states
  and moves between them on conditions.
- **step (of the cycle)** (stop 11): one named stage of the script, such as "touch glass"
  or "carry"; `PickPlaceStep`.
- **stopping error** (stop 7): how far the conveyor really stops from where it was told
  to; drawn per move, and not cumulative because each move aims at an absolute position.
- **stretch** (stop 10): a piece of the path traversed under one set of caps on path
  speed, acceleration and jerk.
- **sweep bound** (stop 8): the most any point can move along a straight joint-space
  segment: the sum over joints of lever arm times angle turned.
- **synchronized** (stop 3): every joint follows the same fraction of its own distance, so
  the slowest sets the pace.

## T

- **tf2** (stop 2): ROS 2's shared tree of named, time-stamped frames.
- **tick** (stop 11): one call of a function that is run at a fixed rate and must return
  at once; here `PickPlace::Update`, once per physics step.
- **time parameterization** (stop 10): turning a path into a trajectory; MoveIt's name for
  this step.
- **time step** (stop 1): the slice of simulated time one `mj_step` advances: 0.002 s
  here.
- **time-optimal** (stop 10): the fastest trajectory along a path within the limits; this
  code is near it, not at it.
- **tolerance** (stop 12): how far from its slot a module may land and still count as
  placed; the owner's decision, here 3 mm and 0.15 degrees.
- **tool tip** (stop 2): the frame where the cups meet the glass, +Z out of the tool.
  Industry calls it the TCP, tool centre point.
- **TOPP-RA** (stop 10): a well-known general method and library for time-optimal timing
  along a path.
- **torque tube** (stop 0): the steel tube of a solar tracker that the modules' rails are
  mounted on.
- **tracking error** (stops 4, 11): measured joint angle minus planned joint angle.
- **trajectory** (stop 10): a path with a clock: position, speed, acceleration and jerk at
  every instant.
- **trapezoid** (stop 3): the profile that accelerates at the limit, cruises at top speed
  and brakes at the limit; its jerk is unbounded at the corners.
- **trial** (stop 12): one randomized pallet, drawn from a seed, and the cycles run on it.
- **truth** (stops 0, 7): what the simulator knows and a real cell could not: where things
  really are. Named `True...` in the code.

## U

- **undefined behaviour** (stop 3): code the language gives no meaning to; it may crash,
  give wrong numbers or seem to work.
- **unit test** (stop 12): a test of one small piece in isolation, fast enough to run
  constantly.
- **URDF** (stop 1): the robot description format of ROS; MuJoCo's own is MJCF.

## V

- **vacuum switch** (stop 11): the suction tool's sensor that says whether something is
  held.
- **verdict** (stop 12): the judgement on a cycle or a trial: in tolerance or not, passed
  or not.

## W

- **wall-clock time** (stop 12): time as a clock on the wall measures it, as opposed to
  simulated time.
- **waypoint** (stop 10): one configuration a path is given through.
- **world frame** (stop 2): the cell's fixed frame: origin on the floor under the arm's
  base, +X toward the pallet, +Z up.
- **wrist flip** (stop 5): the same tool pose with joint 4 and joint 6 half a turn on and
  joint 5 bent the other way.

## Y

- **yaw, tilt** (stop 2): turned about the vertical; out of flat.
