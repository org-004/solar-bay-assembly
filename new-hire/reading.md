# Reading list

Everything the chapters recommend, in one place, stop by stop. Each stop offers a
foundation, something practical and something current. Nobody is expected to read all
of it in two weeks; each entry says how much is worth reading.

## If you read only five things

- Kevin Lynch and Frank Park, *Modern Robotics*, free at <http://modernrobotics.org>
  with a short video for every section. Chapters 3 (poses), 5 and 6 (the Jacobian and
  inverse kinematics), 9 (trajectories) and 11 (control) cover stops 2 to 5 and 10.
- Steven LaValle, *Planning Algorithms*, free at <https://lavalle.pl/planning/>.
  Chapters 4 and 5 cover stops 8 and 9.
- Bjarne Stroustrup, *A Tour of C++* (3rd edition). The whole language in a short book,
  for people who already program.
- This repository's own long comments, at the top of each header in `src/`, with
  `docs/gate.md` and `docs/ompl-integration.md`. They say why each thing is as it is,
  with the numbers.
- The concepts pages of ROS 2 (<https://docs.ros.org/en/rolling/Concepts/Basic.html>)
  and MoveIt (<https://moveit.picknik.ai/main/doc/concepts/concepts.html>), for the
  vocabulary of the production cells.

## Stop 0: Watch it work

- `docs/gate.md` in this repository: how a change is judged. Skim it now; stop 12
  returns to it.
- MuJoCo's overview, <https://mujoco.readthedocs.io/en/stable/overview.html>: the first
  two sections, for what a physics simulator does.
- ROS 2 concepts, <https://docs.ros.org/en/rolling/Concepts/Basic.html>: nodes, topics,
  services and actions, a page each.
- MoveIt concepts, <https://moveit.picknik.ai/main/doc/concepts/concepts.html>: the
  diagram of `move_group` is enough for now.
## Stop 1: The smallest program

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
## Stop 2: Where things are

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 3, "Rigid-Body Motions". Free
  at <http://modernrobotics.org>, with short videos for each section. Sections 3.1 to
  3.3 are what this stop uses; leave twists and wrenches for later.
- 3Blue1Brown, *Essence of linear algebra*,
  <https://www.3blue1brown.com/topics/linear-algebra>: chapters 3 and 4 (matrices as
  transformations, and multiplying them), if matrices are a distant memory.
- Eigen, "Space transformations",
  <https://eigen.tuxfamily.org/dox/group__TutorialGeometry.html>: the reference for
  `Isometry3d`, `AngleAxisd` and `Quaterniond`.
- ROS REP 103, <https://www.ros.org/reps/rep-0103.html>: one page of conventions.
- tf2 in the ROS 2 documentation,
  <https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Tf2.html>.
## Stop 3: Moving one axis

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 9, "Trajectory Generation",
  sections 9.1 and 9.2: point-to-point moves, time scaling, the trapezoid and the
  S-curve. <http://modernrobotics.org>, with videos.
- Luigi Biagiotti and Claudio Melchiorri, *Trajectory Planning for Automatic Machines
  and Robots* (Springer, 2008), chapter 3: the reference for the trapezoid and the
  seven-segment "double S" profile, every case worked out. For when you need to change
  `profile.cc`, not before.
- Lars Berscheid and Torsten Kröger, "Jerk-limited Real-time Trajectory Generation
  with Arbitrary Target States" (Robotics: Science and Systems, 2021), and the library
  it describes, Ruckig, <https://github.com/pantor/ruckig>. Current practice: the same
  jerk-limited profiles, but from any moving state to any other. Read the README;
  `docs/ompl-integration.md` in this repository discusses bringing it in.
- cppreference on `std::optional`, <https://en.cppreference.com/w/cpp/utility/optional>:
  the first screen, and the list of ways to look inside.
## Stop 4: Motors

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
## Stop 5: Pose to joint angles

- Kevin Lynch and Frank Park, *Modern Robotics*: chapter 4 for forward kinematics,
  section 5.3 for singularities, section 6.2 for numerical inverse kinematics.
  <http://modernrobotics.org>. Chapter 5's Jacobian is the one built in `Evaluate`.
- Samuel Buss, "Introduction to Inverse Kinematics with Jacobian Transpose,
  Pseudoinverse and Damped Least Squares Methods" (2004, revised 2009). Twenty pages on
  why damping works. Search for the title; it is on the author's university page.
- Patrick Beeson and Barrett Ames, "TRAC-IK: An Open-Source Library for Improved
  Solving of Generic Inverse Kinematics" (IEEE-RAS Humanoids, 2015). What a production
  numeric solver adds. Read the introduction and the results table.
- MoveIt's documentation on kinematics configuration and on IKFast,
  <https://moveit.picknik.ai>: how a solver is chosen per group, and its timeout.
- Alexander Elias and John Wen, "IK-Geo: Unified Robot Inverse Kinematics Using
  Subproblem Decomposition" (2022). Current work on closed forms for six-axis arms.
## Stop 6: Seeing

- Richard Szeliski, *Computer Vision: Algorithms and Applications* (2nd edition), free
  at <https://szeliski.org/Book/>: section 2.1, for the pinhole camera done properly.
- Martin Fischler and Robert Bolles, "Random Sample Consensus: A Paradigm for Model
  Fitting with Applications to Image Analysis and Automated Cartography",
  *Communications of the ACM*, 1981. The RANSAC paper; its first pages are the method.
- OpenCV, "Camera Calibration and 3D Reconstruction",
  <https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html>: the introduction, for
  intrinsics and distortion on a real camera, and `calibrateHandEye` for hand-eye.
- Shreeyak Sajjan and others, "ClearGrasp: 3D Shape Estimation of Transparent Objects
  for Manipulation", ICRA 2020. Current work on the weakness this simulation hides.
## Stop 7: The cell and the truth

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
## Stop 8: Will it hit anything

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
## Stop 9: Finding a path

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
## Stop 10: Timing a path

- Kevin Lynch and Frank Park, *Modern Robotics*, chapter 9, "Trajectory Generation",
  free at <http://modernrobotics.org>: 9.1 for path against trajectory, 9.4 for
  time-optimal timing along a path, with the picture every paper on the subject uses.
- Tobias Kunz and Mike Stilman, "Time-Optimal Trajectory Generation for Path Following
  with Bounded Acceleration and Velocity" (Robotics: Science and Systems, 2012): the
  method MoveIt ships. Read the sections on blending corners and on the limit curve.
- Hung Pham and Quang-Cuong Pham, "A New Approach to Time-Optimal Path
  Parameterization Based on Reachability Analysis" (IEEE Transactions on Robotics,
  2018), and <https://github.com/hungpham2511/toppra>: TOPP-RA. The backward and
  forward passes of this stop, made general. Read the introduction and the
  description of the two passes.
- Lars Berscheid and Torsten Kröger, "Jerk-limited Real-time Trajectory Generation
  with Arbitrary Target States" (Robotics: Science and Systems, 2021), and
  <https://github.com/pantor/ruckig>: Ruckig, the current standard for jerk-limited
  motion computed on the fly. `docs/ompl-integration.md` has a note on what bringing
  it into this repository would take ("Applied to Ruckig").
## Stop 11: The cycle

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
## Stop 12: Proving it

- James Hanley and Abby Lippman-Hand, "If Nothing Goes Wrong, Is Everything All Right?
  Interpreting Zero Numerators" (JAMA, 1983). A few pages; the source of the 3/n rule.
- GoogleTest's primer, <https://google.github.io/googletest/primer.html>. All of it:
  tests, fixtures and assertions. `SCOPED_TRACE` is in its advanced guide.
- Bruce Dawson, "Floating-Point Determinism" (Random ASCII, 2013),
  <https://randomascii.wordpress.com/2013/07/16/floating-point-determinism/>. What
  does and does not make two machines agree to the last bit.
- Nidhi Kalra and Susan Paddock, "Driving to Safety: How Many Miles of Driving Would
  It Take to Demonstrate Autonomous Vehicle Reliability?" (RAND, 2016). The same
  arithmetic as the 240, at the scale of an industry. The summary is enough.
- ISO 9283, "Manipulating industrial robots: Performance criteria and related test
  methods". The standard behind "accuracy" and "repeatability" on a robot's
  datasheet. Know that it exists and what it defines.
## Stop 13: Towards a real cell

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
## Stop 14: A question of your own

- Richard Hamming, "You and Your Research", a talk from 1986, widely reprinted. On
  choosing problems worth working on.
- `docs/ompl-integration.md`, "How this was measured": the standard of evidence this
  repository holds itself to.
- Mark Moll, Ioan Sucan and Lydia Kavraki, "Benchmarking Motion Planning Algorithms: An
  Extensible Infrastructure for Analysis and Visualization", IEEE Robotics and
  Automation Magazine, 2015: how the planning community compares planners.
- Any one paper from `reading.md` that touches your question, read properly: the
  method, then the experiments, then what they did not test.
