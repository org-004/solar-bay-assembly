# The flat rack cell's first step in physics: one module carried and set

Until run 019 every figure of the flat rack cell was kinematic. The committed plan
(`plans/flat_rack_3_pin_arms_ordered.json`, [`flat-rack-cell.md`](flat-rack-cell.md),
[`flat-rack-order.md`](flat-rack-order.md)) says where every arm is at every instant,
and the verifier checks it for clearance, but nothing had driven it. This page is about
the first run in physics. In it the module arm, on its position servos and with its real
payload, follows the plan through one module's whole cycle in MuJoCo 4.0.0:
1. it takes module 4 from the pallet;
2. it carries it and turns it from upright to flat past the tube;
3. it sets it on its two rails and lets it go.

The run is measured as the checker measures. Its committed record,
`physics/flat_rack_3_pin_arms_ordered.json`, is what the scorecard's `physics_share`
counts ([`scorecard.md`](scorecard.md)).

Since run 054 the chosen layout is v21, `flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed`
(all KUKA, v20 with an infeed unit of its own), so `run_physics` with no name runs v21's plan; its record is in
"[v21's record (run 054)](#v21s-record-run-054)" below, after v20's, chosen from run 052 to run 054, in
"[v20's record (run 052)](#v20s-record-run-052)", v19's, chosen from run 048 to run 052, in
"[v19's record (run 048)](#v19s-record-run-048)", v18's, chosen from run 046 to run 048, in
"[v18's record (run 046)](#v18s-record-run-046)", v17's, chosen from run 043 to run 046, in
"[v17's record (run 043)](#v17s-record-run-043)", and v16's, chosen from run 040 to run 043, in
"[v16's record (run 040)](#v16s-record-run-040)". The rest of this page is v10's
run, made in run 019.

Every figure below is printed by one of these commands, at the commit that added this
page:

- `bazel run --config=opt //:run_physics` prints the chosen layout and the take before
  its hardest carry (module 4). `-- --json` prints the record, and
  `-- flat_rack_3_pin_arms_ordered --write` writes it. Two runs print the same, byte
  for byte.
- `bazel run --config=opt //:run_physics -- flat_rack_3_pin_arms_ordered --take K` runs
  module_take K instead. It is used for the table of all nine cycles below.
- `bazel run --config=opt //:score_layout -- flat_rack_3_pin_arms_ordered` prints the
  scorecard.
- `bazel run --config=opt //:viewer_layout -- flat_rack_3_pin_arms_ordered --physics`
  shows the run in the viewer, and needs a display. `tools/video/physics_gif.sh` records
  it. The recording is below.

As in the other documents, a figure is `derived` when one of these programs printed it,
and `ours` when it is a choice made here.

![v10 in physics: module 4 taken from the pallet, turned flat past the tube and set on its rails, twice real time](evolution/v10-physics.gif)

## The method

**The model.** It comes from the same builder as the checker's (`src/layout_model.h`,
"Physics"). With `LayoutState::physics_arm` empty the builder's text is what it always
was, so every check, plan and scorecard figure is unchanged.
`tests/layout_model_test.cc`, `Physics.*`, holds that the physics state adds only three
things:

- **The module arm's servos.** It keeps the six position servos of `models/arm.xml`, with
  the gains, damping ratio and torque limits found there, and the joints keep that
  file's armature and damping. The links keep the vendor's masses and inertias
  (published, from KUKA's URDF).
- **A track servo.** The arm's track gets a position servo of our own choosing:
  - gain 7.7e6 N/m;
  - damping 2.45e5 N s/m;
  - force limit 6 kN.

  The argument for these figures is in `src/layout_model.h`. The moving mass is about
  1,950 kg, which gives a natural frequency of 10 Hz and critical damping, and the force
  limit is twice what the planned 1.5 m/s² needs.
- **A free module.** Pallet module 2 in the order of taking is module 4 of the bay
  (order 0, 1, 4, ...). It becomes a free body of 32.4 kg, with the same nine geoms as
  every module. Two welds come with it, with the stand-in cell's suction settings
  (`models/suction.xml`):
  - `pallet_hold` keeps it on its pallet until the grip;
  - `suction` takes it at the grip.

The combined gripper weighs the layout's 36 kg (ours), shared over its three boxes by
volume.

**The run.** It is in `src/physics_run.h`.

1. **Hold.** The arm holds the take's first pose for 1 s while the plan's clock is
   stopped. This hold is not counted.
2. **The two steps.** The arm then runs plan steps 63 (take module 4) and 64 (set
   module 4) end to end, from 268.264885 s to 290.333739 s of the plan. That is
   22.068854 s of the plan's time.
   - **Following.** `ArmFollower` (`src/arm_follower.h`) follows the plan's own angles
     as a `TimedPath`. The angles come from `PlanPlayer`, which the verifier and the
     viewer also play. Speeds and accelerations are central differences over 1 ms.
     The follower uses lag compensation and model feedforward, including the track's
     planned acceleration.
   - **The track.** The track's servo gets the force that the mass matrix gives for the
     planned accelerations of all seven axes.
   - **The grip.** At the end of the take's work segment, `pallet_hold` is switched off
     and `Suction::Grip` binds the module where it is.
   - **The release.** At the end of the set's work segment, `Suction::Release` lets it
     go. The tool tip was 0.0955 mm from the glass at the grip (derived).
3. **Settle.** The arm holds the set's retreat pose for 2 s. The module's pose is then
   taken.

**Physics settings.**
- **Timestep:** 0.5 ms.
- **Contacts:** only the carried module touches anything, and only rails 4 and 5, the
  two it is set on. The contact settings are those of
  `tests/locate_physics_test.cc`, scaled to this timestep:
  - solref of four timesteps;
  - solimp 0.95, 0.99, 0.0005;
  - elliptic cones;
  - friction 0.61.

  Nothing else is given contacts. What the plan keeps 15 mm apart is measured, not left
  to collide.
- **The other three arms** stand where the plan has them at each instant. They are
  kinematic: their joints are written every step, at rest.

**The figures.** Joint tracking errors are measured every step. The other figures are
sampled every 0.01 s of plan time, five times as often as the verifier samples.

- **Joint tracking error:** measured minus planned, the largest over the six joints, in
  mrad. The track's error is given in mm.
- **Frame error:** the Cartesian error against the plan, in mm and degrees.
  - While the module is held, this is the module's frame against the held module in the
    checker's model at the planned configuration.
  - Otherwise it is the error of the tool's working point.
  - A module is the same shape turned half round about its normal: each geom of side
    `a` has a twin on side `b`, and the four holes are symmetric about the centre. The
    plan takes some modules from the pallet that way round, so the smaller of the two
    errors is the figure.
- **Least clearance in physics:** this uses the verifier's own function,
  `InstantDistances` (`src/layout_plan.h`), on the checker's model, where capsules are
  measured as the cylinders that cover them.
  - The module arm is put at its measured joints and track.
  - The module's geoms, held or set, are moved to where the physics has the module.
  - The same function over the plan's own world, at the same instants, gives the plan's
    figure beside it.

  The two agree to 0.02 mm wherever the arm follows closely, which checks the
  measurement itself.
- **Set-down:** the module's pose on its rails after release and settling, against its
  designed place in the bay. The rails are fixed where the bay puts them, so this is the
  gate's reading "from the rails as they lie" ([`bay-hardware.md`](bay-hardware.md),
  "Tolerances").

## What the run carried out (R3)

Module 4, steps 63 and 64. All figures are derived.

| Step | Segment | From s | To s | Joint error mrad | Track error mm | Frame error mm | Frame error deg | Least clearance in physics m | The plan's m | Pair in physics |
|---|---|---|---|---|---|---|---|---|---|---|
| 63 | move | 268.265 | 275.574 | 0.0764 | 0.0119 | 0.0720 | 0.0051 | 0.020254 | 0.020250 | gripper plate to rail 4's top |
| 63 | approach | 275.574 | 276.522 | 0.0054 | 0.0000 | 0.0045 | 0.0001 | 0.031023 | 0.031028 | gripper cups to pallet module 3 |
| 63 | work | 276.522 | 278.522 | 0.0054 | 0.0000 | 0.0045 | 0.0001 | 0.030995 | 0.031000 | gripper cups to pallet module 3 |
| 63 | retreat (held) | 278.522 | 280.922 | 0.5722 | 0.0000 | 0.6021 | 0.0420 | 0.029787 | 0.030001 | held module to pallet module 4 |
| 64 | move (held) | 280.922 | 285.436 | 1.7876 | 0.0419 | 1.2649 | 0.1163 | 0.027430 | 0.027247 | held module to pallet module 3 |
| 64 | approach (held) | 285.436 | 286.385 | 0.2201 | 0.0186 | 0.3589 | 0.0109 | 0.034679 | 0.034709 | held module to the arm's link 2 |
| 64 | work (held) | 286.385 | 289.385 | 0.2242 | 0.0191 | 0.2271 | 0.0107 | 0.062000 | 0.061961 | held module to the tube |
| 64 | retreat | 289.385 | 290.334 | 0.2242 | 0.0104 | 0.0786 | 0.0112 | 0.064079 | 0.064111 | the arm's link 2 to module 4 |

Notes on the table:
- **Pairs.** The names in the last column are shortened. The full geom names are in the
  program's output.
- **The plan's least clearance.** It is 0.0202 m for these two steps. The plan's least
  over the whole schedule is 0.0180 m, elsewhere.
- **Peak torque over its limit**, joints 1 to 6 and then the track:
  - step 63: 0.975, 0.633, 0.167, 0.117, 0.268, 0.292, 0.717;
  - step 64: 0.928, 0.893, 0.710, 0.505, 0.493, 0.404, 0.509.

**Set-down** (derived):

| Figure | Value | Gate |
|---|---|---|
| Offset in the rails' plane | 0.1989 mm (across 0.0417, along -0.1945) | 3 mm |
| Yaw about the rails' normal | 0.0052 deg | 0.15 deg |
| Height | -0.0001 mm | not part of the gate |

**The rule** (`docs/scorecard.md`, `physics_share`): a step counts when its least
clearance in physics is at least the layout's `clearance_m`, 0.015 m, and the module
ends within the gate.

Both steps count, so 22.068854 s of one bay's 560.452329 s (`one_bay_s`) is carried out in physics:
**`physics_share` 0.039377** (derived). Before this run it was 0.

## The tightest margins of the run

| Margin | Figure | Limit | Left |
|---|---|---|---|
| Joint 1's peak torque, step 63 (the take's move, empty gripper, track at 1.5 m/s²) | 0.975 of 7,290 N m | 1.0 | 2.5% |
| Joint 1's peak torque, step 64 | 0.928 | 1.0 | 7.2% |
| Least clearance in physics, step 63 | 0.020254 m | 0.015 m | 5.25 mm |
| Least clearance in physics, step 64 | 0.027430 m | 0.015 m | 12.4 mm |
| Largest joint tracking error, step 64's carry | 1.7876 mrad | none set | |
| Largest module frame error, step 64's carry | 1.2649 mm, 0.1163 deg | none set | |
| Set-down offset | 0.1989 mm | 3 mm | 2.80 mm |
| Set-down yaw | 0.0052 deg | 0.15 deg | 0.145 deg |

**Joint 1's torque is the figure nearest its limit.** It reaches the limit in seven of
the other eight cycles (below). Ways to widen it:
- a slower track acceleration while the arm is extended;
- a plan whose timing knows the payload and the motors' torque, not only the
  datasheet's joint speeds.

**Physics changes the clearance by less than its 0.2 mm.** In this cycle physics took at
most 0.2 mm off the plan's clearance (0.0003 m on the take's retreat). The 3 mm between
the plan's own least and the limit is not at risk here.

## All nine module cycles in physics (not counted)

The record holds module 4 only, as the spec asked, and only it counts towards
`physics_share`. The same program runs any other take. All nine were run with
`--take 0` to `--take 8` at the same commit, to learn for the next step. All figures are
derived.

| Take | Module | Steps | Joint error, take / set, mrad | Frame error, take / set, mm | Least clearance in physics, take / set, m | Peak joint 1 torque, take / set | Set-down mm, deg | Counts by the rule |
|---|---|---|---|---|---|---|---|---|
| 0 | 0 | 33, 34 | 305.80 / 1.61 | 445.4 / 0.74 | **-0.030905** / 0.018782 | **1.000** / 0.374 | 0.167, 0.006 | 33 no, 34 yes |
| 1 | 1 | 49, 50 | 0.57 / 0.82 | 0.59 / 1.20 | 0.021130 / 0.018942 | 0.919 / 0.620 | 0.171, 0.006 | both |
| 2 | 4 | 63, 64 | 0.57 / 1.79 | 0.60 / 1.26 | 0.020254 / 0.027430 | 0.975 / 0.928 | 0.199, 0.005 | both (the record) |
| 3 | 2 | 79, 80 | 7.27 / 4.88 | 12.53 / 11.67 | 0.017794 / 0.017733 | **1.000** / **1.000** | 0.204, 0.005 | both |
| 4 | 5 | 93, 94 | 0.57 / 10.69 | 0.62 / 25.64 | 0.018991 / 0.018592 | 0.471 / **1.000** | 0.206, 0.000 | both |
| 5 | 7 | 107, 108 | 2.10 / 4.41 | 4.04 / 1.24 | 0.018538 / 0.021562 | **1.000** / 0.978 | 0.218, 0.000 | both |
| 6 | 6 | 123, 124 | 0.90 / 0.61 | 1.50 / 0.79 | 0.018847 / 0.018726 | **1.000** / 0.290 | 0.288, 0.003 | both |
| 7 | 3 | 137, 138 | 0.57 / 20.02 | 0.66 / 47.77 | 0.018521 / 0.020366 | 0.556 / **1.000** | 0.242, 0.006 | both |
| 8 | 8 | 151, 152 | 0.57 / 2.75 | 0.69 / 1.05 | 0.023689 / 0.018498 | **1.000** / **1.000** | 0.264, 0.002 | both |

**What the table says.**
- **Seventeen of the eighteen steps would count.** Counting all of them would carry about
  0.39 of the bay's time in physics. That is not claimed: those runs are not committed
  records, and the record format holds one cycle.
- **Joint 1 reaches its torque limit in seven of the nine cycles**: takes 0, 3, 4, 5, 6,
  7 and 8. The track reaches its own limit in take 0 (1.000), and 0.975 of it in take
  8.
- **Where joint 1 saturates, the arm falls behind the plan.** By up to 20 mrad and 48 mm
  in take 7's carry.
- **In take 0 the arm fell 0.31 rad behind on the move to the pallet.** The gripper's
  cups went 31 mm into module 0 on its pallet: that is the clearance of -0.030905 m.
- **The plan's 3 mm of margin hid every other shortfall.** Every other step still kept
  more than 15 mm, but only because of that margin. Take 3's set came within 0.0177 m,
  0.9 mm less than the plan's 0.0186 m.

**What would make take 0 pass.** Its demand is 1.21 times joint 1's limit at the planned
accelerations (an inverse-dynamics estimate made while planning run 019, not a committed
program). Playing that move about 1.1 times slower (the square root of 1.21) would bring
the demand within the limit. That is about 0.7 s more on a 7.1 s move. This is estimated
and was not run.

## What is still not in physics

- **The other three arms.** The pin arms are kinematic: written into the model every
  step, with no contacts and no servos.
- **The rails.** No rail is carried in physics, and the rails are fixed where the bay
  puts them.
- **The other modules.** Except in the table above (which is not counted), no other
  module is carried. The modules already set are drawn, as far as the builder's count
  allows, and touch nothing.
- **The pallet.** A weld stands in for the pallet and the module's neighbours until the
  grip. The pallet's own stability, and the neighbour that leans, are not simulated.
- **Suction.** It is a weld, as in the stand-in cell. Cups, vacuum and their compliance
  are not modelled.
- **The fastenings.** The strap bolts, the locating move, the pins, the swaging and the
  tube's clamp are not in physics. The locating nose has its own physics test
  (`tests/locate_physics_test.cc`), without an arm.
- **Sensing.** There is no sensing and no correction from what is sensed. The arm
  follows the plan blind, and the module's pose is read from the simulation.
  `placement_mm` therefore stays `none`: by `docs/scorecard.md`, "What would make it
  measured", it needs the cell to hold *and sense* the modules. The set-down above is
  the true pose in simulation without sensing, so it is a first figure of the
  placement's own error, not a placement error the cell could know.

## The next step

The spec of run 019 calls this run the first of seven steps that run 012's report
listed. That report is not in this repository and was not available to run 019. The
spec names some of the later steps:
- the pin arms, bolts, locating move and pins in physics;
- the rails' straps;
- the tube's clamp;
- the ports' units;
- as step 5, all four arms following the whole schedule at once.

What this run learned for those steps:

1. **The plan's timing is not torque-feasible for the module arm everywhere.** The
   planner times moves from the datasheet's joint speeds and accelerations, with the
   track at 1.5 m/s². It does not know the payload or the motors' torques. Seven of
   nine cycles drive joint 1 into its limit, and one runs the gripper into the pallet.
   - Before more of the schedule is run in physics, the plan's timing should check
     torque, for example with an inverse-dynamics bound per move (`mj_inverse` along the
     timed path), and slow the moves that exceed it. Each slower move must show in
     `cycle_s`.
   - The cheapest first fix is the track's acceleration while the arm is reaching
     forward, where the coupling into joint 1 is largest.
2. **The plan's 3 mm of margin is what kept the saturated carries clear.** The
   difference from the plan was at most 0.9 mm wherever the arm did not saturate, and
   0.2 mm in the recorded cycle. For an arm that saturates it can be several
   centimetres. A pad of 3 mm is enough only for an arm that follows.
3. **The machinery is reusable.** For the next arm, a pin arm, the UR10e needs servos:
   the builder allows only the KUKA today, because Menagerie's actuators are not the
   follower's position servos. Two things carry over as they are:
   - the `TimedPath` following;
   - measuring through `InstantDistances` with the geoms moved to the physics poses.
4. **The weld holds a 32.4 kg module during the carry.** The settings are the stand-in
   cell's. The set-down was 0.17 to 0.29 mm in all nine cycles, about a tenth of the
   gate's 3 mm. The locating nose's capture range (`bay-hardware.md`) starts from the
   gate's 3 mm, so a module laid down this way is well inside it.

## v16's record (run 040)

`bazel run --config=opt //:run_physics -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat --write`
wrote `physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json` the way run 019
wrote v10's. The same KR 210 module arm, on the same servos; the three KR 20 pin arms are
kinematic in the run. The take before the hardest carry is now module 3's (module_take 3,
plan steps 77 and 78), on v16's track timing.

| Figure (derived) | v16, module 3 | v10, module 4 (run 019) |
|---|---|---|
| counted, of one bay | 18.522716 s of 574.729769 s | 22.068854 s of 560.452329 s |
| `physics_share` | 0.032229 | 0.039377 |
| peak joint 1 torque over its limit, take / set | 1.000 / 1.000 | 0.975 at most |
| peak track force over its limit, take / set | 0.939 / 0.967 | |
| least clearance in physics, take / set | 0.020177 m / 0.025272 m (plan 0.020188 / 0.026289), against 0.015 m | 0.02025 m |
| largest joint tracking error, take / set | 0.6206 / 9.2649 mrad | |
| largest Cartesian error, take / set | 0.6419 mm / 17.7955 mm | |
| set-down offset after settling | 0.2014 mm, yaw 0.0052 deg (gate 3.0 mm, 0.15 deg) | 0.17 to 0.29 mm |

Both steps count. Joint 1 reaches its torque limit in both: the planner times moves from
datasheet joint speeds and knows no torque, as in run 019.

## v17's record (run 043)

`bazel run --config=opt //:run_physics -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths --write`
wrote `physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json` the way run 040
wrote v16's. The take before the hardest carry, as the tool chooses it, is now module 6's
(module_take 7, plan steps 136 and 137): with the modules set in turns (0, 4, 7, 1, 5, 8, 2, 6, 3)
module 6 is the eighth set.

| Figure (derived) | v17, module 6 | v16, module 3 (run 040) |
|---|---|---|
| counted, of one bay | 20.725585 s of 529.197537 s | 18.522716 s of 574.729769 s |
| `physics_share` | 0.039164 | 0.032229 |
| peak torque over its limit, joints 1-6, take | 1.000 0.814 0.251 0.079 0.348 0.163 | 1.000 0.815 0.332 0.241 0.298 0.122 |
| peak torque over its limit, joints 1-6, set | 1.000 0.932 0.712 0.150 0.516 0.459 | 1.000 0.513 0.664 0.491 0.509 0.450 |
| peak track force over its limit, take / set | 0.811 / 0.820 | 0.939 / 0.967 |
| least clearance in physics, take / set | 0.020108 m / 0.018916 m (plan 0.020120 / 0.018628), against 0.015 m | 0.020177 m / 0.025272 m |
| largest joint tracking error, take / set | 7.6382 / 2.1291 mrad | 0.6206 / 9.2649 mrad |
| largest Cartesian error, take / set | 13.9240 mm / 1.0381 mm | 0.6419 mm / 17.7955 mm |
| set-down offset after settling | 0.2434 mm, yaw -0.0013 deg (gate 3.0 mm, 0.15 deg) | 0.2014 mm, yaw 0.0052 deg |

Both steps count. Joint 1 reaches its torque limit in both, as in v16's: the planner times moves
from datasheet joint speeds and knows no torque.

## v18's record (run 046)

`bazel run --config=opt //:run_physics -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths --write`
wrote `physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json` the way run 043
wrote v17's. The take before the hardest carry, as the tool chooses it, is now module 5's
(module_take 7, plan steps 136 and 137): with three modules to each pin arm, set in turns
(0, 3, 6, 1, 4, 7, 2, 5, 8), module 5 is the eighth set.

| Figure (derived) | v18, module 5 | v17, module 6 (run 043) |
|---|---|---|
| counted, of one bay | 20.205074 s of 499.542691 s | 20.725585 s of 529.197537 s |
| `physics_share` | 0.040447 | 0.039164 |
| peak torque over its limit, joints 1-6, take | 1.000 0.887 0.398 0.109 0.489 0.243 | 1.000 0.814 0.251 0.079 0.348 0.163 |
| peak torque over its limit, joints 1-6, set | 0.420 0.597 0.315 0.124 0.335 0.200 | 1.000 0.932 0.712 0.150 0.516 0.459 |
| peak track force over its limit, take / set | 0.664 / 0.579 | 0.811 / 0.820 |
| least clearance in physics, take / set | 0.021488 m / 0.021586 m (plan 0.021488 / 0.021591), against 0.015 m | 0.020108 m / 0.018916 m |
| largest joint tracking error, take / set | 2.4907 / 0.8641 mrad | 7.6382 / 2.1291 mrad |
| largest Cartesian error, take / set | 5.8223 mm / 0.6994 mm | 13.9240 mm / 1.0381 mm |
| set-down offset after settling | 0.2315 mm, yaw 0.0004 deg (gate 3.0 mm, 0.15 deg) | 0.2434 mm, yaw -0.0013 deg |

Both steps count. Joint 1 still reaches its torque limit on the take, as in v17's (the planner
times moves from datasheet joint speeds and knows no torque); on the set it stays at 0.420, where
v17's reached 1.000.

## Torque-feasible timing (run 045)

Item 1 of "The next step", measured and bounded. `src/layout_torque.h` runs inverse
dynamics (`mj_inverse`) of the module arm along each timed move of a plan: the KR 210 in
the physics model (`models/arm.xml`'s links, armature, damping and torque limits, the
track's servo limit `kTrackForce`), with its tool and, while it holds one, the module
(32.4 kg, rigid at the tool's working point), constraints and contacts off. The demand is
|`qfrc_inverse`| over each motor's limit, sampled every 5 ms of the move with the
trajectory's own speeds and accelerations; legs are sampled by central differences over
1 ms, as the physics run follows them.

`bazel run --config=opt //:torque_demand -- <name> [--bay B] [--all]` prints the table for
a committed plan. For v16's bay 0 (columns j1 to track: peak demand over limit; "v16 s"
is the segment's duration, settle included; the last three columns are the layout with
`torque_limit_scale` 0.90 below):

| step | move | held | v16 s | j1 | j2 | j3 | j4 | j5 | j6 | track | 0.90: scale | 0.90: s | 0.90: worst |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 3 | wait -> rail_take 0 approach | - | 4.330 | 0.247 | 0.281 | 0.178 | 0.049 | 0.102 | 0.013 | 0.579 | 1.0000 | 4.330 | 0.579 |
| 4 | rail_take 0 retreat -> rail_set 0 approach | rail | 5.792 | 0.211 | 0.338 | 0.147 | 0.033 | 0.090 | 0.086 | 0.541 | 1.0000 | 5.792 | 0.541 |
| 5 | rail_set 0 retreat -> rail_take 1 approach | - | 5.751 | 0.223 | 0.349 | 0.139 | 0.037 | 0.070 | 0.019 | 0.542 | 1.0000 | 5.751 | 0.542 |
| 6 | rail_take 1 retreat -> rail_set 1 approach | rail | 5.217 | 0.191 | 0.356 | 0.146 | 0.037 | 0.082 | 0.030 | 0.549 | 1.0000 | 5.217 | 0.549 |
| 7 | rail_set 1 retreat -> rail_take 2 approach | - | 4.664 | 0.214 | 0.345 | 0.130 | 0.039 | 0.082 | 0.012 | 0.536 | 1.0000 | 4.664 | 0.536 |
| 8 | rail_take 2 retreat -> rail_set 2 approach | rail | 3.807 | 0.221 | 0.311 | 0.140 | 0.040 | 0.088 | 0.016 | 0.560 | 1.0000 | 3.807 | 0.560 |
| 9 | rail_set 2 retreat -> rail_take 3 approach | - | 3.807 | 0.270 | 0.331 | 0.129 | 0.041 | 0.069 | 0.016 | 0.560 | 1.0000 | 3.807 | 0.560 |
| 10 | rail_take 3 retreat -> rail_set 3 approach | rail | 3.044 | 0.286 | 0.395 | 0.148 | 0.049 | 0.129 | 0.023 | 0.597 | 1.0000 | 3.044 | 0.597 |
| 11 | rail_set 3 retreat -> rail_take 4 approach | - | 3.044 | 0.367 | 0.415 | 0.136 | 0.054 | 0.090 | 0.023 | 0.598 | 1.0000 | 3.044 | 0.598 |
| 12 | rail_take 4 retreat -> rail_set 4 approach | rail | 2.488 | 0.467 | 0.391 | 0.169 | 0.066 | 0.165 | 0.042 | 0.700 | 1.0000 | 2.488 | 0.700 |
| 13 | rail_set 4 retreat -> rail_take 5 approach | - | 2.488 | 0.571 | 0.413 | 0.157 | 0.058 | 0.117 | 0.042 | 0.701 | 1.0000 | 2.488 | 0.701 |
| 14 | rail_take 5 retreat -> rail_set 5 approach | rail | 2.200 | 0.824 | 0.393 | 0.213 | 0.101 | 0.249 | 0.078 | 0.559 | 1.0000 | 2.200 | 0.824 |
| 15 | rail_set 5 retreat -> rail_take 6 approach | - | 2.200 | 0.872 | 0.398 | 0.199 | 0.100 | 0.199 | 0.078 | 0.562 | 1.0000 | 2.200 | 0.872 |
| 16 | rail_take 6 retreat -> rail_set 6 approach | rail | 2.329 | 0.713 | 0.398 | 0.187 | 0.069 | 0.215 | 0.056 | 0.777 | 1.0000 | 2.329 | 0.777 |
| 17 | rail_set 6 retreat -> rail_take 7 approach | - | 2.329 | 0.624 | 0.400 | 0.173 | 0.075 | 0.166 | 0.056 | 0.789 | 1.0000 | 2.329 | 0.789 |
| 18 | rail_take 7 retreat -> rail_set 7 approach | rail | 2.838 | 0.426 | 0.427 | 0.154 | 0.054 | 0.179 | 0.027 | 0.616 | 1.0000 | 2.838 | 0.616 |
| 19 | rail_set 7 retreat -> rail_take 8 approach | - | 2.838 | 0.321 | 0.430 | 0.140 | 0.059 | 0.130 | 0.027 | 0.620 | 1.0000 | 2.838 | 0.620 |
| 20 | rail_take 8 retreat -> rail_set 8 approach | rail | 3.569 | 0.299 | 0.342 | 0.145 | 0.039 | 0.132 | 0.017 | 0.568 | 1.0000 | 3.569 | 0.568 |
| 21 | rail_set 8 retreat -> rail_take 9 approach | - | 3.569 | 0.232 | 0.345 | 0.132 | 0.053 | 0.097 | 0.017 | 0.570 | 1.0000 | 3.569 | 0.570 |
| 22 | rail_take 9 retreat -> rail_set 9 approach | rail | 3.900 | 0.256 | 0.367 | 0.148 | 0.036 | 0.122 | 0.017 | 0.551 | 1.0000 | 3.900 | 0.551 |
| 33 | rail_set 9 retreat -> module_take 0 approach | - | 7.432 | **1.250** | 0.781 | 0.438 | 0.127 | 0.307 | 0.112 | **1.013** | 1.1814 | 8.726 | 0.900 |
| 34 | module_take 0 retreat -> module_set 0 approach | module | 8.364 | 0.940 | 0.518 | 0.492 | 0.171 | 0.466 | 0.126 | 0.634 | 1.0220 | 8.541 | 0.900 |
| 49 | module_set 0 retreat -> module_take 1 approach | - | 6.447 | **1.117** | 0.735 | 0.254 | 0.120 | 0.315 | 0.093 | 0.590 | 1.1156 | 7.158 | 0.900 |
| 50 | module_take 1 retreat -> module_set 1 approach | module | 8.278 | 0.904 | 0.529 | 0.432 | 0.166 | 0.460 | 0.307 | 0.636 | 1.0023 | 8.297 | 0.900 |
| 63 | module_set 1 retreat -> module_take 2 approach | - | 7.494 | **1.034** | 0.616 | 0.258 | 0.115 | 0.176 | 0.272 | 0.855 | 1.0729 | 8.018 | 0.900 |
| 64 | module_take 2 retreat -> module_set 2 approach | module | 6.304 | 0.448 | 0.505 | 0.435 | 0.370 | 0.381 | 0.532 | 0.679 | 1.0000 | 6.304 | 0.679 |
| 77 | module_set 2 retreat -> module_take 3 approach | - | 4.430 | 0.938 | 0.792 | 0.311 | 0.229 | 0.225 | 0.115 | 0.911 | 1.0215 | 4.519 | 0.900 |
| 78 | module_take 3 retreat -> module_set 3 approach | module | 5.447 | **1.148** | 0.510 | 0.625 | 0.485 | 0.502 | 0.424 | 0.955 | 1.1313 | 6.123 | 0.900 |
| 91 | module_set 3 retreat -> module_take 4 approach | - | 4.986 | **1.188** | 0.718 | 0.350 | 0.309 | 0.274 | 0.254 | 0.868 | 1.1509 | 5.693 | 0.900 |
| 92 | module_take 4 retreat -> module_set 4 approach | module | 6.720 | 0.859 | 0.637 | 0.550 | 0.302 | 0.432 | 0.501 | 0.765 | 1.0000 | 6.720 | 0.859 |
| 106 | module_set 4 retreat -> module_take 5 approach | - | 4.689 | **1.100** | 0.716 | 0.170 | 0.196 | 0.192 | 0.078 | 0.657 | 1.1082 | 5.164 | 0.900 |
| 107 | module_take 5 retreat -> module_set 5 approach | module | 6.936 | 0.701 | 0.521 | 0.491 | 0.347 | 0.258 | 0.641 | 0.855 | 1.0000 | 6.936 | 0.855 |
| 120 | module_set 5 retreat -> module_take 6 approach | - | 8.249 | 0.836 | 0.588 | 0.392 | 0.226 | 0.234 | 0.193 | 0.667 | 1.0000 | 8.249 | 0.836 |
| 121 | module_take 6 retreat -> module_set 6 approach | module | 8.493 | 0.643 | 0.570 | 0.385 | 0.164 | 0.335 | 0.230 | 0.690 | 1.0000 | 8.493 | 0.690 |
| 134 | module_set 6 retreat -> module_take 7 approach | - | 6.857 | **1.020** | 0.867 | 0.445 | 0.077 | 0.145 | 0.107 | 0.818 | 1.0649 | 7.283 | 0.900 |
| 135 | module_take 7 retreat -> module_set 7 approach | module | 12.097 | 0.343 | 0.639 | 0.305 | 0.221 | 0.298 | 0.210 | 0.566 | 1.0000 | 12.097 | 0.639 |
| 150 | module_set 7 retreat -> module_take 8 approach | - | 6.977 | **1.097** | 0.708 | 0.364 | 0.149 | 0.142 | 0.058 | 0.610 | 1.1060 | 7.685 | 0.900 |
| 151 | module_take 8 retreat -> module_set 8 approach | module | 10.156 | 0.828 | 0.455 | 0.394 | 0.178 | 0.325 | 0.274 | 0.508 | 1.0000 | 10.156 | 0.828 |
| 166 | module_set 8 retreat -> wait | - | 5.067 | 0.093 | 0.287 | 0.146 | 0.005 | 0.102 | 0.034 | 0.512 | 1.0000 | 5.067 | 0.512 |

Of v16's 39 module arm moves, 8 demand more than joint 1 gives (bold), up to 1.250 on the
move to module_take 0 (step 33); every other axis stays at or under 1.0 except the track
on that move (1.013). Every move over the limit is an empty or loaded move between the
pallet and the bay, where the track and joint 1 accelerate together.

**The method against physics.** v16's record (module_take 3, steps 77 and 78), peak over
the step, physics then inverse dynamics:

| axis | take, physics | take, ID | set, physics | set, ID |
|---|---|---|---|---|
| joint 1 | 1.000 | 0.938 | 1.000 | 1.148 |
| joint 2 | 0.815 | 0.792 | 0.513 | 0.510 |
| joint 3 | 0.332 | 0.311 | 0.664 | 0.625 |
| joint 4 | 0.241 | 0.229 | 0.491 | 0.485 |
| joint 5 | 0.298 | 0.275 | 0.509 | 0.502 |
| joint 6 | 0.122 | 0.115 | 0.450 | 0.424 |
| track | 0.939 | 0.911 | 0.967 | 0.955 |

On every axis that does not saturate the two agree within 0.04, inverse dynamics below
physics by 1 to 7 %: physics adds the servos' feedback to what the planned motion asks.
The set's joint 1 asks 1.148 and physics clips it at 1.000 (and falls 9.3 mrad behind);
the take's asks 0.938 and physics still touches 1.000, in bursts of a few ms at the
acceleration peaks. So a fraction of 0.95 is not enough: with it (8 moves slowed, one bay
599.80 s) physics still reaches 1.000 on joint 1 in both steps.

**The bound.** The optional layout field `torque_limit_scale` (`src/layout.h`, 0.5 to 1):
after timing each module arm move the planner runs the same inverse dynamics, and a move
whose worst ratio r exceeds the fraction f is played at time scale k·sqrt(r / f), again
until it holds (rounded up to 1e-4; `Segment::time_scale`, written in the plan file only
when not 1, with the figure only when set). Only that move is slowed; the schedule is
placed again around it as any move. `TorqueLimit.*` in `//:plan_replay_test` tests it.

`flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_torque` is v16 with
`torque_limit_scale` 0.90, one bay planned whole and committed:

| Figure | v16 | v16 + 0.90 |
|---|---|---|
| moves found | 126 of 126 | 126 of 126 |
| module arm moves slowed | | 11, 5.80 s of motion added |
| `bay_planned_s` | 574.73 s | 601.33 s (+26.60 s) |
| modules phase | 316.04 s | 341.99 s |
| module arm moves over 0.90 / over 1.0 (ID) | 11 / 8 | 0 / 0 |
| physics, take 3: joint 1 peak, take / set | 1.000 / 1.000 | 0.984 / 0.971 |
| physics: track peak, take / set | 0.939 / 0.967 | 0.900 / 0.773 |
| physics: largest Cartesian error, take / set | 0.64 / 17.80 mm | 0.64 / 1.49 mm |
| physics: least clearance, take / set | 0.020177 / 0.025272 m | 0.020195 / 0.025245 m (against 0.015) |

The slowed moves add 5.80 s of motion but the bay 26.60 s, all of it in the modules
phase: the rest is the schedule placing the pin arms' blocks around the later sets (not
taken apart in this run). The price is honest in that the arm can now follow every move of
the take and set in physics, with joint 1 at 0.984 its tightest margin.

## The price of torque-feasible timing (run 047)

Run 045's 26.60 s, taken apart: a per-step diff of bay 0 of v16's committed plan and of
the run 045 plan of `..._fast_retreat_torque` (both before this run's change), along the
chain that ends the bay (pin arm C's last tool change, then bay out).

| where | slowed move or wait | seconds | who then waits longer, and why |
|---|---|---|---|
| module arm, steps 33, 34, 49, 50, 63, 77, 78, 91, 106, 134 | ten slowed moves before set 7 | +5.10 | the module arm works back to back, so every later step of it; pin arm C's module 7 waits for set 7 (its tool change "just in time for locate 7/0") |
| module arm, step 150 | the move to take 8 | +0.71 | set 8 and the module arm's end (+5.80 in all); no pin arm waits on it |
| pin arm C, step 140 (locate 7/2) | new wait "for pin_arm_b to pass" | +17.75 | pin arm B is only +3.45 s late (tied to set 4, +4.20, and its wait for A at step 105 fell from 2.50 to 1.25 s), so B's move to hole 5/1 (step 114) now passes where C stands at 7/2; B was placed first and C waits until it has gone |
| pin arm C, step 144 (locate 7/1) | new wait for pin arm B | +6.75 | B's swages of module 5 are now in C's way |
| pin arm C, step 158 (locate 8/1) | v16's 3.00 s wait for pin arm B gone | −3.00 | B itself now waits 22.25 s for C at locate 6/0 (step 122), off the chain |
| sum | | **26.60** | = 601.33 − 574.73 s |

So 5.80 s is the arm's own; the other 20.80 s is the schedule's ordering. The
schedule (`Planner::Schedule`) places one block at a time, the earliest-ready first,
each later block steering clear of every block placed before it. The slowed moves
shift the pin arms by different amounts (each is tied to a different module set), the
relative timing of B and C near modules 5 and 7 changes, and the one that is placed
first in the shared part of the bay holds it.

**The change.** With `torque_limit_scale` and at least one move slowed,
`Planner::ScheduleSlowed` schedules the bay three times: as before; with every move
unslowed, which gives an order of placing and a start for each block; and slowed in that
order, each block no earlier than its unslowed start plus the seconds that the slowed
moves starting before it (in the unslowed schedule) add. It keeps the shorter of the
first and the third. The moves, their paths and their time scales are the same, so the
module arm's motion (and the physics run) is the same; only the waits move. A plan
without the field is scheduled as before, byte for byte. Placing in the unslowed order
alone gave 597.33 s: the conflict is about relative timing, not order only.

| Figure | v16 | v16 + 0.90, run 045 | v16 + 0.90, run 047 | `_ma_split` | `_ma_split_torque` |
|---|---|---|---|---|---|
| `bay_planned_s` | 574.73 s | 601.33 s | **580.53 s** | 513.80 s | **520.98 s** |
| slowed moves, motion added | | 11, 5.80 s | 11, 5.80 s | | 12, 7.18 s |
| bay over the layout without the field | | +26.60 s (4.6x) | +5.80 s (1.0x) | | +7.18 s (1.0x) |
| module arm moves over 0.90 / over 1.0 (ID) | 11 / 8 | 0 / 0 | 0 / 0 | 12 / 7 | 0 / 0 |
| physics joint 1, worst take / set of the nine takes | 1.000 / 1.000 (take 3) | 0.984 / 0.971 (take 3) | 0.984 / 0.991 (0.984 / 0.971 on take 3) | | 0.971 / 0.991 |

`TorqueLimit.TheBayCostsNoMoreThanTheSlowedMotion` (in `//:plan_replay_test`) holds
both committed plans to the layout without the field plus the slowed motion.
`flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_torque` is run 044's
`_ma_split` with `torque_limit_scale` 0.90: one bay planned whole and committed, least
clearance 0.0160 m and arm to arm 0.0188 m (against 0.015), least joint margin 5.00°
(pin arms' joint 1, as `_ma_split`), the module arm's least 8.64°. In physics every one
of its nine takes counts; joint 1 peaks at 0.991 (take 0, the set), the track at 0.929
(take 3, the take).

## v19's record (run 048)

v19 is v18's layout with `torque_limit_scale` 0.90 and nothing else
(`flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque`). `path_tries` keeps
the quickest of its 8 paths as timed from the datasheet, unslowed (`QuickestPath` in
`PlanMove`, `src/layout_plan.cc`); the torque bound then slows only the kept path. So v19's paths
are v18's, and the two fields combine without a code change; a try that is quicker after slowing
is not looked for. Three bays planned whole: 378 of 378 moves, 26 of the module arm's 117 slowed
(14.30 s of motion), bays 502.55 / 507.79 / 496.04 s against v18's 499.54 / 502.53 / 493.59 s,
`cycle_s` 502.13 s against 498.55 s (+3.58 s). `torque_demand` over the three bays: v18 18 moves
over 1.0 (8 / 6 / 4, worst 1.249, joint 1, step 94), v19 none over 0.90 (worst 0.900).

`bazel run --config=opt //:run_physics -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque --write`
wrote `physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json`. The
record take is module 5's, as in v18 (module_take 7, plan steps 136 and 137).

| Figure (derived) | v19, module 5 | v18, module 5 (run 046) |
|---|---|---|
| counted, of one bay | 20.727562 s of 502.553095 s | 20.205074 s of 499.542691 s |
| `physics_share` | 0.041245 | 0.040447 |
| peak torque over its limit, joints 1-6, take | 0.973 0.795 0.341 0.084 0.408 0.199 | 1.000 0.887 0.398 0.109 0.489 0.243 |
| peak torque over its limit, joints 1-6, set | 0.420 0.597 0.315 0.124 0.335 0.200 | 0.420 0.597 0.315 0.124 0.335 0.200 |
| peak track force over its limit, take / set | 0.540 / 0.579 | 0.664 / 0.579 |
| least clearance in physics, take / set | 0.021487 m / 0.019261 m (plan 0.021487 / 0.019047), against 0.015 m | 0.021488 m / 0.021586 m |
| largest joint tracking error, take / set | 0.6200 / 0.8641 mrad | 2.4907 / 0.8641 mrad |
| largest Cartesian error, take / set | 0.6872 mm / 0.6994 mm | 5.8223 mm / 0.6994 mm |
| set-down offset after settling | 0.2314 mm, yaw 0.0004 deg (gate 3.0 mm, 0.15 deg) | 0.2315 mm, yaw 0.0004 deg |

Both steps count. The take's move (step 136) is slowed, and joint 1 no longer reaches its limit
(0.973); the set's move is not slowed and is v18's. The set's least clearance falls to 0.019261 m
(held module's flange to pin arm B's nose, during the set's work) because the slowed schedule
moves pin arm B relative to the module arm; it is above the 0.015 m margin. v19 is chosen
(run 048) for this record: its cycle time is one the arm can follow.

## v20's record (run 052)

v20 is v19's layout with `bay_out_when_clear` (run 050) and nothing else
(`flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out`): a bay
goes out once its release has ended and no arm comes within the clearance of it on its way, so pin
arm C's last tool change runs while the bay goes out. Three bays planned whole, with no code
change: 378 of 378 moves, 26 of the module arm's 117 slowed (14.30 s of motion, as in v19), bays
496.59 / 501.82 / 490.08 s against v19's 502.55 / 507.79 / 496.04 s, `cycle_s` 496.16 s against
502.13 s (-5.96 s). Each bay out starts at its release's end (456.59, 958.41 and 1448.49 s); the
next `tube_in` starts at the end of bay out, after every arm is free (C is free 5.96 s into each
bay out). The least distance of any arm to the bay going out is 0.2286 / 0.2115 / 0.2243 m.
`torque_demand --all-bays`: none over 0.90 (worst 0.900, joint 1, step 94).

`bazel run --config=opt //:run_physics -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out --write`
wrote `physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out.json`.
The record take is module 5's, as in v19 (module_take 7, plan steps 136 and 137), at the same
times (349.52 to 370.25 s): the bay out comes after it, so the take and set are v19's.

| Figure (derived) | v20, module 5 | v19, module 5 (run 048) |
|---|---|---|
| counted, of one bay | 20.727562 s of 496.589754 s | 20.727562 s of 502.553095 s |
| `physics_share` | 0.041740 | 0.041245 |
| peak torque over its limit, joints 1-6, take | 0.973 0.795 0.341 0.084 0.408 0.199 | 0.973 0.795 0.341 0.084 0.408 0.199 |
| peak torque over its limit, joints 1-6, set | 0.420 0.597 0.315 0.124 0.335 0.200 | 0.420 0.597 0.315 0.124 0.335 0.200 |
| peak track force over its limit, take / set | 0.540 / 0.579 | 0.540 / 0.579 |
| least clearance in physics, take / set | 0.021487 m / 0.019261 m (plan 0.021487 / 0.019047), against 0.015 m | 0.021487 m / 0.019261 m |
| largest joint tracking error, take / set | 0.6200 / 0.8641 mrad | 0.6200 / 0.8641 mrad |
| largest Cartesian error, take / set | 0.6872 mm / 0.6994 mm | 0.6872 mm / 0.6994 mm |
| set-down offset after settling | 0.2314 mm, yaw 0.0004 deg (gate 3.0 mm, 0.15 deg) | 0.2314 mm, yaw 0.0004 deg |

Both steps count. Only `physics_share` moves, because the same 20.73 s are counted against a
shorter bay. v20 is chosen (run 052): it is v19 quicker by the bay out alone, and every module arm
joint stays below its limit on the take and the set.

## v21's record (run 054)

v21 is v20's layout with an infeed unit of its own (run 053) and nothing else
(`flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed`): the
`tubes_in` port names the item `infeed_unit` and a zone that no longer shares with `bays_out`, so bay
b+1's tube comes in 0.45 m above its place over bay b going out, from the moment the holder is
empty. The ways overlap 38.41 s at each boundary; bay ends 496.59 / 960.13 / 1412.03 s, `cycle_s`
470.68 s against v20's 496.16 s (-25.48 s), `one_bay_s` 496.59 s as v20's. 378 of 378 moves, 26 of
the module arm's 117 slowed (14.30 s), as in v20. The unit and its carriage are described in
[`infeed-unit.md`](infeed-unit.md).

`bazel run --config=opt //:run_physics -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed --write`
wrote `physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed.json`.
The record take is module 5's, as in v20 (module_take 7, plan steps 136 and 137), at the same
times (349.52 to 370.25 s): bay 0 up to its bay out is v20's, and the first overlap starts at
458.18 s, after the set.

| Figure (derived) | v21, module 5 | v20, module 5 (run 052) |
|---|---|---|
| counted, of one bay | 20.727562 s of 496.589754 s | 20.727562 s of 496.589754 s |
| `physics_share` | 0.041740 | 0.041740 |
| peak torque over its limit, joints 1-6, take | 0.973 0.795 0.341 0.084 0.408 0.199 | 0.973 0.795 0.341 0.084 0.408 0.199 |
| peak torque over its limit, joints 1-6, set | 0.420 0.597 0.315 0.124 0.335 0.200 | 0.420 0.597 0.315 0.124 0.335 0.200 |
| peak track force over its limit, take / set | 0.540 / 0.579 | 0.540 / 0.579 |
| least clearance in physics, take / set | 0.021487 m / 0.019261 m (plan 0.021487 / 0.019047), against 0.015 m | 0.021487 m / 0.019261 m |
| largest joint tracking error, take / set | 0.6200 / 0.8641 mrad | 0.6200 / 0.8641 mrad |
| largest Cartesian error, take / set | 0.6872 mm / 0.6994 mm | 0.6872 mm / 0.6994 mm |
| set-down offset after settling | 0.2314 mm, yaw 0.0004 deg (gate 3.0 mm, 0.15 deg) | 0.2314 mm, yaw 0.0004 deg |

Both steps count, and every figure is v20's: `physics_share` is counted against `one_bay_s`, which
the overlap does not change (it shortens the cycle, not bay 0). v21 is chosen (run 054): every
module arm joint stays below its limit on the take and the set.
