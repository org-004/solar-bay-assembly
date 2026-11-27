# KUKA KR 20 pin arms for the 40 ft cell (runs 031, 034, 035 and 037)

Every arm KUKA (the owner, 2026-11-26). Run 031 replaced v10's three UR10e pin arms
(`flat_rack_3_pin_arms_ordered`) with KUKA KR 20 R1810-2 and found that one bay does not plan.
Run 034 found which change to the cell lets a KR 20 reach every hole of its modules from one
side, and planned one bay of the layout `flat_rack_1_kr20_pin_arm_ordered` whole. Run 035 let
several carriages ride one beam and split the holes over three KR 20 pin arms:
`flat_rack_3_kr20_pin_arms_shared_ordered`, one bay planned whole in 626.32 s. Run 037 planned its
three bays and scored it as v15 (`cycle_s` 612.40 s), and the same layout with run 036's faster
retreats as v16 (572.56 s).

## Three bays (run 037)

`plan_layout -- <name> --bays 3 --out ...`, layouts unchanged, both plans committed and replayed by
tests/plan_replay_test.cc (`Kr20.ThreeBaysOfTheSharedBeamLayoutPassTheVerifier` and
`Kr20.ThreeBaysOfTheSharedBeamLayoutWithFastRetreatsPassTheVerifier`), scored by
`score_layout --write` and registered with `tools/evolution/add.sh` as candidates; v10 stayed chosen until run 040 made v16
chosen (every arm KUKA, the owner's rule of 2026-11-26).

| | v15 `flat_rack_3_kr20_pin_arms_shared_ordered` | v16 `..._fast_retreat` |
|---|---:|---:|
| moves found | 378 of 378 | 378 of 378 |
| planning | 146.3 s | 121.9 s |
| bay 0 / 1 / 2 planned | 626.32 / 618.65 / 592.22 s | 574.73 / 578.31 / 564.63 s |
| `cycle_s` (three bays back to back / 3) | **612.40 s** | **572.56 s** |
| against v10 (554.55 s, UR10e pin arms) | +57.85 s (+10.4 %) | +18.01 s (+3.2 %) |
| against v14 (717.37 s, best all-KUKA before) | -104.97 s (-14.6 %) | -144.81 s (-20.2 %) |
| time model of a bay | 416.75 s | 416.75 s |
| gates | rows failed 0 of 944, clearance 0.016 m, mass 4,132.2 kg | the same |

v16 is v15 plus `retreat_legs` {0.5 m/s, 3.0 m/s^2} (run 036): it is registered against v15. Each
bay is one tube on the one rack, so bay 1's tube comes in only when bay 0 has gone out; bays 1
and 2 are shorter than bay 0 in v15: rails 163.57 s against bay 0's 174.70 s, and bay 2's modules
333.25 s against 355.65 s.

`tools/time/account.py plans/flat_rack_3_kr20_pin_arms_shared_ordered.json` (bay 0, the first bay):

| arm | moves | moving planned | model, same moves | excess | legs (approach+retreat) | process | tool changes | waiting | busy % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| module_arm | 39 | 205.6 | 130.3 | +75.4 | 93.2 | 95.0 | 0.0 | 232.5 | 63 |
| pin_arm_a | 39 | 110.0 | 69.4 | +40.7 | 57.2 | 162.0 | 16.0 | 281.1 | 55 |
| pin_arm_c | 25 | 60.9 | 36.8 | +24.2 | 33.4 | 98.0 | 16.0 | 418.0 | 33 |
| pin_arm_b | 23 | 71.9 | 39.5 | +32.4 | 37.5 | 100.0 | 0.0 | 416.9 | 33 |

The excess of bay 0 over the time model, by cause along its critical path: waits +209.83 s, legs
+72.23 s, moves slower than modelled +23.50 s, serialized work -95.99 s (total +209.57 s). The
largest remaining share is the waits on the path: pin arm C waits 209.8 s on it, 184.6 s of them
for the module arm to set module 7 (step 135); the module arm (63 % busy, 205.6 s of moves) sets
all ten rails (rails 174.70 s) and every module before C's two can start. The lever below v16 is
the module arm and the rails, as run 035 found, not the pin arms. With v16's retreats the waits
on bay 0's path are 175.09 s and the legs 55.37 s.

Margins of v15's three bays (`plan_layout --margins` over the committed plan): least clearance
0.0160 m against 0.015 (pin arm C link 3 to `holder_stand_8_saddle`, step 152); arm to arm
0.0199 m (A link 3 to B link 1, step 413); joint margin 5.00 degrees against 5 (A joint 1 at
step 36, B joint 1 at its wait pose); track ends 0 slack (A at -5.700, the module arm at -3.600
and 5.500); carriages on the shared beam at least 0.495 m apart (B and C, centres 0.995 m, the
carriages 0.5 m long; A and B 0.600 m). v16: clearance 0.0160 m, arm to arm 0.0184 m (module
arm's cups to C link 3), joint 5.00 degrees, track ends 0 slack, B and C 0.495 m apart.

## Run 035: three KR 20 on one shared beam, one bay planned whole

### The shared-beam rule (`Track::shares_beam_with`, src/layout.h; src/layout_scene.cc)

An arm's track may name another arm whose beam its carriage also rides (`"shares_beam_with":
"pin_arm_a"`; the named arm has a track of the same direction and rides no other's beam). In
`LayoutScene::Obstacles` the carriage (and the base fixed to it) is then skipped against the
beams of every arm of its group, as against its own; carriage to carriage, every link to every
beam, and waiting tools stay counted. Layouts without the field are unchanged (`BeamArm` of every
arm is the arm itself). Test: `LayoutScene.CarriagesOnASharedBeam` in tests/layout_scene_test.cc
(two carriages 3 m apart on one beam pass and, without the field, fail against the other arm's
beam as 034's `k3a` did; carriages 0.3 m apart go into each other; a link of B against A's beam,
raised through it, still counts). B and C draw the same beam box as A, one beam from y -6.0 to
5.3; their `mass_kg` is their carriage only (21.94 kg/m over 1.1 m).

### The layout (`tools/kr20/shared.py`; its arguments are in tools/kr20/README.md)

| Arm | Holes (modules) | Strap bolts | Track | Wait | Tools wait | Magazine nest |
|---|---|---|---|---|---|---|
| pin_arm_a | 0-15 (0-3) | 0-4 | -5.7..-0.4 | -5.7, 034's pose | 1.3 m toward -Y | y -5.2, stop -5.2 |
| pin_arm_b | 16-27 (4-6) | none: pin tool only | -1.5..2.9 | 1.3, stretched low toward -X | none | y -2.6, stop -1.0 |
| pin_arm_c | 28-35 (7-8) | 5-9 (5 and 6 from +Y, stop 2.3) | 1.9..5.0 | 4.6, at its tool-change pose | 1.6 m toward +Y | y 6.0, stop 4.4 |

Every hole from -Y at 034's stops. What it took, each found by checker or planner:
- The middle arm cannot bolt: an arm parks at its wait pose after its last bolt in the planner's
  world with all nine modules on, and every middle last-bolt pose tried (rails 3, 5 and 6, about 25 stops) has a
  link up where a module will be. So B has the pin tool only (no tool change); A bolts 0-4 (its
  bolt-4 pose is low) and C bolts 5-9, rails 5 and 6 from +Y of the rail so that C stays clear of
  B's wait.
- Wait poses under the bay must be low: 034's wait pose and the stow pose reach into the modules.
  B waits stretched low toward -X; C waits at its tool-change pose, so its tool changes are short.
- Pin magazine takes must be low and away from the module arm's corridors (it takes every module
  at track 0 and every rail at 1.75): a nest beside the base puts link 2 into a module; B's nest is
  1.6 m behind its stop, C's 1.6 m ahead. B takes its magazine just in time (`order.pin_tools`); taken
  early, B stood at its retreat and blocked the module arm (schedule blocked).
- Track room: the planner runs a carriage past its stops between two holes (034's move from 3/0 to
  3/2 runs to -2.5), so B's and C's tracks reach 0.4 m beyond their stops.
- Arm order in the file: a, c, b. With b listed before c the checker solved C's bolt-9 and magazine
  poses in another branch, 0.0097 and 0.0143 m into modules 8 and 7 in the planner's world.

### Figures

- `check_layout`: 944 rows, 0 fail. Time model 416.75 s (034: 637.46 s; v10's 416.1 s).
- `plan_layout --bays 1`: 126 of 126 moves, **626.32 s** (034: 897.90 s, -30.2 %; v10: 560.45 s,
  +11.8 %). Rails 174.70 s, modules 355.65 s, end 50.97 s. Least clearance 0.0160 m, arm to arm
  0.0206 m, joint margin 5.00 degrees. Mass in transit 4,135.2 kg against 13,608 kg.
- Busy share of the bay: module arm 63 % (393.8 s), pin arm A 55 % (345.2 s: work 178.0, moves
  110.0, legs 57.2), B 33 % (209.4 s: 100.0, 71.9, 37.5), C 33 % (208.3 s: 114.0, 60.9, 33.4).
  The module arm is now the longest; the rail phase (174.70 s) is 034's.
- Replayed by `Kr20.OneBayOfTheSharedBeamLayoutPassesTheVerifier` (tests/plan_replay_test.cc);
  exempted from the scorecards by name until it has three bays, as 034's. Run 037 gave it three
  bays: the test is now `Kr20.ThreeBaysOfTheSharedBeamLayoutPassTheVerifier` and the layout has a
  scorecard.

## Run 034: one KR 20 along the whole bay, one bay planned whole

### Probes (`tools/kr20/probe.sh`: one pin arm over the whole track, every hole, every 0.1 m)

A hole counts on a side when some pose with the base at that side of the hole passes margin
(5 degrees), clearance (0.015 m) and both legs. "-Y" is the base at smaller y than the hole.

| Cell | -Y | +Y | Note |
|---|---|---|---|
| 031's candidate (stands 0.4 m toward -Y) | 32 | 25 | +Y misses the far holes of rails 1, 3, 5, 7 (the stand columns); -Y misses rail 0 (base off the deck) and rail 1's odd holes |
| (a) stand columns removed (bound) | 32 | 33 | the columns are what blocks +Y; rail 0 still not from -Y, rail 9 not from +Y |
| (a) columns at the deck's -X edge, an arm to each saddle | 32 | 25 | the arm under the saddle blocks as the column did |
| (a) fewer inner stands | - | - | any stand in the stretch between rails r and r+1 blocks +Y to rail r; one stand per side is what the pattern above gives, not a fix |
| (b) a second line of track | - | - | 031: at x -0.6 13 of 36, at -0.25 0; a second line cannot stand within the 0.1 m band (x -0.35 to -0.45) where a KR 20 reaches |
| (c) `one_posture_per_module` | - | - | chooses among the poses above; it adds no side, so it cannot give one side per arm |
| bay 1.0 m toward +Y | 35 | 21 | only hole 0/0 short: the end unit's column (link 3) |
| bay 1.0 m toward +Y, end unit removed (bound) | 36 | 21 | |
| **bay 1.0 m toward +Y, end unit's column 0.7 m beyond the plain end, bar at axis height** | **36** | 22 | **chosen** |
| mirrored: bay 0.95 m toward -Y, end unit at the +Y end | 22 | 28 | +Y still misses the far holes of rails 1, 3, 5, 7 |

### Why one arm, not three

On one line of track arms cannot pass. The end rails force the sides: rail 0 needs the base at
-Y beyond the deck unless the bay moves toward +Y, rail 9 the same toward -Y. With the bay moved
1.0 m toward +Y every hole is reached from -Y, so every arm on the line faces +Y. Two such
neighbours share a rail at their boundary: the lower arm's highest stop is at least y - 1.474
and the upper arm's lowest at most y - 1.526 for that rail's holes, so their stretches overlap
by 0.05 m plus a carriage (0.5 m). The checker counts a carriage on another arm's beam as a
collision (`layout_scene.cc`: only an arm's own beam is near by design), so three arms
(`k3a`, 125 of 944 rows fail, all carriage to beam) do not pass. Facing each other they would
have to pass. One KR 20 does the 36 holes and the 10 strap bolts.

Two stops were found by planning: the arm's last strap bolt (rail 9) is planned to its wait
pose with all nine modules in the world, and only a bolt-9 stop at y 4.45 leaves the pose clear
of module 8 (4.4 to 5.5 tried); with bolt 8 at 3.8 to 4.6 the arm waits at its bolt-8 retreat in
the way of the module arm setting rail 9 and the schedule is blocked; at 3.3 it is not.

### The layout and its plan

- Bay 1.0 m toward +Y (tube plain end at y -4.308; the other end 0.21 m beyond the deck). The
  module arm's track -3.6 to 5.5, its set stops moved with the bay (at most 5.5).
- End unit: column 0.7 m beyond the plain end at the deck's -X edge (x -1.185), its head on a
  bar at the tube's axis height. Stands and saddles as 031.
- One KR 20 R1810-2 at x -0.35, z 0.798, track -5.7 to 5.5; every hole from -Y, stops from the
  probe; tools ride 1.3 m toward -Y on its carriage.
- `check_layout`: 906 rows, 0 fail. Time model 637.46 s.
- `plan_layout --bays 1`: 118 of 118 moves, **897.90 s** (v10 560.45 s, +60.2 %; time model
  637.46 s, +40.9 %). Rails 175.45 s, modules 616.83 s, end 60.62 s. Least clearance 0.0160 m,
  arm to arm 0.0216 m, joint margin 5.00 degrees.
- Busy share of the bay: module arm 44 % (392.6 s), pin arm 80 % (722.3 s: work 368.0, moves
  230.0, legs 124.4). The one pin arm is the bottleneck.

### For a later run

- Three KR 20 on one rail need a carriage to ride another arm's beam: one beam drawn for the
  line, carriages near by design to it, the schedule keeping them apart. That is a change to
  `LayoutScene` (an opt-in field on the track), not to the cell.
- Or stand columns the arm passes from +Y (a saddle hung from a beam along the tube), so that
  arms can face each other with a gap between them.

## Run 031: three KR 20 on one line, 90 of 126 moves

### Base places tried (`check_layout`, failing rows of 948)

| Place | Rows failing | What fails |
|---|---|---|
| UR10e's: x −0.38, z 0.798, yaw 0, tube z 1.598 | 443 | reach 90 (0.005 to 0.044 m short: the hole sits 0.34 to 0.74 m from the shoulder at shoulder height, too close for a 0.78 + 0.87 m arm to fold to), clearance 120, approach 216 |
| x −0.2 / yaw 90 / yaw 180 | 446 / 404 / 452 | as above |
| tube z 1.8 (x −0.38 / −0.2) | 354 / 393 | reach solved; clearance and approach under the modules |
| tube z 2.0 (x −0.38 / −0.2 / −0.6) | 270 / 405 / 201 | the module arm's held module against its own link 2 (the tube 0.4 m higher than its base allows) |

### Probes (one pin arm over the whole track, `LAYOUT_CHECK_PROBE`, 16 seeds, every 0.1 m)

A KR 20 reaches a hole from 1.2 to 1.7 m **along** the tube, arm stretched under the bay. The
probes count how many of the 36 holes have a pose that passes (margin, clearance, both legs).
At x −0.75 the count is 2, at −0.6 13, at −0.45 32, at −0.35 36 and at −0.25 0. The far holes
of a rail (x −1.28, across the tube) are reached only from the side whose stretch has no holder
stand. The stand column blocks the arm's links (`holder_stand_*_column`). Moving each saddle
0.4 m toward −Y in its stretch and shortening it to 0.2 m (column 0.1 m along Y) opens more of
them.

### The candidate on the branch

- Pin arms at x −0.35, z 0.798, yaw 0, on one line. A works modules 0 to 3 (16 holes, track
  −5.3..−1.9) and bolts 0 to 4. B works modules 4 and 5 (8 holes, track −1.0..0.8, pin tool
  only, no bolts). C works modules 6 to 8 (12 holes, track 1.4..5.3) and bolts 5 to 9. Every
  hole has a stop (`stops.locate/pin/swage`, one per point) taken from the probes.
- Tool stands ride 1.3 m along the track from the base. A KR 20 cannot reach a stand beside
  its base, and the tool-change pose solved there rose into the modules. C's wait and tool
  change are at its +Y end, beyond the bay.
- Checker: 944 rows, 0 fail. Time model 438.45 s against v10's 416.1 s. Busy: module arm 227.3 s,
  A 243.3 s, B 96.8 s, C 202.8 s. Mass in transit 4,078.3 kg against 13,608 kg.

### Why one bay does not plan

The planner (`plan_layout -- flat_rack_3_kr20_pin_arms_ordered --bays 1 --summary`, 109 s)
found 90 of 126 moves. Most of the 36 that failed are moves between two holes of one module
(160,000 iterations each). From the probes, the holes of a rail next to a stand can be reached
from one side along Y only, so one module's holes need the arm pointing +Y for some and −Y for
others. That is a swing of the whole arm under the bay, the swing run 028 met on the two-rack
cell. The end modules force the sides: module 0's rail 0 can be reached only from +Y (a base at
−6.4 would be off the deck), and module 8's rail 9 only from −Y. Two arms that face each other
cannot pass on one line of track, so no split of whole modules among arms on one line gives
every arm a single side. The pins-take poses of A and B were also not free with the modules on
(the magazine is taken after the first set).

