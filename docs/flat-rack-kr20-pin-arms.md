# KUKA KR 20 pin arms for the 40 ft cell (runs 031 and 034)

Every arm KUKA (the owner, 2026-11-26). Run 031 replaced v10's three UR10e pin arms
(`flat_rack_3_pin_arms_ordered`) with KUKA KR 20 R1810-2 and found that one bay does not plan.
Run 034 found which change to the cell lets a KR 20 reach every hole of its modules from one
side, and planned one bay of the layout `flat_rack_1_kr20_pin_arm_ordered` whole.

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

