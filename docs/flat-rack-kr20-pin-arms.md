# KUKA KR 20 pin arms for the 40 ft cell (run 031, on the branch only)

Run 031 replaced v10's three UR10e pin arms (`flat_rack_3_pin_arms_ordered`) with KUKA KR 20
R1810-2. The layout `layouts/flat_rack_3_kr20_pin_arms_ordered.json` on branch
`org/031-kuka-kr-20-pin-arms` passes every checker row (944 of 944). One bay does **not** plan:
`plan_layout --bays 1` found 90 of 126 moves. So the layout did not land on `main`. Run 031
landed only the checker's `maker` and `transport mass` rows.

## Base places tried (`check_layout`, failing rows of 948)

| Place | Rows failing | What fails |
|---|---|---|
| UR10e's: x −0.38, z 0.798, yaw 0, tube z 1.598 | 443 | reach 90 (0.005 to 0.044 m short: the hole sits 0.34 to 0.74 m from the shoulder at shoulder height, too close for a 0.78 + 0.87 m arm to fold to), clearance 120, approach 216 |
| x −0.2 / yaw 90 / yaw 180 | 446 / 404 / 452 | as above |
| tube z 1.8 (x −0.38 / −0.2) | 354 / 393 | reach solved; clearance and approach under the modules |
| tube z 2.0 (x −0.38 / −0.2 / −0.6) | 270 / 405 / 201 | the module arm's held module against its own link 2 (the tube 0.4 m higher than its base allows) |

## Probes (one pin arm over the whole track, `LAYOUT_CHECK_PROBE`, 16 seeds, every 0.1 m)

A KR 20 reaches a hole from 1.2 to 1.7 m **along** the tube, arm stretched under the bay. The
probes count how many of the 36 holes have a pose that passes (margin, clearance, both legs).
At x −0.75 the count is 2, at −0.6 13, at −0.45 32, at −0.35 36 and at −0.25 0. The far holes
of a rail (x −1.28, across the tube) are reached only from the side whose stretch has no holder
stand. The stand column blocks the arm's links (`holder_stand_*_column`). Moving each saddle
0.4 m toward −Y in its stretch and shortening it to 0.2 m (column 0.1 m along Y) opens more of
them.

## The candidate on the branch

- Pin arms at x −0.35, z 0.798, yaw 0, on one line. A works modules 0 to 3 (16 holes, track
  −5.3..−1.9) and bolts 0 to 4. B works modules 4 and 5 (8 holes, track −1.0..0.8, pin tool
  only, no bolts). C works modules 6 to 8 (12 holes, track 1.4..5.3) and bolts 5 to 9. Every
  hole has a stop (`stops.locate/pin/swage`, one per point) taken from the probes.
- Tool stands ride 1.3 m along the track from the base. A KR 20 cannot reach a stand beside
  its base, and the tool-change pose solved there rose into the modules. C's wait and tool
  change are at its +Y end, beyond the bay.
- Checker: 944 rows, 0 fail. Time model 438.45 s against v10's 416.1 s. Busy: module arm 227.3 s,
  A 243.3 s, B 96.8 s, C 202.8 s. Mass in transit 4,078.3 kg against 13,608 kg.

## Why one bay does not plan

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

## For a later run

- Give the holder stands a shape that the arm passes: off the tube's axis, or toward the
  deck's −X edge. Then every hole can be reached from both sides, and each arm works from one
  side. The probe (`tools/kr20/`) answers this in about 1 minute per variant.
- Or use two lines of track, so that a +Y-facing arm and a −Y-facing arm can overlap along Y.
- Or check `one_posture_per_module` for the KR 20 here (it searches every 0.1 m but needs one
  family at all four holes).
