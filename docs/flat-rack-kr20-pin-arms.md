# KUKA KR 20 pin arms for the 40 ft cell (runs 031, 034, 035, 037, 041 and 044)

Every arm KUKA (the owner, 2026-11-26). Run 031 replaced v10's three UR10e pin arms
(`flat_rack_3_pin_arms_ordered`) with KUKA KR 20 R1810-2 and found that one bay does not plan.
Run 034 found which change to the cell lets a KR 20 reach every hole of its modules from one
side, and planned one bay of the layout `flat_rack_1_kr20_pin_arm_ordered` whole. Run 035 let
several carriages ride one beam and split the holes over three KR 20 pin arms:
`flat_rack_3_kr20_pin_arms_shared_ordered`, one bay planned whole in 626.32 s. Run 037 planned its
three bays and scored it as v15 (`cycle_s` 612.40 s), and the same layout with run 036's faster
retreats as v16 (572.56 s). Run 041 set the modules in turns of the three pin arms:
`flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma`, one bay planned whole in 543.96 s. Run 044
gave each pin arm three modules: `flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split`,
one bay planned whole in 513.80 s.

## Pin arm A's chain (run 044)

Run 041's `_ma` planned one bay in 543.96 s, and its critical path ended in pin arm A's chain:
four modules (0 to 3) and five bolts, A busy from 240.75 s to 503.96 s with no wait over 0.63 s.
This run moves one module off A and one off B, three modules to each pin arm, in the layout
`flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split`: one bay planned whole, 126 of
126 moves, in **513.80 s** (`_ma` 543.96 s, v16 574.73 s, v10 560.45 s).

### `_ma`'s bay 0 from the pin arms' first modules to bay out

The module arm's sets end at (module: s) 0: 224.11, 4: 246.93, 7: 270.23, 1: 294.58, 5: 316.15,
8: 342.41, 2: 365.89, 6: 388.34, **3: 409.35** (its last set, 398.18 to 409.35); bay out 503.96 to
543.96. Bolts: A drove bolts 0 to 4 from 63.02 to 138.34 s (28.20 s of waits for the rails), C
bolts 5 to 9 from 155.26 to 208.26 s; both then waited for their first module (A 79.26 s, C
60.27 s) and changed to the pin tool (8 s) just in time. A module is its 12 steps (locate, pin and
swage of four holes); its span runs from the end of the arm's step before it.

| arm | work | start s | end s | span s | wait before it, s (for) |
|---|---|---:|---:|---:|---|
| A | take a pin magazine | 233.26 | 240.75 | 7.49 | 0 |
| A | module 0 | 240.75 | 302.42 | 61.67 | 0 |
| A | module 1 | 302.42 | 364.71 | 62.29 | 0 |
| A | module 2 | 365.34 | 424.28 | 58.94 | 0.63 (the set of module 2) |
| A | module 3 | 424.28 | 486.63 | 62.35 | 0 (set at 409.35: waits 14.93 s for A) |
| A | back to wait | 486.63 | 492.56 | 5.92 | 0 |
| A | tool change (to the nutrunner) | 492.56 | **503.96** | 11.40 | 0: bay out starts here |
| B | take a pin magazine | 246.38 | 257.68 | 11.30 | 246.38 (the set of module 4) |
| B | module 4 | 257.93 | 321.95 | 64.02 | 0.25 (the module arm to pass) |
| B | module 5 | 321.95 | 387.26 | 65.31 | 0 |
| B | module 6 | 387.79 | 448.39 | 60.60 | 0.53 (the set of module 6) |
| B | back to wait | 450.39 | 455.34 | 4.95 | 2.00 (A to pass) |
| C | take a pin magazine | 278.04 | 284.58 | 6.54 | 0 |
| C | module 7 | 284.58 | 360.42 | 75.85 | 0 (11.25 s of waits inside) |
| C | module 8 | 360.42 | 423.07 | 62.64 | 0 (4.50 s inside) |
| C | back to wait, tool change | 423.07 | 434.39 | 11.32 | 0 |

### Lower bounds

A module takes a pin arm 58.94 to 65.31 s (`_ma`), whoever pins it. A pin arm's bound for its
share: the start of its first module (its first set, then its pin tool and magazine) + its
modules at the fastest 58.94 s + its way back (A and C also change back to the nutrunner) + bay
out's 40 s.

| arm | share in `_ma` | first module from s | bound for the bay s | as planned s |
|---|---|---:|---:|---:|
| A | modules 0-3, bolts 0-4 | 240.75 | 240.75 + 4 x 58.94 + 17.32 + 40 = **533.83** | 543.96 |
| B | modules 4-6 | 257.68 | 257.68 + 3 x 58.94 + 4.95 + 40 = 479.45 | (455.34 + 40) |
| C | modules 7-8, bolts 5-9 | 284.58 | 284.58 + 2 x 58.94 + 11.32 + 40 = 453.78 | (434.39 + 40) |
| A with 3 modules (0-2) | | 240.75 | 240.75 + 3 x 58.94 + 17.32 + 40 = 474.89 | |
| module arm | the last set | 409.35 | 409.35 + 58.94 + 4.95 + 40 = 513.24 | |

With four modules A cannot plan below 533.83 s whatever the order: four modules need at least
three sets between each two of them to keep A fed, and nine sets have no room for that. With
three modules to each arm, set in turns, the bound is the module arm's last set plus one module.

Which modules B and C can take from the shared beam, by the checker (`check_layout`, 944 rows, on
copies of `_ma` with only the split, the order and B's wait track changed):

| probe (A / B / C, B's wait track) | rows passed | what fails |
|---|---:|---|
| 0-1 / 2-5 / 6-8, 1.3 | 926 of 944 | C at module 6's holes 24-25 (stops 1.4, 1.2) against B's base at its wait (1.3) |
| 0-2 / 3-5 / 6-8, 1.3 | 926 of 944 | the same |
| 0-2 / 3-5 / 6-8, -0.3 | 943 of 944 | B's wait pose: link 3 to `holder_stand_4_column` |
| 0-2 / 3-5 / 6-8, 0.3 | 942 of 944 | A's strap bolt 4 legs: link 5 to B's link 1 |
| **0-2 / 3-5 / 6-8, 0.6** | **944 of 944** | nothing |
| 0 / 1-5 / 6-8, 0.6 | 944 of 944 | nothing: B reaches modules 1 to 6 |
| 0-2 / 3-4 / 5-8, 1.3 | 890 of 944 | C at modules 5 and 6 against B |
| 0-2 / 3-4 / 5-8, -0.3 | 934 of 944 | C at hole 21 against B's base; B's wait pose |
| 0-2 / 3 / 4-8, -0.3 and -0.8 | 898, 901 of 944 | C at module 4: link 2 to module 3's laminate (-0.016 m), B's carriage; A against B |

So B reaches modules 1 to 6 and C modules 6 to 8 with every row passing (C's module 5 failed only
against B's wait pose in the two waits tried, module 4 against module 3 too). The move that gains
most: **module 3 from A to B and module 6 from B to C**, three modules to each pin arm, B waiting at
track 0.6 (its stow track too) so that C can stand at module 6's stops.

### The layout and its plan

`_ma` with: A holes 0-11 (track -5.7 to -0.8), B holes 12-23 (track -2.7 to 1.9, wait and stow
0.6), C holes 24-35 (track 0.8 to 5.0), bolts as before; `order.modules` 0, 3, 6, 1, 4, 7, 2, 5,
8 (turns of A, B and C). The stops are run 034's per hole, as before; the beam, the arms' order in
the file (A, C, B), `pin_tools: just_in_time` and `retreat_legs` are unchanged; no `path_tries`.
944 of 944 checker rows. Probes, one bay each with `--summary`:

| order.modules | bay_planned_s | path ends in |
|---|---:|---|
| 0, 6, 3, 1, 7, 4, 2, 8, 5 (turns of A, C, B) | 536.17 | B: module 3 waits 13.0 s for the module arm, back to wait 12.39 s |
| **0, 3, 6, 1, 4, 7, 2, 5, 8** (turns of A, B, C) | **513.80** | the module arm (last set 405.46 s), then C's module 8 (57.58 s) and its tool change |
| the same with `path_tries: 8` (probe, not committed; planning 172.1 s against 50.6 s) | 499.54 | 126 of 126 moves; least clearance 0.0160 m, arm to arm 0.0188 m, joint 5.00° |

Sets end at 0: 224.11, 3: 244.92, 6: 265.49, 1: 290.15, 4: 309.86, 7: 334.34, 2: 358.67,
5: 380.97, 8: 405.46. Pin arms busy: A 50 %, B 39 %, C 48 % (module arm 68 %); A ends at 440.39 s,
B 472.35 s, C 473.80 s. The critical path is the module arm to 404.91 s, then pin arm C's module 8
and its change back to the nutrunner (68.89 s), then bay out. The one-bay plan is committed and
replayed by `Kr20.OneBayOfTheSharedBeamLayoutWithThreeModulesToEachPinArmPassesTheVerifier`.

Margins of the one-bay plan (`plan_layout -- <name> --margins`, exit 0): least clearance 0.0160 m
against 0.015 (C link 3 to `holder_stand_8_saddle`); arm to arm 0.0188 m (the module arm's held
module to B's pin tool nose); least joint margin 5.00° against 5° (A joint 1 at its pin magazine
take, step 36; B joint 1 at its wait pose); track ends 0 slack (module arm -3.600 / 5.500, A
-5.700); carriage to carriage on the shared beam at least 0.898 m (A to B, centres 1.398 m apart
less the 0.5 m carriage; B to C 1.132 m), from the plan's track samples.

## The module arm on the path (run 041)

v16's bay 0 (`plans/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json`, 574.73 s) has on
its critical path the module arm's ten rails (45.00 to 199.12 s), then pin arm C, which waits
172.1 s (209.41 to 381.50 s) for the module arm's set of module 7, then works modules 7 and 8
(396.40 to 523.41 s) and bay out. The module arm's steps from the end of the rails to the set of
module 7, from the committed plan (planned: the step's span; model: the time model's move plus
the work; legs are not timed by the model):

| step | what | start s | end s | planned s | model s | move planned / model s | legs s | work s | waits for | planned - model |
|---:|---|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 22 | set rail 9 | 199.12 | 199.67 | 0.55 | 0.00 | 0.00 / 0.00 | 0.55 | 0.00 | nothing | +0.55 |
| 33 | take module 0 | 199.67 | 211.25 | 11.58 | 6.97 | 7.43 / 4.97 | 2.15 | 2.00 | nothing | +4.61 |
| 34 | set module 0 | 211.25 | 224.11 | 12.86 | 6.68 | 8.36 / 3.68 | 1.50 | 3.00 | nothing | +6.18 |
| 49 | take module 1 | 224.11 | 234.70 | 10.60 | 5.68 | 6.45 / 3.68 | 2.15 | 2.00 | nothing | +4.91 |
| 50 | set module 1 | 234.70 | 247.48 | 12.77 | 5.92 | 8.28 / 2.92 | 1.50 | 3.00 | nothing | +6.85 |
| 63 | take module 2 | 247.48 | 259.12 | 11.64 | 4.92 | 7.49 / 2.92 | 2.15 | 2.00 | nothing | +6.72 |
| 64 | set module 2 | 259.12 | 269.92 | 10.80 | 5.55 | 6.30 / 2.55 | 1.50 | 3.00 | nothing | +5.25 |
| 77 | take module 3 | 269.92 | 278.50 | 8.58 | 4.55 | 4.43 / 2.55 | 2.15 | 2.00 | nothing | +4.03 |
| 78 | set module 3 | 278.50 | 288.44 | 9.94 | 5.55 | 5.45 / 2.55 | 1.50 | 3.00 | nothing | +4.39 |
| 91 | take module 4 | 288.44 | 297.58 | 9.13 | 4.55 | 4.99 / 2.55 | 2.15 | 2.00 | nothing | +4.58 |
| 92 | set module 4 | 297.58 | 308.80 | 11.22 | 5.55 | 6.72 / 2.55 | 1.50 | 3.00 | nothing | +5.67 |
| 106 | take module 5 | 308.80 | 317.63 | 8.84 | 4.55 | 4.69 / 2.55 | 2.15 | 2.00 | nothing | +4.29 |
| 107 | set module 5 | 317.63 | 329.07 | 11.43 | 5.73 | 6.94 / 2.73 | 1.50 | 3.00 | nothing | +5.70 |
| 120 | take module 6 | 329.07 | 341.46 | 12.40 | 4.73 | 8.25 / 2.73 | 2.15 | 2.00 | nothing | +7.67 |
| 121 | set module 6 | 341.46 | 354.45 | 12.99 | 6.49 | 8.49 / 3.49 | 1.50 | 3.00 | nothing | +6.50 |
| 134 | take module 7 | 354.45 | 365.46 | 11.01 | 5.49 | 6.86 / 3.49 | 2.15 | 2.00 | nothing | +5.51 |
| 135 | set module 7 | 365.46 | 382.05 | 16.59 | 7.25 | 12.10 / 4.25 | 1.50 | 3.00 | nothing | +9.34 |
| | all | 199.12 | 382.06 | 182.93 | 90.17 | 113.22 / 50.17 | 29.71 | 40.00 | waits 0.00 | +92.76 |

The module arm waits for nothing in this stretch: it is busy from the last rail to module 8's set
(412.90 s), and every step is slower than its model by the same three causes: its moves (113.22 s
against 50.17 s, +63.05 s; paths with corners around the pallet and the bay, the planner's timing,
run 042's lever), its approach and retreat legs (29.71 s, not in the model; retreats already at
run 036's 0.5 m/s), and the settle of 0.30 s per move inside the span. None of these is a layout
field. What a layout field does govern is *which* modules come last: `order.modules` was 0 to 8,
so pin arm C's two modules (7 and 8, holes 28-35) were the last two sets, C idled from its last
bolt to 381.50 s, and its two modules (about 66 s each) ran after the module arm was done. That
wait, 175.1 s on the path, is the largest cause a layout field removes.

The module arm's own lower bound for a bay is its moves, legs and process summed: 205.6 + 67.3 +
95.0 = 367.9 s (452.9 s with tube in, clamp and bay out), against 574.73 s. A pin arm's work on
its last module comes after the last set, so a bay cannot end before about 413 + 62 + 17 + 40 ≈
532 s with this arm and this split of holes.

Probes (v16's file with `order.modules` changed, one bay, `check_layout` 944 of 944 rows each):

| order.modules | pin_tools | moves found | bay_planned_s | path after the module arm |
|---|---|---:|---:|---|
| 0, 1, 2, 3, 4, 5, 6, 7, 8 (v16) | just_in_time | 126 / 126 | 574.73 | C, modules 7 and 8 |
| 0, 7, 4, 1, 8, 5, 2, 6, 3 | just_in_time | 126 / 126 | 546.44 | A, modules 2 and 3 |
| 0, 7, 4, 1, 8, 5, 2, 6, 3 | early | blocked | - | schedule blocked at set rail 0 (B's magazine take, as in run 035) |
| 0, 7, 4, 1, 2, 8, 5, 6, 3 | just_in_time | 126 / 126 | 544.11 | A |
| 0, 7, 1, 4, 8, 2, 5, 3, 6 | just_in_time | 126 / 126 | 557.59 | B |
| **0, 4, 7, 1, 5, 8, 2, 6, 3** | just_in_time | 126 / 126 | **543.96** | A, modules 2 and 3 |

The last is `layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma.json` (v16's file
otherwise unchanged; its plan of one bay is `plans/` of the same name, replayed by
`Kr20.OneBayOfTheSharedBeamLayoutWithTheModulesInTurnsPassesTheVerifier`). One bay: 126 of 126
moves, **543.96 s** (v16 574.73 s, v10 560.45 s), time model 416.75 s; planning 36 s. Pin arm C
now waits 60.3 s for its first module and is back at its wait pose with its first tool at 434.4 s; pin arm A, with four modules
and five bolts, is busy without a wait longer than 0.63 s from 240.75 s to 503.96 s and is the
path's tail. Below this the lever is A's share: four modules after the rails (about 62 s each), or
the module arm taking a module before the last rail.

Margins of the one-bay plan (`plan_layout -- <name> --margins`, exit 0): least clearance 0.0160 m
against 0.015 (C link 3 to `holder_stand_8_saddle`); arm to arm 0.0194 m (the module arm's held
module to A's pin tool nose); least joint margin 5.00° against 5° (A joint 1 at its first pin
magazine take; B at its wait pose); track ends 0 slack (module arm −3.600 / 5.500, A −5.700);
carriage to carriage on the shared beam at least 0.495 m (B to C, centres 0.995 m apart less the
0.5 m carriage).

## Three bays (run 037)

`plan_layout -- <name> --bays 3 --out ...`, layouts unchanged, both plans committed and replayed by
tests/plan_replay_test.cc (`Kr20.ThreeBaysOfTheSharedBeamLayoutPassTheVerifier` and
`Kr20.ThreeBaysOfTheSharedBeamLayoutWithFastRetreatsPassTheVerifier`), scored by
`score_layout --write` and registered with `tools/evolution/add.sh` as candidates; v10 stayed chosen until run 040 made v16
chosen (every arm KUKA, the owner's rule of 2026-11-26), and v16 until run 043 made v17 chosen (run 041's module
order and run 042's `path_tries` 8 together, three bays 529.20 / 529.80 / 523.92 s, cycle 527.64 s), and v17 until run
046 made v18 chosen (run 044's three modules to each pin arm with `path_tries` 8, three bays 499.54 / 502.53 / 493.59 s,
cycle 498.55 s), and v18 until run 048 made v19 chosen (v18 with `torque_limit_scale` 0.90, its
module arm within 0.90 of its motors, three bays 502.55 / 507.79 / 496.04 s, cycle 502.13 s).

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

## Run 042: each move the quickest of 8 searched paths (`path_tries`)

Run 038 found the two-rack bay's short pin arm moves slow at their corners. On v16 the slow
multi-piece pin arm moves are not corner cases: their straight line is blocked from its first
centimetres (C's links 3 and 5 against `holder_stand_8_column`, the tool's nose against
`bay_rail_8_strap_*`), and the one path the search found goes the long way round, 2 to 11 times
the straight line in weighted length. Shortening (200 random shortcuts, then every waypoint whose
neighbours see each other dropped) only pulls that way round taut; every waypoint it leaves is one
whose neighbours cannot see each other at the planning clearance (0.018 m). Blending the corners
looser buys little, and a looser blend strays farther than the 3 mm the plan keeps over the
checker's clearance.

The five slowest multi-piece pin arm moves on v16's bay-0 critical path (all pin arm C) and the
two slowest of fast_b's (pin arm B), each timed as planned (corners blended within 0.002 rad), with
every corner a stop, blended within 0.005 and 0.010 rad, and as the straight line (which is not
free); the last column is the move in the plan with `path_tries` 8. Length is the planner's
weighted joint-space length against the straight line's.

| layout | step | move | pieces | length / straight | planned s | all stops s | blend 0.005 s | blend 0.010 s | straight s (blocked by) | model s | quickest of 8: pieces, s |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| v16 | 149 | swage 7/2 → swage 7/0 | 7 | 11.29 | 5.92 | 5.99 | 5.76 | 5.54 | 1.59 (0.203 m) | 1.78 | 7, 5.14 |
| v16 | 152 | swage 7/0 → locate 8/0 | 7 | 4.06 | 4.88 | 4.89 | 4.70 | 4.49 | 1.45 (0.230 m) | 1.65 | 6, 4.05 |
| v16 | 138 | pins_take 2 → locate 7/0 | 7 | 2.41 | 4.79 | 4.92 | 4.43 | 4.39 | 2.05 (0.129 m) | 2.25 | 3, 3.75 |
| v16 | 140 | pin 7/0 → locate 7/2 | 7 | 4.78 | 4.43 | 4.94 | 3.88 | 3.76 | 1.59 (0.203 m) | 1.78 | 7, 4.43 |
| v16 | 148 | swage 7/3 → swage 7/2 | 4 | 5.11 | 2.49 | 2.49 | 2.48 | 2.39 | 0.88 (0.059 m) | 1.25 | 3, 1.55 |
| fast_b | 147 | pin 8/0 → locate 8/2 | 9 | 4.64 | 6.47 | 6.63 | 6.21 | 5.99 | 1.65 (0.209 m) | 1.85 | 5, 4.81 |
| fast_b | 156 | swage 8/2 → swage 8/0 | 7 | 4.32 | 5.03 | 5.21 | 4.79 | 4.58 | 1.65 (0.209 m) | 1.85 | 5, 4.27 |

Over all 67 multi-piece moves of v16's bay 0 (every arm), blending within 0.010 rad instead of
0.002 rad would save 26.95 s of moving time in all (pin arms 12.5 s), little of it on the critical
path, at up to five times the blend's deviation. The lever is the way round instead: search each
move from more seeds and keep the quickest timed path. A probe with 8 seeds planned v16's bay 0 in
556.74 s (expected gain 18.0 s); 4 seeds gave 567.35 s, 4 seeds with five times the shortening
578.71 s (the schedule is not monotonic in the moves' times).

`path_tries` (`src/layout.h`, an optional top-level integer from 2 to 16, flat rack bases): each
move is searched from that many seeds (its own first), each path shortened as before, and the path
with the quickest trajectory kept (`QuickestPath`, the first of equals). Every path is checked at
the same planning clearance as before; the blend figure is unchanged. The plan file names
`path_tries` only when it is not 1, so a layout without it plans byte-identically, and the
verifier holds a plan to its layout's value (`tests/path_tries_test.cc`, in `//:plan_replay_test`).

| | v16 | v16 + `path_tries` 8 | fast_b | fast_b + `path_tries` 8 |
|---|---:|---:|---:|---:|
| layout | `..._shared_ordered_fast_retreat` | `..._shared_ordered_fast_paths` | `..._ordered_fast_b` | `..._ordered_fast_b_paths` |
| moves found | 126 of 126 | 126 of 126 | 127 of 127 | 127 of 127 |
| planning (one bay) | 39.7 s | 119.6 s | 53.2 s | 100.8 s |
| bay 0 planned | 574.73 s | **555.34 s** (-19.39 s) | 740.38 s | **717.97 s** (-22.41 s) |
| moving, module arm / A / B / C | 205.6 / 110.0 / 71.9 / 60.9 s | 180.7 / 100.5 / 63.0 / 54.5 s | | |
| critical path: moves, waits | 119.3 s, 175.1 s | 111.3 s, 163.6 s | | |

The fast_paths plan is committed (one bay, `plans/flat_rack_3_kr20_pin_arms_shared_ordered_fast_paths.json`,
replayed in `Kr20.OneBayOfTheSharedBeamLayoutWithPathTriesPassesTheVerifier`); fast_b_paths is
committed as a layout with its checker report rule, its plan not (it needs three bays and the
staged tests, as fast_b does).

Run 041 landed beside this one: `..._fast_retreat_ma` (v16 with the modules set in turns of the pin
arms, 543.96 s) removes C's wait for the module arm. The two levers are independent; `path_tries` on
`_ma` has not been planned.

Tightest margins of the fast_paths plan (`plan_layout -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_paths --margins`):
least clearance 0.0160 m against 0.015 m (C's link 3 to `holder_stand_8_saddle`, step 152, as in
v16); arm to arm 0.0205 m (C and B); least joint margin 5.00° against 5° (A's joint 1, from the
checker); track ends 0 slack (module arm -3.600 / 5.500, A -5.700); the largest blend deviation
0.002 rad, at most 0.0150 m anywhere on an arm (the unchanged figure, reached by a blend in both plans).

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

