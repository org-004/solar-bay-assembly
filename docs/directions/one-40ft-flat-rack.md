# One 40 ft base: the cell on the real parts

The cell is rebuilt on the real parts of a tracker bay, a round torque tube, rails, modules and their fastenings, on one 40 ft base. In run 011 the base is a container with both long sides open; it later becomes a flat rack. The work is kinematic: layouts are written as data and checked by a program for transport, reach, clearance, load, the ways in and out, and time. The figures are planned or come from a time model, and no controller or cycle runs in physics yet.

**Limits.** The base may carry 13,608 kg in transit (30,000 lb), and the arms, tracks, tools and bay have to fit within it. Times come from a time model, not from a run in physics.

## Where it stands

The chosen version is v19, "One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths, the module arm within 0.90 of its motors: three bays planned".

| Rank | Measure | Value | Unit | Better | How known | Basis |
|---|---|---|---|---|---|---|
| P0 | cycle_s | 502.13 | s per bay | lower | planned | the committed plan's 3 bays back to back: (end of bay 2's bay_out - start of bay 0's tube_in) / 3; kinematic, on assumed speeds and process times |
| not ranked | one_bay_s | 502.55 | s | - | planned | bay 0 of the committed plan alone, from the start of its tube_in to the end of its bay_out |
| not ranked | interval_s | 496.04 | s | - | planned | from the end of bay 1's bay_out to the end of bay 2's |
| P0 | failures_per_bay | 0.383 | - | lower | modelled | count of operations by kind x rates of scores/rates.json; 100% of it on rates of basis ours |
| P0 | physics_share | 0.0412 | - | higher | measured | physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json: steps 136, 137 of module 5 carried out in physics, 20.727562 s of one bay's 502.553095 s counted |
| P1 | part_count | 37 | - | lower | counted | pieces of equipment by the rule of docs/scorecard.md |
| P1 | arms | 4 | arms | lower | counted | arms of the layout |
| P1 | driven_axes | 39 | - | lower | counted | arm joints, tracks, driven tools and grips by the rule of docs/scorecard.md |
| P1 | placement_mm | none | mm | lower | none | nothing is held or sensed in this cell; the stand-in cell's figure is another cell's |
| P2 | mass_in_transit_kg | 4132.2 | kg | lower | counted | the transport row's parts less infed parts (owner's definition) |
| P2 | input_tolerance | none | mm | higher | none | nothing is sensed in this cell, so no error of the infed parts is measured or corrected |
| P2 | plan_s | none | s | lower | none | the committed plan does not hold the time plan_layout took |

| Gate | Figure | Limit | Result |
|---|---|---|---|
| check_rows_failed | 0 rows | <= 0 rows | pass |
| least_clearance_m | 0.016 m | >= 0.015 m | pass |
| mass_in_transit_kg | 4132.2 kg | <= 13608.0 kg | pass |

The measures and gates are defined in [the scorecard](../scorecard.md).

## Progress

| Version | Date | Title | Cycle time | One bay | Arms | Part count | Mass in transit | Cycle time is |
|---|---|---|---|---|---|---|---|---|
| v05 | 2026-11-24 | The real parts of a bay and the container, as models | none | none | none | none | none | none |
| v06 | 2026-11-25 | A bay of nine modules and a checked layout in a 40 ft container | 628.50 s per bay (one bay) | 628.50 s | 2 | none | 3227.3 kg | modelled |
| v07 | 2026-11-25 | One pin arm on the flat rack (candidate) | 907.34 s per bay (one bay) | 907.34 s | 2 | none | none | planned |
| v08 | 2026-11-25 | Two pin arms on the flat rack (candidate) | 686.26 s per bay | 700.08 s | 3 | 31 | 3376.9 kg | planned |
| v09 | 2026-11-25 | Three pin arms on the flat rack: a whole bay planned | 622.31 s per bay | 644.28 s | 4 | 39 | 3426.3 kg | planned |
| v10 | 2026-11-26 | Three pin arms on the flat rack, the bay shortened by the order of work | 554.55 s per bay | 560.45 s | 4 | 39 | 3426.3 kg | planned |
| v15 | 2026-11-27 | One 40 ft flat rack, three KR 20 pin arms on one shared beam, all KUKA: three bays planned (candidate) | 612.40 s per bay | 626.32 s | 4 | 37 | 4132.2 kg | planned |
| v16 | 2026-11-27 | One 40 ft flat rack, three KR 20 pin arms on one shared beam, faster retreats: three bays planned | 572.56 s per bay | 574.73 s | 4 | 37 | 4132.2 kg | planned |
| v17 | 2026-11-27 | One 40 ft flat rack, three KR 20 pin arms on one shared beam, the modules in turns of the pin arms, each move the quickest of 8 paths: three bays planned | 527.64 s per bay | 529.20 s | 4 | 37 | 4132.2 kg | planned |
| v18 | 2026-11-27 | One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths: three bays planned | 498.55 s per bay | 499.54 s | 4 | 37 | 4132.2 kg | planned |
| v19 | 2026-11-27 | One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths, the module arm within 0.90 of its motors: three bays planned | 502.13 s per bay | 502.55 s | 4 | 37 | 4132.2 kg | planned |
| v20 | 2026-11-27 | One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths, the module arm within 0.90 of its motors, the bay out once it is clear: three bays planned (candidate) | 496.16 s per bay | 496.59 s | 4 | 37 | 4132.2 kg | planned |

The cycle time of a version with a plan is the average bay of three bays planned back to back, (end of bay 2's bay out − start of bay 0's tube in) / 3; "One bay" is its bay 0 alone, from tube in to bay out. A version without a plan has the time of one bay, marked so ([the scorecard](../scorecard.md)).

![Cycle time of each version](one-40ft-flat-rack-cycle.svg)

A filled circle is a cycle time that is planned, a square one that is measured, a hollow circle one that is modelled; a lighter marker is a candidate that was not chosen. A value marked "1 bay" is one bay's; the others are the average of three bays back to back.

## v19 One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths, the module arm within 0.90 of its motors: three bays planned

![v19  2026-11-27  One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths, the module arm within 0.90 of its motors: three bays planned; cycle 502.13 s per bay (planned), 4 arms](../evolution/v19.gif)

2026-11-27, chosen, commit the commit that added v19: `git log --format=%h -S '"id": "v19"' -- versions/versions.json | tail -1`

**What changed.** v18's cell and layout with one change and nothing else: torque_limit_scale 0.90 (runs 045 and 047), so each module arm move that inverse dynamics finds needing more than 0.90 of a motor's limit is played slower by uniform time scaling until it holds, and the bay is scheduled in the order and at the starts of its unslowed schedule. Three bays are planned whole, 378 of 378 moves, 26 of them slowed (14.30 s of motion added), verified and committed: 502.55 s for one bay (502.55 / 507.79 / 496.04 s) and a cycle time of 502.13 s over three.

**Why.** v18's cycle time of 498.55 s is one its module arm cannot follow: inverse dynamics asks up to 1.25 of joint 1's torque limit and physics saturates joint 1 at 1.000. v19 is v18 timed so that no module arm move needs more than 0.90 of a motor's limit, at 3.58 s a bay more (502.13 s against 498.55 s). Registered as a candidate weighed against v18 and made the chosen version in the same run (run 048): a cycle time the arm cannot follow is not an honest P0 figure, and in physics v19's record take keeps every module arm joint below its limit (joint 1 0.973 on the take, 0.420 on the set); v18 is superseded with its numbers kept.

| Rank | Measure | Value | Unit | Better | How known | Basis |
|---|---|---|---|---|---|---|
| P0 | cycle_s | 502.13 | s per bay | lower | planned | the committed plan's 3 bays back to back: (end of bay 2's bay_out - start of bay 0's tube_in) / 3; kinematic, on assumed speeds and process times |
| not ranked | one_bay_s | 502.55 | s | - | planned | bay 0 of the committed plan alone, from the start of its tube_in to the end of its bay_out |
| not ranked | interval_s | 496.04 | s | - | planned | from the end of bay 1's bay_out to the end of bay 2's |
| P0 | failures_per_bay | 0.383 | - | lower | modelled | count of operations by kind x rates of scores/rates.json; 100% of it on rates of basis ours |
| P0 | physics_share | 0.0412 | - | higher | measured | physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json: steps 136, 137 of module 5 carried out in physics, 20.727562 s of one bay's 502.553095 s counted |
| P1 | part_count | 37 | - | lower | counted | pieces of equipment by the rule of docs/scorecard.md |
| P1 | arms | 4 | arms | lower | counted | arms of the layout |
| P1 | driven_axes | 39 | - | lower | counted | arm joints, tracks, driven tools and grips by the rule of docs/scorecard.md |
| P1 | placement_mm | none | mm | lower | none | nothing is held or sensed in this cell; the stand-in cell's figure is another cell's |
| P2 | mass_in_transit_kg | 4132.2 | kg | lower | counted | the transport row's parts less infed parts (owner's definition) |
| P2 | input_tolerance | none | mm | higher | none | nothing is sensed in this cell, so no error of the infed parts is measured or corrected |
| P2 | plan_s | none | s | lower | none | the committed plan does not hold the time plan_layout took |

| Gate | Figure | Limit | Result |
|---|---|---|---|
| check_rows_failed | 0 rows | <= 0 rows | pass |
| least_clearance_m | 0.016 m | >= 0.015 m | pass |
| mass_in_transit_kg | 4132.2 kg | <= 13608.0 kg | pass |

The measures and gates are defined in [the scorecard](../scorecard.md).

**Against v18.**

Each figure as the two versions' own sources report it; the conditions of a run may differ between them (see each figure's source).

| Measure | Before (v18) | Now (v19) | Difference |
|---|---|---|---|
| arms | 4 arms | 4 arms | 0 arms |
| cycle_s | 498.55 s per bay | 502.13 s per bay | +3.57 s per bay |
| driven_axes | 39 | 39 | 0 |
| failures_per_bay | 0.383 | 0.383 | 0 |
| input_tolerance | none | none | not comparable: not known |
| interval_s | 493.59 s | 496.04 s | +2.45 s |
| mass_in_transit_kg | 4132.2 kg | 4132.2 kg | 0.0 kg |
| one_bay_s | 499.54 s | 502.55 s | +3.01 s |
| part_count | 37 | 37 | 0 |
| physics_share | 0.0404 | 0.0412 | +0.000798 |
| placement_mm | none | none | not comparable: not known |
| plan_s | none | none | not comparable: not known |

**Candidates it was chosen over.**

| Version | Cycle time | One bay | Arms | Part count | Mass in transit | Failures per bay | Cycle time is | Recording | Layout | Commit | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v19 (this version) | 502.13 s per bay | 502.55 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v19.gif](../evolution/v19.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json) | the commit that added v19: `git log --format=%h -S '"id": "v19"' -- versions/versions.json \| tail -1` | chosen |
| v20 | 496.16 s per bay | 496.59 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v20.gif](../evolution/v20.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out.json) | the commit that added v20: `git log --format=%h -S '"id": "v20"' -- versions/versions.json \| tail -1` | candidate |

**Files.** [layout](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json), [plan](../../plans/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json), [scorecard](../../scores/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json)

## v18 One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths: three bays planned

![v18  2026-11-27  One 40 ft flat rack, three KR 20 pin arms on one shared beam, three modules to each pin arm in turns, each move the quickest of 8 paths: three bays planned; cycle 498.55 s per bay (planned), 4 arms](../evolution/v18.gif)

2026-11-27, superseded, commit the commit that added v18: `git log --format=%h -S '"id": "v18"' -- versions/versions.json | tail -1`

**What changed.** v17's cell (all KUKA, three KR 20 pin arms on one shared beam) with run 044's split and order in place of run 041's: module 3 moved from pin arm A to B and module 6 from B to C, three modules to each pin arm, order.modules 0, 3, 6, 1, 4, 7, 2, 5, 8, and B's wait/stow track 0.6; path_tries 8 as in v17 (each move searched from 8 seeds and the quickest timed path kept). Three bays are planned whole, 378 of 378 moves, verified and committed: 499.54 s for one bay (499.54 / 502.53 / 493.59 s) and a cycle time of 498.55 s over three.

**Why.** It is the measured step after v17 in the all-KUKA 40 ft cell: 498.55 s against v17's 527.64 s (-29.09 s). Registered as a candidate weighed against v17 and made the chosen version in the same run (run 046): the chosen cell is the best all-KUKA cell (the owner's rule), and v17 is superseded with its numbers kept. It was superseded on 2026-11-27 (run 048) by v19, the same layout timed within its motors: v18's moves exceed the motors (inverse dynamics asks up to 1.249 of joint 1's torque limit, 18 of its 117 module arm moves over 1.0, and physics saturates joint 1 at 1.000 on its record take), so its 498.55 s is not a cycle time the arm can follow; its numbers are kept.

| Rank | Measure | Value | Unit | Better | How known | Basis |
|---|---|---|---|---|---|---|
| P0 | cycle_s | 498.55 | s per bay | lower | planned | the committed plan's 3 bays back to back: (end of bay 2's bay_out - start of bay 0's tube_in) / 3; kinematic, on assumed speeds and process times |
| not ranked | one_bay_s | 499.54 | s | - | planned | bay 0 of the committed plan alone, from the start of its tube_in to the end of its bay_out |
| not ranked | interval_s | 493.59 | s | - | planned | from the end of bay 1's bay_out to the end of bay 2's |
| P0 | failures_per_bay | 0.383 | - | lower | modelled | count of operations by kind x rates of scores/rates.json; 100% of it on rates of basis ours |
| P0 | physics_share | 0.0404 | - | higher | measured | physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json: steps 136, 137 of module 5 carried out in physics, 20.205074 s of one bay's 499.542691 s counted |
| P1 | part_count | 37 | - | lower | counted | pieces of equipment by the rule of docs/scorecard.md |
| P1 | arms | 4 | arms | lower | counted | arms of the layout |
| P1 | driven_axes | 39 | - | lower | counted | arm joints, tracks, driven tools and grips by the rule of docs/scorecard.md |
| P1 | placement_mm | none | mm | lower | none | nothing is held or sensed in this cell; the stand-in cell's figure is another cell's |
| P2 | mass_in_transit_kg | 4132.2 | kg | lower | counted | the transport row's parts less infed parts (owner's definition) |
| P2 | input_tolerance | none | mm | higher | none | nothing is sensed in this cell, so no error of the infed parts is measured or corrected |
| P2 | plan_s | none | s | lower | none | the committed plan does not hold the time plan_layout took |

| Gate | Figure | Limit | Result |
|---|---|---|---|
| check_rows_failed | 0 rows | <= 0 rows | pass |
| least_clearance_m | 0.016 m | >= 0.015 m | pass |
| mass_in_transit_kg | 4132.2 kg | <= 13608.0 kg | pass |

The measures and gates are defined in [the scorecard](../scorecard.md).

**Against v17.**

Each figure as the two versions' own sources report it; the conditions of a run may differ between them (see each figure's source).

| Measure | Before (v17) | Now (v18) | Difference |
|---|---|---|---|
| arms | 4 arms | 4 arms | 0 arms |
| cycle_s | 527.64 s per bay | 498.55 s per bay | -29.09 s per bay |
| driven_axes | 39 | 39 | 0 |
| failures_per_bay | 0.383 | 0.383 | 0 |
| input_tolerance | none | none | not comparable: not known |
| interval_s | 523.92 s | 493.59 s | -30.33 s |
| mass_in_transit_kg | 4132.2 kg | 4132.2 kg | 0.0 kg |
| one_bay_s | 529.20 s | 499.54 s | -29.65 s |
| part_count | 37 | 37 | 0 |
| physics_share | 0.0392 | 0.0404 | +0.00128 |
| placement_mm | none | none | not comparable: not known |
| plan_s | none | none | not comparable: not known |

**Candidates it was chosen over.**

| Version | Cycle time | One bay | Arms | Part count | Mass in transit | Failures per bay | Cycle time is | Recording | Layout | Commit | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v18 (this version) | 498.55 s per bay | 499.54 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v18.gif](../evolution/v18.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json) | the commit that added v18: `git log --format=%h -S '"id": "v18"' -- versions/versions.json \| tail -1` | superseded |
| v19 | 502.13 s per bay | 502.55 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v19.gif](../evolution/v19.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque.json) | the commit that added v19: `git log --format=%h -S '"id": "v19"' -- versions/versions.json \| tail -1` | chosen |

**Files.** [layout](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json), [plan](../../plans/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json), [scorecard](../../scores/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json)

## v17 One 40 ft flat rack, three KR 20 pin arms on one shared beam, the modules in turns of the pin arms, each move the quickest of 8 paths: three bays planned

![v17  2026-11-27  One 40 ft flat rack, three KR 20 pin arms on one shared beam, the modules in turns of the pin arms, each move the quickest of 8 paths: three bays planned; cycle 527.64 s per bay (planned), 4 arms](../evolution/v17.gif)

2026-11-27, superseded, commit the commit that added v17: `git log --format=%h -S '"id": "v17"' -- versions/versions.json | tail -1`

**What changed.** v16's cell with two changes and nothing else: run 041's module order (order.modules 0, 4, 7, 1, 5, 8, 2, 6, 3, the modules set in turns of pin arms A, B and C) and run 042's path_tries 8 (each move searched from 8 seeds and the quickest timed path kept). Three bays are planned whole, 378 of 378 moves, verified and committed: 529.20 s for one bay (529.20 / 529.80 / 523.92 s) and a cycle time of 527.64 s over three.

**Why.** It is the measured step after v16 in the all-KUKA 40 ft cell: 527.64 s against v16's 572.56 s (-44.92 s), and 26.91 s below v10's 554.55 s. Registered as a candidate weighed against v16 and made the chosen version in the same run (run 043): the chosen cell is the best all-KUKA cell (the owner's rule), and v16 is superseded with its numbers kept. It was superseded on 2026-11-27 (run 046) by v18, 29.09 s quicker on the same cell, with its numbers kept.

| Rank | Measure | Value | Unit | Better | How known | Basis |
|---|---|---|---|---|---|---|
| P0 | cycle_s | 527.64 | s per bay | lower | planned | the committed plan's 3 bays back to back: (end of bay 2's bay_out - start of bay 0's tube_in) / 3; kinematic, on assumed speeds and process times |
| not ranked | one_bay_s | 529.20 | s | - | planned | bay 0 of the committed plan alone, from the start of its tube_in to the end of its bay_out |
| not ranked | interval_s | 523.92 | s | - | planned | from the end of bay 1's bay_out to the end of bay 2's |
| P0 | failures_per_bay | 0.383 | - | lower | modelled | count of operations by kind x rates of scores/rates.json; 100% of it on rates of basis ours |
| P0 | physics_share | 0.0392 | - | higher | measured | physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json: steps 136, 137 of module 6 carried out in physics, 20.725585 s of one bay's 529.197537 s counted |
| P1 | part_count | 37 | - | lower | counted | pieces of equipment by the rule of docs/scorecard.md |
| P1 | arms | 4 | arms | lower | counted | arms of the layout |
| P1 | driven_axes | 39 | - | lower | counted | arm joints, tracks, driven tools and grips by the rule of docs/scorecard.md |
| P1 | placement_mm | none | mm | lower | none | nothing is held or sensed in this cell; the stand-in cell's figure is another cell's |
| P2 | mass_in_transit_kg | 4132.2 | kg | lower | counted | the transport row's parts less infed parts (owner's definition) |
| P2 | input_tolerance | none | mm | higher | none | nothing is sensed in this cell, so no error of the infed parts is measured or corrected |
| P2 | plan_s | none | s | lower | none | the committed plan does not hold the time plan_layout took |

| Gate | Figure | Limit | Result |
|---|---|---|---|
| check_rows_failed | 0 rows | <= 0 rows | pass |
| least_clearance_m | 0.016 m | >= 0.015 m | pass |
| mass_in_transit_kg | 4132.2 kg | <= 13608.0 kg | pass |

The measures and gates are defined in [the scorecard](../scorecard.md).

**Against v16.**

Each figure as the two versions' own sources report it; the conditions of a run may differ between them (see each figure's source).

| Measure | Before (v16) | Now (v17) | Difference |
|---|---|---|---|
| arms | 4 arms | 4 arms | 0 arms |
| cycle_s | 572.56 s per bay | 527.64 s per bay | -44.92 s per bay |
| driven_axes | 39 | 39 | 0 |
| failures_per_bay | 0.383 | 0.383 | 0 |
| input_tolerance | none | none | not comparable: not known |
| interval_s | 564.63 s | 523.92 s | -40.71 s |
| mass_in_transit_kg | 4132.2 kg | 4132.2 kg | 0.0 kg |
| one_bay_s | 574.73 s | 529.20 s | -45.53 s |
| part_count | 37 | 37 | 0 |
| physics_share | 0.0322 | 0.0392 | +0.00694 |
| placement_mm | none | none | not comparable: not known |
| plan_s | none | none | not comparable: not known |

**Candidates it was chosen over.**

| Version | Cycle time | One bay | Arms | Part count | Mass in transit | Failures per bay | Cycle time is | Recording | Layout | Commit | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v17 (this version) | 527.64 s per bay | 529.20 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v17.gif](../evolution/v17.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json) | the commit that added v17: `git log --format=%h -S '"id": "v17"' -- versions/versions.json \| tail -1` | superseded |
| v18 | 498.55 s per bay | 499.54 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v18.gif](../evolution/v18.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths.json) | the commit that added v18: `git log --format=%h -S '"id": "v18"' -- versions/versions.json \| tail -1` | superseded |

**Files.** [layout](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json), [plan](../../plans/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json), [scorecard](../../scores/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json)

## v16 One 40 ft flat rack, three KR 20 pin arms on one shared beam, faster retreats: three bays planned

![v16  2026-11-27  One 40 ft flat rack, three KR 20 pin arms on one shared beam, faster retreats: three bays planned; cycle 572.56 s per bay (planned), 4 arms](../evolution/v16.gif)

2026-11-27, superseded, commit the commit that added v16: `git log --format=%h -S '"id": "v16"' -- versions/versions.json | tail -1`

**What changed.** v15's cell with retreat_legs (run 036): a leg that leaves contact into free air (a retreat) runs at up to 0.5 m/s and 3.0 m/s^2 of the working point; approaches keep 0.25 m/s and 1.0 m/s^2. Three bays are planned whole, 378 of 378 moves, verified and committed: 574.73 s for one bay and a cycle time of 572.56 s over three.

**Why.** It is the measured step after v15 in the all-KUKA 40 ft cell: 572.56 s against v15's 612.40 s. It was registered as a candidate against v15, 18.01 s above v10's 554.55 s. It became the chosen version on 2026-11-27 (run 040) because the owner decided on 2026-11-26 that every arm of every cell is KUKA, and v10's three pin arms are UR10e; v10 is superseded with its numbers kept. It was superseded on 2026-11-27 (run 043) by v17, 44.92 s quicker on the same cell, with its numbers kept.

| Rank | Measure | Value | Unit | Better | How known | Basis |
|---|---|---|---|---|---|---|
| P0 | cycle_s | 572.56 | s per bay | lower | planned | the committed plan's 3 bays back to back: (end of bay 2's bay_out - start of bay 0's tube_in) / 3; kinematic, on assumed speeds and process times |
| not ranked | one_bay_s | 574.73 | s | - | planned | bay 0 of the committed plan alone, from the start of its tube_in to the end of its bay_out |
| not ranked | interval_s | 564.63 | s | - | planned | from the end of bay 1's bay_out to the end of bay 2's |
| P0 | failures_per_bay | 0.383 | - | lower | modelled | count of operations by kind x rates of scores/rates.json; 100% of it on rates of basis ours |
| P0 | physics_share | 0.0322 | - | higher | measured | physics/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json: steps 77, 78 of module 3 carried out in physics, 18.522716 s of one bay's 574.729769 s counted |
| P1 | part_count | 37 | - | lower | counted | pieces of equipment by the rule of docs/scorecard.md |
| P1 | arms | 4 | arms | lower | counted | arms of the layout |
| P1 | driven_axes | 39 | - | lower | counted | arm joints, tracks, driven tools and grips by the rule of docs/scorecard.md |
| P1 | placement_mm | none | mm | lower | none | nothing is held or sensed in this cell; the stand-in cell's figure is another cell's |
| P2 | mass_in_transit_kg | 4132.2 | kg | lower | counted | the transport row's parts less infed parts (owner's definition) |
| P2 | input_tolerance | none | mm | higher | none | nothing is sensed in this cell, so no error of the infed parts is measured or corrected |
| P2 | plan_s | none | s | lower | none | the committed plan does not hold the time plan_layout took |

| Gate | Figure | Limit | Result |
|---|---|---|---|
| check_rows_failed | 0 rows | <= 0 rows | pass |
| least_clearance_m | 0.016 m | >= 0.015 m | pass |
| mass_in_transit_kg | 4132.2 kg | <= 13608.0 kg | pass |

The measures and gates are defined in [the scorecard](../scorecard.md).

**Against v10.**

Each figure as the two versions' own sources report it; the conditions of a run may differ between them (see each figure's source).

| Measure | Before (v10) | Now (v16) | Difference |
|---|---|---|---|
| arms | 4 arms | 4 arms | 0 arms |
| cycle_s | 554.55 s per bay | 572.56 s per bay | +18.01 s per bay |
| driven_axes | 40 | 39 | -1 |
| failures_per_bay | 0.385 | 0.383 | -0.0022 |
| input_tolerance | none | none | not comparable: not known |
| interval_s | 554.49 s | 564.63 s | +10.14 s |
| mass_in_transit_kg | 3426.3 kg | 4132.2 kg | +705.9 kg |
| one_bay_s | 560.45 s | 574.73 s | +14.28 s |
| part_count | 39 | 37 | -2 |
| physics_share | 0.0394 | 0.0322 | -0.00715 |
| placement_mm | none | none | not comparable: not known |
| plan_s | none | none | not comparable: not known |

**Candidates it was chosen over.**

| Version | Cycle time | One bay | Arms | Part count | Mass in transit | Failures per bay | Cycle time is | Recording | Layout | Commit | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v16 (this version) | 572.56 s per bay | 574.73 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v16.gif](../evolution/v16.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json) | the commit that added v16: `git log --format=%h -S '"id": "v16"' -- versions/versions.json \| tail -1` | superseded |
| v17 | 527.64 s per bay | 529.20 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v17.gif](../evolution/v17.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_paths.json) | the commit that added v17: `git log --format=%h -S '"id": "v17"' -- versions/versions.json \| tail -1` | superseded |

**Files.** [layout](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json), [plan](../../plans/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json), [scorecard](../../scores/flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json)

## v10 Three pin arms on the flat rack, the bay shortened by the order of work

![v10  2026-11-26  Three pin arms on the flat rack, the bay shortened by the order of work; cycle 554.55 s per bay (planned), 4 arms](../evolution/v10.gif)

This recording shows the module arm's take and set of one module of v10 in physics (`physics/flat_rack_3_pin_arms_ordered.json`, `//:run_physics`).

![v10 in physics: the module arm takes and sets one module](../evolution/v10-physics.gif)

2026-11-26, superseded, commit the commit that added v10: `git log --format=%h -S '"id": "v10"' -- versions/versions.json | tail -1`

**What changed.** The same cell as v09, with no new part, arm or tool and no changed time, speed or margin: only the order of work changed. The modules are set in the order 0, 1, 4, 2, 5, 7, 6, 3, 8, so that the three pin arms' modules come in turn. A module is taken as soon as the strap bolts of its own two rails are driven, not after the last bolt. A pin arm changes to its pin tool and takes its magazine as soon as its bolts are done, not when its first module is set. The planned bay is 560.45 s, down from 644.28 s.

**Why.** The cycle time of a bay is the first measure. Of 23 orders planned in full (tools/order/search.py), this one gave the shortest bay that the verifier passes. Every other measure of the scorecard is unchanged: 0.3849 failures per bay, 39 parts, 4 arms, 40 driven axes, 3,426.3 kg in transit. The least clearance over the schedule is 0.0180 m against 0.015 m.

| Rank | Measure | Value | Unit | Better | How known | Basis |
|---|---|---|---|---|---|---|
| P0 | cycle_s | 554.55 | s per bay | lower | planned | the committed plan's 3 bays back to back: (end of bay 2's bay_out - start of bay 0's tube_in) / 3; kinematic, on assumed speeds and process times |
| not ranked | one_bay_s | 560.45 | s | - | planned | bay 0 of the committed plan alone, from the start of its tube_in to the end of its bay_out |
| not ranked | interval_s | 554.49 | s | - | planned | from the end of bay 1's bay_out to the end of bay 2's |
| P0 | failures_per_bay | 0.385 | - | lower | modelled | count of operations by kind x rates of scores/rates.json; 100% of it on rates of basis ours |
| P0 | physics_share | 0.0394 | - | higher | measured | physics/flat_rack_3_pin_arms_ordered.json: steps 63, 64 of module 4 carried out in physics, 22.068854 s of one bay's 560.452329 s counted |
| P1 | part_count | 39 | - | lower | counted | pieces of equipment by the rule of docs/scorecard.md |
| P1 | arms | 4 | arms | lower | counted | arms of the layout |
| P1 | driven_axes | 40 | - | lower | counted | arm joints, tracks, driven tools and grips by the rule of docs/scorecard.md |
| P1 | placement_mm | none | mm | lower | none | nothing is held or sensed in this cell; the stand-in cell's figure is another cell's |
| P2 | mass_in_transit_kg | 3426.3 | kg | lower | counted | the transport row's parts less infed parts (owner's definition) |
| P2 | input_tolerance | none | mm | higher | none | nothing is sensed in this cell, so no error of the infed parts is measured or corrected |
| P2 | plan_s | none | s | lower | none | the committed plan does not hold the time plan_layout took |

| Gate | Figure | Limit | Result |
|---|---|---|---|
| check_rows_failed | 0 rows | <= 0 rows | pass |
| least_clearance_m | 0.018 m | >= 0.015 m | pass |
| mass_in_transit_kg | 3426.3 kg | <= 13608.0 kg | pass |

The measures and gates are defined in [the scorecard](../scorecard.md).

**Against v09.**

Each figure as the two versions' own sources report it; the conditions of a run may differ between them (see each figure's source).

| Measure | Before (v09) | Now (v10) | Difference |
|---|---|---|---|
| arms | 4 arms | 4 arms | 0 arms |
| cycle_s | 622.31 s per bay | 554.55 s per bay | -67.77 s per bay |
| driven_axes | 40 | 40 | 0 |
| failures_per_bay | 0.385 | 0.385 | 0 |
| input_tolerance | none | none | not comparable: not known |
| interval_s | 604.29 s | 554.49 s | -49.80 s |
| mass_in_transit_kg | 3426.3 kg | 3426.3 kg | 0.0 kg |
| one_bay_s | 644.28 s | 560.45 s | -83.83 s |
| part_count | 39 | 39 | 0 |
| physics_share | 0 | 0.0394 | +0.0394 (counted → measured) |
| placement_mm | none | none | not comparable: not known |
| plan_s | none | none | not comparable: not known |

**Candidates it was chosen over.**

| Version | Cycle time | One bay | Arms | Part count | Mass in transit | Failures per bay | Cycle time is | Recording | Layout | Commit | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v10 (this version) | 554.55 s per bay | 560.45 s | 4 | 39 | 3426.3 kg | 0.385 | planned | [v10.gif](../evolution/v10.gif) | [flat_rack_3_pin_arms_ordered.json](../../layouts/flat_rack_3_pin_arms_ordered.json) | the commit that added v10: `git log --format=%h -S '"id": "v10"' -- versions/versions.json \| tail -1` | superseded |
| v15 | 612.40 s per bay | 626.32 s | 4 | 37 | 4132.2 kg | 0.383 | planned | [v15.gif](../evolution/v15.gif) | [flat_rack_3_kr20_pin_arms_shared_ordered.json](../../layouts/flat_rack_3_kr20_pin_arms_shared_ordered.json) | the commit that added v15: `git log --format=%h -S '"id": "v15"' -- versions/versions.json \| tail -1` | candidate |

**Files.** [layout](../../layouts/flat_rack_3_pin_arms_ordered.json), [plan](../../plans/flat_rack_3_pin_arms_ordered.json), [scorecard](../../scores/flat_rack_3_pin_arms_ordered.json)

## v09 Three pin arms on the flat rack: a whole bay planned

![v09  2026-11-25  Three pin arms on the flat rack: a whole bay planned; cycle 622.31 s per bay (planned), 4 arms](../evolution/v09.gif)

2026-11-25, superseded, commit [940a01b](https://github.com/org-004/solar-bay-assembly/commit/940a01b)

**What changed.** The cell stands on the base of a 40 ft flat rack: one KUKA KR 210 on a track sets the ten rails and nine modules, and three UR10e on tracks below the bay drive the strap bolts and locate, pin and swage the modules. Every move of every arm for one bay is planned free of collision, the four arms are scheduled on one clock, and the plan is verified and committed.

**Why.** The owner allowed the first cell to stand on a flat rack with up to three pin arms. Of one, two and three pin arms, three gave the shortest planned bay, 644.28 s against 700.08 s and 907.34 s, with the same tightest clearance at a pose.

| Rank | Measure | Value | Unit | Better | How known | Basis |
|---|---|---|---|---|---|---|
| P0 | cycle_s | 622.31 | s per bay | lower | planned | the committed plan's 3 bays back to back: (end of bay 2's bay_out - start of bay 0's tube_in) / 3; kinematic, on assumed speeds and process times |
| not ranked | one_bay_s | 644.28 | s | - | planned | bay 0 of the committed plan alone, from the start of its tube_in to the end of its bay_out |
| not ranked | interval_s | 604.29 | s | - | planned | from the end of bay 1's bay_out to the end of bay 2's |
| P0 | failures_per_bay | 0.385 | - | lower | modelled | count of operations by kind x rates of scores/rates.json; 100% of it on rates of basis ours |
| P0 | physics_share | 0 | - | higher | counted | no committed, repeatable run in physics of this layout's arms and parts |
| P1 | part_count | 39 | - | lower | counted | pieces of equipment by the rule of docs/scorecard.md |
| P1 | arms | 4 | arms | lower | counted | arms of the layout |
| P1 | driven_axes | 40 | - | lower | counted | arm joints, tracks, driven tools and grips by the rule of docs/scorecard.md |
| P1 | placement_mm | none | mm | lower | none | nothing is held or sensed in this cell; the stand-in cell's figure is another cell's |
| P2 | mass_in_transit_kg | 3426.3 | kg | lower | counted | the transport row's parts less infed parts (owner's definition) |
| P2 | input_tolerance | none | mm | higher | none | nothing is sensed in this cell, so no error of the infed parts is measured or corrected |
| P2 | plan_s | none | s | lower | none | the committed plan does not hold the time plan_layout took |

| Gate | Figure | Limit | Result |
|---|---|---|---|
| check_rows_failed | 0 rows | <= 0 rows | pass |
| least_clearance_m | 0.018 m | >= 0.015 m | pass |
| mass_in_transit_kg | 3426.3 kg | <= 13608.0 kg | pass |

The measures and gates are defined in [the scorecard](../scorecard.md).

**Against v06.**

Each figure as the two versions' own sources report it; the conditions of a run may differ between them (see each figure's source).

| Measure | Before (v06) | Now (v09) | Difference |
|---|---|---|---|
| arms | 2 arms | 4 arms | +2 arms |
| cycle_s | 628.50 s per bay | 622.31 s per bay | not comparable: one bay against the average of three (modelled → planned) |
| mass_in_transit_kg | 3227.3 kg | 3426.3 kg | +199.0 kg |

**Candidates it was chosen over.**

| Version | Cycle time | One bay | Arms | Part count | Mass in transit | Failures per bay | Cycle time is | Recording | Layout | Commit | Status |
|---|---|---|---|---|---|---|---|---|---|---|---|
| v09 (this version) | 622.31 s per bay | 644.28 s | 4 | 39 | 3426.3 kg | 0.385 | planned | [v09.gif](../evolution/v09.gif) | [flat_rack_3_pin_arms.json](../../layouts/flat_rack_3_pin_arms.json) | [940a01b](https://github.com/org-004/solar-bay-assembly/commit/940a01b) | superseded |
| v07 | 907.34 s per bay (one bay) | 907.34 s | 2 | none | none | none | planned | [v07.png](../evolution/v07.png) | [flat_rack_1_pin_arm.json](../../layouts/flat_rack_1_pin_arm.json) | [940a01b](https://github.com/org-004/solar-bay-assembly/commit/940a01b) | candidate |
| v08 | 686.26 s per bay | 700.08 s | 3 | 31 | 3376.9 kg | 0.382 | planned | [v08.gif](../evolution/v08.gif) | [flat_rack_2_pin_arms.json](../../layouts/flat_rack_2_pin_arms.json) | [940a01b](https://github.com/org-004/solar-bay-assembly/commit/940a01b) | candidate |

**Files.** [layout](../../layouts/flat_rack_3_pin_arms.json), [plan](../../plans/flat_rack_3_pin_arms.json), [scorecard](../../scores/flat_rack_3_pin_arms.json)

## v06 A bay of nine modules and a checked layout in a 40 ft container

![v06  2026-11-25  A bay of nine modules and a checked layout in a 40 ft container; cycle 628.50 s per bay (modelled, one bay), 2 arms](../evolution/v06.png)

2026-11-25, superseded, commit [68a1199](https://github.com/org-004/solar-bay-assembly/commit/68a1199)

**What changed.** A bay becomes nine modules and ten rails, so the section is 10.616 m long with 46 fastenings. A layout is one JSON file, and a program checks it for transport, reach, clearance, load, the ways in and out, and time. Five layouts are compared in docs/layout.md and below_rails_by_module_arm is chosen: a KUKA KR 210 on a track sets rails and modules, and a UR10e on a floor track bolts, locates and pins.

**Why.** The container could not take a bay of ten modules with room to work, and the cell needed a layout whose reach, clearance and load were checked before anything was built.

| Figure | Value | Unit | How known | Source |
|---|---|---|---|---|
| cycle_s | 628.50 | s per bay | modelled | docs/layout.md at 68a1199, comparison table, Time for a bay, below_rails_by_module_arm, from the time model whose process times are marked ours; `bazel run --config=opt //:check_layout -- below_rails_by_module_arm` |
| arms | 2 | arms | counted | docs/layout.md at 68a1199, comparison table, Arms: one KUKA KR 210 R2700-2 and one UR10e, both on floor tracks |
| bay_mass_kg | 410.3 | kg | derived | docs/bay-hardware.md at 68a1199, The bay: tube 95.16 + 10 rails x 2.35 + 9 modules x 32.4 (derived); `bazel run --config=opt //:print_bay -- --summary` |
| mass_in_transit_kg | 3227.3 | kg | counted | docs/layout.md at 68a1199, comparison table, Mass that travels inside, of 17,200 kg: arms 1110, tracks 1850, tools 47.0, holder 220; `bazel run --config=opt //:check_layout -- below_rails_by_module_arm`. No stocks travelled inside at that commit, so the whole row is mass in transit by the owner's definition |
| reach_rows_passed | 186 | of 186 | counted | docs/layout.md at 68a1199, comparison table, reach; kinematic check, not a physics run: `bazel run --config=opt //:check_layout -- below_rails_by_module_arm --summary` |
| clearance_rows_passed | 270 | of 270 | counted | docs/layout.md at 68a1199, comparison table, clearance, tightest 0.0003 m; `bazel run --config=opt //:check_layout -- below_rails_by_module_arm --summary` |

**Against v05.**

Each figure as the two versions' own sources report it; the conditions of a run may differ between them (see each figure's source).

| Measure | Before (v05) | Now (v06) | Difference |
|---|---|---|---|
| bay_mass_kg | 455.3 kg | 410.3 kg | -45.0 kg |

## v05 The real parts of a bay and the container, as models

![v05  2026-11-24  The real parts of a bay and the container, as models; cycle time: none at this version](../evolution/v05.png)

2026-11-24, superseded, commit [1213319](https://github.com/org-004/solar-bay-assembly/commit/1213319)

**What changed.** The real parts of one bay of a single-axis tracker, after Nextracker's NX Horizon, are written down in docs/bay-hardware.md and built as models: a round torque tube with 11 rails, 10 modules and 51 fastening points. A 40 ft container with both long sides open is modelled with the figures that decide whether a bay fits and passes out. The cell itself does not change and still uses its old square tube.

**Why.** The stand-in cell used a square tube and ten slots, so the next step needed the real parts of a bay and a container to work in, with each figure marked published, derived or ours.

| Figure | Value | Unit | How known | Source |
|---|---|---|---|---|
| modules_per_bay | 10 | modules | assumed | docs/bay-hardware.md at 1213319, The bay (marked ours): 10 modules, the largest whose bay is no longer than 11.8 m |
| bay_mass_kg | 455.3 | kg | derived | docs/bay-hardware.md at 1213319, The bay (derived: tube 105.42 + 11 rails x 2.35 + 10 modules x 32.4): `bazel run --config=opt //:print_bay -- --summary` |
| bay_length_mm | 11760 | mm | derived | docs/bay-hardware.md at 1213319, The bay: tube as shipped, 11,760 mm (derived) |
| fastenings | 51 | points | counted | docs/bay-hardware.md at 1213319, The bay: 11 rail to tube plus 40 module to rail, named sites in models/bay.xml; `bazel run --config=opt //:print_bay -- --summary` |
| container_gross_kg | 23800.0 | kg | assumed | docs/bay-hardware.md at 1213319, The container this project assumes: gross mass for transport 23,800 kg, tare 6,600 kg (both marked ours) |
| mass_left_for_cell_kg | 17200.0 | kg | derived | docs/bay-hardware.md at 1213319, The container this project assumes: 23,800 - 6,600 (derived) |

First version of this direction.

[Back to the README](../../README.md)
