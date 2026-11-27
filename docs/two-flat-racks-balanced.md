# The two-rack cell's pin arms balanced by whole modules (run 028)

Run 028 tried to give pin arm A of `two_flat_racks_2_pin_arms` more than its 8 of the 36
holes, by whole modules, in a new layout file
(`layouts/two_flat_racks_2_pin_arms_balanced.json`). No split it tried plans: every split
that gives A more than modules 0 and 3 has a move of pin arm A that is not found. The file
on this branch is the last candidate, kept for the follow-up named at the end; it passes
the checker and did not plan. Run 029 made it plan (the last section).

## Where the holes are

The stages move the tube by whole pitches, so every module is worked at one of four places
along Y: P0 (module centre y = -1.742: modules 0, 3), P1 (-0.598: modules 1, 4, 6), P2
(0.546: modules 2, 5, 7) and P3 (1.690: module 8). A KR 20 1.25 m from the tube's axis
reaches a hole only from within about 0.8 m of it along Y, and the two pin arms share one
line of track, so A works the places toward -Y and B those toward +Y. At P1 holes 0 and 1
lie at y = -1.146 and holes 2 and 3 at -0.050 (`check_layout`'s `reach` rows: track
position and the arm's offset, "opposite").

Holes 0 and 2 of a module are one pose family of a pin arm, holes 1 and 3 the other.
The move of pin arm A from hole 2 to hole 3 (and back, at the swage) flips the arm's
posture: joint 1 163 deg, joint 2 144 deg, joint 5 161 deg, joint 6 164 deg
(`plan_layout --why 135`). Pin arm B makes the same crossing at P1 from the +Y side.

## The splits tried

All probes are `bazel run --config=opt --config=agent //:plan_layout --
two_flat_racks_2_pin_arms_balanced --bays 1 --first-step N --last-step M [--why S
--budgets ...]`, on `8720ae3`. The time model is `check_layout`'s row `time,bay`.

| Split | A's work | Tracks A / B | Checker | Time model | What the planner showed |
|---|---|---|---|---|---|
| main (025) | 0, 3 (8 holes) | -2.6..-0.86 / -0.34..2.6 | 985 rows pass | 669.83 s | one bay 1003.47 s planned |
| S1 | 0, 1, 3 (12) | -2.6..-0.86 / -0.34..2.6 | 999 pass | 657.82 s | stage 1 (steps 1-62): 45 of 47; steps 36 and 42 (A, module 1, hole 2 to 3 and swage 3 to 2) not found at 160k, nor at 640k without the next modules (209 s) |
| S2 | as S1, A's base yaw -90 (joint 1 swings the other way) | as S1 | 999 pass | 657.28 s | the same two moves not found at 640k (202 s) |
| S3 | as S1 | -2.6..-0.52 / 0.0..2.6 | 43 rows fail | 656.92 s | B does not reach hole 1 of modules 4 and 6 (0.055 m short) |
| - | as S1 | -2.6..-0.67 / -0.15..2.6 | 55 rows fail | 656.73 s | B 0.001 m short of hole 1 of modules 4 and 6 |
| S4 | as S1 | -2.6..-0.72 / -0.20..2.6 | 999 pass | 656.73 s | stage 1: 45 of 47; steps 36 and 42 not found at 640k (210 s). (At A -0.62, with B's P1 holes given to A to pass the checker, step 36 is not found at 40k.) |
| S5 | 0, 1, 3, 4, 6 (20) | -2.6..-0.05 / 0.47..2.6 | 999 pass | 615.44 s | stage 3 (steps 119-200): 38 of 40; steps 135 and 141 (A, module 6) not found at 160k, nor 640k without the next modules (243 s). Step 93 (module 4) not found at 40k. Step 36 (module 1) found at 40k (35,422 iterations). Whole bay (`--bays 1 --summary`): exit 1, 123 of 127 moves found in 503.5 s; steps 93, 99, 135 and 141 not found |
| S6 | as S5, A's base yaw -90 | as S5 | 999 pass | 613.84 s | step 135 not found at 40k |

## What follows

- B must work every P1 module of stages 2 and 3 (modules 4 and 6): A's crossing there is
  not found with its track at -0.26 (run 022) or -0.05 (S5), either way round (S6), while
  B's is (run 024).
- B reaches hole 1 at P1 only with its track starting at -0.20 or lower, so A's track ends
  at -0.72 or lower (0.52 m between them for the carriage and the beam).
- With its track ending at -0.72 or lower A's crossing at module 1 is not found (S1, S2,
  S4); it is found with A's track at -0.26 (run 022) and -0.05 (S5).
- A cannot reach P2 or P3 while B covers P1 on the same line.

So with one line of track, whole modules and the planner as it is, pin arm A works modules
0 and 3 only: no split chosen. The candidate kept is S5, the split with the best time
model (615.44 s against 669.83 s, A 20 holes, B 16), because it is the one that would
plan if A's hole 2 to hole 3 move at modules 4 and 6 were found.

## For a later run

- The checker solving all four holes of a module in one pose family of pin arm A (the
  alternative the spec of run 028 kept for later). S5 would then need no family crossing.
- Or a second line of track for one pin arm, so that the tracks may overlap along Y.

## One posture per module (run 029)

Run 029 added the checker's opt-in `one_posture_per_module` (a field of an arm; `src/layout_check.cc`,
"One posture per module") and set it for pin arm A of this layout.

- The probe (`LAYOUT_CHECK_PROBE=pin_arm_a`, every candidate of every hole with its family, three
  bits from the joints' axes: shoulder, elbow, wrist): at modules 1, 4 and 6, at the track values
  the checker searched (the point opposite, +-0.6 m), holes 0 and 2 pass only in families 3 and
  7 and holes 1 and 3 only in family 2. Searching every 0.1 m of the track
  (`LAYOUT_CHECK_PROBE_WIDE=0`), families 0, 2, 4 and 6 reach all four holes with A's base behind
  them (toward -Y): for family 2 tracks -2.5, -2.1, -1.4, -1.0 m (holes 0 to 3), joint margin
  53.9 to 58.2 deg, clearance 0.0187 to 0.0193 m, both legs passing, largest joint change between
  holes 18.9 deg (it was 163 deg).
- With the opt-in the checker passes, 999 rows, time model 611.07 s (S5 615.44 s). The moves
  between A's holes are found at budget 5000 or less at every stage.
- The moves of A between its wait pose and hole 0 of modules 4 and 6 were then not found at 640k
  (stage 2 step 116, stage 3 steps 131 and 174): the wait pose, folded over the rack's -X side, is
  in the other posture under the modules set before. A waits now at (40, -30, 150, 0, 60, 0) deg,
  folded in the posture of its holes: step 174 is found in 20 iterations.
- The verifier (R21) refused a wait of A that the schedule put between locate 4/2 and pin 4/2,
  at the work pose, for pin arm B to pass (0.004 m to rail 5's top, near by design at that pose).
  Such a wait now counts as part of that work.
- One bay (`plan_layout -- two_flat_racks_2_pin_arms_balanced --bays 1 --summary`): exit 0, 127 of
  127 moves, 852.84 s planned (stages 227.20, 205.99, 254.66 s), against 1003.47 s planned and
  669.83 s modelled for two_flat_racks_2_pin_arms. Moves: module arm 41, pin arm A 40, pin arm B
  33, bolt arm 13; holes A 20, B 16. Least clearance 0.0180 m, arm to arm 0.0188 m, joint margin
  8.64 deg.

## Three bays (run 030)

`bazel run --config=opt --config=agent //:plan_layout -- two_flat_racks_2_pin_arms_balanced
--bays 3 --out plans/two_flat_racks_2_pin_arms_balanced.json` exits 0 with the layout of run 029
unchanged (sha256 `fb44f3a4…`): 381 of 381 moves found (three times bay 0's 127), `complete: true`,
verified and committed. It is version v12, a `candidate` of the direction of two 20 ft flat racks
weighed against v11; v10 stays chosen. `tests/plan_replay_staged_test.cc` replays it beside v11's plan: its tests of the
committed plan are parameterized by the layout's name (`.../Balanced`).

| Bay | tube in to bay out (s) | Stage 1 | Stage 2 | Stage 3 |
|---|---|---|---|---|
| 0 | 0.00 to 852.84 (852.84) | 227.20 | 205.99 | 254.66 |
| 1 | 805.66 to 1655.52 (849.86) | 226.87 | 205.40 | 252.59 |
| 2 | 1608.33 to 2457.34 (849.00) | 227.06 | 203.73 | 253.22 |

- `cycle_s` **819.11 s** ((2457.34 - 0.00) / 3), against v11's 969.67 s (-150.56 s, -15.5 %) and
  v10's 554.55 s (1.48 times as long). `one_bay_s` 852.84, `interval_s` 801.82. Bay n+1's tube
  comes in 47.19 s before bay n is out, both times.
- The time model: 611.07 s a bay against 852.84 s planned (+39.6 %; v11 +49.8 %). The modules
  phase is 493.02 s planned against 304.07 s modelled (v11 637.15 s), the rails 183.66 against
  129.89 s, the end 71.17 against 72.10 s; the two transfers are 60 s of every bay in both.
- Waits: 134 over the three bays, 6,629.28 s of arm waiting summed (module arm 1,432.5, pin arm A
  1,513.7, pin arm B 1,541.4, bolt arm 2,141.6), against 118 and 8,319.15 s for v11.
- Busy (moving or working) over the 2,457.34 s of three bays: module arm 1,024.8 s (42 %), pin
  arm A 943.7 s (38 %), pin arm B 915.9 s (37 %), bolt arm 315.7 s (13 %). In v11 B was busy 53 %
  and A 15 %. No pin arm is the bottleneck any more: the module arm is now the busiest, with A
  and B within 4 points of it, so the next second comes from the module arm's rail and module
  moves as much as from the pin arms.
- Planning: `planning_s` 119.1 s and 45,969,551 collision queries (v11: 252.1 s); the whole
  command took 219 s of wall time with a 44 s build. The costliest moves are pin arm B's from pin
  7/2 to locate 7/3 (4,228,547 queries in each bay).
- Tightest margins of the plan (the verifier over the schedule in four pieces): least clearance
  0.0180 m against 0.015 m (pin arm A's nose to the tube), least arm to arm 0.0188 m, least joint
  margin 8.64 deg against 5 deg. Pin arm A works at its track end -2.600 m and pin arm B at both
  of its ends, 0.470 and 2.600 m (0 slack, inside the layout's limits). The scorecard's gates:
  `check_rows_failed` 0 of 985 rows, `least_clearance_m` 0.018 >= 0.015, `mass_in_transit_kg`
  3,727 <= 13,608. `plan_replay_staged_test` is now `large`: the balanced layout's checker report takes about
  125 s in each shard.

## Where the time goes (run 032)

Bay 0 of the committed plan of v12 (`plans/two_flat_racks_2_pin_arms_balanced.json`, on `9d6efb2`)
is 852.84 s against its time model's 611.07 s: 241.77 s more. The account is printed, without
planning or building anything, by

```
python3 -B tools/time/account.py plans/two_flat_racks_2_pin_arms_balanced.json [--bay N]
```

**Per arm**, over the bay's 852.84 s (moving: the planned moves, settle included; model: the time
model's figure for the same moves, settle included; legs: approaches and retreats, which the model
does not time; process: the layout's process times; this layout has no tool change):

| arm | moves | moving planned | model, same moves | excess | legs (approach+retreat) | process | tool changes | waiting | busy % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| module_arm | 41 | 159.1 | 113.3 | +45.9 | 93.2 | 95.0 | 0.0 | 505.5 | 41 |
| pin_arm_a | 40 | 89.3 | 61.2 | +28.1 | 61.3 | 164.0 | 0.0 | 538.3 | 37 |
| pin_arm_b | 33 | 123.9 | 56.9 | +67.0 | 49.4 | 132.0 | 0.0 | 547.5 | 36 |
| bolt_arm | 13 | 29.7 | 28.3 | +1.4 | 15.5 | 60.0 | 0.0 | 747.6 | 12 |

**Waiting, by whom or what** (s; "bay out" is the wait from an arm's last step to the end of
bay_out, "stage end" the module arm's wait for the tube to move on):

| arm | bay out | bolt_arm | module_arm | pin_arm_a (passing) | pin_arm_b (passing) | stage end | transfer | tube in |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| module_arm | 200.5 | 25.8 | 0.0 | 0.0 | 0.0 | 174.2 | 60.0 | 45.0 |
| pin_arm_a | 182.3 | 0.0 | 234.4 | 0.0 | 16.5 | 0.0 | 60.0 | 45.0 |
| pin_arm_b | 61.8 | 0.0 | 376.3 | 4.5 | 0.0 | 0.0 | 60.0 | 45.0 |
| bolt_arm | 259.8 | 0.0 | 382.8 | 0.0 | 0.0 | 0.0 | 60.0 | 45.0 |

**The critical path**, walked back from bay_out through the waits (to the arm each wait names, or
to the transfer), covers the 852.84 s without a gap:

| from (s) | to (s) | s | the chain |
|---|---|---:|---|
| 0.00 | 45.00 | 45.00 | tube in, clamp |
| 45.00 | 109.19 | 64.19 | module arm: rails 0-3 |
| 109.19 | 118.66 | 9.47 | bolt arm: the stage's last bolt (rail 3) |
| 118.66 | 139.26 | 20.60 | module arm: take and set module 0 |
| 139.26 | 272.20 | 132.94 | **pin arm A: modules 0 and 1, back to back** |
| 272.20 | 302.20 | 30.00 | transfer to stage 2 |
| 302.20 | 348.97 | 46.78 | module arm: rails 4-6 |
| 348.97 | 358.44 | 9.47 | bolt arm: rail 6 |
| 358.44 | 377.69 | 19.25 | module arm: module 3 |
| 377.69 | 508.19 | 130.49 | **pin arm A: modules 3 and 4** |
| 508.19 | 538.19 | 30.00 | transfer to stage 3 |
| 538.19 | 582.18 | 44.00 | module arm: rails 7-9 |
| 582.18 | 591.95 | 9.76 | bolt arm: rail 9 |
| 591.95 | 630.01 | 38.06 | module arm: modules 6 (pin arm A's) and 7 |
| 630.01 | 787.84 | 157.83 | **pin arm B: modules 7 and 8** |
| 787.84 | 852.84 | 65.00 | release, bay out |

Every stage ends with the pin arm that has two of the stage's three modules: A at stages 1 and 2
(P0 and P1), B at stage 3 (P2 and P3); that is 421.26 s of the path. A module's twelve hole steps
follow each other, so the second pin arm cannot share them, and the other arm's one module is done
long before (B at stage 1 by 249.9 s, A at stage 3 by 670.5 s). Along the path, by kind:

# path covers 852.84 s of 852.84 s
| on the path | moving planned | model, same moves | legs | process | tool changes | waiting/gaps | events | total |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| module_arm | 101.6 | 77.1 | 61.3 | 70.0 | 0.0 | 0.0 | 0.0 | 232.9 |
| pin_arm_a | 71.4 | 47.4 | 47.5 | 128.0 | 0.0 | 16.5 | 0.0 | 263.4 |
| pin_arm_b | 65.6 | 23.0 | 23.8 | 64.0 | 0.0 | 4.5 | 0.0 | 157.8 |
| bolt_arm | 6.0 | 5.4 | 4.6 | 18.0 | 0.0 | 0.0 | 0.0 | 28.7 |
| tube in | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 40.0 | 40.0 |
| clamp | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 5.0 | 5.0 |
| transfer | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 60.0 | 60.0 |
| release | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 5.0 | 5.0 |
| bay out | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 | 60.0 | 60.0 |
| all | 244.7 | 153.0 | 137.2 | 280.0 | 0.0 | 21.0 | 170.0 | 852.8 |

**The 241.77 s by cause along the path**: moves slower than their model +91.69 s (pin arm B
+42.6 on 8 hole steps, 65.6 s against 23.0: its swing from hole 2 to hole 3, the posture change
that pin arm A no longer makes; the module arm +24.5; A +24.0); approach and retreat legs +137.19
s (not timed by the model at all: 0.12 m at 0.25 m/s is 0.73 s a leg, 2 to 4 legs a step);
waits on the path +21.00 s (A waiting 16.5 s for B to pass); the rest -8.11 s (the path's work at
the model's figures, 602.96 s, against the model's 611.07 s: the stages' barriers cost about what
the model's own one-after-another does).

The shares that a layout can remove (the legs are the approach lengths, a design figure kept; the
model simply does not time them):

| share on the path | s | what removes it |
|---|---:|---|
| the stage's last strap bolt between the module arm's last rail and its first module, three stages | 28.70 | `modules_start: after_their_rails`: a module is taken once its own two rails are bolted |
| pin arm A's module 6 taken and set before B's 7 at stage 3 | about 19 | order of modules 7, 8, 6 at stage 3 |
| pin arm B's hole-to-hole moves at stage 3 beyond their model | 42.6 | `one_posture_per_module` for B, as for A in run 029 |

The first two are one change, the order of work (layout field `order`), and the largest measured:
**797.18 s** for one bay (-55.66 s). The staged verifier held each stage's first module take
after the stage's last bolt even where the layout asks for `after_their_rails` (the planner already
scheduled by it), so it now takes the plain rule for that field (`src/layout_plan_stages.cc`,
`StagedFaults`); no committed layout has it. The third, alone on v12, gives **824.35 s** (-28.49 s); it shortens stages 1 and 2 as well, where pin arm A no longer waits for B to pass.

### The changed layout: `two_flat_racks_2_pin_arms_balanced_ordered`

`layouts/two_flat_racks_2_pin_arms_balanced_ordered.json` is v12's layout with `order` (modules
0, 1, 2, 3, 4, 5, 7, 8, 6; `after_their_rails`; `just_in_time`) and pin arm B's
`one_posture_per_module`. One bay, `bazel run --config=opt --config=agent //:plan_layout --
two_flat_racks_2_pin_arms_balanced_ordered --bays 1 --summary`: 127 of 127 moves found,
`complete: true`, **`bay_planned_s` 784.30 s** (-68.54 s, -8.0 %; time model 599.20 s, +30.9 %).

| one bay of | bay_planned_s | stage 1 | stage 2 | stage 3 to bay out | complete |
|---|---:|---:|---:|---:|---|
| v12 `two_flat_racks_2_pin_arms_balanced` | 852.84 | 227.20 | 205.99 | 254.66 | yes |
| order of work alone | 797.18 | 218.68 | 197.47 | 216.03 | yes |
| B one posture per module alone | 824.35 | 218.45 | 198.99 | 241.91 | yes |
| both: `..._balanced_ordered` | 784.30 | 209.93 | 190.47 | 218.89 | yes |

Its path: module arm 85.74 s (rails 0-3 and module 0), pin arm A 124.19 s (modules 0, 1),
transfer, module arm 66.98 s, A 123.49 s (modules 3, 4), transfer, module arm 63.54 s (rails 7-9,
module 7), pin arm B 155.35 s (modules 7 and 8, then a 13.3 s move back to its wait pose that
bay_out waits for), bay out. Still the heavy pin arm of each stage: the next share is the 0.73 s
legs (130.14 s on the path) and A's hole-to-hole moves (+24.0 s on the path). The plan is not
committed: three bays and a version are the next spec's.

Every committed plan re-plans byte-identically with the verifier's change in: `bazel run
--config=opt --config=agent //:plan_layout -- <name> [--bays 3] --out PATH` and `cmp PATH
plans/<name>.json` for all five (`two_flat_racks_2_pin_arms_balanced`, `two_flat_racks_2_pin_arms`,
`flat_rack_3_pin_arms_ordered`, `flat_rack_3_pin_arms`, `flat_rack_2_pin_arms`). The new layout
passes all 985 of `check_layout`'s rows.

## Three bays (run 033)

`bazel run --config=opt --config=agent //:plan_layout -- two_flat_racks_2_pin_arms_balanced_ordered
--bays 3 --out plans/two_flat_racks_2_pin_arms_balanced_ordered.json` exits 0 with the layout of run
032 unchanged (sha256 `d8b0934c…`): 381 of 381 moves found (three times bay 0's 127),
`complete: true`, verified and committed. It is version v13, a `candidate` of the direction of two
20 ft flat racks weighed against v12; v10 stays chosen, v11 and v12 stay candidates.
`tests/plan_replay_staged_test.cc` replays it beside v11's and v12's plans (`.../BalancedOrdered`).

| Bay | tube in to bay out (s) | Stage 1 | Stage 2 | Stage 3 to bay out |
|---|---|---|---|---|
| 0 | 0.00 to 784.30 (784.30) | 209.93 | 190.47 | 218.89 |
| 1 | 737.11 to 1514.89 (777.78) | 210.10 | 187.63 | 215.04 |
| 2 | 1467.70 to 2245.13 (777.43) | 210.79 | 187.21 | 214.43 |

- `cycle_s` **748.38 s** ((2245.13 - 0.00) / 3), against v12's 819.11 s (-70.73 s, -8.6 %) and
  v10's 554.55 s (1.35 times as long). `one_bay_s` 784.30, `interval_s` 730.24. Bay n+1's tube
  comes in 47.19 s before bay n is out, both times, as in v12.
- The time model: 599.20 s a bay against 784.30 s planned in bay 0 (+30.9 %; v12 +39.6 %). The
  modules phase is 416.22 / 409.82 / 410.29 s planned against 292.20 s modelled (v12 493.02 s),
  the rails 183.66 / 183.53 / 182.72 against 129.89 s, the end 79.42 against 72.10 s; the two
  transfers are 60 s of every bay.
- Waits: 116 over the three bays, 5,834.21 s of arm waiting summed (module arm 1,218.3, pin arm A
  1,302.8, pin arm B 1,383.7, bolt arm 1,929.4), against 134 and 6,629.28 s for v12.
- Busy (moving or working) over the 2,245.13 s of three bays: module arm 1,026.9 s (46 %), pin
  arm A 942.3 s (42 %), pin arm B 861.4 s (38 %), bolt arm 315.7 s (14 %).
- Planning: `planning_s` 145.8 s and 34,980,521 collision queries (v12: 119.1 s and 45,969,551).
  The costliest moves are pin arm B's from swage 8/0's retreat back to its wait pose (5,469,484
  queries in each bay).

Bay 1, the middle bay, by `python3 tools/time/account.py
plans/two_flat_racks_2_pin_arms_balanced_ordered.json --bay 1` (planned 777.78 s, model 599.20 s,
excess 178.58 s):

| cause along the critical path | s |
|---|---:|
| moves on the path slower than modelled (planned - model) | +80.85 |
| approach and retreat legs on the path (not timed by the model) | +130.14 |
| waits on the path | +0.00 |
| serialized work: the path's modelled moves, process and events less the model's bay (564.60 - 599.20) | -34.60 |
| total (planned - model) | +178.58 |

| arm | moves | moving planned | model, same moves | excess | legs | process | waiting | busy % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| module_arm | 41 | 151.4 | 113.3 | +38.1 | 93.2 | 95.0 | 438.2 | 44 |
| pin_arm_a | 40 | 88.8 | 61.2 | +27.6 | 61.3 | 164.0 | 463.7 | 40 |
| pin_arm_b | 33 | 105.7 | 50.4 | +55.3 | 49.4 | 132.0 | 490.6 | 37 |
| bolt_arm | 13 | 29.7 | 28.3 | +1.4 | 15.5 | 60.0 | 672.5 | 14 |

The largest remaining share is the approach and retreat legs the time model does not time,
+130.14 s on the path (pin arm A 47.5 s, the module arm 58.9 s, pin arm B 23.8 s), then the moves
slower than modelled, +80.85 s (pin arm B +42.1 s on the path, its 13.3 s return to its wait pose
after swage 8/0 among them; pin arm A +24.0 s). The path still runs module arm, then the pin arm
that holds two of the stage's three modules (A, A, B).

Tightest margins of the plan (the verifier over the schedule in four pieces,
`plan_replay_staged_test`): least clearance 0.0176 m against 0.015 m (pin arm A's nose to the tube,
in bays 1 and 2), least arm to arm 0.0188 m, least joint margin 8.64 deg against 5 deg (bay 0).
Pin arm A waits at its track end -2.600 m, pin arm B at 2.600 m and the bolt arm at 2.500 m (0
slack, inside the layout's limits); B now uses 0.719 to 2.600 m of its track, no longer its 0.470
m end. The scorecard's gates: `check_rows_failed` 0 of 985 rows, `least_clearance_m` 0.0176 >=
0.015, `mass_in_transit_kg` 3,727 <= 13,608. `scorecard_test` computes each committed scorecard in
a test of its own over three shards (one process took 725.1 s with v13's card, of large's 900 s;
three shards 276.7 s), and `plan_replay_staged_test` has the eternal timeout (slowest shard 371.4 s:
each shard makes the checker reports of both one-posture layouts).

## Faster retreats (run 036)

Every leg used to be timed at the planner's one pair of figures, `leg_speed` 0.25 m/s and
`leg_accel` 1.0 m/s^2 (`src/layout_plan.h`, `PlanFigures`). A layout may now opt in to
`retreat_legs` (`src/layout.h`): a leg that leaves contact into free air (a retreat) runs at its
`speed` (at most 0.5 m/s) and `accel` (at most 3.0 m/s^2), by the same law
(`RetreatDuration`, which also keeps every joint within its datasheet speed); an approach, which
ends in contact or insertion, keeps `leg_speed` and `leg_accel`. The plan file names
`retreat_speed` and `retreat_accel` among its figures only when they differ from the approach's,
so a layout without the field plans, writes and replays as before; the verifier holds a plan's
retreat figures to its layout's (`tests/retreat_legs_test.cc`).

Before any code, the critical path of v13's bay 1 (`tools/time/account.py`, its legs split by
kind; each retreat's length read back from its time and the rule's `retreat_m`):

| arm | approaches (s, n) | retreats (s, n) | gain at 0.5 m/s, 3.0 m/s^2 (s) | retreat lengths |
|---|---:|---:|---:|---|
| module arm | 27.18 (26) | 31.69 (23) | 15.11 | 0.15 m x10, 0.25 m x10, 0.40 m x3 |
| pin arm A | 23.76 (28) | 23.76 (28) | 10.04 | 0.12 m x28 |
| pin arm B | 11.88 (14) | 11.88 (14) | 5.02 | 0.12 m x14 |
| all | 62.82 (68) | 67.33 (65) | 30.17 | |

A 0.12 m retreat goes from 0.85 s (by acceleration) to 0.49 s, a 0.15 m one from 0.95 to 0.55 s,
a 0.25 m one from 1.50 (by speed) to 0.75 s and a 0.40 m one from 2.40 to 1.20 s.

`layouts/two_flat_racks_2_pin_arms_balanced_ordered_fast_retreat.json` is v13's layout and
`retreat_legs` {0.5, 3.0}, nothing else. `plan_layout -- two_flat_racks_2_pin_arms_balanced_ordered_fast_retreat
--bays 3` finds 381 of 381 moves, verified, in 145.7 s; it is v14, a `candidate` against v13.

| Bay | tube in to bay out (s) | Stage 1 | Stage 2 | Stage 3 to bay out |
|---|---|---|---|---|
| 0 | 0.00 to 751.62 (751.62) | 199.11 | 180.05 | 207.47 |
| 1 | 704.44 to 1452.04 (747.60) | 199.28 | 177.96 | 205.37 |
| 2 | 1404.86 to 2152.12 (747.26) | 199.97 | 177.54 | 204.75 |

- `cycle_s` **717.37 s** against v13's 748.38 s (-31.01 s, -4.1 %); `one_bay_s` 751.62 against
  784.30 s (-32.68 s); `interval_s` 700.07.
- Bay 0 against v13's bay 0 along the critical path: legs +130.14 to +99.97 s (-30.17 s, all of
  it the retreats: module arm 31.69 to 16.58 s, pin arm A 23.76 to 13.72 s, B 11.88 to 6.86 s;
  approaches unchanged at 62.82 s), waits +3.00 to +0.50 s; moves slower than modelled unchanged
  (+86.55 s). The path is the same chain: module arm, then pin arm A, A, B.
- The fastest retreat is the module arm's 0.40 m one at step 15, 1.20 s: joint 2 peaks at 0.514
  rad/s, 25.6 % of its datasheet's 2.007 rad/s.
- Margins: least clearance 0.0175 m against 0.015 m, arm to arm 0.0186 m, joint margin 8.64 deg
  against 5 deg; the track ends are v13's (A -2.600, B 2.600, bolt arm 2.500 m, 0 slack).
- Next: the moves slower than modelled (+80.85 s on bay 1's path: pin arm B +42.1 s with its
  13.3 s return after swage 8/0, A +24.0 s) are now the largest share; then the approaches
  (62.82 s), which this change leaves as they were.
