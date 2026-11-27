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
weighed against v11; v10 stays chosen. `tests/plan_replay_staged_test.cc` replays it as the target
`plan_replay_staged_balanced_test` (`STAGED_LAYOUT`).

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
MARGINS_PLACEHOLDER
