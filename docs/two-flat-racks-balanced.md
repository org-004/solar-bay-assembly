# The two-rack cell's pin arms balanced by whole modules (run 028)

Run 028 tried to give pin arm A of `two_flat_racks_2_pin_arms` more than its 8 of the 36
holes, by whole modules, in a new layout file
(`layouts/two_flat_racks_2_pin_arms_balanced.json`). No split it tried plans: every split
that gives A more than modules 0 and 3 has a move of pin arm A that is not found. The file
on this branch is the last candidate, kept for the follow-up named at the end; it passes
the checker and does not plan. Nothing of this run is on `main`.

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
