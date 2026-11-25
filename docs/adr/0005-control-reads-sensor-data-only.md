# 0005 Control code reads sensor data only; a list of files says where truth is read

Status: accepted
Date: 2026-11-19
Decided by: the first engineer, commit 83970cc; `AGENTS.md` took the rule up in
12d1127 (2026-11-20) and 4c5bf74 (2026-11-23)

**Context.** The cell is simulated, so the simulation knows what a real cell never
does: where a module and the pallet truly are. A script that read those poses would
pass in simulation and mean nothing for the cell. The project brief lists, among what
the owner decided on 2026-11-24, that there is no fixed tooling that relies on tight
tolerances: the cell corrects itself from what it senses, because it works outdoors,
is moved and jostled, loses its calibration, and meets wind and glare. The scorer
needs the truth to say how well a run did, so truth cannot be removed, only kept out
of control.

**Decision.** The rule is `AGENTS.md`, "Sensor data only", and stays as worded:
control code reads depth images with the calibrated camera, joint angles, the
conveyor's encoder, and what the script did itself; `True...` accessors and body poses
out of the simulation are for the scene generator, the scorer (`src/score.h`) and
tests. The code shows it. `src/cell.h:304-321` puts the accessors in a section "For
setting up scenes, tests and scoring only" (`true_scene()`, `TrueModulePose`,
`TrueModuleSpeedOnTube`, `TruePlacedSlot`). `src/pick_place.h:22-25` and
`src/pick_place.cc:40-41` say the script calls no `True...` accessor and reads no body
pose, and `src/trial_runner.h:13-15` says the runner is "a scorer, not control code".
The list of files in `tools/architecture/deps.json` (`truth`, with `allowed`) is how a
command checks the rule: a file that matches a truth pattern and is not on the list is
counted. `tests/camera_cycle_test.cc:979` already checks one file: it reads the text of
`src/pick_place.cc` and fails if it names truth. The rule and the accessors came in
`83970cc` (2026-11-19).

**Alternatives.** The repository records no alternative that was weighed. Its contrast
is a run without sensing: `README.md:164` says that with `--blind` none of 20 trials
placed a module.

**Consequences.** The one reason written is at `src/pick_place.h:24-25`:
keeping truth out of the script "is what makes the score of a run a fair measure of
what each way of sensing costs". What it buys today is that the new cell's files
(`src/layout*`, `src/bay*`, `src/container*`, `src/flat_rack*`) read no truth. What
it costs: the check sees names and fields, not meaning, so a pose read under another
field, or through a helper in an allowed file, passes it; `src/collision.cc` and
`src/layout_check.cc` are on the list because they read poses out of a private copy
of the model.

**Check.** P3 of `docs/architecture/principles.md`: `measure.sh` line `P3`, files that
read truth and are not allowed. Today `tools/architecture/measure.sh | grep '^P3 '`
prints `P3 5 files that read the simulation's truth and are not allowed to (deps.json
truth)`, all five in the stand-in cell's frozen files (`P3.frozen 5`).

**Migration.** The backlog items that serve P3 in `docs/architecture/backlog.md`.
