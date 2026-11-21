# Figures for the README

Each script reads a CSV a repo program printed and writes one PNG to `docs/img/`.
CSVs live outside the repo; below `$S` is any scratch directory and `$PY` a Python
with matplotlib and numpy (`python3 -m venv venv && venv/bin/pip install matplotlib numpy`).
Run Bazel commands from the repo root, one at a time, always `--config=opt`.
Scripts run from `tools/plots/`.

| Figure | Data | Plot |
|---|---|---|
| `pallet_depth.png` | `bazel run --config=opt //:print_depth -- --noise --pgm $S/depth.pgm` and `bazel run --config=opt //:print_depth -- --noise --points 2>/dev/null > $S/depth_points.csv` | `$PY pallet_depth.py $S/depth.pgm $S/depth_points.csv ../../docs/img/pallet_depth.png` |
| `check_depth.png` | `bazel run --config=opt //:print_check -- --noise --first-seed 1 --pgm $S/check.pgm 2>/dev/null > $S/check.csv` (one trial: the image as a PGM, and the finder's corners and centre in pixels plus the measured and true offsets as CSV on stdout; `$S` absolute) | `$PY check_depth.py $S/check.pgm $S/check.csv ../../docs/img/check_depth.png` |
| `placement.png` | four runs of `bazel run --config=opt //:run_cell -- --trials 20 ARGS 2>/dev/null > $S/FILE.csv`: `--carry planned` > `place_camera.csv`; `--carry planned --calibration 1,0.05` > `place_miscal.csv`; `--check --calibration 1,0.05` > `place_check.csv`; `--check --calibration 1,0.05 --check-calibration 1,0.05` > `place_check_miscal.csv` | `$PY placement.py ../../docs/img/placement.png $S/place_camera.csv $S/place_miscal.csv $S/place_check.csv $S/place_check_miscal.csv` |
| `carry.png` | `bazel run --config=opt //:print_plan 2>/dev/null > $S/plan.csv` (50 seeds) | `$PY carry.py $S/plan.csv ../../docs/img/carry.png` |
| `profile.png` | `bazel run --config=opt //:print_profile 2>/dev/null > $S/profile.csv` | `$PY profile.py $S/profile.csv ../../docs/img/profile.png` |
| `gate.png` | for N in 1..8: `bazel run --config=opt //:run_cell -- --seed N --modules 30 --calibration 1,0.05 --carry planned --check --check-calibration 1,0.05 2>/dev/null > $S/gate_seedN.csv` (the gate's settings; `run_gate` prints no per-module errors) | `$PY gate.py ../../docs/img/gate.png $S/gate_seed{1,2,3,4,5,6,7,8}.csv` |

Notes.

- `pallet_depth.png`: no program prints the finder's fitted outline. The orange
  box is made by the script from the same points, for display only: the glass
  points are picked with a trimmed plane fit and boxed along their principal axes
  (it is 1009 mm tall; the finder's own outline is 1100 mm, which includes the creases).
- `carry.png`: `print_plan` prints no paths or joint angles, only per-seed duration,
  length of motion, smallest clearance and the scripted carry's value.
- The gate seeds were run directly with `bazel-bin/run_cell` from its runfiles
  directory, eight at a time, for speed; the arguments are the same.
