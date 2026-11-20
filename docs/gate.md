# The gate

`//:gate_test` runs whole pallets of the pick and place in the simulated cell and fails if
any module is placed out of tolerance. It is part of `bazel test --config=opt //...` and
is tagged `gate`.

## What it runs

8 trials, seeds 1 to 8. Each trial is a whole pallet: 30 modules onto three 10-slot tubes,
with the tube swapped each time it is full. The settings (`cell::GateTrialSettings`,
`src/trial_runner.h`) are:

- the pallet camera, with noisy pictures;
- the planned carry, with the check after pickup;
- the pallet camera's calibration off by 1 mm and 0.05 degrees;
- the check camera's calibration off by 1 mm and 0.05 degrees;
- the default pallet randomization: up to 30 mm in x and y and 3 degrees of turn, plus the
  conveyor's stopping error.

In run_cell's flags this is `--seed N --modules 30 --calibration 1,0.05 --carry planned
--check --check-calibration 1,0.05`.

The test has four Bazel shards. Each shard runs two whole pallets, trial i in shard
i % 4, one trial at a time. One whole pallet is about 738 simulated seconds and takes
88 to 121 s of wall time on one core with `--config=opt`: about 88 s with the machine
otherwise idle, and 112 to 121 s with four shards running at once. The four shards take
about 4 minutes together (238 s measured on 2026-11-20, on a 9-core machine running
4 shards). That machine was shared with other builds, and the gate was not timed on a
real 4-core machine. On a slower CI runner the gate could take more than 5 minutes. In
the full `bazel test //...` it took 384 s, because it competed with the other tests. The trials are deterministic, so splitting them over shards or threads does
not change any result.

## The tolerance

This is an assumption chosen by the project owner. It is not a figure from a tracker
manufacturer. It is `cell::PlacementTolerance`, beside the runner. A placement is in
tolerance when all of these hold:

- the module rests on the rails. It must be at rest (score.h's `at_rest`: under 1 mm/s and
  0.01 rad/s) and lie within 2 mm of the rails' surface in height. The 2 mm is the
  runner's own choice, not one of the owner's figures. It tells a module lying on the rails
  (measured heights are within 0.01 mm) from one lying on something else.
- its position error in the rails' plane, sqrt(along² + across²), is at most **3 mm**;
- its yaw error is at most **0.15 degrees**, and so is its tilt error.

A cycle that fails, or whose placement is unconfirmed (the set-down was blocked and the
tool let go where it stopped), is out of tolerance whatever its numbers. A trial passes
when it ran all 30 cycles and every one is in tolerance. A failed cycle ends the trial.
An out-of-tolerance placement does not end it.

## Result on the unbroken tree

Measured on 2026-11-20 at commit 69b9087 plus this stage, using
`bazel test --config=opt //:gate_test`:

| | |
|---|---|
| trials passed | 8 of 8 |
| placements in tolerance | 240 of 240 |
| worst position in the rails' plane | 1.95 mm (seed 4, module 6) |
| worst yaw | 0.059 deg (seed 6, module 3) |
| worst tilt | 0.000 deg |
| wall time, 4 shards | 238 s |

## Running it

```
bazel test --config=opt //:gate_test --test_output=all          # the gate
bazel run --config=opt //:run_gate -- --trials 40 --save-failures /tmp/gate   # larger, by hand
bazel run --config=opt --test_sharding_strategy=disabled //:gate_test \
    -- --save-failures /tmp/gate                                  # the gate's 8, by hand
```

`run_gate` runs its trials on as many threads as the machine has cores, with one `Cell`
per thread (`--threads K` changes this). It takes `--seed S` for the first seed and
`--modules M` for a quicker look, though with fewer modules it is no longer the gate. It
prints one line per trial and every bad cycle with the command that reruns it, and exits
with status 1 if any trial failed.

## Replaying a failure

For each trial that does not pass, the gate writes one text file named
`failure_seed_<seed>_module_<module>.txt`. Under `bazel test` the file goes to the test's
undeclared outputs (`bazel-testlogs/gate_test/shard_<k>_of_4/test.outputs/outputs.zip`).
With `--save-failures DIR` it goes to DIR. The file holds:

- the trial's settings;
- its first bad cycle: module, slot, step, reason, why it is out, and the scorer's
  numbers in metres and radians, written so that they read back to the same double;
- two command lines that rerun it.

```
bazel run --config=opt //:run_cell -- --pretty --replay /tmp/gate/failure_seed_4_module_0.txt
bazel run --config=opt //:run_cell -- --pretty --seed 4 --modules 1 --calibration 1,0.05 \
    --carry planned --check --check-calibration 1,0.05
```

`--replay` reruns the trial up to its first bad cycle and prints the usual table. It then
says whether the failure is the same bit for bit: the same module, slot, step, reason and
verdict, and every number exactly equal. If anything differs it lists each difference and
exits with status 2. The second command is the same trial in plain flags, and needs no
file.

Replays are deterministic. The seed draws the cell's randomization, the picture noise and
the conveyor's stops. The planner's base seed is fixed. Nothing the script does depends on
wall-clock time, except the planner's 10 s time budget, against about 0.3 s used.
Wall-clock columns differ between runs: the finder's ms, the planner's ms and its
"fits" flag. These are measured, not simulated, and are not part of the comparison. The
gate's eight per-seed worsts were the same in four shards, single-threaded, as on eight
threads in one process. They also match run_cell tables made before the runner existed.

## Demonstration: the gate catching a break

The deliberate break was a plausible careless edit: the measured grip's correction
applied inverted at the place. The edit was made in `src/pick_place.cc` and reverted
afterwards. Where the release pose is planned, the line

```
held_in_tip_ ? CorrectedTipTarget(*release_module_, *held_in_tip_)
```

was replaced by `design_release * (*held_in_tip_ * DesignModuleInTip(...).inverse())`,
which is the design-to-measured correction the wrong way round.

The gate failed 8 of 8 trials and all 240 cycles were out of tolerance. 216 placements
were more than 3 mm off, with a worst of 7.82 mm. 120 placements were unconfirmed,
because the set-down was blocked. In seed 6 most placements were 2.4 to 3.0 mm off, and
only the unconfirmed set-down put them out. An excerpt of the red output:

```
seed 1: FAIL, 0 of 30 cycles placed in tolerance; worst position 7.82 mm, yaw 0.071 deg, tilt 0.000 deg; 773 s simulated in 119 s wall
tests/gate_test.cc:110: Failure
Failed
seed 1 out of tolerance: 30 of 30 cycles out of tolerance
  seed 1 module 0 slot 0: placed; position 6.27 mm (along 6.14, across 1.27), height -0.00 mm, yaw 0.026 deg, tilt 0.000 deg, at rest: placement unconfirmed (set down blocked)
  seed 1 module 1 slot 1: placed; position 6.22 mm (along 6.19, across 0.59), height -0.00 mm, yaw 0.020 deg, tilt 0.000 deg, at rest: placement unconfirmed (set down blocked)
  ...
  rerun: bazel run --config=opt //:run_cell -- --pretty --seed 1 --modules 1 --calibration 1,0.05 --carry planned --check --check-calibration 1,0.05
  saved failure_seed_1_module_0.txt in the test's outputs (outputs.zip)
...
//:gate_test                                                             FAILED in 4 out of 4 in 257.7s
```

A saved file, `failure_seed_4_module_0.txt`, written by the by-hand run with
`--save-failures`:

```
# seed 4, module 0 (cycle 0, slot 0): placed: position 4.18 mm > 3.00 mm
# module_0 in slot 0: along -2.89 mm, across +3.02 mm, height -0.00 mm, yaw +0.035 deg, tilt 0.000 deg, at rest (0.00 mm/s, 0.000 deg/s)
...
why = position 4.18 mm > 3.00 mm
along_tube = -0.002893938455875584
across_tube = 0.003015551493359414
```

Replaying it with the break still in place printed the same row (along -2.89, across 3.02)
and ended with:

```
replay of .../failure_seed_4_module_0.txt: the same failure, bit-identical (module, slot, step, reason, verdict and every number)
```

The unconfirmed failure of seed 6, module 0, also replayed bit-identical. The edit was
then reverted, and `git diff src/pick_place.cc` was empty.
