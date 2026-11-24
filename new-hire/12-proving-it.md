# Stop 12: Proving it

You can now change the cell. This stop is about how anyone, you included, finds out
whether a change made it better or worse: trials, a verdict, a gate that fails the
build, and a failure you can run again and get the same numbers to the last bit. Your
first real task will end with someone asking "how do you know?". This is the answer the
repository gives. About a day.

## Read

1. `docs/gate.md`, all of it.
2. The comment at the top of `src/trial_runner.h`, then `PlacementTolerance`,
   `JudgeCycle`, `FailureRecord` and `CompareFailure`.
3. In `src/trial_runner.cc`: `RunTrial` (one pallet, its cycles, the two hooks),
   `RunTrials` (the same on several threads) and, near the top, `Number` and
   `ParseNumber`.
4. `tests/gate_test.cc`, including its own `main`, and the `gate_test` target in
   `BUILD.bazel`.
5. `src/score.h`, and `CycleCheck::Allowed` and `CycleCheck::Observe` in
   `src/cycle_check.cc`.
6. One small test, closely: `RandomMovesStayWithinLimitsAndEndOnTarget` in
   `tests/profile_test.cc`. Then the bounds at the end of the statistics tests in
   `tests/motion_planner_test.cc`, with the comment that begins "No bound on the median
   wall time".
7. `.github/workflows/ci.yml`, and in the root `AGENTS.md` the section "Build and
   test" and the rules "The same on every platform" and "A number comes from a run".

## The idea

**What is being claimed.** "It works" is not a claim anyone can check. The repository
replaces it with three definitions.

- A **trial** is one pallet set down somewhere drawn from a seed, and some number of
  cycles on it. The seed draws everything random: where the pallet stands, the noise
  in each picture, the conveyor's stopping errors, the direction of each camera's
  calibration error.
- A **verdict** on each cycle: in tolerance or not. In tolerance means the module
  rests on the rails, within 3 mm of its slot in the rails' plane and within 0.15
  degrees in yaw and in tilt. A failed cycle is out. So is a cycle whose set-down was
  blocked, whatever its numbers.
- A **gate**: eight whole pallets, 240 modules, under the hardest settings the cell
  supports. One module out of tolerance and the test is red.

The 3 mm is the owner's assumption, and `docs/gate.md` says so. A tolerance is a
decision about the product, not something the code can discover. That is why the rule
"do not loosen a tolerance to make something pass" exists.

**Who judges.** The script never reads the truth; the judge does nothing else.
`src/score.h` measures where each module really lies. `CycleCheck` watches every
physics step for contacts that should not happen, for how close things came, and for
how hard the motors worked. A cycle can place its module perfectly and still have
scraped the camera post on the way; only the check would know.

**What 240 of 240 shows.** Less than it feels like. If something fails once in a
hundred times, the chance of seeing no failure in 240 tries is still about one in
eleven. A useful rule of thumb: after n tries with no failure, you can say with 95%
confidence that the failure rate is below about 3/n. For 240 that is 1.25%. For the 900
placements the README reports, 0.33%. At one module every 26 seconds a cell places over
3,000 a day, so "below 0.33%" still allows about ten bad ones daily.

And the 240 are not 240 independent tries. Each pallet draws one calibration error and
keeps it for its 30 modules, so for anything that depends on that draw the gate has
tried eight things, not 240. The gate is a tripwire: it shows that a change has not
broken what worked. It does not measure how often a real cell would fail, and it
cannot find a failure the simulation does not model (the README lists where the
simulation is kinder than a real cell).

**The same bits every time.** Run a trial twice and every number comes out identical,
on any machine. That is not automatic; it took decisions you have met along the way:

- everything random is drawn from the trial's seed, and the planner's seed is fixed;
- the draws are written out by hand, because the standard library's are not the same
  on every platform (stop 6);
- wall-clock time is measured and printed, and nothing acts on it;
- each thread has its own `Cell`, so trials cannot disturb one another;
- tests bound counts (collision queries, iterations), not seconds.

What it buys is that a failure has an address. "Seed 4, module 6" is the whole bug
report.

**A failure you can replay.** When a trial fails, the runner writes a small text file:
the settings, the first bad cycle, the scorer's numbers and the command that reruns it.
The numbers are written so that reading them back gives the same `double` exactly.
`run_cell --replay FILE` runs the trial again and compares every field. It answers a
question that is otherwise hard: after your fix, is this the same failure, a different
one, or gone?

**Tests as measured claims.** Read the tests as statements somebody measured and then
fenced with a little room: the planner's carry takes fewer than 50,000 collision
queries at the median; a random move stays inside its limits. They run from the cheap
(a profile's arithmetic, milliseconds) to the gate (240 cycles, minutes). When one goes
red the question is always the same: did the code get worse, or was the claim wrong?
You may fix the first. The second is the owner's.

**Why CI looks the way it does.** The whole suite in one job took 33 minutes on a
two-core runner, so `ci.yml` divides it over six jobs that run side by side, four of
them a quarter of the gate each. A disk cache keeps test results, so a test whose
inputs did not change is not run again. Everything is built optimized, because the
cycle tests are many times slower otherwise.

## C++ you meet here

**Threads.** `RunTrials` starts several `std::thread`s, each running the same lambda,
and `join`s them: waits for each to finish. Unlike Python, C++ threads really run at
the same time on separate cores, and nothing stops two of them writing the same
variable at once. That is a *data race*, and its result is not a wrong number but
undefined behaviour.

**What is shared, and how.** Read `RunTrials` asking of each variable "who can touch
this?". The trial counter `next` is a `std::atomic<int>`: `next++` on it is one
indivisible step, so no two threads get the same trial. Loading a model is wrapped in a
`std::mutex`, held by a `std::lock_guard`, which is stop 1's ownership idea again: the
lock is released when the guard goes out of scope, on every path. Each thread writes
only its own slot of `results`. Everything else, the `Cell` and the script on it,
belongs to one thread. "One `Cell` per thread" is why so little locking is needed.

**A number as text, exactly.** `printf("%f")` rounds. `std::to_chars` writes the
shortest text that reads back, with `std::from_chars`, to the very same `double`. A
replay that compared rounded text could only say "close". With these it can say
"identical".

**GoogleTest.** `TEST(Suite, Name)` defines a test. `TEST_F` uses a *fixture*, a class
whose `SetUp` runs before each test; fixtures are the only inheritance and the only
`override` in the repository. `EXPECT_...` records a failure and carries on;
`ASSERT_...` stops the test, for when carrying on would crash. Floating-point results
are compared with `EXPECT_NEAR` and a stated tolerance, never `==`, except where bit
equality is the point, as in the replay tests. `SCOPED_TRACE` adds a line (which seed,
which trial) to any failure reported inside its scope.

**The environment, and shards.** `std::getenv("NAME")` returns the variable's text, or
a null pointer if it is not set, which must be checked. Bazel can split a slow test
into *shards* run as separate processes; it tells each one its number through the
environment. `gate_test.cc` reads that and takes every fourth trial. It has its own
`main` so that it can also take `--save-failures`.

**Counts, not seconds.** A test that says "planning takes under 400 ms" passes on your
desk and fails on a loaded CI runner. A test that says "under 50,000 collision queries"
measures the same work and gives the same answer everywhere. The time is still
printed, for people.

## Lab

Predict first, and write it down. Runs 3 and 4 edit code in a throwaway worktree
(`new-hire/AGENTS.md`, "Labs").

1. **A small gate.**
   `bazel run --config=opt //:run_gate -- --trials 2 --modules 3 --threads 2`. Read one
   trial's line: what settings did it run with, and what is its worst placement? Then
   `echo $?`. Why is this not the gate?
2. **The same trial twice.** Run this twice, into two files:
   `bazel run --config=opt //:run_cell -- --seed 4 --modules 2 --calibration 1,0.05 --check --check-calibration 1,0.05 > /tmp/a.csv`
   (then `/tmp/b.csv`), and `diff /tmp/a.csv /tmp/b.csv`. Before you look: will they be
   identical? Which columns, if any, are allowed to differ, and why those?
3. **Catch a break, save it, replay it.** In the worktree, make stop 2's edit again
   (the grip correction the wrong way round, in `PickPlace::PlanPlace`). Then:
   - `bazel run --config=opt //:run_gate -- --trials 2 --modules 3 --threads 2 --save-failures /tmp/gate-lab`.
     Read what it prints and open one of the files it wrote.
   - Replay it with the break still in:
     `bazel run --config=opt //:run_cell -- --pretty --replay /tmp/gate-lab/<file>`.
   - Undo the edit and replay the same file. What does the last line say now, and
     what is the exit status?
4. **A test that fails.** `bazel test --config=opt //:profile_test`. Then, in the
   worktree, in `RandomMovesStayWithinLimitsAndEndOnTarget`, change the last
   `EXPECT_GE` to `EXPECT_LE` and run it again. In the output, find the file and line,
   the two values compared, and the line that says which trial. Where did that last
   line come from?
5. **The gate itself.** Alone on the machine, with about four minutes to spare:
   `bazel test --config=opt //:gate_test --test_output=all`. Compare its worst
   placement with the figure in `docs/gate.md`. Are they the same to the last digit
   printed? Should they be?

## Pitfalls

- **Reading "all passed" as "it works".** Count the independent tries, and ask what
  was not randomized.
- **Loosening the bound.** A red test with a tolerance beside it is an invitation.
  Report what you found.
- **A wall-clock bound in a test.** It will fail on somebody else's machine, on a day
  when nothing is wrong.
- **Sharing by accident.** A `static` variable, or one object handed to two threads,
  and results change with the number of threads. "One per thread" is the rule for
  cells, checkers and random generators alike.
- **Order of evaluation.** `f(draw(), draw())` may draw in either order; the compiler
  chooses, and ARM and x86 builds here choose differently. One draw per statement.
- **Scoring with the wrong clock.** The placement is measured the moment the module is
  let go and the tool has retreated, before the tube moves. Measure later and you are
  scoring the conveyor as well.

## In the MoveIt cells

A ROS 2 stack is tested in layers too. Single functions get unit tests, in GoogleTest
or pytest. Launch tests start a few real nodes and assert on what they publish. Sensor
data recorded in a bag can be played back into the perception and planning nodes. And a
simulator can stand in for the robot in CI.

None of these repeats bit for bit. The nodes are separate processes on the wall clock,
messages arrive in slightly different order from run to run, and sampling planners
draw fresh random numbers unless someone has pinned their seed. A failure seen once
may need many runs to see again. That is the largest practical difference from this
repository, and a reason to keep a planner's core as plain library code that can be
tested like this, with ROS only at the edge.

A real cell has no truth to score against. Whether a module is within tolerance has to
be measured by something outside the cell's own sensing, or the cell is marking its
own homework with the calibration that may be wrong. The production counterpart of the
gate is the acceptance test, at the builder's site before a cell ships and again where
it is installed: a number of consecutive cycles at the agreed rate, with the agreed
measurement, that must all pass.

For your notebook: what is the acceptance test for a production cell, how many cycles
is it, and what measures the placement?

## Reading

- James Hanley and Abby Lippman-Hand, "If Nothing Goes Wrong, Is Everything All Right?
  Interpreting Zero Numerators" (JAMA, 1983). A few pages; the source of the 3/n rule.
- GoogleTest's primer, <https://google.github.io/googletest/primer.html>. All of it:
  tests, fixtures and assertions. `SCOPED_TRACE` is in its advanced guide.
- Bruce Dawson, "Floating-Point Determinism" (Random ASCII, 2013),
  <https://randomascii.wordpress.com/2013/07/16/floating-point-determinism/>. What
  does and does not make two machines agree to the last bit.
- Nidhi Kalra and Susan Paddock, "Driving to Safety: How Many Miles of Driving Would
  It Take to Demonstrate Autonomous Vehicle Reliability?" (RAND, 2016). The same
  arithmetic as the 240, at the scale of an industry. The summary is enough.
- ISO 9283, "Manipulating industrial robots: Performance criteria and related test
  methods". The standard behind "accuracy" and "repeatability" on a robot's
  datasheet. Know that it exists and what it defines.

## Office hours

Be ready to explain, with the chapter closed:

- what a trial, a verdict and the gate are, and who chose the tolerance;
- what 240 placements with no failure do and do not show;
- three things this code does so that a run repeats exactly;
- what a failure file holds, and what a replay tells you after a fix;
- why the tests bound query counts and not milliseconds.

Questions worth asking your teacher:

- The gate uses eight pallets. What would a ninth buy, and what would a different
  kind of trial buy?
- Which of the cycle check's findings would a real cell be able to make for itself?
