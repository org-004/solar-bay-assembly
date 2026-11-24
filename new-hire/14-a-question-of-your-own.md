# Stop 14: A question of your own

The last stop has no code to read. You ask one small question about the cell, answer it
with runs, and present the answer at office hours the way this repository reports
things. It is the job you were hired for, at the size of two days.

## Read

1. `AGENTS.md`, "Rules", once more. "A number comes from a run" and "What 'correct'
   means is the owner's" are the two that shape a report.
2. `docs/gate.md` as an example of the form: what was run, the settings, the result, a
   deliberate break and what it showed.
3. `docs/ompl-comparison.md` as the same form at full size. Read "Verdict" for a rule
   fixed before the numbers, "What is left unequal" for saying what a result does not
   show, and "How this was measured" for the commands.
4. `tools/plots/README.md`: how each figure in the README is tied to the command that
   produced its data.
5. Your notebook's "Ideas" list.

## The idea

**A good question** is small, has an answer a run can give, and changes one thing.
"Is the planner good?" is none of these. "How does the carry's duration change when the
planner checks edges every 15 mm instead of every 30?" is all three.

**Predict first.** Write down what you expect and why before running anything. A
prediction that turns out wrong is the most useful thing this stop can give you: it
marks exactly where your picture of the cell is off. The OMPL comparison did the
grown-up version of this: it wrote down what would count as "better" before it had a
single number, and then reported the verdict the rule gave.

**A baseline.** Every result is a difference from something. Run the unchanged cell on
the same seeds first, on the same machine, the same day.

**Seeds are samples.** One seed is an anecdote. `run_cell --trials N` gives N pallets,
and its summary reports the worst and the root mean square, which is the form to copy.
Use the same seeds for every row, so that rows differ only in what you changed. Do not
pick the seeds after seeing the results.

**Count, then time.** Collision queries, iterations and simulated seconds are the same
on every machine. Wall-clock milliseconds are not. Report the first kind as the result
and the second as a remark.

**Say what it does not show.** The simulation is kind in the ways stop 13 listed. A
result about cycle time on a welded grip and a rigid module is a result about that.

**The form.** A page, in this order:

1. The question, in one sentence.
2. Why a cell would care.
3. The prediction, made before the runs.
4. The setup: commit, machine, the exact commands, the seeds.
5. What was changed: flags, or a diff from a throwaway worktree.
6. The table.
7. What it shows, and what it does not.
8. What you would try next.

**A worked example.** Measured for this chapter on 2026-11-24 at commit `a091cc2`,
and again at `9397bdd` with the same figures.

*Question.* How does placement error grow with the check camera's calibration error,
and does the cell notice when it leaves the tolerance?

*Why.* The check after pickup removes the pallet camera's error and puts the check
camera's in its place (README). The gate assumes that error is 1 mm and 0.05 degrees.

*Prediction.* The error passes straight through, so placement error should grow in
proportion. Somewhere past the plausibility test's 0.2 degree limit the cycle should
start refusing.

*Setup.* Five pallets, seeds 1 to 5, one module each:
`bazel run --config=opt //:run_cell -- --pretty --trials 5 --check --calibration 1,0.05 --check-calibration MM,DEG`

| Check camera off by | Placed | Worst along, mm | Worst across, mm | Unconfirmed | Refused |
|---|---|---|---|---|---|
| nothing | 5 of 5 | 0.07 | 0.08 | 0 | 0 |
| 1 mm, 0.05 deg | 5 of 5 | 1.74 | 1.39 | 0 | 0 |
| 2 mm, 0.10 deg | 5 of 5 | 3.51 | 2.99 | 2 | 0 |
| 3 mm, 0.15 deg | 5 of 5 | 5.28 | 4.61 | 3 | 0 |

*What it shows.* The first half of the prediction held: two and three times the error
in, two and three times out. The second half did not. Nothing was refused at any
level; at three times the gate's assumption every module was "placed", up to 5 mm
off. The only sign was that some set-downs were blocked ("placement unconfirmed"),
which the trial runner counts as out of tolerance but the script does not treat as a
failure.

*What it does not show.* Five seeds; one module per pallet; a calibration error whose
direction is drawn at random, not one a real mounting would produce.

*Next.* Whether a second, independent measurement (the tube's rails seen by a camera
at the place, say) could catch what the plausibility test cannot.

**Where questions come from.** Your "Ideas" list first. If it is short, these are the
right size, and each can be done with flags or a one-line edit in a worktree:

- *Where do the seconds go?* From `--phases`, which steps are motion, which are waits
  and which are slow on purpose (`speed_touching`, the dwells, `look_time` in
  `src/pick_place.h`)? Which three changes buy the most cycle time, and what does each
  put at risk?
- *The planner's resolution.* `print_plan --resolution` against planning time, queries
  and clearance. Where is the knee, and what does the margin budget say you may do?
- *How many seeds are worth it?* `print_plan --plan-seeds`: the best of 1, 2, 4, 8
  plans against the carry's duration.
- *The envelope.* For which pallet offsets (`--xy`, `--yaw`) and camera errors
  (`--calibration`) does the cell still place in tolerance, and which failure appears
  first at each edge?
- *Blending.* `deviation_far` and `deviation_near` against cycle time and against how
  far the tool strays.
- *The deep end of the stack.* `--first 25 --modules 5`: what changes for the last
  modules of a pallet, in time, torque and the finder's error?
- *A paper.* Take one item from `reading.md` (TOPP-RA, Ruckig, cuRobo, VAMP). One page:
  what would it replace here, what would have to be true for it to help, and what
  experiment would tell.

The comparison with OMPL leaves questions of the right size too, each one flag away
with `print_plan --planner ompl` (its header lists the flags):

- *The row that decided.* BIT* on `--other-way` came out at 1.053 of the own planner's
  duration, against a limit of 1.05, in a row that does not repeat. Run that row many
  times. How much does it vary, and how often would the rule have said something
  else?
- *OMPL's own step.* `--ompl-range 0` lets OMPL choose its step where the switch gives
  it this planner's 3 m (the comparison's `ompl-range` rows). Reproduce one of those
  rows, then explain from the step lengths why it costs what it does.
- *A budget.* The planners that keep improving were given the own search's median
  queries, and ten times that. What does the curve between and beyond look like, in
  the trajectory's duration, which is what the cycle pays for?

The comparison does not decide whether OMPL should become the default search, and
neither should your page: that is the owner's decision. A result from this list is a
fact for it.

## C++ you meet here

Nothing new. You will read more of it than you write: most questions are answered with
flags, and the rest with one changed default in a worktree. If your question needs
more than a dozen changed lines, make it smaller.

## Lab

The lab is the question.

1. Choose it, write the first three parts of the form, and bring them to office hours
   before running anything.
2. Run the baseline and the variations. Keep every command.
3. Write the page. Put it in your notebook, or wherever your team keeps such things.
4. Present it at office hours in ten minutes.

## Pitfalls

- **A question that needs a week.** Halve it.
- **No baseline**, or a baseline from another day, another machine or other seeds.
- **Changing two things.** Then the table cannot say which one did it.
- **Reporting the best seed.** Report the worst and the spread.
- **Tuning until the gate passes.** A tolerance or a bound is the owner's. If your
  change needs one loosened, that is your finding; report it and stop.
- **Committing the experiment.** Lab edits live and die in the worktree. A result that
  ought to change the code is a proposal to the owner, with your page attached.
- **A number copied from somewhere.** Every figure in your table is from your own run.

## In the MoveIt cells

Ask the same question of a production cell, on paper, and notice what is missing.
There are no seeds, so two runs differ for reasons you did not choose. There is no
truth, so "placement error" needs an instrument and the instrument needs a
calibration. A trial costs a module and minutes, not five seconds. That is why
questions are settled here first where they can be, and why what the simulation is
kind about has to be on the page.

Then ask your team how they answer such questions today, and what they wish they
could measure.

## Reading

- Richard Hamming, "You and Your Research", a talk from 1986, widely reprinted. On
  choosing problems worth working on.
- `docs/ompl-comparison.md`, "Verdict" and "How this was measured": the standard of
  evidence this repository holds itself to.
- Mark Moll, Ioan Sucan and Lydia Kavraki, "Benchmarking Motion Planning Algorithms: An
  Extensible Infrastructure for Analysis and Visualization", IEEE Robotics and
  Automation Magazine, 2015: how the planning community compares planners.
- Any one paper from `reading.md` that touches your question, read properly: the
  method, then the experiments, then what they did not test.

## Office hours

Be ready to explain:

- your question, and why a cell would care about the answer;
- what you predicted and what happened;
- what your table does not show;
- what you would do with another two days.

Questions worth asking your teacher:

- Is there a cheaper experiment that would have answered the same question?
- Which of my result's assumptions is most likely to fail on a real cell?
