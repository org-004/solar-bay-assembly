# 0007 A figure in a document comes from a run of `run_cell` or a `print_*` program

Status: accepted
Date: 2026-11-20
Decided by: the first engineer, commit 12d1127 (the habit, in `AGENTS.md`); the
present wording is from commit 4c5bf74 (2026-11-23)

**Context.** The documents of this repository carry figures that others build on:
the README's Status, the comparisons in `docs/`, the margins a report names. Agents
write them in parallel, and one agent's summary of a figure is a second-hand copy.
The programs that print a table are many (`run_cell`, `print_perception`,
`print_plan`, `print_check`, `print_bay`, `check_layout`, `plan_layout` and more,
each a `cc_binary` of `BUILD.bazel`), and each has been added together with the
stage whose figures it prints.

**Decision.** A figure in the README, a report or a commit message is reproduced with
`run_cell` or a `print_*` program at that commit, not copied from another agent's
summary; a change that moves a number in the README's Status updates it there
(`AGENTS.md`, "A number comes from a run"). The documents follow it and say so:
`docs/ompl-comparison.md:12-13` "Every figure below was printed by `print_plan` or
`run_cell`", `docs/layout.md:20-22` and `docs/flat-rack-cell.md:9` "Every result below
is printed by these commands". The habit is in `12d1127` (2026-11-20), where a stage
was verified by reproducing "the stage's table with `run_cell` or the `print_*`
program" and comparing it with the agent's figures; `4c5bf74` gave it its present
wording.

**Alternatives.** The repository records no alternative that was weighed.

**Consequences.** No reason for the rule is recorded; what can be said today is that a
figure has a command beside it that a reader can run, and that a wrong figure is
found by running that command. It costs a run per figure, and it ties the documents
to the programs: a program that stops printing what a document quotes breaks the
document without failing a test, which is why the programs are captured byte for byte
by `tools/architecture/behavior.sh` (ADR 0021) and why a new document of this
repository gives the command beside each figure it prints.

**Check.** P7 of `docs/architecture/principles.md` (a fact has one home) lists this
record under `Decisions`, and P2 holds the programs that print the figures: its line
`P2.unrun` counts the programs no capture runs. Today `tools/architecture/measure.sh |
grep '^P2.unrun '` prints `P2.unrun 11 cc_binary targets that programs.txt does not
capture`; `grep -n 'Every figure below' docs/ompl-comparison.md` prints line 12.

**Migration.** None: the code already follows this.
