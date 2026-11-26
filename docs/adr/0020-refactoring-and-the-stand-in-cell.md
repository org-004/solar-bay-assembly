# 0020 A refactoring does not edit the stand-in cell's files, and moves or renames them last

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R12 and R20)

**Context.** The stand-in cell (`models/cell.xml`, ten slots, a pallet of thirty) is
kept as it is until the new cell replaces it (ADR 0013), and the gate
(`tests/gate_test.cc`, `docs/gate.md`) runs on it: eight pallets, about four minutes,
the one test of the whole cycle. Its files are about half of `src/`. Cleaning code
that is about to be deleted is wasted work, and every edit to it risks the gate for
nothing. But the stand-in shares one directory, one build file and several headers
with code that stays, so "leave it alone" needs a list. Spec 013 names the family
(the cell, the cycle, pallet, conveyor, suction, the cameras, the runner, the gate)
and leaves the list to this record.

**Decision.** The stand-in cell's files are the files of the groups `standin`,
`perception` and `programs` in `tools/architecture/deps.json`, which marks those
groups `frozen`; the tests in those groups, `models/cell.xml` and the models it
includes belong to it. A refactoring does not change the contents of a frozen file:
not a line, not an include path, not a name. It may move or rename a frozen file only
as an item that comes after every item that does not, with the gate run before it
lands (`AGENTS.md`, "Build and test"). A feature that a spec asks for is not a
refactoring and is not bound by this. The arm's motion stack (group `arm`) and the
general parts (`base`, `planning`, `viewer`) are not frozen, although the stand-in
uses them: the new cell is to use them too, so they are worth improving, and the
captures and the tests hold what they do (ADR 0021). A change there that moves a
figure of the stand-in's programs is seen by `behavior.sh compare` and is then not a
refactoring. Every measure of `measure.sh` whose places are in files prints a line
`P<n>.frozen` for the part this record puts out of reach.

**Alternatives.**
- Freeze everything the gate depends on, the arm's stack and the planner included.
  Lost: that is nearly all the code that carries over to the new cell, and the
  backlog would hold only documents.
- Freeze nothing and rely on the captures. Lost: work on code that will be deleted,
  and `src/pick_place.cc`, the bottleneck of `AGENTS.md`, edited for style while
  features wait for it.
- Freeze by a list of files in this record. Lost: a second list beside `deps.json`
  that would drift; the groups are the list.

**Consequences.** The backlog is about the half of the code that stays. Departures
inside frozen files are counted, shown apart, and left: the targets of the principles
are their frozen figures until the stand-in is removed. A departure that crosses the
line (a file that stays including a frozen header) is cured on the side that stays.
The price: the frozen half keeps its long files and its duplicated figures until it
is deleted.

**Check.** `measure.sh` prints `P1.frozen_files`, the number of frozen files, which
may not grow, and `git diff --stat <before> <after> -- $(tools/architecture/measure.sh
--list P1.frozen_files)` is empty for a refactoring commit.

**Migration.** None: the code already follows this. `docs/architecture/backlog.md`,
"Left as it is", names the departures this record leaves.
