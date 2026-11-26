# 0021 A refactoring proves itself by a capture before and after that compare the same

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R14 to R17)

**Context.** "Without breaking behavior" had no meaning a command could decide. The
tests show that pieces work, and they pass over many changes of behavior: no test runs
a program (`bazel query 'kind(cc_binary, deps(tests(//...))) intersect //:*'` prints
nothing), while the programs print every figure the documents state (`AGENTS.md`, "A
number comes from a run"). Agents that restructure code change what it does more often
than their tests notice ([S32] of `docs/architecture/research.md`), and the known cure
before restructuring is to pin what the code does now, right or wrong, and compare
([S15], [S17]). Two things stand in the way here: several programs print wall-clock
times, and the planner's draws differ between platforms (`src/rrt.cc:442`), so a
comparison holds on one machine only.

**Decision.** For a refactoring, behavior is what `docs/architecture/behavior.md`
lists, in three classes: what must stay, what may change if every reference changes
in the same commit, and what is free. A refactoring is shown not to have changed it by
`tools/architecture/behavior.sh capture <dir>` before the change and after it, on the
same machine, and `tools/architecture/behavior.sh compare` printing `same`: the same
bytes from every program of `tools/architecture/programs.txt` run with fixed
arguments, wall-clock fields cut, the same test targets and the same test results. A
comparison that differs is not explained away: the change is not a refactoring, and
the agent stops and reports. Code that neither a test nor a capture reaches
(`behavior.md`, "Outside the net") is not restructured until a test pins it, in a
commit of its own that changes nothing else. The gate is part of a capture only on
request (`--gate`); `AGENTS.md` says when it runs.

**Alternatives.**
- The tests alone. Lost: no test runs a program, and 24 of 99 files under `src/` are
  in no test's dependencies.
- Golden files committed under `tests/`, compared by a test. Lost for now: the outputs
  depend on the platform's random distributions (P6), so a committed file would fail
  on CI's x86-64; and committing 58 outputs makes every intended change of a table a
  conflict. A capture is made where and when it is needed and thrown away.
- Comparing numbers within a tolerance. Lost: a refactoring changes no number at all,
  and a tolerance would hide a reordered sum.
- A capture that includes the wall-clock fields. Lost: two runs of the same commit
  differ in them.

**Consequences.** An agent can show, in about three minutes without the tests
(`behavior.md` gives the measured times), that a change altered nothing a program
prints. The comparison is as
wide as `programs.txt`: a program or a flag that is not listed is not held, and the
viewers are not held at all. It holds on one machine; it says nothing about another
platform until P6 is met. A change that is meant to alter an output is by this
definition not a refactoring and needs its own reason.

**Check.** P2 of `docs/architecture/principles.md`. Two captures of one commit compare
`same` (`behavior.md` gives the command), and a change of one figure in a layout file
makes `compare` name `check_layout` for that layout.

**Migration.** The backlog items that serve P2 in `docs/architecture/backlog.md`:
tests that bring code into the net.
