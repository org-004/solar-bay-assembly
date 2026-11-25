# 0001 Decisions about structure are kept as records in docs/adr, and an accepted record is superseded, not edited

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R9)

**Context.** Before this record the repository's decisions were written in several
places and in no common form: the "Decisions" sections of `docs/layout.md` and
`docs/flat-rack-cell.md`, `docs/ompl-integration.md` and `docs/ompl-comparison.md`,
the rules of `AGENTS.md` (mostly without the reason), comments in `.bazelrc`,
`MODULE.bazel` and `bazel/*.bzl`, commit messages, and the project brief, which is not
in the repository. An agent that is asked to improve the structure cannot tell a
choice from an accident, and re-opens what was settled. A short record of each
decision with its forces and consequences is the known cure ([S19], [S21] of
`docs/architecture/research.md`); what goes wrong with such records is that they are
edited after the fact, or pile up as unread essays ([S20], [S22]).

**Decision.** A decision about the repository's structure, its build, its tools or
its rules of work is one file `docs/adr/NNNN-short-title.md`, numbered from 0001 in
the order written, in the form `docs/adr/README.md` gives: a title that states the
decision, `Status:`, `Date:`, `Decided by:`, then Context, Decision, Alternatives,
Consequences, Check and Migration, in at most 80 lines. `docs/adr/README.md` lists
every record in one table.

The statuses are `proposed`, `accepted` and `superseded by NNNN`. Anyone, an agent
included, may write a record as `proposed`; nothing may be built on a proposed
record. A record becomes `accepted` in a run whose approved spec gives it that
authority, or by the owner. The owner may refuse an accepted record at any time: the
refusal is a new record that supersedes it.

An accepted record is not edited, except for its `Status:` line when a later record
supersedes it, and to correct a path that moved (P10 of
`docs/architecture/principles.md`). A decision that changes is a new record that
names the one it supersedes and says what changed in the forces. A change to the code
that departs from an accepted record comes with that new record in the same commit
(`AGENTS.md`, "Rules").

The decisions about the cell's design (the layout, the parts, the margins) stay where
they were made, with their figures; the second table of `docs/adr/README.md` points
to them. The owner's decisions about the product stay in the project brief.

**Alternatives.**
- Leave decisions where they are. Lost: the one sentence the owner wants to hand an
  agent, "improve the repo to be more in line with the design principles", has no
  decisions to be in line with.
- One document of all decisions. Lost: a long file is read once; a record is read
  when its subject is touched, and superseding one does not rewrite the others.
- MADR's full template with drivers and a pro and con list for each option ([S21]).
  Lost: more headings than these decisions have content for; its Confirmation section
  is kept, as Check.
- Records for the cell's design as well. Lost: a copy of figures that would drift
  from `docs/layout.md` and `docs/bay-hardware.md`.

**Consequences.** A reader finds why the structure is as it is in one place, and a
refactoring agent knows which conventions it may not overturn alone. The price is a
file for each decision and a table to keep; a record nobody checks decays, which is
why each names the command or the question that shows it is still true.

**Check.** `tools/architecture/measure.sh --check` fails when a document cites a
record that does not exist, and `ls docs/adr/[0-9]*.md | wc -l` equals the number of
rows of the first table in `docs/adr/README.md`.

**Migration.** None: the code already follows this.
