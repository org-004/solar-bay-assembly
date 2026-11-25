# 0019 Namespaces stay as they are: cell, and one nested namespace for each family of the new cell

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R12)

**Context.** All 42 headers under `src/` are inside `namespace cell`
(`grep -l 'namespace cell' src/*.h | wc -l` prints 42). 31 open `cell` itself: the
stand-in cell and everything it shares (`src/rrt.h`, `src/profile.h`,
`src/mj_handles.h`). The 11 headers of the new cell open only a nested one:
`cell::layout` in seven, `cell::bay` in two, `cell::container` and `cell::flat_rack`
in one each. Nobody decided this in writing; it grew, the first `namespace cell` in
commit `8adf3bd`. The usual advice is a namespace for each library or directory.
Renaming to follow the ten groups of ADR 0017 (`cell::planning`, `cell::perception`,
and so on) would change the name of nearly every function and type, in files of the
stand-in cell that a refactoring does not edit (ADR 0020), for no dependency that the
include check does not already hold.

**Decision.** Namespaces stay as they are. The stand-in cell and the parts it shares
are in `cell`. Code of the new cell is in a nested namespace named after its family:
`cell::layout`, `cell::bay`, `cell::container`, `cell::flat_rack`. A new header opens a
namespace that its group allows; the list is `namespaces` of each group in
`tools/architecture/deps.json`. A new family of the new cell may add a nested
namespace by adding it to that list in the commit that adds the header. Nothing that
exists is renamed for the sake of its namespace.

**Alternatives.**
- A namespace for each group. Lost: a rename across about 30 headers and every file
  that uses them, half of them frozen, and the names of functions and types are
  behavior that may change only with every reference (`docs/architecture/behavior.md`).
- One flat `cell` for everything, the nested ones folded in. Lost: it renames through
  the eleven headers of the new cell and their users to remove the one mark that tells
  a reader which family a name belongs to.
- No rule. Lost: the next header would have to guess.

**Consequences.** Nothing changes today. A reader cannot tell a file's group from its
namespace for the 31 headers in `cell`; the group is in `deps.json` and in
`AGENTS.md`'s map. When the stand-in cell is removed, what is left in `cell` is small
enough to decide again.

**Check.** `measure.sh` line `P1.namespace`: headers under `src/` that open a namespace
their group does not allow.

**Migration.** None: the code already follows this.
