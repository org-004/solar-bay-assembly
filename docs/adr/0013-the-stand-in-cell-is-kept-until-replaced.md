# 0013 The stand-in cell is kept as it is, with the gate on it, until it is replaced

Status: accepted
Date: 2026-11-24
Decided by: the owner, 2026-11-24 (the project brief); kept by run 011, commit 68a1199

**Context.** The cell that runs today is `models/cell.xml` with `src/pick_place.*`, the
pallet, the conveyor, the cameras and the gate. It was built as a stand-in: ten slots on
the tube, a pallet of thirty modules, a square tube (`src/cell.h:142`, `docs/gate.md`,
"What it runs": 30 modules onto three 10-slot tubes). The real parts are not those. A
bay is nine modules and ten rails, a pallet holds 36, the tube is round
(`docs/bay-hardware.md`, the project brief). From 1213319 (2026-11-24) the real parts
exist as models beside the cell, and 68a1199 (2026-11-25) gave the bay's header, model
and document the number nine. The owner's decision about the old cell, in the project
brief: it "still has ten slots and a pallet of thirty, on purpose: it is not being
converted, it is being replaced."

**Decision.** The stand-in cell stays as it is, and the gate (`tests/gate_test.cc`,
`docs/gate.md`) runs on it, until the new cell replaces it. It is not converted to nine
modules or to a round tube, and its files are not edited to look like the new cell's.
The commits that brought the real parts left it unchanged: 1213319 ("The cell does not
change: models/cell.xml loads neither part and still carries its old square tube") and
68a1199 ("The stand-in cell is unchanged"); `AGENTS.md` and `README.md` say that the new
pieces are beside the cell, not in it. What a refactoring may touch of the stand-in is
ADR 0020 and is not repeated here.

**Alternatives.** The repository records one: converting the cell, which the owner's
line above rules out. It records no reason beyond the line, and none for the choice of
replacing over converting; what can be said is that the gate's tolerance is the owner's
assumption (`docs/gate.md`, "The tolerance") and that its figures are the only
whole-cycle check there is.

**Consequences.** The repository holds two families of code, the stand-in and the new
cell, that hardly meet (ADR 0017), and the stand-in's duplications and long files stay
until it is deleted. The gate keeps meaning what it meant. The new cell does not get the
stand-in's code: what carries over is named in the project brief (`src/rrt.h`,
`src/path_timing.h`, `src/viewer_lib.h`, `src/mj_handles.h`), and what does not is
`src/pick_place.*`, `src/pallet.*`, `src/conveyor.*`, `src/check_rig.*` and
`models/cell.xml`.

**Check.** `bazel test --config=opt //:gate_test` runs the gate on the stand-in as it
is, and the lines `P<n>.frozen` of `tools/architecture/measure.sh` show the departures
kept in its files (`P1.frozen_files` prints 66 files, a figure that may not grow). `git
diff --stat 9397bdd 940a01b -- models/cell.xml src/cell.* src/pick_place.* src/pallet.*
src/conveyor.* src/check_rig.* src/trial_runner.* tests/gate_test.cc docs/gate.md`
prints nothing.

**Migration.** None: the code already follows this.