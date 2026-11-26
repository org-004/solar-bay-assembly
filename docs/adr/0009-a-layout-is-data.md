# 0009 A cell's layout is a JSON file under layouts/, checked by one program

Status: accepted
Date: 2026-11-25
Decided by: the engineer of run 011, commit 68a1199, as the owner asked on 2026-11-24

**Context.** The owner decided on 2026-11-24 that the layout of the new cell is the
organization's to design and that "Layouts will be compared, so a quick way to describe
and check a layout is part of the work" (the project brief). Run 011 had five layouts to
compare: the small arm below, beside or above the tube, and three ways of putting the
rails on (`docs/layout.md`, "The layouts"). Each one is a model to look at and a set of
checks, and if the two were written separately for each layout they would drift apart.
The stand-in cell had no such thing: its layout is `models/cell.xml` and the code that
reads it, and the rule of `AGENTS.md` against changing the cell's layout gives way only
where a spec says so (the project brief).

**Decision.** One JSON file under `layouts/` is one layout, and nothing about a layout
is written anywhere else (`src/layout.h:6-10`). The model that is looked at
(`src/layout_model.h`, built at run time, nothing written to disk) and the checks
(`src/layout_check.h`) are both made from the file. `LoadLayout` refuses a file with a
missing or unknown field, a wrong type, an arm model that is not in `src/arm_catalog.h`
or a name used twice, and says which (`src/layout.h:13-16`). The program
`//:check_layout` (`src/check_layout.cc`) prints every row of every check as CSV for one
layout and exits 0 only when every row passes. It takes about four seconds for one
layout (the project brief). `layouts/` holds eight files: run 011's five and, since
940a01b (run 012), the three of the flat rack. The nlohmann_json library
(`MODULE.bazel:22`) was added with the format.

**Alternatives.** The repository records one comparison, of the layouts themselves
(`docs/layout.md`, "The comparison": `above` fails on clearance and is kept as the
failing case, `beside` passes and is not chosen), and a list of decisions inside it
(`docs/layout.md`, "Decisions"). It records no alternative to data read by a checker,
and no reason for JSON over another format: `MODULE.bazel:21` says what the library is
for, not why it was chosen.

**Consequences.** A layout is added by writing a file, and the model, the checks and the
planned motion (`src/layout_plan.h`) follow it without a change to code. A field added
to `src/layout.h` is a change to every file under `layouts/`, because an unknown or a
missing field is refused. The five layouts of run 011 had to be loaded in the new form
when the flat rack came (940a01b). A file is long, 875 to 1,757 lines (`wc -l
layouts/*.json`), because every figure of a layout is in it.

**Check.** `bazel run --config=opt //:check_layout -- --list` prints one line for each
file under `layouts/`, eight today, with the name, `chosen` for `flat_rack_3_pin_arms`,
the title and the container. The captures of `tools/architecture/programs.txt` run
`check_layout` for every name the list prints (P2, ADR 0021), so a layout that no longer
loads or checks differently is seen by `tools/architecture/behavior.sh compare`.
`layout_check_test`, `layout_scene_test` and `layout_rows_test` hold the checks
themselves.

**Migration.** None: the code already follows this.