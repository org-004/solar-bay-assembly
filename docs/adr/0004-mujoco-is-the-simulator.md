# 0004 MuJoCo 4.0.0 is the simulator, perception, planning and control are our own C++20

Status: accepted
Date: 2026-11-19
Decided by: the first engineer, commit 8adf3bd (MuJoCo, C++20); 83970cc
(`--config=opt`)

**Context.** The project is a simulated cell: a robot arm, a conveyor, a pallet, a
torque tube and cameras, in which a script picks solar modules and places them. The
repository began without a language, a planner or a robot model chosen
(`dd65a4d`, `README.md:16`). `AGENTS.md`, "Build and test", says unoptimized tests
take 5 to 70 times longer and that plain `-c opt` fails inside MuJoCo.

**Decision.** The simulator is MuJoCo, version 4.0.0, a `bazel_dep` at
`MODULE.bazel:12`, used through its C API (`@mujoco//:mujoco`, owning handles in
`src/mj_handles.h`). Perception, motion planning and control are written in this
repository against it: "hand-written in C++ against MuJoCo" (`README.md:6-7`, the
same sentence as in `8adf3bd`). OMPL is the one planning library, behind a switch
(ADR 0008). The language is C++20 for the project's code and for everything it builds
from source (`.bazelrc:1-3`). Builds and tests use `--config=opt`, which sets `-c opt`
and turns one MuJoCo error back into a warning (`.bazelrc:21-27`: GCC warns that a
variable in a MuJoCo file may be used before it is set, MuJoCo's build makes warnings
errors, so plain `-c opt` fails). MuJoCo and C++20 came in `8adf3bd` (2026-11-19);
`--config=opt` in `83970cc` (2026-11-19).

**Alternatives.** The repository records no alternative that was weighed, for the
simulator or for the language. For OMPL's flags it records one:
`docs/ompl-integration.md:1390` keeps the repository's C++20, since "C++17 for OMPL
alone would go against `.bazelrc`'s rule and gains nothing known".

**Consequences.** No reason for MuJoCo, for hand-writing the perception, planning
and control, or for C++20 is recorded; the comment at `.bazelrc:1` gives scope only.
What can be said today from the code: the whole cell depends on a physics engine whose
version is pinned, and 70 files include MuJoCo, so `planning` staying free of it
is worth keeping (ADR 0017). One C++ standard for dependencies means a flag change in
`.bazelrc` rebuilds MuJoCo; a new flag is scoped with `--per_file_copt` or per
target (the project brief, "Conventions beyond AGENTS.md"). Every command needs
`--config=opt`, which costs a reader who forgets it a failed or very slow run.

**Check.** P4 of `docs/architecture/principles.md`: `measure.sh` line `P4.mujoco_files`
counts the files that include MuJoCo and may not grow. Today
`tools/architecture/measure.sh | grep '^P4.mujoco_files '` prints
`P4.mujoco_files 70 files that include mujoco/ (may not grow)`.

**Migration.** None: the code already follows this.
