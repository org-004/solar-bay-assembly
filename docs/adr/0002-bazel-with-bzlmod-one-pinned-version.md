# 0002 The build is Bazel with bzlmod, one version pinned, one `BUILD.bazel`

Status: accepted
Date: 2026-11-19
Decided by: the first engineer, commit 8adf3bd

**Context.** The repository began empty: its first commit says "Language, planning
stack, and robot model are not chosen yet" (`dd65a4d`, `README.md`, 2026-11-19). The
code is C++20 against MuJoCo and Eigen, with GLFW, nlohmann_json and, since 9397bdd,
OMPL and seventeen Boost modules, and it is built and tested by agents on 64-bit ARM
machines and by CI on x86-64. All of it has to build from a fresh clone, and a result
printed on one machine is to be compared with one printed on another.

**Decision.** The build tool is Bazel, run through Bazelisk, which reads the version
from `.bazelversion` (`9.3.0`); `README.md:40-41` and `.github/workflows/ci.yml:47`
say so. Dependencies are declared as modules in `MODULE.bazel` (bzlmod), for example
`bazel_dep(name = "mujoco", version = "4.0.0")` at line 12, and the resolved graph is
committed as `MODULE.bazel.lock`. Every target of the code is in one file, the
top-level `BUILD.bazel`; `bazel/BUILD.bazel` is empty but for a comment, and only lets
Bazel load the rules in `bazel/`. The toolchain, the module file, the lock file and
the first `BUILD.bazel` came in one commit, `8adf3bd` (2026-11-19, "Toolchain: Bazel
build, MuJoCo stepping a UR10e from C++, viewer, tests and CI"). The lock file changed
at 9397bdd (OMPL and Boost) and 68a1199 (nlohmann_json).

**Alternatives.** The repository records no alternative that was weighed.

**Consequences.** No reason for Bazel, for bzlmod or for the pin is recorded; the body
of `8adf3bd` is its subject only. What can be said today from the code: a fresh
checkout builds with one tool and no system library, the lock file lets a second
machine fetch the same files, and a version change is one line in `.bazelversion`.
The costs are visible too. `bazel mod deps` and `bazel mod graph` fail on this
repository, a flaw of the `mujoco` module (the project brief, "Known flaws"), so the
lock file is checked by `bazel build --nobuild --lockfile_mode=error --config=opt
//...` instead. One `BUILD.bazel` is shared by every agent, which is why it has a
section per piece (`AGENTS.md`, "Dividing the work"); ADR 0018 keeps it that way.
`.bazelrc` is read by every build, so a flag there rebuilds MuJoCo (ADR 0004).

**Check.** P4 of `docs/architecture/principles.md`: `measure.sh` line `P4` counts the
outside dependencies in `MODULE.bazel` and `P4.unpinned` those without a version,
commit or hash. Today `tools/architecture/measure.sh | grep -E '^P4(\.unpinned)? '`
prints `P4 26 ...` and `P4.unpinned 0 ...`; `cat .bazelversion` prints `9.3.0`.

**Migration.** None: the code already follows this.
