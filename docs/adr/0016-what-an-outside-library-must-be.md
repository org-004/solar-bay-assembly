# 0016 An outside library is added pinned, from source, licensed, behind one target

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R31)

**Context.** Each dependency is a cost that a security or platform change can force on
the project, and a diamond of incompatible versions is a failure C++ bears badly
[S43]. At `940a01b` everything outside is already pinned: `tools/architecture/
measure.sh` prints `P4 26` outside dependencies and `P4.unpinned 0`. Of the 26, 17 are
`boost.*` modules that came with OMPL (`grep -c 'bazel_dep(name = "boost'
MODULE.bazel` prints 17), a cost [S43] asks to have written down. `bazel mod graph`
fails on this repository, so nothing prints the tree and a rule cannot rest on it.
`MODULE.bazel.lock` and `.bazelversion` are committed [S44]. No licence file is
tracked, and for one owner a table of libraries and their licences is the
proportionate part of REUSE [S46]. OMPL is reached through one target, `//:ompl`,
and included by one file, `src/ompl_search.cc` (ADR 0003 fixes how outside code is
fetched). Research: `docs/architecture/research.md` (k).

**Decision.** A library is added to this repository only when all of these hold.
- It is pinned: by a version in `MODULE.bazel` (the registry), or by SHA-256 or
  commit in a repository rule under `bazel/` that has a `base_url` (ADR 0003).
- It comes from the Bazel Central Registry or is built from source here. A prebuilt
  binary is never taken.
- Its licence allows use and redistribution of the source, and the licence is
  recorded in `docs/architecture/dependencies.md` before the library is used.
- Where more than one target needs it, the rest of the code reaches it through one
  target of `BUILD.bazel`, as `//:ompl` is for OMPL [S3], and it is included only
  from the groups that `tools/architecture/deps.json` allows it in.
- It is added with a decision record that gives what it is for and what it costs.

The owner decides whether a library is added, through the spec that asks for it. An
agent that thinks one is needed says so in the plan and does not add it.

**Alternatives.**
- Vendoring the sources into the repository. Lost: the files become ours to keep
  current, and a version is no longer a line that anyone can compare.
- A system package. Lost: the build then differs between a development machine and
  CI, and between Debian 13 and Ubuntu 24.04; nothing pins the version.
- No rule. Lost: 17 modules came with OMPL and nothing prints their tree, so without
  a rule the cost of the next library would be seen only after it is in.

**Consequences.** Adding a library is slower: a record, a licence row and a pin come
first. The count `P4` cannot rise unseen, and a library no group may include fails
the check. The 17 Boost modules are inside the rule only as OMPL's dependencies:
`dependencies.md` says which OMPL needs, and removing the rest is a backlog item. The
rule does not say how a library is updated; `dependencies.md` does.

**Check.** P4 of `docs/architecture/principles.md`: `tools/architecture/measure.sh`
prints `P4` (outside dependencies, may not grow), `P4.unpinned` (fetches without a
version, commit or hash, stays 0), `P4.group` (includes of a library from a group
that may not, stays 0) and `P4.allowed` (pairs of group and library, may not grow);
`tools/architecture/measure.sh --check` exits 1 when one grows.

**Migration.** None: the code already follows this.
