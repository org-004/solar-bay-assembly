# 0003 Outside code comes from the registry or is fetched by hash

Status: accepted
Date: 2026-11-19
Decided by: the first engineer, commit 8adf3bd (`bazel/menagerie.bzl`); the same rule
again in 83970cc (`bazel/kuka.bzl`) and 9397bdd (OMPL)

**Context.** Some code and data the cell needs is in the Bazel Central Registry
(MuJoCo, Eigen, GLFW, GoogleTest, nlohmann_json, Boost) and some is not. A robot model
from MuJoCo Menagerie and the KUKA meshes are loose files inside repositories far
larger than the files used, and OMPL is a release archive with no Bazel files of its
own. A build must give the same bytes on every machine, and a file served from a host
that changes (or goes away) must not change a result silently.

**Decision.** A library that is in the registry is a `bazel_dep` of `MODULE.bazel`
(lines 9 to 25 and the Boost modules from line 121). What is not there is fetched
pinned, by the hash of each file. `bazel/menagerie.bzl` downloads the files of one
Menagerie model at one commit, "each checked against a SHA-256 hash" (lines 3-4, the
download at lines 20-26); `bazel/kuka.bzl` does the same for the KUKA meshes. Both
rules take a `base_url` attribute, "Point it at another host to self-host"
(`bazel/menagerie.bzl:49-52`, `bazel/kuka.bzl:57-60`), and `MODULE.bazel` lists every
file with its SHA-256 (lines 27-71). OMPL is an `http_archive` with `sha256` and
`urls` at `MODULE.bazel:109-115`, built by `bazel/ompl.BUILD.bazel`; to self-host,
the archive's address is put first in `urls` (lines 105-106). The rule is written in
the project brief ("Conventions beyond AGENTS.md"), not in `AGENTS.md`. The two rules
date from 2026-11-19 (`8adf3bd`, `83970cc`); OMPL's archive from 2026-11-24
(`9397bdd`).

**Alternatives.** For OMPL only, `docs/ompl-integration.md:1388-1389` records two
choices. The archive is the release asset upstream uploads (150 MB, with the
submodules, its hash published by upstream) and not GitHub's archive of the tag (77
MB, submodule directories empty). The fetch is `http_archive`, not a rule of the
repository's own with a `base_url`, because `urls` already takes a list and the
own rules exist for loose files in large repositories. For the two own rules the
repository records no alternative that was weighed.

**Consequences.** Why files are hashed is not recorded beyond the comment in
`bazel/menagerie.bzl`; what the code shows is that a changed file fails the build
instead of changing a result. The sources can be moved to a host of ours without
editing a rule. A new outside library costs a hash to look up and a rule to read
(ADR 0016 says what one must be). The convention for archives, `http_archive`, was
never added to the written convention.

**Check.** P4 of `docs/architecture/principles.md`: `P4.unpinned` counts fetches
without a version, commit, integrity or sha256. Today
`tools/architecture/measure.sh | grep '^P4.unpinned '` prints
`P4.unpinned 0 of them not pinned by a version, a commit, an integrity or a sha256`.

**Migration.** None: the code already follows this.
