# Bringing OMPL into this repository: the options, built and measured

This report is for deciding how OMPL (the Open Motion Planning Library, C++, built with
CMake, absent from the Bazel Central Registry) should be brought into this repository,
and what the comparison of its planners with the hand-written one in `src/rrt.h` will
need. It integrates nothing: `main` is unchanged but for this file. Four ways of
bringing OMPL in were built on branches that have no pull request, and measured; four
more were assessed on paper. The numbers from a made-up world under `## Measurements`
show that OMPL plans here; they are not the comparison.

Written 2026-11-23 against `main` at `014ba3a` and OMPL 2.0.2 (`main` has since gained
one commit, which changes `AGENTS.md` only). Every figure was either measured for this
report, with its command under `## How this was measured`, or is cited
there with the date it was read. What was neither says "not verified".

## Summary and recommendation

Four options were built and measured (O1 to O4); four were assessed on paper. "n.v."
is "not verified". Times are a cold and a cached build of `//:ompl_smoke` on six cores.

| Option | How it works | Host needs | Existing versions | Cold; cached | Upgrade | Pin; self-host | Reuse | Optional parts | Licence | Breaks on |
|---|---|---|---|---|---|---|---|---|---|---|
| **O1 native** (built) | `http_archive`, own build file (41 + 125 lines) | none | none moves; +82 Boost | 148 s; 5 s; per file | 3 values; re-read OMPL's CMake | SHA-256; `urls` | copy 2 blocks, 1 file | FLANN small; VAMP, Python: new build files | BSD notice | upstream build changes |
| **O2 foreign** (built, hermetic) | OMPL's CMake via `rules_foreign_cc` (56 + 97 lines, shim, patch) | none | 3 move; +89 | 183 s; 4 s; one action | 3 values; patch must apply | as O1, and a fetched CMake | copy 4 files | each needs a CMake shim | as O1 | `-O3`, not the repository's flags |
| **O3 ROS registry** (built) | 2 `--registry` lines, 1 `bazel_dep`, 4 Python lines | none | 16 move; +485; viewer's libraries rebuilt | 203 s; 6 s; per file | wait for that registry | lock file; mirror | the same lines | VAMP, Python targets exist | BSD; overlay Apache 2.0 | graph follows ROS; links `libpython` |
| **O4 own module** (built) | O1's build file in `registry/`; 1 `bazel_dep`, 2 `--registry` lines | none | none moves; +83 | 133 s; 3 s; per file | new version directory, 2 integrity values | integrity; the commit | 1 registry line, 1 `bazel_dep` | as O1 | as O1 | registry edits unseen until restart |
| **O5 upstream** | O4's module in the central registry | none | n.v. | as O1 | a registry pull request per release | registry, lock file | 1 line, for anybody | users would ask | as O1 | untried platforms; standing duty |
| **O6 in-tree** | source in `third_party/ompl/`, O1's build file | none | as O1 | as O1 less the download (n.v.) | replace a 5.7 MB tree | the commit | copy the directory | as O1 | repository redistributes source | local edits; history grows |
| **O7 prebuilt** | system package, wrapper rule | package, its Boost and Eigen | Bazel's unchanged; risk of a second Eigen | nothing built | the packager decides | not pinned here | same install step | packager's choice | packager's | `apt`: 1.6.0 here, 1.5.2 on CI |
| **O8 outside Bazel** | CMake project or `pip install ompl` | CMake and OMPL, or a wheel (2.0.1) | untouched | none | a version in a file | outside Bazel | none | the wheel's | not distributed | no in-process collision check |

**Recommended: O1 native.** O1 and O4 are the two built options that left the existing
build exactly as it was (no module version moved, and the test suite took every
existing action from `main`'s cache) and needed no patch to OMPL; O2 moved three
modules and compiles OMPL at `-O3`, O3 moved sixteen and rebuilds the viewer's
libraries. O1 and O4 both compile OMPL with the repository's compiler and flags, which
a yardstick for the repository's own planner should have. Of the two, O1 is the one with
nothing in it that a single project does not need: three files, no registry lines in
`.bazelrc`, no integrity values kept by hand. Its cost is that its 75 lines of build
rules restate OMPL's CMake and have to be checked against it at each upgrade.

**Runner-up: O4 own module.** It builds exactly what O1 builds and was made from O1 in
five minutes. It becomes the better choice on the day a second Bazel project needs
OMPL, or a second library is packaged the same way and a registry of the owner's is
worth having.

## Options

Each option is described under the same ten criteria. A statement followed by a tag
such as [S5] is taken from the source of that number in the list at the end of
`## How this was measured`; a statement about a spike was measured for this report and its
command is in that section too. "Registry" without a name means the Bazel Central
Registry.

What every option that compiles OMPL has to reproduce is small, and it helps to have it
in one place [S1]:

- The core library is every `.cpp` under seven directories of `src/ompl`
  (`base control datastructures geometric multilevel tools util`): 249 files at 2.0.2.
  Two more directories are conditional: `extensions/triangle` (2 files, compiled only
  with Triangle) and `vamp` (headers only, used with the VAMP submodule).
- One generated header, `ompl/config.h`, from `config.h.in`: four version placeholders
  and three `#cmakedefine01` switches (`OMPL_HAVE_FLANN`, `OMPL_HAVE_SPOT`,
  `OMPL_HAVE_NUMPY`). There are no compile-time feature checks.
- One definition that matters for OMPL's own files,
  `BOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS`. OMPL's CMake also sets
  `_HAS_AUTO_PTR_ETC=0` and the warning flags `-W -Wall -Wextra`; O1 and O4 pass
  neither, O2 gets both from CMake.
- OMPL's CMake links one Boost library, Boost.Serialization, with Eigen and the thread
  library. Its headers include 17 Boost libraries by name. Under Bazel one of those,
  `boost.thread` (one include in OMPL), brings compiled code of its own, so the link
  line of O1 also carries the archives of `boost.thread`, `boost.chrono`,
  `boost.container` and `boost.filesystem`.
- OMPL's CMake asks for C++17 and builds Release as `-O3 -DNDEBUG`.

### O1 native

**How it works.** `MODULE.bazel` fetches OMPL's release archive with `http_archive` and
names a build file kept in this repository; Boost comes from the registry, one module
per Boost library; Eigen is the repository's. Built on `spike/ompl-native`. The lines it
adds to `MODULE.bazel` (41 with comments):

```
http_archive = use_repo_rule("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "ompl",
    build_file = "//bazel:ompl.BUILD.bazel",
    sha256 = "b9155e243e555b04b01f8e4ff7e21a48582edbab9c300a611289655778bcd29d",
    strip_prefix = "ompl-2.0.2",
    urls = ["https://github.com/ompl/ompl/releases/download/2.0.2/ompl-2.0.2.tar.gz"],
)

bazel_dep(name = "boost.algorithm", version = "1.90.0.bcr.1")
# ... 16 more boost.* lines: assert, concept_check, config, dynamic_bitset, foreach,
# functional, graph (1.90.0.bcr.2), math, numeric_odeint, property_map, random, range,
# serialization, smart_ptr, thread, type_traits
```

Build files: `bazel/ompl.BUILD.bazel`, 125 lines, 75 of them not comments or blank. It
holds a `genrule` that makes `config.h` with `sed` (the version is read from OMPL's
`CMakeLists.txt`, and the rule fails if a placeholder is left unfilled) and one
`cc_library` with `glob` over the seven directories, `includes = ["src"]`,
`local_defines` for the one definition, `linkopts = ["-pthread"]` and the 17 Boost
modules and `@eigen` as `deps`. In `BUILD.bazel`, an `alias` `//:ompl` tagged `manual`.
No patch to OMPL and no `copts`.

**On the host.** Nothing beyond the compiler and the shell tools every Bazel `genrule`
already needs (`sh`, `sed`, `grep`). No package was installed for it.

**Existing dependency versions.** Unchanged: the module graph gains 82 modules, all of
them Boost (`boost` and 81 `boost.*`), and no module `main` used changes version (see
Measurements).

**First build and cache.** 147.5 s on six cores from nothing, in 340 actions. 253 MB
is downloaded, 150 MB of it OMPL's archive. One action per source file, so a
disk or remote cache stores each object by itself: after `bazel clean` the same build
took 5.1 s, all 335 compile, archive and link actions being cache hits. Because OMPL is
compiled with the flags of the build it is in, a change to a global flag in `.bazelrc`
recompiles it, as it does MuJoCo.

**Upgrade to the next release.** Three values in `MODULE.bazel` (address, hash,
`strip_prefix`); nothing in the build file names the version. Then read the new
release's `src/ompl/CMakeLists.txt` and `CMakeModules/OMPLDependencies.cmake` for a new
source directory, a new dependency or a new placeholder in `config.h.in`. A new
placeholder stops the build with a message; a new directory does not, it shows as a
link error where it is first used. Not verified against a real upgrade: 2.0.2 is the
newest release [S2].

**Pinned, self-hosted, offline.** The archive is pinned by the SHA-256 that OMPL's
maintainers publish beside it [S2]; the Boost modules are pinned by the hashes of their
registry files in `MODULE.bazel.lock` and by the integrity values in the registry [S8].
To self-host OMPL, put another address first in `urls`; `http_archive` tries them in
order. That is the same idea as the `base_url` of this repository's own rules, with a
standard rule instead of a new one. Boost can be served from elsewhere with
`--module_mirrors` or a downloader configuration, or the whole set captured with
`bazel vendor` [S9][S10]. A build with the network off was not tried: not
verified.

**Reuse in a second project.** Copy the `http_archive` block, the 17 `bazel_dep` lines
and the build file. There is no single line that brings it; that is what O4 adds.

**Optional parts, on paper.** FLANN: the registry has `flann` 1.9.2 with a target
`@flann//:flann_cpp` [S11]; it would be one more `bazel_dep`, one more dependency of the
library and `OMPL_HAVE_FLANN` set to 1 in the `config.h` rule. At 2.0.2 only one of
OMPL's tests includes the FLANN header, so the library gains nothing by default [S1].
VAMP: its sources are in the release archive under `external/vamp` (the reason this
archive was pinned and not GitHub's archive of the tag, where that directory is empty),
but it needs a build file of its own and SIMD flags; the ROS registry's overlay builds
it and is the prior art [S16]. Python bindings: nanobind is in the registry as
`nanobind_bazel` [S11]; the bindings are 114 files in the archive and would be a
second, larger build file. Neither was attempted.

**Licence.** OMPL is BSD 3-clause: a binary that contains it has to reproduce the
copyright notice, the conditions and the disclaimer in its documentation [S1]. The
build file exports `@ompl//:LICENSE` for that. The files under `demos/multicopter/` are
GPL [S1, found by `grep`]; they are in the archive but outside the directories the
build file compiles. Boost's licence asks for nothing when only object code is
distributed [S25]. Eigen (MPL 2.0 [S26]) is already a dependency of the repository.

**Known ways to break.**
- Upstream changes its build in a way the build file does not follow: a new directory,
  a source file that needs an optional dependency, a new configure-time switch.
- OMPL is compiled as C++20 at `-O2` here; upstream builds and tests C++17 at `-O3`
  [S1]. It compiled without a patch at 2.0.2 with GCC 14.2 and Boost 1.90. A later
  release or another compiler may not; nothing but a build finds out.
- Boost is resolved by Bazel's version selection. If another dependency later asks for a
  newer `boost.*`, OMPL is compiled against that one.
- The archive's address is a GitHub release asset. If upstream deletes or replaces it
  the hash check fails and the build stops (it cannot silently change).
- OMPL's headers reach the compiler as `-I`, not `-isystem`, so a warning raised inside
  one of them while this repository's code is compiled is not hidden. None is today.
- `bazel/ompl.BUILD.bazel` is one more thing a reviewer of an upgrade must understand.

### O2 foreign

Built on `spike/ompl-foreign-cc`, variant a: Boost and Eigen from Bazel targets, CMake
and Ninja from the rule set. It built at the first attempt, so its time box never
started, and variant b (system Boost) was not built.

**How it works.** `rules_foreign_cc` runs OMPL's own CMake build as one Bazel action
and hands the installed library and headers on as a C++ target. `MODULE.bazel` gains
56 lines: the same `http_archive` as O1 with a patch added, the rule set, and the Boost
modules of O1 plus `boost.program_options`:

```
http_archive(
    name = "ompl",
    build_file = "//bazel:ompl.BUILD.bazel",
    patch_args = ["-p1"],
    patches = ["//bazel:ompl_cxx20.patch"],
    sha256 = "b9155e243e555b04b01f8e4ff7e21a48582edbab9c300a611289655778bcd29d",
    strip_prefix = "ompl-2.0.2",
    urls = ["https://github.com/ompl/ompl/releases/download/2.0.2/ompl-2.0.2.tar.gz"],
)

bazel_dep(name = "rules_foreign_cc", version = "0.17.0")
# ... the 17 boost.* lines of O1 and boost.program_options
```

Build files: `bazel/ompl.BUILD.bazel` (97 lines), a `filegroup` of the whole archive and
one `cmake()` rule [S12] with Ninja as generator and 19 cache entries; and three pieces
that
exist only to make OMPL's CMake accept what Bazel gives it:

| Piece | Why it is needed |
|---|---|
| `bazel/ompl_cmake/Eigen3Config.cmake` (25 lines) | OMPL asks for Eigen with `find_package(Eigen3 REQUIRED NO_MODULE)`, which accepts only a CMake package file. The registry's `@eigen` is a Bazel library and has none. The file defines `Eigen3::Eigen` from the headers the rule copies in. |
| five `Boost_*` cache entries | OMPL finds Boost with CMake's own `FindBoost`, which looks for `libboost_serialization.*`. The registry's Boost modules produce `libboost.serialization.a`. The entries name the include directory and the two libraries outright and forbid looking anywhere else. `boost.program_options` is there only because OMPL's `find_package` demands it; nothing built here links it. |
| `bazel/ompl_cxx20.patch` (one line changed) | OMPL sets `CMAKE_CXX_STANDARD 17` as a plain variable, which a cache entry cannot override, and CMake appends the `-std=c++17` it makes of it after the flags Bazel hands over. Without the patch every one of the 249 files was compiled as C++17 in spite of `.bazelrc` (seen in the first build's log). The patch changes 17 to 20. |

The other cache entries switch off the shared library, VAMP, the Python bindings, the
tests and the demos, and stop CMake from looking for FLANN, Spot, Triangle, yaml-cpp,
Doxygen and Python on the machine.

**On the host.** Nothing was installed. The rule set downloads a prebuilt CMake 3.31.12
and builds Ninja from the registry's module. The action's script uses the compiler and
`ar` of Bazel's toolchain and `bash`, `find`, `sed`, `readlink`, `ln`, `cp`, `touch`,
`dirname` from the machine.

**Existing dependency versions.** Three modules `main` already used move:
`bazel_lib` 3.1.0 to 3.2.0, `rules_cc_autoconf` 0.5.5 to 0.32.0, `rules_shell` 0.6.1 to
0.8.0, because `rules_foreign_cc` asks for the newer ones. Six modules besides Boost are
added: `rules_foreign_cc`, `ninja`, `make`, `m4`, `meson`, `pkgconf`. The effect on the
existing build was measured: the checker programs of `rules_cc_autoconf` are rebuilt
and the configure checks it runs for `xkbcommon` (which GLFW uses) run again; no object
of the repository's code, MuJoCo, GLFW or `xkbcommon` is compiled again (see
Measurements).

**First build and cache.** 183.3 s on six cores from nothing, 4,870 actions, 1,750 of
them run. All of OMPL is one of those actions, and it is the critical path (126 s). A
cache stores that action whole: after `bazel clean` the build took 4.3 s. Anything that
changes the action's inputs (a cache entry, the patch, a Boost module, a global compile
flag) compiles all 249 files again; the third build of the spike did exactly that after
one unused cache entry was removed from the rule. Inside the action Ninja chooses its own number of
jobs, which Bazel's scheduler does not see.

**Upgrade to the next release.** The same three values as O1. OMPL's own CMake follows
its own source lists, so a new directory or source file needs nothing here; that is
this option's advantage. What can go wrong instead: the patch no longer applies (the
build stops and says so); an option is renamed upstream (CMake only notes an unused cache
entry in its log, as "Manually-specified variables were not used by the project",
which the log of this build already shows for one of the rule set's own entries; the part the
option switched off would be built again); a new required dependency needs another shim. Not verified against a real
upgrade.

**Pinned, self-hosted, offline.** As O1 for OMPL and Boost. In addition the CMake
binary is downloaded from Kitware's releases by the rule set, checked against integrity
values kept in the rule set [S13]. The action itself has no network unless a target asks for it [S13].

**Reuse in a second project.** Copy four files and the `MODULE.bazel` lines. The build
file names this repository in a label (`@solar_panel_pick_place//bazel:...`) to reach
the Eigen shim, which has to be changed in the copy.

**Optional parts, on paper.** In principle a cache entry each. In practice each needs
CMake to find something Bazel built: FLANN is looked for in config mode and would need
a second shim; the Python bindings need Python's headers from a Bazel toolchain. VAMP's
sources are in the archive, but it adds `-mcpu=native -mtune=native` on arm64 and
`-march=native -mavx2` on x86-64 unless `VAMP_PORTABLE_BUILD` is set [S1], which would
make a cached library wrong for another machine. None was attempted.

**Licence.** As O1. `rules_foreign_cc` is a build tool and is not distributed with the
result.

**Known ways to break.**
- OMPL is compiled with CMake's choice of flags, not the repository's: `-O3` here
  against `-O2` for everything else (see Measurements).
- The Boost library file names are written into the build file. A registry change to
  how Boost's archives are named breaks it.
- CMake 3.31's `FindBoost` does not know Boost 1.90 and says so three times in the log
  ("New Boost version may have incorrect or missing dependencies and imported
  targets"); it worked. OMPL's own summary then lists Boost under "REQUIRED packages
  have not been found", wrongly: a second, optional search for Boost's test library in
  `tests/CMakeLists.txt` resets the result. Both are harmless and both will mislead
  whoever reads the log after a real failure.
- One opaque action: the compile lines and the compiler's messages go to a log file in
  the output tree (`ompl_foreign_cc/CMake.log`), not to the terminal. No build of O2
  failed, so how a compile error in OMPL is reported was not seen.
- Open issues of the rule set with Bazel 8 and later (wrong include paths, 1430) and
  with path rewriting (1138) [S14]. Neither was met here.
- The rule set calls itself "not an officially supported Google product" [S13]; it
  brings six modules and moves three.

### O3 ROS registry

Built on `spike/ompl-ros-registry`. The first build failed in analysis; the next, with
four more lines in `MODULE.bazel`, worked. `//:ompl_smoke` builds with the repository's
own compiler.

**How it works.** The ROS Central Registry is named before the central one in
`.bazelrc`, and the root module asks it for `ompl`:

```
common --registry=https://bazel.ros.org
common --registry=https://bcr.bazel.build
```

```
bazel_dep(name = "ompl", version = "2.0.2-1.rcr.1")

bazel_dep(name = "rules_python", version = "2.4.5")
python = use_extension("@rules_python//python/extensions:python.bzl", "python")
python.defaults(python_version = "3.12")
python.toolchain(python_version = "3.15")
```

The last four lines were not planned. That registry's OMPL depends on `rosdistro`,
which uses the pip extension of `rules_python`; the extension is evaluated once for all
modules, and the `mujoco` module's part of it fails (the failure this report lists
under "Found along the way", harmless on `main` because nothing there needs pip). With
OMPL in the graph it stops the build:

```
$ bazel build --config=opt --config=agent //:ompl_smoke
Error in fail: Unable to find interpreter for pip hub 'mujoco_pip' for python_version=3.15: Make sure a corresponding `python.toolchain(python_version="3.15")` call exists.
ERROR: Analysis of target '//:ompl_smoke' failed; build aborted: error evaluating module extension @@rules_python+//python/extensions:pip.bzl%pip
```

Registering Python 3.15 in the root lets the extension evaluate. No build file is
written here: the registry's overlay (3,857 lines [S16]) is the build file. `//:ompl` is
an alias for `@ompl//:ompl_library`; the registry's `@ompl//:ompl` is a shared library,
which a `cc_binary` cannot depend on directly.

**On the host.** Nothing was installed. The compiler was the machine's GCC.

**Existing dependency versions.** This is where the option costs. The module graph goes
from 62 entries to 547. Sixteen modules `main` already used change version:

| Module | on `main` | on the branch |
|---|---|---|
| `bazel_lib` | 3.1.0 | 3.7.2 |
| `jsoncpp` | 1.9.6 | 1.9.6.bcr.2 |
| `libexpat` | 2.7.1.bcr.1 | 2.7.1.bcr.1.rcr.0 |
| `libffi` | 3.4.7.bcr.4 | 3.4.7.bcr.5 |
| `libx11` | 1.8.12.bcr.4 | 1.8.12.bcr.5 |
| `libxcb` | 1.17.0.bcr.2 | 1.17.0.bcr.3.rcr.0 |
| `nlohmann_json` | 3.12.0.bcr.1 | 3.12.0.bcr.2 |
| `protobuf` | 33.4 | 36.2 |
| `rules_android` | 0.1.1 | 0.7.1 |
| `rules_cc_autoconf` | 0.5.5 | 0.27.0.rcr.0 |
| `rules_java` | 9.1.0 | 9.3.0 |
| `rules_jvm_external` | 6.7 | 6.9 |
| `rules_kotlin` | 1.9.6 | 2.3.20 |
| `rules_pkg` | 1.0.1 | 1.3.0 |
| `rules_shell` | 0.6.1 | 0.8.0 |
| `zlib` | 1.3.1.bcr.5 | 1.3.2.bcr.2.rcr.0 |

Four of them (`libexpat`, `libxcb`, `rules_cc_autoconf`, `zlib`) now come from the ROS
registry in versions it has patched (`.rcr.`); 195 of the 547 modules have such a
version.
The five modules the root names itself (`rules_cc`, `mujoco`, `eigen`, `glfw`,
`googletest`) keep their versions. Among the 485 added: `rosdistro`, `llvm`
0.8.25.rcr.0, 36 Qt modules, 110 Boost modules, `protobuf`, `rules_rust`,
`rules_foreign_cc` 0.16.0, `pybind11_bazel`, `yaml-cpp`. Most are never fetched for
this target; they are in the graph and in the lock file. What this did to the existing
build is under Measurements.

**Toolchains.** The registry is written for a hermetic LLVM toolchain [S15], but it is
the root module that registers it in the registry's examples [S15], and this root does
not.
Nothing took the compiler over: toolchain resolution selects the repository's
`local_config_cc` and `/usr/bin/gcc` compiles the existing targets, OMPL and the smoke
program alike. `rosdistro` registers Rust toolchains and nothing for C++ [S17, and by
`grep` in the fetched module]. No LLVM and no Qt was downloaded; Bazel's own Python
3.12 was, because the library depends on it.

**First build and cache.** See Measurements. One action per source file, as O1.

**Upgrade to the next release.** Wait for the ROS registry to publish it, then change
the version. Its versions follow ROS's release repository (`2.0.2-1.rcr.1` is built
from `ros2-gbp/ompl-release` [S16], which takes upstream's release archive [S4]), and each is tied to one `rosdistro` version, so an
upgrade of OMPL can move the other 546 modules too.

**Pinned, self-hosted, offline.** Pinned by the lock file, which hashes every registry
file it read (1,407 entries for `bazel.ros.org`, 1,009 of them recording that a module
is not there). Self-hosting means mirroring that registry; its README names a second
address for when the first refuses requests [S15].

**Reuse in a second project.** The same two registry lines and the `bazel_dep`; the
Python lines only where the MuJoCo module is also present.

**Optional parts.** The registry's overlay has separate targets for a VAMP SIMD library
and for the Python extension [S16]; they were not built for `ompl_library` (249 OMPL
objects, Boost, yaml-cpp, and nothing of VAMP or nanobind). FLANN and Spot are off in
it.

**Licence.** OMPL's as before. The registry's overlay carries an Apache 2.0 header
[S16].

**Known ways to break.**
- The registry that is named first wins for every module it has, including patched
  versions of modules this repository already used. That is a standing change to the
  whole build for the sake of one yardstick.
- `ompl_library` depends on Python: the smoke binary links `libpython3.12.so.1.0` from
  Bazel's hermetic Python at run time. A C++ planner library that drags an interpreter
  into the binary is the wrong shape for this use.
- The lock file grows from 463 lines to 9,434, of which 3,617 are addresses of Python
  packages.
- The registry states support for Ubuntu 26.04 and macOS with its LLVM toolchain
  [S15]. Debian 13 with GCC, as here, and CI's Ubuntu 24.04 are outside what it tests.
- It worked on the second attempt only because of an unrelated flaw in another module;
  a graph of 547 modules has room for more of those.

### O4 own module

Built on `spike/ompl-own-module`. It worked at the first attempt; its time box never
started.

**How it works.** O1's build file becomes the overlay of a module `ompl` in a registry
that is a directory of this repository, and the root module asks for it like any other
module:

```
registry/modules/ompl/metadata.json                  11 lines: the versions that exist
registry/modules/ompl/2.0.2/MODULE.bazel             28 lines: the module, and its bazel_deps
registry/modules/ompl/2.0.2/source.json               8 lines: address, integrity, strip_prefix, overlay
registry/modules/ompl/2.0.2/overlay/BUILD.bazel     132 lines: O1's build file
```

`MODULE.bazel` gains one line, `bazel_dep(name = "ompl", version = "2.0.2")`. `.bazelrc`
gains two (and a comment):

```
common --registry=file://%workspace%/registry
common --registry=https://bcr.bazel.build
```

Naming any registry switches the default one off, so the Bazel Central Registry has to
be named again after it [S7]. `%workspace%` is expanded in `--registry` by Bazel 9.3.0;
the documentation does not say so, the build shows it. A new file `.bazelignore` with
the one entry `registry` is needed too: without it `//...` takes the overlay for a
package of this repository and stops with `glob pattern 'src/ompl/base/**/*.cpp' didn't
match anything`.

**What changed from O1.**
- `bazel/ompl.BUILD.bazel` moved to the overlay. Its rules are unchanged; only its
  opening comment differs.
- The `http_archive` block and the 17 Boost lines left the root `MODULE.bazel`. The
  archive's address and hash are now in `source.json`, the hash rewritten as an
  integrity value (`sha256-` and base 64); the Boost lines are in the module's own
  `MODULE.bazel`, with `rules_cc` and `eigen`, which the root no longer supplies
  implicitly.
- New: `metadata.json`, the integrity value of the overlay file in `source.json`, the
  two registry lines, `.bazelignore`.
- No overlay `MODULE.bazel` is needed: Bazel reads the module's `MODULE.bazel` from the
  registry.
- `MODULE.bazel.lock` is byte for byte the lock file of O1. It records nothing for the
  registry directory: Bazel does not hash the files of a `file://` registry [S33, and
  seen here]. The module's files are pinned by being in the same commit as what uses
  them.

The rearrangement took five minutes.

**On the host.** As O1.

**Existing dependency versions.** Unchanged. The module's own `bazel_dep` lines name
the versions `main` already resolves, so nothing moves; a module that named newer ones
would move them.

**First build and cache.** The same actions as O1, action for action (see
Measurements).

**Upgrade to the next release.** A new directory `registry/modules/ompl/<version>/`
with its three files, the version added to `metadata.json`, the version in the root's
`bazel_dep`. Two integrity values have to be computed by hand (`openssl dgst -sha256
-binary FILE | base64`): the archive's and the overlay's. The old version stays, which
is the point of a registry and is more files than O1's three changed values.

**Pinned, self-hosted, offline.** The archive by the integrity value in `source.json`;
`mirror_urls` there is the place for a self-hosted copy [S7]. The registry's own files
by the commit. Boost as O1.

**Reuse in a second project.** This is what the option is for. The second project adds
`--registry=https://raw.githubusercontent.com/<owner>/<repository>/<commit>/registry/`
before the central registry's line [S7] and one `bazel_dep`. Over `https` the registry's
files are hashed in that project's lock file [S8]. Not tried: there is no second
project.

**Optional parts.** As O1.

**Licence.** As O1.

**Known ways to break.**
- Editing the registry. A running Bazel does not notice an edit to the registry's
  files at all (`--watch_registry` exists for that [S7]; not tried). After a restart
  an edit of the overlay is used only if its integrity value in `source.json` was
  changed with it; otherwise Bazel says nothing and keeps building from the copy it
  fetched before the edit (see Measurements).
- The two `--registry` lines apply to every Bazel command in the repository and to
  every module, not only to OMPL: each module is first looked for in `registry/`.
- `.bazelignore` hides `registry/` from Bazel altogether; a real package could never
  live under it.
- Leaving out the central registry's line breaks every other dependency at once.

### O5 upstream

Not built. Assessed from the registry's own rules.

**How it works.** The module of O4 is sent to the Bazel Central Registry as a pull
request; afterwards any project, this one included, writes
`bazel_dep(name = "ompl", version = "2.0.2")` and nothing else. The build file leaves
this repository and lives in the registry as an overlay.

**What it needs beyond O4** [S5][S6], with the practice of current modules from [S11]:
- `source.json` of type `archive` only, with an integrity value for the archive and for
  every overlay file. A GitHub source address is checked for being stable; a release
  asset such as OMPL's is the kind other modules use.
- A `presubmit.yml` for every version, naming the platforms and Bazel versions to build
  on. The rules do not fix the list; current modules test Debian, Ubuntu, macOS and
  often Windows, with Bazel 7, 8 and 9. O1's build file has been built on one platform
  with one compiler.
- `metadata.json` with a maintainer, who is asked to review later versions. The first
  contributor of a module is "likely" to be appointed.
- For a module with overlays: `bazel_compatibility` of at least 7.2.1, a target named
  like the module (`@ompl//:ompl`, as O1 has), few public targets, and, for a large
  build file, a note on how it was made.
- Dependencies only on modules already in the registry. Boost and Eigen are.
- A signed contributor agreement, and review by a registry maintainer because the
  module is new; "most PRs may be addressed in 3 business days".
- Versions are add-only. A fix to the build file alone is published as `2.0.2.bcr.1`.

Whether the registry expects upstream's consent for a module upstream does not maintain
is not stated in its rules: not verified. OMPL has no issue or pull request about Bazel
[S1, searched 2026-11-23], and its maintainers were not contacted for this report.

**On the host.** As O1.

**Existing dependency versions.** As O4 once resolved; the module's own `MODULE.bazel`
would name `rules_cc`, `eigen` and the Boost modules, and version selection takes the
higher of its and this repository's. Not verified.

**First build and cache.** As O1.

**Upgrade.** A new pull request to the registry for each OMPL release, then one line
here. The cost moves from this repository to whoever maintains the module.

**Pinned, self-hosted, offline.** Pinned by the registry and the lock file. Self-hosting
is by registry mirror [S7].

**Reuse.** The best of all options: one line, for anybody.

**Optional parts.** A public module is expected to serve more users than this
repository; requests for FLANN, VAMP or the bindings would come to the maintainer.

**Licence.** As O1; the overlay itself needs a licence the registry accepts.

**Known ways to break.** The presubmit on platforms nobody here has tried (macOS,
Windows, clang); the standing duty of maintenance; a second Bazel packaging of OMPL
beside the ROS registry's, with a different target layout.

### O6 in-tree

Not built: it is O1 with the files in another place.

**How it works.** OMPL's source is committed under `third_party/ompl/` (a plain copy, a
`git subtree`, or a submodule) with O1's build file beside it as
`third_party/ompl/BUILD.bazel`, its `glob`s unchanged. `MODULE.bazel` gains the 17 Boost
lines and no fetch rule. `src/ompl` at 2.0.2 is 5.7 MB in 581 files (6.5 MB on disk). A submodule keeps
the repository small but adds a step Bazel does not perform (`git submodule update`,
also in CI's checkout), and OMPL's own submodules come with `--recursive`.

**On the host.** As O1; with a submodule, also the checkout step.

**Existing dependency versions.** As O1 (the same Boost modules).

**First build and cache.** The same actions as O1 without the 150 MB download. Not
measured.

**Upgrade.** Replace the tree. The diff of an upgrade is reviewable line by line, and
is thousands of lines.

**Pinned, self-hosted, offline.** Pinned by the commit; nothing of OMPL to fetch. Boost
still comes from the registry.

**Reuse.** Copy the directory, or share the submodule.

**Optional parts.** As O1.

**Licence.** The repository would redistribute OMPL's source. It is public, so the
first condition of the licence applies to the repository itself: keep `LICENSE` and the
file headers. A copy limited to `src/ompl` leaves the GPL demo files out.

**Known ways to break.** Local edits to the copy, which are easy to make and then have
to be carried through every upgrade; a second build file, against the repository's rule
of one; history that grows by megabytes per upgrade.

### O7 prebuilt

Not built.

**How it works.** OMPL is installed by a package manager and a repository rule
(`new_local_repository`, or a small rule of the repository's own) wraps the installed
headers and library in a `cc_library` or `cc_import`. `MODULE.bazel` would gain one
`use_repo_rule` line and a call of about five lines naming the install prefix, and the
repository a build file of about ten lines with the header `glob` and the library.
Not written: these sizes are an estimate.

**On the host.** The package and everything it depends on. What the packages are
decides the option:

| Source | OMPL version | Notes |
|---|---|---|
| Debian 13 `libompl-dev` (the development machines) | 1.6.0 | depends on system Boost and `libeigen3-dev` [S18] |
| Ubuntu 24.04 `libompl-dev` (CI's `ubuntu-latest`) | 1.5.2 | [S19][S20] |
| ROS `lyrical`, `rolling` | 2.0.2 | built for Ubuntu 26.04; `humble` to `kilted` have 1.7.0 [S24] |
| conda-forge | 2.0.2 | `linux-64` and `linux-aarch64`; brings its own Boost, Eigen, FLANN, Python [S21] |
| vcpkg, Homebrew | 2.0.2 | [S23] |
| Nix | not verified | |

So `apt` gives neither the version asked for nor the same version on a development
machine and on CI. conda-forge and Homebrew have 2.0.2 as binaries for Linux on both
architectures [S21][S23]; vcpkg builds it from source.

**Existing dependency versions.** Bazel's modules do not change. But the installed
library was compiled against the packager's Eigen and Boost, and its headers include
them: a program that includes both OMPL's headers and this repository's Eigen 5.0.1
compiles two Eigens into one binary unless the include paths are kept apart with care.
Not tried.

**First build and cache.** Nothing to compile. Whether Bazel notices an upgrade of the
package depends on the wrapper: files reached through a `glob` of a local repository
are inputs like any other, but a library named only by `-lompl` in `linkopts` is not
an input, and results built against the old one would be reused. Not tried.

**Upgrade.** Whenever the packager does, not when this repository chooses.

**Pinned, self-hosted, offline.** Not pinned by anything in the repository. A conda
lock file or a container image can pin it outside Bazel; self-hosting is a mirror of
the package channel. Once the package is installed the Bazel build needs no network
for it.

**Reuse.** The same install step and wrapper in the second project.

**Optional parts.** Whatever the packager enabled; conda-forge's build depends on FLANN
and Python [S21].

**Licence.** The packager's concern for the library; unchanged for a distributed
binary.

**Known ways to break.** Version drift between machines; CI needs an install step,
which is a change to the workflow; a compiler or standard-library difference between
the package and the repository's build; the two-Eigen problem above.

### O8 outside Bazel

Not built.

**How it works.** The Bazel build knows nothing of OMPL. A separate program does the
comparison: a CMake project that finds an installed OMPL, or a Python script with
`pip install ompl`.

**On the host.** For CMake: CMake, OMPL and its dependencies, by any of O7's routes.
For Python: the wheel. PyPI's newest is 2.0.1, not 2.0.2, with wheels for Linux x86-64
and aarch64 and macOS, Python 3.10 to 3.13 as far as was read; the 2.x bindings are
new and upstream's notes call complete bindings "under review" [S22][S3].

**Existing dependency versions, first build, cache.** Untouched; nothing to build in
Bazel.

**Upgrade, pinning, offline.** A version in a `requirements.txt` pins a wheel; a CMake
`find_package` only states a minimum and takes what is installed. Self-hosting is a
package mirror, outside Bazel. The Bazel build is offline as far as OMPL goes, because
it does not know of it.

**Reuse.** Nothing to reuse.

**Optional parts.** Whatever the wheel or the package has.

**Licence.** Nothing is distributed with the repository's binaries.

**Known ways to break.** The comparison the owner wants calls the cell's collision
check from inside OMPL's planning loop, thousands of times a plan (see `## What the
comparison needs`). That check is C++ on MuJoCo, built by Bazel. An outside program
must either build the repository's sources a second time under CMake, or reach them
from Python, for which no binding exists, and then time a planner through a Python
callback per query. Either way the two planners are no longer compiled and measured
alike. For the toy world alone it would do; for the cell it defeats the purpose.

## Measurements

All figures are from one machine, described under `## How this was measured`, on
2026-11-23. Each timed build is given with the machine's state just before it. The
toy-world lines are a proof that OMPL plans here and a first look at method; they are
not the comparison of the two planners.

### The spikes

| Option | Branch | Commit | Builds | `//:ompl_smoke` exits 0 |
|---|---|---|---|---|
| O1 native | `spike/ompl-native` | `8fa42501546a8b4067fc186dba75a1e83f6b2523` | yes | yes |
| O2 foreign, variant a (hermetic) | `spike/ompl-foreign-cc` | `a28edc96a5932a6b1f903c10c74ac62fc6cd070e` | yes | yes |
| O3 ROS registry | `spike/ompl-ros-registry` | `38545fbf1b6b8e17bdf823d720e4aea970eea4d5` | yes, with the machine's GCC | yes |
| O4 own module | `spike/ompl-own-module` | `5c01cff3dcb7e63430200992eaf4ef23d4c9ad5a` | yes | yes |

All are cut from `main` at `014ba3a23b0c5b3d45f9b0fcc6d6d9f9644f91d4`.

### O1 native

**Build cost**, from a fresh clone at `8fa4250`, `--config=opt`, default `--jobs`
(6):

| | Wall time | Actions | Machine before |
|---|---|---|---|
| `bazel build //:ompl_smoke`, empty output base, empty repository and disk caches | 147.5 s | 340 (335 in the sandbox, 5 internal); critical path 8.9 s | `nproc` 6; load 1.15, 2.59, 2.59; 6,183 MiB free, 10,697 MiB available |
| the same one commit earlier (`9730735`; the two differ by a comment in the build file) | 148.7 s | 340; critical path 8.8 s | load 0.36, 1.44, 1.63; 6,923 MiB free, 10,692 MiB available |
| again after `bazel clean`, caches kept (at `8fa4250`) | 5.1 s | 340 (335 disk-cache hits, 5 internal) | load 6.39, 4.25, 3.22 (the build had just finished); 5,363 MiB free |

The first row's load of 1.15 was the tail of a test run that had ended; nothing else
was running. The second row, taken on a quieter machine, agrees with it to within a
second and a half, so the first was not repeated.

What the cold build left on disk:

| | Bytes |
|---|---|
| output base | 98,075,789 (of which `execroot` 88 MB) |
| repository cache | 913,226,427: downloads 253,470,462, of which OMPL's archive is 149,922,968; unpacked repositories 659,755,965 |
| disk cache | 93,551,194 |

Bazel 9 keeps unpacked external repositories under the repository cache, not in the
output base, which is why the output base is small. `libompl.a` is 34 MB; the
`ompl_smoke` binary is 1.6 MB.

**Glue** (`git diff --stat main...spike/ompl-native`, lock file apart):

| File | Lines added |
|---|---|
| `BUILD.bazel` | 29 |
| `MODULE.bazel` | 41 |
| `bazel/ompl.BUILD.bazel` | 125 |
| `src/ompl_smoke.cc` (shared by all spikes) | 331 |
| total | 526, or 195 without the smoke program |
| `MODULE.bazel.lock` | 245 |

**Patches and flags OMPL needed.** No patch. One definition for OMPL's own files,
`BOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS` (as OMPL's CMake sets it), and `-pthread` at
link time. `config.h` is generated with every optional part off.

**Flags OMPL was compiled with** (`bazel aquery`, the action for
`src/ompl/geometric/planners/rrt/src/RRTConnect.cpp`):

```
/usr/bin/gcc -U_FORTIFY_SOURCE -fstack-protector -Wall -Wunused-but-set-parameter
  -Wno-free-nonheap-object -fno-omit-frame-pointer -g0 -O2 -D_FORTIFY_SOURCE=1 -DNDEBUG
  -ffunction-sections -fdata-sections -std=c++17 [...] -DBOOST_ALL_NO_LIB
  -DBOOST_THREAD_DONT_USE_ATOMIC -DBOOST_FILESYSTEM_NO_CXX20_ATOMIC_REF
  -DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS -std=c++20 -c [...]
```

| | |
|---|---|
| Optimization | `-O2` (OMPL's own CMake Release build uses `-O3`) |
| `NDEBUG` | defined |
| Standard | `-std=c++20`; the toolchain's `-std=c++17` comes first and the later flag wins |
| `-march`, `-mcpu`, SIMD flags | none |
| Linking | static: `libompl.a` on the link line with the archives of `boost.serialization`, `boost.thread`, `boost.chrono`, `boost.container` and `boost.filesystem`; `ldd` of the binary shows only `libstdc++`, `libm`, `libgcc_s` and `libc` |

These are the flags the repository's own optimized code gets. The three other `BOOST_`
definitions come from the Boost modules.

**The smoke program** (`bazel run --config=opt //:ompl_smoke`, exit status 0):

```
planner=own world=arm_like seeds=20 found=20 valid=17 median_queries=14690 median_ms=0.511 median_length=9.099
planner=ompl world=arm_like seeds=20 found=20 valid=17 median_queries=18640 median_ms=1.677 median_length=9.142
```

A second run printed the same lines but for `median_ms`. Over the four runs logged for
O1 (two at this commit, two at the one before) it was 0.38 to 0.51 for the first line
and 1.57 to 1.68 for the second. Three things about these
numbers, each taken up in `## What the comparison needs`:

- `valid=17`: three paths of each planner fail the recheck at 0.005 rad. With `--plans`
  the program also rechecks against obstacles shrunk by 0.010 rad, and all 40 paths
  pass: the failures are corners clipped between two checked points, which both
  planners allow by construction, and nothing worse.
- More than a millisecond of OMPL's `median_ms` is not planning. `solve(5.0)` starts a
  timer thread and joins it at the end. With the limit set to 0.99 s, which OMPL
  handles without a thread, the second line read `median_ms=0.521` and `0.482` in two
  runs, with the same queries and length. (A temporary edit of one constant; the
  branch has 5 s, as the smoke program was specified.)
- The two planners were not set alike: step length, the measure of the edge spacing and
  the shortening all differ. The medians say that both work, not which is better.

**The existing build**.

The module graph, from `bazel mod graph` reduced to `name@version`: 62 entries on
`main`, 144 on the branch. Added: `boost@1.90.0.bcr.1` and 81 `boost.*` modules.
Changed: none. Removed: none.

The repository's tests, `bazel test --config=opt --config=agent //...
--test_output=errors --nocache_test_results`, with a disk cache shared between the
checkouts so that only the difference between them is rebuilt:

| Checkout | State before | Wall time | Test targets | Actions |
|---|---|---|---|---|
| `main` `014ba3a` | fresh clone, empty disk cache | 345.1 s | 28 of 28 pass | 2,637: 2,154 run, 483 internal |
| `main` `014ba3a` | output base emptied, disk cache from the row above | 145.7 s | 28 of 28 pass | 2,637: 2,123 cache hits, 31 run, 483 internal |
| `spike/ompl-native` `8fa4250` | fresh clone, the same disk cache | 157.2 s | 28 of 28 pass | 2,637: 2,123 cache hits, 31 run, 483 internal |

The branch gets the same 2,123 cache hits as `main`: every compile and link action of
the existing targets has the same inputs and command line as on `main`. The 31 actions
that run are the tests themselves. The 11 s between the last two rows is within what
the tests vary by; the branch's run started seconds after another build (load 5.77).
After the branch's test run the output base held no OMPL repository: `//...` does not
fetch it.

**Review points**.
- Warnings. The repository's `--output_filter=^//` sufficed: a build of `//:ompl_smoke`
  prints no warning. With the filter switched off (`--output_filter=`) the same build
  shows 8 warning lines, 5 in OMPL (four `-Wmaybe-uninitialized` in `BinaryHeap.h`, one
  `-Wdeprecated-declarations` in `Constraint.cpp`), 2 in Boost.Serialization and 1 in
  Boost.Graph. OMPL is not compiled with `-w`. The filter goes by the target being
  compiled; OMPL's headers are passed as `-I`, not `-isystem`, so a warning inside one
  of them would show when the repository's own file is compiled. `src/ompl_smoke.cc`
  compiles without any.
- No flag reaches the repository's own code: nothing was added to `.bazelrc`; the
  library has no `copts` and no `defines`; its one definition is `local_defines`. The
  cache-hit count above is the check.
- The licence file is exported as `@ompl//:LICENSE`, and the build file declares
  `licenses(["notice"])`.

**x86-64**. Not verified on x86-64. Nothing in the build file or in the flags
above depends on the architecture: no `select`, no `-march`. The Boost modules and
Eigen are the registry's; the registry's own checks build Boost.Serialization and a
later revision of the same Eigen on x86-64 Linux [S11]. OMPL's core has
branches by operating system in two files (`tools/benchmark/src/MachineSpecs.cpp`,
`util/src/Console.cpp`) and none by processor [S1, by `grep`].

### O2 foreign

Variant a (hermetic). From a fresh clone at `a28edc9`, `--config=opt`, default `--jobs`.

**Build cost**:

| | Wall time | Actions | Machine before |
|---|---|---|---|
| `bazel build //:ompl_smoke`, empty output base, empty repository and disk caches | 183.3 s | 4,870 (1,750 in the sandbox, 3,120 internal); critical path 126 s, which is the one action that builds OMPL | `nproc` 6; load 0.90, 3.52, 3.54; 8,840 MiB free, 11,564 MiB available |
| again after `bazel clean`, caches kept | 4.3 s | 4,870 (1,750 disk-cache hits, 3,120 internal) | load 7.55, 5.82, 4.46 (the build had just finished); 7,117 MiB free |

The action count is not comparable with O1's 340: here Bazel also builds Ninja, the
compiled part of `boost.program_options`, and the rule set's own machinery, and stages
every header of the dependencies; OMPL's 249 compilations happen inside one action and
are not counted. While the spike was being written that one action took about 141 s
and 131 s in its two builds as C++20 (and 106 s in the first, as C++17), by the notes
kept then.

| | Bytes |
|---|---|
| output base | 165,598,815 |
| repository cache | 1,159,113,762: downloads 316,262,687; unpacked repositories 842,851,075 |
| disk cache | 94,087,323 |

**Glue** (`git diff --stat main...spike/ompl-foreign-cc`, lock file apart):

| File | Lines |
|---|---|
| `BUILD.bazel` | +30 |
| `MODULE.bazel` | +56 |
| `bazel/BUILD.bazel` | +4, -2 |
| `bazel/ompl.BUILD.bazel` | +97 |
| `bazel/ompl_cmake/Eigen3Config.cmake` | +25 |
| `bazel/ompl_cxx20.patch` | +8 |
| `src/ompl_smoke.cc` (shared) | +331 |
| total | 551 added and 2 removed, or 220 added without the smoke program |
| `MODULE.bazel.lock` | +270, -3 |

**Patches and flags OMPL needed.** One patch, to `CMakeModules/OMPLCompilerSettings.cmake`
(`CMAKE_CXX_STANDARD` 17 to 20). One CMake file written for it (`Eigen3Config.cmake`).
Nineteen cache entries: `OMPL_BUILD_SHARED=OFF`, `CMAKE_INSTALL_LIBDIR=lib`,
`OMPL_VERSIONED_INSTALL=OFF`; `OMPL_BUILD_VAMP`, `OMPL_BUILD_PYTHON_BINDINGS`,
`OMPL_BUILD_TESTS`, `OMPL_BUILD_DEMOS` all `OFF`; `CMAKE_DISABLE_FIND_PACKAGE_` for
`flann`, `spot`, `Triangle`, `yaml-cpp`, `Doxygen`, `Python`; `Eigen3_DIR`;
`Boost_NO_BOOST_CMAKE`, `Boost_NO_SYSTEM_PATHS`, `Boost_INCLUDE_DIR`,
`Boost_SERIALIZATION_LIBRARY_RELEASE`, `Boost_PROGRAM_OPTIONS_LIBRARY_RELEASE`.

**Flags OMPL was compiled with** (the line for `RRTConnect.cpp` in the action's
`CMake.log`, include paths and file names left out):

```
/usr/bin/gcc -DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS -DBOOST_TEST_DYN_LINK -D_HAS_AUTO_PTR_ETC=0
  -U_FORTIFY_SOURCE -fstack-protector -Wall -Wunused-but-set-parameter -Wno-free-nonheap-object
  -fno-omit-frame-pointer -g0 -O2 -D_FORTIFY_SOURCE=1 -DNDEBUG -ffunction-sections -fdata-sections
  -std=c++17 -fPIC -DBOOST_ALL_NO_LIB -DBOOST_THREAD_DONT_USE_ATOMIC -DBOOST_FILESYSTEM_NO_CXX20_ATOMIC_REF
  [...] -std=c++20 -O3 -O3 -DNDEBUG -std=c++20 -W -Wall -Wextra -Wno-system-headers -fPIC
```

The first half is what Bazel hands over; from `-O3` on it is what OMPL's CMake adds,
and for options given twice the later one wins.

| | |
|---|---|
| Optimization | `-O3` (CMake's Release flags, after Bazel's `-O2`) |
| `NDEBUG` | defined |
| Standard | `-std=c++20`, by the patch; `-std=c++17` without it |
| `-march`, `-mcpu`, SIMD flags | none |
| Linking | static `libompl.a`, compiled `-fPIC`; `ldd` of the binary shows the same four system libraries as O1 |

**The smoke program** (`bazel run --config=opt //:ompl_smoke`, exit status 0):

```
planner=own world=arm_like seeds=20 found=20 valid=17 median_queries=14690 median_ms=0.478 median_length=9.099
planner=ompl world=arm_like seeds=20 found=20 valid=17 median_queries=18640 median_ms=1.642 median_length=9.142
```

The same queries, lengths and counts as O1: `-O3` against `-O2` changes none of them.
The `median_ms` of the second line is within the range seen for O1 (1.57 to 1.68),
most of which is the timer thread; these runs cannot show what `-O3` buys.

**The existing build**.

The module graph: 62 entries on `main`, 151 on the branch. Changed: `bazel_lib` 3.1.0
to 3.2.0, `rules_cc_autoconf` 0.5.5 to 0.32.0, `rules_shell` 0.6.1 to 0.8.0. Added:
`rules_foreign_cc` 0.17.0, `ninja` 1.13.2, `make` 4.4.1.bcr.3, `m4` 1.4.21.bcr.8,
`meson` 1.12.1, `pkgconf` 3.0.10, and 83 Boost modules. Removed: none.

The repository's tests, same command and disk cache as for O1:

| Checkout | Wall time | Test targets | Actions |
|---|---|---|---|
| `main` `014ba3a` (from O1's table) | 145.7 s | 28 of 28 pass | 2,637: 2,123 cache hits, 31 run, 483 internal |
| `spike/ompl-foreign-cc` `a28edc9`, fresh clone | 165.9 s | 28 of 28 pass | 2,634: 2,083 cache hits, 64 run, 487 internal |

Here the existing build is not untouched: 40 fewer cache hits and 33 more actions run
than on `main`. What they are was found two ways. By action key (`bazel aquery` on both
checkouts): the keys that differ belong to `rules_cc_autoconf`'s own programs and to
the configure checks it runs for `xkbcommon`, plus bookkeeping actions that record
which repositories exist; no compile or link action of the repository's own targets,
of MuJoCo, Eigen, GLFW or googletest has a different key. By what was compiled on the
machine during the branch's test run (object files present in the output base, as
against taken from the cache): 13 objects of `rules_cc_autoconf`'s checker and
resolver, and none of `xkbcommon`, GLFW, MuJoCo or the repository. So the moved
versions change no object the cell is made of. The branch's run took 20 s longer than
`main`'s; it started with a load of 7.04, straight after the cold build, so how much
of that is the extra actions was not separated.

**x86-64**. Not verified on x86-64. The rule set has a prebuilt CMake for
`linux-x86_64` as for `linux-aarch64` [S13]; Ninja is built from source on either. The
glue has no `select` and no architecture in a file name. OMPL's CMake adds no `-march`
with VAMP off.

### O3 ROS registry

From a fresh clone at `38545fb`, `--config=opt`, default `--jobs`. It builds with the
machine's GCC; no other C++ toolchain was registered or downloaded.

**Build cost**:

| | Wall time | Actions | Machine before |
|---|---|---|---|
| `bazel build //:ompl_smoke`, empty output base, empty repository and disk caches | 203.4 s | 986 (369 in the sandbox, 617 internal); critical path 8.8 s | `nproc` 6; load 0.88, 2.87, 2.86; 6,220 MiB free, 9,774 MiB available |
| again after `bazel clean`, caches kept | 5.6 s | 986 (369 disk-cache hits, 617 internal) | load 6.15, 4.06, 3.28 (the build had just finished); 6,024 MiB free |

The critical path is O1's and the download is nearly twice O1's (488 MB against
253 MB); how the 203 s divide between fetching and compiling was not measured.

| | Bytes |
|---|---|
| output base | 109,481,107 |
| repository cache | 1,727,428,522: downloads 487,974,389; unpacked repositories 1,239,454,133 |
| disk cache | 98,810,910 |

**Glue** (`git diff --stat main...spike/ompl-ros-registry`, lock file apart):

| File | Lines added |
|---|---|
| `.bazelrc` | 7 |
| `BUILD.bazel` | 30 |
| `MODULE.bazel` | 16 |
| `src/ompl_smoke.cc` (shared) | 331 |
| total | 384, or 53 without the smoke program |
| `MODULE.bazel.lock` | +9,037, -66 (463 lines to 9,434) |

**Patches and flags OMPL needed.** None written here. Four lines in the root
`MODULE.bazel` that register Python 3.15, without which nothing that uses this registry
analyses in this repository (see O3 under Options).

**Flags OMPL was compiled with** (`bazel aquery`, the action for `RRTConnect.cpp` of
`@ompl//:ompl_library`):

```
/usr/bin/gcc -U_FORTIFY_SOURCE -fstack-protector -Wall -Wunused-but-set-parameter
  -Wno-free-nonheap-object -fno-omit-frame-pointer -g0 -O2 -D_FORTIFY_SOURCE=1 -DNDEBUG
  -ffunction-sections -fdata-sections -std=c++17 [...] -fPIC -DBOOST_ALL_NO_LIB
  -DBOOST_THREAD_DONT_USE_ATOMIC -DROS_PACKAGE_NAME="ompl"
  -DDEFAULT_RMW_IMPLEMENTATION=rmw_fastrtps_cpp -DOMPL_BUILDING_DLL -DOMPL_BUILDING_LIBRARY
  -DNDEBUG -D_HAS_AUTO_PTR_ETC=0 -std=c++20 -Wno-missing-template-arg-list-after-template-kw -c [...]
```

| | |
|---|---|
| Compiler | `/usr/bin/gcc`, the repository's; toolchain resolution selects `local_config_cc` |
| Optimization | `-O2` |
| `NDEBUG` | defined |
| Standard | `-std=c++20` |
| `-march`, `-mcpu`, SIMD flags | none |
| Linking | `libompl_library.a` statically, compiled `-fPIC`; but the binary also needs `libpython3.12.so.1.0` at run time (`ldd`), from Bazel's own Python, because `ompl_library` depends on Python's libraries |

`BOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS`, which OMPL's CMake sets and O1 and O2 pass,
is not among the definitions.

**The smoke program** (`bazel run --config=opt //:ompl_smoke`, exit status 0):

```
planner=own world=arm_like seeds=20 found=20 valid=17 median_queries=14690 median_ms=0.470 median_length=9.099
planner=ompl world=arm_like seeds=20 found=20 valid=17 median_queries=18640 median_ms=1.863 median_length=9.142
```

The same queries, lengths and counts as O1, from a different source archive (ROS's
release repository) and a different build file.

**The existing build**.

The module graph: 62 entries on `main`, 547 on the branch; 16 existing modules changed
version (the table under Options), 485 added, none removed. On this branch
`bazel mod graph` and `bazel mod deps --lockfile_mode=error` exit 0: the Python lines
that the registry made necessary also cure the failure these commands have on `main`.

The repository's tests, same command and disk cache as for O1:

| Checkout | Wall time | Test targets | Actions |
|---|---|---|---|
| `main` `014ba3a` (from O1's table) | 145.7 s | 28 of 28 pass | 2,637: 2,123 cache hits, 31 run, 483 internal |
| `spike/ompl-ros-registry` `38545fb`, fresh clone | 170.4 s | 28 of 28 pass | 2,667: 827 cache hits, 1,322 run, 518 internal |

The existing build is disturbed: 1,296 fewer cache hits than on `main`. By what was
compiled on the machine during that run (object files in the output base): 824 of
`libx11`, 114 of `libxi`, 78 of `libxcb`, 52 of GLFW, 140 of the other X and Wayland
libraries and `libffi`, 13 of `rules_cc_autoconf`'s tools, and 9 of this repository's
own, all of them the viewer programs (`viewer.cc`, `viewer_lib.cc`, `view_cell.cc`). None of MuJoCo. This fits the changed versions of
`libxcb`, `libx11`, `libffi` and `zlib`: everything under the viewer is built again and
is no longer the code `main` builds. The tests pass.

**x86-64**. Not verified on x86-64. The registry's overlay adds `-mavx2` on
x86-64 to its VAMP target only [S16], which `ompl_library` does not use; whether the
rest of what it pulls in builds with GCC on Ubuntu 24.04 is not verified, and is
outside what the registry says it supports [S15].

### O4 own module

From a fresh clone at `5c01cff`, `--config=opt`, default `--jobs`.

**Build cost**. This was measured twice, and the first time shows what a shared
host does to a timing:

| | Wall time | Actions | Machine before |
|---|---|---|---|
| `bazel build //:ompl_smoke`, empty output base and caches, first measurement | 337.7 s | 340 (335 in the sandbox, 5 internal); critical path 26.9 s | `nproc` 6; load 0.08, 0.22, 0.99; 5,124 MiB free, 10,200 MiB available |
| the same, second measurement 23 minutes later | 133.4 s | 340; critical path 7.2 s | load 0.86, 3.05, 3.18; 6,414 MiB free, 9,751 MiB available; calibration 0.44 s alone, 0.69 s six at once |
| after `bazel clean`, caches kept (second measurement) | 2.9 s | 340 (335 disk-cache hits, 5 internal) | load 5.82, 4.09, 3.54 (the build had just finished); 5,412 MiB free; calibration 0.51 s, 0.69 s |

During the first measurement the container was idle by its own load average, and
the build took two and a half times as long as the same build did later (its critical
path 26.9 s against 7.2 s). The smoke program was slower too: `median_ms` 0.54 and
0.67 for the repository's planner and 2.34 and 2.37 for OMPL, against 0.40 and 0.37,
and 1.88 and 1.88, in the second measurement. The cause was probably outside the
container, whose load average does not count the host's other containers; it was not
identified. From then on a short
calibration (SHA-256 of 1 GiB, once and six at a time) was recorded before each timed
build. The second measurement is the one to read: the same 340 actions as O1, in
133.4 s against O1's 147.5 and 148.7 s. The difference between those is the machine's
variation from hour to hour, not a property of the option.

| | Bytes (second measurement) |
|---|---|
| output base | 97,980,186 |
| repository cache | 913,235,603 |
| disk cache | 93,397,187 |

**Glue** (`git diff --stat main...spike/ompl-own-module`, lock file apart):

| File | Lines added |
|---|---|
| `.bazelignore` | 3 |
| `.bazelrc` | 7 |
| `BUILD.bazel` | 30 |
| `MODULE.bazel` | 8 |
| `registry/modules/ompl/2.0.2/MODULE.bazel` | 28 |
| `registry/modules/ompl/2.0.2/overlay/BUILD.bazel` | 132 |
| `registry/modules/ompl/2.0.2/source.json` | 8 |
| `registry/modules/ompl/metadata.json` | 11 |
| `src/ompl_smoke.cc` (shared) | 331 |
| total | 558, or 227 without the smoke program |
| `MODULE.bazel.lock` | 245, identical to O1's lock file |

**Patches and flags OMPL needed.** As O1: none, and the same one definition. The build
file's rules are O1's.

**Flags OMPL was compiled with.** As O1 (`-O2 -DNDEBUG -std=c++20`, no `-march`,
static): the same `aquery` shows the same flags.

**The smoke program** (`bazel run --config=opt //:ompl_smoke`, exit status 0, second
measurement):

```
planner=own world=arm_like seeds=20 found=20 valid=17 median_queries=14690 median_ms=0.404 median_length=9.099
planner=ompl world=arm_like seeds=20 found=20 valid=17 median_queries=18640 median_ms=1.875 median_length=9.142
```

**The existing build**.

The module graph: 62 entries on `main`, 145 on the branch. Added: `ompl@2.0.2` and the
82 Boost modules of O1. Changed: none. Removed: none.

| Checkout | Wall time | Test targets | Actions |
|---|---|---|---|
| `main` `014ba3a` (from O1's table) | 145.7 s | 28 of 28 pass | 2,637: 2,123 cache hits, 31 run, 483 internal |
| `spike/ompl-own-module` `5c01cff`, first measurement (the slow period) | 256.0 s | 28 of 28 pass | 2,637: 2,123 cache hits, 31 run, 483 internal |
| the same, second measurement, output base emptied | 134.5 s | 28 of 28 pass | 2,637: 2,123 cache hits, 31 run, 483 internal |

The same 2,123 cache hits as `main`: the registry lines in `.bazelrc` and
`.bazelignore` change no action of the existing build.

**Editing the registry.** Tried on the branch's working copy, with `bazel build
--nobuild //:ompl_smoke` after each step. While a Bazel server is running it does not
notice an edit to the registry's files, whether or not the integrity value in
`source.json` is changed with the overlay: the build succeeds and uses the repository
it fetched before. After `bazel shutdown`, an edit of the overlay together with its
integrity value is fetched and used; an edit of the overlay alone is still ignored,
without a word. What a machine with an empty cache would do with the mismatch was not
tried: not verified.

**x86-64**. Not verified on x86-64. As O1: nothing in the module depends on the
architecture.

## What the comparison needs

This section is about the comparison a later spec would build: OMPL planning the cell's
carry and return with the cell's own collision check, beside `PlanMove`. Nothing of it
exists yet. The two lines of `//:ompl_smoke` come from a made-up world and are used
below only for what they show about the method.

### The adapter

`PlanMove` (`src/motion_planner.h`) is the generic planner of `src/rrt.h` with five
things decided for this arm. The adapter has to give OMPL the same five, or the
comparison is between two problems.

| Piece | In `PlanMove` | For OMPL |
|---|---|---|
| State space | six joints as intervals, never circles | `RealVectorStateSpace(6)`; not `SO2` for joints 4 and 6 |
| Bounds | each range shrunk by `limit_inset`; joints 4 and 6 narrowed to the span of start and goal plus `wrist_slack`. The box used is reported in `MotionPlanReport::lower`, `upper` | the same box, read from a `PlanMove` report for the same start and goal, or computed by a function factored out of `PlanMove`; OMPL's samples must not get a larger box |
| Metric and weights | weighted Euclidean distance with `CollisionChecker::LeverArms()`, read after `SetHeldModule` (metres per radian) | plan in scaled coordinates `x_i = w_i * q_i`: bounds scaled, the validity checker divides by `w_i` before asking. OMPL's Euclidean distance in `x` is then the planner's metric, and its path lengths are in the same "metres of motion". A subclass of the state space that overrides `distance` would also do; scaling is fewer lines |
| Validity | `CollisionChecker::IsFree(q)` with the margins raised to the planning margin (45 mm to the world, 25 mm among the arm and module) and put back afterwards; scene by `SetScene`, the module by `SetHeldModule(k, module_in_tip)` | a state validity checker that calls the same `IsFree` on the same checker, set up by the same calls and with `SetMargins(planning_margin, planning_self_margin)` from the report. The checker holds one `mjData` and `IsFree` is not `const`: one planning thread, or one checker per thread |
| Edges | every edge checked at `edge_resolution` (30 mm) measured as the weighted sum of joint changes, which is what the margin budget's first term rests on | a subclass of `ompl::base::MotionValidator` (two `checkMotion` functions to write) that calls `cell::EdgeIsFree` or `CollisionChecker::SegmentIsFree` with the same resolution and weights. OMPL's default validator spaces its checks by Euclidean distance; with it, the same guarantee needs a spacing of `edge_resolution / sqrt(6)` in `x`, which is sound but checks up to 2.4 times as many points |
| Margins after the fact | clearance measured along the kept path at the checker's own margins (`MotionPlanReport::clearance`) | the same measurement, `DenseSamples` and `CollisionChecker::Check`, on OMPL's path |
| Seeds | `base_seed` gives 4 seeds; the same base seed gives the same path | one seed for the process (`ompl::RNG::setSeed`), set before anything else of OMPL; every later generator draws its seed from it. Plans repeat as a set, not one by one. For a plan that can be repeated alone, run it in its own process |
| Time limits | `time_budget` 10 s for all seeds and rounds, `max_iterations` per seed | `solve()` with a termination condition. `solve(seconds)` with one second or more starts a timer thread that costs about a millisecond a plan (measured below); pass `ompl::base::timedPlannerTerminationCondition(seconds)` yourself, or better a condition that stops after a number of collision queries, which does not depend on the machine |
| What is kept | best of 4 seeds after shortening, retries with twice the iterations, optionally both ways round for joint 1 | decide which of these OMPL gets too (below) |

About 150 to 250 lines of C++ in one file, depending on OMPL and on `:collision`,
`:motion_planner` and `:rrt`. It links MuJoCo and OMPL into one binary, which every
option that builds OMPL inside Bazel allows and O8 does not.

### What makes a naive comparison unfair

Each row is a difference between the two planners as they would be run by default,
with how to remove it or report it. The second column says what the toy run showed of
it, if anything.

| Difference | Seen in the toy run | Remove or report |
|---|---|---|
| Edge-check spacing is measured differently: sum of joint changes here, Euclidean in OMPL. At the same number OMPL checks fewer points on the same edge (by up to sqrt(6)) | both set to 0.02 rad; not separable from the other rows in two lines of medians | the custom motion validator above, so both check an edge at the same points |
| Step length: 1 rad here (3 m in the cell); OMPL's RRT-Connect takes a fifth of the space's extent, which in the toy world's box is 3.9 rad | part of why the query counts differ (14,690 against 18,640 at the median) | set OMPL's `range` to the planner's step, and report OMPL's default as a second row |
| Shortening: 200 random shortcuts here; OMPL's `simplifySolution` with no limit runs shortcuts, a B-spline pass and vertex removal until nothing helps | median length 9.099 against 9.142 rad, after different amounts of work | report raw and shortened lengths separately; run OMPL's raw path through `ShortenPath` as well, so one row differs only in the search |
| Timer thread: OMPL's `solve(5.0)` starts a thread that sleeps in 1 ms slices and is joined at the end | OMPL's median was 1.6 to 1.7 ms with the 5 s limit and 0.48 to 0.52 ms with a 0.99 s limit, same queries and paths; the planner here took 0.38 to 0.51 ms | the termination condition without a thread; and report collision queries first, wall time second |
| Seeds: one per plan here, one per process in OMPL | OMPL's 20 plans repeat as a set (same medians in every run) | compare distributions over the same number of plans (median, 90th percentile, worst), not plan against plan; give OMPL's set seed in the table |
| What "valid" means: both planners check edges at points, so both can clip a corner between two points | 17 of 20 paths passed the 0.005 rad recheck for each planner; all 40 passed against obstacles shrunk by 0.010 rad | in the cell the margin budget covers this; measure clearance of every kept path the same way and report the smallest |
| The box: `PlanMove` narrows joints 4 and 6 | not in the toy world | the same box for both |
| First try: the planner here checks the straight line first (`try_direct_first`) | not visible: the toy world's straight line is blocked | leave on and count its queries, or turn off for both |
| Best of four and retries: `PlanMove` keeps the shortest of 4 seeds and retries with more iterations | the toy run used single plans | two rows: single plan against single plan (`seeds = 1`, `rounds = 1`), and best-of-4 against OMPL's best of 4 |
| Nearest neighbour: a linear scan here, a GNAT tree in OMPL | not separable | this is a real difference between the implementations and is left in; report planner time net of collision time (`time - queries * time per query`) so it shows |
| Compile flags | O1, O3 and O4 compile both planners with the same compiler at `-O2 -DNDEBUG`; O2 compiles OMPL at `-O3`; O7 would use the packager's flags (see `## Measurements`) | prefer an option that compiles both alike; otherwise state the flags beside the timings |
| The adapter's own cost: a copy into an `Eigen::VectorXd` per query | in the toy world a query costs nanoseconds, so this is part of OMPL's time | in the cell a query is about 9 microseconds and the copy is noise; say so |
| Approximate solutions: OMPL can return a path that stops short of the goal | none occurred; the smoke program counts only exact solutions | count only exact solutions |
| Length units | unweighted radians in the toy run | metres of motion (weighted) for both, and the duration of the timed trajectory, which is what the cell pays for |

### A proposed design

**Problems.** The four moves `print_plan` already knows, in the nominal scene
(`Reset(1, Randomization::None())`, obstacles from `SetScene`), with the scripted
cycle's own start and goal:

1. the carry of module 0 to slot 0 (the default);
2. the carry of module 29 to slot 9 (`--module 29 --slot 9`: the stack nearly empty,
   the tube nearly full);
3. the carry the other way round, past the camera's post (`--other-way`);
4. the bare arm's return (`--return`).

**Planners.** From OMPL 2.0.2 [S1, `src/ompl/geometric/planners`]:

- `RRTConnect`, the same algorithm. With range, edge check and box made equal, what
  remains is the implementation: this is the row that says whether the hand-written
  planner has a defect.
- `BKPIECE1` or `BiTRRT`, a different way of exploring, for whether RRT-Connect is the
  right family for this cell's passages.
- `LazyPRM`, because the cell plans nearly the same carry for every module of a pallet;
  a roadmap that is reused is a different trade from planning afresh each time.
- `InformedRRTstar` and `BITstar` (or `AITstar`), which improve a path for as long as
  they are given. Run with 1 s and 10 s they estimate how short the carry can be, which
  is not known, and so how far the shortened paths are from it.
- `AORRTC`, new in the 2.x line; OMPL's release notes call it asymptotically optimal
  RRT-Connect [S3].

VAMP is not a candidate: it brings its own robot models and collision checking.

**Measures.** Per planner and problem, over 50 plans (`print_plan`'s default): paths
found within the budget; collision queries in search and in shortening, median and
90th percentile; wall time, and time net of collision queries; length in metres of
motion before and after shortening; smallest clearance; duration of the trajectory
timed by `src/path_timing.h` at the cycle's speed. OMPL's rows for three process
seeds, to show how much a set of 50 varies.

**The table.** One row per planner and problem, the columns above, with the scripted
cycle's own path as the first row, as `print_plan` prints it today. A second, smaller
table for the single change that matters most, each time with everything else equal:
OMPL's default range, OMPL's default edge check, OMPL's simplifier.

**Where it lives.** A binary `//:print_ompl_plan` beside `print_plan`, tagged `manual`,
with the adapter in its own header; nothing in `:motion_planner`, `:pick_place` or the
cycle depends on it.

### What an integration option must offer the comparison

| Property | Why | O1 | O2 | O3 | O4 | O5 | O6 | O7 | O8 |
|---|---|---|---|---|---|---|---|---|---|
| Links into one binary with `CollisionChecker` and MuJoCo | the validity check is called in OMPL's loop | yes | yes | yes | yes | yes | yes | yes | no |
| Both planners compiled by one compiler with the same flags | timings mean the same | yes (`-O2`, measured) | same compiler, but OMPL at `-O3` (measured) | yes (`-O2`, measured), with the registry's extra definitions | yes (as O1, measured) | yes | yes | no: the packager's | no |
| One Eigen in the binary | OMPL's headers include Eigen; the adapter passes vectors across | yes (5.0.1) | yes (the same `@eigen`, through a shim) | yes (5.0.1.bcr.2 still resolved) | yes | yes | yes | no: the package's Eigen beside 5.0.1 | does not arise |
| The same OMPL on every machine and on CI | numbers that can be compared between machines | yes, pinned | yes, pinned | yes, pinned by the lock file | yes, pinned | yes | yes | not with `apt` (1.6.0 and 1.5.2) | by a pinned wheel, 2.0.1 |
| OMPL's source at hand | reading why a planner does what it does; a counter or a patch if needed | in Bazel's external directory | in Bazel's external directory | external, from ROS's release repository | external | external | in the tree | no | no |
| Cost when the comparison is not being run | the cell's build and CI should not pay | none: `manual`, not fetched by `//...` (measured) | not fetched, but three module versions move (measured) | high: 16 module versions move and the viewer's libraries are rebuilt, OMPL built or not (measured) | none (measured) | none | repository size | none | none |

## Vendoring C++ libraries into Bazel

This section is meant to be read without the rest. It is about a C++ library that is
not in the Bazel Central Registry and builds with CMake, and a Bazel project (bzlmod)
that wants it as a pinned, reproducible dependency. "Vendoring" here means that, by
whatever mechanism; copying the source into the tree is one mechanism of several.

### The approaches

| | Approach | What the project keeps | What builds the library |
|---|---|---|---|
| A | It is in a registry after all: `bazel_dep` | one line | the registry's build file |
| B | A second registry that has it (a company's, a community's), added with `--registry` | one line and a registry address | that registry's build file, with that registry's assumptions |
| C | Fetch the release archive by hash (`http_archive`) and write the build file yourself with native rules (`cc_library`) | a fetch rule and a build file | Bazel, one action per source file |
| D | The same build file packaged as a module in a registry of your own, or contributed to the central one | a registry directory, or nothing | Bazel |
| E | `rules_foreign_cc`: Bazel runs the library's own CMake build as one action | a fetch rule, a `cmake()` target, and whatever makes CMake find its dependencies | CMake, inside Bazel |
| F | The source in the tree (copy, subtree, submodule) with a build file beside it | the source | Bazel |
| G | A prebuilt binary: upstream's release binaries fetched by hash and wrapped with `cc_import`, or a system package wrapped by a repository rule | a fetch or wrap rule | nobody; it is built elsewhere |
| H | Outside Bazel: the part that needs the library is a separate program | nothing | another build system |

C, D and F use the same build file; they differ in where it and the source live. E and
G avoid writing one. Hybrids are common: a native build file for the C++ part and a
prebuilt binary for a part in another language.

### The properties of a library that decide

| Property | What to look for | What it decides |
|---|---|---|
| Generated headers and configure-time checks | `configure_file`, `check_cxx_source_compiles`, `check_include_file` in its CMake files | A version header and a few fixed switches can be written by a `genrule` or a template rule (C). Dozens of platform checks mean a `config.h` per platform by hand, or E, or G |
| Dependencies, and whether they are in the registry | `find_package`, `FetchContent`, `ExternalProject_Add`; then look each one up | Every required dependency has to come from somewhere first. All in the registry: C stays cheap, and E gets harder, because CMake has to be made to find libraries that Bazel built. One that is missing goes through this same procedure before the library does. A build that downloads at configure time does not run in Bazel's sandbox |
| Code generation | sources produced by a tool at build time (protobuf, flatbuffers, the project's own generator) | Checked-in generated sources cost nothing. A generator that must run needs a Bazel rule for it (C becomes dear) or E |
| Git submodules | `.gitmodules` | GitHub's automatic archive of a tag leaves submodule directories empty. Use an archive upstream uploads with them inside, or fetch each submodule as its own repository, or leave the feature out |
| Flags the library imposes on its users | `PUBLIC` or `INTERFACE` compile options and definitions; `-march=native` | A definition its headers need goes in `defines` of the library target. Flags for its own files go on its target (`copts`, `local_defines`), never in `.bazelrc`. A library that insists on `-march=native` for users cannot be cached across machines |
| Number of sources and how they are listed | a `glob` in its CMake, or a list with conditions | A `glob` over a few directories translates directly. Lists with many conditions have to be followed by hand at every upgrade |
| Parts in other languages | Rust, Fortran, CUDA | G for that part (upstream's prebuilt artefact by hash), unless the project already has that toolchain in Bazel |
| How often it releases | release dates over two years | Each release repeats the upgrade work of the chosen approach. Monthly releases argue for the approach with the least per-release work, or for upgrading rarely on purpose |
| Whether upstream would take Bazel files, or the registry a module | issues that ask for Bazel; other build systems maintained in the tree | If yes, D pays back for everybody. If upstream has said no, the build file is yours to keep |
| Where it is used | in the product, or in a tool beside it | A tool that only reads the product's output can stay outside Bazel (H). Anything called in-process cannot |

### A decision procedure

1. Look in the Bazel Central Registry for the library at a version you can use. If it
   is there, add `bazel_dep` and stop.
2. Decide whether it is needed in-process. If a separate program that reads files would
   do, and nothing in the product links it, consider leaving it outside Bazel (H) and
   stop.
3. Read its build for half an hour and write down: the sources and how they are
   listed, generated files, configure-time checks, dependencies, submodules, flags it
   puts on its users, the languages involved.
4. Look up every required dependency in the registry. For each one that is missing,
   run this procedure on it first. If several are missing and large, go to step 7.
5. If the generated files are few and fixed, the sources can be listed by `glob` or a
   short list, and everything is C or C++: fetch the release archive by SHA-256 and
   write a native build file (C). Prefer an archive upstream uploads over a forge's
   automatic one. Compile it once with the project's own flags before committing to
   it.
6. For a part in another language, or one whose build cannot be reproduced, use
   upstream's prebuilt artefact, fetched by hash, one per platform behind a `select`
   (G), and keep step 5 for the rest.
7. If the build has heavy configure logic, runs its own generators, or step 4 failed:
   use `rules_foreign_cc` (E), and accept one opaque action, CMake's choice of flags,
   and the work of making CMake find Bazel's dependencies. If that fails too, a
   prebuilt package (G) is what remains.
8. Put the flags the library needs on its own target. Check with `bazel aquery` that
   no existing target's command line changed, and with `bazel mod graph` that no
   existing module changed version.
9. If a second project will want it, move the same files into a registry module (D): a
   directory used with `--registry`, or a pull request to the central registry if you
   will maintain it there.
10. Write the pin, the licence file, and the steps of an upgrade into the build file's
    opening comment; tag the targets `manual` if the product does not need them; have
    CI build them on a schedule so that a broken pin is found by CI and not by a
    person.

### Applied to Ruckig

Not built; everything below is from Ruckig's repository, read 2026-11-23
[S27].

| Fact | |
|---|---|
| Registry | not in the Bazel Central Registry, no pull request for it; the ROS registry has a module at the old 0.9.2 [S11][S32] |
| Build | CMake 3.15, C++20 (`target_compile_features(ruckig PUBLIC cxx_std_20)`) |
| Sources | 11 `.cpp` (about 102 KB) and 16 headers; a twelfth file for the cloud client |
| Generated files, configure checks, code generation, submodules | none |
| Dependencies | none required. The cloud client (`BUILD_CLOUD_CLIENT`, on by default) uses `nlohmann/json` and `cpp-httplib`, both copied into `third_party/` and both in the registry; it sends waypoint problems to `http://api.ruckig.com` |
| Flags on users | `WITH_CLOUD_CLIENT` as a public definition when the cloud client is on; nothing else |
| Releases | v0.19.4 on 2026-07-15; one to three a year; `main` is ahead of the last tag |
| Bazel upstream | no Bazel files, no issue asking for them |
| Licence | MIT |

The procedure: step 1, no. Step 2, a trajectory generator is called in-process. Step
3 finds almost nothing to write down. Step 4, no required dependency. Step 5 applies:
`http_archive` of the tag `v0.19.4` by hash, and a `cc_library` of the 11 files with
`includes = ["include"]`, the cloud client left out so that no definition is needed and
nothing talks to the network. Leaving it out also gives up intermediate waypoints,
which the open version computes only through that cloud service. That is about 20
lines; another project's build file
with the same 11 files exists as prior art [S28]. The repository is already C++20.
Step 9: a library this small with no dependencies is a good candidate for the central
registry. Expected cost: under an hour. Not verified by a build.

### Applied to Rerun's C++ SDK

Not built; from Rerun's repository at 0.43.2, read 2026-11-23 [S29].

| Fact | |
|---|---|
| Registry | not in the Bazel Central Registry [S11]. Maintainers: "we don't maintain Bazel support ourselves" (issue 5530), and they point to a community example |
| Build | CMake 3.16, C++17; the library is every `.cpp` under `src/` (300 files) |
| Code generation | most of the sources are generated (494 of the files shipped are in the generated type directories), and they are checked in and shipped; nothing to generate when consuming |
| Generated headers, configure checks, submodules | none |
| Dependency 1: `rerun_c` | a static library written in Rust. The SDK does not build it; the release ships it prebuilt for Linux x86-64, Linux arm64, macOS arm64 and Windows x64, 44 to 72 MB each, inside `rerun_cpp_sdk.zip` and as separate assets. Its version must match the headers exactly. Which C library versions the Linux archives need is not verified |
| Dependency 2: Apache Arrow C++ | required. By default Rerun's CMake downloads Arrow 18.0.0 at configure time and builds it with two patches. The registry has `apache_arrow` 25.0.1 only [S31]; whether Rerun 0.43.2 compiles against it is not verified |
| Flags on users | C++17 or newer; links `rerun_c` and, on Linux, `-lm -ldl -pthread`. Whether it compiles as C++20, which this repository forces, is not verified |
| Releases | about weekly (0.39.0 to 0.43.2 between 2026-10-12 and 2026-11-13), before 1.0 |
| Prior art | the community example uses `rules_foreign_cc` and says it is "stuck on 0.19.1" because Arrow 18 fails to build that way [S30] |
| Licence | MIT or Apache 2.0 |

The procedure: step 1, no. Step 2 is the real question for Rerun: it is a logging and
viewing tool. If the cell would only be looked at afterwards, a separate program that
reads what the cell already writes and logs it to Rerun (its Python package, outside
Bazel) costs nothing in the build, and the procedure stops there. If the cell's code is
to log while it runs, go on. Step 3: sources by `glob`, no configure step, one Rust
part, one large C++ dependency. Step 4: Arrow is in the registry, at another major
version than Rerun pins; this is the risk, and one compile and one recording that
opens in the viewer would settle it. Step 5 for
the SDK's own sources: `http_archive` of `rerun_cpp_sdk.zip` by hash, a `cc_library`
over `src/**/*.cpp` depending on `@apache_arrow`. Step 6 for `rerun_c`: `cc_import` of
the prebuilt archive for the platform, chosen by `select`; both of this project's
platforms (Linux arm64 for development, x86-64 for CI) are shipped. Step 7 only if
Arrow 25 does not work: then Arrow 18 would have to be built by `rules_foreign_cc`,
which is exactly what the community example reports as failing, and staying outside
Bazel becomes the better answer. The weekly releases and the exact version match argue
for upgrading on purpose, a few times a year, not for following upstream. Expected
cost if Arrow 25 works: half a day. Not verified by a build.

### What the worked example (OMPL) adds

OMPL sits between the two: 249 sources by `glob`, one generated header with seven
substitutions, two dependencies that are both in the registry, two submodules needed
only by optional parts, no flags on its users. Step 5 applied, and took one build file
of 75 lines without comments, and no patch. The same
library through `rules_foreign_cc` (step 7, tried here for comparison, not because the
procedure called for it) is described under O2.

## Decisions for the owner

Each decision has to be made before OMPL goes onto `main`. The recommended answer is
the report's; the reason is the measurement or the fact it rests on.

| # | Decision | Options | Recommended | Reason |
|---|---|---|---|---|
| 1 | How OMPL comes into the build | O1 to O8 | O1 native | With O4, one of the two built options that changed nothing in the existing build, needed no patch, and compile OMPL with the repository's own flags; of the two, the one without a registry to keep. Cached per file. See the summary. |
| 2 | Which archive to pin | the release asset upstream uploads (150 MB, with the submodules, hash published by upstream); or GitHub's archive of the tag (77 MB, submodule directories empty) | the release asset | Its hash can be checked against upstream's own; it is a file that was uploaded, not one generated on request; and VAMP's sources are in it should they be wanted. The cost is 73 MB more to download, once per cache. `src/ompl` is identical in the two. |
| 3 | The rule that fetches it | Bazel's `http_archive`; or a rule of the repository's own in `bazel/` with a `base_url`, like `kuka.bzl` | `http_archive` | It already does what `base_url` is for: `urls` takes a list and tries each in turn. The repository's own rules exist because their sources are loose files in large repositories, which an archive is not. The repository's written convention for dependencies outside the registry would need a sentence saying archives use `http_archive`. |
| 4 | The flags OMPL is compiled with | the repository's (`-O2`, C++20); or upstream's (`-O3`, C++17) | the repository's, for both planners | A yardstick should be compiled like what it measures. If upstream's optimization level is wanted for a second column, it is `copts = ["-O3"]` on the one target; C++17 for OMPL alone would go against `.bazelrc`'s rule and gains nothing known. |
| 5 | What may depend on OMPL, and how that is kept | by convention only; or checked | checked: `bazel query 'rdeps(//:all, //:ompl) except attr(tags, "manual", //:all)'` must print nothing, as a step of CI | The cell's control code must never link OMPL. Tags and a comment say so; a query that fails says so to whoever breaks it. A separate package with `visibility` would also do it, against the rule of one build file. |
| 6 | Whether CI builds and runs the OMPL targets | never; on every pull request; in a job of its own when the OMPL files change and on a schedule | a job of its own: on changes to `MODULE.bazel`, `MODULE.bazel.lock`, `bazel/ompl*` and `src/ompl*`, and weekly | A pin that is never built breaks unnoticed (an upstream asset withdrawn, a registry change to Boost). The six existing jobs should not pay for it. A cold build was 147.5 s on six arm64 cores here; on CI's x86-64 runners it is not verified. |
| 7 | Boost in the module graph | accept it; there is no alternative that keeps OMPL | accept | OMPL cannot be built without Boost. It adds 82 modules and 245 lines to the lock file and moved no existing version. |
| 8 | OMPL's optional parts | none; FLANN; VAMP; Python bindings | none | At 2.0.2 FLANN is used by one of OMPL's tests only; VAMP brings its own robot models and collision checking, which is not what is being compared; nothing here is in Python. |
| 9 | What the comparison compares | as proposed in `## What the comparison needs`, or narrower | as proposed: four moves, 50 plans each; OMPL's RRT-Connect set up like the planner here, plus its defaults as separate rows; one planner of another family, one roadmap, two that optimize | The owner should confirm three things in particular: that OMPL gets the planner's own box, metric and edge check through an adapter; that collision queries are the first measure and wall time the second; and the list of planners. |
| 10 | A registry of the owner's (O4) | now; when a second project or a second library needs it; never | when a second project needs OMPL, or a second library (Ruckig) is packaged the same way | It works (built and measured), it compiles exactly what O1 compiles, and moving from O1 to it took five minutes. Until something else uses the registry it adds two lines to `.bazelrc` that touch every module lookup, a `.bazelignore`, and integrity values kept by hand, for no gain. |
| 11 | Contributing the module to the Bazel Central Registry (O5) | now; later; never | later, and only after the build file has been through one OMPL upgrade and has been built on x86-64 | It would then be built on platforms nobody here has tried, and whoever sends it is asked to maintain it. Until then it costs more than it gives this project. |
| 12 | `bazel mod deps` failing on `main` | leave; fix in a change of its own | fix in a change of its own | Any check that the lock file is current stumbles on it, as the one planned for this report did. What the fix is was not looked into. |

### The spec that would follow

"OMPL on `main`, and the comparison", following the recommended answers.

**What lands on `main`, in which files.**

| File | Change | Lines (from the spike where it exists) |
|---|---|---|
| `MODULE.bazel` | the `http_archive` for OMPL and 17 `bazel_dep` lines for Boost | +41 |
| `MODULE.bazel.lock` | regenerated | +245 |
| `bazel/ompl.BUILD.bazel` | new, as on `spike/ompl-native` | 125 |
| `BUILD.bazel` | a `# BEGIN ompl` section: the alias `//:ompl`; `//:ompl_smoke`; a library for the adapter; `//:print_ompl_plan`; all tagged `manual` | about 60 |
| `src/ompl_smoke.cc` | as on the spike | 331 |
| `src/ompl_adapter.h`, `.cc` | new: state space in scaled coordinates, validity checker and motion validator over `CollisionChecker`, the box, the termination condition | about 250 |
| `src/print_ompl_plan.cc` | new: the four problems, the planners, the table | about 350 |
| `README.md` | the last line ("Not done: ...") replaced by the result and a pointer | a paragraph |
| `docs/` | the comparison's table and what it shows | one file |
| `.github/workflows/ci.yml` | the query of decision 5 in an existing job; one new job for decision 6 | about 25 |

The first three rows, the alias and `//:ompl_smoke` in `BUILD.bazel`, and
`src/ompl_smoke.cc` are the integration and can be taken from the spike branch as they
are (commits `b90da7c`, `9730735`, `8fa4250`). Nothing in `src/motion_planner.*`,
`src/pick_place.*`, the gate or the cycle changes; if the adapter needs the planner's
joint box or margins, it reads them from `MotionPlanReport`, or one small function is
factored out of `PlanMove` without changing what it does.

**What CI would do with it.** The existing six jobs: unchanged but for the one query,
which needs no build. The new job: `bazel build --config=opt //:ompl_smoke
//:print_ompl_plan` and `bazel run --config=opt //:ompl_smoke`, whose exit status is
the check. It does not run the comparison itself; the table is produced by hand on a
development machine, as `print_plan`'s is, because its timings mean nothing on a shared
runner. The workflow file is one that the access used for this report could not
change, so that change has to be made by the owner or allowed for.

**Rough size.** About 1,000 lines of code and build files besides the lock file. The
integration alone is under an hour, since it exists. The adapter, the program and the
runs are the work: four to six hours of the kind of work this report was, as a guess
and no more.

## How this was measured

### The machine

| | |
|---|---|
| Kind | an Incus system container (`systemd-detect-virt` prints `lxc`), Debian 13.7; every build was run as root |
| Processor | aarch64, 6 cores (`nproc`), on an Apple-silicon host (`lscpu`: vendor Apple) |
| Memory | 11,951 MiB (`free -m`) |
| Compiler | `gcc (Debian 14.2.0-19) 14.2.0`, from `build-essential`; the repository brings no toolchain |
| Bazel | 9.3.0, chosen by `.bazelversion`, through Bazelisk |
| Date | 2026-11-23 |

The cores are shared with other containers of the host. Before each timed build the
scripts record `nproc`, `uptime` and `free -m`; those values are beside each figure.

**Packages installed with `apt` for the spikes:** none, for any of the four. The
machine had Debian's `build-essential`, `git`, `curl`, `python3` and Bazelisk when the
work started, and no CMake, Ninja, Boost or Eigen; nothing was added.

### Commands

`$W` is a working directory outside the repository. Each spike was measured in a fresh clone of
its branch from `origin`, so that nothing uncommitted took part.

**Cold build, cached build, sizes**:

```
OB=$W/ob/<name>; RC=$W/cache/<name>-repo; DC=$W/cache/<name>-disk     # none exists yet
FLAGS="--config=opt --repository_cache=$RC --disk_cache=$DC --experimental_convenience_symlinks=ignore"
nproc; uptime; free -m
t0=$(date +%s.%N); bazel --output_base=$OB build $FLAGS //:ompl_smoke; t1=$(date +%s.%N)
du -sb $OB $OB/execroot $RC $RC/* $DC
bazel --output_base=$OB clean
t0=$(date +%s.%N); bazel --output_base=$OB build $FLAGS //:ompl_smoke; t1=$(date +%s.%N)
```

Wall time is `t1 - t0`. The action count is Bazel's own last line ("340 total
actions"). `--experimental_convenience_symlinks=ignore` keeps Bazel from leaving
`bazel-*` links in the clone; it does not change what is built. Bazel's own program
files (its install base under `~/.cache/bazel`) were already unpacked; everything else
was fetched and built from nothing.

**The smoke program and its per-plan lines:**

```
bazel --output_base=$OB run $FLAGS //:ompl_smoke
bazel --output_base=$OB run $FLAGS //:ompl_smoke -- --plans      # per-plan lines on standard error
```

The figure with a 0.99 s limit came from changing `kTimeLimit` in `src/ompl_smoke.cc`
from `5.0` to `0.99` in a working copy of `spike/ompl-native`, running
`bazel run --config=opt //:ompl_smoke` twice, and discarding the change.

**Compile flags and linking:**

```
bazel --output_base=$OB aquery $FLAGS 'mnemonic("CppCompile", inputs(".*RRTConnect\.cpp", deps(//:ompl_smoke)))' --output=text
bazel --output_base=$OB aquery $FLAGS 'mnemonic("CppLink", //:ompl_smoke)' --output=text
ldd "$(bazel --output_base=$OB info $FLAGS bazel-bin)/ompl_smoke"
```

The flags quoted are those of the action whose output is `_objs/ompl/RRTConnect.o`,
with the 332 include-path arguments, the file names and three `-D__DATE__`-style
redactions left out. The sizes of the library and the binary are from
`ls -l "$(bazel info --config=opt bazel-bin)"/ompl_smoke` in the measured clone and
`ls -l "$(bazel info --config=opt bazel-bin)"/external/+http_archive+ompl/libompl.a` in
the working copy the branch was written in (the measured clone, building from its disk
cache, does not keep the library; see below).

**Warnings with the filter off:**

```
bazel --output_base=$OB clean
bazel --output_base=$OB build $FLAGS --output_filter= //:ompl_smoke 2>&1 | grep -c 'warning:'
```

This build takes every action from the disk cache and runs no compiler; the warning
lines it prints are the compiler's output saved with each cached action, which Bazel
shows again. That is what is counted.

**Glue:**

```
git diff --stat origin/main...HEAD -- . ':(exclude)MODULE.bazel.lock'
git diff --stat origin/main...HEAD -- MODULE.bazel.lock
grep -vc '^\s*#\|^\s*$' bazel/ompl.BUILD.bazel      # O1: 75 lines that are not comments or blank
wc -l MODULE.bazel.lock                             # 463 on main; 9,434 on the O3 branch
grep -c pythonhosted MODULE.bazel.lock              # O3: 3,617
```

`origin/main` was `014ba3a` when these were taken.

**Module graph**:

```
bazel mod graph | grep -o '[A-Za-z0-9_.+-]*@[A-Za-z0-9_.+-]*' | sort -u      # on main, and on the branch
comm -3 main.txt branch.txt
```

`grep -c '^qt' branch.txt` and `grep -c '^boost' branch.txt` gave the 36 and 110 of O3.
`bazel mod graph` exits with status 2 on `main` and on every branch but
`spike/ompl-ros-registry`, after printing the graph; see "Found along the way" below.

**The repository's tests**:

```
bazel test --config=opt --config=agent --disk_cache=$W/cache/r8-disk //... --test_output=errors --nocache_test_results
```

run in this order: in a fresh clone of `main` with that disk cache empty; in the same
clone after `bazel clean --expunge`; then once in a fresh clone of each branch (O1, O2,
O4, O3); and for O4 a second time, in the same clone after `bazel clean --expunge`.
The count of
test targets is Bazel's line "Found 52 targets and 28 test targets" and its last line
"Executed 28 out of 28 tests: 28 tests pass". Whether `//...` fetched OMPL was checked
with `ls "$(bazel info output_base)/external" | grep -c -i ompl`, which printed 0.

**Acceptance checks on each branch:**

```
bazel query 'rdeps(//:all, //:ompl) except attr(tags, "manual", //:all)'     # prints nothing
bazel query 'attr(tags, "manual", //:ompl_smoke)'                            # prints //:ompl_smoke
bazel mod deps --lockfile_mode=error                                          # exits 2 on main, O1, O2, O4; 0 on O3; see below
bazel build --nobuild --lockfile_mode=error --config=opt //:ompl_smoke //...  # exits 0
git status --short                                                            # prints nothing afterwards
grep -rn 'sha256\|integrity' MODULE.bazel bazel/
```

**Machine speed before a timed build** (from the second measurement of O4 on):

```
t0=$(date +%s.%N); head -c 1G /dev/zero | sha256sum > /dev/null; t1=$(date +%s.%N)
for i in 1 2 3 4 5 6; do head -c 1G /dev/zero | sha256sum > /dev/null & done; wait; t2=$(date +%s.%N)
```

printed as "one" (`t1 - t0`) and "six at once" (`t2 - t1`). The cold builds of O1, O2
and O3 were taken before this existed. What speaks for them: O1 was built cold twice,
twelve minutes apart (148.7 and 147.5 s); O3's critical path (8.8 s) is O1's (8.9 s); O2
was built cold once, and the 126 s of its one large action agree with the 131 and
141 s noted for the same action while the branch was being written.

**O2: the compile line inside the CMake action.** In the working copy of
`spike/ompl-foreign-cc` at `a28edc9`, after `bazel build --config=opt //:ompl_smoke`
(the same commit as the measured clone, where the log of a cached action is not kept):

```
grep -m1 'planners/rrt/src/RRTConnect.cpp' \
  "$(bazel info --config=opt bazel-bin)/external/+http_archive+ompl/ompl_foreign_cc/CMake.log"
grep -c 'New Boost version may have incorrect' <the same file>          # 3
grep -n 'REQUIRED packages have not been found' -A3 <the same file>
grep -n 'Manually-specified variables were not used' -A3 <the same file>
```

The statement that all 249 files were compiled as C++17 before the patch is from the
same `grep` on the log of the branch's first build, before the patch existed, as noted
at the time; that build is not on the branch.

**Which existing actions a branch changes**, on fresh clones of `main` and of
each branch:

```
bazel aquery --config=opt --config=agent 'deps(//... except attr(tags, "manual", //...))' \
  --output=text --include_commandline=false --include_artifacts=false \
  | awk '/^  Mnemonic:/{m=$2} /^  Target:/{t=$2} /^  ActionKey:/{print $2, m, t}' | sort > keys.txt
comm -13 <(cut -d' ' -f1 keys_main.txt | sort -u) <(cut -d' ' -f1 keys_branch.txt | sort -u)
```

and, after the branch's test run, the objects that were compiled on the machine and not
taken from the cache. With a disk cache, Bazel did not write the object files of cache
hits into the output tree here: after `main`'s fully cached run the same `find` shows
one object file, and after O2's and O3's it shows the ones listed.

```
find "$(bazel info output_base)/execroot/_main/bazel-out" -name '*.o' \
  | sed 's|.*/bazel-out/||' | awk -F/ '{ if ($3 == "external") print $1, $4; else print $1, "(this repository)" }' | sort | uniq -c
```

For O1 the keys that differ from `main`'s are 173 bookkeeping actions (`SymlinkTree`,
`SourceSymlinkManifest`, `RepoMappingManifest`) and nothing else.

**Module graphs compared by name:**

```
join <(sed 's/@/ /' main.txt | sort) <(sed 's/@/ /' branch.txt | sort) | awk '$2 != $3'     # changed
grep -c 'bazel.ros.org' MODULE.bazel.lock; grep -c 'bazel.ros.org.*not found' MODULE.bazel.lock   # O3: 1407, 1009
grep -c '\.rcr\.' branch.txt                                                                   # O3: 195
```

**O3: which compiler.**

```
bazel cquery --config=opt //:rrt --toolchain_resolution_debug='@bazel_tools//tools/cpp:toolchain_type'
#   Selected @@rules_cc++cc_configure_extension+local_config_cc//:cc-compiler-aarch64
bazel aquery --config=opt 'mnemonic("CppCompile", inputs(".*RRTConnect\.cpp", deps(//:ompl_smoke)))' --output=text   # /usr/bin/gcc
ldd "$(bazel info --config=opt bazel-bin)/ompl_smoke"                                                                 # libpython3.12.so.1.0
grep -n -A2 register_toolchains "$(bazel info output_base)/external/rosdistro+/MODULE.bazel"                         # "@default_rust_toolchains//..."
```

The failing first build of O3 and its error are quoted under O3 in `## Options`; the
command was `bazel build --config=opt --config=agent //:ompl_smoke` at commit `cb1f7ee`
of that branch.

**O4: editing the registry.** In the working copy of `spike/ompl-own-module`, with
`F=registry/modules/ompl/2.0.2/overlay/BUILD.bazel` and `S=.../2.0.2/source.json`:

```
check() { bazel build --nobuild --config=opt //:ompl_smoke; tail -1 "$(bazel info output_base)/external/ompl+/BUILD.bazel"; }
bazel shutdown
echo "# an edit without a new integrity value" >> $F; check     # succeeds; last line ")": the copy fetched before
bazel shutdown; check                                          # the same
git checkout $F; bazel shutdown
echo "# an edit with its integrity value" >> $F
sed -i "s|\"BUILD.bazel\": \"sha256-[^\"]*\"|\"BUILD.bazel\": \"sha256-$(openssl dgst -sha256 -binary $F | base64)\"|" $S
check                                                          # a new server: last line is the edit
git checkout $F $S; check                                      # server still running: still the edit
bazel shutdown; check                                          # ")" again
echo "# second edit, server running" >> $F; (the same sed); check   # still ")": not noticed
git checkout $F $S; bazel shutdown
```

That O4's lock file is O1's: `diff <(git show spike/ompl-native:MODULE.bazel.lock)
<(git show spike/ompl-own-module:MODULE.bazel.lock)` prints nothing.

**What is from notes and not from the commands above.** O2, O3 and O4 were each
written in a working copy first, with a note kept of every build attempt. "Built at
the first attempt", "five minutes", the error of O3's first build and the times of
O2's large action during that work are from those notes. Every other figure is from a
fresh clone of the pushed branch with the commands above.

**OMPL's archive:**

```
curl -sL -o ompl-2.0.2.tar.gz https://github.com/ompl/ompl/releases/download/2.0.2/ompl-2.0.2.tar.gz
curl -sL https://github.com/ompl/ompl/releases/download/2.0.2/ompl-2.0.2.tar.gz.sha256
sha256sum ompl-2.0.2.tar.gz; ls -l ompl-2.0.2.tar.gz   # b9155e24...bcd29d, equal to the published value; 149,922,968 bytes
curl -sL -o tag.tar.gz https://github.com/ompl/ompl/archive/refs/tags/2.0.2.tar.gz
sha256sum tag.tar.gz                 # d867190a...0c3ae4; 77,135,911 bytes
diff -rq <release>/src/ompl <tag>/src/ompl      # no difference; nor in CMakeLists.txt, LICENSE
find src/ompl/{base,control,datastructures,geometric,multilevel,tools,util} -name '*.cpp' | wc -l    # 249
du -sb src/ompl; du -sh src/ompl; find src/ompl -type f | wc -l     # 5,693,418 bytes; 6.5M on disk; 581 files
find py-bindings -type f | wc -l                                    # 114
grep -rl 'GNU General Public' --include='*.h' --include='*.cpp' .   # 7 files, all under demos/multicopter
```

Statements about OMPL's source marked "by `grep`" were checked in the unpacked release
archive.

### Found along the way

None of these was fixed; they are listed for the owner.

- `bazel mod deps` and `bazel mod graph` exit with status 2 on `main` at `014ba3a`,
  with or without `--lockfile_mode=error`:
  `Error in fail: Unable to find interpreter for pip hub 'mujoco_pip' for
  python_version=3.15` and `ERROR: error evaluating module extension
  @@rules_python+//python/extensions:pip.bzl%pip`. The hub is declared by the `mujoco` 4.0.0 module;
  why it asks for a Python that is not registered was not looked into. Both
  commands still print the graph ("Results may be incomplete as 1 extension failed"),
  and builds and tests are not affected. Because of it `bazel mod deps
  --lockfile_mode=error`, the natural check that a branch's lock file is current, exits
  2 on the O1, O2 and O4 branches as it does on `main`; their lock files were checked
  with `bazel build --nobuild --lockfile_mode=error` instead. On the O3 branch it exits
  0 (below).
- `src/ompl` holds 251 `.cpp` files at 2.0.2, a number this work started from; the
  seven directories OMPL's CMake compiles hold 249. The other two are under
  `extensions/triangle`.
- A Bazel run with `--output_base` set leaves `bazel-<directory>` links in the checkout
  that a later Bazel with the default output base walks into under `//...`. The first
  attempt at the branch's test run failed that way within a second. It is a property of
  the measuring, not of any option.
- GNU `time` was not installed on the machine; the timings use `date +%s.%N`.
- The container's load average does not show what the host's other containers are
  doing. One cold build (O4's first) took two and a half times as long as its repeat,
  with the container idle by its own account. And the same 340 actions took 148 s and
  133 s in two undisturbed measurements two hours apart. Timings from this host are
  good to about a tenth at best; the comparison that follows should count collision
  queries first for this reason too.
- On `spike/ompl-ros-registry`, registering Python 3.15 in the root module makes
  `bazel mod graph` and `bazel mod deps` exit 0. The same four lines would probably do
  so on `main`; that was not tried there.
- `--config=opt` is defined for `build` only, so `bazel query` and `bazel mod` reject
  it; the acceptance commands that are queries were run without it.

### Sources

All read on 2026-11-23.

- [S1] OMPL's source at tag 2.0.2, https://github.com/ompl/ompl/tree/2.0.2 :
  `CMakeLists.txt`, `CMakeModules/OMPLDependencies.cmake`,
  `CMakeModules/OMPLCompilerSettings.cmake`, `CMakeModules/VampConfig.cmake`,
  `src/ompl/CMakeLists.txt`, `src/ompl/config.h.in`, `.gitmodules`, `LICENSE`,
  `src/ompl/base/src/Planner.cpp`, `src/ompl/base/src/PlannerTerminationCondition.cpp`,
  `src/ompl/geometric/planners/`. Read in the release archive. Searches of OMPL's
  issues and pull requests for "bazel": https://github.com/ompl/ompl/issues?q=bazel
- [S2] OMPL release 2.0.2 and its assets, https://github.com/ompl/ompl/releases/tag/2.0.2
- [S3] OMPL's release notes, https://github.com/ompl/ompl/blob/2.0.2/doc/markdown/releaseNotes.md
- [S4] OMPL issue 1429, https://github.com/ompl/ompl/issues/1429
- [S5] Bazel Central Registry, contribution guide, https://github.com/bazelbuild/bazel-central-registry/blob/main/docs/README.md ; and its note on the contributor agreement, https://github.com/bazelbuild/bazel-central-registry/blob/main/docs/contributing.md
- [S6] Bazel Central Registry, policies, https://github.com/bazelbuild/bazel-central-registry/blob/main/docs/bcr-policies.md
- [S7] Bazel, registries, https://bazel.build/external/registry
- [S8] Bazel, lock file, https://bazel.build/external/lockfile
- [S9] Bazel, vendor mode, https://bazel.build/external/vendor
- [S10] Bazel, command-line reference, https://bazel.build/reference/command-line-reference
- [S11] Bazel Central Registry, modules, https://github.com/bazelbuild/bazel-central-registry/tree/main/modules :
  no `ompl`, `ruckig` or `rerun*` directory; `flann/1.9.2`, `nanobind_bazel`,
  `eigen/5.0.1.bcr.5/presubmit.yml`, `boost.serialization/1.90.0.bcr.1/presubmit.yml`,
  `rules_foreign_cc/0.17.0`. Searches of its pull requests and issues for "ompl" and
  "ruckig" returned nothing.
- [S12] `rules_foreign_cc`, the `cmake` rule, https://bazel-contrib.github.io/rules_foreign_cc/cmake.html
- [S13] `rules_foreign_cc` 0.17.0 source, https://github.com/bazel-contrib/rules_foreign_cc/tree/0.17.0 :
  `README.md`, `MODULE.bazel`, `foreign_cc/private/cmake_script.bzl`,
  `foreign_cc/private/framework.bzl`, `docs/src/bzlmod_hub.md`,
  `toolchains/private/cmake_versions.bzl`
- [S14] `rules_foreign_cc` issues 1008, 1138 and 1430,
  https://github.com/bazel-contrib/rules_foreign_cc/issues/1008 , `/1138` , `/1430`
- [S15] ROS Central Registry, README, https://github.com/intrinsic-opensource/ros-central-registry/blob/main/README.md ; and its example workspace, https://github.com/intrinsic-opensource/ros-central-registry/tree/main/examples (`.bazelrc`, `MODULE.bazel`)
- [S16] ROS Central Registry, the `ompl` module, https://github.com/intrinsic-opensource/ros-central-registry/tree/main/modules/ompl
- [S17] ROS Central Registry, `rosdistro` `lyrical.2026-10-05.rcr.4`, https://github.com/intrinsic-opensource/ros-central-registry/tree/main/modules/rosdistro
- [S18] Debian 13 `libompl-dev`, https://packages.debian.org/trixie/libompl-dev
- [S19] Ubuntu 24.04 `libompl-dev`, https://packages.ubuntu.com/noble/libompl-dev
- [S20] GitHub's runner images, https://github.com/actions/runner-images/blob/main/README.md
- [S21] conda-forge `ompl`, https://api.anaconda.org/package/conda-forge/ompl and https://github.com/conda-forge/ompl-feedstock/blob/main/recipe/recipe.yaml
- [S22] PyPI `ompl`, https://pypi.org/pypi/ompl/json
- [S23] vcpkg's port, https://github.com/microsoft/vcpkg/blob/master/ports/ompl/vcpkg.json ; Homebrew's formula, https://formulae.brew.sh/api/formula/ompl.json
- [S24] ROS `rosdistro` distribution files, https://github.com/ros/rosdistro (`humble`, `jazzy`, `kilted`, `lyrical`, `rolling`)
- [S25] Boost Software License 1.0, https://www.boost.org/LICENSE_1_0.txt
- [S26] Mozilla Public License 2.0, https://www.mozilla.org/en-US/MPL/2.0/
- [S27] Ruckig, https://github.com/pantor/ruckig : `CMakeLists.txt`, `README.md`, `LICENSE`, the file tree, the releases list, `.github/workflows/ci.yml`
- [S28] A build file for Ruckig in another project, https://github.com/RoboCup-MSL/MRA-components/blob/d985c19bf1cb5b27ffd678b6c36ae00bae72c444/third-party-libraries/ruckig/BUILD
- [S29] Rerun at 0.43.2, https://github.com/rerun-io/rerun/tree/0.43.2 : `rerun_cpp/CMakeLists.txt`, `rerun_cpp/download_and_build_arrow.cmake`, `rerun_cpp/README.md`; the release's assets, https://github.com/rerun-io/rerun/releases/tag/0.43.2 ; issues https://github.com/rerun-io/rerun/issues/5530 and https://github.com/rerun-io/rerun/issues/4579
- [S30] The community's Bazel example for Rerun, https://github.com/kyle-figure/bazel-minimal-rerun
- [S31] Bazel Central Registry, `apache_arrow`, https://github.com/bazelbuild/bazel-central-registry/tree/main/modules/apache_arrow
- [S32] ROS Central Registry, the `ruckig` module, https://github.com/intrinsic-opensource/ros-central-registry/tree/main/modules/ruckig
- [S33] Bazel's source, how each kind of registry is checked against the lock file, https://github.com/bazelbuild/bazel/blob/master/src/main/java/com/google/devtools/build/lib/bazel/bzlmod/RegistryFactoryImpl.java

