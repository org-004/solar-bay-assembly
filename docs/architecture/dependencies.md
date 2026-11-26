# The tree toward the outside

This document lists everything the repository takes from outside itself: what it is,
where it comes from, under what licence, what it is for and what changes when it is
updated. It serves P4 of `docs/architecture/principles.md` and ADR 0016, which says
what an outside library must be to be added. Every figure was printed in the checkout
on 2026-11-25; the command stands beside it.

## The rows are the measured list

`tools/architecture/measure.sh` prints `P4 26 outside dependencies`, and
`tools/architecture/measure.sh --list P4` prints the 26 names. They are the 23
`bazel_dep` lines of `MODULE.bazel` (6 libraries and the 17 `boost.*` modules) and the
3 fetches of a repository rule: `menagerie_ur10e`, `kuka_kr210_r2700_2` and `ompl`.
The tables below have one row for each of the 26, in the order of `MODULE.bazel`.
BCR is the Bazel Central Registry (`https://bcr.bazel.build`, the registry that
`MODULE.bazel.lock` names for every module). Licences were read from the files under
`$(bazel info output_base)/external/<name>+/`, named by what the file says.

## Identity: version or hash, source, licence

| name | version, or commit and hash | comes from | licence (file read) |
|---|---|---|---|
| rules_cc | 0.2.28 (`MODULE.bazel:9`) | BCR | Apache-2.0 (`LICENSE`) |
| mujoco | 4.0.0 (`MODULE.bazel:12`) | BCR | Apache-2.0 (`LICENSE`) |
| eigen | 5.0.1.bcr.2 (`MODULE.bazel:16`) | BCR | MPL-2.0 (`LICENSE`; `COPYING.README`) |
| glfw | 3.4.0.bcr.4 (`MODULE.bazel:19`) | BCR | zlib/libpng (`LICENSE.md`) |
| nlohmann_json | 3.12.0.bcr.2 (`MODULE.bazel:22`) | BCR | MIT (`LICENSE.MIT`) |
| googletest | 1.18.0.bcr.1 (`MODULE.bazel:25`) | BCR | BSD-3-Clause (`LICENSE`) |
| menagerie_ur10e | commit `370c43965323` (`MODULE.bazel:35`); 23 files, e.g. `LICENSE` `5ec71ccf66c8` (`MODULE.bazel:37`) | `raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{commit}/universal_robots_ur10e` (`bazel/menagerie.bzl:50`) | BSD-3-Clause (its `LICENSE`, ROS Industrial copyright) |
| kuka_kr210_r2700_2 | commit `d2fa4377b77b` (`MODULE.bazel:75`); 15 files, e.g. `LICENSE` `b246743db274` (`MODULE.bazel:77`) | `raw.githubusercontent.com/kroshu/kuka_robot_descriptions/{commit}` (`bazel/kuka.bzl:58`) | Apache-2.0 (its `LICENSE`) |
| ompl | 2.0.2, sha256 `b9155e243e55` (`MODULE.bazel:112`) | `github.com/ompl/ompl/releases/download/2.0.2/ompl-2.0.2.tar.gz` (`MODULE.bazel:114`) | BSD-3-Clause (`LICENSE`, "3-clause BSD") |
| boost.algorithm | 1.90.0.bcr.1 (`MODULE.bazel:121`) | BCR | BSL-1.0 (`LICENSE`) |
| boost.assert | 1.90.0.bcr.1 (`:122`) | BCR | BSL-1.0 (header comments) |
| boost.concept_check | 1.90.0.bcr.1 (`:123`) | BCR | BSL-1.0 (`LICENSE`) |
| boost.config | 1.90.0.bcr.1 (`:124`) | BCR | BSL-1.0 (header comments) |
| boost.dynamic_bitset | 1.90.0.bcr.1 (`:125`) | BCR | BSL-1.0 (`LICENSE`) |
| boost.foreach | 1.90.0.bcr.1 (`:126`) | BCR | BSL-1.0 (header comments) |
| boost.functional | 1.90.0.bcr.1 (`:127`) | BCR | BSL-1.0 (header comments) |
| boost.graph | 1.90.0.bcr.2 (`:128`) | BCR | BSL-1.0 (header comments) |
| boost.math | 1.90.0.bcr.1 (`:129`) | BCR | BSL-1.0 (`LICENSE`) |
| boost.numeric_odeint | 1.90.0.bcr.1 (`:130`) | BCR | BSL-1.0 (`LICENSE`) |
| boost.property_map | 1.90.0.bcr.1 (`:131`) | BCR | BSL-1.0 (`LICENSE`) |
| boost.random | 1.90.0.bcr.1 (`:132`) | BCR | BSL-1.0 (header comments) |
| boost.range | 1.90.0.bcr.1 (`:133`) | BCR | BSL-1.0 (header comments) |
| boost.serialization | 1.90.0.bcr.1 (`:134`) | BCR | BSL-1.0 (header comments) |
| boost.smart_ptr | 1.90.0.bcr.1 (`:135`) | BCR | BSL-1.0 (header comments) |
| boost.thread | 1.90.0.bcr.1 (`:136`) | BCR | BSL-1.0 (header comments) |
| boost.type_traits | 1.90.0.bcr.1 (`:137`) | BCR | BSL-1.0 (header comments) |

The file lines for the per-file hashes are `MODULE.bazel:37` to `59` (Menagerie, 23
entries, counted with `sed -n 36,60p MODULE.bazel | grep -c '": "'`) and `77` to `91`
(KUKA, 15 entries). The Menagerie files are the UR10e's `scene.xml`, `ur10e.xml`, 20
meshes and the `LICENSE`, all at commit `370c43965323`. The KUKA files are 14 STL
meshes (7 visual, 7 collision) and the `LICENSE`, at commit `d2fa4377b77b`.
A "header comments" licence means the fetched tree has no licence file: no `LICENSE*` or
`COPYING*` exists in `boost.<name>+/` (listed with `find`), and the headers say
"Distributed under the Boost Software License, Version 1.0" (`grep -rl "Boost Software
License" boost.<name>+/include` finds from 2 to 200 headers in each). The Boost
licence files that exist read "Boost Software License - Version 1.0". None of the 26
has a licence that restricts use as the GPL does. Eigen's MPL-2.0 is file-level
copyleft: changes to Eigen's own files would have to be published, and use of it in
this repository's code does not. The MuJoCo model files are data under permissive
licences (BSD-3-Clause, Apache-2.0).

## Use: what for, which targets, tests only, update

The target counts are the `P4.<library>` lines of `tools/architecture/measure.sh`
(targets of `BUILD.bazel` that list the library directly); where there is none, the
count is from `grep -c`.

| name | what it is for here | targets listing it | only tests or tools? | update |
|---|---|---|---|---|
| rules_cc | `cc_library`, `cc_binary`, `cc_test` (`BUILD.bazel:1` to `3`) | 0 (3 `load` lines, `grep -c '@rules_cc' BUILD.bazel`) | build rules | registry module |
| mujoco | the physics engine, C API | 67 (`P4.mujoco`) | no | registry module, then the gate |
| eigen | vectors and matrices | 78 (`P4.eigen`) | no | registry module |
| glfw | window and OpenGL context for the viewer | 1 (`P4.glfw`; `viewer_lib`) | tool (viewer, needs a display) | registry module |
| nlohmann_json | JSON of the layouts (`src/layout.h`) | 4 (`P4.nlohmann_json`) | no | registry module |
| googletest | the test framework | 42 (`P4.googletest`) | tests only (`dev_dependency = True`) | registry module |
| menagerie_ur10e | the stand-in arm, UR10e, with scene | 3 lines of `BUILD.bazel` name it (`grep -c`); go through `UR10E_FILES` (`BUILD.bazel:12`) | no: tests and programs of the stand-in cell | files by hash |
| kuka_kr210_r2700_2 | the KR 210 R2700-2 meshes and licence; `models/arm.xml` is hand-made | 3 lines name it; go through `ARM_MESH_FILES` (`BUILD.bazel:189`) | no | files by hash |
| ompl | the planner's second search (`src/ompl_search.h`) | 1 (`P4.ompl`); go through `//:ompl` (`BUILD.bazel:1585`) | no: every target that plans links it | hash-pinned archive |
| boost.* (each of the 17) | headers OMPL includes; only `boost.serialization` compiles code | 0 in `BUILD.bazel`; each is listed once, by `@ompl//:ompl` (`bazel/ompl.BUILD.bazel:107` to `123`) | no: linked through OMPL | registry module |

The rows of the 17 Boost modules are the same for every module, and the next section
says which are needed. In the first table each module has its own row.

## The 17 Boost modules and OMPL

Commands: `bazel query 'deps(@ompl//:ompl)' --output=package | grep boost` prints
all 17; `grep -n 'boost' bazel/ompl.BUILD.bazel MODULE.bazel` shows each listed once
by the build file and once in `MODULE.bazel`. Because the build file lists all 17,
the first query cannot tell them apart. Two further tests can. The fourth column
asks whether the module is still reached when its own edge is left out:
`bazel query "deps(set(<the other 16> @eigen//:eigen)) intersect @boost.X//:boost.X"`
(1 line means reached). The fifth counts `#include <boost/X` lines in the fetched
OMPL tree, `grep -rhoE '#\s*include\s*<boost/X(/|\.hpp)' $(bazel info
output_base)/external/+http_archive+ompl/src/ompl | wc -l`.

| module | in `MODULE.bazel` | in deps of `@ompl//:ompl` | listed directly by OMPL's build file | reached without its own edge | `#include` lines in OMPL |
|---|---|---|---|---|---|
| boost.algorithm | yes | yes | yes | yes | 1 |
| boost.assert | yes | yes | yes | yes | 1 |
| boost.concept_check | yes | yes | yes | yes | 8 |
| boost.config | yes | yes | yes | yes | 0 |
| boost.dynamic_bitset | yes | yes | yes | yes | 1 |
| boost.foreach | yes | yes | yes | yes | 12 |
| boost.functional | yes | yes | yes | yes | 3 |
| boost.graph | yes | yes | yes | no | 59 |
| boost.math | yes | yes | yes | yes | 40 |
| boost.numeric_odeint | yes | yes | yes | no | 7 (`boost/numeric/odeint`) |
| boost.property_map | yes | yes | yes | yes | 8 |
| boost.random | yes | yes | yes | yes | 6 |
| boost.range | yes | yes | yes | yes | 7 |
| boost.serialization | yes | yes | yes | yes | 9 |
| boost.smart_ptr | yes | yes | yes | yes | 6 (`boost/scoped_ptr.hpp`) |
| boost.thread | yes | yes | yes | yes | 1 |
| boost.type_traits | yes | yes | yes | yes | 1 |

### Not needed, by the query

No module is outside the dependencies of `@ompl//:ompl`: all 17 are in them, so none
fails the first query. By the second test, 15 are reached through the others as well,
and only `boost.graph` and `boost.numeric_odeint` need their own edge. The one module
that OMPL's sources never include by name and that is reached without its own edge is
`boost.config`: it is a candidate to remove from `MODULE.bazel` and from
`bazel/ompl.BUILD.bazel`. Nothing shows that a registry version needs it: the other
Boost modules ask for `boost.config` at the same `1.90.0.bcr.1`
(`grep -h 'bazel_dep(name = "boost.config"' $(bazel info output_base)/external/boost.*+/MODULE.bazel | sort | uniq -c`
printed one line, `75` times). The other 14 are included by name by OMPL's sources
(last column), so they are not candidates. Nothing was removed, and no build was run
with one removed, so the query is evidence and not proof.

## Updating

A registry module: change the version in `MODULE.bazel`, run
`bazel mod tidy` or build once to rewrite `MODULE.bazel.lock`, then run
`bazel build --nobuild --lockfile_mode=error --config=opt //...` (it must succeed:
the lock is current), `bazel test --config=opt --config=agent //...` (everything but
the gate) and `tools/architecture/measure.sh --check`. For `mujoco` also the gate,
alone on the machine (`bazel test --config=opt //...`, `docs/gate.md`), since every
figure changes.

A hash-pinned archive (`ompl`): put the new URL, `strip_prefix` and the `sha256` the
maintainers publish beside the release in `http_archive`; if OMPL's source layout
changed, adjust `bazel/ompl.BUILD.bazel` (its `_DIRS`, the `config_h` rule). The
same checks follow, and `bazel test --config=opt --config=agent //:ompl_search_test
//:motion_planner_test` first.

Files by hash (`menagerie_ur10e`, `kuka_kr210_r2700_2`): set the new `commit` and
the hash of every file in `files`; a wrong hash fails the fetch. The same checks
follow; a changed mesh may also move the figures of the cell, so the gate is run
before the change lands.

## What reaches the code without being a `bazel_dep`

MuJoCo's own dependencies are not in `MODULE.bazel`. `bazel query
'deps(@mujoco//:mujoco)' --output=package | sed 's/\/\/.*//' | sort -u` lists 13
packages, 10 of them outside the repository's own list: `ccd`, `lodepng`,
`marchingcubecpp`, `miniz`, `package_metadata`, `platforms`, `qhull`,
`rules_license`, `tinyobjloader` and `tinyxml2`. Their licences were not read for this
document: not established. The other 3 are `@mujoco`, `@rules_cc` and `@bazel_tools`.
Eigen is not among them, although the comment in `MODULE.bazel` says MuJoCo depends on
it. Bazel also loads its own modules (`rules_java`, `rules_python`, `abseil-cpp`,
`zlib` and others, listed under `$(bazel info output_base)/external/`); they do not
reach this repository's code.

`bazel mod graph` and `bazel mod deps` fail on this repository. `bazel mod graph`
prints `ERROR: error evaluating module extension
@@rules_python+//python/extensions:pip.bzl%pip`, with the cause `Error in fail:
Unable to find interpreter for pip hub 'mujoco_pip' for python_version=3.15`, and
ends `ERROR: Results may be incomplete as 1 extension failed.` (it still prints the
tree, with the extension's result missing). What is used instead is `bazel query`
on labels, as above, `MODULE.bazel.lock`, and `bazel mod graph` read for its tree
while ignoring that error.

`MODULE.bazel.lock` (lock file version 28, 554 registry file hashes, 4 module
extensions, from `python3 -I -c` over the JSON) holds the hash of every registry file
read while resolving the 23 modules and what they pull in. The command that checks
it is current is `bazel build --nobuild --lockfile_mode=error --config=opt //...`;
it printed `Build completed successfully, 0 total actions`.

## The lines of `tools/architecture/measure.sh` that hold this tree

Run `tools/architecture/measure.sh | grep '^P4'`; today it prints:

| line | figure | what it counts |
|---|---|---|
| `P4` | 26 | outside dependencies in `MODULE.bazel`; may not grow |
| `P4.unpinned` | 0 | of them not pinned by a version, a commit, an integrity or a sha256 |
| `P4.mujoco` | 67 | targets that list `@mujoco` directly |
| `P4.eigen` | 78 | targets that list `@eigen` directly |
| `P4.googletest` | 42 | targets that list `@googletest` directly |
| `P4.nlohmann_json` | 4 | targets that list `@nlohmann_json` directly |
| `P4.ompl` | 1 | targets that list `@ompl` directly |
| `P4.glfw` | 1 | targets that list `@glfw` directly |
| `P4.mujoco_files` | 70 | files that include `mujoco/` |
| `P4.group` | 0 | `#include` lines of an outside library from a group that may not |
| `P4.allowed` | 22 | pairs (group, outside library), a rule of `deps.json` |

The 22 pairs are listed by `tools/architecture/measure.sh --list P4.allowed`.
