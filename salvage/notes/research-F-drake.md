# Research F: Drake (RobotLocomotion/drake, branch master), read in its source

Commit read: `5186757f28d541f90428158c123fd84585e8eaee` (via `gh api repos/RobotLocomotion/drake/commits/master --jq .sha`, 2026-11-25).
Base for citation: `https://github.com/RobotLocomotion/drake/blob/master/<path>`; fetched from `https://raw.githubusercontent.com/RobotLocomotion/drake/master/<path>`. Counts below are mine, made with grep over the fetched files, unless a source is named.

## Answers to the seven questions

### 1. How the tree is divided
- Root listing (GitHub contents API, not a file): top-level directories are `bindings, cmake, common, doc, examples, geometry, lcm, lcmtypes, manipulation, math, multibody, perception, planning, setup, solvers, systems, third_party, tools, tutorials, visualization`, plus `.github, .jenkins`. Root files include `.clang-format, .clang-tidy, CPPLINT.cfg, MODULE.bazel, WORKSPACE, CMakeLists.txt, CONTRIBUTING.md, .ruff.toml, .bazelrc`. There is no CODEOWNERS at root or in `.github/` (the contents API returned 404 for `.github/CODEOWNERS`; `.github/` holds `ISSUE_TEMPLATE, renovate.json, workflows`). The full tree has 348 `BUILD.bazel` files (git trees API, recursive).
- Nesting (subdirectory listings via the contents API): `common/` has 8 subdirectories (`benchmarking proto schema symbolic test test_utilities trajectories yaml`); `math/` only `test`; `geometry/` 11 (`proximity, query_results, render, render_gl, render_gltf_client, render_vtk, optimization, profiling, benchmarking, test, test_utilities`); `multibody/` 20 (`plant, tree, parsing, contact_solvers, fem, hydroelastics, inverse_kinematics, meshcat, topology ...`); `systems/` 9 (`framework, primitives, controllers, analysis, sensors ...`); `planning/` 6 (`experimental, graph_algorithms, iris, locomotion, trajectory_optimization ...`); `solvers/` only `benchmarking, test, test_utilities`; `manipulation/` 5 robot-specific dirs. Depth is typically 2 below the root (`geometry/proximity`, `multibody/plant`), and every directory with C++ is one Bazel package. Each library directory has its own `test/` subdirectory and often `test_utilities/` and `benchmarking/`.
- Dependency direction is stated in prose, not enforced by a tool I found: `doc/_pages/directory_structure.md` says the layout is meant to "group functional components, limit dependencies, prevent circular dependencies"; gives the example that `systems` depends on `solvers`, so `solvers` must not depend on `systems`; `examples` are logically after the core libraries; `test` directories are logically last and may depend on anything if the dependency is declared in the build rule. It also says code under `experimental/` or `dev/` may be depended on only by other code in such directories.
- The rule is not kept perfectly: `math/BUILD.bazel` makes `//math:continuous_algebraic_riccati_equation` depend on `//systems/primitives:linear_system_internal` (math -> systems) with the comment "TODO(jwnimmer-tri) Move this code into //math to avoid the circular dependency", and `math:compute_numerical_gradient_test` depends on `//solvers:evaluator_base`. And `common/BUILD.bazel` mentions `//multibody`, `//geometry` or `//systems` only once, at line 1299, as `//systems/framework/test_utilities:my_vector` (a dependency of a test target; I did not read the target around it to confirm it is a test). So the stated direction is common < math < solvers < systems < geometry/multibody < planning/manipulation < examples, but the build files contain at least one known exception with a TODO, and no "layering test" was found in the files I read.

### 2. How targets and visibility are cut
- Every BUILD file I read starts with `package(default_visibility = ["//visibility:public"])` (common line 18, math line 9, planning line 9, geometry/proximity line 14; `multibody/plant` sets it inside a `package(...)` call at line 10). So the default is open; narrowing is done per target with `visibility = [...]` (e.g. `common/BUILD.bazel` has `["//common/test_utilities:__pkg__"]`, `["//tools/install/libdrake:__pkg__"]`, `["//:__subpackages__"]`, `["//bindings/pydrake/common:__pkg__"]`, `["//visibility:private"]`).
- One library per header (or per small header group) with a rollup: `drake_cc_library` counts per file: common 51, math 22, multibody/plant 24, planning 18, geometry/proximity 64. `drake_cc_googletest` counts: common 63, math 33, multibody/plant 76, planning 14, geometry/proximity 72. Each package ends with a `drake_cc_package_library` (named after the directory) listing all of them (planning: 16 deps) and `add_lint_tests()`.
- `tools/skylark/drake_cc.bzl` `drake_cc_library` takes `implementation_deps` (include-what-srcs-use; "firewall" for headers and defines), `internal = True` (fails unless `visibility` is set and not public; excludes headers from install and adds tags `exclude_from_libdrake`, `exclude_from_package`), `install_hdrs_exclude`, `compile_once_per_scalar`, `declare_installed_headers`. Counts: `internal = True` in geometry/proximity 11, common 2, multibody/plant 2, math 0, planning 0; `implementation_deps` in geometry/proximity 14, common 9, planning 4, math 2, plant 1; `install_hdrs_exclude` in common 3.
- `drake_cc_package_library` fails the analysis if no visibility is given, if the name differs from the directory name, or if used in `examples/`; `library_lint` (tools/lint) checks that its deps are exactly the package's non-testonly, non-excluded `cc_library`s. Libraries must be made with the macro; deps must be drake libraries or `@...` externals.
- Tests: `drake_cc_googletest(name = "x_test", deps = [...])` defaults `srcs = ["test/x_test.cc"]`, `size = "small"` (so only exceptions state a size), forces `testonly`, adds the default gtest main from `//common/test_utilities`. Exceptions in files read: `timeout = "moderate"` (7 in multibody/plant), `timeout = "long"` (1), `shard_count` 2 or 3, `data = [...]`, `tags = ["nolint"]` only on filegroups. Test helper libraries live in `test/` and are tagged to stay out of the package library (`exclude_from_package`).
- Library headers are dropped in `.cc`-private form: `.h` files in `srcs` become private headers (`_prune_private_hdrs`).

### 3. Warnings and lint, exactly
- `tools/skylark/drake_cc.bzl` `CXX_FLAGS` (all compilers): `-Werror=all, attributes, cpp, deprecated, deprecated-copy-dtor, deprecated-declarations, ignored-qualifiers, missing-declarations, overloaded-virtual, shadow, unused-result`, plus `-DEIGEN_NO_IO=1`. `GCC_FLAGS` adds `-Werror=extra, logical-op, non-virtual-dtor, pessimizing-move, return-local-addr, uninitialized, unused-but-set-parameter` and turns off, each with a comment about a false positive: `-Wno-missing-field-initializers, -Wno-dangling-reference, -Wno-maybe-uninitialized, -Wno-stringop-overflow, -Wno-array-bounds, -Wno-stringop-overread, -Wno-psabi`. `CLANG_FLAGS` adds `-Werror=absolute-value, c99-designator, inconsistent-missing-override, final-dtor-non-final-class, literal-conversion, non-virtual-dtor, range-loop-analysis, return-stack-address, sign-compare, unqualified-std-cast-call` and `-fno-assume-unique-vtables`. Test targets add `-Wno-sign-compare` (all) and, on GCC, `-Wno-missing-declarations -Wno-unused-parameter`.
- Errors vs warnings is a build setting: `//tools/cc_toolchain:error_severity` (default `warning`); `tools/bazel-developer.rc` sets it to `error` only when Drake is the main module; `_defang()` turns `-Werror=x` into `-Wx` otherwise, so downstream users of Drake as a dependency get warnings only. (A comment in `tools/cc_toolchain/BUILD.bazel` points to `tools/cc_toolchain/bazel.rc`, which does not exist at that path: stale comment.)
- Third-party code kept out: `tools/bazel-build.rc` has `build --strict_system_includes` and `build --per_file_copt=external/libx11./.*@-w` (same for libxcb); externals are compiled without the project macro's flags; `CPPLINT.cfg` has `exclude_files=third_party`; `tags = ["nolint"]` opts a target out of lint.
- Lint is Bazel tests tagged `lint`: `bazel test --config lint //...` (`build:lint --test_tag_filters=lint`). `tools/lint` has `cpplint.bzl`, `clang_format_lint.py`, `drakelint.py` (own checks: unguarded OpenMP, disallowed includes in headers, doxygen `@file` spelling, line endings, 80 columns, shebang, `<iostream>`, clang-format toggles, `experimental` namespace and isolation), `library_lint.bzl`, `bazel_lint.bzl` (buildifier), `ruff` for Python.
- `CPPLINT.cfg` turns off several cpplint categories (`build/c++11`, `build/c++17`, `build/header_guard` replaced by `build/pragma_once`, `build/include_order`, `build/include`, whitespace braces/indent/parens that clang-format covers, `legal/copyright`) and keeps `build/include_what_you_use`.
- `.clang-tidy` exists with `clang-analyzer-*, clang-diagnostic-*, cppcoreguidelines-*, google-*, modernize-*, performance-*, readability-*` minus 8 exclusions, but its first line says "This file is not used by CI and the checks included are not part of the Drake style guide". So Drake does NOT gate on clang-tidy.
- `.clang-format`: `BasedOnStyle: Google`, pointer left, short functions only inline, include categories (C system, C++ system, other libs, `"drake/..."`).
- Sanitizers (`tools/bazel-developer.rc`): configs `asan` (clang; adds LSan; timeouts 2.5x), `lsan`, `tsan` (OpenMP turned off), `ubsan` (disables `float-divide-by-zero,function,vptr`), `kcov`, `memcheck` and `fastmemcheck` (valgrind; `-O2`, `-DNDEBUG` but `-DDRAKE_ENABLE_ASSERTS`; gurobi and mosek off), each with scaled `--test_timeout`. `tools/dynamic_analysis/` holds only a BUILD and a `dump_limit_malloc_stacks` helper (the `bazel.rc` named in the task does not exist; the rc content is in `tools/bazel-developer.rc`).
- Language standard: `build --cxxopt=-std=c++23` (`tools/bazel-build.rc`).

### 4. Where tests live
- Each package has `test/` (`multibody/plant/test`, `math/test`, `common/test`, ...), files named `<unit>_test.cc`, mapped by the macro default `test/%s.cc`. Shared helpers: `common/test_utilities/` (listing: `eigen_matrix_compare.h, expect_throws_message.h, limit_malloc.h, drake_cc_googletest_main.cc, random_polynomial_matrix.h, symbolic_test_util.h, set_env.h, measure_execution.h ...`), per-area `test_utilities/` in geometry, multibody, solvers, planning, systems.
- Kinds: unit tests (size small default); long ones with `timeout`/`shard_count`; `benchmarking/` and `multibody/benchmarks/` directories (e.g. `common/benchmarking` holds `autodiff.cc, benchmark_polynomial.cc`; `geometry/BUILD` visibility lists `//geometry/benchmarking`); regression data kept as files next to tests, passed via `data = [...]` (e.g. `multibody/plant/test/box.sdf, box.vtk, block_on_halfspace.sdf, atlas_with_fixed_joints.urdf`) or externals (`@drake_models//:iiwa_description`). Tests that need display, network, rendering, threads are declared with `display`, `allow_network`, `rendering`, `num_threads` arguments; `opt_in_condition`/`opt_out_conditions` with `build_when_skipped` compile but skip a test.
- Determinism rule in the review checklist: no `Eigen::Random`, no libc `rand`; use local generator seeded with a hard-coded value, "This includes test code." `common/random.h` defines `RandomGenerator` with a `default_seed`.

### 5. Outside dependencies
- `MODULE.bazel` (468 lines) declares 32 `bazel_dep`s (grep count) at fixed versions (e.g. `rules_cc 0.2.27`, `bazel_skylib 1.9.2`; `bazel_compatibility >=8.7.0`) and module extensions for C++ libraries. `tools/workspace/README.md`: prefer the Bazel Central Registry; otherwise a module extension and a `repository.bzl` per external in `tools/workspace/<name>/` (the directory has 93 entries by the API), with `package.BUILD.bazel` for libraries without Bazel support, the name of internal-only externals ending `_internal`, licenses by SPDX id, a credits page entry; pins by `upgrade_type = "release" | "tag" | "commit"`; monthly upgrade with `bazel run //tools/workspace:new_release -- --lint --commit foo`. `tools/bazel-build.rc` sets `--module_mirrors=https://bcr.cloudflaremirrors.com`.
- Where an external comes from is behind an alias (`drake_dep_repositories`, e.g. `use_repo(drake_dep_repositories, "eigen")`), controlled by flags in `tools/flags/BUILD.bazel`.
- Eigen is NOT wrapped. `common/eigen_types.h` (listed in the `:essential` library) offers type aliases, and `-DEIGEN_NO_IO=1` forces formatting through `fmt_eigen`; but `@eigen` appears in a library's own deps only in a few places: common 2, math 4, geometry/proximity 1, multibody/plant 0, planning 0 (the rest get it through `//common:essential`); GitHub code search counted 21 BUILD files with the literal `@eigen`. For solvers, there is a facade: `solvers/solver_interface.h`, `mathematical_program.h`, `mathematical_program_result.h`; commercial solvers are optional (`with_gurobi`, `with_mosek`, `with_snopt` flags).

### 6. How decisions and style are recorded
- `doc/_pages/`: `directory_structure.md`, `code_style_guide.md` (defers to the Google-derived C++ guide kept in a separate repo `RobotLocomotion/styleguide`), `code_style_tools.md`, `code_review_checklist.md`, `developers.md`, `stable.md`, `platform_reviewer_checklist.md`, `issues.md`, `buildcop.md`, `unit_testing_instructions.md`. Decisions live in GitHub issues (the developers page asks for an issue and design feedback before big changes); I did not find a design-decision-record directory.
- Deprecation: `stable.md`: monthly releases 1.x.y, not SemVer; deprecations finalized "3+ minor versions later"; `[[deprecated]]`/`DRAKE_DEPRECATED` plus release notes; the `-Werror=deprecated` flags above keep Drake's own code from calling deprecated API.
- Review: two phases (feature, then "platform" review by one of 9 named owners), PR size limit (should be under 750 added or changed lines, must not exceed 1500, except data files, generated files, `dev/` directories, or two platform reviewers agree), build cop who reverts breakers. Experimental and dev directories lighten the rules.
- I did not find a monogram-notation or units page in the files I read; `multibody_plant.h` states units per quantity in comments (e.g. N and N m for effort limits) rather than in a policy page, so no claim is made about monogram notation.

### 7. Not to copy, and to copy
Do NOT copy:
1. One Bazel library per header plus a per-package rollup plus a linter that checks the rollup (`drake_cc_package_library`, `library_lint`). Drake pays for it in 51 to 64 library blocks per big file, a macro layer of 936 lines in `drake_cc.bzl` (install-header providers, scalar sharding, test-weight heuristics), and a rule that every library must be declared by the macro. For a repository with 105 targets in one package and one owner, the cost exceeds the gain; the useful piece is only the narrower `implementation_deps` / `internal` idea if the repo splits into packages.
2. The full Google-style apparatus: cpplint with 12+ filters, own `drakelint.py`, clang-format with Google style, and a two-reviewer process with named owners, PR size limits and a build cop. These serve many external contributors; here agents are the contributors and one owner reviews. Drake itself does not run clang-tidy in CI ("This file is not used by CI").
Copy in small:
1. A warning set as one list with a reason on each disabled flag, errors on for own targets only, warnings for externals (`-Werror=` list in `drake_cc.bzl` and `error_severity`); grep-checkable: a single `COPTS` list, no `-Wno-` without a comment.
2. A written dependency direction between top-level areas ("A may depend on B, not the reverse") together with a TODO-marked exception, as `directory_structure.md` and `math/BUILD.bazel` do; here it can be a check that lists forbidden `#include` or `deps` pairs.
3. Determinism rule (no `rand`, local seeded generator, also in tests) and tests named `<unit>_test.cc` with test-size defaults (small unless stated, long ones named); plus `experimental/` isolation (stable code must not depend on it), which fits the "stand-in cell" about to be replaced.

## Source blocks

### Drake drake_cc.bzl (macro layer and warning flags)
- Citation: RobotLocomotion/drake, "tools/skylark/drake_cc.bzl", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/skylark/drake_cc.bzl
- Kind: primary
- Read: 2026-11-25, in part (lines 1-330 and 543-936: flags, library, package library, test, googletest macros)
- Topics: B, C, D
- Claim 1: `CXX_FLAGS`, `GCC_FLAGS`, `CLANG_FLAGS` are lists of `-Werror=<name>` flags attached to every Drake C++ rule; GCC-only false positives are switched off by individual `-Wno-` lines each with a comment.
  Quote: "The CXX_FLAGS will be enabled for all C++ rules in the project building with any compiler."
- Claim 2: `drake_cc_library` accepts `implementation_deps`, `internal = True` (needs a non-public visibility, excludes headers from install) and `install_hdrs_exclude`; `drake_cc_package_library` must be named after its directory, must set visibility, and is forbidden in `examples/`.
  Quote: "Setting `internal = True` is convenient sugar to flag code that is never used by Drake's installed headers."
- Claim 3: `drake_cc_test` defaults to `size = "small"` and `srcs = ["test/<name>.cc"]`, forces `testonly`; `drake_cc_googletest` adds a shared gtest main.
- Bears on this repository: a single copts list with commented exceptions and a "small unless stated" test default can be copied; the install and scalar-sharding machinery cannot.

### Drake common/BUILD.bazel
- Citation: RobotLocomotion/drake, "common/BUILD.bazel", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/common/BUILD.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1-120 and grep over whole file, 1325 lines)
- Topics: A, B
- Claim 1: the package default is public; narrower visibility is given per target (for example `["//common/test_utilities:__pkg__"]`, `["//tools/install/libdrake:__pkg__"]`, `["//visibility:private"]`); the file has 51 `drake_cc_library`, 63 `drake_cc_googletest`, 9 `implementation_deps`, 2 `internal = True`.
- Claim 2: one rollup `drake_cc_package_library(name = "common", ...)` lists about 45 library targets, and `:essential` is a deliberately small library for "EVERYONE".
  Quote: "A library of things that EVERYONE should want and MUST EAT."
- Bears on this repository: shows what one-library-per-header looks like at scale (1325 lines for one directory); a 105-target single package should split by directory before it adopts per-header libraries.

### Drake math/BUILD.bazel
- Citation: RobotLocomotion/drake, "math/BUILD.bazel", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/math/BUILD.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1-30, 95-120, 355-370 and counts)
- Topics: A, B
- Claim 1: `math` has 22 libraries and 33 tests; two libraries depend on `//systems/primitives:linear_system_internal`, with a TODO to move the code to remove "the circular dependency", so the stated layering has known exceptions.
  Quote: "Move this code into //math to avoid the circular dependency."
- Claim 2: `math` has `@eigen` as a direct dep of 4 targets and uses `implementation_deps` in 2.
- Bears on this repository: a dependency-check rule should allow a short, named exception list with a TODO, not forbid all violations at once.

### Drake multibody/plant/BUILD.bazel
- Citation: RobotLocomotion/drake, "multibody/plant/BUILD.bazel", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/multibody/plant/BUILD.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1-30, 210-250, 415-430, 860-1170 and counts)
- Topics: B, D
- Claim 1: 24 libraries and 76 tests; tests state `timeout = "moderate"` (7 times) or `"long"` (1) and `shard_count` 2 or 3 only when needed; test data are listed via `data = [...]` (SDF/VTK files in `test/`, and `@drake_models//:...`).
- Claim 2: a test helper library is tagged out of the package rollup with a comment ("Don't add this library into the ':plant' package library").
- Bears on this repository: golden or model files next to the test and named in `data`, with size stated only for exceptions, is checkable and small.

### Drake planning/BUILD.bazel
- Citation: RobotLocomotion/drake, "planning/BUILD.bazel", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/planning/BUILD.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1-60 and counts; 412 lines)
- Topics: A, B
- Claim 1: a mid-size package: 18 libraries, 14 tests, one rollup `//planning` of 16 deps, 4 `implementation_deps`, no `internal`, no `@eigen`; each library depends on a few siblings plus `//common:essential`, `//geometry`, `//multibody/plant`.
- Claim 2: it ends with `add_lint_tests()` like every package.
- Bears on this repository: a planner package that depends only downward (common, geometry, multibody) is the shape a `planning/` split of the repo's flat `src/` would have.

### Drake geometry/proximity/BUILD.bazel
- Citation: RobotLocomotion/drake, "geometry/proximity/BUILD.bazel", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/geometry/proximity/BUILD.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1-40, 425-440, 1550-1565 and counts; 1903 lines)
- Topics: B
- Claim 1: the largest file read: 64 libraries, 72 tests, 11 `internal = True`, 14 `implementation_deps`, one `shard_count`; `internal` and `implementation_deps` are used most where the code is a hidden implementation of `//geometry`.
- Claim 2: a visibility exception lists `//geometry/benchmarking:__pkg__`, so benchmarks reach into the package by name.
- Bears on this repository: `internal` plus `implementation_deps` only pay off in a package with a public interface; here they would apply once `src/` gets directories.

### Drake directory_structure.md
- Citation: RobotLocomotion/drake, "Directory Structure", doc/_pages/directory_structure.md, master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/doc/_pages/directory_structure.md
- Kind: primary
- Read: 2026-11-25, whole
- Topics: A, F
- Claim 1: the dependency direction is stated in prose with one example: `systems` uses `solvers`, so `solvers` must not use `systems`; `examples` come after core; `test` directories are last.
  Quote: "some code in ``drake/systems`` depends on code in ``drake/solvers``, so code in ``drake/solvers`` should *not* depend on ``drake/systems``."
- Claim 2: `experimental` and `dev` directories have lighter standards, own BUILD file inside the directory, may be depended on only by other experimental or dev code, and need `experimental` in the C++ namespace; the build cop may disable their targets.
- Bears on this repository: write the layer order and the "stand-in cell may not be depended on by stable code" rule as a command (grep over `#include` and `deps`).

### Drake .clang-tidy
- Citation: RobotLocomotion/drake, ".clang-tidy", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/.clang-tidy
- Kind: primary
- Read: 2026-11-25, whole (26 lines)
- Topics: C
- Claim 1: the check list is `clang-analyzer-*, clang-diagnostic-*, cppcoreguidelines-*, google-*, modernize-*, performance-*, readability-*` minus eight exclusions, with five naming options (CamelCase classes, lower_case variables, trailing `_` private members).
- Claim 2: the file says it is not enforced.
  Quote: "This file is not used by CI and the checks included are not part of the Drake style guide"
- Bears on this repository: even Drake runs clang-tidy only by hand; a rule here should name a small tidy check set run in CI, not copy this broad list.

### Drake CPPLINT.cfg
- Citation: RobotLocomotion/drake, "CPPLINT.cfg", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/CPPLINT.cfg
- Kind: primary
- Read: 2026-11-25, whole (58 lines)
- Topics: C
- Claim 1: cpplint is configured by turning off categories that clash with the project (C++11/17 warnings, header-guard replaced by `#pragma once`, include order, whitespace categories covered by clang-format, copyright) and keeping `build/include_what_you_use`.
- Claim 2: third-party and docs are excluded.
  Quote: "exclude_files=third_party"
- Bears on this repository: cpplint on top of clang-format and clang-tidy adds a third tool; the repository should not adopt it.

### Drake tools/bazel-developer.rc
- Citation: RobotLocomotion/drake, "tools/bazel-developer.rc", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/bazel-developer.rc
- Kind: primary
- Read: 2026-11-25, in part (lines 1-175: error severity, lint, kcov, asan, lsan, tsan, ubsan, memcheck configs)
- Topics: C
- Claim 1: `-Werror` for Drake's own targets is switched on by one flag, only when Drake is the main module.
  Quote: "we enable -Werror by default for Drake's *own* targets, but not for our externals."
- Claim 2: sanitizer configs are named build configs (`asan`, `lsan`, `tsan`, `ubsan`, `memcheck`, `kcov`) each with a multiplied test timeout (asan 2.5x, tsan 5x, memcheck much longer); asan and lsan are marked clang-only; `lint` runs only tests tagged `lint`.
- Bears on this repository: with GCC only, `asan`+`ubsan` configs through GCC with scaled timeouts, and a `--config lint` that runs only lint-tagged targets, are small to copy; valgrind and kcov are not.

### Drake tools/bazel-build.rc
- Citation: RobotLocomotion/drake, "tools/bazel-build.rc", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/bazel-build.rc
- Kind: primary
- Read: 2026-11-25, in part (first 200 lines)
- Topics: C, E
- Claim 1: the default build is `-c opt` with `-std=c++23`, `--strict_system_includes` (external headers are system headers so their warnings are hidden) and `--per_file_copt=external/libx11./.*@-w` for two externals.
- Claim 2: `--module_mirrors=https://bcr.cloudflaremirrors.com` and `--incompatible_autoload_externally=` (all rules loaded explicitly in `MODULE.bazel`); test environment pinned (`LCM_DEFAULT_URL=memq://`, `MPLBACKEND=Template`).
- Bears on this repository: `--strict_system_includes` plus `-isystem` style for MuJoCo, OMPL, Boost keeps their warnings away from the project's `-Werror`; check with one grep of `.bazelrc`.

### Drake tools/lint/library_lint.bzl and lint.bzl
- Citation: RobotLocomotion/drake, "tools/lint/library_lint.bzl" and "tools/lint/lint.bzl", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/lint/library_lint.bzl and https://github.com/RobotLocomotion/drake/blob/master/tools/lint/lint.bzl
- Kind: primary
- Read: 2026-11-25, in part (library_lint first 80 lines; lint.bzl first 80 lines)
- Topics: B, C
- Claim 1: `add_lint_tests()` is called at the end of every BUILD file and adds cpplint, python lint, bazel lint and library lint tests for all rules in that file.
- Claim 2: `library_lint` computes the exact deps the package library must have and reports missing and extra ones as a test failure; a target tagged `nolint` is skipped.
- Bears on this repository: the idea of a lint test that fails when a rollup list and the real targets diverge is only worth it with a rollup; the one-call-per-BUILD-file pattern is a cheap way to attach checks.

### Drake tools/lint/drakelint.py
- Citation: RobotLocomotion/drake, "tools/lint/drakelint.py", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/lint/drakelint.py
- Kind: primary
- Read: 2026-11-25, in part (the list of check functions and `main()`)
- Topics: C, F
- Claim 1: project-specific checks in plain Python: unguarded OpenMP, disallowed includes in headers, 80 columns, `<iostream>` use, shebangs, line endings, clang-format toggles, `experimental` namespace and isolation. Each is a function returning an error count.
- Claim 2: a line can opt out with `// drakelint: ignore`.
- Bears on this repository: a short script of project rules (for example "no `<iostream>` in `src/`", "control code includes no sensor-writing header") fits an agent repo better than cpplint.

### Drake tools/workspace/README.md
- Citation: RobotLocomotion/drake, "tools/workspace/README.md", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/workspace/README.md
- Kind: primary
- Read: 2026-11-25, in part (lines 1-130 and 355-414)
- Topics: E
- Claim 1: externals are Bazel modules from the Central Registry where possible, otherwise module extensions with one `tools/workspace/<name>/` directory each; pins follow `upgrade_type` release, tag or commit; upgrades are a monthly run of `new_release`.
  Quote: "When the Bazel Central Registry (https://registry.bazel.build/) contains the module we need, we prefer to use that mechanism."
- Claim 2: licences are written as SPDX identifiers; externals not covered by the stability promise are named `foo_internal`.
- Bears on this repository: a rule "every external is listed once in `MODULE.bazel` with an exact version; no `latest`" is checkable; the monthly tool is too heavy, a Renovate-style bot is the small version (Drake has `.github/renovate.json`).

### Drake MODULE.bazel
- Citation: RobotLocomotion/drake, "MODULE.bazel", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/MODULE.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1-60 and grep of `bazel_dep` count)
- Topics: E
- Claim 1: rule sets are fixed `bazel_dep` versions; a patch is applied to `rules_python` by `single_version_override`; the module requires Bazel `>=8.7.0`.
- Claim 2: comments such as "Keep this in sync with cmake/MODULE.bazel.in" show the same pin recorded in two files, kept by hand.
- Bears on this repository: duplicate pins are a drift risk; a check that each version appears once is worth one line.

### Drake developers.md, code_review_checklist.md, stable.md
- Citation: RobotLocomotion/drake, "For Developers", "Code Review Checklist", "Drake Stability Guidelines", doc/_pages/developers.md, code_review_checklist.md, stable.md, master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/doc/_pages/developers.md , https://github.com/RobotLocomotion/drake/blob/master/doc/_pages/code_review_checklist.md , https://github.com/RobotLocomotion/drake/blob/master/doc/_pages/stable.md
- Kind: primary
- Read: 2026-11-25, in part (developers.md lines 1-263 outside IDE lists; checklist lines 25-117; stable.md lines 1-60)
- Topics: F
- Claim 1: two-phase review (feature then platform by one of nine named owners), PR size under 750 lines normally and 1500 at most, issue first for complex changes; the platform reviewer checks "architectural compatibility, stability, performance, test coverage, and style".
- Claim 2: determinism rule for code and tests.
  Quote: "Do not use ``Eigen::Random``, ``libc rand``, or anything like it."
- Claim 3: deprecation is a rolling window of 3+ minor versions, announced in release notes plus `[[deprecated]]`, with no SemVer.
- Bears on this repository: the PR size cap and the determinism rule can be turned into checks for agents; the owner list and release-note deprecation window do not fit a one-owner repository without releases.

### Drake common/random.h
- Citation: RobotLocomotion/drake, "common/random.h", master @ 5186757f. https://github.com/RobotLocomotion/drake/blob/master/common/random.h
- Kind: primary
- Read: 2026-11-25, in part (first 60 lines)
- Topics: F
- Claim 1: `RandomGenerator` wraps `std::mt19937`, is copyable, has a public `default_seed`, and a constructor taking a seed; engine creation is lazy.
- Bears on this repository: a rule "no `rand()`, no `std::random_device` in `src/`; planners take a seeded generator by argument" is grep-checkable.

## Does not fit
- Drake's "one `drake_cc_library` per header with a package rollup" (`tools/skylark/drake_cc.bzl`, `common/BUILD.bazel`): needs a macro layer of 936 lines, a lint test for the rollup, and gives 51 to 64 targets per directory file; the repository's 105 targets in one package with no visibility are better served by directories first.
- Drake's two-phase human review with named platform owners and a build cop (`developers.md`): built for a team of nine owners plus outside contributors; here one owner reviews agent PRs.
- Drake's cpplint + `drakelint.py` + Google style guide fork (`code_style_guide.md`, `CPPLINT.cfg`): three style tools, and its own `.clang-tidy` is declared not part of CI; for this repository clang-format plus a short clang-tidy set is enough.
- Monthly `new_release` upgrade tool, CMake wrapper, install rules, Python bindings (`tools/workspace/README.md`): serve downstream users of a library; this repository is an application.
- Valgrind, kcov, Clang-only sanitizer configs (`tools/bazel-developer.rc`): the repository is GCC only; only GCC `asan` and `ubsan` carry over.

## Leads not followed
- `https://drake.mit.edu/styleguide/cppguide.html` (the C++ style guide fork in `RobotLocomotion/styleguide`), cited by `code_style_guide.md`.
- `doc/_pages/issues.md`, `buildcop.md`, `bazel.md`, `reviewable.md`: listed in the `doc/_pages` directory, not read.
- `tools/skylark/README.md` (test options such as `allow_network`, `display`), cited by `drake_cc.bzl`.
- `tools/lint/cpplint.bzl`, `tools/install/`, `tools/workspace/new_release.py`: seen, not read in full.
- A monogram-notation page and a units policy: not found in the files read, so nothing is claimed.
- Not found: `tools/cc_toolchain/bazel.rc` and `tools/dynamic_analysis/bazel.rc` (HTTP 404; the content they are named for is in `tools/bazel-developer.rc` and `tools/bazel-build.rc`); `.github/CODEOWNERS` (HTTP 404).
