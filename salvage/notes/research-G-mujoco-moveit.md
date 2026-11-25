# Research G: MuJoCo and MoveIt 2 read in source (and OMPL briefly)

Commits read (all fetched 2026-11-25, branch main / main):
- google-deepmind/mujoco main: 72a06685bffb8f3a39af0a1573fab1521dc94693
- moveit/moveit2 main: 8c99e523bc23149d24a61990c64d53c9c7d2905f
- ompl/ompl main: 340bd214be6e87fe758a75e04203d665429b020f

Topic letters used in the blocks: 1 tree division and dependencies; 2 build targets and visibility; 3 tests; 4 warnings and analysis; 5 decisions and style records; 6 units, frames, determinism; 7 copy / do not copy.

---------------------------------------------------------------------------
# PART 1: MuJoCo (all file URLs on branch main)

## Section answers (every statement names the file it was read in)

1. Tree division.
   - Root listing (gh api contents): `.bazelrc BUILD.bazel CMakeLists.txt MODULE.bazel STYLEGUIDE.md CONTRIBUTING.md VERSIONING.md cmake doc include mjx model plugin python sample simulate src studio test unity wasm`.
   - `include/mujoco/` holds only public headers: `mjdata.h mjmodel.h mjexport.h mjmacro.h mjplugin.h mjrender.h mjspec.h mjtype.h mjui.h mjvisualize.h mjxmacro.h mujoco.h mjsan.h mjassert.h mjbatch.h` plus `experimental/` (listing of include/mujoco).
   - `src/` holds `batch cc engine experimental render ui user xml` (listing of src). `src/engine` is the simulator core, flat, with file-name prefixes as the only grouping: `engine_core_*` (constraint, smooth, util), `engine_collision_*` (box, convex, gjk, primitive, sdf, driver ...), `engine_util_*` (blas, errmem, misc, solve, sparse, spatial), `engine_vis_*`, `engine_derivative*`, `engine_forward`, `engine_inverse`, `engine_io`, `engine_plugin`, `engine_sensor`, `engine_solver` (listing of src/engine). Each pair is `.c` + `.h`.
   - `src/user` is the model compiler (`user_model.cc`, `user_objects.cc`, `user_mesh.cc`, `user_api.cc`); `src/xml` is the MJCF/URDF reader and writer (`xml_native_reader.cc`, `xml_urdf.cc`, `mjcf.schema`); `src/render/{classic,filament,noop}` are renderers; `src/ui` is a small immediate-mode UI (`ui_main.c`). `src/thirdparty` does NOT exist in the tree (404 from the contents API); third-party code is fetched as dependencies (`ccd lodepng qhull tinyobjloader tinyxml2` appear in `target_link_libraries(mujoco PRIVATE ...)` in CMakeLists.txt and as `bazel_dep` lines in MODULE.bazel).
   - `plugin/` holds plugins (actuator, sensor, sdf, decoders) each a separate directory with its own build file (CMakeLists.txt add_subdirectory lines); `simulate/` is the GUI viewer app; `python/` the Python bindings and notebooks; `mjx/` the JAX port (only README, pyproject, notebooks and a `mujoco` dir); `test/` mirrors `src/` (`test/engine test/user test/xml test/render test/plugin test/benchmark test/testdata`).
   - What depends on what: `src/engine/BUILD.bazel` gives one `cc_library` per `.c` file (e.g. `engine_core_util`, `engine_forward`), and the `deps` lists show the direction (the `engine` target depends on `engine_core_util`, `engine_forward`, `engine_io` ...). Higher layers (`src/user`, `src/xml`, `src/batch`, `src/render/classic`, `test`) are allowed to see engine internals; this is written as a `package_group` (see below).
2. Build targets and visibility.
   - CMake: one library target `mujoco` (SHARED by default: `add_library(mujoco SHARED ...)`), sources added by `add_subdirectory(src/engine)`, `src/user`, `src/xml`, `src/batch`, `src/render/...`, `src/ui` (root CMakeLists.txt). Third-party libs are linked `PRIVATE`. `CMAKE_C_VISIBILITY_PRESET hidden` and `CMAKE_CXX_VISIBILITY_PRESET hidden` are set in cmake/MujocoOptions.cmake, so only symbols marked `MJAPI` are exported.
   - `MJAPI` is defined in include/mujoco/mjexport.h: dllexport or `visibility("default")` when building the library (`MUJOCO_DLL_EXPORTS`), dllimport otherwise; `MJLOCAL` is hidden.
   - Bazel exists in parallel: root BUILD.bazel has `package(default_visibility = ["//visibility:private"])`, a `cc_library(name="mujoco")` that depends on private `:_core` and others, and a `cc_shared_library` `mujoco_shared` with a linker version script (`mujoco.lds`): "only MuJoCo's C API is exported ... Engine internals and the bundled third-party code stay hidden" (comment in BUILD.bazel). `src/engine/BUILD.bazel` sets `package(default_visibility = [":_impl_visibility"])` where `_impl_visibility` is a `package_group` of exactly `//src/batch/...`, `//src/render/classic/...`, `//src/user/...`, `//src/xml/...`, `//test/...`; only the aggregate `engine` target has `visibility = [":_api_visibility"]` (`//...`). `MODULE.bazel` is a bzlmod module "mujoco" version 4.0.2 with `bazel_compatibility >=8.0.0`, so it is in the Bazel Central Registry style. A test (`test/mujoco_shared_exports_test.sh`, listed in test/) checks the exports; BUILD.bazel says "the exports test checks that".
3. Tests.
   - `test/` mirrors `src/`: `test/engine/engine_forward_test.cc` for `src/engine/engine_forward.c`, one test file per engine file (listing of test/engine). Kinds seen: unit tests per file, fuzz (`engine_collision_box_fuzz_test.cc`), convergence (`engine_cg_convergence_test.cc`), header test (`test/header_test.cc`), plugin tests, exports shell test, benchmarks in `test/benchmark/` (`step_benchmark_test.cc`, `ccd_benchmark_test.cc` ...), test models in `test/testdata` and `test/engine/testdata/*.xml`.
   - Models are mostly small XML strings or files loaded by `LoadModelFromString` (test/engine/engine_forward_test.cc). Outputs are compared with explicit tolerances per precision: `EXPECT_NEAR(data->act[0], 1, MjTol(0, 5e-6))` (engine_forward_test.cc). test/fixture.h defines `MjTol(double_tol, float_tol)` and the env var `MJTOL_SCALE`; tolerances are scaled by it; setting 0 shows the exact residual, and the comment says to set the tolerance to about 10 times that residual. CONTRIBUTING.md requires all tests to pass in double and single precision. I did not find stored golden output files in test/engine (only input models).
   - test/CMakeLists.txt defines a `mujoco_test` macro: each test links `mujoco fixture gmock gtest_main`, and adds `-ffp-contract=off` for GNU/Clang.
4. Warnings and analysis, exactly.
   - cmake/MujocoOptions.cmake, for GNU or Clang: `-Werror -Wall -Wpedantic -Wimplicit-fallthrough -Wunused -Wvla -Wno-int-in-bool-context -Wno-sign-compare -Wno-unknown-pragmas`; for GNU additionally `-Wimplicit-fallthrough=5 -Wno-maybe-uninitialized -Wno-psabi`. In the same file: C11 and C++20 standards required, `CMAKE_CXX_EXTENSIONS OFF`, `CMAKE_EXPORT_COMPILE_COMMANDS ON` ("For LLVM tooling"), LTO on by default for non-Debug, `-ffp-contract=off -fno-math-errno` on the target (root CMakeLists.txt).
   - src/engine/BUILD.bazel (Bazel build) uses a stricter C list `_C_CONLYOPTS`: `-std=c11 -Werror -Wall -Wpedantic -Wstrict-prototypes -Wshadow -Wunused -Wvla -Wimplicit-fallthrough` and `-ffp-contract=off -fvisibility=hidden`.
   - cmake/MujocoHarden.cmake: option `MUJOCO_HARDEN` (default OFF, Clang only): `-D_FORTIFY_SOURCE=2 -fstack-protector`, linker `-Wl,-z,relro -Wl,-z,now`.
   - No sanitizer switch found in CMakeLists.txt or MujocoOptions.cmake (grep for "sanitize" in the root CMakeLists.txt: no hit). include/mujoco/mjsan.h has code that cooperates with AddressSanitizer (poisoning of the arena stack), so ASan is supported but is not a CMake option I found. No clang-tidy config found at the repository root (root listing has `.clang-format` only). `.pre-commit-config.yaml` runs `trailing-whitespace`, `end-of-file-fixer` and clang-format (rev v23.1.0) on C/C++; `.github/workflows/lint.yml` runs pre-commit on files changed in the pull request.
5. Decisions and style.
   - STYLEGUIDE.md (189 lines): "Maximise consistency with the rest of the code." Rules: 2-space indent, 100 columns, short names, one-line lowercase comments before blocks, K&R braces. If guide and code disagree, the guide wins. CONTRIBUTING.md: tests required; "Resolve compiler warnings."; small pull requests; tests in both precisions; get in touch before large work. VERSIONING.md: custom semantic versioning from 3.5.0 (`SUPERMAJOR.MAJOR.MINOR_OR_PATCH`), ABI-only breaks listed in the changelog. `doc/changelog.rst` has an "Upcoming Version" section with a "Breaking API changes" admonition and a "Migration:" line per break. No ADR directory seen in the root listing.
6. Units, frames, determinism.
   - doc/overview.rst: "Units are unspecified" (MuJoCo asks only for consistency, recommends MKS; angles in degrees by default in MJCF is not something I read, so not claimed). `mjData` (include/mujoco/mjdata.h) is the workspace; doc/programming/simulation.rst: "mjData is the workspace where all computations are performed" and is specific to one `mjModel`; `mjModel` is the constant description. API functions "perform minimal argument checking" (undefined behavior on mismatch).
   - Determinism (docs computation/index, "Reproducibility"): the pipeline is deterministic; exact reproducibility only within one version on the same architecture; differs across versions and operating systems.
   - Naming (doc/programming/index.rst): all API symbols start with `mj`; prefix families `mjt` (types), `mju_` (utilities), `mj_` (functions): namespaces by prefix in a C API.
7. Do not copy / copy.
   - Do NOT copy: very large single-file units with prefix-named globals and C structs in place of modules. Evidence: `src/user/user_objects.cc` is 10,439 lines, `src/engine/engine_forward.c` 2,790, `engine_core_smooth.c` 2,648 (line counts from raw files). A 40,000-line repository whose contributors are LLMs gains from small files that fit a context window; MuJoCo's size is tolerable only because it has a human-learned naming convention (STYLEGUIDE.md "Keep names short", doc naming prefixes) and a hand-kept public API.
   - Also do NOT copy: a hand-maintained export script and parallel CMake plus Bazel builds with two sets of flags (MujocoOptions.cmake against src/engine/BUILD.bazel list different warnings: `-Wshadow -Wstrict-prototypes` only in Bazel); one build system is enough here.
   - COPY 1: the public/private split enforced by the build tool, not by habit: `package_group(name="_impl_visibility", packages=[...])` and `default_visibility = [":_impl_visibility"]` in src/engine/BUILD.bazel. A rule checkable with `bazel query`: no target outside the named packages depends on a private target. This directly answers "no `visibility`" in the target repository.
   - COPY 2: one mutable state object (`mjData`, with `mjModel` constant and passed as `const mjModel*`) and tolerance helper with a calibration switch (`MjTol`, `MJTOL_SCALE` in test/fixture.h). Also a "Migration:" line for each break in the changelog.
   - COPY 3 (small): "Resolve compiler warnings" plus `-Werror` set in the build file, with warnings listed once (MujocoOptions.cmake).

---------------------------------------------------------------------------
## MuJoCo source blocks

### MuJoCo root CMakeLists.txt
- Citation: google-deepmind/mujoco, "CMakeLists.txt", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/CMakeLists.txt
- Kind: primary
- Read: 2026-11-25, in part (target definitions, add_subdirectory lines, link lines)
- Topics: 1, 2, 3
- Claim 1: One library target `mujoco` (shared by default) is assembled from `add_subdirectory(src/engine)`, `src/user`, `src/xml`, `src/batch`, `src/render/*`, `src/ui`; plugins and `simulate`, `sample`, `test` are separate subdirectories; tests are enabled by `enable_testing()` then `add_subdirectory(test)`.
- Claim 2: Third-party libraries (ccd, lodepng, qhullstatic_r, tinyobjloader, tinyxml2) are linked `PRIVATE` to `mujoco`, so users of the library do not see them.
- Bears on this repository: a rule "the program target names each third-party dependency once and no header outside the dependency's wrapper includes it" can be checked with a grep over `#include`.

### MuJoCo cmake/MujocoOptions.cmake
- Citation: google-deepmind/mujoco, "cmake/MujocoOptions.cmake", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/cmake/MujocoOptions.cmake
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 2, 4
- Claim 1: Warnings for GNU and Clang are `-Werror -Wall -Wpedantic -Wimplicit-fallthrough -Wunused -Wvla` with three explicit opt-outs (`-Wno-int-in-bool-context -Wno-sign-compare -Wno-unknown-pragmas`); GCC adds `-Wimplicit-fallthrough=5 -Wno-maybe-uninitialized -Wno-psabi`.
  Quote: "-Werror"
- Claim 2: Symbol visibility is hidden for C and C++ by default and inlines are hidden; `CMAKE_EXPORT_COMPILE_COMMANDS` is on "For LLVM tooling"; C++20 with extensions off.
- Bears on this repository: GCC-only repo can adopt the same short list with `-Werror` and list each opt-out with a reason; the check is the build.

### MuJoCo src/engine/BUILD.bazel
- Citation: google-deepmind/mujoco, "src/engine/BUILD.bazel", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/src/engine/BUILD.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1 to about 120 and a scan of the rest)
- Topics: 1, 2, 4
- Claim 1: Visibility is closed by default: `package(default_visibility = [":_impl_visibility"])` where `_impl_visibility` is a `package_group` of `//src/batch/...`, `//src/render/classic/...`, `//src/user/...`, `//src/xml/...`, `//test/...`; the public aggregate `engine` target alone is visible to `//...`.
- Claim 2: There is one `cc_library` per `.c` file and `deps` name the other engine files by prefix (`engine_core_util`, `engine_forward`, `engine_io`...), so the layering is visible in the deps. The C copts are `-Werror -Wall -Wpedantic -Wstrict-prototypes -Wshadow -Wunused -Wvla -Wimplicit-fallthrough` and `-ffp-contract=off`.
  Quote: "package(default_visibility = [\":_impl_visibility\"])"
- Bears on this repository: the target repository (one BUILD.bazel, 105 targets, no visibility) could add a `package_group` per layer; a command (`bazel query 'visible(...)'` or a failed build) enforces it.

### MuJoCo root BUILD.bazel
- Citation: google-deepmind/mujoco, "BUILD.bazel", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/BUILD.bazel
- Kind: primary
- Read: 2026-11-25, in part (lines 1 to 140)
- Topics: 2
- Claim 1: Root package is private by default (`default_visibility = ["//visibility:private"]`); only `:mujoco`, `:graphics` and aliases are public through a `package_group`. A `cc_shared_library` with a linker version script exports only the C API.
  Quote: "Engine internals and the bundled third-party code stay hidden."
- Claim 2: There is a `single_precision` `bool_flag` that switches `mjtNum` to float for the whole build; tests must pass in both.
- Bears on this repository: keeps the case for a closed-by-default visibility with a few public aliases, instead of 105 targets all visible.

### MuJoCo include/mujoco/mjexport.h
- Citation: google-deepmind/mujoco, "include/mujoco/mjexport.h", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjexport.h
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 2
- Claim 1: `MJAPI` expands to `visibility("default")` (GCC 4 and up) or `__declspec(dllexport/dllimport)`; `MJLOCAL` is hidden; `MJ_STATIC` makes both empty. Everything not marked is private when the build sets hidden visibility.
  Quote: "#define MJAPI MUJOCO_HELPER_DLL_EXPORT"
- Bears on this repository: the export macro is not needed for a Bazel static build; the equivalent rule is a closed `visibility`.

### MuJoCo STYLEGUIDE.md
- Citation: google-deepmind/mujoco, "MuJoCo Style Guide (STYLEGUIDE.md)", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/STYLEGUIDE.md
- Kind: primary
- Read: 2026-11-25, in part (first 120 of 189 lines)
- Topics: 5, 7
- Claim 1: The single principle for anything unspecified is consistency with existing code, and if guide and code contradict, "the guide takes precedence"; a pattern found in code but not in the guide is to be added to the guide.
  Quote: "Maximise consistency with the rest of the code."
- Claim 2: Concrete checkable limits: 2-space indent, 100-column lines, short names, one-line comments that let a reader "get a sense of what is happening in a function just by reading the comments"; public header comments are required docstrings.
- Bears on this repository: AGENTS.md style rules should name one mechanical check (clang-format) and put the rest in a short guide that wins over existing code.

### MuJoCo CONTRIBUTING.md
- Citation: google-deepmind/mujoco, "Contributing to MuJoCo", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/CONTRIBUTING.md
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 3, 4, 5
- Claim 1: A pull request is accepted "in most cases" only with tests; touching an untested function obliges the contributor to add tests for the existing behavior too; warnings must be resolved; all tests must pass in double and single precision using the tolerance helpers in `test/fixture.h`.
  Quote: "Resolve compiler warnings."
- Claim 2: Pull requests must be small and a large one gets split into self-contained ones.
- Bears on this repository: "a changed function has a test that fails when the change is reverted" and "one pull request, one purpose" are questions an agent can answer.

### MuJoCo VERSIONING.md
- Citation: google-deepmind/mujoco, "MuJoCo Versioning", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/VERSIONING.md
- Kind: primary
- Read: 2026-11-25, in part (first 60 of 75 lines)
- Topics: 5, 6
- Claim 1: Because of its large API surface, nearly every release would be MAJOR under strict semver, so since 3.5.0 it uses SUPERMAJOR.MAJOR.MINOR_OR_PATCH; MINOR_OR_PATCH keeps API compatibility; ABI-only breaks are listed in the changelog.
- Claim 2: "numerical reproducibility is *never* guaranteed" across versions, although the simulation is deterministic.
  Quote: "numerical reproducibility is *never* guaranteed"
- Bears on this repository: for a one-owner repository with no external users, versioning rules do not fit; the part that fits is a changelog line per behavior change.

### MuJoCo test/fixture.h and test/engine/engine_forward_test.cc
- Citation: google-deepmind/mujoco, "test/fixture.h" and "test/engine/engine_forward_test.cc", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/test/fixture.h and https://github.com/google-deepmind/mujoco/blob/main/test/engine/engine_forward_test.cc
- Kind: primary
- Read: 2026-11-25, in part (comment block and helpers in fixture.h; grep of tolerance uses and model loading in the test)
- Topics: 3, 6
- Claim 1: Tolerances are explicit per precision, `MjTol(double_tol, float_tol)`, scaled by the env var `MJTOL_SCALE`; the documented calibration is to run with `MJTOL_SCALE=0` to see the residual and set the tolerance about ten times above it.
  Quote: "Setting MJTOL_SCALE=0 scales tolerances to zero"
- Claim 2: Tests build small models from XML strings (`LoadModelFromString`) or from `testdata/*.xml` and assert with `EXPECT_NEAR(..., MjTol(0, 5e-6))`; I found inputs but no stored expected-output files.
- Bears on this repository: a test-tolerance helper with one calibration command would replace ad hoc literals in the "gate" and unit tests.

### MuJoCo documentation: Reproducibility (computation/index)
- Citation: MuJoCo documentation, "Computation: Reproducibility", rendered from main. https://mujoco.readthedocs.io/en/latest/computation/index.html
- Kind: primary
- Read: 2026-11-25, in part (the Reproducibility paragraph)
- Topics: 6
- Claim 1: "MuJoCo's simulation pipeline is entirely deterministic and reproducible" given saved state (including warmstart accelerations); exact reproducibility holds only within one version on one architecture.
  Quote: "Exact reproducibility is only guaranteed within a single version, on the same architecture."
- Claim 2: Contact events amplify any tiny numerical difference, so cross-version trajectory comparison cannot use tight tolerances.
- Bears on this repository: golden CSV outputs from the gate test must be pinned to the MuJoCo 4.0.0 version and compared with a tolerance, and re-baselined when the version changes (a stated rule).

### MuJoCo doc/overview.rst (Units) and doc/programming
- Citation: google-deepmind/mujoco, "doc/overview.rst" (section "Units are unspecified") and "doc/programming/index.rst" (Naming convention) and "doc/programming/simulation.rst". https://github.com/google-deepmind/mujoco/blob/main/doc/overview.rst , https://github.com/google-deepmind/mujoco/blob/main/doc/programming/index.rst , https://github.com/google-deepmind/mujoco/blob/main/doc/programming/simulation.rst
- Kind: primary
- Read: 2026-11-25, in part (the named sections)
- Topics: 1, 6
- Claim 1: MuJoCo does not enforce units; the user must be consistent and is encouraged to use MKS.
  Quote: "MuJoCo does not specify basic physical units."
- Claim 2: `mjData` is "the workspace where all computations are performed" and belongs to one `mjModel`; API symbols are namespaced by prefix (`mj`, `mjt`, `mju_`).
- Bears on this repository: since MuJoCo itself does not enforce units, the repository's own convention (SI, radians) must be written in AGENTS.md and checked by naming (suffix `_m`, `_rad`) or tests, not by the library.

### MuJoCo include/mujoco/mjdata.h and mjsan.h
- Citation: google-deepmind/mujoco, "include/mujoco/mjdata.h", "include/mujoco/mjsan.h", main @72a0668. https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h , https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjsan.h
- Kind: primary
- Read: 2026-11-25, in part (struct head and typedefs of mjdata.h; the macro block of mjsan.h)
- Topics: 6, 4
- Claim 1: `mjData` is one C struct holding stack and arena pointers and all per-step state; callbacks take `const mjModel* m, mjData* d` (const on the constant part, mutable on the state).
- Claim 2: mjsan.h defines `mjUSEASAN` when AddressSanitizer is active and poisons the arena stack, so ASan is supported at the library level.
- Bears on this repository: "control code may read sensors only" matches the const-model, mutable-data signature split; a check is a grep that controllers take `const` references.

---------------------------------------------------------------------------
# PART 2: MoveIt 2 (all file URLs on branch main)

## Section answers

1. Tree division.
   - Root listing: `moveit` (metapackage), `moveit_common`, `moveit_configs_utils`, `moveit_core`, `moveit_kinematics`, `moveit_planners`, `moveit_plugins`, `moveit_py`, `moveit_ros`, `moveit_runtime`, `moveit_setup_assistant`, plus `.clang-tidy .clang-format AGENTS.md CLAUDE.md GEMINI.md MIGRATION.md doc` (listing of the root).
   - AGENTS.md "Architecture": "The layers depend in one direction, from bottom to top": `moveit_common` (the `moveit_package()` macro), `moveit_core` (libraries using ROS only for messages, tf2 math, pluginlib, logging, parameters), plugin implementations (`moveit_kinematics`, `moveit_planners` containing OMPL, Pilz, CHOMP, STOMP; `moveit_plugins`), `moveit_ros` (integration: planning, move_group, servo, perception...), then `moveit_py`, `moveit_configs_utils`, `moveit_setup_assistant`.
   - `moveit_core` subdirectories (listing and moveit_core/CMakeLists.txt `add_subdirectory` lines): collision_detection, collision_detection_bullet, collision_detection_fcl, collision_distance_field, constraint_samplers, controller_manager, distance_field, dynamics_solver, exceptions, kinematic_constraints, kinematics_base, kinematics_metrics, macros, online_signal_smoothing, planning_interface, planning_scene, robot_model, robot_state, robot_trajectory, trajectory_processing, transforms, utils, version. Each has `CMakeLists.txt include src test` (listing of robot_state, robot_model, utils).
   - Dependency direction is visible in each subdirectory's `target_link_libraries`: `moveit_kinematics_base` links `moveit_macros moveit_robot_model moveit_utils`; `moveit_planning_interface` links `moveit_robot_trajectory moveit_robot_state moveit_planning_scene moveit_utils`; `moveit_collision_detection` links `moveit_robot_state moveit_utils` (the three CMakeLists.txt). Packages declare their ROS and system dependencies in package.xml (`<depend>` lines of moveit_core/package.xml).
2. Build targets and visibility.
   - One `add_library(<name> SHARED ...)` per subdirectory, `target_include_directories(PUBLIC $<BUILD_INTERFACE:.../include> $<INSTALL_INTERFACE:include/moveit_core>)`, and `generate_export_header(<name>)` installing `<name>_export.h` (collision_detection/CMakeLists.txt and kinematics_base/CMakeLists.txt). `moveit_core/CMakeLists.txt` adds `add_library(moveit_core INTERFACE)` that links every sub-library, so users can link one name. No per-target visibility mechanism like Bazel's exists; "what may see what" is by link line and by the include path.
   - Plugin boundary: AGENTS.md says moveit_core "defines the abstract plugin interfaces": `kinematics::KinematicsBase`, `planning_interface::PlannerManager`, `PlanningRequestAdapter`/`PlanningResponseAdapter`, `collision_detection::CollisionDetectorAllocator`, `moveit_controller_manager::MoveItControllerManager`; implementations load at run time through pluginlib. The FCL and Bullet checkers are separate libraries (`collision_detection_fcl`, `collision_detection_bullet`) plus plugin libraries `collision_detector_fcl_plugin`, `collision_detector_bullet_plugin` (the install block in moveit_core/CMakeLists.txt), implementing `CollisionDetectorAllocator` (collision_detector_allocator.hpp) whose pure virtuals are `getName()` and three `allocateEnv(...)`. The planner interface is `PlanningContext` with pure virtual `solve`, `terminate`, `clear`, and `PlannerManager` (planning_interface.hpp); the OMPL plugin lives in `moveit_planners/ompl`.
3. Tests.
   - `test/` inside each subdirectory (`moveit_core/robot_state/test/robot_state_test.cpp`, `test_aabb.cpp`, `test_cartesian_interpolator.cpp`, `robot_state_benchmark.cpp`), registered in that subdirectory's CMakeLists.txt under `if(BUILD_TESTING)` with `ament_add_gtest` (collision_detection/CMakeLists.txt). Fixture robots come from the separate repository `moveit_resources` (`<test_depend>moveit_resources_panda_moveit_config</test_depend>` in moveit_core/package.xml) through `moveit::core::loadTestingRobotModel("panda")`, or from `RobotModelBuilder` (AGENTS.md "Testing"). Comparisons use explicit tolerances: `constexpr double EPSILON{ 1.e-9 }` and `EXPECT_NEAR(angle, 0.0, 1e-05)` in robot_state_test.cpp. I saw no stored expected-output files in the tests I read. AGENTS.md also asks for a test that "a naive implementation would fail" and for error-path and boundary tests first. CI uploads a coverage report (CCOV job in ci.yaml).
4. Warnings and analysis, exactly.
   - moveit_common/cmake/moveit_package.cmake (macro `moveit_package()`): C++17 default, extensions off; for non-MSVC `-Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wcast-qual -Wold-style-cast -Wformat=2`; option `MOVEIT_CI_WARNINGS` (ON) repeats the list for GNU and Clang and adds `-Wno-maybe-uninitialized` for GNU; `CMAKE_EXPORT_COMPILE_COMMANDS ON`; default build type Release. There is NO `-Werror` in this file.
   - `-Werror` is added in `.github/workflows/ci.yaml`: `-DCMAKE_CXX_FLAGS="-Werror ${EXTRA_CXX_FLAGS} $CXXFLAGS ..."` with `CXXFLAGS: -Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls`; matrix images lyrical-ci-testing, rolling-ci, jazzy-ci, humble-ci; some use `-Wno-error=deprecated-declarations`.
   - `.clang-tidy`: `Checks: '-*, performance-*, -performance-enum-size, llvm-namespace-comment, modernize-redundant-void-arg, modernize-use-nullptr, modernize-use-default, modernize-use-override, modernize-loop-convert, modernize-make-shared, modernize-make-unique, modernize-avoid-bind, misc-unused-parameters, readability-braces-around-statements, readability-named-parameter, readability-redundant-smartptr-get, readability-redundant-string-cstr, readability-simplify-boolean-expr, readability-container-size-empty, readability-identifier-naming, readability-static-definition-in-anonymous-namespace'`; `HeaderFilterRegex: ''`; options include `misc-unused-parameters.StrictMode: 1`, `llvm-namespace-comment.ShortNamespaceLines: 10`, and `readability-identifier-naming` cases (CamelCase types, camelBack functions, lower_case variables, trailing `_` for private/protected members, UPPER_CASE constants, `s_` prefix for statics). There is no `WarningsAsErrors` key in the file.
   - clang-tidy in CI: matrix entry with `CLANG_TIDY: pedantic` (industrial_ci) builds with `CC: clang`, `CXX: clang++`, with `CLANG_TIDY_BASE_REF` set to the base ref, so only changed code is checked (job name suffix "clang-tidy (delta)"); `workflow_dispatch` runs "(all)". Generated IKFast plugin code is excluded by deleting its compile_commands.json. No sanitizer job found in ci.yaml (grep for sanit/asan: no hit); `--coverage` is used in one job.
   - `.clang-format`: Google base style, 120 columns, `SortIncludes: false`, indent 2. `.pre-commit-config.yaml` hooks: pre-commit-hooks (check-added-large-files, check-merge-conflict, check-yaml, detect-private-key, trailing-whitespace and others), black, clang-format, codespell, cmake-format and cmake-lint; `.github/workflows/format.yaml` runs it on pull requests.
5. Decisions and style.
   - AGENTS.md (the repository's file for coding agents, with CLAUDE.md and GEMINI.md that only import it) states principles ("Fail loudly, never fake it", "Safety checks fail closed", "Keep the scope minimal", "Validate before calling it done"), build and test commands, architecture, testing rules, git rules and a tag for AI-written GitHub text. `.claude/rules/cpp-style.md` holds the C++ style in a file that loads only when matching files are read. MIGRATION.md has date-prefixed entries `- [MM/YYYY] ...` with `diff` blocks; AGENTS.md says an API change needs one. `doc/DISTRO_COMPAT_POLICY.md` is a written policy on branches and CI matrix. CONTRIBUTING.md is 4 lines and links to a website. CHANGELOG.rst files are generated at release ("Do not edit"). No ADR directory found (root listing and `doc/` contain `DISTRO_COMPAT_POLICY.md` and `MIGRATION_GUIDE.md` only).
6. Units, frames, determinism.
   - Frames and poses: `Eigen::Isometry3d` is the pose type in `robot_state.hpp` (`setFromIK(const JointModelGroup*, const Eigen::Isometry3d& pose, ...)`, `setJointPositions(..., const Eigen::Isometry3d&)`); frames are named by strings: `getGlobalLinkTransform("base_link")` in robot_state_test.cpp, and `RobotModel::getModelFrame()` returns `const std::string&` ("the frame in which the transforms for this model are computed"). Units are the ROS convention (meters, radians, seconds); I did not find a file that states this and do not claim it beyond the test values.
   - Random numbers: `RobotState::setToRandomPositions()` uses `random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();` and an overload takes the generator as a parameter (robot_state.cpp, robot_state.hpp); `random_numbers` is a package dependency in moveit_core/package.xml. So a caller can pass its own generator. In OMPL (below) a process-wide seed is available.
7. Do not copy / copy.
   - Do NOT copy: run-time plugin loading (pluginlib, `*_plugin_description.xml` files, a "plugin cache") and the split into many packages with `package.xml` files. Reason: AGENTS.md says implementations are "loaded at runtime through pluginlib", which exists so separately released packages can be swapped by configuration (the pipeline loads planner plugins "from parameters"). The target repository has one owner and one build; its switch for OMPL against its own RRT is a compile-time or config enum, and an error in a plugin name would then appear only at run time.
   - Do NOT copy: tying warnings-as-errors to the CI command line only (moveit_package.cmake has no `-Werror`; ci.yaml adds it) and clang-tidy on "delta only" with no `WarningsAsErrors`. For a repository changed by agents, the build file should hold `-Werror` so every local build is the same as CI.
   - Do NOT copy: the 4-line CONTRIBUTING.md that links to an external website (MoveIt's own AGENTS.md, in contrast, is the single instruction file).
   - COPY 1: abstract interfaces at the two seams with pure virtual functions and one aggregate target (`CollisionDetectorAllocator`, `PlanningContext::solve/terminate/clear`) in a base library with no implementation dependencies. In the target repository: "planner" and "collision checker" as small abstract classes in their own targets; check: implementations are not included by the code that uses them.
   - COPY 2: AGENTS.md as one file with principles, exact commands, test rules ("Test the contract, not the implementation", "Include at least one test that a naive implementation would fail", "Assert specific values"), a MIGRATION.md entry per API change, and rules in a file that loads only when matching files are touched (`.claude/rules/cpp-style.md`). Also `.clang-tidy` naming checks with a small check list, and "delta only" clang-tidy as a starting step for a legacy code base.

---------------------------------------------------------------------------
## MoveIt 2 source blocks

### MoveIt AGENTS.md
- Citation: moveit/moveit2, "AGENTS.md", main @8c99e52. https://github.com/moveit/moveit2/blob/main/AGENTS.md
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 1, 2, 3, 4, 5, 7
- Claim 1: Architecture is stated as a one-directional layering (`moveit_common` below `moveit_core` below plugin packages below `moveit_ros`); `moveit_core` "defines the abstract plugin interfaces" and each sub-library is a separate shared library with one INTERFACE target `moveit_core::moveit_core` linking all.
  Quote: "The layers depend in one direction, from bottom to top"
- Claim 2: Rules for agents: "Fail loudly, never fake it", "Keep the scope minimal" (delete code left without callers), "Validate before calling it done"; tests first, contract not implementation, error paths and boundaries first, at least one test a naive implementation would fail, explicit tolerances; CI builds with `-Werror` and clang-tidy on changed code only.
  Quote: "Test the contract, not the implementation."
- Claim 3: API changes need an entry in MIGRATION.md; GitHub text written by AI starts with `[written by AI]`; `CHANGELOG.rst` is not edited by hand.
- Bears on this repository: AGENTS.md rules phrased as yes/no questions (does every deleted caller's code get deleted; is there a regression test that fails on the old code) are directly reusable.

### MoveIt .claude/rules/cpp-style.md
- Citation: moveit/moveit2, ".claude/rules/cpp-style.md", main @8c99e52. https://github.com/moveit/moveit2/blob/main/.claude/rules/cpp-style.md
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 5, 4, 6
- Claim 1: Formatting is mechanical (clang-format) and the naming table is enforced by clang-tidy `readability-identifier-naming`, which "CI runs ... on changed lines"; extra rules cover what tools do not: do not use `auto` for Eigen expression results, use `moveit::getLogger("moveit.<area>.<component>")`, `#pragma once`, `.hpp` headers.
  Quote: "Do not use `auto` for the result of an Eigen expression."
- Claim 2: `moveit_core` "uses ROS only for messages ... Do not add subscriptions, publishers ..." so the core layer stays free of middleware: a layering rule stated in prose.
- Bears on this repository: the Eigen `auto` rule applies directly (Eigen is a dependency) and can be checked with a grep or a clang-tidy check; the "core has no middleware" rule maps to "core does not include GLFW or JSON".

### MoveIt .clang-tidy
- Citation: moveit/moveit2, ".clang-tidy", main @8c99e52. https://github.com/moveit/moveit2/blob/main/.clang-tidy
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 4
- Claim 1: The check list starts from nothing (`-*`) and enables `performance-*` (minus `performance-enum-size`), `llvm-namespace-comment`, seven `modernize-*` checks (redundant-void-arg, use-nullptr, use-default, use-override, loop-convert, make-shared, make-unique, avoid-bind), `misc-unused-parameters`, and `readability-*` checks (braces-around-statements, named-parameter, redundant-smartptr-get, redundant-string-cstr, simplify-boolean-expr, container-size-empty, identifier-naming, static-definition-in-anonymous-namespace).
  Quote: "Checks:          '-*,"
- Claim 2: `HeaderFilterRegex: ''` and no `WarningsAsErrors` key: findings are not errors by this file, and headers are not reported.
- Bears on this repository: an opt-in list that begins with `-*` is the safe shape for a first clang-tidy file; the repository must add `WarningsAsErrors` itself to make the check enforceable.

### MoveIt moveit_common/cmake/moveit_package.cmake
- Citation: moveit/moveit2, "moveit_common/cmake/moveit_package.cmake", main @8c99e52. https://github.com/moveit/moveit2/blob/main/moveit_common/cmake/moveit_package.cmake
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 4, 1
- Claim 1: One macro `moveit_package()` is called by every package and sets the C++ standard, `-Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wcast-qual -Wold-style-cast -Wformat=2`, compile_commands export and default build type; `-Werror` does not appear.
- Claim 2: `-Wno-maybe-uninitialized` is added for GNU with the comment "This too often has false-positives" (the opt-out has a stated reason).
  Quote: "This too often has false-positives"
- Bears on this repository: one place for the warning list is the right shape; `-Wold-style-cast` and `-Wcast-qual` are cheap extra GCC flags to consider; the single `.bazelrc` or a `copts` constant plays the macro's role.

### MoveIt .github/workflows/ci.yaml
- Citation: moveit/moveit2, ".github/workflows/ci.yaml", main @8c99e52. https://github.com/moveit/moveit2/blob/main/.github/workflows/ci.yaml
- Kind: primary
- Read: 2026-11-25, in part (lines 1 to 80 and grep of the rest for tidy, sanitizer, Werror)
- Topics: 4, 3
- Claim 1: `-Werror` is passed in `TARGET_CMAKE_ARGS` with the warning list in `CXXFLAGS`; a matrix entry sets `CLANG_TIDY: pedantic`, uses `clang`/`clang++`, and compares to `CLANG_TIDY_BASE_REF` so only the diff is analysed; one entry builds `--coverage`.
  Quote: "-DCMAKE_CXX_FLAGS=\"-Werror ${{ matrix.env.EXTRA_CXX_FLAGS }} $CXXFLAGS"
- Claim 2: No AddressSanitizer or UBSan job exists in this workflow (no match for sanit/asan).
- Bears on this repository: an Ubuntu 24.04 job running clang-tidy on the changed files only is a feasible first step, with GCC for the real build.

### MoveIt moveit_core/CMakeLists.txt and package.xml
- Citation: moveit/moveit2, "moveit_core/CMakeLists.txt" and "moveit_core/package.xml", main @8c99e52. https://github.com/moveit/moveit2/blob/main/moveit_core/CMakeLists.txt , https://github.com/moveit/moveit2/blob/main/moveit_core/package.xml
- Kind: primary
- Read: 2026-11-25, in part (find_package block, add_subdirectory list, INTERFACE target, install and export lists; `<depend>` lines)
- Topics: 1, 2
- Claim 1: 23 `add_subdirectory` entries, each its own shared library; `add_library(moveit_core INTERFACE)` links them all; plugin libraries (`collision_detector_fcl_plugin`, `collision_detector_bullet_plugin`, filters) are installed under a separate export set `moveit_core_pluginTargets`.
- Claim 2: package.xml lists third-party dependencies with `<depend>` (angles, bullet, eigen, libfcl-dev, octomap, pluginlib, random_numbers, ruckig, ...) and test-only ones with `<test_depend>` (moveit_resources_panda_moveit_config); the declared dependency list matches the `find_package` list in CMake.
- Bears on this repository: a "dependency check" could compare declared third-party deps (MODULE.bazel and BUILD `deps`) to actual `#include`s, as package.xml and find_package are meant to agree.

### MoveIt collision_detection/CMakeLists.txt and collision_detector_allocator.hpp
- Citation: moveit/moveit2, "moveit_core/collision_detection/CMakeLists.txt" and "moveit_core/collision_detection/include/moveit/collision_detection/collision_detector_allocator.hpp", main @8c99e52. https://github.com/moveit/moveit2/blob/main/moveit_core/collision_detection/CMakeLists.txt , https://github.com/moveit/moveit2/blob/main/moveit_core/collision_detection/include/moveit/collision_detection/collision_detector_allocator.hpp
- Kind: primary
- Read: 2026-11-25, whole (CMakeLists.txt); in part (class declarations of the header)
- Topics: 1, 2, 3
- Claim 1: The base library `moveit_collision_detection` has `PUBLIC` include dir `include`, `generate_export_header`, links `moveit_robot_state moveit_utils` and not FCL or Bullet; tests (`test_world`, `test_world_diff`, `test_all_valid`) link only the base library, using a built-in `collision_env_allvalid` implementation.
- Claim 2: `CollisionDetectorAllocator` has pure virtual `getName()` and three `allocateEnv(...)` overloads, plus a `CollisionDetectorAllocatorTemplate<CollisionEnvType, AllocatorType>` that creates the pair; each detector library (FCL, Bullet) provides one.
  Quote: "A unique name identifying the CollisionWorld/CollisionRobot pairing."
- Bears on this repository: shows a small abstract seam plus a trivial "all valid" implementation used for tests, a pattern for the stand-in cell and the real cell behind one interface.

### MoveIt planning_interface.hpp and planning_interface/CMakeLists.txt
- Citation: moveit/moveit2, "moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp" and "moveit_core/planning_interface/CMakeLists.txt", main @8c99e52. https://github.com/moveit/moveit2/blob/main/moveit_core/planning_interface/include/moveit/planning_interface/planning_interface.hpp , https://github.com/moveit/moveit2/blob/main/moveit_core/planning_interface/CMakeLists.txt
- Kind: primary
- Read: 2026-11-25, in part (first 120 lines of the header; CMake whole)
- Topics: 1, 2
- Claim 1: A planner is a `PlanningContext` with pure virtual `solve(MotionPlanResponse&)`, `solve(MotionPlanDetailedResponse&)`, `terminate()` and `clear()`, created by a `PlannerManager`; settings arrive as `PlannerConfigurationSettings` (group, name, string-to-string map), where "Settings with unknown keys are ignored."
  Quote: "Settings with unknown keys are ignored."
- Claim 2: The interface library links only trajectory, state, scene, utils and message libraries, and does not link OMPL.
- Bears on this repository: the repository's "OMPL behind a switch" would be one `Planner` interface with `solve` and `terminate`; a yes-or-no check is that nothing outside the OMPL adapter includes `ompl/`. The string-keyed unknown-key-ignored settings are a thing NOT to copy (typos pass silently); a JSON schema checked by program would reject them.

### MoveIt kinematics_base/CMakeLists.txt
- Citation: moveit/moveit2, "moveit_core/kinematics_base/CMakeLists.txt", main @8c99e52. https://github.com/moveit/moveit2/blob/main/moveit_core/kinematics_base/CMakeLists.txt
- Kind: primary
- Read: 2026-11-25, whole
- Topics: 1, 2
- Claim 1: `moveit_kinematics_base` is a shared library of one source file (`src/kinematics_base.cpp`) that links `moveit_macros moveit_robot_model moveit_utils` and ROS message libraries; implementations (KDL, IKFast, cached IK) are separate in `moveit_kinematics` (listing) and loaded via `*_kinematics_plugin_description.xml` files.
- Bears on this repository: an IK seam is the same shape (one abstract class, many solvers), but the run-time loading is not needed.

### MoveIt MIGRATION.md
- Citation: moveit/moveit2, "Migration Notes (MIGRATION.md)", main @8c99e52. https://github.com/moveit/moveit2/blob/main/MIGRATION.md
- Kind: primary
- Read: 2026-11-25, in part (first 40 of 106 lines)
- Topics: 5
- Claim 1: API changes are recorded as one bullet each, date-prefixed (`[11/2024]`), with the old and new usage in a `diff` block (for example the change from `planning_plugin:` to `planning_plugins:`), newest first.
  Quote: "All MoveIt 2 headers have been updated to use the .hpp extension."
- Bears on this repository: a `MIGRATION.md`-style log is useful for a changed command-line or JSON format; for internal C++ API with one owner an ADR plus the commit history is enough.

### MoveIt robot_state_test.cpp and robot_state.hpp
- Citation: moveit/moveit2, "moveit_core/robot_state/test/robot_state_test.cpp" and "moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp" and "moveit_core/robot_state/src/robot_state.cpp", main @8c99e52. https://github.com/moveit/moveit2/blob/main/moveit_core/robot_state/test/robot_state_test.cpp , https://github.com/moveit/moveit2/blob/main/moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp , https://github.com/moveit/moveit2/blob/main/moveit_core/robot_state/src/robot_state.cpp
- Kind: primary
- Read: 2026-11-25, in part (grep for tolerances, transform types, random generator use)
- Topics: 3, 6
- Claim 1: Tests compare with named tolerances (`constexpr double EPSILON{ 1.e-9 }`, `EXPECT_NEAR(angle, 0.0, 1e-05)`) and a traced macro `EXPECT_NEAR_TRACED` for matrix elements; poses are `Eigen::Isometry3d` and links are addressed by string (`getGlobalLinkTransform("base_link")`).
- Claim 2: `setToRandomPositions()` draws from `random_numbers::RandomNumberGenerator& rng = getRandomNumberGenerator();`, and an overload takes an external generator, so a test can supply a seeded one.
- Bears on this repository: pass the random generator (seeded) as an argument instead of reading a global one; checkable by grep for `std::random_device` and `rand(` in `src/`.

### MoveIt doc/DISTRO_COMPAT_POLICY.md
- Citation: moveit/moveit2, "Distro Compatibility Policy", main @8c99e52. https://github.com/moveit/moveit2/blob/main/doc/DISTRO_COMPAT_POLICY.md
- Kind: primary
- Read: 2026-11-25, in part (first 30 lines)
- Topics: 5
- Claim 1: A written policy document states four topics (per-distro branches, code patterns for differences, dependencies without a binary, CI matrix policy) and tells contributors which source to use for stability against features; it is the closest thing to a design record in the repository (no ADR directory).
- Bears on this repository: policy documents can be cited from AGENTS.md rules ("see section 2.6"), but there is nothing here to copy on distros; the repository has one build.

---------------------------------------------------------------------------
# PART 3: OMPL (brief)

### OMPL RandomNumbers.h and RandomNumbers.cpp (RNG::setSeed)
- Citation: ompl/ompl, "src/ompl/util/RandomNumbers.h" and "src/ompl/util/src/RandomNumbers.cpp", main @340bd21. https://github.com/ompl/ompl/blob/main/src/ompl/util/RandomNumbers.h , https://github.com/ompl/ompl/blob/main/src/ompl/util/src/RandomNumbers.cpp
- Kind: primary
- Read: 2026-11-25, in part (the seed functions and the seed-generator class)
- Topics: 6
- Claim 1: Every `RNG` instance gets its seed from a process-wide seed generator, whose first seed is taken from the clock unless `RNG::setSeed(seed)` is called; `getSeed()` returns it so that a failing run can be repeated ("Useful for debugging").
  Quote: "Set the seed used to generate the seeds of each RNG instance."
- Claim 2: `setSeed` must be called before any RNG is created: afterwards it logs "Random number generation already started. Changing seed now will not lead to deterministic sampling." and a seed of 0 is replaced by 1 with a warning. The rng class takes a mutex for seed generation, so seeds depend on the order of RNG creation across threads.
- Bears on this repository: before any OMPL planner call, one rule: set `ompl::RNG::setSeed(<config seed>)` once at start-up and print the seed; with threads, determinism is not guaranteed by this alone.

### OMPL src/ompl/CMakeLists.txt and tree
- Citation: ompl/ompl, "src/ompl/CMakeLists.txt" and the directory listing of `src/ompl`, main @340bd21. https://github.com/ompl/ompl/blob/main/src/ompl/CMakeLists.txt
- Kind: primary
- Read: 2026-11-25, in part (source globbing and library definition; directory listing)
- Topics: 1, 2
- Claim 1: `src/ompl` is divided into `base control datastructures extensions geometric multilevel tools util vamp`; one library `ompl` is built by `file(GLOB_RECURSE ... datastructures/*.cpp util/*.cpp base/*.cpp geometric/*.cpp control/*.cpp multilevel/*.cpp tools/*.cpp)`, with `Boost::serialization` and `Eigen3::Eigen` as PUBLIC links. The planners are under `geometric/planners` (rrt, prm, kpiece, sbl, sst, est, fmt, informedtrees ...).
- Bears on this repository: OMPL is one library with an internal directory split by layer (util below base below geometric), not many packages; the layering is by directory, not enforced by the build.

---------------------------------------------------------------------------
## Does not fit

- MuJoCo, "STYLEGUIDE.md" (short names, horizontal-space economy, prefix namespaces such as `mju_`, `mjv_`): a C API with a global namespace needs prefixes and short names; a C++20 repository with `namespace`, longer descriptive names and agent contributors gains from clarity, and names must carry units.
- MuJoCo, "VERSIONING.md" (SUPERMAJOR.MAJOR.MINOR_OR_PATCH and ABI notes in a changelog): serves external users of a library; a repository with one owner and no downstream users needs no version scheme for internal code.
- MuJoCo, "CONTRIBUTING.md" (contact maintainers before large work, CLA, review by maintainers): the process presumes a community; here the equivalent is AGENTS.md plus the checks.
- MoveIt, "moveit_core/CMakeLists.txt" and pluginlib files (many shared libraries, plugin description XML, run-time loading): suits independently released ROS packages; a Bazel repository builds static libraries in one workspace and wants link-time errors.
- MoveIt, ".github/workflows/ci.yaml" (Docker images, industrial_ci, matrix of four ROS distros, upstream workspace caches): ROS-specific; the repository has one OS and no ROS.
- MoveIt, "DISTRO_COMPAT_POLICY.md" (per-distro branches and backports): no distros here.
- MoveIt, `-Wold-style-cast`, `-Wformat=2` (from moveit_package.cmake) are candidates to adopt but only after counting existing hits; turning them on with `-Werror` in a 40,000-line repository at once can break the build.

## Leads not followed

- https://moveit.ai/documentation/contributing/ (named in MoveIt's CONTRIBUTING.md; the MoveIt code style page) not fetched.
- MuJoCo `.github/workflows/build.yml` and `build_steps.sh` read only by grep for sanitizers and clang (found only Clang 13 official builds and clang-18 wasm); the full job list not read.
- MuJoCo `cmake/MujocoLinkOptions.cmake`, `cmake/CheckAvxSupport.cmake` not read.
- MoveIt `.github/workflows/prerelease.yaml`, `sonar.yaml` (SonarCloud analysis) not read.
- MoveIt `moveit_planners/ompl` source (how the OMPL plugin implements `PlanningContext` and whether it seeds `RNG`) not read; the claim above on the plugin is from AGENTS.md and the directory listing only.
- MoveIt `moveit_resources` repository (the stored robot models) not read.
- MuJoCo `doc/XMLreference.rst` (MJCF angle default) not read, so no claim is made on angle units.
