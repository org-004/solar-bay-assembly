# Research B: Bazel package cut and header checks (b); compiler warnings (i)

All sources fetched 2026-11-25 with HTTP status 200 (curl). Sources 1-17 are primary, source 18 secondary. Web pages were converted to text and the quoted passages copied from that text.

### Bazel "Visibility"
- Citation: Bazel project, "Visibility", current docs (HEAD). https://bazel.build/concepts/visibility
- Kind: primary
- Read: 2026-11-25, in part (Target visibility, specifications, package_group)
- Topics: b
- Claim 1: Visibility is a list of allowed packages. The forms are `//visibility:public`, `//visibility:private`, `//pkg:__pkg__` (that package only), `//pkg:__subpackages__` (it and everything below) and a `package_group` label. A target that violates a dependency's visibility fails during analysis. Visibility checking can be switched off with `--check_visibility=false`, which the page says is for prototyping and not for submitted code.
  Quote: "To make several targets visible to the same set of packages, use a package_group instead of repeating the list in each target’s visibility attribute."
- Claim 2: Within a `package_group` the syntax differs: `//foo/bar` for one package and `//foo/bar/...` for the tree, and `public`/`private` without the `//visibility:` prefix.
- Bears on this repository: with one package, `visibility` has no effect (everything is same-package). After the cut, a rule such as "every non-public target has an explicit `visibility`, and no `package(default_visibility = ["//visibility:public"])`" is checkable by `grep`/`bazel query`.

### Bazel "BUILD Style Guide"
- Citation: Bazel project, "BUILD Style Guide", current docs (HEAD). https://bazel.build/build/style-guide
- Kind: primary
- Read: 2026-11-25, in part (Target naming, Visibility, Dependencies, Globs)
- Topics: b
- Claim 1: Visibility should be as tight as possible while still allowing tests and reverse dependencies; package-wide `default_visibility = //visibility:public` is to be avoided, and `//visibility:public` is for the project's public API only.
  Quote: "Avoid setting package default_visibility to //visibility:public."
- Claim 2: `deps` lists only direct dependencies (those needed by the rule's own sources), never transitive ones. A one-source target is named after its source (`chat` for `chat.cc`). Recursive globs are discouraged in favour of a BUILD file per directory with a dependency graph between them.
  Quote: "It is good practice to author a BUILD file in each directory and define a dependency graph between them."
- Claim 3: `package(default_testonly = True)` appears in the guide's example for test-only packages.
- Bears on this repository: gives checkable rules (no public default visibility, one-source targets named after the source, deps direct only). It also says one BUILD file per directory, which clashes with the flat `src/` unless `src/` is first split into directories; see "Does not fit".

### Bazel "Best Practices"
- Citation: Bazel project, "Best Practices", current docs (HEAD). https://bazel.build/configure/best-practices
- Kind: primary
- Read: 2026-11-25, in part (introduction, Packages)
- Topics: b
- Claim 1: The goals named are fine-grained dependencies for parallelism and incrementality, well-encapsulated dependencies, testable structure. The page says its guidelines are not requirements and few projects meet all of them.
  Quote: "Every directory that contains buildable files should be a package."
- Claim 2: A BUILD file that reaches into subdirectories (`srcs = ["a/b/C.java"]`) is a sign a BUILD file is missing there; the longer such structure persists, the more likely circular dependencies and target scope creep.
- Bears on this repository: supports a rule "no `srcs` entry contains a `/`" as a yes-or-no check once `src/` is split. At about 100 targets the page's "every directory a package" would give many tiny packages, so the editor should choose a coarser cut (a handful of packages by layer), which the page permits since it is advisory.

### Bazel "C / C++ Rules" (cc_library, layering_check, implementation_deps)
- Citation: Bazel project, "C / C++ Rules", Build Encyclopedia, current docs (HEAD). https://bazel.build/reference/be/c-cpp
- Kind: primary
- Read: 2026-11-25, in part (cc_library: hdrs/srcs, layering_check, implementation_deps, strip_include_prefix)
- Topics: b
- Claim 1: The `hdrs` against `srcs` choice is a public/private choice: a header in `hdrs` may be included by dependents, one in `srcs` may not. `cc_binary` and `cc_test` have no `hdrs`; their headers go in `srcs`.
  Quote: "you should ask whether you want consumers of this library to be able to directly include it."
- Claim 2: `layering_check` makes a `.cc` include only headers of its own target and of its direct `deps`; transitive includes by the compiler are allowed but a source may not name them. It is a toolchain feature that must be requested (`--features=layering_check` or the `features` parameter of `package()`); Bazel's own toolchains support it only with clang on Unix and macOS.
  Quote: "The toolchains provided by Bazel only support this feature with clang on Unix and macOS."
- Claim 3: `implementation_deps` are like `deps` but their headers and include paths are used only to compile this library, not its dependents (still linked into binaries). `strip_include_prefix` makes `hdrs` visible at a path with the prefix cut off; the prefix `include_prefix` is added afterwards. The attribute text says it is only legal under third_party (as fetched).
- Bears on this repository: the project builds with GCC only, so `layering_check` and `parse_headers` do not run there; a check by `implementation_deps` or `depend_on_what_you_use` is the GCC-compatible route. Rule: "a header listed in `hdrs` is included from outside its package, otherwise move it to `srcs`".

### Bazel command-line reference (`--per_file_copt`, `--features`)
- Citation: Bazel project, "Command-Line Reference", current docs (HEAD). https://bazel.build/reference/command-line-reference
- Kind: primary
- Read: 2026-11-25, in part (entries `--features`, `--per_file_copt`)
- Topics: b
- Claim 1: `--features` turns features on (or off with `-feature`) for targets in the target configuration; negative features always override positive ones; values accumulate.
  Quote: "Specifying -{feature} will disable the feature. Negative features always override positive ones."
- Claim 2: `--per_file_copt=<regex filter>@opt1,opt2` adds compiler options to files whose path matches, with `-` prefixed regexes for exclusions; example from the page: `//foo/.*.cc,-//foo/bar.cc@-O0`. It can be repeated.
- Bears on this repository: a `.bazelrc` line such as `build --per_file_copt=src/legacy/.*@-Wno-conversion` would let a stricter flag be switched on for new directories only (staged rollout), and `-Wconversion` debt in the stand-in cell can be fenced off until it is replaced. Checkable by `grep per_file_copt .bazelrc`.

### depend_on_what_you_use (DWYU) README
- Citation: Martin Medler (martis42), "Depend on what you use (DWYU)" README, main branch. https://raw.githubusercontent.com/martis42/depend_on_what_you_use/main/README.md
- Kind: primary
- Read: 2026-11-25, whole
- Topics: b
- Claim 1: DWYU is a Bazel aspect that checks the headers a `cc_*` target includes against its `deps`: it finds includes only reachable transitively, finds unused deps, and (when enabled) checks `implementation_deps` against `deps`. The rule it enforces is that a target depends directly on the targets providing the headers it includes. It needs Bazel 8.2.0 or newer and C++11 or newer; Linux is fully supported.
  Quote: "A cc_* target X shall depend directly on the targets providing the header files which are included in the source code of X."
- Claim 2: Compared with `layering_check`, DWYU only runs the preprocessor (no full compile), handles header-only libraries, is not limited to LLVM toolchains, detects unused deps and offers automatic fixes. It assumes the code compiles and that include paths are unambiguous.
- Bears on this repository: this is the only listed tool that checks `deps` against includes under GCC. A rule "`bazel build //... --aspects=...dwyu...` passes" is a yes-or-no CI check; it becomes useful only after the one package is cut into several, since inside one target the aspect has nothing to compare.

### bazel_clang_tidy README
- Citation: Erenon, "bazel_clang_tidy" README, master branch. https://raw.githubusercontent.com/erenon/bazel_clang_tidy/master/README.md
- Kind: primary
- Read: 2026-11-25, whole
- Topics: b
- Claim 1: It runs clang-tidy per Bazel target as an aspect (`--aspects @bazel_clang_tidy//clang_tidy:clang_tidy.bzl%clang_tidy_aspect --output_groups=report`), without a compile-commands database, with reports cached by Bazel so only stale ones are recomputed.
  Quote: "Run clang-tidy on Bazel C/C++ targets directly, without generating a compile commands database"
- Claim 2: The config file is not passed on the command line, so `.clang-tidy` must sit in a parent of all sources; targets tagged `no-clang-tidy` or `noclangtidy` and external targets are skipped. There is a section on using it with a non-system GCC (`clang_tidy_gcc_install_dir`), so a GCC-built repo is a supported case.
- Bears on this repository: a `.bazelrc` config `build:clang-tidy` plus a `tags = ["noclangtidy"]` marker for the stand-in cell would give a per-target check that works with the GCC-only toolchain. The page offers no pinned release (the example uses a commit), so pin a commit in `MODULE.bazel`.

### Hedron compile-commands extractor README
- Citation: Hedron Vision, "bazel-compile-commands-extractor" README, main branch. https://raw.githubusercontent.com/hedronvision/bazel-compile-commands-extractor/main/README.md
- Kind: primary
- Read: 2026-11-25, in part (introduction, setup, "second, get the extractor running")
- Topics: b
- Claim 1: It exports Bazel's compile actions to `compile_commands.json` so clangd, clang-tidy and similar tools can work outside Bazel; run with `bazel run @hedron_compile_commands//:refresh_all`, and it must be re-run after BUILD changes. It has a bzlmod setup section.
- Claim 2: Bazel flags you use when developing (`--config=...`, `--compilation_mode=dbg`) must be passed to the tool too (after `--`), otherwise it will misread the build. Header-only libraries need a test or binary target that compiles them to be listed.
- Bears on this repository: a `compile_commands.json` is the input for plain `clang-tidy` / `run-clang-tidy` if the aspect route is not chosen; pass the same `--config` as CI. File is generated, so it belongs in `.gitignore`.

### Drake warning flags (`drake_cc.bzl`)
- Citation: Robot Locomotion Group, Drake, `tools/skylark/drake_cc.bzl`, master branch. https://raw.githubusercontent.com/RobotLocomotion/drake/master/tools/skylark/drake_cc.bzl
- Kind: primary
- Read: 2026-11-25, in part (lines 20-115 flags; 595-620 `internal`/visibility)
- Topics: b, i
- Claim 1: Exact flags. All compilers (`CXX_FLAGS`): `-Werror=all -Werror=attributes -Werror=cpp -Werror=deprecated -Werror=deprecated-copy-dtor -Werror=deprecated-declarations -Werror=ignored-qualifiers -Werror=missing-declarations -Werror=overloaded-virtual -Werror=shadow -Werror=unused-result`. GCC adds `-Werror=extra -Werror=logical-op -Werror=non-virtual-dtor -Werror=pessimizing-move -Werror=return-local-addr -Werror=uninitialized -Werror=unused-but-set-parameter`. Clang adds `-Werror=absolute-value -Werror=c99-designator -Werror=inconsistent-missing-override -Werror=final-dtor-non-final-class -Werror=literal-conversion -Werror=non-virtual-dtor -Werror=range-loop-analysis -Werror=return-stack-address -Werror=sign-compare -Werror=unqualified-std-cast-call`. Tests add `-Wno-sign-compare` (and for GCC `-Wno-missing-declarations -Wno-unused-parameter`).
- Claim 2: Under GCC Drake switches off the optimizer-dependent warnings, each with a stated cause: `-Wno-missing-field-initializers` (too strict as an error), `-Wno-dangling-reference` (false on const-reference returns), `-Wno-maybe-uninitialized` ("falsely dings code inside Eigen"), `-Wno-stringop-overflow` (libstdc++), `-Wno-array-bounds` and `-Wno-stringop-overread` (Eigen 16-byte alignment with Vector1d), `-Wno-psabi` (ARM64 noise). The errors can be demoted to plain warnings by a `_defang` function and a `select()` on `gcc_with_errors`/`gcc_with_warnings`.
  Quote: "This falsely dings code inside Eigen."
- Claim 3: Drake's `drake_cc_library` has an `internal = True` option that fails the build if `visibility` is empty or `//visibility:public`, and the same file fails when `internal` is used together with `testonly`.
- Bears on this repository: a ready, field-tested GCC list that includes `-Wshadow`, `-Wlogical-op` and `-Wuninitialized` (not `-Wconversion`) for a numerical Eigen-based code base; also shows the pattern "errors in CI, warnings for developers" via a `select()`. Note Drake does not enable `-Wconversion`, `-Wfloat-equal` or `-Wold-style-cast`.

### Abseil compiler options (`copts.py`)
- Citation: Abseil project, `absl/copts/copts.py`, master branch. https://raw.githubusercontent.com/abseil/abseil-cpp/master/absl/copts/copts.py
- Kind: primary
- Read: 2026-11-25, in part (GCC, LLVM and test flag lists)
- Topics: i
- Claim 1: `ABSL_GCC_FLAGS` = `-Wall -Wextra -Wcast-qual -Wconversion-null -Wformat-security -Wmissing-declarations -Wnon-virtual-dtor -Woverlength-strings -Wpointer-arith -Wundef -Wunused-local-typedefs -Wunused-result -Wvarargs -Wvla -Wwrite-strings -DNOMINMAX`. No `-Werror`.
- Claim 2: The LLVM list is much longer and does include `-Wconversion`, `-Wsign-conversion`, `-Wshorten-64-to-32`, `-Wshadow-all`, `-Wundef`, `-Wunreachable-code`, `-Wthread-safety`, with `-Wno-float-conversion -Wno-implicit-float-conversion -Wno-implicit-int-float-conversion` switched off. For tests it turns off `-Wno-sign-compare`, `-Wno-shadow`, `-Wno-sign-conversion`, `-Wno-unused-parameter` and others. The file's comment notes that clang-cl treats `-Wall` as `-Weverything`.
- Bears on this repository: shows the pattern "stricter set for library code, relaxed set for tests" (separate `*_TEST_ADDITIONAL_FLAGS`), and that GCC and Clang lists differ because the compilers' groupings differ; the repository, GCC-only, can take the GCC list plus `-Wshadow`, and keep `-Wconversion` in the "later" list.

### MuJoCo build options (`MujocoOptions.cmake`)
- Citation: Google DeepMind, MuJoCo, `cmake/MujocoOptions.cmake`, main branch. https://raw.githubusercontent.com/google-deepmind/mujoco/main/cmake/MujocoOptions.cmake
- Kind: primary
- Read: 2026-11-25, in part (compile options block, lines 85-125)
- Topics: i
- Claim 1: For GCC and Clang MuJoCo uses `-Werror -Wall -Wpedantic -Wimplicit-fallthrough -Wunused -Wvla -Wno-int-in-bool-context -Wno-sign-compare -Wno-unknown-pragmas`; for GCC only it adds `-Wimplicit-fallthrough=5` (a fallthrough only counts if annotated with the attribute, comments are not accepted), `-Wno-maybe-uninitialized` and `-Wno-psabi`.
- Claim 2: MuJoCo does not use `-Wextra`, `-Wshadow` or `-Wconversion`, though it is a numerical C library; the blanket `-Werror` applies to its own build.
- Bears on this repository: shows a physics engine that turns off `-Wmaybe-uninitialized` and `-Wsign-compare`, and the same choice as Drake and MoveIt on `-Wmaybe-uninitialized`. `-Wimplicit-fallthrough=5` plus `[[fallthrough]]` is a cheap, checkable rule.

### MoveIt 2 package defaults (`moveit_package.cmake`)
- Citation: MoveIt project, `moveit_common/cmake/moveit_package.cmake`, main branch. https://raw.githubusercontent.com/moveit/moveit2/main/moveit_common/cmake/moveit_package.cmake
- Kind: primary
- Read: 2026-11-25, in part (lines 36-95)
- Topics: i
- Claim 1: Every MoveIt package gets `-Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wcast-qual -Wold-style-cast -Wformat=2`; C++17 standard, no extensions. Behind option `MOVEIT_CI_WARNINGS` (default ON) the GCC list adds `-Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls -Wcast-qual` plus `-Wno-maybe-uninitialized`, with a comment.
  Quote: "This too often has false-positives"
- Claim 2: It sets `CMAKE_EXPORT_COMPILE_COMMANDS ON`. No `-Werror` here.
- Bears on this repository: this is a robotics (motion planning) project that enables `-Wold-style-cast` and `-Wformat=2` as plain warnings; a motion-planning peer, and it disables `-Wmaybe-uninitialized` under GCC as the others do.

### OMPL compiler settings
- Citation: OMPL project, `CMakeModules/OMPLCompilerSettings.cmake`, main branch. https://raw.githubusercontent.com/ompl/ompl/main/CMakeModules/OMPLCompilerSettings.cmake
- Kind: primary
- Read: 2026-11-25, in part (compiler flags, lines 1-20)
- Topics: i
- Claim 1: For GCC and Clang OMPL uses only `-W -Wall -Wextra -Wno-system-headers`, and prepends `-O3` to release flags. No `-Werror`.
- Bears on this repository: a library the repository links against sets no further flags; its own headers produce noise, so the repository's flags should be kept off third-party headers (use `-isystem`/`includes` via external repos; GCC's `-Wno-system-headers` is on by default).

### GCC 14.2 manual, "Options to Request or Suppress Warnings"
- Citation: GNU Project, "Options to Request or Suppress Warnings", GCC 14.2.0 manual. https://gcc.gnu.org/onlinedocs/gcc-14.2.0/gcc/Warning-Options.html
- Kind: primary
- Read: 2026-11-25, in part (-Werror, -Wall list, -Wextra list, and the entries for -Wdouble-promotion, -Wfloat-equal, -Wformat=2, -Wnull-dereference, -Wmaybe-uninitialized, -Warray-bounds, -Wstringop-overflow, -Wshadow, -Wconversion, -Wsign-conversion, -Wduplicated-cond, -Wlogical-op, -Wswitch-enum, -Wuseless-cast, -Wcast-align, -Wundef)
- Topics: i
- Claim 1: What is in which group. `-Wall` includes `-Wmaybe-uninitialized`, `-Wuninitialized`, `-Warray-bounds=1 (only with -O2)`, `-Wsign-compare` (C++), `-Wreorder`, `-Wunused`, `-Wparentheses`, `-Wswitch`, `-Wreturn-type`, `-Wformat=1`, `-Wrestrict`, `-Wnarrowing`, `-Wpessimizing-move`, `-Wrange-loop-construct`, `-Wuse-after-free=2`, `-Wdangling-pointer=2` and others. `-Wextra` adds `-Wimplicit-fallthrough=3`, `-Wmissing-field-initializers`, `-Wdeprecated-copy`, `-Wredundant-move`, `-Wignored-qualifiers`, `-Wsign-compare`, `-Wclobbered`, `-Wempty-body`, `-Wcast-function-type`, and also lists `-Wmaybe-uninitialized`. Not in either: `-Wshadow`, `-Wconversion`, `-Wsign-conversion`, `-Wfloat-equal`, `-Wdouble-promotion`, `-Wnull-dereference`, `-Wduplicated-cond`, `-Wlogical-op`, `-Wformat=2`, `-Wswitch-enum`, `-Wuseless-cast`, `-Wcast-align`, `-Wundef`, `-Wold-style-cast` (these are not in the two lists on the page).
- Claim 2: Optimizer dependence is stated by the manual. `-Wmaybe-uninitialized`: only possible with optimization, and "GCC may not be able to determine when the code is correct in spite of appearing to have an error". `-Wnull-dereference`: only active with `-fdelete-null-pointer-checks` (on by optimization on most targets). `-Warray-bounds`: more effective with `-ftree-vrp` (default at -O2 and up), level 2 gives "a larger number of false positives".
  Quote: "The precision of the warnings depends on the optimization options used."
- Claim 3: Per-flag purpose for numerical code. `-Wfloat-equal` flags every `==`/`!=` on floating-point values ("equality comparisons are probably mistaken"), so exact-zero or sentinel tests become noise. `-Wdouble-promotion` flags implicit `float` to `double`; the manual's example is a double literal in `float` code. `-Wduplicated-cond` flags a repeated condition in an if-else-if chain; `-Wlogical-op` flags `a < 0 && a < 0` and logical operators where a bit-wise operator was likely meant. `-Wswitch-enum` warns for a missing enumerator even when a `default:` exists. `-Wshadow` covers locals, parameters and class members. `-Wconversion` covers real/integer, narrowing and sign changes, but in C++ "warnings about conversions between signed and unsigned integers are disabled by default ... unless -Wsign-conversion is explicitly enabled". `-Werror=<name>` promotes one warning; `-Wno-error=<name>` exempts one under `-Werror`. `-Wformat=2` is "-Wformat -Wformat-nonliteral -Wformat-security -Wformat-y2k". `-Wundef` flags undefined identifiers in `#if`.
- Bears on this repository: the data for a two-tier rule: tier 1 (error): `-Wall -Wextra` plus the optimizer-independent `-Wshadow -Wlogical-op -Wduplicated-cond -Wformat=2 -Wundef`; tier 2 (warn, review): `-Wconversion -Wsign-conversion -Wfloat-equal -Wdouble-promotion -Wnull-dereference`; the four optimizer-dependent ones are not made errors. A pin to one GCC version in CI (Ubuntu 24.04 ships GCC 13) is needed for `-Werror` to be stable.

### GCC 14.2 manual, "Options Controlling C++ Dialect"
- Citation: GNU Project, "Options Controlling C++ Dialect", GCC 14.2.0 manual. https://gcc.gnu.org/onlinedocs/gcc-14.2.0/gcc/C_002b_002b-Dialect-Options.html
- Kind: primary
- Read: 2026-11-25, in part (entry `-Wold-style-cast`)
- Topics: i
- Claim 1: `-Wold-style-cast` warns on every C-style cast to a non-void type in C++, because the named casts are less prone to unintended effects and easier to search for. It is not in `-Wall` or `-Wextra`.
  Quote: "The new-style casts (dynamic_cast, static_cast, reinterpret_cast, and const_cast) are less vulnerable to unintended effects and much easier to search for."
- Bears on this repository: `-Wold-style-cast` is checkable and cheap to satisfy in modern C++20 code; it also fires on functional casts only partially, so a `grep` for `(double)` is not a substitute. Candidate for tier 1 if the code is already clean (a measurement the agent must do).

### Clang diagnostics reference
- Citation: LLVM project, "Diagnostic flags in Clang", Clang documentation (current). https://clang.llvm.org/docs/DiagnosticsReference.html
- Kind: primary
- Read: 2026-11-25, in part (entries -Wall, -Wmost, -Wextra, -Wconversion, -Wfloat-equal, -Wshadow-all, -Wnull-dereference)
- Topics: i
- Claim 1: Clang's `-Wall` is small: it controls `-Wmisleading-indentation`, `-Wmost`, `-Wpacked-non-pod`, `-Wparentheses`, `-Wswitch`, `-Wswitch-bool`, `-Wvla-cxx-extension`; most checks sit in `-Wmost`. `-Wextra` includes `-Wdeprecated-copy`, `-Wignored-qualifiers`, `-Wmissing-field-initializers`, `-Wsign-compare`, `-Wnull-pointer-arithmetic`, etc. Neither equals the GCC group of the same name.
- Claim 2: `-Wconversion` in Clang is a group of many (`-Wsign-conversion`, `-Wfloat-conversion`, `-Wimplicit-int-conversion`, `-Wimplicit-float-conversion`, `-Wliteral-conversion`...), and `-Wshadow-all` group includes `-Wshadow`, `-Wshadow-field`, `-Wshadow-field-in-constructor`, `-Wshadow-uncaptured-local`. Clang's `-Wnull-dereference` entry on that page lists only two messages (binding a dereferenced null pointer to a reference; indirection of a non-volatile null pointer), marked as on by default; the page does not describe the GCC path-based behaviour.
- Bears on this repository: since only GCC is built, Clang-only names (`-Wshadow-all`, `-Wmost`) must not be put in the shared flags list; they matter only if `clang-tidy` is run from a compile-commands file, where GCC-only flags (`-Wlogical-op`, `-Wduplicated-cond`) are not in the Clang list on that page, so the flags passed to clang-tidy should be checked (this behaviour was not tested).

### OpenSSF "Compiler Options Hardening Guide for C and C++"
- Citation: OpenSSF Best Practices WG, "Compiler Options Hardening Guide for C and C++", current version. https://best.openssf.org/Compiler-Hardening-Guides/Compiler-Options-Hardening-Guide-for-C-and-C++.html
- Kind: primary
- Read: 2026-11-25, in part (summary table, -Wall/-Wextra, -Wformat, -Wconversion, -Werror, brown-field advice)
- Topics: i
- Claim 1: Recommended warning set: `-Wall -Wextra -Wformat -Wformat=2 -Wconversion -Wsign-conversion -Wimplicit-fallthrough -Wtrampolines -Wbidi-chars=any`, with `-Werror=format-security` and `-Werror=implicit -Werror=incompatible-pointer-types -Werror=int-conversion`. It notes Clang's `-Wformat=2` does not include `-Wformat`, so give both.
- Claim 2: On `-Werror`: a blanket `-Werror` is for development only and should not be in distributed source because it ties the build to compiler vendor and version; the selective `-Werror=<flag>` form is fine for distribution.
  Quote: "-Werror should not be used in this blanket form when distributing source code, as this use of -Werror creates a dependency on specific toolchain vendors and versions."
- Claim 3: For `-Wconversion` on an existing code base the guide gives a staged rollout (baseline without `-Werror`, suppress benign patterns, triage high-risk ones, then warning to error to CI), and says new code should enable it from day one.
- Bears on this repository: gives the justification for a staged rule "`-Wconversion` is on as warning with a count in CI that may not rise" instead of `-Werror`, and for pinning exact `-Werror=<name>` lists while keeping blanket `-Werror` to CI only (via a `--config=ci`).

### MaskRay, "Layering check with Clang"
- Citation: Fangrui Song (MaskRay), "Layering check with Clang", 2022 (updated 2023-07). https://maskray.me/blog/2022-09-25-layering-check-with-clang
- Kind: secondary
- Read: 2026-11-25, whole
- Topics: b
- Claim 1: `layering_check` is built on Clang's `-fmodules-decluse` over module maps that Bazel generates: an include of a header from a module not declared in `use` is an error. `parse_headers` compiles each non-textual header in `srcs`/`hdrs` as a main file with `-fsyntax-only`, so every header is self-contained.
  Quote: "The feature parse_headers ensures that header files are self-contained."
- Claim 2: With both on, the header compiles done for `parse_headers` are also layering-checked; llvm-project's Bazel build sets `features = ["layering_check"]` for `llvm/`, `clang/` and `mlir/`.
- Bears on this repository: both features need a Clang toolchain, absent here; a self-containment check under GCC can be done by a generated test that compiles each header alone (`#include "x.h"` and nothing else) using `-fsyntax-only`, a yes-or-no per header.

## Does not fit

- Bazel "Best Practices" / "BUILD Style Guide", "a BUILD file in every directory with a package per directory": at 99 files in a flat `src/` this would give either 99 tiny packages or a cut that first needs directories; a handful of layer packages (core, perception, planning, control, cell, tools) is the proportionate version, and file moves are a refactoring that the "without breaking behavior" constraint makes risky for LLM contributors.
- Bazel "C / C++ Rules", `layering_check` and `parse_headers`: they run only on a Clang toolchain, and the repository has no Clang toolchain; DWYU (preprocessor-only) is the substitute that works with GCC.
- Abseil `copts.py` Clang list (`-Wshorten-64-to-32`, `-Wshadow-all`, `-Wthread-safety`, `-Wmost`): Clang-only names; GCC rejects `-Wshadow-all` etc. as unrecognized options, so they cannot be in the shared list.
- OpenSSF hardening guide, blanket `-Werror` "during development": with one owner and LLM contributors on GCC 13 (Ubuntu 24.04 in CI) a blanket `-Werror` breaks when the compiler or OMPL/Boost versions change, because optimizer-dependent warnings (`-Wmaybe-uninitialized`, `-Warray-bounds`, `-Wstringop-overflow`) appear in third-party templates. Drake, MuJoCo and MoveIt all switch these off or do not use `-Werror` at all.
- `bazel_clang_tidy` / Hedron extractor with a "Clang toolchain": both assume clang tooling reads the compile flags; with GCC-only flags clang-tidy needs the flags filtered, so these are extra work, not a drop-in.

## Leads not followed

- Bazel "Dependencies" concepts page (https://bazel.build/concepts/dependencies) and "C++ and Bazel" (rules_cc docs): not opened; the same material was covered by the c-cpp reference and style guide.
- `rules_cc` source of the `layering_check` and `parse_headers` features (`cc_toolchain_config_lib`/`unix_cc_toolchain_config.bzl`): not read; the semantic claim rests on the Bazel reference and MaskRay.
- Eigen's CMake warning options: the GitHub mirror paths (`cmake/EigenTesting.cmake`, `CMakeLists.txt` on `libeigen/eigen`) returned 404; Eigen is hosted on GitLab and was not tried.
- `ament_cmake_ros` / `ament_cmake` default compile options: the files fetched (`ament_package.cmake`) contain no warning flags; the ROS 2 default `-Wall -Wextra -Wpedantic` lives in `ament_cmake_ros` or per-package CMake, not located; MoveIt's file was used instead.
- `depend_on_what_you_use` docs (`docs/project/known_limitations.md`, aspect user guide), `bazel_clang_tidy` forks, `gazelle_cc`: not read.
- GCC 13 manual and Bugzilla reports on `-Wmaybe-uninitialized` / `-Wstringop-overflow` false positives (GCC 14.2 manual used; GCC 13 differences not checked).
- Clang manual on `-Wuninitialized` and optimization independence: not read.
