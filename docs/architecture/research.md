# Research for the design principles

This is the research behind the repository's design principles: what is known, from
primary sources and from three comparable code bases, about how software of this kind and
size is divided, built, tested, documented and kept in shape. Every source was read on
2026-11-25 by a reader that fetched it that day and took notes, and this document was
written from those notes. Afterwards, on the same day, checkers that had written none of
it opened all 59 sources again and compared every claim and every quotation with the
source; 29 passages were corrected from what they found. A source is primary when it is
the author's own text, a project's own documentation or its source code; a summary of
someone else's work is secondary. Sources are cited as `[S7]` and listed under Sources
with what was read of each. There are 59 sources, 58 of them primary. Each topic section
says what the sources say and ends with what follows for this repository; those paragraphs
state implications, not decisions, which belong to the principles and the decision
records. Facts about this repository are those of the base commit `940a01b`, each with the
command that printed it.

## Sources

- [S1] Adrian Colyer, "On the criteria to be used in decomposing systems into modules" (review of D. L. Parnas, CACM 1972). https://blog.acolyer.org/2016/09/05/on-the-criteria-to-be-used-in-decomposing-systems-into-modules/
  Secondary. Read 2026-11-25, in part (the passages quoting the paper).
  Parnas splits a system by design decisions likely to change: "Every module in the second
  decomposition is characterized by its knowledge of a design decision which it hides from
  all others."
- [S2] Robert C. Martin, "Granularity", C++ Report column, Object Mentor PDF (archived). https://web.archive.org/web/2007/http://www.objectmentor.com/resources/articles/granularity.pdf
  Primary. Read 2026-11-25, in part (the Acyclic Dependencies Principle, text extracted).
  Package dependencies must form a directed acyclic graph, since a cycle binds its members
  into one release: "Thus the dependency structure must always be monitored for cycles."
- [S3] Winters, Manshreck, Wright (eds.), "Software Engineering at Google", ch. 18, 2020. https://abseil.io/resources/swe-book/html/ch18.html
  Primary. Read 2026-11-25, in part (strict transitive dependencies, visibility).
  An undeclared use of a dependency's dependency fails the build "with an error and a
  shell command that can be used to automatically insert the dependency."; visibility is
  kept as narrow as possible.
- [S4] Bazel project, "Visibility". https://bazel.build/concepts/visibility
  Primary. Read 2026-11-25, in part (target visibility, specifications, package_group).
  Visibility lists allowed packages and is checked at analysis; a target without one is
  private to its package, so within one package every target sees every other.
- [S5] Bazel project, "BUILD Style Guide". https://bazel.build/build/style-guide
  Primary. Read 2026-11-25, in part (naming, visibility, dependencies, globs).
  "Avoid setting package default_visibility to //visibility:public."; `deps` holds direct
  dependencies only; a BUILD file in each directory, with a dependency graph between them.
- [S6] Bazel project, "C / C++ Rules" (Build Encyclopedia). https://bazel.build/reference/be/c-cpp
  Primary. Read 2026-11-25, in part (hdrs and srcs, layering_check, implementation_deps).
  `hdrs` against `srcs` is the choice of what dependents may include; for
  `layering_check`, "The toolchains provided by Bazel only support this feature with clang
  on Unix and macOS."
- [S7] Martin Medler, "Depend on what you use (DWYU)", README. https://raw.githubusercontent.com/martis42/depend_on_what_you_use/main/README.md
  Primary. Read 2026-11-25, whole.
  An aspect that checks a `cc_*` target's includes against its `deps`, finds unused deps,
  runs only the preprocessor and is not limited to LLVM toolchains.
- [S8] Erenon, "bazel_clang_tidy", README. https://raw.githubusercontent.com/erenon/bazel_clang_tidy/master/README.md
  Primary. Read 2026-11-25, whole.
  Runs clang-tidy per target as an aspect, "without generating a compile commands
  database"; it offers no pinned release.
- [S9] Vanthienen, Klotzbuecher, Bruyninckx, "The 5C-based architectural Composition Pattern", JOSER 5(1), 2014. https://aisberg.unibg.it/bitstream/10446/87662/1/78-379-1-PB.pdf
  Primary. Read 2026-11-25, in part (abstract, 1.1, sections 4 to 8 opened, conclusions).
  Communication, computation, coordination and configuration "should be kept fully
  separated"; "Configuration influences the behavior of entities of the other concerns by
  changing its settings."
- [S10] Foote, Purvis, "REP 103: Standard Units of Measure and Coordinate Conventions" (source file). https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst
  Primary. Read 2026-11-25, in part (rationale, units, frames, suffix frames).
  SI units, radians, right-handed frames; another convention is carried by a named suffix,
  and "choosing different conventions should be well justified and well documented."
- [S11] MuJoCo documentation, "Computation", section "Reproducibility". https://mujoco.readthedocs.io/en/stable/computation/index.html
  Primary. Read 2026-11-25, in part (section Reproducibility).
  Deterministic given the full state, but "Exact reproducibility is only guaranteed within
  a single version, on the same architecture."
- [S12] ISO C++ working draft, [rand.predef] and [rand.dist.general]. https://eel.is/c++draft/rand.dist.general
  Primary. Read 2026-11-25, in part (required behaviour; clause 3). Also https://eel.is/c++draft/rand.predef
  Engines such as `mt19937` have fixed outputs; "The algorithms for producing each of the
  specified distributions are implementation-defined."
- [S13] ros2_control, "Mock Components" (Rolling). https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html
  Primary. Read 2026-11-25, in part (introduction, the generic system).
  A mock takes the real hardware's place in the same configuration, chosen in the robot's
  description: "They provide ideal behavior by mirroring commands to their states."
- [S14] GNU Project, GCC manual, Optimize Options, `-ffp-contract`. https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
  Primary. Read 2026-11-25, in part (the `-ffp-contract` entry).
  Fused multiply-add is formed where the target supports it; "The default is
  -ffp-contract=off for C in a standards compliant mode (-std=c11 or similar),
  -ffp-contract=fast otherwise."
- [S15] Michael Feathers, "Characterization Testing", 2016-08-08. https://michaelfeathers.silvrback.com/characterization-testing
  Primary. Read 2026-11-25, whole.
  "The purpose of characterization testing is to document your system's actual behavior,
  not check for the behavior you wish your system had."; most useful before a refactoring.
- [S16] Martin Fowler, "Refactoring", refactoring.com. https://refactoring.com/
  Primary. Read 2026-11-25, in part (the opening section).
  Refactoring is "altering its internal structure without changing its external
  behavior." in small steps; the page does not say what that means for numeric output.
- [S17] ApprovalTests.cpp, "Custom Comparators" and "ApprovalTesting (the concept)". https://raw.githubusercontent.com/approvals/ApprovalTests.cpp/master/doc/CustomComparators.md
  Primary. Read 2026-11-25, in part (both short pages).
  The default comparator "reads both files, one character at a time, and tests if their
  contents are identical."; its own example of a custom one is output that varies by
  machine.
- [S18] Bazel project, "Test Encyclopedia". https://bazel.build/reference/test-encyclopedia
  Primary. Read 2026-11-25, in part (size and timeout).
  A test's `size` implies its timeout; an unstated size is `medium`, and
  `--test_verbose_timeout_warnings` names tests whose size is too large.
- [S19] Michael Nygard, "Documenting Architecture Decisions", 2011-11-15. https://www.cognitect.com/blog/2011/11/15/documenting-architecture-decisions
  Primary. Read 2026-11-25, whole.
  Title, context, decision, status, consequences, one decision per record; "ADRs will be
  numbered sequentially and monotonically. Numbers will not be reused." Experience only.
- [S20] Martin Fowler, "Architecture Decision Record", 2026-03-24. https://martinfowler.com/bliki/ArchitectureDecisionRecord.html
  Primary. Read 2026-11-25, whole.
  "Once an ADR is accepted, it should never be reopened or changed - instead it should be
  superseded."; one short numbered file each, with real alternatives and consequences.
- [S21] ADR organization, MADR 4.0.0 template. https://raw.githubusercontent.com/adr/madr/4.0.0/template/adr-template.md
  Primary. Read 2026-11-25, whole.
  An optional Confirmation section: "Describe how the implementation of/compliance with
  the ADR can/will be confirmed."
- [S22] Olaf Zimmermann, "How to create Architectural Decision Records (ADRs) — and how not to", 2023-04-03. https://ozimmer.ch/practices/2023/04/03/ADRCreation.html
  Primary. Read 2026-11-25, whole.
  Named failures, among them Fairy Tale, Free Lunch Coupon, Dummy Alternative and the
  Mega-ADR, detail "stuffed into several multi-page ADRs serving as documentation master".
- [S23] Hunt and Thomas, "The Pragmatic Programmer", 20th anniversary ed., Topic 9 "DRY" (sample). https://media.pragprog.com/titles/tpp20/dry.pdf
  Primary. Read 2026-11-25, in part (the passages that came out of the PDF intact).
  "Every piece of knowledge must have a single, unambiguous, authoritative representation
  within a system."; it applies to comments and documents as well as code.
- [S24] Google, "Documentation Best Practices" (docguide). https://google.github.io/styleguide/docguide/best_practices.html
  Primary. Read 2026-11-25, whole.
  Update documents in the same change as the code, delete dead ones, link rather than
  duplicate; a small set of fresh documents beats a large stale one.
- [S25] Daniele Procida, "Diátaxis" and "Start here". https://diataxis.fr/
  Primary. Read 2026-11-25, whole (both pages).
  Tutorials, how-to guides, reference and explanation; "Crossing or blurring the
  boundaries described in the map is at the heart of a vast number of problems in
  documentation."
- [S26] TNG, "ArchUnit User Guide". https://www.archunit.org/userguide/html/000_Index.html
  Primary. Read 2026-11-25, in part (cycle checks, layer checks, freezing).
  Rules as tests, with known violations frozen: "If violations are fixed, FreezingArchRule
  will automatically reduce the known stored violations to prevent any regression."
- [S27] qntm, "Ratchets in software development", 2021-11-21. https://qntm.org/ratchet
  Primary. Read 2026-11-25, whole.
  A script counts a banned pattern; more fails, and "If it counts too few, it also raises
  an error, this time congratulating you and prompting you to lower the expected number."
- [S28] Gloaguen et al., "Evaluating AGENTS.md", arXiv:2602.11988v3, 2026. https://arxiv.org/abs/2602.11988
  Primary. Read 2026-11-25, in part (abstract, setup, results, analysis, limitations).
  Four agents, Python repositories: "providing context files does not generally improve
  task success rates, while increasing inference cost by over 20% on average".
- [S29] Anthropic, "How Claude remembers your project" (Claude Code docs). https://code.claude.com/docs/en/memory
  Primary (vendor guidance, no data). Read 2026-11-25, in part (instructions, size).
  The file is context, not enforcement; "The more specific and concise your instructions,
  the more consistently Claude follows them."; under 200 lines.
- [S30] Anthropic, "Best practices for Claude Code". https://code.claude.com/docs/en/best-practices
  Primary (vendor guidance). Read 2026-11-25, in part (length, verification, hooks).
  "Claude reads the repo itself. Only write down what isn’t in there."; a rule with no
  exceptions belongs in a hook, a deny rule or a managed setting.
- [S31] "CodeTaste: Can LLMs Generate Human-Level Code Refactorings?", arXiv:2603.04177, 2026. https://arxiv.org/abs/2603.04177
  Primary. Read 2026-11-25, in part (abstract, introduction, benchmark, results).
  "agents perform well at implementing refactorings that are specified in detail, but
  often fail to discover the human refactoring choices"
- [S32] "A Differential Fuzzing-Based Evaluation of Functional Equivalence in LLM-Generated Code Refactorings", arXiv:2602.15761. https://arxiv.org/abs/2602.15761
  Primary. Read 2026-11-25, in part (abstract only).
  19 to 35 percent of refactorings were not equivalent, about a fifth of those passed the
  tests: "reliance on existing tests might overestimate functional equivalence".
- [S33] GNU Project, "Options to Request or Suppress Warnings", GCC 14.2.0 manual. https://gcc.gnu.org/onlinedocs/gcc-14.2.0/gcc/Warning-Options.html
  Primary. Read 2026-11-25, in part (-Werror, -Wall, -Wextra and about twenty entries).
  What `-Wall` and `-Wextra` hold and leave out; "The precision of the warnings depends on
  the optimization options used."
- [S34] OpenSSF, "Compiler Options Hardening Guide for C and C++". https://best.openssf.org/Compiler-Hardening-Guides/Compiler-Options-Hardening-Guide-for-C-and-C++.html
  Primary. Read 2026-11-25, in part (summary table, warnings, -Werror, existing code).
  "-Werror should not be used in this blanket form when distributing source code"; for
  `-Wconversion` on old code, a staged rollout from a counted baseline.
- [S35] LLVM project, "Diagnostic flags in Clang". https://clang.llvm.org/docs/DiagnosticsReference.html
  Primary. Read 2026-11-25, in part (-Wall, -Wmost, -Wextra, -Wconversion, -Wshadow-all).
  Clang's `-Wall` and `-Wextra` differ from GCC's groups; some names exist in one compiler
  only (`-Wshadow-all`, `-Wmost` in Clang; `-Wlogical-op` is not on the page).
- [S36] LLVM project, "Clang-Tidy" (index page). https://clang.llvm.org/extra/clang-tidy/index.html
  Primary. Read 2026-11-25, in part (options, NOLINT, compilation database, scripts).
  `WarningsAsErrors` promotes only enabled checks; `NOLINT` can name its check; `-p` reads
  `compile_commands.json` and `RemovedArgs` drops flags clang does not know.
- [S37] LLVM project, "bugprone-narrowing-conversions". https://clang.llvm.org/extra/clang-tidy/checks/bugprone/narrowing-conversions.html
  Primary. Read 2026-11-25, whole.
  Finds int, float and double narrowing beyond `-Wconversion`; by default not inside
  template instantiations (the Eigen remark is this document's inference; the page does
  not mention Eigen).
- [S38] GCC project, "Static Analyzer Options" (14.2) and GCC 13, 14, 15 release notes. https://gcc.gnu.org/onlinedocs/gcc-14.2.0/gcc/Static-Analyzer-Options.html and https://gcc.gnu.org/gcc-14/changes.html
  Primary. Read 2026-11-25, in part (the `-fanalyzer` sentences; the quotation is from the
  release notes, in GCC 13, 14 and 15, not on the 14.2 options page).
  "In particular, using it on C++ is unlikely to give meaningful output."
- [S39] GCC project, "Program Instrumentation Options", GCC 14.2 manual. https://gcc.gnu.org/onlinedocs/gcc-14.2.0/gcc/Instrumentation-Options.html
  Primary. Read 2026-11-25, in part (the sanitizer entries).
  ASan, TSan, LSan, UBSan; ASan and TSan do not combine; float checks are outside
  `undefined`, as "floating-point division by zero can be a legitimate way of obtaining
  infinities and NaNs."
- [S40] LLVM project, "MemorySanitizer". https://clang.llvm.org/docs/MemorySanitizer.html
  Primary. Read 2026-11-25, in part (key sentences checked).
  "MemorySanitizer requires that all program code is instrumented."; described for Clang.
- [S41] Chromium project, "clang-tidy" (docs/clang_tidy.md). https://chromium.googlesource.com/chromium/src/+/main/docs/clang_tidy.md?format=TEXT
  Primary. Read 2026-11-25, whole.
  A new check needs a count and a sample of more than 30 reviewed findings; old code is
  reported on touched lines only: "making Chromium clang-tidy clean is an explicit
  non-goal".
- [S42] gRPC project, `.clang-tidy`, master. https://raw.githubusercontent.com/grpc/grpc/master/.clang-tidy
  Primary. Read 2026-11-25, in part (the comment block and the check list).
  Each check disabled on purpose in the comment block has a reason; the checks to enable
  later (numeric ones among them) share one: "it's not activated yet due to the existing
  issues".
- [S43] Titus Winters, "Dependency Management", Software Engineering at Google, ch. 21. https://abseil.io/resources/swe-book/html/ch21.html
  Primary. Read 2026-11-25, in part (opening, diamond dependencies, SemVer, pinning).
  "Just because you get to avoid a development cost doesn’t mean importing a dependency is
  the correct choice."; diamonds are a core failure, and C++ bears them badly.
- [S44] Bazel project, "Bazel Modules: Lockfile". https://bazel.build/external/lockfile
  Primary. Read 2026-11-25, whole.
  `--lockfile_mode=error` fails on a missing or stale lockfile; commit it, with
  `.bazelversion`, since it is tied to the Bazel version.
- [S45] include-what-you-use, "Why include what you use?". https://raw.githubusercontent.com/include-what-you-use/include-what-you-use/master/docs/WhyIWYU.md
  Primary. Read 2026-11-25, whole.
  If every file includes what it uses, an include can be removed without breaking distant
  code.
- [S46] FSFE, "REUSE Specification, version 3.3". https://reuse.software/spec-3.3/
  Primary. Read 2026-11-25, in part (headers, LICENSES directory).
  An SPDX identifier per file, and "Each License File MUST be placed in the LICENSES/
  directory in the root of the Project."
- [S47] Drake, tools/skylark/drake_cc.bzl at commit 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/skylark/drake_cc.bzl
  Primary. Read 2026-11-25, in part (lines 1-330 and 543-936).
  A list of `-Werror=` flags for all C++ rules, each GCC opt-out commented ("This falsely
  dings code inside Eigen."); tests default to `small` under `test/`.
- [S48] Drake, doc/_pages/directory_structure.md at commit 5186757f. https://github.com/RobotLocomotion/drake/blob/master/doc/_pages/directory_structure.md
  Primary. Read 2026-11-25, whole.
  The direction between areas in prose (`systems` uses `solvers`, never the reverse);
  experimental code may be used only by experimental code.
- [S49] Drake, math/BUILD.bazel at commit 5186757f. https://github.com/RobotLocomotion/drake/blob/master/math/BUILD.bazel
  Primary. Read 2026-11-25, in part (lines 1-30, 95-120, 355-370 and counts).
  22 libraries, 33 tests, and an exception to the layering with a TODO: "Move this code
  into //math to avoid the circular dependency."
- [S50] Drake, .clang-tidy at commit 5186757f. https://github.com/RobotLocomotion/drake/blob/master/.clang-tidy
  Primary. Read 2026-11-25, whole.
  Broad check groups under the first line "This file is not used by CI and the checks
  included are not part of the Drake style guide".
- [S51] Drake, tools/bazel-developer.rc at commit 5186757f. https://github.com/RobotLocomotion/drake/blob/master/tools/bazel-developer.rc
  Primary. Read 2026-11-25, in part (lines 1-175).
  "we enable -Werror by default for Drake's *own* targets, but not for our externals.";
  sanitizers are named configs with scaled timeouts.
- [S52] Drake, doc/_pages/developers.md and code_review_checklist.md at commit 5186757f. https://github.com/RobotLocomotion/drake/blob/master/doc/_pages/developers.md
  Primary. Read 2026-11-25, in part (developers.md outside IDE lists, checklist lines
  25-117).
  Two review phases, the second by one of nine named owners, and a cap on change size;
  `Eigen::Random` and libc `rand` are forbidden in code and tests.
- [S53] MuJoCo, cmake/MujocoOptions.cmake at commit 72a06685. https://github.com/google-deepmind/mujoco/blob/main/cmake/MujocoOptions.cmake
  Primary. Read 2026-11-25, whole.
  `-Werror -Wall -Wpedantic -Wimplicit-fallthrough -Wunused -Wvla` and three opt-outs; GCC
  adds `-Wno-maybe-uninitialized`; symbols hidden unless exported.
- [S54] MuJoCo, src/engine/BUILD.bazel at commit 72a06685. https://github.com/google-deepmind/mujoco/blob/main/src/engine/BUILD.bazel
  Primary. Read 2026-11-25, in part (lines 1 to about 120, a scan of the rest).
  Private by default to a `package_group` of five trees; one `cc_library` per one to three
  C files; its options (`-Wshadow`, `-ffp-contract=off`) differ from the CMake build's.
- [S55] MuJoCo, test/fixture.h and test/engine/engine_forward_test.cc at commit 72a06685. https://github.com/google-deepmind/mujoco/blob/main/test/fixture.h
  Primary. Read 2026-11-25, in part (helpers of fixture.h; tolerance uses in the test).
  Tolerances per precision, `MjTol(double_tol, float_tol)`: "Setting MJTOL_SCALE=0 scales
  tolerances to zero" shows the residual; no stored expected outputs were found.
- [S56] MuJoCo, STYLEGUIDE.md at commit 72a06685. https://github.com/google-deepmind/mujoco/blob/main/STYLEGUIDE.md
  Primary. Read 2026-11-25, in part (first 120 of 189 lines).
  "Maximise consistency with the rest of the code."; where guide and code disagree, the
  guide wins and grows.
- [S57] MoveIt 2, AGENTS.md at commit 8c99e523. https://github.com/moveit/moveit2/blob/main/AGENTS.md
  Primary. Read 2026-11-25, whole.
  "The layers depend in one direction, from bottom to top"; plugins load at run time;
  "Test the contract, not the implementation."; CI uses `-Werror` and clang-tidy on diffs.
- [S58] MoveIt 2, .clang-tidy at commit 8c99e523. https://github.com/moveit/moveit2/blob/main/.clang-tidy
  Primary. Read 2026-11-25, whole.
  An allow-list from `-*`: performance, some modernize and readability checks; no
  `bugprone-*` and no `WarningsAsErrors`.
- [S59] MoveIt 2, moveit_common/cmake/moveit_package.cmake at commit 8c99e523. https://github.com/moveit/moveit2/blob/main/moveit_common/cmake/moveit_package.cmake
  Primary. Read 2026-11-25, whole.
  `-Wall -Wextra` and seven more for every package, no `-Werror`, and GCC's
  `-Wno-maybe-uninitialized`: "This too often has false-positives".

## (a) Boundaries between modules and the direction of dependencies

Parnas, as quoted in a review, divides a system by the decisions likely to change, each
module hiding one, rather than by the steps of processing [S1]. Martin requires package
dependencies to form a directed acyclic graph and to be watched for cycles [S2]. Google's
build makes an undeclared use an error, so a target's contract is what it names [S3].
Drake states its direction between areas in prose [S48] and keeps a known exception with
a TODO in the build file [S49]; MoveIt writes "The layers depend in one direction" in its
file for agents [S57]; MuJoCo names in one `package_group` the trees that may see its
engine's internals [S54]. The sources agree on acyclic and one-way; they differ on
enforcement, prose with exceptions in Drake, a build error in MuJoCo.

**What this means here.** Direction can be checked without moving a file:
`tools/architecture/deps.py` checks every `#include` of `src/` and `tests/` against the
ordered groups of `tools/architecture/deps.json`, each allowed departure named, as Drake
names its exception [S49]. The code fits a one-way rule already: `src/rrt.h` includes no
project header (`grep -n '#include "src' src/rrt.h` prints nothing), no stand-in file
includes a header of the new cell, and the one include from the new cell into the
planner's helpers is `src/layout_plan.h:137`, which includes `src/joint_move.h`. Parnas's
test fits outside libraries: OMPL is hidden (`grep -l '#include <ompl' src/* tests/*`
prints only `src/ompl_search.cc`), MuJoCo is not (`grep -l 'mujoco/mujoco.h' src/*.h`
names 17 headers), and the second is a choice to state, not a fault. The stand-in cell is
the decision most likely to change, so the rule that matters most is that nothing outside
it depends on it. A cycle check over includes fits in the same script.

## (b) How a Bazel C++ project of this size is cut into packages and targets, and how visibility and header checks keep the cut

Visibility is checked between packages, and a target with none is private only to its
package, so in one package every target sees every other [S4]. The style guide asks for
narrow visibility, no public default, direct `deps` only and a BUILD file per directory
[S5]. `hdrs` against `srcs` decides what dependents may include, and `layering_check`
holds includes to direct deps only with Clang in Bazel's own toolchains [S6]. DWYU checks
the same with the preprocessor and also finds unused deps [S7]; clang-tidy can run as an
aspect per target [S8]. Drake cuts much finer than the guide asks, one library per header
with a rollup and a `test/` directory per package [S47][S49]; MuJoCo shows that one
`package_group` over a few trees is enough for a public and a private part [S54].

**What this means here.** All code is one package:
`bazel query 'kind(rule, //...)' | wc -l` prints 110 and `grep -c visibility BUILD.bazel`
prints 0, so visibility keeps no cut today [S4]. The one file is a decision of its own:
`AGENTS.md` makes `BUILD.bazel` the one shared file with a section per piece, and
`docs/ompl-integration.md:1391` already weighed a package with `visibility` against that
rule. `layering_check` does not apply:
`grep -n -i 'clang\|toolchain' .bazelrc MODULE.bazel BUILD.bazel` prints nothing [S6].
What works without a toolchain is comparing each target's includes with its `deps`, which
`tools/architecture/measure.sh` already counts [S7]. A split into packages would turn
visibility into a build error, as in MuJoCo [S54]; until then a script holds the cut.

## (c) What is particular to robotics

The 5C paper keeps computation, coordination, configuration and communication apart, with
configuration as settings other parts read, and warns that imposing a whole architecture
early is "a recipe for failure in the long term" [S9]. ros2_control puts a mock in the
place of the real hardware, chosen in the robot's description, so controllers run
unchanged against either [S13]. REP 103 fixes SI units, radians and right-handed frames
and asks that another convention be well justified and well documented [S10]. Repeatable
runs have three enemies: MuJoCo is exact only within one version on one architecture
[S11]; the standard pins engines such as `mt19937` but leaves distributions to the library
[S12]; GCC may fuse multiply and add where the target supports it, and the default depends
on the language mode [S14]. MuJoCo builds its engine with `-ffp-contract=off` [S54]; Drake
forbids `Eigen::Random` and libc `rand` [S52].

**What this means here.** `src/portable_random.h` answers the standard's gap, yet
`src/rrt.cc:442` still draws through `std::uniform_real_distribution`, and the committed
plans rest on that planner; a draw rule can be a grep for `_distribution` under `src/`
with named exceptions [S12]. The line between the simulated world and control is the
`True...` accessors of `src/cell.h` and `src/conveyor.h`, which `AGENTS.md` keeps to the
scene, the scorer and tests; the names are greppable, but any file that includes
`src/cell.h` can read the whole simulation, so a seam like ros2_control's [S13] is work
for the new cell, not the stand-in. A cell as data is already the rule of `src/layout.h`:
a file in `layouts/*.json` is the one source of model and checks, configuration kept apart
from computation as [S9] asks. `AGENTS.md` says
metres and radians while `base_rpy_deg` in `src/layout.h:195` carries degrees from the
JSON into the checker, which REP 103 allows if it is well justified and well documented
[S10]. One bound mixes units: `src/path_timing.h:138-141` gives `max_deviation` in
radians, and `src/layout_plan.h:145-146` passes six joints in radians and a track in
metres to it. `.bazelrc` sets no `-ffp-contract` [S14].

## (d) Tests

Characterization tests record what the code does now, so a refactoring that changes it
fails at once, and they stay until someone decides the behaviour is a bug [S15].
Refactoring keeps "external behavior" [S16], which for numbers the project must define.
Golden-master tools compare character by character by default [S17], which MuJoCo's own
documentation undercuts for simulation output [S11]; MuJoCo's tests compare with a
tolerance per precision, calibrated by one variable [S55], and MoveIt asks for explicit
tolerances and tests of the contract [S57]. Bazel gives each test a size that sets its
timeout, `medium` when unstated [S18]; Drake defaults tests to `small` under each
package's `test/` [S47]. The sources disagree on exact golden files: one tool's default
[S17] is what a simulator's documentation shows to be fragile across versions and
systems [S11].

**What this means here.** No test and no CI step runs a program: `bazel query
'kind(cc_binary, deps(tests(//...))) intersect //:*'` prints nothing, and
`.github/workflows/ci.yml` runs only `bazel test`, so the tables of `run_cell`,
`print_plan` and `check_layout` are pinned by nothing, and capturing them is the first
step before restructuring [S15]. `src/check_placement.cc` is compiled by no test (`bazel
query "kind('source file', deps(tests(//...)))" --output=label | grep -c check_placement`
prints 0). `tests/bay_model_test.cc:97` compares `models/bay.xml` byte for byte with its
generator, which holds while the generator's arithmetic prints the same text on ARM and
x86-64; it is where [S11] and [S14] would show first. `tests/plan_replay_test.cc`
verifies a committed plan with tolerances and never compares a fresh plan
(`src/layout_plan.h:118-121`), the design the sources favour [S55]. A capture of program
output should compare numbers with stated tolerances (a custom comparator allows it [S17])
and leave out columns of time.

## (e) Decision records

Nygard's record has title, context, decision, status and consequences, one decision per
record, numbers never reused, kept with the code [S19]. Fowler adds the lifecycle:
proposed, accepted, superseded, and an accepted record is never edited but superseded by
a new one [S20]. MADR's optional Confirmation section says how compliance is checked,
for example by an architecture test [S21]. Zimmermann names what goes wrong: only pros,
no real consequences, invented alternatives, a policy written as a record, and the
Mega-ADR that becomes the documentation [S22]. They differ on old records: Fowler keeps
and supersedes [S20]; Zimmermann allows outdated ones to be deleted [S22]. None of this
is measured; Nygard reports a few months of experience [S19].

**What this means here.** The repository keeps its decisions inside design documents:
"Decisions" sections at `docs/layout.md:720` and `docs/flat-rack-cell.md:546`, and twelve
recommendations for the owner at `docs/ompl-integration.md:1380`, none recorded as
decided. A short record per standing decision, with a Confirmation that names a command
[S21], suits contributors who are agents: a record they can check is one they can keep.
The Mega-ADR warning [S22] argues for records that point to those sections rather than
copy them. Where no reason was ever written, as for Bazel, MuJoCo and C++20 since the
first commits, the record should say so; an alternative invented afterwards is the Dummy
Alternative [S22]. Fowler's rule that an accepted record is not edited [S20] is
checkable on a diff.

## (f) How a repository's documents are organized so that each fact has one home

DRY is about knowledge, comments and documents included: "Every piece of knowledge must
have a single, unambiguous, authoritative representation within a system." [S23].
Google's guide asks that documents change in the same commit as the code they describe,
that dead ones be deleted, and that a document link rather than repeat [S24]. Diátaxis
holds that mixing tutorial, how-to, reference and explanation is at the heart of a vast
number of documentation problems [S25]. MuJoCo settles conflicts by letting its style
guide win over the code [S56]. All of this rests on experience and testimonials, not
measurement.

**What this means here.** Facts written more than once can be counted. The stand-in
module's size, 2.3 by 1.1 by 0.035 m, is written as constants in four headers,
`src/suction.h:45`, `src/pallet.h:27-29`, `src/conveyor.h:69-71` and
`src/point_generator.h:64-66`, none derived from another; `Fmt` is defined in four files
(`grep -ln '^std::string Fmt(' src/*.cc | wc -l` prints 4) and `SecondsSince` in five
(`grep -ln '^double SecondsSince(' src/*.cc | wc -l` prints 5). The real hardware has one
home, `src/bay_hardware.h`, from which `models/bay.xml` is generated, as DRY asks [S23].
Documents drift as [S24] warns: `docs/layout.md:667` still tells the reader to run a
program whose source, `src/print_layout_moves.cc` (gone), was removed. For the stand-in
cell, which is to be replaced, a count of copies that may not grow may be enough.

## (g) How principles are kept true by machine

ArchUnit writes architecture rules, layers and cycles among them, as ordinary tests, and
freezes the violations that exist so that only new ones fail and fixed ones cannot return
[S26]. A ratchet does the same with a plain script: it counts a pattern and fails both
when the count rises and when it falls, so the expected number is lowered by hand [S27].
Chromium applies this to clang-tidy by reporting on touched lines only [S41], and gRPC
lists in its configuration the checks deferred for existing findings, with one reason for
all [S42]. Drake's math/BUILD.bazel ends with a lint-test call [S49] but declares its
broad `.clang-tidy` unused by CI [S50]: a rule without a machine. The fitness-function
literature itself could not be read first-hand (see the last section).

**What this means here.** Part of the machine exists: `tools/architecture/deps.py`
checks the include direction of (a), and `tools/architecture/measure.sh` prints one count
per checkable rule, each a ratchet in the sense of [S27] once a baseline holds it. The
departures listed in `tools/architecture/deps.json` are ArchUnit's frozen store [S26]: a
departure fixed is removed and cannot return unnoticed. Rules of `AGENTS.md` that are
prose today become counts where a name makes them greppable, for example "Sensor data
only" as the list of files allowed to call a `True...` accessor. A rule that no command
can decide stays a yes-or-no question for review. A count that may only fall needs a
written reason to rise; that is what makes it a ratchet rather than a report [S27].

## (h) What makes a repository workable for coding agents, and what is known about instructions that agents follow or ignore

What was measured. Four agents on Python tasks, with no context file, a generated one or
the developers' own: success did not rise significantly, cost rose by over 20 percent,
agents followed the instructions, and repository overviews did not help them find files
[S28]. On 100 refactorings, agents did well when the refactoring was specified in detail
and poorly when left to find it, and the benchmark needed static checks beside tests to
see whether the intended change was made [S31]. Differential fuzzing found 19 to 35
percent of LLM refactorings of function-level and program-level code not equivalent, a
fifth of those unnoticed by the tests [S32]. None of the three measured C++ under Bazel.

What vendors recommend, without data. Instruction files are context, not enforcement;
specific and concise instructions are followed more consistently; stay under 200 lines;
a rule with no exceptions belongs in a hook, a deny rule or a managed setting [S29][S30].
MoveIt's file for agents states its layers and test rules as short imperatives [S57].
Measurements and vendors agree on short and specific, and part on overviews, which
generators write and the study found of no help [S28].

**What this means here.** `AGENTS.md` is short (`wc -l AGENTS.md` prints 124) and opens
with exact commands, as [S29] and [S30] advise; its last section, `## Map`, is a tour of
pieces, the kind of overview [S28] found no help in Python repositories, a result not
tested on a repository like this one. Its rules that a script can decide, such as "Sensor
data only", are better held by `tools/architecture/measure.sh` than by prose [S30].
Since no test runs a program (section d) and tests can pass over changed behaviour [S32],
"without breaking behavior" needs a capture of program output before and after, not only
`bazel test`. Work given to an agent should name each refactoring and a check that the
old pattern is gone, which is where [S31] found agents reliable.

## (i) Compiler warnings that find defects and those that only make noise

GCC's manual says what `-Wall` and `-Wextra` hold and leave out (`-Wshadow`,
`-Wconversion`, `-Wfloat-equal`, `-Wlogical-op`, `-Wduplicated-cond`, among others), and
that several warnings depend on the optimizer: `-Wmaybe-uninitialized` is wrong where
GCC cannot prove code correct, and `-Warray-bounds=2` gives more false positives [S33].
Clang's groups of the same names differ, and some names exist in one compiler only
[S35]. OpenSSF recommends `-Wconversion` and `-Wsign-conversion`, a selective
`-Werror=<flag>` rather than a blanket one when distributing source, and a counted
baseline for old code [S34]. Drake, MuJoCo and MoveIt all switch `-Wmaybe-uninitialized`
off under GCC [S47][S53][S59]; none turns on `-Wconversion`; Drake makes `-Wshadow` and
`-Wlogical-op` errors [S47], and MoveIt adds `-Wold-style-cast` and `-Wformat=2` as
warnings [S59]. They differ on `-Werror`: in MuJoCo's build [S53], for Drake's own targets
only [S51], in MoveIt's CI only [S57][S59].

**What this means here.** The repository sets no warning flag:
`grep -n 'Werror\|-Wall' .bazelrc BUILD.bazel` prints nothing and
`grep -c 'copts\|features' BUILD.bazel` prints 0. `.bazelrc` already scopes a flag to the
repository's own files with `--per_file_copt=^(src|tests)/.*\.cc$@-Wno-psabi`, and its
`opt` config turns MuJoCo's `-Werror` on `-Wmaybe-uninitialized` back into a warning,
the same optimizer-dependent warning the three projects switch off [S33]. Every build
here is optimized, so such warnings are for counting, not for errors. GCC-only names
(`-Wlogical-op`, `-Wduplicated-cond`) are safe while there is no Clang build, and
Clang-only names must stay out [S35]. Which flags become errors should follow from a
count of findings on this code, the clean ones first, `-Wconversion` through a counted
baseline [S34].

## (j) Static analysis and run-time checking

clang-tidy reads a compilation database, `WarningsAsErrors` promotes only checks already
on, and a `NOLINT` can name its check [S36]. `bugprone-narrowing-conversions` finds
narrowing beyond `-Wconversion` but skips template instantiations by default [S37]. gRPC
defers, among others, the numeric checks because of existing findings [S42], and Chromium
admits a check only with a count and reviewed findings, on touched lines only [S41]. The
robotics projects enforce little: Drake's `.clang-tidy` is not used by CI [S50], and
MoveIt's allow-list has style and performance checks, no `bugprone-*` and no
`WarningsAsErrors` [S58]. GCC's analyzer is, by GCC's own notes, unlikely to help on C++
[S38]. GCC has ASan, UBSan, LSan and TSan, with float division by zero outside
`-fsanitize=undefined` [S39]; MemorySanitizer needs every library instrumented [S40].
Drake keeps each sanitizer as a named config with longer timeouts [S51].

**What this means here.** The build uses GCC, so clang-tidy's input is the compilation
database that `tools/architecture/compdb.py` writes from Bazel's own commands, with
flags clang rejects removed through `RemovedArgs` [S36]. A short allow-list of defect
checks suits the numeric code of `src/layout_check.cc`, `src/ik.cc` and `src/rrt.cc`
better than style groups, each check entering with its count of findings and deferred
with a stated reason when it has many, as gRPC does with one shared reason [S42].
`-fanalyzer` is not a candidate for C++20 code [S38]. ASan and UBSan as named configs in
`.bazelrc` [S51] work with GCC [S39]; float division by zero stays off where NaN and
infinity are used on purpose. The slow tests listed in `AGENTS.md` would be slower still
under a sanitizer, so such a run belongs in a config run on demand, not in every agent's
loop.

## (k) A healthy dependency tree

Each dependency is a maintenance cost that security or platform changes can force, and a
diamond of incompatible versions is a failure C++ bears badly [S43]. Bazel's lockfile
records registry hashes, `--lockfile_mode=error` fails when it is stale, and it is
committed with `.bazelversion` [S44]. A file should include what it uses, so that
removing an include breaks nothing far away [S45]; DWYU checks the same at the level of
targets and finds deps listed but unused [S7]. REUSE puts licence texts in a LICENSES
directory and an SPDX identifier on each file [S46]. The code bases keep outside code
behind few targets: MuJoCo hides everything not exported [S53], and MoveIt keeps planner
and kinematics implementations out of its core, behind abstract interfaces [S57].

**What this means here.** Pinning is in place: each `bazel_dep` in `MODULE.bazel` names
a version, the robot models come file by file at a commit with a SHA-256 each
(`bazel/menagerie.bzl`, `bazel/kuka.bzl`), OMPL is an `http_archive` with a hash, and
`MODULE.bazel.lock` and `.bazelversion` are committed [S44]. The graph cannot be printed
whole (`bazel mod graph 2>&1 | tail -1` prints "ERROR: Results may be incomplete as 1
extension failed."), so a one-version check cannot rest on it. OMPL brings seventeen
Boost modules (`grep -c 'bazel_dep(name = "boost' MODULE.bazel` prints 17), a cost of the
kind [S43] asks to have written down. Declared and used differ a little:
`//:bay_model_test`, `//:print_plane_fit` and `//:print_points` list `@eigen`, yet
`grep -l '#include <Eigen' tests/bay_model_test.cc src/print_plane_fit.cc src/print_points.cc`
prints nothing [S7]. `tools/architecture/measure.sh` counts the targets that list each
outside library, which may not grow. No licence file is tracked
(`git ls-files | grep -i licen` prints nothing); for one owner a table of outside
libraries and their licences is the proportionate part of [S46].

## Three code bases, read in their source

### Drake

Read at commit `5186757f28d541f90428158c123fd84585e8eaee` of RobotLocomotion/drake.

- The tree: top-level areas (among them common, math, solvers, systems, geometry,
  multibody, planning, examples), with a written direction between them and experimental
  code kept apart [S48], and at least one exception marked by a TODO [S49].
- Targets and visibility: one library per header and a rollup per package, made through a
  macro layer; `internal = True` requires a non-public visibility [S47][S49].
- Tests: in each package's `test/` directory, named after the unit, `small` unless stated
  [S47].
- Decisions and style: pages under doc/_pages, design feedback in issues and two review
  phases; no directory of decision records was found [S48][S52].
- Warnings and analysis: a `-Werror=` list with each GCC opt-out commented [S47], errors
  for Drake's own targets only and sanitizers as named configs [S51], and a
  `.clang-tidy` that CI does not use [S50].

**Not to copy:** one library per header with a package rollup and the macro layer that
checks it [S47][S49]. It serves Drake's installed headers and downstream users; here one
owner works in one package, and the cost would come before any split.

**To copy in small:** one warning list with a reason on every disabled flag, errors for
the repository's own files only [S47][S51]; a written direction with named exceptions
[S48][S49]; randomness only from a seeded generator, in tests too [S52].

### MuJoCo

Read at commit `72a06685bffb8f3a39af0a1573fab1521dc94693` of google-deepmind/mujoco.

- The tree: public headers apart from the sources, the engine one flat directory grouped
  by file-name prefixes, the tests mirroring the sources (the reader's listing); the
  engine's build file names one library per one to three C files [S54].
- Targets and visibility: private by default, a `package_group` of the five trees that
  may see engine internals, one public aggregate [S54]; CMake hides unexported symbols
  [S53].
- Tests: GoogleTest per engine file, small models from XML, tolerances per precision with
  a calibration switch, no stored expected outputs [S55].
- Decisions and style: a style guide of consistency that wins over the code [S56]; no
  directory of decision records in the root listing.
- Warnings and analysis: `-Werror -Wall -Wpedantic` and a few more,
  `-Wmaybe-uninitialized` off under GCC [S53]; the engine's Bazel build adds `-Wshadow`
  and `-ffp-contract=off` [S54].

**Not to copy:** two build systems with two warning lists, the CMake options [S53]
against the engine's Bazel options [S54]. This repository has one build; a flag list kept
in two places drifts, as MuJoCo's has.

**To copy in small:** a closed default visibility with a named `package_group` once there
is more than one package [S54]; one tolerance helper with a switch that shows the
residual [S55]; a short style guide that wins over the code [S56].

### MoveIt 2

Read at commit `8c99e523bc23149d24a61990c64d53c9c7d2905f` of moveit/moveit2.

- The tree: packages in layers that depend one way, from a common package through the
  core and the plugins to the ROS integration [S57].
- Targets and visibility: a shared library per subdirectory of the core, one interface
  target over them, abstract interfaces in the core and implementations loaded at run
  time [S57].
- Tests: a `test/` directory in each subdirectory, explicit tolerances, tests of the
  contract [S57].
- Decisions and style: AGENTS.md as the one file of principles and commands, a migration
  entry per API change, no directory of decision records [S57].
- Warnings and analysis: one macro with `-Wall -Wextra` and seven more, no `-Werror`
  [S59]; CI adds `-Werror` and runs clang-tidy on changed code [S57] with a style and
  performance allow-list [S58].

**Not to copy:** run-time plugin loading and the split into many packages [S57]. They let
separately released packages be swapped by configuration; here one owner builds one
program, and a wrong plugin name would fail at run time instead of at link time.

**To copy in small:** the file for agents as short rules answerable yes or no [S57]; a
clang-tidy allow-list that starts from `-*` [S58]; the warning list in one place [S59],
with `-Werror` held by the build, not by CI alone.

## Considered and not taken

1. `layering_check` [S6]: it needs a Clang toolchain; this
   repository builds with GCC and names no Clang toolchain in `.bazelrc` or
   `MODULE.bazel`.
2. DWYU or a clang-tidy aspect as a build dependency today [S7][S8]: each adds a module to
   `MODULE.bazel`, and `bazel_clang_tidy` has no pinned release; a script over includes
   and the database of `tools/architecture/compdb.py` answer most of the same questions.
3. One library per header with a rollup per package [S47][S49]: with 110 rules in one
   `BUILD.bazel` and one owner it multiplies targets before any package cut uses them.
4. A blanket `-Werror`, which [S34] advises against only for source that is distributed:
   optimizer-dependent warnings move with the compiler version and every build here is
   optimized; `-Werror=<flag>` for flags already clean is the form that holds.
5. Wholesale `modernize-*` and `readability-*` groups [S50]: style findings by the
   hundred, which agents would "fix" at a risk to behaviour that their task forbids.
6. GCC's `-fanalyzer` [S38]: GCC's own notes say it is unlikely to help on C++, and this
   code is C++20.
7. MemorySanitizer and ThreadSanitizer by default [S40][S39]: MemorySanitizer needs Clang
   and every library instrumented; ThreadSanitizer does not combine with ASan, and only
   the threads of `src/layout_plan.cc` and `src/trial_runner.cc` would gain from it.
8. Run-time plugin loading and many packages [S57]: they serve separately released ROS
   packages; this repository has no ROS, one build and one owner.
9. Two review phases with named owners [S52]: written for nine maintainers and outside
   contributors; here one owner reviews agents' work, and checks stand in for a second
   reviewer.
10. Diátaxis's four-way split [S25]: tutorials and how-to guides serve outside learners;
    here the readers are one owner and agents, who need reference and explanation.
11. Byte-for-byte golden files across platforms [S17]: MuJoCo is exact only on one
    version and architecture [S11], and development here is on ARM while CI is x86-64.
12. A long `AGENTS.md` with a tour of directories [S28]: overviews did not help agents
    and raised cost, and the stand-in cell's part of a tour goes stale when it is
    replaced.
13. A tree made clean of findings before a check is switched on [S41]: Chromium calls
    that a non-goal; a count that may not grow lets agents keep working meanwhile.
14. A full 5C component framework [S9]: built for teams on a framework; for one owner and
    a stand-in cell about to be replaced, only keeping configuration apart from
    computation is worth taking.

## What could not be read

These were tried and could not be fetched or read, or were read only through another
text; their absence here is not a judgment on them.

- D. L. Parnas, "On the criteria to be used in decomposing systems into modules", 1972:
  scanned images without text, a block, an expired certificate; only the review [S1].
- Ford, Parsons, Kua, "Building Evolutionary Architectures", chapter 2: blocked; an
  InfoQ article quoting its definition of a fitness function was read and is not cited.
- Lakos's two books on large-scale C++, Martin's "Stability" column and "Clean
  Architecture" chapter 14: not freely readable or not fetched.
- Olaf Zimmermann's posts on Medium (MADR, Y-statements): refused; his "Ten Common
  Mistakes" post: not fetched.
- SEI CERT C++ FLP30-CPP: timed out; only the C rule FLP30-C was read.
- REP 103 and REP 105 as rendered pages: refused; their source files were read [S10].
- The `.clang-tidy` files of Abseil, MuJoCo, OMPL, Pinocchio and Ceres: not at the paths
  tried. Eigen's CMake warning options: not on the GitHub mirror.
- Wilson et al., "Good Enough Practices in Scientific Computing": text too garbled.
- SDFormat's specification, the ROS URDF page, MoveIt's URDF tutorial: checked to exist,
  not read.
- No paper was found that measures which properties of a repository (fast tests, small
  files) raise an agent's success; that part of (h) rests on vendor advice alone.
