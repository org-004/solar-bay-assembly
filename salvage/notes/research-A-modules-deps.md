# Research A: module boundaries, dependency health, machine-kept principles

Topics: (a) boundaries and direction of dependencies; (k) healthy dependency tree, declared vs used, third-party, pinning, licences; (g) fitness functions, architecture tests, ratchets.
All pages below were fetched on 2026-11-25 with status 200.

### Parnas, criteria for decomposition (quoted via a review)
- Citation: Adrian Colyer ("the morning paper"), "On the criteria to be used in decomposing systems into modules" (summary of D. L. Parnas, CACM 1972). https://blog.acolyer.org/2016/09/05/on-the-criteria-to-be-used-in-decomposing-systems-into-modules/
- Kind: secondary (the primary PDFs at wstomv.win.tue.nl and cse.sc.edu are scanned images with no text layer; the uniroma3 copy has an expired certificate; ACM blocks fetching. The page quotes the paper.)
- Read: 2026-11-25, in part (the passages quoting the paper)
- Topics: a
- Claim 1: The second (better) decomposition in Parnas's example splits the system by design decisions that are likely to change, and each module hides one such decision; the first splits by processing steps.
  Quote: "Every module in the second decomposition is characterized by its knowledge of a design decision which it hides from all others."
- Bears on this repository: a yes/no review question per module ("which one decision does this module hide, and does any other module's header expose it?"), e.g. MuJoCo, OMPL and GLFW types should appear in the headers of one module each; the stand-in cell is a decision likely to change and so belongs behind one interface.

### Martin, "Granularity" (Engineering Notebook, C++ Report)
- Citation: Robert C. Martin, "Granularity", C++ Report column (1996-era; the text lists his 1996 columns), Object Mentor PDF, archived. https://web.archive.org/web/2007/http://www.objectmentor.com/resources/articles/granularity.pdf
- Kind: primary
- Read: 2026-11-25, in part (the Acyclic Dependencies Principle section; text extracted from the PDF streams, so the quotes below are from that extraction)
- Topics: a, k
- Claim 1: Dependencies between packages must form a directed acyclic graph; cycles make the "morning after" integration problem unavoidable.
  Quote: "THE DEPENDENCY STRUCTURE BETWEEN PACKAGES MUST BE A DIRECTED ACYCLIC GRAPH (DAG). THAT IS, THERE MUST BE NO CYCLES IN THE DEPENDENCY STRUCTURE."
- Claim 2: A cycle forces the packages in it to be built, tested and released together, and is the reason a unit test of one class drags in many libraries; cycles can always be broken by dependency inversion or by moving shared parts into a new package.
  Quote: "Thus the dependency structure must always be monitored for cycles."
- Bears on this repository: the cycle check can be a command (graph of `bazel query 'deps(...)'` over the 105 targets after splitting into packages must be acyclic); in one Bazel package targets cannot form a cycle anyway, but header `#include` cycles between files still can.

### Martin, Principles of OOD (SDP statement)
- Citation: Robert C. Martin, "Principles of OOD", butunclebob.com, archived 2010. https://web.archive.org/web/2010/http://butunclebob.com/ArticleS.UncleBob.PrinciplesOfOod
- Kind: primary
- Read: 2026-11-25, in part (the list of package principles)
- Topics: a
- Claim 1: States the Stable Dependencies Principle as "depend in the direction of stability" and the Acyclic Dependencies Principle as "no cycles in the package graph".
  Quote: "Depend in the direction of stability."
- Bears on this repository: gives a direction rule: volatile modules (stand-in cell, OMPL switch, CSV programs) may depend on stable ones (geometry, kinematics types), never the reverse; checkable as "no target under `stable` layer lists a target from `volatile` layer in deps".

### Bloomberg BDE, physical code organization (Lakos-style levelization)
- Citation: Bloomberg BDE project wiki, "Physical Code Organization", last edited 2018-03-19. https://github.com/bloomberg/bde/wiki/physical-code-organization
- Kind: primary (a project's own documentation of the Lakos method)
- Read: 2026-11-25, in part (component design rules, package and package-group rules)
- Topics: a, k
- Claim 1: The project forbids cyclic physical dependencies between components, packages and package groups; classes that would form a cycle if separated must live in one component.
  Quote: "Cyclic physical dependencies among components are not permitted."
- Claim 2: Three levels of aggregation (component, package, package group) are used, because a levelizable hierarchy of components alone is not enough for large systems.
- Bears on this repository: supports splitting the flat `src/` into a few packages with a written allowed-dependency order; a flat 99-file directory has only the component level, so "levels" are not yet visible.

### Software Engineering at Google, ch. 18 Build Systems and Build Philosophy
- Citation: Winters, Manshreck, Wright (eds.), "Software Engineering at Google", ch. 18, 2020. https://abseil.io/resources/swe-book/html/ch18.html
- Kind: primary
- Read: 2026-11-25, in part (sections on strict transitive dependencies and visibility)
- Topics: k, a
- Claim 1: Allowing a target to use symbols of its dependencies' dependencies made those dependencies part of its public contract; Google fixed it with a strict transitive dependency mode where an undeclared use fails the build with a command that adds the dependency. The cost is more verbose build files.
  Quote: "fails with an error and a shell command that can be used to automatically insert the dependency."
- Claim 2: Visibility should be as narrow as possible, since it is the reverse of a dependency.
  Quote: "it is usually best to minimize visibility as much as possible."
- Bears on this repository: rule "every `#include` of a header from another target is covered by a direct `deps` entry" and "no target is `//visibility:public` unless it is a binary or documented API"; with one package and no `visibility` neither is enforced today.

### Software Engineering at Google, ch. 16 Version Control and Branch Management (One-Version Rule)
- Citation: same book, ch. 16, 2020. https://abseil.io/resources/swe-book/html/ch16.html
- Kind: primary
- Read: 2026-11-25, in part (the One-Version section)
- Topics: k
- Claim 1: Developers should have no choice of which version of a component to depend on; violations lead to diamond dependencies. The book says the rule is not hard and fast, and not tied to monorepos.
  Quote: "Developers must never have a choice of “What version of this component should I depend upon?”"
- Bears on this repository: one version of each third-party library in `MODULE.bazel` (one `bazel_dep` per name; no second copy of Eigen or Boost via a transitive module); a check is `bazel mod graph` showing one resolved version per module.

### Software Engineering at Google, ch. 21 Dependency Management
- Citation: Titus Winters, "Dependency Management", in Software Engineering at Google, 2020. https://abseil.io/resources/swe-book/html/ch21.html
- Kind: primary
- Read: 2026-11-25, in part (opening, diamond dependencies, SemVer, pinning)
- Topics: k
- Claim 1: Importing a dependency saves development cost but adds ongoing maintenance cost, and upgrades can be forced by security or platform changes. Avoiding a dependency is not always wrong.
  Quote: "Just because you get to avoid a development cost doesn’t mean importing a dependency is the correct choice."
- Claim 2: A diamond dependency (two dependencies needing incompatible versions of a shared one) is a core failure; C++ tolerates it badly because of One Definition Rule violations. Version numbers (SemVer) are a lossy summary of risk.
- Bears on this repository: argues for a short, justified list of third-party libraries (OMPL pulls 17 Boost modules, a cost to be recorded in an ADR); not a command-checkable rule by itself.

### Bazel documentation, lockfile
- Citation: Bazel project, "Bazel Modules: Lockfile" (bazel.build/external/lockfile), current docs. https://bazel.build/external/lockfile
- Kind: primary
- Read: 2026-11-25, whole (fetched and checked against the raw page)
- Topics: k
- Claim 1: `MODULE.bazel.lock` records hashes of registry files and extension results and is created or updated by the build; `--lockfile_mode` values are `update` (default), `refresh`, `error` (fail if lockfile is missing or outdated) and `off`.
  Quote: "The lockfile contains all the necessary information to determine whether the project state has changed."
- Claim 2: The docs recommend committing it, and pairing it with Bazelisk and `.bazelversion` because it is tied to the Bazel version.
  Quote: "Include the lockfile in version control to facilitate collaboration and ensure that all team members have access to the same lockfile"
- Bears on this repository: CI rule `bazel ... --lockfile_mode=error` plus `git diff --exit-code MODULE.bazel.lock`; `.bazelversion` committed.

### Bazel documentation, module system and registries
- Citation: Bazel project, "Bazel modules" and "Bazel registries", current docs. https://bazel.build/external/module and https://bazel.build/external/registry
- Kind: primary
- Read: 2026-11-25, in part (version resolution; registry sections, found by search in the page text)
- Topics: k
- Claim 1: Bazel resolves one version per module by Minimal Version Selection (borrowed from Go), picking the highest version any dependent asks for, on the assumption that new versions are backward compatible.
  Quote: "Bazel uses the Minimal Version Selection (MVS) algorithm introduced in the Go module system."
- Claim 2: Modules are found through registries, which are databases of modules; the Bazel Central Registry is the default one.
- Bears on this repository: pin `bazel_dep` versions explicitly and take them from the BCR; an `archive_override`/`http_archive` of a library with `sha256` is needed only for what the BCR lacks (check which of MuJoCo, OMPL, GLFW are there before writing the rule).

### include-what-you-use, "Why include what you use?"
- Citation: include-what-you-use project, docs/WhyIWYU.md, master. https://raw.githubusercontent.com/include-what-you-use/include-what-you-use/master/docs/WhyIWYU.md
- Kind: primary
- Read: 2026-11-25, whole
- Topics: k
- Claim 1: If every file includes what it uses, a header can drop an include without breaking far-away code; this is the benefit that needs the policy, while "don't include what you don't use" brings compile time and fewer rebuilds.
  Quote: "If everyone who uses vector is #including `<vector>` themselves, then you can remove `<vector>` without fear of breaking anything."
- Claim 2: The project reports one Google case where it cut a file's compile time by 30 percent.
- Bears on this repository: IWYU is a Clang tool (needs a Clang toolchain, which this repository lacks); the principle can still be a review rule and be checked at target level by DWYU.

### depend_on_what_you_use (DWYU) README
- Citation: martis42, "depend_on_what_you_use", README.md, branch main. https://raw.githubusercontent.com/martis42/depend_on_what_you_use/main/README.md
- Kind: primary
- Read: 2026-11-25, in part (introduction, alternatives)
- Topics: k
- Claim 1: DWYU is a Bazel aspect that fails the build when a `cc_*` target includes a header not provided by a direct dependency, or lists an unused dependency; it can check `implementation_deps` use too.
  Quote: "A `cc_*` target _X_ shall depend directly on the targets providing the header files which are included in the source code of _X_."
- Claim 2: Against Bazel's `layering_check` it needs no full compile, handles header-only libraries, finds unused deps, and is not limited to LLVM toolchains.
- Bears on this repository: a candidate dependency check that works with GCC-only; it is on the BCR (badge in README), so adding it as a `bazel_dep` and running one aspect build in CI is a command-checkable rule. Whether it handles 105 targets in one package is untested here.

### Bazel docs, visibility and layering_check
- Citation: Bazel project, "Visibility" (https://bazel.build/concepts/visibility) and "C++ rules" (https://bazel.build/reference/be/c-cpp), current docs.
- Kind: primary
- Read: 2026-11-25, in part (`default_visibility`, private default; `layering_check` paragraph)
- Topics: k, a
- Claim 1: A target without `visibility` in a package with no `default_visibility` is private to its package; `package(default_visibility = ...)` sets a default; `package_group` is the recommended way to list a common set of packages.
  Quote: "the default is just [\"//visibility:private\"]"  (text appears as `["//visibility:private"]` on the page)
- Claim 2: `layering_check` must be supported by the toolchain and requested explicitly with `--features=layering_check`.
  Quote: "The layering_check feature has to be supported by the toolchain and requested explicitly"
- Bears on this repository: in a single package everything is visible to everything, so visibility cannot yet enforce layers; splitting into packages makes `visibility` a compiler-enforced layer rule. `layering_check` needs Clang modules maps, so it does not run on the GCC toolchain (see Does not fit).

### Google C++ Style Guide, Include What You Use and Forward Declarations
- Citation: Google, "C++ Style Guide", sections "Include What You Use" and "Forward Declarations", current page. https://google.github.io/styleguide/cppguide.html
- Kind: primary
- Read: 2026-11-25, in part (two sections)
- Topics: k
- Claim 1: A file that refers to a symbol should directly include a header meant to provide it.
  Quote: "the file should directly include a header file which properly intends to provide a declaration or definition of that symbol."
- Claim 2: Forward declarations are to be avoided where possible: they can save compile time but can hide a dependency and let code skip a needed recompile.
  Quote: "Avoid using forward declarations where possible."
- Bears on this repository: gives a yes/no review rule (every `.cc`/`.h` includes the header of each type it names, no reliance on transitive includes) and a reason not to adopt blanket forward declarations; IWYU's own docs differ in preferring them.

### REUSE Specification 3.3
- Citation: FSFE, "REUSE Specification, version 3.3". https://reuse.software/spec-3.3/
- Kind: primary
- Read: 2026-11-25, in part (headers, LICENSES directory)
- Topics: k
- Claim 1: Each file carries a comment header with copyright text and an `SPDX-License-Identifier` tag whose value is an SPDX licence expression; or a `.license` side file or `REUSE.toml`/dep5 file covers it.
  Quote: "The SPDX-License-Identifier tag MUST be followed by a valid SPDX License Expression"
- Claim 2: Licence texts go into a `LICENSES/` directory at the project root, each file named by its SPDX identifier.
  Quote: "Each License File MUST be placed in the LICENSES/ directory in the root of the Project."
- Bears on this repository: the `reuse lint` command can check a "licence of each file and of each vendored thing is recorded" rule; with all code written by agents for one owner, per-file headers may be more than needed (see Does not fit), but a `LICENSES/` plus a list of third-party licences is cheap.

### OpenSSF Scorecard, checks documentation
- Citation: OpenSSF, "Scorecard checks" (docs/checks.md), main. https://raw.githubusercontent.com/ossf/scorecard/main/docs/checks.md
- Kind: primary
- Read: 2026-11-25, in part (Pinned-Dependencies, Dependency-Update-Tool)
- Topics: k
- Claim 1: A pinned dependency is set to a specific hash, not a mutable version or range; the check looks at Dockerfiles, shell scripts and GitHub workflows. Pinning can block updates, so it should come with automated update tools.
  Quote: "A \"pinned dependency\" is a dependency that is explicitly set to a specific hash instead of allowing a mutable version or range of versions."
- Claim 2: The Dependency-Update-Tool check looks for Dependabot or Renovate.
- Bears on this repository: rule "every `uses:` in `.github/workflows` is a full commit SHA" and a Dependabot or Renovate config that updates actions and Bazel modules; checkable by `scorecard --checks=Pinned-Dependencies,Dependency-Update-Tool`. Scorecard counts a whole-repository score, which is not needed.

### ArchUnit User Guide
- Citation: TNG Technology Consulting, "ArchUnit User Guide", current. https://www.archunit.org/userguide/html/000_Index.html
- Kind: primary
- Read: 2026-11-25, in part (cycle checks, layer checks, freezing)
- Topics: g, a
- Claim 1: Architecture rules are written as ordinary unit tests: dependencies between packages, layers (`layeredArchitecture()`), slices, and cycles (`beFreeOfCycles()`).
- Claim 2: `FreezingArchRule` stores existing violations and fails only on new ones; fixed violations shrink the stored set so they cannot return.
  Quote: "If violations are fixed, FreezingArchRule will automatically reduce the known stored violations to prevent any regression."
- Bears on this repository: ArchUnit is Java only; the pattern (a rule as a test, plus a frozen file of known violations in git) can be copied with a small script over `#include` lines or `bazel query` output.

### InfoQ, "Fitness Functions for Your Architecture"
- Citation: InfoQ article on fitness functions (author's name not captured), citing Ford, Parsons, Kua, Sadalage, "Building Evolutionary Architectures". https://www.infoq.com/articles/fitness-functions-architecture/
- Kind: secondary
- Read: 2026-11-25, in part (introduction and definition)
- Topics: g
- Claim 1: Quotes the book's definition and says fitness functions are guardrails that can be thought of as automated tests for the architecture; libraries like ArchUnit make structural ones possible.
  Quote: "An architectural fitness function is any mechanism that provides an objective integrity assessment of some architectural characteristic(s)"
- Bears on this repository: each design principle should be accompanied by a mechanism with an objective result (a command or a count); a principle without one stays a review question.

### qntm, "Ratchets in software development"
- Citation: qntm, "Ratchets in software development", Things Of Interest, 2021-11-21. https://qntm.org/ratchet
- Kind: primary (the author describing his own practice)
- Read: 2026-11-25, whole
- Topics: g
- Claim 1: A lint-time script counts instances of a banned pattern (plain text matching, expected numbers hard-coded in the script); too many fails, and too few also fails, prompting the developer to lower the number.
  Quote: "If it counts too few, it also raises an error, this time congratulating you and prompting you to lower the expected number."
- Claim 2: The method stops copy-and-paste spread of old patterns and replaces repeating the same review comment, but does nothing to remove the remaining ones.
- Bears on this repository: a ratchet file (`tools/ratchet.txt`: pattern, allowed count) for legacy findings such as direct `mujoco/` includes outside the wrapper module or clang-tidy warnings, checked by a 30-line script; counts may only fall.

### ratchets (tool), README
- Citation: "ratchets" crate, README for version 0.2.6, docs.rs. https://docs.rs/crate/ratchets/0.2.6/source/README.md
- Kind: primary (the tool's own documentation)
- Read: 2026-11-25, in part (summary, commands, exit codes)
- Topics: g
- Claim 1: Allows a budgeted number of violations per rule and region that can only decrease; commands `check`, `bump` (needs justification in the commit message) and `tighten`; merge driver takes the minimum count.
  Quote: "These budgets can only decrease over time (the \"ratchet\" mechanism), ensuring technical debt monotonically decreases."
- Bears on this repository: shows the shape of a baseline file (per rule, per directory) and that raising a budget should need a written reason; the tool itself is a Rust crate and adds a dependency, so a home-made script may suit better.

## Does not fit

- Clang `layering_check` (Bazel C++ rules docs; DWYU README): it needs a toolchain with module-map support, and this repository builds with GCC only; use DWYU or a script over `#include` lines instead.
- include-what-you-use as a CI tool (IWYU "Why" page): it is built on Clang and needs a compilation database from a Clang toolchain; the principle fits, the tool does not. Its advice to prefer forward declarations also conflicts with the Google style guide's "Avoid using forward declarations where possible".
- ArchUnit (User Guide): the most developed architecture-test library, but Java and Kotlin only; only its ideas (layer rules, cycle rules, frozen violations) carry over.
- The One-Version rule as a monorepo and "Live at Head" policy (SWE at Google ch. 16 and 21): written for an organization with thousands of engineers in one tree; for one owner the useful part is only "one resolved version per module" in `MODULE.bazel`. Likewise REUSE's per-file SPDX headers on every source file are heavy for a single-owner private repository; a `LICENSES/` directory and a third-party table suffice there.
- Bloomberg-style three-level package groups (BDE wiki): a scheme for very large code bases; at 99 files two levels (packages and files) are enough.

## Leads not followed

- Parnas 1972 primary PDF: fetched (win.tue.nl, cse.sc.edu) but scanned images with no text; ACM DL blocked (403); uniroma3 copy has an expired certificate. Only the review's quotations are cited.
- Lakos, "Large-Scale C++ Software Design" (1996) and "Large-Scale C++ Vol. I" (2019): books, not freely readable; the BDE wiki stands in for them. His ACCU 2019 slides (accu.org PDF) were seen but not read.
- Martin, "Stability" column (source of SDP maths: instability I = Ce/(Ca+Ce)) and Clean Architecture ch. 14: not fetched; SDP cited only from the one-line statement.
- Ford, Parsons, Kua, "Building Evolutionary Architectures" (O'Reilly ch. 2): blocked (403); Thoughtworks "Fitness function-driven development" page was fetched but is about logging, not used.
- Chromium/Mozilla documentation on incremental warnings and clang-tidy: not found in search; the ratchet sources above are the substitute.
- SPDX specification itself (spdx.dev), `bazel mod` command reference, `code-baseline` crate: not read.
