# Research D: robotics-specific design and tests (topics c, d)

All pages fetched on 2026-11-25 with status 200. "Read: whole" means the page or the relevant section was read in full; web pages were stripped to text and searched, so some are "in part".

### 5C paper (Vanthienen, Klotzbuecher, Bruyninckx)
- Citation: Vanthienen, Klotzbuecher, Bruyninckx, "The 5C-based architectural Composition Pattern: lessons learned from re-developing the iTaSC framework for constraint-based robot programming", Journal of Software Engineering for Robotics 5(1), May 2014, pp. 17-35. https://aisberg.unibg.it/bitstream/10446/87662/1/78-379-1-PB.pdf
- Kind: primary
- Read: 2026-11-25, in part (abstract, 1.1, the opening line of each of sections 4-8, conclusions; PDF text extracted by a script)
- Topics: c
- Claim 1: The 5C principle splits robot software into communication, computation, coordination, configuration and composition. The authors call it their most often proven best practice after about 5 + 5 person-years of building and refactoring one framework. Note: this is the 2014 JOSER paper, not the 2013 "BRICS Component Model" paper; it cites the earlier 5C work as refs [3], [4].
  Quote: "the communication, computation, coordination, and configuration aspects of any software project should be kept fully separated, but ready to be integrated into a composition architecture"
- Claim 2: Each concern is defined short and operationally: computation is the algorithmic functionality; configuration changes the settings (for example control gains, buffer sizes) of entities of other concerns; coordination selects behaviours (discrete, as a state machine); communication is data exchange.
  Quote: "Configuration influences the behavior of entities of the other concerns by changing its settings."
- Claim 3: Configuration is kept apart from coordination by giving it its own entity (a "Configurator") that holds named sets of parameters, and communicated data carries a semantic tag because "many data types ... have no specific semantic meaning". The paper also says that imposing an architecture at the start of a complex development is "a recipe for failure in the long term".
- Bears on this repository: A checkable rule is "algorithm code (IK, planner, perception) does not read JSON/config files or write CSV; a separate layer loads configuration and passes plain structs in", which can be verified with an include/dependency check; the full 5C component framework (Composer, Coordinator, rFSM) does not fit a single-owner 40k-line repository.

### REP 103 Standard Units of Measure and Coordinate Conventions
- Citation: Foote, Purvis, "REP 103: Standard Units of Measure and Coordinate Conventions", created 2010, post-history 2014 (source file in the REP repository, master). https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst (the rendered page https://www.ros.org/reps/rep-0103.html returned an Anubis "Access Denied" page to the fetcher, so the reStructuredText source was read instead)
- Kind: primary
- Read: 2026-11-25, in part (Rationale, Exceptions, Units, Coordinate Frame Conventions, Suffix Frames)
- Topics: c
- Claim 1: ROS standardises on SI units: metre, kilogram, second, ampere; radian for angle, hertz, newton, watt, volt, celsius. The reason given is that inconsistency in units "is a common source of integration issues for developers and can also lead to software bugs".
  Quote: "choosing different conventions should be well justified and well documented."
- Claim 2: Frames are right handed; for a body x is forward, y left, z up; a camera's optical frame (suffix "_optical") is z forward, x right, y down. A second frame with a suffix ("_ned") is the way to carry a different convention, not a silent change of axes.
- Bears on this repository: Suggests one written rule (units SI, radians, metres, in `AGENTS.md`) plus a check that any variable or field holding a non-SI quantity has the unit in its name (for example `_deg`, `_mm`); a typed-units library is not required by the source.

### REP 105 Coordinate Frames for Mobile Platforms
- Citation: Meeussen, "REP 105: Coordinate Frames for Mobile Platforms", created 2010 (source file, master). https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst (rendered page blocked as above)
- Kind: primary
- Read: 2026-11-25, in part (Specification: base_link, odom, map, earth)
- Topics: c
- Claim 1: Frames are given fixed names with defined meaning and error behaviour (odom: continuous but drifts; map: no drift but may jump), so that components written by different people can be combined.
  Quote: "the most important part is that the choice of reference position is clearly documented for users to avoid confusion."
- Claim 2: The REP is about mobile bases (localization), so most of its content (odom, map, earth) does not apply to a fixed-base cell; what carries over is the practice of naming frames and documenting the world origin.
- Bears on this repository: Fits as a rule "each cell layout file documents the world frame origin and axes; frame names are fixed identifiers (`world`, `torque_tube`, `camera_optical`)"; the odom/map/earth content does not fit a fixed cell.

### Drake multibody notation (monogram notation)
- Citation: Drake project, "Multibody Notation: Basics" (group `multibody_notation_basics`), Drake C++ documentation, current at fetch. https://drake.mit.edu/doxygen_cxx/group__multibody__notation__basics.html
- Kind: primary
- Read: 2026-11-25, in part (the section on monogram notation; fetched through a summarising tool, quotes were returned verbatim by it and checked against its answer to a second, narrower request)
- Topics: c
- Claim 1: A quantity is named with the type first, then reference and target, then (if needed) the frame it is expressed in, for example `w_AB_W` for the angular velocity of B in A expressed in W; the notation is meant to let code be compared with the theory it implements.
  Quote: "an unambiguous specification that can be used to compare code with the theory it implements."
- Claim 2: The frame in the name makes mistakes checkable by eye: "verification can be achieved by rote pattern matching of symbols". When the reference frame is obvious, the expressed-in frame may be left out and defaults to it.
  Quote: "verification can be achieved by rote pattern matching of symbols."
- Bears on this repository: A yes/no rule: "a variable holding a pose or vector between frames is named `X_AB` / `p_AB_W` style, or has a comment stating frame; `T`/`pose`/`pos` alone is a finding" (a reviewer or `grep` heuristic, not a compiler check); full monogram is heavy for a repository not using Drake, so a lighter `<quantity>_<target>_in_<frame>` form may fit better.

### MuJoCo "Computation" chapter: Reproducibility
- Citation: MuJoCo project (Google DeepMind), "Computation", section "Reproducibility", MuJoCo documentation (stable). https://mujoco.readthedocs.io/en/stable/computation/index.html
- Kind: primary
- Read: 2026-11-25, in part (section Reproducibility)
- Topics: c, d
- Claim 1: The pipeline is deterministic and reproducible if the full integration state is restored (including warm-start accelerations), but tiny numerical differences grow quickly, especially with contact.
  Quote: "Exact reproducibility is only guaranteed within a single version, on the same architecture."
- Claim 2: The same trajectory is "likely" different between MuJoCo versions or different operating systems, so a recorded trajectory or golden file is valid only for one version and platform. This repository is on MuJoCo 4.0.0 and Ubuntu 24.04 in CI, so golden values recorded on a developer machine of another architecture (for example aarch64) may not match CI.
- Bears on this repository: Rules "golden numbers are compared with a tolerance, never byte for byte" and "a MuJoCo version bump is its own change, with golden values re-recorded in that change only".

### MuJoCo programming guide: Simulation (determinism, warmstart, batch)
- Citation: MuJoCo project, "Simulation" (programming chapter), MuJoCo documentation (stable). https://mujoco.readthedocs.io/en/stable/programming/simulation.html
- Kind: primary
- Read: 2026-11-25, in part (first 100,000 of 117,276 characters via a summarising tool, then the determinism paragraphs directly)
- Topics: c, d
- Claim 1: A simulation is a function of the "integration state"; two `mjData` with the same integration state give identical output. Warm starts matter for bit-wise equality when loading a non-initial state (the initial state is always cold started).
  Quote: "MuJoCo simulations are deterministic."
- Claim 2: One `mjData` per thread with a shared read-only `mjModel` is the supported threading pattern; the batch API gives results bit-identical to a serial loop at any thread count. The page also warns that results compared across samples should differ for physical reasons, "not different warm-start or termination of the iterative solver".
- Bears on this repository: Rule: "a test that must be repeatable builds a fresh `mjData` (or resets with `mj_resetData`) per run and does not share one between tests"; check by grep for `mjData` in file-scope or `static` variables.

### MuJoCo test tree (engine tests with tolerance matchers)
- Citation: MuJoCo project, `test/BUILD.bazel`, `test/pipeline_test.cc`, `test/engine/engine_forward_test.cc`, branch `main`. https://raw.githubusercontent.com/google-deepmind/mujoco/main/test/BUILD.bazel , https://raw.githubusercontent.com/google-deepmind/mujoco/main/test/pipeline_test.cc , https://raw.githubusercontent.com/google-deepmind/mujoco/main/test/engine/engine_forward_test.cc
- Kind: primary
- Read: 2026-11-25, in part (header of BUILD.bazel; first 80 lines of pipeline_test.cc; matcher lines of the engine test)
- Topics: d
- Claim 1: A simulator's own regression tests live in a `test/` tree that mirrors `src/` ("Subdirectories mirror the structure in mujoco/src"), use GoogleTest/GoogleMock, and compare vectors with tolerances rather than exact equality: `Pointwise(MjNear(1e-14, 1e-6), qacc)` and `const mjtNum tol = MjTol(1e-11, 5e-4);` (a tighter tolerance for double, a looser one for single precision).
- Claim 2: Tests are written as properties that survive restructuring (for example "sparse and dense pipelines should produce the same results, for all solvers"; "energy never increases (small tolerance for rounding)"), not as pinned output files.
- Bears on this repository: Rule: "numeric comparisons in tests use a named tolerance (`EXPECT_NEAR` or a helper) and each tolerance has a comment; `EXPECT_EQ` on a `double` is a finding"; checkable by grep. Mirroring `src/` in `tests/` only matters once `src/` has subdirectories.

### GCC manual: `-ffp-contract`
- Citation: GNU Project, "GCC 16 manual (Optimize Options)", entry `-ffp-contract=style`. https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
- Kind: primary
- Read: 2026-11-25, in part (the `-ffp-contract` entry)
- Topics: c
- Claim 1: `-ffp-contract=fast` lets the compiler form fused multiply-add where the target has it, which changes rounding; `off` disables it; `on` allows it within one expression only.
  Quote: "The default is -ffp-contract=off for C in a standards compliant mode (-std=c11 or similar), -ffp-contract=fast otherwise."
- Claim 2: The sentence names C, so the default for C++ built with `-std=c++20` or `-std=gnu++20` is not stated by this page; I did not test it. Whether results change then depends on the target having FMA (default aarch64 does; default x86-64 without `-march=` does not), so the same source can give different bits on a developer machine and on CI.
- Bears on this repository: Rule candidate: "the build sets `-ffp-contract=off` explicitly (one `copt` line in `.bazelrc`)", which a grep can check, and which makes golden values less dependent on architecture; the cost is some speed and that MuJoCo itself (a prebuilt binary) is not covered by the flag.

### GCC wiki: Floating-point math (x87 note)
- Citation: GCC project wiki, "FloatingPointMath", last edited 2026-06-02. https://gcc.gnu.org/wiki/FloatingPointMath
- Kind: primary
- Read: 2026-11-25, in part (the note on x86 floating-point math)
- Topics: c
- Claim 1: Only on legacy x86 without SSE2 are `float`/`double` operations done in extended precision with double rounding; on x86 with SSE2 the options `-mfpmath=sse -msse2` make all operations happen in SSE registers and be correctly rounded.
  Quote: "should therefore be used whenever possible for predictable numerical results."
- Claim 2: x87 excess precision is therefore not a concern for x86-64 GCC builds, where SSE math is the default; FMA contraction (previous block) is the bigger risk for this repository.
- Bears on this repository: Does not add a rule; supports saying "x87 is not a concern on x86-64 Ubuntu 24.04", so the x87 warning should not be turned into work items.

### C++ draft standard: random number engines and distributions
- Citation: ISO C++ working draft (Standard for C++, eel.is/c++draft), [rand.predef] and [rand.dist.general]. https://eel.is/c++draft/rand.predef , https://eel.is/c++draft/rand.dist.general
- Kind: primary
- Read: 2026-11-25, in part ([rand.predef] required-behaviour clauses; [rand.dist.general] clause 3)
- Topics: c
- Claim 1: Engines are pinned exactly: for `mt19937` the 10000th consecutive call of a default-constructed object "produces the value 4123659995", so a seeded engine gives the same raw sequence on every conforming implementation.
- Claim 2: Distributions are not pinned.
  Quote: "The algorithms for producing each of the specified distributions are implementation-defined."
  So `std::uniform_real_distribution`, `std::normal_distribution` and `std::uniform_int_distribution` may give different values from the same engine on libstdc++, libc++ or MSVC, and between library versions.
- Bears on this repository: Rule: "seeded randomness uses `std::mt19937`/`mt19937_64` directly and a project-owned function (for example `u01 = (g() >> 11) * 0x1.0p-53`) in place of `std::*_distribution` wherever a result is recorded in a golden file"; check by grep for `_distribution` in `src/`. With GCC only and one libstdc++ it is a risk for upgrades, not today.

### cppreference: Pseudo-random number generation
- Citation: cppreference.com, "Pseudo-random number generation", `<random>`. https://en.cppreference.com/w/cpp/numeric/random
- Kind: secondary (community reference wiki, not the standard)
- Read: 2026-11-25, in part (engines, default_random_engine, random_device)
- Topics: c
- Claim 1: `default_random_engine` is "an implementation-defined RandomNumberEngine type", and `std::random_device` may be a pseudo-random engine when no entropy source exists; neither is a safe choice when a run must be repeatable.
- Claim 2: The Mersenne twister is described as slower with more state but with the longest non-repeating sequence of the standard engines.
- Bears on this repository: Rule: "no `std::default_random_engine`, no `std::random_device` outside one explicit 'make a seed' place; every program prints its seed"; grep-checkable.

### Drake: Stochastic Systems and RandomGenerator
- Citation: Drake project, "Stochastic Systems" and "RandomGenerator Class Reference", Drake C++ documentation. https://drake.mit.edu/doxygen_cxx/group__stochastic__systems.html , https://drake.mit.edu/doxygen_cxx/classdrake_1_1_random_generator.html
- Kind: primary
- Read: 2026-11-25, in part (the detailed description of the first; the class description of the second)
- Topics: c
- Claim 1: Drake's rule is that anything callable during a simulation is a deterministic function; randomness enters only through an explicit random input or a passed-in generator, and the state of the generator is kept in the simulation context.
  Quote: "every method that can be called during the lifetime of a simulation ... must be a completely deterministic function."
- Claim 2: The project's random generator is `std::mt19937` with an explicit seed constructor and a `default_seed`; random state and parameter functions "must be deterministic functions of their input arguments", with the generator the "only source of randomness"; distribution objects are made locally rather than held as mutable members.
- Bears on this repository: Rule: "no function in `src/` creates its own RNG; a seeded `std::mt19937&` (or a seed integer) is a parameter"; check by grep for `std::mt19937`/`random_device` constructions outside `main`/test setup.

### ros2_control: Mock Components
- Citation: ros2_control project, "Mock Components", ros2_control documentation (Rolling, Nov 2026). https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html
- Kind: primary
- Read: 2026-11-25, in part (introduction and the generic system)
- Topics: c
- Claim 1: In ros2_control a "hardware component" (Actuator, Sensor or System) abstracts physical hardware; a mock component can be put in place of the real one in the same configuration, so controllers, launch files and the rest are tested without hardware. The mock only mirrors commands to states.
  Quote: "They provide ideal behavior by mirroring commands to their states."
- Claim 2: The hardware is selected in a `<ros2_control>` element of the robot description (URDF) by a plugin name and parameters, so swapping a mock for a real driver is a change of data, not of controller code. (The hardware components overview page, https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html, gives only the three types and links; I did not read the linked pages.)
- Bears on this repository: Fits as a rule "controller code depends on an abstract interface (`read` sensors / `write` commands) that has a stand-in cell implementation and, later, a real one; no `mjModel`/`mjData` appears in the controller target's dependencies", checkable with a Bazel `deps` query once targets are split; ros2_control's plugin loading and lifecycle do not fit.

### Software Engineering at Google, ch. 11 "Testing Overview"
- Citation: Winters, Manshreck, Wright (eds.), "Software Engineering at Google", chapter 11 "Testing Overview", O'Reilly 2020, free edition. https://abseil.io/resources/swe-book/html/ch11.html
- Kind: primary
- Read: 2026-11-25, in part (section on test size)
- Topics: d
- Claim 1: Google sorts tests by size, not by scope: small tests run in one process (often one thread), medium on one machine, large wherever they want; size is defined by what a test is allowed to do and which resources it uses, and the infrastructure can enforce it.
  Quote: "the most important qualities we want from our test suite are speed and determinism, regardless of the scope of the test."
- Claim 2: Small tests may not sleep, do I/O or use the network; medium tests have more freedom but "more risk of nondeterminism"; the large ones are kept for the hardest cases.
- Bears on this repository: Rule: "every `cc_test` declares `size`: `small` for pure logic, `medium` for tests that load a MuJoCo model, `large` for the gate test"; checkable by `bazel query 'attr(size, ...)'` or grep in `BUILD.bazel`.

### Google Testing Blog: Test Sizes
- Citation: Stewart, "Test Sizes", Google Testing Blog, 13 December 2010. https://testing.googleblog.com/2010/12/test-sizes.html
- Kind: primary (author describing his employer's own practice; not a summary of someone else)
- Read: 2026-11-25, whole (post; comments skimmed)
- Topics: d
- Claim 1: Small, medium, large replace "unit", "integration", "system" because those names mean different things to different teams; the table sets limits (small: no network, no database, no file system, no threads, no sleeps, 60 s; medium: 300 s; large: 900+ s).
- Claim 2: The sizes can be policed by the test runner, and tests must be runnable in any order, which needs high isolation.
  Quote: "tests need high isolation"
- Bears on this repository: Supports a rule that tests do not depend on order or on files left by another test (check: run with `--runs_per_test` and `--test_sharding_strategy`/shuffle); the 60 s small limit is a number that could be adopted.

### Bazel: Test Encyclopedia
- Citation: Bazel project, "Test Encyclopedia", bazel.build reference. https://bazel.build/reference/test-encyclopedia
- Kind: primary
- Read: 2026-11-25, in part (size and timeout section)
- Topics: d
- Claim 1: Bazel has a `size` for tests (small, medium, large, enormous) that implies a timeout (short, moderate, long, eternal) and an assumed resource use.
- Claim 2: If a test does not state its size it is "medium"; the docs suggest lowering the size when a test runs much faster than its class ("if a 'moderate' test completes in 5.5s, consider setting timeout = 'short' or size = 'small'"), and `--test_verbose_timeout_warnings` lists oversized tests.
- Bears on this repository: Rule: "no `cc_test` leaves `size` unset" (Bazel would otherwise silently use `medium`) and the CI step runs `--test_verbose_timeout_warnings`; both checkable.

### Drake's test macro (`drake_cc_googletest`) in `drake_cc.bzl`
- Citation: Drake project, `tools/skylark/drake_cc.bzl`, branch `master`, lines about 822-870. https://raw.githubusercontent.com/RobotLocomotion/drake/master/tools/skylark/drake_cc.bzl
- Kind: primary
- Read: 2026-11-25, in part (docstring and body of the C++ unit test rule)
- Topics: d
- Claim 1: Drake's test rule defaults to `size = "small"` ("because that indicates a unit test"), forces `testonly = 1`, and by default expects the test source at `test/<name>.cc`: tests sit in a `test/` directory next to each library's sources, one directory per package.
  Quote: "By default, sets name=\"test/${name}.cc\" per Drake's filename convention."
- Claim 2: The macro also has opt-ins for the things that break determinism or speed (network, display, rendering, threads), so a test that needs them has to say so.
- Bears on this repository: A project-owned `cc_test` macro (`repo_test`) that defaults `size = "small"`, sets `testonly`, and requires `allow_network`-style opt-ins would make the size rule above mechanical; moving to a `test/` per-package layout depends on first splitting the flat `src/`.

### Feathers: "Characterization Testing"
- Citation: Michael Feathers, "Characterization Testing: Writing tests to describe and fix existing code", 8 August 2016. https://michaelfeathers.silvrback.com/characterization-testing
- Kind: primary
- Read: 2026-11-25, whole
- Topics: d
- Claim 1: A characterization test records what code actually does, not what it should do: write a test with a placeholder expectation (he names it "x"), run it, copy the actual value into the test, then rename the test to say what was learned.
  Quote: "The purpose of characterization testing is to document your system's actual behavior, not check for the behavior you wish your system had."
- Claim 2: Such tests are most valuable "when we want to refactor or rewrite" because a failing run says at once that behaviour changed; the hard part is "breaking dependencies around a piece of code well enough to be able to exercise it in a test harness".
- Claim 3: Tests that capture behaviour should stay even if the behaviour looks like a bug, until someone has decided it is one: "When a system goes into production, in a way, it becomes its own specification."
- Bears on this repository: Fits: before restructuring a module, an agent first adds characterization tests (recorded outputs of the existing functions on fixed inputs) and the restructuring change must pass them unchanged; the rule "a change that edits an expected value in a characterization test must say so in the commit text" is checkable on the diff.

### Fowler: Self-Testing Code
- Citation: Martin Fowler, "Self Testing Code", martinfowler.com bliki, 1 May 2014. https://martinfowler.com/bliki/SelfTestingCode.html
- Kind: primary
- Read: 2026-11-25, in part (first screenfuls; about two thirds)
- Topics: d
- Claim 1: Self-testing code means one command runs automated tests and, if they pass, you can be confident the code has no substantial defects; he used the term in "Refactoring".
  Quote: "You have self-testing code when you can run a series of automated tests against the code base and be confident that, should the tests pass, your code is free of any substantial defects."
- Claim 2: He treats it as a precondition for continuous integration and for the confidence to change software; "assume that any non-trivial code without tests is broken".
- Bears on this repository: Supports a single command (`bazel test //...`) as the definition of "behavior unchanged" in `AGENTS.md`; whether the tests are strong enough for that is a separate question (see the coverage note under Does not fit).

### Fowler: Refactoring (the catalog's own definition)
- Citation: Martin Fowler, "Refactoring", refactoring.com (Refactoring Catalog), current at fetch. https://refactoring.com/
- Kind: primary
- Read: 2026-11-25, in part (the opening section)
- Topics: d
- Claim 1: Refactoring changes internal structure "without changing its external behavior" through a series of small behaviour-preserving steps, and the system is "kept fully working after each refactoring".
  Quote: "altering its internal structure without changing its external behavior."
- Claim 2: He says refactoring is part of day-to-day programming rather than a planned phase, done before adding a feature to make it easy and after to clean up. The page does not define what counts as "external behavior" for numeric programs; that has to be fixed by the repository (for example tolerance).
- Bears on this repository: The rule "a refactoring commit contains no change to any test expectation and each commit builds and passes the tests" is checkable on the diff; it also means refactors and behaviour changes go in separate commits.

### ApprovalTests: concept and custom comparators
- Citation: ApprovalTests.cpp project, "ApprovalTesting (the concept)" and "Custom Comparators", docs in the repository, branch `master`. https://raw.githubusercontent.com/approvals/ApprovalTests.cpp/master/doc/ApprovalTestingConcept.md , https://raw.githubusercontent.com/approvals/ApprovalTests.cpp/master/doc/CustomComparators.md
- Kind: primary
- Read: 2026-11-25, in part (both pages are short; the first is mostly an outline)
- Topics: d
- Claim 1: Approval, snapshot and golden master testing are one idea: you state the data you want to check, look at it by hand until satisfied, and the library then ensures it "remains consistent for the future"; approving a result is just saving the `.approved` file (via a diff tool, a command, or renaming `.received` to `.approved`), which is the point at which a wrong result can be blessed unread.
- Claim 2: The default comparison reads both files character by character (ignoring line endings), and the project's own example for a custom comparator is output with tiny differences between environments that no human would see, so exact comparison is brittle for numeric or platform-dependent output and a tolerance-based comparator has to be registered per file extension.
  Quote: "reads both files, one character at a time, and tests if their contents are identical."
- Bears on this repository: Rule: "CSV golden files for the table programs are compared with a parsing comparator that uses a stated tolerance per column, or numbers are printed with a fixed small number of digits; re-approval is a separate commit that shows the diff"; checkable only partly (the diff rule by review).

### MuJoCo modeling guide and XML reference: model as data with a schema
- Citation: MuJoCo project, "Modeling" and "XML Reference" (XSD schema section), MuJoCo documentation (stable). https://mujoco.readthedocs.io/en/stable/modeling.html , https://mujoco.readthedocs.io/en/stable/XMLreference.html
- Kind: primary
- Read: 2026-11-25, in part (introductions and the XSD schema section)
- Topics: c
- Claim 1: MuJoCo's native model format MJCF is an XML description of the whole dynamical system (it can also load URDF), with a defaults mechanism "inspired by the idea of Cascading Style Sheets (CSS)" so models start small.
- Claim 2: The schema is "generated from the same source of truth and checked in as src/xml/generated/mjcf.xsd", and editors use it to report ill-formed values as you type: the format has a machine-checked schema, in addition to the loader's own errors.
- Bears on this repository: Fits as a rule "every cell-layout JSON validates against a schema file in the repository, and the checking program and the schema agree (one test loads every file in `layouts/`)"; the repository's JSON is checked by a program, so the rule asks for a schema file as the second, declarative description.

## Does not fit

- Software Engineering at Google, ch. 11 / Google Testing Blog "Test Sizes" (test sizes enforced by the test infrastructure, for example a security manager that blocks network or file access in small tests): Bazel has `size` labels but no enforcement of the Google limits, and a flat suite of 41 test files with MuJoCo loads, so the enforced version is not available; only the labelling rule fits.
- Bruyninckx et al., 5C paper (Composer, Coordinator, Configurator entities, rFSM statecharts, a model-driven code generator): built for component frameworks with dozens of researchers and DSLs; for one owner and agent contributors on 40k lines it would add more structure than it removes. Only the separation of concerns (algorithms vs configuration vs I/O) is worth adopting.
- REP 105 (base_link / odom / map / earth): a convention for mobile robots and localization; a fixed-base assembly cell has no odometry or map, so only "name and document your frames" carries over.
- ros2_control hardware components and Mock Components (plugin loading, lifecycle nodes, `<ros2_control>` URDF tags): needs ROS 2 and a URDF description; the repository has neither. Take only the idea of a hardware interface that has a stand-in and a real implementation.
- Approval/snapshot tests of whole CSV outputs with exact (character-for-character) comparison, the ApprovalTests default: the MuJoCo documentation itself says bit-exact reproduction is only guaranteed within one version and architecture, so a byte-for-byte golden file would fail on a compiler or library upgrade or on a developer machine of a different architecture; use tolerance or round the numbers.
- Test coverage thresholds as a gate: Fowler's TestCoverage page (a snippet was seen while searching, not cited as a source above: it says coverage is "of little use as a numeric statement of how good your tests are") argues against a numeric target; a threshold would also push agents to add assertion-free tests.
- A `test/` directory per package as in Drake and MuJoCo: needs `src/` to be split into packages first; with a flat `src/` and flat `tests/` the question is moot until then.

## Leads not followed

- ROS REP 103 and REP 105 rendered pages on ros.org: fetch returned an Anubis "Access Denied" page; the source files on GitHub were used instead.
- Bruyninckx et al., "The BRICS Component Model: A Model-Based Development Paradigm for Complex Robotics Software Systems" (2013): not fetched; the 2014 JOSER 5C paper was read instead. The 5C paper cites earlier 5C work as its refs [3] and [4], which were not read.
- Search results seen but not read: Bruyninckx LCCC 2012 slides (lccc.control.lth.se), TU Eindhoven "Embedded Motion Control" lecture slides (dsdwiki.wtb.tue.nl), Springer chapter 10.1007/978-3-030-66494-7_3.
- Drake: separation of plant and controller, hardware station, `ManipulationStation` docs; not read (only the stochastic-systems and notation pages).
- SDFormat spec (http://sdformat.org/spec, status 200), ROS wiki URDF XML page (status 200) and MoveIt's URDF/SRDF tutorial (https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html, seen with status 200): fetched only to check they exist; their content was not read, so nothing is claimed from them.
- Fowler's "Definition Of Refactoring" bliki page: the page header loaded but its text was not read.
- GCC "Semantics of Floating Point Math" and the x87 note (`x87note` on the GCC wiki): not read.
- Gazebo's and Drake's own regression-test/golden-file descriptions: not read (MuJoCo's test directory was used instead; no golden-file README was found there).
