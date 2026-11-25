# Research C: static analysis and run-time checking for C++ (topic j)

Note on versions: every clang.llvm.org page below is the "current" (trunk) edition fetched 2026-11-25. Check names and aliases change between releases (the alias table below shows `cert-err33-c`, `cert-err34-c`, `cert-err58-cpp` now pointing at new `bugprone-*` names), so any rule for this repository must pin the clang-tidy version it was tested with.

### clang-tidy: list of checks and alias table
- Citation: LLVM project, "Clang-Tidy Checks" (list, with "Check aliases" table), trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/list.html
- Kind: primary
- Read: 2026-11-25, in part (the whole alias table, read in two fetches)
- Topics: j
- Claim 1: The CERT names are mostly aliases of `bugprone-*`/`misc-*` checks: cert-flp30-c -> bugprone-float-loop-counter; cert-err33-c -> bugprone-unused-return-value; cert-err34-c -> bugprone-unchecked-string-to-number-conversion; cert-err58-cpp -> bugprone-throwing-static-initialization; cert-msc50-cpp -> misc-predictable-rand; cert-msc51-cpp -> bugprone-random-generator-seed; cert-str34-c -> bugprone-signed-char-misuse; cert-arr39-c -> bugprone-sizeof-expression; cert-flp37-c and cert-exp42-c -> bugprone-suspicious-memory-comparison. Enabling both an alias and its target only duplicates the work.
- Claim 2: cppcoreguidelines aliases: cppcoreguidelines-narrowing-conversions -> bugprone-narrowing-conversions; cppcoreguidelines-avoid-magic-numbers -> readability-magic-numbers; cppcoreguidelines-avoid-c-arrays -> modernize-avoid-c-arrays; cppcoreguidelines-explicit-constructor -> misc-explicit-constructor; cppcoreguidelines-noexcept-move-operations -> performance-noexcept-move-constructor; cppcoreguidelines-use-default-member-init -> modernize-use-default-member-init. `cppcoreguidelines-init-variables` and `cppcoreguidelines-pro-type-member-init` are not aliases (they have their own pages). The `clang-analyzer-*` entries redirect to the Static Analyzer's own documentation.
- Bears on this repository: a `.clang-tidy` should list the target (`bugprone-*`) name, not the alias, and the document should say which alias names are not to be listed twice.

### clang-tidy: main documentation page
- Citation: LLVM project, "Clang-Tidy" (index page: options, NOLINT, compilation database, scripts), trunk docs. https://clang.llvm.org/extra/clang-tidy/index.html
- Kind: primary
- Read: 2026-11-25, in part (options, suppression, compile database, run-clang-tidy and clang-tidy-diff sections as returned by the fetch tool)
- Topics: j
- Claim 1: `WarningsAsErrors` only upgrades diagnostics from checks that are already enabled; it enables nothing. `HeaderFilterRegex` selects which headers' diagnostics are shown (default: all non-system headers); diagnostics in the main file always show. `NOLINT`, `NOLINTNEXTLINE` and `NOLINTBEGIN`/`NOLINTEND` take an optional parenthesised check list, and a mismatched BEGIN/END is itself an error (`clang-tidy-nolint`).
- Claim 2: `-p <build path>` reads `compile_commands.json`; `--line-filter` restricts output to JSON-given line ranges; `--export-fixes` writes a YAML for `clang-apply-replacements`; `run-clang-tidy.py` runs in parallel; `clang-tidy-diff.py` reports only changed lines but still analyses whole files, so it does not speed anything up. The page says nothing general about databases produced by GCC; its only example of foreign flags is NVCC, handled with `--removed-arg` / `RemovedArgs` and `--extra-arg`.
- Bears on this repository: a command-checkable rule is `run-clang-tidy -p <dir> -warnings-as-errors=<list>` with an explicit short check list, and a NOLINT must name its check (`NOLINT(check-name)`), which a grep can verify.

### bugprone-use-after-move
- Citation: LLVM project, "bugprone-use-after-move", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/use-after-move.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags a use of a local or parameter that is reachable after `std::move` (also double `std::forward`), across loop iterations, and unsequenced move and use in one expression; reinitialisation (assignment, `clear()`, `reset()`, `[[clang::reinitializes]]`) clears it. Moves of members and globals are not checked.
- Claim 2: Documented false positive: it is not path-sensitive, so mutually exclusive branches still warn.
  Quote: "In this case, the check will erroneously produce a warning, even though it is not possible for both the move and the use to be executed."
- Bears on this repository: a real-defect check worth enabling (agents often move into containers and keep using the variable), with a short NOLINT list expected for exclusive branches. Envoy enables exactly this one among its `bugprone-*` (see Envoy block).

### bugprone-unchecked-optional-access
- Citation: LLVM project, "bugprone-unchecked-optional-access", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/unchecked-optional-access.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `value()`, `operator*`, `operator->` on `std::optional` (and absl/folly/base variants) unless a `has_value()` check or guaranteed construction is visible on every path in the same function.
  Quote: "The check relies on local reasoning."
- Claim 2: Limits: flow-sensitive so heavier on CPU and RAM than most checks; accesses inside lambdas are not currently reported; caller-side checks do not count; option `BoolSolverModel=Simple` is faster but gives more false positives.
- Bears on this repository: the repository returns optionals from perception/IK/planning; a yes/no rule "no `bugprone-unchecked-optional-access` findings" is checkable, but expect extra CPU time in CI. gRPC lists it among checks "not activated yet" because of existing findings.

### bugprone-narrowing-conversions (alias cppcoreguidelines-narrowing-conversions)
- Citation: LLVM project, "bugprone-narrowing-conversions", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/narrowing-conversions.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Finds int->narrower int, int->float, float->int and double->float conversions without an explicit cast, including compound assignments like `i += 0.1`; the page says it finds more than `-Wconversion` because it also considers promotion of small types. `cppcoreguidelines-narrowing-conversions` is only an alias.
- Claim 2: Options of note for numerical code: `WarnOnIntegerToFloatingPointNarrowingConversion`, `WarnOnFloatingPointNarrowingConversion`, `WarnOnEquivalentBitWidth` (all default true), `PedanticMode` (default false; `int i = 1.0;` only warned in pedantic mode), `IgnoreConversionFromTypes`. Narrowing inside template instantiations is not reported by default (`WarnWithinTemplateInstantiation=false`), which matters for Eigen-heavy code.
- Bears on this repository: the likeliest real-defect check for index/size/double mixing in the IK, planner and CSV code, but also the noisiest; gRPC defers it for existing findings. Candidate for the "enable one at a time, ratchet" list.

### bugprone-integer-division
- Citation: LLVM project, "bugprone-integer-division", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/integer-division.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags integer division whose result is used in a floating-point context (`d = 8 * floatFunc(1 + 7 / 2);`, `d = i / (1 << 4);`). It deliberately ignores divisions that sit inside integral operands, integral-typed calls, or explicit casts to integer.
- Claim 2: The page lists no false positives other than those exclusions.
- Bears on this repository: cheap, low-noise and a classic defect in code that computes step counts and averages; a good first check for the numerical code.

### bugprone-implicit-widening-of-multiplication-result
- Citation: LLVM project, "bugprone-implicit-widening-of-multiplication-result", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/implicit-widening-of-multiplication-result.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `int * int` whose result is then implicitly widened (to `long`, or as a pointer offset), because the multiplication has already overflowed in the narrow type; the fix-it casts an operand first. The page says the issue mostly matters for buffers of 4 GB or more.
- Claim 2: `IgnoreConstantIntExpr` (default false) suppresses warnings when operands are compile-time constants that fit.
- Bears on this repository: depth images and point clouds in `row * width + col` index arithmetic are within int range at the sizes of this repository, so this check is likely mostly noise here; gRPC also defers it.

### bugprone-incorrect-roundings
- Citation: LLVM project, "bugprone-incorrect-roundings", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/incorrect-roundings.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `(int)(x + 0.5)` as rounding. The page gives the failures: 0.499999975 (smallest float below 0.5) rounds to 1.0, and -0.5f and -1.4f both come out 0.0.
- Claim 2: The page documents no limits or false positives.
- Bears on this repository: a real numerical defect with a one-line fix (`std::lround`); worth enabling once, since pixel and grid-cell indexing from depth data is where this idiom appears.

### bugprone-misplaced-widening-cast
- Citation: LLVM project, "bugprone-misplaced-widening-cast", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/misplaced-widening-cast.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `(long)(x * 1000)` where the arithmetic is done in the narrow type before the cast; fix is `(long)x * 1000`. `CheckImplicitCasts` (default false) extends it to implicit widening.
- Claim 2: Integer only.
  Quote: "Currently warnings are only written for integer conversion."
  So `(double)(x * 10.0f)` (float arithmetic then widened) is NOT caught; `-Wdouble-promotion` is the tool for that.
- Bears on this repository: limited value for the float-heavy code; cover float-to-double promotion with the compiler warning instead.

### bugprone-too-small-loop-variable
- Citation: LLVM project, "bugprone-too-small-loop-variable", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/too-small-loop-variable.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `for` loops whose counter type cannot represent the bound (a `short` counter against a `long` bound is an infinite loop).
- Claim 2: Does not inspect compile-time constant bounds (the page recommends `-Wtautological-constant-out-of-range-compare`); `MagnitudeBitsUpperLimit` (default 16) sets which types are ignored.
- Bears on this repository: a rare defect in modern code; low cost, may stay in the list.

### bugprone-exception-escape
- Citation: LLVM project, "bugprone-exception-escape", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/exception-escape.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Finds functions that can throw (directly or through callees) although they must not: destructors, move constructors/assignments, `main()`, `swap`, and anything `noexcept`.
- Claim 2: Cost warning.
  Quote: "WARNING! This check may be expensive on large source files."
  Defaults assume an undefined callee does not throw (`TreatFunctionsWithoutSpecificationAsThrowing=None`).
- Bears on this repository: matters because nlohmann_json and OMPL/Boost throw; `main()` of the CSV tools and the gate test should not let exceptions escape. gRPC disables it, citing a LLVM issue about wrong results (comment in its `.clang-tidy`).

### cert-flp30-c and bugprone-float-loop-counter
- Citation: LLVM project, "cert-flp30-c" and "bugprone-float-loop-counter", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/cert/flp30-c.html and https://clang.llvm.org/extra/clang-tidy/checks/bugprone/float-loop-counter.html
- Kind: primary
- Read: 2026-11-25, whole (both pages are very short)
- Topics: j
- Claim 1: `cert-flp30-c` is an alias of `bugprone-float-loop-counter`, which flags `for` loops whose induction expression has floating-point type.
- Claim 2: Neither page documents limits.
- Bears on this repository: checkable ("no `for (double t = ...; ...; t += dt)`"); see the CERT block for why. Planner interpolation and trajectory sampling loops are where it would fire.

### CERT FLP30-C
- Citation: SEI CERT, "FLP30-C. Do not use floating-point variables as loop counters", SEI CERT C Coding Standard (current site). https://cmu-sei.github.io/secure-coding-standards/sei-cert-c-coding-standard/rules/floating-point-flp/flp30-c
- Kind: primary
- Read: 2026-11-25, in part (text pulled by curl and grepped: examples, compliant solutions, risk table; not the full tool table)
- Topics: j
- Claim 1: The non-compliant `for (float x = 0.1f; x <= 1.0f; x += 0.1f)` "may iterate 9 or 10 times"; with GCC on x86 the page says it runs nine times. A second example adds an increment too small to change the value, giving an infinite loop on many implementations.
- Claim 2: Compliant form: an integer counter from which the floating value is computed (`x = i * 0.1f`), which the page says also reduces compounded rounding error. Severity Low, likelihood Probable, detectable and repairable.
- Bears on this repository: rule wording that a command can check: "loop counters have integral type; sample times are computed as `t0 + i * dt`".

### bugprone-sizeof-expression
- Citation: LLVM project, "bugprone-sizeof-expression", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/sizeof-expression.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `sizeof` on constants, `this`, pointers where an array/object was meant, `sizeof(buf)/sizeof(int)` mismatches, `sizeof` of `sizeof`, and pointer arithmetic scaled by `sizeof`; it implements CERT ARR39-C (alias `cert-arr39-c`).
- Claim 2: Limit: pointee types of 1 byte (char) are excluded; several options default off (`WarnOnSizeOfPointer`, `WarnOnSizeOfIntegerExpression`).
- Bears on this repository: relevant only for raw buffers handed to MuJoCo/GLFW C APIs (e.g. `mjtNum` arrays, depth buffers); gRPC defers it for findings.

### bugprone-dangling-handle
- Citation: LLVM project, "bugprone-dangling-handle", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/dangling-handle.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `std::string_view`/`std::span` built from a temporary or local that dies first (assigning from a temporary string, returning a view of a local).
- Claim 2: By default only `std::basic_string_view`, `std::experimental::basic_string_view` and `std::span` count as handles; other handle types need `HandleClasses` (Envoy sets it for `absl::string_view`). Eigen `Map`/`Ref` and expression templates are not covered.
- Bears on this repository: real-defect check; cheap. It does not cover the Eigen expression-lifetime problem (see Eigen block).

### bugprone-signed-char-misuse
- Citation: LLVM project, "bugprone-signed-char-misuse", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/signed-char-misuse.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `signed char` assigned to an integer, compared with `unsigned char` by `==`/`!=`, or used as array subscript; `cert-str34-c` is an alias that sets `DiagnoseSignedUnsignedCharComparisons=false`.
- Claim 2: Whether plain `char` is signed depends on the target (`-fsigned-char`/`-funsigned-char`).
- Bears on this repository: matters at the boundary where 8-bit depth/colour image buffers are read; narrow use.

### bugprone-unchecked-string-to-number-conversion (alias cert-err34-c)
- Citation: LLVM project, "bugprone-unchecked-string-to-number-conversion", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/bugprone/unchecked-string-to-number-conversion.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags `atoi()`/`scanf()`-style conversions that cannot report errors (CERT ERR34-C); `strtol` family that report errors are not flagged. The page does not say whether `atof`/`strtod` are covered.
- Claim 2: No options or limits documented.
- Bears on this repository: matters for the CSV and JSON/command-line paths; a grep for `atoi|atof|stoi` is an equivalent yes/no rule, and `std::stod` throws instead.

### cppcoreguidelines-init-variables
- Citation: LLVM project, "cppcoreguidelines-init-variables", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines/init-variables.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags local integer, bool, float, double and pointer variables declared without an initialiser (enums warned but with no fix-it); the fix-it uses zero, `nullptr`, and NaN for floating types.
- Claim 2: Only those types are checked; it matches C++ Core Guidelines ES.20.
- Bears on this repository: the NaN default is a good fit for numerics (a missed assignment poisons results visibly) but the fix-it should be reviewed, since `= 0` on a double is what many agents would write by hand. Compiler `-Wuninitialized` / `-Wmaybe-uninitialized` already cover flow-based cases in GCC.

### cppcoreguidelines-pro-type-member-init
- Citation: LLVM project, "cppcoreguidelines-pro-type-member-init", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/cppcoreguidelines/pro-type-member-init.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags user-provided constructors that leave builtin/pointer fields uninitialised and automatic record variables with such fields; fix-it uses in-class `{}` initialisers. `IgnoreArrays` (default false) exempts arrays for performance-critical code.
- Claim 2: Documented false positive: fields initialised by methods the constructor calls are still reported.
- Bears on this repository: aimed at structs of joint values, poses and config; high value for result structs filled field by field.

### misc-redundant-expression
- Citation: LLVM project, "misc-redundant-expression", trunk docs. https://clang.llvm.org/extra/clang-tidy/checks/misc/redundant-expression.html
- Kind: primary
- Read: 2026-11-25, whole
- Topics: j
- Claim 1: Flags identical operands on both sides of an operator (`p->x == p->x`, `speed - speed + 1 == 12`), usually copy-paste errors in comparisons.
- Claim 2: Exempts the NaN idiom `F != F && F == F` (but flags `F == F` alone); no other limits documented.
- Bears on this repository: cheap, finds copy-paste defects in per-axis code (x/y/z limits); keep.

### Clang Static Analyzer: Available Checkers
- Citation: LLVM project, "Available Checkers", Clang Static Analyzer docs. https://clang.llvm.org/docs/analyzer/checkers.html
- Kind: primary
- Read: 2026-11-25, in part (first ~100,000 characters of 171,000: intro, default checkers; the alpha section was not reached)
- Topics: j
- Claim 1: `core` (DivideZero, NullDereference, `uninitialized.Assign`, `uninitialized.UndefReturn`) must always be on because other checkers rely on it; `cplusplus.Move` (use-after-move), `cplusplus.NewDelete` (double-free/use-after-free) and `optin.cplusplus.UninitializedObject` (uninitialised fields after a constructor) are the C++ ones that matter here. `core.DivideZero` is integer division; float division by zero is not its subject.
- Claim 2: Experimental `alpha` checkers are off by default and "may crash or emit a higher number of false positives" (as the fetch tool relayed the intro; I did not read the alpha section itself).
- Bears on this repository: `clang-analyzer-*` is path-sensitive and slow; as a first step enable only `clang-analyzer-core.*` and `clang-analyzer-cplusplus.*`. Envoy keeps `clang-analyzer-core.DivideZero` while excluding `clang-analyzer-optin.cplusplus.UninitializedObject` and `core.NonNullParamChecker` (Envoy block).

### GCC -fanalyzer: manual and release notes for GCC 13, 14, 15
- Citation: GCC project, "Static Analyzer Options" (GCC 14.2 manual) https://gcc.gnu.org/onlinedocs/gcc-14.2.0/gcc/Static-Analyzer-Options.html ; "GCC 13 Release Series Changes" https://gcc.gnu.org/gcc-13/changes.html ; "GCC 14 Release Series Changes" https://gcc.gnu.org/gcc-14/changes.html ; "GCC 15 Release Series Changes" https://gcc.gnu.org/gcc-15/changes.html
- Kind: primary
- Read: 2026-11-25, in part (the sentences on `-fanalyzer` and the GCC 14 analyzer section, by grep)
- Topics: j
- Claim 1: The GCC 13, 14 and 15 release-note pages each carry the same sentence about C++.
  Quote: "In particular, using it on C++ is unlikely to give meaningful output."
  The 14.2 manual says the analyzer "is only suitable for use on C code in this release".
- Claim 2: The manual calls it "much more expensive than other GCC warnings", "neither sound nor complete" and a bug-finding tool; GCC 14 added taint warnings enabled by default and `-Wanalyzer-infinite-loop` and others, all C-oriented.
- Bears on this repository: do not add `-fanalyzer` to a C++20 build (see Does not fit). Static analysis for C++ here has to come from `clang-tidy`/Clang Static Analyzer run over the compile database.

### GCC Instrumentation Options (sanitizers under GCC)
- Citation: GCC project, "Program Instrumentation Options", GCC 14.2 manual. https://gcc.gnu.org/onlinedocs/gcc-14.2.0/gcc/Instrumentation-Options.html
- Kind: primary
- Read: 2026-11-25, in part (the `-fsanitize=address|thread|leak|undefined|float-*` entries and `-fsanitize-recover`)
- Topics: j
- Claim 1: GCC implements ASan, TSan, LSan and UBSan. ASan and TSan cannot be combined with each other (TSan also not with LSan); ASan enables use-after-scope.
- Claim 2: `-fsanitize=float-divide-by-zero` and `-fsanitize=float-cast-overflow` are NOT part of `-fsanitize=undefined` and must be named explicitly. Reason for the first: "floating-point division by zero can be a legitimate way of obtaining infinities and NaNs." The second "does not work well with FE_INVALID exceptions enabled".
- Bears on this repository: with GCC the CI sanitizer job can use `-fsanitize=address,undefined` plus `-fsanitize=float-cast-overflow` (float to int index/pixel conversions); float-divide-by-zero would fire on deliberate NaN/inf code, so use it only in a run where NaN/inf is an error. Not tested here (no compiler runs allowed).

### Clang AddressSanitizer
- Citation: LLVM project, "AddressSanitizer", Clang docs. https://clang.llvm.org/docs/AddressSanitizer.html
- Kind: primary
- Read: 2026-11-25, in part (as returned by the fetch tool, then three sentences checked by curl)
- Topics: j
- Claim 1: Finds heap/stack/global out-of-bounds, use-after-free, use-after-return, use-after-scope, double/invalid free, and leaks.
  Quote: "Typical slowdown introduced by AddressSanitizer is 2x."
- Claim 2: Partial instrumentation is allowed but has a documented catch.
  Quote: "If the binary is partially AddressSanitizer instrumented, these checks can cause false positives."
  (this refers to container-overflow checks). Static linking of executables is unsupported; it reserves 16+ TB of virtual address space.
- Bears on this repository: ASan can run only on test binaries; uninstrumented libraries (MuJoCo, OMPL, Boost, GLFW prebuilt) are fine for ASan, but Eigen/`std::vector` annotations may give container-overflow false positives when a vector crosses the instrumented boundary.

### google/sanitizers wiki: ASan performance numbers
- Citation: Google, "AddressSanitizerPerformanceNumbers", google/sanitizers wiki. https://raw.githubusercontent.com/wiki/google/sanitizers/AddressSanitizerPerformanceNumbers.md (rendered at https://github.com/google/sanitizers/wiki/AddressSanitizerPerformanceNumbers)
- Kind: primary
- Read: 2026-11-25, in part (first 40 lines: SPEC table and summary)
- Topics: j
- Claim 1: On SPEC CPU2006 with Clang 3.3 (2013) the average ASan slowdown was 1.93x, median 1.86x, range 1.12x (libquantum) to 3.79x (perlbench); numerical C++ codes such as 433.milc, 444.namd, 447.dealII, 470.lbm were 1.24x to 1.86x.
- Claim 2: Older measurements show instrumenting only writes cut the average slowdown from 1.73x to 1.26x.
- Bears on this repository: a 2013 measurement on SPEC, not on this codebase; the planning "gate" test would be expected to be 2x-ish slower under ASan, which sizes a CI timeout but is not measured here.

### Clang UndefinedBehaviorSanitizer
- Citation: LLVM project, "UndefinedBehaviorSanitizer", Clang docs. https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html
- Kind: primary
- Read: 2026-11-25, in part (the check list entries named, as returned by the fetch tool; one sentence checked by curl)
- Topics: j
- Claim 1: `float-divide-by-zero` is outside the `undefined` group because in Clang the result is inf or NaN; `float-cast-overflow` (float to integer overflow) is in the `undefined` group in Clang's description, whereas GCC's manual says it is not in `undefined`, so list both options explicitly in either compiler. Also in `undefined`: signed-integer-overflow, shift, null, alignment; `bounds` (array-bounds yes, local-bounds no).
- Claim 2: `-fno-sanitize-recover=` makes the first failure abort with a report; `-fsanitize-trap=` traps without runtime.
  Quote: "The checks have small runtime cost and no impact on address space layout or ABI."
- Bears on this repository: UBSan is the cheapest run-time tool and needs no re-instrumentation of dependencies; the rule "tests pass under `-fsanitize=undefined -fno-sanitize-recover=all`" is checkable. Note the Clang/GCC discrepancy on float-cast-overflow: name it explicitly.

### Clang ThreadSanitizer
- Citation: LLVM project, "ThreadSanitizer", Clang docs. https://clang.llvm.org/docs/ThreadSanitizer.html
- Kind: primary
- Read: 2026-11-25, in part (as returned, key sentences checked by curl)
- Topics: j
- Claim 1: Detects data races; "slowdown ... about 5x-15x" and memory about 5x-10x.
- Claim 2: Instrumentation must be (nearly) total.
  Quote: "ThreadSanitizer generally requires all code to be compiled with -fsanitize=thread."
  Pre-compiled libraries may cause missed races or false positives; static linking of libc/libstdc++ and non-PIE executables are unsupported.
- Bears on this repository: only worth running if the repository has threads (OMPL planner threads, render thread); with Bazel-built deps from source it is possible, but prebuilt MuJoCo is uninstrumented. A TSan job is conditional, not default.

### Clang MemorySanitizer
- Citation: LLVM project, "MemorySanitizer", Clang docs. https://clang.llvm.org/docs/MemorySanitizer.html
- Kind: primary
- Read: 2026-11-25, in part (as returned, key sentences checked by curl)
- Topics: j
- Claim 1: Detects use of uninitialised memory; typical slowdown 3x.
  Quote: "MemorySanitizer requires that all program code is instrumented."
- Claim 2: This includes libraries; the page describes Clang being bootstrapped against a self-built instrumented libc++. The page does not mention GCC.
- Bears on this repository: not usable here (Clang-only per this page, needs instrumented libstdc++/Boost/MuJoCo); use `-Wmaybe-uninitialized`, `cppcoreguidelines-init-variables`, and ASan/Valgrind-like runs instead.

### C++ Core Guidelines ES.20 and ES.46
- Citation: Stroustrup, Sutter (eds.), "C++ Core Guidelines", sections ES.20 and ES.46, master branch of isocpp/CppCoreGuidelines. https://raw.githubusercontent.com/isocpp/CppCoreGuidelines/master/CppCoreGuidelines.md
- Kind: primary
- Read: 2026-11-25, in part (ES.20 and ES.46 sections)
- Topics: j
- Claim 1: ES.20 "Always initialize an object": reason is to avoid used-before-set errors and ease refactoring; the rule is "deliberately stronger" than the language rule that an object must be set before use.
- Claim 2: ES.46 enforcement text is candid about noise: "A good analyzer can detect all narrowing conversions. However, flagging all narrowing conversions will lead to a lot of false positives." It suggests flagging float-to-integer conversions first; `gsl::narrow_cast` and `gsl::narrow` express intent.
- Bears on this repository: supports enabling the narrowing check with float-to-int first and ratcheting; repository has no GSL, so intent would be an explicit `static_cast` plus a short comment.

### Eigen: Common pitfalls
- Citation: Eigen project, "Common pitfalls" (TopicPitfalls), docs for 5.0.1-dev nightly. https://eigen.tuxfamily.org/dox/TopicPitfalls.html (redirects to libeigen.gitlab.io docs-nightly)
- Kind: primary
- Read: 2026-11-25, in part (auto, ternary, alignment, pass-by-value sections)
- Topics: j
- Claim 1: `auto` with an Eigen expression stores an expression object holding references to its operands, so the product is recomputed on each use, changes when operands change, and can dangle (`auto C = ((A+B).eval()).transpose();` references a deleted temporary).
  Quote: "do not use the auto keywords with Eigen's expressions, unless you are 100% sure about what you are doing"
- Claim 2: The ternary operator with expression operands converts the else branch to the then-branch's expression type and can silently return a wrong result (`B = 3,2,1` example). Alignment problems are runtime assertions; pre-C++17 alignment was the cause, and templates taking Eigen types by value reintroduce it.
- Bears on this repository: none of the surveyed clang-tidy pages documents a check for `auto` over Eigen expressions (`bugprone-dangling-handle` covers only string_view/span), so this needs a grep-style rule ("no `auto` initialised from an Eigen expression; write the type or `.eval()`") and ASan runs to catch the dangling case. I found no clang-tidy check page that targets it; that is a statement about the pages I read, not about all checks.

### Eigen: Aliasing
- Citation: Eigen project, "Aliasing" (TopicAliasing.dox source), master. https://gitlab.com/libeigen/eigen/-/raw/master/doc/TopicAliasing.dox
- Kind: primary
- Read: 2026-11-25, in part (first ~230 lines: definition, run-time assertion, noalias)
- Topics: j
- Claim 1: Aliasing is the same matrix on both sides of an assignment (`mat = 2*mat.transpose()`, `vec = vec.head(n)`); it cannot be detected at compile time in general, and Eigen checks only some cases at run time through an assertion for transposition ("aliasing detected during transposition"), which `EIGEN_NO_DEBUG` disables.
- Claim 2: `matB.noalias() = matA * matA` skips the temporary and must not be used when aliasing actually occurs.
- Bears on this repository: the transposition check is an `eigen_assert`, so a Debug or assertions-on test build finds it and a `-DNDEBUG`/`EIGEN_NO_DEBUG` build does not; a rule "tests build with Eigen assertions on" is checkable (`grep` the flags), and `noalias()` uses need a reviewer comment.

### .clang-tidy of MoveIt 2
- Citation: MoveIt project, `.clang-tidy`, branch main. https://raw.githubusercontent.com/moveit/moveit2/main/.clang-tidy
- Kind: primary
- Read: 2026-11-25, in part (first 3,500 characters: Checks, HeaderFilterRegex, naming options)
- Topics: j
- Claim 1: Robotics project config is an explicit allow-list starting `-*`: `performance-*` (minus `performance-enum-size`), `modernize-*` selected items, `misc-unused-parameters` (StrictMode on), and `readability-*` items including `readability-identifier-naming`. No `bugprone-*`, `cert-*`, `cppcoreguidelines-*` or `clang-analyzer-*`. `HeaderFilterRegex: ''`.
- Claim 2: So the config is mostly modernisation and style; its only defect-oriented entries are the `performance-*` group and unused parameters.
- Bears on this repository: shows what a robotics project actually enforces (style), which is not what this repository wants; a defect-first list should be built from the `bugprone-*` blocks above instead.

### .clang-tidy of LLVM
- Citation: LLVM project, top-level `.clang-tidy`, branch main. https://raw.githubusercontent.com/llvm/llvm-project/main/.clang-tidy
- Kind: primary
- Read: 2026-11-25, in part (first ~45 lines)
- Topics: j
- Claim 1: LLVM turns on `-*, clang-diagnostic-*, llvm-*, misc-*, readability-identifier-naming` and then removes `misc-const-correctness`, `misc-include-cleaner`, `misc-no-recursion`, `misc-non-private-member-variables-in-classes`, `misc-unused-parameters`, `misc-use-anonymous-namespace`; `HeaderFilterRegex: ''`.
- Claim 2: Pattern: enable a whole group, subtract the checks that do not fit with a `-name` list (the opposite of MoveIt's allow-list). It includes `misc-unused-*` except unused-parameters, and `misc-redundant-expression`, but no `bugprone-*`.
- Bears on this repository: "group minus exclusions" lets new checks arrive silently on a clang-tidy upgrade, so with a pinned version it is fine, without one it is not; an explicit allow-list is more predictable for an agent-maintained repository.

### .clang-tidy of Drake
- Citation: Drake project, `.clang-tidy` (master) and `doc/_pages/code_style_tools.md`, `doc/_pages/bazel.md`. https://raw.githubusercontent.com/RobotLocomotion/drake/master/.clang-tidy ; https://raw.githubusercontent.com/RobotLocomotion/drake/master/doc/_pages/code_style_tools.md ; https://raw.githubusercontent.com/RobotLocomotion/drake/master/doc/_pages/bazel.md
- Kind: primary
- Read: 2026-11-25, in part (the .clang-tidy head, plus grep of the two docs for tidy/sanitizer/lint lines)
- Topics: j
- Claim 1: Drake's `.clang-tidy` begins with the comment "This file is not used by CI and the checks included are not part of the Drake style guide"; it lists `clang-analyzer-*`, `clang-diagnostic-*`, `cppcoreguidelines-*`, `google-*`, `modernize-*`, `performance-*`, `readability-*` minus a few. Its style tooling docs name cpplint and clang-format, not clang-tidy.
- Claim 2: Sanitizers are used through Bazel configs: the doc shows `bazel test --config=debug --config=clang --config=asan common:polynomial_test` and a `--config=memcheck` (valgrind) variant. Drake builds its ASan run with Clang there.
- Bears on this repository: a leading robotics code base ships a broad `.clang-tidy` as advisory-only and gates CI on cpplint/format; supports an advisory-first stage, and a Bazel `--config=asan` in `.bazelrc` is the pattern to copy (GCC can do it as well).

### gRPC .clang-tidy: disabled checks and why
- Citation: gRPC project, `.clang-tidy`, branch master (last commit touching it: 048ea42). https://raw.githubusercontent.com/grpc/grpc/master/.clang-tidy
- Kind: primary
- Read: 2026-11-25, in part (first ~130 lines: the comment block and the Checks list)
- Topics: j
- Claim 1: gRPC enables `bugprone-*`, `google-*`, `performance-*` and `abseil-*` as groups, then subtracts checks. Its header comment separates "checks disabled on purpose" (e.g. `bugprone-exception-escape` with a LLVM issue link, `bugprone-reserved-identifier` for portability macros) from "checks which will be enabled in future ... not activated yet due to the existing issues": this deferred list includes bugprone-narrowing-conversions, -implicit-widening-of-multiplication-result, -signed-char-misuse, -sizeof-expression, -too-small-loop-variable and -unchecked-optional-access.
- Claim 2: That is the project's own record of the adoption order: the checks that find numeric/conversion defects are exactly the ones deferred because of existing findings, while the structural ones are on.
- Bears on this repository: this is the in-source model for a per-check backlog: a comment block listing "deferred, with reason" next to `-check` lines, which a script can verify (every disabled check has a comment line).

### Envoy .clang-tidy and CI target
- Citation: Envoy project, `.clang-tidy` (main, last commit touching it c8f5228) and `ci/do_ci.sh`, `ci/README.md`. https://raw.githubusercontent.com/envoyproxy/envoy/main/.clang-tidy ; https://raw.githubusercontent.com/envoyproxy/envoy/main/ci/do_ci.sh ; https://raw.githubusercontent.com/envoyproxy/envoy/main/ci/README.md
- Kind: primary
- Read: 2026-11-25, in part (whole .clang-tidy head and tail; the clang-tidy case of do_ci.sh; one README line)
- Topics: j
- Claim 1: Envoy uses an allow-list (starting with exclusions of two `clang-analyzer` checks, `core.NonNullParamChecker` and `optin.cplusplus.UninitializedObject`) with `bugprone-assert-side-effect`, `bugprone-unused-raii`, `bugprone-use-after-move`, `clang-analyzer-core.DivideZero`, `misc-unused-using-decls`, many `modernize-*` and `performance-*`; `HeaderFilterRegex` is set to its own source directories (`^./source/.*|^./contrib/.…`). A comment at the end says any change to the file triggers a full clang-tidy run on all files.
- Claim 2: CI runs clang-tidy through Bazel (`bazel build --config=clang-tidy <targets>`, then collects fixes into a YAML) per target set, and the README says a single target set can be given to run only part of the tree.
- Bears on this repository: confirms the Bazel-integrated route (clang-tidy as a build config on targets) and an allow-list with a tiny number of defect checks; Envoy's route needs a Clang toolchain, which this repository lacks (see Does not fit).

### Chromium: clang-tidy in Chromium
- Citation: Chromium project, "clang-tidy" (docs/clang_tidy.md), main. https://chromium.googlesource.com/chromium/src/+/main/docs/clang_tidy.md?format=TEXT
- Kind: primary
- Read: 2026-11-25, whole (fetched and base64-decoded text; key lines checked)
- Topics: j
- Claim 1: Chromium enables a subset of checks to keep signal high; new checks need C++ owner review with a count of how often the check fires and a manually reviewed sample of 30+ findings; provisional approval can be withdrawn on "Not Useful" feedback.
- Claim 2: Legacy code is handled by only reporting on touched lines in code review.
  Quote: "making Chromium clang-tidy clean is an explicit non-goal"
  Comments added only to silence the tool are "actively discouraged".
- Bears on this repository: gives the checkable shape of a ratchet ("new findings in changed lines fail, old findings do not") and a NOLINT-abuse rule (a NOLINT must name the check and carry a reason); it also warns against a whole-code clean-up goal.

### Linux kernel mailing list: clang-tidy on a GCC build
- Citation: Nick Desaulniers (LLVM/kernel developer) reply in thread "[PATCH 1/2] clang-tools: Generate clang compatible output even with gcc builds", LKML, 2022-06-28. https://lkml.iu.edu/hypermail/linux/kernel/2206.3/05984.html
- Kind: secondary (a mailing-list reply by one developer, not project documentation)
- Read: 2026-11-25, in part (the top of the reply)
- Topics: j
- Claim 1: A compilation database from a GCC build can contain GCC-only options that clang-based tools do not understand; the patch under discussion proposed a `--gcc` mode to make the generated database clang-compatible.
- Claim 2: The reviewer's position: keep `compile_commands.json` unmodified and, only for the tidy/analyzer targets, produce a second database with the unusable flags filtered out and pass it with `-p`.
- Bears on this repository: Bazel with GCC will emit GCC-only flags (for example `-fno-canonical-system-headers` or `-Wno-` variants); the rule is "run `clang-tidy -p <filtered dir>`, where the filtered copy is produced by a script and the original is kept", with `--extra-arg=-Wno-unknown-warning-option` and `--removed-arg`/`RemovedArgs` from the clang-tidy index page as other tools. The flags in this repository's database were not inspected here.

## Does not fit

- "Enable `cppcoreguidelines-*`, `google-*`, `modernize-*`, `readability-*` wholesale" (Drake's `.clang-tidy`, comment says not used by CI; MoveIt's list): style and modernisation checks produce hundreds of findings in numerical code (magic numbers via the `cppcoreguidelines-avoid-magic-numbers` alias, naming) that agents would "fix" with behaviour risk; the requirement is "without breaking behavior".
- "Run `-fanalyzer` on C++" (GCC 13/14/15 release notes, GCC manual): the project's own notes say it is unlikely to give meaningful output on C++; this repository is GCC-only C++20.
- "Run MemorySanitizer" (Clang MemorySanitizer docs): needs all code including libstdc++/Boost/MuJoCo instrumented and is documented for Clang only; without a Clang toolchain it cannot be used.
- "Run clang-tidy as a Bazel aspect with a Clang toolchain, as Envoy does" (Envoy `ci/do_ci.sh`): requires a Clang toolchain; this repository would instead export `compile_commands.json` from GCC and filter flags (Linux kernel thread).
- "Make the whole code base clang-tidy clean first" (contradicted by Chromium's "explicit non-goal"): for a 40,000-line repository with agents as contributors a per-check ratchet is cheaper and safer; also `bugprone-implicit-widening-of-multiplication-result` and `bugprone-exception-escape` are expensive/noisy relative to value here (gRPC defers or disables them).
- "Run ThreadSanitizer by default" (Clang ThreadSanitizer docs): 5x-15x slowdown and all code must be instrumented; keep it conditional on the repository using threads.

## Leads not followed

- Mozilla's static-analysis documentation and LLVM/kernel sanitizer docs (KASAN/UBSAN) as adoption accounts: not fetched.
- Abseil, MuJoCo, OMPL, Pinocchio, Ceres `.clang-tidy`: the raw URLs returned 404 (at the paths tried), so they were not cited.
- SEI CERT C++ pages (FLP30-CPP, INT rules): the C++ page timed out ("socket hang up"); only the C rule FLP30-C was read. CERT INT rules not read.
- Pages of `bugprone-suspicious-*`, `bugprone-undefined-memory-manipulation`, `cert-err33-c`/`bugprone-unused-return-value`, `cert-err58-cpp`, `cert-msc50/51`, `cppcoreguidelines-slicing`, `misc-unused-*`, `clang-tidy concurrency-*`, `portability-*`: not read (the alias targets of several are named in the alias-table block only).
- Clang Static Analyzer "alpha" section and `optin` default status: not read.
- google/sanitizers wiki pages on TSan/MSan performance and partial instrumentation: not read.
- Eigen "Unaligned array assert" page: not read.
- Clang/clang-tidy baseline tools (e.g. `clang-tidy-diff.py` source, `--line-filter` usage reports) beyond the index page: not read.
