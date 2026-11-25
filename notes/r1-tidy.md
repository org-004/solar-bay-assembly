# clang-tidy counts at B (agent tidy)

Tool: `tools/architecture/tidy.sh` (with `tidy.py`). Versions: 18.1.8 (Debian 13 host, GCC 14 headers) and 18.1.3 (sba-ci, Ubuntu 24.04, GCC 13.3).
Raw outputs: `/root/work/013-design-principles-and-r1/out/tidy/` (`host-noan.txt`, `host-an.txt`, `dock.txt`, `raw-*` per unit, `table.md`, `clean.txt`).

## Things the lead must know first

- Candidate set counted (98 translation units, each `.cc` of src/ and tests/):
  `-*,bugprone-*,cert-*,cppcoreguidelines-*,misc-*,performance-*,portability-*,concurrency-*,readability-misleading-indentation,readability-suspicious-call-argument,readability-container-data-pointer,hicpp-multiway-paths-covered,hicpp-signed-bitwise` plus `clang-analyzer-*` (separately, host only),
  LEFT OUT: `misc-include-cleaner`, `misc-const-correctness`, `cppcoreguidelines-avoid-magic-numbers` (style noise; I did not measure their cost, I left them out up front). 326 checks are enabled by that set (`enabled.txt`).
- Analyzer: counted under 18.1.8 only, as the lead asked (not run in the container). Its cost is small: see times.
- Aliases (switch on only one of each pair; the script prints `alias ...` lines on stderr): `bugprone-narrowing-conversions` = `cppcoreguidelines-narrowing-conversions` (52 each); `bugprone-suspicious-memory-comparison` = `cert-exp42-c` = `cert-flp37-c` (1 place); `cert-msc32-c` = `cert-msc51-cpp` (47; it is `bugprone-random-generator-seed`); `cert-flp30-c` and `clang-analyzer-security.FloatLoopCounter` find the same loops (11 and 9) but are not reported together. `misc-non-private-member-variables-in-classes` (212) and `cppcoreguidelines-...` (63) are the same check with different default options, not aliases.
- A header seen from many units counts once; a place counted under two names counts under each, so `T total` is a count of (place, check) pairs.
- The execution root's links to outside libraries (googletest, glfw, nlohmann_json) disappeared between runs because another Bazel command had rewritten it. `tidy.py` therefore rewrites `external/...` in the database to the output base's `external/` (this survives). A built tree is still required.
- Reproducibility: two runs on `src/rrt.cc src/path_timing.cc src/profile.cc` gave identical output (`out/tidy/v/a1.txt`, `a2.txt`). 18.1.8 and 18.1.3 differ in two places only (below).

## Times and memory (three jobs, under the lock, `bin/peakmem tools/architecture/tidy.sh --jobs 3 --checks ...`)

| run | wall | peak anon (whole container) |
|---|---|---|
| host 18.1.8, set without analyzer | 413 s (another run of the same had 318 s) | 3608 MiB |
| host 18.1.8, `clang-analyzer-*` alone | 378 s | 2872 MiB |
| sba-ci 18.1.3, set without analyzer | 465 s | 4226 MiB |

The whole thing at once would take about 13 min, so I did not hit the 25 min limit. The peak is of the whole container, whose idle anon memory was about 2.5 GB at the time (Bazel servers of other agents), so clang-tidy's own share is about 1.1 to 1.7 GB; nothing was killed. 4226 MiB in the container is above the 3,500 MiB line, but it is mostly the other tenants; if the lead wants a safe default, `--jobs 2`.
Slowest five (host, analyzer pass): tests/layout_check_test.cc 50.2 s, tests/check_cycle_test.cc 49.8, tests/planned_carry_test.cc 38.4, tests/rrt_test.cc 35.0, tests/layout_rows_test.cc 34.1. Slowest five (host, no analyzer): tests/check_cycle_test.cc 28.2, src/trial_runner.cc 27.1, src/print_perception.cc 23.5, tests/arm_follower_test.cc 22.2, src/print_depth.cc 21.6. Container: tests/check_cycle_test.cc 35.3, src/trial_runner.cc 29.6, src/layout_check.cc 26.6, tests/arm_follower_test.cc 25.7, src/print_perception.cc 25.1.

## Version comparison

Totals: 6619 (18.1.8) against 6621 (18.1.3). Only two checks differ by one place: `cert-err33-c` 280/281 and `cppcoreguidelines-pro-bounds-array-to-pointer-decay` 246/247 (GCC 13 against 14 standard library headers). Every other check has the same count under both. Either version is fine for counting; the baseline needs a margin of at least 1 on those two if the CI image differs from the host, or avoid those two checks (they are noise anyway).

## Table (every check with at least one place; "18.1.3 = not run" for the analyzer's checks)

| check | 18.1.8 | 18.1.3 | files | alias / twin | judgment |
|---|---|---|---|---|---|
| bugprone-assignment-in-if-condition | 1 | 1 | 1 |  | intended; src/rrt.cc:66 `else if ((free = free_at(n)))` is deliberate. Style. |
| bugprone-branch-clone | 3 | 3 | 3 |  | FINDS DEFECT-ish: src/layout_plan.cc:972 `s0.kind == "move" ? s0.path.front() : s0.path.front()` identical arms; other two are intended chains (path_timing.cc:438, plan_layout.cc:303). |
| bugprone-easily-swappable-parameters | 68 | 68 | 42 |  | noise (e.g. NormalDraw(double mean, double stddev)); not a defect. |
| bugprone-implicit-widening-of-multiplication-result | 188 | 188 | 30 |  | noise here: int*int used as index (src/arm.cc:44), sizes far from overflow. |
| bugprone-integer-division | 3 | 3 | 2 |  | src/cell.cc:105 `i / kParkingColumns` is intended row index; tests/rectangle_fit_test.cc:500 likewise. False positives. |
| bugprone-macro-parentheses | 2 | 2 | 1 |  | test macro NEED_COMMITTED (tests/plan_replay_test.cc:472); noise. |
| bugprone-misplaced-widening-cast | 2 | 2 | 2 |  | src/print_plan.cc:441 static_cast<long long>(int64 sum): harmless, long==int64 on both targets. |
| bugprone-narrowing-conversions | 52 | 52 | 22 | same as cppcoreguidelines-narrowing-conversions | size_t/mjtSize to int or double (src/module_finder.cc:90 `values.size()` to double, src/collision.cc:205 mjtSize to int): no figure lost at these sizes; noise, but it is the "narrowing" class. |
| bugprone-suspicious-memory-comparison | 1 | 1 | 1 | reported with cert-exp42-c, cert-flp37-c | tests/depth_camera_test.cc:61 memcmp of floats: NaN-safe by intent (bit equality); not a defect. |
| bugprone-unchecked-optional-access | 879 | 879 | 43 |  | 810 of 879 are in tests after ASSERT_TRUE(opt) (GoogleTest macro: false positive). 69 in src (e.g. src/cell.cc:231 `suction_->Release`): optional members set at Load; unread for real defect. |
| cert-dcl50-cpp | 6 | 6 | 6 |  | C-style variadic helpers calling vsnprintf (src/module_finder.cc:36); noise. |
| cert-err33-c | 280 | 281 | 50 |  | ignored printf/fclose return values (src/bay_model.cc:49); noise. |
| cert-err34-c | 60 | 60 | 17 |  | REAL class: atoi/atof/sscanf in CLI parsing (src/run_cell.cc:490-500, src/print_plan.cc:179-201): a typo such as `--seed 3x` or `--seed abc` silently becomes 0. Not a core-path defect. |
| cert-err58-cpp | 566 | 566 | 44 |  | static-init of Eigen/std::string constants (src/module_finder.cc:30 kUp); also GoogleTest TEST objects. Noise. |
| cert-exp42-c | 1 | 1 | 1 | alias of bugprone-suspicious-memory-comparison | alias, see bugprone-suspicious-memory-comparison. |
| cert-flp30-c | 11 | 11 | 6 |  | FINDS REAL RISK: double loop counters, src/check_placement.cc:180 `for (z = lens.z(); z <= top; z += 0.02)` may or may not visit the end point depending on rounding. |
| cert-flp37-c | 1 | 1 | 1 | alias of bugprone-suspicious-memory-comparison | alias, see bugprone-suspicious-memory-comparison. |
| cert-msc32-c | 47 | 47 | 13 | same as cert-msc51-cpp | constant RNG seeds (46 of 47): deliberate (reproducible tests); noise. |
| cert-msc51-cpp | 47 | 47 | 13 | same as cert-msc32-c (is bugprone-random-generator-seed) | same finding as cert-msc32-c. |
| clang-analyzer-core.CallAndMessage | 2 | not run | 2 |  | false positive: src/container.h:163 `ends[end]` is in range (end is 0..1, loop of 2); analyzer loses the count in the braced for. |
| clang-analyzer-cplusplus.StringChecker | 3 | not run | 3 |  | REAL but test-only: tests/arm_follower_test.cc:718 `std::string path = std::getenv("MODEL_PATH")` is undefined behavior if the variable is unset (Bazel always sets it; running the binary by hand does not). |
| clang-analyzer-deadcode.DeadStores | 1 | not run | 1 |  | src/run_cell.cc:179 `status = "-"` overwritten in every switch arm; harmless. |
| clang-analyzer-optin.performance.Padding | 6 | not run | 5 |  | struct padding in test logs (tests/pick_place_test.cc:173: 76 bytes of 12 optimal); noise. |
| clang-analyzer-security.FloatLoopCounter | 9 | not run | 5 |  | same finding as cert-flp30-c (9 of its 11 places), src/check_placement.cc:180. |
| concurrency-mt-unsafe | 65 | 65 | 40 |  | getenv/strerror-class calls (src/check_layout.cc:38); program is not threaded around them; noise. |
| cppcoreguidelines-avoid-c-arrays | 168 | 168 | 56 |  | style (arm.cc:11 constant tables). |
| cppcoreguidelines-avoid-const-or-ref-data-members | 25 | 25 | 8 |  | style (src/cycle_check.h:167 const Cell& cell_); not a defect. |
| cppcoreguidelines-avoid-do-while | 2 | 2 | 2 |  | style. |
| cppcoreguidelines-avoid-goto | 3 | 3 | 2 |  | tests/bay_model_test.cc:213 goto out of nested loops; style. |
| cppcoreguidelines-avoid-non-const-global-variables | 8 | 8 | 8 |  | src/bay_model.cc:241 non-const namespace-scope table; low value. |
| cppcoreguidelines-init-variables | 12 | 12 | 2 |  | uninitialized locals later set through out-parameter (src/layout_check.cc:705 margin set by Margin()), or by loop (portable_random.h:80): all set before use; read, not defects. |
| cppcoreguidelines-macro-usage | 2 | 2 | 2 |  | style (src/layout.cc:131 FIELD). |
| cppcoreguidelines-narrowing-conversions | 52 | 52 | 22 | same as bugprone-narrowing-conversions | twin name of bugprone-narrowing-conversions. |
| cppcoreguidelines-non-private-member-variables-in-classes | 63 | 63 | 15 | twin of misc-non-private-member-variables-in-classes (own counts: differing defaults) | style (public data in structs). |
| cppcoreguidelines-owning-memory | 554 | 554 | 44 |  | FILE* from fopen (src/print_check.cc:129): noise. |
| cppcoreguidelines-prefer-member-initializer | 1 | 1 | 1 |  | src/cycle_check.cc:83 tool_ set in body; style. |
| cppcoreguidelines-pro-bounds-array-to-pointer-decay | 246 | 247 | 41 |  | noise (string literals, GoogleTest macros). |
| cppcoreguidelines-pro-bounds-constant-array-index | 437 | 437 | 45 |  | noise. |
| cppcoreguidelines-pro-bounds-pointer-arithmetic | 1311 | 1311 | 67 |  | noise (MuJoCo arrays, 1311 places). |
| cppcoreguidelines-pro-type-const-cast | 6 | 6 | 5 |  | src/cycle_check.cc:307 const_cast<mjData*> to call mj_geomDistance, which restores state: a deliberate cast; real smell. |
| cppcoreguidelines-pro-type-member-init | 23 | 23 | 15 |  | FINDS maybe: uninitialized struct members, e.g. src/ik.cc:58 Kinematics.jacobian (Eigen), src/layout_plan.cc:853 Other{arm,c,segment}; all assigned before reading as far as read. |
| cppcoreguidelines-pro-type-reinterpret-cast | 1 | 1 | 1 |  | src/layout_replay.cc:514 char buffer to unsigned char for hashing: fine. |
| cppcoreguidelines-pro-type-vararg | 1074 | 1074 | 64 |  | printf and GoogleTest; noise (1074). |
| cppcoreguidelines-special-member-functions | 5 | 5 | 5 |  | rule of five (src/layout_scene.h:242); noise. |
| hicpp-signed-bitwise | 50 | 50 | 12 |  | signed operands of | and &, e.g. MuJoCo flags (src/collision.cc:349); well-defined for the values used; noise. |
| misc-no-recursion | 3 | 3 | 1 |  | src/pick_place.cc:1387 Finish->Close chain, not unbounded; noise. |
| misc-non-private-member-variables-in-classes | 212 | 212 | 37 | twin of cppcoreguidelines-non-private-member-variables-in-classes | style (default ignores nothing), twin of the cppcoreguidelines one. |
| misc-unused-using-decls | 2 | 2 | 2 |  | tests/path_follow_test.cc:40 unused `using JointLimits`: dead code, harmless. |
| performance-enum-size | 32 | 32 | 19 |  | style/micro (src/cell.h:130). |
| performance-inefficient-string-concatenation | 13 | 13 | 8 |  | noise (diagnostic text). |
| performance-inefficient-vector-operation | 25 | 25 | 15 |  | push_back without reserve (src/layout_check.cc:1490); micro. |
| performance-unnecessary-copy-initialization | 2 | 2 | 2 |  | tests/layout_model_test.cc:1138 copies a const reference; micro. |
| performance-unnecessary-value-param | 2 | 2 | 2 |  | src/print_cell.cc:43 std::string by value; micro. |
| readability-container-data-pointer | 2 | 2 | 2 |  | src/viewer_lib.cc:242 `&v[0]` for sscanf; style, would be UB-adjacent only if empty. |

## Checks of the candidate groups that found nothing (candidates for WarningsAsErrors)

Found nothing under 18.1.8 (and, for the non-analyzer ones, under 18.1.3). The Objective-C, macOS, WebKit and Fuchsia analyzer checks are left out of the list (they cannot fire on this code). `clang-analyzer-*` were counted under 18.1.8 only.
Of note among them: `bugprone-use-after-move`, `bugprone-dangling-handle`, `bugprone-infinite-loop`, `bugprone-sizeof-expression`, `bugprone-too-small-loop-variable`, `bugprone-swapped-arguments`, `bugprone-suspicious-memset-usage`, `bugprone-undefined-memory-manipulation`, `bugprone-unused-return-value`, `bugprone-signed-char-misuse`, `bugprone-incorrect-roundings`, `bugprone-inaccurate-erase`, `bugprone-string-constructor`, `bugprone-exception-escape`, `bugprone-misplaced-pointer-arithmetic-in-alloc`, `clang-analyzer-core.NullDereference`, `clang-analyzer-core.DivideZero`, `clang-analyzer-core.UndefinedBinaryOperatorResult`, `clang-analyzer-core.uninitialized.*`, `clang-analyzer-cplusplus.Move`, `clang-analyzer-cplusplus.NewDelete*`, `clang-analyzer-unix.Malloc`, `clang-analyzer-core.StackAddressEscape`, `clang-analyzer-core.VLASize`, `clang-analyzer-optin.cplusplus.UninitializedObject`.
Caution: `bugprone-unchecked-optional-access`, `cert-err33-c`, `cert-err58-cpp` and similar are not clean and are noisy; a check being clean in tests is also partly because GoogleTest macros hide code from the analyzer.

Full list (one per line in `clean.txt`, 18.1.8; those without clang-analyzer were also clean under 18.1.3):

- 
- bugprone-argument-comment
- bugprone-assert-side-effect
- bugprone-bad-signal-to-kill-thread
- bugprone-bool-pointer-implicit-conversion
- bugprone-casting-through-void
- bugprone-chained-comparison
- bugprone-compare-pointer-to-member-virtual-function
- bugprone-copy-constructor-init
- bugprone-dangling-handle
- bugprone-dynamic-static-initializers
- bugprone-empty-catch
- bugprone-exception-escape
- bugprone-fold-init-type
- bugprone-forward-declaration-namespace
- bugprone-forwarding-reference-overload
- bugprone-inaccurate-erase
- bugprone-inc-dec-in-conditions
- bugprone-incorrect-enable-if
- bugprone-incorrect-roundings
- bugprone-infinite-loop
- bugprone-lambda-function-name
- bugprone-macro-repeated-side-effects
- bugprone-misplaced-operator-in-strlen-in-alloc
- bugprone-misplaced-pointer-arithmetic-in-alloc
- bugprone-move-forwarding-reference
- bugprone-multi-level-implicit-pointer-conversion
- bugprone-multiple-new-in-one-expression
- bugprone-multiple-statement-macro
- bugprone-no-escape
- bugprone-non-zero-enum-to-bool-conversion
- bugprone-not-null-terminated-result
- bugprone-optional-value-conversion
- bugprone-parent-virtual-call
- bugprone-posix-return
- bugprone-redundant-branch-condition
- bugprone-reserved-identifier
- bugprone-shared-ptr-array-mismatch
- bugprone-signal-handler
- bugprone-signed-char-misuse
- bugprone-sizeof-container
- bugprone-sizeof-expression
- bugprone-spuriously-wake-up-functions
- bugprone-standalone-empty
- bugprone-string-constructor
- bugprone-string-integer-assignment
- bugprone-string-literal-with-embedded-nul
- bugprone-stringview-nullptr
- bugprone-suspicious-enum-usage
- bugprone-suspicious-include
- bugprone-suspicious-memset-usage
- bugprone-suspicious-missing-comma
- bugprone-suspicious-realloc-usage
- bugprone-suspicious-semicolon
- bugprone-suspicious-string-compare
- bugprone-swapped-arguments
- bugprone-switch-missing-default-case
- bugprone-terminating-continue
- bugprone-throw-keyword-missing
- bugprone-too-small-loop-variable
- bugprone-undefined-memory-manipulation
- bugprone-undelegated-constructor
- bugprone-unhandled-exception-at-new
- bugprone-unhandled-self-assignment
- bugprone-unique-ptr-array-mismatch
- bugprone-unsafe-functions
- bugprone-unused-local-non-trivial-variable
- bugprone-unused-raii
- bugprone-unused-return-value
- bugprone-use-after-move
- bugprone-virtual-near-miss
- cert-con36-c
- cert-con54-cpp
- cert-dcl03-c
- cert-dcl16-c
- cert-dcl21-cpp
- cert-dcl37-c
- cert-dcl51-cpp
- cert-dcl54-cpp
- cert-dcl58-cpp
- cert-dcl59-cpp
- cert-env33-c
- cert-err09-cpp
- cert-err52-cpp
- cert-err60-cpp
- cert-err61-cpp
- cert-fio38-c
- cert-mem57-cpp
- cert-msc24-c
- cert-msc30-c
- cert-msc33-c
- cert-msc50-cpp
- cert-msc54-cpp
- cert-oop11-cpp
- cert-oop54-cpp
- cert-oop57-cpp
- cert-oop58-cpp
- cert-pos44-c
- cert-pos47-c
- cert-sig30-c
- cert-str34-c
- clang-analyzer-apiModeling.Errno
- clang-analyzer-apiModeling.TrustNonnull
- clang-analyzer-apiModeling.TrustReturnsNonnull
- clang-analyzer-apiModeling.google.GTest
- clang-analyzer-apiModeling.llvm.CastValue
- clang-analyzer-apiModeling.llvm.ReturnValue
- clang-analyzer-core.BitwiseShift
- clang-analyzer-core.CallAndMessageModeling
- clang-analyzer-core.DivideZero
- clang-analyzer-core.DynamicTypePropagation
- clang-analyzer-core.NonNullParamChecker
- clang-analyzer-core.NonnilStringConstants
- clang-analyzer-core.NullDereference
- clang-analyzer-core.StackAddrEscapeBase
- clang-analyzer-core.StackAddressEscape
- clang-analyzer-core.UndefinedBinaryOperatorResult
- clang-analyzer-core.VLASize
- clang-analyzer-core.builtin.BuiltinFunctions
- clang-analyzer-core.builtin.NoReturnFunctions
- clang-analyzer-core.uninitialized.ArraySubscript
- clang-analyzer-core.uninitialized.Assign
- clang-analyzer-core.uninitialized.Branch
- clang-analyzer-core.uninitialized.CapturedBlockVariable
- clang-analyzer-core.uninitialized.NewArraySize
- clang-analyzer-core.uninitialized.UndefReturn
- clang-analyzer-cplusplus.InnerPointer
- clang-analyzer-cplusplus.Move
- clang-analyzer-cplusplus.NewDelete
- clang-analyzer-cplusplus.NewDeleteLeaks
- clang-analyzer-cplusplus.PlacementNew
- clang-analyzer-cplusplus.PureVirtualCall
- clang-analyzer-cplusplus.SelfAssignment
- clang-analyzer-cplusplus.SmartPtrModeling
- clang-analyzer-cplusplus.VirtualCallModeling
- clang-analyzer-optin.core.EnumCastOutOfRange
- clang-analyzer-optin.cplusplus.UninitializedObject
- clang-analyzer-optin.cplusplus.VirtualCall
- clang-analyzer-optin.mpi.MPI-Checker
- clang-analyzer-optin.performance.GCDAntipattern
- clang-analyzer-optin.portability.UnixAPI
- clang-analyzer-security.cert.env.InvalidPtr
- clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling
- clang-analyzer-security.insecureAPI.SecuritySyntaxChecker
- clang-analyzer-security.insecureAPI.UncheckedReturn
- clang-analyzer-security.insecureAPI.bcmp
- clang-analyzer-security.insecureAPI.bcopy
- clang-analyzer-security.insecureAPI.bzero
- clang-analyzer-security.insecureAPI.decodeValueOfObjCType
- clang-analyzer-security.insecureAPI.getpw
- clang-analyzer-security.insecureAPI.gets
- clang-analyzer-security.insecureAPI.mkstemp
- clang-analyzer-security.insecureAPI.mktemp
- clang-analyzer-security.insecureAPI.rand
- clang-analyzer-security.insecureAPI.strcpy
- clang-analyzer-security.insecureAPI.vfork
- clang-analyzer-unix.API
- clang-analyzer-unix.DynamicMemoryModeling
- clang-analyzer-unix.Errno
- clang-analyzer-unix.Malloc
- clang-analyzer-unix.MallocSizeof
- clang-analyzer-unix.MismatchedDeallocator
- clang-analyzer-unix.StdCLibraryFunctions
- clang-analyzer-unix.Vfork
- clang-analyzer-unix.cstring.BadSizeArg
- clang-analyzer-unix.cstring.CStringModeling
- clang-analyzer-unix.cstring.NullArg
- clang-analyzer-valist.CopyToSelf
- clang-analyzer-valist.Uninitialized
- clang-analyzer-valist.Unterminated
- clang-analyzer-valist.ValistBase
- concurrency-thread-canceltype-asynchronous
- cppcoreguidelines-avoid-capturing-lambda-coroutines
- cppcoreguidelines-avoid-reference-coroutine-parameters
- cppcoreguidelines-c-copy-assignment-signature
- cppcoreguidelines-explicit-virtual-functions
- cppcoreguidelines-interfaces-global-init
- cppcoreguidelines-macro-to-enum
- cppcoreguidelines-misleading-capture-default-by-value
- cppcoreguidelines-missing-std-forward
- cppcoreguidelines-no-malloc
- cppcoreguidelines-no-suspend-with-lock
- cppcoreguidelines-noexcept-destructor
- cppcoreguidelines-noexcept-move-operations
- cppcoreguidelines-noexcept-swap
- cppcoreguidelines-pro-type-cstyle-cast
- cppcoreguidelines-pro-type-static-cast-downcast
- cppcoreguidelines-pro-type-union-access
- cppcoreguidelines-rvalue-reference-param-not-moved
- cppcoreguidelines-slicing
- cppcoreguidelines-use-default-member-init
- cppcoreguidelines-virtual-class-destructor
- hicpp-multiway-paths-covered
- misc-confusable-identifiers
- misc-coroutine-hostile-raii
- misc-definitions-in-headers
- misc-header-include-cycle
- misc-misleading-bidirectional
- misc-misleading-identifier
- misc-misplaced-const
- misc-new-delete-overloads
- misc-non-copyable-objects
- misc-redundant-expression
- misc-static-assert
- misc-throw-by-value-catch-by-reference
- misc-unconventional-assign-operator
- misc-uniqueptr-reset-release
- misc-unused-alias-decls
- misc-unused-parameters
- misc-use-anonymous-namespace
- performance-avoid-endl
- performance-faster-string-find
- performance-for-range-copy
- performance-implicit-conversion-in-loop
- performance-inefficient-algorithm
- performance-move-const-arg
- performance-move-constructor-init
- performance-no-automatic-move
- performance-no-int-to-ptr
- performance-noexcept-destructor
- performance-noexcept-move-constructor
- performance-noexcept-swap
- performance-trivially-destructible
- performance-type-promotion-in-math-fn
- portability-restrict-system-includes
- portability-simd-intrinsics
- portability-std-allocator-const
- readability-misleading-indentation
- readability-suspicious-call-argument

## Findings that may be real defects (read in the source)

The tool finds no use-after-move, no dangling handle, no uninitialized read, no comparison that cannot be true, and no real null dereference on the production path: those checks (above) are clean. What is left, ordered by likelihood:

1. `clang-analyzer-cplusplus.StringChecker` (3 places, tests only): `tests/arm_follower_test.cc:718:28`, `tests/layout_model_test.cc:1135:7`, `tests/motion_planner_test.cc:852:43`, "The parameter must not be null". Code at arm_follower_test.cc:718: `const std::string path = std::getenv("MODEL_PATH");`. REAL (undefined behavior): if the binary is run by hand without `MODEL_PATH` (Bazel sets it), `std::string(nullptr)` throws or crashes with no message. Cure changes no output.
2. `cert-flp30-c` / `clang-analyzer-security.FloatLoopCounter` (11 / 9 places): `src/check_placement.cc:180:41`, `746:48`, `747:49`, `774:46`, `src/layout_plan.cc:825:37`, `tests/conveyor_test.cc:467`, `tests/ik_test.cc:273`, `tests/layout_plan_test.cc:86,87`, `tests/suction_test.cc:302,303`. Read `check_placement.cc:180` (`for (double z = lens.z(); z <= top; z += 0.02)`) and `layout_plan.cc:825` (`for (double t = s; t < horizon; t += f_.sample_s) times.push_back(t)`): by rounding the last sample may or may not be taken, and the count of iterations can differ between platforms (ARM against x86 should still agree, as both are IEEE double, so risk is low); the layout_plan one then pushes `end` and `horizon` too, so a duplicate near-equal time is possible (sort follows; duplicates are not removed). REAL as a fragility; cure would change which samples are taken, so "Needs the owner".
3. `bugprone-branch-clone`, `src/layout_plan.cc:972:38`: `e.config = s0.kind == "move" ? s0.path.front() : s0.path.front();` identical arms. REAL (dead condition): the author likely meant `back()` for one arm (the start of a "move" segment against the end of another kind); today every segment gives its first config. Whether to change it is the owner's (changes the plan's printed content if it was a slip).
4. `cert-err34-c` (60 places, all CLI parsers): `src/run_cell.cc:490-500` (`--seed`, `--threads` etc. via `atoi`/`atol`/`atof`), `src/print_plan.cc:179-201`, `src/run_gate.cc:66-72`, `src/view_cell.cc:205-264`, `src/print_profile.cc:39-42`. REAL: `--seed 3x` or `--seed abc` runs with a seed of 0 (or 3), no message; a user thinks they ran seed N. Also overflow is undefined behavior for `atoi`. Test: `tests/gate_test.cc:51`, `tests/layout_check_test.cc:1025` parse the environment the same way.
5. `bugprone-narrowing-conversions` (52): `src/collision.cc:205:53`, `206:55`, `281`, `307` (mjtSize to int in `3 * m.nmocap` style counts), `src/check_placement.cc:783` (`std::min<int>(15, found.size())`), `src/motion_planner.cc:447` (`1000 + leg` with size_t leg), `src/cycle_check.cc:348`, `src/module_finder.cc:54` (`begin() + middle`). Read: sizes are at most thousands, no figure is lost; not a defect today. `src/module_finder.cc:90,91,612-635` size_t to double (counts for means): exact up to 2^53.
6. `cppcoreguidelines-pro-type-member-init` / `cppcoreguidelines-init-variables` (23 / 12): `src/ik.cc:58` `Kinematics.jacobian` (an Eigen matrix, filled by the one function that makes it), `src/layout_plan.cc:853` `Other{arm, c, segment}`, `src/layout_check.cc:359` (`perr, aerr, perrn, aerrn`, assigned through `eval(...)` out-parameters), `src/layout_check.cc:705,865,2106,2158` (`margin`, always set by `Margin()` at layout_check.cc:472 with `*margin = 1e9`), `src/layout_check.cc:113` `deg` filled by the loop, `src/rectangle_fit.cc:661` `farthest.fill(-kInfinity)` follows. All read: set before use. NOT real.
7. `cppcoreguidelines-pro-type-const-cast` (6): `src/cycle_check.cc:307` `const_cast<mjData*>(cell_.data())` passed to `mj_geomDistance` (the comment says it restores state). A smell: if the call ever writes, a "const" check reads a simulation that mutates. Not a defect today (MuJoCo's function does not modify contact state outside `d`'s scratch). Other five: `src/view_layout.cc:348,349`, tests.
8. `bugprone-unchecked-optional-access` (69 in src, 810 in tests): `src/cell.cc:231`, `268`, `281`, `292`, `294`, `src/module_finder.cc` (18), `src/pick_place.cc` (14), `src/check_placement.cc` (5), `src/check_rig.cc` (4). Read cell.cc:231 (`suction_->Release(data)`) and cell.cc:288-295 (`holding()` checked then used by `*body`): the optional is checked or an invariant. The 810 test hits are `ASSERT_TRUE(opt)` then `*opt` (the checker does not know GoogleTest). Not a defect found; the check is mostly false positives in tests, so `-header-filter` cannot save it; it would need a baseline of 879.
9. `bugprone-integer-division` (3): `src/cell.cc:105:58`, `tests/rectangle_fit_test.cc:500:50`, `501:69`. Intended (integer row/column index). NOT real.
10. `clang-analyzer-core.CallAndMessage` (2): `src/container.h:163:20`, `src/flat_rack.h:133:20`, "2nd function call argument is an uninitialized value": `std::string("container_sill_") + ends[end]` with `end` always 0 or 1. False positive of the analyzer.
11. `bugprone-misplaced-widening-cast` (2): `src/print_plan.cc:441:19`, `tests/motion_planner_test.cc:163:17`: `static_cast<long long>(a + b)` of two int64: harmless, would only overflow at 2^63.
12. `clang-analyzer-deadcode.DeadStores`, `src/run_cell.cc:179:15` (`const char* status = "-"` then every switch arm sets it): harmless.
13. `performance-unnecessary-copy-initialization`, `tests/layout_model_test.cc:1138:16`, `tests/check_cycle_test.cc:395:23`: copy of a large struct; harmless.
14. `readability-container-data-pointer`, `src/view_cell.cc:149:49`, `src/viewer_lib.cc:242:43`: `&v[0]` on a container that is sized before: fine.

So there are fewer than ten real defects: items 1 to 4 are the real ones (1 undefined behavior in tests when `MODEL_PATH` is missing, 2 fragile float loop counters, 3 a branch whose two arms are identical, 4 silent bad CLI numbers); items 7 and 8 are smells.

## Verification of the tool

Commands run from the repository root, all on `src/rrt.cc src/path_timing.cc src/profile.cc` with `--checks='-*,bugprone-*,cert-*'`:
- Two runs, `cmp` of stdout: identical (rc 0 both).
- Baseline: a made-up file `out/tidy/v/base.txt` with `T bugprone-branch-clone 0` after the `# not compared` line: `tidy.sh ... --baseline out/tidy/v/base.txt` exits 1 and prints `tidy.sh: bugprone-branch-clone has 1 places, above its baseline 0`. With the line set to 1: exit 0.
- `--warnings-as-errors=bugprone-branch-clone`: exits 1, prints `src/path_timing.cc:438:30: error: repeated branch body in conditional chain [bugprone-branch-clone,-warnings-as-errors]` and `error from a check under WarningsAsErrors: bugprone-branch-clone`.
- A missing header (the first run, before the `external/` rewrite) printed `src/layout.cc: ... 'nlohmann/json.hpp' file not found [clang-diagnostic-error]` and exited 1 with the hint to build first.
- Not done: the `.clang-tidy`'s `WarningsAsErrors` path (there is no `.clang-tidy`; the script passes it as `--config-file` when the file exists, untested).
