# clang-tidy 18.1.8 (host, Debian), 98 units, src/ and tests/ places

Commands (each under the lock, from the checkout, `out/counts/runt.py` wraps `tools/architecture/tidy.sh --checks <set> --jobs 3 --raw <dir>`; the sets are in out/counts/P1.txt and P2.txt):
  python3 out/counts/runt.py P1 out/counts/P1.txt out/counts/tidy18_8   (pass 1)
  python3 out/counts/runt.py P2 out/counts/P2.txt out/counts/tidy18_8   (pass 2, `-*,clang-analyzer-*`)
Counts, by-file lines and lists come from the raw per-unit output (out/counts/tidy18_8/raw-P1, raw-P2) read by out/counts/post_tidy.py with tidy.py's own regex and `repo_path`; the totals agree with tidy.sh's `T total` lines (6621 and 21).

## Times and memory

- Pass 1: rc=0 wall_s_inside_lock=378.2 waited_for_lock_s=0.0 memory.peak_after_MiB=7445(before=7445) max_current_MiB=7445 min_current_MiB=5386
- Pass 2: rc=0 wall_s_inside_lock=344.2 waited_for_lock_s=0.0 memory.peak_after_MiB=7445(before=7445) max_current_MiB=6610 min_current_MiB=5098
- Slowest units, pass 1: tests/check_cycle_test.cc 26.0 s, src/trial_runner.cc 24.7 s, src/print_perception.cc 21.3 s; pass 2: tests/layout_check_test.cc 47.7 s, tests/check_cycle_test.cc 47.0 s, tests/planned_carry_test.cc 36.8 s. tidy.sh's own wall_s: 378.2 (pass 1), 344.1 (pass 2). Exit code 0 both, no unit failed to parse.
- Memory: `/sys/fs/cgroup/memory.peak` is read-only here and is the whole machine's lifetime peak (7445 MiB = the 7,445 MiB limit already, before the runs). memory.current sampled each second during the run (page cache included, whole machine): pass 1 max 7445 MiB, min 5386 MiB; pass 2 max 6610 MiB, min 5098 MiB. Running at the same time: three Bazel servers (host repo, the Ubuntu container's, clone2), the orchestrating Claude processes and Docker; no compiler of another agent was seen at the start, but the baseline of 5.1-5.4 GiB is theirs. So the memory the three clang-tidy processes themselves need is not separable; the runs were not killed.

## Aliases printed by tidy.sh (one place counted under each name)

- alias bugprone-narrowing-conversions cppcoreguidelines-narrowing-conversions
- alias bugprone-suspicious-memory-comparison cert-exp42-c cert-flp37-c
- alias cert-msc32-c cert-msc51-cpp
- alias cppcoreguidelines-non-private-member-variables-in-classes misc-non-private-member-variables-in-classes

Note: `cppcoreguidelines-non-private-member-variables-in-classes` (63) and `misc-non-private-member-variables-in-classes` (212) are aliases of one check but its default options differ between the two names, so the counts differ (63 of the 212 places are named by both).

## Checks with at least one place

| check | 18.1.8 places | files | 18.1.3 places |
|---|---|---|---|
| bugprone-assignment-in-if-condition | 1 | 1 | not run (skipped by the coordinator) |
| bugprone-branch-clone | 3 | 3 | not run (skipped by the coordinator) |
| bugprone-easily-swappable-parameters | 68 | 42 | not run (skipped by the coordinator) |
| bugprone-implicit-widening-of-multiplication-result | 188 | 30 | not run (skipped by the coordinator) |
| bugprone-integer-division | 3 | 2 | not run (skipped by the coordinator) |
| bugprone-macro-parentheses | 2 | 1 | not run (skipped by the coordinator) |
| bugprone-misplaced-widening-cast | 2 | 2 | not run (skipped by the coordinator) |
| bugprone-narrowing-conversions | 52 | 22 | not run (skipped by the coordinator) |
| bugprone-suspicious-memory-comparison | 1 | 1 | not run (skipped by the coordinator) |
| bugprone-unchecked-optional-access | 879 | 43 | not run (skipped by the coordinator) |
| cert-dcl50-cpp | 6 | 6 | not run (skipped by the coordinator) |
| cert-err33-c | 280 | 49 | not run (skipped by the coordinator) |
| cert-err34-c | 60 | 17 | not run (skipped by the coordinator) |
| cert-err58-cpp | 566 | 44 | not run (skipped by the coordinator) |
| cert-exp42-c | 1 | 1 | not run (skipped by the coordinator) |
| cert-flp30-c | 11 | 6 | not run (skipped by the coordinator) |
| cert-flp37-c | 1 | 1 | not run (skipped by the coordinator) |
| cert-msc32-c | 47 | 13 | not run (skipped by the coordinator) |
| cert-msc51-cpp | 47 | 13 | not run (skipped by the coordinator) |
| clang-analyzer-core.CallAndMessage | 2 | 2 | not run (skipped by the coordinator) |
| clang-analyzer-cplusplus.StringChecker | 3 | 3 | not run (skipped by the coordinator) |
| clang-analyzer-deadcode.DeadStores | 1 | 1 | not run (skipped by the coordinator) |
| clang-analyzer-optin.performance.Padding | 6 | 5 | not run (skipped by the coordinator) |
| clang-analyzer-security.FloatLoopCounter | 9 | 5 | not run (skipped by the coordinator) |
| concurrency-mt-unsafe | 65 | 40 | not run (skipped by the coordinator) |
| cppcoreguidelines-avoid-c-arrays | 168 | 56 | not run (skipped by the coordinator) |
| cppcoreguidelines-avoid-const-or-ref-data-members | 25 | 8 | not run (skipped by the coordinator) |
| cppcoreguidelines-avoid-do-while | 2 | 2 | not run (skipped by the coordinator) |
| cppcoreguidelines-avoid-goto | 3 | 2 | not run (skipped by the coordinator) |
| cppcoreguidelines-avoid-non-const-global-variables | 8 | 8 | not run (skipped by the coordinator) |
| cppcoreguidelines-init-variables | 12 | 2 | not run (skipped by the coordinator) |
| cppcoreguidelines-macro-usage | 2 | 2 | not run (skipped by the coordinator) |
| cppcoreguidelines-narrowing-conversions | 52 | 22 | not run (skipped by the coordinator) |
| cppcoreguidelines-non-private-member-variables-in-classes | 63 | 15 | not run (skipped by the coordinator) |
| cppcoreguidelines-owning-memory | 554 | 44 | not run (skipped by the coordinator) |
| cppcoreguidelines-prefer-member-initializer | 1 | 1 | not run (skipped by the coordinator) |
| cppcoreguidelines-pro-bounds-array-to-pointer-decay | 248 | 40 | not run (skipped by the coordinator) |
| cppcoreguidelines-pro-bounds-constant-array-index | 437 | 45 | not run (skipped by the coordinator) |
| cppcoreguidelines-pro-bounds-pointer-arithmetic | 1311 | 67 | not run (skipped by the coordinator) |
| cppcoreguidelines-pro-type-const-cast | 6 | 5 | not run (skipped by the coordinator) |
| cppcoreguidelines-pro-type-member-init | 23 | 15 | not run (skipped by the coordinator) |
| cppcoreguidelines-pro-type-reinterpret-cast | 1 | 1 | not run (skipped by the coordinator) |
| cppcoreguidelines-pro-type-vararg | 1074 | 64 | not run (skipped by the coordinator) |
| cppcoreguidelines-special-member-functions | 5 | 5 | not run (skipped by the coordinator) |
| hicpp-signed-bitwise | 50 | 12 | not run (skipped by the coordinator) |
| misc-no-recursion | 3 | 1 | not run (skipped by the coordinator) |
| misc-non-private-member-variables-in-classes | 212 | 37 | not run (skipped by the coordinator) |
| misc-unused-using-decls | 2 | 2 | not run (skipped by the coordinator) |
| performance-enum-size | 32 | 19 | not run (skipped by the coordinator) |
| performance-inefficient-string-concatenation | 13 | 8 | not run (skipped by the coordinator) |
| performance-inefficient-vector-operation | 25 | 15 | not run (skipped by the coordinator) |
| performance-unnecessary-copy-initialization | 2 | 2 | not run (skipped by the coordinator) |
| performance-unnecessary-value-param | 2 | 2 | not run (skipped by the coordinator) |
| readability-container-data-pointer | 2 | 2 | not run (skipped by the coordinator) |

## Enabled checks that found nothing

272 of 326 enabled checks (pass 1: 207, pass 2: 119; from `clang-tidy-18 --list-checks --checks=<set>`):

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
- clang-analyzer-fuchsia.HandleChecker
- clang-analyzer-nullability.NullPassedToNonnull
- clang-analyzer-nullability.NullReturnedFromNonnull
- clang-analyzer-nullability.NullabilityBase
- clang-analyzer-nullability.NullableDereferenced
- clang-analyzer-nullability.NullablePassedToNonnull
- clang-analyzer-nullability.NullableReturnedFromNonnull
- clang-analyzer-optin.core.EnumCastOutOfRange
- clang-analyzer-optin.cplusplus.UninitializedObject
- clang-analyzer-optin.cplusplus.VirtualCall
- clang-analyzer-optin.mpi.MPI-Checker
- clang-analyzer-optin.osx.OSObjectCStyleCast
- clang-analyzer-optin.osx.cocoa.localizability.EmptyLocalizationContextChecker
- clang-analyzer-optin.osx.cocoa.localizability.NonLocalizedStringChecker
- clang-analyzer-optin.performance.GCDAntipattern
- clang-analyzer-optin.portability.UnixAPI
- clang-analyzer-osx.API
- clang-analyzer-osx.MIG
- clang-analyzer-osx.NSOrCFErrorDerefChecker
- clang-analyzer-osx.NumberObjectConversion
- clang-analyzer-osx.OSObjectRetainCount
- clang-analyzer-osx.ObjCProperty
- clang-analyzer-osx.SecKeychainAPI
- clang-analyzer-osx.cocoa.AtSync
- clang-analyzer-osx.cocoa.AutoreleaseWrite
- clang-analyzer-osx.cocoa.ClassRelease
- clang-analyzer-osx.cocoa.Dealloc
- clang-analyzer-osx.cocoa.IncompatibleMethodTypes
- clang-analyzer-osx.cocoa.Loops
- clang-analyzer-osx.cocoa.MissingSuperCall
- clang-analyzer-osx.cocoa.NSAutoreleasePool
- clang-analyzer-osx.cocoa.NSError
- clang-analyzer-osx.cocoa.NilArg
- clang-analyzer-osx.cocoa.NonNilReturnValue
- clang-analyzer-osx.cocoa.ObjCGenerics
- clang-analyzer-osx.cocoa.RetainCount
- clang-analyzer-osx.cocoa.RetainCountBase
- clang-analyzer-osx.cocoa.RunLoopAutoreleaseLeak
- clang-analyzer-osx.cocoa.SelfInit
- clang-analyzer-osx.cocoa.SuperDealloc
- clang-analyzer-osx.cocoa.UnusedIvars
- clang-analyzer-osx.cocoa.VariadicMethodTypes
- clang-analyzer-osx.coreFoundation.CFError
- clang-analyzer-osx.coreFoundation.CFNumber
- clang-analyzer-osx.coreFoundation.CFRetainRelease
- clang-analyzer-osx.coreFoundation.containers.OutOfBounds
- clang-analyzer-osx.coreFoundation.containers.PointerSizedValues
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
- clang-analyzer-webkit.NoUncountedMemberChecker
- clang-analyzer-webkit.RefCntblBaseVirtualDtor
- clang-analyzer-webkit.UncountedLambdaCapturesChecker
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

Checks that fired but are not in the `--list-checks` output (diagnostic names): none

## Places of every check with 1 to 15 places

### bugprone-assignment-in-if-condition (1)
```
src/rrt.cc:66:20: an assignment within an 'if' condition is bug-prone [bugprone-assignment-in-if-condition]
```

### bugprone-branch-clone (3)
```
src/layout_plan.cc:972:38: conditional operator with identical true and false expressions [bugprone-branch-clone]
src/path_timing.cc:438:30: repeated branch body in conditional chain [bugprone-branch-clone]
src/plan_layout.cc:303:43: repeated branch body in conditional chain [bugprone-branch-clone]
```

### bugprone-integer-division (3)
```
src/cell.cc:105:58: result of integer division used in a floating point context; possible loss of precision [bugprone-integer-division]
tests/rectangle_fit_test.cc:500:50: result of integer division used in a floating point context; possible loss of precision [bugprone-integer-division]
tests/rectangle_fit_test.cc:501:69: result of integer division used in a floating point context; possible loss of precision [bugprone-integer-division]
```

### bugprone-macro-parentheses (2)
```
tests/plan_replay_test.cc:472:8: macro argument should be enclosed in parentheses [bugprone-macro-parentheses]
tests/plan_replay_test.cc:472:35: macro argument should be enclosed in parentheses [bugprone-macro-parentheses]
```

### bugprone-misplaced-widening-cast (2)
```
src/print_plan.cc:441:19: either cast from 'std::int64_t' (aka 'long') to 'long long' is ineffective, or there is loss of precision before the conversion [bugprone-misplaced-widening-cast]
tests/motion_planner_test.cc:163:17: either cast from 'std::int64_t' (aka 'long') to 'long long' is ineffective, or there is loss of precision before the conversion [bugprone-misplaced-widening-cast]
```

### bugprone-suspicious-memory-comparison (1)
```
tests/depth_camera_test.cc:61:10: comparing object representation of type 'float' which does not have a unique object representation; consider comparing the values manually [bugprone-suspicious-memory-comparison]
```

### cert-dcl50-cpp (6)
```
src/held_module_finder.cc:35:13: do not define a C-style variadic function; consider using a function parameter pack or currying instead [cert-dcl50-cpp]
src/layout_check.cc:51:13: do not define a C-style variadic function; consider using a function parameter pack or currying instead [cert-dcl50-cpp]
src/layout_plan.cc:38:13: do not define a C-style variadic function; consider using a function parameter pack or currying instead [cert-dcl50-cpp]
src/layout_replay.cc:194:13: do not define a C-style variadic function; consider using a function parameter pack or currying instead [cert-dcl50-cpp]
src/layout_rows.cc:39:13: do not define a C-style variadic function; consider using a function parameter pack or currying instead [cert-dcl50-cpp]
src/module_finder.cc:36:13: do not define a C-style variadic function; consider using a function parameter pack or currying instead [cert-dcl50-cpp]
```

### cert-exp42-c (1)
```
tests/depth_camera_test.cc:61:10: comparing object representation of type 'float' which does not have a unique object representation; consider comparing the values manually [cert-exp42-c]
```

### cert-flp30-c (11)
```
src/check_placement.cc:180:41: loop induction expression should not have floating-point type [cert-flp30-c]
src/check_placement.cc:746:48: loop induction expression should not have floating-point type [cert-flp30-c]
src/check_placement.cc:747:49: loop induction expression should not have floating-point type [cert-flp30-c]
src/check_placement.cc:774:46: loop induction expression should not have floating-point type [cert-flp30-c]
src/layout_plan.cc:825:37: loop induction expression should not have floating-point type [cert-flp30-c]
tests/conveyor_test.cc:467:58: loop induction expression should not have floating-point type [cert-flp30-c]
tests/ik_test.cc:273:42: loop induction expression should not have floating-point type [cert-flp30-c]
tests/layout_plan_test.cc:86:50: loop induction expression should not have floating-point type [cert-flp30-c]
tests/layout_plan_test.cc:87:54: loop induction expression should not have floating-point type [cert-flp30-c]
tests/suction_test.cc:302:55: loop induction expression should not have floating-point type [cert-flp30-c]
tests/suction_test.cc:303:55: loop induction expression should not have floating-point type [cert-flp30-c]
```

### cert-flp37-c (1)
```
tests/depth_camera_test.cc:61:10: comparing object representation of type 'float' which does not have a unique object representation; consider comparing the values manually [cert-flp37-c]
```

### clang-analyzer-core.CallAndMessage (2)
```
src/container.h:163:20: 2nd function call argument is an uninitialized value [clang-analyzer-core.CallAndMessage]
src/flat_rack.h:133:20: 2nd function call argument is an uninitialized value [clang-analyzer-core.CallAndMessage]
```

### clang-analyzer-cplusplus.StringChecker (3)
```
tests/arm_follower_test.cc:718:28: The parameter must not be null [clang-analyzer-cplusplus.StringChecker]
tests/layout_model_test.cc:1135:7: The parameter must not be null [clang-analyzer-cplusplus.StringChecker]
tests/motion_planner_test.cc:852:43: The parameter must not be null [clang-analyzer-cplusplus.StringChecker]
```

### clang-analyzer-deadcode.DeadStores (1)
```
src/run_cell.cc:179:15: Value stored to 'status' during its initialization is never read [clang-analyzer-deadcode.DeadStores]
```

### clang-analyzer-optin.performance.Padding (6)
```
src/layout_check.cc:150:8: Excessive padding in 'struct cell::layout::(anonymous namespace)::LegResult' (16 padding bytes, where 0 is optimal). Optimal fields order: length, figure, at_m, margin_deg, nearest, path, margin_joint, held, why, far, samples, walked, reached, by_design, at_far, consider reordering the fields or adding explicit padding members [clang-analyzer-optin.performance.Padding]
src/pick_place.h:645:8: Excessive padding in 'struct cell::CheckReport' (34 padding bytes, where 2 is optimal). Optimal fields order: module_in_tip, finder_seconds, render_seconds, tilt, gap, recheck_margin, recheck_self_margin, recheck_seconds, replan_seconds, offset_shift, offset_rotation, failure_detail, recheck_hit, replan, status, pictures, failure, recheck_samples, retaken, vacuum_holds, implausible, rechecked, recheck_clear, replanned, consider reordering the fields or adding explicit padding members [clang-analyzer-optin.performance.Padding]
src/pick_place.h:831:7: Excessive padding in 'class cell::PickPlace' (63 padding bytes, where 15 is optimal). Optimal fields order: grip_tip_, release_tip_, release_module_, held_in_tip_, prior_, estimate_, checker_, outcome_, look_ahead_, cell_, scratch_, last_time_, step_start_, plan_end_, look_hold_start_, place_x_sign_, carry_leg_, carry_dwell_end_, look_pose_, check_leg_, planned_, records_, look_, looks_, to_standoff_, approach_, touch_, lift_, pull_back_, clear_, flatten_, swing_, turn_, lower_, set_down_, retreat_, return_, known_, carry_legs_, carry_, checks_, placed_hook_, check_hook_, look_note_, grip_note_, carry_failure_, carry_note_, check_note_, follower_, options_, step_, pending_, held_in_tip_module_, pause_after_, paused_, look_moving_, look_moved_, set_down_blocked_, carry_dwelling_, consider reordering the fields or adding explicit padding members [clang-analyzer-optin.performance.Padding]
tests/camera_cycle_test.cc:225:8: Excessive padding in 'struct (anonymous namespace)::PhaseLog' (99 padding bytes, where 3 is optimal). Optimal fields order: grip_delta, outcome, second, gap, square, off_centre, approach_stack, module_1_distance, deck_distance, carry_tilt, carry_yaw, carry_off_slot, carry_height, saturated, closest_module_stack, closest_module_arm, closest_module_conveyor, tool_to_module, module_0_moved_y, module_0_y_placed, forbidden, place_x_sign, second_look_time, second_forbidden, first_cycle_time, second_cycle_time, peak_holding, error, look_result, first_refusal, touch_result, forbidden_pick, forbidden_carry, forbidden_place, phases, forbidden_list, report, second_phases, lift_off_stack, seen, second_seen, at_release, after_step, placed, second_placed, scene, seed, pictures, arm_pixels_at_look, outcome_module, arm_pixels_at_retreat, steps_after, held, view, look_ahead_view, look_done, first_refused, moved_to_look, approach_done, grip_done, lift_off_done, carry_done, release_done, still_held, cycle_done, look_ahead_found, second_done, second_moved_to_look, consider reordering the fields or adding explicit padding members [clang-analyzer-optin.performance.Padding]
tests/check_cycle_test.cc:430:8: Excessive padding in 'struct (anonymous namespace)::ConstructedRun' (50 padding bytes, where 2 is optimal). Optimal fields order: tip_at_check_pose, truth_in_tip, scene_grip_same, scene_grip_other, measured, module_at_release, release_module, release_tip, outcome, forbidden, check_pose, turn_end, arm_at_end, legs, checks, tip_from_record, arm_at_picture, leg1_end, error, phases, forbidden_list, placed, measured_module, slot, state_at_end, done, vacuum_at_end, consider reordering the fields or adding explicit padding members [clang-analyzer-optin.performance.Padding]
tests/pick_place_test.cc:173:8: Excessive padding in 'struct (anonymous namespace)::NominalLog' (76 padding bytes, where 12 is optimal). Optimal fields order: module_in_tip, grip_tip_0, grip_tip_1, outcome, second, gap, square, off_centre, approach_stack, grip_pose_shift, grip_pose_turn, module_1_distance, deck_distance, carry_tilt, carry_yaw, carry_off_slot, carry_height, saturated_by_carry, tool_to_module, cycle_time, arm_off_end, tool_after_retreat, slide_after, nominal_slide_after, module_0_y_placed, module_0_moved_y, forbidden, saturated, out_of_range, non_finite, place_x_sign, second_forbidden, peak_holding, waypoints, error, phases, forbidden_approach, forbidden_lift_off, forbidden_carry, forbidden_set_down, forbidden_retreat, forbidden_list, check_report, second_phases, lift_off_stack, placed, placed_stack, placed_slot, at_release, after_step, closest, steps_after, held, approach_done, grip_done, lift_off_done, carry_done, release_done, still_held, cycle_done, second_done, consider reordering the fields or adding explicit padding members [clang-analyzer-optin.performance.Padding]
```

### clang-analyzer-security.FloatLoopCounter (9)
```
src/check_placement.cc:180:3: Variable 'z' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
src/check_placement.cc:746:5: Variable 'a' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
src/check_placement.cc:747:7: Variable 'r' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
src/check_placement.cc:774:3: Variable 'a' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
src/layout_plan.cc:825:3: Variable 't' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
tests/conveyor_test.cc:467:5: Variable 't' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
tests/ik_test.cc:273:5: Variable 'q3' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
tests/suction_test.cc:302:5: Variable 'a' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
tests/suction_test.cc:303:7: Variable 'b' with floating point type 'double' should not be used as a loop counter [clang-analyzer-security.FloatLoopCounter]
```

### cppcoreguidelines-avoid-do-while (2)
```
src/pick_place.cc:1524:3: avoid do-while loops [cppcoreguidelines-avoid-do-while]
src/portable_random.h:86:7: avoid do-while loops [cppcoreguidelines-avoid-do-while]
```

### cppcoreguidelines-avoid-goto (3)
```
tests/bay_model_test.cc:213:3: avoid using 'goto' for flow control [cppcoreguidelines-avoid-goto]
tests/bay_model_test.cc:216:3: avoid using 'goto' for flow control [cppcoreguidelines-avoid-goto]
tests/ompl_search_test.cc:500:3: avoid using 'goto' for flow control [cppcoreguidelines-avoid-goto]
```

### cppcoreguidelines-avoid-non-const-global-variables (8)
```
src/bay_model.cc:241:13: variable 'kBodyClose' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
src/layout_rows.cc:183:13: variable 'kFaces' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
src/rectangle_fit.cc:412:13: variable 'kSideNames' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
tests/arm_follower_test.cc:508:1: variable 'gtest_PayloadLimitsTest_dummy_' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
tests/gate_test.cc:47:13: variable 'save_failures' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
tests/joint_move_test.cc:225:1: variable 'gtest_ShapesJointMoveTest_dummy_' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
tests/layout_plan_test.cc:23:13: variable 'kName' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
tests/path_follow_test.cc:383:1: variable 'gtest_PayloadPayloadTest_dummy_' is non-const and globally accessible, consider making it const [cppcoreguidelines-avoid-non-const-global-variables]
```

### cppcoreguidelines-init-variables (12)
```
src/layout_check.cc:359:10: variable 'perr' is not initialized [cppcoreguidelines-init-variables]
src/layout_check.cc:359:16: variable 'aerr' is not initialized [cppcoreguidelines-init-variables]
src/layout_check.cc:359:22: variable 'perrn' is not initialized [cppcoreguidelines-init-variables]
src/layout_check.cc:359:29: variable 'aerrn' is not initialized [cppcoreguidelines-init-variables]
src/layout_check.cc:705:14: variable 'margin' is not initialized [cppcoreguidelines-init-variables]
src/layout_check.cc:865:12: variable 'margin' is not initialized [cppcoreguidelines-init-variables]
src/layout_check.cc:2106:16: variable 'margin' is not initialized [cppcoreguidelines-init-variables]
src/layout_check.cc:2158:14: variable 'margin' is not initialized [cppcoreguidelines-init-variables]
src/portable_random.h:80:12: variable 'standard' is not initialized [cppcoreguidelines-init-variables]
src/portable_random.h:85:14: variable 'x' is not initialized [cppcoreguidelines-init-variables]
src/portable_random.h:85:17: variable 'y' is not initialized [cppcoreguidelines-init-variables]
src/portable_random.h:85:20: variable 'r2' is not initialized [cppcoreguidelines-init-variables]
```

### cppcoreguidelines-macro-usage (2)
```
src/layout.cc:131:9: function-like macro 'FIELD' used; consider a 'constexpr' template function [cppcoreguidelines-macro-usage]
tests/plan_replay_test.cc:470:9: function-like macro 'NEED_COMMITTED' used; consider a 'constexpr' template function [cppcoreguidelines-macro-usage]
```

### cppcoreguidelines-prefer-member-initializer (1)
```
src/cycle_check.cc:83:3: 'tool_' should be initialized in a member initializer of the constructor [cppcoreguidelines-prefer-member-initializer]
```

### cppcoreguidelines-pro-type-const-cast (6)
```
src/cycle_check.cc:307:28: do not use const_cast to remove const qualifier [cppcoreguidelines-pro-type-const-cast]
src/view_layout.cc:348:20: do not use const_cast to remove const qualifier [cppcoreguidelines-pro-type-const-cast]
src/view_layout.cc:349:20: do not use const_cast to remove const qualifier [cppcoreguidelines-pro-type-const-cast]
tests/camera_cycle_test.cc:150:45: do not use const_cast to remove const qualifier [cppcoreguidelines-pro-type-const-cast]
tests/motion_planner_test.cc:597:20: do not use const_cast to remove const qualifier [cppcoreguidelines-pro-type-const-cast]
tests/pick_place_test.cc:151:39: do not use const_cast to remove const qualifier [cppcoreguidelines-pro-type-const-cast]
```

### cppcoreguidelines-pro-type-reinterpret-cast (1)
```
src/layout_replay.cc:514:24: do not use reinterpret_cast [cppcoreguidelines-pro-type-reinterpret-cast]
```

### cppcoreguidelines-special-member-functions (5)
```
src/layout_model.cc:684:10: class 'VfsGuard' defines a non-default destructor but does not define a copy constructor, a copy assignment operator, a move constructor or a move assignment operator [cppcoreguidelines-special-member-functions]
src/layout_plan.cc:579:10: class 'Report' defines a non-default destructor but does not define a copy constructor, a copy assignment operator, a move constructor or a move assignment operator [cppcoreguidelines-special-member-functions]
src/layout_scene.h:242:7: class 'LayoutScene' defines a copy constructor and a copy assignment operator but does not define a destructor, a move constructor or a move assignment operator [cppcoreguidelines-special-member-functions]
src/motion_planner.cc:60:7: class 'MarginGuard' defines a non-default destructor, a copy constructor and a copy assignment operator but does not define a move constructor or a move assignment operator [cppcoreguidelines-special-member-functions]
src/ompl_search.h:228:7: class 'OmplSpace' defines a destructor, a copy constructor and a copy assignment operator but does not define a move constructor or a move assignment operator [cppcoreguidelines-special-member-functions]
```

### misc-no-recursion (3)
```
src/pick_place.cc:1387:17: function 'Finish' is within a recursive call chain [misc-no-recursion]
src/pick_place.cc:1555:17: function 'UseLook' is within a recursive call chain [misc-no-recursion]
src/pick_place.cc:1627:17: function 'BeginStep' is within a recursive call chain [misc-no-recursion]
```

### misc-unused-using-decls (2)
```
tests/path_follow_test.cc:40:13: using decl 'JointLimits' is unused [misc-unused-using-decls]
tests/rectangle_fit_test.cc:29:13: using decl 'Canonical' is unused [misc-unused-using-decls]
```

### performance-inefficient-string-concatenation (13)
```
src/check_layout.cc:93:44: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
src/layout.cc:57:57: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
src/layout_model.cc:217:66: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
src/layout_replay.cc:1211:60: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
src/layout_rows.cc:702:47: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
src/print_plan.cc:434:9: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
tests/gate_test.cc:153:7: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
tests/locate_physics_test.cc:227:31: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
tests/locate_physics_test.cc:230:31: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
tests/locate_physics_test.cc:233:31: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
tests/locate_physics_test.cc:235:30: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
tests/locate_physics_test.cc:237:30: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
tests/locate_physics_test.cc:246:57: string concatenation results in allocation of unnecessary temporary strings; consider using 'operator+=' or 'string::append()' instead [performance-inefficient-string-concatenation]
```

### performance-unnecessary-copy-initialization (2)
```
tests/check_cycle_test.cc:395:23: local copy 'leg1_options' of the variable 'options' is never modified; consider avoiding the copy [performance-unnecessary-copy-initialization]
tests/layout_model_test.cc:1138:16: the const qualified variable 'layout' is copy-constructed from a const reference; consider making it a const reference [performance-unnecessary-copy-initialization]
```

### performance-unnecessary-value-param (2)
```
src/print_cell.cc:43:32: the parameter 'name' is copied for each invocation but only used as a const reference; consider making it a const reference [performance-unnecessary-value-param]
src/rrt_worlds.cc:28:66: the parameter 'free_by' is copied for each invocation but only used as a const reference; consider making it a const reference [performance-unnecessary-value-param]
```

### readability-container-data-pointer (2)
```
src/view_cell.cc:149:49: 'data' should be used for accessing the data pointer instead of taking the address of the 0-th element [readability-container-data-pointer]
src/viewer_lib.cc:242:43: 'data' should be used for accessing the data pointer instead of taking the address of the 0-th element [readability-container-data-pointer]
```

