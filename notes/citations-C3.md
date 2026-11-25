# Citation check C3: S41 to S59

All 19 sources opened (no UNREACHABLE). GitHub files via raw.githubusercontent.com at the named full commits
(Drake 5186757f28d5..., MuJoCo 72a06685bffb..., MoveIt 8c99e523bc23...); directory statements via
api.github.com contents API at the same commits. Web pages fetched with curl (googlesource page is base64, decoded;
first attempt returned a 503, second worked).

## S41 Chromium clang_tidy.md: CORRECTED
Fetched the googlesource URL (format=TEXT, decoded, 281 lines). 3 claims, 1 quotation (OK, lines 123-124).
Fail: entry says "30 or more reviewed findings"; source says "a random sample (>30) of places where the check fires".
Also the sample is surveyed manually; "count" is right. Fix in JSON (line 182).

## S42 gRPC .clang-tidy: CORRECTED
Fetched raw master file (170 lines). 2 claims, 1 quotation ("not activated yet": the source has "it's not activated yet" - OK).
Fail: "Each disabled check has a reason" is false for the Checks list: about a dozen disabled checks have no comment
(bugprone-casting-through-void, crtp-constructor-accessibility, empty-catch, inc-dec-in-conditions,
multi-level-implicit-pointer-conversion, return-const-ref-from-parameter, suspicious-stringview-data-usage,
unused-local-non-trivial-variable, performance-enum-size, inefficient-vector-operation,
clang-diagnostic-thread-safety-reference-return, readability-math-missing-parentheses). The "future" list (40 checks) has
one shared reason, "due to the existing issues with the checks", and is not only numeric (modernize-*, readability-*,
performance-*, google-*). Citing lines 409, 484 and 497 corrected. (Master may have moved since; the file read is today's.)

## S43 SWE at Google ch. 21: OK
Fetched abseil.io page. 2 claims, 1 quotation present word for word. Diamonds, SemVer, pinning all present;
"C++ has nearly zero tolerance for diamond dependencies". Primary OK.

## S44 Bazel lockfile: OK
Fetched bazel.build/external/lockfile. 2 claims. `--lockfile_mode`'s `error` mode: "if any information consulted during
resolution is missing or out-of-date, Bazel will fail with an error"; .bazelversion advice present. No quotations. OK.

## S45 IWYU WhyIWYU.md: CORRECTED
Fetched raw master (120 lines). Claim 1 true ("Allow refactoring" section). Claim 2 "the tool is built on Clang" is not in the
file (0 mentions of Clang). Remove it (line 199).

## S46 REUSE 3.3: OK
Fetched reuse.software/spec-3.3/. 2 claims, 1 quotation: "Each License File MUST be placed in the LICENSES/ directory in the
root of the Project." present. OK.

## S47 Drake drake_cc.bzl: OK as a source, one citing sentence corrected
Raw at 5186757f (936 lines). Claims 2/2 true: CXX_FLAGS "-Werror=..." for all rules; GCC opt-outs each commented ("This falsely
dings code inside Eigen." present, with -Wno-maybe-uninitialized); drake_cc_googletest sets size="small" and
srcs "test/%s.cc"; internal=True fails if visibility empty or public. Shadow (CXX_FLAGS) and logical-op (GCC_FLAGS) are errors.
1 quotation OK. Citing sentences checked: lines 291, 342, 461-462, 541-543, 546, 551, 555, 616 all OK.
Fail: line 410 "Drake attaches lint tests to every BUILD file [S47]": drake_cc.bzl has no add_lint_tests; the only evidence read is
math/BUILD.bazel line 610 `add_lint_tests()` (S49). Reworded.

## S48 Drake directory_structure.md: OK (one list qualified)
Raw (58 lines). Claims true (systems uses solvers, so solvers must not depend on systems; experimental/dev may be depended on only by
such code). Line 537 lists eight areas as "top-level areas"; they exist at the commit but the root also has bindings, lcm,
manipulation, perception, visualization, tutorials: added "among them". doc/_pages exists (45 files); no decision-record directory in doc/.

## S49 Drake math/BUILD.bazel: OK
Raw (610 lines). 22 drake_cc_library, 33 drake_cc_googletest, 1 package library: counts as stated. Quotation "Move this code into
//math to avoid the circular dependency." present (split by a comment line break, lines 101-102 and 114-115). Also add_lint_tests() on line 610.

## S50 Drake .clang-tidy: OK
Raw (26 lines). First line quotation exact. Check groups: clang-analyzer, clang-diagnostic, cppcoreguidelines, google, modernize,
performance, readability.

## S51 Drake bazel-developer.rc: OK as a source, one citing sentence corrected
Raw (174 lines). Quotation present (the "*own*" wording, split by a line break). Sanitizers asan, lsan, tsan, ubsan, memcheck with scaled
--test_timeout. Fail: line 499 cites S51 for sanitizers "work with GCC"; S51 labels ASan and LSan "Clang only". Moved S51 to the named-config part.

## S52 Drake developers.md + code_review_checklist.md: OK
Raw both. Two phases (feature, platform); nine owners listed; 750/1500-line limits; "Do not use Eigen::Random, libc rand ... This includes
test code" and a hard-coded seed. Design feedback via issue (developers.md line 88). 

## S53 MuJoCo MujocoOptions.cmake: OK
Raw (133 lines). Flags -Werror -Wall -Wpedantic -Wimplicit-fallthrough -Wunused -Wvla; three -Wno (int-in-bool-context, sign-compare,
unknown-pragmas); GNU adds -Wno-maybe-uninitialized (and -Wimplicit-fallthrough=5, -Wno-psabi); visibility presets hidden. No quotations.

## S54 MuJoCo src/engine/BUILD.bazel: CORRECTED
Raw (1406 lines). package_group _impl_visibility has 5 trees (batch, render/classic, user, xml, test); default_visibility is it; "engine" is the
public aggregate (_api_visibility). -Wshadow and -ffp-contract=off present. Fail: "one cc_library per C file" is false: 41 cc_library rules for 40 .c files, with
libraries holding 2-3 files (engine_forward = forward+inverse+ipc, engine_derivative = derivative+metric+passive, engine_core,
engine_vis, engine_collision_convex, engine_collision_raysdf). Fixed at lines 234 and 564. Directory: src/engine has no subdirectories (flat).

## S55 MuJoCo fixture.h + engine_forward_test.cc: OK
Raw. Quotation "Setting MJTOL_SCALE=0 scales tolerances to zero" present in fixture.h lines 53-54; MjTol(double_tol, float_tol) present;
tests use XML models from test/engine/testdata; no stored expected output files seen.

## S56 MuJoCo STYLEGUIDE.md: OK
Raw (189 lines). "Maximise consistency with the rest of the code." present; "the guide takes precedence"; guide to be expanded.

## S57 MoveIt AGENTS.md: CORRECTED (one citing sentence)
Raw (97 lines). Quotations "The layers depend in one direction, from bottom to top" and "Test the contract, not the implementation." present.
CI uses -Werror and clang-tidy on changed code only. Fail: line 513 says MoveIt "keeps planner and collision implementations out of its core";
AGENTS.md puts "collision detection (FCL, Bullet, distance field)" in moveit_core and the tree has moveit_core/collision_detection_fcl and
_bullet. Planner and kinematics are the separate plugin packages. Replaced "collision" with "kinematics".

## S58 MoveIt .clang-tidy: OK as a source, citing sentence corrected
Raw (65 lines). Starts from '-*', performance-*, some modernize/readability/misc, no bugprone, no WarningsAsErrors. Fail: line 621
"Wholesale modernize-* and readability-* groups [S50][S58]": MoveIt lists single checks, not groups; S58 removed there.

## S59 MoveIt moveit_package.cmake: OK
Raw (101 lines). -Wall -Wextra + seven (write-strings, unreachable-code, pointer-arith, redundant-decls, cast-qual, old-style-cast, format=2); no -Werror;
GNU `-Wno-maybe-uninitialized` with the comment "This too often has false-positives" exact.

## Tree statements (lines 531-608)
Checked against the contents API: Drake areas exist; MuJoCo include/ separate, src/engine flat, test/ mirrors (engine, user, xml, render, batch);
no ADR directory in any of the three roots or docs; MoveIt migration entry rule in AGENTS.md. 13 corrections in the JSON.
