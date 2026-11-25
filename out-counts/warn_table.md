| flag | in -Wall/-Wextra already? (GCC manual, not measured) | GCC 14.2 inside | GCC 14.2 outside | GCC 13.3 inside | GCC 13.3 outside | files (GCC 14.2) | unknown to which compiler |
|---|---|---|---|---|---|---|---|
| -Wextra | (is the flag) | 11 | 0 | 11 | 0 | 2 | none |
| -Wpedantic | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wshadow | no | 8 | 0 | 8 | 0 | 8 | none |
| -Wconversion | no | 1314 | 111 | 1314 | 111 | 74 | none |
| -Wsign-conversion | no | 1253 | 109 | 1253 | 109 | 68 | none |
| -Wfloat-conversion | no | 0 | 1 | 0 | 1 | 0 | none |
| -Warith-conversion | no | 0 | 10 | 0 | 10 | 0 | none |
| -Wfloat-equal | no | 53 | 8 | 53 | 8 | 27 | none |
| -Wdouble-promotion | no | 27 | 1 | 27 | 1 | 6 | none |
| -Wnull-dereference | no | 2 | 10 | 2 | 8 | 2 | none | <-- differs
| -Wswitch-enum | no | 4 | 0 | 4 | 0 | 3 | none |
| -Wswitch-default | no | 22 | 0 | 22 | 0 | 13 | none |
| -Wformat=2 | Wall (lower level) | 48 | 0 | 48 | 0 | 13 | none |
| -Wformat-signedness | Wall (lower level) | 0 | 0 | 0 | 0 | 0 | none |
| -Wformat-overflow=2 | Wall (lower level) | 0 | 0 | 0 | 0 | 0 | none |
| -Wformat-truncation=2 | Wall (lower level) | 68 | 0 | 68 | 0 | 24 | none |
| -Wuseless-cast | no | 4 | 45 | 4 | 45 | 2 | none |
| -Wduplicated-cond | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wduplicated-branches | no | 0 | 5 | 0 | 4 | 0 | none | <-- differs
| -Wlogical-op | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wimplicit-fallthrough=5 | Wextra (level 3; asked =5) | 0 | 0 | 0 | 0 | 0 | none |
| -Wvla | no | 0 | 0 | 0 | 0 | 0 | none |
| -Walloca | no | 0 | 1 | 0 | 1 | 0 | none |
| -Wnon-virtual-dtor | no | 0 | 0 | 0 | 0 | 0 | none |
| -Woverloaded-virtual | Wall (lower level) | 0 | 3 | 0 | 3 | 0 | none |
| -Wcast-align | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wcast-qual | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wold-style-cast | no | 0 | 113 | 0 | 113 | 0 | none |
| -Wundef | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused | Wall -Wunused-function -Wunused-variable ... | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-parameter | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wmissing-declarations | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wredundant-decls | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wpointer-arith | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wwrite-strings | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wdate-time | no | 0 | 0 | 0 | 0 | 0 | none |
| -Warray-bounds=2 | Wall (lower level) | 0 | 0 | 0 | 0 | 0 | none |
| -Wstringop-overflow=4 | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wshift-overflow=2 | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wstrict-overflow=2 | Wall (lower level) | 0 | 0 | 0 | 0 | 0 | none |
| -Wdeprecated-copy-dtor | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wextra-semi | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wzero-as-null-pointer-constant | no | 0 | 27 | 0 | 27 | 0 | none |
| -Wsuggest-override | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wctor-dtor-privacy | no | 0 | 5 | 0 | 5 | 0 | none |
| -Wmismatched-tags | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wredundant-tags | no | 0 | 5 | 0 | 5 | 0 | none |
| -Wcomma-subscript | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wvolatile | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wdeprecated-enum-enum-conversion | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wdeprecated-enum-float-conversion | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wenum-conversion | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wint-in-bool-context | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wunsafe-loop-optimizations | no (option is ignored by GCC 14) | 0 | 0 | 0 | 0 | 0 | none |
| -Wdangling-reference | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wdangling-pointer=2 | Wall (lower level) | 0 | 0 | 0 | 0 | 0 | none |
| -Wuse-after-free=3 | Wall (lower level) | 0 | 0 | 0 | 0 | 0 | none |
| -Wself-move | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wcatch-value=3 | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wplacement-new=2 | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wtrampolines | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wmissing-include-dirs | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-macros | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-const-variable=2 | no | 0 | 6 | 0 | 6 | 0 | none |
| -Winvalid-pch | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wcast-function-type | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wbidi-chars=any | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Winvalid-utf8 | no | 0 | 0 | 0 | 0 | 0 | none |
| -Wxor-used-as-pow | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wreturn-type | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wuninitialized | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wmaybe-uninitialized | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wparentheses | Wall | 4 | 0 | 4 | 0 | 3 | none |
| -Wsequence-point | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wmisleading-indentation | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wdangling-else | Wall | 4 | 0 | 4 | 0 | 3 | none |
| -Wcomment | Wall | 3 | 0 | 3 | 0 | 2 | none |
| -Wsign-compare | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wreorder | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wdelete-non-virtual-dtor | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wnarrowing | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Waddress | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wbool-compare | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wtautological-compare | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wsizeof-pointer-memaccess | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wmemset-transposed-args | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wswitch | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-value | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-variable | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-but-set-variable | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-function | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wunused-result | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wempty-body | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wignored-qualifiers | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wtype-limits | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wmissing-field-initializers | Wextra | 11 | 0 | 11 | 0 | 2 | none |
| -Wimplicit-fallthrough | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wpessimizing-move | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wredundant-move | Wextra | 0 | 0 | 0 | 0 | 0 | none |
| -Wrange-loop-construct | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wclass-memaccess | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wreturn-local-addr | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wnonnull | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wformat | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wformat-security | no (on with -Wformat=2) | 0 | 0 | 0 | 0 | 0 | none |
| -Wstrict-aliasing | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Warray-bounds | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wstringop-overflow | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wstringop-truncation | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wint-to-pointer-cast | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wshift-count-overflow | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wshift-count-negative | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wdiv-by-zero | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Woverflow | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wmultichar | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wdeprecated | default on | 0 | 46 | 0 | 46 | 0 | none |
| -Wdeprecated-declarations | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Winfinite-recursion | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wmismatched-new-delete | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wfree-nonheap-object | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Wuse-after-free | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wdangling-pointer | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wvexing-parse | default on | 0 | 0 | 0 | 0 | 0 | none |
| -Winit-self | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wlogical-not-parentheses | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wsizeof-array-div | Wall | 0 | 0 | 0 | 0 | 0 | none |
| -Wc++20-compat | Wall | 0 | 0 | 0 | 0 | 0 | none |

| diagnostic name printed by GCC | asked flags that stand for it (D line) | GCC 14.2 inside | outside | GCC 13.3 inside | outside |
|---|---|---|---|---|---|
| -Walloca | -Walloca | 0 | 1 | 0 | 1 |
| -Warith-conversion | -Warith-conversion | 0 | 10 | 0 | 10 |
| -Wcomment | -Wcomment | 3 | 0 | 3 | 0 |
| -Wconversion | -Wconversion | 61 | 1 | 61 | 1 |
| -Wctor-dtor-privacy | -Wctor-dtor-privacy | 0 | 5 | 0 | 5 |
| -Wdangling-else | -Wdangling-else,-Wparentheses | 4 | 0 | 4 | 0 |
| -Wdeprecated | -Wdeprecated | 0 | 46 | 0 | 46 |
| -Wdouble-promotion | -Wdouble-promotion | 27 | 1 | 27 | 1 |
| -Wduplicated-branches | -Wduplicated-branches | 0 | 5 | 0 | 4 | <-- differs
| -Wfloat-conversion | -Wconversion,-Wfloat-conversion | 0 | 1 | 0 | 1 |
| -Wfloat-equal | -Wfloat-equal | 53 | 8 | 53 | 8 |
| -Wformat-nonliteral | -Wformat=2 | 48 | 0 | 48 | 0 |
| -Wformat-truncation | -Wformat-truncation=2 | 68 | 0 | 68 | 0 |
| -Wmissing-field-initializers | -Wmissing-field-initializers | 11 | 0 | 11 | 0 |
| -Wnull-dereference | -Wnull-dereference | 2 | 10 | 2 | 8 | <-- differs
| -Wold-style-cast | -Wold-style-cast | 0 | 113 | 0 | 113 |
| -Woverloaded-virtual | -Woverloaded-virtual | 0 | 3 | 0 | 3 |
| -Wredundant-tags | -Wredundant-tags | 0 | 5 | 0 | 5 |
| -Wshadow | -Wshadow | 8 | 0 | 8 | 0 |
| -Wsign-conversion | -Wconversion,-Wsign-conversion | 1253 | 109 | 1253 | 109 |
| -Wswitch-default | -Wswitch-default | 22 | 0 | 22 | 0 |
| -Wswitch-enum | -Wswitch-enum | 4 | 0 | 4 | 0 |
| -Wunused-const-variable | -Wunused-const-variable=2 | 0 | 6 | 0 | 6 |
| -Wuseless-cast | -Wuseless-cast | 4 | 45 | 4 | 45 |
| -Wzero-as-null-pointer-constant | -Wzero-as-null-pointer-constant | 0 | 27 | 0 | 27 |

### -Wextra (11 places, GCC 14.2)
```
src/layout_check.cc:1695:58: missing initializer for member 'cell::layout::PostureChanges::from' [-Wmissing-field-initializers]
tests/layout_model_test.cc:495:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:499:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:500:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:502:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:503:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:504:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:505:55: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:506:57: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:507:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:509:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
```

### -Wshadow (8 places, GCC 14.2)
```
src/path_timing.cc:732:57: declaration of 'const cell::Limits& limits' shadows a parameter [-Wshadow]
src/view_cell.cc:352:31: declaration of 'error' shadows a previous local [-Wshadow]
tests/arm_follower_test.cc:509:67: declaration of 'const testing::TestParamInfo<bool>& info' shadows a parameter [-Wshadow]
tests/joint_move_test.cc:228:54: declaration of 'const testing::TestParamInfo<cell::ProfileShape>& info' shadows a parameter [-Wshadow]
tests/motion_planner_test.cc:927:27: declaration of 'error' shadows a previous local [-Wshadow]
tests/path_follow_test.cc:384:67: declaration of 'const testing::TestParamInfo<bool>& info' shadows a parameter [-Wshadow]
tests/path_timing_test.cc:564:37: declaration of 'limits' shadows a previous local [-Wshadow]
tests/plan_replay_test.cc:413:11: declaration of 'c' shadows a previous local [-Wshadow]
```

### -Wfloat-equal (53 places, GCC 14.2)
```
src/arm.cc:48:17: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/arm_follower.cc:182:41: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/camera_model.cc:58:17: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/camera_model.cc:58:38: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/cell.cc:48:13: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/check_placement.cc:234:13: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/check_placement.cc:234:25: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/collision.cc:493:30: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/joint_move.cc:22:18: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_check.cc:1310:69: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_replay.cc:155:16: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_replay.cc:1711:20: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_replay.cc:1712:13: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_replay.cc:1725:32: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_rows.cc:160:57: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_rows.cc:546:43: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/layout_scene.cc:565:15: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/ompl_search.cc:155:21: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/path_timing.cc:157:23: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/path_timing.cc:170:14: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/path_timing.cc:194:14: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/path_timing.cc:203:12: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/path_timing.cc:665:14: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/path_timing.cc:734:12: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/portable_random.h:90:31: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_bay.cc:215:35: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_bay.cc:216:35: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_bay.cc:240:25: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_check.cc:418:22: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_check.cc:418:48: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_perception.cc:376:22: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_perception.cc:376:48: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/print_plan.cc:404:47: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/profile.cc:57:9: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/profile.cc:87:9: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/rectangle_fit.cc:461:39: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/rectangle_fit.cc:559:39: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/rectangle_fit.cc:704:36: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/rectangle_fit.cc:837:15: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/rectangle_fit.cc:1176:22: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/rrt.cc:187:18: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/trial_runner.cc:403:24: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/trial_runner.cc:403:52: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/trial_runner.cc:410:30: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/trial_runner.cc:410:64: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
src/trial_runner.cc:655:13: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
tests/arm_follower_test.cc:893:16: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
tests/bay_hardware_test.cc:133:21: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
tests/bay_hardware_test.cc:654:47: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
tests/bay_hardware_test.cc:655:47: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
tests/collision_test.cc:764:38: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
tests/held_module_finder_test.cc:650:73: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
tests/motion_planner_test.cc:699:53: comparing floating-point with '==' or '!=' is unsafe [-Wfloat-equal]
```

### -Wdouble-promotion (27 places, GCC 14.2)
```
src/depth_camera.cc:86:39: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/depth_camera.cc:98:48: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/depth_camera.cc:98:66: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/depth_camera.cc:184:58: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/depth_camera.cc:184:62: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/depth_camera.cc:185:32: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/depth_camera.cc:192:43: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/layout_check.cc:1310:69: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/layout_rows.cc:160:57: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/print_check.cc:133:54: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/print_depth.cc:78:31: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/print_depth.cc:78:47: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/print_depth.cc:83:56: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
src/print_depth.cc:197:52: implicit conversion from 'float' to 'double' when passing argument to function [-Wdouble-promotion]
src/print_depth.cc:197:61: implicit conversion from 'float' to 'double' when passing argument to function [-Wdouble-promotion]
src/print_depth.cc:221:45: implicit conversion from 'const float' to 'double' when passing argument to function [-Wdouble-promotion]
src/print_depth.cc:221:65: implicit conversion from 'const float' to 'double' when passing argument to function [-Wdouble-promotion]
tests/depth_camera_test.cc:179:51: implicit conversion from '__gnu_cxx::__alloc_traits<std::allocator<float>, float>::value_type' {aka 'float'} to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:512:60: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:542:27: implicit conversion from '__gnu_cxx::__alloc_traits<std::allocator<float>, float>::value_type' {aka 'float'} to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:542:49: implicit conversion from '__gnu_cxx::__alloc_traits<std::allocator<float>, float>::value_type' {aka 'float'} to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:574:44: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:575:62: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:576:56: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:576:64: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:676:42: implicit conversion from 'float' to 'double' to match other operand of binary expression [-Wdouble-promotion]
tests/depth_camera_test.cc:718:49: implicit conversion from '__gnu_cxx::__alloc_traits<std::allocator<float>, float>::value_type' {aka 'float'} to 'double' to match other operand of binary expression [-Wdouble-promotion]
```

### -Wnull-dereference (2 places, GCC 14.2)
```
src/arm_catalog.h:116:10: potential null pointer dereference [-Wnull-dereference]
tests/ik_test.cc:414:8: potential null pointer dereference [-Wnull-dereference]
```

### -Wswitch-enum (4 places, GCC 14.2)
```
src/cycle_check.cc:136:12: enumeration value 'kHeld' not handled in switch [-Wswitch-enum]
src/ompl_search.cc:427:12: enumeration value 'kNotStarted' not handled in switch [-Wswitch-enum]
src/pick_place.cc:1327:10: enumeration value 'kIdle' not handled in switch [-Wswitch-enum]
src/pick_place.cc:1507:10: enumeration value 'kIdle' not handled in switch [-Wswitch-enum]
```

### -Wswitch-default (22 places, GCC 14.2)
```
src/collision.cc:346:12: switch missing default case [-Wswitch-default]
src/cycle_check.cc:51:10: switch missing default case [-Wswitch-default]
src/cycle_check.cc:241:12: switch missing default case [-Wswitch-default]
src/held_module_finder.cc:99:10: switch missing default case [-Wswitch-default]
src/module_finder.cc:802:10: switch missing default case [-Wswitch-default]
src/motion_planner.cc:118:10: switch missing default case [-Wswitch-default]
src/motion_planner.cc:134:10: switch missing default case [-Wswitch-default]
src/ompl_search.cc:299:10: switch missing default case [-Wswitch-default]
src/pick_place.cc:149:10: switch missing default case [-Wswitch-default]
src/pick_place.cc:175:10: switch missing default case [-Wswitch-default]
src/pick_place.cc:227:10: switch missing default case [-Wswitch-default]
src/pick_place.cc:237:10: switch missing default case [-Wswitch-default]
src/pick_place.cc:275:12: switch missing default case [-Wswitch-default]
src/pick_place.cc:1187:14: switch missing default case [-Wswitch-default]
src/pick_place.cc:1629:10: switch missing default case [-Wswitch-default]
src/pick_place.cc:1804:10: switch missing default case [-Wswitch-default]
src/rectangle_fit.cc:793:10: switch missing default case [-Wswitch-default]
src/rrt.cc:349:10: switch missing default case [-Wswitch-default]
src/run_cell.cc:180:10: switch missing default case [-Wswitch-default]
src/suction.cc:174:12: switch missing default case [-Wswitch-default]
tests/ik_test.cc:202:10: switch missing default case [-Wswitch-default]
tests/path_follow_test.cc:509:16: switch missing default case [-Wswitch-default]
```

### -Wformat=2 (48 places, GCC 14.2)
```
src/check_placement.cc:158:18: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_check.cc:149:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_check.cc:185:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_check.cc:189:19: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_check.cc:194:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_check.cc:358:16: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_depth.cc:100:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_depth.cc:127:19: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_perception.cc:130:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_perception.cc:164:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_perception.cc:168:19: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_perception.cc:173:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_perception.cc:299:18: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_plan.cc:439:18: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_plane_fit.cc:88:16: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/print_profile.cc:82:16: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/rectangle_fit.cc:428:16: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_cell.cc:426:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_cell.cc:456:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_cell.cc:466:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_cell.cc:473:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_cell.cc:482:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_cell.cc:506:19: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_cell.cc:549:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_gate.cc:55:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_gate.cc:76:19: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/run_gate.cc:81:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/suction.cc:40:16: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/trial_runner.cc:60:16: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:101:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:140:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:150:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:156:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:162:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:168:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:174:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:187:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:197:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:208:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:214:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:221:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:227:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:233:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:250:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:258:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:272:21: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:276:19: format not a string literal, argument types not checked [-Wformat-nonliteral]
src/view_cell.cc:295:17: format not a string literal, argument types not checked [-Wformat-nonliteral]
```

### -Wuseless-cast (4 places, GCC 14.2)
```
src/depth_camera.cc:207:44: useless cast to type 'Eigen::Vector3d' {aka 'class Eigen::Matrix<double, 3, 1>'} [-Wuseless-cast]
src/portable_random.h:57:27: useless cast to type 'uint64_t' {aka 'long unsigned int'} [-Wuseless-cast]
src/portable_random.h:60:37: useless cast to type 'uint32_t' {aka 'unsigned int'} [-Wuseless-cast]
src/portable_random.h:62:17: useless cast to type 'uint64_t' {aka 'long unsigned int'} [-Wuseless-cast]
```

### -Wparentheses (4 places, GCC 14.2)
```
tests/ik_test.cc:971:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
tests/layout_check_test.cc:242:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
tests/layout_check_test.cc:879:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
tests/planned_carry_test.cc:997:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
```

### -Wdangling-else (4 places, GCC 14.2)
```
tests/ik_test.cc:971:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
tests/layout_check_test.cc:242:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
tests/layout_check_test.cc:879:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
tests/planned_carry_test.cc:997:8: suggest explicit braces to avoid ambiguous 'else' [-Wdangling-else]
```

### -Wcomment (3 places, GCC 14.2)
```
src/view_layout.cc:9:1: multi-line comment [-Wcomment]
tests/gate_test.cc:18:1: multi-line comment [-Wcomment]
tests/gate_test.cc:25:1: multi-line comment [-Wcomment]
```

### -Wmissing-field-initializers (11 places, GCC 14.2)
```
src/layout_check.cc:1695:58: missing initializer for member 'cell::layout::PostureChanges::from' [-Wmissing-field-initializers]
tests/layout_model_test.cc:495:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:499:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:500:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:502:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:503:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:504:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:505:55: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:506:57: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
tests/layout_model_test.cc:507:6: missing initializer for member 'cell::layout::{anonymous}::Thing::arm' [-Wmissing-field-initializers]
tests/layout_model_test.cc:509:8: missing initializer for member 'cell::layout::{anonymous}::Thing::name' [-Wmissing-field-initializers]
```

INSIDE PLACE SETS DIFFERING:
[]