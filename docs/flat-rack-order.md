# A shorter bay by the order of work (run 018)

The chosen layout, `flat_rack_3_pin_arms_ordered` (version v10), is the cell of
[`flat_rack_3_pin_arms`](flat-rack-cell.md) with one field more, `order`. It has the same
arms, tools, parts, process times, speeds, settle figures and margins (`clearance_m`
0.015 m, `plan_pad_m` 0.003 m, `joint_margin_deg` 5.0), and the same division of work
among the arms. `//:layout_model_test` (`Layouts.OrderOfWork`) checks that the two
files differ only in name, title, summary and `order`. Only the order of work differs,
and the planned bay is **560.45 s against 644.28 s** (-83.83 s, -13.0%). Every other
measure of the scorecard is as it was.

```
bazel run --config=opt //:plan_layout -- flat_rack_3_pin_arms_ordered --summary
bazel run --config=opt //:score_layout -- flat_rack_3_pin_arms_ordered
python3 -I tools/order/search.py try layouts/flat_rack_3_pin_arms.json --out DIR SPEC...
```

The recording of the bay as planned is [`docs/evolution/v10.gif`](evolution/v10.gif),
beside v09's [`docs/evolution/v09.gif`](evolution/v09.gif). Both are captioned with their
version and cycle time.

## The order

```json
"order": {
  "modules": [0, 1, 4, 2, 5, 7, 6, 3, 8],
  "modules_start": "after_their_rails",
  "pin_tools": "early"
}
```

- `modules`: the order in which the nine modules are set. Module m is still set on rails
  m and m+1, and the k-th module taken is still the nearest left on the pallet. Pin arm
  A's modules (0 to 3) are set 1st, 2nd, 4th and 8th; B's (4 to 6) 3rd, 5th and 7th;
  C's (7, 8) 6th and 9th. Each pin arm gets its first module early and the next before
  it has finished the one before.
- `modules_start`: `after_their_rails` takes a module as soon as the strap bolts of its
  own two rails are driven. Before, the first module waited for the last bolt (rail 9,
  by pin arm C).
- `pin_tools`: with `early`, a pin arm changes to its pin tool and takes its magazine as
  soon as its bolts are driven. It then waits for its first module where the magazine
  take's retreat left it. Before ("just in time"), it did both only once its first module
  was set, and those 17 s or so came before every pin arm's first hole.

The rules, and how the planner and the verifier hold them, are in the opening comment of
`src/layout_plan.h`. The verifier allows a third place to wait: the tool-change pose,
right after a tool change (R21). The bay's steps follow the order
(`src/layout_check.h`), and so does what is built at each work point
(`src/layout_scene.h`). Without `order`, every layout plans as before:
`flat_rack_3_pin_arms` plans byte for byte as its committed plan, 644.28 s.

## Time by phase

The phase "rails" ends with the last strap bolt, as before. Modules now start before
that, so the first set falls in the rails' phase and "modules" counts from the last bolt.

| Phase | Planned | By the time model | Planned over model | Before (`flat_rack_3_pin_arms`) | Difference | Basis |
|---|---|---|---|---|---|---|
| Tube in | 40.00 s | 40.00 s | 1.000 | 40.00 s | +0.00 s | derived |
| Clamp | 5.00 s | 5.00 s | 1.000 | 5.00 s | +0.00 s | derived |
| Rails | 180.67 s | 127.39 s | 1.418 | 180.42 s | +0.25 s | derived |
| Modules | 278.59 s | 198.70 s | 1.402 | 365.10 s | -86.51 s | derived |
| End (release and bay out) | 56.19 s | 45.00 s | 1.249 | 53.77 s | +2.42 s | derived |
| **Bay** | **560.45 s** | **416.09 s** | 1.347 | **644.28 s** | -83.83 s | derived |

## Each arm's share of the bay

Percent of the planned bay, 560.45 s (derived, ratio).

| Arm | Working | Moving | Waiting | Waiting before | Basis |
|---|---|---|---|---|---|
| module_arm | 188.19 s (33.6%) | 197.76 s (35.3%) | 174.51 s (31.1%) | 258.45 s | derived |
| pin_arm_a | 235.16 s (42.0%) | 112.79 s (20.1%) | 212.50 s (37.9%) | 294.12 s | derived |
| pin_arm_b | 176.18 s (31.4%) | 82.62 s (14.7%) | 301.64 s (53.8%) | 391.90 s | derived |
| pin_arm_c | 124.75 s (22.3%) | 60.49 s (10.8%) | 375.21 s (66.9%) | 456.74 s | derived |

## Every wait

25 waits, 1063.86 arm-seconds in all (derived), against 25 and 1401.20 before.

| Arm | Step | From | Seconds | Reason | Basis |
|---|---|---|---|---|---|
| module_arm | 2 | 0.00 s | 45.00 | for the tube to come in and be clamped | derived |
| module_arm | 15 | 157.05 s | 0.75 | for pin_arm_b to pass: module_arm_tool_combined_gripper_plate to pin_arm_b_wrist_2_link/geom698 at 158.05 s | derived |
| module_arm | 22 | 208.46 s | 0.25 | for pin_arm_b to pass: module_arm_arm_link_1_collision to pin_arm_b_wrist_2_link/geom698 at 209.26 s | derived |
| module_arm | 79 | 290.33 s | 5.00 | for pin_arm_b to pass: module_arm_tool_combined_gripper_plate to pin_arm_b_forearm_link/geom689 at 295.58 s | derived |
| module_arm | 166 | 436.94 s | 123.51 | bay out | derived |
| pin_arm_a | 2 | 0.00 s | 45.00 | for the tube to come in and be clamped | derived |
| pin_arm_a | 23 | 45.00 s | 19.24 | for step 4 (set rail 0) by module_arm | derived |
| pin_arm_a | 24 | 73.19 s | 13.14 | for step 6 (set rail 1) by module_arm | derived |
| pin_arm_a | 25 | 95.80 s | 11.30 | for step 8 (set rail 2) by module_arm | derived |
| pin_arm_a | 26 | 117.11 s | 7.77 | for step 10 (set rail 3) by module_arm | derived |
| pin_arm_a | 27 | 134.34 s | 6.79 | for step 12 (set rail 4) by module_arm | derived |
| pin_arm_a | 37 | 174.01 s | 68.51 | for step 34 (set module 0) by module_arm | derived |
| pin_arm_a | 37 | 242.52 s | 0.75 | for module_arm to pass: pin_arm_a_wrist_2_link/geom660 to module_arm_arm_link_1_collision at 243.72 s | derived |
| pin_arm_a | 166 | 520.45 s | 40.00 | bay out | derived |
| pin_arm_b | 2 | 0.00 s | 45.00 | for the tube to come in and be clamped | derived |
| pin_arm_b | 28 | 45.00 s | 111.10 | for step 14 (set rail 5) by module_arm | derived |
| pin_arm_b | 29 | 166.71 s | 4.37 | for step 16 (set rail 6) by module_arm | derived |
| pin_arm_b | 30 | 180.83 s | 4.44 | for step 18 (set rail 7) by module_arm | derived |
| pin_arm_b | 67 | 214.56 s | 74.82 | for step 64 (set module 4) by module_arm | derived |
| pin_arm_b | 166 | 498.54 s | 61.91 | bay out | derived |
| pin_arm_c | 2 | 0.00 s | 45.00 | for the tube to come in and be clamped | derived |
| pin_arm_c | 31 | 45.00 s | 155.16 | for step 20 (set rail 8) by module_arm | derived |
| pin_arm_c | 32 | 209.52 s | 6.69 | for step 22 (set rail 9) by module_arm | derived |
| pin_arm_c | 111 | 243.29 s | 119.28 | for step 108 (set module 7) by module_arm | derived |
| pin_arm_c | 166 | 511.37 s | 49.09 | bay out | derived |

## Which change bought how many seconds

Each row is a whole bay planned and verified with `tools/order/search.py try` from the
same base file. Only the `order` field differs. The variants were planned in run 018, and
their files are not kept.

| `modules` | `modules_start` | `pin_tools` | Bay planned | Against the old bay | Basis |
|---|---|---|---|---|---|
| 0-8 (as before) | after_all_bolts | just_in_time | 644.28 s | 0 | derived |
| 0-8 | after_their_rails | early | 592.56 s | -51.72 s | derived |
| 0, 1, 4, 2, 5, 7, 3, 6, 8 | after_all_bolts | just_in_time | 608.34 s | -35.94 s | derived |
| 0, 1, 4, 2, 5, 7, 3, 6, 8 | after_their_rails | just_in_time | 599.83 s | -44.45 s | derived |
| 0, 1, 4, 2, 5, 7, 3, 6, 8 | after_all_bolts | early | 569.84 s | -74.44 s | derived |
| 0, 1, 4, 2, 5, 7, 3, 6, 8 | after_their_rails | early | 561.32 s | -82.96 s | derived |
| **0, 1, 4, 2, 5, 7, 6, 3, 8** | **after_their_rails** | **early** | **560.45 s** | **-83.83 s** | derived |

The account, in the order the changes were made:

- **The order of modules: -35.94 s.** Before, pin arm C began only when module 7 was set,
  at 429 s, and ended last at 590.52 s. Interleaved, each pin arm starts on its first
  module soon after the first sets.
- **Early pin tools: -38.50 s more** (608.34 to 569.84). Each pin arm's tool change
  (8 s and its moves) and magazine take (4 s and its legs) leave the time between a set
  and the first hole, which is where the tail of the bay is.
- **Modules after their own rails: -8.52 s more** (569.84 to 561.32). This is the module
  arm's wait for rail 9's bolt before its first take. That bolt still ends at 225.67 s,
  but module 0 is now taken at 217.16 s.
- **The tail of the order: -0.87 s more.** Eight orders that interleave the pin arms'
  modules were planned with these rules. The best sets module 6 before module 3; the
  other seven came out at 561.32 s to 577.57 s.

The same rules with the old order give 592.56 s. The new order with the old rules gives
608.34 s. The two halves are worth about the same, and neither reaches 570 s alone.

Not tried: other orders of holes within a module. The order of locating
(`bay::kLocateOrder`) is part of the locating move's design (`src/bay_hardware.h`), and
the swage order is already a loop with no move longer than one rail pitch. Also not tried:
a module's holes divided between two pin arms. Three modules to each arm cannot be
reached: pin arm B is 0.32 m short of module 3's holes on rail 3, and C as far from module
6's on rail 6. A module divided by rail can be reached (every row of `check_layout`
passes), but its holes then alternate between two arms in the order of holes. One arm
then waits in a hole for the other, which the verifier refuses: 913.74 s and 470 samples
below the clearance, in a probe of run 018's planning.

## The bound, and how far the result is from it

The critical path of the new bay is pin arm A: it does 16 holes, more than any other
arm. A is free from its bolts and pin tool at 174 s, waits 68.51 s for module 0 (set at
242 s), and then works its four modules almost without a wait. The last work it does
(its tool change back) ends at 519.19 s, and bay out follows.
Pin arm C ends at 510.10 s and B at 497.27 s. The module arm's last set (module 8)
ends at 431.55 s.

The floor of this cell by the order of work, with the rails set first (derived):

- The module arm's own work and moves up to its last set are 380.55 s. After tube in
  and clamp (45 s), its last set cannot end before 425.55 s. The new bay has it at
  431.55 s: 6.00 s are waits for pin arm B to pass (0.75, 0.25 and 5.00 s).
- One module's four holes take a pin arm about 63 to 65 s. Back to the wait pose and the
  tool change take 13.8 s, and bay out 40 s. Release (5 s) overlaps the return.
- So the bay cannot be shorter than about 425.55 + 64 + 13.8 + 40 = **543 s**, whatever
  the order of modules, while the rails come first and each module's holes are one arm's.
  The result, 560.45 s, is about **17 s** from it.

The 17 s are pin arm A's: its 16 holes start no earlier than the first set, at 242 s.
To come nearer the floor, A would have to start its holes earlier, which means setting
modules between the rails (module 0 once rails 0 and 1 are on and bolted). Or one of
A's holes would go to B, which means a rule for a module whose holes two arms work
without one waiting in a hole. Neither is in this run.

The planner's method of one arm at a time (out of scope) costs the module arm 6.00 s of
waits in this bay, all for pin arm B to pass. The bay ends with pin arm A, so only the
5.00 s wait at 290 s could reach the end, and only if it delayed A's modules; it does not.

## Margins, before and after

From `plan_layout <name> --margins` (the verifier over the committed plan) and
`score_layout`.

| Margin | `flat_rack_3_pin_arms` (v09) | `flat_rack_3_pin_arms_ordered` (v10) | Basis |
|---|---|---|---|
| Least clearance over the schedule | 0.0181 m, pin arm B's wrist 2 to holder stand 6's saddle, step 115, 455.90 s | 0.0180 m, pin arm B's pin tool nose to the tube, step 71, 315.15 s | derived |
| Least clearance arm to arm | 0.0187 m, the held module's flange to pin arm B's nose, step 108, 380.25 s | 0.0188 m, the held module's flange to pin arm A's nose, step 50, 264.35 s | derived |
| Least joint margin | 8.64 deg, module arm joint 3, step 34 | 8.65 deg, module arm joint 3, step 34 | derived |
| Tightest pose at a work point (`approach` rows) | 0.0187 m, `locate 0/3 approach` | 0.0187 m, `locate 1/1 approach` | derived |
| Waits | 25, 1401.20 arm-seconds | 25, 1063.86 arm-seconds | derived |
| Rows of `check_layout` passed | 951 of 951 | 951 of 951 | derived |
