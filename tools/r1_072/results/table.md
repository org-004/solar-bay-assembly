Run 072, three bays, `--budgets 5000,40000,160000,640000`, every candidate with `swage_after_pins` and `borrowed_last`. Threshold (R3): two-seed mean at most 386.56 s.

| candidate | what moved | s1 cycle / interval / one bay | s2 cycle / interval / one bay | mean | verifies s1/s2 |
|---|---|---|---|---|---|
| v24 (`_pallet_near`) | nothing | 387.23 / 375.08 / 408.03 | 395.89 / 386.86 / 423.58 | 391.56 | yes/yes |
| a31 | A swages 3/1, 3/0 | 388.93 / 374.84 / 412.67 | 390.92 / 374.99 / 416.92 | 389.93 | yes/yes |
| a31_b60 | A 3/1, 3/0; B 6/0 | 387.27 / 372.73 / 411.88 | 389.79 / 373.36 / 415.04 | 388.53 | no/yes |
| a31_b61 | A 3/1, 3/0; B 6/1, 6/0 | 385.68 / 372.23 / 408.88 | 389.12 / 371.36 / 415.04 | 387.40 | yes/yes |
| a30 | A swages 3/0 | 389.60 / 375.84 / 413.67 | 391.00 / 375.24 / 416.92 | 390.30 | yes/yes |

Critical chain of each bay at seed 1 (`plan_layout --margins`, `chain.sh`; waits over 1 s; v24 from run 070's `ctl`, the same plan):

- **v24**
  - bay 0: module_arm>c | waits: - | next: pin_arm_b swage 5/0 gap 3.43
  - bay 1: module_arm>c | waits: c swage 7/3 2.5s module_arm | next: pin_arm_b swage 5/0 gap 1.64
  - bay 2: module_arm>b | waits: b swage 3/0 18.0s pin_arm_a; b swage 4/0 1.5s pin_arm_a | next: pin_arm_c swage 8/0 gap 2.41
- **a31**
  - bay 0: module_arm>c | waits: c swage 6/0 1.8s pin_arm_b; c swage 7/0 4.2s pin_arm_b | next: pin_arm_a swage 3/0 gap 24.42
  - bay 1: module_arm>c | waits: c swage 6/0 1.8s pin_arm_b; c swage 7/0 2.5s pin_arm_b | next: pin_arm_a swage 3/0 gap 27.79
  - bay 2: module_arm>c | waits: c swage 6/0 1.8s pin_arm_b; c swage 7/0 2.5s pin_arm_b | next: pin_arm_a swage 3/0 gap 24.81
- **a31_b60**
  - bay 0: module_arm>c | waits: m set module 5 6.2s pin_arm_c; c locate 8/0 4.5s pin_arm_b | next: pin_arm_b swage 6/0 gap 13.97
  - bay 1: module_arm>c | waits: c swage 7/0 9.5s pin_arm_b | next: pin_arm_b swage 6/0 gap 21.92
  - bay 2: module_arm>c | waits: c swage 7/0 9.5s pin_arm_b | next: pin_arm_b swage 6/0 gap 22.05
- **a31_b61**
  - bay 0: module_arm>c | waits: m set module 5 3.2s pin_arm_c; c locate 8/0 4.5s pin_arm_b | next: pin_arm_b swage 6/0 gap 8.86
  - bay 1: module_arm>c | waits: c swage 7/0 11.2s pin_arm_b | next: pin_arm_b swage 6/0 gap 16.94
  - bay 2: module_arm>c | waits: c swage 7/0 12.0s pin_arm_b | next: pin_arm_b swage 6/0 gap 16.92
- **a30**
  - bay 0: module_arm>c | waits: c swage 6/0 4.5s pin_arm_b; c swage 7/0 2.5s pin_arm_b | next: pin_arm_b swage 5/0 gap 31.48
  - bay 1: module_arm>c | waits: c swage 6/0 1.8s pin_arm_b; c swage 7/0 2.5s pin_arm_b | next: pin_arm_b swage 5/0 gap 31.55
  - bay 2: module_arm>c | waits: c swage 6/0 2.8s pin_arm_b; c swage 7/0 2.5s pin_arm_b | next: pin_arm_b swage 5/0 gap 31.53

a31_b60 seed 1 fails the verifier: at 354.20 s (bay 0's end, step 166 = bay 1's set rail 0) pin_arm_b's move after its borrowed swage 6/0 (352.90 to 358.74 s) passes through the module arm's base while the module arm waits for bay 1's tube (−0.0745 m, 36 samples below 0.015 m); run 070's scheduler gap, reported, not fixed.
