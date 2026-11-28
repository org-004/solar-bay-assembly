Run 073, three bays, `--budgets 5000,40000,160000,640000`; every candidate is v24 + A swaging 3/1, 3/0 and B swaging 6/1, 6/0 with `swage_after_pins` and `borrowed_last`; `a31_b61` seeds 1 and 2 are run 072's logs. Gate (R3): three verified seeds, mean at most 388.08 s (v24 393.08 s).

| candidate | s1 cycle / interval / one bay | s2 | s3 | mean (seeds) | verifies s1/s2/s3 |
|---|---|---|---|---|---|
| a31_b61 | 385.68 / 372.23 / 408.88 | 389.12 / 371.36 / 415.04 | 385.81 / 369.19 / 413.14 | 386.87 (3) | yes/yes/yes |
| b2 | 376.99 / 361.05 / 404.41 | 382.34 / 361.54 / 419.10 | - | 379.66 (2) | no/yes/- |
| a2_b2 | 375.27 / 359.75 / 401.85 | 378.60 / 362.12 / 406.96 | 376.29 / 360.63 / 402.42 | 376.72 (3) | no/yes/no |

Critical chain of each bay (chain.py over --margins):

- a31_b61_s3
  - bay 0: module_arm>c | waits: c swage 7/0 17.2s pin_arm_b | next: pin_arm_b swage 6/0 gap 15.99
  - bay 1: module_arm>c | waits: c swage 7/0 13.2s pin_arm_b | next: pin_arm_b swage 6/0 gap 16.11
  - bay 2: module_arm>c | waits: c swage 7/0 9.2s pin_arm_b | next: pin_arm_b swage 6/0 gap 16.20

- b2
  - bay 0: module_arm>c | waits: - | next: pin_arm_b swage 5/0 gap 3.92
  - bay 1: module_arm>c | waits: - | next: pin_arm_b swage 5/0 gap 2.44
  - bay 2: module_arm>c | waits: - | next: pin_arm_b swage 5/0 gap 2.32

- a2_b2
  - bay 0: module_arm>c | waits: - | next: pin_arm_b swage 5/0 gap 4.41
  - bay 1: module_arm>c | waits: - | next: pin_arm_b swage 5/0 gap 1.94
  - bay 2: module_arm>c | waits: - | next: pin_arm_b swage 5/0 gap 2.07

Every plan that does not verify fails at step 166 (bay 1's set rail 0, 355-358 s): pin_arm_b's move after its borrowed swage passes through the module arm's base or link 1 (run 070's scheduler gap). b1 passed the 120 s screen and was not planned (dropped); b2's seed 3 was not planned (dropped).
