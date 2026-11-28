# Run 073: borrowed swages at a chosen place in the arm's order (probe)

`make.py` builds the candidates from `_pallet_near` (v24): run 072's `a31_b61` (A swages B's
3/1 and 3/0, B swages C's 6/1 and 6/0, with `swage_after_pins` and `borrowed_last`) plus
`borrowed_after`, which puts an arm's borrowed swages right after its n-th own module of the
bay in set order (`<tag>`, `<tag>_s2`, `<tag>_s3`). `plan.sh` plans tags three bays one after
another; `queue.sh` screens each candidate one bay with a 120 s timeout and plans the
passing ones three bays at seed 1 (`--budgets 5000,40000,160000,640000`), one at a time,
from a copied binary on one core. `chain.sh` prints each bay's critical chain from
`plan_layout --margins`; `table.py` prints the R2 table from the logs. `results/` holds
`check_layout` output (`c_<tag>.check`), plan summaries (`<tag>.log`, `<tag>_s2.log`,
`<tag>_s3.log`), seed 1's chain digest (`<tag>.chain`) and `table.md`. The reading is in the
run's report.
