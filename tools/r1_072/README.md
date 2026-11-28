# Run 072: borrowed swages after the arm's own modules (probe, not for main)

`make.py` builds the candidates from `_pallet_near` (v24), each with `swage_after_pins` and
`borrowed_last` (`<tag>`, `<tag>_s2`, `<tag>_s3`): run 070's `make.py` with the second field.
`queue.sh` screens each candidate one bay with a 120 s timeout (a deadlock shows within
about 90 s) and plans the passing ones three bays at seeds 1 and 2
(`--budgets 5000,40000,160000,640000`), one at a time, from a copied binary.
`chain.sh` prints each bay's critical chain from `plan_layout --margins`; `table.py` prints
the R2 table from the logs. `candidates/` holds the layouts; `results/` each candidate's
`check_layout` output (`c_<tag>.check`), plan summaries (`<tag>.log`, `<tag>_s2.log`),
seed 1's `--margins` (`<tag>.margins`) and chain digest (`<tag>.chain`), and `table.md`.
The reading is in the run's report.
