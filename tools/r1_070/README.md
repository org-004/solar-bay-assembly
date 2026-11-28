# Run 070: two pin arms swage one module (probe, not for main)

`make.py` builds the candidates from `_pallet_near` (v24), each with `swage_after_pins`,
into a layouts directory (`<tag>`, `<tag>_s2`, `<tag>_s3`); `queue.sh` plans them three
bays one at a time (`--budgets 5000,40000,160000,640000`); `chain.sh` prints each bay's
critical chain from `plan_layout --margins`; `table.py` prints the R2 table from the logs.
`candidates/` holds the layouts, `results/` each candidate's `check_layout` output
(`c_<tag>.check`), plan summary (`<tag>.log`) and `--margins` (`<tag>.margins`, seed 1).
`c53` (C swages 5/2 and 5/3) failed `check_layout` (C's carriage into B's at 5/3) and was
not planned; `a31_c52` passed the check and was not planned. The table and the reading
are in the run's report.
