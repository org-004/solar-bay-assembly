# Run 067: pin arm hole work across A, B and C (probe, not for main)

`make.py` builds the candidates from `_pallet_near` into a layouts directory;
`queue.sh` plans them three bays one at a time (`--budgets 5000,40000,160000,640000`);
`chain.sh` prints each bay's critical chain from `plan_layout --margins`.
`candidates/` holds the layouts, `results/` each plan's summary (`<tag>.log`),
`--margins` (`<tag>.margins`) and `torque_demand --all-bays` output.
`results/st30_swage_only.*` is the first st30 (swage stop only), which the checker
ignores (a hole is posed once, at its locate stop): its plan is _pallet_near's.
The table and the reading are in the run's report.
