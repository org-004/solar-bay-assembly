Run 058's R1 analysis of rail_mag's one-bay plan (not landed on main: R1 found no pin-arm lever
with a ceiling of 8 s or more). `rail_mag_margins.csv` is `plan_layout <rail_mag> --margins` at
e7298e5. `steps.py <plan.json> <arm>...` prints each step of an arm with its motion, work and waits.
`probe_early.json` is rail_mag with `order.pin_tools: "early"`; planned with
`--budgets 5000,40000,160000,640000` it ends "the schedule is blocked" at module 0's set.
