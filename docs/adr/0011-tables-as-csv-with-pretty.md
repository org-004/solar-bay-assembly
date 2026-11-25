# 0011 Programs that print tables print CSV, and `--pretty` prints an aligned table

Status: accepted
Date: 2026-11-19
Decided by: the engineer of commit f8d4b94 (run not recorded)

**Context.** The programs of the project print the figures that documents quote
(`AGENTS.md`, "A number comes from a run"), and the figures are read in two ways: by a
plotting tool or another program, which wants a plain table, and by a person in a
terminal, who wants columns that line up. The first program of the kind was
`src/print_profile.cc` (f8d4b94, 2026-11-19); its header says: "The output is CSV, for a
plotting tool. With --pretty it is a table of aligned columns, for reading in a
terminal." That is the only reason the repository records.

**Decision.** A program that prints a table prints CSV on standard output, and
`--pretty` prints the same figures as a table of aligned columns. Eight programs have
the flag: `check_layout`, `plan_layout`, `print_perception`, `print_plan`,
`print_check`, `print_profile`, `run_cell` and `print_plane_fit` (`grep -l -e '--pretty'
src/*.cc`). `AGENTS.md`, "Map", lists the four that the documents quote (`run_cell`,
`print_perception`, `print_plan`, `print_check`), and `README.md` says "`--pretty`
(aligned table; CSV without it)". `check_layout` and `plan_layout` print CSV and then
`#` lines for what is not a row (`docs/layout.md`, the list of commands near its top).

**Alternatives.** The repository records no alternative that was weighed.

**Consequences.** One command gives both forms, so a figure in a document and a plot
come from the same program (ADR 0007). The rule is not applied everywhere: `print_bay`
and `print_points` print CSV and have no `--pretty`, `print_depth` prints a text summary
and CSV only with `--points`, and `print_cell`, `print_pallet` and `print_rrt` have no
flag either. A new program that prints a table is held to the rule by this record and
not by a check, since no script looks for the flag.

**Check.** The captures of `tools/architecture/programs.txt` run `print_check`,
`print_perception` and `print_plan` with `--pretty` and compare what they print (P2, ADR
0021): `grep -n pretty tools/architecture/programs.txt` prints those lines. `grep -L -e
'--pretty' src/print_*.cc` prints the programs that have no such flag today:
`print_bay`, `print_cell`, `print_depth`, `print_pallet`, `print_points` and
`print_rrt`.

**Migration.** None: the code already follows this.