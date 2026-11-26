# Decision records

Why the repository is built and organized as it is, one decision to a file. A record
states a decision about the structure, the build, the tools or the rules of work, with
the forces behind it and what it costs. ADR 0001 says how records are kept: who may
write one, what the statuses mean, and that an accepted record is superseded by a new
one, never edited. The principles these decisions serve are in
[`docs/architecture/principles.md`](../architecture/principles.md); how to act on
them is in [`docs/architecture/README.md`](../architecture/README.md).

## The records

Records 0002 to 0013 write down decisions that were made before records were kept,
each with the reason the repository's history gives and the commit that made it.
Records 0014 to 0021 are decisions of the run that introduced the records. Record 0022
is the owner's decision of the same day. Record 0023 is spec 017's.

| No | Decision | Status | Date |
|---|---|---|---|
| 0001 | [Decisions about structure are kept as records in docs/adr, and an accepted record is superseded, not edited](0001-keep-decision-records.md) | accepted | 2026-11-25 |
| 0002 | [The build is Bazel with bzlmod, one version pinned, one `BUILD.bazel`](0002-bazel-with-bzlmod-one-pinned-version.md) | accepted | 2026-11-19 |
| 0003 | [Outside code comes from the registry or is fetched by hash](0003-outside-code-fetched-by-hash.md) | accepted | 2026-11-19 |
| 0004 | [MuJoCo 4.0.0 is the simulator, perception, planning and control are our own C++20](0004-mujoco-is-the-simulator.md) | accepted | 2026-11-19 |
| 0005 | [Control code reads sensor data only; a list of files says where truth is read](0005-control-reads-sensor-data-only.md) | accepted | 2026-11-19 |
| 0006 | [A bound on random scenes takes its draws from `src/portable_random.h`](0006-same-results-on-every-platform.md) | accepted | 2026-11-20 |
| 0007 | [A figure in a document comes from a run of `run_cell` or a `print_*` program](0007-a-figure-comes-from-a-run.md) | accepted | 2026-11-20 |
| 0008 | [OMPL is a second search behind a switch; the own RRT-Connect is the default](0008-ompl-behind-a-switch.md) | accepted | 2026-11-24 |
| 0009 | [A cell's layout is a JSON file under layouts/, checked by one program](0009-a-layout-is-data.md) | accepted | 2026-11-25 |
| 0010 | [A generated model is committed with its generator and a test that they agree](0010-generated-models-are-committed.md) | accepted | 2026-11-25 |
| 0011 | [Programs that print tables print CSV, and `--pretty` prints an aligned table](0011-tables-as-csv-with-pretty.md) | accepted | 2026-11-19 |
| 0012 | [The new cell is checked and planned kinematically before physics steps it](0012-kinematic-before-physics.md) | accepted | 2026-11-24 |
| 0013 | [The stand-in cell is kept as it is, with the gate on it, until it is replaced](0013-the-stand-in-cell-is-kept-until-replaced.md) | accepted | 2026-11-24 |
| 0014 | [Warnings for correctness are errors where the front end decides and code is clean](0014-compiler-warnings.md) | accepted | 2026-11-25 |
| 0015 | [clang-tidy 18 over a compile database finds defects; sanitizers run by hand](0015-static-analysis.md) | accepted | 2026-11-25 |
| 0016 | [An outside library is added pinned, from source, licensed, behind one target](0016-what-an-outside-library-must-be.md) | accepted | 2026-11-25 |
| 0017 | [The code is ten groups with one direction of dependency, held by a check on the include lines](0017-ten-groups-one-direction.md) | accepted | 2026-11-25 |
| 0018 | [The build stays one BUILD.bazel and src/ stays flat until the stand-in cell is removed](0018-one-build-file-until-the-stand-in-goes.md) | accepted | 2026-11-25 |
| 0019 | [Namespaces stay as they are: cell, and one nested namespace for each family of the new cell](0019-namespaces-follow-the-families.md) | accepted | 2026-11-25 |
| 0020 | [A refactoring does not edit the stand-in cell's files, and moves or renames them last](0020-refactoring-and-the-stand-in-cell.md) | accepted | 2026-11-25 |
| 0021 | [A refactoring proves itself by a capture before and after that compare the same](0021-behavior-is-what-the-net-holds.md) | accepted | 2026-11-25 |
| 0022 | [CI runs on 64-bit ARM, the platform the code is developed and proved on](0022-ci-runs-on-arm64.md) | accepted | 2026-11-25 |
| 0023 | [Scorecards live in scores/, versions in one registry, and pages are written from both](0023-scores-and-versions-live-beside-the-layouts.md) | accepted | 2026-11-26 |

## The form

A record is `NNNN-short-title.md` under `docs/adr/`, numbered from 0001 in the order
written, at most 80 lines, with these parts in this order:

```
# NNNN <a title that states the decision as a sentence>

Status: accepted | proposed | superseded by NNNN
Date: YYYY-MM-DD
Decided by: the owner, a spec's number, or a run

**Context.** The forces, with facts and paths.
**Decision.** What is done, in the present tense.
**Alternatives.** What else was weighed and why it lost.
**Consequences.** What gets easier, what gets harder, what it costs.
**Check.** The principle's check that holds it, or how a reader sees it is still true.
**Migration.** "None: the code already follows this.", or the backlog items.
```

The status is on a line of its own. A new record also gets a row in the table above.
When a record is superseded, only its status line changes, and its row with it.

## Where the cell's design decisions are written

Decisions about the cell itself (its layout, its parts, its margins, its planner's
verdict) are not records here. They stay where they were made, beside their figures:

| Document | Section | What is decided there |
|---|---|---|
| `docs/layout.md` | "Decisions"; "The comparison" | The closed-container cell of run 011: fifteen numbered decisions the brief left open, and the choice among five layouts |
| `docs/flat-rack-cell.md` | "Decisions"; "What is allowed"; "Margins" | The cell on a flat rack: fifteen numbered decisions, the owner's limits for the first cell, the margins and why |
| `docs/bay-hardware.md` | "The reference"; the tables of each part, rows marked `ours`; "Holding the tube"; "Locating a module before pinning"; "The container" | Which real parts the bay follows, and every figure that was designed here because no drawing is public |
| `docs/ompl-comparison.md` | "Verdict" | The rule, fixed before measuring, by which the project's own planner stays the default (ADR 0008) |
| `docs/gate.md` | "The tolerance" | What the gate's 3 mm and 0.15 degrees are, and that the owner chose them |
| `docs/ompl-integration.md` | "Decisions for the owner" | The options for building OMPL, as recommended to the owner (ADR 0003 and ADR 0008 record what was taken) |

The owner's decisions about the product are in the project brief, which the manager
keeps outside the repository; a spec carries what a run needs of them.
