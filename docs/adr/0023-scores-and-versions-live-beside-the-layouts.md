# 0023 Scorecards live in scores/, versions in one registry, and pages are written from both

Status: accepted
Date: 2026-11-26
Decided by: spec 017 (a scorecard for every layout and a browsable history of the cell)

**Context.** The owner steers the cell by ranked measures: cycle time and expected
failures per bay first, then part count and placement error, then the mass in transit and
the tolerated error of the infed parts. Before spec 017 these figures were scattered or
absent: cycle time was a field of a plan (`plans/*.json`, `bay_planned_s`), the mass was
one row of `check_layout` held against another limit, and nothing computed part count or
expected failures. The owner also asked for "some kind of versioning for the different
layouts & their corresponding motion plan" and for a page for each research direction.
A layout is already a file named for itself (ADR 0009), and a plan is a 770 kB file that
names its layout's sha256.

**Decision.** `//:score_layout` (`src/score_layout.cc` over `src/scorecard.h`) prints a
layout's scorecard. Each measure has its rank, its direction and how it is known, and the
gates come after. Every figure that the checker or the planner computes is read from
their code: `CheckLayout`, `VerifyPlan`, and `TransportMassParts` of
`src/layout_rows.h`. The committed scorecard of every layout with a committed plan is
`scores/<name>.json`, which holds the sha256 of the layout, the plan and
`scores/rates.json`. `//:scorecard_test` fails when a committed scorecard is not what
the program computes now. Versions are entries of one registry,
`versions/versions.json`, oldest first. A version names its layout and plan by path and
sha256 and does not copy them, and `//:versions_test` fails when a registered file
changes. A later change to the chosen layout or its plan therefore adds a layout file and
a version, with `tools/evolution/add.sh`. `tools/evolution/pages.py` writes the pages of
the directions (`docs/directions/`), their charts and the README's results from the
registry and the scorecards, and `tools/architecture/ci.sh` (its third step,
`pages.py check`) fails when a committed page differs from what it writes. Recordings of the versions, with burned-in captions, are
under `docs/evolution/`. The scorecard library is the fifth target that lists
`@nlohmann_json` (`P4.nlohmann_json` 4 to 5). Nothing in `MODULE.bazel` is added, and
the pages are written in Python with the standard library only.

**Alternatives.** Measures written by hand in a document after each run: they would
drift from the code, and two layouts could not be compared by a command. A copy of each
version's layout and plan in a directory of its own: a layout's name is already its
version, and a plan is 770 kB. Git tags for versions: a squashed landing would need a
step after the landing, and the registry needs none. One combined score: the owner ranked
the measures and did not weight them. A JSON reader of the scorecard's own, to keep
`P4.nlohmann_json` at 4: a second parser is worse than one more target behind the library
that the layout code already uses. A Bazel test of the pages: `rules_python` or `rules_shell`
would add to `MODULE.bazel` (P4), and a `cc_test` that starts `python3` needs POSIX
headers that `tools/architecture/deps.py` does not allow, so the check is a step of
`ci.sh`, which CI's architecture job runs. For the same reason `score_layout`'s work is a
function of the library (`RunScoreLayout`), which its test calls in-process.

**Consequences.** A spec that should move a measure can quote `score_layout` before and
after. Changing the chosen layout costs a new file and a version, which is the point.
`scorecard_test` computes every committed plan's scorecard, about 16 s each. Python 3
must be on the machine that runs `ci.sh`, as it already is for `tools/architecture/`.
The three new tests raise `P4.googletest` from 42 to 45, as every new test does.
The pages' entry of `generated` in `tools/architecture/deps.json` is excused from a Bazel test
because `ci.sh` compares them, so `P8.no_test` goes from 6 to 7.

**Check.** `bazel test --config=opt //:scorecard_test //:versions_test
//:score_layout_test` and `python3 tools/evolution/pages.py check`. `tools/architecture/programs.txt` captures `score_layout`
for each committed plan and for `--all`, and the `generated` entries of
`tools/architecture/deps.json` name each generator and its test (P8).

**Migration.** None: the code already follows this.
