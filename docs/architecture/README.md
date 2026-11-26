# How this repository is organized, and how to improve it

Start here when the task is to improve the repository's structure or quality, for
example "improve the repo to be more in line with the design principles without
breaking behavior". Read `AGENTS.md` first; its rules apply to everything below.

## The documents

| Document | What it is for | Read it |
|---|---|---|
| [`principles.md`](principles.md) | The eleven design principles, P1 to P11, each with the command that measures it | before any change of structure |
| [`backlog.md`](backlog.md) | Where the code departs from the principles, as items B1, B2, ... in the order to do them | to pick work |
| [`behavior.md`](behavior.md) | What "behavior" means here, what a capture holds and what it does not | before the first capture |
| [`baseline.txt`](baseline.txt) | The figures of `tools/architecture/measure.sh`, of the compiler's strict warnings (`W`) and of `clang-tidy` (`T`) at the last commit | when a figure moves |
| [`dependencies.md`](dependencies.md) | Every outside library: version, source, licence, who may use it | before adding or updating one |
| [`research.md`](research.md) | The sources behind the principles, cited as `[S7]` | when a principle's reason is in doubt |
| [`../adr/`](../adr/README.md) | The decision records: why the structure is as it is, and what was decided about where it goes | before departing from a convention |

The scripts are in `tools/architecture/`: `measure.sh` (the figures, in two seconds,
no Bazel), `behavior.sh` (capture and compare), `warnings.sh` and `tidy.sh` (the
guardrails' counts), `ci.sh` (what CI runs), and `deps.json`, the rules as data.

## The procedure for one item

Do one item in a run and stop, unless the task says otherwise. Work in the checkout
you were started in, on a branch as the task says; if it says nothing,
`git checkout -b refactor/b<n>`, commit there, and neither push nor merge.

1. **Pick the item.** Take the first item of `backlog.md` whose **State** is `open`,
   whose **After** items are all `done`, and whose **Touches** no other work under
   way shares. Work under way is what the task names, and any remote branch whose tip
   is newer than `main`'s: `git fetch origin && git for-each-ref
   --sort=-committerdate --format='%(committerdate:short) %(refname:short)'
   refs/remotes/origin | head`, then `git diff --name-only origin/main...<branch>`
   for each such branch. If a file of the item's **Touches** is in that list, take
   the next item. A checkout without a remote has no other work under way but what
   the task names. Do not pick by interest or by size.
2. **Measure and capture before**, on a clean checkout (`git status --short` prints
   nothing), with the captures outside the checkout:
   `tools/architecture/measure.sh > "$HOME/before.txt"` and
   `tools/architecture/behavior.sh capture "$HOME/capture-before"`.
   A capture builds the programs, runs them and runs every test but the gate;
   `behavior.md` gives the times. An item that touches documents alone needs no
   capture.
3. **Make the change the item's Change describes, and nothing else.** Edit only the
   files in its **Touches**; afterwards `git status --short` lists only those. If
   the change removes a departure that `tools/architecture/deps.json` lists, remove
   its entry there: `python3 tools/architecture/deps.py` says which.
4. **Capture after and compare:**
   `tools/architecture/behavior.sh capture "$HOME/capture-after"`, then
   `tools/architecture/behavior.sh compare "$HOME/capture-before" "$HOME/capture-after"`.
   Edit nothing while a capture runs. It must print `same`. The one exception: an item
   whose **Change** adds a test target or a line of `tools/architecture/programs.txt`
   widens the net, so `compare` names that target or that program's file as present on
   one side only, and nothing else; the item says so, and its proof is two captures
   after the change that compare `same`.
5. **Run the tests `AGENTS.md` asks for before code lands:** everything but the
   gate, `bazel test --config=opt --config=agent //...`, and the gate as `AGENTS.md`,
   "Build and test", says when the commit is to land on `main`.
6. **Measure after:** `tools/architecture/measure.sh > "$HOME/after.txt"` and
   `diff "$HOME/before.txt" "$HOME/after.txt"`. The lines that moved must be the ones
   the item's **Done when** names, at the figures it names, and no other line may
   have moved. Where **Done when** names a line that rises (a target that now lists
   a library it already used), that rise is part of the item. For an item about a
   compiler flag run `tools/architecture/warnings.sh` (it prints
   `W <flag> <places in src/ and tests/> <places outside>`), for one about a
   `clang-tidy` check `tools/architecture/tidy.sh` (it prints `T <check> <places>`).
7. **Put the record straight in the same commit.** In `backlog.md` set the item's
   state to `**State.** done`, followed by every line that moved, for example
   `(P5 16 to 12, P4.eigen 78 to 77)`. In `principles.md` correct every **Now** line
   that holds a figure that moved, whichever principle it is under. Update
   `baseline.txt`:
   `{ tools/architecture/measure.sh && sed -n '/^# not compared/,$p' docs/architecture/baseline.txt; } > "$HOME/baseline.txt" && cp "$HOME/baseline.txt" docs/architecture/baseline.txt`,
   and change a `W` or `T` line by hand to the count just printed. Change any
   sentence of `AGENTS.md`, `README.md` or `docs/` that the item or a decision record
   says changes with it. Then `tools/architecture/measure.sh --check` exits 0, and
   `tools/architecture/tidy.sh` exits 0 if code changed. Commit as `AGENTS.md`,
   "Commits", says, with the figures in the message.

**Stop and report, changing nothing further,** when:

- the comparison of step 4 differs. Do not explain the difference away and do not
  land the change. Put the tree back (`git checkout -- .`), and report the lines
  `compare` printed and what you think caused them;
- the item turns out to need a decision that no record under `docs/adr/` holds
  (where a function should live, what a new file is called, whether a group may
  include another). Write the record as `Status: proposed`, as below, and stop;
- the item cannot be done without something that is not a refactoring's to change:
  the contents of a frozen file (ADR 0020; `tools/architecture/measure.sh --list
  P1.frozen_files`), a tolerance, a test's bound, a layout, a figure of the hardware,
  what a program prints, or a rule of `AGENTS.md`. Say so in the report; the item
  belongs under "Needs the owner".

In the report, give the item, the figures before and after, what `compare` printed,
and everything you had to guess.

## When no item is open

Run `tools/architecture/measure.sh` and the two guardrails, and audit the code
against each principle: its **Check** says what the figure cannot see, and that is
where to look. Write what you find as new items at the end of `backlog.md`, in the
form of the items there (the eleven labels and `**State.** open`), or under "Left as it
is" or "Needs the owner" with the reason. Do none of the new items in that run
unless the task says so.

## Decisions

A convention you would change is probably a decision: look in
[`docs/adr/README.md`](../adr/README.md) before changing it. A change that departs
from an accepted record comes with a new record that supersedes it, in the same
commit (`AGENTS.md`, "Rules").

- **A new decision** is the next free number, `NNNN-short-title.md` under
  `docs/adr/`, in the form `docs/adr/README.md` gives, and a row in its table. Write it as
  `Status: proposed` unless your task's spec gives you the decision; nothing is
  built on a proposed record.
- **Superseding a record:** the new record names the old one in its Context and says
  what changed. In the old record only the status line changes, to
  `Status: superseded by NNNN`, and its row in the table with it.

## The guardrails, and what each costs

| Check | Command | Takes | Fails when |
|---|---|---|---|
| Dependency rules, figures, documents' paths | `tools/architecture/measure.sh --check` | 2 s, no Bazel | an include against the groups, a `deps` entry not used or missing, a figure that differs from `baseline.txt`, a path or a record that a document cites and that does not exist |
| Compiler flags that are errors | every build | nothing extra | one of the `-Werror=` flags of `.bazelrc` fires (ADR 0014) |
| Compiler flags that are counted | `tools/architecture/warnings.sh --check` | about 2 minutes | a flag of `--config=strict` finds more places than its `W` line |
| Static analysis | `tools/architecture/tidy.sh` | about 9 minutes at three jobs; needs `clang-tidy-18` | a check under `WarningsAsErrors` fires, or a check finds more places than its `T` line (ADR 0015) |
| Both of the first and the last, as CI runs them | `tools/architecture/ci.sh` | about 10 minutes | either fails |
| Behavior | `tools/architecture/behavior.sh capture`, `compare` | `behavior.md` | a program prints other bytes, or a test's result changes |

A finding is cured in the code, never silenced (P9). A count that reaches 0 moves
its flag or check to the errors in the same commit; the item says how.
