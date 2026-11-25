# Design principles

Eleven rules for how this repository is organized, most valuable first. Each names the
facts of this code that make it worth having and the command that measures it:
`tools/architecture/measure.sh` prints one line `P<n> <figure> <what is counted>` for
every principle, each figure a count of departures, so "more in line with the
principles" is a figure going down. `tools/architecture/measure.sh --list P<n>` prints
the places counted. The figures under **Now** are those of
[`baseline.txt`](baseline.txt); `measure.sh --check` fails when one of them changes
without the baseline changing in the same commit. A detail line `P<n>.frozen` counts
the places in the stand-in cell's files, which a refactoring does not edit (ADR 0020);
until the stand-in cell is removed, a principle's target is its frozen figure.

The rules of `AGENTS.md` stand as written and are cited here, not repeated. The reasons
behind each principle are in [`research.md`](research.md), cited as `[S7]`; the
decisions that follow from them are the records under [`docs/adr/`](../adr/README.md);
where the code departs today, and what to do about it, is [`backlog.md`](backlog.md).

## P1 Dependencies point one way

**Rule.** Put every file under `src/` and `tests/` in exactly one group of
`tools/architecture/deps.json`, and include only headers of the file's own group and of
the groups that group may include.

**Why here.** `src/` is flat: 99 files in one directory and one Bazel package, so
neither a directory nor `visibility` says which piece may use which
(`grep -c visibility BUILD.bazel` prints 0). Two families of code share that directory:
the stand-in cell, which is to be replaced, and the new cell, which must not grow roots
into it. At the base commit no file of the stand-in includes the new cell and the
project's headers have no cycle; that is true by luck, and this rule keeps it. The
groups are `base`, `planning`, `hardware`, `perception`, `arm`, `standin`, `layout`,
`viewer`, `programs` and `layout_programs`; `python3 tools/architecture/deps.py --graph`
prints who includes whom.

**Sources.** [S1], [S2], [S26], [S48], [S57]

**Check.** `measure.sh` line `P1`: `#include` lines against the allowed direction.
`P1.cycles` counts cycles among the headers and `P1.namespace` the headers that open a
namespace their group may not. `measure.sh --check` fails on a new one at once: a
departure must be on the list in `deps.json`, the list can only shrink, and a file in
no group is an error.

**Now.** NOW_P1

**Target.** `P1` equals `P1.frozen`, then 0 when the stand-in cell is removed;
`P1.cycles` and `P1.namespace` stay 0. Never worse meanwhile.

**Exceptions.** None. A file that fits no group is a reason for a new decision record,
not for an exception.

**Decisions.** ADR 0017, ADR 0018, ADR 0019, ADR 0020

## P2 Nothing is restructured outside the net

**Rule.** Before you move, split or rewrite code, make sure a test target or a captured
program depends on it; where neither does, first pin what it does with a test, in a
commit of its own.

**Why here.** A refactoring is only safe where a change of behavior would be seen. No
test runs a program (`bazel query 'kind(cc_binary, deps(tests(//...))) intersect //:*'`
prints nothing), and 24 of the 99 files under `src/` are in the dependencies of no test
target, the programs that print every figure of the documents among them (`AGENTS.md`,
"A number comes from a run"). `tools/architecture/behavior.sh` closes most of that gap:
it runs the programs of `tools/architecture/programs.txt` with fixed arguments and
compares what they print byte for byte. What is still outside is listed in
[`behavior.md`](behavior.md), "Outside the net".

**Sources.** [S15], [S16], [S17], [S31], [S32]

**Check.** `measure.sh` line `P2` (needs Bazel): files under `src/` that neither a
test target nor a captured program depends on. `P2.untested` counts the files no test
target depends on, `P2.unrun` the programs no capture runs.

**Now.** NOW_P2

**Target.** `P2` is 0 but for the programs that need a display; `P2.untested` falls as
tests are written. Never worse meanwhile.

**Exceptions.** The viewers (`src/viewer.cc`, `src/view_cell.cc`, `src/view_layout.cc`)
open a window; nothing compares what they draw, and `behavior.md` says what to do
before touching them.

**Decisions.** ADR 0021

## P3 Control code cannot read the simulation's truth

**Rule.** Read the simulation's truth (a `True...` accessor, `true_scene()`, a body's
pose out of the simulation's `mjData`) only in a file that `deps.json` lists under
`truth` as allowed, and add a file to that list only with a decision record.

**Why here.** `AGENTS.md`, "Sensor data only", is the rule the whole project rests on:
a cell that corrects itself from what it senses cannot be developed against poses that
only a simulator knows. As worded it cannot be checked by a command. One test checks it
for one file (`tests/camera_cycle_test.cc:974` reads `src/pick_place.cc` and fails on a
`True` name), while any file that includes `src/cell.h` can call
`Cell::TrueModulePose`. This principle makes the rule a list of files: the scene
generator, the scorer and the tests may, everything else may not.

**Sources.** [S9], [S13]

**Check.** `measure.sh` line `P3`: files that match a truth pattern of `deps.json` and
are not on its allowed list.

**Now.** NOW_P3

**Target.** `P3` equals `P3.frozen`, then 0. Never worse meanwhile.

**Exceptions.** `src/collision.cc` and `src/layout_check.cc` read poses out of a
private copy of the model that the planner or the checker poses itself; `deps.json`
lists them with that reason.

**Decisions.** ADR 0005

## P4 Few outside libraries, pinned, each behind the groups that need it

**Rule.** Include an outside library only from a group that `deps.json` allows it in,
and add one only as ADR 0016 says: pinned by version or hash, built from source or
taken from the registry, with a licence that `dependencies.md` records, and reached by
the rest of the code through one target.

**Why here.** The build is reproducible because every outside file is pinned
(`MODULE.bazel`, `MODULE.bazel.lock`, `bazel/kuka.bzl`, `bazel/menagerie.bzl`,
`bazel/ompl.BUILD.bazel`), and it is small enough to hold that way: OMPL is included by
one file (`src/ompl_search.cc`), GLFW by one (`src/viewer_lib.cc`), nlohmann_json by
three. MuJoCo and Eigen are everywhere, which is their place in a simulation, but
`planning` includes no MuJoCo today and that is what lets `src/rrt.h` be tested without
a model. `bazel mod graph` fails on this repository (a flaw of the `mujoco` module), so
the tree toward the outside is described in [`dependencies.md`](dependencies.md) and
counted here.

**Sources.** [S3], [S43], [S44], [S46]

**Check.** `measure.sh` line `P4`: the number of outside dependencies. `P4.unpinned`
counts fetches without a version, commit or hash, `P4.group` the includes of a library
from a group that may not, and one line per library counts the targets that list it.

**Now.** NOW_P4

**Target.** `P4` does not grow without a decision record; `P4.unpinned` and `P4.group`
stay 0; the lines per library do not grow.

**Exceptions.** None.

**Decisions.** ADR 0002, ADR 0003, ADR 0004, ADR 0008, ADR 0016

## P5 A target lists exactly what its files include

**Rule.** Give every target in `BUILD.bazel` a `deps` entry for each library whose
header its files include, and no other.

**Why here.** `BUILD.bazel` holds every target of the repository in one package, where
Bazel checks nothing between them: a target compiles as long as some dependency drags
the header in. A `deps` list that says more than the code uses makes every change
rebuild and retest more than it must, in a suite where eight tests take 370 of 417
seconds (`AGENTS.md`, "Build and test"); one that says less breaks when an unrelated
target is tidied. The `deps` lists are also the only statement of structure Bazel has,
so they must be true before anything is built on them.

**Sources.** [S3], [S5], [S7], [S45]

**Check.** `measure.sh` line `P5`: `deps` entries none of whose headers the target's
files include, plus includes of a library that is not in `deps`, plus files under
`src/` and `tests/` that belong to no target or to several (`P5.unused_project`,
`P5.unused_outside`, `P5.missing`, `P5.files`). `measure.sh --check` fails on a new one.

**Now.** NOW_P5

**Target.** 0. Never worse meanwhile.

**Exceptions.** None.

**Decisions.** ADR 0018

## P6 The same on every platform and in every run

**Rule.** Draw random numbers only through `src/portable_random.h`, and let no result
depend on a clock.

**Why here.** `AGENTS.md`, "The same on every platform", asks for it where a bound
rests on random scenes; development is on 64-bit ARM and CI on x86-64. The standard
fixes the sequence of `std::mt19937` and leaves the distributions to each library, and
this code still draws through them in files that decide results: `src/rrt.cc:442`
samples with `std::uniform_real_distribution`, so a plan made on one machine is not
the plan made on another, and `src/rrt.cc:451` stops a search on wall-clock time. That
is why `plan_layout` repeats its bytes on one machine only, and why the comparison of
P2 is made on one machine.

**Sources.** [S11], [S12], [S14], [S52]

**Check.** `measure.sh` line `P6`: files under `src/` and `tests/`, other than
`src/portable_random.h`, that use a distribution of `<random>`, `std::rand`,
`std::random_device` or the wall clock.

**Now.** NOW_P6

**Target.** `P6` equals `P6.frozen`, then 0. Never worse meanwhile.

**Exceptions.** `std::chrono::steady_clock` for a time that is only printed, and OMPL's
planners that seed themselves from the clock (`src/ompl_search.cc`, ADR 0008): a figure
of theirs is a sample, never a bound.

**Decisions.** ADR 0006, ADR 0008

## P7 A fact has one home

**Rule.** Write a figure of the hardware, a tolerance, a conversion or a distance
function in one file, and have every other file take it from there.

**Why here.** The cell's figures decide whether a module fits its holes, and they have
drifted before: the stand-in still has ten slots where the bay has nine. Some facts
are written many times today. `mj_geomDistance`, which in MuJoCo 4.0.0 measures a
capsule through a box wrongly unless the model was mended by `src/layout_model.cc`, is
called from four files under `src/`, where one function (`LeastDistance` in
`src/layout_scene.cc`) is meant to be the only caller. The stand-in module's size and
the gate's 3 mm are spelled out as literals in files that include the header that
names them. `AGENTS.md`, "Frames and units", is the home of the units; this principle
is about the values.

**Sources.** [S10], [S23], [S24]

**Check.** `measure.sh` line `P7`: files that write again a fact whose home `deps.json`
names under `single_home`, one detail line for each fact. The list is of known
duplicates; a fact found written twice is added to it with its home, in the commit
that finds it.

**Now.** NOW_P7

**Target.** `P7` equals `P7.frozen`, then 0. Never worse meanwhile.

**Exceptions.** Tests may state an expected figure as a literal: a test that takes its
expectation from the header it checks would check nothing.

**Decisions.** ADR 0007, ADR 0009

## P8 A committed output agrees with its source

**Rule.** When a program or a script writes a file that is committed, record its
generator in `deps.json` under `generated` and keep a test that fails when the file
and its source disagree.

**Why here.** `models/bay.xml` shows the form: `//:gen_bay` prints it from
`src/bay_model.h`, and `tests/bay_model_test.cc:97` compares the committed file with
what the generator prints, byte for byte. Other committed outputs have no such test: a
plan under `plans/` that no test replays, `MODULE.bazel.lock`, and the pictures and
recordings under `docs/`, which need a display and can only name the command that made
them. A committed file that has drifted from its source is a wrong figure waiting to
be read.

**Sources.** [S17], [S23], [S44]

**Check.** `measure.sh` line `P8`: entries of `generated` with neither a test nor a
stated reason why no test can compare them.

**Now.** NOW_P8

**Target.** 0. Never worse meanwhile.

**Exceptions.** Pictures and recordings (`docs/img/`, `docs/*.gif`): their entries name
the command that makes them, and `docs/layout.md` lists those commands.

**Decisions.** ADR 0010

## P9 A finding is cured, not silenced

**Rule.** Add no place that a compiler flag of `.bazelrc` or a check of `.clang-tidy`
reports, and never quiet one with `NOLINT`, a pragma, an attribute or a `-Wno-` flag:
change the code so that the finding is gone.

**Why here.** The guardrails were switched on over some 65,000 existing lines, and
what they found was counted, not fixed. A count only means something if it cannot be
lowered by hiding: one `NOLINT` is cheaper than a cure and looks the same in the
figure. What was clean at the base commit is an error in every build (the `-Werror=`
flags of `.bazelrc`, `WarningsAsErrors` in `.clang-tidy`); what was not is counted
under `--config=strict` and by `tools/architecture/tidy.sh`, and those counts may only
fall.

**Sources.** [S27], [S33], [S34], [S36], [S41], [S42], [S47], [S50]

**Check.** Three guardrails. `measure.sh` line `P9`: places under `src/`, `tests/`,
`BUILD.bazel` and `.bazelrc` that silence a finding. `tools/architecture/warnings.sh
--check`: the places each flag of `--config=strict` finds, against the `W` lines of
`baseline.txt`. `tools/architecture/tidy.sh`: the places each check finds, against the
`T` lines, and no check under `WarningsAsErrors` fires.

**Now.** NOW_P9

**Target.** `P9` stays 0; every `W` and `T` line reaches 0, and the commit that brings
one there makes the flag or the check an error. No line ever grows.

**Exceptions.** The two `-Wno-` lines that `.bazelrc` had at the base commit
(`-Wno-psabi`, and `-Wno-error=maybe-uninitialized` for MuJoCo's files), each with its
reason beside it.

**Decisions.** ADR 0014, ADR 0015

## P10 A document points at what exists

**Rule.** When you move, rename or remove a file or a target, change every document
that names it in the same commit.

**Why here.** Agents find their way here by `AGENTS.md`'s map and the documents under
`docs/`, and they follow a path or a command as written. `docs/layout.md:667` still
tells the reader to run `//:print_layout_moves`, a target that left with
`src/print_layout_moves.cc`. Every refactoring in the backlog moves or renames
something, so without this rule each one would leave the documents a little more
wrong.

**Sources.** [S24], [S28], [S29], [S30]

**Check.** `measure.sh` line `P10`: paths in backticks and Bazel labels, in
`AGENTS.md`, `README.md` and the documents under `docs/`, that do not exist. For the
documents under `docs/architecture/` and `docs/adr/`, `measure.sh --check` fails on one.

**Now.** NOW_P10

**Target.** 0. Never worse meanwhile.

**Exceptions.** Documents that record a state that is gone on purpose, listed in
`deps.json` with the reason (`docs/ompl-integration.md` describes four trial builds
whose files were never on `main`).

**Decisions.** ADR 0001

## P11 A file is small enough to read and says what it is

**Rule.** Open every header with a comment that explains the piece, and when a change
would take a `.cc` file past 1,000 lines or a header past 600, split the file by what
it does first.

**Why here.** `AGENTS.md`'s map sends the reader to each header's opening comment
before the body, and that works: every header under `src/` has one, of 6 to 275 lines.
Size is the other half. An agent reads a file whole to change it safely, and
`src/layout_check.cc` is 2,610 lines, `src/layout_replay.cc` 1,812 and
`src/layout_plan.cc` 1,462: three files of the new cell, written in two runs, that
every later change to the layout's checks and plans has to read. The limits are those
of `deps.json`, `size`.

**Sources.** [S1], [S29], [S56]

**Check.** `measure.sh` line `P11`: `.cc` files under `src/` over the line limit,
headers over theirs, and headers whose opening comment is shorter than five lines
(`P11.cc`, `P11.h`, `P11.comment`).

**Now.** NOW_P11

**Target.** `P11` equals `P11.frozen`; `P11.comment` stays 0. Never worse meanwhile.

**Exceptions.** Tests are not counted: a long test file is many short cases.
`src/bay_hardware.h` is a table of the hardware's figures with their sources and is
left whole (`backlog.md`, "Left as it is").

**Decisions.** ADR 0017
