# 0010 A generated model is committed with its generator and a test that they agree

Status: accepted
Date: 2026-11-25
Decided by: the engineer of run 011, commit 68a1199 (run 007 report, 2026-11-24)

**Context.** `models/bay.xml` is the bay's model: a round tube, the rails with their
straps, the modules, and a site at every fastening. It was added in 1213319 (2026-11-24)
and was, in the words of `src/bay_model.h:5-6`, "once written out by a script that is
lost". Then the bay changed from ten modules to nine (the project brief, decided
2026-11-24), and every position, size and mass in the model follows from the count. The
owner accepted from run 007's report that "generated models get a committed generator"
to be done when a spec asks for it (the project brief).

**Decision.** `models/bay.xml` is committed and is the output of `//:gen_bay`
(`BUILD.bazel`, `src/gen_bay.cc`), which prints `BayModelXml()` of `src/bay_model.h`, a
function of the constants of `src/bay_hardware.h`. To change the bay the header is
changed and the file written again with `bazel run --config=opt //:gen_bay >
models/bay.xml` (`src/bay_model.h:13`). `tests/bay_model_test.cc:97` fails when the
committed file differs from the text by a single byte, shows the first line that differs
and prints that command. With the count at ten the generator printed the model of
1213319 byte for byte (the body of 68a1199). The generator and the test are entries of
`generated` in `tools/architecture/deps.json`.

**Alternatives.** The repository records none that was weighed. What can be said is what
the code shows: the other models, `models/cell.xml`, `models/container.xml` and
`models/flat_rack.xml`, are written by hand and checked against their headers
(`tests/container_test.cc`, `tests/flat_rack_test.cc`); the layouts' models are built at
run time and never written (`src/layout_model.cc`, ADR 0009).

**Consequences.** A change of the bay is one change to a header and one regenerated
file, and a forgotten regeneration fails a test. A change to the generator's output that
is not wanted is seen as a diff of one file. The cost is that the file may not be edited
by hand, not even to fix a typo, and that a test must exist for every output of this
kind. Two committed outputs have none today: `plans/flat_rack_2_pin_arms.json`
(`tests/plan_replay_test.cc` reads only the chosen layout's plan, and checks it against
its layout's SHA-256 without planning again) and `MODULE.bazel.lock`.

**Check.** P8: `tools/architecture/measure.sh --list P8` prints `MODULE.bazel.lock` and
`plans/flat_rack_2_pin_arms.json`, the two entries of `generated` that have neither a
test nor a stated reason why none can exist. `bazel test --config=opt //:bay_model_test`
holds `models/bay.xml`.

**Migration.** The backlog items that serve P8 in `docs/architecture/backlog.md`.