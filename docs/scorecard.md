# The scorecard of a layout

The owner steers the cell by ranked measures and does not weight them: there is no
combined score. The scorecard of a layout lists each measure with its rank, which way is
better and how it is known, then the gates the layout must pass. The definitions live in
the opening comment of `src/scorecard.h`; this document says them in words, and the
program prints the figures.

## Running it

- `bazel run --config=opt //:score_layout -- <name>` prints the scorecard of one layout:
  the measures, the gates, the breakdowns of the part count, the driven axes and the mass
  in transit, and the operations by kind. It prints from the committed `scores/<name>.json`
  when that file's three sha256 (layout, plan, rates) match the files as they are, which
  takes well under a second; otherwise it computes the scorecard afresh, running
  `CheckLayout` and `VerifyPlan` (about 16 s), and says so on standard error.
- `-- <name> --json` prints the same scorecard as JSON; `-- <name> --write` writes it to
  `scores/<name>.json`.
- `-- --all` prints one row per layout that has a committed scorecard (`scores/*.json`),
  one column per measure, and the gates as one column; `-- --all --write` first writes the
  scorecard of every layout that has a committed plan.
- The exit code is 0 when every gate passes, 1 when one fails, and 2 for a layout without
  a committed plan, an unknown layout or a usage error.

A layout needs its file `layouts/<name>.json` and its committed plan `plans/<name>.json`.
Every figure that the checker or the planner computes is taken from their code
(`CheckLayout`, `VerifyPlan`, `TransportMassParts` of `src/layout_rows.h`) and never
computed a second time. A figure that is not known is printed as `none`.

How a figure is known, in the word of the `known` column:

- **measured**: a run in physics produced it.
- **planned**: the figure of a committed motion plan.
- **counted**: counted from the layout file or the plan by a rule written in
  `src/scorecard.h` and in this document.
- **modelled**: a model over counted operations, such as the expected failures.
- **none**: not known. The basis column says why.

The figures of the versions older than layouts (`versions/versions.json`, `figures`) use
two words more, because their documents report figures of those kinds: **assumed** (a
figure chosen or designed, marked `ours` or an owner's choice in its document) and
**derived** (computed from other figures by the formula its document gives).

## The measures

Ranks are the owner's: P0 first. Within a rank the measures are not ordered.

| Rank | Measure | Better | Known | Unit |
|---|---|---|---|---|
| P0 | `cycle_s` | lower | planned | s |
| P0 | `failures_per_bay` | lower | modelled | 1 |
| P0 | `physics_share` | higher | counted or measured | 1 |
| P1 | `part_count` | lower | counted | 1 |
| P1 | `arms` | lower | counted | 1 |
| P1 | `driven_axes` | lower | counted | 1 |
| P1 | `placement_mm` | lower | none | mm |
| P2 | `mass_in_transit_kg` | lower | counted | kg |
| P2 | `input_tolerance` | higher | none | mm |
| P2 | `plan_s` | lower | none | s |

**`cycle_s`** (P0, lower, planned). The time a bay takes when the cell builds bays one
after another: the committed plan holds three bays planned back to back
(`src/layout_bays.h`), and `cycle_s` is (end of bay 2's `bay_out` − start of bay 0's
`tube_in`) / 3. It is kinematic, on assumed speeds and process times
(`docs/flat-rack-cell.md`), so it is a planned figure and not a measured one. Two figures
stand beside it and are not ranked measures: `one_bay_s`, bay 0 alone from the start of
its `tube_in` to the end of its `bay_out` (the plan's `bay_planned_s`, and what `cycle_s`
was before the plans held three bays), and `interval_s`, from the end of bay 1's `bay_out`
to the end of bay 2's. `score_layout` prints both beside `cycle_s`, and `--all` and
`--json` carry them. A plan of one bay has `cycle_s` equal to `one_bay_s` and no
`interval_s`. [`docs/three-bays.md`](three-bays.md) gives the rules of the three bays, the
layouts' figures and where the time between bays goes.

**`failures_per_bay`** (P0, lower, modelled). The sum, over the kinds of operation, of the
count of operations of that kind in one bay of the plan (bay 0's steps) times the rate of that kind in
`scores/rates.json`. The rates are described below. The program also prints the share of
the figure that rests on rates whose basis is `ours`.

**`physics_share`** (P0, higher, counted or measured). The share of one bay, `one_bay_s`,
that a committed run in physics carried out; a plan of three bays does not dilute it. A committed run is the file `physics/<layout>.json`,
written by `//:run_physics`; it holds, for each step the run covered, what the run measured
(see `src/physics_record.h`). A step counts as carried out in physics when two things hold:
its least clearance in physics, between the moving arm with its tool and part and everything
else, is at least the layout's `clearance_m` (0.015 m for the flat rack layouts), and the
module the run carried ends within the gate on its rails: at most 3 mm from its place in the
rails' plane and at most 0.15 degrees of yaw, measured from the rails as they lie. The
figure is the sum of the planned seconds of the steps that count divided by `one_bay_s`. It is
`measured` when a record is used. A layout without a record has 0, `counted`, as before. A
record that does not agree with the committed plan (another plan sha256, a step the plan
does not have or with other times, a stored flag the rule does not give) counts nothing: the
figure is 0 and the basis says what does not agree. The scorecard file then holds the
sha256 of the record (`physics_sha256`), so that a changed record makes the committed card
stale.

**`part_count`** (P1, lower, counted). The pieces of equipment of the cell, by the rule
below.

**`arms`** (P1, lower, counted). The arms of the layout.

**`driven_axes`** (P1, lower, counted). Counted from the layout: each arm's joints (six for
every arm of `src/arm_catalog.h`), each arm's track (one), each tool (one, because every
tool of these layouts drives something), each grip of the holder (one) and the holder's end
unit's angle pin (one).

**`placement_mm`** (P1, lower, none). The placement error of a module. Nothing is sensed
in the flat rack cell, so no error is measured that the cell could know. Run 019's run in
physics sets one module down 0.20 mm and 0.005 degrees from its place
([`flat-rack-physics.md`](flat-rack-physics.md)), but that is the true pose in simulation
without sensing, and by "What would make it measured" below this measure needs the
modules held and sensed; it stays `none`. The stand-in cell has a figure
(`docs/gate.md`), but it is another cell's and is not entered here.

**`mass_in_transit_kg`** (P2, lower, counted). The mass the owner's definition counts, from
the parts of the transport row. The rule is below.

**`input_tolerance`** (P2, higher, none). The largest error of the infed parts that the
cell still places in tolerance. Nothing is sensed in the cell, so no error of the infed
parts is measured or corrected.

**`plan_s`** (P2, lower, none). The time that `plan_layout` took to plan the bay. The
committed plan does not hold that time, so it is not known.

## The gates

A layout passes when all three hold.

| Gate | Figure | Limit |
|---|---|---|
| `check_rows_failed` | rows of `check_layout` that do not pass | at most 0 |
| `least_clearance_m` | the least distance over the plan's schedule, from `VerifyPlan` | at least the layout's `margins.clearance_m` |
| `mass_in_transit_kg` | the measure of the same name | at most 13,608 kg (30,000 lb, the owner's limit for one container) |

## The rule of the part count

The part count is one for each physical instance of equipment. The kinds, as the program
prints them:

- `arm`: each arm.
- `track`: each arm's track.
- `tool`: each tool of each arm.
- `tool_stand`: each tool's stand, for a tool that waits off the flange (`wait_size` not
  zero).
- `holder_stand`: the holder's stands, one per grip.
- `holder_grip`: the holder's grips, one saddle per grip.
- `holder_end_unit`: the holder's end unit, one.
- `item`: each item of the layout (feeders, lanes, carts), whether it travels or not.
- `pin_magazine`: each pin magazine that a stock inside the cell holds (a stock whose part
  is a magazine counts its `holds` of them, the number on its nest when the nest is full).
- `rail_magazine`: each tool with a `rail_magazine` (run 056, `src/layout_rail_magazine.h`),
  one; a row only on the cards of layouts that have one.

Not counted: the parts of the bay (tube, rails, modules, pins, bolts), and what they
arrive in (the pallets and the rail rack, which are stocks outside). The bay's parts are
what the cell makes its output from and not equipment of the cell. A pallet or a rack is
the container of a part that arrives and is not part of the cell. A pin magazine is counted
because the spec names magazines among the equipment of the cell, while the pins in it are
not. This choice could go the other way: the layout says a pin arm's tool takes one
magazine for each bay, which makes a magazine also what pins arrive in. The mass in
transit takes the other side of the same question (below): the 3 kg that the layout gives
its stocks of magazines are left out as infed parts. Counted the other way, the part
count of `flat_rack_3_pin_arms` would be 33 and its mass in transit 3,429.3 kg.

The breakdown for `flat_rack_3_pin_arms`, as `score_layout` prints it (39 in all):

| Kind | Count |
|---|---|
| arm | 4 |
| track | 4 |
| tool | 7 |
| tool_stand | 6 |
| holder_stand | 4 |
| holder_grip | 4 |
| holder_end_unit | 1 |
| item | 3 |
| pin_magazine | 6 |

The driven axes of the same layout (40 in all): `arm_joint` 24, `track` 4, `tool` 7,
`holder_grip` 4, `holder_angle_pin` 1.

## What counts toward the mass in transit

The owner's definition: the mass in transit is "the robotics assembly in transit (not
including infed parts or the flat bed container itself)". The limit is 30,000 lb, which is
13,608 kg, per container.

The measure is the parts of `check_layout`'s transport mass row (`TransportMassParts`),
less the infed parts. The table names every component of that row, then the things that are
already outside it.

| Component | Counts | Why |
|---|---|---|
| `arms` | yes | An arm by its catalogue mass (`src/arm_catalog.h`). It is robotics and it travels. |
| `tracks` | yes | An arm's track, beam and carriage. It is robotics and it travels. |
| `tools` | yes | Each tool an arm has. It is robotics and it travels. |
| `holder` | yes | The holder travels inside and holds the tube. It is part of the robotics assembly. |
| `items inside` | yes | An item that travels inside, such as the pin lane. It is part of the assembly. |
| `stocks inside` | no | A stock inside holds infed parts: the pin magazines with their pins, fed in for each bay. The owner's definition leaves infed parts out. The part count counts the magazines all the same (see its rule): a choice that could go the other way, worth 3 kg here. |
| Items that do not travel (the transfer cart, the pin feeder) | no, outside the row | They are not on the flat rack in transit, so the row does not add them. |
| Pallets | no, outside the row | They are infed parts' containers and always outside the cell: for a layout on the flat rack the loader refuses a pallet that is not `outside`. |
| The rail rack | no, outside the row | It is a stock outside (`outside: true`) that holds infed parts. |
| The flat rack itself | no, outside the row | The owner's definition names the flat bed container. The transport row's limit is what the flat rack leaves for the cell. |
| The bay's parts (tube, rails, modules, pins, bolts) | no, outside the row | They are infed parts, not robotics. |

`check_layout` adds all six components of the row, so its transport row for
`flat_rack_3_pin_arms` is 3,429.3 kg against its limit of 17,900 kg. That includes 3 kg of
pin magazines (three stocks of 1 kg each) which the scorecard leaves out, so the scorecard
says 3,426.3 kg. The two limits differ because they answer different questions: 17,900 kg
is what the flat rack leaves for the cell, and 13,608 kg is the owner's limit of the
container.

## The rates

A rate is the probability that one operation of the kind fails so that the bay stops or
needs a person. `scores/rates.json` holds one rate for each of 13 kinds of operation, with
its basis, either `source` (a document we opened, named in its `source` field) or `ours`
(our own order-of-magnitude judgment). All 13 are `ours` today.

| Kind | What | Rate | Basis | Reasoning |
|---|---|---|---|---|
| `tube_in` | The transfer cart brings a bare tube in and lowers it into the holder's saddles. | 0.001 | ours | A cart move with a seating step into saddles is a simple fixed-path operation, but a mis-seat needs a person, so about one in a thousand. |
| `clamp` | The holder closes on the tube. | 0.0005 | ours | A pneumatic or electric clamp on a known tube is very repeatable, with sensor or jam faults rarer than a gripper pick. |
| `grip` | An arm's gripper takes a part from a rack, pallet or nest. | 0.001 | ours | A known part at a known place with a purpose-built gripper should rarely fail, and about one in a thousand is a judgment; Brightpick reports over 99.7 percent pick success for mixed unknown warehouse items, which is a harder task and not defined as stopping the line, so it was not used. |
| `set_down` | An arm sets a held part in its place and lets go. | 0.001 | ours | Placing a rail or a 32 kg module onto the tube or rails needs a tolerance fit and a release, so it is judged as hard as a grip. |
| `bolt_run` | A nutrunner runs one strap bolt to torque. | 0.003 | ours | Fastening NOK from cross-threading, a wrong seat or a torque fault is judged at a few in a thousand; a research-lab screw pick-and-fasten with a general tool failed 3 of 27 times in the Team O2AS World Robot Summit 2018 paper, which is a far less controlled task and was not used. |
| `locate` | The tapered nose pulls a module's hole into line by contact. | 0.003 | ours | Contact alignment depends on module and hole tolerance and can jam, so it is judged a few in a thousand. |
| `pin_set` | One lock pin is inserted into a hole. | 0.003 | ours | Peg-in-hole insertion with a tapered lead-in into a hole the locate step has just lined up works most of the time, and a stuck or missed pin is judged at a few in a thousand. |
| `swage` | One pin is swaged or collared. | 0.002 | ours | A swage tool on a seated pin is a well-controlled process, and tool or collar faults are judged at about two in a thousand. |
| `tool_change` | An automatic tool change on a robot flange. | 0.001 | ours | Vendor tool changers are rated for hundreds of thousands of cycles, but misalignment or air and signal faults still happen, so about one in a thousand. |
| `unclamp` | The holder releases the finished bay. | 0.0005 | ours | The same device as clamp, with a small added risk of sticking on the finished bay. |
| `bay_out` | The cart lifts the bay and takes it out. | 0.001 | ours | Same kind of cart operation as tube_in, with a heavier load. |
| `arm_move` | One planned, collision-free move of an arm between poses, as a protective stop or fault risk. | 0.0001 | ours | Maker MTBF claims of 40,000 to 100,000 hours (The Robot Report, 2019) cover hardware failure only and would give about 1e-7 per move at a few hundred moves per hour, so we set 1e-4, which is about 1.7 percent per bay at 170 moves, to allow for protective stops and faults that a person must clear. |
| `hand_over` | A part passed directly from one arm to another. | 0.005 | ours | Arm-to-arm transfer needs two arms to agree on a part at the same time, so it is judged riskier than a grip, and it is unused today. |

**The rule that maps a plan's steps to kinds.** Every step of a plan maps to one or more
kinds by its `what`, its first word or one of the phrases below. A step that maps to none
is an error, and so is a kind that `rates.json` does not hold.

| Step | Kinds |
|---|---|
| `tube in` | `tube_in` |
| `clamp` | `clamp` |
| `take ...` | `grip`, `arm_move` |
| `set ...` | `set_down`, `arm_move` |
| `bolt ...` | `bolt_run`, `arm_move` |
| `locate ...` | `locate`, `arm_move` |
| `pin ...` | `pin_set`, `arm_move` |
| `swage ...` | `swage`, `arm_move` |
| `tool change` | `tool_change`, `arm_move` |
| `back to wait` | `arm_move` |
| `release` | `unclamp` |
| `bay out` | `bay_out` |
| `move the tube to stage ...` | `unclamp`, `tube_in`, `clamp` |

A staged layout's transfer (`docs/two-flat-racks-cell.md`) counts as the holder letting the
tube go, the infeed and outfeed units moving and seating it as a cart does at `tube_in`, and
the holder gripping it again. No kind was added to `scores/rates.json`, so the other scores
are unchanged.

A kind that no step maps to (`hand_over` today) is counted 0.

`failures_per_bay` compares layouts under the same rates. It predicts nothing: the rates
are our judgment and not measurements, and it says how a layout would rank if they held,
not how often a real cell would stop.

## What would make it measured

Three measures are modelled or not known because the flat rack cell is kinematic. Each
would need the cell to do the following in physics first.

| Measure | What the cell would have to do before the figure is measured |
|---|---|
| `failures_per_bay` | Run the flat rack cell's committed plan in MuJoCo with the parts held and sensed. |
| `failures_per_bay` | Do that over many randomized bays (tube, rails, modules and pins each set down with their own error). |
| `failures_per_bay` | Count the stops and the bays that need a person, by kind of operation. |
| `failures_per_bay` | Replace each rate of `scores/rates.json` by the count of stops over the count of operations of its kind. |
| `placement_mm` | Hold and sense the modules, as the stand-in cell does with its two cameras. |
| `placement_mm` | Place each module with that sensing, and measure its error against its slot at the gate. |
| `input_tolerance` | Feed the parts (tubes, rails, modules, pins) with a randomized error, larger in each series. |
| `input_tolerance` | Find the largest error of the infed parts that is still placed in tolerance. |

## Versions and pages

`versions/versions.json` is the registry of versions, oldest first. It holds `about`, the
`directions` (each with `id`, `title`, `description`, `limits`) and the `versions`. A
version has:

- `id`, `date`, `commit`, `title`, `direction`;
- `status`: `chosen`, `superseded` or `candidate`;
- `changed` and `why`, in plain sentences;
- `recording`, the file under `docs/evolution/`, and `recording_source`, which holds the
  path of the source of the recording and the commit it was recorded at (`added_at`);
- `competed_with` for a candidate: the version it was weighed against;
- `rehashed`, only where a version's files changed without changing the version: one entry
  per change with `date`, `commit`, `why` and the `layout_sha256_was` (and
  `plan_sha256_was`, where it records a plan) it replaced, written by `pages.py rehash`;
- either `layout`, `plan` (each with `path` and `sha256`) and `scores` (the path of
  `scores/<name>.json`), or `figures`, each with `name`, `value`, `unit`, `known` and
  `source`, for a version that has no layout file or plan of its own.

The chosen layout is the layout of the newest version whose status is `chosen`: that is
the one place it is kept, read by `ChosenLayout` (`src/layout.h`) for `check_layout --list`,
`plan_replay_test`, `layout_check_test`, `layout_model_test` and `versions_test`, and by
`pages.py` for the pages. A layout file has no `chosen` field (the loader refuses one), so
choosing a later layout changes no registered file: `add.sh` with `--status chosen`
supersedes the direction's chosen version.

A registered version does not change. `//:versions_test` fails when the sha256 of a
recorded layout or plan changes, when a direction has more than one chosen version, and
when the chosen layout is not a layout that loads. A change to the chosen layout or its
plan is therefore a new layout file and a new version, added with

`tools/evolution/add.sh NAME "TITLE" "CHANGED" "WHY" [--status chosen|candidate] [--commit HASH|this] [--competed-with vNN]`

which writes `scores/<name>.json`, records the bay of the plan, appends the version to the
registry, burns the caption into the recording and writes the pages.

The pages of the directions (`docs/directions/`), their charts and the README's results are
written from the registry and the scorecards by `python3 tools/evolution/pages.py write`.
Step 3 of `tools/architecture/ci.sh` runs `pages.py check`, which fails when a committed
page differs from what the program writes. `//:scorecard_test` fails when a committed
`scores/<name>.json` is not what `score_layout` computes now. The decision is
`docs/adr/0023-scores-and-versions-live-beside-the-layouts.md`.

The directions:

- [`docs/directions/one-40ft-flat-rack.md`](directions/one-40ft-flat-rack.md)
- [`docs/directions/stand-in-cell.md`](directions/stand-in-cell.md)
- [`docs/directions/two-20ft-flat-racks.md`](directions/two-20ft-flat-racks.md)

### Changing the chosen layout

Until run 018, `chosen: true` was a field of the layout file, so choosing another layout
would have changed the file, and so the sha256, of a registered version. Run 018 moved
the flag to the registry (above). Taking the line out of the eight layout files changed
the bytes of the three registered ones, `flat_rack_1_pin_arm` (v07, which records no
plan), `flat_rack_2_pin_arms` (v08) and `flat_rack_3_pin_arms` (v09). The two plans were made again (planning repeats itself, so
they came out the same but for `layout_sha256`), the two scorecards written again (the same
but for the two hashes), and their new hashes recorded with
`python3 tools/evolution/pages.py rehash vNN --why "..."`, which keeps the old ones and
the reason under `rehashed`. `rehash` is for a change that leaves the version what it was.
A change of the cell is a new layout file and a new version.
