# 0012 The new cell is checked and planned kinematically before physics steps it

Status: accepted
Date: 2026-11-24
Decided by: the owner, 2026-11-24; applied in 68a1199 (run 011) and 940a01b (run 012)

**Context.** The cell that is to build bays from the real parts does not exist except as
a design. The owner's line on 2026-11-24, after run 011, is "A working system comes
first, then iteration" (the project brief). The first cell's limits, set the same day,
end with "The work is kinematic only" (`docs/flat-rack-cell.md:32-40`). What a kinematic
check cannot see is plain: the time model, which moves each arm on a straight line in
joint space, came out low against the repository's own planner by 1.731 to 3.488 times
on six moves (`docs/layout.md`, "The time model against the planner"). The stand-in
cell, by contrast, is a physics simulation with controllers, a camera and a gate (ADR
0013).

**Decision.** The new cell's arms, tools, parts and envelopes are checked at poses
(`src/layout_check.h`, run by `//:check_layout`) and along planned motion
(`src/layout_plan.h`, run by `//:plan_layout`), and nothing steps the physics
(`src/layout_plan.h:13`, `docs/flat-rack-cell.md:7-9`). The layout's model is built
without actuators or keyframes, "for looking and measuring" (`src/layout_model.cc`), and
its viewer does not step physics. Controllers, sensing, wind and the base's structure
are outside the design (`docs/flat-rack-cell.md:633`). The one place the new parts are
put in physics is the locating move of the nose on one module, which the owner called
welcome and never required (the project brief): `tests/locate_physics_test.cc`.

**Alternatives.** The repository records the weighing only inside the kinematic work: a
time model against the planner (`docs/layout.md`, "The time model against the planner"),
and moves planned one arm at a time, the others at wait, not jointly
(`docs/flat-rack-cell.md`, "The method and its limits"). `docs/layout.md`, "What was not
considered", says that paths between poses were not checked in run 011 "as the brief
scopes it", and run 012 then planned them. It records no alternative that would have put
physics first, and no reason for the order beyond the owner's line above.

**Consequences.** A layout is judged in seconds and by arithmetic, so many layouts can
be compared and a plan can be committed and replayed (`tests/plan_replay_test.cc`)
without a simulation. The price is what the design does not show: contact, grip,
controller tracking and sensing are unchecked, so a margin in the design is a margin of
geometry only, and the planned times are a plan, not a measurement. The new cell has no
gate; the gate stays on the stand-in (ADR 0020). When physics comes to the new cell,
this record is superseded by one that says so.

**Check.** `grep -n 'Kinematic only' src/layout_plan.h` prints line 13. The captures of
`plan_layout` in `tools/architecture/programs.txt` (`--margins` and `--summary` for each
committed plan, P2, ADR 0021) print what the planner makes without a physics step, and
`tools/architecture/behavior.sh compare` prints `same` while it does.

**Migration.** None: the code already follows this.