# 0017 The code is ten groups with one direction of dependency, held by a check on the include lines

Status: accepted
Date: 2026-11-25
Decided by: run 013-design-principles-and-r2 (spec 013, R12)

**Context.** `src/` (99 files) and `tests/` (41) are flat, and nothing written says
which piece may use which. The code is two families that hardly meet: the stand-in
cell, which is to be replaced (ADR 0013), and the new cell on the real parts. At the
base commit `940a01b` no file of the stand-in includes the new cell, and the new cell
reaches the older code only through general parts (`src/joint_move.h`, `src/rrt.h`,
`src/path_timing.h`, `src/viewer_lib.h`). The arm's motion stack (`src/arm.h`,
`src/ik.h`, `src/collision.h`, `src/motion_planner.h`, `src/arm_follower.h`) is what
the new cell is to take over next, and one line ties it to the stand-in:
`src/collision.cc` includes `src/pallet.h`. Modules are worth dividing by what each
hides ([S1] of `docs/architecture/research.md`), dependencies between them must have
no cycle ([S2]), and a layering stays true only when a machine refuses what breaks it,
with today's breaches listed and unable to grow ([S26], [S27]).

**Decision.** Every file under `src/` and `tests/` is in exactly one of ten groups,
listed lowest first in `tools/architecture/deps.json` with what each may include:

| Group | Holds | May include |
|---|---|---|
| `base` | MuJoCo handles, portable random draws, velocity profiles, a timed joint move | nothing |
| `planning` | RRT-Connect, OMPL as second search, path timing, toy worlds | `base` |
| `hardware` | the bay's parts and generated model, the container, the flat rack | `base` |
| `layout` | the new cell: layouts as data, arm catalogue, model, checks, scene, plan, replay | `base`, `planning`, `hardware` |
| `perception` | camera model, depth image, plane and rectangle fits, the two finders | `base` |
| `arm` | the one arm's motion stack: arm, inverse kinematics, collision, planner, follower | `base`, `planning` |
| `standin` | the stand-in cell: pallet, conveyor, suction, cell, cycle, checks, score, runner | `base`, `planning`, `perception`, `arm` |
| `viewer` | the plain viewer and its helpers | `base` |
| `programs` | the stand-in's programs with a `main` | `base`, `planning`, `perception`, `arm`, `standin`, `viewer` |
| `layout_programs` | the new cell's programs with a `main` | `base`, `planning`, `hardware`, `layout`, `viewer` |

A test is at `tests/<piece>_test.cc` and sits in the group of the piece it tests, or of
the highest piece it includes when it tests several. A program that needs only one
library group sits in that group (`src/print_rrt.cc` in `planning`). The groups
`standin`, `perception` and `programs` are frozen (ADR 0020). The groups say what may
include what; they move no file (ADR 0018) and rename no namespace (ADR 0019). A new
file is added to a group's list in the commit that adds it. A new group, or a group
allowed to include one more, needs a new decision record.

**Alternatives.**
- Two groups, stand-in and new cell. Lost: it would not say that `planning` knows no
  MuJoCo, nor that the arm's stack must come free of the stand-in.
- The arm's stack inside `standin`, as the first run of this spec had it. Lost: the
  one include that ties it there would be allowed, not counted.
- A group for each header. Lost: that is the `deps` lists of `BUILD.bazel`, which P5
  already holds, and it states no direction.
- The layering held by Bazel packages. Not now: ADR 0018.

**Consequences.** A wrong include fails `measure.sh --check` in a second, with the
file and the rule named. Five pairs of a file and a header go against the direction
today and are listed as departures: four are in frozen files and stay until the
stand-in goes, one (`src/collision.cc`) is for the backlog. The check reads include
lines: a forward declaration of another group's name passes it, and review holds
that. `deps.json` must be edited when a file is added, which a failing check says.

**Check.** P1 of `docs/architecture/principles.md`: `measure.sh` lines `P1`,
`P1.cycles`, `P1.namespace`; `python3 tools/architecture/deps.py --graph` prints the
table of who includes whom.

**Migration.** The backlog items that serve P1 in `docs/architecture/backlog.md`.
