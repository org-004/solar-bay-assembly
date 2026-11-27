// The critical path of a committed plan (run 055): for each bay, the chain
// of steps that ends at the bay's release, walked back from the release,
// and for each link why it starts when it does. Worked out from the plan's
// own times and the scheduler's rules (src/layout_plan.h, "The schedule";
// src/layout_bays.h), never from the reasons the scheduler wrote on its
// waits: it plans nothing and changes nothing.
//
// A block is what the scheduler places at once: the run of an arm's
// segments of one step between two waits (a step whose arm parks first has
// two, the parking move and the rest). A block of step n of arm A, starting
// at s, may start no earlier than each of
//   arm     the end of A's block before it (A's previous step);
//   need    what step n needs, by the scheduler's rules: a strap bolt its
//           rail's set (the end of the set's work), a hole step its
//           module's set (the end of its work) and the hole step before it
//           in the module's order of holes (its end), the first module take
//           every strap bolt (or, "after_their_rails", the bolts of its two
//           rails); a tool change or magazine take of phase "modules", just
//           in time, what A's next hole step needs;
//   clamp   for a step that touches the tube or the bay (TouchesTheBay) and
//           for a parking move, the end of the bay's clamp (in a later bay
//           no earlier than the end of the bay before's bay_out, ArmShift);
//   early   for the first block of an arm in a later bay, the start of the
//           bay's tube_in or the end of the bay before's bay_out, whichever
//           is later (R10).
// The latest of these is the block's ready time, and the one that is
// latest is the link before it in the chain (ties: a need, then the clamp
// or the boundary, then the arm). A block that starts after its ready time
// waited; the wait is named:
//   way     an arm in its way: with the block put k 0.25 s earlier (the
//           scheduler's step of delay), measured at the scheduler's
//           instants (every sample_s of the block and of the plan clock)
//           through InstantDistances, an other arm comes within
//           clearance_m + plan_pad_m of it; the way is the k 0.25 s back
//           from the start, without a break, at which one does, and the
//           arm named is the first found at 0.25 s;
//   hold    the rest of the wait: the torque bound's order (ScheduleSlowed
//           places a slowed plan in its unslowed order, and a block is held
//           to its unslowed start plus what slowed moves before it added),
//           which the plan's times do not hold, so it is what is left.
// The events: the release starts at the end of the bay's last block of
// phase "modules"; the clamp follows the tube_in; a later bay's tube_in
// starts once the holder is empty (HolderEmptyAt of the bay before's
// bay_out; its end where the ways share their zone), later by a wait while
// the tube may not pass the bay going out; the bay before's bay_out starts
// at its release's end, later by a wait while an arm is in the bay's way.
// The walk stops at the bay before's release (bay 0: at its tube_in), so a
// chain runs from that release's end to its own release's end: in a later
// bay, the time between the two bay_outs' ends where bay_out follows the
// release at once.
//
// Every link covers the time from the end of the wait before it to the
// moment its successor may start (a set's end of work for a need of
// work_end, else the block's end), and the links and the waits join with
// no gap: the chain's seconds are its span. Each link's seconds are split
// by the kind of its segments: motion (move, approach, retreat), work, and,
// of a move played slower by the torque bound (time_scale > 1), the
// slowdown planned_s (1 - 1 / time_scale); a wait inside a block (none in
// these plans) and the named waits count as wait. Staged plans are not
// read.

#ifndef SRC_LAYOUT_CRITICAL_H_
#define SRC_LAYOUT_CRITICAL_H_

#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {

// One link of a chain.
struct ChainLink {
  std::string what;        // The step's (or "release", "bay_out", ...).
  std::string arm;         // "" for an event.
  int step = 0;            // Its number in the plan.
  bool event = false;
  double start = 0.0;      // When the block (or event) starts.
  double end = 0.0;        // When its successor in the chain may start.
  double block_end = 0.0;  // When the block (or event) ends.
  // Why it starts then: "arm", "need", "clamp", "early", "tube", "holder",
  // "bay_out", "release", "modules" (the release), "root"; and the wait
  // before it, if any: "way", "hold", "way+hold" or "" (none).
  std::string cause;
  std::string wait_cause;
  double wait_s = 0.0;     // start - the end of the link before it.
  double way_s = 0.0, hold_s = 0.0;  // Its parts; way_s + hold_s = wait_s.
  std::string by;          // way: the arm (or "the tube") in its way.
  std::string text;        // The reason in words.
  // The split of [start, end].
  double motion_s = 0.0, work_s = 0.0, slowed_s = 0.0, inner_wait_s = 0.0;
};

// What one arm (or the events, arm "") spends on a chain.
struct ChainArm {
  std::string arm;
  double motion_s = 0.0, work_s = 0.0, slowed_s = 0.0, wait_s = 0.0;
  int links = 0;
};

struct CriticalChain {
  int bay = 0;
  double from_s = 0.0;     // The root's end (0 for bay 0).
  double to_s = 0.0;       // The release's end.
  std::vector<ChainLink> links;  // From the root's successor to the release.
  std::vector<ChainArm> arms;    // In order of first appearance.
  // The other arms' ends: for each arm but the last block's, the end of its
  // last block of phase "modules" in the bay and the span of the chain
  // that ends there (from the same root); latest first.
  struct Next {
    std::string arm;
    int step = 0;
    std::string what;
    double end = 0.0;
    double span_s = 0.0;
    // The last link it shares with the chain of the release, and its end in
    // this chain: a lever before it shortens both chains.
    int shared_step = 0;
    double shared_until = 0.0;
  };
  std::vector<Next> next;
};

// The chain of each bay of the plan the player plays (src/layout_plan.h,
// PlanPlayer, of `layout`'s scene and the checker's `report`). Returns an
// empty list and sets *error when the plan cannot be read so (staged work,
// a step not found).
std::vector<CriticalChain> CriticalChains(const Layout& layout, const CheckReport& report,
                                          const LayoutScene& scene, const PlanPlayer& player,
                                          std::string* error);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_CRITICAL_H_
