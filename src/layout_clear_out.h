// Bay out when the arms are clear (run 050): the layout's opt-in field
// "bay_out_when_clear" (src/layout.h) and the sampled test behind it.
//
// Without the field a bay goes out only when every arm is free: after the
// release and after every arm's last segment. On the all-KUKA cells the last
// pin arm's tool change back to its first tool (it needs that tool for the
// next bay) ends the bay some 6 s after the release has ended, though the
// tool change does not touch the bay. With the field the planner starts
// bay_out at the first instant, from the release's end in steps of
// sample_s, from which every arm that is not standing at its wait pose stays
// at least clearance_m from the bay as it travels its bay_out path; at the
// latest when every arm is free, as without the field. An arm whose motion
// would pass too close to the outgoing bay holds bay out back.
//
// What is measured, at every sample (bay_out.t0 + k sample_s and the plan
// clock's k sample_s, as TubePassesBay samples, src/layout_bays.h) while the
// bay is on its way: each arm that does not stand at its wait pose (a wait
// segment at the wait configuration; there the checker's ways rows hold it
// apart from the bay), with its tool and what it holds, against the bay's
// tube, rails and modules at their place on the way. The holder does not
// move: every planned move is already held clearance_m from it, and the
// verifier's sampled distances measure it at every instant.
//
// The verifier (src/layout_replay.cc) checks the same with the plan's own
// bay_out event for a plan whose layout has the field, at clearance_m less
// its tolerance, in place of its rule that every arm stands at its wait
// pose with its first tool through bay_out.

#ifndef SRC_LAYOUT_CLEAR_OUT_H_
#define SRC_LAYOUT_CLEAR_OUT_H_

#include <functional>
#include <string>

#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {

// Where arm `arm` is at time t: its configuration, tool and held part, and
// whether it stands at its wait pose (then it is not measured).
struct ArmInstant {
  Config config{};
  std::string tool, held;
  bool at_wait = false;
};
using ArmInstantFn = std::function<ArmInstant(int arm, double t)>;

// The test of the arms while a bay travels its bay_out from `out_t0` for
// `span` seconds, measured from out_t0 to `until` (at most out_t0 + span;
// after it every arm stands at its wait pose). `d` is the scene's data; its
// arm poses and the bay's geoms are put back to the keyframe's. With
// `first_only` the test ends at the first sample below clearance_m;
// otherwise it measures every sample, out to `far`, for the least distance.
struct ArmsPassing {
  bool clear = true;
  double t = 0.0;          // The first sample below clearance_m.
  int arm = -1;            // Its arm.
  Nearest nearest;         // Its pair.
  double least_m = kSceneFar;  // The least distance of any arm to the bay.
  double least_t = 0.0;
  int least_arm = -1;
  Nearest least;
  int samples = 0;         // Arm instants measured.
};
ArmsPassing ArmsPassBay(const LayoutScene& scene, mjData* d, double out_t0, double span,
                        double until, double sample_s, double clearance_m,
                        const ArmInstantFn& at, bool first_only, double far);

// The text of a test that found an arm too close, for faults.
std::string ArmsPassingText(const LayoutScene& scene, const ArmsPassing& p);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_CLEAR_OUT_H_
