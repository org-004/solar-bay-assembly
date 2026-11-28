// Bays planned back to back: the pieces that turn the plan of one bay
// (src/layout_plan.h) into a plan of several, so that the cycle is measured
// over more than the first bay.
//
// PlanLayout plans each bay alone, from t = 0 and in the order of the
// checker's steps, as it plans one; what differs from bay to bay is said
// here, and so is how the bays are put on one clock:
//   - numbering: with N steps in a bay (the checker's report.steps, numbered
//     1 .. N), step n of bay b has the number n + b N in the joint plan, in
//     its segments, its events and its steps (StepNumberInBay, BayOfStep,
//     StepOfBay);
//   - takes: bay b takes pallet points 9b .. 9b+8 (9 is ModulesPerBay, the
//     bay's module_take steps), in the layout's order of modules; every
//     other work point is bay 0's (PointIndexInBay, StepsOfBays);
//   - what is built at a move of bay b is bay 0's at the same step, with
//     9b fewer modules left on the pallets (BuiltInBay);
//   - when bay b+1's tube comes in (NextTubeStart): not before the holder is
//     empty (HolderEmptyAt, the end of the first leg of bay_out.path, which
//     lifts the bay out of the saddles); where the tubes' and the bays' ports
//     share their zone or unit (WaysShareZone), not before bay b's bay_out
//     has ended; otherwise at the first instant, from the holder's being
//     empty in steps of sample_s, from which the incoming tube and the
//     outgoing bay stay clearance_m apart at every sample (TubePassesBay);
//   - R10: in bay b > 0 a step that touches neither the tube nor the bay
//     (TouchesTheBay) may start before the clamp ends, from the tube_in's
//     start; its planner knows where the bay lies on the joint clock
//     (BayClock) and holds it apart from the tube coming in;
//   - JoinBays puts the bays' plans on one clock, labels their events, steps
//     and phases with the bay, and keeps every arm's row without a gap;
//   - CycleFiguresOf gives the cycle of the joint plan.
// The verifier (src/layout_replay.cc) plays and checks a plan of several
// bays with the same pieces: StepsOfBays for its steps, BetweenBays for R2.

#ifndef SRC_LAYOUT_BAYS_H_
#define SRC_LAYOUT_BAYS_H_

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {

// ------------------------------------------------------------ numbering

// The modules one bay takes from the pallets: its module_take steps.
int ModulesPerBay(const CheckReport& report);
// Step `number` (1 .. N) of bay `bay` in the joint plan, and back. A number
// of 0 (no step) stays 0.
int StepNumberInBay(int number, int bay, int steps_per_bay);
int BayOfStep(int number, int steps_per_bay);
int StepOfBay(int number, int steps_per_bay);
// The work point a step of the bay works at: a module_take's index moved on
// by the modules the earlier bays took; every other step's own.
int PointIndexInBay(const BayStep& step, int bay, int modules_per_bay);

// A step of one of several bays: the checker's step with its number in the
// joint plan and, for a module_take, its pallet point.
struct StepInBay {
  int bay = 0;
  BayStep step;
};
// Every step of `bays` bays, bay by bay, each bay in the checker's order.
std::vector<StepInBay> StepsOfBays(const CheckReport& report, int bays);

// The bay as built at a move of bay `bay`, from bay 0's at the same step:
// the earlier bays' modules are gone from the pallets. (LayoutScene::
// AtWorkPoint of a later bay's pallet point would count all nine modules
// set; it is not used for that.)
Built BuiltInBay(Built bay0, int bay, int modules_per_bay);

// ---------------------------------------------------------------- the ways

// A point at `u` (0 .. 1) of a polyline of offsets, at constant speed: where
// the tube (coming in) or the bay (going out) is along its way.
Eigen::Vector3d WayOffsetAt(const std::vector<Vec3>& path, double u);
// The fraction of a way of `span` seconds that started at t0, at time t.
double WayFraction(double t0, double span, double t);

// Whether the ports that hand over way:tube_in and way:bay_out share their
// zone or unit: one port, one names the other in shares_zone_with, or both
// hand over the same item. A layout without both ports counts as sharing.
bool WaysShareZone(const Layout& layout);

// When the holder is empty: the end of the first leg of bay_out.path (the
// lift out of the saddles), at the way's constant speed.
double HolderEmptyAt(const Layout& layout, const BayEvent& bay_out);

// The test of a tube that starts its tube_in at `tube_start` while the bay
// travels its bay_out: at every sample, tube_start + k sample_s and the plan
// clock's k sample_s, while the bay is still on its way (before
// bay_out.t1), the tube at least clearance_m
// from the bay's rails and modules. The model has one tube, so the bay's
// own tube is not measured: its rails' closed straps go round it and are
// nearer than it to anything that comes at it across its axis, as both ways
// of the layouts here do. `d` is the scene's data with every geom at its
// place (MakeData; mjData comes through src/layout_plan.h); its geoms are
// put back. In staged work the scene is the first stage's, where the tube
// comes in, and the bay goes out from the last stage's place.
// With `far` above clearance_m every sample is measured, out to `far`, for
// the least distance (the margins' tube_bay rows); otherwise the test ends
// at the first sample that is not clear.
struct WayPassing {
  bool clear = true;
  double t = 0.0;          // The first sample that is not clear.
  Nearest nearest;         // Its pair, measured out to clearance_m.
  double least_m = kSceneFar;  // The least distance of the samples measured.
  double least_t = 0.0;
};
WayPassing TubePassesBay(const LayoutScene& scene, mjData* d,
                         const BayEvent& bay_out, double tube_start,
                         double sample_s, double clearance_m, double far = 0.0);

// When the next bay's tube_in starts, on the clock of `bay_out` (R2): the
// end of bay_out where the ways share their zone; otherwise the first of
// HolderEmptyAt + k sample_s that TubePassesBay finds clear, and at the
// latest the end of bay_out.
double NextTubeStart(const LayoutScene& scene, const BayEvent& bay_out,
                     double sample_s, double clearance_m);

// ------------------------------------------------------------ joining

// When bay b+1's tube_in starts, given bay b's bay_out on the joint clock.
using NextTubeStartFn = std::function<double(const BayEvent& bay_out)>;

// One plan of the bays' plans, each scheduled from t = 0 as one bay is,
// steps_per_bay steps each. Bay 0 is as it was. Bay b > 0: its tube_in and
// clamp start at next_start(bay b-1's bay_out); its arms, steps, release
// and bay_out are put off by as much, and further, if need be, so that no
// arm starts before bay b-1's bay_out has ended. Numbers are moved on by
// b N; events, steps and phases carry their bay. Each arm's row stays
// without a gap: bay b's first wait is cut to start where the arm's row of
// bay b-1 ends, or that row's last wait is drawn out to where it starts;
// where an arm's first block of bay b starts before its row of bay b-1 ends
// (next_bay_fill), that row's trailing waits end where the block starts.
// The joint plan is complete when every bay's is. A wait's reason names
// steps and instants on its own bay's clock. One plan is returned as it is.
Plan JoinBays(const std::vector<Plan>& bays, int steps_per_bay,
              const NextTubeStartFn& next_start);

// Where bay b's arms start on the joint clock: its tube's start, put off so
// that no arm starts (at the clamp's end, `clamp_end` after the tube's
// start) before the bay before is out.
double ArmShift(double tube_start, double clamp_end, const BayEvent& last_out);

// The cycle of a plan: one_bay_s, bay 0 from the start of its tube_in to
// the end of its bay_out; cycle_s, from bay 0's tube_in to the last bay's
// bay_out, over the number of bays; interval_s, from the end of the last
// but one bay's bay_out to the end of the last one's. A plan of one bay has
// cycle_s = one_bay_s and interval_s 0.
struct CycleFigures {
  int bays = 0;
  double one_bay_s = 0.0;
  double cycle_s = 0.0;
  double interval_s = 0.0;
};
CycleFigures CycleFiguresOf(const Plan& plan);

// ------------------------------------------------------------ early steps

// R10: a step that touches neither the tube nor the bay (a take from a stock
// or the pallet, a magazine take, a tool change) may start in bay b > 0
// before the bay's clamp has ended: from its tube_in's start, and not before
// bay b-1 is out, so that of the ways only the tube coming in moves while it
// runs. Every other step starts when the clamp has ended. Bay 0 starts as
// it always has, every arm after the clamp.
bool TouchesTheBay(const std::string& kind);

// "first_bay_early" (optional, flat rack bases only; run 059; Layout holds
// its basis): bay 0's steps that touch neither the tube nor the bay (a take
// from a stock or the pallet, a magazine take, a tool change; TouchesTheBay)
// may start while the first tube comes in, from its tube_in's start, as R10
// lets a later bay's do (src/layout_plan.h, "Between bay b and bay b+1"):
// on the rail magazine layout the module arm fills its magazine at the rail
// rack while the first tube comes in, and waits with it for the clamp. The
// schedule and the verifier hold such an arm apart from the tube on its way.
// Without it bay 0 starts every arm after its clamp, so that every committed
// plan replays as it was.

// "next_bay_fill" (optional, flat rack bases only; run 061; Layout holds the
// arm it names): in a later bay b+1 that arm's steps that touch neither the
// tube nor the bay, before its first step that does, may start as soon as
// the arm has ended its last block of bay b (on the rail magazine layout,
// its move to its wait pose after setting bay b's last module), while bay
// b's other arms still work, bay b goes out and bay b+1's tube comes in;
// the arm then waits where its take's retreat left it for bay b+1's clamp.
// Its moves there are planned with the whole of bay b built (every rail and
// module), and JoinBays ends its trailing waits of bay b where its first
// block of bay b+1 begins. Bay b+1's scheduler does not see bay b's arms;
// the verifier samples the joined plan, so every arm is held apart from it,
// and ClearOut and TubeComingIn hold it apart from the bay going out and
// the tube coming in. Without the field no arm starts a step of bay b+1
// but a wait before bay b's bay_out ends (BetweenBays), as before.

// Where a bay lies on the joint clock, for its planner (PlanBays). Times
// but `shift` are on the bay's own clock, whose 0 is its arms' start.
struct BayClock {
  double shift = 0.0;       // The joint clock's time of the bay's own 0.
  double tube_start = 0.0;  // Its tube_in's start (start - shift, at most 0).
  bool early = false;       // Steps that do not touch the bay may start early,
  double early_from = 0.0;  // from here.
  int early_arm = -1;           // next_bay_fill: this arm's early steps may start
  double early_arm_from = 0.0;  // from here (on the bay's clock, perhaps before 0).
};

// The least distance from `moving` (posed in d) to the tube at `u` (0 .. 1)
// of its tube_in way, out to `far`; the tube's geoms are put back.
Nearest AgainstTheTubeComingIn(const LayoutScene& scene, mjData* d,
                               const std::vector<MovingGeom>& moving, double u, double far);

// ------------------------------------------------------------ checking

// R2 between the bays of a plan, for the verifier (src/layout_replay.cc),
// as faults; none for one bay. For each bay b > 0: its tube_in starts when
// the holder is empty; where the ways share their zone, not before bay
// b-1's bay_out ends; where they do not, TubePassesBay holds (at
// clearance_m less tolerance_m); and no arm starts a segment of bay b's
// steps but a wait before bay b's clamp ends, unless the step touches
// neither the tube nor the bay (R10, TouchesTheBay). `report` is the
// checker's, of one bay. That every arm stands at its
// wait pose with its first tool during each tube_in and bay_out is the
// verifier's own check of each bay's events.
std::vector<std::string> BetweenBays(const LayoutScene& scene, const Plan& plan,
                                     const CheckReport& report, double tolerance_s,
                                     double tolerance_m);

// ------------------------------------------------------------ planning

// Plans one bay of the layout (src/layout_plan.cc's planner, without the
// verifier) with the checker's report and where the bay lies on the joint
// clock (BayClock).
using PlanOneBayFn =
    std::function<PlanResult(int bay, const CheckReport& report, const BayClock& clock)>;

// Checks the layout once, plans options.bays bays one after another with
// `plan_bay`, each knowing where it starts (NextTubeStart on the layout's
// scene after the bay before, ArmShift), and joins them. Rows'
// steps are numbered in the joint plan and a bay's faults are named with
// it; the first bay that is not complete ends planning, and its plan is
// returned. Neither the wall time nor the verifier is this function's.
PlanResult PlanBays(const Layout& layout, const ModelPaths& paths,
                    const PlanOptions& options, const PlanOneBayFn& plan_bay);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_BAYS_H_
