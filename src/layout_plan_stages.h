// Staged work in the plan: the pieces that let PlanLayout and VerifyPlan
// (src/layout_plan.h) plan and replay a bay built in stages (the layout's
// optional block "staging", src/layout.h, and the checker's "Stages" in
// src/layout_check.h). A layout without staging has one stage and nothing
// here changes its plan: every function below gives the plain answer for
// it, and the plan file has no field of stages.
//
// The rules (R1), which the planner follows and the verifier holds:
//   - places: at stage s the tube, and the part of the bay built so far,
//     stand at the stage's place (StageLayout(layout, s) of
//     src/layout_stages.h, the tube moved by its tube_offset_m along +Y);
//     only the stage's rails and modules are set there. Each move of a
//     step of stage s is planned, and verified edge by edge, in the scene of
//     StageLayout(layout, s) (StageScenes). The schedule's samples measure
//     in the first stage's scene with the tube, rails and modules moved by
//     the stage's offset from the first stage's place (WorldAt::
//     stage_offset): the stages' models differ in nothing else.
//   - the bay at a move: the checker's (LayoutScene::AtWorkPoint) kept to
//     the stage by BuiltInStage; for an arm that follows another, the
//     leaders' parts are the earlier stages' and this stage's rails (and,
//     from the modules on, modules), never a later stage's (LeadersParts).
//   - transfers: between stage s and s+1 the tube moves along +Y by the
//     difference of their offsets in staging.transfer_s, at constant speed
//     (an event "transfer", from_stage s+1 and to_stage s+2, counted from
//     1, numbered as the checker's step "move the tube to stage ..."). It
//     starts when every arm has ended every step of the stages up to s,
//     each standing at its wait pose with its first tool and nothing held
//     (every stage of the checker ends each arm there), and no arm starts a
//     segment of a step of a later stage before it ends. Through it every
//     arm waits at its wait pose, in a wait whose reason names the
//     transfer, where the checker's ways rows ("stage <a> to <b>") hold
//     it; an arm that is not there is a fault, and the schedule's samples
//     measure it against the tube where it is on its way.
//   - the ways: tube_in brings the tube to the first stage's place; release
//     and bay_out are after the last transfer and the bay leaves from the
//     last stage's place. Between bays the tube coming in is held apart from
//     the bay going out (src/layout_bays.h, TubePassesBay) with each at its
//     place.
//   - the first module take of each stage starts after the stage's last
//     strap bolt (the plain rule within the stage).
//   - the magazine (the planner's finding of run 022): in staged work an
//     arm takes its pin magazine before its stage's first module set and
//     then parks at its wait pose (a move of its next step, as the parking
//     of src/layout_plan.h); every module set of the stage starts only when
//     that park has ended. The rule of just in time is not applied to the
//     magazine take, whose moves are planned with the stage's rails and no
//     module of the stage: the take's approach pose is nearer a set module
//     0 than the clearance allows.
//   - a move between the holes of module m (an arm that follows another)
//     that is not found in its world, which holds the modules that may be
//     set after m while m's holes are worked (the checker's next module and
//     the leaders' parts), is planned once more without them (run 022: in
//     stages 2 and 3 a pin arm's swing from hole 2 to hole 3 passes where
//     module m+1 would be). The set of each module that world lacks then
//     starts only when that move has ended; the verifier holds it as it
//     holds every move's world (what is there while it runs).
//   - phases: per bay "tube in", "clamp", "rails", "modules", "end" and
//     "transfer". For each stage, from its start (the clamp's end, or its
//     transfer's) to its end (the next transfer's start, or the bay's end):
//     "rails" to its last strap bolt, "modules" to its last step of phase
//     modules, "end" the rest; each is the sum over the stages, and
//     "transfer" the transfers' sum.

#ifndef SRC_LAYOUT_PLAN_STAGES_H_
#define SRC_LAYOUT_PLAN_STAGES_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {

// The layout's stages: StagesOf's count, 1 without staging.
int StageCount(const Layout& layout);
// The index (into StagesOf) of the stage a step of one bay belongs to, by
// its BayStep::stage; 0 without staging. A transfer's is the stage it
// moves the tube to.
int StageIndexOf(const Layout& layout, const BayStep& step);
// The offset of stage s's place from the first stage's, along +Y.
Eigen::Vector3d StageOffset(const Layout& layout, int s);

// The layout the scenes of a plan are built from: StageLayout(layout, s)
// for a staged layout, the layout itself without staging.
Layout LayoutOfStage(const Layout& layout, int s);
// One scene for each stage (LayoutOfStage); empty and *error set on a fault.
std::vector<std::unique_ptr<LayoutScene>> StageScenes(const Layout& layout,
                                                      const ModelPaths& paths,
                                                      double joint_margin_deg,
                                                      std::string* error);

// For an arm that follows another (bolts, pins): every part the leading
// arms may set before its move ends, by the move's phase: "rails" adds all
// rails, "modules" and "end" all rails and all nine modules, anything else
// nothing. In staged work (`stage` given) "all" is the earlier stages' and
// this stage's (above).
void LeadersParts(const StageScope* stage, const std::string& phase, Built* b);

// Where the tube stands at time t of a plan, for bay `bay`: the stage
// (0-based), its offset from the first stage's place, and whether a
// transfer is under way (the offset then between the two stages' at
// constant speed). Without transfers: stage 0, zero, false.
struct StagePlace {
  int stage = 0;
  Eigen::Vector3d offset = Eigen::Vector3d::Zero();
  bool moving = false;
};
StagePlace StagePlaceAt(const Layout& layout, const std::vector<BayEvent>& events, int bay,
                        double t);

// The phases of bay `bay` of a staged plan (above), with the model's
// figures of `report` (one bay's checker report); `times` are the steps'
// times by their number in the plan, `bay_end` the end of the bay.
std::vector<PhaseTime> StagedPhases(const Layout& layout, const CheckReport& report, int bay,
                                    const std::map<int, StepTime>& times,
                                    const std::vector<BayEvent>& events, double bay_end);

// Cuts every wait of the rows that overlaps a transfer into the piece
// before it, the piece during it (whose reason names the transfer) and the
// piece after it. Nothing changes without transfers.
void WaitsThroughTransfers(const Layout& layout, const std::vector<BayEvent>& events,
                           std::vector<ArmPlan>* rows);

// The verifier's checks of the rules above for a plan of the layout, as
// faults: the transfers (their number, steps, stages, durations and order,
// the arms at their wait poses through them, no step across them), the
// first take of each stage after its bolts and the magazine before the
// stage's module sets. `scene` is the plan's scene (for the wait poses),
// `report` one bay's checker report. None for a layout without staging but
// a transfer event, which is a fault there.
std::vector<std::string> StagedFaults(const LayoutScene& scene, const CheckReport& report,
                                      const Plan& plan, const VerifyOptions& options);

// VerifyPlan with the checker's report of the layout already made
// (CheckLayout(layout, paths)): for tests that verify many copies of one
// plan. Defined in src/layout_replay.cc.
VerifyReport VerifyPlanWithReport(const Layout& layout, const ModelPaths& paths,
                                  const Plan& plan, const std::string& layout_sha256,
                                  const CheckReport& report,
                                  const VerifyOptions& options = {});

}  // namespace cell::layout

#endif  // SRC_LAYOUT_PLAN_STAGES_H_
