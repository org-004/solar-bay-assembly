// What a run in physics of a layout's committed plan carried out, as a
// committed record, and the rule by which its seconds count.
//
// //:run_physics (src/physics_run.h) runs some steps of the committed plan
// of a layout in MuJoCo with real masses and position servos, and writes
// physics/<layout>.json. The record holds, for each step it ran, the step's
// number, what it is, its arm, its start and end in the plan and its planned
// seconds, and what the run measured: the largest joint tracking error, the
// largest Cartesian error of the module's frame (or, where nothing is held,
// of the tool's working point) against the plan, and the least distance in
// physics between the moving arm with its tool and part and everything
// else, measured as the checker measures (LayoutScene::LeastDistance on the
// checker's own model, the pairs by design of the step's work point), beside
// the plan's own least distance over the same instants. It also holds, for
// the whole run, the module's pose on its rails after release and settling
// against the gate (src/bay_hardware.h, kGateOffset and kGateYaw: 3 mm in
// the rails' plane and 0.15 degrees of yaw, from the rails as they lie).
//
// The rule (docs/scorecard.md, physics_share). A step counts as carried
// out in physics when
//   1. its least clearance in physics is at least the record's clearance_m
//      (the layout's margins.clearance_m), and
//   2. the module the run carried ends within the gate: its offset in the
//      rails' plane at most gate_offset_mm and its yaw at most
//      gate_yaw_deg.
// physics_share is the sum of the planned seconds of the steps that count
// over one_bay_s, the time of bay 0 of the plan from tube in to bay out
// (its bay_planned_s): the share of one bay carried out in physics, also
// for a plan of three bays, whose cycle_s is their average. The record's
// steps are bay 0's. StepCounts applies the rule;
// the record's `counted` flags are what it gave when the record was
// written, and the test checks that they still agree.
//
// The file, physics/<layout>.json (PhysicsRecordToJson writes it with fixed
// precision, so that the same record is the same text):
//   {"layout", "layout_sha256", "plan_sha256", "command",
//    "module": the bay's module index, "take": the module_take index,
//    "clearance_m", "gate_offset_mm", "gate_yaw_deg",
//    "steps": [{"number", "what", "arm", "start_s", "end_s", "planned_s",
//               "max_joint_error_mrad", "max_frame_error_mm",
//               "max_frame_error_deg", "least_clearance_m",
//               "least_clearance_pair", "least_clearance_t",
//               "plan_least_clearance_m", "counted"}],
//    "set_down": {"offset_mm", "across_mm", "along_mm", "yaw_deg",
//                 "height_mm"},
//    "counted_s"}
// Seconds to 6 decimals (as the plan holds them), distances in metres to 6,
// millimetres, milliradians and degrees to 4.

#ifndef SRC_PHYSICS_RECORD_H_
#define SRC_PHYSICS_RECORD_H_

#include <optional>
#include <string>
#include <vector>

#include "src/layout_plan.h"

namespace cell::layout {

struct PhysicsStep {
  int number = 0;               // BayStep::number, as in the plan's steps.
  std::string what;             // The plan's StepTime::what.
  std::string arm;              // The plan's StepTime::arm.
  double start_s = 0.0;         // The plan's StepTime::start and end.
  double end_s = 0.0;
  double planned_s = 0.0;       // end_s - start_s.
  double max_joint_error_mrad = 0.0;
  double max_frame_error_mm = 0.0;
  double max_frame_error_deg = 0.0;
  double least_clearance_m = 0.0;     // In physics.
  std::string least_clearance_pair;   // "a to b", as LayoutScene::PairText.
  double least_clearance_t = 0.0;     // Plan time of the least distance.
  double plan_least_clearance_m = 0.0;  // The plan's, same instants.
  bool counted = false;         // StepCounts when the record was written.
};

// The module's pose on its rails after release and settling, against its
// designed place: the offset in the rails' plane (its length), across the
// tube and along it, the yaw about the rails' normal and the height above
// the designed place (not part of the gate).
struct SetDown {
  double offset_mm = 0.0;
  double across_mm = 0.0;
  double along_mm = 0.0;
  double yaw_deg = 0.0;
  double height_mm = 0.0;
};

struct PhysicsRecord {
  std::string layout;
  std::string layout_sha256;    // Of layouts/<layout>.json.
  std::string plan_sha256;      // Of plans/<layout>.json.
  std::string command;          // The command that wrote the record.
  int module = -1;              // The bay's module the run carried.
  int take = -1;                // Its module_take index.
  double clearance_m = 0.0;     // The layout's margins.clearance_m.
  double gate_offset_mm = 0.0;  // kGateOffset in mm.
  double gate_yaw_deg = 0.0;    // kGateYaw in degrees.
  std::vector<PhysicsStep> steps;  // In the plan's order.
  SetDown set_down;
  double counted_s = 0.0;       // CountedSeconds when it was written.
};

// The rule above, for one step of the record.
bool StepCounts(const PhysicsRecord& record, const PhysicsStep& step);

// The sum of planned_s over the steps for which StepCounts holds (not the
// stored flags).
double CountedSeconds(const PhysicsRecord& record);

// The record as text, the same for the same record, and back. FromJson
// returns nothing and sets *error on a missing field or a wrong type.
std::string PhysicsRecordToJson(const PhysicsRecord& record);
std::optional<PhysicsRecord> PhysicsRecordFromJson(const std::string& text,
                                                   std::string* error);

// What does not agree between the record and a committed plan, one line
// each; empty when it agrees: the layout's name, plan_sha256 against
// `plan_sha256`, and each step: it exists in plan.steps by number with the
// same what and arm, and start_s, end_s and planned_s within 1e-6 s of the
// plan's. Also each stored `counted` against StepCounts and counted_s
// against CountedSeconds (within 1e-6 s).
std::vector<std::string> CheckPhysicsRecord(const PhysicsRecord& record,
                                            const Plan& plan,
                                            const std::string& plan_sha256);

}  // namespace cell::layout

#endif  // SRC_PHYSICS_RECORD_H_
