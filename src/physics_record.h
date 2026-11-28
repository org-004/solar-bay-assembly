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
// A record holds one or several takes of one bay (PhysicsTake): each with
// its module, its module_take index, its steps (the take and the set after
// it) and the module's set-down. One bay is counted once: no module twice,
// and every step of the record of the same bay of the plan (bay 0, whose
// one_bay_s is the denominator; the later bays repeat the same modules).
// CountedSeconds sums over every take, each step judged against its own
// take's set-down.
//
// Rail sets (run 074). A record may also hold rail sets of the same bay
// (PhysicsRailSet): each with its rail, its one step (the module arm's
// `set rail k`, from the step's start to its end, the rail held at the
// gripper by a weld and let go at the end of the step's work segment), the
// peak torque of each motor over its limit, and the rail's pose on the tube
// after release and settling against its designed place (RailSetPose). A
// rail set's step counts by its own rule, the rail gate
// (docs/flat-rack-physics.md, "The rail gate"), when
//   1. its least clearance in physics is at least clearance_m,
//   2. no motor reached its limit (every torque_ratio below 1: a position
//      servo's force is clipped at its limit, so a ratio of 1 is a motor
//      that asked for more), and
//   3. the settled rail is within the play of docs/bay-hardware.md,
//      Tolerances: at its tab, across the tube at most rail_gate.across_mm
//      and along it at most along_mm; round the tube at most round_deg and
//      in yaw at most yaw_deg (src/bay_hardware.h, kTabPlayAcrossTube,
//      kRailPlayAlongTube, kRailPlayRound, kRailPlayYaw).
// CountedSeconds sums the rail sets' counted steps with the takes'. One bay
// is counted once: no rail twice. A record without rail sets is written as
// it always was: "rail_gate" and "rails" appear only when there are rail
// sets, after the takes (or the one take's fields):
//   "rail_gate": {"across_mm", "along_mm", "round_deg", "yaw_deg"},
//   "rails": [{"rail", "step": {as a step above}, "torque_ratio": [7],
//              "pose": {"across_mm", "along_mm", "round_deg", "yaw_deg",
//                       "height_mm"}}]
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
// for a record of one take (as every record written before run 071 is, so
// that their text and their sha256 stay what they were), and for several
//   {"layout", "layout_sha256", "plan_sha256", "command",
//    "clearance_m", "gate_offset_mm", "gate_yaw_deg",
//    "takes": [{"module", "take", "steps": [...], "set_down": {...}}],
//    "counted_s"}
// with the takes in the plan's order. Seconds to 6 decimals (as the plan
// holds them), distances in metres to 6, millimetres, milliradians and
// degrees to 4.

#ifndef SRC_PHYSICS_RECORD_H_
#define SRC_PHYSICS_RECORD_H_

#include <array>
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

// One take of the record: the module's take and the set after it.
struct PhysicsTake {
  int module = -1;              // The bay's module the run carried.
  int take = -1;                // Its module_take index.
  std::vector<PhysicsStep> steps;  // In the plan's order.
  SetDown set_down;
};

// A rail's pose on the tube after release and settling, against its
// designed place, in the tube's frame (docs/bay-hardware.md): the tab's
// offset across (X) and along (Y) the tube, the turn round the tube (about
// Y) and the yaw (about Z), and the height of the rail's origin (not part
// of the gate).
struct RailSetPose {
  double across_mm = 0.0;
  double along_mm = 0.0;
  double round_deg = 0.0;
  double yaw_deg = 0.0;
  double height_mm = 0.0;
};

// The rail's play on the tube (the rail gate's last part).
struct RailGate {
  double across_mm = 0.0;
  double along_mm = 0.0;
  double round_deg = 0.0;
  double yaw_deg = 0.0;
};

// One rail set of the record.
struct PhysicsRailSet {
  int rail = -1;
  PhysicsStep step;                      // The plan's `set rail <rail>`.
  std::array<double, 7> torque_ratio{};  // Joints 1-6, then the track.
  RailSetPose pose;
};

struct PhysicsRecord {
  std::string layout;
  std::string layout_sha256;    // Of layouts/<layout>.json.
  std::string plan_sha256;      // Of plans/<layout>.json.
  std::string command;          // The command that wrote the record.
  double clearance_m = 0.0;     // The layout's margins.clearance_m.
  double gate_offset_mm = 0.0;  // kGateOffset in mm.
  double gate_yaw_deg = 0.0;    // kGateYaw in degrees.
  std::vector<PhysicsTake> takes;  // In the plan's order; one or more.
  RailGate rail_gate;           // The rail's play; with rail sets only.
  std::vector<PhysicsRailSet> rails;  // In the plan's order; may be empty.
  double counted_s = 0.0;       // CountedSeconds when it was written.
};

// The rail gate's play, from src/bay_hardware.h, to the record's precision.
RailGate RailPlay();

// The rail gate above, for one rail set of the record.
bool RailSetCounts(const PhysicsRecord& record, const PhysicsRailSet& set);

// The rails of the rail sets that count, in the record's order.
std::vector<int> CountedRails(const PhysicsRecord& record);

// The rails that count as the scorecard's basis names them: "rails 0, 1
// (2 of the record's 3 rail sets)".
std::string RailsText(const PhysicsRecord& record);

// The numbers of the steps that count: the takes' in the record's order,
// then, with rail sets, theirs, all in increasing order.
std::vector<int> CountedSteps(const PhysicsRecord& record);

// The rule above, for one step of a take of the record.
bool StepCounts(const PhysicsRecord& record, const PhysicsTake& take,
                const PhysicsStep& step);

// The sum of planned_s over the steps of every take for which StepCounts
// holds, and of every rail set for which RailSetCounts holds (not the
// stored flags).
double CountedSeconds(const PhysicsRecord& record);

// The modules of the takes of which a step counts, in the record's order.
std::vector<int> CountedModules(const PhysicsRecord& record);

// Several records of runs of the same layout, plan and gate as one record
// whose takes are theirs in the plan's order, its flags and counted_s
// computed again. Returns nothing and sets *error when they are of another
// layout, plan or gate, or when there are none.
std::optional<PhysicsRecord> JoinPhysicsRecords(
    const std::vector<PhysicsRecord>& records, const std::string& command,
    std::string* error);

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
// against CountedSeconds (within 1e-6 s). And one bay counted once: a
// record without takes or rail sets, a take without steps, a module held by two takes,
// or steps of different bays of the plan (StepTime::bay) are faults; so
// are a rail set twice for one rail, and a rail set whose step is not
// `set rail <rail>`. Rail sets' steps are checked as the takes' are.
std::vector<std::string> CheckPhysicsRecord(const PhysicsRecord& record,
                                            const Plan& plan,
                                            const std::string& plan_sha256);

}  // namespace cell::layout

#endif  // SRC_PHYSICS_RECORD_H_
