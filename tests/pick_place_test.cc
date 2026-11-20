// Checks the blind pick and place (src/pick_place.h), phase by phase and end
// to end.
//
//   bazel test //:pick_place_test
//   bazel test --config=opt //:pick_place_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/cell.xml).
//
// Simulated time is what this test spends, and the cell simulates at about
// real time unoptimized. So the nominal cycle is simulated once, for the
// whole suite: NominalCycle's SetUpTestSuite runs module_0's cycle with the
// script paused after each phase, measures what that phase left behind from
// the truth, and goes on; then it runs module_1's cycle. Each phase's test
// then checks its own numbers from that record, and names the phase and the
// number when it fails. Other tests build their own scenes: the pallet a
// few millimetres off (blindness), the set-down started from a module put
// straight into the grip over the slot (the place side alone), and a few
// randomized pallets.
//
// The phases, as the script's steps group:
//   approach          to standoff, approach, touch glass
//   grip              grip
//   lift-off          lift, pull back
//   carry             clear, flatten, swing, turn, wait conveyor, lower
//   set-down/release  set down, release
//   retreat/conveyor  retreat, conveyor, return; then the tube's step
//
// Every number measured is printed (--test_output=all); the bounds below
// say what each was measured at and how much margin they leave.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/cell.h"
#include "src/conveyor.h"
#include "src/cycle_check.h"
#include "src/pallet.h"
#include "src/pick_place.h"
#include "src/score.h"
#include "src/suction.h"

namespace {

using cell::Cell;
using cell::Clearance;
using cell::CycleCheck;
using cell::PickPlace;
using cell::PickPlaceOutcome;
using cell::PickPlaceStep;
using Step = cell::PickPlaceStep;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;
constexpr double kMm = 1e-3;

// A cycle that has not ended after this much simulated time has hung.
constexpr double kCycleLimit = 90.0;  // s

std::optional<Cell> LoadCell(std::string* error) {
  const char* path = std::getenv("MODEL_PATH");
  if (path == nullptr) {
    *error = "MODEL_PATH is not set";
    return std::nullopt;
  }
  return Cell::Load(path, error);
}

// Runs the script one physics step at a time until it pauses or ends, or
// `limit` seconds have passed. Returns whether it paused or ended.
bool RunScript(Cell& cell, PickPlace& script, CycleCheck* check, int module,
         double limit = kCycleLimit) {
  const double until = cell.data()->time + limit;
  while (!script.paused() && !script.Done() && cell.data()->time < until) {
    script.Update();
    cell.Step();
    if (check != nullptr) check->Observe(script.step(), module);
  }
  mj_forward(cell.model(), cell.data());
  return script.paused() || script.Done();
}

// Steps with the script holding the arm until the conveyor has stopped.
void RunUntilConveyorStops(Cell& cell, PickPlace& script, CycleCheck* check,
                           int module) {
  const double until = cell.data()->time + 10.0;
  while (cell.conveyor().Moving(*cell.data()) && cell.data()->time < until) {
    script.Update();
    cell.Step();
    if (check != nullptr) check->Observe(script.step(), module);
  }
  mj_forward(cell.model(), cell.data());
}

// Forbidden contacts the check saw in steps first .. last.
std::vector<cell::ForbiddenContact> ForbiddenIn(const CycleCheck& check,
                                                Step first, Step last) {
  std::vector<cell::ForbiddenContact> found;
  for (const cell::ForbiddenContact& f : check.forbidden()) {
    if (static_cast<int>(f.step) >= static_cast<int>(first) &&
        static_cast<int>(f.step) <= static_cast<int>(last)) {
      found.push_back(f);
    }
  }
  return found;
}

std::string Describe(const std::vector<cell::ForbiddenContact>& contacts) {
  std::string text;
  for (const cell::ForbiddenContact& f : contacts) {
    char line[300];
    std::snprintf(line, sizeof(line), "\n  t %.3f s in %s: %s", f.time,
                  cell::StepName(f.step), f.between.c_str());
    text += line;
  }
  return text;
}

// Angle of a rotation, rad.
double Angle(const Eigen::Matrix3d& r) {
  return Eigen::AngleAxisd(r).angle();
}

// The smallest distance between any geom of body a and any of body b.
double BodyDistance(const Cell& cell, int a, int b) {
  const mjModel& m = *cell.model();
  double best = 1.0;
  for (int g = m.body_geomadr[a]; g < m.body_geomadr[a] + m.body_geomnum[a];
       ++g) {
    if (m.geom_contype[g] == 0 && m.geom_conaffinity[g] == 0) continue;
    for (int h = m.body_geomadr[b]; h < m.body_geomadr[b] + m.body_geomnum[b];
         ++h) {
      if (m.geom_contype[h] == 0 && m.geom_conaffinity[h] == 0) continue;
      best = std::min(best,
                      mj_geomDistance(cell.model(),
                                      const_cast<mjData*>(cell.data()), g, h,
                                      1.0, nullptr));
    }
  }
  return best;
}

std::string Phases(const PickPlace& script) {
  std::string text;
  for (const cell::StepRecord& r : script.records()) {
    char line[300];
    std::snprintf(line, sizeof(line), "\n  %-14s %6.3f s  %s",
                  cell::StepName(r.step), r.duration(),
                  r.finished ? r.result.c_str() : "(running)");
    text += line;
  }
  return text;
}

// ===================================================== the nominal cycle

// What NominalCycle's set-up measured, phase by phase.
struct NominalLog {
  std::string error;  // set-up went wrong: every test fails with this

  // After touch glass: where the tool tip is against module_0's true glass.
  bool approach_done = false;
  double gap = 0.0;          // m, tip in front of the glass plane
  double square = 0.0;       // rad, tool +Z against the glass's inward normal
  double off_centre = 0.0;   // m, tip from the glass centre, in the plane
  double approach_stack = 0.0;  // m, worst module move since Reset
  Eigen::Isometry3d module_in_tip = Eigen::Isometry3d::Identity();

  // After grip.
  bool grip_done = false;
  std::optional<int> held;  // module index held, from the bookkeeping
  double grip_pose_shift = 0.0;  // m, weld's pose against module_in_tip
  double grip_pose_turn = 0.0;   // rad

  // After pull back.
  bool lift_off_done = false;
  double module_1_distance = 0.0;  // m, module_0 to module_1
  double deck_distance = 0.0;      // m, module_0 to the pallet's deck
  cell::StackDisturbance lift_off_stack;  // the 29 still standing

  // After lower (the end of the carry): module_0 against its true slot.
  bool carry_done = false;
  double carry_tilt = 0.0;        // rad, glass normal from straight up
  double carry_yaw = 0.0;         // rad, long side from across the tube
  double carry_off_slot = 0.0;    // m, centre from the slot's, level
  double carry_height = 0.0;      // m, centre above where it will lie
  long saturated_by_carry = 0;
  Eigen::VectorXd peak_holding;   // torque / limit with the module held

  // After release.
  bool release_done = false;
  bool still_held = true;
  cell::PlacementError at_release;
  double tool_to_module = 0.0;  // m

  // At the placed hook (after the retreat, before the tube moves).
  std::map<int, cell::PlacementError> placed;
  std::map<int, cell::StackDisturbance> placed_stack;
  std::map<int, std::optional<int>> placed_slot;

  // After the cycle and the tube's step.
  bool cycle_done = false;
  PickPlaceOutcome outcome;
  double cycle_time = 0.0;
  double arm_off_end = 0.0;  // rad, worst joint from the return's end
  double tool_after_retreat = 0.0;  // m, tool to module_0 at the hook
  int steps_after = 0;
  double slide_after = 0.0;      // m, encoder
  double nominal_slide_after = 0.0;
  cell::PlacementError after_step;  // module_0 against slot 0, tube moved
  double module_0_y_placed = 0.0;   // m, module_0's centre at the hook
  double module_0_moved_y = 0.0;    // m, ...and how far it then moved
  std::string phases;

  // Forbidden contacts in each phase, as "time step: between" lines.
  std::string forbidden_approach, forbidden_lift_off, forbidden_carry,
      forbidden_set_down, forbidden_retreat;

  // The whole of module_0's cycle.
  long forbidden = 0;
  std::string forbidden_list;
  std::string check_report;
  long saturated = 0, out_of_range = 0, non_finite = 0;
  double closest[static_cast<int>(Clearance::kCount)] = {};

  // Every step's waypoints of module_0's cycle, for the blindness test.
  std::vector<std::pair<Step, std::vector<Eigen::VectorXd>>> waypoints;
  Eigen::Isometry3d grip_tip_0 = Eigen::Isometry3d::Identity();
  double place_x_sign = 1.0;

  // module_1's cycle, right after.
  bool second_done = false;
  PickPlaceOutcome second;
  Eigen::Isometry3d grip_tip_1 = Eigen::Isometry3d::Identity();
  long second_forbidden = 0;
  std::string second_phases;
};

class NominalCycle : public ::testing::Test {
 protected:
  static void SetUpTestSuite();
  static void TearDownTestSuite() {
    delete log_;
    log_ = nullptr;
  }
  void SetUp() override {
    ASSERT_NE(log_, nullptr);
    ASSERT_TRUE(log_->error.empty()) << log_->error;
  }
  static const NominalLog& log() { return *log_; }

  static NominalLog* log_;
};

NominalLog* NominalCycle::log_ = nullptr;

void NominalCycle::SetUpTestSuite() {
  log_ = new NominalLog;
  NominalLog& L = *log_;
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  if (!loaded) {
    L.error = error;
    return;
  }
  Cell& cell = *loaded;
  cell.Reset(1, cell::Randomization::None());
  const mjModel& model = *cell.model();
  const int m0 = cell.pallet().modules[0];
  const int m1 = cell.pallet().modules[1];
  const int deck = mj_name2id(&model, mjOBJ_GEOM, "pallet_deck");
  const int tool = cell.arm().tool_body;

  PickPlace script(cell);
  script.set_placed_hook([&](int module, int slot) {
    mj_forward(cell.model(), cell.data());
    L.placed[module] = cell::MeasurePlacement(cell, module, slot);
    L.placed_stack[module] = cell::MeasureStackDisturbance(cell);
    if (module == 0) {
      L.tool_after_retreat = BodyDistance(cell, tool, m0);
      L.module_0_y_placed = cell.TrueModulePose(0).translation().y();
    }
  });
  CycleCheck check(cell, {.clearance_every = 10});
  auto fail = [&](const std::string& where) {
    L.error = where + ": " + script.outcome().reason + Phases(script);
  };

  // ---- approach
  script.set_pause_after(Step::kTouchGlass);
  script.Start(0);
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("approach did not end");
  }
  L.approach_done = true;
  {
    const Eigen::Isometry3d tip = cell::TipPose(cell.arm(), *cell.data());
    const cell::GlassFace glass =
        cell::ModuleGlass(*cell.data(), cell.pallet(), 0);
    const Eigen::Vector3d d = tip.translation() - glass.centre;
    L.gap = d.dot(glass.normal);
    L.off_centre = (d - L.gap * glass.normal).norm();
    L.square = std::acos(
        std::clamp(-tip.linear().col(2).dot(glass.normal), -1.0, 1.0));
    L.approach_stack = cell::MeasureStackDisturbance(cell).worst_displacement;
    L.module_in_tip = tip.inverse() * cell.TrueModulePose(0);
  }

  // ---- grip
  script.set_pause_after(Step::kGrip);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("grip did not end");
  }
  L.grip_done = true;
  for (int i = 0; i < cell.module_count(); ++i) {
    if (cell.module_state(i) == cell::ModuleState::kHeld) L.held = i;
  }
  if (const std::optional<cell::Pose> weld =
          cell.suction().HeldPoseInTipAtGrip()) {
    L.grip_pose_shift =
        (weld->position - L.module_in_tip.translation()).norm();
    L.grip_pose_turn = Angle(weld->rotation.toRotationMatrix().transpose() *
                             L.module_in_tip.linear());
  }

  // ---- lift-off
  script.set_pause_after(Step::kPullBack);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("lift-off did not end");
  }
  L.lift_off_done = true;
  L.module_1_distance = BodyDistance(cell, m0, m1);
  L.deck_distance = mj_geomDistance(cell.model(), cell.data(),
                                    model.body_geomadr[m0], deck, 1.0, nullptr);
  L.lift_off_stack = cell::MeasureStackDisturbance(cell);

  // ---- carry
  script.set_pause_after(Step::kLower);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("carry did not end");
  }
  L.carry_done = true;
  {
    const Eigen::Isometry3d module = cell.TrueModulePose(0);
    const Eigen::Isometry3d slot = cell.conveyor().TrueSlotPose(*cell.data(), 0);
    L.carry_tilt = std::acos(std::clamp(
        module.linear().col(2).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0));
    Eigen::Vector3d long_side = module.linear().col(0);
    long_side.z() = 0.0;
    // Folded: either end along the slot's x is the same.
    L.carry_yaw = std::acos(std::min(
        1.0, std::abs(long_side.normalized().dot(slot.linear().col(0)))));
    const Eigen::Vector3d d = module.translation() - slot.translation();
    L.carry_off_slot = Eigen::Vector2d(d.x(), d.y()).norm();
    L.carry_height = d.z();
    L.saturated_by_carry = check.saturated_steps();
    L.peak_holding = check.peak_torque_holding();
  }

  // ---- set-down and release
  script.set_pause_after(Step::kRelease);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("set-down did not end");
  }
  L.release_done = true;
  L.still_held = cell.suction().holding().has_value();
  L.at_release = cell::MeasurePlacement(cell, 0, 0);
  L.tool_to_module = BodyDistance(cell, tool, m0);

  // ---- retreat, conveyor, return; then let the tube finish its step
  script.set_pause_after(std::nullopt);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.Done()) {
    return fail("the cycle did not end");
  }
  L.outcome = script.outcome();
  L.cycle_done = L.outcome.status == PickPlaceOutcome::Status::kPlaced;
  L.cycle_time = L.outcome.end_time - L.outcome.start_time;
  L.phases = Phases(script);
  L.arm_off_end = (cell::JointAngles(cell.arm(), *cell.data()) -
                   script.Waypoints(Step::kReturn)->back())
                      .cwiseAbs()
                      .maxCoeff();
  RunUntilConveyorStops(cell, script, &check, 0);
  L.module_0_moved_y =
      cell.TrueModulePose(0).translation().y() - L.module_0_y_placed;
  L.steps_after = cell.conveyor().steps();
  L.slide_after = cell.conveyor().SensorPosition(*cell.data());
  L.nominal_slide_after = cell.conveyor().nominal_position();
  L.after_step = cell::MeasurePlacement(cell, 0, 0);
  L.placed_slot[0] = cell.TruePlacedSlot(0);

  L.forbidden_approach =
      Describe(ForbiddenIn(check, Step::kToStandoff, Step::kGrip));
  L.forbidden_lift_off =
      Describe(ForbiddenIn(check, Step::kLift, Step::kPullBack));
  L.forbidden_carry = Describe(ForbiddenIn(check, Step::kClear, Step::kLower));
  L.forbidden_set_down =
      Describe(ForbiddenIn(check, Step::kSetDown, Step::kRelease));
  L.forbidden_retreat = Describe(ForbiddenIn(check, Step::kRetreat, Step::kDone));
  L.forbidden = check.forbidden_count();
  L.forbidden_list = Describe(check.forbidden());
  L.check_report = check.Report(true);
  L.saturated = check.saturated_steps();
  L.out_of_range = check.out_of_range_steps();
  L.non_finite = check.non_finite_steps();
  for (int c = 0; c < static_cast<int>(Clearance::kCount); ++c) {
    L.closest[c] = check.closest(static_cast<Clearance>(c)).distance;
  }
  for (Step s : {Step::kToStandoff, Step::kApproach, Step::kTouchGlass,
                 Step::kLift, Step::kPullBack, Step::kClear, Step::kFlatten,
                 Step::kSwing, Step::kTurn, Step::kLower, Step::kSetDown,
                 Step::kRetreat, Step::kReturn}) {
    L.waypoints.push_back({s, *script.Waypoints(s)});
  }
  L.grip_tip_0 = script.grip_tip();
  L.place_x_sign = script.place_x_sign();

  // ---- module_1, straight after
  CycleCheck second_check(cell, {.clearance_every = 0});
  script.Start(1);
  L.grip_tip_1 = script.grip_tip();
  RunScript(cell, script, &second_check, 1);
  L.second = script.outcome();
  L.second_done = L.second.status == PickPlaceOutcome::Status::kPlaced;
  L.second_phases = Phases(script);
  RunUntilConveyorStops(cell, script, &second_check, 1);
  L.placed_slot[1] = cell.TruePlacedSlot(1);
  L.second_forbidden = second_check.forbidden_count();

  std::printf("module_0's cycle, %.2f s:%s\n%s", L.cycle_time,
              L.phases.c_str(), L.check_report.c_str());
  std::printf("module_1's cycle:%s\n", L.second_phases.c_str());
}

// ---- approach: the tool stops at the planned gap from module_0's glass,
// square to it, having touched nothing.
TEST_F(NominalCycle, ApproachStopsAtTheGapSquareToTheGlassTouchingNothing) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.approach_done);
  std::printf("approach: gap %.3f mm (planned 2.0), off centre %.3f mm, "
              "square within %.4f deg, stack moved %.4f mm\n",
              L.gap / kMm, L.off_centre / kMm, L.square / kDegree,
              L.approach_stack / kMm);
  // Planned 2 mm; measured about 2.1 mm: the arm's tracking error and the
  // stack settling, each a tenth of a millimetre. 0.5 mm either way leaves
  // the grip (10 mm) far away.
  EXPECT_NEAR(L.gap, 2.0 * kMm, 0.5 * kMm) << "approach: gap";
  // Measured a few hundredths of a millimetre.
  EXPECT_LT(L.off_centre, 0.5 * kMm) << "approach: tip off the glass centre";
  // Within the grip's 3 degrees by far: measured under 0.01 degree.
  EXPECT_LT(L.square, 0.05 * kDegree) << "approach: tool not square";
  // Nothing touched: the stack as it settled at Reset, to a hundredth of a
  // millimetre (it moves less than 0.1 mm in 10 s left alone).
  EXPECT_LT(L.approach_stack, 0.01 * kMm) << "approach: the stack moved";
  EXPECT_EQ(L.forbidden_approach, "") << "approach: forbidden contacts";
}

// ---- grip: holds module_0, at the pose the approach left it in.
TEST_F(NominalCycle, GripHoldsModule0AsTheApproachLeftIt) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.grip_done);
  std::printf("grip: holding module_%d, weld pose off the approach's by "
              "%.4f mm and %.5f deg\n",
              L.held.value_or(-1), L.grip_pose_shift / kMm,
              L.grip_pose_turn / kDegree);
  ASSERT_TRUE(L.held.has_value()) << "grip: nothing held";
  EXPECT_EQ(*L.held, 0) << "grip: held the wrong module";
  // The weld keeps the pose of the instant of the grip, which is the
  // instant the approach ended; nothing moves in between.
  EXPECT_LT(L.grip_pose_shift, 0.01 * kMm) << "grip: captured pose moved";
  EXPECT_LT(L.grip_pose_turn, 0.001 * kDegree) << "grip: captured pose turned";
}

// ---- lift-off: module_0 clear of module_1 by the pull-back, of the deck by
// the lift, the rest of the stack left standing, nothing hit.
TEST_F(NominalCycle, LiftOffClearsModule0OfTheStackLeavingItStanding) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.lift_off_done);
  const cell::PickPlaceOptions options;
  const double planned_back = options.pull_back + options.lift_back;
  std::printf("lift-off: module_0 %.2f mm from module_1 (planned %.0f), "
              "%.2f mm from the deck (planned %.0f); %s\n",
              L.module_1_distance / kMm, planned_back / kMm,
              L.deck_distance / kMm, options.lift / kMm,
              cell::Describe(L.lift_off_stack).c_str());
  // The lift (30 mm up, 4 mm back) and the pull-back (50 mm back),
  // straight legs: measured within a few tenths of a millimetre of plan.
  EXPECT_NEAR(L.module_1_distance, planned_back, 1.0 * kMm)
      << "lift-off: module_0's clearance from module_1";
  EXPECT_NEAR(L.deck_distance, options.lift, 1.0 * kMm)
      << "lift-off: module_0's clearance from the deck";
  // Measured 0.002 mm: the lift backs off the glass a little, so module_0
  // no longer rubs on module_1 (lifting straight along the glass moved
  // module_1 by 0.08 mm). 0.25 mm and 0.02 degrees: a module rubbed or
  // knocked shows here, the stack's own settling (0.1 mm in 10 s) does not.
  EXPECT_EQ(L.lift_off_stack.modules, 29);
  EXPECT_LT(L.lift_off_stack.worst_displacement, 0.25 * kMm)
      << "lift-off: the stack moved, worst module_"
      << L.lift_off_stack.worst_displacement_module;
  EXPECT_LT(L.lift_off_stack.worst_rotation, 0.02 * kDegree)
      << "lift-off: the stack turned";
  EXPECT_EQ(L.forbidden_lift_off, "") << "lift-off: forbidden contacts";
}

// ---- carry: ends over the slot, flat and long side across the tube,
// without touching anything or saturating a motor.
TEST_F(NominalCycle, CarryEndsFlatOverTheSlotWithoutTouchingAnything) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.carry_done);
  const cell::PickPlaceOptions options;
  std::printf("carry: tilt %.4f deg, long side %.4f deg from across, centre "
              "%.3f mm from the slot's, %.2f mm above where it will lie "
              "(planned %.0f + %.0f)\n",
              L.carry_tilt / kDegree, L.carry_yaw / kDegree,
              L.carry_off_slot / kMm, L.carry_height / kMm,
              options.set_down_slow / kMm, options.release_clearance / kMm);
  // Flat and square to the tube within a hundredth of a degree measured;
  // 0.1 degree is 2 mm over the module's length.
  EXPECT_LT(L.carry_tilt, 0.1 * kDegree) << "carry: module not flat";
  EXPECT_LT(L.carry_yaw, 0.1 * kDegree) << "carry: long side not across";
  EXPECT_LT(L.carry_off_slot, 1.0 * kMm) << "carry: not over the slot";
  EXPECT_NEAR(L.carry_height, options.set_down_slow + options.release_clearance,
              1.0 * kMm)
      << "carry: height above the rails";
  // The 30 kg module hangs on the weld, outside the follower's model.
  EXPECT_EQ(L.forbidden_carry, "") << "carry: forbidden contacts";
  EXPECT_EQ(L.saturated_by_carry, 0) << "carry: a motor saturated";
  for (int j = 0; j < cell::Arm::kJoints; ++j) {
    EXPECT_LT(L.peak_holding[j], 0.9)
        << "carry: joint " << j + 1 << " near its torque limit";
  }
}

// ---- set-down and release: lying on its rails within the placement
// bounds, let go, the tool off it.
TEST_F(NominalCycle, SetDownLaysModule0OnItsRailsAndLetsGo) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.release_done);
  std::printf("set-down: %s; tool %.2f mm from it\n",
              cell::Describe(L.at_release).c_str(), L.tool_to_module / kMm);
  EXPECT_FALSE(L.still_held) << "set-down: still held";
  const cell::PlacementError& e = L.at_release;
  // Measured a tenth of a millimetre and a few thousandths of a degree.
  EXPECT_LT(std::abs(e.along_tube), 1.0 * kMm) << "set-down: along error";
  EXPECT_LT(std::abs(e.across_tube), 1.0 * kMm) << "set-down: across error";
  // Let go 1 mm up; lying on the rails, within the contacts' give.
  EXPECT_LT(std::abs(e.height), 0.3 * kMm) << "set-down: not on the rails";
  EXPECT_LT(std::abs(e.yaw), 0.05 * kDegree) << "set-down: yaw";
  EXPECT_LT(e.tilt, 0.05 * kDegree) << "set-down: tilt";
  // The cups 2 mm above the glass, as they held it.
  EXPECT_GT(L.tool_to_module, 0.5 * kMm) << "set-down: tool still on it";
  EXPECT_EQ(L.forbidden_set_down, "") << "set-down: forbidden contacts";
}

// ---- retreat and conveyor: the arm back at ready, the tube one step on,
// the module riding along in its slot.
TEST_F(NominalCycle, RetreatThenTheTubeMovesOnWithTheModule) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.cycle_done) << L.outcome.reason << L.phases;
  ASSERT_TRUE(L.placed.count(0));
  const cell::PickPlaceOptions options;
  std::printf("retreat: tool %.1f mm above module_0 (planned %.0f); arm "
              "%.3f mrad from ready; tube at %.4f m (nominal %.4f), %d "
              "steps; module_0 moved %.2f mm along Y; after the step: %s\n",
              L.tool_after_retreat / kMm, options.retreat / kMm,
              L.arm_off_end / 1e-3, L.slide_after, L.nominal_slide_after,
              L.steps_after, L.module_0_moved_y / kMm,
              cell::Describe(L.after_step).c_str());
  EXPECT_NEAR(L.tool_after_retreat, options.retreat, 5.0 * kMm)
      << "retreat: tool clearance above the module";
  EXPECT_LT(L.arm_off_end, 1e-3) << "return: arm not at ready";
  EXPECT_EQ(L.steps_after, 1) << "conveyor: steps";
  // No stopping error at nominal; the drive stops within its 0.5 mm.
  EXPECT_NEAR(L.slide_after, L.nominal_slide_after, 0.5 * kMm)
      << "conveyor: where the tube stopped";
  EXPECT_NEAR(L.module_0_moved_y, -cell::Conveyor::kStepLength, 1.0 * kMm)
      << "conveyor: module_0 did not ride along";
  const cell::PlacementError& before = L.placed.at(0);
  const cell::PlacementError& after = L.after_step;
  EXPECT_TRUE(after.at_rest) << "conveyor: module_0 still moving";
  EXPECT_NEAR(after.along_tube, before.along_tube, 0.2 * kMm)
      << "conveyor: module_0 slid along the tube";
  EXPECT_NEAR(after.across_tube, before.across_tube, 0.2 * kMm)
      << "conveyor: module_0 slid across the tube";
  EXPECT_NEAR(after.yaw, before.yaw, 0.01 * kDegree)
      << "conveyor: module_0 turned";
  EXPECT_EQ(L.placed_slot.at(0), std::optional<int>(0));
  EXPECT_EQ(L.forbidden_retreat, "") << "retreat: forbidden contacts";
}

// ---- end to end: placed, every bound at once, the stack left standing,
// nothing forbidden touched, motors and joints within their limits.
TEST_F(NominalCycle, WholeCycleIsCleanAndWithinBounds) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.cycle_done) << L.outcome.reason << L.phases;
  ASSERT_TRUE(L.placed.count(0));
  const cell::PlacementError& e = L.placed.at(0);
  std::printf("end to end: %s\n  %s\n", cell::Describe(e).c_str(),
              cell::Describe(L.placed_stack.at(0)).c_str());
  // The placement, measured after the retreat: along +0.03 mm, across
  // +0.09 mm, height 0.00 mm, yaw and tilt under 0.001 degree. Bounds of
  // 1 mm and 0.05 degree: ten times what was measured, and still well
  // inside "a few millimetres and a fraction of a degree".
  EXPECT_LT(std::abs(e.along_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(e.across_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(e.height), 0.3 * kMm);
  EXPECT_LT(std::abs(e.yaw), 0.05 * kDegree);
  EXPECT_LT(e.tilt, 0.05 * kDegree);
  EXPECT_TRUE(e.at_rest);
  // The 29 left on the pallet, against where they stood at Reset: measured
  // 0.002 mm (0.004 mm at worst over a whole pallet in src/run_cell.cc).
  EXPECT_EQ(L.placed_stack.at(0).modules, 29);
  EXPECT_LT(L.placed_stack.at(0).worst_displacement, 0.25 * kMm);
  EXPECT_LT(L.placed_stack.at(0).worst_rotation, 0.02 * kDegree);

  EXPECT_EQ(L.forbidden, 0) << L.forbidden_list;
  EXPECT_EQ(L.saturated, 0) << "a motor saturated";
  EXPECT_EQ(L.out_of_range, 0) << "a joint left its range";
  EXPECT_EQ(L.non_finite, 0) << "the state went non-finite";
  // The tightest clearances measured, with room for small changes in the
  // poses: the held module and the upper arm at the quarter turn (about
  // 159 mm), the held module and the rails during the carry (about 100 mm),
  // the bottom edge and the deck just after the lift (about 30 mm, the
  // lift itself).
  EXPECT_GT(L.closest[static_cast<int>(Clearance::kModuleArm)], 0.10);
  EXPECT_GT(L.closest[static_cast<int>(Clearance::kModuleConveyor)], 0.07);
  EXPECT_GT(L.closest[static_cast<int>(Clearance::kModuleStack)], 0.025);
  EXPECT_GT(L.closest[static_cast<int>(Clearance::kModulePost)], 0.5);
  EXPECT_GT(L.closest[static_cast<int>(Clearance::kArmPost)], 0.5);
}

// ---- two in a row: module_1's face is one thickness behind module_0's,
// and module_1 lands in slot 1, beside module_0, as well placed.
TEST_F(NominalCycle, SecondModuleFromOneThicknessBackIntoTheNextSlot) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.cycle_done);
  ASSERT_TRUE(L.second_done) << L.second.reason << L.second_phases;
  ASSERT_TRUE(L.placed.count(1));
  const Eigen::Vector3d step =
      L.grip_tip_1.translation() - L.grip_tip_0.translation();
  // Along the glass's normal, into the stack: the tool's +Z.
  const Eigen::Vector3d into = L.grip_tip_0.linear().col(2);
  std::printf("second: grip tip %.3f mm further into the stack, %.4f mm "
              "sideways; %s; %ld forbidden contacts\n",
              step.dot(into) / kMm, (step - step.dot(into) * into).norm() / kMm,
              cell::Describe(L.placed.at(1)).c_str(), L.second_forbidden);
  EXPECT_NEAR(step.dot(into), cell::kModuleThickness, 1e-9);
  EXPECT_LT((step - step.dot(into) * into).norm(), 1e-9);
  EXPECT_EQ(L.placed.at(1).slot, 1);
  EXPECT_EQ(L.placed_slot.at(1), std::optional<int>(1));
  const cell::PlacementError& e = L.placed.at(1);
  EXPECT_LT(std::abs(e.along_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(e.across_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(e.height), 0.3 * kMm);
  EXPECT_LT(std::abs(e.yaw), 0.05 * kDegree);
  EXPECT_LT(e.tilt, 0.05 * kDegree);
  EXPECT_EQ(L.second_forbidden, 0);
}

// ---- blindness: with the pallet a few millimetres off, the script plans
// the very same joint waypoints, and the placement is off by what the
// pallet was.
TEST_F(NominalCycle, BlindScriptPlansTheSameWaypointsWhereverThePalletIs) {
  const NominalLog& L = log();
  ASSERT_TRUE(L.cycle_done);
  ASSERT_TRUE(L.placed.count(0));
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  cell.Reset(1, cell::Randomization::None());
  // Moved after Reset, behind the script's back: 3 mm away from the arm
  // (the glass 3 mm further than designed: the grip still takes, at 5 mm)
  // and 5 mm to +Y, along the modules' long side. Then settled, as Reset
  // does.
  constexpr double kDx = 0.003, kDy = 0.005;
  cell::PlacePallet(*cell.model(), cell.data(), cell.pallet(), kDx, kDy, 0.0);
  cell.StepFor(Cell::kSettleTime);
  mj_forward(cell.model(), cell.data());
  cell.RebaselineStack();

  PickPlace script(cell);
  std::optional<cell::PlacementError> placed;
  script.set_placed_hook([&](int module, int slot) {
    mj_forward(cell.model(), cell.data());
    placed = cell::MeasurePlacement(cell, module, slot);
  });
  script.Start(0);
  RunScript(cell, script, nullptr, 0);
  ASSERT_EQ(script.outcome().status, PickPlaceOutcome::Status::kPlaced)
      << script.outcome().reason << Phases(script);
  ASSERT_TRUE(placed);

  // The same waypoints, to the last bit: nothing the script read differed.
  for (const auto& [step, nominal] : L.waypoints) {
    const std::vector<Eigen::VectorXd>& now = *script.Waypoints(step);
    ASSERT_EQ(now.size(), nominal.size()) << cell::StepName(step);
    for (size_t i = 0; i < now.size(); ++i) {
      EXPECT_EQ(now[i], nominal[i])
          << cell::StepName(step) << " waypoint " << i << " differs by "
          << (now[i] - nominal[i]).cwiseAbs().maxCoeff() << " rad";
    }
  }

  // The placement moves with the pallet. The module was gripped 5 mm
  // toward +Y of its centre's design place, along its long side, which
  // lies across the tube at the place: across moves by -sign * 5 mm. And
  // 3 mm further into the stack, which the grip takes up as a larger gap,
  // and sin(10 deg) * 3 mm = 0.52 mm further up the lean, which ends along
  // the tube.
  const cell::PlacementError& n = L.placed.at(0);
  const double sign = L.place_x_sign;
  const double d_across = placed->across_tube - n.across_tube;
  const double d_along = placed->along_tube - n.along_tube;
  std::printf("blind, pallet +%.0f mm x, +%.0f mm y: %s; across moved %+.3f "
              "mm (expected %+.3f), along %+.3f mm (expected %+.3f)\n",
              kDx / kMm, kDy / kMm, cell::Describe(*placed).c_str(),
              d_across / kMm, -sign * kDy / kMm, d_along / kMm,
              sign * std::sin(10.0 * kDegree) * kDx / kMm);
  EXPECT_NEAR(d_across, -sign * kDy, 0.3 * kMm);
  EXPECT_NEAR(d_along, sign * std::sin(10.0 * kDegree) * kDx, 0.3 * kMm);
}

// ===================================================== the place side alone

// The set-down, release, retreat and conveyor step from a constructed
// state: the arm put at the end of the quarter turn, module_0 put into the
// grip exactly as designed (tip 2 mm from the glass centre, tool X along
// its long side) and gripped. Nothing before the wait for the conveyor
// runs, so a failure here is the place side's own.
TEST(PlaceSide, SetDownFromAModulePutIntoTheGripOverTheSlot) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  cell.Reset(1, cell::Randomization::None());

  // A script only to plan: where the turn ends.
  Eigen::VectorXd turned;
  double gap = 0.0;
  {
    PickPlace planner(cell);
    planner.Start(0);
    ASSERT_NE(planner.Waypoints(Step::kTurn), nullptr);
    turned = planner.Waypoints(Step::kTurn)->back();
    gap = planner.options().grip_gap;
  }
  cell.PutArm(turned);
  const Eigen::Isometry3d tip = cell::TipPose(cell.arm(), *cell.data());
  // module_0 in the tip's frame as the grip holds it: its +x along the
  // tool's X, its glass (+z) facing the tool, its centre gap + half a
  // thickness along the tool's +Z.
  Eigen::Isometry3d in_tip = Eigen::Isometry3d::Identity();
  in_tip.linear() << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  in_tip.translation() = Eigen::Vector3d(0, 0, gap + cell::kModuleThickness / 2);
  cell.TeleportModule(0, tip * in_tip);
  cell::GripFailure failure;
  ASSERT_EQ(cell.Grip(&failure), std::optional<int>(0)) << failure.message;

  PickPlace script(cell);
  std::optional<cell::PlacementError> placed;
  script.set_placed_hook([&](int module, int slot) {
    mj_forward(cell.model(), cell.data());
    placed = cell::MeasurePlacement(cell, module, slot);
  });
  CycleCheck check(cell, {.clearance_every = 0});
  ASSERT_TRUE(script.Start(0, Step::kWaitForConveyor))
      << script.outcome().reason;
  RunScript(cell, script, &check, 0);
  ASSERT_EQ(script.outcome().status, PickPlaceOutcome::Status::kPlaced)
      << "place side: " << script.outcome().reason << Phases(script);
  ASSERT_TRUE(placed);
  std::printf("place side alone: %s%s\n", cell::Describe(*placed).c_str(),
              Phases(script).c_str());
  // As in the whole cycle: the place side adds about a tenth of a
  // millimetre of its own.
  EXPECT_LT(std::abs(placed->along_tube), 1.0 * kMm) << "place side: along";
  EXPECT_LT(std::abs(placed->across_tube), 1.0 * kMm) << "place side: across";
  EXPECT_LT(std::abs(placed->height), 0.3 * kMm) << "place side: height";
  EXPECT_LT(std::abs(placed->yaw), 0.05 * kDegree) << "place side: yaw";
  EXPECT_LT(placed->tilt, 0.05 * kDegree) << "place side: tilt";
  EXPECT_TRUE(placed->at_rest) << "place side: still moving";
  EXPECT_EQ(check.forbidden_count(), 0) << Describe(check.forbidden());
  EXPECT_EQ(check.saturated_steps(), 0);
}

// ===================================================== randomized pallets

// A few pallets set down off nominal. The blind script may well fail here:
// that is the point of the next stages. What must hold: every run ends in a
// defined outcome (placed, or failed with a reason, in a named step), the
// state stays finite, the joints in range, and a failed run leaves the
// stack standing.
TEST(RandomizedPallets, EveryRunEndsInADefinedOutcome) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  for (std::uint32_t seed : {1u, 2u, 4u, 5u}) {
    cell.Reset(seed, cell::Randomization{});
    PickPlace script(cell);
    CycleCheck check(cell, {.clearance_every = 0});
    script.Start(0);
    const bool ended = RunScript(cell, script, &check, 0);
    const cell::TrueScene& scene = cell.true_scene();
    const PickPlaceOutcome& outcome = script.outcome();
    std::printf("seed %u (pallet %+.1f, %+.1f mm, %+.2f deg): %s%s%s\n", seed,
                scene.pallet_x / kMm, scene.pallet_y / kMm,
                scene.pallet_yaw / kDegree,
                outcome.status == PickPlaceOutcome::Status::kPlaced ? "placed"
                                                                    : "failed",
                outcome.reason.empty() ? "" : ": ", outcome.reason.c_str());
    ASSERT_TRUE(ended) << "seed " << seed << ": hung" << Phases(script);
    EXPECT_TRUE(outcome.status == PickPlaceOutcome::Status::kPlaced ||
                outcome.status == PickPlaceOutcome::Status::kFailed)
        << "seed " << seed;
    if (outcome.status == PickPlaceOutcome::Status::kFailed) {
      EXPECT_FALSE(outcome.reason.empty()) << "seed " << seed;
      EXPECT_NE(outcome.failed_step, Step::kIdle) << "seed " << seed;
      // Stopped by the follower's error within about a millimetre of the
      // cups meeting the glass: the pushed module moves about a millimetre.
      // Let it settle, holding the arm, then look.
      for (int k = 0; k < 500; ++k) {
        script.Update();
        cell.Step();
        check.Observe(script.step(), 0);
      }
      mj_forward(cell.model(), cell.data());
      const cell::StackDisturbance d = cell::MeasureStackDisturbance(cell);
      std::printf("  %s\n", cell::Describe(d).c_str());
      EXPECT_LT(d.worst_displacement, 5.0 * kMm) << "seed " << seed;
    }
    EXPECT_EQ(check.non_finite_steps(), 0) << "seed " << seed;
    EXPECT_EQ(check.out_of_range_steps(), 0) << "seed " << seed;
  }
}

}  // namespace
