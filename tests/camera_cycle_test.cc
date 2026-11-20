// Checks the pick and place in camera mode (src/pick_place.h with
// SensingMode::kCamera): the look, each phase after it on a pallet set down
// well off nominal, whole cycles on several randomized pallets, what a
// calibration error does, the finder's refusals, and that the script reads
// no truth.
//
//   bazel test //:camera_cycle_test
//   bazel test --config=opt //:camera_cycle_test --test_output=all
//
// The scene's path comes from MODEL_PATH, and the script's source from
// SCRIPT_SOURCE (for the no-truth check), both set in BUILD.bazel.
//
// Time. Every picture costs about 2 s of computer time unoptimized (1 s to
// cast the rays, 1 s to find the module) and a cycle about 33 simulated
// seconds, which the unoptimized build runs at about real time. So cycles
// are simulated once per suite and shared: CameraCycle runs module_0 and
// then module_1 on one randomized pallet, pausing after each phase to
// measure; RandomizedRuns runs three more pallets and one with a
// calibration error, each up to the moment the module has been placed and
// measured.
//
// Every number measured is printed (--test_output=all); the bounds below
// say what each was measured at and how much margin they leave.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <numbers>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/camera_model.h"
#include "src/cell.h"
#include "src/conveyor.h"
#include "src/cycle_check.h"
#include "src/depth_camera.h"
#include "src/module_finder.h"
#include "src/pallet.h"
#include "src/pick_place.h"
#include "src/score.h"
#include "src/suction.h"

namespace {

using cell::Cell;
using cell::CycleCheck;
using cell::EstimateError;
using cell::PickPlace;
using cell::PickPlaceOutcome;
using Step = cell::PickPlaceStep;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;
constexpr double kMm = 1e-3;

// A cycle that has not ended after this much simulated time has hung.
constexpr double kCycleLimit = 90.0;  // s

// The pallet for the phase tests: seed 8 of the default randomization sets
// it down 22.4 mm away from the arm and 29.3 mm to the right, turned 2.81
// degrees: near the corner of what Reset can draw (30 mm, 3 degrees).
constexpr std::uint32_t kPhaseSeed = 8;

cell::PickPlaceOptions CameraOptions() {
  cell::PickPlaceOptions options;
  options.sensing = cell::SensingMode::kCamera;
  return options;
}

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

std::string Phases(const PickPlace& script) {
  std::string text;
  for (const cell::StepRecord& r : script.records()) {
    char line[600];
    std::snprintf(line, sizeof(line), "\n  %-14s %6.3f s  %s",
                  cell::StepName(r.step), r.duration(),
                  r.finished ? r.result.c_str() : "(running)");
    text += line;
  }
  return text;
}

std::string Describe(const EstimateError& e) {
  char text[200];
  std::snprintf(text, sizeof(text),
                "normal %+.3f mm, long side %+.3f mm, up the lean %+.3f mm, "
                "face %.4f deg, turn %+.4f deg",
                e.normal / kMm, e.long_side / kMm, e.up_lean / kMm,
                e.face_angle / kDegree, e.in_plane / kDegree);
  return text;
}

double BodyDistance(const Cell& cell, int a, int b) {
  const mjModel& m = *cell.model();
  double best = 1.0;
  for (int g = m.body_geomadr[a]; g < m.body_geomadr[a] + m.body_geomnum[a];
       ++g) {
    if (m.geom_contype[g] == 0 && m.geom_conaffinity[g] == 0) continue;
    for (int h = m.body_geomadr[b]; h < m.body_geomadr[b] + m.body_geomnum[b];
         ++h) {
      if (m.geom_contype[h] == 0 && m.geom_conaffinity[h] == 0) continue;
      best = std::min(best, mj_geomDistance(cell.model(),
                                            const_cast<mjData*>(cell.data()),
                                            g, h, 1.0, nullptr));
    }
  }
  return best;
}

// Truth: how many pixels of a picture taken now, from the true camera, see
// the arm or the tool where module i's glass truly is (inside the outline
// of its glass, shrunk by 3 cm as the finder's view check does). Zero means
// the arm is out of the way of that glass.
int ArmPixelsOnGlass(Cell& cell, int i) {
  const mjModel& model = *cell.model();
  mj_forward(cell.model(), cell.data());
  const cell::CameraModel camera = cell.DepthCamera();
  std::vector<int> geom;
  cell::RenderDepth(model, cell.data(), camera, {}, &geom);
  const Eigen::Isometry3d module = cell.TrueModulePose(i);
  const int base = mj_name2id(&model, mjOBJ_BODY, "arm_base");
  auto on_arm = [&](int g) {
    for (int b = model.geom_bodyid[g]; b > 0; b = model.body_parentid[b]) {
      if (b == base) return true;
    }
    return false;
  };
  int count = 0;
  for (int v = 0; v < camera.height; ++v) {
    for (int u = 0; u < camera.width; ++u) {
      const int g = geom[v * camera.width + u];
      if (g < 0 || !on_arm(g)) continue;
      // Where this pixel's ray meets the glass's plane, in the module's
      // frame.
      const Eigen::Vector3d ray = cell::RayThroughPixel(camera, u + 0.5, v + 0.5);
      const Eigen::Vector3d n = module.linear().col(2);
      const Eigen::Vector3d glass =
          module.translation() + 0.5 * cell::kModuleThickness * n;
      const double t = n.dot(glass - camera.pose.translation()) / n.dot(ray);
      if (t <= 0.0) continue;
      const Eigen::Vector3d local =
          module.inverse() * (camera.pose.translation() + t * ray);
      if (std::abs(local.x()) < 0.5 * cell::kModuleLength - 0.03 &&
          std::abs(local.y()) < 0.5 * cell::kModuleWidth - 0.03) {
        ++count;
      }
    }
  }
  return count;
}

// The placement the geometry predicts from an error of the estimate. The
// pick aims the tool at the estimate, so the module hangs in the tool off
// the planned grip by the estimate's error, undone: in the module's own
// frame, the true module is at delta^-1 from where the plan has it, delta
// being the estimate in the true module's frame. The place side lays the
// planned module exactly on the slot, so the true module lands at
// slot * delta^-1, the module's axes x and y along the slot's x and y
// times `sign` (Pickplace::place_x_sign) and its z up. The tilt goes when
// it lies down flat on the rails; what is left is the shift in the
// rails' plane and the turn about the vertical.
struct PredictedPlacement {
  double across = 0.0;  // m, slot x
  double along = 0.0;   // m, slot y
  double yaw = 0.0;     // rad
};
PredictedPlacement Predict(const Eigen::Isometry3d& delta, double sign) {
  const Eigen::Isometry3d undo = delta.inverse();
  PredictedPlacement p;
  p.across = sign * undo.translation().x();
  p.along = sign * undo.translation().y();
  p.yaw = std::atan2(undo.linear()(1, 0), undo.linear()(0, 0));
  return p;
}

// ===================================================== one pallet, by phase

struct PhaseLog {
  std::string error;
  std::uint32_t seed = 0;
  cell::TrueScene scene;

  // Look.
  bool look_done = false;
  std::string look_result;
  int pictures = 0;
  bool first_refused = false;   // the picture from arm_home
  std::string first_refusal;
  bool moved_to_look = false;   // a move to the look pose was planned
  std::optional<EstimateError> seen;
  cell::ViewCheck view;         // the finder's own check of the used picture
  int arm_pixels_at_look = -1;  // truth, at the look pose

  // Approach and touch, against module_0's TRUE glass.
  bool approach_done = false;
  double gap = 0.0, square = 0.0, off_centre = 0.0, approach_stack = 0.0;
  std::string touch_result;

  // Grip.
  bool grip_done = false;
  std::optional<int> held;
  int outcome_module = -1;
  // The module in the tool as the weld holds it, against what the plan
  // assumes (the design grip: glass grip_gap from the tip, centred, square).
  Eigen::Isometry3d grip_delta = Eigen::Isometry3d::Identity();

  // Lift-off.
  bool lift_off_done = false;
  double module_1_distance = 0.0, deck_distance = 0.0;
  cell::StackDisturbance lift_off_stack;
  std::string forbidden_pick;

  // Carry (to the end of lower).
  bool carry_done = false;
  double carry_tilt = 0.0, carry_yaw = 0.0, carry_off_slot = 0.0;
  double carry_height = 0.0;
  long saturated = 0;
  Eigen::VectorXd peak_holding;
  std::string forbidden_carry;
  double closest_module_stack = 1.0, closest_module_arm = 1.0,
         closest_module_conveyor = 1.0;

  // Set-down and release.
  bool release_done = false;
  bool still_held = true;
  cell::PlacementError at_release;
  double tool_to_module = 0.0;
  std::string forbidden_place;

  // Retreat, conveyor, look ahead, return.
  bool cycle_done = false;
  PickPlaceOutcome outcome;
  std::string phases;
  std::optional<cell::PlacementError> placed;
  int arm_pixels_at_retreat = -1;  // truth, on module_1's glass
  bool look_ahead_found = false;
  cell::ViewCheck look_ahead_view;
  int steps_after = 0;
  double module_0_moved_y = 0.0;
  double module_0_y_placed = 0.0;
  cell::PlacementError after_step;
  long forbidden = 0;
  std::string forbidden_list;
  std::string report;
  double place_x_sign = 1.0;

  // module_1, from the look-ahead picture.
  bool second_done = false;
  PickPlaceOutcome second;
  std::string second_phases;
  double second_look_time = -1.0;
  bool second_moved_to_look = true;
  std::optional<EstimateError> second_seen;
  std::optional<cell::PlacementError> second_placed;
  long second_forbidden = 0;
  double first_cycle_time = 0.0, second_cycle_time = 0.0;
};

class CameraCycle : public ::testing::Test {
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
  static const PhaseLog& log() { return *log_; }
  static PhaseLog* log_;
};

PhaseLog* CameraCycle::log_ = nullptr;

void CameraCycle::SetUpTestSuite() {
  log_ = new PhaseLog;
  PhaseLog& L = *log_;
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  if (!loaded) {
    L.error = error;
    return;
  }
  Cell& cell = *loaded;
  L.seed = kPhaseSeed;
  cell.Reset(kPhaseSeed, cell::Randomization{});
  L.scene = cell.true_scene();
  const mjModel& model = *cell.model();
  const int m0 = cell.pallet().modules[0];
  const int m1 = cell.pallet().modules[1];
  const int deck = mj_name2id(&model, mjOBJ_GEOM, "pallet_deck");
  const int tool = cell.arm().tool_body;
  const cell::PickPlaceOptions options = CameraOptions();

  PickPlace script(cell, options);
  script.set_placed_hook([&](int module, int slot) {
    mj_forward(cell.model(), cell.data());
    if (module == 0) {
      L.placed = cell::MeasurePlacement(cell, module, slot);
      L.module_0_y_placed = cell.TrueModulePose(0).translation().y();
    } else if (module == 1) {
      L.second_placed = cell::MeasurePlacement(cell, module, slot);
    }
  });
  CycleCheck check(cell, {.clearance_every = 10});
  auto fail = [&](const std::string& where) {
    L.error = where + ": " + script.outcome().reason + Phases(script);
  };

  // ---- look
  script.set_pause_after(Step::kLook);
  script.Start(0);
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("look did not end");
  }
  L.look_done = true;
  L.look_result = script.records().back().result;
  L.pictures = static_cast<int>(script.looks().size());
  if (!script.looks().empty() && !script.looks().front().estimate) {
    L.first_refused = true;
    L.first_refusal = cell::FinderFailureName(
        script.looks().front().diagnostics.failure);
  }
  L.moved_to_look = !script.Waypoints(Step::kLook)->empty();
  if (script.estimate()) {
    L.seen = cell::EstimateErrorOf(script.estimate()->pose,
                                   cell.TrueModulePose(0));
    L.view = script.estimate()->diagnostics.view;
  }
  L.arm_pixels_at_look = ArmPixelsOnGlass(cell, 0);

  // ---- approach and touch
  script.set_pause_after(Step::kTouchGlass);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("approach did not end");
  }
  L.approach_done = true;
  L.touch_result = script.records().back().result;
  Eigen::Isometry3d module_in_tip;
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
    module_in_tip = tip.inverse() * cell.TrueModulePose(0);
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
  L.outcome_module = script.outcome().module;
  if (const std::optional<cell::Pose> weld =
          cell.suction().HeldPoseInTipAtGrip()) {
    // What the plan assumes: module +x along the tool's X, glass facing the
    // tool, centre grip_gap + half a thickness along the tool's +Z.
    Eigen::Isometry3d assumed = Eigen::Isometry3d::Identity();
    assumed.linear() << 1, 0, 0, 0, -1, 0, 0, 0, -1;
    assumed.translation() = Eigen::Vector3d(
        0, 0, options.grip_gap + 0.5 * cell::kModuleThickness);
    Eigen::Isometry3d actual = Eigen::Isometry3d::Identity();
    actual.translation() = weld->position;
    actual.linear() = weld->rotation.toRotationMatrix();
    L.grip_delta = assumed.inverse() * actual;
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
    L.carry_yaw = std::acos(std::min(
        1.0, std::abs(long_side.normalized().dot(slot.linear().col(0)))));
    const Eigen::Vector3d d = module.translation() - slot.translation();
    L.carry_off_slot = Eigen::Vector2d(d.x(), d.y()).norm();
    L.carry_height = d.z();
    L.saturated = check.saturated_steps();
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

  // ---- retreat and conveyor: pause before the look ahead to see what the
  // camera will see from there; then the look ahead and the return.
  script.set_pause_after(Step::kConveyor);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.paused()) {
    return fail("retreat did not end");
  }
  L.arm_pixels_at_retreat = ArmPixelsOnGlass(cell, 1);
  script.set_pause_after(std::nullopt);
  script.Resume();
  if (!RunScript(cell, script, &check, 0) || !script.Done()) {
    return fail("the cycle did not end");
  }
  L.outcome = script.outcome();
  L.cycle_done = L.outcome.status == PickPlaceOutcome::Status::kPlaced;
  L.first_cycle_time = L.outcome.end_time - L.outcome.start_time;
  L.phases = Phases(script);
  if (!script.looks().empty() &&
      script.looks().back().step == Step::kLookAhead) {
    L.look_ahead_found = script.looks().back().estimate.has_value();
    L.look_ahead_view = script.looks().back().diagnostics.view;
  }
  RunUntilConveyorStops(cell, script, &check, 0);
  L.steps_after = cell.conveyor().steps();
  L.module_0_moved_y =
      cell.TrueModulePose(0).translation().y() - L.module_0_y_placed;
  L.after_step = cell::MeasurePlacement(cell, 0, 0);
  L.place_x_sign = script.place_x_sign();

  auto forbidden_in = [&](Step first, Step last) {
    std::string text;
    for (const cell::ForbiddenContact& f : check.forbidden()) {
      if (static_cast<int>(f.step) >= static_cast<int>(first) &&
          static_cast<int>(f.step) <= static_cast<int>(last)) {
        text += std::string("\n  ") + cell::StepName(f.step) + ": " + f.between;
      }
    }
    return text;
  };
  L.forbidden_pick = forbidden_in(Step::kLook, Step::kPullBack);
  L.forbidden_carry = forbidden_in(Step::kClear, Step::kLower);
  L.forbidden_place = forbidden_in(Step::kSetDown, Step::kRelease);
  L.forbidden = check.forbidden_count();
  L.forbidden_list = forbidden_in(Step::kIdle, Step::kFailed);
  L.report = check.Report(true);
  L.closest_module_stack =
      check.closest(cell::Clearance::kModuleStack).distance;
  L.closest_module_arm = check.closest(cell::Clearance::kModuleArm).distance;
  L.closest_module_conveyor =
      check.closest(cell::Clearance::kModuleConveyor).distance;

  // ---- module_1, from the picture taken at the end of module_0's retreat
  CycleCheck second_check(cell, {.clearance_every = 0});
  script.Start(1);
  RunScript(cell, script, &second_check, 1);
  L.second = script.outcome();
  L.second_done = L.second.status == PickPlaceOutcome::Status::kPlaced;
  L.second_phases = Phases(script);
  L.second_cycle_time = L.second.end_time - L.second.start_time;
  if (!script.records().empty() && script.records().front().step == Step::kLook) {
    L.second_look_time = script.records().front().duration();
  }
  L.second_moved_to_look = !script.Waypoints(Step::kLook)->empty();
  L.second_forbidden = second_check.forbidden_count();

  std::printf("pallet seed %u: %+.1f, %+.1f mm, %+.2f deg\n", L.seed,
              L.scene.pallet_x / kMm, L.scene.pallet_y / kMm,
              L.scene.pallet_yaw / kDegree);
  std::printf("module_0's cycle, %.2f s:%s\n%s", L.first_cycle_time,
              L.phases.c_str(), L.report.c_str());
  std::printf("module_1's cycle, %.2f s:%s\n", L.second_cycle_time,
              L.second_phases.c_str());
}

// ---- look: the first picture, from arm_home, is refused (the tool hangs
// in front of the glass); the arm goes to the look pose, out of the view,
// and the second picture gives the module within bounds of the truth.
TEST_F(CameraCycle, LookRetakesFromTheLookPoseAndSeesTheModule) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.look_done);
  std::printf("look: %d pictures, first refused (%s), moved to the look "
              "pose: %s; view: %d of %d pixels on the glass; arm pixels on "
              "the true glass at the look pose: %d; %s\n",
              L.pictures, L.first_refusal.c_str(), L.moved_to_look ? "yes" : "no",
              L.view.on_glass, L.view.pixels, L.arm_pixels_at_look,
              L.look_result.c_str());
  EXPECT_EQ(L.pictures, 2) << "look: pictures taken";
  EXPECT_TRUE(L.first_refused) << "look: the picture from arm_home was used";
  EXPECT_TRUE(L.moved_to_look) << "look: no move to the look pose";
  ASSERT_TRUE(L.seen.has_value()) << "look: no estimate";
  // The arm is out of the picture: the finder's own check found every
  // pixel of the glass reading glass, and the truth agrees.
  EXPECT_EQ(L.view.in_front, 0) << "look: something in front of the glass";
  EXPECT_EQ(L.arm_pixels_at_look, 0) << "look: the arm is in the view";
  const EstimateError& e = *L.seen;
  std::printf("look: estimate against the truth: %s\n", Describe(e).c_str());
  // Measured over 20 randomized pallets with noise: at most 0.02 mm along
  // the normal, 0.68 mm along the long side, 0.13 mm up the lean, 0.003
  // degrees of face and 0.012 of turn; the finder's own tests saw 0.78 mm
  // at worst. Bounds about twice the worst.
  EXPECT_LT(std::abs(e.normal), 0.5 * kMm) << "look: estimate along the normal";
  EXPECT_LT(std::abs(e.long_side), 1.5 * kMm) << "look: estimate along the long side";
  EXPECT_LT(std::abs(e.up_lean), 0.5 * kMm) << "look: estimate up the lean";
  EXPECT_LT(e.face_angle, 0.02 * kDegree) << "look: estimate's face direction";
  EXPECT_LT(std::abs(e.in_plane), 0.03 * kDegree) << "look: estimate's turn";
}

// ---- approach and touch: the tool stops at the planned gap from the TRUE
// glass, square to it, having touched nothing.
TEST_F(CameraCycle, ApproachStopsAtTheGapFromTheTrueGlass) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.approach_done);
  std::printf("approach: gap %.3f mm from the true glass (planned 2.0), off "
              "centre %.3f mm, square within %.4f deg, stack moved %.4f mm; "
              "touch glass: %s\n",
              L.gap / kMm, L.off_centre / kMm, L.square / kDegree,
              L.approach_stack / kMm, L.touch_result.c_str());
  // The gap is off by the estimate's error along the normal (0.02 mm at
  // worst) and the arm's tracking (0.1 mm): measured within 0.1 mm.
  EXPECT_NEAR(L.gap, 2.0 * kMm, 0.5 * kMm) << "approach: gap";
  // Off the centre by the estimate's in-plane error: under 0.7 mm.
  EXPECT_LT(L.off_centre, 1.5 * kMm) << "approach: off the glass centre";
  EXPECT_LT(L.square, 0.05 * kDegree) << "approach: not square";
  EXPECT_LT(L.approach_stack, 0.01 * kMm) << "approach: the stack moved";
  EXPECT_EQ(L.forbidden_pick, "") << "approach/lift-off: forbidden contacts";
}

// ---- grip: holds module_0, as the approach left it: the weld's pose is
// the planned grip off by the estimate's error.
TEST_F(CameraCycle, GripHoldsModule0) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.grip_done);
  ASSERT_TRUE(L.held.has_value()) << "grip: nothing held";
  EXPECT_EQ(*L.held, 0) << "grip: held the wrong module";
  EXPECT_EQ(L.outcome_module, 0) << "grip: the script thinks it holds another";
  const Eigen::Vector3d t = L.grip_delta.translation();
  const double turn = Eigen::AngleAxisd(L.grip_delta.linear()).angle();
  std::printf("grip: module_0 held %.3f, %.3f, %.3f mm and %.4f deg off the "
              "planned grip (the estimate's error, undone)\n",
              t.x() / kMm, t.y() / kMm, t.z() / kMm, turn / kDegree);
  // What the place side will lay down wrong: the estimate's error plus the
  // approach's; measured under a millimetre.
  EXPECT_LT(t.norm(), 1.5 * kMm) << "grip: held off the plan";
  EXPECT_LT(turn, 0.05 * kDegree) << "grip: held turned off the plan";
}

// ---- lift-off: module_0 clear of module_1 and the deck, the stack
// standing, on the turned pallet as at nominal.
TEST_F(CameraCycle, LiftOffClearsModule0OfTheTurnedStack) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.lift_off_done);
  const cell::PickPlaceOptions options;
  const double planned_back = options.pull_back + options.lift_back;
  std::printf("lift-off: module_0 %.2f mm from module_1 (planned %.0f), %.2f "
              "mm from the deck (planned %.0f); %s\n",
              L.module_1_distance / kMm, planned_back / kMm,
              L.deck_distance / kMm, options.lift / kMm,
              cell::Describe(L.lift_off_stack).c_str());
  // The same as at nominal (53.7 and 29.5 mm): every pose of the pick side
  // follows the estimate.
  EXPECT_NEAR(L.module_1_distance, planned_back, 1.0 * kMm)
      << "lift-off: clearance from module_1";
  EXPECT_NEAR(L.deck_distance, options.lift, 1.0 * kMm)
      << "lift-off: clearance from the deck";
  EXPECT_LT(L.lift_off_stack.worst_displacement, 0.25 * kMm)
      << "lift-off: the stack moved";
  EXPECT_LT(L.lift_off_stack.worst_rotation, 0.02 * kDegree)
      << "lift-off: the stack turned";
}

// ---- carry: on this pallet too, nothing touched on the way, no motor at
// its limit, and the module flat over its slot at the end.
TEST_F(CameraCycle, CarryClearsEverythingOnTheTurnedPallet) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.carry_done);
  std::printf("carry: tilt %.4f deg, long side %.4f deg from across, centre "
              "%.3f mm from the slot's, %.2f mm above where it will lie; "
              "closest: held module to stack %.1f mm, to arm %.1f mm, to "
              "conveyor %.1f mm\n",
              L.carry_tilt / kDegree, L.carry_yaw / kDegree,
              L.carry_off_slot / kMm, L.carry_height / kMm,
              L.closest_module_stack / kMm, L.closest_module_arm / kMm,
              L.closest_module_conveyor / kMm);
  EXPECT_EQ(L.forbidden_carry, "") << "carry: forbidden contacts";
  EXPECT_EQ(L.saturated, 0) << "carry: a motor saturated";
  for (int j = 0; j < cell::Arm::kJoints; ++j) {
    EXPECT_LT(L.peak_holding[j], 0.9) << "carry: joint " << j + 1;
  }
  // The module hangs in the tool off the plan by the estimate's error
  // (under a millimetre and 0.05 degrees), so it is over the slot within
  // that, not exactly.
  EXPECT_LT(L.carry_tilt, 0.1 * kDegree) << "carry: not flat";
  EXPECT_LT(L.carry_yaw, 0.1 * kDegree) << "carry: long side not across";
  EXPECT_LT(L.carry_off_slot, 1.5 * kMm) << "carry: not over the slot";
  EXPECT_NEAR(L.carry_height, 0.041, 1.0 * kMm) << "carry: height";
  // The tightest clearances measured over 20 randomized pallets: the bottom
  // edge 29.5 mm off the deck just after the lift (the lift itself), the
  // module 159 mm from the upper arm at the quarter turn, 101 mm from the
  // rails.
  EXPECT_GT(L.closest_module_stack, 0.025);
  EXPECT_GT(L.closest_module_arm, 0.10);
  EXPECT_GT(L.closest_module_conveyor, 0.07);
}

// ---- set-down and release: on the rails, let go, the tool off it.
TEST_F(CameraCycle, SetDownLaysModule0OnItsRails) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.release_done);
  std::printf("set-down: %s; tool %.2f mm from it\n",
              cell::Describe(L.at_release).c_str(), L.tool_to_module / kMm);
  EXPECT_FALSE(L.still_held) << "set-down: still held";
  EXPECT_LT(std::abs(L.at_release.along_tube), 1.0 * kMm) << "set-down: along";
  EXPECT_LT(std::abs(L.at_release.across_tube), 1.0 * kMm) << "set-down: across";
  EXPECT_LT(std::abs(L.at_release.height), 0.3 * kMm) << "set-down: height";
  EXPECT_LT(std::abs(L.at_release.yaw), 0.05 * kDegree) << "set-down: yaw";
  EXPECT_LT(L.at_release.tilt, 0.05 * kDegree) << "set-down: tilt";
  EXPECT_GT(L.tool_to_module, 0.5 * kMm) << "set-down: tool still on it";
  EXPECT_EQ(L.forbidden_place, "") << "set-down: forbidden contacts";
}

// ---- retreat, conveyor and look ahead: the arm at the end of the retreat
// is out of the camera's view of the next module, the picture there finds
// it, the tube moves on with module_0 aboard.
TEST_F(CameraCycle, LookAheadFromTheRetreatThenTheTubeMovesOn) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.cycle_done) << L.outcome.reason << L.phases;
  ASSERT_TRUE(L.placed.has_value());
  std::printf("retreat: arm pixels on module_1's true glass %d; look ahead "
              "found it: %s (%d of %d pixels on glass); tube %d steps on, "
              "module_0 moved %.2f mm along Y; after the step: %s\n",
              L.arm_pixels_at_retreat, L.look_ahead_found ? "yes" : "no",
              L.look_ahead_view.on_glass, L.look_ahead_view.pixels,
              L.steps_after, L.module_0_moved_y / kMm,
              cell::Describe(L.after_step).c_str());
  EXPECT_EQ(L.arm_pixels_at_retreat, 0) << "look ahead: the arm is in the view";
  EXPECT_TRUE(L.look_ahead_found) << "look ahead: no module found";
  EXPECT_EQ(L.look_ahead_view.in_front, 0) << "look ahead: something in front";
  EXPECT_EQ(L.steps_after, 1) << "conveyor: steps";
  // The tube stops within its stopping error (2 mm standard deviation) of
  // the nominal step.
  EXPECT_NEAR(L.module_0_moved_y, -cell::Conveyor::kStepLength, 8.0 * kMm)
      << "conveyor: module_0 did not ride along";
  EXPECT_TRUE(L.after_step.at_rest) << "conveyor: module_0 still moving";
  EXPECT_NEAR(L.after_step.along_tube, L.placed->along_tube, 0.2 * kMm)
      << "conveyor: module_0 slid along the tube";
  EXPECT_NEAR(L.after_step.across_tube, L.placed->across_tube, 0.2 * kMm)
      << "conveyor: module_0 slid across the tube";
}

// ---- end to end on this pallet, and module_1 after it: the second look
// uses the look-ahead picture with no arm motion.
TEST_F(CameraCycle, WholeCycleAndTheNextFromTheLookAheadPicture) {
  const PhaseLog& L = log();
  ASSERT_TRUE(L.cycle_done) << L.outcome.reason << L.phases;
  ASSERT_TRUE(L.placed.has_value());
  std::printf("end to end: %s; %ld forbidden contacts; cycle %.2f s\n",
              cell::Describe(*L.placed).c_str(), L.forbidden,
              L.first_cycle_time);
  EXPECT_LT(std::abs(L.placed->along_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(L.placed->across_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(L.placed->height), 0.3 * kMm);
  EXPECT_LT(std::abs(L.placed->yaw), 0.05 * kDegree);
  EXPECT_LT(L.placed->tilt, 0.05 * kDegree);
  EXPECT_TRUE(L.placed->at_rest);
  EXPECT_EQ(L.forbidden, 0) << L.forbidden_list;

  ASSERT_TRUE(L.second_done) << L.second.reason << L.second_phases;
  ASSERT_TRUE(L.second_placed.has_value());
  std::printf("second: look took %.3f s, moved to the look pose: %s; %s; "
              "cycle %.2f s\n",
              L.second_look_time, L.second_moved_to_look ? "yes" : "no",
              cell::Describe(*L.second_placed).c_str(), L.second_cycle_time);
  // The picture was already taken: no time, no motion.
  EXPECT_EQ(L.second_look_time, 0.0);
  EXPECT_FALSE(L.second_moved_to_look);
  EXPECT_EQ(L.second_placed->slot, 1);
  EXPECT_LT(std::abs(L.second_placed->along_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(L.second_placed->across_tube), 1.0 * kMm);
  EXPECT_LT(std::abs(L.second_placed->yaw), 0.05 * kDegree);
  EXPECT_EQ(L.second_forbidden, 0);
}

// ================================================ several pallets, whole

// One cycle of module_0 on one pallet, up to its placement and the scoring.
struct CycleRun {
  std::uint32_t seed = 0;
  cell::TrueScene scene;
  PickPlaceOutcome outcome;
  std::string phases;
  std::optional<cell::PlacementError> placed;
  std::optional<EstimateError> seen;
  Eigen::Isometry3d estimate = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();  // module_0
  Eigen::Isometry3d camera = Eigen::Isometry3d::Identity();  // true pose
  double place_x_sign = 1.0;
  long forbidden = 0;
  long non_finite = 0;
};

CycleRun RunOnce(Cell& cell, std::uint32_t seed,
            const cell::Randomization& randomization) {
  CycleRun run;
  run.seed = seed;
  cell.Reset(seed, randomization);
  run.scene = cell.true_scene();
  run.camera = cell.DepthCamera().pose;
  PickPlace script(cell, CameraOptions());
  script.set_placed_hook([&](int module, int slot) {
    mj_forward(cell.model(), cell.data());
    run.placed = cell::MeasurePlacement(cell, module, slot);
  });
  // Stop once the module has been placed and measured: the return and the
  // next picture are the phase tests' business.
  script.set_pause_after(Step::kConveyor);
  CycleCheck check(cell, {.clearance_every = 0});
  script.Start(0);
  RunScript(cell, script, &check, 0);
  run.outcome = script.outcome();
  run.phases = Phases(script);
  if (script.estimate()) {
    run.estimate = script.estimate()->pose;
    run.truth = cell.true_stack_baseline()[0];
    run.seen = cell::EstimateErrorOf(run.estimate, run.truth);
  }
  run.place_x_sign = script.place_x_sign();
  run.forbidden = check.forbidden_count();
  run.non_finite = check.non_finite_steps();
  return run;
}

struct RunsLog {
  std::string error;
  std::vector<CycleRun> runs;          // no calibration error
  std::optional<CycleRun> calibrated;  // kCalibrationSeed with one
  std::optional<CycleRun> baseline;    // the same seed without
};

// The calibration error for the calibration test: 1 mm and 0.05 degrees,
// in directions drawn for the seed.
constexpr double kCalibrationShift = 0.001;               // m
constexpr double kCalibrationTurn = 0.05 * kPi / 180.0;   // rad
constexpr std::uint32_t kCalibrationSeed = 2;

class RandomizedRuns : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    log_ = new RunsLog;
    std::string error;
    std::optional<Cell> loaded = LoadCell(&error);
    if (!loaded) {
      log_->error = error;
      return;
    }
    // Seeds 2, 5 and 7: turned -2.8, +2.2 and +1.7 degrees, shifted up to
    // 27 mm, toward the arm and away. (Seed 8 is CameraCycle's.)
    for (std::uint32_t seed : {2u, 5u, 7u}) {
      log_->runs.push_back(RunOnce(*loaded, seed, cell::Randomization{}));
      if (seed == kCalibrationSeed) log_->baseline = log_->runs.back();
    }
    cell::Randomization off;
    off.camera_shift = kCalibrationShift;
    off.camera_turn = kCalibrationTurn;
    log_->calibrated = RunOnce(*loaded, kCalibrationSeed, off);
  }
  static void TearDownTestSuite() {
    delete log_;
    log_ = nullptr;
  }
  void SetUp() override {
    ASSERT_NE(log_, nullptr);
    ASSERT_TRUE(log_->error.empty()) << log_->error;
  }
  static RunsLog* log_;
};

RunsLog* RandomizedRuns::log_ = nullptr;

// Placed every time, within bounds stated from measurement.
TEST_F(RandomizedRuns, PlacedEveryTimeWithinBounds) {
  for (const CycleRun& run : log_->runs) {
    SCOPED_TRACE("seed " + std::to_string(run.seed));
    ASSERT_EQ(run.outcome.status, PickPlaceOutcome::Status::kRunning)
        << "(paused after the conveyor step) " << run.outcome.reason
        << run.phases;
    ASSERT_TRUE(run.placed.has_value()) << run.phases;
    ASSERT_TRUE(run.seen.has_value());
    const cell::PlacementError& e = *run.placed;
    std::printf("seed %u (pallet %+.1f, %+.1f mm, %+.2f deg): %s; seen: %s\n",
                run.seed, run.scene.pallet_x / kMm, run.scene.pallet_y / kMm,
                run.scene.pallet_yaw / kDegree, cell::Describe(e).c_str(),
                Describe(*run.seen).c_str());
    // Measured over 20 randomized pallets with noise: worst along 0.13 mm,
    // across 0.60 mm (rms 0.27), height 0, yaw 0.012 degrees, tilt 0. The
    // across error is the estimate's error along the long side, undone
    // (see the calibration test); the rest is a tenth of a millimetre.
    EXPECT_LT(std::abs(e.along_tube), 1.0 * kMm);
    EXPECT_LT(std::abs(e.across_tube), 1.5 * kMm);
    EXPECT_LT(std::abs(e.height), 0.3 * kMm);
    EXPECT_LT(std::abs(e.yaw), 0.05 * kDegree);
    EXPECT_LT(e.tilt, 0.05 * kDegree);
    EXPECT_TRUE(e.at_rest);
    EXPECT_EQ(run.forbidden, 0);
    EXPECT_EQ(run.non_finite, 0);
    // What dominates: the placement is the estimate's error undone, to
    // within a few tenths of a millimetre (the approach's and the place's
    // own share).
    const PredictedPlacement p =
        Predict(run.truth.inverse() * run.estimate, run.place_x_sign);
    EXPECT_NEAR(e.across_tube, p.across, 0.3 * kMm);
    EXPECT_NEAR(e.along_tube, p.along, 0.3 * kMm);
  }
}

// A calibration error moves every point the camera sees by one rigid motion
// E, so the estimate is E applied to the module's true pose: the pick takes
// it there, the module hangs in the tool off the plan by E undone, and the
// place, blind to it, lays it down off by just that. Predicted from E alone
// (the recorded calibration error and the camera's true pose), the
// placement moves from the run without the error by the in-plane part of
// E undone, expressed in the module's frame.
TEST_F(RandomizedRuns, CalibrationErrorShiftsThePlacementAsPredicted) {
  ASSERT_TRUE(log_->calibrated && log_->baseline);
  const CycleRun& with = *log_->calibrated;
  const CycleRun& without = *log_->baseline;
  ASSERT_TRUE(with.placed && without.placed) << with.phases;
  // E: the believed camera is the true one moved by D in its own frame, so
  // a point seen at c in the camera's frame is put at P D c instead of P c:
  // E = P D P^-1.
  Eigen::Isometry3d d = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d turn = with.scene.camera_turn;
  if (turn.norm() > 0.0) {
    d.linear() = Eigen::AngleAxisd(turn.norm(), turn.normalized())
                     .toRotationMatrix();
  }
  d.translation() = with.scene.camera_shift;
  const Eigen::Isometry3d e = with.camera * d * with.camera.inverse();
  // In the true module's frame.
  const Eigen::Isometry3d delta = with.truth.inverse() * e * with.truth;
  const PredictedPlacement p = Predict(delta, with.place_x_sign);
  const double shift_across = with.placed->across_tube - without.placed->across_tube;
  const double shift_along = with.placed->along_tube - without.placed->along_tube;
  const double shift_yaw = with.placed->yaw - without.placed->yaw;
  std::printf("calibration off by %.2f mm and %.3f deg: estimate moved %s "
              "(%s without); placement moved across %+.3f mm (predicted "
              "%+.3f), along %+.3f mm (predicted %+.3f), yaw %+.4f deg "
              "(predicted %+.4f)\n",
              with.scene.camera_shift.norm() / kMm, turn.norm() / kDegree,
              Describe(*with.seen).c_str(), Describe(*without.seen).c_str(),
              shift_across / kMm, p.across / kMm, shift_along / kMm,
              p.along / kMm, shift_yaw / kDegree, p.yaw / kDegree);
  // The prediction leaves out what the error along the glass's normal does:
  // the tool meets the glass early or late by it, and the module, held
  // that much nearer or further than planned, lands on the rails or drops
  // to them by it, and with the estimate's tilt, rocks flat on landing.
  // Measured: within a few tenths of a millimetre.
  EXPECT_NEAR(shift_across, p.across, 0.5 * kMm);
  EXPECT_NEAR(shift_along, p.along, 0.5 * kMm);
  EXPECT_NEAR(shift_yaw, p.yaw, 0.01 * kDegree);
  // And it is a real shift: the error is millimetres where the run
  // without it was tenths.
  EXPECT_GT(std::hypot(p.across, p.along), 1.0 * kMm);
}

// ================================================= refusals and blindness

// An empty pallet: the first picture (from arm_home) is refused, the arm
// goes to the look pose and looks again, that is refused too, and the
// cycle ends with the finder's reason, the arm holding still.
TEST(CameraRefusals, EmptyPalletEndsWithTheFindersReasonArmHolding) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  cell.Reset(1, cell::Randomization{});
  for (int i = 0; i < cell.module_count(); ++i) cell.ParkModule(i);
  PickPlace script(cell, CameraOptions());
  script.Start(0);
  RunScript(cell, script, nullptr, 0);
  ASSERT_TRUE(script.Done());
  const PickPlaceOutcome& outcome = script.outcome();
  std::printf("empty pallet: %s in %s; %zu pictures%s\n",
              outcome.reason.c_str(), cell::StepName(outcome.failed_step),
              script.looks().size(), Phases(script).c_str());
  EXPECT_EQ(outcome.status, PickPlaceOutcome::Status::kFailed);
  EXPECT_EQ(outcome.failed_step, Step::kLook);
  EXPECT_NE(outcome.reason.find("the finder refused"), std::string::npos);
  EXPECT_EQ(script.looks().size(), 2u);  // from home, then the look pose
  // Holding still where it stopped (the look pose): a second later it has
  // not moved.
  const Eigen::VectorXd at_failure = cell::JointAngles(cell.arm(), *cell.data());
  EXPECT_LT((at_failure - script.look_pose()).cwiseAbs().maxCoeff(), 2e-3);
  for (int k = 0; k < 500; ++k) {
    script.Update();
    cell.Step();
  }
  const Eigen::VectorXd later = cell::JointAngles(cell.arm(), *cell.data());
  EXPECT_LT((later - at_failure).cwiseAbs().maxCoeff(), 1e-4);
  EXPECT_LT(cell::JointSpeeds(cell.arm(), *cell.data()).cwiseAbs().maxCoeff(),
            1e-3);
}

// The script reads no truth: its source names no True... accessor, no
// body or geom pose out of mjData, no contact, no suction pose. Its only
// inputs beyond the joint angles, the conveyor's encoder and Grip's result
// are the camera's picture and calibration.
TEST(NoTruth, TheScriptsSourceReadsOnlyWhatACellKnows) {
  const char* path = std::getenv("SCRIPT_SOURCE");
  ASSERT_NE(path, nullptr) << "SCRIPT_SOURCE is not set";
  std::ifstream file(path);
  ASSERT_TRUE(file) << "cannot read " << path;
  std::stringstream all;
  all << file.rdbuf();
  // Code only: comments say what the file must not read, by name.
  std::string code;
  std::istringstream lines(all.str());
  for (std::string line; std::getline(lines, line);) {
    code += line.substr(0, line.find("//")) + "\n";
  }
  for (const char* forbidden :
       {"True", "true_", "xpos", "xquat", "xmat", "ModulePose", "PalletPose",
        "ModuleGlass", "suction()", "HeldPose", "contact", "DepthCamera()",
        "module_state", "qpos", "TipPose("}) {
    EXPECT_EQ(code.find(forbidden), std::string::npos)
        << "pick_place.cc uses " << forbidden;
  }
  // And it does use the two camera inputs.
  EXPECT_NE(code.find("TakeDepthImage"), std::string::npos);
  EXPECT_NE(code.find("CalibratedCamera"), std::string::npos);
}

}  // namespace
