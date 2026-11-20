// The check after pickup wired into the cycle (PickPlaceOptions::
// check_after_pickup), one action at a time, so that a failure names the
// phase:
//
//   feasible   the check pose, holding a module at the design grip and at
//              grips a few millimetres and a degree off, clear of the
//              planner's planning margin at every stack depth and every slot
//              state, with the stack's estimate off by up to the pallet's
//              tolerance; and what the via costs in carry time against the
//              single-leg planned carry;
//   via        the script's carry in two legs through the check pose, with
//              the dwell at its end;
//   picture    at the end of the dwell, the arm at the check pose, the
//              module in view, the finder's answer, the expected pose from
//              the joint angles the arm reported;
//   offset     the measured grip against the true one (a known offset put
//              into the grip), within 0.5 mm and 0.03 degrees, with noise;
//   place      lower and set down corrected: the MODULE on its release pose
//              and the placement scored inside a tenth of the offset;
//   refusals   a refusal twice: placed uncorrected and "unchecked"; a slip
//              (kFarFromExpected): the cycle fails at the check, no retake;
//   options    the check refused with the scripted carry;
//   cycles     whole camera-mode cycles with the pallet camera's calibration
//              off by 1 mm and 0.05 degrees, with and without the check, on
//              the same seeds: placement with the check well inside
//              placement without it;
//   stage 6    the record of a blocked set-down (kLetGo, unconfirmed), one
//              rule for records across a Reset, and set_pause_after refusing
//              a step the mode skips.
//
// The tests may read the truth (True..., the suction weld's pose) to judge
// the script; the script does not.
//
//   bazel test --config=opt //:check_cycle_test --test_output=all

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
#include "src/arm_follower.h"
#include "src/cell.h"
#include "src/check_rig.h"
#include "src/collision.h"
#include "src/conveyor.h"
#include "src/cycle_check.h"
#include "src/held_module_finder.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/motion_planner.h"
#include "src/pallet.h"
#include "src/pick_place.h"
#include "src/score.h"
#include "src/suction.h"

namespace {

using cell::CarryMode;
using cell::CarrySceneInput;
using cell::Cell;
using cell::CheckReport;
using cell::CollisionChecker;
using cell::Conveyor;
using cell::CycleCheck;
using cell::KnownModule;
using cell::ModulePlace;
using cell::MotionPlanOptions;
using cell::MotionPlanReport;
using cell::PickPlace;
using cell::PickPlaceOutcome;
using cell::PlanScene;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Step = cell::PickPlaceStep;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;
constexpr double kMm = 1e-3;
constexpr int kModules = 30;
constexpr double kCycleLimit = 90.0;  // s of simulation per cycle

std::optional<Cell> LoadCell(std::string* error) {
  const char* path = std::getenv("MODEL_PATH");
  if (path == nullptr) {
    *error = "MODEL_PATH is not set";
    return std::nullopt;
  }
  return Cell::Load(path, error);
}

cell::PickPlaceOptions CheckOptions(cell::SensingMode sensing) {
  cell::PickPlaceOptions options;
  options.sensing = sensing;
  options.carry = CarryMode::kPlanned;
  options.check_after_pickup = true;
  return options;
}

double PlanningMargin(const MotionPlanOptions& o) {
  return o.required_margin + 0.5 * o.edge_resolution + o.corner_allowance +
         o.tracking_allowance;
}
double PlanningSelfMargin(const MotionPlanOptions& o) {
  return o.required_self_margin + 0.5 * o.edge_resolution +
         o.corner_allowance + o.tracking_allowance;
}

double Distance(const Isometry3d& a, const Isometry3d& b) {
  return (a.translation() - b.translation()).norm();
}
double Angle(const Isometry3d& a, const Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}

Isometry3d FromPose(const cell::Pose& pose) {
  Isometry3d result = Isometry3d::Identity();
  result.translation() = pose.position;
  result.linear() = pose.rotation.normalized().toRotationMatrix();
  return result;
}

std::string Phases(const PickPlace& script) {
  std::string text;
  for (const cell::StepRecord& r : script.records()) {
    char line[900];
    std::snprintf(line, sizeof(line), "\n  %-14s %6.3f s  %s",
                  cell::StepName(r.step), r.duration(), r.result.c_str());
    text += line;
  }
  return text;
}

// Runs the script until Done (or the limit), with `check` watching if given.
void RunCycle(Cell& cell, PickPlace& script, CycleCheck* check = nullptr) {
  const double began = cell.data()->time;
  while (!script.Done() && cell.data()->time - began < kCycleLimit) {
    script.Update();
    cell.Step();
    if (check != nullptr) check->Observe(script.step(), script.outcome().module);
  }
}

// What a run has done by the time module k is picked with slot `slot` in
// front of the arm: the `slot` modules before k (as many as there are) in
// slots 0 .. slot-1, those before them parked; the tube at its nominal stop.
struct History {
  int module = 0;
  int slot = 0;
  std::vector<KnownModule> known;
  double slide = 0.0;
};
History HistoryFor(int k, int slot) {
  History h;
  h.module = k;
  h.slot = slot;
  h.known.assign(kModules, KnownModule{});
  for (int i = 0; i < k; ++i) {
    const int s = slot - (k - i);
    h.known[i] = s >= 0 ? KnownModule{ModulePlace::kOnTube, s}
                        : KnownModule{ModulePlace::kParked, -1};
  }
  h.known[k] = {ModulePlace::kHeld, -1};
  h.slide = Conveyor::NominalSlidePosition(slot);
  return h;
}

void Apply(const PlanScene& scene, CollisionChecker& checker) {
  if (scene.pallet) checker.SetPalletPose(*scene.pallet);
  for (const auto& [i, pose] : scene.modules) checker.SetModulePose(i, pose);
  if (scene.tube_slide) checker.SetTubeSlide(*scene.tube_slide);
  if (scene.held_module) {
    checker.SetHeldModule(*scene.held_module, scene.module_in_tip);
  } else {
    checker.ClearHeldModule();
  }
}

// ============================================================ feasibility

class CheckPlanning : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    std::string error;
    std::optional<Cell> loaded = LoadCell(&error);
    ASSERT_TRUE(loaded) << error;
    cell_ = new Cell(std::move(*loaded));
    ASSERT_TRUE(cell_->Reset(1, cell::Randomization::None()));
    for (int k = 0; k < kModules; ++k) {
      PickPlace script(*cell_, CheckOptions(cell::SensingMode::kBlind));
      ASSERT_TRUE(script.Start(k, Step::kSetDown)) << script.outcome().reason;
      starts_[k] = script.Waypoints(Step::kClear)->back();
      goals_[k] = script.Waypoints(Step::kTurn)->back();
      check_pose_ = script.options().check_pose;
      grip_gap_ = script.options().grip_gap;
    }
    std::optional<CollisionChecker> checker =
        CollisionChecker::Create(*cell_->model(), {}, &error);
    ASSERT_TRUE(checker) << error;
    checker_ = new CollisionChecker(std::move(*checker));
  }
  static void TearDownTestSuite() {
    delete checker_;
    delete cell_;
    checker_ = nullptr;
    cell_ = nullptr;
  }
  void SetUp() override {
    ASSERT_NE(cell_, nullptr);
    ASSERT_NE(checker_, nullptr);
  }

  static CarrySceneInput Input(const History& h) {
    CarrySceneInput input;
    input.held = h.module;
    input.grip_gap = grip_gap_;
    input.modules = h.known;
    input.tube_slide = h.slide;
    return input;
  }

  static inline Cell* cell_ = nullptr;
  static inline CollisionChecker* checker_ = nullptr;
  static inline Eigen::VectorXd starts_[kModules];
  static inline Eigen::VectorXd goals_[kModules];
  static inline Eigen::VectorXd check_pose_;
  static inline double grip_gap_ = 0.0;
};

TEST_F(CheckPlanning, CheckPoseClearOfThePlanningMarginAtEveryDepthAndSlot) {
  const MotionPlanOptions options;
  const double margin = PlanningMargin(options);
  const double self_margin = PlanningSelfMargin(options);
  CollisionChecker& checker = *checker_;
  const double old_margin = checker.margin();
  const double old_self = checker.self_margin();
  cell::CheckOptions measure;
  measure.clearance = true;

  // The stack's estimate off by the pallet's tolerance (3 cm, 3 degrees),
  // four ways; and grips off the design by 5 mm along each axis and 1
  // degree about each, both ways.
  std::vector<Isometry3d> shifts = {Isometry3d::Identity()};
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 1.0}) {
      Isometry3d shift = Isometry3d::Identity();
      shift.translation() = Vector3d(0.03 * sx, 0.03 * sy, 0.0);
      shift.linear() =
          Eigen::AngleAxisd(3.0 * kDegree * sx * sy, Vector3d::UnitZ())
              .toRotationMatrix();
      shifts.push_back(shift);
    }
  }
  const Isometry3d design = cell::DesignModuleInTip(cell::kModuleThickness,
                                                    grip_gap_);
  std::vector<Isometry3d> grips = {design};
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {-1.0, 1.0}) {
      Vector3d shift = Vector3d::Zero();
      shift[axis] = 5.0 * kMm * sign;
      grips.push_back(design * cell::GripOffset(shift, Vector3d::Zero()));
      Vector3d turn = Vector3d::Zero();
      turn[axis] = 1.0 * kDegree * sign;
      grips.push_back(design * cell::GripOffset(Vector3d::Zero(), turn));
    }
  }

  std::printf("planning margin %.1f mm (self %.1f mm); check pose, design "
              "grip, design stack:\n",
              margin / kMm, self_margin / kMm);
  std::printf("%6s | %s\n", "module", "clearance mm by slot 0 .. 9 (the "
              "nearest pair at the smallest)");
  double worst = 1.0, worst_self = 1.0;
  std::string worst_between, worst_self_between;
  int checked = 0, not_free = 0;
  std::string first_not_free;
  for (int k = 0; k < kModules; ++k) {
    std::printf("%6d |", k);
    for (int slot = 0; slot < Conveyor::kSlotCount; ++slot) {
      const History h = HistoryFor(k, slot);
      for (std::size_t s = 0; s < shifts.size(); ++s) {
        for (std::size_t g = 0; g < grips.size(); ++g) {
          // The grips at slot k % 10 only (the run in order), with the
          // design stack: the grip moves the module, not the scene.
          if (g > 0 && (s > 0 || slot != k % Conveyor::kSlotCount)) continue;
          CarrySceneInput input = Input(h);
          input.pallet_shift = shifts[s];
          input.module_in_tip = grips[g];
          Apply(cell::CarryScene(cell_->pallet(), input), checker);
          ASSERT_TRUE(checker.SetMargins(margin, self_margin));
          const bool free = checker.IsFree(check_pose_);
          ASSERT_TRUE(checker.SetMargins(old_margin, old_self));
          ++checked;
          if (!free) {
            ++not_free;
            if (first_not_free.empty()) {
              first_not_free = "module_" + std::to_string(k) + " slot " +
                               std::to_string(slot) + " shift " +
                               std::to_string(s) + " grip " +
                               std::to_string(g);
            }
          }
          if (s == 0 && g == 0) {
            const std::optional<cell::CollisionReport> report =
                checker.Check(check_pose_, measure);
            ASSERT_TRUE(report && report->clearance && report->self_clearance);
            std::printf(" %5.0f", *report->clearance / kMm);
            if (*report->clearance < worst) {
              worst = *report->clearance;
              worst_between = report->clearance_between;
            }
            if (*report->self_clearance < worst_self) {
              worst_self = *report->self_clearance;
              worst_self_between = report->self_clearance_between;
            }
          }
        }
      }
    }
    std::printf("\n");
  }
  std::printf("smallest: %.1f mm (%s) against %.1f; self %.1f mm (%s) against "
              "%.1f; %d configurations checked at the planning margins, %d "
              "not clear\n",
              worst / kMm, worst_between.c_str(), margin / kMm,
              worst_self / kMm, worst_self_between.c_str(), self_margin / kMm,
              checked, not_free);
  EXPECT_EQ(not_free, 0) << "first: " << first_not_free;
  EXPECT_GT(worst, margin);
  EXPECT_GT(worst_self, self_margin);
}

TEST_F(CheckPlanning, TheViaCostsThisMuchCarryTimeAgainstTheSingleLeg) {
  MotionPlanOptions options;  // the script's: its defaults, fixed seed
  options.speed = cell::PickPlaceOptions{}.speed_carrying;
  const double dwell = cell::PickPlaceOptions{}.check_dwell;
  std::printf("%6s %4s | %8s %8s | %8s %8s %8s %8s | %8s\n", "module", "slot",
              "single s", "plan ms", "leg 1 s", "leg 2 s", "dwell s",
              "plan ms", "cost s");
  for (int k : {0, 9, 15, 29}) {
    SCOPED_TRACE("module_" + std::to_string(k));
    const History h = HistoryFor(k, k % Conveyor::kSlotCount);
    const PlanScene scene = cell::CarryScene(cell_->pallet(), Input(h));
    MotionPlanReport report;
    auto begin = std::chrono::steady_clock::now();
    const std::optional<cell::TimedMove> single = cell::PlanArmMove(
        *checker_, scene, starts_[k], goals_[k], cell_->arm().limits, options,
        &report);
    const double single_wall = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - begin)
                                   .count();
    ASSERT_TRUE(single) << report.reason;
    // The two legs as the script plans them: their own seeds.
    begin = std::chrono::steady_clock::now();
    MotionPlanOptions leg1_options = options;
    const std::optional<cell::TimedMove> leg1 = cell::PlanArmMove(
        *checker_, scene, starts_[k], check_pose_, cell_->arm().limits,
        leg1_options, &report);
    ASSERT_TRUE(leg1) << "leg 1: " << report.reason;
    EXPECT_GE(report.clearance, options.required_margin);
    MotionPlanOptions leg2_options = options;
    leg2_options.base_seed += 1000;
    const std::optional<cell::TimedMove> leg2 = cell::PlanArmMove(
        *checker_, scene, check_pose_, goals_[k], cell_->arm().limits,
        leg2_options, &report);
    ASSERT_TRUE(leg2) << "leg 2: " << report.reason;
    EXPECT_GE(report.clearance, options.required_margin);
    const double legs_wall = std::chrono::duration<double>(
                                 std::chrono::steady_clock::now() - begin)
                                 .count();
    EXPECT_EQ(leg1->path.back(), check_pose_);
    EXPECT_EQ(leg2->path.front(), check_pose_);
    EXPECT_EQ(leg2->path.back(), goals_[k]);
    const double two = leg1->trajectory.duration() +
                       leg2->trajectory.duration() + dwell;
    std::printf("%6d %4d | %8.2f %8.0f | %8.2f %8.2f %8.2f %8.0f | %+8.2f\n",
                k, h.slot, single->trajectory.duration(), 1e3 * single_wall,
                leg1->trajectory.duration(), leg2->trajectory.duration(),
                dwell, 1e3 * legs_wall, two - single->trajectory.duration());
  }
}

// ================================================ the check, step by step

// module_0 put into the grip at a known offset from the design grip, the
// arm at the end of clear (Cell::PutArm, TeleportModule, Grip), and the
// cycle started at carry: the carry through the check pose, the check, and
// the place side, with nothing of the pick side run. The check camera's
// pictures noisy; the grip's truth exact (the weld's pose at the grip).
struct ConstructedRun {
  std::string error;
  bool done = false;
  PickPlaceOutcome outcome;
  std::string phases;
  std::vector<cell::CarryLeg> legs;
  std::vector<cell::CheckRecord> checks;
  std::optional<Isometry3d> measured;
  Eigen::VectorXd check_pose, turn_end;
  Isometry3d truth_in_tip = Isometry3d::Identity();
  // At each picture: the arm's true angles and the tip's pose from them.
  std::vector<Eigen::VectorXd> arm_at_picture;
  std::vector<double> leg1_end;  // when the arm arrived at the check pose
  // After the set down, before the release: the module's true pose, and
  // where the script meant it; the tip it aimed for and the design's.
  std::optional<Isometry3d> module_at_release;
  std::optional<Isometry3d> release_module, release_tip;
  std::optional<cell::PlacementError> placed;
  cell::ModuleState state_at_end = cell::ModuleState::kOnPallet;
  Eigen::VectorXd arm_at_end;
  long forbidden = 0;
  std::string forbidden_list;
};

ConstructedRun RunConstructed(const Isometry3d& offset,
                              const cell::PickPlaceOptions& options) {
  ConstructedRun R;
  std::optional<Cell> loaded = LoadCell(&R.error);
  if (!loaded) return R;
  Cell& cell = *loaded;
  cell::Randomization randomization = cell::Randomization::None();
  randomization.camera_noise = true;  // the check camera's pictures too
  cell.Reset(1, randomization);
  Eigen::VectorXd clear_end;
  {
    PickPlace planner(cell, options);
    if (!planner.Start(0, Step::kSetDown)) {
      R.error = "planning: " + planner.outcome().reason;
      return R;
    }
    clear_end = planner.Waypoints(Step::kClear)->back();
    R.turn_end = planner.Waypoints(Step::kTurn)->back();
  }
  R.check_pose = options.check_pose;
  cell.PutArm(clear_end);
  cell::DataPtr scratch(mj_makeData(cell.model()));
  const Isometry3d tip =
      cell::TipPoseAt(*cell.model(), scratch.get(), cell.arm(), clear_end);
  cell.TeleportModule(
      0, tip * cell::DesignModuleInTip(cell::kModuleThickness,
                                       options.grip_gap) *
             offset);
  cell::GripFailure failure;
  if (cell.Grip(&failure) != std::optional<int>(0)) {
    R.error = "grip: " + failure.message;
    return R;
  }
  R.truth_in_tip = FromPose(*cell.suction().HeldPoseInTipAtGrip());

  PickPlace script(cell, options);
  script.set_placed_hook([&](int module, int slot) {
    mj_forward(cell.model(), cell.data());
    R.placed = cell::MeasurePlacement(cell, module, slot);
  });
  script.set_check_hook([&](const cell::CheckRecord&) {
    R.arm_at_picture.push_back(cell::JointAngles(cell.arm(), *cell.data()));
  });
  if (!script.set_pause_after(Step::kSetDown)) {
    R.error = "set_pause_after(set down) refused";
    return R;
  }
  if (!script.Start(0, Step::kCarry)) {
    R.outcome = script.outcome();
    R.phases = Phases(script);
    R.error = "start: " + script.outcome().reason;
    return R;
  }
  CycleCheck check(cell, {.clearance_every = 0});
  bool was_moving_leg1 = true;
  const double began = cell.data()->time;
  while (!script.Done() && cell.data()->time - began < kCycleLimit) {
    if (script.paused()) {
      mj_forward(cell.model(), cell.data());
      R.module_at_release = cell.TrueModulePose(0);
      R.release_module = script.release_module();
      R.release_tip = script.release_tip();
      script.Resume();
    }
    script.Update();
    cell.Step();
    check.Observe(script.step(), 0);
    if (was_moving_leg1 && script.step() == Step::kCarry &&
        (cell::JointAngles(cell.arm(), *cell.data()) - R.check_pose)
                .cwiseAbs()
                .maxCoeff() < cell::ArmFollower::kArrivedAngle) {
      was_moving_leg1 = false;
      R.leg1_end.push_back(cell.data()->time);
    }
  }
  R.done = script.Done();
  R.outcome = script.outcome();
  R.phases = Phases(script);
  R.legs = script.carry_legs();
  R.checks = script.checks();
  R.measured = script.measured_grip();
  R.state_at_end = cell.module_state(0);
  R.arm_at_end = cell::JointAngles(cell.arm(), *cell.data());
  R.forbidden = check.forbidden_count();
  for (const cell::ForbiddenContact& f : check.forbidden()) {
    R.forbidden_list += std::string("\n  ") + cell::StepName(f.step) + ": " +
                        f.between;
  }
  return R;
}

// The offset put into the grip: about 3.7 mm and 0.7 degrees, the size a
// pallet camera off by 1 mm and 0.05 degrees gives (up to 2.7 and 4.0 mm).
Isometry3d KnownOffset() {
  return cell::GripOffset(Vector3d(3.0, -2.0, -1.0) * kMm,
                          Vector3d(0.2, -0.3, 0.6) * kDegree);
}

class CheckSteps : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    run_ = new ConstructedRun(
        RunConstructed(KnownOffset(), CheckOptions(cell::SensingMode::kBlind)));
  }
  static void TearDownTestSuite() {
    delete run_;
    run_ = nullptr;
  }
  void SetUp() override {
    ASSERT_NE(run_, nullptr);
    ASSERT_TRUE(run_->error.empty()) << run_->error << run_->phases;
  }
  static inline ConstructedRun* run_ = nullptr;
};

TEST_F(CheckSteps, CarryInTwoLegsThroughTheCheckPoseWithItsDwell) {
  const ConstructedRun& R = *run_;
  ASSERT_TRUE(R.outcome.carry.found) << R.outcome.reason << R.phases;
  ASSERT_EQ(R.legs.size(), 2u);
  ASSERT_EQ(R.outcome.carry.legs.size(), 2u);
  EXPECT_EQ(R.legs[0].move.path.back(), R.check_pose) << "leg 1 ends there";
  EXPECT_NEAR(R.legs[0].dwell, 0.2, 1e-12);
  EXPECT_EQ(R.legs[1].move.path.front(), R.check_pose) << "leg 2 starts there";
  EXPECT_EQ(R.legs[1].move.path.back(), R.turn_end);
  EXPECT_EQ(R.legs[1].dwell, 0.0);
  EXPECT_GE(R.outcome.carry.clearance, MotionPlanOptions{}.required_margin);
  std::printf("carry: legs %.2f + %.2f s, dwell %.2f s; carry step %.2f s; "
              "clearance %.1f mm (%s)\n",
              R.legs[0].move.trajectory.duration(),
              R.legs[1].move.trajectory.duration(), R.legs[0].dwell,
              R.outcome.carry.seconds, 1e3 * R.outcome.carry.clearance,
              R.outcome.carry.clearance_between.c_str());
  // The carry's time holds both legs, the dwell and nothing else much.
  EXPECT_GE(R.outcome.carry.seconds, R.legs[0].move.trajectory.duration() +
                                         R.legs[1].move.trajectory.duration() +
                                         R.legs[0].dwell);
  EXPECT_LT(R.outcome.carry.seconds,
            R.legs[0].move.trajectory.duration() +
                R.legs[1].move.trajectory.duration() + R.legs[0].dwell + 0.5);
}

TEST_F(CheckSteps, PictureAtTheEndOfTheDwellModuleInViewFinderAnswers) {
  const ConstructedRun& R = *run_;
  ASSERT_EQ(R.checks.size(), 1u) << R.phases;
  const cell::CheckRecord& c = R.checks[0];
  ASSERT_TRUE(c.estimate.has_value())
      << cell::HeldFinderFailureName(c.diagnostics.failure) << ": "
      << c.diagnostics.detail;
  EXPECT_EQ(c.diagnostics.failure, cell::HeldFinderFailure::kNone);
  EXPECT_EQ(c.attempt, 1);
  // At the end of the dwell: at least the dwell after the arm arrived.
  ASSERT_EQ(R.leg1_end.size(), 1u);
  std::printf("picture at %.3f s, arm at the check pose from %.3f s; finder "
              "%.0f ms, picture %.0f ms; %d of %.0f pixels in the crop\n",
              c.time, R.leg1_end[0], 1e3 * c.diagnostics.seconds_total,
              1e3 * c.render_seconds, c.diagnostics.points_in_crop,
              c.diagnostics.expected_pixels);
  EXPECT_GE(c.time - R.leg1_end[0], 0.2 - 0.01);
  EXPECT_LT(c.time - R.leg1_end[0], 0.2 + 0.5);
  // The expected pose from the angles the arm reported, which are its true
  // angles at that moment, not the commanded check pose.
  ASSERT_EQ(R.arm_at_picture.size(), 1u);
  EXPECT_EQ(c.arm, R.arm_at_picture[0]);
  EXPECT_LT((c.arm - R.check_pose).cwiseAbs().maxCoeff(),
            cell::ArmFollower::kArrivedAngle);
  EXPECT_GT((c.arm - R.check_pose).cwiseAbs().maxCoeff(), 0.0)
      << "reported angles, not the commanded ones";
  EXPECT_LT(Distance(c.expected,
                     c.tip * cell::DesignModuleInTip(cell::kModuleThickness,
                                                     0.002)),
            1e-12);
  EXPECT_EQ(R.outcome.check.status, CheckReport::Status::kChecked);
  EXPECT_EQ(R.outcome.check.pictures, 1);
}

TEST_F(CheckSteps, MeasuredGripMatchesTheTrueGripWithNoise) {
  const ConstructedRun& R = *run_;
  ASSERT_TRUE(R.measured.has_value()) << R.phases;
  const Eigen::Matrix<double, 6, 1> error =
      cell::PoseError6(*R.measured, R.truth_in_tip);
  const Eigen::Matrix<double, 6, 1> offset = cell::PoseError6(
      R.truth_in_tip,
      cell::DesignModuleInTip(cell::kModuleThickness, 0.002));
  std::printf("true grip off the design by %.3f %.3f %.3f mm, %.3f %.3f %.3f "
              "deg\nmeasured grip's error: %.4f %.4f %.4f mm, %.4f %.4f %.4f "
              "deg\n",
              offset[0] / kMm, offset[1] / kMm, offset[2] / kMm,
              offset[3] / kDegree, offset[4] / kDegree, offset[5] / kDegree,
              error[0] / kMm, error[1] / kMm, error[2] / kMm,
              error[3] / kDegree, error[4] / kDegree, error[5] / kDegree);
  EXPECT_GT(offset.head<3>().norm(), 3.0 * kMm) << "the offset was put in";
  EXPECT_LT(error.head<3>().norm(), 0.5 * kMm);
  EXPECT_LT(error.tail<3>().norm(), 0.03 * kDegree);
  // The report's offset is the measured grip's.
  const CheckReport& c = R.outcome.check;
  EXPECT_LT(Distance(c.module_in_tip, *R.measured), 1e-12);
  EXPECT_LT((c.offset_shift - cell::PoseError6(
                                  *R.measured, cell::DesignModuleInTip(
                                                   cell::kModuleThickness,
                                                   0.002))
                                  .head<3>())
                .norm(),
            1e-9);
  // Leg 2 checked again with it.
  EXPECT_TRUE(c.rechecked);
  EXPECT_GT(c.recheck_samples, 10);
  std::printf("leg 2 recheck at %.1f mm (self %.1f): %s, %d samples, %.0f ms; "
              "replanned: %s\n",
              c.recheck_margin / kMm, c.recheck_self_margin / kMm,
              c.recheck_clear ? "clear" : c.recheck_hit.c_str(),
              c.recheck_samples, 1e3 * c.recheck_seconds,
              c.replanned ? "yes" : "no");
  EXPECT_EQ(c.replanned, !c.recheck_clear);
}

TEST_F(CheckSteps, LowerAndSetDownPutTheModuleNotTheToolOnTheSlot) {
  const ConstructedRun& R = *run_;
  ASSERT_TRUE(R.done) << "timed out" << R.phases;
  ASSERT_EQ(R.outcome.status, PickPlaceOutcome::Status::kPlaced)
      << R.outcome.reason << R.phases;
  ASSERT_TRUE(R.module_at_release && R.release_module && R.release_tip);
  ASSERT_TRUE(R.measured.has_value());
  // The tip aimed where the measured grip puts the module on its release
  // pose; with the design grip it would have aimed elsewhere.
  const Isometry3d corrected =
      cell::CorrectedTipTarget(*R.release_module, *R.measured);
  EXPECT_LT(Distance(*R.release_tip, corrected), 1e-9);
  EXPECT_LT(Angle(*R.release_tip, corrected), 1e-9);
  const Isometry3d uncorrected =
      *R.release_module *
      cell::DesignModuleInTip(cell::kModuleThickness, 0.002).inverse();
  std::printf("release tip moved %.2f mm and %.3f deg from the design grip's\n",
              Distance(*R.release_tip, uncorrected) / kMm,
              Angle(*R.release_tip, uncorrected) / kDegree);
  EXPECT_GT(Distance(*R.release_tip, uncorrected), 2.0 * kMm);
  // At the end of the set down the module, truly, is where the script
  // meant it.
  const double off = Distance(*R.module_at_release, *R.release_module);
  const double turned = Angle(*R.module_at_release, *R.release_module);
  std::printf("module at the release: %.3f mm, %.4f deg from its release "
              "pose\n",
              off / kMm, turned / kDegree);
  EXPECT_LT(off, 0.5 * kMm);
  EXPECT_LT(turned, 0.05 * kDegree);
  ASSERT_TRUE(R.placed.has_value());
  const cell::PlacementError& e = *R.placed;
  std::printf("placed: %s%s\n", cell::Describe(e).c_str(), R.phases.c_str());
  EXPECT_LT(std::abs(e.along_tube), 0.4 * kMm);
  EXPECT_LT(std::abs(e.across_tube), 0.4 * kMm);
  EXPECT_LT(std::abs(e.height), 0.3 * kMm);
  EXPECT_LT(std::abs(e.yaw), 0.03 * kDegree);
  EXPECT_LT(e.tilt, 0.05 * kDegree);
  EXPECT_TRUE(e.at_rest);
  EXPECT_FALSE(R.outcome.placement_unconfirmed);
  EXPECT_EQ(R.forbidden, 0) << R.forbidden_list;
}

// ============================================================== refusals

TEST(CheckRefusal, RefusedTwicePlacesUncorrectedAndUnchecked) {
  cell::PickPlaceOptions options = CheckOptions(cell::SensingMode::kBlind);
  // No picture can hold this many points: every one refused, "too few
  // points" (not a slip).
  options.held_finder.min_point_fraction = 1e6;
  const ConstructedRun R = RunConstructed(KnownOffset(), options);
  ASSERT_TRUE(R.error.empty()) << R.error << R.phases;
  ASSERT_EQ(R.checks.size(), 2u) << "one retake" << R.phases;
  EXPECT_EQ(R.checks[0].diagnostics.failure,
            cell::HeldFinderFailure::kTooFewPoints);
  EXPECT_EQ(R.checks[1].attempt, 2);
  EXPECT_GE(R.checks[1].time - R.checks[0].time, 0.2 - 1e-9)
      << "after another dwell";
  EXPECT_EQ(R.outcome.status, PickPlaceOutcome::Status::kPlaced)
      << R.outcome.reason << R.phases;
  EXPECT_EQ(R.outcome.check.status, CheckReport::Status::kUnchecked);
  EXPECT_EQ(R.outcome.check.failure, cell::HeldFinderFailure::kTooFewPoints);
  EXPECT_FALSE(R.measured.has_value());
  EXPECT_FALSE(R.outcome.check.rechecked);
  // Uncorrected: the tip aimed with the design grip.
  ASSERT_TRUE(R.release_module && R.release_tip);
  const Isometry3d design_tip =
      *R.release_module *
      cell::DesignModuleInTip(cell::kModuleThickness, 0.002).inverse();
  EXPECT_LT(Distance(*R.release_tip, design_tip), 1e-9);
  // The carry's record says so.
  EXPECT_NE(R.phases.find("UNCHECKED"), std::string::npos) << R.phases;
  ASSERT_TRUE(R.placed.has_value());
  std::printf("unchecked: %s (the offset put in: 3.7 mm, 0.7 deg)%s\n",
              cell::Describe(*R.placed).c_str(), R.phases.c_str());
}

TEST(CheckRefusal, SlipFailsTheCycleAtTheCheckWithoutRetake) {
  cell::PickPlaceOptions options = CheckOptions(cell::SensingMode::kBlind);
  // Tolerances far under the offset put in: the module is found, but further
  // from the expected pose than they allow, as one that slipped.
  options.held_finder.position_tolerance = 0.5 * kMm;
  const ConstructedRun R = RunConstructed(KnownOffset(), options);
  ASSERT_TRUE(R.error.empty()) << R.error << R.phases;
  ASSERT_EQ(R.outcome.status, PickPlaceOutcome::Status::kFailed) << R.phases;
  EXPECT_EQ(R.outcome.failed_step, Step::kCarry);
  EXPECT_NE(R.outcome.reason.find("slipped"), std::string::npos)
      << R.outcome.reason;
  EXPECT_EQ(R.outcome.check.status, CheckReport::Status::kSlipped);
  EXPECT_EQ(R.outcome.check.failure,
            cell::HeldFinderFailure::kFarFromExpected);
  ASSERT_EQ(R.checks.size(), 1u) << "no retake after a slip";
  // Holding the module, at the check pose.
  EXPECT_EQ(R.state_at_end, cell::ModuleState::kHeld);
  EXPECT_LT((R.arm_at_end - R.check_pose).cwiseAbs().maxCoeff(),
            cell::ArmFollower::kArrivedAngle);
}

// =============================================================== options

TEST(CheckOptionsTest, RefusedWithTheScriptedCarry) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  ASSERT_TRUE(cell.Reset(1, cell::Randomization::None()));
  cell::PickPlaceOptions options = CheckOptions(cell::SensingMode::kBlind);
  options.carry = CarryMode::kScripted;
  PickPlace script(cell, options);
  EXPECT_FALSE(script.Start(0));
  EXPECT_EQ(script.outcome().status, PickPlaceOutcome::Status::kFailed);
  EXPECT_NE(script.outcome().reason.find("needs the planned carry"),
            std::string::npos)
      << script.outcome().reason;
  // Off by default, and then nothing changes: one leg, no check.
  EXPECT_FALSE(cell::PickPlaceOptions{}.check_after_pickup);
  PickPlace plain(cell);
  ASSERT_TRUE(plain.Start(0, Step::kSetDown)) << plain.outcome().reason;
  EXPECT_EQ(plain.outcome().check.status, CheckReport::Status::kOff);
}

// ================================================= whole cycles, calibrated

// The pallet camera's calibration off by 1 mm and 0.05 degrees (the grip
// off by a few millimetres), on the same seeds with and without the check.
class CheckCycles : public ::testing::Test {
 protected:
  struct Result {
    std::uint32_t seed = 0;
    bool check = false;
    PickPlaceOutcome outcome;
    std::string phases;
    std::optional<cell::PlacementError> placed;
    std::optional<Isometry3d> measured;
    Isometry3d truth_in_tip = Isometry3d::Identity();
    long forbidden = 0;
    std::string forbidden_list;
  };

  static void SetUpTestSuite() {
    results_ = new std::vector<Result>;
    std::optional<Cell> loaded = LoadCell(&error_);
    if (!loaded) return;
    Cell& cell = *loaded;
    cell::Randomization randomization;  // pallet off nominal, noisy pictures
    randomization.camera_shift = 1.0 * kMm;
    randomization.camera_turn = 0.05 * kDegree;
    for (std::uint32_t seed : kSeeds) {
      for (bool check : {false, true}) {
        Result r;
        r.seed = seed;
        r.check = check;
        cell.Reset(seed, randomization);
        cell::PickPlaceOptions options =
            CheckOptions(cell::SensingMode::kCamera);
        options.check_after_pickup = check;
        PickPlace script(cell, options);
        script.set_placed_hook([&](int module, int slot) {
          mj_forward(cell.model(), cell.data());
          r.placed = cell::MeasurePlacement(cell, module, slot);
        });
        script.set_check_hook([&](const cell::CheckRecord&) {
          r.truth_in_tip = FromPose(*cell.suction().HeldPoseInTip(*cell.data()));
        });
        CycleCheck watch(cell, {.clearance_every = 0});
        script.Start(0);
        RunCycle(cell, script, &watch);
        r.outcome = script.outcome();
        r.phases = Phases(script);
        r.measured = script.measured_grip();
        r.forbidden = watch.forbidden_count();
        for (const cell::ForbiddenContact& f : watch.forbidden()) {
          r.forbidden_list += std::string("\n  ") + cell::StepName(f.step) +
                              ": " + f.between;
        }
        results_->push_back(r);
      }
    }
  }
  static void TearDownTestSuite() {
    delete results_;
    results_ = nullptr;
  }
  void SetUp() override {
    ASSERT_NE(results_, nullptr);
    ASSERT_TRUE(error_.empty()) << error_;
  }
  static constexpr std::uint32_t kSeeds[] = {1, 2, 3};
  static inline std::vector<Result>* results_ = nullptr;
  static inline std::string error_;
};

TEST_F(CheckCycles, EveryCyclePlacedCheckedAndMeasuredWithinTolerance) {
  ASSERT_EQ(results_->size(), 2 * std::size(kSeeds));
  for (const Result& r : *results_) {
    SCOPED_TRACE("seed " + std::to_string(r.seed) +
                 (r.check ? " with" : " without") + " the check");
    ASSERT_EQ(r.outcome.status, PickPlaceOutcome::Status::kPlaced)
        << r.outcome.reason << r.phases;
    EXPECT_EQ(r.forbidden, 0) << r.forbidden_list;
    if (!r.check) {
      EXPECT_EQ(r.outcome.check.status, CheckReport::Status::kOff);
      EXPECT_EQ(r.outcome.carry.legs.size(), 1u);
      continue;
    }
    EXPECT_EQ(r.outcome.check.status, CheckReport::Status::kChecked)
        << r.phases;
    EXPECT_EQ(r.outcome.carry.legs.size(), 2u);
    ASSERT_TRUE(r.measured.has_value());
    const Eigen::Matrix<double, 6, 1> error =
        cell::PoseError6(*r.measured, r.truth_in_tip);
    std::printf("seed %u: grip off the design by %.2f mm, %.3f deg; measured "
                "to %.3f mm, %.4f deg\n",
                r.seed, 1e3 * r.outcome.check.offset_shift.norm(),
                r.outcome.check.offset_rotation.norm() / kDegree,
                error.head<3>().norm() / kMm,
                error.tail<3>().norm() / kDegree);
    EXPECT_LT(error.head<3>().norm(), 0.5 * kMm);
    EXPECT_LT(error.tail<3>().norm(), 0.03 * kDegree);
  }
}

TEST_F(CheckCycles, PlacementWithTheCheckWellInsidePlacementWithout) {
  double worst[2] = {0.0, 0.0};      // mm, along or across, without / with
  double worst_yaw[2] = {0.0, 0.0};  // deg
  for (const Result& r : *results_) {
    ASSERT_TRUE(r.placed.has_value()) << r.outcome.reason << r.phases;
    const cell::PlacementError& e = *r.placed;
    const int i = r.check ? 1 : 0;
    std::printf("seed %u %-7s the check: %s\n", r.seed,
                r.check ? "with" : "without", cell::Describe(e).c_str());
    worst[i] = std::max({worst[i], std::abs(e.along_tube) / kMm,
                         std::abs(e.across_tube) / kMm});
    worst_yaw[i] = std::max(worst_yaw[i], std::abs(e.yaw) / kDegree);
    EXPECT_TRUE(e.at_rest);
  }
  std::printf("worst along or across: %.2f mm without, %.2f mm with; yaw "
              "%.3f deg without, %.3f deg with\n",
              worst[0], worst[1], worst_yaw[0], worst_yaw[1]);
  // The calibration error shows without the check...
  EXPECT_GT(worst[0], 1.0);
  // ...and the check takes it out: under half a millimetre, and under a
  // quarter of the worst without.
  EXPECT_LT(worst[1], 0.5);
  EXPECT_LT(worst[1], 0.25 * worst[0]);
  EXPECT_LT(worst_yaw[1], 0.03);
}

// =============================================================== stage 6

TEST(Stage6, PauseAfterAStepTheModeSkipsIsRefused) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  ASSERT_TRUE(cell.Reset(1, cell::Randomization::None()));
  PickPlace planned(cell, CheckOptions(cell::SensingMode::kBlind));
  EXPECT_FALSE(planned.set_pause_after(Step::kFlatten));
  EXPECT_FALSE(planned.set_pause_after(Step::kTurn));
  EXPECT_FALSE(planned.set_pause_after(Step::kLook)) << "blind";
  EXPECT_FALSE(planned.set_pause_after(Step::kIdle));
  EXPECT_FALSE(planned.set_pause_after(Step::kDone));
  EXPECT_FALSE(planned.set_pause_after(Step::kFailed));
  EXPECT_TRUE(planned.set_pause_after(Step::kCarry));
  EXPECT_TRUE(planned.set_pause_after(std::nullopt));
  PickPlace scripted(cell);
  EXPECT_FALSE(scripted.set_pause_after(Step::kCarry));
  EXPECT_TRUE(scripted.set_pause_after(Step::kSwing));
  cell::PickPlaceOptions camera;
  camera.sensing = cell::SensingMode::kCamera;
  PickPlace seeing(cell, camera);
  EXPECT_TRUE(seeing.set_pause_after(Step::kLook));
  EXPECT_TRUE(seeing.set_pause_after(Step::kLookAhead));
}

TEST(Stage6, RecordsSetAfterAResetSurviveStartRecordsFromBeforeDoNot) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  ASSERT_TRUE(cell.Reset(1, cell::Randomization::None()));
  PickPlace script(cell, CheckOptions(cell::SensingMode::kBlind));
  // The script sees time pass, and is told of a module.
  cell.StepFor(0.5);
  script.Update();
  script.set_known_module(2, {ModulePlace::kParked, -1});
  ASSERT_EQ(script.known_modules()[2].place, ModulePlace::kParked);
  // A Reset: the clock goes back. A record set now is kept, the one from
  // before is gone, and Start keeps it so.
  ASSERT_TRUE(cell.Reset(1, cell::Randomization::None()));
  script.set_known_module(0, {ModulePlace::kParked, -1});
  EXPECT_EQ(script.known_modules()[0].place, ModulePlace::kParked);
  EXPECT_EQ(script.known_modules()[2].place, ModulePlace::kOnStack);
  ASSERT_TRUE(script.Start(1, Step::kSetDown)) << script.outcome().reason;
  EXPECT_EQ(script.known_modules()[0].place, ModulePlace::kParked);
  EXPECT_EQ(script.known_modules()[2].place, ModulePlace::kOnStack);
}

TEST(Stage6, LetGoAndUnaccountedModulesInTheScene) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  CarrySceneInput input;
  input.held = 3;
  input.modules.assign(kModules, KnownModule{});
  KnownModule let_go;
  let_go.place = ModulePlace::kLetGo;
  let_go.slot = 1;
  let_go.slide = Conveyor::NominalSlidePosition(1);
  let_go.pose = Conveyor::SlotPose(1, let_go.slide) *
                cell::GripOffset(Vector3d(0.004, -0.003, 0.002),
                                 Vector3d(0.0, 0.0, 0.01));
  input.modules[0] = let_go;
  input.modules[1] = {ModulePlace::kUnaccounted, -1};
  input.modules[2] = {ModulePlace::kOnTube, 0};
  input.tube_slide = Conveyor::NominalSlidePosition(2);
  const PlanScene scene = cell::CarryScene(loaded->pallet(), input);
  for (const auto& [i, pose] : scene.modules) {
    if (i == 0) {
      // Where it was let go, carried one step along with the tube.
      const Isometry3d want = Conveyor::SlotPose(1, input.tube_slide) *
                              Conveyor::SlotPose(1, let_go.slide).inverse() *
                              let_go.pose;
      EXPECT_LT(Distance(pose, want), 1e-12);
      EXPECT_GT(Distance(pose, let_go.pose), 1.0) << "the tube carried it";
      EXPECT_LT(Angle(pose, let_go.pose), 1e-9);
    } else if (i == 1) {
      EXPECT_LT(Distance(pose, cell::ParkingPose(1)), 1e-12);
    }
  }
}

// A blocked set-down from a constructed state (as tests/pick_place_test.cc's
// place side): the release aimed 4 mm into the rails, so the module meets
// them early and the tool lets go where it stopped.
TEST(Stage6, BlockedSetDownIsRecordedWhereTheToolLetGo) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  cell.Reset(1, cell::Randomization::None());
  cell::PickPlaceOptions options;
  options.release_clearance = -0.004;
  Eigen::VectorXd turned;
  {
    PickPlace planner(cell, options);
    ASSERT_TRUE(planner.Start(0, Step::kSetDown)) << planner.outcome().reason;
    turned = planner.Waypoints(Step::kTurn)->back();
  }
  cell.PutArm(turned);
  cell::DataPtr scratch(mj_makeData(cell.model()));
  const Isometry3d tip =
      cell::TipPoseAt(*cell.model(), scratch.get(), cell.arm(), turned);
  cell.TeleportModule(0, tip * cell::DesignModuleInTip(cell::kModuleThickness,
                                                       options.grip_gap));
  cell::GripFailure failure;
  ASSERT_EQ(cell.Grip(&failure), std::optional<int>(0)) << failure.message;
  PickPlace script(cell, options);
  std::optional<Isometry3d> truth_at_release;
  script.set_placed_hook([&](int, int) {
    mj_forward(cell.model(), cell.data());
    truth_at_release = cell.TrueModulePose(0);
  });
  ASSERT_TRUE(script.Start(0, Step::kWaitForConveyor))
      << script.outcome().reason;
  RunCycle(cell, script);
  ASSERT_EQ(script.outcome().status, PickPlaceOutcome::Status::kPlaced)
      << script.outcome().reason << Phases(script);
  EXPECT_TRUE(script.outcome().placement_unconfirmed) << Phases(script);
  const KnownModule& known = script.known_modules()[0];
  ASSERT_EQ(known.place, ModulePlace::kLetGo) << Phases(script);
  EXPECT_EQ(known.slot, 0);
  ASSERT_TRUE(truth_at_release.has_value());
  std::printf("let go %.2f mm from where it truly lay after the retreat%s\n",
              Distance(known.pose, *truth_at_release) / kMm,
              Phases(script).c_str());
  EXPECT_LT(Distance(known.pose, *truth_at_release), 5.0 * kMm);
  // Once the tube has stopped again, the scene has it where the tube took
  // it.
  while (cell.conveyor().Moving(*cell.data())) {
    script.Update();
    cell.Step();
  }
  mj_forward(cell.model(), cell.data());
  const PlanScene scene = script.CarryPlanScene(1);
  bool found = false;
  for (const auto& [i, pose] : scene.modules) {
    if (i != 0) continue;
    found = true;
    std::printf("after the step: scene %.2f mm from the truth\n",
                Distance(pose, cell.TrueModulePose(0)) / kMm);
    EXPECT_LT(Distance(pose, cell.TrueModulePose(0)), 5.0 * kMm);
  }
  EXPECT_TRUE(found);
}

}  // namespace
