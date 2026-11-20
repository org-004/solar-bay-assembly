// The planned carry (PickPlaceOptions::carry = CarryMode::kPlanned), one
// action at a time, so that a failure names the phase:
//
//   scene      CarryScene and the script's CarryPlanScene: built from what the
//              cell knows (the stack's estimate, where it put each module,
//              the encoder, the design grip) and nothing else;
//   margins    the carry's start (end of clear) and goal (end of the turn)
//              clear of the planner's planning margin at every stack depth
//              and every slot, with the modules taken before lying in their
//              slots or parked;
//   plan       a path found for the first, middle and last module and for
//              slots 0 and 9, the same path again from the same seed;
//   failure    no path: the cycle fails in "carry" with the planner's reason,
//              and nothing of the scripted carry runs;
//   cell       two camera-mode cycles in the simulated cell on a pallet set
//              down well off nominal: the carry planned during lift, pull
//              back and clear, followed with no forbidden contact and no
//              motor saturated, and both modules placed within the bounds
//              tests/camera_cycle_test.cc holds the scripted carry to.
//
// The tests may read the truth (True..., PalletPose) to judge the script;
// the script does not.

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
#include "src/cell.h"
#include "src/collision.h"
#include "src/conveyor.h"
#include "src/cycle_check.h"
#include "src/held_module_finder.h"
#include "src/motion_planner.h"
#include "src/pallet.h"
#include "src/pick_place.h"
#include "src/score.h"

namespace {

using cell::CarryMode;
using cell::CarrySceneInput;
using cell::Cell;
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
using Step = cell::PickPlaceStep;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;
constexpr double kMm = 1e-3;
constexpr int kModules = 30;

// camera_cycle_test's pallet for its phase tests: turned and shifted well
// off nominal.
constexpr std::uint32_t kCellSeed = 8;
constexpr double kCycleLimit = 90.0;  // s of simulation per cycle

std::optional<Cell> LoadCell(std::string* error) {
  const char* path = std::getenv("MODEL_PATH");
  if (path == nullptr) {
    *error = "MODEL_PATH is not set";
    return std::nullopt;
  }
  return Cell::Load(path, error);
}

cell::PickPlaceOptions PlannedOptions(cell::SensingMode sensing) {
  cell::PickPlaceOptions options;
  options.sensing = sensing;
  options.carry = CarryMode::kPlanned;
  return options;
}

// The planning margin the planner checks start and goal against: the
// required margin plus the budget (src/motion_planner.h).
double PlanningMargin(const MotionPlanOptions& o) {
  return o.required_margin + 0.5 * o.edge_resolution + o.corner_allowance +
         o.tracking_allowance;
}
double PlanningSelfMargin(const MotionPlanOptions& o) {
  return o.required_self_margin + 0.5 * o.edge_resolution +
         o.corner_allowance + o.tracking_allowance;
}

double Distance(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return (a.translation() - b.translation()).norm();
}
double Angle(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}

Eigen::Isometry3d FromWorldPose(const cell::WorldPose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = pose.position;
  result.linear() = pose.orientation.normalized().toRotationMatrix();
  return result;
}

// What a full run in order has done by the time module k is picked:
// modules 0 .. k-1 taken, those of the current tube (slot k % 10) lying in
// slots 0 .. slot-1, the earlier tubes' parked; the tube at its nominal stop
// for that slot.
struct History {
  int module = 0;
  int slot = 0;
  std::vector<KnownModule> known;
  double slide = 0.0;
};
History HistoryFor(int k) {
  History h;
  h.module = k;
  h.slot = k % Conveyor::kSlotCount;
  h.known.assign(kModules, KnownModule{});
  const int first_on_tube = k - h.slot;
  for (int i = 0; i < k; ++i) {
    h.known[i] = i < first_on_tube
                     ? KnownModule{ModulePlace::kParked, -1}
                     : KnownModule{ModulePlace::kOnTube, i - first_on_tube};
  }
  h.known[k] = {ModulePlace::kHeld, -1};
  h.slide = Conveyor::NominalSlidePosition(h.slot);
  return h;
}

CarrySceneInput DesignInput(const History& h, double grip_gap) {
  CarrySceneInput input;
  input.held = h.module;
  input.grip_gap = grip_gap;
  input.modules = h.known;
  input.tube_slide = h.slide;
  return input;
}

// Puts `scene` into the checker as PlanArmMove does, for questions asked
// without planning.
void Apply(const PlanScene& scene, CollisionChecker& checker) {
  if (scene.world != nullptr) checker.SetScene(*scene.world);
  if (scene.pallet) checker.SetPalletPose(*scene.pallet);
  for (const auto& [i, pose] : scene.modules) checker.SetModulePose(i, pose);
  if (scene.tube_slide) checker.SetTubeSlide(*scene.tube_slide);
  if (scene.held_module) {
    checker.SetHeldModule(*scene.held_module, scene.module_in_tip);
  } else {
    checker.ClearHeldModule();
  }
  checker.SetAllowedContacts(scene.allowed_contacts);
}

// ===================================================== scene: pure function

TEST(CarryScene, BuiltFromTheEstimateTheRecordAndTheEncoder) {
  std::string error;
  std::optional<Cell> cell = LoadCell(&error);
  ASSERT_TRUE(cell) << error;
  const cell::PalletBodies& design = cell->pallet();

  CarrySceneInput input;
  input.held = 12;
  input.grip_gap = 0.002;
  // An estimate 1 cm and 2 cm off, turned a degree.
  input.pallet_shift = Eigen::Translation3d(0.01, -0.02, 0.0) *
                       Eigen::AngleAxisd(1.0 * kDegree,
                                         Eigen::Vector3d::UnitZ());
  input.modules.assign(kModules, KnownModule{});
  for (int i = 0; i < 10; ++i) input.modules[i] = {ModulePlace::kParked, -1};
  input.modules[10] = {ModulePlace::kOnTube, 0};
  input.modules[11] = {ModulePlace::kOnTube, 1};
  input.modules[12] = {ModulePlace::kHeld, -1};
  input.tube_slide = Conveyor::NominalSlidePosition(2) + 0.0013;

  const PlanScene scene = cell::CarryScene(design, input);
  EXPECT_EQ(scene.world, nullptr) << "nothing from the simulator's state";
  EXPECT_TRUE(scene.allowed_contacts.empty());
  ASSERT_TRUE(scene.pallet.has_value());
  EXPECT_LT(Distance(*scene.pallet,
                     input.pallet_shift * FromWorldPose(design.nominal_pallet)),
            1e-12);
  ASSERT_TRUE(scene.tube_slide.has_value());
  EXPECT_EQ(*scene.tube_slide, input.tube_slide);
  ASSERT_TRUE(scene.held_module.has_value());
  EXPECT_EQ(*scene.held_module, 12);
  // The design grip with its gap: the module's centre gap + thickness / 2
  // along the tip's +Z, glass toward the tip.
  const Eigen::Isometry3d grip =
      cell::DesignModuleInTip(cell::kModuleThickness, input.grip_gap);
  EXPECT_LT(Distance(scene.module_in_tip, grip), 1e-12);
  EXPECT_LT(Angle(scene.module_in_tip, grip), 1e-12);
  EXPECT_NEAR(scene.module_in_tip.translation().z(),
              input.grip_gap + 0.5 * cell::kModuleThickness, 1e-12);

  ASSERT_EQ(scene.modules.size(), static_cast<std::size_t>(kModules - 1));
  for (const auto& [i, pose] : scene.modules) {
    SCOPED_TRACE("module_" + std::to_string(i));
    EXPECT_NE(i, 12) << "the held module is the tool's, not an obstacle";
    Eigen::Isometry3d want;
    if (i < 10) {
      want = cell::ParkingPose(i);
    } else if (i < 12) {
      want = Conveyor::SlotPose(i - 10, input.tube_slide);
    } else {
      want = input.pallet_shift * FromWorldPose(design.nominal_modules[i]);
    }
    EXPECT_LT(Distance(pose, want), 1e-12);
    EXPECT_LT(Angle(pose, want), 1e-9);
  }
}

TEST(CarryScene, ModulesInFrontWithNoRecordAreTakenAsCarriedAway) {
  std::string error;
  std::optional<Cell> cell = LoadCell(&error);
  ASSERT_TRUE(cell) << error;
  CarrySceneInput input;
  input.held = 5;
  input.modules.assign(kModules, KnownModule{});  // no record at all
  const PlanScene scene = cell::CarryScene(cell->pallet(), input);
  for (const auto& [i, pose] : scene.modules) {
    const Eigen::Isometry3d want =
        i < 5 ? cell::ParkingPose(i)
              : FromWorldPose(cell->pallet().nominal_modules[i]);
    EXPECT_LT(Distance(pose, want), 1e-12) << "module_" << i;
  }
}

// ===================================================== margins and planning

// The scripted plan of every module on the nominal pallet (its clear and
// turn ends), the cell itself never stepped.
class CarryPlanning : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    std::string error;
    std::optional<Cell> loaded = LoadCell(&error);
    ASSERT_TRUE(loaded) << error;
    cell_ = new Cell(std::move(*loaded));
    ASSERT_TRUE(cell_->Reset(1, cell::Randomization::None()));
    for (int k = 0; k < kModules; ++k) {
      // Every pose of the cycle planned from the design (starting at the
      // set-down plans the place side too); nothing is simulated.
      PickPlace script(*cell_, PlannedOptions(cell::SensingMode::kBlind));
      ASSERT_TRUE(script.Start(k, Step::kSetDown)) << script.outcome().reason;
      starts_[k] = script.Waypoints(Step::kClear)->back();
      goals_[k] = script.Waypoints(Step::kTurn)->back();
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

  // Plans module k's carry in the scene of a full run in order.
  static std::optional<cell::TimedMove> Plan(int k, MotionPlanReport* report,
                                             double* wall = nullptr) {
    const History h = HistoryFor(k);
    const PlanScene scene =
        cell::CarryScene(cell_->pallet(), DesignInput(h, grip_gap_));
    MotionPlanOptions options;  // the script's: its defaults, fixed seed
    options.speed = cell::PickPlaceOptions{}.speed_carrying;
    const auto begin = std::chrono::steady_clock::now();
    std::optional<cell::TimedMove> move =
        cell::PlanArmMove(*checker_, scene, starts_[k], goals_[k],
                          cell_->arm().limits, options, report);
    if (wall != nullptr) {
      *wall = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            begin)
                  .count();
    }
    return move;
  }

  static void ExpectPlan(int k) {
    MotionPlanReport report;
    double wall = 0.0;
    const std::optional<cell::TimedMove> move = Plan(k, &report, &wall);
    ASSERT_TRUE(move) << "module_" << k << ", slot " << k % 10 << ": "
                      << cell::ToString(report.status) << ": "
                      << report.reason;
    std::printf("module_%d slot %d: %.0f ms wall, %d waypoints, %.2f s, "
                "clearance %.1f mm (%s)\n",
                k, k % 10, 1e3 * wall, report.waypoints,
                move->trajectory.duration(), 1e3 * report.clearance,
                report.clearance_between.c_str());
    EXPECT_EQ(move->path.front(), starts_[k]);
    EXPECT_EQ(move->path.back(), goals_[k]);
    // Planned at 45 mm, measured along the path at the samples: at least
    // the required 20 mm everywhere.
    EXPECT_GE(report.clearance, MotionPlanOptions{}.required_margin);
  }

  static inline Cell* cell_ = nullptr;
  static inline CollisionChecker* checker_ = nullptr;
  static inline Eigen::VectorXd starts_[kModules];
  static inline Eigen::VectorXd goals_[kModules];
  static inline double grip_gap_ = 0.0;
};

TEST_F(CarryPlanning, GoalIsTheTurnsEndRoundPlusY) {
  for (int k = 0; k < kModules; ++k) {
    // Joint 1 at -180 degrees: the swing round +Y, away from the post.
    EXPECT_NEAR(goals_[k][0], -kPi, 1.0 * kDegree) << "module_" << k;
  }
}

TEST_F(CarryPlanning, StartAndGoalClearOfThePlanningMarginAtEveryDepthAndSlot) {
  const MotionPlanOptions options;
  const double margin = PlanningMargin(options);
  const double self_margin = PlanningSelfMargin(options);
  CollisionChecker& checker = *checker_;
  const double old_margin = checker.margin();
  const double old_self = checker.self_margin();
  cell::CheckOptions measure;
  measure.clearance = true;
  double worst_start = 1.0, worst_goal = 1.0;
  std::printf("planning margin %.1f mm (self %.1f mm)\n", margin / kMm,
              self_margin / kMm);
  std::printf("%6s %4s | %8s %-44s | %8s %s\n", "module", "slot",
              "start mm", "nearest", "goal mm", "nearest");
  for (int k = 0; k < kModules; ++k) {
    const History h = HistoryFor(k);
    Apply(cell::CarryScene(cell_->pallet(), DesignInput(h, grip_gap_)),
          checker);
    double clearance[2] = {0.0, 0.0};
    std::string between[2];
    bool free[2] = {false, false};
    for (int end = 0; end < 2; ++end) {
      const Eigen::VectorXd& q = end == 0 ? starts_[k] : goals_[k];
      ASSERT_TRUE(checker.SetMargins(margin, self_margin));
      free[end] = checker.IsFree(q);
      ASSERT_TRUE(checker.SetMargins(old_margin, old_self));
      const std::optional<cell::CollisionReport> report =
          checker.Check(q, measure);
      ASSERT_TRUE(report && report->clearance);
      clearance[end] = *report->clearance;
      between[end] = report->clearance_between;
    }
    std::printf("%6d %4d | %8.1f %-44.44s | %8.1f %s\n", k, h.slot,
                clearance[0] / kMm, between[0].c_str(), clearance[1] / kMm,
                between[1].c_str());
    worst_start = std::min(worst_start, clearance[0]);
    worst_goal = std::min(worst_goal, clearance[1]);
    EXPECT_TRUE(free[0]) << "start of module_" << k << " inside the planning "
                         << "margin: " << clearance[0] / kMm << " mm, "
                         << between[0];
    EXPECT_TRUE(free[1]) << "goal of module_" << k << " (slot " << h.slot
                         << ") inside the planning margin: "
                         << clearance[1] / kMm << " mm, " << between[1];
  }
  std::printf("smallest: start %.1f mm, goal %.1f mm, against %.1f mm\n",
              worst_start / kMm, worst_goal / kMm, margin / kMm);
}

TEST_F(CarryPlanning, PlanFoundForTheFirstModuleSlot0) { ExpectPlan(0); }
TEST_F(CarryPlanning, PlanFoundForAMiddleModule) { ExpectPlan(15); }
TEST_F(CarryPlanning, PlanFoundForTheLastModuleSlot9) { ExpectPlan(29); }
TEST_F(CarryPlanning, PlanFoundForSlot9OfTheFirstTube) { ExpectPlan(9); }
TEST_F(CarryPlanning, PlanFoundForSlot0AfterATubeSwap) { ExpectPlan(10); }

TEST_F(CarryPlanning, SameSeedSamePath) {
  MotionPlanReport a, b;
  const std::optional<cell::TimedMove> first = Plan(15, &a);
  const std::optional<cell::TimedMove> second = Plan(15, &b);
  ASSERT_TRUE(first && second);
  ASSERT_EQ(first->path.size(), second->path.size());
  for (std::size_t i = 0; i < first->path.size(); ++i) {
    EXPECT_EQ(first->path[i], second->path[i]) << "waypoint " << i;
  }
}

// ============================================================ the failure

TEST(PlannedCarryFailure, NoPathFailsTheCycleInCarryWithThePlannersReason) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  ASSERT_TRUE(cell.Reset(1, cell::Randomization::None()));
  cell::PickPlaceOptions options = PlannedOptions(cell::SensingMode::kBlind);
  // A margin the end of clear cannot keep from the stack.
  options.planner.required_margin = 0.3;
  PickPlace script(cell, options);
  ASSERT_TRUE(script.Start(0));
  while (!script.Done() && cell.data()->time < kCycleLimit) {
    script.Update();
    cell.Step();
  }
  const PickPlaceOutcome& outcome = script.outcome();
  ASSERT_EQ(outcome.status, PickPlaceOutcome::Status::kFailed);
  EXPECT_EQ(outcome.failed_step, Step::kCarry) << outcome.reason;
  EXPECT_NE(outcome.reason.find("kStartNotClear"), std::string::npos)
      << outcome.reason;
  EXPECT_TRUE(outcome.carry.attempted);
  EXPECT_FALSE(outcome.carry.found);
  ASSERT_FALSE(outcome.carry.legs.empty());
  EXPECT_EQ(outcome.carry.legs.back().status,
            cell::MotionPlanStatus::kStartNotClear);
  // The carry step's record carries the reason; nothing scripted ran.
  ASSERT_FALSE(script.records().empty());
  EXPECT_EQ(script.records().back().step, Step::kCarry);
  EXPECT_EQ(script.records().back().result.rfind("failed: no planned carry",
                                                 0),
            0u)
      << script.records().back().result;
  for (const cell::StepRecord& r : script.records()) {
    EXPECT_NE(r.step, Step::kFlatten);
    EXPECT_NE(r.step, Step::kSwing);
    EXPECT_NE(r.step, Step::kTurn);
  }
  // Holding the module where clear left it.
  EXPECT_EQ(cell.module_state(0), cell::ModuleState::kHeld);
  EXPECT_LT((cell::JointAngles(cell.arm(), *cell.data()) -
             script.Waypoints(Step::kClear)->back())
                .cwiseAbs()
                .maxCoeff(),
            1e-3);
}

// ======================================================== in the cell

// Two camera-mode cycles, modules 0 and 1, with the planned carry; paused
// after each grip to look at the scene the script will plan in.
struct CycleLog {
  bool done = false;
  PickPlaceOutcome outcome;
  std::string phases;
  std::optional<cell::PlacementError> placed;
  // At the grip: the script's scene and the truth it should match.
  std::optional<PlanScene> scene;
  std::vector<Eigen::Isometry3d> truth;  // every module, at the grip
  Eigen::Isometry3d true_pallet = Eigen::Isometry3d::Identity();
  double encoder = 0.0;
  KnownModule held_record;
  // At the end of the carry: the arm against the turn's end.
  double carry_arrival = 1.0;  // rad, largest joint error
  std::string carry_result;
};

struct CellLog {
  std::string error;
  CycleLog cycles[2];
  long forbidden_carry = 0, forbidden_all = 0;
  std::string forbidden_list;
  long saturated = 0, out_of_range = 0, non_finite = 0;
  Eigen::VectorXd carry_torque;
  std::string report;
};

std::string Phases(const PickPlace& script) {
  std::string text;
  for (const cell::StepRecord& r : script.records()) {
    char line[600];
    std::snprintf(line, sizeof(line), "\n  %-14s %6.3f s  %s",
                  cell::StepName(r.step), r.duration(), r.result.c_str());
    text += line;
  }
  return text;
}

class PlannedCarryInTheCell : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    log_ = new CellLog;
    CellLog& L = *log_;
    std::optional<Cell> loaded = LoadCell(&L.error);
    if (!loaded) return;
    Cell& cell = *loaded;
    cell.Reset(kCellSeed, cell::Randomization{});
    PickPlace script(cell, PlannedOptions(cell::SensingMode::kCamera));
    std::optional<cell::PlacementError>* placed = nullptr;
    script.set_placed_hook([&](int module, int slot) {
      mj_forward(cell.model(), cell.data());
      if (placed != nullptr) *placed = cell::MeasurePlacement(cell, module, slot);
    });
    script.set_pause_after(Step::kGrip);
    CycleCheck check(cell);
    for (int m = 0; m < 2; ++m) {
      CycleLog& C = L.cycles[m];
      placed = &C.placed;
      if (!script.Start(m)) {
        C.outcome = script.outcome();
        return;
      }
      const double began = cell.data()->time;
      Step last = script.step();
      while (!script.Done() && cell.data()->time - began < kCycleLimit) {
        if (script.paused()) {
          // After the grip: the scene the carry will be planned in.
          const int held = script.outcome().module;
          C.scene = script.CarryPlanScene(held);
          mj_forward(cell.model(), cell.data());
          for (int i = 0; i < cell.module_count(); ++i) {
            C.truth.push_back(cell.TrueModulePose(i));
          }
          C.true_pallet =
              FromWorldPose(cell::PalletPose(*cell.data(), cell.pallet()));
          C.encoder = cell.conveyor().SensorPosition(*cell.data());
          C.held_record = script.known_modules()[held];
          script.Resume();
        }
        script.Update();
        cell.Step();
        check.Observe(script.step(), script.outcome().module);
        if (last == Step::kCarry && script.step() != Step::kCarry) {
          C.carry_arrival =
              (cell::JointAngles(cell.arm(), *cell.data()) -
               script.Waypoints(Step::kTurn)->back())
                  .cwiseAbs()
                  .maxCoeff();
          C.carry_result = script.records().empty()
                               ? ""
                               : script.records()[script.records().size() - 2]
                                     .result;
        }
        last = script.step();
      }
      C.done = script.Done();
      C.outcome = script.outcome();
      C.phases = Phases(script);
      if (C.outcome.status != PickPlaceOutcome::Status::kPlaced) break;
    }
    for (const cell::ForbiddenContact& f : check.forbidden()) {
      L.forbidden_list += std::string("\n  ") + cell::StepName(f.step) +
                          ": " + f.between;
      if (f.step == Step::kCarry) ++L.forbidden_carry;
    }
    L.forbidden_all = check.forbidden_count();
    L.saturated = check.saturated_steps();
    L.out_of_range = check.out_of_range_steps();
    L.non_finite = check.non_finite_steps();
    L.carry_torque = check.peak_torque(Step::kCarry);
    L.report = check.Report(true);
  }
  static void TearDownTestSuite() {
    delete log_;
    log_ = nullptr;
  }
  void SetUp() override {
    ASSERT_NE(log_, nullptr);
    ASSERT_TRUE(log_->error.empty()) << log_->error;
  }
  static inline CellLog* log_ = nullptr;
};

TEST_F(PlannedCarryInTheCell, SceneAtTheGripMatchesTheCellWithinTheEstimate) {
  for (int m = 0; m < 2; ++m) {
    SCOPED_TRACE("cycle " + std::to_string(m));
    const CycleLog& C = log_->cycles[m];
    ASSERT_TRUE(C.scene.has_value()) << C.outcome.reason << C.phases;
    const PlanScene& scene = *C.scene;
    ASSERT_TRUE(scene.held_module.has_value());
    EXPECT_EQ(*scene.held_module, m);
    EXPECT_EQ(C.held_record.place, ModulePlace::kHeld);
    EXPECT_EQ(scene.world, nullptr);
    ASSERT_TRUE(scene.tube_slide.has_value());
    EXPECT_NEAR(*scene.tube_slide, C.encoder, 1e-9) << "the encoder's";
    // The stack where the estimate puts it: the camera's error, a
    // millimetre or so, not the pallet's 3 cm.
    ASSERT_TRUE(scene.pallet.has_value());
    EXPECT_LT(Distance(*scene.pallet, C.true_pallet), 5 * kMm);
    EXPECT_LT(Angle(*scene.pallet, C.true_pallet), 0.1 * kDegree);
    int on_tube = 0;
    for (const auto& [i, pose] : scene.modules) {
      SCOPED_TRACE("module_" + std::to_string(i));
      if (i > m) {
        EXPECT_LT(Distance(pose, C.truth[i]), 5 * kMm);
        EXPECT_LT(Angle(pose, C.truth[i]), 0.1 * kDegree);
      } else {
        // Module 0 in cycle 1: in slot 0, where the encoder says the tube
        // carried it. Within the placement error and the tube's give.
        ++on_tube;
        EXPECT_LT(Distance(pose, C.truth[i]), 3 * kMm);
        EXPECT_LT(Angle(pose, C.truth[i]), 0.1 * kDegree);
      }
    }
    EXPECT_EQ(on_tube, m);
  }
}

TEST_F(PlannedCarryInTheCell, CarryPlannedDuringLiftPullBackAndClear) {
  for (int m = 0; m < 2; ++m) {
    SCOPED_TRACE("cycle " + std::to_string(m));
    const CycleLog& C = log_->cycles[m];
    const cell::CarryReport& carry = C.outcome.carry;
    ASSERT_TRUE(carry.found) << C.outcome.reason << C.phases;
    EXPECT_EQ(carry.mode, CarryMode::kPlanned);
    ASSERT_EQ(carry.legs.size(), 1u);
    std::printf("cycle %d: planned in %.0f ms against %.2f s of lift, pull "
                "back and clear (%s); carry %.2f s; clearance %.1f mm (%s)\n",
                m, 1e3 * carry.plan_seconds, carry.overlap_seconds,
                carry.fits() ? "fits" : "does NOT fit", carry.seconds,
                1e3 * carry.clearance, carry.clearance_between.c_str());
    EXPECT_GT(carry.overlap_seconds, 1.0);
    EXPECT_GT(carry.plan_seconds, 0.0);
    EXPECT_FALSE(carry.tube_moving);
    EXPECT_GE(carry.clearance, MotionPlanOptions{}.required_margin);
    EXPECT_NE(C.carry_result.find("planned in"), std::string::npos)
        << C.carry_result;
  }
}

TEST_F(PlannedCarryInTheCell, CarryFollowedWithoutContactOrSaturation) {
  const CellLog& L = *log_;
  for (int m = 0; m < 2; ++m) {
    const CycleLog& C = L.cycles[m];
    ASSERT_TRUE(C.outcome.carry.found) << C.outcome.reason << C.phases;
    EXPECT_LT(C.carry_arrival, cell::ArmFollower::kArrivedAngle)
        << "cycle " << m << ": the carry ended off the turn's end";
  }
  EXPECT_EQ(L.forbidden_carry, 0) << L.forbidden_list;
  EXPECT_EQ(L.saturated, 0);
  EXPECT_EQ(L.out_of_range, 0);
  EXPECT_EQ(L.non_finite, 0);
  ASSERT_EQ(L.carry_torque.size(), cell::Arm::kJoints);
  std::printf("carry: peak torque fraction %.3f\n", L.carry_torque.maxCoeff());
  EXPECT_LT(L.carry_torque.maxCoeff(), 0.95);
}

TEST_F(PlannedCarryInTheCell, WholeCameraCyclesPlaceWithinTheCameraBounds) {
  const CellLog& L = *log_;
  for (int m = 0; m < 2; ++m) {
    SCOPED_TRACE("cycle " + std::to_string(m));
    const CycleLog& C = L.cycles[m];
    ASSERT_TRUE(C.done) << "timed out" << C.phases;
    ASSERT_EQ(C.outcome.status, PickPlaceOutcome::Status::kPlaced)
        << C.outcome.reason << C.phases;
    ASSERT_TRUE(C.placed.has_value());
    const cell::PlacementError& e = *C.placed;
    std::printf("cycle %d: %s; cycle %.2f s%s\n", m, cell::Describe(e).c_str(),
                C.outcome.end_time - C.outcome.start_time, C.phases.c_str());
    EXPECT_EQ(e.slot, m);
    // camera_cycle_test's bounds for a randomized pallet.
    EXPECT_LT(std::abs(e.along_tube), 1.0 * kMm);
    EXPECT_LT(std::abs(e.across_tube), 1.5 * kMm);
    EXPECT_LT(std::abs(e.height), 0.3 * kMm);
    EXPECT_LT(std::abs(e.yaw), 0.05 * kDegree);
    EXPECT_LT(e.tilt, 0.05 * kDegree);
    EXPECT_TRUE(e.at_rest);
  }
  EXPECT_EQ(L.forbidden_all, 0) << L.forbidden_list;
}

}  // namespace
