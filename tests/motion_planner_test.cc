// Checks the motion planner (src/motion_planner.h) on the cell's real moves:
// the carry of module_0, module_15 and module_29 from the end of the
// scripted cycle's "clear" to the start of its "lower", with the tube empty
// and with modules already lying on earlier slots, and the bare arm's
// return from the end of "retreat" to the ready pose. Every path is checked
// again much more finely than it was planned, at the full required margin.
// Then: the same seed gives the same path; statistics over 50 base seeds
// against the scripted path; which way round the module goes, past the
// camera's post or not; failures and their reasons; the rounded trajectory
// sampled densely; and one planned carry executed in the simulated cell
// with the module really held, under cycle_check.
//
//   bazel test //:motion_planner_test
//   bazel test --config=opt //:motion_planner_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/cell.xml). Every scene is the
// nominal one: Reset(1, Randomization::None()). The start and goal of each
// move are the scripted cycle's own waypoints (PickPlace::Waypoints), the
// module held at the design grip: the tool tip at the touch pose, the module
// where it stands in the stack.
//
// Most of the time goes to the 50-seed statistics (some 200 plans) and to
// simulating one cycle up to the end of the carry. The numbers measured are
// printed; read them with --test_output=all, and the times only from the
// optimized build.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
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
#include "src/collision.h"
#include "src/conveyor.h"
#include "src/cycle_check.h"
#include "src/joint_move.h"
#include "src/motion_planner.h"
#include "src/path_timing.h"
#include "src/pick_place.h"
#include "src/rrt.h"

namespace {

using cell::Cell;
using cell::CollisionChecker;
using cell::Conveyor;
using cell::JointPath;
using cell::MotionPlanOptions;
using cell::MotionPlanReport;
using cell::MotionPlanStatus;
using Step = cell::PickPlaceStep;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;

// Paths are checked again at this many times finer than the planner's edge
// resolution: 6 mm of motion between samples against its 30 mm.
constexpr int kFiner = 5;

// Base seeds for the statistics.
constexpr int kStatisticsSeeds = 50;

// One move to plan: the scene in its checker, start and goal, and the
// scripted cycle's own path between them for comparison.
struct Case {
  std::string name;
  int module = 0;
  int slot = 0;  // the tube this many steps on, slots before it full
  bool carry = true;
  std::optional<CollisionChecker> checker;
  Eigen::VectorXd start, goal;
  JointPath scripted;
  double scripted_speed = 0.9;
};

double Median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return n % 2 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

double Max(const std::vector<double>& values) {
  return values.empty() ? 0.0
                        : *std::max_element(values.begin(), values.end());
}

double Min(const std::vector<double>& values) {
  return values.empty() ? 0.0
                        : *std::min_element(values.begin(), values.end());
}

// The scripted moves timed as the script's follower plays them: one
// JointMove from rest to rest per leg, at `speed`.
double StopAndGoDuration(const JointPath& path, const cell::JointLimits& limits,
                         double speed) {
  cell::JointLimits scaled = limits;
  scaled.velocity *= speed;
  scaled.acceleration *= speed * speed;
  scaled.jerk *= speed * speed * speed;
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    std::optional<cell::JointMove> move = cell::JointMove::Plan(
        path[i - 1], path[i], scaled, cell::ProfileShape::kSCurve);
    if (move) total += move->duration();
  }
  return total;
}

// The smallest clearance along a path, sampled `spacing` apart, with what
// it is between.
std::pair<double, std::string> SmallestClearance(CollisionChecker& checker,
                                                 const JointPath& path,
                                                 double spacing) {
  double smallest = std::numeric_limits<double>::infinity();
  std::string between;
  cell::CheckOptions measure;
  measure.clearance = true;
  for (const Eigen::VectorXd& q : cell::DenseSamples(checker, path, spacing)) {
    std::optional<cell::CollisionReport> c = checker.Check(q, measure);
    if (c && c->clearance && *c->clearance < smallest) {
      smallest = *c->clearance;
      between = c->clearance_between;
    }
  }
  return {smallest, between};
}

// Six numbers in a row, scaled.
std::string Row(const Eigen::VectorXd& v, double scale) {
  std::string text;
  for (int j = 0; j < v.size(); ++j) {
    char number[24];
    std::snprintf(number, sizeof(number), " %8.2f", scale * v[j]);
    text += number;
  }
  return text;
}

std::string Describe(const MotionPlanReport& r) {
  char text[400];
  std::snprintf(text, sizeof(text),
                "%s: search %.1f ms, shorten %.1f ms, %lld queries, %d "
                "waypoints, %.2f m of motion (raw %.2f), clearance %.1f mm "
                "(%s), self %.1f mm, seed %u",
                cell::ToString(r.status), 1e3 * r.search_seconds,
                1e3 * r.shorten_seconds,
                static_cast<long long>(r.search_queries + r.shorten_queries),
                r.waypoints, r.length, r.raw_length, 1e3 * r.clearance,
                r.clearance_between.c_str(), 1e3 * r.self_clearance, r.seed);
  return text;
}

class MotionPlannerTest : public ::testing::Test {
 protected:
  // The cell and every case are built once for the suite: the carries of
  // deeper modules need the modules in front parked, which is done once,
  // in order, instead of a Reset per case.
  static void SetUpTestSuite() {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";
    std::string error;
    std::optional<Cell> loaded = Cell::Load(path, &error);
    ASSERT_TRUE(loaded) << error;
    cell_.emplace(std::move(*loaded));
    ASSERT_TRUE(cell_->Reset(1, cell::Randomization::None()));
    Cell& cell = *cell_;

    int parked = 0;
    for (int k : {0, 15, 29}) {
      // The scripted cycle for module k, all of it planned (starting at the
      // set-down plans the place side too); nothing is simulated.
      cell::PickPlace script(cell);
      ASSERT_TRUE(script.Start(k, Step::kSetDown)) << script.outcome().reason;
      auto waypoints = [&](Step s) { return *script.Waypoints(s); };

      // The design grip: the tool at the touch pose, the module where it
      // stands.
      cell.PutArm(waypoints(Step::kTouchGlass).back());
      const Eigen::Isometry3d grip =
          cell::TipPose(cell.arm(), *cell.data()).inverse() *
          cell.TrueModulePose(k);
      cell.PutArm(cell.arm_home());
      grips_[k] = grip;

      // The modules in front are gone.
      for (; parked < k; ++parked) cell.ParkModule(parked);

      JointPath carry = {waypoints(Step::kClear).back()};
      for (Step s : {Step::kFlatten, Step::kSwing, Step::kTurn}) {
        for (const Eigen::VectorXd& q : waypoints(s)) carry.push_back(q);
      }
      JointPath back = {waypoints(Step::kRetreat).back()};
      for (const Eigen::VectorXd& q : waypoints(Step::kReturn)) {
        back.push_back(q);
      }
      // Where this module's cycle really puts it: tube k / 10, slot k % 10,
      // with the modules before it on that tube on the slots before.
      const int real_slot = k % Conveyor::kSlotCount;
      std::vector<int> slots = {0};
      if (real_slot > 0) {
        slots.push_back(real_slot);
      } else {
        slots.push_back(Conveyor::kSlotCount - 1);  // module_0: borrowed
      }
      for (int slot : slots) {
        // The modules lying on slots 0 .. slot - 1: those picked just
        // before, or for module_0, which has none, the stack's back ones.
        std::vector<int> on_tube;
        for (int j = 0; j < slot; ++j) {
          on_tube.push_back(k >= slot ? k - slot + j
                                      : cell.module_count() - slot + j);
        }
        cases_.push_back(MakeCase(
            "carry module_" + std::to_string(k) + " slot " + std::to_string(slot),
            k, slot, true, on_tube, grip, carry,
            script.options().speed_carrying));
        if (slot == real_slot) {
          cases_.push_back(MakeCase(
              "return module_" + std::to_string(k) + " slot " +
                  std::to_string(slot),
              k, slot, false, on_tube, grip, back,
              script.options().speed_empty));
        }
      }
    }
  }

  static void TearDownTestSuite() {
    cases_.clear();
    cell_.reset();
  }

  static Case MakeCase(const std::string& name, int k, int slot, bool carry,
                       const std::vector<int>& on_tube,
                       const Eigen::Isometry3d& grip, const JointPath& scripted,
                       double speed) {
    Case c;
    c.name = name;
    c.module = k;
    c.slot = slot;
    c.carry = carry;
    std::string error;
    c.checker = CollisionChecker::Create(*cell_->model(), {}, &error);
    EXPECT_TRUE(c.checker) << error;
    c.checker->SetScene(*cell_->data());
    const double slide = Conveyor::NominalSlidePosition(slot);
    c.checker->SetTubeSlide(slide);
    for (int j = 0; j < static_cast<int>(on_tube.size()); ++j) {
      c.checker->SetModulePose(on_tube[j], Conveyor::SlotPose(j, slide));
    }
    if (carry) {
      c.checker->SetHeldModule(k, grip);
    } else {
      // Placed: module k lies in its slot, and the arm is bare.
      c.checker->SetModulePose(k, Conveyor::SlotPose(slot, slide));
    }
    c.start = scripted.front();
    c.goal = scripted.back();
    c.scripted = scripted;
    c.scripted_speed = speed;
    return c;
  }

  static Case& Find(const std::string& name) {
    for (Case& c : cases_) {
      if (c.name == name) return c;
    }
    ADD_FAILURE() << "no case " << name;
    return cases_.front();
  }

  // Every guarantee a planned path must keep, checked independently of the
  // planner: ends, ranges, the wrist's winding, and every edge free when
  // checked kFiner times more finely than planned at the required margin.
  // Returns the number of samples checked.
  static long ExpectSound(Case& c, const JointPath& path,
                          const MotionPlanReport& report,
                          const MotionPlanOptions& options = {}) {
    CollisionChecker& checker = *c.checker;
    const cell::Arm& arm = cell_->arm();
    EXPECT_FALSE(path.empty()) << c.name;
    if (path.empty()) return 0;
    EXPECT_EQ((path.front() - c.start).norm(), 0.0) << c.name;
    EXPECT_EQ((path.back() - report.goal).norm(), 0.0) << c.name;
    for (const Eigen::VectorXd& q : path) {
      for (int j = 0; j < cell::Arm::kJoints; ++j) {
        EXPECT_GE(q[j], arm.lower[j]) << c.name << " joint " << j + 1;
        EXPECT_LE(q[j], arm.upper[j]) << c.name << " joint " << j + 1;
        EXPECT_GE(q[j], report.lower[j] - 1e-12) << c.name;
        EXPECT_LE(q[j], report.upper[j] + 1e-12) << c.name;
      }
      // Joints 4 and 6 never wound beyond the span from start to goal
      // plus the slack the planner allows.
      for (int j : {3, 5}) {
        const double low = std::min(c.start[j], report.goal[j]);
        const double high = std::max(c.start[j], report.goal[j]);
        EXPECT_GE(q[j], low - options.wrist_slack - 1e-12)
            << c.name << " joint " << j + 1;
        EXPECT_LE(q[j], high + options.wrist_slack + 1e-12)
            << c.name << " joint " << j + 1;
      }
    }
    // The checker's own margins are the required ones (the defaults), and
    // the planner put them back.
    EXPECT_EQ(checker.margin(), options.required_margin);
    EXPECT_EQ(checker.self_margin(), options.required_self_margin);
    const JointPath samples =
        cell::DenseSamples(checker, path, options.edge_resolution / kFiner);
    long blocked = 0;
    for (const Eigen::VectorXd& q : samples) {
      if (checker.IsFree(q)) continue;
      if (blocked++ == 0) {
        std::optional<cell::CollisionReport> why = checker.Check(q);
        ADD_FAILURE() << c.name << ": a sample is not free: "
                      << (why && !why->pairs.empty() ? why->pairs[0].between
                                                     : "?");
      }
    }
    EXPECT_EQ(blocked, 0) << c.name << ", of " << samples.size();
    // The soundness argument, measured: every point along the edges is
    // within half the resolution of a configuration checked at the
    // planning margin.
    EXPECT_GE(report.clearance,
              report.planning_margin - 0.5 * options.edge_resolution)
        << c.name << ": " << report.clearance_between;
    EXPECT_GE(report.self_clearance,
              report.planning_self_margin - 0.5 * options.edge_resolution)
        << c.name << ": " << report.self_clearance_between;
    return static_cast<long>(samples.size());
  }

  static void Statistics(const std::string& name);

  static inline std::optional<Cell> cell_;
  static inline std::vector<Case> cases_;
  static inline Eigen::Isometry3d grips_[30];
};

// ----------------------------------------------------- the cell's own moves

TEST_F(MotionPlannerTest, CarriesArePlannedAndSoundEverywhere) {
  int carries = 0;
  for (Case& c : cases_) {
    if (!c.carry) continue;
    ++carries;
    MotionPlanReport report;
    std::optional<JointPath> path =
        cell::PlanMove(*c.checker, c.start, c.goal, {}, &report);
    ASSERT_TRUE(path) << c.name << ": " << report.reason;
    const long samples = ExpectSound(c, *path, report);
    std::printf("%-24s %s; re-checked at %d samples\n", c.name.c_str(),
                Describe(report).c_str(), static_cast<int>(samples));
    // The carry goes round the +Y side: the goal's joint 1 is -180
    // degrees, and joint 1 turns only through -90 to get there.
    EXPECT_LT(c.goal[0], -170.0 * kDegree);
  }
  EXPECT_EQ(carries, 6);
  // The setup the planner made for the carry of module_0.
  MotionPlanReport setup;
  cell::PlanMove(*cases_[0].checker, cases_[0].start, cases_[0].goal, {},
                 &setup);
  std::printf("setup, carry: planning margin %.1f mm, self %.1f mm, max "
              "deviation %.3f mrad, tracking bound %.3f mrad\n",
              1e3 * setup.planning_margin, 1e3 * setup.planning_self_margin,
              1e3 * setup.max_deviation, 1e3 * setup.tracking_bound);
  std::printf("  weights m/rad   %s\n", Row(setup.weights, 1.0).c_str());
  std::printf("  box lower deg   %s\n", Row(setup.lower, 1 / kDegree).c_str());
  std::printf("  box upper deg   %s\n", Row(setup.upper, 1 / kDegree).c_str());
  std::printf("  start deg       %s\n",
              Row(cases_[0].start, 1 / kDegree).c_str());
  std::printf("  goal deg        %s\n",
              Row(cases_[0].goal, 1 / kDegree).c_str());
}

// Which module lies on slot j of case c's tube: the one just placed on its
// own slot, the ones picked before on the slots before (for module_0, which
// has none, the stack's back ones, as SetUpTestSuite put them).
int OnSlot(const Case& c, int j, int module_count) {
  if (j == c.slot) return c.module;
  return c.module >= c.slot ? c.module - c.slot + j
                            : module_count - c.slot + j;
}

// The tube `fraction` of a step on from case c's slot, everything on it
// moved along.
void MoveTube(Case& c, double fraction, int module_count) {
  const double slide = Conveyor::NominalSlidePosition(c.slot) +
                       fraction * (Conveyor::NominalSlidePosition(c.slot + 1) -
                                   Conveyor::NominalSlidePosition(c.slot));
  c.checker->SetTubeSlide(slide);
  for (int j = 0; j <= c.slot; ++j) {
    c.checker->SetModulePose(OnSlot(c, j, module_count),
                             Conveyor::SlotPose(j, slide));
  }
}

TEST_F(MotionPlannerTest, ReturnsArePlannedAndSoundWhereverTheTubeIs) {
  int returns = 0;
  for (Case& c : cases_) {
    if (c.carry) continue;
    ++returns;
    MotionPlanReport report;
    std::optional<JointPath> path =
        cell::PlanMove(*c.checker, c.start, c.goal, {}, &report);
    ASSERT_TRUE(path) << c.name << ": " << report.reason;
    ExpectSound(c, *path, report);
    std::printf("%-24s %s\n", c.name.c_str(), Describe(report).c_str());

    // The return runs while the tube moves on a step (src/pick_place.h),
    // the module just placed riding on it; the plan saw it standing. The
    // path is free at the required margin wherever the tube is along that
    // step: checked with the tube and everything on it moved by quarter
    // steps.
    for (double f : {0.25, 0.5, 0.75, 1.0}) {
      MoveTube(c, f, cell_->module_count());
      long blocked = 0;
      for (const Eigen::VectorXd& q : cell::DenseSamples(
               *c.checker, *path, MotionPlanOptions{}.edge_resolution / kFiner)) {
        blocked += !c.checker->IsFree(q);
      }
      EXPECT_EQ(blocked, 0) << c.name << ", tube " << f << " of a step on";
    }
    MoveTube(c, 0.0, cell_->module_count());  // back as the case had it
  }
  EXPECT_EQ(returns, 3);
}

// --------------------------------------------------------------- repeatable

TEST_F(MotionPlannerTest, SameBaseSeedSamePathOtherSeedsOtherValidPaths) {
  Case& c = Find("carry module_15 slot 5");
  MotionPlanOptions options;
  options.base_seed = 7;
  MotionPlanReport first, second, other;
  std::optional<JointPath> a =
      cell::PlanMove(*c.checker, c.start, c.goal, options, &first);
  std::optional<JointPath> b =
      cell::PlanMove(*c.checker, c.start, c.goal, options, &second);
  ASSERT_TRUE(a && b);
  ASSERT_EQ(a->size(), b->size());
  for (std::size_t i = 0; i < a->size(); ++i) {
    EXPECT_EQ(((*a)[i] - (*b)[i]).norm(), 0.0) << "waypoint " << i;
  }
  EXPECT_EQ(first.seed, second.seed);
  EXPECT_EQ(first.search_queries, second.search_queries);
  EXPECT_EQ(first.shorten_queries, second.shorten_queries);

  options.base_seed = 8;
  std::optional<JointPath> d =
      cell::PlanMove(*c.checker, c.start, c.goal, options, &other);
  ASSERT_TRUE(d) << other.reason;
  ExpectSound(c, *d, other, options);
  EXPECT_NE(other.seed, first.seed);
  EXPECT_NE(other.length, first.length);
}

// --------------------------------------------------------------- statistics

void MotionPlannerTest::Statistics(const std::string& name) {
  Case& c = Find(name);
  CollisionChecker& checker = *c.checker;
  std::vector<double> search, shorten, total, queries, length, clearance,
      duration;
  int found = 0;
  long samples = 0;
  MotionPlanReport last;
  for (int s = 1; s <= kStatisticsSeeds; ++s) {
    MotionPlanOptions options;
    options.base_seed = static_cast<std::uint32_t>(s);
    MotionPlanReport report;
    std::optional<JointPath> path =
        cell::PlanMove(checker, c.start, c.goal, options, &report);
    search.push_back(1e3 * report.search_seconds);
    shorten.push_back(1e3 * report.shorten_seconds);
    total.push_back(1e3 * (report.search_seconds + report.shorten_seconds));
    queries.push_back(
        static_cast<double>(report.search_queries + report.shorten_queries));
    if (!path) {
      ADD_FAILURE() << name << ", base seed " << s << ": " << report.reason;
      continue;
    }
    ++found;
    samples += ExpectSound(c, *path, report, options);
    length.push_back(report.length);
    clearance.push_back(1e3 * report.clearance);
    std::optional<cell::Trajectory> trajectory = cell::Trajectory::Plan(
        *path, cell_->arm().limits,
        {.max_deviation = report.max_deviation, .speed = c.scripted_speed});
    ASSERT_TRUE(trajectory);
    duration.push_back(trajectory->duration());
    last = report;
  }

  const Eigen::VectorXd weights = checker.LeverArms();
  const double scripted_length = cell::PathLength(c.scripted, weights);
  const auto [scripted_clearance, scripted_between] =
      SmallestClearance(checker, c.scripted, 0.01);
  const double scripted_duration =
      StopAndGoDuration(c.scripted, cell_->arm().limits, c.scripted_speed);
  std::printf(
      "%s, %d base seeds (each the best of %d plans): found %d (%.0f%%); "
      "re-checked at %ld samples\n"
      "  planning ms     median %8.1f  worst %8.1f   (search %.1f / %.1f, "
      "shorten %.1f / %.1f)\n"
      "  queries         median %8.0f  worst %8.0f\n"
      "  length m        median %8.2f  worst %8.2f   scripted %.2f\n"
      "  clearance mm    median %8.1f  worst %8.1f   scripted %.1f (%s)\n"
      "  trajectory s    median %8.2f  worst %8.2f   scripted, stopping at "
      "each waypoint, %.2f\n",
      name.c_str(), kStatisticsSeeds, MotionPlanOptions{}.seeds, found,
      100.0 * found / kStatisticsSeeds, samples, Median(total), Max(total),
      Median(search), Max(search), Median(shorten), Max(shorten),
      Median(queries), Max(queries), Median(length), Max(length),
      scripted_length, Median(clearance), Min(clearance),
      1e3 * scripted_clearance, scripted_between.c_str(), Median(duration),
      Max(duration), scripted_duration);

  // Every base seed finds a path: the passages are wide, a first round of
  // four seeds that all fail within 2,000 iterations has not been seen, and
  // two more rounds with twice and four times the iterations follow.
  EXPECT_EQ(found, kStatisticsSeeds) << name;
  // Within the budget with room to spare: the planner is to run between
  // two moves of the cycle, and the arm needs seconds for each move. A
  // carry takes some 25,000 queries (9 to 17 us each optimized); a free
  // return is one straight line.
  EXPECT_LT(Median(queries), 50000) << name;
  EXPECT_LT(Max(queries), 150000) << name;
  EXPECT_LT(Max(total), 1e3 * MotionPlanOptions{}.time_budget) << name;
#ifdef NDEBUG
  // Optimized only: unoptimized MuJoCo is some four times slower.
  EXPECT_LT(Median(total), 1000.0) << name;
#endif
  // Never longer than the scripted path: the script's hand-chosen poses go
  // high over the pallet and out over the rails, the planner straight
  // round. The best of four seeds keeps out the rare wandering path that
  // shortcutting cannot straighten.
  EXPECT_LE(Max(length), scripted_length * (1.0 + 1e-9)) << name;
}

TEST_F(MotionPlannerTest, StatisticsCarryModule0) {
  Statistics("carry module_0 slot 0");
}
TEST_F(MotionPlannerTest, StatisticsCarryModule15) {
  Statistics("carry module_15 slot 5");
}
TEST_F(MotionPlannerTest, StatisticsCarryModule29) {
  Statistics("carry module_29 slot 9");
}
TEST_F(MotionPlannerTest, StatisticsReturn) {
  Statistics("return module_29 slot 9");
}

// ---------------------------------------------------------- which way round

// The smallest distance from the arm, tool and held module to the camera's
// post, boom and housing along a path, m.
std::pair<double, std::string> PostClearance(CollisionChecker& checker,
                                             const Cell& cell, int module,
                                             const JointPath& path) {
  const mjModel& m = *cell.model();
  const int post = mj_name2id(&m, mjOBJ_BODY, "camera_post");
  const std::string held = "module_" + std::to_string(module);
  std::vector<int> moving, fixed;
  for (int g = 0; g < m.ngeom; ++g) {
    if (m.geom_contype[g] == 0 && m.geom_conaffinity[g] == 0) continue;
    const int body = m.geom_bodyid[g];
    const char* name = mj_id2name(&m, mjOBJ_BODY, body);
    const std::string body_name = name != nullptr ? name : "";
    if (body == post) fixed.push_back(g);
    if (body_name.rfind("arm_", 0) == 0 || body_name == "tool" ||
        body_name == held) {
      moving.push_back(g);
    }
  }
  double smallest = 10.0;
  std::string between;
  for (const Eigen::VectorXd& q : cell::DenseSamples(checker, path, 0.05)) {
    checker.Check(q);  // poses the checker's scratch data at q
    // mj_geomDistance takes a writable mjData only for its scratch stack;
    // it leaves the state as it was.
    mjData* data = const_cast<mjData*>(&checker.data());
    for (int g : moving) {
      for (int h : fixed) {
        const double d =
            mj_geomDistance(&checker.model(), data, g, h, 10.0, nullptr);
        if (d < smallest) {
          smallest = d;
          // The module's geom has no name of its own; its body has.
          const char* a = mj_id2name(&m, mjOBJ_GEOM, g);
          const char* b = mj_id2name(&m, mjOBJ_GEOM, h);
          between = std::string(a ? a : held.c_str()) + " - " + (b ? b : "?");
        }
      }
    }
  }
  return {smallest, between};
}

TEST_F(MotionPlannerTest, WhichWayRoundThePlannerTakesTheModule) {
  for (const char* name : {"carry module_0 slot 0", "carry module_29 slot 9"}) {
    Case& c = Find(name);
    CollisionChecker& checker = *c.checker;
    // The scripted goal, joint 1 at -180 degrees: round +Y. The same pose
    // with joint 1 at +180: round -Y, past the camera's post (and, with the
    // tube filling, over the modules already on it).
    Eigen::VectorXd other_way = c.goal;
    other_way[0] += 2.0 * kPi;
    ASSERT_LE(other_way[0], cell_->arm().upper[0]);

    struct Way {
      const char* name;
      Eigen::VectorXd goal;
      bool both = false;
    };
    const Way ways[] = {{"+Y (scripted goal)", c.goal},
                        {"-Y (joint 1 +360)", other_way},
                        {"both, shorter kept", c.goal, true}};
    std::printf("%s:\n", name);
    for (const Way& way : ways) {
      std::vector<double> length, post, duration;
      std::string nearest;
      int found = 0;
      int took_other = 0;
      for (std::uint32_t s = 1; s <= 10; ++s) {
        MotionPlanOptions options;
        options.base_seed = s;
        options.try_both_ways = way.both;
        MotionPlanReport report;
        std::optional<JointPath> path =
            cell::PlanMove(checker, c.start, way.goal, options, &report);
        if (!path) continue;
        ++found;
        ExpectSound(c, *path, report, options);
        if (report.goal[0] > 0.0) ++took_other;
        length.push_back(report.length);
        const auto [to_post, between] =
            PostClearance(checker, *cell_, c.module, *path);
        if (post.empty() || to_post < Min(post)) nearest = between;
        post.push_back(to_post);
        std::optional<cell::Trajectory> trajectory = cell::Trajectory::Plan(
            *path, cell_->arm().limits,
            {.max_deviation = report.max_deviation, .speed = c.scripted_speed});
        if (trajectory) duration.push_back(trajectory->duration());
        // Which side joint 1 passed: it can only go through -90 to reach
        // -180 and through +90 to reach +180.
        double lowest = 10.0, highest = -10.0;
        for (const Eigen::VectorXd& q : *path) {
          lowest = std::min(lowest, q[0]);
          highest = std::max(highest, q[0]);
        }
        if (report.goal[0] < 0.0) {
          EXPECT_LT(lowest, -170.0 * kDegree);
          EXPECT_LT(highest, 30.0 * kDegree);
        } else {
          EXPECT_GT(highest, 170.0 * kDegree);
          EXPECT_GT(lowest, -30.0 * kDegree);
        }
      }
      // Both ways are open at the margins: the post stands 3.8 m out. But
      // round -Y the module passes within some 0.2 m of it, against 2 m
      // round +Y: the scripted side is the one to plan for.
      EXPECT_EQ(found, 10) << name << ", " << way.name;
      std::printf("  %-20s found %2d of 10, %2d round -Y; length m median "
                  "%.2f, closest to the camera's post %.2f m (%s), trajectory "
                  "s median %.2f\n",
                  way.name, found, took_other, Median(length), Min(post),
                  nearest.c_str(), Median(duration));
    }
  }

  Case& c = Find("carry module_29 slot 9");
  CollisionChecker& checker = *c.checker;
  // A via forces a way round, or any other pose on the way: here the
  // scripted swing's pose over the slot. The path passes through it
  // exactly and keeps every guarantee.
  const Eigen::VectorXd via = c.scripted[c.scripted.size() - 2];
  MotionPlanReport through_report;
  std::optional<JointPath> through = cell::PlanThrough(
      checker, c.start, {via}, c.goal, {}, &through_report);
  ASSERT_TRUE(through) << through_report.reason;
  EXPECT_TRUE(std::any_of(through->begin(), through->end(),
                          [&](const Eigen::VectorXd& q) {
                            return (q - via).norm() == 0.0;
                          }));
  through_report.goal = c.goal;
  ExpectSound(c, *through, through_report);
  std::printf("through the scripted swing's pose: %s\n",
              Describe(through_report).c_str());
}

// ----------------------------------------------------------------- failures

TEST_F(MotionPlannerTest, FailuresSayWhyAndLeaveTheCheckerAsItWas) {
  Case& c = Find("carry module_0 slot 0");
  CollisionChecker& checker = *c.checker;
  const double margin = checker.margin();
  MotionPlanReport report;

  // A goal with the module on the rails: the scripted set-down's end.
  cell::PickPlace script(*cell_);
  ASSERT_TRUE(script.Start(0, Step::kSetDown));
  const Eigen::VectorXd on_rails = script.Waypoints(Step::kSetDown)->back();
  EXPECT_FALSE(cell::PlanMove(checker, c.start, on_rails, {}, &report));
  EXPECT_EQ(report.status, MotionPlanStatus::kGoalNotClear);
  EXPECT_NE(report.reason.find("(tube)"), std::string::npos) << report.reason;
  std::printf("goal on the rails: %s\n", report.reason.c_str());

  // The other way round: the start too close.
  EXPECT_FALSE(cell::PlanMove(checker, on_rails, c.goal, {}, &report));
  EXPECT_EQ(report.status, MotionPlanStatus::kStartNotClear);

  // Out of range.
  Eigen::VectorXd wound = c.goal;
  wound[0] = cell_->arm().upper[0] + 0.1;
  EXPECT_FALSE(cell::PlanMove(checker, c.start, wound, {}, &report));
  EXPECT_EQ(report.status, MotionPlanStatus::kGoalOutOfRange);
  EXPECT_FALSE(
      cell::PlanMove(checker, c.start, Eigen::VectorXd::Zero(5), {}, &report));
  EXPECT_EQ(report.status, MotionPlanStatus::kBadInput);

  // Starved of iterations: every seed of every round fails, and it says
  // so. One iteration cannot cross the carry (the straight line is
  // blocked: it swings the module through the arm).
  MotionPlanOptions starved;
  starved.max_iterations = 1;
  starved.rounds = 2;
  EXPECT_FALSE(cell::PlanMove(checker, c.start, c.goal, starved, &report));
  EXPECT_EQ(report.status, MotionPlanStatus::kNotFound);
  EXPECT_EQ(report.rounds, 2);
  EXPECT_EQ(report.plans_tried, 2 * starved.seeds);
  EXPECT_EQ(report.last_rrt_status, cell::RrtStatus::kIterationLimit);
  std::printf("starved: %s\n", report.reason.c_str());

  // Out of time.
  MotionPlanOptions hurried;
  hurried.time_budget = 1e-4;
  EXPECT_FALSE(cell::PlanMove(checker, c.start, c.goal, hurried, &report));
  EXPECT_EQ(report.status, MotionPlanStatus::kTimeBudget);

  // A margin budget the checker cannot take (self more than twice world).
  MotionPlanOptions lopsided;
  lopsided.required_self_margin = 1.0;
  EXPECT_FALSE(cell::PlanMove(checker, c.start, c.goal, lopsided, &report));
  EXPECT_EQ(report.status, MotionPlanStatus::kBadInput);

  // Whatever happened, the checker's margins are as they were.
  EXPECT_EQ(checker.margin(), margin);
  EXPECT_EQ(checker.self_margin(), 0.0);
}

// ------------------------------------------------------- the timed path

TEST_F(MotionPlannerTest, RoundedTrajectoryIsClearWhenSampledDensely) {
  for (const char* name : {"carry module_0 slot 0", "carry module_29 slot 9",
                           "return module_29 slot 9"}) {
    Case& c = Find(name);
    CollisionChecker& checker = *c.checker;
    MotionPlanOptions options;
    options.speed = c.scripted_speed;
    MotionPlanReport report;
    std::optional<JointPath> path =
        cell::PlanMove(checker, c.start, c.goal, options, &report);
    ASSERT_TRUE(path) << report.reason;
    std::optional<cell::Trajectory> trajectory = cell::Trajectory::Plan(
        *path, cell_->arm().limits,
        {.max_deviation = report.max_deviation, .speed = options.speed});
    ASSERT_TRUE(trajectory);
    EXPECT_LE(trajectory->deviation(), report.max_deviation + 1e-12);

    // The rounded path, not the polyline, sampled in time so that no point
    // of the arm or module moves more than 5 mm between samples. It must
    // keep the required margin plus the tracking allowance (what is left
    // for the real arm's error), plus half the sample spacing.
    const double spacing = 0.005;
    const double keep = options.required_margin + options.tracking_allowance +
                        0.5 * spacing;
    ASSERT_TRUE(checker.SetMargins(keep, options.required_self_margin +
                                             options.tracking_allowance +
                                             0.5 * spacing));
    long samples = 0;
    long blocked = 0;
    double smallest = std::numeric_limits<double>::infinity();
    Eigen::VectorXd before = trajectory->At(0.0).position;
    double t = 0.0;
    double dt = 1e-3;
    cell::CheckOptions measure;
    measure.clearance = true;
    while (t < trajectory->duration()) {
      Eigen::VectorXd q = trajectory->At(t + dt).position;
      // Halve the time step until the sweep is within the spacing; grow it
      // again where the arm is slow.
      while (checker.SweepBound(before, q) > spacing && dt > 1e-6) {
        dt *= 0.5;
        q = trajectory->At(t + dt).position;
      }
      t += dt;
      ++samples;
      if (!checker.IsFree(q)) ++blocked;
      // The clearance, every 20th sample (it costs four times a query).
      if (samples % 20 == 0) {
        std::optional<cell::CollisionReport> r = checker.Check(q, measure);
        if (r && r->clearance) smallest = std::min(smallest, *r->clearance);
      }
      before = q;
      if (checker.SweepBound(before, trajectory->At(t + 2 * dt).position) <
          spacing) {
        dt *= 2.0;
      }
    }
    ASSERT_TRUE(checker.SetMargins(options.required_margin,
                                   options.required_self_margin));
    EXPECT_EQ(blocked, 0) << name << ", of " << samples;
    EXPECT_EQ((trajectory->At(trajectory->duration()).position - c.goal).norm(),
              0.0);
    const double scripted =
        StopAndGoDuration(c.scripted, cell_->arm().limits, c.scripted_speed);
    std::printf("%-24s trajectory %.2f s through %zu waypoints (%d corners "
                "rounded, deviation %.3f mrad of %.3f), %ld samples at "
                "<= 5 mm, smallest clearance %.1f mm; scripted moves %.2f s\n",
                name, trajectory->duration(), trajectory->waypoints().size(),
                trajectory->blends(), 1e3 * trajectory->deviation(),
                1e3 * report.max_deviation, samples, 1e3 * smallest, scripted);
    if (c.carry) {
      EXPECT_LT(trajectory->duration(), scripted) << name;
    }
  }
}

// ------------------------------------------------------- in the simulation

TEST_F(MotionPlannerTest, PlannedCarryRunsInTheCellWithTheModuleHeld) {
  // A cell of its own: this test simulates, and the suite's cell has its
  // modules parked.
  const char* path = std::getenv("MODEL_PATH");
  std::string error;
  std::optional<Cell> loaded = Cell::Load(path, &error);
  ASSERT_TRUE(loaded) << error;
  Cell& cell = *loaded;
  ASSERT_TRUE(cell.Reset(1, cell::Randomization::None()));
  const cell::Arm& arm = cell.arm();

  // The scripted cycle for module_0 up to the end of "clear": the cell's
  // own grip, the module really held by the weld.
  cell::PickPlace script(cell);
  script.set_pause_after(Step::kClear);
  ASSERT_TRUE(script.Start(0)) << script.outcome().reason;
  cell::CycleCheck check(cell);
  while (!script.paused() && !script.Done() && cell.data()->time < 60.0) {
    script.Update();
    cell.Step();
    check.Observe(script.step(), 0);
  }
  ASSERT_TRUE(script.paused()) << script.outcome().reason;
  ASSERT_EQ(cell.module_state(0), cell::ModuleState::kHeld);
  const double scripted_clear = script.records().back().duration();

  // What the pick and place will hand the planner: the arm's commanded
  // configuration, the goal (the start of "lower"), the module held at the
  // design grip, the obstacles where they are (the truth here: the test
  // has no camera).
  const Eigen::VectorXd start = script.follower().Goal(*cell.data());
  const Eigen::VectorXd goal = script.Waypoints(Step::kTurn)->back();
  ASSERT_EQ((start - script.Waypoints(Step::kClear)->back()).norm(), 0.0);
  // The design grip for module_0, as everywhere else in this file (the
  // same nominal scene).
  const Eigen::Isometry3d grip = grips_[0];

  std::optional<CollisionChecker> checker =
      CollisionChecker::Create(*cell.model(), {}, &error);
  ASSERT_TRUE(checker) << error;
  cell::PlanScene scene;
  scene.world = cell.data();
  scene.tube_slide = cell.conveyor().SensorPosition(*cell.data());
  scene.held_module = 0;
  scene.module_in_tip = grip;
  MotionPlanOptions options;
  options.speed = script.options().speed_carrying;
  MotionPlanReport report;
  std::optional<cell::TimedMove> move = cell::PlanArmMove(
      *checker, scene, start, goal, arm.limits, options, &report);
  ASSERT_TRUE(move) << report.reason;
  std::printf("planned carry: %s; trajectory %.2f s\n",
              Describe(report).c_str(), move->trajectory.duration());

  // Take over from the script: a follower of our own, holding exactly the
  // commanded configuration (it adopts ctrl as its held pose, so ctrl is
  // set to the plain angles first; its own Update overwrites them before
  // the next step), then following the planned trajectory.
  for (int j = 0; j < cell::Arm::kJoints; ++j) {
    cell.data()->ctrl[arm.motor[j]] = start[j];
  }
  cell::ArmFollower follower(arm);
  std::string reason;
  ASSERT_TRUE(follower.Follow(move->trajectory, *cell.data(), &reason))
      << reason;
  const double began = cell.data()->time;
  const Eigen::VectorXd weights = checker->LeverArms();
  double worst_joint_error = 0.0;
  double worst_point_error = 0.0;
  // How far the module's far corners get from where the checker puts them
  // (the tool tip at the measured angles times the grip): the weld's give.
  double worst_weld_give = 0.0;
  const Eigen::Vector3d corner(0.5 * Conveyor::kModuleLength,
                               0.5 * Conveyor::kModuleWidth, 0.0);
  while (cell.data()->time - began < move->trajectory.duration() + 1.0) {
    follower.Update(*cell.model(), cell.data());
    cell.Step();
    // Labelled "swing": carrying, after the pull-back and before the
    // set-down, so the module may touch only the tool.
    check.Observe(Step::kSwing, 0);
    const Eigen::VectorXd error = follower.TrackingError(*cell.data());
    worst_joint_error = std::max(worst_joint_error, error.cwiseAbs().maxCoeff());
    worst_point_error =
        std::max(worst_point_error, error.cwiseAbs().dot(weights));
    const Eigen::Isometry3d expected =
        cell::TipPose(arm, *cell.data()) * grip;
    const Eigen::Isometry3d actual = cell.TrueModulePose(0);
    for (double sx : {-1.0, 1.0}) {
      for (double sy : {-1.0, 1.0}) {
        const Eigen::Vector3d p(sx * corner.x(), sy * corner.y(), 0.0);
        worst_weld_give =
            std::max(worst_weld_give, (expected * p - actual * p).norm());
      }
    }
  }
  EXPECT_TRUE(follower.Done(*cell.data()));
  EXPECT_EQ(cell.module_state(0), cell::ModuleState::kHeld);
  EXPECT_TRUE(cell.suction().holding().has_value());
  EXPECT_LT((cell::JointAngles(arm, *cell.data()) - goal).cwiseAbs().maxCoeff(),
            cell::ArmFollower::kArrivedAngle);

  // Nothing touched; no motor saturated; the joints tracked within the
  // follower's arrival tolerance, and every point within the tracking
  // allowance of the margin budget.
  EXPECT_EQ(check.forbidden_count(), 0)
      << (check.forbidden().empty() ? "" : check.forbidden()[0].between);
  EXPECT_EQ(check.saturated_steps(), 0);
  EXPECT_EQ(check.out_of_range_steps(), 0);
  EXPECT_EQ(check.non_finite_steps(), 0);
  const Eigen::VectorXd torque = check.peak_torque(Step::kSwing);
  EXPECT_LT(torque.maxCoeff(), 0.95);
  EXPECT_LT(worst_joint_error, cell::ArmFollower::kArrivedAngle);
  EXPECT_LT(worst_point_error, options.tracking_allowance);

  using cell::Clearance;
  double module_closest = 1.0;
  std::string module_between;
  for (Clearance k : {Clearance::kModuleArm, Clearance::kModuleStack,
                      Clearance::kModuleConveyor, Clearance::kModulePost,
                      Clearance::kModuleFloor}) {
    const cell::ClosestApproach& a = check.closest(k, Step::kSwing);
    if (a.measured && a.distance < module_closest) {
      module_closest = a.distance;
      module_between = std::string(cell::ClearanceName(k)) + ": " + a.between;
    }
  }
  double arm_closest = 1.0;
  for (Clearance k : {Clearance::kArmStack, Clearance::kArmConveyor,
                      Clearance::kArmPost, Clearance::kArmFloor}) {
    const cell::ClosestApproach& a = check.closest(k, Step::kSwing);
    if (a.measured) arm_closest = std::min(arm_closest, a.distance);
  }
  // The world clearances of the truth keep the required margin.
  for (Clearance k : {Clearance::kModuleStack, Clearance::kModuleConveyor,
                      Clearance::kModulePost, Clearance::kModuleFloor,
                      Clearance::kArmStack, Clearance::kArmConveyor,
                      Clearance::kArmPost, Clearance::kArmFloor}) {
    const cell::ClosestApproach& a = check.closest(k, Step::kSwing);
    if (a.measured) {
      EXPECT_GE(a.distance, options.required_margin)
          << cell::ClearanceName(k) << ": " << a.between;
    }
  }
  std::printf(
      "executed carry: %.2f s planned (scripted clear before it %.2f s), "
      "%d forbidden contacts, peak torque in the carry %.2f %.2f %.2f %.2f "
      "%.2f %.2f of the limits, worst tracking error %.3f mrad (any point "
      "%.2f mm, allowance %.0f mm), weld give at the corners %.2f mm\n"
      "  held module's closest approach %.1f mm (%s); arm and tool %.1f mm\n",
      move->trajectory.duration(), scripted_clear,
      static_cast<int>(check.forbidden_count()), torque[0], torque[1],
      torque[2], torque[3], torque[4], torque[5], 1e3 * worst_joint_error,
      1e3 * worst_point_error, 1e3 * options.tracking_allowance,
      1e3 * worst_weld_give, 1e3 * module_closest, module_between.c_str(),
      1e3 * arm_closest);
}

}  // namespace
