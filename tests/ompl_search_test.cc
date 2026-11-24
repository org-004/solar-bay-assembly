// OMPL's search (src/ompl_search.h) against the planner's own (src/rrt.h):
// that OMPL is given the same metric and the same edge check, that a seed
// gives a path again, that a name it does not know is refused; then the
// switch (MotionPlanOptions::search): the cell's carry planned with OMPL
// chosen, and one whole cycle run with it.
//
// Nothing here bounds a wall-clock time, and every time limit (of a search,
// and PlanMove's time budget) is set so high that a count always ends a
// search first. Random configurations come
// from src/portable_random.h, one draw per statement, so both platforms
// test the same ones.

#include "src/ompl_search.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/cell.h"
#include "src/collision.h"
#include "src/conveyor.h"
#include "src/motion_planner.h"
#include "src/pick_place.h"
#include "src/portable_random.h"
#include "src/rrt.h"
#include "src/rrt_worlds.h"
#include "src/score.h"
#include "src/trial_runner.h"

namespace {

using cell::Cell;
using cell::CollisionChecker;
using cell::JointPath;
using cell::MotionPlanOptions;
using cell::MotionPlanReport;
using cell::MotionPlanStatus;
using cell::OmplReport;
using cell::OmplSettings;
using cell::OmplSpace;
using cell::OmplStop;
using cell::PlanSearch;
using cell::RrtOptions;
using cell::RrtReport;
using cell::RrtStatus;
using cell::ToyWorld;
using Step = cell::PickPlaceStep;

// A time limit no search here comes near: the counts are the limits.
constexpr double kNoTimeLimit = 3600.0;

// Weights of the size of the cell's lever arms (metres per radian), all
// different, so that a mistake in the scaling of any joint shows.
Eigen::VectorXd SixWeights() {
  Eigen::VectorXd w(6);
  w << 3.9, 3.4, 2.6, 1.7, 1.5, 0.8;
  return w;
}

// A configuration drawn evenly from the box.
Eigen::VectorXd Draw(std::mt19937& rng, const Eigen::VectorXd& lower,
                     const Eigen::VectorXd& upper) {
  Eigen::VectorXd q(lower.size());
  for (Eigen::Index i = 0; i < q.size(); ++i) {
    q[i] = cell::UniformReal(rng, lower[i], upper[i]);
  }
  return q;
}

// The options the six-joint toy world is planned with in these tests.
RrtOptions ToyOptions(std::uint32_t seed) {
  RrtOptions options;
  options.step_size = 1.0;
  options.edge_resolution = 0.02;
  options.max_seconds = kNoTimeLimit;
  options.seed = seed;
  return options;
}

bool SamePath(const JointPath& a, const JointPath& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].size() != b[i].size() || (a[i].array() != b[i].array()).any()) {
      return false;
    }
  }
  return true;
}

// ------------------------------------------------------------ the problem

// R5. The distance OMPL's space gives is the planner's weighted distance.
TEST(OmplSearch, DistanceIsThePlannersMetric) {
  const ToyWorld world = cell::ArmLikeWorld();
  RrtOptions weighted;
  weighted.weights = SixWeights();
  const OmplSpace space(world.lower, world.upper, world.is_free, weighted);
  const OmplSpace plain(world.lower, world.upper, world.is_free, RrtOptions());
  ASSERT_TRUE(space.ok());
  ASSERT_TRUE(plain.ok());

  std::mt19937 rng(20261124);
  double largest = 0.0;
  for (int i = 0; i < 200; ++i) {
    const Eigen::VectorXd a = Draw(rng, world.lower, world.upper);
    const Eigen::VectorXd b = Draw(rng, world.lower, world.upper);
    const double expected = cell::Distance(a, b, weighted.weights);
    EXPECT_NEAR(space.Distance(a, b), expected, 1e-9) << "pair " << i;
    EXPECT_NEAR(plain.Distance(a, b), cell::Distance(a, b), 1e-9)
        << "pair " << i << ", no weights";
    largest = std::max(largest, expected);
  }
  // The pairs were not all close together.
  EXPECT_GT(largest, 10.0);
  EXPECT_DOUBLE_EQ(space.Distance(world.start, world.start), 0.0);
}

// R6. OMPL's motion validator gives EdgeIsFree's verdict on every edge and
// makes the same number of IsFree calls on every free one; both of its
// functions do. Half the edges are long (two configurations anywhere in
// the box, mostly blocked), half short (the far end near the near one,
// mostly free), so both verdicts are well represented.
TEST(OmplSearch, EdgesCheckedLikeEdgeIsFree) {
  const ToyWorld world = cell::ArmLikeWorld();
  RrtOptions options;
  options.weights = SixWeights();
  options.edge_resolution = 0.05;
  const OmplSpace space(world.lower, world.upper, world.is_free, options);
  ASSERT_TRUE(space.ok());

  std::mt19937 rng(424242);
  int free_edges = 0;
  int blocked_edges = 0;
  std::int64_t free_calls = 0;
  for (int i = 0; i < 400; ++i) {
    SCOPED_TRACE("edge " + std::to_string(i));
    const Eigen::VectorXd from = Draw(rng, world.lower, world.upper);
    Eigen::VectorXd to = Draw(rng, world.lower, world.upper);
    if (i % 2 == 1) {
      // A short edge: a tenth of the way toward the second draw.
      to = from + 0.1 * (to - from);
    }
    std::int64_t expected_calls = 0;
    const bool expected = cell::EdgeIsFree(
        from, to, world.is_free, options.edge_resolution, options.weights,
        options.edge_order, &expected_calls);

    std::int64_t calls = 0;
    const bool verdict = space.CheckMotion(from, to, &calls);
    EXPECT_EQ(verdict, expected);

    std::int64_t calls_with_last = 0;
    Eigen::VectorXd last_valid;
    double fraction = -1.0;
    const bool verdict_with_last =
        space.CheckMotion(from, to, &last_valid, &fraction, &calls_with_last);
    EXPECT_EQ(verdict_with_last, expected);
    ASSERT_EQ(last_valid.size(), from.size());

    if (expected) {
      ++free_edges;
      free_calls += expected_calls;
      EXPECT_EQ(calls, expected_calls);
      EXPECT_EQ(calls_with_last, expected_calls);
      EXPECT_DOUBLE_EQ(fraction, 1.0);
      EXPECT_LT((last_valid - to).cwiseAbs().maxCoeff(), 1e-12);
    } else {
      ++blocked_edges;
      // A blocked edge stops at its first point in collision, and the
      // points are checked in the same order, so the counts agree here too.
      EXPECT_EQ(calls, expected_calls);
      EXPECT_EQ(calls_with_last, expected_calls);
      EXPECT_DOUBLE_EQ(fraction, 0.0);
      EXPECT_LT((last_valid - from).cwiseAbs().maxCoeff(), 1e-12);
    }
  }
  std::printf("edges: %d free (%lld calls), %d blocked\n", free_edges,
              static_cast<long long>(free_calls), blocked_edges);
  EXPECT_GE(free_edges, 50);
  EXPECT_GE(blocked_edges, 50);
  // The free edges were real work: many points each, not one.
  EXPECT_GT(free_calls, 20 * free_edges);
}

// R4. Samples fall in the box given, and fill it.
TEST(OmplSearch, SamplesStayInTheBoxAndFillIt) {
  const ToyWorld world = cell::ArmLikeWorld();
  RrtOptions options;
  options.weights = SixWeights();
  options.seed = 7;
  OmplSpace space(world.lower, world.upper, world.is_free, options);
  ASSERT_TRUE(space.ok());
  const Eigen::VectorXd width = world.upper - world.lower;
  Eigen::VectorXd low = world.upper;
  Eigen::VectorXd high = world.lower;
  for (int i = 0; i < 2000; ++i) {
    const Eigen::VectorXd q = space.Sample();
    ASSERT_EQ(q.size(), world.lower.size());
    // Scaling there and back may move the last bit, no more.
    EXPECT_TRUE(((q - world.lower).array() >= -1e-12).all()) << q.transpose();
    EXPECT_TRUE(((world.upper - q).array() >= -1e-12).all()) << q.transpose();
    low = low.cwiseMin(q);
    high = high.cwiseMax(q);
  }
  for (Eigen::Index j = 0; j < width.size(); ++j) {
    EXPECT_LT(low[j], world.lower[j] + 0.02 * width[j]) << "joint " << j;
    EXPECT_GT(high[j], world.upper[j] - 0.02 * width[j]) << "joint " << j;
  }
  // The same seed draws the same samples; another seed does not.
  OmplSpace again(world.lower, world.upper, world.is_free, options);
  OmplSpace seeded(world.lower, world.upper, world.is_free, options);
  options.seed = 8;
  OmplSpace other(world.lower, world.upper, world.is_free, options);
  const Eigen::VectorXd first = again.Sample();
  EXPECT_TRUE((first.array() == seeded.Sample().array()).all());
  EXPECT_TRUE((first.array() != other.Sample().array()).any());
}

// ------------------------------------------------------------ the search

// R9 for RRTConnect. One seed planned twice gives the same path, the second
// time after other plans (of other seeds and of another planner) have been
// made in between; and the seed matters: five seeds do not all give one
// path.
TEST(OmplSearch, SameSeedSamePath) {
  const ToyWorld world = cell::ArmLikeWorld();
  auto plan = [&](std::uint32_t seed, const std::string& planner,
                  RrtReport* report) {
    OmplSettings settings;
    settings.planner = planner;
    return cell::PlanOmpl(world.start, world.goal, world.lower, world.upper,
                          world.is_free, ToyOptions(seed), settings, report);
  };

  RrtReport first_report;
  const std::optional<JointPath> first = plan(1, "RRTConnect", &first_report);
  ASSERT_TRUE(first.has_value()) << cell::ToString(first_report.status);

  std::vector<JointPath> paths = {*first};
  for (std::uint32_t seed = 2; seed <= 5; ++seed) {
    RrtReport report;
    const std::optional<JointPath> path = plan(seed, "RRTConnect", &report);
    ASSERT_TRUE(path.has_value())
        << "seed " << seed << ": " << cell::ToString(report.status);
    paths.push_back(*path);
  }
  RrtReport other_report;
  plan(1, "BiTRRT", &other_report);

  RrtReport again_report;
  const std::optional<JointPath> again = plan(1, "RRTConnect", &again_report);
  ASSERT_TRUE(again.has_value());
  EXPECT_TRUE(SamePath(*first, *again));
  EXPECT_EQ(first_report.collision_queries, again_report.collision_queries);
  EXPECT_EQ(first_report.path_length, again_report.path_length);

  int different = 0;
  for (std::size_t i = 1; i < paths.size(); ++i) {
    if (!SamePath(paths[0], paths[i])) ++different;
  }
  EXPECT_GE(different, 1) << "five seeds gave one path";
}

// R9 and R10. Searches running in several threads at once, each on a world
// of its own, give the paths the same seeds give alone.
TEST(OmplSearch, ThreadsPlanningAtOnceGiveTheSamePaths) {
  constexpr int kThreads = 4;
  auto plan = [](std::uint32_t seed, std::optional<JointPath>* path) {
    const ToyWorld world = cell::ArmLikeWorld();
    *path = cell::PlanOmpl(world.start, world.goal, world.lower, world.upper,
                           world.is_free, ToyOptions(seed));
  };
  std::vector<std::optional<JointPath>> alone(kThreads);
  for (int t = 0; t < kThreads; ++t) plan(11 + t, &alone[t]);

  std::vector<std::optional<JointPath>> together(kThreads);
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back(plan, 11 + t, &together[t]);
  }
  for (std::thread& thread : threads) thread.join();

  for (int t = 0; t < kThreads; ++t) {
    SCOPED_TRACE("seed " + std::to_string(11 + t));
    ASSERT_TRUE(alone[t].has_value());
    ASSERT_TRUE(together[t].has_value());
    EXPECT_TRUE(SamePath(*alone[t], *together[t]));
  }
}

// R7. A planner's name that is not known is refused with a reason, by the
// search and by PlanMove's options check alike; and each known name is one.
TEST(OmplSearch, UnknownPlannerIsRefused) {
  const ToyWorld world = cell::CircleWorld();
  for (const char* name : {"Nonsense", "", "rrtconnect", "RRTConnect "}) {
    SCOPED_TRACE(std::string("name \"") + name + "\"");
    EXPECT_FALSE(cell::IsOmplPlanner(name));
    EXPECT_FALSE(cell::OmplPlannerRepeats(name));
    OmplSettings settings;
    settings.planner = name;
    RrtReport report;
    OmplReport more;
    const std::optional<JointPath> path =
        cell::PlanOmpl(world.start, world.goal, world.lower, world.upper,
                       world.is_free, ToyOptions(1), settings, &report, &more);
    EXPECT_FALSE(path.has_value());
    EXPECT_EQ(report.status, RrtStatus::kBadInput);
    EXPECT_EQ(report.collision_queries, 0);
    EXPECT_EQ(more.stopped_by, OmplStop::kNotStarted);
    EXPECT_NE(more.reason.find("RRTConnect"), std::string::npos)
        << "the reason names what is known: " << more.reason;
  }
  const std::vector<std::string>& names = cell::OmplPlannerNames();
  ASSERT_GE(names.size(), 3u);
  EXPECT_EQ(names[0], "RRTConnect");
  EXPECT_EQ(OmplSettings().planner, "RRTConnect");
  for (const char* name : {"RRTConnect", "BiTRRT", "BITstar"}) {
    EXPECT_TRUE(cell::IsOmplPlanner(name)) << name;
  }
  EXPECT_TRUE(cell::OmplPlannerRepeats("RRTConnect"));
}

// R8. Every planner returns an exact path round the circle: it starts at
// the start and ends at the goal, bit for bit, and each of its segments
// passes the planner's own edge check. The planners that only seek a path
// stop by themselves; the ones that go on improving stop at the limit of
// queries. (The three that do not repeat a plan are still safe to ask this
// of: the world is one circle in an open square, and the limit is many
// times what a first path costs. InformedRRTstar, the dearest, found one
// within 10,000 queries in 20 runs of 20, against the 60,000 allowed.)
TEST(OmplSearch, EveryPlannerReturnsAnExactPathAndSaysWhatStoppedIt) {
  const ToyWorld world = cell::CircleWorld();
  for (const std::string& name : cell::OmplPlannerNames()) {
    SCOPED_TRACE(name);
    const bool improves =
        name == "BITstar" || name == "AORRTC" || name == "InformedRRTstar";
    RrtOptions options = ToyOptions(3);
    OmplSettings settings;
    settings.planner = name;
    settings.max_queries = 60000;
    RrtReport report;
    OmplReport more;
    const std::optional<JointPath> path =
        cell::PlanOmpl(world.start, world.goal, world.lower, world.upper,
                       world.is_free, options, settings, &report, &more);
    ASSERT_TRUE(path.has_value())
        << cell::ToString(report.status) << " " << more.reason;
    EXPECT_EQ(report.status, RrtStatus::kFound);
    ASSERT_GE(path->size(), 2u);
    EXPECT_TRUE((path->front().array() == world.start.array()).all());
    EXPECT_TRUE((path->back().array() == world.goal.array()).all());
    for (std::size_t i = 1; i < path->size(); ++i) {
      EXPECT_TRUE(cell::EdgeIsFree((*path)[i - 1], (*path)[i], world.is_free,
                                   options.edge_resolution))
          << "segment " << i;
    }
    EXPECT_NEAR(report.path_length, cell::PathLength(*path), 1e-12);
    EXPECT_GE(report.path_length, world.shortest - 1e-9);
    EXPECT_GT(report.collision_queries, 2);
    if (improves) {
      EXPECT_EQ(more.stopped_by, OmplStop::kQueryLimit);
      EXPECT_GE(report.collision_queries, settings.max_queries);
    } else {
      EXPECT_EQ(more.stopped_by, OmplStop::kSolved);
      EXPECT_LT(report.collision_queries, settings.max_queries);
    }
    std::printf("%-16s %6lld queries, length %.3f (shortest %.3f), %s\n",
                name.c_str(), static_cast<long long>(report.collision_queries),
                report.path_length, world.shortest,
                cell::ToString(more.stopped_by));
  }
}

// R8. Where there is no path the search ends at its limit of queries and
// says so; it does not hang and it does not return a path that stops short.
TEST(OmplSearch, NoPathEndsAtTheQueryLimit) {
  const ToyWorld world = cell::EnclosedGoalWorld();
  for (const char* name : {"RRTConnect", "BiTRRT", "BITstar"}) {
    SCOPED_TRACE(name);
    OmplSettings settings;
    settings.planner = name;
    settings.max_queries = 5000;
    RrtReport report;
    OmplReport more;
    const std::optional<JointPath> path =
        cell::PlanOmpl(world.start, world.goal, world.lower, world.upper,
                       world.is_free, ToyOptions(1), settings, &report, &more);
    EXPECT_FALSE(path.has_value());
    EXPECT_EQ(report.status, RrtStatus::kIterationLimit);
    EXPECT_EQ(more.stopped_by, OmplStop::kQueryLimit);
    EXPECT_GE(report.collision_queries, settings.max_queries);
    // Overrun by one iteration's work at most, which is far less than this.
    EXPECT_LT(report.collision_queries, 2 * settings.max_queries);
  }
}

// The refusals PlanRrtConnect makes, made alike.
TEST(OmplSearch, RefusesWhatPlanRrtConnectRefuses) {
  const ToyWorld world = cell::CircleWorld();
  auto status = [&](const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
                    const RrtOptions& options, const OmplSettings& settings) {
    RrtReport report;
    EXPECT_FALSE(cell::PlanOmpl(start, goal, world.lower, world.upper,
                                world.is_free, options, settings, &report)
                     .has_value());
    return report.status;
  };
  const Eigen::VectorXd outside = world.upper + Eigen::VectorXd::Ones(2);
  const Eigen::VectorXd inside_circle = Eigen::Vector2d(5.0, 5.0);
  EXPECT_EQ(status(outside, world.goal, ToyOptions(1), {}),
            RrtStatus::kStartOutOfBounds);
  EXPECT_EQ(status(world.start, outside, ToyOptions(1), {}),
            RrtStatus::kGoalOutOfBounds);
  EXPECT_EQ(status(inside_circle, world.goal, ToyOptions(1), {}),
            RrtStatus::kStartInCollision);
  EXPECT_EQ(status(world.start, inside_circle, ToyOptions(1), {}),
            RrtStatus::kGoalInCollision);
  RrtOptions bad = ToyOptions(1);
  bad.edge_resolution = 0.0;
  EXPECT_EQ(status(world.start, world.goal, bad, {}), RrtStatus::kBadInput);
  OmplSettings no_queries;
  no_queries.max_queries = 0;
  EXPECT_EQ(status(world.start, world.goal, ToyOptions(1), no_queries),
            RrtStatus::kBadInput);
  OmplSettings negative_range;
  negative_range.range = -1.0;
  EXPECT_EQ(status(world.start, world.goal, ToyOptions(1), negative_range),
            RrtStatus::kBadInput);
  // A range of zero is OMPL's own choice, and plans.
  OmplSettings own_range;
  own_range.range = 0.0;
  EXPECT_TRUE(cell::PlanOmpl(world.start, world.goal, world.lower, world.upper,
                             world.is_free, ToyOptions(1), own_range)
                  .has_value());
}

// ---------------------------------------------------- the switch, the cell

std::optional<Cell> LoadCell(std::string* error) {
  const char* path = std::getenv("MODEL_PATH");
  if (path == nullptr) {
    *error = "MODEL_PATH is not set";
    return std::nullopt;
  }
  return Cell::Load(path, error);
}

// print_plan's default move: module 0, held as the grip is designed, carried
// from the end of the scripted "clear" to the end of its "turn", in the
// nominal scene with the tube at slot 0.
struct Carry {
  std::optional<CollisionChecker> checker;
  Eigen::VectorXd start;
  Eigen::VectorXd goal;
  double speed = 0.9;
};

void SetUpCarry(Cell& cell, Carry* carry) {
  cell.Reset(1, cell::Randomization::None());
  cell::PickPlace script(cell);
  ASSERT_TRUE(script.Start(0, Step::kSetDown)) << script.outcome().reason;
  cell.PutArm(script.Waypoints(Step::kTouchGlass)->back());
  const Eigen::Isometry3d grip =
      cell::TipPose(cell.arm(), *cell.data()).inverse() *
      cell.TrueModulePose(0);
  cell.PutArm(cell.arm_home());

  std::string error;
  carry->checker = CollisionChecker::Create(*cell.model(), {}, &error);
  ASSERT_TRUE(carry->checker.has_value()) << error;
  carry->checker->SetScene(*cell.data());
  carry->checker->SetTubeSlide(cell::Conveyor::NominalSlidePosition(0));
  carry->checker->SetHeldModule(0, grip);
  carry->start = script.Waypoints(Step::kClear)->back();
  carry->goal = script.Waypoints(Step::kTurn)->back();
  carry->speed = script.options().speed_carrying;
}

// R11. With OMPL chosen, PlanMove finds the cell's default carry, and the
// path it keeps has the clearance the margin budget promises of a path
// whose edges were checked every edge_resolution: the planning margin less
// half the resolution, to the world and among the arm and module.
TEST(OmplSearch, PlanMovePlansTheCarry) {
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded.has_value()) << error;
  Carry carry;
  ASSERT_NO_FATAL_FAILURE(SetUpCarry(*loaded, &carry));

  MotionPlanOptions options;
  options.speed = carry.speed;
  options.time_budget = kNoTimeLimit;
  options.search = PlanSearch::kOmpl;
  MotionPlanReport report;
  const std::optional<JointPath> path = cell::PlanMove(
      *carry.checker, carry.start, carry.goal, options, &report);
  ASSERT_TRUE(path.has_value())
      << cell::ToString(report.status) << ": " << report.reason;
  EXPECT_EQ(report.status, MotionPlanStatus::kFound);
  EXPECT_GE(report.plans_found, 1);
  EXPECT_GT(report.search_queries, 0);
  EXPECT_TRUE((path->front().array() == carry.start.array()).all());
  EXPECT_TRUE((path->back().array() == carry.goal.array()).all());
  std::printf("carry with OMPL: %lld + %lld queries, length %.3f m, "
              "clearance %.1f mm (%s), self %.1f mm\n",
              static_cast<long long>(report.search_queries),
              static_cast<long long>(report.shorten_queries), report.length,
              1e3 * report.clearance, report.clearance_between.c_str(),
              1e3 * report.self_clearance);
  EXPECT_GE(report.clearance,
            report.planning_margin - 0.5 * options.edge_resolution);
  EXPECT_GE(report.self_clearance,
            report.planning_self_margin - 0.5 * options.edge_resolution);

  // The same call again gives the same path: the base seed fixes it.
  MotionPlanReport again_report;
  const std::optional<JointPath> again = cell::PlanMove(
      *carry.checker, carry.start, carry.goal, options, &again_report);
  ASSERT_TRUE(again.has_value());
  EXPECT_TRUE(SamePath(*path, *again));

  // The switch is what chooses: a planner's name OMPL's search does not
  // know fails the plan when OMPL is chosen, and is not looked at when the
  // own search is.
  options.ompl.planner = "Nonsense";
  MotionPlanReport refused;
  EXPECT_FALSE(cell::PlanMove(*carry.checker, carry.start, carry.goal, options,
                              &refused)
                   .has_value());
  EXPECT_EQ(refused.status, MotionPlanStatus::kBadInput);
  EXPECT_NE(refused.reason.find("Nonsense"), std::string::npos)
      << refused.reason;
  EXPECT_EQ(refused.plans_tried, 0);
  options.search = PlanSearch::kOwn;
  MotionPlanReport own;
  EXPECT_TRUE(cell::PlanMove(*carry.checker, carry.start, carry.goal, options,
                             &own)
                  .has_value());
}

// One whole cycle of module 0 with the planned carry and OMPL chosen, run
// as tests/planned_carry_test.cc runs one (a randomized pallet, the camera,
// the script stepped with the simulation): the module ends placed, within
// the tolerance the gate judges a cycle by.
TEST(OmplSearch, OneCycleWithOmplPlaces) {
  constexpr std::uint32_t kCellSeed = 8;
  constexpr double kCycleLimit = 90.0;  // s of simulation
  std::string error;
  std::optional<Cell> loaded = LoadCell(&error);
  ASSERT_TRUE(loaded.has_value()) << error;
  Cell& cell = *loaded;
  cell.Reset(kCellSeed, cell::Randomization{});

  cell::PickPlaceOptions options;
  options.sensing = cell::SensingMode::kCamera;
  options.carry = cell::CarryMode::kPlanned;
  options.planner.search = PlanSearch::kOmpl;
  options.planner.time_budget = kNoTimeLimit;
  cell::PickPlace script(cell, options);
  std::optional<cell::PlacementError> placement;
  script.set_placed_hook([&](int module, int slot) {
    mj_forward(cell.model(), cell.data());
    placement = cell::MeasurePlacement(cell, module, slot);
  });

  ASSERT_TRUE(script.Start(0)) << script.outcome().reason;
  const double began = cell.data()->time;
  while (!script.Done() && cell.data()->time - began < kCycleLimit) {
    script.Update();
    cell.Step();
  }
  ASSERT_TRUE(script.Done()) << "timed out in " << script.step_name();
  const cell::PickPlaceOutcome& outcome = script.outcome();
  ASSERT_EQ(outcome.status, cell::PickPlaceOutcome::Status::kPlaced)
      << outcome.reason;

  // The carry was a planned one, and its plan was found.
  EXPECT_EQ(outcome.carry.mode, cell::CarryMode::kPlanned);
  EXPECT_TRUE(outcome.carry.attempted);
  EXPECT_TRUE(outcome.carry.found);
  ASSERT_FALSE(outcome.carry.legs.empty());
  for (const MotionPlanReport& leg : outcome.carry.legs) {
    EXPECT_EQ(leg.status, MotionPlanStatus::kFound) << leg.reason;
    EXPECT_GE(leg.clearance,
              leg.planning_margin - 0.5 * options.planner.edge_resolution);
  }

  // Judged as the runner judges a cycle.
  cell::CycleResult result;
  result.seed = kCellSeed;
  result.module = 0;
  result.placed = true;
  result.slot = outcome.slot;
  result.error = placement;
  result.unconfirmed = outcome.placement_unconfirmed;
  const cell::CycleVerdict verdict = cell::JudgeCycle(result);
  EXPECT_TRUE(verdict.in_tolerance) << verdict.why;
  ASSERT_TRUE(placement.has_value());
  std::printf("cycle with OMPL: %s; %.2f s\n",
              cell::Describe(*placement).c_str(),
              outcome.end_time - outcome.start_time);
}

}  // namespace
