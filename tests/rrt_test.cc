// Checks the planner of src/rrt.h on the toy worlds of src/rrt_worlds.h,
// where the right answer is known: paths are found and are free when
// checked far more finely than the planner checks them; every failure is
// reported with its own reason, promptly; the 6-joint world shaped like the
// cell is solved on every seed within a budget of collision queries;
// shortening never collides or lengthens and comes within a stated factor
// of the hand-worked shortest path; the same seed repeats the same path;
// and bisection rejects a colliding edge in fewer queries than marching.
//
//   bazel test //:rrt_test
//   bazel test --config=opt //:rrt_test --test_output=all   (the numbers)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "src/rrt.h"
#include "src/rrt_worlds.h"

namespace {

using cell::EdgeIsFree;
using cell::EdgeOrder;
using cell::JointPath;
using cell::PathLength;
using cell::PlanRrtConnect;
using cell::RrtOptions;
using cell::RrtReport;
using cell::RrtStatus;
using cell::ShortenOptions;
using cell::ShortenPath;
using cell::ShortenReport;
using cell::ToyWorld;
using Eigen::VectorXd;

VectorXd Vec(std::initializer_list<double> values) {
  VectorXd v(values.size());
  int i = 0;
  for (double x : values) v[i++] = x;
  return v;
}

// The planner promises that every point of an edge is within half the
// edge resolution (as a sum of joint changes) of a point it found free.
// The test re-checks every edge at a tenth of the resolution, against the
// obstacles shrunk by that half: what the promise says, and no more.
const double kResolution = RrtOptions().edge_resolution;
const double kMargin = kResolution / 2.0;
const double kDense = kResolution / 10.0;

// The same promise lets a path cut a corner by up to kMargin, and so come
// out a little shorter than the true shortest path round it. A path may
// beat the hand-worked shortest by this much and no more: a generous
// allowance for the few corners in each world.
const double kCornerCut = 4.0 * kMargin;

// How much longer than the hand-worked shortest path a shortened path may
// be, by world. Over 100 seeds the worst seen were 1.000, 1.013, 1.038,
// 1.031 and 1.088; the cup's mouth leaves the most to straighten.
const std::map<std::string, double> kShortenFactor = {
    {"open", 1.0 + 1e-9}, {"circle", 1.03},  {"gap", 1.06},
    {"goal_in_gap", 1.06}, {"bug_trap", 1.15},
};

// -1 if every configuration of `path` is free and every edge is free when
// checked every kDense against the shrunk obstacles; otherwise the index of
// the first configuration or edge that is not.
int FirstNotFree(const ToyWorld& world, const JointPath& path) {
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!world.is_free(path[i])) return static_cast<int>(i);
    if (i + 1 == path.size()) break;
    const VectorXd& a = path[i];
    const VectorXd& b = path[i + 1];
    const int n = std::max(
        1, static_cast<int>(std::ceil((b - a).lpNorm<1>() / kDense)));
    for (int k = 1; k < n; ++k) {
      if (!world.free_by(a + (static_cast<double>(k) / n) * (b - a),
                         kMargin)) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

double LargestGap(const JointPath& path) {
  double gap = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    gap = std::max(gap, (path[i] - path[i - 1]).norm());
  }
  return gap;
}

std::optional<JointPath> Plan(const ToyWorld& world,
                              const RrtOptions& options = {},
                              RrtReport* report = nullptr) {
  return PlanRrtConnect(world.start, world.goal, world.lower, world.upper,
                        world.is_free, options, report);
}

RrtOptions Seeded(std::uint32_t seed) {
  RrtOptions options;
  options.seed = seed;
  return options;
}

template <typename T>
T Median(std::vector<T> values) {
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

// ------------------------------------------------------------ path helpers

TEST(PathTest, LengthIsTheSumOfWeightedDistances) {
  const JointPath path = {Vec({0, 0}), Vec({3, 4}), Vec({3, 5})};
  EXPECT_DOUBLE_EQ(PathLength(path), 6.0);
  // Weights scale each joint's change before the distance is taken.
  EXPECT_DOUBLE_EQ(PathLength(path, Vec({2, 1})), std::hypot(6.0, 4.0) + 1.0);
  EXPECT_DOUBLE_EQ(PathLength({}), 0.0);
  EXPECT_DOUBLE_EQ(PathLength({Vec({1, 1})}), 0.0);
}

TEST(PathTest, ResampleKeepsEveryCornerAndSpacesEachSegmentEvenly) {
  const JointPath path = {Vec({0, 0}), Vec({1, 0}), Vec({1, 2})};
  const std::optional<JointPath> resampled = cell::Resample(path, 0.3);
  ASSERT_TRUE(resampled.has_value());
  // 1 / 0.3 rounds up to 4 parts, 2 / 0.3 to 7.
  ASSERT_EQ(resampled->size(), 1u + 4u + 7u);
  EXPECT_EQ(resampled->front(), path[0]);
  EXPECT_EQ((*resampled)[4], path[1]);
  EXPECT_EQ(resampled->back(), path[2]);
  EXPECT_LE(LargestGap(*resampled), 0.3);
  for (int i = 1; i <= 4; ++i) {
    EXPECT_NEAR(((*resampled)[i] - (*resampled)[i - 1]).norm(), 0.25, 1e-12);
  }
  for (int i = 5; i < 12; ++i) {
    EXPECT_NEAR(((*resampled)[i] - (*resampled)[i - 1]).norm(), 2.0 / 7.0,
                1e-12);
  }
  EXPECT_DOUBLE_EQ(PathLength(*resampled), PathLength(path));
}

TEST(PathTest, ResampleRefusesBadInput) {
  const JointPath path = {Vec({0, 0}), Vec({1, 0})};
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(cell::Resample(path, 0.0).has_value());
  EXPECT_FALSE(cell::Resample(path, -1.0).has_value());
  EXPECT_FALSE(cell::Resample(path, nan).has_value());
  EXPECT_FALSE(cell::Resample({}, 0.1).has_value());
  EXPECT_FALSE(cell::Resample({Vec({0, 0}), Vec({1, 0, 0})}, 0.1).has_value());
  EXPECT_FALSE(cell::Resample(path, 0.1, Vec({1, 1, 1})).has_value());
  EXPECT_FALSE(cell::Resample(path, 0.1, Vec({1, 0})).has_value());
}

// ------------------------------------------------------------- edge checks

// A thin wall across the square at x = 7.95 to 8.15, and the edge from
// (1, 5) to (9, 5) through it: 8 long, so 400 points at 0.02.
TEST(EdgeTest, BisectionRejectsACollidingEdgeInFewerQueries) {
  const ToyWorld world =
      cell::MakeWorld2d("wall", {}, {{Vec({7.95, 0}), Vec({8.15, 10})}});
  const VectorXd from = Vec({1, 5});
  const VectorXd to = Vec({9, 5});

  std::int64_t marching = 0;
  std::int64_t bisection = 0;
  EXPECT_FALSE(EdgeIsFree(from, to, world.is_free, 0.02, {},
                          EdgeOrder::kMarching, &marching));
  EXPECT_FALSE(EdgeIsFree(from, to, world.is_free, 0.02, {},
                          EdgeOrder::kBisection, &bisection));
  // Marching walks 347 free points, x = 1.02 to 7.94, before the wall.
  EXPECT_EQ(marching, 348);
  // Bisection: the end (x = 9), the middle (5), the quarters (3, 7), and
  // the eighths (2, 4, 6, 8), the last of which is in the wall.
  EXPECT_EQ(bisection, 8);
  std::printf("wall edge rejected after %lld queries marching, %lld by "
              "bisection\n",
              static_cast<long long>(marching),
              static_cast<long long>(bisection));

  // A free edge costs every point either way.
  marching = 0;
  bisection = 0;
  EXPECT_TRUE(EdgeIsFree(Vec({1, 1}), Vec({1, 9}), world.is_free, 0.02, {},
                         EdgeOrder::kMarching, &marching));
  EXPECT_TRUE(EdgeIsFree(Vec({1, 1}), Vec({1, 9}), world.is_free, 0.02, {},
                         EdgeOrder::kBisection, &bisection));
  EXPECT_EQ(marching, 400);
  EXPECT_EQ(bisection, 400);
}

// Over many random edges among the circle: both orders agree on every edge,
// and on the colliding ones bisection needs far fewer queries on average.
TEST(EdgeTest, BothOrdersAgreeAndBisectionIsCheaperOnAverage) {
  const ToyWorld world = cell::CircleWorld();
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> coordinate(0.0, 10.0);
  auto free_point = [&] {
    while (true) {
      const VectorXd q = Vec({coordinate(rng), coordinate(rng)});
      if (world.is_free(q)) return q;
    }
  };
  int colliding = 0;
  std::int64_t marching_total = 0;
  std::int64_t bisection_total = 0;
  for (int trial = 0; trial < 1000; ++trial) {
    const VectorXd a = free_point();
    const VectorXd b = free_point();
    std::int64_t marching = 0;
    std::int64_t bisection = 0;
    const bool by_marching = EdgeIsFree(a, b, world.is_free, kResolution, {},
                                        EdgeOrder::kMarching, &marching);
    const bool by_bisection = EdgeIsFree(a, b, world.is_free, kResolution, {},
                                         EdgeOrder::kBisection, &bisection);
    ASSERT_EQ(by_marching, by_bisection) << "trial " << trial;
    if (!by_marching) {
      ++colliding;
      marching_total += marching;
      bisection_total += bisection;
    }
  }
  ASSERT_GT(colliding, 100);
  const double marching_mean = static_cast<double>(marching_total) / colliding;
  const double bisection_mean =
      static_cast<double>(bisection_total) / colliding;
  std::printf("%d colliding edges: mean %.1f queries marching, %.1f by "
              "bisection\n",
              colliding, marching_mean, bisection_mean);
  EXPECT_LT(bisection_mean, 0.25 * marching_mean);
}

TEST(EdgeTest, BadResolutionIsNotFree) {
  const auto always = [](const VectorXd&) { return true; };
  EXPECT_FALSE(EdgeIsFree(Vec({0, 0}), Vec({1, 1}), always, 0.0));
  EXPECT_FALSE(EdgeIsFree(Vec({0, 0}), Vec({1, 1}), always, -0.1));
  EXPECT_TRUE(EdgeIsFree(Vec({0, 0}), Vec({1, 1}), always, 0.1));
}

// ------------------------------------------------------------- 2D worlds

// The main claim on every 2D world with a path, over many seeds.
TEST(PlanTest, FindsFreePathsInEvery2dWorld) {
  for (const ToyWorld& world : cell::SolvableWorlds2d()) {
    for (std::uint32_t seed = 1; seed <= 20; ++seed) {
      SCOPED_TRACE(testing::Message() << world.name << " seed " << seed);
      RrtReport report;
      cell::RrtTrees trees;
      const RrtOptions options = Seeded(seed);
      const std::optional<JointPath> path =
          PlanRrtConnect(world.start, world.goal, world.lower, world.upper,
                         world.is_free, options, &report, &trees);
      ASSERT_TRUE(path.has_value()) << cell::ToString(report.status);
      EXPECT_EQ(report.status, RrtStatus::kFound);
      EXPECT_EQ(path->front(), world.start);
      EXPECT_EQ(path->back(), world.goal);
      EXPECT_EQ(FirstNotFree(world, *path), -1);
      EXPECT_LE(LargestGap(*path), options.step_size + 1e-12);
      EXPECT_NEAR(report.path_length, PathLength(*path), 1e-12);
      // No path can beat the shortest worked out by hand, beyond cut
      // corners.
      EXPECT_GE(report.path_length, world.shortest - kCornerCut);

      // The trees handed back match the report.
      EXPECT_EQ(static_cast<int>(trees.start_nodes.size()),
                report.start_tree_size);
      EXPECT_EQ(static_cast<int>(trees.goal_nodes.size()),
                report.goal_tree_size);
      EXPECT_EQ(trees.start_nodes[0], world.start);
      EXPECT_EQ(trees.goal_nodes[0], world.goal);
      for (std::size_t i = 1; i < trees.start_parents.size(); ++i) {
        EXPECT_LT(trees.start_parents[i], static_cast<int>(i));
      }
    }
  }
}

TEST(PlanTest, NoObstaclesGivesTheStraightLine) {
  const ToyWorld world = cell::OpenWorld();
  // Tried first, the straight line is the answer, split into steps.
  RrtReport report;
  const std::optional<JointPath> direct = Plan(world, {}, &report);
  ASSERT_TRUE(direct.has_value());
  EXPECT_NEAR(PathLength(*direct), world.shortest, 1e-12);
  EXPECT_LE(LargestGap(*direct), RrtOptions().step_size + 1e-12);
  EXPECT_EQ(report.start_tree_size, 1);
  EXPECT_EQ(report.goal_tree_size, 1);

  // Grown by the trees instead, the raw path wanders; shortened, it is the
  // straight line again: the two ends and nothing between.
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    RrtOptions options = Seeded(seed);
    options.try_direct_first = false;
    const std::optional<JointPath> raw = Plan(world, options);
    ASSERT_TRUE(raw.has_value());
    ShortenOptions shorten;
    shorten.seed = seed;
    const std::optional<JointPath> shortened =
        ShortenPath(*raw, world.is_free, shorten);
    ASSERT_TRUE(shortened.has_value());
    ASSERT_EQ(shortened->size(), 2u);
    EXPECT_EQ(shortened->front(), world.start);
    EXPECT_EQ(shortened->back(), world.goal);
  }
}

TEST(PlanTest, StartEqualToGoalIsAPathOfTwo) {
  const ToyWorld world = cell::CircleWorld();
  const std::optional<JointPath> path = PlanRrtConnect(
      world.start, world.start, world.lower, world.upper, world.is_free);
  ASSERT_TRUE(path.has_value());
  ASSERT_EQ(path->size(), 2u);
  EXPECT_EQ((*path)[0], world.start);
  EXPECT_EQ((*path)[1], world.start);
}

// The narrow passages: a gap 0.2 wide in a wall 1 thick, five times
// narrower than a step, and the goal inside such a gap. Within the default
// 5,000 iterations the gap was found on 91 of 100 seeds; given 20,000, on
// all of these 20, the worst taking some 10,400. The goal inside the gap
// is easy: its own tree grows out of the gap's two ends.
TEST(PlanTest, NarrowGapsAreThreaded) {
  ToyWorld narrow = cell::GapWorld(0.2);
  narrow.name = "gap_0.2";
  for (const ToyWorld& world : {narrow, cell::GoalInGapWorld(0.2)}) {
    std::vector<int> iterations;
    for (std::uint32_t seed = 1; seed <= 20; ++seed) {
      SCOPED_TRACE(testing::Message() << world.name << " seed " << seed);
      RrtOptions options = Seeded(seed);
      options.max_iterations = 20000;
      RrtReport report;
      const std::optional<JointPath> path = Plan(world, options, &report);
      ASSERT_TRUE(path.has_value()) << cell::ToString(report.status);
      EXPECT_EQ(FirstNotFree(world, *path), -1);
      iterations.push_back(report.iterations);
    }
    std::printf("%s: median %d, worst %d iterations over 20 seeds\n",
                world.name.c_str(), Median(iterations),
                *std::max_element(iterations.begin(), iterations.end()));
  }
}

// --------------------------------------------------------------- failures

// There is no way in, and the planner can only say so by running out of
// iterations: no hanging, and the reason is the limit.
TEST(PlanTest, EnclosedGoalRunsOutOfIterationsPromptly) {
  const ToyWorld world = cell::EnclosedGoalWorld();
  RrtReport report;
  const auto began = std::chrono::steady_clock::now();
  EXPECT_FALSE(Plan(world, {}, &report).has_value());
  const double seconds = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - began)
                             .count();
  EXPECT_EQ(report.status, RrtStatus::kIterationLimit);
  EXPECT_EQ(report.iterations, RrtOptions().max_iterations);
  EXPECT_GT(report.goal_tree_size, 1);  // it grew, inside its ring
  // About 3 ms optimized and some 50 ms unoptimized here, for about 39,000
  // queries; on the arm the queries would cost the time.
  EXPECT_LT(seconds, 2.0);
  std::printf("enclosed goal: %s after %d iterations, %lld queries, %.1f ms; "
              "trees of %d and %d\n",
              cell::ToString(report.status), report.iterations,
              static_cast<long long>(report.collision_queries),
              1e3 * report.seconds, report.start_tree_size,
              report.goal_tree_size);
}

TEST(PlanTest, TimeLimitStopsAHopelessSearch) {
  const ToyWorld world = cell::EnclosedGoalWorld();
  RrtOptions options;
  options.max_iterations = std::numeric_limits<int>::max();
  options.max_seconds = 0.05;
  RrtReport report;
  EXPECT_FALSE(Plan(world, options, &report).has_value());
  EXPECT_EQ(report.status, RrtStatus::kTimeLimit);
  EXPECT_GE(report.seconds, 0.05);
  EXPECT_LT(report.seconds, 0.5);
}

TEST(PlanTest, EachFailureHasItsOwnReason) {
  const ToyWorld world = cell::CircleWorld();
  const VectorXd inside_circle = Vec({5, 5});
  const VectorXd outside_box = Vec({-1, 5});
  RrtReport report;
  auto status = [&](const VectorXd& start, const VectorXd& goal) {
    EXPECT_FALSE(PlanRrtConnect(start, goal, world.lower, world.upper,
                                world.is_free, {}, &report)
                     .has_value());
    return report.status;
  };
  EXPECT_EQ(status(inside_circle, world.goal), RrtStatus::kStartInCollision);
  EXPECT_EQ(report.collision_queries, 1);
  EXPECT_EQ(status(world.start, inside_circle), RrtStatus::kGoalInCollision);
  EXPECT_EQ(report.collision_queries, 2);
  // Out of bounds is found without asking is_free.
  EXPECT_EQ(status(outside_box, world.goal), RrtStatus::kStartOutOfBounds);
  EXPECT_EQ(report.collision_queries, 0);
  EXPECT_EQ(status(world.start, Vec({5, 10.5})), RrtStatus::kGoalOutOfBounds);
  EXPECT_EQ(report.collision_queries, 0);
  // Both wrong: bounds are looked at first.
  EXPECT_EQ(status(inside_circle, outside_box), RrtStatus::kGoalOutOfBounds);
}

TEST(PlanTest, BadInputIsRefused) {
  const ToyWorld world = cell::CircleWorld();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  RrtReport report;
  auto refused = [&](const VectorXd& start, const VectorXd& lower,
                     const VectorXd& upper, const RrtOptions& options,
                     const cell::IsFree& is_free) {
    const bool planned = PlanRrtConnect(start, world.goal, lower, upper,
                                        is_free, options, &report)
                             .has_value();
    return !planned && report.status == RrtStatus::kBadInput;
  };
  const RrtOptions good;
  EXPECT_FALSE(
      refused(world.start, world.lower, world.upper, good, world.is_free));

  EXPECT_TRUE(refused(Vec({1, 1, 1}), world.lower, world.upper, good,
                      world.is_free));
  EXPECT_TRUE(
      refused(Vec({1, nan}), world.lower, world.upper, good, world.is_free));
  EXPECT_TRUE(refused(world.start, world.upper, world.lower, good,
                      world.is_free));  // lower above upper
  EXPECT_TRUE(
      refused(world.start, Vec({0, 10}), world.upper, good, world.is_free));
  EXPECT_TRUE(refused(world.start, world.lower, world.upper, good, nullptr));

  RrtOptions options;
  options.step_size = 0.0;
  EXPECT_TRUE(
      refused(world.start, world.lower, world.upper, options, world.is_free));
  options = good;
  options.edge_resolution = -0.01;
  EXPECT_TRUE(
      refused(world.start, world.lower, world.upper, options, world.is_free));
  options = good;
  options.goal_bias = 1.5;
  EXPECT_TRUE(
      refused(world.start, world.lower, world.upper, options, world.is_free));
  options = good;
  options.max_iterations = 0;
  EXPECT_TRUE(
      refused(world.start, world.lower, world.upper, options, world.is_free));
  options = good;
  options.max_seconds = 0.0;
  EXPECT_TRUE(
      refused(world.start, world.lower, world.upper, options, world.is_free));
  options = good;
  options.weights = Vec({1, 0});
  EXPECT_TRUE(
      refused(world.start, world.lower, world.upper, options, world.is_free));
  options.weights = Vec({1, 1, 1});
  EXPECT_TRUE(
      refused(world.start, world.lower, world.upper, options, world.is_free));
}

// ------------------------------------------------------- the 6-joint world

// The stage's main claim, on a world shaped like the cell, over 100 seeds.
//
// The thresholds come from the time budget: with a collision query of
// 20 us, a plausible figure for the arm and module against the cell,
// 20,000 queries are 0.4 s ("well under a second") and 60,000 are 1.2 s
// ("a few seconds at worst"), planning and shortening together. Measured:
// planning a median of 9,700 and a worst of 35,000, shortening 6,400 and
// 9,500 more, together 16,400 and 43,000.
TEST(ArmLikeTest, EverySeedIsSolvedWithinTheQueryBudget) {
  const ToyWorld world = cell::ArmLikeWorld();
  const int seeds = 100;
  int found = 0;
  std::vector<std::int64_t> queries;
  std::vector<std::int64_t> shorten_queries;
  std::vector<std::int64_t> total_queries;
  std::vector<int> iterations;
  std::vector<int> nodes;
  std::vector<double> seconds;
  std::vector<double> raw_length;
  std::vector<double> short_length;
  for (std::uint32_t seed = 1; seed <= seeds; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    RrtReport report;
    const std::optional<JointPath> path = Plan(world, Seeded(seed), &report);
    if (!path) continue;
    ++found;
    ASSERT_EQ(path->front(), world.start);
    ASSERT_EQ(path->back(), world.goal);
    ASSERT_EQ(FirstNotFree(world, *path), -1);
    ASSERT_LE(LargestGap(*path), RrtOptions().step_size + 1e-12);

    ShortenOptions shorten;
    shorten.seed = seed;
    ShortenReport shortened;
    const std::optional<JointPath> short_path =
        ShortenPath(*path, world.is_free, shorten, &shortened);
    ASSERT_TRUE(short_path.has_value());
    ASSERT_EQ(FirstNotFree(world, *short_path), -1);
    ASSERT_LE(shortened.length_after, shortened.length_before);

    queries.push_back(report.collision_queries);
    shorten_queries.push_back(shortened.collision_queries);
    total_queries.push_back(report.collision_queries +
                            shortened.collision_queries);
    iterations.push_back(report.iterations);
    nodes.push_back(report.start_tree_size + report.goal_tree_size);
    seconds.push_back(report.seconds + shortened.seconds);
    raw_length.push_back(shortened.length_before);
    short_length.push_back(shortened.length_after);
  }
  EXPECT_EQ(found, seeds);
  ASSERT_GT(found, 0);
  const auto worst = [](const auto& values) {
    return *std::max_element(values.begin(), values.end());
  };
  std::printf(
      "6-joint world: %d/%d found\n"
      "  planning queries  median %lld worst %lld\n"
      "  shortening queries median %lld worst %lld\n"
      "  both              median %lld worst %lld\n"
      "  iterations        median %d worst %d\n"
      "  tree nodes        median %d worst %d\n"
      "  seconds (with this is_free) median %.5f worst %.5f\n"
      "  path length (rad) raw median %.2f, shortened median %.2f worst "
      "%.2f\n",
      found, seeds, static_cast<long long>(Median(queries)),
      static_cast<long long>(worst(queries)),
      static_cast<long long>(Median(shorten_queries)),
      static_cast<long long>(worst(shorten_queries)),
      static_cast<long long>(Median(total_queries)),
      static_cast<long long>(worst(total_queries)), Median(iterations),
      worst(iterations), Median(nodes), worst(nodes), Median(seconds),
      worst(seconds), Median(raw_length), Median(short_length),
      worst(short_length));
  EXPECT_LE(Median(total_queries), 20000);
  EXPECT_LE(worst(total_queries), 60000);
}

// Weights in metres per radian, roughly each joint's reach to the far
// corner of the module, with the resolution in metres to match (8 cm, as
// 0.02 rad is at 4 m). Still solved, and the length reported is the
// weighted one.
TEST(ArmLikeTest, WeightedMetricPlans) {
  const ToyWorld world = cell::ArmLikeWorld();
  for (std::uint32_t seed = 1; seed <= 10; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    RrtOptions options = Seeded(seed);
    options.weights = Vec({4.0, 3.5, 2.5, 1.3, 1.3, 1.2});
    options.step_size = 3.0;  // metres of motion, as 1 rad is at 3 m
    options.edge_resolution = 0.08;
    RrtReport report;
    const std::optional<JointPath> path = Plan(world, options, &report);
    ASSERT_TRUE(path.has_value()) << cell::ToString(report.status);
    EXPECT_NEAR(report.path_length, PathLength(*path, options.weights),
                1e-9);
    for (std::size_t i = 1; i < path->size(); ++i) {
      EXPECT_LE(cell::Distance((*path)[i - 1], (*path)[i], options.weights),
                options.step_size + 1e-9);
    }
  }
}

// ---------------------------------------------------------------- shortening

TEST(ShortenTest, NeverCollidesNeverLengthensAndNearsTheShortest) {
  for (const ToyWorld& world : cell::SolvableWorlds2d()) {
    std::vector<double> ratios;
    std::vector<std::int64_t> queries;
    for (std::uint32_t seed = 1; seed <= 50; ++seed) {
      SCOPED_TRACE(testing::Message() << world.name << " seed " << seed);
      RrtOptions options = Seeded(seed);
      options.try_direct_first = false;
      const std::optional<JointPath> raw = Plan(world, options);
      ASSERT_TRUE(raw.has_value());

      ShortenOptions shorten;
      shorten.seed = seed;
      ShortenReport report;
      const std::optional<JointPath> shortened =
          ShortenPath(*raw, world.is_free, shorten, &report);
      ASSERT_TRUE(shortened.has_value());
      EXPECT_EQ(shortened->front(), world.start);
      EXPECT_EQ(shortened->back(), world.goal);
      EXPECT_EQ(FirstNotFree(world, *shortened), -1);
      EXPECT_NEAR(report.length_before, PathLength(*raw), 1e-12);
      EXPECT_NEAR(report.length_after, PathLength(*shortened), 1e-12);
      EXPECT_LE(report.length_after, report.length_before);
      EXPECT_GE(report.length_after, world.shortest - kCornerCut);
      EXPECT_LE(report.length_after,
                kShortenFactor.at(world.name) * world.shortest);
      ratios.push_back(report.length_after / world.shortest);
      queries.push_back(report.collision_queries);
    }
    std::printf("%-12s shortened/shortest median %.4f worst %.4f; "
                "shortening queries median %lld\n",
                world.name.c_str(), Median(ratios),
                *std::max_element(ratios.begin(), ratios.end()),
                static_cast<long long>(Median(queries)));
  }
}

// A path that wanders but is free, shortened against a world where only
// that path's own corridor is known free: every shortcut outside it is
// refused, and the result stays inside.
TEST(ShortenTest, ShortcutsThatWouldCollideAreRefused) {
  // A corridor shaped like a U: down the left, along the bottom, up the
  // right. The straight line across the top is blocked.
  const ToyWorld world = cell::MakeWorld2d(
      "u", {}, {{Vec({2, 2}), Vec({8, 10})}});
  const JointPath path = {Vec({1, 9}), Vec({1, 1}), Vec({9, 1}), Vec({9, 9})};
  ShortenReport report;
  const std::optional<JointPath> shortened =
      ShortenPath(path, world.is_free, {}, &report);
  ASSERT_TRUE(shortened.has_value());
  EXPECT_EQ(FirstNotFree(world, *shortened), -1);
  // The corners can be cut toward the box's corners, but no more: the
  // shortest such path runs (1, 9), (2, 2), (8, 2), (9, 9).
  const double best = 2.0 * std::hypot(1.0, 7.0) + 6.0;
  EXPECT_LT(report.length_after, report.length_before);
  EXPECT_GE(report.length_after, best - kCornerCut);
  // 3.6% over, as measured.
  EXPECT_LE(report.length_after, best * 1.05);
}

TEST(ShortenTest, ShortPathsComeBackAsTheyWere) {
  const auto always = [](const VectorXd&) { return true; };
  const JointPath two = {Vec({0, 0}), Vec({1, 1})};
  EXPECT_EQ(*ShortenPath(two, always), two);
  const JointPath one = {Vec({0, 0})};
  EXPECT_EQ(*ShortenPath(one, always), one);
  EXPECT_FALSE(ShortenPath({}, always).has_value());
  EXPECT_FALSE(ShortenPath({Vec({0, 0}), Vec({1, 1, 1})}, always).has_value());
  ShortenOptions bad;
  bad.edge_resolution = 0.0;
  EXPECT_FALSE(ShortenPath(two, always, bad).has_value());
}

// Points on one straight line, or repeated, are dropped without a query.
TEST(ShortenTest, CollinearAndRepeatedPointsGoForFree) {
  int calls = 0;
  const auto counting = [&calls](const VectorXd&) {
    ++calls;
    return true;
  };
  const JointPath path = {Vec({0, 0}), Vec({1, 1}), Vec({1, 1}), Vec({2, 2}),
                          Vec({3, 3})};
  ShortenOptions options;
  options.max_iterations = 0;
  const std::optional<JointPath> shortened =
      ShortenPath(path, counting, options);
  ASSERT_TRUE(shortened.has_value());
  EXPECT_EQ(*shortened, (JointPath{Vec({0, 0}), Vec({3, 3})}));
  EXPECT_EQ(calls, 0);
}

// ------------------------------------------------------------ determinism

TEST(DeterminismTest, SameSeedSamePathDifferentSeedsDifferentPaths) {
  for (const ToyWorld& world : {cell::CircleWorld(), cell::ArmLikeWorld()}) {
    SCOPED_TRACE(world.name);
    std::vector<JointPath> paths;
    for (std::uint32_t seed = 1; seed <= 10; ++seed) {
      RrtReport first_report;
      RrtReport second_report;
      const std::optional<JointPath> first =
          Plan(world, Seeded(seed), &first_report);
      const std::optional<JointPath> second =
          Plan(world, Seeded(seed), &second_report);
      ASSERT_TRUE(first.has_value());
      ASSERT_TRUE(second.has_value());
      EXPECT_EQ(*first, *second);
      EXPECT_EQ(first_report.collision_queries,
                second_report.collision_queries);

      ShortenOptions shorten;
      shorten.seed = seed;
      EXPECT_EQ(*ShortenPath(*first, world.is_free, shorten),
                *ShortenPath(*second, world.is_free, shorten));
      paths.push_back(*first);
    }
    for (std::size_t i = 0; i < paths.size(); ++i) {
      for (std::size_t j = i + 1; j < paths.size(); ++j) {
        EXPECT_NE(paths[i], paths[j]) << "seeds " << i + 1 << ", " << j + 1;
      }
    }
  }
}

// Budgets in one run give what one run per budget gives: the same path, and
// for each budget tried the same report (all but its seconds), whether the
// path is found at the first budget, a later one or none. The queries made
// are fewer: the smaller budgets' iterations are not made again.
TEST(RrtBudgetsTest, OneRunGivesWhatOneRunPerBudgetGives) {
  const ToyWorld world = cell::ArmLikeWorld();
  const std::vector<int> budgets = {20, 80, 320};
  int found_late = 0, not_found = 0;
  std::int64_t one_run = 0, per_budget = 0;
  for (std::uint32_t seed = 1; seed <= 40; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    RrtOptions options = Seeded(seed);
    options.max_seconds = 1e9;
    std::int64_t calls = 0;
    const cell::IsFree counted = [&](const Eigen::VectorXd& q) {
      ++calls;
      return world.is_free(q);
    };
    std::vector<RrtReport> expected;
    std::optional<JointPath> want;
    for (int budget : budgets) {
      options.max_iterations = budget;
      RrtReport r;
      want = PlanRrtConnect(world.start, world.goal, world.lower, world.upper,
                            counted, options, &r);
      expected.push_back(r);
      if (want || r.status != cell::RrtStatus::kIterationLimit) break;
    }
    per_budget += calls;
    calls = 0;
    std::vector<RrtReport> got;
    const std::optional<JointPath> path = cell::PlanRrtConnectBudgets(
        world.start, world.goal, world.lower, world.upper, counted, options,
        budgets, &got);
    one_run += calls;
    ASSERT_EQ(path.has_value(), want.has_value());
    if (path) EXPECT_TRUE(*path == *want);
    ASSERT_EQ(got.size(), expected.size());
    std::int64_t reported = 0;
    for (std::size_t k = 0; k < got.size(); ++k) {
      EXPECT_EQ(got[k].status, expected[k].status);
      EXPECT_EQ(got[k].iterations, expected[k].iterations);
      EXPECT_EQ(got[k].collision_queries, expected[k].collision_queries);
      EXPECT_EQ(got[k].start_tree_size, expected[k].start_tree_size);
      EXPECT_EQ(got[k].goal_tree_size, expected[k].goal_tree_size);
      EXPECT_EQ(got[k].path_length, expected[k].path_length);
      reported += got[k].collision_queries;
    }
    // The one run's calls are its last report's.
    EXPECT_EQ(calls, got.back().collision_queries);
    EXPECT_LE(calls, reported);
    if (path && got.size() > 1) ++found_late;
    if (!path) ++not_found;
  }
  // Both cases were met.
  EXPECT_GT(found_late, 0);
  EXPECT_GT(not_found, 0);
  EXPECT_LT(one_run, per_budget);
}

// Budgets that do not increase are run one by one.
TEST(RrtBudgetsTest, BudgetsThatDoNotIncreaseAreRunOneByOne) {
  const ToyWorld world = cell::ArmLikeWorld();
  RrtOptions options = Seeded(3);
  std::vector<RrtReport> got;
  cell::PlanRrtConnectBudgets(world.start, world.goal, world.lower,
                              world.upper, world.is_free, options, {5, 5, 2},
                              &got);
  ASSERT_FALSE(got.empty());
  EXPECT_EQ(got.front().iterations, 5);
}

}  // namespace
