// Plans the 6-joint toy world with this repository's planner and with OMPL's
// RRT-Connect, and prints one line of medians for each.
//
//   bazel run --config=opt //:ompl_smoke
//
// What it is for. It shows that OMPL (https://ompl.kavrakilab.org) builds
// here, links against this repository's Eigen, and plans in the same world
// through the same collision question as the planner of src/rrt.h. It is a
// first look at the two side by side and not a comparison of them: the world
// is made up (cell::ArmLikeWorld, no arm and no MuJoCo), and the two planners
// differ in ways the numbers do not correct for. The ones that matter most:
//
//   - Edge checks. The planner of src/rrt.h checks an edge at points no more
//     than 0.02 rad apart measured as the sum of the joint changes. OMPL is
//     set to 0.02 rad measured as the Euclidean distance, which for the same
//     edge is the smaller number (by up to sqrt(6) with six joints), so OMPL
//     checks fewer points on the same edge.
//   - Shortening. ShortenPath tries 200 random shortcuts. OMPL's
//     simplifySolution with no time limit runs its whole sequence: shortcuts,
//     a B-spline smoothing pass, and removal of vertices, until none helps.
//   - Step length. The planner of src/rrt.h steps 1 rad; OMPL's RRT-Connect
//     chooses its own range from the size of the space.
//   - Seeds. The planner of src/rrt.h takes a seed per plan, here 1 to 20.
//     OMPL takes one seed for the whole process and gives every random
//     generator made afterwards a seed derived from it, so its 20 plans are
//     20 different draws that repeat as a set from run to run, not plan by
//     plan.
//
// Both planners ask the same function whether a configuration is free,
// wrapped in one counter. Every path returned is then checked again, outside
// the count, at 0.005 rad spacing (Euclidean) along its straight segments.
//
// Output: exactly two lines on standard output, the first for this
// repository's planner and the second for OMPL:
//
//   planner=<own|ompl> world=arm_like seeds=20 found=<n> valid=<n>
//       median_queries=<int> median_ms=<float> median_length=<float>
//
// (one line each; broken here to fit). `found` is the plans that returned a
// path and `valid` those whose path passed the second check. The medians are
// over the found plans: collision queries during planning and shortening,
// wall time of planning and shortening in milliseconds, and cell::PathLength
// of the shortened path with no weights, in radians. With an even number of
// found plans a median is the mean of the two middle values, rounded to a
// whole number for the queries; with none found the medians print as zero.
// OMPL's warnings and errors, if any, go to standard error.
//
// Exit status 0 if both lines were printed and each planner found at least
// one path, 1 otherwise.
//
//   bazel run --config=opt //:ompl_smoke -- --plans
//
// also writes one line per plan to standard error, with that plan's numbers
// and two more checks of its path, at the same spacing, against the world's
// obstacles shrunk by 0.010 rad and by 0.015 rad on every side. A path that
// fails the plain check and passes these cut a corner between two checked
// points and did nothing worse: 0.010 is half the edge resolution of the
// planner of src/rrt.h, which is all it promises, and 0.015 covers half of
// OMPL's spacing where an obstacle's boundary depends on two joints.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>
#include <ompl/util/RandomNumbers.h>

#include "src/rrt.h"
#include "src/rrt_worlds.h"

namespace {

using cell::IsFree;
using cell::JointPath;
using cell::ToyWorld;

constexpr int kPlans = 20;

// The most time one plan may take, in seconds. It is RrtOptions' default,
// and is given to OMPL's solve().
constexpr double kTimeLimit = 5.0;

// The largest Euclidean distance, in radians, between the states OMPL checks
// along a motion.
constexpr double kOmplCheckSpacing = 0.02;

// The spacing, in radians, of the second check of every returned path.
constexpr double kRecheckSpacing = 0.005;

// The margins, in radians, the obstacles are shrunk by for the two extra
// checks of --plans.
constexpr double kShrunkBy[2] = {0.010, 0.015};

struct Plan {
  bool found = false;
  bool valid = false;
  // Whether the path is free of the obstacles shrunk by kShrunkBy[i].
  bool valid_shrunk[2] = {false, false};
  std::int64_t queries = 0;
  double milliseconds = 0.0;
  double length = 0.0;
};

double Milliseconds(std::chrono::steady_clock::time_point from,
                    std::chrono::steady_clock::time_point to) {
  return std::chrono::duration<double, std::milli>(to - from).count();
}

// Whether every configuration of `path`, and of its straight segments at
// kRecheckSpacing, is free.
bool PathIsFree(const JointPath& path, const IsFree& is_free) {
  const std::optional<JointPath> dense = cell::Resample(path, kRecheckSpacing);
  if (!dense) return false;
  return std::all_of(dense->begin(), dense->end(), is_free);
}

// Fills in what is measured on the path a plan returned.
void Judge(const JointPath& path, const ToyWorld& world, Plan* plan) {
  plan->found = true;
  plan->length = cell::PathLength(path);
  plan->valid = PathIsFree(path, world.is_free);
  for (int i = 0; i < 2; ++i) {
    plan->valid_shrunk[i] =
        PathIsFree(path, [&world, i](const Eigen::VectorXd& q) {
          return world.free_by(q, kShrunkBy[i]);
        });
  }
}

// One plan by PlanRrtConnect and ShortenPath, both with their default
// options and the given seed. `counted` is the world's is_free behind the
// counter `queries`.
Plan PlanOwn(const ToyWorld& world, const IsFree& counted,
             const std::int64_t& queries, std::uint32_t seed) {
  Plan plan;
  const std::int64_t queries_before = queries;
  const auto begin = std::chrono::steady_clock::now();

  cell::RrtOptions rrt_options;
  rrt_options.seed = seed;
  std::optional<JointPath> path =
      cell::PlanRrtConnect(world.start, world.goal, world.lower, world.upper,
                           counted, rrt_options);
  if (path) {
    cell::ShortenOptions shorten_options;
    shorten_options.seed = seed;
    std::optional<JointPath> shorter =
        cell::ShortenPath(*path, counted, shorten_options);
    // ShortenPath returns nothing only for input it rejects, which a path
    // from PlanRrtConnect is not; the path found is kept if it ever does.
    if (shorter) path = std::move(shorter);
  }

  plan.milliseconds = Milliseconds(begin, std::chrono::steady_clock::now());
  plan.queries = queries - queries_before;
  if (path) Judge(*path, world, &plan);
  return plan;
}

// One plan by OMPL's RRT-Connect and SimpleSetup::simplifySolution, on a
// fresh state space and setup. There is no seed to give: see the top of the
// file.
Plan PlanOmpl(const ToyWorld& world, const IsFree& counted,
              const std::int64_t& queries) {
  namespace ob = ompl::base;
  namespace og = ompl::geometric;

  const int joints = static_cast<int>(world.lower.size());
  auto space = std::make_shared<ob::RealVectorStateSpace>(joints);
  ob::RealVectorBounds bounds(joints);
  for (int i = 0; i < joints; ++i) {
    bounds.setLow(i, world.lower[i]);
    bounds.setHigh(i, world.upper[i]);
  }
  space->setBounds(bounds);

  og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&counted, joints](const ob::State* state) {
    const double* values =
        state->as<ob::RealVectorStateSpace::StateType>()->values;
    const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(values, joints);
    return counted(q);
  });

  // OMPL checks a motion at evenly spaced states: as many segments as the
  // motion's length divided by the "longest valid segment", rounded up. That
  // segment is given as a fraction of the space's largest extent (the
  // diagonal of the joint box), so the fraction is the wanted spacing over
  // the extent, nudged down if rounding made the product come out above.
  const double extent = space->getMaximumExtent();
  double fraction = kOmplCheckSpacing / extent;
  while (fraction * extent > kOmplCheckSpacing) {
    fraction = std::nextafter(fraction, 0.0);
  }
  setup.getSpaceInformation()->setStateValidityCheckingResolution(fraction);

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  for (int i = 0; i < joints; ++i) {
    start[i] = world.start[i];
    goal[i] = world.goal[i];
  }
  setup.setStartAndGoalStates(start, goal);
  setup.setPlanner(
      std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));

  Plan plan;
  const std::int64_t queries_before = queries;
  const auto begin = std::chrono::steady_clock::now();

  // solve() also sets everything up, which checks the start and the goal.
  // An approximate solution (a path that ends short of the goal) does not
  // count as found.
  setup.solve(kTimeLimit);
  const bool found = setup.haveExactSolutionPath();
  if (found) setup.simplifySolution();

  plan.milliseconds = Milliseconds(begin, std::chrono::steady_clock::now());
  plan.queries = queries - queries_before;
  if (!found) return plan;

  JointPath path;
  for (const ob::State* state : setup.getSolutionPath().getStates()) {
    const double* values =
        state->as<ob::RealVectorStateSpace::StateType>()->values;
    path.push_back(Eigen::Map<const Eigen::VectorXd>(values, joints));
  }
  Judge(path, world, &plan);
  return plan;
}

// The middle value, or the mean of the two middle values. Zero if empty.
double Median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return n % 2 == 1 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

// Writes one line per plan to standard error, for --plans.
void PrintPlans(const char* planner, const std::vector<Plan>& plans) {
  for (std::size_t i = 0; i < plans.size(); ++i) {
    const Plan& plan = plans[i];
    std::fprintf(stderr,
                 "planner=%s plan=%zu found=%d valid=%d valid_shrunk_0.010=%d "
                 "valid_shrunk_0.015=%d queries=%lld ms=%.3f length=%.3f\n",
                 planner, i + 1, plan.found, plan.valid, plan.valid_shrunk[0],
                 plan.valid_shrunk[1], static_cast<long long>(plan.queries),
                 plan.milliseconds, plan.length);
  }
}

// Prints one planner's line. True if it was written.
bool PrintLine(const char* planner, const ToyWorld& world,
               const std::vector<Plan>& plans) {
  int found = 0;
  int valid = 0;
  std::vector<double> queries, milliseconds, lengths;
  for (const Plan& plan : plans) {
    if (!plan.found) continue;
    ++found;
    if (plan.valid) ++valid;
    queries.push_back(static_cast<double>(plan.queries));
    milliseconds.push_back(plan.milliseconds);
    lengths.push_back(plan.length);
  }
  const int written = std::printf(
      "planner=%s world=%s seeds=%d found=%d valid=%d median_queries=%lld "
      "median_ms=%.3f median_length=%.3f\n",
      planner, world.name.c_str(), static_cast<int>(plans.size()), found,
      valid, std::llround(Median(queries)), Median(milliseconds),
      Median(lengths));
  return written > 0 && std::fflush(stdout) == 0;
}

int Found(const std::vector<Plan>& plans) {
  return static_cast<int>(std::count_if(
      plans.begin(), plans.end(), [](const Plan& plan) { return plan.found; }));
}

}  // namespace

int main(int argc, char** argv) {
  const bool print_plans = argc > 1 && std::strcmp(argv[1], "--plans") == 0;

  // Before anything else of OMPL: the seed every random generator OMPL makes
  // in this process is derived from. Without it OMPL seeds from the clock.
  ompl::RNG::setSeed(1);
  // OMPL writes "Info:" and "Debug:" lines to standard output, which would
  // land between the two lines below. Warnings and errors go to standard
  // error and are kept.
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

  const ToyWorld world = cell::ArmLikeWorld();
  std::int64_t queries = 0;
  const IsFree counted = [&world, &queries](const Eigen::VectorXd& q) {
    ++queries;
    return world.is_free(q);
  };

  std::vector<Plan> own;
  for (int seed = 1; seed <= kPlans; ++seed) {
    own.push_back(
        PlanOwn(world, counted, queries, static_cast<std::uint32_t>(seed)));
  }
  const bool own_printed = PrintLine("own", world, own);
  if (print_plans) PrintPlans("own", own);

  std::vector<Plan> ompl;
  for (int i = 0; i < kPlans; ++i) {
    ompl.push_back(PlanOmpl(world, counted, queries));
  }
  const bool ompl_printed = PrintLine("ompl", world, ompl);
  if (print_plans) PrintPlans("ompl", ompl);

  const bool ok =
      own_printed && ompl_printed && Found(own) >= 1 && Found(ompl) >= 1;
  return ok ? 0 : 1;
}
