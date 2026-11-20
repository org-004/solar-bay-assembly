// Runs the planner of src/rrt.h on the toy worlds of src/rrt_worlds.h.
//
//   bazel run //:print_rrt -- [<world>] [<seed>]
//       One plan in a 2D world, as CSV for plotting: the obstacles, both
//       trees, the raw path and the shortened path. Worlds: open, circle,
//       gap, goal_in_gap, bug_trap, enclosed_goal (default circle). The
//       first column says what each row is:
//         circle,<x>,<y>,<radius>
//         box,<x0>,<y0>,<x1>,<y1>
//         start_tree,<x0>,<y0>,<x1>,<y1>   one edge, parent to child
//         goal_tree,<x0>,<y0>,<x1>,<y1>
//         raw,<x>,<y>                     one configuration of the path
//         short,<x>,<y>
//   bazel run --config=opt //:print_rrt -- --stats [<seeds>]
//       Every world over many seeds (default 100): success rate, collision
//       queries, time, and path lengths before and after shortening against
//       the shortest. Time it optimized.
//   bazel run --config=opt //:print_rrt -- --sweep
//       Every world over step sizes, edge resolutions and goal biases.
//   bazel run --config=opt //:print_rrt -- --shorten-sweep
//       Shortening over its number of tries and its minimum saving.
//   bazel run --config=opt //:print_rrt -- --nn
//       The planner's own cost per iteration, without the collision query,
//       against tree size: what the nearest-neighbour scan costs.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "src/rrt.h"
#include "src/rrt_worlds.h"

namespace {

using cell::JointPath;
using cell::RrtOptions;
using cell::RrtReport;
using cell::ToyWorld;

std::vector<ToyWorld> AllWorlds() {
  ToyWorld narrow = cell::GapWorld(0.2);
  narrow.name = "gap_0.2";
  return {cell::OpenWorld(),   cell::CircleWorld(),       cell::GapWorld(),
          narrow,              cell::GoalInGapWorld(),    cell::BugTrapWorld(),
          cell::EnclosedGoalWorld(), cell::ArmLikeWorld()};
}

template <typename T>
T Percentile(std::vector<T> values, double fraction) {
  if (values.empty()) return T();
  std::sort(values.begin(), values.end());
  const std::size_t i = static_cast<std::size_t>(
      std::min<double>(values.size() - 1, fraction * (values.size() - 1) + 0.5));
  return values[i];
}

int Draw(const std::string& name, std::uint32_t seed) {
  std::optional<ToyWorld> chosen;
  for (const ToyWorld& world : AllWorlds()) {
    if (world.name == name && world.lower.size() == 2) chosen = world;
  }
  if (!chosen) {
    std::fprintf(stderr, "unknown 2D world: %s\n", name.c_str());
    return 1;
  }
  const ToyWorld& world = *chosen;
  RrtOptions options;
  options.seed = seed;
  RrtReport report;
  cell::RrtTrees trees;
  const std::optional<JointPath> raw =
      cell::PlanRrtConnect(world.start, world.goal, world.lower, world.upper,
                           world.is_free, options, &report, &trees);

  std::printf("kind,a,b,c,d\n");
  for (const cell::Circle& c : world.circles) {
    std::printf("circle,%g,%g,%g\n", c.centre.x(), c.centre.y(), c.radius);
  }
  for (const cell::Box& b : world.boxes) {
    std::printf("box,%g,%g,%g,%g\n", b.low[0], b.low[1], b.high[0], b.high[1]);
  }
  auto edges = [](const char* kind, const JointPath& nodes,
                  const std::vector<int>& parents) {
    for (std::size_t i = 1; i < nodes.size(); ++i) {
      const Eigen::VectorXd& p = nodes[parents[i]];
      std::printf("%s,%.5f,%.5f,%.5f,%.5f\n", kind, p[0], p[1], nodes[i][0],
                  nodes[i][1]);
    }
  };
  edges("start_tree", trees.start_nodes, trees.start_parents);
  edges("goal_tree", trees.goal_nodes, trees.goal_parents);
  if (raw) {
    for (const Eigen::VectorXd& q : *raw) {
      std::printf("raw,%.5f,%.5f\n", q[0], q[1]);
    }
    cell::ShortenOptions shorten;
    shorten.seed = seed;
    const std::optional<JointPath> shortened =
        cell::ShortenPath(*raw, world.is_free, shorten);
    for (const Eigen::VectorXd& q : *shortened) {
      std::printf("short,%.5f,%.5f\n", q[0], q[1]);
    }
  }
  std::fprintf(stderr, "%s: %s after %d iterations, %lld queries\n",
               world.name.c_str(), cell::ToString(report.status),
               report.iterations,
               static_cast<long long>(report.collision_queries));
  return 0;
}

// One world over `seeds` seeds, as one row.
void StatsRow(const ToyWorld& world, int seeds, const RrtOptions& base,
              const char* label) {
  int found = 0;
  std::vector<long long> queries;
  std::vector<double> times;
  std::vector<int> nodes;
  std::vector<double> raw_ratio;
  std::vector<double> short_ratio;
  std::vector<long long> short_queries;
  std::vector<double> short_times;
  double raw_sum = 0.0;
  double short_sum = 0.0;
  for (int seed = 1; seed <= seeds; ++seed) {
    RrtOptions options = base;
    options.seed = seed;
    // Grow trees even in the open world, to see what shortening does.
    options.try_direct_first = false;
    RrtReport report;
    const std::optional<JointPath> raw =
        cell::PlanRrtConnect(world.start, world.goal, world.lower, world.upper,
                             world.is_free, options, &report);
    queries.push_back(report.collision_queries);
    times.push_back(report.seconds);
    if (!raw) continue;
    ++found;
    nodes.push_back(report.start_tree_size + report.goal_tree_size);
    cell::ShortenOptions shorten;
    shorten.seed = seed;
    shorten.edge_resolution = options.edge_resolution;
    cell::ShortenReport shortened;
    cell::ShortenPath(*raw, world.is_free, shorten, &shortened);
    raw_sum += shortened.length_before;
    short_sum += shortened.length_after;
    short_queries.push_back(shortened.collision_queries);
    short_times.push_back(shortened.seconds);
    if (std::isfinite(world.shortest) && world.shortest > 0.0) {
      raw_ratio.push_back(shortened.length_before / world.shortest);
      short_ratio.push_back(shortened.length_after / world.shortest);
    }
  }
  std::printf(
      "%-14s %-10s %4d/%-4d %8lld %8lld %9.2f %9.2f %6d | %6.2f %6.2f %6.3f "
      "%6.3f | %7lld %7.2f\n",
      world.name.c_str(), label, found, seeds, Percentile(queries, 0.5),
      *std::max_element(queries.begin(), queries.end()),
      1e3 * Percentile(times, 0.5),
      1e3 * *std::max_element(times.begin(), times.end()),
      Percentile(nodes, 0.5), found ? raw_sum / found : 0.0,
      found ? short_sum / found : 0.0, Percentile(raw_ratio, 0.5),
      raw_ratio.empty() ? 0.0
                        : *std::max_element(short_ratio.begin(),
                                            short_ratio.end()),
      Percentile(short_queries, 0.5), 1e3 * Percentile(short_times, 0.5));
}

void StatsHeader() {
  std::printf(
      "%-14s %-10s %9s %8s %8s %9s %9s %6s | %6s %6s %6s %6s | %7s %7s\n",
      "world", "options", "found", "q med", "q worst", "ms med", "ms worst",
      "nodes", "raw", "short", "raw/sp", "sh/sp", "shq med", "sh ms");
  std::printf(
      "%-14s %-10s %9s %8s %8s %9s %9s %6s | %6s %6s %6s %6s | %7s %7s\n", "",
      "", "", "", "", "", "", "", "mean", "mean", "median", "worst", "", "");
}

int Stats(int seeds) {
  StatsHeader();
  for (const ToyWorld& world : AllWorlds()) {
    RrtOptions options;
    StatsRow(world, seeds, options, "default");
  }
  return 0;
}

// Shortening alone over its budget and its minimum saving, on the worlds
// whose shortest path is known and on the 6-joint world (whose "ratio" is
// then the length itself). Each row is 100 seeds' raw paths.
int ShortenSweep() {
  std::printf("%-10s %6s %7s | %8s %8s | %7s %7s | %8s %8s\n", "world",
              "tries", "saving", "q med", "q worst", "before", "after",
              "ratio", "worst");
  for (const ToyWorld& world : {cell::CircleWorld(), cell::GapWorld(),
                                cell::BugTrapWorld(), cell::ArmLikeWorld()}) {
    for (int tries : {0, 50, 100, 200, 400}) {
      for (double saving : {0.0, 0.003, 0.01, 0.03}) {
        std::vector<long long> queries;
        std::vector<double> ratios;
        double before = 0.0;
        double after = 0.0;
        for (int seed = 1; seed <= 100; ++seed) {
          RrtOptions options;
          options.seed = seed;
          options.try_direct_first = false;
          const std::optional<JointPath> raw = cell::PlanRrtConnect(
              world.start, world.goal, world.lower, world.upper,
              world.is_free, options);
          if (!raw) continue;
          cell::ShortenOptions shorten;
          shorten.seed = seed;
          shorten.max_iterations = tries;
          shorten.min_saving = saving;
          cell::ShortenReport report;
          cell::ShortenPath(*raw, world.is_free, shorten, &report);
          queries.push_back(report.collision_queries);
          before += report.length_before;
          after += report.length_after;
          ratios.push_back(std::isfinite(world.shortest)
                               ? report.length_after / world.shortest
                               : report.length_after);
        }
        if (queries.empty()) continue;
        std::printf("%-10s %6d %7.3f | %8lld %8lld | %7.3f %7.3f | %8.4f "
                    "%8.4f\n",
                    world.name.c_str(), tries, saving,
                    Percentile(queries, 0.5),
                    *std::max_element(queries.begin(), queries.end()),
                    before / queries.size(), after / queries.size(),
                    Percentile(ratios, 0.5),
                    *std::max_element(ratios.begin(), ratios.end()));
      }
    }
  }
  return 0;
}

int Sweep() {
  StatsHeader();
  for (const ToyWorld& world : AllWorlds()) {
    if (!std::isfinite(world.shortest) && !std::isnan(world.shortest)) continue;
    for (double step : {0.25, 0.5, 1.0, 1.5}) {
      for (double resolution : {0.01, 0.02}) {
        RrtOptions options;
        options.step_size = step;
        options.edge_resolution = resolution;
        char label[32];
        std::snprintf(label, sizeof label, "s%.2f r%.2f", step, resolution);
        StatsRow(world, 100, options, label);
      }
    }
    for (double bias : {0.05, 0.1}) {
      RrtOptions options;
      options.goal_bias = bias;
      char label[32];
      std::snprintf(label, sizeof label, "bias %.2f", bias);
      StatsRow(world, 100, options, label);
    }
  }
  return 0;
}

// Six joints, everything free except a hollow shell round the goal, so the
// start tree grows without ever meeting the goal's. The collision query is
// a few comparisons, so what is timed is the planner's own work: mostly
// the nearest-neighbour scan once the tree is large, plus the edge points.
int NearestNeighbourCost() {
  const ToyWorld arm = cell::ArmLikeWorld();
  const Eigen::VectorXd centre = arm.goal;
  const auto is_free = [centre](const Eigen::VectorXd& q) {
    const double d = (q - centre).cwiseAbs().maxCoeff();
    return d < 0.3 || d > 0.6;
  };
  std::printf("%10s %10s %12s %14s %16s\n", "iterations", "nodes", "seconds",
              "us/iteration", "ns/node scanned");
  double previous_seconds = 0.0;
  int previous_iterations = 0;
  long long previous_nodes = 0;
  for (int iterations : {250, 500, 1000, 2000, 4000, 8000, 16000, 32000}) {
    RrtOptions options;
    options.max_iterations = iterations;
    options.max_seconds = 600.0;
    RrtReport report;
    cell::PlanRrtConnect(arm.start, arm.goal, arm.lower, arm.upper, is_free,
                         options, &report);
    const long long nodes = report.start_tree_size + report.goal_tree_size;
    // Over the iterations since the previous row, the cost per iteration,
    // and per node of the trees scanned. Each iteration scans tree a once
    // for the step, and tree b once per CONNECT step.
    const double dt = report.seconds - previous_seconds;
    const int di = report.iterations - previous_iterations;
    const double mean_nodes = 0.5 * (nodes + previous_nodes);
    std::printf("%10d %10lld %12.4f %14.2f %16.2f\n", report.iterations, nodes,
                report.seconds, 1e6 * dt / di, 1e9 * dt / di / mean_nodes);
    previous_seconds = report.seconds;
    previous_iterations = report.iterations;
    previous_nodes = nodes;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  if (!args.empty() && args[0] == "--stats") {
    const int seeds = args.size() > 1 ? std::atoi(args[1].c_str()) : 100;
    if (seeds < 1) return 1;
    return Stats(seeds);
  }
  if (!args.empty() && args[0] == "--sweep") return Sweep();
  if (!args.empty() && args[0] == "--shorten-sweep") return ShortenSweep();
  if (!args.empty() && args[0] == "--nn") return NearestNeighbourCost();
  if (args.size() > 2 || (!args.empty() && args[0].rfind("--", 0) == 0)) {
    std::fprintf(stderr,
                 "usage: %s [<world>] [<seed>] | --stats [<seeds>] | --sweep "
                 "| --shorten-sweep | --nn\n",
                 argv[0]);
    return 1;
  }
  const std::string world = args.empty() ? "circle" : args[0];
  const std::uint32_t seed =
      args.size() > 1 ? static_cast<std::uint32_t>(std::atoi(args[1].c_str()))
                      : 1;
  return Draw(world, seed);
}
