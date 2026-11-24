// Plans the cell's carry (or its return) with the motion planner
// (src/motion_planner.h) for many base seeds and prints, for each: whether
// a path was found, the time spent searching and shortening, the collision
// queries, the waypoints, the length in metres of motion, the smallest
// clearance measured along it, and the duration of its trajectory. Then a
// summary, with the scripted cycle's own path for comparison. CSV, or with
// --pretty an aligned table. This is the planning stage's "planning time
// reported"; time it optimized:
//
//   bazel run --config=opt //:print_plan -- --pretty
//   bazel run --config=opt //:print_plan -- --pretty --module 29 --slot 9
//   bazel run --config=opt //:print_plan -- --pretty --return
//   bazel run //:print_plan -- [--pretty] [--module K] [--seeds N] [--return]
//                              [--slot S] [--first-seed B] [--both-ways]
//                              [--other-way] [--plan-seeds P] [--resample M]
//                              [--corner MM] [--resolution MM] [--over-slot]
//                              [--planner own|ompl] [--ompl-planner NAME]
//                              [--ompl-range M] [--ompl-queries Q]
//                              [--summary]
//
//   --module K      the module carried (default 0): modules 0 .. K-1 are
//                   taken off the stack first (Cell::ParkModule)
//   --slot S        the tube S steps on, with modules lying on its slots
//                   0 .. S-1 (default 0: the tube empty). They are the
//                   modules already taken off, then the stack's back ones.
//   --return        the bare arm's return from the end of the retreat to
//                   the ready pose, with module K lying in slot S
//   --seeds N       how many base seeds (default 50)
//   --first-seed B  the first base seed (default 1)
//   --plan-seeds P  MotionPlanOptions::seeds, plans per base seed (default 4)
//   --both-ways     MotionPlanOptions::try_both_ways
//   --other-way     the goal with joint 1 a whole turn the other way: for
//                   the carry, round the -Y side, past the camera's post
//   --over-slot     the carry on to over the slot itself (the end of the
//                   lowering's way in) instead of the start of "lower",
//                   0.35 m further out, where the script turns the module
//   --resample M    MotionPlanOptions::resample_spacing, m of motion
//   --corner MM     MotionPlanOptions::corner_allowance, mm
//   --resolution MM MotionPlanOptions::edge_resolution, mm
//   --planner own|ompl
//                   MotionPlanOptions::search: the hand-written RRT-Connect
//                   of src/rrt.h (own, the default) or OMPL's
//                   (src/ompl_search.h)
//   --ompl-planner NAME
//                   OmplSettings::planner, one of cell::OmplPlannerNames()
//                   (default RRTConnect); refused if unknown
//   --ompl-range M  OmplSettings::range, m of motion; 0 leaves the choice
//                   to OMPL (default: MotionPlanOptions::step)
//   --ompl-queries Q
//                   OmplSettings::max_queries, a positive integer
//                   (default 200000)
//   --summary       in place of the header and the rows, two lines of CSV: the
//                   column names and one row of figures over all the plans
//                   (a plan is one base seed); the "#" lines stay
//
// Given any of --planner and the --ompl- flags, the first "#" line ends with
// the search and, for OMPL, its settings.
//
// The scene is the nominal one, Reset(1, Randomization::None()), and the
// obstacles are where the simulation has them (SetScene); the start and
// goal are the scripted cycle's own (src/pick_place.h): the carry from the
// end of "clear" to the start of "lower" with the module held at the design
// grip, the return from the end of "retreat" to the ready pose.
//
// The first argument is the cell's model; BUILD.bazel passes it.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/cell.h"
#include "src/collision.h"
#include "src/conveyor.h"
#include "src/joint_move.h"
#include "src/motion_planner.h"
#include "src/path_timing.h"
#include "src/pick_place.h"
#include "src/rrt.h"

namespace {

using cell::Cell;
using cell::CollisionChecker;
using cell::JointPath;
using Step = cell::PickPlaceStep;

constexpr double kTwoPi = 2.0 * std::numbers::pi;

struct Args {
  bool pretty = false;
  int module = 0;
  int slot = 0;
  bool return_move = false;
  int seeds = 50;
  std::uint32_t first_seed = 1;
  int plan_seeds = 4;
  bool both_ways = false;
  bool other_way = false;
  bool over_slot = false;
  double resample = 0.0;  // m; 0: the default
  double corner = -1.0;   // m; negative: the default
  double resolution = 0.0;  // m; 0: the default
  bool search_given = false;  // any of the search flags
  cell::PlanSearch search = cell::PlanSearch::kOwn;
  std::string ompl_planner = "RRTConnect";
  std::optional<double> ompl_range;  // m; 0: OMPL's own
  std::int64_t ompl_queries = 200000;
  bool summary = false;
};

struct Row {
  std::uint32_t base_seed = 0;
  cell::MotionPlanReport report;
  bool found = false;
  double duration = 0.0;
};

double Median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t n = values.size();
  return n % 2 ? values[n / 2] : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

double Max(const std::vector<double>& values) {
  return values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
}

// The scripted moves timed as the script's follower plays them: a JointMove
// from rest to rest for each leg, at `speed`.
double ScriptedDuration(const JointPath& path, const cell::JointLimits& limits,
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: print_plan <cell.xml> [options]\n");
    return 2;
  }
  Args args;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&]() -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs a value\n", a.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--pretty") {
      args.pretty = true;
    } else if (a == "--module") {
      args.module = std::atoi(value());
    } else if (a == "--slot") {
      args.slot = std::atoi(value());
    } else if (a == "--return") {
      args.return_move = true;
    } else if (a == "--seeds") {
      args.seeds = std::atoi(value());
    } else if (a == "--first-seed") {
      args.first_seed = static_cast<std::uint32_t>(std::atol(value()));
    } else if (a == "--plan-seeds") {
      args.plan_seeds = std::atoi(value());
    } else if (a == "--both-ways") {
      args.both_ways = true;
    } else if (a == "--other-way") {
      args.other_way = true;
    } else if (a == "--over-slot") {
      args.over_slot = true;
    } else if (a == "--resample") {
      args.resample = std::atof(value());
    } else if (a == "--corner") {
      args.corner = 1e-3 * std::atof(value());
    } else if (a == "--resolution") {
      args.resolution = 1e-3 * std::atof(value());
    } else if (a == "--planner") {
      const std::string name = value();
      const std::optional<cell::PlanSearch> search =
          cell::PlanSearchFromName(name);
      if (!search) {
        std::fprintf(stderr, "--planner must be own or ompl, not %s\n",
                     name.c_str());
        return 2;
      }
      args.search = *search;
      args.search_given = true;
    } else if (a == "--ompl-planner") {
      args.ompl_planner = value();
      if (!cell::IsOmplPlanner(args.ompl_planner)) {
        std::string known;
        for (const std::string& n : cell::OmplPlannerNames()) {
          known += (known.empty() ? "" : ", ") + n;
        }
        std::fprintf(stderr, "unknown OMPL planner %s; known: %s\n",
                     args.ompl_planner.c_str(), known.c_str());
        return 2;
      }
      args.search_given = true;
    } else if (a == "--ompl-range") {
      const char* text = value();
      char* end = nullptr;
      const double range = std::strtod(text, &end);
      if (end == text || *end != '\0' || !(range >= 0.0)) {
        std::fprintf(stderr,
                     "--ompl-range must be a number of metres, 0 or more, "
                     "not %s\n",
                     text);
        return 2;
      }
      args.ompl_range = range;
      args.search_given = true;
    } else if (a == "--ompl-queries") {
      const char* text = value();
      char* end = nullptr;
      const long long queries = std::strtoll(text, &end, 10);
      if (end == text || *end != '\0' || queries <= 0) {
        std::fprintf(stderr,
                     "--ompl-queries must be a positive integer, not %s\n",
                     text);
        return 2;
      }
      args.ompl_queries = queries;
      args.search_given = true;
    } else if (a == "--summary") {
      args.summary = true;
    } else {
      std::fprintf(stderr, "unknown option %s\n", a.c_str());
      return 2;
    }
  }

  std::string error;
  std::optional<Cell> loaded = Cell::Load(argv[1], &error);
  if (!loaded) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  Cell& cell = *loaded;
  cell.Reset(1, cell::Randomization::None());
  if (args.module < 0 || args.module >= cell.module_count() || args.slot < 0 ||
      args.slot >= cell::Conveyor::kSlotCount) {
    std::fprintf(stderr, "module or slot out of range\n");
    return 2;
  }

  // The scripted cycle for the module, all of it planned (starting at the
  // set-down plans the place side too); nothing is simulated.
  cell::PickPlace script(cell);
  if (!script.Start(args.module, Step::kSetDown)) {
    std::fprintf(stderr, "script: %s\n", script.outcome().reason.c_str());
    return 1;
  }
  auto waypoints = [&](Step s) { return *script.Waypoints(s); };

  // The grip as designed: the tool at the touch pose, the module where it
  // stands.
  cell.PutArm(waypoints(Step::kTouchGlass).back());
  const Eigen::Isometry3d grip =
      cell::TipPose(cell.arm(), *cell.data()).inverse() *
      cell.TrueModulePose(args.module);
  cell.PutArm(cell.arm_home());
  for (int i = 0; i < args.module; ++i) cell.ParkModule(i);

  std::optional<CollisionChecker> made =
      CollisionChecker::Create(*cell.model(), {}, &error);
  if (!made) {
    std::fprintf(stderr, "checker: %s\n", error.c_str());
    return 1;
  }
  CollisionChecker& checker = *made;
  checker.SetScene(*cell.data());

  // The tube args.slot steps on, with modules on the slots before.
  const double slide = cell::Conveyor::NominalSlidePosition(args.slot);
  checker.SetTubeSlide(slide);
  std::vector<int> spare;
  for (int i = 0; i < args.module; ++i) spare.push_back(i);
  for (int i = cell.module_count() - 1; i > args.module; --i) spare.push_back(i);
  for (int j = 0; j < args.slot; ++j) {
    checker.SetModulePose(spare[j], cell::Conveyor::SlotPose(j, slide));
  }

  Eigen::VectorXd start, goal;
  JointPath scripted;
  double scripted_speed = 0.9;
  if (args.return_move) {
    // Module K lies in the slot now; the arm is bare.
    checker.SetModulePose(args.module, cell::Conveyor::SlotPose(args.slot, slide));
    start = waypoints(Step::kRetreat).back();
    goal = waypoints(Step::kReturn).back();
    scripted = {start};
    for (const Eigen::VectorXd& q : waypoints(Step::kReturn)) scripted.push_back(q);
    scripted_speed = script.options().speed_empty;
  } else {
    checker.SetHeldModule(args.module, grip);
    start = waypoints(Step::kClear).back();
    goal = waypoints(Step::kTurn).back();
    scripted = {start};
    for (Step s : {Step::kFlatten, Step::kSwing, Step::kTurn}) {
      for (const Eigen::VectorXd& q : waypoints(s)) scripted.push_back(q);
    }
    if (args.over_slot) {
      // On through the lowering's way in, to over the slot at the turn's
      // height: the end of its last leg in, before it comes down.
      const std::vector<Eigen::VectorXd> lower = waypoints(Step::kLower);
      const int in_legs = std::max(1, script.options().in_legs);
      for (int i = 0; i < in_legs; ++i) scripted.push_back(lower[i]);
      goal = scripted.back();
    }
    scripted_speed = script.options().speed_carrying;
  }
  if (args.other_way) goal[0] += goal[0] < 0.0 ? kTwoPi : -kTwoPi;

  cell::MotionPlanOptions options;
  options.seeds = args.plan_seeds;
  options.try_both_ways = args.both_ways;
  options.speed = scripted_speed;
  if (args.resample > 0.0) options.resample_spacing = args.resample;
  if (args.corner >= 0.0) options.corner_allowance = args.corner;
  if (args.resolution > 0.0) options.edge_resolution = args.resolution;
  options.search = args.search;
  options.ompl.planner = args.ompl_planner;
  options.ompl.range = args.ompl_range;
  options.ompl.max_queries = args.ompl_queries;

  std::vector<Row> rows;
  for (int s = 0; s < args.seeds; ++s) {
    Row row;
    row.base_seed = args.first_seed + s;
    options.base_seed = row.base_seed;
    std::optional<JointPath> path =
        cell::PlanMove(checker, start, goal, options, &row.report);
    row.found = path.has_value();
    if (path) {
      std::optional<cell::Trajectory> trajectory = cell::Trajectory::Plan(
          *path, cell.arm().limits,
          {.max_deviation = row.report.max_deviation, .speed = options.speed});
      if (trajectory) row.duration = trajectory->duration();
    }
    rows.push_back(std::move(row));
  }

  // The scripted path: its length and clearance, measured as the planner's.
  const Eigen::VectorXd weights = checker.LeverArms();
  double scripted_clearance = 1.0;
  std::string scripted_between;
  for (const Eigen::VectorXd& q : cell::DenseSamples(checker, scripted, 0.01)) {
    cell::CheckOptions measure;
    measure.clearance = true;
    std::optional<cell::CollisionReport> c = checker.Check(q, measure);
    if (c && c->clearance && *c->clearance < scripted_clearance) {
      scripted_clearance = *c->clearance;
      scripted_between = c->clearance_between;
    }
  }
  const double scripted_length = cell::PathLength(scripted, weights);
  const double scripted_duration =
      ScriptedDuration(scripted, cell.arm().limits, scripted_speed);

  const char* format =
      args.pretty ? "%6u %5s %9.1f %10.1f %8lld %9d %8.2f %9.1f %8.1f %8.2f  %s\n"
                  : "%u,%s,%.2f,%.2f,%lld,%d,%.3f,%.2f,%.2f,%.3f,%s\n";
  std::printf("# %s of module_%d, tube at slot %d%s%s; planning margin %.1f mm "
              "(self %.1f), max deviation %.2f mrad, tracking %.2f mrad",
              args.return_move ? "return" : "carry", args.module, args.slot,
              args.other_way ? ", goal the other way round" : "",
              args.both_ways ? ", both ways tried" : "",
              rows.empty() ? 0.0 : 1e3 * rows[0].report.planning_margin,
              rows.empty() ? 0.0 : 1e3 * rows[0].report.planning_self_margin,
              rows.empty() ? 0.0 : 1e3 * rows[0].report.max_deviation,
              rows.empty() ? 0.0 : 1e3 * rows[0].report.tracking_bound);
  if (args.search_given) {
    std::printf("; search %s", cell::PlanSearchName(args.search));
    if (args.search == cell::PlanSearch::kOmpl) {
      std::printf(" (%s, ", args.ompl_planner.c_str());
      const double range = args.ompl_range.value_or(options.step);
      if (args.ompl_range && *args.ompl_range == 0.0) {
        std::printf("range OMPL's own");
      } else {
        std::printf("range %.2f m", range);
      }
      std::printf(", %lld queries)",
                  static_cast<long long>(args.ompl_queries));
    }
  }
  std::printf("\n");
  if (args.summary) {
    // No header and no rows: the figures below.
  } else if (args.pretty) {
    std::printf("%6s %5s %9s %10s %8s %9s %8s %9s %8s %8s  %s\n", "seed",
                "found", "search_ms", "shorten_ms", "queries", "waypoints",
                "length_m", "clear_mm", "self_mm", "time_s", "note");
  } else {
    std::printf("seed,found,search_ms,shorten_ms,queries,waypoints,length_m,"
                "clearance_mm,self_clearance_mm,duration_s,note\n");
  }
  std::vector<double> search, shorten, total, queries, length, duration,
      clearance, self_clearance;
  int found = 0;
  int other_side = 0;
  for (const Row& row : rows) {
    const cell::MotionPlanReport& r = row.report;
    std::string note = row.found ? "" : std::string(cell::ToString(r.status)) + ": " + r.reason;
    if (row.found) {
      note = r.clearance_between;
      if (std::abs(r.goal[0] - goal[0]) > 1.0) {
        note = "other way round; " + note;
        ++other_side;
      }
    }
    if (!args.summary) {
      std::printf(format, row.base_seed, row.found ? "yes" : "no",
                  1e3 * r.search_seconds, 1e3 * r.shorten_seconds,
                  static_cast<long long>(r.search_queries + r.shorten_queries),
                  r.waypoints, r.length, row.found ? 1e3 * r.clearance : 0.0,
                  row.found ? 1e3 * r.self_clearance : 0.0, row.duration,
                  note.c_str());
    }
    search.push_back(1e3 * r.search_seconds);
    shorten.push_back(1e3 * r.shorten_seconds);
    total.push_back(1e3 * (r.search_seconds + r.shorten_seconds));
    queries.push_back(static_cast<double>(r.search_queries + r.shorten_queries));
    if (row.found) self_clearance.push_back(1e3 * r.self_clearance);
    if (row.found) {
      ++found;
      length.push_back(r.length);
      duration.push_back(row.duration);
      clearance.push_back(1e3 * r.clearance);
    }
  }
  const double min_clearance =
      clearance.empty() ? 0.0 : *std::min_element(clearance.begin(), clearance.end());
  if (args.summary) {
    auto smallest = [](const std::vector<double>& v) {
      return v.empty() ? 0.0 : *std::min_element(v.begin(), v.end());
    };
    std::printf("plans,found,queries_median,queries_worst,ms_median,ms_worst,"
                "length_median,length_worst,clearance_min_mm,"
                "self_clearance_min_mm,duration_median_s,duration_worst_s\n");
    std::printf("%d,%d,%.1f,%.0f,%.2f,%.2f,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f\n",
                static_cast<int>(rows.size()), found, Median(queries),
                Max(queries), Median(total), Max(total), Median(length),
                Max(length), smallest(clearance), smallest(self_clearance),
                Median(duration), Max(duration));
  }
  std::printf("# found %d of %zu (%.0f%%)%s\n", found, rows.size(),
              rows.empty() ? 0.0 : 100.0 * found / rows.size(),
              other_side ? (", " + std::to_string(other_side) +
                            " the other way round").c_str()
                         : "");
  std::printf("# planning ms, median / worst: search %.1f / %.1f, shorten "
              "%.1f / %.1f, both %.1f / %.1f\n",
              Median(search), Max(search), Median(shorten), Max(shorten),
              Median(total), Max(total));
  std::printf("# queries median / worst: %.0f / %.0f\n", Median(queries),
              Max(queries));
  std::printf("# length m of motion, median / worst: %.2f / %.2f; scripted "
              "%.2f\n",
              Median(length), Max(length), scripted_length);
  std::printf("# smallest clearance mm, median / worst: %.1f / %.1f; scripted "
              "%.1f (%s)\n",
              Median(clearance), min_clearance, 1e3 * scripted_clearance,
              scripted_between.c_str());
  std::printf("# trajectory s, median / worst: %.2f / %.2f; scripted moves "
              "(stopping at each waypoint) %.2f\n",
              Median(duration), Max(duration), scripted_duration);
  return 0;
}
