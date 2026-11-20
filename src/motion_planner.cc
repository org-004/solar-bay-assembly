#include "src/motion_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "src/arm.h"
#include "src/collision.h"
#include "src/path_timing.h"
#include "src/rrt.h"

namespace cell {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double kTwoPi = 2.0 * std::numbers::pi;

// The joints whose box is narrowed around start and goal: 4 and 6, the
// +-350 degree ones (0-based).
constexpr int kWristJoints[] = {3, 5};

double SecondsSince(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

bool PositiveFinite(double x) { return std::isfinite(x) && x > 0.0; }
bool NonNegativeFinite(double x) { return std::isfinite(x) && x >= 0.0; }

// The seed of plan k from a base seed: SplitMix64 of the two together, so
// that neighbouring base seeds do not share any of their plans' seeds (base
// + k would: base 1's second plan would be base 2's first).
std::uint32_t PlanSeed(std::uint32_t base, int k) {
  std::uint64_t z = (static_cast<std::uint64_t>(base) << 32) +
                    static_cast<std::uint64_t>(k) + 0x9E3779B97F4A7C15ULL;
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  z ^= z >> 31;
  return static_cast<std::uint32_t>(z ^ (z >> 32));
}

std::string Millimetres(double metres) {
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f mm", 1e3 * metres);
  return text;
}

// Puts the checker's margins back as they were when it goes out of scope,
// whichever way PlanMove returns.
class MarginGuard {
 public:
  explicit MarginGuard(CollisionChecker& checker)
      : checker_(checker),
        margin_(checker.margin()),
        self_margin_(checker.self_margin()) {}
  ~MarginGuard() { checker_.SetMargins(margin_, self_margin_); }
  MarginGuard(const MarginGuard&) = delete;
  MarginGuard& operator=(const MarginGuard&) = delete;

 private:
  CollisionChecker& checker_;
  double margin_;
  double self_margin_;
};

bool InRange(const Arm& arm, const Eigen::VectorXd& q) {
  return (q.array() >= arm.lower.array()).all() &&
         (q.array() <= arm.upper.array()).all();
}

// What is in the way at `q`, in words: the first colliding pair and how
// far apart it is.
std::string Obstruction(CollisionChecker& checker, const Eigen::VectorXd& q) {
  std::optional<CollisionReport> check = checker.Check(q);
  if (!check || check->pairs.empty()) return "?";
  return check->pairs[0].between + " at " +
         Millimetres(check->pairs[0].distance);
}

// Measures the smallest clearance along `path` into the report.
void MeasureClearance(CollisionChecker& checker, const JointPath& path,
                      double spacing, MotionPlanReport* report) {
  const Clock::time_point began = Clock::now();
  report->clearance = std::numeric_limits<double>::infinity();
  report->self_clearance = std::numeric_limits<double>::infinity();
  CheckOptions measure;
  measure.clearance = true;
  for (const Eigen::VectorXd& q : DenseSamples(checker, path, spacing)) {
    std::optional<CollisionReport> check = checker.Check(q, measure);
    ++report->check_queries;
    if (!check) continue;
    if (check->clearance && *check->clearance < report->clearance) {
      report->clearance = *check->clearance;
      report->clearance_between = check->clearance_between;
    }
    if (check->self_clearance &&
        *check->self_clearance < report->self_clearance) {
      report->self_clearance = *check->self_clearance;
      report->self_clearance_between = check->self_clearance_between;
    }
  }
  report->check_seconds += SecondsSince(began);
}

}  // namespace

const char* ToString(MotionPlanStatus status) {
  switch (status) {
    case MotionPlanStatus::kFound:
      return "kFound";
    case MotionPlanStatus::kBadInput:
      return "kBadInput";
    case MotionPlanStatus::kStartOutOfRange:
      return "kStartOutOfRange";
    case MotionPlanStatus::kGoalOutOfRange:
      return "kGoalOutOfRange";
    case MotionPlanStatus::kStartNotClear:
      return "kStartNotClear";
    case MotionPlanStatus::kGoalNotClear:
      return "kGoalNotClear";
    case MotionPlanStatus::kNotFound:
      return "kNotFound";
    case MotionPlanStatus::kTimeBudget:
      return "kTimeBudget";
  }
  return "?";
}

JointPath DenseSamples(const CollisionChecker& checker, const JointPath& path,
                       double spacing) {
  JointPath samples;
  if (path.empty() || !PositiveFinite(spacing)) return samples;
  samples.push_back(path[0]);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::VectorXd& from = path[i - 1];
    const Eigen::VectorXd& to = path[i];
    const int n = std::max(
        1, static_cast<int>(std::ceil(checker.SweepBound(from, to) / spacing)));
    for (int k = 1; k < n; ++k) {
      samples.push_back(from + (static_cast<double>(k) / n) * (to - from));
    }
    samples.push_back(to);
  }
  return samples;
}

std::optional<JointPath> PlanMove(CollisionChecker& checker,
                                  const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal,
                                  const MotionPlanOptions& options,
                                  MotionPlanReport* report) {
  MotionPlanReport local;
  MotionPlanReport& r = report != nullptr ? *report : local;
  r = MotionPlanReport{};
  const Clock::time_point began = Clock::now();
  auto fail = [&](MotionPlanStatus status, std::string reason) {
    r.status = status;
    r.reason = std::move(reason);
    return std::nullopt;
  };

  const Arm& arm = checker.arm();
  const int n = Arm::kJoints;
  if (start.size() != n || goal.size() != n || !start.allFinite() ||
      !goal.allFinite()) {
    return fail(MotionPlanStatus::kBadInput,
                "start and goal must be six finite angles");
  }
  if (!NonNegativeFinite(options.required_margin) ||
      !NonNegativeFinite(options.required_self_margin) ||
      !PositiveFinite(options.edge_resolution) ||
      !NonNegativeFinite(options.corner_allowance) ||
      !NonNegativeFinite(options.tracking_allowance) ||
      !NonNegativeFinite(options.limit_inset) ||
      !NonNegativeFinite(options.wrist_slack) ||
      !PositiveFinite(options.step) || options.max_iterations < 1 ||
      options.seeds < 1 || options.rounds < 1 ||
      !PositiveFinite(options.time_budget) ||
      options.shorten_iterations < 0 ||
      !PositiveFinite(options.resample_spacing) ||
      !PositiveFinite(options.clearance_spacing)) {
    return fail(MotionPlanStatus::kBadInput, "an option is out of range");
  }
  if (!InRange(arm, start)) {
    return fail(MotionPlanStatus::kStartOutOfRange,
                "the start is outside the arm's joint ranges");
  }
  if (!InRange(arm, goal)) {
    return fail(MotionPlanStatus::kGoalOutOfRange,
                "the goal is outside the arm's joint ranges");
  }

  // The metric, from the checker as it is now (with the module, if held).
  r.weights = checker.LeverArms();

  // The margin budget (see the header): plan with the required margins
  // plus half the edge resolution, the corner allowance and the tracking
  // allowance.
  const double budget = 0.5 * options.edge_resolution +
                        options.corner_allowance + options.tracking_allowance;
  r.planning_margin = options.required_margin + budget;
  r.planning_self_margin = options.required_self_margin + budget;
  r.max_deviation = options.corner_allowance / r.weights.norm();
  r.tracking_bound = options.tracking_allowance / r.weights.sum();
  std::optional<MarginGuard> guard;
  guard.emplace(checker);
  if (!checker.SetMargins(r.planning_margin, r.planning_self_margin)) {
    return fail(MotionPlanStatus::kBadInput,
                "the checker refused the planning margins " +
                    Millimetres(r.planning_margin) + " and " +
                    Millimetres(r.planning_self_margin));
  }

  // The joint box: the ranges shrunk by the inset; joints 4 and 6 only
  // over the span the move needs plus the slack; and wide enough for start
  // and goal whatever else.
  r.lower = arm.lower.array() + options.limit_inset;
  r.upper = arm.upper.array() - options.limit_inset;

  // Which goals: the one given, and with try_both_ways joint 1 a whole turn
  // the other way, if that is within range and clear.
  std::vector<Eigen::VectorXd> goals = {goal};
  if (options.try_both_ways) {
    Eigen::VectorXd other = goal;
    other[0] += goal[0] > start[0] ? -kTwoPi : kTwoPi;
    if (other[0] >= r.lower[0] && other[0] <= r.upper[0]) {
      goals.push_back(other);
    }
  }
  for (int j : kWristJoints) {
    double low = start[j];
    double high = start[j];
    for (const Eigen::VectorXd& g : goals) {
      low = std::min(low, g[j]);
      high = std::max(high, g[j]);
    }
    r.lower[j] = std::max(r.lower[j], low - options.wrist_slack);
    r.upper[j] = std::min(r.upper[j], high + options.wrist_slack);
  }
  for (const Eigen::VectorXd& q : goals) {
    r.lower = r.lower.cwiseMin(q);
    r.upper = r.upper.cwiseMax(q);
  }
  r.lower = r.lower.cwiseMin(start);
  r.upper = r.upper.cwiseMax(start);

  // Start and goal must keep the planning margin themselves: RRT would
  // refuse them anyway, but this says what is in the way.
  if (!checker.IsFree(start)) {
    return fail(MotionPlanStatus::kStartNotClear,
                "the start is within the planning margin " +
                    Millimetres(r.planning_margin) + ": " +
                    Obstruction(checker, start));
  }
  if (!checker.IsFree(goal)) {
    return fail(MotionPlanStatus::kGoalNotClear,
                "the goal is within the planning margin " +
                    Millimetres(r.planning_margin) + ": " +
                    Obstruction(checker, goal));
  }
  // The other way round is the same pose, so it is as clear as the goal;
  // checked all the same, since it is only a candidate.
  if (goals.size() > 1 && !checker.IsFree(goals[1])) goals.resize(1);

  const IsFree is_free = [&checker](const Eigen::VectorXd& q) {
    return checker.IsFree(q);
  };

  std::optional<JointPath> best;
  double best_length = std::numeric_limits<double>::infinity();
  bool out_of_time = false;
  int k = 0;  // plans made, for their seeds
  for (int round = 0; round < options.rounds && !best && !out_of_time;
       ++round) {
    ++r.rounds;
    // Twice the iterations each round: a query that needs more than the
    // first round allowed is a hard one, and more of the same seeds' kind
    // of luck does not help as much as a longer search.
    const int iterations =
        options.max_iterations * (1 << std::min(round, 16));
    for (int s = 0; s < options.seeds && !out_of_time; ++s) {
      for (std::size_t g = 0; g < goals.size(); ++g, ++k) {
        const double left = options.time_budget - SecondsSince(began);
        if (left <= 0.0) {
          out_of_time = true;
          break;
        }
        RrtOptions rrt;
        rrt.step_size = options.step;
        rrt.edge_resolution = options.edge_resolution;
        rrt.max_iterations = iterations;
        rrt.max_seconds = left;
        rrt.seed = PlanSeed(options.base_seed, k);
        rrt.weights = r.weights;
        RrtReport found;
        std::optional<JointPath> raw = PlanRrtConnect(
            start, goals[g], r.lower, r.upper, is_free, rrt, &found);
        ++r.plans_tried;
        r.search_seconds += found.seconds;
        r.search_queries += found.collision_queries;
        if (!raw) {
          r.last_rrt_status = found.status;
          if (found.status == RrtStatus::kTimeLimit) out_of_time = true;
          continue;
        }
        ++r.plans_found;

        ShortenOptions shorten;
        shorten.max_iterations = options.shorten_iterations;
        shorten.max_seconds = std::max(
            1e-3, options.time_budget - SecondsSince(began));
        shorten.edge_resolution = options.edge_resolution;
        shorten.weights = r.weights;
        shorten.seed = rrt.seed;
        ShortenReport shortened;
        std::optional<JointPath> path =
            ShortenPath(*raw, is_free, shorten, &shortened);
        r.shorten_seconds += shortened.seconds;
        r.shorten_queries += shortened.collision_queries;
        if (!path) path = std::move(raw);
        const double length = PathLength(*path, r.weights);
        if (length < best_length) {
          best_length = length;
          best = std::move(path);
          r.seed = rrt.seed;
          r.goal = goals[g];
          r.raw_length = found.path_length;
        }
      }
      // A straight line found by the first seed is the shortest path
      // there is; the other seeds would find the same.
      if (best && best->size() >= 2 &&
          PathLength(*best, r.weights) <=
              (1.0 + 1e-9) * Distance(start, r.goal, r.weights)) {
        break;
      }
    }
  }
  if (!best) {
    char text[200];
    std::snprintf(text, sizeof(text),
                  "no path in %d plans over %d rounds (last: %s)",
                  r.plans_tried, r.rounds, ToString(r.last_rrt_status));
    return fail(out_of_time ? MotionPlanStatus::kTimeBudget
                            : MotionPlanStatus::kNotFound,
                text);
  }

  // Resampled: the corners stay, the straight runs get waypoints at most
  // resample_spacing apart. Nothing new needs checking: the segments are
  // the same.
  std::optional<JointPath> resampled =
      Resample(*best, options.resample_spacing, r.weights);
  JointPath path = resampled ? std::move(*resampled) : std::move(*best);
  r.length = PathLength(path, r.weights);
  r.waypoints = static_cast<int>(path.size());
  r.status = MotionPlanStatus::kFound;

  // Measured with the checker's margins put back, as the caller left them
  // (a margin does not change a measured distance, only what is free).
  guard.reset();
  MeasureClearance(checker, path, options.clearance_spacing, &r);
  return path;
}

std::optional<JointPath> PlanThrough(CollisionChecker& checker,
                                     const Eigen::VectorXd& start,
                                     const std::vector<Eigen::VectorXd>& vias,
                                     const Eigen::VectorXd& goal,
                                     const MotionPlanOptions& options,
                                     MotionPlanReport* report) {
  MotionPlanReport local;
  MotionPlanReport& r = report != nullptr ? *report : local;
  r = MotionPlanReport{};
  std::vector<Eigen::VectorXd> ends = {start};
  ends.insert(ends.end(), vias.begin(), vias.end());
  ends.push_back(goal);

  JointPath path;
  r.clearance = std::numeric_limits<double>::infinity();
  r.self_clearance = std::numeric_limits<double>::infinity();
  for (std::size_t leg = 0; leg + 1 < ends.size(); ++leg) {
    MotionPlanOptions leg_options = options;
    // Each leg its own seeds, still all from the one base seed.
    leg_options.base_seed = PlanSeed(options.base_seed, 1000 + leg);
    // Only the last leg may end the other way round: the vias are fixed.
    leg_options.try_both_ways = options.try_both_ways && leg + 2 == ends.size();
    MotionPlanReport part;
    std::optional<JointPath> piece =
        PlanMove(checker, ends[leg], ends[leg + 1], leg_options, &part);
    r.search_seconds += part.search_seconds;
    r.shorten_seconds += part.shorten_seconds;
    r.check_seconds += part.check_seconds;
    r.search_queries += part.search_queries;
    r.shorten_queries += part.shorten_queries;
    r.check_queries += part.check_queries;
    r.plans_tried += part.plans_tried;
    r.plans_found += part.plans_found;
    r.rounds += part.rounds;
    r.raw_length += part.raw_length;
    if (leg == 0) {
      r.seed = part.seed;
      r.lower = part.lower;
      r.upper = part.upper;
    } else if (part.lower.size() == r.lower.size()) {
      r.lower = r.lower.cwiseMin(part.lower);
      r.upper = r.upper.cwiseMax(part.upper);
    }
    r.weights = part.weights;
    r.planning_margin = part.planning_margin;
    r.planning_self_margin = part.planning_self_margin;
    r.max_deviation = part.max_deviation;
    r.tracking_bound = part.tracking_bound;
    r.last_rrt_status = part.last_rrt_status;
    if (!piece) {
      r.status = part.status;
      r.reason = "leg " + std::to_string(leg + 1) + " of " +
                 std::to_string(ends.size() - 1) + ": " + part.reason;
      return std::nullopt;
    }
    if (part.clearance < r.clearance) {
      r.clearance = part.clearance;
      r.clearance_between = part.clearance_between;
    }
    if (part.self_clearance < r.self_clearance) {
      r.self_clearance = part.self_clearance;
      r.self_clearance_between = part.self_clearance_between;
    }
    path.insert(path.end(), piece->begin() + (path.empty() ? 0 : 1),
                piece->end());
    r.goal = part.goal;
  }
  r.status = MotionPlanStatus::kFound;
  r.length = PathLength(path, r.weights);
  r.waypoints = static_cast<int>(path.size());
  return path;
}

std::optional<TimedMove> PlanArmMove(CollisionChecker& checker,
                                     const PlanScene& scene,
                                     const Eigen::VectorXd& start,
                                     const Eigen::VectorXd& goal,
                                     const JointLimits& limits,
                                     const MotionPlanOptions& options,
                                     MotionPlanReport* report) {
  MotionPlanReport local;
  MotionPlanReport& r = report != nullptr ? *report : local;
  auto fail = [&](std::string reason) {
    r = MotionPlanReport{};
    r.status = MotionPlanStatus::kBadInput;
    r.reason = std::move(reason);
    return std::nullopt;
  };

  // The scene, in the order the header gives.
  if (scene.world != nullptr) checker.SetScene(*scene.world);
  if (scene.pallet && !checker.SetPalletPose(*scene.pallet)) {
    return fail("the checker's model has no pallet");
  }
  for (const auto& [i, pose] : scene.modules) {
    if (!checker.SetModulePose(i, pose)) {
      return fail("no module_" + std::to_string(i) + " in the checker's model");
    }
  }
  if (scene.tube_slide && !checker.SetTubeSlide(*scene.tube_slide)) {
    return fail("the checker's model has no conveyor slide");
  }
  if (scene.held_module) {
    if (!checker.SetHeldModule(*scene.held_module, scene.module_in_tip)) {
      return fail("no module_" + std::to_string(*scene.held_module) +
                  " to hold");
    }
  } else {
    checker.ClearHeldModule();
  }
  if (!checker.SetAllowedContacts(scene.allowed_contacts)) {
    return fail("an allowed contact names a body that does not exist");
  }

  std::optional<JointPath> path = PlanMove(checker, start, goal, options, &r);
  if (!path) return std::nullopt;

  PathTimingOptions timing;
  timing.max_deviation = r.max_deviation;
  timing.speed = options.speed;
  std::optional<Trajectory> trajectory =
      Trajectory::Plan(*path, limits, timing);
  if (!trajectory) {
    r.status = MotionPlanStatus::kBadInput;
    r.reason = "the path could not be timed (limits or speed out of range)";
    return std::nullopt;
  }
  return TimedMove{std::move(*path), std::move(*trajectory)};
}

}  // namespace cell
