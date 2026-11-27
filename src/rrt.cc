#include "src/rrt.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <utility>

namespace cell {
namespace {

using Clock = std::chrono::steady_clock;

double SecondsSince(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// The weights to use for configurations with `dimension` entries: the given
// ones, or all ones if none were given. Nothing if they are the wrong length
// or not all positive and finite.
std::optional<Eigen::VectorXd> WeightsFor(const Eigen::VectorXd& weights,
                                          Eigen::Index dimension) {
  if (weights.size() == 0) return Eigen::VectorXd::Ones(dimension);
  if (weights.size() != dimension || !weights.allFinite() ||
      !(weights.array() > 0.0).all()) {
    return std::nullopt;
  }
  return weights;
}

bool PathIsWellFormed(const JointPath& path) {
  if (path.empty()) return false;
  for (const Eigen::VectorXd& q : path) {
    if (q.size() != path[0].size() || !q.allFinite()) return false;
  }
  return true;
}

bool PositiveFinite(double x) { return std::isfinite(x) && x > 0.0; }

// The edge check, with weights already made whole.
bool CheckEdge(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
               const IsFree& is_free, double resolution,
               const Eigen::VectorXd& weights, EdgeOrder order,
               std::int64_t* queries) {
  // The edge is split into n equal parts, so the points checked are
  // from + (k / n) (to - from) for k = 1 ... n; k = 0 is `from` itself.
  const double span = (weights.array() * (to - from).array().abs()).sum();
  const double parts = std::ceil(span / resolution);
  // A huge count would mean a resolution far too fine for the edge; cap it
  // so the count fits an int. Two billion checks would never finish anyway.
  const int n = static_cast<int>(std::clamp(parts, 1.0, 1e9));

  std::int64_t count = 0;
  auto free_at = [&](int k) {
    ++count;
    if (k == n) return is_free(to);
    const double t = static_cast<double>(k) / n;
    return is_free(from + t * (to - from));
  };

  bool free = true;
  if (order == EdgeOrder::kMarching) {
    for (int k = 1; k <= n && free; ++k) free = free_at(k);
  } else if ((free = free_at(n))) {
    // Breadth first over the gaps between points already checked: each
    // round halves every gap, so the whole edge is covered coarsely before
    // any part of it finely. A gap of one part has nothing left inside.
    std::vector<std::pair<int, int>> gaps = {{0, n}};
    for (std::size_t next = 0; next < gaps.size() && free; ++next) {
      const auto [low, high] = gaps[next];
      if (high - low < 2) continue;
      const int middle = low + (high - low) / 2;
      free = free_at(middle);
      gaps.push_back({low, middle});
      gaps.push_back({middle, high});
    }
  }
  if (queries != nullptr) *queries += count;
  return free;
}

// One tree of the planner. Its nodes are kept one after another in a flat
// array, `dimension` numbers each, so that the nearest-neighbour scan
// reads memory in order.
class Tree {
 public:
  Tree(const Eigen::VectorXd& root, const Eigen::VectorXd& weights)
      : dimension_(root.size()),
        weights_squared_(weights.array().square().matrix()) {
    Add(root, -1);
  }

  int size() const { return static_cast<int>(parents_.size()); }

  Eigen::Map<const Eigen::VectorXd> node(int i) const {
    return Eigen::Map<const Eigen::VectorXd>(&values_[i * dimension_],
                                             dimension_);
  }

  int parent(int i) const { return parents_[i]; }

  int Add(const Eigen::VectorXd& q, int parent) {
    values_.insert(values_.end(), q.data(), q.data() + dimension_);
    parents_.push_back(parent);
    return size() - 1;
  }

  // The node nearest to q, by a scan of every node.
  //
  // Why a plain scan. A tree here holds tens to a few thousand nodes. A
  // scan costs well under a nanosecond per node of six joints: measured
  // with `print_rrt --nn` in the optimized build, the planner's own work
  // per iteration (two or more scans, plus the arithmetic of the edge
  // checks) was 1.2 us with 500 nodes, 2.0 us with 2,000, 5.5 us with
  // 8,000 and 10 us with 16,000: some 0.6 ns more per node. Every iteration also makes at least one
  // collision query, and on the arm one costs tens of microseconds to a
  // millisecond; on the 6-joint toy world an iteration made 60 on average,
  // a millisecond or more of queries. The scan is a few percent of that up
  // to some 10,000 nodes, more than the planner builds before its iteration
  // limit stops it (the trees there held 97 nodes at the median and 331 at
  // worst). A k-d tree would pay off only beyond that.
  int Nearest(const Eigen::VectorXd& q) const {
    int best = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    const double* values = values_.data();
    const double* target = q.data();
    const double* weights = weights_squared_.data();
    for (int i = 0; i < size(); ++i) {
      const double* node = values + static_cast<std::size_t>(i) * dimension_;
      double distance = 0.0;
      for (Eigen::Index j = 0; j < dimension_; ++j) {
        const double d = node[j] - target[j];
        distance += weights[j] * d * d;
      }
      if (distance < best_distance) {
        best_distance = distance;
        best = i;
      }
    }
    return best;
  }

  // From node i back to the root, i first.
  JointPath PathToRoot(int i) const {
    JointPath path;
    for (; i >= 0; i = parents_[i]) path.push_back(node(i));
    return path;
  }

  void CopyTo(JointPath* nodes, std::vector<int>* parents) const {
    nodes->clear();
    for (int i = 0; i < size(); ++i) nodes->push_back(node(i));
    *parents = parents_;
  }

 private:
  Eigen::Index dimension_;
  Eigen::VectorXd weights_squared_;
  std::vector<double> values_;
  std::vector<int> parents_;
};

enum class Growth {
  kTrapped,   // The first step was blocked: nothing added.
  kAdvanced,  // One step added, short of the target.
  kReached,   // The target itself added.
};

// Everything the trees share while planning.
struct Planner {
  const IsFree& is_free;
  const RrtOptions& options;
  const Eigen::VectorXd& weights;
  std::int64_t queries = 0;

  // One step of `tree` from its node nearest `target` toward it, at most
  // step_size long, kept if the straight line to it is free. *added is the
  // new node's index.
  Growth Extend(Tree& tree, const Eigen::VectorXd& target, int* added) {
    const int near = tree.Nearest(target);
    const Eigen::VectorXd from = tree.node(near);
    const double distance = Distance(from, target, weights);
    // Already in the tree (a goal-biased step can land exactly on the other
    // tree's root): nothing to add, and no zero-length edge in the path.
    if (distance == 0.0) {
      *added = near;
      return Growth::kReached;
    }
    Growth growth = Growth::kReached;
    Eigen::VectorXd to = target;
    if (distance > options.step_size) {
      to = from + (options.step_size / distance) * (target - from);
      growth = Growth::kAdvanced;
    }
    if (!CheckEdge(from, to, is_free, options.edge_resolution, weights,
                   options.edge_order, &queries)) {
      return Growth::kTrapped;
    }
    *added = tree.Add(to, near);
    return growth;
  }

  // Steps toward `target` until it is reached or a step is blocked. The
  // number of steps is bounded: each one that falls short of the target
  // moves the nearest node a full step_size closer to it.
  Growth Connect(Tree& tree, const Eigen::VectorXd& target, int* added) {
    Growth growth = Extend(tree, target, added);
    if (growth != Growth::kAdvanced) return growth;
    while ((growth = Extend(tree, target, added)) == Growth::kAdvanced) {
    }
    // Blocked after at least one step: the tree still advanced.
    return growth == Growth::kTrapped ? Growth::kAdvanced : growth;
  }
};

bool InBox(const Eigen::VectorXd& q, const Eigen::VectorXd& lower,
           const Eigen::VectorXd& upper) {
  return (q.array() >= lower.array()).all() &&
         (q.array() <= upper.array()).all();
}

// Splits every segment longer than `spacing` into equal parts.
JointPath Subdivide(const JointPath& path, double spacing,
                    const Eigen::VectorXd& weights) {
  JointPath out = {path.front()};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::VectorXd& a = path[i - 1];
    const Eigen::VectorXd& b = path[i];
    const double parts = std::ceil(Distance(a, b, weights) / spacing);
    const int n = static_cast<int>(std::clamp(parts, 1.0, 1e7));
    for (int k = 1; k < n; ++k) {
      out.push_back(a + (static_cast<double>(k) / n) * (b - a));
    }
    out.push_back(b);
  }
  return out;
}

// ------------------------------------------------------------- shortening

// Step 1 of ShortenPath: drop repeats and configurations on the straight
// line between their neighbours. A configuration is on that line if going
// through it is no longer than going straight, to rounding.
JointPath DropCollinear(const JointPath& path, const Eigen::VectorXd& weights) {
  JointPath out = {path.front()};
  for (std::size_t i = 1; i < path.size(); ++i) {
    const bool repeat = out.back() == path[i];
    if (i + 1 == path.size()) {
      // The end is always kept, but only once, unless the whole path is
      // one configuration: then it is that configuration twice.
      if (!repeat || out.size() == 1) out.push_back(path[i]);
      continue;
    }
    if (repeat) continue;
    const double through = Distance(out.back(), path[i], weights) +
                           Distance(path[i], path[i + 1], weights);
    const double direct = Distance(out.back(), path[i + 1], weights);
    if (through - direct <= 1e-12 * through) continue;
    out.push_back(path[i]);
  }
  return out;
}

// Step 2 of ShortenPath: walk the path, and drop each configuration whose
// neighbours (the last one kept and the next one) see each other.
JointPath DropRedundant(const JointPath& path, const IsFree& is_free,
                        const ShortenOptions& options,
                        const Eigen::VectorXd& weights,
                        std::int64_t* queries) {
  JointPath out = {path.front()};
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    if (!CheckEdge(out.back(), path[i + 1], is_free, options.edge_resolution,
                   weights, EdgeOrder::kBisection, queries)) {
      out.push_back(path[i]);
    }
  }
  out.push_back(path.back());
  return out;
}

// The distance along the path from its start to each configuration.
std::vector<double> CumulativeLengths(const JointPath& path,
                                      const Eigen::VectorXd& weights) {
  std::vector<double> lengths = {0.0};
  for (std::size_t i = 1; i < path.size(); ++i) {
    lengths.push_back(lengths.back() + Distance(path[i - 1], path[i], weights));
  }
  return lengths;
}

// The point `s` along the path. *segment is the index of the configuration
// at the start of the segment it is on.
Eigen::VectorXd PointAt(const JointPath& path,
                        const std::vector<double>& lengths, double s,
                        int* segment) {
  // The last configuration whose distance along is at most s.
  const auto it = std::upper_bound(lengths.begin(), lengths.end(), s);
  int i = static_cast<int>(it - lengths.begin()) - 1;
  i = std::clamp(i, 0, static_cast<int>(path.size()) - 2);
  *segment = i;
  const double length = lengths[i + 1] - lengths[i];
  const double t = length > 0.0 ? std::clamp((s - lengths[i]) / length, 0.0, 1.0)
                                : 0.0;
  return path[i] + t * (path[i + 1] - path[i]);
}

}  // namespace

double Distance(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                const Eigen::VectorXd& weights) {
  if (weights.size() == 0) return (a - b).norm();
  return (weights.array() * (a - b).array()).matrix().norm();
}

double PathLength(const JointPath& path, const Eigen::VectorXd& weights) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += Distance(path[i - 1], path[i], weights);
  }
  return length;
}

std::optional<JointPath> Resample(const JointPath& path, double max_spacing,
                                  const Eigen::VectorXd& weights) {
  if (!PositiveFinite(max_spacing) || !PathIsWellFormed(path)) {
    return std::nullopt;
  }
  const std::optional<Eigen::VectorXd> w = WeightsFor(weights, path[0].size());
  if (!w) return std::nullopt;
  return Subdivide(path, max_spacing, *w);
}

bool EdgeIsFree(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                const IsFree& is_free, double resolution,
                const Eigen::VectorXd& weights, EdgeOrder order,
                std::int64_t* queries) {
  if (!PositiveFinite(resolution) || from.size() != to.size() ||
      !from.allFinite() || !to.allFinite()) {
    return false;
  }
  const std::optional<Eigen::VectorXd> w = WeightsFor(weights, from.size());
  if (!w) return false;
  return CheckEdge(from, to, is_free, resolution, *w, order, queries);
}

const char* ToString(RrtStatus status) {
  switch (status) {
    case RrtStatus::kFound:
      return "kFound";
    case RrtStatus::kBadInput:
      return "kBadInput";
    case RrtStatus::kStartOutOfBounds:
      return "kStartOutOfBounds";
    case RrtStatus::kGoalOutOfBounds:
      return "kGoalOutOfBounds";
    case RrtStatus::kStartInCollision:
      return "kStartInCollision";
    case RrtStatus::kGoalInCollision:
      return "kGoalInCollision";
    case RrtStatus::kIterationLimit:
      return "kIterationLimit";
    case RrtStatus::kTimeLimit:
      return "kTimeLimit";
  }
  return "unknown";
}

namespace {

// PlanRrtConnect, and, at the end of each iteration whose count is in
// `checkpoints` (increasing, each below options.max_iterations) with no
// path yet, the report a run with that many iterations would have given.
std::optional<JointPath> Plan(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const IsFree& is_free, const RrtOptions& options, RrtReport* report,
    RrtTrees* trees, const std::vector<int>& checkpoints,
    std::vector<RrtReport>* at) {
  const Clock::time_point began = Clock::now();
  RrtReport local;
  RrtReport& out = report != nullptr ? *report : local;
  out = RrtReport();
  if (trees != nullptr) *trees = RrtTrees();

  auto fail = [&](RrtStatus status) -> std::optional<JointPath> {
    out.status = status;
    out.seconds = SecondsSince(began);
    return std::nullopt;
  };

  // Input and options.
  const Eigen::Index n = start.size();
  if (n == 0 || goal.size() != n || lower.size() != n || upper.size() != n ||
      !start.allFinite() || !goal.allFinite() || !lower.allFinite() ||
      !upper.allFinite() || !(lower.array() < upper.array()).all() ||
      !is_free) {
    return fail(RrtStatus::kBadInput);
  }
  const std::optional<Eigen::VectorXd> weights = WeightsFor(options.weights, n);
  if (!weights || !PositiveFinite(options.step_size) ||
      !PositiveFinite(options.edge_resolution) ||
      !(options.goal_bias >= 0.0 && options.goal_bias <= 1.0) ||
      options.max_iterations < 1 || !(options.max_seconds > 0.0)) {
    return fail(RrtStatus::kBadInput);
  }
  if (!InBox(start, lower, upper)) return fail(RrtStatus::kStartOutOfBounds);
  if (!InBox(goal, lower, upper)) return fail(RrtStatus::kGoalOutOfBounds);

  Planner planner{is_free, options, *weights};
  ++planner.queries;
  if (!is_free(start)) {
    out.collision_queries = planner.queries;
    return fail(RrtStatus::kStartInCollision);
  }
  ++planner.queries;
  if (!is_free(goal)) {
    out.collision_queries = planner.queries;
    return fail(RrtStatus::kGoalInCollision);
  }

  Tree start_tree(start, *weights);
  Tree goal_tree(goal, *weights);

  auto finish = [&](JointPath path) -> std::optional<JointPath> {
    out.collision_queries = planner.queries;
    out.start_tree_size = start_tree.size();
    out.goal_tree_size = goal_tree.size();
    if (trees != nullptr) {
      start_tree.CopyTo(&trees->start_nodes, &trees->start_parents);
      goal_tree.CopyTo(&trees->goal_nodes, &trees->goal_parents);
    }
    if (path.empty()) return fail(out.status);
    out.status = RrtStatus::kFound;
    out.path_length = PathLength(path, *weights);
    out.seconds = SecondsSince(began);
    return path;
  };

  if (options.try_direct_first &&
      CheckEdge(start, goal, is_free, options.edge_resolution, *weights,
                options.edge_order, &planner.queries)) {
    return finish(Subdivide({start, goal}, options.step_size, *weights));
  }

  std::mt19937 rng(options.seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  Eigen::VectorXd sample(n);
  std::size_t next_checkpoint = 0;

  // `a` grows toward a sample, `b` then tries to CONNECT to what `a` added.
  // They swap after every iteration, so both trees grow evenly.
  Tree* a = &start_tree;
  Tree* b = &goal_tree;
  out.status = RrtStatus::kIterationLimit;
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    if (SecondsSince(began) > options.max_seconds) {
      out.status = RrtStatus::kTimeLimit;
      break;
    }
    out.iterations = iteration + 1;

    // Draw every number of the sample even when the goal bias replaces it,
    // so that the random sequence does not depend on the bias.
    for (Eigen::Index j = 0; j < n; ++j) {
      sample[j] = lower[j] + unit(rng) * (upper[j] - lower[j]);
    }
    if (options.goal_bias > 0.0 && unit(rng) < options.goal_bias) {
      sample = b->node(0);
    }

    int added_a = -1;
    if (planner.Extend(*a, sample, &added_a) != Growth::kTrapped) {
      const Eigen::VectorXd target = a->node(added_a);
      int added_b = -1;
      if (planner.Connect(*b, target, &added_b) == Growth::kReached) {
        // b's last node is exactly a's new one: join the two halves there.
        JointPath from_a = a->PathToRoot(added_a);   // meeting ... a's root
        JointPath from_b = b->PathToRoot(added_b);   // meeting ... b's root
        JointPath& to_start = a == &start_tree ? from_a : from_b;
        JointPath& to_goal = a == &start_tree ? from_b : from_a;
        JointPath path(to_start.rbegin(), to_start.rend());  // start ... meet
        path.insert(path.end(), to_goal.begin() + 1, to_goal.end());
        return finish(std::move(path));
      }
    }
    std::swap(a, b);
    if (next_checkpoint < checkpoints.size() &&
        iteration + 1 == checkpoints[next_checkpoint]) {
      ++next_checkpoint;
      RrtReport there;
      there.status = RrtStatus::kIterationLimit;
      there.iterations = iteration + 1;
      there.collision_queries = planner.queries;
      there.start_tree_size = start_tree.size();
      there.goal_tree_size = goal_tree.size();
      there.seconds = SecondsSince(began);
      at->push_back(there);
    }
  }
  return finish({});
}

}  // namespace

std::optional<JointPath> PlanRrtConnect(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const IsFree& is_free, const RrtOptions& options, RrtReport* report,
    RrtTrees* trees) {
  return Plan(start, goal, lower, upper, is_free, options, report, trees, {},
              nullptr);
}

std::optional<JointPath> PlanRrtConnectBudgets(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const IsFree& is_free, const RrtOptions& options,
    const std::vector<int>& budgets, std::vector<RrtReport>* reports) {
  reports->clear();
  bool increasing = !budgets.empty() && budgets.front() >= 1;
  for (std::size_t k = 1; k < budgets.size(); ++k) {
    increasing = increasing && budgets[k] > budgets[k - 1];
  }
  if (!increasing) {
    // One run per budget, as the caller would.
    std::optional<JointPath> path;
    RrtOptions o = options;
    for (int budget : budgets) {
      o.max_iterations = budget;
      RrtReport r;
      path = PlanRrtConnect(start, goal, lower, upper, is_free, o, &r);
      reports->push_back(r);
      if (path || r.status != RrtStatus::kIterationLimit) break;
    }
    return path;
  }
  RrtOptions o = options;
  o.max_iterations = budgets.back();
  const std::vector<int> checkpoints(budgets.begin(), budgets.end() - 1);
  std::vector<RrtReport> at;
  RrtReport last;
  std::optional<JointPath> path =
      Plan(start, goal, lower, upper, is_free, o, &last, nullptr, checkpoints, &at);
  // The budgets a path or a failure other than the iteration limit came
  // before were never run out: their reports are the run's own.
  *reports = at;
  reports->push_back(last);
  return path;
}

std::optional<JointPath> ShortenPath(const JointPath& path,
                                     const IsFree& is_free,
                                     const ShortenOptions& options,
                                     ShortenReport* report) {
  const Clock::time_point began = Clock::now();
  ShortenReport local;
  ShortenReport& out = report != nullptr ? *report : local;
  out = ShortenReport();

  if (!PathIsWellFormed(path) || !is_free ||
      !PositiveFinite(options.edge_resolution) || options.max_iterations < 0 ||
      !(options.max_seconds >= 0.0)) {
    return std::nullopt;
  }
  const std::optional<Eigen::VectorXd> w =
      WeightsFor(options.weights, path[0].size());
  if (!w) return std::nullopt;
  const Eigen::VectorXd& weights = *w;

  out.length_before = PathLength(path, weights);
  if (path.size() <= 2) {
    out.length_after = out.length_before;
    out.seconds = SecondsSince(began);
    return path;
  }

  JointPath current = DropCollinear(path, weights);

  std::mt19937 rng(options.seed);
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    if (current.size() <= 2) break;  // Already a straight line.
    if (SecondsSince(began) > options.max_seconds) break;
    out.iterations = iteration + 1;

    const std::vector<double> lengths = CumulativeLengths(current, weights);
    const double total = lengths.back();
    std::uniform_real_distribution<double> along(0.0, total);
    double s1 = along(rng);
    double s2 = along(rng);
    if (s1 > s2) std::swap(s1, s2);
    int segment1 = 0;
    int segment2 = 0;
    const Eigen::VectorXd q1 = PointAt(current, lengths, s1, &segment1);
    const Eigen::VectorXd q2 = PointAt(current, lengths, s2, &segment2);
    // On one segment the path is already straight between them.
    if (segment1 == segment2) continue;

    // Splice only if it saves something beyond rounding: the shortcut is
    // never longer than the stretch it replaces, but may be equal to it.
    const double shortcut = Distance(q1, q2, weights);
    if (!(shortcut < (s2 - s1) * (1.0 - std::max(1e-9, options.min_saving)))) continue;

    // q1 lies on a checked segment but need not itself have been one of
    // the checked points, so it is checked too.
    ++out.collision_queries;
    if (!is_free(q1)) continue;
    if (!CheckEdge(q1, q2, is_free, options.edge_resolution, weights,
                   EdgeOrder::kBisection, &out.collision_queries)) {
      continue;
    }

    JointPath spliced(current.begin(), current.begin() + segment1 + 1);
    spliced.push_back(q1);
    spliced.push_back(q2);
    spliced.insert(spliced.end(), current.begin() + segment2 + 1,
                   current.end());
    current = DropCollinear(spliced, weights);
    ++out.shortcuts;
  }

  current = DropRedundant(current, is_free, options, weights,
                          &out.collision_queries);
  current = DropCollinear(current, weights);

  out.length_after = PathLength(current, weights);
  out.seconds = SecondsSince(began);
  return current;
}

}  // namespace cell
