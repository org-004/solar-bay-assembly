#include "src/ompl_search.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/AORRTC.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/Console.h>

#include "src/rrt.h"

namespace cell {
namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Clock = std::chrono::steady_clock;

double SecondsSince(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

bool PositiveFinite(double x) { return std::isfinite(x) && x > 0.0; }

// The planners this file can make, the default first, and whether a single
// plan of each repeats from its seed (see Seeds in the header).
struct KnownPlanner {
  const char* name;
  bool repeats;
};
constexpr KnownPlanner kPlanners[] = {
    {"RRTConnect", true},      {"BiTRRT", true},
    {"BITstar", false},        {"AORRTC", false},
    {"InformedRRTstar", false},
};

// As src/rrt.cc: the given weights, or all ones if none were given; nothing
// if they are the wrong length or not all positive and finite.
std::optional<Eigen::VectorXd> WeightsFor(const Eigen::VectorXd& weights,
                                          Eigen::Index dimension) {
  if (weights.size() == 0) return Eigen::VectorXd::Ones(dimension);
  if (weights.size() != dimension || !weights.allFinite() ||
      !(weights.array() > 0.0).all()) {
    return std::nullopt;
  }
  return weights;
}

// OMPL's uniform sampler for a box, with its generator seeded by this file
// instead of from OMPL's process-wide sequence of seeds.
class SeededSampler : public ob::RealVectorStateSampler {
 public:
  SeededSampler(const ob::StateSpace* space, std::uint32_t seed)
      : ob::RealVectorStateSampler(space) {
    rng_.setLocalSeed(seed);
  }
};

// A box of joint angles in the scaled coordinates, every joint an interval,
// whose samplers are all SeededSamplers with the search's seed. OMPL has two
// ways of asking a space for a sampler: allocStateSampler, which RRTConnect
// and BiTRRT use, and allocDefaultStateSampler, which the informed samplers
// of the optimizing planners use for as long as they have no path. The
// first falls back on the second, so overriding the second serves both.
// Every sampler starts the same sequence; no planner here draws from two.
class SeededBox : public ob::RealVectorStateSpace {
 public:
  SeededBox(unsigned int dimension, std::uint32_t seed)
      : ob::RealVectorStateSpace(dimension), seed_(seed) {}

  ob::StateSamplerPtr allocDefaultStateSampler() const override {
    return std::make_shared<SeededSampler>(this, seed_);
  }

 private:
  std::uint32_t seed_;
};

// What OMPL is given: the space in scaled coordinates x_i = w_i * q_i with
// its box and sampler, and the checks, which turn a state back into joint
// angles and count their calls of IsFree. One per search, used by one
// thread.
struct Problem {
  Eigen::VectorXd weights;
  IsFree is_free;
  double resolution = 0.0;
  EdgeOrder order = EdgeOrder::kBisection;

  // Calls of is_free so far.
  std::int64_t queries = 0;
  // Configurations already found free by the caller (the start and the
  // goal), as OMPL will hold them. OMPL asks about them again; the answer
  // is known, and is given without another call.
  std::vector<Eigen::VectorXd> known_free;

  std::shared_ptr<ob::RealVectorStateSpace> space;
  ob::SpaceInformationPtr si;

  Eigen::Index dimension() const { return weights.size(); }

  // The joint angles of a state of OMPL's.
  Eigen::VectorXd Joints(const ob::State* state) const {
    const double* x = state->as<ob::RealVectorStateSpace::StateType>()->values;
    Eigen::VectorXd q(dimension());
    for (Eigen::Index i = 0; i < dimension(); ++i) q[i] = x[i] / weights[i];
    return q;
  }

  // The joint angles q written into a state of OMPL's.
  void Fill(const Eigen::VectorXd& q, ob::State* state) const {
    double* x = state->as<ob::RealVectorStateSpace::StateType>()->values;
    for (Eigen::Index i = 0; i < dimension(); ++i) x[i] = weights[i] * q[i];
  }

  bool StateIsFree(const ob::State* state) {
    const double* x = state->as<ob::RealVectorStateSpace::StateType>()->values;
    for (const Eigen::VectorXd& known : known_free) {
      bool same = true;
      for (Eigen::Index i = 0; i < dimension() && same; ++i) {
        same = x[i] == known[i];
      }
      if (same) return true;
    }
    ++queries;
    return is_free(Joints(state));
  }
};

// OMPL's question "is the straight segment from s1 to s2 free", answered by
// cell::EdgeIsFree on the joint angles: the points, their order and the
// count of calls are the planner's own (src/rrt.h). s1 is taken to be free,
// as OMPL promises and EdgeIsFree assumes.
class EdgeValidator : public ob::MotionValidator {
 public:
  EdgeValidator(const ob::SpaceInformationPtr& si, Problem* problem)
      : ob::MotionValidator(si), problem_(problem) {}

  bool checkMotion(const ob::State* s1, const ob::State* s2) const override {
    const bool free =
        EdgeIsFree(problem_->Joints(s1), problem_->Joints(s2),
                   problem_->is_free, problem_->resolution, problem_->weights,
                   problem_->order, &problem_->queries);
    if (free) {
      ++valid_;
    } else {
      ++invalid_;
    }
    return free;
  }

  // The same verdict from the same points. When the edge is blocked OMPL
  // also wants a state up to which it is free: that is s1, at fraction 0,
  // because bisection checks the points coarse to fine and so knows of no
  // free stretch from s1 onward. As OMPL's own validator does, a free edge
  // leaves last_valid alone.
  bool checkMotion(const ob::State* s1, const ob::State* s2,
                   std::pair<ob::State*, double>& last_valid) const override {
    const bool free = checkMotion(s1, s2);
    if (!free) {
      if (last_valid.first != nullptr) si_->copyState(last_valid.first, s1);
      last_valid.second = 0.0;
    }
    return free;
  }

 private:
  Problem* problem_;
};

// OMPL prints its messages on standard output. Off, once, for the process.
void SilenceOmpl() {
  static std::once_flag once;
  std::call_once(once, [] { ompl::msg::setLogLevel(ompl::msg::LOG_NONE); });
}

// The space, its box, its sampler and the checks. The inputs have been
// checked by the caller: sizes agree, lower is below upper, the weights
// are whole and positive, the resolution is positive.
std::unique_ptr<Problem> MakeProblem(const Eigen::VectorXd& lower,
                                     const Eigen::VectorXd& upper,
                                     const IsFree& is_free,
                                     const Eigen::VectorXd& weights,
                                     const RrtOptions& options) {
  SilenceOmpl();
  auto problem = std::make_unique<Problem>();
  Problem* p = problem.get();
  p->weights = weights;
  p->is_free = is_free;
  p->resolution = options.edge_resolution;
  p->order = options.edge_order;

  const unsigned int n = static_cast<unsigned int>(weights.size());
  p->space = std::make_shared<SeededBox>(n, options.seed);
  ob::RealVectorBounds bounds(n);
  for (unsigned int i = 0; i < n; ++i) {
    bounds.setLow(i, weights[i] * lower[i]);
    bounds.setHigh(i, weights[i] * upper[i]);
  }
  p->space->setBounds(bounds);

  p->si = std::make_shared<ob::SpaceInformation>(p->space);
  p->si->setStateValidityChecker(
      [p](const ob::State* state) { return p->StateIsFree(state); });
  p->si->setMotionValidator(std::make_shared<EdgeValidator>(p->si, p));
  p->si->setup();
  return problem;
}

// The planner of that name on the problem, or null for a name not among
// kPlanners. `range` zero leaves OMPL to choose its own.
ob::PlannerPtr MakePlanner(const std::string& name,
                           const ob::SpaceInformationPtr& si, double range) {
  if (name == "RRTConnect") {
    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setRange(range);
    return planner;
  }
  if (name == "BiTRRT") {
    auto planner = std::make_shared<og::BiTRRT>(si);
    planner->setRange(range);
    return planner;
  }
  if (name == "BITstar") return std::make_shared<og::BITstar>(si);
  if (name == "AORRTC") {
    auto planner = std::make_shared<og::AORRTC>(si);
    planner->setRange(range);
    return planner;
  }
  if (name == "InformedRRTstar") {
    auto planner = std::make_shared<og::InformedRRTstar>(si);
    planner->setRange(range);
    return planner;
  }
  return nullptr;
}

bool InBox(const Eigen::VectorXd& q, const Eigen::VectorXd& lower,
           const Eigen::VectorXd& upper) {
  return (q.array() >= lower.array()).all() &&
         (q.array() <= upper.array()).all();
}

}  // namespace

const std::vector<std::string>& OmplPlannerNames() {
  static const std::vector<std::string> names = [] {
    std::vector<std::string> out;
    for (const KnownPlanner& p : kPlanners) out.push_back(p.name);
    return out;
  }();
  return names;
}

bool IsOmplPlanner(const std::string& name) {
  for (const KnownPlanner& p : kPlanners) {
    if (name == p.name) return true;
  }
  return false;
}

bool OmplPlannerRepeats(const std::string& name) {
  for (const KnownPlanner& p : kPlanners) {
    if (name == p.name) return p.repeats;
  }
  return false;
}

const char* ToString(OmplStop stop) {
  switch (stop) {
    case OmplStop::kNotStarted:
      return "kNotStarted";
    case OmplStop::kSolved:
      return "kSolved";
    case OmplStop::kQueryLimit:
      return "kQueryLimit";
    case OmplStop::kTimeLimit:
      return "kTimeLimit";
  }
  return "unknown";
}

std::optional<JointPath> PlanOmpl(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const IsFree& is_free, const RrtOptions& options,
    const OmplSettings& settings, RrtReport* report, OmplReport* more) {
  const Clock::time_point began = Clock::now();
  RrtReport local;
  RrtReport& out = report != nullptr ? *report : local;
  out = RrtReport();
  OmplReport local_more;
  OmplReport& extra = more != nullptr ? *more : local_more;
  extra = OmplReport();

  auto fail = [&](RrtStatus status,
                  std::string reason) -> std::optional<JointPath> {
    out.status = status;
    out.seconds = SecondsSince(began);
    extra.reason = std::move(reason);
    return std::nullopt;
  };

  // Input and options, as PlanRrtConnect checks them.
  const Eigen::Index n = start.size();
  if (n == 0 || goal.size() != n || lower.size() != n || upper.size() != n ||
      !start.allFinite() || !goal.allFinite() || !lower.allFinite() ||
      !upper.allFinite() || !(lower.array() < upper.array()).all() ||
      !is_free) {
    return fail(RrtStatus::kBadInput,
                "start, goal and box must be finite and of one length, "
                "with lower below upper");
  }
  const std::optional<Eigen::VectorXd> weights = WeightsFor(options.weights, n);
  if (!weights || !PositiveFinite(options.step_size) ||
      !PositiveFinite(options.edge_resolution) ||
      !(options.max_seconds > 0.0)) {
    return fail(RrtStatus::kBadInput, "an option is out of range");
  }
  if (!IsOmplPlanner(settings.planner)) {
    std::string known;
    for (const std::string& name : OmplPlannerNames()) {
      known += (known.empty() ? "" : ", ") + name;
    }
    return fail(RrtStatus::kBadInput, "no planner named \"" +
                                          settings.planner +
                                          "\"; known: " + known);
  }
  if (settings.max_queries < 1 ||
      (settings.range && !(std::isfinite(*settings.range) &&
                           *settings.range >= 0.0))) {
    return fail(RrtStatus::kBadInput,
                "max_queries must be at least 1 and the range not negative");
  }
  if (!InBox(start, lower, upper)) {
    return fail(RrtStatus::kStartOutOfBounds, "the start is outside the box");
  }
  if (!InBox(goal, lower, upper)) {
    return fail(RrtStatus::kGoalOutOfBounds, "the goal is outside the box");
  }

  std::unique_ptr<Problem> problem =
      MakeProblem(lower, upper, is_free, *weights, options);
  Problem& p = *problem;
  ++p.queries;
  if (!is_free(start)) {
    out.collision_queries = p.queries;
    return fail(RrtStatus::kStartInCollision, "the start is in collision");
  }
  ++p.queries;
  if (!is_free(goal)) {
    out.collision_queries = p.queries;
    return fail(RrtStatus::kGoalInCollision, "the goal is in collision");
  }

  // The problem: one start, one goal state, and the path's length as what
  // an optimizing planner minimizes. The goal's threshold is left at OMPL's
  // default, the smallest step between two doubles near 1: a planner with
  // one tree counts the goal as reached only when a node is nearer than the
  // threshold, strictly, so zero would never be met. A path that ends
  // within it ends at the goal; its last configuration is set to the goal
  // given below.
  ob::State* start_state = p.si->allocState();
  ob::State* goal_state = p.si->allocState();
  p.Fill(start, start_state);
  p.Fill(goal, goal_state);
  p.known_free = {weights->cwiseProduct(start), weights->cwiseProduct(goal)};
  auto definition = std::make_shared<ob::ProblemDefinition>(p.si);
  definition->setStartAndGoalStates(start_state, goal_state);
  p.si->freeState(start_state);
  p.si->freeState(goal_state);
  definition->setOptimizationObjective(
      std::make_shared<ob::PathLengthOptimizationObjective>(p.si));

  const ob::PlannerPtr planner = MakePlanner(
      settings.planner, p.si, settings.range.value_or(options.step_size));
  planner->setProblemDefinition(definition);
  planner->setup();

  // The two limits, asked by the planner in its own thread, once in each of
  // its iterations. The first that is reached is remembered.
  OmplStop limit = OmplStop::kSolved;
  const ob::PlannerTerminationCondition stop([&]() {
    if (limit != OmplStop::kSolved) return true;
    if (p.queries >= settings.max_queries) {
      limit = OmplStop::kQueryLimit;
    } else if (SecondsSince(began) >= options.max_seconds) {
      limit = OmplStop::kTimeLimit;
    }
    return limit != OmplStop::kSolved;
  });
  const ob::PlannerStatus solved = planner->solve(stop);
  extra.stopped_by = limit;
  out.collision_queries = p.queries;

  if (!definition->hasExactSolution()) {
    switch (limit) {
      case OmplStop::kQueryLimit:
        return fail(RrtStatus::kIterationLimit,
                    "no path within the limit of collision queries");
      case OmplStop::kTimeLimit:
        return fail(RrtStatus::kTimeLimit, "no path within the time limit");
      default:
        return fail(RrtStatus::kBadInput,
                    "OMPL's " + settings.planner + " gave up by itself: " +
                        solved.asString());
    }
  }

  // The best exact path, turned back into joint angles. Its ends are the
  // start and goal as given: scaling there and back may move the last bit.
  const og::PathGeometric* solution =
      definition->getSolutionPath()->as<og::PathGeometric>();
  JointPath path;
  path.reserve(solution->getStateCount());
  for (std::size_t i = 0; i < solution->getStateCount(); ++i) {
    path.push_back(p.Joints(solution->getState(i)));
  }
  if (path.size() < 2) path = {start, goal};
  path.front() = start;
  path.back() = goal;
  out.status = RrtStatus::kFound;
  out.path_length = PathLength(path, *weights);
  out.seconds = SecondsSince(began);
  return path;
}

// ------------------------------------------------------------- OmplSpace

struct OmplSpace::Impl {
  std::unique_ptr<Problem> problem;
  ob::StateSamplerPtr sampler;
};

OmplSpace::OmplSpace(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
                     const IsFree& is_free, const RrtOptions& options)
    : impl_(std::make_unique<Impl>()) {
  const Eigen::Index n = lower.size();
  const std::optional<Eigen::VectorXd> weights = WeightsFor(options.weights, n);
  if (n == 0 || upper.size() != n || !lower.allFinite() ||
      !upper.allFinite() || !(lower.array() < upper.array()).all() ||
      !is_free || !weights || !PositiveFinite(options.edge_resolution)) {
    return;
  }
  impl_->problem = MakeProblem(lower, upper, is_free, *weights, options);
  impl_->sampler = impl_->problem->si->allocStateSampler();
}

OmplSpace::~OmplSpace() = default;

bool OmplSpace::ok() const { return impl_->problem != nullptr; }

double OmplSpace::Distance(const Eigen::VectorXd& a,
                           const Eigen::VectorXd& b) const {
  if (!ok()) return 0.0;
  const Problem& p = *impl_->problem;
  if (a.size() != p.dimension() || b.size() != p.dimension()) return 0.0;
  ob::State* s1 = p.si->allocState();
  ob::State* s2 = p.si->allocState();
  p.Fill(a, s1);
  p.Fill(b, s2);
  const double distance = p.si->distance(s1, s2);
  p.si->freeState(s1);
  p.si->freeState(s2);
  return distance;
}

bool OmplSpace::CheckMotion(const Eigen::VectorXd& from,
                            const Eigen::VectorXd& to,
                            std::int64_t* queries) const {
  if (!ok()) return false;
  Problem& p = *impl_->problem;
  if (from.size() != p.dimension() || to.size() != p.dimension()) return false;
  ob::State* s1 = p.si->allocState();
  ob::State* s2 = p.si->allocState();
  p.Fill(from, s1);
  p.Fill(to, s2);
  const std::int64_t before = p.queries;
  const bool free = p.si->checkMotion(s1, s2);
  if (queries != nullptr) *queries += p.queries - before;
  p.si->freeState(s1);
  p.si->freeState(s2);
  return free;
}

bool OmplSpace::CheckMotion(const Eigen::VectorXd& from,
                            const Eigen::VectorXd& to,
                            Eigen::VectorXd* last_valid, double* fraction,
                            std::int64_t* queries) const {
  if (!ok()) return false;
  Problem& p = *impl_->problem;
  if (from.size() != p.dimension() || to.size() != p.dimension()) return false;
  ob::State* s1 = p.si->allocState();
  ob::State* s2 = p.si->allocState();
  std::pair<ob::State*, double> last(p.si->allocState(), 1.0);
  p.Fill(from, s1);
  p.Fill(to, s2);
  p.si->copyState(last.first, s2);
  const std::int64_t before = p.queries;
  const bool free = p.si->checkMotion(s1, s2, last);
  if (queries != nullptr) *queries += p.queries - before;
  if (last_valid != nullptr) *last_valid = p.Joints(last.first);
  if (fraction != nullptr) *fraction = last.second;
  p.si->freeState(s1);
  p.si->freeState(s2);
  p.si->freeState(last.first);
  return free;
}

Eigen::VectorXd OmplSpace::Sample() {
  if (!ok()) return {};
  const Problem& p = *impl_->problem;
  ob::State* state = p.si->allocState();
  impl_->sampler->sampleUniform(state);
  Eigen::VectorXd q = p.Joints(state);
  p.si->freeState(state);
  return q;
}

}  // namespace cell
