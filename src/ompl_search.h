// A second search for the planner: OMPL (the Open Motion Planning Library,
// https://ompl.kavrakilab.org) posed exactly the problem that PlanRrtConnect
// of src/rrt.h is posed.
//
//   RrtOptions options;            // step, edge resolution, seed, weights
//   OmplSettings settings;         // which of OMPL's planners, and its budget
//   RrtReport report;
//   std::optional<JointPath> raw = PlanOmpl(
//       start, goal, lower, upper, is_free, options, settings, &report);
//
// It is to OMPL what PlanRrtConnect is to src/rrt.cc: given a start, a goal,
// the joint box, an IsFree and an RrtOptions, it returns a raw path or
// nothing and fills an RrtReport. PlanMove (src/motion_planner.h) calls one
// or the other, by MotionPlanOptions::search, and everything round the call
// is the same code for both. So that a comparison of the two is a comparison
// of the searches and of nothing else, OMPL is not left to its defaults
// where the planner of src/rrt.h has decided something:
//
//   The box. OMPL samples in the box [lower, upper] it is given here and
//   nowhere else. Every joint is an interval (a RealVectorStateSpace); no
//   joint is a circle, as in src/rrt.h.
//
//   The metric. OMPL plans in the scaled coordinates x_i = w_i * q_i, with
//   w the RrtOptions' weights. Its own Euclidean distance between two
//   states is then cell::Distance(a, b, weights), so its nearest
//   neighbours, its range and its path lengths are in the unit of
//   RrtOptions::step_size ("metres of motion" in the cell). Every
//   configuration that leaves this file is turned back, q_i = x_i / w_i,
//   which is q to within one unit in the last place; the two ends of a path
//   returned are the start and goal given, bit for bit.
//
//   Edges. OMPL asks a "motion validator" whether a straight segment is
//   free. Its default one spaces its checks by Euclidean distance. The one
//   used here calls cell::EdgeIsFree with the RrtOptions' edge_resolution,
//   weights and edge_order: the same points checked in the same order, so
//   the same verdict, and on a free edge the same number of IsFree calls.
//   The margin budget of src/motion_planner.h rests on that spacing, and so
//   holds for a path of OMPL's as it does for one of src/rrt.h.
//
//   Limits. A search ends when a planner that only seeks a path has one;
//   when it has made OmplSettings::max_queries calls of IsFree; or when
//   RrtOptions::max_seconds have passed on std::chrono::steady_clock. The
//   count of calls is the limit to rely on: it is the same on every machine,
//   a time is not. Both limits are looked at where OMPL asks whether to
//   stop, once in each iteration of a planner, so a search can overrun
//   either by one iteration's work, as PlanRrtConnect can. OMPL's
//   solve(seconds) is not used: from one second on it starts a timer thread.
//   RrtOptions::max_iterations is not used either; an iteration means
//   something else in each of OMPL's planners.
//
//   Solutions. Only an exact solution is returned: a path that ends at the
//   goal. OMPL's "approximate" solutions, which stop short of it, count as
//   nothing found.
//
//   Seeds. Every state sampler OMPL allocates for the space is seeded with
//   RrtOptions::seed. Nothing here sets or reads OMPL's process-wide seed
//   (ompl::RNG::setSeed). Whether a single plan repeats then depends on
//   what else a planner draws from generators of its own, which OMPL seeds
//   from that process-wide sequence (started from the clock) and, in 2.0.2,
//   keeps private:
//     RRTConnect       repeats: the same seed, problem and settings give the
//                      same path, in this process or another, whatever was
//                      planned before and whatever other threads plan
//                      meanwhile (unless max_seconds ends the search). Its
//                      samples come from the state sampler alone. The tree
//                      it keeps its nodes in for nearest-neighbour search
//                      (a GNAT) does pick its pivots with a generator of
//                      its own, but that changes how the tree is laid out,
//                      not which node it finds nearest.
//     BiTRRT           repeats, likewise.
//     BITstar          does not: once it has a path it samples through an
//                      "informed sampler" with a generator of its own.
//     AORRTC           does not: its cost samples and the path simplifier
//                      inside it have generators of their own.
//     InformedRRTstar  does not: its goal bias and its informed sampler
//                      have generators of their own.
//   OmplPlannerRepeats says the same in code. For the three that do not,
//   the seed fixes only the samples drawn before the first path; two runs
//   differ, and a table of them is a sample, not a record. A consequence
//   worth knowing of the seeding: OMPL's RRTConnect draws the very samples
//   PlanRrtConnect draws (both seed a std::mt19937 with the seed and take
//   one uniform number per joint), and on the cell's moves it then grows
//   the same trees. See docs/ompl-comparison.md.
//
//   Threads. A search runs in the calling thread and starts none. Several
//   threads may each run a search on a problem of their own at once; each
//   call builds its own OMPL objects and shares none.
//
// What is not made equal, because it is the search itself: how a planner
// spends its iterations, its nearest-neighbour structure, and
// RrtOptions::try_direct_first and goal_bias, which belong to
// PlanRrtConnect and are not used here.
//
// The objective every planner is given is the path's length in the metric
// above. OMPL's own path simplifier is not run on the result: the caller
// shortens a path of either search with cell::ShortenPath.
//
// AORRTC is the exception to two things said above. It runs OMPL's path
// simplifier inside its own loop; that is part of that planner. And the
// simplifier's last step (PathGeometric::checkAndRepair) may move a
// configuration of the path and, if it finds no free place for it, leave it
// where no check passed, which AORRTC does not look at. So a path of
// AORRTC's is checked once more here, every segment by cell::EdgeIsFree,
// before it is returned; the calls are counted, and a path that fails is
// not returned. The other planners' paths consist of edges their trees
// already passed through the same check, and are not checked twice.
//
// OMPL writes its messages to standard output; the first search of a
// process turns them off, for the whole process (ompl::msg::setLogLevel).
//
// Units: as src/rrt.h. This header includes nothing of OMPL's, so including
// it costs a file none of OMPL's or Boost's headers.

#ifndef SRC_OMPL_SEARCH_H_
#define SRC_OMPL_SEARCH_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "src/rrt.h"

namespace cell {

// OMPL's own settings for one search; the rest comes from the RrtOptions.
struct OmplSettings {
  // The planner, by its name in OMPL: one of OmplPlannerNames(). A name not
  // among them is refused (RrtStatus::kBadInput, with the reason in
  // OmplReport::reason); it is never replaced by a default.
  //   RRTConnect       the same algorithm as src/rrt.h. Stops at its first
  //                    path.
  //   BiTRRT           two trees that also weigh the cost of a step. Stops
  //                    at its first path.
  //   BITstar          improves its path for as long as it is given: runs
  //                    until max_queries or max_seconds and returns the best
  //                    exact path it has then.
  //   AORRTC           likewise: RRT-Connect run again and again under a
  //                    falling bound on the cost.
  //   InformedRRTstar  likewise: RRT* that samples only where a shorter
  //                    path could still pass.
  std::string planner = "RRTConnect";

  // The planner's range, its longest step toward a sample, as a weighted
  // distance (the unit of RrtOptions::step_size). Not set: the RrtOptions'
  // step_size, so that both searches take steps of one length. Zero: OMPL's
  // own choice, which for RRTConnect is a fifth of the box's diagonal in
  // the metric and for BiTRRT 3.5% of it. BITstar has no range and takes no
  // notice.
  std::optional<double> range;

  // The most calls of IsFree in one search. The default stands for the
  // 2,000 iterations PlanMove allows its own search, at 100 calls each: a
  // 3 m step checked every 30 mm. It is a round figure for "as much work",
  // not a bound on what 2,000 iterations can cost (a step that turns
  // several joints is checked at more points, and a connect takes several
  // steps).
  std::int64_t max_queries = 200000;
};

// The names OmplSettings::planner may have, the default first.
const std::vector<std::string>& OmplPlannerNames();

// Whether `name` is one of them.
bool IsOmplPlanner(const std::string& name);

// Whether a single plan of this planner repeats from its seed alone (see
// Seeds at the top). False for a name that is not a planner's.
bool OmplPlannerRepeats(const std::string& name);

// What ended a search.
enum class OmplStop {
  kNotStarted,   // Refused before OMPL was asked: bad input, a start or goal
                 // outside the box or in collision, an unknown planner.
  kSolved,       // The planner returned by itself; one that only seeks a
                 // path does when it has one.
  kQueryLimit,   // OmplSettings::max_queries calls had been made.
  kTimeLimit,    // RrtOptions::max_seconds had passed.
};

// A short name for it, such as "kSolved", for messages.
const char* ToString(OmplStop stop);

// What an RrtReport has no place for. The RrtReport itself is filled as
// PlanRrtConnect fills it: status (kFound whenever a path is returned;
// kIterationLimit when max_queries ended the search without one, the limit
// that stands where max_iterations stands; kTimeLimit when max_seconds did;
// and the refusals of src/rrt.h), collision_queries (every call of IsFree
// made, the start's and the goal's included), seconds and path_length. Its
// iterations and tree sizes stay zero: OMPL's planners do not tell.
struct OmplReport {
  // What ended the search. For a planner that goes on improving this is a
  // limit even when a path is returned.
  OmplStop stopped_by = OmplStop::kNotStarted;
  // Why nothing was planned, in words, when it was refused; else empty.
  std::string reason;
};

// A collision-free path from `start` to `goal`, both included, inside the
// joint box [lower, upper], found by the OMPL planner the settings name.
// Every straight segment between neighbours has been checked at
// options.edge_resolution, as PlanRrtConnect's are. Neighbours may be
// farther apart than the range (BITstar's are).
//
// Uses of `options`: step_size (the range, unless the settings give one),
// edge_resolution, edge_order, max_seconds, seed and weights, each checked
// as PlanRrtConnect checks it.
//
// Returns nothing if the input is bad, the planner's name is not known,
// start or goal is outside the box or in collision, no exact path was
// found within the limits, or (AORRTC only, see the top) the path found did
// not pass the edge check; `report` and `more` say which.
std::optional<JointPath> PlanOmpl(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const IsFree& is_free, const RrtOptions& options = {},
    const OmplSettings& settings = {}, RrtReport* report = nullptr,
    OmplReport* more = nullptr);

// What OMPL is given by PlanOmpl, to ask it questions directly: its
// distance between two configurations, its verdict on an edge, its samples.
// Built from the same code PlanOmpl builds its problem with, so that a test
// of this is a test of that. Of `options` it uses weights, edge_resolution,
// edge_order and seed. Check ok() before anything else.
class OmplSpace {
 public:
  OmplSpace(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
            const IsFree& is_free, const RrtOptions& options = {});
  ~OmplSpace();
  OmplSpace(const OmplSpace&) = delete;
  OmplSpace& operator=(const OmplSpace&) = delete;

  // False if the box, the weights or the resolution are not usable; every
  // function below then returns zero, false or an empty vector.
  bool ok() const;

  // The distance OMPL's space gives between two configurations.
  double Distance(const Eigen::VectorXd& a, const Eigen::VectorXd& b) const;

  // OMPL's motion validator asked about the edge from `from` to `to`:
  // ompl::base::MotionValidator::checkMotion(s1, s2). Adds the calls of
  // IsFree it made to *queries if given.
  bool CheckMotion(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                   std::int64_t* queries = nullptr) const;

  // The validator's other function, checkMotion(s1, s2, last_valid), which
  // also names a state up to which the edge is known to be free and how far
  // along that is, from 0 to 1. The verdict and the points checked are the
  // same as above. For a free edge that state is `to`, at 1. For a blocked
  // one it is `from`, at 0: the points are checked coarse to fine, not in
  // order, so nothing nearer the block is known to be reachable.
  bool CheckMotion(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                   Eigen::VectorXd* last_valid, double* fraction,
                   std::int64_t* queries = nullptr) const;

  // The next configuration from the state sampler, seeded with
  // options.seed when this object was made.
  Eigen::VectorXd Sample();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace cell

#endif  // SRC_OMPL_SEARCH_H_
