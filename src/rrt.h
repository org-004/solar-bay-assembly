// A motion planner for the arm: RRT-Connect in joint space, then shortening
// of the path it finds.
//
// The question. The arm carries a 2.3 m module from the pallet side of the
// cell to the conveyor side. Given the joint angles at the start and at the
// goal, find joint angles in between, a path, along which nothing touches
// anything: not the arm itself, not the module, not the post, the conveyor
// or the stack.
//
// Why sample. The set of free joint angles ("free space") has six
// dimensions and no formula: each obstacle in the cell becomes, in joint
// space, a curved region whose shape depends on the arm's and the module's
// geometry at every pose. Writing it down is out of the question, and so is
// covering it with a grid: at 0.01 rad over six joints spanning a few
// radians each, that is about 10^15 cells. What can be done is to take one
// set of joint angles, place the arm and module there, and ask whether
// anything overlaps. That single question, IsFree below, is all a
// sampling-based planner needs. It tries configurations, keeps the free
// ones, and joins them by short straight lines (in joint space) whose
// points it also tries. The cost is counted in such questions.
//
// RRT-Connect. A Rapidly-exploring Random Tree grows from a configuration by
// repeatedly drawing a random configuration in the joint box, finding the
// tree's node nearest to it, and adding a new node a short step from that
// node toward it, if the step is free. Nodes on the edge of the explored
// region are nearest to the most of the box, so they are picked most often:
// the tree is pulled outward into unexplored space, fast. RRT-Connect grows
// two such trees, one from the start and one from the goal. After every step
// of one tree, the other tree tries to reach the new node greedily: step
// after step straight toward it, until it arrives or is blocked (the
// CONNECT). Then the trees swap roles. A path is found when a CONNECT
// arrives; it runs from the start up its tree to the meeting node and down
// the other tree to the goal.
//
// Why two trees meet sooner than one tree reaches the goal. One tree must
// grow until one of its nodes lands where the goal can be seen in a straight
// line, and if the goal sits in a pocket (between the stack and the post,
// say) that region is small and is found only by chance. With two trees the
// goal's own tree grows out of the pocket by itself, and a meeting needs
// only that some node of one tree can see some node of the other: a target
// that is the whole of the other tree, and grows with it. The greedy
// CONNECT crosses the open space between them in one go instead of one
// random step at a time. Each tree also has only about half the way to go.
//
// What "probabilistically complete" promises. If a path with some clearance
// exists, the chance that the planner has found one tends to 1 as it is
// allowed more iterations. It does not promise how many: a narrow passage
// is entered only when a sample happens to fall where a step leads into it,
// and the narrower it is the rarer that is. It cannot tell that no path
// exists: running out of iterations means "not found yet", never "none".
// It does not promise a short path, only some path; ShortenPath is for that.
// And it promises nothing about the parts of an edge between the points
// that were checked; see RrtOptions::edge_resolution.
//
// Joints are treated as straight intervals [lower, upper], not as circles:
// on a joint with a range of +-350 degrees, 170 and -190 degrees are two
// different configurations, 6.3 rad apart, as they are for the cables.
//
// Units: radians, and the caller's units for weighted distances (see
// RrtOptions::weights). Configurations are Eigen::VectorXd of one length,
// one entry per joint. Nothing here knows about the arm or MuJoCo.

#ifndef SRC_RRT_H_
#define SRC_RRT_H_

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace cell {

// True if the configuration is free of collision. Called many times; the
// planner counts the calls. It must give the same answer for the same
// configuration every time, or nothing below is repeatable.
using IsFree = std::function<bool(const Eigen::VectorXd&)>;

using JointPath = std::vector<Eigen::VectorXd>;

// --------------------------------------------------------------- distance
//
// The metric. Distances between configurations, the step size and path
// lengths are all weighted Euclidean distances in joint space:
//
//   |a - b| = sqrt( sum over joints i of (w_i * (a_i - b_i))^2 )
//
// Why weights. A radian at joint 1 swings the module's far corner some 3 to
// 4 m; a radian at joint 6 turns it about its own centre, moving its
// corners about 1.2 m. Unweighted, the planner thinks those moves are equal,
// so it considers a big swing of the base as cheap as a twist of the wrist:
// nearest neighbours and shortcuts favour moves that sweep the module
// through far more of the cell. Setting w_i to roughly the farthest any
// point of the arm or module can be from joint i's axis (metres per radian)
// makes a distance roughly "metres of motion". The default, all ones, is
// plain radians.
//
// Why Euclidean and not, say, the sum of the joint changes: under a sum,
// a zig-zag path that only ever moves joints in one direction each is
// exactly as long as the straight line, so shortening would see nothing to
// gain from straightening it. Under the Euclidean metric the straight line
// is the only shortest path between two configurations.

// The weighted distance above. `weights` empty means all ones.
double Distance(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                const Eigen::VectorXd& weights = {});

// The sum of the weighted distances between neighbours. Zero for fewer than
// two configurations. `weights` empty means all ones.
double PathLength(const JointPath& path, const Eigen::VectorXd& weights = {});

// The same path with configurations added on each straight segment, evenly
// within that segment, so that no two neighbours are more than
// `max_spacing` apart (weighted distance). Every original configuration is
// kept, in particular every corner: spacing evenly along the whole path
// instead would cut the corners, through space that was never checked.
// Returns nothing if max_spacing is not positive and finite, the path is
// empty, or the configurations or the weights differ in length.
std::optional<JointPath> Resample(const JointPath& path, double max_spacing,
                                  const Eigen::VectorXd& weights = {});

// ----------------------------------------------------------- edge checks

// The order in which the points of an edge are checked.
enum class EdgeOrder {
  // Coarse to fine: the far end, then the midpoint, then the quarter
  // points, the eighth points, and so on until neighbouring checked points
  // are within the resolution. The default.
  kBisection,
  // From the near end to the far end, one resolution step at a time.
  kMarching,
};

// Whether the straight segment from `from` to `to` is free, checked at
// evenly spaced points with no more than `resolution` between neighbours,
// measured as the sum over joints of w_i * |change of joint i| (see
// RrtOptions::edge_resolution for why that measure). `from` is taken to be
// free and is not checked; `to` is. Stops at the first point found in
// collision. Adds the number of is_free calls made to *queries if given.
//
// Both orders check the same points and give the same answer; they differ
// in how soon they find a collision. A colliding edge usually collides
// along a stretch of it, not at one point. Marching from one end reaches
// that stretch only after checking everything before it, so the farther
// along the stretch, the longer it takes. Bisection checks a few points
// spread over the whole edge first and then fills in the gaps everywhere at
// once, so it hits any stretch longer than a fraction f of the edge within
// about 2/f checks, wherever the stretch lies. Free edges cost the same
// either way: every point is checked. Most edges the planner rejects are
// rejected after a handful of checks this way: over random colliding edges
// in the circle world, 2.4 queries on average against 136 marching (see
// tests/rrt_test.cc).
//
// Returns false for a resolution that is not positive and finite.
bool EdgeIsFree(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                const IsFree& is_free, double resolution,
                const Eigen::VectorXd& weights = {},
                EdgeOrder order = EdgeOrder::kBisection,
                std::int64_t* queries = nullptr);

// ---------------------------------------------------------------- planner

struct RrtOptions {
  // The longest step a tree takes toward a sample, as a weighted distance
  // (radians with the default weights). A new node is at most this far from
  // its parent, so it is also the largest gap between neighbours of the raw
  // path.
  //
  // Larger steps cross open space in fewer nodes and iterations; but a long
  // step toward a random sample is less likely to line up with a narrow
  // opening, and when blocked it wastes the checks made before the block.
  // On the 6-joint toy world (src/rrt_worlds.h, joint ranges like the
  // arm's, ranges of 2.4 to 12 rad), over 100 seeds, the median collision
  // queries per plan were 51,000 at 0.25 rad (6 seeds failing within 5,000
  // iterations), 30,000 at 0.5, 19,000 at 1.0 and 17,000 at 1.5, but at 1.5
  // the worst seed took 250,000 against 68,000 at 1.0 (all at a resolution
  // of 0.01; `print_rrt --sweep`). In the 2D world with a 0.2-wide gap,
  // where the opening is narrower than the step, 0.5 did better than 1.0:
  // a median of 8,000 queries against 17,000. 1.0 is the default, a little
  // under half the shortest joint range of the arm.
  double step_size = 1.0;

  // The largest gap between checked points along an edge, measured as
  //   sum over joints i of w_i * |change of joint i|
  // which with the default weights is the sum of the joint changes in
  // radians. Between two checked points the planner has not looked, so an
  // obstacle thinner than this gap can be passed straight through, and a
  // corner can be clipped.
  //
  // Why that measure. If w_i is at least the farthest any point of the arm
  // and module is from joint i's axis, then no point of the arm or module
  // moves farther than the measure between two checked points (each joint
  // moves it at most w_i times its turn, and the moves add up at worst). So
  // with weights in metres per radian, `edge_resolution` is a distance in
  // metres that no point moves unchecked, and every pose along the edge is
  // within half of it of a checked pose. Pad the collision geometry by
  // half the resolution (or more) and a free edge is truly free.
  //
  // The default, 0.02 rad, with the module's far corner up to about 4 m
  // from joint 1's axis, lets a point move at most 8 cm between checks, so
  // every pose along an edge is within 4 cm of a checked one: the collision
  // geometry needs 4 cm of padding for the check to be sound. Halve it for
  // 2 cm. It must also be well under the thinnest obstacle in joint space.
  //
  // The cost is one query per resolution along every edge kept, and edges
  // kept are most of the queries: a full step of 1 rad spread over six
  // joints is up to 1 * sqrt(6) / 0.02, about 120 queries, while a blocked
  // step is mostly rejected within a handful (see EdgeOrder). Planning cost
  // is in proportion to 1 / edge_resolution: on the 6-joint toy world a
  // median of 9,700 queries at 0.02 against 19,000 at 0.01.
  double edge_resolution = 0.02;

  EdgeOrder edge_order = EdgeOrder::kBisection;

  // The chance, at each iteration, of growing toward the other tree's root
  // (the start or the goal) instead of toward a random configuration. Not
  // part of classic RRT-Connect, which needs no goal bias: the CONNECT step
  // already pulls each tree toward the other. Off by default: on the
  // 6-joint toy world 0.05 and 0.1 left the median unchanged and made the
  // worst seed two to four times worse (77,000 and 159,000 queries against
  // 35,000), because a tree that keeps reaching for a root it cannot see
  // stops exploring; in the 2D world with the 0.2 gap they found a path on
  // 96 and 97 seeds of 100 against 91. From 0 to 1;
  // at 1 the trees only ever reach for each other's roots and explore
  // nothing.
  double goal_bias = 0.0;

  // The most iterations (one random sample and the step and CONNECT that
  // follow) before giving up with kIterationLimit. The 6-joint toy world
  // needed a median of about 160 and at worst about 750 (see the test).
  int max_iterations = 5000;

  // The most time, in seconds, before giving up with kTimeLimit. Checked
  // once per iteration, so it may be overrun by one iteration's work. A
  // time limit makes the outcome depend on the machine's speed: within the
  // limit the same seed gives the same path, but whether a hard query
  // finishes in time does not repeat exactly. For repeatable failures, let
  // max_iterations be the limit that binds.
  double max_seconds = 5.0;

  // Seeds the random samples, so the same query gives the same path.
  std::uint32_t seed = 1;

  // Per-joint weights for the distance and the edge resolution; see the
  // metric at the top and edge_resolution. Empty means all ones; otherwise
  // one positive, finite entry per joint.
  Eigen::VectorXd weights;

  // Before growing any tree, check the straight line from start to goal and
  // return it (split into steps of step_size) if it is free. Costs one edge
  // check, which bisection usually rejects after a few queries when the
  // line is blocked.
  bool try_direct_first = true;
};

// Why planning ended.
enum class RrtStatus {
  kFound,
  kBadInput,         // Lengths differ, a value is not finite, lower is not
                     // below upper for some joint, or an option is out of
                     // range (step_size and edge_resolution must be positive,
                     // goal_bias in [0, 1], max_iterations at least 1,
                     // max_seconds positive, weights positive).
  kStartOutOfBounds,  // Start outside [lower, upper].
  kGoalOutOfBounds,
  kStartInCollision,
  kGoalInCollision,
  kIterationLimit,  // No path within max_iterations.
  kTimeLimit,       // No path within max_seconds.
};

// A short name for a status, such as "kFound", for messages.
const char* ToString(RrtStatus status);

// How planning went. Filled in whether or not it succeeded.
struct RrtReport {
  RrtStatus status = RrtStatus::kBadInput;
  int iterations = 0;
  // Calls to is_free, all of them: start, goal and every edge point.
  std::int64_t collision_queries = 0;
  // Nodes in each tree at the end, roots included.
  int start_tree_size = 0;
  int goal_tree_size = 0;
  double seconds = 0.0;
  // PathLength of the path found, with the options' weights. Zero if none.
  double path_length = 0.0;
};

// Both trees, for drawing. Filled in only if passed to PlanRrtConnect.
struct RrtTrees {
  // Each tree's nodes and, for each node, the index of its parent in the
  // same tree (-1 for the root, which is node 0).
  JointPath start_nodes;
  std::vector<int> start_parents;
  JointPath goal_nodes;
  std::vector<int> goal_parents;
};

// A collision-free path from `start` to `goal`, both included, inside the
// joint box [lower, upper], found by RRT-Connect. Neighbouring
// configurations are at most options.step_size apart, and every straight
// segment between them has been checked at options.edge_resolution.
//
// Returns nothing if the input is bad, start or goal is outside the box or
// in collision, or no path was found within the limits; `report` says
// which. Never loops for longer than the limits allow.
//
// Deterministic: the same input, options and is_free give the same path,
// unless max_seconds runs out.
std::optional<JointPath> PlanRrtConnect(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
    const IsFree& is_free, const RrtOptions& options = {},
    RrtReport* report = nullptr, RrtTrees* trees = nullptr);

// ------------------------------------------------------------- shortening
//
// Why RRT paths are jagged. Every node is a step toward a random sample, so
// the path is a chain of steps in random directions, and the trees keep the
// first way they found to each node, never a better one. The raw path
// wanders: on the 2D toy worlds, a median of 20 to 50% longer than the
// shortest, and on the 6-joint world twice as long as after shortening.
//
// Why shortcutting is enough here. The shortest path that goes the same way
// round the obstacles is a taut string, straight except where it bends
// round them. Shortcutting pulls the path toward that string: pick two
// points on it, and if the straight line between them is free, replace the
// stretch between with it. 200 tries left every path on the 2D toy worlds
// within 1.5% (round a circle) to 9% (out of a cup) of the string, at a
// median of 1,200 to 5,700 queries (`print_rrt --stats`). The cell's free space is
// mostly open with a few obstacles to get round, so that is close to the
// best path there is. What shortcutting cannot do is change which way round
// an obstacle the path goes: if the planner went the long way round the
// post, the shortened path still does. Planning with a few seeds and
// keeping the shortest result covers that, at the price of a few plans.
// Planners that do approach the shortest path in the limit (RRT*, for one)
// spend far more queries to get there.

struct ShortenOptions {
  // Random shortcuts to try. Each one checks only where it would go, so a
  // try costs a few queries if blocked and the shortcut's whole length if
  // free. On the 2D toy worlds, 100 tries left the worst of 100 paths 2 to
  // 15% longer than the shortest, 200 tries 1.3 to 9%, 400 tries 0.8 to 6%
  // (`print_rrt --shorten-sweep`).
  int max_iterations = 200;
  // The most time, in seconds. Checked once per shortcut tried.
  double max_seconds = 1.0;
  // As in RrtOptions; use the same values as the plan, so that the new
  // segments are checked as finely as the old.
  double edge_resolution = 0.02;
  Eigen::VectorXd weights;
  std::uint32_t seed = 1;
  // A shortcut is tried only if it would save at least this fraction of
  // the stretch of path it replaces. A free shortcut costs its whole length
  // in queries however little it saves, and late in shortening most free
  // ones straighten stretches that are already nearly straight. At 1%,
  // with 200 tries, the queries spent fell by 1.7 to 3.3 times on the toy
  // worlds and the lengths grew by 0.15 to 1%.
  double min_saving = 0.01;
};

struct ShortenReport {
  double length_before = 0.0;
  double length_after = 0.0;
  int iterations = 0;          // Shortcuts tried.
  int shortcuts = 0;           // Shortcuts taken.
  std::int64_t collision_queries = 0;
  double seconds = 0.0;
};

// A shorter path with the same ends, every new straight segment checked at
// options.edge_resolution. Steps:
//   1. Drop configurations that repeat the one before, or that lie on the
//      straight line between their neighbours (no checks needed: the line
//      is the same).
//   2. Drop each configuration whose neighbours can see each other in a
//      straight line.
//   3. Random shortcuts: pick two points anywhere along the path (not only
//      at its configurations), and if the straight line between them is
//      free and shorter, splice it in.
//   4. Steps 2 and 1 again.
// Never longer than the input, and never in collision if the input's
// segments were free (checked at the same resolution). The output's
// neighbours may be far apart; Resample it before timing.
//
// Returns nothing if the path is empty, its configurations or the weights
// differ in length, or an option is out of range. A path of one or two
// configurations comes back as it was.
std::optional<JointPath> ShortenPath(const JointPath& path,
                                     const IsFree& is_free,
                                     const ShortenOptions& options = {},
                                     ShortenReport* report = nullptr);

}  // namespace cell

#endif  // SRC_RRT_H_
