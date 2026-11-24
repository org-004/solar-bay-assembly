// The motion planner for this cell: the generic RRT-Connect and shortening
// of src/rrt.h, set up for the KR 210 R2700-2 carrying a module, against the
// MuJoCo collision check of src/collision.h, and timed along the path by
// src/path_timing.h.
//
//   CollisionChecker checker = *CollisionChecker::Create(*cell.model());
//   checker.SetScene(*cell.data());           // where the obstacles are
//   checker.SetHeldModule(k, module_in_tip);  // the module is part of the arm
//   MotionPlanReport report;
//   std::optional<JointPath> path = PlanMove(checker, start, goal, {}, &report);
//
// or, all in one call, as the pick and place will use it (PlanArmMove below):
// the obstacles and the held module from a PlanScene, the path, and its
// Trajectory ready for ArmFollower::Follow.
//
// What has to be decided for THIS arm, and why (each is an option below):
//
//   The joint box. RRT samples inside a box of joint angles. Each joint's
//   real range, shrunk a little: the follower clamps commands at the range
//   ends and MuJoCo's limits are soft, so a path that runs along a range end
//   is followed less well there. Joints 4 and 6 turn +-350 degrees; the
//   planner treats a joint as an interval, not a circle (src/rrt.h), so a
//   path that winds joint 6 a whole turn is, to it, a different and longer
//   path, and one the cables would not like. Their box is narrowed to the
//   span between start and goal plus `wrist_slack` on each side: enough to
//   get round things, not enough to wind up.
//
//   The metric. Distances are weighted by the checker's LeverArms(): for
//   each joint the farthest any point of the arm, tool or held module can be
//   from its axis, metres per radian. A distance is then roughly "metres of
//   motion" of the farthest point, so the step, the resolution and the path
//   lengths below are in metres. The weights are read after SetHeldModule:
//   the module adds about 0.7 m to every joint's lever.
//
//   The margin budget (soundness). The checker answers only for the
//   configurations it is asked about. What the arm really does differs from
//   those in three ways, and each is bounded in metres moved by any point of
//   the arm, tool and module:
//     1. Between two checked samples of an edge. RRT checks every edge at
//        `edge_resolution`, measured as sum_i w_i |dq_i| (src/rrt.h): no
//        point moves more than that between neighbouring samples, so every
//        configuration along a free edge is within edge_resolution / 2 of a
//        checked one.
//     2. Corner rounding. The Trajectory leaves the polyline at each corner
//        by at most max_deviation radians of joint-space (Euclidean)
//        distance, which moves a point by at most |w| max_deviation metres
//        (Cauchy-Schwarz, src/path_timing.h). The planner sets
//        max_deviation = corner_allowance / |w|.
//     3. Tracking. The real joints trail the plan by a fraction of a
//        milliradian, and the module gives a little on the weld; each point
//        then strays at most sum_i w_i |error_i| (plus the weld's give).
//        The allowance is given in metres. MotionPlanReport::tracking_bound
//        is the error every joint may have at once and still keep it,
//        allowance / sum_i w_i (0.26 mrad with a module held); the real
//        errors are not all the worst at once. tests/motion_planner_test.cc
//        measures sum_i w_i |error_i| on a carry in the simulated cell: at
//        most 2.5 mm, with the worst joint at 0.6 mrad, and 0.1 mm of give
//        at the module's corners.
//   By the triangle inequality, if every checked sample keeps
//     planning margin = required_margin + edge_resolution / 2
//                       + corner_allowance + tracking_allowance
//   from every obstacle, the real arm keeps required_margin. So the planner
//   plans with the larger margin and leaves the checker as it found it. The
//   same holds for the arm against itself and the held module against the
//   arm: in the frame of the nearer-the-base body of a pair, only the joints
//   between the two move the other, each by at most w_i |dq_i|, so the same
//   three bounds apply and self pairs are planned at
//     required_self_margin + edge_resolution / 2 + corner + tracking.
//
//   How the budget is split, with the defaults: 20 mm required, plus 15 mm
//   for the edges (a 30 mm resolution), 5 mm for the corners (0.6 mrad of
//   deviation) and 5 mm for tracking (twice what was measured): 45 mm to
//   the world, 25 mm among the arm's links and the module. Why so: the
//   carry's start, the end of the scripted "clear", stands only 58 mm from
//   the next module in the stack, so the planning margin must stay well
//   under that, with room for the stack's estimated pose to be off by a
//   centimetre. The resolution is the item that costs: queries go as
//   1 / resolution (20 mm: 40% more planning time). Rounding the corners
//   more buys little: on the carry, 0.6 mrad against 1.2 or 2.4 mrad changes
//   the trajectory's duration by a few per cent, because the joints' speed
//   limits, not the corners, set it.

//   The step. RRT's longest step, 3 m of motion: the carry crosses some
//   20 m of motion in open space, and the narrowest passages (round the
//   upper arm while the module turns, over the rails) are wider than that
//   in joint space. A blocked step costs only a few queries (bisection).
//
//   Seeds. Shortcutting cannot change which way round an obstacle a path
//   goes, so `seeds` plans are made with seeds drawn from `base_seed` and
//   the shortest after shortening is kept. The same base seed gives the
//   same path (unless the time budget runs out).
//
//   Retries. If no seed of a round finds a path, another round is tried
//   with new seeds and twice the iterations, up to `rounds` rounds or
//   `time_budget` seconds in all. Running out of either is "not found", never
//   "no path exists" (src/rrt.h).
//
//   The search. All of the above is decided round one call, the search for
//   a raw path of one seed, and that call can be answered by either of two
//   searches: the RRT-Connect of src/rrt.h, written for this cell and the
//   default, or one of OMPL's planners (src/ompl_search.h), chosen by
//   `search` and set up by `ompl`. OMPL is given the same box, weights, step,
//   edge resolution and edge check, the same seed for the k-th plan and the
//   time that is left; where the own search is limited by `max_iterations`
//   it is limited by a number of collision queries, OmplSettings::max_queries,
//   doubled each round as the iterations are. The margins, the start and
//   goal checks, the shortening, the choice of the shortest, the rounds, the
//   resampling, the clearance measured afterwards and the report are one
//   code for both, so the margin budget above holds for a path of OMPL's as
//   it does for one of the own search. docs/ompl-comparison.md compares the
//   two on the cell's moves. With `search` left alone nothing of OMPL runs.
//
// Which way round. Joint 1 is an interval too: from the pallet (joint 1
// near 0) to the place with joint 1 at -180 degrees every path turns joint 1
// through -90, round the +Y side, away from the camera's post; to the same
// place with joint 1 at +180 it goes round -Y, past the post. The goal's
// joint 1 picks the side, and the planner does not change it unless told
// to: with `try_both_ways` it also plans to the goal with joint 1 turned a
// whole turn the other way (when that is within range) and keeps the
// shorter path; MotionPlanReport::goal says which. Both sides are open in
// this cell, at about the same length: round -Y the module passes 0.2 to
// 0.5 m from the post's column, round +Y some 2 m from anything of the
// camera's (tests/motion_planner_test.cc). Plan to the +Y goal, as the
// script does; try_both_ways is for seeing what the other side would give.

// Units: metres, radians, seconds. Joint order is the arm's (src/arm.h).

#ifndef SRC_MOTION_PLANNER_H_
#define SRC_MOTION_PLANNER_H_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/collision.h"
#include "src/joint_move.h"
#include "src/ompl_search.h"
#include "src/path_timing.h"
#include "src/rrt.h"

namespace cell {

// Which search finds the raw path of each plan (see "The search" at the top
// of this file).
enum class PlanSearch {
  kOwn,   // PlanRrtConnect of src/rrt.h.
  kOmpl,  // PlanOmpl of src/ompl_search.h.
};

// "own" or "ompl": the names the programs' --planner flag and the trial
// settings' text use.
const char* PlanSearchName(PlanSearch search);

// The search of that name, or nothing for any other text.
std::optional<PlanSearch> PlanSearchFromName(const std::string& name);

struct MotionPlanOptions {
  // ------------------------------------------------------------- margins
  // What the arm, tool and held module must keep from every obstacle while
  // really moving along the timed path: the checker's default margin, which
  // the scripted cycle keeps by 30 mm or more away from where it is meant
  // to touch.
  double required_margin = 0.02;  // m
  // ...and from each other. Zero, as the checker's default (src/collision.h:
  // the links' hulls are fatter than the links). The planner still plans
  // the self pairs with the budget below added, so the real motion keeps
  // at least this.
  double required_self_margin = 0.0;  // m
  // The budget, see the top of this file. edge_resolution is also the
  // largest gap between checked points along every edge.
  double edge_resolution = 0.03;      // m
  double corner_allowance = 0.005;    // m
  double tracking_allowance = 0.005;  // m

  // ----------------------------------------------------------- joint box
  // Each joint's range shrunk by this at both ends (start and goal may lie
  // in the strip; the box is widened to take them).
  double limit_inset = 0.035;  // rad, 2 degrees
  // Joints 4 and 6: the box is the span between start and goal plus this on
  // each side (within the shrunk range).
  double wrist_slack = 1.05;  // rad, 60 degrees

  // -------------------------------------------------------------- search
  // Which search plans each seed, and OMPL's own settings for when it is
  // OMPL: its planner by name, its range (by default `step` below) and the
  // collision queries one search may make in the first round. A planner
  // name OMPL's search does not know fails the plan with kBadInput.
  PlanSearch search = PlanSearch::kOwn;
  OmplSettings ompl;
  double step = 3.0;  // m of motion (weighted distance)
  int max_iterations = 2000;  // per seed, in the first round
  int seeds = 4;
  std::uint32_t base_seed = 1;
  // Rounds of seeds before giving up, each with twice the iterations.
  int rounds = 3;
  // Wall time for the whole plan, search and shortening, all rounds.
  double time_budget = 10.0;  // s
  // Also plan to the goal with joint 1 a whole turn the other way round,
  // and keep the shorter (see the top of this file).
  bool try_both_ways = false;

  // ---------------------------------------------------------- shortening
  int shorten_iterations = 200;
  // The kept path is resampled so that neighbours are at most this far
  // apart (m of motion); corners are kept.
  double resample_spacing = 1.0;  // m

  // -------------------------------------------------------------- timing
  // Fraction of full speed for PlanArmMove's trajectory, as
  // MoveOptions::speed (src/arm_follower.h; the cycle carries at 0.9).
  double speed = 0.9;

  // ----------------------------------------------------- measuring after
  // The smallest clearance along the kept path is measured with
  // CollisionChecker::Check at samples this far apart (m of motion). Only a
  // report: the soundness comes from the margins, not from this.
  double clearance_spacing = 0.03;  // m
};

enum class MotionPlanStatus {
  kFound,
  kBadInput,          // sizes, non-finite values, options out of range
  kStartOutOfRange,   // outside the arm's joint ranges
  kGoalOutOfRange,
  kStartNotClear,     // closer than the planning margin to something
  kGoalNotClear,
  kNotFound,          // no seed of any round found a path
  kTimeBudget,        // the time budget ran out first
};

const char* ToString(MotionPlanStatus status);

// How a plan went, found or not.
struct MotionPlanReport {
  MotionPlanStatus status = MotionPlanStatus::kBadInput;
  // In words, with the numbers: why it failed, or empty.
  std::string reason;

  // The setup: the box, the weights, the margins planned with, and the
  // corner deviation the path may be timed with (rad), and the per-joint
  // tracking error the tracking allowance stands for (rad).
  Eigen::VectorXd lower, upper, weights;
  double planning_margin = 0.0;
  double planning_self_margin = 0.0;
  double max_deviation = 0.0;
  double tracking_bound = 0.0;

  // The work. Search: the RRT runs of every seed and round; shortening:
  // the ShortenPath runs of every path found; check: measuring the
  // clearance of the kept path afterwards.
  double search_seconds = 0.0;
  double shorten_seconds = 0.0;
  double check_seconds = 0.0;
  std::int64_t search_queries = 0;
  std::int64_t shorten_queries = 0;
  std::int64_t check_queries = 0;
  int plans_tried = 0;
  int plans_found = 0;
  int rounds = 0;
  RrtStatus last_rrt_status = RrtStatus::kBadInput;  // of the last failing run

  // The kept path: its seed, the goal it ends at (the one given, or with
  // try_both_ways perhaps joint 1 a turn round), its length before and after
  // shortening (m of motion), its waypoints after resampling.
  std::uint32_t seed = 0;
  Eigen::VectorXd goal;
  double raw_length = 0.0;
  double length = 0.0;
  int waypoints = 0;
  // Measured along it afterwards, at the checker's own margins and allowed
  // contacts: the smallest distance to an obstacle, and among the arm and
  // module themselves, and between what.
  double clearance = 0.0;
  std::string clearance_between;
  double self_clearance = 0.0;
  std::string self_clearance_between;
};

// A collision-free joint path from `start` to `goal`, both included, for the
// checker's scene and held module as they are now: shortened, resampled, and
// sound for timing with corners rounded by report->max_deviation (see the
// top of this file). Returns nothing, with the reason in `report` if it is
// not null, if the input is bad, start or goal is out of range or closer to
// something than the planning margin, or no path was found within the
// rounds and the time budget.
//
// Changes the checker's margins while planning and puts them back; the
// queries it makes count in checker.queries().
std::optional<JointPath> PlanMove(CollisionChecker& checker,
                                  const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal,
                                  const MotionPlanOptions& options = {},
                                  MotionPlanReport* report = nullptr);

// The same through each of `vias` in turn: one PlanMove per leg, joined.
// Each leg is shortened on its own, so the path really passes through every
// via (shortening the whole would cut them out). For forcing a way round
// that the planner would not take by itself. The report sums the legs'
// work; its path figures are of the whole path, its clearance the smallest
// of any leg, its seed the first leg's.
std::optional<JointPath> PlanThrough(CollisionChecker& checker,
                                     const Eigen::VectorXd& start,
                                     const std::vector<Eigen::VectorXd>& vias,
                                     const Eigen::VectorXd& goal,
                                     const MotionPlanOptions& options = {},
                                     MotionPlanReport* report = nullptr);

// Every configuration along the path's segments, no two neighbours more
// than `spacing` apart in the checker's sweep measure (sum_i w_i |dq_i|,
// metres): for checking a path again more finely than it was planned.
JointPath DenseSamples(const CollisionChecker& checker, const JointPath& path,
                       double spacing);

// ------------------------------------------------- the one call for the cell

// What the cell knows when it plans a move: where the obstacles are and
// what the tool holds. Applied to the checker in this order.
struct PlanScene {
  // Copy every obstacle from this state first (CollisionChecker::SetScene).
  // The simulator's truth: right for what is fixed by design (the conveyor's
  // bed, the camera's post) and for tests. Null: start from what the checker
  // last had.
  const mjData* world = nullptr;
  // Then the estimates: the pallet's pose, and the modules' poses (module
  // index and centre pose, as Cell::TrueModulePose) that the camera
  // measured or that the cell knows it put on the tube.
  std::optional<Eigen::Isometry3d> pallet;
  std::vector<std::pair<int, Eigen::Isometry3d>> modules;
  // The conveyor's slide, from its encoder (Conveyor::SensorPosition).
  std::optional<double> tube_slide;
  // The module on the tool, if any, and where it is in the tool tip's frame
  // (Suction::HeldPoseInTip; by design, the grip pose). Nothing held clears
  // the checker's held module.
  std::optional<int> held_module;
  Eigen::Isometry3d module_in_tip = Eigen::Isometry3d::Identity();
  // Contacts that do not count during the move (body id pairs). A carry or
  // a return allows none; this replaces whatever the checker had.
  std::vector<std::pair<int, int>> allowed_contacts;
};

// A planned move and its timing.
struct TimedMove {
  JointPath path;
  Trajectory trajectory;
};

// Plans from `start` (the arm's commanded configuration: ArmFollower::Goal)
// to `goal` in `scene`, and times the path within `limits` (Arm::limits) at
// options.speed, corners rounded by the deviation the margin budget allows.
// The trajectory starts exactly at `start`, so ArmFollower::Follow takes it.
// Returns nothing, with the reason in `report`, as PlanMove does, or if the
// scene names a module the checker does not have, or the timing fails.
std::optional<TimedMove> PlanArmMove(CollisionChecker& checker,
                                     const PlanScene& scene,
                                     const Eigen::VectorXd& start,
                                     const Eigen::VectorXd& goal,
                                     const JointLimits& limits,
                                     const MotionPlanOptions& options = {},
                                     MotionPlanReport* report = nullptr);

}  // namespace cell

#endif  // SRC_MOTION_PLANNER_H_
