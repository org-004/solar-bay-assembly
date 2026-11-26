// One whole bay of a layout as planned motion: every move of every arm
// planned free of collision, all arms on one clock, and the test of both.
//
// PlanLayout takes a layout that passes the checker (src/layout_check.h),
// plans every move of every arm for one bay (or several back to back, as
// below) in the order of the bay's steps, times it, and schedules the arms
// so that they work at the same time without meeting. The result is a Plan: for every arm a row of segments
// that covers the bay's time without a gap. //:plan_layout prints it and
// writes it as plans/<name>.json; VerifyPlan checks a plan without planning
// anything, and //:plan_replay_test runs it on the committed plan of the
// chosen layout; PlanPlayer says where everything is at an instant, for the
// verifier and for the viewer's playback, so what is watched is what was
// checked. Kinematic only: nothing here steps the physics.
//
// What moves, what is there, what touches and what is near by design, and
// the one distance function are src/layout_scene.h's. The poses are the
// checker's (CheckReport::points): the work pose of every work point, and
// the samples of its approach and retreat legs.
//
// A configuration (Config) of an arm is its six joints in radians and then
// its track's value in metres (0 for an arm without a track).
//
// The segments of an arm, in time order, each from t0 to t1, t1 of one being
// t0 of the next, the first starting at 0 and the last ending when the bay's
// time ends:
//   wait      the arm stands still at `path[0]`, which is its wait pose or
//             the retreat pose it last came to; `reason` says what it waits
//             for;
//   move      a planned move from path.front() to path.back(): the waypoints
//             found by RRT-Connect and shortening (src/rrt.h), timed by
//             Trajectory::Plan (src/path_timing.h) within the arm's limits,
//             then settle_s at rest at the end. It starts at the arm's wait
//             pose, a retreat pose or the tool-change pose and ends at the
//             approach pose of the step it leads to, at the tool-change pose,
//             or at the wait pose;
//   approach  the straight move of the tool's working point from the
//             approach pose to the work pose: `path` holds the checker's
//             samples of the leg, far pose first, work pose last;
//   work      the arm stands at the work pose (or the tool-change pose) for
//             the layout's process time (or tool_change_s);
//   retreat   the straight move from the work pose to the retreat pose:
//             `path` holds the leg's samples, work pose first.
// Two steps of one arm that follow each other at the same work pose (locate
// and pin at one hole) have no retreat, move or approach between them.
// Parking: after its last step of phase "rails" an arm that follows another
// (bolts, pins) and has steps in a later phase moves to its wait pose as
// soon as that step has ended. The move belongs to the arm's next step,
// which is then preceded by two moves, retreat -> wait and wait -> its goal
// (for the arms of these layouts, the tool-change pose), with the arm's
// wait segment between them. The module arm does not park. After the last
// tool change of the bay, which puts the arm's first tool back, the arm
// moves from the tool-change pose to its wait pose: that step's segments
// are move, work, move.
//
// What an arm has and holds. `tool` is on the flange during the whole
// segment and `held` is in it during the whole segment. Things change only
// at the END of a work segment: at a take the part leaves its pallet or
// stock and is held from then on; at a set the part is let go and lies in
// the bay from then on; at a tool change the other tool is on the flange
// from then on.
//
// Bays back to back (src/layout_bays.h; PlanOptions::bays, 1 by default,
// 3 for //:plan_layout). Each bay is planned as one is, from t = 0, and the
// plan holds them on one clock. With N steps in a bay (the checker's), step
// n of bay b has the number n + bN in segments, events and steps; events,
// steps and phases carry their `bay`. Bay b takes pallet points 9b .. 9b+8
// (the checker solves all 36) in the layout's order of modules: one pallet
// of 36, 27 used by three bays. The rails come from the stock's 40 at its
// one pick pose, 10 a bay, 30 used, with no refill. The pin magazines are
// refilled for each bay through the port item:pin_feeder, so every bay
// takes its magazines at bay 0's pins_take points. A move of bay b is
// planned in bay 0's bay at that step with 9b fewer modules on the pallets.
// The bay's own events (BayEvent): tube_in and clamp before any arm of the
// bay moves (but for R10's early steps, below), release and bay_out after
// the last. During tube_in and bay_out every other arm stands at its wait
// pose, so the `ways` rows of the checker, which sweep the tube and the bay
// past the arms at their wait poses, hold for the schedule as they are.
// Between bay b and bay b+1:
//   - bay b+1's tube_in does not start before the holder is empty, when bay
//     b has been lifted out of the saddles: the end of the first (lift) leg
//     of the layout's bay_out.path;
//   - where the ports of tubes_in and bays_out share their zone or unit
//     (shares_zone_with names the other, or both hand over one item), bay
//     b+1's tube_in starts no earlier than bay b's bay_out ends;
//   - where they do not, the next tube may travel its tube_in path while
//     the bay travels bay_out, provided the tube, the bay and every arm stay
//     at least clearance_m apart at every sample: the arms stand at their
//     wait poses, where the checker's ways rows hold, and the tube is held
//     apart from the bay every sample_s;
//   - no arm starts a step of bay b+1 before bay b's bay_out has ended, and
//     a step that touches the tube or the bay not before bay b+1's clamp
//     has ended. R10: a step that touches neither (a take from a stock or
//     the pallet, a magazine take, a tool change; src/layout_bays.h,
//     TouchesTheBay) may start from bay b+1's tube_in on (not during bay
//     b's bay_out), and its arm then waits where the rules above allow (a
//     rail held where its take's retreat ended) for the clamp. While the
//     tube comes in, the schedule holds such an arm clearance_m +
//     plan_pad_m from the tube where it is on its way, at every sample, and
//     the verifier measures it so (PlanOptions::early_steps, on by default;
//     bay 0 starts every arm after its clamp, as before).
// The verifier holds all four (src/layout_bays.h, BetweenBays, and its own
// check of each bay's events: an arm off its wait pose during a tube_in or
// bay_out is measured against the tube or the bay where it is on its way;
// at its wait pose the checker's ways rows hold it); it checks each bay's events, order of work
// and phases on their own, and a later bay's moves and design pairs as bay
// 0's at the same step, with the pallet counting on.
//
// The planning world of a move (R16): the moving set is the arm's links,
// carriage, the tool on its flange and the part it holds (LayoutScene::
// Moving); the obstacles are LayoutScene::Obstacles for the move's `built`,
// with no pair by design, and the other arms where they wait (links,
// carriages, first tools on the flange, beams and waiting tools), so that
// no move passes through an arm that stands at its wait pose. A
// configuration is free when the least distance is at least clearance_m;
// the planner plans at clearance_m + plan_pad_m, and the schedule holds the
// arms that far apart, so that what is verified at clearance_m has room.
// `built` is the bay as it is when the move's step begins by the checker's
// rule and, for an arm that follows another (bolts, pins), also every part
// the leading arm may set before the move ends: all rails for a move of the
// rail phase, all rails and all nine modules for a move of the module phase.
// More parts are more obstacles, so a move free in its world is free against
// the bay at any earlier state of it.
//
// The schedule (R21, R22). Each arm does its steps in the order of the bay's
// steps. A strap bolt's step starts only after its rail's set has ended; a
// hole's step (locate, pin, swage) only after its module's set has ended and
// after the step before it in the module's order of holes has ended; the
// first module is taken only when every strap bolt is driven. Just in
// time: a step of phase modules of an arm that is neither a take nor a set
// (its tool change, the take of its pin magazine) starts only when what
// that arm's next hole step waits for has happened, so that the arm does
// not stand over the lane before its module is set. Release follows the
// last step of the module phase and bay_out starts when release has ended
// and every arm stands at its wait pose with its first tool. An arm whose
// next step may not start yet waits where it is, and the wait is a segment
// with its reason. An arm waits only at its wait pose, where a retreat of
// its own has just ended, or at the tool-change pose where it has just
// changed its tool (the verifier holds all three).
//
// The order of work (the layout's optional field "order", src/layout.h
// WorkOrder, run 018). What is said above is the order without it; each
// member changes one rule, and the verifier (src/layout_replay.cc) holds
// the same:
//   modules        the modules are set in this order: the k-th module taken
//                  is pallet module k (the nearest left), and it is set as
//                  bay module modules[k]; its holes follow its set as
//                  above. The bay's steps (src/layout_check.h, BaySteps),
//                  and so each arm's order, follow it; what is built at
//                  each work point follows it (src/layout_scene.h).
//   modules_start  "after_all_bolts": the first module is taken only when
//                  every strap bolt is driven (as above);
//                  "after_their_rails": the take of module m starts only
//                  when the strap bolts of rails m and m+1, the two it lies
//                  on, are driven, so that modules start while the other
//                  rails are still being bolted. The phase "rails" still
//                  ends with the last bolt and "modules" is counted from
//                  there, so with overlap the rails' phase holds the first
//                  sets.
//   pin_tools      "just_in_time" as above; "early": the rule of just in
//                  time is not applied, so a pin arm changes to its pin
//                  tool and takes its magazine as soon as its bolts are
//                  driven, and then waits for its first module where the
//                  magazine take's retreat left it (or, if another arm is in
//                  its way, at the tool-change pose).
// Which pin arm works which hole is the layout's `work` list, as before.
// At every instant, sampled every sample_s, every arm with its carriage, the tool on its flange, its waiting tools and what
// it holds is at least clearance_m from every other arm with the same, and
// from the bay, the pallets and the stocks as they are at that instant;
// while an arm is on a leg or at work, the pairs that touch or are near by
// design at that work point (LayoutScene::AtWorkPoint) are held as the
// checker's approach rows hold them: never into each other. Where two arms
// would come too near, the one that is scheduled later waits and the wait's
// reason names the other arm. The blocks are placed one at a time: of the
// arms whose next block may start, the one that can start earliest (ties by
// the layout's order of arms), put off by 0.25 s at a time while another
// arm is in its way. Ready blocks are tried in order of their earliest
// start; one that meets an arm standing still for good, or that 2,000
// delays of 0.25 s do not clear, gives way to the next, and the schedule
// is blocked only when no ready block can start, unless one can be given
// way: the first ready block that met an arm standing for good at the end
// of that arm's last block has the schedule taken back to before that
// block, which is placed again only after it (once for each pair of
// blocks; the first bay of the chosen layout never needs it, its later
// bays do). No evasive move is made. The verifier needs no check of giving
// way: it decides only the order in which blocks are placed, and what is
// placed holds every rule above. A block is checked at instants sample_s
// apart from its start and at the plan clock's k sample_s, the verifier's
// instants (for a later bay, the instants its arms will have once joined:
// PlanBays tells the planner where they start).
//
// Repeatable (R18). Every move has a seed drawn through
// src/portable_random.h from PlanFigures::seed and the move's ordinal, and
// iteration budgets only (no time limit), so a move's result depends on
// nothing but the layout and the figures. Moves are planned on several
// threads and put together in order: the plan does not depend on the number
// of threads. A move not found with one budget is tried with the next.
//
// The same on every platform. A plan is made on one machine and replayed on
// others: VerifyPlan compares poses through forward kinematics with
// tolerances and recomputes timings with tolerances; it never compares a
// freshly planned path with the stored one.

#ifndef SRC_LAYOUT_PLAN_H_
#define SRC_LAYOUT_PLAN_H_

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "src/joint_move.h"
#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_scene.h"

namespace cell::layout {

// Six joints (rad), then the track's value (m).
using Config = std::array<double, 7>;

// The figures a plan was made with. All but the margins are ours.
struct PlanFigures {
  double clearance_m = 0.010;        // The checker's, for this layout.
  double joint_margin_deg = 5.0;     // The checker's, for this layout.
  // Moves are planned, and the schedule's arms are held apart, at
  // clearance_m + plan_pad_m, because the schedule is sampled between the
  // planner's checked points and a shortened path runs along its obstacles
  // at exactly the clearance it was planned with (ours). The verifier keeps
  // clearance_m.
  double plan_pad_m = 0.003;
  double to_full_speed_s = 0.3;      // A joint reaches its datasheet speed in this.
  double jerk_rise_s = 0.1;          // Jerk builds the acceleration in this.
  double settle_s = 0.3;             // At rest after a planned move.
  double leg_speed = 0.25;           // m/s of the working point on a leg.
  double leg_accel = 1.0;            // m/s^2 of the working point on a leg.
  double max_deviation = 0.002;      // rad, Trajectory::Plan's blend limit.
  double edge_resolution = 0.010;    // m of weighted motion between checked points.
  int shorten_tries = 200;
  std::vector<int> budgets = {5000, 40000, 160000};  // RRT iterations, in turn.
  std::uint32_t seed = 1;            // Every move's seed is drawn from this.
  double sample_s = 0.05;            // The schedule is checked this often.
};

// How one arm moves: its limits for Trajectory::Plan and its weights for the
// planner's distance (metres a point of the arm can move for one radian of
// each joint; 1 for the track). Seven entries each, as a Config.
struct ArmMotion {
  JointLimits limits;                // velocity, acceleration, jerk.
  Eigen::VectorXd weights;
  Eigen::VectorXd lower, upper;      // Where a planner may put the arm.
};
// From the catalogue's datasheet speeds, the layout's track and the figures.
ArmMotion MotionOf(const LayoutScene& scene, int arm, const PlanFigures& f);

struct Segment {
  std::string kind;        // "wait", "move", "approach", "work", "retreat".
  int step = 0;            // The BayStep::number it belongs to; a wait belongs
                           // to the step it waits for.
  double t0 = 0.0, t1 = 0.0;
  std::string tool;        // On the flange during the segment ("" bare).
  std::string held;        // "module", "rail", "pin magazine" or "".
  std::vector<Config> path;  // See the opening comment for each kind.
  // A move only.
  std::string from, to;    // "wait", "tools", "<item> retreat", "<item> approach".
  bool found = false;
  int budget = 0;          // The budget that found it; 0: the straight line was free.
  std::int64_t queries = 0;  // Collision queries spent on it.
  double planned_s = 0.0;  // Its duration without the settle.
  double model_s = 0.0;    // The time model's figure for the same two poses.
  std::uint32_t seed = 0;
  Built built;             // The bay it was planned in.
  // A wait only.
  std::string reason;
};

struct ArmPlan {
  std::string name;
  std::vector<Segment> segments;
};

struct BayEvent {
  std::string kind;        // "tube_in", "clamp", "release", "bay_out".
  int step = 0;
  double t0 = 0.0, t1 = 0.0;
  int bay = 0;             // Which of the plan's bays (src/layout_bays.h).
};

// When a step of the bay starts (its move, or its approach or work where it
// has no move) and ends (its retreat, or its work where it has none).
struct StepTime {
  int number = 0;
  std::string what;
  std::string arm;
  double start = 0.0, end = 0.0;
  int bay = 0;
};

struct PhaseTime {
  std::string name;        // "tube in", "clamp", "rails", "modules", "end".
  double planned_s = 0.0;
  double model_s = 0.0;
  int bay = 0;             // One set of phases for each bay.
};

struct Plan {
  std::string layout;          // The layout's name.
  std::string layout_sha256;   // Of the layout's file as it was planned.
  PlanFigures figures;
  std::vector<ArmPlan> arms;   // In the layout's order.
  std::vector<BayEvent> events;  // Bay by bay.
  std::vector<StepTime> steps; // Every step of each bay, in the steps' order.
  std::vector<PhaseTime> phases;
  int bays = 1;                // Bays planned back to back (src/layout_bays.h).
  double bay_planned_s = 0.0;  // Bay 0, from its tube_in to its bay_out.
  double bay_model_s = 0.0;
  bool complete = false;       // Every move found and the schedule exists.
};

// ----------------------------------------------------------------- motion
// (src/layout_replay.cc)

// How long a planned move takes without its settle: Trajectory::Plan's
// duration through the waypoints. Negative if the path cannot be timed.
double MoveDuration(const std::vector<Config>& path, const ArmMotion& motion,
                    const PlanFigures& f);
// How long a leg takes. The arm runs through the leg's samples by straight
// joint-space pieces, at the fraction s(u) = 3u^2 - 2u^3 of the path's
// joint-space length, u = (t - t0) / T. T is the largest of
// 1.5 * length_m / leg_speed (the working point's peak speed is leg_speed;
// `length_m` is the rule's distance of the leg), sqrt(6 * length_m /
// leg_accel) (its peak acceleration is leg_accel) and what keeps every joint
// within its speed on every piece. A leg of zero length takes 0.
double LegDuration(const std::vector<Config>& path, double length_m,
                   const ArmMotion& motion, const PlanFigures& f);
// Where the arm is at time t of a segment (t0 <= t <= t1; clamped).
Config ConfigAt(const Segment& segment, const ArmMotion& motion,
                const PlanFigures& f, double t);

// --------------------------------------------------------------- planning
// (src/layout_plan.cc)

// One move's world: R16's test of a configuration.
class MoveWorld {
 public:
  MoveWorld(const LayoutScene& scene, int arm, const std::string& tool,
            const std::string& held, const Built& built, double clearance_m);
  // The least distance at the configuration, out to `far`.
  Nearest Least(mjData* d, const Config& c, double far) const;
  bool Free(mjData* d, const Config& c) const;

 private:
  const LayoutScene& scene_;
  int arm_;
  double clearance_m_;
  std::vector<MovingGeom> moving_;
  std::vector<Obstacle> obstacles_;
};

// One row of //:plan_layout's table.
struct MoveRow {
  int step = 0;
  std::string arm, from, to;
  bool found = false;
  int budget = 0;
  int waypoints = 0;
  std::int64_t queries = 0;
  double planned_s = 0.0;
  double model_s = 0.0;
  std::string note;        // Why it was not found.
};

struct PlanOptions {
  int threads = 3;
  // Only the steps numbered first..last are planned and scheduled (0, 0:
  // the whole bay): for trying a part of the bay. A part is never complete.
  int first_step = 0, last_step = 0;
  // The RRT iteration budgets, in turn; empty: PlanFigures' own. The
  // budgets used go into the plan's figures.
  std::vector<int> budgets;
  // Bays planned back to back (src/layout_bays.h); a part is one bay.
  int bays = 1;
  // R10: in bays after the first, steps that touch neither the tube nor the
  // bay may start before the clamp has ended (src/layout_bays.h).
  bool early_steps = true;
};

// PlanResult and PlanLayout are declared at the end, after VerifyReport.

// ------------------------------------------------------------- plan file
// (src/layout_plan_file.cc)

// The plan as text, the same for the same plan on one machine, and back.
std::string PlanToJson(const Plan& plan);
std::optional<Plan> PlanFromJson(const std::string& text, std::string* error);
// Lower-case hexadecimal SHA-256 of a file's bytes; empty if it cannot be read.
std::string Sha256OfFile(const std::string& path);

// ----------------------------------------------------------------- replay
// (src/layout_replay.cc)

// Where one arm is at an instant.
struct ArmAt {
  Config config{};
  std::string tool, held;
  int segment = -1;        // Index into its segments.
  std::string kind;        // The segment's kind.
  int step = 0;
};

// The cell at an instant of a plan.
struct WorldAt {
  double t = 0.0;
  std::vector<ArmAt> arms;     // In the layout's order.
  bool tube = false;           // A tube lies in the holder or is on its way.
  // The offset of the tube (coming in) or of the bay (going out) from its
  // place in the holder; zero when it lies in the holder.
  Eigen::Vector3d way_offset = Eigen::Vector3d::Zero();
  std::string event;           // "tube_in", "clamp", "release", "bay_out" or "".
  // The bay in the holder or going out: the first whose bay_out has not
  // ended (the last once all are out). What follows is that bay's; the
  // pallet counts on over the bays.
  int bay = 0;
  std::vector<int> rails;      // On the tube.
  std::vector<int> modules;    // On their rails.
  std::vector<int> pallet_taken;  // module_take points already taken (held or set).
  int magazines_taken = 0;
};

// Plays a plan: where everything is at an instant.
class PlanPlayer {
 public:
  // The report is the checker's for the layout (its steps and points, of
  // one bay); a plan of several bays is played with StepsOfBays.
  PlanPlayer(const LayoutScene& scene, const CheckReport& report,
             const Plan& plan);
  double duration() const;
  // A step of the plan by its number in the plan, nullptr if none.
  const BayStep* Step(int number) const;
  WorldAt At(double t) const;
  // Puts every arm's pose of `w` into the data (no kinematics).
  void SetPoses(const WorldAt& w, mjData* d) const;
  // The bay as built at the instant, for LayoutScene::Obstacles.
  Built BuiltAt(const WorldAt& w) const;

 private:
  const LayoutScene& scene_;
  const CheckReport& report_;
  const Plan& plan_;
  std::vector<ArmMotion> motions_;
  // What the constructor works out once: one trajectory per move, the
  // instants at which parts are taken and set, the steps by number. Never
  // changed after construction, so several threads may play one player.
  struct Cache;
  std::shared_ptr<const Cache> cache_;
};

// One measured distance at an instant.
struct Closest {
  double distance = kSceneFar;
  double t = 0.0;
  int step = 0;
  std::string arm;         // The arm whose moving set it is.
  std::string a, b;        // The two things (geom names).
  std::string other_arm;   // Set when b belongs to another arm.
  bool by_design = false;  // A pair near by design at that work point.
};

// The least distances of the cell at one instant: for every arm against the
// bay, the base, the holder, the pallets, the stocks and the items as they
// are at the instant, and for every pair of arms, each arm with its
// carriage, the tool on its flange, its waiting tools and what it holds.
// Pairs that touch by design are not measured; pairs near by design at the
// arm's work point are returned with by_design set. `adjust`, when given, is
// called after the poses of `w` are set and their kinematics done, before
// anything is measured: a run in physics (src/physics_run.h) moves the
// geoms of a part to where the physics has it there, so that what it
// measures is measured as the verifier measures.
std::vector<Closest> InstantDistances(
    const LayoutScene& scene, const CheckReport& report,
    const PlanPlayer& player, const WorldAt& w, mjData* d,
    const std::function<void(mjData*)>& adjust = {});

struct VerifyOptions {
  bool edges = true;       // Every edge of every move, by R16's test.
  bool schedule = true;    // R21 and R22 along the schedule.
  double tolerance_m = 1e-4;       // On distances, for another platform.
  double pose_tolerance_m = 5e-4;  // On a pose's working point.
  double pose_tolerance_deg = 0.2;
  double time_tolerance_s = 1e-3;
  int keep = 20;           // How many least distances to keep.
  // The schedule's samples are taken from from_s to to_s only (to_s < 0:
  // to the end), for a test that looks at one stretch of time.
  double from_s = 0.0, to_s = -1.0;
};

struct VerifyReport {
  bool ok = false;
  std::vector<std::string> faults;      // Each says what, where and when.
  // Least distances over the whole schedule between things not near by
  // design, tightest first, one for each pair of things (the instant it is
  // least); `keep` of them.
  std::vector<Closest> least;
  // The least distance of each pair of arms.
  std::vector<Closest> arm_pairs;
  double least_clearance_m = kSceneFar;   // least.front().distance.
  double least_arm_to_arm_m = kSceneFar;
  // The least joint margins over the schedule: degrees from a datasheet
  // limit, with the arm, the joint, the step and the time; five of them.
  struct JointMargin {
    double margin_deg = 0.0;
    std::string arm, joint;
    int step = 0;
    double t = 0.0;
  };
  std::vector<JointMargin> joint_margins;
  double least_joint_margin_deg = 0.0;
  int waits = 0;
  double wait_s = 0.0;
  std::int64_t queries = 0;
};

// Checks a plan against its layout without planning anything: that it was
// made for this layout's file (`layout_sha256`: of the file as it is now),
// that the segments of every arm cover the time without a gap and in the
// order of the bay's steps, that every move starts and ends at the poses the
// steps give, that every edge of every move is free by R16's test at the
// plan's edge resolution, that every timing is what its figures give, that
// the order of R21 holds and that the clearances of R22 hold at every
// sample.
VerifyReport VerifyPlan(const Layout& layout, const ModelPaths& paths,
                        const Plan& plan, const std::string& layout_sha256,
                        const VerifyOptions& options = {});

// --------------------------------------------------------------- planning
// (src/layout_plan.cc)

struct PlanResult {
  Plan plan;
  std::vector<MoveRow> rows;   // Every move, in the order of the steps.
  std::int64_t queries = 0;    // All collision queries of planning.
  double seconds = 0.0;        // Wall time of planning (steady clock).
  std::vector<std::string> faults;  // Why the plan is not complete.
  VerifyReport verify;         // VerifyPlan's report on the plan, when run.
};

// Plans and schedules one bay, or options.bays back to back. `layout_sha256`
// goes into the plan. The figures' margins are the checker's for the layout.
// The plan is complete when every move is found, every bay is scheduled
// and the verifier finds no fault.
PlanResult PlanLayout(const Layout& layout, const ModelPaths& paths,
                      const std::string& layout_sha256,
                      const PlanOptions& options = {});

// Why the moves that lead to the given steps are (or are not) found, as
// plain lines (//:plan_layout --why): for each move its arm, tool and held
// part, its ends and their joint differences, the least distance and pair
// at each end, where the straight line is first blocked marched from each
// end, and, when it is not found, each budget's RRT report. Plans no other
// move. Uses the options' budgets.
std::vector<std::string> WhyMoves(const Layout& layout, const ModelPaths& paths,
                                  const std::vector<int>& steps,
                                  const PlanOptions& options = {});

}  // namespace cell::layout

#endif  // SRC_LAYOUT_PLAN_H_
