// The checks on a layout.
//
// CheckLayout takes a layout (src/layout.h), builds its model
// (src/layout_model.h) and checks one whole bay. It returns one row for each
// check with the figure and whether it passes, the steps of building one
// bay with their times, and the poses it solved (SolvedPoint).
// //:check_layout prints them and exits non-zero when a row fails;
// CheckOptions::checks (--checks a,b,c) makes only the kinds asked for, and
// the report passes by them alone. The kinds of check, the `check` of a row,
// in the order of the output (CheckKinds):
//
//   transport  on the container: everything that travels inside is, in the
//              stowed state, within the container's inside envelope, and
//              its mass is within what is left for the cell
//              (src/container.h); on the flat rack: the rows of
//              src/layout_rows.h in their place;
//   hold       the holder grips only stretches the document allows, not the
//              two ends alone, and is built for the moment the document asks;
//   load       what each arm carries, tool and part, is within its payload;
//   reach      every point an arm must work at is reached by the arm the
//              layout gives it, the tool along the direction the document
//              gives, inside the datasheet's joint ranges less a margin;
//   clearance  at each of those poses the arm, its tool and what it carries
//              keep a stated distance from what is there at that step; and
//              the free spaces of the tools and of the locating move are
//              empty;
//   ways       the space a tube sweeps coming in and a bay sweeps leaving is
//              empty;
//   time       the time one bay takes, from the steps, by a stated model of
//              the arms' moves; it has no limit: its rows pass when every
//              step has a pose;
//   allowed, port
//              the rows of src/layout_rows.h, which need no pose;
//   approach   every work point is come to and left along its kind's rule
//              (below).
//
// What moves, what is there at a step, what touches by design and what is
// near by design are defined in src/layout_scene.h, once, for the checker
// and for every program that plans motion in a layout; so are the bay as
// built at each work point and the one distance function (LeastDistance,
// mj_geomDistance after a bounding-box test, capped at 1 m: a figure of
// 1.000 means nothing is within a metre). The checker loads the layout's
// model once in the `everything` state through LayoutScene, and once
// `stowed` for the container's transport rows. The other arms stand at their
// wait poses with their first tools.
//
// The work points, in the bay's frame of src/bay_hardware.h moved by the
// layout's "tube" (BayPoint), with the tool's working point at the point and
// its z axis along the direction given (WorkPoints):
//   rail_set k     the centre of rail k's top face; z down; x along the
//                  bay's X, either sign;
//   strap_bolt k   5 mm short of point a of rail k's fastening along the
//                  tool's travel; z along -X; any spin;
//   locate, pin, swage, index 4 m + i (item "pin m/i")
//                  5 mm below the rail's site (point a) of module m's
//                  fastening i; z up; any spin; the three kinds share one
//                  pose when one arm and tool do them and their rules of
//                  approach are the same;
//   module_set m   the centre of module m's glass top; z down; x along X,
//                  either sign;
//   module_take j  over all pallets in order (src/layout.h, Pallets): module
//                  j's place on its pallet (src/layout.h defines the
//                  pallet's frame): standing "on_long_edge", the centre of
//                  its glass face, which faces the pallet's -y, the tool's z
//                  along the pallet's +y (level, onto the glass); lying
//                  "flat", the centre of the top module's glass, z down;
//                  either way the tool's x along the pallet's x, either
//                  sign. Module m of the bay is take j = m; the other takes
//                  are checked for reach, clearance and approach only. The
//                  turn from upright to flat is part of the move from the
//                  take pose to the set-down pose;
//   rail_take k    the pick point of the stock "rails", z along its pick
//                  axis, any spin;
//   pins_take i    the pick point of the i-th stock of pins (PinStocks).
// Who does each point is the layout's division of the kind's points
// (FindWork(layout, kind, index)).
//
// The margins. On the flat rack clearance_m and joint_margin_deg are the
// larger of CheckOptions' and the layout's "margins"; on the container
// CheckOptions' alone; //:check_layout prints the values used as a # line.
// A joint is kept joint_margin_deg inside each end of the datasheet's range
// of src/arm_catalog.h (and inside the model's own range where that is
// tighter); a reach row's figure is the least distance, in degrees, of any
// joint from its datasheet limit, and the row names the joint. A pose is
// reached when the working point is within 0.1 mm of the point and the
// tool's axes within 0.06 degrees of their directions. A clearance row
// passes when the least distance is at least clearance_m; a free space row
// when it is more than zero; a ways row when the least distance over the
// sweep is at least clearance_m.
//
// The inverse kinematics is the checker's own: damped least squares over
// the arm's six joints with the carriage held at one track value, from the
// arm's previous solution, then its wait pose, then ik_seeds seeds drawn
// once from a fixed generator (src/portable_random.h), so every platform
// finds the same poses. Each joint is clamped into its range at every step.
// The track value is the layout's stop for the kind (one for every point or
// one per point of the kind, by the kind's point number); without one it is
// searched: the value that stands the base opposite the point, then 0.15,
// 0.30, 0.45, 0.60 m either side, clamped to the track. The candidates are
// tried in that order (track values, then seeds, then the two signs of x).
// Where the layout has no approach for the kind, the first pose that is
// reached and clear wins; where it has one, the first that is reached, clear
// and whose two legs both pass by the approach rows' rule wins, and if none
// does, the first reached and clear one is kept and the approach rows report
// its legs. If none is clear, the reached pose with the most clearance is
// reported. A stop outside the track's lo..hi fails the reach row of its
// point ("stop X outside the track's lo..hi"); a wait_track outside it fails
// the reach row "wait track <arm>", a stow_track the transport row "stow
// track <arm>" (figure: how far inside the range, negative outside, limit
// 0).
//
// The tool in the free space. The probe's cylinders say what else must keep
// out of the free space; the tool itself must fit in it too. One row for
// each arm and tool a fastening kind is given to, "tool in free space
// <kind> <tool>": every box of the tool that reaches into the length of the
// free space behind the working point (kStrapBoltFreeLength for
// strap_bolt; kPinToolLength for pin and swage; kLocateFreeLengthBelow for
// locate) must lie within its radius (kStrapBoltFreeRadius, kPinFreeRadius,
// kLocateFreeRadiusBelow). Figure: the radius less the box's farthest reach
// from the tool's axis (worst box), limit 0.
//
// The free spaces. For each strap bolt, pin and locating point the probe of
// LayoutHandles is put at point a of the fastening with its z along the
// tool's travel, and each cylinder (bolt; pin; locate below and above) must
// be at more than zero from everything there at that step, the active arm
// at its pose included, except the active arm's own tool and, since each
// cylinder starts at a sheet, the parts the fastening joins (rail k and the
// tube for a bolt; the rail and the module for a pin and the locating). Rows
// "free space bolt k", "free space pin m/i", "free space locate m/i" (the
// worse of its two cylinders).
//
// The ways. The tube alone along tube_in.path and the finished bay (tube,
// ten rails, nine modules) along bay_out.path, sampled every sweep_step_m
// or less along each segment, by moving the geoms' world positions in the
// model's data (exact for a translation), against everything there with
// every arm at its wait pose; the items whose `does` names the tube or the
// bay (the cart that carries it) are left out, and a holder geom that is
// within clearance_m of the moving set in its place is left out while the
// offset is straight up from the place (lifting out of a saddle is the
// design). A sample is the figure; between two samples the true least
// distance can be smaller by at most half a step for a thin obstacle.
//
// Transport on the container. The stowed model: every geom that is not the
// base's, at its stow pose, its world bounding box (exact corners for
// boxes, every vertex for meshes, the rotated local box otherwise) inside
// container::InsideEnvelope(), four rows (x, y, top, bottom), figure the
// worst geom's distance inside (negative: outside), and one mass row: the
// catalogue's arm masses, tracks, tools, holder and items that travel
// inside, against container::kCellMassBudget.
//
// Load. For each arm and each of its tools: the tool's mass and the
// heaviest part it holds in the layout's work (a module 32.4 kg, a rail
// with its strap 2.35 kg, a pin magazine 0.5 kg `ours`) against the
// catalogue's rated payload.
//
// Approach. For each kind of work that has points: without a rule
// (FindApproach) one failing row, item "<kind>", detail "the layout gives no
// approach for <kind>"; with one, two rows for every point, items "<item>
// approach" and "<item> retreat" (e.g. "module_set 3 approach"), unit m,
// limit clearance_m. The approach pose is the work pose with the tool's
// working point moved by approach_m along approach_dir (in the tool's frame
// as it is at the work pose, or the layout's frame), the orientation kept;
// the retreat pose likewise with retreat_dir and retreat_m. Each leg is
// walked from the work pose along the straight line in n equal steps of at
// most 5 mm, each sample solved by the inverse kinematics above from the
// sample before, the carriage at the work pose's track value, so the arm
// stays on the work pose's branch; every sample must be reached within the
// usual tolerances and inside the joint margin. What is there and what
// moves on each leg is src/layout_scene.h's (Leg): on the approach of a take
// nothing is held and the tool is near the part it is about to take; on the
// retreat of a set nothing is held and the part lies in the bay, the tool
// near it; otherwise the work point's own part, Builts and pairs. A leg
// passes when every sample is reached inside the joint margin; at the far
// pose (the last sample) every pair, the work point's pairs by design
// included, is at least clearance_m apart; and along the move (samples 1 to
// n-1) no other pair is nearer than clearance_m and no pair that touches or
// is near by design goes below zero. The figure is the least distance over
// the samples of a pair not by design, and at the far pose of every pair;
// when a pair by design goes below zero along the move its distance is the
// figure ("(near by design, but into each other)"), and when the far pose's
// nearest pair is one by design the detail adds "(by design at the work
// point)". The detail: "<pair> at <s> mm along" or "<pair> at the
// approach|retreat pose", then "least joint margin <deg> deg at <joint>;
// <n> samples over <length> mm". A sample not reached fails the row with
// "not reached at <s> of <length> mm: <mm> mm and <deg> deg off[; joint J
// held at its margin]" or "...: joint J inside its margin (<deg> deg)"; a
// point without a work pose fails both its rows with "no work pose: ...".
//
// The order of the steps of one bay (BaySteps): tube in, clamp; the rails
// and their bolts in the layout's rail_order (take rail k, set rail k, then
// the bolts, or each rail's bolt after it); then for each module m: take
// pallet module m and set it, then locate and pin its four holes in
// bay::kLocateOrder and swage them in bay::kSwageOrder; release, bay out,
// and each arm back to its wait pose and first tool. Each step is done by
// the arm the layout's division gives it; an arm takes its magazine (its
// pins_take points) before its first hole (an arm with a magazine and no
// hole takes it after module 0 is set); a tool change comes wherever an
// arm's next work needs another tool. Every arm starts at its wait pose
// with its first tool. The time model of a phase (the ten rails; the nine
// modules) is TimeModelText(): each unit has a leading piece (its take and
// set) and following pieces (its bolt; its holes), a following piece
// starting when the unit's leading pieces have ended and its arm has ended
// the pieces before it; with one leading and one following arm this is
// SideBySide, with one arm the plain sum. The time rows: "bay", whose detail
// gives the phases, and "busy <arm>" for each arm, the sum of its steps.
// The time row has no limit: it passes when every step it needs has a pose.

#ifndef SRC_LAYOUT_CHECK_H_
#define SRC_LAYOUT_CHECK_H_

#include <array>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "src/layout.h"
#include "src/layout_model.h"

namespace cell::layout {

struct CheckRow {
  std::string check;   // One of the kinds above.
  std::string item;    // What was checked: "rail_set 3", "stowed height".
  std::string arm;     // Empty where no arm is meant.
  std::string tool;
  double figure = 0.0;
  std::string unit;    // Of the figure: "deg", "m", "kg", "s".
  double limit = 0.0;  // What the figure is held against.
  bool pass = false;
  std::string detail;  // What limits it: the joint, the nearest thing.
};

// One step of building a bay.
struct BayStep {
  int number = 0;
  std::string what;    // "set rail 3"
  std::string arm;
  std::string tool;
  std::string part;
  std::string from;
  std::string to;
  double time_s = 0.0;
};

struct CheckOptions {
  // Kept from each end of a joint's range. For a layout on the flat rack the
  // larger of this and the layout's margins.joint_margin_deg is used.
  double joint_margin_deg = 5.0;
  // The least distance that counts as clear. For a layout on the flat rack
  // the larger of this and the layout's margins.clearance_m is used.
  double clearance_m = 0.010;
  // Which kinds of check to make (a row's `check`, one of CheckKinds());
  // empty: all of them. Reach, clearance, time and approach all need the
  // poses; asking for any of them solves the poses of the work kinds below.
  std::vector<std::string> checks;
  // Which kinds of work (src/layout.h's WorkKinds) the reach, clearance and
  // approach checks look at; empty: all. The time check needs all of them
  // and is left out when this is not empty.
  std::vector<std::string> work_kinds;
  // The ways' sweep is sampled at least this often.
  double sweep_step_m = 0.025;
  // Seeds of the inverse kinematics after the previous solution and the
  // wait pose.
  int ik_seeds = 16;
};

// The poses solved for one work point, for a planner: the work pose and,
// where the kind has an approach, the far ends of its two legs.
struct SolvedPoint {
  std::string kind;
  int index = 0;
  std::string item;           // "module_set 3"
  std::string arm;            // The arm the layout gives the point.
  std::string tool;
  // The part held on each leg and at the work pose: "module", "rail",
  // "pin magazine" or "" (src/layout_scene.h, Leg).
  std::string held_approach;
  std::string held_work;
  std::string held_retreat;
  double track = 0.0;         // The carriage's value, the same on both legs.
  std::array<double, 6> work_deg{};
  std::array<double, 6> approach_deg{};
  std::array<double, 6> retreat_deg{};
  bool work_reached = false;
  // Every sample of the leg reached inside the joint margin; set only when
  // the kind has an approach and the approach rows were made.
  bool approach_reached = false;
  bool retreat_reached = false;
};

struct CheckReport {
  std::vector<CheckRow> rows;
  std::vector<BayStep> steps;
  double bay_time_s = 0.0;
  std::string time_model;  // The model of a move, in words, for the output.
  double clearance_m = 0.0;       // The margins used.
  double joint_margin_deg = 0.0;
  // Every work point whose pose was looked for, kinds in WorkKinds() order,
  // points in index order. Empty when no check that needs poses was made.
  std::vector<SolvedPoint> points;
  bool pass = false;       // Every row passes.
  std::string error;       // Set when the model could not be built.
};

// The kinds of check, in the order of the output.
const std::vector<std::string>& CheckKinds();

CheckReport CheckLayout(const Layout& layout, const ModelPaths& paths,
                        const CheckOptions& options = {});

// The state of the cell with an arm at one of its work points, for a
// picture: the bay as built up to that point, the tool on the flange, the
// part in it, the arm at the pose the reach check found. `kind` is a kind of
// work of src/layout.h and `index` the point's number. Returns false and
// sets *error when the point is not reached.
bool StateAt(const Layout& layout, const ModelPaths& paths,
             const std::string& kind, int index, LayoutState* state,
             std::string* error);

// One point an arm must work at, in the layout's frame.
struct WorkPoint {
  std::string kind;
  int index = 0;
  std::string item;          // "pin 4/2"
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();  // The tool's working point.
  Eigen::Vector3d z = Eigen::Vector3d::UnitZ();   // The tool's z axis.
  bool has_x = false;        // The tool's x axis is along `x`, either sign.
  Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  int rail = -1;             // The rail worked on, if any.
  int module = -1;           // The module worked on, if any.
  int hole = -1;             // The hole of the module, 0..3.
  int pallet = -1;           // module_take: the pallet place.
  int pallet_modules = -1;   // module_take: modules on its pallet, j's included.
};

// The points of one kind of work, in index order. Empty for a kind that
// needs a stock the layout does not have.
std::vector<WorkPoint> WorkPoints(const Layout& layout,
                                  const std::string& kind);

// The model of a move (see TimeModelText). Joint changes in degrees.
double JointMoveTime(const std::array<double, 6>& change_deg,
                     const std::array<double, 6>& speed_deg_s);
double TrackMoveTime(double distance, double speed, double accel);
inline constexpr double kAccelFactor = 1.5;
inline constexpr double kSettleS = 0.3;

// Two arms side by side on a row of units (rails or modules), the second
// one unit behind the first: lead[k] and follow[k] are each arm's time on
// unit k, its tool changes included. The follower starts unit k when the
// leader has finished unit k and it has finished unit k-1, so the end is
// the largest over k of sum(lead[0..k]) + sum(follow[k..end]); 0 when
// either is empty.
double SideBySide(const std::vector<double>& lead,
                  const std::vector<double>& follow);

// The model of a move and of the bay's time, in words.
std::string TimeModelText();

}  // namespace cell::layout

#endif  // SRC_LAYOUT_CHECK_H_
