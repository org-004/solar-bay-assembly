// The checks on a layout.
//
// CheckLayout takes a layout (src/layout.h), builds its model
// (src/layout_model.h) and checks one whole bay. It returns one row for each
// check with the figure and whether it passes, and the steps of building one
// bay with their times. //:check_layout prints them and exits non-zero when
// a row fails. The kinds of check, the `check` of a row:
//
//   transport  everything that travels inside is, in the stowed state,
//              within the container's inside envelope, and its mass is
//              within what is left for the cell (src/container.h);
//   reach      every point an arm must work at is reached by the arm the
//              layout gives it, the tool along the direction the document
//              gives, inside the datasheet's joint ranges less a margin;
//   clearance  at each of those poses the arm, its tool and what it carries
//              keep a stated distance from the bay as built up to that step,
//              the holder, the container, the pallet, the stocks and the
//              other arms where they wait; and the free spaces of the tools
//              and of the locating move are empty;
//   load       what each arm carries, tool and part, is within its payload;
//   ways       the space a tube sweeps coming in and a bay sweeps leaving is
//              empty;
//   time       the time one bay takes, from the steps, by a stated model of
//              the arms' moves; it has no limit: its rows pass when every
//              step has a pose;
//   hold       the holder grips only stretches the document allows, not the
//              two ends alone, and is built for the moment the document asks.
//
// One model for everything. The checker loads the layout's model once in
// the `everything` state and once `stowed` (for transport only). What is
// "there" at a step is a list of geoms taken from LayoutHandles; what is not
// there is simply not measured against. Distances are MuJoCo's
// mj_geomDistance between two geoms (convex hulls for meshes), after a
// box-against-box test on the geoms' world bounding boxes that skips every
// pair that cannot be nearer than the nearest found so far, and out to 1 m:
// a figure of 1.000 means nothing is within a metre.
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
//                  pose when one arm and tool do them;
//   module_set m   the centre of module m's glass top; z down; x along X,
//                  either sign;
//   module_take j  module j of the pallet (0 is taken first; src/layout.h
//                  defines the pallet's frame): standing "on_long_edge", the
//                  centre of its glass face, which faces the pallet's -y,
//                  the tool's z along the pallet's +y (level, onto the
//                  glass); lying "flat", the centre of the top module's
//                  glass, z down; either way the tool's x along the
//                  pallet's x, either sign. Module m of the bay is take
//                  j = m; the other takes are checked for reach and
//                  clearance only. The turn from upright to flat is part
//                  of the move from the take pose to the set-down pose;
//   rail_take k, pins_take 0
//                  the stock's pick point, z along its pick axis, any spin.
//
// The margins. A joint is kept joint_margin_deg inside each end of the
// datasheet's range of src/arm_catalog.h (and inside the model's own range
// where that is tighter); a reach row's figure is the least distance, in
// degrees, of any joint from its datasheet limit, and the row names the
// joint. A pose is reached when the working point is within 0.1 mm of the
// point and the tool's axes within 0.06 degrees of their directions. A
// clearance row passes when the least distance is at least clearance_m; a
// free space row when it is more than zero; a ways row when the least
// distance over the sweep is at least clearance_m.
//
// The inverse kinematics is the checker's own: damped least squares over
// the arm's six joints with the carriage held at one track value, from the
// arm's previous solution, then its wait pose, then ik_seeds seeds drawn
// once from a fixed generator (src/portable_random.h), so every platform
// finds the same poses. Each joint is clamped into its range at every step.
// The track value is the layout's stop for the kind (one for every point or
// one per point); without one it is searched: the value that stands the
// base opposite the point, then 0.15, 0.30, 0.45, 0.60 m either side,
// clamped to the track. The first pose that is reached and clear wins; if
// none is clear, the reached pose with the most clearance is reported. A
// stop outside the track's lo..hi fails the reach row of its point ("stop X
// outside the track's lo..hi"); a wait_track outside it fails the reach row
// "wait track <arm>", a stow_track the transport row "stow track <arm>"
// (figure: how far inside the range, negative outside, limit 0).
//
// What is there at a step. The container, the holder, the tube, the pallet's
// base and the modules left on it, the stocks, the items, every arm's beam,
// the other arms at their wait poses with their carriages, their first
// tools on the flange and their other tools where they wait, and the active
// arm's own tools that wait. The bay as built: at rail_take k and
// rail_set k rails 0 .. k-1; at strap_bolt k all rails if the order is
// all_rails_then_all_bolts, otherwise rails 0 .. k; at module_take j for
// module m < 9 all rails and modules 0 .. m-1 (for the other takes all nine
// modules); at module_set m all rails and modules 0 .. m-1; at locate, pin
// and swage on module m all rails and modules 0 .. m, and, for m < 8, also
// with module m+1 already on: the worse of the two counts. The pallet holds
// 36 modules in the rail phase, modules j .. 35 at take j (module j is the
// one held), and modules - m - 1 while module m is pinned. What moves: the
// active arm's links, its carriage, its tool on the flange and the part it
// holds (a rail at rail_take and rail_set, a module at module_take and
// module_set).
//
// What touches by design and is not counted at all:
//   - the arm's links against each other, against its own carriage and its
//     tool, and the tool against the part it holds: the maker's ranges and
//     the tool's mount are trusted;
//   - the arm's carriage, and its base fixed to the carriage, against its
//     own beam (it runs on it) and against its own tools that wait on it;
//   - a held part against what it is set on or taken from: a rail against
//     the tube at rail_set, a taken rail against its stock's boxes, module
//     m against rails m and m+1 at module_set, pallet module j at its take
//     against module j+1 behind (or below) it and the deck it stands on
//     (flat: the deck only for the last), a magazine against its stock;
// What is near by design, and so is held to no distance but must not go
// into it: a pair of these counts only when its distance is below zero, and
// then it is the row's figure, named with "(near by design, but into each
// other)":
//   - at module_set m, the held module against module m-1 beside it: the
//     bay's own gap, kModuleGap (10 mm), which is not the layout's;
//   - a fastening tool's last box against the two parts its fastening
//     joins: the nutrunner's socket against rail k (its strap's ear) and the
//     tube, whose underside is 18 mm above the bolt's axis; the pin tool's
//     nose against the rail and the module of the hole. The working point
//     stops 5 mm short of the sheets, so these are 5 to 10 mm by design.
// The held part counts against the arm's own links and carriage, at
// clearance_m: every link geom except those of the last wrist body (the
// body that carries the flange, with the bodies welded to it, up to and
// including the first one with a joint: the UR's wrist_3_link, the KUKA's
// link 6 and its flange), to which the tool is bolted and which the part
// cannot but be near.
//
// The tool in the free space. The probe's cylinders say what else must keep
// out of the free space; the tool itself must fit in it too. One row per
// fastening kind that a layout gives to a tool, "tool in free space <kind>
// <tool>": every box of the tool that reaches into the length of the free
// space behind the working point (kStrapBoltFreeLength for
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
// Transport. The stowed model: every geom that is not the container's, at
// its stow pose, its world bounding box (exact corners for boxes, every
// vertex for meshes, the rotated local box otherwise) inside
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
// The order of the steps of one bay (BaySteps): tube in, clamp; the rails
// and their bolts in the layout's rail_order (take rail k, set rail k,
// then the bolts, or each rail's bolt after it); then for each module m:
// the module arm takes pallet module m and sets it, and the pin arm locates
// and pins the four holes in bay::kLocateOrder and swages them in
// bay::kSwageOrder, having taken a magazine once before the first pin; a
// tool change wherever an arm's next work needs another tool; release, bay
// out, and each arm back to its wait pose and first tool. Every arm starts
// at its wait pose with its first tool. Two phases can be worked by two
// arms side by side (SideBySide): the rails, when rail_set and strap_bolt
// are given to different arms (the bolting arm one rail behind, each take
// counted with the arm that takes), and the modules, when module_set and
// pin are (the pin arm one module behind); with one arm a phase is the
// plain sum of its steps. The time row has no limit: it passes when every
// step it needs has a pose. The model of a move
// and of the bay's time is TimeModelText(), printed with the rows.

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
