// What moves, what is there, what touches by design and what is near by
// design, in a layout's model: the one definition of these for every program
// that checks or plans motion in a layout (the checker of src/layout_check.h,
// the planner of the next milestone).
//
// One model for everything. LayoutScene loads the layout's model once in the
// `everything` state of src/layout_model.h: all ten rails, all nine modules,
// every pallet module, every tool of every arm on its flange at once and
// every part an arm can hold. What is "there" at a step is a list of geoms
// taken from LayoutHandles; what is not there is simply not measured
// against. The model is LoadLayoutModel's, which turns every capsule that
// can collide into the cylinder that covers it (MuJoCo 4.0.0 measures a
// capsule whose axis passes through a box as apart), so every distance
// measured here goes round that defect. The scene keeps no state that a
// query changes: the caller brings its own mjData (MakeData), sets the arms'
// poses in it (SetPose, AllAtWait, then mj_kinematics) and asks for
// distances with it, so several threads may each use their own mjData on one
// scene.
//
// Distances. LeastDistance is the one distance function. It measures with
// MuJoCo's mj_geomDistance between two geoms (convex hulls for meshes), after
// a box-against-box test on the geoms' world bounding boxes that skips every
// pair that cannot be nearer than the nearest found so far, and out to a cap
// of 1 m: a figure of 1.000 means nothing is within a metre.
//
// What moves. The moving set of an arm (Moving) is its link geoms (those
// fixed to its carriage, the base, count as the carriage), its carriage, the
// boxes of the tool on its flange (the last box of the tool, the one at the
// working point, in a group of its own) and the geoms of the part it holds:
// a rail at rail_take and rail_set, a module at module_take and module_set;
// a pin magazine has no geoms. Each geom of the moving set is in one group
// (MovingGroup), and an obstacle can be skipped by some groups.
//
// What is there (Obstacles), for an arm with a tool on its flange and the
// bay as built (Built: which rails and modules are on the tube and how many
// modules are left on the pallets): the base (the container or the flat
// rack), the holder, the tube, the rails and modules built, each pallet's
// base and the modules left on it (the last ones in the order of taking),
// the stocks, the items, every arm's beam, the tools of every arm that wait
// on its carriage, and, unless the caller leaves them out, the other arms
// where the caller's mjData has them, with their carriages and the tool on
// their flange (their first tool unless the caller says another). The
// checker has the other arms at their wait poses; a planner that plans one
// arm alone leaves them out and then measures one arm's Moving() against
// another's through the same LeastDistance.
//
// What touches by design and is not counted at all:
//   - the arm's links against each other, against its own carriage and its
//     tool, and the tool against the part it holds: the maker's ranges and
//     the tool's mount are trusted;
//   - the arm's carriage, and its base fixed to the carriage, against its
//     own beam (it runs on it) and against its own tools that wait on it;
//   - at a work point, a held part against what it is set on or taken from
//     (DesignPairs::held_skip): a rail against the tube at rail_set, a taken
//     rail against its stock's boxes, module m against rails m and m+1 at
//     module_set, pallet module j at its take against module j+1 behind (or
//     below) it on the same pallet and the deck it stands on (flat: the deck
//     only for the last of its pallet), a magazine against its stock.
// What is near by design, and so is held to no distance but must not go
// into it: a pair of these counts only when its distance is below zero, and
// then it is the figure, named with "(near by design, but into each other)":
//   - at module_set m, the held module against module m-1 beside it
//     (DesignPairs::held_near): the bay's own gap, kModuleGap (10 mm), which
//     is not the layout's;
//   - a fastening tool's last box against the two parts its fastening joins
//     (DesignPairs::last_near): the nutrunner's socket against rail k (its
//     strap's ear) and the tube, whose underside is 18 mm above the bolt's
//     axis; the pin tool's nose against the rail and the module of the hole.
//     The working point stops 5 mm short of the sheets, so these are 5 to
//     10 mm by design;
//   - on the legs of a take or a set where nothing is held (see Leg), the
//     tool against the part it is about to take or has just set
//     (DesignPairs::tool_near): pallet module j, or the stock's boxes, or
//     rail k on the tube, or module m on its rails.
// The held part counts against the arm's own links and carriage, at the
// caller's clearance: every link geom except those of the last wrist body
// (LinksForHeld: the body that carries the flange, with the bodies welded
// to it, up to and including the first one with a joint: the UR's
// wrist_3_link, the KUKA's link 6 and its flange), to which the tool is
// bolted and which the part cannot but be near.
//
// The ways (WayMoving, WayObstacles). Along tube_in the tube alone moves;
// along bay_out the finished bay (tube, ten rails, nine modules). What is
// there: the base, the holder, every pallet's deck and every pallet module,
// the stocks, the items but the way's carriers, and every arm at its wait
// pose with its beam, carriage, first tool on the flange and other tools
// where they wait. The carriers (WayCarriers): for a layout with ports, the
// items named "item:<name>" in hands_over of the ports that name
// "way:tube_in" (for the tube's way) or "way:bay_out" (for the bay's), and
// of the ports that share a zone with one of them; for a layout without
// ports (run 011's five on the container), run 011's rule, kept for its
// record: every item whose `does` mentions "tube" or "bay".
//
// The bay at a work point (AtWorkPoint). What follows is said for the plain
// order of modules, 0, 1, ..., 8. With a layout's order (src/layout.h,
// WorkOrder) "modules 0 .. j-1" means the first j modules of that order,
// "module m-1 beside it" means each neighbour already set, "module m+1
// already on" means the next module of the order, and at the holes the
// second Built has both neighbours too (a module arm that runs ahead may
// set them while the holes are worked); for the plain order this is the
// same. With Leg::kWork: at rail_take k and
// rail_set k rails 0 .. k-1; at strap_bolt k all rails if the order is
// all_rails_then_all_bolts, otherwise rails 0 .. k; at module_take j, for a
// take the bay uses (j < 9), all rails and modules 0 .. j-1, and for the
// other takes all nine modules; at module_set m all rails and modules
// 0 .. m-1; at locate, pin and swage on module m all rails and modules
// 0 .. m, and, for m < 8, also with module m+1 already on: two Builts, the
// worse of the two counts. The pallets hold all their modules in the rail
// phase, all but takes 0 .. j at take j (module j is the one held), and all
// but 0 .. m while module m is pinned. Who does the point is the layout's
// division of the kind's points (FindWork of src/layout.h).
//
// The legs. A work point is come to along a straight line from its approach
// pose and left along one to its retreat pose (src/layout.h, Approach). What
// is there and what moves on each:
//   - kApproach of a take (rail_take, module_take, pins_take): nothing is
//     held; the part is still where it is taken from (pallet module j stands
//     on its pallet and is an obstacle; a rail or a magazine in its stock has
//     no geoms of its own), and the tool against what it is about to take
//     (pallet module j; the stock's boxes) is near by design. kRetreat of a
//     take: the part is held, with the work point's own pairs.
//   - kApproach of a set (rail_set, module_set): the part is held, with the
//     work point's own pairs. kRetreat of a set: nothing is held; the part
//     lies in the bay as an obstacle (rail k on the tube; module m on its
//     rails), and the tool against it is near by design.
//   - strap_bolt, locate, pin, swage: both legs are the work point's own.

#ifndef SRC_LAYOUT_SCENE_H_
#define SRC_LAYOUT_SCENE_H_

#include <array>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "src/arm_catalog.h"
#include "src/layout.h"
#include "src/layout_model.h"
#include "src/mj_handles.h"

namespace cell::layout {

struct WorkPoint;  // src/layout_check.h

// The cap of a distance: nothing is looked for further than this.
inline constexpr double kSceneFar = 1.0;  // m

// The groups of a moving set. An obstacle can be skipped by some of them.
enum MovingGroup : unsigned {
  kLinks = 1,
  kCarriage = 2,
  kToolBody = 4,
  kToolLast = 8,
  kHeld = 16,
  kProbe = 32,  // A probe cylinder of a free space (src/layout_model.h).
};

struct MovingGeom {
  int geom = -1;
  unsigned group = 0;  // One MovingGroup.
};

struct Obstacle {
  int geom = -1;
  unsigned skip = 0;  // Groups that do not measure against it.
  // Of those, the groups that are near it by design but must not go into
  // it: their pair counts only when its distance is below zero.
  unsigned near = 0;
};

// The least distance found and its pair.
struct Nearest {
  double distance = kSceneFar;
  int moving = -1;
  int obstacle = -1;
  bool near = false;  // A pair near by design that goes into each other.
};

// The bay as built at a step.
struct Built {
  std::vector<int> rails;    // On the tube.
  std::vector<int> modules;  // On their rails.
  // Modules left on the pallets: the last `on_pallets` in the order of
  // taking (module_take's points) are there.
  int on_pallets = 0;
};

// The pairs of one step that touch or are near by design (see above).
struct DesignPairs {
  std::set<int> held_skip;  // The held part touches these: not measured.
  std::set<int> held_near;  // The held part is near these: not into them.
  std::set<int> last_near;  // The tool's last box is near these.
  std::set<int> tool_near;  // The whole tool is near these.
};

enum class Leg { kWork, kApproach, kRetreat };

// The part an arm holds at a work point of the kind: "rail" (rail_take,
// rail_set), "module" (module_take, module_set), "pin magazine"
// (pins_take) or "".
std::string HeldAtWork(const std::string& kind);

// What is there and what moves at a work point, on one of its legs.
struct StepScene {
  int arm = -1;              // Index into the layout's arms.
  std::string tool;
  std::string held;          // "module", "rail", "pin magazine" or "".
  std::vector<Built> built;  // One or two; the worse counts.
  DesignPairs pairs;
  std::string error;         // No arm, no tool, no work: nothing else is set.
};

// The two ways of a layout (src/layout.h, Way).
enum class WayKind { kTubeIn, kBayOut };

// Whether the other arms are obstacles, and with which tools.
struct ObstacleOptions {
  // The other arms' links, carriages and tools on the flange, where the
  // caller's mjData has them. Their beams and waiting tools always are.
  bool other_arms = true;
  // The tool on another arm's flange, by arm index; its first tool if not
  // listed ("" for a bare flange).
  std::map<int, std::string> tools;
};

// One arm of the scene.
struct SceneArm {
  const ArmUse* use = nullptr;
  const ArmModel* cat = nullptr;
  const ArmHandle* h = nullptr;
  std::array<double, 6> lo{}, hi{};    // Where a solver may put each joint, rad:
                                       // the datasheet's range less the joint
                                       // margin, inside the model's own range.
  std::array<double, 6> dlo{}, dhi{};  // The datasheet's range, rad.
  std::array<double, 6> wait{};        // The wait pose, rad.
  double wait_track = 0.0;
};

// A pose of an arm: six joints in radians and the carriage's track value.
struct ArmPose {
  std::array<double, 6> q{};
  double track = 0.0;
};

class LayoutScene {
 public:
  // Builds the layout's `everything` model. The joint margin sets each
  // arm's lo and hi. Returns nullptr and sets *error on a fault.
  static std::unique_ptr<LayoutScene> Load(const Layout& layout,
                                           const ModelPaths& paths,
                                           double joint_margin_deg,
                                           std::string* error);
  LayoutScene(const LayoutScene&) = delete;
  LayoutScene& operator=(const LayoutScene&) = delete;

  const Layout& layout() const { return layout_; }
  const mjModel& model() const { return *model_; }
  const LayoutHandles& handles() const { return handles_; }
  int ArmCount() const { return static_cast<int>(arms_.size()); }
  const SceneArm& arm(int i) const { return arms_[i]; }
  int ArmIndex(const std::string& name) const;  // -1 if none.

  // A data for the model at its keyframe 0, every arm at its wait pose,
  // its kinematics done.
  DataPtr MakeData() const;
  // Puts one arm's joints and track into the data (no kinematics).
  void SetPose(mjData* d, int arm, const ArmPose& pose) const;
  // Every arm at its wait pose (no kinematics).
  void AllAtWait(mjData* d) const;

  // (a) What moves: the arm's links, carriage, tool on the flange and the
  // part it holds ("module", "rail"; anything else has no geoms).
  std::vector<MovingGeom> Moving(int arm, const std::string& tool,
                                 const std::string& held) const;
  // The arm's link geoms that its held part counts against.
  std::vector<int> LinksForHeld(int arm) const;

  // (b) What is there for `arm` with `tool` on its flange, the bay as
  // `built`, with the pairs' marks.
  std::vector<Obstacle> Obstacles(int arm, const std::string& tool,
                                  const Built& built,
                                  const DesignPairs& pairs,
                                  const ObstacleOptions& options = {}) const;

  // (c) The bay, the arm, its tool and part and the pairs by design at a
  // work point of the kind, on one of its legs.
  StepScene AtWorkPoint(const WorkPoint& point, Leg leg = Leg::kWork) const;

  // What moves along a way (group kHeld), what is there, and the items
  // that carry it and are left out.
  std::vector<MovingGeom> WayMoving(WayKind way) const;
  std::vector<Obstacle> WayObstacles(WayKind way) const;
  std::vector<std::string> WayCarriers(WayKind way) const;

  // (d) The least distance between the moving geoms and the obstacles, as
  // of the data's geom poses, out to `far`.
  Nearest LeastDistance(mjData* d, const std::vector<MovingGeom>& moving,
                        const std::vector<Obstacle>& obstacles,
                        double far = kSceneFar) const;

  // Per body of the model, whether its pose depends on one of the arm's
  // joints or its track: the bodies SetPose moves.
  std::vector<bool> MovesWith(int arm) const;

  // "a to b", "a to b (near by design, but into each other)" or "nothing
  // within 1 m".
  std::string PairText(const Nearest& n) const;
  std::string GeomName(int geom) const;

  // The geoms of the base the cell stands on (the container's frame or the
  // flat rack).
  const std::vector<int>& BaseGeoms() const;

 private:
  LayoutScene() = default;
  bool Init(const ModelPaths& paths, double joint_margin_deg,
            std::string* error);
  bool RidesCarriage(const SceneArm& a, int g) const;

  Layout layout_;
  ModelPtr model_;
  LayoutHandles handles_;
  std::vector<SceneArm> arms_;
};

// (e) LeastDistance prepared for many queries of one arm's moving set
// against one set of obstacles, as a move's planner asks them (MoveWorld,
// src/layout_plan.h): the same answers, bit for bit, in a fraction of the
// time. The rule it rests on: between two queries on one mjData nothing but
// that arm's pose changes in it. So
//   - Place runs the whole of mj_kinematics only on the first query on an
//     mjData; after that it runs MuJoCo's bodies' pass (mj_kinematics1) and
//     places again only the inertial frames, geoms and sites of the bodies
//     that move with the arm (MovesWith), which are the only ones whose
//     poses can differ, through the very function mj_kinematics uses;
//   - the world boxes of geoms that do not move with the arm are kept from
//     the first query;
//   - the pairs that are never measured (a geom against itself, or skipped
//     and not near by design) are dropped once;
//   - an obstacle whose box is farther than `far` from the box around the
//     whole moving set (with a margin of 1 nm against rounding) cannot give
//     a pair nearer than `far` and is passed over whole.
// Every pair that remains is measured as LeastDistance measures it, in the
// same order. Closer asks only whether anything is nearer than `far`, and
// stops at the first pair that shows it: the answer of
// `LeastDistance(...).distance < far`, which does not depend on the order
// in which pairs are tried. One PairQuery per thread.
class PairQuery {
 public:
  PairQuery(const LayoutScene& scene, int arm, std::vector<MovingGeom> moving,
            std::vector<Obstacle> obstacles);
  ~PairQuery();
  PairQuery(const PairQuery&) = delete;
  PairQuery& operator=(const PairQuery&) = delete;
  // Forward kinematics after SetPose of the arm: geom poses as
  // mj_kinematics leaves them.
  void Place(mjData* d);
  // LeastDistance(d, moving, obstacles, far), after Place.
  Nearest Least(mjData* d, double far);
  // LeastDistance(d, moving, obstacles, far).distance < far, after Place.
  bool Closer(mjData* d, double far);

 private:
  struct State;  // Boxes and pairs (src/layout_scene.cc).
  // Indices of the obstacles that may give a pair nearer than `far`.
  void Keep(double far);

  const LayoutScene& scene_;
  std::vector<MovingGeom> moving_;
  std::vector<Obstacle> obstacles_;
  std::vector<int> bodies_;  // Moving with the arm, in order.
  std::vector<int> geoms_;   // Their geoms.
  std::vector<int> sites_;   // Their sites.
  std::vector<bool> moving_moves_, obstacle_moves_;
  std::vector<unsigned char> kind_;  // Per pair i * obstacles + j: 0, 1 or 2.
  std::unique_ptr<State> s_;
  std::vector<int> keep_;
  const mjData* placed_ = nullptr;
  int last_i_ = -1, last_j_ = -1;  // The pair that last showed Closer.
};

}  // namespace cell::layout

#endif  // SRC_LAYOUT_SCENE_H_
