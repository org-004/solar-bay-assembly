// Collision checking for the motion planner: at given joint angles, is the
// arm, with its tool and whatever module it carries, clear of everything
// else in the cell, and of itself?
//
// The answer comes from MuJoCo's own collision detection, run on a private
// copy of the model and a scratch mjData that belong to the checker. The
// mjData being simulated is never written; SetScene only reads it.
//
//   std::optional<CollisionChecker> checker =
//       CollisionChecker::Create(*cell.model());
//   checker->SetScene(*cell.data());        // where the obstacles are
//   checker->IsFree(q);                     // the planner's question
//   checker->SetHeldModule(0, module_in_tip);  // after the grip
//   checker->SegmentIsFree(a, b, step);     // a straight joint-space move
//
// What moves and what stands. The arm's links (arm_base and everything
// under it) and the tool move with the queried angles. A held module
// (SetHeldModule) moves with the tool: at each query it is put where the
// tool tip is at those angles, times the grip offset, so for the check it
// is part of the arm. Everything else is an obstacle and stays where the
// scene says: the modules on the pallet, the pallet, the conveyor's bed,
// the tube with its rails and the modules lying on it, the camera's post,
// boom and housing, and the floor.
//
// What counts as a collision: any pair of geoms closer than the margin (or
// overlapping) where at least one belongs to the arm, the tool or the held
// module. So: the arm, tool or held module against any obstacle, and
// against each other (self-collision: link against link, the held module
// against a link). What does not count:
//   - two obstacles touching each other: the stack's modules resting on one
//     another and on the pallet, modules lying on the rails. They are
//     there whatever the arm does.
//   - the tool against the held module: the cups sit on its glass.
//   - pairs MuJoCo never tests in this model: two geoms of one rigid body
//     or of bodies welded together, a body and its parent (so the tool and
//     links 5 and 6, each link and the next), the base and link 1 (excluded
//     in models/arm.xml), the base and anything fixed to the world such as
//     the floor, and the bed and the tube (excluded in conveyor.xml).
//   - pairs allowed for the query with AllowContact: where a move is meant
//     to touch, such as the cups on the module about to be gripped or the
//     held module on the rails at the set-down.
//
// The margin. A configuration is free only if every counted pair is at
// least `margin` apart (self-collision pairs: `self_margin`). It is done
// with MuJoCo's own contact margin, on the private model only: MuJoCo
// reports a contact for a pair of geoms closer than the sum of their two
// geom_margin values, with its distance. The checker sets every moving
// geom's margin to self_margin / 2 and every obstacle's to
// margin - self_margin / 2, so moving against obstacle sums to `margin` and
// moving against moving to `self_margin`. This costs next to nothing: the
// same one collision pass, with slightly larger bounding volumes. The other
// way, mj_geomDistance on every pair that matters, costs 4 to 5 times more
// (42 to 50 us against 9 us a query, optimized): it measures how far apart
// each pair is, out to CheckOptions::clearance_range, not only whether it
// is closer than the margin. Check uses it only when asked for the
// clearance.
//
// Speed. A query writes the arm's six angles (and the held module's pose)
// into the scratch data and runs two MuJoCo calls, nothing else:
//   mj_kinematics  body, geom and site poses from qpos: all that collision
//                  detection reads (geom_xpos, geom_xmat, and the bodies'
//                  inertial frames, xipos and ximat, for the bounding-volume
//                  trees). mj_fwdPosition would also build the mass matrix,
//                  its factorization, the tendons and the constraints,
//                  none of which collision needs.
//   mj_collision   broad phase (sweep and prune over the bodies' boxes),
//                  mid phase (each body's tree of geom boxes) and narrow
//                  phase (the exact distance, by GJK and EPA for the mesh
//                  hulls), filling contact. It computes nothing else.
// The scene holds 30 modules standing in a stack, with some 240 contacts
// among them and with the pallet, that have nothing to do with the arm.
// By default (CollisionOptions::skip_obstacle_pairs) the private model's
// contype and conaffinity bits are rewritten so that MuJoCo itself never
// pairs two obstacles, nor the tool with the held module; the broad phase
// still sees every body, but the expensive phases only arm against world.
// tests/collision_test.cc checks over thousands of random configurations
// that this gives the same answers as an unmodified copy of the model, and
// times it: optimized, about 9 us a query along the scripted path and 15
// to 17 us over random configurations, against 48 to 56 us unfiltered;
// MuJoCo then makes 5 to 10 contacts a query instead of 240 to 370.
//
// Resolution of a segment. Between two checked configurations the checker
// has not looked. A joint turning by dq moves a point at most dq times its
// distance from the joint's axis, and the six add up at worst; LeverArms
// bounds those distances (metres per radian), for the arm, the tool and the
// held module. So along a straight joint-space segment no point moves more
// than SweepBound(from, to) metres, and between two neighbouring samples no
// more than that divided by the number of steps. A sample spacing of
// `resolution` metres leaves every pose within resolution / 2 of a checked
// one; with margin >= resolution / 2, a segment found free is truly clear.
// SuggestStep and SegmentStep turn a resolution into a joint step.
//
// What the checker knows of the world. A real cell does not know where its
// modules and its pallet truly are; the planner must work from what the
// camera estimated. SetScene(world_state) copies the simulator's true state
// of every obstacle: a convenience for tests, and right for what is known
// by design (the conveyor's bed, the camera's post) or measured (the tube,
// from the conveyor's encoder). The cell's control code should set the
// obstacles it estimated itself, with SetModulePose and SetPalletPose, after
// SetScene or instead of it.
//
// One checker per thread: queries write the checker's own scratch data.
//
// Units: metres and radians. Poses are in the world frame unless named
// otherwise.

#ifndef SRC_COLLISION_H_
#define SRC_COLLISION_H_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/mj_handles.h"

namespace cell {

struct CollisionOptions {
  // How far the arm, the tool and the held module must stay from every
  // obstacle. 2 cm: a planner that weights its metric by LeverArms and
  // checks edges every 4 cm (src/rrt.h) leaves every pose along a free edge
  // within 2 cm of a checked one, and the margin covers that. The scripted
  // cycle stays 30 mm or more from everything it is not meant to touch: the
  // cups at the end of the approach, and the held module's bottom edge over
  // the deck as it leaves the stack (tests/collision_test.cc).
  double margin = 0.02;  // m

  // How far the arm's links, the tool and the held module must stay from
  // each other. Zero by default: the joint ranges keep the real links
  // apart, and the links' collision hulls are fatter than the links; those
  // of links 4 and 6 are only 77 mm apart at an ordinary wrist bend on the
  // scripted cycle, so a self margin would take away wrist range the real
  // arm has. At most 2 * margin (the margins are shared out as described at
  // the top of this file).
  double self_margin = 0.0;  // m

  // Lets MuJoCo skip pairs of obstacles and the tool-module pair in the
  // private model (see Speed at the top). Changes no answer, only the time;
  // off is for tests that prove it.
  bool skip_obstacle_pairs = true;
};

// What Check reports beyond yes or no.
struct CheckOptions {
  // Every colliding pair rather than the first found.
  bool all_pairs = false;
  // Measure the smallest distance from the arm, tool and held module to
  // the obstacles, and among themselves, with mj_geomDistance. Distances
  // beyond clearance_range are not measured and read as clearance_range.
  bool clearance = false;
  double clearance_range = 1.0;  // m
};

// One pair of geoms closer than its margin.
struct CollidingPair {
  int geom1 = -1;  // the arm's, the tool's or the held module's
  int geom2 = -1;
  int body1 = -1;
  int body2 = -1;
  // How far apart, m: negative when they overlap, positive but under the
  // margin when they are merely too close.
  double distance = 0.0;
  // "geom (body) - geom (body)", for messages, the moving one first.
  std::string between;
};

struct CollisionReport {
  bool free = true;
  // False when an angle is outside its joint's range; then nothing was
  // checked and free is false too.
  bool in_range = true;
  // The colliding pairs: the first found, or with all_pairs every one (one
  // entry per geom pair, its smallest distance), deepest first.
  std::vector<CollidingPair> pairs;
  // With CheckOptions::clearance: the smallest distance from the arm, tool
  // and held module to any obstacle, and the pair it is between; and the
  // same among the arm, tool and held module. Allowed pairs are left out.
  // Negative when overlapping.
  std::optional<double> clearance;
  std::string clearance_between;
  std::optional<double> self_clearance;
  std::string self_clearance_between;
};

// The outcome of a segment check.
struct SegmentReport {
  bool free = true;
  // Configurations checked, both ends included.
  int samples = 0;
  // Where the first colliding sample is along the segment, from 0 (`from`)
  // to 1 (`to`), and the sample before it, the last known free one (-1 if
  // `from` itself collides). Both 1 when the whole segment is free.
  double hit_fraction = 1.0;
  double free_fraction = 1.0;
  Eigen::VectorXd hit;  // the first colliding configuration, or empty
  // The report of the first colliding sample.
  CollisionReport hit_report;
};

class CollisionChecker {
 public:
  // Builds a checker for `model`, which must hold the arm (src/arm.h).
  // Modules (bodies module_<i>) and the pallet are found if present
  // (src/pallet.h). The scene starts as the model was built: everything at
  // its nominal place. Returns nothing, with the reason in `error` if it is
  // not null, if the arm is missing, the margins are out of range, or the
  // model holds explicit contact pairs (<pair>, which bypass the margins
  // set here) or contact bits under which some geom of the arm, the tool or
  // a module would not collide with some other colliding geom (the fast
  // filtering could then not reproduce MuJoCo's answer).
  static std::optional<CollisionChecker> Create(
      const mjModel& model, const CollisionOptions& options = {},
      std::string* error = nullptr);

  // ---------------------------------------------------------------- scene
  // Copies every obstacle's position from `world_state`, an mjData of the
  // same model: every joint that is not the arm's (the modules' free
  // joints, the conveyor's slide) and the pallet's mocap pose. A module
  // that is held by the checker (SetHeldModule) is still placed by the tool
  // at each query; its pose here is kept for when it is let go
  // (ClearHeldModule). Reads only; `world_state` is not changed. Truth: see
  // the top of this file.
  void SetScene(const mjData& world_state);

  // Puts module i (module_<i>) at `pose` (its centre and axes, as
  // Cell::TrueModulePose), for an estimated stack. Returns false if there
  // is no such module.
  bool SetModulePose(int i, const Eigen::Isometry3d& pose);

  // Puts the pallet at `pose` (its origin, as PalletPose in src/pallet.h).
  // Its modules do not follow: set them with SetModulePose. Returns false
  // if the model has no pallet.
  bool SetPalletPose(const Eigen::Isometry3d& pose);

  // Sets the conveyor's slide, which carries the tube and its rails, to
  // `position` (metres, as Conveyor::SensorPosition reads it from the
  // encoder). Modules lying on the tube do not follow. Returns false if
  // the model has no joint conveyor_slide.
  bool SetTubeSlide(double position);

  // ---------------------------------------------------------- held module
  // From now on module i moves with the tool: at each query its frame is
  // the tool tip's frame at the queried angles times `module_in_tip` (as
  // Suction::HeldPoseInTip). Returns false if there is no such module.
  // Holding another module first lets go of the one held.
  bool SetHeldModule(int i, const Eigen::Isometry3d& module_in_tip);

  // Nothing held. The module goes back to where the scene last put it
  // (SetScene or SetModulePose); after a placement, say where it now lies
  // with SetModulePose or SetScene.
  void ClearHeldModule();

  std::optional<int> held_module() const { return held_; }

  // ------------------------------------------------------ allowed contacts
  // Contacts between bodies a and b (body ids, or names) do not count, in
  // either order, until ClearAllowedContacts. For the moments a move is
  // meant to touch: the tool and module_<k> while gripping it; the held
  // module and module_<k+1> or the pallet while it leaves the stack; the
  // held module and "tube" (whose geoms are the rails) at the set-down; the
  // tool and the module just let go, while retreating from it. Returns
  // false if a body does not exist.
  bool AllowContact(int body_a, int body_b);
  bool AllowContact(const std::string& body_a, const std::string& body_b);
  void ClearAllowedContacts();
  // The allowed pairs (smaller id first), and setting them all at once: to
  // save and restore them around a phase.
  const std::vector<std::pair<int, int>>& allowed_contacts() const {
    return allowed_pairs_;
  }
  bool SetAllowedContacts(const std::vector<std::pair<int, int>>& pairs);

  // -------------------------------------------------------------- margins
  // Changes the margins (see CollisionOptions). Returns false and changes
  // nothing if one is negative or not finite, or self_margin > 2 * margin.
  bool SetMargins(double margin, double self_margin);
  double margin() const { return options_.margin; }
  double self_margin() const { return options_.self_margin; }

  // -------------------------------------------------------------- queries
  // True if the arm at `angles` (six, arm order) is within its joint ranges
  // and nothing counted is closer than the margins. False for angles of the
  // wrong size or not finite.
  bool IsFree(const Eigen::VectorXd& angles);

  // The same with the details. Returns nothing for angles of the wrong size
  // or not finite.
  std::optional<CollisionReport> Check(const Eigen::VectorXd& angles,
                                       const CheckOptions& options = {});

  // Checks the straight joint-space segment from `from` to `to`, both ends
  // included, at evenly spaced configurations no joint more than `step`
  // radians apart, in order from `from`, stopping at the first that is not
  // free. Returns whether all are free, and where the first collision is in
  // `report` if it is not null. Returns false, with report->samples 0, for
  // ends of the wrong size or not finite, or a step that is not positive.
  bool SegmentIsFree(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                     double step, SegmentReport* report = nullptr);

  // ----------------------------------------------------------- resolution
  // For each joint, an upper bound on how far any point of the arm's
  // colliding geoms, the tool's and the held module's can be from that
  // joint's axis, in any configuration: metres moved per radian turned.
  // Includes the held module while one is held. Good weights for the
  // planner's metric (src/rrt.h).
  Eigen::VectorXd LeverArms() const;

  // The farthest any point of the arm, tool and held module can move along
  // the straight segment from `from` to `to`: sum over joints of |change| *
  // LeverArms. Metres.
  double SweepBound(const Eigen::VectorXd& from,
                    const Eigen::VectorXd& to) const;

  // The joint step for SegmentIsFree that moves no point more than
  // `resolution` metres between neighbouring samples on any segment: for
  // all six joints changing by the step at once, resolution /
  // sum(LeverArms). The worst case; SegmentStep is the same for one
  // segment, usually several times larger.
  double SuggestStep(double resolution) const;
  double SegmentStep(const Eigen::VectorXd& from, const Eigen::VectorXd& to,
                     double resolution) const;

  // --------------------------------------------------------------- access
  // The private model (margins and contact bits as set here) and the
  // scratch data, as the last query left it: for drawing what was checked.
  const mjModel& model() const { return *model_; }
  const mjData& data() const { return *data_; }
  const Arm& arm() const { return arm_; }
  // Body ids of module_<i>, in order; empty if the model has none.
  const std::vector<int>& modules() const { return modules_; }
  // Configurations checked so far, by every query.
  long queries() const { return queries_; }

 private:
  // What a body is to the check.
  enum class Role : unsigned char {
    kObstacle,
    kLink,  // the arm's links, base included
    kTool,
    kHeld,  // the held module
  };

  CollisionChecker() = default;

  bool Moving(int body) const { return role_[body] != Role::kObstacle; }
  // Whether a contact or distance between geoms g and h counts, by role
  // and allowance (not by distance).
  bool Counts(int g, int h) const;
  // Writes the angles and the held module's pose into the scratch data and
  // runs mj_kinematics. Does not check `angles`.
  void Pose(const Eigen::VectorXd& angles);
  // The query itself; `angles` has been checked for size and finiteness.
  // Stops at the first colliding pair unless options.all_pairs. Fills in
  // CollidingPair::between only with `names`.
  CollisionReport Query(const Eigen::VectorXd& angles,
                        const CheckOptions& options, bool names);
  // Rewrites the private model's contact bits and margins from roles and
  // options_, then the bodies' summaries of them.
  void ApplyFilters();
  // The pairs whose distance Check measures for the clearance: every geom
  // pair MuJoCo would test that counts.
  void BuildDistancePairs();
  std::string PairName(int g, int h) const;
  double HeldLever() const;

  ModelPtr model_;
  DataPtr data_;
  CollisionOptions options_;
  Arm arm_;
  std::vector<int> modules_;
  int pallet_mocap_ = -1;
  int slide_qpos_ = -1;
  std::vector<bool> is_arm_qpos_;  // per qpos entry

  std::vector<Role> role_;  // per body
  // The contact bits as loaded, per geom.
  std::vector<int> contype_;
  std::vector<int> conaffinity_;
  std::vector<unsigned char> allowed_;  // nbody x nbody
  std::vector<std::pair<int, int>> allowed_pairs_;

  std::optional<int> held_;
  Eigen::Isometry3d held_in_tip_ = Eigen::Isometry3d::Identity();
  // The held module's qpos as the scene last set it, restored on release.
  std::vector<double> held_scene_qpos_;

  // Lever arms of the arm and tool, without a held module, and the bound on
  // the distance from each joint's axis to the tool body's origin, to which
  // a held module adds.
  Eigen::VectorXd lever_;
  Eigen::VectorXd lever_to_tool_;

  std::vector<std::pair<int, int>> distance_pairs_;
  bool distance_pairs_valid_ = false;
  long queries_ = 0;
};

}  // namespace cell

#endif  // SRC_COLLISION_H_
