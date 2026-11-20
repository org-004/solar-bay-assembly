// Inverse kinematics: which joint angles put the tool tip at a wanted pose.
//
// Forward kinematics goes from joint angles to where the tool tip is, and
// MuJoCo does it for us. The inverse has no simple formula to read off, so
// it is solved by searching: start from a guess (the seed), see how far the
// tip is from where it should be, turn the joints a little in the direction
// that closes the gap, and repeat until the gap is below a tolerance.
//
// "The direction that closes the gap" comes from the Jacobian: a 6 x 6
// table, recomputed at every guess, that says how fast the tip moves (three
// rows) and turns (three rows) for a small turn of each joint (one column per
// joint). Solving "Jacobian times joint change = wanted tip change" gives the
// joint change. The method is damped least squares; the damping is explained
// at IkOptions::damping.
//
// A six-axis arm usually has several sets of joint angles for one tip pose:
// elbow up or down, wrist flipped or not, joint 1 facing the target or
// turned half a turn away and reaching back over. A search that starts at
// the seed and takes small steps ends at the set nearest the seed, which is
// what a motion planner wants: a path solved waypoint by waypoint, each
// seeded from the one before, stays on one branch. IkOptions can also refuse
// an answer that ended up far from its seed.
//
// The scratch mjData. Each guess is tried by writing it into an mjData and
// running MuJoCo's kinematics, which overwrites body and site poses. Doing
// that in the mjData being simulated would corrupt the simulation, so the
// caller keeps a second mjData for the model (cell::MakeData) and hands that
// to these functions. Only the arm's six entries of its qpos are written by
// us; MuJoCo's kinematics then recomputes the poses, Jacobian inputs and
// centres of mass in it from qpos. Every other qpos entry (a free-floating
// module, the conveyor) is left as it was and has no effect on the arm,
// whose base is fixed to the world.
//
// The solver knows only the arm's geometry and joint ranges. It does not
// check collisions: an answer may put the arm through the pallet, the floor
// or itself.
//
// Units: metres and radians. Poses are in the world frame.

#ifndef SRC_IK_H_
#define SRC_IK_H_

#include <limits>
#include <numbers>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"

namespace cell {

struct IkOptions {
  // The answer is accepted once the tool tip is this close to the target...
  double position_tolerance = 1e-4;  // 0.1 mm
  // ...and its orientation is within this angle of the target's.
  double orientation_tolerance = 0.01 * std::numbers::pi / 180.0;  // 0.01 deg

  // The most guesses to try, counting those rejected for making things
  // worse. From a seed near the answer it takes a handful (a hundred very
  // close to a singularity); from the home pose to anywhere in reach, a few
  // dozen. A solve that stops improving ends sooner, as kStalled.
  int max_iterations = 200;

  // Damping, in the units of the Jacobian (metres per radian and radians per
  // radian). In plain words: the step solves "move the tip by the error"
  // but also pays a penalty of damping^2 times the size of the joint step.
  //
  // Without it, the step is the exact solution of the straight-line guess.
  // Near a singularity, a pose where some direction of tip motion needs a
  // huge joint motion (the arm stretched straight cannot move the tip
  // further out; with joint 5 at zero, joints 4 and 6 turn the tool about
  // the same axis and nothing turns it about the axis square to both), that
  // exact solution asks for enormous joint turns to gain a tiny bit in the
  // nearly impossible direction, and the next guess is somewhere wild.
  //
  // With it, any direction the arm can only move very slowly in (slower than
  // about `damping` metres or radians per radian of joint turn) is mostly
  // given up on for this step, instead of being chased with large joint
  // turns. The cost: every step falls a little short of the exact one, so
  // more iterations are needed, and a target that does need motion in such
  // a direction (one very close to a singular pose) is approached slowly.
  // The arm's ordinary rates are 0.3 to 3, far above the default, so away
  // from singularities the cost is a fraction of a percent per step.
  //
  // Larger is more cautious: on the tests, 0.05 kept a wrist-singular solve
  // a little closer to its seed but left some targets within a few degrees
  // of joint 5 = 0 short of the tolerance after 200 iterations; 0.01 solved
  // them all.
  //
  // This is the smallest damping used. When a step makes the error larger,
  // it is undone and retried with ten times the damping (shorter, more
  // cautious), and the damping is halved again after each good step.
  double damping = 0.01;

  // No joint turns by more than this in one step, in radians. The whole step
  // is scaled down to respect it, so the step keeps its direction. Small
  // steps keep the search on the branch of solutions nearest the seed, and
  // keep the straight-line guess the Jacobian gives a fair one.
  double max_step = 0.2;

  // The answer is refused if any joint ended up more than this far from the
  // seed, in radians. Off (infinite) by default, since a solve from the home
  // pose to a far target must move far. A planner that solves a sequence of
  // nearby poses sets it to catch the solver jumping to another branch
  // (elbow or wrist flipped) between neighbours; SolveIkPath does.
  double max_distance_from_seed = std::numeric_limits<double>::infinity();
};

// Why a solve ended.
enum class IkStatus {
  kConverged,       // Within both tolerances.
  kBadInput,        // Seed or target not finite, seed not six entries, or
                    // an option out of range (tolerances, damping and steps
                    // must be positive).
  kIterationLimit,  // Still improving when max_iterations ran out.
  kStalled,         // Stopped improving (less than a tenth off the squared
                    // error in 40 iterations, or no step at all helps):
                    // out of reach, or held at a joint limit away from the
                    // target.
  kTooFarFromSeed,  // Converged, but further from the seed than allowed.
};

// How a solve went. Filled in whether or not it succeeded.
struct IkReport {
  IkStatus status = IkStatus::kBadInput;
  // The last guess's errors: how far the tool tip is from the target, and
  // the angle between its orientation and the target's.
  double position_error = 0.0;
  double orientation_error = 0.0;
  // Guesses tried, including rejected ones.
  int iterations = 0;
  // The last guess, within the joint ranges. Only guesses that improved on
  // the one before are kept, so when the solve failed this is the closest
  // it got. Empty for kBadInput.
  Eigen::VectorXd angles;
  // The largest change of any one joint from the seed to `angles`.
  double distance_from_seed = 0.0;
};

// Where the tool tip is with the arm at `angles` (six entries, arm order).
// Writes them into scratch's qpos at the arm's joints and runs
// mj_kinematics; nothing else in scratch is read or written by us.
Eigen::Isometry3d TipPoseAt(const mjModel& model, mjData* scratch,
                            const Arm& arm, const Eigen::VectorXd& angles);

// Joint angles (six entries, arm order) that put tool_tip at `target`, the
// solution reached from `seed` by small steps. A seed outside the joint
// ranges is first moved inside them; every guess is kept inside them.
//
// Returns nothing if it does not get within the tolerances, or the answer is
// further from the seed than options.max_distance_from_seed. Pass `report`
// to learn why and how close it got. On return, scratch holds the last guess
// (report->angles) at the arm's joints, with body and site poses up to date
// as after mj_kinematics.
//
// Deterministic: the same model, target, seed and options give the same
// answer, whatever was in scratch before.
std::optional<Eigen::VectorXd> SolveIk(const mjModel& model, mjData* scratch,
                                       const Arm& arm,
                                       const Eigen::Isometry3d& target,
                                       const Eigen::VectorXd& seed,
                                       const IkOptions& options = {},
                                       IkReport* report = nullptr);

struct IkPathReport {
  // The first waypoint that failed, or -1 if none did.
  int failed_waypoint = -1;
  // The report for that waypoint, or for the last one if all succeeded.
  IkReport waypoint;
  // The largest change of any one joint between neighbouring answers
  // (counting `start` as the answer before the first waypoint), over the
  // waypoints solved.
  double largest_step = 0.0;
};

// Joint angles for each of `poses` in turn, each solved from the answer to
// the one before; the first from `start`, which should be the arm's angles
// at or near poses[0] (within max_joint_step of its answer). Returns
// nothing if any waypoint fails, or if any joint changes by more than
// `max_joint_step` radians between neighbours, which is the sign of a jump
// to another branch (elbow or wrist flipped) rather than a smooth path, or
// of a path passing so close to a singularity that some joint must swing
// fast there. `max_joint_step` has to suit the spacing of the poses: for 1
// cm apart, the cell's straight pull-back from a module face needs at most
// about 0.02 rad. A max_joint_step that is not positive fails at waypoint 0
// with kBadInput.
//
// Only the waypoints are solved. Between them the arm moves in joint space,
// which strays from the straight line by an amount that shrinks with the
// spacing: about 0.07 mm at 1 cm on the cell's pull-back, the solver's own
// tolerance included.
//
// options.max_distance_from_seed is lowered to max_joint_step if larger.
std::optional<std::vector<Eigen::VectorXd>> SolveIkPath(
    const mjModel& model, mjData* scratch, const Arm& arm,
    const Eigen::VectorXd& start, const std::vector<Eigen::Isometry3d>& poses,
    double max_joint_step, const IkOptions& options = {},
    IkPathReport* report = nullptr);

// ------------------------------------------------------- poses to solve for

// The tool-tip pose that grips a flat face: tool_tip on `face_centre`, the
// tool's +Z pointing into the face (along -outward_normal), and the tool's
// X axis along `long_side`. The tool's plate is long along its X, so this
// lays the plate along the face's long side.
//
// `long_side` need not be exactly square to the normal or of length 1: its
// part along the normal is dropped and the rest scaled to length 1. Returns
// nothing if a vector is not finite, the normal has no length, or
// long_side is (nearly) along the normal, so no direction along the face is
// given.
//
// The sign of `long_side`. The plate and its cups are symmetric about the
// tool's centre, so +long_side and -long_side grip the face identically;
// the two poses differ by half a turn of joint 6 (whose range is about
// +-350 degrees, so both are reachable). What the sign decides is which end
// of the held module the tool's +X points at, and the module keeps that
// relation while it is carried. Pass the module's own +x direction (for
// the pallet in models/pallet.xml, world -Y; on the conveyor, Conveyor::
// PlacementPose wants it along world +X), and a place pose built the same
// way puts the module down the right way round.
std::optional<Eigen::Isometry3d> GripPose(const Eigen::Vector3d& face_centre,
                                          const Eigen::Vector3d& outward_normal,
                                          const Eigen::Vector3d& long_side);

// Poses along the straight line from `from` to `to`, both included, evenly
// spaced: no two neighbours more than `spacing` metres apart, nor turned by
// more than `angle_spacing` radians. Position moves along the straight
// line at a steady rate; orientation turns about one fixed axis at a steady
// rate (spherical linear interpolation, "slerp", the shorter way round).
// The first pose is exactly `from` and the last exactly `to`. When the two
// are equal, that is the whole list: [from, to].
//
// Feed it to SolveIkPath with the arm's angles at `from` as the start: the
// approach to a face and the pull-back from it are both such lines.
//
// Returns an empty list if `spacing` or `angle_spacing` is not positive or
// a pose is not finite.
std::vector<Eigen::Isometry3d> StraightLine(
    const Eigen::Isometry3d& from, const Eigen::Isometry3d& to,
    double spacing, double angle_spacing = std::numbers::pi / 180.0);

}  // namespace cell

#endif  // SRC_IK_H_
