#include "src/ik.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Cholesky>

namespace cell {
namespace {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// When a step makes the error larger, the damping is multiplied by
// kDampingUp and the step tried again. Past kMaxDamping the steps are so
// short that nothing useful is left to try: the solve has stalled.
constexpr double kDampingUp = 10.0;
constexpr double kDampingDown = 0.5;
constexpr double kMaxDamping = 1e3;

// A solve that is still going after kWindow iterations must have cut its
// squared error by at least a tenth over the last kWindow, or it has
// stalled. The usual way to stall is a joint pressed against its limit with
// the target on the far side of it: each step still gains a little by
// sliding along the limit, but only ever a little, and the iterations would
// run out without getting there. A target out of reach is the same: the tip
// creeps toward the nearest point it can reach. Converging, even slowly
// near a singularity, gains far more than this. (Tried on the tests: a
// window of 25 with a gain of a half cut real solves short at the wrist
// singularity.)
constexpr int kWindow = 40;
constexpr double kWindowGain = 0.9;

void WriteAngles(const Arm& arm, const Eigen::VectorXd& angles,
                 mjData* scratch) {
  for (int i = 0; i < Arm::kJoints; ++i) {
    scratch->qpos[arm.qpos[i]] = angles[i];
  }
}

// Forward kinematics plus the Jacobian, at `angles`.
//
// mj_kinematics computes every body's and site's pose from qpos, which is
// all TipPose needs. mj_jacSite also needs two things that mj_kinematics
// leaves stale: each joint's axis of motion in world coordinates (cdof),
// and each body tree's centre of mass (subtree_com), which MuJoCo measures
// those axes about. Both come from mj_comPos, which uses only what
// mj_kinematics just wrote. mj_forward would compute these too, but also
// collisions, forces and accelerations, which the solver does not need.
// (Checked against MuJoCo 4.0.0's source: mj_jac reads cdof, subtree_com
// and, for the point, site_xpos.) With MuJoCo's sleep option on, both calls
// would skip sleeping bodies; the cell's scenes do not turn it on.
//
// The Jacobian is the 6 x 6 table described in ik.h: column i says how the
// tip moves (rows 0 to 2, metres per radian) and turns (rows 3 to 5,
// radians per radian, as an axis times a rate) when joint i alone turns.
struct Kinematics {
  Eigen::Isometry3d tip;
  Matrix6d jacobian;  // rows: tip velocity x, y, z, then turning rate x, y, z
};

Kinematics Evaluate(const mjModel& model, mjData* scratch, const Arm& arm,
                    const Eigen::VectorXd& angles, std::vector<mjtNum>* jacp,
                    std::vector<mjtNum>* jacr) {
  WriteAngles(arm, angles, scratch);
  mj_kinematics(&model, scratch);
  mj_comPos(&model, scratch);

  // MuJoCo's Jacobian has one column per degree of freedom in the whole
  // model: a free-floating module has six, the conveyor one. Only the arm's
  // six columns matter here; the others say how the tip would move if a
  // module moved, which is not at all.
  mj_jacSite(&model, scratch, jacp->data(), jacr->data(), arm.tip_site);

  Kinematics k;
  k.tip = TipPose(arm, *scratch);
  const int nv = model.nv;
  for (int i = 0; i < Arm::kJoints; ++i) {
    const int column = arm.dof[i];
    for (int row = 0; row < 3; ++row) {
      k.jacobian(row, i) = (*jacp)[row * nv + column];
      k.jacobian(row + 3, i) = (*jacr)[row * nv + column];
    }
  }
  return k;
}

// How far the tip is from the target, as six numbers: three for position
// (target minus tip, metres) and three for orientation.
//
// The orientation error is a rotation vector: the single turn that takes
// the tip's orientation to the target's, written as an arrow whose direction
// is the axis to turn about and whose length is the angle to turn, in
// radians. Turning rate is the same kind of quantity (an axis times radians
// per second), which is what the Jacobian's lower three rows give, so the
// two can be compared directly.
Vector6d PoseError(const Eigen::Isometry3d& target,
                   const Eigen::Isometry3d& tip) {
  Vector6d error;
  error.head<3>() = target.translation() - tip.translation();
  // The turn that, applied after the tip's orientation, gives the target's.
  const Eigen::AngleAxisd turn(target.linear() * tip.linear().transpose());
  error.tail<3>() = turn.angle() * turn.axis();
  return error;
}

bool Converged(const Vector6d& error, const IkOptions& options) {
  return error.head<3>().norm() <= options.position_tolerance &&
         error.tail<3>().norm() <= options.orientation_tolerance;
}

Eigen::VectorXd Clamp(const Arm& arm, const Eigen::VectorXd& angles) {
  return angles.cwiseMax(arm.lower).cwiseMin(arm.upper);
}

bool ValidOptions(const IkOptions& o) {
  return o.position_tolerance > 0.0 && o.orientation_tolerance > 0.0 &&
         o.max_iterations >= 0 && o.damping > 0.0 && std::isfinite(o.damping) &&
         o.max_step > 0.0 && std::isfinite(o.max_step) &&
         o.max_distance_from_seed > 0.0;
}

}  // namespace

Eigen::Isometry3d TipPoseAt(const mjModel& model, mjData* scratch,
                            const Arm& arm, const Eigen::VectorXd& angles) {
  WriteAngles(arm, angles, scratch);
  mj_kinematics(&model, scratch);
  return TipPose(arm, *scratch);
}

std::optional<Eigen::VectorXd> SolveIk(const mjModel& model, mjData* scratch,
                                       const Arm& arm,
                                       const Eigen::Isometry3d& target,
                                       const Eigen::VectorXd& seed,
                                       const IkOptions& options,
                                       IkReport* report) {
  IkReport local;
  IkReport& r = report != nullptr ? *report : local;
  r = IkReport();

  if (seed.size() != Arm::kJoints || !seed.allFinite() ||
      !target.matrix().allFinite() || !ValidOptions(options)) {
    r.status = IkStatus::kBadInput;
    return std::nullopt;
  }

  std::vector<mjtNum> jacp(3 * model.nv);
  std::vector<mjtNum> jacr(3 * model.nv);

  // Joint ranges. Every guess is clamped into them: a step that would carry
  // a joint past its limit stops it at the limit instead, and the other
  // joints' parts of the step still happen. Clamping changes the step's
  // direction, so near a limit the search is no longer the plain damped
  // least-squares step and converges more slowly. If the solution near the
  // seed needs a joint beyond its range, the search presses against the
  // limit, cannot improve, and reports kStalled (or, rarely, slides along
  // the limit to another solution that is within range).
  Eigen::VectorXd q = Clamp(arm, seed);
  Kinematics k = Evaluate(model, scratch, arm, q, &jacp, &jacr);
  Vector6d error = PoseError(target, k.tip);
  // One number for "how wrong": the sum of squares of all six errors, so 1
  // mm of position counts the same as 1 mrad of orientation (0.06 deg). With
  // a tool 0.5 m from the wrist and an arm 2.7 m long, a milliradian of
  // turn moves the tip about a millimetre, so neither swamps the other.
  double cost = error.squaredNorm();

  double damping = options.damping;
  bool scratch_at_q = true;
  // The squared error after each iteration, for the stall check.
  std::vector<double> history;
  r.status = IkStatus::kIterationLimit;

  while (true) {
    if (Converged(error, options)) {
      r.status = IkStatus::kConverged;
      break;
    }
    if (r.iterations >= options.max_iterations) break;

    // The damped least-squares step: the joint change dq that makes
    //   |J dq - error|^2 + damping^2 |dq|^2
    // smallest. The first term asks the tip to move by the error; the
    // second keeps the joint change modest (see IkOptions::damping). Its
    // solution is dq = J^T (J J^T + damping^2 I)^-1 error, which stays
    // finite even where J cannot be inverted.
    const Matrix6d& j = k.jacobian;
    const Matrix6d a =
        j * j.transpose() + damping * damping * Matrix6d::Identity();
    Vector6d dq = j.transpose() * a.ldlt().solve(error);

    // The step-size limit: scale the whole step down so no joint turns by
    // more than max_step.
    const double largest = dq.cwiseAbs().maxCoeff();
    if (largest > options.max_step) dq *= options.max_step / largest;

    const Eigen::VectorXd q_new = Clamp(arm, q + dq);
    ++r.iterations;
    const Kinematics k_new =
        Evaluate(model, scratch, arm, q_new, &jacp, &jacr);
    const Vector6d error_new = PoseError(target, k_new.tip);
    const double cost_new = error_new.squaredNorm();

    if (cost_new < cost) {
      q = q_new;
      k = k_new;
      error = error_new;
      cost = cost_new;
      scratch_at_q = true;
      damping = std::max(options.damping, damping * kDampingDown);
    } else {
      // The straight-line guess the Jacobian makes was wrong over this
      // distance, or a joint limit is in the way. Undo, and try a shorter,
      // more cautious step.
      scratch_at_q = false;
      damping *= kDampingUp;
      if (damping > kMaxDamping) {
        r.status = IkStatus::kStalled;
        break;
      }
    }

    history.push_back(cost);
    const int n = static_cast<int>(history.size());
    if (n > kWindow && !Converged(error, options) &&
        cost > kWindowGain * history[n - 1 - kWindow]) {
      r.status = IkStatus::kStalled;
      break;
    }
  }

  if (!scratch_at_q) TipPoseAt(model, scratch, arm, q);

  r.position_error = error.head<3>().norm();
  r.orientation_error = error.tail<3>().norm();
  r.angles = q;
  r.distance_from_seed = (q - seed).cwiseAbs().maxCoeff();

  if (r.status != IkStatus::kConverged) return std::nullopt;
  if (r.distance_from_seed > options.max_distance_from_seed) {
    r.status = IkStatus::kTooFarFromSeed;
    return std::nullopt;
  }
  return q;
}

std::optional<std::vector<Eigen::VectorXd>> SolveIkPath(
    const mjModel& model, mjData* scratch, const Arm& arm,
    const Eigen::VectorXd& start, const std::vector<Eigen::Isometry3d>& poses,
    double max_joint_step, const IkOptions& options, IkPathReport* report) {
  IkPathReport local;
  IkPathReport& r = report != nullptr ? *report : local;
  r = IkPathReport();

  if (!(max_joint_step > 0.0)) {
    r.failed_waypoint = 0;
    r.waypoint.status = IkStatus::kBadInput;
    return std::nullopt;
  }

  // Each waypoint may be no further from the previous answer than one
  // allowed step, so a jump to another branch fails as kTooFarFromSeed.
  IkOptions step_options = options;
  step_options.max_distance_from_seed =
      std::min(options.max_distance_from_seed, max_joint_step);

  std::vector<Eigen::VectorXd> path;
  path.reserve(poses.size());
  Eigen::VectorXd previous = start;
  for (int i = 0; i < static_cast<int>(poses.size()); ++i) {
    const std::optional<Eigen::VectorXd> angles = SolveIk(
        model, scratch, arm, poses[i], previous, step_options, &r.waypoint);
    if (!angles) {
      r.failed_waypoint = i;
      return std::nullopt;
    }
    r.largest_step = std::max(r.largest_step, r.waypoint.distance_from_seed);
    previous = *angles;
    path.push_back(*angles);
  }
  return path;
}

std::optional<Eigen::Isometry3d> GripPose(const Eigen::Vector3d& face_centre,
                                          const Eigen::Vector3d& outward_normal,
                                          const Eigen::Vector3d& long_side) {
  if (!face_centre.allFinite() || !outward_normal.allFinite() ||
      !long_side.allFinite()) {
    return std::nullopt;
  }
  const double normal_length = outward_normal.norm();
  if (!(normal_length > 0.0)) return std::nullopt;
  const Eigen::Vector3d z = -outward_normal / normal_length;
  // The part of long_side that lies in the face. If almost nothing is left,
  // long_side was along the normal and says nothing about the face.
  const Eigen::Vector3d in_face = long_side - long_side.dot(z) * z;
  const double in_face_length = in_face.norm();
  if (!(in_face_length > 1e-9 * long_side.norm())) return std::nullopt;
  const Eigen::Vector3d x = in_face / in_face_length;

  // A frame's axes are the columns of its rotation; Y completes it so that
  // X, Y, Z are right-handed, as every frame in MuJoCo is.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear().col(0) = x;
  pose.linear().col(1) = z.cross(x);
  pose.linear().col(2) = z;
  pose.translation() = face_centre;
  return pose;
}

std::vector<Eigen::Isometry3d> StraightLine(const Eigen::Isometry3d& from,
                                            const Eigen::Isometry3d& to,
                                            double spacing,
                                            double angle_spacing) {
  if (!(spacing > 0.0) || !(angle_spacing > 0.0) ||
      !from.matrix().allFinite() || !to.matrix().allFinite()) {
    return {};
  }
  const Eigen::Quaterniond q_from(from.linear());
  const Eigen::Quaterniond q_to(to.linear());
  const double distance = (to.translation() - from.translation()).norm();
  const double angle = q_from.angularDistance(q_to);

  // Enough equal pieces that neither limit is broken. The small allowance
  // keeps an exact multiple (0.3 m at 0.01 m) from gaining a piece through
  // rounding.
  const double pieces = std::max({1.0, std::ceil(distance / spacing - 1e-9),
                                  std::ceil(angle / angle_spacing - 1e-9)});
  const int n = static_cast<int>(pieces);

  std::vector<Eigen::Isometry3d> poses;
  poses.reserve(n + 1);
  poses.push_back(from);
  for (int k = 1; k < n; ++k) {
    const double t = static_cast<double>(k) / n;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() =
        (1.0 - t) * from.translation() + t * to.translation();
    // Eigen's slerp takes the shorter way round.
    pose.linear() = q_from.slerp(t, q_to).toRotationMatrix();
    poses.push_back(pose);
  }
  poses.push_back(to);
  return poses;
}

}  // namespace cell
