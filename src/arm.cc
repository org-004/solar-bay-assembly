#include "src/arm.h"

#include <string>

namespace cell {
namespace {

// KUKA's limits for the KR 210 R2700-2, joint 1 to joint 6. Velocity is from
// the robot description (kuka_quantec_support, kr210_r2700_2), acceleration
// from its joint_limits.yaml, at the commit pinned in MODULE.bazel.
constexpr double kVelocity[Arm::kJoints] = {2.0384,  1.9451,  1.95477,
                                            3.12414, 3.00197, 3.83972};
constexpr double kAcceleration[Arm::kJoints] = {1.98271, 1.26288, 2.56508,
                                                7.03857, 1.76397, 1.87268};

// Jerk limit as a multiple of the acceleration limit: full acceleration is
// reached in 1 / kJerkPerAcceleration seconds.
constexpr double kJerkPerAcceleration = 10.0;

}  // namespace

std::optional<Arm> FindArm(const mjModel& model) {
  Arm arm;
  arm.lower.resize(Arm::kJoints);
  arm.upper.resize(Arm::kJoints);
  arm.torque_limit.resize(Arm::kJoints);
  arm.lag.resize(Arm::kJoints);
  arm.limits.velocity.resize(Arm::kJoints);
  arm.limits.acceleration.resize(Arm::kJoints);
  arm.limits.jerk.resize(Arm::kJoints);

  for (int i = 0; i < Arm::kJoints; ++i) {
    const std::string number = std::to_string(i + 1);
    const int joint =
        mj_name2id(&model, mjOBJ_JOINT, ("arm_joint_" + number).c_str());
    const int motor =
        mj_name2id(&model, mjOBJ_ACTUATOR, ("arm_motor_" + number).c_str());
    if (joint < 0 || motor < 0) return std::nullopt;

    // The motor must drive this joint, and be a position servo: its force
    // is gain * command + bias[1] * angle + bias[2] * speed, with
    // bias[1] = -gain. That is what "push in proportion to the error, brake
    // in proportion to the speed" looks like in MuJoCo's arrays.
    const mjtNum gain = model.actuator_gainprm[mjNGAIN * motor];
    const mjtNum* bias = &model.actuator_biasprm[mjNBIAS * motor];
    if (model.actuator_trntype[motor] != mjTRN_JOINT ||
        model.actuator_trnid[2 * motor] != joint || !(gain > 0.0) ||
        bias[1] != -gain) {
      return std::nullopt;
    }

    arm.qpos[i] = model.jnt_qposadr[joint];
    arm.dof[i] = model.jnt_dofadr[joint];
    arm.motor[i] = motor;
    arm.lower[i] = model.jnt_range[2 * joint];
    arm.upper[i] = model.jnt_range[2 * joint + 1];
    arm.torque_limit[i] = model.actuator_forcerange[2 * motor + 1];

    // The servo's own brake and the joint's friction both act on speed.
    const mjtNum brake = -bias[2] + model.dof_damping[arm.dof[i]];
    arm.lag[i] = brake / gain;

    arm.limits.velocity[i] = kVelocity[i];
    arm.limits.acceleration[i] = kAcceleration[i];
    arm.limits.jerk[i] = kJerkPerAcceleration * kAcceleration[i];
  }

  arm.tool_body = mj_name2id(&model, mjOBJ_BODY, "tool");
  arm.tip_site = mj_name2id(&model, mjOBJ_SITE, "tool_tip");
  if (arm.tool_body < 0 || arm.tip_site < 0) return std::nullopt;
  return arm;
}

Eigen::VectorXd JointAngles(const Arm& arm, const mjData& data) {
  Eigen::VectorXd q(Arm::kJoints);
  for (int i = 0; i < Arm::kJoints; ++i) q[i] = data.qpos[arm.qpos[i]];
  return q;
}

Eigen::VectorXd JointSpeeds(const Arm& arm, const mjData& data) {
  Eigen::VectorXd v(Arm::kJoints);
  for (int i = 0; i < Arm::kJoints; ++i) v[i] = data.qvel[arm.dof[i]];
  return v;
}

Eigen::Isometry3d TipPose(const Arm& arm, const mjData& data) {
  // MuJoCo stores a site's position as three numbers and its orientation as
  // a 3x3 matrix written out row by row.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
      Eigen::Map<const Eigen::Vector3d>(&data.site_xpos[3 * arm.tip_site]);
  pose.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
      &data.site_xmat[9 * arm.tip_site]);
  return pose;
}

}  // namespace cell
