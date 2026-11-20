// The arm as the rest of the code sees it: where its joints, motors and tool
// are in a loaded model, and how fast each joint may be driven.
//
// MuJoCo keeps every joint of every body in a few long arrays. The indices
// here say which entries are the arm's, so that code can read and command
// the arm in a scene that also holds a pallet of modules and a conveyor.
//
// Joint order is the arm's own, base to wrist: entry 0 is arm_joint_1.
// Angles are radians, with KUKA's zero and KUKA's signs; see models/arm.xml.

#ifndef SRC_ARM_H_
#define SRC_ARM_H_

#include <array>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/joint_move.h"

namespace cell {

struct Arm {
  static constexpr int kJoints = 6;

  // Where each joint's angle is in mjData::qpos, its speed in mjData::qvel,
  // and its motor's command in mjData::ctrl.
  std::array<int, kJoints> qpos{};
  std::array<int, kJoints> dof{};
  std::array<int, kJoints> motor{};

  int tool_body = -1;  // body "tool"
  int tip_site = -1;   // site "tool_tip", where the cups meet the glass

  // The range each joint can turn through.
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;

  // The fastest each joint may be driven. Velocity and acceleration are
  // KUKA's figures for the KR 210 R2700-2. KUKA gives no jerk limit; the
  // jerk here is chosen so that acceleration takes a tenth of a second to
  // build, and is ours to change.
  JointLimits limits;

  // The most torque each motor gives, N m, from the model.
  Eigen::VectorXd torque_limit;

  // How far behind a moving command each joint runs, in seconds. A motor
  // pushes in proportion to the angle error and is braked in proportion to
  // the joint's speed, so at a steady speed the joint settles a fixed time
  // behind the command: brake gain over push gain. Commanding
  //   angle + lag * speed
  // cancels it.
  Eigen::VectorXd lag;
};

// Finds the arm in a model. Returns nothing if a joint, a motor, the tool or
// its tip is missing, or a motor is not a position servo on its own joint.
std::optional<Arm> FindArm(const mjModel& model);

// The arm's joint angles and speeds now.
Eigen::VectorXd JointAngles(const Arm& arm, const mjData& data);
Eigen::VectorXd JointSpeeds(const Arm& arm, const mjData& data);

// Where the tool tip is in the world, as of the last mj_forward, mj_step or
// mj_kinematics. Its +Z points out of the tool, toward what it grips.
Eigen::Isometry3d TipPose(const Arm& arm, const mjData& data);

}  // namespace cell

#endif  // SRC_ARM_H_
