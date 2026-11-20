#include "src/arm_follower.h"

#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <variant>

namespace cell {

double ArmFollower::Leg::duration() const {
  return std::visit([](const auto& m) { return m.duration(); }, move);
}

JointSample ArmFollower::Leg::At(double t) const {
  return std::visit([t](const auto& m) { return m.At(t); }, move);
}

ArmFollower::ArmFollower(const Arm& arm, const FollowerSettings& settings)
    : arm_(arm), settings_(settings) {}

void ArmFollower::Adopt(const mjData& data) {
  if (adopted_) return;
  held_ = HeldPose(data);
  adopted_ = true;
}

Eigen::VectorXd ArmFollower::HeldPose(const mjData& data) const {
  if (adopted_) return held_;
  Eigen::VectorXd q(Arm::kJoints);
  for (int i = 0; i < Arm::kJoints; ++i) q[i] = data.ctrl[arm_.motor[i]];
  return q;
}

std::string ArmFollower::CheckGoal(const Eigen::VectorXd& goal) const {
  if (goal.size() != Arm::kJoints) {
    return "goal has " + std::to_string(goal.size()) + " angles, not " +
           std::to_string(Arm::kJoints);
  }
  for (int i = 0; i < Arm::kJoints; ++i) {
    const std::string joint = "joint " + std::to_string(i + 1);
    if (!std::isfinite(goal[i])) return joint + " is not finite";
    if (goal[i] < arm_.lower[i] || goal[i] > arm_.upper[i]) {
      return joint + " at " + std::to_string(goal[i]) + " is outside [" +
             std::to_string(arm_.lower[i]) + ", " +
             std::to_string(arm_.upper[i]) + "]";
    }
  }
  return "";
}

bool ArmFollower::MoveTo(const Eigen::VectorXd& goal, const mjData& data,
                         const MoveOptions& options, std::string* reason) {
  return MoveThrough({goal}, data, options, reason);
}

bool ArmFollower::MoveThrough(const std::vector<Eigen::VectorXd>& waypoints,
                              const mjData& data, const MoveOptions& options,
                              std::string* reason) {
  auto refuse = [reason](std::string why) {
    if (reason != nullptr) *reason = std::move(why);
    return false;
  };

  if (waypoints.empty()) return refuse("no waypoints");
  if (!(options.speed > 0.0 && options.speed <= 1.0)) {
    return refuse("speed " + std::to_string(options.speed) +
                  " is not in (0, 1]");
  }
  if (TimeLeft(data) > 0.0) return refuse("a move is still running");
  for (size_t k = 0; k < waypoints.size(); ++k) {
    const std::string why = CheckGoal(waypoints[k]);
    if (!why.empty()) {
      return refuse("waypoint " + std::to_string(k) + ": " + why);
    }
  }

  // Playing the full-speed move slower by `speed` scales its velocity by
  // speed, its acceleration by speed^2 and its jerk by speed^3.
  const double s = options.speed;
  JointLimits limits = arm_.limits;
  limits.velocity *= s;
  limits.acceleration *= s * s;
  limits.jerk *= s * s * s;

  // Every leg starts where the one before ends, and the first where the
  // arm is commanded to be now.
  std::vector<Leg> legs;
  Eigen::VectorXd from = Goal(data);
  double start_time = data.time;
  for (size_t k = 0; k < waypoints.size(); ++k) {
    std::optional<JointMove> move =
        JointMove::Plan(from, waypoints[k], limits, options.shape);
    if (!move) {
      return refuse("waypoint " + std::to_string(k) + ": could not plan");
    }
    legs.push_back({*move, start_time});
    start_time += move->duration();
    from = waypoints[k];
  }

  Adopt(data);
  held_ = from;
  legs_ = std::move(legs);
  return true;
}

bool ArmFollower::MoveAlong(const std::vector<Eigen::VectorXd>& waypoints,
                            const mjData& data, const MoveOptions& options,
                            std::string* reason) {
  auto refuse = [reason](std::string why) {
    if (reason != nullptr) *reason = std::move(why);
    return false;
  };

  if (waypoints.empty()) return refuse("no waypoints");
  if (!(options.speed > 0.0 && options.speed <= 1.0)) {
    return refuse("speed " + std::to_string(options.speed) +
                  " is not in (0, 1]");
  }
  if (!(options.max_deviation >= 0.0)) {
    return refuse("max_deviation " + std::to_string(options.max_deviation) +
                  " is not zero or more");
  }
  if (TimeLeft(data) > 0.0) return refuse("a move is still running");
  for (size_t k = 0; k < waypoints.size(); ++k) {
    const std::string why = CheckGoal(waypoints[k]);
    if (!why.empty()) {
      return refuse("waypoint " + std::to_string(k) + ": " + why);
    }
  }

  // From where the arm is commanded to be now, as MoveThrough's first leg.
  std::vector<Eigen::VectorXd> path = {Goal(data)};
  path.insert(path.end(), waypoints.begin(), waypoints.end());
  std::optional<Trajectory> trajectory = Trajectory::Plan(
      path, arm_.limits,
      {.max_deviation = options.max_deviation, .speed = options.speed});
  if (!trajectory) return refuse("could not plan");
  return Follow(*trajectory, data, reason);
}

bool ArmFollower::Follow(const Trajectory& trajectory, const mjData& data,
                         std::string* reason) {
  auto refuse = [reason](std::string why) {
    if (reason != nullptr) *reason = std::move(why);
    return false;
  };

  if (TimeLeft(data) > 0.0) return refuse("a move is still running");
  // Checking the waypoints is enough: the path's straight parts and blends
  // lie between them, so inside the box of joint ranges if they are.
  const std::vector<Eigen::VectorXd>& waypoints = trajectory.waypoints();
  for (size_t k = 0; k < waypoints.size(); ++k) {
    const std::string why = CheckGoal(waypoints[k]);
    if (!why.empty()) {
      return refuse("waypoint " + std::to_string(k) + ": " + why);
    }
  }
  const double gap = (waypoints.front() - Goal(data)).cwiseAbs().maxCoeff();
  if (gap > 1e-9) {
    return refuse("the trajectory starts " + std::to_string(gap) +
                  " rad from the commanded pose");
  }

  Adopt(data);
  held_ = waypoints.back();
  legs_.clear();
  legs_.push_back({trajectory, data.time});
  return true;
}

void ArmFollower::Hold(const mjData& data) {
  adopted_ = true;
  held_ = JointAngles(arm_, data);
  legs_.clear();
}

void ArmFollower::Update(const mjModel& model, mjData* data) {
  Adopt(*data);
  // Once the plan has ended, only its goal matters: held_ already is that.
  if (!legs_.empty() && TimeLeft(*data) == 0.0) legs_.clear();

  const JointSample plan = Planned(*data);
  Eigen::VectorXd command = plan.position;
  if (settings_.lag_compensation) {
    command += arm_.lag.cwiseProduct(plan.velocity);
  }

  // The servo pushes kp * (command - angle), so asking for `torque` more
  // means commanding torque / kp further on. M * planned acceleration
  // includes the coupling between joints; qfrc_bias is gravity plus the
  // Coriolis and centrifugal torques, at the state of the last step.
  if (settings_.model_feedforward) {
    acceleration_.setZero(model.nv);
    torque_.setZero(model.nv);
    for (int i = 0; i < Arm::kJoints; ++i) {
      acceleration_[arm_.dof[i]] = plan.acceleration[i];
    }
    mj_mulM(&model, data, torque_.data(), acceleration_.data());
    for (int i = 0; i < Arm::kJoints; ++i) {
      const int dof = arm_.dof[i];
      const double kp = model.actuator_gainprm[mjNGAIN * arm_.motor[i]];
      command[i] += (torque_[dof] + data->qfrc_bias[dof]) / kp;
    }
  }

  for (int i = 0; i < Arm::kJoints; ++i) data->ctrl[arm_.motor[i]] = command[i];
  commanded_ = command;
}

bool ArmFollower::Done(const mjData& data) const {
  if (TimeLeft(data) > 0.0) return false;
  const Eigen::VectorXd error = JointAngles(arm_, data) - Goal(data);
  return error.cwiseAbs().maxCoeff() < kArrivedAngle &&
         JointSpeeds(arm_, data).cwiseAbs().maxCoeff() < kArrivedSpeed;
}

double ArmFollower::TimeLeft(const mjData& data) const {
  if (legs_.empty()) return 0.0;
  const Leg& last = legs_.back();
  const double end = last.start_time + last.duration();
  return data.time < end ? end - data.time : 0.0;
}

JointSample ArmFollower::Planned(const mjData& data) const {
  for (const Leg& leg : legs_) {
    if (data.time < leg.start_time + leg.duration()) {
      return leg.At(data.time - leg.start_time);
    }
  }
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(Arm::kJoints);
  return {HeldPose(data), zero, zero, zero};
}

Eigen::VectorXd ArmFollower::Goal(const mjData& data) const {
  return HeldPose(data);
}

Eigen::VectorXd ArmFollower::TrackingError(const mjData& data) const {
  return JointAngles(arm_, data) - Planned(data).position;
}

}  // namespace cell
