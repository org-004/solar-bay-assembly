#include "src/conveyor.h"

#include <cmath>
#include <optional>
#include <random>

#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/profile.h"

namespace cell {
namespace {

bool IsPositiveAndFinite(double x) { return std::isfinite(x) && x > 0.0; }

// The id of the sensor `name` if it is of `type` and reads `joint`.
std::optional<int> FindJointSensor(const mjModel& model, const char* name,
                                   mjtSensor type, int joint) {
  const int id = mj_name2id(&model, mjOBJ_SENSOR, name);
  if (id < 0 || model.sensor_type[id] != type ||
      model.sensor_objtype[id] != mjOBJ_JOINT ||
      model.sensor_objid[id] != joint) {
    return std::nullopt;
  }
  return id;
}

}  // namespace

Eigen::Isometry3d Conveyor::PlacementPose() {
  return SlotPose(0, NominalSlidePosition(0));
}

Eigen::Isometry3d Conveyor::SlotPoseOnTube(int slot) {
  // The module's underside on the rails' top surface, which is the tube
  // frame's z = 0, so its centre is half a thickness up.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(
      0.0, -kTravelDirection * slot * kStepLength, kModuleThickness / 2.0);
  return pose;
}

Eigen::Isometry3d Conveyor::SlotPose(int slot, double slide_position) {
  // The tube's frame is at (kTubeAxisX, 0, kRailTopHeight) with the slide at
  // 0, and the slide moves it along Y without turning it.
  Eigen::Isometry3d tube = Eigen::Isometry3d::Identity();
  tube.translation() =
      Eigen::Vector3d(kTubeAxisX, slide_position, kRailTopHeight);
  return tube * SlotPoseOnTube(slot);
}

double Conveyor::NominalSlidePosition(int steps) {
  return kTravelDirection * steps * kStepLength;
}

std::optional<Conveyor> Conveyor::Find(const mjModel& model,
                                       const ConveyorSettings& settings,
                                       std::mt19937::result_type seed) {
  const Limits& limits = settings.limits;
  if (!IsPositiveAndFinite(limits.velocity) ||
      !IsPositiveAndFinite(limits.acceleration) ||
      !IsPositiveAndFinite(limits.jerk) ||
      !std::isfinite(settings.stop_error_stddev) ||
      settings.stop_error_stddev < 0.0 ||
      !IsPositiveAndFinite(settings.in_position_tolerance) ||
      !IsPositiveAndFinite(settings.in_position_speed)) {
    return std::nullopt;
  }

  const int joint = mj_name2id(&model, mjOBJ_JOINT, "conveyor_slide");
  if (joint < 0 || model.jnt_type[joint] != mjJNT_SLIDE) return std::nullopt;

  // The motor has to drive that joint, or its ctrl would move something else.
  const int actuator = mj_name2id(&model, mjOBJ_ACTUATOR, "conveyor_motor");
  if (actuator < 0 || model.actuator_trntype[actuator] != mjTRN_JOINT ||
      model.actuator_trnid[2 * actuator] != joint) {
    return std::nullopt;
  }

  const std::optional<int> position =
      FindJointSensor(model, "conveyor_position", mjSENS_JOINTPOS, joint);
  const std::optional<int> velocity =
      FindJointSensor(model, "conveyor_velocity", mjSENS_JOINTVEL, joint);
  if (!position || !velocity) return std::nullopt;

  const int tube = mj_name2id(&model, mjOBJ_BODY, "tube");
  if (tube < 0 || model.jnt_bodyid[joint] != tube) return std::nullopt;

  Conveyor conveyor;
  conveyor.settings_ = settings;
  conveyor.random_.seed(seed);
  conveyor.model_ = &model;
  conveyor.actuator_ = actuator;
  conveyor.qpos_address_ = model.jnt_qposadr[joint];
  conveyor.dof_address_ = model.jnt_dofadr[joint];
  conveyor.position_sensor_address_ = model.sensor_adr[*position];
  conveyor.velocity_sensor_address_ = model.sensor_adr[*velocity];
  conveyor.tube_body_ = tube;
  conveyor.range_limited_ = model.jnt_limited[joint];
  conveyor.range_low_ = model.jnt_range[2 * joint];
  conveyor.range_high_ = model.jnt_range[2 * joint + 1];
  // Hold where the model starts.
  conveyor.start_position_ = model.qpos0[conveyor.qpos_address_];
  conveyor.target_ = conveyor.start_position_;
  return conveyor;
}

bool Conveyor::StartStep(const mjData& data) {
  if (Moving(data)) return false;

  // An absolute position, not one step on from the last stop: the encoder
  // knows where the tube is, so last time's error is not carried forward.
  // Checked before the error is drawn, so a refused step leaves the random
  // sequence as it was.
  const double exact = start_position_ + NominalSlidePosition(steps_ + 1);
  if (range_limited_ && (exact < range_low_ || exact > range_high_)) {
    return false;
  }

  double error = 0.0;
  if (settings_.stop_error_stddev > 0.0) {
    std::normal_distribution<double> stop_error(0.0,
                                                settings_.stop_error_stddev);
    error = stop_error(random_);
  }

  // The drive is told to go to the off-target spot: it is the real stop.
  const double stop = exact + error;
  std::optional<Profile> move = Profile::SCurve(stop - target_,
                                                settings_.limits);
  if (!move) return false;

  move_ = *move;
  move_start_time_ = data.time;
  move_start_position_ = target_;
  target_ = stop;
  last_error_ = error;
  ++steps_;
  return true;
}

bool Conveyor::SwapTube(mjData* data) {
  if (Moving(*data)) return false;

  data->qpos[qpos_address_] = start_position_;
  data->qvel[dof_address_] = 0.0;
  data->ctrl[actuator_] = start_position_;
  // Recompute positions and sensor readings for the new state.
  mj_forward(model_, data);

  move_.reset();
  target_ = start_position_;
  last_error_ = 0.0;
  steps_ = 0;
  ++tubes_completed_;
  return true;
}

void Conveyor::Update(mjData* data) const {
  double setpoint = target_;
  if (move_) {
    // At(t) is exactly the move's distance once t is past its end.
    const Sample now = move_->At(data->time - move_start_time_);
    setpoint = move_start_position_ + now.position;
  }
  data->ctrl[actuator_] = setpoint;
}

bool Conveyor::Moving(const mjData& data) const {
  if (move_ && data.time < move_start_time_ + move_->duration()) return true;
  // The servo lags the setpoint, so wait for the encoder to say it is there.
  return std::abs(SensorPosition(data) - target_) >
             settings_.in_position_tolerance ||
         std::abs(SensorVelocity(data)) > settings_.in_position_speed;
}

double Conveyor::StepDuration() const {
  const std::optional<Profile> step =
      Profile::SCurve(kStepLength, settings_.limits);
  return step ? step->duration() : 0.0;
}

double Conveyor::SensorPosition(const mjData& data) const {
  return data.sensordata[position_sensor_address_];
}

double Conveyor::SensorVelocity(const mjData& data) const {
  return data.sensordata[velocity_sensor_address_];
}

double Conveyor::TrueSlidePosition(const mjData& data) const {
  return data.qpos[qpos_address_];
}

Eigen::Isometry3d Conveyor::TrueTubePose(const mjData& data) const {
  // xpos and xmat are as of the last mj_forward or mj_step. xmat is
  // row-major.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
      Eigen::Map<const Eigen::Vector3d>(data.xpos + 3 * tube_body_);
  pose.linear() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          data.xmat + 9 * tube_body_);
  return pose;
}

Eigen::Isometry3d Conveyor::TrueSlotPose(const mjData& data, int slot) const {
  return TrueTubePose(data) * SlotPoseOnTube(slot);
}

}  // namespace cell
