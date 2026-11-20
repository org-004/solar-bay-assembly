#include "src/check_rig.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/depth_camera.h"
#include "src/held_module_finder.h"
#include "src/mj_handles.h"
#include "src/suction.h"

namespace cell {
namespace {

Eigen::Isometry3d SitePose(const mjData& data, int site) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(data.site_xpos + 3 * site);
  pose.linear() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          data.site_xmat + 9 * site);
  return pose;
}

Eigen::Isometry3d ToIsometry(const Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = pose.position;
  result.linear() = pose.rotation.normalized().toRotationMatrix();
  return result;
}

}  // namespace

Eigen::Isometry3d GripOffset(const Eigen::Vector3d& shift,
                             const Eigen::Vector3d& rotation_vector) {
  Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
  const double angle = rotation_vector.norm();
  if (angle > 0.0) {
    offset.linear() =
        Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
  }
  offset.translation() = shift;
  return offset;
}

Eigen::Isometry3d DrawGripOffset(std::uint32_t seed, double position,
                                 double angle) {
  std::mt19937 rng(seed);
  // Uniform in [-1, 1), from the generator's raw 32-bit output.
  auto draw = [&rng]() { return rng() / 2147483648.0 - 1.0; };
  Eigen::Vector3d shift, turn;
  for (int i = 0; i < 3; ++i) shift[i] = position * draw();
  for (int i = 0; i < 3; ++i) turn[i] = angle * draw();
  return GripOffset(shift, turn);
}

Eigen::Matrix<double, 6, 1> PoseError6(const Eigen::Isometry3d& estimate,
                                       const Eigen::Isometry3d& truth) {
  const Eigen::Isometry3d e = truth.inverse() * estimate;
  const Eigen::AngleAxisd turn(e.linear());
  Eigen::Matrix<double, 6, 1> result;
  result.head<3>() = e.translation();
  result.tail<3>() = turn.angle() * turn.axis();
  return result;
}

std::optional<CheckRig> CheckRig::Load(const std::string& path,
                                       std::string* error) {
  auto fail = [&](const std::string& why) -> std::optional<CheckRig> {
    if (error) *error = why;
    return std::nullopt;
  };
  CheckRig rig;
  std::string message;
  rig.model_ = LoadModel(path, &message);
  if (!rig.model_) return fail("could not load " + path + ": " + message);
  rig.data_ = MakeData(*rig.model_);
  const mjModel* m = rig.model_.get();

  const int carriage = mj_name2id(m, mjOBJ_BODY, "rig_carriage");
  const int roof = mj_name2id(m, mjOBJ_BODY, "rig_ceiling");
  rig.camera_ = mj_name2id(m, mjOBJ_CAMERA, "check_camera");
  rig.tip_site_ = mj_name2id(m, mjOBJ_SITE, "tool_tip");
  if (carriage < 0 || roof < 0 || rig.camera_ < 0 || rig.tip_site_ < 0) {
    return fail("the scene lacks rig_carriage, rig_ceiling, check_camera or "
                "tool_tip");
  }
  rig.carriage_mocap_ = m->body_mocapid[carriage];
  rig.roof_mocap_ = m->body_mocapid[roof];
  if (rig.carriage_mocap_ < 0 || rig.roof_mocap_ < 0) {
    return fail("rig_carriage and rig_ceiling must be mocap bodies");
  }
  for (int i = 0;; ++i) {
    const std::string name = "module_" + std::to_string(i);
    const int module = mj_name2id(m, mjOBJ_BODY, name.c_str());
    if (module < 0) break;
    const int joint = m->body_jntadr[module];
    if (joint < 0 || m->jnt_type[joint] != mjJNT_FREE) {
      return fail(name + " has no free joint");
    }
    rig.module_qpos_.push_back(m->jnt_qposadr[joint]);
    rig.module_dof_.push_back(m->jnt_dofadr[joint]);
  }
  if (rig.module_qpos_.empty()) return fail("the scene has no module_0");

  SuctionOptions options;
  options.max_distance = 0.015;
  std::optional<Suction> suction = Suction::Create(*m, options, &message);
  if (!suction) return fail(message);
  rig.suction_.emplace(std::move(*suction));

  mj_forward(m, rig.data_.get());
  rig.nominal_tip_ = SitePose(*rig.data_, rig.tip_site_);
  Eigen::Isometry3d carriage_pose = Eigen::Isometry3d::Identity();
  carriage_pose.translation() = Eigen::Vector3d(rig.data_->xpos + 3 * carriage);
  carriage_pose.linear() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          rig.data_->xmat + 9 * carriage);
  rig.carriage_in_tip_ = rig.nominal_tip_.inverse() * carriage_pose;
  rig.roof_position_ =
      Eigen::Vector3d(rig.data_->mocap_pos + 3 * rig.roof_mocap_);
  return rig;
}

CameraModel CheckRig::camera() const {
  return *CameraModelFromMujoco(*model_, *data_, camera_);
}

void CheckRig::PutTool(const Eigen::Isometry3d& tip) {
  const Eigen::Isometry3d carriage = tip * carriage_in_tip_;
  const Eigen::Quaterniond q(carriage.linear());
  for (int i = 0; i < 3; ++i) {
    data_->mocap_pos[3 * carriage_mocap_ + i] = carriage.translation()[i];
  }
  data_->mocap_quat[4 * carriage_mocap_ + 0] = q.w();
  data_->mocap_quat[4 * carriage_mocap_ + 1] = q.x();
  data_->mocap_quat[4 * carriage_mocap_ + 2] = q.y();
  data_->mocap_quat[4 * carriage_mocap_ + 3] = q.z();
}

void CheckRig::PutModule(int module, const Eigen::Isometry3d& pose) {
  const Eigen::Quaterniond q(pose.linear());
  mjtNum* qpos = data_->qpos + module_qpos_[module];
  for (int i = 0; i < 3; ++i) qpos[i] = pose.translation()[i];
  qpos[3] = q.w();
  qpos[4] = q.x();
  qpos[5] = q.y();
  qpos[6] = q.z();
  for (int i = 0; i < 6; ++i) data_->qvel[module_dof_[module] + i] = 0.0;
}

void CheckRig::SetRoof(bool present) {
  roof_ = present;
  const Eigen::Vector3d at =
      present ? roof_position_ : Eigen::Vector3d(0.0, 0.0, 1000.0);
  for (int i = 0; i < 3; ++i) data_->mocap_pos[3 * roof_mocap_ + i] = at[i];
  mj_forward(model_.get(), data_.get());
}

void CheckRig::Restart() {
  mj_resetData(model_.get(), data_.get());
  suction_->Release(data_.get());
  SetRoof(roof_);
  // Every module on the floor, 3 m apart, 20 m out: nowhere near the view.
  for (int i = 0; i < static_cast<int>(module_qpos_.size()); ++i) {
    Eigen::Isometry3d parked = Eigen::Isometry3d::Identity();
    parked.translation() = Eigen::Vector3d(20.0, 3.0 * i, 0.0175);
    PutModule(i, parked);
  }
}

CheckShot CheckRig::Shoot() {
  CheckShot shot;
  shot.camera = camera();
  shot.tip = SitePose(*data_, tip_site_);
  const auto start = std::chrono::steady_clock::now();
  shot.image = Render(&shot.geoms);
  shot.render_seconds = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - start)
                            .count();
  return shot;
}

DepthImage CheckRig::Render(std::vector<int>* geoms) {
  return RenderDepth(*model_, data_.get(), camera(), {}, geoms);
}

std::optional<CheckShot> CheckRig::Take(const Eigen::Isometry3d& tip,
                                        const Eigen::Isometry3d& offset,
                                        std::string* error, double settle,
                                        int module) {
  if (module < 0 || module >= static_cast<int>(module_qpos_.size())) {
    if (error) *error = "no module_" + std::to_string(module);
    return std::nullopt;
  }
  Restart();
  PutTool(tip);
  PutModule(module, tip * DesignModuleInTip() * offset);
  mj_forward(model_.get(), data_.get());
  GripFailure failure;
  if (!suction_->Grip(model_.get(), data_.get(), &failure)) {
    if (error) *error = failure.message;
    return std::nullopt;
  }
  const double end = data_->time + settle;
  while (data_->time < end) mj_step(model_.get(), data_.get());
  mj_forward(model_.get(), data_.get());

  CheckShot shot = Shoot();
  shot.truth_in_tip = ToIsometry(*suction_->HeldPoseInTip(*data_));
  shot.truth_world = shot.tip * shot.truth_in_tip;
  shot.held = true;
  return shot;
}

CheckShot CheckRig::TakeEmpty(const Eigen::Isometry3d& tip) {
  Restart();
  PutTool(tip);
  mj_forward(model_.get(), data_.get());
  CheckShot shot = Shoot();
  // Nothing held: the truth is module_0 where it lies.
  shot.truth_world.translation() = Eigen::Vector3d(20.0, 0.0, 0.0175);
  shot.truth_in_tip = shot.tip.inverse() * shot.truth_world;
  return shot;
}

}  // namespace cell
