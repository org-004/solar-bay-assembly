#include "src/camera_model.h"

#include <cmath>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

namespace cell {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

std::optional<Eigen::Vector2d> Project(const CameraModel& camera,
                                       const Eigen::Vector3d& world) {
  const Eigen::Vector3d p = camera.pose.inverse() * world;
  if (p.z() >= 0.0) return std::nullopt;  // behind: the camera looks along -Z
  const double depth = -p.z();
  // +X right is +u; +Y up is -v, since v counts down from the top.
  return Eigen::Vector2d(camera.cx + camera.focal * p.x() / depth,
                         camera.cy - camera.focal * p.y() / depth);
}

Eigen::Vector3d PointAtUnitDepth(const CameraModel& camera, double u,
                                 double v) {
  // Project solved for x and y with depth 1.
  return Eigen::Vector3d((u - camera.cx) / camera.focal,
                         -(v - camera.cy) / camera.focal, -1.0);
}

Eigen::Vector3d RayThroughPixel(const CameraModel& camera, double u,
                                double v) {
  return camera.pose.linear() * PointAtUnitDepth(camera, u, v).normalized();
}

std::optional<CameraModel> CameraModelFromMujoco(const mjModel& model,
                                                 const mjData& data,
                                                 int camera_id) {
  if (camera_id < 0 || camera_id >= model.ncam) return std::nullopt;
  if (model.cam_projection[camera_id] != mjPROJ_PERSPECTIVE) {
    return std::nullopt;
  }
  CameraModel camera;
  camera.pose.translation() =
      Eigen::Map<const Eigen::Vector3d>(data.cam_xpos + 3 * camera_id);
  // Row-major, columns are the camera's axes in the world.
  camera.pose.linear() =
      Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
          data.cam_xmat + 9 * camera_id);
  camera.width = model.cam_resolution[2 * camera_id];
  camera.height = model.cam_resolution[2 * camera_id + 1];

  const float* sensor = model.cam_sensorsize + 2 * camera_id;
  if (sensor[0] != 0.0f && sensor[1] != 0.0f) {
    // Focal lengths and principal point given as lengths on a sensor of a
    // given size: scale to pixels.
    const float* intrinsic = model.cam_intrinsic + 4 * camera_id;
    const double fx = intrinsic[0] / sensor[0] * camera.width;
    const double fy = intrinsic[1] / sensor[1] * camera.height;
    if (std::abs(fx - fy) > 1e-6 * fy) return std::nullopt;
    camera.focal = fy;
    camera.cx = intrinsic[2] / sensor[0] * camera.width;
    camera.cy = intrinsic[3] / sensor[1] * camera.height;
    camera.fovy = 2.0 * std::atan(0.5 * camera.height / camera.focal);
  } else {
    // MuJoCo keeps fovy in degrees.
    camera.fovy = model.cam_fovy[camera_id] * kPi / 180.0;
    camera.focal = 0.5 * camera.height / std::tan(0.5 * camera.fovy);
    camera.cx = 0.5 * camera.width;
    camera.cy = 0.5 * camera.height;
  }
  return camera;
}

std::optional<CameraModel> CameraModelFromMujoco(const mjModel& model,
                                                 const mjData& data,
                                                 const std::string& name) {
  return CameraModelFromMujoco(model, data,
                               mj_name2id(&model, mjOBJ_CAMERA, name.c_str()));
}

CameraModel Perturbed(const CameraModel& camera,
                      const Eigen::Vector3d& rotation_vector,
                      const Eigen::Vector3d& translation) {
  Eigen::Isometry3d error = Eigen::Isometry3d::Identity();
  const double angle = rotation_vector.norm();
  if (angle > 0.0) {
    error.linear() =
        Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
  }
  error.translation() = translation;
  CameraModel result = camera;
  result.pose = camera.pose * error;
  return result;
}

}  // namespace cell
