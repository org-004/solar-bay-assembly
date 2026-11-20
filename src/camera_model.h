// A camera as a pinhole: where it is, and how points in the world map onto
// its pixels and back.
//
// The pinhole model. Every ray of light that reaches the image passes through
// one point, the camera's centre. A point in front of the camera, at
// (x, y, -z) in the camera's frame (z > 0 is how far in front it is), lands
// on the image at
//
//   u = cx + focal * x / z        v = cy - focal * y / z
//
// so things twice as far away look half as big. `focal` is the distance from
// the centre to the image in pixels, and (cx, cy) is where the camera's
// viewing axis meets the image, the principal point. Going back is the same
// in reverse: a pixel does not say how far away its point is, only which ray
// through the centre it lies on. The depth image (src/depth_camera.h)
// supplies the missing distance.
//
// Frame: MuJoCo's camera convention. The camera looks along its own -Z, +X
// is to the right in the image and +Y up. (OpenCV and ROS's optical frames
// look along +Z with +Y down instead; convert at the boundary if one is ever
// needed.) Pixels: u to the right and v down, from the top-left corner of the
// image, so the top-left pixel covers [0, 1) x [0, 1) and its centre is
// (0.5, 0.5); the pixel in column c and row r has its centre at
// (c + 0.5, r + 0.5). mjr_readPixels returns rows bottom first; flip them to
// get this v.
//
// Nothing here reads where anything but the camera is: perception code can
// use all of it.

#ifndef SRC_CAMERA_MODEL_H_
#define SRC_CAMERA_MODEL_H_

#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

namespace cell {

// The camera as a pinhole. Fixed, like a calibrated real camera, so control
// code may use it.
struct CameraModel {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // camera in world
  int width = 0;        // pixels
  int height = 0;
  double fovy = 0.0;    // rad, vertical field of view
  double focal = 0.0;   // pixels, the same along u and v
  double cx = 0.0;      // principal point, pixels: the image centre
  double cy = 0.0;
};

// Where a world point lands in the image, or nothing if it is behind the
// camera (or level with it). The result may lie outside the image; check
// against width and height.
std::optional<Eigen::Vector2d> Project(const CameraModel& camera,
                                       const Eigen::Vector3d& world);

// The point at depth 1 m on the ray through image point (u, v), in the
// camera's frame: ((u - cx) / focal, -(v - cy) / focal, -1). Multiplying it
// by a pixel's depth gives the pixel's point in the camera's frame. Not of
// length 1, except at the principal point.
Eigen::Vector3d PointAtUnitDepth(const CameraModel& camera, double u, double v);

// The direction, in the world, of the ray from the camera's centre through
// image point (u, v), of length 1. The inverse of Project: every point
// camera.pose.translation() + t * RayThroughPixel(camera, u, v) with t > 0
// projects back to (u, v). For the centre of the pixel in column c and row r
// pass (c + 0.5, r + 0.5). At the principal point it is the camera's -Z.
Eigen::Vector3d RayThroughPixel(const CameraModel& camera, double u, double v);

// The camera with id `camera_id` in `model`, posed as `data` has it now
// (cam_xpos and cam_xmat, which mj_forward, or mj_kinematics followed by
// mj_camlight, bring up to date). Returns nothing for an id out of range, an
// orthographic camera, or one whose sensor size and intrinsics give pixels
// that are not square, which CameraModel cannot describe.
//
// The numbers are worked out as MuJoCo's own rendering and rangefinder do:
// from fovy, with the principal point at the image centre, unless the
// camera gives sensorsize and focal/principal in MJCF, in which case from
// those.
std::optional<CameraModel> CameraModelFromMujoco(const mjModel& model,
                                                 const mjData& data,
                                                 int camera_id);

// The same for the camera named `name`; nothing if there is none.
std::optional<CameraModel> CameraModelFromMujoco(const mjModel& model,
                                                 const mjData& data,
                                                 const std::string& name);

// The camera moved by a small error, as a calibration that is slightly off
// would have it: turned by `rotation_vector` (axis times angle, radians)
// about its own centre and then shifted by `translation` (metres), both in
// the camera's own frame. That is pose * [R(rotation_vector) | translation].
// Everything else is kept.
//
// Use: render the image from the true camera, then hand perception the
// perturbed one. A point the perturbed camera reads at p in its own frame is
// put in the world at pose * (R p + translation) instead of pose * p.
CameraModel Perturbed(const CameraModel& camera,
                      const Eigen::Vector3d& rotation_vector,
                      const Eigen::Vector3d& translation);

}  // namespace cell

#endif  // SRC_CAMERA_MODEL_H_
