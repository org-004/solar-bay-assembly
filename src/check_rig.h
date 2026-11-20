// The test rig of the check after pickup, models/check_scene.xml: the tool
// on a mocap carriage, module_0 held under it by the real suction weld at a
// chosen grip offset, and check_camera on the floor looking up. Shared by
// tests/held_module_finder_test.cc and src/print_check.cc.
//
// A shot: start from the scene as built (mj_resetData, so that no solver
// state carries over from the shot before and the same shot is the same bit
// for bit), put the tool tip where wanted, put the module at the grip offset
// wanted, grip it with Suction::Grip, let the physics run a moment so the
// weld takes the module's weight, and take the depth image. The truth of
// where the module then sits on the tool (Suction::HeldPoseInTip) comes
// with it. For tests and the table only: the finder never sees the truth.

#ifndef SRC_CHECK_RIG_H_
#define SRC_CHECK_RIG_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/depth_camera.h"
#include "src/mj_handles.h"
#include "src/suction.h"

namespace cell {

struct CheckShot {
  DepthImage image;            // exact; add noise to copies (AddDepthNoise)
  std::vector<int> geoms;      // the geom each pixel saw: truth
  CameraModel camera;          // the true camera
  Eigen::Isometry3d tip = Eigen::Isometry3d::Identity();  // tool tip, world
  // Truth, after the weld took the weight: module_0 in the tool tip's frame
  // and in the world. With nothing held, the module's parked pose.
  Eigen::Isometry3d truth_in_tip = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d truth_world = Eigen::Isometry3d::Identity();
  bool held = false;
  double render_seconds = 0.0;
};

// A grip offset: the module turned and moved from the design grip, in the
// design module's own frame: design * offset is where it is held.
Eigen::Isometry3d GripOffset(const Eigen::Vector3d& shift,
                             const Eigen::Vector3d& rotation_vector);

// One drawn from `seed`: each of the three shifts uniform in +-position and
// each of the three rotation-vector components uniform in +-angle. Draws
// straight from std::mt19937, whose sequence the standard fixes, so the
// same seed gives the same offset everywhere.
Eigen::Isometry3d DrawGripOffset(std::uint32_t seed, double position,
                                 double angle);

// How far `estimate` is from `truth`, in the truth's frame: the shift along
// its x, y, z, then the rotation vector of truth^-1 * estimate (about x, y
// and z, radians).
Eigen::Matrix<double, 6, 1> PoseError6(const Eigen::Isometry3d& estimate,
                                       const Eigen::Isometry3d& truth);

class CheckRig {
 public:
  // Loads the scene and finds the carriage, the roof, the modules (module_0,
  // and module_1 of another size if present), the suction weld and
  // check_camera. `suction` defaults to the cell's, except that the
  // tip may be 15 mm off the glass (the rig grips at offsets up to 10 mm
  // along the normal, with a tilt). Returns nothing, with the reason in
  // `error`, if something is missing.
  static std::optional<CheckRig> Load(const std::string& path,
                                      std::string* error = nullptr);

  // The tool tip as built: (0, 0, 1.585), pointing down, its X along +X.
  const Eigen::Isometry3d& nominal_tip() const { return nominal_tip_; }

  // Takes a shot: the tool tip at `tip`, module_<module> gripped at
  // tip * DesignModuleInTip() * offset, `settle` seconds of physics, then
  // the image. The other modules lie on the floor far out of view. Returns
  // nothing, with the reason, if the grip fails or there is no such module.
  std::optional<CheckShot> Take(const Eigen::Isometry3d& tip,
                                const Eigen::Isometry3d& offset,
                                std::string* error = nullptr,
                                double settle = 0.2, int module = 0);

  // The same with nothing on the tool: every module lies on the floor far
  // out of the camera's view.
  CheckShot TakeEmpty(const Eigen::Isometry3d& tip);

  // The roof over the rig, in place (4 m up) or moved far away.
  void SetRoof(bool present);

  // Renders what the camera sees now.
  DepthImage Render(std::vector<int>* geoms = nullptr);

  const mjModel& model() const { return *model_; }
  mjData* data() { return data_.get(); }
  const Suction& suction() const { return *suction_; }
  CameraModel camera() const;

 private:
  CheckRig() = default;
  // Back to the scene as built, the roof as last set, nothing held, every
  // module on the floor out of view.
  void Restart();
  void PutTool(const Eigen::Isometry3d& tip);
  void PutModule(int module, const Eigen::Isometry3d& pose);
  CheckShot Shoot();

  ModelPtr model_;
  DataPtr data_;
  std::optional<Suction> suction_;
  int carriage_mocap_ = -1;
  int roof_mocap_ = -1;
  std::vector<int> module_qpos_;  // per module, its free joint's
  std::vector<int> module_dof_;
  bool roof_ = true;
  int camera_ = -1;
  int tip_site_ = -1;
  Eigen::Isometry3d nominal_tip_ = Eigen::Isometry3d::Identity();
  // The carriage's frame in the tool tip's, fixed by the tool's design.
  Eigen::Isometry3d carriage_in_tip_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d roof_position_ = Eigen::Vector3d::Zero();
};

}  // namespace cell

#endif  // SRC_CHECK_RIG_H_
