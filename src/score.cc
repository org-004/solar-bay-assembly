#include "src/score.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/cell.h"

namespace cell {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegrees = 180.0 / kPi;

}  // namespace

PlacementError MeasurePlacement(const Cell& cell, int i, int slot) {
  const Eigen::Isometry3d slot_pose =
      cell.conveyor().TrueSlotPose(*cell.data(), slot);
  const Eigen::Isometry3d module = slot_pose.inverse() * cell.TrueModulePose(i);

  PlacementError error;
  error.module = i;
  error.slot = slot;
  error.across_tube = module.translation().x();
  error.along_tube = module.translation().y();
  error.height = module.translation().z();

  // The long side seen from above. Only its direction in the slot's floor
  // plane counts; tilt is measured on its own below.
  const Eigen::Vector3d long_side = module.rotation().col(0);
  double yaw = std::atan2(long_side.y(), long_side.x());
  if (yaw > kPi / 2) yaw -= kPi;
  if (yaw <= -kPi / 2) yaw += kPi;
  error.yaw = yaw;

  const Eigen::Vector3d normal = module.rotation().col(2);
  error.tilt = std::acos(std::clamp(normal.z(), -1.0, 1.0));

  const Eigen::Vector2d speed = cell.TrueModuleSpeedOnTube(i);
  error.speed = speed[0];
  error.spin = speed[1];
  error.at_rest = error.speed < kRestSpeed && error.spin < kRestSpin;
  return error;
}

std::optional<PlacementError> MeasurePlacement(const Cell& cell, int i) {
  const std::optional<int> slot = cell.TruePlacedSlot(i);
  if (!slot) return std::nullopt;
  return MeasurePlacement(cell, i, *slot);
}

StackDisturbance MeasureStackDisturbance(const Cell& cell) {
  StackDisturbance result;
  const auto& baseline = cell.true_stack_baseline();
  for (int i = 0; i < cell.module_count(); ++i) {
    if (cell.module_state(i) != ModuleState::kOnPallet) continue;
    ++result.modules;
    const Eigen::Isometry3d now = cell.TrueModulePose(i);
    const double displacement =
        (now.translation() - baseline[i].translation()).norm();
    const double rotation =
        Eigen::AngleAxisd(baseline[i].rotation().transpose() * now.rotation())
            .angle();
    if (result.worst_displacement_module < 0 ||
        displacement > result.worst_displacement) {
      result.worst_displacement = displacement;
      result.worst_displacement_module = i;
    }
    if (result.worst_rotation_module < 0 ||
        rotation > result.worst_rotation) {
      result.worst_rotation = rotation;
      result.worst_rotation_module = i;
    }
  }
  return result;
}

std::string Describe(const PlacementError& error) {
  char text[300];
  std::snprintf(text, sizeof(text),
                "module_%d in slot %d: along %+.2f mm, across %+.2f mm, "
                "height %+.2f mm, yaw %+.3f deg, tilt %.3f deg, %s "
                "(%.2f mm/s, %.3f deg/s)",
                error.module, error.slot, 1e3 * error.along_tube,
                1e3 * error.across_tube, 1e3 * error.height,
                kDegrees * error.yaw, kDegrees * error.tilt,
                error.at_rest ? "at rest" : "moving", 1e3 * error.speed,
                kDegrees * error.spin);
  return text;
}

std::string Describe(const StackDisturbance& disturbance) {
  char text[300];
  std::snprintf(text, sizeof(text),
                "%d modules on the pallet: worst moved %.3f mm (module_%d), "
                "worst turned %.4f deg (module_%d)",
                disturbance.modules, 1e3 * disturbance.worst_displacement,
                disturbance.worst_displacement_module,
                kDegrees * disturbance.worst_rotation,
                disturbance.worst_rotation_module);
  return text;
}

}  // namespace cell
