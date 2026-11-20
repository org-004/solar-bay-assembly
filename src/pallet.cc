#include "src/pallet.h"

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

namespace cell {
namespace {

// If `name` is "module_<i>" with i written plainly (no sign, no leading
// zeros), returns i.
std::optional<int> ModuleIndex(const char* name) {
  if (name == nullptr) return std::nullopt;
  const std::string text(name);
  const std::string prefix = "module_";
  if (text.size() <= prefix.size() || text.compare(0, prefix.size(), prefix) != 0) {
    return std::nullopt;
  }
  const std::string digits = text.substr(prefix.size());
  if (digits.size() > 6) return std::nullopt;  // no pallet holds a million
  if (digits.size() > 1 && digits[0] == '0') return std::nullopt;
  int index = 0;
  for (char digit : digits) {
    if (digit < '0' || digit > '9') return std::nullopt;
    index = 10 * index + (digit - '0');
  }
  return index;
}

// MuJoCo stores quaternions as w, x, y, z. Eigen's constructor takes them in
// the same order, but Eigen keeps them as x, y, z, w in memory, so the
// arrays cannot be copied across directly.
Eigen::Quaterniond QuatFrom(const mjtNum* wxyz) {
  return Eigen::Quaterniond(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
}

void QuatTo(const Eigen::Quaterniond& q, mjtNum* wxyz) {
  wxyz[0] = q.w();
  wxyz[1] = q.x();
  wxyz[2] = q.y();
  wxyz[3] = q.z();
}

}  // namespace

std::optional<PalletBodies> FindPallet(const mjModel& model) {
  PalletBodies bodies;
  bodies.pallet = mj_name2id(&model, mjOBJ_BODY, "pallet");
  if (bodies.pallet < 0) return std::nullopt;
  bodies.pallet_mocap = model.body_mocapid[bodies.pallet];
  if (bodies.pallet_mocap < 0) return std::nullopt;

  // A mocap body hangs off the world, so its body_pos and body_quat are
  // already world coordinates.
  bodies.nominal_pallet.position =
      Eigen::Vector3d(model.body_pos[3 * bodies.pallet],
                      model.body_pos[3 * bodies.pallet + 1],
                      model.body_pos[3 * bodies.pallet + 2]);
  bodies.nominal_pallet.orientation =
      QuatFrom(model.body_quat + 4 * bodies.pallet).normalized();

  // Ordered by index, whatever order the bodies are in the file.
  std::map<int, int> found;  // index -> body id
  for (int body = 0; body < model.nbody; ++body) {
    const std::optional<int> index =
        ModuleIndex(mj_id2name(&model, mjOBJ_BODY, body));
    if (index) found[*index] = body;
  }
  if (found.empty()) return std::nullopt;
  // A map is sorted, so the numbers run 0..n-1 exactly when the last is n-1.
  if (found.rbegin()->first != static_cast<int>(found.size()) - 1) {
    return std::nullopt;
  }

  for (const auto& [index, body] : found) {
    if (model.body_jntnum[body] != 1) return std::nullopt;
    const int joint = model.body_jntadr[body];
    if (model.jnt_type[joint] != mjJNT_FREE) return std::nullopt;

    // A free joint's position coordinates are the body's world position
    // and then its orientation. qpos0 holds them as built.
    const mjtNum* qpos0 = model.qpos0 + model.jnt_qposadr[joint];
    bodies.modules.push_back(body);
    bodies.nominal_modules.push_back(
        {Eigen::Vector3d(qpos0[0], qpos0[1], qpos0[2]),
         QuatFrom(qpos0 + 3).normalized()});
  }
  return bodies;
}

void PlacePallet(const mjModel& model, mjData* data, const PalletBodies& bodies,
                 double x, double y, double yaw) {
  const WorldPose& nominal = bodies.nominal_pallet;
  const Eigen::Quaterniond turn(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d position = nominal.position + Eigen::Vector3d(x, y, 0);
  const Eigen::Quaterniond orientation = (turn * nominal.orientation).normalized();

  data->mocap_pos[3 * bodies.pallet_mocap] = position.x();
  data->mocap_pos[3 * bodies.pallet_mocap + 1] = position.y();
  data->mocap_pos[3 * bodies.pallet_mocap + 2] = position.z();
  QuatTo(orientation, data->mocap_quat + 4 * bodies.pallet_mocap);

  // Each module keeps the pose it was built at relative to the pallet.
  const Eigen::Quaterniond nominal_inverse = nominal.orientation.conjugate();
  for (std::size_t i = 0; i < bodies.modules.size(); ++i) {
    const WorldPose& module = bodies.nominal_modules[i];
    const Eigen::Vector3d offset =
        nominal_inverse * (module.position - nominal.position);
    const Eigen::Quaterniond relative = nominal_inverse * module.orientation;

    const Eigen::Vector3d placed = position + orientation * offset;
    const Eigen::Quaterniond turned = (orientation * relative).normalized();

    const int joint = model.body_jntadr[bodies.modules[i]];
    mjtNum* qpos = data->qpos + model.jnt_qposadr[joint];
    qpos[0] = placed.x();
    qpos[1] = placed.y();
    qpos[2] = placed.z();
    QuatTo(turned, qpos + 3);

    // At rest, and with no memory of the old pose: the solver starts its
    // search from last step's accelerations, which belonged elsewhere.
    const int dof = model.jnt_dofadr[joint];
    for (int k = 0; k < 6; ++k) {
      data->qvel[dof + k] = 0;
      data->qacc_warmstart[dof + k] = 0;
    }
  }

  mj_forward(&model, data);
}

WorldPose PalletPose(const mjData& data, const PalletBodies& bodies) {
  const int b = bodies.pallet;
  return {Eigen::Vector3d(data.xpos[3 * b], data.xpos[3 * b + 1],
                          data.xpos[3 * b + 2]),
          QuatFrom(data.xquat + 4 * b)};
}

WorldPose ModulePose(const mjData& data, const PalletBodies& bodies, int i) {
  const int b = bodies.modules.at(i);
  return {Eigen::Vector3d(data.xpos[3 * b], data.xpos[3 * b + 1],
                          data.xpos[3 * b + 2]),
          QuatFrom(data.xquat + 4 * b)};
}

GlassFace ModuleGlass(const mjData& data, const PalletBodies& bodies, int i) {
  const WorldPose pose = ModulePose(data, bodies, i);
  const Eigen::Vector3d normal = pose.orientation * Eigen::Vector3d::UnitZ();
  return {pose.position + 0.5 * kModuleThickness * normal, normal};
}

}  // namespace cell
