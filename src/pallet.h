// The pallet and the modules standing on it, in a loaded scene.
//
// models/pallet.xml defines a body `pallet` and bodies `module_0`,
// `module_1`, ... with module_0 at the front, nearest the arm. This finds
// them by name, moves the pallet with its stack to a new pose for each trial,
// and reports where things truly are, for scoring.
//
// The pallet is a mocap body: the physics holds it fixed, but its pose lives
// in mjData (mocap_pos and mocap_quat) rather than in mjModel. Moving it
// therefore leaves the model untouched, so one model can serve many trials,
// and mj_resetData puts it back at its nominal pose together with the
// modules.

#ifndef SRC_PALLET_H_
#define SRC_PALLET_H_

#include <optional>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

namespace cell {

// Size and mass of one module, as in models/pallet.xml.
inline constexpr double kModuleLength = 2.3;      // m, along the module's x
inline constexpr double kModuleWidth = 1.1;       // m, along y
inline constexpr double kModuleThickness = 0.035; // m, along z; glass at +z
inline constexpr double kModuleMass = 30.0;       // kg

// Where a body is in the world: the position of its origin and its
// orientation (body axes in world coordinates are the rotation's columns).
struct WorldPose {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

// The pallet and its modules in one model, and where each was built: the
// pose a trial starts from before PlacePallet moves anything.
struct PalletBodies {
  int pallet = -1;        // body id
  int pallet_mocap = -1;  // index into mjData::mocap_pos and mocap_quat
  std::vector<int> modules;  // body ids, modules[i] is module_<i>

  WorldPose nominal_pallet;
  std::vector<WorldPose> nominal_modules;  // in the same order
};

// Finds body `pallet` and every body named module_<i>, ordered by i. Returns
// nothing if the pallet is missing or is not a mocap body, if there are no
// modules, if the numbers have a gap, or if a module does not have exactly
// one free joint.
std::optional<PalletBodies> FindPallet(const mjModel& model);

// Puts the pallet at its nominal pose moved by (x, y) in the floor plane and
// turned by `yaw` radians about the vertical through its origin, and puts
// every module at its nominal place on the moved pallet, at rest. Then calls
// mj_forward, so positions and contacts in `data` are current.
//
// The modules go where they were built: touching, but not yet pressed
// together by their weight. The stack takes that up within a few tenths of
// a second, moving no module more than 0.2 mm; give it one second before
// relying on it standing still (see tests/pallet_test.cc).
void PlacePallet(const mjModel& model, mjData* data, const PalletBodies& bodies,
                 double x, double y, double yaw);

// Where the pallet is now.
WorldPose PalletPose(const mjData& data, const PalletBodies& bodies);

// Where module i is now: its centre and axes. Read after mj_forward or
// mj_step, which bring xpos and xquat up to date.
WorldPose ModulePose(const mjData& data, const PalletBodies& bodies, int i);

// The centre of module i's glass face (its +z face) and the direction the
// glass faces, both in world coordinates.
struct GlassFace {
  Eigen::Vector3d centre;
  Eigen::Vector3d normal;  // unit, out of the glass
};
GlassFace ModuleGlass(const mjData& data, const PalletBodies& bodies, int i);

}  // namespace cell

#endif  // SRC_PALLET_H_
