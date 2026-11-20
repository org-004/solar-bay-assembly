// Loads the whole cell, resets it, and prints what is in it, what touches
// what, where everything is, and how fast it simulates. Time it optimized:
//
//   bazel run --config=opt //:print_cell
//   bazel run --config=opt //:print_cell -- <seed> <seconds>
//
// With no seed the scene is nominal (no randomization); with one it is
// randomized as a trial would be. The arm holds arm_home throughout.

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/cell.h"
#include "src/pallet.h"
#include "src/score.h"

namespace {

constexpr double kDegrees = 180.0 / 3.14159265358979323846;

std::string BodyName(const mjModel& model, int geom) {
  const char* name = mj_id2name(&model, mjOBJ_BODY, model.geom_bodyid[geom]);
  return name != nullptr ? name : "world";
}

// Contacts counted by the pair of bodies, with module_<i> written as
// module_* so that the 30 modules make a few lines rather than hundreds.
void PrintContacts(const mjModel& model, const mjData& data) {
  std::map<std::pair<std::string, std::string>, int> pairs;
  for (int c = 0; c < data.ncon; ++c) {
    auto kind = [](std::string name) {
      return name.rfind("module_", 0) == 0 ? std::string("module_*") : name;
    };
    std::string a = kind(BodyName(model, data.contact[c].geom[0]));
    std::string b = kind(BodyName(model, data.contact[c].geom[1]));
    if (b < a) std::swap(a, b);
    ++pairs[{a, b}];
  }
  std::printf("contacts %d, constraint rows %d\n", data.ncon, data.nefc);
  for (const auto& [pair, count] : pairs) {
    std::printf("  %-10s - %-10s %4d\n", pair.first.c_str(),
                pair.second.c_str(), count);
  }
}

void PrintPose(const char* name, const Eigen::Isometry3d& pose) {
  const Eigen::Vector3d p = pose.translation();
  const Eigen::Vector3d z = pose.rotation().col(2);
  std::printf("  %-22s at %8.4f %8.4f %8.4f   z axis %7.4f %7.4f %7.4f\n",
              name, p.x(), p.y(), p.z(), z.x(), z.y(), z.z());
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argc > 4) {
    std::fprintf(stderr, "usage: %s <cell.xml> [seed [seconds]]\n", argv[0]);
    return 1;
  }
  const bool randomized = argc >= 3;
  const std::uint32_t seed =
      randomized ? static_cast<std::uint32_t>(std::strtoul(argv[2], nullptr, 10))
                 : 0;
  const double seconds = argc >= 4 ? std::atof(argv[3]) : 10.0;

  std::string error;
  std::optional<cell::Cell> loaded = cell::Cell::Load(argv[1], &error);
  if (!loaded) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  cell::Cell& c = *loaded;
  const mjModel& model = *c.model();

  // MuJoCo's counts are wider than int on some platforms.
  std::printf("bodies %ld, joints %ld, dofs %ld, actuators %ld, geoms %ld, "
              "modules %d, slots %d, cameras %ld\n",
              static_cast<long>(model.nbody), static_cast<long>(model.njnt),
              static_cast<long>(model.nv), static_cast<long>(model.nu),
              static_cast<long>(model.ngeom), c.module_count(),
              cell::Conveyor::kSlotCount, static_cast<long>(model.ncam));

  const auto reset_start = std::chrono::steady_clock::now();
  c.Reset(seed, randomized ? cell::Randomization{}
                           : cell::Randomization::None());
  const double reset_wall = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - reset_start)
                                .count();
  const cell::TrueScene& scene = c.true_scene();
  std::printf("\nreset seed %u (%s), settled %.1f s in %.2f s wall\n", seed,
              randomized ? "randomized" : "nominal", cell::Cell::kSettleTime,
              reset_wall);
  std::printf("pallet offset (truth): x %+.4f m, y %+.4f m, yaw %+.3f deg\n",
              scene.pallet_x, scene.pallet_y, kDegrees * scene.pallet_yaw);
  std::printf("\nat arm_home after settling: ");
  PrintContacts(model, *c.data());

  // Steps per second with everything present, the arm holding home.
  const auto wall_start = std::chrono::steady_clock::now();
  const double sim_start = c.data()->time;
  c.StepFor(seconds);
  const double wall = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - wall_start)
                          .count();
  const double simulated = c.data()->time - sim_start;
  const double steps = simulated / model.opt.timestep;
  std::printf("\n%.0f steps (%.1f s) in %.2f s wall: %.0f steps/s, %.1fx real "
              "time\n",
              steps, simulated, wall, steps / wall, simulated / wall);
  mj_forward(c.model(), c.data());
  std::printf("after: ");
  PrintContacts(model, *c.data());
  std::printf("stack: %s\n",
              cell::Describe(cell::MeasureStackDisturbance(c)).c_str());

  std::printf("\nwhere things are:\n");
  const Eigen::VectorXd q = cell::JointAngles(c.arm(), *c.data());
  std::printf("  arm joints (rad)       %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n",
              q[0], q[1], q[2], q[3], q[4], q[5]);
  std::printf("  arm from home (mrad)   %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n",
              1e3 * (q[0] - c.arm_home()[0]), 1e3 * (q[1] - c.arm_home()[1]),
              1e3 * (q[2] - c.arm_home()[2]), 1e3 * (q[3] - c.arm_home()[3]),
              1e3 * (q[4] - c.arm_home()[4]), 1e3 * (q[5] - c.arm_home()[5]));
  const Eigen::Isometry3d tip = cell::TipPose(c.arm(), *c.data());
  PrintPose("tool_tip", tip);
  // What lies straight below the tool tip, leaving out the arm and tool.
  {
    const Eigen::Vector3d down(0, 0, -1);
    int geom = -1;
    const mjtNum distance =
        mj_ray(c.model(), c.data(), tip.translation().data(), down.data(),
               nullptr, 1, c.arm().tool_body, &geom, nullptr);
    const char* name = geom >= 0 ? mj_id2name(c.model(), mjOBJ_GEOM, geom)
                                 : nullptr;
    std::printf("  %-22s %s, %.3f m below\n", "below the tool tip",
                name != nullptr ? name : "(unnamed)", distance);
  }
  const cell::WorldPose pallet = cell::PalletPose(*c.data(), c.pallet());
  std::printf("  %-22s at %8.4f %8.4f %8.4f\n", "pallet", pallet.position.x(),
              pallet.position.y(), pallet.position.z());
  for (int i : {0, c.module_count() - 1}) {
    const cell::GlassFace glass = cell::ModuleGlass(*c.data(), c.pallet(), i);
    std::printf("  module_%-2d glass centre  at %8.4f %8.4f %8.4f   normal "
                "%7.4f %7.4f %7.4f\n",
                i, glass.centre.x(), glass.centre.y(), glass.centre.z(),
                glass.normal.x(), glass.normal.y(), glass.normal.z());
  }
  std::printf("  %-22s %8.5f m (truth %8.5f m)\n", "conveyor encoder",
              c.conveyor().SensorPosition(*c.data()),
              c.conveyor().TrueSlidePosition(*c.data()));
  PrintPose("slot 0 (true)", c.conveyor().TrueSlotPose(*c.data(), 0));
  PrintPose("parking spot 0", cell::ParkingPose(0));

  const cell::CameraModel camera = c.DepthCamera();
  PrintPose("depth_camera", camera.pose);
  std::printf("  camera %d x %d, fovy %.1f deg, focal %.1f px\n", camera.width,
              camera.height, kDegrees * camera.fovy, camera.focal);
  std::printf("  module_0 glass corners in the image (u, v px):");
  const Eigen::Isometry3d front = c.TrueModulePose(0);
  for (double a : {-1.0, 1.0}) {
    for (double b : {-1.0, 1.0}) {
      const Eigen::Vector3d corner =
          front * Eigen::Vector3d(a * cell::kModuleLength / 2,
                                  b * cell::kModuleWidth / 2,
                                  cell::kModuleThickness / 2);
      const std::optional<Eigen::Vector2d> pixel = cell::Project(camera, corner);
      if (pixel) std::printf("  (%.0f, %.0f)", pixel->x(), pixel->y());
    }
  }
  std::printf("\n");
  return 0;
}
