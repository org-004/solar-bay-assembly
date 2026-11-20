// Leaves the stack of modules alone for a while and reports how still it
// stays, and how fast the scene simulates.
//
//   bazel run --config=opt //:print_pallet
//   bazel run --config=opt //:print_pallet -- <seconds> <x> <y> <yaw>
//
// The optional arguments move the pallet off its nominal pose first (metres,
// radians; see PlacePallet). Settling takes the first second; after that
// each module's pose is recorded and compared with later ones.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/mj_handles.h"
#include "src/pallet.h"

namespace {

constexpr double kSettleTime = 1.0;  // s

// Angle of the rotation that takes `from` to `to`.
double AngleBetween(const Eigen::Quaterniond& from,
                    const Eigen::Quaterniond& to) {
  return from.angularDistance(to);
}

// How far module i leans back from vertical: the angle between its +y axis
// (up along the glass) and world +Z.
double Lean(const cell::WorldPose& pose) {
  const Eigen::Vector3d up = pose.orientation * Eigen::Vector3d::UnitY();
  return std::acos(std::clamp(up.z(), -1.0, 1.0));
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2 && argc != 3 && argc != 6) {
    std::fprintf(stderr, "usage: %s <model.xml> [seconds [x y yaw]]\n", argv[0]);
    return 1;
  }
  const double seconds = argc >= 3 ? std::atof(argv[2]) : 30.0;

  std::string error;
  cell::ModelPtr model = cell::LoadModel(argv[1], &error);
  if (!model) {
    std::fprintf(stderr, "could not load %s: %s\n", argv[1], error.c_str());
    return 1;
  }
  cell::DataPtr data = cell::MakeData(*model);
  const std::optional<cell::PalletBodies> bodies = cell::FindPallet(*model);
  if (!bodies) {
    std::fprintf(stderr, "no pallet with modules in %s\n", argv[1]);
    return 1;
  }
  const int n = static_cast<int>(bodies->modules.size());
  if (argc == 6) {
    cell::PlacePallet(*model, data.get(), *bodies, std::atof(argv[3]),
                      std::atof(argv[4]), std::atof(argv[5]));
  } else {
    mj_forward(model.get(), data.get());
  }

  const auto wall_start = std::chrono::steady_clock::now();
  long steps = 0;
  const auto all_poses = [&] {
    std::vector<cell::WorldPose> poses;
    for (int i = 0; i < n; ++i) poses.push_back(cell::ModulePose(*data, *bodies, i));
    return poses;
  };
  // Snapshots every 0.1 s while settling, to see afterwards how far from
  // its final rest the stack still was at each.
  std::vector<std::pair<double, std::vector<cell::WorldPose>>> snapshots;
  while (data->time < kSettleTime) {
    if (data->time >= 0.1 * snapshots.size() - 1e-9) {
      snapshots.push_back({data->time, all_poses()});
    }
    mj_step(model.get(), data.get());
    ++steps;
  }
  const std::vector<cell::WorldPose> settled = all_poses();

  // Largest distance and angle from the settled pose, over the whole run.
  std::vector<double> max_move(n, 0.0);
  std::vector<double> max_turn(n, 0.0);
  std::vector<cell::WorldPose> halfway;
  const double end = kSettleTime + seconds;
  const double middle = kSettleTime + seconds / 2;
  while (data->time < end) {
    mj_step(model.get(), data.get());
    ++steps;
    for (int i = 0; i < n; ++i) {
      const cell::WorldPose now = cell::ModulePose(*data, *bodies, i);
      max_move[i] = std::max(max_move[i], (now.position - settled[i].position).norm());
      max_turn[i] = std::max(max_turn[i], AngleBetween(settled[i].orientation, now.orientation));
    }
    if (halfway.empty() && data->time >= middle) {
      for (int i = 0; i < n; ++i) halfway.push_back(cell::ModulePose(*data, *bodies, i));
    }
  }
  const double wall = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - wall_start)
                          .count();

  std::printf("modules %d, simulated %.1f s after %.1f s settling\n", n, seconds,
              kSettleTime);
  std::printf("%ld steps in %.2f s wall: %.0f steps/s, %.1fx real time\n", steps,
              wall, steps / wall, data->time / wall);
  std::printf("contacts %d, constraint rows %d\n", data->ncon, data->nefc);
  std::printf("\n%6s %10s %10s %10s %10s %10s %10s %9s\n", "module", "max_mm",
              "max_mrad", "half1_mm", "half2_mm", "v_mm/s", "w_mrad/s", "lean_deg");
  double worst_move = 0, worst_turn = 0, worst_v = 0, worst_w = 0;
  for (int i = 0; i < n; ++i) {
    const cell::WorldPose now = cell::ModulePose(*data, *bodies, i);
    const double first = (halfway[i].position - settled[i].position).norm();
    const double second = (now.position - halfway[i].position).norm();
    // A free joint's velocity: linear in world coordinates, then angular in
    // the body's own frame.
    const int dof = model->jnt_dofadr[model->body_jntadr[bodies->modules[i]]];
    const double v = Eigen::Vector3d(data->qvel[dof], data->qvel[dof + 1],
                                     data->qvel[dof + 2]).norm();
    const double w = Eigen::Vector3d(data->qvel[dof + 3], data->qvel[dof + 4],
                                     data->qvel[dof + 5]).norm();
    worst_move = std::max(worst_move, max_move[i]);
    worst_turn = std::max(worst_turn, max_turn[i]);
    worst_v = std::max(worst_v, v);
    worst_w = std::max(worst_w, w);
    std::printf("%6d %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %9.4f\n", i,
                1e3 * max_move[i], 1e3 * max_turn[i], 1e3 * first, 1e3 * second,
                1e3 * v, 1e3 * w, Lean(now) * 180 / M_PI);
  }
  std::printf("worst: %.4f mm, %.4f mrad; end speed %.5f mm/s, %.5f mrad/s\n",
              1e3 * worst_move, 1e3 * worst_turn, 1e3 * worst_v, 1e3 * worst_w);
  for (int i : {0, n - 1}) {
    const cell::WorldPose now = cell::ModulePose(*data, *bodies, i);
    const Eigen::Vector3d moved = 1e3 * (now.position - settled[i].position);
    const Eigen::AngleAxisd turned(now.orientation * settled[i].orientation.conjugate());
    std::printf("module_%d moved %.4f %.4f %.4f mm, turned %.4f mrad about %.3f %.3f %.3f\n",
                i, moved.x(), moved.y(), moved.z(), 1e3 * turned.angle(),
                turned.axis().x(), turned.axis().y(), turned.axis().z());
  }

  std::printf("\nsettling: furthest any module was from where it ended\n");
  for (const auto& [time, poses] : snapshots) {
    double distance = 0, angle = 0;
    for (int i = 0; i < n; ++i) {
      const cell::WorldPose now = cell::ModulePose(*data, *bodies, i);
      distance = std::max(distance, (poses[i].position - now.position).norm());
      angle = std::max(angle, AngleBetween(poses[i].orientation, now.orientation));
    }
    std::printf("  t = %.1f s  %.4f mm  %.4f mrad\n", time, 1e3 * distance,
                1e3 * angle);
  }

  const cell::GlassFace front = cell::ModuleGlass(*data, *bodies, 0);
  const cell::GlassFace back = cell::ModuleGlass(*data, *bodies, n - 1);
  std::printf("\nglass centre module_0  %.5f %.5f %.5f  normal %.4f %.4f %.4f\n",
              front.centre.x(), front.centre.y(), front.centre.z(),
              front.normal.x(), front.normal.y(), front.normal.z());
  std::printf("glass centre module_%d %.5f %.5f %.5f  normal %.4f %.4f %.4f\n",
              n - 1, back.centre.x(), back.centre.y(), back.centre.z(),
              back.normal.x(), back.normal.y(), back.normal.z());
  // Face to face, along the front module's normal.
  double min_pitch = 1e9, max_pitch = 0;
  for (int i = 0; i + 1 < n; ++i) {
    const cell::GlassFace a = cell::ModuleGlass(*data, *bodies, i);
    const cell::GlassFace b = cell::ModuleGlass(*data, *bodies, i + 1);
    const double pitch = (a.centre - b.centre).dot(a.normal);
    min_pitch = std::min(min_pitch, pitch);
    max_pitch = std::max(max_pitch, pitch);
  }
  std::printf("glass to glass pitch: %.5f to %.5f m\n", min_pitch, max_pitch);
  return 0;
}
