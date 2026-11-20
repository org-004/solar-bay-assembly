// Prints one made-up pallet view as CSV, one row per point, for plotting.
// Each row has the point, the surface it was generated on, and whether the
// RANSAC fit counted it as part of the face's plane.
//
//   bazel run //:print_points
//   bazel run //:print_points -- <seed>
//
// With no number the seed is 1. The camera is at the origin looking along
// +X, with +Y to its left and +Z up. Lengths are metres.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <random>
#include <vector>

#include "src/plane_fit.h"
#include "src/point_generator.h"

int main(int argc, char** argv) {
  int seed = 1;
  if (argc == 2) seed = std::atoi(argv[1]);
  if (argc > 2 || seed < 1) {
    std::fprintf(stderr, "usage: %s [<seed>]\n", argv[0]);
    return 1;
  }

  const cell::PalletViewOptions scene;
  std::mt19937 rng(seed);
  const cell::PalletView view = cell::MakePalletView(scene, rng);

  cell::RansacOptions fit;
  fit.inlier_distance = 3.0 * scene.noise;
  fit.seed = seed;
  const std::optional<cell::RansacResult> result =
      cell::FitPlaneRansac(view.points, fit);

  std::vector<bool> inlier(view.points.size(), false);
  if (result) {
    for (int index : result->inliers) inlier[index] = true;
  }

  // The generator lays the surfaces down in this order.
  const int behind_start = scene.face_points;
  const int floor_start = behind_start + scene.behind_points;
  const int clutter_start = floor_start + scene.floor_points;

  std::printf("x,y,z,surface,inlier\n");
  for (int i = 0; i < static_cast<int>(view.points.size()); ++i) {
    const char* surface = i < behind_start    ? "face"
                          : i < floor_start   ? "behind"
                          : i < clutter_start ? "floor"
                                              : "clutter";
    const Eigen::Vector3d& p = view.points[i];
    std::printf("%.6f,%.6f,%.6f,%s,%d\n", p.x(), p.y(), p.z(), surface,
                inlier[i] ? 1 : 0);
  }

  std::fprintf(stderr, "seed %d: %d points, %d counted as the face's plane\n",
               seed, static_cast<int>(view.points.size()),
               result ? static_cast<int>(result->inliers.size()) : 0);
  return result ? 0 : 1;
}
