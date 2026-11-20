// Fits a plane to a series of made-up pallet views and prints how far each
// answer is from the truth, one row per view.
// The output is CSV, for a plotting tool. With --pretty it is a table of
// aligned columns, for reading in a terminal.
//
//   bazel run //:print_plane_fit
//   bazel run //:print_plane_fit -- --pretty
//   bazel run //:print_plane_fit -- <views>
//
// With no number there are 20 views. View n is made from seed n.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <numbers>
#include <optional>
#include <random>
#include <vector>

#include "src/plane_fit.h"
#include "src/point_generator.h"

int main(int argc, char** argv) {
  // --pretty may appear anywhere. What is left must be nothing or one count.
  bool pretty = false;
  std::vector<const char*> numbers;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--pretty") == 0) {
      pretty = true;
    } else {
      numbers.push_back(argv[i]);
    }
  }
  int views = 20;
  if (numbers.size() == 1) views = std::atoi(numbers[0]);
  if (numbers.size() > 1 || views < 1) {
    std::fprintf(stderr, "usage: %s [--pretty] [<views>]\n", argv[0]);
    return 1;
  }

  const cell::PalletViewOptions scene;
  cell::RansacOptions fit;
  fit.inlier_distance = 3.0 * scene.noise;

  const Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
  const double to_degrees = 180.0 / std::numbers::pi;
  const double to_mm = 1000.0;

  const char* row_format = "%d,%.4f,%.4f,%.3f,%.4f,%.4f,%d\n";
  if (pretty) {
    row_format = "%5d %9.4f %9.4f | %9.3f %9.4f | %9.4f %8d\n";
    std::printf("%5s %-19s | %-19s | %-9s\n", "", "distance",
                "lean", "normal");
    std::printf("%5s %9s %9s | %9s %9s | %9s %8s\n", "seed", "true m",
                "err mm", "true deg", "err deg", "err deg", "inliers");
  } else {
    std::printf("seed,true_distance_m,distance_error_mm,true_lean_deg,"
                "lean_error_deg,normal_error_deg,inliers\n");
  }

  double worst_distance = 0.0;
  double worst_angle = 0.0;
  int failures = 0;
  for (int seed = 1; seed <= views; ++seed) {
    std::mt19937 rng(seed);
    const cell::PalletView view = cell::MakePalletView(scene, rng);
    const cell::Plane truth = view.face.plane();

    fit.seed = seed;
    const std::optional<cell::RansacResult> result =
        cell::FitPlaneRansac(view.points, fit);
    if (!result) {
      ++failures;
      continue;
    }

    const double distance_error =
        (result->plane.distance - truth.distance) * to_mm;
    const double lean_error =
        (cell::Lean(result->plane, up) - cell::Lean(truth, up)) * to_degrees;
    const double normal_error =
        cell::AngleBetween(result->plane, truth) * to_degrees;
    worst_distance = std::max(worst_distance, std::abs(distance_error));
    worst_angle = std::max(worst_angle, normal_error);

    std::printf(row_format, seed, truth.distance, distance_error,
                cell::Lean(truth, up) * to_degrees, lean_error, normal_error,
                static_cast<int>(result->inliers.size()));
  }

  std::fprintf(stderr,
               "%d views of %d points, %d on the face, noise %.1f mm\n"
               "worst distance error %.4f mm, worst normal error %.4f deg, "
               "%d not fitted\n",
               views,
               scene.face_points + scene.behind_points + scene.floor_points +
                   scene.clutter_points,
               scene.face_points, scene.noise * to_mm, worst_distance,
               worst_angle, failures);
  return failures == 0 ? 0 : 1;
}
