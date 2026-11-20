// The perception stage's table: the front module's pose as estimated from
// one depth image, against the truth, over randomized pallets and stack
// depths.
//
//   bazel run --config=opt //:print_perception -- [--pretty] [--trials T]
//       [--noise] [--calibration-error MM,DEG] [--depths 0,10,20,29]
//       [--first-seed S] [--no-creases] [--arm-home]
//
//   --pretty      an aligned table instead of CSV
//   --trials T    seeds S to S + T - 1, each a randomized pallet (default 20)
//   --first-seed S  (default 1)
//   --depths ...  stack depths k: modules 0 to k - 1 are carried away first,
//                 so module_k is the front one (default 0,10,20,29); 30
//                 carries every module away, and the finder must refuse
//   --noise       switch on every effect of the camera's noise model
//   --calibration-error MM,DEG
//                 hand perception a camera model that is off: shifted MM mm
//                 along the camera's own +X (to the right in the image) and
//                 turned DEG degrees about that axis (tipping the view), as
//                 Perturbed in src/camera_model.h does
//   --no-creases  keep the rectangle fit's own long edges, for comparison
//   --arm-home    take the picture with the arm at arm_home instead of
//                 arm_clear: its tool stands in front of the glass
//   --verbose     after each row, the finder's diagnostics on stderr
//
// For each seed and depth: reset the cell with that seed, carry the front
// modules away (Cell::ParkModule), let the stack settle for half a second,
// put the arm at arm_clear, take the depth image, and estimate the front
// module's pose (src/module_finder.h). One row per picture: the pallet's true
// offset, the outcome, and the error of the estimate in the true module's
// frame: along the glass's normal, along the long side and up the lean (mm),
// the angle between the estimated and true glass normals and the rotation
// about the normal (degrees), the points used and the time taken. Then a
// summary: worst, root mean square and mean of every error column, and the
// refusals.
//
// The truth (Cell::TrueModulePose, true_scene) is read here, to score, and
// nowhere in the perception code.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/cell.h"
#include "src/depth_camera.h"
#include "src/module_finder.h"

namespace {

constexpr double kToDegrees = 180.0 / std::numbers::pi;
constexpr double kToMm = 1000.0;

// How far `estimate` is from `truth`, in the true module's frame.
struct PoseError {
  double normal = 0.0;    // m, along the true +z
  double long_side = 0.0; // m, along the true +x
  double up_lean = 0.0;   // m, along the true +y
  double face = 0.0;      // rad, between the +z axes
  double in_plane = 0.0;  // rad, about the true +z
};

PoseError ErrorOf(const Eigen::Isometry3d& estimate,
                  const Eigen::Isometry3d& truth) {
  const Eigen::Matrix3d r = truth.linear();
  const Eigen::Vector3d d =
      r.transpose() * (estimate.translation() - truth.translation());
  PoseError e;
  e.long_side = d.x();
  e.up_lean = d.y();
  e.normal = d.z();
  const Eigen::Vector3d z = estimate.linear().col(2);
  e.face = std::atan2(z.cross(r.col(2)).norm(), z.dot(r.col(2)));
  const Eigen::Vector3d x = estimate.linear().col(0);
  e.in_plane = std::atan2(r.col(0).cross(x).dot(r.col(2)), r.col(0).dot(x));
  return e;
}

// Worst, root mean square and mean of one column.
struct Column {
  double worst = 0.0;
  double squares = 0.0;
  double sum = 0.0;
  int n = 0;
  void Add(double value) {
    worst = std::max(worst, std::abs(value));
    squares += value * value;
    sum += value;
    ++n;
  }
  double Rms() const { return n == 0 ? 0.0 : std::sqrt(squares / n); }
  double Mean() const { return n == 0 ? 0.0 : sum / n; }
};

std::vector<int> ParseList(const char* text) {
  std::vector<int> values;
  const char* p = text;
  while (*p != '\0') {
    char* end = nullptr;
    const long value = std::strtol(p, &end, 10);
    if (end == p) return {};
    values.push_back(static_cast<int>(value));
    p = end;
    if (*p == ',') ++p;
  }
  return values;
}

}  // namespace

int main(int argc, char** argv) {
  const char* usage =
      "usage: %s <cell.xml> [--pretty] [--trials T] [--first-seed S] "
      "[--noise] [--calibration-error MM,DEG] [--depths 0,10,20,29] "
      "[--no-creases] [--arm-home] [--verbose]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  bool pretty = false;
  bool noise = false;
  bool creases = true;
  bool arm_home = false;
  bool verbose = false;
  int trials = 20;
  int first_seed = 1;
  double calibration_mm = 0.0;
  double calibration_deg = 0.0;
  std::vector<int> depths = {0, 10, 20, 29};
  for (int i = 2; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--pretty") == 0) {
      pretty = true;
    } else if (std::strcmp(argv[i], "--noise") == 0) {
      noise = true;
    } else if (std::strcmp(argv[i], "--no-creases") == 0) {
      creases = false;
    } else if (std::strcmp(argv[i], "--arm-home") == 0) {
      arm_home = true;
    } else if (std::strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (std::strcmp(argv[i], "--trials") == 0 && has_value) {
      trials = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--first-seed") == 0 && has_value) {
      first_seed = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--depths") == 0 && has_value) {
      depths = ParseList(argv[++i]);
    } else if (std::strcmp(argv[i], "--calibration-error") == 0 && has_value) {
      if (std::sscanf(argv[++i], "%lf,%lf", &calibration_mm,
                      &calibration_deg) != 2) {
        std::fprintf(stderr, usage, argv[0]);
        return 1;
      }
    } else {
      std::fprintf(stderr, usage, argv[0]);
      return 1;
    }
  }
  if (trials < 1 || depths.empty()) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }

  std::string error;
  std::optional<cell::Cell> loaded = cell::Cell::Load(argv[1], &error);
  if (!loaded) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  cell::Cell& c = *loaded;
  for (int k : depths) {
    if (k < 0 || k > c.module_count()) {
      std::fprintf(stderr, "depth %d: the pallet holds modules 0 to %d\n", k,
                   c.module_count() - 1);
      return 1;
    }
  }

  // Design knowledge only: what perception is allowed to start from.
  const cell::PalletPrior prior = cell::PriorFromDesign(c.pallet());
  cell::FinderOptions options;
  options.outline.use_creases = creases;

  const char* row_format =
      "%u,%d,%.1f,%.1f,%.2f,%s,%.2f,%.2f,%.2f,%.3f,%.3f,%d,%.0f,%.0f\n";
  if (pretty) {
    row_format =
        "%4u %3d %6.1f %6.1f %6.2f  %-14s %7.2f %7.2f %7.2f  %6.3f %6.3f  "
        "%6d %6.0f %5.0f\n";
    std::printf("%4s %3s %-20s  %-14s %-23s  %-13s  %6s %-12s\n", "", "",
                "  true pallet offset", "", "    position error mm",
                " angle err deg", "", "  time ms");
    std::printf("%4s %3s %6s %6s %6s  %-14s %7s %7s %7s  %6s %6s  %6s %6s "
                "%5s\n",
                "seed", "k", "x mm", "y mm", "yaw", "outcome", "normal",
                "long", "lean", "face", "inpl", "points", "render", "fit");
  } else {
    std::printf("seed,k,pallet_x_mm,pallet_y_mm,pallet_yaw_deg,outcome,"
                "error_normal_mm,error_long_mm,error_lean_mm,"
                "error_face_deg,error_in_plane_deg,points,render_ms,fit_ms\n");
  }

  Column normal, long_side, up_lean, face, in_plane, render_ms, fit_ms;
  // Each error over the finder's own standard deviation for it: a root
  // mean square near 1 means the stated uncertainty is honest, well below 1
  // that it is cautious.
  Column z_normal, z_long, z_lean, z_face, z_in_plane;
  Column predicted_normal, predicted_long, predicted_lean, predicted_face;
  std::map<std::string, int> refusals;
  int pictures = 0;
  for (int seed = first_seed; seed < first_seed + trials; ++seed) {
    for (int k : depths) {
      ++pictures;
      c.Reset(static_cast<std::uint32_t>(seed));
      for (int i = 0; i < k; ++i) c.ParkModule(i);
      c.PutArm(arm_home ? c.arm_home() : c.arm_clear());
      c.StepFor(0.5);
      mj_forward(c.model(), c.data());

      const cell::CameraModel camera = c.DepthCamera();
      cell::DepthOptions depth_options;
      if (noise) {
        depth_options.noise.depth_noise = true;
        depth_options.noise.edge_effects = true;
        depth_options.noise.quantize = true;
        depth_options.noise.seed = static_cast<std::uint32_t>(1000 * seed + k);
      }
      const auto start = std::chrono::steady_clock::now();
      const cell::DepthImage image =
          cell::RenderDepth(*c.model(), c.data(), camera, depth_options);
      const double render = std::chrono::duration<double>(
                                std::chrono::steady_clock::now() - start)
                                .count();

      // The camera perception is told about.
      const Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
      const cell::CameraModel told = cell::Perturbed(
          camera, axis * (calibration_deg / kToDegrees),
          axis * (calibration_mm / kToMm));

      cell::FinderDiagnostics diagnostics;
      const std::optional<cell::ModuleEstimate> estimate =
          cell::FindFrontModule(image, told, prior, options, &diagnostics);

      const cell::TrueScene& scene = c.true_scene();
      // With every module taken there is nothing to find; the pallet's
      // design pose of the last module stands in for the truth, and any
      // answer at all is wrong.
      const bool empty = k == c.module_count();
      const Eigen::Isometry3d truth = c.TrueModulePose(empty ? k - 1 : k);
      render_ms.Add(render * 1e3);
      fit_ms.Add(diagnostics.seconds_total * 1e3);

      // What the calibration error alone does: every point, and so the
      // answer, moved by camera' * camera^-1.
      const Eigen::Isometry3d moved = told.pose * camera.pose.inverse();
      const PoseError predicted = ErrorOf(moved * truth, truth);
      predicted_normal.Add(predicted.normal * kToMm);
      predicted_long.Add(predicted.long_side * kToMm);
      predicted_lean.Add(predicted.up_lean * kToMm);
      predicted_face.Add(predicted.face * kToDegrees);

      const char* outcome = "ok";
      PoseError e;
      if (estimate && empty) {
        outcome = "FOUND NOTHING";
        ++refusals["an answer with no module left"];
      } else if (estimate) {
        e = ErrorOf(estimate->pose, truth);
        normal.Add(e.normal * kToMm);
        long_side.Add(e.long_side * kToMm);
        up_lean.Add(e.up_lean * kToMm);
        face.Add(e.face * kToDegrees);
        in_plane.Add(e.in_plane * kToDegrees);
        const cell::PoseUncertainty& sigma = estimate->sigma;
        z_normal.Add(e.normal / sigma.along_normal);
        z_long.Add(e.long_side / sigma.along_length);
        z_lean.Add(e.up_lean / sigma.up_lean);
        z_face.Add(e.face / sigma.face_direction);
        z_in_plane.Add(e.in_plane / sigma.in_plane_rotation);
      } else {
        outcome = cell::FinderFailureName(diagnostics.failure);
        ++refusals[std::string(outcome) + ": " + diagnostics.detail];
      }
      const double nan = std::nan("");
      std::printf(row_format, static_cast<unsigned>(seed), k,
                  scene.pallet_x * kToMm, scene.pallet_y * kToMm,
                  scene.pallet_yaw * kToDegrees, outcome,
                  estimate ? e.normal * kToMm : nan,
                  estimate ? e.long_side * kToMm : nan,
                  estimate ? e.up_lean * kToMm : nan,
                  estimate ? e.face * kToDegrees : nan,
                  estimate ? e.in_plane * kToDegrees : nan,
                  diagnostics.interior_points, render * 1e3,
                  diagnostics.seconds_total * 1e3);
      if (verbose) {
        const cell::FinderDiagnostics& d = diagnostics;
        std::fprintf(
            stderr,
            "  read %d crop %d ransac %d inliers %d interior %d dots %d | "
            "inlier mm %.1f-%.1f (ransac %.1f) rms %.2f mm (%.2f) lean err "
            "%.3f first %.3f deg | fit %.1f x %.1f, final %.1f x %.1f mm, "
            "spacing %.1f "
            "| top %d pts shift %.1f rms %.1f, bottom %d pts shift %.1f rms "
            "%.1f | view %d/%d (front %d behind %d missing %d)\n",
            d.pixels_read, d.points_in_crop, d.ransac_inliers, d.plane_inliers,
            d.interior_points, d.rectangle_dots, d.min_inlier_distance * kToMm,
            d.max_inlier_distance * kToMm, d.ransac_distance * kToMm,
            d.plane_rms * kToMm, d.plane_normalised_rms,
            d.lean_error * kToDegrees, d.first_lean_error * kToDegrees,
            d.fit_length * kToMm,
            d.fit_width * kToMm, d.length * kToMm, d.width * kToMm,
            d.spacing * kToMm, d.top.points, d.top.shift * kToMm,
            d.top.rms * kToMm, d.bottom.points, d.bottom.shift * kToMm,
            d.bottom.rms * kToMm, d.view.on_glass, d.view.pixels,
            d.view.in_front, d.view.behind, d.view.missing);
        std::fprintf(stderr,
                     "  ms: points+crop %.1f plane %.1f outline %.1f "
                     "view+refit %.1f total %.1f\n",
                     d.seconds_points * 1e3, d.seconds_plane * 1e3,
                     d.seconds_outline * 1e3, d.seconds_view * 1e3,
                     d.seconds_total * 1e3);
        if (estimate) {
          std::fprintf(stderr,
                       "  sigma: normal %.2f long %.2f lean %.2f mm, face "
                       "%.4f in-plane %.4f deg\n",
                       estimate->sigma.along_normal * kToMm,
                       estimate->sigma.along_length * kToMm,
                       estimate->sigma.up_lean * kToMm,
                       estimate->sigma.face_direction * kToDegrees,
                       estimate->sigma.in_plane_rotation * kToDegrees);
        }
      }
      std::fflush(stdout);
    }
  }

  std::fprintf(stderr,
               "\n%d pictures: %d seeds from %d x %zu depths; noise %s; "
               "calibration error %.2f mm, %.3f deg; creases %s; arm at %s\n",
               pictures, trials, first_seed, depths.size(),
               noise ? "on" : "off", calibration_mm, calibration_deg,
               creases ? "used" : "not used",
               arm_home ? "arm_home" : "arm_clear");
  std::fprintf(stderr, "%-24s %9s %9s %9s\n", "", "worst", "rms", "mean");
  auto line = [](const char* name, const Column& column) {
    std::fprintf(stderr, "%-24s %9.3f %9.3f %9.3f\n", name, column.worst,
                 column.Rms(), column.Mean());
  };
  line("normal mm", normal);
  line("long side mm", long_side);
  line("up the lean mm", up_lean);
  line("face direction deg", face);
  line("in-plane rotation deg", in_plane);
  line("render ms", render_ms);
  line("fit ms", fit_ms);
  std::fprintf(stderr, "error over the finder's own sigma:\n");
  line("  normal", z_normal);
  line("  long side", z_long);
  line("  up the lean", z_lean);
  line("  face direction", z_face);
  line("  in-plane rotation", z_in_plane);
  if (calibration_mm != 0.0 || calibration_deg != 0.0) {
    std::fprintf(stderr, "the calibration error alone moves the module by:\n");
    line("  normal mm", predicted_normal);
    line("  long side mm", predicted_long);
    line("  up the lean mm", predicted_lean);
    line("  face direction deg", predicted_face);
  }
  const int failures = pictures - normal.n;
  std::fprintf(stderr, "%d of %d refused\n", failures, pictures);
  for (const auto& [reason, count] : refusals) {
    std::fprintf(stderr, "  %3d  %s\n", count, reason.c_str());
  }
  return 0;
}
