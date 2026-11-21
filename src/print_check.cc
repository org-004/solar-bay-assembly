// The check-after-pickup stage's table: the grip offset recovered from one
// depth image of the held module's underside, against the truth, over many
// seeded grip offsets on the rig (models/check_scene.xml).
//
//   bazel run --config=opt //:print_check -- [--pretty] [--trials N]
//       [--first-seed S] [--noise] [--calibration MM,DEG] [--no-roof]
//       [--position MM] [--angle DEG] [--verbose] [--pgm FILE]
//
//   --pretty      an aligned table instead of CSV
//   --trials N    seeds S to S + N - 1, each a grip offset (default 50)
//   --first-seed S  (default 1)
//   --noise       switch on every effect of the camera's noise model
//   --calibration MM,DEG
//                 hand the finder a camera model that is off: shifted MM mm
//                 along the camera's own +X and turned DEG degrees about
//                 that axis, as Perturbed in src/camera_model.h does
//   --no-roof     take the rig's roof away: nothing is seen beyond the
//                 module's edges, so they only drop out, never fly
//   --position MM, --angle DEG
//                 the grip offsets drawn: each of the three shifts uniform
//                 in +-MM (default 10) and each of the three turns in +-DEG
//                 (default 1.5)
//   --verbose     after each row, the finder's diagnostics on stderr
//   --pgm FILE    one trial only (seed --first-seed, default 1; --trials is
//                 ignored): write that trial's depth image, with --noise the
//                 noisy one, as a 16-bit PGM (each pixel the depth in whole
//                 millimetres, 0 where there is no reading), and print, in
//                 place of the table, CSV on stdout: the outline the finder
//                 found, its four corners (the module's -x-y, +x-y, +x+y,
//                 -x+y, on the underside) and its centre, projected with the
//                 calibrated camera model into pixels, as lines
//                 `item,u_px,v_px` (u, v as Project gives them: the centre of
//                 the pixel in column c and row r is (c + 0.5, r + 0.5)); then
//                 the lines `measured_offset,...` and `true_offset,...`:
//                 x, y, z in mm, then rx, ry, rz in degrees, from the design
//                 grip in the design module's frame. FILE should be absolute
//                 (bazel run starts in the runfiles directory).
//
//   bazel run --config=opt //:print_check -- --study [--cell CELL.xml]
//
//   --study       instead of the table, the placement study for the real
//                 cell (src/check_placement.h): where check_camera should
//                 stand and the arm's check pose above it. On a cell without
//                 check_camera (models/cell.xml, the default --cell) it
//                 searches and prints the recommendation and the XML for
//                 the camera; on a copy with that XML added it verifies the
//                 check pose in full, pictures included.
//
// For each seed: the tool tip at the rig's nominal check pose, moved by up
// to 1 cm and turned up to 0.5 degree about the vertical (the arm is never
// exactly there; the expected pose follows the tool), module_0 gripped at
// the drawn offset from the design grip by the real suction weld, a moment
// of physics, the depth image, FindHeldModule with the expected pose
// tip * DesignModuleInTip(), and the grip offset GripOffsetFromCheck. One
// row per seed: the true offset from the design grip (mm and degrees, in
// the design module's frame), the outcome, and the error of the recovered
// module-in-tip against the truth (Suction::HeldPoseInTip), in the true
// module's frame: along its long side x, short side y and normal z (mm),
// and turned about x, y and z (degrees; z is the vertical at the check
// pose). Then the size the finder read against the design, the points it
// used and the time. Then a summary: worst, root mean square and mean of
// every column, the errors over the finder's own standard deviations, and
// the refusals.
//
// The truth is read here, to score, and nowhere in the finder.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <numbers>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/camera_model.h"
#include "src/check_placement.h"
#include "src/check_rig.h"
#include "src/depth_camera.h"
#include "src/held_module_finder.h"

namespace {

constexpr double kToDegrees = 180.0 / std::numbers::pi;
constexpr double kToMm = 1000.0;

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

// The tool tip for trial `seed`: the nominal one moved up to 1 cm in each
// direction and turned up to half a degree about the vertical.
Eigen::Isometry3d TrialTip(const Eigen::Isometry3d& nominal,
                           std::uint32_t seed) {
  const Eigen::Isometry3d jiggle = cell::DrawGripOffset(
      seed + 100000u, 0.01, 0.5 * std::numbers::pi / 180.0);
  // Only the turn about the vertical: the tool stays pointing down.
  const Eigen::Matrix<double, 6, 1> six = cell::PoseError6(
      jiggle, Eigen::Isometry3d::Identity());
  Eigen::Isometry3d tip = nominal;
  tip.translation() += six.head<3>();
  tip.linear() =
      Eigen::AngleAxisd(six[5], Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      nominal.linear();
  return tip;
}

// print_depth's 16-bit PGM: each pixel the depth in whole millimetres, 0
// where there is no reading.
bool WritePgm(const std::string& path, const cell::DepthImage& image) {
  std::FILE* file = std::fopen(path.c_str(), "wb");
  if (file == nullptr) return false;
  std::fprintf(file, "P5\n%d %d\n65535\n", image.width, image.height);
  for (float z : image.depth) {
    const long mm = std::isfinite(z) ? std::lround(z * 1000.0) : 0;
    const unsigned value = static_cast<unsigned>(std::clamp(mm, 0L, 65535L));
    std::fputc(static_cast<int>(value >> 8), file);  // big-endian
    std::fputc(static_cast<int>(value & 0xff), file);
  }
  return std::fclose(file) == 0;
}

}  // namespace

int main(int argc, char** argv) {
  const char* usage =
      "usage: %s <check_scene.xml> [--cell <cell.xml>] [--study] [--pretty] "
      "[--trials N] [--first-seed S] [--noise] [--calibration MM,DEG] "
      "[--no-roof] [--position MM] [--angle DEG] [--verbose] [--pgm FILE]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  bool pretty = false, noise = false, roof = true, verbose = false;
  bool study = false;
  std::string cell_path, pgm;
  int trials = 50, first_seed = 1;
  double calibration_mm = 0.0, calibration_deg = 0.0;
  double position_mm = 10.0, angle_deg = 1.5;
  for (int i = 2; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--pretty") == 0) {
      pretty = true;
    } else if (std::strcmp(argv[i], "--noise") == 0) {
      noise = true;
    } else if (std::strcmp(argv[i], "--no-roof") == 0) {
      roof = false;
    } else if (std::strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (std::strcmp(argv[i], "--study") == 0) {
      study = true;
    } else if (std::strcmp(argv[i], "--cell") == 0 && has_value) {
      cell_path = argv[++i];
    } else if (std::strcmp(argv[i], "--pgm") == 0 && has_value) {
      pgm = argv[++i];
    } else if (std::strcmp(argv[i], "--trials") == 0 && has_value) {
      trials = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--first-seed") == 0 && has_value) {
      first_seed = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--position") == 0 && has_value) {
      position_mm = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--angle") == 0 && has_value) {
      angle_deg = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--calibration") == 0 && has_value) {
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
  if (trials < 1) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }

  if (study) {
    if (cell_path.empty()) {
      std::fprintf(stderr, "--study needs --cell <cell.xml>\n");
      return 1;
    }
    return cell::RunPlacementStudy(cell_path);
  }

  std::string error;
  std::optional<cell::CheckRig> loaded = cell::CheckRig::Load(argv[1], &error);
  if (!loaded) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  cell::CheckRig& rig = *loaded;
  rig.SetRoof(roof);
  const Eigen::Isometry3d design = cell::DesignModuleInTip();

  const char* row_format =
      "%u,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
      "%.2f,%.2f,%d,%.0f,%.0f\n";
  if (!pgm.empty()) {
    trials = 1;
  } else if (pretty) {
    row_format =
        "%4u %6.2f %6.2f %6.2f %6.3f %6.3f %6.3f  %-14s %6.3f %6.3f %6.3f "
        "%7.4f %7.4f %7.4f  %6.2f %6.2f %7d %5.0f %4.0f\n";
    std::printf("%4s %-41s  %-14s %-20s %-23s  %-13s %7s %-10s\n", "",
                "   true offset from design: mm, deg", "",
                " error mm", " error deg", " size err mm", "", " time ms");
    std::printf("%4s %6s %6s %6s %6s %6s %6s  %-14s %6s %6s %6s %7s %7s %7s  "
                "%6s %6s %7s %5s %4s\n",
                "seed", "x", "y", "z", "rx", "ry", "rz", "outcome", "x", "y",
                "z", "rx", "ry", "rz", "length", "width", "points", "render",
                "fit");
  } else {
    std::printf("seed,offset_x_mm,offset_y_mm,offset_z_mm,offset_rx_deg,"
                "offset_ry_deg,offset_rz_deg,outcome,error_x_mm,error_y_mm,"
                "error_z_mm,error_rx_deg,error_ry_deg,error_rz_deg,"
                "length_error_mm,width_error_mm,points,render_ms,fit_ms\n");
  }

  Column error_col[6], z_col[6], length_col, width_col, render_ms, fit_ms;
  Column predicted[6];
  std::map<std::string, int> refusals;
  for (int seed = first_seed; seed < first_seed + trials; ++seed) {
    const std::uint32_t s = static_cast<std::uint32_t>(seed);
    const Eigen::Isometry3d tip = TrialTip(rig.nominal_tip(), s);
    const Eigen::Isometry3d offset = cell::DrawGripOffset(
        s, position_mm / kToMm, angle_deg / kToDegrees);
    std::optional<cell::CheckShot> shot = rig.Take(tip, offset, &error);
    if (!shot) {
      std::fprintf(stderr, "seed %d: the rig could not grip: %s\n", seed,
                   error.c_str());
      return 1;
    }
    cell::DepthImage image = shot->image;
    if (noise) {
      cell::DepthNoise n;
      n.depth_noise = true;
      n.edge_effects = true;
      n.quantize = true;
      n.seed = 1000u + s;
      cell::AddDepthNoise(&image, n);
    }
    const Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
    const cell::CameraModel told =
        cell::Perturbed(shot->camera, axis * (calibration_deg / kToDegrees),
                        axis * (calibration_mm / kToMm));

    // What the control code has: the tool tip from the joint angles (here
    // the carriage), and the design grip.
    const Eigen::Isometry3d expected = shot->tip * design;
    cell::HeldFinderDiagnostics d;
    const std::optional<cell::HeldModuleEstimate> estimate =
        cell::FindHeldModule(image, told, expected, {}, &d);
    render_ms.Add(shot->render_seconds * 1e3);
    fit_ms.Add(d.seconds_total * 1e3);

    // The true offset from the design grip, in the design module's frame.
    const Eigen::Matrix<double, 6, 1> truth_offset =
        cell::PoseError6(shot->truth_in_tip, design);
    // What the calibration error alone does: the module moved by
    // told * camera^-1, its offset in the tip with it.
    const Eigen::Isometry3d moved = told.pose * shot->camera.pose.inverse();
    const Eigen::Matrix<double, 6, 1> shift =
        cell::PoseError6(moved * shot->truth_world, shot->truth_world);
    for (int k = 0; k < 6; ++k) {
      predicted[k].Add(k < 3 ? shift[k] * kToMm : shift[k] * kToDegrees);
    }

    if (!pgm.empty()) {
      if (!WritePgm(pgm, image)) {
        std::fprintf(stderr, "could not write %s\n", pgm.c_str());
        return 1;
      }
      std::fprintf(stderr, "wrote %s\n", pgm.c_str());
      if (!estimate) {
        std::fprintf(stderr, "the finder refused: %s: %s\n",
                     cell::HeldFinderFailureName(d.failure), d.detail.c_str());
        return 1;
      }
      // The outline on the underside, in the module's axes.
      const Eigen::Matrix3d axes = estimate->pose.linear();
      const Eigen::Vector3d origin = estimate->underside_centre;
      const double hx = 0.5 * estimate->length, hy = 0.5 * estimate->width;
      const double signs[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
      std::printf("item,u_px,v_px\n");
      for (int k = 0; k < 5; ++k) {
        const Eigen::Vector3d p =
            k < 4 ? origin + axes.col(0) * (signs[k][0] * hx) +
                        axes.col(1) * (signs[k][1] * hy)
                  : origin;
        const std::optional<Eigen::Vector2d> uv = cell::Project(told, p);
        if (!uv) {
          std::fprintf(stderr, "outline point %d is behind the camera\n", k);
          return 1;
        }
        if (k < 4) {
          std::printf("corner%d,%.2f,%.2f\n", k, uv->x(), uv->y());
        } else {
          std::printf("centre,%.2f,%.2f\n", uv->x(), uv->y());
        }
      }
      const Eigen::Matrix<double, 6, 1> measured = cell::PoseError6(
          cell::GripOffsetFromCheck(estimate->pose, shot->tip), design);
      for (const auto& [name, v] :
           {std::pair{"measured_offset", measured},
            std::pair{"true_offset", truth_offset}}) {
        std::printf("%s,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f\n", name, v[0] * kToMm,
                    v[1] * kToMm, v[2] * kToMm, v[3] * kToDegrees,
                    v[4] * kToDegrees, v[5] * kToDegrees);
      }
      return 0;
    }

    const char* outcome = "ok";
    Eigen::Matrix<double, 6, 1> e = Eigen::Matrix<double, 6, 1>::Constant(
        std::nan(""));
    double length_error = std::nan(""), width_error = std::nan("");
    if (estimate) {
      const Eigen::Isometry3d measured_in_tip =
          cell::GripOffsetFromCheck(estimate->pose, shot->tip);
      e = cell::PoseError6(measured_in_tip, shot->truth_in_tip);
      for (int k = 0; k < 6; ++k) {
        error_col[k].Add(k < 3 ? e[k] * kToMm : e[k] * kToDegrees);
      }
      const cell::PoseUncertainty& sigma = estimate->sigma;
      const double sigmas[6] = {sigma.along_length, sigma.up_lean,
                                sigma.along_normal, sigma.face_direction,
                                sigma.face_direction, sigma.in_plane_rotation};
      for (int k = 0; k < 6; ++k) z_col[k].Add(e[k] / sigmas[k]);
      length_error = (estimate->length - cell::kModuleLength) * kToMm;
      width_error = (estimate->width - cell::kModuleWidth) * kToMm;
      length_col.Add(length_error);
      width_col.Add(width_error);
    } else {
      outcome = cell::HeldFinderFailureName(d.failure);
      ++refusals[std::string(outcome) + ": " + d.detail];
    }
    std::printf(row_format, s, truth_offset[0] * kToMm,
                truth_offset[1] * kToMm, truth_offset[2] * kToMm,
                truth_offset[3] * kToDegrees, truth_offset[4] * kToDegrees,
                truth_offset[5] * kToDegrees, outcome, e[0] * kToMm,
                e[1] * kToMm, e[2] * kToMm, e[3] * kToDegrees,
                e[4] * kToDegrees, e[5] * kToDegrees, length_error,
                width_error, d.interior_points, shot->render_seconds * 1e3,
                d.seconds_total * 1e3);
    if (verbose) {
      std::fprintf(
          stderr,
          "  read %d crop %d (expected %.0f px) inliers %d interior %d dots "
          "%d | tilt %.3f first %.3f deg rms %.2f mm (%.2f) | %.1f x %.1f "
          "mm spacing %.2f | side rms %.2f %.2f %.2f %.2f mm, corrections "
          "%.2f %.2f %.2f %.2f mm | view %d/%d (front %d behind %d missing "
          "%d)\n",
          d.pixels_read, d.points_in_crop, d.expected_pixels, d.plane_inliers,
          d.interior_points, d.rectangle_dots, d.tilt * kToDegrees,
          d.first_tilt * kToDegrees, d.plane_rms * kToMm,
          d.plane_normalised_rms, d.length * kToMm, d.width * kToMm,
          d.spacing * kToMm, d.side_rms[0] * kToMm, d.side_rms[1] * kToMm,
          d.side_rms[2] * kToMm, d.side_rms[3] * kToMm,
          d.edge_correction[0] * kToMm, d.edge_correction[1] * kToMm,
          d.edge_correction[2] * kToMm, d.edge_correction[3] * kToMm,
          d.view.on_glass, d.view.pixels, d.view.in_front, d.view.behind,
          d.view.missing);
      std::fprintf(stderr,
                   "  ms: points+crop %.1f plane %.1f outline %.1f "
                   "view+refit %.1f total %.1f\n",
                   d.seconds_points * 1e3, d.seconds_plane * 1e3,
                   d.seconds_outline * 1e3, d.seconds_view * 1e3,
                   d.seconds_total * 1e3);
    }
    std::fflush(stdout);
  }

  std::fprintf(stderr,
               "\n%d grip offsets from seed %d (+-%.1f mm, +-%.2f deg); noise "
               "%s; roof %s; calibration error %.2f mm, %.3f deg\n",
               trials, first_seed, position_mm, angle_deg,
               noise ? "on" : "off", roof ? "on" : "off", calibration_mm,
               calibration_deg);
  std::fprintf(stderr, "%-30s %9s %9s %9s\n", "", "worst", "rms", "mean");
  auto line = [](const char* name, const Column& column) {
    std::fprintf(stderr, "%-30s %9.4f %9.4f %9.4f\n", name, column.worst,
                 column.Rms(), column.Mean());
  };
  const char* names[6] = {"x, long side mm", "y, short side mm",
                          "z, normal mm", "about x deg", "about y deg",
                          "about z (vertical) deg"};
  for (int k = 0; k < 6; ++k) line(names[k], error_col[k]);
  line("length read - design mm", length_col);
  line("width read - design mm", width_col);
  line("render ms", render_ms);
  line("fit ms", fit_ms);
  std::fprintf(stderr, "error over the finder's own sigma:\n");
  for (int k = 0; k < 6; ++k) {
    const std::string name = std::string("  ") + names[k];
    line(name.c_str(), z_col[k]);
  }
  if (calibration_mm != 0.0 || calibration_deg != 0.0) {
    std::fprintf(stderr,
                 "the calibration error alone moves the module by:\n");
    for (int k = 0; k < 6; ++k) {
      const std::string name = std::string("  ") + names[k];
      line(name.c_str(), predicted[k]);
    }
  }
  const int failures = trials - error_col[0].n;
  std::fprintf(stderr, "%d of %d refused\n", failures, trials);
  for (const auto& [reason, count] : refusals) {
    std::fprintf(stderr, "  %3d  %s\n", count, reason.c_str());
  }
  return 0;
}
