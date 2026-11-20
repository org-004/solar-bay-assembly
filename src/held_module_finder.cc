#include "src/held_module_finder.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/camera_model.h"
#include "src/depth_camera.h"
#include "src/module_finder.h"
#include "src/plane_fit.h"
#include "src/rectangle_fit.h"

namespace cell {
namespace {

using Clock = std::chrono::steady_clock;

constexpr double kToDegrees = 180.0 / std::numbers::pi;

double SecondsSince(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

std::string Format(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// Half a turn about the module's long side: what takes the frame of "the
// underside as if it were a glass" (+z toward the camera) to the module's
// own frame (+z out of the real glass), and back.
Eigen::Isometry3d HalfTurnAboutX() {
  Eigen::Isometry3d turn = Eigen::Isometry3d::Identity();
  turn.linear() = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  return turn;
}

double AngleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

// The rectangle's own coordinates of a 2D point: along its axis from its
// centre, and across it.
Eigen::Vector2d InRectangle(const OrientedRectangle& r,
                            const Eigen::Vector2d& q) {
  const Eigen::Vector2d d = q - r.centre;
  return {d.dot(r.axis), d.dot(r.Across())};
}

// A side of FitRectangle's answer as a standard error of its position: its
// edge dots' scatter over the root of their number, and what is left of the
// pixel grid, `grid` times the dot spacing. As SideSigma in
// src/module_finder.cc, for a side that is a depth step; see kStepGrid.
double SideSigma(const SideFit& side, double spacing, double grid) {
  const double n = std::max(side.dots_used, 1);
  return std::hypot(side.rms_residual / std::sqrt(n), grid * spacing);
}

// What is left of the pixel grid at a side that is a step in depth, in dot
// spacings, after FitRectangle's dot counting. Square-on, the camera's rows
// and columns would run along the module's sides, the case the counting
// fixes least well (src/rectangle_fit.h: up to half a spacing at a side
// whose rows run exactly along it); on the rig, with the rows along the
// long side, the centre came out up to 1.35 mm off (0.4 spacings) and the
// turn 0.07 degrees. So the camera is turned 5 degrees on its axis
// (models/check_scene.xml), and then over 100 noiseless pictures the centre
// was off by 0.065 mm at worst, 0.02 spacings; this says 0.05, with margin.
constexpr double kStepGrid = 0.05;

// The area of a polygon, by the shoelace formula.
double PolygonArea(const std::array<Eigen::Vector2d, 4>& corners) {
  double twice = 0.0;
  for (int k = 0; k < 4; ++k) {
    const Eigen::Vector2d& a = corners[k];
    const Eigen::Vector2d& b = corners[(k + 1) % 4];
    twice += a.x() * b.y() - b.x() * a.y();
  }
  return 0.5 * std::abs(twice);
}

}  // namespace

const char* HeldFinderFailureName(HeldFinderFailure failure) {
  switch (failure) {
    case HeldFinderFailure::kNone: return "none";
    case HeldFinderFailure::kBadInput: return "bad input";
    case HeldFinderFailure::kNotInView: return "not in view";
    case HeldFinderFailure::kTooFewPoints: return "too few points";
    case HeldFinderFailure::kNoPlane: return "no plane";
    case HeldFinderFailure::kWrongTilt: return "wrong tilt";
    case HeldFinderFailure::kNoRectangle: return "no rectangle";
    case HeldFinderFailure::kSizeMismatch: return "size mismatch";
    case HeldFinderFailure::kViewBlocked: return "view blocked";
    case HeldFinderFailure::kFarFromExpected: return "far from expected";
  }
  return "unknown";
}

std::optional<HeldModuleEstimate> FindHeldModule(
    const DepthImage& image, const CameraModel& camera,
    const Eigen::Isometry3d& expected_module_pose_in_world,
    const HeldFinderOptions& options, HeldFinderDiagnostics* diagnostics) {
  const Clock::time_point start = Clock::now();
  HeldFinderDiagnostics diag;
  auto refuse = [&](HeldFinderFailure failure, std::string detail) {
    diag.failure = failure;
    diag.detail = std::move(detail);
    diag.seconds_total = SecondsSince(start);
    if (diagnostics) *diagnostics = diag;
    return std::nullopt;
  };

  if (image.width != camera.width || image.height != camera.height ||
      image.size() == 0 ||
      static_cast<int>(image.depth.size()) != image.size()) {
    return refuse(HeldFinderFailure::kBadInput,
                  "the image is not the camera's size, or empty");
  }

  const Eigen::Isometry3d& expected = expected_module_pose_in_world;
  const double half_length = 0.5 * options.module_length;
  const double half_width = 0.5 * options.module_width;
  const double half_thickness = 0.5 * options.module_thickness;
  // How far the tolerances can move a corner of the underside: the shift,
  // plus the turn times the corner's distance from the centre.
  const double reach = options.position_tolerance +
                       options.angle_tolerance *
                           std::hypot(half_length, half_width);
  const double margin = reach + options.crop_extra;
  // The expected underside, in the expected module's frame, and its
  // outward normal (toward the camera) in the world.
  const Eigen::Vector3d underside_centre =
      expected * Eigen::Vector3d(0.0, 0.0, -half_thickness);
  const Eigen::Vector3d expected_normal = -expected.linear().col(2);

  // 1. The expected underside, grown by the reach, wholly in the picture
  // and facing the camera.
  {
    const Eigen::Vector3d to_camera =
        camera.pose.translation() - underside_centre;
    if (!(to_camera.dot(expected_normal) > 0.0)) {
      return refuse(HeldFinderFailure::kNotInView,
                    "the expected underside faces away from the camera");
    }
    std::array<Eigen::Vector2d, 4> pixels;
    std::array<Eigen::Vector2d, 4> grown;
    const double sx[4] = {-1, 1, 1, -1};
    const double sy[4] = {-1, -1, 1, 1};
    for (int k = 0; k < 4; ++k) {
      const std::optional<Eigen::Vector2d> exact = Project(
          camera, expected * Eigen::Vector3d(sx[k] * half_length,
                                             sy[k] * half_width,
                                             -half_thickness));
      const std::optional<Eigen::Vector2d> wide = Project(
          camera, expected * Eigen::Vector3d(sx[k] * (half_length + reach),
                                             sy[k] * (half_width + reach),
                                             -half_thickness));
      if (!exact || !wide) {
        return refuse(HeldFinderFailure::kNotInView,
                      "a corner of the expected underside is behind the "
                      "camera");
      }
      pixels[k] = *exact;
      grown[k] = *wide;
    }
    diag.expected_pixels = PolygonArea(pixels);
    for (const Eigen::Vector2d& p : grown) {
      const double m = options.image_margin;
      if (p.x() < m || p.x() > camera.width - m || p.y() < m ||
          p.y() > camera.height - m) {
        return refuse(
            HeldFinderFailure::kNotInView,
            Format("the expected underside, grown by %.0f mm, reaches pixel "
                   "(%.0f, %.0f), outside the %d x %d image",
                   reach * 1e3, p.x(), p.y(), camera.width, camera.height));
      }
    }
  }

  // 2 and 3. Points in the world, and only where the underside can be.
  Clock::time_point step = Clock::now();
  const PixelPoints cloud = ImageToArmFrame(image, camera);
  diag.pixels_read = static_cast<int>(cloud.points.size());
  const Eigen::Isometry3d to_module = expected.inverse();
  Points points;
  points.reserve(cloud.points.size());
  for (const Eigen::Vector3d& p : cloud.points) {
    const Eigen::Vector3d local = to_module * p;
    if (std::abs(local.x()) <= half_length + margin &&
        std::abs(local.y()) <= half_width + margin &&
        std::abs(local.z() + half_thickness) <= margin) {
      points.push_back(p);
    }
  }
  diag.points_in_crop = static_cast<int>(points.size());
  diag.seconds_points = SecondsSince(step);
  const double needed = options.min_point_fraction * diag.expected_pixels;
  if (diag.points_in_crop < needed) {
    return refuse(
        HeldFinderFailure::kTooFewPoints,
        Format("%d points where the underside can be, fewer than %.0f%% of "
               "the %.0f pixels it should cover: no module on the tool",
               diag.points_in_crop, 100.0 * options.min_point_fraction,
               diag.expected_pixels));
  }

  // 4. The underside's plane. FindGlassPlane looks for the plane facing the
  // way its prior's glass does, and reads nothing else of the prior: here
  // the face it is to find is the underside, facing the camera.
  step = Clock::now();
  PalletPrior face;
  face.front_module = expected * HalfTurnAboutX();
  face.module_length = options.module_length;
  face.module_width = options.module_width;
  face.module_thickness = options.module_thickness;
  std::string why;
  const std::optional<PlaneFitResult> plane =
      FindGlassPlane(points, camera, face, options.plane, &why);
  diag.seconds_plane = SecondsSince(step);
  if (!plane) return refuse(HeldFinderFailure::kNoPlane, why);
  diag.plane_inliers = static_cast<int>(plane->inliers.size());
  diag.first_tilt = AngleBetween(plane->plane.normal, expected_normal);
  if (diag.first_tilt > options.angle_tolerance) {
    return refuse(HeldFinderFailure::kWrongTilt,
                  Format("the underside is tipped %.2f deg from where the "
                         "tool should hold it, more than %.2f deg",
                         diag.first_tilt * kToDegrees,
                         options.angle_tolerance * kToDegrees));
  }

  // 5 and 6. Flattened with x along the expected long side, and the outline.
  step = Clock::now();
  PlaneFrame frame;
  frame.normal = plane->plane.normal;
  const Eigen::Vector3d long_side = expected.linear().col(0);
  frame.x = (long_side - frame.normal * frame.normal.dot(long_side))
                .normalized();
  frame.y = frame.normal.cross(frame.x);
  frame.origin = underside_centre -
                 frame.normal * plane->plane.SignedDistance(underside_centre);
  const Points2d dots = Flatten(points, plane->inliers, frame);
  RectangleRefusal rectangle_refusal;
  const std::optional<RectangleFit> fit =
      FitRectangle(dots, options.module_length, options.module_width,
                   options.rectangle, &rectangle_refusal);
  diag.seconds_outline = SecondsSince(step);
  if (!fit) {
    return refuse(rectangle_refusal.reason == RectangleFitFailure::kSizeMismatch
                      ? HeldFinderFailure::kSizeMismatch
                      : HeldFinderFailure::kNoRectangle,
                  rectangle_refusal.detail);
  }
  const OrientedRectangle& outline = fit->rectangle;
  diag.rectangle_dots = fit->dots_kept;
  diag.length = outline.length;
  diag.width = outline.width;
  diag.spacing = fit->spacing;
  for (int k = 0; k < 4; ++k) {
    diag.side_rms[k] = fit->sides[k].rms_residual;
    diag.edge_correction[k] = fit->sides[k].edge_correction;
  }

  // The plane once more, from the underside's inside only.
  step = Clock::now();
  std::vector<int> interior;
  const double inner_s = 0.5 * outline.length - options.interior_margin;
  const double inner_t = 0.5 * outline.width - options.interior_margin;
  for (int i : plane->inliers) {
    const Eigen::Vector2d st = InRectangle(outline, frame.Flatten(points[i]));
    if (std::abs(st.x()) <= inner_s && std::abs(st.y()) <= inner_t) {
      interior.push_back(i);
    }
  }
  diag.interior_points = static_cast<int>(interior.size());
  const std::optional<WeightedPlane> final_plane = FitWeightedPlane(
      points, interior, camera, options.plane.noise, plane->plane.normal);
  if (!final_plane) {
    return refuse(HeldFinderFailure::kNoPlane,
                  "the underside's inside gave no plane on its own");
  }
  const Eigen::Vector3d n = final_plane->plane.normal;
  diag.tilt = AngleBetween(n, expected_normal);
  {
    // Each point's off-plane noise as OffPlaneSigma works it out: its depth
    // noise times |n . (p - c)| / depth.
    const Eigen::Vector3d c = camera.pose.translation();
    const Eigen::Vector3d viewing_axis = -camera.pose.linear().col(2);
    double sum = 0.0, normalised = 0.0;
    for (int i : interior) {
      const double d = final_plane->plane.SignedDistance(points[i]);
      const double depth = viewing_axis.dot(points[i] - c);
      const double sigma = options.plane.noise.Sigma(depth) *
                           std::abs(n.dot(points[i] - c)) / depth;
      sum += d * d;
      normalised += d * d / (sigma * sigma);
    }
    const double count = std::max<double>(interior.size(), 1.0);
    diag.plane_rms = std::sqrt(sum / count);
    diag.plane_normalised_rms = std::sqrt(normalised / count);
  }

  // The outline was measured on the first plane; its centre and axis go
  // onto the final one at right angles (the two differ by hundredths of a
  // degree).
  Eigen::Vector3d centre = frame.Lift(outline.centre);
  centre -= n * final_plane->plane.SignedDistance(centre);
  Eigen::Vector3d axis = outline.axis.x() * frame.x + outline.axis.y() * frame.y;
  axis = (axis - n * n.dot(axis)).normalized();
  PlaneFrame final_frame;
  final_frame.normal = n;
  final_frame.x = axis;
  final_frame.y = n.cross(axis);
  final_frame.origin = centre;
  OrientedRectangle on_final = outline;
  on_final.centre = Eigen::Vector2d::Zero();
  on_final.axis = Eigen::Vector2d::UnitX();

  // 7. Nothing between the camera and the underside, and no holes in it.
  diag.view = CheckView(image, camera, final_plane->plane, final_frame,
                        on_final, options.plane.noise, options.view);
  diag.seconds_view = SecondsSince(step);
  if (diag.view.BlockedFraction() > options.view.max_blocked_fraction) {
    return refuse(
        HeldFinderFailure::kViewBlocked,
        Format("%.1f%% of %d pixels inside the outline do not read the "
               "underside (%d in front, %d behind, %d no reading); at most "
               "%.1f%%",
               100.0 * diag.view.BlockedFraction(), diag.view.pixels,
               diag.view.in_front, diag.view.behind, diag.view.missing,
               100.0 * options.view.max_blocked_fraction));
  }

  // 8. The pose: the frame of a module whose glass is the face seen, turned
  // half a turn about its long side.
  HeldModuleEstimate estimate;
  estimate.pose =
      ModulePoseFromOutline(final_frame, on_final, options.module_thickness) *
      HalfTurnAboutX();
  estimate.underside_centre = centre;
  estimate.underside_normal = n;
  estimate.length = outline.length;
  estimate.width = outline.width;

  // 9. Near where the tool should hold it.
  const Eigen::Isometry3d off = to_module * estimate.pose;
  diag.offset_from_expected = off.translation();
  diag.angle_from_expected = Eigen::AngleAxisd(off.linear()).angle();
  if (diag.offset_from_expected.lpNorm<Eigen::Infinity>() >
          options.position_tolerance ||
      diag.angle_from_expected > options.angle_tolerance) {
    const Eigen::Vector3d mm = diag.offset_from_expected * 1e3;
    return refuse(
        HeldFinderFailure::kFarFromExpected,
        Format("the module is (%.1f, %.1f, %.1f) mm and %.2f deg from where "
               "the tool should hold it, more than %.0f mm or %.1f deg: it "
               "has slipped",
               mm.x(), mm.y(), mm.z(), diag.angle_from_expected * kToDegrees,
               options.position_tolerance * 1e3,
               options.angle_tolerance * kToDegrees));
  }

  // Standard errors: of the plane from its fit, of the centre and the turn
  // from the four sides' edge dots.
  const double spacing = fit->spacing;
  const double sigma_side[4] = {
      SideSigma(fit->sides[0], spacing, kStepGrid),
      SideSigma(fit->sides[1], spacing, kStepGrid),
      SideSigma(fit->sides[2], spacing, kStepGrid),
      SideSigma(fit->sides[3], spacing, kStepGrid)};
  estimate.sigma.along_normal = final_plane->sigma_offset;
  estimate.sigma.face_direction = final_plane->sigma_tilt;
  estimate.sigma.along_length = 0.5 * std::hypot(sigma_side[1], sigma_side[3]);
  estimate.sigma.up_lean = 0.5 * std::hypot(sigma_side[0], sigma_side[2]);
  {
    // Each long side's direction is known to its scatter over the spread
    // of its dots, length / sqrt(12) for evenly spread dots.
    double sum = 0.0;
    for (int k : {0, 2}) {
      const SideFit& side = fit->sides[k];
      const double count = std::max(side.dots_used, 1);
      const double s = side.rms_residual * std::sqrt(12.0) /
                       (std::sqrt(count) * outline.length);
      sum += s * s;
    }
    // And the grid's share at each end of the side, over its length.
    const double grid = std::sqrt(2.0) * kStepGrid * spacing / outline.length;
    estimate.sigma.in_plane_rotation = std::hypot(0.5 * std::sqrt(sum), grid);
  }

  diag.seconds_total = SecondsSince(start);
  estimate.diagnostics = diag;
  if (diagnostics) *diagnostics = diag;
  return estimate;
}

// ------------------------------------------------------- the frame algebra

Eigen::Isometry3d DesignModuleInTip(double thickness, double gap) {
  Eigen::Isometry3d module_in_tip = HalfTurnAboutX();
  module_in_tip.translation() = Eigen::Vector3d(0.0, 0.0, gap + 0.5 * thickness);
  return module_in_tip;
}

Eigen::Isometry3d GripOffsetFromCheck(
    const Eigen::Isometry3d& measured_module_pose_world,
    const Eigen::Isometry3d& tool_tip_pose_world) {
  // module in world = tip in world * module in tip, solved for the last.
  return tool_tip_pose_world.inverse() * measured_module_pose_world;
}

Eigen::Isometry3d CorrectedTipTarget(
    const Eigen::Isometry3d& desired_module_pose_world,
    const Eigen::Isometry3d& measured_module_in_tip) {
  // module in world = tip in world * module in tip, solved for the first.
  return desired_module_pose_world * measured_module_in_tip.inverse();
}

}  // namespace cell
