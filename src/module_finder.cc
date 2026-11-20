#include "src/module_finder.h"

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
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include "src/camera_model.h"
#include "src/depth_camera.h"
#include "src/pallet.h"
#include "src/plane_fit.h"
#include "src/rectangle_fit.h"

namespace cell {
namespace {

using Clock = std::chrono::steady_clock;

const Eigen::Vector3d kUp = Eigen::Vector3d::UnitZ();

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

// The camera's viewing axis in the world: its -Z.
Eigen::Vector3d ViewingAxis(const CameraModel& camera) {
  return -camera.pose.linear().col(2);
}

// The median of `values`, which it reorders. Zero for none.
double Median(std::vector<double>& values) {
  if (values.empty()) return 0.0;
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  return values[middle];
}

// The rectangle's own coordinates of a 2D point: along its axis from its
// centre, and across it.
Eigen::Vector2d InRectangle(const OrientedRectangle& r,
                            const Eigen::Vector2d& q) {
  const Eigen::Vector2d d = q - r.centre;
  return {d.dot(r.axis), d.dot(r.Across())};
}

// A straight line t = offset + slope s through band points, by least
// squares, then again without the points more than `reject` robust standard
// deviations off it (the median absolute deviation times 1.4826 is the
// standard deviation of normally scattered values, but a few far strays do
// not move it). Twice, so the second pass starts from a line the strays did
// not pull.
struct BandLine {
  double offset = 0.0;
  double slope = 0.0;
  double rms = 0.0;
  std::vector<Eigen::Vector2d> used;  // (s, t) of the points on it
};

std::optional<BandLine> FitBandLine(const std::vector<Eigen::Vector2d>& st,
                                    double reject) {
  std::vector<Eigen::Vector2d> use = st;
  BandLine line;
  for (int pass = 0; pass < 3; ++pass) {
    if (use.size() < 10) return std::nullopt;
    double ms = 0.0, mt = 0.0;
    for (const auto& p : use) {
      ms += p.x();
      mt += p.y();
    }
    ms /= use.size();
    mt /= use.size();
    double sst = 0.0, sss = 0.0;
    for (const auto& p : use) {
      sss += (p.x() - ms) * (p.x() - ms);
      sst += (p.x() - ms) * (p.y() - mt);
    }
    if (!(sss > 0.0)) return std::nullopt;
    line.slope = sst / sss;
    line.offset = mt - line.slope * ms;
    if (pass == 2) break;

    std::vector<double> deviations;
    deviations.reserve(st.size());
    for (const auto& p : st) {
      deviations.push_back(
          std::abs(p.y() - line.offset - line.slope * p.x()));
    }
    const double limit = reject * 1.4826 * Median(deviations);
    use.clear();
    for (const auto& p : st) {
      if (std::abs(p.y() - line.offset - line.slope * p.x()) <= limit) {
        use.push_back(p);
      }
    }
  }
  double sum = 0.0;
  for (const auto& p : use) {
    const double r = p.y() - line.offset - line.slope * p.x();
    sum += r * r;
  }
  line.rms = std::sqrt(sum / use.size());
  line.used = std::move(use);
  return line;
}

// A side of FitRectangle's answer, as a standard error of its position: its
// edge dots' scatter over the root of their number, and what is left of the
// pixel grid, `grid` times the dot spacing.
//
// For the module's ends (steps in depth) FitRectangle's dot counting takes
// out nearly all of the grid: over 100 noiseless pictures of this cell the
// centre along the long side was off by 0.09 mm RMS, 0.24 mm at worst, with
// dots 6 mm apart, which is 0.02 spacings for each end. kEndGrid says that,
// with a little margin. A long side measured by FitRectangle alone (no
// crease) is taken as off by anything within one spacing, uniformly: a
// spacing over sqrt(12). That is cautious for the grid, and says nothing
// of the crease points' bias, which is larger (see the header).
constexpr double kEndGrid = 0.025;
constexpr double kLongSideGrid = 0.2887;  // 1 / sqrt(12)

double SideSigma(const SideFit& side, double spacing, double grid) {
  const double n = std::max(side.dots_used, 1);
  return std::hypot(side.rms_residual / std::sqrt(n), grid * spacing);
}

}  // namespace

// ------------------------------------------------------------------ prior

Eigen::Isometry3d PalletPrior::FrontGlass() const {
  return front_module *
         Eigen::Translation3d(0.0, 0.0, 0.5 * module_thickness);
}

Eigen::Vector3d PalletPrior::GlassNormal() const {
  return front_module.linear().col(2);
}

double PalletPrior::DesignLean() const {
  // Leaning back tips the normal, which faces the arm, upward by the lean.
  return std::asin(std::clamp(GlassNormal().dot(kUp), -1.0, 1.0));
}

PalletPrior PriorFromDesign(const PalletBodies& bodies,
                            double placement_tolerance,
                            double placement_yaw_tolerance) {
  PalletPrior prior;
  prior.module_count = static_cast<int>(bodies.nominal_modules.size());
  if (prior.module_count == 0) return prior;
  const WorldPose& first = bodies.nominal_modules.front();
  prior.front_module.translation() = first.position;
  prior.front_module.linear() = first.orientation.toRotationMatrix();
  if (prior.module_count > 1) {
    // The design spaces the modules evenly, one behind the other along the
    // glass's normal.
    const WorldPose& last = bodies.nominal_modules.back();
    prior.pitch = std::abs((first.position - last.position)
                               .dot(prior.GlassNormal())) /
                  (prior.module_count - 1);
  }
  prior.pallet_origin = bodies.nominal_pallet.position;
  // The pallet's origin is on the floor.
  prior.floor_height = bodies.nominal_pallet.position.z();
  prior.placement_tolerance = placement_tolerance;
  prior.placement_yaw_tolerance = placement_yaw_tolerance;
  return prior;
}

// ---------------------------------------------------------------- points

PixelPoints ImageToArmFrame(const DepthImage& image,
                            const CameraModel& camera) {
  return PointsWithPixels(image, camera, PointFrame::kWorld);
}

// ------------------------------------------------------------------ crop

namespace {

bool InBox(const Eigen::Isometry3d& to_box, const StackCrop& crop,
           const Eigen::Vector3d& world) {
  if (!(world.z() >= crop.min_height)) return false;
  const Eigen::Vector3d local = to_box * world;
  return (local.array() >= crop.low.array()).all() &&
         (local.array() <= crop.high.array()).all();
}

}  // namespace

bool StackCrop::Contains(const Eigen::Vector3d& world) const {
  return InBox(frame.inverse(), *this, world);
}

StackCrop CropFromPrior(const PalletPrior& prior, double extra,
                        double floor_clearance) {
  const Eigen::Isometry3d glass = prior.FrontGlass();
  const double half_length = 0.5 * prior.module_length;
  const double half_width = 0.5 * prior.module_width;
  const double depth = (prior.module_count - 1) * prior.pitch;

  // The farthest a glass corner, first module's or last's, is from the
  // vertical line the pallet turns about. Turning by the tolerance moves it
  // by about that times the angle.
  double reach = 0.0;
  for (double z : {0.0, -depth}) {
    for (double x : {-half_length, half_length}) {
      for (double y : {-half_width, half_width}) {
        const Eigen::Vector3d corner = glass * Eigen::Vector3d(x, y, z);
        const Eigen::Vector2d off = (corner - prior.pallet_origin).head<2>();
        reach = std::max(reach, off.norm());
      }
    }
  }
  const double margin = std::sqrt(2.0) * prior.placement_tolerance +
                        prior.placement_yaw_tolerance * reach + extra;

  StackCrop crop;
  crop.frame = glass;
  crop.low = Eigen::Vector3d(-half_length - margin, -half_width - margin,
                             -depth - margin);
  crop.high =
      Eigen::Vector3d(half_length + margin, half_width + margin, margin);
  crop.min_height = prior.floor_height + floor_clearance;
  return crop;
}

std::vector<int> CropIndices(const Points& points, const StackCrop& crop) {
  const Eigen::Isometry3d to_box = crop.frame.inverse();
  std::vector<int> inside;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (InBox(to_box, crop, points[i])) inside.push_back(i);
  }
  return inside;
}

// ----------------------------------------------------------------- plane

namespace {

// A point at depth z seen through pixel (u, v) is c + z R d, with c the
// camera's centre, R its orientation and d = PointAtUnitDepth(u, v), whose
// depth is 1. A depth error e moves it by e R d, which takes it off a plane
// with unit normal n by e (n . R d). And R d = (p - c) / z. So the off-plane
// standard deviation is Sigma(z) |n . (p - c)| / z: a scale, Sigma(z), and
// a direction, (p - c) / z, worked out once per point here and used for
// every plane tried.
struct PointNoise {
  std::vector<Eigen::Vector3d> directions;  // (p - c) / z
  std::vector<double> scales;               // Sigma(z)

  double Sigma(int i, const Eigen::Vector3d& normal) const {
    return scales[i] * std::abs(directions[i].dot(normal));
  }
};

PointNoise NoiseOf(const Points& points, const CameraModel& camera,
                   const NoiseSpec& noise) {
  const Eigen::Vector3d axis = ViewingAxis(camera);
  const Eigen::Vector3d centre = camera.pose.translation();
  PointNoise result;
  result.directions.reserve(points.size());
  result.scales.reserve(points.size());
  for (const Eigen::Vector3d& p : points) {
    const Eigen::Vector3d offset = p - centre;
    // Every point a camera reads is in front of it; one that is not gets
    // the noise of its distance, in every direction.
    double depth = axis.dot(offset);
    Eigen::Vector3d direction = offset / depth;
    if (!(depth > 0.0)) {
      depth = offset.norm();
      direction = Eigen::Vector3d::Ones() / std::sqrt(3.0);
    }
    result.directions.push_back(direction);
    result.scales.push_back(noise.Sigma(depth));
  }
  return result;
}

// FitWeightedPlane with each point's noise worked out already.
std::optional<WeightedPlane> FitWeighted(const Points& points,
                                         const std::vector<int>& indices,
                                         const PointNoise& noise,
                                         const Eigen::Vector3d& normal_hint) {
  if (indices.size() < 4) return std::nullopt;
  const Eigen::Vector3d hint = normal_hint.normalized();

  std::vector<double> weights(indices.size());
  double total = 0.0;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (std::size_t k = 0; k < indices.size(); ++k) {
    const double sigma = noise.Sigma(indices[k], hint);
    weights[k] = 1.0 / (sigma * sigma);
    total += weights[k];
    centroid += weights[k] * points[indices[k]];
  }
  centroid /= total;

  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (std::size_t k = 0; k < indices.size(); ++k) {
    const Eigen::Vector3d d = points[indices[k]] - centroid;
    scatter.noalias() += weights[k] * (d * d.transpose());
  }
  if (!scatter.allFinite()) return std::nullopt;
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success) return std::nullopt;
  const Eigen::Vector3d spread = solver.eigenvalues();  // smallest first
  if (!(spread[1] > 1e-12 * spread[2])) return std::nullopt;

  WeightedPlane fit;
  Eigen::Vector3d normal = solver.eigenvectors().col(0);
  if (normal.dot(hint) < 0.0) normal = -normal;
  fit.plane.normal = normal;
  fit.plane.point = centroid;
  fit.centroid = centroid;

  // spread[0] is the weighted sum of squared residuals, in units of each
  // point's own variance: about the number of points if the noise model is
  // right. Scaling by it per degree of freedom makes the errors follow the
  // scatter actually seen.
  const double dof = static_cast<double>(indices.size()) - 3.0;
  const double scale = std::max(spread[0], 0.0) / dof;
  fit.sigma_offset = std::sqrt(scale / total);
  fit.sigma_tilt = std::sqrt(scale / spread[1] + scale / spread[2]);
  return fit;
}

// Root mean square distance of points[indices] from `plane`, in metres and
// in units of each point's own noise.
std::pair<double, double> Residuals(const Points& points,
                                    const std::vector<int>& indices,
                                    const PointNoise& noise,
                                    const GlassPlane& plane) {
  if (indices.empty()) return {0.0, 0.0};
  double sum = 0.0, normalised = 0.0;
  for (int i : indices) {
    const double d = plane.SignedDistance(points[i]);
    const double sigma = noise.Sigma(i, plane.normal);
    sum += d * d;
    normalised += d * d / (sigma * sigma);
  }
  const double n = static_cast<double>(indices.size());
  return {std::sqrt(sum / n), std::sqrt(normalised / n)};
}

// FindGlassPlane with each point's noise worked out already.
std::optional<PlaneFitResult> FindPlane(const Points& points,
                                        const PointNoise& noise,
                                        const PalletPrior& prior,
                                        const PlaneOptions& options,
                                        std::string* detail);

}  // namespace

double OffPlaneSigma(const NoiseSpec& noise, const CameraModel& camera,
                     const Eigen::Vector3d& point,
                     const Eigen::Vector3d& normal) {
  return NoiseOf({point}, camera, noise).Sigma(0, normal);
}

std::optional<WeightedPlane> FitWeightedPlane(
    const Points& points, const std::vector<int>& indices,
    const CameraModel& camera, const NoiseSpec& noise,
    const Eigen::Vector3d& normal_hint) {
  return FitWeighted(points, indices, NoiseOf(points, camera, noise),
                     normal_hint);
}

std::optional<PlaneFitResult> FindGlassPlane(const Points& points,
                                             const CameraModel& camera,
                                             const PalletPrior& prior,
                                             const PlaneOptions& options,
                                             std::string* detail) {
  return FindPlane(points, NoiseOf(points, camera, options.noise), prior,
                   options, detail);
}

namespace {

std::optional<PlaneFitResult> FindPlane(const Points& points,
                                        const PointNoise& noise,
                                        const PalletPrior& prior,
                                        const PlaneOptions& options,
                                        std::string* detail) {
  const Eigen::Vector3d expected = prior.GlassNormal();
  if (static_cast<int>(points.size()) < options.min_inliers) {
    if (detail) {
      *detail = Format("%d points where the stack can be, fewer than %d",
                       static_cast<int>(points.size()), options.min_inliers);
    }
    return std::nullopt;
  }

  // RANSAC takes one inlier distance for all points: the one the noisiest
  // point given needs, for a plane facing the expected way.
  double widest = 0.0;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    widest = std::max(widest, noise.Sigma(i, expected));
  }
  RansacOptions ransac;
  ransac.inlier_distance = options.inlier_sigmas * widest;
  ransac.max_iterations = options.ransac_iterations;
  // Every stride-th point, at most options.ransac_points of them.
  const int count = static_cast<int>(points.size());
  const int stride =
      options.ransac_points > 0
          ? std::max(1, (count + options.ransac_points - 1) /
                            options.ransac_points)
          : 1;
  Points sample;
  sample.reserve(points.size() / stride + 1);
  for (int i = 0; i < count; i += stride) sample.push_back(points[i]);
  ransac.min_inliers = std::max(3, options.min_inliers / stride);
  ransac.seed = options.seed;
  ransac.expected_normal = expected;
  ransac.max_normal_angle = options.max_normal_angle;
  const std::optional<RansacResult> found = FitPlaneRansac(sample, ransac);
  if (!found) {
    if (detail) {
      *detail = Format(
          "no plane within %.1f deg of the glass's direction has %d points "
          "within %.1f mm",
          options.max_normal_angle * 180.0 / std::numbers::pi,
          options.min_inliers, ransac.inlier_distance * 1e3);
    }
    return std::nullopt;
  }

  PlaneFitResult result;
  result.ransac_inliers = static_cast<int>(found->inliers.size()) * stride;
  result.ransac_distance = ransac.inlier_distance;
  GlassPlane plane;
  plane.normal = found->plane.normal.dot(expected) >= 0.0
                     ? found->plane.normal
                     : Eigen::Vector3d(-found->plane.normal);
  plane.point = found->plane.distance * found->plane.normal;

  // Each point's own limit, then a weighted refit, until the set settles.
  std::vector<int> inliers;
  for (int round = 0; round < 6; ++round) {
    std::vector<int> next;
    double low = 1e9, high = 0.0;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
      const double limit = options.inlier_sigmas * noise.Sigma(i, plane.normal);
      if (std::abs(plane.SignedDistance(points[i])) <= limit) {
        next.push_back(i);
        low = std::min(low, limit);
        high = std::max(high, limit);
      }
    }
    result.min_inlier_distance = low;
    result.max_inlier_distance = high;
    const bool settled = next == inliers;
    inliers = std::move(next);
    if (settled) break;
    const std::optional<WeightedPlane> refit =
        FitWeighted(points, inliers, noise, plane.normal);
    if (!refit) break;
    plane = refit->plane;
  }
  if (static_cast<int>(inliers.size()) < options.min_inliers) {
    if (detail) {
      *detail = Format("the plane kept %d points, fewer than %d",
                       static_cast<int>(inliers.size()), options.min_inliers);
    }
    return std::nullopt;
  }

  result.plane = plane;
  std::tie(result.rms_error, result.normalised_rms) =
      Residuals(points, inliers, noise, plane);
  result.inliers = std::move(inliers);
  return result;
}

}  // namespace

// --------------------------------------------------------------- flatten

Eigen::Vector2d PlaneFrame::Flatten(const Eigen::Vector3d& p) const {
  const Eigen::Vector3d d = p - origin;
  return {d.dot(x), d.dot(y)};
}

Eigen::Vector3d PlaneFrame::Lift(const Eigen::Vector2d& q) const {
  return origin + q.x() * x + q.y() * y;
}

PlaneFrame FlatteningFrame(const GlassPlane& plane, const Eigen::Vector3d& near,
                           const Eigen::Vector3d& up) {
  PlaneFrame frame;
  frame.normal = plane.normal.normalized();
  frame.x = up.cross(frame.normal).normalized();
  frame.y = frame.normal.cross(frame.x);
  frame.origin = near - frame.normal * frame.normal.dot(near - plane.point);
  return frame;
}

Points2d Flatten(const Points& points, const std::vector<int>& indices,
                 const PlaneFrame& frame) {
  Points2d flat;
  flat.reserve(indices.size());
  for (int i : indices) flat.push_back(frame.Flatten(points[i]));
  return flat;
}

// --------------------------------------------------------------- outline

std::optional<ModuleOutline> FitModuleOutline(
    const Points& points, const PlaneFitResult& glass, const PlaneFrame& frame,
    const CameraModel& camera, const PalletPrior& prior,
    const PlaneOptions& plane_options, const OutlineOptions& options,
    OutlineRefusal* refusal) {
  const Points2d dots = Flatten(points, glass.inliers, frame);
  RectangleRefusal why;
  const std::optional<RectangleFit> fit =
      FitRectangle(dots, prior.module_length, prior.module_width,
                   options.rectangle, &why);
  if (!fit) {
    if (refusal) {
      refusal->rectangle = why;
      refusal->detail = why.detail;
    }
    return std::nullopt;
  }

  ModuleOutline outline;
  outline.fit = *fit;
  outline.rectangle = fit->rectangle;
  const OrientedRectangle& first = fit->rectangle;

  // Spacings of the dots along the rectangle's axis (the smaller one, as
  // DotSpacing gives it, since the camera's rows run along the long side)
  // and across it, from the area each dot stands for.
  const double spacing_along = fit->spacing;
  const double spacing_across =
      fit->dots_kept > 0 && spacing_along > 0.0
          ? first.length * first.width / fit->dots_kept / spacing_along
          : spacing_along;

  if (options.use_creases) {
    // The band points of each long edge, as (s, t) in the first rectangle's
    // frame.
    std::vector<Eigen::Vector2d> top_band, bottom_band;
    const double half_s =
        0.5 * first.length * (1.0 - 2.0 * options.crease_end_margin);
    const double half_t = 0.5 * first.width;
    for (const Eigen::Vector3d& p : points) {
      const Eigen::Vector2d st = InRectangle(first, frame.Flatten(p));
      if (std::abs(st.x()) > half_s) continue;
      const bool near_top = std::abs(st.y() - half_t) <= options.crease_window;
      const bool near_bottom =
          std::abs(st.y() + half_t) <= options.crease_window;
      if (!near_top && !near_bottom) continue;
      const double distance = glass.plane.SignedDistance(p);
      if (std::abs(distance) > options.crease_depth) continue;
      const double limit =
          plane_options.inlier_sigmas *
          OffPlaneSigma(plane_options.noise, camera, p, glass.plane.normal);
      // The module's own top face is behind the glass; the deck in front.
      if (near_top && distance < -limit) top_band.push_back(st);
      if (near_bottom && distance > limit) bottom_band.push_back(st);
    }

    std::optional<BandLine> top, bottom;
    if (static_cast<int>(top_band.size()) >= options.min_crease_points) {
      top = FitBandLine(top_band, options.crease_reject_sigmas);
    }
    if (static_cast<int>(bottom_band.size()) >= options.min_crease_points) {
      bottom = FitBandLine(bottom_band, options.crease_reject_sigmas);
    }
    if (top && static_cast<int>(top->used.size()) < options.min_crease_points) {
      top.reset();
    }
    if (bottom &&
        static_cast<int>(bottom->used.size()) < options.min_crease_points) {
      bottom.reset();
    }

    double slope = 0.0;
    double sigma_slope = 0.0;
    if (top && bottom) {
      // Both edges are the same direction: one slope through both bands,
      // each band about its own mean.
      double sss = 0.0, sst = 0.0, sum = 0.0;
      int n = 0;
      for (const BandLine* line : {&*top, &*bottom}) {
        double ms = 0.0, mt = 0.0;
        for (const auto& p : line->used) {
          ms += p.x();
          mt += p.y();
        }
        ms /= line->used.size();
        mt /= line->used.size();
        for (const auto& p : line->used) {
          sss += (p.x() - ms) * (p.x() - ms);
          sst += (p.x() - ms) * (p.y() - mt);
        }
      }
      slope = sst / sss;
      for (BandLine* line : {&*top, &*bottom}) {
        double ms = 0.0, mt = 0.0;
        for (const auto& p : line->used) {
          ms += p.x();
          mt += p.y();
        }
        ms /= line->used.size();
        mt /= line->used.size();
        line->slope = slope;
        line->offset = mt - slope * ms;
        double squares = 0.0;
        for (const auto& p : line->used) {
          const double r = p.y() - line->offset - slope * p.x();
          squares += r * r;
        }
        line->rms = std::sqrt(squares / line->used.size());
        sum += squares;
        n += static_cast<int>(line->used.size());
      }
      sigma_slope = std::sqrt(sum / n / sss);
    }

    auto record = [](const std::optional<BandLine>& line, double fit_edge,
                     double outward, CreaseEdge* edge) {
      if (!line) return;
      edge->found = true;
      edge->points = static_cast<int>(line->used.size());
      edge->offset = line->offset;
      edge->slope = line->slope;
      edge->rms = line->rms;
      edge->shift = outward * (line->offset - fit_edge);
    };
    record(top, half_t, 1.0, &outline.top);
    record(bottom, -half_t, -1.0, &outline.bottom);

    // The long edges' offsets across the first rectangle, at its centre.
    const double top_edge = top ? top->offset : half_t;
    const double bottom_edge = bottom ? bottom->offset : -half_t;
    const double angle = std::atan(slope);
    OrientedRectangle r = first;
    r.axis = (first.axis + slope * first.Across()).normalized();
    r.centre = first.centre + first.Across() * (0.5 * (top_edge + bottom_edge));
    r.width = (top_edge - bottom_edge) * std::cos(angle);
    outline.rectangle = Canonical(r);

    const double sigma_top =
        top ? top->rms / std::sqrt(static_cast<double>(top->used.size()))
            : SideSigma(fit->sides[2], spacing_across, kLongSideGrid);
    const double sigma_bottom =
        bottom
            ? bottom->rms / std::sqrt(static_cast<double>(bottom->used.size()))
            : SideSigma(fit->sides[0], spacing_across, kLongSideGrid);
    outline.sigma_across =
        0.5 * std::sqrt(sigma_top * sigma_top + sigma_bottom * sigma_bottom);
    if (top && bottom) outline.sigma_angle = sigma_slope;
  }

  if (!(options.use_creases && outline.top.found && outline.bottom.found)) {
    // The rectangle fit's own long sides: each one's line through its edge
    // dots, whose direction is known to its scatter over the spread of the
    // dots along it, length / sqrt(12) for evenly spread dots.
    double sum = 0.0;
    for (int k : {0, 2}) {
      const SideFit& side = fit->sides[k];
      const double n = std::max(side.dots_used, 1);
      const double s = side.rms_residual * std::sqrt(12.0) /
                       (std::sqrt(n) * first.length);
      sum += s * s;
    }
    outline.sigma_angle = 0.5 * std::sqrt(sum);
  }
  if (!options.use_creases) {
    outline.sigma_across =
        0.5 *
        std::hypot(SideSigma(fit->sides[0], spacing_across, kLongSideGrid),
                   SideSigma(fit->sides[2], spacing_across, kLongSideGrid));
  }
  outline.sigma_along =
      0.5 * std::hypot(SideSigma(fit->sides[1], spacing_along, kEndGrid),
                       SideSigma(fit->sides[3], spacing_along, kEndGrid));

  const double tolerance = options.rectangle.size_tolerance;
  if (std::abs(outline.rectangle.width - prior.module_width) > tolerance) {
    if (refusal) {
      refusal->width_mismatch = true;
      refusal->detail = Format(
          "the creases put the long edges %.1f mm apart, not %.1f mm within "
          "%.1f mm",
          outline.rectangle.width * 1e3, prior.module_width * 1e3,
          tolerance * 1e3);
    }
    return std::nullopt;
  }
  return outline;
}

// ------------------------------------------------------------ clear view

ViewCheck CheckView(const DepthImage& image, const CameraModel& camera,
                    const GlassPlane& glass, const PlaneFrame& frame,
                    const OrientedRectangle& outline, const NoiseSpec& noise,
                    const ViewOptions& options) {
  ViewCheck check;
  const double half_s = 0.5 * outline.length - options.margin;
  const double half_t = 0.5 * outline.width - options.margin;
  if (!(half_s > 0.0 && half_t > 0.0)) return check;

  // Only the pixels round the outline's picture need looking at.
  double u_low = image.width, u_high = 0.0, v_low = image.height, v_high = 0.0;
  for (const Eigen::Vector2d& corner : outline.Corners()) {
    const std::optional<Eigen::Vector2d> pixel =
        Project(camera, frame.Lift(corner));
    if (!pixel) return check;
    u_low = std::min(u_low, pixel->x());
    u_high = std::max(u_high, pixel->x());
    v_low = std::min(v_low, pixel->y());
    v_high = std::max(v_high, pixel->y());
  }
  const int u0 = std::max(0, static_cast<int>(std::floor(u_low)));
  const int u1 = std::min(image.width - 1, static_cast<int>(std::ceil(u_high)));
  const int v0 = std::max(0, static_cast<int>(std::floor(v_low)));
  const int v1 =
      std::min(image.height - 1, static_cast<int>(std::ceil(v_high)));

  // Each pixel's ray, c + z R d with d = PointAtUnitDepth(u, v) at depth z,
  // meets the plane at the depth z_glass where n . (c + z R d - p0) = 0.
  // A reading z is then (z - z_glass)(n . R d) off the plane, and its noise
  // Sigma(z) |n . R d| (see PointNoise), so the test is simply whether the
  // reading is within `sigmas` of its noise of the glass's depth.
  const Eigen::Matrix3d rotation = camera.pose.linear();
  const Eigen::Vector3d centre = camera.pose.translation();
  const Eigen::Vector3d normal_in_camera = rotation.transpose() * glass.normal;
  const double reach = glass.normal.dot(glass.point - centre);
  for (int v = v0; v <= v1; ++v) {
    for (int u = u0; u <= u1; ++u) {
      const Eigen::Vector3d d = PointAtUnitDepth(camera, u + 0.5, v + 0.5);
      const double facing = normal_in_camera.dot(d);
      if (std::abs(facing) < 1e-9) continue;
      const double glass_depth = reach / facing;
      if (!(glass_depth > 0.0)) continue;
      const Eigen::Vector2d st = InRectangle(
          outline, frame.Flatten(centre + glass_depth * (rotation * d)));
      if (std::abs(st.x()) > half_s || std::abs(st.y()) > half_t) continue;
      ++check.pixels;
      if (!image.has_reading(u, v)) {
        ++check.missing;
        continue;
      }
      const double depth = image.at(u, v);
      const double difference = depth - glass_depth;
      if (std::abs(difference) <= options.sigmas * noise.Sigma(depth)) {
        ++check.on_glass;
      } else if (difference < 0.0) {
        ++check.in_front;  // nearer the camera, so between it and the glass
      } else {
        ++check.behind;
      }
    }
  }
  return check;
}

// ------------------------------------------------------------------ pose

Eigen::Isometry3d ModulePoseFromOutline(const PlaneFrame& frame,
                                        const OrientedRectangle& outline,
                                        double thickness) {
  const Eigen::Vector3d x =
      (outline.axis.x() * frame.x + outline.axis.y() * frame.y).normalized();
  const Eigen::Vector3d z = frame.normal;
  const Eigen::Vector3d y = z.cross(x);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear().col(0) = x;
  pose.linear().col(1) = y;
  pose.linear().col(2) = z;
  pose.translation() = frame.Lift(outline.centre) - 0.5 * thickness * z;
  return pose;
}

// --------------------------------------------------------- all together

const char* FinderFailureName(FinderFailure failure) {
  switch (failure) {
    case FinderFailure::kNone: return "none";
    case FinderFailure::kBadInput: return "bad input";
    case FinderFailure::kTooFewPoints: return "too few points";
    case FinderFailure::kNoPlane: return "no plane";
    case FinderFailure::kWrongLean: return "wrong lean";
    case FinderFailure::kNoRectangle: return "no rectangle";
    case FinderFailure::kSizeMismatch: return "size mismatch";
    case FinderFailure::kViewBlocked: return "view blocked";
  }
  return "unknown";
}

std::optional<ModuleEstimate> FindFrontModule(const DepthImage& image,
                                              const CameraModel& camera,
                                              const PalletPrior& prior,
                                              const FinderOptions& options,
                                              FinderDiagnostics* diagnostics) {
  const Clock::time_point start = Clock::now();
  FinderDiagnostics diag;
  auto refuse = [&](FinderFailure failure, std::string detail) {
    diag.failure = failure;
    diag.detail = std::move(detail);
    diag.seconds_total = SecondsSince(start);
    if (diagnostics) *diagnostics = diag;
    return std::nullopt;
  };

  if (image.width != camera.width || image.height != camera.height ||
      image.size() == 0 ||
      static_cast<int>(image.depth.size()) != image.size() ||
      prior.module_count < 1) {
    return refuse(FinderFailure::kBadInput,
                  "the image is not the camera's size, or there is no prior");
  }

  // 1 and 2. Points in the arm's frame, and only where the stack can be.
  Clock::time_point step = Clock::now();
  const PixelPoints cloud = ImageToArmFrame(image, camera);
  diag.pixels_read = static_cast<int>(cloud.points.size());
  const StackCrop crop =
      CropFromPrior(prior, options.crop_extra, options.floor_clearance);
  const std::vector<int> kept = CropIndices(cloud.points, crop);
  Points points;
  points.reserve(kept.size());
  for (int i : kept) points.push_back(cloud.points[i]);
  diag.points_in_crop = static_cast<int>(points.size());
  diag.seconds_points = SecondsSince(step);
  if (diag.points_in_crop < options.plane.min_inliers) {
    return refuse(FinderFailure::kTooFewPoints,
                  Format("%d points where the stack can be, fewer than %d",
                         diag.points_in_crop, options.plane.min_inliers));
  }

  // 3. The glass's plane.
  step = Clock::now();
  std::string why;
  const PointNoise noise = NoiseOf(points, camera, options.plane.noise);
  const std::optional<PlaneFitResult> glass =
      FindPlane(points, noise, prior, options.plane, &why);
  diag.seconds_plane = SecondsSince(step);
  if (!glass) return refuse(FinderFailure::kNoPlane, why);
  diag.ransac_inliers = glass->ransac_inliers;
  diag.plane_inliers = static_cast<int>(glass->inliers.size());
  diag.ransac_distance = glass->ransac_distance;
  diag.min_inlier_distance = glass->min_inlier_distance;
  diag.max_inlier_distance = glass->max_inlier_distance;
  diag.plane_rms = glass->rms_error;
  diag.plane_normalised_rms = glass->normalised_rms;
  diag.first_lean =
      std::asin(std::clamp(glass->plane.normal.dot(kUp), -1.0, 1.0));
  diag.first_lean_error = diag.first_lean - prior.DesignLean();
  if (std::abs(diag.first_lean_error) > options.max_lean_error) {
    return refuse(FinderFailure::kWrongLean,
                  Format("the plane leans %.2f deg, the design %.2f deg",
                         diag.first_lean * 180.0 / std::numbers::pi,
                         prior.DesignLean() * 180.0 / std::numbers::pi));
  }

  // 4 and 5. Flattened, and the outline.
  step = Clock::now();
  const Eigen::Vector3d near = prior.FrontGlass().translation();
  const PlaneFrame frame = FlatteningFrame(glass->plane, near);
  OutlineRefusal outline_refusal;
  const std::optional<ModuleOutline> outline =
      FitModuleOutline(points, *glass, frame, camera, prior, options.plane,
                       options.outline, &outline_refusal);
  diag.seconds_outline = SecondsSince(step);
  if (!outline) {
    const bool size = outline_refusal.width_mismatch ||
                      outline_refusal.rectangle.reason ==
                          RectangleFitFailure::kSizeMismatch;
    return refuse(size ? FinderFailure::kSizeMismatch
                       : FinderFailure::kNoRectangle,
                  outline_refusal.detail);
  }
  const RectangleFit& fit = outline->fit;
  diag.rectangle_dots = fit.dots_kept;
  diag.fit_length = fit.rectangle.length;
  diag.fit_width = fit.rectangle.width;
  for (int k = 0; k < 4; ++k) diag.side_rms[k] = fit.sides[k].rms_residual;
  diag.spacing = fit.spacing;
  diag.length = outline->rectangle.length;
  diag.width = outline->rectangle.width;
  diag.top = outline->top;
  diag.bottom = outline->bottom;

  // The plane once more, from the glass's inside only: the crease points
  // and edge pixels near the outline are left out.
  step = Clock::now();
  std::vector<int> interior;
  const double half_s =
      0.5 * outline->rectangle.length - options.interior_margin;
  const double half_t =
      0.5 * outline->rectangle.width - options.interior_margin;
  for (int i : glass->inliers) {
    const Eigen::Vector2d st =
        InRectangle(outline->rectangle, frame.Flatten(points[i]));
    if (std::abs(st.x()) <= half_s && std::abs(st.y()) <= half_t) {
      interior.push_back(i);
    }
  }
  diag.interior_points = static_cast<int>(interior.size());
  const std::optional<WeightedPlane> final_plane =
      FitWeighted(points, interior, noise, glass->plane.normal);
  if (!final_plane) {
    return refuse(FinderFailure::kNoPlane,
                  "the glass's inside gave no plane on its own");
  }
  std::tie(diag.plane_rms, diag.plane_normalised_rms) =
      Residuals(points, interior, noise, final_plane->plane);
  diag.lean = std::asin(
      std::clamp(final_plane->plane.normal.dot(kUp), -1.0, 1.0));
  diag.lean_error = diag.lean - prior.DesignLean();

  // The outline was measured in `frame`, on the first plane. Its centre and
  // axis go onto the final plane at right angles: the two planes differ by
  // a few hundredths of a degree, so nothing in the plane moves measurably.
  const Eigen::Vector3d n = final_plane->plane.normal;
  Eigen::Vector3d centre = frame.Lift(outline->rectangle.centre);
  centre -= n * final_plane->plane.SignedDistance(centre);
  Eigen::Vector3d axis = outline->rectangle.axis.x() * frame.x +
                         outline->rectangle.axis.y() * frame.y;
  axis = (axis - n * n.dot(axis)).normalized();
  PlaneFrame final_frame;
  final_frame.normal = n;
  final_frame.x = axis;
  final_frame.y = n.cross(axis);
  final_frame.origin = centre;
  OrientedRectangle on_final = outline->rectangle;
  on_final.centre = Eigen::Vector2d::Zero();
  on_final.axis = Eigen::Vector2d::UnitX();

  // 6. Nothing in front of the glass, and no holes in it.
  diag.view = CheckView(image, camera, final_plane->plane, final_frame,
                        on_final, options.plane.noise, options.view);
  diag.seconds_view = SecondsSince(step);
  if (diag.view.BlockedFraction() > options.view.max_blocked_fraction) {
    return refuse(
        FinderFailure::kViewBlocked,
        Format("%.1f%% of %d pixels inside the outline do not read the glass "
               "(%d in front, %d behind, %d no reading); at most %.1f%%",
               100.0 * diag.view.BlockedFraction(), diag.view.pixels,
               diag.view.in_front, diag.view.behind, diag.view.missing,
               100.0 * options.view.max_blocked_fraction));
  }

  // 7. The pose.
  ModuleEstimate estimate;
  estimate.pose =
      ModulePoseFromOutline(final_frame, on_final, prior.module_thickness);
  estimate.glass_centre = centre;
  estimate.glass_normal = n;
  estimate.length = outline->rectangle.length;
  estimate.width = outline->rectangle.width;
  estimate.sigma.along_normal = final_plane->sigma_offset;
  estimate.sigma.face_direction = final_plane->sigma_tilt;
  estimate.sigma.along_length = outline->sigma_along;
  estimate.sigma.up_lean = outline->sigma_across;
  estimate.sigma.in_plane_rotation = outline->sigma_angle;
  diag.seconds_total = SecondsSince(start);
  estimate.diagnostics = diag;
  if (diagnostics) *diagnostics = diag;
  return estimate;
}

}  // namespace cell
