#include "src/plane_fit.h"

#include <algorithm>
#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

namespace cell {
namespace {

// The least-squares plane through the points at `indices`.
//
// The plane passes through the points' centroid. Its normal is the direction
// in which the points are spread least: the eigenvector of their scatter
// matrix with the smallest eigenvalue.
std::optional<Plane> FitSubset(const Points& points,
                               const std::vector<int>& indices) {
  if (indices.size() < 3) return std::nullopt;

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (int i : indices) centroid += points[i];
  centroid /= static_cast<double>(indices.size());

  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  for (int i : indices) {
    const Eigen::Vector3d d = points[i] - centroid;
    scatter += d * d.transpose();
  }
  if (!scatter.allFinite()) return std::nullopt;

  // Eigenvalues come out smallest first. Each is the spread of the points
  // along its eigenvector.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  if (solver.info() != Eigen::Success) return std::nullopt;
  const Eigen::Vector3d spread = solver.eigenvalues();

  // Points on one line are spread along it and in no other direction, so
  // every plane containing the line fits equally well.
  if (!(spread[2] > 0.0) || spread[1] <= 1e-12 * spread[2]) {
    return std::nullopt;
  }

  return PlaneThrough(solver.eigenvectors().col(0), centroid);
}

// Indices of the points within `limit` of the plane. A comparison with a
// value that is not a number is false, which leaves such points out.
std::vector<int> InliersOf(const Points& points, const Plane& plane,
                           double limit) {
  std::vector<int> inliers;
  for (int i = 0; i < static_cast<int>(points.size()); ++i) {
    if (std::abs(plane.SignedDistance(points[i])) <= limit) {
      inliers.push_back(i);
    }
  }
  return inliers;
}

int CountInliers(const Points& points, const Plane& plane, double limit) {
  int count = 0;
  for (const Eigen::Vector3d& point : points) {
    if (std::abs(plane.SignedDistance(point)) <= limit) ++count;
  }
  return count;
}

}  // namespace

Plane PlaneThrough(const Eigen::Vector3d& normal,
                   const Eigen::Vector3d& point) {
  Plane plane;
  plane.normal = normal.normalized();
  plane.distance = plane.normal.dot(point);
  if (plane.distance < 0.0) {
    plane.normal = -plane.normal;
    plane.distance = -plane.distance;
  }
  return plane;
}

double Lean(const Plane& plane, const Eigen::Vector3d& up) {
  // The normal of an upright plane is level. Leaning the plane back tips
  // its normal, which points away from the origin, downward by the same
  // angle.
  const double sine = -plane.normal.dot(up.normalized());
  return std::asin(std::clamp(sine, -1.0, 1.0));
}

double AngleBetween(const Plane& a, const Plane& b) {
  // The cross product's length is the sine of the angle and the dot product
  // its cosine. atan2 of the two stays accurate for tiny angles, where acos
  // of the cosine alone does not.
  return std::atan2(a.normal.cross(b.normal).norm(),
                    std::abs(a.normal.dot(b.normal)));
}

std::optional<Plane> FitPlane(const Points& points) {
  std::vector<int> all(points.size());
  for (int i = 0; i < static_cast<int>(all.size()); ++i) all[i] = i;
  return FitSubset(points, all);
}

std::optional<RansacResult> FitPlaneRansac(const Points& points,
                                           const RansacOptions& options) {
  const int n = static_cast<int>(points.size());
  const double limit = options.inlier_distance;
  if (n < 3 || !std::isfinite(limit) || limit <= 0.0 ||
      options.max_iterations < 1 || options.min_inliers < 3 ||
      !(options.confidence > 0.0 && options.confidence < 1.0)) {
    return std::nullopt;
  }

  // The direction constraint, if any, as the smallest |cos| a sample's
  // normal may have with the expected one.
  // (A NaN component compares unequal to zero, so it counts as set and is
  // then refused.)
  const bool constrained = (options.expected_normal.array() != 0.0).any();
  Eigen::Vector3d expected = Eigen::Vector3d::Zero();
  double min_cosine = 0.0;
  if (constrained) {
    if (!options.expected_normal.allFinite() ||
        !(options.max_normal_angle > 0.0 &&
          options.max_normal_angle <= std::acos(0.0))) {
      return std::nullopt;
    }
    expected = options.expected_normal.normalized();
    min_cosine = std::cos(options.max_normal_angle);
  }

  std::mt19937 rng(options.seed);
  std::uniform_int_distribution<int> pick(0, n - 1);

  // Try planes through three random points and keep the best supported.
  Plane best;
  int best_count = 0;
  int iterations = options.max_iterations;
  for (int iteration = 0; iteration < iterations; ++iteration) {
    const int i = pick(rng);
    const int j = pick(rng);
    const int k = pick(rng);
    if (i == j || j == k || i == k) continue;

    // Two edges of the triangle. Their cross product is at right angles to
    // both, so it is the plane's normal, and its length is twice the
    // triangle's area. No area means the three points are in a line and fix
    // no plane. Written so that a value that is not a number also fails.
    const Eigen::Vector3d edge1 = points[j] - points[i];
    const Eigen::Vector3d edge2 = points[k] - points[i];
    const Eigen::Vector3d normal = edge1.cross(edge2);
    if (!(normal.norm() > 1e-12 * edge1.norm() * edge2.norm())) continue;

    // Facing the wrong way: not worth counting. Cheap, so a constrained
    // search spends its time on the planes it could accept.
    if (constrained &&
        std::abs(expected.dot(normal)) < min_cosine * normal.norm()) {
      continue;
    }

    const Plane candidate = PlaneThrough(normal, points[i]);
    const int count = CountInliers(points, candidate, limit);
    if (count > best_count) {
      best_count = count;
      best = candidate;

      // If the true plane holds this fraction of the points, a sample lands
      // on it with probability fraction^3. Enough samples have been tried
      // once the chance that every one missed, (1 - fraction^3)^samples,
      // has fallen to 1 - confidence. Solve for the number of samples.
      const double fraction = static_cast<double>(count) / n;
      const double miss = 1.0 - fraction * fraction * fraction;
      if (miss <= 0.0) break;  // Every point is on this plane.
      const double enough =
          std::ceil(std::log(1.0 - options.confidence) / std::log(miss));
      if (enough < iterations) iterations = static_cast<int>(enough);
    }
  }
  if (best_count < options.min_inliers) return std::nullopt;

  // The winner passes exactly through three noisy points, so it is only
  // roughly right. Fit to all of its supporters instead, which averages
  // their noise. The better plane may gain or lose supporters at the
  // edges, so repeat until the set settles.
  std::vector<int> inliers = InliersOf(points, best, limit);
  const int max_rounds = 5;
  for (int round = 0; round < max_rounds; ++round) {
    const std::optional<Plane> refit = FitSubset(points, inliers);
    if (!refit) break;
    std::vector<int> next = InliersOf(points, *refit, limit);
    if (next.size() < 3) break;
    const bool settled = next == inliers;
    best = *refit;
    inliers = std::move(next);
    if (settled) break;
  }
  if (static_cast<int>(inliers.size()) < options.min_inliers) {
    return std::nullopt;
  }

  RansacResult result;
  result.plane = best;
  double sum_of_squares = 0.0;
  for (int i : inliers) {
    const double d = best.SignedDistance(points[i]);
    sum_of_squares += d * d;
  }
  result.rms_error =
      std::sqrt(sum_of_squares / static_cast<double>(inliers.size()));
  result.inliers = std::move(inliers);
  return result;
}

}  // namespace cell
