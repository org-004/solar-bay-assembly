// Plane fitting: finding the flat surface that most of a cloud of 3D points
// lies on.
//
// The cloud comes from a depth camera looking at the front module on the
// pallet. Most points are on the module's face. The rest are on the floor,
// the pallet and whatever is behind, and must not pull the answer.
//
// Two fits:
//
//   Least squares  Uses every point given. Exact for clean points and the
//                  best answer under small noise, but one stray point far
//                  from the plane drags it.
//
//   RANSAC         Random sample consensus. Repeatedly draws three points,
//                  takes the plane through them, and counts how many points
//                  lie close to it. The plane with the most support wins
//                  and is then refined by least squares on its supporters
//                  alone. Points off the plane get no vote.
//
// Neither assumes which way is up: a vertical plane is as good as a flat one.
//
// Units are the caller's; metres in this repository. Angles are radians.

#ifndef SRC_PLANE_FIT_H_
#define SRC_PLANE_FIT_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

namespace cell {

using Points = std::vector<Eigen::Vector3d>;

// A plane: the points p for which normal . p = distance.
//
// `normal` has length 1 and points from the origin toward the plane, so
// `distance`, the shortest distance from the origin to the plane, is never
// negative. With a camera at the origin, the normal points away from the
// camera. For a plane through the origin either direction may be reported.
struct Plane {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitX();
  double distance = 0.0;

  // How far `point` is from the plane. Positive beyond the plane as seen
  // from the origin, negative in front of it.
  double SignedDistance(const Eigen::Vector3d& point) const {
    return normal.dot(point) - distance;
  }
};

// The plane through `point` at right angles to `normal`, which need not have
// length 1 or point away from the origin.
Plane PlaneThrough(const Eigen::Vector3d& normal, const Eigen::Vector3d& point);

// How far the plane leans from upright, given which way is up. Zero for a
// vertical plane. Positive when the plane leans back as seen from the
// origin: its top is farther away than its bottom. A quarter turn (pi/2)
// for a floor seen from above. `up` need not have length 1.
double Lean(const Plane& plane, const Eigen::Vector3d& up);

// The angle between two planes' normals, from 0 to pi/2. The direction a
// normal points along its line does not matter.
double AngleBetween(const Plane& a, const Plane& b);

// The least-squares plane through all of `points`: the one with the smallest
// sum of squared distances to them. Returns nothing for fewer than three
// points, points that all lie on one line, or a coordinate that is not
// finite.
std::optional<Plane> FitPlane(const Points& points);

struct RansacOptions {
  // A point this close to a plane supports it. Must be positive. About three
  // times the noise in the points is a usual choice: wide enough to keep
  // nearly all of the plane's own points, narrow enough to leave out a
  // parallel surface behind it.
  double inlier_distance = 0.0;

  // The most three-point samples to try. A sample finds the plane only if
  // all three points are on it. If a fraction w of the points are, one
  // sample succeeds with probability w^3, and n of them all fail with
  // probability (1 - w^3)^n.
  int max_iterations = 1000;

  // Sampling stops early once the chance of having missed a plane as well
  // supported as the best so far is below 1 - confidence. The better the
  // best plane, the sooner: when half the points are on it, 52 samples
  // reach the default. Must be above 0 and below 1.
  double confidence = 0.999;

  // The fit is refused if the winning plane has fewer supporters than this.
  int min_inliers = 3;

  // Seeds the random draws, so the same input gives the same answer.
  std::uint32_t seed = 1;
};

struct RansacResult {
  Plane plane;
  // Indices into the input of every point within inlier_distance of `plane`.
  std::vector<int> inliers;
  // Root mean square distance of the inliers from `plane`.
  double rms_error = 0.0;
};

// The plane with the most points within options.inlier_distance of it, among
// those tried, refined by least squares on those points. Points with a
// coordinate that is not finite are ignored, as a depth image has pixels
// with no reading.
//
// Returns nothing for a bad option, or if no plane with at least
// options.min_inliers supporters was found.
//
// The most supported plane wins, whatever it is. If the floor has more
// points than the face being looked for, the floor is the answer; the caller
// has to hand over a cloud in which the wanted plane is the largest.
std::optional<RansacResult> FitPlaneRansac(const Points& points,
                                           const RansacOptions& options);

}  // namespace cell

#endif  // SRC_PLANE_FIT_H_
