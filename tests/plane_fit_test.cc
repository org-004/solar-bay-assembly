// Checks the plane fits: the least-squares fit is exact on clean points, and
// RANSAC recovers a module face's distance and lean within a stated error
// from clouds with noise, a floor, a module behind and clutter, over many
// seeds and at any orientation.
//
//   bazel test //:plane_fit_test

#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "src/plane_fit.h"
#include "src/point_generator.h"

namespace {

using cell::AngleBetween;
using cell::FitPlane;
using cell::FitPlaneRansac;
using cell::Lean;
using cell::PalletView;
using cell::PalletViewOptions;
using cell::Plane;
using cell::PlaneThrough;
using cell::Points;
using cell::RansacOptions;
using cell::RansacResult;
using cell::Rectangle;
using Eigen::Vector3d;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kMillimetre = 0.001;

// The stated error. From a pallet view with 2 mm of depth noise and 2,000
// points on the face, the fit must recover the face's distance within half
// a millimetre and its direction, and so its lean, within 0.05 degrees.
// Over 20,000 seeds the worst seen was 0.36 mm and 0.038 degrees.
constexpr double kDistanceTolerance = 0.5 * kMillimetre;
constexpr double kAngleTolerance = 0.05 * kDegree;

// Depth noise used throughout, and the inlier distance that goes with it.
constexpr double kNoise = 2.0 * kMillimetre;

RansacOptions Options(std::uint32_t seed = 1) {
  RansacOptions options;
  options.inlier_distance = 3.0 * kNoise;
  options.seed = seed;
  return options;
}

// A module-sized rectangle with the given normal, 2 m from the origin along
// that normal.
Rectangle FaceWithNormal(const Vector3d& normal) {
  const Vector3d n = normal.normalized();
  // Any direction not along the normal, to build the other two axes from.
  const Vector3d other =
      std::abs(n.x()) < 0.9 ? Vector3d::UnitX() : Vector3d::UnitY();
  Rectangle face;
  face.length = 2.3;
  face.width = 1.1;
  face.axes.col(2) = n;
  face.axes.col(0) = n.cross(other).normalized();
  face.axes.col(1) = n.cross(face.axes.col(0));
  face.centre = 2.0 * n;
  return face;
}

// ------------------------------------------------------------------- Plane

TEST(PlaneTest, NormalPointsAwayFromTheOrigin) {
  // Given the normal pointing back at the origin, it is turned round.
  const Plane plane = PlaneThrough(Vector3d(-2.0, 0.0, 0.0), Vector3d(3, 1, 1));
  EXPECT_DOUBLE_EQ(plane.normal.x(), 1.0);
  EXPECT_DOUBLE_EQ(plane.normal.norm(), 1.0);
  EXPECT_DOUBLE_EQ(plane.distance, 3.0);
}

TEST(PlaneTest, SignedDistanceIsPositiveBeyondThePlane) {
  const Plane plane = PlaneThrough(Vector3d::UnitX(), Vector3d(3, 0, 0));
  EXPECT_DOUBLE_EQ(plane.SignedDistance(Vector3d(3.5, 7, -2)), 0.5);
  EXPECT_DOUBLE_EQ(plane.SignedDistance(Vector3d(3.0, 7, -2)), 0.0);
  EXPECT_DOUBLE_EQ(plane.SignedDistance(Vector3d(1.0, 7, -2)), -2.0);
}

TEST(PlaneTest, AngleBetweenIgnoresWhichWayTheNormalsPoint) {
  Plane a;
  a.normal = Vector3d::UnitX();
  Plane b;
  b.normal = Vector3d(std::cos(0.1), std::sin(0.1), 0.0);
  EXPECT_NEAR(AngleBetween(a, b), 0.1, 1e-12);
  b.normal = -b.normal;
  EXPECT_NEAR(AngleBetween(a, b), 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(AngleBetween(a, a), 0.0);
}

// -------------------------------------------------------------------- Lean

TEST(LeanTest, UprightPlaneHasNoLean) {
  const Plane wall = PlaneThrough(Vector3d::UnitX(), Vector3d(2, 0, 0));
  EXPECT_NEAR(Lean(wall, Vector3d::UnitZ()), 0.0, 1e-15);
}

TEST(LeanTest, LeaningBackIsPositive) {
  // Seen from the origin looking along +X with +Z up, a face whose top is
  // farther away than its bottom. Two points on it: the bottom at x = 2 and
  // the top 1 m higher and tan(10 degrees) farther off.
  const double lean = 10.0 * kDegree;
  const Vector3d bottom(2.0, 0.0, 0.0);
  const Vector3d top(2.0 + std::tan(lean), 0.0, 1.0);
  const Vector3d side = Vector3d::UnitY();
  const Plane back = PlaneThrough((top - bottom).cross(side), bottom);
  EXPECT_NEAR(Lean(back, Vector3d::UnitZ()), lean, 1e-12);

  // Leaning toward the origin instead.
  const Vector3d near_top(2.0 - std::tan(lean), 0.0, 1.0);
  const Plane forward = PlaneThrough((near_top - bottom).cross(side), bottom);
  EXPECT_NEAR(Lean(forward, Vector3d::UnitZ()), -lean, 1e-12);
}

TEST(LeanTest, UpCanBeAnyDirectionAndAnyLength) {
  // The same leaning face as above, in a frame where +Y is down and +Z is
  // forward, as a camera's own frame usually is.
  const double lean = 10.0 * kDegree;
  const Vector3d bottom(0.0, 0.0, 2.0);
  const Vector3d top(0.0, -1.0, 2.0 + std::tan(lean));
  const Plane plane = PlaneThrough((top - bottom).cross(Vector3d::UnitX()),
                                   bottom);
  EXPECT_NEAR(Lean(plane, Vector3d(0.0, -5.0, 0.0)), lean, 1e-12);
}

TEST(LeanTest, FloorSeenFromAboveIsAQuarterTurn) {
  const Plane floor = PlaneThrough(Vector3d::UnitZ(), Vector3d(0, 0, -1));
  EXPECT_NEAR(Lean(floor, Vector3d::UnitZ()), std::numbers::pi / 2.0, 1e-12);
}

// ----------------------------------------------------------- Least squares

TEST(FitPlaneTest, CleanPointsGiveTheExactPlaneAtAnyOrientation) {
  std::mt19937 rng(7);
  for (int trial = 0; trial < 100; ++trial) {
    Rectangle face;
    face.axes = cell::RandomRotation(rng);
    face.centre = cell::SampleBox(Vector3d(-3, -3, -3), Vector3d(3, 3, 3), 1,
                                  rng)[0];
    const Points points = cell::SampleRectangle(face, 50, 0.0, rng);

    const std::optional<Plane> fit = FitPlane(points);
    ASSERT_TRUE(fit.has_value());
    EXPECT_NEAR(AngleBetween(*fit, face.plane()), 0.0, 1e-9);
    EXPECT_NEAR(fit->distance, face.plane().distance, 1e-9);
    EXPECT_NEAR(fit->normal.norm(), 1.0, 1e-12);
    EXPECT_GE(fit->distance, 0.0);
  }
}

TEST(FitPlaneTest, NoiseAveragesOut) {
  std::mt19937 rng(8);
  const Rectangle face = FaceWithNormal(Vector3d(1.0, 0.2, -0.1));
  const Points points = cell::SampleRectangle(face, 2000, kNoise, rng);

  const std::optional<Plane> fit = FitPlane(points);
  ASSERT_TRUE(fit.has_value());
  // Far closer than the 2 mm by which any one point is off.
  EXPECT_NEAR(fit->distance, face.plane().distance, kDistanceTolerance);
  EXPECT_LT(AngleBetween(*fit, face.plane()), kAngleTolerance);
}

// The reason for RANSAC: least squares gives every point a say.
TEST(FitPlaneTest, StrayPointsDragTheLeastSquaresFitButNotRansac) {
  std::mt19937 rng(9);
  const Rectangle face = FaceWithNormal(Vector3d::UnitX());
  Points points = cell::SampleRectangle(face, 2000, kNoise, rng);
  // Twenty points, 1% of the cloud, half a metre behind one end of the face.
  for (int i = 0; i < 20; ++i) {
    points.push_back(face.centre + 0.5 * face.axes.col(2) +
                     1.0 * face.axes.col(0));
  }

  const std::optional<Plane> least_squares = FitPlane(points);
  const std::optional<RansacResult> ransac = FitPlaneRansac(points, Options());
  ASSERT_TRUE(least_squares.has_value());
  ASSERT_TRUE(ransac.has_value());

  EXPECT_GT(AngleBetween(*least_squares, face.plane()), 10 * kAngleTolerance);
  EXPECT_LT(AngleBetween(ransac->plane, face.plane()), kAngleTolerance);
  EXPECT_NEAR(ransac->plane.distance, face.plane().distance,
              kDistanceTolerance);
}

TEST(FitPlaneTest, BadInputGivesNothing) {
  const Vector3d a(0, 0, 0), b(1, 0, 0), c(0, 1, 0);
  EXPECT_FALSE(FitPlane({}).has_value());
  EXPECT_FALSE(FitPlane({a, b}).has_value());
  EXPECT_TRUE(FitPlane({a, b, c}).has_value());

  // All on one line.
  EXPECT_FALSE(FitPlane({a, b, 2 * b, 3 * b, -b}).has_value());
  // All the same point.
  EXPECT_FALSE(FitPlane({b, b, b, b}).has_value());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(FitPlane({a, b, c, Vector3d(nan, 0, 0)}).has_value());
  EXPECT_FALSE(FitPlane({a, b, c, Vector3d(0, inf, 0)}).has_value());
}

// ------------------------------------------------------------------ RANSAC

// The stage's main claim, on clouds like those the cell's camera will give.
TEST(RansacTest, PalletViewsAreRecoveredWithinTheStatedError) {
  const Vector3d up = Vector3d::UnitZ();
  for (std::uint32_t seed = 1; seed <= 100; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(seed);
    PalletViewOptions scene;
    scene.noise = kNoise;
    const PalletView view = cell::MakePalletView(scene, rng);
    const Plane truth = view.face.plane();

    const std::optional<RansacResult> fit =
        FitPlaneRansac(view.points, Options(seed));
    ASSERT_TRUE(fit.has_value());
    EXPECT_NEAR(fit->plane.distance, truth.distance, kDistanceTolerance);
    EXPECT_NEAR(Lean(fit->plane, up), Lean(truth, up), kAngleTolerance);
    EXPECT_LT(AngleBetween(fit->plane, truth), kAngleTolerance);
  }
}

// Nothing in the fit may depend on which way the plane faces: a face at any
// orientation, among clutter, is found as well as an upright one.
TEST(RansacTest, AnyOrientationIsRecoveredWithinTheStatedError) {
  for (std::uint32_t seed = 1; seed <= 100; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(seed);
    Rectangle face;
    face.length = 2.3;
    face.width = 1.1;
    face.axes = cell::RandomRotation(rng);
    face.centre = cell::SampleBox(Vector3d(-2, -2, -2), Vector3d(2, 2, 2), 1,
                                  rng)[0];
    Points points = cell::SampleRectangle(face, 2000, kNoise, rng);
    const Points clutter = cell::SampleBox(
        face.centre - Vector3d(2, 2, 2), face.centre + Vector3d(2, 2, 2),
        1500, rng);
    points.insert(points.end(), clutter.begin(), clutter.end());

    const std::optional<RansacResult> fit =
        FitPlaneRansac(points, Options(seed));
    ASSERT_TRUE(fit.has_value());
    // Distance from the face's centre to the fitted plane, not from the
    // origin: a face seen edge-on from the origin turns a small error in
    // direction into a large one in distance from the origin.
    EXPECT_NEAR(fit->plane.SignedDistance(face.centre), 0.0,
                kDistanceTolerance);
    EXPECT_LT(AngleBetween(fit->plane, face.plane()), kAngleTolerance);
  }
}

// Orientations a fit that assumed "up" would get wrong: exactly upright,
// facing each axis, exactly level, and a degree either side of upright.
TEST(RansacTest, UprightLevelAndNearlyUprightFacesAreRecovered) {
  const double tilt = std::tan(1.0 * kDegree);
  const std::vector<Vector3d> normals = {
      Vector3d::UnitX(),      Vector3d::UnitY(),       Vector3d::UnitZ(),
      Vector3d(1, 0, tilt),   Vector3d(1, 0, -tilt),   Vector3d(0, 1, tilt),
      Vector3d(1, 1, 0),      Vector3d(-1, 0, 0),      Vector3d(0, 0, -1),
  };
  std::uint32_t seed = 1;
  for (const Vector3d& normal : normals) {
    SCOPED_TRACE(testing::Message() << "normal " << normal.transpose());
    std::mt19937 rng(seed);
    const Rectangle face = FaceWithNormal(normal);
    Points points = cell::SampleRectangle(face, 2000, kNoise, rng);
    const Points clutter = cell::SampleBox(
        face.centre - Vector3d(2, 2, 2), face.centre + Vector3d(2, 2, 2),
        1000, rng);
    points.insert(points.end(), clutter.begin(), clutter.end());

    const std::optional<RansacResult> fit =
        FitPlaneRansac(points, Options(seed++));
    ASSERT_TRUE(fit.has_value());
    EXPECT_NEAR(fit->plane.distance, 2.0, kDistanceTolerance);
    EXPECT_LT(AngleBetween(fit->plane, face.plane()), kAngleTolerance);
  }
}

TEST(RansacTest, KeepsTheFacePointsAndDropsTheRest) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(seed);
    PalletViewOptions scene;
    scene.noise = kNoise;
    const PalletView view = cell::MakePalletView(scene, rng);

    const std::optional<RansacResult> fit =
        FitPlaneRansac(view.points, Options(seed));
    ASSERT_TRUE(fit.has_value());

    // The face's points come first in the cloud.
    int on_face = 0;
    for (int index : fit->inliers) {
      if (index < scene.face_points) ++on_face;
    }
    const int strays = static_cast<int>(fit->inliers.size()) - on_face;
    // Three standard deviations keep 99.7% of the face's points.
    EXPECT_GE(on_face, 0.99 * scene.face_points);
    // A few floor and clutter points lie within the inlier distance of the
    // face's plane by chance.
    EXPECT_LE(strays, 0.02 * scene.face_points);
  }
}

// A parallel surface one module thickness back, seen nearly as well as the
// face itself. A fit that averaged the two would be off by millimetres.
TEST(RansacTest, DoesNotBlendInTheModuleBehind) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(seed);
    PalletViewOptions scene;
    scene.noise = kNoise;
    scene.behind_points = 1500;
    const PalletView view = cell::MakePalletView(scene, rng);

    const std::optional<RansacResult> fit =
        FitPlaneRansac(view.points, Options(seed));
    ASSERT_TRUE(fit.has_value());
    EXPECT_NEAR(fit->plane.distance, view.face.plane().distance,
                kDistanceTolerance);
  }
}

TEST(RansacTest, InliersAreExactlyThePointsWithinTheDistance) {
  std::mt19937 rng(3);
  const PalletView view = cell::MakePalletView(PalletViewOptions(), rng);
  const RansacOptions options = Options();

  const std::optional<RansacResult> fit =
      FitPlaneRansac(view.points, options);
  ASSERT_TRUE(fit.has_value());

  std::vector<int> expected;
  for (int i = 0; i < static_cast<int>(view.points.size()); ++i) {
    if (std::abs(fit->plane.SignedDistance(view.points[i])) <=
        options.inlier_distance) {
      expected.push_back(i);
    }
  }
  EXPECT_EQ(fit->inliers, expected);
  EXPECT_GT(fit->rms_error, 0.0);
  EXPECT_LT(fit->rms_error, options.inlier_distance);
}

// A depth image has pixels with no reading.
TEST(RansacTest, PointsThatAreNotNumbersAreIgnored) {
  std::mt19937 rng(4);
  const Rectangle face = FaceWithNormal(Vector3d(1.0, 0.1, 0.2));
  const Points clean = cell::SampleRectangle(face, 2000, kNoise, rng);

  // Every third point of the cloud has no reading.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  Points points;
  for (const Vector3d& point : clean) {
    points.push_back(point);
    points.push_back(points.size() % 2 == 0 ? Vector3d(nan, nan, nan)
                                            : Vector3d(0.0, inf, 0.0));
    points.push_back(point);
  }

  const std::optional<RansacResult> fit = FitPlaneRansac(points, Options());
  ASSERT_TRUE(fit.has_value());
  EXPECT_NEAR(fit->plane.distance, 2.0, kDistanceTolerance);
  EXPECT_LT(AngleBetween(fit->plane, face.plane()), kAngleTolerance);
  for (int index : fit->inliers) {
    EXPECT_TRUE(points[index].allFinite());
  }
}

TEST(RansacTest, SameSeedGivesTheSameAnswer) {
  std::mt19937 rng(5);
  const PalletView view = cell::MakePalletView(PalletViewOptions(), rng);

  const std::optional<RansacResult> first =
      FitPlaneRansac(view.points, Options(42));
  const std::optional<RansacResult> second =
      FitPlaneRansac(view.points, Options(42));
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->plane.normal, second->plane.normal);
  EXPECT_EQ(first->plane.distance, second->plane.distance);
  EXPECT_EQ(first->inliers, second->inliers);
}

// A floor with twice the face's points wins an unconstrained fit, as the
// header warns. Told roughly which way the face looks (here 5 degrees off,
// within a limit of 10), the fit never considers the floor and finds the
// face within the stated error.
TEST(RansacTest, ExpectedNormalRejectsABiggerPlaneFacingElsewhere) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(seed);
    PalletViewOptions scene;
    scene.noise = kNoise;
    scene.floor_points = 2 * scene.face_points;
    const PalletView view = cell::MakePalletView(scene, rng);
    const Plane face = view.face.plane();

    const std::optional<RansacResult> free =
        FitPlaneRansac(view.points, Options(seed));
    ASSERT_TRUE(free.has_value());
    EXPECT_GT(AngleBetween(free->plane, face), 45.0 * kDegree);

    // Turned 5 degrees about the face's long side, and pointing back toward
    // the camera: the sign of the expected normal does not matter.
    const Vector3d guess =
        -(Eigen::AngleAxisd(5.0 * kDegree, view.face.axes.col(0)) *
          face.normal);
    RansacOptions options = Options(seed);
    options.expected_normal = guess;
    options.max_normal_angle = 10.0 * kDegree;
    const std::optional<RansacResult> fit =
        FitPlaneRansac(view.points, options);
    ASSERT_TRUE(fit.has_value());
    EXPECT_NEAR(fit->plane.distance, face.distance, kDistanceTolerance);
    EXPECT_LT(AngleBetween(fit->plane, face), kAngleTolerance);
  }
}

// Nothing faces the expected way: nothing is found.
TEST(RansacTest, ExpectedNormalWithNoSuchPlaneGivesNothing) {
  std::mt19937 rng(8);
  const Points wall =
      cell::SampleRectangle(FaceWithNormal(Vector3d::UnitX()), 1000, kNoise,
                            rng);
  RansacOptions options = Options();
  options.min_inliers = 100;
  options.expected_normal = Vector3d::UnitZ();
  options.max_normal_angle = 20.0 * kDegree;
  EXPECT_FALSE(FitPlaneRansac(wall, options).has_value());
  options.expected_normal = Vector3d(-3.0, 0.0, 0.1);  // any length
  EXPECT_TRUE(FitPlaneRansac(wall, options).has_value());
}

TEST(RansacTest, TooLittleSupportGivesNothing) {
  // Clutter only: no plane holds more than a few of these points.
  std::mt19937 rng(6);
  const Points clutter =
      cell::SampleBox(Vector3d(0, -2, -2), Vector3d(4, 2, 2), 2000, rng);

  RansacOptions options = Options();
  options.min_inliers = 500;
  EXPECT_FALSE(FitPlaneRansac(clutter, options).has_value());
}

TEST(RansacTest, BadInputGivesNothing) {
  std::mt19937 rng(7);
  const Points good =
      cell::SampleRectangle(FaceWithNormal(Vector3d::UnitX()), 100, kNoise,
                            rng);
  ASSERT_TRUE(FitPlaneRansac(good, Options()).has_value());

  const double nan = std::numeric_limits<double>::quiet_NaN();
  RansacOptions options = Options();
  options.inlier_distance = 0.0;
  EXPECT_FALSE(FitPlaneRansac(good, options).has_value());
  options.inlier_distance = -1.0;
  EXPECT_FALSE(FitPlaneRansac(good, options).has_value());
  options.inlier_distance = nan;
  EXPECT_FALSE(FitPlaneRansac(good, options).has_value());

  options = Options();
  options.max_iterations = 0;
  EXPECT_FALSE(FitPlaneRansac(good, options).has_value());

  options = Options();
  options.min_inliers = 2;
  EXPECT_FALSE(FitPlaneRansac(good, options).has_value());

  for (double confidence : {0.0, 1.0, -0.5, 1.5, nan}) {
    options = Options();
    options.confidence = confidence;
    EXPECT_FALSE(FitPlaneRansac(good, options).has_value());
  }

  // A direction constraint needs an angle in (0, pi/2] and a finite
  // direction.
  for (double angle : {0.0, -0.1, 2.0, nan}) {
    options = Options();
    options.expected_normal = Vector3d::UnitX();
    options.max_normal_angle = angle;
    EXPECT_FALSE(FitPlaneRansac(good, options).has_value());
  }
  options = Options();
  options.expected_normal = Vector3d(nan, 0.0, 1.0);
  options.max_normal_angle = 0.1;
  EXPECT_FALSE(FitPlaneRansac(good, options).has_value());

  // Too few points, and points that fix no plane.
  const Vector3d a(0, 0, 0), b(1, 0, 0);
  EXPECT_FALSE(FitPlaneRansac({}, Options()).has_value());
  EXPECT_FALSE(FitPlaneRansac({a, b}, Options()).has_value());
  EXPECT_FALSE(
      FitPlaneRansac({a, b, 2 * b, 3 * b, 4 * b}, Options()).has_value());
}

}  // namespace
