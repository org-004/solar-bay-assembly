// Checks the made-up point clouds: points land where they are said to, the
// noise has the spread asked for, and a pallet view has the stated shape.
// The plane fit's tests rest on these.
//
//   bazel test //:point_generator_test

#include <cmath>
#include <numbers>
#include <random>

#include <Eigen/Core>
#include <Eigen/LU>
#include <gtest/gtest.h>

#include "src/plane_fit.h"
#include "src/point_generator.h"

namespace {

using cell::PalletView;
using cell::PalletViewOptions;
using cell::Points;
using cell::Rectangle;
using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double kDegree = std::numbers::pi / 180.0;

// A rectangle that is not lined up with anything.
Rectangle TiltedRectangle(std::mt19937& rng) {
  Rectangle rectangle;
  rectangle.centre = Vector3d(1.0, -2.0, 0.5);
  rectangle.axes = cell::RandomRotation(rng);
  rectangle.length = 2.3;
  rectangle.width = 1.1;
  return rectangle;
}

TEST(SampleRectangleTest, CleanPointsLieOnTheRectangleAndCoverIt) {
  std::mt19937 rng(1);
  const Rectangle rectangle = TiltedRectangle(rng);
  const Points points = cell::SampleRectangle(rectangle, 5000, 0.0, rng);
  ASSERT_EQ(points.size(), 5000u);

  double least_along = 0.0, most_along = 0.0;
  for (const Vector3d& point : points) {
    // The point in the rectangle's own axes: along the length, along the
    // width, off the surface.
    const Vector3d local =
        rectangle.axes.transpose() * (point - rectangle.centre);
    EXPECT_LE(std::abs(local.x()), rectangle.length / 2.0);
    EXPECT_LE(std::abs(local.y()), rectangle.width / 2.0);
    EXPECT_NEAR(local.z(), 0.0, 1e-12);
    least_along = std::min(least_along, local.x());
    most_along = std::max(most_along, local.x());
  }
  // The points reach nearly to both ends.
  EXPECT_LT(least_along, -0.49 * rectangle.length);
  EXPECT_GT(most_along, 0.49 * rectangle.length);
}

TEST(SampleRectangleTest, NoiseIsAlongTheNormalWithTheSpreadAskedFor) {
  std::mt19937 rng(2);
  const Rectangle rectangle = TiltedRectangle(rng);
  const double noise = 0.002;
  const int count = 20000;
  const Points points = cell::SampleRectangle(rectangle, count, noise, rng);

  double sum = 0.0, sum_of_squares = 0.0;
  for (const Vector3d& point : points) {
    const double off = rectangle.plane().SignedDistance(point);
    sum += off;
    sum_of_squares += off * off;
  }
  const double mean = sum / count;
  const double spread = std::sqrt(sum_of_squares / count - mean * mean);
  // The mean of 20,000 draws is within noise / sqrt(20000) or so of zero,
  // and their spread within a percent or so of the noise.
  EXPECT_NEAR(mean, 0.0, 0.05 * noise);
  EXPECT_NEAR(spread, noise, 0.05 * noise);
}

TEST(SampleBoxTest, PointsStayInTheBoxAndFillIt) {
  std::mt19937 rng(3);
  const Vector3d low(-1.0, 2.0, 0.0), high(3.0, 2.5, 10.0);
  const Points points = cell::SampleBox(low, high, 5000, rng);
  ASSERT_EQ(points.size(), 5000u);

  Vector3d least = high, most = low;
  for (const Vector3d& point : points) {
    EXPECT_TRUE((point.array() >= low.array()).all());
    EXPECT_TRUE((point.array() <= high.array()).all());
    least = least.cwiseMin(point);
    most = most.cwiseMax(point);
  }
  const Vector3d size = high - low;
  EXPECT_TRUE(((least - low).array() < 0.01 * size.array()).all());
  EXPECT_TRUE(((high - most).array() < 0.01 * size.array()).all());
}

TEST(RandomRotationTest, IsARotationAndDiffersFromDrawToDraw) {
  std::mt19937 rng(4);
  Matrix3d previous = Matrix3d::Identity();
  for (int i = 0; i < 100; ++i) {
    const Matrix3d r = cell::RandomRotation(rng);
    // Columns of length 1 at right angles to each other, not mirrored.
    EXPECT_TRUE((r.transpose() * r).isApprox(Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
    EXPECT_FALSE(r.isApprox(previous, 1e-6));
    previous = r;
  }
}

TEST(PalletViewTest, FaceStandsWhereTheHeaderSays) {
  const Vector3d up = Vector3d::UnitZ();
  for (std::uint32_t seed = 1; seed <= 200; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(seed);
    const PalletView view = cell::MakePalletView(PalletViewOptions(), rng);
    const Rectangle& face = view.face;

    EXPECT_TRUE(
        (face.axes.transpose() * face.axes).isApprox(Matrix3d::Identity()));
    EXPECT_NEAR(face.axes.determinant(), 1.0, 1e-12);

    // 1.5 to 2.5 m in front of the camera.
    EXPECT_GE(face.centre.x(), 1.5);
    EXPECT_LE(face.centre.x(), 2.5);
    // Long side level, normal away from the camera.
    EXPECT_NEAR(face.axes.col(0).dot(up), 0.0, 1e-12);
    EXPECT_GT(face.axes.col(2).x(), 0.0);
    // Leaning back 5 to 15 degrees.
    const double lean = cell::Lean(face.plane(), up);
    EXPECT_GE(lean, 5.0 * kDegree - 1e-12);
    EXPECT_LE(lean, 15.0 * kDegree + 1e-12);
    // Turned at most 5 degrees to either side: the long side's direction,
    // which is level, is within 5 degrees of the camera's Y axis.
    const double turn = std::atan2(-face.axes.col(0).x(), face.axes.col(0).y());
    EXPECT_LE(std::abs(turn), 5.0 * kDegree + 1e-12);
  }
}

TEST(PalletViewTest, FacePointsComeFirstAndTheRestAreElsewhere) {
  std::mt19937 rng(5);
  PalletViewOptions options;
  const PalletView view = cell::MakePalletView(options, rng);
  const cell::Plane plane = view.face.plane();

  ASSERT_EQ(static_cast<int>(view.points.size()),
            options.face_points + options.behind_points +
                options.floor_points + options.clutter_points);

  for (int i = 0; i < options.face_points; ++i) {
    EXPECT_LT(std::abs(plane.SignedDistance(view.points[i])),
              6.0 * options.noise);
  }

  // The module behind is one thickness beyond the face, on average.
  double sum = 0.0;
  for (int i = 0; i < options.behind_points; ++i) {
    sum += plane.SignedDistance(view.points[options.face_points + i]);
  }
  EXPECT_NEAR(sum / options.behind_points, options.module_thickness,
              options.noise);

  // The floor is level and below the face's bottom edge.
  const int floor_start = options.face_points + options.behind_points;
  const double face_bottom = view.face.centre.z() - options.module_width / 2.0;
  for (int i = 0; i < options.floor_points; ++i) {
    EXPECT_LT(view.points[floor_start + i].z(), face_bottom);
  }
}

TEST(PalletViewTest, SameSeedGivesTheSameCloud) {
  std::mt19937 first_rng(6), second_rng(6), other_rng(7);
  const PalletView first = cell::MakePalletView(PalletViewOptions(), first_rng);
  const PalletView second =
      cell::MakePalletView(PalletViewOptions(), second_rng);
  const PalletView other = cell::MakePalletView(PalletViewOptions(), other_rng);
  EXPECT_EQ(first.points, second.points);
  EXPECT_NE(first.points, other.points);
}

}  // namespace
