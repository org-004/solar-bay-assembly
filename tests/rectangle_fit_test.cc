// Checks the rectangle fit: the convex hull and smallest-area rectangle on
// inputs whose answers can be worked out by hand, and the whole fit on
// made-up dots like those the camera gives of a module's face, at every
// rotation, with ragged edges, missing and flying edge dots, strays and
// bites, and the cases it must refuse.
//
//   bazel test //:rectangle_fit_test
//   bazel test //:rectangle_fit_test --test_output=all   (to see the numbers)

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "src/portable_random.h"
#include "src/rectangle_fit.h"

namespace {

using cell::Canonical;
using cell::ConvexHull;
using cell::DotSpacing;
using cell::FitRectangle;
using cell::LargestCluster;
using cell::MinAreaRectangle;
using cell::OrientedRectangle;
using cell::Points2d;
using cell::RectangleFit;
using cell::RectangleFitFailure;
using cell::RectangleFitOptions;
using cell::RectangleRefusal;
using cell::RefineOptions;
using cell::RefineRectangle;
using cell::UniformReal;
using cell::UniformUnit;
using Eigen::Vector2d;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;
constexpr double kMillimetre = 0.001;

// The module's face.
constexpr double kLength = 2.3;
constexpr double kWidth = 1.1;

// --------------------------------------------------------------- the dots

// How the camera's pixels fall on the face: a grid of dots, rows
// `spacing_a` apart along the row and `spacing_b` between rows, turned by
// `angle` and shifted so that one dot sits at `phase`. Seen 39 to 49 degrees
// off square from about 4 m, the cell's camera gives about 6 by 9 mm.
struct Grid {
  double spacing_a = 6.0 * kMillimetre;
  double spacing_b = 9.0 * kMillimetre;
  double angle = 0.0;
  Vector2d phase = Vector2d::Zero();

  // Within the plane, depth noise moves a dot only a little: up to this
  // much either way in x and in y, evenly.
  double jitter = 0.5 * kMillimetre;

  double Larger() const { return std::max(spacing_a, spacing_b); }
};

// What goes wrong at the edges and around.
struct Damage {
  // In larger spacings: each dot near the edge is kept or dropped as if the
  // edge there were moved in or out by up to this much.
  double ragged = 0.0;
  // Fraction of edge dots (within one larger spacing of the edge) dropped.
  double missing = 0.0;
  // Fraction of edge dots thrown outward by up to one larger spacing:
  // flying pixels that still passed the plane fit.
  double flying = 0.0;
  // Single dots 0.1 to 1.5 m outside the face.
  int strays = 0;
  // Clumps of 7 x 7 dots, 0.2 to 1 m outside: another surface crossing the
  // plane.
  int patches = 0;
};

Damage Typical() {
  Damage damage;
  damage.ragged = 1.0;
  damage.missing = 0.1;
  damage.flying = 0.05;
  damage.strays = 20;
  damage.patches = 2;
  return damage;
}

struct Scene {
  Vector2d centre = Vector2d::Zero();
  double angle = 0.0;  // of the long side, from +x
  double length = kLength;
  double width = kWidth;
  Grid grid;
  Damage damage;

  // Dots for which this is true, given their place along the length and
  // across it from the centre, are hidden by something in front.
  std::function<bool(double, double)> hidden;

  Vector2d Axis() const { return {std::cos(angle), std::sin(angle)}; }
};

Vector2d Perp(const Vector2d& v) { return {-v.y(), v.x()}; }

// The scenes are the same on every platform: the draws come from
// src/portable_random.h, not the library's distributions, and each is taken
// in a statement of its own, never two as arguments of one call, whose order
// the compiler chooses.
Points2d MakeDots(const Scene& scene, std::mt19937& rng) {
  auto unit = [](std::mt19937& r) { return UniformUnit(r); };
  const Vector2d a = scene.Axis();
  const Vector2d b = Perp(a);
  const Grid& g = scene.grid;
  const Vector2d g1(std::cos(g.angle), std::sin(g.angle));
  const Vector2d g2 = Perp(g1);
  const double big = g.Larger();
  const Damage& d = scene.damage;
  const double half_l = 0.5 * scene.length;
  const double half_w = 0.5 * scene.width;

  Points2d dots;
  // Every grid dot within reach of the face.
  const double reach = std::hypot(half_l, half_w) + 2.0 * big;
  const Vector2d from = scene.centre - g.phase;
  const double i0 = from.dot(g1) / g.spacing_a;
  const double j0 = from.dot(g2) / g.spacing_b;
  const int ni = static_cast<int>(std::ceil(reach / g.spacing_a));
  const int nj = static_cast<int>(std::ceil(reach / g.spacing_b));
  // Plain numbers rather than vectors in this loop, which runs over some
  // hundred thousand grid points per call.
  const double ax = a.x(), ay = a.y();
  const double cx = scene.centre.x(), cy = scene.centre.y();
  for (int i = static_cast<int>(i0) - ni; i <= static_cast<int>(i0) + ni;
       ++i) {
    for (int j = static_cast<int>(j0) - nj; j <= static_cast<int>(j0) + nj;
         ++j) {
      double x = g.phase.x() + i * g.spacing_a * g1.x() +
                 j * g.spacing_b * g2.x();
      double y = g.phase.y() + i * g.spacing_a * g1.y() +
                 j * g.spacing_b * g2.y();
      const double s = (x - cx) * ax + (y - cy) * ay;
      const double t = (y - cy) * ax - (x - cx) * ay;
      // How far inside the edge: negative outside.
      const double inside =
          std::min(half_l - std::abs(s), half_w - std::abs(t));
      if (inside < -big) continue;
      double cut = 0.0;
      if (d.ragged > 0.0 && std::abs(inside) <= d.ragged * big) {
        cut = d.ragged * big * (2.0 * unit(rng) - 1.0);
      }
      if (inside <= cut) continue;
      if (scene.hidden && scene.hidden(s, t)) continue;
      if (inside < big) {
        if (d.missing > 0.0 && unit(rng) < d.missing) continue;
        if (d.flying > 0.0 && unit(rng) < d.flying) {
          const Vector2d out = half_l - std::abs(s) < half_w - std::abs(t)
                                   ? (s > 0 ? a : -a)
                                   : (t > 0 ? b : -b);
          const double throw_out = big * unit(rng);
          x += throw_out * out.x();
          y += throw_out * out.y();
        }
      }
      if (g.jitter > 0.0) {
        x += g.jitter * (2.0 * unit(rng) - 1.0);
        y += g.jitter * (2.0 * unit(rng) - 1.0);
      }
      dots.emplace_back(x, y);
    }
  }

  // A random place outside the face, between `near` and `far` from it.
  auto outside = [&](double near, double far) {
    for (;;) {
      const double s = (2.0 * unit(rng) - 1.0) * (half_l + far);
      const double t = (2.0 * unit(rng) - 1.0) * (half_w + far);
      const double gap = std::max(std::abs(s) - half_l, std::abs(t) - half_w);
      if (gap >= near && gap <= far) return Vector2d(scene.centre + s * a + t * b);
    }
  };
  for (int k = 0; k < d.strays; ++k) dots.push_back(outside(0.1, 1.5));
  for (int k = 0; k < d.patches; ++k) {
    const Vector2d corner = outside(0.2, 1.0);
    const double turn = 2.0 * kPi * unit(rng);
    const Vector2d u(std::cos(turn), std::sin(turn));
    for (int i = 0; i < 7; ++i) {
      for (int j = 0; j < 7; ++j) {
        dots.push_back(corner + 6.0 * kMillimetre * (i * u + j * Perp(u)));
      }
    }
  }
  return dots;
}

// A module anywhere within a metre of the origin, at any rotation, seen
// through a grid at any rotation and phase.
Scene RandomScene(std::mt19937& rng, double spacing_a, double spacing_b) {
  Scene scene;
  // One draw per statement: see MakeDots.
  scene.centre.x() = 2.0 * UniformUnit(rng) - 1.0;
  scene.centre.y() = 2.0 * UniformUnit(rng) - 1.0;
  scene.angle = 2.0 * kPi * UniformUnit(rng);
  scene.grid.spacing_a = spacing_a;
  scene.grid.spacing_b = spacing_b;
  scene.grid.angle = 2.0 * kPi * UniformUnit(rng);
  scene.grid.phase.x() = spacing_a * UniformUnit(rng);
  scene.grid.phase.y() = spacing_b * UniformUnit(rng);
  return scene;
}

// ------------------------------------------------------------- the errors

// The angle between two directions along a line, either way round: 0 to
// pi/2.
double LineAngle(double a, double b) {
  return std::abs(std::remainder(a - b, kPi));
}

struct Errors {
  double centre = 0.0;  // metres
  double angle = 0.0;   // radians
  double length = 0.0;  // fitted minus true, metres
  double width = 0.0;
};

Errors ErrorsOf(const OrientedRectangle& fit, const Scene& truth) {
  Errors e;
  e.centre = (fit.centre - truth.centre).norm();
  e.angle = LineAngle(fit.Angle(), truth.angle);
  e.length = fit.length - truth.length;
  e.width = fit.width - truth.width;
  return e;
}

// Worst, root mean square and mean of a quantity over many fits.
struct Stats {
  double worst = 0.0;
  double sum = 0.0;
  double sum_of_squares = 0.0;
  int count = 0;

  void Add(double x) {
    worst = std::max(worst, std::abs(x));
    sum += x;
    sum_of_squares += x * x;
    ++count;
  }
  double Rms() const { return std::sqrt(sum_of_squares / count); }
  double Mean() const { return sum / count; }
};

// Every value of a quantity over many scenes, for its median, a high
// percentile and its worst, and the seed that gave the worst.
struct Spread {
  std::vector<double> values;
  double worst = -std::numeric_limits<double>::infinity();
  int worst_seed = 0;

  void Add(double x, int seed) {
    values.push_back(x);
    if (x > worst) {
      worst = x;
      worst_seed = seed;
    }
  }
  // The value that a fraction `p` of the others do not exceed.
  double Quantile(double p) const {
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    const size_t k = static_cast<size_t>(
        std::ceil(p * static_cast<double>(sorted.size())));
    return sorted[std::min(sorted.size(), std::max<size_t>(k, 1)) - 1];
  }
  void Print(const char* name, double unit, const char* unit_name) const {
    std::printf(
        "%s over %zu scenes, %s: median %.3f, 99th percentile %.3f, worst "
        "%.3f (seed %d)\n",
        name, values.size(), unit_name, Quantile(0.5) / unit,
        Quantile(0.99) / unit, worst / unit, worst_seed);
  }
};

struct Accuracy {
  Stats centre, angle, length, width;
  void Add(const Errors& e) {
    centre.Add(e.centre);
    angle.Add(e.angle);
    length.Add(e.length);
    width.Add(e.width);
  }
};

void PrintHeader() {
  std::printf(
      "%-30s %-11s %17s %17s %17s %17s\n", "", "", "centre mm",
      "rotation deg", "length mm", "width mm");
  std::printf("%-30s %-11s %8s %8s %8s %8s %8s %8s %8s %8s\n", "dots", "fit",
              "worst", "rms", "worst", "rms", "mean", "rms", "mean", "rms");
}

void PrintRow(const std::string& name, const char* fit, const Accuracy& a) {
  std::printf(
      "%-30s %-11s %8.2f %8.2f %8.3f %8.3f %8.2f %8.2f %8.2f %8.2f\n",
      name.c_str(), fit, a.centre.worst / kMillimetre,
      a.centre.Rms() / kMillimetre, a.angle.worst / kDegree,
      a.angle.Rms() / kDegree, a.length.Mean() / kMillimetre,
      a.length.Rms() / kMillimetre, a.width.Mean() / kMillimetre,
      a.width.Rms() / kMillimetre);
}

// ------------------------------------------------- the scenes' random draws

// The draws the scenes are made from give these numbers on every platform:
// they were printed once, on 64-bit ARM, and match libstdc++'s
// distributions there bit for bit. The normal draws go through std::log,
// which may differ in the last bit between math libraries.
TEST(PortableRandomTest, SameDrawsEverywhere) {
  std::mt19937 rng(42);
  EXPECT_EQ(UniformUnit(rng), 0x1.97d47b66bfc3cp-1);
  EXPECT_EQ(UniformUnit(rng), 0x1.77aca8779b102p-3);
  EXPECT_EQ(UniformUnit(rng), 0x1.8f33a88f76c7fp-1);
  EXPECT_EQ(UniformReal(rng, -2.0, 3.0), 0x1.f7efb8cdfc90ep-1);
  EXPECT_EQ(UniformReal(rng, -2.0, 3.0), 0x1.d553d2ee3da1cp-3);
  EXPECT_EQ(cell::UniformIndex(rng, 1000), 155);
  EXPECT_EQ(cell::UniformIndex(rng, 1000), 99);
  EXPECT_EQ(cell::UniformIndex(rng, 1000), 58);
  EXPECT_EQ(cell::UniformIndex(rng, 1000), 459);
  cell::NormalDraw normal(1.0, 0.5);
  EXPECT_NEAR(normal(rng), 0.5573848333118554, 1e-15);
  EXPECT_NEAR(normal(rng), 0.79390576512471278, 1e-15);
  EXPECT_NEAR(normal(rng), 0.75869061194450604, 1e-15);
  EXPECT_EQ(rng(), 4165731073u);
}

// ----------------------------------------------------------- convex hull

TEST(ConvexHullTest, SquareKeepsOnlyItsCorners) {
  // Corners, points along the edges, points inside, and repeats.
  const Points2d points = {
      {0.5, 0.5}, {1, 1},     {0, 0},    {0.5, 0},  {1, 0},   {1, 0.25},
      {0, 1},     {0.2, 0.7}, {0, 0.5},  {1, 1},    {0.5, 1}, {0, 0},
  };
  const Points2d hull = ConvexHull(points);
  // Anticlockwise from the lowest x, then lowest y.
  const Points2d expected = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  ASSERT_EQ(hull.size(), expected.size());
  for (size_t k = 0; k < hull.size(); ++k) {
    EXPECT_EQ(hull[k], expected[k]) << "corner " << k;
  }
}

TEST(ConvexHullTest, FewRepeatedOrCollinearPoints) {
  const Vector2d a(1, 2), b(3, 2), c(2, 2);
  EXPECT_TRUE(ConvexHull({}).empty());
  EXPECT_EQ(ConvexHull({a}), Points2d({a}));
  EXPECT_EQ(ConvexHull({a, a, a}), Points2d({a}));
  EXPECT_EQ(ConvexHull({b, a}), Points2d({a, b}));
  // All on one line: only the two ends.
  EXPECT_EQ(ConvexHull({c, b, a, c, Vector2d(2.5, 2)}), Points2d({a, b}));
  // Points that are not numbers are ignored.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const Points2d hull =
      ConvexHull({Vector2d(0, 0), Vector2d(nan, 1), Vector2d(1, 0),
                  Vector2d(0, inf), Vector2d(0, 1)});
  EXPECT_EQ(hull, Points2d({{0, 0}, {1, 0}, {0, 1}}));
}

// ------------------------------------------------ minimum-area rectangle

TEST(MinAreaRectangleTest, RotatedRectangleFromItsCorners) {
  for (double degrees : {0.0, 30.0, 90.0, 120.0, 179.0, -45.0, 270.0}) {
    SCOPED_TRACE(testing::Message() << degrees << " degrees");
    OrientedRectangle truth;
    truth.centre = Vector2d(2.0, -1.0);
    truth.axis = Vector2d(std::cos(degrees * kDegree),
                          std::sin(degrees * kDegree));
    truth.length = 3.0;
    truth.width = 1.0;
    const auto corners = truth.Corners();
    // Corners and a few points inside.
    Points2d points(corners.begin(), corners.end());
    points.push_back(truth.centre);
    points.push_back(truth.centre + 0.4 * truth.axis);

    const std::optional<OrientedRectangle> r = MinAreaRectangle(points);
    ASSERT_TRUE(r.has_value());
    EXPECT_NEAR((r->centre - truth.centre).norm(), 0.0, 1e-12);
    EXPECT_NEAR(r->length, 3.0, 1e-12);
    EXPECT_NEAR(r->width, 1.0, 1e-12);
    EXPECT_NEAR(LineAngle(r->Angle(), degrees * kDegree), 0.0, 1e-12);
    // The canonical one of the two directions: positive x, so the angle is
    // in (-90, 90] degrees.
    EXPECT_GT(r->axis.x(), -1e-12);
    EXPECT_NEAR(r->axis.norm(), 1.0, 1e-12);
  }
}

TEST(MinAreaRectangleTest, TriangleAndSquare) {
  // A flat triangle: the smallest rectangle lies along its long base.
  const std::optional<OrientedRectangle> t =
      MinAreaRectangle({{0, 0}, {10, 0}, {5, 1}});
  ASSERT_TRUE(t.has_value());
  EXPECT_NEAR(t->length, 10.0, 1e-12);
  EXPECT_NEAR(t->width, 1.0, 1e-12);
  EXPECT_NEAR((t->centre - Vector2d(5, 0.5)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(t->axis.x(), 1.0, 1e-12);

  // A square turned 20 degrees, with a point on each edge's middle.
  OrientedRectangle square;
  square.axis = Vector2d(std::cos(20 * kDegree), std::sin(20 * kDegree));
  square.length = square.width = 1.0;
  const auto c = square.Corners();
  const std::optional<OrientedRectangle> s = MinAreaRectangle(
      {c[0], c[1], c[2], c[3], 0.5 * (c[0] + c[1]), 0.5 * (c[1] + c[2])});
  ASSERT_TRUE(s.has_value());
  EXPECT_NEAR(s->length, 1.0, 1e-12);
  EXPECT_NEAR(s->width, 1.0, 1e-12);
  EXPECT_NEAR(s->centre.norm(), 0.0, 1e-12);
  // Either side of a square is its "long" side.
  EXPECT_NEAR(std::remainder(s->Angle() - 20 * kDegree, kPi / 2), 0.0, 1e-12);
}

// The same answer as trying every hull edge by brute force, on random
// clouds, some with repeats, some nearly in a line, and some large enough
// for the hull's quick pre-pass to throw most points out.
TEST(MinAreaRectangleTest, MatchesBruteForce) {
  std::mt19937 rng(11);
  for (int trial = 0; trial < 300; ++trial) {
    SCOPED_TRACE(testing::Message() << "trial " << trial);
    const int n = trial % 10 == 9 ? 400 : 3 + trial % 40;
    const double squash = trial % 7 == 0 ? 1e-3 : 1.0;
    Points2d points;
    for (int i = 0; i < n; ++i) {
      const double x = UniformReal(rng, -1.0, 1.0);
      const double y = squash * UniformReal(rng, -1.0, 1.0);
      points.emplace_back(x, y);
      if (i % 5 == 0) points.push_back(points.back());
    }
    const std::optional<OrientedRectangle> r = MinAreaRectangle(points);
    ASSERT_TRUE(r.has_value());

    const Points2d hull = ConvexHull(points);
    double best = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < hull.size(); ++i) {
      const Vector2d u = (hull[(i + 1) % hull.size()] - hull[i]).normalized();
      double lo = 1e9, hi = -1e9, top = 0.0;
      for (const Vector2d& p : hull) {
        lo = std::min(lo, u.dot(p - hull[i]));
        hi = std::max(hi, u.dot(p - hull[i]));
        top = std::max(top, Perp(u).dot(p - hull[i]));
      }
      best = std::min(best, (hi - lo) * top);
    }
    EXPECT_NEAR(r->length * r->width, best, 1e-12);

    // Every point inside.
    for (const Vector2d& p : points) {
      const Vector2d q = p - r->centre;
      EXPECT_LE(std::abs(q.dot(r->axis)), 0.5 * r->length + 1e-12);
      EXPECT_LE(std::abs(q.dot(r->Across())), 0.5 * r->width + 1e-12);
    }
    EXPECT_GE(r->length, r->width);
  }
}

TEST(MinAreaRectangleTest, NoAreaGivesNothing) {
  const Vector2d a(1, 1), b(2, 3);
  EXPECT_FALSE(MinAreaRectangle({}).has_value());
  EXPECT_FALSE(MinAreaRectangle({a}).has_value());
  EXPECT_FALSE(MinAreaRectangle({a, b}).has_value());
  EXPECT_FALSE(MinAreaRectangle({a, a, a, a}).has_value());
  EXPECT_FALSE(
      MinAreaRectangle({a, b, 2 * b - a, 3 * b - 2 * a, a}).has_value());
}

// ------------------------------------------------------------- clusters

TEST(LargestClusterTest, KeepsTheBiggestClump) {
  Points2d points;
  std::vector<int> big;
  // Interleaved: a 20 x 20 block at 10 mm, a 10 x 10 block a metre away,
  // single strays, and points that are not numbers.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int i = 0; i < 400; ++i) {
    big.push_back(static_cast<int>(points.size()));
    points.emplace_back(0.01 * (i % 20), 0.01 * (i / 20));
    if (i < 100) points.emplace_back(1.0 + 0.01 * (i % 10), 0.01 * (i / 10));
    if (i % 50 == 0) points.emplace_back(-0.5 - 0.01 * i, 0.3);
    if (i % 70 == 0) points.emplace_back(nan, 0.0);
  }
  EXPECT_EQ(LargestCluster(points, 0.025), big);

  EXPECT_TRUE(LargestCluster(points, 0.0).empty());
  EXPECT_TRUE(LargestCluster(points, -1.0).empty());
  EXPECT_TRUE(LargestCluster(points, nan).empty());
  EXPECT_TRUE(LargestCluster({}, 0.02).empty());
}

TEST(LargestClusterTest, GapsBelowOneCellJoinAndBeyondTwoCellsSeparate) {
  // Two rows of dots 10 mm apart along x, with a gap between them.
  auto rows = [](double gap) {
    Points2d points;
    for (int i = 0; i < 30; ++i) points.emplace_back(0.01 * i, 0.0);
    for (int i = 0; i < 20; ++i) points.emplace_back(0.29 + gap + 0.01 * i, 0);
    return points;
  };
  const double cell_size = 0.02;
  EXPECT_EQ(LargestCluster(rows(0.019), cell_size).size(), 50u);
  EXPECT_EQ(LargestCluster(rows(2.0 * std::sqrt(2.0) * cell_size + 1e-3),
                           cell_size)
                .size(),
            30u);
}

TEST(DotSpacingTest, IsTheSmallerGridSpacing) {
  std::mt19937 rng(12);
  Scene scene = RandomScene(rng, 6 * kMillimetre, 9 * kMillimetre);
  scene.damage.strays = 30;
  const std::optional<double> spacing = DotSpacing(MakeDots(scene, rng));
  ASSERT_TRUE(spacing.has_value());
  EXPECT_NEAR(*spacing, 6 * kMillimetre, 0.5 * kMillimetre);

  EXPECT_FALSE(DotSpacing({}).has_value());
  EXPECT_FALSE(DotSpacing({Vector2d(1, 1)}).has_value());
  EXPECT_FALSE(DotSpacing({Vector2d(1, 1), Vector2d(1, 1)}).has_value());
}

// --------------------------------------------------------- the whole fit

// The stated error, from the dot spacing. Most of it comes from where each
// edge falls in the gap between rows of dots; see
// RefineOptions::correct_edge_bias.
//
//   Centre: each edge is found to within half a row spacing, exactly that
//   bound only for a grid whose rows run along it and far better for a
//   slanted one. The centre is halfway between two opposite edges, so it is
//   within half the larger spacing too. 0.6 of it allows for ragged and
//   missing edge dots.
//   Size: the errors of two opposite edges add: one larger spacing, 1.2
//   with the same allowance.
//   Rotation: for rows along a side, the edge can be anywhere in the gap
//   beyond the outermost row, at each end of the side independently: a tilt
//   of up to one spacing over the module's length. On a slanted grid the
//   error is several times smaller, except on one whose rows drift by about
//   a spacing over the length (slanted by about 0.45 degrees on the coarse
//   grid), which the scenes below with rows along a side do not try. There
//   0.6 of a spacing is sometimes exceeded: by 24 of 10000 clean random
//   scenes on the 12 x 18 mm grid (the first in RaggedScene), at worst 0.44
//   degrees, 0.98 of a spacing over the length. The fixed seeds below do not
//   draw one.
double CentreTolerance(const Grid& g) { return 0.6 * g.Larger(); }
double SizeTolerance(const Grid& g) { return 1.2 * g.Larger(); }
double AngleTolerance(const Grid& g) { return 0.6 * g.Larger() / kLength; }

void ExpectWithin(const Errors& e, const Grid& g) {
  EXPECT_LT(e.centre, CentreTolerance(g));
  EXPECT_LT(e.angle, AngleTolerance(g));
  EXPECT_LT(std::abs(e.length), SizeTolerance(g));
  EXPECT_LT(std::abs(e.width), SizeTolerance(g));
}

// Nothing in a returned fit may be a NaN or infinite.
void ExpectFinite(const RectangleFit& fit) {
  EXPECT_TRUE(fit.rectangle.centre.allFinite());
  EXPECT_TRUE(fit.rectangle.axis.allFinite());
  EXPECT_TRUE(std::isfinite(fit.rectangle.length));
  EXPECT_TRUE(std::isfinite(fit.rectangle.width));
  EXPECT_TRUE(std::isfinite(fit.rms_residual));
  for (const Vector2d& corner : fit.corners) EXPECT_TRUE(corner.allFinite());
  for (const cell::SideFit& side : fit.sides) {
    EXPECT_TRUE(side.outward.allFinite());
    EXPECT_TRUE(std::isfinite(side.angle_error));
    EXPECT_TRUE(std::isfinite(side.edge_correction));
  }
}

std::optional<RectangleFit> Fit(const Points2d& dots,
                                const RectangleFitOptions& options = {},
                                RectangleRefusal* refusal = nullptr) {
  return FitRectangle(dots, kLength, kWidth, options, refusal);
}

struct Config {
  std::string name;
  double spacing_a;  // mm
  double spacing_b;
  bool damaged;
  bool rows_along_a_side;
  int seeds;
};

// The stage's main claim, and the table of how well it does. Damaged means
// Typical(): edges ragged by a pixel, 10% of edge dots missing and 5%
// flying, 20 strays and two stray patches.
TEST(FitRectangleTest, AccuracyOverSeedsAndRotations) {
  const std::vector<Config> configs = {
      {"6 x 9 mm", 6, 9, false, false, 8},
      {"6 x 9 mm, damaged", 6, 9, true, false, 8},
      {"6 x 9 mm, rows along a side", 6, 9, false, true, 8},
      {"12 x 18 mm", 12, 18, false, false, 20},
      {"12 x 18 mm, damaged", 12, 18, true, false, 20},
      {"12 x 18 mm, rows along a side", 12, 18, false, true, 16},
  };
  PrintHeader();
  Accuracy all_first, all_refined;
  for (const Config& config : configs) {
    Accuracy first, refined;
    for (int seed = 1; seed <= config.seeds; ++seed) {
      SCOPED_TRACE(testing::Message() << config.name << ", seed " << seed);
      std::mt19937 rng(seed);
      Scene scene = RandomScene(rng, config.spacing_a * kMillimetre,
                                config.spacing_b * kMillimetre);
      if (config.rows_along_a_side) {
        // Rows exactly along the long or the short side, or a hair off,
        // and every third module exactly level or at a quarter turn.
        const double off[4] = {0.0, 0.01, -0.03, 0.1};
        if (seed % 3 == 0) scene.angle = (seed % 4) * kPi / 2;
        scene.grid.angle =
            scene.angle + (seed % 4) * kPi / 2 + off[seed % 4] * kDegree;
      }
      if (config.damaged) scene.damage = Typical();
      const Points2d dots = MakeDots(scene, rng);

      RectangleRefusal refusal;
      const std::optional<RectangleFit> fit = Fit(dots, {}, &refusal);
      ASSERT_TRUE(fit.has_value()) << refusal.detail;
      ExpectFinite(*fit);
      const Errors e = ErrorsOf(fit->rectangle, scene);
      ExpectWithin(e, scene.grid);
      first.Add(ErrorsOf(fit->first_guess, scene));
      refined.Add(e);
      all_first.Add(ErrorsOf(fit->first_guess, scene));
      all_refined.Add(e);
    }
    PrintRow(config.name, "first guess", first);
    PrintRow(config.name, "refined", refined);
    // With damaged edges, as real ones are, the refinement pays. On a clean
    // slanted grid the outermost dots are already within a fraction of a
    // millimetre of the true edges, and the first guess is about as good.
    if (config.damaged) {
      EXPECT_LT(refined.centre.Rms(), 0.5 * first.centre.Rms());
      EXPECT_LT(refined.angle.Rms(), 0.75 * first.angle.Rms());
      EXPECT_LT(std::abs(refined.length.Mean()),
                0.2 * std::abs(first.length.Mean()));
    }
  }
  PrintRow("all of the above", "first guess", all_first);
  PrintRow("all of the above", "refined", all_refined);
  EXPECT_LT(all_refined.centre.Rms(), all_first.centre.Rms());
  EXPECT_LT(all_refined.angle.Rms(), all_first.angle.Rms());
}

// A rectangle looks the same turned half a turn; the answer's long side
// points toward +x. Near a quarter turn that flips from one side to the
// other.
TEST(FitRectangleTest, ReturnsTheHalfTurnWithPositiveX) {
  std::mt19937 rng(21);
  for (double degrees : {0.0, 100.0, 180.0, 269.5, 270.5, -0.3, 45.0}) {
    SCOPED_TRACE(testing::Message() << degrees << " degrees");
    Scene scene = RandomScene(rng, 12 * kMillimetre, 18 * kMillimetre);
    scene.angle = degrees * kDegree;
    const std::optional<RectangleFit> fit = Fit(MakeDots(scene, rng));
    ASSERT_TRUE(fit.has_value());
    const OrientedRectangle& r = fit->rectangle;
    EXPECT_GT(r.axis.x(), 0.0);
    EXPECT_GT(r.Angle(), -kPi / 2);
    EXPECT_LE(r.Angle(), kPi / 2);
    EXPECT_NEAR(r.Angle(), std::remainder(degrees * kDegree, kPi),
                AngleTolerance(scene.grid));

    // Corners anticlockwise from centre - length/2 axis - width/2 across,
    // and each side between its corners, facing out.
    double area = 0.0;
    for (int k = 0; k < 4; ++k) {
      const Vector2d& p = fit->corners[k];
      const Vector2d& q = fit->corners[(k + 1) % 4];
      area += 0.5 * (p.x() * q.y() - q.x() * p.y());
      EXPECT_EQ(fit->sides[k].start, p);
      EXPECT_EQ(fit->sides[k].end, q);
    }
    EXPECT_NEAR(area, r.length * r.width, 1e-9);
    EXPECT_NEAR((fit->corners[0] - (r.centre - 0.5 * r.length * r.axis -
                                    0.5 * r.width * r.Across()))
                    .norm(),
                0.0, 1e-9);
    EXPECT_NEAR((fit->sides[0].outward + r.Across()).norm(), 0.0, 1e-9);
    EXPECT_NEAR((fit->sides[1].outward - r.axis).norm(), 0.0, 1e-9);
    EXPECT_NEAR((fit->sides[2].outward - r.Across()).norm(), 0.0, 1e-9);
    EXPECT_NEAR((fit->sides[3].outward + r.axis).norm(), 0.0, 1e-9);
  }
}

// Dots are pixel centres, so the outermost ones sit inside the true edge.
// Without the correction the size comes out small; with it, right on
// average. Rows along a side are the worst case for the outermost dots.
TEST(FitRectangleTest, SizeIsUnbiasedOnACoarseGrid) {
  const int seeds = 16;
  for (bool rows_along_a_side : {false, true}) {
    Stats length_with, width_with, length_without, width_without;
    Grid grid;
    for (int seed = 1; seed <= seeds; ++seed) {
      SCOPED_TRACE(testing::Message() << "seed " << seed);
      std::mt19937 rng(100 + seed);
      Scene scene = RandomScene(rng, 12 * kMillimetre, 18 * kMillimetre);
      if (rows_along_a_side) {
        scene.grid.angle = scene.angle + (seed % 4) * kPi / 2;
      }
      grid = scene.grid;
      const Points2d dots = MakeDots(scene, rng);
      RectangleFitOptions without;
      without.refine.correct_edge_bias = false;
      // Measuring here, not checking: a size short by more than the usual
      // tolerance is what this is about.
      without.size_tolerance = 0.1;
      const std::optional<RectangleFit> a = Fit(dots);
      const std::optional<RectangleFit> b = Fit(dots, without);
      ASSERT_TRUE(a.has_value());
      ASSERT_TRUE(b.has_value());
      length_with.Add(a->rectangle.length - kLength);
      width_with.Add(a->rectangle.width - kWidth);
      length_without.Add(b->rectangle.length - kLength);
      width_without.Add(b->rectangle.width - kWidth);
    }
    std::printf(
        "12 x 18 mm, %s: size error, mean (rms) mm\n"
        "  corrected:            length %6.2f (%5.2f)  width %6.2f (%5.2f)\n"
        "  through outermost dots: length %6.2f (%5.2f)  width %6.2f (%5.2f)\n",
        rows_along_a_side ? "rows along a side" : "slanted grid",
        length_with.Mean() / kMillimetre, length_with.Rms() / kMillimetre,
        width_with.Mean() / kMillimetre, width_with.Rms() / kMillimetre,
        length_without.Mean() / kMillimetre,
        length_without.Rms() / kMillimetre,
        width_without.Mean() / kMillimetre, width_without.Rms() / kMillimetre);
    // The mean of the corrected size is zero within three standard errors
    // (the scatter over this many seeds); the uncorrected one is short by a
    // good part of a spacing.
    auto three_errors = [seeds](const Stats& x) {
      return 3.0 * x.Rms() / std::sqrt(static_cast<double>(seeds));
    };
    EXPECT_LT(std::abs(length_with.Mean()), three_errors(length_with));
    EXPECT_LT(std::abs(width_with.Mean()), three_errors(width_with));
    EXPECT_LT(length_without.Mean(), -0.2 * grid.Larger());
    EXPECT_LT(width_without.Mean(), -0.2 * grid.Larger());
    EXPECT_LT(length_without.Mean(), length_with.Mean() -
                                         three_errors(length_with));
    EXPECT_LT(width_without.Mean(), width_with.Mean() -
                                        three_errors(width_with));
  }
}

TEST(FitRectangleTest, StraysAreIgnored) {
  for (int seed = 1; seed <= 2; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(200 + seed);
    const Scene scene = RandomScene(rng, 6 * kMillimetre, 9 * kMillimetre);
    std::mt19937 again = rng;
    const Points2d clean = MakeDots(scene, rng);

    // The same dots with strays added: far ones and patches, which the
    // clump drops, and close ones, 20 to 45 mm outside the edges, which it
    // keeps and the line fits must reject.
    Scene messy = scene;
    messy.damage.strays = 40;
    messy.damage.patches = 4;
    Points2d dots = MakeDots(messy, again);
    const Vector2d a = scene.Axis();
    const Vector2d b = Perp(a);
    for (int k = 0; k < 20; ++k) {
      const double out = 0.02 + 0.025 * UniformUnit(rng);
      const double along = UniformUnit(rng) - 0.5;
      dots.push_back(k % 2 == 0
                         ? Vector2d(scene.centre + along * kLength * a +
                                    (0.5 * kWidth + out) * b)
                         : Vector2d(scene.centre + (0.5 * kLength + out) * a +
                                    along * kWidth * b));
    }

    const std::optional<RectangleFit> reference = Fit(clean);
    const std::optional<RectangleFit> fit = Fit(dots);
    ASSERT_TRUE(reference.has_value());
    ASSERT_TRUE(fit.has_value());
    // Every face dot kept, the far strays dropped, some of the close ones
    // kept.
    EXPECT_GE(fit->dots_kept, static_cast<int>(clean.size()));
    EXPECT_LE(fit->dots_kept, static_cast<int>(clean.size()) + 20);
    ExpectWithin(ErrorsOf(fit->rectangle, scene), scene.grid);
    // The close strays change the first guess, and so which dots fall in
    // which strip, so the answer moves by about its own scatter (see the
    // table), and no more.
    const double spacing = scene.grid.Larger();
    EXPECT_LT((fit->rectangle.centre - reference->rectangle.centre).norm(),
              0.1 * spacing);
    EXPECT_LT(LineAngle(fit->rectangle.Angle(), reference->rectangle.Angle()),
              0.01 * kDegree);
    EXPECT_NEAR(fit->rectangle.length, reference->rectangle.length,
                0.15 * spacing);
    EXPECT_NEAR(fit->rectangle.width, reference->rectangle.width,
                0.15 * spacing);
  }
}

// How many random scenes the tests of a bound over many scenes run. The
// bounds were set from the worst of 10000 (seeds 1 to 10000); these are the
// first of them. Seeds 1 to 3 alone, which the tests below once ran, passed
// a bound on the turn that 58% of scenes exceed.
constexpr int kManyScenes = 500;

// A module on the coarse grid, its dots clean, and the same module and grid
// with its edges damaged worse than Typical(): a third of the edge dots
// missing, a tenth flying.
struct RaggedScene {
  Scene scene;
  std::optional<RectangleFit> clean;
  std::optional<RectangleFit> damaged;
};

RaggedScene MakeRaggedScene(int seed) {
  std::mt19937 rng(300 + seed);
  RaggedScene r;
  r.scene = RandomScene(rng, 12 * kMillimetre, 18 * kMillimetre);
  r.clean = Fit(MakeDots(r.scene, rng));
  Scene damaged = r.scene;
  damaged.damage.ragged = 1.0;
  damaged.damage.missing = 0.3;
  damaged.damage.flying = 0.1;
  r.damaged = Fit(MakeDots(damaged, rng));
  return r;
}

TEST(FitRectangleTest, RaggedEdgesAndMissingEdgeDotsMoveItLittle) {
  for (int seed = 1; seed <= 3; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const RaggedScene r = MakeRaggedScene(seed);
    const Scene& scene = r.scene;
    ASSERT_TRUE(r.clean.has_value());
    ASSERT_TRUE(r.damaged.has_value());
    const RectangleFit* clean = &*r.clean;
    const RectangleFit* fit = &*r.damaged;
    ExpectWithin(ErrorsOf(fit->rectangle, scene), scene.grid);
    EXPECT_LT((fit->rectangle.centre - clean->rectangle.centre).norm(),
              0.2 * scene.grid.Larger());
    // How far the damage turns it is checked over many scenes, in the next
    // test.

    // Missing edge dots make the face look smaller by their share of the
    // edge: about 0.3 of a spacing each side here.
    EXPECT_NEAR(fit->rectangle.length, clean->rectangle.length,
                0.8 * scene.grid.Larger());
    EXPECT_NEAR(fit->rectangle.width, clean->rectangle.width,
                0.8 * scene.grid.Larger());
    // The edge dots the sides were fitted to scatter by about the
    // raggedness.
    for (const cell::SideFit& side : fit->sides) {
      EXPECT_GT(side.rms_residual, 0.2 * scene.grid.Larger());
      EXPECT_GT(side.support, 0.8);
    }
  }
}

// The damage turns the fit by the scatter of the edge dots it rests on, one
// per strip three spacings wide, some 57 along a long side: a few hundredths
// of a degree typically, and over 10000 scenes up to 0.567 degrees (seed
// 6091). The bound is about 1.25 times that worst. Half the scenes exceed
// the 0.05 degrees once claimed here. On a scene of rows nearly along a side
// part of the difference is the clean fit's own error.
TEST(FitRectangleTest, RaggedEdgesTurnItLittleOverManyScenes) {
  Spread turn;
  for (int seed = 1; seed <= kManyScenes; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const RaggedScene r = MakeRaggedScene(seed);
    ASSERT_TRUE(r.clean.has_value());
    ASSERT_TRUE(r.damaged.has_value());
    const double angle =
        LineAngle(r.damaged->rectangle.Angle(), r.clean->rectangle.Angle());
    EXPECT_LT(angle, 0.7 * kDegree);
    turn.Add(angle, seed);
  }
  turn.Print("ragged edges: turn from the clean fit", kDegree, "deg");
}

TEST(FitRectangleTest, WrongSizesAreRefused) {
  std::mt19937 rng(400);
  Scene scene = RandomScene(rng, 12 * kMillimetre, 18 * kMillimetre);
  scene.damage = Typical();
  RectangleRefusal why;

  // A face 0.9 m wide.
  Scene narrow = scene;
  narrow.width = 0.9;
  EXPECT_FALSE(Fit(MakeDots(narrow, rng), {}, &why).has_value());
  EXPECT_EQ(why.reason, RectangleFitFailure::kSizeMismatch);
  EXPECT_NE(why.detail.find("width"), std::string::npos) << why.detail;
  std::printf("2.3 x 0.9 m: %s: %s\n", cell::FailureName(why.reason),
              why.detail.c_str());

  // Two modules side by side, 10 mm apart: one clump of 2.3 x 2.21 m.
  Scene beside = scene;
  beside.centre += (kWidth + 0.01) * Perp(scene.Axis());
  Points2d both = MakeDots(scene, rng);
  const Points2d other = MakeDots(beside, rng);
  both.insert(both.end(), other.begin(), other.end());
  EXPECT_FALSE(Fit(both, {}, &why).has_value());
  EXPECT_EQ(why.reason, RectangleFitFailure::kSizeMismatch);
  std::printf("two side by side: %s: %s\n", cell::FailureName(why.reason),
              why.detail.c_str());

  // End to end.
  Scene after = scene;
  after.centre += (kLength + 0.01) * scene.Axis();
  Points2d row = MakeDots(scene, rng);
  const Points2d next = MakeDots(after, rng);
  row.insert(row.end(), next.begin(), next.end());
  EXPECT_FALSE(Fit(row, {}, &why).has_value());
  EXPECT_EQ(why.reason, RectangleFitFailure::kSizeMismatch);

  // Far enough apart to be two clumps: the larger is fitted, here the
  // module rather than the 0.6 m wide piece of another beside it.
  Scene apart = scene;
  apart.width = 0.6;
  apart.centre += (0.5 * kWidth + 0.5 * apart.width + 0.2) * Perp(scene.Axis());
  Points2d two = MakeDots(scene, rng);
  const Points2d second = MakeDots(apart, rng);
  two.insert(two.end(), second.begin(), second.end());
  const std::optional<RectangleFit> fit = Fit(two);
  ASSERT_TRUE(fit.has_value());
  EXPECT_LT((fit->rectangle.centre - scene.centre).norm(),
            CentreTolerance(scene.grid));
}

// Something in front of the module hides part of it.
TEST(FitRectangleTest, BitesOutOfACornerOrASideAreHandled) {
  const double l = 0.5 * kLength;
  const double w = 0.5 * kWidth;
  const std::vector<std::pair<std::string, std::function<bool(double, double)>>>
      bites = {
          {"corner 0.5 x 0.35 m",
           [=](double s, double t) { return s > l - 0.5 && t > w - 0.35; }},
          {"disc of radius 0.3 m on a long side",
           [=](double s, double t) {
             return std::hypot(s - 0.4, t + w) < 0.3;
           }},
          {"disc of radius 0.25 m on a short side",
           [=](double s, double t) {
             return std::hypot(s - l, t - 0.1) < 0.25;
           }},
          {"three quarters of a long side, 0.25 m deep",
           [=](double s, double t) { return s > -l + 0.6 && t > w - 0.25; }},
      };
  for (const auto& [name, hidden] : bites) {
    for (int seed = 1; seed <= 2; ++seed) {
      SCOPED_TRACE(testing::Message() << name << ", seed " << seed);
      std::mt19937 rng(500 + seed);
      Scene scene = RandomScene(rng, 6 * kMillimetre, 9 * kMillimetre);
      scene.damage = Typical();
      scene.hidden = hidden;
      RectangleRefusal why;
      const std::optional<RectangleFit> fit =
          Fit(MakeDots(scene, rng), {}, &why);
      ASSERT_TRUE(fit.has_value()) << why.detail;
      ExpectFinite(*fit);
      const Errors e = ErrorsOf(fit->rectangle, scene);
      ExpectWithin(e, scene.grid);
      EXPECT_FALSE(fit->used_known_size);
      double least = 1.0;
      for (const cell::SideFit& side : fit->sides) {
        least = std::min(least, side.support);
      }
      if (seed == 1) {
        std::printf(
            "bite, %s: centre %.2f mm, rotation %.3f deg, length %+.2f mm, "
            "width %+.2f mm; least side support %.2f\n",
            name.c_str(), e.centre / kMillimetre, e.angle / kDegree,
            e.length / kMillimetre, e.width / kMillimetre, least);
      }
    }
  }
}

// A whole end hidden. If the edge of what hides it is not a straight line
// along the end, the end is missing: refused, or with allow_missing_side
// placed from the other end at the known length. If it is, nothing in the
// dots tells it from a shorter module: refused either way.
TEST(FitRectangleTest, AHiddenEndIsRefusedOrPlacedFromTheKnownSize) {
  const double l = 0.5 * kLength;
  const double w = 0.5 * kWidth;
  RectangleFitOptions allow;
  allow.allow_missing_side = true;
  for (int seed = 1; seed <= 2; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    std::mt19937 rng(600 + seed);
    Scene scene = RandomScene(rng, 6 * kMillimetre, 9 * kMillimetre);
    scene.damage = Typical();
    // A curved, slanted outline: 0.1 m of the length hidden at one corner
    // and 0.45 m at the other.
    scene.hidden = [=](double s, double t) {
      const double x = (t + w) / kWidth;
      return s > l - 0.1 - 0.35 * x * x;
    };
    const Points2d dots = MakeDots(scene, rng);
    RectangleRefusal why;
    EXPECT_FALSE(Fit(dots, {}, &why).has_value());
    EXPECT_TRUE(why.reason == RectangleFitFailure::kSideMissing ||
                why.reason == RectangleFitFailure::kNotSquare)
        << cell::FailureName(why.reason);
    if (seed == 1) {
      std::printf("irregular hidden end: %s: %s\n",
                  cell::FailureName(why.reason), why.detail.c_str());
    }

    const std::optional<RectangleFit> fit = Fit(dots, allow, &why);
    ASSERT_TRUE(fit.has_value()) << why.detail;
    EXPECT_TRUE(fit->used_known_size);
    int placed = 0;
    for (int k = 0; k < 4; ++k) {
      if (fit->sides[k].from_known_size) {
        ++placed;
        EXPECT_EQ(k % 2, 1) << "a short side";
      }
    }
    EXPECT_EQ(placed, 1);
    ExpectWithin(ErrorsOf(fit->rectangle, scene), scene.grid);

    // A straight edge across the end, 0.3 m in.
    Scene cut = scene;
    cut.hidden = [=](double s, double) { return s > l - 0.3; };
    const Points2d short_dots = MakeDots(cut, rng);
    EXPECT_FALSE(Fit(short_dots, {}, &why).has_value());
    EXPECT_EQ(why.reason, RectangleFitFailure::kSizeMismatch);
    EXPECT_FALSE(Fit(short_dots, allow, &why).has_value());
    EXPECT_EQ(why.reason, RectangleFitFailure::kSizeMismatch);
  }
}

// A module on the coarse grid with Typical() damage, fitted with square
// corners and with each side's own line.
struct OwnLinesScene {
  Scene scene;
  std::optional<RectangleFit> square;
  std::optional<RectangleFit> own;
};

OwnLinesScene MakeOwnLinesScene(int seed) {
  std::mt19937 rng(700 + seed);
  OwnLinesScene r;
  r.scene = RandomScene(rng, 12 * kMillimetre, 18 * kMillimetre);
  r.scene.damage = Typical();
  const Points2d dots = MakeDots(r.scene, rng);
  RectangleFitOptions own;
  own.refine.square_corners = false;
  r.square = Fit(dots);
  r.own = Fit(dots, own);
  return r;
}

// The farthest the corners where the sides' own lines meet are from the
// square fit's corners. The two fits can come back in opposite half turns
// for a long side close to the y axis (see OrientedRectangle), and then
// corner k of one is corner k + 2 of the other.
double FarthestCorner(const RectangleFit& own, const RectangleFit& square) {
  const int shift =
      own.rectangle.axis.dot(square.rectangle.axis) < 0.0 ? 2 : 0;
  double farthest = 0.0;
  for (int k = 0; k < 4; ++k) {
    farthest = std::max(
        farthest, (own.corners[k] - square.corners[(k + shift) % 4]).norm());
  }
  return farthest;
}

TEST(FitRectangleTest, SidesOwnLinesMeetNearTheSquareCorners) {
  for (int seed = 1; seed <= 3; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const OwnLinesScene r = MakeOwnLinesScene(seed);
    ASSERT_TRUE(r.square.has_value());
    ASSERT_TRUE(r.own.has_value());
    ExpectFinite(*r.own);
    ExpectWithin(ErrorsOf(r.own->rectangle, r.scene), r.scene.grid);
  }
}

// A side's own direction is off the shared one by a few hundredths of a
// degree typically, which over half a side is a few millimetres: over 10000
// scenes a median of 3.8 mm and at worst 12.9 mm (seed 8437), 0.72 of the
// larger spacing. The bound, 0.9 of it, is about 1.25 times that worst. A
// fifth of the scenes exceed the 0.3 of a spacing once claimed here.
TEST(FitRectangleTest, SidesOwnLinesMeetNearTheSquareCornersOverManyScenes) {
  Spread farthest;
  for (int seed = 1; seed <= kManyScenes; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const OwnLinesScene r = MakeOwnLinesScene(seed);
    ASSERT_TRUE(r.square.has_value());
    ASSERT_TRUE(r.own.has_value());
    const double d = FarthestCorner(*r.own, *r.square);
    EXPECT_LT(d, 0.9 * r.scene.grid.Larger());
    farthest.Add(d, seed);
  }
  farthest.Print("own lines: farthest corner from the square fit's",
                 kMillimetre, "mm");
}

TEST(RefineRectangleTest, ConvergesFromAPoorGuess) {
  std::mt19937 rng(800);
  Scene scene = RandomScene(rng, 6 * kMillimetre, 9 * kMillimetre);
  scene.damage = Typical();
  scene.damage.strays = 0;
  scene.damage.patches = 0;
  const Points2d dots = MakeDots(scene, rng);
  // 3 degrees, 40 mm and 5% off, given width first.
  OrientedRectangle guess;
  guess.centre = scene.centre + Vector2d(0.03, -0.0265);
  guess.axis = Perp(Vector2d(std::cos(scene.angle + 3 * kDegree),
                             std::sin(scene.angle + 3 * kDegree)));
  guess.length = 1.05 * kWidth;
  guess.width = 1.05 * kLength;
  RectangleRefusal why;
  const std::optional<RectangleFit> fit =
      RefineRectangle(dots, guess, RefineOptions(), &why);
  ASSERT_TRUE(fit.has_value()) << why.detail;
  ExpectWithin(ErrorsOf(fit->rectangle, scene), scene.grid);
  EXPECT_NEAR(fit->spacing, 6 * kMillimetre, 0.5 * kMillimetre);
  EXPECT_EQ(fit->first_guess.length, 1.05 * kLength);
}

TEST(FitRectangleTest, SameInputGivesTheSameAnswer) {
  std::mt19937 rng(900);
  Scene scene = RandomScene(rng, 6 * kMillimetre, 9 * kMillimetre);
  scene.damage = Typical();
  const Points2d dots = MakeDots(scene, rng);
  const std::optional<RectangleFit> a = Fit(dots);
  const std::optional<RectangleFit> b = Fit(dots);
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_EQ(a->rectangle.centre, b->rectangle.centre);
  EXPECT_EQ(a->rectangle.axis, b->rectangle.axis);
  EXPECT_EQ(a->rectangle.length, b->rectangle.length);
  for (int k = 0; k < 4; ++k) EXPECT_EQ(a->corners[k], b->corners[k]);
}

TEST(FitRectangleTest, BadInputIsRefusedWithAReason) {
  std::mt19937 rng(1000);
  const Scene scene = RandomScene(rng, 12 * kMillimetre, 18 * kMillimetre);
  const Points2d good = MakeDots(scene, rng);
  ASSERT_TRUE(Fit(good).has_value());
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  auto refused = [](const Points2d& dots, RectangleFitFailure reason,
                    RectangleFitOptions options = {},
                    double length = kLength, double width = kWidth) {
    RectangleRefusal why;
    const std::optional<RectangleFit> fit =
        FitRectangle(dots, length, width, options, &why);
    EXPECT_FALSE(fit.has_value());
    EXPECT_EQ(why.reason, reason) << cell::FailureName(why.reason) << ": "
                                  << why.detail;
    EXPECT_FALSE(why.detail.empty());
  };

  refused({}, RectangleFitFailure::kTooFewPoints);
  refused(Points2d(good.begin(), good.begin() + 100),
          RectangleFitFailure::kTooFewPoints);

  Points2d with_nan = good;
  with_nan[17] = Vector2d(nan, 0.0);
  refused(with_nan, RectangleFitFailure::kNonFinite);
  Points2d with_inf = good;
  with_inf.back() = Vector2d(0.0, -inf);
  refused(with_inf, RectangleFitFailure::kNonFinite);

  // All in a line, and all one point.
  Points2d line;
  for (int i = 0; i < 2000; ++i) line.emplace_back(0.3 + 0.001 * i, 0.002 * i);
  refused(line, RectangleFitFailure::kDegenerate);
  refused(Points2d(2000, Vector2d(1.0, 2.0)), RectangleFitFailure::kDegenerate);
  // Dots scattered at random over a disc: no straight edges.
  Points2d scatter;
  while (scatter.size() < 3000) {
    const double x = UniformReal(rng, -1.0, 1.0);
    const double y = UniformReal(rng, -1.0, 1.0);
    const Vector2d p(x, y);
    if (p.norm() < 1.0) scatter.push_back(p);
  }
  {
    RectangleRefusal why;
    EXPECT_FALSE(Fit(scatter, {}, &why).has_value());
    std::printf("random scatter: %s: %s\n", cell::FailureName(why.reason),
                why.detail.c_str());
  }

  RectangleFitOptions options;
  refused(good, RectangleFitFailure::kBadOptions, options, 0.0, kWidth);
  refused(good, RectangleFitFailure::kBadOptions, options, kLength, nan);
  options.size_tolerance = 0.0;
  refused(good, RectangleFitFailure::kBadOptions, options);
  options = RectangleFitOptions();
  options.refine.iterations = 0;
  refused(good, RectangleFitFailure::kBadOptions, options);
  options = RectangleFitOptions();
  options.refine.spacing = -1.0;
  refused(good, RectangleFitFailure::kBadOptions, options);

  // A first guess with no area or no direction.
  OrientedRectangle guess;
  guess.centre = scene.centre;
  guess.length = kLength;
  guess.width = 0.0;
  RectangleRefusal why;
  EXPECT_FALSE(RefineRectangle(good, guess, {}, &why).has_value());
  EXPECT_EQ(why.reason, RectangleFitFailure::kBadOptions);
  guess.width = kWidth;
  guess.axis = Vector2d(nan, 1.0);
  EXPECT_FALSE(RefineRectangle(good, guess, {}, &why).has_value());
  EXPECT_EQ(why.reason, RectangleFitFailure::kBadOptions);
  guess.axis = Vector2d::UnitX();
  EXPECT_FALSE(RefineRectangle(with_nan, guess, {}, &why).has_value());
  EXPECT_EQ(why.reason, RectangleFitFailure::kNonFinite);
  EXPECT_FALSE(RefineRectangle({}, guess, {}, &why).has_value());
  EXPECT_EQ(why.reason, RectangleFitFailure::kTooFewPoints);
}

}  // namespace
