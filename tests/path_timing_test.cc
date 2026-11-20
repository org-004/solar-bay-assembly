// Checks timing along a path: every trajectory starts at rest on the first
// waypoint and ends at rest exactly on the last, keeps every joint within
// its velocity, acceleration and jerk limits, has no jump in velocity or
// acceleration, strays no further from the waypoints' polyline than
// allowed, and is never slower than stopping at every waypoint. Also the
// awkward inputs, and how the time compares with stopping at every waypoint
// and with lower bounds.
//
//   bazel test //:path_timing_test
//
// The measured numbers are printed; to see them:
//   bazel test //:path_timing_test --test_output=all

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "src/joint_move.h"
#include "src/path_timing.h"
#include "src/profile.h"

namespace {

using cell::JointLimits;
using cell::JointMove;
using cell::JointSample;
using cell::PathTimingOptions;
using cell::ProfileShape;
using cell::Trajectory;
using Eigen::VectorXd;

constexpr int kJoints = 6;
constexpr double kPi = 3.141592653589793;
constexpr double kDegree = kPi / 180.0;

// Room for rounding: a limit may be exceeded by one part in a billion.
constexpr double kSlack = 1e-9;

double Bound(double limit) { return limit * (1.0 + kSlack) + 1e-12; }

// The KR 210 R2700-2's limits, as in src/arm.cc: KUKA's velocities and
// accelerations, and jerk ten times the acceleration.
JointLimits ArmLimits() {
  JointLimits limits;
  limits.velocity = VectorXd{{2.0384, 1.9451, 1.95477, 3.12414, 3.00197,
                              3.83972}};
  limits.acceleration = VectorXd{{1.98271, 1.26288, 2.56508, 7.03857,
                                  1.76397, 1.87268}};
  limits.jerk = 10.0 * limits.acceleration;
  return limits;
}

JointLimits Scaled(JointLimits limits, double speed) {
  limits.velocity *= speed;
  limits.acceleration *= speed * speed;
  limits.jerk *= speed * speed * speed;
  return limits;
}

Trajectory MakeTrajectory(const std::vector<VectorXd>& waypoints,
                          const PathTimingOptions& options = {},
                          const JointLimits& limits = ArmLimits()) {
  std::optional<Trajectory> trajectory =
      Trajectory::Plan(waypoints, limits, options);
  EXPECT_TRUE(trajectory.has_value());
  return *trajectory;
}

// Distance from q to the polyline through `points`.
double DistanceToPolyline(const VectorXd& q,
                          const std::vector<VectorXd>& points) {
  double closest = (q - points.front()).norm();
  for (size_t k = 0; k + 1 < points.size(); ++k) {
    const VectorXd ab = points[k + 1] - points[k];
    const double squared = ab.squaredNorm();
    const double x =
        squared > 0.0 ? std::clamp((q - points[k]).dot(ab) / squared, 0.0, 1.0)
                      : 0.0;
    closest = std::min(closest, (q - (points[k] + x * ab)).norm());
  }
  return closest;
}

// Stopping at every waypoint: one S-curve JointMove per leg, one after
// another, as ArmFollower::MoveThrough does.
double StopAtEveryWaypoint(const std::vector<VectorXd>& waypoints,
                           const JointLimits& limits) {
  double total = 0.0;
  for (size_t k = 0; k + 1 < waypoints.size(); ++k) {
    total += JointMove::Plan(waypoints[k], waypoints[k + 1], limits,
                             ProfileShape::kSCurve)
                 ->duration();
  }
  return total;
}

// Straight along the first leg's direction for the whole length of the
// polyline: about what the path would take if it did not bend. Not a strict
// lower bound (another direction may suit the joints' limits better), but
// close to the best possible for a path that bends only a little.
double StraightSameLength(const std::vector<VectorXd>& waypoints,
                          const JointLimits& limits) {
  double length = 0.0;
  for (size_t k = 0; k + 1 < waypoints.size(); ++k) {
    length += (waypoints[k + 1] - waypoints[k]).norm();
  }
  const VectorXd direction =
      (waypoints[1] - waypoints[0]).normalized() * length;
  return JointMove::Plan(waypoints[0], waypoints[0] + direction, limits,
                         ProfileShape::kSCurve)
      ->duration();
}

// Seconds that Plan takes, on this machine, in this build.
double PlanTime(const std::vector<VectorXd>& waypoints,
                const PathTimingOptions& options = {}) {
  const auto start = std::chrono::steady_clock::now();
  const int repeats = 5;
  for (int k = 0; k < repeats; ++k) {
    EXPECT_TRUE(Trajectory::Plan(waypoints, ArmLimits(), options));
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       start)
             .count() /
         repeats;
}

// Straight from the first waypoint to the last, ignoring the others.
double StraightThere(const std::vector<VectorXd>& waypoints,
                     const JointLimits& limits) {
  return JointMove::Plan(waypoints.front(), waypoints.back(), limits,
                         ProfileShape::kSCurve)
      ->duration();
}

// What dense sampling saw. The peaks are the largest fraction of any
// joint's limit.
struct Peaks {
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
  double deviation = 0.0;  // rad, from the polyline
  // A lower bound on the time to go along this same path: summed over the
  // samples, the time the slowest joint needs to cover its share at its
  // velocity limit. Any timing of the path takes at least this long.
  double full_speed_time = 0.0;
  // The slowest the joints move, as a fraction of their limits, at the
  // slowest point between the first and last 5% of the time.
  double slowest = std::numeric_limits<double>::infinity();
};

// Samples the trajectory finely and checks everything that must hold for
// any trajectory. `limits` are the ones it was planned with, after scaling
// by speed; `waypoints` the ones given, repeats included.
//
// As in tests/profile_test.cc, the limits are checked on the values At()
// reports and again on the changes between neighbouring samples, which do
// not trust the reported derivatives and fail on a jump. The reported
// velocity and acceleration are also checked against finite differences of
// the reported positions, so they cannot be wrong together.
Peaks CheckTrajectory(const Trajectory& trajectory,
                      const std::vector<VectorXd>& waypoints,
                      const JointLimits& limits, double max_deviation,
                      int steps = 3000) {
  Peaks peaks;
  const double duration = trajectory.duration();
  const VectorXd zero = VectorXd::Zero(kJoints);
  EXPECT_TRUE(std::isfinite(duration));
  EXPECT_GE(duration, 0.0);
  EXPECT_LE(trajectory.deviation(), max_deviation);

  // Starts at rest on the first waypoint.
  for (double t : {-1.0, 0.0}) {
    const JointSample s = trajectory.At(t);
    EXPECT_EQ(s.position, waypoints.front()) << "t=" << t;
    EXPECT_EQ(s.velocity, zero);
    EXPECT_EQ(s.acceleration, zero);
  }
  // Ends at rest exactly on the last, and stays there.
  for (double t : {duration, duration + 1.0}) {
    const JointSample s = trajectory.At(t);
    EXPECT_EQ(s.position, waypoints.back()) << "t=" << t;
    EXPECT_EQ(s.velocity, zero);
    EXPECT_EQ(s.acceleration, zero);
    EXPECT_EQ(s.jerk, zero);
  }
  // At(duration) is the last waypoint by definition; the motion itself must
  // arrive there too, at rest with no acceleration left.
  if (duration > 0.0) {
    const JointSample last = trajectory.At(std::nextafter(duration, 0.0));
    EXPECT_LT((last.position - waypoints.back()).cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT(last.velocity.cwiseAbs().maxCoeff(), 1e-9);
    EXPECT_LT(last.acceleration.cwiseAbs().maxCoeff(), 1e-8);
    const JointSample first = trajectory.At(std::nextafter(0.0, 1.0));
    EXPECT_LT((first.position - waypoints.front()).cwiseAbs().maxCoeff(),
              1e-12);
    EXPECT_LT(first.acceleration.cwiseAbs().maxCoeff(), 1e-8);
  }

  const double dt = duration / steps;
  const double h = 1e-3;  // for the finite differences
  JointSample previous = trajectory.At(0.0);
  for (int k = 0; k <= steps && duration > 0.0; ++k) {
    const double t = dt * k;
    const JointSample s = trajectory.At(t);
    const JointSample before = trajectory.At(t - h);
    const JointSample after = trajectory.At(t + h);

    double slowest_here = 0.0;
    for (int i = 0; i < kJoints; ++i) {
      const double v = limits.velocity[i], a = limits.acceleration[i],
                   j = limits.jerk[i];
      peaks.velocity = std::max(peaks.velocity, std::abs(s.velocity[i]) / v);
      peaks.acceleration =
          std::max(peaks.acceleration, std::abs(s.acceleration[i]) / a);
      peaks.jerk = std::max(peaks.jerk, std::abs(s.jerk[i]) / j);
      slowest_here = std::max(slowest_here, std::abs(s.velocity[i]) / v);

      // Reported values within limits.
      EXPECT_LE(std::abs(s.velocity[i]), Bound(v)) << "t=" << t << " i=" << i;
      EXPECT_LE(std::abs(s.acceleration[i]), Bound(a))
          << "t=" << t << " i=" << i;
      EXPECT_LE(std::abs(s.jerk[i]), Bound(j)) << "t=" << t << " i=" << i;

      // Reported velocity and acceleration agree with the positions. With
      // jerk at most j, the central difference of positions is within
      // j h^2 / 6 of the velocity, that of velocities within j h / 2 of
      // the acceleration, and the second difference of positions within
      // j h / 3 of it; the rest is rounding.
      const double dq = (after.position[i] - before.position[i]) / (2 * h);
      EXPECT_NEAR(dq, s.velocity[i], j * h * h / 6 + 1e-9)
          << "t=" << t << " i=" << i;
      const double dv = (after.velocity[i] - before.velocity[i]) / (2 * h);
      EXPECT_NEAR(dv, s.acceleration[i], j * h / 2 + 1e-9)
          << "t=" << t << " i=" << i;
      const double ddq = (after.position[i] - 2 * s.position[i] +
                          before.position[i]) /
                         (h * h);
      EXPECT_NEAR(ddq, s.acceleration[i], j * h / 3 + 1e-7)
          << "t=" << t << " i=" << i;

      // Change since the previous sample within limits: no jump in
      // position, velocity or acceleration.
      if (k > 0) {
        EXPECT_LE(std::abs(s.position[i] - previous.position[i]),
                  Bound(v * dt))
            << "t=" << t << " i=" << i;
        EXPECT_LE(std::abs(s.velocity[i] - previous.velocity[i]),
                  Bound(a * dt))
            << "t=" << t << " i=" << i;
        EXPECT_LE(std::abs(s.acceleration[i] - previous.acceleration[i]),
                  Bound(j * dt))
            << "t=" << t << " i=" << i;
      }
    }
    if (k > 0) {
      double needed = 0.0;
      for (int i = 0; i < kJoints; ++i) {
        needed = std::max(needed, std::abs(s.position[i] -
                                           previous.position[i]) /
                                      limits.velocity[i]);
      }
      peaks.full_speed_time += needed;
    }
    if (k > steps / 20 && k < steps - steps / 20) {
      peaks.slowest = std::min(peaks.slowest, slowest_here);
    }

    // Within the allowed distance of the polyline. 2e-9 for repeats that
    // were dropped (each moves the path by up to 1e-9).
    const double off = DistanceToPolyline(s.position, waypoints);
    peaks.deviation = std::max(peaks.deviation, off);
    EXPECT_LE(off, max_deviation + 2e-9) << "t=" << t;
    previous = s;
  }
  return peaks;
}

// A unit vector in the plane of joints 1 and 2, at `angle` from joint 1.
VectorXd InPlane(double angle) {
  VectorXd u = VectorXd::Zero(kJoints);
  u[0] = std::cos(angle);
  u[1] = std::sin(angle);
  return u;
}

const VectorXd kHome{{0, -kPi / 2, kPi / 2, 0, kPi / 2, 0}};

// --------------------------------------------------------- simple cases

TEST(PathTimingTest, OneWaypointTakesNoTime) {
  const Trajectory trajectory = MakeTrajectory({kHome});
  EXPECT_EQ(trajectory.duration(), 0.0);
  for (double t : {-1.0, 0.0, 1.0}) {
    EXPECT_EQ(trajectory.At(t).position, kHome);
    EXPECT_EQ(trajectory.At(t).velocity, VectorXd::Zero(kJoints));
  }
  // The same point many times over is one point.
  EXPECT_EQ(MakeTrajectory({kHome, kHome, kHome}).duration(), 0.0);
}

TEST(PathTimingTest, TwoWaypointsAreTheSCurveJointMove) {
  // With nothing to blend, the trajectory is the synchronized S-curve
  // JointMove: same time, same positions.
  const JointLimits limits = ArmLimits();
  const std::vector<VectorXd> goals = {
      kHome + VectorXd{{1.0, 0.6, -1.0, 1.0, -1.0, 1.0}},
      kHome + VectorXd{{0.001, 0, 0, 0, 0, 0}},
      kHome + VectorXd{{-0.02, 0.01, 0.03, 0, 0, -0.05}},
      kHome + VectorXd{{3.0, 0, 0, 0, 0, 0}}};
  for (const VectorXd& goal : goals) {
    const Trajectory trajectory = MakeTrajectory({kHome, goal});
    const JointMove move =
        *JointMove::Plan(kHome, goal, limits, ProfileShape::kSCurve);
    EXPECT_NEAR(trajectory.duration(), move.duration(),
                1e-9 * move.duration());
    for (int k = 0; k <= 20; ++k) {
      const double t = move.duration() * k / 20;
      EXPECT_LT((trajectory.At(t).position - move.At(t).position)
                    .cwiseAbs()
                    .maxCoeff(),
                1e-9);
      EXPECT_LT((trajectory.At(t).velocity - move.At(t).velocity)
                    .cwiseAbs()
                    .maxCoeff(),
                1e-8);
    }
    CheckTrajectory(trajectory, {kHome, goal}, limits,
                    PathTimingOptions::kDefaultMaxDeviation);
    EXPECT_EQ(trajectory.stretches(), 1);
  }
}

TEST(PathTimingTest, RepeatedWaypointsAreDropped) {
  const VectorXd a = kHome;
  const VectorXd b = a + 0.5 * InPlane(0.0);
  const VectorXd c = b + 0.5 * InPlane(kPi / 2);
  const Trajectory plain = MakeTrajectory({a, b, c});
  const std::vector<VectorXd> repeated = {a, a, b, b, b, c, c};
  const Trajectory trajectory = MakeTrajectory(repeated);
  EXPECT_EQ(trajectory.waypoints().size(), 3u);
  EXPECT_NEAR(trajectory.duration(), plain.duration(), 1e-12);
  CheckTrajectory(trajectory, repeated, ArmLimits(),
                  PathTimingOptions::kDefaultMaxDeviation);

  // Nearly repeated: within 1e-9 of the one before. The last waypoint is
  // still where the trajectory ends, exactly.
  VectorXd c_again = c;
  c_again[3] += 5e-10;
  VectorXd b_again = b;
  b_again[2] -= 5e-10;
  const std::vector<VectorXd> nearly = {a, b, b_again, c, c_again};
  const Trajectory near = MakeTrajectory(nearly);
  EXPECT_EQ(near.waypoints().size(), 3u);
  EXPECT_EQ(near.At(near.duration()).position, c_again);
  CheckTrajectory(near, nearly, ArmLimits(),
                  PathTimingOptions::kDefaultMaxDeviation);
}

TEST(PathTimingTest, SmallOutAndBackWithinTheDeviation) {
  // Out by less than the deviation allows and back to the start: whatever
  // is made of it, it ends exactly where it began and stays within bounds.
  const VectorXd a = kHome;
  const VectorXd b = a + VectorXd{{4e-4, 0, 0, 0, 0, 0}};
  const VectorXd c = a + VectorXd{{4e-4, 3e-4, 0, 0, 0, 0}};
  for (const std::vector<VectorXd>& waypoints :
       {std::vector<VectorXd>{a, b, a}, std::vector<VectorXd>{a, b, c, a}}) {
    const Trajectory trajectory = MakeTrajectory(waypoints);
    CheckTrajectory(trajectory, waypoints, ArmLimits(),
                    PathTimingOptions::kDefaultMaxDeviation);
    EXPECT_EQ(trajectory.At(trajectory.duration()).position, a);
  }
}

TEST(PathTimingTest, CollinearWaypointsNeedNoSlowing) {
  // Waypoints on one straight line, unevenly spaced: exactly the time of
  // the straight move, in one stretch.
  const VectorXd a = kHome;
  const VectorXd d{{0.8, -0.3, 0.5, 1.2, -0.4, 0.9}};
  const std::vector<VectorXd> waypoints = {a, a + 0.2 * d, a + 0.25 * d,
                                           a + 0.9 * d, a + d};
  const Trajectory trajectory = MakeTrajectory(waypoints);
  const Trajectory straight = MakeTrajectory({a, a + d});
  EXPECT_NEAR(trajectory.duration(), straight.duration(),
              1e-9 * straight.duration());
  EXPECT_EQ(trajectory.stretches(), 1);
  const Peaks peaks = CheckTrajectory(trajectory, waypoints, ArmLimits(),
                                      PathTimingOptions::kDefaultMaxDeviation);
  EXPECT_LT(peaks.deviation, 1e-12);
  // Stopping at each would take far longer.
  const double stop = StopAtEveryWaypoint(waypoints, ArmLimits());
  std::printf("collinear, 4 legs: %.3f s; straight %.3f s; stopping at each "
              "%.3f s\n",
              trajectory.duration(), straight.duration(), stop);
  EXPECT_LT(trajectory.duration(), 0.6 * stop);
}

TEST(PathTimingTest, ReversalComesToRestAtTheTurn) {
  // Straight back the way it came: the joints must stop to turn round,
  // and the path turns round on the segment itself.
  const VectorXd a = kHome;
  const VectorXd b = a + VectorXd{{0.5, 0.2, -0.3, 0, 0.1, 0}};
  const std::vector<VectorXd> back = {a, b, a};
  const Trajectory trajectory = MakeTrajectory(back);
  const Peaks peaks = CheckTrajectory(trajectory, back, ArmLimits(),
                                      PathTimingOptions::kDefaultMaxDeviation);
  EXPECT_LT(peaks.deviation, 1e-12);
  EXPECT_LT(peaks.slowest, 0.01);
  EXPECT_LE(trajectory.duration(),
            StopAtEveryWaypoint(back, ArmLimits()) * (1 + kSlack));

  // Nearly straight back, turning by 179 degrees: still nearly at rest at
  // the turn, and within the deviation.
  const VectorXd c = a + 0.6 * InPlane(0.0);
  const VectorXd e = c + 0.6 * InPlane(kPi - 1.0 * kDegree);
  const std::vector<VectorXd> nearly = {a, c, e};
  const Trajectory sharp = MakeTrajectory(nearly);
  const Peaks sharp_peaks = CheckTrajectory(
      sharp, nearly, ArmLimits(), PathTimingOptions::kDefaultMaxDeviation);
  std::printf("reversal: %.3f s, slowest %.4f of the limit; 179 degree turn: "
              "%.3f s, slowest %.4f, %.2e rad off the polyline\n",
              trajectory.duration(), peaks.slowest, sharp.duration(),
              sharp_peaks.slowest, sharp_peaks.deviation);
  EXPECT_LT(sharp_peaks.slowest, 0.05);
}

TEST(PathTimingTest, CornersUseTheDeviationAndStraightPartsAreExact) {
  // Long legs and one corner gentle enough to be worth blending: the blend
  // is as large as the deviation allows, so the path comes within a
  // whisker of it; away from the corner it is on the segments exactly.
  for (double turn : {5.0, 10.0, 20.0}) {
    SCOPED_TRACE(testing::Message() << turn << " degrees");
    const double deviation = 0.01;
    const VectorXd corner = kHome + 1.0 * InPlane(0.0);
    const std::vector<VectorXd> waypoints = {
        kHome, corner, corner + 1.0 * InPlane(turn * kDegree)};
    const Trajectory trajectory =
        MakeTrajectory(waypoints, {.max_deviation = deviation});
    const Peaks peaks =
        CheckTrajectory(trajectory, waypoints, ArmLimits(), deviation);
    EXPECT_EQ(trajectory.blends(), 1);
    EXPECT_NEAR(trajectory.deviation(), deviation, 1e-9);
    EXPECT_GT(peaks.deviation, 0.99 * deviation);

    // The blend reaches deviation / (3/16 sin(turn)) either side of the
    // corner; beyond that the path is on the polyline.
    const double reach = deviation / (3.0 / 16.0 * std::sin(turn * kDegree));
    int beyond = 0;
    for (int k = 0; k <= 2000; ++k) {
      const VectorXd q =
          trajectory.At(trajectory.duration() * k / 2000).position;
      if ((q - corner).norm() > reach * 1.001) {
        ++beyond;
        EXPECT_LT(DistanceToPolyline(q, waypoints), 1e-12);
      }
    }
    EXPECT_GT(beyond, 1000);
  }
}

TEST(PathTimingTest, SharpCornersAreTakenAsQuicklyAsStopping) {
  // A sharp corner within a tight deviation can only be taken slowly: its
  // blend is small and tight. Where stopping there is quicker, the corner
  // becomes a stop; either way the time is no worse than stopping.
  for (double turn : {45.0, 90.0, 150.0, 180.0}) {
    SCOPED_TRACE(testing::Message() << turn << " degrees");
    const VectorXd corner = kHome + 1.0 * InPlane(0.0);
    const std::vector<VectorXd> waypoints = {
        kHome, corner, corner + 1.0 * InPlane(turn * kDegree)};
    const Trajectory trajectory = MakeTrajectory(waypoints);
    const Peaks peaks = CheckTrajectory(trajectory, waypoints, ArmLimits(),
                                        PathTimingOptions::kDefaultMaxDeviation);
    EXPECT_LE(trajectory.duration(),
              StopAtEveryWaypoint(waypoints, ArmLimits()) * (1 + kSlack));
    EXPECT_LT(peaks.slowest, 0.1);
  }
}

TEST(PathTimingTest, ShortSegmentsShrinkTheBlends) {
  // A zig-zag of legs far shorter than the blends would like, turning 20
  // degrees one way and then the other: neighbouring blends share each
  // leg, everything stays within the limits and the deviation, and the
  // path never stops. (With a looser deviation the whole zig-zag is within
  // it of a straight line, and becomes one.)
  const double deviation = 0.002;
  std::vector<VectorXd> waypoints = {kHome};
  for (int k = 0; k < 12; ++k) {
    const double length = k % 3 == 0 ? 0.002 : 0.02;
    waypoints.push_back(waypoints.back() +
                        length * InPlane((k % 2 == 0 ? 10.0 : -10.0) * kDegree));
  }
  const Trajectory trajectory =
      MakeTrajectory(waypoints, {.max_deviation = deviation});
  const Peaks peaks =
      CheckTrajectory(trajectory, waypoints, ArmLimits(), deviation);
  // The pieces fit together end to end.
  double s = 0.0;
  for (const Trajectory::Piece& piece : trajectory.pieces()) {
    EXPECT_NEAR(piece.start, s, 1e-15);
    s += piece.length;
  }
  // And blends are no longer than the legs they sit on allow: each pair of
  // neighbouring blends fits in the leg between their corners.
  const double stop = StopAtEveryWaypoint(waypoints, ArmLimits());
  std::printf("zig-zag of 12 short legs: %.3f s with %d blends, stopping at "
              "each %.3f s; %.4f rad off at most, slowest %.3f of the "
              "limit\n",
              trajectory.duration(), trajectory.blends(), stop,
              peaks.deviation, peaks.slowest);
  EXPECT_GT(trajectory.blends(), 0);
  EXPECT_GT(peaks.slowest, 0.0);
  EXPECT_LT(trajectory.duration(), 0.5 * stop);
}

TEST(PathTimingTest, ZeroDeviationStopsAtEveryCornerLikeJointMoves) {
  const VectorXd a = kHome;
  const std::vector<VectorXd> waypoints = {
      a, a + VectorXd{{0.4, 0, 0, 0, 0, 0}},
      a + VectorXd{{0.4, 0.3, 0, 0.2, 0, 0}},
      a + VectorXd{{0.1, 0.3, -0.5, 0.2, 0, 0.6}}};
  const Trajectory trajectory =
      MakeTrajectory(waypoints, {.max_deviation = 0.0});
  const double stop = StopAtEveryWaypoint(waypoints, ArmLimits());
  EXPECT_NEAR(trajectory.duration(), stop, 1e-9 * stop);
  EXPECT_EQ(trajectory.blends(), 0);
  const Peaks peaks = CheckTrajectory(trajectory, waypoints, ArmLimits(), 0.0);
  EXPECT_LT(peaks.deviation, 1e-12);
}

TEST(PathTimingTest, SpeedPlaysTheSameTrajectorySlower) {
  const std::vector<VectorXd> waypoints = {
      kHome, kHome + VectorXd{{0.5, 0.1, 0, 0, 0, 0}},
      kHome + VectorXd{{0.6, 0.4, -0.2, 0, 0.3, 0}},
      kHome + VectorXd{{0.2, 0.5, -0.4, 0.5, 0.3, 0.2}}};
  const Trajectory full = MakeTrajectory(waypoints, {.max_deviation = 0.02});
  for (double speed : {0.9, 0.5, 0.1}) {
    const Trajectory slow =
        MakeTrajectory(waypoints, {.max_deviation = 0.02, .speed = speed});
    EXPECT_NEAR(slow.duration(), full.duration() / speed,
                1e-9 * slow.duration());
    for (int k = 0; k <= 50; ++k) {
      const double t = full.duration() * k / 50;
      EXPECT_LT(
          (slow.At(t / speed).position - full.At(t).position).norm(), 1e-9);
    }
    CheckTrajectory(slow, waypoints, Scaled(ArmLimits(), speed), 0.02, 1000);
  }
}

TEST(PathTimingTest, BadInputGivesNothing) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const JointLimits limits = ArmLimits();
  const VectorXd a = kHome, b = kHome + VectorXd::Constant(kJoints, 0.1);
  auto plan = [](const std::vector<VectorXd>& waypoints,
                 const JointLimits& limits,
                 const PathTimingOptions& options = {}) {
    return Trajectory::Plan(waypoints, limits, options).has_value();
  };
  EXPECT_TRUE(plan({a, b}, limits));

  EXPECT_FALSE(plan({}, limits));
  EXPECT_FALSE(plan({a, VectorXd::Zero(5)}, limits));
  EXPECT_FALSE(plan({VectorXd(), VectorXd()}, JointLimits{}));
  for (double bad : {nan, inf, -inf}) {
    VectorXd q = b;
    q[2] = bad;
    EXPECT_FALSE(plan({a, q}, limits)) << bad;
    EXPECT_FALSE(plan({q}, limits)) << bad;
  }
  // Finite but too far apart to measure.
  EXPECT_FALSE(plan({VectorXd::Constant(kJoints, -1e308),
                     VectorXd::Constant(kJoints, 1e308)},
                    limits));
  for (double bad : {0.0, -1.0, nan, inf}) {
    JointLimits l = limits;
    l.velocity[1] = bad;
    EXPECT_FALSE(plan({a, b}, l)) << bad;
    l = limits;
    l.acceleration[4] = bad;
    EXPECT_FALSE(plan({a, b}, l)) << bad;
    l = limits;
    l.jerk[0] = bad;
    EXPECT_FALSE(plan({a, b}, l)) << bad;
  }
  JointLimits no_jerk = limits;
  no_jerk.jerk.resize(0);
  EXPECT_FALSE(plan({a, b}, no_jerk));
  for (double speed : {0.0, -0.5, 1.01, nan}) {
    EXPECT_FALSE(plan({a, b}, limits, {.speed = speed})) << speed;
  }
  for (double deviation : {-1e-6, nan}) {
    EXPECT_FALSE(plan({a, b}, limits, {.max_deviation = deviation}))
        << deviation;
  }
  // No bound at all is allowed: the blends are then as large as the legs.
  EXPECT_TRUE(plan({a, b, b + 0.2 * InPlane(1.0)}, limits,
                   {.max_deviation = inf}));
}

// ------------------------------------------------- how much time is saved

TEST(PathTimingTest, TimeSavedAtOneCornerByItsAngle) {
  // Two 0.5 rad legs of joints 1 and 2 meeting at a corner, at the default
  // deviation and at a looser one. A gentle corner costs almost nothing; a
  // sharp one must slow nearly to rest, and then saves little.
  std::printf("one corner between two 0.5 rad legs (stopping there: %.3f s)\n",
              StopAtEveryWaypoint({kHome, kHome + 0.5 * InPlane(0.0),
                                   kHome + 0.5 * InPlane(0.0) +
                                       0.5 * InPlane(kPi / 2)},
                                  ArmLimits()));
  std::printf("  turn, deg   time at 0.002 rad   at 0.02 rad   stop at it   "
              "straight, same length\n");
  for (double turn : {2.0, 10.0, 30.0, 60.0, 90.0, 135.0, 170.0, 180.0}) {
    const VectorXd corner = kHome + 0.5 * InPlane(0.0);
    const std::vector<VectorXd> waypoints = {
        kHome, corner, corner + 0.5 * InPlane(turn * kDegree)};
    const Trajectory tight = MakeTrajectory(waypoints);
    const Trajectory loose =
        MakeTrajectory(waypoints, {.max_deviation = 0.02});
    const double stop = StopAtEveryWaypoint(waypoints, ArmLimits());
    std::printf("  %9.0f   %17.3f   %11.3f   %10.3f   %21.3f\n", turn,
                tight.duration(), loose.duration(), stop,
                StraightSameLength(waypoints, ArmLimits()));
    EXPECT_LE(tight.duration(), stop * (1 + kSlack));
    EXPECT_LE(loose.duration(), tight.duration() * (1 + kSlack));
    CheckTrajectory(tight, waypoints, ArmLimits(),
                    PathTimingOptions::kDefaultMaxDeviation, 1000);
    CheckTrajectory(loose, waypoints, ArmLimits(), 0.02, 1000);
  }
}

TEST(PathTimingTest, FinelySampledCurveRunsAsOneMotion) {
  // Like a straight tool-tip line solved every centimetre: 31 waypoints,
  // about 0.02 rad apart, on a gently bending curve. Stopping at each costs
  // a rest-to-rest move per waypoint; one trajectory through them all does
  // not.
  std::vector<VectorXd> waypoints;
  for (int k = 0; k <= 30; ++k) {
    const double x = k / 30.0;
    waypoints.push_back(
        kHome + VectorXd{{0.2 * x, 0.35 * x + 0.05 * x * x,
                          -0.4 * x + 0.1 * x * x, 0.05 * std::sin(2 * x),
                          0.1 * x * x, -0.15 * x}});
  }
  const Trajectory trajectory = MakeTrajectory(waypoints);
  const Peaks peaks = CheckTrajectory(trajectory, waypoints, ArmLimits(),
                                      PathTimingOptions::kDefaultMaxDeviation);
  const double stop = StopAtEveryWaypoint(waypoints, ArmLimits());
  const double straight = StraightThere(waypoints, ArmLimits());
  std::printf(
      "31 waypoints on a gentle curve: %.3f s in %d stretch(es); stopping at "
      "each %.3f s; straight from first to last %.3f s, straight for the same "
      "length %.3f s; at the velocity limit along the same path (a lower "
      "bound) %.3f s; %.1e rad off the polyline; planned in %.1f ms\n",
      trajectory.duration(), trajectory.stretches(), stop, straight,
      StraightSameLength(waypoints, ArmLimits()), peaks.full_speed_time,
      peaks.deviation, 1e3 * PlanTime(waypoints));
  EXPECT_LT(trajectory.duration(), 0.2 * stop);
  EXPECT_LT(trajectory.duration(), 1.5 * straight);
}

// Polylines with random corners in six joints, legs from a few milliradians
// to a radian, some repeats and some collinear points, at several
// deviations and speeds. The fixed seed makes a failure repeatable.
TEST(PathTimingTest, RandomPolylinesKeepEveryGuarantee) {
  std::mt19937 generator(20261119);
  auto unit = [&generator]() { return generator() / 4294967296.0; };
  // Spread evenly by order of magnitude.
  auto draw = [&unit](double low, double high) {
    return std::exp(std::log(low) + unit() * (std::log(high) - std::log(low)));
  };
  std::normal_distribution<double> normal;

  const double deviations[] = {0.0, 1e-4, 0.002, 0.01, 0.05};
  const double speeds[] = {1.0, 0.9, 0.5};
  double worst_vs_stop = 0.0, sum_vs_stop = 0.0;
  double worst_vs_bound = 0.0, sum_vs_bound = 0.0;
  double worst_vs_straight = 0.0;
  double worst_velocity = 0.0, worst_acceleration = 0.0, worst_jerk = 0.0;
  double worst_plan = 0.0, sum_plan = 0.0;
  const int trials = 40;
  for (int trial = 0; trial < trials; ++trial) {
    SCOPED_TRACE(testing::Message() << "trial " << trial);
    const int count = 3 + static_cast<int>(generator() % 28);
    std::vector<VectorXd> waypoints = {kHome};
    VectorXd direction = VectorXd::Zero(kJoints);
    for (int k = 1; k < count; ++k) {
      const unsigned kind = generator() % 10;
      if (kind == 0) {  // a repeat
        waypoints.push_back(waypoints.back());
        continue;
      }
      if (kind > 1 || k == 1) {  // a new direction (else: straight on)
        for (int i = 0; i < kJoints; ++i) direction[i] = normal(generator);
        direction.normalize();
      }
      waypoints.push_back(waypoints.back() + draw(0.003, 1.0) * direction);
    }
    const double deviation = deviations[generator() % 5];
    const double speed = speeds[generator() % 3];
    const PathTimingOptions options{.max_deviation = deviation,
                                    .speed = speed};
    const JointLimits limits = Scaled(ArmLimits(), speed);
    const Trajectory trajectory = MakeTrajectory(waypoints, options);
    const double plan = PlanTime(waypoints, options);
    worst_plan = std::max(worst_plan, plan);
    sum_plan += plan;
    const Peaks peaks =
        CheckTrajectory(trajectory, waypoints, limits, deviation, 2000);
    worst_velocity = std::max(worst_velocity, peaks.velocity);
    worst_acceleration = std::max(worst_acceleration, peaks.acceleration);
    worst_jerk = std::max(worst_jerk, peaks.jerk);

    const double stop = StopAtEveryWaypoint(waypoints, limits);
    EXPECT_LE(trajectory.duration(), stop * (1 + kSlack));
    const double vs_stop = trajectory.duration() / stop;
    // The better of two lower bounds: the same path at the velocity limit,
    // and each joint moving from its first angle to its last in the least
    // time its own limits allow, ignoring the others.
    double bound = peaks.full_speed_time;
    for (int i = 0; i < kJoints; ++i) {
      const double travel = waypoints.back()[i] - waypoints.front()[i];
      const cell::Limits joint{limits.velocity[i], limits.acceleration[i],
                               limits.jerk[i]};
      bound = std::max(bound, cell::Profile::SCurve(travel, joint)->duration());
    }
    const double vs_bound = trajectory.duration() / bound;
    EXPECT_GE(vs_bound, 1.0 - 1e-6);
    worst_vs_stop = std::max(worst_vs_stop, vs_stop);
    sum_vs_stop += vs_stop;
    worst_vs_bound = std::max(worst_vs_bound, vs_bound);
    sum_vs_bound += vs_bound;
    const double straight = StraightThere(waypoints, limits);
    if (straight > 0.0) {
      worst_vs_straight =
          std::max(worst_vs_straight, trajectory.duration() / straight);
    }
  }
  std::printf(
      "%d random polylines of 3 to 30 waypoints:\n"
      "  time / stopping at every waypoint: mean %.2f, worst %.2f\n"
      "  time / lower bound: mean %.2f, worst %.2f\n"
      "  time / straight from first to last: worst %.1f\n"
      "  peak fraction of the limits: velocity %.3f, acceleration %.3f, "
      "jerk %.3f\n"
      "  planning: mean %.1f ms, worst %.1f ms\n",
      trials, sum_vs_stop / trials, worst_vs_stop, sum_vs_bound / trials,
      worst_vs_bound, worst_vs_straight, worst_velocity, worst_acceleration,
      worst_jerk, 1e3 * sum_plan / trials, 1e3 * worst_plan);
}

}  // namespace
