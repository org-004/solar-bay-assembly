// Checks joint moves: every joint stays within its own limits, all joints
// finish together exactly on the goal, and the slowest joint sets the pace.
//
//   bazel test //:joint_move_test

#include <cmath>
#include <optional>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "src/joint_move.h"

namespace {

using cell::JointLimits;
using cell::JointMove;
using cell::JointSample;
using cell::ProfileShape;
using Eigen::VectorXd;

constexpr double kSlack = 1e-9;

// Six joints with different limits, as on a real arm: the big joints near
// the base are slower than the wrist. Radians and seconds.
JointLimits SixJointLimits() {
  JointLimits limits;
  limits.velocity = VectorXd{{2.0, 2.0, 2.5, 3.0, 3.0, 4.0}};
  limits.acceleration = VectorXd{{5.0, 5.0, 8.0, 12.0, 12.0, 15.0}};
  limits.jerk = VectorXd{{50.0, 50.0, 80.0, 120.0, 120.0, 150.0}};
  return limits;
}

// For each joint, the largest magnitude seen over the move divided by that
// joint's limit. 1 means the joint reached its limit.
struct PeakRatios {
  VectorXd velocity;
  VectorXd acceleration;
  VectorXd jerk;
};

PeakRatios Sweep(const JointMove& move, const JointLimits& limits,
                 bool with_jerk) {
  const Eigen::Index n = limits.velocity.size();
  PeakRatios peaks{VectorXd::Zero(n), VectorXd::Zero(n), VectorXd::Zero(n)};

  const int steps = 2000;
  for (int i = 0; i <= steps; ++i) {
    const JointSample s = move.At(move.duration() * i / steps);
    peaks.velocity = peaks.velocity.cwiseMax(
        s.velocity.cwiseAbs().cwiseQuotient(limits.velocity));
    peaks.acceleration = peaks.acceleration.cwiseMax(
        s.acceleration.cwiseAbs().cwiseQuotient(limits.acceleration));
    if (with_jerk) {
      peaks.jerk =
          peaks.jerk.cwiseMax(s.jerk.cwiseAbs().cwiseQuotient(limits.jerk));
    }
  }
  return peaks;
}

class JointMoveTest : public ::testing::TestWithParam<ProfileShape> {
 protected:
  bool with_jerk() const { return GetParam() == ProfileShape::kSCurve; }
};

TEST_P(JointMoveTest, EveryJointStaysWithinItsLimits) {
  const JointLimits limits = SixJointLimits();
  const VectorXd start{{0.0, -1.5, 1.5, 0.0, 0.5, 0.0}};
  const VectorXd goal{{3.0, -0.5, 0.2, -2.0, 0.5, 6.0}};

  const std::optional<JointMove> move =
      JointMove::Plan(start, goal, limits, GetParam());
  ASSERT_TRUE(move.has_value());
  ASSERT_GT(move->duration(), 0.0);

  const PeakRatios peaks = Sweep(*move, limits, with_jerk());
  EXPECT_LE(peaks.velocity.maxCoeff(), 1.0 + kSlack);
  EXPECT_LE(peaks.acceleration.maxCoeff(), 1.0 + kSlack);
  EXPECT_LE(peaks.jerk.maxCoeff(), 1.0 + kSlack);

  // The move is as fast as allowed: some joint reaches its velocity limit
  // and some joint its acceleration limit. This move is long enough to
  // reach both.
  EXPECT_NEAR(peaks.velocity.maxCoeff(), 1.0, 1e-6);
  EXPECT_NEAR(peaks.acceleration.maxCoeff(), 1.0, 1e-6);
  if (with_jerk()) {
    EXPECT_NEAR(peaks.jerk.maxCoeff(), 1.0, 1e-6);
  }
}

TEST_P(JointMoveTest, StartsAtStartAndEndsExactlyOnGoalAtRest) {
  const JointLimits limits = SixJointLimits();
  const VectorXd start{{0.1, -1.5, 1.5, 0.3, 0.5, -0.7}};
  const VectorXd goal{{3.0, -0.5, 0.2, -2.0, 0.5, 6.0}};

  const std::optional<JointMove> move =
      JointMove::Plan(start, goal, limits, GetParam());
  ASSERT_TRUE(move.has_value());

  for (double t : {-1.0, 0.0}) {
    const JointSample s = move->At(t);
    EXPECT_EQ(s.position, start);
    EXPECT_TRUE(s.velocity.isZero(0.0));
  }
  for (double t : {move->duration(), move->duration() + 1.0}) {
    const JointSample s = move->At(t);
    EXPECT_EQ(s.position, goal);
    EXPECT_TRUE(s.velocity.isZero(0.0));
    EXPECT_TRUE(s.acceleration.isZero(0.0));
    EXPECT_TRUE(s.jerk.isZero(0.0));
  }

  // The motion itself arrives: the instant before the end is on the goal.
  const JointSample last = move->At(std::nextafter(move->duration(), 0.0));
  EXPECT_LT((last.position - goal).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT(last.velocity.cwiseAbs().maxCoeff(), 1e-9);
}

TEST_P(JointMoveTest, JointsMoveInStep) {
  // Every joint has done the same fraction of its travel at every instant,
  // which is what makes them finish together.
  const JointLimits limits = SixJointLimits();
  const VectorXd start{{0.0, -1.5, 1.5, 0.0, 0.5, 0.0}};
  const VectorXd goal{{3.0, -0.5, 0.2, -2.0, 1.0, 6.0}};

  const std::optional<JointMove> move =
      JointMove::Plan(start, goal, limits, GetParam());
  ASSERT_TRUE(move.has_value());

  for (int i = 1; i < 20; ++i) {
    const JointSample s = move->At(move->duration() * i / 20);
    const VectorXd fraction =
        (s.position - start).cwiseQuotient(goal - start);
    EXPECT_GT(fraction[0], 0.0);
    EXPECT_LT(fraction[0], 1.0);
    for (Eigen::Index joint = 1; joint < fraction.size(); ++joint) {
      EXPECT_NEAR(fraction[joint], fraction[0], 1e-12) << "joint " << joint;
    }
  }
}

TEST_P(JointMoveTest, SlowestJointSetsThePace) {
  // Moving joint 0 alone takes some time. Adding a small move of a faster
  // joint must not change it. Adding a huge move of another joint must make
  // it longer.
  const JointLimits limits = SixJointLimits();
  const VectorXd start = VectorXd::Zero(6);

  VectorXd goal = VectorXd::Zero(6);
  goal[0] = 2.0;
  const std::optional<JointMove> alone =
      JointMove::Plan(start, goal, limits, GetParam());
  ASSERT_TRUE(alone.has_value());

  goal[5] = 0.1;
  const std::optional<JointMove> with_small =
      JointMove::Plan(start, goal, limits, GetParam());
  ASSERT_TRUE(with_small.has_value());
  EXPECT_DOUBLE_EQ(with_small->duration(), alone->duration());

  goal[5] = 60.0;
  const std::optional<JointMove> with_large =
      JointMove::Plan(start, goal, limits, GetParam());
  ASSERT_TRUE(with_large.has_value());
  EXPECT_GT(with_large->duration(), alone->duration());
}

TEST_P(JointMoveTest, JointsThatDoNotMoveStayPut) {
  const JointLimits limits = SixJointLimits();
  const VectorXd start{{0.3, -1.5, 1.5, 0.0, 0.5, 0.0}};
  VectorXd goal = start;
  goal[2] = -1.0;

  const std::optional<JointMove> move =
      JointMove::Plan(start, goal, limits, GetParam());
  ASSERT_TRUE(move.has_value());

  for (int i = 0; i <= 20; ++i) {
    const JointSample s = move->At(move->duration() * i / 20);
    for (Eigen::Index joint : {0, 1, 3, 4, 5}) {
      EXPECT_EQ(s.position[joint], start[joint]) << "joint " << joint;
      EXPECT_EQ(s.velocity[joint], 0.0) << "joint " << joint;
    }
  }
}

TEST_P(JointMoveTest, NoMoveTakesNoTime) {
  const JointLimits limits = SixJointLimits();
  const VectorXd start{{0.3, -1.5, 1.5, 0.0, 0.5, 0.0}};

  const std::optional<JointMove> move =
      JointMove::Plan(start, start, limits, GetParam());
  ASSERT_TRUE(move.has_value());
  EXPECT_EQ(move->duration(), 0.0);
  EXPECT_EQ(move->At(0.0).position, start);
  EXPECT_TRUE(move->At(0.0).velocity.isZero(0.0));
}

TEST_P(JointMoveTest, BadInputGivesNothing) {
  const JointLimits limits = SixJointLimits();
  const VectorXd six = VectorXd::Zero(6);
  const VectorXd five = VectorXd::Zero(5);
  const VectorXd ones = VectorXd::Ones(6);

  // Lengths that do not match.
  EXPECT_FALSE(JointMove::Plan(six, five, limits, GetParam()));
  EXPECT_FALSE(JointMove::Plan(five, five, limits, GetParam()));

  // A value that is not a number.
  VectorXd broken = ones;
  broken[3] = std::nan("");
  EXPECT_FALSE(JointMove::Plan(six, broken, limits, GetParam()));

  // A limit of zero.
  JointLimits zero_velocity = limits;
  zero_velocity.velocity[2] = 0.0;
  EXPECT_FALSE(JointMove::Plan(six, ones, zero_velocity, GetParam()));

  JointLimits negative_acceleration = limits;
  negative_acceleration.acceleration[4] = -1.0;
  EXPECT_FALSE(JointMove::Plan(six, ones, negative_acceleration, GetParam()));
}

INSTANTIATE_TEST_SUITE_P(
    Shapes, JointMoveTest,
    ::testing::Values(ProfileShape::kTrapezoid, ProfileShape::kSCurve),
    [](const ::testing::TestParamInfo<ProfileShape>& info) {
      return info.param == ProfileShape::kTrapezoid ? "Trapezoid" : "SCurve";
    });

TEST(JointMoveShapeTest, TrapezoidDoesNotNeedJerkLimits) {
  JointLimits limits = SixJointLimits();
  limits.jerk.resize(0);
  const VectorXd start = VectorXd::Zero(6);
  const VectorXd goal = VectorXd::Ones(6);

  EXPECT_TRUE(JointMove::Plan(start, goal, limits, ProfileShape::kTrapezoid));
  EXPECT_FALSE(JointMove::Plan(start, goal, limits, ProfileShape::kSCurve));
}

TEST(JointMoveShapeTest, SCurveTakesLongerThanTrapezoid) {
  const JointLimits limits = SixJointLimits();
  const VectorXd start = VectorXd::Zero(6);
  const VectorXd goal{{3.0, -0.5, 0.2, -2.0, 0.5, 6.0}};

  const std::optional<JointMove> trapezoid =
      JointMove::Plan(start, goal, limits, ProfileShape::kTrapezoid);
  const std::optional<JointMove> s_curve =
      JointMove::Plan(start, goal, limits, ProfileShape::kSCurve);
  ASSERT_TRUE(trapezoid.has_value());
  ASSERT_TRUE(s_curve.has_value());
  EXPECT_GT(s_curve->duration(), trapezoid->duration());
}

}  // namespace
