// Checks the velocity profiles: every one stays within its limits, ends at
// rest exactly on target, and takes the time worked out by hand for moves
// where that is easy to do.
//
//   bazel test //:profile_test

#include <algorithm>
#include <cmath>
#include <optional>
#include <random>

#include <gtest/gtest.h>

#include "src/profile.h"

namespace {

using cell::Limits;
using cell::Profile;
using cell::Sample;

// Room for rounding: a limit may be exceeded by one part in a billion.
constexpr double kSlack = 1e-9;

double Bound(double limit) { return limit * (1.0 + kSlack) + 1e-12; }

// Largest magnitudes seen while sampling a profile.
struct Peaks {
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
};

// Samples the profile finely and checks everything that must hold for any
// move. `limits.jerk` is checked only if `jerk_limited`.
//
// The limits are checked twice. Once on the values At() reports. Once on the
// change between neighbouring samples: position may not change faster than
// the velocity limit, velocity no faster than the acceleration limit, and
// acceleration no faster than the jerk limit. The second check does not
// trust the reported derivatives, and it fails on a jump.
Peaks CheckProfile(const Profile& profile, double distance,
                   const Limits& limits, bool jerk_limited) {
  Peaks peaks;
  const double duration = profile.duration();
  const double direction = distance < 0.0 ? -1.0 : 1.0;

  EXPECT_TRUE(std::isfinite(duration));
  EXPECT_GE(duration, 0.0);
  EXPECT_EQ(profile.distance(), distance);

  // Starts at rest at zero.
  const Sample first = profile.At(0.0);
  EXPECT_EQ(first.position, 0.0);
  EXPECT_EQ(first.velocity, 0.0);

  // Ends at rest on target, and stays there.
  for (double t : {duration, duration + 1.0}) {
    const Sample end = profile.At(t);
    EXPECT_EQ(end.position, distance);
    EXPECT_EQ(end.velocity, 0.0);
    EXPECT_EQ(end.acceleration, 0.0);
    EXPECT_EQ(end.jerk, 0.0);
  }

  // At(duration) returns the target by definition, so also check that the
  // motion itself arrives there: the instant before the end.
  if (duration > 0.0) {
    const Sample last = profile.At(std::nextafter(duration, 0.0));
    EXPECT_NEAR(last.position, distance, kSlack * std::abs(distance));
    EXPECT_NEAR(last.velocity, 0.0, kSlack * limits.velocity);
    if (jerk_limited) {
      EXPECT_NEAR(last.acceleration, 0.0, kSlack * limits.acceleration);
    }
  }

  const int steps = 2000;
  const double dt = duration / steps;
  Sample previous = first;
  for (int i = 0; i <= steps; ++i) {
    const double t = dt * i;
    const Sample s = profile.At(t);

    peaks.velocity = std::max(peaks.velocity, std::abs(s.velocity));
    peaks.acceleration = std::max(peaks.acceleration, std::abs(s.acceleration));
    peaks.jerk = std::max(peaks.jerk, std::abs(s.jerk));

    // Reported values within limits.
    EXPECT_LE(std::abs(s.velocity), Bound(limits.velocity)) << "t=" << t;
    EXPECT_LE(std::abs(s.acceleration), Bound(limits.acceleration))
        << "t=" << t;
    if (jerk_limited) {
      EXPECT_LE(std::abs(s.jerk), Bound(limits.jerk)) << "t=" << t;
    }

    // Never moves backwards, never overshoots.
    EXPECT_GE(s.velocity * direction, -1e-12) << "t=" << t;
    EXPECT_LE(s.position * direction, Bound(std::abs(distance))) << "t=" << t;

    // Change since the previous sample within limits.
    if (i > 0) {
      EXPECT_LE(std::abs(s.position - previous.position),
                Bound(limits.velocity * dt))
          << "t=" << t;
      EXPECT_LE(std::abs(s.velocity - previous.velocity),
                Bound(limits.acceleration * dt))
          << "t=" << t;
      if (jerk_limited) {
        EXPECT_LE(std::abs(s.acceleration - previous.acceleration),
                  Bound(limits.jerk * dt))
            << "t=" << t;
      }
    }
    previous = s;
  }
  return peaks;
}

Profile MakeTrapezoid(double distance, double velocity, double acceleration) {
  std::optional<Profile> profile =
      Profile::Trapezoid(distance, velocity, acceleration);
  EXPECT_TRUE(profile.has_value());
  return *profile;
}

Profile MakeSCurve(double distance, const Limits& limits) {
  std::optional<Profile> profile = Profile::SCurve(distance, limits);
  EXPECT_TRUE(profile.has_value());
  return *profile;
}

// ---------------------------------------------------------------- Trapezoid

TEST(TrapezoidTest, LongMoveCruisesAtTheVelocityLimit) {
  // 0.5 s to reach 1 m/s covers 0.25 m; the same to stop. The 0.5 m between
  // takes 0.5 s at 1 m/s.
  const Limits limits{.velocity = 1.0, .acceleration = 2.0};
  const Profile profile = MakeTrapezoid(1.0, 1.0, 2.0);
  EXPECT_NEAR(profile.duration(), 1.5, 1e-12);

  const Peaks peaks = CheckProfile(profile, 1.0, limits, false);
  EXPECT_NEAR(peaks.velocity, 1.0, 1e-9);
  EXPECT_NEAR(peaks.acceleration, 2.0, 1e-9);

  // Halfway through the time is halfway through the distance, at full speed.
  const Sample middle = profile.At(0.75);
  EXPECT_NEAR(middle.position, 0.5, 1e-12);
  EXPECT_NEAR(middle.velocity, 1.0, 1e-12);
  EXPECT_EQ(middle.acceleration, 0.0);
}

TEST(TrapezoidTest, ShortMoveIsATriangle) {
  // Too short to reach 10 m/s. Accelerates for half the distance:
  // 0.25 = 2 t^2 / 2, so t = 0.5 s, peaking at 1 m/s.
  const Limits limits{.velocity = 10.0, .acceleration = 2.0};
  const Profile profile = MakeTrapezoid(0.5, 10.0, 2.0);
  EXPECT_NEAR(profile.duration(), 1.0, 1e-12);

  const Peaks peaks = CheckProfile(profile, 0.5, limits, false);
  EXPECT_NEAR(peaks.velocity, 1.0, 1e-9);
}

TEST(TrapezoidTest, NegativeMoveMirrorsThePositiveOne) {
  const Limits limits{.velocity = 1.0, .acceleration = 2.0};
  const Profile forward = MakeTrapezoid(1.0, 1.0, 2.0);
  const Profile backward = MakeTrapezoid(-1.0, 1.0, 2.0);
  EXPECT_EQ(backward.duration(), forward.duration());
  CheckProfile(backward, -1.0, limits, false);

  for (double t : {0.1, 0.4, 0.75, 1.2}) {
    EXPECT_EQ(backward.At(t).position, -forward.At(t).position);
    EXPECT_EQ(backward.At(t).velocity, -forward.At(t).velocity);
    EXPECT_EQ(backward.At(t).acceleration, -forward.At(t).acceleration);
  }
}

// ------------------------------------------------------------------ S-curve
//
// The four cases differ in which limits the move is long enough to reach.
// Each is sized so the segment times are round numbers.

TEST(SCurveTest, LongMoveReachesEveryLimit) {
  // Ramp 2/8 = 0.25 s, hold 0.25 s, ramp 0.25 s: 0.75 s to reach 1 m/s,
  // covering 0.375 m. The same to stop. The 1.25 m between takes 1.25 s.
  const Limits limits{.velocity = 1.0, .acceleration = 2.0, .jerk = 8.0};
  const Profile profile = MakeSCurve(2.0, limits);
  EXPECT_NEAR(profile.duration(), 2.75, 1e-12);

  const Peaks peaks = CheckProfile(profile, 2.0, limits, true);
  EXPECT_NEAR(peaks.velocity, 1.0, 1e-9);
  EXPECT_NEAR(peaks.acceleration, 2.0, 1e-9);
  EXPECT_NEAR(peaks.jerk, 8.0, 1e-9);
}

TEST(SCurveTest, VelocityLimitReachedBeforeAccelerationLimit) {
  // Ramping up for 0.5 s and straight back down already gains
  // 4 * 0.5^2 = 1 m/s, with acceleration peaking at 2 of the allowed 10.
  // 1 s to speed up over 0.5 m, the same to stop, 2 m of cruise between.
  const Limits limits{.velocity = 1.0, .acceleration = 10.0, .jerk = 4.0};
  const Profile profile = MakeSCurve(3.0, limits);
  EXPECT_NEAR(profile.duration(), 4.0, 1e-12);

  const Peaks peaks = CheckProfile(profile, 3.0, limits, true);
  EXPECT_NEAR(peaks.velocity, 1.0, 1e-9);
  EXPECT_NEAR(peaks.acceleration, 2.0, 1e-9);
}

TEST(SCurveTest, ShortMoveReachesAccelerationLimitOnly) {
  // Ramp 0.25 s, hold 0.25 s, ramp 0.25 s reaches 1 m/s of the allowed 10
  // and covers 0.375 m. Stopping covers the other 0.375 m. No cruise.
  const Limits limits{.velocity = 10.0, .acceleration = 2.0, .jerk = 8.0};
  const Profile profile = MakeSCurve(0.75, limits);
  EXPECT_NEAR(profile.duration(), 1.5, 1e-12);

  const Peaks peaks = CheckProfile(profile, 0.75, limits, true);
  EXPECT_NEAR(peaks.velocity, 1.0, 1e-9);
  EXPECT_NEAR(peaks.acceleration, 2.0, 1e-9);
}

TEST(SCurveTest, VeryShortMoveReachesJerkLimitOnly) {
  // Four ramps of 0.5 s and nothing else: 2 * 4 * 0.5^3 = 1 m. Acceleration
  // peaks at 2 and velocity at 1, both far below their limits.
  const Limits limits{.velocity = 10.0, .acceleration = 10.0, .jerk = 4.0};
  const Profile profile = MakeSCurve(1.0, limits);
  EXPECT_NEAR(profile.duration(), 2.0, 1e-12);

  const Peaks peaks = CheckProfile(profile, 1.0, limits, true);
  EXPECT_NEAR(peaks.velocity, 1.0, 1e-9);
  EXPECT_NEAR(peaks.acceleration, 2.0, 1e-9);
  EXPECT_NEAR(peaks.jerk, 4.0, 1e-9);
}

TEST(SCurveTest, NegativeMoveMirrorsThePositiveOne) {
  const Limits limits{.velocity = 1.0, .acceleration = 2.0, .jerk = 8.0};
  const Profile forward = MakeSCurve(2.0, limits);
  const Profile backward = MakeSCurve(-2.0, limits);
  EXPECT_EQ(backward.duration(), forward.duration());
  CheckProfile(backward, -2.0, limits, true);

  for (double t : {0.1, 0.4, 1.0, 2.2, 2.7}) {
    EXPECT_EQ(backward.At(t).position, -forward.At(t).position);
    EXPECT_EQ(backward.At(t).velocity, -forward.At(t).velocity);
    EXPECT_EQ(backward.At(t).acceleration, -forward.At(t).acceleration);
    EXPECT_EQ(backward.At(t).jerk, -forward.At(t).jerk);
  }
}

TEST(SCurveTest, SecondHalfMirrorsTheFirst) {
  // Slowing down is speeding up run backwards.
  const Limits limits{.velocity = 1.0, .acceleration = 2.0, .jerk = 8.0};
  const Profile profile = MakeSCurve(2.0, limits);
  const double duration = profile.duration();
  for (double t : {0.05, 0.3, 0.6, 1.0, 1.3}) {
    const Sample early = profile.At(t);
    const Sample late = profile.At(duration - t);
    EXPECT_NEAR(late.position, 2.0 - early.position, 1e-12);
    EXPECT_NEAR(late.velocity, early.velocity, 1e-12);
    EXPECT_NEAR(late.acceleration, -early.acceleration, 1e-12);
  }
}

TEST(SCurveTest, TakesLongerThanTheTrapezoidAndApproachesItAsJerkGrows) {
  const Profile trapezoid = MakeTrapezoid(1.0, 1.0, 2.0);

  double previous = std::numeric_limits<double>::infinity();
  for (double jerk : {4.0, 40.0, 400.0, 4000.0}) {
    const Profile s_curve =
        MakeSCurve(1.0, {.velocity = 1.0, .acceleration = 2.0, .jerk = jerk});
    EXPECT_GT(s_curve.duration(), trapezoid.duration()) << "jerk=" << jerk;
    EXPECT_LT(s_curve.duration(), previous) << "jerk=" << jerk;
    previous = s_curve.duration();
  }
  // At 4000 m/s^3 each ramp lasts 0.5 ms.
  EXPECT_NEAR(previous, trapezoid.duration(), 1e-3);
}

// --------------------------------------------------------------- Both shapes

TEST(ProfileTest, ZeroDistanceTakesNoTime) {
  const Limits limits{.velocity = 1.0, .acceleration = 2.0, .jerk = 8.0};
  for (const Profile& profile :
       {MakeTrapezoid(0.0, 1.0, 2.0), MakeSCurve(0.0, limits)}) {
    EXPECT_EQ(profile.duration(), 0.0);
    for (double t : {-1.0, 0.0, 1.0}) {
      EXPECT_EQ(profile.At(t).position, 0.0);
      EXPECT_EQ(profile.At(t).velocity, 0.0);
      EXPECT_EQ(profile.At(t).acceleration, 0.0);
    }
  }
}

TEST(ProfileTest, BeforeTheStartIsTheStartAtRest) {
  const Limits limits{.velocity = 1.0, .acceleration = 2.0, .jerk = 8.0};
  for (const Profile& profile :
       {MakeTrapezoid(1.0, 1.0, 2.0), MakeSCurve(1.0, limits)}) {
    const Sample s = profile.At(-0.5);
    EXPECT_EQ(s.position, 0.0);
    EXPECT_EQ(s.velocity, 0.0);
    EXPECT_EQ(s.acceleration, 0.0);
    EXPECT_EQ(s.jerk, 0.0);
  }
}

TEST(ProfileTest, BadInputGivesNothing) {
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();

  EXPECT_FALSE(Profile::Trapezoid(nan, 1.0, 1.0));
  EXPECT_FALSE(Profile::Trapezoid(inf, 1.0, 1.0));
  EXPECT_FALSE(Profile::Trapezoid(1.0, 0.0, 1.0));
  EXPECT_FALSE(Profile::Trapezoid(1.0, -1.0, 1.0));
  EXPECT_FALSE(Profile::Trapezoid(1.0, 1.0, 0.0));
  EXPECT_FALSE(Profile::Trapezoid(1.0, 1.0, nan));
  EXPECT_FALSE(Profile::Trapezoid(1.0, inf, 1.0));

  EXPECT_FALSE(Profile::SCurve(nan, {1.0, 1.0, 1.0}));
  EXPECT_FALSE(Profile::SCurve(1.0, {0.0, 1.0, 1.0}));
  EXPECT_FALSE(Profile::SCurve(1.0, {1.0, -1.0, 1.0}));
  EXPECT_FALSE(Profile::SCurve(1.0, {1.0, 1.0, 0.0}));
  EXPECT_FALSE(Profile::SCurve(1.0, {1.0, 1.0, inf}));
}

// Draws distances and limits across several orders of magnitude, in both
// directions, and runs the full set of checks on each. The fixed seed makes
// a failure repeatable.
TEST(ProfileTest, RandomMovesStayWithinLimitsAndEndOnTarget) {
  std::mt19937 generator(20261119);
  // A value between `low` and `high`, spread evenly by order of magnitude.
  auto draw = [&generator](double low, double high) {
    const double unit = generator() / 4294967296.0;  // [0, 1)
    return std::exp(std::log(low) + unit * (std::log(high) - std::log(low)));
  };

  for (int trial = 0; trial < 300; ++trial) {
    SCOPED_TRACE(testing::Message() << "trial " << trial);
    const double sign = generator() % 2 == 0 ? 1.0 : -1.0;
    const double distance = sign * draw(1e-4, 10.0);
    const Limits limits{.velocity = draw(0.05, 5.0),
                        .acceleration = draw(0.1, 50.0),
                        .jerk = draw(1.0, 1000.0)};

    const Profile trapezoid =
        MakeTrapezoid(distance, limits.velocity, limits.acceleration);
    CheckProfile(trapezoid, distance, limits, false);

    const Profile s_curve = MakeSCurve(distance, limits);
    CheckProfile(s_curve, distance, limits, true);

    // Limiting jerk can only cost time.
    EXPECT_GE(s_curve.duration(), trapezoid.duration() * (1.0 - kSlack));
  }
}

}  // namespace
