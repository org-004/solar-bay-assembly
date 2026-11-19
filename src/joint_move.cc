#include "src/joint_move.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cell {
namespace {

bool AllPositiveAndFinite(const Eigen::VectorXd& v) {
  return v.allFinite() && (v.array() > 0.0).all();
}

// The largest rate at which the fraction may change so that joint i, which
// moves by delta[i], stays within limit[i]. A joint's rate is delta[i] times
// the fraction's rate, so each moving joint allows limit[i] / |delta[i]| and
// the smallest of those governs. Joints that do not move allow anything.
double FractionLimit(const Eigen::VectorXd& limit,
                     const Eigen::VectorXd& delta) {
  double smallest = std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < delta.size(); ++i) {
    if (delta[i] != 0.0) {
      smallest = std::min(smallest, limit[i] / std::abs(delta[i]));
    }
  }
  return smallest;
}

}  // namespace

std::optional<JointMove> JointMove::Plan(const Eigen::VectorXd& start,
                                         const Eigen::VectorXd& goal,
                                         const JointLimits& limits,
                                         ProfileShape shape) {
  const Eigen::Index n = start.size();
  const bool with_jerk = shape == ProfileShape::kSCurve;

  if (goal.size() != n || limits.velocity.size() != n ||
      limits.acceleration.size() != n ||
      (with_jerk && limits.jerk.size() != n)) {
    return std::nullopt;
  }
  if (!start.allFinite() || !goal.allFinite() ||
      !AllPositiveAndFinite(limits.velocity) ||
      !AllPositiveAndFinite(limits.acceleration) ||
      (with_jerk && !AllPositiveAndFinite(limits.jerk))) {
    return std::nullopt;
  }

  const Eigen::VectorXd delta = goal - start;

  // Nothing moves: a profile of zero distance, which has zero duration.
  if ((delta.array() == 0.0).all()) {
    std::optional<Profile> still = Profile::Trapezoid(0.0, 1.0, 1.0);
    return JointMove(start, goal, *still);
  }

  Limits fraction_limits;
  fraction_limits.velocity = FractionLimit(limits.velocity, delta);
  fraction_limits.acceleration = FractionLimit(limits.acceleration, delta);

  std::optional<Profile> fraction;
  if (with_jerk) {
    fraction_limits.jerk = FractionLimit(limits.jerk, delta);
    fraction = Profile::SCurve(1.0, fraction_limits);
  } else {
    fraction = Profile::Trapezoid(1.0, fraction_limits.velocity,
                                  fraction_limits.acceleration);
  }
  // Empty if a limit divided by a tiny delta overflowed to infinity.
  if (!fraction) return std::nullopt;

  return JointMove(start, goal, *fraction);
}

JointSample JointMove::At(double t) const {
  const Sample f = fraction_.At(t);

  JointSample s;
  // At the end, `goal` itself: start + (goal - start) can differ from it in
  // the last bit.
  s.position = t >= duration() ? goal_ : (start_ + delta_ * f.position).eval();
  s.velocity = delta_ * f.velocity;
  s.acceleration = delta_ * f.acceleration;
  s.jerk = delta_ * f.jerk;
  return s;
}

}  // namespace cell
