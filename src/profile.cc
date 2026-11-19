#include "src/profile.h"

#include <algorithm>
#include <cmath>

namespace cell {
namespace {

bool IsPositiveAndFinite(double x) { return std::isfinite(x) && x > 0.0; }

// The state `dt` after `start`, with start.jerk held constant.
Sample Advance(const Sample& start, double dt) {
  Sample s;
  s.jerk = start.jerk;
  s.acceleration = start.acceleration + start.jerk * dt;
  s.velocity = start.velocity + start.acceleration * dt +
               start.jerk * dt * dt / 2.0;
  s.position = start.position + start.velocity * dt +
               start.acceleration * dt * dt / 2.0 +
               start.jerk * dt * dt * dt / 6.0;
  return s;
}

}  // namespace

void Profile::Append(double duration, double acceleration, double jerk) {
  if (duration <= 0.0) return;

  Segment segment;
  segment.duration = duration;
  if (!segments_.empty()) {
    const Segment& last = segments_.back();
    segment.start_time = last.start_time + last.duration;
    segment.start = Advance(last.start, last.duration);
  }
  // Position and velocity carry over from the previous segment. Acceleration
  // and jerk are set, not carried: the trapezoid's acceleration jumps, and
  // setting the S-curve's keeps rounding error from building up.
  segment.start.acceleration = acceleration;
  segment.start.jerk = jerk;

  segments_.push_back(segment);
  duration_ = segment.start_time + segment.duration;
}

std::optional<Profile> Profile::Trapezoid(double distance, double max_velocity,
                                          double max_acceleration) {
  if (!std::isfinite(distance) || !IsPositiveAndFinite(max_velocity) ||
      !IsPositiveAndFinite(max_acceleration)) {
    return std::nullopt;
  }

  Profile profile;
  profile.distance_ = distance;
  profile.direction_ = distance < 0.0 ? -1.0 : 1.0;
  const double d = std::abs(distance);
  if (d == 0.0) return profile;

  const double a = max_acceleration;

  // Time spent accelerating. Reaching full speed takes v/a. A short move has
  // to start braking at its midpoint, d/2 = a t^2 / 2, before that.
  const double t_accel = std::min(max_velocity / a, std::sqrt(d / a));
  const double peak_velocity = a * t_accel;

  // Speeding up and slowing down each cover peak_velocity * t_accel / 2.
  // The cruise covers the rest.
  const double t_cruise = std::max(0.0, d / peak_velocity - t_accel);

  profile.Append(t_accel, a, 0.0);
  profile.Append(t_cruise, 0.0, 0.0);
  profile.Append(t_accel, -a, 0.0);
  return profile;
}

std::optional<Profile> Profile::SCurve(double distance, const Limits& limits) {
  if (!std::isfinite(distance) || !IsPositiveAndFinite(limits.velocity) ||
      !IsPositiveAndFinite(limits.acceleration) ||
      !IsPositiveAndFinite(limits.jerk)) {
    return std::nullopt;
  }

  Profile profile;
  profile.distance_ = distance;
  profile.direction_ = distance < 0.0 ? -1.0 : 1.0;
  const double d = std::abs(distance);
  if (d == 0.0) return profile;

  const double j = limits.jerk;

  // Speeding up has three parts: acceleration ramps up for t_ramp, holds for
  // t_hold, ramps down for t_ramp. With peak acceleration A = j * t_ramp:
  //
  //   velocity gained    V = A * (t_ramp + t_hold)
  //   distance covered   V * (2 t_ramp + t_hold) / 2
  //
  // Slowing down is the mirror image and covers the same distance.

  // The ramp ends at whichever comes first:
  //   the acceleration limit                       t = a / j
  //   the velocity limit, with no hold   j t^2 = v  →  t = sqrt(v / j)
  //   the distance, with no hold or cruise
  //                                    2 j t^3 = d  →  t = cbrt(d / (2 j))
  const double t_ramp =
      std::min({limits.acceleration / j, std::sqrt(limits.velocity / j),
                std::cbrt(d / (2.0 * j))});
  const double peak_acceleration = j * t_ramp;

  // The hold ends at whichever comes first:
  //   the velocity limit    A (t_ramp + t_hold) = v
  //   the distance, with no cruise
  //                         A (t_ramp + t_hold)(2 t_ramp + t_hold) = d,
  //                         a quadratic in t_hold
  // Both come out as zero when the ramp was cut short by velocity or
  // distance, so a hold only happens at the acceleration limit.
  const double hold_for_velocity =
      limits.velocity / peak_acceleration - t_ramp;
  const double hold_for_distance =
      (std::sqrt(t_ramp * t_ramp + 4.0 * d / peak_acceleration) -
       3.0 * t_ramp) /
      2.0;
  const double t_hold =
      std::max(0.0, std::min(hold_for_velocity, hold_for_distance));

  const double peak_velocity = peak_acceleration * (t_ramp + t_hold);
  const double t_cruise =
      std::max(0.0, d / peak_velocity - (2.0 * t_ramp + t_hold));

  const double a = peak_acceleration;
  profile.Append(t_ramp, 0.0, j);    // acceleration ramps up
  profile.Append(t_hold, a, 0.0);    // holds
  profile.Append(t_ramp, a, -j);     // ramps down to zero
  profile.Append(t_cruise, 0.0, 0.0);
  profile.Append(t_ramp, 0.0, -j);   // deceleration ramps up
  profile.Append(t_hold, -a, 0.0);   // holds
  profile.Append(t_ramp, -a, j);     // ramps down to zero
  return profile;
}

Sample Profile::At(double t) const {
  if (t >= duration_) {
    // The exact end, not the last segment's arithmetic. Also covers the
    // zero-distance move, which has no segments.
    Sample end;
    end.position = distance_;
    return end;
  }
  if (t <= 0.0) return Sample{};

  // Last segment that starts at or before t. There are at most seven.
  const Segment* segment = &segments_.front();
  for (const Segment& candidate : segments_) {
    if (candidate.start_time > t) break;
    segment = &candidate;
  }

  Sample s = Advance(segment->start, t - segment->start_time);
  s.position *= direction_;
  s.velocity *= direction_;
  s.acceleration *= direction_;
  s.jerk *= direction_;
  return s;
}

}  // namespace cell
