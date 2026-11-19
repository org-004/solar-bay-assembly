// Velocity profiles: how far along a move to be at each instant.
//
// A profile covers one move along one axis, from rest at position 0 to rest
// at position `distance`. It answers "where, how fast, at what acceleration
// at time t", staying within given limits and taking the shortest time those
// limits allow.
//
// Two shapes:
//
//   Trapezoid  Limits velocity and acceleration. Acceleration jumps between
//              +max, 0 and -max, so velocity over time is a trapezoid (or a
//              triangle, when the move is too short to reach full speed).
//              Jerk, the rate of change of acceleration, is unbounded at the
//              jumps.
//
//   S-curve    Also limits jerk. Acceleration ramps instead of jumping, which
//              rounds the corners of the trapezoid. Up to seven segments:
//              ramp acceleration up, hold it, ramp it down, cruise, then the
//              mirror image to stop.
//
// Units are the caller's: metres or radians, and seconds.

#ifndef SRC_PROFILE_H_
#define SRC_PROFILE_H_

#include <optional>
#include <vector>

namespace cell {

// Largest allowed magnitudes. All must be positive and finite.
struct Limits {
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;  // Not used by the trapezoid.
};

// The state of the move at one instant.
struct Sample {
  double position = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double jerk = 0.0;
};

class Profile {
 public:
  // Shortest rest-to-rest move over `distance` within the velocity and
  // acceleration limits. `distance` may be negative or zero. Returns nothing
  // if `distance` is not finite or a limit is not positive and finite.
  static std::optional<Profile> Trapezoid(double distance, double max_velocity,
                                          double max_acceleration);

  // The same with the jerk limit as well.
  static std::optional<Profile> SCurve(double distance, const Limits& limits);

  // How long the move takes. Zero for a move of zero distance.
  double duration() const { return duration_; }

  // Where the move ends.
  double distance() const { return distance_; }

  // The state at time `t`. Before 0 it is the start at rest; after
  // duration() it is the end at rest.
  //
  // The trapezoid reports jerk as 0 everywhere. At an instant where its
  // acceleration jumps, the value reported is the one after the jump.
  Sample At(double t) const;

 private:
  // A stretch of time over which jerk is constant. `start` is the state at
  // the beginning of the stretch, for a move in the positive direction.
  struct Segment {
    double start_time = 0.0;
    double duration = 0.0;
    Sample start;
  };

  Profile() = default;

  // Adds a segment that begins where the last one ended, at the given
  // acceleration and jerk. A segment of zero duration is left out.
  void Append(double duration, double acceleration, double jerk);

  std::vector<Segment> segments_;
  double duration_ = 0.0;
  double distance_ = 0.0;
  double direction_ = 1.0;  // +1 or -1: segments describe a positive move.
};

}  // namespace cell

#endif  // SRC_PROFILE_H_
