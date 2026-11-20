// Timing along a path: one motion through a list of joint-space waypoints
// that rounds each corner instead of stopping at it.
//
// A JointMove goes from rest to rest along one straight line in joint
// space. Run one after another (ArmFollower::MoveThrough), the arm stops at
// every waypoint. That is safe but slow: each stop costs the time to brake
// to zero and to speed up again, most of a second at the arm's limits, and a
// line of the tool tip solved every centimetre has thirty of them. A
// Trajectory goes through the whole list in one motion, from rest at the
// first waypoint to rest exactly at the last, slowing at a corner only as
// much as the corner needs.
//
// The method has two parts: first the shape of the path, then how fast to go
// along it. Keeping them apart is what makes the guarantees easy to state:
// the shape alone decides how far the path strays from the waypoints, and
// the timing alone decides the speeds, accelerations and jerks.
//
// 1. Shape: straight segments, with each corner rounded by a blend.
//
//    The path leaves the incoming segment a distance l before the corner
//    and joins the outgoing one l after it. In between, the direction of
//    travel turns from the incoming direction to the outgoing one: slowly at
//    first, fastest midway, slowly again at the end (it is the mix
//    (1 - g) * incoming + g * outgoing, with g the "smoothstep"
//    3x^2 - 2x^3 of how far through the blend). Because the turning starts
//    and ends gently, the curvature is zero where blend meets segment, so
//    the arm needs no sudden sideways acceleration there. Outside the
//    blends the path is the straight segments themselves, exactly.
//
//    How far a blend strays: its farthest point from the two segments is
//    its middle, (3/16) * l * sin(turn), the turn being the angle between
//    the two directions. l is chosen per corner as the largest that keeps
//    this within max_deviation, and short enough that neighbouring blends
//    do not overlap (each gets at least half of a segment it shares). A
//    gentle corner gets a long blend and is taken fast; a sharp one gets a
//    short, tight blend and is taken slowly, or becomes a stop (see 3). A
//    full reversal (sin = 0) could turn round on the segment itself, but
//    the joints must come to rest to reverse anyway, and stopping there is
//    quicker.
//
// 2. Speed along the path: a jerk-limited path-speed profile from a forward
//    and backward pass.
//
//    Let s measure how far along the path the arm is. The joints follow
//    q(s), so their speed is q'(s) * ds/dt: the path's direction times the
//    path speed. Acceleration and jerk pick up terms from the path's bend:
//
//      speed          q' s.
//      acceleration   q' s..  +  q'' s.^2                   (along + sideways)
//      jerk           q' s... +  3 q'' s. s..  +  q''' s.^3
//
//    The path is cut into stretches. On each, the path speed s., its rate
//    s.. and that rate's rate s... are capped by constants chosen so that,
//    with the largest |q'|, |q''| and |q'''| of each joint anywhere in the
//    stretch, no joint can exceed its velocity, acceleration or jerk limit.
//    On a straight stretch q'' = q''' = 0 and the caps are exactly those of
//    a JointMove along it. On a curved one, the sideways term takes part of
//    the acceleration and jerk, so the caps leave part (a share, see below)
//    for speeding up and slowing down and give the rest to the bend, which
//    sets how fast the stretch may be taken.
//
//    Within a stretch the path speed follows S-curve speed changes (as in
//    Profile: jerk ramps the acceleration up, holds it, ramps it down) and
//    cruises between them. At every boundary between stretches the path
//    acceleration is zero, so the pieces join without a jump in any joint's
//    acceleration. The speed at each boundary is found by the usual pair of
//    passes: backward from the end, how fast the arm may be there and still
//    slow down in time for everything after; forward from the start, how
//    fast it can have got there. Each stretch then speeds up to the highest
//    speed from which it can still slow down to its exit speed in time.
//
//    Stretches start as the pieces of the path (each segment's straight
//    part, each blend). Requiring zero acceleration at every boundary costs
//    time where pieces are short, as on a tool-tip line solved every
//    centimetre, so neighbouring stretches are merged (taking the stricter
//    caps of the two) wherever that makes the whole shorter.
//
// 3. Choices, each tried and the quickest kept.
//
//    - Stop or blend, corner by corner. A sharp corner within a tight
//      deviation gets a small, tight blend, which must be crossed end to
//      end at the slow speed its tightest point allows; stopping slows the
//      arm only near the corner, and is often as quick. Each corner is
//      made a stop if that is quicker.
//    - Fewer waypoints. Waypoints that lie within part of max_deviation of
//      the straight line between their neighbours can be left out (the
//      Douglas-Peucker method); the blends then get the rest. A tool-tip
//      line solved every centimetre is a gently bending joint path with a
//      small corner every 0.02 rad, whose blends can be only a segment
//      long each; many short blends turn the path in bursts, each of which
//      takes jerk to start and stop. Fewer, longer ones turn it evenly.
//    - The share of a curved stretch's acceleration and jerk kept for
//      speeding up and slowing down: 0.3, 0.5 or 0.7.
//    - Stopping at every turning corner, as JointMoves one after another
//      do. So the result is never slower than that.
//
// What is guaranteed, everywhere along the trajectory and for every joint:
//   - velocity, acceleration and jerk within the limits given (scaled by
//     `speed`); velocity and acceleration are continuous, jerk may jump
//     (where a blend meets a straight part, and at stretch boundaries);
//   - distance from the polyline through the waypoints given at most
//     max_deviation (radians, Euclidean distance in joint space);
//   - the straight parts of segments, outside blends, are followed exactly
//     (unless waypoints were left out: then the straight line between the
//     ones kept, which is within max_deviation too);
//   - rest at the first waypoint at t = 0 and exactly at the last from
//     duration() on.
// What is not: time-optimality. It is near it, not at it: each stretch is
// held to the worst bend inside it, the speed changes inside a curved
// stretch get only part of the acceleration, the path acceleration is zero
// at stretch boundaries, and the choices above are made one at a time
// (so a looser max_deviation is usually, not always, quicker). On the
// tests: a single gentle corner is within 1% of a straight move of the
// same length; 31 waypoints on a gentle curve take 1.63 s against 1.15 s
// for a straight move of the same length (most of the difference is jerk:
// with the jerk limit 100 times higher it is 1.21 s against 1.04 s) and
// 9.7 s stopping at each. A sharp corner within a tight deviation must be
// taken nearly at rest by any method: its radius of curvature is a few
// times max_deviation, and the speed around a bend of radius R is at most
// sqrt(acceleration * R). Planning 31 waypoints takes about 20 ms in an
// optimized build.
//
// Units are radians and seconds, as in JointMove.

#ifndef SRC_PATH_TIMING_H_
#define SRC_PATH_TIMING_H_

#include <optional>
#include <vector>

#include <Eigen/Core>

#include "src/joint_move.h"
#include "src/profile.h"

namespace cell {

struct PathTimingOptions {
  // The farthest the path may stray from the polyline through the
  // waypoints, in radians: the Euclidean distance in joint space,
  // sqrt(sum over joints of the angle difference squared).
  //
  // From a Cartesian margin. Turning joint i by an angle moves any point of
  // the arm and what it carries by at most r_i times that angle, where r_i
  // is the farthest that point can be from joint i's axis. Changing all
  // joints moves it by at most sum r_i |dq_i| <= |r| |dq|, where |r| is
  // sqrt(sum r_i^2) (the Cauchy-Schwarz inequality). So a path within
  // max_deviation of the polyline keeps every point of the arm and its
  // load within |r| * max_deviation of where it would be somewhere on the
  // polyline, the path that was checked for collisions.
  //
  // For the KR 210 R2700-2 (models/arm.xml) holding a module whose corners
  // are 1.28 m from the tool's axis, 0.3 m out from the flange, r is about
  // 4.5 m for joint 1 (0.33 + 1.15 + 1.23 + 0.22 + 0.3 + 1.28), 4.2 m for
  // joint 2, 3.0 m for joint 3, 1.8 m for joints 4 and 5 and 1.3 m for
  // joint 6: |r| is about 7.4 m. A collision margin of m metres therefore
  // allows max_deviation = m / 7.4 at most, less whatever else needs a
  // share of the margin (tracking error, the planner's edge checks). The
  // default 0.002 rad is at most 15 mm anywhere on the arm or module, which
  // leaves 5 mm of a 20 mm margin for the rest. It is a worst case: all
  // joints' contributions would have to line up at the farthest point. On
  // 0.3 m tool-tip lines solved every centimetre (tests/path_follow_test.cc)
  // the tip strayed 0.5 to 0.6 mm from the line at the default, and 0.05 to
  // 0.07 mm at 0.0002 rad, which is about what the waypoints' own straight
  // joint moves stray.
  static constexpr double kDefaultMaxDeviation = 0.002;  // rad
  double max_deviation = kDefaultMaxDeviation;

  // Fraction of full speed, in (0, 1], as MoveOptions::speed: velocity
  // limits are scaled by speed, accelerations by speed^2 and jerks by
  // speed^3, so the trajectory is the full-speed one played slower.
  double speed = 1.0;
};

class Trajectory {
 public:
  // The trajectory through `waypoints` within `limits` (velocity,
  // acceleration and jerk, all needed). Returns nothing if there are no
  // waypoints, they or the limits differ in length, a value is not
  // finite, a limit is not positive, options.speed is not in (0, 1], or
  // options.max_deviation is negative or not a number.
  //
  // One waypoint gives a trajectory of zero duration. A waypoint within
  // 1e-9 rad of the one kept before it is dropped as a repeat (the last
  // one replaces the one before it instead, so the end is exact); the
  // path then strays up to 1e-9 rad more. max_deviation 0 stops at every
  // corner that turns, which is what JointMoves one after another do.
  static std::optional<Trajectory> Plan(
      const std::vector<Eigen::VectorXd>& waypoints, const JointLimits& limits,
      const PathTimingOptions& options = {});

  // How long it takes. Zero if every waypoint is the same.
  double duration() const { return duration_; }

  // The state at time `t`. Before 0 it is the first waypoint at rest; from
  // duration() on it is the last, exactly, at rest.
  JointSample At(double t) const;

  // The waypoints as used: the ones given, less repeats.
  const std::vector<Eigen::VectorXd>& waypoints() const { return waypoints_; }

  // The farthest the path strays from the waypoints' polyline, as built.
  // At most options.max_deviation.
  double deviation() const { return deviation_; }

  // A piece of the path: the straight part of a segment, or a blend. It
  // begins at path position `start` and is `length` long. On a line,
  // s = start + x is at point + x * in. A blend is 2 l long, from
  // point - l * in to point + l * out.
  struct Piece {
    double start = 0.0;
    double length = 0.0;
    bool blend = false;
    Eigen::VectorXd point;  // line: where it begins; blend: the corner
    Eigen::VectorXd in;     // unit direction in (line: its direction)
    Eigen::VectorXd out;    // unit direction out (line: the same)
    double half = 0.0;      // blend: l
  };

  // The path's pieces in order, for seeing what the planner did; as are
  // the counts of corners taken without stopping, and of stretches in the
  // speed profile.
  const std::vector<Piece>& pieces() const { return pieces_; }
  int blends() const { return blends_; }
  int stretches() const { return stretches_; }

 private:
  // A stretch of time over which the path jerk is constant. `state` is the
  // path position, speed, acceleration and jerk at its beginning.
  struct Segment {
    double start_time = 0.0;
    double duration = 0.0;
    Sample state;
  };

  Trajectory() = default;

  std::vector<Eigen::VectorXd> waypoints_;
  std::vector<Piece> pieces_;      // in path order
  std::vector<Segment> segments_;  // in time order
  double length_ = 0.0;            // of the path, along s
  double duration_ = 0.0;
  double deviation_ = 0.0;
  int blends_ = 0;
  int stretches_ = 0;
};

}  // namespace cell

#endif  // SRC_PATH_TIMING_H_
