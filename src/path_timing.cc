#include "src/path_timing.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace cell {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

// A waypoint this close to the one kept before it is a repeat, rad. The
// direction from one to the other would be mostly rounding error.
constexpr double kSamePoint = 1e-9;

// A corner whose turn has a sine below this goes straight on (or straight
// back): its blend does not stray, whatever its size.
constexpr double kStraightOn = 1e-12;

// A blend shorter than this is not worth having: the corner becomes a stop.
// It would be crossed at well under a micro-radian per second anyway.
constexpr double kShortestBlend = 1e-12;

// The straight part of a segment shorter than this is left out. Leaving it
// out moves where the next piece starts by no more than this.
constexpr double kShortestLine = 1e-15;

// A blend's middle is this times l * sin(turn) from the two segments.
//
// Why. Put the corner at the origin, e1 pointing back along the incoming
// segment and e2 along the outgoing one. The blend is
//   a(x) e1 + b(x) e2,  a(x) = l (1 - 2x + 2x^3 - x^4),  b(x) = a(1 - x),
// for x from 0 to 1: its derivative along the path is the smoothstep mix of
// the two directions (see the header). a falls from l to 0 and b rises
// from 0 to l. In the first half b <= a, so the point is b sin(angle) from
// the incoming segment's line, with its foot on the segment; b is at most
// b(1/2) = 3/16 l. The second half is the mirror image.
constexpr double kBulge = 3.0 / 16.0;

// Shares of a curved stretch's acceleration and jerk kept for speeding up
// and slowing down along the path; the rest is the bend's. More share means
// quicker speed changes inside the stretch but a lower speed through it.
// Each is tried, and the one that gives the shortest whole is kept.
constexpr double kShares[] = {0.3, 0.5, 0.7};

// Parts of the deviation tried for leaving waypoints out (see Simplify);
// the rest is for the blends. 0 keeps every waypoint.
constexpr double kSimplify[] = {0.0, 0.25, 0.5};

// Steps of bisection. Each halves the interval: 60 leave about 1e-18 of it.
constexpr int kBisections = 60;

// A merge of neighbouring stretches that leaves the whole no more than this
// fraction slower is accepted: it is as good, and fewer boundaries leave
// more freedom for the next merge.
constexpr double kSameTime = 1e-12;

bool AllPositiveAndFinite(const Eigen::VectorXd& v) {
  return v.allFinite() && (v.array() > 0.0).all();
}

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

// ------------------------------------------------------------ path speed
//
// A change of path speed by dv, starting and ending with zero acceleration,
// as fast as the path limits allow: the first half of an S-curve, ramping
// the acceleration up at the jerk limit, holding it, ramping it down. If dv
// is too small to reach the acceleration limit, the ramps meet in the
// middle (each lasting sqrt(dv / j)).

double ChangeTime(double dv, const Limits& limits) {
  if (dv <= 0.0) return 0.0;
  const double a = limits.acceleration, j = limits.jerk;
  if (dv * j <= a * a) return 2.0 * std::sqrt(dv / j);
  return dv / a + a / j;
}

// The acceleration curve of a change is the same read forwards and
// backwards, so the speed curve is symmetric about its midpoint and its
// average is the average of the two ends.
double ChangeDistance(double v1, double v2, const Limits& limits) {
  return 0.5 * (v1 + v2) * ChangeTime(std::abs(v2 - v1), limits);
}

// --------------------------------------------------------------- stretches

// A stretch of the path traversed under one set of path limits, while
// planning. m1, m2 and m3 are each joint's largest |q'|, |q''| and |q'''|
// anywhere in the stretch, with s as the variable.
struct Stretch {
  double length = 0.0;
  Eigen::VectorXd m1, m2, m3;
  bool stop_after = false;  // a sharp corner at its end: at rest there
  Limits limits;            // on s., s.. and s...
};

// The largest |q'|, |q''|, |q'''| of a piece.
//
// On a line q' is the direction, and nothing else. On a blend of half
// length l, with x = (s - start) / 2l and g the smoothstep 3x^2 - 2x^3,
//   q'   = (1 - g) in + g out          between the two: |q'_i| at most
//                                      max(|in_i|, |out_i|)
//   q''  = g'(x) / 2l (out - in)       g' = 6x(1 - x) at most 3/2
//   q''' = g''(x) / 4l^2 (out - in)    g'' = 6 - 12x at most 6
Stretch FromPiece(const Trajectory::Piece& piece) {
  Stretch stretch;
  stretch.length = piece.length;
  stretch.m1 = piece.in.cwiseAbs().cwiseMax(piece.out.cwiseAbs());
  const Eigen::VectorXd turn = (piece.out - piece.in).cwiseAbs();
  if (piece.blend) {
    stretch.m2 = 0.75 / piece.half * turn;
    stretch.m3 = 1.5 / (piece.half * piece.half) * turn;
  } else {
    stretch.m2 = Eigen::VectorXd::Zero(turn.size());
    stretch.m3 = Eigen::VectorXd::Zero(turn.size());
  }
  return stretch;
}

// Path limits under which no joint can exceed its own anywhere in the
// stretch. With path speed at most V, acceleration at most A and jerk at
// most J, joint i's
//   speed          |q'_i| s.                       <= m1 V
//   acceleration   |q'_i s.. + q''_i s.^2|         <= m1 A + m2 V^2
//   jerk           |q'_i s... + 3 q''_i s. s..     <= m1 J + 3 m2 V A
//                    + q'''_i s.^3|                     + m3 V^3
// and each must be within the joint's limit.
//
// On a straight stretch (m2 = m3 = 0) that gives each of V, A and J
// directly, the same as JointMove does. On a curved one, A is first set to
// `share` of what a straight stretch would allow, J must keep at least
// `share` of it too, and V is the largest that leaves room for both. Then,
// at that V, A is raised as far as the acceleration limits allow, if the
// jerk can still keep its share: on a gentle curve, where V is held by the
// velocity limit, the bend takes almost nothing.
Limits StretchLimits(const Stretch& stretch, const JointLimits& joint,
                     double share) {
  double v_max = kInfinity, a_straight = kInfinity, j_straight = kInfinity;
  bool curved = false;
  for (Eigen::Index i = 0; i < stretch.m1.size(); ++i) {
    // A joint that does not move here bends nothing either: its m2 and m3
    // are at most twice its m1.
    if (stretch.m1[i] == 0.0) continue;
    v_max = std::min(v_max, joint.velocity[i] / stretch.m1[i]);
    a_straight = std::min(a_straight, joint.acceleration[i] / stretch.m1[i]);
    j_straight = std::min(j_straight, joint.jerk[i] / stretch.m1[i]);
    if (stretch.m2[i] > 0.0 || stretch.m3[i] > 0.0) curved = true;
  }
  if (!curved) return {v_max, a_straight, j_straight};

  const double a = share * a_straight;
  const double j_min = share * j_straight;
  auto fits = [&](double v) {
    for (Eigen::Index i = 0; i < stretch.m1.size(); ++i) {
      const double m1 = stretch.m1[i], m2 = stretch.m2[i], m3 = stretch.m3[i];
      if (m1 == 0.0) continue;
      if (m1 * a + m2 * v * v > joint.acceleration[i]) return false;
      if (m1 * j_min + 3.0 * m2 * v * a + m3 * v * v * v > joint.jerk[i]) {
        return false;
      }
    }
    return true;
  };
  // Both tests only get harder as v grows, so bisection finds the largest.
  double v = v_max;
  if (!fits(v)) {
    double low = 0.0, high = v_max;
    for (int k = 0; k < kBisections; ++k) {
      const double mid = 0.5 * (low + high);
      (fits(mid) ? low : high) = mid;
    }
    v = low;
  }

  // The jerk left at this speed with acceleration `accel`.
  auto jerk_left = [&](double accel) {
    double j = kInfinity;
    for (Eigen::Index i = 0; i < stretch.m1.size(); ++i) {
      const double m1 = stretch.m1[i], m2 = stretch.m2[i], m3 = stretch.m3[i];
      if (m1 == 0.0) continue;
      j = std::min(j, (joint.jerk[i] - 3.0 * m2 * v * accel - m3 * v * v * v) /
                          m1);
    }
    return j;
  };
  double a_raised = kInfinity;
  for (Eigen::Index i = 0; i < stretch.m1.size(); ++i) {
    const double m1 = stretch.m1[i], m2 = stretch.m2[i];
    if (m1 == 0.0) continue;
    a_raised = std::min(a_raised, (joint.acceleration[i] - m2 * v * v) / m1);
  }
  const double j_raised = jerk_left(a_raised);
  if (a_raised > a && j_raised >= j_min) return {v, a_raised, j_raised};
  return {v, a, jerk_left(a)};
}

// Two neighbouring stretches as one, held to the stricter bounds of both.
Stretch Join(const Stretch& first, const Stretch& second,
             const JointLimits& joint, double share) {
  Stretch joined;
  joined.length = first.length + second.length;
  joined.m1 = first.m1.cwiseMax(second.m1);
  joined.m2 = first.m2.cwiseMax(second.m2);
  joined.m3 = first.m3.cwiseMax(second.m3);
  joined.stop_after = second.stop_after;
  joined.limits = StretchLimits(joined, joint, share);
  return joined;
}

// The fastest path speed at the far end of `stretch` that can be reached
// from `from` at its near end; by symmetry also the fastest at the near end
// from which the far end can be reached at `from`.
double Reach(double from, const Stretch& stretch) {
  const double cap = stretch.limits.velocity;
  if (from >= cap) return cap;
  if (ChangeDistance(from, cap, stretch.limits) <= stretch.length) return cap;
  double low = from, high = cap;
  for (int k = 0; k < kBisections; ++k) {
    const double mid = 0.5 * (low + high);
    (ChangeDistance(from, mid, stretch.limits) <= stretch.length ? low
                                                                  : high) = mid;
  }
  return low;
}

// The highest speed a stretch entered at `in` and left at `out` can reach in
// between: the most it can speed up and still slow down in time.
double Peak(double in, double out, const Stretch& stretch) {
  auto needed = [&](double v) {
    return ChangeDistance(in, v, stretch.limits) +
           ChangeDistance(v, out, stretch.limits);
  };
  const double cap = stretch.limits.velocity;
  if (needed(cap) <= stretch.length) return cap;
  double low = std::max(in, out), high = cap;
  for (int k = 0; k < kBisections; ++k) {
    const double mid = 0.5 * (low + high);
    (needed(mid) <= stretch.length ? low : high) = mid;
  }
  return low;
}

// How a list of stretches is taken: the path speed at each boundary
// (speed[0] at the start, speed[n] at the end), the peak in each, and the
// whole time.
struct Timing {
  std::vector<double> speed;
  std::vector<double> peak;
  double duration = 0.0;
};

Timing Time(const std::vector<Stretch>& stretches) {
  const int n = static_cast<int>(stretches.size());
  Timing timing;
  // At rest at both ends and at every sharp corner; elsewhere no faster
  // than either side allows.
  timing.speed.assign(n + 1, 0.0);
  for (int b = 1; b < n; ++b) {
    if (!stretches[b - 1].stop_after) {
      timing.speed[b] = std::min(stretches[b - 1].limits.velocity,
                                 stretches[b].limits.velocity);
    }
  }
  // Backward: no faster than allows slowing down in time for what follows.
  for (int b = n - 1; b >= 1; --b) {
    timing.speed[b] =
        std::min(timing.speed[b], Reach(timing.speed[b + 1], stretches[b]));
  }
  // Forward: no faster than the arm can have got to by then.
  for (int b = 1; b < n; ++b) {
    timing.speed[b] = std::min(timing.speed[b],
                               Reach(timing.speed[b - 1], stretches[b - 1]));
  }

  timing.peak.resize(n);
  for (int k = 0; k < n; ++k) {
    const Stretch& stretch = stretches[k];
    const double in = timing.speed[k], out = timing.speed[k + 1];
    const double peak = Peak(in, out, stretch);
    timing.peak[k] = peak;
    const double changing = ChangeDistance(in, peak, stretch.limits) +
                            ChangeDistance(peak, out, stretch.limits);
    timing.duration += ChangeTime(peak - in, stretch.limits) +
                       ChangeTime(peak - out, stretch.limits);
    if (peak > 0.0) {
      timing.duration += std::max(0.0, stretch.length - changing) / peak;
    }
  }
  return timing;
}

// Merges neighbouring stretches wherever that makes the whole no slower,
// one boundary at a time, sweeping until nothing changes. Also tries one
// stretch between each pair of stops. Returns the best found and its time.
std::vector<Stretch> Merge(std::vector<Stretch> stretches,
                           const JointLimits& joint, double share,
                           double* duration) {
  double best = Time(stretches).duration;
  for (bool changed = true; changed;) {
    changed = false;
    size_t b = 0;
    while (b + 1 < stretches.size()) {
      if (stretches[b].stop_after) {
        ++b;
        continue;
      }
      std::vector<Stretch> candidate = stretches;
      candidate[b] = Join(stretches[b], stretches[b + 1], joint, share);
      candidate.erase(candidate.begin() + static_cast<long>(b) + 1);
      const double time = Time(candidate).duration;
      if (time <= best * (1.0 + kSameTime)) {
        // Stay at b: the merged stretch may merge with the next one too.
        if (time < best * (1.0 - kSameTime)) changed = true;
        stretches = std::move(candidate);
        best = std::min(best, time);
      } else {
        ++b;
      }
    }
  }

  std::vector<Stretch> whole;
  for (const Stretch& stretch : stretches) {
    if (!whole.empty() && !whole.back().stop_after) {
      whole.back() = Join(whole.back(), stretch, joint, share);
    } else {
      whole.push_back(stretch);
    }
  }
  const double whole_time = Time(whole).duration;
  if (whole_time < best) {
    best = whole_time;
    stretches = std::move(whole);
  }
  *duration = best;
  return stretches;
}

// ------------------------------------------------------------------ shape

// The waypoints less any within kSamePoint of the one kept before it. The
// last is kept exactly: if it repeats the one before, it takes that one's
// place.
std::vector<Eigen::VectorXd> WithoutRepeats(
    const std::vector<Eigen::VectorXd>& waypoints) {
  std::vector<Eigen::VectorXd> points = {waypoints.front()};
  for (size_t w = 1; w < waypoints.size(); ++w) {
    const double gap = (waypoints[w] - points.back()).norm();
    if (gap > kSamePoint) {
      points.push_back(waypoints[w]);
    } else if (w + 1 == waypoints.size() && gap > 0.0) {
      if (points.size() > 1) {
        points.back() = waypoints[w];
      } else {
        points.push_back(waypoints[w]);
      }
    }
  }
  return points;
}

struct Shape {
  std::vector<Trajectory::Piece> pieces;
  std::vector<bool> stop_after;  // per piece: a sharp corner at its end
  double length = 0.0;
  double deviation = 0.0;
  int blends = 0;
};

// The polyline through `points` (at least two, no two neighbours the same)
// cut into segments, and how each corner turns.
struct Polyline {
  std::vector<double> length;              // of each segment
  std::vector<Eigen::VectorXd> direction;  // of each, length 1
  // Corner c is at points[c], between segments c - 1 and c, for c from 1 to
  // segments - 1 (entry 0 is unused). The sine of its turn, and whether it
  // goes straight on (no turn, as far as rounding can tell).
  std::vector<double> sine;
  std::vector<bool> straight_on;
};

Polyline Measure(const std::vector<Eigen::VectorXd>& points) {
  const int segments = static_cast<int>(points.size()) - 1;
  Polyline line;
  line.length.resize(segments);
  line.direction.resize(segments);
  line.sine.assign(segments, 0.0);
  line.straight_on.assign(segments, false);
  for (int k = 0; k < segments; ++k) {
    const Eigen::VectorXd delta = points[k + 1] - points[k];
    line.length[k] = delta.norm();
    line.direction[k] = delta / line.length[k];
  }
  for (int c = 1; c < segments; ++c) {
    // The part of the new direction square to the old one: |sin(turn)|.
    // (Taken this way rather than from the cosine, which loses the small
    // angles to rounding.)
    const Eigen::VectorXd& before = line.direction[c - 1];
    const Eigen::VectorXd& after = line.direction[c];
    const double along = after.dot(before);
    line.sine[c] = std::min(1.0, (after - along * before).norm());
    line.straight_on[c] = along > 0.0 && line.sine[c] < kStraightOn;
  }
  return line;
}

// The polyline with its corners blended, none straying more than
// `max_deviation`, except that corners marked in `stop` are sharp, with the
// arm at rest there.
Shape BuildShape(const std::vector<Eigen::VectorXd>& points,
                 const Polyline& line, double max_deviation,
                 const std::vector<bool>& stop) {
  const int segments = static_cast<int>(line.length.size());
  const std::vector<double>& length = line.length;
  const std::vector<Eigen::VectorXd>& direction = line.direction;
  const std::vector<double>& sine = line.sine;

  // First the blend each corner would like: the largest that strays no
  // more than allowed. Straight on, any size will do. Straight back (sine
  // also zero) any size stays on the segment too, unless the corner is to
  // be a stop.
  std::vector<double> wanted(segments, 0.0);
  for (int c = 1; c < segments; ++c) {
    if (line.straight_on[c]) {
      wanted[c] = kInfinity;
    } else if (stop[c]) {
      wanted[c] = 0.0;
    } else if (sine[c] < kStraightOn) {
      wanted[c] = kInfinity;
    } else {
      // A hair smaller than exact, so that rounding cannot take the bulge
      // past the bound.
      wanted[c] = max_deviation / (kBulge * sine[c]) * (1.0 - 1e-12);
    }
  }
  // Then what it can have. Each blend is l long on each side of its corner,
  // so two blends sharing a segment must fit in it: each gets at least
  // half, and more if the other wants less. A segment at either end of the
  // path has only one blend on it.
  std::vector<double> half(segments + 1, 0.0);
  for (int c = 1; c < segments; ++c) {
    const double before =
        c == 1 ? length[0]
               : std::max(length[c - 1] / 2, length[c - 1] - wanted[c - 1]);
    const double after =
        c == segments - 1
            ? length[c]
            : std::max(length[c] / 2, length[c] - wanted[c + 1]);
    half[c] = std::min({wanted[c], before, after});
    if (half[c] < kShortestBlend) half[c] = 0.0;
  }

  Shape shape;
  double s = 0.0;
  for (int k = 0; k < segments; ++k) {
    const double begin = half[k];      // zero for k == 0
    const double end = half[k + 1];    // zero for the last segment
    const double straight = length[k] - begin - end;
    if (straight > kShortestLine) {
      Trajectory::Piece part;
      part.start = s;
      part.length = straight;
      part.point = points[k] + begin * direction[k];
      part.in = direction[k];
      part.out = direction[k];
      shape.pieces.push_back(part);
      shape.stop_after.push_back(false);
      s += straight;
    }
    if (k + 1 == segments) break;
    if (half[k + 1] > 0.0) {
      Trajectory::Piece blend;
      blend.start = s;
      blend.length = 2.0 * half[k + 1];
      blend.blend = true;
      blend.point = points[k + 1];
      blend.in = direction[k];
      blend.out = direction[k + 1];
      blend.half = half[k + 1];
      shape.pieces.push_back(blend);
      shape.stop_after.push_back(false);
      s += blend.length;
      ++shape.blends;
      // A corner that turns by less than rounding can tell strays by less
      // than rounding: not counted.
      if (sine[k + 1] >= kStraightOn) {
        shape.deviation =
            std::max(shape.deviation, kBulge * half[k + 1] * sine[k + 1]);
      }
    } else {
      // A sharp corner: the direction jumps, so the arm must be at rest.
      // There is always a piece before it: the segment's straight part, or
      // the blend before it if that took the whole segment.
      shape.stop_after.back() = true;
    }
  }
  shape.length = s;
  return shape;
}

// Distance from q to the segment from a to b.
double DistanceToSegment(const Eigen::VectorXd& q, const Eigen::VectorXd& a,
                         const Eigen::VectorXd& b) {
  const Eigen::VectorXd ab = b - a;
  const double squared = ab.squaredNorm();
  // A path that comes back to where it was: the "segment" is a point.
  if (!(squared > 0.0)) return (q - a).norm();
  const double x = std::clamp((q - a).dot(ab) / squared, 0.0, 1.0);
  return (q - a - x * ab).norm();
}

// The waypoints with those left out that lie within `tolerance` of the
// straight line between the ones kept either side (Douglas and Peucker's
// method: keep the ends; find the waypoint farthest from the line between
// them; if it is within the tolerance, drop everything between, else keep
// it and do the same on each side). Sets `*strayed` to the largest distance
// of a dropped waypoint from its line.
//
// Why. A waypoint every centimetre of a tool-tip line makes a gently bending
// joint path with a small corner every 0.02 rad. Each corner's blend can
// then be only one segment long, and many short blends in a row turn the
// path in bursts, each starting and stopping its turn: that takes jerk, and
// holds the speed down. Fewer, longer blends turn it more evenly.
//
// The path between kept waypoints is then within `*strayed` of the original
// polyline: every point of the new line has a point of the old polyline
// square to it (the old one runs from one end of the line to the other),
// and that point is within the tolerance of the line, because the old
// waypoints there are and the tolerance's neighbourhood of a segment is
// convex.
std::vector<Eigen::VectorXd> Simplify(const std::vector<Eigen::VectorXd>& points,
                                      double tolerance, double* strayed) {
  *strayed = 0.0;
  std::vector<bool> keep(points.size(), false);
  keep.front() = keep.back() = true;
  std::vector<std::pair<size_t, size_t>> spans = {{0, points.size() - 1}};
  while (!spans.empty()) {
    const auto [first, last] = spans.back();
    spans.pop_back();
    if (last <= first + 1) continue;
    double farthest = -1.0;
    size_t at = first;
    for (size_t i = first + 1; i < last; ++i) {
      const double d = DistanceToSegment(points[i], points[first], points[last]);
      if (d > farthest) {
        farthest = d;
        at = i;
      }
    }
    if (farthest <= tolerance) {
      *strayed = std::max(*strayed, farthest);
    } else {
      keep[at] = true;
      spans.push_back({first, at});
      spans.push_back({at, last});
    }
  }
  std::vector<Eigen::VectorXd> kept;
  for (size_t i = 0; i < points.size(); ++i) {
    if (keep[i]) kept.push_back(points[i]);
  }
  return kept;
}

}  // namespace

std::optional<Trajectory> Trajectory::Plan(
    const std::vector<Eigen::VectorXd>& waypoints, const JointLimits& limits,
    const PathTimingOptions& options) {
  if (waypoints.empty()) return std::nullopt;
  const Eigen::Index n = waypoints.front().size();
  if (n == 0 || limits.velocity.size() != n ||
      limits.acceleration.size() != n || limits.jerk.size() != n) {
    return std::nullopt;
  }
  for (const Eigen::VectorXd& q : waypoints) {
    if (q.size() != n || !q.allFinite()) return std::nullopt;
  }
  if (!(options.speed > 0.0 && options.speed <= 1.0) ||
      !(options.max_deviation >= 0.0)) {
    return std::nullopt;
  }
  // Playing the full-speed trajectory slower by `speed` scales its
  // velocity by speed, its acceleration by speed^2 and its jerk by
  // speed^3. Checked after scaling, in case a tiny speed underflowed.
  const double k = options.speed;
  JointLimits joint = limits;
  joint.velocity *= k;
  joint.acceleration *= k * k;
  joint.jerk *= k * k * k;
  if (!AllPositiveAndFinite(joint.velocity) ||
      !AllPositiveAndFinite(joint.acceleration) ||
      !AllPositiveAndFinite(joint.jerk)) {
    return std::nullopt;
  }

  for (size_t w = 1; w < waypoints.size(); ++w) {
    if (!std::isfinite((waypoints[w] - waypoints[w - 1]).norm())) {
      return std::nullopt;
    }
  }
  const std::vector<Eigen::VectorXd> points = WithoutRepeats(waypoints);

  Trajectory trajectory;
  trajectory.waypoints_ = points;
  if (points.size() == 1) return trajectory;

  // One stretch per piece of a shape.
  auto stretches_of = [&](const Shape& shape, double share) {
    std::vector<Stretch> stretches;
    for (size_t p = 0; p < shape.pieces.size(); ++p) {
      Stretch stretch = FromPiece(shape.pieces[p]);
      stretch.stop_after = shape.stop_after[p];
      stretch.limits = StretchLimits(stretch, joint, share);
      stretches.push_back(std::move(stretch));
    }
    return stretches;
  };

  std::optional<Shape> best_shape;
  std::vector<Stretch> best_stretches;
  double best_time = kInfinity;
  auto consider = [&](const Shape& shape, double share) {
    double time = kInfinity;
    std::vector<Stretch> stretches =
        Merge(stretches_of(shape, share), joint, share, &time);
    if (time < best_time) {
      best_time = time;
      best_shape = shape;
      best_stretches = std::move(stretches);
    }
  };

  // Stopping at every corner that turns: what JointMoves one after another
  // do. Kept if nothing else is quicker.
  {
    const Polyline line = Measure(points);
    std::vector<bool> stop(line.length.size(), false);
    for (size_t c = 1; c < stop.size(); ++c) stop[c] = !line.straight_on[c];
    consider(BuildShape(points, line, options.max_deviation, stop),
             kShares[1]);
  }

  // Blended, from the waypoints as given and from fewer of them (see
  // Simplify), the deviation shared between leaving waypoints out and
  // blending what is left.
  for (double part : kSimplify) {
    double strayed = 0.0;
    // (A path that comes back to where it started may simplify to nothing.)
    const std::vector<Eigen::VectorXd> kept =
        part == 0.0 ? points
                    : WithoutRepeats(Simplify(
                          points, part * options.max_deviation, &strayed));
    if (part > 0.0 && (kept.size() == points.size() || kept.size() < 2)) {
      continue;
    }
    const double blending = std::max(0.0, options.max_deviation - strayed);
    const Polyline line = Measure(kept);
    const size_t legs = line.length.size();
    for (double share : kShares) {
      // Which corners to stop at. Blend every corner first; then, corner by
      // corner, stop there instead if that is quicker. A sharp turn can be:
      // its blend is small and tight, and the whole of it must be crossed
      // at the speed its tightest point allows, where a stop slows the arm
      // only close to the corner. Judged with one stretch per piece, before
      // any merging, which is quick.
      std::vector<bool> stop(legs, false);
      Shape shape = BuildShape(kept, line, blending, stop);
      double time = Time(stretches_of(shape, share)).duration;
      for (size_t c = 1; c < legs; ++c) {
        if (line.straight_on[c]) continue;
        stop[c] = true;
        Shape trial = BuildShape(kept, line, blending, stop);
        const double trial_time = Time(stretches_of(trial, share)).duration;
        if (trial_time < time) {
          time = trial_time;
          shape = std::move(trial);
        } else {
          stop[c] = false;
        }
      }
      shape.deviation += strayed;
      consider(shape, share);
    }
  }
  if (!best_shape || !std::isfinite(best_time)) return std::nullopt;

  trajectory.pieces_ = std::move(best_shape->pieces);
  trajectory.length_ = best_shape->length;
  trajectory.deviation_ = best_shape->deviation;
  trajectory.blends_ = best_shape->blends;
  trajectory.stretches_ = static_cast<int>(best_stretches.size());

  // The path-speed profile, segment by segment. Each stretch starts at its
  // own exact position and entry speed rather than wherever the arithmetic
  // of the one before ended, so rounding does not build up.
  std::vector<Segment>& segments = trajectory.segments_;
  std::optional<std::pair<double, double>> restart;  // position, speed
  auto append = [&](double duration, double acceleration, double jerk) {
    if (!(duration > 0.0)) return;
    Segment segment;
    segment.duration = duration;
    if (!segments.empty()) {
      const Segment& last = segments.back();
      segment.start_time = last.start_time + last.duration;
      segment.state = Advance(last.state, last.duration);
    }
    if (restart) {
      segment.state.position = restart->first;
      segment.state.velocity = restart->second;
      restart.reset();
    }
    segment.state.acceleration = acceleration;
    segment.state.jerk = jerk;
    segments.push_back(segment);
  };
  // A speed change as ChangeTime describes it.
  auto change = [&](double v1, double v2, const Limits& limits) {
    const double dv = std::abs(v2 - v1);
    if (dv == 0.0) return;
    const double sign = v2 > v1 ? 1.0 : -1.0;
    const double a = limits.acceleration, j = limits.jerk;
    const double ramp = dv * j <= a * a ? std::sqrt(dv / j) : a / j;
    const double hold = dv * j <= a * a ? 0.0 : dv / a - a / j;
    const double peak = sign * j * ramp;
    append(ramp, 0.0, sign * j);
    append(hold, peak, 0.0);
    append(ramp, peak, -sign * j);
  };

  const Timing timing = Time(best_stretches);
  double start = 0.0;
  for (size_t s = 0; s < best_stretches.size(); ++s) {
    const Stretch& stretch = best_stretches[s];
    const double in = timing.speed[s], out = timing.speed[s + 1];
    const double peak = timing.peak[s];
    restart = std::make_pair(start, in);
    change(in, peak, stretch.limits);
    const double changing = ChangeDistance(in, peak, stretch.limits) +
                            ChangeDistance(peak, out, stretch.limits);
    if (peak > 0.0) append((stretch.length - changing) / peak, 0.0, 0.0);
    change(peak, out, stretch.limits);
    start += stretch.length;
  }
  if (!segments.empty()) {
    trajectory.duration_ =
        segments.back().start_time + segments.back().duration;
  }
  return trajectory;
}

JointSample Trajectory::At(double t) const {
  const Eigen::Index n = waypoints_.front().size();
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
  // The exact ends, not the arithmetic's. Also covers a single waypoint.
  if (t >= duration_) return {waypoints_.back(), zero, zero, zero};
  if (t <= 0.0) return {waypoints_.front(), zero, zero, zero};

  // The last segment that starts at or before t, and the path state there.
  auto segment = std::upper_bound(
      segments_.begin(), segments_.end(), t,
      [](double time, const Segment& s) { return time < s.start_time; });
  if (segment != segments_.begin()) --segment;
  const Sample path = Advance(segment->state, t - segment->start_time);
  const double s = std::clamp(path.position, 0.0, length_);
  const double v = path.velocity, a = path.acceleration, j = path.jerk;

  // The last piece that starts at or before s.
  auto piece = std::upper_bound(
      pieces_.begin(), pieces_.end(), s,
      [](double x, const Piece& p) { return x < p.start; });
  if (piece != pieces_.begin()) --piece;

  // The path's position and its first three derivatives with respect to s
  // (see FromPiece), then the chain rule for time.
  Eigen::VectorXd q, d1, d2, d3;
  if (piece->blend) {
    const double l = piece->half;
    const double x = std::clamp((s - piece->start) / (2.0 * l), 0.0, 1.0);
    const double y = 1.0 - x;
    const double back = l * (1.0 - 2.0 * x + 2.0 * x * x * x - x * x * x * x);
    const double on = l * (1.0 - 2.0 * y + 2.0 * y * y * y - y * y * y * y);
    const double g = x * x * (3.0 - 2.0 * x);
    const Eigen::VectorXd turn = piece->out - piece->in;
    q = piece->point - back * piece->in + on * piece->out;
    d1 = (1.0 - g) * piece->in + g * piece->out;
    d2 = (6.0 * x * y / (2.0 * l)) * turn;
    d3 = ((6.0 - 12.0 * x) / (4.0 * l * l)) * turn;
  } else {
    q = piece->point + (s - piece->start) * piece->in;
    d1 = piece->in;
    d2 = zero;
    d3 = zero;
  }

  JointSample sample;
  sample.position = std::move(q);
  sample.velocity = d1 * v;
  sample.acceleration = d1 * a + d2 * (v * v);
  sample.jerk = d1 * j + d2 * (3.0 * v * a) + d3 * (v * v * v);
  return sample;
}

}  // namespace cell
