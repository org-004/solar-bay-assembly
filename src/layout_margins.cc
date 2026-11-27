// The track ends, carriage gaps and torque by bay of a committed plan
// (src/layout_margins.h).

#include "src/layout_margins.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

namespace cell::layout {
namespace {

using Mat3 = std::array<std::array<double, 3>, 3>;

// The base frame's rotation, Rz(yaw) Ry(pitch) Rx(roll), as the model builds it.
Mat3 RotationOf(const Vec3& rpy_deg) {
  const double d = std::numbers::pi / 180.0;
  const double cr = std::cos(rpy_deg[0] * d), sr = std::sin(rpy_deg[0] * d);
  const double cp = std::cos(rpy_deg[1] * d), sp = std::sin(rpy_deg[1] * d);
  const double cy = std::cos(rpy_deg[2] * d), sy = std::sin(rpy_deg[2] * d);
  return {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
           {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
           {-sp, cp * sr, cp * cr}}};
}

Vec3 Unit(const Vec3& v) {
  const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (n <= 0.0) return {0.0, 0.0, 0.0};
  return {v[0] / n, v[1] / n, v[2] / n};
}

double Dot(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// Where a carriage stands on a line of direction `dir` (unit) at track value
// 0: the centre of its box projected on the line, and its half extent along
// the line.
struct OnLine {
  double centre = 0.0;
  double half = 0.0;
};
OnLine CarriageOnLine(const ArmUse& arm, const Vec3& dir) {
  const Track& track = *arm.track;
  const Mat3 r = RotationOf(arm.base_rpy_deg);
  OnLine o;
  o.centre = Dot(arm.base_pos, dir);
  for (int k = 0; k < 3; ++k) {
    // The base frame's axis k in the layout's frame, along the line.
    const double along = r[0][k] * dir[0] + r[1][k] * dir[1] + r[2][k] * dir[2];
    o.centre += along * track.carriage.pos[k];
    o.half += std::abs(along) * track.carriage.size[k] / 2.0;
  }
  return o;
}

}  // namespace

std::vector<std::pair<int, int>> ArmsOnOneLine(const Layout& layout) {
  std::vector<std::pair<int, int>> pairs;
  const int n = static_cast<int>(layout.arms.size());
  for (int i = 0; i < n; ++i) {
    if (!layout.arms[i].track) continue;
    for (int j = i + 1; j < n; ++j) {
      if (!layout.arms[j].track) continue;
      if (BeamArm(layout, i) == BeamArm(layout, j)) pairs.emplace_back(i, j);
    }
  }
  return pairs;
}

TrackMargins TrackMarginsOf(const Layout& layout, const PlanPlayer& player,
                            double sample_s) {
  TrackMargins out;
  // The instants: the grid and every segment's ends.
  std::vector<double> times;
  const double duration = player.duration();
  const long grid = static_cast<long>(std::floor(duration / sample_s));
  for (long k = 0; k <= grid; ++k) times.push_back(static_cast<double>(k) * sample_s);
  times.push_back(duration);
  for (const ArmPlan& a : player.plan().arms) {
    for (const Segment& g : a.segments) {
      times.push_back(g.t0);
      times.push_back(g.t1);
    }
  }
  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end()), times.end());

  const int n = static_cast<int>(layout.arms.size());
  std::vector<int> on_track;
  for (int i = 0; i < n; ++i) {
    if (!layout.arms[i].track) continue;
    on_track.push_back(i);
    for (const char* end : {"lo", "hi"}) {
      TrackEnd e;
      e.arm = layout.arms[i].name;
      e.end = end;
      e.limit = e.end == "lo" ? layout.arms[i].track->lo : layout.arms[i].track->hi;
      e.slack = std::numeric_limits<double>::infinity();
      out.ends.push_back(e);
    }
  }
  struct Pair {
    int i = 0, j = 0;
    OnLine a, b;
  };
  std::vector<Pair> pairs;
  for (const auto& [i, j] : ArmsOnOneLine(layout)) {
    const Vec3 dir = Unit(layout.arms[i].track->direction);
    Pair p{i, j, CarriageOnLine(layout.arms[i], dir), CarriageOnLine(layout.arms[j], dir)};
    pairs.push_back(p);
    CarriageGap g;
    g.a = layout.arms[i].name;
    g.b = layout.arms[j].name;
    g.distance = std::numeric_limits<double>::infinity();
    // The boxes touch when their centres are the two half extents apart.
    g.contact = p.a.half + p.b.half;
    out.gaps.push_back(g);
  }

  for (double t : times) {
    const WorldAt w = player.At(t);
    ++out.samples;
    for (size_t k = 0; k < on_track.size(); ++k) {
      const int i = on_track[k];
      const double s = w.arms[i].config[6];
      for (int side = 0; side < 2; ++side) {
        TrackEnd& e = out.ends[2 * k + side];
        const double slack = side == 0 ? s - e.limit : e.limit - s;
        if (slack < e.slack) {
          e.slack = slack;
          e.value = s;
          e.step = w.arms[i].step;
          e.t = t;
        }
      }
    }
    for (size_t k = 0; k < pairs.size(); ++k) {
      const Pair& p = pairs[k];
      const Vec3 dir = Unit(layout.arms[p.i].track->direction);
      // Each track's value runs along its own direction, which is the
      // line's or its opposite.
      const double sign_j = Dot(dir, Unit(layout.arms[p.j].track->direction)) < 0.0 ? -1.0 : 1.0;
      const double at_i = p.a.centre + w.arms[p.i].config[6];
      const double at_j = p.b.centre + sign_j * w.arms[p.j].config[6];
      const double d = std::abs(at_i - at_j);
      CarriageGap& g = out.gaps[k];
      if (d < g.distance) {
        g.distance = d;
        g.step_a = w.arms[p.i].step;
        g.step_b = w.arms[p.j].step;
        g.t = t;
      }
    }
  }
  std::stable_sort(out.ends.begin(), out.ends.end(),
                   [](const TrackEnd& x, const TrackEnd& y) { return x.slack < y.slack; });
  std::stable_sort(out.gaps.begin(), out.gaps.end(),
                   [](const CarriageGap& x, const CarriageGap& y) {
                     return x.distance - x.contact < y.distance - y.contact;
                   });
  return out;
}

bool TorqueByBay(const Plan& plan, ArmDynamics* dynamics, const ArmMotion& motion,
                 const std::string& arm, double scale, std::vector<BayTorque>* bays,
                 std::string* error) {
  bays->clear();
  for (int b = 0; b < std::max(1, plan.bays); ++b) {
    BayTorque t;
    t.bay = b;
    bays->push_back(t);
  }
  std::map<int, int> bay_of;
  for (const StepTime& s : plan.steps) bay_of[s.number] = s.bay;
  for (const ArmPlan& ap : plan.arms) {
    if (ap.name != arm) continue;
    for (const Segment& s : ap.segments) {
      if (s.kind != "move" || s.path.empty()) continue;
      auto it = bay_of.find(s.step);
      if (it == bay_of.end() || it->second < 0 ||
          it->second >= static_cast<int>(bays->size())) {
        continue;
      }
      TorqueDemand d;
      if (!dynamics->AlongSegment(s, motion, plan.figures, &d, error)) {
        *error = "step " + std::to_string(s.step) + " move: " + *error;
        return false;
      }
      BayTorque& t = (*bays)[it->second];
      ++t.moves;
      if (d.Worst() > 1.0) ++t.over_one;
      if (d.Worst() > scale) ++t.over_scale;
      if (d.Worst() > t.worst.Worst()) {
        t.worst = d;
        t.worst_step = s.step;
      }
    }
  }
  return true;
}

}  // namespace cell::layout
