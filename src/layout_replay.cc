// See src/layout_plan.h. STUB written with the header; the replay's owner
// replaces it.

#include "src/layout_plan.h"

#include <algorithm>
#include <cmath>

#include "src/path_timing.h"

namespace cell::layout {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::VectorXd ToVector(const Config& c) {
  Eigen::VectorXd v(7);
  for (int j = 0; j < 7; ++j) v[j] = c[j];
  return v;
}

Config ToConfig(const Eigen::VectorXd& v) {
  Config c{};
  for (int j = 0; j < 7 && j < v.size(); ++j) c[j] = v[j];
  return c;
}

double PieceNorm(const Config& a, const Config& b) {
  double s = 0.0;
  for (int j = 0; j < 7; ++j) s += (b[j] - a[j]) * (b[j] - a[j]);
  return std::sqrt(s);
}

// The joint-space length of a path: the sum of its pieces' norms.
double PathLength(const std::vector<Config>& path) {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) length += PieceNorm(path[i - 1], path[i]);
  return length;
}

// The point at `fraction` (0..1) of a path's joint-space length.
Config AlongPath(const std::vector<Config>& path, double fraction) {
  if (path.empty()) return {};
  const double total = PathLength(path);
  if (total <= 0.0 || fraction <= 0.0) return path.front();
  if (fraction >= 1.0) return path.back();
  double left = fraction * total;
  for (size_t i = 1; i < path.size(); ++i) {
    const double piece = PieceNorm(path[i - 1], path[i]);
    if (left <= piece && piece > 0.0) {
      const double a = left / piece;
      Config c{};
      for (int j = 0; j < 7; ++j) {
        c[j] = path[i - 1][j] + a * (path[i][j] - path[i - 1][j]);
      }
      return c;
    }
    left -= piece;
  }
  return path.back();
}

// The leg's law: the fraction of the leg's length at time u of 1.
double LegFraction(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * (3.0 - 2.0 * u);
}

std::optional<Trajectory> TrajectoryOf(const std::vector<Config>& path,
                                       const ArmMotion& motion,
                                       const PlanFigures& f) {
  if (path.empty()) return std::nullopt;
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.reserve(path.size());
  for (const Config& c : path) waypoints.push_back(ToVector(c));
  PathTimingOptions options;
  options.max_deviation = f.max_deviation;
  return Trajectory::Plan(waypoints, motion.limits, options);
}

bool AllEqual(const std::vector<Config>& path) {
  for (const Config& c : path) {
    if (c != path.front()) return false;
  }
  return true;
}

}  // namespace

ArmMotion MotionOf(const LayoutScene& scene, int arm, const PlanFigures& f) {
  const SceneArm& a = scene.arm(arm);
  ArmMotion m;
  m.limits.velocity.resize(7);
  m.limits.acceleration.resize(7);
  m.limits.jerk.resize(7);
  m.weights.resize(7);
  m.lower.resize(7);
  m.upper.resize(7);
  const std::string model = a.use ? a.use->model : std::string();
  static const std::array<double, 6> kKukaWeights = {4.5, 4.2, 3.0, 1.8, 1.8, 1.3};
  static const std::array<double, 6> kUrWeights = {1.8, 1.6, 1.0, 0.6, 0.5, 0.4};
  for (int j = 0; j < 6; ++j) {
    const double v = a.cat->joint_speed_deg_s[j] * kPi / 180.0;
    m.limits.velocity[j] = v;
    m.limits.acceleration[j] = v / f.to_full_speed_s;
    m.limits.jerk[j] = m.limits.acceleration[j] / f.jerk_rise_s;
    if (model == "kr210_r2700_2") {
      m.weights[j] = kKukaWeights[j];
    } else if (model == "ur10e") {
      m.weights[j] = kUrWeights[j];
    } else {
      m.weights[j] = j < 3 ? a.cat->reach_m : 0.5 * a.cat->reach_m;
    }
    m.lower[j] = a.lo[j];
    m.upper[j] = a.hi[j];
  }
  const bool track = a.use && a.use->track.has_value();
  const double tv = track ? a.use->track->speed : 1.0;
  const double ta = track ? a.use->track->accel : tv / f.to_full_speed_s;
  m.limits.velocity[6] = tv;
  m.limits.acceleration[6] = ta;
  m.limits.jerk[6] = ta / f.jerk_rise_s;
  m.weights[6] = 1.0;
  m.lower[6] = track ? a.use->track->lo : 0.0;
  m.upper[6] = track ? a.use->track->hi : 0.0;
  return m;
}

double MoveDuration(const std::vector<Config>& path, const ArmMotion& motion,
                    const PlanFigures& f) {
  if (path.empty()) return -1.0;
  if (AllEqual(path)) return 0.0;
  const std::optional<Trajectory> t = TrajectoryOf(path, motion, f);
  if (!t) return -1.0;
  return t->duration();
}

double LegDuration(const std::vector<Config>& path, double length_m,
                   const ArmMotion& motion, const PlanFigures& f) {
  if (path.empty() || !(length_m >= 0.0)) return -1.0;
  if (length_m == 0.0) return 0.0;
  double t = std::max(1.5 * length_m / f.leg_speed,
                      std::sqrt(6.0 * length_m / f.leg_accel));
  const double total = PathLength(path);
  if (total > 0.0) {
    for (size_t i = 1; i < path.size(); ++i) {
      const double share = PieceNorm(path[i - 1], path[i]) / total;
      if (share <= 0.0) continue;
      for (int j = 0; j < 7; ++j) {
        const double change = std::abs(path[i][j] - path[i - 1][j]);
        t = std::max(t, 1.5 * change / (share * motion.limits.velocity[j]));
      }
    }
  }
  return t;
}

Config ConfigAt(const Segment& segment, const ArmMotion& motion,
                const PlanFigures& f, double t) {
  if (segment.path.empty()) return {};
  if (segment.kind == "move") {
    const std::optional<Trajectory> traj = TrajectoryOf(segment.path, motion, f);
    if (!traj) return t >= segment.t1 ? segment.path.back() : segment.path.front();
    return ToConfig(traj->At(std::max(0.0, t - segment.t0)).position);
  }
  if (segment.kind == "approach" || segment.kind == "retreat") {
    const double span = segment.t1 - segment.t0;
    if (t >= segment.t1) return segment.path.back();
    if (span <= 0.0 || t <= segment.t0) return segment.path.front();
    return AlongPath(segment.path, LegFraction((t - segment.t0) / span));
  }
  return segment.path.front();
}

std::string PlanToJson(const Plan&) { return "{}"; }

std::optional<Plan> PlanFromJson(const std::string&, std::string* error) {
  if (error) *error = "PlanFromJson is not written yet";
  return std::nullopt;
}

std::string Sha256OfFile(const std::string&) { return ""; }

PlanPlayer::PlanPlayer(const LayoutScene& scene, const CheckReport& report,
                       const Plan& plan)
    : scene_(scene), report_(report), plan_(plan) {}

double PlanPlayer::duration() const { return 0.0; }

WorldAt PlanPlayer::At(double) const { return {}; }

void PlanPlayer::SetPoses(const WorldAt&, mjData*) const {}

Built PlanPlayer::BuiltAt(const WorldAt&) const { return {}; }

std::vector<Closest> InstantDistances(const LayoutScene&, const CheckReport&,
                                      const PlanPlayer&, const WorldAt&,
                                      mjData*) {
  return {};
}

VerifyReport VerifyPlan(const Layout&, const ModelPaths&, const Plan&,
                        const std::string&, const VerifyOptions&) {
  VerifyReport r;
  r.faults.push_back("VerifyPlan is not written yet");
  return r;
}

}  // namespace cell::layout
