// The replay half of src/layout_plan.h: how an arm moves on a segment, the
// plan as a file, the player that says where everything is at an instant,
// the distances at an instant and the verifier of a whole plan.

#include "src/layout_plan.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <nlohmann/json.hpp>

#include "src/path_timing.h"
#include "src/rrt.h"

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

// ------------------------------------------------------------------- json

namespace {

std::string Fmt(const char* format, ...) __attribute__((format(printf, 1, 2)));
std::string Fmt(const char* format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// A configuration's number, a time and a figure, each in its fixed form.
std::string G9(double x) { return Fmt("%.9g", x); }
std::string F6(double x) { return Fmt("%.6f", x); }
std::string Fig(double x) { return Fmt("%.15g", x); }
std::string Q(const std::string& s) { return nlohmann::json(s).dump(); }
std::string B(bool b) { return b ? "true" : "false"; }

std::string Ints(const std::vector<int>& v) {
  std::string s = "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i > 0) s += ",";
    s += std::to_string(v[i]);
  }
  return s + "]";
}

std::string ConfigText(const Config& c) {
  std::string s = "[";
  for (int j = 0; j < 7; ++j) {
    if (j > 0) s += ",";
    s += G9(c[j]);
  }
  return s + "]";
}

// A list of lines as a JSON array, one element on each line.
std::string Lines(const std::vector<std::string>& lines) {
  if (lines.empty()) return "[]";
  std::string s = "[\n";
  for (size_t i = 0; i < lines.size(); ++i) {
    s += lines[i];
    s += i + 1 < lines.size() ? ",\n" : "\n";
  }
  return s + "]";
}

std::string SegmentText(const Segment& g) {
  std::string s = "{\"kind\":" + Q(g.kind) + ",\"step\":" + std::to_string(g.step) +
                  ",\"t0\":" + F6(g.t0) + ",\"t1\":" + F6(g.t1) +
                  ",\"tool\":" + Q(g.tool) + ",\"held\":" + Q(g.held) +
                  ",\"from\":" + Q(g.from) + ",\"to\":" + Q(g.to) +
                  ",\"found\":" + B(g.found) +
                  ",\"budget\":" + std::to_string(g.budget) +
                  ",\"queries\":" + std::to_string(g.queries) +
                  ",\"planned_s\":" + F6(g.planned_s) +
                  ",\"model_s\":" + F6(g.model_s) +
                  ",\"seed\":" + std::to_string(g.seed) +
                  ",\"built\":{\"rails\":" + Ints(g.built.rails) +
                  ",\"modules\":" + Ints(g.built.modules) +
                  ",\"on_pallets\":" + std::to_string(g.built.on_pallets) + "}" +
                  ",\"reason\":" + Q(g.reason) + ",\"path\":[";
  for (size_t i = 0; i < g.path.size(); ++i) {
    if (i > 0) s += ",";
    s += ConfigText(g.path[i]);
  }
  return s + "]}";
}

std::string FiguresText(const PlanFigures& f) {
  return "{\"clearance_m\":" + Fig(f.clearance_m) +
         ",\"joint_margin_deg\":" + Fig(f.joint_margin_deg) +
         ",\"plan_pad_m\":" + Fig(f.plan_pad_m) +
         ",\"to_full_speed_s\":" + Fig(f.to_full_speed_s) +
         ",\"jerk_rise_s\":" + Fig(f.jerk_rise_s) +
         ",\"settle_s\":" + Fig(f.settle_s) +
         ",\"leg_speed\":" + Fig(f.leg_speed) +
         ",\"leg_accel\":" + Fig(f.leg_accel) +
         ",\"max_deviation\":" + Fig(f.max_deviation) +
         ",\"edge_resolution\":" + Fig(f.edge_resolution) +
         ",\"shorten_tries\":" + std::to_string(f.shorten_tries) +
         ",\"budgets\":" + Ints(f.budgets) +
         ",\"seed\":" + std::to_string(f.seed) +
         ",\"sample_s\":" + Fig(f.sample_s) + "}";
}

using nlohmann::json;

template <typename T>
T Get(const json& j, const char* key) {
  return j.at(key).get<T>();
}

Config ConfigFrom(const json& j) {
  if (!j.is_array() || j.size() != 7) {
    throw std::runtime_error("a configuration has not seven numbers: " + j.dump());
  }
  Config c{};
  for (int k = 0; k < 7; ++k) c[k] = j[k].get<double>();
  return c;
}

Segment SegmentFrom(const json& j) {
  Segment g;
  g.kind = Get<std::string>(j, "kind");
  g.step = Get<int>(j, "step");
  g.t0 = Get<double>(j, "t0");
  g.t1 = Get<double>(j, "t1");
  g.tool = Get<std::string>(j, "tool");
  g.held = Get<std::string>(j, "held");
  g.from = Get<std::string>(j, "from");
  g.to = Get<std::string>(j, "to");
  g.found = Get<bool>(j, "found");
  g.budget = Get<int>(j, "budget");
  g.queries = Get<std::int64_t>(j, "queries");
  g.planned_s = Get<double>(j, "planned_s");
  g.model_s = Get<double>(j, "model_s");
  g.seed = Get<std::uint32_t>(j, "seed");
  const json& b = j.at("built");
  g.built.rails = Get<std::vector<int>>(b, "rails");
  g.built.modules = Get<std::vector<int>>(b, "modules");
  g.built.on_pallets = Get<int>(b, "on_pallets");
  g.reason = Get<std::string>(j, "reason");
  for (const json& c : j.at("path")) g.path.push_back(ConfigFrom(c));
  return g;
}

}  // namespace

std::string PlanToJson(const Plan& plan) {
  std::string s = "{\n";
  s += "\"layout\": " + Q(plan.layout) + ",\n";
  s += "\"layout_sha256\": " + Q(plan.layout_sha256) + ",\n";
  s += "\"complete\": " + B(plan.complete) + ",\n";
  s += "\"bay_planned_s\": " + F6(plan.bay_planned_s) + ",\n";
  s += "\"bay_model_s\": " + F6(plan.bay_model_s) + ",\n";
  s += "\"figures\": " + FiguresText(plan.figures) + ",\n";
  std::vector<std::string> lines;
  for (const BayEvent& e : plan.events) {
    lines.push_back("{\"kind\":" + Q(e.kind) + ",\"step\":" + std::to_string(e.step) +
                    ",\"t0\":" + F6(e.t0) + ",\"t1\":" + F6(e.t1) + "}");
  }
  s += "\"events\": " + Lines(lines) + ",\n";
  lines.clear();
  for (const StepTime& t : plan.steps) {
    lines.push_back("{\"number\":" + std::to_string(t.number) + ",\"what\":" +
                    Q(t.what) + ",\"arm\":" + Q(t.arm) + ",\"start\":" +
                    F6(t.start) + ",\"end\":" + F6(t.end) + "}");
  }
  s += "\"steps\": " + Lines(lines) + ",\n";
  lines.clear();
  for (const PhaseTime& p : plan.phases) {
    lines.push_back("{\"name\":" + Q(p.name) + ",\"planned_s\":" + F6(p.planned_s) +
                    ",\"model_s\":" + F6(p.model_s) + "}");
  }
  s += "\"phases\": " + Lines(lines) + ",\n";
  lines.clear();
  for (const ArmPlan& a : plan.arms) {
    std::vector<std::string> segments;
    for (const Segment& g : a.segments) segments.push_back(SegmentText(g));
    lines.push_back("{\"name\":" + Q(a.name) + ",\"segments\":" + Lines(segments) +
                    "}");
  }
  s += "\"arms\": " + Lines(lines) + "\n}\n";
  return s;
}

std::optional<Plan> PlanFromJson(const std::string& text, std::string* error) {
  try {
    const json j = json::parse(text);
    Plan p;
    p.layout = Get<std::string>(j, "layout");
    p.layout_sha256 = Get<std::string>(j, "layout_sha256");
    p.complete = Get<bool>(j, "complete");
    p.bay_planned_s = Get<double>(j, "bay_planned_s");
    p.bay_model_s = Get<double>(j, "bay_model_s");
    const json& f = j.at("figures");
    p.figures.clearance_m = Get<double>(f, "clearance_m");
    p.figures.joint_margin_deg = Get<double>(f, "joint_margin_deg");
    // Plans written before the pad was a figure were planned without one.
    p.figures.plan_pad_m = f.contains("plan_pad_m") ? Get<double>(f, "plan_pad_m") : 0.0;
    p.figures.to_full_speed_s = Get<double>(f, "to_full_speed_s");
    p.figures.jerk_rise_s = Get<double>(f, "jerk_rise_s");
    p.figures.settle_s = Get<double>(f, "settle_s");
    p.figures.leg_speed = Get<double>(f, "leg_speed");
    p.figures.leg_accel = Get<double>(f, "leg_accel");
    p.figures.max_deviation = Get<double>(f, "max_deviation");
    p.figures.edge_resolution = Get<double>(f, "edge_resolution");
    p.figures.shorten_tries = Get<int>(f, "shorten_tries");
    p.figures.budgets = Get<std::vector<int>>(f, "budgets");
    p.figures.seed = Get<std::uint32_t>(f, "seed");
    p.figures.sample_s = Get<double>(f, "sample_s");
    for (const json& e : j.at("events")) {
      BayEvent b;
      b.kind = Get<std::string>(e, "kind");
      b.step = Get<int>(e, "step");
      b.t0 = Get<double>(e, "t0");
      b.t1 = Get<double>(e, "t1");
      p.events.push_back(b);
    }
    for (const json& e : j.at("steps")) {
      StepTime t;
      t.number = Get<int>(e, "number");
      t.what = Get<std::string>(e, "what");
      t.arm = Get<std::string>(e, "arm");
      t.start = Get<double>(e, "start");
      t.end = Get<double>(e, "end");
      p.steps.push_back(t);
    }
    for (const json& e : j.at("phases")) {
      PhaseTime t;
      t.name = Get<std::string>(e, "name");
      t.planned_s = Get<double>(e, "planned_s");
      t.model_s = Get<double>(e, "model_s");
      p.phases.push_back(t);
    }
    for (const json& a : j.at("arms")) {
      ArmPlan ap;
      ap.name = Get<std::string>(a, "name");
      for (const json& g : a.at("segments")) ap.segments.push_back(SegmentFrom(g));
      p.arms.push_back(std::move(ap));
    }
    return p;
  } catch (const std::exception& e) {
    if (error) *error = std::string("the plan cannot be read: ") + e.what();
    return std::nullopt;
  }
}

// ---------------------------------------------------------------- sha-256

namespace {

class Sha256 {
 public:
  void Add(const unsigned char* data, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      block_[used_++] = data[i];
      if (used_ == 64) {
        Compress();
        used_ = 0;
      }
    }
    bits_ += static_cast<std::uint64_t>(n) * 8;
  }
  std::string Hex() {
    const std::uint64_t bits = bits_;
    const unsigned char one = 0x80, zero = 0;
    Add(&one, 1);
    while (used_ != 56) Add(&zero, 1);
    unsigned char length[8];
    for (int i = 0; i < 8; ++i) length[i] = static_cast<unsigned char>(bits >> (56 - 8 * i));
    Add(length, 8);
    std::string s;
    for (std::uint32_t v : h_) s += Fmt("%08x", v);
    return s;
  }

 private:
  static std::uint32_t Rotr(std::uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }
  void Compress() {
    static const std::uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1,
        0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
        0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786,
        0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147,
        0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
        0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a,
        0x5b9cca4f, 0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (std::uint32_t(block_[4 * i]) << 24) | (std::uint32_t(block_[4 * i + 1]) << 16) |
             (std::uint32_t(block_[4 * i + 2]) << 8) | std::uint32_t(block_[4 * i + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const std::uint32_t s0 = Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const std::uint32_t s1 = Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3], e = h_[4], f = h_[5],
                  g = h_[6], h = h_[7];
    for (int i = 0; i < 64; ++i) {
      const std::uint32_t s1 = Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25);
      const std::uint32_t ch = (e & f) ^ (~e & g);
      const std::uint32_t t1 = h + s1 + ch + k[i] + w[i];
      const std::uint32_t s0 = Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = s0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + t1;
      d = c;
      c = b;
      b = a;
      a = t1 + t2;
    }
    h_[0] += a; h_[1] += b; h_[2] += c; h_[3] += d;
    h_[4] += e; h_[5] += f; h_[6] += g; h_[7] += h;
  }

  std::uint32_t h_[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                         0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  unsigned char block_[64] = {};
  size_t used_ = 0;
  std::uint64_t bits_ = 0;
};

}  // namespace

std::string Sha256OfFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return "";
  Sha256 sha;
  char buffer[65536];
  while (in) {
    in.read(buffer, sizeof(buffer));
    const std::streamsize n = in.gcount();
    if (n > 0) sha.Add(reinterpret_cast<const unsigned char*>(buffer), static_cast<size_t>(n));
  }
  if (in.bad()) return "";
  return sha.Hex();
}

// ----------------------------------------------------------------- player

struct PlanPlayer::Cache {
  std::vector<int> plan_arm;  // For each arm of the scene, its ArmPlan or -1.
  // For each ArmPlan, one trajectory per segment (set for moves only).
  std::vector<std::vector<std::optional<Trajectory>>> trajectories;
  std::map<int, const BayStep*> steps;
  // When each part is set or taken: the end of its work segment.
  std::vector<std::pair<double, int>> rail_set, module_set, module_take;
  std::vector<double> magazines;
  const BayEvent* tube_in = nullptr;
  const BayEvent* bay_out = nullptr;
  int takes = 0;
  double duration = 0.0;
};

namespace {

// A point at `u` (0..1) of a polyline of offsets, at constant speed.
Eigen::Vector3d AlongWay(const std::vector<Vec3>& path, double u) {
  if (path.empty()) return Eigen::Vector3d::Zero();
  auto at = [&](size_t i) { return Eigen::Vector3d(path[i][0], path[i][1], path[i][2]); };
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) total += (at(i) - at(i - 1)).norm();
  u = std::clamp(u, 0.0, 1.0);
  if (total <= 0.0) return at(path.size() - 1);
  double left = u * total;
  for (size_t i = 1; i < path.size(); ++i) {
    const double piece = (at(i) - at(i - 1)).norm();
    if (left <= piece && piece > 0.0) return at(i - 1) + (at(i) - at(i - 1)) * (left / piece);
    left -= piece;
  }
  return at(path.size() - 1);
}

Config ConfigOn(const Segment& segment, const ArmMotion& motion,
                const PlanFigures& f, double t, const Trajectory* trajectory) {
  if (segment.path.empty()) return {};
  if (segment.kind == "move") {
    if (trajectory == nullptr) {
      return t >= segment.t1 ? segment.path.back() : segment.path.front();
    }
    return ToConfig(trajectory->At(std::max(0.0, t - segment.t0)).position);
  }
  return ConfigAt(segment, motion, f, t);
}

}  // namespace

PlanPlayer::PlanPlayer(const LayoutScene& scene, const CheckReport& report,
                       const Plan& plan)
    : scene_(scene), report_(report), plan_(plan) {
  auto cache = std::make_shared<Cache>();
  for (int i = 0; i < scene.ArmCount(); ++i) {
    motions_.push_back(MotionOf(scene, i, plan.figures));
    int found = -1;
    for (size_t k = 0; k < plan.arms.size(); ++k) {
      if (plan.arms[k].name == scene.arm(i).use->name) found = static_cast<int>(k);
    }
    cache->plan_arm.push_back(found);
  }
  for (const BayStep& s : report.steps) cache->steps[s.number] = &s;
  cache->trajectories.resize(plan.arms.size());
  for (size_t k = 0; k < plan.arms.size(); ++k) {
    const ArmPlan& ap = plan.arms[k];
    const int arm = scene.ArmIndex(ap.name);
    auto& list = cache->trajectories[k];
    list.resize(ap.segments.size());
    for (size_t g = 0; g < ap.segments.size(); ++g) {
      const Segment& seg = ap.segments[g];
      cache->duration = std::max(cache->duration, seg.t1);
      if (seg.kind == "move" && arm >= 0) {
        list[g] = TrajectoryOf(seg.path, motions_[arm], plan.figures);
      }
      if (seg.kind != "work") continue;
      auto it = cache->steps.find(seg.step);
      if (it == cache->steps.end()) continue;
      const BayStep& s = *it->second;
      if (s.kind == "rail_set") cache->rail_set.push_back({seg.t1, s.index});
      if (s.kind == "module_set") cache->module_set.push_back({seg.t1, s.index});
      if (s.kind == "module_take") cache->module_take.push_back({seg.t1, s.index});
      if (s.kind == "pins_take") cache->magazines.push_back(seg.t1);
    }
  }
  for (const BayEvent& e : plan.events) {
    if (e.kind == "tube_in") cache->tube_in = &e;
    if (e.kind == "bay_out") cache->bay_out = &e;
    cache->duration = std::max(cache->duration, e.t1);
  }
  cache->takes = PointCount(scene.layout(), "module_take");
  cache_ = cache;
}

double PlanPlayer::duration() const { return cache_->duration; }

WorldAt PlanPlayer::At(double t) const {
  const Cache& c = *cache_;
  WorldAt w;
  w.t = t;
  for (int i = 0; i < scene_.ArmCount(); ++i) {
    const SceneArm& sa = scene_.arm(i);
    ArmAt a;
    const int k = c.plan_arm[i];
    if (k < 0 || plan_.arms[k].segments.empty()) {
      for (int j = 0; j < 6; ++j) a.config[j] = sa.wait[j];
      a.config[6] = sa.wait_track;
      a.tool = sa.use->first_tool;
      w.arms.push_back(a);
      continue;
    }
    const std::vector<Segment>& segs = plan_.arms[k].segments;
    auto it = std::upper_bound(segs.begin(), segs.end(), t,
                               [](double v, const Segment& s) { return v < s.t0; });
    int g = static_cast<int>(it - segs.begin()) - 1;
    g = std::clamp(g, 0, static_cast<int>(segs.size()) - 1);
    const Segment& seg = segs[g];
    const std::optional<Trajectory>& traj = c.trajectories[k][g];
    a.config = ConfigOn(seg, motions_[i], plan_.figures, t, traj ? &*traj : nullptr);
    a.tool = seg.tool;
    a.held = seg.held;
    a.segment = g;
    a.kind = seg.kind;
    a.step = seg.step;
    w.arms.push_back(a);
  }
  for (const auto& [time, k] : c.rail_set) if (t >= time) w.rails.push_back(k);
  for (const auto& [time, k] : c.module_set) if (t >= time) w.modules.push_back(k);
  for (const auto& [time, k] : c.module_take) if (t >= time) w.pallet_taken.push_back(k);
  for (double time : c.magazines) if (t >= time) ++w.magazines_taken;
  std::sort(w.rails.begin(), w.rails.end());
  std::sort(w.modules.begin(), w.modules.end());
  std::sort(w.pallet_taken.begin(), w.pallet_taken.end());
  w.tube = c.tube_in == nullptr || t >= c.tube_in->t0;
  for (const BayEvent& e : plan_.events) {
    if (t >= e.t0 && t < e.t1) w.event = e.kind;
  }
  const Layout& L = scene_.layout();
  if (c.tube_in != nullptr && t >= c.tube_in->t0 && t < c.tube_in->t1) {
    const double span = c.tube_in->t1 - c.tube_in->t0;
    w.way_offset = AlongWay(L.tube_in.path, span > 0.0 ? (t - c.tube_in->t0) / span : 1.0);
  } else if (c.bay_out != nullptr && t >= c.bay_out->t0) {
    // The bay stays where its way ends once it is out.
    w.event = "bay_out";
    const double span = c.bay_out->t1 - c.bay_out->t0;
    w.way_offset = AlongWay(L.bay_out.path, span > 0.0 ? (t - c.bay_out->t0) / span : 1.0);
  }
  return w;
}

void PlanPlayer::SetPoses(const WorldAt& w, mjData* d) const {
  for (int i = 0; i < scene_.ArmCount() && i < static_cast<int>(w.arms.size()); ++i) {
    ArmPose p;
    for (int j = 0; j < 6; ++j) p.q[j] = w.arms[i].config[j];
    p.track = w.arms[i].config[6];
    scene_.SetPose(d, i, p);
  }
}

Built PlanPlayer::BuiltAt(const WorldAt& w) const {
  Built b;
  b.rails = w.rails;
  b.modules = w.modules;
  b.on_pallets = std::max(0, cache_->takes - static_cast<int>(w.pallet_taken.size()));
  return b;
}

// -------------------------------------------------------------- distances

namespace {

void Merge(DesignPairs* into, const DesignPairs& p) {
  into->held_skip.insert(p.held_skip.begin(), p.held_skip.end());
  into->held_near.insert(p.held_near.begin(), p.held_near.end());
  into->last_near.insert(p.last_near.begin(), p.last_near.end());
  into->tool_near.insert(p.tool_near.begin(), p.tool_near.end());
}

// Measures the cell at instants, keeping the moving sets and the obstacle
// lists while nothing they depend on changes.
class Measurer {
 public:
  Measurer(const LayoutScene& scene, const CheckReport& report,
           const PlanPlayer& player)
      : scene_(scene), player_(player), arms_(scene.ArmCount()) {
    for (const BayStep& s : report.steps) steps_[s.number] = &s;
    const LayoutHandles& H = scene.handles();
    way_geoms_.insert(H.tube_geoms.begin(), H.tube_geoms.end());
    for (const auto& r : H.rail_geoms) way_geoms_.insert(r.begin(), r.end());
    for (const auto& m : H.module_geoms) way_geoms_.insert(m.begin(), m.end());
  }

  std::vector<Closest> Measure(const WorldAt& w, mjData* d, std::int64_t* queries) {
    player_.SetPoses(w, d);
    mj_kinematics(&scene_.model(), d);
    const int n = scene_.ArmCount();
    std::vector<Closest> out;
    const Built built = player_.BuiltAt(w);
    const bool way = w.event == "tube_in" || w.event == "bay_out";
    for (int i = 0; i < n; ++i) {
      const ArmAt& a = w.arms[i];
      ArmCache& c = arms_[i];
      std::string key = std::to_string(a.segment) + "|" + a.tool + "|" + a.held + "|" +
                        std::to_string(a.step) + "|" + a.kind + "|" +
                        std::to_string(built.rails.size()) + "|" +
                        std::to_string(built.modules.size()) + "|" +
                        std::to_string(built.on_pallets) + "|" + (way ? "w" : "");
      ObstacleOptions options;
      options.other_arms = false;
      for (int k = 0; k < n; ++k) {
        if (k == i) continue;
        options.tools[k] = w.arms[k].tool;
        key += "|" + w.arms[k].tool;
      }
      if (key != c.key) {
        c.key = key;
        c.moving = scene_.Moving(i, a.tool, a.held);
        c.obstacles = scene_.Obstacles(i, a.tool, built, PairsOf(a), options);
        if (way) {
          std::vector<Obstacle> kept;
          for (const Obstacle& o : c.obstacles) {
            if (!way_geoms_.count(o.geom)) kept.push_back(o);
          }
          c.obstacles = kept;
        }
        c.plain = c.obstacles;
        for (Obstacle& o : c.plain) o.near = 0;
        c.as_obstacle.clear();
        for (const MovingGeom& g : c.moving) c.as_obstacle.push_back({g.geom, 0u, 0u});
      }
    }
    for (int i = 0; i < n; ++i) {
      ArmCache& c = arms_[i];
      Nearest nearest = scene_.LeastDistance(d, c.moving, c.obstacles);
      ++*queries;
      if (nearest.near) {
        out.push_back(ClosestOf(nearest, w, i, -1, true));
        nearest = scene_.LeastDistance(d, c.moving, c.plain);
        ++*queries;
      }
      out.push_back(ClosestOf(nearest, w, i, -1, false));
    }
    for (int i = 0; i < n; ++i) {
      for (int k = i + 1; k < n; ++k) {
        const Nearest nearest =
            scene_.LeastDistance(d, arms_[i].moving, arms_[k].as_obstacle);
        ++*queries;
        out.push_back(ClosestOf(nearest, w, i, k, false));
      }
    }
    return out;
  }

 private:
  struct ArmCache {
    std::string key = "none";
    std::vector<MovingGeom> moving;
    std::vector<Obstacle> obstacles, plain, as_obstacle;
  };

  Closest ClosestOf(const Nearest& nearest, const WorldAt& w, int arm, int other,
                    bool by_design) const {
    Closest c;
    c.distance = nearest.distance;
    c.t = w.t;
    c.step = w.arms[arm].step;
    c.arm = scene_.arm(arm).use->name;
    c.a = scene_.GeomName(nearest.moving);
    c.b = scene_.GeomName(nearest.obstacle);
    if (other >= 0) c.other_arm = scene_.arm(other).use->name;
    c.by_design = by_design;
    return c;
  }

  DesignPairs PairsOf(const ArmAt& a) {
    DesignPairs pairs;
    if (a.kind != "approach" && a.kind != "retreat" && a.kind != "work") return pairs;
    auto it = steps_.find(a.step);
    if (it == steps_.end()) return pairs;
    const BayStep& s = *it->second;
    if (s.index < 0) return pairs;
    auto pit = points_.find(s.kind);
    if (pit == points_.end()) {
      pit = points_.emplace(s.kind, WorkPoints(scene_.layout(), s.kind)).first;
    }
    if (s.index >= static_cast<int>(pit->second.size())) return pairs;
    const WorkPoint& p = pit->second[s.index];
    if (a.kind == "approach") return scene_.AtWorkPoint(p, Leg::kApproach).pairs;
    if (a.kind == "retreat") return scene_.AtWorkPoint(p, Leg::kRetreat).pairs;
    for (Leg leg : {Leg::kWork, Leg::kApproach, Leg::kRetreat}) {
      Merge(&pairs, scene_.AtWorkPoint(p, leg).pairs);
    }
    return pairs;
  }

  const LayoutScene& scene_;
  const PlanPlayer& player_;
  std::vector<ArmCache> arms_;
  std::map<int, const BayStep*> steps_;
  std::map<std::string, std::vector<WorkPoint>> points_;
  std::set<int> way_geoms_;
};

}  // namespace

std::vector<Closest> InstantDistances(const LayoutScene& scene,
                                      const CheckReport& report,
                                      const PlanPlayer& player,
                                      const WorldAt& w, mjData* d) {
  Measurer m(scene, report, player);
  std::int64_t queries = 0;
  return m.Measure(w, d, &queries);
}

// ----------------------------------------------------------------- verify

namespace {

constexpr double kSameConfig = 1e-6;
constexpr double kInRange = 1e-9;

std::string StepName(const BayStep* s, int number) {
  if (s == nullptr) return Fmt("step %d", number);
  return Fmt("step %d (%s)", number, s->what.c_str());
}

double MaxDiff(const Config& a, const Config& b) {
  double m = 0.0;
  for (int j = 0; j < 7; ++j) m = std::max(m, std::abs(a[j] - b[j]));
  return m;
}

Config FromDeg(const std::array<double, 6>& deg, double track) {
  Config c{};
  for (int j = 0; j < 6; ++j) c[j] = deg[j] * kPi / 180.0;
  c[6] = track;
  return c;
}

Config FromRad(const std::array<double, 6>& rad, double track) {
  Config c{};
  for (int j = 0; j < 6; ++j) c[j] = rad[j];
  c[6] = track;
  return c;
}

Config SegmentStart(const Segment& s) { return s.path.front(); }
Config SegmentEnd(const Segment& s) {
  if (s.kind == "move" || s.kind == "approach" || s.kind == "retreat") return s.path.back();
  return s.path.front();
}

// A pose the steps give, to compare a stored configuration with.
struct PoseRef {
  std::string what;           // "the approach pose of module_set 3"
  Config c{};
  bool joints_only = false;   // The tool-change pose: the track is the stored one's.
  bool wait = false;          // Compared in joints and track.
  bool set = false;
};

class Verifier {
 public:
  Verifier(const Layout& layout, const Plan& plan, const CheckReport& report,
           const LayoutScene& scene, const PlanPlayer& player, const VerifyOptions& options,
           VerifyReport* r)
      : L_(layout), plan_(plan), report_(report), scene_(scene), player_(player), o_(options),
        r_(r),
        f_(plan.figures), data_(scene.MakeData()) {
    for (const BayStep& s : report.steps) steps_[s.number] = &s;
    for (const SolvedPoint& p : report.points) points_[{p.kind, p.index}] = &p;
    for (const BayEvent& e : plan.events) events_[e.kind] = &e;
  }

  void Fault(const std::string& s) { r_->faults.push_back(s); }

  void Run() {
    Events();
    for (int i = 0; i < scene_.ArmCount(); ++i) CheckArm(i);
    StepsAndPhases();
    Order();
  }

  double bay_end() const { return bay_end_; }

 private:
  const BayStep* Step(int number) const {
    auto it = steps_.find(number);
    return it == steps_.end() ? nullptr : it->second;
  }
  const SolvedPoint* Point(const BayStep& s) const {
    auto it = points_.find({s.kind, s.index});
    return it == points_.end() ? nullptr : it->second;
  }
  std::string Arm(int i) const { return scene_.arm(i).use->name; }

  void Events() {
    const std::vector<std::pair<std::string, double>> want = {
        {"tube_in", L_.tube_in.time_s},
        {"clamp", L_.holder.clamp_s},
        {"release", L_.holder.release_s},
        {"bay_out", L_.bay_out.time_s}};
    for (const auto& [kind, seconds] : want) {
      auto it = events_.find(kind);
      if (it == events_.end()) {
        Fault("the plan has no event " + kind);
        continue;
      }
      const BayEvent& e = *it->second;
      if (std::abs((e.t1 - e.t0) - seconds) > o_.time_tolerance_s) {
        Fault(Fmt("the event %s takes %.3f s from %.3f s, the layout gives %.3f s",
                  kind.c_str(), e.t1 - e.t0, e.t0, seconds));
      }
      const BayStep* s = Step(e.step);
      if (s == nullptr || s->kind != kind) {
        Fault(Fmt("the event %s names step %d, which is not the bay's %s step",
                  kind.c_str(), e.step, kind.c_str()));
      }
    }
    if (events_.count("tube_in") && std::abs(events_["tube_in"]->t0) > o_.time_tolerance_s) {
      Fault(Fmt("tube_in starts at %.3f s, not at 0", events_["tube_in"]->t0));
    }
    auto follows = [&](const char* a, const char* b) {
      if (!events_.count(a) || !events_.count(b)) return;
      if (events_[b]->t0 < events_[a]->t1 - o_.time_tolerance_s) {
        Fault(Fmt("%s starts at %.3f s, before %s ends at %.3f s", b, events_[b]->t0, a,
                  events_[a]->t1));
      }
    };
    follows("tube_in", "clamp");
    follows("clamp", "release");
    follows("release", "bay_out");
    bay_end_ = 0.0;
    if (events_.count("bay_out")) {
      bay_end_ = events_["bay_out"]->t1;
    } else {
      for (const ArmPlan& a : plan_.arms) {
        if (!a.segments.empty()) bay_end_ = std::max(bay_end_, a.segments.back().t1);
      }
    }
  }

  // Forward kinematics of the arm's working point with `tool` at `c`.
  void Frame(int arm, const std::string& tool, const Config& c, Eigen::Vector3d* pos,
             Eigen::Matrix3d* rot) {
    const SceneArm& a = scene_.arm(arm);
    ArmPose p;
    for (int j = 0; j < 6; ++j) p.q[j] = c[j];
    p.track = c[6];
    scene_.SetPose(data_.get(), arm, p);
    mj_kinematics(&scene_.model(), data_.get());
    int site = a.h->flange_site;
    if (!tool.empty()) {
      auto it = a.h->work_site.find(tool);
      if (it != a.h->work_site.end()) site = it->second;
    }
    const mjData* d = data_.get();
    *pos = Eigen::Vector3d(d->site_xpos[3 * site], d->site_xpos[3 * site + 1],
                           d->site_xpos[3 * site + 2]);
    for (int r = 0; r < 3; ++r) {
      for (int k = 0; k < 3; ++k) (*rot)(r, k) = d->site_xmat[9 * site + 3 * r + k];
    }
  }

  // Empty if `c` is at the pose, else how far off it is.
  std::string Off(int arm, const std::string& tool, const Config& c, const PoseRef& ref) {
    if (ref.wait) {
      double deg = 0.0;
      for (int j = 0; j < 6; ++j) deg = std::max(deg, std::abs(c[j] - ref.c[j]) * 180.0 / kPi);
      const double m = std::abs(c[6] - ref.c[6]);
      if (deg <= o_.pose_tolerance_deg && m <= o_.pose_tolerance_m) return "";
      return Fmt("a joint %.3f deg and the track %.4f m off %s", deg, m, ref.what.c_str());
    }
    Config want = ref.c;
    if (ref.joints_only) want[6] = c[6];
    Eigen::Vector3d p1, p2;
    Eigen::Matrix3d r1, r2;
    Frame(arm, tool, c, &p1, &r1);
    Frame(arm, tool, want, &p2, &r2);
    const double mm = (p1 - p2).norm();
    double deg = 0.0;
    for (int k = 0; k < 3; ++k) {
      const double dot = std::clamp(r1.col(k).dot(r2.col(k)), -1.0, 1.0);
      deg = std::max(deg, std::acos(dot) * 180.0 / kPi);
    }
    if (mm <= o_.pose_tolerance_m && deg <= o_.pose_tolerance_deg) return "";
    return Fmt("the working point %.2f mm and the axes %.3f deg off %s", mm * 1000.0,
               deg, ref.what.c_str());
  }

  void CheckPose(int arm, const Segment& s, const Config& c, const PoseRef& ref,
                 const char* end) {
    if (!ref.set) return;
    const std::string off = Off(arm, s.tool, c, ref);
    if (!off.empty()) {
      Fault(Fmt("arm %s, %s, the %s %s at %.3f s: %s", Arm(arm).c_str(),
                StepName(Step(s.step), s.step).c_str(), s.kind.c_str(), end,
                std::strcmp(end, "ends") == 0 ? s.t1 : s.t0, off.c_str()));
    }
  }

  PoseRef WaitPose(int arm) const {
    PoseRef p;
    p.what = "the wait pose";
    p.c = FromRad(scene_.arm(arm).wait, scene_.arm(arm).wait_track);
    p.wait = true;
    p.set = true;
    return p;
  }
  PoseRef ToolChangePose(int arm) const {
    PoseRef p;
    for (const SolvedToolChange& tc : report_.tool_changes) {
      if (tc.arm == Arm(arm)) {
        p.what = "the tool-change pose";
        p.c = FromDeg(tc.deg, 0.0);
        p.joints_only = true;
        p.set = true;
      }
    }
    return p;
  }
  static PoseRef PointPose(const SolvedPoint* p, const char* which) {
    PoseRef r;
    if (p == nullptr) return r;
    const std::string w = which;
    const std::array<double, 6>& deg =
        w == "work" ? p->work_deg : w == "approach" ? p->approach_deg : p->retreat_deg;
    r.what = "the " + w + " pose of " + p->item;
    r.c = FromDeg(deg, p->track);
    r.set = true;
    return r;
  }

  void CheckArm(int i) {
    const std::string name = Arm(i);
    const ArmPlan* ap = nullptr;
    for (const ArmPlan& a : plan_.arms) {
      if (a.name == name) ap = &a;
    }
    if (ap == nullptr) {
      Fault("arm " + name + " has no plan");
      return;
    }
    const std::vector<Segment>& segs = ap->segments;
    if (segs.empty()) {
      Fault("arm " + name + " has no segments");
      return;
    }
    const ArmMotion motion = MotionOf(scene_, i, f_);
    const double tol = o_.time_tolerance_s;
    // Coverage and continuity.
    for (size_t g = 0; g < segs.size(); ++g) {
      const Segment& s = segs[g];
      const std::string where =
          Fmt("arm %s, %s, its %s from %.3f s", name.c_str(),
              StepName(Step(s.step), s.step).c_str(), s.kind.c_str(), s.t0);
      if (s.path.empty()) {
        Fault(where + " has no configuration");
        continue;
      }
      if (s.t1 < s.t0 - tol) Fault(where + Fmt(" ends before it starts (%.3f s)", s.t1));
      if (g == 0 && std::abs(s.t0) > tol) Fault(where + " is the first and does not start at 0");
      if (g > 0) {
        const Segment& p = segs[g - 1];
        if (s.t0 > p.t1 + tol) {
          Fault(where + Fmt(" leaves a gap of %.3f s after the one before, which ends at %.3f s",
                            s.t0 - p.t1, p.t1));
        } else if (s.t0 < p.t1 - tol) {
          Fault(where + Fmt(" overlaps the one before, which ends at %.3f s", p.t1));
        }
        if (!p.path.empty() && MaxDiff(SegmentEnd(p), SegmentStart(s)) > kSameConfig) {
          Fault(where + Fmt(" does not start where the one before ended (%.6f off)",
                            MaxDiff(SegmentEnd(p), SegmentStart(s))));
        }
      }
      if (s.kind == "wait") {
        if (s.reason.empty()) Fault(where + " is a wait with no reason");
        // R21: an arm waits at its wait pose, where its last retreat ended,
        // or at the tool-change pose where it has just changed its tool.
        int before = static_cast<int>(g) - 1;
        while (before >= 0 && segs[before].kind == "wait") --before;
        const bool after_retreat = before >= 0 && segs[before].kind == "retreat" &&
                                   !segs[before].path.empty() &&
                                   MaxDiff(segs[before].path.back(), s.path.front()) <= kSameConfig;
        const BayStep* by = before >= 0 ? Step(segs[before].step) : nullptr;
        const bool after_tool_change = before >= 0 && segs[before].kind == "work" &&
                                       by != nullptr && by->kind == "tool_change" &&
                                       !segs[before].path.empty() &&
                                       MaxDiff(segs[before].path.back(), s.path.front()) <= kSameConfig;
        if (!after_retreat && !after_tool_change &&
            !Off(i, s.tool, s.path.front(), WaitPose(i)).empty()) {
          Fault(where + " stands neither at the arm's wait pose, where a retreat just before "
                        "it ended, nor where a tool change just before it ended");
        }
        if (s.t1 - s.t0 > tol) {
          ++r_->waits;
          r_->wait_s += s.t1 - s.t0;
        }
      } else if (s.kind != "move" && s.kind != "approach" && s.kind != "work" &&
                 s.kind != "retreat") {
        Fault(where + " is of no known kind");
      }
      if (s.kind == "move") MoveBuilt(s, where);
      if (s.kind == "move" && o_.edges) Edges(i, s, motion, where);
      if (s.kind == "approach" || s.kind == "retreat") Leg(i, s, where);
    }
    if (std::abs(segs.back().t1 - bay_end_) > tol) {
      Fault(Fmt("arm %s ends at %.3f s, the bay at %.3f s", name.c_str(), segs.back().t1,
                bay_end_));
    }
    // The arm at its wait pose during tube_in, clamp and bay_out.
    for (const char* kind : {"tube_in", "clamp", "bay_out"}) {
      auto it = events_.find(kind);
      if (it == events_.end()) continue;
      const BayEvent& e = *it->second;
      for (const Segment& s : segs) {
        if (s.path.empty() || s.t1 <= e.t0 + tol || s.t0 >= e.t1 - tol) continue;
        const bool still = s.kind == "wait";
        const std::string off = Off(i, s.tool, s.path.front(), WaitPose(i));
        const bool tool_ok = std::string(kind) != "bay_out" ||
                             (s.tool == scene_.arm(i).use->first_tool && s.held.empty());
        if (!still || !off.empty() || !tool_ok) {
          Fault(Fmt("arm %s is not at its wait pose%s during %s (%.3f to %.3f s): its %s "
                    "of %s from %.3f s%s",
                    name.c_str(), std::string(kind) == "bay_out" ? " with its first tool" : "",
                    kind, e.t0, e.t1, s.kind.c_str(),
                    StepName(Step(s.step), s.step).c_str(), s.t0,
                    off.empty() ? (tool_ok ? "" : (", with tool \"" + s.tool + "\" holding \"" +
                                                   s.held + "\"").c_str())
                                : (": " + off).c_str()));
          break;
        }
      }
    }
    Sequence(i, segs, motion);
  }

  // The world a move was planned in holds at least the bay as it is while
  // the move runs: every rail and module there at its end, and at least the
  // pallet modules there at its start.
  void MoveBuilt(const Segment& s, const std::string& where) {
    const Built at_end = player_.BuiltAt(player_.At(s.t1));
    const Built at_start = player_.BuiltAt(player_.At(s.t0));
    std::string missing;
    for (int k : at_end.rails) {
      if (std::find(s.built.rails.begin(), s.built.rails.end(), k) == s.built.rails.end()) {
        missing += Fmt(" rail %d", k);
      }
    }
    for (int k : at_end.modules) {
      if (std::find(s.built.modules.begin(), s.built.modules.end(), k) == s.built.modules.end()) {
        missing += Fmt(" module %d", k);
      }
    }
    if (s.built.on_pallets < at_start.on_pallets) {
      missing += Fmt(" %d pallet modules (it has %d)", at_start.on_pallets, s.built.on_pallets);
    }
    if (!missing.empty()) {
      Fault(where + " was planned in a bay without what is there while it runs:" + missing);
    }
  }

  // A leg's samples: on the straight line between its ends, the tool's axes
  // those of the work pose, at most 5.5 mm and 10 degrees of any joint apart.
  void Leg(int arm, const Segment& s, const std::string& where) {
    const size_t n = s.path.size();
    if (n < 2) return;
    std::vector<Eigen::Vector3d> pos(n);
    std::vector<Eigen::Matrix3d> rot(n);
    for (size_t k = 0; k < n; ++k) Frame(arm, s.tool, s.path[k], &pos[k], &rot[k]);
    const Eigen::Matrix3d& work = s.kind == "approach" ? rot[n - 1] : rot[0];
    const Eigen::Vector3d a = pos[0], b = pos[n - 1];
    const double length = (b - a).norm();
    for (size_t k = 0; k < n; ++k) {
      double off = (pos[k] - a).norm();
      if (length > 0.0) {
        const double u = std::clamp((pos[k] - a).dot(b - a) / (length * length), 0.0, 1.0);
        off = (pos[k] - (a + u * (b - a))).norm();
      }
      double deg = 0.0;
      for (int c = 0; c < 3; ++c) {
        deg = std::max(deg, std::acos(std::clamp(rot[k].col(c).dot(work.col(c)), -1.0, 1.0)) *
                                180.0 / kPi);
      }
      std::string why;
      if (off > o_.pose_tolerance_m) {
        why = Fmt("its working point is %.2f mm off the leg's straight line", off * 1000.0);
      } else if (deg > o_.pose_tolerance_deg) {
        why = Fmt("its tool's axes are %.3f deg off the work pose's", deg);
      } else if (k > 0 && (pos[k] - pos[k - 1]).norm() > 0.0055) {
        why = Fmt("its working point is %.2f mm from the sample before",
                  (pos[k] - pos[k - 1]).norm() * 1000.0);
      } else if (k > 0) {
        for (int j = 0; j < 6 && why.empty(); ++j) {
          const double turn = std::abs(s.path[k][j] - s.path[k - 1][j]) * 180.0 / kPi;
          if (turn > 10.0) why = Fmt("joint %d turns %.2f deg from the sample before", j + 1, turn);
        }
      }
      if (!why.empty()) {
        Fault(where + Fmt(": at sample %zu of %zu ", k, n) + why);
        return;
      }
    }
  }

  void Edges(int arm, const Segment& s, const ArmMotion& motion, const std::string& where) {
    for (size_t k = 0; k < s.path.size(); ++k) {
      for (int j = 0; j < 7; ++j) {
        if (s.path[k][j] < motion.lower[j] - kInRange ||
            s.path[k][j] > motion.upper[j] + kInRange) {
          Fault(where + Fmt(": waypoint %zu has %s %.6f outside %.6f .. %.6f", k,
                            j < 6 ? ("joint " + std::to_string(j + 1)).c_str() : "the track",
                            s.path[k][j], motion.lower[j], motion.upper[j]));
          return;
        }
      }
    }
    const MoveWorld world(scene_, arm, s.tool, s.held, s.built,
                          f_.clearance_m - o_.tolerance_m);
    mjData* d = data_.get();
    // As the planner: the other arms stand at their wait poses, whatever
    // this data was used for before.
    scene_.AllAtWait(d);
    std::int64_t queries = 0;
    const IsFree is_free = [&](const Eigen::VectorXd& v) {
      return world.Free(d, ToConfig(v));
    };
    ++queries;
    if (!is_free(ToVector(s.path.front()))) {
      const Nearest n = world.Least(d, s.path.front(), kSceneFar);
      Fault(where + Fmt(": its first waypoint is not free: %s at %.4f m", scene_.PairText(n).c_str(),
                        n.distance));
      r_->queries += queries;
      return;
    }
    for (size_t k = 1; k < s.path.size(); ++k) {
      if (!EdgeIsFree(ToVector(s.path[k - 1]), ToVector(s.path[k]), is_free,
                      f_.edge_resolution, motion.weights, EdgeOrder::kBisection,
                      &queries)) {
        Fault(where + Fmt(": the edge from waypoint %zu to %zu is not free by R16's test", k - 1,
                          k));
        break;
      }
    }
    r_->queries += queries;
  }

  // The segments of the arm against the bay's steps of the arm.
  void Sequence(int i, const std::vector<Segment>& segs, const ArmMotion& motion) {
    const std::string name = Arm(i);
    const SceneArm& sa = scene_.arm(i);
    std::vector<const BayStep*> mine;
    for (const BayStep& s : report_.steps) {
      if (s.arm == name) mine.push_back(&s);
    }
    // Groups of the non-wait segments by step.
    struct Group {
      int step = 0;
      std::vector<int> segs;
    };
    std::vector<Group> groups;
    for (size_t g = 0; g < segs.size(); ++g) {
      if (segs[g].kind == "wait") continue;
      if (groups.empty() || groups.back().step != segs[g].step) groups.push_back({segs[g].step, {}});
      groups.back().segs.push_back(static_cast<int>(g));
    }
    size_t next = 0;  // Into `mine`.
    std::string tool = sa.use->first_tool;
    std::string held;
    PoseRef at = WaitPose(i);
    std::map<int, int> group_of;  // Step number to group.
    for (size_t gi = 0; gi < groups.size(); ++gi) {
      const Group& grp = groups[gi];
      const BayStep* step = Step(grp.step);
      const Segment& first = segs[grp.segs.front()];
      if (step == nullptr || step->arm != name) {
        Fault(Fmt("arm %s has segments for step %d from %.3f s, which is not a step of this arm",
                  name.c_str(), grp.step, first.t0));
        continue;
      }
      if (group_of.count(grp.step)) {
        Fault(Fmt("arm %s: the segments of %s are not together (again from %.3f s)",
                  name.c_str(), StepName(step, grp.step).c_str(), first.t0));
        continue;
      }
      group_of[grp.step] = static_cast<int>(gi);
      // In the order of the bay's steps.
      size_t found = next;
      while (found < mine.size() && mine[found]->number != grp.step) ++found;
      if (found == mine.size()) {
        Fault(Fmt("arm %s does %s at %.3f s out of the order of the bay's steps", name.c_str(),
                  StepName(step, grp.step).c_str(), first.t0));
        // Its time still counts for R21's order of the steps.
        times_[grp.step] = {grp.step, step->what, name, first.t0, segs[grp.segs.back()].t1};
        continue;
      }
      for (size_t k = next; k < found; ++k) {
        Fault(Fmt("arm %s leaves out %s (before %s at %.3f s)", name.c_str(),
                  StepName(mine[k], mine[k]->number).c_str(),
                  StepName(step, grp.step).c_str(), first.t0));
      }
      next = found + 1;
      // The waits before the group: what the arm has and holds.
      for (int g = grp.segs.front() - 1; g >= 0 && segs[g].kind == "wait"; --g) {
        WaitHas(i, segs[g], tool, held);
      }
      // Parking is allowed directly after the arm's last step of phase
      // "rails", for an arm that does not lead the phase.
      bool park = false;
      if (found > 0 && mine[found - 1]->phase == "rails") {
        park = true;
        for (size_t k = found; k < mine.size(); ++k) {
          if (mine[k]->phase == "rails") park = false;
        }
        const Work* lead = FindWork(L_, "rail_set");
        if (lead != nullptr && lead->arm == name) park = false;
      }
      StepSegments(i, *step, found + 1 == mine.size(), park, segs, grp.segs, motion, &tool,
                   &held, &at);
    }
    for (size_t k = next; k < mine.size(); ++k) {
      Fault(Fmt("arm %s leaves out %s", name.c_str(), StepName(mine[k], mine[k]->number).c_str()));
    }
    for (int g = static_cast<int>(segs.size()) - 1; g >= 0 && segs[g].kind == "wait"; --g) {
      WaitHas(i, segs[g], tool, held);
    }
  }

  void WaitHas(int i, const Segment& s, const std::string& tool, const std::string& held) {
    if (s.tool != tool || s.held != held) {
      Fault(Fmt("arm %s, the wait of %s from %.3f s has tool \"%s\" holding \"%s\"; the steps "
                "give tool \"%s\" holding \"%s\"",
                Arm(i).c_str(), StepName(Step(s.step), s.step).c_str(), s.t0, s.tool.c_str(),
                s.held.c_str(), tool.c_str(), held.c_str()));
    }
  }

  void Timing(int i, const Segment& s, double want, const char* what) {
    if (want < 0.0) {
      Fault(Fmt("arm %s, %s: its %s from %.3f s cannot be timed", Arm(i).c_str(),
                StepName(Step(s.step), s.step).c_str(), s.kind.c_str(), s.t0));
      return;
    }
    if (std::abs((s.t1 - s.t0) - want) > o_.time_tolerance_s) {
      Fault(Fmt("arm %s, %s: its %s from %.3f s takes %.4f s, %s gives %.4f s", Arm(i).c_str(),
                StepName(Step(s.step), s.step).c_str(), s.kind.c_str(), s.t0, s.t1 - s.t0, what,
                want));
    }
  }

  // `last`: the arm's last step of the bay. After its last tool change an
  // arm goes back to its wait pose, so that it stands there when the bay
  // goes out: that move is the tool change's.
  void StepSegments(int i, const BayStep& step, bool last, bool park,
                    const std::vector<Segment>& segs,
                    const std::vector<int>& idx, const ArmMotion& motion, std::string* tool,
                    std::string* held, PoseRef* at) {
    const std::string name = Arm(i);
    const std::string label = StepName(&step, step.number);
    // Parking: a step may begin with a move to the wait pose (and a wait,
    // which is not in `idx`) before the move to its goal.
    size_t own = 0;
    if (idx.size() >= 2 && segs[idx[0]].kind == "move" && segs[idx[1]].kind == "move") {
      const Segment& s = segs[idx[0]];
      if (!park) {
        Fault(Fmt("arm %s, %s: it parks at its wait pose from %.3f s, which only an arm that "
                  "follows the rails' leader may do, after its last step of the rails",
                  name.c_str(), label.c_str(), s.t0));
      }
      if (s.tool != *tool || s.held != *held) {
        Fault(Fmt("arm %s, %s: its move to park from %.3f s has tool \"%s\" holding \"%s\"; "
                  "the step gives tool \"%s\" holding \"%s\"",
                  name.c_str(), label.c_str(), s.t0, s.tool.c_str(), s.held.c_str(),
                  tool->c_str(), held->c_str()));
      }
      if (!s.path.empty()) {
        CheckPose(i, s, s.path.front(), *at, "starts");
        CheckPose(i, s, s.path.back(), WaitPose(i), "ends");
        const double d = MoveDuration(s.path, motion, f_);
        Timing(i, s, d < 0.0 ? d : d + f_.settle_s, "its path and settle_s");
        if (d >= 0.0 && std::abs(s.planned_s - d) > o_.time_tolerance_s) {
          Fault(Fmt("arm %s, %s: its move from %.3f s says planned_s %.4f s, its path gives "
                    "%.4f s",
                    name.c_str(), label.c_str(), s.t0, s.planned_s, d));
        }
      }
      *at = WaitPose(i);
      own = 1;
    }
    std::string kinds;
    for (size_t k = own; k < idx.size(); ++k) kinds += (kinds.empty() ? "" : ",") + segs[idx[k]].kind;
    const bool work_kind = step.index >= 0 && step.kind != "tool_change" &&
                           step.kind != "to_wait";
    bool pattern_ok = false;
    if (work_kind) {
      pattern_ok = kinds == "move,approach,work,retreat" || kinds == "move,approach,work" ||
                   kinds == "approach,work,retreat" || kinds == "approach,work" ||
                   kinds == "work,retreat" || kinds == "work";
    } else if (step.kind == "tool_change") {
      pattern_ok = kinds == "move,work" || kinds == "work" ||
                   (last && (kinds == "move,work,move" || kinds == "work,move"));
    } else if (step.kind == "to_wait") {
      pattern_ok = kinds == "move";
    }
    if (!pattern_ok) {
      Fault(Fmt("arm %s, %s from %.3f s has the segments %s, which a %s step cannot have",
                name.c_str(), label.c_str(), segs[idx.front()].t0, kinds.c_str(),
                step.kind.c_str()));
    }
    const SolvedPoint* p = work_kind ? Point(step) : nullptr;
    if (work_kind && p == nullptr) {
      Fault(Fmt("arm %s, %s: the checker has no pose for it", name.c_str(), label.c_str()));
    }
    if (work_kind && step.tool != *tool) {
      Fault(Fmt("arm %s, %s needs tool \"%s\" and the arm has \"%s\"", name.c_str(),
                label.c_str(), step.tool.c_str(), tool->c_str()));
    }
    const PoseRef work = step.kind == "tool_change" ? ToolChangePose(i) : PointPose(p, "work");
    const PoseRef approach =
        p != nullptr && !p->approach_leg_deg.empty() ? PointPose(p, "approach") : work;
    const PoseRef retreat =
        p != nullptr && !p->retreat_leg_deg.empty() ? PointPose(p, "retreat") : work;
    bool changed = false;  // The tool change's work is behind.
    for (size_t k = own; k < idx.size(); ++k) {
      const Segment& s = segs[idx[k]];
      std::string want_held = *held;
      if (p != nullptr && (s.kind == "approach" || s.kind == "work")) want_held = p->held_approach;
      if (p != nullptr && s.kind == "retreat") want_held = p->held_retreat;
      if (s.tool != *tool || s.held != want_held) {
        Fault(Fmt("arm %s, %s: its %s from %.3f s has tool \"%s\" holding \"%s\"; the step "
                  "gives tool \"%s\" holding \"%s\"",
                  name.c_str(), label.c_str(), s.kind.c_str(), s.t0, s.tool.c_str(),
                  s.held.c_str(), tool->c_str(), want_held.c_str()));
      }
      if (s.path.empty()) continue;
      if (s.kind == "move") {
        CheckPose(i, s, s.path.front(), *at, "starts");
        const PoseRef& to = step.kind == "to_wait" || changed ? WaitPose(i)
                            : step.kind == "tool_change"      ? work
                                                              : approach;
        CheckPose(i, s, s.path.back(), to, "ends");
        const double d = MoveDuration(s.path, motion, f_);
        Timing(i, s, d < 0.0 ? d : d + f_.settle_s, "its path and settle_s");
        if (d >= 0.0 && std::abs(s.planned_s - d) > o_.time_tolerance_s) {
          Fault(Fmt("arm %s, %s: its move from %.3f s says planned_s %.4f s, its path gives "
                    "%.4f s",
                    name.c_str(), label.c_str(), s.t0, s.planned_s, d));
        }
        *at = to;
      } else if (s.kind == "approach") {
        CheckPose(i, s, s.path.front(), approach, "starts");
        CheckPose(i, s, s.path.back(), work, "ends");
        Timing(i, s,
               LegDuration(s.path, p != nullptr ? p->approach_length_m : 0.0, motion, f_),
               "the leg's law");
        *at = work;
      } else if (s.kind == "work") {
        CheckPose(i, s, s.path.front(), work, "starts");
        double want = -1.0;
        if (step.kind == "tool_change") {
          want = scene_.arm(i).use->tool_change_s;
        } else if (const Work* w = FindWork(L_, step.kind, step.index)) {
          want = w->process_s;
        }
        Timing(i, s, want,
               step.kind == "tool_change" ? "the arm's tool_change_s" : "the work's process_s");
        *at = work;
        if (step.kind == "tool_change") {
          *tool = step.tool_after.empty() ? step.tool : step.tool_after;
          changed = true;
        }
      } else if (s.kind == "retreat") {
        CheckPose(i, s, s.path.front(), work, "starts");
        CheckPose(i, s, s.path.back(), retreat, "ends");
        Timing(i, s,
               LegDuration(s.path, p != nullptr ? p->retreat_length_m : 0.0, motion, f_),
               "the leg's law");
        *at = retreat;
      }
    }
    if (p != nullptr) *held = p->held_retreat;
    StepTime t;
    t.number = step.number;
    t.start = segs[idx.front()].t0;
    t.end = segs[idx.back()].t1;
    times_[step.number] = t;
  }

  void StepsAndPhases() {
    const double tol = o_.time_tolerance_s;
    for (const BayEvent& e : plan_.events) times_[e.step] = {e.step, "", "", e.t0, e.t1};
    if (plan_.steps.size() != report_.steps.size()) {
      Fault(Fmt("the plan has %zu step times, the bay %zu steps", plan_.steps.size(),
                report_.steps.size()));
    }
    for (size_t k = 0; k < plan_.steps.size() && k < report_.steps.size(); ++k) {
      const StepTime& t = plan_.steps[k];
      const BayStep& s = report_.steps[k];
      if (t.number != s.number || t.what != s.what || t.arm != s.arm) {
        Fault(Fmt("the plan's step time %zu is \"%d %s %s\", the bay's step is \"%d %s %s\"",
                  k, t.number, t.what.c_str(), t.arm.c_str(), s.number, s.what.c_str(),
                  s.arm.c_str()));
        continue;
      }
      auto it = times_.find(s.number);
      if (it == times_.end()) {
        Fault(Fmt("%s has no segments and no event", StepName(&s, s.number).c_str()));
        continue;
      }
      if (std::abs(it->second.start - t.start) > tol || std::abs(it->second.end - t.end) > tol) {
        Fault(Fmt("%s runs from %.3f to %.3f s by the segments, the plan says %.3f to %.3f s",
                  StepName(&s, s.number).c_str(), it->second.start, it->second.end, t.start,
                  t.end));
      }
    }
    // The phases.
    const double clamp_end = events_.count("clamp") ? events_["clamp"]->t1 : 0.0;
    double rails_end = clamp_end, modules_end = 0.0;
    for (const BayStep& s : report_.steps) {
      auto it = times_.find(s.number);
      if (it == times_.end() || s.arm.empty()) continue;
      if (s.kind == "strap_bolt") rails_end = std::max(rails_end, it->second.end);
      if (s.phase == "modules") modules_end = std::max(modules_end, it->second.end);
    }
    modules_end = std::max(modules_end, rails_end);
    last_strap_end_ = rails_end;
    last_modules_end_ = modules_end;
    auto event_span = [&](const char* k) {
      return events_.count(k) ? events_[k]->t1 - events_[k]->t0 : 0.0;
    };
    const std::vector<std::pair<std::string, double>> want = {
        {"tube in", event_span("tube_in")},
        {"clamp", event_span("clamp")},
        {"rails", rails_end - clamp_end},
        {"modules", modules_end - rails_end},
        {"end", bay_end_ - modules_end}};
    if (plan_.phases.size() != want.size()) {
      Fault(Fmt("the plan has %zu phases, not five", plan_.phases.size()));
    }
    double sum = 0.0;
    for (size_t k = 0; k < want.size(); ++k) {
      sum += want[k].second;
      if (k >= plan_.phases.size()) continue;
      const PhaseTime& p = plan_.phases[k];
      if (p.name != want[k].first || std::abs(p.planned_s - want[k].second) > tol) {
        Fault(Fmt("the phase \"%s\" says %.3f s; by the schedule \"%s\" takes %.3f s",
                  p.name.c_str(), p.planned_s, want[k].first.c_str(), want[k].second));
      }
    }
    if (std::abs(plan_.bay_planned_s - sum) > tol || std::abs(plan_.bay_planned_s - bay_end_) > tol) {
      Fault(Fmt("bay_planned_s is %.3f s; the phases sum to %.3f s and the bay ends at %.3f s",
                plan_.bay_planned_s, sum, bay_end_));
    }
  }

  // R21's order.
  void Order() {
    const double tol = o_.time_tolerance_s;
    // The end of the work segment of each step.
    std::map<int, double> work_end;
    for (const ArmPlan& a : plan_.arms) {
      for (const Segment& s : a.segments) {
        if (s.kind == "work") work_end[s.step] = s.t1;
      }
    }
    std::map<std::pair<std::string, int>, int> number;  // (kind, index) to step.
    for (const BayStep& s : report_.steps) {
      if (s.index >= 0) number[{s.kind, s.index}] = s.number;
    }
    auto start_of = [&](int n) {
      auto it = times_.find(n);
      return it == times_.end() ? -1.0 : it->second.start;
    };
    auto end_of_work = [&](const std::string& kind, int index) {
      auto it = number.find({kind, index});
      if (it == number.end()) return -1.0;
      auto w = work_end.find(it->second);
      return w == work_end.end() ? -1.0 : w->second;
    };
    std::map<int, int> last_hole;  // Module to the step before in its holes.
    bool first_take = true;
    for (const BayStep& s : report_.steps) {
      const double start = start_of(s.number);
      if (start < 0.0) continue;
      const std::string label = StepName(&s, s.number);
      auto before = [&](double t, const std::string& what) {
        if (t >= 0.0 && start < t - tol) {
          Fault(Fmt("arm %s starts %s at %.3f s, before %s at %.3f s", s.arm.c_str(),
                    label.c_str(), start, what.c_str(), t));
        }
      };
      if (s.kind == "strap_bolt") {
        before(end_of_work("rail_set", s.index), Fmt("rail %d's set has ended", s.index));
      }
      if (s.kind == "locate" || s.kind == "pin" || s.kind == "swage") {
        const int m = s.index / 4;
        before(end_of_work("module_set", m), Fmt("module %d's set has ended", m));
        auto it = last_hole.find(m);
        if (it != last_hole.end()) {
          auto t = times_.find(it->second);
          if (t != times_.end()) {
            before(t->second.end, StepName(Step(it->second), it->second) + " has ended");
          }
        }
        last_hole[m] = s.number;
      }
      if (s.kind == "module_take" && L_.order.modules_start == "after_their_rails") {
        // src/layout.h, WorkOrder: the take of module m after the bolts of
        // rails m and m+1.
        const std::vector<int> order = ModuleOrder(L_);
        if (s.index >= 0 && s.index < static_cast<int>(order.size())) {
          const int m = order[s.index];
          for (int rail : {m, m + 1}) {
            auto it = number.find({"strap_bolt", rail});
            if (it == number.end()) continue;
            auto t = times_.find(it->second);
            if (t != times_.end()) {
              before(t->second.end, Fmt("rail %d's strap bolt is driven", rail));
            }
          }
        }
      } else if (s.kind == "module_take" && first_take) {
        first_take = false;
        before(last_strap_end_, "the last strap bolt is driven");
      }
      if (s.kind == "release") before(last_modules_end_, "the last step of the modules has ended");
      if (s.kind == "bay_out" && events_.count("release")) {
        before(events_["release"]->t1, "release has ended");
      }
    }
  }

  const Layout& L_;
  const Plan& plan_;
  const CheckReport& report_;
  const LayoutScene& scene_;
  const PlanPlayer& player_;
  const VerifyOptions& o_;
  VerifyReport* r_;
  const PlanFigures& f_;
  DataPtr data_;
  std::map<int, const BayStep*> steps_;
  std::map<std::pair<std::string, int>, const SolvedPoint*> points_;
  std::map<std::string, const BayEvent*> events_;
  std::map<int, StepTime> times_;
  double bay_end_ = 0.0;
  double last_strap_end_ = 0.0, last_modules_end_ = 0.0;
};

// R22 along the schedule, and the least distances and joint margins.
void Schedule(const LayoutScene& scene, const CheckReport& report, const Plan& plan,
              const VerifyOptions& o, double end, VerifyReport* r) {
  const PlanPlayer player(scene, report, plan);
  Measurer measurer(scene, report, player);
  DataPtr data = scene.MakeData();
  const double sample = plan.figures.sample_s > 0.0 ? plan.figures.sample_s : 0.05;
  const double clearance = plan.figures.clearance_m;
  std::map<std::string, Closest> least, arm_pairs;
  std::map<std::string, std::pair<int, int>> faults;  // Key to (shown, more).
  std::map<std::pair<int, int>, VerifyReport::JointMargin> margins;
  const long n = static_cast<long>(std::floor(end / sample + 1e-9));
  std::vector<double> times;
  for (long k = 0; k <= n; ++k) times.push_back(k * sample);
  if (times.empty() || times.back() < end - 1e-9) times.push_back(end);
  for (double t : times) {
    if (t < o.from_s - 1e-9 || (o.to_s >= 0.0 && t > o.to_s + 1e-9)) continue;
    const WorldAt w = player.At(t);
    for (int i = 0; i < scene.ArmCount(); ++i) {
      const SceneArm& a = scene.arm(i);
      for (int j = 0; j < 6; ++j) {
        const double q = w.arms[i].config[j];
        const double m = std::min(q - a.dlo[j], a.dhi[j] - q) * 180.0 / kPi;
        auto it = margins.find({i, j});
        if (it == margins.end() || m < it->second.margin_deg) {
          margins[{i, j}] = {m, a.use->name, a.cat->joint_names[j], w.arms[i].step, t};
        }
      }
    }
    for (const Closest& c : measurer.Measure(w, data.get(), &r->queries)) {
      const bool bad = c.by_design ? c.distance < -o.tolerance_m
                                   : c.distance < clearance - o.tolerance_m;
      if (bad) {
        const std::string key = c.other_arm.empty() ? c.arm : c.arm + "|" + c.other_arm;
        auto& [shown, more] = faults[key];
        if (shown < 3) {
          ++shown;
          r->faults.push_back(Fmt(
              "at %.2f s (step %d) arm %s%s%s: %s to %s at %.4f m%s, below %.4f m", c.t, c.step,
              c.arm.c_str(), c.other_arm.empty() ? "" : " and arm ",
              c.other_arm.c_str(), c.a.c_str(), c.b.c_str(), c.distance,
              c.by_design ? " (near by design, but into each other)" : "",
              c.by_design ? 0.0 : clearance));
        } else {
          ++more;
        }
      }
      if (c.by_design) continue;
      const std::string key = c.a + "|" + c.b;
      auto it = least.find(key);
      if (it == least.end() || c.distance < it->second.distance) least[key] = c;
      if (!c.other_arm.empty()) {
        const std::string pk = c.arm + "|" + c.other_arm;
        auto pt = arm_pairs.find(pk);
        if (pt == arm_pairs.end() || c.distance < pt->second.distance) arm_pairs[pk] = c;
      }
    }
  }
  for (const auto& [key, count] : faults) {
    if (count.second > 0) {
      r->faults.push_back(Fmt("and %d more samples below the clearance for %s", count.second,
                              key.c_str()));
    }
  }
  auto tighter = [](const Closest& a, const Closest& b) {
    if (a.distance != b.distance) return a.distance < b.distance;
    if (a.t != b.t) return a.t < b.t;
    return a.a + a.b < b.a + b.b;
  };
  for (const auto& [key, c] : least) r->least.push_back(c);
  std::sort(r->least.begin(), r->least.end(), tighter);
  if (static_cast<int>(r->least.size()) > o.keep) r->least.resize(o.keep);
  for (const auto& [key, c] : arm_pairs) r->arm_pairs.push_back(c);
  std::sort(r->arm_pairs.begin(), r->arm_pairs.end(), tighter);
  if (!r->least.empty()) r->least_clearance_m = r->least.front().distance;
  if (!r->arm_pairs.empty()) r->least_arm_to_arm_m = r->arm_pairs.front().distance;
  for (const auto& [key, m] : margins) r->joint_margins.push_back(m);
  std::sort(r->joint_margins.begin(), r->joint_margins.end(),
            [](const VerifyReport::JointMargin& a, const VerifyReport::JointMargin& b) {
              if (a.margin_deg != b.margin_deg) return a.margin_deg < b.margin_deg;
              return a.arm + a.joint < b.arm + b.joint;
            });
  if (r->joint_margins.size() > 5) r->joint_margins.resize(5);
  if (!r->joint_margins.empty()) r->least_joint_margin_deg = r->joint_margins.front().margin_deg;
}

}  // namespace

VerifyReport VerifyPlan(const Layout& layout, const ModelPaths& paths, const Plan& plan,
                        const std::string& layout_sha256, const VerifyOptions& options) {
  VerifyReport r;
  if (layout_sha256 != plan.layout_sha256) {
    r.faults.push_back("the plan was made for another version of the layout: its file's "
                       "sha256 is " + layout_sha256 + ", the plan's is " + plan.layout_sha256);
    r.ok = false;
    return r;
  }
  const CheckReport report = CheckLayout(layout, paths);
  if (!report.error.empty()) {
    r.faults.push_back("the checker cannot build the layout's model: " + report.error);
    return r;
  }
  if (!report.pass) {
    std::string rows;
    int count = 0;
    for (const CheckRow& row : report.rows) {
      if (row.pass) continue;
      if (++count <= 3) rows += (rows.empty() ? "" : "; ") + row.check + "," + row.item;
    }
    r.faults.push_back(Fmt("the layout does not pass the checker: %d rows fail (%s)", count,
                           rows.c_str()));
  }
  if (std::abs(plan.figures.clearance_m - report.clearance_m) > 1e-12 ||
      std::abs(plan.figures.joint_margin_deg - report.joint_margin_deg) > 1e-12) {
    r.faults.push_back(Fmt("the plan's margins are %.4f m and %.2f deg, the checker's for the "
                           "layout %.4f m and %.2f deg",
                           plan.figures.clearance_m, plan.figures.joint_margin_deg,
                           report.clearance_m, report.joint_margin_deg));
  }
  if (!plan.complete) r.faults.push_back("the plan is not complete");
  // Every figure but the margins and the budgets is ours and fixed: a plan
  // that brings its own cannot loosen what is checked.
  {
    const PlanFigures d;
    const PlanFigures& f = plan.figures;
    const std::vector<std::tuple<const char*, double, double>> figures = {
        {"plan_pad_m", f.plan_pad_m, d.plan_pad_m},
        {"to_full_speed_s", f.to_full_speed_s, d.to_full_speed_s},
        {"jerk_rise_s", f.jerk_rise_s, d.jerk_rise_s},
        {"settle_s", f.settle_s, d.settle_s},
        {"leg_speed", f.leg_speed, d.leg_speed},
        {"leg_accel", f.leg_accel, d.leg_accel},
        {"max_deviation", f.max_deviation, d.max_deviation},
        {"edge_resolution", f.edge_resolution, d.edge_resolution},
        {"sample_s", f.sample_s, d.sample_s}};
    for (const auto& [name, value, want] : figures) {
      if (!(std::abs(value - want) <= 1e-12)) {
        r.faults.push_back(Fmt("the plan's figure %s is %.9g, not ours, %.9g", name, value, want));
      }
    }
    if (f.shorten_tries != d.shorten_tries) {
      r.faults.push_back(Fmt("the plan's figure shorten_tries is %d, not ours, %d",
                             f.shorten_tries, d.shorten_tries));
    }
    if (f.seed != d.seed) {
      r.faults.push_back(Fmt("the plan's figure seed is %u, not ours, %u", f.seed, d.seed));
    }
  }
  if (plan.layout != layout.name) {
    r.faults.push_back("the plan is of layout " + plan.layout + ", not " + layout.name);
  }
  std::string error;
  const std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(layout, paths, plan.figures.joint_margin_deg, &error);
  if (scene == nullptr) {
    r.faults.push_back("the layout's scene cannot be loaded: " + error);
    return r;
  }
  const PlanPlayer player(*scene, report, plan);
  Verifier v(layout, plan, report, *scene, player, options, &r);
  v.Run();
  if (options.schedule) Schedule(*scene, report, plan, options, v.bay_end(), &r);
  r.ok = r.faults.empty();
  return r;
}

}  // namespace cell::layout
