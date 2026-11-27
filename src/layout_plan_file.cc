// The plan as a file (src/layout_plan.h): PlanToJson, PlanFromJson and the
// SHA-256 of a layout's file that a plan names.

#include "src/layout_plan.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace cell::layout {

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

// A plan's retreat figures, only when they are not the approach's: a plan of
// a layout without retreat_legs is written as before they were figures.
std::string RetreatText(const PlanFigures& f) {
  if (f.retreat_speed == f.leg_speed && f.retreat_accel == f.leg_accel) return "";
  return ",\"retreat_speed\":" + Fig(f.retreat_speed) +
         ",\"retreat_accel\":" + Fig(f.retreat_accel);
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
         RetreatText(f) +
         ",\"max_deviation\":" + Fig(f.max_deviation) +
         ",\"edge_resolution\":" + Fig(f.edge_resolution) +
         ",\"shorten_tries\":" + std::to_string(f.shorten_tries) +
         // Only when not 1: a plan of a layout without path_tries is
         // written as before it was a figure.
         (f.path_tries == 1 ? std::string()
                            : ",\"path_tries\":" + std::to_string(f.path_tries)) +
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
  // A plan of one bay is written as it was before plans had bays.
  const bool bays = plan.bays > 1;
  auto bay = [&](int b) { return bays ? ",\"bay\":" + std::to_string(b) : std::string(); };
  std::string s = "{\n";
  s += "\"layout\": " + Q(plan.layout) + ",\n";
  s += "\"layout_sha256\": " + Q(plan.layout_sha256) + ",\n";
  s += "\"complete\": " + B(plan.complete) + ",\n";
  s += "\"bay_planned_s\": " + F6(plan.bay_planned_s) + ",\n";
  s += "\"bay_model_s\": " + F6(plan.bay_model_s) + ",\n";
  if (bays) s += "\"bays\": " + std::to_string(plan.bays) + ",\n";
  s += "\"figures\": " + FiguresText(plan.figures) + ",\n";
  std::vector<std::string> lines;
  for (const BayEvent& e : plan.events) {
    // A transfer's stages (staged work only, so other plans read as before).
    const std::string stages =
        e.kind == "transfer" ? ",\"from_stage\":" + std::to_string(e.from_stage) +
                                   ",\"to_stage\":" + std::to_string(e.to_stage)
                             : std::string();
    lines.push_back("{\"kind\":" + Q(e.kind) + ",\"step\":" + std::to_string(e.step) +
                    ",\"t0\":" + F6(e.t0) + ",\"t1\":" + F6(e.t1) + bay(e.bay) + stages +
                    "}");
  }
  s += "\"events\": " + Lines(lines) + ",\n";
  lines.clear();
  for (const StepTime& t : plan.steps) {
    lines.push_back("{\"number\":" + std::to_string(t.number) + ",\"what\":" +
                    Q(t.what) + ",\"arm\":" + Q(t.arm) + ",\"start\":" +
                    F6(t.start) + ",\"end\":" + F6(t.end) + bay(t.bay) + "}");
  }
  s += "\"steps\": " + Lines(lines) + ",\n";
  lines.clear();
  for (const PhaseTime& p : plan.phases) {
    lines.push_back("{\"name\":" + Q(p.name) + ",\"planned_s\":" + F6(p.planned_s) +
                    ",\"model_s\":" + F6(p.model_s) + bay(p.bay) + "}");
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
    // A plan without bays is one bay, bay 0.
    p.bays = j.contains("bays") ? Get<int>(j, "bays") : 1;
    auto bay = [](const json& e) { return e.contains("bay") ? Get<int>(e, "bay") : 0; };
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
    // Plans without retreat figures time a retreat as an approach.
    p.figures.retreat_speed =
        f.contains("retreat_speed") ? Get<double>(f, "retreat_speed") : p.figures.leg_speed;
    p.figures.retreat_accel =
        f.contains("retreat_accel") ? Get<double>(f, "retreat_accel") : p.figures.leg_accel;
    p.figures.max_deviation = Get<double>(f, "max_deviation");
    p.figures.edge_resolution = Get<double>(f, "edge_resolution");
    p.figures.shorten_tries = Get<int>(f, "shorten_tries");
    p.figures.path_tries = f.contains("path_tries") ? Get<int>(f, "path_tries") : 1;
    p.figures.budgets = Get<std::vector<int>>(f, "budgets");
    p.figures.seed = Get<std::uint32_t>(f, "seed");
    p.figures.sample_s = Get<double>(f, "sample_s");
    for (const json& e : j.at("events")) {
      BayEvent b;
      b.kind = Get<std::string>(e, "kind");
      b.step = Get<int>(e, "step");
      b.t0 = Get<double>(e, "t0");
      b.t1 = Get<double>(e, "t1");
      b.bay = bay(e);
      if (e.contains("from_stage")) b.from_stage = Get<int>(e, "from_stage");
      if (e.contains("to_stage")) b.to_stage = Get<int>(e, "to_stage");
      p.events.push_back(b);
    }
    for (const json& e : j.at("steps")) {
      StepTime t;
      t.number = Get<int>(e, "number");
      t.what = Get<std::string>(e, "what");
      t.arm = Get<std::string>(e, "arm");
      t.start = Get<double>(e, "start");
      t.end = Get<double>(e, "end");
      t.bay = bay(e);
      p.steps.push_back(t);
    }
    for (const json& e : j.at("phases")) {
      PhaseTime t;
      t.name = Get<std::string>(e, "name");
      t.planned_s = Get<double>(e, "planned_s");
      t.model_s = Get<double>(e, "model_s");
      t.bay = bay(e);
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

}  // namespace cell::layout
