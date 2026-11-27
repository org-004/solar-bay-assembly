// A checker's report as a file: see src/layout_report_file.h.

#include "src/layout_report_file.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace cell::layout {

namespace {

using Json = nlohmann::ordered_json;

// A double that reads back bit for bit; not finite as a string.
Json Num(double v) {
  if (std::isnan(v)) return "nan";
  if (std::isinf(v)) return v > 0 ? "inf" : "-inf";
  return v;
}

double ToNum(const Json& j) {
  if (j.is_string()) {
    const std::string s = j.get<std::string>();
    if (s == "nan") return std::numeric_limits<double>::quiet_NaN();
    if (s == "inf") return std::numeric_limits<double>::infinity();
    if (s == "-inf") return -std::numeric_limits<double>::infinity();
    throw std::runtime_error("not a number: \"" + s + "\"");
  }
  if (!j.is_number()) throw std::runtime_error("not a number: " + j.dump());
  return j.get<double>();
}

Json Joints(const std::array<double, 6>& a) {
  Json j = Json::array();
  for (double v : a) j.push_back(Num(v));
  return j;
}

std::array<double, 6> ToJoints(const Json& j) {
  if (!j.is_array() || j.size() != 6) {
    throw std::runtime_error("not six joint values: " + j.dump());
  }
  std::array<double, 6> a{};
  for (size_t i = 0; i < 6; ++i) a[i] = ToNum(j[i]);
  return a;
}

Json Leg(const std::vector<std::array<double, 6>>& leg) {
  Json j = Json::array();
  for (const auto& s : leg) j.push_back(Joints(s));
  return j;
}

std::vector<std::array<double, 6>> ToLeg(const Json& j) {
  std::vector<std::array<double, 6>> leg;
  for (const Json& s : j) leg.push_back(ToJoints(s));
  return leg;
}

Json RowJson(const CheckRow& r) {
  return Json{{"check", r.check}, {"item", r.item},   {"arm", r.arm},
              {"tool", r.tool},   {"figure", Num(r.figure)}, {"unit", r.unit},
              {"limit", Num(r.limit)}, {"pass", r.pass}, {"detail", r.detail}};
}

CheckRow ToRow(const Json& j) {
  CheckRow r;
  r.check = j.at("check").get<std::string>();
  r.item = j.at("item").get<std::string>();
  r.arm = j.at("arm").get<std::string>();
  r.tool = j.at("tool").get<std::string>();
  r.figure = ToNum(j.at("figure"));
  r.unit = j.at("unit").get<std::string>();
  r.limit = ToNum(j.at("limit"));
  r.pass = j.at("pass").get<bool>();
  r.detail = j.at("detail").get<std::string>();
  return r;
}

Json StepJson(const BayStep& s) {
  return Json{{"number", s.number}, {"what", s.what},     {"arm", s.arm},
              {"tool", s.tool},     {"part", s.part},     {"from", s.from},
              {"to", s.to},         {"time_s", Num(s.time_s)}, {"kind", s.kind},
              {"index", s.index},   {"phase", s.phase},   {"tool_after", s.tool_after},
              {"stage", s.stage}};
}

BayStep ToStep(const Json& j) {
  BayStep s;
  s.number = j.at("number").get<int>();
  s.what = j.at("what").get<std::string>();
  s.arm = j.at("arm").get<std::string>();
  s.tool = j.at("tool").get<std::string>();
  s.part = j.at("part").get<std::string>();
  s.from = j.at("from").get<std::string>();
  s.to = j.at("to").get<std::string>();
  s.time_s = ToNum(j.at("time_s"));
  s.kind = j.at("kind").get<std::string>();
  s.index = j.at("index").get<int>();
  s.phase = j.at("phase").get<std::string>();
  s.tool_after = j.at("tool_after").get<std::string>();
  s.stage = j.at("stage").get<std::string>();
  return s;
}

Json PointJson(const SolvedPoint& p) {
  return Json{{"kind", p.kind},
              {"index", p.index},
              {"item", p.item},
              {"arm", p.arm},
              {"tool", p.tool},
              {"held_approach", p.held_approach},
              {"held_work", p.held_work},
              {"held_retreat", p.held_retreat},
              {"track", Num(p.track)},
              {"work_deg", Joints(p.work_deg)},
              {"approach_deg", Joints(p.approach_deg)},
              {"retreat_deg", Joints(p.retreat_deg)},
              {"work_reached", p.work_reached},
              {"approach_reached", p.approach_reached},
              {"retreat_reached", p.retreat_reached},
              {"approach_leg_deg", Leg(p.approach_leg_deg)},
              {"retreat_leg_deg", Leg(p.retreat_leg_deg)},
              {"approach_length_m", Num(p.approach_length_m)},
              {"retreat_length_m", Num(p.retreat_length_m)},
              {"stage", p.stage}};
}

SolvedPoint ToPoint(const Json& j) {
  SolvedPoint p;
  p.kind = j.at("kind").get<std::string>();
  p.index = j.at("index").get<int>();
  p.item = j.at("item").get<std::string>();
  p.arm = j.at("arm").get<std::string>();
  p.tool = j.at("tool").get<std::string>();
  p.held_approach = j.at("held_approach").get<std::string>();
  p.held_work = j.at("held_work").get<std::string>();
  p.held_retreat = j.at("held_retreat").get<std::string>();
  p.track = ToNum(j.at("track"));
  p.work_deg = ToJoints(j.at("work_deg"));
  p.approach_deg = ToJoints(j.at("approach_deg"));
  p.retreat_deg = ToJoints(j.at("retreat_deg"));
  p.work_reached = j.at("work_reached").get<bool>();
  p.approach_reached = j.at("approach_reached").get<bool>();
  p.retreat_reached = j.at("retreat_reached").get<bool>();
  p.approach_leg_deg = ToLeg(j.at("approach_leg_deg"));
  p.retreat_leg_deg = ToLeg(j.at("retreat_leg_deg"));
  p.approach_length_m = ToNum(j.at("approach_length_m"));
  p.retreat_length_m = ToNum(j.at("retreat_length_m"));
  p.stage = j.at("stage").get<std::string>();
  return p;
}

}  // namespace

std::string CheckReportToJson(const CheckReport& report, const std::string& name,
                              const std::string& layout_sha256) {
  Json r;
  Json rows = Json::array(), steps = Json::array(), points = Json::array();
  Json tool_changes = Json::array(), phases = Json::array(), postures = Json::array();
  for (const CheckRow& row : report.rows) rows.push_back(RowJson(row));
  for (const BayStep& s : report.steps) steps.push_back(StepJson(s));
  for (const SolvedPoint& p : report.points) points.push_back(PointJson(p));
  for (const SolvedToolChange& t : report.tool_changes) {
    tool_changes.push_back(Json{{"arm", t.arm}, {"deg", Joints(t.deg)}, {"reached", t.reached}});
  }
  for (const auto& [phase, seconds] : report.phases) {
    phases.push_back(Json::array({phase, Num(seconds)}));
  }
  for (const PostureChanges& c : report.postures) {
    postures.push_back(Json{{"arm", c.arm},
                            {"largest_s", Num(c.largest_s)},
                            {"from", c.from},
                            {"to", c.to},
                            {"above_2s", c.above_2s}});
  }
  r["rows"] = rows;
  r["steps"] = steps;
  r["bay_time_s"] = Num(report.bay_time_s);
  r["time_model"] = report.time_model;
  r["clearance_m"] = Num(report.clearance_m);
  r["joint_margin_deg"] = Num(report.joint_margin_deg);
  r["points"] = points;
  r["tool_changes"] = tool_changes;
  r["phases"] = phases;
  r["postures"] = postures;
  r["pass"] = report.pass;
  r["error"] = report.error;
  Json file;
  file["layout"] = name;
  file["layout_sha256"] = layout_sha256;
  file["report"] = r;
  return file.dump(2) + "\n";
}

std::optional<CheckReport> CheckReportFromJson(const std::string& text,
                                               const std::string& name,
                                               const std::string& layout_sha256,
                                               std::string* error) {
  try {
    const Json file = Json::parse(text);
    const std::string file_name = file.at("layout").get<std::string>();
    const std::string file_sha = file.at("layout_sha256").get<std::string>();
    if (file_name != name) {
      *error = "the report is of layout " + file_name + ", not " + name;
      return std::nullopt;
    }
    if (file_sha != layout_sha256) {
      *error = "the report was made from another version of layout " + name +
               ": its layout_sha256 is " + file_sha + ", the layout file's is " +
               layout_sha256;
      return std::nullopt;
    }
    const Json& j = file.at("report");
    CheckReport r;
    for (const Json& row : j.at("rows")) r.rows.push_back(ToRow(row));
    for (const Json& s : j.at("steps")) r.steps.push_back(ToStep(s));
    r.bay_time_s = ToNum(j.at("bay_time_s"));
    r.time_model = j.at("time_model").get<std::string>();
    r.clearance_m = ToNum(j.at("clearance_m"));
    r.joint_margin_deg = ToNum(j.at("joint_margin_deg"));
    for (const Json& p : j.at("points")) r.points.push_back(ToPoint(p));
    for (const Json& t : j.at("tool_changes")) {
      SolvedToolChange c;
      c.arm = t.at("arm").get<std::string>();
      c.deg = ToJoints(t.at("deg"));
      c.reached = t.at("reached").get<bool>();
      r.tool_changes.push_back(c);
    }
    for (const Json& p : j.at("phases")) {
      if (!p.is_array() || p.size() != 2) throw std::runtime_error("a phase is not [name, s]");
      r.phases.emplace_back(p[0].get<std::string>(), ToNum(p[1]));
    }
    for (const Json& p : j.at("postures")) {
      PostureChanges c;
      c.arm = p.at("arm").get<std::string>();
      c.largest_s = ToNum(p.at("largest_s"));
      c.from = p.at("from").get<std::string>();
      c.to = p.at("to").get<std::string>();
      c.above_2s = p.at("above_2s").get<int>();
      r.postures.push_back(c);
    }
    r.pass = j.at("pass").get<bool>();
    r.error = j.at("error").get<std::string>();
    return r;
  } catch (const std::exception& e) {
    *error = std::string("the report does not read: ") + e.what();
    return std::nullopt;
  }
}

std::optional<CheckReport> ReadCheckReportFile(const std::string& path,
                                               const std::string& name,
                                               const std::string& layout_sha256,
                                               std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    *error = "no report file " + path;
    return std::nullopt;
  }
  std::stringstream s;
  s << in.rdbuf();
  std::optional<CheckReport> r = CheckReportFromJson(s.str(), name, layout_sha256, error);
  if (!r) *error = path + ": " + *error;
  return r;
}

std::optional<CheckReport> CheckReportOf(const Layout& layout,
                                         const std::string& layout_sha256,
                                         const ModelPaths& paths,
                                         const std::string& report_dir,
                                         std::string* error) {
  if (!HasOnePosturePerModule(layout)) return CheckLayout(layout, paths);
  return ReadCheckReportFile(report_dir + "/" + layout.name + ".json", layout.name,
                             layout_sha256, error);
}

bool HasOnePosturePerModule(const Layout& layout) {
  for (const auto& arm : layout.arms) {
    if (arm.one_posture_per_module) return true;
  }
  return false;
}

std::string ReportDir() {
  const char* dir = std::getenv("CHECK_REPORT_DIR");
  return dir != nullptr ? dir : "check_reports";
}

}  // namespace cell::layout
