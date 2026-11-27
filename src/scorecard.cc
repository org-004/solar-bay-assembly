// The scorecard of a layout (src/scorecard.h): the counting rules, the
// operations of a plan, the gates, the committed JSON and the text.
//
// Every figure that the checker or the planner computes is taken from
// CheckLayout, VerifyPlan and TransportMassParts; nothing here computes one
// a second time. Each double is rounded where it is made (Round, Sig6), so
// ScorecardToJson gives the same text on every run.
#include "src/scorecard.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "src/flat_rack.h"
#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_rows.h"
#include "src/physics_record.h"

namespace cell::layout {

namespace {

using OJson = nlohmann::ordered_json;

double Round(double v, int decimals) {
  const double scale = std::pow(10.0, decimals);
  const double r = std::round(v * scale) / scale;
  return r == 0.0 ? 0.0 : r;
}

// Six significant digits.
double Sig6(double v) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.5e", v);
  return std::strtod(buffer, nullptr);
}

std::string ReadFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
}

// A number as text: integers without decimals, others as the shortest text
// that reads back the same.
std::string Num(double v) {
  if (std::floor(v) == v && std::fabs(v) < 1e15) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.0f", v);
    return buffer;
  }
  return OJson(v).dump();
}

// Rows of text, each column padded to its widest cell except the last.
std::string Aligned(const std::vector<std::vector<std::string>>& rows) {
  std::vector<size_t> width;
  for (const auto& row : rows) {
    for (size_t i = 0; i < row.size(); ++i) {
      if (width.size() <= i) width.resize(i + 1, 0);
      width[i] = std::max(width[i], row[i].size());
    }
  }
  std::string out;
  for (const auto& row : rows) {
    std::string line;
    for (size_t i = 0; i < row.size(); ++i) {
      std::string cell = row[i];
      if (i + 1 < row.size()) cell.resize(width[i], ' ');
      if (i > 0) line += "  ";
      line += cell;
    }
    out += line + "\n";
  }
  return out;
}

// The first word of a step's text, or its whole text for the steps of
// several words.
std::string StepHead(const std::string& what) {
  for (const char* whole : {"tube in", "tool change", "back to wait", "bay out"}) {
    if (what == whole) return what;
  }
  return what.substr(0, what.find(' '));
}

Count MakeCount(const std::string& kind, int count) {
  Count c;
  c.kind = kind;
  c.count = count;
  return c;
}

bool HasStand(const Tool& tool) {
  return tool.wait_size[0] != 0.0 || tool.wait_size[1] != 0.0 ||
         tool.wait_size[2] != 0.0;
}

bool IsLeftOut(const MassPart& part) { return part.component == "stocks inside"; }

const char* const kLeftOutWhy =
    "infed parts: pin magazines with their pins, which the owner's "
    "definition leaves out";

int RowsFailed(const CheckReport& report) {
  int failed = 0;
  for (const CheckRow& row : report.rows) failed += row.pass ? 0 : 1;
  return failed;
}

Measure MakeMeasure(const std::string& name, const std::string& rank,
                    std::optional<double> value, const std::string& unit,
                    const std::string& better, const std::string& known,
                    const std::string& basis) {
  Measure m;
  m.name = name;
  m.rank = rank;
  m.value = value;
  m.unit = unit;
  m.better = better;
  m.known = known;
  m.basis = basis;
  return m;
}

int Total(const std::vector<Count>& counts) {
  int sum = 0;
  for (const Count& c : counts) sum += c.count;
  return sum;
}

std::string Percent(double part, double whole) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.0f%%",
                whole > 0.0 ? 100.0 * part / whole : 0.0);
  return buffer;
}

}  // namespace

const Measure* FindMeasure(const Scorecard& card, const std::string& name) {
  for (const Measure& m : card.measures) {
    if (m.name == name) return &m;
  }
  return nullptr;
}

std::optional<std::vector<Rate>> LoadRates(const std::string& path,
                                           std::string* error) {
  const std::string text = ReadFile(path);
  if (text.empty()) {
    *error = "cannot read " + path;
    return std::nullopt;
  }
  try {
    const nlohmann::json j = nlohmann::json::parse(text);
    std::vector<Rate> rates;
    std::set<std::string> seen;
    for (const nlohmann::json& k : j.at("kinds")) {
      Rate r;
      r.kind = k.at("kind").get<std::string>();
      r.what = k.at("what").get<std::string>();
      r.rate = k.at("rate").get<double>();
      r.basis = k.at("basis").get<std::string>();
      r.source = k.at("source").get<std::string>();
      r.reasoning = k.at("reasoning").get<std::string>();
      if (!(r.rate >= 0.0 && r.rate <= 1.0)) {
        *error = path + ": the rate of " + r.kind + " is outside [0, 1]";
        return std::nullopt;
      }
      if (r.basis != "source" && r.basis != "ours") {
        *error = path + ": the basis of " + r.kind +
                 " is neither source nor ours";
        return std::nullopt;
      }
      if (r.basis == "source" && r.source.empty()) {
        *error = path + ": " + r.kind + " has basis source and no source";
        return std::nullopt;
      }
      if (!seen.insert(r.kind).second) {
        *error = path + ": the kind " + r.kind + " is there twice";
        return std::nullopt;
      }
      rates.push_back(r);
    }
    return rates;
  } catch (const nlohmann::json::exception& e) {
    *error = path + ": " + e.what();
    return std::nullopt;
  }
}

std::vector<std::string> OperationKinds(const StepTime& step) {
  const std::string head = StepHead(step.what);
  if (head == "tube in") return {"tube_in"};
  if (head == "clamp") return {"clamp"};
  if (head == "release") return {"unclamp"};
  if (head == "bay out") return {"bay_out"};
  if (head == "back to wait") return {"arm_move"};
  if (head == "tool change") return {"tool_change", "arm_move"};
  if (head == "take") return {"grip", "arm_move"};
  if (head == "set") return {"set_down", "arm_move"};
  if (head == "bolt") return {"bolt_run", "arm_move"};
  if (head == "locate") return {"locate", "arm_move"};
  if (head == "pin") return {"pin_set", "arm_move"};
  if (head == "swage") return {"swage", "arm_move"};
  // A staged layout's transfer: the holder lets the tube go, the infeed and
  // outfeed units move it by whole pitches and seat it, and the holder grips
  // it again.
  if (step.what.rfind("move the tube to stage ", 0) == 0) {
    return {"unclamp", "tube_in", "clamp"};
  }
  return {};
}

std::optional<std::vector<OperationCount>> CountOperations(
    const Plan& plan, const std::vector<Rate>& rates, std::string* error) {
  std::vector<OperationCount> out;
  for (const Rate& r : rates) {
    OperationCount o;
    o.kind = r.kind;
    o.rate = Sig6(r.rate);
    o.basis = r.basis;
    out.push_back(o);
  }
  for (const StepTime& step : plan.steps) {
    if (step.bay != 0) continue;  // One bay's operations.
    const std::vector<std::string> kinds = OperationKinds(step);
    const std::string where =
        "step " + std::to_string(step.number) + " \"" + step.what + "\"";
    if (kinds.empty()) {
      *error = where + " maps to no kind of operation";
      return std::nullopt;
    }
    for (const std::string& kind : kinds) {
      OperationCount* found = nullptr;
      for (OperationCount& o : out) {
        if (o.kind == kind) found = &o;
      }
      if (found == nullptr) {
        *error = where + " maps to " + kind + ", which rates.json does not hold";
        return std::nullopt;
      }
      ++found->count;
    }
  }
  for (OperationCount& o : out) o.expected = Sig6(o.count * o.rate);
  return out;
}

std::vector<Count> PartCount(const Layout& layout) {
  int arms = 0, tracks = 0, tools = 0, stands = 0, magazines = 0;
  for (const ArmUse& a : layout.arms) {
    ++arms;
    tracks += a.track ? 1 : 0;
    for (const Tool& t : a.tools) {
      ++tools;
      stands += HasStand(t) ? 1 : 0;
    }
  }
  for (const Stock& s : layout.stocks) {
    if (!s.outside && s.part == "pin magazine") magazines += s.holds;
  }
  const int grips = static_cast<int>(layout.holder.grips.size());
  return {MakeCount("arm", arms),
          MakeCount("track", tracks),
          MakeCount("tool", tools),
          MakeCount("tool_stand", stands),
          MakeCount("holder_stand", grips),
          MakeCount("holder_grip", grips),
          MakeCount("holder_end_unit", 1),
          MakeCount("item", static_cast<int>(layout.items.size())),
          MakeCount("pin_magazine", magazines)};
}

std::vector<Count> DrivenAxes(const Layout& layout) {
  int joints = 0, tracks = 0, tools = 0;
  for (const ArmUse& a : layout.arms) {
    joints += 6;
    tracks += a.track ? 1 : 0;
    tools += static_cast<int>(a.tools.size());
  }
  return {MakeCount("arm_joint", joints), MakeCount("track", tracks),
          MakeCount("tool", tools),
          MakeCount("holder_grip", static_cast<int>(layout.holder.grips.size())),
          MakeCount("holder_angle_pin", 1)};
}

std::vector<MassPart> MassInTransit(const Layout& layout) {
  std::vector<MassPart> parts;
  for (MassPart p : TransportMassParts(layout)) {
    if (IsLeftOut(p)) continue;
    p.mass_kg = Round(p.mass_kg, 1);
    parts.push_back(p);
  }
  return parts;
}

std::vector<MassLeftOut> MassLeftOutOfTransit(const Layout& layout) {
  std::vector<MassLeftOut> parts;
  for (MassPart p : TransportMassParts(layout)) {
    if (!IsLeftOut(p)) continue;
    p.mass_kg = Round(p.mass_kg, 1);
    MassLeftOut left;
    left.part = p;
    left.why = kLeftOutWhy;
    parts.push_back(left);
  }
  return parts;
}

namespace {

// physics_share: 0 without a record or with one that does not agree with the
// committed plan, else the planned seconds that count over one_bay_s.
Measure PhysicsShare(const std::string& name, const Plan& plan, double one_bay_s,
                     const std::string& plan_sha256,
                     const PhysicsRecord* physics) {
  if (physics == nullptr) {
    return MakeMeasure("physics_share", "P0", 0.0, "1", "higher", "counted",
                       "no committed, repeatable run in physics of this "
                       "layout's arms and parts");
  }
  const std::string file = "physics/" + name + ".json";
  const std::vector<std::string> faults =
      CheckPhysicsRecord(*physics, plan, plan_sha256);
  if (physics->plan_sha256 != plan_sha256 || !faults.empty()) {
    return MakeMeasure(
        "physics_share", "P0", 0.0, "1", "higher", "counted",
        file + " does not agree with the committed plan: " +
            (faults.empty() ? "plan_sha256 differs" : faults.front()));
  }
  std::string steps;
  for (const PhysicsStep& s : physics->steps) {
    if (!StepCounts(*physics, s)) continue;
    steps += (steps.empty() ? "" : ", ") + std::to_string(s.number);
  }
  char seconds[64];
  std::snprintf(seconds, sizeof(seconds), "%.6f s of one bay's %.6f s counted",
                CountedSeconds(*physics), one_bay_s);
  return MakeMeasure(
      "physics_share", "P0",
      Round(CountedSeconds(*physics) / one_bay_s, 6), "1", "higher",
      "measured",
      file + ": " + (steps.empty() ? "no step" : "steps " + steps) +
          " of module " + std::to_string(physics->module) +
          " carried out in physics, " + seconds);
}

}  // namespace

std::optional<Scorecard> ComputeScorecard(
    const Layout& layout, const Plan& plan, const std::vector<Rate>& rates,
    const ModelPaths& paths, const std::string& layout_sha256,
    const std::string& plan_sha256, const std::string& rates_sha256,
    const PhysicsRecord* physics, std::string* error) {
  Scorecard card;
  card.layout = layout.name;
  card.layout_sha256 = layout_sha256;
  card.plan_sha256 = plan_sha256;
  card.rates_sha256 = rates_sha256;
  const std::optional<std::vector<OperationCount>> ops =
      CountOperations(plan, rates, error);
  if (!ops) return std::nullopt;
  card.operations = *ops;
  const CheckReport report = CheckLayout(layout, paths);
  if (!report.error.empty()) {
    *error = "the model cannot be built: " + report.error;
    return std::nullopt;
  }
  const VerifyReport verify = VerifyPlan(layout, paths, plan, layout_sha256);
  card.part_count = PartCount(layout);
  card.driven_axes = DrivenAxes(layout);
  card.mass_in_transit = MassInTransit(layout);
  card.mass_left_out = MassLeftOutOfTransit(layout);

  double failures = 0.0, on_ours = 0.0;
  for (const OperationCount& o : card.operations) {
    failures += o.expected;
    if (o.basis == "ours") on_ours += o.expected;
  }
  double mass = 0.0;
  for (const MassPart& p : card.mass_in_transit) mass += p.mass_kg;
  mass = Round(mass, 1);
  const int arms = static_cast<int>(layout.arms.size());

  const CycleFigures cycle = CycleFiguresOf(plan);
  const double one_bay = cycle.bays > 0 ? cycle.one_bay_s : plan.bay_planned_s;
  const int bays = std::max(1, cycle.bays);
  char how[256];
  std::snprintf(how, sizeof(how),
                "the committed plan's %d bays back to back: (end of bay %d's bay_out - "
                "start of bay 0's tube_in) / %d; kinematic, on assumed speeds and "
                "process times",
                bays, bays - 1, bays);
  card.measures = {
      MakeMeasure("cycle_s", "P0", Round(cycle.bays > 0 ? cycle.cycle_s : one_bay, 6), "s",
                  "lower", "planned",
                  bays > 1 ? std::string(how)
                           : "the committed plan's one bay, from tube in to bay out: "
                             "kinematic, on assumed speeds and process times"),
      MakeMeasure("failures_per_bay", "P0", Sig6(failures), "1", "lower",
                  "modelled",
                  "count of operations by kind x rates of scores/rates.json; " +
                      Percent(on_ours, failures) +
                      " of it on rates of basis ours"),
      PhysicsShare(layout.name, plan, one_bay, plan_sha256, physics),
      MakeMeasure("part_count", "P1", Total(card.part_count), "1", "lower",
                  "counted", "pieces of equipment by the rule of "
                             "docs/scorecard.md"),
      MakeMeasure("arms", "P1", arms, "1", "lower", "counted",
                  "arms of the layout"),
      MakeMeasure("driven_axes", "P1", Total(card.driven_axes), "1", "lower",
                  "counted",
                  "arm joints, tracks, driven tools and grips by the rule of "
                  "docs/scorecard.md"),
      MakeMeasure("placement_mm", "P1", std::nullopt, "mm", "lower", "none",
                  "nothing is held or sensed in this cell; the stand-in "
                  "cell's figure is another cell's"),
      MakeMeasure("mass_in_transit_kg", "P2", mass, "kg", "lower", "counted",
                  "the transport row's parts less infed parts (owner's "
                  "definition)"),
      MakeMeasure("input_tolerance", "P2", std::nullopt, "mm", "higher", "none",
                  "nothing is sensed in this cell, so no error of the infed "
                  "parts is measured or corrected"),
      MakeMeasure("plan_s", "P2", std::nullopt, "s", "lower", "none",
                  "the committed plan does not hold the time plan_layout "
                  "took"),
  };

  card.figures = {
      {"one_bay_s", Round(one_bay, 6), "s", "planned",
       "bay 0 of the committed plan alone, from the start of its tube_in to the end of "
       "its bay_out"},
      bays > 1 ? Figure{"interval_s", Round(cycle.interval_s, 6), "s", "planned",
                        "from the end of bay " + std::to_string(bays - 2) +
                            "'s bay_out to the end of bay " + std::to_string(bays - 1) + "'s"}
               : Figure{"interval_s", std::nullopt, "s", "none",
                        "the committed plan holds one bay"},
  };

  Gate rows;
  rows.name = "check_rows_failed";
  rows.figure = RowsFailed(report);
  rows.limit = 0.0;
  rows.unit = "rows";
  rows.holds = "<=";
  rows.pass = rows.figure <= rows.limit;
  rows.basis = "rows of check_layout that do not pass, of " +
               std::to_string(report.rows.size());
  Gate clearance;
  clearance.name = "least_clearance_m";
  clearance.limit = layout.margins.clearance_m;
  clearance.unit = "m";
  clearance.holds = ">=";
  if (verify.ok) {
    // Held against the limit unrounded; rounded only for the output.
    clearance.pass = verify.least_clearance_m >= clearance.limit;
    clearance.figure = Round(verify.least_clearance_m, 4);
    clearance.basis =
        "least distance over the plan's schedule, from VerifyPlan";
  } else {
    clearance.pass = false;
    clearance.basis = "the plan does not verify: " +
                      (verify.faults.empty() ? std::string("no reason given")
                                             : verify.faults.front());
  }
  Gate load;
  load.name = "mass_in_transit_kg";
  load.figure = mass;
  load.limit = flat_rack::kTransitMassLimitKg;
  load.unit = "kg";
  load.holds = "<=";
  load.pass = load.figure <= load.limit;
  load.basis = "the owner's limit for one container, 30,000 lb";
  card.gates = {rows, clearance, load};
  card.pass = true;
  for (const Gate& g : card.gates) card.pass = card.pass && g.pass;
  return card;
}

std::string ScorecardToJson(const Scorecard& card) {
  OJson j = OJson::object();
  j["layout"] = card.layout;
  j["layout_sha256"] = card.layout_sha256;
  j["plan_sha256"] = card.plan_sha256;
  j["rates_sha256"] = card.rates_sha256;
  if (!card.physics_sha256.empty()) j["physics_sha256"] = card.physics_sha256;
  OJson measures = OJson::array();
  for (const Measure& m : card.measures) {
    OJson e = OJson::object();
    e["name"] = m.name;
    e["rank"] = m.rank;
    e["value"] = m.value ? OJson(*m.value) : OJson(nullptr);
    e["unit"] = m.unit;
    e["better"] = m.better;
    e["known"] = m.known;
    e["basis"] = m.basis;
    measures.push_back(e);
  }
  j["measures"] = measures;
  OJson figures = OJson::array();
  for (const Figure& f : card.figures) {
    OJson e = OJson::object();
    e["name"] = f.name;
    e["value"] = f.value ? OJson(*f.value) : OJson(nullptr);
    e["unit"] = f.unit;
    e["known"] = f.known;
    e["basis"] = f.basis;
    figures.push_back(e);
  }
  j["figures"] = figures;
  OJson gates = OJson::array();
  for (const Gate& g : card.gates) {
    OJson e = OJson::object();
    e["name"] = g.name;
    e["figure"] = g.figure;
    e["limit"] = g.limit;
    e["unit"] = g.unit;
    e["holds"] = g.holds;
    e["pass"] = g.pass;
    e["basis"] = g.basis;
    gates.push_back(e);
  }
  j["gates"] = gates;
  const std::pair<const char*, const std::vector<Count>*> lists[] = {
      {"part_count", &card.part_count}, {"driven_axes", &card.driven_axes}};
  for (const auto& [key, counts] : lists) {
    OJson list = OJson::array();
    for (const Count& c : *counts) {
      OJson e = OJson::object();
      e["kind"] = c.kind;
      e["count"] = c.count;
      list.push_back(e);
    }
    j[key] = list;
  }
  OJson transit = OJson::array();
  for (const MassPart& p : card.mass_in_transit) {
    OJson e = OJson::object();
    e["component"] = p.component;
    e["name"] = p.name;
    e["mass_kg"] = p.mass_kg;
    transit.push_back(e);
  }
  j["mass_in_transit"] = transit;
  OJson left = OJson::array();
  for (const MassLeftOut& p : card.mass_left_out) {
    OJson e = OJson::object();
    e["component"] = p.part.component;
    e["name"] = p.part.name;
    e["mass_kg"] = p.part.mass_kg;
    e["why"] = p.why;
    left.push_back(e);
  }
  j["mass_left_out"] = left;
  OJson ops = OJson::array();
  for (const OperationCount& o : card.operations) {
    OJson e = OJson::object();
    e["kind"] = o.kind;
    e["count"] = o.count;
    e["rate"] = o.rate;
    e["basis"] = o.basis;
    e["expected"] = o.expected;
    ops.push_back(e);
  }
  j["operations"] = ops;
  return j.dump(2) + "\n";
}

std::optional<Scorecard> ScorecardFromJson(const std::string& text,
                                           std::string* error) {
  try {
    const nlohmann::json j = nlohmann::json::parse(text);
    Scorecard card;
    card.layout = j.at("layout").get<std::string>();
    card.layout_sha256 = j.at("layout_sha256").get<std::string>();
    card.plan_sha256 = j.at("plan_sha256").get<std::string>();
    card.rates_sha256 = j.at("rates_sha256").get<std::string>();
    if (j.contains("physics_sha256")) {
      card.physics_sha256 = j.at("physics_sha256").get<std::string>();
    }
    for (const nlohmann::json& e : j.at("measures")) {
      Measure m;
      m.name = e.at("name").get<std::string>();
      m.rank = e.at("rank").get<std::string>();
      if (!e.at("value").is_null()) m.value = e.at("value").get<double>();
      m.unit = e.at("unit").get<std::string>();
      m.better = e.at("better").get<std::string>();
      m.known = e.at("known").get<std::string>();
      m.basis = e.at("basis").get<std::string>();
      card.measures.push_back(m);
    }
    if (j.contains("figures")) {
      for (const nlohmann::json& e : j.at("figures")) {
        Figure f;
        f.name = e.at("name").get<std::string>();
        if (!e.at("value").is_null()) f.value = e.at("value").get<double>();
        f.unit = e.at("unit").get<std::string>();
        f.known = e.at("known").get<std::string>();
        f.basis = e.at("basis").get<std::string>();
        card.figures.push_back(f);
      }
    }
    card.pass = true;
    for (const nlohmann::json& e : j.at("gates")) {
      Gate g;
      g.name = e.at("name").get<std::string>();
      g.figure = e.at("figure").get<double>();
      g.limit = e.at("limit").get<double>();
      g.unit = e.at("unit").get<std::string>();
      g.holds = e.at("holds").get<std::string>();
      g.pass = e.at("pass").get<bool>();
      g.basis = e.at("basis").get<std::string>();
      card.pass = card.pass && g.pass;
      card.gates.push_back(g);
    }
    for (const nlohmann::json& e : j.at("part_count")) {
      card.part_count.push_back(
          MakeCount(e.at("kind").get<std::string>(), e.at("count").get<int>()));
    }
    for (const nlohmann::json& e : j.at("driven_axes")) {
      card.driven_axes.push_back(
          MakeCount(e.at("kind").get<std::string>(), e.at("count").get<int>()));
    }
    for (const nlohmann::json& e : j.at("mass_in_transit")) {
      card.mass_in_transit.push_back({e.at("component").get<std::string>(),
                                      e.at("name").get<std::string>(),
                                      e.at("mass_kg").get<double>()});
    }
    for (const nlohmann::json& e : j.at("mass_left_out")) {
      MassLeftOut p;
      p.part = {e.at("component").get<std::string>(),
                e.at("name").get<std::string>(), e.at("mass_kg").get<double>()};
      p.why = e.at("why").get<std::string>();
      card.mass_left_out.push_back(p);
    }
    for (const nlohmann::json& e : j.at("operations")) {
      OperationCount o;
      o.kind = e.at("kind").get<std::string>();
      o.count = e.at("count").get<int>();
      o.rate = e.at("rate").get<double>();
      o.basis = e.at("basis").get<std::string>();
      o.expected = e.at("expected").get<double>();
      card.operations.push_back(o);
    }
    return card;
  } catch (const nlohmann::json::exception& e) {
    *error = std::string("not a scorecard: ") + e.what();
    return std::nullopt;
  }
}

std::string ScorecardText(const Scorecard& card) {
  std::string out = "# scorecard of " + card.layout + ": layout sha256 " +
                    card.layout_sha256.substr(0, 12) + ", plan sha256 " +
                    card.plan_sha256.substr(0, 12) + "\n";
  std::vector<std::vector<std::string>> table = {
      {"rank", "name", "value", "unit", "better", "known", "basis"}};
  for (const Measure& m : card.measures) {
    table.push_back({m.rank, m.name, m.value ? Num(*m.value) : "none", m.unit,
                     m.better, m.known, m.basis});
  }
  out += Aligned(table) + "\n";
  if (!card.figures.empty()) {
    table = {{"figure", "value", "unit", "known", "basis"}};
    for (const Figure& f : card.figures) {
      table.push_back({f.name, f.value ? Num(*f.value) : "none", f.unit, f.known, f.basis});
    }
    out += "# beside cycle_s, not ranked\n" + Aligned(table) + "\n";
  }
  table = {{"gate", "figure", "limit", "unit", "pass", "basis"}};
  for (const Gate& g : card.gates) {
    table.push_back({g.name, Num(g.figure), g.holds + " " + Num(g.limit), g.unit,
                     g.pass ? "pass" : "FAIL", g.basis});
  }
  out += Aligned(table);
  const std::pair<const char*, const std::vector<Count>*> lists[] = {
      {"part_count", &card.part_count}, {"driven_axes", &card.driven_axes}};
  for (const auto& [title, counts] : lists) {
    table = {{"kind", "count"}};
    for (const Count& c : *counts) table.push_back({c.kind, Num(c.count)});
    out += std::string("\n# ") + title + " by kind\n" + Aligned(table);
  }
  table = {{"component", "name", "kg", "why"}};
  for (const MassPart& p : card.mass_in_transit) {
    table.push_back({p.component, p.name, Num(p.mass_kg), ""});
  }
  for (const MassLeftOut& p : card.mass_left_out) {
    table.push_back({p.part.component, p.part.name, Num(p.part.mass_kg),
                     "left out: " + p.why});
  }
  out += "\n# mass_in_transit_kg by part\n" + Aligned(table);
  table = {{"kind", "count", "rate", "basis", "expected"}};
  for (const OperationCount& o : card.operations) {
    table.push_back({o.kind, Num(o.count), Num(o.rate), o.basis, Num(o.expected)});
  }
  out += "\n# operations by kind\n" + Aligned(table);
  return out;
}

std::string ScorecardsTable(const std::vector<Scorecard>& cards) {
  std::vector<std::vector<std::string>> table;
  std::vector<std::string> head = {"layout"};
  if (!cards.empty()) {
    for (const Measure& m : cards.front().measures) {
      head.push_back(m.name);
      if (m.name != "cycle_s") continue;
      for (const Figure& f : cards.front().figures) head.push_back(f.name);
    }
  }
  head.push_back("gates");
  table.push_back(head);
  for (const Scorecard& c : cards) {
    std::vector<std::string> row = {c.layout};
    for (const Measure& m : c.measures) {
      row.push_back(m.value ? Num(*m.value) : "none");
      if (m.name != "cycle_s") continue;
      for (const Figure& f : c.figures) row.push_back(f.value ? Num(*f.value) : "none");
    }
    std::string failing;
    for (const Gate& g : c.gates) {
      if (!g.pass) failing += (failing.empty() ? "" : ", ") + g.name;
    }
    row.push_back(failing.empty() ? "pass" : "FAIL: " + failing);
    table.push_back(row);
  }
  return Aligned(table);
}

std::optional<std::vector<Version>> LoadVersions(const std::string& path,
                                                 std::string* error) {
  const std::string text = ReadFile(path);
  if (text.empty()) {
    *error = "cannot read " + path;
    return std::nullopt;
  }
  try {
    const nlohmann::json j = nlohmann::json::parse(text);
    std::vector<Version> versions;
    auto string_of = [](const nlohmann::json& e, const char* key) {
      return e.contains(key) && e.at(key).is_string()
                 ? e.at(key).get<std::string>()
                 : std::string();
    };
    auto file_of = [&](const nlohmann::json& e, const char* key) {
      std::optional<VersionFile> f;
      if (e.contains(key)) {
        f = VersionFile{e.at(key).at("path").get<std::string>(),
                        e.at(key).at("sha256").get<std::string>()};
      }
      return f;
    };
    for (const nlohmann::json& e : j.at("versions")) {
      Version v;
      v.id = e.at("id").get<std::string>();
      v.date = string_of(e, "date");
      v.commit = string_of(e, "commit");
      v.title = string_of(e, "title");
      v.direction = string_of(e, "direction");
      v.status = string_of(e, "status");
      v.recording = string_of(e, "recording");
      v.layout = file_of(e, "layout");
      v.plan = file_of(e, "plan");
      v.scores = string_of(e, "scores");
      versions.push_back(v);
    }
    return versions;
  } catch (const nlohmann::json::exception& e) {
    *error = path + ": " + e.what();
    return std::nullopt;
  }
}


// ------------------------------------------------------- //:score_layout

namespace {


std::string Env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

bool Exists(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  return file.good();
}

// The scorecard of `name` from its file when that is fresh, else computed
// afresh (with a note on stderr). Nothing when it cannot be had.
std::optional<Scorecard> Get(const std::string& name, bool compute) {
  const std::string layout_path = Env("LAYOUT_DIR", "layouts") + "/" + name + ".json";
  const std::string plan_path = Env("PLAN_DIR", "plans") + "/" + name + ".json";
  const std::string scores_dir = Env("SCORES_DIR", "scores");
  const std::string rates_path = scores_dir + "/rates.json";
  const std::string layout_sha = Sha256OfFile(layout_path);
  const std::string plan_sha = Sha256OfFile(plan_path);
  const std::string rates_sha = Sha256OfFile(rates_path);
  const std::string physics_path =
      Env("PHYSICS_DIR", "physics") + "/" + name + ".json";
  const std::string physics_sha =
      Exists(physics_path) ? Sha256OfFile(physics_path) : "";
  std::string error;
  if (!compute) {
    const std::string text = ReadFile(scores_dir + "/" + name + ".json");
    const std::optional<Scorecard> card =
        text.empty() ? std::nullopt : ScorecardFromJson(text, &error);
    if (card && card->layout_sha256 == layout_sha &&
        card->plan_sha256 == plan_sha && card->rates_sha256 == rates_sha &&
        card->physics_sha256 == physics_sha) {
      return card;
    }
    std::fprintf(stderr,
                 "scores/%s.json is missing or stale; computed afresh (write "
                 "it with --write)\n",
                 name.c_str());
  }
  const std::optional<Layout> layout =
      LoadLayout(layout_path, &error);
  if (!layout) {
    std::fprintf(stderr, "score_layout: %s\n", error.c_str());
    return std::nullopt;
  }
  const std::optional<Plan> plan =
      PlanFromJson(ReadFile(plan_path), &error);
  if (!plan) {
    std::fprintf(stderr, "score_layout: %s: %s\n", plan_path.c_str(), error.c_str());
    return std::nullopt;
  }
  const auto rates = LoadRates(rates_path, &error);
  if (!rates) {
    std::fprintf(stderr, "score_layout: %s\n", error.c_str());
    return std::nullopt;
  }
  std::optional<PhysicsRecord> record;
  if (!physics_sha.empty()) {
    record = PhysicsRecordFromJson(ReadFile(physics_path), &error);
    if (!record) {
      std::fprintf(stderr, "score_layout: %s: %s\n", physics_path.c_str(),
                   error.c_str());
      return std::nullopt;
    }
  }
  std::optional<Scorecard> card = ComputeScorecard(
      *layout, *plan, *rates, ModelPaths::FromEnvironment(),
      layout_sha, plan_sha, rates_sha, record ? &*record : nullptr, &error);
  if (!card) {
    std::fprintf(stderr, "score_layout: %s: %s\n", name.c_str(), error.c_str());
  } else {
    card->physics_sha256 = physics_sha;
  }
  return card;
}

bool Write(const Scorecard& card) {
  const std::string dir =
      Env("BUILD_WORKSPACE_DIRECTORY", "").empty()
          ? Env("SCORES_DIR", "scores")
          : Env("BUILD_WORKSPACE_DIRECTORY", "") + "/scores";
  const std::string path = dir + "/" + card.layout + ".json";
  std::ofstream file(path, std::ios::binary);
  file << ScorecardToJson(card);
  file.close();
  if (!file) {
    std::fprintf(stderr, "score_layout: cannot write %s\n", path.c_str());
    return false;
  }
  std::fprintf(stderr, "wrote %s\n", path.c_str());
  return true;
}

int Usage() {
  std::fprintf(stderr,
               "usage: score_layout <name> [--json | --write]\n"
               "       score_layout --all [--write]\n");
  return 2;
}

// The layouts that have a scorecard file, from SCORES_DIR, or with --write
// every layout that has a plan.
std::vector<std::string> AllNames(bool write) {
  std::vector<std::string> names;
  for (const std::string& n :
       ListLayouts(write ? Env("PLAN_DIR", "plans")
                                       : Env("SCORES_DIR", "scores"))) {
    if (n != "rates") names.push_back(n);
  }
  return names;
}

int All(bool write, std::string* out) {
  std::vector<Scorecard> cards;
  for (const std::string& name : AllNames(write)) {
    const std::optional<Scorecard> card = Get(name, write);
    if (!card) return 2;
    if (write && !Write(*card)) return 2;
    cards.push_back(*card);
  }
  *out += ScorecardsTable(cards);
  for (const Scorecard& c : cards) {
    if (!c.pass) return 1;
  }
  return 0;
}

}  // namespace


int RunScoreLayout(const std::vector<std::string>& args, std::string* out) {
  std::string name;
  bool json = false, write = false, all = false;
  for (const std::string& a : args) {
    if (a == "--json") {
      json = true;
    } else if (a == "--write") {
      write = true;
    } else if (a == "--all") {
      all = true;
    } else if ((!a.empty() && a[0] == '-') || !name.empty()) {
      return Usage();
    } else {
      name = a;
    }
  }
  if (all) return name.empty() && !json ? All(write, out) : Usage();
  if (name.empty()) return Usage();
  const std::string plan = Env("PLAN_DIR", "plans") + "/" + name + ".json";
  if (!Exists(plan)) {
    *out += name + ": no committed plan, so no scorecard (plans/" + name +
            ".json does not exist)\n";
    return 2;
  }
  if (!Exists(Env("LAYOUT_DIR", "layouts") + "/" + name + ".json")) {
    *out += name + ": no such layout (layouts/" + name +
            ".json does not exist)\n";
    return 2;
  }
  const std::optional<Scorecard> card = Get(name, write);
  if (!card) return 2;
  if (write && !Write(*card)) return 2;
  *out += json ? ScorecardToJson(*card)
              : ScorecardText(*card);
  return card->pass ? 0 : 1;
}

}  // namespace cell::layout
