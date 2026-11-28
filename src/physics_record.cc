// The committed record of a run in physics (src/physics_record.h): the rule
// by which a step counts, the text of the file and what is checked against a
// committed plan.
//
// Each double is rounded where it is written (Round), as in the scorecard, so
// that the same record is the same text and a record whose figures already
// have the precisions of the header reads back as it was.
#include "src/physics_record.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "src/bay_hardware.h"
#include "src/layout_plan.h"

namespace cell::layout {

namespace {

using OJson = nlohmann::ordered_json;

double Round(double v, int decimals) {
  const double scale = std::pow(10.0, decimals);
  const double r = std::round(v * scale) / scale;
  return r == 0.0 ? 0.0 : r;
}

// Seconds and metres to 6 decimals, millimetres, milliradians and degrees to 4.
double Seconds(double v) { return Round(v, 6); }
double Fine(double v) { return Round(v, 4); }

constexpr double kSecondsTolerance = 1e-6;

}  // namespace

bool StepCounts(const PhysicsRecord& record, const PhysicsTake& take,
                const PhysicsStep& step) {
  return step.least_clearance_m >= record.clearance_m &&
         take.set_down.offset_mm <= record.gate_offset_mm &&
         std::abs(take.set_down.yaw_deg) <= record.gate_yaw_deg;
}

RailGate RailPlay() {
  constexpr double kDeg = 180.0 / std::numbers::pi;
  RailGate g;
  g.across_mm = Fine(bay::kTabPlayAcrossTube * 1e3);
  g.along_mm = Fine(bay::kRailPlayAlongTube * 1e3);
  g.round_deg = Fine(bay::kRailPlayRound * kDeg);
  g.yaw_deg = Fine(bay::kRailPlayYaw * kDeg);
  return g;
}

bool RailSetCounts(const PhysicsRecord& record, const PhysicsRailSet& set) {
  bool motors = true;
  for (const double r : set.torque_ratio) motors = motors && r < 1.0;
  const RailGate& g = record.rail_gate;
  const RailSetPose& p = set.pose;
  return set.step.least_clearance_m >= record.clearance_m && motors &&
         std::abs(p.across_mm) <= g.across_mm && std::abs(p.along_mm) <= g.along_mm &&
         std::abs(p.round_deg) <= g.round_deg && std::abs(p.yaw_deg) <= g.yaw_deg;
}

double CountedSeconds(const PhysicsRecord& record) {
  double sum = 0.0;
  for (const PhysicsTake& take : record.takes) {
    for (const PhysicsStep& step : take.steps) {
      if (StepCounts(record, take, step)) sum += step.planned_s;
    }
  }
  for (const PhysicsRailSet& set : record.rails) {
    if (RailSetCounts(record, set)) sum += set.step.planned_s;
  }
  return sum;
}

std::vector<int> CountedRails(const PhysicsRecord& record) {
  std::vector<int> rails;
  for (const PhysicsRailSet& set : record.rails) {
    if (RailSetCounts(record, set)) rails.push_back(set.rail);
  }
  return rails;
}

std::string RailsText(const PhysicsRecord& record) {
  const std::vector<int> counted = CountedRails(record);
  std::string t = counted.empty() ? "no rail" : counted.size() == 1 ? "rail " : "rails ";
  for (size_t i = 0; i < counted.size(); ++i) {
    t += (i == 0 ? "" : ", ") + std::to_string(counted[i]);
  }
  return t + " (" + std::to_string(counted.size()) + " of the record's " +
         std::to_string(record.rails.size()) + " rail sets)";
}

std::vector<int> CountedSteps(const PhysicsRecord& record) {
  std::vector<int> steps;
  for (const PhysicsTake& take : record.takes) {
    for (const PhysicsStep& s : take.steps) {
      if (StepCounts(record, take, s)) steps.push_back(s.number);
    }
  }
  if (record.rails.empty()) return steps;
  for (const PhysicsRailSet& set : record.rails) {
    if (RailSetCounts(record, set)) steps.push_back(set.step.number);
  }
  std::sort(steps.begin(), steps.end());
  return steps;
}

std::vector<int> CountedModules(const PhysicsRecord& record) {
  std::vector<int> modules;
  for (const PhysicsTake& take : record.takes) {
    bool any = false;
    for (const PhysicsStep& step : take.steps) {
      any = any || StepCounts(record, take, step);
    }
    if (any) modules.push_back(take.module);
  }
  return modules;
}

std::optional<PhysicsRecord> JoinPhysicsRecords(
    const std::vector<PhysicsRecord>& records, const std::string& command,
    std::string* error) {
  if (records.empty()) {
    *error = "no record to join";
    return std::nullopt;
  }
  PhysicsRecord joined = records.front();
  joined.command = command;
  joined.takes.clear();
  joined.rails.clear();
  for (const PhysicsRecord& r : records) {
    if (r.rails.empty()) continue;
    const RailGate& a = r.rail_gate;
    const RailGate& b = joined.rail_gate;
    if (!joined.rails.empty() && (a.across_mm != b.across_mm || a.along_mm != b.along_mm ||
                                  a.round_deg != b.round_deg || a.yaw_deg != b.yaw_deg)) {
      *error = "the records' rail gates differ";
      return std::nullopt;
    }
    joined.rail_gate = a;
    joined.rails.insert(joined.rails.end(), r.rails.begin(), r.rails.end());
  }
  for (const PhysicsRecord& r : records) {
    if (r.layout != joined.layout || r.layout_sha256 != joined.layout_sha256 ||
        r.plan_sha256 != joined.plan_sha256 || r.clearance_m != joined.clearance_m ||
        r.gate_offset_mm != joined.gate_offset_mm ||
        r.gate_yaw_deg != joined.gate_yaw_deg) {
      *error = "the records are of different layouts, plans or gates";
      return std::nullopt;
    }
    joined.takes.insert(joined.takes.end(), r.takes.begin(), r.takes.end());
  }
  auto first_step = [](const PhysicsTake& t) {
    return t.steps.empty() ? 0 : t.steps.front().number;
  };
  std::stable_sort(joined.takes.begin(), joined.takes.end(),
                   [&](const PhysicsTake& x, const PhysicsTake& y) {
                     return first_step(x) < first_step(y);
                   });
  std::stable_sort(joined.rails.begin(), joined.rails.end(),
                   [](const PhysicsRailSet& x, const PhysicsRailSet& y) {
                     return x.step.number < y.step.number;
                   });
  for (PhysicsTake& take : joined.takes) {
    for (PhysicsStep& s : take.steps) s.counted = StepCounts(joined, take, s);
  }
  for (PhysicsRailSet& set : joined.rails) set.step.counted = RailSetCounts(joined, set);
  joined.counted_s = CountedSeconds(joined);
  return joined;
}

namespace {

OJson StepToJson(const PhysicsStep& s) {
    OJson e = OJson::object();
    e["number"] = s.number;
    e["what"] = s.what;
    e["arm"] = s.arm;
    e["start_s"] = Seconds(s.start_s);
    e["end_s"] = Seconds(s.end_s);
    e["planned_s"] = Seconds(s.planned_s);
    e["max_joint_error_mrad"] = Fine(s.max_joint_error_mrad);
    e["max_frame_error_mm"] = Fine(s.max_frame_error_mm);
    e["max_frame_error_deg"] = Fine(s.max_frame_error_deg);
    e["least_clearance_m"] = Seconds(s.least_clearance_m);
    e["least_clearance_pair"] = s.least_clearance_pair;
    e["least_clearance_t"] = Seconds(s.least_clearance_t);
    e["plan_least_clearance_m"] = Seconds(s.plan_least_clearance_m);
    e["counted"] = s.counted;
    return e;
}

OJson StepsToJson(const std::vector<PhysicsStep>& steps) {
  OJson out = OJson::array();
  for (const PhysicsStep& s : steps) out.push_back(StepToJson(s));
  return out;
}

OJson RailsToJson(const std::vector<PhysicsRailSet>& rails) {
  OJson out = OJson::array();
  for (const PhysicsRailSet& r : rails) {
    OJson e = OJson::object();
    e["rail"] = r.rail;
    e["step"] = StepToJson(r.step);
    OJson torque = OJson::array();
    for (const double t : r.torque_ratio) torque.push_back(Fine(t));
    e["torque_ratio"] = torque;
    OJson pose = OJson::object();
    pose["across_mm"] = Fine(r.pose.across_mm);
    pose["along_mm"] = Fine(r.pose.along_mm);
    pose["round_deg"] = Fine(r.pose.round_deg);
    pose["yaw_deg"] = Fine(r.pose.yaw_deg);
    pose["height_mm"] = Fine(r.pose.height_mm);
    e["pose"] = pose;
    out.push_back(e);
  }
  return out;
}

OJson SetDownToJson(const SetDown& d) {
  OJson out = OJson::object();
  out["offset_mm"] = Fine(d.offset_mm);
  out["across_mm"] = Fine(d.across_mm);
  out["along_mm"] = Fine(d.along_mm);
  out["yaw_deg"] = Fine(d.yaw_deg);
  out["height_mm"] = Fine(d.height_mm);
  return out;
}

PhysicsStep StepFromJson(const nlohmann::json& e) {
  PhysicsStep s;
  s.number = e.at("number").get<int>();
  s.what = e.at("what").get<std::string>();
  s.arm = e.at("arm").get<std::string>();
  s.start_s = e.at("start_s").get<double>();
  s.end_s = e.at("end_s").get<double>();
  s.planned_s = e.at("planned_s").get<double>();
  s.max_joint_error_mrad = e.at("max_joint_error_mrad").get<double>();
  s.max_frame_error_mm = e.at("max_frame_error_mm").get<double>();
  s.max_frame_error_deg = e.at("max_frame_error_deg").get<double>();
  s.least_clearance_m = e.at("least_clearance_m").get<double>();
  s.least_clearance_pair = e.at("least_clearance_pair").get<std::string>();
  s.least_clearance_t = e.at("least_clearance_t").get<double>();
  s.plan_least_clearance_m = e.at("plan_least_clearance_m").get<double>();
  s.counted = e.at("counted").get<bool>();
  return s;
}

PhysicsRailSet RailFromJson(const nlohmann::json& e) {
  PhysicsRailSet r;
  r.rail = e.at("rail").get<int>();
  r.step = StepFromJson(e.at("step"));
  const nlohmann::json& torque = e.at("torque_ratio");
  if (torque.size() != r.torque_ratio.size()) {
    throw std::runtime_error("a rail set's torque_ratio needs 7 values");
  }
  for (size_t i = 0; i < r.torque_ratio.size(); ++i) r.torque_ratio[i] = torque.at(i).get<double>();
  const nlohmann::json& p = e.at("pose");
  r.pose.across_mm = p.at("across_mm").get<double>();
  r.pose.along_mm = p.at("along_mm").get<double>();
  r.pose.round_deg = p.at("round_deg").get<double>();
  r.pose.yaw_deg = p.at("yaw_deg").get<double>();
  r.pose.height_mm = p.at("height_mm").get<double>();
  return r;
}

// The fields of one take, from the take's object or, for a record of one
// take, from the record's own.
PhysicsTake TakeFromJson(const nlohmann::json& j) {
  PhysicsTake t;
  t.module = j.at("module").get<int>();
  t.take = j.at("take").get<int>();
  for (const nlohmann::json& e : j.at("steps")) t.steps.push_back(StepFromJson(e));
  const nlohmann::json& d = j.at("set_down");
  t.set_down.offset_mm = d.at("offset_mm").get<double>();
  t.set_down.across_mm = d.at("across_mm").get<double>();
  t.set_down.along_mm = d.at("along_mm").get<double>();
  t.set_down.yaw_deg = d.at("yaw_deg").get<double>();
  t.set_down.height_mm = d.at("height_mm").get<double>();
  return t;
}

}  // namespace

std::string PhysicsRecordToJson(const PhysicsRecord& record) {
  // One take: the format of the records before run 071, field for field.
  const bool one = record.takes.size() == 1;
  OJson j = OJson::object();
  j["layout"] = record.layout;
  j["layout_sha256"] = record.layout_sha256;
  j["plan_sha256"] = record.plan_sha256;
  j["command"] = record.command;
  if (one) {
    j["module"] = record.takes.front().module;
    j["take"] = record.takes.front().take;
  }
  j["clearance_m"] = Seconds(record.clearance_m);
  j["gate_offset_mm"] = Fine(record.gate_offset_mm);
  j["gate_yaw_deg"] = Fine(record.gate_yaw_deg);
  if (one) {
    j["steps"] = StepsToJson(record.takes.front().steps);
    j["set_down"] = SetDownToJson(record.takes.front().set_down);
  } else {
    OJson takes = OJson::array();
    for (const PhysicsTake& t : record.takes) {
      OJson e = OJson::object();
      e["module"] = t.module;
      e["take"] = t.take;
      e["steps"] = StepsToJson(t.steps);
      e["set_down"] = SetDownToJson(t.set_down);
      takes.push_back(e);
    }
    j["takes"] = takes;
  }
  if (!record.rails.empty()) {
    OJson g = OJson::object();
    g["across_mm"] = Fine(record.rail_gate.across_mm);
    g["along_mm"] = Fine(record.rail_gate.along_mm);
    g["round_deg"] = Fine(record.rail_gate.round_deg);
    g["yaw_deg"] = Fine(record.rail_gate.yaw_deg);
    j["rail_gate"] = g;
    j["rails"] = RailsToJson(record.rails);
  }
  j["counted_s"] = Seconds(record.counted_s);
  return j.dump(2) + "\n";
}

std::optional<PhysicsRecord> PhysicsRecordFromJson(const std::string& text,
                                                   std::string* error) {
  try {
    const nlohmann::json j = nlohmann::json::parse(text);
    PhysicsRecord r;
    r.layout = j.at("layout").get<std::string>();
    r.layout_sha256 = j.at("layout_sha256").get<std::string>();
    r.plan_sha256 = j.at("plan_sha256").get<std::string>();
    r.command = j.at("command").get<std::string>();
    r.clearance_m = j.at("clearance_m").get<double>();
    r.gate_offset_mm = j.at("gate_offset_mm").get<double>();
    r.gate_yaw_deg = j.at("gate_yaw_deg").get<double>();
    if (j.contains("takes")) {
      for (const nlohmann::json& e : j.at("takes")) r.takes.push_back(TakeFromJson(e));
    } else {
      r.takes.push_back(TakeFromJson(j));
    }
    if (j.contains("rails")) {
      const nlohmann::json& g = j.at("rail_gate");
      r.rail_gate.across_mm = g.at("across_mm").get<double>();
      r.rail_gate.along_mm = g.at("along_mm").get<double>();
      r.rail_gate.round_deg = g.at("round_deg").get<double>();
      r.rail_gate.yaw_deg = g.at("yaw_deg").get<double>();
      for (const nlohmann::json& e : j.at("rails")) r.rails.push_back(RailFromJson(e));
    }
    r.counted_s = j.at("counted_s").get<double>();
    return r;
  } catch (const nlohmann::json::exception& e) {
    *error = std::string("not a physics record: ") + e.what();
    return std::nullopt;
  } catch (const std::runtime_error& e) {
    *error = std::string("not a physics record: ") + e.what();
    return std::nullopt;
  }
}

std::vector<std::string> CheckPhysicsRecord(const PhysicsRecord& record,
                                            const Plan& plan,
                                            const std::string& plan_sha256) {
  std::vector<std::string> faults;
  if (record.layout != plan.layout) {
    faults.push_back("the record is of layout " + record.layout +
                     ", the plan is of " + plan.layout);
  }
  if (record.plan_sha256 != plan_sha256) {
    faults.push_back("plan_sha256 is " + record.plan_sha256 +
                     ", the plan file's is " + plan_sha256);
  }
  auto differs = [](double a, double b) {
    return std::abs(a - b) > kSecondsTolerance;
  };
  // One bay, counted once.
  if (record.takes.empty() && record.rails.empty()) faults.push_back("the record holds no take");
  std::set<int> modules;
  std::set<int> bays;
  for (const PhysicsTake& take : record.takes) {
    if (take.steps.empty()) {
      faults.push_back("the take of module " + std::to_string(take.module) +
                       " holds no step");
    }
    if (!modules.insert(take.module).second) {
      faults.push_back("module " + std::to_string(take.module) +
                       " is held by two takes: one bay counts each module once");
    }
    for (const PhysicsStep& s : take.steps) {
      for (const StepTime& t : plan.steps) {
        if (t.number == s.number) bays.insert(t.bay);
      }
    }
  }
  std::set<int> rails;
  for (const PhysicsRailSet& set : record.rails) {
    if (!rails.insert(set.rail).second) {
      faults.push_back("rail " + std::to_string(set.rail) +
                       " is set by two rail sets: one bay counts each rail once");
    }
    if (set.step.what != "set rail " + std::to_string(set.rail)) {
      faults.push_back("the rail set of rail " + std::to_string(set.rail) + " is of step \"" +
                       set.step.what + "\"");
    }
    for (const StepTime& t : plan.steps) {
      if (t.number == set.step.number) bays.insert(t.bay);
    }
  }
  if (bays.size() > 1) {
    faults.push_back("the record holds steps of " + std::to_string(bays.size()) +
                     " bays: it counts one bay");
  }
  // A step against the plan's, and its stored flag against the rule.
  auto check_step = [&](const PhysicsStep& s, bool counts) {
    const std::string name = "step " + std::to_string(s.number);
    const StepTime* found = nullptr;
    for (const StepTime& t : plan.steps) {
      if (t.number == s.number) {
        found = &t;
        break;
      }
    }
    if (found == nullptr) {
      faults.push_back(name + " is not a step of the plan");
    } else {
      if (s.what != found->what) {
        faults.push_back(name + " is \"" + s.what + "\", the plan's is \"" +
                         found->what + "\"");
      }
      if (s.arm != found->arm) {
        faults.push_back(name + " is of arm " + s.arm + ", the plan's is " +
                         found->arm);
      }
      if (differs(s.start_s, found->start)) {
        faults.push_back(name + " starts at " + std::to_string(s.start_s) +
                         " s, the plan's at " + std::to_string(found->start) +
                         " s");
      }
      if (differs(s.end_s, found->end)) {
        faults.push_back(name + " ends at " + std::to_string(s.end_s) +
                         " s, the plan's at " + std::to_string(found->end) +
                         " s");
      }
      if (differs(s.planned_s, found->end - found->start)) {
        faults.push_back(name + " has planned_s " +
                         std::to_string(s.planned_s) + " s, the plan's is " +
                         std::to_string(found->end - found->start) + " s");
      }
    }
    if (s.counted != counts) {
      faults.push_back(name + " is stored as " +
                       (s.counted ? "counted" : "not counted") +
                       ", the rule gives the other");
    }
  };
  for (const PhysicsTake& take : record.takes) {
    for (const PhysicsStep& s : take.steps) check_step(s, StepCounts(record, take, s));
  }
  for (const PhysicsRailSet& set : record.rails) check_step(set.step, RailSetCounts(record, set));
  if (differs(record.counted_s, CountedSeconds(record))) {
    faults.push_back("counted_s is " + std::to_string(record.counted_s) +
                     " s, the steps that count add up to " +
                     std::to_string(CountedSeconds(record)) + " s");
  }
  return faults;
}

}  // namespace cell::layout
