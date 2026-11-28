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
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

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

double CountedSeconds(const PhysicsRecord& record) {
  double sum = 0.0;
  for (const PhysicsTake& take : record.takes) {
    for (const PhysicsStep& step : take.steps) {
      if (StepCounts(record, take, step)) sum += step.planned_s;
    }
  }
  return sum;
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
  for (PhysicsTake& take : joined.takes) {
    for (PhysicsStep& s : take.steps) s.counted = StepCounts(joined, take, s);
  }
  joined.counted_s = CountedSeconds(joined);
  return joined;
}

namespace {

OJson StepsToJson(const std::vector<PhysicsStep>& steps) {
  OJson out = OJson::array();
  for (const PhysicsStep& s : steps) {
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

// The fields of one take, from the take's object or, for a record of one
// take, from the record's own.
PhysicsTake TakeFromJson(const nlohmann::json& j) {
  PhysicsTake t;
  t.module = j.at("module").get<int>();
  t.take = j.at("take").get<int>();
  for (const nlohmann::json& e : j.at("steps")) {
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
    t.steps.push_back(s);
  }
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
    r.counted_s = j.at("counted_s").get<double>();
    return r;
  } catch (const nlohmann::json::exception& e) {
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
  if (record.takes.empty()) faults.push_back("the record holds no take");
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
  if (bays.size() > 1) {
    faults.push_back("the record holds steps of " + std::to_string(bays.size()) +
                     " bays: it counts one bay");
  }
  for (const PhysicsTake& take : record.takes) {
    for (const PhysicsStep& s : take.steps) {
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
      if (s.counted != StepCounts(record, take, s)) {
        faults.push_back(name + " is stored as " +
                         (s.counted ? "counted" : "not counted") +
                         ", the rule gives the other");
      }
    }
  }
  if (differs(record.counted_s, CountedSeconds(record))) {
    faults.push_back("counted_s is " + std::to_string(record.counted_s) +
                     " s, the steps that count add up to " +
                     std::to_string(CountedSeconds(record)) + " s");
  }
  return faults;
}

}  // namespace cell::layout
