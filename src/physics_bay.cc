// The bay's runs in physics, several takes and rail sets of one bay, and
// the text of a rail set (src/physics_run.h). BayTakes, RunPhysicsTakes and
// PhysicsBayText moved here from src/physics_run.cc (run 074), which keeps
// the run itself.

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "src/layout_check.h"
#include "src/physics_record.h"
#include "src/physics_run.h"

namespace cell::layout {
namespace {

std::string Fmt(const char* format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// The rail gate's figures of a rail set, and whether it counts.
std::string RailTable(const PhysicsRecord& rec) {
  const RailGate& g = rec.rail_gate;
  std::string t = Fmt(
      "the rail sets: %zu (clearance against %.3f m, motors below their limits, the rail "
      "within its play: across %.4f mm, along %.4f mm, round %.4f deg, yaw %.4f deg)\n",
      rec.rails.size(), rec.clearance_m, g.across_mm, g.along_mm, g.round_deg, g.yaw_deg);
  t += Fmt("%4s %5s %10s %10s %10s %10s %9s %9s %7s %7s %7s  %-6s %s\n", "rail", "step",
           "planned_s", "clear_m", "across_mm", "along_mm", "round_deg", "yaw_deg", "joint",
           "j_no", "track", "counts", "least pair");
  for (const PhysicsRailSet& s : rec.rails) {
    int worst = 0;
    for (int j = 1; j < 6; ++j) {
      if (s.torque_ratio[j] > s.torque_ratio[worst]) worst = j;
    }
    t += Fmt("%4d %5d %10.6f %10.6f %10.4f %10.4f %9.4f %9.4f %7.4f %7d %7.4f  %-6s %s\n",
             s.rail, s.step.number, s.step.planned_s, s.step.least_clearance_m,
             s.pose.across_mm, s.pose.along_mm, s.pose.round_deg, s.pose.yaw_deg,
             s.torque_ratio[worst], worst + 1, s.torque_ratio[6],
             RailSetCounts(rec, s) ? "yes" : "no", s.step.least_clearance_pair.c_str());
  }
  return t;
}

}  // namespace

std::string PhysicsRailText(const PhysicsResult& r) {
  if (!r.ok) return "run_physics: " + r.error + "\n";
  const PhysicsRecord& rec = r.record;
  const PhysicsRailSet& set = rec.rails.front();
  const PhysicsStep& s = set.step;
  std::string t = Fmt("layout %s, arm %s: rail %d, step %d (%s), %.6f to %.6f s, planned %.6f s\n",
                      r.layout.c_str(), r.arm.c_str(), r.rail, s.number, s.what.c_str(),
                      s.start_s, s.end_s, s.planned_s);
  t += Fmt("plan %s, layout %s\n", rec.plan_sha256.substr(0, 12).c_str(),
           rec.layout_sha256.substr(0, 12).c_str());
  t += "\nsegments (plan time; errors against the plan; clearance in physics and in the plan)\n";
  t += Fmt("%-9s %10s %10s %9s %9s %9s %9s %10s %10s  %s\n", "kind", "t0_s", "t1_s",
           "joint_mrad", "track_mm", "frame_mm", "frame_deg", "phys_m", "plan_m",
           "least pair in physics");
  auto row = [&](const PhysicsFigures& f, const char* kind) {
    return Fmt("%-9s %10.3f %10.3f %9.4f %9.4f %9.4f %9.4f %10.6f %10.6f  %s%s\n", kind, f.t0,
               f.t1, f.joint_error_mrad, f.track_error_mm, f.frame_error_mm, f.frame_error_deg,
               f.clearance_m, f.plan_clearance_m, f.clearance_pair.c_str(),
               f.frame_is_module ? " [rail frame]" : "");
  };
  for (const PhysicsFigures& f : r.segments) t += row(f, f.kind.c_str());
  t += row(r.settle, "settle");
  t += "\npeak torque over its limit (joints 1-6, track), the settling included:";
  for (const double v : set.torque_ratio) t += Fmt(" %.4f", v);
  t += Fmt("\nleast clearance in physics %.6f m at %.3f s (%s); the plan's %.6f m; against %.3f m\n",
           s.least_clearance_m, s.least_clearance_t, s.least_clearance_pair.c_str(),
           s.plan_least_clearance_m, rec.clearance_m);
  t += Fmt("\nthe rail after release and settling, at its tab: across %.4f mm, along %.4f mm; "
           "round the tube %.4f deg, yaw %.4f deg; height %.4f mm\n",
           set.pose.across_mm, set.pose.along_mm, set.pose.round_deg, set.pose.yaw_deg,
           set.pose.height_mm);
  t += Fmt("at rest on the tube: %d contacts with it, speed %.4f mm/s\n", r.rail_tube_contacts,
           r.rail_speed_mm_s);
  t += RailTable(rec);
  t += Fmt("counted %.6f s of one_bay_s %.6f s: physics_share %.6f\n", rec.counted_s,
           r.one_bay_s, r.one_bay_s > 0.0 ? rec.counted_s / r.one_bay_s : 0.0);
  return t;
}

std::vector<int> BayRails(const Layout& layout, const ModelPaths& paths, std::string* error) {
  const CheckReport report = CheckLayout(layout, paths);
  std::vector<int> rails;
  if (!report.error.empty()) {
    *error = report.error;
    return rails;
  }
  std::string arm;
  for (const BayStep& s : report.steps) {
    if (s.kind == "module_take" && arm.empty()) arm = s.arm;
  }
  for (const BayStep& s : report.steps) {
    if (s.kind == "rail_set" && s.arm == arm) rails.push_back(s.index);
  }
  return rails;
}

std::vector<int> BayTakes(const Layout& layout, const ModelPaths& paths, std::string* error) {
  const CheckReport report = CheckLayout(layout, paths);
  std::vector<int> takes;
  if (!report.error.empty()) {
    *error = report.error;
    return takes;
  }
  for (const BayStep& s : report.steps) {
    if (s.kind == "module_take") takes.push_back(s.index);
  }
  if (takes.empty()) *error = "the bay has no module take";
  return takes;
}

PhysicsBayResult RunPhysicsTakes(const Layout& layout, const Plan& plan,
                                 const ModelPaths& paths, const PhysicsOptions& options,
                                 const std::vector<int>& takes, const std::vector<int>& rails,
                                 const std::string& command,
                                 const std::string& layout_sha256,
                                 const std::string& plan_sha256) {
  PhysicsBayResult out;
  std::vector<PhysicsRecord> records;
  for (const int k : takes) {
    PhysicsOptions one = options;
    one.take = k;
    out.runs.push_back(RunPhysics(layout, plan, paths, one, layout_sha256, plan_sha256));
    if (!out.runs.back().ok) {
      out.error = Fmt("module_take %d: %s", k, out.runs.back().error.c_str());
      return out;
    }
    records.push_back(out.runs.back().record);
  }
  for (const int k : rails) {
    PhysicsOptions one = options;
    one.take = -1;
    one.rail = k;
    out.runs.push_back(RunPhysics(layout, plan, paths, one, layout_sha256, plan_sha256));
    if (!out.runs.back().ok) {
      out.error = Fmt("rail_set %d: %s", k, out.runs.back().error.c_str());
      return out;
    }
    records.push_back(out.runs.back().record);
  }
  std::optional<PhysicsRecord> joined = JoinPhysicsRecords(records, command, &out.error);
  if (!joined) return out;
  out.record = *joined;
  out.one_bay_s = plan.bay_planned_s;
  out.ok = true;
  return out;
}

std::string PhysicsBayText(const PhysicsBayResult& r) {
  if (!r.ok) return "run_physics: " + r.error + "\n";
  std::string t;
  for (const PhysicsResult& run : r.runs) t += PhysicsText(run) + "\n";
  const PhysicsRecord& rec = r.record;
  t += Fmt("the record: %zu takes of one bay, each module once (clearance against %.3f m, "
           "set-down against %.1f mm and %.2f deg)\n",
           rec.takes.size(), rec.clearance_m, rec.gate_offset_mm, rec.gate_yaw_deg);
  t += Fmt("%5s %6s %5s %5s %11s %10s %10s %9s %9s  %s\n", "take", "module", "step", "step",
           "planned_s", "take_m", "set_m", "offset_mm", "yaw_deg", "counts");
  for (const PhysicsTake& take : rec.takes) {
    if (take.steps.size() != 2) continue;
    const PhysicsStep& a = take.steps[0];
    const PhysicsStep& b = take.steps[1];
    t += Fmt("%5d %6d %5d %5d %11.6f %10.6f %10.6f %9.4f %9.4f  %s\n", take.take, take.module,
             a.number, b.number, a.planned_s + b.planned_s, a.least_clearance_m,
             b.least_clearance_m, take.set_down.offset_mm, take.set_down.yaw_deg,
             a.counted && b.counted ? "both" : a.counted || b.counted ? "one" : "none");
  }
  if (!rec.rails.empty()) t += RailTable(rec);
  t += Fmt("counted %.6f s of one_bay_s %.6f s: physics_share %.6f\n", rec.counted_s,
           r.one_bay_s, r.one_bay_s > 0.0 ? rec.counted_s / r.one_bay_s : 0.0);
  return t;
}

}  // namespace cell::layout
