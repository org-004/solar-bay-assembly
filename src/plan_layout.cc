// Plans one bay of a layout (src/layout_plan.h) and prints one row per move.
//
//   plan_layout <name> [--summary] [--out PATH] [--threads N]
//               [--first-step A --last-step B] [--pretty]
//
// The default output is CSV: a header and one row per move
//   step,arm,from,to,found,budget,waypoints,queries,planned_s,model_s,ratio,note
// (budget: the iteration budget that found it, 0 when the straight line was
// free; planned_s and model_s both with the settle; ratio planned_s /
// model_s, empty when model_s is 0), then `#` lines: the figures, each
// phase's planned and model seconds, each arm's working, moving and waiting
// seconds, every wait with its reason, how long planning took and its
// collision queries, and the faults. --pretty lines the CSV's columns up.
// --summary prints `key: value` lines instead. --out PATH writes the plan
// as JSON (PlanToJson). Layouts are read from LAYOUT_DIR.
//
// Exit status: 0 when the plan is complete; 1 when a move was not found,
// the schedule is blocked or the verifier found a fault; 2 on a usage or
// loading error.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"

namespace {

using namespace cell::layout;

std::string LayoutDir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr && dir[0] != '\0' ? dir : "layouts";
}

std::string Csv(const std::string& s) {
  if (s.find_first_of(",\"\n") == std::string::npos) return s;
  std::string out = "\"";
  for (char c : s) {
    if (c == '"') out += '"';
    out += c;
  }
  return out + "\"";
}

std::string Num(double v, int digits = 2) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", digits, v);
  std::string s = buffer;
  if (s.size() > 1 && s[0] == '-' && s.find_first_not_of("-0.") == std::string::npos) {
    s = s.substr(1);
  }
  return s;
}

int Usage() {
  std::fprintf(stderr,
               "usage: plan_layout <name> [--summary] [--out PATH] "
               "[--threads N] [--first-step A --last-step B] [--pretty]\n");
  return 2;
}

void PrintTable(const std::vector<std::vector<std::string>>& rows,
                bool pretty) {
  std::vector<size_t> width;
  if (pretty) {
    for (const auto& r : rows) {
      if (width.size() < r.size()) width.resize(r.size(), 0);
      for (size_t i = 0; i < r.size(); ++i) {
        width[i] = std::max(width[i], r[i].size());
      }
    }
  }
  for (const auto& r : rows) {
    std::string line;
    for (size_t i = 0; i < r.size(); ++i) {
      if (i > 0) line += pretty ? "  " : ",";
      std::string cell = pretty ? r[i] : Csv(r[i]);
      if (pretty && i + 1 < r.size()) cell.resize(width[i], ' ');
      line += cell;
    }
    std::printf("%s\n", line.c_str());
  }
}

// Each arm's seconds of work (approach, work, retreat), moving and waiting.
struct ArmSeconds {
  double working = 0.0, moving = 0.0, waiting = 0.0;
};
ArmSeconds SecondsOf(const ArmPlan& a) {
  ArmSeconds s;
  for (const Segment& g : a.segments) {
    const double d = g.t1 - g.t0;
    if (g.kind == "wait") {
      s.waiting += d;
    } else if (g.kind == "move") {
      s.moving += d;
    } else {
      s.working += d;
    }
  }
  return s;
}

}  // namespace

int main(int argc, char** argv) {
  std::string name, out;
  bool summary = false, pretty = false;
  PlanOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&](std::string* v) {
      if (i + 1 >= argc) return false;
      *v = argv[++i];
      return true;
    };
    std::string v;
    if (a == "--summary") {
      summary = true;
    } else if (a == "--pretty") {
      pretty = true;
    } else if (a == "--out") {
      if (!value(&out)) return Usage();
    } else if (a == "--threads") {
      if (!value(&v)) return Usage();
      options.threads = std::atoi(v.c_str());
      if (options.threads < 1) return Usage();
    } else if (a == "--first-step") {
      if (!value(&v)) return Usage();
      options.first_step = std::atoi(v.c_str());
    } else if (a == "--last-step") {
      if (!value(&v)) return Usage();
      options.last_step = std::atoi(v.c_str());
    } else if (a == "--margins") {
      // The next milestone's: the tightest margins of the plan.
      std::fprintf(stderr, "plan_layout: --margins is not written yet\n");
      return 2;
    } else if (!a.empty() && a[0] == '-') {
      return Usage();
    } else if (name.empty()) {
      name = a;
    } else {
      return Usage();
    }
  }
  if (name.empty()) return Usage();
  if ((options.first_step == 0) != (options.last_step == 0) ||
      options.first_step > options.last_step) {
    return Usage();
  }
  const std::string path = LayoutDir() + "/" + name + ".json";
  std::string error;
  const std::optional<Layout> layout = LoadLayout(path, &error);
  if (!layout) {
    std::fprintf(stderr, "plan_layout: %s\n", error.c_str());
    return 2;
  }
  const std::string sha = Sha256OfFile(path);
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const PlanResult r = PlanLayout(*layout, paths, sha, options);
  const Plan& plan = r.plan;

  if (!out.empty()) {
    std::ofstream file(out, std::ios::binary);
    file << PlanToJson(plan);
    if (!file) {
      std::fprintf(stderr, "plan_layout: cannot write %s\n", out.c_str());
      return 2;
    }
  }

  int found = 0;
  for (const MoveRow& row : r.rows) found += row.found ? 1 : 0;
  int waits = 0;
  double wait_s = 0.0;
  for (const ArmPlan& a : plan.arms) {
    for (const Segment& g : a.segments) {
      if (g.kind == "wait" && !g.reason.empty()) {
        ++waits;
        wait_s += g.t1 - g.t0;
      }
    }
  }
  if (summary) {
    int module_arms = 0, pin_arms = 0;
    const std::vector<std::string> leaders = ArmsDoing(
        *layout, {"rail_take", "rail_set", "module_take", "module_set"});
    const std::vector<std::string> holers =
        ArmsDoing(*layout, {"locate", "pin", "swage"});
    module_arms = static_cast<int>(leaders.size());
    pin_arms = static_cast<int>(holers.size());
    std::printf("layout: %s\n", layout->name.c_str());
    std::printf("base: %s\n", layout->base.c_str());
    std::printf("module_arms: %d\n", module_arms);
    std::printf("pin_arms: %d\n", pin_arms);
    std::printf("arms: %d\n", static_cast<int>(layout->arms.size()));
    std::printf("clearance_m: %s\n", Num(plan.figures.clearance_m, 4).c_str());
    std::printf("joint_margin_deg: %s\n",
                Num(plan.figures.joint_margin_deg, 1).c_str());
    std::printf("moves: %d\n", static_cast<int>(r.rows.size()));
    std::printf("moves_found: %d\n", found);
    std::printf("bay_planned_s: %s\n", Num(plan.bay_planned_s).c_str());
    std::printf("bay_model_s: %s\n", Num(plan.bay_model_s).c_str());
    for (const PhaseTime& p : plan.phases) {
      std::string key = p.name;
      for (char& c : key) {
        if (c == ' ') c = '_';
      }
      std::printf("phase_%s_planned_s: %s\n", key.c_str(),
                  Num(p.planned_s).c_str());
      std::printf("phase_%s_model_s: %s\n", key.c_str(), Num(p.model_s).c_str());
    }
    const bool verified = !r.verify.least.empty() || r.verify.ok;
    auto least = [&](double v, int digits) {
      return verified ? Num(v, digits) : std::string("none");
    };
    std::printf("least_clearance_m: %s\n",
                least(r.verify.least_clearance_m, 4).c_str());
    std::printf("least_arm_to_arm_m: %s\n",
                least(r.verify.least_arm_to_arm_m, 4).c_str());
    std::printf("least_joint_margin_deg: %s\n",
                least(r.verify.least_joint_margin_deg, 2).c_str());
    std::printf("waits: %d\n", verified ? r.verify.waits : waits);
    std::printf("wait_s: %s\n",
                Num(verified ? r.verify.wait_s : wait_s).c_str());
    std::printf("planning_s: %s\n", Num(r.seconds, 1).c_str());
    std::printf("planning_queries: %lld\n", static_cast<long long>(r.queries));
    std::printf("complete: %s\n", plan.complete ? "true" : "false");
    for (const std::string& f : r.faults) {
      std::printf("fault: %s\n", f.c_str());
    }
  } else {
    std::vector<std::vector<std::string>> table;
    table.push_back({"step", "arm", "from", "to", "found", "budget",
                     "waypoints", "queries", "planned_s", "model_s", "ratio",
                     "note"});
    for (const MoveRow& row : r.rows) {
      table.push_back(
          {std::to_string(row.step), row.arm, row.from, row.to,
           row.found ? "yes" : "no", std::to_string(row.budget),
           std::to_string(row.waypoints), std::to_string(row.queries),
           row.found ? Num(row.planned_s) : "",
           Num(row.model_s),
           row.found && row.model_s > 0.0 ? Num(row.planned_s / row.model_s)
                                          : "",
           row.note});
    }
    PrintTable(table, pretty);
    const PlanFigures& f = plan.figures;
    std::string budgets;
    for (int b : f.budgets) budgets += (budgets.empty() ? "" : "/") + std::to_string(b);
    std::printf(
        "# figures: clearance_m %s, joint_margin_deg %s, to_full_speed_s %s, "
        "jerk_rise_s %s, settle_s %s, leg_speed %s m/s, leg_accel %s m/s^2, "
        "max_deviation %s rad, edge_resolution %s m, shorten_tries %d, "
        "budgets %s, seed %u, sample_s %s; planned_s and model_s include the "
        "settle\n",
        Num(f.clearance_m, 4).c_str(), Num(f.joint_margin_deg, 1).c_str(),
        Num(f.to_full_speed_s).c_str(), Num(f.jerk_rise_s).c_str(),
        Num(f.settle_s).c_str(), Num(f.leg_speed).c_str(),
        Num(f.leg_accel).c_str(), Num(f.max_deviation, 4).c_str(),
        Num(f.edge_resolution, 4).c_str(), f.shorten_tries, budgets.c_str(),
        f.seed, Num(f.sample_s).c_str());
    for (const PhaseTime& p : plan.phases) {
      std::printf("# phase %s: planned %s s, model %s s\n", p.name.c_str(),
                  Num(p.planned_s).c_str(), Num(p.model_s).c_str());
    }
    std::printf("# bay: planned %s s, model %s s\n",
                Num(plan.bay_planned_s).c_str(), Num(plan.bay_model_s).c_str());
    for (const ArmPlan& a : plan.arms) {
      const ArmSeconds s = SecondsOf(a);
      std::printf("# arm %s: working %s s, moving %s s, waiting %s s\n",
                  a.name.c_str(), Num(s.working).c_str(), Num(s.moving).c_str(),
                  Num(s.waiting).c_str());
    }
    for (const ArmPlan& a : plan.arms) {
      for (const Segment& g : a.segments) {
        if (g.kind != "wait") continue;
        std::printf("# wait %s step %d: %s s from %s s, %s\n", a.name.c_str(),
                    g.step, Num(g.t1 - g.t0).c_str(), Num(g.t0).c_str(),
                    g.reason.c_str());
      }
    }
    std::printf("# planning: %s s, %lld collision queries, %d of %d moves "
                "found\n",
                Num(r.seconds, 1).c_str(), static_cast<long long>(r.queries),
                found, static_cast<int>(r.rows.size()));
    std::vector<const MoveRow*> costly;
    for (const MoveRow& row : r.rows) costly.push_back(&row);
    std::stable_sort(costly.begin(), costly.end(),
                     [](const MoveRow* x, const MoveRow* y) {
                       return x->queries > y->queries;
                     });
    for (size_t i = 0; i < costly.size() && i < 5; ++i) {
      std::printf("# costly move %d: step %d %s from %s to %s, %lld queries\n",
                  static_cast<int>(i + 1), costly[i]->step,
                  costly[i]->arm.c_str(), costly[i]->from.c_str(),
                  costly[i]->to.c_str(),
                  static_cast<long long>(costly[i]->queries));
    }
    for (const std::string& fault : r.faults) {
      std::printf("# fault: %s\n", fault.c_str());
    }
    std::printf("# complete: %s\n", plan.complete ? "true" : "false");
  }
  return plan.complete ? 0 : 1;
}
