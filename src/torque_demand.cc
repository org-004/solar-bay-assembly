// torque_demand: what a layout's committed plan asks of the module arm's
// motors (src/layout_torque.h), move by move.
//
//   torque_demand [<name>] [--bay B] [--all]
//   torque_demand [<name>] --all-bays
//
// <name> is a layout with a committed plan, plans/<name>.json; without it,
// the chosen layout (versions/versions.json). For every move of the module
// arm in bay B (0 without --bay) it prints the step, the move's start, its
// duration, its time scale, and the peak of |inverse dynamics| over the
// motor's limit for the six joints and the track; --all prints the legs and
// work segments too. Then how many moves exceed 1.0 and the layout's
// torque_limit_scale, if it has one, and the peak of every module step over
// all its segments (what the physics record measures, step by step). Exit 0
// when the table was made, 1 when it could not be, 2 on a wrong argument.
//
// --all-bays prints no table: for each of the plan's bays and then for all
// of them, the module arm's moves, how many exceed 1.0 and how many the
// layout's torque_limit_scale (1.0 without it), and the worst fraction with
// its axis and step (src/layout_margins.h, TorqueByBay).
//
//   bazel run --config=opt //:torque_demand
//   bazel run --config=opt //:torque_demand -- flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_margins.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"
#include "src/layout_torque.h"

namespace {

std::string Env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

int Usage() {
  std::fprintf(stderr, "usage: torque_demand [<name>] [--bay B] [--all] | [<name>] --all-bays\n");
  return 2;
}

const char* const kAxes[] = {"joint 1", "joint 2", "joint 3", "joint 4",
                             "joint 5", "joint 6", "track"};

void Ratios(const std::array<double, 7>& r) {
  for (double x : r) std::printf(" %6.3f", x);
}

}  // namespace

int main(int argc, char** argv) {
  using namespace cell::layout;
  const std::vector<std::string> args(argv + 1, argv + argc);
  std::string name;
  int bay = 0;
  bool all = false, all_bays = false;
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    if (a == "--all") {
      all = true;
    } else if (a == "--all-bays") {
      all_bays = true;
    } else if (a == "--bay" && i + 1 < args.size()) {
      const std::string& k = args[++i];
      char* end = nullptr;
      const long b = std::strtol(k.c_str(), &end, 10);
      if (k.empty() || *end != '\0' || b < 0) return Usage();
      bay = static_cast<int>(b);
    } else if ((!a.empty() && a[0] == '-') || !name.empty()) {
      return Usage();
    } else {
      name = a;
    }
  }
  std::string error;
  if (name.empty()) {
    const std::optional<std::string> chosen = ChosenLayout(VersionsPath(), &error);
    if (!chosen) {
      std::fprintf(stderr, "torque_demand: %s\n", error.c_str());
      return 1;
    }
    name = *chosen;
  }
  const std::string layout_path = Env("LAYOUT_DIR", "layouts") + "/" + name + ".json";
  const std::string plan_path = Env("PLAN_DIR", "plans") + "/" + name + ".json";
  const std::optional<Layout> layout = LoadLayout(layout_path, &error);
  if (!layout) {
    std::fprintf(stderr, "torque_demand: %s\n", error.c_str());
    return 1;
  }
  std::ifstream in(plan_path);
  if (!in) {
    std::fprintf(stderr, "torque_demand: no committed plan %s\n", plan_path.c_str());
    return 1;
  }
  std::stringstream text;
  text << in.rdbuf();
  const std::optional<Plan> plan = PlanFromJson(text.str(), &error);
  if (!plan) {
    std::fprintf(stderr, "torque_demand: %s: %s\n", plan_path.c_str(), error.c_str());
    return 1;
  }
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const std::string arm = ModuleArm(*layout);
  std::unique_ptr<ArmDynamics> dynamics = ArmDynamics::Create(*layout, paths, arm, &error);
  if (!dynamics) {
    std::fprintf(stderr, "torque_demand: %s\n", error.c_str());
    return 1;
  }
  // The arm's motion as the plan timed it.
  const CheckReport report = CheckLayout(*layout, paths);
  std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(*layout, paths, report.joint_margin_deg, &error);
  if (!scene) {
    std::fprintf(stderr, "torque_demand: %s\n", error.c_str());
    return 1;
  }
  const int scene_arm = scene->ArmIndex(arm);
  const ArmMotion motion = MotionOf(*scene, scene_arm, plan->figures);
  const ArmPlan* ap = nullptr;
  for (const ArmPlan& a : plan->arms) {
    if (a.name == arm) ap = &a;
  }
  if (ap == nullptr) {
    std::fprintf(stderr, "torque_demand: the plan has no arm %s\n", arm.c_str());
    return 1;
  }
  if (all_bays) {
    const double scale = plan->figures.torque_limit_scale > 0.0
                             ? plan->figures.torque_limit_scale
                             : 1.0;
    std::vector<BayTorque> bays;
    if (!TorqueByBay(*plan, dynamics.get(), motion, arm, scale, &bays, &error)) {
      std::fprintf(stderr, "torque_demand: %s\n", error.c_str());
      return 1;
    }
    std::printf("torque_demand: %s, all %d bays, arm %s\n", name.c_str(),
                static_cast<int>(bays.size()), arm.c_str());
    std::printf("torque_limit_scale: %s\n", plan->figures.torque_limit_scale > 0.0
                                                 ? std::to_string(scale).c_str()
                                                 : "none (counted against 1.0)");
    BayTorque total;
    auto line = [&](const std::string& what, const BayTorque& t) {
      std::printf("%s: moves %d; over 1.0: %d; over %.3f: %d; worst %.3f (%s) in step %d\n",
                  what.c_str(), t.moves, t.over_one, scale, t.over_scale, t.worst.Worst(),
                  kAxes[t.worst.WorstAxis()], t.worst_step);
    };
    for (const BayTorque& t : bays) {
      line("bay " + std::to_string(t.bay), t);
      total.moves += t.moves;
      total.over_one += t.over_one;
      total.over_scale += t.over_scale;
      if (t.worst.Worst() > total.worst.Worst()) {
        total.worst = t.worst;
        total.worst_step = t.worst_step;
      }
    }
    line("total", total);
    return 0;
  }
  std::map<int, const StepTime*> steps;
  for (const StepTime& s : plan->steps) steps[s.number] = &s;
  const double fraction = plan->figures.torque_limit_scale;
  const std::array<double, 7>& lim = dynamics->limits();
  std::printf("torque_demand: %s, bay %d, arm %s\n", name.c_str(), bay, arm.c_str());
  std::printf("limits: joints 1-6 %.0f %.0f %.0f %.0f %.0f %.0f N m, track %.0f N\n", lim[0],
              lim[1], lim[2], lim[3], lim[4], lim[5], lim[6]);
  std::printf("torque_limit_scale: %s\n",
              fraction > 0.0 ? std::to_string(fraction).c_str() : "none");
  std::printf("%5s %-8s %-6s %10s %8s %7s %6s %6s %6s %6s %6s %6s %6s  %s\n", "step", "kind",
              "held", "t0_s", "dur_s", "scale", "j1", "j2", "j3", "j4", "j5", "j6", "track",
              "from -> to");
  int moves = 0, over_one = 0, over_fraction = 0;
  TorqueDemand worst;
  int worst_step = 0;
  std::map<int, TorqueDemand> by_step;
  std::map<int, std::string> step_what;
  for (const Segment& s : ap->segments) {
    auto it = steps.find(s.step);
    if (it == steps.end() || it->second->bay != bay) continue;
    if (s.kind == "wait" || s.path.empty()) continue;
    TorqueDemand d;
    if (!dynamics->AlongSegment(s, motion, plan->figures, &d, &error)) {
      std::fprintf(stderr, "torque_demand: step %d %s: %s\n", s.step, s.kind.c_str(),
                   error.c_str());
      return 1;
    }
    TorqueDemand& st = by_step[s.step];
    for (int j = 0; j < 7; ++j) st.ratio[j] = std::max(st.ratio[j], d.ratio[j]);
    step_what[s.step] = it->second->what;
    const bool move = s.kind == "move";
    if (move) {
      ++moves;
      if (d.Worst() > 1.0) ++over_one;
      if (fraction > 0.0 && d.Worst() > fraction) ++over_fraction;
      if (d.Worst() > worst.Worst()) {
        worst = d;
        worst_step = s.step;
      }
    }
    if (!move && !all) continue;
    std::printf("%5d %-8s %-6s %10.3f %8.3f %7.4f", s.step, s.kind.c_str(),
                s.held.empty() ? "-" : s.held.c_str(), s.t0, s.t1 - s.t0, s.time_scale);
    Ratios(d.ratio);
    std::printf("  %s\n", move ? (s.from + " -> " + s.to).c_str() : "");
  }
  std::printf("moves: %d; over 1.0: %d", moves, over_one);
  if (fraction > 0.0) std::printf("; over %.3f: %d", fraction, over_fraction);
  std::printf("; worst %.3f (%s) in step %d\n", worst.Worst(), kAxes[worst.WorstAxis()],
              worst_step);
  std::printf("steps, every segment (as the physics record's steps):\n");
  for (const auto& [n, d] : by_step) {
    std::printf("%5d", n);
    Ratios(d.ratio);
    std::printf("  %s\n", step_what[n].c_str());
  }
  return 0;
}
