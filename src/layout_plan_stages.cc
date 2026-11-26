// See src/layout_plan_stages.h: staged work in the plan.

#include "src/layout_plan_stages.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <numbers>
#include <set>
#include <utility>

#include "src/bay_hardware.h"
#include "src/layout_bays.h"
#include "src/layout_stages.h"

namespace cell::layout {

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

void AddAll(const std::vector<int>& from, std::vector<int>* into) {
  std::set<int> all(into->begin(), into->end());
  all.insert(from.begin(), from.end());
  into->assign(all.begin(), all.end());
}

// The bay's transfers in the order of the plan.
std::vector<const BayEvent*> TransfersOf(const std::vector<BayEvent>& events, int bay) {
  std::vector<const BayEvent*> out;
  for (const BayEvent& e : events) {
    if (e.kind == "transfer" && e.bay == bay) out.push_back(&e);
  }
  return out;
}

const BayEvent* EventOf(const std::vector<BayEvent>& events, int bay, const std::string& kind) {
  for (const BayEvent& e : events) {
    if (e.kind == kind && e.bay == bay) return &e;
  }
  return nullptr;
}

std::string StageName(const std::vector<Stage>& stages, int s) {
  return s >= 0 && s < static_cast<int>(stages.size()) ? stages[s].name : std::to_string(s + 1);
}

}  // namespace

int StageCount(const Layout& layout) {
  return layout.staging ? static_cast<int>(StagesOf(layout).size()) : 1;
}

int StageIndexOf(const Layout& layout, const BayStep& step) {
  if (!layout.staging || step.stage.empty()) return 0;
  const std::vector<Stage> stages = StagesOf(layout);
  for (size_t s = 0; s < stages.size(); ++s) {
    if (stages[s].name == step.stage) return static_cast<int>(s);
  }
  return 0;
}

Eigen::Vector3d StageOffset(const Layout& layout, int s) {
  if (!layout.staging) return Eigen::Vector3d::Zero();
  const std::vector<Stage> stages = StagesOf(layout);
  if (stages.empty()) return Eigen::Vector3d::Zero();
  s = std::clamp(s, 0, static_cast<int>(stages.size()) - 1);
  return {0.0, stages[s].tube_offset_m - stages[0].tube_offset_m, 0.0};
}

Layout LayoutOfStage(const Layout& layout, int s) {
  return layout.staging ? StageLayout(layout, s) : layout;
}

std::vector<std::unique_ptr<LayoutScene>> StageScenes(const Layout& layout,
                                                      const ModelPaths& paths,
                                                      double joint_margin_deg,
                                                      std::string* error) {
  std::vector<std::unique_ptr<LayoutScene>> out;
  for (int s = 0; s < StageCount(layout); ++s) {
    std::unique_ptr<LayoutScene> scene =
        LayoutScene::Load(LayoutOfStage(layout, s), paths, joint_margin_deg, error);
    if (!scene) return {};
    out.push_back(std::move(scene));
  }
  return out;
}

void LeadersParts(const StageScope* stage, const std::string& phase, Built* b) {
  if (phase != "rails" && phase != "modules" && phase != "end") return;
  std::vector<int> rails, modules;
  for (int k = 0; k < bay::kRailCount; ++k) rails.push_back(k);
  for (int k = 0; k < bay::kModuleCount; ++k) modules.push_back(k);
  if (stage != nullptr) {
    rails = stage->rails_before;
    rails.insert(rails.end(), stage->rails.begin(), stage->rails.end());
    modules = stage->modules_before;
    modules.insert(modules.end(), stage->modules.begin(), stage->modules.end());
  }
  AddAll(rails, &b->rails);
  if (phase != "rails") AddAll(modules, &b->modules);
}

StagePlace StagePlaceAt(const Layout& layout, const std::vector<BayEvent>& events, int bay,
                        double t) {
  StagePlace p;
  bool any = false;
  for (const BayEvent& e : events) {
    if (e.kind != "transfer" || e.bay != bay) continue;
    any = true;
    if (t >= e.t1) {
      p.stage = e.to_stage - 1;
      continue;
    }
    if (t >= e.t0) {
      p.stage = e.from_stage - 1;
      p.moving = true;
      const double u = e.t1 > e.t0 ? (t - e.t0) / (e.t1 - e.t0) : 1.0;
      p.offset = StageOffset(layout, e.from_stage - 1) +
                 u * (StageOffset(layout, e.to_stage - 1) - StageOffset(layout, e.from_stage - 1));
    }
    break;
  }
  if (any && !p.moving) p.offset = StageOffset(layout, p.stage);
  return p;
}

std::vector<PhaseTime> StagedPhases(const Layout& layout, const CheckReport& report, int bay,
                                    const std::map<int, StepTime>& times,
                                    const std::vector<BayEvent>& events, double bay_end) {
  const int stages = StageCount(layout);
  const int n = static_cast<int>(report.steps.size());
  const BayEvent* tube = EventOf(events, bay, "tube_in");
  const BayEvent* clamp = EventOf(events, bay, "clamp");
  const std::vector<const BayEvent*> transfers = TransfersOf(events, bay);
  std::vector<double> start(stages, clamp != nullptr ? clamp->t1 : 0.0), end(stages, bay_end);
  double transfer = 0.0;
  for (size_t k = 0; k < transfers.size(); ++k) {
    const int to = transfers[k]->to_stage - 1;
    if (to >= 1 && to < stages) {
      start[to] = transfers[k]->t1;
      end[to - 1] = transfers[k]->t0;
    }
    transfer += transfers[k]->t1 - transfers[k]->t0;
  }
  std::vector<double> rails_end = start, modules_end(stages, -1.0);
  for (const BayStep& st : report.steps) {
    if (st.arm.empty()) continue;
    auto it = times.find(StepNumberInBay(st.number, bay, n));
    if (it == times.end()) continue;
    const int k = StageIndexOf(layout, st);
    if (st.kind == "strap_bolt") rails_end[k] = std::max(rails_end[k], it->second.end);
    if (st.phase == "modules") modules_end[k] = std::max(modules_end[k], it->second.end);
  }
  double rails = 0.0, modules = 0.0, rest = 0.0;
  for (int k = 0; k < stages; ++k) {
    modules_end[k] = std::max(modules_end[k], rails_end[k]);
    rails += rails_end[k] - start[k];
    modules += modules_end[k] - rails_end[k];
    rest += end[k] - modules_end[k];
  }
  std::map<std::string, double> model;
  for (const auto& p : report.phases) model[p.first] = p.second;
  const double tube_s = tube != nullptr ? tube->t1 - tube->t0 : 0.0;
  const double clamp_s = clamp != nullptr ? clamp->t1 - clamp->t0 : 0.0;
  return {{"tube in", tube_s, model["tube in"], bay},
          {"clamp", clamp_s, model["clamp"], bay},
          {"rails", rails, model["rails"], bay},
          {"modules", modules, model["modules"], bay},
          {"end", rest, model["end"], bay},
          {"transfer", transfer, model["transfer"], bay}};
}

void WaitsThroughTransfers(const Layout& layout, const std::vector<BayEvent>& events,
                           std::vector<ArmPlan>* rows) {
  std::vector<const BayEvent*> transfers;
  for (const BayEvent& e : events) {
    if (e.kind == "transfer") transfers.push_back(&e);
  }
  if (transfers.empty()) return;
  const std::vector<Stage> stages = StagesOf(layout);
  for (ArmPlan& row : *rows) {
    std::vector<Segment> out;
    for (const Segment& s : row.segments) {
      if (s.kind != "wait") {
        out.push_back(s);
        continue;
      }
      Segment rest = s;
      for (const BayEvent* e : transfers) {
        if (e->t1 <= rest.t0 || e->t0 >= rest.t1) continue;
        if (e->t0 > rest.t0) {
          Segment before = rest;
          before.t1 = e->t0;
          out.push_back(before);
          rest.t0 = e->t0;
        }
        Segment during = rest;
        during.t1 = std::min(rest.t1, e->t1);
        during.reason = "while the tube moves to stage " + StageName(stages, e->to_stage - 1) +
                        " (transfer)";
        out.push_back(during);
        rest.t0 = during.t1;
        if (rest.t0 >= rest.t1) break;
      }
      if (rest.t1 > rest.t0) out.push_back(rest);
    }
    row.segments = std::move(out);
  }
}

std::vector<std::string> StagedFaults(const LayoutScene& scene, const CheckReport& report,
                                      const Plan& plan, const VerifyOptions& options) {
  std::vector<std::string> out;
  const Layout& L = scene.layout();
  const double tol = options.time_tolerance_s;
  if (!L.staging) {
    for (const BayEvent& e : plan.events) {
      if (e.kind == "transfer") {
        out.push_back(Fmt("bay %d: a transfer from %.3f s, and the layout has no staging", e.bay,
                          e.t0));
      }
    }
    return out;
  }
  const std::vector<Stage> stages = StagesOf(L);
  const int count = static_cast<int>(stages.size());
  const int n = static_cast<int>(report.steps.size());
  const int bays = std::max(1, plan.bays);
  std::map<int, int> stage_of;  // A step of one bay to its stage.
  std::vector<const BayStep*> transfer_steps;
  for (const BayStep& s : report.steps) {
    stage_of[s.number] = StageIndexOf(L, s);
    if (s.kind == "transfer") transfer_steps.push_back(&s);
  }
  std::map<int, const StepTime*> times;
  for (const StepTime& t : plan.steps) times[t.number] = &t;
  auto start_of = [&](int number) {
    auto it = times.find(number);
    return it == times.end() ? -1.0 : it->second->start;
  };
  auto end_of = [&](int number) {
    auto it = times.find(number);
    return it == times.end() ? -1.0 : it->second->end;
  };
  auto at_wait = [&](int arm, const Config& c) {
    const SceneArm& a = scene.arm(arm);
    for (int j = 0; j < 6; ++j) {
      if (std::abs(c[j] - a.wait[j]) * 180.0 / std::numbers::pi > options.pose_tolerance_deg) {
        return false;
      }
    }
    return std::abs(c[6] - a.wait_track) <= options.pose_tolerance_m;
  };
  for (int b = 0; b < bays; ++b) {
    const std::string of = bays > 1 ? Fmt("bay %d: ", b) : "";
    const std::vector<const BayEvent*> transfers = TransfersOf(plan.events, b);
    if (static_cast<int>(transfers.size()) != count - 1) {
      out.push_back(of + Fmt("the plan has %zu transfers, the layout's %d stages need %d",
                             transfers.size(), count, count - 1));
    }
    const BayEvent* clamp = EventOf(plan.events, b, "clamp");
    const BayEvent* release = EventOf(plan.events, b, "release");
    double after = clamp != nullptr ? clamp->t1 : 0.0;
    for (size_t k = 0; k < transfers.size(); ++k) {
      const BayEvent& e = *transfers[k];
      const int from = static_cast<int>(k), to = static_cast<int>(k) + 1;
      const std::string name = "the transfer to stage " + StageName(stages, to);
      if (k < transfer_steps.size() && e.step != StepNumberInBay(transfer_steps[k]->number, b, n)) {
        out.push_back(of + Fmt("%s names step %d, not the bay's step %d (%s)", name.c_str(), e.step,
                               StepNumberInBay(transfer_steps[k]->number, b, n),
                               transfer_steps[k]->what.c_str()));
      }
      if (e.from_stage != from + 1 || e.to_stage != to + 1) {
        out.push_back(of + Fmt("%s from %.3f s goes from stage %d to %d, not %d to %d",
                               name.c_str(), e.t0, e.from_stage, e.to_stage, from + 1, to + 1));
      }
      if (std::abs((e.t1 - e.t0) - L.staging->transfer_s) > tol) {
        out.push_back(of + Fmt("%s takes %.3f s from %.3f s, the layout's transfer_s is %.3f s",
                               name.c_str(), e.t1 - e.t0, e.t0, L.staging->transfer_s));
      }
      if (e.t0 < after - tol) {
        out.push_back(of + Fmt("%s starts at %.3f s, before %.3f s (the clamp's or the transfer "
                               "before's end)",
                               name.c_str(), e.t0, after));
      }
      after = e.t1;
      for (const ArmPlan& ap : plan.arms) {
        const int arm = scene.ArmIndex(ap.name);
        if (arm < 0) continue;
        const std::string& first = scene.arm(arm).use->first_tool;
        for (const Segment& s : ap.segments) {
          if (s.path.empty() || s.t1 <= e.t0 + tol || s.t0 >= e.t1 - tol) continue;
          if (s.kind != "wait" || !at_wait(arm, s.path.front()) || s.tool != first ||
              !s.held.empty()) {
            out.push_back(of + Fmt("arm %s is not at its wait pose with its first tool during "
                                   "%s (%.3f to %.3f s): its %s of step %d from %.3f s",
                                   ap.name.c_str(), name.c_str(), e.t0, e.t1, s.kind.c_str(),
                                   s.step, s.t0));
            break;
          }
        }
        for (const Segment& s : ap.segments) {
          if (s.kind == "wait" || BayOfStep(s.step, n) != b) continue;
          const int stage = stage_of[StepOfBay(s.step, n)];
          if (stage >= to && s.t0 < e.t1 - tol) {
            out.push_back(of + Fmt("arm %s starts its %s of step %d at %.3f s, before %s ends at "
                                   "%.3f s",
                                   ap.name.c_str(), s.kind.c_str(), s.step, s.t0, name.c_str(),
                                   e.t1));
            break;
          }
          if (stage <= from && s.t1 > e.t0 + tol) {
            out.push_back(of + Fmt("arm %s ends its %s of step %d at %.3f s, after %s starts at "
                                   "%.3f s",
                                   ap.name.c_str(), s.kind.c_str(), s.step, s.t1, name.c_str(),
                                   e.t0));
            break;
          }
        }
      }
    }
    if (release != nullptr && !transfers.empty() && release->t0 < transfers.back()->t1 - tol) {
      out.push_back(of + Fmt("release starts at %.3f s, before the last transfer ends at %.3f s",
                             release->t0, transfers.back()->t1));
    }
    // The first take of each stage after the stage's bolts; the magazine
    // before the stage's module sets.
    std::vector<double> last_bolt(count, -1.0);
    std::vector<bool> taken(count, false);
    for (const BayStep& s : report.steps) {
      if (s.kind == "strap_bolt") {
        last_bolt[stage_of[s.number]] =
            std::max(last_bolt[stage_of[s.number]], end_of(StepNumberInBay(s.number, b, n)));
      }
    }
    for (const BayStep& s : report.steps) {
      const int k = stage_of[s.number];
      const int number = StepNumberInBay(s.number, b, n);
      if (s.kind == "module_take" && !taken[k]) {
        taken[k] = true;
        const double start = start_of(number);
        if (start >= 0.0 && start < last_bolt[k] - tol) {
          out.push_back(of + Fmt("arm %s starts step %d (%s) at %.3f s, before the stage's last "
                                 "strap bolt is driven at %.3f s",
                                 s.arm.c_str(), number, s.what.c_str(), start, last_bolt[k]));
        }
      }
      if (s.kind != "pins_take") continue;
      const ArmPlan* ap = nullptr;
      for (const ArmPlan& a : plan.arms) {
        if (a.name == s.arm) ap = &a;
      }
      const int arm = scene.ArmIndex(s.arm);
      if (ap == nullptr || arm < 0) continue;
      // The take's last segment, then the arm's next one that is not a wait.
      int last = -1;
      for (size_t g = 0; g < ap->segments.size(); ++g) {
        if (ap->segments[g].step == number && ap->segments[g].kind != "wait") {
          last = static_cast<int>(g);
        }
      }
      if (last < 0) continue;  // A step left out: the verifier says so.
      double parked = -1.0;
      for (size_t g = static_cast<size_t>(last) + 1; g < ap->segments.size(); ++g) {
        const Segment& x = ap->segments[g];
        if (x.kind == "wait") continue;
        if (x.kind == "move" && !x.path.empty() && at_wait(arm, x.path.back())) parked = x.t1;
        break;
      }
      if (parked < 0.0) {
        out.push_back(of + Fmt("arm %s does not park at its wait pose after its magazine take, "
                               "step %d (in staged work the magazine comes before the stage's "
                               "module sets)",
                               s.arm.c_str(), number));
        continue;
      }
      for (const BayStep& m : report.steps) {
        if (m.kind != "module_set" || stage_of[m.number] != k) continue;
        const int mn = StepNumberInBay(m.number, b, n);
        const double start = start_of(mn);
        if (start >= 0.0 && start < parked - tol) {
          out.push_back(of + Fmt("step %d (%s) starts at %.3f s, before arm %s has taken its pin "
                                 "magazine (step %d) and parked at its wait pose at %.3f s",
                                 mn, m.what.c_str(), start, s.arm.c_str(), number, parked));
        }
      }
    }
  }
  return out;
}

}  // namespace cell::layout
