// Plans bays of a layout back to back (src/layout_plan.h, src/layout_bays.h)
// and prints one row per move.
//
//   plan_layout <name> [--bays N] [--no-early-steps] [--summary] [--out PATH] [--threads N]
//               [--first-step A --last-step B] [--pretty]
//               [--budgets A,B,C] [--why STEP]... [--why all-failed]
//   plan_layout <name> --margins [--pretty]
//
// The default output is CSV: a header and one row per move
//   step,arm,from,to,found,budget,waypoints,queries,planned_s,model_s,ratio,note
// (budget: the iteration budget that found it, 0 when the straight line was
// free; planned_s and model_s both with the settle; ratio planned_s /
// model_s, empty when model_s is 0), then `#` lines: the figures, each
// phase's planned and model seconds, each arm's working, moving and waiting
// seconds, every wait with its reason, how long planning took and its
// collision queries, and the faults. --pretty lines the CSV's columns up.
// --summary prints `key: value` lines instead, with each bay's events and
// the cycle's figures (CycleFiguresOf). --bays N plans N bays back to back
// (default 3; 1 gives the plan of one bay as before; a part of the bay,
// --first-step, is always one bay). --no-early-steps keeps every step of a
// later bay after its clamp (without R10, src/layout_bays.h, TouchesTheBay).
// --out PATH writes the plan as JSON
// (PlanToJson). --budgets sets the RRT iteration budgets (default
// the figures' 5000,40000,160000). --why STEP explains the moves that lead
// to a step and plans nothing else; --why all-failed plans as usual and then
// explains every move not found. Layouts are read from LAYOUT_DIR.
//
// --margins plans nothing: it reads the committed plan PLAN_DIR/<name>.json,
// refuses it (exit 1) when it was made for another version of the layout, runs
// the verifier and the checker, and prints one CSV table of the tightest
// margins (kinds distance, arm_pair, joint_margin, tool_fit, load, ways,
// transport, track_end and carriage_gap, tightest first within each) and then
// `#` lines. track_end is each tracked arm's least slack to its track's lo
// and hi over the plan (limit 0); carriage_gap is, for each pair of arms on
// one beam, the least distance between their carriages along it, against
// the distance at which the carriage boxes touch (src/layout_margins.h).
// After the boundary lines it prints the critical section (run 055,
// src/layout_critical.h): for each bay the chain of steps that ends at its
// release, one `critical` row per link from the chain's root (its start,
// end, the wait before it and what it waited for, its motion, work and
// torque slowdown, and why it starts then), one `critical_arm` row per arm
// on the chain with its share of the chain, and one `critical_next` row per
// other arm's last module step, the ends that would set the bay next. It
// exits 0 when the plan verifies and every checker row passes.
//
// Exit status: 0 when the plan is complete; 1 when a move was not found,
// the schedule is blocked or the verifier found a fault; 2 on a usage or
// loading error.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <optional>
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_check.h"
#include "src/layout_critical.h"
#include "src/layout_model.h"
#include "src/layout_margins.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

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
               "usage: plan_layout <name> [--bays N] [--no-early-steps] [--summary] [--out PATH] "
               "[--threads N] [--first-step A --last-step B] [--pretty] "
               "[--budgets A,B,C] [--why STEP|all-failed]... "
               "| plan_layout <name> --margins [--pretty]\n");
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

// How often --margins samples the tracks (src/layout_margins.h).
constexpr double kTrackSampleS = 0.01;

// --margins: reads the committed plan, verifies it and prints the tightest
// margins of the plan and of the checker. Plans nothing.
// The critical section of --margins: for each bay, one row per link of the
// chain that ends at its release (walked back from it, printed from its
// root), then one critical_arm row per arm on the chain (events: arm
// "events") and one critical_next row per other arm's last module step.
void PrintCritical(const std::vector<CriticalChain>& chains, bool pretty) {
  std::vector<std::vector<std::string>> rows;
  rows.push_back({"critical", "bay", "link", "step", "what", "arm", "start_s", "end_s",
                  "chain_s", "wait_s", "way_s", "way_for", "hold_s", "motion_s", "work_s",
                  "slowed_s", "cause", "why"});
  for (const CriticalChain& c : chains) {
    int k = 0;
    for (const ChainLink& l : c.links) {
      rows.push_back({"critical", std::to_string(c.bay), std::to_string(++k),
                      std::to_string(l.step), l.what, l.arm.empty() ? "events" : l.arm,
                      Num(l.start), Num(l.end), Num(l.end - l.start), Num(l.wait_s),
                      Num(l.way_s), l.by, Num(l.hold_s),
                      Num(l.motion_s), Num(l.work_s), Num(l.slowed_s), l.cause, l.text});
    }
  }
  PrintTable(rows, pretty);
  std::vector<std::vector<std::string>> sums;
  sums.push_back({"critical_arm", "bay", "arm", "links", "motion_s", "work_s", "slowed_s",
                  "wait_s", "total_s", "share_of_chain"});
  for (const CriticalChain& c : chains) {
    const double span = c.to_s - c.from_s;
    for (const ChainArm& a : c.arms) {
      const double total = a.motion_s + a.work_s + a.wait_s;
      sums.push_back({"critical_arm", std::to_string(c.bay), a.arm.empty() ? "events" : a.arm,
                      std::to_string(a.links), Num(a.motion_s), Num(a.work_s), Num(a.slowed_s),
                      Num(a.wait_s), Num(total), span > 0.0 ? Num(total / span, 4) : ""});
    }
  }
  PrintTable(sums, pretty);
  std::vector<std::vector<std::string>> next;
  next.push_back({"critical_next", "bay", "arm", "step", "what", "end_s", "chain_s",
                  "before_release_s", "shared_step", "shared_until_s"});
  for (const CriticalChain& c : chains) {
    const double last = c.links.size() >= 2 ? c.links[c.links.size() - 2].block_end : c.to_s;
    for (const CriticalChain::Next& n : c.next) {
      next.push_back({"critical_next", std::to_string(c.bay), n.arm, std::to_string(n.step),
                      n.what, Num(n.end), Num(n.span_s), Num(last - n.end),
                      std::to_string(n.shared_step), Num(n.shared_until)});
    }
  }
  PrintTable(next, pretty);
  for (const CriticalChain& c : chains) {
    std::printf("# critical bay %d: %s s from %s s (%s) to the release's end at %s s, %d links\n",
                c.bay, Num(c.to_s - c.from_s).c_str(), Num(c.from_s).c_str(),
                c.bay == 0 ? "the first tube_in" : "the bay before's release ends",
                Num(c.to_s).c_str(), static_cast<int>(c.links.size()));
  }
}

int PrintMargins(const Layout& layout, const ModelPaths& paths,
                 const std::string& layout_path, const std::string& sha,
                 const std::string& name, bool pretty) {
  const char* dir = std::getenv("PLAN_DIR");
  const std::string plan_path =
      std::string(dir != nullptr && dir[0] != '\0' ? dir : "plans") + "/" +
      name + ".json";
  std::ifstream file(plan_path, std::ios::binary);
  if (!file) {
    std::fprintf(stderr, "plan_layout: cannot read %s\n", plan_path.c_str());
    return 2;
  }
  const std::string text((std::istreambuf_iterator<char>(file)),
                         std::istreambuf_iterator<char>());
  std::string error;
  const std::optional<Plan> plan = PlanFromJson(text, &error);
  if (!plan) {
    std::fprintf(stderr, "plan_layout: %s: %s\n", plan_path.c_str(),
                 error.c_str());
    return 2;
  }
  VerifyOptions options;
  options.keep = 20;
  const VerifyReport v = VerifyPlan(layout, paths, *plan, sha, options);
  if (!v.ok && v.least.empty() && v.arm_pairs.empty()) {
    for (const std::string& f : v.faults) {
      std::fprintf(stderr, "plan_layout: %s\n", f.c_str());
    }
    return 1;
  }
  const CheckReport check = CheckLayout(layout, paths);
  const PlanFigures& fig = plan->figures;
  bool all_pass = v.ok;
  std::vector<std::vector<std::string>> table;
  table.push_back({"kind", "rank", "figure", "unit", "limit", "a", "b", "arm",
                   "step", "time_s", "detail"});
  auto against = [](const Closest& c) {
    return c.other_arm.empty() ? std::string("against the cell")
                               : "against " + c.other_arm;
  };
  int rank = 0;
  for (const Closest& c : v.least) {
    table.push_back({"distance", std::to_string(++rank), Num(c.distance, 4),
                     "m", Num(fig.clearance_m, 4), c.a, c.b, c.arm,
                     std::to_string(c.step), Num(c.t), against(c)});
  }
  rank = 0;
  for (const Closest& c : v.arm_pairs) {
    const std::string other = c.other_arm.empty() ? "?" : c.other_arm;
    table.push_back({"arm_pair", std::to_string(++rank), Num(c.distance, 4),
                     "m", Num(fig.clearance_m, 4), c.a, c.b, c.arm,
                     std::to_string(c.step), Num(c.t),
                     "arms " + c.arm + " and " + other});
  }
  rank = 0;
  for (const VerifyReport::JointMargin& j : v.joint_margins) {
    table.push_back({"joint_margin", std::to_string(++rank),
                     Num(j.margin_deg, 2), "deg", Num(fig.joint_margin_deg, 1),
                     j.joint, "", j.arm, std::to_string(j.step), Num(j.t), ""});
  }
  // The checker's rows of a kind, tightest first: by how far the figure is
  // inside its limit (for a mass, the share of the limit that is left).
  auto add_rows = [&](const std::string& kind, const std::string& check_name,
                      const std::string& prefix) {
    std::vector<const CheckRow*> rows;
    for (const CheckRow& r : check.rows) {
      if (r.check != check_name) continue;
      if (r.item.compare(0, prefix.size(), prefix) != 0) continue;
      all_pass = all_pass && r.pass;
      rows.push_back(&r);
    }
    auto tightness = [](const CheckRow* r) {
      if (r->unit == "kg" && r->limit > 0.0) {
        return (r->limit - r->figure) / r->limit;
      }
      return r->figure;
    };
    std::stable_sort(rows.begin(), rows.end(),
                     [&](const CheckRow* x, const CheckRow* y) {
                       return tightness(x) < tightness(y);
                     });
    int n = 0;
    for (const CheckRow* r : rows) {
      const bool tool = kind == "tool_fit";
      // A tool's row names the kind of work after "tool in free space".
      std::string work;
      if (tool && r->item.size() > prefix.size() + 1) {
        work = r->item.substr(prefix.size() + 1);
        work = work.substr(0, work.find(' '));
      }
      table.push_back({kind, std::to_string(++n), Num(r->figure, 4), r->unit,
                       Num(r->limit, 4), tool ? r->tool : r->item, work,
                       r->arm, "", "", r->detail});
    }
  };
  add_rows("tool_fit", "clearance", "tool in free space");
  add_rows("load", "load", "");
  add_rows("ways", "ways", "");
  add_rows("transport", "transport", "");
  // Track ends and carriage gaps (src/layout_margins.h), from the plan's
  // timed paths.
  std::string scene_error;
  const std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(layout, paths, check.joint_margin_deg, &scene_error);
  if (!scene) {
    std::fprintf(stderr, "plan_layout: %s\n", scene_error.c_str());
    return 2;
  }
  const PlanPlayer player(*scene, check, *plan);
  const TrackMargins tracks = TrackMarginsOf(layout, player, kTrackSampleS);
  rank = 0;
  for (const TrackEnd& e : tracks.ends) {
    const ArmUse* arm = FindArm(layout, e.arm);
    table.push_back({"track_end", std::to_string(++rank), Num(e.slack, 4), "m",
                     Num(0.0, 4), e.end, "", e.arm, std::to_string(e.step),
                     Num(e.t), "track " + Num(arm->track->lo, 3) + " to " +
                     Num(arm->track->hi, 3) + ", least at " + Num(e.value, 3)});
  }
  rank = 0;
  for (const CarriageGap& g : tracks.gaps) {
    table.push_back({"carriage_gap", std::to_string(++rank), Num(g.distance, 4), "m",
                     Num(g.contact, 4), g.a, g.b, g.a, std::to_string(g.step_a),
                     Num(g.t), g.b + " at step " + std::to_string(g.step_b) +
                     "; boxes " + Num(g.distance - g.contact, 4) + " m apart"});
  }
  // Run 053: where the ways do not share their zone, at each boundary the
  // least distance of the incoming tube to the outgoing bay (TubePassesBay,
  // every sample, out to kSceneFar) and of the arms off their wait poses to
  // the incoming tube (the verifier's), with the seconds the ways overlap.
  std::vector<std::string> boundaries;
  if (!WaysShareZone(layout)) {
    cell::DataPtr d = scene->MakeData();
    for (int b = 1; b < plan->bays; ++b) {
      const BayEvent* out = nullptr;
      const BayEvent* in = nullptr;
      for (const BayEvent& e : plan->events) {
        if (e.bay == b - 1 && e.kind == "bay_out") out = &e;
        if (e.bay == b && e.kind == "tube_in") in = &e;
      }
      if (out == nullptr || in == nullptr) continue;
      const WayPassing p =
          TubePassesBay(*scene, d.get(), *out, in->t0, fig.sample_s, fig.clearance_m, kSceneFar);
      all_pass = all_pass && p.clear;
      table.push_back({"tube_bay", std::to_string(b), Num(p.least_m, 4), "m",
                       Num(fig.clearance_m, 4), "tube_in " + std::to_string(b),
                       "bay_out " + std::to_string(b - 1), "", std::to_string(in->step),
                       Num(p.least_t), "the incoming tube to the outgoing bay's rails and modules"});
      boundaries.push_back(
          "# boundary " + std::to_string(b - 1) + "/" + std::to_string(b) + ": bay_out " +
          Num(out->t0, 2) + " to " + Num(out->t1, 2) + " s, tube_in " + Num(in->t0, 2) + " to " +
          Num(in->t1, 2) + " s, overlap " +
          Num(std::max(0.0, std::min(out->t1, in->t1) - std::max(out->t0, in->t0)), 2) + " s");
    }
    table.push_back({"tube_arms", "1", Num(v.tube_in_least_m, 4), "m", Num(fig.clearance_m, 4),
                     "", "", "", "", "",
                     "arms off their wait poses to the tube coming in (the verifier's)"});
  }
  PrintTable(table, pretty);
  for (const std::string& line : boundaries) std::printf("%s\n", line.c_str());
  // Run 055: each bay's critical chain (src/layout_critical.h), from the
  // plan's times and the scheduler's rules.
  std::string chain_error;
  const std::vector<CriticalChain> chains =
      CriticalChains(layout, check, *scene, player, &chain_error);
  if (chains.empty()) {
    std::printf("# critical: %s\n", chain_error.c_str());
  } else {
    PrintCritical(chains, pretty);
  }
  std::printf("# layout: %s\n", layout_path.c_str());
  std::printf("# plan sha256: %s\n", plan->layout_sha256.c_str());
  std::printf("# least_clearance_m %s\n", Num(v.least_clearance_m, 4).c_str());
  std::printf("# least_arm_to_arm_m %s\n",
              Num(v.least_arm_to_arm_m, 4).c_str());
  if (!v.least.empty() &&
      Num(v.least.front().distance, 4) != Num(v.least_clearance_m, 4)) {
    std::printf("# note: the first distance row is not least_clearance_m\n");
  }
  std::printf("# schedule collision queries: %lld\n",
              static_cast<long long>(v.queries));
  std::printf("# track_end and carriage_gap: %d instants, every %s s and at "
              "every segment's ends\n",
              tracks.samples, Num(kTrackSampleS, 3).c_str());
  for (const std::string& f : v.faults) std::printf("# fault: %s\n", f.c_str());
  return all_pass ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  // Each line as it is printed, also into a file or a pipe, so that a long
  // probe shows its progress.
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  std::string name, out;
  bool summary = false, pretty = false, margins = false;
  PlanOptions options;
  options.bays = 3;
  std::vector<int> why;
  bool why_failed = false;
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
    } else if (a == "--no-early-steps") {
      options.early_steps = false;
    } else if (a == "--bays") {
      if (!value(&v)) return Usage();
      char* end = nullptr;
      const long bays = std::strtol(v.c_str(), &end, 10);
      if (end == v.c_str() || *end != '\0' || bays < 1 || bays > 100) return Usage();
      options.bays = static_cast<int>(bays);
    } else if (a == "--first-step") {
      if (!value(&v)) return Usage();
      options.first_step = std::atoi(v.c_str());
    } else if (a == "--last-step") {
      if (!value(&v)) return Usage();
      options.last_step = std::atoi(v.c_str());
    } else if (a == "--why") {
      if (!value(&v)) return Usage();
      if (v == "all-failed") {
        why_failed = true;
      } else {
        const int step = std::atoi(v.c_str());
        if (step <= 0) return Usage();
        why.push_back(step);
      }
    } else if (a == "--budgets") {
      if (!value(&v)) return Usage();
      options.budgets.clear();
      size_t at = 0;
      while (at <= v.size()) {
        const size_t comma = v.find(',', at);
        const std::string part =
            v.substr(at, comma == std::string::npos ? std::string::npos
                                                    : comma - at);
        const int b = std::atoi(part.c_str());
        if (b <= 0) return Usage();
        options.budgets.push_back(b);
        if (comma == std::string::npos) break;
        at = comma + 1;
      }
    } else if (a == "--margins") {
      margins = true;
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
  if (margins) return PrintMargins(*layout, paths, path, sha, name, pretty);
  // --why STEP: explain the moves that lead to the steps, planning nothing
  // else.
  if (!why.empty() && !why_failed) {
    for (const std::string& line : WhyMoves(*layout, paths, why, options)) {
      std::printf("%s\n", line.c_str());
    }
    return 0;
  }
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
    std::printf("plan_pad_m: %s\n", Num(plan.figures.plan_pad_m, 4).c_str());
    std::printf("joint_margin_deg: %s\n",
                Num(plan.figures.joint_margin_deg, 1).c_str());
    std::printf("moves: %d\n", static_cast<int>(r.rows.size()));
    std::printf("moves_found: %d\n", found);
    std::printf("bay_planned_s: %s\n", Num(plan.bay_planned_s).c_str());
    std::printf("bay_model_s: %s\n", Num(plan.bay_model_s).c_str());
    std::printf("bays: %d\n", plan.bays);
    // The torque bound (src/layout_torque.h), only when the layout has it.
    if (plan.figures.torque_limit_scale > 0.0) {
      int slowed = 0;
      double added = 0.0;
      for (const MoveRow& row : r.rows) {
        if (row.time_scale <= 1.0) continue;
        ++slowed;
        added += (row.planned_s - plan.figures.settle_s) * (1.0 - 1.0 / row.time_scale);
      }
      std::printf("torque_limit_scale: %s\n", Num(plan.figures.torque_limit_scale, 3).c_str());
      std::printf("moves_slowed: %d\n", slowed);
      std::printf("moves_slowed_added_s: %s\n", Num(added).c_str());
    }
    for (const BayEvent& e : plan.events) {
      const std::string kind =
          e.kind == "transfer" ? "transfer_" + std::to_string(e.from_stage) + "_to_" +
                                     std::to_string(e.to_stage)
                               : e.kind;
      std::printf("bay_%d_%s: %s to %s s\n", e.bay, kind.c_str(),
                  Num(e.t0).c_str(), Num(e.t1).c_str());
    }
    // Staged work: each stage's stretch, from the clamp's or its transfer's
    // end to the next transfer's start or the bay_out's.
    const std::vector<Stage> stages = StagesOf(*layout);
    for (int b = 0; layout->staging && b < plan.bays; ++b) {
      double from = 0.0;
      for (const BayEvent& e : plan.events) {
        if (e.bay != b || (e.kind != "clamp" && e.kind != "transfer" && e.kind != "bay_out")) {
          continue;
        }
        if (e.kind != "clamp") {
          const int k = e.kind == "transfer" ? e.from_stage - 1 : static_cast<int>(stages.size()) - 1;
          std::printf("bay_%d_stage_%s: %s to %s s (%s s)\n", b, stages[k].name.c_str(),
                      Num(from).c_str(), Num(e.t0).c_str(), Num(e.t0 - from).c_str());
        }
        from = e.t1;
      }
    }
    // How long bay b+1's tube comes in while bay b goes out.
    for (int b = 0; b + 1 < plan.bays; ++b) {
      double out = 0.0, in = 0.0;
      for (const BayEvent& e : plan.events) {
        if (e.bay == b && e.kind == "bay_out") out = e.t1;
        if (e.bay == b + 1 && e.kind == "tube_in") in = e.t0;
      }
      std::printf("bay_%d_out_bay_%d_in_overlap_s: %s\n", b, b + 1,
                  Num(std::max(0.0, out - in)).c_str());
    }
    const CycleFigures cycle = CycleFiguresOf(plan);
    std::printf("one_bay_s: %s\n", Num(cycle.one_bay_s).c_str());
    std::printf("cycle_s: %s\n", Num(cycle.cycle_s).c_str());
    std::printf("interval_s: %s\n", Num(cycle.interval_s).c_str());
    for (const PhaseTime& p : plan.phases) {
      std::string key = p.name;
      for (char& c : key) {
        if (c == ' ') c = '_';
      }
      if (plan.bays > 1) key = "bay_" + std::to_string(p.bay) + "_" + key;
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
        "# figures: clearance_m %s (planned at + plan_pad_m %s), "
        "joint_margin_deg %s, to_full_speed_s %s, "
        "jerk_rise_s %s, settle_s %s, leg_speed %s m/s, leg_accel %s m/s^2, "
        "retreat_speed %s m/s, retreat_accel %s m/s^2, "
        "max_deviation %s rad, edge_resolution %s m, shorten_tries %d, "
        "budgets %s, seed %u, sample_s %s; planned_s and model_s include the "
        "settle\n",
        Num(f.clearance_m, 4).c_str(), Num(f.plan_pad_m, 4).c_str(),
        Num(f.joint_margin_deg, 1).c_str(),
        Num(f.to_full_speed_s).c_str(), Num(f.jerk_rise_s).c_str(),
        Num(f.settle_s).c_str(), Num(f.leg_speed).c_str(),
        Num(f.leg_accel).c_str(), Num(f.retreat_speed).c_str(),
        Num(f.retreat_accel).c_str(), Num(f.max_deviation, 4).c_str(),
        Num(f.edge_resolution, 4).c_str(), f.shorten_tries, budgets.c_str(),
        f.seed, Num(f.sample_s).c_str());
    for (const PhaseTime& p : plan.phases) {
      const std::string bay =
          plan.bays > 1 ? "bay " + std::to_string(p.bay) + " " : "";
      std::printf("# phase %s%s: planned %s s, model %s s\n", bay.c_str(),
                  p.name.c_str(), Num(p.planned_s).c_str(),
                  Num(p.model_s).c_str());
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
  // --why all-failed: after the normal output, explain every move that was
  // not found (and any steps asked for by number).
  if (why_failed) {
    for (const MoveRow& row : r.rows) {
      if (!row.found &&
          std::find(why.begin(), why.end(), row.step) == why.end()) {
        why.push_back(row.step);
      }
    }
    if (!why.empty()) {
      std::printf("# why:\n");
      for (const std::string& line : WhyMoves(*layout, paths, why, options)) {
        std::printf("%s\n", line.c_str());
      }
    }
  }
  return plan.complete ? 0 : 1;
}
