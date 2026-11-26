// The checks of staged work and of two racks' gap, put together from the
// checks of one place of the tube (src/layout_stages.h, src/layout_check.h).

#include "src/layout_stages.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "src/bay_hardware.h"

namespace cell::layout {
namespace {

// The most runs checked side by side.
constexpr unsigned kMaxThreads = 4;

std::string Fmt(const char* format, double a) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), format, a);
  return buffer;
}

bool Has(const std::vector<int>& v, int x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

bool StartsWith(const std::string& s, const std::string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

// The stage of each module and of each rail.
std::vector<int> ModuleStages(const std::vector<Stage>& stages) {
  std::vector<int> out(bay::kModuleCount, 0);
  for (size_t s = 0; s < stages.size(); ++s) {
    for (int m : StageModules(stages[s])) out[m] = static_cast<int>(s);
  }
  return out;
}

std::vector<int> RailStages(const std::vector<Stage>& stages) {
  std::vector<int> out(bay::kRailCount, 0);
  for (size_t s = 0; s < stages.size(); ++s) {
    for (int r : StageRails(stages[s])) out[r] = static_cast<int>(s);
  }
  return out;
}

// The stage of an arm's pin magazine (pins_take point i): its first hole's,
// in the staged order of modules; an arm with no hole takes it after the
// first module is set.
int MagazineStage(const Layout& layout, int i) {
  const std::vector<Stage> stages = StagesOf(layout);
  const std::vector<int> module_stage = ModuleStages(stages);
  const std::vector<int> order = StagedModuleOrder(layout);
  if (order.empty()) return 0;
  const Work* w = FindWork(layout, "pins_take", i);
  if (w != nullptr) {
    for (int m : order) {
      for (const char* kind : {"locate", "pin", "swage"}) {
        for (int h = 0; h < bay::kPinsPerModule; ++h) {
          const Work* hw = FindWork(layout, kind, bay::kPinsPerModule * m + h);
          if (hw != nullptr && hw->arm == w->arm) return module_stage[m];
        }
      }
    }
  }
  return module_stage[order[0]];
}

std::string Prefix(const std::string& stage) { return "stage " + stage + ": "; }

// Rows that do not depend on the tube's place or the bay as built: made
// once, from the first stage's run.
bool OnceRow(const CheckRow& r) {
  if (r.check == "reach") {
    return StartsWith(r.item, "wait track ") || StartsWith(r.item, "wait pose ");
  }
  if (r.check == "clearance") return StartsWith(r.item, "tool in free space ");
  return false;
}

// One check of the tube at one place: a stage, or the whole bay.
struct Run {
  int variant = 0;  // 0: nominal; v: GapVariants()[v - 1].
  int stage = 0;
  Layout layout;
  CheckOptions options;
  CheckReport report;
};

void RunAll(std::vector<Run>* runs, const ModelPaths& paths) {
  std::atomic<size_t> next{0};
  auto work = [&] {
    for (size_t i = next++; i < runs->size(); i = next++) {
      Run& r = (*runs)[i];
      r.report = CheckAtOnePlace(r.layout, paths, r.options);
    }
  };
  const unsigned hw = std::max(1u, std::thread::hardware_concurrency());
  const size_t n = std::min<size_t>({runs->size(), hw, kMaxThreads});
  std::vector<std::thread> threads;
  for (size_t t = 1; t < n; ++t) threads.emplace_back(work);
  work();
  for (std::thread& t : threads) t.join();
}

const CheckRow* Find(const CheckReport& r, const std::string& check,
                     const std::string& item) {
  for (const CheckRow& row : r.rows) {
    if (row.check == check && row.item == item) return &row;
  }
  return nullptr;
}

// The rows of one kind of a staged bay from its stages' reports (in stage
// order), as the opening comment of src/layout_check.h, "Stages", orders
// them.
std::vector<CheckRow> StagedRows(const Layout& layout,
                                 const std::vector<const CheckReport*>& reports,
                                 const std::string& kind) {
  const std::vector<Stage> stages = StagesOf(layout);
  std::vector<CheckRow> out;
  auto of_kind = [&](const CheckReport& r, auto keep) {
    for (const CheckRow& row : r.rows) {
      if (row.check == kind && keep(row)) out.push_back(row);
    }
  };
  auto prefixed = [&](size_t s, auto keep) {
    for (const CheckRow& row : reports[s]->rows) {
      if (row.check != kind || !keep(row)) continue;
      CheckRow p = row;
      p.item = Prefix(stages[s].name) + row.item;
      out.push_back(p);
    }
  };
  auto all = [](const CheckRow&) { return true; };
  if (kind == "transport" || kind == "load" || kind == "allowed" ||
      kind == "port") {
    of_kind(*reports[0], all);
  } else if (kind == "ways") {
    for (const CheckReport* r : reports) of_kind(*r, all);
  } else if (kind == "hold") {
    for (size_t s = 0; s < reports.size(); ++s) {
      prefixed(s, all);
      for (CheckRow row : GripInHolderRows(StageLayout(layout, static_cast<int>(s)))) {
        row.item = Prefix(stages[s].name) + row.item;
        out.push_back(row);
      }
    }
  } else if (kind == "time") {
    for (size_t s = 0; s < reports.size(); ++s) prefixed(s, all);
    const double transfer = layout.staging ? layout.staging->transfer_s : 0.0;
    double total = 0.0;
    bool pass = true;
    std::string detail;
    for (size_t s = 0; s < reports.size(); ++s) {
      const CheckRow* bay = Find(*reports[s], "time", "bay");
      if (s > 0) {
        total += transfer;
        detail += " + transfer" + Fmt(" %.1f", transfer) + " + ";
      }
      total += bay != nullptr ? bay->figure : 0.0;
      pass = pass && bay != nullptr && bay->pass;
      detail += "stage " + stages[s].name +
                (bay != nullptr ? Fmt(" %.1f", bay->figure) : " (no time)");
    }
    if (reports.empty() || Find(*reports[0], "time", "bay") == nullptr) {
      return out;
    }
    out.push_back({"time", "bay", "", "", total, "s", 0.0, pass, detail});
    for (const ArmUse& a : layout.arms) {
      double busy = 0.0;
      for (const CheckReport* r : reports) {
        const CheckRow* row = Find(*r, "time", "busy " + a.name);
        busy += row != nullptr ? row->figure : 0.0;
      }
      out.push_back({"time", "busy " + a.name, a.name, "", busy, "s", 0.0, pass,
                     "the sum of its steps over the stages"});
    }
  } else {  // reach, clearance, approach
    of_kind(*reports[0], OnceRow);
    for (size_t s = 0; s < reports.size(); ++s) {
      prefixed(s, [](const CheckRow& r) { return !OnceRow(r); });
    }
  }
  return out;
}

// The kinds a gap variant reruns.
const std::vector<std::string>& GapKinds() {
  static const std::vector<std::string> kinds = {"reach", "clearance", "ways",
                                                 "approach"};
  return kinds;
}

// The row of one gap variant from its rerun rows.
CheckRow GapRow(const GapVariant& v, const std::vector<CheckRow>& rows,
                double clearance_m) {
  const CheckRow* least = nullptr;
  const CheckRow* worst = nullptr;
  std::map<std::string, int> failing;
  int fails = 0;
  for (const CheckRow& r : rows) {
    const bool distance = r.check != "reach" && r.unit == "m" &&
                          std::abs(r.limit - clearance_m) < 1e-12;
    if (distance && (least == nullptr || r.figure < least->figure)) least = &r;
    if (r.pass) continue;
    ++fails;
    ++failing[r.check];
    // The worst failing row: of distances, the one furthest below its
    // limit; else of the others (joint margins), likewise.
    const bool m = r.unit == "m";
    const bool worst_m = worst != nullptr && worst->unit == "m";
    if (worst == nullptr || (m && !worst_m) ||
        (m == worst_m && r.figure - r.limit < worst->figure - worst->limit)) {
      worst = &r;
    }
  }
  // A failing least distance is the worst row.
  if (least != nullptr && !least->pass) worst = least;
  auto name = [](const CheckRow& r) { return r.check + " " + r.item; };
  std::string detail;
  if (least != nullptr) {
    detail = "least" + Fmt(" %.4f m", least->figure) + " at " + name(*least);
  } else {
    detail = "no distance rows";
  }
  if (fails == 0) {
    detail += "; all " + std::to_string(rows.size()) + " rows pass";
  } else {
    std::string kinds;
    for (const std::string& k : CheckKinds()) {
      auto it = failing.find(k);
      if (it == failing.end()) continue;
      kinds += (kinds.empty() ? "" : ", ") + k + " " + std::to_string(it->second);
    }
    detail += "; " + std::to_string(fails) + " of " +
              std::to_string(rows.size()) + " rows fail (" + kinds +
              "); worst " + name(*worst) + Fmt(" %.4f", worst->figure) + " " +
              worst->unit + " against" + Fmt(" %.4f", worst->limit);
  }
  return {"gap",  v.item,
          "",     "",
          least != nullptr ? least->figure : 0.0,
          "m",    clearance_m,
          fails == 0 && !rows.empty(),
          detail};
}

}  // namespace

std::vector<int> StagedModuleOrder(const Layout& layout) {
  const std::vector<int> order = ModuleOrder(layout);
  if (!layout.staging) return order;
  std::vector<int> out;
  for (const Stage& st : StagesOf(layout)) {
    const std::vector<int> mods = StageModules(st);
    for (int m : order) {
      if (Has(mods, m)) out.push_back(m);
    }
  }
  return out;
}

Layout StageLayout(const Layout& layout, int s) {
  const std::vector<Stage> stages = StagesOf(layout);
  Layout l = AtStage(layout, stages.at(s));
  if (layout.staging) {
    const std::vector<int> order = StagedModuleOrder(layout);
    if (order != ModuleOrder(layout)) l.order.modules = order;
  }
  return l;
}

StageScope ScopeOfStage(const Layout& layout, int s) {
  const std::vector<Stage> stages = StagesOf(layout);
  const std::vector<int> order = StagedModuleOrder(layout);
  const std::vector<int> module_stage = ModuleStages(stages);
  const int last = static_cast<int>(stages.size()) - 1;
  StageScope sc;
  sc.name = stages.at(s).name;
  sc.first = s == 0;
  sc.last = s == last;
  for (int i = 0; i < s; ++i) {
    for (int r : StageRails(stages[i])) sc.rails_before.push_back(r);
  }
  std::sort(sc.rails_before.begin(), sc.rails_before.end());
  sc.rails = StageRails(stages[s]);
  std::vector<int> takes;
  for (size_t k = 0; k < order.size(); ++k) {
    const int m = order[k];
    if (module_stage[m] < s) sc.modules_before.push_back(m);
    if (module_stage[m] == s) {
      sc.modules.push_back(m);
      takes.push_back(static_cast<int>(k));
    }
  }
  std::sort(sc.modules_before.begin(), sc.modules_before.end());
  sc.points["rail_take"] = sc.rails;
  sc.points["rail_set"] = sc.rails;
  sc.points["strap_bolt"] = sc.rails;
  std::vector<int> mods = sc.modules;
  std::sort(mods.begin(), mods.end());
  sc.points["module_set"] = mods;
  // The takes the bay does not use, from the other pallets, at the last
  // stage, with the whole bay built.
  if (sc.last) {
    for (int j = bay::kModuleCount; j < PointCount(layout, "module_take"); ++j) {
      takes.push_back(j);
    }
  }
  std::sort(takes.begin(), takes.end());
  sc.points["module_take"] = takes;
  std::vector<int> holes;
  for (int m : mods) {
    for (int h = 0; h < bay::kPinsPerModule; ++h) {
      holes.push_back(bay::kPinsPerModule * m + h);
    }
  }
  sc.points["locate"] = holes;
  sc.points["pin"] = holes;
  sc.points["swage"] = holes;
  std::vector<int> magazines;
  for (int i = 0; i < PointCount(layout, "pins_take"); ++i) {
    if (MagazineStage(layout, i) == s) magazines.push_back(i);
  }
  sc.points["pins_take"] = magazines;
  if (!sc.last) {
    sc.next = stages[s + 1].name;
    sc.to_next_m = stages[s + 1].tube_offset_m - stages[s].tube_offset_m;
  }
  return sc;
}

int StageOfPoint(const Layout& layout, const std::string& kind, int index) {
  if (!layout.staging) return 0;
  const std::vector<Stage> stages = StagesOf(layout);
  const int last = static_cast<int>(stages.size()) - 1;
  if (kind == "rail_take" || kind == "rail_set" || kind == "strap_bolt") {
    return index >= 0 && index < bay::kRailCount ? RailStages(stages)[index] : 0;
  }
  const std::vector<int> module_stage = ModuleStages(stages);
  if (kind == "module_set") {
    return index >= 0 && index < bay::kModuleCount ? module_stage[index] : 0;
  }
  if (kind == "module_take") {
    const std::vector<int> order = StagedModuleOrder(layout);
    return index >= 0 && index < static_cast<int>(order.size())
               ? module_stage[order[index]]
               : last;
  }
  if (kind == "locate" || kind == "pin" || kind == "swage") {
    const int m = index / bay::kPinsPerModule;
    return m >= 0 && m < bay::kModuleCount ? module_stage[m] : 0;
  }
  if (kind == "pins_take") return MagazineStage(layout, index);
  return 0;
}

void BuiltInStage(const StageScope& stage, StepScene* scene) {
  for (Built& b : scene->built) {
    std::vector<int> rails = stage.rails_before;
    std::vector<int> modules = stage.modules_before;
    for (int r : b.rails) {
      if (Has(stage.rails, r) && !Has(rails, r)) rails.push_back(r);
    }
    for (int m : b.modules) {
      if (Has(stage.modules, m) && !Has(modules, m)) modules.push_back(m);
    }
    std::sort(rails.begin(), rails.end());
    std::sort(modules.begin(), modules.end());
    b.rails = rails;
    b.modules = modules;
  }
}

Built BuiltAtStageStart(const std::optional<StageScope>& stage,
                        int pallet_total) {
  Built b;
  b.on_pallets = pallet_total;
  if (stage) {
    b.rails = stage->rails_before;
    b.modules = stage->modules_before;
    b.on_pallets -= static_cast<int>(b.modules.size());
  }
  return b;
}

Built BuiltAtStageEnd(const std::optional<StageScope>& stage,
                      int pallet_total) {
  Built b;
  if (stage) {
    b.rails = stage->rails_before;
    b.rails.insert(b.rails.end(), stage->rails.begin(), stage->rails.end());
    b.modules = stage->modules_before;
    b.modules.insert(b.modules.end(), stage->modules.begin(),
                     stage->modules.end());
    std::sort(b.rails.begin(), b.rails.end());
    std::sort(b.modules.begin(), b.modules.end());
  } else {
    for (int k = 0; k < bay::kRailCount; ++k) b.rails.push_back(k);
    for (int k = 0; k < bay::kModuleCount; ++k) b.modules.push_back(k);
  }
  b.on_pallets =
      std::max(0, pallet_total - static_cast<int>(b.modules.size()));
  return b;
}

std::vector<CheckRow> GripInHolderRows(const Layout& stage_layout) {
  std::vector<CheckRow> rows;
  const std::vector<bay::Hold> holds = bay::Holds();
  for (const Grip& g : stage_layout.holder.grips) {
    const std::string item =
        "grip bay_hold_" + std::to_string(g.hold) + " in the holder";
    if (g.hold < 0 || g.hold >= static_cast<int>(holds.size())) {
      rows.push_back({"hold", item, "", "", 0.0, "m", 0.0, false,
                      "no such stretch"});
      continue;
    }
    // The gripped piece: the stretch's middle, at the stage's place of the
    // tube, +/- half the grip's length along Y.
    const double y =
        BayPoint(stage_layout, holds[g.hold].pose.translation()).y();
    const double lo = y - g.length / 2.0, hi = y + g.length / 2.0;
    double best = -1e9;
    std::string which = "the holder has no boxes";
    for (const Box& b : stage_layout.holder.boxes) {
      const double blo = b.pos[1] - b.size[1] / 2.0;
      const double bhi = b.pos[1] + b.size[1] / 2.0;
      const double inside = std::min(lo - blo, bhi - hi);
      if (inside > best) {
        best = inside;
        which = "box " + b.name + " y" + Fmt(" %.3f", blo) + ".." +
                Fmt("%.3f", bhi);
      }
    }
    const bool any = !stage_layout.holder.boxes.empty();
    rows.push_back({"hold", item, "", "", any ? best : 0.0, "m", 0.0,
                    any && best >= -1e-9,
                    "the piece y" + Fmt(" %.3f", lo) + ".." + Fmt("%.3f", hi) +
                        "; " + which});
  }
  return rows;
}

std::vector<GapVariant> GapVariants(const Layout& layout) {
  const TwoFlatRacksState& t = layout.two_flat_racks;
  return {{"gap" + Fmt(" %.0f mm", t.gap_lo_m * 1000.0), t.gap_lo_m - t.gap_m, 0.0},
          {"gap" + Fmt(" %.0f mm", t.gap_hi_m * 1000.0), t.gap_hi_m - t.gap_m, 0.0},
          {"misaligned" + Fmt(" %+.0f mm", t.misalign_m * 1000.0), 0.0,
           t.misalign_m},
          {"misaligned" + Fmt(" %+.0f mm", -t.misalign_m * 1000.0), 0.0,
           -t.misalign_m}};
}

CheckReport CheckLayout(const Layout& layout, const ModelPaths& paths,
                        const CheckOptions& options) {
  const bool staged = layout.staging.has_value();
  const bool two = layout.base == "two_flat_racks";
  if (!staged && !two) return CheckAtOnePlace(layout, paths, options);
  auto want = [&](const std::string& kind) {
    return options.checks.empty() ||
           std::find(options.checks.begin(), options.checks.end(), kind) !=
               options.checks.end();
  };
  const std::vector<Stage> stages = StagesOf(layout);
  const int n = staged ? static_cast<int>(stages.size()) : 1;
  const std::vector<GapVariant> variants =
      two && want("gap") ? GapVariants(layout) : std::vector<GapVariant>();
  // The kinds of the nominal runs: without staging the one run makes them
  // all; with it the first stage's run makes the rows that are made once.
  std::vector<std::string> nominal, once;
  for (const std::string& k : CheckKinds()) {
    if (k == "gap" || !want(k)) continue;
    nominal.push_back(k);
    if (k == "transport" || k == "load" || k == "allowed" || k == "port") {
      once.push_back(k);
    }
  }
  std::vector<std::string> rerun;
  for (const std::string& k : GapKinds()) {
    if (want("gap")) rerun.push_back(k);
  }
  std::vector<Run> runs;
  auto add = [&](int variant, const Layout& l, std::vector<std::string> kinds) {
    for (int s = 0; s < n; ++s) {
      Run r;
      r.variant = variant;
      r.stage = s;
      r.options = options;
      r.options.checks.clear();
      for (const std::string& k : kinds) {
        const bool is_once =
            std::find(once.begin(), once.end(), k) != once.end();
        if (variant > 0 || !is_once || s == 0) r.options.checks.push_back(k);
      }
      if (r.options.checks.empty()) continue;
      r.layout = staged ? StageLayout(l, s) : l;
      if (staged) r.options.stage = ScopeOfStage(l, s);
      runs.push_back(std::move(r));
    }
  };
  if (!nominal.empty()) add(0, layout, nominal);
  for (size_t v = 0; v < variants.size(); ++v) {
    add(static_cast<int>(v) + 1,
        MovedRackB(layout, variants[v].dx, variants[v].dy), rerun);
  }
  RunAll(&runs, paths);

  CheckReport report;
  report.time_model = TimeModelText();
  if (staged) {
    report.time_model +=
        "\nStaged work: each stage by the model above, with only its own "
        "rails and modules (tube in and clamp at the first stage, release "
        "and bay out at the last, and every arm back to its wait pose and "
        "first tool at the end of each), and staging.transfer_s for each "
        "move of the tube to the next stage; the bay is their sum.";
  }
  CheckOptions margins = options;
  margins.clearance_m = std::max(margins.clearance_m, layout.margins.clearance_m);
  margins.joint_margin_deg =
      std::max(margins.joint_margin_deg, layout.margins.joint_margin_deg);
  report.clearance_m = margins.clearance_m;
  report.joint_margin_deg = margins.joint_margin_deg;
  // The reports of one variant, in stage order; empty where none ran.
  auto reports_of = [&](int variant) {
    std::vector<const CheckReport*> out(n, nullptr);
    for (const Run& r : runs) {
      if (r.variant == variant) out[r.stage] = &r.report;
    }
    static const CheckReport kNone;
    for (const CheckReport*& p : out) {
      if (p == nullptr) p = &kNone;
    }
    return out;
  };
  const std::vector<const CheckReport*> main = reports_of(0);
  for (const std::string& kind : CheckKinds()) {
    if (kind == "gap" || !want(kind)) continue;
    if (!staged) {
      for (const CheckRow& r : main[0]->rows) {
        if (r.check == kind) report.rows.push_back(r);
      }
    } else {
      const std::vector<CheckRow> rows = StagedRows(layout, main, kind);
      report.rows.insert(report.rows.end(), rows.begin(), rows.end());
    }
  }
  for (size_t v = 0; v < variants.size(); ++v) {
    const std::vector<const CheckReport*> reps =
        reports_of(static_cast<int>(v) + 1);
    std::vector<CheckRow> rows;
    for (const std::string& kind : GapKinds()) {
      if (staged) {
        const std::vector<CheckRow> k = StagedRows(layout, reps, kind);
        rows.insert(rows.end(), k.begin(), k.end());
      } else {
        for (const CheckRow& r : reps[0]->rows) {
          if (r.check == kind) rows.push_back(r);
        }
      }
    }
    report.rows.push_back(GapRow(variants[v], rows, report.clearance_m));
  }
  // The steps, points, poses and phases of the nominal runs.
  const double transfer = layout.staging ? layout.staging->transfer_s : 0.0;
  std::map<std::string, double> phase;
  for (int s = 0; s < n; ++s) {
    const CheckReport& r = *main[s];
    if (s > 0 && !r.steps.empty()) {
      BayStep t;
      t.what = "move the tube to stage " + stages[s].name;
      t.part = "bay";
      t.from = "stage " + stages[s - 1].name;
      t.to = "stage " + stages[s].name;
      t.time_s = transfer;
      t.kind = "transfer";
      t.phase = "transfer";
      t.stage = stages[s].name;
      report.steps.push_back(t);
      phase["transfer"] += transfer;
    }
    report.steps.insert(report.steps.end(), r.steps.begin(), r.steps.end());
    report.points.insert(report.points.end(), r.points.begin(), r.points.end());
    for (const auto& [name, t] : r.phases) phase[name] += t;
    for (const PostureChanges& p : r.postures) {
      auto it = std::find_if(report.postures.begin(), report.postures.end(),
                             [&](const PostureChanges& q) { return q.arm == p.arm; });
      PostureChanges q = p;
      if (staged) {
        q.from = Prefix(stages[s].name) + p.from;
        q.to = Prefix(stages[s].name) + p.to;
      }
      if (it == report.postures.end()) {
        report.postures.push_back(q);
      } else {
        const int above = it->above_2s + p.above_2s;
        if (q.largest_s > it->largest_s) *it = q;
        it->above_2s = above;
      }
    }
    if (report.error.empty()) report.error = r.error;
  }
  for (size_t i = 0; i < report.steps.size(); ++i) {
    report.steps[i].number = static_cast<int>(i) + 1;
  }
  if (!main.empty()) report.tool_changes = main[0]->tool_changes;
  const std::vector<std::string>& kinds = WorkKinds();
  std::stable_sort(report.points.begin(), report.points.end(),
                   [&](const SolvedPoint& a, const SolvedPoint& b) {
                     const auto ka = std::find(kinds.begin(), kinds.end(), a.kind);
                     const auto kb = std::find(kinds.begin(), kinds.end(), b.kind);
                     return ka != kb ? ka < kb : a.index < b.index;
                   });
  for (const char* name : {"tube in", "clamp", "rails", "modules", "end", "transfer"}) {
    if (phase.count(name)) report.phases.push_back({name, phase[name]});
  }
  report.bay_time_s = 0.0;
  for (const CheckRow& r : report.rows) {
    if (r.check == "time" && r.item == "bay") report.bay_time_s = r.figure;
  }
  report.pass = !report.rows.empty();
  for (const CheckRow& r : report.rows) report.pass = report.pass && r.pass;
  return report;
}

}  // namespace cell::layout
