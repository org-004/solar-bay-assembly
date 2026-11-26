// See src/layout_bays.h: bays planned back to back.

#include "src/layout_bays.h"

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <memory>
#include <set>
#include <utility>

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

const BayEvent* FindEvent(const std::vector<BayEvent>& events,
                          const std::string& kind) {
  for (const BayEvent& e : events) {
    if (e.kind == kind) return &e;
  }
  return nullptr;
}

const Port* PortHanding(const Layout& layout, const std::string& what) {
  for (const Port& p : layout.ports) {
    if (std::find(p.hands_over.begin(), p.hands_over.end(), what) !=
        p.hands_over.end()) {
      return &p;
    }
  }
  return nullptr;
}

Eigen::Vector3d At(const std::vector<Vec3>& path, size_t i) {
  return {path[i][0], path[i][1], path[i][2]};
}

}  // namespace

// ------------------------------------------------------------ numbering

int ModulesPerBay(const CheckReport& report) {
  int n = 0;
  for (const BayStep& s : report.steps) n += s.kind == "module_take" ? 1 : 0;
  return n;
}

int StepNumberInBay(int number, int bay, int steps_per_bay) {
  return number > 0 ? number + bay * steps_per_bay : number;
}

int BayOfStep(int number, int steps_per_bay) {
  if (number <= 0 || steps_per_bay <= 0) return 0;
  return (number - 1) / steps_per_bay;
}

int StepOfBay(int number, int steps_per_bay) {
  if (number <= 0 || steps_per_bay <= 0) return number;
  return (number - 1) % steps_per_bay + 1;
}

int PointIndexInBay(const BayStep& step, int bay, int modules_per_bay) {
  return step.kind == "module_take" ? step.index + bay * modules_per_bay
                                    : step.index;
}

std::vector<StepInBay> StepsOfBays(const CheckReport& report, int bays) {
  const int n = static_cast<int>(report.steps.size());
  const int per_bay = ModulesPerBay(report);
  std::vector<StepInBay> out;
  out.reserve(static_cast<size_t>(std::max(0, bays)) * report.steps.size());
  for (int b = 0; b < bays; ++b) {
    for (const BayStep& s : report.steps) {
      StepInBay x{b, s};
      x.step.number = StepNumberInBay(s.number, b, n);
      x.step.index = PointIndexInBay(s, b, per_bay);
      out.push_back(std::move(x));
    }
  }
  return out;
}

Built BuiltInBay(Built bay0, int bay, int modules_per_bay) {
  bay0.on_pallets = std::max(0, bay0.on_pallets - bay * modules_per_bay);
  return bay0;
}

// ---------------------------------------------------------------- the ways

Eigen::Vector3d WayOffsetAt(const std::vector<Vec3>& path, double u) {
  if (path.empty()) return Eigen::Vector3d::Zero();
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) total += (At(path, i) - At(path, i - 1)).norm();
  u = std::clamp(u, 0.0, 1.0);
  if (total <= 0.0) return At(path, path.size() - 1);
  double left = u * total;
  for (size_t i = 1; i < path.size(); ++i) {
    const double piece = (At(path, i) - At(path, i - 1)).norm();
    if (left <= piece && piece > 0.0) {
      return At(path, i - 1) + (At(path, i) - At(path, i - 1)) * (left / piece);
    }
    left -= piece;
  }
  return At(path, path.size() - 1);
}

double WayFraction(double t0, double span, double t) {
  return span > 0.0 ? (t - t0) / span : 1.0;
}

bool WaysShareZone(const Layout& layout) {
  const Port* tubes = PortHanding(layout, "way:tube_in");
  const Port* bays = PortHanding(layout, "way:bay_out");
  if (tubes == nullptr || bays == nullptr || tubes == bays) return true;
  if (tubes->shares_zone_with == bays->name ||
      bays->shares_zone_with == tubes->name) {
    return true;
  }
  for (const std::string& h : tubes->hands_over) {
    if (h.rfind("item:", 0) == 0 &&
        std::find(bays->hands_over.begin(), bays->hands_over.end(), h) !=
            bays->hands_over.end()) {
      return true;
    }
  }
  return false;
}

double HolderEmptyAt(const Layout& layout, const BayEvent& bay_out) {
  const std::vector<Vec3>& path = layout.bay_out.path;
  if (path.size() < 2) return bay_out.t1;
  double total = 0.0;
  for (size_t i = 1; i < path.size(); ++i) total += (At(path, i) - At(path, i - 1)).norm();
  if (total <= 0.0) return bay_out.t1;
  const double lift = (At(path, 1) - At(path, 0)).norm();
  return bay_out.t0 + (bay_out.t1 - bay_out.t0) * lift / total;
}

WayPassing TubePassesBay(const LayoutScene& scene, mjData* d,
                         const BayEvent& bay_out, double tube_start,
                         double sample_s, double clearance_m) {
  WayPassing out;
  if (!(sample_s > 0.0)) return out;
  const Layout& L = scene.layout();
  const LayoutHandles& H = scene.handles();
  const std::vector<MovingGeom> tube = scene.WayMoving(WayKind::kTubeIn);
  std::vector<int> bay;
  for (const auto& r : H.rail_geoms) bay.insert(bay.end(), r.begin(), r.end());
  for (const auto& m : H.module_geoms) bay.insert(bay.end(), m.begin(), m.end());
  std::vector<Obstacle> obstacles;
  obstacles.reserve(bay.size());
  for (int g : bay) obstacles.push_back({g, 0u, 0u});
  // Every geom that moves here, with where it is in the data.
  std::vector<std::pair<int, Eigen::Vector3d>> placed;
  for (const MovingGeom& g : tube) {
    placed.emplace_back(g.geom, Eigen::Vector3d(d->geom_xpos + 3 * g.geom));
  }
  for (int g : bay) placed.emplace_back(g, Eigen::Vector3d(d->geom_xpos + 3 * g));
  const size_t tube_count = tube.size();
  auto put = [&](const Eigen::Vector3d& tube_off, const Eigen::Vector3d& bay_off) {
    for (size_t i = 0; i < placed.size(); ++i) {
      const Eigen::Vector3d p = placed[i].second + (i < tube_count ? tube_off : bay_off);
      for (int c = 0; c < 3; ++c) d->geom_xpos[3 * placed[i].first + c] = p[c];
    }
  };
  const double bay_span = bay_out.t1 - bay_out.t0;
  for (int k = 0;; ++k) {
    const double t = tube_start + k * sample_s;
    if (t >= bay_out.t1) break;
    put(WayOffsetAt(L.tube_in.path, WayFraction(tube_start, L.tube_in.time_s, t)),
        WayOffsetAt(L.bay_out.path, WayFraction(bay_out.t0, bay_span, t)));
    const Nearest n = scene.LeastDistance(d, tube, obstacles, clearance_m);
    if (n.distance < clearance_m) {
      out.clear = false;
      out.t = t;
      out.nearest = n;
      break;
    }
  }
  put(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  return out;
}

double NextTubeStart(const LayoutScene& scene, const BayEvent& bay_out,
                     double sample_s, double clearance_m) {
  if (WaysShareZone(scene.layout()) || !(sample_s > 0.0)) return bay_out.t1;
  const double empty = HolderEmptyAt(scene.layout(), bay_out);
  DataPtr d = scene.MakeData();
  for (int k = 0;; ++k) {
    const double s = empty + k * sample_s;
    if (s >= bay_out.t1) return bay_out.t1;
    if (TubePassesBay(scene, d.get(), bay_out, s, sample_s, clearance_m).clear) return s;
  }
}

// ------------------------------------------------------------ joining

Plan JoinBays(const std::vector<Plan>& bays, int steps_per_bay,
              const NextTubeStartFn& next_start) {
  if (bays.size() <= 1) return bays.empty() ? Plan{} : bays.front();
  Plan out = bays.front();
  out.bays = static_cast<int>(bays.size());
  out.events.clear();
  out.steps.clear();
  out.phases.clear();
  for (ArmPlan& a : out.arms) a.segments.clear();
  BayEvent last_out;
  for (size_t bi = 0; bi < bays.size(); ++bi) {
    const int b = static_cast<int>(bi);
    const Plan& p = bays[bi];
    out.complete = out.complete && p.complete;
    // The tube's and the clamp's offset, and the arms' (with release and
    // bay_out): no arm starts before the previous bay has gone out.
    double start = 0.0, shift = 0.0;
    if (b > 0) {
      start = next_start(last_out);
      const BayEvent* clamp = FindEvent(p.events, "clamp");
      const double clamp_end = start + (clamp != nullptr ? clamp->t1 : 0.0);
      shift = start + std::max(0.0, last_out.t1 - clamp_end);
    }
    std::set<int> early;  // The steps of tube_in and clamp.
    for (BayEvent e : p.events) {
      const bool first = e.kind == "tube_in" || e.kind == "clamp";
      if (first) early.insert(e.step);
      const double by = first ? start : shift;
      e.t0 += by;
      e.t1 += by;
      e.step = StepNumberInBay(e.step, b, steps_per_bay);
      e.bay = b;
      if (e.kind == "bay_out") last_out = e;
      out.events.push_back(e);
    }
    for (StepTime s : p.steps) {
      const double by = s.arm.empty() && early.count(s.number) ? start : shift;
      s.start += by;
      s.end += by;
      s.number = StepNumberInBay(s.number, b, steps_per_bay);
      s.bay = b;
      out.steps.push_back(s);
    }
    for (PhaseTime ph : p.phases) {
      ph.bay = b;
      out.phases.push_back(ph);
    }
    for (size_t a = 0; a < out.arms.size() && a < p.arms.size(); ++a) {
      std::vector<Segment>& row = out.arms[a].segments;
      for (size_t k = 0; k < p.arms[a].segments.size(); ++k) {
        Segment g = p.arms[a].segments[k];
        g.t0 += shift;
        g.t1 += shift;
        g.step = StepNumberInBay(g.step, b, steps_per_bay);
        if (k == 0 && !row.empty()) {
          const double end = row.back().t1;
          if (g.t0 < end && g.kind == "wait") {
            g.t0 = end;
          } else if (g.t0 > end) {
            if (row.back().kind == "wait") {
              row.back().t1 = g.t0;
            } else {
              Segment w;
              w.kind = "wait";
              w.step = g.step;
              w.t0 = end;
              w.t1 = g.t0;
              w.tool = row.back().tool;
              w.held = row.back().held;
              w.path = {row.back().path.back()};
              w.reason = "for the next bay";
              row.push_back(w);
            }
          }
        }
        row.push_back(std::move(g));
      }
    }
  }
  return out;
}

CycleFigures CycleFiguresOf(const Plan& plan) {
  CycleFigures c;
  const BayEvent* first_in = nullptr;
  std::vector<const BayEvent*> outs;  // Each bay's bay_out, by bay.
  for (const BayEvent& e : plan.events) {
    if (e.kind == "tube_in" && e.bay == 0 && first_in == nullptr) first_in = &e;
    if (e.kind != "bay_out" || e.bay < 0) continue;
    if (outs.size() <= static_cast<size_t>(e.bay)) outs.resize(e.bay + 1, nullptr);
    outs[e.bay] = &e;
  }
  if (first_in == nullptr || outs.empty() || outs.front() == nullptr) return c;
  for (const BayEvent* e : outs) {
    if (e == nullptr) return c;
  }
  c.bays = static_cast<int>(outs.size());
  c.one_bay_s = outs.front()->t1 - first_in->t0;
  c.cycle_s = (outs.back()->t1 - first_in->t0) / c.bays;
  c.interval_s = c.bays > 1 ? outs.back()->t1 - outs[outs.size() - 2]->t1 : 0.0;
  return c;
}

// ------------------------------------------------------------ planning

PlanResult PlanBays(const Layout& layout, const ModelPaths& paths,
                    const PlanOptions& options, const PlanOneBayFn& plan_bay) {
  const CheckReport report = CheckLayout(layout, paths);
  if (!report.error.empty() || !report.pass) return plan_bay(0, report);
  const int n = static_cast<int>(report.steps.size());
  PlanResult out;
  std::vector<Plan> plans;
  for (int b = 0; b < std::max(1, options.bays); ++b) {
    PlanResult r = plan_bay(b, report);
    for (MoveRow row : r.rows) {
      row.step = StepNumberInBay(row.step, b, n);
      out.rows.push_back(std::move(row));
    }
    out.queries += r.queries;
    for (const std::string& f : r.faults) out.faults.push_back(Fmt("bay %d: %s", b, f.c_str()));
    if (!r.plan.complete) {
      out.plan = std::move(r.plan);
      return out;
    }
    plans.push_back(std::move(r.plan));
  }
  std::string error;
  const std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(layout, paths, report.joint_margin_deg, &error);
  if (!scene) {
    out.faults.push_back("the scene could not be loaded: " + error);
    out.plan = plans.front();
    out.plan.complete = false;
    return out;
  }
  const PlanFigures& f = plans.front().figures;
  out.plan = JoinBays(plans, n, [&](const BayEvent& bay_out) {
    return NextTubeStart(*scene, bay_out, f.sample_s, f.clearance_m);
  });
  return out;
}

}  // namespace cell::layout
