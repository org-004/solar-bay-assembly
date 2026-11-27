// What moves, what is there and what touches or is near by design, in a
// layout's model. See src/layout_scene.h.

#include "src/layout_scene.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <set>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "src/bay_hardware.h"
#include "src/layout_check.h"

namespace cell::layout {
namespace {

using Vec = Eigen::Vector3d;

constexpr double kDeg = std::numbers::pi / 180.0;

struct Aabb {
  Vec c = Vec::Zero();
  Vec h = Vec::Zero();
  bool plane = false;
};

// Not inlined: LeastDistance and PairQuery run the very same code, so their
// boxes and gaps agree to the bit.
[[gnu::noinline]] Aabb WorldAabb(const mjModel& m, const mjData& d, int g) {
  Aabb box;
  if (m.geom_type[g] == mjGEOM_PLANE) {
    box.plane = true;
    return box;
  }
  const mjtNum* a = m.geom_aabb + 6 * g;
  Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> r(
      d.geom_xmat + 9 * g);
  const Vec p(d.geom_xpos[3 * g], d.geom_xpos[3 * g + 1],
              d.geom_xpos[3 * g + 2]);
  box.c = p + r * Vec(a[0], a[1], a[2]);
  box.h = r.cwiseAbs() * Vec(a[3], a[4], a[5]);
  return box;
}

[[gnu::noinline]] double AabbGap(const Aabb& a, const Aabb& b) {
  if (a.plane || b.plane) return 0.0;
  const Vec gap = ((a.c - b.c).cwiseAbs() - a.h - b.h).cwiseMax(0.0);
  return gap.norm();
}

std::vector<int> Range(int n) {
  std::vector<int> v;
  for (int i = 0; i < n; ++i) v.push_back(i);
  return v;
}

void Insert(std::set<int>* s, const std::vector<int>& g) {
  s->insert(g.begin(), g.end());
}

}  // namespace

std::string HeldAtWork(const std::string& kind) {
  if (kind == "rail_take" || kind == "rail_set") return "rail";
  if (kind == "module_take" || kind == "module_set") return "module";
  if (kind == "pins_take") return "pin magazine";
  return "";
}

std::unique_ptr<LayoutScene> LayoutScene::Load(const Layout& layout,
                                               const ModelPaths& paths,
                                               double joint_margin_deg,
                                               std::string* error) {
  std::unique_ptr<LayoutScene> scene(new LayoutScene());
  scene->layout_ = layout;
  std::string e;
  if (!scene->Init(paths, joint_margin_deg, &e)) {
    if (error != nullptr) *error = e;
    return nullptr;
  }
  return scene;
}

bool LayoutScene::Init(const ModelPaths& paths, double joint_margin_deg,
                       std::string* error) {
  LayoutState everything;
  everything.everything = true;
  model_ = LoadLayoutModel(layout_, everything, paths, error);
  if (!model_) return false;
  handles_ = FindHandles(*model_, layout_);
  if (handles_.arms.size() != layout_.arms.size()) {
    *error = "the model has " + std::to_string(handles_.arms.size()) +
             " arms, the layout " + std::to_string(layout_.arms.size());
    return false;
  }
  for (size_t i = 0; i < layout_.arms.size(); ++i) {
    SceneArm a;
    a.use = &layout_.arms[i];
    a.cat = FindArmModel(a.use->model);
    a.h = &handles_.arms[i];
    if (a.cat == nullptr) {
      *error = "arm " + a.use->name + ": model " + a.use->model +
               " is not in the catalogue";
      return false;
    }
    for (int j = 0; j < 6; ++j) {
      a.dlo[j] = a.cat->joint_lo_deg[j] * kDeg;
      a.dhi[j] = a.cat->joint_hi_deg[j] * kDeg;
      a.lo[j] = a.dlo[j] + joint_margin_deg * kDeg;
      a.hi[j] = a.dhi[j] - joint_margin_deg * kDeg;
      int jnt = -1;
      for (int k = 0; k < model_->njnt; ++k) {
        if (model_->jnt_qposadr[k] == a.h->qpos[j]) {
          jnt = k;
          break;
        }
      }
      if (jnt >= 0 && model_->jnt_limited[jnt]) {
        a.lo[j] = std::max(a.lo[j], model_->jnt_range[2 * jnt]);
        a.hi[j] = std::min(a.hi[j], model_->jnt_range[2 * jnt + 1]);
      }
    }
    for (int j = 0; j < 6; ++j) a.wait[j] = a.use->wait_joints_deg[j] * kDeg;
    a.wait_track = a.use->wait_track;
    arms_.push_back(a);
  }
  return true;
}

int LayoutScene::ArmIndex(const std::string& name) const {
  for (size_t i = 0; i < arms_.size(); ++i) {
    if (arms_[i].use->name == name) return static_cast<int>(i);
  }
  return -1;
}

DataPtr LayoutScene::MakeData() const {
  DataPtr d = cell::MakeData(*model_);
  if (model_->nkey > 0) mj_resetDataKeyframe(model_.get(), d.get(), 0);
  AllAtWait(d.get());
  mj_kinematics(model_.get(), d.get());
  return d;
}

void LayoutScene::SetPose(mjData* d, int arm, const ArmPose& pose) const {
  const SceneArm& a = arms_[arm];
  for (int j = 0; j < 6; ++j) d->qpos[a.h->qpos[j]] = pose.q[j];
  if (a.h->track_qpos >= 0) d->qpos[a.h->track_qpos] = pose.track;
}

void LayoutScene::AllAtWait(mjData* d) const {
  for (int i = 0; i < ArmCount(); ++i) {
    SetPose(d, i, {arms_[i].wait, arms_[i].wait_track});
  }
}

const std::vector<int>& LayoutScene::BaseGeoms() const {
  return handles_.base_geoms;
}

// True if the geom is fixed to the arm's carriage: no joint between its body
// and the track's slide.
bool LayoutScene::RidesCarriage(const SceneArm& a, int g) const {
  const mjModel& m = *model_;
  if (a.h->track_qpos < 0) return false;
  for (int b = m.geom_bodyid[g]; b > 0; b = m.body_parentid[b]) {
    for (int k = 0; k < m.body_jntnum[b]; ++k) {
      const int j = m.body_jntadr[b] + k;
      return m.jnt_qposadr[j] == a.h->track_qpos;
    }
  }
  return false;
}

std::vector<MovingGeom> LayoutScene::Moving(int arm, const std::string& tool,
                                            const std::string& held) const {
  std::vector<MovingGeom> out;
  const SceneArm& a = arms_[arm];
  // Link geoms fixed to the carriage (the base) count as the carriage.
  for (int g : a.h->link_geoms) {
    out.push_back({g, RidesCarriage(a, g) ? kCarriage : kLinks});
  }
  for (int g : a.h->carriage_geoms) out.push_back({g, kCarriage});
  auto t = a.h->tool_geoms.find(tool);
  if (t != a.h->tool_geoms.end()) {
    for (size_t i = 0; i < t->second.size(); ++i) {
      out.push_back({t->second[i],
                     i + 1 == t->second.size() ? kToolLast : kToolBody});
    }
  }
  if (!held.empty()) {
    auto h = a.h->held_geoms.find(held);
    if (h != a.h->held_geoms.end()) {
      for (int g : h->second) out.push_back({g, kHeld});
    }
  }
  return out;
}

std::vector<int> LayoutScene::LinksForHeld(int arm) const {
  const mjModel& m = *model_;
  const SceneArm& a = arms_[arm];
  std::set<int> wrist;  // The flange's body, up to and with the last joint.
  if (a.h->flange_site >= 0) {
    for (int b = m.site_bodyid[a.h->flange_site]; b > 0;
         b = m.body_parentid[b]) {
      wrist.insert(b);
      if (m.body_jntnum[b] > 0) break;
    }
  }
  std::vector<int> out;
  for (int g : a.h->link_geoms) {
    if (!wrist.count(m.geom_bodyid[g])) out.push_back(g);
  }
  return out;
}

std::vector<Obstacle> LayoutScene::Obstacles(
    int active, const std::string& tool, const Built& b,
    const DesignPairs& pairs, const ObstacleOptions& options) const {
  const LayoutHandles& H = handles_;
  std::vector<Obstacle> out;
  constexpr unsigned kTool = kToolBody | kToolLast;
  auto add = [&](const std::vector<int>& geoms, unsigned skip = 0) {
    for (int g : geoms) {
      Obstacle o{g, skip, 0u};
      if (pairs.held_skip.count(g)) o.skip |= kHeld;
      if (pairs.last_near.count(g)) {
        o.skip |= kToolLast;
        o.near |= kToolLast;
      }
      if (pairs.tool_near.count(g)) {
        o.skip |= kTool;
        o.near |= kTool;
      }
      if (pairs.held_near.count(g) && !pairs.held_skip.count(g)) {
        o.skip |= kHeld;
        o.near |= kHeld;
      }
      out.push_back(o);
    }
  };
  add(H.base_geoms);
  add(H.holder_geoms);
  add(H.tube_geoms);
  for (int k : b.rails) {
    if (k >= 0 && k < static_cast<int>(H.rail_geoms.size())) {
      add(H.rail_geoms[k]);
    }
  }
  for (int k : b.modules) {
    if (k >= 0 && k < static_cast<int>(H.module_geoms.size())) {
      add(H.module_geoms[k]);
    }
  }
  add(H.pallet_base_geoms);
  // pallet_module_geoms[j] is the j-th module taken.
  const int pallet_n = static_cast<int>(H.pallet_module_geoms.size());
  for (int k = std::max(0, pallet_n - b.on_pallets); k < pallet_n; ++k) {
    add(H.pallet_module_geoms[k]);
  }
  for (const auto& s : H.stock_geoms) add(s.second);
  for (const auto& s : H.item_geoms) add(s.second);
  for (int i = 0; i < ArmCount(); ++i) {
    const SceneArm& a = arms_[i];
    const bool me = i == active;
    // The arm's carriage runs on its own beam and on a beam it shares.
    const bool rides = me || BeamArm(layout_, i) == BeamArm(layout_, active);
    add(a.h->beam_geoms, rides ? kCarriage : 0u);
    std::string on = tool;
    if (!me) {
      auto o = options.tools.find(i);
      on = o != options.tools.end() ? o->second : a.use->first_tool;
    }
    if (!me && options.other_arms) {
      add(a.h->link_geoms);
      add(a.h->carriage_geoms);
      auto t = a.h->tool_geoms.find(on);
      if (t != a.h->tool_geoms.end()) add(t->second);
    }
    // The arm's own waiting tools ride on its carriage.
    for (const auto& w : a.h->waiting_tool_geoms) {
      if (w.first != on) add(w.second, me ? kCarriage : 0u);
    }
    // The part it holds against its own links and carriage.
    if (me) {
      add(LinksForHeld(i), ~static_cast<unsigned>(kHeld));
      add(a.h->carriage_geoms, ~static_cast<unsigned>(kHeld));
    }
  }
  return out;
}

StepScene LayoutScene::AtWorkPoint(const WorkPoint& p, Leg leg) const {
  StepScene s;
  const LayoutHandles& H = handles_;
  const std::string& kind = p.kind;
  const Work* w = FindWork(layout_, kind, p.index);
  if (w == nullptr) {
    s.error = "the layout gives no work " + kind;
    return s;
  }
  s.arm = ArmIndex(w->arm);
  s.tool = w->tool;
  if (s.arm < 0) {
    s.error = "no arm " + w->arm;
    return s;
  }
  const SceneArm& a = arms_[s.arm];
  if (FindTool(*a.use, w->tool) == nullptr ||
      a.h->work_site.find(w->tool) == a.h->work_site.end()) {
    s.error = "arm " + w->arm + " has no tool " + w->tool;
    return s;
  }
  s.held = HeldAtWork(kind);
  const int R = bay::kRailCount, M = bay::kModuleCount;
  const int pallet_total = PointCount(layout_, "module_take");
  static const std::vector<int> none;
  auto rail = [&](int k) -> const std::vector<int>& {
    return k >= 0 && k < static_cast<int>(H.rail_geoms.size()) ? H.rail_geoms[k]
                                                               : none;
  };
  auto module = [&](int k) -> const std::vector<int>& {
    return k >= 0 && k < static_cast<int>(H.module_geoms.size())
               ? H.module_geoms[k]
               : none;
  };
  auto pallet_module = [&](int j) -> const std::vector<int>& {
    return j >= 0 && j < static_cast<int>(H.pallet_module_geoms.size())
               ? H.pallet_module_geoms[j]
               : none;
  };
  // The modules in the order they are set (src/layout.h, WorkOrder): the
  // first k of them, sorted, are on the bay once k have been set.
  const std::vector<int> order = ModuleOrder(layout_);
  const std::vector<int> pos = ModulePositions(layout_);
  auto set_before = [&](int k) {
    k = std::clamp(k, 0, M);
    std::vector<int> v(order.begin(), order.begin() + k);
    std::sort(v.begin(), v.end());
    return v;
  };
  // Whether module a is set before module b.
  auto set_earlier = [&](int a, int b) {
    return a >= 0 && a < M && b >= 0 && b < M && pos[a] < pos[b];
  };
  DesignPairs& d = s.pairs;
  Built b;
  b.on_pallets = pallet_total;
  if (kind == "rail_take" || kind == "rail_set") {
    b.rails = Range(p.index);
    if (kind == "rail_set") Insert(&d.held_skip, H.tube_geoms);
    if (kind == "rail_take") {
      auto st = H.stock_geoms.find("rails");
      if (st != H.stock_geoms.end()) Insert(&d.held_skip, st->second);
    }
  } else if (kind == "strap_bolt") {
    b.rails = layout_.rail_order == "each_rail_then_its_bolt"
                  ? Range(p.index + 1)
                  : Range(R);
    Insert(&d.last_near, rail(p.index));
    // The bolt joins the strap's ears 18 mm under the tube, so the socket
    // on it is nearer the tube than the clearance by the bay's design.
    Insert(&d.last_near, H.tube_geoms);
  } else if (kind == "module_take") {
    b.rails = Range(R);
    b.modules = set_before(std::min(p.index, M));
    b.on_pallets = pallet_total - p.index - 1;  // Module j is the one held.
    // It touches module j+1 behind (or below) it on its pallet, and the
    // deck it stands on (flat: only the last of its pallet).
    const int pi = std::max(0, p.pallet);
    const Pallet* pallet =
        pi < static_cast<int>(layout_.pallets.size()) ? &layout_.pallets[pi]
                                                      : nullptr;
    const int n = p.pallet_modules;  // On its pallet, j included.
    if (n > 1) Insert(&d.held_skip, pallet_module(p.index + 1));
    if (pallet == nullptr || pallet->stands != "flat" || n == 1) {
      if (H.pallet_base_geoms.size() == layout_.pallets.size()) {
        Insert(&d.held_skip, {H.pallet_base_geoms[pi]});
      } else {
        Insert(&d.held_skip, H.pallet_base_geoms);
      }
    }
  } else if (kind == "module_set") {
    b.rails = Range(R);
    const int k = p.index >= 0 && p.index < M ? pos[p.index] : p.index;
    b.modules = set_before(k);
    b.on_pallets = pallet_total - k - 1;
    Insert(&d.held_skip, rail(p.index));
    Insert(&d.held_skip, rail(p.index + 1));
    Insert(&d.held_near, module(p.index - 1));
    // Set out of order, a module may have its neighbour on either side.
    if (set_earlier(p.index + 1, p.index)) Insert(&d.held_near, module(p.index + 1));
  } else if (kind == "locate" || kind == "pin" || kind == "swage") {
    const int k = p.module >= 0 && p.module < M ? pos[p.module] : p.module;
    b.rails = Range(R);
    b.modules = set_before(k + 1);
    b.on_pallets = pallet_total - k - 1;
    // The nose goes into the holes of the rail and the module it joins.
    Insert(&d.last_near, rail(p.rail));
    Insert(&d.last_near, module(p.module));
    s.built.push_back(b);
    if (k + 1 < M) {
      // The next module set, and both neighbours, which a module arm that
      // runs ahead may set while these holes are worked.
      std::vector<int> more = set_before(k + 2);
      for (int nb : {p.module - 1, p.module + 1}) {
        if (nb >= 0 && nb < M && std::find(more.begin(), more.end(), nb) == more.end()) {
          more.push_back(nb);
        }
      }
      std::sort(more.begin(), more.end());
      b.modules = more;
    } else {
      return s;
    }
  } else if (kind == "pins_take") {
    b.rails = Range(R);
    const std::vector<const Stock*> stocks = PinStocks(layout_);
    if (p.index >= 0 && p.index < static_cast<int>(stocks.size())) {
      auto st = H.stock_geoms.find(stocks[p.index]->name);
      if (st != H.stock_geoms.end()) Insert(&d.held_skip, st->second);
    }
  }
  s.built.push_back(b);
  if (leg == Leg::kWork) return s;

  const bool take =
      kind == "rail_take" || kind == "module_take" || kind == "pins_take";
  const bool set = kind == "rail_set" || kind == "module_set";
  if (take && leg == Leg::kApproach) {
    // Nothing held yet; the tool against what it is about to take.
    const std::set<int> from = d.held_skip;
    s.held.clear();
    s.pairs = DesignPairs();
    if (kind == "module_take") {
      for (Built& x : s.built) x.on_pallets += 1;
      Insert(&s.pairs.tool_near, pallet_module(p.index));
    } else {
      // A rail or a magazine in its stock has no geoms of its own; the
      // tool is near the stock's boxes.
      s.pairs.tool_near = from;
    }
  } else if (set && leg == Leg::kRetreat) {
    // Let go: the part lies in the bay.
    s.held.clear();
    s.pairs = DesignPairs();
    for (Built& x : s.built) {
      if (kind == "rail_set") {
        x.rails = Range(p.index + 1);
        Insert(&s.pairs.tool_near, rail(p.index));
      } else {
        x.modules = set_before(
            (p.index >= 0 && p.index < M ? pos[p.index] : p.index) + 1);
        Insert(&s.pairs.tool_near, module(p.index));
      }
    }
  }
  return s;
}

std::vector<std::string> LayoutScene::WayCarriers(WayKind way) const {
  std::vector<std::string> out;
  auto lower = [](std::string t) {
    for (char& c : t) c = static_cast<char>(std::tolower(c));
    return t;
  };
  if (layout_.ports.empty()) {
    // Run 011's rule, kept for its record.
    for (const Item& item : layout_.items) {
      const std::string does = lower(item.does);
      if (does.find("tube") != std::string::npos ||
          does.find("bay") != std::string::npos) {
        out.push_back(item.name);
      }
    }
    return out;
  }
  const std::string named = way == WayKind::kTubeIn ? "way:tube_in" : "way:bay_out";
  std::set<std::string> ports;
  for (const Port& p : layout_.ports) {
    if (std::find(p.hands_over.begin(), p.hands_over.end(), named) !=
        p.hands_over.end()) {
      ports.insert(p.name);
    }
  }
  // The ports that share a zone with one of those, either way round.
  std::set<std::string> sharing = ports;
  for (const Port& p : layout_.ports) {
    if (ports.count(p.name) && !p.shares_zone_with.empty()) {
      sharing.insert(p.shares_zone_with);
    }
    if (!p.shares_zone_with.empty() && ports.count(p.shares_zone_with)) {
      sharing.insert(p.name);
    }
  }
  std::set<std::string> carriers;
  for (const Port& p : layout_.ports) {
    if (!sharing.count(p.name)) continue;
    for (const std::string& h : p.hands_over) {
      if (h.rfind("item:", 0) == 0) carriers.insert(h.substr(5));
    }
  }
  for (const Item& item : layout_.items) {
    if (carriers.count(item.name)) out.push_back(item.name);
  }
  return out;
}

std::vector<MovingGeom> LayoutScene::WayMoving(WayKind way) const {
  const LayoutHandles& H = handles_;
  std::vector<int> geoms = H.tube_geoms;
  if (way == WayKind::kBayOut) {
    for (const auto& r : H.rail_geoms) geoms.insert(geoms.end(), r.begin(), r.end());
    for (const auto& r : H.module_geoms) {
      geoms.insert(geoms.end(), r.begin(), r.end());
    }
  }
  std::vector<MovingGeom> out;
  for (int g : geoms) out.push_back({g, kHeld});
  return out;
}

std::vector<Obstacle> LayoutScene::WayObstacles(WayKind way) const {
  const LayoutHandles& H = handles_;
  std::vector<Obstacle> out;
  auto add = [&](const std::vector<int>& g) {
    for (int x : g) out.push_back({x, 0u, 0u});
  };
  add(H.base_geoms);
  add(H.holder_geoms);
  add(H.pallet_base_geoms);
  for (const auto& p : H.pallet_module_geoms) add(p);
  for (const auto& s : H.stock_geoms) add(s.second);
  const std::vector<std::string> carriers = WayCarriers(way);
  for (const Item& item : layout_.items) {
    if (std::find(carriers.begin(), carriers.end(), item.name) !=
        carriers.end()) {
      continue;
    }
    auto it = H.item_geoms.find(item.name);
    if (it != H.item_geoms.end()) add(it->second);
  }
  for (const SceneArm& a : arms_) {
    const std::string& first = a.use->first_tool;
    add(a.h->beam_geoms);
    add(a.h->link_geoms);
    add(a.h->carriage_geoms);
    auto t = a.h->tool_geoms.find(first);
    if (t != a.h->tool_geoms.end()) add(t->second);
    for (const auto& w : a.h->waiting_tool_geoms) {
      if (w.first != first) add(w.second);
    }
  }
  return out;
}

namespace {

struct Pair {
  double gap;
  int i, j;
};

// LeastDistance's measuring, given its pairs (box gap below `far`) and the
// pairs near by design whose boxes touch, in the order they were found.
Nearest Measure(const mjModel& m, mjData* d,
                const std::vector<MovingGeom>& moving,
                const std::vector<Obstacle>& obstacles, std::vector<Pair>& pairs,
                const std::vector<Pair>& near, double far) {
  std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
    if (a.gap != b.gap) return a.gap < b.gap;
    if (a.i != b.i) return a.i < b.i;
    return a.j < b.j;
  });
  Nearest best;
  best.distance = far;
  for (const Pair& p : pairs) {
    if (p.gap >= best.distance) break;
    const int g = moving[p.i].geom;
    const int h = obstacles[p.j].geom;
    const double dist = mj_geomDistance(&m, d, g, h, best.distance, nullptr);
    if (dist < best.distance) {
      best.distance = dist;
      best.moving = g;
      best.obstacle = h;
    }
  }
  // Pairs near by design: only going into each other counts.
  for (const Pair& p : near) {
    const int g = moving[p.i].geom;
    const int h = obstacles[p.j].geom;
    const double dist = mj_geomDistance(&m, d, g, h, 0.0, nullptr);
    if (dist < 0.0 && dist < best.distance) {
      best.distance = dist;
      best.moving = g;
      best.obstacle = h;
      best.near = true;
    }
  }
  return best;
}

}  // namespace

Nearest LayoutScene::LeastDistance(mjData* d,
                                   const std::vector<MovingGeom>& moving,
                                   const std::vector<Obstacle>& obstacles,
                                   double far) const {
  const mjModel& m = *model_;
  std::vector<Aabb> mb(moving.size()), ob(obstacles.size());
  for (size_t i = 0; i < moving.size(); ++i) {
    mb[i] = WorldAabb(m, *d, moving[i].geom);
  }
  for (size_t j = 0; j < obstacles.size(); ++j) {
    ob[j] = WorldAabb(m, *d, obstacles[j].geom);
  }
  std::vector<Pair> pairs, near;
  for (size_t i = 0; i < moving.size(); ++i) {
    for (size_t j = 0; j < obstacles.size(); ++j) {
      if (moving[i].geom == obstacles[j].geom) continue;
      if (obstacles[j].skip & moving[i].group) {
        if ((obstacles[j].near & moving[i].group) &&
            AabbGap(mb[i], ob[j]) <= 0.0) {
          near.push_back({0.0, static_cast<int>(i), static_cast<int>(j)});
        }
        continue;
      }
      const double gap = AabbGap(mb[i], ob[j]);
      if (gap < far) {
        pairs.push_back({gap, static_cast<int>(i), static_cast<int>(j)});
      }
    }
  }
  return Measure(m, d, moving, obstacles, pairs, near, far);
}

std::vector<bool> LayoutScene::MovesWith(int arm) const {
  const mjModel& m = *model_;
  const SceneArm& a = arms_[arm];
  std::set<int> qpos(a.h->qpos.begin(), a.h->qpos.end());
  if (a.h->track_qpos >= 0) qpos.insert(a.h->track_qpos);
  std::vector<bool> moves(m.nbody, false);
  // A parent's id is below its children's.
  for (int b = 1; b < m.nbody; ++b) {
    bool own = false;
    for (int k = 0; k < m.body_jntnum[b]; ++k) {
      own = own || qpos.count(m.jnt_qposadr[m.body_jntadr[b] + k]) > 0;
    }
    moves[b] = own || moves[m.body_parentid[b]];
  }
  return moves;
}

// ------------------------------------------------------------- PairQuery

namespace {

// The margin against rounding when an obstacle is passed over whole: far
// below any distance that matters, far above the rounding of coordinates
// of some metres.
constexpr double kCullSlack = 1e-9;  // m

// The largest separation, over the three axes, between box b and the box
// [lo, hi]; no more than the separation of b from any box inside [lo, hi].
double Separation(const Aabb& b, const Vec& lo, const Vec& hi) {
  double sep = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < 3; ++k) {
    sep = std::max(sep, lo[k] - (b.c[k] + b.h[k]));
    sep = std::max(sep, (b.c[k] - b.h[k]) - hi[k]);
  }
  return sep;
}

}  // namespace

// Obstacles are passed over in runs of this many (consecutive in the list,
// so mostly of one item), by the box around the run, before one by one.
constexpr int kRun = 8;

struct PairQuery::State {
  std::vector<Aabb> mb, ob;
  std::vector<Pair> pairs, near;
  // Per run of kRun obstacles: whether none moves with the arm and none is
  // a plane, and the box around them.
  std::vector<bool> run_fixed;
  std::vector<Vec> run_lo, run_hi;
};

PairQuery::PairQuery(const LayoutScene& scene, int arm,
                     std::vector<MovingGeom> moving,
                     std::vector<Obstacle> obstacles)
    : scene_(scene),
      moving_(std::move(moving)),
      obstacles_(std::move(obstacles)),
      s_(std::make_unique<State>()) {
  const mjModel& m = scene_.model();
  const std::vector<bool> moves = scene_.MovesWith(arm);
  for (int b = 1; b < m.nbody; ++b) {
    if (moves[b]) bodies_.push_back(b);
    if (moves[b] && !Placeable(m, b)) partial_ = false;
  }
  for (int b : bodies_) {
    for (int g = m.body_geomadr[b]; g < m.body_geomadr[b] + m.body_geomnum[b]; ++g) {
      geoms_.push_back(g);
    }
  }
  for (int i = 0; i < m.nsite; ++i) {
    if (moves[m.site_bodyid[i]]) sites_.push_back(i);
  }
  for (const MovingGeom& g : moving_) moving_moves_.push_back(moves[m.geom_bodyid[g.geom]]);
  for (const Obstacle& o : obstacles_) obstacle_moves_.push_back(moves[m.geom_bodyid[o.geom]]);
  const size_t n = obstacles_.size();
  kind_.assign(moving_.size() * n, 0);
  for (size_t i = 0; i < moving_.size(); ++i) {
    for (size_t j = 0; j < n; ++j) {
      if (moving_[i].geom == obstacles_[j].geom) continue;
      if (obstacles_[j].skip & moving_[i].group) {
        if (obstacles_[j].near & moving_[i].group) kind_[i * n + j] = 2;
        continue;
      }
      kind_[i * n + j] = 1;
    }
  }
  s_->mb.resize(moving_.size());
  s_->ob.resize(n);
  const size_t runs = (n + kRun - 1) / kRun;
  s_->run_fixed.assign(runs, true);
  s_->run_lo.assign(runs, Vec::Zero());
  s_->run_hi.assign(runs, Vec::Zero());
  for (size_t j = 0; j < n; ++j) {
    if (obstacle_moves_[j]) s_->run_fixed[j / kRun] = false;
  }
}

PairQuery::~PairQuery() = default;

void PairQuery::Place(mjData* d) {
  const mjModel& m = scene_.model();
  const bool all = !partial_ || d != placed_ ||
                   (m.opt.enableflags & mjENBL_SLEEP) || d->ntree_awake < m.ntree;
  if (all) {
    mj_kinematics(&m, d);
    placed_ = d;
  } else {
    // mj_kinematics: the bodies' pass (mj_kinematics1), then
    // (mj_kinematics2) the inertial frames, geoms and sites, here only of
    // the bodies that moved, parents before children.
    for (int b : bodies_) PlaceBody(m, d, b);
    for (int b : bodies_) {
      PlaceFrame(d, d->xipos + 3 * b, d->xiquat + 4 * b, d->ximat + 9 * b,
                          m.body_ipos + 3 * b, m.body_iquat + 4 * b, b,
                          m.body_sameframe[b]);
    }
    for (int g : geoms_) {
      PlaceFrame(d, d->geom_xpos + 3 * g, d->geom_xquat + 4 * g,
                          d->geom_xmat + 9 * g, m.geom_pos + 3 * g, m.geom_quat + 4 * g,
                          m.geom_bodyid[g], m.geom_sameframe[g]);
    }
    for (int i : sites_) {
      PlaceFrame(d, d->site_xpos + 3 * i, d->site_xquat + 4 * i,
                          d->site_xmat + 9 * i, m.site_pos + 3 * i, m.site_quat + 4 * i,
                          m.site_bodyid[i], m.site_sameframe[i]);
    }
  }
  for (size_t i = 0; i < moving_.size(); ++i) {
    if (all || moving_moves_[i]) s_->mb[i] = WorldAabb(m, *d, moving_[i].geom);
  }
  for (size_t j = 0; j < obstacles_.size(); ++j) {
    if (all || obstacle_moves_[j]) s_->ob[j] = WorldAabb(m, *d, obstacles_[j].geom);
  }
  if (!all) return;
  // The runs' boxes, fixed from here on.
  for (size_t r = 0; r < s_->run_fixed.size(); ++r) {
    Vec lo = Vec::Constant(std::numeric_limits<double>::infinity());
    Vec hi = -lo;
    for (size_t j = r * kRun; j < std::min(obstacles_.size(), (r + 1) * kRun); ++j) {
      const Aabb& b = s_->ob[j];
      if (b.plane) s_->run_fixed[r] = false;
      lo = lo.cwiseMin(b.c - b.h);
      hi = hi.cwiseMax(b.c + b.h);
    }
    s_->run_lo[r] = lo;
    s_->run_hi[r] = hi;
  }
}

void PairQuery::Keep(double far) {
  keep_.clear();
  const std::vector<Aabb>& mb = s_->mb;
  const std::vector<Aabb>& ob = s_->ob;
  bool cull = far > 0.0 && std::isfinite(far) && !mb.empty();
  Vec lo = Vec::Constant(std::numeric_limits<double>::infinity());
  Vec hi = -lo;
  for (const Aabb& b : mb) {
    if (b.plane) cull = false;
    lo = lo.cwiseMin(b.c - b.h);
    hi = hi.cwiseMax(b.c + b.h);
  }
  const double beyond = far + kCullSlack;
  for (size_t r = 0; r < s_->run_fixed.size(); ++r) {
    const size_t end = std::min(ob.size(), (r + 1) * kRun);
    if (cull && s_->run_fixed[r]) {
      double sep = -std::numeric_limits<double>::infinity();
      for (int k = 0; k < 3; ++k) {
        sep = std::max(sep, lo[k] - s_->run_hi[r][k]);
        sep = std::max(sep, s_->run_lo[r][k] - hi[k]);
      }
      if (sep >= beyond) continue;
    }
    for (size_t j = r * kRun; j < end; ++j) {
      if (cull && !ob[j].plane && Separation(ob[j], lo, hi) >= beyond) continue;
      keep_.push_back(static_cast<int>(j));
    }
  }
}

Nearest PairQuery::Least(mjData* d, double far) {
  Keep(far);
  std::vector<Pair>& pairs = s_->pairs;
  std::vector<Pair>& near = s_->near;
  pairs.clear();
  near.clear();
  const size_t n = obstacles_.size();
  for (size_t i = 0; i < moving_.size(); ++i) {
    const unsigned char* kind = kind_.data() + i * n;
    for (int j : keep_) {
      if (kind[j] == 0) continue;
      const double gap = AabbGap(s_->mb[i], s_->ob[j]);
      if (kind[j] == 2) {
        if (gap <= 0.0) near.push_back({0.0, static_cast<int>(i), j});
      } else if (gap < far) {
        pairs.push_back({gap, static_cast<int>(i), j});
      }
    }
  }
  return Measure(scene_.model(), d, moving_, obstacles_, pairs, near, far);
}

bool PairQuery::Closer(mjData* d, double far) {
  const mjModel& m = scene_.model();
  const size_t n = obstacles_.size();
  // One pair: whether it alone makes LeastDistance's answer nearer than far.
  auto shows = [&](int i, int j) {
    const unsigned char kind = kind_[static_cast<size_t>(i) * n + j];
    if (kind == 0) return false;
    const double gap = AabbGap(s_->mb[i], s_->ob[j]);
    const int g = moving_[i].geom;
    const int h = obstacles_[j].geom;
    if (kind == 2) {
      if (gap > 0.0) return false;
      const double dist = mj_geomDistance(&m, d, g, h, 0.0, nullptr);
      return dist < 0.0 && dist < far;
    }
    return gap < far && mj_geomDistance(&m, d, g, h, far, nullptr) < far;
  };
  // The pair that showed it last time first: a move's next configuration is
  // usually stopped by the same pair.
  if (last_i_ >= 0 && shows(last_i_, last_j_)) return true;
  Keep(far);
  for (int pass = 1; pass <= 2; ++pass) {  // Measured pairs first.
    for (size_t i = 0; i < moving_.size(); ++i) {
      const unsigned char* kind = kind_.data() + i * n;
      for (int j : keep_) {
        if (kind[j] != pass) continue;
        if (shows(static_cast<int>(i), j)) {
          last_i_ = static_cast<int>(i);
          last_j_ = j;
          return true;
        }
      }
    }
  }
  return false;
}

std::string LayoutScene::GeomName(int g) const {
  const mjModel& m = *model_;
  if (g < 0) return "nothing";
  const char* name = mj_id2name(&m, mjOBJ_GEOM, g);
  if (name != nullptr && name[0] != '\0') return name;
  const char* body = mj_id2name(&m, mjOBJ_BODY, m.geom_bodyid[g]);
  return std::string(body != nullptr ? body : "body") + "/geom" +
         std::to_string(g);
}

std::string LayoutScene::PairText(const Nearest& n) const {
  if (n.moving < 0) return "nothing within 1 m";
  return GeomName(n.moving) + " to " + GeomName(n.obstacle) +
         (n.near ? " (near by design, but into each other)" : "");
}

}  // namespace cell::layout
