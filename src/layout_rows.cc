// See src/layout_rows.h. Every row here measures boxes only: the layout's own
// boxes, and the world bounding boxes of geoms of its model at rest.

#include "src/layout_rows.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "src/arm_catalog.h"
#include "src/container.h"
#include "src/flat_rack.h"
#include "src/mj_handles.h"

namespace cell::layout {
namespace {

using Vec = Eigen::Vector3d;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kPortTol = 1e-9;     // m, a zone that touches is not inside.
constexpr double kFrameTol = 0.001;   // m, a frame on the outline.
constexpr double kDeckTol = 0.001;    // m, a box on the deck or touching.
constexpr double kSameZone = 1e-9;    // m, two zones that are one box.
constexpr double kFootHalfX = flat_rack::kWidth / 2.0;
constexpr double kFootHalfY = flat_rack::kLength / 2.0;

std::string Fmt(const char* format, ...) {
  char buffer[4096];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

Vec V(const Vec3& a) { return {a[0], a[1], a[2]}; }

CheckRow Row(const std::string& check, const std::string& item,
             const std::string& arm, double figure, const std::string& unit,
             double limit, bool pass, const std::string& detail) {
  return {check, item, arm, "", figure, unit, limit, pass, detail};
}

std::string Join(const std::vector<std::string>& names) {
  std::string out;
  for (const std::string& n : names) out += (out.empty() ? "" : ", ") + n;
  return out.empty() ? "none" : out;
}

// ------------------------------------------------------------ geoms
std::string GeomName(const mjModel& m, int g) {
  if (g < 0) return "nothing";
  const char* name = mj_id2name(&m, mjOBJ_GEOM, g);
  if (name != nullptr && name[0] != '\0') return name;
  const char* body = mj_id2name(&m, mjOBJ_BODY, m.geom_bodyid[g]);
  return std::string(body != nullptr ? body : "body") + "/geom" +
         std::to_string(g);
}

// The exact extent of a geom in the world.
void ExactExtent(const mjModel& m, const mjData& d, int g, Vec* lo, Vec* hi) {
  Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> r(
      d.geom_xmat + 9 * g);
  const Vec p(d.geom_xpos[3 * g], d.geom_xpos[3 * g + 1],
              d.geom_xpos[3 * g + 2]);
  *lo = Vec::Constant(kInf);
  *hi = -*lo;
  auto add = [&](const Vec& local) {
    const Vec w = p + r * local;
    *lo = lo->cwiseMin(w);
    *hi = hi->cwiseMax(w);
  };
  if (m.geom_type[g] == mjGEOM_MESH && m.geom_dataid[g] >= 0) {
    const int mesh = m.geom_dataid[g];
    const int adr = m.mesh_vertadr[mesh];
    for (int v = 0; v < m.mesh_vertnum[mesh]; ++v) {
      const float* x = m.mesh_vert + 3 * (adr + v);
      add(Vec(x[0], x[1], x[2]));
    }
    return;
  }
  Vec c = Vec::Zero(), h;
  if (m.geom_type[g] == mjGEOM_BOX) {
    h = Vec(m.geom_size[3 * g], m.geom_size[3 * g + 1], m.geom_size[3 * g + 2]);
  } else {
    const mjtNum* a = m.geom_aabb + 6 * g;
    c = Vec(a[0], a[1], a[2]);
    h = Vec(a[3], a[4], a[5]);
  }
  for (int i = 0; i < 8; ++i) {
    add(c + Vec(i & 1 ? h.x() : -h.x(), i & 2 ? h.y() : -h.y(),
                i & 4 ? h.z() : -h.z()));
  }
}

struct Extent {
  Vec lo = Vec::Constant(kInf);
  Vec hi = Vec::Constant(-kInf);
  bool empty() const { return lo.x() > hi.x(); }
  void Add(const Vec& l, const Vec& h) {
    lo = lo.cwiseMin(l);
    hi = hi.cwiseMax(h);
  }
};

// A model at rest, forwarded, with its handles.
struct Scene {
  ModelPtr model;
  DataPtr data;
  LayoutHandles handles;
  std::set<int> base;  // The base's geoms.
};

bool Build(const Layout& layout, const LayoutState& state,
           const ModelPaths& paths, Scene* scene, std::string* error) {
  scene->model = LoadLayoutModel(layout, state, paths, error);
  if (!scene->model) return false;
  scene->data = MakeData(*scene->model);
  if (scene->model->nkey > 0) {
    mj_resetDataKeyframe(scene->model.get(), scene->data.get(), 0);
  }
  mj_forward(scene->model.get(), scene->data.get());
  scene->handles = FindHandles(*scene->model, layout);
  scene->base.insert(scene->handles.base_geoms.begin(),
                     scene->handles.base_geoms.end());
  return true;
}

Extent ExtentOf(const Scene& s, const std::vector<int>& geoms,
                const Vec& move = Vec::Zero()) {
  Extent e;
  for (int g : geoms) {
    if (s.model->geom_type[g] == mjGEOM_PLANE) continue;
    Vec lo, hi;
    ExactExtent(*s.model, *s.data, g, &lo, &hi);
    e.Add(lo + move, hi + move);
  }
  return e;
}

// Every geom that is not the base's and is not a marker of the world.
std::vector<int> CellGeoms(const Scene& s) {
  std::vector<int> out;
  const mjModel& m = *s.model;
  for (int g = 0; g < m.ngeom; ++g) {
    if (s.base.count(g)) continue;
    if (m.geom_type[g] == mjGEOM_PLANE) continue;
    if (m.geom_bodyid[g] == 0 && m.geom_rgba[4 * g + 3] == 0.0) continue;
    out.push_back(g);
  }
  return out;
}

// The geoms of pallet place p: its base and its modules.
std::vector<int> PalletGeoms(const LayoutHandles& h, int p) {
  if (p < 0 || p >= static_cast<int>(h.pallet_place_geoms.size())) return {};
  return h.pallet_place_geoms[p];
}

// ------------------------------------------------------------ boxes
struct BoxLoHi {
  Vec lo, hi;
};

BoxLoHi LayoutBox(const Box& b) {
  return {V(b.pos) - V(b.size) / 2.0, V(b.pos) + V(b.size) / 2.0};
}

BoxLoHi Zone(const Port& p) { return LayoutBox(p.zone); }

const char* kFaces[6] = {"-x", "-y", "-z", "+x", "+y", "+z"};

// How far the extent is inside the zone (least over the six faces) and the
// face that is nearest.
double InsideZone(const Vec& lo, const Vec& hi, const BoxLoHi& z, int* face) {
  double least = kInf;
  for (int a = 0; a < 3; ++a) {
    if (lo[a] - z.lo[a] < least) { least = lo[a] - z.lo[a]; *face = a; }
    if (z.hi[a] - hi[a] < least) { least = z.hi[a] - hi[a]; *face = a + 3; }
  }
  return least;
}

// Separation of two boxes: the largest over the axes of the gap, negative
// when they overlap (then the least overlap depth, negated).
double Separation(const Vec& alo, const Vec& ahi, const Vec& blo,
                  const Vec& bhi) {
  double s = -kInf;
  for (int a = 0; a < 3; ++a) {
    s = std::max(s, std::max(alo[a] - bhi[a], blo[a] - ahi[a]));
  }
  return s;
}

double DeckHeight(const Layout& layout) {
  return layout.base == "container" ? container::kFloorHeight
                                    : flat_rack::kDeckHeight;
}

// Distance of a point from the outline of the footprint in the plane.
double OutlineDistance(double x, double y) {
  const double dx = std::abs(x) - kFootHalfX;
  const double dy = std::abs(y) - kFootHalfY;
  if (dx > 0.0 || dy > 0.0) return std::hypot(std::max(dx, 0.0), std::max(dy, 0.0));
  return -std::max(dx, dy);
}

// The gap in the plane between a zone's rectangle and the footprint.
double FootprintGap(const BoxLoHi& z) {
  const double c_x = (z.lo.x() + z.hi.x()) / 2.0, h_x = (z.hi.x() - z.lo.x()) / 2.0;
  const double c_y = (z.lo.y() + z.hi.y()) / 2.0, h_y = (z.hi.y() - z.lo.y()) / 2.0;
  const double sx = std::abs(c_x) - (kFootHalfX + h_x);
  const double sy = std::abs(c_y) - (kFootHalfY + h_y);
  if (sx > 0.0 || sy > 0.0) return std::hypot(std::max(sx, 0.0), std::max(sy, 0.0));
  return std::max(sx, sy);
}

// ------------------------------------------------------------ allowed
bool StartsWith(const std::string& s, const std::string& prefix) {
  return s.rfind(prefix, 0) == 0;
}

}  // namespace

std::vector<CheckRow> AllowedRows(const Layout& layout) {
  std::vector<CheckRow> rows;
  const std::vector<std::string> movers =
      ArmsDoing(layout, {"module_take", "module_set"});
  const int n_movers = static_cast<int>(movers.size());
  rows.push_back(Row("allowed", "module arms", "", n_movers, "arms", 2,
                     n_movers >= 1 && n_movers <= 2, Join(movers)));
  for (const std::string& name : movers) {
    const ArmUse* arm = FindArm(layout, name);
    const ArmModel* c = arm != nullptr ? FindArmModel(arm->model) : nullptr;
    if (c == nullptr) {
      rows.push_back(Row("allowed", "module arm " + name, name, 0.0, "kg", 210,
                         false, "the arm's model is not in the catalogue"));
      continue;
    }
    const bool ok = c->key == "kr210_r2700_2" ||
                    (StartsWith(c->maker, "KUKA") && c->payload_kg <= 210.0 &&
                     c->reach_m <= 2.701 + 1e-9);
    rows.push_back(Row("allowed", "module arm " + name, name, c->payload_kg,
                       "kg", 210, ok,
                       Fmt("%s, reach %.3f m", c->maker.c_str(), c->reach_m)));
  }
  const std::vector<std::string> pinners =
      ArmsDoing(layout, {"locate", "pin", "swage"});
  const int n_pinners = static_cast<int>(pinners.size());
  rows.push_back(Row("allowed", "pin arms", "", n_pinners, "arms", 3,
                     n_pinners >= 1 && n_pinners <= 3, Join(pinners)));
  const int n_arms = static_cast<int>(layout.arms.size());
  std::vector<std::string> all;
  for (const ArmUse& a : layout.arms) all.push_back(a.name);
  rows.push_back(Row("allowed", "arms", "", n_arms, "arms", 5, n_arms <= 5,
                     Join(all)));
  for (size_t p = 0; p < layout.pallets.size(); ++p) {
    const Pallet& pl = layout.pallets[p];
    rows.push_back(Row("allowed", "pallet " + std::to_string(p), "",
                       pl.modules, "modules", 36,
                       pl.modules == 36 && pl.stands == "on_long_edge",
                       "stands " + pl.stands));
  }
  return rows;
}

// ------------------------------------------------------------ ports
std::vector<CheckRow> PortRows(const Layout& layout, const ModelPaths& paths) {
  std::vector<CheckRow> rows;
  const std::string kind = "port";
  Scene work, stowed;
  std::string error;
  LayoutState everything;
  everything.everything = true;
  LayoutState stow;
  stow.stowed = true;
  if (!Build(layout, everything, paths, &work, &error) ||
      !Build(layout, stow, paths, &stowed, &error)) {
    return {Row(kind, "model", "", 0.0, "", 0.0, false, error)};
  }
  const std::vector<Port>& ports = layout.ports;
  const double deck = DeckHeight(layout);

  int n_tubes = 0, n_bays = 0;
  for (const Port& p : ports) {
    n_tubes += p.crosses == "tubes";
    n_bays += p.crosses == "bays";
  }
  rows.push_back(Row(kind, "tubes and bays are two ports", "", n_tubes + n_bays,
                     "ports", 2, n_tubes >= 1 && n_bays >= 1,
                     Fmt("%d for tubes, %d for bays", n_tubes, n_bays)));

  for (const Port& p : ports) {
    const double out = OutlineDistance(p.frame_pos[0], p.frame_pos[1]);
    const double up = std::abs(p.frame_pos[2] - deck);
    const double f = std::max(out, up);
    rows.push_back(Row(kind, "frame " + p.name + " on the outline", "", f, "m",
                       kFrameTol, f <= kFrameTol,
                       Fmt("%.4f m from the outline in the plane, %.4f m "
                           "from the deck's height %.3f",
                           out, up, deck)));
  }
  for (const Port& p : ports) {
    const double gap = FootprintGap(Zone(p));
    rows.push_back(Row(kind, "zone " + p.name + " off the base", "", gap, "m",
                       0.0, gap >= -kPortTol,
                       gap >= 0.0 ? "clear of the footprint"
                                  : "reaches over the footprint"));
  }
  for (size_t i = 0; i < ports.size(); ++i) {
    for (size_t j = i + 1; j < ports.size(); ++j) {
      const Port& a = ports[i];
      const Port& b = ports[j];
      const BoxLoHi za = Zone(a), zb = Zone(b);
      const double gap = Separation(za.lo, za.hi, zb.lo, zb.hi);
      const bool a_names = a.shares_zone_with == b.name;
      const bool b_names = b.shares_zone_with == a.name;
      bool pass = gap >= -kPortTol;
      std::string detail = pass ? "apart" : "overlap";
      if (a_names && b_names) {
        const bool kinds = (a.crosses == "tubes" && b.crosses == "bays") ||
                           (a.crosses == "bays" && b.crosses == "tubes");
        const bool same = (za.lo - zb.lo).cwiseAbs().maxCoeff() <= kSameZone &&
                          (za.hi - zb.hi).cwiseAbs().maxCoeff() <= kSameZone;
        pass = kinds && same;
        detail = pass ? "shared: one unit serves both"
                      : std::string("shared, but ") +
                            (kinds ? "the zones are not the same box"
                                   : "not one for tubes and one for bays");
      } else if (a_names || b_names) {
        pass = false;
        detail = Fmt("%s names %s as sharing its zone, which does not say so "
                     "in return",
                     (a_names ? a : b).name.c_str(),
                     (a_names ? b : a).name.c_str());
      }
      rows.push_back(Row(kind, "zones " + a.name + " and " + b.name + " apart",
                         "", gap, "m", 0.0, pass, detail));
    }
  }

  // Everything that does not travel stands in the zone of one port.
  struct Thing {
    std::string label, key;
    Extent box;
    std::vector<Vec> points;  // Must also be inside.
  };
  std::vector<Thing> things;
  const LayoutHandles& h = work.handles;
  for (size_t p = 0; p < layout.pallets.size(); ++p) {
    if (!layout.pallets[p].outside) continue;
    things.push_back({"pallet " + std::to_string(p),
                      "pallet:" + std::to_string(p),
                      ExtentOf(work, PalletGeoms(h, static_cast<int>(p))), {}});
  }
  for (const Stock& s : layout.stocks) {
    if (!s.outside) continue;
    auto it = h.stock_geoms.find(s.name);
    Thing t{"stock " + s.name, "stock:" + s.name,
            it != h.stock_geoms.end() ? ExtentOf(work, it->second) : Extent{},
            {V(s.pick_pos)}};
    things.push_back(t);
  }
  for (const Item& i : layout.items) {
    if (i.travels_inside) continue;
    auto it = h.item_geoms.find(i.name);
    things.push_back({"item " + i.name, "item:" + i.name,
                      it != h.item_geoms.end() ? ExtentOf(work, it->second)
                                               : Extent{},
                      {}});
  }
  {
    const Vec first = layout.tube_in.path.empty() ? Vec::Zero()
                                                  : V(layout.tube_in.path.front());
    things.push_back({"way tube_in", "way:tube_in",
                      ExtentOf(work, h.tube_geoms, first), {}});
    const Vec last = layout.bay_out.path.empty() ? Vec::Zero()
                                                 : V(layout.bay_out.path.back());
    std::vector<int> bay = h.tube_geoms;
    for (const auto& r : h.rail_geoms) bay.insert(bay.end(), r.begin(), r.end());
    for (const auto& m : h.module_geoms) bay.insert(bay.end(), m.begin(), m.end());
    things.push_back({"way bay_out", "way:bay_out", ExtentOf(work, bay, last), {}});
  }
  for (const Thing& t : things) {
    std::vector<const Port*> naming;
    for (const Port& p : ports) {
      if (std::find(p.hands_over.begin(), p.hands_over.end(), t.key) !=
          p.hands_over.end()) {
        naming.push_back(&p);
      }
    }
    const std::string item = t.label + " in a zone";
    if (naming.size() != 1) {
      std::vector<std::string> names;
      for (const Port* p : naming) names.push_back(p->name);
      rows.push_back(Row(kind, item, "", -1.0, "m", 0.0, false,
                         naming.empty()
                             ? "no port names " + t.key + " in hands_over"
                             : "several ports name " + t.key + ": " +
                                   Join(names)));
      continue;
    }
    const Port& p = *naming[0];
    const BoxLoHi z = Zone(p);
    if (t.box.empty()) {
      rows.push_back(Row(kind, item, "", -1.0, "m", 0.0, false,
                         "the model has no geom for " + t.key));
      continue;
    }
    int face = 0;
    double f = InsideZone(t.box.lo, t.box.hi, z, &face);
    std::string what = "box";
    for (const Vec& pt : t.points) {
      int pf = 0;
      const double pfig = InsideZone(pt, pt, z, &pf);
      if (pfig < f) { f = pfig; face = pf; what = "pick point"; }
    }
    rows.push_back(Row(kind, item, "", f, "m", 0.0, f >= -kPortTol,
                       Fmt("port %s, nearest face %s of the %s", p.name.c_str(),
                           kFaces[face], what.c_str())));
  }

  // In the stowed state nothing of the cell is in a zone.
  const std::vector<int> cell = CellGeoms(stowed);
  for (const Port& p : ports) {
    const BoxLoHi z = Zone(p);
    double least = kInf;
    int nearest = -1;
    for (int g : cell) {
      Vec lo, hi;
      ExactExtent(*stowed.model, *stowed.data, g, &lo, &hi);
      const double s = Separation(lo, hi, z.lo, z.hi);
      if (s < least) { least = s; nearest = g; }
    }
    if (nearest < 0) {
      rows.push_back(Row(kind, "stowed clear of zone " + p.name, "", 0.0, "m",
                         0.0, true, "no geom in the stowed model"));
      continue;
    }
    rows.push_back(Row(kind, "stowed clear of zone " + p.name, "", least, "m",
                       0.0, least >= -kPortTol,
                       "nearest " + GeomName(*stowed.model, nearest)));
  }
  return rows;
}

// ------------------------------------------------------------ transport
namespace {

// The geoms of the stowed model against an envelope: for each of x, y and
// the top (and the bottom), the worst distance inside.
struct Worst {
  double figure = kInf;
  int geom = -1;
  void Take(double f, int g) {
    if (f < figure) { figure = f; geom = g; }
  }
};

// The `no limit` row: what stands outside, worst first, up to five.
CheckRow NoLimitRow(const std::string& item, const Scene& s,
                    const std::vector<std::pair<double, int>>& per_geom) {
  std::vector<std::pair<double, int>> out;
  double worst = kInf;
  for (const auto& [f, g] : per_geom) {
    worst = std::min(worst, f);
    if (f < -1e-6) out.push_back({f, g});
  }
  std::sort(out.begin(), out.end());
  std::string detail = "no limit: ";
  if (out.empty()) {
    detail += "everything inside";
  } else {
    detail += Fmt("%zu outside: ", out.size());
    for (size_t i = 0; i < out.size() && i < 5; ++i) {
      detail += (i ? ", " : "") + GeomName(*s.model, out[i].second) +
                Fmt(" %.3f m", out[i].first);
    }
  }
  return Row("transport", item, "", worst == kInf ? 0.0 : worst, "m", 0.0, true,
             detail);
}

// Whether every box of a thing is carried by the deck, itself or through
// what it touches. Returns the number not carried and the first of them.
int NotCarried(const std::vector<Box>& boxes, double deck, std::string* first) {
  const size_t n = boxes.size();
  std::vector<BoxLoHi> b;
  for (const Box& x : boxes) b.push_back(LayoutBox(x));
  std::vector<bool> carried(n, false);
  std::vector<size_t> queue;
  for (size_t i = 0; i < n; ++i) {
    if (std::abs(b[i].lo.z() - deck) <= kDeckTol) {
      carried[i] = true;
      queue.push_back(i);
    }
  }
  while (!queue.empty()) {
    const size_t i = queue.back();
    queue.pop_back();
    for (size_t j = 0; j < n; ++j) {
      if (carried[j]) continue;
      if (Separation(b[i].lo, b[i].hi, b[j].lo, b[j].hi) <= kDeckTol) {
        carried[j] = true;
        queue.push_back(j);
      }
    }
  }
  int missing = 0;
  for (size_t i = 0; i < n; ++i) {
    if (!carried[i]) {
      if (missing++ == 0) *first = boxes[i].name;
    }
  }
  return missing;
}

}  // namespace

std::vector<CheckRow> FlatRackTransportRows(const Layout& layout,
                                            const ModelPaths& paths) {
  std::vector<CheckRow> rows;
  const std::string kind = "transport";
  const double deck = flat_rack::kDeckHeight;
  Scene s;
  std::string error;
  LayoutState stow;
  stow.stowed = true;
  const flat_rack::Box env = flat_rack::CellEnvelope();
  const Vec elo = env.centre - env.half_size;
  const Vec ehi = env.centre + env.half_size;
  if (!Build(layout, stow, paths, &s, &error)) {
    for (const char* item : {"stowed x", "stowed y", "stowed top",
                             "stowed bottom"}) {
      rows.push_back(Row(kind, item, "", 0.0, "m", 0.0, false,
                         "no stowed model: " + error));
    }
  } else {
    Worst wx, wy, wtop, wbot;
    for (int g : CellGeoms(s)) {
      Vec lo, hi;
      ExactExtent(*s.model, *s.data, g, &lo, &hi);
      wx.Take(std::min(ehi.x() - hi.x(), lo.x() - elo.x()), g);
      wy.Take(std::min(ehi.y() - hi.y(), lo.y() - elo.y()), g);
      wtop.Take(ehi.z() - hi.z(), g);
      wbot.Take(lo.z() - elo.z(), g);
    }
    auto row = [&](const char* item, const Worst& w, const std::string& what) {
      const double f = w.geom < 0 ? 0.0 : w.figure;
      rows.push_back(Row(kind, item, "", f, "m", 0.0, f >= -1e-6,
                         Fmt("worst %s, %s", GeomName(*s.model, w.geom).c_str(),
                             what.c_str())));
    };
    row("stowed x", wx, Fmt("inside +/-%.3f", ehi.x()));
    row("stowed y", wy, Fmt("inside +/-%.3f", ehi.y()));
    row("stowed top", wtop, Fmt("under %.3f", ehi.z()));
    row("stowed bottom", wbot, Fmt("above %.3f", elo.z()));
  }
  for (const ArmUse& a : layout.arms) {
    if (!a.track) continue;
    const double in = std::min(a.stow_track - a.track->lo, a.track->hi - a.stow_track);
    rows.push_back(Row(kind, "stow track " + a.name, a.name, in, "m", 0.0,
                       in >= -1e-9,
                       Fmt("stow_track %.3f on the track's %.3f..%.3f",
                           a.stow_track, a.track->lo, a.track->hi)));
  }

  // What travels and does not move stands on the deck.
  auto on_deck = [&](const std::string& thing, const std::vector<Box>& boxes) {
    if (boxes.empty()) {
      rows.push_back(Row(kind, "on the deck " + thing, "", 0.0, "boxes", 0.0,
                         true, "no box"));
      return;
    }
    std::string first;
    const int missing = NotCarried(boxes, deck, &first);
    rows.push_back(Row(kind, "on the deck " + thing, "", missing, "boxes", 0.0,
                       missing == 0,
                       missing == 0
                           ? Fmt("all %zu stand on the deck", boxes.size())
                           : Fmt("%d of %zu not carried, first %s", missing,
                                 boxes.size(), first.c_str())));
  };
  if (layout.holder.travels_inside) on_deck("holder", layout.holder.boxes);
  for (const ArmUse& a : layout.arms) {
    if (a.track) on_deck("beam " + a.name, {a.track->beam});
  }
  for (const Stock& st : layout.stocks) {
    if (!st.outside) on_deck("stock " + st.name, st.boxes);
  }
  for (const Item& it : layout.items) {
    if (it.travels_inside) on_deck("item " + it.name, it.boxes);
  }

  // Mass.
  double arms = 0.0, tracks = 0.0, tools = 0.0, holder = 0.0, items = 0.0,
         stocks = 0.0;
  for (const ArmUse& a : layout.arms) {
    const ArmModel* c = FindArmModel(a.model);
    if (c != nullptr) arms += c->mass_kg;
    if (a.track) tracks += a.track->mass_kg;
    for (const Tool& t : a.tools) tools += t.mass_kg;
  }
  if (layout.holder.travels_inside) holder = layout.holder.mass_kg;
  for (const Item& i : layout.items) {
    if (i.travels_inside) items += i.mass_kg;
  }
  for (const Stock& st : layout.stocks) {
    if (!st.outside) stocks += st.mass_kg;
  }
  const double total = arms + tracks + tools + holder + items + stocks;
  rows.push_back(Row(kind, "mass", "", total, "kg", flat_rack::kCellMassBudget,
                     total <= flat_rack::kCellMassBudget,
                     Fmt("arms %.0f, tracks %.0f, tools %.1f, holder %.0f, "
                         "items inside %.0f, stocks inside %.0f",
                         arms, tracks, tools, holder, items, stocks)));

  // What a container that closes would give, as rows without a limit.
  if (s.model) {
    const flat_rack::Box closed = flat_rack::ClosedInsideEnvelope();
    const Vec clo = closed.centre - closed.half_size;
    const Vec chi = closed.centre + closed.half_size;
    const double half_between = flat_rack::kBetweenEndFrames / 2.0;
    std::vector<std::pair<double, int>> fx, fy, ftop, fbetween;
    for (int g : CellGeoms(s)) {
      Vec lo, hi;
      ExactExtent(*s.model, *s.data, g, &lo, &hi);
      fx.push_back({std::min(chi.x() - hi.x(), lo.x() - clo.x()), g});
      fy.push_back({std::min(chi.y() - hi.y(), lo.y() - clo.y()), g});
      ftop.push_back({chi.z() - hi.z(), g});
      fbetween.push_back({std::min(half_between - hi.y(), lo.y() + half_between), g});
    }
    rows.push_back(NoLimitRow("closed container x", s, fx));
    rows.push_back(NoLimitRow("closed container y", s, fy));
    rows.push_back(NoLimitRow("closed container top", s, ftop));
    rows.push_back(NoLimitRow("between end frames y", s, fbetween));
  } else {
    for (const char* item : {"closed container x", "closed container y",
                             "closed container top", "between end frames y"}) {
      rows.push_back(Row(kind, item, "", 0.0, "m", 0.0, true,
                         "no limit: no stowed model"));
    }
  }
  return rows;
}

}  // namespace cell::layout
