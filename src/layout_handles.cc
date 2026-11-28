// Where to find everything in a model made by src/layout_model.cc
// (LayoutHandles, FindHandles), and the bay's frame in the layout
// (BayPoint). Moved here from src/layout_model.cc unchanged (run 074), so
// that the builder has room; the names it looks for are the builder's.

#include <string>
#include <vector>

#include "src/arm_catalog.h"
#include "src/bay_model.h"
#include "src/layout_model.h"

namespace cell::layout {
namespace {

// As src/layout_model.cc names the base's body and the pallets.
bool OnFlatRack(const Layout& layout) { return layout.base == "flat_rack"; }
bool OnTwoRacks(const Layout& layout) { return layout.base == "two_flat_racks"; }
std::string BaseBody(const Layout& layout) {
  if (OnTwoRacks(layout)) return "two_flat_racks_frame";
  return OnFlatRack(layout) ? "flat_rack_frame" : "container_frame";
}
std::string PalletPrefix(size_t p) {
  return p == 0 ? std::string("pallet") : "pallet" + std::to_string(p);
}

}  // namespace

LayoutHandles FindHandles(const mjModel& model, const Layout& layout) {
  LayoutHandles h;
  const mjModel* m = &model;
  auto body_id = [&](const std::string& name) {
    return mj_name2id(m, mjOBJ_BODY, name.c_str());
  };
  auto geom_id = [&](const std::string& name) {
    return mj_name2id(m, mjOBJ_GEOM, name.c_str());
  };
  // Whether body b is `root` or below it.
  auto under = [&](int b, int root) {
    if (root < 0) return false;
    while (b > 0) {
      if (b == root) return true;
      b = m->body_parentid[b];
    }
    return b == root;
  };
  auto geoms_under = [&](int root) {
    std::vector<int> out;
    if (root < 0) return out;
    for (int g = 0; g < m->ngeom; ++g) {
      if (under(m->geom_bodyid[g], root)) out.push_back(g);
    }
    return out;
  };
  auto geoms_of_body = [&](int body) {
    std::vector<int> out;
    if (body < 0) return out;
    for (int g = 0; g < m->ngeom; ++g) {
      if (m->geom_bodyid[g] == body) out.push_back(g);
    }
    return out;
  };
  auto push_if = [](std::vector<int>* v, int id) {
    if (id >= 0) v->push_back(id);
  };

  for (const ArmUse& arm : layout.arms) {
    ArmHandle a;
    a.name = arm.name;
    const ArmModel* catalog = FindArmModel(arm.model);
    const std::string p = arm.name + "_";
    if (catalog != nullptr) {
      for (int i = 0; i < 6; ++i) {
        const int j = mj_name2id(m, mjOBJ_JOINT, (p + catalog->joint_names[i]).c_str());
        a.qpos[i] = j >= 0 ? m->jnt_qposadr[j] : -1;
        a.dof[i] = j >= 0 ? m->jnt_dofadr[j] : -1;
      }
    }
    const int track = mj_name2id(m, mjOBJ_JOINT, (arm.name + "_track").c_str());
    if (track >= 0) {
      a.track_qpos = m->jnt_qposadr[track];
      a.track_dof = m->jnt_dofadr[track];
    }
    a.flange_site = mj_name2id(m, mjOBJ_SITE, (p + "flange_site").c_str());
    for (const Tool& tool : arm.tools) {
      const int site = mj_name2id(m, mjOBJ_SITE, (p + "work_" + tool.name).c_str());
      if (site >= 0) a.work_site[tool.name] = site;
      std::vector<int> on;
      for (const Box& b : tool.boxes) push_if(&on, geom_id(p + "tool_" + tool.name + "_" + b.name));
      if (!on.empty()) a.tool_geoms[tool.name] = on;
      const int wait = geom_id(p + "wait_" + tool.name);
      if (wait >= 0) a.waiting_tool_geoms[tool.name] = {wait};
    }
    for (const char* part : {"module", "rail"}) {
      std::vector<int> held = geoms_under(body_id(p + "held_" + part));
      if (!held.empty()) a.held_geoms[part] = held;
    }
    if (catalog != nullptr) {
      const int root = body_id(p + catalog->root_body);
      const int mount = body_id(p + "tool_mount");
      for (int g : geoms_under(root)) {
        if (under(m->geom_bodyid[g], mount)) continue;
        if (m->geom_contype[g] == 0 && m->geom_conaffinity[g] == 0) continue;
        a.link_geoms.push_back(g);
      }
    }
    push_if(&a.carriage_geoms, geom_id(arm.name + "_carriage"));
    push_if(&a.beam_geoms, geom_id(arm.name + "_beam"));
    h.arms.push_back(a);
  }

  h.base_body = BaseBody(layout);
  h.base_geoms = geoms_of_body(body_id(h.base_body));
  h.holder_geoms = geoms_of_body(body_id("holder"));
  h.tube_geoms = geoms_of_body(body_id(bay::TubeName()));
  for (int k = 0; k < bay::kRailCount; ++k) {
    const int b = body_id(bay::RailName(k));
    if (b < 0) break;
    h.rail_geoms.push_back(geoms_of_body(b));
  }
  for (int i = 0; i < bay::kModuleCount; ++i) {
    const int b = body_id(bay::ModuleName(i));
    if (b < 0) break;
    h.module_geoms.push_back(geoms_of_body(b));
  }
  // The pallets, their modules indexed as the points of module_take: [j]
  // runs through the pallets in order, empty when the module is gone.
  {
    std::vector<std::vector<int>> modules;
    std::vector<std::vector<int>> places;
    bool any_module = false, any_place = false;
    for (size_t pi = 0; pi < layout.pallets.size(); ++pi) {
      const std::string prefix = PalletPrefix(pi);
      std::vector<int> place;
      const int base = geom_id(prefix + "_base");
      push_if(&h.pallet_base_geoms, base);
      push_if(&place, base);
      any_place = any_place || base >= 0;
      for (int j = 0; j < layout.pallets[pi].modules; ++j) {
        std::vector<int> g =
            geoms_of_body(body_id(prefix + "_module_" + std::to_string(j)));
        any_module = any_module || !g.empty();
        place.insert(place.end(), g.begin(), g.end());
        modules.push_back(std::move(g));
      }
      places.push_back(std::move(place));
    }
    if (any_module) h.pallet_module_geoms = std::move(modules);
    if (any_place) h.pallet_place_geoms = std::move(places);
  }
  for (const Stock& s : layout.stocks) {
    std::vector<int> g = geoms_of_body(body_id("stock_" + s.name));
    if (!g.empty()) h.stock_geoms[s.name] = g;
  }
  for (const Item& item : layout.items) {
    std::vector<int> g = geoms_of_body(body_id("item_" + item.name));
    if (!g.empty()) h.item_geoms[item.name] = g;
  }
  const int probe = body_id("probe");
  if (probe >= 0) {
    h.probe_mocap = m->body_mocapid[probe];
    h.probe_bolt_geom = geom_id("probe_bolt");
    h.probe_pin_geom = geom_id("probe_pin");
    h.probe_locate_below_geom = geom_id("probe_locate_below");
    h.probe_locate_above_geom = geom_id("probe_locate_above");
  }
  return h;
}

Eigen::Vector3d BayPoint(const Layout& layout, const Eigen::Vector3d& in_bay) {
  return Eigen::Vector3d(layout.tube.axis_x, layout.tube.plain_end_y,
                         layout.tube.axis_z) +
         in_bay;
}

}  // namespace cell::layout
