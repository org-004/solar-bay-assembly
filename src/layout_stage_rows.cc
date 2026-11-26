// The ways rows with the units measured (src/layout_stage_rows.h).

#include "src/layout_stage_rows.h"

#include <set>

#include "src/layout_sweep.h"

namespace cell::layout {
namespace {

void AddGeoms(const LayoutScene& scene, const std::vector<std::string>& items,
              std::set<int>* by_design, std::vector<Obstacle>* obstacles) {
  const LayoutHandles& H = scene.handles();
  for (const std::string& name : items) {
    auto it = H.item_geoms.find(name);
    if (it == H.item_geoms.end()) continue;
    for (int g : it->second) {
      if (by_design != nullptr) by_design->insert(g);
      bool there = false;
      for (const Obstacle& o : *obstacles) there = there || o.geom == g;
      if (!there) obstacles->push_back({g, 0u, 0u});
    }
  }
}

// The grips (boxes named "jaw..." and "head") of the item that the stage's
// angle_pin_by names, if it is an item.
std::set<int> GripGeoms(const LayoutScene& scene, const Layout& layout,
                        const Stage& stage) {
  std::set<int> out;
  if (stage.angle_pin_by.rfind("item:", 0) != 0) return out;
  const std::string name = stage.angle_pin_by.substr(5);
  const auto it = scene.handles().item_geoms.find(name);
  if (it == scene.handles().item_geoms.end()) return out;
  const std::string prefix = "item_" + name + "_";
  for (int g : it->second) {
    const char* raw = mj_id2name(&scene.model(), mjOBJ_GEOM, g);
    const std::string full = raw != nullptr ? raw : "";
    if (full.rfind(prefix, 0) != 0) continue;
    const std::string box = full.substr(prefix.size());
    if (box.rfind("jaw", 0) == 0 || box == "head") out.insert(g);
  }
  (void)layout;
  return out;
}

SweepSpec Base(const LayoutScene& scene, const std::string& item,
               double clearance_m, double step_m) {
  const LayoutHandles& H = scene.handles();
  SweepSpec s;
  s.item = item;
  s.holder.insert(H.holder_geoms.begin(), H.holder_geoms.end());
  s.tube.insert(H.tube_geoms.begin(), H.tube_geoms.end());
  s.clearance_m = clearance_m;
  s.step_m = step_m;
  return s;
}

}  // namespace

CheckRow WayRow(const LayoutScene& scene, mjData* d, const std::string& item,
                const Layout& layout, WayKind way,
                const std::vector<Vec3>& path, bool units, double clearance_m,
                double step_m) {
  SweepSpec s = Base(scene, item, clearance_m, step_m);
  s.moving = scene.WayMoving(way);
  s.obstacles = scene.WayObstacles(way);
  s.path = path;
  if (units) {
    AddGeoms(scene, scene.WayCarriers(way), &s.by_design, &s.obstacles);
    // The bay out: the grips of the unit that pins the angle at the last
    // stage draw the bay out with them (layout_sweep.h, travels).
    const std::vector<Stage> stages = StagesOf(layout);
    if (way == WayKind::kBayOut && !stages.empty()) {
      s.travels = GripGeoms(scene, layout, stages.back());
      for (int g : s.travels) s.by_design.erase(g);
    }
  }
  return SweepRow(scene, d, s);
}

CheckRow TransferRow(const LayoutScene& scene, mjData* d,
                     const StageScope& stage, const Built& end,
                     double clearance_m, double step_m) {
  const LayoutHandles& H = scene.handles();
  SweepSpec s = Base(scene, "stage " + stage.name + " to " + stage.next,
                     clearance_m, step_m);
  // The tube in a group of its own (kHeld), the rails and modules built on
  // it in another (kCarriage), so that the holder's geoms the tube slides in
  // can be left out against the tube alone.
  for (int g : H.tube_geoms) s.moving.push_back({g, kHeld});
  for (int k : end.rails) {
    if (k < static_cast<int>(H.rail_geoms.size())) {
      for (int g : H.rail_geoms[k]) s.moving.push_back({g, kCarriage});
    }
  }
  for (int k : end.modules) {
    if (k < static_cast<int>(H.module_geoms.size())) {
      for (int g : H.module_geoms[k]) s.moving.push_back({g, kCarriage});
    }
  }
  s.obstacles = scene.WayObstacles(WayKind::kTubeIn);
  s.path = {Vec3{0.0, 0.0, 0.0}, Vec3{0.0, stage.to_next_m, 0.0}};
  s.slide = true;
  AddGeoms(scene, scene.WayCarriers(WayKind::kTubeIn), &s.by_design,
           &s.obstacles);
  AddGeoms(scene, scene.WayCarriers(WayKind::kBayOut), &s.by_design,
           &s.obstacles);
  return SweepRow(scene, d, s);
}

CheckRow TubeAtRestRow(const LayoutScene& scene, mjData* d,
                       const Layout& stage_layout, const StageScope& stage,
                       const Built& end, double clearance_m, double step_m) {
  const LayoutHandles& H = scene.handles();
  SweepSpec s = Base(scene, "stage " + stage.name + ": tube at rest",
                     clearance_m, step_m);
  for (int g : H.tube_geoms) s.moving.push_back({g, kHeld});
  for (int k : end.rails) {
    if (k < static_cast<int>(H.rail_geoms.size())) {
      for (int g : H.rail_geoms[k]) s.moving.push_back({g, kCarriage});
    }
  }
  for (int k : end.modules) {
    if (k < static_cast<int>(H.module_geoms.size())) {
      for (int g : H.module_geoms[k]) s.moving.push_back({g, kCarriage});
    }
  }
  // Every item's every box; the grips of the unit that pins the tube here
  // are near it by design: only below zero (touching) counts, since they
  // hold the tube from beside it; never the unit's bed.
  for (const Stage& st : StagesOf(stage_layout)) {
    if (st.name == stage.name) {
      s.by_design = GripGeoms(scene, stage_layout, st);
    }
  }
  for (const Item& item : stage_layout.items) {
    auto it = H.item_geoms.find(item.name);
    if (it == H.item_geoms.end()) continue;
    for (int g : it->second) s.obstacles.push_back({g, 0u, 0u});
  }
  return SweepRow(scene, d, s);
}

}  // namespace cell::layout
