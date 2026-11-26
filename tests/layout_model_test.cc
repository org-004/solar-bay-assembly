// The layouts and the models made from them (src/layout.h,
// src/layout_model.h).
//
// Every file in layouts/ loads, and broken copies are refused with a message
// that names the field: of the old form and of the wider one (the base, the
// flat rack's margins, work divided among arms, ports, approaches). A layout
// moved onto the flat rack as a file loads and builds with the flat rack's
// base, two pallet places and a port's sites, and a layout with three arms of
// one model builds. The chosen layout is the one versions/versions.json
// names (src/layout.h, ChosenLayout), and a layout file may not say it is
// chosen. For the reference layout the model compiles in
// the states that matter, agrees with the file, and nothing in it overlaps:
// every pair of geoms of two different things is further apart than zero by
// mj_geomDistance, except the pairs that touch by design (TouchesByDesign
// below gives each rule with its reason). Contacts are not used: MuJoCo
// never reports one between two bodies fixed to the world.
//
//   bazel test --config=opt //:layout_model_test --test_output=all

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include "src/arm_catalog.h"
#include "src/bay_hardware.h"
#include "src/container.h"
#include "src/flat_rack.h"
#include "src/layout.h"
#include "src/layout_model.h"
#include "src/mj_handles.h"

namespace cell::layout {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string LayoutDir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr ? dir : "layouts";
}

std::string LayoutPath(const std::string& name) {
  return LayoutDir() + "/" + name + ".json";
}

std::string ReadText(const std::string& path) {
  std::ifstream in(path);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

TEST(Layouts, EveryFileLoads) {
  const std::vector<std::string> names = ListLayouts(LayoutDir());
  ASSERT_FALSE(names.empty()) << "no layouts in " << LayoutDir();
  for (const std::string& name : names) {
    std::string error;
    std::optional<Layout> layout = LoadLayout(LayoutPath(name), &error);
    ASSERT_TRUE(layout.has_value()) << error;
    EXPECT_EQ(layout->name, name);
  }
  std::string error;
  const std::optional<std::string> chosen = ChosenLayout(VersionsPath(), &error);
  ASSERT_TRUE(chosen.has_value()) << error;
  EXPECT_NE(std::find(names.begin(), names.end(), *chosen), names.end())
      << "the chosen layout " << *chosen << " is not in " << LayoutDir();
}

// The order of work (src/layout.h, WorkOrder): a layout without "order" has
// the plain one, the ordered layout reads its own, and the ordered layout is
// flat_rack_3_pin_arms but for its name, title, summary and order (spec 018,
// R2: the order of work changes, nothing of the cell does).
TEST(Layouts, OrderOfWork) {
  std::string error;
  const std::optional<Layout> plain = LoadLayout(LayoutPath("flat_rack_3_pin_arms"), &error);
  ASSERT_TRUE(plain.has_value()) << error;
  EXPECT_TRUE(plain->order.modules.empty());
  EXPECT_EQ(plain->order.modules_start, "after_all_bolts");
  EXPECT_EQ(plain->order.pin_tools, "just_in_time");
  EXPECT_EQ(ModuleOrder(*plain), (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8}));
  const std::optional<Layout> ordered =
      LoadLayout(LayoutPath("flat_rack_3_pin_arms_ordered"), &error);
  ASSERT_TRUE(ordered.has_value()) << error;
  const std::vector<int> order = ModuleOrder(*ordered);
  const std::vector<int> pos = ModulePositions(*ordered);
  ASSERT_EQ(order.size(), 9u);
  for (int k = 0; k < 9; ++k) EXPECT_EQ(pos[order[k]], k);
  EXPECT_EQ(ordered->order.modules_start, "after_their_rails");
  EXPECT_EQ(ordered->order.pin_tools, "early");

  nlohmann::json a = nlohmann::json::parse(ReadText(LayoutPath("flat_rack_3_pin_arms")));
  nlohmann::json b = nlohmann::json::parse(ReadText(LayoutPath("flat_rack_3_pin_arms_ordered")));
  for (const char* k : {"name", "title", "summary", "order"}) {
    a.erase(k);
    b.erase(k);
  }
  EXPECT_EQ(a, b) << "the ordered layout differs from flat_rack_3_pin_arms in more than its "
                     "name, title, summary and order";
}

// Writes `j` as <tmp>/<case>/below.json and loads it.
std::string LoadBroken(const nlohmann::json& j, const std::string& name,
                       const std::string& which) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::filesystem::path dir =
      std::filesystem::path(tmp != nullptr ? tmp : "/tmp") / which;
  std::filesystem::create_directories(dir);
  const std::string path = (dir / (name + ".json")).string();
  std::ofstream(path) << j.dump(1);
  std::string error;
  EXPECT_FALSE(LoadLayout(path, &error).has_value()) << which;
  EXPECT_NE(error.find(path), std::string::npos) << error;
  return error;
}

TEST(Layouts, BrokenCopiesAreRefused) {
  const nlohmann::json good = nlohmann::json::parse(ReadText(LayoutPath("below")));
  {
    nlohmann::json j = good;
    j["arms"][0]["colour"] = "orange";
    const std::string e = LoadBroken(j, "below", "unknown_field");
    EXPECT_NE(e.find("arms[0].colour"), std::string::npos) << e;
    EXPECT_NE(e.find("unknown field"), std::string::npos) << e;
  }
  {
    // An order of modules that is not one of each, a word that is not one of
    // the rules, and an order on a layout of the container.
    const nlohmann::json flat =
        nlohmann::json::parse(ReadText(LayoutPath("flat_rack_3_pin_arms_ordered")));
    nlohmann::json j = flat;
    j["order"]["modules"][1] = 0;
    std::string e = LoadBroken(j, "flat_rack_3_pin_arms_ordered", "order_twice");
    EXPECT_NE(e.find("order.modules[1]"), std::string::npos) << e;
    j = flat;
    j["order"]["modules"].erase(8);
    e = LoadBroken(j, "flat_rack_3_pin_arms_ordered", "order_short");
    EXPECT_NE(e.find("order.modules"), std::string::npos) << e;
    j = flat;
    j["order"]["modules_start"] = "soon";
    e = LoadBroken(j, "flat_rack_3_pin_arms_ordered", "order_start");
    EXPECT_NE(e.find("order.modules_start"), std::string::npos) << e;
    j = flat;
    j["order"]["pin_tools"] = "late";
    e = LoadBroken(j, "flat_rack_3_pin_arms_ordered", "order_tools");
    EXPECT_NE(e.find("order.pin_tools"), std::string::npos) << e;
    j = good;
    j["order"] = flat["order"];
    e = LoadBroken(j, "below", "order_on_container");
    EXPECT_NE(e.find("order"), std::string::npos) << e;
    EXPECT_NE(e.find("unknown field"), std::string::npos) << e;
  }
  {
    // Which layout is chosen lives in versions/versions.json, not here.
    nlohmann::json j = good;
    j["chosen"] = true;
    const std::string e = LoadBroken(j, "below", "chosen_field");
    EXPECT_NE(e.find("chosen"), std::string::npos) << e;
    EXPECT_NE(e.find("unknown field"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["arms"][1]["model"] = "ur99";
    const std::string e = LoadBroken(j, "below", "unknown_model");
    EXPECT_NE(e.find("arms[1].model"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["pallets"][0].erase("pitch");
    const std::string e = LoadBroken(j, "below", "missing_field");
    EXPECT_NE(e.find("pallets[0].pitch"), std::string::npos) << e;
    EXPECT_NE(e.find("missing"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["holder"]["mass_kg"] = "heavy";
    const std::string e = LoadBroken(j, "below", "wrong_type");
    EXPECT_NE(e.find("holder.mass_kg"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["arms"][1]["name"] = j["arms"][0]["name"];
    const std::string e = LoadBroken(j, "below", "duplicate_arm");
    EXPECT_NE(e.find("arms[1].name"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["work"][0]["kind"] = "weld";
    const std::string e = LoadBroken(j, "below", "unknown_kind");
    EXPECT_NE(e.find("work[0].kind"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["work"][0]["tool"] = "hammer";
    const std::string e = LoadBroken(j, "below", "unknown_tool");
    EXPECT_NE(e.find("work[0].tool"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["work"][0]["arm"] = "third_arm";
    const std::string e = LoadBroken(j, "below", "unknown_arm");
    EXPECT_NE(e.find("work[0].arm"), std::string::npos) << e;
  }
  {
    const std::string e = LoadBroken(good, "elsewhere", "wrong_name");
    EXPECT_NE(e.find("name"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["arms"][0]["track"] = nullptr;
    std::string error;
    const char* tmp = std::getenv("TEST_TMPDIR");
    const std::filesystem::path dir =
        std::filesystem::path(tmp != nullptr ? tmp : "/tmp") / "null_track";
    std::filesystem::create_directories(dir);
    std::ofstream((dir / "below.json").string()) << j.dump(1);
    std::optional<Layout> l = LoadLayout((dir / "below.json").string(), &error);
    ASSERT_TRUE(l.has_value()) << error;
    EXPECT_FALSE(l->arms[0].track.has_value());
  }
}

// Run 011's chosen layout on the container, which most tests here are
// written against. The layout marked chosen today stands on the flat rack.
Layout Chosen() {
  std::string error;
  std::optional<Layout> layout =
      LoadLayout(LayoutPath("below_rails_by_module_arm"), &error);
  if (layout) return *layout;
  ADD_FAILURE() << "below_rails_by_module_arm does not load: " << error;
  return {};
}

struct Built {
  ModelPtr model;
  DataPtr data;
  LayoutHandles handles;
};

Built Build(const Layout& layout, const LayoutState& state) {
  Built b;
  std::string error;
  b.model = LoadLayoutModel(layout, state, ModelPaths::FromEnvironment(), &error);
  EXPECT_TRUE(b.model) << error;
  if (!b.model) return b;
  b.data = MakeData(*b.model);
  EXPECT_EQ(b.model->nkey, 1);
  mj_resetDataKeyframe(b.model.get(), b.data.get(), 0);
  mj_forward(b.model.get(), b.data.get());
  b.handles = FindHandles(*b.model, layout);
  return b;
}

LayoutState Bay(int rails, int modules) {
  LayoutState s;
  s.rails = rails;
  s.modules = modules;
  return s;
}

Eigen::Vector3d V(const Vec3& v) { return {v[0], v[1], v[2]}; }
Eigen::Vector3d V(const mjtNum* p) { return {p[0], p[1], p[2]}; }
Eigen::Matrix3d M(const mjtNum* p) {
  Eigen::Matrix3d r;
  r << p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8];
  return r;
}

Eigen::Matrix3d Rpy(const Vec3& rpy_deg) {
  const double d = kPi / 180.0;
  return (Eigen::AngleAxisd(rpy_deg[2] * d, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy_deg[1] * d, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy_deg[0] * d, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

std::string GeomName(const mjModel* m, int g) {
  const char* n = mj_id2name(m, mjOBJ_GEOM, g);
  if (n != nullptr) return n;
  const char* b = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[g]);
  return std::string("(geom ") + std::to_string(g) + " of " +
         (b != nullptr ? b : "?") + ")";
}


// The greatest value of dir . p over the points p of a geom: exact for a
// mesh (its vertices) and a box, a bound for other primitives (their aabb).
double Extent(const mjModel* m, const mjData* d, int g, const Eigen::Vector3d& dir) {
  const Eigen::Vector3d pos = V(d->geom_xpos + 3 * g);
  const Eigen::Matrix3d r = M(d->geom_xmat + 9 * g);
  if (m->geom_type[g] == mjGEOM_MESH) {
    const int mesh = m->geom_dataid[g];
    const int first = m->mesh_vertadr[mesh];
    double best = -1e300;
    for (int v = 0; v < m->mesh_vertnum[mesh]; ++v) {
      const float* p = m->mesh_vert + 3 * (first + v);
      best = std::max(best, dir.dot(pos + r * Eigen::Vector3d(p[0], p[1], p[2])));
    }
    return best;
  }
  const double* c = m->geom_aabb + 6 * g;
  const Eigen::Vector3d local = r.transpose() * dir;
  return dir.dot(pos + r * V(c)) + V(c + 3).dot(local.cwiseAbs());
}

TEST(ChosenLayout, CompilesWithABareTubeAndAFinishedBay) {
  const Layout layout = Chosen();
  for (const LayoutState& state : {Bay(0, 0), Bay(10, 9)}) {
    Built b = Build(layout, state);
    ASSERT_TRUE(b.model);
    EXPECT_EQ(b.handles.rail_geoms.size(), static_cast<size_t>(state.rails));
    EXPECT_EQ(b.handles.module_geoms.size(), static_cast<size_t>(state.modules));
    EXPECT_EQ(b.handles.pallet_module_geoms.size(),
              static_cast<size_t>(layout.pallets[0].modules));
    EXPECT_EQ(b.handles.tube_geoms.size(), 2u);
    EXPECT_FALSE(b.handles.base_geoms.empty());
    EXPECT_EQ(b.handles.base_body, "container_frame");
    EXPECT_EQ(b.handles.holder_geoms.size(), layout.holder.boxes.size());
    EXPECT_EQ(b.handles.pallet_base_geoms.size(), 1u);
    EXPECT_EQ(b.handles.stock_geoms.size(), layout.stocks.size());
    EXPECT_EQ(b.handles.item_geoms.size(), layout.items.size());
    EXPECT_EQ(b.handles.probe_mocap, -1);
    ASSERT_EQ(b.handles.arms.size(), layout.arms.size());
    for (size_t i = 0; i < layout.arms.size(); ++i) {
      const ArmHandle& a = b.handles.arms[i];
      const ArmUse& arm = layout.arms[i];
      EXPECT_EQ(a.name, arm.name);
      for (int j = 0; j < 6; ++j) EXPECT_GE(a.qpos[j], 0) << arm.name << " joint " << j;
      EXPECT_GE(a.flange_site, 0);
      EXPECT_FALSE(a.link_geoms.empty());
      EXPECT_EQ(a.work_site.size(), arm.tools.size());
      // In the waiting state the first tool is on the flange and no other.
      EXPECT_EQ(a.tool_geoms.size(), arm.first_tool.empty() ? 0u : 1u);
      if (!arm.first_tool.empty()) {
        EXPECT_EQ(a.tool_geoms.count(arm.first_tool), 1u);
      }
      EXPECT_TRUE(a.held_geoms.empty());
      if (arm.track) {
        EXPECT_GE(a.track_qpos, 0);
        EXPECT_EQ(a.carriage_geoms.size(), 1u);
        EXPECT_EQ(a.beam_geoms.size(), 1u);
      }
    }
  }
}

TEST(ChosenLayout, AgreesWithTheFile) {
  const Layout layout = Chosen();
  Built b = Build(layout, Bay(10, 9));
  ASSERT_TRUE(b.model);
  const mjModel* m = b.model.get();
  const mjData* d = b.data.get();
  const double tol = 1e-9;

  // Each arm's base where the file says, at its wait track value, turned as
  // the file says; its joints at the wait pose.
  for (size_t i = 0; i < layout.arms.size(); ++i) {
    const ArmUse& arm = layout.arms[i];
    const ArmHandle& a = b.handles.arms[i];
    const ArmModel* model = FindArmModel(arm.model);
    const int root = mj_name2id(m, mjOBJ_BODY, (arm.name + "_" + model->root_body).c_str());
    ASSERT_GE(root, 0) << arm.name;
    Eigen::Vector3d want = V(arm.base_pos);
    if (arm.track) {
      want += arm.wait_track * V(arm.track->direction).normalized();
      EXPECT_NEAR(d->qpos[a.track_qpos], arm.wait_track, tol);
    }
    EXPECT_LT((V(d->xpos + 3 * root) - want).norm(), tol) << arm.name;
    const int frame = m->body_parentid[root];
    EXPECT_LT((M(d->xmat + 9 * frame) - Rpy(arm.base_rpy_deg)).norm(), 1e-9) << arm.name;
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(d->qpos[a.qpos[j]], arm.wait_joints_deg[j] * kPi / 180.0, tol);
    }
    // Every tool's working point `length` from the flange along its z.
    const Eigen::Vector3d flange = V(d->site_xpos + 3 * a.flange_site);
    const Eigen::Matrix3d fr = M(d->site_xmat + 9 * a.flange_site);
    for (const Tool& tool : arm.tools) {
      const int s = a.work_site.at(tool.name);
      EXPECT_LT((V(d->site_xpos + 3 * s) - (flange + tool.length * fr.col(2))).norm(), 1e-9)
          << arm.name << " " << tool.name;
      EXPECT_LT((M(d->site_xmat + 9 * s) - fr).norm(), 1e-9);
    }
    // The tool on the flange weighs what the file says.
    const int mount = mj_name2id(m, mjOBJ_BODY, (arm.name + "_tool_mount").c_str());
    ASSERT_GE(mount, 0);
    const Tool* first = FindTool(arm, arm.first_tool);
    EXPECT_NEAR(m->body_mass[mount], first != nullptr ? first->mass_kg : 0.0, 1e-6);
    // Its boxes where the file says, in the flange's frame.
    if (first != nullptr) {
      for (const Box& box : first->boxes) {
        const int g = mj_name2id(m, mjOBJ_GEOM,
                                 (arm.name + "_tool_" + first->name + "_" + box.name).c_str());
        ASSERT_GE(g, 0);
        EXPECT_LT((V(d->geom_xpos + 3 * g) - (flange + fr * V(box.pos))).norm(), 1e-9);
        EXPECT_LT((V(m->geom_size + 3 * g) - V(box.size) / 2.0).norm(), 1e-12);
      }
    }
    // The waiting boxes ride in the base's frame.
    for (const Tool& tool : arm.tools) {
      auto it = a.waiting_tool_geoms.find(tool.name);
      if (tool.name == arm.first_tool) {
        EXPECT_TRUE(it == a.waiting_tool_geoms.end());
        continue;
      }
      if (tool.wait_size[0] <= 0.0) continue;
      ASSERT_TRUE(it != a.waiting_tool_geoms.end()) << tool.name;
      const Eigen::Vector3d at = want + Rpy(arm.base_rpy_deg) * V(tool.waits_at);
      EXPECT_LT((V(d->geom_xpos + 3 * it->second[0]) - at).norm(), 1e-9);
    }
  }

  // The tube's axis.
  const int frame = mj_name2id(m, mjOBJ_BODY, "bay_frame");
  ASSERT_GE(frame, 0);
  EXPECT_LT((V(d->xpos + 3 * frame) -
             Eigen::Vector3d(layout.tube.axis_x, layout.tube.plain_end_y, layout.tube.axis_z))
                .norm(),
            tol);
  const int tube = mj_name2id(m, mjOBJ_GEOM, bay::TubeName().c_str());
  EXPECT_NEAR(d->geom_xpos[3 * tube + 0], layout.tube.axis_x, tol);
  EXPECT_NEAR(d->geom_xpos[3 * tube + 2], layout.tube.axis_z, tol);
  EXPECT_NEAR(std::fabs(d->geom_xmat[9 * tube + 5]), 1.0, tol);  // Its z along Y.
  // Rail 9 and module 8 where the header puts them, in the layout's frame.
  const int rail9 = mj_name2id(m, mjOBJ_BODY, bay::RailName(9).c_str());
  EXPECT_LT((V(d->xpos + 3 * rail9) - BayPoint(layout, bay::RailPose(9).translation())).norm(), tol);
  const int module8 = mj_name2id(m, mjOBJ_BODY, bay::ModuleName(8).c_str());
  EXPECT_LT((V(d->xpos + 3 * module8) - BayPoint(layout, bay::ModulePose(8).translation())).norm(),
            tol);

  // The holder's boxes and mass.
  for (const Box& box : layout.holder.boxes) {
    const int g = mj_name2id(m, mjOBJ_GEOM, ("holder_" + box.name).c_str());
    ASSERT_GE(g, 0) << box.name;
    EXPECT_LT((V(d->geom_xpos + 3 * g) - V(box.pos)).norm(), tol) << box.name;
    EXPECT_LT((V(m->geom_size + 3 * g) - V(box.size) / 2.0).norm(), tol) << box.name;
  }
  const int holder = mj_name2id(m, mjOBJ_BODY, "holder");
  EXPECT_NEAR(m->body_mass[holder], layout.holder.mass_kg, 1e-6);

  // The pallet's place, its deck and its stack.
  const Pallet& p = layout.pallets[0];
  const int deck = b.handles.pallet_base_geoms.at(0);
  EXPECT_LT((V(d->geom_xpos + 3 * deck) - (V(p.pos) + Eigen::Vector3d(0, 0, p.base_height / 2))).norm(),
            tol);
  EXPECT_NEAR(m->geom_size[3 * deck + 2], p.base_height / 2, tol);
  ASSERT_EQ(b.handles.pallet_module_geoms.size(), static_cast<size_t>(p.modules));
  // Each module where src/layout.h, Pallet, puts it, in the pallet's frame.
  const Eigen::Matrix3d yaw = Rpy({0.0, 0.0, p.yaw_deg});
  for (int j = 0; j < p.modules; ++j) {
    ASSERT_FALSE(b.handles.pallet_module_geoms[j].empty()) << j;
    const int body = m->geom_bodyid[b.handles.pallet_module_geoms[j][0]];
    Eigen::Vector3d local;
    Eigen::Vector3d glass_normal;  // The module's own z, in the pallet's frame.
    if (p.stands == "on_long_edge") {
      local = {0.0, (j - (p.modules - 1) / 2.0) * p.pitch, p.base_height + bay::kModuleWidth / 2.0};
      glass_normal = -Eigen::Vector3d::UnitY();
    } else {
      local = {0.0, 0.0, p.base_height + (p.modules - 1 - j) * p.pitch + bay::kModuleThickness / 2.0};
      glass_normal = Eigen::Vector3d::UnitZ();
    }
    EXPECT_LT((V(d->xpos + 3 * body) - (V(p.pos) + yaw * local)).norm(), 1e-9) << j;
    const Eigen::Matrix3d r = yaw.transpose() * M(d->xmat + 9 * body);
    EXPECT_LT((r.col(2) - glass_normal).norm(), 1e-9) << j;
    EXPECT_LT((r.col(0) - Eigen::Vector3d::UnitX()).norm(), 1e-9) << j;  // Long side along x.
  }
  // With n left, the first modules - n are gone.
  {
    LayoutState state = Bay(10, 9);
    state.pallet_modules = 5;
    Built few = Build(layout, state);
    ASSERT_TRUE(few.model);
    ASSERT_EQ(few.handles.pallet_module_geoms.size(), static_cast<size_t>(p.modules));
    for (int j = 0; j < p.modules; ++j) {
      EXPECT_EQ(few.handles.pallet_module_geoms[j].empty(), j < p.modules - 5) << j;
    }
  }

  // Stocks and items where the file says.
  for (const Stock& s : layout.stocks) {
    for (const Box& box : s.boxes) {
      const int g = mj_name2id(m, mjOBJ_GEOM, ("stock_" + s.name + "_" + box.name).c_str());
      ASSERT_GE(g, 0);
      EXPECT_LT((V(d->geom_xpos + 3 * g) - V(box.pos)).norm(), tol);
    }
  }
  for (const Item& item : layout.items) {
    for (const Box& box : item.boxes) {
      const int g = mj_name2id(m, mjOBJ_GEOM, ("item_" + item.name + "_" + box.name).c_str());
      ASSERT_GE(g, 0);
      EXPECT_LT((V(d->geom_xpos + 3 * g) - V(box.pos)).norm(), tol);
    }
  }
}

// Each flange's z points out of the flange: along the last joint's axis
// (either way round) and away from the wrist, the flange's centre lying
// beyond the last joint's anchor along it.
TEST(ChosenLayout, FlangeZPointsOut) {
  const Layout layout = Chosen();
  Built b = Build(layout, Bay(0, 0));
  ASSERT_TRUE(b.model);
  const mjModel* m = b.model.get();
  const mjData* d = b.data.get();
  for (size_t i = 0; i < layout.arms.size(); ++i) {
    const ArmUse& arm = layout.arms[i];
    const ArmModel* model = FindArmModel(arm.model);
    const ArmHandle& a = b.handles.arms[i];
    const int j6 = mj_name2id(m, mjOBJ_JOINT, (arm.name + "_" + model->joint_names[5]).c_str());
    ASSERT_GE(j6, 0);
    const Eigen::Vector3d z = M(d->site_xmat + 9 * a.flange_site).col(2);
    const Eigen::Vector3d axis = V(d->xaxis + 3 * j6);
    EXPECT_LT(z.cross(axis).norm(), 1e-9) << arm.name;
    const double out = (V(d->site_xpos + 3 * a.flange_site) - V(d->xanchor + 3 * j6)).dot(z);
    EXPECT_GT(out, 0.05) << arm.name;
    // And the last link's collision geoms lie behind the flange's face (on
    // its -z side, to within 5 mm).
    const int last = m->jnt_bodyid[j6];
    for (int g : a.link_geoms) {
      if (m->geom_bodyid[g] != last) continue;
      const double front = Extent(m, d, g, z) - V(d->site_xpos + 3 * a.flange_site).dot(z);
      EXPECT_LT(front, 0.005) << arm.name << " " << GeomName(m, g);
    }
  }
}

// ------------------------------------------------- the smaller KUKA arms

// One joint of a KUKA arm's URDF: the joint's origin in its parent link's
// frame (xyz in metres, roll, pitch and yaw in radians, applied about the
// fixed axes: R = Rz(yaw) Ry(pitch) Rx(roll)), copied from the origin of
// <joint name="joint_N"> (and of link6-tool0 for the flange) of the vendor's
// urdf/<arm>_macro.xacro at the commit pinned in MODULE.bazel
// (kroshu/kuka_robot_descriptions d2fa4377b77b7370cdcf6e52b430fc514a0eef19).
// Every joint turns about the z axis of its own frame. Nothing here is read
// from models/*.xml, so a mistake in a model file shows up as a difference.
struct UrdfJoint {
  double xyz[3];
  double rpy[3];
};

struct SmallKukaArm {
  const char* key;
  // joint_1 .. joint_6, then link6-tool0 (the flange, +z out of it).
  std::array<UrdfJoint, 7> chain;
  // The flange's centre at all joints zero, from a separate calculation of
  // the same chain (a Python script, 9 digits), and at two other poses.
  std::array<double, 3> zero_pos;
  std::array<double, 6> pose_a;
  std::array<double, 3> pose_a_pos;
  std::array<double, 3> pose_a_z;
  std::array<double, 6> pose_b;
  std::array<double, 3> pose_b_pos;
  std::array<double, 3> pose_b_z;
};

std::vector<SmallKukaArm> SmallKukaArms() {
  const double h = kPi / 2.0;
  return {
      {"kr10_r1100_2",
       {{{{0.0, 0.0, 0.208}, {kPi, 0.0, 0.0}},
         {{0.025, -0.0907, -0.192}, {h, 0.0, 0.0}},
         {{0.56, 0.0, -0.0042}, {0.0, 0.0, 0.0}},
         {{0.196, -0.025, -0.0865}, {h, 0.0, -h}},
         {{0.0, 0.0505, -0.319}, {0.0, h, h}},
         {{0.0615, 0.0, -0.0505}, {h, 0.0, -h}},
         {{0.0, 0.0, -0.0285}, {kPi, 0.0, kPi}}}},
       {1.19, 0.0, 0.425},
       {0.3, -1.2, 1.0, 0.5, 0.7, -0.4},
       {0.761079936, -0.264526059, 1.012566191},
       {0.732145946, -0.549773131, -0.402134079},
       {-0.8, -0.6, 0.9, -1.1, -0.5, 1.3},
       {0.771537703, 0.739210894, 0.583247138},
       {0.935386776, 0.349842809, -0.051590591}},
      {"kr20_r1810_2",
       {{{{0.0, 0.0, 0.3127}, {kPi, 0.0, 0.0}},
         {{0.16, 0.0, -0.2073}, {h, 0.0, 0.0}},
         {{0.78, 0.0, 0.0}, {0.0, 0.0, 0.0}},
         {{0.427, -0.15, 0.0}, {h, 0.0, -h}},
         {{0.0, 0.0, -0.433}, {0.0, h, h}},
         {{0.153, 0.0, 0.0}, {h, 0.0, -h}},
         {{0.0, 0.0, 0.0}, {kPi, 0.0, kPi}}}},
       {1.953, 0.0, 0.67},
       {0.3, -1.2, 1.0, 0.5, 0.7, -0.4},
       {1.311630448, -0.455198803, 1.503329584},
       {0.732145946, -0.549773131, -0.402134079},
       {-0.8, -0.6, 0.9, -1.1, -0.5, 1.3},
       {1.306390872, 1.251280487, 0.841680864},
       {0.935386776, 0.349842809, -0.051590591}},
  };
}

Eigen::Isometry3d UrdfOrigin(const UrdfJoint& j) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = (Eigen::AngleAxisd(j.rpy[2], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(j.rpy[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(j.rpy[0], Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  t.translation() = Eigen::Vector3d(j.xyz[0], j.xyz[1], j.xyz[2]);
  return t;
}

// The flange's frame in the arm's base frame for joint angles q.
Eigen::Isometry3d UrdfFlange(const SmallKukaArm& arm, const std::array<double, 6>& q) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 6; ++i) {
    t = t * UrdfOrigin(arm.chain[i]) *
        Eigen::Isometry3d(Eigen::AngleAxisd(q[i], Eigen::Vector3d::UnitZ()));
  }
  return t * UrdfOrigin(arm.chain[6]);
}

// Each smaller KUKA arm (KR 10 R1100-2, KR 20 R1810-2) builds into a layout
// in place of the KR 210, its flange follows the vendor's chain at all
// joints zero and at two other poses (position, z axis and x axis, against a
// chain computed here from the URDF's joint origins and against the numbers
// of a separate calculation), and the catalog's joint ranges lie inside the
// model's. The model's own ranges are the URDF's; the catalog's are the data
// sheet's, which for these two arms are equal to them.
TEST(SmallKukaArms, ModelsFollowTheVendorChain) {
  std::string error;
  const std::optional<Layout> below = LoadLayout(LayoutPath("below"), &error);
  ASSERT_TRUE(below.has_value()) << error;
  for (const SmallKukaArm& spec : SmallKukaArms()) {
    SCOPED_TRACE(spec.key);
    const ArmModel* catalog = FindArmModel(spec.key);
    ASSERT_NE(catalog, nullptr);
    EXPECT_EQ(catalog->licence, "Apache-2.0 (kroshu/kuka_robot_descriptions)");
    Layout layout = *below;
    ArmUse* use = nullptr;
    for (ArmUse& a : layout.arms) {
      if (a.model == "kr210_r2700_2") use = &a;
    }
    ASSERT_NE(use, nullptr);
    use->model = spec.key;
    Built b = Build(layout, Bay(0, 0));
    ASSERT_TRUE(b.model);
    const mjModel* m = b.model.get();
    mjData* d = b.data.get();
    size_t index = 0;
    while (layout.arms[index].name != use->name) ++index;
    const ArmHandle& a = b.handles.arms[index];
    ASSERT_GE(a.flange_site, 0);
    const int root =
        mj_name2id(m, mjOBJ_BODY, (use->name + "_" + catalog->root_body).c_str());
    ASSERT_GE(root, 0);

    // The joint ranges: the catalog's inside the model's. The URDF writes its
    // limits to nine decimals (350 degrees as 6.108652375), so they may fall
    // short of the data sheet's degrees by that much.
    constexpr double kUrdfRounding = 1e-7;
    for (int i = 0; i < 6; ++i) {
      const int j = mj_name2id(m, mjOBJ_JOINT, (use->name + "_" + catalog->joint_names[i]).c_str());
      ASSERT_GE(j, 0) << i;
      EXPECT_TRUE(m->jnt_limited[j]) << i;
      EXPECT_GE(catalog->joint_lo_deg[i] * kPi / 180.0, m->jnt_range[2 * j] - kUrdfRounding) << i;
      EXPECT_LE(catalog->joint_hi_deg[i] * kPi / 180.0, m->jnt_range[2 * j + 1] + kUrdfRounding) << i;
      EXPECT_LT(catalog->joint_lo_deg[i], catalog->joint_hi_deg[i]) << i;
      EXPECT_GT(catalog->joint_speed_deg_s[i], 0.0) << i;
    }

    const std::array<std::array<double, 6>, 3> poses = {
        std::array<double, 6>{}, spec.pose_a, spec.pose_b};
    for (size_t p = 0; p < poses.size(); ++p) {
      SCOPED_TRACE("pose " + std::to_string(p));
      for (int i = 0; i < 6; ++i) d->qpos[a.qpos[i]] = poses[p][i];
      mj_kinematics(m, d);
      const Eigen::Isometry3d base = [&] {
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
        t.linear() = M(d->xmat + 9 * root);
        t.translation() = V(d->xpos + 3 * root);
        return t;
      }();
      const Eigen::Isometry3d want = base * UrdfFlange(spec, poses[p]);
      const Eigen::Vector3d pos = V(d->site_xpos + 3 * a.flange_site);
      const Eigen::Matrix3d rot = M(d->site_xmat + 9 * a.flange_site);
      EXPECT_LT((pos - want.translation()).norm(), 1e-9);
      EXPECT_LT((rot.col(2) - want.linear().col(2)).norm(), 1e-9);
      EXPECT_LT((rot.col(0) - want.linear().col(0)).norm(), 1e-9);
      // The separate calculation, in the base frame.
      const Eigen::Isometry3d in_base = UrdfFlange(spec, poses[p]);
      const std::array<double, 3>& expect_pos =
          p == 0 ? spec.zero_pos : p == 1 ? spec.pose_a_pos : spec.pose_b_pos;
      EXPECT_LT((in_base.translation() - Eigen::Vector3d(expect_pos[0], expect_pos[1], expect_pos[2])).norm(), 1e-8);
      if (p > 0) {
        const std::array<double, 3>& z = p == 1 ? spec.pose_a_z : spec.pose_b_z;
        EXPECT_LT((in_base.linear().col(2) - Eigen::Vector3d(z[0], z[1], z[2])).norm(), 1e-8);
      } else {
        // At zero the flange points along +x.
        EXPECT_LT((in_base.linear().col(2) - Eigen::Vector3d::UnitX()).norm(), 1e-9);
      }
    }

    // The meshes sit where the links are. At the zero pose the last link's
    // collision geoms lie behind the flange's face, as in FlangeZPointsOut,
    // and no link's geom reaches further out along the arm than the flange
    // (to 5 mm) or below the floor under the base (to 5 mm), both measured
    // in the arm's base frame.
    for (int i = 0; i < 6; ++i) d->qpos[a.qpos[i]] = 0.0;
    mj_kinematics(m, d);
    const Eigen::Vector3d origin = V(d->xpos + 3 * root);
    const Eigen::Matrix3d frame = M(d->xmat + 9 * root);
    const Eigen::Vector3d flange = V(d->site_xpos + 3 * a.flange_site);
    const Eigen::Vector3d z = M(d->site_xmat + 9 * a.flange_site).col(2);
    const int j6 = mj_name2id(m, mjOBJ_JOINT, (use->name + "_" + catalog->joint_names[5]).c_str());
    ASSERT_GE(j6, 0);
    int last_link_geoms = 0;
    ASSERT_FALSE(a.link_geoms.empty());
    for (int g : a.link_geoms) {
      if (m->geom_bodyid[g] == m->jnt_bodyid[j6]) {
        ++last_link_geoms;
        EXPECT_LT(Extent(m, d, g, z) - flange.dot(z), 0.005) << GeomName(m, g);
      }
      EXPECT_LT(Extent(m, d, g, frame.col(0)) - flange.dot(frame.col(0)), 0.005) << GeomName(m, g);
      EXPECT_LT(Extent(m, d, g, -frame.col(2)) + origin.dot(frame.col(2)), 0.005) << GeomName(m, g);
    }
    EXPECT_GE(last_link_geoms, 1);
  }
}

// ---------------------------------------------------------------- overlaps

// What a geom belongs to, for the overlap check. Geoms of the same thing are
// not measured against each other.
struct Thing {
  std::string kind;  // "links", "tool", "held", "waiting", "carriage", "beam",
                     // "base", "holder", "tube", "rail", "module",
                     // "pallet_base", "pallet_module", "stock", "item".
  std::string arm;   // For an arm's things.
  int index = -1;    // Rail, module or pallet module number.
  std::string name;  // Stock or item name, held part.
  bool operator<(const Thing& o) const {
    return std::tie(kind, arm, index, name) < std::tie(o.kind, o.arm, o.index, o.name);
  }
  std::string Text() const {
    std::string t = kind;
    if (!arm.empty()) t = arm + "." + t;
    if (index >= 0) t += "_" + std::to_string(index);
    if (!name.empty()) t += "(" + name + ")";
    return t;
  }
};

std::map<int, Thing> Things(const mjModel* m, const Layout& layout, const LayoutHandles& h) {
  std::map<int, Thing> t;
  auto put = [&](const std::vector<int>& geoms, const Thing& thing) {
    for (int g : geoms) t[g] = thing;
  };
  for (const ArmHandle& a : h.arms) {
    put(a.link_geoms, {"links", a.name});
    for (const auto& [tool, geoms] : a.tool_geoms) put(geoms, {"tool", a.name, -1, tool});
    for (const auto& [tool, geoms] : a.waiting_tool_geoms) put(geoms, {"waiting", a.name, -1, tool});
    for (const auto& [part, geoms] : a.held_geoms) put(geoms, {"held", a.name, -1, part});
    put(a.carriage_geoms, {"carriage", a.name});
    put(a.beam_geoms, {"beam", a.name});
  }
  put(h.base_geoms, {"base"});
  put(h.holder_geoms, {"holder"});
  put(h.tube_geoms, {"tube"});
  for (size_t k = 0; k < h.rail_geoms.size(); ++k) put(h.rail_geoms[k], {"rail", "", int(k)});
  for (size_t k = 0; k < h.module_geoms.size(); ++k) put(h.module_geoms[k], {"module", "", int(k)});
  put(h.pallet_base_geoms, {"pallet_base"});
  for (size_t k = 0; k < h.pallet_module_geoms.size(); ++k) {
    put(h.pallet_module_geoms[k], {"pallet_module", "", int(k)});
  }
  for (const auto& [name, geoms] : h.stock_geoms) put(geoms, {"stock", "", -1, name});
  for (const auto& [name, geoms] : h.item_geoms) put(geoms, {"item", "", -1, name});
  (void)m;
  (void)layout;
  return t;
}

// The pairs that touch by design, each with its reason: a held part
// against what it is set on or taken from, the tube in the holder's
// saddles, an arm's base on its carriage, a carriage on its beam, beams and
// stands on the floor (docs/layout.md). Such a pair may touch, but not
// interpenetrate by more than kTouchDepth (Overlaps). `ga`/`gb` are
// the geoms, so that a rule can name the body a geom is on. Returns the
// reason, or "" when the pair must be clear. The ground plane of common.xml
// ("floor", z = 0) is not a thing at all: everything outside the container
// (pallet, stocks, cart) and the container's own bottom stand on it.
std::string TouchesByDesign(const mjModel* m, const Layout& layout, const Thing& a, int ga,
                            const Thing& b, int gb) {
  auto is = [](const Thing& t, const char* kind) { return t.kind == kind; };
  auto either = [&](const char* k1, const char* k2) -> bool {
    return (is(a, k1) && is(b, k2)) || (is(a, k2) && is(b, k1));
  };
  auto geom_of = [&](const char* kind) { return is(a, kind) ? ga : gb; };
  auto thing_of = [&](const char* kind) -> const Thing& { return is(a, kind) ? a : b; };
  auto other_of = [&](const char* kind) -> const Thing& { return is(a, kind) ? b : a; };
  auto name = [&](int g) {
    const char* n = mj_id2name(m, mjOBJ_GEOM, g);
    return std::string(n != nullptr ? n : "");
  };
  // Beams and stands on the container's floor (the floor panel and the bottom
  // side rails it rests on, both at z = 0.45), or on the flat rack's deck.
  if (either("beam", "base") || either("holder", "base")) {
    const std::string n = name(is(a, "base") ? ga : gb);
    if (n == "container_floor" || n.rfind("container_bottom_rail_", 0) == 0 ||
        n == "flat_rack_deck") {
      return "beams and stands stand on the floor";
    }
  }
  // A carriage on its beam.
  if (either("carriage", "beam") && a.arm == b.arm) return "a carriage runs on its beam";
  // An arm's base on its carriage: the root body's geoms against the
  // carriage.
  if (either("links", "carriage") && a.arm == b.arm) {
    const Thing& links = thing_of("links");
    const ArmUse* arm = FindArm(layout, links.arm);
    const int root =
        mj_name2id(m, mjOBJ_BODY, (arm->name + "_" + FindArmModel(arm->model)->root_body).c_str());
    if (m->geom_bodyid[geom_of("links")] == root) return "an arm's base stands on its carriage";
  }
  // A tool waiting on its carriage (the rack is the carriage's top).
  if (either("waiting", "carriage") && a.arm == b.arm) return "a waiting tool stands on its carriage";
  // The waiting tools of one arm stand side by side in one rack.
  if (is(a, "waiting") && is(b, "waiting") && a.arm == b.arm) {
    return "waiting tools stand side by side in one rack";
  }
  // A tool on the flange against the wrist link the flange is on: it is
  // bolted there.
  if (either("tool", "links") && a.arm == b.arm) {
    const ArmUse* arm = FindArm(layout, a.arm);
    const ArmModel* model = FindArmModel(arm->model);
    const int j6 = mj_name2id(m, mjOBJ_JOINT, (arm->name + "_" + model->joint_names[5]).c_str());
    // The wrist's last two bodies: Menagerie's wrist_2 capsule ends on the
    // flange's face too (its cap reaches y = 0.117 of wrist_3).
    const int body = m->geom_bodyid[geom_of("links")];
    const int last = m->jnt_bodyid[j6];
    if (body == last || body == m->body_parentid[last]) {
      return "a tool is bolted to the flange";
    }
  }
  // A held part against the tool that holds it.
  if (either("held", "tool") && a.arm == b.arm) return "a held part against its tool";
  // The tube in the holder's saddles and against its end unit.
  if (either("tube", "holder")) return "the tube lies in the holder";
  // A rail on the tube (its webs on top, its strap round it).
  if (either("rail", "tube")) return "a rail sits on the tube";
  // A module on its two rails.
  if (either("module", "rail")) {
    const int mi = thing_of("module").index, ri = other_of("module").index;
    if (ri == mi || ri == mi + 1) return "a module lies on its two rails";
  }
  // The pallet: modules on their long edge each stand on the deck and lean
  // on their neighbours; flat ones lie each on the one below, the lowest
  // (the last taken) on the deck.
  if (either("pallet_module", "pallet_base") &&
      (layout.pallets[0].stands == "on_long_edge" ||
       thing_of("pallet_module").index == layout.pallets[0].modules - 1)) {
    return "a pallet module stands on the deck";
  }
  if (is(a, "pallet_module") && is(b, "pallet_module") && std::abs(a.index - b.index) == 1) {
    return "a pallet module against its neighbour in the stack";
  }
  return "";
}

// How far a pair that touches by design may sink into each other. Every
// such pair is built to meet at exactly zero (a stand's foot on the floor's
// top, a rail's web ending on the tube, a carriage's top at the base's
// frame); the deliberate gaps of the model are 1 mm and more (the gripper's
// cups 1 mm short of the working point, the saddles 3 mm under the tube). So
// 1 mm allows for the convex hulls of the KUKA's meshes and round-off, and
// still catches a box modelled into what it rests on.
constexpr double kTouchDepth = 0.001;

struct OverlapReport {
  std::vector<std::string> lines;  // Each pair that fails.
  double worst_excused = 1.0;      // The least distance of an excused pair.
  std::string worst_excused_pair;
};

// Every pair of geoms of different things that is not further apart than
// zero and does not touch by design, and every pair that touches by design
// but sinks in deeper than kTouchDepth, one line each.
OverlapReport Overlaps(const mjModel* m, mjData* d, const Layout& layout,
                       const LayoutHandles& h) {
  OverlapReport report;
  std::vector<std::string>& out = report.lines;
  const std::map<int, Thing> things = Things(m, layout, h);
  std::vector<int> geoms;
  for (const auto& [g, t] : things) geoms.push_back(g);
  // Every geom that collides must belong to a thing, or it is not checked.
  for (int g = 0; g < m->ngeom; ++g) {
    if (m->geom_contype[g] == 0 && m->geom_conaffinity[g] == 0) continue;
    if (m->geom_type[g] == mjGEOM_PLANE) continue;
    if (things.count(g) == 0) out.push_back("geom " + GeomName(m, g) + " belongs to nothing");
  }
  for (size_t i = 0; i < geoms.size(); ++i) {
    for (size_t j = i + 1; j < geoms.size(); ++j) {
      const int g1 = geoms[i], g2 = geoms[j];
      const Thing& t1 = things.at(g1);
      const Thing& t2 = things.at(g2);
      if (!(t1 < t2) && !(t2 < t1)) continue;
      const double reach = m->geom_rbound[g1] + m->geom_rbound[g2] + 0.01;
      if ((V(d->geom_xpos + 3 * g1) - V(d->geom_xpos + 3 * g2)).norm() > reach) continue;
      const double dist = mj_geomDistance(m, d, g1, g2, 0.01, nullptr);
      if (dist > 0.0) continue;
      const std::string reason = TouchesByDesign(m, layout, t1, g1, t2, g2);
      char line[400];
      std::snprintf(line, sizeof(line), "%s [%s] and %s [%s]: %.6f m%s%s",
                    GeomName(m, g1).c_str(), t1.Text().c_str(), GeomName(m, g2).c_str(),
                    t2.Text().c_str(), dist, reason.empty() ? "" : ", touching by design: ",
                    reason.c_str());
      if (!reason.empty()) {
        if (dist < report.worst_excused) {
          report.worst_excused = dist;
          report.worst_excused_pair = line;
        }
        if (dist >= -kTouchDepth) continue;
      }
      out.push_back(line);
    }
  }
  return report;
}

void ExpectNoOverlaps(const Layout& layout, const LayoutState& state, const std::string& what) {
  Built b = Build(layout, state);
  ASSERT_TRUE(b.model);
  const OverlapReport report = Overlaps(b.model.get(), b.data.get(), layout, b.handles);
  std::string all;
  for (const std::string& line : report.lines) all += "  " + line + "\n";
  std::printf("%s, %s: worst pair touching by design: %s\n", layout.name.c_str(), what.c_str(),
              report.worst_excused_pair.empty() ? "none" : report.worst_excused_pair.c_str());
  EXPECT_TRUE(report.lines.empty())
      << layout.name << ", " << what << ": " << report.lines.size() << " overlaps\n" << all;
}

TEST(ChosenLayout, NothingOverlapsWithABareTube) {
  ExpectNoOverlaps(Chosen(), Bay(0, 0), "bare tube");
}

TEST(ChosenLayout, NothingOverlapsWithAFinishedBay) {
  ExpectNoOverlaps(Chosen(), Bay(10, 9), "finished bay");
}

TEST(ChosenLayout, StowedHasOnlyWhatTravelsInside) {
  const Layout layout = Chosen();
  LayoutState state;
  state.stowed = true;
  Built b = Build(layout, state);
  ASSERT_TRUE(b.model);
  const LayoutHandles& h = b.handles;
  EXPECT_TRUE(h.tube_geoms.empty());
  EXPECT_TRUE(h.rail_geoms.empty());
  EXPECT_TRUE(h.module_geoms.empty());
  EXPECT_TRUE(h.pallet_base_geoms.empty());
  EXPECT_TRUE(h.pallet_module_geoms.empty());
  EXPECT_TRUE(h.stock_geoms.empty());
  EXPECT_EQ(h.holder_geoms.empty(), !layout.holder.travels_inside);
  for (const Item& item : layout.items) {
    EXPECT_EQ(h.item_geoms.count(item.name), item.travels_inside ? 1u : 0u) << item.name;
  }
  const mjModel* m = b.model.get();
  const mjData* d = b.data.get();
  for (size_t i = 0; i < layout.arms.size(); ++i) {
    const ArmUse& arm = layout.arms[i];
    const ArmHandle& a = h.arms[i];
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(d->qpos[a.qpos[j]], arm.stow_joints_deg[j] * kPi / 180.0, 1e-12);
    }
    if (arm.track) {
      EXPECT_NEAR(d->qpos[a.track_qpos], arm.stow_track, 1e-12);
    }
    EXPECT_EQ(a.tool_geoms.size(), arm.stow_tool.empty() ? 0u : 1u) << arm.name;
    if (!arm.stow_tool.empty()) {
      EXPECT_EQ(a.tool_geoms.count(arm.stow_tool), 1u);
    }
    // Every link and tool inside the container's inside envelope.
    std::vector<int> geoms = a.link_geoms;
    for (const auto& [tool, g] : a.tool_geoms) geoms.insert(geoms.end(), g.begin(), g.end());
    for (int g : geoms) {
      const Eigen::Vector3d x = Eigen::Vector3d::UnitX(), z = Eigen::Vector3d::UnitZ();
      const double half_width = container::kInsideWidth / 2.0;
      const double floor = container::kFloorHeight;
      const double ceiling = container::kFloorHeight + container::kInsideHeight;
      EXPECT_GE(-Extent(m, d, g, -x), -half_width) << arm.name << " " << GeomName(m, g);
      EXPECT_LE(Extent(m, d, g, x), half_width) << arm.name << " " << GeomName(m, g);
      EXPECT_GE(-Extent(m, d, g, -z), floor) << arm.name << " " << GeomName(m, g);
      EXPECT_LE(Extent(m, d, g, z), ceiling) << arm.name << " " << GeomName(m, g);
    }
  }
  const OverlapReport report = Overlaps(m, b.data.get(), layout, h);
  std::string all;
  for (const std::string& line : report.lines) all += "  " + line + "\n";
  std::printf("%s, stowed: worst pair touching by design: %s\n", layout.name.c_str(),
              report.worst_excused_pair.empty() ? "none" : report.worst_excused_pair.c_str());
  EXPECT_TRUE(report.lines.empty()) << "stowed: " << report.lines.size() << " overlaps\n" << all;
}

// Every layout's model compiles with a bare tube, a finished bay and
// stowed, so that no candidate rots.
TEST(Layouts, EveryModelCompiles) {
  for (const std::string& name : ListLayouts(LayoutDir())) {
    std::string error;
    std::optional<Layout> layout = LoadLayout(LayoutPath(name), &error);
    ASSERT_TRUE(layout.has_value()) << error;
    LayoutState stowed;
    stowed.stowed = true;
    for (const LayoutState& state : {LayoutState{}, Bay(10, 9), stowed}) {
      ModelPtr model = LoadLayoutModel(*layout, state, ModelPaths::FromEnvironment(), &error);
      EXPECT_TRUE(model) << name << ": " << error;
      if (model) {
        EXPECT_EQ(model->nkey, 1) << name;
      }
    }
  }
}

TEST(ChosenLayout, EverythingHasEveryToolPartAndTheProbe) {
  const Layout layout = Chosen();
  LayoutState state;
  state.everything = true;
  Built b = Build(layout, state);
  ASSERT_TRUE(b.model);
  const LayoutHandles& h = b.handles;
  EXPECT_EQ(h.rail_geoms.size(), static_cast<size_t>(bay::kRailCount));
  EXPECT_EQ(h.module_geoms.size(), static_cast<size_t>(bay::kModuleCount));
  EXPECT_EQ(h.pallet_module_geoms.size(), static_cast<size_t>(layout.pallets[0].modules));
  for (size_t i = 0; i < layout.arms.size(); ++i) {
    const ArmUse& arm = layout.arms[i];
    const ArmHandle& a = h.arms[i];
    EXPECT_EQ(a.tool_geoms.size(), arm.tools.size());
    for (const Tool& tool : arm.tools) {
      EXPECT_EQ(a.waiting_tool_geoms.count(tool.name), tool.wait_size[0] > 0 ? 1u : 0u);
    }
    for (const char* kind : {"module_set", "rail_set"}) {
      const std::vector<std::string> doing = ArmsDoing(layout, {kind});
      const bool does = std::find(doing.begin(), doing.end(), arm.name) != doing.end();
      const std::string part = std::string(kind) == "module_set" ? "module" : "rail";
      EXPECT_EQ(a.held_geoms.count(part), does ? 1u : 0u) << arm.name << " " << part;
    }
  }
  const mjModel* m = b.model.get();
  mjData* d = b.data.get();
  ASSERT_GE(h.probe_mocap, 0);
  struct Cyl {
    int geom;
    double radius, z0, z1;
  };
  for (const Cyl& c : {Cyl{h.probe_bolt_geom, bay::kStrapBoltFreeRadius, -0.10, -0.001},
                       Cyl{h.probe_pin_geom, bay::kPinFreeRadius, -bay::kPinToolLength, -0.001},
                       Cyl{h.probe_locate_below_geom, bay::kLocateFreeRadiusBelow,
                           -bay::kLocateFreeLengthBelow, -0.001},
                       Cyl{h.probe_locate_above_geom, bay::kLocateFreeRadiusAbove, 0.004,
                           0.004 + bay::kLocateFreeHeightAbove}}) {
    ASSERT_GE(c.geom, 0);
    EXPECT_EQ(m->geom_type[c.geom], mjGEOM_CYLINDER);
    EXPECT_EQ(m->geom_contype[c.geom], 0);
    EXPECT_EQ(m->geom_conaffinity[c.geom], 0);
    EXPECT_NEAR(m->geom_size[3 * c.geom], c.radius, 1e-12);
    EXPECT_NEAR(m->geom_size[3 * c.geom + 1], (c.z1 - c.z0) / 2, 1e-12);
    EXPECT_NEAR(m->geom_pos[3 * c.geom + 2], (c.z0 + c.z1) / 2, 1e-12);
  }
  // Moving the mocap body moves the cylinders.
  d->mocap_pos[3 * h.probe_mocap + 0] = 1.0;
  mj_forward(m, d);
  EXPECT_NEAR(d->geom_xpos[3 * h.probe_pin_geom + 0], 1.0, 1e-12);
}

// ------------------------------------------------- the wider form of a file

// Writes `j` as <tmp>/<which>/<name>.json and loads it; the load must work.
std::optional<Layout> LoadGood(const nlohmann::json& j, const std::string& name,
                               const std::string& which) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::filesystem::path dir =
      std::filesystem::path(tmp != nullptr ? tmp : "/tmp") / which;
  std::filesystem::create_directories(dir);
  const std::string path = (dir / (name + ".json")).string();
  std::ofstream(path) << j.dump(1);
  std::string error;
  std::optional<Layout> l = LoadLayout(path, &error);
  EXPECT_TRUE(l.has_value()) << which << ": " << error;
  return l;
}

constexpr const char* kFlatName = "below_rails_by_module_arm";

// below_rails_by_module_arm moved onto the flat rack as a file: base
// flat_rack with its fields, one port that hands over both pallets, an
// approach for each of the nine kinds, and a second pallet place 1 m beyond
// the first. Nothing is moved to suit the deck's height: this is a test of
// the form and the model's names, not a layout to check.
nlohmann::json FlatRackJson() {
  nlohmann::json j = nlohmann::json::parse(ReadText(LayoutPath(kFlatName)));
  j["base"] = "flat_rack";
  j.erase("container");
  j["flat_rack"] = {{"end_frames", "taken_off"}, {"note", "The base alone."}};
  j["margins"] = {{"clearance_m", 0.010},
                  {"joint_margin_deg", 5.0},
                  {"reason", "The floors."},
                  {"basis", "ours"}};
  nlohmann::json second = j["pallets"][0];
  second["pos"][0] = second["pos"][0].get<double>() + 1.0;
  j["pallets"].push_back(second);
  j["ports"] = nlohmann::json::array();
  j["ports"].push_back({{"name", "modules_in"},
                        {"crosses", "modules"},
                        {"unit", "A forklift sets pallets down here."},
                        {"frame_pos", {1.219, 0.0, 0.648}},
                        {"frame_yaw_deg", 0.0},
                        {"zone", {{"size", {3.0, 3.0, 2.0}}, {"pos", {3.2, 0.0, 1.0}}}},
                        {"hands_over", {"pallet:0", "pallet:1"}},
                        {"shares_zone_with", ""},
                        {"presents", "Standing on their long edge."},
                        {"per_bay", 9.0},
                        {"basis", "ours"}});
  j["approaches"] = nlohmann::json::array();
  for (const std::string& kind : WorkKinds()) {
    j["approaches"].push_back({{"kind", kind},
                               {"approach_frame", "tool"},
                               {"approach_dir", {0.0, 0.0, -1.0}},
                               {"approach_m", 0.05},
                               {"retreat_frame", "world"},
                               {"retreat_dir", {0.0, 0.0, 1.0}},
                               {"retreat_m", 0.05},
                               {"note", "Straight back."},
                               {"basis", "ours"}});
  }
  return j;
}

TEST(LayoutForm, RefusalsOfTheWiderForm) {
  const nlohmann::json good = nlohmann::json::parse(ReadText(LayoutPath(kFlatName)));
  const nlohmann::json flat = FlatRackJson();
  auto has = [](const std::string& e, const std::string& words) {
    return e.find(words) != std::string::npos;
  };
  {
    nlohmann::json j = good;
    j.erase("base");
    const std::string e = LoadBroken(j, kFlatName, "missing_base");
    EXPECT_TRUE(has(e, "base: missing field")) << e;
  }
  {
    nlohmann::json j = good;
    j["base"] = "raft";
    const std::string e = LoadBroken(j, kFlatName, "unknown_base");
    EXPECT_TRUE(has(e, "\"flat_rack\" or \"two_flat_racks\"")) << e;
  }
  {
    nlohmann::json j = flat;
    j["container"] = good["container"];
    const std::string e = LoadBroken(j, kFlatName, "flat_rack_with_container");
    EXPECT_TRUE(has(e, "container: unknown field")) << e;
  }
  {
    nlohmann::json j = good;
    j["ports"] = flat["ports"];
    const std::string e = LoadBroken(j, kFlatName, "container_with_ports");
    EXPECT_TRUE(has(e, "ports: unknown field")) << e;
  }
  {
    nlohmann::json j = flat;
    j.erase("margins");
    const std::string e = LoadBroken(j, kFlatName, "flat_rack_without_margins");
    EXPECT_TRUE(has(e, "margins: missing field")) << e;
  }
  {
    nlohmann::json j = flat;
    j["margins"]["clearance_m"] = 0.005;
    const std::string e = LoadBroken(j, kFlatName, "clearance_below_floor");
    EXPECT_TRUE(has(e, "margins.clearance_m")) << e;
    EXPECT_TRUE(has(e, "floor of 0.010 m")) << e;
  }
  {
    nlohmann::json j = flat;
    j["margins"]["joint_margin_deg"] = 4.0;
    const std::string e = LoadBroken(j, kFlatName, "joint_margin_below_floor");
    EXPECT_TRUE(has(e, "margins.joint_margin_deg")) << e;
    EXPECT_TRUE(has(e, "floor of 5.0 degrees")) << e;
  }
  {
    nlohmann::json j = good;
    j["work"][0]["points"] = "0-8";
    const std::string e = LoadBroken(j, kFlatName, "point_to_no_arm");
    EXPECT_TRUE(has(e, "point rail_take 9 is given to no arm")) << e;
  }
  {
    // rail_take split between the two arms, point 3 given to both.
    nlohmann::json j = good;
    j["work"][0]["points"] = "0-5";
    nlohmann::json other = j["work"][0];
    other["arm"] = "small_arm";
    other["tool"] = "nutrunner";
    other["points"] = "3,6-9";
    j["work"].push_back(other);
    const std::string e = LoadBroken(j, kFlatName, "point_to_two_arms");
    EXPECT_TRUE(has(e, "point rail_take 3 is given to two arms (module_arm and small_arm)"))
        << e;
  }
  for (const char* bad : {"0-3,x", "", "1-", "-2", "5-2", "0,,1", "1 2", "0-9,4"}) {
    nlohmann::json j = good;
    j["work"][0]["points"] = bad;
    const std::string e = LoadBroken(j, kFlatName, "bad_points");
    EXPECT_TRUE(has(e, "work[0].points")) << bad << ": " << e;
  }
  {
    nlohmann::json j = good;
    j["work"][0]["points"] = "0-10";
    const std::string e = LoadBroken(j, kFlatName, "point_out_of_range");
    EXPECT_TRUE(has(e, "the point 10 is outside 0..9")) << e;
  }
  {
    nlohmann::json j = flat;
    j["ports"][0]["crosses"] = "people";
    const std::string e = LoadBroken(j, kFlatName, "unknown_crosses");
    EXPECT_TRUE(has(e, "ports[0].crosses")) << e;
    EXPECT_TRUE(has(e, "\"people\"")) << e;
  }
  for (const char* bad : {"stock:nails", "pallet:2", "item:crane", "way:sideways", "shelf"}) {
    nlohmann::json j = flat;
    j["ports"][0]["hands_over"].push_back(bad);
    const std::string e = LoadBroken(j, kFlatName, "hands_over_nothing");
    EXPECT_TRUE(has(e, "ports[0].hands_over[2]")) << bad << ": " << e;
  }
  {
    nlohmann::json j = flat;
    j["ports"][0]["shares_zone_with"] = "bays_out";
    const std::string e = LoadBroken(j, kFlatName, "shares_with_nothing");
    EXPECT_TRUE(has(e, "is not the name of another port")) << e;
  }
  {
    nlohmann::json j = flat;
    j["approaches"][2]["approach_dir"] = {0.0, 0.0, 2.0};
    const std::string e = LoadBroken(j, kFlatName, "approach_not_unit");
    EXPECT_TRUE(has(e, "approaches[2].approach_dir")) << e;
    EXPECT_TRUE(has(e, "not of unit length")) << e;
  }
  {
    nlohmann::json j = flat;
    j["approaches"].erase(j["approaches"].begin() + 4);
    const std::string e = LoadBroken(j, kFlatName, "missing_approach");
    EXPECT_TRUE(has(e, "\"module_set\" has points and no approach")) << e;
  }
  {
    nlohmann::json j = flat;
    j["approaches"].push_back(j["approaches"][0]);
    const std::string e = LoadBroken(j, kFlatName, "two_approaches");
    EXPECT_TRUE(has(e, "has an approach already")) << e;
  }
}

TEST(LayoutForm, DividedWorkAndTheLookups) {
  nlohmann::json j = nlohmann::json::parse(ReadText(LayoutPath(kFlatName)));
  j["work"][0]["points"] = "0-5,9";
  nlohmann::json other = j["work"][0];
  other["arm"] = "small_arm";
  other["tool"] = "nutrunner";
  other["points"] = " 6 - 8 ";
  j["work"].push_back(other);
  std::optional<Layout> l = LoadGood(j, kFlatName, "divided");
  ASSERT_TRUE(l.has_value());
  EXPECT_EQ(PointCount(*l, "rail_take"), 10);
  EXPECT_EQ(PointCount(*l, "module_set"), 9);
  EXPECT_EQ(PointCount(*l, "pin"), 36);
  EXPECT_EQ(PointCount(*l, "module_take"), 36);
  EXPECT_EQ(PointCount(*l, "pins_take"), 1);
  EXPECT_EQ(PinStocks(*l).size(), 1u);
  EXPECT_EQ(PointsOf(*l, l->work[0]), (std::vector<int>{0, 1, 2, 3, 4, 5, 9}));
  EXPECT_EQ(PointsOf(*l, l->work.back()), (std::vector<int>{6, 7, 8}));
  EXPECT_EQ(FindWork(*l, "rail_take", 9)->arm, "module_arm");
  EXPECT_EQ(FindWork(*l, "rail_take", 7)->arm, "small_arm");
  EXPECT_EQ(FindWork(*l, "rail_take", 10), nullptr);
  EXPECT_EQ(PointsOf(*l, *FindWork(*l, "swage")).size(), 36u);
  EXPECT_EQ(ArmsDoing(*l, {"rail_take"}), (std::vector<std::string>{"module_arm", "small_arm"}));
  EXPECT_EQ(ArmsDoing(*l, {"pin"}), (std::vector<std::string>{"small_arm"}));
  EXPECT_EQ(FindApproach(*l, "pin"), nullptr);
  EXPECT_EQ(FindPort(*l, "modules_in"), nullptr);
}

TEST(LayoutForm, AFlatRackLayoutLoadsAndBuilds) {
  std::optional<Layout> l = LoadGood(FlatRackJson(), kFlatName, "flat_rack");
  ASSERT_TRUE(l.has_value());
  EXPECT_EQ(l->base, "flat_rack");
  EXPECT_EQ(l->flat_rack.end_frames, "taken_off");
  ASSERT_EQ(l->pallets.size(), 2u);
  EXPECT_EQ(PointCount(*l, "module_take"), 72);
  ASSERT_NE(FindPort(*l, "modules_in"), nullptr);
  EXPECT_EQ(FindPort(*l, "modules_in")->zone.size[0], 3.0);
  ASSERT_NE(FindApproach(*l, "swage"), nullptr);
  EXPECT_EQ(FindApproach(*l, "swage")->retreat_frame, "world");

  Built b = Build(*l, Bay(0, 0));
  ASSERT_TRUE(b.model);
  const mjModel* m = b.model.get();
  const LayoutHandles& h = b.handles;
  EXPECT_EQ(h.base_body, "flat_rack_frame");
  std::set<std::string> base;
  for (int g : h.base_geoms) base.insert(GeomName(m, g));
  EXPECT_EQ(base, (std::set<std::string>{"flat_rack_side_rail_0", "flat_rack_side_rail_1",
                                         "flat_rack_end_sill_front", "flat_rack_end_sill_rear",
                                         "flat_rack_deck"}));
  for (int g = 0; g < m->ngeom; ++g) {
    EXPECT_NE(GeomName(m, g).rfind("container_", 0), 0u) << GeomName(m, g);
  }
  // Both pallets: the first keeps its old names, the second has its own.
  EXPECT_EQ(h.pallet_base_geoms.size(), 2u);
  EXPECT_GE(mj_name2id(m, mjOBJ_GEOM, "pallet_base"), 0);
  EXPECT_GE(mj_name2id(m, mjOBJ_GEOM, "pallet1_base"), 0);
  ASSERT_EQ(h.pallet_module_geoms.size(), 72u);
  for (const auto& g : h.pallet_module_geoms) EXPECT_FALSE(g.empty());
  EXPECT_NE(mj_name2id(m, mjOBJ_BODY, "pallet1_module_4"), -1);
  EXPECT_EQ(m->geom_bodyid[h.pallet_module_geoms[40][0]],
            mj_name2id(m, mjOBJ_BODY, "pallet1_module_4"));
  ASSERT_EQ(h.pallet_place_geoms.size(), 2u);
  EXPECT_EQ(h.pallet_place_geoms[1][0], mj_name2id(m, mjOBJ_GEOM, "pallet1_base"));
  // The ports' sites, in group 4, of the zone's size.
  const int zone = mj_name2id(m, mjOBJ_SITE, "port_modules_in_zone");
  const int frame = mj_name2id(m, mjOBJ_SITE, "port_modules_in_frame");
  ASSERT_GE(zone, 0);
  ASSERT_GE(frame, 0);
  EXPECT_EQ(m->site_group[zone], 4);
  EXPECT_EQ(m->site_group[frame], 4);
  EXPECT_NEAR(m->site_size[3 * zone], 1.5, 1e-12);
  EXPECT_NEAR(b.data->site_xpos[3 * frame], 1.219, 1e-12);

  // With one module taken the first pallet has 35, the second 36.
  {
    LayoutState s = Bay(0, 0);
    s.pallet_modules = 35;
    Built few = Build(*l, s);
    ASSERT_TRUE(few.model);
    EXPECT_TRUE(few.handles.pallet_module_geoms[0].empty());
    EXPECT_FALSE(few.handles.pallet_module_geoms[36].empty());
  }
  // Stowed: no pallet, no port.
  {
    LayoutState s;
    s.stowed = true;
    Built stowed = Build(*l, s);
    ASSERT_TRUE(stowed.model);
    EXPECT_TRUE(stowed.handles.pallet_base_geoms.empty());
    EXPECT_TRUE(stowed.handles.pallet_place_geoms.empty());
    EXPECT_EQ(mj_name2id(stowed.model.get(), mjOBJ_SITE, "port_modules_in_zone"), -1);
    EXPECT_EQ(stowed.handles.base_geoms.size(), 5u);
  }
}

// Several arms of one model in one layout: three UR10e, and separately two
// KR 210, each built from a layout changed in memory.
TEST(LayoutForm, SeveralArmsOfOneModelBuild) {
  std::string error;
  std::optional<Layout> below = LoadLayout(LayoutPath("below"), &error);
  ASSERT_TRUE(below.has_value()) << error;
  for (const std::string model : {"ur10e", "kr210_r2700_2"}) {
    Layout l = *below;
    const ArmUse* source = nullptr;
    for (const ArmUse& a : l.arms) {
      if (a.model == model) source = &a;
    }
    ASSERT_NE(source, nullptr) << model;
    const ArmUse copy = *source;
    const int extra = model == "ur10e" ? 2 : 1;
    for (int k = 1; k <= extra; ++k) {
      ArmUse a = copy;
      a.name = copy.name + "_" + std::to_string(k + 1);
      a.base_pos[1] += 3.0 * k;
      l.arms.push_back(a);
    }
    LayoutState state;
    state.everything = true;
    Built b = Build(l, state);
    ASSERT_TRUE(b.model) << model;
    ASSERT_EQ(b.handles.arms.size(), l.arms.size());
    std::set<int> qpos;
    for (const ArmHandle& a : b.handles.arms) {
      for (int j = 0; j < 6; ++j) {
        EXPECT_GE(a.qpos[j], 0) << a.name;
        qpos.insert(a.qpos[j]);
      }
      EXPECT_GE(a.flange_site, 0) << a.name;
      EXPECT_FALSE(a.link_geoms.empty()) << a.name;
    }
    EXPECT_EQ(qpos.size(), 6 * l.arms.size()) << model;
  }
}

// A held rail has its strap open: the two open halves and none of the four
// boxes of the closed strap, and nothing of it below the open halves'
// underside, which is 20 mm below the webs' lower edges. The rails on the
// tube keep the closed strap.
TEST(FlatRackLayout, AHeldRailHasItsStrapOpen) {
  std::string load_error;
  const std::optional<Layout> loaded = LoadLayout(
      std::string(std::getenv("LAYOUT_DIR")) + "/flat_rack_3_pin_arms.json",
      &load_error);
  ASSERT_TRUE(loaded) << load_error;
  const Layout layout = *loaded;
  // On the container the held rail stays as run 011 drew it, strap closed.
  {
    const Layout old = Chosen();
    ASSERT_EQ(old.base, "container");
    LayoutState everything;
    everything.everything = true;
    Built o = Build(old, everything);
    ASSERT_TRUE(o.model);
    for (const std::string& arm : ArmsDoing(old, {"rail_set"})) {
      EXPECT_GE(mj_name2id(o.model.get(), mjOBJ_GEOM,
                           (arm + "_held_rail_strap_ears").c_str()), 0);
      EXPECT_EQ(mj_name2id(o.model.get(), mjOBJ_GEOM,
                           (arm + "_held_rail_strap_open_neg").c_str()), -1);
    }
  }
  LayoutState state;
  state.everything = true;
  Built b = Build(layout, state);
  ASSERT_TRUE(b.model);
  const mjModel* m = b.model.get();
  const mjData* d = b.data.get();
  const std::vector<std::string> rail_arms = ArmsDoing(layout, {"rail_set"});
  ASSERT_FALSE(rail_arms.empty());
  for (const std::string& arm : rail_arms) {
    const std::string held = arm + "_held_rail";
    for (const char* open : {"_strap_open_neg", "_strap_open_pos"}) {
      EXPECT_GE(mj_name2id(m, mjOBJ_GEOM, (held + open).c_str()), 0) << held << open;
    }
    for (const char* closed : {"_strap_neg", "_strap_pos", "_strap_bottom", "_strap_ears"}) {
      EXPECT_EQ(mj_name2id(m, mjOBJ_GEOM, (held + closed).c_str()), -1) << held << closed;
    }
    const int body = mj_name2id(m, mjOBJ_BODY, held.c_str());
    ASSERT_GE(body, 0) << held;
    // The rail's frame: origin on the tube's axis, z up through the rail.
    const Eigen::Vector3d origin = V(d->xpos + 3 * body);
    const Eigen::Vector3d down = -M(d->xmat + 9 * body).col(2);
    const double underside = bay::kRailTopHeight - bay::kRailDepth - 0.020;
    int geoms = 0;
    for (int g = 0; g < m->ngeom; ++g) {
      if (m->geom_bodyid[g] != body) continue;
      ++geoms;
      // The lowest point of the geom, as a height in the rail's frame.
      const double lowest = -(Extent(m, d, g, down) - down.dot(origin));
      EXPECT_GE(lowest, underside - 1e-9) << GeomName(m, g);
    }
    EXPECT_GT(geoms, 2);
  }
  const std::string on_tube = bay::RailName(0);
  for (const char* closed : {"_strap_neg", "_strap_pos", "_strap_bottom", "_strap_ears"}) {
    EXPECT_GE(mj_name2id(m, mjOBJ_GEOM, (on_tube + closed).c_str()), 0) << on_tube << closed;
  }
  EXPECT_EQ(mj_name2id(m, mjOBJ_GEOM, (on_tube + "_strap_open_neg").c_str()), -1);
}

// Stowed, a stock whose "outside" is false travels with the cell and is in
// the model; one that stands outside is not.
TEST(LayoutForm, StowedHasTheStocksThatTravel) {
  std::string error;
  std::optional<Layout> l = LoadLayout(LayoutPath("flat_rack_3_pin_arms"), &error);
  ASSERT_TRUE(l.has_value()) << error;
  Stock* pins = nullptr;
  for (Stock& s : l->stocks) {
    if (s.name == "pins") pins = &s;
  }
  ASSERT_NE(pins, nullptr);
  ASSERT_FALSE(pins->outside);
  pins->boxes = {Box{"magazine", {0.2, 0.2, 0.2}, {0.0, 0.0, 1.0}}};
  LayoutState state;
  state.stowed = true;
  Built b = Build(*l, state);
  ASSERT_TRUE(b.model);
  EXPECT_GE(mj_name2id(b.model.get(), mjOBJ_GEOM, "stock_pins_magazine"), 0);
  EXPECT_EQ(b.handles.stock_geoms.count("pins"), 1u);
  EXPECT_EQ(b.handles.stock_geoms.count("rails"), 0u);  // It stands outside.
  EXPECT_TRUE(b.handles.pallet_base_geoms.empty());     // Every pallet is outside.
}

// On the flat rack a pallet stands outside and the holder travels; on the
// container either may be otherwise, as before.
TEST(LayoutForm, FlatRackRefusesAPalletInsideAndAHolderLeftBehind) {
  const nlohmann::json flat =
      nlohmann::json::parse(ReadText(LayoutPath("flat_rack_3_pin_arms")));
  {
    nlohmann::json j = flat;
    j["pallets"][0]["outside"] = false;
    const std::string e = LoadBroken(j, "flat_rack_3_pin_arms", "pallet_inside");
    EXPECT_NE(e.find("pallets[0].outside"), std::string::npos) << e;
    EXPECT_NE(e.find("on the flat rack a pallet stands at a port, outside the base"),
              std::string::npos)
        << e;
  }
  {
    nlohmann::json j = flat;
    j["holder"]["travels_inside"] = false;
    const std::string e = LoadBroken(j, "flat_rack_3_pin_arms", "holder_stays");
    EXPECT_NE(e.find("on the flat rack the holder travels with the cell and stands on the deck"),
              std::string::npos)
        << e;
  }
  {
    nlohmann::json j = nlohmann::json::parse(ReadText(LayoutPath("below")));
    j["pallets"][0]["outside"] = false;
    j["holder"]["travels_inside"] = false;
    EXPECT_TRUE(LoadGood(j, "below", "container_pallet_inside").has_value());
  }
}

// Base "two_flat_racks": both 20 ft racks are one base body, rack b is at
// its place with the layout's shift, and the working and stowed states build.
TEST(TwoRacks, TheLayoutBuildsWithBothRacksAsTheBase) {
  std::string error;
  std::optional<Layout> l = LoadLayout(LayoutPath("two_flat_racks_2_pin_arms"), &error);
  ASSERT_TRUE(l.has_value()) << error;
  LayoutState stowed;
  stowed.stowed = true;
  for (const LayoutState& state : {LayoutState{}, Bay(10, 9), stowed}) {
    Built b = Build(*l, state);
    ASSERT_TRUE(b.model);
    EXPECT_EQ(b.handles.base_body, "two_flat_racks_frame");
    EXPECT_EQ(b.handles.base_geoms.size(), 10u);
    for (int g : b.handles.base_geoms) {
      const char* name = mj_id2name(b.model.get(), mjOBJ_GEOM, g);
      ASSERT_NE(name, nullptr);
      EXPECT_EQ(std::string(name).rfind("flat_rack_", 0), 0u) << name;
    }
  }
  // Rack a on -X, rack b on +X, the long sides at +-gap/2, the deck 0.370 up.
  Built b = Build(*l, LayoutState{});
  for (int g : b.handles.base_geoms) {
    const std::string name = mj_id2name(b.model.get(), mjOBJ_GEOM, g);
    if (name != "flat_rack_a_deck" && name != "flat_rack_b_deck") continue;
    const double sign = name == "flat_rack_a_deck" ? -1.0 : 1.0;
    EXPECT_NEAR(b.data->geom_xpos[3 * g], sign * (0.0125 + 1.219), 1e-9) << name;
    EXPECT_NEAR(b.data->geom_xpos[3 * g + 2] + b.model->geom_size[3 * g + 2],
                flat_rack::twenty::kDeckHeight, 1e-9) << name;
  }
}

TEST(TwoRacks, MovedRackBMovesRackBsBaseGeomsByTheShift) {
  std::string error;
  std::optional<Layout> l = LoadLayout(LayoutPath("two_flat_racks_2_pin_arms"), &error);
  ASSERT_TRUE(l.has_value()) << error;
  const Layout moved = MovedRackB(*l, 0.025, -0.025);
  EXPECT_NEAR(moved.two_flat_racks.b_shift[0], 0.025, 1e-12);
  EXPECT_NEAR(moved.two_flat_racks.b_shift[1], -0.025, 1e-12);
  Built before = Build(*l, LayoutState{});
  Built after = Build(moved, LayoutState{});
  ASSERT_TRUE(before.model);
  ASSERT_TRUE(after.model);
  ASSERT_EQ(before.handles.base_geoms.size(), after.handles.base_geoms.size());
  for (size_t i = 0; i < before.handles.base_geoms.size(); ++i) {
    const int g = before.handles.base_geoms[i];
    const int h = after.handles.base_geoms[i];
    const std::string name = mj_id2name(before.model.get(), mjOBJ_GEOM, g);
    const bool b_rack = name.rfind("flat_rack_b_", 0) == 0;
    EXPECT_NEAR(after.data->geom_xpos[3 * h] - before.data->geom_xpos[3 * g],
                b_rack ? 0.025 : 0.0, 1e-9) << name;
    EXPECT_NEAR(after.data->geom_xpos[3 * h + 1] - before.data->geom_xpos[3 * g + 1],
                b_rack ? -0.025 : 0.0, 1e-9) << name;
    EXPECT_NEAR(after.data->geom_xpos[3 * h + 2], before.data->geom_xpos[3 * g + 2], 1e-9);
  }
  // What stands on rack b moved with it: the module arm's beam.
  const int beam_before = before.handles.arms[0].beam_geoms.at(0);
  const int beam_after = after.handles.arms[0].beam_geoms.at(0);
  EXPECT_NEAR(after.data->geom_xpos[3 * beam_after] - before.data->geom_xpos[3 * beam_before], 0.025, 1e-9);
  // Rack a's holder stayed.
  EXPECT_NEAR(after.data->geom_xpos[3 * after.handles.holder_geoms.at(0)],
              before.data->geom_xpos[3 * before.handles.holder_geoms.at(0)], 1e-9);
}

// The loader refuses what the second direction forbids: small JSON variants of
// the two-rack layout, written to a temporary directory.
std::string LoadVariant(const nlohmann::json& j, const std::string& which) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::filesystem::path dir =
      std::filesystem::path(tmp != nullptr ? tmp : "/tmp") / ("racks_" + which);
  std::filesystem::create_directories(dir);
  const std::string path = (dir / "two_flat_racks_2_pin_arms.json").string();
  std::ofstream(path) << j.dump(1);
  std::string error;
  EXPECT_FALSE(LoadLayout(path, &error).has_value()) << which;
  return error;
}

nlohmann::json TwoRacksJson() {
  std::ifstream in(LayoutPath("two_flat_racks_2_pin_arms"));
  return nlohmann::json::parse(in);
}

TEST(TwoRackLoader, RefusesWhatTheFormForbids) {
  {
    nlohmann::json j = TwoRacksJson();
    j["arms"][1].erase("rack");
    EXPECT_NE(LoadVariant(j, "arm_no_rack").find("arms[1].rack"), std::string::npos);
  }
  {
    nlohmann::json j = TwoRacksJson();
    j["holder"]["rack"] = "c";
    EXPECT_NE(LoadVariant(j, "rack_c").find("holder.rack"), std::string::npos);
  }
  {
    nlohmann::json j = TwoRacksJson();
    j["two_flat_racks"]["gap_m"] = 0.06;  // Above gap_hi_m.
    const std::string e = LoadVariant(j, "gap");
    EXPECT_NE(e.find("two_flat_racks"), std::string::npos) << e;
    EXPECT_NE(e.find("tolerance"), std::string::npos) << e;
  }
  {
    nlohmann::json j = TwoRacksJson();
    j["staging"]["stages"][1]["rails"] = "3-6";  // Rail 3 is also in stage 1.
    const std::string e = LoadVariant(j, "rail_twice");
    EXPECT_NE(e.find("rail 3"), std::string::npos) << e;
    EXPECT_NE(e.find("two stages"), std::string::npos) << e;
  }
  {
    // Module 0 lies on rails 0 and 1; staged with rails 2-3 only it is before
    // its rail.
    nlohmann::json j = TwoRacksJson();
    j["staging"]["stages"][0]["rails"] = "0";
    j["staging"]["stages"][1]["rails"] = "1-6";
    const std::string e = LoadVariant(j, "module_before_rail");
    EXPECT_NE(e.find("module"), std::string::npos) << e;
  }
  {
    nlohmann::json j = TwoRacksJson();
    j["staging"]["stages"][0]["angle_pin_by"] = "item:nothing_of_that_name";
    EXPECT_NE(LoadVariant(j, "angle_pin_by").find("angle_pin_by"), std::string::npos);
  }
  {
    // A rack on a flat rack layout, which has no second rack.
    nlohmann::json j = nlohmann::json::parse(
        std::ifstream(LayoutPath("flat_rack_3_pin_arms")));
    j["arms"][0]["rack"] = "a";
    const std::string e = LoadVariant(j, "rack_on_flat_rack");
    EXPECT_NE(e.find("arms[0].rack"), std::string::npos) << e;
  }
}

}  // namespace
}  // namespace cell::layout
