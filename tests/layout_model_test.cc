// The layouts and the models made from them (src/layout.h,
// src/layout_model.h).
//
// Every file in layouts/ loads, and broken copies are refused with a message
// that names the field. For the layout marked "chosen" the model compiles in
// the states that matter, agrees with the file, and nothing in it overlaps:
// every pair of geoms of two different things is further apart than zero by
// mj_geomDistance, except the pairs that touch by design (TouchesByDesign
// below gives each rule with its reason). Contacts are not used: MuJoCo
// never reports one between two bodies fixed to the world.
//
//   bazel test --config=opt //:layout_model_test --test_output=all

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
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
  int chosen = 0;
  for (const std::string& name : names) {
    std::string error;
    std::optional<Layout> layout = LoadLayout(LayoutPath(name), &error);
    ASSERT_TRUE(layout.has_value()) << error;
    EXPECT_EQ(layout->name, name);
    chosen += layout->chosen ? 1 : 0;
  }
  EXPECT_EQ(chosen, 1) << "exactly one layout is marked chosen";
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
    nlohmann::json j = good;
    j["arms"][1]["model"] = "ur99";
    const std::string e = LoadBroken(j, "below", "unknown_model");
    EXPECT_NE(e.find("arms[1].model"), std::string::npos) << e;
  }
  {
    nlohmann::json j = good;
    j["pallet"].erase("pitch");
    const std::string e = LoadBroken(j, "below", "missing_field");
    EXPECT_NE(e.find("pallet.pitch"), std::string::npos) << e;
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

Layout Chosen() {
  for (const std::string& name : ListLayouts(LayoutDir())) {
    std::string error;
    std::optional<Layout> layout = LoadLayout(LayoutPath(name), &error);
    if (layout && layout->chosen) return *layout;
  }
  ADD_FAILURE() << "no chosen layout";
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
              static_cast<size_t>(layout.pallet.modules));
    EXPECT_EQ(b.handles.tube_geoms.size(), 2u);
    EXPECT_FALSE(b.handles.container_geoms.empty());
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
  const Pallet& p = layout.pallet;
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

// ---------------------------------------------------------------- overlaps

// What a geom belongs to, for the overlap check. Geoms of the same thing are
// not measured against each other.
struct Thing {
  std::string kind;  // "links", "tool", "held", "waiting", "carriage", "beam",
                     // "container", "holder", "tube", "rail", "module",
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
  put(h.container_geoms, {"container"});
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
  // side rails it rests on, both at z = 0.45).
  if (either("beam", "container") || either("holder", "container")) {
    const std::string n = name(is(a, "container") ? ga : gb);
    if (n == "container_floor" || n.rfind("container_bottom_rail_", 0) == 0) {
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
      (layout.pallet.stands == "on_long_edge" ||
       thing_of("pallet_module").index == layout.pallet.modules - 1)) {
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
  EXPECT_EQ(h.pallet_module_geoms.size(), static_cast<size_t>(layout.pallet.modules));
  for (size_t i = 0; i < layout.arms.size(); ++i) {
    const ArmUse& arm = layout.arms[i];
    const ArmHandle& a = h.arms[i];
    EXPECT_EQ(a.tool_geoms.size(), arm.tools.size());
    for (const Tool& tool : arm.tools) {
      EXPECT_EQ(a.waiting_tool_geoms.count(tool.name), tool.wait_size[0] > 0 ? 1u : 0u);
    }
    for (const char* kind : {"module_set", "rail_set"}) {
      const Work* w = FindWork(layout, kind);
      const std::string part = std::string(kind) == "module_set" ? "module" : "rail";
      EXPECT_EQ(a.held_geoms.count(part), w != nullptr && w->arm == arm.name ? 1u : 0u)
          << arm.name << " " << part;
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

}  // namespace
}  // namespace cell::layout
