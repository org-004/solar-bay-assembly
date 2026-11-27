// What moves and what is there (src/layout_scene.h): the one distance
// function reports a capsule whose axis passes through a box as overlapping
// (R15), the other arms can be left out, and the legs of a take and a set
// change what is held and what is there.

#include "src/layout_scene.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"

namespace cell::layout {
namespace {

std::string Dir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr ? dir : "layouts";
}

Layout Named(const std::string& name) {
  std::string error;
  const std::optional<Layout> layout =
      LoadLayout(Dir() + "/" + name + ".json", &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout.value_or(Layout{});
}

std::unique_ptr<LayoutScene> Scene(const Layout& layout) {
  std::string error;
  auto scene =
      LayoutScene::Load(layout, ModelPaths::FromEnvironment(), 5.0, &error);
  EXPECT_NE(scene, nullptr) << error;
  return scene;
}

std::string BodyOf(const mjModel& m, int g) {
  const char* name = mj_id2name(&m, mjOBJ_BODY, m.geom_bodyid[g]);
  return name != nullptr ? name : "";
}

// The longest collision geom of the UR10e's forearm in a model made here
// (the other one is the short one across the elbow).
int Forearm(const LayoutScene& scene, int arm) {
  const mjModel& m = scene.model();
  int best = -1;
  for (int g : scene.arm(arm).h->link_geoms) {
    if (BodyOf(m, g).find("forearm") == std::string::npos) continue;
    if (best < 0 || m.geom_size[3 * g + 1] > m.geom_size[3 * best + 1]) {
      best = g;
    }
  }
  return best;
}

// R15: a capsule of Menagerie's UR10e (the forearm) with its axis through a
// thin plate among the obstacles is reported as overlapping, by the
// function the planner uses, in the model LoadLayoutModel makes.
TEST(LayoutScene, CapsuleThroughABoxOverlaps) {
  Layout layout = Named("below");
  const int arm = 1;
  ASSERT_EQ(layout.arms[arm].model, "ur10e");
  auto plain = Scene(layout);
  ASSERT_NE(plain, nullptr);
  // No capsule that collides is left in the compiled model.
  const mjModel& m0 = plain->model();
  for (int g = 0; g < m0.ngeom; ++g) {
    if (m0.geom_contype[g] == 0 && m0.geom_conaffinity[g] == 0) continue;
    EXPECT_NE(m0.geom_type[g], mjGEOM_CAPSULE) << plain->GeomName(g);
  }
  // The arm standing straight up from its wait pose, so that the upper arm
  // is away from the forearm's middle and nothing else is near. Menagerie's forearm is a capsule (a
  // cylinder here), its axis the geom's z.
  ArmPose pose{plain->arm(arm).wait, plain->arm(arm).wait_track};
  pose.q[1] = -M_PI / 2.0;  // The upper arm straight up,
  pose.q[2] = 0.0;          // the forearm straight on from it.
  const int fa = Forearm(*plain, arm);
  ASSERT_GE(fa, 0);
  EXPECT_EQ(m0.geom_type[fa], mjGEOM_CYLINDER);
  DataPtr d0 = plain->MakeData();
  plain->SetPose(d0.get(), arm, pose);
  mj_kinematics(&m0, d0.get());
  const Eigen::Vector3d centre(d0->geom_xpos[3 * fa], d0->geom_xpos[3 * fa + 1],
                               d0->geom_xpos[3 * fa + 2]);
  const Eigen::Vector3d axis(d0->geom_xmat[9 * fa + 2],
                             d0->geom_xmat[9 * fa + 5],
                             d0->geom_xmat[9 * fa + 8]);
  int along = 0;
  axis.cwiseAbs().maxCoeff(&along);
  // A plate 4 mm thick across the world axis nearest the forearm's, 0.2 m
  // square, centred on the forearm: its axis passes through the plate.
  Item plate;
  plate.name = "plate";
  plate.does = "A plate across the forearm, for the test.";
  Box box;
  box.name = "plate";
  box.size = {0.2, 0.2, 0.2};
  box.size[along] = 0.004;
  box.pos = {centre.x(), centre.y(), centre.z()};
  plate.boxes.push_back(box);
  layout.items.push_back(plate);
  auto scene = Scene(layout);
  ASSERT_NE(scene, nullptr);
  DataPtr d = scene->MakeData();
  scene->SetPose(d.get(), arm, pose);
  mj_kinematics(&scene->model(), d.get());
  const int g = Forearm(*scene, arm);
  ASSERT_GE(g, 0);
  const auto& items = scene->handles().item_geoms;
  ASSERT_TRUE(items.count("plate"));
  const int plate_geom = items.at("plate").front();
  const std::vector<MovingGeom> moving =
      scene->Moving(arm, layout.arms[arm].first_tool, "");
  Built built;
  built.on_pallets = PointCount(layout, "module_take");
  const std::vector<Obstacle> obstacles = scene->Obstacles(
      arm, layout.arms[arm].first_tool, built, DesignPairs());
  const Nearest n = scene->LeastDistance(d.get(), moving, obstacles);
  EXPECT_LT(n.distance, 0.0) << scene->PairText(n);
  EXPECT_EQ(n.obstacle, plate_geom) << scene->PairText(n);
  EXPECT_NE(BodyOf(scene->model(), n.moving).find("forearm"),
            std::string::npos)
      << scene->PairText(n);
  // The forearm alone against the plate alone, too.
  const Nearest one = scene->LeastDistance(
      d.get(), {{g, kLinks}}, {{plate_geom, 0u, 0u}});
  EXPECT_LT(one.distance, 0.0);
}

// The other arms are obstacles or not, as the caller asks; their beams are
// always there. One arm's moving set measures against another's through
// the same function.
TEST(LayoutScene, OtherArmsLeftOut) {
  const Layout layout = Named("below");
  auto scene = Scene(layout);
  ASSERT_NE(scene, nullptr);
  const std::string tool = layout.arms[0].first_tool;
  Built built;
  ObstacleOptions without;
  without.other_arms = false;
  const auto all = scene->Obstacles(0, tool, built, DesignPairs());
  const auto alone = scene->Obstacles(0, tool, built, DesignPairs(), without);
  std::set<int> in_all, in_alone;
  for (const Obstacle& o : all) in_all.insert(o.geom);
  for (const Obstacle& o : alone) in_alone.insert(o.geom);
  const ArmHandle& other = *scene->arm(1).h;
  for (int g : other.link_geoms) {
    EXPECT_TRUE(in_all.count(g));
    EXPECT_FALSE(in_alone.count(g));
  }
  for (int g : other.beam_geoms) EXPECT_TRUE(in_alone.count(g));
  // Arm against arm: both at their wait poses, apart.
  DataPtr d = scene->MakeData();
  std::vector<Obstacle> them;
  for (const MovingGeom& g :
       scene->Moving(1, layout.arms[1].first_tool, "")) {
    them.push_back({g.geom, 0u, 0u});
  }
  const Nearest n =
      scene->LeastDistance(d.get(), scene->Moving(0, tool, ""), them);
  EXPECT_GT(n.distance, 0.0) << scene->PairText(n);
  // Two datas on one scene give the same figure.
  DataPtr e = scene->MakeData();
  EXPECT_EQ(scene->LeastDistance(e.get(), scene->Moving(0, tool, ""), them)
                .distance,
            n.distance);
}

// What is held and what is there on the legs of a take and a set.
TEST(LayoutScene, LegsOfATakeAndASet) {
  const Layout layout = Named("below");
  auto scene = Scene(layout);
  ASSERT_NE(scene, nullptr);
  const LayoutHandles& h = scene->handles();
  const int total = PointCount(layout, "module_take");
  const WorkPoint take = WorkPoints(layout, "module_take")[2];
  const StepScene work = scene->AtWorkPoint(take, Leg::kWork);
  const StepScene in = scene->AtWorkPoint(take, Leg::kApproach);
  const StepScene out = scene->AtWorkPoint(take, Leg::kRetreat);
  ASSERT_TRUE(work.error.empty()) << work.error;
  EXPECT_EQ(work.held, "module");
  EXPECT_EQ(in.held, "");
  EXPECT_EQ(out.held, "module");
  EXPECT_EQ(work.built.front().on_pallets, total - 3);
  EXPECT_EQ(in.built.front().on_pallets, total - 2);
  for (int g : h.pallet_module_geoms[2]) {
    EXPECT_TRUE(in.pairs.tool_near.count(g));
  }
  for (int g : h.pallet_module_geoms[3]) {
    EXPECT_TRUE(work.pairs.held_skip.count(g));
    EXPECT_TRUE(out.pairs.held_skip.count(g));
  }
  const WorkPoint set = WorkPoints(layout, "rail_set")[3];
  const StepScene before = scene->AtWorkPoint(set, Leg::kApproach);
  const StepScene after = scene->AtWorkPoint(set, Leg::kRetreat);
  EXPECT_EQ(before.held, "rail");
  EXPECT_EQ(before.built.front().rails.size(), 3u);
  EXPECT_EQ(after.held, "");
  EXPECT_EQ(after.built.front().rails.size(), 4u);
  for (int g : h.rail_geoms[3]) EXPECT_TRUE(after.pairs.tool_near.count(g));
  // A pin on module 2: two Builts, before and with module 3.
  const StepScene pin = scene->AtWorkPoint(WorkPoints(layout, "pin")[9]);
  ASSERT_EQ(pin.built.size(), 2u);
  EXPECT_EQ(pin.built[0].modules.size(), 3u);
  EXPECT_EQ(pin.built[1].modules.size(), 4u);
}

// PairQuery gives LeastDistance's answers to the bit, and Closer its
// answer to "nearer than far", over random poses of each arm of the 40 ft
// cell, one pose after another on one mjData as a move's planner asks them;
// and after Place every geom and body stands where mj_kinematics puts it.
TEST(LayoutScene, PairQueryAgreesWithLeastDistance) {
  const Layout layout = Named("flat_rack_3_pin_arms_ordered");
  auto scene = Scene(layout);
  ASSERT_NE(scene, nullptr);
  const mjModel& m = scene->model();
  Built built;
  built.rails = {0, 1, 2};
  built.modules = {0};
  built.on_pallets = PointCount(layout, "module_take") - 1;
  std::uint64_t state = 12345;
  auto unit = [&]() {  // A fixed sequence in [0, 1).
    state = state * 6364136223846793005ull + 1442695040888963407ull;
    return static_cast<double>(state >> 11) / 9007199254740992.0;
  };
  int free = 0, not_free = 0;
  for (int arm = 0; arm < scene->ArmCount(); ++arm) {
    SCOPED_TRACE(testing::Message() << "arm " << arm);
    const std::string tool = layout.arms[arm].first_tool;
    const std::vector<MovingGeom> moving = scene->Moving(arm, tool, "");
    const std::vector<Obstacle> obstacles =
        scene->Obstacles(arm, tool, built, DesignPairs());
    PairQuery query(*scene, arm, moving, obstacles);
    DataPtr ref = scene->MakeData();
    DataPtr d = scene->MakeData();
    const auto& h = *scene->arm(arm).h;
    std::vector<int> qpos(h.qpos.begin(), h.qpos.end());
    if (h.track_qpos >= 0) qpos.push_back(h.track_qpos);
    for (int k = 0; k < 300; ++k) {
      for (int adr : qpos) {
        double lo = -3.14, hi = 3.14;
        for (int j = 0; j < m.njnt; ++j) {
          if (m.jnt_qposadr[j] == adr && m.jnt_limited[j]) {
            lo = m.jnt_range[2 * j];
            hi = m.jnt_range[2 * j + 1];
          }
        }
        ref->qpos[adr] = d->qpos[adr] = lo + unit() * (hi - lo);
      }
      mj_kinematics(&m, ref.get());
      query.Place(d.get());
      auto same = [&](const mjtNum* a, const mjtNum* b, int n) {
        return std::memcmp(a, b, sizeof(mjtNum) * n) == 0;
      };
      ASSERT_TRUE(same(ref->xpos, d->xpos, 3 * m.nbody));
      ASSERT_TRUE(same(ref->xquat, d->xquat, 4 * m.nbody));
      ASSERT_TRUE(same(ref->xmat, d->xmat, 9 * m.nbody));
      ASSERT_TRUE(same(ref->xipos, d->xipos, 3 * m.nbody));
      ASSERT_TRUE(same(ref->xiquat, d->xiquat, 4 * m.nbody));
      ASSERT_TRUE(same(ref->ximat, d->ximat, 9 * m.nbody));
      ASSERT_TRUE(same(ref->xanchor, d->xanchor, 3 * m.njnt));
      ASSERT_TRUE(same(ref->xaxis, d->xaxis, 3 * m.njnt));
      ASSERT_TRUE(same(ref->geom_xpos, d->geom_xpos, 3 * m.ngeom));
      ASSERT_TRUE(same(ref->geom_xquat, d->geom_xquat, 4 * m.ngeom));
      ASSERT_TRUE(same(ref->geom_xmat, d->geom_xmat, 9 * m.ngeom));
      ASSERT_TRUE(same(ref->site_xpos, d->site_xpos, 3 * m.nsite));
      ASSERT_TRUE(same(ref->site_xquat, d->site_xquat, 4 * m.nsite));
      ASSERT_TRUE(same(ref->site_xmat, d->site_xmat, 9 * m.nsite));
      for (double far : {0.0, 0.03, 0.1, kSceneFar}) {
        const Nearest want = scene->LeastDistance(ref.get(), moving, obstacles, far);
        const Nearest got = query.Least(d.get(), far);
        EXPECT_EQ(got.distance, want.distance);
        EXPECT_EQ(got.moving, want.moving);
        EXPECT_EQ(got.obstacle, want.obstacle);
        EXPECT_EQ(got.near, want.near);
        EXPECT_EQ(query.Closer(d.get(), far), want.distance < far);
        if (far == 0.03) ++(want.distance < far ? not_free : free);
      }
    }
  }
  // Both answers were met often.
  EXPECT_GT(free, 50);
  EXPECT_GT(not_free, 50);
}

}  // namespace
}  // namespace cell::layout
