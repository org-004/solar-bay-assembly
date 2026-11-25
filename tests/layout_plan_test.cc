// The planner's pieces on small cases (src/layout_plan.h). No test plans a
// whole bay: that is //:plan_layout's work.

#include <cmath>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {
namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;
const char* kName = "flat_rack_3_pin_arms";

std::string LayoutPath() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return std::string(dir != nullptr ? dir : "layouts") + "/" + kName + ".json";
}

Layout Load() {
  std::string error;
  std::optional<Layout> layout = LoadLayout(LayoutPath(), &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout ? *layout : Layout{};
}

Config WaitOf(const LayoutScene& scene, int arm) {
  Config c{};
  for (int j = 0; j < 6; ++j) c[j] = scene.arm(arm).wait[j];
  c[6] = scene.arm(arm).wait_track;
  return c;
}

// The first two rails: take and set rails 0 and 1, planned once with one
// thread and once with three, shared by the tests below.
struct Part {
  PlanResult one, three;
};
const Part& FirstRails() {
  static const Part* part = [] {
    const Layout layout = Load();
    const ModelPaths paths = ModelPaths::FromEnvironment();
    PlanOptions options;
    options.first_step = 3;
    options.last_step = 6;
    auto* p = new Part;
    options.threads = 1;
    p->one = PlanLayout(layout, paths, "test", options);
    options.threads = 3;
    p->three = PlanLayout(layout, paths, "test", options);
    return p;
  }();
  return *part;
}

TEST(MoveWorld, FreeAtWaitAndNotInTheDeck) {
  const Layout layout = Load();
  std::string error;
  std::unique_ptr<LayoutScene> scene = LayoutScene::Load(
      layout, ModelPaths::FromEnvironment(), 5.0, &error);
  ASSERT_NE(scene, nullptr) << error;
  const int arm = scene->ArmIndex("module_arm");
  ASSERT_GE(arm, 0);
  const std::string tool = scene->arm(arm).use->first_tool;
  Built built;
  built.on_pallets = PointCount(layout, "module_take");
  const MoveWorld world(*scene, arm, tool, "", built, 0.015);
  DataPtr d = scene->MakeData();
  const Config wait = WaitOf(*scene, arm);
  EXPECT_TRUE(world.Free(d.get(), wait))
      << scene->PairText(world.Least(d.get(), wait, kSceneFar));
  // Drive the arm down until its flange is well below the deck's top.
  const int flange = scene->arm(arm).h->flange_site;
  ASSERT_GE(flange, 0);
  std::optional<Config> low;
  for (double j2 = -140.0; j2 <= 0.0 && !low; j2 += 10.0) {
    for (double j3 = 150.0; j3 >= -100.0 && !low; j3 -= 10.0) {
      Config c = wait;
      c[1] = j2 * kDeg;
      c[2] = j3 * kDeg;
      scene->SetPose(d.get(), arm, {{c[0], c[1], c[2], c[3], c[4], c[5]}, c[6]});
      mj_kinematics(&scene->model(), d.get());
      if (d->site_xpos[3 * flange + 2] < 0.3) low = c;
    }
  }
  ASSERT_TRUE(low.has_value());
  const Nearest n = world.Least(d.get(), *low, kSceneFar);
  EXPECT_FALSE(world.Free(d.get(), *low)) << scene->PairText(n);
  EXPECT_LT(n.distance, 0.015);
}

TEST(PlanLayout, FirstRailsFoundAndTheSameOnAnyThreads) {
  const Part& p = FirstRails();
  ASSERT_FALSE(p.one.rows.empty());
  for (const MoveRow& row : p.one.rows) {
    EXPECT_TRUE(row.found) << "step " << row.step << " " << row.from << " to "
                           << row.to << ": " << row.note;
  }
  // A part is never complete, but nothing else is wrong with it.
  EXPECT_FALSE(p.one.plan.complete);
  ASSERT_EQ(p.one.faults.size(), 1u);
  EXPECT_NE(p.one.faults[0].find("never complete"), std::string::npos)
      << p.one.faults[0];
  EXPECT_EQ(PlanToJson(p.one.plan), PlanToJson(p.three.plan));
  ASSERT_EQ(p.one.rows.size(), p.three.rows.size());
  for (size_t i = 0; i < p.one.rows.size(); ++i) {
    EXPECT_EQ(p.one.rows[i].queries, p.three.rows[i].queries);
    EXPECT_EQ(p.one.rows[i].waypoints, p.three.rows[i].waypoints);
  }
}

TEST(PlanLayout, MovesHaveTheirSeedsBoundsAndEnds) {
  const Part& p = FirstRails();
  const Layout layout = Load();
  std::string error;
  std::unique_ptr<LayoutScene> scene = LayoutScene::Load(
      layout, ModelPaths::FromEnvironment(), p.one.plan.figures.joint_margin_deg,
      &error);
  ASSERT_NE(scene, nullptr) << error;
  std::vector<std::uint32_t> seeds;
  int moves = 0;
  for (size_t a = 0; a < p.one.plan.arms.size(); ++a) {
    const ArmMotion m = MotionOf(*scene, static_cast<int>(a), p.one.plan.figures);
    const std::vector<Segment>& segs = p.one.plan.arms[a].segments;
    for (size_t k = 0; k < segs.size(); ++k) {
      const Segment& s = segs[k];
      if (s.kind != "move") continue;
      ++moves;
      seeds.push_back(s.seed);
      ASSERT_GE(s.path.size(), 2u);
      for (const Config& c : s.path) {
        for (int j = 0; j < 7; ++j) {
          EXPECT_GE(c[j], m.lower[j] - 1e-9) << "joint " << j;
          EXPECT_LE(c[j], m.upper[j] + 1e-9) << "joint " << j;
        }
      }
      // The move starts where the arm stood and ends where the next
      // segment begins.
      ASSERT_GT(k, 0u);
      EXPECT_EQ(s.path.front(), segs[k - 1].path.back());
      if (k + 1 < segs.size()) {
        EXPECT_EQ(s.path.back(), segs[k + 1].path.front());
      }
    }
  }
  EXPECT_GE(moves, 3);
  ASSERT_GE(seeds.size(), 2u);
  EXPECT_NE(seeds[0], seeds[1]);
  // The module arm's first move starts at its wait pose.
  const int arm = scene->ArmIndex("module_arm");
  for (const Segment& s : p.one.plan.arms[arm].segments) {
    if (s.kind != "move") continue;
    EXPECT_EQ(s.path.front(), WaitOf(*scene, arm));
    EXPECT_EQ(s.from, "wait");
    break;
  }
}

}  // namespace
}  // namespace cell::layout
