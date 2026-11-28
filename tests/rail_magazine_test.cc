// A rail magazine on the module arm's gripper (src/layout_rail_magazine.h,
// run 056): a layout without the field keeps one take for each rail and its
// committed plan verifies as before; the magazine layout's committed bay
// verifies with one take of all ten rails; and the same plan against a
// magazine that holds fewer rails than the arm sets after a take is refused.

#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "src/bay_hardware.h"
#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_rail_magazine.h"

namespace cell::layout {
namespace {

constexpr const char* kInfeed =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed";
constexpr const char* kRailMag =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag";

std::string Dir(const char* var, const char* fallback) {
  const char* dir = std::getenv(var);
  return dir != nullptr ? dir : fallback;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

struct Named {
  std::string error;
  Layout layout;
  Plan plan;
  std::string sha256;
};

Named Load(const std::string& name) {
  Named o;
  const std::string file = Dir("LAYOUT_DIR", "layouts") + "/" + name + ".json";
  const std::optional<Layout> l = LoadLayout(file, &o.error);
  if (!l) return o;
  o.layout = *l;
  const std::optional<Plan> p =
      PlanFromJson(ReadFile(Dir("PLAN_DIR", "plans") + "/" + name + ".json"), &o.error);
  if (!p) return o;
  o.plan = *p;
  o.sha256 = Sha256OfFile(file);
  return o;
}

std::string Faults(const VerifyReport& r) {
  std::string s;
  for (const std::string& f : r.faults) s += "  " + f + "\n";
  return s;
}

int CountSteps(const Plan& p, const std::string& prefix) {
  int n = 0;
  for (const BayStep& s : p.steps) n += s.what.rfind(prefix, 0) == 0 ? 1 : 0;
  return n;
}

Tool* Gripper(Layout* l) {
  for (ArmUse& a : l->arms) {
    for (Tool& t : a.tools) {
      if (a.name == "module_arm" && t.name == "combined_gripper") return &t;
    }
  }
  return nullptr;
}

TEST(RailMagazine, ALayoutWithoutTheFieldKeepsATakeForEachRail) {
  const Named o = Load(kInfeed);
  ASSERT_TRUE(o.error.empty()) << o.error;
  for (const ArmUse& a : o.layout.arms) {
    for (const Tool& t : a.tools) EXPECT_EQ(t.magazine_holds, 0) << a.name << " " << t.name;
  }
  EXPECT_EQ(RailsPerTake(o.layout), 1);
  const Work* take = FindWork(o.layout, "rail_take");
  ASSERT_NE(take, nullptr);
  for (int k = 0; k < bay::kRailCount; ++k) {
    EXPECT_TRUE(RailHasTake(o.layout, k));
    EXPECT_EQ(WorkSeconds(o.layout, *take, "rail_take", k), take->process_s);
  }
  EXPECT_DOUBLE_EQ(MagazineRailsMass(o.layout), bay::kRailMass);
  // Its committed plan still has a take for every rail of every bay, and
  // it verifies with its schedule (also Kr20.ThreeBaysWithTheInfeed...).
  EXPECT_EQ(CountSteps(o.plan, "take rail"), 3 * bay::kRailCount);
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
}

TEST(RailMagazine, TheMagazineRidesOnTheGripper) {
  const Named o = Load(kRailMag);
  ASSERT_TRUE(o.error.empty()) << o.error;
  Layout l = o.layout;
  const Tool* g = Gripper(&l);
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->magazine_holds, 10);
  EXPECT_DOUBLE_EQ(g->mass_kg, 36.0 + 12.0);
  ASSERT_EQ(g->boxes.size(), 5U);
  EXPECT_EQ(g->boxes[1].name, "magazine_pos_x");
  EXPECT_EQ(g->boxes[2].name, "magazine_neg_x");
  EXPECT_EQ(g->boxes.back().name, "cups");  // The box that touches stays last.
  EXPECT_EQ(RailsPerTake(l), 10);
  EXPECT_TRUE(RailHasTake(l, 0));
  for (int k = 1; k < bay::kRailCount; ++k) EXPECT_FALSE(RailHasTake(l, k));
  EXPECT_NEAR(MagazineRailsMass(l), 23.5, 1e-9);
}

TEST(RailMagazine, OneBayWithTheMagazinePassesTheVerifier) {
  const Named o = Load(kRailMag);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_TRUE(o.plan.complete);
  EXPECT_EQ(o.plan.bays, 1);
  EXPECT_EQ(CountSteps(o.plan, "take rail"), 1);
  EXPECT_EQ(CountSteps(o.plan, "set rail"), bay::kRailCount);
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  std::printf("one bay %.2f s (v21 496.59 s)\n", o.plan.bay_planned_s);
  EXPECT_LE(o.plan.bay_planned_s, 475.0);
}

TEST(RailMagazine, SettingMoreRailsThanTheMagazineHoldsIsRefused) {
  Named o = Load(kRailMag);
  ASSERT_TRUE(o.error.empty()) << o.error;
  Tool* g = Gripper(&o.layout);
  ASSERT_NE(g, nullptr);
  g->magazine_holds = 9;  // The plan loads ten at its one take and sets ten.
  VerifyOptions quick;
  quick.schedule = false;
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, quick);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (const std::string& f : r.faults) {
    found = found || (f.find("set rail 9") != std::string::npos &&
                      f.find("rail magazine is empty") != std::string::npos);
  }
  EXPECT_TRUE(found) << Faults(r);
}

}  // namespace
}  // namespace cell::layout
