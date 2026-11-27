// A layout's retreat_legs (src/layout.h, run 036): the figures of a leg that
// leaves contact into free air. With the field a retreat is timed at its
// figures and an approach keeps leg_speed and leg_accel; the plan file names
// them and the verifier holds a plan to the layout's. Without it everything
// is as before: a retreat is timed as an approach, the plan file names no
// retreat figure, and a plan that brings one is refused.

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_plan_stages.h"
#include "src/layout_scene.h"

namespace cell::layout {
namespace {

std::string Env(const char* name, const char* fallback) {
  const char* v = std::getenv(name);
  return v != nullptr ? v : fallback;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

std::string LayoutPath(const std::string& name) {
  return Env("LAYOUT_DIR", "layouts") + "/" + name + ".json";
}

// The layout `name` with its retreat_legs replaced by `legs` (null: none),
// written as <name>.json under the test's own directory and loaded.
std::optional<Layout> WithLegs(const std::string& name, const nlohmann::json& legs,
                               std::string* error) {
  nlohmann::json j = nlohmann::json::parse(ReadFile(LayoutPath(name)));
  j.erase("retreat_legs");
  if (!legs.is_null()) j["retreat_legs"] = legs;
  const std::string path = Env("TEST_TMPDIR", ".") + "/" + name + ".json";
  std::ofstream(path) << j.dump(2);
  return LoadLayout(path, error);
}

constexpr const char* kV13 = "two_flat_racks_2_pin_arms_balanced_ordered";
constexpr const char* kFast = "two_flat_racks_2_pin_arms_balanced_ordered_fast_retreat";
constexpr const char* kKr20 = "flat_rack_1_kr20_pin_arm_ordered";

// ArmMotion of the KR 210 of a layout's scene.
ArmMotion KukaMotion(const Layout& layout, const PlanFigures& f) {
  std::string error;
  const std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(layout, ModelPaths::FromEnvironment(), 5.0, &error);
  EXPECT_NE(scene, nullptr) << error;
  for (int i = 0; scene && i < scene->ArmCount(); ++i) {
    if (scene->arm(i).use->model == "kr210_r2700_2") return MotionOf(*scene, i, f);
  }
  ADD_FAILURE() << "no KR 210 in " << layout.name;
  return {};
}

TEST(RetreatLegs, TheFieldIsReadAndBounded) {
  std::string error;
  const std::optional<Layout> v13 = LoadLayout(LayoutPath(kV13), &error);
  ASSERT_TRUE(v13.has_value()) << error;
  EXPECT_FALSE(v13->retreat_legs.has_value());
  const std::optional<Layout> fast = LoadLayout(LayoutPath(kFast), &error);
  ASSERT_TRUE(fast.has_value()) << error;
  ASSERT_TRUE(fast->retreat_legs.has_value());
  EXPECT_EQ(fast->retreat_legs->speed, 0.5);
  EXPECT_EQ(fast->retreat_legs->accel, 3.0);
  // The new layout is v13's and the field, nothing else.
  nlohmann::json a = nlohmann::json::parse(ReadFile(LayoutPath(kV13)));
  nlohmann::json b = nlohmann::json::parse(ReadFile(LayoutPath(kFast)));
  for (const char* key : {"name", "title", "summary"}) {
    a.erase(key);
    b.erase(key);
  }
  b.erase("retreat_legs");
  EXPECT_EQ(a, b);
  const auto legs = [](double speed, double accel) {
    return nlohmann::json{{"speed", speed}, {"accel", accel}, {"basis", "test"}};
  };
  EXPECT_TRUE(WithLegs(kV13, legs(0.25, 1.0), &error).has_value()) << error;
  EXPECT_FALSE(WithLegs(kV13, legs(0.51, 3.0), &error).has_value());
  EXPECT_NE(error.find("retreat_legs.speed"), std::string::npos) << error;
  EXPECT_FALSE(WithLegs(kV13, legs(0.5, 3.01), &error).has_value());
  EXPECT_NE(error.find("retreat_legs.accel"), std::string::npos) << error;
  EXPECT_FALSE(WithLegs(kV13, legs(0.0, 1.0), &error).has_value());
  EXPECT_FALSE(WithLegs(kV13, nlohmann::json{{"speed", 0.5}, {"accel", 3.0}}, &error));
}

TEST(RetreatLegs, ARetreatRunsAtItsFiguresAndAnApproachAtTheLegs) {
  std::string error;
  const std::optional<Layout> l = LoadLayout(LayoutPath(kV13), &error);
  ASSERT_TRUE(l.has_value()) << error;
  PlanFigures f;
  // Without the field the two are one.
  EXPECT_EQ(f.retreat_speed, f.leg_speed);
  EXPECT_EQ(f.retreat_accel, f.leg_accel);
  const ArmMotion m = KukaMotion(*l, f);
  const Config a{0.0, -1.0, 1.0, 0.0, 0.5, 0.0, 0.0};
  Config b = a, c = a;
  b[1] += 0.01;
  c[1] += 0.03;
  for (double length : {0.10, 0.12, 0.15, 0.25, 0.40}) {
    EXPECT_EQ(RetreatDuration({a, b, c}, length, m, f), LegDuration({a, b, c}, length, m, f));
  }
  f.retreat_speed = 0.5;
  f.retreat_accel = 3.0;
  for (double length : {0.10, 0.12, 0.15, 0.25, 0.40}) {
    EXPECT_NEAR(LegDuration({a, b, c}, length, m, f),
                std::max(1.5 * length / 0.25, std::sqrt(6.0 * length / 1.0)), 1e-12);
    EXPECT_NEAR(RetreatDuration({a, b, c}, length, m, f),
                std::max(1.5 * length / 0.5, std::sqrt(6.0 * length / 3.0)), 1e-12);
  }
  // A 0.15 m retreat: 0.95 s by acceleration at the legs' figures, 0.55 s now.
  EXPECT_NEAR(RetreatDuration({a, b, c}, 0.15, m, f), std::sqrt(0.3), 1e-12);
  EXPECT_EQ(RetreatDuration({a, b}, 0.0, m, f), 0.0);
  // A joint's speed still bounds a retreat.
  Config far = a;
  far[0] += 2.0;
  EXPECT_NEAR(RetreatDuration({a, far}, 0.05, m, f), 1.5 * 2.0 / m.limits.velocity[0], 1e-12);
}

TEST(RetreatLegs, ThePlanFileNamesThemOnlyWhenTheyDiffer) {
  // A committed plan without the figures reads them as the legs' and is
  // written again byte for byte.
  const std::string text = ReadFile(Env("PLAN_DIR", "plans") + "/" + kKr20 + ".json");
  std::string error;
  std::optional<Plan> p = PlanFromJson(text, &error);
  ASSERT_TRUE(p.has_value()) << error;
  EXPECT_EQ(text.find("retreat_speed"), std::string::npos);
  EXPECT_EQ(p->figures.retreat_speed, p->figures.leg_speed);
  EXPECT_EQ(p->figures.retreat_accel, p->figures.leg_accel);
  EXPECT_EQ(PlanToJson(*p), text);
  p->figures.retreat_speed = 0.5;
  p->figures.retreat_accel = 3.0;
  const std::string fast = PlanToJson(*p);
  EXPECT_NE(fast.find("\"retreat_speed\":0.5,\"retreat_accel\":3,"), std::string::npos);
  const std::optional<Plan> back = PlanFromJson(fast, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(back->figures.retreat_speed, 0.5);
  EXPECT_EQ(back->figures.retreat_accel, 3.0);
  EXPECT_EQ(back->figures.leg_speed, 0.25);
  EXPECT_EQ(PlanToJson(*back), fast);
}

bool AnyFault(const VerifyReport& r, const std::string& words) {
  for (const std::string& f : r.faults) {
    if (f.find(words) != std::string::npos) return true;
  }
  return false;
}

std::string Faults(const VerifyReport& r) {
  std::string s;
  for (const std::string& f : r.faults) s += "  " + f + "\n";
  return s;
}

// The verifier holds the plan's retreat figures to the layout's, on the
// committed one-bay plan of the KR 20 layout (the quickest to check).
TEST(RetreatLegs, TheVerifierHoldsThePlanToTheLayout) {
  std::string error;
  const std::optional<Layout> l = LoadLayout(LayoutPath(kKr20), &error);
  ASSERT_TRUE(l.has_value()) << error;
  const std::optional<Plan> plan =
      PlanFromJson(ReadFile(Env("PLAN_DIR", "plans") + "/" + kKr20 + ".json"), &error);
  ASSERT_TRUE(plan.has_value()) << error;
  const std::string sha = Sha256OfFile(LayoutPath(kKr20));
  VerifyOptions quick;
  quick.schedule = false;
  quick.edges = false;
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const CheckReport report = CheckLayout(*l, paths);
  // As committed: it passes.
  const VerifyReport ok = VerifyPlanWithReport(*l, paths, *plan, sha, report, quick);
  EXPECT_TRUE(ok.ok) << Faults(ok);
  // A plan that brings faster retreats to a layout without the field.
  Plan faster = *plan;
  faster.figures.retreat_speed = 0.5;
  faster.figures.retreat_accel = 3.0;
  const VerifyReport r1 = VerifyPlanWithReport(*l, paths, faster, sha, report, quick);
  EXPECT_FALSE(r1.ok);
  EXPECT_TRUE(AnyFault(r1, "the plan's figure retreat_speed is 0.5, not ours, 0.25")) << Faults(r1);
  EXPECT_TRUE(AnyFault(r1, "the plan's figure retreat_accel is 3, not ours, 1")) << Faults(r1);
  // And the retreats, timed at the leg's figures, are not what those give.
  EXPECT_TRUE(AnyFault(r1, "the retreat's law")) << Faults(r1);
  // A layout with the field and a plan without it.
  Layout with = *l;
  with.retreat_legs = RetreatLegs{0.5, 3.0, "test"};
  const VerifyReport r2 = VerifyPlanWithReport(with, paths, *plan, sha, report, quick);
  EXPECT_FALSE(r2.ok);
  EXPECT_TRUE(AnyFault(r2, "the plan's figure retreat_speed is 0.25, not ours, 0.5")) << Faults(r2);
  // Both agree but the segments were timed at the legs' figures.
  const VerifyReport r3 = VerifyPlanWithReport(with, paths, faster, sha, report, quick);
  EXPECT_FALSE(AnyFault(r3, "retreat_speed")) << Faults(r3);
  EXPECT_TRUE(AnyFault(r3, "the retreat's law")) << Faults(r3);
  // An approach is still timed at leg_speed: no approach is at fault.
  for (const std::string& f : r3.faults) {
    EXPECT_EQ(f.find("approach"), std::string::npos) << f;
  }
}

}  // namespace
}  // namespace cell::layout
