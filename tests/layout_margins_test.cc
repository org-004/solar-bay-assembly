// The margins of src/layout_margins.h on v19's committed plan
// (flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque,
// three bays): every arm on a track has a track_end for lo and for hi, the
// three pin arms of the shared beam have a carriage_gap for each pair, and
// the module arm's torque by bay is what //:torque_demand prints one bay at
// a time. The figures are the ones run 051's report gives, to 1e-6. Part
// of //:plan_replay_test.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_margins.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"
#include "src/layout_torque.h"

namespace cell::layout {
namespace {

constexpr char kName[] = "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque";
constexpr double kTol = 1e-6;

std::string Env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

struct V19 {
  std::string error;
  Layout layout;
  Plan plan;
  CheckReport check;
  std::unique_ptr<LayoutScene> scene;
};

const V19& TheV19() {
  static const V19* v = [] {
    auto* v = new V19;
    const std::string layout_path = Env("LAYOUT_DIR", "layouts") + "/" + kName + ".json";
    const std::string plan_path = Env("PLAN_DIR", "plans") + "/" + kName + ".json";
    const std::optional<Layout> layout = LoadLayout(layout_path, &v->error);
    if (!layout) return v;
    v->layout = *layout;
    std::ifstream in(plan_path, std::ios::binary);
    std::stringstream text;
    text << in.rdbuf();
    const std::optional<Plan> plan = PlanFromJson(text.str(), &v->error);
    if (!plan) return v;
    v->plan = *plan;
    const ModelPaths paths = ModelPaths::FromEnvironment();
    v->check = CheckLayout(v->layout, paths);
    v->scene = LayoutScene::Load(v->layout, paths, v->check.joint_margin_deg, &v->error);
    return v;
  }();
  return *v;
}

TEST(LayoutMargins, PinArmsOfTheSharedBeamAreOnOneLine) {
  const V19& v = TheV19();
  ASSERT_TRUE(v.scene) << v.error;
  std::vector<std::pair<std::string, std::string>> names;
  for (const auto& [i, j] : ArmsOnOneLine(v.layout)) {
    names.emplace_back(v.layout.arms[i].name, v.layout.arms[j].name);
  }
  const std::vector<std::pair<std::string, std::string>> want = {
      {"pin_arm_a", "pin_arm_c"}, {"pin_arm_a", "pin_arm_b"}, {"pin_arm_c", "pin_arm_b"}};
  EXPECT_EQ(names, want);
}

TEST(LayoutMargins, TrackEndsAndCarriageGapsOfV19) {
  const V19& v = TheV19();
  ASSERT_TRUE(v.scene) << v.error;
  ASSERT_EQ(v.plan.bays, 3);
  const PlanPlayer player(*v.scene, v.check, v.plan);
  const TrackMargins m = TrackMarginsOf(v.layout, player, 0.01);
  for (const TrackEnd& e : m.ends) {
    std::printf("track_end %s %s slack %.9f value %.9f step %d t %.9f\n", e.arm.c_str(),
                e.end.c_str(), e.slack, e.value, e.step, e.t);
  }
  for (const CarriageGap& g : m.gaps) {
    std::printf("carriage_gap %s %s distance %.9f contact %.9f steps %d %d t %.9f\n",
                g.a.c_str(), g.b.c_str(), g.distance, g.contact, g.step_a, g.step_b, g.t);
  }
  // Every arm on a track, at both ends.
  std::map<std::string, std::pair<double, double>> slack;  // lo, hi.
  std::map<std::string, int> step;
  for (const TrackEnd& e : m.ends) {
    (e.end == "lo" ? slack[e.arm].first : slack[e.arm].second) = e.slack;
    step[e.arm + " " + e.end] = e.step;
  }
  ASSERT_EQ(slack.size(), 4u);
  ASSERT_EQ(m.ends.size(), 8u);
  struct Want {
    const char* arm;
    double lo, hi;
  };
  const Want want[] = {
      {"module_arm", 0.0, 0.0},
      {"pin_arm_a", 0.0, 0.4},
      {"pin_arm_b", 0.271673283, 0.4},
      {"pin_arm_c", 0.4, 0.4},
  };
  for (const Want& w : want) {
    SCOPED_TRACE(w.arm);
    ASSERT_EQ(slack.count(w.arm), 1u);
    EXPECT_NEAR(slack[w.arm].first, w.lo, kTol);
    EXPECT_NEAR(slack[w.arm].second, w.hi, kTol);
  }
  EXPECT_EQ(step["module_arm lo"], 4);
  EXPECT_EQ(step["module_arm hi"], 22);
  EXPECT_EQ(step["pin_arm_b lo"], 223);
  // Tightest first.
  for (size_t k = 1; k < m.ends.size(); ++k) EXPECT_LE(m.ends[k - 1].slack, m.ends[k].slack);

  ASSERT_EQ(m.gaps.size(), 3u);
  struct Gap {
    const char* a;
    const char* b;
    double distance;
    int step_a;
  };
  const Gap gaps[] = {
      {"pin_arm_a", "pin_arm_b", 1.159890914, 468},
      {"pin_arm_c", "pin_arm_b", 1.635349484, 292},
      {"pin_arm_a", "pin_arm_c", 4.297604241, 475},
  };
  for (size_t k = 0; k < 3; ++k) {
    SCOPED_TRACE(gaps[k].a + std::string(" ") + gaps[k].b);
    EXPECT_EQ(m.gaps[k].a, gaps[k].a);
    EXPECT_EQ(m.gaps[k].b, gaps[k].b);
    EXPECT_NEAR(m.gaps[k].distance, gaps[k].distance, kTol);
    EXPECT_NEAR(m.gaps[k].contact, 0.5, kTol);
    EXPECT_EQ(m.gaps[k].step_a, gaps[k].step_a);
  }
}

TEST(LayoutMargins, TorqueByBayOfV19) {
  const V19& v = TheV19();
  ASSERT_TRUE(v.scene) << v.error;
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const std::string arm = ModuleArm(v.layout);
  ASSERT_EQ(arm, "module_arm");
  std::string error;
  std::unique_ptr<ArmDynamics> dynamics = ArmDynamics::Create(v.layout, paths, arm, &error);
  ASSERT_TRUE(dynamics) << error;
  const ArmMotion motion = MotionOf(*v.scene, v.scene->ArmIndex(arm), v.plan.figures);
  ASSERT_NEAR(v.plan.figures.torque_limit_scale, 0.9, 1e-12);
  std::vector<BayTorque> bays;
  ASSERT_TRUE(TorqueByBay(v.plan, dynamics.get(), motion, arm, 0.9, &bays, &error)) << error;
  ASSERT_EQ(bays.size(), 3u);
  const int worst_step[] = {94, 294, 406};
  const int worst_axis[] = {0, 1, 0};
  const double worst[] = {0.899999752, 0.899998970, 0.899994568};
  for (int b = 0; b < 3; ++b) {
    SCOPED_TRACE("bay " + std::to_string(b));
    std::printf("bay %d moves %d over_one %d over_scale %d worst %.9f axis %d step %d\n", b,
                bays[b].moves, bays[b].over_one, bays[b].over_scale, bays[b].worst.Worst(),
                bays[b].worst.WorstAxis(), bays[b].worst_step);
    EXPECT_EQ(bays[b].bay, b);
    EXPECT_EQ(bays[b].moves, 39);
    EXPECT_EQ(bays[b].over_one, 0);
    EXPECT_EQ(bays[b].over_scale, 0);
    EXPECT_NEAR(bays[b].worst.Worst(), worst[b], kTol);
    EXPECT_EQ(bays[b].worst.WorstAxis(), worst_axis[b]);
    EXPECT_EQ(bays[b].worst_step, worst_step[b]);
  }
}

}  // namespace
}  // namespace cell::layout
