// The module arm's right of way over a pin arm standing in its way
// ("torque_arm_way", src/layout_torque.h, run 064): in the committed
// three-bay plan of run 061's next_bay_fill layout, bay 1's module arm waits
// 15.50 s before set module 4 while pin arm C stands at module 6's holes
// across its path; the layout with the field has a committed plan in which
// the module arm waits less than 1 s there in every bay, that passes the
// verifier, and whose cycle is shorter than run 061's at no longer an
// interval.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"

namespace cell::layout {
namespace {

constexpr const char* kNextFill =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag_early_next_fill";
constexpr const char* kTorqueWay =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag_early_next_fill_torque_way";

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

// The seconds the module arm waits under the step "set module 4" of `bay`
// (its wait before the set's move), or -1 without that step.
double WaitBeforeSetModule4(const Plan& p, int bay) {
  int number = 0;
  for (const StepTime& s : p.steps) {
    if (s.what == "set module 4" && s.bay == bay) number = s.number;
  }
  if (number == 0) return -1.0;
  double wait = 0.0;
  for (const ArmPlan& row : p.arms) {
    if (row.name != "module_arm") continue;
    for (const Segment& g : row.segments) {
      if (g.kind == "wait" && g.step == number) wait += g.t1 - g.t0;
    }
  }
  return wait;
}

TEST(TorqueArmWay, Run061sBay1WaitsForPinArmCAtSetModule4) {
  const Named o = Load(kNextFill);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_FALSE(o.layout.torque_arm_way.has_value());
  ASSERT_EQ(o.plan.bays, 3);
  EXPECT_NEAR(WaitBeforeSetModule4(o.plan, 1), 15.50, 0.01);
  EXPECT_LT(WaitBeforeSetModule4(o.plan, 2), 1.0);
}

TEST(TorqueArmWay, TheModuleArmDoesNotWaitAtSetModule4AndThePlanVerifies) {
  const Named o = Load(kTorqueWay);
  ASSERT_TRUE(o.error.empty()) << o.error;
  ASSERT_TRUE(o.layout.torque_arm_way.has_value());
  EXPECT_EQ(o.layout.torque_arm_way->rfind("ours", 0), 0U);
  EXPECT_TRUE(o.plan.complete);
  ASSERT_EQ(o.plan.bays, 3);
  for (int b = 0; b < o.plan.bays; ++b) {
    const double wait = WaitBeforeSetModule4(o.plan, b);
    std::printf("bay %d: module arm waits %.2f s at set module 4\n", b, wait);
    EXPECT_GE(wait, 0.0) << "bay " << b;
    EXPECT_LT(wait, 1.0) << "bay " << b;
  }
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  const CycleFigures c = CycleFiguresOf(o.plan);
  std::printf("bays %d: one bay %.2f s, cycle %.2f s, interval %.2f s (run 061: 417.06, "
              "397.86, 378.07 s)\n",
              c.bays, c.one_bay_s, c.cycle_s, c.interval_s);
  EXPECT_LT(c.cycle_s, 397.86 - 2.0);
  EXPECT_LE(c.interval_s, 378.07 + 0.005);
}

}  // namespace
}  // namespace cell::layout
