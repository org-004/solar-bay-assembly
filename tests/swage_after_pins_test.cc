// Two pin arms swage one module ("swage_after_pins", src/layout.h, run 070).
// Without the field each hole step of a module waits for the module's hole
// step before it, whichever arm does it, so a swage by one arm waits for the
// other arm's swage before it. With it a swage waits only for the module's
// set and its last pin. The tests give pin arm A module 3's swage 3/0 (or
// 3/1 and 3/0) on v24's layout, B keeps the others, and plan steps 49 to 54
// (locate and pin 3/1, then swages 3/1, 3/3, 3/2, 3/0) under both rules. v24's committed plan, without the field, is unchanged
// (plan_replay_test).

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"

namespace cell::layout {
namespace {

constexpr const char* kV24 =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag_early_next_fill_pallet_near";

std::string LayoutFile(const std::string& name) {
  const char* dir = std::getenv("LAYOUT_DIR");
  return std::string(dir != nullptr ? dir : "layouts") + "/" + name + ".json";
}

// v24 with A swaging holes 12 and 13 (3/0, 3/1), or 12 alone, from B's stops (the three pin
// arms share one beam, base and orientation), written to a file and loaded,
// with or without the field (tools/r1_070/make.py's a31).
Layout Split(bool swage_after_pins, bool both = true) {
  std::ifstream in(LayoutFile(kV24), std::ios::binary);
  nlohmann::json j = nlohmann::json::parse(in);
  EXPECT_FALSE(j.contains("swage_after_pins"));
  nlohmann::json* a = nullptr;
  nlohmann::json* b = nullptr;
  for (nlohmann::json& u : j["arms"]) {
    if (u["name"] == "pin_arm_a") a = &u;
    if (u["name"] == "pin_arm_b") b = &u;
  }
  EXPECT_TRUE(a != nullptr && b != nullptr);
  if (a == nullptr || b == nullptr) return {};
  for (int h : {12, 13}) {
    if (h == 13 && !both) continue;
    (*a)["stops"]["swage"][h] = (*b)["stops"]["swage"][h];
  }
  for (nlohmann::json& w : j["work"]) {
    if (w["kind"] != "swage") continue;
    if (w["arm"] == "pin_arm_a") w["points"] = both ? "0-13" : "0-12";
    if (w["arm"] == "pin_arm_b") w["points"] = both ? "14-23" : "13-23";
  }
  j["swage_after_pins"] = swage_after_pins;
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string path = std::string(tmp != nullptr ? tmp : "/tmp") + "/" + kV24 + ".json";
  std::ofstream(path, std::ios::binary) << j.dump(2);
  std::string error;
  std::optional<Layout> l = LoadLayout(path, &error);
  EXPECT_TRUE(l.has_value()) << error;
  if (!l) return {};
  EXPECT_EQ(l->swage_after_pins, swage_after_pins);
  return *l;
}

struct Times {
  double start = -1.0, end = -1.0;  // Of the step's block: its move, then its work.
  double work = -1.0;               // The start of its work.
  std::string waited;               // The reason of a wait before it.
};

// The first and last segment of `step` on `arm`, and a wait just before it.
Times Of(const Plan& p, const std::string& arm, int step) {
  Times t;
  for (const ArmPlan& a : p.arms) {
    if (a.name != arm) continue;
    for (const Segment& s : a.segments) {
      if (s.step != step) continue;
      if (s.kind == "wait") {
        t.waited = s.reason;
        continue;
      }
      if (t.start < 0.0) t.start = s.t0;
      if (s.kind == "work" && t.work < 0.0) t.work = s.t0;
      t.end = s.t1;
    }
  }
  return t;
}

PlanResult PlanSwages(const Layout& layout) {
  PlanOptions o;
  o.first_step = 49;
  o.budgets = {5000, 40000, 160000, 640000};
  o.last_step = 54;
  return PlanLayout(layout, ModelPaths::FromEnvironment(), Sha256OfFile(LayoutFile(kV24)), o);
}

TEST(SwageAfterPins, TheFieldIsReadAndOptional) {
  std::string error;
  std::optional<Layout> l = LoadLayout(LayoutFile(kV24), &error);
  ASSERT_TRUE(l.has_value()) << error;
  EXPECT_FALSE(l->swage_after_pins);
}

// A swages 3/0 alone, the module's last swage (run 067's sw30): under the old
// rule it waits for B's swage 3/2; under the new one it waits only for the
// module's last pin, B's pin 3/1, and starts while B still swages.
TEST(SwageAfterPins, ASecondArmsSwageWaitsOnlyUnderTheOldRule) {
  const PlanResult old_rule = PlanSwages(Split(false, false));
  const PlanResult new_rule = PlanSwages(Split(true, false));
  for (const PlanResult* r : {&old_rule, &new_rule}) {
    for (const std::string& f : r->faults) {
      if (f.find("never complete") == std::string::npos) ADD_FAILURE() << f;
    }
  }
  const Times a30_old = Of(old_rule.plan, "pin_arm_a", 54);
  const Times b32_old = Of(old_rule.plan, "pin_arm_b", 53);
  const Times a30_new = Of(new_rule.plan, "pin_arm_a", 54);
  const Times b32_new = Of(new_rule.plan, "pin_arm_b", 53);
  const Times pin31_new = Of(new_rule.plan, "pin_arm_b", 50);
  for (const Times* t : {&a30_old, &b32_old, &a30_new, &b32_new, &pin31_new}) {
    ASSERT_GE(t->start, 0.0);
    ASSERT_GE(t->work, 0.0);
  }
  std::printf("old rule: B swage 3/2 ends %.2f s, A swage 3/0 works from %.2f s (%s)\n",
              b32_old.end, a30_old.work, a30_old.waited.c_str());
  std::printf("new rule: B swage 3/2 ends %.2f s, A swage 3/0 works from %.2f s (%s)\n",
              b32_new.end, a30_new.work, a30_new.waited.c_str());
  // Old rule: A's block starts after B's swage 3/2 has ended, and says so.
  EXPECT_GE(a30_old.start, b32_old.end - 1e-6);
  EXPECT_NE(a30_old.waited.find("swage 3/2"), std::string::npos) << a30_old.waited;
  // New rule: A swages 3/0 before B's swage 3/2 has ended, after the last pin.
  EXPECT_LT(a30_new.work, b32_new.end);
  EXPECT_GE(a30_new.start, pin31_new.end - 1e-6);
  EXPECT_EQ(a30_new.waited.find("swage"), std::string::npos) << a30_new.waited;
  // B's own steps keep their order.
  EXPECT_GE(b32_new.start, Of(new_rule.plan, "pin_arm_b", 52).end - 1e-6);
}

// A swages 3/1 and 3/0, B 3/3 and 3/2: under the old rule B's swage 3/3
// waits for A's swage 3/1 while B stands where A must go, and the schedule
// blocks (run 067's deadlock); under the new one B swages 3/3 right after its
// pin 3/1 and the part completes its schedule.
TEST(SwageAfterPins, TwoSwagesEachBlockOnlyUnderTheOldRule) {
  const PlanResult old_rule = PlanSwages(Split(false));
  const PlanResult new_rule = PlanSwages(Split(true));
  bool blocked = false;
  for (const std::string& f : old_rule.faults) blocked |= f.find("is blocked") != std::string::npos;
  EXPECT_TRUE(blocked);
  for (const std::string& f : new_rule.faults) {
    if (f.find("never complete") == std::string::npos) ADD_FAILURE() << f;
  }
  const Times a31 = Of(new_rule.plan, "pin_arm_a", 51);
  const Times b33 = Of(new_rule.plan, "pin_arm_b", 52);
  const Times pin31 = Of(new_rule.plan, "pin_arm_b", 50);
  const Times a30 = Of(new_rule.plan, "pin_arm_a", 54);
  ASSERT_GE(a31.end, 0.0);
  ASSERT_GE(b33.start, 0.0);
  ASSERT_GE(a30.start, 0.0);
  std::printf("new rule: A swage 3/1 ends %.2f s, B swage 3/3 starts %.2f s\n", a31.end, b33.start);
  EXPECT_LT(b33.start, a31.end);
  EXPECT_NEAR(b33.start, pin31.end, 1e-6);
  EXPECT_GE(a31.start, pin31.end - 1e-6);
}

}  // namespace
}  // namespace cell::layout
