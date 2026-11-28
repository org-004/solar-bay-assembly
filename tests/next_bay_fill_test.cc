// The next bay's magazine filled after the last set ("next_bay_fill",
// src/layout_bays.h, run 061): the layout with the field has a committed
// plan of several bays in which, in every bay after the first, the module
// arm fills its magazine at the rail rack before the bay before's release,
// while that bay's pin arms still work, and sets rail 0 at the clamp's end;
// the plan passes the verifier; and the same plan against the layout
// without the field is refused, because an arm starts a step of a bay
// before the bay before's bay_out ends.

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

const BayEvent* Event(const Plan& p, const std::string& kind, int bay) {
  for (const BayEvent& e : p.events) {
    if (e.kind == kind && e.bay == bay) return &e;
  }
  return nullptr;
}

const StepTime* Step(const Plan& p, const std::string& what, int bay) {
  for (const StepTime& s : p.steps) {
    if (s.what == what && s.bay == bay) return &s;
  }
  return nullptr;
}

TEST(NextBayFill, TheNextBaysFillBeforeTheReleasePassesTheVerifier) {
  const Named o = Load(kNextFill);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_EQ(o.layout.next_bay_fill, "module_arm");
  ASSERT_TRUE(o.layout.first_bay_early.has_value());
  EXPECT_TRUE(o.plan.complete);
  ASSERT_GE(o.plan.bays, 2);
  for (int b = 1; b < o.plan.bays; ++b) {
    const BayEvent* release = Event(o.plan, "release", b - 1);
    const BayEvent* clamp = Event(o.plan, "clamp", b);
    const StepTime* take = Step(o.plan, "take rail 0", b);
    const StepTime* set = Step(o.plan, "set rail 0", b);
    ASSERT_NE(release, nullptr);
    ASSERT_NE(clamp, nullptr);
    ASSERT_NE(take, nullptr);
    ASSERT_NE(set, nullptr);
    // The fill of bay b ends before bay b-1 is released; the first set,
    // which touches the tube, waits for bay b's clamp.
    EXPECT_LT(take->start, release->t0) << "bay " << b;
    EXPECT_LT(take->end, release->t0) << "bay " << b;
    EXPECT_GE(set->start, clamp->t1 - 1e-6) << "bay " << b;
  }
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  const CycleFigures c = CycleFiguresOf(o.plan);
  std::printf("bays %d: one bay %.2f s, cycle %.2f s, interval %.2f s (rail_mag 396.30 s)\n",
              c.bays, c.one_bay_s, c.cycle_s, c.interval_s);
  EXPECT_LT(c.interval_s, 396.30 - 6.0);
}

TEST(NextBayFill, TheSamePlanWithoutTheFieldIsRefused) {
  Named o = Load(kNextFill);
  ASSERT_TRUE(o.error.empty()) << o.error;
  o.layout.next_bay_fill.clear();
  VerifyOptions quick;
  quick.schedule = false;
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, quick);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (const std::string& f : r.faults) {
    if (f.find("arm module_arm starts its") != std::string::npos &&
        f.find("bay_out ends") != std::string::npos) {
      found = true;
    }
  }
  EXPECT_TRUE(found) << Faults(r);
}

}  // namespace
}  // namespace cell::layout
