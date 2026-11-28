// The rail magazine filled while the first tube comes in ("first_bay_early",
// src/layout.h, run 059): the magazine layout without the field starts
// every arm of bay 0 after its clamp, as before; the layout with it has a
// committed plan in which the module arm fills its magazine at the rail
// rack during the first tube_in and sets rail 0 at the clamp's end, and
// which passes the verifier; and the same plan against the layout without
// the field is refused, because an arm is off its wait pose during the
// tube_in.

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

constexpr const char* kRailMag =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag";
constexpr const char* kEarly =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag_early";

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

TEST(FirstBayEarly, ALayoutWithoutTheFieldStartsBayZeroAfterItsClamp) {
  const Named o = Load(kRailMag);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_FALSE(o.layout.first_bay_early.has_value());
  const BayEvent* clamp = Event(o.plan, "clamp", 0);
  ASSERT_NE(clamp, nullptr);
  const StepTime* take = Step(o.plan, "take rail 0", 0);
  ASSERT_NE(take, nullptr);
  EXPECT_GE(take->start, clamp->t1 - 1e-6);
}

TEST(FirstBayEarly, TheMagazineFilledWhileTheFirstTubeComesInPassesTheVerifier) {
  const Named o = Load(kEarly);
  ASSERT_TRUE(o.error.empty()) << o.error;
  ASSERT_TRUE(o.layout.first_bay_early.has_value());
  EXPECT_EQ(o.layout.first_bay_early->basis.rfind("ours", 0), 0U);
  EXPECT_TRUE(o.plan.complete);
  const BayEvent* in = Event(o.plan, "tube_in", 0);
  const BayEvent* clamp = Event(o.plan, "clamp", 0);
  ASSERT_NE(in, nullptr);
  ASSERT_NE(clamp, nullptr);
  const StepTime* take = Step(o.plan, "take rail 0", 0);
  const StepTime* set = Step(o.plan, "set rail 0", 0);
  ASSERT_NE(take, nullptr);
  ASSERT_NE(set, nullptr);
  // The fill runs during the tube_in; the first set, which touches the
  // tube, waits for the clamp.
  EXPECT_LT(take->start, in->t1);
  EXPECT_LE(take->end, clamp->t1 + 1e-6);
  EXPECT_GE(set->start, clamp->t1 - 1e-6);
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  const CycleFigures c = CycleFiguresOf(o.plan);
  std::printf("one bay %.2f s (rail_mag 442.03 s)\n", c.one_bay_s);
  EXPECT_LT(c.one_bay_s, 442.03 - 8.0);
}

TEST(FirstBayEarly, TheSamePlanWithoutTheFieldIsRefused) {
  Named o = Load(kEarly);
  ASSERT_TRUE(o.error.empty()) << o.error;
  o.layout.first_bay_early.reset();
  VerifyOptions quick;
  quick.schedule = false;
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, quick);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (const std::string& f : r.faults) {
    if (f.find("module_arm is not at its wait pose") != std::string::npos &&
        f.find("during tube_in") != std::string::npos) {
      found = true;
    }
  }
  EXPECT_TRUE(found) << Faults(r);
}

}  // namespace
}  // namespace cell::layout
