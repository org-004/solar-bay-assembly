// The plan's seed ("plan_seed", src/layout.h, run 066): a layout may name the
// seed from which every move's seeds are drawn (PlanFigures::seed), so that
// one layout can be planned from other seeds and the quickest verified plan
// kept. Without the field the seed is 1, as before: a layout without it
// plans its moves from the same seeds as its committed plan. The landed
// layout's committed three-bay plan was planned from its own seed, records
// it, and passes the verifier with its figures; the same plan against the
// layout with another seed is refused.

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"

namespace cell::layout {
namespace {

// v23 (run 063): no plan_seed.
constexpr const char* kV23 =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag_early_next_fill";
// The quickest verified candidate of run 066.
constexpr const char* kLanded = kV23;  // LANDED
constexpr double kV23CycleS = 397.86;

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

std::string LayoutFile(const std::string& name) {
  return Dir("LAYOUT_DIR", "layouts") + "/" + name + ".json";
}

struct Named {
  std::string error;
  Layout layout;
  Plan plan;
  std::string sha256;
};

Named Load(const std::string& name) {
  Named o;
  const std::optional<Layout> l = LoadLayout(LayoutFile(name), &o.error);
  if (!l) return o;
  o.layout = *l;
  const std::optional<Plan> p =
      PlanFromJson(ReadFile(Dir("PLAN_DIR", "plans") + "/" + name + ".json"), &o.error);
  if (!p) return o;
  o.plan = *p;
  o.sha256 = Sha256OfFile(LayoutFile(name));
  return o;
}

std::string Faults(const VerifyReport& r) {
  std::string s;
  for (const std::string& f : r.faults) s += "  " + f + "\n";
  return s;
}

// Each move's seed by arm, step, from and to, the first of each (bay 0).
using MoveKey = std::tuple<std::string, int, std::string, std::string>;
std::map<MoveKey, std::uint32_t> MoveSeeds(const Plan& p) {
  std::map<MoveKey, std::uint32_t> seeds;
  for (const ArmPlan& a : p.arms) {
    for (const Segment& s : a.segments) {
      if (s.kind == "move") seeds.emplace(MoveKey{a.name, s.step, s.from, s.to}, s.seed);
    }
  }
  return seeds;
}

// The layout's text with plan_seed set (after path_tries), loaded from a file.
std::optional<Layout> WithSeed(const std::string& name, const std::string& seed,
                               std::string* error) {
  std::string text = ReadFile(LayoutFile(name));
  const std::string at = "\"path_tries\": 8,";
  const std::size_t i = text.find(at);
  if (i == std::string::npos) {
    *error = "no path_tries in " + name;
    return std::nullopt;
  }
  text.insert(i + at.size(), "\n  \"plan_seed\": " + seed + ",");
  const std::string path = Dir("TEST_TMPDIR", "/tmp") + "/" + name + ".json";
  std::ofstream(path, std::ios::binary) << text;
  return LoadLayout(path, error);
}

// Plans only the bay's first moves, steps 3 to 6 (a part of the bay is quick).
PlanResult PlanFirstSteps(const Layout& layout) {
  PlanOptions o;
  o.first_step = 3;
  o.last_step = 6;
  return PlanLayout(layout, ModelPaths::FromEnvironment(), Sha256OfFile(LayoutFile(kV23)), o);
}

TEST(PlanSeed, TheLandedPlanVerifiesWithItsFigures) {
  const Named o = Load(kLanded);
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_EQ(o.plan.figures.seed, static_cast<std::uint32_t>(o.layout.plan_seed.value_or(1)));
  EXPECT_TRUE(o.plan.complete);
  EXPECT_EQ(o.plan.bays, 3);
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  const CycleFigures c = CycleFiguresOf(o.plan);
  std::printf("seed %u, bays %d: one bay %.2f s, cycle %.2f s, interval %.2f s (v23 cycle %.2f s)\n",
              o.plan.figures.seed, c.bays, c.one_bay_s, c.cycle_s, c.interval_s, kV23CycleS);
  EXPECT_NEAR(c.cycle_s, 397.86, 0.01);
  EXPECT_NEAR(c.interval_s, 378.07, 0.01);
  EXPECT_NEAR(c.one_bay_s, 417.06, 0.01);
  EXPECT_LE(c.cycle_s, kV23CycleS - 3.0);
}

TEST(PlanSeed, TheSamePlanAgainstAnotherSeedIsRefused) {
  Named o = Load(kLanded);
  ASSERT_TRUE(o.error.empty()) << o.error;
  o.layout.plan_seed = o.layout.plan_seed.value_or(1) + 1;
  VerifyOptions quick;
  quick.schedule = false;
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, quick);
  EXPECT_FALSE(r.ok);
  bool found = false;
  for (const std::string& f : r.faults) {
    if (f.find("the plan's figure seed is") != std::string::npos) found = true;
  }
  EXPECT_TRUE(found) << Faults(r);
}

TEST(PlanSeed, ALayoutWithoutTheSeedPlansWithTodaysSeeds) {
  const Named o = Load(kV23);
  ASSERT_TRUE(o.error.empty()) << o.error;
  ASSERT_FALSE(o.layout.plan_seed.has_value());
  EXPECT_EQ(o.plan.figures.seed, 1u);
  const PlanResult part = PlanFirstSteps(o.layout);
  EXPECT_EQ(part.plan.figures.seed, 1u);
  const std::map<MoveKey, std::uint32_t> today = MoveSeeds(o.plan);
  const std::map<MoveKey, std::uint32_t> now = MoveSeeds(part.plan);
  ASSERT_FALSE(now.empty());
  for (const auto& [key, seed] : now) {
    const auto it = today.find(key);
    ASSERT_NE(it, today.end()) << std::get<0>(key) << " step " << std::get<1>(key);
    EXPECT_EQ(seed, it->second) << std::get<0>(key) << " step " << std::get<1>(key);
  }
}

TEST(PlanSeed, ASeedMovesEveryMovesSeed) {
  std::string error;
  const std::optional<Layout> seeded = WithSeed(kV23, "7", &error);
  ASSERT_TRUE(seeded.has_value()) << error;
  ASSERT_EQ(seeded->plan_seed, 7);
  const Named o = Load(kV23);
  ASSERT_TRUE(o.error.empty()) << o.error;
  const PlanResult part = PlanFirstSteps(*seeded);
  EXPECT_EQ(part.plan.figures.seed, 7u);
  const std::map<MoveKey, std::uint32_t> today = MoveSeeds(o.plan);
  const std::map<MoveKey, std::uint32_t> now = MoveSeeds(part.plan);
  ASSERT_FALSE(now.empty());
  for (const auto& [key, seed] : now) {
    const auto it = today.find(key);
    ASSERT_NE(it, today.end());
    EXPECT_NE(seed, it->second) << std::get<0>(key) << " step " << std::get<1>(key);
  }
}

TEST(PlanSeed, ASeedOutOfRangeIsRefused) {
  for (const char* bad : {"1", "0", "-3", "1000001", "2.5"}) {
    std::string error;
    EXPECT_FALSE(WithSeed(kV23, bad, &error).has_value()) << bad;
    EXPECT_NE(error.find("plan_seed"), std::string::npos) << bad << ": " << error;
  }
}

}  // namespace
}  // namespace cell::layout
