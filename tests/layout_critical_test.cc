// The critical chains of src/layout_critical.h on two committed plans. On
// the small one, flat_rack_1_kr20_pin_arm_ordered (one bay, the module arm
// and one pin arm), the chain is known: the tube comes in and is clamped,
// the module arm takes and sets the ten rails back to back (waiting 0.75 s
// for pin_arm_a before taking rail 9), pin_arm_a bolts rail 9 once its set
// is done, waits 16.00 s for the module arm before its tool change, and then
// works every hole to swage 8/0, which ends at the release. Every link's
// times join the one before with no gap but its named wait, and where the
// scheduler wrote that a block waited for a step, the chain names the same
// step. On the infeed layout's plan (run 053) bay 2's chain spans
// interval_s, 451.90 s. Part of //:plan_replay_test.

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_critical.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {
namespace {

constexpr double kTol = 1e-6;

std::string Env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

struct Chains {
  std::string error;
  Layout layout;
  Plan plan;
  std::vector<CriticalChain> chains;
};

Chains Load(const std::string& name) {
  Chains c;
  const std::optional<Layout> layout =
      LoadLayout(Env("LAYOUT_DIR", "layouts") + "/" + name + ".json", &c.error);
  if (!layout) return c;
  c.layout = *layout;
  std::ifstream in(Env("PLAN_DIR", "plans") + "/" + name + ".json", std::ios::binary);
  std::stringstream text;
  text << in.rdbuf();
  const std::optional<Plan> plan = PlanFromJson(text.str(), &c.error);
  if (!plan) return c;
  c.plan = *plan;
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const CheckReport check = CheckLayout(c.layout, paths);
  const std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(c.layout, paths, check.joint_margin_deg, &c.error);
  if (!scene) return c;
  const PlanPlayer player(*scene, check, c.plan);
  c.chains = CriticalChains(c.layout, check, *scene, player, &c.error);
  return c;
}

const Chains& Small() {
  static const Chains* c = new Chains(Load("flat_rack_1_kr20_pin_arm_ordered"));
  return *c;
}

const Chains& Infeed() {
  static const Chains* c = new Chains(Load(
      "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed"));
  return *c;
}

// The links join: each starts where the one before hands over plus its
// wait, the chain's seconds are its span, and a block's split adds up.
void ExpectJoined(const CriticalChain& c) {
  ASSERT_FALSE(c.links.empty());
  double at = c.from_s, total = 0.0;
  for (const ChainLink& l : c.links) {
    EXPECT_NEAR(l.start - l.wait_s, at, kTol) << l.what;
    EXPECT_GE(l.wait_s, 0.0) << l.what;
    EXPECT_NEAR(l.way_s + l.hold_s, l.wait_s, kTol) << l.what;
    if (l.wait_s > 0.0) {
      EXPECT_FALSE(l.wait_cause.empty()) << l.what;
    }
    EXPECT_GE(l.end, l.start - kTol) << l.what;
    EXPECT_NEAR(l.motion_s + l.work_s + l.inner_wait_s, l.end - l.start, kTol) << l.what;
    total += l.wait_s + (l.end - l.start);
    at = l.end;
  }
  EXPECT_NEAR(at, c.to_s, kTol);
  EXPECT_NEAR(total, c.to_s - c.from_s, kTol);
  EXPECT_EQ(c.links.back().what, "release " + std::to_string(c.bay));
}

TEST(LayoutCritical, SmallPlansChainIsKnown) {
  const Chains& s = Small();
  ASSERT_EQ(s.chains.size(), 1u) << s.error;
  const CriticalChain& c = s.chains[0];
  ExpectJoined(c);
  EXPECT_NEAR(c.from_s, 0.0, kTol);
  EXPECT_NEAR(c.to_s, 842.28, 0.01);
  ASSERT_EQ(c.links.size(), 134u);
  EXPECT_EQ(c.links[0].what, "tube_in 0");
  EXPECT_EQ(c.links[1].what, "clamp 0");
  EXPECT_EQ(c.links[2].what, "take rail 0");
  EXPECT_EQ(c.links[2].cause, "clamp");
  // The ten rails, back to back on the module arm.
  for (int k = 3; k < 22; ++k) {
    EXPECT_EQ(c.links[k].arm, "module_arm") << k;
    EXPECT_EQ(c.links[k].cause, "arm") << k;
  }
  EXPECT_EQ(c.links[20].what, "take rail 9");
  EXPECT_EQ(c.links[20].wait_cause, "way");
  EXPECT_EQ(c.links[20].by, "pin_arm_a");
  EXPECT_NEAR(c.links[20].wait_s, 0.75, kTol);
  EXPECT_EQ(c.links[22].what, "bolt rail 9");
  EXPECT_EQ(c.links[22].arm, "pin_arm_a");
  EXPECT_EQ(c.links[22].cause, "need");
  EXPECT_NE(c.links[22].text.find("step 22 (set rail 9)"), std::string::npos) << c.links[22].text;
  EXPECT_EQ(c.links[23].what, "tool change");
  EXPECT_EQ(c.links[23].wait_cause, "way");
  EXPECT_EQ(c.links[23].by, "module_arm");
  EXPECT_NEAR(c.links[23].wait_s, 16.0, kTol);
  for (size_t k = 22; k + 1 < c.links.size(); ++k) EXPECT_EQ(c.links[k].arm, "pin_arm_a") << k;
  EXPECT_EQ(c.links[c.links.size() - 2].what, "swage 8/0");
  // What each arm spends on it.
  ASSERT_EQ(c.arms.size(), 3u);
  EXPECT_EQ(c.arms[1].arm, "module_arm");
  EXPECT_NEAR(c.arms[1].wait_s, 0.75, kTol);
  EXPECT_EQ(c.arms[2].arm, "pin_arm_a");
  EXPECT_NEAR(c.arms[2].wait_s, 16.0, kTol);
  EXPECT_NEAR(c.arms[0].work_s, 50.0, kTol);  // tube_in, clamp, release.
  // The other arm's end: the module arm's last set, long before.
  ASSERT_EQ(c.next.size(), 1u);
  EXPECT_EQ(c.next[0].arm, "module_arm");
  EXPECT_EQ(c.next[0].what, "set module 8");
}

// Where the scheduler wrote "for step N" on the wait just before a block
// on a chain that starts at its ready time, the chain names step N (by
// what it does).
int CheckNeeds(const Chains& s) {
  int checked = 0;
  for (const CriticalChain& c : s.chains) {
    for (const ChainLink& l : c.links) {
      if (l.event || l.wait_s > 0.0) continue;
      for (const ArmPlan& a : s.plan.arms) {
        if (a.name != l.arm) continue;
        for (const Segment& g : a.segments) {
          if (g.kind != "wait" || std::abs(g.t1 - l.start) > kTol) continue;
          if (g.reason.rfind("for step ", 0) != 0) continue;
          // A later bay's reasons carry bay 0's step numbers (the bays
          // are planned alone and joined), so the step is matched by what
          // it does.
          const size_t open = g.reason.find('(');
          const std::string named = g.reason.substr(open, g.reason.find(')') - open + 1);
          EXPECT_EQ(l.cause, "need") << l.what << ": " << g.reason;
          EXPECT_NE(l.text.find(named), std::string::npos) << l.text << " / " << g.reason;
          ++checked;
        }
      }
    }
  }
  return checked;
}

TEST(LayoutCritical, AgreesWithTheSchedulersNeeds) {
  ASSERT_EQ(Small().chains.size(), 1u) << Small().error;
  ASSERT_EQ(Infeed().chains.size(), 3u) << Infeed().error;
  EXPECT_GE(CheckNeeds(Small()) + CheckNeeds(Infeed()), 3);
}

TEST(LayoutCritical, InfeedBayTwoSpansTheInterval) {
  const Chains& s = Infeed();
  ASSERT_EQ(s.chains.size(), 3u) << s.error;
  for (const CriticalChain& c : s.chains) ExpectJoined(c);
  std::vector<double> outs;
  for (const BayEvent& e : s.plan.events) {
    if (e.kind == "bay_out") outs.push_back(e.t1);
  }
  ASSERT_EQ(outs.size(), 3u);
  // Bay out follows each release at once here, so a later bay's chain
  // spans the time between two bay outs' ends: bay 2's is interval_s.
  EXPECT_NEAR(s.chains[2].to_s - s.chains[2].from_s, outs[2] - outs[1] + 0.0, 1e-3);
  EXPECT_NEAR(outs[2] - outs[1], 451.90, 0.01);
  EXPECT_NEAR(s.chains[1].to_s - s.chains[1].from_s, outs[1] - outs[0], 1e-3);
  for (const CriticalChain& c : s.chains) {
    EXPECT_EQ(c.links[c.links.size() - 2].arm, "pin_arm_c") << c.bay;
    EXPECT_EQ(c.links[c.links.size() - 2].what, "swage 8/0") << c.bay;
    ASSERT_FALSE(c.next.empty());
    EXPECT_EQ(c.next[0].arm, "pin_arm_b") << c.bay;
  }
}

}  // namespace
}  // namespace cell::layout
