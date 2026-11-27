// A layout's torque_limit_scale (src/layout.h, src/layout_torque.h, run 045):
// inverse dynamics of the module arm along each timed move, and the moves
// that demand more than the fraction of a motor's limit played slower. The
// method against the physics record of v16 (steps 77 and 78), the time
// scale, the plan file and the verifier, and the committed plan of the
// layout with the field: no module arm move above its fraction. Without the
// field nothing changes: no figure and no time_scale in the plan file, and
// a plan that brings one is refused.

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
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
#include "src/layout_torque.h"

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

std::string PlanPath(const std::string& name) {
  return Env("PLAN_DIR", "plans") + "/" + name + ".json";
}

constexpr const char* kV16 = "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat";
constexpr const char* kTorque = "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_torque";
constexpr const char* kKr20 = "flat_rack_1_kr20_pin_arm_ordered";
constexpr const char* kSplit = "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split";
constexpr const char* kSplitTorque =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_torque";

// A committed plan with what it takes to measure its module arm.
struct Measured {
  std::string error;
  std::optional<Layout> layout;
  std::optional<Plan> plan;
  std::unique_ptr<LayoutScene> scene;
  ArmMotion motion;
  std::unique_ptr<ArmDynamics> dynamics;
  const ArmPlan* arm = nullptr;
  std::map<int, int> bay_of_step;
};

std::unique_ptr<Measured> Measure(const std::string& name) {
  auto m = std::make_unique<Measured>();
  m->layout = LoadLayout(LayoutPath(name), &m->error);
  if (!m->layout) return m;
  m->plan = PlanFromJson(ReadFile(PlanPath(name)), &m->error);
  if (!m->plan) return m;
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const CheckReport report = CheckLayout(*m->layout, paths);
  m->scene = LayoutScene::Load(*m->layout, paths, report.joint_margin_deg, &m->error);
  if (!m->scene) return m;
  const std::string arm = ModuleArm(*m->layout);
  m->motion = MotionOf(*m->scene, m->scene->ArmIndex(arm), m->plan->figures);
  m->dynamics = ArmDynamics::Create(*m->layout, paths, arm, &m->error);
  for (const ArmPlan& a : m->plan->arms) {
    if (a.name == arm) m->arm = &a;
  }
  if (m->arm == nullptr) m->error = "no module arm in the plan";
  for (const StepTime& s : m->plan->steps) m->bay_of_step[s.number] = s.bay;
  return m;
}

// The peak over every segment of a step, as the physics record's steps.
TorqueDemand StepDemand(Measured& m, int step) {
  TorqueDemand out;
  for (const Segment& s : m.arm->segments) {
    if (s.step != step || s.kind == "wait") continue;
    TorqueDemand d;
    std::string error;
    EXPECT_TRUE(m.dynamics->AlongSegment(s, m.motion, m.plan->figures, &d, &error)) << error;
    for (int j = 0; j < 7; ++j) out.ratio[j] = std::max(out.ratio[j], d.ratio[j]);
  }
  return out;
}

const Segment* MoveOfStep(const Measured& m, int step) {
  for (const Segment& s : m.arm->segments) {
    if (s.step == step && s.kind == "move") return &s;
  }
  return nullptr;
}

std::optional<Layout> WithScale(const nlohmann::json& scale, std::string* error) {
  nlohmann::json j = nlohmann::json::parse(ReadFile(LayoutPath(kV16)));
  if (!scale.is_null()) j["torque_limit_scale"] = scale;
  const std::string path = Env("TEST_TMPDIR", ".") + "/" + kV16 + ".json";
  std::ofstream(path) << j.dump(2);
  return LoadLayout(path, error);
}

TEST(TorqueLimit, TheFieldIsReadAndBounded) {
  std::string error;
  const std::optional<Layout> v16 = LoadLayout(LayoutPath(kV16), &error);
  ASSERT_TRUE(v16.has_value()) << error;
  EXPECT_FALSE(v16->torque_limit_scale.has_value());
  const std::optional<Layout> torque = LoadLayout(LayoutPath(kTorque), &error);
  ASSERT_TRUE(torque.has_value()) << error;
  ASSERT_TRUE(torque->torque_limit_scale.has_value());
  for (double ok : {0.5, 0.95, 1.0}) {
    const std::optional<Layout> l = WithScale(ok, &error);
    ASSERT_TRUE(l.has_value()) << error;
    EXPECT_EQ(*l->torque_limit_scale, ok);
  }
  for (const nlohmann::json& bad : {nlohmann::json(0.4), nlohmann::json(1.01),
                                    nlohmann::json(0.0), nlohmann::json("0.9")}) {
    error.clear();
    EXPECT_FALSE(WithScale(bad, &error).has_value()) << bad;
    EXPECT_NE(error.find("torque_limit_scale"), std::string::npos) << error;
  }
}

// The method against physics: v16's record of module_take 3 (physics/
// flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat.json, steps 77 and
// 78) has joint 1 at its limit, 1.000, in both, and the other axes at the
// figures below. Physics adds the servos' feedback to what the plan's own
// motion asks; inverse dynamics of the planned motion alone is within 0.05
// of the record on every axis that does not saturate, and at least 0.9 on
// joint 1.
TEST(TorqueLimit, InverseDynamicsAgreesWithThePhysicsRecord) {
  std::unique_ptr<Measured> m = Measure(kV16);
  ASSERT_TRUE(m->error.empty()) << m->error;
  const std::map<int, std::array<double, 7>> record = {
      {77, {1.000, 0.815, 0.332, 0.241, 0.298, 0.122, 0.939}},
      {78, {1.000, 0.513, 0.664, 0.491, 0.509, 0.450, 0.967}}};
  for (const auto& [step, want] : record) {
    const TorqueDemand d = StepDemand(*m, step);
    EXPECT_GE(d.ratio[0], 0.9) << "step " << step;
    for (int j = 1; j < 7; ++j) {
      EXPECT_NEAR(d.ratio[j], want[j], 0.05) << "step " << step << " axis " << j;
    }
  }
}

// The time scale: a move over the fraction is slowed until it holds, one
// within it is left alone, and the demand falls as the move slows.
TEST(TorqueLimit, TheTimeScaleSlowsOnlyAMoveOverTheFraction) {
  std::unique_ptr<Measured> m = Measure(kV16);
  ASSERT_TRUE(m->error.empty()) << m->error;
  const Segment* hard = MoveOfStep(*m, 33);   // To module_take 0: joint 1 over 1.2.
  const Segment* easy = MoveOfStep(*m, 166);  // Back to wait: everything under 0.6.
  ASSERT_NE(hard, nullptr);
  ASSERT_NE(easy, nullptr);
  std::string error;
  for (const Segment* s : {hard, easy}) {
    const std::optional<Trajectory> t = MoveTrajectory(s->path, m->motion, m->plan->figures);
    ASSERT_TRUE(t.has_value());
    TorqueDemand at1, at2, scaled;
    ASSERT_TRUE(m->dynamics->AlongMove(*t, 1.0, s->tool, s->held, &at1, &error)) << error;
    ASSERT_TRUE(m->dynamics->AlongMove(*t, 1.5, s->tool, s->held, &at2, &error)) << error;
    EXPECT_LT(at2.Worst(), at1.Worst());
    const double k = TorqueTimeScale(*m->dynamics, *t, s->tool, s->held, 0.95, &scaled, &error);
    if (s == hard) {
      EXPECT_GT(at1.Worst(), 1.2);
      EXPECT_GT(k, 1.0);
      EXPECT_LT(k, 1.5);
      EXPECT_LE(scaled.Worst(), 0.95);
    } else {
      EXPECT_LT(at1.Worst(), 0.6);
      EXPECT_EQ(k, 1.0);
      EXPECT_EQ(scaled.Worst(), at1.Worst());
    }
  }
}

// A move at time scale k is the same path k times slower.
TEST(TorqueLimit, AMoveAtItsTimeScaleIsTheSamePathSlower) {
  std::unique_ptr<Measured> m = Measure(kV16);
  ASSERT_TRUE(m->error.empty()) << m->error;
  Segment s = *MoveOfStep(*m, 33);
  const double span = s.planned_s;
  Segment slow = s;
  slow.time_scale = 1.25;
  slow.t1 = s.t0 + (s.t1 - s.t0 - span) + 1.25 * span;
  for (double u : {0.0, 0.1, 0.37, 0.5, 0.9, 1.0}) {
    const Config a = ConfigAt(s, m->motion, m->plan->figures, s.t0 + u * span);
    const Config b = ConfigAt(slow, m->motion, m->plan->figures, s.t0 + 1.25 * u * span);
    for (int j = 0; j < 7; ++j) EXPECT_NEAR(a[j], b[j], 1e-12) << u << " " << j;
  }
}

TEST(TorqueLimit, ThePlanFileNamesItOnlyWhenSet) {
  const std::string text = ReadFile(PlanPath(kKr20));
  std::string error;
  std::optional<Plan> p = PlanFromJson(text, &error);
  ASSERT_TRUE(p.has_value()) << error;
  EXPECT_EQ(text.find("torque_limit_scale"), std::string::npos);
  EXPECT_EQ(text.find("time_scale"), std::string::npos);
  EXPECT_EQ(p->figures.torque_limit_scale, 0.0);
  EXPECT_EQ(PlanToJson(*p), text);
  p->figures.torque_limit_scale = 0.95;
  Segment* move = nullptr;
  for (Segment& s : p->arms[0].segments) {
    if (s.kind == "move" && move == nullptr) move = &s;
  }
  ASSERT_NE(move, nullptr);
  move->time_scale = 1.1049;
  const std::string scaled = PlanToJson(*p);
  EXPECT_NE(scaled.find(",\"torque_limit_scale\":0.95,"), std::string::npos);
  EXPECT_NE(scaled.find(",\"time_scale\":1.104900,"), std::string::npos);
  const std::optional<Plan> back = PlanFromJson(scaled, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(back->figures.torque_limit_scale, 0.95);
  EXPECT_EQ(PlanToJson(*back), scaled);
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

// The verifier holds the figure to the layout's and refuses a slowed move
// in a plan without the figure (the KR 20 layout's one bay, the quickest).
TEST(TorqueLimit, TheVerifierHoldsThePlanToTheLayout) {
  std::string error;
  const std::optional<Layout> l = LoadLayout(LayoutPath(kKr20), &error);
  ASSERT_TRUE(l.has_value()) << error;
  const std::optional<Plan> plan = PlanFromJson(ReadFile(PlanPath(kKr20)), &error);
  ASSERT_TRUE(plan.has_value()) << error;
  const std::string sha = Sha256OfFile(LayoutPath(kKr20));
  VerifyOptions quick;
  quick.schedule = false;
  quick.edges = false;
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const CheckReport report = CheckLayout(*l, paths);
  const VerifyReport ok = VerifyPlanWithReport(*l, paths, *plan, sha, report, quick);
  EXPECT_TRUE(ok.ok) << Faults(ok);
  Plan figured = *plan;
  figured.figures.torque_limit_scale = 0.95;
  const VerifyReport r1 = VerifyPlanWithReport(*l, paths, figured, sha, report, quick);
  EXPECT_TRUE(AnyFault(r1, "the plan's figure torque_limit_scale is 0.95, not ours, 0"))
      << Faults(r1);
  Layout with = *l;
  with.torque_limit_scale = 0.95;
  const VerifyReport r2 = VerifyPlanWithReport(with, paths, *plan, sha, report, quick);
  EXPECT_TRUE(AnyFault(r2, "the plan's figure torque_limit_scale is 0, not ours, 0.95"))
      << Faults(r2);
  const VerifyReport r3 = VerifyPlanWithReport(with, paths, figured, sha, report, quick);
  EXPECT_TRUE(r3.ok) << Faults(r3);
  // A slowed move without the figure, and one sped up.
  for (double k : {1.2, 0.8}) {
    Plan slowed = *plan;
    for (ArmPlan& a : slowed.arms) {
      for (Segment& s : a.segments) {
        if (s.kind == "move" && !s.path.empty()) s.time_scale = k;
      }
    }
    const VerifyReport r = VerifyPlanWithReport(*l, paths, slowed, sha, report, quick);
    EXPECT_TRUE(AnyFault(r, "time_scale")) << k << "\n" << Faults(r);
  }
}

// The committed plans of the layouts with the field, R1's table run on
// them: every module arm move within the fraction, and some slowed.
class CommittedTorquePlan : public ::testing::TestWithParam<const char*> {};

TEST_P(CommittedTorquePlan, HoldsItsFraction) {
  std::unique_ptr<Measured> m = Measure(GetParam());
  ASSERT_TRUE(m->error.empty()) << m->error;
  const double fraction = m->plan->figures.torque_limit_scale;
  ASSERT_GT(fraction, 0.0);
  EXPECT_EQ(fraction, *m->layout->torque_limit_scale);
  int moves = 0, slowed = 0;
  for (const Segment& s : m->arm->segments) {
    if (s.kind != "move" || s.path.empty()) continue;
    ++moves;
    if (s.time_scale > 1.0) ++slowed;
    TorqueDemand d;
    std::string error;
    ASSERT_TRUE(m->dynamics->AlongSegment(s, m->motion, m->plan->figures, &d, &error)) << error;
    EXPECT_LE(d.Worst(), fraction) << "step " << s.step;
  }
  EXPECT_GT(moves, 30);
  EXPECT_GT(slowed, 0);
}

INSTANTIATE_TEST_SUITE_P(TorqueLimit, CommittedTorquePlan,
                         ::testing::Values(kTorque, kSplitTorque));

// The seconds the slowed moves of the module arm add to its motion: each
// slowed move's time without its settle, less the same at time scale 1.
double AddedBySlowing(const ArmPlan& arm, double settle_s) {
  double added = 0.0;
  for (const Segment& s : arm.segments) {
    if (s.kind != "move" || s.time_scale <= 1.0) continue;
    const double motion = s.t1 - s.t0 - settle_s;
    added += motion - motion / s.time_scale;
  }
  return added;
}

// Run 047: the schedule of a plan with slowed moves keeps the order and the
// starts of the plan without them (ScheduleSlowed in src/layout_plan.cc), so
// the bay costs no more than the layout without the field plus the motion
// the slowed moves add; before run 047 v16's bay grew 26.60 s for 5.80 s.
TEST(TorqueLimit, TheBayCostsNoMoreThanTheSlowedMotion) {
  const std::pair<const char*, const char*> pairs[] = {{kV16, kTorque}, {kSplit, kSplitTorque}};
  for (const auto& [without, with] : pairs) {
    std::string error;
    const std::optional<Plan> base = PlanFromJson(ReadFile(PlanPath(without)), &error);
    ASSERT_TRUE(base) << error;
    std::unique_ptr<Measured> m = Measure(with);
    ASSERT_TRUE(m->error.empty()) << m->error;
    const double added = AddedBySlowing(*m->arm, m->plan->figures.settle_s);
    double base_bay0 = 0.0;
    for (const StepTime& st : base->steps) {
      if (st.bay == 0) base_bay0 = std::max(base_bay0, st.end);
    }
    EXPECT_GT(added, 1.0) << with;
    EXPECT_GT(m->plan->bay_planned_s, base_bay0) << with;
    EXPECT_LE(m->plan->bay_planned_s, base_bay0 + added + 0.01) << with << ": " << base_bay0
                                                                 << " + " << added;
  }
}

}  // namespace
}  // namespace cell::layout
