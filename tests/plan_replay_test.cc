// The replay of src/layout_plan.h: the motion of a segment, the plan as a
// file, the player, and the committed plans' order of work, verified
// without planning anything (R20). The committed plan of the chosen layout,
// whole, is tests/plan_replay_committed_test.cc's.

#include <cmath>
#include <cstdio>
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
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string LayoutDir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr ? dir : "layouts";
}

std::string PlanDir() {
  const char* dir = std::getenv("PLAN_DIR");
  return dir != nullptr ? dir : "plans";
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

// A layout on the flat rack and its scene, for the tests of the pieces.
struct Small {
  Layout layout;
  std::unique_ptr<LayoutScene> scene;
};

const Small& SmallScene() {
  static const Small* small = [] {
    auto* s = new Small;
    std::string error;
    std::optional<Layout> l =
        LoadLayout(LayoutDir() + "/flat_rack_3_pin_arms.json", &error);
    if (!l) {
      std::fprintf(stderr, "%s\n", error.c_str());
      return s;
    }
    s->layout = *l;
    s->scene = LayoutScene::Load(s->layout, ModelPaths::FromEnvironment(), 5.0, &error);
    if (!s->scene) std::fprintf(stderr, "%s\n", error.c_str());
    return s;
  }();
  return *small;
}

int KukaArm(const LayoutScene& scene) {
  for (int i = 0; i < scene.ArmCount(); ++i) {
    if (scene.arm(i).use->model == "kr210_r2700_2") return i;
  }
  return -1;
}

// ------------------------------------------------------------- motion

TEST(Motion, MotionOfTakesTheCatalogueTheTrackAndTheFigures) {
  const Small& s = SmallScene();
  ASSERT_NE(s.scene, nullptr);
  const int arm = KukaArm(*s.scene);
  ASSERT_GE(arm, 0);
  const SceneArm& a = s.scene->arm(arm);
  PlanFigures f;
  const ArmMotion m = MotionOf(*s.scene, arm, f);
  ASSERT_EQ(m.limits.velocity.size(), 7);
  ASSERT_EQ(m.weights.size(), 7);
  EXPECT_NEAR(m.limits.velocity[0], 120.0 * kPi / 180.0, 1e-12);
  EXPECT_NEAR(m.limits.acceleration[0], m.limits.velocity[0] / f.to_full_speed_s, 1e-12);
  EXPECT_NEAR(m.limits.jerk[0], m.limits.acceleration[0] / f.jerk_rise_s, 1e-9);
  EXPECT_DOUBLE_EQ(m.weights[0], 4.5);
  EXPECT_DOUBLE_EQ(m.weights[5], 1.3);
  EXPECT_DOUBLE_EQ(m.weights[6], 1.0);
  EXPECT_DOUBLE_EQ(m.lower[1], a.lo[1]);
  EXPECT_DOUBLE_EQ(m.upper[1], a.hi[1]);
  if (a.use->track) {
    EXPECT_DOUBLE_EQ(m.limits.velocity[6], a.use->track->speed);
    EXPECT_DOUBLE_EQ(m.limits.acceleration[6], a.use->track->accel);
    EXPECT_DOUBLE_EQ(m.lower[6], a.use->track->lo);
    EXPECT_DOUBLE_EQ(m.upper[6], a.use->track->hi);
  }
}

TEST(Motion, MoveDurationAndWhereTheArmIs) {
  const Small& s = SmallScene();
  ASSERT_NE(s.scene, nullptr);
  const int arm = KukaArm(*s.scene);
  PlanFigures f;
  const ArmMotion m = MotionOf(*s.scene, arm, f);
  const Config a{0.1, -1.2, 1.0, 0.0, 0.5, 0.0, 0.0};
  Config b = a;
  b[0] += 0.8;
  b[2] -= 0.3;
  EXPECT_EQ(MoveDuration({a}, m, f), 0.0);
  EXPECT_EQ(MoveDuration({a, a, a}, m, f), 0.0);
  EXPECT_LT(MoveDuration({}, m, f), 0.0);
  const double d = MoveDuration({a, b}, m, f);
  // Not faster than joint 1 at its datasheet speed, and not much slower.
  EXPECT_GT(d, 0.8 / m.limits.velocity[0]);
  EXPECT_LT(d, 3.0);
  Segment g;
  g.kind = "move";
  g.t0 = 10.0;
  g.t1 = 10.0 + d + f.settle_s;
  g.path = {a, b};
  for (int j = 0; j < 7; ++j) {
    EXPECT_NEAR(ConfigAt(g, m, f, 10.0)[j], a[j], 1e-9);
    EXPECT_NEAR(ConfigAt(g, m, f, 10.0 + d)[j], b[j], 1e-9);
    EXPECT_NEAR(ConfigAt(g, m, f, g.t1)[j], b[j], 1e-9);
  }
  const Config mid = ConfigAt(g, m, f, 10.0 + d / 2);
  EXPECT_GT(mid[0], a[0] + 0.1);
  EXPECT_LT(mid[0], b[0] - 0.1);
  g.kind = "wait";
  EXPECT_EQ(ConfigAt(g, m, f, 11.0), a);
}

TEST(Motion, LegDurationAndTheLegsLaw) {
  const Small& s = SmallScene();
  ASSERT_NE(s.scene, nullptr);
  const int arm = KukaArm(*s.scene);
  PlanFigures f;
  const ArmMotion m = MotionOf(*s.scene, arm, f);
  const Config a{0.0, -1.0, 1.0, 0.0, 0.5, 0.0, 0.0};
  Config b = a, c = a;
  b[1] += 0.01;
  c[1] += 0.03;
  // Short in joints: the working point's speed and acceleration set it.
  const double length = 0.10;
  EXPECT_NEAR(LegDuration({a, b, c}, length, m, f),
              std::max(1.5 * length / f.leg_speed, std::sqrt(6.0 * length / f.leg_accel)),
              1e-12);
  EXPECT_EQ(LegDuration({a, b}, 0.0, m, f), 0.0);
  // A joint that turns far over the first piece sets it.
  Config far = a;
  far[0] += 2.0;
  const double t = LegDuration({a, far}, 0.05, m, f);
  EXPECT_NEAR(t, 1.5 * 2.0 / m.limits.velocity[0], 1e-12);
  // The law: s(u) = 3u^2 - 2u^3 of the joint-space length.
  Segment g;
  g.kind = "approach";
  g.t0 = 2.0;
  g.t1 = 4.0;
  g.path = {a, b, c};
  EXPECT_EQ(ConfigAt(g, m, f, 2.0), a);
  EXPECT_EQ(ConfigAt(g, m, f, 4.0), c);
  EXPECT_NEAR(ConfigAt(g, m, f, 3.0)[1], a[1] + 0.015, 1e-12);
  const double u = 0.25, frac = 3 * u * u - 2 * u * u * u;
  EXPECT_NEAR(ConfigAt(g, m, f, 2.5)[1], a[1] + 0.03 * frac, 1e-12);
}

// --------------------------------------------------------------- json

Plan HandMadePlan() {
  Plan p;
  p.layout = "hand \"made\"";
  p.layout_sha256 = "ab12";
  p.figures.clearance_m = 0.015;
  p.figures.budgets = {10, 20};
  p.figures.seed = 4000000000u;
  p.complete = true;
  p.bay_planned_s = 123.4567891;
  p.bay_model_s = 99.0;
  ArmPlan a;
  a.name = "arm_a";
  Segment w;
  w.kind = "wait";
  w.step = 3;
  w.t0 = 0.0;
  w.t1 = 1.0 / 3.0;
  w.tool = "gripper";
  w.reason = "waits for rail 0, which arm b sets";
  w.path = {{0.1, -0.2, 1.0 / 7.0, 0.0, 1e-12, -3.14159265358979, 0.5}};
  Segment m = w;
  m.kind = "move";
  m.t0 = w.t1;
  m.t1 = 2.5;
  m.reason = "";
  m.held = "module";
  m.from = "wait";
  m.to = "rail_set 0 approach";
  m.found = true;
  m.budget = 40000;
  m.queries = 123456789012;
  m.planned_s = 1.86666666;
  m.model_s = 2.1;
  m.seed = 77;
  m.built.rails = {0, 1};
  m.built.modules = {};
  m.built.on_pallets = 9;
  m.path.push_back({0.2, -0.3, 0.4, 0.5, 0.6, 0.7, 0.8});
  a.segments = {w, m};
  p.arms = {a, ArmPlan{"arm_b", {}}};
  p.events = {{"tube_in", 1, 0.0, 40.0}, {"clamp", 2, 40.0, 45.0}};
  p.steps = {{1, "tube in", "", 0.0, 40.0}, {3, "take rail 0", "arm_a", 0.3, 2.5}};
  p.phases = {{"tube in", 40.0, 40.0}, {"end", 1.0, 2.0}};
  return p;
}

TEST(Json, RoundTripIsByteIdentical) {
  const Plan p = HandMadePlan();
  const std::string text = PlanToJson(p);
  std::string error;
  const std::optional<Plan> back = PlanFromJson(text, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(PlanToJson(*back), text);
  EXPECT_EQ(back->layout, p.layout);
  EXPECT_EQ(back->figures.seed, p.figures.seed);
  EXPECT_EQ(back->figures.budgets, p.figures.budgets);
  ASSERT_EQ(back->arms.size(), 2u);
  ASSERT_EQ(back->arms[0].segments.size(), 2u);
  const Segment& m = back->arms[0].segments[1];
  EXPECT_EQ(m.queries, 123456789012);
  EXPECT_EQ(m.built.rails, (std::vector<int>{0, 1}));
  EXPECT_EQ(m.built.on_pallets, 9);
  EXPECT_EQ(m.held, "module");
  EXPECT_TRUE(m.found);
  EXPECT_NEAR(m.path[0][2], 1.0 / 7.0, 1e-9);
  EXPECT_EQ(back->arms[0].segments[0].reason, p.arms[0].segments[0].reason);
  // One line per segment.
  int lines = 0;
  for (char c : text) lines += c == '\n';
  EXPECT_LT(lines, 30);
  EXPECT_FALSE(PlanFromJson("{\"layout\": 1}", &error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST(Json, Sha256OfAFile) {
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string path = std::string(tmp != nullptr ? tmp : "/tmp") + "/abc.txt";
  {
    std::ofstream out(path, std::ios::binary);
    out << "abc";
  }
  EXPECT_EQ(Sha256OfFile(path),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  {
    std::ofstream out(path, std::ios::binary);
    out << "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  }
  EXPECT_EQ(Sha256OfFile(path),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  EXPECT_EQ(Sha256OfFile(path + ".none"), "");
}

// ------------------------------------------------------------- player

TEST(Player, TwoArmsAndTheBay) {
  const Small& s = SmallScene();
  ASSERT_NE(s.scene, nullptr);
  ASSERT_GE(s.scene->ArmCount(), 2);
  const LayoutScene& scene = *s.scene;
  CheckReport report;
  BayStep tube;
  tube.number = 1;
  tube.kind = "tube_in";
  BayStep set;
  set.number = 3;
  set.kind = "rail_set";
  set.index = 0;
  set.arm = scene.arm(0).use->name;
  BayStep out;
  out.number = 9;
  out.kind = "bay_out";
  report.steps = {tube, set, out};
  Plan plan;
  plan.events = {{"tube_in", 1, 0.0, 4.0}, {"bay_out", 9, 8.0, 10.0}};
  Config wait0{}, wait1{};
  for (int j = 0; j < 6; ++j) {
    wait0[j] = scene.arm(0).wait[j];
    wait1[j] = scene.arm(1).wait[j];
  }
  wait0[6] = scene.arm(0).wait_track;
  wait1[6] = scene.arm(1).wait_track;
  ArmPlan a{scene.arm(0).use->name, {}};
  Segment g;
  g.kind = "wait";
  g.step = 3;
  g.t0 = 0.0;
  g.t1 = 5.0;
  g.tool = "tool_a";
  g.held = "rail";
  g.path = {wait0};
  a.segments.push_back(g);
  g.kind = "work";
  g.t0 = 5.0;
  g.t1 = 6.0;
  a.segments.push_back(g);
  g.kind = "wait";
  g.held = "";
  g.t0 = 6.0;
  g.t1 = 10.0;
  a.segments.push_back(g);
  ArmPlan b{scene.arm(1).use->name, {}};
  Config to = wait1;
  to[0] += 0.4;
  const ArmMotion mb = MotionOf(scene, 1, plan.figures);
  const double d = MoveDuration({wait1, to}, mb, plan.figures);
  ASSERT_GT(d, 0.0);
  Segment m;
  m.kind = "move";
  m.step = 4;
  m.t0 = 0.0;
  m.t1 = d + plan.figures.settle_s;
  m.path = {wait1, to};
  m.tool = "tool_b";
  b.segments.push_back(m);
  Segment w2;
  w2.kind = "wait";
  w2.step = 4;
  w2.t0 = m.t1;
  w2.t1 = 10.0;
  w2.path = {to};
  w2.tool = "tool_b";
  b.segments.push_back(w2);
  plan.arms = {a, b};
  const PlanPlayer player(scene, report, plan);
  EXPECT_DOUBLE_EQ(player.duration(), 10.0);

  WorldAt w = player.At(2.0);
  ASSERT_EQ(static_cast<int>(w.arms.size()), scene.ArmCount());
  EXPECT_EQ(w.event, "tube_in");
  EXPECT_TRUE(w.tube);
  EXPECT_TRUE(w.rails.empty());
  EXPECT_EQ(w.arms[0].kind, "wait");
  EXPECT_EQ(w.arms[0].held, "rail");
  EXPECT_EQ(w.arms[1].kind, "wait");
  for (int j = 0; j < 7; ++j) EXPECT_NEAR(w.arms[1].config[j], to[j], 1e-9);
  // Half way along the way in, by length at constant speed.
  const std::vector<Vec3>& path = s.layout.tube_in.path;
  if (path.size() >= 2) {
    double total = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      total += std::sqrt(std::pow(path[i][0] - path[i - 1][0], 2) +
                         std::pow(path[i][1] - path[i - 1][1], 2) +
                         std::pow(path[i][2] - path[i - 1][2], 2));
    }
    EXPECT_GT(total, 0.0);
    const WorldAt start = player.At(0.0);
    EXPECT_NEAR(start.way_offset.x(), path[0][0], 1e-9);
    EXPECT_NEAR(start.way_offset.y(), path[0][1], 1e-9);
  }
  w = player.At(d / 2);
  EXPECT_EQ(w.arms[1].kind, "move");
  EXPECT_GT(w.arms[1].config[0], wait1[0]);
  EXPECT_LT(w.arms[1].config[0], to[0]);
  w = player.At(5.5);
  EXPECT_EQ(w.arms[0].kind, "work");
  EXPECT_TRUE(w.rails.empty());
  EXPECT_EQ(w.event, "");
  EXPECT_EQ(w.way_offset, Eigen::Vector3d::Zero());
  w = player.At(6.0);
  EXPECT_EQ(w.arms[0].kind, "wait");
  EXPECT_EQ(w.arms[0].held, "");
  EXPECT_EQ(w.rails, (std::vector<int>{0}));
  const Built built = player.BuiltAt(w);
  EXPECT_EQ(built.rails, (std::vector<int>{0}));
  EXPECT_EQ(built.on_pallets, PointCount(s.layout, "module_take"));
  w = player.At(9.0);
  EXPECT_EQ(w.event, "bay_out");
  // The arms not in the plan stand at their wait poses with their first tools.
  for (int i = 2; i < scene.ArmCount(); ++i) {
    EXPECT_EQ(w.arms[i].tool, scene.arm(i).use->first_tool);
    EXPECT_NEAR(w.arms[i].config[0], scene.arm(i).wait[0], 1e-12);
  }
  // The poses go into the data, and the distances come back named.
  DataPtr data = scene.MakeData();
  player.SetPoses(w, data.get());
  EXPECT_NEAR(data->qpos[scene.arm(1).h->qpos[0]], to[0], 1e-9);
  const std::vector<Closest> near = InstantDistances(scene, report, player, player.At(7.0), data.get());
  const int n = scene.ArmCount();
  EXPECT_EQ(static_cast<int>(near.size()), n + n * (n - 1) / 2);
  for (const Closest& c : near) {
    EXPECT_FALSE(c.arm.empty());
    EXPECT_FALSE(c.a.empty());
    EXPECT_DOUBLE_EQ(c.t, 7.0);
  }
}

// ----------------------------------------------------- helpers

std::string Faults(const VerifyReport& r) {
  std::string s;
  for (const std::string& f : r.faults) s += "  " + f + "\n";
  return s;
}

bool AnyFault(const VerifyReport& r, const std::vector<std::string>& words) {
  for (const std::string& f : r.faults) {
    bool all = true;
    for (const std::string& w : words) all = all && f.find(w) != std::string::npos;
    if (all) {
      std::printf("found: %s\n", f.c_str());
      return true;
    }
  }
  return false;
}

// The verifier without the schedule's samples.
VerifyOptions Quick(bool edges) {
  VerifyOptions o;
  o.schedule = false;
  o.edges = edges;
  return o;
}

// ------------------------------------------------- the order of work

// The committed plan of flat_rack_3_pin_arms_ordered (src/layout.h,
// WorkOrder) against the verifier's copies of the order's rules: it passes
// as it is, a take before its own rails' bolts is found, and so is the first
// take before the last bolt when the layout says modules start after all
// bolts; a wait right after a tool change, at the tool-change pose, is
// allowed (R21), and one moved off that pose is not.
struct OrderedPlan {
  std::string error;
  Layout layout;
  Plan plan;
  std::string sha256;
};

// A committed plan by its layout's name.
OrderedPlan NamedPlan(const std::string& name) {
  OrderedPlan o;
  std::string error;
  const std::optional<Layout> l = LoadLayout(LayoutDir() + "/" + name + ".json", &error);
  if (!l) {
    o.error = error;
    return o;
  }
  o.layout = *l;
  const std::optional<Plan> p = PlanFromJson(ReadFile(PlanDir() + "/" + name + ".json"), &error);
  if (!p) {
    o.error = "plans/" + name + ".json: " + error;
    return o;
  }
  o.plan = *p;
  o.sha256 = Sha256OfFile(LayoutDir() + "/" + name + ".json");
  return o;
}

const OrderedPlan& TheOrdered() {
  static const OrderedPlan o = NamedPlan("flat_rack_3_pin_arms_ordered");
  return o;
}

// v09's plan, no longer the chosen one, still verifies against its layout.
TEST(Superseded, FlatRack3PinArmsPassesTheVerifierWithoutTheSchedule) {
  const OrderedPlan o = NamedPlan("flat_rack_3_pin_arms");
  ASSERT_TRUE(o.error.empty()) << o.error;
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, Quick(false));
  EXPECT_TRUE(r.ok) << Faults(r);
  EXPECT_NEAR(o.plan.bay_planned_s, 644.28, 0.005);
}

// Run 034: the all-KUKA 40 ft layout, one KR 20 pin arm, one bay planned
// whole; the verifier over the whole plan, its schedule included.
TEST(Kr20, OneBayOfTheKr20LayoutPassesTheVerifier) {
  const OrderedPlan o = NamedPlan("flat_rack_1_kr20_pin_arm_ordered");
  ASSERT_TRUE(o.error.empty()) << o.error;
  const VerifyReport r = VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  EXPECT_NEAR(o.plan.bay_planned_s, 897.90, 0.005);
}

// Run 035: the same cell with three KR 20 pin arms on one shared beam
// (Track::shares_beam_with); run 037 planned its three bays (v15). The
// verifier over the whole plan, its schedule included.
TEST(Kr20, ThreeBaysOfTheSharedBeamLayoutPassTheVerifier) {
  const OrderedPlan o = NamedPlan("flat_rack_3_kr20_pin_arms_shared_ordered");
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_TRUE(o.plan.complete);
  EXPECT_EQ(o.plan.bays, 3);
  const VerifyReport r = VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  EXPECT_NEAR(o.plan.bay_planned_s, 626.32, 0.005);
}

// Run 037: the same layout with run 036's faster retreats (retreat_legs),
// three bays (v16); the verifier over the whole plan, its schedule included.
TEST(Kr20, ThreeBaysOfTheSharedBeamLayoutWithFastRetreatsPassTheVerifier) {
  const OrderedPlan o = NamedPlan("flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat");
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_TRUE(o.plan.complete);
  EXPECT_EQ(o.plan.bays, 3);
  const VerifyReport r = VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  EXPECT_NEAR(o.plan.bay_planned_s, 574.73, 0.005);
}

// Run 041: v16's layout with the modules set in turns of the three pin arms
// (order.modules 0, 4, 7, 1, 5, 8, 2, 6, 3), so that pin arm C no longer waits
// for the last two sets; one bay planned whole, the verifier over the whole
// plan, its schedule included.
TEST(Kr20, OneBayOfTheSharedBeamLayoutWithTheModulesInTurnsPassesTheVerifier) {
  const OrderedPlan o = NamedPlan("flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma");
  ASSERT_TRUE(o.error.empty()) << o.error;
  EXPECT_TRUE(o.plan.complete);
  const VerifyReport r = VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, VerifyOptions{});
  EXPECT_TRUE(r.ok) << Faults(r);
  EXPECT_NEAR(o.plan.bay_planned_s, 543.96, 0.005);
}

TEST(Ordered, PassesTheVerifierWithoutTheSchedule) {
  const OrderedPlan& o = TheOrdered();
  ASSERT_TRUE(o.error.empty()) << o.error;
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, Quick(false));
  EXPECT_TRUE(r.ok) << Faults(r);
  EXPECT_LE(o.plan.bay_planned_s, 600.0);
}

TEST(Ordered, ATakeBeforeItsRailsBoltsIsFound) {
  const OrderedPlan& o = TheOrdered();
  ASSERT_TRUE(o.error.empty()) << o.error;
  // Module 8 first: its rails 8 and 9 are bolted after the first take starts.
  Layout l = o.layout;
  l.order.modules = {8, 1, 4, 2, 5, 7, 6, 3, 0};
  const VerifyReport r = VerifyPlan(l, ModelPaths::FromEnvironment(), o.plan, o.sha256, Quick(false));
  EXPECT_TRUE(AnyFault(r, {"take module 8", "rail 9's strap bolt is driven"})) << Faults(r);
}

TEST(Ordered, TheFirstTakeBeforeTheLastBoltIsFoundWhenAllBoltsComeFirst) {
  const OrderedPlan& o = TheOrdered();
  ASSERT_TRUE(o.error.empty()) << o.error;
  Layout l = o.layout;
  l.order.modules_start = "after_all_bolts";
  const VerifyReport r = VerifyPlan(l, ModelPaths::FromEnvironment(), o.plan, o.sha256, Quick(false));
  EXPECT_TRUE(AnyFault(r, {"take module 0", "the last strap bolt is driven"})) << Faults(r);
}

TEST(Ordered, AWaitAtTheToolChangePose) {
  const OrderedPlan& o = TheOrdered();
  ASSERT_TRUE(o.error.empty()) << o.error;
  // A wait of no length put right after a pin arm's first tool change.
  Plan p = o.plan;
  int arm = -1;
  size_t g = 0;
  for (size_t a = 0; a < p.arms.size() && arm < 0; ++a) {
    const std::vector<Segment>& ps = p.arms[a].segments;
    for (size_t i = 0; i + 1 < ps.size(); ++i) {
      if (ps[i].kind == "work" && ps[i + 1].kind == "move" && !ps[i].path.empty()) {
        bool change = false;
        for (const StepTime& st : p.steps) {
          change = change || (st.number == ps[i].step && st.what == "tool change");
        }
        if (change) {
          arm = static_cast<int>(a);
          g = i + 1;
          break;
        }
      }
    }
  }
  ASSERT_GE(arm, 0) << "no tool change followed by a move";
  std::vector<Segment>& ps = p.arms[arm].segments;
  Segment w;
  w.kind = "wait";
  w.step = ps[g].step;
  w.t0 = w.t1 = ps[g].t0;
  w.tool = ps[g].tool;
  w.held = ps[g].held;
  w.path = {ps[g - 1].path.back()};
  w.reason = "for nothing (a test)";
  ps.insert(ps.begin() + static_cast<long>(g), w);
  VerifyReport r = VerifyPlan(o.layout, ModelPaths::FromEnvironment(), p, o.sha256, Quick(false));
  EXPECT_FALSE(AnyFault(r, {"its wait", "stands neither"})) << Faults(r);
  // The same wait elsewhere: half way along the move after it.
  ps[g].path[0] = ps[g + 1].path[ps[g + 1].path.size() / 2];
  r = VerifyPlan(o.layout, ModelPaths::FromEnvironment(), p, o.sha256, Quick(false));
  EXPECT_TRUE(AnyFault(r, {"its wait", "stands neither"})) << Faults(r);
}

// R21 (run 029): a wait at a work pose between two work steps there (the
// locate and the pin of one hole) is part of that work and is allowed; the
// same wait after the last of them, just before the retreat, is not.
TEST(Ordered, AWaitAtItsWorkPose) {
  const OrderedPlan& o = TheOrdered();
  ASSERT_TRUE(o.error.empty()) << o.error;
  Plan p = o.plan;
  int arm = -1;
  size_t g = 0;
  for (size_t a = 0; a < p.arms.size() && arm < 0; ++a) {
    const std::vector<Segment>& ps = p.arms[a].segments;
    for (size_t i = 0; i + 2 < ps.size(); ++i) {
      if (ps[i].kind == "work" && ps[i + 1].kind == "work" && ps[i + 2].kind == "retreat" &&
          !ps[i].path.empty() && !ps[i + 1].path.empty()) {
        arm = static_cast<int>(a);
        g = i + 1;
        break;
      }
    }
  }
  ASSERT_GE(arm, 0) << "no two work steps one after the other before a retreat";
  // A wait of no length between the two work steps, at their pose.
  std::vector<Segment>& ps = p.arms[arm].segments;
  Segment w;
  w.kind = "wait";
  w.step = ps[g].step;
  w.t0 = w.t1 = ps[g].t0;
  w.tool = ps[g].tool;
  w.held = ps[g].held;
  w.path = {ps[g - 1].path.back()};
  w.reason = "for nothing (a test)";
  Plan between = p;
  between.arms[arm].segments.insert(between.arms[arm].segments.begin() + static_cast<long>(g), w);
  VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), between, o.sha256, Quick(false));
  EXPECT_FALSE(AnyFault(r, {"its wait", "stands neither"})) << Faults(r);
  // The same wait outside that span: after the second work step, at its
  // pose, just before the retreat.
  w.step = ps[g + 1].step;
  w.t0 = w.t1 = ps[g + 1].t0;
  w.path = {ps[g].path.back()};
  ps.insert(ps.begin() + static_cast<long>(g + 1), w);
  r = VerifyPlan(o.layout, ModelPaths::FromEnvironment(), p, o.sha256, Quick(false));
  EXPECT_TRUE(AnyFault(r, {"its wait", "stands neither"})) << Faults(r);
}

}  // namespace
}  // namespace cell::layout
