// The replay of src/layout_plan.h: the motion of a segment, the plan as a
// file, the player, and the committed plan of the chosen layout, verified
// without planning anything (R20). The committed plan's tests fail until
// plans/<chosen>.json exists for a chosen layout on the flat rack: passing
// only with a real plan is what they are for. The chosen layout is the one
// versions/versions.json names (src/layout.h, ChosenLayout).

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

// ----------------------------------------------------- the committed plan

struct Committed {
  std::string error;  // Why there is nothing to replay.
  Layout layout;
  std::string layout_path, plan_path, text, sha256;
  Plan plan;
};

const Committed& TheCommitted() {
  static const Committed* c = [] {
    auto* c = new Committed;
    std::string why;
    const std::optional<std::string> chosen = ChosenLayout(VersionsPath(), &why);
    if (!chosen) {
      c->error = "no layout is chosen: " + why;
      return c;
    }
    std::optional<Layout> l = LoadLayout(LayoutDir() + "/" + *chosen + ".json", &why);
    if (!l) {
      c->error = "the chosen layout does not load: " + why;
      return c;
    }
    c->layout = *l;
    if (c->layout.base != "flat_rack") {
      c->error = "the chosen layout, " + c->layout.name +
                 ", is not on the flat rack: no plan of it is replayed";
      return c;
    }
    c->layout_path = LayoutDir() + "/" + c->layout.name + ".json";
    c->plan_path = PlanDir() + "/" + c->layout.name + ".json";
    c->text = ReadFile(c->plan_path);
    if (c->text.empty()) {
      c->error = "the chosen layout " + c->layout.name + " has no committed plan " +
                 c->plan_path;
      return c;
    }
    std::string error;
    std::optional<Plan> p = PlanFromJson(c->text, &error);
    if (!p) {
      c->error = c->plan_path + ": " + error;
      return c;
    }
    c->plan = *p;
    c->sha256 = Sha256OfFile(c->layout_path);
    return c;
  }();
  return *c;
}

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

#define NEED_COMMITTED(c)                                                       \
  const Committed& c = TheCommitted();                                          \
  if (!c.error.empty()) FAIL() << c.error;

TEST(Committed, PassesTheVerifier) {
  NEED_COMMITTED(c);
  const VerifyReport r =
      VerifyPlan(c.layout, ModelPaths::FromEnvironment(), c.plan, c.sha256);
  EXPECT_TRUE(r.ok) << r.faults.size() << " faults:\n" << Faults(r);
  EXPECT_TRUE(r.faults.empty());
  const double floor = c.plan.figures.clearance_m - VerifyOptions{}.tolerance_m;
  EXPECT_GE(r.least_clearance_m, floor);
  EXPECT_GE(r.least_arm_to_arm_m, floor);
  EXPECT_GT(r.queries, 0);
  EXPECT_FALSE(r.least.empty());
  EXPECT_EQ(r.joint_margins.size(), 5u);
  std::printf("least clearance %.4f m, least arm to arm %.4f m, least joint margin %.2f deg, "
              "%d waits %.1f s, %lld queries\n",
              r.least_clearance_m, r.least_arm_to_arm_m, r.least_joint_margin_deg, r.waits,
              r.wait_s, static_cast<long long>(r.queries));
}

TEST(Committed, AnotherLayoutFileIsRefused) {
  NEED_COMMITTED(c);
  const VerifyReport r = VerifyPlan(c.layout, ModelPaths::FromEnvironment(), c.plan,
                                    std::string(64, '0'));
  EXPECT_FALSE(r.ok);
  ASSERT_EQ(r.faults.size(), 1u);
  EXPECT_NE(r.faults[0].find("the plan was made for another version of the layout"),
            std::string::npos);
}

TEST(Committed, JsonRoundTripIsByteIdentical) {
  NEED_COMMITTED(c);
  EXPECT_EQ(PlanToJson(c.plan), c.text);
}

// The breakages: each one gives a fault that names the thing broken.
VerifyOptions Quick(bool edges) {
  VerifyOptions o;
  o.schedule = false;
  o.edges = edges;
  return o;
}

// The arm that sets the modules, and the index of its plan.
int ModuleArmPlan(const Committed& c) {
  const Work* w = FindWork(c.layout, "module_set");
  if (w == nullptr) return -1;
  for (size_t k = 0; k < c.plan.arms.size(); ++k) {
    if (c.plan.arms[k].name == w->arm) return static_cast<int>(k);
  }
  return -1;
}

TEST(Committed, BreakagesAreFound) {
  NEED_COMMITTED(c);
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const int ma = ModuleArmPlan(c);
  ASSERT_GE(ma, 0);
  const std::string arm = c.plan.arms[ma].name;
  const std::vector<Segment>& segs = c.plan.arms[ma].segments;

  {
    // A waypoint pushed 0.3 rad into joint 2: an edge is not free, or the
    // waypoint is out of range. A push need not bring every move into
    // something, so up to four moves are tried, those that carry a module
    // and have a waypoint inside first; the first that faults is the case.
    std::vector<int> moves;
    for (int pass = 0; pass < 3; ++pass) {
      for (size_t i = 0; i < segs.size(); ++i) {
        const Segment& g = segs[i];
        if (g.kind != "move" || g.path.size() < 2) continue;
        const bool inner = g.path.size() >= 3, module = g.held == "module";
        if ((pass == 0 && inner && module) || (pass == 1 && inner && !module) ||
            (pass == 2 && !inner)) {
          moves.push_back(static_cast<int>(i));
        }
      }
    }
    ASSERT_FALSE(moves.empty()) << "the module arm has no move";
    bool found = false;
    std::string faults;
    for (size_t m = 0; m < moves.size() && m < 4 && !found; ++m) {
      Plan p = c.plan;
      std::vector<Config>& path = p.arms[ma].segments[moves[m]].path;
      const size_t k = path.size() >= 3 ? path.size() / 2 : path.size() - 1;
      path[k][1] += 0.3;
      const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(true));
      found = AnyFault(r, {"arm " + arm, "not free"}) || AnyFault(r, {"arm " + arm, "outside"}) ||
              (k + 1 == path.size() && AnyFault(r, {"arm " + arm, "move ends"}));
      faults += Faults(r);
    }
    EXPECT_TRUE(found) << faults;
  }
  {
    // A work segment's end 1 s earlier, the segments after it left alone.
    Plan p = c.plan;
    int g = -1;
    for (size_t i = 0; i + 1 < segs.size() && g < 0; ++i) {
      if (segs[i].kind == "work") g = static_cast<int>(i);
    }
    ASSERT_GE(g, 0);
    p.arms[ma].segments[g].t1 -= 1.0;
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(AnyFault(r, {"arm " + arm, "gap"})) << Faults(r);
  }
  {
    // A strap bolt started before its rail is set: the bolting arm's
    // segments of the rail phase moved earlier together.
    // The steps are found by their names in the plan's step times.
    Plan p = c.plan;
    const StepTime* bolt = nullptr;
    int rail = -1;
    for (const StepTime& s : p.steps) {
      if (std::sscanf(s.what.c_str(), "bolt rail %d", &rail) == 1) {
        bolt = &s;
        break;
      }
    }
    ASSERT_NE(bolt, nullptr);
    int set_step = -1;
    for (const StepTime& s : p.steps) {
      if (s.what == "set rail " + std::to_string(rail)) set_step = s.number;
    }
    int pa = -1;
    for (size_t k = 0; k < p.arms.size(); ++k) {
      if (p.arms[k].name == bolt->arm) pa = static_cast<int>(k);
    }
    ASSERT_GE(pa, 0);
    double set_end = -1.0;
    for (const ArmPlan& a : p.arms) {
      for (const Segment& s : a.segments) {
        if (s.step == set_step && s.kind == "work") set_end = s.t1;
      }
    }
    ASSERT_GT(set_end, 0.0);
    std::vector<Segment>& bs = p.arms[pa].segments;
    int first = -1;
    for (size_t i = 0; i < bs.size() && first < 0; ++i) {
      if (bs[i].step == bolt->number && bs[i].kind != "wait") first = static_cast<int>(i);
    }
    ASSERT_GT(first, 0);
    ASSERT_EQ(bs[first - 1].kind, "wait");
    const double shift = bs[first].t0 - set_end + 1.0;
    ASSERT_LT(shift, bs[first - 1].t1 - bs[first - 1].t0) << "the wait before is too short";
    bs[first - 1].t1 -= shift;
    int last = first;
    while (last < static_cast<int>(bs.size()) && bs[last].kind != "wait") {
      bs[last].t0 -= shift;
      bs[last].t1 -= shift;
      ++last;
    }
    if (last < static_cast<int>(bs.size())) bs[last].t0 -= shift;
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(AnyFault(r, {bolt->what, "rail " + std::to_string(rail) + "'s set has ended"}))
        << Faults(r);
  }
  {
    // A move's last waypoint 0.05 rad off: not at the step's pose.
    Plan p = c.plan;
    int g = -1;
    for (size_t i = 0; i < segs.size() && g < 0; ++i) {
      if (segs[i].kind == "move" && segs[i].path.size() >= 2) g = static_cast<int>(i);
    }
    ASSERT_GE(g, 0);
    p.arms[ma].segments[g].path.back()[0] += 0.05;
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(AnyFault(r, {"arm " + arm, "move ends", "off"})) << Faults(r);
  }
  {
    // A held module made "".
    Plan p = c.plan;
    int g = -1;
    for (size_t i = 0; i < segs.size() && g < 0; ++i) {
      if (segs[i].held == "module") g = static_cast<int>(i);
    }
    ASSERT_GE(g, 0);
    p.arms[ma].segments[g].held = "";
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(AnyFault(r, {"arm " + arm, "holding \"\"", "holding \"module\""})) << Faults(r);
  }
  {
    // A wait's reason emptied.
    Plan p = c.plan;
    int pa = -1, g = -1;
    for (size_t k = 0; k < p.arms.size() && g < 0; ++k) {
      for (size_t i = 0; i < p.arms[k].segments.size() && g < 0; ++i) {
        if (p.arms[k].segments[i].kind == "wait") {
          pa = static_cast<int>(k);
          g = static_cast<int>(i);
        }
      }
    }
    ASSERT_GE(g, 0);
    p.arms[pa].segments[g].reason = "";
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(AnyFault(r, {"arm " + p.arms[pa].name, "wait with no reason"})) << Faults(r);
  }
}


const StepTime* StepOf(const Plan& p, int number) {
  for (const StepTime& t : p.steps) {
    if (t.number == number) return &t;
  }
  return nullptr;
}

bool Begins(const std::string& s, const std::string& with) { return s.rfind(with, 0) == 0; }

TEST(Committed, TamperingIsFound) {
  NEED_COMMITTED(c);
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const int ma = ModuleArmPlan(c);
  ASSERT_GE(ma, 0);
  const std::string arm = c.plan.arms[ma].name;
  const std::vector<Segment>& segs = c.plan.arms[ma].segments;
  {
    // Figures of the plan's own that would loosen the checks.
    Plan p = c.plan;
    p.figures.sample_s = 10.0;
    p.figures.edge_resolution = 5.0;
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_TRUE(AnyFault(r, {"figure sample_s is 10"})) << Faults(r);
    EXPECT_TRUE(AnyFault(r, {"figure edge_resolution is 5"})) << Faults(r);
  }
  {
    // One move's world emptied.
    Plan p = c.plan;
    int g = -1;
    for (size_t i = 0; i < segs.size(); ++i) {
      if (segs[i].kind == "move" && !segs[i].built.modules.empty()) g = static_cast<int>(i);
    }
    ASSERT_GE(g, 0);
    p.arms[ma].segments[g].built = Built{};
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_TRUE(AnyFault(r, {"arm " + arm, "its move", "planned in a bay without"})) << Faults(r);
  }
  {
    // A middle sample of a leg moved 0.05 rad.
    Plan p = c.plan;
    int g = -1;
    for (size_t i = 0; i < segs.size() && g < 0; ++i) {
      if (segs[i].kind == "approach" && segs[i].path.size() >= 3) g = static_cast<int>(i);
    }
    ASSERT_GE(g, 0);
    std::vector<Config>& path = p.arms[ma].segments[g].path;
    path[path.size() / 2][1] += 0.05;
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_TRUE(AnyFault(r, {"arm " + arm, "its approach", "at sample"})) << Faults(r);
  }
  {
    // A wait that stands neither at the wait pose nor where a retreat ended:
    // the first wait of the module arm put at the next move's second waypoint.
    Plan p = c.plan;
    std::vector<Segment>& ps = p.arms[ma].segments;
    int g = -1;
    for (size_t i = 0; i + 1 < ps.size() && g < 0; ++i) {
      if (ps[i].kind == "wait" && ps[i + 1].kind == "move" && ps[i + 1].path.size() >= 2) {
        g = static_cast<int>(i);
      }
    }
    ASSERT_GE(g, 0);
    ps[g].path[0] = ps[g + 1].path[1];
    ps[g + 1].path[0] = ps[g + 1].path[1];
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, Quick(false));
    EXPECT_TRUE(AnyFault(r, {"arm " + arm, "its wait", "stands neither"})) << Faults(r);
  }
  // R22 along the schedule, in a short stretch of it.
  {
    // A pin arm's longest wait put at the work pose of its next locate: the
    // tool's nose then stands 5 to 10 mm from the rail outside its work.
    Plan p = c.plan;
    int pa = -1, g = -1, w = -1;
    double longest = 0.0;
    for (size_t k = 0; k < p.arms.size(); ++k) {
      if (static_cast<int>(k) == ma) continue;
      const std::vector<Segment>& as = p.arms[k].segments;
      for (size_t i = 0; i < as.size(); ++i) {
        if (as[i].kind != "wait" || as[i].t1 - as[i].t0 <= longest) continue;
        for (size_t j = i + 1; j < as.size(); ++j) {
          if (as[j].kind != "work") continue;
          const StepTime* st = StepOf(p, as[j].step);
          if (st != nullptr && Begins(st->what, "locate ")) {
            pa = static_cast<int>(k);
            g = static_cast<int>(i);
            w = static_cast<int>(j);
            longest = as[i].t1 - as[i].t0;
            break;
          }
        }
      }
    }
    ASSERT_GE(g, 0);
    Segment& wait = p.arms[pa].segments[g];
    wait.path[0] = p.arms[pa].segments[w].path[0];
    VerifyOptions o = Quick(false);
    o.schedule = true;
    o.from_s = wait.t0;
    o.to_s = std::min(wait.t1, wait.t0 + 1.0);
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, o);
    const std::string name = p.arms[pa].name;
    EXPECT_TRUE(AnyFault(r, {"(step " + std::to_string(wait.step) + ") arm " + name, " to ",
                             "below"}))
        << Faults(r);
  }
  {
    // A locate and the pin after it at the same hole exchanged in time.
    Plan p = c.plan;
    int pa = -1, a = -1, b = -1;
    for (size_t k = 0; k < p.arms.size() && pa < 0; ++k) {
      const std::vector<Segment>& as = p.arms[k].segments;
      for (size_t i = 0; i + 1 < as.size() && pa < 0; ++i) {
        const StepTime* s0 = StepOf(p, as[i].step);
        const StepTime* s1 = StepOf(p, as[i + 1].step);
        if (as[i].kind == "work" && as[i + 1].kind == "work" && s0 && s1 &&
            Begins(s0->what, "locate ") && Begins(s1->what, "pin ")) {
          pa = static_cast<int>(k);
          a = as[i].step;
          b = as[i + 1].step;
        }
      }
    }
    ASSERT_GE(pa, 0);
    double t = 0.0;
    for (Segment& g : p.arms[pa].segments) {
      if (g.step == a && g.kind != "wait") {
        g.step = b;
        t = g.t0;
      } else if (g.step == b && g.kind != "wait") {
        g.step = a;
      }
    }
    VerifyOptions o = Quick(false);
    o.schedule = true;
    o.from_s = t;
    o.to_s = t + 0.2;
    const VerifyReport r = VerifyPlan(c.layout, paths, p, c.sha256, o);
    const StepTime* sa = StepOf(p, a);
    EXPECT_TRUE(AnyFault(r, {"before step " + std::to_string(a) + " (" + sa->what +
                             ") has ended"}))
        << Faults(r);
  }
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

const OrderedPlan& TheOrdered() {
  static const OrderedPlan* o = [] {
    auto* o = new OrderedPlan;
    const std::string name = "flat_rack_3_pin_arms_ordered";
    std::string error;
    const std::optional<Layout> l = LoadLayout(LayoutDir() + "/" + name + ".json", &error);
    if (!l) {
      o->error = error;
      return o;
    }
    o->layout = *l;
    const std::optional<Plan> p = PlanFromJson(ReadFile(PlanDir() + "/" + name + ".json"), &error);
    if (!p) {
      o->error = "plans/" + name + ".json: " + error;
      return o;
    }
    o->plan = *p;
    o->sha256 = Sha256OfFile(LayoutDir() + "/" + name + ".json");
    return o;
  }();
  return *o;
}

#define NEED_ORDERED(o)                \
  const OrderedPlan& o = TheOrdered(); \
  if (!o.error.empty()) FAIL() << o.error;

TEST(Ordered, PassesTheVerifierWithoutTheSchedule) {
  NEED_ORDERED(o);
  const VerifyReport r =
      VerifyPlan(o.layout, ModelPaths::FromEnvironment(), o.plan, o.sha256, Quick(false));
  EXPECT_TRUE(r.ok) << Faults(r);
  EXPECT_LE(o.plan.bay_planned_s, 600.0);
}

TEST(Ordered, ATakeBeforeItsRailsBoltsIsFound) {
  NEED_ORDERED(o);
  // Module 8 first: its rails 8 and 9 are bolted after the first take starts.
  Layout l = o.layout;
  l.order.modules = {8, 1, 4, 2, 5, 7, 6, 3, 0};
  const VerifyReport r = VerifyPlan(l, ModelPaths::FromEnvironment(), o.plan, o.sha256, Quick(false));
  EXPECT_TRUE(AnyFault(r, {"take module 8", "rail 9's strap bolt is driven"})) << Faults(r);
}

TEST(Ordered, TheFirstTakeBeforeTheLastBoltIsFoundWhenAllBoltsComeFirst) {
  NEED_ORDERED(o);
  Layout l = o.layout;
  l.order.modules_start = "after_all_bolts";
  const VerifyReport r = VerifyPlan(l, ModelPaths::FromEnvironment(), o.plan, o.sha256, Quick(false));
  EXPECT_TRUE(AnyFault(r, {"take module 0", "the last strap bolt is driven"})) << Faults(r);
}

TEST(Ordered, AWaitAtTheToolChangePose) {
  NEED_ORDERED(o);
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

}  // namespace
}  // namespace cell::layout
