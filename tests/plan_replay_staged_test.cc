// Staged work in the plan (src/layout_plan_stages.h), on the two 20 ft flat
// racks cell, a bay built in three stages. Without planning anything: the
// stages' scenes, where the tube stands through a transfer, the phases and
// the waits of a staged bay, and the verifier's rules of staged work on a
// plan made up here (a transfer short, missing, a step across one, an arm
// off its wait pose through one, a module set before the stage's magazine
// is taken and parked). With each committed staged plan plans/<name>.json,
// v11's and the balanced layout's (v12), once it exists (the tests of it are
// skipped until then): the verifier over the whole plan, its schedule
// sampled in four pieces, and the same tampering on copies of it; these
// tests are parameterized by the layout's name. The checker's report is made once and shared
// (VerifyPlanWithReport).

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_plan_stages.h"
#include "src/layout_scene.h"

namespace cell::layout {
namespace {

// The staged layouts with a committed plan: v11's, which the made-up plans
// use too, and the balanced one (v12).
const std::vector<std::string>& StagedNames() {
  static const std::vector<std::string> names = {"two_flat_racks_2_pin_arms",
                                                 "two_flat_racks_2_pin_arms_balanced"};
  return names;
}

std::string Dir(const char* variable, const char* fallback) {
  const char* dir = std::getenv(variable);
  return dir != nullptr ? dir : fallback;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

struct Committed {
  std::string error;
  bool no_plan = false;  // The layout and the report are there, the plan is not.
  Layout layout;
  std::string text, sha256;
  Plan plan;
  CheckReport report;
};

Committed* Load(const std::string& name) {
  auto* c = new Committed;
  const std::string layout_path = Dir("LAYOUT_DIR", "layouts") + "/" + name + ".json";
  std::string why;
  std::optional<Layout> l = LoadLayout(layout_path, &why);
  if (!l) {
    c->error = "the layout does not load: " + why;
    return c;
  }
  c->layout = *l;
  c->sha256 = Sha256OfFile(layout_path);
  c->report = CheckLayout(c->layout, ModelPaths::FromEnvironment());
  if (!c->report.pass) {
    c->error = "the layout does not pass the checker";
    return c;
  }
  c->text = ReadFile(Dir("PLAN_DIR", "plans") + "/" + name + ".json");
  if (c->text.empty()) {
    c->no_plan = true;
    return c;
  }
  std::optional<Plan> p = PlanFromJson(c->text, &why);
  if (!p) {
    c->error = "the plan does not read: " + why;
    return c;
  }
  c->plan = *p;
  return c;
}

// Each layout and plan is loaded and checked once, when a test first needs it.
const Committed& TheCommitted(const std::string& name) {
  static std::map<std::string, const Committed*> loaded;
  const Committed*& c = loaded[name];
  if (c == nullptr) c = Load(name);
  return *c;
}

// The layout and its checker's report: v11's, or the one named.
// It names the test's Committed `c`.
#define NEED_LAYOUT_OF(name)                   \
  const Committed& c = TheCommitted(name);     \
  if (!c.error.empty()) FAIL() << c.error;
#define NEED_LAYOUT() NEED_LAYOUT_OF(StagedNames()[0])

// And the committed plan, skipped while there is none.
#define NEED_COMMITTED_OF(name) \
  NEED_LAYOUT_OF(name);         \
  if (c.no_plan) GTEST_SKIP() << "no committed plan plans/" << (name) << ".json";
// In a test of CommittedPlan or TamperedPlan: the layout its parameter
// indexes in StagedNames(). The parameters are indices, not names:
// ::testing::ValuesIn over strings makes GCC 14.2 crash.
#define NEED_COMMITTED() NEED_COMMITTED_OF(StagedNames()[GetParam()])

// A layout's index as a test's name.
std::string Short(int index) { return index == 0 ? "TwoPinArms" : "Balanced"; }

// The indices of StagedNames(), named.
const auto kStagedIndices = ::testing::Range(0, static_cast<int>(StagedNames().size()));
std::string ShortName(const ::testing::TestParamInfo<int>& i) { return Short(i.param); }

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

VerifyReport Verify(const Committed& c, const Plan& p, const VerifyOptions& o) {
  return VerifyPlanWithReport(c.layout, ModelPaths::FromEnvironment(), p, c.sha256, c.report, o);
}

// Without the edges and the schedule's samples: the rules of the steps.
VerifyOptions Quick() {
  VerifyOptions o;
  o.edges = false;
  o.schedule = false;
  return o;
}

BayEvent* Transfer(Plan& p, int bay, int to_stage) {
  for (BayEvent& e : p.events) {
    if (e.kind == "transfer" && e.bay == bay && e.to_stage == to_stage) return &e;
  }
  return nullptr;
}

// ------------------------------------------------------------- the scenes

TEST(Scenes, TheStagesDifferOnlyByTheTubesPlace) {
  NEED_LAYOUT();
  std::string error;
  const std::vector<std::unique_ptr<LayoutScene>> scenes =
      StageScenes(c.layout, ModelPaths::FromEnvironment(), 5.0, &error);
  ASSERT_EQ(scenes.size(), 3u) << error;
  const LayoutHandles& h = scenes[0]->handles();
  std::vector<bool> moves(scenes[0]->model().ngeom, false);
  for (int g : h.tube_geoms) moves[g] = true;
  for (const auto& r : h.rail_geoms) {
    for (int g : r) moves[g] = true;
  }
  for (const auto& m : h.module_geoms) {
    for (int g : m) moves[g] = true;
  }
  const DataPtr d0 = scenes[0]->MakeData();
  for (size_t s = 1; s < scenes.size(); ++s) {
    ASSERT_EQ(scenes[s]->model().ngeom, scenes[0]->model().ngeom);
    const DataPtr d = scenes[s]->MakeData();
    const Eigen::Vector3d off = StageOffset(c.layout, static_cast<int>(s));
    for (int g = 0; g < scenes[0]->model().ngeom; ++g) {
      for (int k = 0; k < 3; ++k) {
        const double want = moves[g] ? off[k] : 0.0;
        ASSERT_NEAR(d->geom_xpos[3 * g + k] - d0->geom_xpos[3 * g + k], want, 1e-9)
            << "stage " << s << ", geom " << scenes[0]->GeomName(g);
      }
      for (int k = 0; k < 9; ++k) {
        ASSERT_NEAR(d->geom_xmat[9 * g + k], d0->geom_xmat[9 * g + k], 1e-12);
      }
    }
  }
}

// --------------------------------------------------- a plan made up here

// The steps of one bay of the checker's report.
const BayStep* StepOf(const CheckReport& r, const std::string& kind, int stage_index,
                      const Layout& l, int nth = 0) {
  for (const BayStep& s : r.steps) {
    if (s.kind == kind && StageIndexOf(l, s) == stage_index && nth-- == 0) return &s;
  }
  return nullptr;
}

Config WaitOf(const LayoutScene& scene, int arm) {
  Config c{};
  for (int j = 0; j < 6; ++j) c[j] = scene.arm(arm).wait[j];
  c[6] = scene.arm(arm).wait_track;
  return c;
}

// One bay: tube_in, clamp, transfers at 100 and 200 s, release, bay_out,
// every arm waiting at its wait pose with its first tool throughout.
struct MadeUp {
  std::unique_ptr<LayoutScene> scene;
  Plan plan;
};
MadeUp MakeUp(const Committed& c) {
  MadeUp m;
  std::string error;
  m.scene = LayoutScene::Load(LayoutOfStage(c.layout, 0), ModelPaths::FromEnvironment(),
                              c.report.joint_margin_deg, &error);
  Plan& p = m.plan;
  p.layout = c.layout.name;
  auto event = [&](const std::string& kind, double t0, double t1, int from = 0) {
    BayEvent e{kind, 0, t0, t1};
    for (const BayStep& s : c.report.steps) {
      if (s.kind == kind && (kind != "transfer" || StageIndexOf(c.layout, s) == from)) {
        e.step = s.number;
      }
    }
    e.from_stage = kind == "transfer" ? from : 0;
    e.to_stage = kind == "transfer" ? from + 1 : 0;
    p.events.push_back(e);
  };
  event("tube_in", 0.0, 40.0);
  event("clamp", 40.0, 45.0);
  event("transfer", 100.0, 130.0, 1);
  event("transfer", 200.0, 230.0, 2);
  event("release", 300.0, 305.0);
  event("bay_out", 305.0, 365.0);
  for (int a = 0; m.scene && a < m.scene->ArmCount(); ++a) {
    ArmPlan row;
    row.name = m.scene->arm(a).use->name;
    Segment w;
    w.kind = "wait";
    w.t0 = 0.0;
    w.t1 = 365.0;
    w.tool = m.scene->arm(a).use->first_tool;
    w.path = {WaitOf(*m.scene, a)};
    w.reason = "made up";
    row.segments.push_back(w);
    p.arms.push_back(row);
  }
  return m;
}

// Puts a segment of `kind` and step `step` into arm a's row from t0 to t1,
// cutting its wait.
void Put(Plan& p, int a, const std::string& kind, int step, double t0, double t1,
         const std::vector<Config>& path) {
  std::vector<Segment>& row = p.arms[a].segments;
  Segment wait = row.front();
  Segment after = wait;
  wait.t1 = t0;
  after.t0 = t1;
  Segment s = wait;
  s.kind = kind;
  s.step = step;
  s.t0 = t0;
  s.t1 = t1;
  s.path = path;
  s.reason.clear();
  row = {wait, s, after};
}

TEST(MadeUp, AFineStagedBayHasNoFault) {
  NEED_LAYOUT();
  MadeUp m = MakeUp(c);
  ASSERT_TRUE(m.scene);
  const std::vector<std::string> f = StagedFaults(*m.scene, c.report, m.plan, VerifyOptions{});
  EXPECT_TRUE(f.empty()) << f.front();
}

TEST(MadeUp, TheTransfersAreHeld) {
  NEED_LAYOUT();
  auto faults = [&](const std::function<void(MadeUp&)>& change) {
    MadeUp m = MakeUp(c);
    change(m);
    VerifyReport r;
    r.faults = StagedFaults(*m.scene, c.report, m.plan, VerifyOptions{});
    return r;
  };
  EXPECT_TRUE(AnyFault(faults([](MadeUp& m) { m.plan.events[2].t1 -= 10.0; }),
                       {"the transfer to stage 2 takes 20.000 s", "transfer_s is 30.000 s"}));
  EXPECT_TRUE(AnyFault(faults([](MadeUp& m) { m.plan.events.erase(m.plan.events.begin() + 3); }),
                       {"the plan has 1 transfers"}));
  EXPECT_TRUE(AnyFault(faults([](MadeUp& m) { m.plan.events[2].to_stage = 3; }),
                       {"goes from stage 1 to 3, not 1 to 2"}));
  EXPECT_TRUE(AnyFault(faults([](MadeUp& m) { m.plan.arms[1].segments[0].path[0][0] += 0.2; }),
                       {"arm " + c.layout.arms[1].name, "is not at its wait pose",
                        "during the transfer to stage 2"}));
  EXPECT_TRUE(AnyFault(faults([](MadeUp& m) { m.plan.events[4].t0 = 220.0; }),
                       {"release starts at 220.000 s, before the last transfer ends"}));
  // A step of stage 2 before its transfer has ended; one of stage 1 after it
  // has started.
  const BayStep* second = StepOf(c.report, "rail_take", 1, c.layout);
  const BayStep* first = StepOf(c.report, "rail_take", 0, c.layout);
  ASSERT_NE(second, nullptr);
  ASSERT_NE(first, nullptr);
  int arm = 0;
  while (arm < static_cast<int>(c.layout.arms.size()) && c.layout.arms[arm].name != second->arm) {
    ++arm;
  }
  ASSERT_LT(arm, static_cast<int>(c.layout.arms.size()));
  EXPECT_TRUE(AnyFault(faults([&](MadeUp& m) {
                         const Config w = WaitOf(*m.scene, arm);
                         Put(m.plan, arm, "move", second->number, 125.0, 128.0, {w, w});
                       }),
                       {"starts its move of step " + std::to_string(second->number),
                        "before the transfer to stage 2 ends"}));
  EXPECT_TRUE(AnyFault(faults([&](MadeUp& m) {
                         const Config w = WaitOf(*m.scene, arm);
                         Put(m.plan, arm, "move", first->number, 98.0, 102.0, {w, w});
                       }),
                       {"ends its move of step " + std::to_string(first->number),
                        "after the transfer to stage 2 starts"}));
}

TEST(MadeUp, AModuleSetWaitsForTheMagazineAndItsPark) {
  NEED_LAYOUT();
  const BayStep* take = StepOf(c.report, "pins_take", 0, c.layout);
  const BayStep* set = StepOf(c.report, "module_set", 0, c.layout);
  ASSERT_NE(take, nullptr);
  ASSERT_NE(set, nullptr);
  MadeUp m = MakeUp(c);
  int a = 0;
  while (a < m.scene->ArmCount() && m.scene->arm(a).use->name != take->arm) ++a;
  ASSERT_LT(a, m.scene->ArmCount());
  const Config w = WaitOf(*m.scene, a);
  Config off = w;
  off[0] += 0.3;
  // The take's work from 50 s, then the park to the wait pose, 55 to 60 s.
  std::vector<Segment>& row = m.plan.arms[a].segments;
  Put(m.plan, a, "work", take->number, 50.0, 55.0, {off});
  Segment park = row[1];
  park.kind = "move";
  park.step = take->number + 1;
  park.t0 = 55.0;
  park.t1 = 60.0;
  park.path = {off, w};
  row.insert(row.begin() + 2, park);
  row[3].t0 = 60.0;
  StepTime st;
  st.number = set->number;
  st.what = set->what;
  st.start = 58.0;
  st.end = 65.0;
  m.plan.steps.push_back(st);
  VerifyReport r;
  r.faults = StagedFaults(*m.scene, c.report, m.plan, VerifyOptions{});
  EXPECT_TRUE(AnyFault(r, {set->what, "starts at 58.000 s", "before arm " + take->arm,
                           "parked at its wait pose at 60.000 s"}))
      << Faults(r);
  m.plan.steps.back().start = 60.0;
  EXPECT_TRUE(StagedFaults(*m.scene, c.report, m.plan, VerifyOptions{}).empty());
  // Without the park.
  row[2].path = {off, off};
  r.faults = StagedFaults(*m.scene, c.report, m.plan, VerifyOptions{});
  EXPECT_TRUE(AnyFault(r, {"arm " + take->arm, "does not park at its wait pose"})) << Faults(r);
}

TEST(MadeUp, AnUnstagedPlanHasNoTransfer) {
  std::string why;
  const std::optional<Layout> l =
      LoadLayout(Dir("LAYOUT_DIR", "layouts") + "/flat_rack_3_pin_arms_ordered.json", &why);
  ASSERT_TRUE(l.has_value()) << why;
  EXPECT_EQ(StageCount(*l), 1);
  EXPECT_EQ(StageOffset(*l, 0), Eigen::Vector3d::Zero());
  std::string error;
  const std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(*l, ModelPaths::FromEnvironment(), 5.0, &error);
  ASSERT_TRUE(scene) << error;
  Plan p;
  p.events.push_back({"transfer", 5, 10.0, 40.0});
  VerifyReport r;
  r.faults = StagedFaults(*scene, CheckReport{}, p, VerifyOptions{});
  EXPECT_TRUE(AnyFault(r, {"a transfer from 10.000 s, and the layout has no staging"}));
  EXPECT_EQ(StagePlaceAt(*l, {}, 0, 5.0).stage, 0);
}

TEST(MadeUp, TheTubeStandsAtItsStageAndMovesThroughATransfer) {
  NEED_LAYOUT();
  MadeUp m = MakeUp(c);
  const Eigen::Vector3d two = StageOffset(c.layout, 1), three = StageOffset(c.layout, 2);
  EXPECT_NEAR(two.y(), -3.432, 1e-9);
  EXPECT_NEAR(three.y(), -5.720, 1e-9);
  const StagePlace before = StagePlaceAt(c.layout, m.plan.events, 0, 99.0);
  EXPECT_EQ(before.stage, 0);
  EXPECT_FALSE(before.moving);
  EXPECT_EQ(before.offset, Eigen::Vector3d::Zero());
  const StagePlace mid = StagePlaceAt(c.layout, m.plan.events, 0, 115.0);
  EXPECT_EQ(mid.stage, 0);
  EXPECT_TRUE(mid.moving);
  EXPECT_NEAR(mid.offset.y(), 0.5 * two.y(), 1e-9);
  const StagePlace second = StagePlaceAt(c.layout, m.plan.events, 0, 150.0);
  EXPECT_EQ(second.stage, 1);
  EXPECT_NEAR(second.offset.y(), two.y(), 1e-9);
  const StagePlace last = StagePlaceAt(c.layout, m.plan.events, 0, 400.0);
  EXPECT_EQ(last.stage, 2);
  EXPECT_NEAR(last.offset.y(), three.y(), 1e-9);
  // Another bay's transfers do not move this one's tube.
  EXPECT_EQ(StagePlaceAt(c.layout, m.plan.events, 1, 150.0).stage, 0);
}

TEST(MadeUp, WaitsAreCutAtTransfersAndPhasesSumToTheBay) {
  NEED_LAYOUT();
  MadeUp m = MakeUp(c);
  WaitsThroughTransfers(c.layout, m.plan.events, &m.plan.arms);
  const std::vector<Segment>& row = m.plan.arms[0].segments;
  ASSERT_EQ(row.size(), 5u);
  EXPECT_EQ(row[1].t0, 100.0);
  EXPECT_EQ(row[1].t1, 130.0);
  EXPECT_EQ(row[1].reason, "while the tube moves to stage 2 (transfer)");
  EXPECT_EQ(row[3].reason, "while the tube moves to stage 3 (transfer)");
  EXPECT_EQ(row[4].reason, "made up");
  for (size_t k = 1; k < row.size(); ++k) EXPECT_EQ(row[k].t0, row[k - 1].t1);
  // Phases: every stage's work is "end" here, no step has a time.
  const std::vector<PhaseTime> phases =
      StagedPhases(c.layout, c.report, 0, {}, m.plan.events, 365.0);
  ASSERT_EQ(phases.size(), 6u);
  double sum = 0.0;
  for (const PhaseTime& p : phases) sum += p.planned_s;
  EXPECT_NEAR(sum, 365.0, 1e-9);
  EXPECT_EQ(phases[5].name, "transfer");
  EXPECT_NEAR(phases[5].planned_s, 60.0, 1e-9);
}

// R2: the next tube comes in at the first stage's place while the bay goes
// out from the last stage's, as soon as the two stay clearance_m apart; the
// ports of this layout do not share their zone.
TEST(MadeUp, TheNextTubeComesInWhileTheBayGoesOutFromTheLastStage) {
  NEED_LAYOUT();
  ASSERT_FALSE(WaysShareZone(c.layout));
  std::string error;
  const std::unique_ptr<LayoutScene> scene = LayoutScene::Load(
      LayoutOfStage(c.layout, 0), ModelPaths::FromEnvironment(), c.report.joint_margin_deg, &error);
  ASSERT_TRUE(scene) << error;
  const BayEvent out{"bay_out", 0, 100.0, 100.0 + c.layout.bay_out.time_s};
  const double empty = HolderEmptyAt(c.layout, out);
  const double start = NextTubeStart(*scene, out, 0.05, c.report.clearance_m);
  std::printf("holder empty at %.2f s, next tube from %.2f s, bay out at %.2f s: %.2f s of "
              "overlap\n",
              empty, start, out.t1, out.t1 - start);
  EXPECT_GE(start, empty);
  EXPECT_LT(start, out.t1);
  const DataPtr d = scene->MakeData();
  EXPECT_TRUE(TubePassesBay(*scene, d.get(), out, start, 0.05, c.report.clearance_m).clear);
  if (start > empty) {
    EXPECT_FALSE(
        TubePassesBay(*scene, d.get(), out, start - 0.05, 0.05, c.report.clearance_m).clear);
  }
}

// ----------------------------------------------------- the committed plan

class CommittedPlan : public ::testing::TestWithParam<int> {};

TEST_P(CommittedPlan, IsCompleteAndRoundTrips) {
  NEED_COMMITTED();
  EXPECT_TRUE(c.plan.complete);
  EXPECT_EQ(c.plan.bays, 3);
  EXPECT_EQ(PlanToJson(c.plan), c.text);
  EXPECT_EQ(c.plan.layout_sha256, c.sha256);
}

// AC4: each bay's events in time order are tube_in, clamp, transfer,
// transfer, release, bay_out; each transfer takes transfer_s.
TEST_P(CommittedPlan, EachBaysEventsAreInOrder) {
  NEED_COMMITTED();
  for (int b = 0; b < c.plan.bays; ++b) {
    std::vector<const BayEvent*> events;
    for (const BayEvent& e : c.plan.events) {
      if (e.bay == b) events.push_back(&e);
    }
    std::stable_sort(events.begin(), events.end(),
                     [](const BayEvent* x, const BayEvent* y) { return x->t0 < y->t0; });
    std::string kinds;
    for (const BayEvent* e : events) kinds += (kinds.empty() ? "" : ",") + e->kind;
    EXPECT_EQ(kinds, "tube_in,clamp,transfer,transfer,release,bay_out") << "bay " << b;
    for (const BayEvent* e : events) {
      if (e->kind != "transfer") continue;
      EXPECT_NEAR(e->t1 - e->t0, c.layout.staging->transfer_s, 1e-6);
      EXPECT_EQ(e->to_stage, e->from_stage + 1);
    }
  }
  // The next tube comes in while the bay before goes out, as far as the
  // clearances allow (src/layout_bays.h, TubePassesBay).
  for (int b = 0; b + 1 < c.plan.bays; ++b) {
    double out = 0.0, in = 0.0;
    for (const BayEvent& e : c.plan.events) {
      if (e.bay == b && e.kind == "bay_out") out = e.t1;
      if (e.bay == b + 1 && e.kind == "tube_in") in = e.t0;
    }
    std::printf("bay %d out and bay %d in overlap %.2f s\n", b, b + 1, out - in);
    EXPECT_LE(in, out + 1e-6);
  }
}

TEST_P(CommittedPlan, PassesTheRulesOfTheSteps) {
  NEED_COMMITTED();
  VerifyOptions o = Quick();
  o.edges = true;
  const VerifyReport r = Verify(c, c.plan, o);
  EXPECT_TRUE(r.ok) << r.faults.size() << " faults:\n" << Faults(r);
}

// The schedule's samples in pieces of the plan's time, which together cover
// it: each piece is one test.
// The parameter is the layout's index in StagedNames() times kPieces plus
// the piece.
class Schedule : public ::testing::TestWithParam<int> {};
constexpr int kPieces = 4;

TEST_P(Schedule, PassesTheVerifier) {
  NEED_COMMITTED_OF(StagedNames()[GetParam() / kPieces]);
  const int piece = GetParam() % kPieces;
  double end = 0.0;
  for (const ArmPlan& a : c.plan.arms) {
    if (!a.segments.empty()) end = std::max(end, a.segments.back().t1);
  }
  VerifyOptions o;
  o.edges = false;
  o.from_s = end * piece / kPieces;
  o.to_s = piece + 1 == kPieces ? -1.0 : end * (piece + 1) / kPieces;
  const VerifyReport r = Verify(c, c.plan, o);
  EXPECT_TRUE(r.ok) << r.faults.size() << " faults:\n" << Faults(r);
  const double floor = c.plan.figures.clearance_m - VerifyOptions{}.tolerance_m;
  EXPECT_GE(r.least_clearance_m, floor);
  EXPECT_GE(r.least_arm_to_arm_m, floor);
  std::printf("from %.1f s: least clearance %.4f m (%s to %s), least arm to arm %.4f m, least "
              "joint margin %.2f deg\n",
              o.from_s, r.least_clearance_m, r.least.empty() ? "" : r.least[0].a.c_str(),
              r.least.empty() ? "" : r.least[0].b.c_str(), r.least_arm_to_arm_m,
              r.least_joint_margin_deg);
}

INSTANTIATE_TEST_SUITE_P(Pieces, CommittedPlan, kStagedIndices, ShortName);
INSTANTIATE_TEST_SUITE_P(
    Pieces, Schedule,
    ::testing::Range(0, static_cast<int>(StagedNames().size()) * kPieces),
    [](const ::testing::TestParamInfo<int>& i) {
      return Short(i.param / kPieces) + "_" + std::to_string(i.param % kPieces);
    });

// --------------------------------------------------------------- tampering

class TamperedPlan : public ::testing::TestWithParam<int> {};

TEST_P(TamperedPlan, AStepBeforeItsTransferEndsIsFound) {
  NEED_COMMITTED();
  Plan p = c.plan;
  BayEvent* e = Transfer(p, 0, 2);
  ASSERT_NE(e, nullptr);
  e->t0 += 5.0;
  e->t1 += 5.0;
  const VerifyReport r = Verify(c, p, Quick());
  EXPECT_TRUE(AnyFault(r, {"arm", "starts its", "before the transfer to stage 2 ends"}))
      << Faults(r);
}

TEST_P(TamperedPlan, AStepAfterItsTransferStartsIsFound) {
  NEED_COMMITTED();
  Plan p = c.plan;
  BayEvent* e = Transfer(p, 1, 3);
  ASSERT_NE(e, nullptr);
  e->t0 -= 5.0;
  e->t1 -= 5.0;
  const VerifyReport r = Verify(c, p, Quick());
  EXPECT_TRUE(AnyFault(r, {"bay 1: arm", "ends its", "after the transfer to stage 3 starts"}))
      << Faults(r);
}

TEST_P(TamperedPlan, AnArmOffItsWaitPoseDuringATransferIsFound) {
  NEED_COMMITTED();
  Plan p = c.plan;
  const BayEvent* e = Transfer(p, 0, 2);
  ASSERT_NE(e, nullptr);
  const double mid = 0.5 * (e->t0 + e->t1);
  ArmPlan& a = p.arms.back();
  bool moved = false;
  for (Segment& g : a.segments) {
    if (g.kind == "wait" && g.t0 < mid && mid < g.t1 && !g.path.empty()) {
      g.path[0][0] += 0.2;
      moved = true;
    }
  }
  ASSERT_TRUE(moved);
  const VerifyReport r = Verify(c, p, Quick());
  EXPECT_TRUE(AnyFault(r, {"arm " + a.name, "is not at its wait pose",
                           "during the transfer to stage 2"}))
      << Faults(r);
  // The schedule's samples measure it against the tube on its way.
  VerifyOptions o;
  o.edges = false;
  o.from_s = mid;
  o.to_s = mid + 0.1;
  const VerifyReport s = Verify(c, p, o);
  EXPECT_FALSE(s.ok);
}

TEST_P(TamperedPlan, AShortTransferIsFound) {
  NEED_COMMITTED();
  Plan p = c.plan;
  BayEvent* e = Transfer(p, 2, 2);
  ASSERT_NE(e, nullptr);
  e->t1 -= 10.0;
  const VerifyReport r = Verify(c, p, Quick());
  EXPECT_TRUE(AnyFault(r, {"bay 2: the transfer to stage 2 takes 20.000 s",
                           "transfer_s is 30.000 s"}))
      << Faults(r);
}

TEST_P(TamperedPlan, AMissingTransferIsFound) {
  NEED_COMMITTED();
  Plan p = c.plan;
  const BayEvent* e = Transfer(p, 1, 3);
  ASSERT_NE(e, nullptr);
  p.events.erase(p.events.begin() + (e - p.events.data()));
  const VerifyReport r = Verify(c, p, Quick());
  EXPECT_TRUE(AnyFault(r, {"bay 1: the plan has 1 transfers"})) << Faults(r);
}

TEST_P(TamperedPlan, AModuleSetBeforeTheMagazineIsFound) {
  NEED_COMMITTED();
  Plan p = c.plan;
  // The first module set of bay 0: its start, as the plan says it, put
  // before the magazine takes of its stage.
  StepTime* set = nullptr;
  for (StepTime& t : p.steps) {
    if (set == nullptr && t.what.rfind("set module", 0) == 0) set = &t;
  }
  ASSERT_NE(set, nullptr);
  const BayEvent* clamp = nullptr;
  for (const BayEvent& e : p.events) {
    if (clamp == nullptr && e.kind == "clamp") clamp = &e;
  }
  ASSERT_NE(clamp, nullptr);
  set->start = clamp->t1;
  const VerifyReport r = Verify(c, p, Quick());
  EXPECT_TRUE(AnyFault(r, {set->what, "before arm", "has taken its pin magazine"})) << Faults(r);
}

INSTANTIATE_TEST_SUITE_P(Pieces, TamperedPlan, kStagedIndices, ShortName);

}  // namespace
}  // namespace cell::layout
