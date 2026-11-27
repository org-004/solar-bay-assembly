// The committed plan of the chosen layout, verified whole without planning
// anything (R20), broken and tampered with in ways the verifier must find,
// and, once it is a plan of several bays (src/layout_bays.h), doctored
// between its bays. The tests fail until plans/<chosen>.json exists for a
// chosen layout on the flat rack: passing only with a real plan is what they
// are for. The chosen layout is the one versions/versions.json names
// (src/layout.h, ChosenLayout). The tests of the bays are skipped while the
// committed plan has one bay. The replay's pieces are
// tests/plan_replay_test.cc's.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"

namespace cell::layout {
namespace {

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
    // segments of the rail phase moved earlier together, to 1 s before the
    // set's end. The steps are found by their names in the plan's step times;
    // the bolt is the first one whose wait before it is long enough for the
    // move (run 043: in v17's plan the first bolt waits only 0.25 s).
    Plan p = c.plan;
    const StepTime* bolt = nullptr;
    int rail = -1;
    int pa = -1;
    int first = -1;
    double shift = 0.0;
    for (const StepTime& b : p.steps) {
      int r = -1;
      if (std::sscanf(b.what.c_str(), "bolt rail %d", &r) != 1) continue;
      int set_step = -1;
      for (const StepTime& s : p.steps) {
        if (set_step < 0 && s.what == "set rail " + std::to_string(r)) set_step = s.number;
      }
      int arm_index = -1;
      for (size_t k = 0; k < p.arms.size(); ++k) {
        if (p.arms[k].name == b.arm) arm_index = static_cast<int>(k);
      }
      if (set_step < 0 || arm_index < 0) continue;
      double set_end = -1.0;
      for (const ArmPlan& a : p.arms) {
        for (const Segment& s : a.segments) {
          if (s.step == set_step && s.kind == "work") set_end = s.t1;
        }
      }
      if (set_end <= 0.0) continue;
      const std::vector<Segment>& segments = p.arms[arm_index].segments;
      int f = -1;
      for (size_t i = 0; i < segments.size() && f < 0; ++i) {
        if (segments[i].step == b.number && segments[i].kind != "wait") f = static_cast<int>(i);
      }
      if (f <= 0 || segments[f - 1].kind != "wait") continue;
      const double sh = segments[f].t0 - set_end + 1.0;
      if (sh >= segments[f - 1].t1 - segments[f - 1].t0) continue;
      bolt = &b;
      rail = r;
      pa = arm_index;
      first = f;
      shift = sh;
      break;
    }
    ASSERT_NE(bolt, nullptr) << "no strap bolt with a wait before it long enough to move it before its rail's set ends";
    std::vector<Segment>& bs = p.arms[pa].segments;
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
    // A pin arm's longest wait before a locate put at that locate's work pose: the
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
          // Only a wait whose next work is the locate.
          const StepTime* st = StepOf(p, as[j].step);
          if (st != nullptr && Begins(st->what, "locate ")) {
            pa = static_cast<int>(k);
            g = static_cast<int>(i);
            w = static_cast<int>(j);
            longest = as[i].t1 - as[i].t0;
          }
          break;
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

// ----------------------------------------------------- bays back to back

// The committed plan of several bays doctored between its bays: each change
// must fault. Skipped while the committed plan has one bay.
BayEvent* EventOf(Plan& p, int bay, const std::string& kind) {
  for (BayEvent& e : p.events) {
    if (e.bay == bay && e.kind == kind) return &e;
  }
  return nullptr;
}

TEST(Bays, TheNextTubeBeforeTheBayIsOutIsFound) {
  const Committed& c = TheCommitted();
  ASSERT_TRUE(c.error.empty()) << c.error;
  if (c.plan.bays < 2) GTEST_SKIP() << "the committed plan has one bay";
  Plan p = c.plan;
  BayEvent* in = EventOf(p, 1, "tube_in");
  ASSERT_NE(in, nullptr);
  in->t0 -= 10.0;
  in->t1 -= 10.0;
  const VerifyReport r = VerifyPlan(c.layout, ModelPaths::FromEnvironment(), p, c.sha256,
                                    Quick(false));
  EXPECT_TRUE(AnyFault(r, {"bay 1: tube_in starts", "before bay 0's bay_out ends"}))
      << Faults(r);
}

TEST(Bays, AnArmOffItsWaitPoseDuringABayOutIsFound) {
  const Committed& c = TheCommitted();
  ASSERT_TRUE(c.error.empty()) << c.error;
  if (c.plan.bays < 2) GTEST_SKIP() << "the committed plan has one bay";
  Plan p = c.plan;
  const BayEvent* out = EventOf(p, 0, "bay_out");
  ASSERT_NE(out, nullptr);
  const double mid = 0.5 * (out->t0 + out->t1);
  ArmPlan& a = p.arms.back();
  bool moved = false;
  for (Segment& g : a.segments) {
    if (g.kind == "wait" && g.t0 < mid && mid < g.t1 && !g.path.empty()) {
      g.path[0][0] += 0.2;
      moved = true;
    }
  }
  ASSERT_TRUE(moved);
  const VerifyReport r = VerifyPlan(c.layout, ModelPaths::FromEnvironment(), p, c.sha256,
                                    Quick(false));
  EXPECT_TRUE(AnyFault(r, {"bay 0: arm " + a.name, "is not at its wait pose", "during bay_out"}))
      << Faults(r);
}

TEST(Bays, AnArmOfTheNextBayBeforeItsClampEndsIsFound) {
  const Committed& c = TheCommitted();
  ASSERT_TRUE(c.error.empty()) << c.error;
  if (c.plan.bays < 2) GTEST_SKIP() << "the committed plan has one bay";
  // Bay 1's tube and clamp 30 s later: its arms now start before its clamp
  // has ended.
  Plan p = c.plan;
  for (const char* kind : {"tube_in", "clamp"}) {
    BayEvent* e = EventOf(p, 1, kind);
    ASSERT_NE(e, nullptr);
    e->t0 += 30.0;
    e->t1 += 30.0;
  }
  const VerifyReport r = VerifyPlan(c.layout, ModelPaths::FromEnvironment(), p, c.sha256,
                                    Quick(false));
  EXPECT_TRUE(AnyFault(r, {"bay 1: arm", "before the bay's clamp ends"})) << Faults(r);
  // The first such step touches the bay: a set, never a take (R10).
  for (const std::string& f : r.faults) {
    if (f.find("before the bay's clamp ends") == std::string::npos) continue;
    const size_t at = f.find(" of step ");
    ASSERT_NE(at, std::string::npos) << f;
    const long step = std::strtol(f.c_str() + at + 9, nullptr, 10);
    for (const StepTime& t : p.steps) {
      if (t.number == step) EXPECT_FALSE(Begins(t.what, "take ")) << f;
    }
  }
}

TEST(Bays, AnEarlyTakeBeforeTheClampVerifies) {
  const Committed& c = TheCommitted();
  ASSERT_TRUE(c.error.empty()) << c.error;
  if (c.plan.bays < 2) GTEST_SKIP() << "the committed plan has one bay";
  // R10: a later bay's first take of a rail may run while its tube comes in.
  const BayEvent* clamp = nullptr;
  for (const BayEvent& e : c.plan.events) {
    if (e.bay == 1 && e.kind == "clamp") clamp = &e;
  }
  ASSERT_NE(clamp, nullptr);
  const StepTime* take = nullptr;
  for (const StepTime& t : c.plan.steps) {
    if (t.bay == 1 && take == nullptr && Begins(t.what, "take rail ")) take = &t;
  }
  ASSERT_NE(take, nullptr);
  if (take->start >= clamp->t1) GTEST_SKIP() << "the committed plan has no early take";
  const VerifyReport r = VerifyPlan(c.layout, ModelPaths::FromEnvironment(), c.plan, c.sha256,
                                    Quick(false));
  EXPECT_TRUE(r.ok) << Faults(r);
}

TEST(Bays, ATakeAtTheFirstBaysPalletPointIsFound) {
  const Committed& c = TheCommitted();
  ASSERT_TRUE(c.error.empty()) << c.error;
  if (c.plan.bays < 2) GTEST_SKIP() << "the committed plan has one bay";
  // Bay 1's first take done at pallet point k, bay 0's, not at 9 + k.
  Plan p = c.plan;
  const int ma = ModuleArmPlan(c);
  ASSERT_GE(ma, 0);
  const int n = static_cast<int>(p.steps.size()) / p.bays;
  int take = -1, per_bay = 0;
  for (const StepTime& s : p.steps) {
    if (s.bay != 0 || !Begins(s.what, "take module ")) continue;
    if (take < 0) take = s.number;
    ++per_bay;
  }
  ASSERT_GT(take, 0);
  std::vector<Segment>& segs = p.arms[ma].segments;
  int copied = 0;
  for (Segment& later : segs) {
    if (later.step != take + n || later.kind == "wait" || later.kind == "move") continue;
    for (const Segment& first : c.plan.arms[ma].segments) {
      if (first.step == take && first.kind == later.kind) {
        later.path = first.path;
        ++copied;
      }
    }
  }
  ASSERT_GT(copied, 0);
  const VerifyReport r = VerifyPlan(c.layout, ModelPaths::FromEnvironment(), p, c.sha256,
                                    Quick(false));
  EXPECT_TRUE(AnyFault(r, {"step " + std::to_string(take + n),
                           "pose of module_take " + std::to_string(per_bay)}))
      << Faults(r);
}

}  // namespace
}  // namespace cell::layout
