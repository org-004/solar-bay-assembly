// Bays back to back (src/layout_bays.h) on small cases: the numbering, the
// takes, the bay as built, the joining of plans made here by hand, the
// cycle's figures, and when the next tube comes in on the chosen layout and
// on copies of it whose ways do not share their zone. No bay is planned:
// that is //:plan_layout's work.

#include <algorithm>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"
#include "src/mj_handles.h"

namespace cell::layout {
namespace {

const char* kName = "flat_rack_3_pin_arms_ordered";

Layout Load() {
  const char* dir = std::getenv("LAYOUT_DIR");
  const std::string path =
      std::string(dir != nullptr ? dir : "layouts") + "/" + kName + ".json";
  std::string error;
  std::optional<Layout> layout = LoadLayout(path, &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout ? *layout : Layout{};
}

// ------------------------------------------------------------ numbering

TEST(Numbering, StepsOfEachBayAndBack) {
  const int n = 173;
  for (int b = 0; b < 3; ++b) {
    for (int k = 1; k <= n; ++k) {
      const int number = StepNumberInBay(k, b, n);
      EXPECT_EQ(number, k + b * n);
      EXPECT_EQ(BayOfStep(number, n), b);
      EXPECT_EQ(StepOfBay(number, n), k);
    }
  }
  EXPECT_EQ(StepNumberInBay(0, 2, n), 0);
  EXPECT_EQ(BayOfStep(0, n), 0);
}

BayStep Step(int number, const std::string& kind, int index) {
  BayStep s;
  s.number = number;
  s.kind = kind;
  s.index = index;
  s.what = kind + " " + std::to_string(index);
  return s;
}

TEST(Numbering, StepsOfBaysMoveOnlyTheModuleTakes) {
  CheckReport report;
  report.steps = {Step(1, "tube_in", -1),    Step(2, "rail_take", 0),
                  Step(3, "rail_set", 0),    Step(4, "module_take", 0),
                  Step(5, "module_set", 1),  Step(6, "module_take", 1),
                  Step(7, "module_set", 0),  Step(8, "pins_take", 0),
                  Step(9, "pin", 5),         Step(10, "bay_out", -1)};
  ASSERT_EQ(ModulesPerBay(report), 2);
  const std::vector<StepInBay> all = StepsOfBays(report, 3);
  ASSERT_EQ(all.size(), 30u);
  for (size_t i = 0; i < all.size(); ++i) {
    const int b = static_cast<int>(i / 10);
    const BayStep& own = report.steps[i % 10];
    const StepInBay& s = all[i];
    EXPECT_EQ(s.bay, b);
    EXPECT_EQ(s.step.number, own.number + 10 * b);
    EXPECT_EQ(s.step.index, own.kind == "module_take" ? own.index + 2 * b : own.index)
        << own.kind;
    EXPECT_EQ(s.step.kind, own.kind);
    EXPECT_EQ(s.step.what, own.what);
    EXPECT_EQ(PointIndexInBay(own, b, 2), s.step.index);
  }
}

TEST(Built, AMoveOfALaterBayHasFewerModulesOnThePallets) {
  Built b0;
  b0.rails = {0, 1, 2};
  b0.modules = {0, 1};
  b0.on_pallets = 34;
  for (int bay = 0; bay < 3; ++bay) {
    const Built b = BuiltInBay(b0, bay, 9);
    EXPECT_EQ(b.rails, b0.rails);
    EXPECT_EQ(b.modules, b0.modules);
    EXPECT_EQ(b.on_pallets, 34 - 9 * bay);
  }
  b0.on_pallets = 5;
  EXPECT_EQ(BuiltInBay(b0, 2, 9).on_pallets, 0);
}

// ------------------------------------------------------------ joining

Segment Seg(const std::string& kind, int step, double t0, double t1) {
  Segment s;
  s.kind = kind;
  s.step = step;
  s.t0 = t0;
  s.t1 = t1;
  s.path = {Config{}};
  if (kind == "wait") s.reason = "test";
  return s;
}

// One bay of six steps: tube_in (1), clamp (2), arm a's work (3, 4), release
// (5), bay_out (6); arm b only waits.
Plan OneBay() {
  Plan p;
  p.layout = "test";
  p.complete = true;
  p.events = {{"tube_in", 1, 0.0, 40.0},
              {"clamp", 2, 40.0, 45.0},
              {"release", 5, 90.0, 95.0},
              {"bay_out", 6, 100.0, 140.0}};
  p.steps = {{1, "tube in", "", 0.0, 40.0},   {2, "clamp", "", 40.0, 45.0},
             {3, "move", "a", 45.0, 60.0},    {4, "work", "a", 60.0, 90.0},
             {5, "release", "", 90.0, 95.0},  {6, "bay out", "", 100.0, 140.0}};
  p.phases = {{"tube in", 40.0, 40.0}, {"clamp", 5.0, 5.0}};
  p.bay_planned_s = 140.0;
  ArmPlan a{"a", {Seg("wait", 2, 0.0, 45.0), Seg("move", 3, 45.0, 60.0),
                  Seg("work", 4, 60.0, 90.0), Seg("wait", 6, 90.0, 140.0)}};
  ArmPlan b{"b", {Seg("wait", 2, 0.0, 45.0), Seg("wait", 6, 45.0, 140.0)}};
  p.arms = {a, b};
  return p;
}

void ExpectNoGaps(const Plan& p, double end) {
  for (const ArmPlan& a : p.arms) {
    ASSERT_FALSE(a.segments.empty());
    EXPECT_EQ(a.segments.front().t0, 0.0) << a.name;
    for (size_t k = 1; k < a.segments.size(); ++k) {
      EXPECT_EQ(a.segments[k].t0, a.segments[k - 1].t1) << a.name << " " << k;
      EXPECT_LE(a.segments[k].t0, a.segments[k].t1) << a.name << " " << k;
    }
    EXPECT_EQ(a.segments.back().t1, end) << a.name;
  }
}

const BayEvent& Event(const Plan& p, int bay, const std::string& kind) {
  for (const BayEvent& e : p.events) {
    if (e.bay == bay && e.kind == kind) return e;
  }
  static const BayEvent none;
  ADD_FAILURE() << "no " << kind << " of bay " << bay;
  return none;
}

TEST(JoinBays, ASharedCartPutsEachBayAfterTheLast) {
  const Plan one = OneBay();
  const Plan p = JoinBays({one, one, one}, 6,
                          [](const BayEvent& out) { return out.t1; });
  EXPECT_EQ(p.bays, 3);
  EXPECT_TRUE(p.complete);
  EXPECT_EQ(p.bay_planned_s, 140.0);
  ASSERT_EQ(p.events.size(), 12u);
  ASSERT_EQ(p.steps.size(), 18u);
  ASSERT_EQ(p.phases.size(), 6u);
  for (size_t i = 0; i < p.events.size(); ++i) {
    const int b = static_cast<int>(i / 4);
    const BayEvent& own = one.events[i % 4];
    EXPECT_EQ(p.events[i].bay, b);
    EXPECT_EQ(p.events[i].kind, own.kind);
    EXPECT_EQ(p.events[i].step, own.step + 6 * b);
    EXPECT_EQ(p.events[i].t0, own.t0 + 140.0 * b);
    EXPECT_EQ(p.events[i].t1, own.t1 + 140.0 * b);
  }
  for (size_t i = 0; i < p.steps.size(); ++i) {
    const int b = static_cast<int>(i / 6);
    EXPECT_EQ(p.steps[i].bay, b);
    EXPECT_EQ(p.steps[i].number, one.steps[i % 6].number + 6 * b);
    EXPECT_EQ(p.steps[i].start, one.steps[i % 6].start + 140.0 * b);
  }
  for (size_t i = 0; i < p.phases.size(); ++i) {
    EXPECT_EQ(p.phases[i].bay, static_cast<int>(i / 2));
  }
  ExpectNoGaps(p, 420.0);
  ASSERT_EQ(p.arms[0].segments.size(), 12u);
  const Segment& work2 = p.arms[0].segments[10];
  EXPECT_EQ(work2.kind, "work");
  EXPECT_EQ(work2.step, 4 + 12);
  EXPECT_EQ(work2.t0, 60.0 + 280.0);
  // Bay 0 is as it was.
  for (size_t k = 0; k < 4; ++k) {
    EXPECT_EQ(p.arms[0].segments[k].t0, one.arms[0].segments[k].t0);
    EXPECT_EQ(p.arms[0].segments[k].step, one.arms[0].segments[k].step);
  }
}

TEST(JoinBays, AnEarlierTubeCutsTheFirstWait) {
  const Plan one = OneBay();
  // The next tube comes in 30 s before the bay is out: its clamp ends at
  // 110 + 45 = 155 s, after the bay is out at 140 s.
  const Plan p = JoinBays({one, one}, 6,
                          [](const BayEvent& out) { return out.t1 - 30.0; });
  EXPECT_EQ(Event(p, 1, "tube_in").t0, 110.0);
  EXPECT_EQ(Event(p, 1, "clamp").t1, 155.0);
  EXPECT_EQ(Event(p, 1, "bay_out").t1, 250.0);
  ExpectNoGaps(p, 250.0);
  const Segment& first = p.arms[0].segments[4];
  EXPECT_EQ(first.kind, "wait");
  EXPECT_EQ(first.t0, 140.0);
  EXPECT_EQ(first.t1, 155.0);
  EXPECT_EQ(p.arms[0].segments[5].t0, 155.0);  // Its move: after the clamp.
  EXPECT_EQ(p.steps[6 + 2].start, 155.0);
  EXPECT_EQ(p.steps[6].start, 110.0);  // The tube's step.
}

TEST(JoinBays, NoArmStartsBeforeTheLastBayIsOut) {
  const Plan one = OneBay();
  // A tube 50 s early would let the arms start at 135 s, while the last bay
  // still goes out: they and release and bay_out are put off by 5 s.
  const Plan p = JoinBays({one, one}, 6,
                          [](const BayEvent& out) { return out.t1 - 50.0; });
  EXPECT_EQ(Event(p, 1, "tube_in").t0, 90.0);
  EXPECT_EQ(Event(p, 1, "clamp").t1, 135.0);
  EXPECT_EQ(Event(p, 1, "release").t0, 90.0 + 95.0);
  EXPECT_EQ(Event(p, 1, "bay_out").t1, 95.0 + 140.0);
  ExpectNoGaps(p, 235.0);
  for (const ArmPlan& a : p.arms) {
    for (const Segment& s : a.segments) {
      if (s.kind != "wait" && s.step > 6) {
        EXPECT_GE(s.t0, 140.0);
      }
    }
  }
}

TEST(JoinBays, ALaterTubeDrawsOutTheLastWait) {
  const Plan one = OneBay();
  const Plan p = JoinBays({one, one}, 6,
                          [](const BayEvent& out) { return out.t1 + 10.0; });
  EXPECT_EQ(Event(p, 1, "tube_in").t0, 150.0);
  ExpectNoGaps(p, 290.0);
  EXPECT_EQ(p.arms[1].segments[1].t1, 150.0);
  EXPECT_EQ(p.arms[1].segments[2].t0, 150.0);
}

TEST(JoinBays, OnePlanIsAsItWasAndAFileOfBaysReadsBack) {
  const Plan one = OneBay();
  const Plan same = JoinBays({one}, 6, [](const BayEvent& out) { return out.t1; });
  EXPECT_EQ(PlanToJson(same), PlanToJson(one));
  EXPECT_EQ(PlanToJson(one).find("\"bay"), PlanToJson(one).find("\"bay_planned_s\""));
  const Plan three = JoinBays({one, one, one}, 6,
                              [](const BayEvent& out) { return out.t1; });
  const std::string text = PlanToJson(three);
  std::string error;
  const std::optional<Plan> back = PlanFromJson(text, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(back->bays, 3);
  EXPECT_EQ(back->events.back().bay, 2);
  EXPECT_EQ(back->steps.back().bay, 2);
  EXPECT_EQ(back->phases.back().bay, 2);
  EXPECT_EQ(PlanToJson(*back), text);
  const std::optional<Plan> single = PlanFromJson(PlanToJson(one), &error);
  ASSERT_TRUE(single.has_value()) << error;
  EXPECT_EQ(single->bays, 1);
  EXPECT_EQ(single->events.back().bay, 0);
}

TEST(CycleFigures, OfOneBayAndOfThree) {
  const Plan one = OneBay();
  const CycleFigures c1 = CycleFiguresOf(one);
  EXPECT_EQ(c1.bays, 1);
  EXPECT_EQ(c1.one_bay_s, 140.0);
  EXPECT_EQ(c1.cycle_s, 140.0);
  EXPECT_EQ(c1.interval_s, 0.0);
  const Plan three = JoinBays({one, one, one}, 6,
                              [](const BayEvent& out) { return out.t1 - 30.0; });
  const CycleFigures c3 = CycleFiguresOf(three);
  EXPECT_EQ(c3.bays, 3);
  EXPECT_EQ(c3.one_bay_s, 140.0);
  // Each later bay ends 110 s after the one before.
  EXPECT_DOUBLE_EQ(c3.interval_s, 110.0);
  EXPECT_DOUBLE_EQ(c3.cycle_s, (140.0 + 2 * 110.0) / 3.0);
}

// ---------------------------------------------------------------- the ways

struct Ways {
  Layout layout;
  std::unique_ptr<LayoutScene> scene;
};

Ways WaysOf(const Layout& layout) {
  Ways w;
  w.layout = layout;
  std::string error;
  w.scene = LayoutScene::Load(w.layout, ModelPaths::FromEnvironment(),
                              w.layout.margins.joint_margin_deg, &error);
  EXPECT_NE(w.scene, nullptr) << error;
  return w;
}

// v10's bay_out, as planned for one bay.
BayEvent V10BayOut() { return {"bay_out", 166, 520.452329, 560.452329}; }

Layout Unshared(Layout l) {
  for (Port& p : l.ports) {
    if (p.name == "tubes_in" || p.name == "bays_out") p.shares_zone_with.clear();
  }
  return l;
}

TEST(NextTube, TheChosenLayoutsCartWaitsForTheBayToBeOut) {
  const Layout layout = Load();
  EXPECT_TRUE(WaysShareZone(layout));
  const BayEvent out = V10BayOut();
  // The lift is 0.12 m of the way's 3.02 m.
  EXPECT_NEAR(HolderEmptyAt(layout, out), out.t0 + 40.0 * 0.12 / 3.02, 1e-9);
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  EXPECT_EQ(NextTubeStart(*w.scene, out, 0.05, layout.margins.clearance_m), out.t1);
}

TEST(NextTube, WaysInOneLaneCannotPass) {
  // Without the shared zone the tube still comes in along the lane the bay
  // leaves by, the other way: it cannot pass the bay before the bay is out.
  const Layout layout = Unshared(Load());
  EXPECT_FALSE(WaysShareZone(layout));
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  const BayEvent out = V10BayOut();
  const double clear = layout.margins.clearance_m;
  const double empty = HolderEmptyAt(layout, out);
  EXPECT_EQ(NextTubeStart(*w.scene, out, 0.05, clear), out.t1);
  DataPtr d = w.scene->MakeData();
  const WayPassing at_empty = TubePassesBay(*w.scene, d.get(), out, empty, 0.05, clear);
  EXPECT_FALSE(at_empty.clear);
  EXPECT_GT(at_empty.t, empty);
}

TEST(NextTube, ATubeFromAboveComesInWhileTheBayGoesOut) {
  // A copy whose tube is lowered from 1.5 m above its place in 10 s: it
  // may start once the bay has moved far enough out from under it.
  Layout layout = Unshared(Load());
  layout.tube_in.path = {Vec3{0.0, 0.0, 1.5}, Vec3{0.0, 0.0, 0.0}};
  layout.tube_in.time_s = 10.0;
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  const BayEvent out = V10BayOut();
  const double clear = layout.margins.clearance_m;
  const double empty = HolderEmptyAt(layout, out);
  const double start = NextTubeStart(*w.scene, out, 0.05, clear);
  EXPECT_GT(start, empty);
  EXPECT_LT(start, out.t1);
  DataPtr d = w.scene->MakeData();
  EXPECT_TRUE(TubePassesBay(*w.scene, d.get(), out, start, 0.05, clear).clear);
  EXPECT_FALSE(TubePassesBay(*w.scene, d.get(), out, start - 0.05, 0.05, clear).clear);
  // The data's geoms are back where they were.
  DataPtr fresh = w.scene->MakeData();
  for (int i = 0; i < 3 * w.scene->model().ngeom; ++i) {
    ASSERT_EQ(d->geom_xpos[i], fresh->geom_xpos[i]) << i;
  }
}

// The layout with a separate infeed unit (run 053): its tube comes in over
// the bay going out, 0.45 m above its place, from the moment the holder is
// empty. Its bay_out is _clear_out's one bay's.
const char* kInfeed =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed";

Layout LoadNamed(const std::string& name) {
  const char* dir = std::getenv("LAYOUT_DIR");
  const std::string path = std::string(dir != nullptr ? dir : "layouts") + "/" + name + ".json";
  std::string error;
  std::optional<Layout> layout = LoadLayout(path, &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout ? *layout : Layout{};
}

TEST(NextTube, TheInfeedBringsTheTubeInOverTheBayGoingOut) {
  const Layout layout = LoadNamed(kInfeed);
  EXPECT_FALSE(WaysShareZone(layout));
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  const BayEvent out{"bay_out", 173, 456.59, 496.59};
  const double clear = layout.margins.clearance_m;
  const double empty = HolderEmptyAt(layout, out);
  EXPECT_DOUBLE_EQ(NextTubeStart(*w.scene, out, 0.05, clear), empty);
  DataPtr d = w.scene->MakeData();
  const WayPassing p = TubePassesBay(*w.scene, d.get(), out, empty, 0.05, clear);
  EXPECT_TRUE(p.clear);
  // Lowered 0.30 m less, the tube would meet the bay's modules on its way over.
  Layout low = layout;
  for (Vec3& v : low.tube_in.path) v[2] = std::max(0.0, v[2] - 0.30);
  const Ways lw = WaysOf(low);
  ASSERT_NE(lw.scene, nullptr);
  DataPtr ld = lw.scene->MakeData();
  EXPECT_FALSE(TubePassesBay(*lw.scene, ld.get(), out, empty, 0.05, clear).clear);
  EXPECT_GT(NextTubeStart(*lw.scene, out, 0.05, clear), empty);
}

// ------------------------------------------------------------ checking

// Two bays' events, the second's tube_in from `tube_start`, on the chosen
// layout's figures, and one arm whose bay 1 move starts at `move_at`.
Plan TwoBays(const Layout& layout, double tube_start, double move_at) {
  Plan p;
  p.bays = 2;
  p.figures.clearance_m = layout.margins.clearance_m;
  const double in = layout.tube_in.time_s, clamp = layout.holder.clamp_s;
  p.events = {{"tube_in", 1, 0.0, in, 0},
              {"clamp", 2, in, in + clamp, 0},
              {"release", 5, 90.0, 95.0, 0},
              {"bay_out", 6, 100.0, 100.0 + layout.bay_out.time_s, 0},
              {"tube_in", 7, tube_start, tube_start + in, 1},
              {"clamp", 8, tube_start + in, tube_start + in + clamp, 1}};
  ArmPlan a{"module_arm", {Seg("wait", 8, 0.0, move_at), Seg("move", 9, move_at, move_at + 5.0)}};
  p.arms = {a};
  return p;
}

// The checker's six steps of a bay of TwoBays; step 3 is of `third` kind.
CheckReport SixSteps(const std::string& third) {
  CheckReport r;
  r.steps = {Step(1, "tube_in", -1), Step(2, "clamp", -1), Step(3, third, 0),
             Step(4, "strap_bolt", 0), Step(5, "release", -1), Step(6, "bay_out", -1)};
  r.steps[2].arm = "module_arm";
  return r;
}

TEST(BetweenBays, ATakeMayStartBeforeTheClampEndsASetMayNot) {
  const Layout layout = Load();
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  const double out_end = 100.0 + layout.bay_out.time_s;
  const Plan p = TwoBays(layout, out_end, out_end + 1.0);
  EXPECT_TRUE(BetweenBays(*w.scene, p, SixSteps("rail_take"), 1e-3, 1e-4).empty());
  const std::vector<std::string> set = BetweenBays(*w.scene, p, SixSteps("rail_set"), 1e-3, 1e-4);
  ASSERT_EQ(set.size(), 1u);
  EXPECT_NE(set[0].find("before the bay's clamp ends"), std::string::npos) << set[0];
  EXPECT_FALSE(TouchesTheBay("rail_take"));
  EXPECT_FALSE(TouchesTheBay("module_take"));
  EXPECT_FALSE(TouchesTheBay("pins_take"));
  EXPECT_FALSE(TouchesTheBay("tool_change"));
  for (const char* k : {"rail_set", "strap_bolt", "module_set", "locate", "pin", "swage"}) {
    EXPECT_TRUE(TouchesTheBay(k)) << k;
  }
}

TEST(BetweenBays, ASharedCartWaitsForTheBayToBeOut) {
  const Layout layout = Load();
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  const double out_end = 100.0 + layout.bay_out.time_s;
  const double clamp_end = out_end + layout.tube_in.time_s + layout.holder.clamp_s;
  EXPECT_TRUE(BetweenBays(*w.scene, TwoBays(layout, out_end, clamp_end), SixSteps("rail_set"), 1e-3, 1e-4).empty());
  const std::vector<std::string> early =
      BetweenBays(*w.scene, TwoBays(layout, out_end - 10.0, clamp_end), SixSteps("rail_set"), 1e-3, 1e-4);
  ASSERT_EQ(early.size(), 1u);
  EXPECT_NE(early[0].find("before bay 0's bay_out ends"), std::string::npos) << early[0];
  const std::vector<std::string> arm =
      BetweenBays(*w.scene, TwoBays(layout, out_end, clamp_end - 1.0), SixSteps("rail_set"), 1e-3, 1e-4);
  ASSERT_EQ(arm.size(), 1u);
  EXPECT_NE(arm[0].find("arm module_arm starts its move of step 9"), std::string::npos) << arm[0];
}

TEST(BetweenBays, WithoutASharedZoneTheTubeMustPassTheBay) {
  const Layout layout = Unshared(Load());
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  const BayEvent out{"bay_out", 6, 100.0, 100.0 + layout.bay_out.time_s};
  const double empty = HolderEmptyAt(layout, out);
  // Before the holder is empty, and in the lane while the bay is in it.
  const std::vector<std::string> before = BetweenBays(
      *w.scene, TwoBays(layout, empty - 0.5, 1000.0), SixSteps("rail_set"), 1e-3, 1e-4);
  ASSERT_EQ(before.size(), 2u);
  EXPECT_NE(before[0].find("before the holder is empty"), std::string::npos) << before[0];
  EXPECT_NE(before[1].find("its tube coming in is"), std::string::npos) << before[1];
  // Once the bay is out there is nothing to pass.
  EXPECT_TRUE(BetweenBays(*w.scene, TwoBays(layout, out.t1, 1000.0), SixSteps("rail_set"), 1e-3, 1e-4).empty());
}

TEST(Player, EachBayStartsBareAndThePalletCountsOn) {
  const Layout layout = Load();
  const Ways w = WaysOf(layout);
  ASSERT_NE(w.scene, nullptr);
  CheckReport report;
  report.steps = {Step(1, "tube_in", -1), Step(2, "rail_set", 0), Step(3, "module_take", 0),
                  Step(4, "bay_out", -1)};
  for (int k : {1, 2}) report.steps[k].arm = "module_arm";
  Plan p;
  p.bays = 2;
  p.events = {{"tube_in", 1, 0.0, 4.0, 0},    {"bay_out", 4, 9.0, 10.0, 0},
              {"tube_in", 5, 10.0, 14.0, 1},  {"bay_out", 8, 19.0, 20.0, 1}};
  p.arms = {{"module_arm",
             {Seg("wait", 2, 0.0, 5.0), Seg("work", 2, 5.0, 6.0), Seg("wait", 3, 6.0, 7.0),
              Seg("work", 3, 7.0, 8.0), Seg("wait", 6, 8.0, 15.0), Seg("work", 6, 15.0, 16.0),
              Seg("wait", 7, 16.0, 17.0), Seg("work", 7, 17.0, 18.0), Seg("wait", 8, 18.0, 20.0)}}};
  const PlanPlayer player(*w.scene, report, p);
  ASSERT_NE(player.Step(7), nullptr);
  EXPECT_EQ(player.Step(7)->kind, "module_take");
  EXPECT_EQ(player.Step(7)->index, 1);  // One take a bay here: bay 1 takes point 1.
  const WorldAt a = player.At(6.5);
  EXPECT_EQ(a.bay, 0);
  EXPECT_EQ(a.rails, std::vector<int>{0});
  const WorldAt going = player.At(9.5);
  EXPECT_EQ(going.bay, 0);
  EXPECT_EQ(going.event, "bay_out");
  EXPECT_EQ(going.rails, std::vector<int>{0});
  const WorldAt coming = player.At(12.0);
  EXPECT_EQ(coming.bay, 1);
  EXPECT_EQ(coming.event, "tube_in");
  EXPECT_TRUE(coming.rails.empty());
  EXPECT_EQ(coming.pallet_taken, std::vector<int>{0});
  const WorldAt later = player.At(18.5);
  EXPECT_EQ(later.rails, std::vector<int>{0});
  EXPECT_EQ(later.pallet_taken, (std::vector<int>{0, 1}));
  EXPECT_EQ(player.BuiltAt(later).on_pallets, PointCount(layout, "module_take") - 2);
  const WorldAt out = player.At(25.0);
  EXPECT_EQ(out.bay, 1);
  EXPECT_EQ(out.event, "bay_out");
}

}  // namespace
}  // namespace cell::layout
