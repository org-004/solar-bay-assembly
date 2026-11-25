// The layout checker: the reference layout's report, and copies of it broken
// in memory, one check at a time, which must fail that check; the choice of
// kinds of check; work divided among arms; the approach rows; the margins.

#include "src/layout_check.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>

#include "src/bay_hardware.h"
#include "src/container.h"
#include "src/layout.h"
#include "src/layout_model.h"

namespace cell::layout {
namespace {

std::string Dir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr ? dir : "layouts";
}

Layout Reference() {
  std::string error;
  const std::optional<Layout> layout = LoadLayout(Dir() + "/below.json", &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout.value_or(Layout{});
}

ModelPaths Paths() { return ModelPaths::FromEnvironment(); }

int Failing(const CheckReport& report, const std::string& check) {
  int n = 0;
  for (const CheckRow& r : report.rows) n += r.check == check && !r.pass;
  return n;
}

const CheckRow* FindRow(const CheckReport& report, const std::string& check,
                        const std::string& item) {
  for (const CheckRow& r : report.rows) {
    if (r.check == check && r.item == item) return &r;
  }
  return nullptr;
}

// The kinds of check of run 011. A layout on the container is held to these
// alone; the kinds added since may fail on it (R13).
const std::vector<std::string>& OldKinds() {
  static const std::vector<std::string> kinds = {
      "transport", "hold", "reach", "clearance", "load", "ways", "time"};
  return kinds;
}

std::string Failures(const CheckReport& report) {
  std::string out;
  for (const CheckRow& r : report.rows) {
    if (!r.pass) {
      out += r.check + " " + r.item + " " + std::to_string(r.figure) + " " +
             r.detail + "\n";
    }
  }
  return out;
}

TEST(LayoutCheck, OneLayoutIsChosen) {
  int chosen = 0;
  const std::vector<std::string> names = ListLayouts(Dir());
  ASSERT_FALSE(names.empty());
  for (const std::string& name : names) {
    std::string error;
    const auto layout = LoadLayout(Dir() + "/" + name + ".json", &error);
    ASSERT_TRUE(layout.has_value()) << name << ": " << error;
    chosen += layout->chosen ? 1 : 0;
  }
  EXPECT_EQ(chosen, 1);
}

TEST(LayoutCheck, WorkPointsOfOneBay) {
  const Layout layout = Reference();
  EXPECT_EQ(WorkPoints(layout, "rail_set").size(), 10u);
  EXPECT_EQ(WorkPoints(layout, "strap_bolt").size(), 10u);
  EXPECT_EQ(WorkPoints(layout, "pin").size(), 36u);
  EXPECT_EQ(WorkPoints(layout, "locate").size(), 36u);
  EXPECT_EQ(WorkPoints(layout, "swage").size(), 36u);
  EXPECT_EQ(WorkPoints(layout, "module_set").size(), 9u);
  EXPECT_EQ(WorkPoints(layout, "module_take").size(), 36u);
  EXPECT_EQ(WorkPoints(layout, "rail_take").size(), 10u);
  EXPECT_EQ(WorkPoints(layout, "pins_take").size(), 1u);
  // Module 4's set-down: the centre of its glass top, in the layout.
  const WorkPoint set = WorkPoints(layout, "module_set")[4];
  const Eigen::Vector3d expect =
      Eigen::Vector3d(layout.tube.axis_x, layout.tube.plain_end_y,
                      layout.tube.axis_z) +
      bay::ModulePose(4).translation() +
      Eigen::Vector3d(0, 0, bay::kModuleThickness / 2.0);
  EXPECT_LT((set.pos - expect).norm(), 1e-12);
  EXPECT_LT((set.z - Eigen::Vector3d(0, 0, -1)).norm(), 1e-12);
  EXPECT_TRUE(set.has_x);
  // A pin works 5 mm below the rail's site, from below.
  const WorkPoint pin = WorkPoints(layout, "pin")[4 * 2 + 3];
  EXPECT_EQ(pin.item, "pin 2/3");
  EXPECT_EQ(pin.rail, 3);
  EXPECT_NEAR(pin.pos.z(),
              layout.tube.axis_z + bay::kRailTopHeight - bay::kRailSheet / 2 -
                  0.005,
              1e-12);
  // A bolt works 5 mm short of the ear's mouth, travelling along -X.
  const WorkPoint bolt = WorkPoints(layout, "strap_bolt")[0];
  EXPECT_NEAR(bolt.pos.x(), layout.tube.axis_x + bay::kStrapEarHalfWidth + 0.005,
              1e-12);
  EXPECT_LT((bolt.z - Eigen::Vector3d(-1, 0, 0)).norm(), 1e-12);
  // Standing on their long edge: one pitch further along the pallet's y
  // each take, level, the tool pressing along +y.
  const auto takes = WorkPoints(layout, "module_take");
  EXPECT_NEAR((takes[1].pos - takes[0].pos).norm(), layout.pallets[0].pitch, 1e-12);
  EXPECT_NEAR(takes[0].pos.z(),
              layout.pallets[0].pos[2] + layout.pallets[0].base_height +
                  bay::kModuleWidth / 2.0,
              1e-12);
  EXPECT_NEAR(takes[0].z.z(), 0.0, 1e-12);
  EXPECT_EQ(takes[0].pallet_modules, 36);
}

TEST(LayoutCheck, MoveModel) {
  // The slowest joint sets the time, times 1.5.
  EXPECT_DOUBLE_EQ(JointMoveTime({120, 0, 0, 0, 0, 0}, {120, 1, 1, 1, 1, 1}),
                   1.5);
  EXPECT_DOUBLE_EQ(JointMoveTime({0, 0, 0, 0, 0, 0}, {1, 1, 1, 1, 1, 1}), 0.0);
  EXPECT_DOUBLE_EQ(TrackMoveTime(3.0, 1.5, 1.5), 3.0);
  EXPECT_DOUBLE_EQ(TrackMoveTime(0.0, 1.5, 1.5), 0.0);
  EXPECT_FALSE(TimeModelText().empty());
}

// Two arms side by side, the second one unit behind the first.
TEST(LayoutCheck, SideBySide) {
  // The follower is the slower: the leader's first unit, then all of its.
  EXPECT_DOUBLE_EQ(SideBySide({5, 5, 5}, {10, 10, 10}), 5.0 + 30.0);
  // A slow unit in the middle of both: the follower waits for the leader's
  // unit 1 and then does its own units 1 and 2 (11 + 11), more than either
  // end alone gives (13).
  EXPECT_DOUBLE_EQ(SideBySide({1, 10, 1}, {1, 10, 1}), 22.0);
  // The leader is the slower: all of its, then the follower's last unit.
  EXPECT_DOUBLE_EQ(SideBySide({10, 10, 10}, {2, 2, 3}), 30.0 + 3.0);
  // Never less than either arm alone, never more than the plain sum.
  const std::vector<double> a = {12, 7, 9, 4}, b = {3, 11, 6, 8};
  EXPECT_GE(SideBySide(a, b), 32.0);
  EXPECT_LE(SideBySide(a, b), 32.0 + 28.0);
  EXPECT_DOUBLE_EQ(SideBySide({}, {1}), 0.0);
}

// The whole check of the reference layout.
TEST(LayoutCheck, ReferenceReport) {
  const Layout layout = Reference();
  const CheckReport report = CheckLayout(layout, Paths());
  ASSERT_TRUE(report.error.empty()) << report.error;
  std::map<std::string, int> kinds;
  for (const CheckRow& r : report.rows) ++kinds[r.check];
  for (const char* kind :
       {"transport", "reach", "clearance", "load", "ways", "time"}) {
    EXPECT_GT(kinds[kind], 0) << kind;
  }
  // One reach row per point, every one passing.
  std::set<std::string> items;
  int reach = 0;
  for (const CheckRow& r : report.rows) {
    if (r.check != "reach") continue;
    const std::string kind = r.item.substr(0, r.item.find(' '));
    if (kind == "rail_set" || kind == "strap_bolt" || kind == "pin" ||
        kind == "module_set" || kind == "module_take") {
      ++reach;
      EXPECT_TRUE(items.insert(r.item).second) << r.item;
    }
    EXPECT_TRUE(r.pass) << r.item << ": " << r.detail;
  }
  EXPECT_GE(reach, 101);
  EXPECT_GT(report.bay_time_s, 0.0);
  EXPECT_FALSE(report.steps.empty());
  std::printf("reference failures:\n%s", Failures(report).c_str());
}

// The layout marked chosen passes every row (of the old kinds, if it stands
// on the container).
TEST(LayoutCheck, ChosenLayoutPasses) {
  std::optional<Layout> chosen;
  for (const std::string& name : ListLayouts(Dir())) {
    std::string error;
    auto layout = LoadLayout(Dir() + "/" + name + ".json", &error);
    ASSERT_TRUE(layout.has_value()) << name << ": " << error;
    if (layout->chosen) chosen = layout;
  }
  ASSERT_TRUE(chosen.has_value());
  CheckOptions options;
  if (chosen->base == "container") options.checks = OldKinds();
  const CheckReport report = CheckLayout(*chosen, Paths(), options);
  ASSERT_TRUE(report.error.empty()) << report.error;
  EXPECT_TRUE(report.pass) << chosen->name << " fails:\n" << Failures(report);
  std::map<std::string, int> kinds;
  for (const CheckRow& r : report.rows) ++kinds[r.check];
  for (const char* kind :
       {"transport", "reach", "clearance", "load", "ways", "time"}) {
    EXPECT_GT(kinds[kind], 0) << kind;
  }
  int reach = 0;
  for (const CheckRow& r : report.rows) {
    if (r.check != "reach") continue;
    const std::string kind = r.item.substr(0, r.item.find(' '));
    if (kind == "rail_set" || kind == "strap_bolt" || kind == "pin" ||
        kind == "module_set" || kind == "module_take") {
      ++reach;
      EXPECT_TRUE(r.pass) << r.item << ": " << r.detail;
    }
  }
  EXPECT_GE(reach, 101);
}

// "above" fails, and of the old kinds only on clearance: its small arms
// cannot work under the modules from above.
TEST(LayoutCheck, AboveFailsOnClearanceOnly) {
  std::string error;
  const auto above = LoadLayout(Dir() + "/above.json", &error);
  ASSERT_TRUE(above.has_value()) << error;
  CheckOptions options;
  options.checks = OldKinds();
  const CheckReport report = CheckLayout(*above, Paths(), options);
  EXPECT_FALSE(report.pass);
  for (const CheckRow& r : report.rows) {
    if (!r.pass) EXPECT_EQ(r.check, "clearance") << r.item << ": " << r.detail;
  }
}

// Each check made to fail in a copy changed in memory. Each compares with
// the reference under the same options.
CheckReport RunChecks(const Layout& layout, std::vector<std::string> checks,
                std::vector<std::string> work = {}) {
  CheckOptions options;
  options.checks = std::move(checks);
  options.work_kinds = std::move(work);
  return CheckLayout(layout, Paths(), options);
}

TEST(LayoutCheck, TransportFails) {
  const Layout reference = Reference();
  const CheckReport before = RunChecks(reference, {"transport"});
  // A stow pose that sticks out: the module arm stretched out across.
  Layout out = reference;
  out.arms[0].stow_joints_deg = {90, -10, 0, 0, 0, 0};
  const CheckReport pose = RunChecks(out, {"transport"});
  // Worse than the reference, and failing.
  const CheckRow* x = FindRow(pose, "transport", "stowed x");
  const CheckRow* x0 = FindRow(before, "transport", "stowed x");
  ASSERT_NE(x, nullptr);
  ASSERT_NE(x0, nullptr);
  EXPECT_FALSE(x->pass) << x->detail;
  EXPECT_LT(x->figure, x0->figure - 0.5) << x->detail;
  // A mass over the budget.
  Layout heavy = reference;
  heavy.holder.mass_kg = container::kCellMassBudget;
  const CheckReport mass = RunChecks(heavy, {"transport"});
  const CheckRow* m = FindRow(mass, "transport", "mass");
  ASSERT_NE(m, nullptr);
  EXPECT_FALSE(m->pass);
  const CheckRow* m0 = FindRow(before, "transport", "mass");
  ASSERT_NE(m0, nullptr);
  EXPECT_TRUE(m0->pass);
}

TEST(LayoutCheck, ReachFails) {
  const Layout reference = Reference();
  const CheckReport before = RunChecks(reference, {"reach"}, {"rail_set"});
  // The tube 1.5 m further from the small arm.
  Layout far = reference;
  far.tube.axis_x -= 1.5;
  const CheckReport after = RunChecks(far, {"reach"}, {"rail_set"});
  EXPECT_EQ(Failing(before, "reach"), 0) << Failures(before);
  EXPECT_EQ(Failing(after, "reach"), 10) << Failures(after);
  for (int k = 0; k < bay::kRailCount; ++k) {
    const std::string item = "rail_set " + std::to_string(k);
    const CheckRow* r0 = FindRow(before, "reach", item);
    const CheckRow* r = FindRow(after, "reach", item);
    ASSERT_NE(r0, nullptr) << item;
    ASSERT_NE(r, nullptr) << item;
    EXPECT_TRUE(r0->pass) << item;
    EXPECT_FALSE(r->pass) << item;
    EXPECT_NE(r->detail.find("not reached"), std::string::npos) << r->detail;
  }
}

// A stop, a wait or a stow off the end of the track.
TEST(LayoutCheck, TrackValuesOutsideTheTrackFail) {
  const Layout reference = Reference();
  ASSERT_TRUE(reference.arms[0].track.has_value());
  const double beyond = reference.arms[0].track->hi + 1.0;
  Layout off = reference;
  off.arms[0].stops["module_set"] = {beyond};
  off.arms[0].wait_track = beyond;
  off.arms[0].stow_track = beyond;
  const CheckReport before =
      RunChecks(reference, {"reach", "transport"}, {"module_set"});
  const CheckReport after =
      RunChecks(off, {"reach", "transport"}, {"module_set"});
  for (const char* item : {"module_set 0", "module_set 8"}) {
    const CheckRow* r0 = FindRow(before, "reach", item);
    const CheckRow* r = FindRow(after, "reach", item);
    ASSERT_NE(r0, nullptr);
    ASSERT_NE(r, nullptr);
    EXPECT_TRUE(r0->pass);
    EXPECT_FALSE(r->pass);
    EXPECT_NE(r->detail.find("outside the track's"), std::string::npos)
        << r->detail;
  }
  const std::string arm = reference.arms[0].name;
  const CheckRow* wait0 = FindRow(before, "reach", "wait track " + arm);
  const CheckRow* wait = FindRow(after, "reach", "wait track " + arm);
  const CheckRow* stow0 = FindRow(before, "transport", "stow track " + arm);
  const CheckRow* stow = FindRow(after, "transport", "stow track " + arm);
  ASSERT_NE(wait0, nullptr);
  ASSERT_NE(wait, nullptr);
  ASSERT_NE(stow0, nullptr);
  ASSERT_NE(stow, nullptr);
  EXPECT_TRUE(wait0->pass);
  EXPECT_TRUE(stow0->pass);
  EXPECT_FALSE(wait->pass);
  EXPECT_FALSE(stow->pass);
  EXPECT_NEAR(wait->figure, -1.0, 1e-9);
}

Tool* FindToolIn(Layout* layout, const std::string& name) {
  for (ArmUse& a : layout->arms) {
    for (Tool& t : a.tools) {
      if (t.name == name) return &t;
    }
  }
  return nullptr;
}
Box* FindBox(Tool* tool, const std::string& name) {
  for (Box& b : tool->boxes) {
    if (b.name == name) return &b;
  }
  return nullptr;
}

// A tool too fat for the free space it works in fails, however its last
// box is excused against the parts it joins.
TEST(LayoutCheck, ToolTooFatForItsFreeSpaceFails) {
  const std::string item = "tool in free space strap_bolt nutrunner";
  // First a nutrunner whose body stays behind the free space: it passes.
  Layout slim = Reference();
  Tool* nut = FindToolIn(&slim, "nutrunner");
  ASSERT_NE(nut, nullptr);
  Box* body = FindBox(nut, "body");
  Box* socket = FindBox(nut, "socket");
  ASSERT_NE(body, nullptr);
  ASSERT_NE(socket, nullptr);
  const double body_top = nut->length - 0.11;
  body->size[2] = body_top;
  body->pos[2] = body_top / 2.0;
  socket->size[2] = nut->length - body_top;
  socket->pos[2] = body_top + socket->size[2] / 2.0;
  const CheckReport ok = RunChecks(slim, {"clearance"}, {"strap_bolt"});
  const CheckRow* r0 = FindRow(ok, "clearance", item);
  ASSERT_NE(r0, nullptr);
  EXPECT_TRUE(r0->pass) << r0->detail;
  // Then the same with a socket 50 mm square.
  Layout fat = slim;
  Box* fat_socket = FindBox(FindToolIn(&fat, "nutrunner"), "socket");
  fat_socket->size[0] = 0.05;
  fat_socket->size[1] = 0.05;
  const CheckReport bad = RunChecks(fat, {"clearance"}, {"strap_bolt"});
  const CheckRow* r = FindRow(bad, "clearance", item);
  ASSERT_NE(r, nullptr);
  EXPECT_FALSE(r->pass) << r->detail;
  EXPECT_NEAR(r->figure,
              bay::kStrapBoltFreeRadius - std::hypot(0.025, 0.025), 1e-9);
}

// A nose so long and wide that it goes into the rail fails the pin's
// clearance, although the nose is excused against the rail down to zero.
TEST(LayoutCheck, NoseIntoTheRailFails) {
  const Layout reference = Reference();
  const CheckReport before = RunChecks(reference, {"clearance"}, {"pin"});
  Layout bad = reference;
  Tool* pin = FindToolIn(&bad, "pin_tool");
  ASSERT_NE(pin, nullptr);
  Box* nose = FindBox(pin, "nose");
  ASSERT_NE(nose, nullptr);
  // 40 mm square, reaching 20 mm past the working point: through the rail's
  // sheet, 5 mm above the working point.
  const double lo = nose->pos[2] - nose->size[2] / 2.0;
  const double hi = pin->length + 0.020;
  nose->size = {0.04, 0.04, hi - lo};
  nose->pos[2] = (lo + hi) / 2.0;
  const CheckReport after = RunChecks(bad, {"clearance"}, {"pin"});
  const CheckRow* r0 = FindRow(before, "clearance", "pin 0/0");
  const CheckRow* r = FindRow(after, "clearance", "pin 0/0");
  ASSERT_NE(r0, nullptr);
  ASSERT_NE(r, nullptr);
  EXPECT_TRUE(r0->pass) << r0->detail;
  EXPECT_FALSE(r->pass) << r->detail;
  EXPECT_LT(r->figure, 0.0);
  EXPECT_NE(r->detail.find("into each other"), std::string::npos) << r->detail;
  EXPECT_GT(Failing(after, "clearance"), Failing(before, "clearance"));
}

TEST(LayoutCheck, ClearanceFails) {
  const Layout reference = Reference();
  const CheckReport before = RunChecks(reference, {"clearance"}, {"strap_bolt"});
  // A box of the holder where the nutrunner must go at rail 3.
  Layout blocked = reference;
  const Eigen::Vector3d at =
      BayPoint(reference, bay::Fastenings()[3].pose_a.translation());
  Box box;
  box.name = "in_the_way";
  box.size = {0.04, 0.04, 0.04};
  box.pos = {at.x() + 0.08, at.y(), at.z()};
  blocked.holder.boxes.push_back(box);
  const CheckReport after = RunChecks(blocked, {"clearance"}, {"strap_bolt"});
  EXPECT_GT(Failing(after, "clearance"), Failing(before, "clearance"));
  const CheckRow* row = FindRow(after, "clearance", "strap_bolt 3");
  ASSERT_NE(row, nullptr);
  EXPECT_FALSE(row->pass) << row->detail;
  const CheckRow* free = FindRow(after, "clearance", "free space bolt 3");
  ASSERT_NE(free, nullptr);
  EXPECT_FALSE(free->pass) << free->detail;
}

TEST(LayoutCheck, LoadFails) {
  const Layout reference = Reference();
  EXPECT_EQ(Failing(RunChecks(reference, {"load"}), "load"), 0);
  Layout heavy = reference;
  heavy.arms[1].tools[1].mass_kg = 200.0;
  const CheckReport after = RunChecks(heavy, {"load"});
  const CheckRow* row =
      FindRow(after, "load", heavy.arms[1].name + " " + heavy.arms[1].tools[1].name);
  ASSERT_NE(row, nullptr);
  EXPECT_FALSE(row->pass);
}

TEST(LayoutCheck, WaysFail) {
  const Layout reference = Reference();
  const CheckReport before = RunChecks(reference, {"ways"});
  Layout blocked = reference;
  Item stand;
  stand.name = "stand_in_the_way";
  stand.does = "Stands where it should not.";
  Box box;
  box.name = "post";
  box.size = {0.2, 0.2, 2.0};
  box.pos = {reference.tube.axis_x - 1.6, 0.0, 1.0};
  stand.boxes.push_back(box);
  blocked.items.push_back(stand);
  const CheckReport after = RunChecks(blocked, {"ways"});
  const CheckRow* row = FindRow(after, "ways", "bay out");
  const CheckRow* row0 = FindRow(before, "ways", "bay out");
  ASSERT_NE(row, nullptr);
  ASSERT_NE(row0, nullptr);
  EXPECT_FALSE(row->pass) << row->detail;
  // The stand, not what the reference already has, is the nearest.
  EXPECT_LT(row->figure, 0.0) << row->detail;
  EXPECT_NE(row->detail.find("stand_in_the_way"), std::string::npos)
      << row->detail;
}

TEST(LayoutCheck, TimeFails) {
  // The magazine is to be taken by an arm that has no tool for it: that
  // step has no pose, and the bay has no time.
  Layout broken = Reference();
  for (Work& w : broken.work) {
    if (w.kind == "pins_take") w.arm = broken.arms[0].name;
  }
  const CheckReport after = RunChecks(broken, {"time"});
  const CheckRow* row = FindRow(after, "time", "bay");
  ASSERT_NE(row, nullptr);
  EXPECT_FALSE(row->pass) << row->detail;
}

}  // namespace
}  // namespace cell::layout

namespace cell::layout {
namespace {

TEST(LayoutCheck, StateAtModuleEight) {
  LayoutState state;
  std::string error;
  ASSERT_TRUE(StateAt(Reference(), Paths(), "module_set", 8, &state, &error))
      << error;
  EXPECT_EQ(state.rails, bay::kRailCount);
  EXPECT_EQ(state.modules, 8);
  EXPECT_EQ(state.held.at("module_arm"), "module");
  EXPECT_EQ(state.tool.at("module_arm"), "module_gripper");
  EXPECT_NEAR(state.track.at("module_arm"), 3.432, 1e-9);
  EXPECT_FALSE(StateAt(Reference(), Paths(), "module_set", 9, &state, &error));
}

}  // namespace
}  // namespace cell::layout

namespace cell::layout {
namespace {

Layout Named(const std::string& name) {
  std::string error;
  const std::optional<Layout> layout =
      LoadLayout(Dir() + "/" + name + ".json", &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout.value_or(Layout{});
}

int CountKind(const CheckReport& report, const std::string& check) {
  int n = 0;
  for (const CheckRow& r : report.rows) n += r.check == check;
  return n;
}

// Only the kinds asked for are made, and the report passes by them alone.
TEST(LayoutCheck, ChecksChooseTheKinds) {
  const std::vector<std::string>& kinds = CheckKinds();
  ASSERT_EQ(kinds.size(), 10u);
  EXPECT_EQ(kinds[0], "transport");
  EXPECT_EQ(kinds[6], "time");
  EXPECT_EQ(kinds[9], "approach");
  const Layout layout = Reference();
  CheckOptions options;
  options.checks = {"load", "hold"};
  const CheckReport some = CheckLayout(layout, Paths(), options);
  ASSERT_FALSE(some.rows.empty());
  for (const CheckRow& r : some.rows) {
    EXPECT_TRUE(r.check == "load" || r.check == "hold") << r.check;
  }
  EXPECT_GT(CountKind(some, "load"), 0);
  EXPECT_GT(CountKind(some, "hold"), 0);
  EXPECT_TRUE(some.pass) << Failures(some);
  EXPECT_TRUE(some.steps.empty());
  EXPECT_TRUE(some.points.empty());
  // An old layout has no approach: asking for it fails the report.
  options.checks = {"load", "approach"};
  const CheckReport with = CheckLayout(layout, Paths(), options);
  EXPECT_GT(CountKind(with, "approach"), 0);
  EXPECT_FALSE(with.pass);
}

// The old layouts' bay times, as at the base of this work (68a1199).
// "beside" is left out: the bay's model changed under it since (its time is
// 750.3 s with the checker of that commit too).
TEST(LayoutCheck, OldBayTimesUnchanged) {
  const std::map<std::string, double> expect = {
      {"above", 832.2021},
      {"below", 738.8765},
      {"below_each_rail", 874.1356},
      {"below_rails_by_module_arm", 628.5435}};
  for (const auto& [name, seconds] : expect) {
    CheckOptions options;
    options.checks = {"time"};
    const CheckReport report = CheckLayout(Named(name), Paths(), options);
    EXPECT_NEAR(report.bay_time_s, seconds, 0.05) << name;
    // One busy row per arm.
    EXPECT_EQ(CountKind(report, "time"), 3) << name;
  }
}

// below_rails_by_module_arm with a second small arm on the same track line
// (it waits at the track's other end; the beams are one and the clearance
// rows are not asked for), and the holes divided: 0-17 to the new arm,
// 18-35 to the old one, each with its own stock of pins at its end.
Layout Divided() {
  Layout layout = Named("below_rails_by_module_arm");
  ArmUse* a = nullptr;
  for (ArmUse& arm : layout.arms) {
    if (arm.name == "small_arm") a = &arm;
  }
  EXPECT_NE(a, nullptr);
  if (a == nullptr) return layout;
  EXPECT_TRUE(a->track.has_value());
  ArmUse b = *a;
  b.name = "small_arm_b";
  b.first_tool = "pin_tool";
  b.wait_track = b.track->lo;
  b.stow_track = b.track->lo;
  b.stops = {{"pins_take", {b.track->lo}}};
  layout.arms.push_back(b);
  Stock pins_b = *FindStock(layout, "pins");
  pins_b.name = "pins_b";
  pins_b.pick_pos[1] = -pins_b.pick_pos[1];
  for (Box& box : pins_b.boxes) box.pos[1] = -box.pos[1];
  layout.stocks.push_back(pins_b);
  std::vector<Work> work;
  for (const Work& w : layout.work) {
    if (w.kind == "locate" || w.kind == "pin" || w.kind == "swage") {
      Work mine = w, theirs = w;
      mine.points = "18-35";
      theirs.arm = "small_arm_b";
      theirs.points = "0-17";
      work.push_back(mine);
      work.push_back(theirs);
    } else if (w.kind == "pins_take") {
      Work mine = w, theirs = w;
      mine.points = "0";
      theirs.arm = "small_arm_b";
      theirs.points = "1";
      work.push_back(mine);
      work.push_back(theirs);
    } else {
      work.push_back(w);
    }
  }
  layout.work = work;
  return layout;
}

TEST(LayoutCheck, WorkDividedAmongArms) {
  const Layout layout = Divided();
  ASSERT_EQ(PointCount(layout, "pins_take"), 2);
  CheckOptions options;
  options.checks = {"reach", "time"};
  const CheckReport report = CheckLayout(layout, Paths(), options);
  ASSERT_TRUE(report.error.empty()) << report.error;
  auto expected = [](int index) {
    return index < 18 ? std::string("small_arm_b") : std::string("small_arm");
  };
  // The jobs: each point with the arm the division gives it.
  for (const char* kind : {"locate", "pin", "swage"}) {
    for (int index : {0, 17, 18, 35}) {
      const std::string item = std::string(kind) + " " +
                               std::to_string(index / 4) + "/" +
                               std::to_string(index % 4);
      const CheckRow* r = FindRow(report, "reach", item);
      ASSERT_NE(r, nullptr) << item;
      EXPECT_EQ(r->arm, expected(index)) << item;
    }
  }
  const CheckRow* magazine = FindRow(report, "reach", "pins_take 1");
  ASSERT_NE(magazine, nullptr);
  EXPECT_EQ(magazine->arm, "small_arm_b");
  // The steps: each hole by its arm, each arm's magazine before its first
  // hole, every arm back at the end.
  std::map<std::string, int> first_hole, first_magazine, magazines;
  for (const BayStep& s : report.steps) {
    int m = -1, i = -1;
    char what[16];
    if (std::sscanf(s.what.c_str(), "%15s %d/%d", what, &m, &i) == 3) {
      EXPECT_EQ(s.arm, expected(4 * m + i)) << s.what;
      if (!first_hole.count(s.arm)) first_hole[s.arm] = s.number;
    }
    if (s.what == "take a pin magazine") {
      ++magazines[s.arm];
      if (!first_magazine.count(s.arm)) first_magazine[s.arm] = s.number;
    }
  }
  for (const char* arm : {"small_arm", "small_arm_b"}) {
    EXPECT_EQ(magazines[arm], 1) << arm;
    ASSERT_TRUE(first_hole.count(arm)) << arm;
    EXPECT_LT(first_magazine[arm], first_hole[arm]) << arm;
  }
  // A busy row for each of the three arms, and the module phase by three.
  for (const char* arm : {"module_arm", "small_arm", "small_arm_b"}) {
    const CheckRow* busy = FindRow(report, "time", std::string("busy ") + arm);
    ASSERT_NE(busy, nullptr) << arm;
    EXPECT_GT(busy->figure, 0.0) << arm;
  }
  const CheckRow* bay = FindRow(report, "time", "bay");
  ASSERT_NE(bay, nullptr);
  EXPECT_NE(bay->detail.find("pin arms"), std::string::npos) << bay->detail;
  EXPECT_NE(bay->detail.find("3 arms"), std::string::npos) << bay->detail;
  std::printf("divided bay %.1f s: %s\n", report.bay_time_s,
              bay->detail.c_str());
  // Two pin arms share the holes: the bay is no longer than with one.
  EXPECT_LT(report.bay_time_s, 628.5435 + 1e-6);
}

Approach Rule(const std::string& kind, const std::string& frame, Vec3 dir,
              double m) {
  Approach a;
  a.kind = kind;
  a.approach_frame = frame;
  a.approach_dir = dir;
  a.approach_m = m;
  a.retreat_frame = frame;
  a.retreat_dir = dir;
  a.retreat_m = m;
  return a;
}

// A layout without rules: one failing row per kind.
TEST(LayoutCheck, NoApproachRulesFailOncePerKind) {
  CheckOptions options;
  options.checks = {"approach"};
  const CheckReport report = CheckLayout(Reference(), Paths(), options);
  std::set<std::string> kinds;
  for (const CheckRow& r : report.rows) {
    ASSERT_EQ(r.check, "approach");
    EXPECT_FALSE(r.pass);
    EXPECT_EQ(r.detail, "the layout gives no approach for " + r.item);
    EXPECT_TRUE(kinds.insert(r.item).second) << r.item;
  }
  EXPECT_EQ(kinds.size(), WorkKinds().size());
}

// Rules added in memory: two rows per point; a module set down from above
// passes, one brought up from below runs into the bay.
TEST(LayoutCheck, ApproachRows) {
  Layout layout = Named("below_rails_by_module_arm");
  layout.approaches = {Rule("module_set", "world", {0, 0, 1}, 0.10),
                       Rule("strap_bolt", "tool", {0, 0, -1}, 0.03)};
  CheckOptions options;
  options.checks = {"approach"};
  options.work_kinds = {"module_set", "strap_bolt", "pin"};
  const CheckReport report = CheckLayout(layout, Paths(), options);
  ASSERT_TRUE(report.error.empty()) << report.error;
  int set_rows = 0, bolt_rows = 0;
  for (const CheckRow& r : report.rows) {
    if (r.item.rfind("module_set ", 0) == 0) ++set_rows;
    if (r.item.rfind("strap_bolt ", 0) == 0) ++bolt_rows;
  }
  EXPECT_EQ(set_rows, 2 * bay::kModuleCount);
  EXPECT_EQ(bolt_rows, 2 * bay::kRailCount);
  const CheckRow* no_rule = FindRow(report, "approach", "pin");
  ASSERT_NE(no_rule, nullptr);
  EXPECT_FALSE(no_rule->pass);
  const CheckRow* up = FindRow(report, "approach", "module_set 3 approach");
  const CheckRow* away = FindRow(report, "approach", "module_set 3 retreat");
  ASSERT_NE(up, nullptr);
  ASSERT_NE(away, nullptr);
  EXPECT_TRUE(up->pass) << up->detail;
  EXPECT_TRUE(away->pass) << away->detail;
  EXPECT_EQ(up->unit, "m");
  EXPECT_DOUBLE_EQ(up->limit, 0.010);
  EXPECT_NE(up->detail.find("20 samples over 100.0 mm"), std::string::npos)
      << up->detail;
  // The poses for a planner: the approach pose 100 mm above, the module held
  // on the way in and nothing on the way out.
  const SolvedPoint* point = nullptr;
  for (const SolvedPoint& p : report.points) {
    if (p.kind == "module_set" && p.index == 3) point = &p;
  }
  ASSERT_NE(point, nullptr);
  EXPECT_EQ(point->item, "module_set 3");
  EXPECT_EQ(point->arm, "module_arm");
  EXPECT_TRUE(point->work_reached);
  EXPECT_TRUE(point->approach_reached);
  EXPECT_TRUE(point->retreat_reached);
  EXPECT_EQ(point->held_approach, "module");
  EXPECT_EQ(point->held_work, "module");
  EXPECT_EQ(point->held_retreat, "");
  EXPECT_NE(point->approach_deg, point->work_deg);
  for (int j = 0; j < 6; ++j) {
    EXPECT_NEAR(point->approach_deg[j], point->retreat_deg[j], 1e-6);
  }
  // From below: the held module goes through its rails and the tube.
  layout.approaches = {Rule("module_set", "world", {0, 0, -1}, 0.30)};
  options.work_kinds = {"module_set"};
  const CheckReport below = CheckLayout(layout, Paths(), options);
  const CheckRow* into = FindRow(below, "approach", "module_set 3 approach");
  ASSERT_NE(into, nullptr);
  EXPECT_FALSE(into->pass) << into->detail;
  std::printf("from below: %.4f %s\n", into->figure, into->detail.c_str());
}

// On the flat rack the layout's margins count where they are larger; on the
// container they are not read.
TEST(LayoutCheck, MarginsOfAFlatRackLayout) {
  Layout layout = Reference();
  layout.margins.clearance_m = 0.02;
  layout.margins.joint_margin_deg = 7.0;
  CheckOptions options;
  options.checks = {"load"};
  const CheckReport container = CheckLayout(layout, Paths(), options);
  EXPECT_DOUBLE_EQ(container.clearance_m, 0.010);
  EXPECT_DOUBLE_EQ(container.joint_margin_deg, 5.0);
  layout.base = "flat_rack";
  const CheckReport rack = CheckLayout(layout, Paths(), options);
  EXPECT_DOUBLE_EQ(rack.clearance_m, 0.02);
  EXPECT_DOUBLE_EQ(rack.joint_margin_deg, 7.0);
  options.clearance_m = 0.03;
  const CheckReport more = CheckLayout(layout, Paths(), options);
  EXPECT_DOUBLE_EQ(more.clearance_m, 0.03);
  EXPECT_DOUBLE_EQ(more.joint_margin_deg, 7.0);
}

}  // namespace
}  // namespace cell::layout
