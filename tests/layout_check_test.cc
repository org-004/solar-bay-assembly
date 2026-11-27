// The layout checker: the reference layout's report, and copies of it broken
// in memory, one check at a time, which must fail that check; the choice of
// kinds of check; work divided among arms; the approach rows; the margins.

#include "src/layout_check.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm_catalog.h"
#include "src/bay_hardware.h"
#include "src/container.h"
#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_scene.h"
#include "src/layout_stages.h"
#include "src/mj_handles.h"

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

// versions/versions.json names one chosen layout, and it is a layout here.
TEST(LayoutCheck, OneLayoutIsChosen) {
  const std::vector<std::string> names = ListLayouts(Dir());
  ASSERT_FALSE(names.empty());
  std::string error;
  const std::optional<std::string> chosen = ChosenLayout(VersionsPath(), &error);
  ASSERT_TRUE(chosen.has_value()) << error;
  EXPECT_NE(std::find(names.begin(), names.end(), *chosen), names.end()) << *chosen;
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

// The chosen layout (versions/versions.json) passes every row (of the old
// kinds, if it stands on the container).
TEST(LayoutCheck, ChosenLayoutPasses) {
  std::string error;
  const std::optional<std::string> name = ChosenLayout(VersionsPath(), &error);
  ASSERT_TRUE(name.has_value()) << error;
  const std::optional<Layout> chosen = LoadLayout(Dir() + "/" + *name + ".json", &error);
  ASSERT_TRUE(chosen.has_value()) << *name << ": " << error;
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
  ASSERT_EQ(kinds.size(), 11u);
  EXPECT_EQ(kinds[0], "transport");
  EXPECT_EQ(kinds[6], "time");
  EXPECT_EQ(kinds[9], "approach");
  EXPECT_EQ(kinds[10], "gap");
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

namespace cell::layout {
namespace {

Layout FlatRack() {
  std::string error;
  const std::optional<Layout> layout =
      LoadLayout(Dir() + "/flat_rack_1_pin_arm.json", &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout.value_or(Layout{});
}

// On the flat rack the carriers of a way are the items its port names: a
// travelling rack whose words mention the bay, standing in the tube's way,
// fails the ways rows.
TEST(LayoutCheck, WaysOnTheFlatRackLeaveOutOnlyThePortsCarriers) {
  Layout layout = FlatRack();
  CheckOptions options;
  options.checks = {"ways"};
  const CheckReport before = CheckLayout(layout, Paths(), options);
  const CheckRow* in0 = FindRow(before, "ways", "tube in");
  ASSERT_NE(in0, nullptr);
  EXPECT_TRUE(in0->pass) << in0->detail;
  Item rack;
  rack.name = "rack_for_bay_parts";
  rack.does = "A rack for bay parts.";
  rack.travels_inside = true;
  Box box;
  box.name = "rack";
  box.size = {0.3, 0.3, 0.3};
  box.pos = {layout.tube.axis_x - 1.5, layout.tube.plain_end_y + 5.5,
             layout.tube.axis_z + 0.12};
  rack.boxes.push_back(box);
  layout.items.push_back(rack);
  const CheckReport after = CheckLayout(layout, Paths(), options);
  const CheckRow* in = FindRow(after, "ways", "tube in");
  ASSERT_NE(in, nullptr);
  EXPECT_FALSE(in->pass) << in->detail;
  EXPECT_NE(in->detail.find("rack_for_bay_parts"), std::string::npos)
      << in->detail;
}

// Two leading arms: a set waits for the unit before it to be set, so the
// rails and modules go on in order whoever sets them.
TEST(LayoutCheck, SetsGoOnInOrder) {
  // Arm 0 sets unit 0 in 10 s, arm 1 sets unit 1 in 5 s, arm 2 follows
  // with 1 s on each: unit 1 is set from 10 to 15, its follower ends at 16.
  std::vector<TimePiece> pieces(4);
  pieces[0] = {0, 0, true, true, false, 10.0};
  pieces[1] = {1, 1, true, true, false, 5.0};
  pieces[2] = {2, 0, false, false, false, 1.0};
  pieces[3] = {2, 1, false, false, false, 1.0};
  EXPECT_DOUBLE_EQ(PhaseEnd(pieces, 2, 3), 16.0);
  // A take does not wait: arm 1 takes unit 1 (4 s) while arm 0 sets unit 0.
  std::vector<TimePiece> takes = {{0, 0, true, true, false, 10.0},
                                  {1, 1, true, false, false, 4.0},
                                  {1, 1, true, true, false, 5.0},
                                  {2, 0, false, false, false, 1.0},
                                  {2, 1, false, false, false, 1.0}};
  EXPECT_DOUBLE_EQ(PhaseEnd(takes, 2, 3), 16.0);
  // In a layout: a second module arm on the same track takes and sets the
  // odd modules.
  Layout layout = Named("below_rails_by_module_arm");
  ArmUse b = layout.arms[0];
  ASSERT_EQ(b.name, "module_arm");
  b.name = "module_arm_b";
  layout.arms.push_back(b);
  for (Work& w : layout.work) {
    if (w.kind == "module_take" || w.kind == "module_set") w.points = "0,2,4,6,8";
  }
  for (const char* kind : {"module_take", "module_set"}) {
    Work w = *FindWork(layout, kind);
    w.arm = "module_arm_b";
    w.points = "1,3,5,7";
    layout.work.push_back(w);
  }
  CheckOptions options;
  options.checks = {"time"};
  const CheckReport report = CheckLayout(layout, Paths(), options);
  for (const BayStep& s : report.steps) {
    if (s.what == "set module 1" || s.what == "take module 3") {
      EXPECT_EQ(s.arm, "module_arm_b") << s.what;
    }
    if (s.what == "set module 2") EXPECT_EQ(s.arm, "module_arm") << s.what;
  }
  const CheckRow* bay = FindRow(report, "time", "bay");
  ASSERT_NE(bay, nullptr);
  EXPECT_NE(bay->detail.find("module arms"), std::string::npos) << bay->detail;
  EXPECT_GT(report.bay_time_s, 0.0);
}

// Legs that fail: the far pose still too near what the tool works on, a
// leg too long to reach, a take's approach that drives the tool into its
// part.
TEST(LayoutCheck, ApproachLegsThatFail) {
  Layout layout = Named("below_rails_by_module_arm");
  CheckOptions options;
  options.checks = {"approach"};
  // Backing the nutrunner off 3 mm leaves its socket by the strap and the
  // tube, by design 5 to 10 mm away: under clearance at the retreat pose.
  layout.approaches = {Rule("strap_bolt", "tool", {0, 0, -1}, 0.003),
                       Rule("module_set", "world", {0, 0, 1}, 2.0),
                       Rule("module_take", "tool", {0, 0, 1}, 0.05)};
  options.work_kinds = {"strap_bolt", "module_set", "module_take"};
  const CheckReport report = CheckLayout(layout, Paths(), options);
  const CheckRow* bolt = FindRow(report, "approach", "strap_bolt 3 retreat");
  ASSERT_NE(bolt, nullptr);
  EXPECT_FALSE(bolt->pass) << bolt->detail;
  EXPECT_LT(bolt->figure, 0.010);
  EXPECT_NE(bolt->detail.find("by design at the work point"),
            std::string::npos)
      << bolt->detail;
  EXPECT_NE(bolt->detail.find("at the retreat pose"), std::string::npos)
      << bolt->detail;
  const CheckRow* far = FindRow(report, "approach", "module_set 3 approach");
  ASSERT_NE(far, nullptr);
  EXPECT_FALSE(far->pass) << far->detail;
  EXPECT_NE(far->detail.find("not reached at"), std::string::npos)
      << far->detail;
  // Forward along the tool's axis: into the glass it is about to take.
  const CheckRow* into = FindRow(report, "approach", "module_take 3 approach");
  ASSERT_NE(into, nullptr);
  EXPECT_FALSE(into->pass) << into->detail;
  EXPECT_LT(into->figure, 0.0) << into->detail;
  EXPECT_NE(into->detail.find("pallet_module_3"), std::string::npos)
      << into->detail;
}

// On the flat rack the load is held to the datasheet's load centre: the
// module arm's gripper made 2 m long fails although its mass passes.
TEST(LayoutCheck, LoadCentreOnTheFlatRack) {
  Layout layout = FlatRack();
  CheckOptions options;
  options.checks = {"load"};
  const CheckReport before = CheckLayout(layout, Paths(), options);
  EXPECT_TRUE(before.pass) << Failures(before);
  ArmUse* arm = nullptr;
  for (ArmUse& a : layout.arms) {
    if (a.model == "kr210_r2700_2") arm = &a;
  }
  ASSERT_NE(arm, nullptr);
  const ArmModel* cat = FindArmModel(arm->model);
  ASSERT_NE(cat, nullptr);
  ASSERT_GT(cat->payload_centre_axial_m, 0.0);
  const std::string item = arm->name + " " + arm->tools[0].name;
  arm->tools[0].length = 2.0;
  const CheckReport after = CheckLayout(layout, Paths(), options);
  const CheckRow* row = FindRow(after, "load", item);
  ASSERT_NE(row, nullptr);
  EXPECT_LE(row->figure, row->limit);
  EXPECT_FALSE(row->pass) << row->detail;
  EXPECT_NE(row->detail.find("centre of mass"), std::string::npos)
      << row->detail;
}

}  // namespace
}  // namespace cell::layout

namespace cell::layout {
namespace {

Layout FlatRackNamed(const std::string& name) {
  std::string error;
  const std::optional<Layout> layout =
      LoadLayout(Dir() + "/" + name + ".json", &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout.value_or(Layout{});
}

// The 40 ft flat rack's transport mass row is held to the owner's 13,608 kg
// (30,000 lb) of robotics in transit, not to the road's gross mass less tare.
TEST(LayoutCheck, FlatRackTransportMassIsTheOwnersLimit) {
  const Layout layout = FlatRackNamed("flat_rack_3_pin_arms_ordered");
  const CheckReport report = RunChecks(layout, {"transport"});
  const CheckRow* m = FindRow(report, "transport", "mass");
  ASSERT_NE(m, nullptr);
  EXPECT_DOUBLE_EQ(m->limit, 13608.0);
  EXPECT_NEAR(m->figure, 3429.3, 0.05) << m->detail;  // Stocks inside too.
  EXPECT_TRUE(m->pass) << m->detail;
}

// The steps carry their kind, point, phase and the tool after a change.
// The steps follow the layout's order of modules: the k-th take is pallet
// module k and the k-th set is module order[k], each module's holes come
// after its set, and the plain layout keeps 0, 1, ..., 8.
TEST(LayoutCheck, StepsFollowTheOrderOfModules) {
  CheckOptions options;
  options.checks = {"time"};
  for (const char* name : {"flat_rack_3_pin_arms", "flat_rack_3_pin_arms_ordered"}) {
    const Layout layout = FlatRackNamed(name);
    const std::vector<int> order = ModuleOrder(layout);
    const CheckReport report = CheckLayout(layout, Paths(), options);
    ASSERT_TRUE(report.error.empty()) << report.error;
    std::vector<int> takes, sets;
    std::set<int> set_so_far;
    for (const BayStep& s : report.steps) {
      if (s.kind == "module_take") takes.push_back(s.index);
      if (s.kind == "module_set") {
        sets.push_back(s.index);
        set_so_far.insert(s.index);
      }
      if (s.kind == "locate" || s.kind == "pin" || s.kind == "swage") {
        EXPECT_TRUE(set_so_far.count(s.index / 4)) << name << ": " << s.what;
      }
      if (s.kind == "module_take") {
        EXPECT_EQ(s.what, "take module " + std::to_string(order[takes.size() - 1])) << name;
      }
    }
    EXPECT_EQ(takes, (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7, 8})) << name;
    EXPECT_EQ(sets, order) << name;
  }
}

// What is built at a work point follows the order too: at the set of a module
// the modules set before it are there, with both neighbours near by design
// when both are, and at its holes also the next set and both neighbours.
TEST(LayoutCheck, BuiltFollowsTheOrderOfModules) {
  const Layout layout = FlatRackNamed("flat_rack_3_pin_arms_ordered");
  std::string error;
  const auto scene = LayoutScene::Load(layout, Paths(), 5.0, &error);
  ASSERT_NE(scene, nullptr) << error;
  // The order is 0, 1, 4, 2, 5, 7, 6, 3, 8: module 3 is the 8th set.
  ASSERT_EQ(ModuleOrder(layout), (std::vector<int>{0, 1, 4, 2, 5, 7, 6, 3, 8}));
  const int total = PointCount(layout, "module_take");
  for (const WorkPoint& p : WorkPoints(layout, "module_set")) {
    if (p.index != 3) continue;
    const StepScene s = scene->AtWorkPoint(p, Leg::kWork);
    ASSERT_EQ(s.built.size(), 1u);
    EXPECT_EQ(s.built[0].modules, (std::vector<int>{0, 1, 2, 4, 5, 6, 7}));
    EXPECT_EQ(s.built[0].on_pallets, total - 8);
    const std::vector<int>& two = scene->handles().module_geoms[2];
    const std::vector<int>& four = scene->handles().module_geoms[4];
    ASSERT_FALSE(two.empty());
    EXPECT_TRUE(s.pairs.held_near.count(two[0]));
    EXPECT_TRUE(s.pairs.held_near.count(four[0]));
    const StepScene r = scene->AtWorkPoint(p, Leg::kRetreat);
    for (const Built& b : r.built) {
      EXPECT_EQ(b.modules, (std::vector<int>{0, 1, 2, 3, 4, 5, 6, 7}));
    }
  }
  for (const WorkPoint& p : WorkPoints(layout, "locate")) {
    if (p.index != 4 * 4) continue;  // Module 4, the 3rd set.
    const StepScene s = scene->AtWorkPoint(p, Leg::kWork);
    ASSERT_EQ(s.built.size(), 2u);
    EXPECT_EQ(s.built[0].modules, (std::vector<int>{0, 1, 4}));
    EXPECT_EQ(s.built[1].modules, (std::vector<int>{0, 1, 2, 3, 4, 5}));
  }
}

TEST(LayoutCheck, StepsCarryKindIndexAndPhase) {
  const Layout layout = FlatRackNamed("flat_rack_2_pin_arms");
  CheckOptions options;
  options.checks = {"time"};
  const CheckReport report = CheckLayout(layout, Paths(), options);
  ASSERT_FALSE(report.steps.empty());
  std::map<std::string, int> kinds;
  std::map<std::string, std::set<int>> points;
  const std::set<std::string> phases = {"tube in", "clamp", "rails", "modules",
                                        "end"};
  for (const BayStep& s : report.steps) {
    ++kinds[s.kind];
    EXPECT_TRUE(phases.count(s.phase)) << s.what << ": " << s.phase;
    if (s.index >= 0) {
      EXPECT_TRUE(points[s.kind].insert(s.index).second) << s.what;
    }
    if (s.kind == "tool_change") {
      const ArmUse* arm = FindArm(layout, s.arm);
      ASSERT_NE(arm, nullptr) << s.what;
      EXPECT_NE(FindTool(*arm, s.tool_after), nullptr) << s.tool_after;
    } else {
      EXPECT_TRUE(s.tool_after.empty()) << s.what;
    }
  }
  for (const char* k : {"tube_in", "clamp", "release", "bay_out"}) {
    EXPECT_EQ(kinds[k], 1) << k;
  }
  for (const std::string& kind : WorkKinds()) {
    const int expect = kind == "module_take" ? bay::kModuleCount
                                             : PointCount(layout, kind);
    EXPECT_EQ(kinds[kind], expect) << kind;
  }
  EXPECT_EQ(report.steps.front().phase, "tube in");
  EXPECT_EQ(report.steps.back().phase, "end");
  // The phases add up to the bay's time.
  ASSERT_EQ(report.phases.size(), 5u);
  double sum = 0.0;
  for (const auto& [name, seconds] : report.phases) sum += seconds;
  EXPECT_NEAR(sum, report.bay_time_s, 1e-9);
  EXPECT_EQ(report.phases[2].first, "rails");
}

// A leg's samples start at the work pose and end at the far pose, as many
// as the row says.
TEST(LayoutCheck, LegSamplesForThePlanner) {
  const Layout layout = FlatRackNamed("flat_rack_1_pin_arm");
  CheckOptions options;
  options.checks = {"approach"};
  options.work_kinds = {"module_set", "pin"};
  const CheckReport report = CheckLayout(layout, Paths(), options);
  int checked = 0;
  for (const SolvedPoint& p : report.points) {
    if (p.item != "module_set 4" && p.item != "pin 3/2") continue;
    ++checked;
    const CheckRow* row = FindRow(report, "approach", p.item + " approach");
    ASSERT_NE(row, nullptr);
    int samples = 0;
    ASSERT_NE(row->detail.find(" samples over "), std::string::npos);
    const size_t at = row->detail.rfind("; ", row->detail.find(" samples over "));
    samples = std::atoi(row->detail.c_str() + at + 2);
    ASSERT_GT(samples, 0);
    ASSERT_EQ(static_cast<int>(p.approach_leg_deg.size()), samples + 1);
    ASSERT_EQ(static_cast<int>(p.retreat_leg_deg.size()), samples + 1)
        << "the rules of these kinds have legs of one length";
    for (int j = 0; j < 6; ++j) {
      EXPECT_NEAR(p.approach_leg_deg.front()[j], p.work_deg[j], 1e-9);
      EXPECT_NEAR(p.approach_leg_deg.back()[j], p.approach_deg[j], 1e-9);
      EXPECT_NEAR(p.retreat_leg_deg.front()[j], p.work_deg[j], 1e-9);
      EXPECT_NEAR(p.retreat_leg_deg.back()[j], p.retreat_deg[j], 1e-9);
    }
    const Approach* rule = FindApproach(layout, p.kind);
    ASSERT_NE(rule, nullptr);
    EXPECT_DOUBLE_EQ(p.approach_length_m, rule->approach_m);
    EXPECT_DOUBLE_EQ(p.retreat_length_m, rule->retreat_m);
  }
  EXPECT_EQ(checked, 2);
}

// The tool-change pose: rows on the flat rack, and the flange where the
// rule puts it.
TEST(LayoutCheck, ToolChangePose) {
  const Layout layout = FlatRackNamed("flat_rack_1_pin_arm");
  CheckOptions options;
  options.checks = {"reach", "clearance"};
  options.work_kinds = {"pins_take"};
  const CheckReport report = CheckLayout(layout, Paths(), options);
  std::string error;
  auto scene = LayoutScene::Load(layout, Paths(), 5.0, &error);
  ASSERT_NE(scene, nullptr) << error;
  int arms = 0;
  for (const ArmUse& a : layout.arms) arms += a.tools.size() > 1 ? 1 : 0;
  ASSERT_GT(arms, 0);
  ASSERT_EQ(static_cast<int>(report.tool_changes.size()), arms);
  for (const SolvedToolChange& tc : report.tool_changes) {
    EXPECT_TRUE(tc.reached) << tc.arm;
    const CheckRow* reach = FindRow(report, "reach", "tools " + tc.arm);
    const CheckRow* clear = FindRow(report, "clearance", "tools " + tc.arm);
    ASSERT_NE(reach, nullptr) << tc.arm;
    ASSERT_NE(clear, nullptr) << tc.arm;
    EXPECT_TRUE(reach->pass) << reach->detail;
    // Where the rule puts the flange, by forward kinematics.
    const ArmUse& use = *FindArm(layout, tc.arm);
    const int ai = scene->ArmIndex(tc.arm);
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e9);
    Eigen::Vector3d hi = -lo;
    for (const Tool& t : use.tools) {
      const Eigen::Vector3d c(t.waits_at[0], t.waits_at[1], t.waits_at[2]);
      const Eigen::Vector3d h(t.wait_size[0] / 2, t.wait_size[1] / 2,
                              t.wait_size[2] / 2);
      lo = lo.cwiseMin(c - h);
      hi = hi.cwiseMax(c + h);
    }
    const Eigen::Vector3d local((lo.x() + hi.x()) / 2, (lo.y() + hi.y()) / 2,
                                hi.z() + kToolChangeHeight);
    const double d = M_PI / 180.0;
    const Eigen::Matrix3d r =
        (Eigen::AngleAxisd(use.base_rpy_deg[2] * d, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(use.base_rpy_deg[1] * d, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(use.base_rpy_deg[0] * d, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    Eigen::Vector3d base(use.base_pos[0], use.base_pos[1], use.base_pos[2]);
    if (use.track) {
      const Eigen::Vector3d dir(use.track->direction[0],
                                use.track->direction[1],
                                use.track->direction[2]);
      base += use.wait_track * dir.normalized();
    }
    const Eigen::Vector3d expect = base + r * local;
    DataPtr data = scene->MakeData();
    ArmPose pose;
    for (int j = 0; j < 6; ++j) pose.q[j] = tc.deg[j] * d;
    pose.track = use.wait_track;
    scene->SetPose(data.get(), ai, pose);
    mj_kinematics(&scene->model(), data.get());
    const int site = scene->arm(ai).h->flange_site;
    const Eigen::Vector3d at(data->site_xpos[3 * site],
                             data->site_xpos[3 * site + 1],
                             data->site_xpos[3 * site + 2]);
    EXPECT_LT((at - expect).norm(), 2e-4) << tc.arm;
    EXPECT_NEAR(data->site_xmat[9 * site + 8], -1.0, 1e-5) << tc.arm;
    std::printf("tools %s: reach %.4f deg, clearance %.4f m (%s)\n",
                tc.arm.c_str(), reach->figure, clear->figure,
                clear->detail.c_str());
  }
}

// On the three flat rack files every tool-change pose is clear.
TEST(LayoutCheck, ToolChangePosesClearOnTheFlatRackFiles) {
  for (const char* name :
       {"flat_rack_1_pin_arm", "flat_rack_2_pin_arms", "flat_rack_3_pin_arms"}) {
    CheckOptions options;
    options.checks = {"reach", "clearance"};
    options.work_kinds = {"pins_take"};
    const CheckReport report = CheckLayout(FlatRackNamed(name), Paths(), options);
    int rows = 0;
    for (const CheckRow& r : report.rows) {
      if (r.item.rfind("tools ", 0) != 0) continue;
      ++rows;
      EXPECT_TRUE(r.pass) << name << " " << r.check << " " << r.item << ": "
                          << r.figure << " " << r.detail;
    }
    EXPECT_GT(rows, 0) << name;
  }
}

}  // namespace
}  // namespace cell::layout

namespace cell::layout {
namespace {

// The windings brought near the anchor change the joints' numbers, not the
// pose: every work pose still puts the tool's working point on its point.
// And each arm's posture changes are reported, one line per arm.
TEST(LayoutCheck, PostureKeptAndPosesUnchanged) {
  std::string error;
  const std::optional<Layout> layout =
      LoadLayout(Dir() + "/flat_rack_1_pin_arm.json", &error);
  ASSERT_TRUE(layout.has_value()) << error;
  CheckOptions options;
  options.checks = {"reach", "time"};
  const CheckReport report = CheckLayout(*layout, Paths(), options);
  auto scene = LayoutScene::Load(*layout, Paths(), 5.0, &error);
  ASSERT_NE(scene, nullptr) << error;
  DataPtr data = scene->MakeData();
  std::map<std::string, std::vector<WorkPoint>> points;
  int checked = 0;
  for (const SolvedPoint& p : report.points) {
    if (!p.work_reached) continue;
    if (!points.count(p.kind)) points[p.kind] = WorkPoints(*layout, p.kind);
    const WorkPoint& w = points[p.kind][p.index];
    const int ai = scene->ArmIndex(p.arm);
    ASSERT_GE(ai, 0);
    ArmPose pose;
    for (int j = 0; j < 6; ++j) pose.q[j] = p.work_deg[j] * M_PI / 180.0;
    pose.track = p.track;
    scene->AllAtWait(data.get());
    scene->SetPose(data.get(), ai, pose);
    mj_kinematics(&scene->model(), data.get());
    const int site = scene->arm(ai).h->work_site.at(p.tool);
    const Eigen::Vector3d at(data->site_xpos[3 * site],
                             data->site_xpos[3 * site + 1],
                             data->site_xpos[3 * site + 2]);
    EXPECT_LT((at - w.pos).norm(), 1.5e-4) << p.item;
    const Eigen::Vector3d z(data->site_xmat[9 * site + 2],
                            data->site_xmat[9 * site + 5],
                            data->site_xmat[9 * site + 8]);
    EXPECT_GT(z.dot(w.z.normalized()), std::cos(1.5e-3)) << p.item;
    ++checked;
  }
  EXPECT_GT(checked, 150);
  ASSERT_EQ(report.postures.size(), layout->arms.size());
  for (const PostureChanges& p : report.postures) {
    EXPECT_FALSE(p.from.empty()) << p.arm;
    EXPECT_GT(p.largest_s, 0.0) << p.arm;
  }
}

// One posture per module (ArmUse::one_posture_per_module): the balanced
// two-rack layout sets it for pin arm A, whose holes 0 and 2 of a module
// otherwise come out in one pose family and holes 1 and 3 in the other
// (run 028: joint 1 163 degrees from hole 2 to hole 3). With it every move
// of A from one hole of a module to the next stays small and every row
// passes; without it the crossing is there.
TEST(LayoutCheck, OnePosturePerModule) {
  std::string error;
  const std::optional<Layout> layout =
      LoadLayout(Dir() + "/two_flat_racks_2_pin_arms_balanced.json", &error);
  ASSERT_TRUE(layout.has_value()) << error;
  CheckOptions options;
  options.checks = {"reach", "clearance", "approach"};
  options.work_kinds = {"locate"};
  // The largest change of one joint between consecutive holes of a module
  // of pin arm A, degrees, over its modules.
  auto largest = [](const CheckReport& report) {
    std::map<int, std::map<int, std::array<double, 6>>> by_module;
    for (const SolvedPoint& p : report.points) {
      if (p.kind != "locate" || p.arm != "pin_arm_a" || !p.work_reached) {
        continue;
      }
      by_module[p.index / 4][p.index % 4] = p.work_deg;
    }
    double worst = 0.0;
    int holes = 0;
    for (const auto& [module, holes_of] : by_module) {
      for (int h = 1; h < 4; ++h) {
        if (!holes_of.count(h) || !holes_of.count(h - 1)) continue;
        for (int j = 0; j < 6; ++j) {
          worst = std::max(worst, std::abs(holes_of.at(h)[j] -
                                           holes_of.at(h - 1)[j]));
        }
      }
      holes += static_cast<int>(holes_of.size());
    }
    EXPECT_EQ(holes, 20);  // Modules 0, 1, 3, 4 and 6.
    return worst;
  };
  ASSERT_TRUE(layout->arms[1].one_posture_per_module);
  const CheckReport with = CheckLayout(*layout, Paths(), options);
  EXPECT_EQ(Failures(with), "");
  EXPECT_LT(largest(with), 30.0);
  int preset = 0;
  for (const CheckRow& r : with.rows) {
    preset += r.check == "reach" &&
              r.detail.find("(one posture per module)") != std::string::npos;
  }
  EXPECT_EQ(preset, 20);
  Layout without = *layout;
  without.arms[1].one_posture_per_module = false;
  const CheckReport plain = CheckLayout(without, Paths(), options);
  EXPECT_GT(largest(plain), 150.0);
}

// A layout worked in stages, made in memory from a flat rack layout: each
// stage the holder's own grips and angle pin, at its offset.
Layout Staged(const Layout& base,
              const std::vector<std::tuple<std::string, double, std::string,
                                           std::string>>& stages) {
  Layout l = base;
  Staging staging;
  staging.why = "test";
  staging.transfer_s = 30.0;
  for (const auto& [name, offset, rails, modules] : stages) {
    Stage st;
    st.name = name;
    st.tube_offset_m = offset;
    st.rails = rails;
    st.modules = modules;
    st.grips = base.holder.grips;
    st.angle_pin_hold = base.holder.angle_pin_hold;
    staging.stages.push_back(st);
  }
  l.staging = staging;
  return l;
}

// One stage with everything at offset 0 is the unstaged bay: the same
// figures, the stage's rows prefixed, the rows made once not.
TEST(LayoutCheck, OneStageIsTheWholeBay) {
  const Layout plain = FlatRackNamed("flat_rack_2_pin_arms");
  const Layout staged = Staged(plain, {{"1", 0.0, "all", "all"}});
  CheckOptions options;
  options.checks = {"reach", "clearance"};
  options.work_kinds = {"strap_bolt", "module_set"};
  const CheckReport a = CheckLayout(plain, Paths(), options);
  const CheckReport b = CheckLayout(staged, Paths(), options);
  ASSERT_FALSE(a.rows.empty());
  EXPECT_EQ(a.rows.size(), b.rows.size());
  int prefixed = 0;
  for (const CheckRow& r : a.rows) {
    const CheckRow* s = FindRow(b, r.check, "stage 1: " + r.item);
    if (s != nullptr) {
      ++prefixed;
    } else {
      s = FindRow(b, r.check, r.item);
    }
    ASSERT_NE(s, nullptr) << r.check << " " << r.item;
    EXPECT_DOUBLE_EQ(s->figure, r.figure) << r.check << " " << r.item;
    EXPECT_EQ(s->pass, r.pass) << r.check << " " << r.item;
  }
  EXPECT_GT(prefixed, 19);
  ASSERT_NE(FindRow(b, "reach", "wait pose module_arm"), nullptr);
}

// Two stages, the second with the tube moved by d along Y: its points are
// solved there, and the first stage's modules are on at its points.
TEST(LayoutCheck, SecondStageMovesTheTubeAndKeepsTheFirstStagesModules) {
  constexpr double d = 0.2;
  const Layout plain = FlatRackNamed("flat_rack_2_pin_arms");
  const Layout staged =
      Staged(plain, {{"1", 0.0, "0-4", "0-3"}, {"2", d, "5-9", "4-8"}});
  CheckOptions options;
  options.checks = {"hold", "reach"};
  options.work_kinds = {"rail_set", "module_set"};
  const CheckReport report = CheckLayout(staged, Paths(), options);
  ASSERT_NE(FindRow(report, "reach", "stage 2: rail_set 7"), nullptr);
  // The holder's saddles stay where the grips are at stage 1; at stage 2 the
  // gripped pieces are d off them along Y.
  const CheckRow* grip1 =
      FindRow(report, "hold", "stage 1: grip bay_hold_2 in the holder");
  const CheckRow* grip2 =
      FindRow(report, "hold", "stage 2: grip bay_hold_2 in the holder");
  ASSERT_NE(grip1, nullptr);
  ASSERT_NE(grip2, nullptr);
  EXPECT_TRUE(grip1->pass) << grip1->detail;
  EXPECT_FALSE(grip2->pass);
  EXPECT_NEAR(grip2->figure, grip1->figure - d, 1e-9);
  // The angle pin's box must reach the end stretch along Y: moved d away
  // from it, the overlap is d less.
  const CheckRow* pin1 =
      FindRow(report, "hold", "stage 1: angle pin bay_hold_0 at holder");
  const CheckRow* pin2 =
      FindRow(report, "hold", "stage 2: angle pin bay_hold_0 at holder");
  ASSERT_NE(pin1, nullptr);
  ASSERT_NE(pin2, nullptr);
  EXPECT_NEAR(pin2->figure, pin1->figure - d, 1e-9) << pin2->detail;
  EXPECT_DOUBLE_EQ(pin2->limit, 0.0);
  ASSERT_NE(FindRow(report, "reach", "stage 1: module_set 3"), nullptr);
  EXPECT_EQ(FindRow(report, "reach", "stage 1: rail_set 7"), nullptr);
  const Layout second = StageLayout(staged, 1);
  std::string error;
  auto scene = LayoutScene::Load(second, Paths(), 5.0, &error);
  ASSERT_NE(scene, nullptr) << error;
  DataPtr data = scene->MakeData();
  int checked = 0;
  for (const SolvedPoint& p : report.points) {
    const bool two = p.stage == "2";
    EXPECT_EQ(two, StageOfPoint(staged, p.kind, p.index) == 1) << p.item;
    if (!two || !p.work_reached) continue;
    // The working point is at the plain point moved by d along Y.
    const WorkPoint w = WorkPoints(plain, p.kind)[p.index];
    const int ai = scene->ArmIndex(p.arm);
    ArmPose pose;
    for (int j = 0; j < 6; ++j) pose.q[j] = p.work_deg[j] * M_PI / 180.0;
    pose.track = p.track;
    scene->AllAtWait(data.get());
    scene->SetPose(data.get(), ai, pose);
    mj_kinematics(&scene->model(), data.get());
    const int site = scene->arm(ai).h->work_site.at(p.tool);
    const Eigen::Vector3d at(data->site_xpos[3 * site],
                             data->site_xpos[3 * site + 1],
                             data->site_xpos[3 * site + 2]);
    EXPECT_LT((at - w.pos - Eigen::Vector3d(0.0, d, 0.0)).norm(), 1.5e-4)
        << p.item;
    ++checked;
  }
  EXPECT_GT(checked, 3);
  // At stage 2 the modules of stage 1 are on; rails of stage 2 come in
  // order; a module of stage 1 is not worked with one of stage 2 beside it.
  const StageScope two = ScopeOfStage(staged, 1);
  StepScene rail6 = scene->AtWorkPoint(WorkPoints(second, "rail_set")[6]);
  BuiltInStage(two, PointCount(staged, "module_take"), &rail6);
  ASSERT_EQ(rail6.built.size(), 1u);
  EXPECT_EQ(rail6.built[0].rails, (std::vector<int>{0, 1, 2, 3, 4, 5}));
  EXPECT_EQ(rail6.built[0].modules, (std::vector<int>{0, 1, 2, 3}));
  EXPECT_EQ(rail6.built[0].on_pallets, PointCount(staged, "module_take") - 4);
  StepScene set5 = scene->AtWorkPoint(WorkPoints(second, "module_set")[5]);
  BuiltInStage(two, PointCount(staged, "module_take"), &set5);
  EXPECT_EQ(set5.built[0].rails.size(), 10u);
  EXPECT_EQ(set5.built[0].modules, (std::vector<int>{0, 1, 2, 3, 4}));
  StepScene hole = scene->AtWorkPoint(WorkPoints(second, "locate")[4 * 3 + 2]);
  BuiltInStage(ScopeOfStage(staged, 0), PointCount(staged, "module_take"),
               &hole);
  ASSERT_EQ(hole.built.size(), 2u);
  EXPECT_EQ(hole.built[1].modules, (std::vector<int>{0, 1, 2, 3}));
  EXPECT_EQ(hole.built[1].rails, (std::vector<int>{0, 1, 2, 3, 4}));
}

// On two racks: the gap rows, one per variant from the layout's figures,
// each the least distance of its rerun rows; rack "b" moved by nothing is
// the nominal cell.
TEST(LayoutCheck, GapRowsOnTwoRacks) {
  const Layout layout = FlatRackNamed("two_flat_racks_2_pin_arms");
  ASSERT_EQ(layout.base, "two_flat_racks");
  const TwoFlatRacksState& t = layout.two_flat_racks;
  CheckOptions options;
  options.checks = {"gap"};
  options.work_kinds = {"rail_set"};
  const CheckReport gap = CheckLayout(layout, Paths(), options);
  auto mm = [](const char* format, double m) {
    char b[64];
    std::snprintf(b, sizeof(b), format, m * 1000.0);
    return std::string(b);
  };
  std::vector<std::string> items = {
      mm("gap %.0f mm", t.gap_lo_m), mm("gap %.0f mm", t.gap_hi_m),
      mm("misaligned %+.0f mm", t.misalign_m),
      mm("misaligned %+.0f mm", -t.misalign_m)};
  for (double g : {t.gap_lo_m, t.gap_hi_m}) {
    for (double m : {t.misalign_m, -t.misalign_m}) {
      items.push_back(mm("gap %.0f mm", g) + ", " + mm("misaligned %+.0f mm", m));
    }
  }
  ASSERT_EQ(gap.rows.size(), 8u);
  for (size_t i = 0; i < items.size(); ++i) {
    EXPECT_EQ(gap.rows[i].check, "gap");
    EXPECT_EQ(gap.rows[i].item, items[i]);
    EXPECT_EQ(gap.rows[i].unit, "m");
    EXPECT_DOUBLE_EQ(gap.rows[i].limit, gap.clearance_m);
    EXPECT_NE(gap.rows[i].detail.find("against the nominal gap"),
              std::string::npos);
  }
  // The first variant by hand: the gap at its low end.
  options.checks = {"reach", "clearance", "ways", "approach"};
  const CheckReport low = CheckLayout(
      MovedRackB(layout, t.gap_lo_m - t.gap_m, 0.0), Paths(), options);
  double least = 1e9;
  bool pass = true;
  for (const CheckRow& r : low.rows) {
    pass = pass && r.pass;
    if (r.check != "reach" && r.unit == "m" && r.limit == low.clearance_m) {
      least = std::min(least, r.figure);
    }
  }
  EXPECT_DOUBLE_EQ(gap.rows[0].figure, least);
  EXPECT_EQ(gap.rows[0].pass, pass);
  // MovedRackB by nothing: the nominal figures.
  options.checks = {"reach", "clearance", "ways"};
  const CheckReport a = CheckLayout(layout, Paths(), options);
  const CheckReport b = CheckLayout(MovedRackB(layout, 0.0, 0.0), Paths(), options);
  ASSERT_EQ(a.rows.size(), b.rows.size());
  ASSERT_FALSE(a.rows.empty());
  for (size_t i = 0; i < a.rows.size(); ++i) {
    EXPECT_EQ(a.rows[i].item, b.rows[i].item);
    EXPECT_DOUBLE_EQ(a.rows[i].figure, b.rows[i].figure) << a.rows[i].item;
  }
}

}  // namespace
}  // namespace cell::layout
