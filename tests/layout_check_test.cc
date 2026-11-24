// The layout checker: the reference layout's report, and copies of it broken
// in memory, one check at a time, which must fail that check.

#include "src/layout_check.h"

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
  EXPECT_NEAR((takes[1].pos - takes[0].pos).norm(), layout.pallet.pitch, 1e-12);
  EXPECT_NEAR(takes[0].pos.z(),
              layout.pallet.pos[2] + layout.pallet.base_height +
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
