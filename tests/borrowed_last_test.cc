// Borrowed work after an arm's own modules ("borrowed_last", src/layout.h,
// run 072). With swage_after_pins two pin arms may swage one module; without
// borrowed_last the arm that borrows a swage does it at its module's place
// in set order, so pin arm A swages B's 3/1 and 3/0 before its own module 1
// and stands waiting for B's last pin 3/1. With borrowed_last the checker's
// step list (src/layout_check.cc, SolveAll), which the planner, the verifier
// and the critical-chain walk follow, puts A's borrowed swages after A's own
// last module (module 2, set seventh). The tests use v24's layout with A
// swaging 3/1 and 3/0 (tools/r1_072/make.py's a31); v24's committed plan,
// without the field, is unchanged (plan_replay_test).
//
// The same target tests borrowed_after (run 073): an arm it names does its
// borrowed work right after its n-th own module of the bay in set order. With
// B also swaging C's 6/1 and 6/0 (tools/r1_073/make.py's a31_b61) and
// {"pin_arm_b": 2}, B's borrowed swages stand between its own modules 4 and 5.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"

namespace cell::layout {
namespace {

constexpr const char* kV24 =
    "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat_ma_split_paths_torque_clear_out_infeed_"
    "rail_mag_early_next_fill_pallet_near";

std::string LayoutFile(const std::string& name) {
  const char* dir = std::getenv("LAYOUT_DIR");
  return std::string(dir != nullptr ? dir : "layouts") + "/" + name + ".json";
}

// v24 with A swaging holes 12 and 13 (3/0, 3/1) from B's stops, with
// swage_after_pins and with or without borrowed_last. With b61 B also swages
// holes 24 and 25 (6/0, 6/1) from C's stops; `after`, if not null, is the
// layout's borrowed_after.
Layout Lent(bool borrowed_last, bool b61 = false, const nlohmann::json* after = nullptr,
            std::string* load_error = nullptr) {
  std::ifstream in(LayoutFile(kV24), std::ios::binary);
  nlohmann::json j = nlohmann::json::parse(in);
  EXPECT_FALSE(j.contains("borrowed_last"));
  EXPECT_FALSE(j.contains("borrowed_after"));
  nlohmann::json* a = nullptr;
  nlohmann::json* b = nullptr;
  nlohmann::json* c = nullptr;
  for (nlohmann::json& u : j["arms"]) {
    if (u["name"] == "pin_arm_a") a = &u;
    if (u["name"] == "pin_arm_b") b = &u;
    if (u["name"] == "pin_arm_c") c = &u;
  }
  EXPECT_TRUE(a != nullptr && b != nullptr && c != nullptr);
  if (a == nullptr || b == nullptr || c == nullptr) return {};
  for (int h : {12, 13}) (*a)["stops"]["swage"][h] = (*b)["stops"]["swage"][h];
  if (b61) {
    for (int h : {24, 25}) (*b)["stops"]["swage"][h] = (*c)["stops"]["swage"][h];
  }
  for (nlohmann::json& w : j["work"]) {
    if (w["kind"] != "swage") continue;
    if (w["arm"] == "pin_arm_a") w["points"] = "0-13";
    if (w["arm"] == "pin_arm_b") w["points"] = b61 ? "14-25" : "14-23";
    if (w["arm"] == "pin_arm_c" && b61) w["points"] = "26-35";
  }
  j["swage_after_pins"] = true;
  j["borrowed_last"] = borrowed_last;
  if (after != nullptr) j["borrowed_after"] = *after;
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string path = std::string(tmp != nullptr ? tmp : "/tmp") + "/" + kV24 + ".json";
  std::ofstream(path, std::ios::binary) << j.dump(2);
  std::string error;
  std::optional<Layout> l = LoadLayout(path, &error);
  if (load_error != nullptr) {
    *load_error = error;
    return l ? *l : Layout{};
  }
  EXPECT_TRUE(l.has_value()) << error;
  if (!l) return {};
  EXPECT_EQ(l->borrowed_last, borrowed_last);
  return *l;
}

// The checker's steps of one bay.
std::vector<BayStep> Steps(const Layout& l) {
  const CheckReport r = CheckLayout(l, ModelPaths::FromEnvironment());
  EXPECT_TRUE(r.error.empty()) << r.error;
  return r.steps;
}

int Number(const std::vector<BayStep>& steps, const std::string& what) {
  for (const BayStep& s : steps) {
    if (s.what == what) return s.number;
  }
  ADD_FAILURE() << "no step " << what;
  return -1;
}

// The hole steps of `arm`, in the order of the step list.
std::vector<std::string> Holes(const std::vector<BayStep>& steps, const std::string& arm) {
  std::vector<std::string> out;
  for (const BayStep& s : steps) {
    if (s.arm == arm && (s.kind == "locate" || s.kind == "pin" || s.kind == "swage")) {
      out.push_back(s.what);
    }
  }
  return out;
}

TEST(BorrowedLast, TheFieldIsReadAndOptional) {
  std::string error;
  std::optional<Layout> l = LoadLayout(LayoutFile(kV24), &error);
  ASSERT_TRUE(l.has_value()) << error;
  EXPECT_FALSE(l->borrowed_last);
  EXPECT_TRUE(l->borrowed_after.empty());
}

// Without the field A's borrowed swages stand at module 3's place, between
// A's modules 0 and 1; with it they are A's last two hole steps, after its
// swage 2/0, in the order 3/1, 3/0. Every other arm's hole steps keep their
// order, and the step list has the same steps.
TEST(BorrowedLast, TheStepListPutsBorrowedSwagesAfterTheArmsOwnModules) {
  const std::vector<BayStep> before = Steps(Lent(false));
  const std::vector<BayStep> after = Steps(Lent(true));
  ASSERT_EQ(before.size(), after.size());
  const std::vector<std::string> a_before = Holes(before, "pin_arm_a");
  const std::vector<std::string> a_after = Holes(after, "pin_arm_a");
  ASSERT_GE(a_after.size(), 3u);
  EXPECT_EQ(a_after[a_after.size() - 3], "swage 2/0");
  EXPECT_EQ(a_after[a_after.size() - 2], "swage 3/1");
  EXPECT_EQ(a_after.back(), "swage 3/0");
  EXPECT_LT(Number(before, "swage 3/0"), Number(before, "locate 1/0"));
  EXPECT_GT(Number(after, "swage 3/1"), Number(after, "swage 2/0"));
  EXPECT_GT(Number(after, "swage 3/1"), Number(after, "swage 8/0"));
  for (const char* arm : {"pin_arm_b", "pin_arm_c", "module_arm"}) {
    EXPECT_EQ(Holes(before, arm), Holes(after, arm)) << arm;
  }
}

// In a plan of the steps from A's swage 2/0 to the end of the module phase,
// A swages 3/1 and 3/0 after its swage 2/0 has ended, in that order.
TEST(BorrowedLast, APlanDoesTheBorrowedSwagesAfterTheOwnLastModule) {
  const Layout l = Lent(true);
  const std::vector<BayStep> steps = Steps(l);
  const int own = Number(steps, "swage 2/0");
  const int s31 = Number(steps, "swage 3/1");
  const int s30 = Number(steps, "swage 3/0");
  PlanOptions o;
  o.first_step = own;
  o.last_step = s30;
  o.budgets = {5000, 40000, 160000, 640000};
  const PlanResult r =
      PlanLayout(l, ModelPaths::FromEnvironment(), Sha256OfFile(LayoutFile(kV24)), o);
  for (const std::string& f : r.faults) {
    if (f.find("never complete") == std::string::npos) ADD_FAILURE() << f;
  }
  double own_end = -1.0, s31_start = -1.0, s31_end = -1.0, s30_start = -1.0;
  for (const ArmPlan& a : r.plan.arms) {
    if (a.name != "pin_arm_a") continue;
    for (const Segment& s : a.segments) {
      if (s.kind == "wait") continue;
      if (s.step == own) own_end = std::max(own_end, s.t1);
      if (s.step == s31 && s31_start < 0.0) s31_start = s.t0;
      if (s.step == s31) s31_end = std::max(s31_end, s.t1);
      if (s.step == s30 && s30_start < 0.0) s30_start = s.t0;
    }
  }
  ASSERT_GE(own_end, 0.0);
  ASSERT_GE(s31_start, 0.0);
  ASSERT_GE(s30_start, 0.0);
  std::printf("A: swage 2/0 (step %d) ends %.2f s; borrowed swage 3/1 (step %d) %.2f to %.2f s, "
              "3/0 (step %d) from %.2f s\n",
              own, own_end, s31, s31_start, s31_end, s30, s30_start);
  EXPECT_GE(s31_start, own_end - 1e-6);
  EXPECT_GE(s30_start, s31_end - 1e-6);
}

// The position of `what` in `list`, or -1.
int At(const std::vector<std::string>& list, const std::string& what) {
  const auto it = std::find(list.begin(), list.end(), what);
  return it == list.end() ? -1 : static_cast<int>(it - list.begin());
}

// borrowed_after {"pin_arm_b": 2}: B's own modules in set order are 3, 4, 5,
// so its borrowed swages 6/1 and 6/0 come right after its swage 4/0 (the
// last step of its second own module) and before its locate 5/0. A, not
// named, keeps borrowed_last's order. With {"pin_arm_b": 3} B's order is
// borrowed_last's; with {"pin_arm_b": 1} it is the set order (module 6 is
// set after B's module 3).
TEST(BorrowedAfter, TheStepListPutsBorrowedSwagesBetweenTheArmsOwnModules) {
  const nlohmann::json two = {{"pin_arm_b", 2}};
  const Layout l = Lent(true, true, &two);
  ASSERT_EQ(l.borrowed_after.size(), 1u);
  EXPECT_EQ(l.borrowed_after.at("pin_arm_b"), 2);
  const std::vector<BayStep> last = Steps(Lent(true, true));
  const std::vector<BayStep> after = Steps(l);
  ASSERT_EQ(last.size(), after.size());
  const std::vector<std::string> b = Holes(after, "pin_arm_b");
  const int s40 = At(b, "swage 4/0");
  ASSERT_GE(s40, 0);
  ASSERT_LT(s40 + 3, static_cast<int>(b.size()));
  EXPECT_EQ(b[s40 + 1], "swage 6/1");
  EXPECT_EQ(b[s40 + 2], "swage 6/0");
  EXPECT_EQ(b[s40 + 3], "locate 5/0");
  EXPECT_EQ(b.back(), "swage 5/0");
  EXPECT_GT(Number(after, "swage 6/1"), Number(after, "swage 4/0"));
  EXPECT_LT(Number(after, "swage 6/0"), Number(after, "locate 5/0"));
  // Under borrowed_last alone the same swages are B's last.
  const std::vector<std::string> b_last = Holes(last, "pin_arm_b");
  EXPECT_EQ(b_last.back(), "swage 6/0");
  for (const char* arm : {"pin_arm_a", "pin_arm_c", "module_arm"}) {
    EXPECT_EQ(Holes(last, arm), Holes(after, arm)) << arm;
  }
  const nlohmann::json three = {{"pin_arm_b", 3}};
  EXPECT_EQ(Holes(Steps(Lent(true, true, &three)), "pin_arm_b"), b_last);
  const nlohmann::json one = {{"pin_arm_b", 1}};
  EXPECT_EQ(Holes(Steps(Lent(true, true, &one)), "pin_arm_b"),
            Holes(Steps(Lent(false, true)), "pin_arm_b"));
}

TEST(BorrowedAfter, TheFieldNamesAnArmAndACountFromOneToNine) {
  for (const nlohmann::json& bad :
       {nlohmann::json{{"pin_arm_x", 2}}, nlohmann::json{{"pin_arm_b", 0}},
        nlohmann::json{{"pin_arm_b", 10}}, nlohmann::json::object(), nlohmann::json(2)}) {
    std::string error;
    Lent(true, true, &bad, &error);
    EXPECT_NE(error.find("borrowed_after"), std::string::npos) << bad.dump() << ": " << error;
  }
}

// In a plan of B's steps from its swage 4/0 to its locate 5/0, B swages 6/1
// and 6/0 after its swage 4/0 has ended and before its locate 5/0 starts.
TEST(BorrowedAfter, APlanDoesTheBorrowedSwagesBetweenTheOwnModules) {
  const nlohmann::json two = {{"pin_arm_b", 2}};
  const Layout l = Lent(true, true, &two);
  const std::vector<BayStep> steps = Steps(l);
  const int own = Number(steps, "swage 4/0");
  const int s61 = Number(steps, "swage 6/1");
  const int s60 = Number(steps, "swage 6/0");
  const int l50 = Number(steps, "locate 5/0");
  PlanOptions o;
  o.first_step = own;
  o.last_step = l50;
  o.budgets = {5000, 40000, 160000, 640000};
  const PlanResult r =
      PlanLayout(l, ModelPaths::FromEnvironment(), Sha256OfFile(LayoutFile(kV24)), o);
  for (const std::string& f : r.faults) {
    if (f.find("never complete") == std::string::npos) ADD_FAILURE() << f;
  }
  double own_end = -1.0, s61_start = -1.0, s61_end = -1.0, s60_start = -1.0, s60_end = -1.0,
         l50_start = -1.0;
  for (const ArmPlan& a : r.plan.arms) {
    if (a.name != "pin_arm_b") continue;
    for (const Segment& s : a.segments) {
      if (s.kind == "wait") continue;
      if (s.step == own) own_end = std::max(own_end, s.t1);
      if (s.step == s61 && s61_start < 0.0) s61_start = s.t0;
      if (s.step == s61) s61_end = std::max(s61_end, s.t1);
      if (s.step == s60 && s60_start < 0.0) s60_start = s.t0;
      if (s.step == s60) s60_end = std::max(s60_end, s.t1);
      if (s.step == l50 && l50_start < 0.0) l50_start = s.t0;
    }
  }
  ASSERT_GE(own_end, 0.0);
  ASSERT_GE(s61_start, 0.0);
  ASSERT_GE(s60_start, 0.0);
  ASSERT_GE(l50_start, 0.0);
  std::printf("B: swage 4/0 (step %d) ends %.2f s; borrowed 6/1 (step %d) %.2f to %.2f s, 6/0 "
              "(step %d) %.2f to %.2f s; locate 5/0 (step %d) from %.2f s\n",
              own, own_end, s61, s61_start, s61_end, s60, s60_start, s60_end, l50, l50_start);
  EXPECT_GE(s61_start, own_end - 1e-6);
  EXPECT_GE(s60_start, s61_end - 1e-6);
  EXPECT_GE(l50_start, s60_end - 1e-6);
}

}  // namespace
}  // namespace cell::layout
