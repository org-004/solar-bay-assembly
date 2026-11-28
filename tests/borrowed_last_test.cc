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
// swage_after_pins and with or without borrowed_last.
Layout Lent(bool borrowed_last) {
  std::ifstream in(LayoutFile(kV24), std::ios::binary);
  nlohmann::json j = nlohmann::json::parse(in);
  EXPECT_FALSE(j.contains("borrowed_last"));
  nlohmann::json* a = nullptr;
  nlohmann::json* b = nullptr;
  for (nlohmann::json& u : j["arms"]) {
    if (u["name"] == "pin_arm_a") a = &u;
    if (u["name"] == "pin_arm_b") b = &u;
  }
  EXPECT_TRUE(a != nullptr && b != nullptr);
  if (a == nullptr || b == nullptr) return {};
  for (int h : {12, 13}) (*a)["stops"]["swage"][h] = (*b)["stops"]["swage"][h];
  for (nlohmann::json& w : j["work"]) {
    if (w["kind"] != "swage") continue;
    if (w["arm"] == "pin_arm_a") w["points"] = "0-13";
    if (w["arm"] == "pin_arm_b") w["points"] = "14-23";
  }
  j["swage_after_pins"] = true;
  j["borrowed_last"] = borrowed_last;
  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string path = std::string(tmp != nullptr ? tmp : "/tmp") + "/" + kV24 + ".json";
  std::ofstream(path, std::ios::binary) << j.dump(2);
  std::string error;
  std::optional<Layout> l = LoadLayout(path, &error);
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

}  // namespace
}  // namespace cell::layout
