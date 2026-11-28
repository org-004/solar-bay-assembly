// The run in physics of the chosen layout (src/physics_run.h), again: it
// runs, both its steps count by the rule of docs/scorecard.md, and it gives
// the committed record's figures (physics/<layout>.json) within tolerances,
// so that a change to the plan, the model or the follower that breaks the
// run, or moves its figures, is caught. The tolerances are for another
// platform or compiler, not for a change: MuJoCo's arithmetic may differ in
// the last digits, which a contact can grow to a few microns.

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/physics_record.h"
#include "src/physics_run.h"

namespace cell::layout {
namespace {

std::string Env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

std::string ReadText(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

TEST(PhysicsRun, TheChosenLayoutsModuleIsCarriedAndSetAsRecorded) {
  std::string error;
  const std::optional<std::string> name = ChosenLayout(VersionsPath(), &error);
  ASSERT_TRUE(name.has_value()) << error;
  const std::string layout_path = Env("LAYOUT_DIR", "layouts") + "/" + *name + ".json";
  const std::string plan_path = Env("PLAN_DIR", "plans") + "/" + *name + ".json";
  const std::optional<Layout> layout = LoadLayout(layout_path, &error);
  ASSERT_TRUE(layout.has_value()) << error;
  const std::optional<Plan> plan = PlanFromJson(ReadText(plan_path), &error);
  ASSERT_TRUE(plan.has_value()) << error;
  const PhysicsResult r =
      RunPhysics(*layout, *plan, ModelPaths::FromEnvironment(), PhysicsOptions{},
                 Sha256OfFile(layout_path), Sha256OfFile(plan_path));
  ASSERT_TRUE(r.ok) << r.error;
  std::printf("%s", PhysicsText(r).c_str());
  ASSERT_EQ(r.record.takes.size(), 1u);
  const PhysicsTake& run = r.record.takes.front();
  ASSERT_EQ(run.steps.size(), 2u);
  for (const PhysicsStep& s : run.steps) {
    EXPECT_TRUE(s.counted) << "step " << s.number << ": least clearance "
                           << s.least_clearance_m << " m, set-down "
                           << run.set_down.offset_mm << " mm";
  }
  EXPECT_TRUE(CheckPhysicsRecord(r.record, *plan, Sha256OfFile(plan_path)).empty());

  const std::string record_path = Env("PHYSICS_DIR", "physics") + "/" + *name + ".json";
  const std::string text = ReadText(record_path);
  if (text.empty()) {
    GTEST_SKIP() << record_path << " is not committed yet";
  }
  const std::optional<PhysicsRecord> committed = PhysicsRecordFromJson(text, &error);
  ASSERT_TRUE(committed.has_value()) << error;
  EXPECT_EQ(committed->plan_sha256, r.record.plan_sha256);
  EXPECT_EQ(committed->layout_sha256, r.record.layout_sha256);
  // The record may hold several takes of the bay: this run's is the one of
  // its module.
  const PhysicsTake* kept = nullptr;
  for (const PhysicsTake& t : committed->takes) {
    if (t.module == run.module) kept = &t;
  }
  ASSERT_NE(kept, nullptr) << "the record holds no take of module " << run.module;
  EXPECT_EQ(kept->take, run.take);
  ASSERT_EQ(kept->steps.size(), run.steps.size());
  for (size_t i = 0; i < kept->steps.size(); ++i) {
    const PhysicsStep& a = kept->steps[i];
    const PhysicsStep& b = run.steps[i];
    EXPECT_EQ(a.number, b.number);
    EXPECT_EQ(a.counted, b.counted) << "step " << a.number;
    EXPECT_NEAR(a.planned_s, b.planned_s, 1e-6) << "step " << a.number;
    EXPECT_NEAR(a.least_clearance_m, b.least_clearance_m, 1e-4) << "step " << a.number;
    EXPECT_NEAR(a.plan_least_clearance_m, b.plan_least_clearance_m, 1e-4) << "step " << a.number;
    EXPECT_NEAR(a.max_joint_error_mrad, b.max_joint_error_mrad,
                0.05 + 0.05 * a.max_joint_error_mrad) << "step " << a.number;
    EXPECT_NEAR(a.max_frame_error_mm, b.max_frame_error_mm,
                0.05 + 0.05 * a.max_frame_error_mm) << "step " << a.number;
  }
  EXPECT_NEAR(kept->set_down.offset_mm, run.set_down.offset_mm, 0.05);
  EXPECT_NEAR(kept->set_down.yaw_deg, run.set_down.yaw_deg, 0.005);
  if (committed->takes.size() == 1) {
    EXPECT_NEAR(committed->counted_s, r.record.counted_s, 1e-6);
  }
}

// The chosen layout's first rail set (run 074): the rail is carried at the
// gripper, let go on the tube and comes to rest there, it counts by the
// rail gate, and its figures are the committed record's.
TEST(PhysicsRun, TheChosenLayoutsFirstRailIsCarriedAndSetAsRecorded) {
  std::string error;
  const std::optional<std::string> name = ChosenLayout(VersionsPath(), &error);
  ASSERT_TRUE(name.has_value()) << error;
  const std::string layout_path = Env("LAYOUT_DIR", "layouts") + "/" + *name + ".json";
  const std::string plan_path = Env("PLAN_DIR", "plans") + "/" + *name + ".json";
  const std::optional<Layout> layout = LoadLayout(layout_path, &error);
  ASSERT_TRUE(layout.has_value()) << error;
  const std::optional<Plan> plan = PlanFromJson(ReadText(plan_path), &error);
  ASSERT_TRUE(plan.has_value()) << error;
  const std::vector<int> rails = BayRails(*layout, ModelPaths::FromEnvironment(), &error);
  if (rails.empty()) GTEST_SKIP() << "the chosen layout's module arm sets no rail";
  PhysicsOptions options;
  options.rail = rails.front();
  const PhysicsResult r = RunPhysics(*layout, *plan, ModelPaths::FromEnvironment(), options,
                                     Sha256OfFile(layout_path), Sha256OfFile(plan_path));
  ASSERT_TRUE(r.ok) << r.error;
  std::printf("%s", PhysicsText(r).c_str());
  EXPECT_GE(r.rail_tube_contacts, 1);
  EXPECT_LT(r.rail_speed_mm_s, 1.0);
  ASSERT_TRUE(r.record.takes.empty());
  ASSERT_EQ(r.record.rails.size(), 1u);
  const PhysicsRailSet& run = r.record.rails.front();
  EXPECT_TRUE(run.step.counted);
  EXPECT_TRUE(CheckPhysicsRecord(r.record, *plan, Sha256OfFile(plan_path)).empty());

  const std::string text = ReadText(Env("PHYSICS_DIR", "physics") + "/" + *name + ".json");
  const std::optional<PhysicsRecord> committed = PhysicsRecordFromJson(text, &error);
  ASSERT_TRUE(committed.has_value()) << error;
  const PhysicsRailSet* kept = nullptr;
  for (const PhysicsRailSet& s : committed->rails) {
    if (s.rail == run.rail) kept = &s;
  }
  ASSERT_NE(kept, nullptr) << "the record holds no set of rail " << run.rail;
  EXPECT_EQ(kept->step.number, run.step.number);
  EXPECT_EQ(kept->step.counted, run.step.counted);
  EXPECT_NEAR(kept->step.least_clearance_m, run.step.least_clearance_m, 1e-4);
  for (int j = 0; j < 7; ++j) EXPECT_NEAR(kept->torque_ratio[j], run.torque_ratio[j], 0.01);
  EXPECT_NEAR(kept->pose.across_mm, run.pose.across_mm, 0.05);
  EXPECT_NEAR(kept->pose.along_mm, run.pose.along_mm, 0.05);
  EXPECT_NEAR(kept->pose.round_deg, run.pose.round_deg, 0.005);
  EXPECT_NEAR(kept->pose.yaw_deg, run.pose.yaw_deg, 0.005);
}

}  // namespace
}  // namespace cell::layout
