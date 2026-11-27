// A layout's path_tries (src/layout.h, run 042): each move searched from
// that many seeds, the quickest timed path kept. With the field the planner
// keeps the quickest of the paths (QuickestPath), the plan file names the
// figure and the verifier holds a plan to the layout's. Without it nothing
// changes: one search a move, no figure in the plan file, and a plan that
// brings one is refused.

#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_scene.h"

namespace cell::layout {
namespace {

std::string Env(const char* name, const char* fallback) {
  const char* v = std::getenv(name);
  return v != nullptr ? v : fallback;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

std::string LayoutPath(const std::string& name) {
  return Env("LAYOUT_DIR", "layouts") + "/" + name + ".json";
}

constexpr const char* kV16 = "flat_rack_3_kr20_pin_arms_shared_ordered_fast_retreat";
constexpr const char* kPaths = "flat_rack_3_kr20_pin_arms_shared_ordered_fast_paths";
constexpr const char* kKr20 = "flat_rack_1_kr20_pin_arm_ordered";

// v16 with path_tries set to `tries` (null: none), written under the test's
// own directory and loaded.
std::optional<Layout> WithTries(const nlohmann::json& tries, std::string* error) {
  nlohmann::json j = nlohmann::json::parse(ReadFile(LayoutPath(kV16)));
  j.erase("path_tries");
  if (!tries.is_null()) j["path_tries"] = tries;
  const std::string path = Env("TEST_TMPDIR", ".") + "/" + kV16 + ".json";
  std::ofstream(path) << j.dump(2);
  return LoadLayout(path, error);
}

TEST(PathTries, TheFieldIsReadAndBounded) {
  std::string error;
  const std::optional<Layout> v16 = LoadLayout(LayoutPath(kV16), &error);
  ASSERT_TRUE(v16.has_value()) << error;
  EXPECT_FALSE(v16->path_tries.has_value());
  const std::optional<Layout> paths = LoadLayout(LayoutPath(kPaths), &error);
  ASSERT_TRUE(paths.has_value()) << error;
  ASSERT_TRUE(paths->path_tries.has_value());
  EXPECT_EQ(*paths->path_tries, 8);
  // The new layout is v16's and the field, nothing else.
  nlohmann::json a = nlohmann::json::parse(ReadFile(LayoutPath(kV16)));
  nlohmann::json b = nlohmann::json::parse(ReadFile(LayoutPath(kPaths)));
  for (const char* key : {"name", "title", "summary"}) {
    a.erase(key);
    b.erase(key);
  }
  b.erase("path_tries");
  EXPECT_EQ(a, b);
  for (int ok : {2, kMaxPathTries}) {
    const std::optional<Layout> l = WithTries(ok, &error);
    ASSERT_TRUE(l.has_value()) << error;
    EXPECT_EQ(*l->path_tries, ok);
  }
  for (const nlohmann::json& bad : {nlohmann::json(1), nlohmann::json(0),
                                    nlohmann::json(kMaxPathTries + 1), nlohmann::json(2.5),
                                    nlohmann::json("8")}) {
    EXPECT_FALSE(WithTries(bad, &error).has_value()) << bad;
    EXPECT_NE(error.find("path_tries"), std::string::npos) << error;
  }
}

// ArmMotion of the first KR 20 of v16's scene.
ArmMotion Kr20Motion(const PlanFigures& f) {
  std::string error;
  const std::optional<Layout> l = LoadLayout(LayoutPath(kV16), &error);
  EXPECT_TRUE(l.has_value()) << error;
  if (!l) return {};
  const std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(*l, ModelPaths::FromEnvironment(), 5.0, &error);
  EXPECT_NE(scene, nullptr) << error;
  for (int i = 0; scene && i < scene->ArmCount(); ++i) {
    if (scene->arm(i).use->model != "kr210_r2700_2") return MotionOf(*scene, i, f);
  }
  ADD_FAILURE() << "no pin arm in " << kV16;
  return {};
}

// Made-up paths of one short move, hole to hole with the straight line
// blocked: a wide way round of seven pieces (as one search found it), a
// close one of three, and the close one again. The close one is kept, the
// first of the two equal ones; a path that cannot be timed is passed over.
TEST(PathTries, TheQuickestOfMadeUpPathsIsKept) {
  const PlanFigures f;
  const ArmMotion m = Kr20Motion(f);
  ASSERT_EQ(m.limits.velocity.size(), 7);
  const Config from{0.2, -1.4, 1.6, 0.0, 0.9, 0.0, 1.0};
  Config to = from;
  to[0] += 0.10;
  to[6] += 0.30;
  auto via = [&](double lift2, double lift3, double turn4, double along) {
    Config c = from;
    for (int j = 0; j < 7; ++j) c[j] += along * (to[j] - from[j]);
    c[1] += lift2;
    c[2] += lift3;
    c[3] += turn4;
    return c;
  };
  const std::vector<Config> wide = {from,
                                    via(-0.3, 0.2, 0.8, 0.1),
                                    via(-0.5, 0.5, 1.6, 0.3),
                                    via(-0.6, 0.6, 2.2, 0.5),
                                    via(-0.5, 0.5, 1.6, 0.7),
                                    via(-0.3, 0.2, 0.8, 0.9),
                                    via(-0.1, 0.05, 0.1, 0.95),
                                    to};
  const std::vector<Config> close = {from, via(-0.12, 0.10, 0.0, 0.3),
                                     via(-0.12, 0.10, 0.0, 0.7), to};
  const double wide_s = MoveDuration(wide, m, f);
  const double close_s = MoveDuration(close, m, f);
  ASSERT_GT(wide_s, 0.0);
  ASSERT_GT(close_s, 0.0);
  EXPECT_LT(close_s, 0.6 * wide_s) << close_s << " s against " << wide_s << " s";
  EXPECT_EQ(QuickestPath({wide}, m, f), 0);
  EXPECT_EQ(QuickestPath({wide, close}, m, f), 1);
  EXPECT_EQ(QuickestPath({wide, close, close}, m, f), 1);
  EXPECT_EQ(QuickestPath({close, wide}, m, f), 0);
  // An empty path cannot be timed.
  EXPECT_EQ(QuickestPath({{}, wide}, m, f), 1);
  EXPECT_EQ(QuickestPath({{}}, m, f), -1);
  EXPECT_EQ(QuickestPath({}, m, f), -1);
}

TEST(PathTries, ThePlanFileNamesItOnlyWhenNotOne) {
  // A committed plan without the figure reads it as 1 and is written again
  // byte for byte.
  const std::string text = ReadFile(Env("PLAN_DIR", "plans") + "/" + kKr20 + ".json");
  std::string error;
  std::optional<Plan> p = PlanFromJson(text, &error);
  ASSERT_TRUE(p.has_value()) << error;
  EXPECT_EQ(text.find("path_tries"), std::string::npos);
  EXPECT_EQ(p->figures.path_tries, 1);
  EXPECT_EQ(PlanToJson(*p), text);
  p->figures.path_tries = 8;
  const std::string tries = PlanToJson(*p);
  EXPECT_NE(tries.find("\"shorten_tries\":200,\"path_tries\":8,\"budgets\""), std::string::npos);
  const std::optional<Plan> back = PlanFromJson(tries, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(back->figures.path_tries, 8);
  EXPECT_EQ(PlanToJson(*back), tries);
}

bool AnyFault(const VerifyReport& r, const std::string& words) {
  for (const std::string& f : r.faults) {
    if (f.find(words) != std::string::npos) return true;
  }
  return false;
}

std::string Faults(const VerifyReport& r) {
  std::string s;
  for (const std::string& f : r.faults) s += "  " + f + "\n";
  return s;
}

// The verifier holds the plan's path_tries to the layout's, on the
// committed one-bay plan of the KR 20 layout (the quickest to check).
TEST(PathTries, TheVerifierHoldsThePlanToTheLayout) {
  std::string error;
  const std::optional<Layout> l = LoadLayout(LayoutPath(kKr20), &error);
  ASSERT_TRUE(l.has_value()) << error;
  const std::optional<Plan> plan =
      PlanFromJson(ReadFile(Env("PLAN_DIR", "plans") + "/" + kKr20 + ".json"), &error);
  ASSERT_TRUE(plan.has_value()) << error;
  const std::string sha = Sha256OfFile(LayoutPath(kKr20));
  VerifyOptions quick;
  quick.schedule = false;
  quick.edges = false;
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const CheckReport report = CheckLayout(*l, paths);
  const VerifyReport ok = VerifyPlanWithReport(*l, paths, *plan, sha, report, quick);
  EXPECT_TRUE(ok.ok) << Faults(ok);
  // A plan that says it kept the quickest of 8 for a layout without the field.
  Plan tried = *plan;
  tried.figures.path_tries = 8;
  const VerifyReport r1 = VerifyPlanWithReport(*l, paths, tried, sha, report, quick);
  EXPECT_FALSE(r1.ok);
  EXPECT_TRUE(AnyFault(r1, "the plan's figure path_tries is 8, not ours, 1")) << Faults(r1);
  // A layout with the field and a plan without it.
  Layout with = *l;
  with.path_tries = 8;
  const VerifyReport r2 = VerifyPlanWithReport(with, paths, *plan, sha, report, quick);
  EXPECT_FALSE(r2.ok);
  EXPECT_TRUE(AnyFault(r2, "the plan's figure path_tries is 1, not ours, 8")) << Faults(r2);
  // Both agree: no fault of the figure (the paths are checked as any).
  const VerifyReport r3 = VerifyPlanWithReport(with, paths, tried, sha, report, quick);
  EXPECT_FALSE(AnyFault(r3, "path_tries")) << Faults(r3);
  EXPECT_TRUE(r3.ok) << Faults(r3);
}

}  // namespace
}  // namespace cell::layout
