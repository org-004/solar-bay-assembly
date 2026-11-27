// The checker's report as a file (src/layout_report_file.h, run 039): the
// report of the smallest one-posture layout, as its check_report rule in
// BUILD.bazel wrote it, read back equals the one CheckLayout computes here,
// field by field, every field of CheckReport (the verifier's and the
// scorecard's among them); a report of another version of the layout, or of
// another layout, is refused, not used; and every layout with
// one_posture_per_module has its file. Compiled into
// //:plan_replay_staged_test, whose data has the files (a cc_test of its own
// would add targets that list googletest).

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_report_file.h"

namespace cell::layout {
namespace {

// The one-posture layout whose report is the quickest to make (R1 of run
// 039: about 125 s, v13's and v14's about 180 s).
constexpr char kSmallest[] = "two_flat_racks_2_pin_arms_balanced";

std::string LayoutDir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr ? dir : "layouts";
}

std::string ReadText(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream s;
  s << in.rdbuf();
  return s.str();
}

// Equal as doubles, NaN equal to NaN.
bool Same(double a, double b) { return (std::isnan(a) && std::isnan(b)) || a == b; }

bool Same(const std::array<double, 6>& a, const std::array<double, 6>& b) {
  for (size_t i = 0; i < 6; ++i) {
    if (!Same(a[i], b[i])) return false;
  }
  return true;
}

bool Same(const std::vector<std::array<double, 6>>& a,
          const std::vector<std::array<double, 6>>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (!Same(a[i], b[i])) return false;
  }
  return true;
}

#define EXPECT_SAME(a, b, where) EXPECT_TRUE(Same((a), (b))) << (where) << ": " #a

// Every field of the two reports, element by element.
void ExpectEqualReports(const CheckReport& got, const CheckReport& want) {
  ASSERT_EQ(got.rows.size(), want.rows.size());
  for (size_t i = 0; i < want.rows.size(); ++i) {
    const CheckRow& g = got.rows[i];
    const CheckRow& w = want.rows[i];
    const std::string at = "row " + std::to_string(i) + " " + w.check + " " + w.item;
    EXPECT_EQ(g.check, w.check) << at;
    EXPECT_EQ(g.item, w.item) << at;
    EXPECT_EQ(g.arm, w.arm) << at;
    EXPECT_EQ(g.tool, w.tool) << at;
    EXPECT_SAME(g.figure, w.figure, at);
    EXPECT_EQ(g.unit, w.unit) << at;
    EXPECT_SAME(g.limit, w.limit, at);
    EXPECT_EQ(g.pass, w.pass) << at;
    EXPECT_EQ(g.detail, w.detail) << at;
  }
  ASSERT_EQ(got.steps.size(), want.steps.size());
  for (size_t i = 0; i < want.steps.size(); ++i) {
    const BayStep& g = got.steps[i];
    const BayStep& w = want.steps[i];
    const std::string at = "step " + std::to_string(i) + " " + w.what;
    EXPECT_EQ(g.number, w.number) << at;
    EXPECT_EQ(g.what, w.what) << at;
    EXPECT_EQ(g.arm, w.arm) << at;
    EXPECT_EQ(g.tool, w.tool) << at;
    EXPECT_EQ(g.part, w.part) << at;
    EXPECT_EQ(g.from, w.from) << at;
    EXPECT_EQ(g.to, w.to) << at;
    EXPECT_SAME(g.time_s, w.time_s, at);
    EXPECT_EQ(g.kind, w.kind) << at;
    EXPECT_EQ(g.index, w.index) << at;
    EXPECT_EQ(g.phase, w.phase) << at;
    EXPECT_EQ(g.tool_after, w.tool_after) << at;
    EXPECT_EQ(g.stage, w.stage) << at;
  }
  EXPECT_SAME(got.bay_time_s, want.bay_time_s, "report");
  EXPECT_EQ(got.time_model, want.time_model);
  EXPECT_SAME(got.clearance_m, want.clearance_m, "report");
  EXPECT_SAME(got.joint_margin_deg, want.joint_margin_deg, "report");
  ASSERT_EQ(got.points.size(), want.points.size());
  for (size_t i = 0; i < want.points.size(); ++i) {
    const SolvedPoint& g = got.points[i];
    const SolvedPoint& w = want.points[i];
    const std::string at = "point " + std::to_string(i) + " " + w.item + " " + w.stage;
    EXPECT_EQ(g.kind, w.kind) << at;
    EXPECT_EQ(g.index, w.index) << at;
    EXPECT_EQ(g.item, w.item) << at;
    EXPECT_EQ(g.arm, w.arm) << at;
    EXPECT_EQ(g.tool, w.tool) << at;
    EXPECT_EQ(g.held_approach, w.held_approach) << at;
    EXPECT_EQ(g.held_work, w.held_work) << at;
    EXPECT_EQ(g.held_retreat, w.held_retreat) << at;
    EXPECT_SAME(g.track, w.track, at);
    EXPECT_SAME(g.work_deg, w.work_deg, at);
    EXPECT_SAME(g.approach_deg, w.approach_deg, at);
    EXPECT_SAME(g.retreat_deg, w.retreat_deg, at);
    EXPECT_EQ(g.work_reached, w.work_reached) << at;
    EXPECT_EQ(g.approach_reached, w.approach_reached) << at;
    EXPECT_EQ(g.retreat_reached, w.retreat_reached) << at;
    EXPECT_SAME(g.approach_leg_deg, w.approach_leg_deg, at);
    EXPECT_SAME(g.retreat_leg_deg, w.retreat_leg_deg, at);
    EXPECT_SAME(g.approach_length_m, w.approach_length_m, at);
    EXPECT_SAME(g.retreat_length_m, w.retreat_length_m, at);
    EXPECT_EQ(g.stage, w.stage) << at;
  }
  ASSERT_EQ(got.tool_changes.size(), want.tool_changes.size());
  for (size_t i = 0; i < want.tool_changes.size(); ++i) {
    const std::string at = "tool change of " + want.tool_changes[i].arm;
    EXPECT_EQ(got.tool_changes[i].arm, want.tool_changes[i].arm) << at;
    EXPECT_SAME(got.tool_changes[i].deg, want.tool_changes[i].deg, at);
    EXPECT_EQ(got.tool_changes[i].reached, want.tool_changes[i].reached) << at;
  }
  ASSERT_EQ(got.phases.size(), want.phases.size());
  for (size_t i = 0; i < want.phases.size(); ++i) {
    EXPECT_EQ(got.phases[i].first, want.phases[i].first);
    EXPECT_SAME(got.phases[i].second, want.phases[i].second, want.phases[i].first);
  }
  ASSERT_EQ(got.postures.size(), want.postures.size());
  for (size_t i = 0; i < want.postures.size(); ++i) {
    const PostureChanges& g = got.postures[i];
    const PostureChanges& w = want.postures[i];
    const std::string at = "postures of " + w.arm;
    EXPECT_EQ(g.arm, w.arm) << at;
    EXPECT_SAME(g.largest_s, w.largest_s, at);
    EXPECT_EQ(g.from, w.from) << at;
    EXPECT_EQ(g.to, w.to) << at;
    EXPECT_EQ(g.above_2s, w.above_2s) << at;
  }
  EXPECT_EQ(got.pass, want.pass);
  EXPECT_EQ(got.error, want.error);
}

std::string PathOf(const std::string& name) { return LayoutDir() + "/" + name + ".json"; }

TEST(CheckReportFile, ReadBackEqualsWhatTheCheckerComputes) {
  std::string why;
  const std::optional<Layout> layout = LoadLayout(PathOf(kSmallest), &why);
  ASSERT_TRUE(layout.has_value()) << why;
  ASSERT_TRUE(layout->one_posture_per_module);
  const std::string sha = Sha256OfFile(PathOf(kSmallest));
  const std::optional<CheckReport> read =
      ReadCheckReportFile(ReportDir() + "/" + kSmallest + ".json", kSmallest, sha, &why);
  ASSERT_TRUE(read.has_value()) << why;
  const CheckReport computed = CheckLayout(*layout, ModelPaths::FromEnvironment());
  ASSERT_TRUE(computed.pass);
  // The report is not trivial: the poses of every point with their legs.
  EXPECT_GT(computed.points.size(), 100u);
  EXPECT_GT(computed.rows.size(), 900u);
  ExpectEqualReports(*read, computed);
  // And what CheckReportOf gives the tests is that file.
  const std::optional<CheckReport> of =
      CheckReportOf(*layout, sha, ModelPaths::FromEnvironment(), ReportDir(), &why);
  ASSERT_TRUE(of.has_value()) << why;
  ExpectEqualReports(*of, computed);
}

TEST(CheckReportFile, ATextRoundTripKeepsEveryNumberAndTheNonFiniteOnes) {
  CheckReport r;
  CheckRow row;
  row.check = "clearance";
  row.figure = 0.1 + 0.2;  // Not a short decimal.
  row.limit = std::numeric_limits<double>::infinity();
  r.rows.push_back(row);
  row.figure = std::nan("");
  row.limit = -std::numeric_limits<double>::infinity();
  r.rows.push_back(row);
  SolvedPoint p;
  p.work_deg = {1.0 / 3.0, -2.0 / 7.0, 1e-300, 5e-324, 179.99999999999997, 0.0};
  p.approach_leg_deg = {p.work_deg, p.work_deg};
  r.points.push_back(p);
  r.phases = {{"tube in", 12.345678901234567}};
  std::string why;
  const std::optional<CheckReport> back =
      CheckReportFromJson(CheckReportToJson(r, "x", "abc"), "x", "abc", &why);
  ASSERT_TRUE(back.has_value()) << why;
  ExpectEqualReports(*back, r);
}

TEST(CheckReportFile, AReportOfAnotherVersionOrLayoutIsRefused) {
  const std::string sha = Sha256OfFile(PathOf(kSmallest));
  const std::string text = ReadText(ReportDir() + "/" + kSmallest + ".json");
  ASSERT_FALSE(text.empty());
  std::string why;
  ASSERT_TRUE(CheckReportFromJson(text, kSmallest, sha, &why).has_value()) << why;
  // The layout file changed since the report was made.
  const std::string other(64, '0');
  why.clear();
  EXPECT_FALSE(CheckReportFromJson(text, kSmallest, other, &why).has_value());
  EXPECT_NE(why.find("another version"), std::string::npos) << why;
  // The file claims another version than the layout as it is.
  std::string tampered = text;
  const size_t at = tampered.find(sha);
  ASSERT_NE(at, std::string::npos);
  tampered.replace(at, sha.size(), other);
  why.clear();
  EXPECT_FALSE(CheckReportFromJson(tampered, kSmallest, sha, &why).has_value());
  EXPECT_NE(why.find("another version"), std::string::npos) << why;
  // Another layout's report under this name.
  why.clear();
  EXPECT_FALSE(CheckReportFromJson(text, "two_flat_racks_2_pin_arms", sha, &why).has_value());
  EXPECT_NE(why.find("not two_flat_racks_2_pin_arms"), std::string::npos) << why;
  // What the tests call refuses it too, and a layout whose file changed gets
  // no report, not a stale one.
  const std::optional<Layout> layout = LoadLayout(PathOf(kSmallest), &why);
  ASSERT_TRUE(layout.has_value()) << why;
  why.clear();
  EXPECT_FALSE(
      CheckReportOf(*layout, other, ModelPaths::FromEnvironment(), ReportDir(), &why).has_value());
  EXPECT_NE(why.find("another version"), std::string::npos) << why;
}

TEST(CheckReportFile, EveryOnePostureLayoutHasItsFile) {
  int one_posture = 0;
  for (const std::string& name : ListLayouts(LayoutDir())) {
    std::string why;
    const std::optional<Layout> layout = LoadLayout(PathOf(name), &why);
    ASSERT_TRUE(layout.has_value()) << name << ": " << why;
    if (!layout->one_posture_per_module) continue;
    ++one_posture;
    const std::optional<CheckReport> r = ReadCheckReportFile(
        ReportDir() + "/" + name + ".json", name, Sha256OfFile(PathOf(name)), &why);
    EXPECT_TRUE(r.has_value()) << name << ": " << why
                               << " (add the layout to ONE_POSTURE_LAYOUTS in BUILD.bazel)";
  }
  EXPECT_GE(one_posture, 3);
}

}  // namespace
}  // namespace cell::layout
