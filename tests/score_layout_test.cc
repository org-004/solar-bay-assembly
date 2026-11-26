// The program score_layout (src/score_layout.cc, whose work is
// RunScoreLayout of src/scorecard.h): its exit codes, the rows it prints for
// a layout, --all and --json. It runs in the test's own process with the
// test's environment (LAYOUT_DIR, PLAN_DIR, SCORES_DIR from BUILD.bazel), so
// it reads the committed scorecards; nothing is written.

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/scorecard.h"

namespace cell::layout {
namespace {

struct Result {
  int exit_code = -1;
  std::string out;
};

Result Score(const std::vector<std::string>& args) {
  Result run;
  run.exit_code = RunScoreLayout(args, &run.out);
  return run;
}

// The first line of the output with `name` as a word of it.
std::string RowOf(const std::string& out, const std::string& name) {
  std::size_t at = 0;
  while (at < out.size()) {
    std::size_t end = out.find('\n', at);
    if (end == std::string::npos) end = out.size();
    const std::string line = out.substr(at, end - at);
    if (line.compare(0, name.size() + 1, name + " ") == 0 ||
        line.find(" " + name + " ") != std::string::npos) {
      return line;
    }
    at = end + 1;
  }
  return "";
}

TEST(ScoreLayout, PrintsEveryMeasureAndGateOfTheChosenLayout) {
  const Result run = Score({"flat_rack_3_pin_arms"});
  EXPECT_EQ(run.exit_code, 0) << run.out;
  for (const char* name :
       {"cycle_s", "failures_per_bay", "physics_share", "part_count", "arms",
        "driven_axes", "placement_mm", "mass_in_transit_kg", "input_tolerance",
        "plan_s"}) {
    EXPECT_FALSE(RowOf(run.out, name).empty()) << name << " in\n" << run.out;
  }
  for (const char* name :
       {"check_rows_failed", "least_clearance_m", "mass_in_transit_kg"}) {
    EXPECT_NE(run.out.find(std::string("\n") + name), std::string::npos) << name;
  }
  EXPECT_NE(run.out.find(" pass "), std::string::npos);
  EXPECT_EQ(run.out.find("FAIL"), std::string::npos) << run.out;
}

TEST(ScoreLayout, NoPlanNoScorecard) {
  EXPECT_EQ(Score({"flat_rack_1_pin_arm"}).exit_code, 2);
  EXPECT_EQ(Score({"no_such_layout"}).exit_code, 2);
}

TEST(ScoreLayout, AllListsEveryScoredLayout) {
  const Result run = Score({"--all"});
  EXPECT_EQ(run.exit_code, 0) << run.out;
  EXPECT_FALSE(RowOf(run.out, "flat_rack_2_pin_arms").empty()) << run.out;
  EXPECT_FALSE(RowOf(run.out, "flat_rack_3_pin_arms").empty()) << run.out;
}

TEST(ScoreLayout, JsonReadsBack) {
  const Result run = Score({"flat_rack_3_pin_arms", "--json"});
  EXPECT_EQ(run.exit_code, 0);
  std::string error;
  const std::optional<Scorecard> card = ScorecardFromJson(run.out, &error);
  ASSERT_TRUE(card.has_value()) << error;
  EXPECT_EQ(card->layout, "flat_rack_3_pin_arms");
  EXPECT_EQ(card->measures.size(), 10u);
}

}  // namespace
}  // namespace cell::layout
