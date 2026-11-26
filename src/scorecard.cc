// The scorecard of a layout (src/scorecard.h). Milestone stub: every
// function is declared and builds; the bodies come with the program.
#include "src/scorecard.h"

#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_rows.h"

namespace cell::layout {

const Measure* FindMeasure(const Scorecard& card, const std::string& name) {
  for (const Measure& m : card.measures) {
    if (m.name == name) return &m;
  }
  return nullptr;
}

std::optional<std::vector<Rate>> LoadRates(const std::string& path,
                                           std::string* error) {
  *error = "not implemented: " + path;
  return std::nullopt;
}

std::vector<std::string> OperationKinds(const StepTime& step) {
  (void)step;
  return {};
}

std::optional<std::vector<OperationCount>> CountOperations(
    const Plan& plan, const std::vector<Rate>& rates, std::string* error) {
  (void)plan;
  (void)rates;
  *error = "not implemented";
  return std::nullopt;
}

std::vector<Count> PartCount(const Layout& layout) {
  (void)layout;
  return {};
}

std::vector<Count> DrivenAxes(const Layout& layout) {
  (void)layout;
  return {};
}

std::vector<MassPart> MassInTransit(const Layout& layout) {
  (void)layout;
  return {};
}

std::vector<MassLeftOut> MassLeftOutOfTransit(const Layout& layout) {
  (void)layout;
  return {};
}

std::optional<Scorecard> ComputeScorecard(
    const Layout& layout, const Plan& plan, const std::vector<Rate>& rates,
    const ModelPaths& paths, const std::string& layout_sha256,
    const std::string& plan_sha256, const std::string& rates_sha256,
    std::string* error) {
  (void)layout;
  (void)plan;
  (void)rates;
  (void)paths;
  (void)layout_sha256;
  (void)plan_sha256;
  (void)rates_sha256;
  *error = "not implemented";
  return std::nullopt;
}

std::string ScorecardToJson(const Scorecard& card) { return card.layout; }

std::optional<Scorecard> ScorecardFromJson(const std::string& text,
                                           std::string* error) {
  (void)text;
  *error = "not implemented";
  return std::nullopt;
}

std::string ScorecardText(const Scorecard& card) { return card.layout; }

std::string ScorecardsTable(const std::vector<Scorecard>& cards) {
  return std::to_string(cards.size());
}

std::optional<std::vector<Version>> LoadVersions(const std::string& path,
                                                 std::string* error) {
  *error = "not implemented: " + path;
  return std::nullopt;
}

}  // namespace cell::layout
