// The scorecard of a layout: every measure the owner steers the cell by,
// each with how it is known, and the gates a layout must pass.
//
// The owner ranks the measures (P0 first) and does not weight them; there is
// no combined score. Each measure says which way is better and how it is
// known: `measured` (a run in physics produced it), `planned` (a committed
// motion plan's figure), `counted` (counted from the layout or plan file by a
// rule written here and in docs/scorecard.md), `modelled` (a model over
// counted operations, such as expected failures) or `none` (not known; the
// basis says why). A figure that is not known is printed as not known.
//
//   P0  cycle_s             lower   planned   the plan's bay_planned_s
//   P0  failures_per_bay    lower   modelled  sum over kinds of operation of
//                                             count in the plan x rate of
//                                             scores/rates.json
//   P0  physics_share       higher  counted   share of cycle_s carried out
//                                             in physics by a committed run;
//                                             0 for every flat rack layout
//   P1  part_count          lower   counted   pieces of equipment of the cell
//   P1  arms                lower   counted
//   P1  driven_axes         lower   counted   arm joints, tracks, driven
//                                             tools and grips
//   P1  placement_mm        lower   none      nothing is held or sensed
//   P2  mass_in_transit_kg  lower   counted   the owner's definition, from
//                                             the transport row's parts
//   P2  input_tolerance     higher  none      nothing is sensed
//   P2  plan_s              lower   none      the plan file holds no time
//
// The gates: every row of check_layout passes (figure: rows failed, limit
// 0); the least clearance over the schedule, from VerifyPlan, is at least
// the layout's margins.clearance_m; mass_in_transit_kg is at most 13,608 kg
// (30,000 lb, the owner's limit for one container). Figures that the checker
// or the planner computes are taken from their code (CheckLayout,
// VerifyPlan, TransportMassParts of src/layout_rows.h), never computed again.
//
// The rules of counting (docs/scorecard.md says them in words):
//
//   part_count   one for each physical instance: each arm; each arm's track;
//                each tool; each tool's stand (a tool that waits off the
//                flange, wait_size not zero); the holder's stands (one per
//                grip) and its grips (one saddle per grip) and its end unit
//                (one); each item of the layout (feeders, lanes, carts),
//                whether it travels or not; each pin magazine that a stock
//                inside the cell holds (stocks whose part is a magazine,
//                `holds` of them). Not counted: parts of the bay (tube,
//                rails, modules, pins, bolts) and what they arrive in
//                (pallets, the rail rack: stocks outside).
//   driven_axes  each arm's joints (six for every arm of src/arm_catalog.h),
//                each track (one), each tool (one: every tool of these
//                layouts drives something), each grip of the holder (one)
//                and its end unit's angle pin (one).
//   mass         every part of TransportMassParts except "stocks inside",
//                which are infed parts (pin magazines with their pins). The
//                items that do not travel, the pallets, the rail rack and
//                the flat rack itself are already outside the row.
//
// Operations (for failures_per_bay): every step of a plan maps to one or
// more kinds by OperationKinds below; a step that maps to none is an error.
//
//   tube in -> tube_in              clamp -> clamp
//   take ... -> grip, arm_move      set ... -> set_down, arm_move
//   bolt ... -> bolt_run, arm_move  locate ... -> locate, arm_move
//   pin ... -> pin_set, arm_move    swage ... -> swage, arm_move
//   tool change -> tool_change, arm_move
//   back to wait -> arm_move        release -> unclamp
//   bay out -> bay_out
//
// The kinds of scores/rates.json that no step maps to (hand_over today) are
// counted 0.
//
// The files:
//
//   scores/rates.json      {"about": "...", "kinds": [{"kind", "what",
//                          "rate", "basis" ("source" or "ours"), "source",
//                          "reasoning"}, ...]}
//   scores/<name>.json     ScorecardToJson: {"layout", "layout_sha256",
//                          "plan_sha256", "rates_sha256", "measures": [{"name",
//                          "rank", "value" (null when not known), "unit",
//                          "better", "known", "basis"}], "gates": [{"name",
//                          "figure", "limit", "unit", "holds" ("<=" or ">="),
//                          "pass", "basis"}], "part_count": [{"kind",
//                          "count"}], "driven_axes": [{"kind", "count"}],
//                          "mass_in_transit": [{"component", "name",
//                          "mass_kg"}], "mass_left_out": [{"component",
//                          "name", "mass_kg", "why"}], "operations":
//                          [{"kind", "count", "rate", "basis",
//                          "expected"}]}. Written by //:score_layout --write.
//   versions/versions.json {"about", "directions": [{"id", "title",
//                          "description", "limits"}], "versions": [{"id",
//                          "date", "commit", "title", "direction", "status",
//                          "changed", "why", "recording", and either
//                          "layout": {"path", "sha256"}, "plan": {"path",
//                          "sha256"}, "scores": path, or "figures": [{"name",
//                          "value", "unit", "known", "source"}]}]}.
//                          LoadVersions reads the fields the tests need.
//
// Figures are rounded where they are made, so that the committed file is
// the same text on every run: seconds to 6 decimals (as the plan holds them),
// distances to 4 (as plan_layout prints them), masses to 0.1 kg, rates and
// expected failures to 6 significant digits.

#ifndef SRC_SCORECARD_H_
#define SRC_SCORECARD_H_

#include <optional>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_rows.h"

namespace cell::layout {

// The owner's limit for what one container carries, 30,000 lb.
inline constexpr double kMassInTransitLimitKg = 13608.0;

struct Measure {
  std::string name;              // "cycle_s"
  std::string rank;              // "P0", "P1", "P2"
  std::optional<double> value;   // Empty when not known.
  std::string unit;              // "s", "kg", "1", ...
  std::string better;            // "lower" or "higher"
  std::string known;             // measured, planned, counted, modelled, none
  std::string basis;             // One phrase.
};

struct Gate {
  std::string name;              // "check_rows_failed", ...
  double figure = 0.0;
  double limit = 0.0;
  std::string unit;
  std::string holds;             // "<=" or ">=": figure holds limit.
  bool pass = false;
  std::string basis;
};

struct Count {
  std::string kind;              // "arm", "track", "tool", ...
  int count = 0;
};

struct MassLeftOut {
  MassPart part;
  std::string why;
};

struct Rate {
  std::string kind, what;
  double rate = 0.0;
  std::string basis;             // "source" or "ours"
  std::string source, reasoning;
};

struct OperationCount {
  std::string kind;
  int count = 0;
  double rate = 0.0;
  std::string basis;             // Of the rate.
  double expected = 0.0;         // count x rate.
};

struct Scorecard {
  std::string layout;            // The layout's name.
  std::string layout_sha256, plan_sha256, rates_sha256;
  std::vector<Measure> measures;     // In the order of the table above.
  std::vector<Gate> gates;           // In the order of the comment above.
  std::vector<Count> part_count;     // The breakdown of part_count.
  std::vector<Count> driven_axes;    // The breakdown of driven_axes.
  std::vector<MassPart> mass_in_transit;
  std::vector<MassLeftOut> mass_left_out;
  std::vector<OperationCount> operations;  // In the order of rates.json.
  bool pass = false;                 // Every gate passes.
};

// A measure by name; nullptr when there is none.
const Measure* FindMeasure(const Scorecard& card, const std::string& name);

// Reads scores/rates.json. Returns nothing and sets *error on any fault
// (a missing field, a rate outside [0, 1], a basis other than source/ours,
// a source missing where basis is source, a kind twice).
std::optional<std::vector<Rate>> LoadRates(const std::string& path,
                                           std::string* error);

// The kinds of operation of one step of a plan, by the rule above, read
// from the step's `what` (the first word, or "tool change", "back to wait",
// "tube in", "bay out"). Empty when the step maps to none, which the caller
// treats as an error.
std::vector<std::string> OperationKinds(const StepTime& step);

// The count of operations of each kind of `rates` in a plan, in the order of
// `rates`. Returns nothing and sets *error when a step maps to no kind or to
// a kind that rates.json does not hold; the error names the step.
std::optional<std::vector<OperationCount>> CountOperations(
    const Plan& plan, const std::vector<Rate>& rates, std::string* error);

// The breakdowns of the counted measures, by the rules above.
std::vector<Count> PartCount(const Layout& layout);
std::vector<Count> DrivenAxes(const Layout& layout);

// The mass in transit by the owner's definition: the transport row's parts
// that count, and those that do not with the reason.
std::vector<MassPart> MassInTransit(const Layout& layout);
std::vector<MassLeftOut> MassLeftOutOfTransit(const Layout& layout);

// Computes the scorecard of a layout and its committed plan: runs
// CheckLayout and VerifyPlan (about 16 s for a flat rack layout). The
// sha256s are of the files as they are; `layout_sha256` is also what the
// plan is verified against. Returns nothing and sets *error when the plan
// does not map, a rate is missing or the model cannot be built.
std::optional<Scorecard> ComputeScorecard(
    const Layout& layout, const Plan& plan, const std::vector<Rate>& rates,
    const ModelPaths& paths, const std::string& layout_sha256,
    const std::string& plan_sha256, const std::string& rates_sha256,
    std::string* error);

// scores/<name>.json and --json: the same text, 2-space indent, a final
// newline. ScorecardFromJson reads it back.
std::string ScorecardToJson(const Scorecard& card);
std::optional<Scorecard> ScorecardFromJson(const std::string& text,
                                           std::string* error);

// The text of one scorecard: the measures' table (rank, name, value, unit,
// better, known, basis), the gates' table (name, figure, limit, pass/FAIL),
// then the breakdowns of part_count, driven_axes, mass_in_transit_kg (and
// what is left out) and the operations by kind.
std::string ScorecardText(const Scorecard& card);

// --all: one row per layout, one column per measure, the gates as one
// column ("pass" or the names of the gates that fail).
std::string ScorecardsTable(const std::vector<Scorecard>& cards);

// The fields of versions/versions.json that the tests check.
struct VersionFile {
  std::string path, sha256;
};
struct Version {
  std::string id, date, commit, title, direction, status, recording;
  std::optional<VersionFile> layout, plan;
  std::string scores;            // Empty for a version without a scorecard.
};
std::optional<std::vector<Version>> LoadVersions(const std::string& path,
                                                 std::string* error);

}  // namespace cell::layout

#endif  // SRC_SCORECARD_H_
