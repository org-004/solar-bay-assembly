// The scorecard (src/scorecard.h): every committed scores/<name>.json is what
// score_layout computes for the layout, its plan and the rates as they are,
// every step of every plan maps to a kind of operation, and the counting
// rules give the figures they should on the committed layouts.

#include "src/scorecard.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_report_file.h"
#include "src/physics_record.h"

namespace cell::layout {
namespace {

std::string Dir(const char* variable, const char* fallback) {
  const char* dir = std::getenv(variable);
  return dir != nullptr && dir[0] != '\0' ? dir : fallback;
}

std::string ReadFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
}

struct Inputs {
  Layout layout;
  Plan plan;
  std::vector<Rate> rates;
  std::string layout_sha, plan_sha, rates_sha;
  // physics/<name>.json when the layout has one (none is committed yet).
  std::optional<PhysicsRecord> physics;
  std::string physics_sha;
};

std::optional<Inputs> Read(const std::string& name) {
  Inputs in;
  std::string error;
  const std::string layout_path = Dir("LAYOUT_DIR", "layouts") + "/" + name + ".json";
  const std::string plan_path = Dir("PLAN_DIR", "plans") + "/" + name + ".json";
  const std::string rates_path = Dir("SCORES_DIR", "scores") + "/rates.json";
  const auto layout = LoadLayout(layout_path, &error);
  EXPECT_TRUE(layout.has_value()) << error;
  const auto plan = PlanFromJson(ReadFile(plan_path), &error);
  EXPECT_TRUE(plan.has_value()) << plan_path << ": " << error;
  const auto rates = LoadRates(rates_path, &error);
  EXPECT_TRUE(rates.has_value()) << error;
  if (!layout || !plan || !rates) return std::nullopt;
  in.layout = *layout;
  in.plan = *plan;
  in.rates = *rates;
  in.layout_sha = Sha256OfFile(layout_path);
  in.plan_sha = Sha256OfFile(plan_path);
  in.rates_sha = Sha256OfFile(rates_path);
  const std::string physics_path =
      Dir("PHYSICS_DIR", "physics") + "/" + name + ".json";
  const std::string physics_text = ReadFile(physics_path);
  if (!physics_text.empty()) {
    in.physics = PhysicsRecordFromJson(physics_text, &error);
    EXPECT_TRUE(in.physics.has_value()) << physics_path << ": " << error;
    in.physics_sha = Sha256OfFile(physics_path);
  }
  return in;
}

std::optional<Scorecard> ComputeUncached(const std::string& name) {
  const std::optional<Inputs> in = Read(name);
  if (!in) return std::nullopt;
  std::string error;
  // A one-posture layout's checker report is the file its check_report rule
  // made once (src/layout_report_file.h, run 039); the others' are computed.
  const std::optional<CheckReport> report = CheckReportOf(
      in->layout, in->layout_sha, ModelPaths::FromEnvironment(), ReportDir(), &error);
  EXPECT_TRUE(report.has_value()) << name << ": " << error;
  if (!report) return std::nullopt;
  auto card = ComputeScorecardWithReport(
      in->layout, in->plan, in->rates, ModelPaths::FromEnvironment(),
      in->layout_sha, in->plan_sha, in->rates_sha,
      in->physics ? &*in->physics : nullptr, *report, &error);
  EXPECT_TRUE(card.has_value()) << name << ": " << error;
  if (card) card->physics_sha256 = in->physics_sha;
  return card;
}

// Computing takes about 12 s, so each layout's card is computed once.
std::optional<Scorecard> Compute(const std::string& name) {
  static std::map<std::string, std::optional<Scorecard>> cache;
  const auto found = cache.find(name);
  if (found != cache.end()) return found->second;
  return cache[name] = ComputeUncached(name);
}

// The committed plans that have a scorecard. A card is of three bays; a plan of
// one bay (run 034's all-KUKA 40 ft layout, whose three bays and card are a
// later spec's) is replayed by tests/plan_replay_test.cc and has none yet.
std::vector<std::string> PlanNames() {
  static const std::set<std::string> kOneBay = {
      "flat_rack_1_kr20_pin_arm_ordered", "flat_rack_3_kr20_pin_arms_shared_ordered_fast_paths"};
  std::vector<std::string> out;
  for (const std::string& name : ListLayouts(Dir("PLAN_DIR", "plans"))) {
    if (kOneBay.count(name) == 0) out.push_back(name);
  }
  return out;
}

StepTime Step(const std::string& what) {
  StepTime s;
  s.what = what;
  return s;
}

// One test per committed plan, so that the test's shards share the cards
// out: the two-rack layouts with one posture per module take minutes each
// (their checker reports). The parameter is the plan's index in PlanNames()
// (::testing::ValuesIn over strings makes GCC 14.2 crash).
class CommittedFile : public ::testing::TestWithParam<int> {};

TEST_P(CommittedFile, IsWhatTheProgramComputes) {
  const std::vector<std::string> names = PlanNames();
  ASSERT_LT(static_cast<size_t>(GetParam()), names.size());
  const std::string& name = names[GetParam()];
  const std::optional<Scorecard> card = Compute(name);
  ASSERT_TRUE(card.has_value());
  const std::string committed =
      ReadFile(Dir("SCORES_DIR", "scores") + "/" + name + ".json");
  std::string error;
  const std::optional<Scorecard> recorded = ScorecardFromJson(committed, &error);
  ASSERT_TRUE(recorded.has_value()) << "scores/" << name << ".json: " << error;
  EXPECT_EQ(committed, ScorecardToJson(*card))
      << "scores/" << name
      << ".json is not what score_layout computes for layouts/" << name
      << ".json and plans/" << name << ".json as they are (layout sha256 now "
      << card->layout_sha256 << ", recorded " << recorded->layout_sha256
      << "; plan sha256 now " << card->plan_sha256 << ", recorded "
      << recorded->plan_sha256 << "; rates sha256 now " << card->rates_sha256
      << ", recorded " << recorded->rates_sha256
      << "). Run: bazel run --config=opt //:score_layout -- " << name
      << " --write";
}

INSTANTIATE_TEST_SUITE_P(EveryPlan, CommittedFile,
                         ::testing::Range(0, static_cast<int>(PlanNames().size())),
                         [](const ::testing::TestParamInfo<int>& i) {
                           return PlanNames()[i.param];
                         });

TEST(Scorecard, ThereAreCommittedPlans) { EXPECT_FALSE(PlanNames().empty()); }

TEST(Scorecard, EveryStepOfEveryPlanMaps) {
  for (const std::string& name : PlanNames()) {
    const std::optional<Inputs> in = Read(name);
    ASSERT_TRUE(in.has_value());
    std::string error;
    const auto counts = CountOperations(in->plan, in->rates, &error);
    ASSERT_TRUE(counts.has_value()) << name << ": " << error;
    int total = 0;
    for (const OperationCount& c : *counts) total += c.count;
    int steps = 0;  // One bay's: bay 0's.
    for (const StepTime& t : in->plan.steps) steps += t.bay == 0 ? 1 : 0;
    EXPECT_GE(total, steps) << name;
    // Every step of every bay maps, though only bay 0's are counted.
    for (const StepTime& t : in->plan.steps) {
      EXPECT_FALSE(OperationKinds(t).empty()) << name << " step " << t.number;
    }
  }
}

// cycle_s, one_bay_s and interval_s of every committed card are the plan's
// CycleFiguresOf, rounded as the card rounds them.
TEST(Scorecard, TheCycleFiguresAreThePlans) {
  for (const std::string& name : PlanNames()) {
    const std::optional<Inputs> in = Read(name);
    ASSERT_TRUE(in.has_value()) << name;
    std::string error;
    const std::optional<Scorecard> card = ScorecardFromJson(
        ReadFile(Dir("SCORES_DIR", "scores") + "/" + name + ".json"), &error);
    ASSERT_TRUE(card.has_value()) << name << ": " << error;
    const CycleFigures cycle = CycleFiguresOf(in->plan);
    ASSERT_EQ(cycle.bays, 3) << name;
    auto rounded = [](double v) { return std::round(v * 1e6) / 1e6; };
    const Measure* m = FindMeasure(*card, "cycle_s");
    ASSERT_NE(m, nullptr) << name;
    EXPECT_EQ(m->value, rounded(cycle.cycle_s)) << name;
    EXPECT_EQ(m->known, "planned");
    ASSERT_EQ(card->figures.size(), 2u) << name;
    EXPECT_EQ(card->figures[0].name, "one_bay_s");
    EXPECT_EQ(card->figures[0].value, rounded(cycle.one_bay_s)) << name;
    EXPECT_EQ(card->figures[0].value, rounded(in->plan.bay_planned_s)) << name;
    EXPECT_EQ(card->figures[1].name, "interval_s");
    EXPECT_EQ(card->figures[1].value, rounded(cycle.interval_s)) << name;
  }
}

// failures_per_bay counts one bay: bay 0's steps, one tube in and one bay out.
// A staged layout's transfer of the tube counts as one more unclamp, tube_in
// and clamp each.
TEST(Scorecard, TheOperationsAreOneBays) {
  for (const std::string& name : PlanNames()) {
    const std::optional<Inputs> in = Read(name);
    ASSERT_TRUE(in.has_value()) << name;
    int transfers = 0;
    for (const StepTime& step : in->plan.steps) {
      if (step.bay == 0 && step.what.rfind("move the tube to stage ", 0) == 0) {
        ++transfers;
      }
    }
    std::string error;
    const auto counts = CountOperations(in->plan, in->rates, &error);
    ASSERT_TRUE(counts.has_value()) << name << ": " << error;
    for (const OperationCount& c : *counts) {
      if (c.kind == "bay_out") EXPECT_EQ(c.count, 1) << name << " " << c.kind;
      if (c.kind == "tube_in" || c.kind == "clamp" || c.kind == "unclamp") {
        EXPECT_EQ(c.count, 1 + transfers) << name << " " << c.kind;
      }
      if (c.kind == "locate") EXPECT_EQ(c.count, 36) << name;
    }
  }
}

TEST(Scorecard, OperationKindsOfSteps) {
  using Kinds = std::vector<std::string>;
  EXPECT_EQ(OperationKinds(Step("tube in")), Kinds({"tube_in"}));
  EXPECT_EQ(OperationKinds(Step("clamp")), Kinds({"clamp"}));
  EXPECT_EQ(OperationKinds(Step("take rail 3")), Kinds({"grip", "arm_move"}));
  EXPECT_EQ(OperationKinds(Step("take a pin magazine")),
            Kinds({"grip", "arm_move"}));
  EXPECT_EQ(OperationKinds(Step("set module 4")),
            Kinds({"set_down", "arm_move"}));
  EXPECT_EQ(OperationKinds(Step("pin 5/2")), Kinds({"pin_set", "arm_move"}));
  EXPECT_EQ(OperationKinds(Step("tool change")),
            Kinds({"tool_change", "arm_move"}));
  EXPECT_EQ(OperationKinds(Step("back to wait")), Kinds({"arm_move"}));
  EXPECT_EQ(OperationKinds(Step("release")), Kinds({"unclamp"}));
  EXPECT_EQ(OperationKinds(Step("bay out")), Kinds({"bay_out"}));
  EXPECT_EQ(OperationKinds(Step("move the tube to stage 2")),
            Kinds({"unclamp", "tube_in", "clamp"}));
  EXPECT_TRUE(OperationKinds(Step("move the tube")).empty());
  EXPECT_TRUE(OperationKinds(Step("juggle 3")).empty());
  EXPECT_TRUE(OperationKinds(Step("")).empty());
}

TEST(Scorecard, AnUnknownStepIsAnErrorThatNamesIt) {
  const std::optional<Inputs> in = Read("flat_rack_3_pin_arms");
  ASSERT_TRUE(in.has_value());
  Plan plan = in->plan;
  StepTime odd = Step("juggle 3");
  odd.number = 777;
  plan.steps.push_back(odd);
  std::string error;
  EXPECT_FALSE(CountOperations(plan, in->rates, &error).has_value());
  EXPECT_NE(error.find("777"), std::string::npos) << error;
  EXPECT_NE(error.find("juggle 3"), std::string::npos) << error;
}

TEST(Scorecard, TheChosenLayoutsCard) {
  const std::optional<Inputs> in = Read("flat_rack_3_pin_arms");
  ASSERT_TRUE(in.has_value());
  const std::optional<Scorecard> card = Compute("flat_rack_3_pin_arms");
  ASSERT_TRUE(card.has_value());
  auto value = [&](const std::string& name) {
    const Measure* m = FindMeasure(*card, name);
    EXPECT_NE(m, nullptr) << name;
    return m != nullptr ? m->value : std::nullopt;
  };
  ASSERT_EQ(card->measures.size(), 10u);
  EXPECT_EQ(value("arms"), 4.0);
  // One bay is v09's bay as it was planned before the plans held three.
  ASSERT_EQ(card->figures.size(), 2u);
  EXPECT_EQ(card->figures[0].name, "one_bay_s");
  EXPECT_EQ(card->figures[0].value, 644.284416);
  EXPECT_EQ(card->figures[1].name, "interval_s");
  EXPECT_LT(value("cycle_s"), 644.284416);
  EXPECT_EQ(value("physics_share"), 0.0);
  EXPECT_FALSE(value("placement_mm").has_value());
  EXPECT_FALSE(value("input_tolerance").has_value());
  EXPECT_FALSE(value("plan_s").has_value());
  ASSERT_EQ(card->gates.size(), 3u);
  for (const Gate& g : card->gates) {
    EXPECT_TRUE(g.pass) << g.name << ": " << g.basis;
  }
  EXPECT_TRUE(card->pass);
  const std::optional<double> mass = value("mass_in_transit_kg");
  ASSERT_TRUE(mass.has_value());
  EXPECT_LE(*mass, 3429.3);
  EXPECT_NEAR(*mass, 3426.3, 0.05);
  double sum = 0.0;
  for (const MassPart& p : card->mass_in_transit) {
    sum += p.mass_kg;
    // The names of the arms ("module_arm", "pin_arm_a") are not bay parts.
    std::string name = p.name;
    for (const ArmUse& arm : in->layout.arms) {
      const size_t at = name.find(arm.name);
      if (at != std::string::npos) name.erase(at, arm.name.size());
    }
    for (const char* word :
         {"module", "rail", "tube", "bolt", "pallet", "base"}) {
      EXPECT_EQ(p.component.find(word), std::string::npos) << p.component;
      EXPECT_EQ(name.find(word), std::string::npos) << p.name;
    }
    EXPECT_NE(p.component, "stocks inside");
  }
  EXPECT_NEAR(sum, *mass, 0.05);
  ASSERT_EQ(card->mass_left_out.size(), 3u);
  for (const MassLeftOut& p : card->mass_left_out) {
    EXPECT_EQ(p.part.component, "stocks inside");
    EXPECT_FALSE(p.why.empty());
  }
}

TEST(Scorecard, CountingRulesOnTheChosenLayout) {
  const std::optional<Inputs> in = Read("flat_rack_3_pin_arms");
  ASSERT_TRUE(in.has_value());
  auto count = [](const std::vector<Count>& counts, const std::string& kind) {
    for (const Count& c : counts) {
      if (c.kind == kind) return c.count;
    }
    return -1;
  };
  const std::vector<Count> parts = PartCount(in->layout);
  EXPECT_EQ(count(parts, "arm"), 4);
  EXPECT_EQ(count(parts, "holder_stand"), count(parts, "holder_grip"));
  EXPECT_EQ(count(parts, "holder_end_unit"), 1);
  EXPECT_EQ(count(parts, "item"), 3);
  EXPECT_EQ(count(parts, "pin_magazine"), 6);
  const std::vector<Count> axes = DrivenAxes(in->layout);
  EXPECT_EQ(count(axes, "arm_joint"), 24);
  EXPECT_EQ(count(axes, "holder_angle_pin"), 1);
}

TEST(Scorecard, FewerArmsLessEverything) {
  const std::optional<Scorecard> two = Compute("flat_rack_2_pin_arms");
  const std::optional<Scorecard> three = Compute("flat_rack_3_pin_arms");
  ASSERT_TRUE(two.has_value());
  ASSERT_TRUE(three.has_value());
  auto value = [](const Scorecard& card, const char* name) {
    const Measure* m = FindMeasure(card, name);
    EXPECT_NE(m, nullptr) << card.layout << " has no measure " << name;
    EXPECT_TRUE(m != nullptr && m->value.has_value())
        << card.layout << ": " << name << " is not known";
    return m != nullptr ? m->value.value_or(0.0) : 0.0;
  };
  for (const char* name : {"part_count", "arms", "mass_in_transit_kg"}) {
    EXPECT_LT(value(*two, name), value(*three, name)) << name;
  }
  EXPECT_GT(value(*two, "cycle_s"), value(*three, "cycle_s"));
}

TEST(Scorecard, JsonRoundTrips) {
  const std::optional<Scorecard> card = Compute("flat_rack_3_pin_arms");
  ASSERT_TRUE(card.has_value());
  const std::string text = ScorecardToJson(*card);
  std::string error;
  const std::optional<Scorecard> back = ScorecardFromJson(text, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(ScorecardToJson(*back), text);
  EXPECT_FALSE(ScorecardFromJson("{}", &error).has_value());
}

TEST(Scorecard, RatesAreChecked) {
  std::string error;
  EXPECT_TRUE(LoadRates(Dir("SCORES_DIR", "scores") + "/rates.json", &error)
                  .has_value())
      << error;
  EXPECT_FALSE(LoadRates("no/such/rates.json", &error).has_value());
}

}  // namespace
}  // namespace cell::layout
