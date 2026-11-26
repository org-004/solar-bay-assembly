// The record of a run in physics (src/physics_record.h): its text and the
// rule by which a step counts, what is checked against the committed plan,
// the scorecard's physics_share with a record, without one and with one that
// does not agree, and every record committed under physics/ (none yet).

#include "src/physics_record.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_bays.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/scorecard.h"

namespace cell::layout {
namespace {

const char kName[] = "flat_rack_3_pin_arms_ordered";

std::string Dir(const char* variable, const char* fallback) {
  const char* dir = std::getenv(variable);
  return dir != nullptr && dir[0] != '\0' ? dir : fallback;
}

std::string ReadFile(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  return std::string((std::istreambuf_iterator<char>(file)),
                     std::istreambuf_iterator<char>());
}

double Round6(double v) {
  const double r = std::round(v * 1e6) / 1e6;
  return r == 0.0 ? 0.0 : r;
}

std::string PlanPath(const std::string& name) {
  return Dir("PLAN_DIR", "plans") + "/" + name + ".json";
}

Plan LoadPlan(const std::string& name) {
  std::string error;
  const std::optional<Plan> plan = PlanFromJson(ReadFile(PlanPath(name)), &error);
  EXPECT_TRUE(plan.has_value()) << name << ": " << error;
  return plan.value_or(Plan());
}

const StepTime* FindStep(const Plan& plan, int number) {
  for (const StepTime& s : plan.steps) {
    if (s.number == number) return &s;
  }
  return nullptr;
}

PhysicsStep StepOf(const Plan& plan, int number, double clearance_m) {
  const StepTime* t = FindStep(plan, number);
  EXPECT_NE(t, nullptr) << "step " << number;
  PhysicsStep s;
  if (t == nullptr) return s;
  s.number = number;
  s.what = t->what;
  s.arm = t->arm;
  s.start_s = Round6(t->start);
  s.end_s = Round6(t->end);
  s.planned_s = Round6(t->end - t->start);
  s.max_joint_error_mrad = 1.2346;
  s.max_frame_error_mm = 2.5;
  s.max_frame_error_deg = 0.0123;
  s.least_clearance_m = clearance_m;
  s.least_clearance_pair = "gripper to rails";
  s.least_clearance_t = Round6(t->start + 0.5);
  s.plan_least_clearance_m = 0.021;
  s.counted = true;
  return s;
}

// A record of steps 63 and 64 that agrees with the committed plan.
PhysicsRecord Fixture(const Plan& plan, const std::string& plan_sha) {
  PhysicsRecord r;
  r.layout = kName;
  r.layout_sha256 = "ab12";
  r.plan_sha256 = plan_sha;
  r.command = "bazel run //:run_physics -- test";
  r.module = 4;
  r.take = 3;
  r.clearance_m = 0.015;
  r.gate_offset_mm = 3.0;
  r.gate_yaw_deg = 0.15;
  r.steps = {StepOf(plan, 63, 0.0212), StepOf(plan, 64, 0.0151)};
  r.set_down = {1.2346, -0.5, 1.1, 0.0123, 0.4};
  r.counted_s = Round6(r.steps[0].planned_s + r.steps[1].planned_s);
  return r;
}

bool Mentions(const std::vector<std::string>& faults, const std::string& text) {
  for (const std::string& f : faults) {
    if (f.find(text) != std::string::npos) return true;
  }
  return false;
}

TEST(PhysicsRecord, TheTextRoundTripsExactlyAndIsStable) {
  const Plan plan = LoadPlan(kName);
  const PhysicsRecord record = Fixture(plan, Sha256OfFile(PlanPath(kName)));
  const std::string text = PhysicsRecordToJson(record);
  EXPECT_EQ(text, PhysicsRecordToJson(record));
  EXPECT_EQ(text.back(), '\n');
  EXPECT_NE(text.find("\n  \"layout\": "), std::string::npos);
  std::string error;
  const std::optional<PhysicsRecord> back = PhysicsRecordFromJson(text, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(PhysicsRecordToJson(*back), text);
  EXPECT_EQ(back->steps.size(), 2u);
  EXPECT_EQ(back->steps[1].end_s, record.steps[1].end_s);
  EXPECT_EQ(back->steps[0].least_clearance_pair, "gripper to rails");
  EXPECT_EQ(back->set_down.offset_mm, 1.2346);
  EXPECT_EQ(back->counted_s, record.counted_s);
  EXPECT_TRUE(back->steps[0].counted);
}

TEST(PhysicsRecord, AMissingFieldIsAnError) {
  const Plan plan = LoadPlan(kName);
  std::string text = PhysicsRecordToJson(Fixture(plan, "x"));
  const size_t at = text.find("\"counted_s\"");
  ASSERT_NE(at, std::string::npos);
  text.replace(at, 11, "\"counted_x\"");
  std::string error;
  EXPECT_FALSE(PhysicsRecordFromJson(text, &error).has_value());
  EXPECT_FALSE(error.empty());
  error.clear();
  EXPECT_FALSE(PhysicsRecordFromJson("[]", &error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST(PhysicsRecord, TheRuleOfWhatCounts) {
  const Plan plan = LoadPlan(kName);
  PhysicsRecord r = Fixture(plan, "x");
  PhysicsStep step = r.steps[0];
  step.least_clearance_m = 0.015;
  EXPECT_TRUE(StepCounts(r, step));
  step.least_clearance_m = 0.014999;
  EXPECT_FALSE(StepCounts(r, step));
  step.least_clearance_m = 0.02;
  r.set_down.offset_mm = 3.0;
  EXPECT_TRUE(StepCounts(r, step));
  r.set_down.offset_mm = 3.0001;
  EXPECT_FALSE(StepCounts(r, step));
  r.set_down.offset_mm = 1.0;
  r.set_down.yaw_deg = -0.15;
  EXPECT_TRUE(StepCounts(r, step));
  r.set_down.yaw_deg = 0.1501;
  EXPECT_FALSE(StepCounts(r, step));
  r.set_down.yaw_deg = -0.1501;
  EXPECT_FALSE(StepCounts(r, step));
  r.set_down.yaw_deg = 0.0;
  EXPECT_DOUBLE_EQ(CountedSeconds(r), r.steps[0].planned_s + r.steps[1].planned_s);
  r.steps[1].least_clearance_m = 0.001;
  r.steps[1].counted = true;  // The stored flag is not what counts.
  EXPECT_DOUBLE_EQ(CountedSeconds(r), r.steps[0].planned_s);
}

TEST(PhysicsRecord, TheCheckAgainstThePlan) {
  const Plan plan = LoadPlan(kName);
  const std::string sha = Sha256OfFile(PlanPath(kName));
  ASSERT_EQ(sha.size(), 64u);
  const PhysicsRecord good = Fixture(plan, sha);
  EXPECT_TRUE(CheckPhysicsRecord(good, plan, sha).empty());

  PhysicsRecord r = good;
  r.steps[0].number = 99999;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "99999"));

  r = good;
  r.steps[1].planned_s += 0.001;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "planned_s"));

  r = good;
  r.steps[0].arm += "x";
  EXPECT_FALSE(CheckPhysicsRecord(r, plan, sha).empty());
  r = good;
  r.steps[0].what += "x";
  EXPECT_FALSE(CheckPhysicsRecord(r, plan, sha).empty());
  r = good;
  r.steps[0].start_s += 0.01;
  EXPECT_FALSE(CheckPhysicsRecord(r, plan, sha).empty());

  r = good;
  r.plan_sha256 = "00" + sha.substr(2);
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "plan_sha256"));

  r = good;
  r.steps[0].counted = false;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "counted"));

  r = good;
  r.counted_s += 1.0;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "counted_s"));
}

// The scorecard takes about 16 s a call, so three calls serve every case.
TEST(PhysicsRecord, ThePhysicsShareOfTheScorecard) {
  std::string error;
  const std::optional<Layout> layout = LoadLayout(
      Dir("LAYOUT_DIR", "layouts") + "/" + kName + ".json", &error);
  ASSERT_TRUE(layout.has_value()) << error;
  const Plan plan = LoadPlan(kName);
  const std::string layout_sha =
      Sha256OfFile(Dir("LAYOUT_DIR", "layouts") + "/" + kName + ".json");
  const std::string plan_sha = Sha256OfFile(PlanPath(kName));
  const std::string rates_path = Dir("SCORES_DIR", "scores") + "/rates.json";
  const std::optional<std::vector<Rate>> rates = LoadRates(rates_path, &error);
  ASSERT_TRUE(rates.has_value()) << error;
  const std::string rates_sha = Sha256OfFile(rates_path);
  const ModelPaths paths = ModelPaths::FromEnvironment();

  const PhysicsRecord record = Fixture(plan, plan_sha);
  PhysicsRecord wrong = record;
  wrong.plan_sha256 = "00" + plan_sha.substr(2);

  const auto with = ComputeScorecard(*layout, plan, *rates, paths, layout_sha,
                                     plan_sha, rates_sha, &record, &error);
  ASSERT_TRUE(with.has_value()) << error;
  const auto without = ComputeScorecard(*layout, plan, *rates, paths,
                                        layout_sha, plan_sha, rates_sha,
                                        nullptr, &error);
  ASSERT_TRUE(without.has_value()) << error;
  const auto bad = ComputeScorecard(*layout, plan, *rates, paths, layout_sha,
                                    plan_sha, rates_sha, &wrong, &error);
  ASSERT_TRUE(bad.has_value()) << error;

  const Measure* m = FindMeasure(*with, "physics_share");
  ASSERT_NE(m, nullptr);
  ASSERT_TRUE(m->value.has_value());
  // The share of one bay (R5): over one_bay_s, not the three bays' cycle_s.
  EXPECT_EQ(*m->value, Round6(CountedSeconds(record) / CycleFiguresOf(plan).one_bay_s));
  EXPECT_GT(*m->value, 0.0);
  EXPECT_NE(m->basis.find("of one bay's"), std::string::npos) << m->basis;
  EXPECT_EQ(m->known, "measured");
  EXPECT_NE(m->basis.find(std::string("physics/") + kName + ".json"),
            std::string::npos)
      << m->basis;
  EXPECT_NE(m->basis.find("steps 63, 64 of module 4"), std::string::npos)
      << m->basis;

  const Measure* none = FindMeasure(*without, "physics_share");
  ASSERT_NE(none, nullptr);
  EXPECT_EQ(none->value, 0.0);
  EXPECT_EQ(none->known, "counted");
  EXPECT_EQ(none->basis,
            "no committed, repeatable run in physics of this layout's arms "
            "and parts");

  const Measure* off = FindMeasure(*bad, "physics_share");
  ASSERT_NE(off, nullptr);
  EXPECT_EQ(off->value, 0.0);
  EXPECT_EQ(off->known, "counted");
  EXPECT_NE(off->basis.find("does not agree with the committed plan"),
            std::string::npos)
      << off->basis;

  ASSERT_EQ(with->measures.size(), without->measures.size());
  for (size_t i = 0; i < with->measures.size(); ++i) {
    if (with->measures[i].name == "physics_share") continue;
    EXPECT_EQ(with->measures[i].value, without->measures[i].value)
        << with->measures[i].name;
    EXPECT_EQ(with->measures[i].basis, without->measures[i].basis)
        << with->measures[i].name;
    EXPECT_EQ(with->measures[i].known, without->measures[i].known);
  }
  EXPECT_EQ(with->gates.size(), without->gates.size());
  EXPECT_EQ(with->operations.size(), without->operations.size());
  // The file's text has no physics_sha256 unless the caller sets one.
  EXPECT_EQ(ScorecardToJson(*without).find("physics_sha256"),
            std::string::npos);
}

TEST(PhysicsRecord, EveryCommittedRecordAgreesWithItsPlanAndItsCard) {
  const std::string dir = Dir("PHYSICS_DIR", "physics");
  std::error_code ec;
  std::filesystem::directory_iterator it(dir, ec);
  if (ec) return;  // No physics directory: no record committed.
  for (const auto& entry : it) {
    if (entry.path().extension() != ".json") continue;
    const std::string name = entry.path().stem().string();
    std::string error;
    const std::optional<PhysicsRecord> record =
        PhysicsRecordFromJson(ReadFile(entry.path().string()), &error);
    ASSERT_TRUE(record.has_value()) << entry.path() << ": " << error;
    EXPECT_EQ(record->layout, name);
    const Plan plan = LoadPlan(name);
    const std::vector<std::string> faults =
        CheckPhysicsRecord(*record, plan, Sha256OfFile(PlanPath(name)));
    for (const std::string& f : faults) {
      ADD_FAILURE() << "physics/" << name << ".json: " << f;
    }
    const std::optional<Scorecard> card = ScorecardFromJson(
        ReadFile(Dir("SCORES_DIR", "scores") + "/" + name + ".json"), &error);
    ASSERT_TRUE(card.has_value()) << "scores/" << name << ".json: " << error;
    const Measure* m = FindMeasure(*card, "physics_share");
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->value,
              Round6(CountedSeconds(*record) / CycleFiguresOf(plan).one_bay_s))
        << "scores/" << name << ".json";
    EXPECT_EQ(card->physics_sha256,
              Sha256OfFile(entry.path().string()));
  }
}

// The chosen layout has a committed record, its scorecard counts it, and so
// its physics_share is above 0 (R5 of run 019).
TEST(PhysicsRecord, TheChosenLayoutHasARecordAndAShareAboveZero) {
  std::string error;
  const std::optional<std::string> name = ChosenLayout(VersionsPath(), &error);
  ASSERT_TRUE(name.has_value()) << error;
  const std::string path = Dir("PHYSICS_DIR", "physics") + "/" + *name + ".json";
  const std::optional<PhysicsRecord> record = PhysicsRecordFromJson(ReadFile(path), &error);
  ASSERT_TRUE(record.has_value()) << path << ": " << error;
  EXPECT_GT(CountedSeconds(*record), 0.0);
  const std::optional<Scorecard> card = ScorecardFromJson(
      ReadFile(Dir("SCORES_DIR", "scores") + "/" + *name + ".json"), &error);
  ASSERT_TRUE(card.has_value()) << error;
  const Measure* m = FindMeasure(*card, "physics_share");
  ASSERT_NE(m, nullptr);
  ASSERT_TRUE(m->value.has_value());
  EXPECT_GT(*m->value, 0.0);
  EXPECT_EQ(m->known, "measured");
  EXPECT_NE(m->basis.find("physics/" + *name + ".json"), std::string::npos) << m->basis;
  // One module's cycle in physics, 22.068854 s of v10's one bay of
  // 560.452329 s: three bays in the plan do not change it.
  if (*name == "flat_rack_3_pin_arms_ordered") EXPECT_EQ(*m->value, 0.039377);
}

}  // namespace
}  // namespace cell::layout
