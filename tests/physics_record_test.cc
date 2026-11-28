// The record of a run in physics (src/physics_record.h): its text and the
// rule by which a step counts, what is checked against the committed plan,
// the scorecard's physics_share with a record, without one and with one that
// does not agree, and every record committed under physics/ (none yet).

#include "src/physics_record.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
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

// A record of steps 63 and 64 (module 4, module_take 3) that agrees with the committed plan.
PhysicsRecord Fixture(const Plan& plan, const std::string& plan_sha) {
  PhysicsRecord r;
  r.layout = kName;
  r.layout_sha256 = "ab12";
  r.plan_sha256 = plan_sha;
  r.command = "bazel run //:run_physics -- test";
  r.clearance_m = 0.015;
  r.gate_offset_mm = 3.0;
  r.gate_yaw_deg = 0.15;
  PhysicsTake take;
  take.module = 4;
  take.take = 3;
  take.steps = {StepOf(plan, 63, 0.0212), StepOf(plan, 64, 0.0151)};
  take.set_down = {1.2346, -0.5, 1.1, 0.0123, 0.4};
  r.takes = {take};
  r.counted_s = Round6(take.steps[0].planned_s + take.steps[1].planned_s);
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
  EXPECT_EQ(back->takes[0].steps.size(), 2u);
  EXPECT_EQ(back->takes[0].steps[1].end_s, record.takes[0].steps[1].end_s);
  EXPECT_EQ(back->takes[0].steps[0].least_clearance_pair, "gripper to rails");
  EXPECT_EQ(back->takes[0].set_down.offset_mm, 1.2346);
  EXPECT_EQ(back->counted_s, record.counted_s);
  EXPECT_TRUE(back->takes[0].steps[0].counted);
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
  PhysicsStep step = r.takes[0].steps[0];
  step.least_clearance_m = 0.015;
  EXPECT_TRUE(StepCounts(r, r.takes[0], step));
  step.least_clearance_m = 0.014999;
  EXPECT_FALSE(StepCounts(r, r.takes[0], step));
  step.least_clearance_m = 0.02;
  r.takes[0].set_down.offset_mm = 3.0;
  EXPECT_TRUE(StepCounts(r, r.takes[0], step));
  r.takes[0].set_down.offset_mm = 3.0001;
  EXPECT_FALSE(StepCounts(r, r.takes[0], step));
  r.takes[0].set_down.offset_mm = 1.0;
  r.takes[0].set_down.yaw_deg = -0.15;
  EXPECT_TRUE(StepCounts(r, r.takes[0], step));
  r.takes[0].set_down.yaw_deg = 0.1501;
  EXPECT_FALSE(StepCounts(r, r.takes[0], step));
  r.takes[0].set_down.yaw_deg = -0.1501;
  EXPECT_FALSE(StepCounts(r, r.takes[0], step));
  r.takes[0].set_down.yaw_deg = 0.0;
  EXPECT_DOUBLE_EQ(CountedSeconds(r), r.takes[0].steps[0].planned_s + r.takes[0].steps[1].planned_s);
  r.takes[0].steps[1].least_clearance_m = 0.001;
  r.takes[0].steps[1].counted = true;  // The stored flag is not what counts.
  EXPECT_DOUBLE_EQ(CountedSeconds(r), r.takes[0].steps[0].planned_s);
}

TEST(PhysicsRecord, TheCheckAgainstThePlan) {
  const Plan plan = LoadPlan(kName);
  const std::string sha = Sha256OfFile(PlanPath(kName));
  ASSERT_EQ(sha.size(), 64u);
  const PhysicsRecord good = Fixture(plan, sha);
  EXPECT_TRUE(CheckPhysicsRecord(good, plan, sha).empty());

  PhysicsRecord r = good;
  r.takes[0].steps[0].number = 99999;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "99999"));

  r = good;
  r.takes[0].steps[1].planned_s += 0.001;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "planned_s"));

  r = good;
  r.takes[0].steps[0].arm += "x";
  EXPECT_FALSE(CheckPhysicsRecord(r, plan, sha).empty());
  r = good;
  r.takes[0].steps[0].what += "x";
  EXPECT_FALSE(CheckPhysicsRecord(r, plan, sha).empty());
  r = good;
  r.takes[0].steps[0].start_s += 0.01;
  EXPECT_FALSE(CheckPhysicsRecord(r, plan, sha).empty());

  r = good;
  r.plan_sha256 = "00" + sha.substr(2);
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "plan_sha256"));

  r = good;
  r.takes[0].steps[0].counted = false;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "counted"));

  r = good;
  r.counted_s += 1.0;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "counted_s"));
}

// A record of two takes of bay 0: module 4 (steps 63, 64) and module 6
// (steps 123, 124), each with its own set-down.
PhysicsRecord TwoTakes(const Plan& plan, const std::string& plan_sha) {
  PhysicsRecord r = Fixture(plan, plan_sha);
  PhysicsTake six;
  six.module = 6;
  six.take = 6;
  six.steps = {StepOf(plan, 123, 0.0187), StepOf(plan, 124, 0.0163)};
  six.set_down = {0.3156, 0.1294, 0.2878, -0.0059, -0.0001};
  r.takes.push_back(six);
  r.counted_s = Round6(CountedSeconds(r));
  return r;
}

// A record of one take and the rail sets of rails 0 and 1 (steps 4 and 6
// of the plan), which agrees with the plan.
PhysicsRecord WithRails(const Plan& plan, const std::string& plan_sha) {
  PhysicsRecord r = Fixture(plan, plan_sha);
  r.rail_gate = RailPlay();
  for (const auto& [rail, step] : {std::pair{0, 4}, std::pair{1, 6}}) {
    PhysicsRailSet set;
    set.rail = rail;
    set.step = StepOf(plan, step, 0.0194);
    set.torque_ratio = {0.3467, 0.499, 0.628, 0.2441, 0.3876, 0.1319, 0.5923};
    set.pose = {0.0023, -0.0073, 0.0011, 0.0002, -0.0005};
    r.rails.push_back(set);
  }
  r.counted_s = Round6(CountedSeconds(r));
  return r;
}

TEST(PhysicsRecord, RailSetsBesideTheTakes) {
  const Plan plan = LoadPlan(kName);
  const std::string sha = Sha256OfFile(PlanPath(kName));
  const PhysicsRecord good = WithRails(plan, sha);
  // The play of docs/bay-hardware.md, Tolerances.
  EXPECT_EQ(good.rail_gate.across_mm, 0.928);
  EXPECT_EQ(good.rail_gate.along_mm, 2.606);
  EXPECT_EQ(good.rail_gate.round_deg, 0.8576);
  EXPECT_EQ(good.rail_gate.yaw_deg, 0.8251);
  EXPECT_TRUE(CheckPhysicsRecord(good, plan, sha).empty());
  const double take = good.takes[0].steps[0].planned_s + good.takes[0].steps[1].planned_s;
  const double rails = good.rails[0].step.planned_s + good.rails[1].step.planned_s;
  EXPECT_NEAR(CountedSeconds(good), take + rails, 1e-9);
  EXPECT_EQ(CountedRails(good), (std::vector<int>{0, 1}));
  EXPECT_EQ(CountedSteps(good), (std::vector<int>{4, 6, 63, 64}));
  EXPECT_EQ(RailsText(good), "rails 0, 1 (2 of the record's 2 rail sets)");

  // The text: the one take's fields, then the rail gate and the rail sets;
  // it reads back as it was. A record without rail sets has neither.
  const std::string text = PhysicsRecordToJson(good);
  EXPECT_NE(text.find("\n  \"rail_gate\": {"), std::string::npos);
  EXPECT_NE(text.find("\n  \"rails\": ["), std::string::npos);
  const std::string plain = PhysicsRecordToJson(Fixture(plan, sha));
  EXPECT_EQ(plain.find("\"rail_gate\""), std::string::npos);
  EXPECT_EQ(plain.find("\"rails\""), std::string::npos);
  std::string error;
  const std::optional<PhysicsRecord> back = PhysicsRecordFromJson(text, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(PhysicsRecordToJson(*back), text);
  ASSERT_EQ(back->rails.size(), 2u);
  EXPECT_EQ(back->rails[1].rail, 1);
  EXPECT_EQ(back->rails[1].torque_ratio[6], 0.5923);
  std::string short_torque = text;
  const std::string last = ",\n        0.5923\n      ]";
  const size_t at = short_torque.find(last);
  ASSERT_NE(at, std::string::npos);
  short_torque.replace(at, last.size(), "\n      ]");
  EXPECT_FALSE(PhysicsRecordFromJson(short_torque, &error).has_value());

  // The rail gate, part by part, at its edges.
  auto counts = [&](const std::function<void(PhysicsRailSet*)>& change) {
    PhysicsRecord r = good;
    change(&r.rails[0]);
    return RailSetCounts(r, r.rails[0]);
  };
  EXPECT_TRUE(counts([](PhysicsRailSet* s) { s->step.least_clearance_m = 0.015; }));
  EXPECT_FALSE(counts([](PhysicsRailSet* s) { s->step.least_clearance_m = 0.014999; }));
  EXPECT_TRUE(counts([](PhysicsRailSet* s) { s->torque_ratio[1] = 0.9999; }));
  EXPECT_FALSE(counts([](PhysicsRailSet* s) { s->torque_ratio[1] = 1.0; }));
  EXPECT_FALSE(counts([](PhysicsRailSet* s) { s->torque_ratio[6] = 1.0; }));
  EXPECT_TRUE(counts([](PhysicsRailSet* s) { s->pose.across_mm = -0.928; }));
  EXPECT_FALSE(counts([](PhysicsRailSet* s) { s->pose.across_mm = 0.9281; }));
  EXPECT_TRUE(counts([](PhysicsRailSet* s) { s->pose.along_mm = 2.606; }));
  EXPECT_FALSE(counts([](PhysicsRailSet* s) { s->pose.along_mm = -2.6061; }));
  EXPECT_TRUE(counts([](PhysicsRailSet* s) { s->pose.round_deg = -0.8576; }));
  EXPECT_FALSE(counts([](PhysicsRailSet* s) { s->pose.round_deg = 0.8577; }));
  EXPECT_TRUE(counts([](PhysicsRailSet* s) { s->pose.yaw_deg = 0.8251; }));
  EXPECT_FALSE(counts([](PhysicsRailSet* s) { s->pose.yaw_deg = -0.8252; }));
  EXPECT_TRUE(counts([](PhysicsRailSet* s) { s->pose.height_mm = 50.0; }));  // Not gated.
  // A rail set out of its gate takes its seconds out, the take's still count.
  PhysicsRecord r = good;
  r.rails[1].pose.yaw_deg = 1.0;
  EXPECT_NEAR(CountedSeconds(r), take + good.rails[0].step.planned_s, 1e-9);
  EXPECT_EQ(CountedRails(r), (std::vector<int>{0}));
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "the rule gives the other"));

  // One bay counts each rail once, and a rail set is the set of its rail.
  r = good;
  r.rails[1].rail = 0;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "rail 0 is set by two rail sets"));
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "is of step \"set rail 1\""));
  r = good;
  r.rails[1].step = StepOf(plan, 179, 0.0194);  // Bay 1's set of rail 1.
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "it counts one bay"));
  r = good;
  r.rails[0].step.start_s += 0.5;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "step 4 starts at"));
  // Rail sets alone are a record.
  r = good;
  r.takes.clear();
  r.counted_s = Round6(CountedSeconds(r));
  EXPECT_TRUE(CheckPhysicsRecord(r, plan, sha).empty());

  // Joined from records of one run each, given out of order.
  PhysicsRecord a = good, b = good, c = good;
  a.rails.clear();
  b.takes.clear();
  b.rails = {good.rails[1]};
  c.takes.clear();
  c.rails = {good.rails[0]};
  const std::optional<PhysicsRecord> joined = JoinPhysicsRecords({b, a, c}, good.command, &error);
  ASSERT_TRUE(joined.has_value()) << error;
  EXPECT_EQ(PhysicsRecordToJson(*joined), text);
  c.rail_gate.yaw_deg = 1.0;
  EXPECT_FALSE(JoinPhysicsRecords({b, a, c}, good.command, &error).has_value());
}

TEST(PhysicsRecord, ARecordOfTwoTakesOfOneBay) {
  const Plan plan = LoadPlan(kName);
  const std::string sha = Sha256OfFile(PlanPath(kName));
  const PhysicsRecord good = TwoTakes(plan, sha);
  const double four = good.takes[0].steps[0].planned_s + good.takes[0].steps[1].planned_s;
  const double six = good.takes[1].steps[0].planned_s + good.takes[1].steps[1].planned_s;
  EXPECT_NEAR(CountedSeconds(good), four + six, 1e-9);
  EXPECT_EQ(CountedModules(good), (std::vector<int>{4, 6}));
  EXPECT_TRUE(CheckPhysicsRecord(good, plan, sha).empty());

  // Its text holds a list of takes, and reads back as it was.
  const std::string text = PhysicsRecordToJson(good);
  EXPECT_NE(text.find("\n  \"takes\": ["), std::string::npos);
  EXPECT_EQ(text.find("\n  \"module\": "), std::string::npos);
  std::string error;
  const std::optional<PhysicsRecord> back = PhysicsRecordFromJson(text, &error);
  ASSERT_TRUE(back.has_value()) << error;
  EXPECT_EQ(PhysicsRecordToJson(*back), text);
  ASSERT_EQ(back->takes.size(), 2u);
  EXPECT_EQ(back->takes[1].module, 6);
  EXPECT_EQ(back->takes[1].steps[0].number, 123);
  EXPECT_EQ(back->takes[1].set_down.yaw_deg, -0.0059);

  // Each step is judged against its own take's set-down: module 6 out of the
  // gate takes its two steps out, module 4's still count.
  PhysicsRecord r = good;
  r.takes[1].set_down.offset_mm = 3.5;
  EXPECT_NEAR(CountedSeconds(r), four, 1e-9);
  EXPECT_EQ(CountedModules(r), (std::vector<int>{4}));
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "the rule gives the other"));
  for (PhysicsStep& s : r.takes[1].steps) s.counted = false;
  r.counted_s = Round6(CountedSeconds(r));
  EXPECT_TRUE(CheckPhysicsRecord(r, plan, sha).empty());

  // One bay counts each module once.
  r = good;
  r.takes[1].module = 4;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "module 4 is held by two takes"));

  // And the record is of one bay: module 6's take and set of bay 1 beside
  // module 4's of bay 0 are refused.
  r = good;
  r.takes[1].steps = {StepOf(plan, 296, 0.0187), StepOf(plan, 297, 0.0163)};
  r.counted_s = Round6(CountedSeconds(r));
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "bays"));

  // A record without takes is a fault, not a share of 0.
  r = good;
  r.takes.clear();
  r.counted_s = 0.0;
  EXPECT_TRUE(Mentions(CheckPhysicsRecord(r, plan, sha), "no take"));
}

TEST(PhysicsRecord, JoiningRecordsOfOneTakeEach) {
  const Plan plan = LoadPlan(kName);
  const std::string sha = Sha256OfFile(PlanPath(kName));
  const PhysicsRecord two = TwoTakes(plan, sha);
  PhysicsRecord a = two;
  PhysicsRecord b = two;
  a.takes = {two.takes[0]};
  b.takes = {two.takes[1]};
  std::string error;
  // Given out of order, the takes come out in the plan's.
  const std::optional<PhysicsRecord> joined = JoinPhysicsRecords({b, a}, two.command, &error);
  ASSERT_TRUE(joined.has_value()) << error;
  EXPECT_EQ(PhysicsRecordToJson(*joined), PhysicsRecordToJson(two));
  b.plan_sha256 = "00";
  EXPECT_FALSE(JoinPhysicsRecords({a, b}, two.command, &error).has_value());
  EXPECT_FALSE(JoinPhysicsRecords({}, two.command, &error).has_value());
}

// The scorecard takes about 16 s a call, so five calls serve every case.
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
  const PhysicsRecord two = TwoTakes(plan, plan_sha);
  const auto both = ComputeScorecard(*layout, plan, *rates, paths, layout_sha,
                                     plan_sha, rates_sha, &two, &error);
  ASSERT_TRUE(both.has_value()) << error;
  const PhysicsRecord rails = WithRails(plan, plan_sha);
  const auto railed = ComputeScorecard(*layout, plan, *rates, paths, layout_sha,
                                       plan_sha, rates_sha, &rails, &error);
  ASSERT_TRUE(railed.has_value()) << error;
  // With rail sets: their steps among the take's, in order, and the rails named.
  const Measure* r = FindMeasure(*railed, "physics_share");
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(r->value, Round6(CountedSeconds(rails) / CycleFiguresOf(plan).one_bay_s));
  EXPECT_NE(r->basis.find("steps 4, 6, 63, 64 of module 4 and rails 0, 1 (2 of the record's 2 "
                          "rail sets) carried out in physics"),
            std::string::npos)
      << r->basis;

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

  // Two takes: their seconds summed, the modules named.
  const Measure* summed = FindMeasure(*both, "physics_share");
  ASSERT_NE(summed, nullptr);
  ASSERT_TRUE(summed->value.has_value());
  EXPECT_EQ(*summed->value, Round6(CountedSeconds(two) / CycleFiguresOf(plan).one_bay_s));
  EXPECT_GT(*summed->value, *m->value);
  EXPECT_NE(summed->basis.find("steps 63, 64, 123, 124 of modules 4, 6 (2 of the record's 2 "
                               "takes of one bay)"),
            std::string::npos)
      << summed->basis;

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
