// The trial runner (src/trial_runner.h) itself, apart from the cell: the
// verdict at the tolerance's edges on hand-made results, a failure record
// saved and loaded back, and, in the cell, a replay of one short trial equal
// to the original bit for bit, also when trials run on two threads.
//
//   bazel test --config=opt //:trial_runner_test --test_output=all

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/trial_runner.h"

namespace cell {
namespace {

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kInf = std::numeric_limits<double>::infinity();

// A clean placement of module 4 in slot 4: on the rails, at rest, no error.
CycleResult Placed(double along = 0.0, double across = 0.0, double yaw = 0.0,
                   double tilt = 0.0) {
  CycleResult c;
  c.seed = 9;
  c.module = 4;
  c.slot = 4;
  c.placed = true;
  PlacementError e;
  e.module = 4;
  e.slot = 4;
  e.along_tube = along;
  e.across_tube = across;
  e.yaw = yaw;
  e.tilt = tilt;
  e.at_rest = true;
  c.error = e;
  return c;
}

double Above(double x) { return std::nextafter(x, kInf); }

// --------------------------------------------------------------- verdict

TEST(Tolerance, DefaultsAreTheOwners) {
  const PlacementTolerance t;
  EXPECT_DOUBLE_EQ(t.position, 3e-3);
  EXPECT_DOUBLE_EQ(t.yaw, 0.15 * kDegree);
  EXPECT_DOUBLE_EQ(t.tilt, 0.15 * kDegree);
}

TEST(Verdict, PositionIsInTheRailsPlane) {
  const PlacementTolerance t;
  EXPECT_TRUE(JudgeCycle(Placed(3e-3, 0.0)).in_tolerance);
  EXPECT_TRUE(JudgeCycle(Placed(0.0, -3e-3)).in_tolerance);
  EXPECT_FALSE(JudgeCycle(Placed(Above(3e-3), 0.0)).in_tolerance);
  EXPECT_FALSE(JudgeCycle(Placed(0.0, -Above(3e-3))).in_tolerance);
  // 2.2 and 2.2 mm: each within 3 mm, together 3.11 mm.
  const CycleVerdict both = JudgeCycle(Placed(2.2e-3, 2.2e-3));
  EXPECT_FALSE(both.in_tolerance);
  EXPECT_NE(both.why.find("position 3.11 mm > 3.00 mm"), std::string::npos)
      << both.why;
  EXPECT_TRUE(JudgeCycle(Placed(2.1e-3, 2.1e-3)).in_tolerance);  // 2.97 mm
  // Height is not part of the position.
  CycleResult high = Placed(3e-3, 0.0);
  high.error->height = 1.9e-3;
  EXPECT_TRUE(JudgeCycle(high, t).in_tolerance);
}

TEST(Verdict, YawAndTiltEachAtTheirLimit) {
  const PlacementTolerance t;
  EXPECT_TRUE(JudgeCycle(Placed(0, 0, t.yaw, 0)).in_tolerance);
  EXPECT_TRUE(JudgeCycle(Placed(0, 0, -t.yaw, 0)).in_tolerance);
  EXPECT_FALSE(JudgeCycle(Placed(0, 0, Above(t.yaw), 0)).in_tolerance);
  EXPECT_FALSE(JudgeCycle(Placed(0, 0, -Above(t.yaw), 0)).in_tolerance);
  EXPECT_TRUE(JudgeCycle(Placed(0, 0, 0, t.tilt)).in_tolerance);
  EXPECT_FALSE(JudgeCycle(Placed(0, 0, 0, Above(t.tilt))).in_tolerance);
  // All three at their limits at once is still in.
  EXPECT_TRUE(JudgeCycle(Placed(3e-3, 0, t.yaw, t.tilt)).in_tolerance);
  const CycleVerdict two = JudgeCycle(Placed(0, 0, 0.2 * kDegree, 0.3 * kDegree));
  EXPECT_FALSE(two.in_tolerance);
  EXPECT_NE(two.why.find("yaw 0.200 deg"), std::string::npos) << two.why;
  EXPECT_NE(two.why.find("tilt 0.300 deg"), std::string::npos) << two.why;
}

TEST(Verdict, MustRestOnTheRails) {
  const PlacementTolerance t;
  CycleResult moving = Placed();
  moving.error->at_rest = false;
  EXPECT_FALSE(JudgeCycle(moving).in_tolerance);
  CycleResult high = Placed();
  high.error->height = Above(t.rest_height);
  EXPECT_FALSE(JudgeCycle(high).in_tolerance);
  high.error->height = -Above(t.rest_height);
  EXPECT_FALSE(JudgeCycle(high).in_tolerance);
  high.error->height = t.rest_height;
  EXPECT_TRUE(JudgeCycle(high).in_tolerance);
  // On another module: a module's thickness up.
  high.error->height = 0.04;
  EXPECT_FALSE(JudgeCycle(high).in_tolerance);
}

TEST(Verdict, FailedUnconfirmedOrUnscoredIsOut) {
  CycleResult failed = Placed();
  failed.placed = false;
  failed.error.reset();
  failed.failed_step = PickPlaceStep::kCarry;
  failed.reason = "no path";
  const CycleVerdict v = JudgeCycle(failed);
  EXPECT_FALSE(v.in_tolerance);
  EXPECT_EQ(v.why, "failed in carry: no path");

  CycleResult unconfirmed = Placed();  // perfect numbers, but let go blocked
  unconfirmed.unconfirmed = true;
  EXPECT_FALSE(JudgeCycle(unconfirmed).in_tolerance);

  CycleResult unscored = Placed();
  unscored.error.reset();
  EXPECT_FALSE(JudgeCycle(unscored).in_tolerance);

  // NaN is never in tolerance.
  EXPECT_FALSE(
      JudgeCycle(Placed(std::numeric_limits<double>::quiet_NaN())).in_tolerance);
}

TEST(Verdict, ATrialPassesWhenEveryCycleIsIn) {
  TrialResult trial;
  trial.settings.modules = 3;
  trial.cycles = {Placed(), Placed(1e-3), Placed(0, 0, 0.1 * kDegree)};
  EXPECT_TRUE(JudgeTrial(trial).passed);

  trial.cycles[1] = Placed(3.5e-3);
  TrialVerdict v = JudgeTrial(trial);
  EXPECT_FALSE(v.passed);
  ASSERT_EQ(v.bad_cycles, std::vector<int>{1});
  EXPECT_EQ(v.why, "1 of 3 cycles out of tolerance");

  // A failed cycle ends the trial: fewer cycles, the failed one is bad.
  trial.cycles = {Placed(), Placed()};
  trial.cycles[1].placed = false;
  v = JudgeTrial(trial);
  EXPECT_FALSE(v.passed);
  EXPECT_EQ(v.bad_cycles, std::vector<int>{1});

  // Fewer cycles than asked, none bad: still not a pass.
  trial.cycles = {Placed(), Placed()};
  v = JudgeTrial(trial);
  EXPECT_FALSE(v.passed);
  EXPECT_TRUE(v.bad_cycles.empty());
  EXPECT_EQ(v.why, "ran 2 of 3 cycles");

  TrialResult refused;
  refused.error = "no";
  EXPECT_FALSE(JudgeTrial(refused).passed);
}

// ------------------------------------------------------ save and load

TEST(FailureRecord, SavedAndLoadedBackExactly) {
  TrialResult trial;
  trial.settings = GateTrialSettings(4242);
  trial.settings.xy_mm = 12.5;
  trial.settings.yaw_deg = 1.0 / 3.0;
  trial.settings.first = 2;
  trial.settings.modules = 5;
  trial.cycles = {Placed(), Placed(), Placed()};
  for (int i = 0; i < 3; ++i) {
    trial.cycles[i].module = 2 + i;
    trial.cycles[i].seed = 4242;
    trial.cycles[i].cycle_time = 23.0 + std::sqrt(2.0) * i;
  }
  // Cycle 1: out by a hair, numbers that need all 17 digits.
  CycleResult& bad = trial.cycles[1];
  bad.slot = 7;
  bad.error->slot = 7;
  bad.error->along_tube = -std::sqrt(10.0) * 1e-3;
  bad.error->across_tube = 1e-17;
  bad.error->height = -3.0e-6 / 7.0;
  bad.error->yaw = std::numbers::pi / 1e5;
  bad.error->tilt = 1.0 / 3.0 * 1e-4;
  bad.error->speed = 2e-4;
  bad.error->spin = 3e-3;
  bad.reason = "check retaken: something: with, commas";
  trial.cycles[2].placed = false;
  trial.cycles[2].error.reset();
  trial.cycles[2].failed_step = PickPlaceStep::kSetDown;
  trial.cycles[2].reason = "blocked\nsecond line";

  const TrialVerdict verdict = JudgeTrial(trial);
  ASSERT_EQ(verdict.bad_cycles, (std::vector<int>{1, 2}));
  const FailureRecord record = MakeFailureRecord(trial, verdict);
  EXPECT_EQ(record.cycle, 1);
  EXPECT_EQ(record.module, 3);
  EXPECT_EQ(record.slot, 7);
  EXPECT_EQ(record.outcome, "placed");
  EXPECT_EQ(record.step, "-");
  EXPECT_EQ(record.bad_cycles, 2);

  const char* tmp = std::getenv("TEST_TMPDIR");
  const std::string dir =
      std::string(tmp != nullptr ? tmp : "/tmp") + "/trial_runner_test";
  std::string error;
  const std::optional<std::string> path = SaveFailure(dir, record, &error);
  ASSERT_TRUE(path.has_value()) << error;
  EXPECT_NE(path->find("failure_seed_4242_module_3.txt"), std::string::npos);
  const std::optional<FailureRecord> loaded = LoadFailure(*path, &error);
  ASSERT_TRUE(loaded.has_value()) << error;
  EXPECT_TRUE(CompareFailure(record, *loaded).empty());
  for (const std::string& d : CompareFailure(record, *loaded)) ADD_FAILURE() << d;

  // The settings come back field for field.
  const TrialSettings& a = record.settings;
  const TrialSettings& b = loaded->settings;
  EXPECT_EQ(a.seed, b.seed);
  EXPECT_EQ(a.modules, b.modules);
  EXPECT_EQ(a.first, b.first);
  EXPECT_EQ(a.nominal, b.nominal);
  EXPECT_EQ(a.xy_mm, b.xy_mm);
  EXPECT_EQ(a.yaw_deg, b.yaw_deg);
  EXPECT_EQ(a.camera, b.camera);
  EXPECT_EQ(a.noise, b.noise);
  EXPECT_EQ(a.calibration_mm, b.calibration_mm);
  EXPECT_EQ(a.calibration_deg, b.calibration_deg);
  EXPECT_EQ(a.carry, b.carry);
  EXPECT_EQ(a.check, b.check);
  EXPECT_EQ(a.check_calibration_mm, b.check_calibration_mm);
  EXPECT_EQ(a.check_calibration_deg, b.check_calibration_deg);
  EXPECT_EQ(loaded->error->along_tube, bad.error->along_tube);
  EXPECT_EQ(loaded->error->across_tube, 1e-17);

  // A replay needs only the trial up to the bad cycle.
  EXPECT_EQ(ReplaySettings(*loaded).modules, 2);
  EXPECT_EQ(RunCellFlags(loaded->settings, loaded->cycle),
            "--seed 4242 --modules 2 --first 2 --xy 12.5 "
            "--yaw 0.3333333333333333 --calibration 1,0.05 --carry planned "
            "--check --check-calibration 1,0.05");
  EXPECT_NE(FormatFailureRecord(record, "/x/f.txt")
                .find("replay = bazel run --config=opt //:run_cell -- "
                      "--pretty --replay /x/f.txt"),
            std::string::npos);

  // A changed number is caught, and named.
  FailureRecord changed = *loaded;
  changed.error->tilt = std::nextafter(changed.error->tilt, 1.0);
  const std::vector<std::string> d = CompareFailure(record, changed);
  ASSERT_EQ(d.size(), 1u);
  EXPECT_EQ(d[0].rfind("tilt: ", 0), 0u) << d[0];
}

TEST(FailureRecord, RefusesWhatItCannotRead) {
  std::string error;
  EXPECT_FALSE(ParseFailureRecord("seed = 1\n", &error).has_value());
  EXPECT_NE(error.find("missing"), std::string::npos) << error;
  EXPECT_FALSE(ParseFailureRecord("seed 1\n", &error).has_value());
  EXPECT_FALSE(LoadFailure("/nonexistent/file.txt", &error).has_value());
}

// --------------------------------------------------- replay in the cell

const char* ModelPath() {
  const char* path = std::getenv("MODEL_PATH");
  return path != nullptr ? path : "models/cell.xml";
}

// Every number of a cycle, compared bit for bit, the wall-clock times
// apart (those are measured, not simulated).
void ExpectSameCycle(const CycleResult& a, const CycleResult& b) {
  EXPECT_EQ(a.module, b.module);
  EXPECT_EQ(a.slot, b.slot);
  EXPECT_EQ(a.placed, b.placed);
  EXPECT_EQ(a.reason, b.reason);
  EXPECT_EQ(a.cycle_time, b.cycle_time);
  ASSERT_EQ(a.error.has_value(), b.error.has_value());
  if (a.error) {
    EXPECT_EQ(a.error->along_tube, b.error->along_tube);
    EXPECT_EQ(a.error->across_tube, b.error->across_tube);
    EXPECT_EQ(a.error->height, b.error->height);
    EXPECT_EQ(a.error->yaw, b.error->yaw);
    EXPECT_EQ(a.error->tilt, b.error->tilt);
    EXPECT_EQ(a.error->speed, b.error->speed);
  }
  EXPECT_EQ(a.disturbance.worst_displacement, b.disturbance.worst_displacement);
  ASSERT_EQ(a.seen.has_value(), b.seen.has_value());
  if (a.seen) {
    EXPECT_EQ(a.seen->normal, b.seen->normal);
    EXPECT_EQ(a.seen->long_side, b.seen->long_side);
  }
  EXPECT_EQ(a.check.status, b.check.status);
  EXPECT_EQ(a.check.offset_shift, b.check.offset_shift);
  EXPECT_EQ(a.check.offset_rotation, b.check.offset_rotation);
  EXPECT_EQ(a.check_error_shift, b.check_error_shift);
  EXPECT_EQ(a.carry.clearance, b.carry.clearance);
  EXPECT_EQ(a.carry.trajectory_seconds, b.carry.trajectory_seconds);
  EXPECT_EQ(a.peak_torque, b.peak_torque);
}

// One camera cycle with the check and both calibrations off, judged against
// a tolerance it cannot meet (1 micrometre), so it has a failure record;
// then the record's replay, on the same cell after another Reset.
TEST(Replay, EqualsTheOriginalBitForBit) {
  std::string error;
  std::optional<Cell> cell = Cell::Load(ModelPath(), &error);
  ASSERT_TRUE(cell.has_value()) << error;
  TrialSettings settings = GateTrialSettings(3);
  settings.modules = 1;
  PlacementTolerance tight;
  tight.position = 1e-6;

  const TrialResult original = RunTrial(*cell, settings);
  ASSERT_TRUE(original.error.empty()) << original.error;
  ASSERT_EQ(original.cycles.size(), 1u);
  ASSERT_TRUE(original.cycles[0].placed) << original.cycles[0].reason;
  EXPECT_TRUE(JudgeTrial(original).passed);  // in the real tolerance
  const TrialVerdict verdict = JudgeTrial(original, tight);
  ASSERT_FALSE(verdict.passed);
  const FailureRecord record = MakeFailureRecord(original, verdict, tight);

  const std::optional<FailureRecord> loaded =
      ParseFailureRecord(FormatFailureRecord(record, "f.txt"), &error);
  ASSERT_TRUE(loaded.has_value()) << error;
  const TrialResult replay = RunTrial(*cell, ReplaySettings(*loaded));
  ASSERT_EQ(replay.cycles.size(), 1u);
  const FailureRecord again =
      MakeFailureRecord(replay, JudgeTrial(replay, tight), tight);
  for (const std::string& d : CompareFailure(*loaded, again)) ADD_FAILURE() << d;
  ExpectSameCycle(original.cycles[0], replay.cycles[0]);
  EXPECT_EQ(original.simulated_seconds, replay.simulated_seconds);
}

// Two trials on two threads, each with its own Cell, give what one thread
// gives one after the other: nothing is shared between Cells.
TEST(Replay, ThreadsDoNotChangeATrial) {
  std::vector<TrialSettings> trials = {GateTrialSettings(5),
                                       GateTrialSettings(6)};
  for (TrialSettings& s : trials) s.modules = 1;
  std::string error;
  const auto parallel = RunTrials(ModelPath(), trials, 2, {}, &error);
  ASSERT_TRUE(parallel.has_value()) << error;
  std::optional<Cell> cell = Cell::Load(ModelPath(), &error);
  ASSERT_TRUE(cell.has_value()) << error;
  for (int i = 0; i < 2; ++i) {
    SCOPED_TRACE("seed " + std::to_string(trials[i].seed));
    const TrialResult alone = RunTrial(*cell, trials[i]);
    ASSERT_EQ((*parallel)[i].cycles.size(), alone.cycles.size());
    ASSERT_EQ(alone.cycles.size(), 1u);
    ExpectSameCycle((*parallel)[i].cycles[0], alone.cycles[0]);
  }
}

}  // namespace
}  // namespace cell
