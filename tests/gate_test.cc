// The gate: whole pallets (30 modules onto three tubes) with the camera,
// noisy pictures, the planned carry, the check after pickup, and both
// cameras' calibrations off by 1 mm and 0.05 degrees
// (cell::GateTrialSettings), each placement judged against the placement
// tolerance (src/trial_runner.h: 3 mm in the rails' plane, 0.15 degrees of
// yaw and of tilt, resting on the rails). Fails on any cycle out of
// tolerance, printing its seed, module, errors and the command that reruns
// it, and writes a failure file per failed trial into the test's
// undeclared outputs (bazel-testlogs/gate_test/shard_<k>_of_<n>/
// test.outputs/outputs.zip), for run_cell --replay.
//
// kGateTrials trials, seeds kGateFirstSeed onward, split over the Bazel
// shards (trial i in shard i % shards), one trial at a time per shard.
// Run by hand (bazel run, no shards) all of them run in one process on as
// many threads as the machine has cores; GATE_THREADS in the environment
// sets the number of threads either way. By hand, --save-failures DIR
// writes the failure files into DIR as well.
//
//   bazel test --config=opt //:gate_test --test_output=all
//   bazel run --config=opt --test_sharding_strategy=disabled //:gate_test \
//       -- --save-failures /tmp/gate
//   bazel run --config=opt //:run_gate -- --trials 40   (by hand, larger)

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "src/trial_runner.h"

namespace cell {
namespace {

// --save-failures, by hand.
std::string save_failures;

int EnvInt(const char* name, int fallback) {
  const char* v = std::getenv(name);
  return v != nullptr && *v != '\0' ? std::atoi(v) : fallback;
}

TEST(Gate, WholePalletsPlaceInTolerance) {
  // Under bazel run the environment of the test is not set; the runfiles
  // are the working directory then.
  const char* model = std::getenv("MODEL_PATH");
  if (model == nullptr) model = "models/cell.xml";
  // Bazel's sharding: say this test knows it, then take every n-th trial.
  if (const char* status = std::getenv("TEST_SHARD_STATUS_FILE")) {
    std::ofstream(status).flush();
  }
  const bool sharded = std::getenv("TEST_TOTAL_SHARDS") != nullptr;
  const int shards = std::max(1, EnvInt("TEST_TOTAL_SHARDS", 1));
  const int shard = EnvInt("TEST_SHARD_INDEX", 0);
  std::vector<TrialSettings> trials;
  for (int i = shard; i < kGateTrials; i += shards) {
    trials.push_back(GateTrialSettings(kGateFirstSeed + i));
  }
  if (trials.empty()) GTEST_SKIP() << "no trial falls in this shard";
  const char* outputs = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR");

  const PlacementTolerance tolerance;
  BatchSummary summary;
  std::printf("shard %d of %d: %zu whole-pallet trials (%s)\n", shard + 1,
              shards, trials.size(), RunCellFlags(trials.front()).c_str());
  std::string error;
  const std::optional<std::vector<TrialResult>> results = RunTrials(
      model, trials,
      EnvInt("GATE_THREADS",
             sharded ? 1
                     : static_cast<int>(std::thread::hardware_concurrency())),
      [&](int, const TrialResult& trial) {
        const TrialVerdict verdict = JudgeTrial(trial, tolerance);
        AddToSummary(trial, verdict, &summary);
        std::printf("%s\n", DescribeTrial(trial, verdict).c_str());
        std::fflush(stdout);
      },
      &error);
  ASSERT_TRUE(results.has_value()) << error;

  for (const TrialResult& trial : *results) {
    const TrialVerdict verdict = JudgeTrial(trial, tolerance);
    if (verdict.passed) continue;
    const FailureRecord record = MakeFailureRecord(trial, verdict, tolerance);
    std::string saved = "not saved (no TEST_UNDECLARED_OUTPUTS_DIR)";
    if (outputs != nullptr) {
      std::string save_error;
      const std::optional<std::string> path =
          SaveFailure(outputs, record, &save_error);
      saved = path ? "saved " + FailureFileName(record) +
                         " in the test's outputs (outputs.zip)"
                   : "not saved: " + save_error;
    }
    if (!save_failures.empty()) {
      std::string save_error;
      const std::optional<std::string> path =
          SaveFailure(save_failures, record, &save_error);
      saved += path ? "; saved " + *path : "; not saved: " + save_error;
    }
    std::string lines;
    for (std::size_t k = 0; k < verdict.bad_cycles.size(); ++k) {
      lines += "\n  " + DescribeBadCycle(trial.cycles[verdict.bad_cycles[k]],
                                         verdict.bad_why[k]);
    }
    ADD_FAILURE() << "seed " << trial.settings.seed
                  << " out of tolerance: " << verdict.why << lines
                  << "\n  rerun: " << RerunCommand(record) << "\n  " << saved
                  << "\n  replay it with: "
                  << ReplayCommand(FailureFileName(record));
  }
  std::printf("\n%s", DescribeSummary(summary, tolerance).c_str());
}

}  // namespace
}  // namespace cell

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  // Bazel's shards are the gate's own (trial i in shard i % shards), not
  // googletest's, which would run this one test in the first shard only.
  // googletest read its shard flags from the environment at start-up.
  GTEST_FLAG_SET(total_shards, -1);
  GTEST_FLAG_SET(shard_index, -1);
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    std::string dir;
    if (arg == "--save-failures" && i + 1 < argc) {
      dir = argv[++i];
    } else if (arg.rfind("--save-failures=", 0) == 0) {
      dir = arg.substr(std::strlen("--save-failures="));
    } else {
      std::fprintf(stderr, "unknown argument %s\n", arg.c_str());
      return 1;
    }
    const char* cwd = std::getenv("BUILD_WORKING_DIRECTORY");
    if (!dir.empty() && dir[0] != '/' && cwd != nullptr) {
      dir = std::string(cwd) + "/" + dir;
    }
    cell::save_failures = dir;
  }
  return RUN_ALL_TESTS();
}
