// The gate by hand, for a larger batch than //:gate_test runs: whole
// pallets with the gate's settings (cell::GateTrialSettings: camera, noisy
// pictures, planned carry, check after pickup, both cameras' calibrations
// off by 1 mm and 0.05 degrees), each judged against the placement
// tolerance (src/trial_runner.h), on several threads, one Cell each.
//
//   bazel run --config=opt //:run_gate -- --trials 40
//   bazel run --config=opt //:run_gate -- [--trials N] [--seed S]
//       [--threads K] [--modules M] [--save-failures DIR]
//
//   --trials N   how many trials (default kGateTrials, the gate's 8)
//   --seed S     the first seed (default kGateFirstSeed, 1); trial t uses
//                S + t, as run_cell
//   --threads K  trials at once (default: the machine's cores, at most N)
//   --modules M  modules per trial (default 30: a whole pallet); fewer for
//                a quick look, but then it is not the gate
//   --save-failures DIR
//                one file per failed trial, for run_cell --replay
//
// One line per trial as it ends (in the order they end), every bad cycle
// with the command that reruns it, and the batch's worsts. Exit status 0
// when every trial passed, 1 when one did not.
//
// The first argument is the cell's model; BUILD.bazel passes it.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "src/trial_runner.h"

namespace {

std::string FromWorkingDirectory(const std::string& path) {
  const char* cwd = std::getenv("BUILD_WORKING_DIRECTORY");
  if (path.empty() || path[0] == '/' || cwd == nullptr || *cwd == '\0') {
    return path;
  }
  return std::string(cwd) + "/" + path;
}

}  // namespace

int main(int argc, char** argv) {
  const char* usage =
      "usage: %s <cell.xml> [--trials N] [--seed S] [--threads K] "
      "[--modules M] [--save-failures DIR]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  int trials = cell::kGateTrials;
  long first_seed = cell::kGateFirstSeed;
  int threads = static_cast<int>(std::thread::hardware_concurrency());
  int modules = 30;
  std::string save_failures;
  for (int i = 2; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--trials") == 0 && has_value) {
      trials = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--seed") == 0 && has_value) {
      first_seed = std::atol(argv[++i]);
    } else if (std::strcmp(argv[i], "--threads") == 0 && has_value) {
      threads = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--modules") == 0 && has_value) {
      modules = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--save-failures") == 0 && has_value) {
      save_failures = FromWorkingDirectory(argv[++i]);
    } else {
      std::fprintf(stderr, usage, argv[0]);
      return 1;
    }
  }
  if (trials < 1 || first_seed < 0 || modules < 1) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  if (threads < 1) threads = 1;

  std::vector<cell::TrialSettings> settings;
  for (int t = 0; t < trials; ++t) {
    settings.push_back(
        cell::GateTrialSettings(static_cast<std::uint32_t>(first_seed + t)));
    settings.back().modules = modules;
  }
  std::printf("gate: %d trials of %d modules from seed %ld (%s), %d "
              "threads\n",
              trials, modules, first_seed,
              cell::RunCellFlags(settings.front()).c_str(),
              std::min(threads, trials));
  std::fflush(stdout);

  const cell::PlacementTolerance tolerance;
  cell::BatchSummary summary;
  std::vector<std::string> failures;
  const auto wall_start = std::chrono::steady_clock::now();
  std::string error;
  const std::optional<std::vector<cell::TrialResult>> results =
      cell::RunTrials(
          argv[1], settings, threads,
          [&](int, const cell::TrialResult& trial) {
            const cell::TrialVerdict verdict =
                cell::JudgeTrial(trial, tolerance);
            cell::AddToSummary(trial, verdict, &summary);
            std::printf("%s\n", cell::DescribeTrial(trial, verdict).c_str());
            for (std::size_t k = 0; k < verdict.bad_cycles.size(); ++k) {
              std::printf("  OUT OF TOLERANCE: %s\n",
                          cell::DescribeBadCycle(
                              trial.cycles[verdict.bad_cycles[k]],
                              verdict.bad_why[k])
                              .c_str());
            }
            if (!verdict.passed) {
              const cell::FailureRecord record =
                  cell::MakeFailureRecord(trial, verdict, tolerance);
              std::printf("  rerun: %s\n", cell::RerunCommand(record).c_str());
              if (!save_failures.empty()) {
                std::string save_error;
                const std::optional<std::string> path =
                    cell::SaveFailure(save_failures, record, &save_error);
                if (path) {
                  std::printf("  saved: %s\n  replay: %s\n", path->c_str(),
                              cell::ReplayCommand(*path).c_str());
                } else {
                  std::printf("  not saved: %s\n", save_error.c_str());
                }
              }
              failures.push_back("seed " + std::to_string(trial.settings.seed));
            }
            std::fflush(stdout);
          },
          &error);
  if (!results) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  const double wall = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - wall_start)
                          .count();
  std::printf("\n%s", cell::DescribeSummary(summary, tolerance).c_str());
  std::printf("wall time %.0f s\n", wall);
  if (!failures.empty()) {
    std::printf("GATE FAILED:");
    for (const std::string& f : failures) std::printf(" %s", f.c_str());
    std::printf("\n");
    return 1;
  }
  std::printf("GATE PASSED\n");
  return 0;
}
