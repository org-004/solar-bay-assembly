// Runs the blind pick and place (src/pick_place.h) headless and scores it:
// for each trial, resets the cell, picks and places modules one after
// another, and prints one row per module. The output is CSV, for a plotting
// tool; with --pretty it is a table of aligned columns, for reading in a
// terminal.
//
//   bazel run --config=opt //:run_cell -- --pretty
//   bazel run --config=opt //:run_cell -- --pretty --trials 10 --modules 2
//   bazel run //:run_cell -- [--pretty] [--seed N] [--modules M] [--phases]
//                            [--trials T] [--nominal] [--verbose]
//
//   --seed N     the first trial's seed (default 1); trial t uses N + t
//   --trials T   how many trials (default 1)
//   --modules M  modules per trial, module_0 first (default 1, up to 30)
//   --first K    start at module_K: modules 0 .. K-1 are carried off the
//                pallet (Cell::ParkModule) before the trial, as if already
//                picked, and the stack is left a second to settle. For
//                trying the deep end of the stack without the cycles before.
//   --nominal    the pallet exactly where it was designed, and the conveyor
//                stopping exactly: no randomness at all. Without it the
//                pallet is set down off nominal as cell::Randomization's
//                defaults allow (3 cm, 3 degrees) and the conveyor stops a
//                little off.
//   --xy MM      the pallet's offset range in x and y instead of the
//                default 30 mm (drawn uniformly in -MM .. MM)
//   --yaw DEG    the pallet's turn range instead of the default 3 degrees
//   --phases     after each cycle's row, one line per step of the cycle:
//                how long it took and how it ended (in CSV output as
//                comment lines starting with #). With --pretty a failed
//                cycle always gets these lines, so a broken run shows where
//                it stopped.
//   --verbose    on stderr: how long each step took and what the cycle
//                check saw (contacts, clearances, torques) for each cycle
//
// A trial stops at its first failed cycle: with the front module still on
// the pallet, nothing behind it can be picked.
//
// The script is blind; this program is not. The pallet's true offset and
// the scores come from the truth (Cell::true_scene, src/score.h), and are
// only printed. The ratio of simulated to wall-clock time goes to stderr.
//
// The first argument is the cell's model; BUILD.bazel passes it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "src/cell.h"
#include "src/cycle_check.h"
#include "src/pick_place.h"
#include "src/score.h"

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

// A cycle that has not ended after this much simulated time has hung.
constexpr double kCycleLimit = 120.0;  // s

// After a failed cycle the stack is left this long before it is measured,
// so a module knocked loose has come to rest.
constexpr double kSettleAfterFailure = 1.0;  // s

struct Row {
  std::uint32_t seed = 0;
  int module = -1;
  double dx = 0.0, dy = 0.0, yaw = 0.0;  // true pallet offset
  bool placed = false;
  std::string reason;
  std::optional<cell::PlacementError> error;
  cell::StackDisturbance disturbance;
  double cycle_time = 0.0;
};

void PrintHeader(bool pretty) {
  if (pretty) {
    std::printf("%5s %3s | %7s %7s %6s | %-6s %4s | %7s %7s %7s %7s %6s %4s | "
                "%7s %7s | %6s | %s\n",
                "seed", "mod", "dx mm", "dy mm", "yaw d", "result", "slot",
                "along", "across", "height", "yaw d", "tilt d", "rest",
                "stack", "turn d", "cycle", "reason");
  } else {
    std::printf("seed,module,pallet_dx_mm,pallet_dy_mm,pallet_yaw_deg,outcome,"
                "slot,along_mm,across_mm,height_mm,yaw_deg,tilt_deg,at_rest,"
                "stack_moved_mm,stack_turned_deg,cycle_s,reason\n");
  }
}

void PrintRow(const Row& r, bool pretty) {
  const cell::PlacementError e = r.error.value_or(cell::PlacementError{});
  const bool scored = r.error.has_value();
  const char* outcome = r.placed ? "placed" : "failed";
  if (pretty) {
    char score[120] = "";
    if (scored) {
      std::snprintf(score, sizeof(score), "%7.2f %7.2f %7.2f %7.3f %6.3f %4s",
                    1e3 * e.along_tube, 1e3 * e.across_tube, 1e3 * e.height,
                    e.yaw / kDegree, e.tilt / kDegree,
                    e.at_rest ? "yes" : "no");
    } else {
      std::snprintf(score, sizeof(score), "%7s %7s %7s %7s %6s %4s", "-", "-",
                    "-", "-", "-", "-");
    }
    std::printf("%5u %3d | %7.2f %7.2f %6.2f | %-6s %4d | %s | %7.3f %7.4f | "
                "%6.2f | %s\n",
                r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                outcome, scored ? e.slot : -1, score,
                1e3 * r.disturbance.worst_displacement,
                r.disturbance.worst_rotation / kDegree, r.cycle_time,
                r.reason.c_str());
  } else {
    // The reason may hold commas; quote it.
    std::string reason = r.reason;
    std::replace(reason.begin(), reason.end(), '"', '\'');
    if (scored) {
      std::printf("%u,%d,%.3f,%.3f,%.4f,%s,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%d,"
                  "%.4f,%.5f,%.3f,\"%s\"\n",
                  r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                  outcome, e.slot, 1e3 * e.along_tube, 1e3 * e.across_tube,
                  1e3 * e.height, e.yaw / kDegree, e.tilt / kDegree,
                  e.at_rest ? 1 : 0, 1e3 * r.disturbance.worst_displacement,
                  r.disturbance.worst_rotation / kDegree, r.cycle_time,
                  reason.c_str());
    } else {
      std::printf("%u,%d,%.3f,%.3f,%.4f,%s,,,,,,,,%.4f,%.5f,%.3f,\"%s\"\n",
                  r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                  outcome, 1e3 * r.disturbance.worst_displacement,
                  r.disturbance.worst_rotation / kDegree, r.cycle_time,
                  reason.c_str());
    }
  }
}

// One line per step of the script's last cycle: how long it took and how it
// ended. `prefix` starts each line ("# " keeps CSV readers off them).
void PrintPhases(const cell::PickPlace& script, std::FILE* out,
                 const char* prefix = "  ") {
  for (const cell::StepRecord& r : script.records()) {
    std::fprintf(out, "%s%-14s %7.3f s  from %7.3f  %s\n", prefix,
                 cell::StepName(r.step), r.duration(), r.start,
                 r.finished ? r.result.c_str() : "(running)");
  }
}

// The kind of a failure, for the summary: its reason up to the first colon.
std::string Kind(const std::string& reason) {
  return reason.substr(0, reason.find(':'));
}

}  // namespace

int main(int argc, char** argv) {
  const char* usage =
      "usage: %s <cell.xml> [--pretty] [--seed N] [--modules M] [--trials T] "
      "[--first K] [--nominal] [--xy MM] [--yaw DEG] [--phases] [--verbose]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  bool pretty = false, nominal = false, verbose = false, phases = false;
  long first_seed = 1;
  int modules = 1, trials = 1, first = 0;
  std::optional<double> xy, yaw;  // ranges, mm and degrees
  for (int i = 2; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--pretty") == 0) {
      pretty = true;
    } else if (std::strcmp(argv[i], "--nominal") == 0) {
      nominal = true;
    } else if (std::strcmp(argv[i], "--phases") == 0) {
      phases = true;
    } else if (std::strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (std::strcmp(argv[i], "--seed") == 0 && has_value) {
      first_seed = std::atol(argv[++i]);
    } else if (std::strcmp(argv[i], "--modules") == 0 && has_value) {
      modules = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--first") == 0 && has_value) {
      first = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--xy") == 0 && has_value) {
      xy = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--yaw") == 0 && has_value) {
      yaw = std::atof(argv[++i]);
    } else if (std::strcmp(argv[i], "--trials") == 0 && has_value) {
      trials = std::atoi(argv[++i]);
    } else {
      std::fprintf(stderr, usage, argv[0]);
      return 1;
    }
  }

  std::string error;
  std::optional<cell::Cell> loaded = cell::Cell::Load(argv[1], &error);
  if (!loaded) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  cell::Cell& cell = *loaded;
  if (modules < 1 || first < 0 || first + modules > cell.module_count() ||
      trials < 1 ||
      first_seed < 0) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  cell::Randomization randomization =
      nominal ? cell::Randomization::None() : cell::Randomization{};
  if (xy) randomization.pallet_xy = *xy * 1e-3;
  if (yaw) randomization.pallet_yaw = *yaw * kDegree;

  PrintHeader(pretty);
  std::vector<Row> rows;
  double simulated = 0.0;
  const auto wall_start = std::chrono::steady_clock::now();

  for (int t = 0; t < trials; ++t) {
    const auto seed = static_cast<std::uint32_t>(first_seed + t);
    if (!cell.Reset(seed, randomization)) {
      std::fprintf(stderr, "--xy and --yaw must not be negative\n");
      return 1;
    }
    if (first > 0) {
      for (int i = 0; i < first; ++i) cell.ParkModule(i);
      cell.StepFor(cell::Cell::kSettleTime);
      mj_forward(cell.model(), cell.data());
      cell.RebaselineStack();
    }
    const cell::TrueScene& scene = cell.true_scene();
    cell::PickPlace script(cell);
    std::optional<cell::PlacementError> placement;
    cell::StackDisturbance disturbance;
    script.set_placed_hook([&](int module, int slot) {
      // Score as the module lies now, before the tube moves on.
      mj_forward(cell.model(), cell.data());
      placement = cell::MeasurePlacement(cell, module, slot);
      disturbance = cell::MeasureStackDisturbance(cell);
      cell.RebaselineStack();
    });

    for (int m = first; m < first + modules; ++m) {
      Row row;
      row.seed = seed;
      row.module = m;
      row.dx = scene.pallet_x;
      row.dy = scene.pallet_y;
      row.yaw = scene.pallet_yaw;
      placement.reset();
      cell::CycleCheck check(cell);
      const double start = cell.data()->time;
      script.Start(m);
      while (!script.Done() && cell.data()->time - start < kCycleLimit) {
        script.Update();
        cell.Step();
        check.Observe(script.step(), m);
      }
      const cell::PickPlaceOutcome& outcome = script.outcome();
      row.placed = outcome.status == cell::PickPlaceOutcome::Status::kPlaced;
      if (!script.Done()) {
        row.reason = std::string("timed out in ") + script.step_name();
      } else if (!row.placed) {
        row.reason = outcome.reason;
      }
      row.cycle_time = cell.data()->time - start;
      if (row.placed) {
        row.error = placement;
        row.disturbance = disturbance;
      } else {
        // Let things come to rest, the arm holding, then see what the
        // failed attempt did to the stack.
        const double until = cell.data()->time + kSettleAfterFailure;
        while (cell.data()->time < until) {
          script.Update();
          cell.Step();
        }
        mj_forward(cell.model(), cell.data());
        row.disturbance = cell::MeasureStackDisturbance(cell);
      }
      if (verbose) {
        std::fprintf(stderr, "seed %u module %d: %s%s%s\n", seed, m,
                     row.placed ? "placed" : "failed",
                     row.reason.empty() ? "" : ": ", row.reason.c_str());
        PrintPhases(script, stderr);
        std::fprintf(stderr, "  planned waypoints, degrees, joints 1 to 6:\n");
        for (const cell::PlannedMove& move : script.planned()) {
          for (const Eigen::VectorXd& q : move.waypoints) {
            std::fprintf(stderr,
                         "    %-14s %8.2f %8.2f %8.2f %8.2f %8.2f %8.2f  "
                         "speed %.2f\n",
                         cell::StepName(move.step), q[0] / kDegree,
                         q[1] / kDegree, q[2] / kDegree, q[3] / kDegree,
                         q[4] / kDegree, q[5] / kDegree, move.speed);
          }
        }
        std::fprintf(stderr, "%s", check.Report(true).c_str());
        if (row.error) {
          std::fprintf(stderr, "  %s\n", cell::Describe(*row.error).c_str());
        }
      }
      PrintRow(row, pretty);
      if (phases || (pretty && !row.placed)) {
        PrintPhases(script, stdout, pretty ? "        " : "# ");
      }
      std::fflush(stdout);
      rows.push_back(row);
      if (!row.placed) break;
    }
    simulated += cell.data()->time;
  }

  // Summary.
  const double wall = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - wall_start)
                          .count();
  int placed = 0;
  double worst_along = 0.0, worst_across = 0.0, worst_height = 0.0,
         worst_yaw = 0.0, worst_tilt = 0.0, total_time = 0.0;
  std::map<std::string, int> failures;
  for (const Row& r : rows) {
    if (r.placed && r.error) {
      ++placed;
      worst_along = std::max(worst_along, std::abs(r.error->along_tube));
      worst_across = std::max(worst_across, std::abs(r.error->across_tube));
      worst_height = std::max(worst_height, std::abs(r.error->height));
      worst_yaw = std::max(worst_yaw, std::abs(r.error->yaw));
      worst_tilt = std::max(worst_tilt, std::abs(r.error->tilt));
      total_time += r.cycle_time;
    } else if (!r.placed) {
      ++failures[Kind(r.reason)];
    }
  }
  std::printf("%s%d of %zu cycles placed", pretty ? "\n" : "# ", placed,
              rows.size());
  if (placed > 0) {
    std::printf("; worst |error| along %.2f mm, across %.2f mm, height %.2f "
                "mm, yaw %.3f deg, tilt %.3f deg; mean cycle %.2f s",
                1e3 * worst_along, 1e3 * worst_across, 1e3 * worst_height,
                worst_yaw / kDegree, worst_tilt / kDegree, total_time / placed);
  }
  std::printf("\n");
  for (const auto& [kind, count] : failures) {
    std::printf("%sfailed %d: %s\n", pretty ? "" : "# ", count, kind.c_str());
  }
  std::fprintf(stderr, "simulated %.1f s in %.1f s of wall time: %.2fx real time\n",
               simulated, wall, wall > 0.0 ? simulated / wall : 0.0);
  return 0;
}
