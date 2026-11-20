// Runs the pick and place (src/pick_place.h) headless and scores it: for
// each trial, resets the cell, picks and places modules one after another,
// and prints one row per module. The output is CSV, for a plotting tool;
// with --pretty it is a table of aligned columns, for reading in a terminal.
//
//   bazel run --config=opt //:run_cell -- --pretty
//   bazel run --config=opt //:run_cell -- --pretty --trials 20
//   bazel run --config=opt //:run_cell -- --pretty --blind --trials 20
//   bazel run --config=opt //:run_cell -- --pretty --trials 20 --carry planned
//   bazel run //:run_cell -- [--pretty] [--seed N] [--modules M] [--phases]
//                            [--trials T] [--nominal] [--blind] [--no-noise]
//                            [--calibration MM,DEG] [--carry planned|scripted]
//                            [--verbose]
//
//   --camera     (the default) one depth picture per pick, and the pick aimed
//                at the module the finder sees there
//   --blind      no camera: the pick aimed at the design's poses, the stage-3
//                baseline
//   --noise      (the default) the camera's own errors in every picture:
//                depth noise, edge effects, whole-millimetre readings; the
//                finder was measured with them on, and a real camera has them
//   --no-noise   exact pictures
//   --calibration MM,DEG
//                the camera's calibration off by a shift of MM millimetres
//                and a turn of DEG degrees, in directions drawn for each seed
//                (default 0,0: exact)
//   --carry scripted|planned
//                the carry from the end of "clear" to over the slot: the
//                three scripted joint moves (the default), or one move
//                planned by the motion planner (src/pick_place.h)
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
// The script never reads the truth; this program does. The pallet's true
// offset, the scores (src/score.h) and the finder's error against where the
// module truly stood come from the truth, and are only printed. The finder's
// error is in the true module's frame: along its glass's normal (+ toward
// the arm), along its long side, up the lean, the angle between the two
// glass normals and the turn about the normal; with the finder's own
// computing time. Then the carry: its simulated time (end of clear to the
// end of the turn or of the planned carry); for the planned carry the
// planner's wall time, a "*" if that did not fit in the lift, pull back and
// clear it ran alongside, and the clearance the planner measured along its
// path; and the highest motor torque during the carry, as a fraction of its
// limit (from the truth, cycle_check; the whole cycle's in the summary). The ratio of simulated to wall-clock
// time goes to stderr.
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
  // Camera mode: the finder's error for the estimate the pick used, and its
  // computing time.
  std::optional<cell::EstimateError> seen;
  double finder_seconds = 0.0;
  // The carry (PickPlaceOutcome::carry), and the cycle's highest motor
  // torque over its limit.
  cell::CarryReport carry;
  double peak_torque = 0.0;
  double carry_torque = 0.0;  // the same during the carry's steps only
};

void PrintHeader(bool pretty) {
  if (pretty) {
    std::printf("%5s %3s | %7s %7s %6s | %-6s %4s | %7s %7s %7s %7s %6s %4s | "
                "%7s %7s | %6s | %6s %6s %6s %6s %6s %4s | %5s %6s %5s %5s | "
                "%s\n",
                "seed", "mod", "dx mm", "dy mm", "yaw d", "result", "slot",
                "along", "across", "height", "yaw d", "tilt d", "rest",
                "stack", "turn d", "cycle", "seen n", "long", "up",
                "face d", "rot d", "ms", "carry", "plan", "clr", "ctorq",
                "reason");
  } else {
    std::printf("seed,module,pallet_dx_mm,pallet_dy_mm,pallet_yaw_deg,outcome,"
                "slot,along_mm,across_mm,height_mm,yaw_deg,tilt_deg,at_rest,"
                "stack_moved_mm,stack_turned_deg,cycle_s,seen_normal_mm,"
                "seen_long_mm,seen_up_mm,seen_face_deg,seen_turn_deg,"
                "finder_ms,carry_s,plan_ms,plan_fits,plan_clearance_mm,"
                "carry_torque,peak_torque,reason\n");
  }
}

// The finder's columns, or blanks.
std::string Seen(const Row& r, bool pretty) {
  char text[160];
  if (!r.seen) {
    if (pretty) {
      std::snprintf(text, sizeof(text), "%6s %6s %6s %6s %6s %4s", "-", "-",
                    "-", "-", "-", "-");
    } else {
      std::snprintf(text, sizeof(text), ",,,,,");
    }
    return text;
  }
  const cell::EstimateError& e = *r.seen;
  std::snprintf(text, sizeof(text),
                pretty ? "%6.2f %6.2f %6.2f %6.3f %6.3f %4.0f"
                       : "%.4f,%.4f,%.4f,%.5f,%.5f,%.1f",
                1e3 * e.normal, 1e3 * e.long_side, 1e3 * e.up_lean,
                e.face_angle / kDegree, e.in_plane / kDegree,
                1e3 * r.finder_seconds);
  return text;
}

// The carry's columns: its time, and for a planned carry the planner's wall
// time (ms, "*" when it did not fit alongside lift, pull back and clear) and
// its path's clearance (mm); the carry's peak torque fraction (the whole
// cycle's goes to the summary).
std::string Carry(const Row& r, bool pretty) {
  const cell::CarryReport& c = r.carry;
  char plan[32] = "", fits[8] = "", clearance[32] = "";
  if (c.attempted) {
    std::snprintf(plan, sizeof(plan), pretty ? "%5.0f" : "%.1f",
                  1e3 * c.plan_seconds);
    std::snprintf(fits, sizeof(fits), "%s",
                  c.fits() ? (pretty ? " " : "1") : (pretty ? "*" : "0"));
  } else if (pretty) {
    std::snprintf(plan, sizeof(plan), "%5s", "-");
    std::snprintf(fits, sizeof(fits), " ");
  }
  if (c.found) {
    std::snprintf(clearance, sizeof(clearance), pretty ? "%5.0f" : "%.1f",
                  1e3 * c.clearance);
  } else if (pretty) {
    std::snprintf(clearance, sizeof(clearance), "%5s", "-");
  }
  char text[160];
  if (pretty) {
    std::snprintf(text, sizeof(text), "%5.2f %s%s %s %5.3f", c.seconds, plan,
                  fits, clearance, r.carry_torque);
  } else {
    std::snprintf(text, sizeof(text), "%.3f,%s,%s,%s,%.4f,%.4f", c.seconds,
                  plan, fits, clearance, r.carry_torque, r.peak_torque);
  }
  return text;
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
                "%6.2f | %s | %s | %s\n",
                r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                outcome, scored ? e.slot : -1, score,
                1e3 * r.disturbance.worst_displacement,
                r.disturbance.worst_rotation / kDegree, r.cycle_time,
                Seen(r, true).c_str(), Carry(r, true).c_str(),
                r.reason.c_str());
  } else {
    // The reason may hold commas; quote it.
    std::string reason = r.reason;
    std::replace(reason.begin(), reason.end(), '"', '\'');
    if (scored) {
      std::printf("%u,%d,%.3f,%.3f,%.4f,%s,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%d,"
                  "%.4f,%.5f,%.3f,%s,%s,\"%s\"\n",
                  r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                  outcome, e.slot, 1e3 * e.along_tube, 1e3 * e.across_tube,
                  1e3 * e.height, e.yaw / kDegree, e.tilt / kDegree,
                  e.at_rest ? 1 : 0, 1e3 * r.disturbance.worst_displacement,
                  r.disturbance.worst_rotation / kDegree, r.cycle_time,
                  Seen(r, false).c_str(), Carry(r, false).c_str(),
                  reason.c_str());
    } else {
      std::printf("%u,%d,%.3f,%.3f,%.4f,%s,,,,,,,,%.4f,%.5f,%.3f,%s,%s,"
                  "\"%s\"\n",
                  r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                  outcome, 1e3 * r.disturbance.worst_displacement,
                  r.disturbance.worst_rotation / kDegree, r.cycle_time,
                  Seen(r, false).c_str(), Carry(r, false).c_str(),
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
      "[--first K] [--nominal] [--xy MM] [--yaw DEG] [--camera | --blind] "
      "[--noise | --no-noise] [--calibration MM,DEG] "
      "[--carry scripted|planned] [--phases] [--verbose]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  bool pretty = false, nominal = false, verbose = false, phases = false;
  bool camera = true, noise = true;
  cell::CarryMode carry = cell::CarryMode::kScripted;
  double calibration_mm = 0.0, calibration_deg = 0.0;
  long first_seed = 1;
  int modules = 1, trials = 1, first = 0;
  std::optional<double> xy, yaw;  // ranges, mm and degrees
  for (int i = 2; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--pretty") == 0) {
      pretty = true;
    } else if (std::strcmp(argv[i], "--nominal") == 0) {
      nominal = true;
    } else if (std::strcmp(argv[i], "--camera") == 0) {
      camera = true;
    } else if (std::strcmp(argv[i], "--blind") == 0) {
      camera = false;
    } else if (std::strcmp(argv[i], "--noise") == 0) {
      noise = true;
    } else if (std::strcmp(argv[i], "--no-noise") == 0) {
      noise = false;
    } else if (std::strcmp(argv[i], "--calibration") == 0 && has_value) {
      if (std::sscanf(argv[++i], "%lf,%lf", &calibration_mm,
                      &calibration_deg) != 2) {
        std::fprintf(stderr, usage, argv[0]);
        return 1;
      }
    } else if (std::strcmp(argv[i], "--carry") == 0 && has_value) {
      const char* mode = argv[++i];
      if (std::strcmp(mode, "planned") == 0) {
        carry = cell::CarryMode::kPlanned;
      } else if (std::strcmp(mode, "scripted") == 0) {
        carry = cell::CarryMode::kScripted;
      } else {
        std::fprintf(stderr, usage, argv[0]);
        return 1;
      }
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
  randomization.camera_noise = camera && noise;
  randomization.camera_shift = calibration_mm * 1e-3;
  randomization.camera_turn = calibration_deg * kDegree;
  cell::PickPlaceOptions options;
  options.sensing =
      camera ? cell::SensingMode::kCamera : cell::SensingMode::kBlind;
  options.carry = carry;

  PrintHeader(pretty);
  std::vector<Row> rows;
  double simulated = 0.0;
  const auto wall_start = std::chrono::steady_clock::now();

  for (int t = 0; t < trials; ++t) {
    const auto seed = static_cast<std::uint32_t>(first_seed + t);
    if (!cell.Reset(seed, randomization)) {
      std::fprintf(stderr,
                   "--xy, --yaw and --calibration must not be negative\n");
      return 1;
    }
    if (first > 0) {
      for (int i = 0; i < first; ++i) cell.ParkModule(i);
      cell.StepFor(cell::Cell::kSettleTime);
      mj_forward(cell.model(), cell.data());
      cell.RebaselineStack();
    }
    const cell::TrueScene& scene = cell.true_scene();
    cell::PickPlace script(cell, options);
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
        // The finder's error, from the truth, as soon as the pick has its
        // estimate: the module still stands where it was seen.
        if (!row.seen && script.estimate()) {
          row.seen = cell::EstimateErrorOf(script.estimate()->pose,
                                           cell.TrueModulePose(m));
          row.finder_seconds =
              script.estimate()->diagnostics.seconds_total;
        }
      }
      const cell::PickPlaceOutcome& outcome = script.outcome();
      row.placed = outcome.status == cell::PickPlaceOutcome::Status::kPlaced;
      if (!script.Done()) {
        row.reason = std::string("timed out in ") + script.step_name();
      } else if (!row.placed) {
        row.reason = outcome.reason;
      }
      row.cycle_time = cell.data()->time - start;
      row.carry = outcome.carry;
      row.peak_torque = check.peak_torque().maxCoeff();
      for (cell::PickPlaceStep s :
           {cell::PickPlaceStep::kFlatten, cell::PickPlaceStep::kSwing,
            cell::PickPlaceStep::kTurn, cell::PickPlaceStep::kCarry}) {
        row.carry_torque =
            std::max(row.carry_torque, check.peak_torque(s).maxCoeff());
      }
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
  // Worst and root-mean-square of each column, over the placed cycles (and
  // over every cycle with an estimate, for the finder's).
  struct Stat {
    double worst = 0.0, sum = 0.0, sum_squares = 0.0;
    int count = 0;
    void Add(double v) {
      worst = std::max(worst, std::abs(v));
      sum += v;
      sum_squares += v * v;
      ++count;
    }
    double Mean() const { return count ? sum / count : 0.0; }
    double Rms() const { return count ? std::sqrt(sum_squares / count) : 0.0; }
  };
  Stat along, across, height, yaw_error, tilt, stack, cycle;
  Stat carry_time, plan_time, torque, carry_torque;
  double clearance = 1e9;
  std::string clearance_between;
  int planned = 0, fits = 0;
  Stat seen_normal, seen_long, seen_up, seen_face, seen_turn, finder;
  int placed = 0;
  std::map<std::string, int> failures;
  for (const Row& r : rows) {
    if (r.placed && r.error) {
      ++placed;
      along.Add(1e3 * r.error->along_tube);
      across.Add(1e3 * r.error->across_tube);
      height.Add(1e3 * r.error->height);
      yaw_error.Add(r.error->yaw / kDegree);
      tilt.Add(r.error->tilt / kDegree);
      cycle.Add(r.cycle_time);
      carry_time.Add(r.carry.seconds);
    } else if (!r.placed) {
      ++failures[Kind(r.reason)];
    }
    stack.Add(1e3 * r.disturbance.worst_displacement);
    torque.Add(r.peak_torque);
    carry_torque.Add(r.carry_torque);
    if (r.carry.attempted) {
      ++planned;
      if (r.carry.fits()) ++fits;
      plan_time.Add(1e3 * r.carry.plan_seconds);
    }
    if (r.carry.found && r.carry.clearance < clearance) {
      clearance = r.carry.clearance;
      clearance_between = r.carry.clearance_between;
    }
    if (r.seen) {
      seen_normal.Add(1e3 * r.seen->normal);
      seen_long.Add(1e3 * r.seen->long_side);
      seen_up.Add(1e3 * r.seen->up_lean);
      seen_face.Add(r.seen->face_angle / kDegree);
      seen_turn.Add(r.seen->in_plane / kDegree);
      finder.Add(1e3 * r.finder_seconds);
    }
  }
  const char* lead = pretty ? "" : "# ";
  std::printf("%s%s%d of %zu cycles placed (%s%s%s, %s carry)\n",
              pretty ? "\n" : "", lead, placed, rows.size(),
              camera ? "camera" : "blind",
              camera ? (noise ? ", noisy pictures" : ", exact pictures") : "",
              calibration_mm > 0.0 || calibration_deg > 0.0
                  ? ", calibration off"
                  : "",
              cell::CarryModeName(carry));
  if (placed > 0) {
    std::printf("%splacement, worst / rms: along %.2f / %.2f mm, across %.2f "
                "/ %.2f mm, height %.2f / %.2f mm, yaw %.3f / %.3f deg, tilt "
                "%.3f / %.3f deg\n",
                lead, along.worst, along.Rms(), across.worst, across.Rms(),
                height.worst, height.Rms(), yaw_error.worst, yaw_error.Rms(),
                tilt.worst, tilt.Rms());
    std::printf("%scycle: mean %.2f s, longest %.2f s; carry: mean %.2f s, "
                "longest %.2f s\n",
                lead, cycle.Mean(), cycle.worst, carry_time.Mean(),
                carry_time.worst);
  }
  if (planned > 0) {
    std::printf("%splanning (wall): mean %.0f ms, worst %.0f ms; fits in lift, "
                "pull back and clear: %d of %d\n",
                lead, plan_time.Mean(), plan_time.worst, fits, planned);
  }
  if (clearance < 1e9) {
    std::printf("%splanned path clearance, smallest: %.1f mm (%s)\n", lead,
                1e3 * clearance, clearance_between.c_str());
  }
  std::printf("%speak torque fraction: whole cycle worst %.3f, mean %.3f; "
              "carry worst %.3f, mean %.3f\n",
              lead, torque.worst, torque.Mean(), carry_torque.worst,
              carry_torque.Mean());
  std::printf("%sstack disturbance, worst / rms: %.3f / %.3f mm\n", lead,
              stack.worst, stack.Rms());
  if (seen_normal.count > 0) {
    std::printf("%sfinder error, worst / rms: normal %.2f / %.2f mm, long side "
                "%.2f / %.2f mm, up the lean %.2f / %.2f mm, face %.3f / %.3f "
                "deg, turn %.3f / %.3f deg; finder %.0f ms at most\n",
                lead, seen_normal.worst, seen_normal.Rms(), seen_long.worst,
                seen_long.Rms(), seen_up.worst, seen_up.Rms(), seen_face.worst,
                seen_face.Rms(), seen_turn.worst, seen_turn.Rms(),
                finder.worst);
  }
  for (const auto& [kind, count] : failures) {
    std::printf("%sfailed %d: %s\n", pretty ? "" : "# ", count, kind.c_str());
  }
  std::fprintf(stderr, "simulated %.1f s in %.1f s of wall time: %.2fx real time\n",
               simulated, wall, wall > 0.0 ? simulated / wall : 0.0);
  return 0;
}
