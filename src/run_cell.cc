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
//                            [--planner own|ompl]
//                            [--check] [--check-calibration MM,DEG]
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
//   --planner own|ompl
//                which search plans the planned carry: the hand-written
//                RRT-Connect of src/rrt.h (own, the default) or OMPL's
//                (src/ompl_search.h). It matters only with --carry planned
//                or --check.
//   --check      the check after pickup (src/pick_place.h): the planned
//                carry (implied) through the check pose over the check
//                camera, where the grip is measured and the place corrected
//                for it. Off by default.
//   --check-calibration MM,DEG
//                the check camera's calibration off by MM millimetres and
//                DEG degrees, as --calibration for the pallet camera
//                (default 0,0: exact)
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
//   --save-failures DIR
//                judge each trial (src/trial_runner.h: PlacementTolerance,
//                3 mm in the rails' plane, 0.15 degrees of yaw and of tilt)
//                and write one file per trial that does not pass into DIR,
//                named for its seed and first bad module, with what reruns
//                it; a line on stderr for each
//   --replay FILE
//                rerun the trial of a file --save-failures (or the gate)
//                wrote, up to its first bad cycle, print its table, and say
//                whether it failed the same way, bit for bit (exit status 2
//                if not). Replaces the trial flags.
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
// limit (from the truth, cycle_check; the whole cycle's in the summary).
// Then, with --check: how the check went ("ok" measured, "ok2" measured on
// the retake after a refused first picture, "REF" refused, "SLIP" slipped;
// both failed), the measured grip's
// offset from the design grip (shift and turn, in all), the finder's
// computing time over its pictures, the measured grip's error against the
// true grip (the suction weld's pose in the tool tip's frame at the moment
// of the picture, from the truth, printed only), and "r" if leg 2 was
// replanned with the measured grip. A retaken cycle's reason names the
// first picture's refusal. The ratio of simulated to wall-clock time goes
// to stderr.
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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/cell.h"
#include "src/cycle_check.h"
#include "src/held_module_finder.h"
#include "src/pick_place.h"
#include "src/score.h"
#include "src/suction.h"
#include "src/trial_runner.h"

namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

// One row of the table: one cycle of a trial, as the trial runner
// (src/trial_runner.h) returns it.
using Row = cell::CycleResult;

bool Retaken(const Row& r) {
  return r.check.status == cell::CheckReport::Status::kChecked &&
         r.check.retaken;
}

void PrintHeader(bool pretty, bool check) {
  if (pretty) {
    std::printf("%5s %3s | %7s %7s %6s | %-6s %4s | %7s %7s %7s %7s %6s %4s | "
                "%7s %7s | %6s | %6s %6s %6s %6s %6s %4s | %5s %6s %5s %5s | ",
                "seed", "mod", "dx mm", "dy mm", "yaw d", "result", "slot",
                "along", "across", "height", "yaw d", "tilt d", "rest",
                "stack", "turn d", "cycle", "seen n", "long", "up",
                "face d", "rot d", "ms", "carry", "plan", "clr", "ctorq");
    if (check) {
      std::printf("%4s %6s %6s %4s %6s %6s %2s | ", "chk", "off mm", "off d",
                  "ms", "err mm", "err d", "rp");
    }
    std::printf("%s\n", "reason");
  } else {
    std::printf("seed,module,pallet_dx_mm,pallet_dy_mm,pallet_yaw_deg,outcome,"
                "slot,along_mm,across_mm,height_mm,yaw_deg,tilt_deg,at_rest,"
                "stack_moved_mm,stack_turned_deg,cycle_s,seen_normal_mm,"
                "seen_long_mm,seen_up_mm,seen_face_deg,seen_turn_deg,"
                "finder_ms,carry_s,plan_ms,plan_fits,plan_clearance_mm,"
                "carry_torque,peak_torque,check,check_pictures,"
                "grip_x_mm,grip_y_mm,grip_z_mm,grip_rx_deg,grip_ry_deg,"
                "grip_rz_deg,check_finder_ms,check_error_mm,check_error_deg,"
                "leg2_replanned,replan_ms,reason\n");
  }
}

// The check's columns: status, the measured grip's offset from the design
// (in all), the finder's time, its error against the truth, replanned.
std::string Check(const Row& r, bool pretty) {
  const cell::CheckReport& c = r.check;
  const bool measured = c.status == cell::CheckReport::Status::kChecked ||
                        c.status == cell::CheckReport::Status::kNoPath;
  const char* status = "-";
  switch (c.status) {
    case cell::CheckReport::Status::kOff:
      status = "-";
      break;
    case cell::CheckReport::Status::kNotRun:
      status = pretty ? "-" : "not run";
      break;
    case cell::CheckReport::Status::kChecked:
      status = pretty ? (c.retaken ? "ok2" : "ok") : "checked";
      break;
    case cell::CheckReport::Status::kRefused:
      // IMPL: a measurement the cups could not have produced (tilt, gap).
      status = pretty ? (c.implausible ? "IMPL" : "REF")
                      : (c.implausible ? "implausible" : "refused");
      break;
    case cell::CheckReport::Status::kNoPath:
      status = pretty ? "NOP" : "no path";
      break;
    case cell::CheckReport::Status::kSlipped:
      status = pretty ? "SLIP" : "slipped";
      break;
  }
  char text[300];
  if (pretty) {
    char off[40], err[40];
    if (measured) {
      std::snprintf(off, sizeof(off), "%6.2f %6.3f",
                    1e3 * c.offset_shift.norm(),
                    c.offset_rotation.norm() / kDegree);
    } else {
      std::snprintf(off, sizeof(off), "%6s %6s", "-", "-");
    }
    if (r.check_error_shift) {
      std::snprintf(err, sizeof(err), "%6.3f %6.4f", 1e3 * *r.check_error_shift,
                    *r.check_error_turn / kDegree);
    } else {
      std::snprintf(err, sizeof(err), "%6s %6s", "-", "-");
    }
    char ms[16];
    if (c.pictures > 0) {
      std::snprintf(ms, sizeof(ms), "%4.0f", 1e3 * c.finder_seconds);
    } else {
      std::snprintf(ms, sizeof(ms), "%4s", "-");
    }
    std::snprintf(text, sizeof(text), "%4s %s %s %s %2s", status, off, ms,
                  err, c.replanned ? "r" : "");
    return text;
  }
  char off[160] = ",,,,,";
  if (measured) {
    std::snprintf(off, sizeof(off), "%.4f,%.4f,%.4f,%.5f,%.5f,%.5f",
                  1e3 * c.offset_shift.x(), 1e3 * c.offset_shift.y(),
                  1e3 * c.offset_shift.z(), c.offset_rotation.x() / kDegree,
                  c.offset_rotation.y() / kDegree,
                  c.offset_rotation.z() / kDegree);
  }
  char err[64] = ",";
  if (r.check_error_shift) {
    std::snprintf(err, sizeof(err), "%.4f,%.5f", 1e3 * *r.check_error_shift,
                  *r.check_error_turn / kDegree);
  }
  std::snprintf(text, sizeof(text), "%s,%d,%s,%.1f,%s,%d,%.1f", status,
                c.pictures, off, 1e3 * c.finder_seconds, err,
                c.replanned ? 1 : 0, 1e3 * c.replan_seconds);
  return text;
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

void PrintRow(const Row& r, bool pretty, bool check) {
  const cell::PlacementError e = r.error.value_or(cell::PlacementError{});
  const bool scored = r.error.has_value();
  const char* outcome = !r.placed ? "failed" : "placed";
  // CSV always has the check's columns; the table only with --check.
  const std::string check_columns =
      pretty ? (check ? Check(r, true) + " | " : std::string())
             : Check(r, false) + ",";
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
                "%6.2f | %s | %s | %s%s\n",
                r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                outcome, scored ? e.slot : -1, score,
                1e3 * r.disturbance.worst_displacement,
                r.disturbance.worst_rotation / kDegree, r.cycle_time,
                Seen(r, true).c_str(), Carry(r, true).c_str(),
                check_columns.c_str(), r.reason.c_str());
  } else {
    // The reason may hold commas; quote it.
    std::string reason = r.reason;
    std::replace(reason.begin(), reason.end(), '"', '\'');
    if (scored) {
      std::printf("%u,%d,%.3f,%.3f,%.4f,%s,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%d,"
                  "%.4f,%.5f,%.3f,%s,%s,%s\"%s\"\n",
                  r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                  outcome, e.slot, 1e3 * e.along_tube, 1e3 * e.across_tube,
                  1e3 * e.height, e.yaw / kDegree, e.tilt / kDegree,
                  e.at_rest ? 1 : 0, 1e3 * r.disturbance.worst_displacement,
                  r.disturbance.worst_rotation / kDegree, r.cycle_time,
                  Seen(r, false).c_str(), Carry(r, false).c_str(),
                  check_columns.c_str(), reason.c_str());
    } else {
      std::printf("%u,%d,%.3f,%.3f,%.4f,%s,,,,,,,,%.4f,%.5f,%.3f,%s,%s,%s"
                  "\"%s\"\n",
                  r.seed, r.module, 1e3 * r.dx, 1e3 * r.dy, r.yaw / kDegree,
                  outcome, 1e3 * r.disturbance.worst_displacement,
                  r.disturbance.worst_rotation / kDegree, r.cycle_time,
                  Seen(r, false).c_str(), Carry(r, false).c_str(),
                  check_columns.c_str(), reason.c_str());
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

// After each cycle: with --verbose its steps, planned waypoints and check
// on stderr; its row; its steps after the row with --phases (always for a
// failed cycle with --pretty).
void PrintCycle(const Row& row, const cell::PickPlace& script,
                const cell::CycleCheck& check, bool pretty, bool with_check,
                bool phases, bool verbose) {
  if (verbose) {
    std::fprintf(stderr, "seed %u module %d: %s%s%s\n", row.seed, row.module,
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
  PrintRow(row, pretty, with_check);
  if (phases || (pretty && !row.placed)) {
    PrintPhases(script, stdout, pretty ? "        " : "# ");
  }
  std::fflush(stdout);
}

// A path given on the command line, relative to where `bazel run` was
// started (it runs the program in its runfiles).
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
      "usage: %s <cell.xml> [--pretty] [--seed N] [--modules M] [--trials T] "
      "[--first K] [--nominal] [--xy MM] [--yaw DEG] [--camera | --blind] "
      "[--noise | --no-noise] [--calibration MM,DEG] "
      "[--carry scripted|planned] [--planner own|ompl] [--check] "
      "[--check-calibration MM,DEG] "
      "[--phases] [--verbose] [--save-failures DIR] [--replay FILE]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  bool pretty = false, nominal = false, verbose = false, phases = false;
  bool camera = true, noise = true, with_check = false;
  cell::CarryMode carry = cell::CarryMode::kScripted;
  cell::PlanSearch planner = cell::PlanSearch::kOwn;
  double calibration_mm = 0.0, calibration_deg = 0.0;
  double check_calibration_mm = 0.0, check_calibration_deg = 0.0;
  long first_seed = 1;
  int modules = 1, trials = 1, first = 0;
  std::optional<double> xy, yaw;  // ranges, mm and degrees
  std::string save_failures, replay;  // directory, file
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
    } else if (std::strcmp(argv[i], "--planner") == 0 && has_value) {
      const std::optional<cell::PlanSearch> search =
          cell::PlanSearchFromName(argv[++i]);
      if (!search) {
        std::fprintf(stderr, usage, argv[0]);
        return 1;
      }
      planner = *search;
    } else if (std::strcmp(argv[i], "--check") == 0) {
      with_check = true;
    } else if (std::strcmp(argv[i], "--check-calibration") == 0 && has_value) {
      if (std::sscanf(argv[++i], "%lf,%lf", &check_calibration_mm,
                      &check_calibration_deg) != 2) {
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
    } else if (std::strcmp(argv[i], "--save-failures") == 0 && has_value) {
      save_failures = FromWorkingDirectory(argv[++i]);
    } else if (std::strcmp(argv[i], "--replay") == 0 && has_value) {
      replay = FromWorkingDirectory(argv[++i]);
    } else {
      std::fprintf(stderr, usage, argv[0]);
      return 1;
    }
  }

  // A replay takes the trial from the file, whatever the flags said.
  std::optional<cell::FailureRecord> replayed;
  if (!replay.empty()) {
    std::string load_error;
    replayed = cell::LoadFailure(replay, &load_error);
    if (!replayed) {
      std::fprintf(stderr, "%s\n", load_error.c_str());
      return 1;
    }
    const cell::TrialSettings s = cell::ReplaySettings(*replayed);
    first_seed = s.seed;
    trials = 1;
    modules = s.modules;
    first = s.first;
    nominal = s.nominal;
    xy = s.xy_mm;
    yaw = s.yaw_deg;
    camera = s.camera;
    noise = s.noise;
    calibration_mm = s.calibration_mm;
    calibration_deg = s.calibration_deg;
    carry = s.carry;
    planner = s.planner;
    with_check = s.check;
    check_calibration_mm = s.check_calibration_mm;
    check_calibration_deg = s.check_calibration_deg;
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
  cell::TrialSettings settings;
  settings.modules = modules;
  settings.first = first;
  settings.nominal = nominal;
  settings.xy_mm = xy;
  settings.yaw_deg = yaw;
  settings.camera = camera;
  settings.noise = noise;
  settings.calibration_mm = calibration_mm;
  settings.calibration_deg = calibration_deg;
  settings.carry = carry;
  settings.planner = planner;
  settings.check = with_check;
  settings.check_calibration_mm = check_calibration_mm;
  settings.check_calibration_deg = check_calibration_deg;
  // The check after pickup needs the planned carry.
  if (with_check) carry = cell::CarryMode::kPlanned;

  PrintHeader(pretty, with_check);
  std::vector<Row> rows;
  double simulated = 0.0;
  const auto wall_start = std::chrono::steady_clock::now();
  std::vector<std::string> replay_lines;
  bool replay_same = false;

  for (int t = 0; t < trials; ++t) {
    settings.seed = static_cast<std::uint32_t>(first_seed + t);
    const cell::TrialResult trial = cell::RunTrial(
        cell, settings,
        [&](const Row& row, const cell::PickPlace& script,
            const cell::CycleCheck& check) {
          PrintCycle(row, script, check, pretty, with_check, phases, verbose);
        });
    if (!trial.error.empty()) {
      std::fprintf(stderr, "%s\n", trial.error.c_str());
      return 1;
    }
    rows.insert(rows.end(), trial.cycles.begin(), trial.cycles.end());
    simulated += trial.simulated_seconds;
    if (!save_failures.empty() || replayed) {
      const cell::TrialVerdict verdict = cell::JudgeTrial(trial);
      if (!save_failures.empty() && !verdict.passed) {
        std::string save_error;
        const std::optional<std::string> path = cell::SaveFailure(
            save_failures, cell::MakeFailureRecord(trial, verdict),
            &save_error);
        if (path) {
          std::fprintf(stderr, "seed %u did not pass (%s): saved %s\n",
                       settings.seed, verdict.why.c_str(), path->c_str());
        } else {
          std::fprintf(stderr, "%s\n", save_error.c_str());
        }
      }
      if (replayed) {
        replay_lines =
            cell::CompareFailure(*replayed, cell::MakeFailureRecord(trial, verdict));
        replay_same = replay_lines.empty();
      }
    }
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
  // The check: measured offsets, the finder's error against the truth and
  // its time, leg 2's recheck and replans.
  Stat check_shift, check_turn, check_error_shift, check_error_turn;
  Stat check_finder, recheck_time, replan_time;
  int checked = 0, retaken = 0, refused = 0, slipped = 0, replanned = 0,
      rechecked = 0;
  int implausible = 0, no_path = 0;
  int unconfirmed = 0;
  int placed = 0;
  std::map<std::string, int> failures;
  for (const Row& r : rows) {
    const cell::CheckReport& c = r.check;
    if (c.status == cell::CheckReport::Status::kChecked) {
      ++checked;
      check_shift.Add(1e3 * c.offset_shift.norm());
      check_turn.Add(c.offset_rotation.norm() / kDegree);
    }
    if (Retaken(r)) ++retaken;
    if (c.status == cell::CheckReport::Status::kRefused) ++refused;
    if (c.status == cell::CheckReport::Status::kRefused && c.implausible) {
      ++implausible;
    }
    if (c.status == cell::CheckReport::Status::kNoPath) ++no_path;
    if (c.status == cell::CheckReport::Status::kSlipped) ++slipped;
    if (c.pictures > 0) check_finder.Add(1e3 * c.finder_seconds);
    if (c.rechecked) {
      ++rechecked;
      recheck_time.Add(1e3 * c.recheck_seconds);
    }
    if (c.replanned) {
      ++replanned;
      replan_time.Add(1e3 * c.replan_seconds);
    }
    if (r.check_error_shift) {
      check_error_shift.Add(1e3 * *r.check_error_shift);
      check_error_turn.Add(*r.check_error_turn / kDegree);
    }
    if (r.placed && r.unconfirmed) ++unconfirmed;
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
  std::printf("%s%s%d of %zu cycles placed (%s%s%s, %s carry%s%s%s)",
              pretty ? "\n" : "", lead, placed, rows.size(),
              camera ? "camera" : "blind",
              camera ? (noise ? ", noisy pictures" : ", exact pictures") : "",
              calibration_mm > 0.0 || calibration_deg > 0.0
                  ? ", calibration off"
                  : "",
              cell::CarryModeName(carry),
              carry == cell::CarryMode::kPlanned &&
                      planner == cell::PlanSearch::kOmpl
                  ? " by OMPL"
                  : "",
              with_check ? ", check after pickup" : "",
              with_check && (check_calibration_mm > 0.0 ||
                        check_calibration_deg > 0.0)
                  ? ", check camera's calibration off"
                  : "");
  if (with_check || unconfirmed > 0) {
    std::printf("; %d retaken, %d placement unconfirmed", retaken,
                unconfirmed);
  }
  std::printf("\n");
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
  if (with_check) {
    std::printf("%scheck after pickup: %d checked (%d retaken), %d refused "
                "(%d implausible), %d slipped, %d no path; "
                "measured grip offset, worst / rms: %.2f / %.2f mm, %.3f / "
                "%.3f deg; finder %.0f ms at most (all pictures of a cycle)\n",
                lead, checked, retaken, refused, implausible, slipped, no_path,
                check_shift.worst,
                check_shift.Rms(), check_turn.worst, check_turn.Rms(),
                check_finder.worst);
    if (check_error_shift.count > 0) {
      std::printf("%scheck finder error against the true grip, worst / rms: "
                  "%.3f / %.3f mm, %.4f / %.4f deg\n",
                  lead, check_error_shift.worst, check_error_shift.Rms(),
                  check_error_turn.worst, check_error_turn.Rms());
    }
    std::printf("%sleg 2 rechecked with the measured grip: %d (mean %.0f ms, "
                "worst %.0f ms); replanned: %d (mean %.0f ms, worst %.0f "
                "ms)\n",
                lead, rechecked, recheck_time.Mean(), recheck_time.worst,
                replanned, replan_time.Mean(), replan_time.worst);
  }
  for (const auto& [kind, count] : failures) {
    std::printf("%sfailed %d: %s\n", pretty ? "" : "# ", count, kind.c_str());
  }
  std::fprintf(stderr, "simulated %.1f s in %.1f s of wall time: %.2fx real time\n",
               simulated, wall, wall > 0.0 ? simulated / wall : 0.0);
  if (replayed) {
    std::printf("%sreplay of %s: %s\n", lead, replay.c_str(),
                replay_same ? "the same failure, bit-identical (module, slot, "
                              "step, reason, verdict and every number)"
                            : "NOT the same failure");
    for (const std::string& line : replay_lines) {
      std::printf("%s  %s\n", lead, line.c_str());
    }
    return replay_same ? 0 : 2;
  }
  return 0;
}
