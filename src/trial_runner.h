// Runs trials of the pick and place (src/pick_place.h) in the cell and
// judges them, without printing anything: the library under run_cell (the
// table), run_gate and the gate test (//:gate_test).
//
// A trial is one randomized pallet (Cell::Reset with the trial's seed and
// cell::Randomization) and `modules` modules picked and placed one after
// another, module_<first> first, each cycle scored from the truth by
// src/score.h at the moment the script reports the module let go (the
// placed hook). A trial stops at its first FAILED cycle: with the front
// module still on the pallet nothing behind it can be picked. A cycle that
// placed its module out of tolerance does not stop the trial.
//
// Like run_cell, this reads the truth (the scorer, the finder's error, the
// grip's true pose): it is a scorer, not control code. The script it runs
// never reads the truth.
//
// The verdict. A placement is in tolerance (PlacementTolerance) when the
// module rests on the rails (at rest, by score.h's kRestSpeed and
// kRestSpin, at a height within rest_height of the rails' surface), its
// position error in the rails' plane, sqrt(along^2 + across^2), is at most
// `position`, and its yaw and tilt errors are each at most `yaw` and
// `tilt`. A cycle that failed, or whose placement is unconfirmed (a blocked
// set-down: the script let go where it stopped), is out of tolerance. A
// trial passes when it ran every cycle it was asked for and every one is in
// tolerance.
//
// Failure records. A trial that does not pass is written as one small text
// file (SaveFailure): the trial's settings, its first bad cycle (module,
// slot, step, reason, the scorer's numbers) and two command lines that rerun
// it, `run_cell --replay FILE` and the same trial in run_cell's flags. The
// numbers are written in the shortest form that reads back to the same
// double (std::to_chars), so a replay can say whether it reproduced the
// failure bit for bit (CompareFailure).
//
// Determinism. A trial is a function of its settings: the cell's
// randomization, the pictures' noise and the conveyor's stops are drawn
// from the seed, and the planner's base seed is fixed. Wall-clock times
// (the finder's, the planner's) are measured and reported but nothing the
// script does depends on them, except the planner's time budget (10 s per
// carry, against about 0.3 s used). RunTrials runs trials on several
// threads, one Cell per thread; nothing in the cell, the script or MuJoCo's
// stepping is shared between Cells. Models are loaded one at a time.

#ifndef SRC_TRIAL_RUNNER_H_
#define SRC_TRIAL_RUNNER_H_

#include <cstdint>
#include <functional>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include "src/cell.h"
#include "src/cycle_check.h"
#include "src/pick_place.h"
#include "src/score.h"

namespace cell {

// Everything run_cell's flags set about a trial, and its seed.
struct TrialSettings {
  std::uint32_t seed = 1;
  int modules = 1;  // cycles, module_<first> first
  int first = 0;    // modules 0 .. first - 1 parked before the trial
  // The pallet exactly at its design pose and the conveyor stopping
  // exactly (Randomization::None); else Randomization's defaults, with the
  // two ranges below if set.
  bool nominal = false;
  std::optional<double> xy_mm;    // pallet offset range, mm
  std::optional<double> yaw_deg;  // pallet turn range, degrees
  bool camera = true;             // else blind
  bool noise = true;              // the camera's own errors (camera only)
  double calibration_mm = 0.0;    // pallet camera's calibration error
  double calibration_deg = 0.0;
  CarryMode carry = CarryMode::kScripted;  // as asked; check implies planned
  bool check = false;                      // the check after pickup
  // Which search plans the planned carry (see src/motion_planner.h).
  PlanSearch planner = PlanSearch::kOwn;
  double check_calibration_mm = 0.0;       // check camera's
  double check_calibration_deg = 0.0;

  // The carry the trial runs: planned whenever the check is on.
  CarryMode effective_carry() const {
    return check ? CarryMode::kPlanned : carry;
  }
};

// What Reset is given for these settings, and the script's options.
Randomization RandomizationFor(const TrialSettings& settings);
PickPlaceOptions OptionsFor(const TrialSettings& settings);

// The settings the gate runs every trial with: a whole pallet (30 modules
// onto three tubes), the camera with noisy pictures, the planned carry and
// the check after pickup, the pallet camera's and the check camera's
// calibrations each off by 1 mm and 0.05 degrees, the default pallet
// randomization.
TrialSettings GateTrialSettings(std::uint32_t seed);

// One cycle of a trial: run_cell's row. Placement errors and everything
// marked "truth" come from the simulation, for scoring only.
struct CycleResult {
  std::uint32_t seed = 0;
  int module = -1;                        // the module the cycle was for
  double dx = 0.0, dy = 0.0, yaw = 0.0;  // true pallet offset (truth)
  bool placed = false;
  // Why it failed ("timed out in <step>" if it did not end within the
  // runner's limit), or a note on a placement ("set down blocked: ...",
  // "check retaken: ..."); empty for a clean placement.
  std::string reason;
  // The step the cycle failed in (the step it was in, if it timed out);
  // kIdle when it did not fail.
  PickPlaceStep failed_step = PickPlaceStep::kIdle;
  bool timed_out = false;
  int slot = -1;  // the slot the script meant (outcome().slot)
  // The scorer's placement error, measured at the placed hook (truth).
  std::optional<PlacementError> error;
  StackDisturbance disturbance;  // truth
  double cycle_time = 0.0;       // simulated s
  // Camera mode: the finder's error for the estimate the pick used (truth),
  // and its computing time.
  std::optional<EstimateError> seen;
  double finder_seconds = 0.0;
  // The carry (PickPlaceOutcome::carry), and the cycle's highest motor
  // torque over its limit (truth, from cycle_check).
  CarryReport carry;
  double peak_torque = 0.0;
  double carry_torque = 0.0;  // the same during the carry's steps only
  // The check after pickup, and the measured grip's error against the true
  // grip (shift in m, turn in rad), from the truth at the accepted picture.
  CheckReport check;
  std::optional<double> check_error_shift, check_error_turn;
  bool unconfirmed = false;  // a blocked set-down: placement unconfirmed
  // The script's step records of the cycle (PickPlace::records()).
  std::vector<StepRecord> records;
};

struct TrialResult {
  TrialSettings settings;
  // Empty if the trial ran; else why not (settings the cell refuses).
  std::string error;
  // The pallet's true offset Reset drew (truth).
  double pallet_dx = 0.0, pallet_dy = 0.0, pallet_yaw = 0.0;
  std::vector<CycleResult> cycles;
  double simulated_seconds = 0.0;  // the cell's clock at the end
  double wall_seconds = 0.0;
};

// Called after each cycle with its result and the script and check that
// ran it, for a caller that prints as it goes (run_cell's table, --verbose).
using CycleObserver = std::function<void(
    const CycleResult& result, const PickPlace& script, const CycleCheck& check)>;

// A cycle that has not ended after this much simulated time has hung.
inline constexpr double kCycleLimit = 120.0;  // s
// After a failed cycle the stack is left this long before it is measured,
// so a module knocked loose has come to rest.
inline constexpr double kSettleAfterFailure = 1.0;  // s

// Resets `cell` for `settings` and runs the trial. Refuses (error set,
// nothing run) settings out of range for the cell: modules < 1, first < 0,
// first + modules beyond the pallet, or ranges Reset refuses.
TrialResult RunTrial(Cell& cell, const TrialSettings& settings,
                     const CycleObserver& observer = {});

// Runs each of `trials` on `threads` threads (at least 1), each thread
// with its own Cell loaded from `model_path`. Results are in the order of
// `trials`. `done`, if given, is called as each trial ends, from the thread
// that ran it, one call at a time. Returns nothing, with `error` set, if
// the model does not load.
std::optional<std::vector<TrialResult>> RunTrials(
    const std::string& model_path, const std::vector<TrialSettings>& trials,
    int threads,
    const std::function<void(int index, const TrialResult&)>& done = {},
    std::string* error = nullptr);

// ------------------------------------------------------------ the verdict

struct PlacementTolerance {
  // In the rails' plane: sqrt(along_tube^2 + across_tube^2).
  double position = 3e-3;                       // m
  double yaw = 0.15 * std::numbers::pi / 180;   // rad
  double tilt = 0.15 * std::numbers::pi / 180;  // rad
  // "Rests on the rails": at rest (PlacementError::at_rest) and its height
  // above the rails' surface within this. Not part of the owner's figures:
  // chosen to tell a module lying on the rails (heights measured within
  // 0.01 mm) from one lying on something else (a module's thickness or
  // more), and to leave the plane's tilt to `tilt`.
  double rest_height = 2e-3;  // m
};

struct CycleVerdict {
  bool in_tolerance = false;
  // Why not, in words with the numbers ("position 3.21 mm > 3.00 mm");
  // empty when in tolerance.
  std::string why;
};
CycleVerdict JudgeCycle(const CycleResult& cycle,
                        const PlacementTolerance& tolerance = {});

// The in-plane position error of a placement, m.
double PlanePositionError(const PlacementError& error);

struct TrialVerdict {
  bool passed = false;
  // Indices into TrialResult::cycles of every cycle out of tolerance, in
  // order, with why; and why the trial failed if not for a cycle (it did not
  // run, or ran fewer cycles than asked without a failed one).
  std::vector<int> bad_cycles;
  std::vector<std::string> bad_why;
  std::string why;
};
TrialVerdict JudgeTrial(const TrialResult& trial,
                        const PlacementTolerance& tolerance = {});

// ------------------------------------------------- saving and replaying

// One failed trial: what reruns it and what its first bad cycle was.
struct FailureRecord {
  TrialSettings settings;
  bool has_cycle = false;  // false: the trial failed without a bad cycle
  int cycle = -1;          // index of the first bad cycle in the trial
  int module = -1;
  int slot = -1;
  std::string outcome;  // "placed" or "failed"
  std::string step;     // StepName of the failed step; "-" when placed
  std::string reason;   // CycleResult::reason
  std::string why;      // CycleVerdict::why, or TrialVerdict::why
  std::optional<PlacementError> error;
  double cycle_time = 0.0;
  int bad_cycles = 0;  // how many cycles of the trial were bad
};

// The record of `trial`'s first bad cycle; `verdict` is JudgeTrial's.
FailureRecord MakeFailureRecord(const TrialResult& trial,
                                const TrialVerdict& verdict,
                                const PlacementTolerance& tolerance = {});

// run_cell's flags for `settings` (no --pretty), e.g. "--seed 7 --modules
// 30 --check --calibration 1,0.05 --check-calibration 1,0.05". With
// `through_cycle` >= 0 the modules are cut to end with that cycle, which
// is all a replay of it needs: a cycle does not depend on later ones.
std::string RunCellFlags(const TrialSettings& settings,
                         int through_cycle = -1);

// The file's name: "failure_seed_<seed>_module_<module>.txt" (or
// "failure_seed_<seed>.txt" without a bad cycle).
std::string FailureFileName(const FailureRecord& record);

// The file's text, and back. `saved_as` is the path the replay command
// names (the file's own path, where it is written).
std::string FormatFailureRecord(const FailureRecord& record,
                                const std::string& saved_as);
std::optional<FailureRecord> ParseFailureRecord(const std::string& text,
                                                std::string* error = nullptr);

// Writes the record into `directory` (made if missing) under
// FailureFileName, and returns the file's path; nothing, with `error` set,
// if it cannot be written.
std::optional<std::string> SaveFailure(const std::string& directory,
                                       const FailureRecord& record,
                                       std::string* error = nullptr);
std::optional<FailureRecord> LoadFailure(const std::string& path,
                                         std::string* error = nullptr);

// The settings that replay `record`: its trial, cut to end with the bad
// cycle.
TrialSettings ReplaySettings(const FailureRecord& record);

// Compares a replayed record with the saved one: module, slot, outcome,
// step, reason, verdict and every number. Empty when they are the same bit
// for bit; else one line per difference, with both values.
std::vector<std::string> CompareFailure(const FailureRecord& saved,
                                        const FailureRecord& replayed);

// The replay command for a saved file, and the rerun in flags.
std::string ReplayCommand(const std::string& path);
std::string RerunCommand(const FailureRecord& record);

// One line describing a bad cycle for a report: seed, module, slot, the
// errors in mm and degrees, and why.
std::string DescribeBadCycle(const CycleResult& cycle, const std::string& why);

// ---------------------------------------------------------------- the gate

// The gate's batch: kGateTrials whole pallets with GateTrialSettings, seeds
// kGateFirstSeed onward, in kGateShards Bazel shards (//:gate_test). Chosen
// for about 5 minutes of wall time on 4 cores: a whole pallet takes about
// 2 minutes of one core (docs/gate.md has the measurement).
inline constexpr int kGateTrials = 8;
inline constexpr int kGateShards = 4;
inline constexpr std::uint32_t kGateFirstSeed = 1;

// The worst placement over every scored cycle of a batch, and the counts.
struct BatchSummary {
  int trials = 0;
  int passed = 0;
  int cycles = 0;
  int placements = 0;  // placed and scored
  int bad_cycles = 0;
  double worst_position = 0.0;  // m, in the rails' plane
  double worst_yaw = 0.0;       // rad, absolute
  double worst_tilt = 0.0;      // rad
  double worst_height = 0.0;    // m, absolute
  // Where each worst was: seed and module.
  std::uint32_t worst_position_seed = 0, worst_yaw_seed = 0,
                worst_tilt_seed = 0;
  int worst_position_module = -1, worst_yaw_module = -1,
      worst_tilt_module = -1;
  double wall_seconds = 0.0;       // the trials' own, summed
  double simulated_seconds = 0.0;  // summed
};
void AddToSummary(const TrialResult& trial, const TrialVerdict& verdict,
                  BatchSummary* summary);

// One line for a trial: seed, pass or not, cycles placed, its worst
// position, yaw and tilt, simulated and wall time.
std::string DescribeTrial(const TrialResult& trial,
                          const TrialVerdict& verdict);
// A few lines for a batch: trials passed, placements, the worsts and where.
std::string DescribeSummary(const BatchSummary& summary,
                            const PlacementTolerance& tolerance = {});

}  // namespace cell

#endif  // SRC_TRIAL_RUNNER_H_
