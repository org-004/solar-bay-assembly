#include "src/trial_runner.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/held_module_finder.h"
#include "src/suction.h"

namespace cell {
namespace {

constexpr double kDegree = std::numbers::pi / 180.0;

// The shortest text that reads back to the same double.
std::string Number(double value) {
  char text[64];
  const std::to_chars_result r = std::to_chars(text, text + sizeof(text), value);
  return std::string(text, r.ptr);
}

std::optional<double> ParseNumber(const std::string& text) {
  double value = 0.0;
  const char* end = text.data() + text.size();
  const std::from_chars_result r = std::from_chars(text.data(), end, value);
  if (r.ec != std::errc() || r.ptr != end) return std::nullopt;
  return value;
}

std::optional<long> ParseInteger(const std::string& text) {
  long value = 0;
  const char* end = text.data() + text.size();
  const std::from_chars_result r = std::from_chars(text.data(), end, value);
  if (r.ec != std::errc() || r.ptr != end) return std::nullopt;
  return value;
}

std::string OneLine(std::string text) {
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  return text;
}

std::string Format(const char* format, double a, double b) {
  char text[96];
  std::snprintf(text, sizeof(text), format, a, b);
  return text;
}

}  // namespace

Randomization RandomizationFor(const TrialSettings& s) {
  Randomization randomization =
      s.nominal ? Randomization::None() : Randomization{};
  if (s.xy_mm) randomization.pallet_xy = *s.xy_mm * 1e-3;
  if (s.yaw_deg) randomization.pallet_yaw = *s.yaw_deg * kDegree;
  randomization.camera_noise = s.camera && s.noise;
  randomization.camera_shift = s.calibration_mm * 1e-3;
  randomization.camera_turn = s.calibration_deg * kDegree;
  randomization.check_camera_shift = s.check_calibration_mm * 1e-3;
  randomization.check_camera_turn = s.check_calibration_deg * kDegree;
  return randomization;
}

PickPlaceOptions OptionsFor(const TrialSettings& s) {
  PickPlaceOptions options;
  options.sensing = s.camera ? SensingMode::kCamera : SensingMode::kBlind;
  options.carry = s.effective_carry();
  options.check_after_pickup = s.check;
  options.planner.search = s.planner;
  return options;
}

TrialSettings GateTrialSettings(std::uint32_t seed) {
  TrialSettings s;
  s.seed = seed;
  s.modules = 30;
  s.camera = true;
  s.noise = true;
  s.carry = CarryMode::kPlanned;
  s.check = true;
  s.calibration_mm = 1.0;
  s.calibration_deg = 0.05;
  s.check_calibration_mm = 1.0;
  s.check_calibration_deg = 0.05;
  return s;
}

// The body of run_cell's trial loop, moved here whole: one Reset, then the
// cycles, each scored at the placed hook.
TrialResult RunTrial(Cell& cell, const TrialSettings& settings,
                     const CycleObserver& observer) {
  const auto wall_start = std::chrono::steady_clock::now();
  TrialResult result;
  result.settings = settings;
  const int first = settings.first;
  const int modules = settings.modules;
  if (modules < 1 || first < 0 || first + modules > cell.module_count()) {
    result.error = "modules and first must fit the pallet's " +
                   std::to_string(cell.module_count()) + " modules";
    return result;
  }
  const std::uint32_t seed = settings.seed;
  if (!cell.Reset(seed, RandomizationFor(settings))) {
    result.error = "--xy, --yaw and --calibration must not be negative";
    return result;
  }
  if (first > 0) {
    for (int i = 0; i < first; ++i) cell.ParkModule(i);
    cell.StepFor(Cell::kSettleTime);
    mj_forward(cell.model(), cell.data());
    cell.RebaselineStack();
  }
  const TrueScene& scene = cell.true_scene();
  result.pallet_dx = scene.pallet_x;
  result.pallet_dy = scene.pallet_y;
  result.pallet_yaw = scene.pallet_yaw;
  PickPlace script(cell, OptionsFor(settings));
  std::optional<PlacementError> placement;
  StackDisturbance disturbance;
  script.set_placed_hook([&](int module, int slot) {
    // Score as the module lies now, before the tube moves on.
    mj_forward(cell.model(), cell.data());
    placement = MeasurePlacement(cell, module, slot);
    disturbance = MeasureStackDisturbance(cell);
    cell.RebaselineStack();
  });
  // The measured grip against the true one, at the picture: the suction
  // weld's pose in the tool tip's frame, from the positions the picture
  // brought up to date.
  std::optional<double> check_error_shift, check_error_turn;
  script.set_check_hook([&](const CheckRecord& record) {
    if (!record.module_in_tip) return;
    const std::optional<Pose> held = cell.suction().HeldPoseInTip(*cell.data());
    if (!held) return;
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.translation() = held->position;
    truth.linear() = held->rotation.normalized().toRotationMatrix();
    const Eigen::Isometry3d error = truth.inverse() * *record.module_in_tip;
    check_error_shift = error.translation().norm();
    check_error_turn = Eigen::AngleAxisd(error.linear()).angle();
  });

  for (int m = first; m < first + modules; ++m) {
    CycleResult row;
    check_error_shift.reset();
    check_error_turn.reset();
    row.seed = seed;
    row.module = m;
    row.dx = scene.pallet_x;
    row.dy = scene.pallet_y;
    row.yaw = scene.pallet_yaw;
    placement.reset();
    CycleCheck check(cell);
    const double start = cell.data()->time;
    script.Start(m);
    while (!script.Done() && cell.data()->time - start < kCycleLimit) {
      script.Update();
      cell.Step();
      check.Observe(script.step(), m);
      // The finder's error, from the truth, as soon as the pick has its
      // estimate: the module still stands where it was seen.
      if (!row.seen && script.estimate()) {
        row.seen = EstimateErrorOf(script.estimate()->pose,
                                   cell.TrueModulePose(m));
        row.finder_seconds = script.estimate()->diagnostics.seconds_total;
      }
    }
    const PickPlaceOutcome& outcome = script.outcome();
    row.placed = outcome.status == PickPlaceOutcome::Status::kPlaced;
    if (!script.Done()) {
      row.reason = std::string("timed out in ") + script.step_name();
      row.timed_out = true;
      row.failed_step = script.step();
    } else if (!row.placed) {
      row.reason = outcome.reason;
      row.failed_step = outcome.failed_step;
    }
    row.slot = outcome.slot;
    row.cycle_time = cell.data()->time - start;
    row.carry = outcome.carry;
    row.check = outcome.check;
    row.unconfirmed = outcome.placement_unconfirmed;
    if (row.check.status == CheckReport::Status::kChecked) {
      row.check_error_shift = check_error_shift;
      row.check_error_turn = check_error_turn;
    }
    if (row.placed && row.unconfirmed) {
      row.reason = "set down blocked: placement unconfirmed";
    } else if (row.placed && row.check.retaken) {
      row.reason = "check retaken: picture 1 " +
                   std::string(HeldFinderFailureName(row.check.failure)) +
                   ": " + row.check.failure_detail;
    }
    row.peak_torque = check.peak_torque().maxCoeff();
    for (PickPlaceStep s : {PickPlaceStep::kFlatten, PickPlaceStep::kSwing,
                            PickPlaceStep::kTurn, PickPlaceStep::kCarry}) {
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
      row.disturbance = MeasureStackDisturbance(cell);
    }
    row.records = script.records();
    if (observer) observer(row, script, check);
    result.cycles.push_back(std::move(row));
    if (!result.cycles.back().placed) break;
  }
  result.simulated_seconds = cell.data()->time;
  result.wall_seconds = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - wall_start)
                            .count();
  return result;
}

std::optional<std::vector<TrialResult>> RunTrials(
    const std::string& model_path, const std::vector<TrialSettings>& trials,
    int threads, const std::function<void(int, const TrialResult&)>& done,
    std::string* error) {
  threads = std::max(1, std::min<int>(threads, static_cast<int>(trials.size())));
  std::vector<TrialResult> results(trials.size());
  std::atomic<int> next{0};
  std::atomic<bool> load_failed{false};
  std::mutex load_mutex, done_mutex;
  std::string load_error;
  auto work = [&]() {
    std::optional<Cell> cell;
    {
      // MuJoCo's XML loader is not documented as thread-safe: one at a time.
      std::lock_guard<std::mutex> lock(load_mutex);
      std::string e;
      cell = Cell::Load(model_path, &e);
      if (!cell) {
        load_failed = true;
        load_error = e;
        return;
      }
    }
    for (int i = next++; i < static_cast<int>(trials.size()) && !load_failed;
         i = next++) {
      results[i] = RunTrial(*cell, trials[i]);
      if (done) {
        std::lock_guard<std::mutex> lock(done_mutex);
        done(i, results[i]);
      }
    }
  };
  if (threads == 1) {
    work();
  } else {
    std::vector<std::thread> pool;
    for (int t = 0; t < threads; ++t) pool.emplace_back(work);
    for (std::thread& t : pool) t.join();
  }
  if (load_failed) {
    if (error != nullptr) *error = load_error;
    return std::nullopt;
  }
  return results;
}

// ------------------------------------------------------------ the verdict

double PlanePositionError(const PlacementError& e) {
  return std::hypot(e.along_tube, e.across_tube);
}

CycleVerdict JudgeCycle(const CycleResult& c, const PlacementTolerance& t) {
  CycleVerdict v;
  if (!c.placed) {
    v.why = std::string("failed in ") + StepName(c.failed_step) + ": " +
            c.reason;
    return v;
  }
  if (c.unconfirmed) {
    v.why = "placement unconfirmed (set down blocked)";
    return v;
  }
  if (!c.error) {
    v.why = "placed but not scored";
    return v;
  }
  const PlacementError& e = *c.error;
  std::vector<std::string> out;
  if (!e.at_rest) {
    out.push_back(Format("not at rest (%.4f m/s, %.4f rad/s)", e.speed,
                         e.spin));
  }
  if (!(std::abs(e.height) <= t.rest_height)) {
    out.push_back(Format("height %.2f mm, off the rails by more than %.2f mm",
                         1e3 * e.height, 1e3 * t.rest_height));
  }
  const double position = PlanePositionError(e);
  if (!(position <= t.position)) {
    out.push_back(Format("position %.2f mm > %.2f mm", 1e3 * position,
                         1e3 * t.position));
  }
  if (!(std::abs(e.yaw) <= t.yaw)) {
    out.push_back(Format("yaw %.3f deg > %.3f deg", std::abs(e.yaw) / kDegree,
                         t.yaw / kDegree));
  }
  if (!(std::abs(e.tilt) <= t.tilt)) {
    out.push_back(Format("tilt %.3f deg > %.3f deg",
                         std::abs(e.tilt) / kDegree, t.tilt / kDegree));
  }
  for (std::size_t i = 0; i < out.size(); ++i) {
    v.why += (i ? "; " : "") + out[i];
  }
  v.in_tolerance = out.empty();
  return v;
}

TrialVerdict JudgeTrial(const TrialResult& trial, const PlacementTolerance& t) {
  TrialVerdict v;
  if (!trial.error.empty()) {
    v.why = "did not run: " + trial.error;
    return v;
  }
  for (int i = 0; i < static_cast<int>(trial.cycles.size()); ++i) {
    const CycleVerdict c = JudgeCycle(trial.cycles[i], t);
    if (!c.in_tolerance) {
      v.bad_cycles.push_back(i);
      v.bad_why.push_back(c.why);
    }
  }
  if (v.bad_cycles.empty() &&
      static_cast<int>(trial.cycles.size()) != trial.settings.modules) {
    v.why = "ran " + std::to_string(trial.cycles.size()) + " of " +
            std::to_string(trial.settings.modules) + " cycles";
    return v;
  }
  v.passed = v.bad_cycles.empty();
  if (!v.passed) {
    v.why = std::to_string(v.bad_cycles.size()) + " of " +
            std::to_string(trial.cycles.size()) + " cycles out of tolerance";
  }
  return v;
}

// ------------------------------------------------- saving and replaying

FailureRecord MakeFailureRecord(const TrialResult& trial,
                                const TrialVerdict& verdict,
                                const PlacementTolerance& tolerance) {
  FailureRecord r;
  r.settings = trial.settings;
  r.bad_cycles = static_cast<int>(verdict.bad_cycles.size());
  if (verdict.bad_cycles.empty()) {
    r.why = verdict.why;
    return r;
  }
  const int i = verdict.bad_cycles.front();
  const CycleResult& c = trial.cycles[i];
  r.has_cycle = true;
  r.cycle = i;
  r.module = c.module;
  r.slot = c.slot;
  r.outcome = c.placed ? "placed" : "failed";
  r.step = c.placed ? "-" : StepName(c.failed_step);
  r.reason = OneLine(c.reason);
  r.why = OneLine(JudgeCycle(c, tolerance).why);
  r.error = c.error;
  r.cycle_time = c.cycle_time;
  return r;
}

std::string RunCellFlags(const TrialSettings& s, int through_cycle) {
  const int modules = through_cycle >= 0 ? std::min(s.modules, through_cycle + 1)
                                         : s.modules;
  std::string f = "--seed " + std::to_string(s.seed) + " --modules " +
                  std::to_string(modules);
  if (s.first > 0) f += " --first " + std::to_string(s.first);
  if (s.nominal) f += " --nominal";
  if (s.xy_mm) f += " --xy " + Number(*s.xy_mm);
  if (s.yaw_deg) f += " --yaw " + Number(*s.yaw_deg);
  if (!s.camera) f += " --blind";
  if (!s.noise) f += " --no-noise";
  if (s.calibration_mm != 0.0 || s.calibration_deg != 0.0) {
    f += " --calibration " + Number(s.calibration_mm) + "," +
         Number(s.calibration_deg);
  }
  if (s.carry == CarryMode::kPlanned) f += " --carry planned";
  if (s.planner == PlanSearch::kOmpl) f += " --planner ompl";
  if (s.check) f += " --check";
  if (s.check_calibration_mm != 0.0 || s.check_calibration_deg != 0.0) {
    f += " --check-calibration " + Number(s.check_calibration_mm) + "," +
         Number(s.check_calibration_deg);
  }
  return f;
}

std::string FailureFileName(const FailureRecord& r) {
  std::string name = "failure_seed_" + std::to_string(r.settings.seed);
  if (r.has_cycle) name += "_module_" + std::to_string(r.module);
  return name + ".txt";
}

std::string ReplayCommand(const std::string& path) {
  return "bazel run --config=opt //:run_cell -- --pretty --replay " + path;
}

std::string RerunCommand(const FailureRecord& r) {
  return "bazel run --config=opt //:run_cell -- --pretty " +
         RunCellFlags(r.settings, r.has_cycle ? r.cycle : -1);
}

std::string FormatFailureRecord(const FailureRecord& r,
                                const std::string& saved_as) {
  const TrialSettings& s = r.settings;
  std::ostringstream o;
  o << "# A trial of the pick and place that did not pass\n"
       "# (src/trial_runner.h). Numbers in metres and radians, written to\n"
       "# read back exactly.\n";
  if (r.has_cycle) {
    o << "# seed " << s.seed << ", module " << r.module << " (cycle "
      << r.cycle << ", slot " << r.slot << "): "
      << (r.outcome == "placed" ? "placed: " : "") << r.why << "\n";
    if (r.error) o << "# " << Describe(*r.error) << "\n";
  } else {
    o << "# seed " << s.seed << ": " << r.why << "\n";
  }
  o << "replay = " << ReplayCommand(saved_as) << "\n";
  o << "rerun = " << RerunCommand(r) << "\n";
  o << "\n# the trial\n";
  o << "seed = " << s.seed << "\n";
  o << "modules = " << s.modules << "\n";
  o << "first = " << s.first << "\n";
  o << "nominal = " << (s.nominal ? 1 : 0) << "\n";
  if (s.xy_mm) o << "xy_mm = " << Number(*s.xy_mm) << "\n";
  if (s.yaw_deg) o << "yaw_deg = " << Number(*s.yaw_deg) << "\n";
  o << "camera = " << (s.camera ? 1 : 0) << "\n";
  o << "noise = " << (s.noise ? 1 : 0) << "\n";
  o << "calibration_mm = " << Number(s.calibration_mm) << "\n";
  o << "calibration_deg = " << Number(s.calibration_deg) << "\n";
  o << "carry = " << CarryModeName(s.carry) << "\n";
  o << "planner = " << PlanSearchName(s.planner) << "\n";
  o << "check = " << (s.check ? 1 : 0) << "\n";
  o << "check_calibration_mm = " << Number(s.check_calibration_mm) << "\n";
  o << "check_calibration_deg = " << Number(s.check_calibration_deg) << "\n";
  o << "\n# its first bad cycle\n";
  o << "bad_cycles = " << r.bad_cycles << "\n";
  o << "why = " << OneLine(r.why) << "\n";
  if (r.has_cycle) {
    o << "cycle = " << r.cycle << "\n";
    o << "module = " << r.module << "\n";
    o << "slot = " << r.slot << "\n";
    o << "outcome = " << r.outcome << "\n";
    o << "step = " << r.step << "\n";
    o << "reason = " << OneLine(r.reason) << "\n";
    o << "cycle_time = " << Number(r.cycle_time) << "\n";
    if (r.error) {
      const PlacementError& e = *r.error;
      o << "error_slot = " << e.slot << "\n";
      o << "along_tube = " << Number(e.along_tube) << "\n";
      o << "across_tube = " << Number(e.across_tube) << "\n";
      o << "height = " << Number(e.height) << "\n";
      o << "yaw = " << Number(e.yaw) << "\n";
      o << "tilt = " << Number(e.tilt) << "\n";
      o << "speed = " << Number(e.speed) << "\n";
      o << "spin = " << Number(e.spin) << "\n";
      o << "at_rest = " << (e.at_rest ? 1 : 0) << "\n";
    }
  }
  return o.str();
}

std::optional<FailureRecord> ParseFailureRecord(const std::string& text,
                                                std::string* error) {
  std::map<std::string, std::string> values;
  std::istringstream in(text);
  std::string line;
  int number = 0;
  auto fail = [&](const std::string& message) -> std::optional<FailureRecord> {
    if (error != nullptr) *error = message;
    return std::nullopt;
  };
  while (std::getline(in, line)) {
    ++number;
    if (line.empty() || line[0] == '#') continue;
    const std::size_t eq = line.find(" = ");
    if (eq == std::string::npos) {
      return fail("line " + std::to_string(number) + ": no \" = \"");
    }
    values[line.substr(0, eq)] = line.substr(eq + 3);
  }
  bool ok = true;
  std::string missing;
  auto text_of = [&](const char* key) -> std::string {
    auto it = values.find(key);
    if (it == values.end()) {
      ok = false;
      missing = key;
      return {};
    }
    return it->second;
  };
  auto integer = [&](const char* key) -> long {
    const std::optional<long> v = ParseInteger(text_of(key));
    if (!v) {
      if (ok) missing = key;
      ok = false;
      return 0;
    }
    return *v;
  };
  auto real = [&](const char* key) -> double {
    const std::optional<double> v = ParseNumber(text_of(key));
    if (!v) {
      if (ok) missing = key;
      ok = false;
      return 0.0;
    }
    return *v;
  };
  FailureRecord r;
  TrialSettings& s = r.settings;
  s.seed = static_cast<std::uint32_t>(integer("seed"));
  s.modules = static_cast<int>(integer("modules"));
  s.first = static_cast<int>(integer("first"));
  s.nominal = integer("nominal") != 0;
  if (values.count("xy_mm")) s.xy_mm = real("xy_mm");
  if (values.count("yaw_deg")) s.yaw_deg = real("yaw_deg");
  s.camera = integer("camera") != 0;
  s.noise = integer("noise") != 0;
  s.calibration_mm = real("calibration_mm");
  s.calibration_deg = real("calibration_deg");
  const std::string carry = text_of("carry");
  if (carry == "planned") {
    s.carry = CarryMode::kPlanned;
  } else if (carry == "scripted") {
    s.carry = CarryMode::kScripted;
  } else if (ok) {
    return fail("carry must be planned or scripted, not \"" + carry + "\"");
  }
  // Text saved before the planner was a setting has no such line: own.
  const std::string planner = values.count("planner") ? values["planner"] : "own";
  if (const std::optional<PlanSearch> search = PlanSearchFromName(planner)) {
    s.planner = *search;
  } else if (ok) {
    return fail("planner must be own or ompl, not \"" + planner + "\"");
  }
  s.check = integer("check") != 0;
  s.check_calibration_mm = real("check_calibration_mm");
  s.check_calibration_deg = real("check_calibration_deg");
  r.bad_cycles = static_cast<int>(integer("bad_cycles"));
  r.why = text_of("why");
  r.has_cycle = values.count("cycle") > 0;
  if (r.has_cycle) {
    r.cycle = static_cast<int>(integer("cycle"));
    r.module = static_cast<int>(integer("module"));
    r.slot = static_cast<int>(integer("slot"));
    r.outcome = text_of("outcome");
    r.step = text_of("step");
    r.reason = values.count("reason") ? values["reason"] : "";
    r.cycle_time = real("cycle_time");
    if (values.count("along_tube")) {
      PlacementError e;
      e.module = r.module;
      e.slot = static_cast<int>(integer("error_slot"));
      e.along_tube = real("along_tube");
      e.across_tube = real("across_tube");
      e.height = real("height");
      e.yaw = real("yaw");
      e.tilt = real("tilt");
      e.speed = real("speed");
      e.spin = real("spin");
      e.at_rest = integer("at_rest") != 0;
      r.error = e;
    }
  }
  if (!ok) return fail("missing or unreadable: " + missing);
  return r;
}

std::optional<std::string> SaveFailure(const std::string& directory,
                                       const FailureRecord& record,
                                       std::string* error) {
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) {
    if (error != nullptr) *error = "cannot make " + directory + ": " + ec.message();
    return std::nullopt;
  }
  std::filesystem::path path =
      std::filesystem::path(directory) / FailureFileName(record);
  const std::string absolute = std::filesystem::absolute(path, ec).string();
  std::ofstream out(path);
  out << FormatFailureRecord(record, ec ? path.string() : absolute);
  out.close();
  if (!out) {
    if (error != nullptr) *error = "cannot write " + path.string();
    return std::nullopt;
  }
  return ec ? path.string() : absolute;
}

std::optional<FailureRecord> LoadFailure(const std::string& path,
                                         std::string* error) {
  std::ifstream in(path);
  if (!in) {
    if (error != nullptr) *error = "cannot read " + path;
    return std::nullopt;
  }
  std::ostringstream text;
  text << in.rdbuf();
  return ParseFailureRecord(text.str(), error);
}

TrialSettings ReplaySettings(const FailureRecord& record) {
  TrialSettings s = record.settings;
  if (record.has_cycle) s.modules = std::min(s.modules, record.cycle + 1);
  return s;
}

std::vector<std::string> CompareFailure(const FailureRecord& a,
                                        const FailureRecord& b) {
  std::vector<std::string> d;
  auto same_text = [&](const char* what, const std::string& x,
                       const std::string& y) {
    if (x != y) d.push_back(std::string(what) + ": \"" + x + "\" then \"" + y + "\"");
  };
  auto same_int = [&](const char* what, long x, long y) {
    if (x != y) {
      d.push_back(std::string(what) + ": " + std::to_string(x) + " then " +
                  std::to_string(y));
    }
  };
  auto same_real = [&](const char* what, double x, double y) {
    // Bit for bit: the same double, as the file reads back exactly.
    if (!(x == y) && !(std::isnan(x) && std::isnan(y))) {
      d.push_back(std::string(what) + ": " + Number(x) + " then " + Number(y) +
                  " (differs by " + Number(y - x) + ")");
    }
  };
  same_int("has a bad cycle", a.has_cycle, b.has_cycle);
  same_int("cycle", a.cycle, b.cycle);
  same_int("module", a.module, b.module);
  same_int("slot", a.slot, b.slot);
  same_text("outcome", a.outcome, b.outcome);
  same_text("step", a.step, b.step);
  same_text("reason", a.reason, b.reason);
  same_text("why", a.why, b.why);
  same_real("cycle_time", a.cycle_time, b.cycle_time);
  same_int("scored", a.error.has_value(), b.error.has_value());
  if (a.error && b.error) {
    const PlacementError& x = *a.error;
    const PlacementError& y = *b.error;
    same_int("error slot", x.slot, y.slot);
    same_real("along_tube", x.along_tube, y.along_tube);
    same_real("across_tube", x.across_tube, y.across_tube);
    same_real("height", x.height, y.height);
    same_real("yaw", x.yaw, y.yaw);
    same_real("tilt", x.tilt, y.tilt);
    same_real("speed", x.speed, y.speed);
    same_real("spin", x.spin, y.spin);
    same_int("at_rest", x.at_rest, y.at_rest);
  }
  return d;
}

std::string DescribeBadCycle(const CycleResult& c, const std::string& why) {
  char text[512];
  if (c.error) {
    const PlacementError& e = *c.error;
    std::snprintf(text, sizeof(text),
                  "seed %u module %d slot %d: %s; position %.2f mm (along "
                  "%.2f, across %.2f), height %.2f mm, yaw %.3f deg, tilt "
                  "%.3f deg, %s: ",
                  c.seed, c.module, e.slot, c.placed ? "placed" : "failed",
                  1e3 * PlanePositionError(e), 1e3 * e.along_tube,
                  1e3 * e.across_tube, 1e3 * e.height, e.yaw / kDegree,
                  e.tilt / kDegree, e.at_rest ? "at rest" : "moving");
  } else {
    std::snprintf(text, sizeof(text), "seed %u module %d: %s, not scored: ",
                  c.seed, c.module, c.placed ? "placed" : "failed");
  }
  return text + why;
}

void AddToSummary(const TrialResult& trial, const TrialVerdict& verdict,
                  BatchSummary* s) {
  ++s->trials;
  if (verdict.passed) ++s->passed;
  s->bad_cycles += static_cast<int>(verdict.bad_cycles.size());
  s->wall_seconds += trial.wall_seconds;
  s->simulated_seconds += trial.simulated_seconds;
  for (const CycleResult& c : trial.cycles) {
    ++s->cycles;
    if (!c.placed || !c.error) continue;
    ++s->placements;
    const PlacementError& e = *c.error;
    const double position = PlanePositionError(e);
    if (position > s->worst_position) {
      s->worst_position = position;
      s->worst_position_seed = c.seed;
      s->worst_position_module = c.module;
    }
    if (std::abs(e.yaw) > s->worst_yaw) {
      s->worst_yaw = std::abs(e.yaw);
      s->worst_yaw_seed = c.seed;
      s->worst_yaw_module = c.module;
    }
    if (std::abs(e.tilt) > s->worst_tilt) {
      s->worst_tilt = std::abs(e.tilt);
      s->worst_tilt_seed = c.seed;
      s->worst_tilt_module = c.module;
    }
    s->worst_height = std::max(s->worst_height, std::abs(e.height));
  }
}

std::string DescribeTrial(const TrialResult& trial,
                          const TrialVerdict& verdict) {
  BatchSummary one;
  AddToSummary(trial, verdict, &one);
  char text[400];
  std::snprintf(text, sizeof(text),
                "seed %u: %s, %d of %d cycles placed in tolerance; worst "
                "position %.2f mm, yaw %.3f deg, tilt %.3f deg; %.0f s "
                "simulated in %.0f s wall",
                trial.settings.seed, verdict.passed ? "PASS" : "FAIL",
                one.cycles - one.bad_cycles, trial.settings.modules,
                1e3 * one.worst_position, one.worst_yaw / kDegree,
                one.worst_tilt / kDegree, trial.simulated_seconds,
                trial.wall_seconds);
  std::string line = text;
  if (!verdict.passed && verdict.bad_cycles.empty()) line += ": " + verdict.why;
  return line;
}

std::string DescribeSummary(const BatchSummary& s,
                            const PlacementTolerance& t) {
  char text[900];
  std::snprintf(
      text, sizeof(text),
      "%d of %d trials passed; %d of %d cycles out of tolerance (%.1f mm in "
      "the rails' plane, %.3f deg yaw, %.3f deg tilt, resting within %.1f "
      "mm of the rails)\n"
      "worst over %d placements: position %.2f mm (seed %u module %d), yaw "
      "%.3f deg (seed %u module %d), tilt %.3f deg (seed %u module %d), "
      "height %.2f mm\n"
      "%.0f s simulated in %.0f s of trial wall time\n",
      s.passed, s.trials, s.bad_cycles, s.cycles, 1e3 * t.position,
      t.yaw / kDegree, t.tilt / kDegree, 1e3 * t.rest_height, s.placements,
      1e3 * s.worst_position, s.worst_position_seed, s.worst_position_module,
      s.worst_yaw / kDegree, s.worst_yaw_seed, s.worst_yaw_module,
      s.worst_tilt / kDegree, s.worst_tilt_seed, s.worst_tilt_module,
      1e3 * s.worst_height, s.simulated_seconds, s.wall_seconds);
  return text;
}

}  // namespace cell
