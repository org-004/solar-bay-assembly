// The pick and place (src/pick_place.h) in a window: the same script that
// src/run_cell.cc runs headless, stepped by the viewer in real time.
//
//   bazel run --config=opt //:view_cell -- [--seed N] [--nominal]
//       [--modules M] [--blind] [--no-noise] [--calibration MM,DEG]
//
//   --camera     (the default) one depth picture per pick; the corner text
//                shows the look step and what the finder made of the
//                picture
//   --blind      no camera, the stage-3 baseline
//   --no-noise   exact pictures (default: the camera's noise on)
//   --calibration MM,DEG  the camera's calibration off by MM millimetres and
//                DEG degrees, in directions drawn for the seed
//   --seed N     the trial's seed (default 1)
//   --nominal    the pallet exactly where it was designed and the conveyor
//                stopping exactly; without it the pallet is set down off
//                nominal as cell::Randomization's defaults allow
//   --modules M  how many modules to pick, module_0 first (default 1)
//
// The corner text shows the module being picked, its slot, the script's
// current step, the last picture's outcome (the finder's estimate, and its
// error against where the module truly stood), and after each placement its
// score, measured from the truth (src/score.h) at the moment the tool has
// retreated. The script never reads the truth; only this program does, to
// show the scores.
// Backspace starts the trial again from its Reset. The other keys are the
// viewer's (src/viewer_lib.h). Needs a display.
//
// The first argument is the cell's model; BUILD.bazel passes it.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>

#include <mujoco/mujoco.h>

#include "src/cell.h"
#include "src/cycle_check.h"
#include "src/module_finder.h"
#include "src/pick_place.h"
#include "src/score.h"
#include "src/viewer_lib.h"

int main(int argc, char** argv) {
  const char* usage =
      "usage: %s <cell.xml> [--seed N] [--nominal] [--modules M] "
      "[--camera | --blind] [--noise | --no-noise] [--calibration MM,DEG]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  long seed = 1;
  bool nominal = false, camera = true, noise = true;
  double calibration_mm = 0.0, calibration_deg = 0.0;
  int modules = 1;
  for (int i = 2; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--nominal") == 0) {
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
    } else if (std::strcmp(argv[i], "--seed") == 0 && has_value) {
      seed = std::atol(argv[++i]);
    } else if (std::strcmp(argv[i], "--modules") == 0 && has_value) {
      modules = std::atoi(argv[++i]);
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
  if (modules < 1 || modules > cell.module_count() || seed < 0) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  constexpr double kDegree = 3.14159265358979 / 180.0;
  cell::Randomization randomization =
      nominal ? cell::Randomization::None() : cell::Randomization{};
  randomization.camera_noise = camera && noise;
  randomization.camera_shift = calibration_mm * 1e-3;
  randomization.camera_turn = calibration_deg * kDegree;
  cell::PickPlaceOptions options;
  options.sensing =
      camera ? cell::SensingMode::kCamera : cell::SensingMode::kBlind;
  // The finder's error for the estimate the current pick uses, from the
  // truth, for the corner text.
  std::string seen;

  std::optional<cell::PickPlace> script;
  int next = 0;           // the next module to pick
  std::string scores;     // one line per placement so far
  int placed = 0;

  // A fresh trial: Reset (which settles the scene for a second), a new
  // script, and module_0's cycle started.
  auto start_trial = [&]() {
    cell.Reset(static_cast<std::uint32_t>(seed), randomization);
    script.emplace(cell, options);
    script->set_placed_hook([&](int module, int slot) {
      mj_forward(cell.model(), cell.data());
      const cell::PlacementError e = cell::MeasurePlacement(cell, module, slot);
      char line[200];
      std::snprintf(line, sizeof(line),
                    "module_%d in slot %d: along %+.2f  across %+.2f  height "
                    "%+.2f mm  yaw %+.3f  tilt %.3f deg\n",
                    module, slot, 1e3 * e.along_tube, 1e3 * e.across_tube,
                    1e3 * e.height, e.yaw * 180.0 / 3.14159265358979,
                    e.tilt * 180.0 / 3.14159265358979);
      scores += line;
      ++placed;
    });
    scores.clear();
    seen.clear();
    placed = 0;
    next = 0;
    script->Start(next++);
  };
  start_trial();

  cell::ViewerHooks hooks;
  hooks.step = [&]() {
    // The next module as soon as the last one is placed; the script waits
    // for the conveyor itself.
    if (script->Done() &&
        script->outcome().status == cell::PickPlaceOutcome::Status::kPlaced &&
        next < modules) {
      seen.clear();
      script->Start(next++);
    }
    script->Update();
    cell.Step();
    if (seen.empty() && script->estimate()) {
      const cell::EstimateError e = cell::EstimateErrorOf(
          script->estimate()->pose,
          cell.TrueModulePose(script->outcome().module));
      char line[200];
      std::snprintf(line, sizeof(line),
                    "  off the truth: normal %+.2f, long side %+.2f, up the "
                    "lean %+.2f mm, face %.3f, turn %+.3f deg\n",
                    1e3 * e.normal, 1e3 * e.long_side, 1e3 * e.up_lean,
                    e.face_angle / kDegree, e.in_plane / kDegree);
      seen = line;
    }
  };
  hooks.reset = start_trial;
  hooks.status = [&]() {
    const cell::TrueScene& scene = cell.true_scene();
    const cell::PickPlaceOutcome& outcome = script->outcome();
    char text[400];
    std::snprintf(text, sizeof(text),
                  "seed %ld%s: pallet off by %+.1f, %+.1f mm, %+.2f deg\n"
                  "module_%d -> slot %d   step: %s   (%.1f s into the cycle)\n",
                  seed, nominal ? " (nominal)" : "", 1e3 * scene.pallet_x,
                  1e3 * scene.pallet_y,
                  scene.pallet_yaw * 180.0 / 3.14159265358979, outcome.module,
                  outcome.slot, script->step_name(),
                  (script->Done() ? outcome.end_time : cell.data()->time) -
                      outcome.start_time);
    std::string status = text;
    if (camera) {
      // The pictures of this cycle: the look step's, and the look ahead's.
      for (const cell::LookRecord& look : script->looks()) {
        char line[400];
        if (look.estimate) {
          std::snprintf(line, sizeof(line),
                        "%s picture at %.1f s: module found (finder %.0f ms, "
                        "camera %.2f s)\n",
                        cell::StepName(look.step), look.time,
                        1e3 * look.diagnostics.seconds_total,
                        look.render_seconds);
        } else {
          std::snprintf(line, sizeof(line),
                        "%s picture at %.1f s: refused, %s\n",
                        cell::StepName(look.step), look.time,
                        cell::FinderFailureName(look.diagnostics.failure));
        }
        status += line;
      }
      if (script->step() == cell::PickPlaceStep::kLook) {
        status += "  looking: holding still for the picture, or moving to "
                  "the look pose\n";
      }
      status += seen;
    }
    if (outcome.status == cell::PickPlaceOutcome::Status::kFailed) {
      status += "FAILED: " + outcome.reason + "\n";
    }
    status += scores;
    if (script->Done() && next >= modules &&
        outcome.status == cell::PickPlaceOutcome::Status::kPlaced) {
      status += "all " + std::to_string(placed) +
                " placed; Backspace runs the trial again\n";
    }
    return status;
  };
  return cell::RunViewer(cell.model(), cell.data(), hooks,
                         "solar-panel-pick-place: pick and place");
}
