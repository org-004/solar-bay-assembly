// Draws a layout of the cell (layouts/<name>.json) in a window.
//
//   bazel run --config=opt //:viewer_layout -- below
//   bazel run --config=opt //:viewer_layout -- below --state bare
//   bazel run --config=opt //:viewer_layout -- below --rails 4 --modules 0
//   bazel run --config=opt //:viewer_layout -- below --at rail_set:3
//   bazel run --config=opt //:viewer_layout -- flat_rack_3_pin_arms --play
//   bazel run --config=opt //:viewer_layout -- flat_rack_3_pin_arms --play --speed 8 --from-time 300
//   tools/video/shot_viewer.sh //:viewer_layout out.png 1600x1000 below \
//       --state finished --lookat 0,0,1.2 --distance 16 --azimuth 130 \
//       --elevation -25 --no-overlay --quit-at 0
//
// The layout's name is the one word that is not an option. Options of this
// program:
//   --state bare|finished|stowed  a bare tube in the holder, the finished
//                bay (the default), or the transport state (only what
//                travels inside, the arms at their stow poses)
//   --rails N --modules M  the bay as far as N rails and M modules (after
//                --state, which they override)
//   --at KIND:INDEX  the state with the arm at that work point, as
//                cell::layout::StateAt of src/layout_check.h finds it
//   --physics    runs one module's take and set of the plan in physics
//                (src/physics_run.h) and shows the physics itself: the module
//                arm on its servos, the module in its grip, the rails it is
//                set on. The same checks and the same plan file as --play.
//     --take K       the module_take index whose take and set are run (default
//                    the hardest carry of the plan, as //:run_physics)
//                The run is 1 s of hold, the plan's take and set step end to
//                end, and 2 s of settling (about 25 s of simulated time); the
//                viewer's clock is the physics clock, and the overlay shows
//                the plan time. With --record each frame advances exactly
//                speed / fps seconds of physics; the recording ends with the
//                run. Backspace does nothing here. --speed is the viewer's
//                own (physics seconds per viewer second), not --play's.
//                tools/video/physics_gif.sh makes the GIF.
//   --play       plays the committed plan of the layout (plans/<name>.json)
//                from its first step to its last: every arm follows its
//                planned moves at the schedule's times, a held part moves
//                with its tool, a part leaves its stock when taken and stays
//                in the bay when set, the tube comes in and the bay leaves
//                along the layout's ways. The layout is checked first (about
//                3 s); the plan must be the one of the layout's file (its
//                sha256), else the program says so and exits 1. Options
//                that go with it (the others above do not apply):
//     --speed N      schedule seconds per viewer second (default 1)
//     --from-time T  the schedule time at the start (default 0)
//     --to-time T    the schedule time at which the playback holds its last
//                    instant (default the plan's end)
//                The overlay shows the schedule time and, for each arm, its
//                step and the kind of segment it is in. With --from-time T
//                --quit-at 0 and --record, one frame of the cell at schedule
//                time T is written, which tools/video/shot_viewer.sh turns
//                into a picture. tools/video/play_gif.sh makes a GIF.
//                Parts that are not there at the instant are not drawn.
// Every other option is the plain viewer's (src/viewer.cc, src/viewer_lib.h):
// the camera pose, --size, --record, --quit-at, --groups, ... --groups 3
// shows the arms' collision geoms, --groups 4 the flange and work sites.
//
// Without --play or --physics the model is only looked at: nothing moves. The viewer's step hook
// advances the clock and recomputes positions (mj_forward) without stepping
// the physics, so the arms hold the state's pose without motors and without
// falling, and --quit-at still counts simulated time. Needs a display.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_plan_stages.h"
#include "src/layout_scene.h"
#include "src/mj_handles.h"
#include "src/physics_run.h"
#include "src/viewer_lib.h"

namespace {

bool ParseInt(const std::string& text, int* out) {
  char* end = nullptr;
  const long v = std::strtol(text.c_str(), &end, 10);
  if (text.empty() || *end != '\0') return false;
  *out = static_cast<int>(v);
  return true;
}

bool ParseDouble(const std::string& text, double* out) {
  char* end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  if (text.empty() || *end != '\0') return false;
  *out = v;
  return true;
}

bool Contains(const std::vector<int>& v, int x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

// The committed plan of the layout, which must be the one made for the
// layout's file (its sha256). Prints the fault and returns nothing otherwise.
std::optional<cell::layout::Plan> LoadCommittedPlan(const std::string& name) {
  using namespace cell::layout;
  const char* plan_dir = std::getenv("PLAN_DIR");
  const std::string plan_path =
      std::string(plan_dir != nullptr ? plan_dir : "plans") + "/" + name + ".json";
  const char* layout_dir = std::getenv("LAYOUT_DIR");
  const std::string layout_path =
      std::string(layout_dir != nullptr ? layout_dir : "layouts") + "/" + name + ".json";
  std::ifstream in(plan_path);
  if (!in) {
    std::fprintf(stderr, "there is no plan for layout %s (%s is missing)\n",
                 name.c_str(), plan_path.c_str());
    return std::nullopt;
  }
  std::stringstream text;
  text << in.rdbuf();
  std::string error;
  std::optional<Plan> plan = PlanFromJson(text.str(), &error);
  if (!plan) {
    std::fprintf(stderr, "%s: %s\n", plan_path.c_str(), error.c_str());
    return std::nullopt;
  }
  if (plan->layout_sha256 != Sha256OfFile(layout_path)) {
    std::fprintf(stderr,
                 "the plan %s was made for another version of %s: plan it again "
                 "with //:plan_layout\n",
                 plan_path.c_str(), layout_path.c_str());
    return std::nullopt;
  }
  return plan;
}

struct PlayOptions {
  double speed = 1.0;
  double from = 0.0;
  double to = -1.0;
};

// Plays the committed plan of the layout in the viewer. Returns the exit code.
int Play(const cell::layout::Layout& layout, const std::string& name,
         const PlayOptions& play, const cell::ViewerOptions& options,
         const cell::ViewerRun& run) {
  using namespace cell::layout;
  std::optional<Plan> plan = LoadCommittedPlan(name);
  if (!plan) return 1;
  std::string error;
  const ModelPaths paths = ModelPaths::FromEnvironment();
  const CheckReport report = CheckLayout(layout, paths);
  if (!report.error.empty()) {
    std::fprintf(stderr, "%s\n", report.error.c_str());
    return 1;
  }
  // Staged work: the first stage's scene, as the verifier plays it
  // (src/layout_plan_stages.h).
  std::unique_ptr<LayoutScene> scene =
      LayoutScene::Load(LayoutOfStage(layout, 0), paths, report.joint_margin_deg, &error);
  if (!scene) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  const PlanPlayer player(*scene, report, *plan);
  const double end = player.duration();
  const double from = std::clamp(play.from, 0.0, end);
  const double to = play.to < 0.0 ? end : std::clamp(play.to, from, end);

  // A private copy of the model, so that colours can change.
  cell::ModelPtr model(mj_copyModel(nullptr, &scene->model()));
  cell::DataPtr data = cell::MakeData(*model);
  const LayoutHandles& h = scene->handles();

  // Every geom of the body of any geom in the list: a part is hidden wholly.
  auto whole = [&](const std::vector<int>& geoms) {
    std::set<int> bodies;
    for (int g : geoms) bodies.insert(model->geom_bodyid[g]);
    std::vector<int> out = geoms;
    for (int g = 0; g < model->ngeom; ++g) {
      if (bodies.count(model->geom_bodyid[g]) != 0 && !Contains(out, g)) out.push_back(g);
    }
    return out;
  };
  const std::vector<int> tube = whole(h.tube_geoms);
  std::vector<std::vector<int>> rails, modules, pallet_modules;
  for (const auto& g : h.rail_geoms) rails.push_back(whole(g));
  for (const auto& g : h.module_geoms) modules.push_back(whole(g));
  for (const auto& g : h.pallet_module_geoms) pallet_modules.push_back(whole(g));
  const int arm_count = static_cast<int>(h.arms.size());
  std::vector<std::map<std::string, std::vector<int>>> tool_geoms(arm_count),
      waiting_geoms(arm_count), held_geoms(arm_count);
  for (int a = 0; a < arm_count; ++a) {
    for (const auto& [k, g] : h.arms[a].tool_geoms) tool_geoms[a][k] = whole(g);
    for (const auto& [k, g] : h.arms[a].waiting_tool_geoms) waiting_geoms[a][k] = whole(g);
    for (const auto& [k, g] : h.arms[a].held_geoms) held_geoms[a][k] = whole(g);
  }
  // Every geom this player shows or hides, with its own alpha at the start.
  std::map<int, float> alpha;
  auto remember = [&](const std::vector<int>& geoms) {
    for (int g : geoms) alpha.emplace(g, model->geom_rgba[4 * g + 3]);
  };
  remember(tube);
  for (const auto& g : rails) remember(g);
  for (const auto& g : modules) remember(g);
  for (const auto& g : pallet_modules) remember(g);
  for (int a = 0; a < arm_count; ++a) {
    for (const auto& [k, g] : tool_geoms[a]) remember(g);
    for (const auto& [k, g] : waiting_geoms[a]) remember(g);
    for (const auto& [k, g] : held_geoms[a]) remember(g);
  }
  // The probes of the free spaces are never drawn.
  std::vector<int> probes;
  for (int g : {h.probe_bolt_geom, h.probe_pin_geom, h.probe_locate_below_geom,
                h.probe_locate_above_geom}) {
    if (g >= 0) probes.push_back(g);
  }
  for (int g : probes) model->geom_rgba[4 * g + 3] = 0.0f;
  std::map<int, const StepTime*> step_of;
  for (const StepTime& s : plan->steps) step_of[s.number] = &s;

  double start = 0.0;
  // A recording steps the clock by one frame before it draws the first, so
  // the clock of the first frame is that far ahead: the schedule starts then.
  const double lead =
      options.record_path.empty() ? 0.0 : options.speed / options.record_fps;
  std::vector<std::string> lines;
  // Puts the cell at the schedule time that the viewer's clock says.
  auto apply = [&]() {
    const double t =
        std::clamp(from + play.speed * (data->time - start - lead), from, to);
    const WorldAt w = player.At(t);
    player.SetPoses(w, data.get());
    mj_kinematics(model.get(), data.get());
    std::set<int> visible;
    auto show = [&](const std::vector<int>& g) { visible.insert(g.begin(), g.end()); };
    if (w.tube) show(tube);
    for (int k : w.rails) {
      if (k >= 0 && k < static_cast<int>(rails.size())) show(rails[k]);
    }
    for (int m : w.modules) {
      if (m >= 0 && m < static_cast<int>(modules.size())) show(modules[m]);
    }
    for (int j = 0; j < static_cast<int>(pallet_modules.size()); ++j) {
      if (!Contains(w.pallet_taken, j)) show(pallet_modules[j]);
    }
    for (int a = 0; a < arm_count && a < static_cast<int>(w.arms.size()); ++a) {
      const ArmAt& at = w.arms[a];
      for (const auto& [k, g] : tool_geoms[a]) {
        if (k == at.tool) show(g);
      }
      for (const auto& [k, g] : waiting_geoms[a]) {
        if (k != at.tool) show(g);
      }
      if (at.held == "module" || at.held == "rail") show(held_geoms[a][at.held]);
    }
    for (const auto& [g, a0] : alpha) {
      model->geom_rgba[4 * g + 3] = visible.count(g) != 0 ? a0 : 0.0f;
    }
    // The tube on its way in, and the bay on its way out, move along the way;
    // in staged work the tube and the bay built so far stand at the stage's
    // place, and move with the tube through a transfer.
    if (w.event == "tube_in" || w.event == "bay_out") {
      std::set<int> moved(tube.begin(), tube.end());
      if (w.event == "bay_out") {
        for (const auto& g : rails) moved.insert(g.begin(), g.end());
        for (const auto& g : modules) moved.insert(g.begin(), g.end());
      }
      for (int g : moved) {
        for (int i = 0; i < 3; ++i) data->geom_xpos[3 * g + i] += w.way_offset[i];
      }
    }
    if (w.stage_offset != Eigen::Vector3d::Zero()) {
      std::set<int> staged(tube.begin(), tube.end());
      for (const auto& g : rails) staged.insert(g.begin(), g.end());
      for (const auto& g : modules) staged.insert(g.begin(), g.end());
      for (int g : staged) {
        for (int i = 0; i < 3; ++i) data->geom_xpos[3 * g + i] += w.stage_offset[i];
      }
    }
    lines.clear();
    char line[200];
    std::snprintf(line, sizeof(line), "t = %.2f s of %.2f s, x%g", t, end, play.speed);
    lines.push_back(line);
    for (int a = 0; a < arm_count && a < static_cast<int>(w.arms.size()); ++a) {
      const ArmAt& at = w.arms[a];
      const auto it = step_of.find(at.step);
      lines.push_back(h.arms[a].name + ": step " + std::to_string(at.step) + " " +
                      (it != step_of.end() ? it->second->what : std::string("")) +
                      ", " + at.kind);
    }
    if (!w.event.empty()) lines.push_back("bay: " + w.event);
  };
  auto reset = [&]() {
    mj_resetData(model.get(), data.get());
    start = data->time;
    apply();
  };
  reset();
  cell::ViewerHooks hooks;
  hooks.reset = reset;
  hooks.step = [&]() {
    data->time += model->opt.timestep;
    apply();
  };
  const bool overlay = options.viewer_line;
  hooks.status = [&]() {
    std::string out;
    if (!overlay) return out;
    for (const std::string& l : lines) out += l + "\n";
    return out;
  };
  if (run.quit_at >= 0.0) {
    hooks.quit = [&]() { return data->time - start >= run.quit_at; };
  }
  const std::string title = "layout " + name + " (plan)";
  return cell::RunViewer(model.get(), data.get(), hooks, title.c_str(), options);
}

// Runs one module's take and set of the committed plan in physics and shows
// it. Returns the exit code.
int Physics(const cell::layout::Layout& layout, const std::string& name, int take,
            const cell::ViewerOptions& options, const cell::ViewerRun& run) {
  using namespace cell::layout;
  const std::optional<Plan> plan = LoadCommittedPlan(name);
  if (!plan) return 1;
  PhysicsOptions physics_options;
  physics_options.take = take;
  std::string error;
  std::unique_ptr<PhysicsRun> physics = PhysicsRun::Create(
      layout, *plan, ModelPaths::FromEnvironment(), physics_options, &error);
  if (!physics) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  cell::ViewerHooks hooks;
  // The run cannot be put back at its start while the viewer holds its model.
  hooks.reset = []() {};
  hooks.step = [&]() { physics->Step(); };
  const bool overlay = options.viewer_line;
  hooks.status = [&]() {
    std::string out;
    if (!overlay) return out;
    char line[100];
    std::snprintf(line, sizeof(line), "physics, plan time %.2f s%s", physics->PlanTime(),
                  physics->Ended() ? " (ended)" : "");
    return std::string(line) + "\n";
  };
  // A recording ends with the run; a window stays open on its last state.
  const bool recording = !options.record_path.empty();
  hooks.quit = [&]() {
    if (run.quit_at >= 0.0 && physics->data()->time >= run.quit_at) return true;
    return recording && physics->Ended();
  };
  const std::string title = "layout " + name + " (physics)";
  return cell::RunViewer(&physics->model(), physics->data(), hooks,
                         title.c_str(), options);
}

}  // namespace

int main(int argc, char** argv) {
  using namespace cell::layout;
  auto usage = [&]() {
    std::fprintf(stderr,
                 "usage: %s <layout name> [--state bare|finished|stowed] "
                 "[--rails N] [--modules M] [--at KIND:INDEX]\n"
                 "       %s <layout name> --play [--speed N] [--from-time T] "
                 "[--to-time T] [viewer options]\n"
                 "       %s <layout name> --physics [--take K] [viewer options]\n%s",
                 argv[0], argv[0], argv[0], cell::ViewerArgsUsage());
  };
  // Take this program's options and the layout's name out; pass the rest on.
  std::string name, state_name = "finished", at;
  int rails = -1, modules = -1;
  bool play = false, physics = false;
  int take = -1;
  PlayOptions play_options;
  std::string speed_text;
  std::vector<char*> rest = {argv[0]};
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto value = [&](std::string* out) {
      if (i + 1 >= argc) return false;
      *out = argv[++i];
      return true;
    };
    std::string v;
    if (arg == "--state") {
      if (!value(&state_name)) return usage(), 1;
    } else if (arg == "--rails") {
      if (!value(&v) || !ParseInt(v, &rails)) return usage(), 1;
    } else if (arg == "--modules") {
      if (!value(&v) || !ParseInt(v, &modules)) return usage(), 1;
    } else if (arg == "--play") {
      play = true;
    } else if (arg == "--physics") {
      physics = true;
    } else if (arg == "--take") {
      if (!value(&v) || !ParseInt(v, &take) || take < 0) return usage(), 1;
    } else if (arg == "--speed") {
      if (!value(&speed_text) || !ParseDouble(speed_text, &play_options.speed) ||
          play_options.speed <= 0.0) {
        return usage(), 1;
      }
    } else if (arg == "--from-time") {
      if (!value(&v) || !ParseDouble(v, &play_options.from)) return usage(), 1;
    } else if (arg == "--to-time") {
      if (!value(&v) || !ParseDouble(v, &play_options.to)) return usage(), 1;
    } else if (arg == "--at") {
      if (!value(&at)) return usage(), 1;
    } else if (arg.rfind("--", 0) == 0) {
      rest.push_back(argv[i]);
      // Every viewer option takes one value, except --no-overlay.
      if (arg != "--no-overlay" && i + 1 < argc) rest.push_back(argv[++i]);
    } else if (name.empty()) {
      name = arg;
    } else {
      std::fprintf(stderr, "more than one layout name: %s and %s\n", name.c_str(),
                   arg.c_str());
      return usage(), 1;
    }
  }
  if (name.empty()) return usage(), 1;
  if (play && physics) {
    std::fprintf(stderr, "--play and --physics do not go together\n");
    return usage(), 1;
  }
  if (!play && !speed_text.empty()) {
    rest.push_back(const_cast<char*>("--speed"));
    rest.push_back(const_cast<char*>(speed_text.c_str()));
  }
  cell::ViewerOptions options;
  cell::ViewerRun run;
  std::string error;
  if (!cell::ParseViewerArgs(static_cast<int>(rest.size()), rest.data(), 1, &options,
                             &run, &error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return usage(), 1;
  }

  const char* dir = std::getenv("LAYOUT_DIR");
  const std::string path = std::string(dir != nullptr ? dir : "layouts") + "/" + name + ".json";
  std::optional<Layout> layout = LoadLayout(path, &error);
  if (!layout) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  if (physics) return Physics(*layout, name, take, options, run);
  if (play) return Play(*layout, name, play_options, options, run);
  const ModelPaths paths = ModelPaths::FromEnvironment();

  LayoutState state;
  if (state_name == "bare") {
    state.rails = 0;
    state.modules = 0;
  } else if (state_name == "finished") {
    state.rails = cell::bay::kRailCount;
    state.modules = cell::bay::kModuleCount;
  } else if (state_name == "stowed") {
    state.stowed = true;
  } else {
    std::fprintf(stderr, "--state is bare, finished or stowed, not %s\n", state_name.c_str());
    return usage(), 1;
  }
  if (rails >= 0) state.rails = rails;
  if (modules >= 0) state.modules = modules;
  if (!at.empty()) {
    const size_t colon = at.find(':');
    int index = 0;
    if (colon == std::string::npos || !ParseInt(at.substr(colon + 1), &index)) {
      std::fprintf(stderr, "--at takes KIND:INDEX, as rail_set:3\n");
      return usage(), 1;
    }
    if (!StateAt(*layout, paths, at.substr(0, colon), index, &state, &error)) {
      std::fprintf(stderr, "--at %s: %s\n", at.c_str(), error.c_str());
      return 1;
    }
  }

  cell::ModelPtr model = LoadLayoutModel(*layout, state, paths, &error);
  if (!model) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  int key = -1;
  if (!cell::ResolveKeyframe(model.get(), run.key, &key, &error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return usage(), 1;
  }
  cell::DataPtr data = cell::MakeData(*model);
  auto reset = [&]() {
    if (key >= 0) {
      mj_resetDataKeyframe(model.get(), data.get(), key);
    } else {
      mj_resetData(model.get(), data.get());
    }
    mj_forward(model.get(), data.get());
  };
  reset();
  cell::ViewerHooks hooks;
  hooks.reset = reset;
  // Nothing moves: the clock runs and the positions are recomputed.
  hooks.step = [&]() {
    data->time += model->opt.timestep;
    mj_forward(model.get(), data.get());
  };
  if (run.quit_at >= 0.0) {
    const double start = data->time;
    hooks.quit = [&]() { return data->time - start >= run.quit_at; };
  }
  const std::string title = "layout " + name;
  return cell::RunViewer(model.get(), data.get(), hooks, title.c_str(), options);
}
