// Draws a layout of the cell (layouts/<name>.json) in a window.
//
//   bazel run --config=opt //:viewer_layout -- below
//   bazel run --config=opt //:viewer_layout -- below --state bare
//   bazel run --config=opt //:viewer_layout -- below --rails 4 --modules 0
//   bazel run --config=opt //:viewer_layout -- below --at rail_set:3
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
// Every other option is the plain viewer's (src/viewer.cc, src/viewer_lib.h):
// the camera pose, --size, --record, --quit-at, --groups, ... --groups 3
// shows the arms' collision geoms, --groups 4 the flange and work sites.
//
// The model is only looked at: nothing moves. The viewer's step hook
// advances the clock and recomputes positions (mj_forward) without stepping
// the physics, so the arms hold the state's pose without motors and without
// falling, and --quit-at still counts simulated time. Needs a display.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/mj_handles.h"
#include "src/viewer_lib.h"

namespace {

bool ParseInt(const std::string& text, int* out) {
  char* end = nullptr;
  const long v = std::strtol(text.c_str(), &end, 10);
  if (text.empty() || *end != '\0') return false;
  *out = static_cast<int>(v);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace cell::layout;
  auto usage = [&]() {
    std::fprintf(stderr,
                 "usage: %s <layout name> [--state bare|finished|stowed] "
                 "[--rails N] [--modules M] [--at KIND:INDEX] [viewer options]\n%s",
                 argv[0], cell::ViewerArgsUsage());
  };
  // Take this program's options and the layout's name out; pass the rest on.
  std::string name, state_name = "finished", at;
  int rails = -1, modules = -1;
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
