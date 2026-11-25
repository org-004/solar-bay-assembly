// Draws a robot model in a window and simulates it in real time.
//
//   bazel run //:viewer
//   bazel run //:viewer_bay -- --lookat 0,0,1.2 --distance 9 --azimuth 140
//       --elevation -25 --no-overlay --size 1200x900 --record out.raw
//       --fps 1 --quit-at 0
//
// Left drag rotates the camera, right drag moves it, scroll zooms.
// Space pauses, and S then steps one frame at a time. Backspace resets the
// simulation. [ and ] cycle through the model's fixed cameras and back to
// the free camera. Escape quits.
//
// If the model has keyframes, it starts from the first one, and Backspace
// goes back to it: a scene with the arm in it then shows the arm at
// arm_home, holding it, instead of stretched out at all joints zero.
//
// Options after the model's path, for pictures and recordings without the
// mouse (same spelling and meaning as in view_cell; src/viewer_lib.h has the
// details):
//   --key NAME_OR_INDEX  start from that keyframe instead of the first
//                (Backspace goes back to it); an unknown one is an error
//   --groups LIST  geom and site groups, comma-separated numbers 0 to 5, to
//                draw in addition to the defaults (0, 1, 2): --groups 3
//                shows the bay's see-through bay_hold_* sites
//   --lookat X,Y,Z --distance D --azimuth DEG --elevation DEG
//                the free camera's start (default: the whole scene)
//   --speed K    K simulated seconds per second (default 1)
//   --overlay full|none  the corner line with the time, speed and camera
//                (the default), or nothing; --no-overlay is --overlay none
//   --overlay-at top|bottom  the corner text top left (the default) or
//                bottom left
//   --caption TEXT  text in the top-right corner
//   --font-scale P  the size of all text in percent: 50, 100, ..., 300
//                (default 150)
//   --size WxH   the window's size (default 1200x900)
//   --record PATH --fps F  a recording: each frame is speed / F simulated
//                seconds, the wall clock ignored, appended to PATH as raw
//                RGB24 frames (default F 15); tools/video/shot_viewer.sh
//                makes a PNG of it
//   --quit-at T  closes T simulated seconds after the start
//
// The window itself is in viewer_lib.h; this program only loads the model
// named on the command line and hands it over. Needs a display.

#include <cstdio>
#include <string>

#include <mujoco/mujoco.h>

#include "src/mj_handles.h"
#include "src/viewer_lib.h"

int main(int argc, char** argv) {
  auto usage = [&]() {
    std::fprintf(stderr, "usage: %s <model.xml>\n%s", argv[0],
                 cell::ViewerArgsUsage());
  };
  if (argc < 2) {
    usage();
    return 1;
  }
  cell::ViewerOptions options;
  cell::ViewerRun run;
  std::string error;
  if (!cell::ParseViewerArgs(argc, argv, 2, &options, &run, &error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    usage();
    return 1;
  }

  cell::ModelPtr model = cell::LoadModel(argv[1], &error);
  if (!model) {
    std::fprintf(stderr, "could not load %s: %s\n", argv[1], error.c_str());
    return 1;
  }
  int key = -1;
  if (!cell::ResolveKeyframe(model.get(), run.key, &key, &error)) {
    std::fprintf(stderr, "%s\n", error.c_str());
    usage();
    return 1;
  }
  cell::DataPtr data = cell::MakeData(*model);
  // Start where Backspace goes back to: the chosen keyframe, else the first.
  cell::ViewerHooks hooks;
  if (key >= 0) {
    mj_resetDataKeyframe(model.get(), data.get(), key);
    hooks.reset = [&]() { mj_resetDataKeyframe(model.get(), data.get(), key); };
  }
  if (run.quit_at >= 0.0) {
    const double start = data->time;
    hooks.quit = [&]() { return data->time - start >= run.quit_at; };
  }

  return cell::RunViewer(model.get(), data.get(), hooks, "solar-bay-assembly",
                         options);
}
