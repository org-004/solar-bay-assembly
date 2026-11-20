// Draws a robot model in a window and simulates it in real time.
//
//   bazel run //:viewer
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
// The window itself is in viewer_lib.h; this program only loads the model
// named on the command line and hands it over. Needs a display.

#include <cstdio>
#include <string>

#include <mujoco/mujoco.h>

#include "src/mj_handles.h"
#include "src/viewer_lib.h"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: %s <model.xml>\n", argv[0]);
    return 1;
  }

  std::string error;
  cell::ModelPtr model = cell::LoadModel(argv[1], &error);
  if (!model) {
    std::fprintf(stderr, "could not load %s: %s\n", argv[1], error.c_str());
    return 1;
  }
  cell::DataPtr data = cell::MakeData(*model);
  // Start where Backspace goes back to: the first keyframe, if there is one.
  if (model->nkey > 0) {
    mj_resetDataKeyframe(model.get(), data.get(), 0);
  }

  return cell::RunViewer(model.get(), data.get());
}
