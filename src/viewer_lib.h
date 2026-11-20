// A window that draws a MuJoCo simulation and steps it in real time.
//
// Programs that want to watch a simulation call RunViewer with their model
// and state, and optionally say how to step it, how to reset it, and what
// text to show. src/viewer.cc is the simplest such program.
//
// Mouse and keys in the window:
//   Left drag    rotates the camera
//   Right drag   moves the camera
//   Scroll       zooms
//   Space        pauses and resumes
//   S            while paused, steps one frame (1/60 s of simulated time)
//   Backspace    resets the simulation
//   [ and ]      cycle through the model's fixed cameras and back to the
//                free camera
//   Escape       quits
//
// The free camera starts framed on the whole scene. Mouse moves only apply
// to the free camera; a fixed camera stays where the model puts it.
//
// GLFW opens the window and delivers keyboard and mouse events. MuJoCo draws
// into the window's OpenGL context. Needs a display.

#ifndef SRC_VIEWER_LIB_H_
#define SRC_VIEWER_LIB_H_

#include <functional>
#include <string>

#include <mujoco/mujoco.h>

namespace cell {

struct ViewerHooks {
  // Advances the simulation by one physics step. If empty, the viewer calls mj_step.
  std::function<void()> step;
  // Puts the simulation back at its start (Backspace). If empty, the viewer resets to keyframe 0 if the model has one, else to the default state.
  std::function<void()> reset;
  // One or more lines of text drawn in a corner of the window each frame. If empty, nothing is drawn.
  std::function<std::string()> status;
};

// Opens a window and runs until it is closed. Returns 0, or 1 if no window could be opened.
//
// The viewer draws `data` as it finds it: it does not reset it first, so the
// caller sets up the starting state. All hooks are called on the calling
// thread, between frames.
//
// The viewer tries to keep simulated time in step with the wall clock. When
// stepping is too slow for that, it steps for a fixed share of each frame and
// lets simulated time fall behind, so the window keeps responding; the
// corner text shows how fast the simulation is actually running.
int RunViewer(const mjModel* model, mjData* data, const ViewerHooks& hooks = {}, const char* title = "solar-panel-pick-place");

}  // namespace cell

#endif  // SRC_VIEWER_LIB_H_
