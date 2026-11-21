// A window that draws a MuJoCo simulation and steps it in real time.
//
// Programs that want to watch a simulation call RunViewer with their model
// and state, and optionally say how to step it, how to reset it, and what
// text to show. src/viewer.cc is the simplest such program.
// ViewerOptions sets the starting view, a speed other than real time, and a
// recording to a file, for pictures made without the mouse.
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

#include <array>
#include <functional>
#include <optional>
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
  // Asked after each frame; when it returns true the viewer closes as if Escape were pressed. If empty, only the window or Escape closes it.
  std::function<bool()> quit;
};

// How the viewer starts and runs, for programs that want a set view or a
// recording without touching the mouse. The defaults are what RunViewer
// without options does.
struct ViewerOptions {
  // The free camera's start. Each field left unset keeps the default framing
  // of the whole scene. Azimuth and elevation are in degrees as MuJoCo's free
  // camera takes them: azimuth 0 looks along +X, 90 along +Y; a negative
  // elevation looks down.
  std::optional<std::array<double, 3>> lookat;
  std::optional<double> distance;
  std::optional<double> azimuth;
  std::optional<double> elevation;
  // Simulated seconds wanted per wall-clock second (or, when recording, per
  // second of the recording).
  double speed = 1.0;
  // The viewer's own corner line (simulated time, speed, camera).
  bool viewer_line = true;
  // The corner text in the bottom-left corner instead of the top-left.
  bool text_at_bottom = false;
  // Text drawn in the top-right corner. Empty draws nothing.
  std::string caption;
  // The size of all text, in percent: one of 50, 100, ..., 300 (mjtFontScale).
  int font_scale = 150;
  // The window's size in screen pixels.
  int width = 1200;
  int height = 900;
  // When set, a recording: each frame advances exactly speed / record_fps
  // simulated seconds whatever the wall clock says, and its pixels are
  // appended to this file as raw RGB24, top row first, at the window's
  // framebuffer size (printed to stderr at the start). Read it with
  //   ffmpeg -f rawvideo -pix_fmt rgb24 -video_size WxH -framerate FPS -i PATH
  // A named pipe works.
  std::string record_path;
  double record_fps = 15.0;
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
// The same, started and run as `options` say. Returns 1 also if the recording
// file cannot be opened or written.
int RunViewer(const mjModel* model, mjData* data, const ViewerHooks& hooks, const char* title, const ViewerOptions& options);

}  // namespace cell

#endif  // SRC_VIEWER_LIB_H_
