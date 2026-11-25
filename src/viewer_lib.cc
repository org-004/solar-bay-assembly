// The window, mouse, keys and render loop behind RunViewer. See viewer_lib.h
// for what the keys do.

#include "src/viewer_lib.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

namespace cell {
namespace {

// Simulated time that S advances while paused: one frame at 60 frames a
// second, the usual screen refresh.
constexpr double kFrameTime = 1.0 / 60.0;

// Wall-clock time each frame may spend stepping. Past this the viewer stops
// stepping for the frame and draws, so a scene that simulates slower than
// real time still redraws and answers the mouse about 30 times a second.
constexpr double kStepBudget = 0.02;

// Most simulated time the viewer will try to catch up on in one frame. After
// a stall (the window being dragged, the machine busy) the simulation picks
// up where it was instead of racing to make up the lost time.
constexpr double kMaxBacklog = 0.1;

// How often the real-time factor in the corner is recomputed. Measuring over
// half a second keeps the number steady enough to read.
constexpr double kSpeedInterval = 0.5;

// Everything the event callbacks need. GLFW callbacks are plain C functions
// with fixed arguments, so the window carries a pointer to this struct and
// each callback fetches it back.
struct Viewer {
  const mjModel* model = nullptr;
  mjData* data = nullptr;
  const ViewerHooks* hooks = nullptr;
  const ViewerOptions* options_in = nullptr;

  mjvCamera camera;   // where the view is looking from
  mjvOption options;  // what gets drawn
  mjvScene scene;     // the list of shapes to draw this frame
  mjrContext context; // MuJoCo's OpenGL resources

  // -1 is the free camera, the mouse-driven one. 0 and up are the model's
  // fixed cameras, in the order the model lists them.
  int camera_index = -1;

  bool paused = false;
  bool step_frame = false;  // S was pressed while paused; done next frame
  bool left_down = false;
  bool right_down = false;
  double last_x = 0;
  double last_y = 0;
};

Viewer* ViewerOf(GLFWwindow* window) {
  return static_cast<Viewer*>(glfwGetWindowUserPointer(window));
}

// Takes one physics step and returns how much simulated time it covered.
// Never returns zero or less, so loops that step until enough time has passed
// always end, even if a step hook forgets to advance the clock.
double StepOnce(Viewer& viewer) {
  const double before = viewer.data->time;
  if (viewer.hooks->step) {
    viewer.hooks->step();
  } else {
    mj_step(viewer.model, viewer.data);
  }
  const double advanced = viewer.data->time - before;
  return advanced > 0 ? advanced : viewer.model->opt.timestep;
}

void Reset(Viewer& viewer) {
  if (viewer.hooks->reset) {
    viewer.hooks->reset();
  } else if (viewer.model->nkey > 0) {
    mj_resetDataKeyframe(viewer.model, viewer.data, 0);
  } else {
    mj_resetData(viewer.model, viewer.data);
  }
  // Recompute where the bodies are from the reset joint angles, so the reset
  // shows at once even while paused. This changes no state.
  mj_forward(viewer.model, viewer.data);
}

// Points the view at camera_index: the free camera, or one of the model's.
void ApplyCamera(Viewer& viewer) {
  if (viewer.camera_index < 0) {
    // The free camera keeps the lookat, distance and angles it had when it
    // was left, so switching back returns to the same view.
    viewer.camera.type = mjCAMERA_FREE;
  } else {
    viewer.camera.type = mjCAMERA_FIXED;
    viewer.camera.fixedcamid = viewer.camera_index;
  }
}

std::string CameraName(const Viewer& viewer) {
  if (viewer.camera_index < 0) return "free";
  const char* name =
      mj_id2name(viewer.model, mjOBJ_CAMERA, viewer.camera_index);
  if (name != nullptr && name[0] != '\0') return name;
  return "camera " + std::to_string(viewer.camera_index);
}

void OnKey(GLFWwindow* window, int key, int /*scancode*/, int action,
           int /*mods*/) {
  if (action == GLFW_RELEASE) return;
  Viewer* viewer = ViewerOf(window);

  // Holding S keeps stepping, at the keyboard's repeat rate.
  if (key == GLFW_KEY_S) {
    if (viewer->paused) viewer->step_frame = true;
    return;
  }

  // The other keys act once per press, not again while held.
  if (action != GLFW_PRESS) return;
  const int ncam = viewer->model->ncam;
  if (key == GLFW_KEY_BACKSPACE) {
    Reset(*viewer);
  } else if (key == GLFW_KEY_SPACE) {
    viewer->paused = !viewer->paused;
  } else if (key == GLFW_KEY_RIGHT_BRACKET) {
    // Free, then each fixed camera, then free again.
    viewer->camera_index =
        viewer->camera_index + 1 < ncam ? viewer->camera_index + 1 : -1;
    ApplyCamera(*viewer);
  } else if (key == GLFW_KEY_LEFT_BRACKET) {
    viewer->camera_index =
        viewer->camera_index >= 0 ? viewer->camera_index - 1 : ncam - 1;
    ApplyCamera(*viewer);
  } else if (key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

void OnMouseButton(GLFWwindow* window, int /*button*/, int /*action*/,
                   int /*mods*/) {
  Viewer* viewer = ViewerOf(window);
  viewer->left_down =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  viewer->right_down =
      glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  glfwGetCursorPos(window, &viewer->last_x, &viewer->last_y);
}

// mjv_moveCamera ignores fixed cameras, so the mouse only moves the free one.
void OnMouseMove(GLFWwindow* window, double x, double y) {
  Viewer* viewer = ViewerOf(window);
  if (!viewer->left_down && !viewer->right_down) return;

  const double dx = x - viewer->last_x;
  const double dy = y - viewer->last_y;
  viewer->last_x = x;
  viewer->last_y = y;

  int width = 0;
  int height = 0;
  glfwGetWindowSize(window, &width, &height);
  if (height == 0) return;

  const mjtMouse action =
      viewer->left_down ? mjMOUSE_ROTATE_V : mjMOUSE_MOVE_V;
  mjv_moveCamera(viewer->model, action, dx / height, dy / height,
                 &viewer->camera);
}

void OnScroll(GLFWwindow* window, double /*dx*/, double dy) {
  Viewer* viewer = ViewerOf(window);
  mjv_moveCamera(viewer->model, mjMOUSE_ZOOM, 0, -0.05 * dy, &viewer->camera);
}

// The text for the top-left corner: the caller's lines, then one line of the
// viewer's own with the simulated time, how fast it is running, and the
// camera.
std::string StatusText(const Viewer& viewer, double speed) {
  std::string text;
  if (viewer.hooks->status) {
    text = viewer.hooks->status();
    if (!text.empty() && text.back() != '\n') text += '\n';
  }
  if (!viewer.options_in->viewer_line) {
    if (!text.empty()) text.pop_back();
    return text;
  }
  char line[200];
  if (viewer.paused) {
    std::snprintf(line, sizeof(line), "t %.2f s   paused   camera %s",
                  viewer.data->time, CameraName(viewer).c_str());
  } else {
    std::snprintf(line, sizeof(line), "t %.2f s   %.2fx real time   camera %s",
                  viewer.data->time, speed, CameraName(viewer).c_str());
  }
  return text + line;
}

bool ParseNumber(const char* text, double* out) {
  char* end = nullptr;
  *out = std::strtod(text, &end);
  return end != text && *end == '\0';
}

}  // namespace

bool ParseViewerArgs(int argc, char** argv, int first, ViewerOptions* options,
                     ViewerRun* run, std::string* error) {
  for (int i = first; i < argc; ++i) {
    const std::string name = argv[i];
    if (name == "--no-overlay") {
      options->viewer_line = false;
      continue;
    }
    const bool known =
        name == "--lookat" || name == "--distance" || name == "--azimuth" ||
        name == "--elevation" || name == "--speed" || name == "--overlay" ||
        name == "--overlay-at" || name == "--caption" ||
        name == "--font-scale" || name == "--size" || name == "--record" ||
        name == "--fps" || name == "--quit-at" || name == "--key" ||
        name == "--groups";
    if (!known) {
      *error = "unknown option " + name;
      return false;
    }
    if (i + 1 >= argc) {
      *error = name + " needs a value";
      return false;
    }
    const char* arg = argv[++i];
    const std::string bad = "bad value for " + name + ": " + arg;
    double value = 0.0;
    if (name == "--lookat") {
      std::array<double, 3> at;
      if (std::sscanf(arg, "%lf,%lf,%lf", &at[0], &at[1], &at[2]) != 3) {
        *error = bad;
        return false;
      }
      options->lookat = at;
    } else if (name == "--distance" || name == "--azimuth" ||
               name == "--elevation") {
      if (!ParseNumber(arg, &value)) {
        *error = bad;
        return false;
      }
      if (name == "--distance") options->distance = value;
      if (name == "--azimuth") options->azimuth = value;
      if (name == "--elevation") options->elevation = value;
    } else if (name == "--speed" || name == "--fps") {
      if (!ParseNumber(arg, &value) || !(value > 0.0)) {
        *error = bad;
        return false;
      }
      (name == "--speed" ? options->speed : options->record_fps) = value;
    } else if (name == "--overlay") {
      if (std::strcmp(arg, "full") == 0) {
        options->viewer_line = true;
      } else if (std::strcmp(arg, "none") == 0) {
        options->viewer_line = false;
      } else {
        *error = bad;
        return false;
      }
    } else if (name == "--overlay-at") {
      if (std::strcmp(arg, "top") == 0) {
        options->text_at_bottom = false;
      } else if (std::strcmp(arg, "bottom") == 0) {
        options->text_at_bottom = true;
      } else {
        *error = bad;
        return false;
      }
    } else if (name == "--caption") {
      options->caption = arg;
    } else if (name == "--font-scale") {
      options->font_scale = std::atoi(arg);
      if (options->font_scale < 50 || options->font_scale > 300 ||
          options->font_scale % 50 != 0) {
        *error = bad;
        return false;
      }
    } else if (name == "--size") {
      if (std::sscanf(arg, "%dx%d", &options->width, &options->height) != 2 ||
          options->width < 64 || options->height < 64) {
        *error = bad;
        return false;
      }
    } else if (name == "--record") {
      options->record_path = arg;
    } else if (name == "--quit-at") {
      if (!ParseNumber(arg, &value) || !(value >= 0.0)) {
        *error = bad;
        return false;
      }
      run->quit_at = value;
    } else if (name == "--key") {
      run->key = arg;
    } else if (name == "--groups") {
      std::vector<int> groups;
      const char* p = arg;
      while (true) {
        char* end = nullptr;
        const long g = std::strtol(p, &end, 10);
        if (end == p || g < 0 || g >= mjNGROUP ||
            (*end != ',' && *end != '\0')) {
          *error = bad;
          return false;
        }
        groups.push_back(static_cast<int>(g));
        if (*end == '\0') break;
        p = end + 1;
      }
      options->extra_groups.insert(options->extra_groups.end(), groups.begin(),
                                   groups.end());
    }
  }
  return true;
}

const char* ViewerArgsUsage() {
  return "    [--key NAME_OR_INDEX] [--groups LIST] [--lookat X,Y,Z] "
         "[--distance D] [--azimuth DEG] [--elevation DEG] [--speed K]\n"
         "    [--overlay full|none | --no-overlay] [--overlay-at top|bottom] "
         "[--caption TEXT] [--font-scale P] [--size WxH]\n"
         "    [--record PATH] [--fps F] [--quit-at T]\n";
}

bool ResolveKeyframe(const mjModel* model, const std::string& key, int* index,
                     std::string* error) {
  *index = -1;
  if (key.empty()) {
    *index = model->nkey > 0 ? 0 : -1;
    return true;
  }
  const int named = mj_name2id(model, mjOBJ_KEY, key.c_str());
  if (named >= 0) {
    *index = named;
    return true;
  }
  char* end = nullptr;
  const long number = std::strtol(key.c_str(), &end, 10);
  if (end != key.c_str() && *end == '\0' && number >= 0 &&
      number < model->nkey) {
    *index = static_cast<int>(number);
    return true;
  }
  *error = "no keyframe " + key + " in the model (it has " +
           std::to_string(model->nkey) + ")";
  return false;
}

int RunViewer(const mjModel* model, mjData* data, const ViewerHooks& hooks,
              const char* title) {
  return RunViewer(model, data, hooks, title, ViewerOptions{});
}

int RunViewer(const mjModel* model, mjData* data, const ViewerHooks& hooks,
              const char* title, const ViewerOptions& options) {
  std::FILE* record = nullptr;
  if (!options.record_path.empty()) {
    record = std::fopen(options.record_path.c_str(), "wb");
    if (record == nullptr) {
      std::fprintf(stderr, "could not open %s\n", options.record_path.c_str());
      return 1;
    }
  }
  if (!glfwInit()) {
    std::fprintf(stderr, "could not start GLFW; is there a display?\n");
    return 1;
  }
  GLFWwindow* window =
      glfwCreateWindow(options.width, options.height, title, nullptr, nullptr);
  if (window == nullptr) {
    std::fprintf(stderr, "could not open a window\n");
    glfwTerminate();
    if (record != nullptr) std::fclose(record);
    return 1;
  }
  glfwMakeContextCurrent(window);
  // Wait for the screen's refresh before each frame; a recording does not
  // follow the wall clock, so it draws as fast as it can.
  glfwSwapInterval(record != nullptr ? 0 : 1);

  Viewer viewer;
  viewer.model = model;
  viewer.data = data;
  viewer.hooks = &hooks;
  viewer.options_in = &options;
  // Start looking at the centre of the scene from far enough back to see all
  // of it. The model says where that is and how big the scene is.
  mjv_defaultFreeCamera(model, &viewer.camera);
  if (options.lookat) {
    for (int i = 0; i < 3; ++i) viewer.camera.lookat[i] = (*options.lookat)[i];
  }
  if (options.distance) viewer.camera.distance = *options.distance;
  if (options.azimuth) viewer.camera.azimuth = *options.azimuth;
  if (options.elevation) viewer.camera.elevation = *options.elevation;
  mjv_defaultOption(&viewer.options);
  for (int g : options.extra_groups) {
    if (g >= 0 && g < mjNGROUP) {
      viewer.options.geomgroup[g] = 1;
      viewer.options.sitegroup[g] = 1;
    }
  }
  mjv_defaultScene(&viewer.scene);
  mjr_defaultContext(&viewer.context);
  mjv_makeScene(model, &viewer.scene, /*maxgeom=*/2000);
  mjr_makeContext(model, &viewer.context, options.font_scale);

  glfwSetWindowUserPointer(window, &viewer);
  glfwSetKeyCallback(window, OnKey);
  glfwSetMouseButtonCallback(window, OnMouseButton);
  glfwSetCursorPosCallback(window, OnMouseMove);
  glfwSetScrollCallback(window, OnScroll);

  // The caller may hand over a state it has set but not run forward yet;
  // this makes the first frame show it right. It changes no state.
  mj_forward(model, data);

  double last_wall = glfwGetTime();
  // Simulated time the simulation is behind the wall clock. Each frame adds
  // the wall time that passed and steps until it is paid off.
  double backlog = 0;
  // Simulated and wall time since the real-time factor was last measured.
  double speed = 1;
  double speed_sim = 0;
  double speed_wall = 0;
  // A recording's frame size, fixed at its first frame, and one frame of it.
  int record_width = 0;
  int record_height = 0;
  std::vector<unsigned char> pixels;
  // Simulated time a recording still owes: each frame adds its share and
  // steps until it is paid off, so the steps' rounding does not drift.
  double record_owed = 0;
  int status_code = 0;

  while (!glfwWindowShouldClose(window)) {
    const double now = glfwGetTime();
    const double wall_dt = now - last_wall;
    last_wall = now;

    if (record != nullptr) {
      // A recording ignores the wall clock and the pause key: every frame
      // is the same share of simulated time.
      record_owed += options.speed / options.record_fps;
      while (record_owed > 0) record_owed -= StepOnce(viewer);
      speed = options.speed;
    } else if (!viewer.paused) {
      // The physics timestep is much shorter than a frame, so this steps
      // many times per frame: enough to keep up with the wall clock, unless
      // stepping takes longer than the budget.
      backlog = std::min(backlog + options.speed * wall_dt,
                         options.speed * kMaxBacklog);
      const double give_up = now + kStepBudget;
      while (backlog > 0) {
        const double advanced = StepOnce(viewer);
        backlog -= advanced;
        speed_sim += advanced;
        if (glfwGetTime() > give_up) {
          // Too slow to keep up. Drop what is still owed instead of carrying
          // it, or every later frame would start further behind.
          backlog = 0;
          break;
        }
      }
      speed_wall += wall_dt;
      if (speed_wall >= kSpeedInterval) {
        speed = speed_sim / speed_wall;
        speed_sim = 0;
        speed_wall = 0;
      }
    } else {
      // Nothing is owed while paused, so resuming does not jump ahead, and
      // the paused time does not count against the real-time factor.
      backlog = 0;
      speed_sim = 0;
      speed_wall = 0;
      if (viewer.step_frame) {
        viewer.step_frame = false;
        double stepped = 0;
        while (stepped < kFrameTime) stepped += StepOnce(viewer);
      }
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(model, data, &viewer.options, nullptr, &viewer.camera,
                    mjCAT_ALL, &viewer.scene);
    mjr_render(viewport, &viewer.scene, &viewer.context);
    const std::string status = StatusText(viewer, speed);
    if (!status.empty()) {
      mjr_overlay(mjFONT_NORMAL,
                  options.text_at_bottom ? mjGRID_BOTTOMLEFT : mjGRID_TOPLEFT,
                  viewport, status.c_str(), nullptr, &viewer.context);
    }
    if (!options.caption.empty()) {
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport,
                  options.caption.c_str(), nullptr, &viewer.context);
    }

    if (record != nullptr) {
      if (record_width == 0) {
        record_width = viewport.width;
        record_height = viewport.height;
        pixels.resize(3 * static_cast<size_t>(record_width) * record_height);
        std::fprintf(stderr, "recording %dx%d at %g frames a second\n",
                     record_width, record_height, options.record_fps);
      }
      if (viewport.width != record_width || viewport.height != record_height) {
        std::fprintf(stderr, "the window changed size while recording\n");
        status_code = 1;
        break;
      }
      mjrRect frame = {0, 0, record_width, record_height};
      mjr_readPixels(pixels.data(), nullptr, frame, &viewer.context);
      // OpenGL's rows run bottom up; the file's run top down.
      const size_t row = 3 * static_cast<size_t>(record_width);
      bool written = true;
      for (int y = record_height - 1; y >= 0 && written; --y) {
        written = std::fwrite(pixels.data() + row * y, 1, row, record) == row;
      }
      if (!written) {
        std::fprintf(stderr, "could not write the recording\n");
        status_code = 1;
        break;
      }
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
    if (hooks.quit && hooks.quit()) glfwSetWindowShouldClose(window, GLFW_TRUE);
  }

  if (record != nullptr && std::fclose(record) != 0) status_code = 1;

  mjv_freeScene(&viewer.scene);
  mjr_freeContext(&viewer.context);
  glfwDestroyWindow(window);
  glfwTerminate();
  return status_code;
}

}  // namespace cell
