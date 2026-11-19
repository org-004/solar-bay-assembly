// Draws a robot model in a window and simulates it in real time.
//
//   bazel run //:viewer
//
// Left drag rotates the camera, right drag moves it, scroll zooms.
// Backspace resets the simulation. Space pauses. Escape quits.
//
// GLFW opens the window and delivers keyboard and mouse events. MuJoCo draws
// into the window's OpenGL context. Needs a display.

#include <cstdio>
#include <string>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "src/mj_handles.h"

namespace {

// Everything the event callbacks need. GLFW callbacks are plain C functions
// with fixed arguments, so the window carries a pointer to this struct and
// each callback fetches it back.
struct Viewer {
  const mjModel* model = nullptr;
  mjData* data = nullptr;

  mjvCamera camera;   // where the view is looking from
  mjvOption options;  // what gets drawn
  mjvScene scene;     // the list of shapes to draw this frame
  mjrContext context; // MuJoCo's OpenGL resources

  bool paused = false;
  bool left_down = false;
  bool right_down = false;
  double last_x = 0;
  double last_y = 0;
};

Viewer* ViewerOf(GLFWwindow* window) {
  return static_cast<Viewer*>(glfwGetWindowUserPointer(window));
}

void OnKey(GLFWwindow* window, int key, int /*scancode*/, int action,
           int /*mods*/) {
  if (action != GLFW_PRESS) return;
  Viewer* viewer = ViewerOf(window);
  if (key == GLFW_KEY_BACKSPACE) {
    mj_resetData(viewer->model, viewer->data);
    mj_forward(viewer->model, viewer->data);
  } else if (key == GLFW_KEY_SPACE) {
    viewer->paused = !viewer->paused;
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

}  // namespace

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

  if (!glfwInit()) {
    std::fprintf(stderr, "could not start GLFW; is there a display?\n");
    return 1;
  }
  GLFWwindow* window =
      glfwCreateWindow(1200, 900, "solar-panel-pick-place", nullptr, nullptr);
  if (window == nullptr) {
    std::fprintf(stderr, "could not open a window\n");
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);  // wait for the screen's refresh before each frame

  Viewer viewer;
  viewer.model = model.get();
  viewer.data = data.get();
  mjv_defaultCamera(&viewer.camera);
  mjv_defaultOption(&viewer.options);
  mjv_defaultScene(&viewer.scene);
  mjr_defaultContext(&viewer.context);
  mjv_makeScene(model.get(), &viewer.scene, /*maxgeom=*/2000);
  mjr_makeContext(model.get(), &viewer.context, mjFONTSCALE_150);

  glfwSetWindowUserPointer(window, &viewer);
  glfwSetKeyCallback(window, OnKey);
  glfwSetMouseButtonCallback(window, OnMouseButton);
  glfwSetCursorPosCallback(window, OnMouseMove);
  glfwSetScrollCallback(window, OnScroll);

  const double frame_time = 1.0 / 60.0;  // seconds of simulation per frame
  while (!glfwWindowShouldClose(window)) {
    // The physics timestep is much shorter than a frame, so step many times
    // per frame: enough to cover one frame's worth of simulated time.
    if (!viewer.paused) {
      const double frame_end = data->time + frame_time;
      while (data->time < frame_end) {
        mj_step(model.get(), data.get());
      }
    }

    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(model.get(), data.get(), &viewer.options, nullptr,
                    &viewer.camera, mjCAT_ALL, &viewer.scene);
    mjr_render(viewport, &viewer.scene, &viewer.context);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&viewer.scene);
  mjr_freeContext(&viewer.context);
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
