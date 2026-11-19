// Loads a robot model, simulates one second with no motor commands, and
// prints where each joint ended up.
//
//   bazel run //:step_arm
//
// The model's path is the first command-line argument. BUILD.bazel supplies
// it when the program is started through `bazel run`.

#include <cstdio>
#include <string>

#include <mujoco/mujoco.h>

#include "src/mj_handles.h"

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

  // One call to mj_step advances the simulation by the model's timestep.
  const double duration = 1.0;  // seconds
  int steps = 0;
  while (data->time < duration) {
    mj_step(model.get(), data.get());
    ++steps;
  }

  std::printf("%d steps of %g s, simulated time %.3f s\n", steps,
              model->opt.timestep, data->time);

  // qpos holds one number per degree of freedom for this arm: the joint
  // angles, in radians.
  for (int joint = 0; joint < model->njnt; ++joint) {
    const char* name = mj_id2name(model.get(), mjOBJ_JOINT, joint);
    const double angle = data->qpos[model->jnt_qposadr[joint]];
    std::printf("  %-22s %9.5f rad\n", name != nullptr ? name : "(unnamed)",
                angle);
  }
  return 0;
}
