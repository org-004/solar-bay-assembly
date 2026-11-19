// Owning handles for MuJoCo's two main objects.
//
// MuJoCo is a C library: mj_loadXML and mj_makeData return raw pointers, and
// the caller must hand them back to mj_deleteModel and mj_deleteData. Wrapping
// each pointer in a std::unique_ptr makes that automatic. When the handle goes
// out of scope, on any path out of a function, the matching delete runs.

#ifndef SRC_MJ_HANDLES_H_
#define SRC_MJ_HANDLES_H_

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

namespace cell {

// What unique_ptr calls in place of `delete`.
struct ModelDeleter {
  void operator()(mjModel* model) const { mj_deleteModel(model); }
};
struct DataDeleter {
  void operator()(mjData* data) const { mj_deleteData(data); }
};

// mjModel: the description of the robot. Constant while simulating.
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
// mjData: the state (time, joint positions, velocities). Changes every step.
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

// Loads a model from an MJCF or URDF file. On failure returns an empty handle
// and, if `error` is not null, writes MuJoCo's message into it.
inline ModelPtr LoadModel(const std::string& path, std::string* error = nullptr) {
  char message[1000] = "";
  ModelPtr model(mj_loadXML(path.c_str(), nullptr, message, sizeof(message)));
  if (!model && error != nullptr) {
    *error = message;
  }
  return model;
}

// Allocates the state for a model, set to the model's initial configuration.
inline DataPtr MakeData(const mjModel& model) {
  return DataPtr(mj_makeData(&model));
}

}  // namespace cell

#endif  // SRC_MJ_HANDLES_H_
