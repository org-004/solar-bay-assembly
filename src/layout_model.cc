// Builds a layout's MuJoCo model. See src/layout_model.h.

#include "src/layout_model.h"

namespace cell::layout {

ModelPaths ModelPaths::FromEnvironment() { return {}; }

std::string BuildLayoutXml(const Layout&, const LayoutState&,
                           const ModelPaths&, std::string* error) {
  if (error != nullptr) *error = "not written yet";
  return {};
}

ModelPtr LoadLayoutModel(const Layout&, const LayoutState&, const ModelPaths&,
                         std::string* error) {
  if (error != nullptr) *error = "not written yet";
  return nullptr;
}

LayoutHandles FindHandles(const mjModel&, const Layout&) { return {}; }

Eigen::Vector3d BayPoint(const Layout& layout, const Eigen::Vector3d& in_bay) {
  return Eigen::Vector3d(layout.tube.axis_x, layout.tube.plain_end_y,
                         layout.tube.axis_z) +
         in_bay;
}

}  // namespace cell::layout
