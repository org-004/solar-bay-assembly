// See src/layout_plan.h. STUB written with the header; the planner's owner
// replaces it.

#include "src/layout_plan.h"

namespace cell::layout {

MoveWorld::MoveWorld(const LayoutScene& scene, int arm, const std::string&,
                     const std::string&, const Built&, double clearance_m)
    : scene_(scene), arm_(arm), clearance_m_(clearance_m) {}

Nearest MoveWorld::Least(mjData*, const Config&, double) const { return {}; }

bool MoveWorld::Free(mjData*, const Config&) const { return false; }

PlanResult PlanLayout(const Layout&, const ModelPaths&, const std::string&,
                      const PlanOptions&) {
  PlanResult r;
  r.faults.push_back("PlanLayout is not written yet");
  return r;
}

}  // namespace cell::layout
