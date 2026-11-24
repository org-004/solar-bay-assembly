#include "src/ompl_search.h"

// A placeholder so that the targets round it build; replaced by the search.
namespace cell {

const std::vector<std::string>& OmplPlannerNames() {
  static const std::vector<std::string> names = {
      "RRTConnect", "BiTRRT", "BITstar", "AORRTC", "InformedRRTstar"};
  return names;
}

bool IsOmplPlanner(const std::string& name) {
  for (const std::string& n : OmplPlannerNames()) {
    if (n == name) return true;
  }
  return false;
}

bool OmplPlannerRepeats(const std::string& name) {
  return name == "RRTConnect" || name == "BiTRRT";
}

const char* ToString(OmplStop) { return "kNotStarted"; }

std::optional<JointPath> PlanOmpl(const Eigen::VectorXd&,
                                  const Eigen::VectorXd&,
                                  const Eigen::VectorXd&,
                                  const Eigen::VectorXd&, const IsFree&,
                                  const RrtOptions&, const OmplSettings&,
                                  RrtReport* report, OmplReport*) {
  if (report != nullptr) *report = RrtReport();
  return std::nullopt;
}

}  // namespace cell
