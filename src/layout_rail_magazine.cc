// A rail magazine on the tool that sets the rails (src/layout_rail_magazine.h).

#include "src/layout_rail_magazine.h"

#include <algorithm>
#include <cstdio>
#include <string>

#include "src/bay_hardware.h"
#include "src/layout.h"

namespace cell::layout {

int RailsPerTake(const Layout& layout) {
  const Work* w = FindWork(layout, "rail_set");
  if (w == nullptr) return 1;
  for (const ArmUse& a : layout.arms) {
    if (a.name != w->arm) continue;
    const Tool* t = FindTool(a, w->tool);
    if (t != nullptr && t->magazine_holds > 0) return t->magazine_holds;
  }
  return 1;
}

bool RailHasTake(const Layout& layout, int rail) {
  return rail % RailsPerTake(layout) == 0;
}

int RailsLoadedAt(const Layout& layout, int rail) {
  return std::max(1, std::min(RailsPerTake(layout), bay::kRailCount - rail));
}

double WorkSeconds(const Layout& layout, const Work& work, const std::string& kind,
                   int index) {
  if (kind == "rail_take") return work.process_s * RailsLoadedAt(layout, index);
  return work.process_s;
}

double MagazineRailsMass(const Layout& layout) {
  return bay::kRailMass * RailsPerTake(layout);
}

std::string MagazineRailsInertialXml(const Layout& layout, const std::string& part,
                                     double tool_length) {
  if (part != "rail" || RailsPerTake(layout) <= 1) return "";
  const double extra = MagazineRailsMass(layout) - bay::kRailMass;
  char text[256];
  std::snprintf(text, sizeof(text),
                "<inertial pos=\"0 0 %.12g\" mass=\"%.12g\" diaginertia=\"%.12g %.12g %.12g\"/>\n",
                tool_length / 2.0, extra, 0.1 * extra, 0.1 * extra, 0.1 * extra);
  return text;
}

}  // namespace cell::layout
