// Reads a layout from its JSON file. See src/layout.h.

#include "src/layout.h"

namespace cell::layout {

std::optional<Layout> LoadLayout(const std::string& path, std::string* error) {
  if (error != nullptr) *error = "not written yet: " + path;
  return std::nullopt;
}

std::vector<std::string> ListLayouts(const std::string&) { return {}; }

const ArmUse* FindArm(const Layout&, const std::string&) { return nullptr; }
const Tool* FindTool(const ArmUse&, const std::string&) { return nullptr; }
const Work* FindWork(const Layout&, const std::string&) { return nullptr; }
const Stock* FindStock(const Layout&, const std::string&) { return nullptr; }

const std::vector<std::string>& WorkKinds() {
  static const std::vector<std::string> kinds = {
      "rail_take",  "rail_set", "strap_bolt", "module_take", "module_set",
      "locate",     "pin",      "swage",      "pins_take"};
  return kinds;
}

}  // namespace cell::layout
