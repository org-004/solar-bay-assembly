// The lookups of src/layout.h that read no layout file: the order of the
// bay's modules a layout gives (WorkOrder), whether it stands on flat racks
// and the stages of its work (Staging), and which layout is chosen, read
// from the registry of versions.

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "src/layout.h"

namespace cell::layout {

using Json = nlohmann::json;

std::vector<int> ModuleOrder(const Layout& layout) {
  if (!layout.order.modules.empty()) return layout.order.modules;
  std::vector<int> order(PointCount(layout, "module_set"));
  for (size_t k = 0; k < order.size(); ++k) order[k] = static_cast<int>(k);
  return order;
}

std::vector<int> ModulePositions(const Layout& layout) {
  const std::vector<int> order = ModuleOrder(layout);
  std::vector<int> pos(order.size());
  for (size_t k = 0; k < order.size(); ++k) pos[order[k]] = static_cast<int>(k);
  return pos;
}

std::string VersionsPath() {
  const char* path = std::getenv("VERSIONS_JSON");
  return path != nullptr && path[0] != '\0' ? path : "versions/versions.json";
}

std::optional<std::string> ChosenLayout(const std::string& versions_path,
                                        std::string* error) {
  auto fail = [&](const std::string& message) -> std::optional<std::string> {
    if (error != nullptr) *error = versions_path + ": " + message;
    return std::nullopt;
  };
  std::ifstream in(versions_path);
  if (!in) return fail("cannot open the file");
  std::stringstream text;
  text << in.rdbuf();
  try {
    const Json j = Json::parse(text.str());
    std::optional<std::string> chosen;
    for (const Json& v : j.at("versions")) {
      if (!v.contains("status") || v.at("status") != "chosen") continue;
      if (!v.contains("layout")) continue;
      const std::string path = v.at("layout").at("path").get<std::string>();
      chosen = std::filesystem::path(path).stem().string();
    }
    if (!chosen) return fail("no version with status \"chosen\" records a layout");
    return chosen;
  } catch (const Json::exception& e) {
    return fail(e.what());
  }
}

bool OnFlatRacks(const Layout& layout) {
  return layout.base == "flat_rack" || layout.base == "two_flat_racks";
}

std::vector<Stage> StagesOf(const Layout& layout) {
  if (layout.staging && !layout.staging->stages.empty()) {
    return layout.staging->stages;
  }
  Stage one;
  one.name = "1";
  one.tube_offset_m = 0.0;
  one.rails = "all";
  one.modules = "all";
  one.grips = layout.holder.grips;
  one.angle_pin_hold = layout.holder.angle_pin_hold;
  one.held_by = "the holder alone";
  return {one};
}

}  // namespace cell::layout
