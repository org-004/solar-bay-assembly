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

const ArmUse* FindArm(const Layout& layout, const std::string& name) {
  for (const ArmUse& a : layout.arms) {
    if (a.name == name) return &a;
  }
  return nullptr;
}

int BeamArm(const Layout& layout, int arm) {
  if (arm < 0 || arm >= static_cast<int>(layout.arms.size())) return arm;
  const auto& track = layout.arms[arm].track;
  if (!track || track->shares_beam_with.empty()) return arm;
  for (size_t i = 0; i < layout.arms.size(); ++i) {
    if (layout.arms[i].name == track->shares_beam_with) {
      return static_cast<int>(i);
    }
  }
  return arm;
}

const Tool* FindTool(const ArmUse& arm, const std::string& name) {
  for (const Tool& t : arm.tools) {
    if (t.name == name) return &t;
  }
  return nullptr;
}

const Work* FindWork(const Layout& layout, const std::string& kind) {
  for (const Work& w : layout.work) {
    if (w.kind == kind) return &w;
  }
  return nullptr;
}

const Stock* FindStock(const Layout& layout, const std::string& name) {
  for (const Stock& s : layout.stocks) {
    if (s.name == name) return &s;
  }
  return nullptr;
}

const std::vector<std::string>& WorkKinds() {
  static const std::vector<std::string> kinds = {
      "rail_take",  "rail_set", "strap_bolt", "module_take", "module_set",
      "locate",     "pin",      "swage",      "pins_take"};
  return kinds;
}

const Work* FindWork(const Layout& layout, const std::string& kind, int index) {
  for (const Work& w : layout.work) {
    if (w.kind != kind) continue;
    const std::vector<int> points = PointsOf(layout, w);
    if (std::binary_search(points.begin(), points.end(), index)) return &w;
  }
  return nullptr;
}

// LoadLayout has refused a points string that does not parse; for a layout
// made in memory, one that does not gives no points.

int PointCount(const Layout& layout, const std::string& kind) {
  if (kind == "rail_take" || kind == "rail_set" || kind == "strap_bolt") {
    return 10;
  }
  if (kind == "module_set") return 9;
  if (kind == "locate" || kind == "pin" || kind == "swage") return 36;
  if (kind == "module_take") {
    int n = 0;
    for (const Pallet& p : layout.pallets) n += p.modules;
    return n;
  }
  if (kind == "pins_take") return static_cast<int>(PinStocks(layout).size());
  return 0;
}

std::vector<const Stock*> PinStocks(const Layout& layout) {
  std::vector<const Stock*> out;
  for (const Stock& s : layout.stocks) {
    if (s.name == "pins" || s.name.rfind("pins_", 0) == 0) out.push_back(&s);
  }
  return out;
}

std::vector<std::string> ArmsDoing(const Layout& layout,
                                   const std::vector<std::string>& kinds) {
  std::vector<std::string> out;
  for (const ArmUse& arm : layout.arms) {
    bool does = false;
    for (const Work& w : layout.work) {
      if (w.arm != arm.name) continue;
      if (std::find(kinds.begin(), kinds.end(), w.kind) == kinds.end()) continue;
      if (!PointsOf(layout, w).empty()) does = true;
    }
    if (does) out.push_back(arm.name);
  }
  return out;
}

const Approach* FindApproach(const Layout& layout, const std::string& kind) {
  for (const Approach& a : layout.approaches) {
    if (a.kind == kind) return &a;
  }
  return nullptr;
}

const Port* FindPort(const Layout& layout, const std::string& name) {
  for (const Port& p : layout.ports) {
    if (p.name == name) return &p;
  }
  return nullptr;
}

}  // namespace cell::layout
