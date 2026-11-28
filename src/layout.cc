// Reads a layout from its JSON file. See src/layout.h.
//
// The reader is strict: every object must have exactly the fields of its
// struct, each of the right type. A fault throws a LayoutError carrying the
// path of the field in the file ("arms[1].track.beam.size[2]"); LoadLayout
// catches it and writes "<file>: <field>: <what is wrong>" into *error.
//
// Which fields the top level has depends on "base", read first: the
// container's layouts have "container", the flat rack's have "flat_rack",
// "margins", "ports" and "approaches", and a field of the other base is
// refused as unknown. Beyond the form, the reader checks what one field says
// of another: each work entry's arm has its tool, every point of every kind
// is given to exactly one work entry, a port hands over only things the
// layout has, and on the flat rack every kind that has points has one
// approach.

#include "src/layout.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include "src/arm_catalog.h"
#include "src/layout_racks.h"
#include "src/layout_read.h"

namespace cell::layout {
namespace {

using Json = nlohmann::json;

// The member of an object, by key; Fields() has made sure it is there.
#define FIELD(j, path, key) (j).at(key), Sub(path, key)

Layout ReadLayout(const Json& j) {
  const std::string path;
  if (!j.is_object()) {
    throw LayoutError("(top)", "expected an object, found " + TypeName(j));
  }
  if (!j.contains("base")) throw LayoutError("base", "missing field");
  Layout l;
  l.base = String(FIELD(j, path, "base"));
  const bool two_racks = l.base == "two_flat_racks";
  const bool flat_rack = l.base == "flat_rack" || two_racks;
  if (l.base == "container") {
    Fields(j, path,
           {"name", "title", "summary", "small_arms_stand", "base",
            "container", "tube", "holder", "arms", "pallets", "stocks",
            "strap_bolts_from", "tube_in", "bay_out", "work", "rail_order", "items"});
  } else if (flat_rack) {
    Fields(j, path,
           {"name", "title", "summary", "small_arms_stand", "base",
            two_racks ? "two_flat_racks" : "flat_rack", "margins", "ports", "approaches", "tube", "holder",
            "arms", "pallets", "stocks", "strap_bolts_from", "tube_in", "bay_out", "work", "rail_order", "items"},
           {"order", "staging", "retreat_legs", "path_tries", "plan_seed", "torque_limit_scale",
            "bay_out_when_clear", "first_bay_early", "next_bay_fill"});
  } else {
    throw LayoutError("base", "is \"container\", \"flat_rack\" or "
                              "\"two_flat_racks\", not \"" +
                                  l.base + "\"");
  }
  l.name = String(FIELD(j, path, "name"));
  l.title = String(FIELD(j, path, "title"));
  l.summary = String(FIELD(j, path, "summary"));
  l.small_arms_stand = String(FIELD(j, path, "small_arms_stand"));
  if (l.small_arms_stand != "below" && l.small_arms_stand != "beside" &&
      l.small_arms_stand != "above") {
    throw LayoutError("small_arms_stand",
                      "is \"below\", \"beside\" or \"above\", not \"" +
                          l.small_arms_stand + "\"");
  }
  if (flat_rack) {
    if (!two_racks) l.flat_rack = ReadFlatRack(FIELD(j, path, "flat_rack"));
    l.margins = ReadMargins(FIELD(j, path, "margins"));
  } else {
    l.container = ReadContainer(FIELD(j, path, "container"));
  }
  l.tube = ReadTube(FIELD(j, path, "tube"));
  l.holder = ReadHolder(FIELD(j, path, "holder"));
  const Json& arms = j.at("arms");
  Array(arms, "arms");
  for (size_t i = 0; i < arms.size(); ++i) {
    l.arms.push_back(ReadArm(arms[i], At("arms", i)));
  }
  Unique(l.arms, "arms", "arm");
  for (size_t i = 0; i < l.arms.size(); ++i) {
    const ArmUse& a = l.arms[i];
    if (!a.track || a.track->shares_beam_with.empty()) continue;
    const std::string where = Sub(Sub(At("arms", i), "track"), "shares_beam_with");
    const ArmUse* host = FindArm(l, a.track->shares_beam_with);
    if (host == nullptr || host == &a || !host->track) {
      throw LayoutError(where, "names no other arm with a track");
    }
    if (!host->track->shares_beam_with.empty()) {
      throw LayoutError(where, "names an arm that rides another's beam");
    }
    if (host->track->direction != a.track->direction) {
      throw LayoutError(where, "names a track of another direction");
    }
  }
  const Json& pallets = j.at("pallets");
  Array(pallets, "pallets");
  if (pallets.empty()) throw LayoutError("pallets", "no pallet place given");
  for (size_t i = 0; i < pallets.size(); ++i) {
    l.pallets.push_back(ReadPallet(pallets[i], At("pallets", i)));
  }
  if (flat_rack) {
    if (!l.holder.travels_inside) {
      throw LayoutError("holder.travels_inside",
                        "on the flat rack the holder travels with the cell and "
                        "stands on the deck");
    }
    for (size_t i = 0; i < l.pallets.size(); ++i) {
      if (!l.pallets[i].outside) {
        throw LayoutError(At("pallets", i) + ".outside",
                          "on the flat rack a pallet stands at a port, outside "
                          "the base");
      }
    }
  }
  const Json& stocks = j.at("stocks");
  Array(stocks, "stocks");
  for (size_t i = 0; i < stocks.size(); ++i) {
    l.stocks.push_back(ReadStock(stocks[i], At("stocks", i)));
  }
  Unique(l.stocks, "stocks", "stock");
  l.strap_bolts_from = String(FIELD(j, path, "strap_bolts_from"));
  l.tube_in = ReadWay(FIELD(j, path, "tube_in"));
  l.bay_out = ReadWay(FIELD(j, path, "bay_out"));
  const Json& work = j.at("work");
  Array(work, "work");
  // For each kind, which entry has each point (-1: none yet).
  std::map<std::string, std::vector<int>> owner;
  for (const std::string& kind : WorkKinds()) {
    owner[kind].assign(PointCount(l, kind), -1);
  }
  for (size_t i = 0; i < work.size(); ++i) {
    const std::string p = At("work", i);
    Work w = ReadWork(work[i], p);
    const ArmUse* arm = FindArm(l, w.arm);
    if (arm == nullptr) {
      throw LayoutError(p + ".arm", "no arm \"" + w.arm + "\" in the layout");
    }
    if (FindTool(*arm, w.tool) == nullptr) {
      throw LayoutError(p + ".tool", "the arm \"" + w.arm + "\" has no tool \"" +
                                         w.tool + "\"");
    }
    std::vector<int>& own = owner[w.kind];
    for (int n : ParsePoints(w.points, static_cast<int>(own.size()),
                             p + ".points")) {
      if (own[n] >= 0) {
        throw LayoutError(p + ".points",
                          "point " + w.kind + " " + std::to_string(n) +
                              " is given to two arms (" + l.work[own[n]].arm +
                              " and " + w.arm + ")");
      }
      own[n] = static_cast<int>(i);
    }
    l.work.push_back(w);
  }
  for (const std::string& kind : WorkKinds()) {
    const std::vector<int>& own = owner[kind];
    for (size_t n = 0; n < own.size(); ++n) {
      if (own[n] < 0) {
        throw LayoutError("work", "point " + kind + " " + std::to_string(n) +
                                      " is given to no arm");
      }
    }
  }
  l.rail_order = String(FIELD(j, path, "rail_order"));
  if (l.rail_order != "all_rails_then_all_bolts" &&
      l.rail_order != "each_rail_then_its_bolt") {
    throw LayoutError("rail_order",
                      "is \"all_rails_then_all_bolts\" or "
                      "\"each_rail_then_its_bolt\", not \"" +
                          l.rail_order + "\"");
  }
  const Json& items = j.at("items");
  Array(items, "items");
  for (size_t i = 0; i < items.size(); ++i) {
    l.items.push_back(ReadItem(items[i], At("items", i)));
  }
  Unique(l.items, "items", "item");
  if (!flat_rack) return l;

  if (j.contains("retreat_legs")) {
    const Json& r = j.at("retreat_legs");
    const std::string rp = "retreat_legs";
    Fields(r, rp, {"speed", "accel", "basis"});
    RetreatLegs legs;
    legs.speed = Number(FIELD(r, rp, "speed"));
    legs.accel = Number(FIELD(r, rp, "accel"));
    legs.basis = String(FIELD(r, rp, "basis"));
    if (!(legs.speed > 0.0 && legs.speed <= kMaxRetreatSpeed)) {
      throw LayoutError(Sub(rp, "speed"), "is not in (0, 0.5] m/s");
    }
    if (!(legs.accel > 0.0 && legs.accel <= kMaxRetreatAccel)) {
      throw LayoutError(Sub(rp, "accel"), "is not in (0, 3.0] m/s^2");
    }
    l.retreat_legs = legs;
  }
  for (const std::string k : {"path_tries", "plan_seed"}) {  // src/layout.h
    if (!j.contains(k)) continue;
    const int v = Integer(j.at(k), k);
    const int most = k == "path_tries" ? kMaxPathTries : kMaxPlanSeed;
    if (v < 2 || v > most) throw LayoutError(k, "is not an integer from 2 to " + std::to_string(most));
    (k == "path_tries" ? l.path_tries : l.plan_seed) = v;
  }
  if (j.contains("torque_limit_scale")) {
    const double scale = Number(j.at("torque_limit_scale"), "torque_limit_scale");
    if (!(scale >= kMinTorqueLimitScale && scale <= 1.0)) {
      throw LayoutError("torque_limit_scale", "is not a fraction in [0.5, 1]");
    }
    l.torque_limit_scale = scale;
  }
  if (j.contains("bay_out_when_clear")) {
    l.bay_out_when_clear = Bool(j.at("bay_out_when_clear"), "bay_out_when_clear");
  }
  if (j.contains("first_bay_early")) {  // src/layout_bays.h, "first_bay_early".
    const Json& e = j.at("first_bay_early");
    Fields(e, "first_bay_early", {"does", "basis"});
    (void)String(FIELD(e, "first_bay_early", "does"));
    l.first_bay_early = String(FIELD(e, "first_bay_early", "basis"));
  }
  if (j.contains("next_bay_fill")) {  // src/layout_bays.h, "next_bay_fill".
    Fields(j.at("next_bay_fill"), "next_bay_fill", {"arm", "does", "basis"});
    l.next_bay_fill = String(FIELD(j.at("next_bay_fill"), "next_bay_fill", "arm"));
  }
  if (j.contains("order")) {
    const Json& o = j.at("order");
    const std::string op = "order";
    Fields(o, op, {"modules", "modules_start", "pin_tools", "note", "basis"});
    const Json& mods = o.at("modules");
    Array(mods, Sub(op, "modules"));
    const int n = PointCount(l, "module_set");
    std::vector<bool> seen(n, false);
    for (size_t i = 0; i < mods.size(); ++i) {
      const int m = Integer(mods[i], At(Sub(op, "modules"), i));
      if (m < 0 || m >= n || seen[m]) {
        throw LayoutError(At(Sub(op, "modules"), i),
                          "is not one of the modules 0.." +
                              std::to_string(n - 1) + " not named before");
      }
      seen[m] = true;
      l.order.modules.push_back(m);
    }
    if (static_cast<int>(l.order.modules.size()) != n) {
      throw LayoutError(Sub(op, "modules"),
                        "names " + std::to_string(l.order.modules.size()) +
                            " modules, not all " + std::to_string(n));
    }
    l.order.modules_start = String(FIELD(o, op, "modules_start"));
    if (l.order.modules_start != "after_all_bolts" &&
        l.order.modules_start != "after_their_rails") {
      throw LayoutError(Sub(op, "modules_start"),
                        "is \"after_all_bolts\" or \"after_their_rails\", not \"" +
                            l.order.modules_start + "\"");
    }
    l.order.pin_tools = String(FIELD(o, op, "pin_tools"));
    if (l.order.pin_tools != "just_in_time" && l.order.pin_tools != "early") {
      throw LayoutError(Sub(op, "pin_tools"),
                        "is \"just_in_time\" or \"early\", not \"" +
                            l.order.pin_tools + "\"");
    }
    l.order.note = String(FIELD(o, op, "note"));
    l.order.basis = String(FIELD(o, op, "basis"));
  }

  const Json& ports = j.at("ports");
  Array(ports, "ports");
  for (size_t i = 0; i < ports.size(); ++i) {
    const std::string p = At("ports", i);
    Port port = ReadPort(ports[i], p);
    for (size_t k = 0; k < port.hands_over.size(); ++k) {
      CheckHandsOver(l, port.hands_over[k], At(p + ".hands_over", k));
    }
    l.ports.push_back(port);
  }
  Unique(l.ports, "ports", "port");
  for (size_t i = 0; i < l.ports.size(); ++i) {
    const std::string& other = l.ports[i].shares_zone_with;
    if (other.empty()) continue;
    if (other == l.ports[i].name || FindPort(l, other) == nullptr) {
      throw LayoutError(At("ports", i) + ".shares_zone_with",
                        "\"" + other + "\" is not the name of another port");
    }
  }
  const Json& approaches = j.at("approaches");
  Array(approaches, "approaches");
  for (size_t i = 0; i < approaches.size(); ++i) {
    const std::string p = At("approaches", i);
    Approach a = ReadApproach(approaches[i], p);
    if (FindApproach(l, a.kind) != nullptr) {
      throw LayoutError(p + ".kind", "the kind \"" + a.kind +
                                         "\" has an approach already");
    }
    l.approaches.push_back(a);
  }
  for (const std::string& kind : WorkKinds()) {
    if (PointCount(l, kind) > 0 && FindApproach(l, kind) == nullptr) {
      throw LayoutError("approaches", "the kind \"" + kind +
                                          "\" has points and no approach");
    }
  }
  ReadRacksAndStages(j, &l);
  return l;
}

#undef FIELD

}  // namespace

std::optional<Layout> LoadLayout(const std::string& path, std::string* error) {
  auto fail = [&](const std::string& message) -> std::optional<Layout> {
    if (error != nullptr) *error = path + ": " + message;
    return std::nullopt;
  };
  std::ifstream in(path);
  if (!in) return fail("cannot open the file");
  std::stringstream text;
  text << in.rdbuf();
  Json j;
  try {
    j = Json::parse(text.str());
  } catch (const Json::parse_error& e) {
    return fail(std::string("not JSON: ") + e.what());
  }
  Layout layout;
  try {
    layout = ReadLayout(j);
  } catch (const std::runtime_error& e) {  // LayoutError, and src/layout_racks.cc's.
    return fail(e.what());
  } catch (const Json::exception& e) {
    return fail(e.what());
  }
  const std::string stem = std::filesystem::path(path).stem().string();
  if (layout.name != stem) {
    return fail("name: \"" + layout.name + "\" is not the file's name \"" +
                stem + "\"");
  }
  return layout;
}

std::vector<std::string> ListLayouts(const std::string& directory) {
  std::vector<std::string> names;
  std::error_code ec;
  for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
    if (entry.is_regular_file() && entry.path().extension() == ".json") {
      names.push_back(entry.path().stem().string());
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

std::vector<int> PointsOf(const Layout& layout, const Work& work) {
  try {
    return ParsePoints(work.points, PointCount(layout, work.kind), "points");
  } catch (const LayoutError&) {
    return {};
  }
}

std::vector<int> ParsePointList(const std::string& text, int count,
                                const std::string& path) {
  return ParsePoints(text, count, path);
}

}  // namespace cell::layout
