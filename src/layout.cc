// Reads a layout from its JSON file. See src/layout.h.
//
// The reader is strict: every object must have exactly the fields of its
// struct, each of the right type. A fault throws a LayoutError carrying the
// path of the field in the file ("arms[1].track.beam.size[2]"); LoadLayout
// catches it and writes "<file>: <field>: <what is wrong>" into *error.

#include "src/layout.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include "src/arm_catalog.h"

namespace cell::layout {
namespace {

using Json = nlohmann::json;

struct LayoutError : std::runtime_error {
  LayoutError(const std::string& field, const std::string& what)
      : std::runtime_error(field + ": " + what) {}
};

std::string TypeName(const Json& j) { return j.type_name(); }

// Checks that `j` is an object with exactly the fields `keys`, in any order.
void Fields(const Json& j, const std::string& path,
            std::initializer_list<const char*> keys,
            std::initializer_list<const char*> optional = {}) {
  if (!j.is_object()) {
    throw LayoutError(path.empty() ? "(top)" : path,
                      "expected an object, found " + TypeName(j));
  }
  std::set<std::string> known;
  for (const char* k : keys) known.insert(k);
  for (const char* k : optional) known.insert(k);
  for (const auto& [key, value] : j.items()) {
    if (known.count(key) == 0) {
      throw LayoutError(path.empty() ? key : path + "." + key,
                        "unknown field");
    }
  }
  for (const char* k : keys) {
    if (!j.contains(k)) {
      throw LayoutError(path.empty() ? std::string(k) : path + "." + k,
                        "missing field");
    }
  }
}

std::string Sub(const std::string& path, const char* key) {
  return path.empty() ? std::string(key) : path + "." + key;
}
std::string At(const std::string& path, size_t i) {
  return path + "[" + std::to_string(i) + "]";
}

double Number(const Json& j, const std::string& path) {
  if (!j.is_number()) {
    throw LayoutError(path, "expected a number, found " + TypeName(j));
  }
  return j.get<double>();
}

int Integer(const Json& j, const std::string& path) {
  if (!j.is_number_integer()) {
    throw LayoutError(path, "expected a whole number, found " + TypeName(j));
  }
  return j.get<int>();
}

bool Bool(const Json& j, const std::string& path) {
  if (!j.is_boolean()) {
    throw LayoutError(path, "expected true or false, found " + TypeName(j));
  }
  return j.get<bool>();
}

std::string String(const Json& j, const std::string& path) {
  if (!j.is_string()) {
    throw LayoutError(path, "expected a string, found " + TypeName(j));
  }
  return j.get<std::string>();
}

const Json& Array(const Json& j, const std::string& path) {
  if (!j.is_array()) {
    throw LayoutError(path, "expected an array, found " + TypeName(j));
  }
  return j;
}

template <size_t N>
std::array<double, N> Numbers(const Json& j, const std::string& path) {
  Array(j, path);
  if (j.size() != N) {
    throw LayoutError(path, "expected " + std::to_string(N) +
                                " numbers, found " + std::to_string(j.size()));
  }
  std::array<double, N> out{};
  for (size_t i = 0; i < N; ++i) out[i] = Number(j[i], At(path, i));
  return out;
}

std::vector<std::string> Strings(const Json& j, const std::string& path) {
  Array(j, path);
  std::vector<std::string> out;
  for (size_t i = 0; i < j.size(); ++i) out.push_back(String(j[i], At(path, i)));
  return out;
}

// The member of an object, by key; Fields() has made sure it is there.
#define FIELD(j, path, key) (j).at(key), Sub(path, key)

template <typename T>
void Unique(const std::vector<T>& items, const std::string& path,
            const char* what) {
  std::set<std::string> seen;
  for (size_t i = 0; i < items.size(); ++i) {
    if (!seen.insert(items[i].name).second) {
      throw LayoutError(At(path, i) + ".name",
                        std::string(what) + " \"" + items[i].name +
                            "\" is used twice");
    }
  }
}

Box ReadBox(const Json& j, const std::string& path) {
  Fields(j, path, {"name", "size", "pos"});
  Box box;
  box.name = String(FIELD(j, path, "name"));
  box.size = Numbers<3>(FIELD(j, path, "size"));
  box.pos = Numbers<3>(FIELD(j, path, "pos"));
  for (size_t i = 0; i < 3; ++i) {
    if (box.size[i] < 0.0) {
      throw LayoutError(At(Sub(path, "size"), i), "a size is not negative");
    }
  }
  return box;
}

std::vector<Box> ReadBoxes(const Json& j, const std::string& path) {
  Array(j, path);
  std::vector<Box> boxes;
  for (size_t i = 0; i < j.size(); ++i) boxes.push_back(ReadBox(j[i], At(path, i)));
  Unique(boxes, path, "box");
  return boxes;
}

ContainerState ReadContainer(const Json& j, const std::string& path) {
  Fields(j, path, {"sides_open", "rear_doors_open", "note"});
  ContainerState c;
  c.sides_open = Strings(FIELD(j, path, "sides_open"));
  for (size_t i = 0; i < c.sides_open.size(); ++i) {
    if (c.sides_open[i] != "-x" && c.sides_open[i] != "+x") {
      throw LayoutError(At(Sub(path, "sides_open"), i),
                        "a side is \"-x\" or \"+x\", not \"" +
                            c.sides_open[i] + "\"");
    }
  }
  c.rear_doors_open = Bool(FIELD(j, path, "rear_doors_open"));
  c.note = String(FIELD(j, path, "note"));
  return c;
}

TubePlace ReadTube(const Json& j, const std::string& path) {
  Fields(j, path, {"axis_x", "axis_z", "plain_end_y"});
  TubePlace t;
  t.axis_x = Number(FIELD(j, path, "axis_x"));
  t.axis_z = Number(FIELD(j, path, "axis_z"));
  t.plain_end_y = Number(FIELD(j, path, "plain_end_y"));
  return t;
}

Holder ReadHolder(const Json& j, const std::string& path) {
  Fields(j, path,
         {"does", "grips", "angle_pin_hold", "resists_nm", "holds_within_deg",
          "clamp_s", "release_s", "mass_kg", "boxes", "travels_inside",
          "basis", "source"});
  Holder h;
  h.does = String(FIELD(j, path, "does"));
  const std::string grips_path = Sub(path, "grips");
  const Json& grips = j.at("grips");
  Array(grips, grips_path);
  for (size_t i = 0; i < grips.size(); ++i) {
    const std::string p = At(grips_path, i);
    Fields(grips[i], p, {"hold", "length"});
    Grip g;
    g.hold = Integer(FIELD(grips[i], p, "hold"));
    g.length = Number(FIELD(grips[i], p, "length"));
    h.grips.push_back(g);
  }
  h.angle_pin_hold = Integer(FIELD(j, path, "angle_pin_hold"));
  h.resists_nm = Number(FIELD(j, path, "resists_nm"));
  h.holds_within_deg = Number(FIELD(j, path, "holds_within_deg"));
  h.clamp_s = Number(FIELD(j, path, "clamp_s"));
  h.release_s = Number(FIELD(j, path, "release_s"));
  h.mass_kg = Number(FIELD(j, path, "mass_kg"));
  h.boxes = ReadBoxes(FIELD(j, path, "boxes"));
  h.travels_inside = Bool(FIELD(j, path, "travels_inside"));
  h.basis = String(FIELD(j, path, "basis"));
  h.source = String(FIELD(j, path, "source"));
  return h;
}

Tool ReadTool(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "does", "mass_kg", "length", "boxes", "waits_at",
          "wait_size", "basis", "source"});
  Tool t;
  t.name = String(FIELD(j, path, "name"));
  t.does = String(FIELD(j, path, "does"));
  t.mass_kg = Number(FIELD(j, path, "mass_kg"));
  t.length = Number(FIELD(j, path, "length"));
  t.boxes = ReadBoxes(FIELD(j, path, "boxes"));
  t.waits_at = Numbers<3>(FIELD(j, path, "waits_at"));
  t.wait_size = Numbers<3>(FIELD(j, path, "wait_size"));
  t.basis = String(FIELD(j, path, "basis"));
  t.source = String(FIELD(j, path, "source"));
  return t;
}

Track ReadTrack(const Json& j, const std::string& path) {
  Fields(j, path,
         {"direction", "lo", "hi", "speed", "accel", "mass_kg", "beam",
          "carriage", "basis", "source"});
  Track t;
  t.direction = Numbers<3>(FIELD(j, path, "direction"));
  const double norm2 = t.direction[0] * t.direction[0] +
                       t.direction[1] * t.direction[1] +
                       t.direction[2] * t.direction[2];
  if (norm2 < 1e-12) {
    throw LayoutError(Sub(path, "direction"), "the direction is zero");
  }
  t.lo = Number(FIELD(j, path, "lo"));
  t.hi = Number(FIELD(j, path, "hi"));
  if (t.hi < t.lo) throw LayoutError(Sub(path, "hi"), "hi is below lo");
  t.speed = Number(FIELD(j, path, "speed"));
  t.accel = Number(FIELD(j, path, "accel"));
  t.mass_kg = Number(FIELD(j, path, "mass_kg"));
  t.beam = ReadBox(FIELD(j, path, "beam"));
  t.carriage = ReadBox(FIELD(j, path, "carriage"));
  t.basis = String(FIELD(j, path, "basis"));
  t.source = String(FIELD(j, path, "source"));
  return t;
}

bool IsWorkKind(const std::string& kind) {
  const auto& kinds = WorkKinds();
  return std::find(kinds.begin(), kinds.end(), kind) != kinds.end();
}

ArmUse ReadArm(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "model", "does", "base_pos", "base_rpy_deg", "tools",
          "first_tool", "stops", "wait_joints_deg", "wait_track",
          "stow_joints_deg", "stow_track", "stow_tool", "tool_change_s"},
         {"track"});
  ArmUse a;
  a.name = String(FIELD(j, path, "name"));
  a.model = String(FIELD(j, path, "model"));
  if (FindArmModel(a.model) == nullptr) {
    throw LayoutError(Sub(path, "model"),
                      "no arm \"" + a.model + "\" in src/arm_catalog.h");
  }
  a.does = String(FIELD(j, path, "does"));
  a.base_pos = Numbers<3>(FIELD(j, path, "base_pos"));
  a.base_rpy_deg = Numbers<3>(FIELD(j, path, "base_rpy_deg"));
  if (j.contains("track") && !j.at("track").is_null()) {
    a.track = ReadTrack(FIELD(j, path, "track"));
  }
  const std::string tools_path = Sub(path, "tools");
  const Json& tools = j.at("tools");
  Array(tools, tools_path);
  for (size_t i = 0; i < tools.size(); ++i) {
    a.tools.push_back(ReadTool(tools[i], At(tools_path, i)));
  }
  Unique(a.tools, tools_path, "tool");
  a.first_tool = String(FIELD(j, path, "first_tool"));
  if (!a.first_tool.empty() && FindTool(a, a.first_tool) == nullptr) {
    throw LayoutError(Sub(path, "first_tool"),
                      "the arm has no tool \"" + a.first_tool + "\"");
  }
  const std::string stops_path = Sub(path, "stops");
  const Json& stops = j.at("stops");
  if (!stops.is_object()) {
    throw LayoutError(stops_path, "expected an object, found " + TypeName(stops));
  }
  for (const auto& [kind, values] : stops.items()) {
    const std::string p = Sub(stops_path, kind.c_str());
    if (!IsWorkKind(kind)) throw LayoutError(p, "not a kind of work");
    Array(values, p);
    std::vector<double> v;
    for (size_t i = 0; i < values.size(); ++i) v.push_back(Number(values[i], At(p, i)));
    if (v.empty()) throw LayoutError(p, "no stop given");
    a.stops[kind] = v;
  }
  a.wait_joints_deg = Numbers<6>(FIELD(j, path, "wait_joints_deg"));
  a.wait_track = Number(FIELD(j, path, "wait_track"));
  a.stow_joints_deg = Numbers<6>(FIELD(j, path, "stow_joints_deg"));
  a.stow_track = Number(FIELD(j, path, "stow_track"));
  a.stow_tool = String(FIELD(j, path, "stow_tool"));
  if (!a.stow_tool.empty() && FindTool(a, a.stow_tool) == nullptr) {
    throw LayoutError(Sub(path, "stow_tool"),
                      "the arm has no tool \"" + a.stow_tool + "\"");
  }
  a.tool_change_s = Number(FIELD(j, path, "tool_change_s"));
  return a;
}

Pallet ReadPallet(const Json& j, const std::string& path) {
  Fields(j, path,
         {"pos", "yaw_deg", "modules", "stands", "how", "base_height", "pitch",
          "outside", "basis", "source"});
  Pallet p;
  p.pos = Numbers<3>(FIELD(j, path, "pos"));
  p.yaw_deg = Number(FIELD(j, path, "yaw_deg"));
  p.modules = Integer(FIELD(j, path, "modules"));
  if (p.modules < 0) throw LayoutError(Sub(path, "modules"), "is negative");
  p.stands = String(FIELD(j, path, "stands"));
  if (p.stands != "on_long_edge" && p.stands != "flat") {
    throw LayoutError(Sub(path, "stands"),
                      "is \"on_long_edge\" or \"flat\", not \"" + p.stands + "\"");
  }
  p.how = String(FIELD(j, path, "how"));
  p.base_height = Number(FIELD(j, path, "base_height"));
  p.pitch = Number(FIELD(j, path, "pitch"));
  p.outside = Bool(FIELD(j, path, "outside"));
  p.basis = String(FIELD(j, path, "basis"));
  p.source = String(FIELD(j, path, "source"));
  return p;
}

Stock ReadStock(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "part", "how", "pick_pos", "pick_axis", "holds", "mass_kg",
          "boxes", "outside", "basis", "source"});
  Stock s;
  s.name = String(FIELD(j, path, "name"));
  s.part = String(FIELD(j, path, "part"));
  s.how = String(FIELD(j, path, "how"));
  s.pick_pos = Numbers<3>(FIELD(j, path, "pick_pos"));
  s.pick_axis = Numbers<3>(FIELD(j, path, "pick_axis"));
  s.holds = Integer(FIELD(j, path, "holds"));
  s.mass_kg = Number(FIELD(j, path, "mass_kg"));
  s.boxes = ReadBoxes(FIELD(j, path, "boxes"));
  s.outside = Bool(FIELD(j, path, "outside"));
  s.basis = String(FIELD(j, path, "basis"));
  s.source = String(FIELD(j, path, "source"));
  return s;
}

Way ReadWay(const Json& j, const std::string& path) {
  Fields(j, path, {"what", "path", "time_s", "basis"});
  Way w;
  w.what = String(FIELD(j, path, "what"));
  const std::string pp = Sub(path, "path");
  const Json& points = j.at("path");
  Array(points, pp);
  for (size_t i = 0; i < points.size(); ++i) {
    w.path.push_back(Numbers<3>(points[i], At(pp, i)));
  }
  w.time_s = Number(FIELD(j, path, "time_s"));
  w.basis = String(FIELD(j, path, "basis"));
  return w;
}

Work ReadWork(const Json& j, const std::string& path) {
  Fields(j, path, {"kind", "arm", "tool", "process_s", "note", "basis"});
  Work w;
  w.kind = String(FIELD(j, path, "kind"));
  if (!IsWorkKind(w.kind)) {
    throw LayoutError(Sub(path, "kind"),
                      "\"" + w.kind + "\" is not a kind of work of src/layout.h");
  }
  w.arm = String(FIELD(j, path, "arm"));
  w.tool = String(FIELD(j, path, "tool"));
  w.process_s = Number(FIELD(j, path, "process_s"));
  w.note = String(FIELD(j, path, "note"));
  w.basis = String(FIELD(j, path, "basis"));
  return w;
}

Item ReadItem(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "does", "travels_inside", "mass_kg", "boxes", "basis",
          "source"});
  Item it;
  it.name = String(FIELD(j, path, "name"));
  it.does = String(FIELD(j, path, "does"));
  it.travels_inside = Bool(FIELD(j, path, "travels_inside"));
  it.mass_kg = Number(FIELD(j, path, "mass_kg"));
  it.boxes = ReadBoxes(FIELD(j, path, "boxes"));
  it.basis = String(FIELD(j, path, "basis"));
  it.source = String(FIELD(j, path, "source"));
  return it;
}

Layout ReadLayout(const Json& j) {
  const std::string path;
  Fields(j, path,
         {"name", "title", "summary", "chosen", "small_arms_stand",
          "container", "tube", "holder", "arms", "pallet", "stocks",
          "strap_bolts_from", "tube_in", "bay_out", "work", "rail_order",
          "items"});
  Layout l;
  l.name = String(FIELD(j, path, "name"));
  l.title = String(FIELD(j, path, "title"));
  l.summary = String(FIELD(j, path, "summary"));
  l.chosen = Bool(FIELD(j, path, "chosen"));
  l.small_arms_stand = String(FIELD(j, path, "small_arms_stand"));
  if (l.small_arms_stand != "below" && l.small_arms_stand != "beside" &&
      l.small_arms_stand != "above") {
    throw LayoutError("small_arms_stand",
                      "is \"below\", \"beside\" or \"above\", not \"" +
                          l.small_arms_stand + "\"");
  }
  l.container = ReadContainer(FIELD(j, path, "container"));
  l.tube = ReadTube(FIELD(j, path, "tube"));
  l.holder = ReadHolder(FIELD(j, path, "holder"));
  const Json& arms = j.at("arms");
  Array(arms, "arms");
  for (size_t i = 0; i < arms.size(); ++i) {
    l.arms.push_back(ReadArm(arms[i], At("arms", i)));
  }
  Unique(l.arms, "arms", "arm");
  l.pallet = ReadPallet(FIELD(j, path, "pallet"));
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
  std::set<std::string> kinds;
  for (size_t i = 0; i < work.size(); ++i) {
    const std::string p = At("work", i);
    Work w = ReadWork(work[i], p);
    if (!kinds.insert(w.kind).second) {
      throw LayoutError(p + ".kind", "the kind \"" + w.kind + "\" is given twice");
    }
    const ArmUse* arm = FindArm(l, w.arm);
    if (arm == nullptr) {
      throw LayoutError(p + ".arm", "no arm \"" + w.arm + "\" in the layout");
    }
    if (FindTool(*arm, w.tool) == nullptr) {
      throw LayoutError(p + ".tool", "the arm \"" + w.arm + "\" has no tool \"" +
                                         w.tool + "\"");
    }
    l.work.push_back(w);
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
  } catch (const LayoutError& e) {
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

const ArmUse* FindArm(const Layout& layout, const std::string& name) {
  for (const ArmUse& a : layout.arms) {
    if (a.name == name) return &a;
  }
  return nullptr;
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

}  // namespace cell::layout
