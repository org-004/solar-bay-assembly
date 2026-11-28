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

bool IsWorkKind(const std::string& kind) {
  const auto& kinds = WorkKinds();
  return std::find(kinds.begin(), kinds.end(), kind) != kinds.end();
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

FlatRackState ReadFlatRack(const Json& j, const std::string& path) {
  Fields(j, path, {"end_frames", "note"});
  FlatRackState f;
  f.end_frames = String(FIELD(j, path, "end_frames"));
  if (f.end_frames != "taken_off" && f.end_frames != "dropped") {
    throw LayoutError(Sub(path, "end_frames"),
                      "is \"taken_off\" or \"dropped\", not \"" +
                          f.end_frames + "\"");
  }
  f.note = String(FIELD(j, path, "note"));
  return f;
}

Margins ReadMargins(const Json& j, const std::string& path) {
  Fields(j, path, {"clearance_m", "joint_margin_deg", "reason", "basis"});
  Margins m;
  m.clearance_m = Number(FIELD(j, path, "clearance_m"));
  if (!(m.clearance_m >= 0.010)) {
    throw LayoutError(Sub(path, "clearance_m"),
                      "is below the floor of 0.010 m");
  }
  m.joint_margin_deg = Number(FIELD(j, path, "joint_margin_deg"));
  if (!(m.joint_margin_deg >= 5.0)) {
    throw LayoutError(Sub(path, "joint_margin_deg"),
                      "is below the floor of 5.0 degrees");
  }
  m.reason = String(FIELD(j, path, "reason"));
  m.basis = String(FIELD(j, path, "basis"));
  return m;
}

// A box given without a name, as a port's zone is.
Box ReadZone(const Json& j, const std::string& path) {
  Fields(j, path, {"size", "pos"});
  Box box;
  box.size = Numbers<3>(FIELD(j, path, "size"));
  box.pos = Numbers<3>(FIELD(j, path, "pos"));
  for (size_t i = 0; i < 3; ++i) {
    if (box.size[i] < 0.0) {
      throw LayoutError(At(Sub(path, "size"), i), "a size is not negative");
    }
  }
  return box;
}

Port ReadPort(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "crosses", "unit", "frame_pos", "frame_yaw_deg", "zone",
          "hands_over", "shares_zone_with", "presents", "per_bay", "basis"});
  Port p;
  p.name = String(FIELD(j, path, "name"));
  p.crosses = String(FIELD(j, path, "crosses"));
  if (p.crosses != "tubes" && p.crosses != "bays" && p.crosses != "modules" &&
      p.crosses != "rails" && p.crosses != "pin_magazines") {
    throw LayoutError(Sub(path, "crosses"),
                      "is \"tubes\", \"bays\", \"modules\", \"rails\" or "
                      "\"pin_magazines\", not \"" + p.crosses + "\"");
  }
  p.unit = String(FIELD(j, path, "unit"));
  p.frame_pos = Numbers<3>(FIELD(j, path, "frame_pos"));
  p.frame_yaw_deg = Number(FIELD(j, path, "frame_yaw_deg"));
  p.zone = ReadZone(FIELD(j, path, "zone"));
  p.hands_over = Strings(FIELD(j, path, "hands_over"));
  p.shares_zone_with = String(FIELD(j, path, "shares_zone_with"));
  p.presents = String(FIELD(j, path, "presents"));
  p.per_bay = Number(FIELD(j, path, "per_bay"));
  p.basis = String(FIELD(j, path, "basis"));
  return p;
}

// A direction of unit length, within 1e-6.
Vec3 UnitDirection(const Json& j, const std::string& path) {
  const Vec3 d = Numbers<3>(j, path);
  const double norm = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
  if (std::abs(norm - 1.0) > 1e-6) {
    throw LayoutError(path, "is not of unit length (its length is " +
                                std::to_string(norm) + ")");
  }
  return d;
}

std::string Frame(const Json& j, const std::string& path) {
  const std::string f = String(j, path);
  if (f != "tool" && f != "world") {
    throw LayoutError(path, "is \"tool\" or \"world\", not \"" + f + "\"");
  }
  return f;
}

double Positive(const Json& j, const std::string& path) {
  const double v = Number(j, path);
  if (!(v > 0.0)) throw LayoutError(path, "is not above zero");
  return v;
}

Approach ReadApproach(const Json& j, const std::string& path) {
  Fields(j, path,
         {"kind", "approach_frame", "approach_dir", "approach_m",
          "retreat_frame", "retreat_dir", "retreat_m", "note", "basis"});
  Approach a;
  a.kind = String(FIELD(j, path, "kind"));
  if (!IsWorkKind(a.kind)) {
    throw LayoutError(Sub(path, "kind"),
                      "\"" + a.kind + "\" is not a kind of work of src/layout.h");
  }
  a.approach_frame = Frame(FIELD(j, path, "approach_frame"));
  a.approach_dir = UnitDirection(FIELD(j, path, "approach_dir"));
  a.approach_m = Positive(FIELD(j, path, "approach_m"));
  a.retreat_frame = Frame(FIELD(j, path, "retreat_frame"));
  a.retreat_dir = UnitDirection(FIELD(j, path, "retreat_dir"));
  a.retreat_m = Positive(FIELD(j, path, "retreat_m"));
  a.note = String(FIELD(j, path, "note"));
  a.basis = String(FIELD(j, path, "basis"));
  return a;
}

// Reads a "points" string: "all", or numbers and ranges such as
// "0-11,14,20-23", both ends included, each number in 0 .. count - 1.
// Spaces around the numbers are allowed.
std::vector<int> ParsePoints(const std::string& text, int count,
                             const std::string& path) {
  std::vector<int> out;
  auto trim = [](const std::string& s) {
    const size_t a = s.find_first_not_of(" \t");
    if (a == std::string::npos) return std::string();
    const size_t b = s.find_last_not_of(" \t");
    return s.substr(a, b - a + 1);
  };
  if (trim(text) == "all") {
    for (int i = 0; i < count; ++i) out.push_back(i);
    return out;
  }
  auto number = [&](const std::string& s) {
    const std::string t = trim(s);
    if (t.empty() || t.size() > 6 ||
        t.find_first_not_of("0123456789") != std::string::npos) {
      throw LayoutError(path, "\"" + text +
                                  "\" is not \"all\" or numbers and ranges "
                                  "such as \"0-11,14\"");
    }
    const int n = std::stoi(t);
    if (n >= count) {
      throw LayoutError(path, "the point " + std::to_string(n) +
                                  " is outside 0.." + std::to_string(count - 1));
    }
    return n;
  };
  size_t start = 0;
  while (true) {
    const size_t comma = text.find(',', start);
    const std::string part = text.substr(
        start, comma == std::string::npos ? std::string::npos : comma - start);
    const size_t dash = part.find('-');
    if (dash == std::string::npos) {
      out.push_back(number(part));
    } else {
      const int lo = number(part.substr(0, dash));
      const int hi = number(part.substr(dash + 1));
      if (hi < lo) {
        throw LayoutError(path, "the range \"" + trim(part) + "\" runs backward");
      }
      for (int i = lo; i <= hi; ++i) out.push_back(i);
    }
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  std::sort(out.begin(), out.end());
  if (std::adjacent_find(out.begin(), out.end()) != out.end()) {
    throw LayoutError(path, "\"" + text + "\" gives a point twice");
  }
  return out;
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
          "basis", "source"},
         {"rack"});
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
  if (j.contains("rack")) h.rack = String(FIELD(j, path, "rack"));
  return h;
}

Tool ReadTool(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "does", "mass_kg", "length", "boxes", "waits_at",
          "wait_size", "basis", "source"},
         {"rail_magazine"});
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
  if (j.contains("rail_magazine")) {
    // A rail magazine rides on the tool (src/layout_rail_magazine.h): its
    // boxes join the tool's before its last (the one that touches the work)
    // and its empty mass joins the tool's, so that the checker sweeps it
    // and the torque counts it wherever the tool goes.
    const std::string mp = Sub(path, "rail_magazine");
    const Json& m = j.at("rail_magazine");
    Fields(m, mp, {"does", "holds", "mass_kg", "boxes", "basis", "source"});
    t.magazine_holds = Integer(FIELD(m, mp, "holds"));
    if (t.magazine_holds < 1) throw LayoutError(Sub(mp, "holds"), "holds at least one rail");
    const double mass = Number(FIELD(m, mp, "mass_kg"));
    if (mass < 0.0) throw LayoutError(Sub(mp, "mass_kg"), "is negative");
    t.mass_kg += mass;
    std::vector<Box> boxes = ReadBoxes(FIELD(m, mp, "boxes"));
    if (t.boxes.empty()) throw LayoutError(Sub(path, "boxes"), "a tool with a magazine has boxes");
    t.boxes.insert(t.boxes.end() - 1, boxes.begin(), boxes.end());
    for (const char* key : {"does", "basis", "source"}) {
      if (String(FIELD(m, mp, key)).empty()) throw LayoutError(Sub(mp, key), "is empty");
    }
  }
  return t;
}

Track ReadTrack(const Json& j, const std::string& path) {
  Fields(j, path,
         {"direction", "lo", "hi", "speed", "accel", "mass_kg", "beam",
          "carriage", "basis", "source"},
         {"shares_beam_with"});
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
  if (j.contains("shares_beam_with")) {
    t.shares_beam_with = String(FIELD(j, path, "shares_beam_with"));
  }
  return t;
}

ArmUse ReadArm(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "model", "does", "base_pos", "base_rpy_deg", "tools",
          "first_tool", "stops", "wait_joints_deg", "wait_track",
          "stow_joints_deg", "stow_track", "stow_tool", "tool_change_s"},
         {"track", "rack", "one_posture_per_module"});
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
  if (j.contains("rack")) a.rack = String(FIELD(j, path, "rack"));
  if (j.contains("one_posture_per_module")) {
    a.one_posture_per_module = Bool(FIELD(j, path, "one_posture_per_module"));
  }
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
          "boxes", "outside", "basis", "source"},
         {"rack"});
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
  if (j.contains("rack")) s.rack = String(FIELD(j, path, "rack"));
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
  Fields(j, path,
         {"kind", "arm", "tool", "points", "process_s", "note", "basis"});
  Work w;
  w.kind = String(FIELD(j, path, "kind"));
  if (!IsWorkKind(w.kind)) {
    throw LayoutError(Sub(path, "kind"),
                      "\"" + w.kind + "\" is not a kind of work of src/layout.h");
  }
  w.arm = String(FIELD(j, path, "arm"));
  w.tool = String(FIELD(j, path, "tool"));
  w.points = String(FIELD(j, path, "points"));
  w.process_s = Number(FIELD(j, path, "process_s"));
  w.note = String(FIELD(j, path, "note"));
  w.basis = String(FIELD(j, path, "basis"));
  return w;
}

Item ReadItem(const Json& j, const std::string& path) {
  Fields(j, path,
         {"name", "does", "travels_inside", "mass_kg", "boxes", "basis",
          "source"},
         {"rack"});
  Item it;
  it.name = String(FIELD(j, path, "name"));
  it.does = String(FIELD(j, path, "does"));
  it.travels_inside = Bool(FIELD(j, path, "travels_inside"));
  it.mass_kg = Number(FIELD(j, path, "mass_kg"));
  it.boxes = ReadBoxes(FIELD(j, path, "boxes"));
  it.basis = String(FIELD(j, path, "basis"));
  it.source = String(FIELD(j, path, "source"));
  if (j.contains("rack")) it.rack = String(FIELD(j, path, "rack"));
  return it;
}

// The text of a hands_over entry names something of the layout: a pallet
// place by index, a stock, an item, or one of the two ways.
void CheckHandsOver(const Layout& l, const std::string& h,
                    const std::string& path) {
  const size_t colon = h.find(':');
  const std::string what = colon == std::string::npos ? h : h.substr(0, colon);
  const std::string name = colon == std::string::npos ? "" : h.substr(colon + 1);
  if (colon != std::string::npos && what == "pallet") {
    const bool digits = !name.empty() && name.size() < 6 &&
                        name.find_first_not_of("0123456789") == std::string::npos;
    if (!digits || std::stoi(name) >= static_cast<int>(l.pallets.size())) {
      throw LayoutError(path, "\"" + h + "\" names no pallet place: there are " +
                                  std::to_string(l.pallets.size()));
    }
    return;
  }
  if (colon != std::string::npos && what == "stock") {
    if (FindStock(l, name) == nullptr) {
      throw LayoutError(path, "\"" + h + "\" names no stock of the layout");
    }
    return;
  }
  if (colon != std::string::npos && what == "item") {
    for (const Item& it : l.items) {
      if (it.name == name) return;
    }
    throw LayoutError(path, "\"" + h + "\" names no item of the layout");
  }
  if (colon != std::string::npos && what == "way") {
    if (name == "tube_in" || name == "bay_out") return;
    throw LayoutError(path, "\"" + h + "\" names no way: tube_in or bay_out");
  }
  throw LayoutError(path, "\"" + h +
                              "\" is not \"pallet:<index>\", \"stock:<name>\", "
                              "\"item:<name>\" or \"way:tube_in|bay_out\"");
}

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
            "strap_bolts_from", "tube_in", "bay_out", "work", "rail_order",
            "items"});
  } else if (flat_rack) {
    Fields(j, path,
           {"name", "title", "summary", "small_arms_stand", "base",
            two_racks ? "two_flat_racks" : "flat_rack", "margins", "ports", "approaches", "tube", "holder",
            "arms", "pallets", "stocks", "strap_bolts_from", "tube_in",
            "bay_out", "work", "rail_order", "items"},
           {"order", "staging", "retreat_legs", "path_tries", "torque_limit_scale",
            "bay_out_when_clear"});
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
  if (j.contains("path_tries")) {
    const int tries = Integer(j.at("path_tries"), "path_tries");
    if (tries < 2 || tries > kMaxPathTries) {
      throw LayoutError("path_tries", "is not an integer from 2 to 16");
    }
    l.path_tries = tries;
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
