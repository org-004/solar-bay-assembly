// The reader of two flat racks and of the stages of a tube, and the layouts
// derived from them (src/layout_racks.h, src/layout.h).

#include "src/layout_racks.h"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace cell::layout {
namespace {

using Json = nlohmann::json;

[[noreturn]] void Fail(const std::string& field, const std::string& what) {
  throw std::runtime_error(field + ": " + what);
}

void Known(const Json& j, const std::string& path,
           const std::vector<std::string>& keys) {
  if (!j.is_object()) Fail(path, "expected an object");
  for (const auto& [key, value] : j.items()) {
    if (std::find(keys.begin(), keys.end(), key) == keys.end()) {
      Fail(path + "." + key, "unknown field");
    }
  }
  for (const std::string& k : keys) {
    if (!j.contains(k)) Fail(path + "." + k, "missing field");
  }
}

double Num(const Json& j, const std::string& path) {
  if (!j.is_number()) Fail(path, "expected a number");
  return j.get<double>();
}

int Int(const Json& j, const std::string& path) {
  if (!j.is_number_integer()) Fail(path, "expected a whole number");
  return j.get<int>();
}

std::string Str(const Json& j, const std::string& path) {
  if (!j.is_string()) Fail(path, "expected a string");
  return j.get<std::string>();
}

std::string Index(const std::string& path, size_t i) {
  return path + "[" + std::to_string(i) + "]";
}

TwoFlatRacksState ReadTwoRacks(const Json& j, const std::string& path) {
  Known(j, path,
        {"end_frames", "gap_m", "gap_lo_m", "gap_hi_m", "misalign_m", "note"});
  TwoFlatRacksState t;
  t.end_frames = Str(j.at("end_frames"), path + ".end_frames");
  if (t.end_frames != "taken_off" && t.end_frames != "dropped") {
    Fail(path + ".end_frames", "is \"taken_off\" or \"dropped\", not \"" +
                                   t.end_frames + "\"");
  }
  t.gap_m = Num(j.at("gap_m"), path + ".gap_m");
  t.gap_lo_m = Num(j.at("gap_lo_m"), path + ".gap_lo_m");
  t.gap_hi_m = Num(j.at("gap_hi_m"), path + ".gap_hi_m");
  t.misalign_m = Num(j.at("misalign_m"), path + ".misalign_m");
  t.note = Str(j.at("note"), path + ".note");
  if (!(0.0 <= t.gap_lo_m && t.gap_lo_m <= t.gap_m && t.gap_m <= t.gap_hi_m)) {
    Fail(path, "the gap is not within its tolerance: 0 <= gap_lo_m <= gap_m "
               "<= gap_hi_m");
  }
  if (t.misalign_m < 0.0) Fail(path + ".misalign_m", "is negative");
  return t;
}

Staging ReadStaging(const Json& j, const std::string& path, const Layout& l) {
  Known(j, path, {"why", "transfer_s", "stages", "basis"});
  Staging s;
  s.why = Str(j.at("why"), path + ".why");
  s.transfer_s = Num(j.at("transfer_s"), path + ".transfer_s");
  s.basis = Str(j.at("basis"), path + ".basis");
  const std::string sp = path + ".stages";
  const Json& stages = j.at("stages");
  if (!stages.is_array() || stages.empty()) Fail(sp, "expected stages, none given");
  const int rails = PointCount(l, "rail_set");
  const int modules = PointCount(l, "module_set");
  const int holds = rails + 1;  // bay::kHoldCount: the stretches between rails.
  std::vector<int> rail_stage(rails, -1), module_stage(modules, -1);
  std::set<std::string> names;
  for (size_t i = 0; i < stages.size(); ++i) {
    const std::string p = Index(sp, i);
    const Json& o = stages[i];
    Known(o, p,
          {"name", "tube_offset_m", "rails", "modules", "grips",
           "angle_pin_hold", "angle_pin_by", "held_by", "note"});
    Stage st;
    st.name = Str(o.at("name"), p + ".name");
    if (!names.insert(st.name).second) {
      Fail(p + ".name", "stage \"" + st.name + "\" is used twice");
    }
    st.tube_offset_m = Num(o.at("tube_offset_m"), p + ".tube_offset_m");
    st.rails = Str(o.at("rails"), p + ".rails");
    st.modules = Str(o.at("modules"), p + ".modules");
    const Json& grips = o.at("grips");
    if (!grips.is_array()) Fail(p + ".grips", "expected an array");
    for (size_t g = 0; g < grips.size(); ++g) {
      const std::string gp = Index(p + ".grips", g);
      Known(grips[g], gp, {"hold", "length"});
      Grip grip;
      grip.hold = Int(grips[g].at("hold"), gp + ".hold");
      grip.length = Num(grips[g].at("length"), gp + ".length");
      if (grip.hold < 0 || grip.hold >= holds) {
        Fail(gp + ".hold", "no stretch bay_hold_" + std::to_string(grip.hold));
      }
      st.grips.push_back(grip);
    }
    st.angle_pin_hold = Int(o.at("angle_pin_hold"), p + ".angle_pin_hold");
    st.angle_pin_by = Str(o.at("angle_pin_by"), p + ".angle_pin_by");
    if (st.angle_pin_by != "holder") {
      const std::string prefix = "item:";
      const std::string name = st.angle_pin_by.rfind(prefix, 0) == 0
                                   ? st.angle_pin_by.substr(prefix.size())
                                   : "";
      const bool found = std::any_of(
          l.items.begin(), l.items.end(),
          [&](const Item& it) { return !name.empty() && it.name == name; });
      if (!found) {
        Fail(p + ".angle_pin_by",
             "is \"holder\" or \"item:<name>\" of an item of the layout, not \"" +
                 st.angle_pin_by + "\"");
      }
    }
    st.held_by = Str(o.at("held_by"), p + ".held_by");
    st.note = Str(o.at("note"), p + ".note");
    auto claim = [&](const std::string& text, int count, std::vector<int>* own,
                     const std::string& field, const char* what) {
      if (text.empty()) return;
      for (int n : ParsePointList(text, count, field)) {
        if ((*own)[n] >= 0) {
          Fail(field, std::string(what) + " " + std::to_string(n) +
                          " is in two stages");
        }
        (*own)[n] = static_cast<int>(i);
      }
    };
    claim(st.rails, rails, &rail_stage, p + ".rails", "rail");
    claim(st.modules, modules, &module_stage, p + ".modules", "module");
    s.stages.push_back(st);
  }
  for (int n = 0; n < rails; ++n) {
    if (rail_stage[n] < 0) Fail(sp, "rail " + std::to_string(n) + " is in no stage");
  }
  for (int n = 0; n < modules; ++n) {
    if (module_stage[n] < 0) {
      Fail(sp, "module " + std::to_string(n) + " is in no stage");
    }
    // Module n lies on rails n and n + 1: both are set by its stage.
    for (int r : {n, n + 1}) {
      if (r < rails && rail_stage[r] > module_stage[n]) {
        Fail(Index(sp, module_stage[n]) + ".modules",
             "module " + std::to_string(n) + " is set before its rail " +
                 std::to_string(r) + " (stage " +
                 s.stages[rail_stage[r]].name + ")");
      }
    }
  }
  return s;
}

// The rack of one thing: required ("a" or "b") where `needed`, refused
// otherwise.
void CheckRack(const std::string& rack, bool needed, const std::string& path,
               const std::string& why_not) {
  if (needed && rack != "a" && rack != "b") {
    Fail(path, rack.empty() ? "missing field: the rack it stands on, \"a\" or \"b\""
                            : "is \"a\" or \"b\", not \"" + rack + "\"");
  }
  if (!needed && !rack.empty()) Fail(path, "unknown field: " + why_not);
}

}  // namespace

void ReadRacksAndStages(const Json& top, Layout* layout) {
  Layout& l = *layout;
  const bool two = l.base == "two_flat_racks";
  if (two) l.two_flat_racks = ReadTwoRacks(top.at("two_flat_racks"), "two_flat_racks");
  const std::string other =
      "only a layout on two_flat_racks says which rack a thing stands on";
  for (size_t i = 0; i < l.arms.size(); ++i) {
    CheckRack(l.arms[i].rack, two, Index("arms", i) + ".rack", other);
  }
  CheckRack(l.holder.rack, two, "holder.rack", other);
  for (size_t i = 0; i < l.stocks.size(); ++i) {
    const bool on = two && !l.stocks[i].outside;
    CheckRack(l.stocks[i].rack, on, Index("stocks", i) + ".rack",
              two ? "a stock outside stands on no rack" : other);
  }
  for (size_t i = 0; i < l.items.size(); ++i) {
    const bool on = two && l.items[i].travels_inside;
    CheckRack(l.items[i].rack, on, Index("items", i) + ".rack",
              two ? "an item that does not travel stands on no rack" : other);
  }
  if (top.contains("staging")) {
    l.staging = ReadStaging(top.at("staging"), "staging", l);
  }
}

std::vector<int> StageRails(const Stage& stage) {
  if (stage.rails.empty()) return {};
  if (stage.rails == "all") {
    std::vector<int> all(10);
    for (int i = 0; i < 10; ++i) all[i] = i;
    return all;
  }
  return ParsePointList(stage.rails, 10, "rails");
}

std::vector<int> StageModules(const Stage& stage) {
  if (stage.modules.empty()) return {};
  if (stage.modules == "all") {
    std::vector<int> all(9);
    for (int i = 0; i < 9; ++i) all[i] = i;
    return all;
  }
  return ParsePointList(stage.modules, 9, "modules");
}

Layout AtStage(const Layout& layout, const Stage& stage) {
  Layout l = layout;
  l.tube.plain_end_y += stage.tube_offset_m;
  l.holder.grips = stage.grips;
  l.holder.angle_pin_hold = stage.angle_pin_hold;
  return l;
}

Layout MovedRackB(const Layout& layout, double dx, double dy) {
  Layout l = layout;
  if (l.base != "two_flat_racks") return l;
  auto move = [&](Vec3* p) {
    (*p)[0] += dx;
    (*p)[1] += dy;
  };
  l.two_flat_racks.b_shift[0] += dx;
  l.two_flat_racks.b_shift[1] += dy;
  for (ArmUse& a : l.arms) {
    if (a.rack != "b") continue;
    move(&a.base_pos);
    if (a.track) move(&a.track->beam.pos);
  }
  if (l.holder.rack == "b") {
    for (Box& b : l.holder.boxes) move(&b.pos);
    l.tube.axis_x += dx;
    l.tube.plain_end_y += dy;
  }
  for (Stock& s : l.stocks) {
    if (s.rack != "b") continue;
    move(&s.pick_pos);
    for (Box& b : s.boxes) move(&b.pos);
  }
  for (Item& it : l.items) {
    if (it.rack != "b") continue;
    for (Box& b : it.boxes) move(&b.pos);
  }
  return l;
}

}  // namespace cell::layout
