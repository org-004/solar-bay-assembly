#include "src/bay_model.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

namespace cell::bay {
namespace {

// What the header does not hold: the look of the model.
constexpr const char* kSteelRgba = "0.62 0.64 0.66 1";
constexpr const char* kAluminiumRgba = "0.80 0.82 0.85 1";
constexpr const char* kLaminateRgba = "0.08 0.12 0.32 1";
constexpr const char* kSiteARgba = "1 0.5 0 1";
constexpr const char* kSiteBRgba = "0.1 0.8 0.2 1";
constexpr const char* kSiteLocateRgba = "0.9 0.1 0.8 1";
constexpr const char* kSiteHoldRgba = "1 0.9 0.1 0.25";
constexpr const char* kSiteSize = "0.004";
constexpr const char* kSiteHoldGroup = "3";

// The classes of the file and the geom or site names that carry them.
constexpr const char* kSteel = "bay_steel";
constexpr const char* kAluminium = "bay_aluminium";
constexpr const char* kLaminate = "bay_laminate";
constexpr const char* kSiteA = "bay_site_a";
constexpr const char* kSiteB = "bay_site_b";
constexpr const char* kSiteLocate = "bay_site_locate";
constexpr const char* kSiteHold = "bay_site_hold";
constexpr const char* kFrameName = "bay_frame";

// The sentence that ends the long comment, about where the file comes from.
constexpr const char* kProvenance =
    "  This file is generated: //:gen_bay prints it from src/bay_model.cc and\n"
    "  the header, and //:bay_model_test fails when the file differs from that\n"
    "  output. After a change to either, write it again with\n"
    "    bazel run //:gen_bay > models/bay.xml\n"
    "  built in the opt configuration, as AGENTS.md says.\n";

// Positions and sizes: twelve significant digits, trailing zeros dropped.
// A value that rounds to zero is written 0, never -0.
std::string Num(double value) {
  if (std::fabs(value) < 5e-13) return "0";
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  return buffer;
}

std::string Vec3(double x, double y, double z) {
  return Num(x) + " " + Num(y) + " " + Num(z);
}

std::string Vec3(const Eigen::Vector3d& v) { return Vec3(v.x(), v.y(), v.z()); }

std::string Vec3(const std::array<double, 3>& v) { return Vec3(v[0], v[1], v[2]); }

// A fixed number of decimals: masses have six, the tube's three, and the
// frame's place in the world three.
std::string Fixed(double value, int decimals = 6) {
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

// A mass as it is written: rounded to six decimals.
double RoundMass(double kilograms) { return std::round(kilograms * 1e6) / 1e6; }

// A quaternion w x y z of a rotation, with the sign that makes the first
// component that is not zero positive.
std::string Quat(const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond q(rotation);
  q.normalize();
  double c[4] = {q.w(), q.x(), q.y(), q.z()};
  for (double v : c) {
    if (std::fabs(v) < 1e-9) continue;
    if (v < 0.0) {
      for (double& w : c) w = -w;
    }
    break;
  }
  return Num(c[0]) + " " + Num(c[1]) + " " + Num(c[2]) + " " + Num(c[3]);
}

// A box geom by its centre and half sizes, in the frame of its body.
struct Box {
  std::string suffix;
  Eigen::Vector3d pos;
  Eigen::Vector3d half;
  double Volume() const { return 8.0 * half.x() * half.y() * half.z(); }
};

// The open strap, ours: each half of the strap swung outward about its pin
// until it lies level under the rail. A half is half the strap's developed
// length, 348 mm (docs/bay-hardware.md, Rail, "Mass of rail, strap, pins and
// bolt"), and it is drawn 20 mm thick: the strap's 2.5 mm with its ear, which
// is 20 mm across.
constexpr double kStrapOpenLength = 0.348 / 2.0;
constexpr double kStrapOpenThickness = 0.020;

// The boxes of a rail with its strap, in the rail's frame (origin on the
// tube's axis, the rail's long side along X), in the order of the file. With
// `strap_open` the strap is drawn open (see RailBodyXml in the header).
std::vector<Box> RailBoxes(bool strap_open) {
  const double half_length = kRailLength / 2.0;
  const double half_top = kRailTopWidth / 2.0;
  const double half_sheet = kRailSheet / 2.0;
  // The top face and the underside of the top sheet.
  const double top_centre = kRailTopHeight - half_sheet;
  const double under_top = kRailTopHeight - kRailSheet;
  const double bottom = kRailTopHeight - kRailDepth;
  // The webs stand just inside the top's edge; the feet reach out from them.
  const double web_y = half_top - half_sheet;
  const double foot_y = half_top + kRailFootWidth / 2.0;
  // The middle part of a web ends on top of the tube; the outer parts reach
  // down to the feet.
  const double mid_bottom = kTubeRadius;
  const double outer_half = (half_length - kSaddleHalfWidth) / 2.0;
  const double outer_x = kSaddleHalfWidth + outer_half;
  std::vector<Box> boxes;
  boxes.push_back({"_top", {0, 0, top_centre}, {half_length, half_top, half_sheet}});
  for (const auto& [side, sign] : {std::pair{"a", -1.0}, std::pair{"b", 1.0}}) {
    const std::string web = std::string("_web_") + side;
    boxes.push_back({web + "_mid",
                     {0, sign * web_y, (mid_bottom + under_top) / 2.0},
                     {kSaddleHalfWidth, half_sheet, (under_top - mid_bottom) / 2.0}});
    for (const auto& [end, x_sign] : {std::pair{"neg", -1.0}, std::pair{"pos", 1.0}}) {
      boxes.push_back({web + "_" + end,
                       {x_sign * outer_x, sign * web_y, (bottom + under_top) / 2.0},
                       {outer_half, half_sheet, (under_top - bottom) / 2.0}});
    }
  }
  for (const auto& [side, sign] : {std::pair{"a", -1.0}, std::pair{"b", 1.0}}) {
    for (const auto& [end, x_sign] : {std::pair{"neg", -1.0}, std::pair{"pos", 1.0}}) {
      boxes.push_back({std::string("_foot_") + side + "_" + end,
                       {x_sign * outer_x, sign * foot_y, bottom + half_sheet},
                       {outer_half, kRailFootWidth / 2.0, half_sheet}});
    }
  }
  // The strap: a square U under the tube, its inside against the tube.
  const double half_strap = kStrapThickness / 2.0;
  const double strap_outer = kTubeRadius + kStrapThickness;
  const double side_top = bottom;
  const double side_bottom = -strap_outer;
  const double strap_y = kStrapWidth / 2.0;
  if (strap_open) {
    // Each half from |x| = kTubeRadius outward, its top where the closed
    // strap's sides end at the webs' lower edges.
    for (const auto& [end, x_sign] : {std::pair{"neg", -1.0}, std::pair{"pos", 1.0}}) {
      boxes.push_back({std::string("_strap_open_") + end,
                       {x_sign * (kTubeRadius + kStrapOpenLength / 2.0), 0,
                        side_top - kStrapOpenThickness / 2.0},
                       {kStrapOpenLength / 2.0, strap_y, kStrapOpenThickness / 2.0}});
    }
    return boxes;
  }
  for (const auto& [end, x_sign] : {std::pair{"neg", -1.0}, std::pair{"pos", 1.0}}) {
    boxes.push_back({std::string("_strap_") + end,
                     {x_sign * (kTubeRadius + half_strap), 0, (side_top + side_bottom) / 2.0},
                     {half_strap, strap_y, (side_top - side_bottom) / 2.0}});
  }
  boxes.push_back({"_strap_bottom",
                   {0, 0, -(kTubeRadius + half_strap)},
                   {strap_outer, strap_y, half_strap}});
  boxes.push_back({"_strap_ears",
                   {0, 0, (kStrapEarBottom + -strap_outer) / 2.0},
                   {kStrapEarHalfWidth, strap_y, (-strap_outer - kStrapEarBottom) / 2.0}});
  return boxes;
}

// The boxes of a module, in the module's frame (origin at its centre), in the
// order of the file, and for each the class that colours it.
struct ModuleBox {
  Box box;
  const char* cls;
  bool laminate;
};

std::vector<ModuleBox> ModuleBoxes() {
  const double half_long = kModuleLength / 2.0;
  const double half_short = kModuleWidth / 2.0;
  const double half_height = kModuleThickness / 2.0;
  const double half_upright = kFrameUprightWidth / 2.0;
  const double flange_z = -half_height + kFlangeThickness / 2.0;
  std::vector<ModuleBox> boxes;
  const std::pair<const char*, double> sides[] = {{"a", -1.0}, {"b", 1.0}};
  for (const auto& [side, sign] : sides) {
    boxes.push_back({{std::string("_frame_long_") + side,
                      {0, sign * (half_short - half_upright), 0},
                      {half_long, half_upright, half_height}},
                     kAluminium, false});
  }
  for (const auto& [side, sign] : sides) {
    const double half_flange = kFlangeWidthLongSide / 2.0;
    boxes.push_back({{std::string("_flange_long_") + side,
                      {0, sign * (half_short - half_flange), flange_z},
                      {half_long, half_flange, kFlangeThickness / 2.0}},
                     kAluminium, false});
  }
  for (const auto& [side, sign] : sides) {
    boxes.push_back({{std::string("_frame_short_") + side,
                      {sign * (half_long - half_upright), 0, 0},
                      {half_upright, half_short, half_height}},
                     kAluminium, false});
  }
  for (const auto& [side, sign] : sides) {
    const double half_flange = kFlangeWidthShortSide / 2.0;
    boxes.push_back({{std::string("_flange_short_") + side,
                      {sign * (half_long - half_flange), 0, flange_z},
                      {half_flange, half_short, kFlangeThickness / 2.0}},
                     kAluminium, false});
  }
  boxes.push_back({{"_laminate",
                    {0, 0, half_height - kLaminateThickness / 2.0},
                    {half_long - half_upright, half_short - half_upright,
                     kLaminateThickness / 2.0}},
                   kLaminate, true});
  return boxes;
}

std::string GeomLine(const std::string& name, const char* cls, const Box& box,
                     double mass) {
  return "        <geom name=\"" + name + box.suffix + "\" class=\"" + cls +
         "\" pos=\"" + Vec3(box.pos) + "\" size=\"" + Vec3(box.half) +
         "\" mass=\"" + Fixed(mass) + "\"/>\n";
}

std::string SiteLine(const std::string& name, const char* cls,
                     const Eigen::Vector3d& pos, const Eigen::Matrix3d& frame) {
  return "        <site name=\"" + name + "\" class=\"" + cls + "\" pos=\"" +
         Vec3(pos) + "\" quat=\"" + Quat(frame) + "\"/>\n";
}

std::string BodyOpen(const std::string& name, const std::array<double, 3>& pos) {
  return "      <body name=\"" + name + "\" pos=\"" + Vec3(pos) + "\">\n";
}

const char* kBodyClose = "      </body>\n";

std::string Words(int n, bool capital) {
  static const char* const kWords[] = {
      "zero",    "one",     "two",       "three",    "four",
      "five",    "six",     "seven",     "eight",    "nine",
      "ten",     "eleven",  "twelve",    "thirteen", "fourteen",
      "fifteen", "sixteen", "seventeen", "eighteen", "nineteen",
      "twenty"};
  std::string word = (n >= 0 && n <= 20) ? kWords[n] : std::to_string(n);
  if (capital && !word.empty()) word[0] = static_cast<char>(std::toupper(word[0]));
  return word;
}

std::string Replace(std::string text, const std::string& token,
                    const std::string& value) {
  const std::string key = "@" + token + "@";
  for (size_t at = text.find(key); at != std::string::npos;
       at = text.find(key, at + value.size())) {
    text.replace(at, key.size(), value);
  }
  return text;
}

// The long comment, written for the finished bay. The numbers are the
// header's, spelt out as words where the text spells them.
std::string OpeningComment(const BayModelOptions& options) {
  std::string text = R"BAYCOMMENT(<!--
  One bay of a single-axis solar tracker as it really is: a round torque tube
  section, the @rails@ rails across it, the @modules@ modules on the rails, and a
  site at every point where one part is fastened to another.

  The tube is a round steel tube @tube_mm@ mm across and @tube_length@ m long, with a neck
  @neck_length@ m long and @neck_mm@ mm across at its swaged end. @Rails@ rails sit across it,
  @pitch@ m apart, the first @first@ m from the plain end. A rail is a hat
  section, open side down, whose webs straddle the tube, with a strap under
  the tube that is closed by one bolt through the strap's two ears. @Modules@
  modules lie between the rails, glass up, long side across the tube, each
  with its two long edges on two rails, so that neighbouring modules share a
  rail and are @gap_mm@ mm apart. docs/bay-hardware.md says where each figure comes
  from.

  The frame (body "bay_frame") is the tube's frame of src/bay_hardware.h: the
  origin is on the tube's axis at the plain end of the section, +Y runs
  along the tube to the swaged end, +Z points up through the locating holes
  and +X is across the tube. Metres, kilograms, radians. This file places
  that frame at (@fx@, @fy@, @fz@) in the world, @height@ m above the floor;
  where it stands is the scene's choice and not a figure of the bay. The
  body "bay_frame" has no geoms of its own. Its children are bay_tube,
  bay_rail_0 .. bay_rail_@last_rail@ and bay_module_0 .. bay_module_@last_module@, each fixed in
  its place: there is no joint, no actuator and no weld, because this is
  the finished bay. Every geom collides.

  The sites. Every name in this file starts with "bay_"; the prefix is left
  out below. A fastening joins two parts and has two sites, one at the
  centre of each part's hole: <name>_a on the part nearer the tool and
  <name>_b on the other, each with its z axis along the way the tool
  travels. fasten_rail_<k>_0 is rail k's strap bolt: its _a site is on the
  rail at the mouth of the ear's hole on the +X side, its _b site on the
  tube under the tube's axis, and the tool travels toward -X.
  fasten_module_<m>_<i> is pin i of module m: its _a site is on the rail in
  the middle of the rail's top sheet, its _b site on the module in the
  middle of the frame's flange, and the tool travels up. locate_<k>_a is
  rail k's locating tab and locate_<k>_b the hole on top of the tube it
  drops into; their z axis points down. hold_0 .. hold_@last_hold@ are the stretches
  of the tube that a holder may grip, given as see-through cylinders along
  the tube: the plain end, the @stretches@ stretches between neighbouring rails, and
  the neck. Fastening sites are orange (_a) and green (_b), locators
  magenta.

  What is simplified. The tube is a solid cylinder; its wall is not drawn.
  A rail's web is cut to an arc where it sits on the tube; here each web is
  three boxes, a middle one that ends on top of the tube and two outer ones
  that reach down to the feet, so the saddle is a step. The strap is a
  square U of three boxes round the lower half of the tube, with one block
  for both ears, and the bolt, the tab, the pins and the holes are not drawn:
  the sites stand for them. A module is a laminate on top of a frame of four
  upright members and four bottom flanges; the junction box, the cables and
  the corners' shape are left out. Each geom has a mass proportional to its
  volume, so that a rail weighs @rail_mass@ kg and a module @module_mass@ kg, of which the
  laminate is @laminate_mass@ kg.

  src/bay_hardware.h holds the same numbers for code, and
  tests/bay_hardware_test.cc checks that this model and the header agree.
@provenance@
  Part file: loaded only through a scene that includes common.xml first.
-->
)BAYCOMMENT";
  const std::pair<const char*, std::string> tokens[] = {
      {"rails", Words(kRailCount, false)},
      {"Rails", Words(kRailCount, true)},
      {"modules", Words(kModuleCount, false)},
      {"Modules", Words(kModuleCount, true)},
      {"tube_mm", Num(2.0 * kTubeRadius * 1000.0)},
      {"tube_length", Num(kTubeLength)},
      {"neck_length", Num(kNeckLength)},
      {"neck_mm", Num(2.0 * kNeckRadius * 1000.0)},
      {"pitch", Num(kModulePitch)},
      {"first", Num(kFirstRailY)},
      {"gap_mm", Num(kModuleGap * 1000.0)},
      {"fx", Num(options.frame_pos[0])},
      {"fy", Fixed(options.frame_pos[1], 3)},
      {"fz", Fixed(options.frame_pos[2], 3)},
      {"height", Num(options.frame_pos[2])},
      {"last_rail", std::to_string(kRailCount - 1)},
      {"last_module", std::to_string(kModuleCount - 1)},
      {"last_hold", std::to_string(kHoldCount - 1)},
      {"stretches", Words(kRailCount - 1, false)},
      {"rail_mass", Num(kRailMass)},
      {"module_mass", Num(kModuleMass)},
      {"laminate_mass", Num(kLaminateMass)},
      {"provenance", kProvenance},
  };
  for (const auto& [token, value] : tokens) text = Replace(text, token, value);
  return text;
}

std::string Defaults() {
  std::string text;
  text += "  <default>\n";
  text += "    <!-- Galvanised steel: the tube, the rails and their straps. -->\n";
  text += std::string("    <default class=\"") + kSteel + "\">\n";
  text += std::string("      <geom type=\"box\" rgba=\"") + kSteelRgba + "\"/>\n";
  text += "    </default>\n";
  text += "    <!-- The modules' aluminium frames. -->\n";
  text += std::string("    <default class=\"") + kAluminium + "\">\n";
  text += std::string("      <geom type=\"box\" rgba=\"") + kAluminiumRgba + "\"/>\n";
  text += "    </default>\n";
  text += "    <!-- The laminate: glass over dark blue cells. -->\n";
  text += std::string("    <default class=\"") + kLaminate + "\">\n";
  text += std::string("      <geom type=\"box\" rgba=\"") + kLaminateRgba + "\"/>\n";
  text += "    </default>\n";
  text +=
      "    <!-- Fastening points: orange on the part nearer the tool (a), green on\n"
      "         the other (b). -->\n";
  const std::pair<const char*, const char*> site_classes[] = {
      {kSiteA, kSiteARgba}, {kSiteB, kSiteBRgba}};
  for (const auto& [cls, rgba] : site_classes) {
    text += std::string("    <default class=\"") + cls + "\">\n";
    text += std::string("      <site type=\"sphere\" size=\"") + kSiteSize +
            "\" rgba=\"" + rgba + "\"/>\n";
    text += "    </default>\n";
  }
  text += "    <!-- Locating tabs and their holes. -->\n";
  text += std::string("    <default class=\"") + kSiteLocate + "\">\n";
  text += std::string("      <site type=\"sphere\" size=\"") + kSiteSize +
          "\" rgba=\"" + kSiteLocateRgba + "\"/>\n";
  text += "    </default>\n";
  text += "    <!-- Stretches of the tube a holder may grip, see-through. -->\n";
  text += std::string("    <default class=\"") + kSiteHold + "\">\n";
  text += std::string("      <site type=\"cylinder\" group=\"") + kSiteHoldGroup +
          "\" rgba=\"" + kSiteHoldRgba + "\"/>\n";
  text += "    </default>\n";
  text += "  </default>\n";
  return text;
}

void CheckRange(const char* what, int value, int most) {
  if (value < 0 || value > most) {
    throw std::invalid_argument(std::string("BayModelOptions::") + what +
                                " must be 0 to " + std::to_string(most));
  }
}

}  // namespace

std::string TubeBodyXml() {
  std::string text = BodyOpen(TubeName(), {0, 0, 0});
  const double neck_start = kTubeLength;
  text += "        <geom name=\"" + TubeName() + "\" class=\"" + kSteel +
          "\" type=\"cylinder\" size=\"" + Num(kTubeRadius) + "\" fromto=\"" +
          Vec3(0, 0, 0) + " " + Vec3(0, neck_start, 0) + "\" mass=\"" +
          Fixed(kTubeMassPerMetre * kTubeLength, 3) + "\"/>\n";
  text += "        <geom name=\"" + TubeName() + "_neck\" class=\"" + kSteel +
          "\" type=\"cylinder\" size=\"" + Num(kNeckRadius) + "\" fromto=\"" +
          Vec3(0, neck_start, 0) + " " + Vec3(0, kSectionLength, 0) +
          "\" mass=\"" + Fixed(kNeckMassPerMetre * kNeckLength, 3) + "\"/>\n";
  text += "        <!-- Where each strap's bolt passes under the tube. -->\n";
  for (const Fastening& f : Fastenings()) {
    if (f.kind != FasteningKind::kRail) continue;
    text += SiteLine(f.name + "_b", kSiteB, f.pose_b.translation(),
                     f.pose_b.linear());
  }
  text += "        <!-- The locating holes on top of the tube. -->\n";
  for (const Locator& l : Locators()) {
    text += SiteLine(l.name + "_b", kSiteLocate, l.pose_b.translation(),
                     l.pose_b.linear());
  }
  text += "        <!-- The stretches a holder may grip. -->\n";
  for (const Hold& h : Holds()) {
    text += "        <site name=\"" + h.name + "\" class=\"" + kSiteHold +
            "\" size=\"" + Num(h.radius) + " " + Num(h.half_length) +
            "\" pos=\"" + Vec3(h.pose.translation()) + "\" quat=\"" +
            Quat(h.pose.linear()) + "\"/>\n";
  }
  text += kBodyClose;
  return text;
}

std::string RailBodyXml(const std::string& name,
                        const std::array<double, 3>& pos, int index,
                        bool strap_open) {
  const std::vector<Box> boxes = RailBoxes(strap_open);
  double volume = 0.0;
  for (const Box& box : boxes) volume += box.Volume();
  std::string text = BodyOpen(name, pos);
  // Each geom's mass is rounded to the six decimals it is written with, and
  // the last takes what is left, so that the printed masses add up to the
  // rail's.
  double rounded_sum = 0.0;
  for (size_t i = 0; i < boxes.size(); ++i) {
    double mass = kRailMass * boxes[i].Volume() / volume;
    mass = (i + 1 == boxes.size()) ? kRailMass - rounded_sum : RoundMass(mass);
    rounded_sum += RoundMass(mass);
    text += GeomLine(name, kSteel, boxes[i], mass);
  }
  if (index >= 0) {
    const Eigen::Vector3d origin = RailPose(index).translation();
    for (const Fastening& f : Fastenings()) {
      if (f.part_a != RailName(index)) continue;
      text += SiteLine(f.name + "_a", kSiteA, f.pose_a.translation() - origin,
                       f.pose_a.linear());
      if (f.kind == FasteningKind::kRail) {
        // The locator's tab comes right after the strap bolt.
        for (const Locator& l : Locators()) {
          if (l.part_a != RailName(index)) continue;
          text += SiteLine(l.name + "_a", kSiteLocate,
                           l.pose_a.translation() - origin, l.pose_a.linear());
        }
      }
    }
  }
  text += kBodyClose;
  return text;
}

std::string ModuleBodyXml(const std::string& name,
                          const std::array<double, 3>& pos, int index) {
  const std::vector<ModuleBox> boxes = ModuleBoxes();
  double frame_volume = 0.0;
  for (const ModuleBox& m : boxes) {
    if (!m.laminate) frame_volume += m.box.Volume();
  }
  std::string text = BodyOpen(name, pos);
  for (const ModuleBox& m : boxes) {
    const double mass = m.laminate
                            ? kLaminateMass
                            : kFrameMass * m.box.Volume() / frame_volume;
    text += GeomLine(name, m.cls, m.box, mass);
  }
  if (index >= 0) {
    const Eigen::Vector3d origin = ModulePose(index).translation();
    for (const Fastening& f : Fastenings()) {
      if (f.part_b != ModuleName(index)) continue;
      text += SiteLine(f.name + "_b", kSiteB, f.pose_b.translation() - origin,
                       f.pose_b.linear());
    }
  }
  text += kBodyClose;
  return text;
}

std::string BayModelXml(const BayModelOptions& options) {
  CheckRange("rails", options.rails, kRailCount);
  CheckRange("modules", options.modules, kModuleCount);
  std::string text;
  if (options.comment) text += OpeningComment(options) + "\n";
  text += "<mujoco>\n";
  text += Defaults();
  text += "\n  <worldbody>\n";
  text += std::string("    <body name=\"") + kFrameName + "\" pos=\"" +
          Fixed(options.frame_pos[0], 3) + " " + Fixed(options.frame_pos[1], 3) +
          " " + Fixed(options.frame_pos[2], 3) + "\">\n";
  text += "      <!-- The tube: from the plain end to the shoulder, then the neck. -->\n";
  text += TubeBodyXml();
  for (int rail = 0; rail < options.rails; ++rail) {
    const Eigen::Vector3d at = RailPose(rail).translation();
    text += "\n      <!-- Rail " + std::to_string(rail) + ". -->\n";
    text += RailBodyXml(RailName(rail), {at.x(), at.y(), at.z()}, rail);
  }
  for (int module = 0; module < options.modules; ++module) {
    const Eigen::Vector3d at = ModulePose(module).translation();
    text += "\n      <!-- Module " + std::to_string(module) + ", on rails " +
            std::to_string(module) + " and " + std::to_string(module + 1) +
            ". -->\n";
    text += ModuleBodyXml(ModuleName(module), {at.x(), at.y(), at.z()}, module);
  }
  text += "    </body>\n  </worldbody>\n</mujoco>\n";
  return text;
}

}  // namespace cell::bay
