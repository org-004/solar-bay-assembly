// Builds a layout's MuJoCo model. See src/layout_model.h.
//
// How the model is put together. The top-level text that BuildLayoutXml
// returns includes common.xml and container.xml, holds the holder, the bay,
// the pallet, the stocks, the items and each arm's track as plain bodies, and
// attaches each arm with
//   <asset><model name="<arm>_model" file="<arm>.arm.xml"/></asset>
//   <body name="<arm>_carriage" ...><attach model="<arm>_model"
//       body="<root_body>" prefix="<arm>_"/></body>
// The file <arm>.arm.xml is made at run time, one for each arm of the layout,
// from the arm's model file (ArmPartXml below): models/arm.xml for the KUKA,
// Menagerie's ur10e.xml for the UR10e. In it:
//   - the KUKA's <include file="tool.xml"/> (the stand-in cell's suction
//     tool, under arm_flange) is replaced by this layout's tools, and the
//     part file gets the compiler settings it leaves to common.xml
//     (angle="radian" autolimits="true");
//   - the UR10e gets a body that carries the tools, turned as its
//     attachment_site is but 17 mm further out, on the face of the flange
//     (see "The flange" below), and its base a collision cylinder of its own
//     (Menagerie's base has visual meshes only: radius 0.095, 0 .. 0.032
//     above the base frame, below the shoulder's capsule);
//   - mesh paths are made absolute (the KUKA's file= attributes, the UR10e's
//     meshdir), so that the meshes are read from where Bazel put them,
//     whatever directory the top-level model is in;
//   - <actuator> and <keyframe> are removed: the model is for looking and
//     measuring, its poses are set in qpos, and the one keyframe is the
//     state's (see the end of BuildLayoutXml).
// LoadLayoutModel hands MuJoCo the top-level text, the arms' part files and
// common.xml and container.xml through an mjVFS, so nothing is written to
// disk (Bazel's runfiles are read-only); the meshes, named by absolute path,
// are not in the VFS and MuJoCo reads them from the file system. The text
// that BuildLayoutXml returns is therefore loadable only together with those
// files, as LoadLayoutModel does.
//
// The flange. Each arm's tools hang from a body "<arm>_tool_mount" whose
// frame is the tool frame of src/layout.h: origin at the flange's centre, z
// out of the flange. For the KUKA it is the body arm_flange itself (the URDF's
// tool0: "+Z pointing out of the flange", models/arm.xml). For the UR10e its
// axes are those of Menagerie's attachment_site, whose z is wrist_3_link's
// +y, the way from the wrist's axis out through the flange; but the site
// stands at y = 0.100, 17 mm inside the arm: wrist_3's collision cylinder
// reaches y = 0.117, and the datasheet's d6 (wrist 2's axis to the flange
// face) is 0.1166. So the mount stands at y = 0.117, on the collision
// geom's face; a tool there no longer reaches into the wrist. The test checks
// both by forward kinematics (tests/layout_model_test.cc, FlangeZPointsOut):
// z lies along the last joint's axis, the flange's centre lies beyond that
// joint along z, and the last link's collision geoms lie behind the flange's
// face.
//
// The base frame. An arm's base frame (base_pos, base_rpy_deg; the frame
// waits_at and the carriage box are given in) is the frame the arm's model is
// attached in: the arm model's root body keeps its own pose in it (identity
// for the KUKA's arm_base, a half turn about z for Menagerie's base).

#include "src/layout_model.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

#include <Eigen/Geometry>

#include "src/arm_catalog.h"
#include "src/bay_model.h"

namespace cell::layout {
namespace {

constexpr double kPi = 3.14159265358979323846;

// The pallet's deck reaches this far beyond the modules' footprint on every
// side. Ours.
constexpr double kPalletDeckMargin = 0.02;

std::string Num(double value) {
  if (std::fabs(value) < 5e-13) return "0";
  char buffer[40];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  return buffer;
}

std::string V3(const Eigen::Vector3d& v) {
  return Num(v.x()) + " " + Num(v.y()) + " " + Num(v.z());
}
std::string V3(const Vec3& v) { return V3(Eigen::Vector3d(v[0], v[1], v[2])); }
std::string Half(const Vec3& v) {
  return V3(Eigen::Vector3d(v[0] / 2.0, v[1] / 2.0, v[2] / 2.0));
}

std::string Quat(const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond q(rotation);
  q.normalize();
  return Num(q.w()) + " " + Num(q.x()) + " " + Num(q.y()) + " " + Num(q.z());
}

// Roll about x, then pitch about y, then yaw about z, all about the fixed
// axes: R = Rz(yaw) Ry(pitch) Rx(roll).
Eigen::Matrix3d Rpy(const Vec3& rpy_deg) {
  const double d = kPi / 180.0;
  return (Eigen::AngleAxisd(rpy_deg[2] * d, Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(rpy_deg[1] * d, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(rpy_deg[0] * d, Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

bool ReadFile(const std::string& path, std::string* text) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  std::stringstream s;
  s << in.rdbuf();
  *text = s.str();
  return true;
}

std::string Absolute(const std::string& path) {
  return std::filesystem::absolute(path).lexically_normal().string();
}

// Removes the first <tag ...> ... </tag> from the text (whole lines).
void RemoveElement(std::string* text, const std::string& tag) {
  const size_t open = text->find("<" + tag + ">");
  if (open == std::string::npos) return;
  const std::string close_tag = "</" + tag + ">";
  const size_t close = text->find(close_tag, open);
  if (close == std::string::npos) return;
  text->erase(open, close + close_tag.size() - open);
}

bool ReplaceOnce(std::string* text, const std::string& from,
                 const std::string& to) {
  const size_t at = text->find(from);
  if (at == std::string::npos) return false;
  text->replace(at, from.size(), to);
  return true;
}

void ReplaceAll(std::string* text, const std::string& from,
                const std::string& to) {
  for (size_t at = text->find(from); at != std::string::npos;
       at = text->find(from, at + to.size())) {
    text->replace(at, from.size(), to);
  }
}

// The colours of the tools, in the order of an arm's tools.
const char* const kToolRgb[] = {"0.15 0.70 0.30", "0.75 0.25 0.70",
                                "0.15 0.65 0.80", "0.90 0.35 0.25",
                                "0.55 0.55 0.15"};
constexpr int kToolColours = 5;

// The default classes the layout's own geoms use. `prefix` makes them
// unique in a file that is attached (an arm's part file).
std::string LayoutDefaults(const std::string& prefix) {
  auto cls = [&](const std::string& name, const std::string& body) {
    return "    <default class=\"" + prefix + name + "\">" + body +
           "</default>\n";
  };
  std::string t = "  <default>\n";
  t += cls("layout_holder", "<geom type=\"box\" rgba=\"0.25 0.35 0.60 1\"/>");
  t += cls("layout_beam", "<geom type=\"box\" rgba=\"0.30 0.30 0.33 1\"/>");
  t += cls("layout_carriage", "<geom type=\"box\" rgba=\"0.90 0.75 0.10 1\"/>");
  t += cls("layout_pallet", "<geom type=\"box\" rgba=\"0.62 0.45 0.25 1\"/>");
  t += cls("layout_stock", "<geom type=\"box\" rgba=\"0.55 0.22 0.15 1\"/>");
  t += cls("layout_item", "<geom type=\"box\" rgba=\"0.55 0.55 0.60 0.4\"/>");
  for (int i = 0; i < kToolColours; ++i) {
    t += cls("layout_tool_" + std::to_string(i),
             std::string("<geom type=\"box\" group=\"0\" rgba=\"") + kToolRgb[i] +
                 " 1\"/>");
    t += cls("layout_wait_" + std::to_string(i),
             std::string("<geom type=\"box\" group=\"0\" rgba=\"") + kToolRgb[i] +
                 " 0.45\"/>");
  }
  t += cls("layout_site",
           "<site type=\"sphere\" size=\"0.01\" group=\"4\" rgba=\"1 1 0 1\"/>");
  t += cls("layout_probe",
           "<geom type=\"cylinder\" contype=\"0\" conaffinity=\"0\" group=\"1\" "
           "rgba=\"1 0.1 0.1 0.3\"/>");
  t += "  </default>\n";
  return t;
}

// The <default> block of the bay's classes, as src/bay_model.cc writes it.
std::string BayDefaults() {
  bay::BayModelOptions options;
  options.rails = 0;
  options.modules = 0;
  options.comment = false;
  const std::string text = bay::BayModelXml(options);
  const size_t open = text.find("  <default>\n");
  const std::string close = "\n  </default>\n";
  const size_t end = text.find(close, open);
  return text.substr(open, end + close.size() - open);
}

// Geoms of boxes, each of class `cls`, named <prefix><box name>, with the
// mass shared by volume.
std::string BoxGeoms(const std::vector<Box>& boxes, const std::string& prefix,
                     const std::string& cls, double mass) {
  double volume = 0.0;
  for (const Box& b : boxes) volume += b.size[0] * b.size[1] * b.size[2];
  std::string t;
  for (const Box& b : boxes) {
    const double v = b.size[0] * b.size[1] * b.size[2];
    const double m = volume > 0.0 ? mass * v / volume
                                  : mass / static_cast<double>(boxes.size());
    t += "      <geom name=\"" + prefix + b.name + "\" class=\"" + cls +
         "\" pos=\"" + V3(b.pos) + "\" size=\"" + Half(b.size) + "\" mass=\"" +
         Num(m) + "\"/>\n";
  }
  return t;
}

// What one arm has on its flange and where it stands, from the layout and
// the state.
struct ArmSetup {
  std::vector<std::string> on_flange;  // Tool names.
  std::vector<std::string> waiting;    // Tool names with a box where they wait.
  std::vector<std::string> held;       // "module", "rail".
  std::array<double, 6> joints_deg{};
  double track = 0.0;
};

// The tool of the arm that does a kind of work, or "" if another arm does it.
std::string ToolFor(const Layout& layout, const ArmUse& arm,
                    const std::string& kind) {
  const Work* w = FindWork(layout, kind);
  if (w == nullptr || w->arm != arm.name) return "";
  return w->tool;
}

std::string HoldingTool(const Layout& layout, const ArmUse& arm,
                        const std::string& part) {
  return ToolFor(layout, arm, part == "module" ? "module_set" : "rail_set");
}

bool Setup(const Layout& layout, const ArmUse& arm, const LayoutState& state,
           ArmSetup* setup, std::string* error) {
  auto fail = [&](const std::string& what) {
    if (error != nullptr) *error = "arm " + arm.name + ": " + what;
    return false;
  };
  std::string tool = state.stowed ? arm.stow_tool : arm.first_tool;
  if (!state.stowed) {
    if (auto it = state.tool.find(arm.name); it != state.tool.end()) tool = it->second;
  }
  if (!tool.empty() && FindTool(arm, tool) == nullptr) {
    return fail("no tool \"" + tool + "\"");
  }
  for (const Tool& t : arm.tools) {
    const bool has_wait_box =
        t.wait_size[0] > 0.0 && t.wait_size[1] > 0.0 && t.wait_size[2] > 0.0;
    if (state.everything) {
      setup->on_flange.push_back(t.name);
      if (has_wait_box) setup->waiting.push_back(t.name);
    } else if (t.name == tool) {
      setup->on_flange.push_back(t.name);
    } else if (has_wait_box) {
      setup->waiting.push_back(t.name);
    }
  }
  if (state.everything) {
    for (const char* part : {"module", "rail"}) {
      if (!HoldingTool(layout, arm, part).empty()) setup->held.push_back(part);
    }
  } else if (!state.stowed) {
    if (auto it = state.held.find(arm.name);
        it != state.held.end() && !it->second.empty()) {
      if (it->second != "module" && it->second != "rail") {
        return fail("cannot hold \"" + it->second + "\"");
      }
      if (HoldingTool(layout, arm, it->second).empty()) {
        return fail("does not hold a " + it->second + " in this layout");
      }
      setup->held.push_back(it->second);
    }
  }
  setup->joints_deg = state.stowed ? arm.stow_joints_deg : arm.wait_joints_deg;
  setup->track = state.stowed ? arm.stow_track : arm.wait_track;
  if (!state.stowed) {
    if (auto it = state.joints_deg.find(arm.name); it != state.joints_deg.end()) {
      setup->joints_deg = it->second;
    }
    if (auto it = state.track.find(arm.name); it != state.track.end()) {
      setup->track = it->second;
    }
  }
  if (!arm.track) setup->track = 0.0;
  return true;
}

int ToolIndex(const ArmUse& arm, const std::string& name) {
  for (size_t i = 0; i < arm.tools.size(); ++i) {
    if (arm.tools[i].name == name) return static_cast<int>(i);
  }
  return 0;
}

// The tool mount's children: the flange site, a work site for every tool,
// the tools on the flange and the parts held. Names are the arm's part
// file's, which the attach prefixes with "<arm>_". `cls` is the prefix of
// the part file's own classes.
std::string ToolMountContents(const Layout& layout, const ArmUse& arm,
                              const ArmSetup& setup, const std::string& cls) {
  std::string t;
  t += "<site name=\"flange_site\" class=\"" + cls + "layout_site\"/>\n";
  for (const Tool& tool : arm.tools) {
    t += "<site name=\"work_" + tool.name + "\" class=\"" + cls +
         "layout_site\" pos=\"0 0 " + Num(tool.length) + "\"/>\n";
  }
  for (const std::string& name : setup.on_flange) {
    const Tool& tool = *FindTool(arm, name);
    const int colour = ToolIndex(arm, name) % kToolColours;
    t += BoxGeoms(tool.boxes, "tool_" + name + "_",
                  cls + "layout_tool_" + std::to_string(colour), tool.mass_kg);
  }
  for (const std::string& part : setup.held) {
    const Tool& tool = *FindTool(arm, HoldingTool(layout, arm, part));
    // The part's top face at the working point, its up along the tool's -z
    // (a half turn about the tool's x), its long side along the tool's x.
    t += "<body name=\"held_" + part + "_mount\" pos=\"0 0 " + Num(tool.length) +
         "\" quat=\"0 1 0 0\">\n";
    std::string body;
    if (part == "module") {
      body = bay::ModuleBodyXml("held_module",
                                {0.0, 0.0, -bay::kModuleThickness / 2.0});
    } else {
      body = bay::RailBodyXml("held_rail", {0.0, 0.0, -bay::kRailTopHeight});
    }
    ReplaceAll(&body, "class=\"bay_", "class=\"" + cls + "bay_");
    t += body;
    t += "</body>\n";
  }
  return t;
}

// The arm's part file: its model file as described at the top.
bool ArmPartXml(const Layout& layout, const ArmUse& arm, const ArmModel& model,
                const ArmSetup& setup, const ModelPaths& paths,
                std::string* text, std::string* error) {
  auto fail = [&](const std::string& what) {
    if (error != nullptr) *error = "arm " + arm.name + ": " + what;
    return false;
  };
  // The part file's own classes carry the arm's name, so that they cannot
  // clash with the top level's or another arm's.
  const std::string cls = arm.name + "_part_";
  std::string defaults = LayoutDefaults(cls);
  std::string bay_defaults = BayDefaults();
  ReplaceAll(&bay_defaults, "class=\"bay_", "class=\"" + cls + "bay_");
  defaults += bay_defaults;
  const std::string mount = ToolMountContents(layout, arm, setup, cls);
  if (model.source == "kuka") {
    const std::string file = paths.models_dir + "/arm.xml";
    if (!ReadFile(file, text)) return fail("cannot read " + file);
    if (!ReplaceOnce(text, "<include file=\"tool.xml\"/>",
                     "<body name=\"tool_mount\">\n" + mount + "</body>")) {
      return fail(file + " has no <include file=\"tool.xml\"/> at the flange");
    }
    ReplaceAll(text, "file=\"kuka_kr210_r2700_2/",
               "file=\"" + Absolute(paths.models_dir) + "/kuka_kr210_r2700_2/");
    if (!ReplaceOnce(text, "<mujoco>",
                     "<mujoco model=\"" + arm.name +
                         "\">\n  <compiler angle=\"radian\" autolimits=\"true\"/>\n" +
                         defaults)) {
      return fail(file + " has no <mujoco>");
    }
  } else if (model.source == "menagerie_ur10e") {
    const std::string& file = paths.ur10e_xml;
    if (!ReadFile(file, text)) return fail("cannot read " + file);
    const std::string assets =
        Absolute((std::filesystem::path(file).parent_path() / "assets").string());
    if (!ReplaceOnce(text, "meshdir=\"assets\"", "meshdir=\"" + assets + "\"")) {
      return fail(file + " has no meshdir=\"assets\"");
    }
    const std::string site = "<site name=\"attachment_site\" pos=\"0 0.1 0\" quat=\"-1 1 0 0\"/>";
    if (!ReplaceOnce(text, site,
                     site + "\n<body name=\"tool_mount\" pos=\"0 0.117 0\" quat=\"-1 1 0 0\">\n" +
                         mount + "</body>")) {
      return fail(file + " has no attachment_site where it was");
    }
    const std::string base = "<body name=\"base\" quat=\"0 0 0 -1\" childclass=\"ur10e\">";
    if (!ReplaceOnce(text, base,
                     base +
                         "\n<geom name=\"base_collision\" class=\"collision\" "
                         "type=\"cylinder\" size=\"0.095 0.016\" pos=\"0 0 0.016\"/>")) {
      return fail(file + " has no base body where it was");
    }
    const size_t open = text->find("<mujoco");
    const size_t close = open == std::string::npos ? open : text->find('>', open);
    if (close == std::string::npos) return fail(file + " has no <mujoco>");
    text->insert(close + 1, "\n" + defaults);
  } else {
    return fail("no way to build the model \"" + model.source + "\"");
  }
  RemoveElement(text, "actuator");
  RemoveElement(text, "keyframe");
  return true;
}

std::string ArmPartName(const ArmUse& arm) { return arm.name + ".arm.xml"; }
std::string TopName(const Layout& layout) { return "layout_" + layout.name + ".xml"; }

bool HasTube(const LayoutState& state) {
  return state.everything || (!state.stowed && state.tube);
}

}  // namespace

ModelPaths ModelPaths::FromEnvironment() {
  ModelPaths paths;
  const char* models = std::getenv("MODELS_DIR");
  const char* ur10e = std::getenv("UR10E_XML");
  paths.models_dir = models != nullptr ? models : "models";
  paths.ur10e_xml = ur10e != nullptr ? ur10e : "";
  return paths;
}

std::string BuildLayoutXml(const Layout& layout, const LayoutState& state,
                           const ModelPaths&, std::string* error) {
  auto fail = [&](const std::string& what) {
    if (error != nullptr) *error = "layout " + layout.name + ": " + what;
    return std::string();
  };
  const bool inside_only = state.stowed && !state.everything;
  int rails = state.rails, modules = state.modules;
  int pallet_modules = state.pallet_modules;
  if (state.everything) {
    rails = bay::kRailCount;
    modules = bay::kModuleCount;
    pallet_modules = layout.pallet.modules;
  }
  if (rails < 0 || rails > bay::kRailCount) return fail("rails out of 0..10");
  if (modules < 0 || modules > bay::kModuleCount) return fail("modules out of 0..9");
  if (modules > 0 && rails < modules + 1) {
    return fail("module " + std::to_string(modules - 1) + " needs rail " +
                std::to_string(modules));
  }
  if (pallet_modules < 0 || pallet_modules > layout.pallet.modules) {
    return fail("pallet_modules out of 0.." + std::to_string(layout.pallet.modules));
  }

  std::string t;
  t += "<mujoco model=\"layout " + layout.name + "\">\n";
  t += "  <include file=\"common.xml\"/>\n";
  t += "  <include file=\"container.xml\"/>\n";
  t += "  <statistic center=\"0 0 1.5\" extent=\"8\"/>\n";
  t += LayoutDefaults("");
  t += BayDefaults();
  t += "  <asset>\n";
  for (const ArmUse& arm : layout.arms) {
    t += "    <model name=\"" + arm.name + "_model\" file=\"" + ArmPartName(arm) +
         "\"/>\n";
  }
  t += "  </asset>\n";
  t += "  <worldbody>\n";

  // The holder.
  if (!inside_only || layout.holder.travels_inside) {
    t += "    <body name=\"holder\">\n";
    t += BoxGeoms(layout.holder.boxes, "holder_", "layout_holder",
                  layout.holder.mass_kg);
    t += "    </body>\n";
  }

  // The bay as far as it is built.
  if (HasTube(state)) {
    t += "    <body name=\"bay_frame\" pos=\"" +
         V3(Eigen::Vector3d(layout.tube.axis_x, layout.tube.plain_end_y,
                            layout.tube.axis_z)) +
         "\">\n";
    t += bay::TubeBodyXml();
    for (int k = 0; k < rails; ++k) {
      const Eigen::Vector3d at = bay::RailPose(k).translation();
      t += bay::RailBodyXml(bay::RailName(k), {at.x(), at.y(), at.z()}, k);
    }
    for (int m = 0; m < modules; ++m) {
      const Eigen::Vector3d at = bay::ModulePose(m).translation();
      t += bay::ModuleBodyXml(bay::ModuleName(m), {at.x(), at.y(), at.z()}, m);
    }
    t += "    </body>\n";
  }

  // The pallet: its deck, and the modules still on it. Module j is the j-th
  // taken (src/layout.h, Pallet, gives the pallet's frame and the places);
  // with n left, j = modules - n .. modules - 1 are there.
  if (!inside_only) {
    const Pallet& p = layout.pallet;
    const bool upright = p.stands == "on_long_edge";
    const Eigen::Vector3d deck =
        upright ? Eigen::Vector3d(bay::kModuleLength / 2.0 + kPalletDeckMargin,
                                  (p.modules * p.pitch) / 2.0 + kPalletDeckMargin,
                                  p.base_height / 2.0)
                : Eigen::Vector3d(bay::kModuleLength / 2.0 + kPalletDeckMargin,
                                  bay::kModuleWidth / 2.0 + kPalletDeckMargin,
                                  p.base_height / 2.0);
    t += "    <body name=\"pallet\" pos=\"" + V3(p.pos) + "\" quat=\"" +
         Quat(Rpy({0.0, 0.0, p.yaw_deg})) + "\">\n";
    t += "      <geom name=\"pallet_base\" class=\"layout_pallet\" pos=\"0 0 " +
         Num(p.base_height / 2.0) + "\" size=\"" + V3(deck) + "\"/>\n";
    for (int j = p.modules - pallet_modules; j < p.modules; ++j) {
      const std::string name = "pallet_module_" + std::to_string(j);
      if (upright) {
        // Standing on its long edge: a quarter turn about x takes the
        // module's z (its glass's normal) to -y.
        t += "      <body name=\"" + name + "_place\" pos=\"0 " +
             Num((j - (p.modules - 1) / 2.0) * p.pitch) + " " +
             Num(p.base_height + bay::kModuleWidth / 2.0) +
             "\" quat=\"0.70710678118654757 0.70710678118654757 0 0\">\n";
        t += bay::ModuleBodyXml(name, {0.0, 0.0, 0.0});
        t += "      </body>\n";
      } else {
        t += bay::ModuleBodyXml(
            name, {0.0, 0.0,
                   p.base_height + (p.modules - 1 - j) * p.pitch +
                       bay::kModuleThickness / 2.0});
      }
    }
    t += "    </body>\n";
    for (const Stock& s : layout.stocks) {
      t += "    <body name=\"stock_" + s.name + "\">\n";
      t += BoxGeoms(s.boxes, "stock_" + s.name + "_", "layout_stock", s.mass_kg);
      t += "    </body>\n";
    }
  }
  for (const Item& item : layout.items) {
    if (inside_only && !item.travels_inside) continue;
    t += "    <body name=\"item_" + item.name + "\">\n";
    t += BoxGeoms(item.boxes, "item_" + item.name + "_", "layout_item", item.mass_kg);
    t += "    </body>\n";
  }

  // The arms, each on its track. qpos runs through them in this order: the
  // track's slide, if any, then the six joints.
  std::vector<double> qpos;
  for (const ArmUse& arm : layout.arms) {
    const ArmModel* model = FindArmModel(arm.model);
    if (model == nullptr) return fail("no arm model \"" + arm.model + "\"");
    ArmSetup setup;
    if (!Setup(layout, arm, state, &setup, error)) return "";
    const Eigen::Matrix3d rotation = Rpy(arm.base_rpy_deg);
    if (arm.track) {
      const Track& track = *arm.track;
      t += "    <body name=\"" + arm.name + "_beam\">\n";
      t += "      <geom name=\"" + arm.name + "_beam\" class=\"layout_beam\" pos=\"" +
           V3(track.beam.pos) + "\" size=\"" + Half(track.beam.size) +
           "\" mass=\"" + Num(track.mass_kg / 2.0) + "\"/>\n";
      t += "    </body>\n";
    }
    const std::string body = arm.name + (arm.track ? "_carriage" : "_mount");
    t += "    <body name=\"" + body + "\" pos=\"" + V3(arm.base_pos) +
         "\" quat=\"" + Quat(rotation) + "\">\n";
    if (arm.track) {
      const Track& track = *arm.track;
      const Eigen::Vector3d d =
          Eigen::Vector3d(track.direction[0], track.direction[1], track.direction[2])
              .normalized();
      t += "      <joint name=\"" + arm.name + "_track\" type=\"slide\" axis=\"" +
           V3(Eigen::Vector3d(rotation.transpose() * d)) + "\" range=\"" +
           Num(track.lo) + " " + Num(track.hi) + "\"/>\n";
      t += "      <geom name=\"" + arm.name + "_carriage\" class=\"layout_carriage\" pos=\"" +
           V3(track.carriage.pos) + "\" size=\"" + Half(track.carriage.size) +
           "\" mass=\"" + Num(track.mass_kg / 2.0) + "\"/>\n";
      qpos.push_back(setup.track);
    }
    for (const std::string& name : setup.waiting) {
      const Tool& tool = *FindTool(arm, name);
      const int colour = ToolIndex(arm, name) % kToolColours;
      t += "      <geom name=\"" + arm.name + "_wait_" + name +
           "\" class=\"layout_wait_" + std::to_string(colour) + "\" pos=\"" +
           V3(tool.waits_at) + "\" size=\"" + Half(tool.wait_size) + "\"/>\n";
    }
    t += "      <attach model=\"" + arm.name + "_model\" body=\"" + model->root_body +
         "\" prefix=\"" + arm.name + "_\"/>\n";
    t += "    </body>\n";
    for (double deg : setup.joints_deg) qpos.push_back(deg * kPi / 180.0);
  }

  // The probe for the free spaces of the fastenings.
  if (state.everything) {
    auto cylinder = [&](const std::string& name, double radius, double z0, double z1) {
      return "      <geom name=\"probe_" + name + "\" class=\"layout_probe\" size=\"" +
             Num(radius) + "\" fromto=\"0 0 " + Num(z0) + " 0 0 " + Num(z1) + "\"/>\n";
    };
    t += "    <body name=\"probe\" mocap=\"true\" pos=\"0 0 -1\">\n";
    t += cylinder("bolt", bay::kStrapBoltFreeRadius, -0.10, -0.001);
    t += cylinder("pin", bay::kPinFreeRadius, -bay::kPinToolLength, -0.001);
    t += cylinder("locate_below", bay::kLocateFreeRadiusBelow,
                  -bay::kLocateFreeLengthBelow, -0.001);
    t += cylinder("locate_above", bay::kLocateFreeRadiusAbove, 0.004,
                  0.004 + bay::kLocateFreeHeightAbove);
    t += "    </body>\n";
  }
  t += "  </worldbody>\n";

  // The state's poses, as the one keyframe (key 0, "state"), written with
  // every digit.
  if (!qpos.empty()) {
    t += "  <keyframe>\n    <key name=\"state\" qpos=\"";
    for (size_t i = 0; i < qpos.size(); ++i) {
      char buffer[40];
      std::snprintf(buffer, sizeof(buffer), "%.17g", qpos[i]);
      t += (i ? " " : "") + std::string(buffer);
    }
    t += "\"";
    if (state.everything) t += " mpos=\"0 0 -1\" mquat=\"1 0 0 0\"";
    t += "/>\n  </keyframe>\n";
  }
  t += "</mujoco>\n";
  return t;
}

ModelPtr LoadLayoutModel(const Layout& layout, const LayoutState& state,
                         const ModelPaths& paths, std::string* error) {
  std::string top = BuildLayoutXml(layout, state, paths, error);
  if (top.empty()) return nullptr;
  auto fail = [&](const std::string& what) {
    if (error != nullptr) *error = "layout " + layout.name + ": " + what;
    return nullptr;
  };
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  struct VfsGuard {
    mjVFS* vfs;
    ~VfsGuard() { mj_deleteVFS(vfs); }
  } guard{&vfs};
  auto add = [&](const std::string& name, const std::string& text) {
    return mj_addBufferVFS(&vfs, name.c_str(), text.data(),
                           static_cast<int>(text.size())) == 0;
  };
  for (const char* file : {"common.xml", "container.xml"}) {
    std::string text;
    const std::string path = paths.models_dir + "/" + file;
    if (!ReadFile(path, &text)) return fail("cannot read " + path);
    if (!add(file, text)) return fail(std::string("cannot add ") + file);
  }
  for (const ArmUse& arm : layout.arms) {
    ArmSetup setup;
    if (!Setup(layout, arm, state, &setup, error)) return nullptr;
    std::string part;
    if (!ArmPartXml(layout, arm, *FindArmModel(arm.model), setup, paths, &part,
                    error)) {
      return nullptr;
    }
    if (!add(ArmPartName(arm), part)) return fail("cannot add " + ArmPartName(arm));
  }
  if (!add(TopName(layout), top)) return fail("cannot add the model");
  char message[1000] = "";
  ModelPtr model(mj_loadXML(TopName(layout).c_str(), &vfs, message, sizeof(message)));
  if (!model) return fail(std::string("MuJoCo: ") + message);
  // qpos0 is not touched: MuJoCo places a hinge or slide at qpos - qpos0
  // (qpos0 is the configuration the bodies are written in), so a qpos0 other
  // than the file's would move the joints' zeros. The state's poses are
  // keyframe 0, "state".
  return model;
}

LayoutHandles FindHandles(const mjModel& model, const Layout& layout) {
  LayoutHandles h;
  const mjModel* m = &model;
  auto body_id = [&](const std::string& name) {
    return mj_name2id(m, mjOBJ_BODY, name.c_str());
  };
  auto geom_id = [&](const std::string& name) {
    return mj_name2id(m, mjOBJ_GEOM, name.c_str());
  };
  // Whether body b is `root` or below it.
  auto under = [&](int b, int root) {
    if (root < 0) return false;
    while (b > 0) {
      if (b == root) return true;
      b = m->body_parentid[b];
    }
    return b == root;
  };
  auto geoms_under = [&](int root) {
    std::vector<int> out;
    if (root < 0) return out;
    for (int g = 0; g < m->ngeom; ++g) {
      if (under(m->geom_bodyid[g], root)) out.push_back(g);
    }
    return out;
  };
  auto geoms_of_body = [&](int body) {
    std::vector<int> out;
    if (body < 0) return out;
    for (int g = 0; g < m->ngeom; ++g) {
      if (m->geom_bodyid[g] == body) out.push_back(g);
    }
    return out;
  };
  auto push_if = [](std::vector<int>* v, int id) {
    if (id >= 0) v->push_back(id);
  };

  for (const ArmUse& arm : layout.arms) {
    ArmHandle a;
    a.name = arm.name;
    const ArmModel* catalog = FindArmModel(arm.model);
    const std::string p = arm.name + "_";
    if (catalog != nullptr) {
      for (int i = 0; i < 6; ++i) {
        const int j = mj_name2id(m, mjOBJ_JOINT, (p + catalog->joint_names[i]).c_str());
        a.qpos[i] = j >= 0 ? m->jnt_qposadr[j] : -1;
        a.dof[i] = j >= 0 ? m->jnt_dofadr[j] : -1;
      }
    }
    const int track = mj_name2id(m, mjOBJ_JOINT, (arm.name + "_track").c_str());
    if (track >= 0) {
      a.track_qpos = m->jnt_qposadr[track];
      a.track_dof = m->jnt_dofadr[track];
    }
    a.flange_site = mj_name2id(m, mjOBJ_SITE, (p + "flange_site").c_str());
    for (const Tool& tool : arm.tools) {
      const int site = mj_name2id(m, mjOBJ_SITE, (p + "work_" + tool.name).c_str());
      if (site >= 0) a.work_site[tool.name] = site;
      std::vector<int> on;
      for (const Box& b : tool.boxes) push_if(&on, geom_id(p + "tool_" + tool.name + "_" + b.name));
      if (!on.empty()) a.tool_geoms[tool.name] = on;
      const int wait = geom_id(p + "wait_" + tool.name);
      if (wait >= 0) a.waiting_tool_geoms[tool.name] = {wait};
    }
    for (const char* part : {"module", "rail"}) {
      std::vector<int> held = geoms_under(body_id(p + "held_" + part));
      if (!held.empty()) a.held_geoms[part] = held;
    }
    if (catalog != nullptr) {
      const int root = body_id(p + catalog->root_body);
      const int mount = body_id(p + "tool_mount");
      for (int g : geoms_under(root)) {
        if (under(m->geom_bodyid[g], mount)) continue;
        if (m->geom_contype[g] == 0 && m->geom_conaffinity[g] == 0) continue;
        a.link_geoms.push_back(g);
      }
    }
    push_if(&a.carriage_geoms, geom_id(arm.name + "_carriage"));
    push_if(&a.beam_geoms, geom_id(arm.name + "_beam"));
    h.arms.push_back(a);
  }

  h.container_geoms = geoms_of_body(body_id("container_frame"));
  h.holder_geoms = geoms_of_body(body_id("holder"));
  h.tube_geoms = geoms_of_body(body_id(bay::TubeName()));
  for (int k = 0; k < bay::kRailCount; ++k) {
    const int b = body_id(bay::RailName(k));
    if (b < 0) break;
    h.rail_geoms.push_back(geoms_of_body(b));
  }
  for (int i = 0; i < bay::kModuleCount; ++i) {
    const int b = body_id(bay::ModuleName(i));
    if (b < 0) break;
    h.module_geoms.push_back(geoms_of_body(b));
  }
  push_if(&h.pallet_base_geoms, geom_id("pallet_base"));
  // Indexed by the order of taking: [j] is module j, empty when it is gone.
  {
    std::vector<std::vector<int>> pallet(layout.pallet.modules);
    bool any = false;
    for (int j = 0; j < layout.pallet.modules; ++j) {
      pallet[j] = geoms_of_body(body_id("pallet_module_" + std::to_string(j)));
      any = any || !pallet[j].empty();
    }
    if (any) h.pallet_module_geoms = pallet;
  }
  for (const Stock& s : layout.stocks) {
    std::vector<int> g = geoms_of_body(body_id("stock_" + s.name));
    if (!g.empty()) h.stock_geoms[s.name] = g;
  }
  for (const Item& item : layout.items) {
    std::vector<int> g = geoms_of_body(body_id("item_" + item.name));
    if (!g.empty()) h.item_geoms[item.name] = g;
  }
  const int probe = body_id("probe");
  if (probe >= 0) {
    h.probe_mocap = m->body_mocapid[probe];
    h.probe_bolt_geom = geom_id("probe_bolt");
    h.probe_pin_geom = geom_id("probe_pin");
    h.probe_locate_below_geom = geom_id("probe_locate_below");
    h.probe_locate_above_geom = geom_id("probe_locate_above");
  }
  return h;
}

Eigen::Vector3d BayPoint(const Layout& layout, const Eigen::Vector3d& in_bay) {
  return Eigen::Vector3d(layout.tube.axis_x, layout.tube.plain_end_y,
                         layout.tube.axis_z) +
         in_bay;
}

}  // namespace cell::layout
