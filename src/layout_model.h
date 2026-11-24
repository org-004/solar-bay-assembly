// A layout as a MuJoCo model.
//
// BuildLayoutXml writes the model of a layout in a given state: the
// container (models/container.xml), the bay as far as it is built
// (src/bay_model.h, placed where the layout's "tube" says), the holder's
// boxes, every arm from its public description with its track, its tools and
// what it holds as simple shapes, the pallet with the modules still on it,
// the stocks and the other items. LoadLayoutModel compiles it. The viewer
// (//:viewer_layout), the test of the model and the layout checker all use
// this one builder, so what is checked is what is looked at.
//
// States. A LayoutState says how far the bay is built, how many modules are
// left on the pallet, which tool each arm has on its flange, what it holds,
// and where each arm is. The default state is the cell waiting with a bare
// tube in the holder. `stowed` gives the transport state instead: only what
// travels inside, each arm at its stow pose. `everything` is for the
// checker: all ten rails, all nine modules, all 36 pallet modules, every
// tool of every arm on its flange at once and every part an arm can hold,
// all in one model, so that one compiled model serves every check and the
// checker leaves out what is not there at a step by leaving its geoms out of
// the lists it measures against (LayoutHandles gives the lists).
//
// Arms keep the joint ranges of their model files; the checker applies the
// datasheet's ranges of src/arm_catalog.h. An arm on a track has one more
// joint, a slide along the track, before its six.
//
// The model needs files beside the layout: models/common.xml,
// models/container.xml, the KUKA's meshes as models/arm.xml names them, and
// MuJoCo Menagerie's ur10e.xml with its assets. ModelPaths says where they
// are; FromEnvironment reads MODELS_DIR and UR10E_XML, which the Bazel
// targets set.

#ifndef SRC_LAYOUT_MODEL_H_
#define SRC_LAYOUT_MODEL_H_

#include <array>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "src/bay_hardware.h"
#include "src/layout.h"
#include "src/mj_handles.h"

namespace cell::layout {

struct ModelPaths {
  std::string models_dir;  // The directory that holds common.xml and arm.xml.
  std::string ur10e_xml;   // Menagerie's ur10e.xml; its assets lie beside it.
  static ModelPaths FromEnvironment();
};

struct LayoutState {
  bool stowed = false;
  bool everything = false;
  bool tube = true;                         // A tube lies in the holder.
  int rails = 0;                            // Rails 0 .. rails-1 are on the tube.
  int modules = 0;                          // Modules 0 .. modules-1 are on.
  int pallet_modules = 36;                  // Modules left on the pallet.
  // For each arm by name; an arm that is not listed has its first tool,
  // holds nothing and stands at its wait pose.
  std::map<std::string, std::string> tool;  // "" for a bare flange.
  std::map<std::string, std::string> held;  // "module", "rail" or "".
  std::map<std::string, std::array<double, 6>> joints_deg;
  std::map<std::string, double> track;
};

// The model's text, or an empty string with *error set.
std::string BuildLayoutXml(const Layout& layout, const LayoutState& state,
                           const ModelPaths& paths, std::string* error);

// The compiled model, or nullptr with *error set. Its keyframe 0 ("state")
// holds the state's arm poses and track values: start with
// mj_resetDataKeyframe(model, data, 0). qpos0 is the model files' zero pose
// and stays so, because MuJoCo places a joint at qpos - qpos0.
ModelPtr LoadLayoutModel(const Layout& layout, const LayoutState& state,
                         const ModelPaths& paths, std::string* error);

// Where to find one arm in a model made here.
struct ArmHandle {
  std::string name;
  std::array<int, 6> qpos{};   // Addresses in qpos of its six joints.
  std::array<int, 6> dof{};    // Addresses in qvel.
  int track_qpos = -1;         // Of its track's slide, or -1.
  int track_dof = -1;
  int flange_site = -1;        // At the flange, z pointing out.
  // For each tool by name: a site at its working point, with the tool's
  // frame; the geoms of the tool on the flange; the geoms of its box where
  // it waits.
  std::map<std::string, int> work_site;
  std::map<std::string, std::vector<int>> tool_geoms;
  std::map<std::string, std::vector<int>> waiting_tool_geoms;
  // The geoms of what it can hold, by part ("module", "rail"), fixed to the
  // working point of the tool that holds that part.
  std::map<std::string, std::vector<int>> held_geoms;
  std::vector<int> link_geoms;      // The arm's own collision geoms, base included.
  std::vector<int> carriage_geoms;  // Moves along the track.
  std::vector<int> beam_geoms;      // Fixed.
};

// Where to find everything in a model made here. A list is empty when the
// state has none of the thing.
struct LayoutHandles {
  std::vector<ArmHandle> arms;  // In the layout's order.
  std::vector<int> container_geoms;
  std::vector<int> holder_geoms;
  std::vector<int> tube_geoms;
  std::vector<std::vector<int>> rail_geoms;           // [rail], strap included.
  std::vector<std::vector<int>> module_geoms;         // [module]
  std::vector<int> pallet_base_geoms;
  // [j] is module j in the order of taking (src/layout.h, Pallet), one entry
  // for each of the pallet's modules; an entry is empty when that module has
  // been taken (state.pallet_modules = n leaves j = modules - n .. modules - 1).
  std::vector<std::vector<int>> pallet_module_geoms;
  std::map<std::string, std::vector<int>> stock_geoms;
  std::map<std::string, std::vector<int>> item_geoms;
  // A mocap body with one see-through cylinder for each free space of
  // docs/bay-hardware.md, none of which collides: the checker moves the body
  // to a fastening point, its z along the way the tool travels, and measures
  // from the cylinder. Each cylinder lies behind the point, from z = -length
  // to z = -0.001. Present only in the `everything` state.
  int probe_mocap = -1;          // Index into mocap_pos, or -1.
  int probe_bolt_geom = -1;      // Radius kStrapBoltFreeRadius, 0.10 long.
  int probe_pin_geom = -1;       // kPinFreeRadius, kPinToolLength.
  int probe_locate_below_geom = -1;  // kLocateFreeRadiusBelow, kLocateFreeLengthBelow.
  // Radius kLocateFreeRadiusAbove, from z = +0.004 to
  // z = +0.004 + kLocateFreeHeightAbove: above the flange's top when the
  // body is at the rail's site of a pin.
  int probe_locate_above_geom = -1;
};

LayoutHandles FindHandles(const mjModel& model, const Layout& layout);

// The bay's frame in the layout: where a point of src/bay_hardware.h is.
Eigen::Vector3d BayPoint(const Layout& layout, const Eigen::Vector3d& in_bay);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_MODEL_H_
