// A layout as a MuJoCo model.
//
// BuildLayoutXml writes the model of a layout in a given state: the
// layout's base (models/container.xml, body "container_frame", for base
// "container"; models/flat_rack.xml, body "flat_rack_frame", for base
// "flat_rack"), the bay as far as it is built (src/bay_model.h, placed where
// the layout's "tube" says), the holder's boxes, every arm from its public
// description with its track, its tools and what it holds as simple shapes
// (on the flat rack a rail an arm holds has its strap hanging open, the rails
// on the tube have theirs closed; on the container the held rail is as run
// 011 drew it, strap closed, so that its record stays as it was), every pallet place with the modules still on it, the
// stocks, the other items and the ports' sites. LoadLayoutModel compiles it. The viewer
// (//:viewer_layout), the test of the model and the layout checker all use
// this one builder, so what is checked is what is looked at.
//
// States. A LayoutState says how far the bay is built, how many modules are
// left on the first pallet (the other pallets are full), which tool each arm
// has on its flange, what it holds, and where each arm is. The default state
// is the cell waiting with a bare tube in the holder. `stowed` gives the
// transport state instead: only what travels with the cell, each arm at its
// stow pose: the holder if it travels inside, the items that travel inside,
// every stock and every pallet (deck and modules) whose "outside" is false;
// no tube, no port, nothing that stands outside. `everything` is for the
// checker: all ten rails, all nine modules, every module of every pallet,
// every tool of every arm on its flange at once and every part an arm can hold,
// all in one model, so that one compiled model serves every check and the
// checker leaves out what is not there at a step by leaving its geoms out of
// the lists it measures against (LayoutHandles gives the lists).
//
// Distances. Everything that measures in a model made here uses
// mj_geomDistance, and MuJoCo 4.0.0 measures a capsule against a box wrongly
// when the capsule's axis passes through the box (it gives a positive
// distance). LoadLayoutModel therefore turns every capsule that can collide
// into the cylinder that covers it; src/layout_model.cc gives the figures.
//
// Pallets. Every pallet place of layout.pallets is in the model. The first
// pallet's names are body "pallet", geom "pallet_base" and bodies
// "pallet_module_<j>"; pallet p >= 1 has "pallet<p>", "pallet<p>_base" and
// "pallet<p>_module_<j>".
//
// Ports. Each port has two sites that collide with nothing, in site group 4
// (hidden by default, as the other sites): "port_<name>_zone", a faint box of
// the zone's size at its place, and "port_<name>_frame", at frame_pos turned
// by frame_yaw_deg about z. They are not in the stowed state.
//
// Several arms of one model are allowed: each arm's part file, its asset and
// every name in it carry the arm's name.
//
// Arms keep the joint ranges of their model files; the checker applies the
// datasheet's ranges of src/arm_catalog.h. An arm on a track has one more
// joint, a slide along the track, before its six.
//
// The model needs files beside the layout: models/common.xml,
// models/container.xml or models/flat_rack.xml, the KUKA's meshes as models/arm.xml names them, and
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
  int pallet_modules = 36;                  // Modules left on the first pallet.
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
  // working point of the tool that holds that part. On the flat rack a held
  // rail has its strap open (geoms ..._held_rail_strap_open_neg and _pos,
  // src/bay_model.h); the rails on the tube have theirs closed, and so has a
  // held rail in the five layouts on the container.
  std::map<std::string, std::vector<int>> held_geoms;
  std::vector<int> link_geoms;      // The arm's own collision geoms, base included.
  std::vector<int> carriage_geoms;  // Moves along the track.
  std::vector<int> beam_geoms;      // Fixed.
};

// Where to find everything in a model made here. A list is empty when the
// state has none of the thing.
struct LayoutHandles {
  std::vector<ArmHandle> arms;  // In the layout's order.
  std::string base_body;         // "container_frame" or "flat_rack_frame".
  std::vector<int> base_geoms;   // The base's members, whichever base.
  // TRANSITIONAL: a copy of base_geoms for code not yet moved to it; it is
  // removed before this work lands.
  std::vector<int> container_geoms;
  std::vector<int> holder_geoms;
  std::vector<int> tube_geoms;
  std::vector<std::vector<int>> rail_geoms;           // [rail], strap included.
  std::vector<std::vector<int>> module_geoms;         // [module]
  std::vector<int> pallet_base_geoms;  // The decks of all pallets, in order.
  // Indexed as the points of module_take (src/layout.h, Pallets): module j of
  // pallet p is entry (modules of the pallets before p) + j, j in the order
  // of taking (src/layout.h, Pallet). An entry is empty when that module has
  // been taken: state.pallet_modules = n leaves j = modules - n .. modules - 1
  // of the first pallet. The list is empty when no pallet module is there.
  std::vector<std::vector<int>> pallet_module_geoms;
  // [p] is the deck and every module present of pallet place p, empty when
  // that pallet is not in the state; the list is empty when no pallet is
  // (stowed, with every pallet outside).
  std::vector<std::vector<int>> pallet_place_geoms;
  std::map<std::string, std::vector<int>> stock_geoms;
  std::map<std::string, std::vector<int>> item_geoms;
  // A mocap body with one see-through cylinder for each free space of
  // docs/bay-hardware.md, none of which collides: the checker moves the body
  // to a fastening point, its z along the way the tool travels, and measures
  // from the cylinder. Each cylinder lies behind the point, from z = -length
  // to z = -0.001. Present only in the `everything` state.
  int probe_mocap = -1;          // Index into mocap_pos, or -1.
  int probe_bolt_geom = -1;      // kStrapBoltFreeRadius, kStrapBoltFreeLength.
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
