// A layout of the cell, as data.
//
// A layout says where everything of the cell that builds bays stands: the
// container in its working state, the tube and what holds it, every arm with
// its tools, the pallet of modules, the stocks of rails and pins, the ways a
// bare tube comes in and a finished bay leaves, and which things travel
// inside the container. One JSON file under layouts/ is one layout, and
// nothing about a layout is written anywhere else: the model that is looked
// at (src/layout_model.h) and the checks (src/layout_check.h) are both made
// from the file. docs/layout.md explains the layouts and compares them.
//
// The structs below mirror the file field for field; a field's name in the
// file is the member's name. LoadLayout reads a file and refuses one with a
// missing or unknown field, a wrong type, an arm model that is not in
// src/arm_catalog.h, or a name that is used twice, and says which.
//
// The frame. One frame for everything: the container's, as
// models/container.xml and src/container.h have it. The origin is on the
// ground under the container's centre, +Y runs along the container toward
// its rear doors, +Z is up, +X is across. Metres, kilograms, seconds;
// angles in degrees in the file because people write it. The container's
// floor is 0.45 above the ground.
//
// The bay in a layout. The tube's axis is parallel to Y. "tube" gives the
// place of the bay's frame of src/bay_hardware.h (origin on the axis at the
// plain end, +Y along the tube, +Z up through the locating holes): its
// origin is at (axis_x, plain_end_y, axis_z) and its axes are the layout's.
//
// A tool's frame has its origin at the centre of the arm's flange and its z
// axis pointing out of the flange. The tool's working point is at
// (0, 0, length); at a work point the tool's z axis lies along the direction
// the document gives for that work, and where the part has a long side (a
// module, a rail) the tool's x axis lies along it. A box is given by its
// full sizes along x, y, z and the place of its centre.
//
// The kinds of work, and what each needs reached (docs/bay-hardware.md gives
// the points and directions; src/bay_hardware.h holds them):
//   rail_take    a rail taken from the stock "rails", once for each rail;
//   rail_set     a rail set down on the tube from above, 10 points;
//   strap_bolt   a strap's bolt driven along -X under the tube, 10 points;
//   module_take  a module taken off the pallet, 36 points, the stack one
//                module smaller each time (see Pallet for the direction);
//   module_set   a module set down on its two rails from above, 9 points;
//   locate       the nose of "Locating a module before pinning" brought up
//                under a hole, 36 points;
//   pin          a pin put in, at the same 36 points;
//   swage        its collar swaged from below, at the same 36 points;
//   pins_take    a magazine of pins taken from the stock "pins", once a bay.

#ifndef SRC_LAYOUT_H_
#define SRC_LAYOUT_H_

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace cell::layout {

using Vec3 = std::array<double, 3>;

// One box of a simple shape: full sizes and the centre, in its owner's frame.
struct Box {
  std::string name;
  Vec3 size{};
  Vec3 pos{};
};

// The container while the cell works. The model is models/container.xml,
// which has both long sides open, the rear doors open, roof and floor kept;
// the fields say so and the checker refuses a state the model does not have.
struct ContainerState {
  std::vector<std::string> sides_open;  // "-x", "+x".
  bool rear_doors_open = true;
  std::string note;  // What the layout asks of the container, in words.
};

struct TubePlace {
  double axis_x = 0.0;
  double axis_z = 0.0;
  double plain_end_y = 0.0;
};

// One place where the holder grips the tube: which of the model's stretches
// bay_hold_<hold> and how long a piece of it, centred on the stretch.
struct Grip {
  int hold = 0;
  double length = 0.0;
};

struct Holder {
  std::string does;
  std::vector<Grip> grips;
  int angle_pin_hold = 0;      // The end stretch whose rivet holes fix the angle.
  double resists_nm = 0.0;     // Moment about the tube's axis it is built for.
  double holds_within_deg = 0.0;  // How close to straight up it keeps the holes.
  double clamp_s = 0.0;        // Time to close on a tube.
  double release_s = 0.0;      // Time to let a finished bay go.
  double mass_kg = 0.0;
  std::vector<Box> boxes;      // Its shape, in the layout's frame.
  bool travels_inside = true;
  std::string basis;           // "ours" or "published".
  std::string source;
};

struct Tool {
  std::string name;
  std::string does;
  double mass_kg = 0.0;
  double length = 0.0;         // Flange to working point.
  std::vector<Box> boxes;      // In the tool's frame.
  // Where it waits when it is not on the flange: in the frame of the arm's
  // base (so it rides with a carriage), the centre of a box of wait_size.
  Vec3 waits_at{};
  Vec3 wait_size{};
  std::string basis;
  std::string source;
};

// A straight track under an arm's base. The base stands at
// base_pos + s * direction for s from lo to hi.
struct Track {
  Vec3 direction{0.0, 1.0, 0.0};
  double lo = 0.0;
  double hi = 0.0;
  double speed = 0.0;          // m/s
  double accel = 0.0;          // m/s^2
  double mass_kg = 0.0;        // Beam and carriage together.
  Box beam;                    // Fixed, in the layout's frame.
  Box carriage;                // Moves with the base, in the base's frame.
  std::string basis;
  std::string source;
};

struct ArmUse {
  std::string name;            // Unique in the layout; prefixes its names in the model.
  std::string model;           // A key of src/arm_catalog.h.
  std::string does;
  Vec3 base_pos{};             // The arm's base frame at s = 0.
  Vec3 base_rpy_deg{};         // Roll, pitch, yaw of the base frame.
  std::optional<Track> track;
  std::vector<Tool> tools;
  std::string first_tool;      // On the flange when a bay starts.
  // Where the carriage stands for a kind of work: one value for every point
  // of the kind, or one for each point. A kind that is not listed leaves the
  // checker to search the track near the point.
  std::map<std::string, std::vector<double>> stops;
  std::array<double, 6> wait_joints_deg{};  // Where it waits while others work.
  double wait_track = 0.0;
  std::array<double, 6> stow_joints_deg{};  // For transport.
  double stow_track = 0.0;
  std::string stow_tool;       // On the flange for transport; may be empty.
  double tool_change_s = 0.0;
};

// The pallet's own frame: its origin at pos, x along the modules' long side,
// y along the stack, z up, turned by yaw_deg about z from the layout's frame.
// "on_long_edge": the modules stand on their long edge on the deck, side by
// side along y, the full stack centred on pos; module j (0 is taken first) has
// its centre at y = (j - (modules - 1) / 2) * pitch and at the deck's height
// plus half a module's width, and its glass faces -y. It is taken by its
// glass: the working point is the centre of the glass face, the tool's z
// along +y, the tool's x along the pallet's x or its opposite. "flat": the
// modules lie glass up in one stack on the deck, module j at the height of
// the deck plus (modules - 1 - j) * pitch, taken from above: the working
// point is the centre of the top module's glass, the tool's z down.
struct Pallet {
  Vec3 pos{};                  // Centre of its footprint, on the ground.
  double yaw_deg = 0.0;        // 0: the modules' long side along X.
  int modules = 0;
  std::string stands;          // "on_long_edge" or "flat".
  std::string how;             // How the modules stand on it, in words.
  double base_height = 0.0;    // Top of the pallet's deck above pos.
  double pitch = 0.0;          // From one module to the next in the stack.
  bool outside = true;
  std::string basis;
  std::string source;
};

// A stock of parts. The pick point is where the tool's working point is when
// a part is taken, with the tool's z along pick_axis.
struct Stock {
  std::string name;            // "rails", "pins".
  std::string part;
  std::string how;             // How the parts arrive and are presented.
  Vec3 pick_pos{};
  Vec3 pick_axis{0.0, 0.0, -1.0};
  int holds = 0;
  double mass_kg = 0.0;
  std::vector<Box> boxes;      // In the layout's frame.
  bool outside = true;
  std::string basis;
  std::string source;
};

// Where the strap bolts come from, if not from a stock: with their rails.
// A way in or out: the thing moves through the offsets in order, each
// measured from its place in the holder. The space it sweeps must be empty.
struct Way {
  std::string what;            // In words: carried by what, through which opening.
  std::vector<Vec3> path;
  double time_s = 0.0;
  std::string basis;
};

// Who does a kind of work, with which tool, and how long it takes at the
// point, not counting the move there.
struct Work {
  std::string kind;
  std::string arm;
  std::string tool;
  double process_s = 0.0;
  std::string note;
  std::string basis;
};

// Anything else: a transfer cart, a tool rack, a feeder.
struct Item {
  std::string name;
  std::string does;
  bool travels_inside = false;
  double mass_kg = 0.0;
  std::vector<Box> boxes;      // In the layout's frame.
  std::string basis;
  std::string source;
};

struct Layout {
  std::string name;            // The file's stem.
  std::string title;
  std::string summary;         // One paragraph.
  bool chosen = false;
  std::string small_arms_stand;  // "below", "beside" or "above" the tube.
  ContainerState container;
  TubePlace tube;
  Holder holder;
  std::vector<ArmUse> arms;
  Pallet pallet;
  std::vector<Stock> stocks;
  std::string strap_bolts_from;  // In words.
  Way tube_in;
  Way bay_out;
  std::vector<Work> work;
  // "all_rails_then_all_bolts" or "each_rail_then_its_bolt".
  std::string rail_order;
  std::vector<Item> items;
};

// Reads layouts/<name>.json. Returns nothing and sets *error on any fault.
std::optional<Layout> LoadLayout(const std::string& path, std::string* error);

// The names of the layouts in a directory (the files' stems), sorted.
std::vector<std::string> ListLayouts(const std::string& directory);

// Small lookups. Each returns nullptr when there is none.
const ArmUse* FindArm(const Layout& layout, const std::string& name);
const Tool* FindTool(const ArmUse& arm, const std::string& name);
const Work* FindWork(const Layout& layout, const std::string& kind);
const Stock* FindStock(const Layout& layout, const std::string& name);

// The kinds of work, in the order of the comment above.
const std::vector<std::string>& WorkKinds();

}  // namespace cell::layout

#endif  // SRC_LAYOUT_H_
