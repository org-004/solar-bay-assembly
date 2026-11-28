// The parts of a layout, as data: the structs that src/layout.h's Layout is
// made of, from a Box to an Approach. Each mirrors one object of a layout
// file field for field, and a field's name in the file is the member's name;
// src/layout.h's opening comment gives the form, the frame and the units, and
// src/layout_read.cc reads each part (src/layout_read.h).
//
// They are in a header of their own only so that src/layout.h stays within
// the size the architecture's rules allow (tools/architecture/deps.json,
// "size"). Include src/layout.h, not this header: it includes this one.
//
// Where a new field goes. A field of one of these parts is declared in its
// struct here and read in its reader in src/layout_read.cc. A new top-level
// field of a layout is declared in Layout in src/layout.h and read in
// ReadLayout in src/layout.cc, beside the other optional fields of the flat
// rack (the list of allowed keys is in ReadLayout's call of Fields).

#ifndef SRC_LAYOUT_PARTS_H_
#define SRC_LAYOUT_PARTS_H_

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

// The flat rack while the cell works: the base alone (src/flat_rack.h).
struct FlatRackState {
  std::string end_frames;  // "taken_off" or "dropped".
  std::string note;        // What the layout asks of the base, in words.
};

// The two racks while the cell works (base "two_flat_racks"): each the base
// alone, and how they are set down. Rack "a" is the reference; the checker
// moves rack "b" and what stands on it.
struct TwoFlatRacksState {
  std::string end_frames;      // "taken_off" or "dropped", both racks.
  double gap_m = 0.025;        // Nominal gap between the long sides.
  double gap_lo_m = 0.0;       // The tolerance of the gap as set down:
  double gap_hi_m = 0.050;     //   from gap_lo_m to gap_hi_m.
  double misalign_m = 0.025;   // Rack "b" along Y, either way, at the nominal gap.
  std::string note;            // What the layout asks of the bases, in words.
  // Not in the file: how far rack "b" and what stands on it are moved from
  // their places at the nominal gap (MovedRackB sets it; zero as read).
  Vec3 b_shift{};
};

// The margins the checks and the planner keep for this layout. Never below
// 0.010 m and 5 degrees.
struct Margins {
  double clearance_m = 0.010;
  double joint_margin_deg = 5.0;
  std::string reason;
  std::string basis;
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
  std::string rack;            // "a" or "b" on two_flat_racks, else "".
  std::string basis;           // "ours" or "published".
  std::string source;
};

// One stage of the work on a tube that is worked in sections (Staging).
struct Stage {
  std::string name;            // Unique in the layout: "1", "infeed end".
  double tube_offset_m = 0.0;  // The tube's place: "tube" moved by this along +Y.
  std::string rails;           // The rails set at this stage: "0-3".
  std::string modules;         // The modules set at this stage: "0-2", or "".
  std::vector<Grip> grips;     // The holder's grips at this stage.
  int angle_pin_hold = 0;      // The stretch whose rivet holes fix the angle.
  // What puts its pin in those holes at this stage: "holder", or
  // "item:<name>", an item of the layout (an infeed or outfeed unit, which
  // need not travel); the stretch must lie at that thing's boxes.
  std::string angle_pin_by = "holder";
  std::string held_by;         // What else holds the tube, in words.
  std::string note;
};

struct Staging {
  std::string why;             // Why these sections and stages.
  double transfer_s = 0.0;     // Moving the tube from one stage to the next.
  std::vector<Stage> stages;   // In order; at least one.
  std::string basis;
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
  std::string basis, source;
  int magazine_holds = 0;  // Its rail_magazine's rails, 0 if none (src/layout_rail_magazine.h).
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
  // Optional: the arm whose beam this carriage also rides ("" when it rides
  // its own only). The arms of one shared beam may stand on one line: each
  // carriage counts as near by design to every beam of the group, as to its
  // own; carriage to carriage and links to beams stay counted. The named arm
  // has a track of the same direction and shares no beam itself.
  std::string shares_beam_with;
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
  std::string rack;            // "a" or "b" on two_flat_racks, else "".
  // "one_posture_per_module" (optional, false when absent): the checker
  // gives all of this arm's holes of a module (locate, pin, swage) poses of
  // one pose family, chosen together (src/layout_check.h, "One posture per
  // module"). Only layouts with approach rules use it.
  bool one_posture_per_module = false;
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
  std::string rack;            // On two_flat_racks, for a stock not outside.
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
// point, not counting the move there. One entry for each arm that does some
// of the kind's points.
struct Work {
  std::string kind;
  std::string arm;
  std::string tool;
  std::string points;          // "all", or numbers and ranges: "0-11,14".
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
  std::string rack;            // On two_flat_racks, for an item that travels.
  std::string basis;
  std::string source;
};

// A port: where a separate unit stands against the cell.
struct Port {
  std::string name;            // Unique in the layout.
  // What crosses it, one of: "tubes" (bare tubes in), "bays" (finished bays
  // out), "modules", "rails", "pin_magazines".
  std::string crosses;
  std::string unit;            // The unit that stands there, in words.
  // The port's frame, fixed to the base: its origin on the outline of the
  // base's footprint at the deck's height, its x axis turned by frame_yaw_deg
  // about z from the layout's x and pointing out of the base into the zone.
  Vec3 frame_pos{};
  double frame_yaw_deg = 0.0;
  // The space the unit may occupy, in the layout's frame: outside the base's
  // footprint. `name` of the box is not used.
  Box zone;
  // What stands in the zone and where parts are handed over, each one of
  //   "pallet:<index>"   a pallet place and the modules on it,
  //   "stock:<name>"     a stock; parts are handed over at its pick point,
  //   "item:<name>"      an item that does not travel (a cart),
  //   "way:tube_in"      the tube at the first offset of tube_in,
  //   "way:bay_out"      the bay at the last offset of bay_out.
  std::vector<std::string> hands_over;
  // The name of another port whose zone this one shares because one unit
  // serves both (tubes in and bays out on one cart), or "".
  std::string shares_zone_with;
  std::string presents;        // In which pose a part is presented, in words.
  double per_bay = 0.0;        // How many parts cross it for one bay.
  std::string basis;
};

// The approach and the retreat of one kind of work, by rule.
struct Approach {
  std::string kind;
  std::string approach_frame;  // "tool" or "world".
  Vec3 approach_dir{};         // From the work pose to the approach pose; unit length.
  double approach_m = 0.0;
  std::string retreat_frame;   // "tool" or "world".
  Vec3 retreat_dir{};          // From the work pose to the retreat pose; unit length.
  double retreat_m = 0.0;
  std::string note;            // Why this direction and distance.
  std::string basis;
};

}  // namespace cell::layout

#endif  // SRC_LAYOUT_PARTS_H_
