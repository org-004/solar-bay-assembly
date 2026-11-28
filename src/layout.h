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
// The structs mirror the file field for field; a field's name in the file is
// the member's name. The parts (Box to Approach) are in src/layout_parts.h,
// the rest below. LoadLayout reads a file and refuses one with a missing or
// unknown field, a wrong type, an arm model that is not in
// src/arm_catalog.h, or a name that is used twice, and says which.
//
// Where a new field goes: a top-level field is declared in Layout below and
// read in ReadLayout in src/layout.cc; a field of a part is declared in its
// struct in src/layout_parts.h and read in src/layout_read.cc. The field
// readers both use (Fields, Number, ...) are in src/layout_read.h.
//
// Which layout is chosen is not in a layout file. A registered version's
// layout file may not change (versions/versions.json keeps its sha256), so a
// flag there could never move to a later layout. The chosen layout is the
// layout of the newest version in versions/versions.json whose status is
// "chosen" (docs/scorecard.md); ChosenLayout reads it there, and
// tools/evolution/add.sh, which registers a version, is what changes it. A
// layout file that still has a "chosen" field is refused as having an
// unknown field.
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
//   pins_take    a magazine of pins taken from a stock of pins, once a bay
//                for each arm that pins: point i is taken from the i-th
//                stock whose name is "pins" or starts with "pins_".
//
// The base. "base" names what the cell stands on: "container", the closed
// container of src/container.h with its sides open (the five layouts of run
// 011), or "flat_rack", the base alone of src/flat_rack.h. The model and
// every check are made with that base. A layout on the container has the
// field "container" and none of "flat_rack", "margins", "ports" and
// "approaches"; a layout on the flat rack has those four and no "container".
// LoadLayout refuses either kind of file with a field of the other.
//
// Two racks. "two_flat_racks" is the second direction: two 20 ft flat racks
// of src/flat_rack.h (ISO 668's 6.058 x 2.438 m each) set down side by side,
// long sides adjacent, structurally separate but one cell. A layout on them
// has the fields of a layout on the flat rack, with "two_flat_racks" in
// place of "flat_rack". The frame: the origin on the ground under the middle
// of the pair as set down at the nominal gap, +Y along the racks, +X across;
// rack "a" stands on -X, its long side at x = -gap_m / 2, rack "b" on +X, its
// long side at x = +gap_m / 2, both centred on y = 0. Every arm, the holder,
// and every stock and item that travels names the rack it stands on, "a" or
// "b" (the field "rack", required on this base and refused on the others):
// a thing stands on one rack only, and travels on it. Arms may reach across
// the gap. The checker checks each rack on its own (its stow envelope, what
// it carries) and the cell as one with rack "b" and everything on it moved to
// the ends of the stated tolerance of where the racks are set down: the gap
// from gap_lo_m to gap_hi_m, and rack "b" shifted along Y by up to
// misalign_m either way. Ports lie on the outline of the pair's outer
// rectangle.
//
// Stages. A torque tube longer than the base is worked in sections: the
// infeed and the outfeed units move it through the cell, and the work is
// done in stages. "staging" (optional, on either flat-rack base) lists them
// in order; without it there is one stage, the whole bay with the tube at
// "tube", as before. At stage s the tube lies at "tube" moved by
// tube_offset_m along +Y (its plain end at plain_end_y + tube_offset_m); the
// stage's rails are set and bolted and its modules taken, set, located,
// pinned and swaged there, while the holder grips the stage's grips (the
// stretches bay_hold_<hold> of the tube, which move with it) and the infeed
// or the outfeed holds the rest of the tube ("held_by", in words). Every
// rail and every module is in exactly one stage. A strap bolt goes with its
// rail, a hole with its module, a take with the part it takes, an arm's pin
// magazine with its first hole. A module's two rails (m and m + 1) are set
// at its stage or an earlier one. Between two stages the units move the tube
// and what is built on it by the difference of their offsets
// (staging.transfer_s each time).
//
// Work divided among arms. "work" may hold several entries of one kind, each
// naming an arm, a tool and the points of the kind that arm does: "points" is
// "all" or a list of numbers and ranges such as "0-11,14,20-23", both ends
// included. PointCount gives how many points a kind has in a layout. Each
// point of each kind must be given to exactly one entry: LoadLayout refuses a
// layout with a point given to no arm or to two, and says which. Several arms
// may be of the same model.
//
// Pallets. "pallets" is a list of pallet places. The points of module_take
// run through the pallets in order: point j of pallet p has the number
// (modules of the pallets before p) + j. The bay that is checked takes its
// module m as point m, so from the first pallet; the points of the other
// pallets are checked for reach, clearance and approach and are in no step
// of the bay.
//
// Ports (flat rack). A port is where a separate unit, one that can be
// exchanged, stands against the cell: what feeds it parts or takes bays
// away. docs/flat-rack-cell.md, "The ports", gives them as an interface.
//
// Approaches (flat rack). For each kind of work, from where the tool comes
// to a work point and to where it leaves it, by rule: a direction and a
// distance. The approach pose is the work pose with the tool's working point
// moved by approach_m along approach_dir and the tool's orientation kept; the
// tool then moves straight to the work pose. The retreat pose is the work
// pose moved by retreat_m along retreat_dir; the tool moves straight to it
// after the work. A direction is given in the tool's frame as it is at the
// work pose ("tool": [0, 0, -1] is straight back along the tool's axis) or in
// the layout's frame ("world": [0, 0, 1] is straight up).

#ifndef SRC_LAYOUT_H_
#define SRC_LAYOUT_H_

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "src/layout_parts.h"

namespace cell::layout {

// The order of the bay's work, where it differs from the plain one (an
// optional field "order" of a layout on the flat rack; without it every
// member has its default, which is the order of run 011). The planner's
// rules for each member are in the opening comment of src/layout_plan.h.
struct WorkOrder {
  // The bay's modules in the order they are set: the k-th module taken (the
  // nearest left on the pallet, module_take point k) is set as bay module
  // modules[k]. A permutation of 0..8; empty: 0, 1, ..., 8.
  std::vector<int> modules;
  // When the first module may be taken: "after_all_bolts" (once every strap
  // bolt is driven) or "after_their_rails" (each module once the strap bolts
  // of its own two rails are driven).
  std::string modules_start = "after_all_bolts";
  // When an arm with holes changes to its pin tool and takes its magazine:
  // "just_in_time" (for its first hole, once that hole's module is set) or
  // "early" (as soon as its work before is done; it then waits for the
  // module where the magazine take's retreat left it).
  std::string pin_tools = "just_in_time";
  std::string note;
  std::string basis;
};

// "retreat_legs" (optional, flat rack bases only): the figures of a leg that
// leaves contact into free air (a retreat), in place of the plan's leg_speed
// and leg_accel, which approaches keep (src/layout_plan.h, PlanFigures).
// At most kMaxRetreatSpeed and kMaxRetreatAccel; without it a retreat runs at
// the approach's figures.
inline constexpr double kMaxRetreatSpeed = 0.5;  // m/s of the working point.
inline constexpr double kMaxRetreatAccel = 3.0;  // m/s^2 of the working point.
struct RetreatLegs {
  double speed = 0.0;
  double accel = 0.0;
  std::string basis;
};

// "path_tries" (optional, flat rack bases only; run 042): each move is
// searched from this many seeds, the move's own first, each path shortened,
// and the one whose timed trajectory is quickest is kept (PlanFigures). An
// integer from 2 to kMaxPathTries. Why: a blocked move gets whatever way round
// the obstacle its one search found, and shortening never changes that way;
// another seed often goes round closer. Without it, one search.
// "plan_seed" (optional, flat rack bases only; run 066): PlanFigures::seed, 2
// to kMaxPlanSeed, to plan one layout from other seeds. Without it, 1.
inline constexpr int kMaxPathTries = 16, kMaxPlanSeed = 1000000;

// "torque_limit_scale" (optional, flat rack bases only; run 045): a fraction
// in [kMinTorqueLimitScale, 1] of the module arm's motor limits. A module arm
// move whose inverse dynamics (src/layout_torque.h) demand more than this of a
// joint's or the track's limit is played slower by uniform time scaling until
// it holds (Segment::time_scale); only that move. Why: the datasheet timing
// knows no payload or torque (docs/flat-rack-physics.md). Without it, none.
inline constexpr double kMinTorqueLimitScale = 0.5;

struct Layout {
  std::string name;            // The file's stem.
  std::string title;
  std::string summary;         // One paragraph.
  std::string small_arms_stand;  // "below", "beside" or "above" the tube.
  // "container", "flat_rack" or "two_flat_racks".
  std::string base;
  ContainerState container;      // Only with base "container".
  FlatRackState flat_rack;       // Only with base "flat_rack".
  TwoFlatRacksState two_flat_racks;  // Only with base "two_flat_racks".
  // In the file only on a flat rack base ("flat_rack", "two_flat_racks"); so
  // are ports, approaches, order and staging below.
  Margins margins;
  TubePlace tube;
  Holder holder;
  std::vector<ArmUse> arms;
  std::vector<Pallet> pallets;   // At least one.
  std::vector<Stock> stocks;
  std::string strap_bolts_from;  // In words.
  Way tube_in;
  Way bay_out;
  std::vector<Work> work;
  std::string rail_order;  // "all_rails_then_all_bolts" or "each_rail_then_its_bolt".
  WorkOrder order;                    // Only with base "flat_rack"; optional.
  std::vector<Item> items;
  std::vector<Port> ports;            // Only with base "flat_rack".
  std::vector<Approach> approaches;   // Only with base "flat_rack"; one a kind.
  std::optional<Staging> staging;  // Optional (flat rack); without it one stage, the bay at "tube".
  std::optional<RetreatLegs> retreat_legs;  // Optional on a flat rack base.
  std::optional<int> path_tries, plan_seed;  // Optional on a flat rack base.
  std::optional<double> torque_limit_scale; // Optional on a flat rack base.
  bool bay_out_when_clear = false;  // Optional on a flat rack base (src/layout_clear_out.h).
  // "swage_after_pins" (optional, flat rack bases only; run 070): a swage of
  // module m waits for the module's set and its last pin step, whichever arm
  // sets it, not for the swage before it, so two arms may swage one module at
  // once (docs/bay-hardware.md: swaging needs only that all four pins are in).
  // Locates and pins keep their order. Without it, or false, each hole step
  // waits for the module's hole step before it.
  bool swage_after_pins = false;
  // "borrowed_last" (optional, flat rack bases only; run 072): a hole step of
  // a module that another arm locates (borrowed work, meant for swages lent
  // under swage_after_pins) is done after the arm's own modules of the bay,
  // in the order of the modules' sets, not at its module's place. The
  // checker's step list carries the order (src/layout_check.cc, SolveAll),
  // so the planner, the verifier and the critical chain follow it. Without
  // it, or false, each arm works its steps in the modules' set order.
  bool borrowed_last = false;
  std::optional<std::string> first_bay_early;  // Its basis; optional (src/layout_bays.h).
  std::string next_bay_fill;  // The arm it names; empty without it (src/layout_bays.h).
};

// Reads layouts/<name>.json. Returns nothing and sets *error on any fault.
std::optional<Layout> LoadLayout(const std::string& path, std::string* error);

// The names of the layouts in a directory (the files' stems), sorted.
std::vector<std::string> ListLayouts(const std::string& directory);

// The registry of versions: the environment's VERSIONS_JSON when it is set,
// else "versions/versions.json".
std::string VersionsPath();

// The name of the chosen layout: the stem of the layout path of the last
// version (the newest; the file lists them oldest first) whose status is
// "chosen" and that records a layout. Returns nothing and sets *error when
// the file cannot be read or parsed or no such version exists.
std::optional<std::string> ChosenLayout(const std::string& versions_path,
                                        std::string* error);

// Whether the layout stands on flat racks: base "flat_rack" or
// "two_flat_racks". The rules of the flat rack (margins, ports, approaches,
// order, staging, the transport rows of src/layout_rows.h) apply to both.
bool OnFlatRacks(const Layout& layout);

// The stages of the layout's work: its staging's, or one stage named "1"
// with offset 0, every rail and module and the holder's own grips and angle
// pin, when it has none.
std::vector<Stage> StagesOf(const Layout& layout);

// The rails and the modules of a stage, sorted ("all" gives every one).
std::vector<int> StageRails(const Stage& stage);
std::vector<int> StageModules(const Stage& stage);

// The layout as it stands at a stage: the tube moved by the stage's
// tube_offset_m along +Y, the holder's grips and angle pin the stage's.
// Everything else, the bay's division of work included, is the layout's.
Layout AtStage(const Layout& layout, const Stage& stage);

// The layout with rack "b" set down (dx, dy) from its nominal place (a gap
// of gap_m + dx; dy along the racks): rack "b" and everything that stands on
// it move with it (each arm's base and track beam, the holder and with it the
// tube if the holder is on "b", the stocks and items that travel on "b").
// Rack "a", what stands on it, and the units outside the bases (pallets,
// stocks and items that do not travel, the ports) stay: they are set down
// relative to the cell by their own means. Adds (dx, dy) to
// two_flat_racks.b_shift. Only for base "two_flat_racks".
Layout MovedRackB(const Layout& layout, double dx, double dy);

// Points as a work entry writes them ("all", "0-3,7"), checked against
// count; throws std::runtime_error naming `path` on a fault.
std::vector<int> ParsePointList(const std::string& text, int count,
                                const std::string& path);

// Small lookups. Each returns nullptr when there is none.
const ArmUse* FindArm(const Layout& layout, const std::string& name);
// The index of the arm whose beam arm `arm` rides: the arm its track's
// shares_beam_with names, else `arm` itself.
int BeamArm(const Layout& layout, int arm);
const Tool* FindTool(const ArmUse& arm, const std::string& name);
// The first entry of the kind: the only one where the kind is not divided.
const Work* FindWork(const Layout& layout, const std::string& kind);
// The entry of the kind that has the point `index`.
const Work* FindWork(const Layout& layout, const std::string& kind, int index);
// The points of a work entry, sorted; "all" gives 0 .. PointCount - 1.
std::vector<int> PointsOf(const Layout& layout, const Work& work);
// How many points a kind of work has in this layout: rail_take, rail_set and
// strap_bolt 10, module_set 9, locate, pin and swage 36, module_take the
// modules of all pallets, pins_take the number of stocks of pins.
int PointCount(const Layout& layout, const std::string& kind);
// The bay's modules in the order they are set (WorkOrder::modules, or
// 0..8 when the layout gives none), and the inverse: where module m comes.
std::vector<int> ModuleOrder(const Layout& layout);
std::vector<int> ModulePositions(const Layout& layout);
// The stocks of pins in the file's order: named "pins" or "pins_...".
std::vector<const Stock*> PinStocks(const Layout& layout);
// The names of the arms that have a point of any of these kinds, in the
// layout's order of arms.
std::vector<std::string> ArmsDoing(const Layout& layout,
                                   const std::vector<std::string>& kinds);
const Approach* FindApproach(const Layout& layout, const std::string& kind);
const Port* FindPort(const Layout& layout, const std::string& name);
const Stock* FindStock(const Layout& layout, const std::string& name);

// The kinds of work, in the order of the comment above.
const std::vector<std::string>& WorkKinds();

}  // namespace cell::layout

#endif  // SRC_LAYOUT_H_
