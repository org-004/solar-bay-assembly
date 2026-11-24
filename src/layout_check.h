// The checks on a layout.
//
// CheckLayout takes a layout (src/layout.h), builds its model
// (src/layout_model.h) and checks one whole bay. It returns one row for each
// check with the figure and whether it passes, and the steps of building one
// bay with their times. //:check_layout prints them and exits non-zero when
// a row fails. The kinds of check, the `check` of a row:
//
//   transport  everything that travels inside is, in the stowed state,
//              within the container's inside envelope, and its mass is
//              within what is left for the cell (src/container.h);
//   reach      every point an arm must work at is reached by the arm the
//              layout gives it, the tool along the direction the document
//              gives, inside the datasheet's joint ranges less a margin;
//   clearance  at each of those poses the arm, its tool and what it carries
//              keep a stated distance from the bay as built up to that step,
//              the holder, the container, the pallet, the stocks and the
//              other arms where they wait; and the free spaces of the tools
//              and of the locating move are empty;
//   load       what each arm carries, tool and part, is within its payload;
//   ways       the space a tube sweeps coming in and a bay sweeps leaving is
//              empty;
//   time       the time one bay takes, from the steps, by a stated model of
//              the arms' moves;
//   hold       the holder grips only stretches the document allows, not the
//              two ends alone, and is built for the moment the document asks.
//
// The header's remaining comment is the checker's to write: the margins, the
// model of a move, the order of the steps, what touches what by design.

#ifndef SRC_LAYOUT_CHECK_H_
#define SRC_LAYOUT_CHECK_H_

#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_model.h"

namespace cell::layout {

struct CheckRow {
  std::string check;   // One of the kinds above.
  std::string item;    // What was checked: "rail_set 3", "stowed height".
  std::string arm;     // Empty where no arm is meant.
  std::string tool;
  double figure = 0.0;
  std::string unit;    // Of the figure: "deg", "m", "kg", "s".
  double limit = 0.0;  // What the figure is held against.
  bool pass = false;
  std::string detail;  // What limits it: the joint, the nearest thing.
};

// One step of building a bay.
struct BayStep {
  int number = 0;
  std::string what;    // "set rail 3"
  std::string arm;
  std::string tool;
  std::string part;
  std::string from;
  std::string to;
  double time_s = 0.0;
};

struct CheckOptions {
  double joint_margin_deg = 5.0;  // Kept from each end of a joint's range.
  double clearance_m = 0.010;     // The least distance that counts as clear.
};

struct CheckReport {
  std::vector<CheckRow> rows;
  std::vector<BayStep> steps;
  double bay_time_s = 0.0;
  std::string time_model;  // The model of a move, in words, for the output.
  bool pass = false;       // Every row passes.
};

CheckReport CheckLayout(const Layout& layout, const ModelPaths& paths,
                        const CheckOptions& options = {});

// The state of the cell with an arm at one of its work points, for a
// picture: the bay as built up to that point, the tool on the flange, the
// part in it, the arm at the pose the reach check found. `kind` is a kind of
// work of src/layout.h and `index` the point's number. Returns false and
// sets *error when the point is not reached.
bool StateAt(const Layout& layout, const ModelPaths& paths,
             const std::string& kind, int index, LayoutState* state,
             std::string* error);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_CHECK_H_
