// Scoring a run: how well each module was laid on the tube, and how much the
// picking disturbed the modules still standing on the pallet.
//
// Everything here reads the truth: where modules and the tube really are in
// the simulation. That is what a scorer is for, and it is why control code
// must never call these.
//
// No tolerances are set here. How close a module has to be to its slot is
// still an open question for the project, so these functions report numbers
// and leave the judging to whoever reads them.
//
// Units are metres and radians. Describe prints millimetres and degrees.

#ifndef SRC_SCORE_H_
#define SRC_SCORE_H_

#include <optional>
#include <string>

#include "src/cell.h"

namespace cell {

// Where a module lies relative to the TRUE pose of its slot: the pose a
// perfectly placed module would have on the tube where the tube really is
// (Conveyor::TrueSlotPose), so the conveyor's stopping error counts against
// the placement only if the placing did not allow for it.
//
// The slot's frame has its axes along the world's: x across the tube (the
// module's long side), y along the tube (the way it travels, and the
// module's short side), z up. Offsets are of the module's centre, in that
// frame.
struct PlacementError {
  int module = -1;
  int slot = -1;

  double along_tube = 0.0;   // m, slot y: toward the next slot is +
  double across_tube = 0.0;  // m, slot x
  double height = 0.0;       // m, slot z: 0 is lying on the rails' surface

  // Turned about the vertical: the angle from the slot's x to the module's
  // long side, seen from above, counterclockwise positive. A rectangle
  // turned half a turn is the same rectangle, so this is folded into
  // (-pi/2, pi/2].
  double yaw = 0.0;  // rad

  // Out of flat: the angle between the glass's normal and straight up. Near
  // pi means glass down; that is not folded.
  double tilt = 0.0;  // rad

  // How fast it is moving relative to the tube, and whether that is slow
  // enough to call it lying still: under kRestSpeed and kRestSpin.
  double speed = 0.0;  // m/s
  double spin = 0.0;   // rad/s
  bool at_rest = false;
};

// What "at rest" means for PlacementError. Chosen to tell a module that has
// come to lie from one still sliding or rocking, not as a tolerance on the
// placement.
inline constexpr double kRestSpeed = 1e-3;  // m/s
inline constexpr double kRestSpin = 1e-2;   // rad/s

// Module i scored against slot `slot` of the tube as it is now. For when the
// caller knows which slot it meant; i need not be recorded in any slot.
PlacementError MeasurePlacement(const Cell& cell, int i, int slot);

// Module i scored against the slot it was recorded in when it was let go
// (Cell::TruePlacedSlot). Nothing if it is not recorded on the tube.
std::optional<PlacementError> MeasurePlacement(const Cell& cell, int i);

// How far the modules still standing on the pallet (ModuleState::kOnPallet)
// have moved since the stack's baseline: the last Reset, or the last
// Cell::RebaselineStack. Modules that have been taken are left out.
struct StackDisturbance {
  int modules = 0;  // how many were compared

  double worst_displacement = 0.0;  // m, of a module's centre
  int worst_displacement_module = -1;
  double worst_rotation = 0.0;      // rad, the angle of a module's turn
  int worst_rotation_module = -1;
};

StackDisturbance MeasureStackDisturbance(const Cell& cell);

// One line each, in millimetres and degrees.
std::string Describe(const PlacementError& error);
std::string Describe(const StackDisturbance& disturbance);

}  // namespace cell

#endif  // SRC_SCORE_H_
