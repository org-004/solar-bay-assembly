// The flat rack the first cell stands on, beside the closed container of
// src/container.h.
//
// A 40 ft high-cube flat rack is a container's base without sides, posts or
// roof: a deep steel base with a deck, and two end frames that fold down or
// come off. The owner allowed it for the first cell on 2026-11-24: the cell
// only has to travel on a semi trailer, where a flat rack rides on a
// container chassis locked by its four bottom corner fittings. In the working
// state, which is what this header and models/flat_rack.xml describe, the end
// frames are dropped or taken off and the flat rack is the base alone: no
// post, side, roof or end frame, so nothing but the deck carries anything and
// nothing stands in an arm's way above it.
//
// The frame is the container's: the origin on the ground under the middle of
// the base, +Y along it, +Z up, +X across. Metres and kilograms.
//
// docs/bay-hardware.md, "The container", table "The flat rack this project
// assumes", says where each figure comes from and marks it published, derived
// or ours. The outside length and width are ISO 668's; the base's height and
// the tare are those of published 40 ft high-cube flat racks; the members the
// base is drawn with are this project's assumption and not a structural
// calculation. models/flat_rack.xml holds the same numbers and
// tests/flat_rack_test.cc checks that the two agree, so a change is made
// here and there together.
//
// Transport. Stowed, the whole cell with its base must be inside the outside
// of a 40 ft high-cube container, TransportEnvelope(): 12.192 x 2.438 x
// 2.896 m from the ground, the base included, so the cell has
// kHeight - kDeckHeight above the deck. Its mass must be within
// kCellMassBudget, the road's gross mass less the flat rack's tare. On a
// chassis whose deck is at kChassisDeckHeight the stowed cell is
// kStowedHeightOnChassis high, the lowest usual state limit in the United
// States (13 ft 6 in).
//
// A container that closes. The owner would rather have one in the end, so
// the checker also reports, without a limit, how the stowed cell compares
// with what a closed container would give it: ClosedInsideEnvelope(), the
// closed container's inside envelope of src/container.h (11.830 x 2.330 x
// 2.240 m) standing on this deck, and kBetweenEndFrames, the least length
// between a flat rack's raised end frames.

#ifndef SRC_FLAT_RACK_H_
#define SRC_FLAT_RACK_H_

#include <string>
#include <vector>

#include <Eigen/Core>

namespace cell::flat_rack {

// The outside of the base: ISO 668's 40 ft length and width.
inline constexpr double kLength = 12.192;  // along Y
inline constexpr double kWidth = 2.438;    // along X
// The top of the deck above the ground the flat rack stands on: the base's
// height of published 40 ft high-cube flat racks.
inline constexpr double kDeckHeight = 0.648;
// The height of the transport envelope, from the ground: a high cube's.
inline constexpr double kHeight = 2.896;

// Centres of the corner fittings, as a container's.
inline constexpr double kFittingSpacingLength = 11.985;
inline constexpr double kFittingSpacingWidth = 2.259;

// The members the base is drawn with (ours): two side rails the full depth
// of the base, an end sill between them at each end, and the deck between
// the side rails and the sills.
inline constexpr double kSideRailWidth = 0.150;
inline constexpr double kEndSillLength = 0.181;
inline constexpr double kDeckThickness = 0.200;

// Masses.
inline constexpr double kTareMass = 5900.0;
inline constexpr double kTransportGrossMass = 23800.0;  // As src/container.h.
inline constexpr double kCellMassBudget = kTransportGrossMass - kTareMass;

// On a chassis.
inline constexpr double kChassisDeckHeight = 1.219;
inline constexpr double kStowedHeightOnChassis = kChassisDeckHeight + kHeight;

// What a container that closes would give the cell (no limit; reported).
inline constexpr double kClosedInsideLength = 11.830;
inline constexpr double kClosedInsideWidth = 2.330;
inline constexpr double kClosedInsideHeight = 2.240;
inline constexpr double kBetweenEndFrames = 11.652;

struct Box {
  std::string name;
  Eigen::Vector3d centre = Eigen::Vector3d::Zero();
  Eigen::Vector3d half_size = Eigen::Vector3d::Zero();
};

// Everything must be inside this when the flat rack travels, the base
// included. The site flat_rack_transport_envelope of the model.
inline Box TransportEnvelope() {
  return {"flat_rack_transport_envelope",
          {0.0, 0.0, kHeight / 2.0},
          {kWidth / 2.0, kLength / 2.0, kHeight / 2.0}};
}

// The space above the deck inside the transport envelope: where the stowed
// cell may be. The site flat_rack_cell_envelope of the model.
inline Box CellEnvelope() {
  return {"flat_rack_cell_envelope",
          {0.0, 0.0, (kDeckHeight + kHeight) / 2.0},
          {kWidth / 2.0, kLength / 2.0, (kHeight - kDeckHeight) / 2.0}};
}

// The closed container's inside envelope standing on this deck.
inline Box ClosedInsideEnvelope() {
  return {"flat_rack_closed_inside_envelope",
          {0.0, 0.0, kDeckHeight + kClosedInsideHeight / 2.0},
          {kClosedInsideWidth / 2.0, kClosedInsideLength / 2.0,
           kClosedInsideHeight / 2.0}};
}

// The base's members, each a box, named as in the model.
inline std::vector<Box> Members() {
  const double hx = kWidth / 2.0;
  const double hy = kLength / 2.0;
  const double between_x = hx - kSideRailWidth;
  const double between_y = hy - kEndSillLength;
  std::vector<Box> all;
  int side = 0;
  for (double sx : {-1.0, 1.0}) {
    all.push_back({"flat_rack_side_rail_" + std::to_string(side++),
                   {sx * (hx - kSideRailWidth / 2.0), 0.0, kDeckHeight / 2.0},
                   {kSideRailWidth / 2.0, hy, kDeckHeight / 2.0}});
  }
  const char* ends[] = {"front", "rear"};
  int end = 0;
  for (double sy : {-1.0, 1.0}) {
    all.push_back({std::string("flat_rack_end_sill_") + ends[end++],
                   {0.0, sy * (hy - kEndSillLength / 2.0), kDeckHeight / 2.0},
                   {between_x, kEndSillLength / 2.0, kDeckHeight / 2.0}});
  }
  all.push_back({"flat_rack_deck",
                 {0.0, 0.0, kDeckHeight - kDeckThickness / 2.0},
                 {between_x, between_y, kDeckThickness / 2.0}});
  return all;
}

}  // namespace cell::flat_rack

#endif  // SRC_FLAT_RACK_H_
