// The flat rack the first cell stands on, beside the container of
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
// The 20 ft flat racks. The namespace `twenty` holds the second direction's
// base: two 20 ft flat racks set down side by side, long sides adjacent
// (src/layout.h, "Two racks"). Each is drawn as the 40 ft one is, scaled: the
// outside is ISO 668's 20 ft (6.058 x 2.438 m), the deck's height and the
// tare are those of published 20 ft flat racks (Hapag-Lloyd's "height bottom"
// 370 mm and tare 2,900 kg), and the members are this project's. Each rack
// travels on its own: its stow envelope is a 20 ft high cube's outside,
// 6.058 x 2.438 x 2.896 m from the ground, the rack included, and the owner
// limits what it carries in transit to 13,608 kg (30,000 lb): the robotics as
// it travels, without parts fed in and without the rack itself, counted as
// docs/scorecard.md counts the mass. Where each figure comes from is in
// docs/bay-hardware.md, "The 20 ft flat racks this project assumes".
// There is no model file: src/layout_model.cc draws both racks from Members().

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

// The two 20 ft flat racks of base "two_flat_racks". Rack "a" stands on -X,
// its long side at x = -gap / 2, rack "b" on +X, its long side at
// x = gap / 2, both centred on y = 0 at the nominal gap; rack "b" may be
// moved by a shift (x, y), as the checker's variants of the gap do.
namespace twenty {

// The outside of one rack: ISO 668's 20 ft length and width.
inline constexpr double kLength = 6.058;  // along Y
inline constexpr double kWidth = 2.438;   // along X
// The top of the deck above the ground: the "height bottom" of Hapag-Lloyd's
// 20' flatrack.
inline constexpr double kDeckHeight = 0.370;
// The height of the stow envelope, from the ground: a high cube's.
inline constexpr double kHeight = 2.896;

// The members, drawn as the 40 ft's, the deck thinner for the lower base.
inline constexpr double kSideRailWidth = 0.150;
inline constexpr double kEndSillLength = 0.181;
inline constexpr double kDeckThickness = 0.115;

// Masses, per rack.
inline constexpr double kTareMass = 2900.0;  // Hapag-Lloyd's 20' flatrack.
// What the robotics on one rack may weigh in transit: 30,000 lb (the owner).
inline constexpr double kTransitMassLimit = 13608.0;

// The nominal gap between the two racks' long sides.
inline constexpr double kNominalGap = 0.025;

// The footprint of the pair at a gap: half its width across and half its
// length along, about the origin.
inline double PairHalfWidth(double gap) { return gap / 2.0 + kWidth; }
inline constexpr double kPairHalfLength = kLength / 2.0;

// The middle of a rack's footprint on the ground at the nominal gap, rack
// "a" (side < 0) or "b" (side > 0), moved by (shift_x, shift_y).
inline Eigen::Vector2d RackCentre(bool b, double gap = kNominalGap,
                                  double shift_x = 0.0, double shift_y = 0.0) {
  const double x = gap / 2.0 + kWidth / 2.0;
  return b ? Eigen::Vector2d(x + shift_x, shift_y) : Eigen::Vector2d(-x, 0.0);
}

// The stow envelope of a rack and the part of it above the deck, at its
// place.
inline Box TransportEnvelope(bool b, double gap = kNominalGap,
                             double shift_x = 0.0, double shift_y = 0.0) {
  const Eigen::Vector2d c = RackCentre(b, gap, shift_x, shift_y);
  return {std::string("flat_rack_") + (b ? "b" : "a") + "_transport_envelope",
          {c.x(), c.y(), kHeight / 2.0},
          {kWidth / 2.0, kLength / 2.0, kHeight / 2.0}};
}
inline Box CellEnvelope(bool b, double gap = kNominalGap, double shift_x = 0.0,
                        double shift_y = 0.0) {
  const Eigen::Vector2d c = RackCentre(b, gap, shift_x, shift_y);
  return {std::string("flat_rack_") + (b ? "b" : "a") + "_cell_envelope",
          {c.x(), c.y(), (kDeckHeight + kHeight) / 2.0},
          {kWidth / 2.0, kLength / 2.0, (kHeight - kDeckHeight) / 2.0}};
}

// One rack's members at its place, named flat_rack_<a|b>_side_rail_0 and so
// on, as the 40 ft's with the rack's letter after "flat_rack_".
inline std::vector<Box> Members(bool b, double gap = kNominalGap,
                                double shift_x = 0.0, double shift_y = 0.0) {
  const Eigen::Vector2d c = RackCentre(b, gap, shift_x, shift_y);
  const std::string p = std::string("flat_rack_") + (b ? "b" : "a") + "_";
  const double hx = kWidth / 2.0;
  const double hy = kLength / 2.0;
  const double between_x = hx - kSideRailWidth;
  const double between_y = hy - kEndSillLength;
  std::vector<Box> all;
  int side = 0;
  for (double sx : {-1.0, 1.0}) {
    all.push_back({p + "side_rail_" + std::to_string(side++),
                   {c.x() + sx * (hx - kSideRailWidth / 2.0), c.y(),
                    kDeckHeight / 2.0},
                   {kSideRailWidth / 2.0, hy, kDeckHeight / 2.0}});
  }
  for (int e = 0; e < 2; ++e) {
    const double sy = e == 0 ? -1.0 : 1.0;
    all.push_back({p + "end_sill_" + (e == 0 ? "front" : "rear"),
                   {c.x(), c.y() + sy * (hy - kEndSillLength / 2.0),
                    kDeckHeight / 2.0},
                   {between_x, kEndSillLength / 2.0, kDeckHeight / 2.0}});
  }
  all.push_back({p + "deck",
                 {c.x(), c.y(), kDeckHeight - kDeckThickness / 2.0},
                 {between_x, between_y, kDeckThickness / 2.0}});
  return all;
}

}  // namespace twenty

}  // namespace cell::flat_rack

#endif  // SRC_FLAT_RACK_H_
