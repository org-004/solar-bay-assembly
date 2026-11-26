// The shipping container the cell is built into, in its working state: the
// members that stay when its long sides are open, and the two envelopes a
// layout has to respect.
//
// docs/bay-hardware.md, section "The container", is the source of every
// figure here. It gives what a 40 ft container is to a truck, a ship and a
// crane from public sources, and then states the container this project
// assumes, which is designed here and is not a structural calculation.
// models/container.xml is the same container as a MuJoCo model, and
// tests/container_test.cc checks that the two agree.
//
// The container. It has the outside of a 40 ft high-cube container. Four
// corner posts carry the eight corner fittings by which it is lifted,
// stacked and locked to a chassis. Two deep bottom side rails and the floor
// between them carry the cell; two top side rails and the roof close the
// top. The front end is a closed wall and the rear end a door frame. Both
// long sides are open between the posts, with no post in between, and the
// roof stays closed.
//
// Two envelopes. The transport envelope is the container's outside:
// everything must be inside it when the container travels. The inside
// envelope is the space a layout may count on between the posts, above the
// floor and under the top side rails, which stays clear when the sides are
// closed for transport. Both envelopes bind the stowed cell only: where a
// bay is built, and whether it overhangs the base or stands beside it, is a
// layout's choice and nothing here limits it.
//
// The frame: the origin is on the ground under the middle of the container,
// +Y along the container toward the rear (door) end, +Z up, +X across.
// Metres and kilograms. models/container.xml places this frame at the
// world's origin as its body "container_frame".

#ifndef SRC_CONTAINER_H_
#define SRC_CONTAINER_H_

#include <string>
#include <vector>

#include <Eigen/Core>

namespace cell::container {

// ------------------------------------------------------------- the outside
// A 40 ft high-cube container: published (ISO 668).
inline constexpr double kLength = 12.192;  // along Y
inline constexpr double kWidth = 2.438;    // along X
inline constexpr double kHeight = 2.896;   // along Z

// The corner fittings' centres, apart along the container and across it:
// published. A chassis and a spreader lock onto these.
inline constexpr double kFittingSpacingLength = 11.985;
inline constexpr double kFittingSpacingWidth = 2.259;

// ------------------------------------------------------------- the members
// A corner post, along the container and across it.
inline constexpr double kPostLength = 0.181;
inline constexpr double kPostWidth = 0.180;
// A bottom side rail; its top is level with the floor.
inline constexpr double kBottomRailWidth = 0.150;
inline constexpr double kBottomRailDepth = 0.450;
// The floor's top above the ground the container stands on, and the
// thickness of the floor.
inline constexpr double kFloorHeight = 0.450;
inline constexpr double kFloorThickness = 0.150;
// A top side rail and the roof. The roof's top is 6 mm below the top of the
// corner fittings.
inline constexpr double kTopRailWidth = 0.100;
inline constexpr double kTopRailDepth = 0.200;
inline constexpr double kRoofTop = 2.890;
inline constexpr double kRoofThickness = 0.050;
inline constexpr double kTopRailUnderside = kRoofTop - kTopRailDepth;
// The closed wall of the front end.
inline constexpr double kFrontWallThickness = 0.040;

// ------------------------------------------------------------ the openings
// Each long side is open between the posts, from the floor to the top side
// rail.
inline constexpr double kSideOpeningLength = kLength - 2.0 * kPostLength;
inline constexpr double kSideOpeningHeight = kTopRailUnderside - kFloorHeight;
// Nothing may pass through the roof.
inline constexpr double kRoofOpeningLength = 0.0;

// ----------------------------------------------------------- the envelopes
// The inside envelope: between the posts, the least inside width ISO 668
// allows, from the floor to the top side rails.
inline constexpr double kInsideLength = kSideOpeningLength;
inline constexpr double kInsideWidth = 2.330;
inline constexpr double kInsideHeight = kSideOpeningHeight;

// --------------------------------------------------------------- the masses
inline constexpr double kTareMass = 6600.0;
// The most the loaded container may weigh on the road.
inline constexpr double kTransportGrossMass = 23800.0;
inline constexpr double kCellMassBudget = kTransportGrossMass - kTareMass;
// The height of a chassis' deck, which the floor's height adds to when the
// container is on the road.
inline constexpr double kChassisDeckHeight = 1.219;

// A box in the container's frame: its centre and half its size along X, Y
// and Z, as MuJoCo's box takes them.
struct Box {
  std::string name;
  Eigen::Vector3d centre = Eigen::Vector3d::Zero();
  Eigen::Vector3d half_size = Eigen::Vector3d::Zero();
};

// The model's two sites.
inline Box TransportEnvelope() {
  return {"container_transport_envelope",
          {0.0, 0.0, kHeight / 2.0},
          {kWidth / 2.0, kLength / 2.0, kHeight / 2.0}};
}
inline Box InsideEnvelope() {
  return {"container_inside_envelope",
          {0.0, 0.0, kFloorHeight + kInsideHeight / 2.0},
          {kInsideWidth / 2.0, kInsideLength / 2.0, kInsideHeight / 2.0}};
}

// The members of the working state, each a geom of the model with the same
// name: four posts, two bottom side rails, the floor and the sills that
// continue it between the posts at each end, two top side rails, the roof,
// the headers of the two ends, and the front wall. The front end is at -Y.
inline std::vector<Box> Members() {
  const double hx = kWidth / 2.0;
  const double hy = kLength / 2.0;
  const double post_x = hx - kPostWidth / 2.0;
  const double post_y = hy - kPostLength / 2.0;
  // Half the distance between the posts, along the container and across it.
  const double between_y = hy - kPostLength;
  const double between_x = hx - kPostWidth;
  std::vector<Box> all;
  int post = 0;
  for (double sy : {-1.0, 1.0}) {
    for (double sx : {-1.0, 1.0}) {
      all.push_back({"container_post_" + std::to_string(post++),
                     {sx * post_x, sy * post_y, kHeight / 2.0},
                     {kPostWidth / 2.0, kPostLength / 2.0, kHeight / 2.0}});
    }
  }
  int side = 0;
  for (double sx : {-1.0, 1.0}) {
    all.push_back(
        {"container_bottom_rail_" + std::to_string(side),
         {sx * (hx - kBottomRailWidth / 2.0), 0.0, kBottomRailDepth / 2.0},
         {kBottomRailWidth / 2.0, between_y, kBottomRailDepth / 2.0}});
    all.push_back(
        {"container_top_rail_" + std::to_string(side),
         {sx * (hx - kTopRailWidth / 2.0), 0.0,
          kTopRailUnderside + kTopRailDepth / 2.0},
         {kTopRailWidth / 2.0, between_y, kTopRailDepth / 2.0}});
    ++side;
  }
  all.push_back({"container_floor",
                 {0.0, 0.0, kFloorHeight - kFloorThickness / 2.0},
                 {hx - kBottomRailWidth, between_y, kFloorThickness / 2.0}});
  all.push_back({"container_roof",
                 {0.0, 0.0, kRoofTop - kRoofThickness / 2.0},
                 {hx - kTopRailWidth, between_y, kRoofThickness / 2.0}});
  const char* ends[] = {"front", "rear"};
  int end = 0;
  for (double sy : {-1.0, 1.0}) {
    all.push_back({std::string("container_sill_") + ends[end],
                   {0.0, sy * post_y, kFloorHeight / 2.0},
                   {between_x, kPostLength / 2.0, kFloorHeight / 2.0}});
    all.push_back({std::string("container_header_") + ends[end],
                   {0.0, sy * post_y, kTopRailUnderside + kTopRailDepth / 2.0},
                   {between_x, kPostLength / 2.0, kTopRailDepth / 2.0}});
    ++end;
  }
  all.push_back({"container_front_wall",
                 {0.0, -(hy - kFrontWallThickness / 2.0),
                  (kFloorHeight + kTopRailUnderside) / 2.0},
                 {between_x, kFrontWallThickness / 2.0,
                  (kTopRailUnderside - kFloorHeight) / 2.0}});
  return all;
}

}  // namespace cell::container

#endif  // SRC_CONTAINER_H_
