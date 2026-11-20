// Made-up point clouds with known answers, for testing the plane fit before
// there is a simulated camera to supply real ones.
//
// The pieces: points scattered on a rectangle, with noise; points scattered
// through a box, as clutter; and a random orientation. MakePalletView puts
// them together into roughly what the depth camera will see in the cell.
//
// Lengths are metres and angles radians. Every function draws from the
// random number generator it is given, so a fixed seed repeats the cloud.

#ifndef SRC_POINT_GENERATOR_H_
#define SRC_POINT_GENERATOR_H_

#include <random>

#include <Eigen/Core>

#include "src/plane_fit.h"

namespace cell {

// A flat rectangle somewhere in space.
struct Rectangle {
  Eigen::Vector3d centre = Eigen::Vector3d::Zero();

  // A rotation matrix. Its columns are unit vectors along the rectangle's
  // length, along its width, and at right angles to its surface.
  Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();

  double length = 1.0;
  double width = 1.0;

  // The plane the rectangle lies in.
  Plane plane() const { return PlaneThrough(axes.col(2), centre); }
};

// `count` points spread evenly at random over the rectangle. Each is then
// moved off the surface, along the normal, by a normally distributed amount
// with standard deviation `noise`. Noise along the surface is not modelled:
// it would not move a point off the plane.
Points SampleRectangle(const Rectangle& rectangle, int count, double noise,
                       std::mt19937& rng);

// `count` points spread evenly at random through the box between two
// opposite corners.
Points SampleBox(const Eigen::Vector3d& low, const Eigen::Vector3d& high,
                 int count, std::mt19937& rng);

// A rotation matrix drawn evenly from all orientations.
Eigen::Matrix3d RandomRotation(std::mt19937& rng);

// What the depth camera sees when it looks at the pallet, roughly. The
// camera is at the origin looking along +X, with +Y to its left and +Z up.
struct PalletViewOptions {
  int face_points = 2000;     // on the front module's face
  int behind_points = 200;    // on the module behind, one thickness back
  int floor_points = 600;     // on the floor under the pallet
  int clutter_points = 800;   // anywhere in the room

  // Standard deviation of the depth noise on every surface.
  double noise = 0.002;

  // Size of a module's face, and how far the next face is behind it.
  double module_length = 2.3;
  double module_width = 1.1;
  double module_thickness = 0.035;
};

struct PalletView {
  // The front module's face, which the fit should find. Its long side is
  // level. It stands 1.5 to 2.5 m from the camera, leans back by 5 to 15
  // degrees, and is turned up to 5 degrees to either side, as a pallet set
  // down by a forklift would be.
  Rectangle face;

  // Everything the camera sees. The first options.face_points are on the
  // face; the rest are not.
  Points points;
};

PalletView MakePalletView(const PalletViewOptions& options, std::mt19937& rng);

}  // namespace cell

#endif  // SRC_POINT_GENERATOR_H_
