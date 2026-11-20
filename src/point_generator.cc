#include "src/point_generator.h"

#include <cmath>
#include <numbers>

#include <Eigen/Geometry>

namespace cell {
namespace {

double Degrees(double degrees) { return degrees * std::numbers::pi / 180.0; }

void Append(Points& to, const Points& from) {
  to.insert(to.end(), from.begin(), from.end());
}

}  // namespace

Points SampleRectangle(const Rectangle& rectangle, int count, double noise,
                       std::mt19937& rng) {
  std::uniform_real_distribution<double> along(-0.5, 0.5);
  std::normal_distribution<double> off(0.0, 1.0);

  Points points;
  points.reserve(count);
  for (int i = 0; i < count; ++i) {
    const Eigen::Vector3d local(along(rng) * rectangle.length,
                                along(rng) * rectangle.width,
                                off(rng) * noise);
    points.push_back(rectangle.centre + rectangle.axes * local);
  }
  return points;
}

Points SampleBox(const Eigen::Vector3d& low, const Eigen::Vector3d& high,
                 int count, std::mt19937& rng) {
  std::uniform_real_distribution<double> fraction(0.0, 1.0);

  Points points;
  points.reserve(count);
  for (int i = 0; i < count; ++i) {
    const Eigen::Vector3d f(fraction(rng), fraction(rng), fraction(rng));
    points.push_back(low + f.cwiseProduct(high - low));
  }
  return points;
}

Eigen::Matrix3d RandomRotation(std::mt19937& rng) {
  // Four normally distributed numbers, scaled to length 1, are a point
  // drawn evenly from the sphere of unit quaternions, and unit quaternions
  // cover the rotations evenly.
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::Quaterniond q(normal(rng), normal(rng), normal(rng), normal(rng));
  q.normalize();
  return q.toRotationMatrix();
}

PalletView MakePalletView(const PalletViewOptions& options,
                          std::mt19937& rng) {
  std::uniform_real_distribution<double> distance(1.5, 2.5);
  std::uniform_real_distribution<double> offset(-0.2, 0.2);
  std::uniform_real_distribution<double> lean(Degrees(5.0), Degrees(15.0));
  std::uniform_real_distribution<double> turn(Degrees(-5.0), Degrees(5.0));

  PalletView view;
  Rectangle& face = view.face;
  face.length = options.module_length;
  face.width = options.module_width;
  face.centre = Eigen::Vector3d(distance(rng), offset(rng), offset(rng));

  // Upright and square to the camera, the face has its length along Y, its
  // width along Z and its normal along X, away from the camera. Turning
  // about Y tips the top away from the camera. Turning about Z after that
  // swings the whole pallet to one side.
  Eigen::Matrix3d upright;
  upright.col(0) = Eigen::Vector3d::UnitY();
  upright.col(1) = Eigen::Vector3d::UnitZ();
  upright.col(2) = Eigen::Vector3d::UnitX();
  const Eigen::AngleAxisd lean_back(lean(rng), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd turn_aside(turn(rng), Eigen::Vector3d::UnitZ());
  face.axes = (turn_aside * lean_back).toRotationMatrix() * upright;

  view.points = SampleRectangle(face, options.face_points, options.noise, rng);

  // The module behind: the same rectangle one thickness farther along the
  // normal. In the cell the front module hides most of it.
  Rectangle behind = face;
  behind.centre += options.module_thickness * face.axes.col(2);
  Append(view.points, SampleRectangle(behind, options.behind_points,
                                      options.noise, rng));

  // A level patch of floor below the pallet.
  Rectangle floor;
  floor.centre = Eigen::Vector3d(face.centre.x(), 0.0,
                                 face.centre.z() - options.module_width / 2.0 -
                                     0.2);
  floor.length = 3.0;
  floor.width = 3.0;
  Append(view.points,
         SampleRectangle(floor, options.floor_points, options.noise, rng));

  // The rest of the room.
  Append(view.points, SampleBox(Eigen::Vector3d(0.5, -2.0, -1.0),
                                Eigen::Vector3d(4.0, 2.0, 2.0),
                                options.clutter_points, rng));
  return view;
}

}  // namespace cell
