// Checks the pinhole camera model and the depth image: projecting and the
// ray through a pixel undo each other; the image of the cell matches the
// true geometry of the front module's glass to a fraction of a millimetre;
// every point goes back to its own pixel and lands on the object it should;
// the image is repeatable and follows the scene; the noise model has the
// statistics it promises; a calibration error moves the points as it
// should; and misses and readings beyond range are NaN.
//
//   bazel test //:depth_camera_test
//   bazel test --config=opt //:depth_camera_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/cell.xml).
//
// A full 640 x 480 image takes about a second unoptimized, so the suite
// loads and resets the cell once, takes the reference image once (nominal
// pallet, arm at arm_clear) and shares both. Tests that change the scene
// change a copy of mjData, never the shared one.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/cell.h"
#include "src/depth_camera.h"
#include "src/mj_handles.h"
#include "src/pallet.h"
#include "src/plane_fit.h"

namespace {

using cell::Cell;
using cell::DepthImage;

constexpr double kPi = 3.14159265358979323846;

// The same bits: NaN equals NaN, which == on floats would deny.
bool SameImage(const DepthImage& a, const DepthImage& b) {
  return a.width == b.width && a.height == b.height &&
         a.depth.size() == b.depth.size() &&
         std::memcmp(a.depth.data(), b.depth.data(),
                     a.depth.size() * sizeof(float)) == 0;
}

bool SameReading(float a, float b) {
  return std::memcmp(&a, &b, sizeof(float)) == 0;
}

// A module's glass face as the truth has it: a rectangle in a plane.
struct Glass {
  Eigen::Vector3d centre;
  Eigen::Vector3d normal;  // out of the glass
  Eigen::Vector3d along_x;  // the long side
  Eigen::Vector3d along_y;  // the short side
};

Glass TrueGlass(const Cell& cell, const mjData& data, int i) {
  const cell::GlassFace face = cell::ModuleGlass(data, cell.pallet(), i);
  const cell::WorldPose pose = cell::ModulePose(data, cell.pallet(), i);
  return {face.centre, face.normal,
          pose.orientation * Eigen::Vector3d::UnitX(),
          pose.orientation * Eigen::Vector3d::UnitY()};
}

// Where the ray through the centre of pixel (u, v) meets the glass's plane,
// as the depth along the viewing axis, and whether that point is inside the
// glass rectangle shrunk by `margin` on every side.
struct RayOnGlass {
  double depth = 0.0;
  bool inside = false;
};

RayOnGlass IntersectGlass(const cell::CameraModel& camera, const Glass& glass,
                          int u, int v, double margin) {
  const Eigen::Vector3d origin = camera.pose.translation();
  const Eigen::Vector3d ray = cell::RayThroughPixel(camera, u + 0.5, v + 0.5);
  const double t =
      glass.normal.dot(glass.centre - origin) / glass.normal.dot(ray);
  const Eigen::Vector3d point = origin + t * ray;
  const Eigen::Vector3d local = point - glass.centre;
  RayOnGlass result;
  result.depth = -(camera.pose.inverse() * point).z();
  result.inside =
      t > 0 &&
      std::abs(local.dot(glass.along_x)) < cell::kModuleLength / 2 - margin &&
      std::abs(local.dot(glass.along_y)) < cell::kModuleWidth / 2 - margin;
  return result;
}

// A copy of `data` to change freely.
cell::DataPtr CopyData(const mjModel& model, const mjData& data) {
  return cell::DataPtr(mj_copyData(nullptr, &model, &data));
}

class DepthCameraTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const char* path = std::getenv("MODEL_PATH");
    if (path == nullptr) {
      load_error_ = "MODEL_PATH is not set";
      return;
    }
    std::optional<Cell> loaded = Cell::Load(path, &load_error_);
    if (!loaded) return;
    cell_.emplace(std::move(*loaded));
    cell_->Reset(0, cell::Randomization::None());
    cell_->PutArm(cell_->arm_clear());  // also runs mj_forward
    camera_ = cell_->DepthCamera();

    const auto start = std::chrono::steady_clock::now();
    reference_ = cell::RenderDepth(*cell_->model(), cell_->data(), camera_, {},
                                   &reference_geom_);
    const double seconds = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - start)
                               .count();
    std::printf("render: %.1f ms per %d x %d image\n", seconds * 1e3,
                reference_.width, reference_.height);
  }

  static void TearDownTestSuite() { cell_.reset(); }

  void SetUp() override { ASSERT_TRUE(cell_.has_value()) << load_error_; }

  static Cell& cell() { return *cell_; }
  static const mjModel& model() { return *cell_->model(); }
  static const mjData& data() { return *cell_->data(); }

  static int ModuleGeom(int i) {
    return model().body_geomadr[cell().pallet().modules[i]];
  }

  // Whether `geom` belongs to the arm or its tool: a body under arm_base.
  static bool IsArm(int geom) {
    const int base = mj_name2id(&model(), mjOBJ_BODY, "arm_base");
    for (int b = model().geom_bodyid[geom]; b > 0; b = model().body_parentid[b]) {
      if (b == base) return true;
    }
    return false;
  }

  // Checks the pixels on module i's glass in `image`, taken of `scene`: every
  // pixel whose centre ray meets the true glass at least 1 cm inside its
  // edges reads the glass, at the true depth, and back-projects onto the
  // glass's plane within 0.1 mm. Returns how many pixels' centre rays meet
  // the glass anywhere on it.
  static int CheckGlass(const mjData& scene, const DepthImage& image,
                        const std::vector<int>& geoms, int i) {
    const Glass glass = TrueGlass(cell(), scene, i);
    int on_glass = 0;
    int checked = 0;
    int failures = 0;
    for (int v = 0; v < image.height; ++v) {
      for (int u = 0; u < image.width; ++u) {
        if (IntersectGlass(camera_, glass, u, v, 0.0).inside) ++on_glass;
        const RayOnGlass truth = IntersectGlass(camera_, glass, u, v, 0.01);
        if (!truth.inside) continue;
        ++checked;
        const int k = image.index(u, v);
        const Eigen::Vector3d point =
            camera_.pose * (static_cast<double>(image.depth[k]) *
                            cell::PointAtUnitDepth(camera_, u + 0.5, v + 0.5));
        const double off_plane = glass.normal.dot(point - glass.centre);
        const bool good = geoms[k] == ModuleGeom(i) && image.has_reading(u, v) &&
                          std::abs(image.depth[k] - truth.depth) < 2e-6 &&
                          std::abs(off_plane) < 0.1e-3;
        if (!good && ++failures <= 5) {
          ADD_FAILURE() << "module_" << i << " pixel (" << u << ", " << v
                        << "): geom " << geoms[k] << ", depth "
                        << image.depth[k] << " against " << truth.depth
                        << ", " << off_plane * 1e3 << " mm off the plane";
        }
      }
    }
    EXPECT_EQ(failures, 0) << "of " << checked << " pixels on module_" << i;
    EXPECT_GT(checked, 10000) << "module_" << i;
    return on_glass;
  }

  static inline std::optional<Cell> cell_;
  static inline std::string load_error_;
  static inline cell::CameraModel camera_;
  static inline DepthImage reference_;
  static inline std::vector<int> reference_geom_;
};

// ------------------------------------------------------- the camera model

TEST_F(DepthCameraTest, ProjectAndRayThroughPixelAreInverses) {
  const cell::CameraModel& camera = camera_;
  const Eigen::Vector3d origin = camera.pose.translation();

  // The principal ray is the camera's -Z.
  const Eigen::Vector3d principal =
      cell::RayThroughPixel(camera, camera.cx, camera.cy);
  EXPECT_LT((principal + camera.pose.linear().col(2)).norm(), 1e-12);

  // The top and bottom edges of the image are fovy apart.
  const double angle = std::acos(
      cell::RayThroughPixel(camera, camera.cx, 0.0)
          .dot(cell::RayThroughPixel(camera, camera.cx, camera.height)));
  EXPECT_NEAR(angle, camera.fovy, 1e-12);

  // Points along the ray through (u, v) project back to (u, v), corners and
  // edges included; v down means a ray through the top row points up.
  for (double u : {0.0, 0.5, 100.25, 320.0, 639.5, 640.0}) {
    for (double v : {0.0, 0.5, 77.75, 240.0, 479.5, 480.0}) {
      const Eigen::Vector3d ray = cell::RayThroughPixel(camera, u, v);
      EXPECT_NEAR(ray.norm(), 1.0, 1e-12);
      for (double t : {0.5, 4.3, 50.0}) {
        const std::optional<Eigen::Vector2d> pixel =
            cell::Project(camera, origin + t * ray);
        ASSERT_TRUE(pixel);
        EXPECT_NEAR(pixel->x(), u, 1e-9);
        EXPECT_NEAR(pixel->y(), v, 1e-9);
      }
    }
  }
  EXPECT_GT((camera.pose.inverse().linear() *
             cell::RayThroughPixel(camera, camera.cx, 0.0))
                .y(),
            0.0);

  // And the other way: the ray through where a point projects passes
  // through the point.
  std::uint32_t state = 12345;
  auto unit = [&] {
    state = state * 1664525u + 1013904223u;
    return (state >> 8) / 16777216.0;
  };
  for (int n = 0; n < 200; ++n) {
    const Eigen::Vector3d in_camera(4 * unit() - 2, 4 * unit() - 2,
                                    -(0.2 + 8 * unit()));
    const Eigen::Vector3d world = camera.pose * in_camera;
    const std::optional<Eigen::Vector2d> pixel = cell::Project(camera, world);
    ASSERT_TRUE(pixel);
    const Eigen::Vector3d ray =
        cell::RayThroughPixel(camera, pixel->x(), pixel->y());
    EXPECT_LT((ray - (world - origin).normalized()).norm(), 1e-12);
  }

  // Behind the camera, or level with it: nothing.
  EXPECT_FALSE(cell::Project(camera, origin - principal));
  EXPECT_FALSE(cell::Project(camera, origin + camera.pose.linear().col(0)));
}

TEST_F(DepthCameraTest, CameraModelFromMujocoReadsTheCellsCamera) {
  const std::optional<cell::CameraModel> named =
      cell::CameraModelFromMujoco(model(), data(), "depth_camera");
  ASSERT_TRUE(named);
  EXPECT_TRUE(named->pose.isApprox(camera_.pose, 0.0));
  EXPECT_EQ(named->width, 640);
  EXPECT_EQ(named->height, 480);
  EXPECT_NEAR(named->focal, 240.0 / std::tan(20.0 * kPi / 180.0), 1e-9);
  EXPECT_EQ(named->cx, 320.0);
  EXPECT_EQ(named->cy, 240.0);
  // Where models/cell.xml puts it.
  EXPECT_LT((named->pose.translation() - Eigen::Vector3d(0, 0, 4)).norm(),
            1e-9);
  EXPECT_FALSE(cell::CameraModelFromMujoco(model(), data(), "no_such_camera"));
  EXPECT_FALSE(cell::CameraModelFromMujoco(model(), data(), model().ncam));
  EXPECT_FALSE(cell::CameraModelFromMujoco(model(), data(), -1));
}

// ------------------------------------------- the image against the truth

// The depth at the pixels where module_0's glass centre and corners project
// is the true depth of the glass at those pixels' centres, and every pixel
// on the glass back-projects onto the true glass plane.
TEST_F(DepthCameraTest, DepthMatchesTheTrueGlass) {
  const Glass glass = TrueGlass(cell(), data(), 0);
  const double in_x = cell::kModuleLength / 2 - 0.01;
  const double in_y = cell::kModuleWidth / 2 - 0.01;
  for (const auto& [a, b] : std::vector<std::pair<double, double>>{
           {0, 0}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}) {
    const Eigen::Vector3d point =
        glass.centre + a * in_x * glass.along_x + b * in_y * glass.along_y;
    const std::optional<Eigen::Vector2d> pixel = cell::Project(camera_, point);
    ASSERT_TRUE(pixel);
    const int u = static_cast<int>(std::floor(pixel->x()));
    const int v = static_cast<int>(std::floor(pixel->y()));
    ASSERT_TRUE(u >= 0 && u < camera_.width && v >= 0 && v < camera_.height);
    const RayOnGlass truth = IntersectGlass(camera_, glass, u, v, 0.0);
    ASSERT_TRUE(truth.inside);
    EXPECT_EQ(reference_geom_[reference_.index(u, v)], ModuleGeom(0));
    EXPECT_NEAR(reference_.at(u, v), truth.depth, 2e-6)
        << "corner " << a << " " << b << " at pixel " << u << ", " << v;
    // The point itself is up to half a pixel from the pixel's centre, where
    // the depth was read: a few millimetres at most on this slanted glass.
    const double point_depth = -(camera_.pose.inverse() * point).z();
    EXPECT_NEAR(reference_.at(u, v), point_depth, 0.01);
  }

  const int on_glass = CheckGlass(data(), reference_, reference_geom_, 0);
  std::printf("full pallet: %d pixels on module_0's glass\n", on_glass);
}

// The same with only the last module left, the farthest the front glass
// gets.
TEST_F(DepthCameraTest, DepthMatchesTheTrueGlassOfTheLastModule) {
  cell::DataPtr scene = CopyData(model(), data());
  for (int i = 0; i + 1 < cell().module_count(); ++i) {
    const int joint = model().body_jntadr[cell().pallet().modules[i]];
    mjtNum* qpos = scene->qpos + model().jnt_qposadr[joint];
    const Eigen::Vector3d parked = cell::ParkingPose(i).translation();
    qpos[0] = parked.x();
    qpos[1] = parked.y();
    qpos[2] = parked.z();
    qpos[3] = 1;  // flat, glass up: ParkingPose's orientation
    qpos[4] = qpos[5] = qpos[6] = 0;
  }
  mj_forward(&model(), scene.get());
  std::vector<int> geoms;
  const DepthImage image =
      cell::RenderDepth(model(), scene.get(), camera_, {}, &geoms);
  const int on_glass = CheckGlass(*scene, image, geoms, 29);
  std::printf("only module_29 left: %d pixels on its glass\n", on_glass);
}

// What the next stage has to know about the front glass's outline, with a
// full pallet: its top edge runs on into the tops of the modules behind and
// its bottom edge into the pallet's deck, so both are creases, where the
// surface turns, with no step in depth. Only its two ends are steps, down to
// the floor.
TEST_F(DepthCameraTest, FrontGlassTopAndBottomAreCreasesItsEndsAreSteps) {
  const Glass glass = TrueGlass(cell(), data(), 0);
  const int u = static_cast<int>(camera_.cx);
  int first = -1, last = -1;
  for (int v = 0; v < camera_.height; ++v) {
    if (IntersectGlass(camera_, glass, u, v, 0.0).inside) {
      if (first < 0) first = v;
      last = v;
    }
  }
  ASSERT_GE(first, 10);
  ASSERT_LT(last, camera_.height - 10);
  double largest = 0.0;
  for (int v = first - 10; v < last + 10; ++v) {
    ASSERT_TRUE(reference_.has_reading(u, v) && reference_.has_reading(u, v + 1));
    largest = std::max(
        largest, static_cast<double>(std::abs(reference_.at(u, v + 1) -
                                              reference_.at(u, v))));
  }
  std::printf("module_0's glass at column %d: rows %d to %d, largest step "
              "between rows across its top and bottom edges %.1f mm\n",
              u, first, last, largest * 1e3);
  EXPECT_LT(largest, 0.01);

  // Along the row through the glass's middle, the ends drop to the floor.
  const int v = (first + last) / 2;
  int left = -1, right = -1;
  for (int k = 0; k < camera_.width; ++k) {
    if (IntersectGlass(camera_, glass, k, v, 0.0).inside) {
      if (left < 0) left = k;
      right = k;
    }
  }
  ASSERT_GT(left, 0);
  ASSERT_LT(right, camera_.width - 1);
  const double left_step = reference_.at(left - 1, v) - reference_.at(left, v);
  const double right_step =
      reference_.at(right + 1, v) - reference_.at(right, v);
  std::printf("  and at row %d its ends step %.3f m and %.3f m deeper\n", v,
              left_step, right_step);
  EXPECT_GT(left_step, 0.5);
  EXPECT_GT(right_step, 0.5);
}

// Every point re-projects to its own pixel's centre, in either frame.
TEST_F(DepthCameraTest, EveryPointReprojectsToItsOwnPixel) {
  const cell::PixelPoints world = cell::PointsWithPixels(reference_, camera_);
  const cell::PixelPoints local =
      cell::PointsWithPixels(reference_, camera_, cell::PointFrame::kCamera);
  ASSERT_EQ(static_cast<int>(world.points.size()), reference_.CountReadings());
  ASSERT_EQ(world.pixels, local.pixels);
  ASSERT_EQ(cell::PointsFromDepth(reference_, camera_), world.points);
  EXPECT_GT(reference_.CountReadings(), 200000);
  double worst = 0.0;
  for (std::size_t k = 0; k < world.points.size(); ++k) {
    const int u = world.pixels[k] % reference_.width;
    const int v = world.pixels[k] / reference_.width;
    const std::optional<Eigen::Vector2d> pixel =
        cell::Project(camera_, world.points[k]);
    ASSERT_TRUE(pixel);
    worst = std::max(worst, (*pixel - Eigen::Vector2d(u + 0.5, v + 0.5)).norm());
    EXPECT_LT((camera_.pose * local.points[k] - world.points[k]).norm(), 1e-9);
    EXPECT_FLOAT_EQ(static_cast<float>(-local.points[k].z()),
                    reference_.at(u, v));
  }
  EXPECT_LT(worst, 1e-6) << "pixels";
}

// Pixels that should see the floor, the pallet's deck and the glass do, and
// their points are where those surfaces are.
TEST_F(DepthCameraTest, PointsLandOnTheRightObjects) {
  const cell::WorldPose pallet = cell::PalletPose(data(), cell().pallet());
  Eigen::Isometry3d to_pallet = Eigen::Isometry3d::Identity();
  to_pallet.translation() = pallet.position;
  to_pallet.linear() = pallet.orientation.toRotationMatrix();
  // The deck's frame, as models/pallet.xml has it: on the deck's surface
  // under module_0's glass's bottom edge, tilted 10 degrees.
  Eigen::Isometry3d deck = Eigen::Isometry3d::Identity();
  deck.translation() = Eigen::Vector3d(-0.095506, 0, 0.208356);
  deck.linear() = Eigen::AngleAxisd(0.174533, Eigen::Vector3d::UnitY())
                      .toRotationMatrix();
  const Eigen::Isometry3d deck_in_world = to_pallet * deck;

  struct Sample {
    const char* what;
    Eigen::Vector3d point;
    Eigen::Vector3d normal;  // of the surface there
    int geom;
  };
  const int floor = mj_name2id(&model(), mjOBJ_GEOM, "floor");
  const int deck_geom = mj_name2id(&model(), mjOBJ_GEOM, "pallet_deck");
  const Glass glass = TrueGlass(cell(), data(), 0);
  const std::vector<Sample> samples = {
      {"floor in front of the pallet", {1.3, 0.0, 0.0}, {0, 0, 1}, floor},
      {"floor left of the pallet", {2.2, 1.6, 0.0}, {0, 0, 1}, floor},
      {"floor right of the pallet", {2.2, -1.6, 0.0}, {0, 0, 1}, floor},
      {"deck in front of module_0", deck_in_world * Eigen::Vector3d(-0.05, 0.6, 0),
       deck_in_world.linear().col(2), deck_geom},
      {"glass of module_0", glass.centre + 0.3 * glass.along_x, glass.normal,
       ModuleGeom(0)},
  };
  for (const Sample& sample : samples) {
    const std::optional<Eigen::Vector2d> pixel =
        cell::Project(camera_, sample.point);
    ASSERT_TRUE(pixel) << sample.what;
    const int u = static_cast<int>(std::floor(pixel->x()));
    const int v = static_cast<int>(std::floor(pixel->y()));
    ASSERT_TRUE(u >= 0 && u < camera_.width && v >= 0 && v < camera_.height)
        << sample.what;
    EXPECT_EQ(reference_geom_[reference_.index(u, v)], sample.geom)
        << sample.what << " at pixel " << u << ", " << v;
    ASSERT_TRUE(reference_.has_reading(u, v)) << sample.what;
    const Eigen::Vector3d seen =
        camera_.pose * (static_cast<double>(reference_.at(u, v)) *
                        cell::PointAtUnitDepth(camera_, u + 0.5, v + 0.5));
    // On the surface, and within a pixel or two of the point aimed at.
    EXPECT_LT(std::abs(sample.normal.dot(seen - sample.point)), 0.1e-3)
        << sample.what;
    EXPECT_LT((seen - sample.point).norm(), 0.03) << sample.what;
  }
}

// mj_multiRay leaves out geoms by bounding angles before casting; it must
// agree with mj_ray, which tries every geom, and the camera never sees the
// arm's hidden collision meshes.
TEST_F(DepthCameraTest, MultiRayAgreesWithMjRayAndSeesTheRightGroups) {
  const cell::DepthOptions options;
  const Eigen::Vector3d origin = camera_.pose.translation();
  int compared = 0;
  for (int v = 0; v < camera_.height; v += 7) {
    for (int u = 0; u < camera_.width; u += 7) {
      const Eigen::Vector3d ray = cell::RayThroughPixel(camera_, u + 0.5, v + 0.5);
      int geom = -1;
      const mjtNum range =
          mj_ray(&model(), &data(), origin.data(), ray.data(),
                 options.geom_groups.data(), 1, -1, &geom, nullptr);
      const int k = reference_.index(u, v);
      if (!reference_.has_reading(u, v)) {
        // Beyond max_depth or nothing hit.
        if (geom >= 0) {
          EXPECT_GT(range * (camera_.pose.inverse().linear() * ray).z() * -1,
                    options.max_depth);
        }
        continue;
      }
      ++compared;
      EXPECT_EQ(geom, reference_geom_[k]) << "pixel " << u << ", " << v;
      const double depth = range * -(camera_.pose.inverse().linear() * ray).z();
      EXPECT_NEAR(depth, reference_.depth[k], 2e-6) << "pixel " << u << ", " << v;
    }
  }
  EXPECT_GT(compared, 4000);
  for (int geom : reference_geom_) {
    if (geom >= 0) {
      EXPECT_LT(model().geom_group[geom], 3);
    }
  }
}

// ------------------------------------------- repeatable, follows the scene

TEST_F(DepthCameraTest, ImageIsRepeatableAndFollowsTheScene) {
  // The same state, the same image, bit for bit.
  const DepthImage again = cell::RenderDepth(model(), cell().data(), camera_);
  EXPECT_TRUE(SameImage(again, reference_));

  // The pallet moved 2 cm along X: the stack's pixels change.
  {
    cell::DataPtr scene = CopyData(model(), data());
    cell::PlacePallet(model(), scene.get(), cell().pallet(), 0.02, 0.0, 0.0);
    const DepthImage moved = cell::RenderDepth(model(), scene.get(), camera_);
    int changed = 0;
    for (int k = 0; k < moved.size(); ++k) {
      if (!(std::abs(moved.depth[k] - reference_.depth[k]) < 1e-3)) ++changed;
    }
    std::printf("pallet moved 2 cm: %d pixels changed by more than 1 mm\n",
                changed);
    EXPECT_GT(changed, 50000);
  }

  // The arm at arm_home: the tool hangs in front of the stack, and some
  // pixels on the glass now read the arm, nearer.
  {
    cell::DataPtr scene = CopyData(model(), data());
    for (int j = 0; j < cell::Arm::kJoints; ++j) {
      scene->qpos[cell().arm().qpos[j]] = cell().arm_home()[j];
    }
    mj_forward(&model(), scene.get());
    std::vector<int> geoms;
    const DepthImage home =
        cell::RenderDepth(model(), scene.get(), camera_, {}, &geoms);
    const Glass glass = TrueGlass(cell(), data(), 0);
    int on_glass = 0;
    int blocked = 0;
    int arm_drawn = 0;
    for (int v = 0; v < home.height; ++v) {
      for (int u = 0; u < home.width; ++u) {
        const int k = home.index(u, v);
        if (geoms[k] >= 0 && IsArm(geoms[k]) && model().geom_group[geoms[k]] == 2) {
          ++arm_drawn;
        }
        if (!IntersectGlass(camera_, glass, u, v, 0.01).inside) continue;
        ++on_glass;
        if (home.depth[k] < reference_.depth[k] - 0.01) {
          ++blocked;
          EXPECT_TRUE(IsArm(geoms[k])) << "pixel " << u << ", " << v;
        }
      }
    }
    std::printf("arm at arm_home: %d of %d glass pixels see the arm or tool, "
                "%d pixels see the arm's drawn meshes\n",
                blocked, on_glass, arm_drawn);
    EXPECT_GT(blocked, 100);
    EXPECT_GT(arm_drawn, 0);
  }
}

// ------------------------------------------------------------- the noise

TEST_F(DepthCameraTest, DepthNoiseHasTheModelsSpread) {
  cell::DepthNoise noise;
  noise.depth_noise = true;
  noise.seed = 7;
  DepthImage noisy = reference_;
  cell::AddDepthNoise(&noisy, noise);

  // Binned by depth, 25 cm at a time: in each bin the spread of the error
  // is a + b z^2, within what chance allows for the bin's pixel count.
  constexpr double kBin = 0.25;
  std::vector<double> sum(40), sum_sq(40), expected_sq(40);
  std::vector<int> count(40);
  for (int k = 0; k < noisy.size(); ++k) {
    const float clean = reference_.depth[k];
    ASSERT_EQ(std::isfinite(clean), std::isfinite(noisy.depth[k]));
    if (!std::isfinite(clean)) continue;
    const int bin = static_cast<int>(clean / kBin);
    const double error = static_cast<double>(noisy.depth[k]) - clean;
    const double sigma = noise.sigma_a + noise.sigma_b * clean * clean;
    sum[bin] += error / sigma;
    sum_sq[bin] += error * error;
    expected_sq[bin] += sigma * sigma;
    ++count[bin];
  }
  int bins = 0;
  for (int bin = 0; bin < 40; ++bin) {
    if (count[bin] < 5000) continue;
    ++bins;
    const double measured = std::sqrt(sum_sq[bin] / count[bin]);
    const double model_sigma = std::sqrt(expected_sq[bin] / count[bin]);
    // The standard deviation measured from n draws is off by about
    // 1 / sqrt(2 n) of itself; allow five times that.
    EXPECT_NEAR(measured / model_sigma, 1.0, 5.0 / std::sqrt(2.0 * count[bin]))
        << "depth " << bin * kBin << " m";
    // The mean, in standard deviations, is off by about 1 / sqrt(n).
    EXPECT_NEAR(sum[bin] / count[bin], 0.0, 5.0 / std::sqrt(count[bin]));
    std::printf("depth noise at %.2f-%.2f m: %6d px, measured %.2f mm, "
                "model %.2f mm\n",
                bin * kBin, (bin + 1) * kBin, count[bin], measured * 1e3,
                model_sigma * 1e3);
  }
  EXPECT_GE(bins, 4);

  // A flat region: module_0's glass. A plane fitted to the noisy points
  // leaves residuals of about the noise along the viewing axis, foreshortened
  // onto the glass's normal by the angle between them.
  const Glass glass = TrueGlass(cell(), data(), 0);
  cell::Points points;
  double expected_sq_normal = 0.0;
  for (int v = 0; v < noisy.height; ++v) {
    for (int u = 0; u < noisy.width; ++u) {
      if (!IntersectGlass(camera_, glass, u, v, 0.05).inside) continue;
      const Eigen::Vector3d unit_depth =
          camera_.pose.linear() * cell::PointAtUnitDepth(camera_, u + 0.5, v + 0.5);
      const double z = reference_.at(u, v);
      const double sigma = noise.sigma_a + noise.sigma_b * z * z;
      // A depth error dz moves the point by dz * unit_depth.
      expected_sq_normal += std::pow(sigma * glass.normal.dot(unit_depth), 2);
      points.push_back(camera_.pose * (static_cast<double>(noisy.at(u, v)) *
                                       cell::PointAtUnitDepth(camera_, u + 0.5,
                                                              v + 0.5)));
    }
  }
  const std::optional<cell::Plane> plane = cell::FitPlane(points);
  ASSERT_TRUE(plane);
  double residual_sq = 0.0;
  for (const Eigen::Vector3d& p : points) {
    residual_sq += std::pow(plane->SignedDistance(p), 2);
  }
  const double measured = std::sqrt(residual_sq / points.size());
  const double expected = std::sqrt(expected_sq_normal / points.size());
  std::printf("glass, %zu px: residual from a fitted plane %.2f mm, "
              "model %.2f mm\n",
              points.size(), measured * 1e3, expected * 1e3);
  EXPECT_NEAR(measured / expected, 1.0, 0.03);

  // The same seed, the same image; another seed, another.
  DepthImage same = reference_;
  cell::AddDepthNoise(&same, noise);
  EXPECT_TRUE(SameImage(same, noisy));
  noise.seed = 8;
  DepthImage other = reference_;
  cell::AddDepthNoise(&other, noise);
  EXPECT_FALSE(SameImage(other, noisy));
  // Rendering with noise in the options is the same as adding it after.
  cell::DepthOptions options;
  options.noise = noise;
  EXPECT_TRUE(SameImage(cell::RenderDepth(model(), cell().data(), camera_, options),
                        other));
}

TEST_F(DepthCameraTest, EdgeEffectsOnlyAtDepthDiscontinuities) {
  cell::DepthNoise noise;
  noise.edge_effects = true;
  noise.seed = 3;
  DepthImage noisy = reference_;
  cell::AddDepthNoise(&noisy, noise);

  const DepthImage& clean = reference_;
  int edges = 0, mixable = 0, hit = 0, dropped = 0, mixed = 0;
  for (int v = 0; v < clean.height; ++v) {
    for (int u = 0; u < clean.width; ++u) {
      const float own = clean.at(u, v);
      const float now = noisy.at(u, v);
      if (!std::isfinite(own)) {
        EXPECT_FALSE(std::isfinite(now));
        continue;
      }
      // The edge test, done again here: the neighbours' depths.
      bool edge = false;
      float low = own, high = own;
      for (const auto& [du, dv] :
           std::vector<std::pair<int, int>>{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}) {
        const int nu = u + du, nv = v + dv;
        if (nu < 0 || nu >= clean.width || nv < 0 || nv >= clean.height) continue;
        const float other = clean.at(nu, nv);
        if (!std::isfinite(other)) {
          edge = true;
        } else if (std::abs(other - own) > noise.edge_threshold) {
          edge = true;
          low = std::min(low, other);
          high = std::max(high, other);
        }
      }
      if (!edge) {
        EXPECT_TRUE(SameReading(now, own)) << "pixel " << u << ", " << v;
        continue;
      }
      ++edges;
      if (high > low) ++mixable;
      if (SameReading(now, own)) continue;
      ++hit;
      if (!std::isfinite(now)) {
        ++dropped;
      } else {
        ++mixed;
        EXPECT_GE(now, low) << "pixel " << u << ", " << v;
        EXPECT_LE(now, high) << "pixel " << u << ", " << v;
      }
    }
  }
  std::printf("edge effects: %d edge pixels (%d next to a surface), %d hit: "
              "%d dropped, %d mixed\n",
              edges, mixable, hit, dropped, mixed);
  EXPECT_GT(edges, 1000);
  // Half the edge pixels are hit; of those next to a surface, half dropped.
  EXPECT_NEAR(static_cast<double>(hit) / edges, noise.edge_probability,
              5 * 0.5 / std::sqrt(edges));
  EXPECT_GT(mixed, 0.2 * hit);
  EXPECT_GT(dropped, 0.2 * hit);
}

TEST_F(DepthCameraTest, QuantizationRoundsToTheStep) {
  cell::DepthNoise noise;
  noise.quantize = true;
  noise.quantum = 1e-3;
  DepthImage stepped = reference_;
  cell::AddDepthNoise(&stepped, noise);
  for (int k = 0; k < stepped.size(); ++k) {
    if (!std::isfinite(reference_.depth[k])) continue;
    const double millimetres = stepped.depth[k] * 1e3;
    EXPECT_NEAR(millimetres, std::round(millimetres), 1e-3);
    EXPECT_LE(std::abs(stepped.depth[k] - reference_.depth[k]), 0.5e-3 + 1e-6);
  }
}

// ---------------------------------------------------- calibration errors

TEST_F(DepthCameraTest, CalibrationErrorMovesThePointsAsExpected) {
  const cell::PixelPoints truth = cell::PointsWithPixels(reference_, camera_);
  const cell::PixelPoints local =
      cell::PointsWithPixels(reference_, camera_, cell::PointFrame::kCamera);

  // No error, no change.
  EXPECT_TRUE(cell::Perturbed(camera_, Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero())
                  .pose.isApprox(camera_.pose, 0.0));

  // A shift alone moves every point by the same shift, turned into the
  // world by the camera's orientation.
  const Eigen::Vector3d shift(0.005, -0.003, 0.002);
  const cell::CameraModel shifted =
      cell::Perturbed(camera_, Eigen::Vector3d::Zero(), shift);
  const cell::Points moved = cell::PointsFromDepth(reference_, shifted);
  const Eigen::Vector3d expected_shift = camera_.pose.linear() * shift;
  for (std::size_t k = 0; k < moved.size(); ++k) {
    EXPECT_LT((moved[k] - truth.points[k] - expected_shift).norm(), 1e-9);
  }

  // A turn of 1 mrad about the camera's X (tilting it up) swings each point
  // about the camera's centre: by 1 mm per metre of its distance from the
  // camera's X axis.
  const Eigen::Vector3d turn(1e-3, 0, 0);
  const cell::CameraModel turned =
      cell::Perturbed(camera_, turn, Eigen::Vector3d::Zero());
  const cell::Points swung = cell::PointsFromDepth(reference_, turned);
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(1e-3, Eigen::Vector3d::UnitX()).toRotationMatrix();
  double total = 0.0;
  for (std::size_t k = 0; k < swung.size(); ++k) {
    EXPECT_LT((swung[k] - camera_.pose * (rotation * local.points[k])).norm(),
              1e-9);
    const double lever = local.points[k].tail<2>().norm();  // from the X axis
    const double distance = (swung[k] - truth.points[k]).norm();
    EXPECT_NEAR(distance, 2 * std::sin(0.5e-3) * lever, 1e-9);
    total += distance;
  }
  std::printf("1 mrad camera tilt: points move %.2f mm on average\n",
              total / swung.size() * 1e3);
  // The perturbed camera still sees each moved point at its own pixel.
  for (std::size_t k = 0; k < swung.size(); k += 997) {
    const std::optional<Eigen::Vector2d> pixel = cell::Project(turned, swung[k]);
    ASSERT_TRUE(pixel);
    EXPECT_EQ(static_cast<int>(pixel->x()) +
                  static_cast<int>(pixel->y()) * reference_.width,
              truth.pixels[k]);
  }
}

// ---------------------------------------------- misses and beyond range

TEST_F(DepthCameraTest, MissesAndReadingsBeyondRangeAreNaN) {
  // A camera high above everything, looking straight up: nothing to hit.
  // A small image, to keep it quick.
  cell::CameraModel sky = camera_;
  sky.width = 64;
  sky.height = 48;
  sky.focal = camera_.focal / 10;
  sky.cx = 32;
  sky.cy = 24;
  sky.pose = Eigen::Isometry3d::Identity();
  sky.pose.translation() = Eigen::Vector3d(0, 0, 10);
  sky.pose.linear() =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitX()).toRotationMatrix();
  ASSERT_GT(cell::RayThroughPixel(sky, 32, 24).z(), 0.99);
  cell::DepthOptions unlimited;
  unlimited.max_depth = std::numeric_limits<double>::infinity();
  std::vector<int> geoms;
  const DepthImage nothing =
      cell::RenderDepth(model(), cell().data(), sky, unlimited, &geoms);
  ASSERT_EQ(nothing.size(), 64 * 48);
  EXPECT_EQ(nothing.CountReadings(), 0);
  for (int geom : geoms) EXPECT_EQ(geom, -1);

  // The cell's camera with a range of 4 m: the deeper readings go, the
  // others are unchanged.
  cell::DepthOptions short_range;
  short_range.max_depth = 4.0;
  const DepthImage near =
      cell::RenderDepth(model(), cell().data(), camera_, short_range, &geoms);
  int kept = 0;
  int lost = 0;
  for (int k = 0; k < near.size(); ++k) {
    const float full = reference_.depth[k];
    if (std::isfinite(full) && full <= 4.0f) {
      ++kept;
      EXPECT_TRUE(SameReading(near.depth[k], full));
    } else {
      ++lost;
      EXPECT_FALSE(std::isfinite(near.depth[k]));
      EXPECT_EQ(geoms[k], -1);
    }
  }
  std::printf("max depth 4 m: %d pixels kept, %d without a reading\n", kept,
              lost);
  EXPECT_GT(kept, 10000);
  EXPECT_GT(lost, 10000);

  // The default range, 6 m, leaves the top rows, which see the floor far
  // beyond the pallet, without a reading.
  EXPECT_FALSE(reference_.has_reading(320, 0));
  EXPECT_LT(reference_.CountReadings(), reference_.size());
}

}  // namespace
