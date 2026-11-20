// Checks the front-module finder (src/module_finder.h) on depth images of
// the cell: each step alone (the prior, the crop, the glass's plane, the
// flattening, the pose), then the whole estimate against the truth over
// several randomized pallets and stack depths, without noise, with the
// camera's noise and with noise and a calibration error; the refusals; and
// that the same picture gives the same answer.
//
//   bazel test //:module_finder_test
//   bazel test --config=opt //:module_finder_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/cell.xml).
//
// Rendering an image takes about a second unoptimized, and setting up its
// scene (Reset settles the cell for a second of simulated time, then half a
// second more after modules are carried away) about as long again. So each
// scene is set up and rendered once, without noise, and kept; noise is added
// to copies (AddDepthNoise), and the calibration error is a different camera
// model handed to the finder for the same image. 3 seeds x 4 depths, plus 3
// pictures for the refusals (the arm at home twice, an empty pallet): 15.
//
// The truth (TrueModulePose, which geom each pixel saw) is used here to
// score and to check the steps; the finder never sees it.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/cell.h"
#include "src/depth_camera.h"
#include "src/module_finder.h"
#include "src/pallet.h"
#include "src/plane_fit.h"
#include "src/rectangle_fit.h"

namespace {

using cell::Cell;
using cell::DepthImage;
using cell::FinderDiagnostics;
using cell::FinderFailure;
using cell::ModuleEstimate;
using cell::PalletPrior;
using Eigen::Isometry3d;
using Eigen::Vector2d;
using Eigen::Vector3d;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kMillimetre = 0.001;

// The stated error, from print_perception over 25 seeds x depths 0, 10, 20,
// 29 (100 pictures each), with the worst seen there in the comments.
//
// Without noise: the module's centre within half a millimetre in every
// direction and its axes within a hundredth of a degree. Worst seen 0.24 mm
// along the long side (the pixel grid at the module's ends); everything else
// under 0.001 mm and 0.0001 degrees.
constexpr double kExactPosition = 0.5 * kMillimetre;
constexpr double kExactAngle = 0.01 * kDegree;

// With the camera's noise (depth noise, flying pixels and dropouts at the
// ends, millimetre quantisation): along the glass's normal within 0.2 mm
// (worst seen 0.044 mm), in the plane within 1.5 mm (worst 0.78 mm along
// the long side, 0.33 mm up the lean), the face's direction within 0.03
// degrees (worst 0.007) and the rotation about the normal within 0.1 degrees
// (worst 0.029).
constexpr double kNoisyNormal = 0.2 * kMillimetre;
constexpr double kNoisyInPlane = 1.5 * kMillimetre;
constexpr double kNoisyFace = 0.03 * kDegree;
constexpr double kNoisyRotation = 0.1 * kDegree;

// The first plane, fitted to every inlier, crease points at its top and
// bottom edges among them, is tipped by them: worst seen 0.026 degrees over
// 100 noiseless pictures. The finder's final plane, fitted to the glass's
// inside only, is not (kExactAngle and kNoisyFace above).
constexpr double kFirstPlaneAngle = 0.05 * kDegree;

// A calibration error moves every point by one rigid motion, so the answer
// must move by the same motion and by nothing else: the estimate from the
// wrong camera within this of the motion applied to the estimate from the
// right one, for the same image. Only the crop, fixed in the world, does not
// move with the points, and its edges are far from the glass. Worst seen
// under 0.001 mm and 0.0001 degrees.
constexpr double kPassThroughPosition = 0.05 * kMillimetre;
constexpr double kPassThroughAngle = 0.002 * kDegree;

// The calibration error used: the camera thought 1 mm off along its own X
// (the image's horizontal) and turned 0.05 degrees (0.87 mrad) about that
// axis, tipping its view. That moves the module by about 2.5 mm along its
// normal, 1 mm along its long side and 2.6 mm up the lean, and turns its
// face by 0.05 degrees.
const Vector3d kCalibrationShift(1.0 * kMillimetre, 0.0, 0.0);
const Vector3d kCalibrationTurn(0.05 * kDegree, 0.0, 0.0);

const std::vector<std::uint32_t> kSeeds = {1, 2, 3};
const std::vector<int> kDepths = {0, 10, 20, 29};

// How far `estimate` is from `truth`, in the true module's frame.
struct PoseError {
  Vector3d position;      // m: along x (long side), y (up the lean), z (normal)
  double face = 0.0;      // rad, between the z axes
  double rotation = 0.0;  // rad, about the true z
};

PoseError ErrorOf(const Isometry3d& estimate, const Isometry3d& truth) {
  const Eigen::Matrix3d r = truth.linear();
  PoseError e;
  e.position = r.transpose() * (estimate.translation() - truth.translation());
  const Vector3d z = estimate.linear().col(2);
  e.face = std::atan2(z.cross(r.col(2)).norm(), z.dot(r.col(2)));
  const Vector3d x = estimate.linear().col(0);
  e.rotation = std::atan2(r.col(0).cross(x).dot(r.col(2)), r.col(0).dot(x));
  return e;
}

DepthImage WithNoise(const DepthImage& exact, std::uint32_t seed) {
  DepthImage noisy = exact;
  cell::DepthNoise noise;
  noise.depth_noise = true;
  noise.edge_effects = true;
  noise.quantize = true;
  noise.seed = seed;
  cell::AddDepthNoise(&noisy, noise);
  return noisy;
}

// One scene's picture and what was truly there.
struct Picture {
  DepthImage image;              // exact
  std::vector<int> geoms;        // the geom each pixel saw: truth
  cell::CameraModel camera;      // the true camera
  Isometry3d truth = Isometry3d::Identity();  // the front module
  int front_geom = -1;           // its geom, or -1 with none left
};

class ModuleFinderTest : public ::testing::Test {
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
    prior_ = cell::PriorFromDesign(cell_->pallet());
  }

  static void TearDownTestSuite() {
    pictures_.clear();
    cell_.reset();
  }

  void SetUp() override { ASSERT_TRUE(cell_.has_value()) << load_error_; }

  // The picture of pallet `seed` with modules 0 to k - 1 carried away and
  // the arm at arm_clear (or arm_home), set up and rendered on first use.
  // k equal to the module count leaves the pallet empty.
  static const Picture& Take(std::uint32_t seed, int k, bool arm_home = false) {
    const auto key = std::make_tuple(seed, k, arm_home);
    auto found = pictures_.find(key);
    if (found != pictures_.end()) return found->second;

    Cell& c = *cell_;
    c.Reset(seed);
    for (int i = 0; i < k; ++i) c.ParkModule(i);
    c.PutArm(arm_home ? c.arm_home() : c.arm_clear());
    c.StepFor(0.5);
    mj_forward(c.model(), c.data());

    Picture picture;
    picture.camera = c.DepthCamera();
    const auto start = std::chrono::steady_clock::now();
    picture.image = cell::RenderDepth(*c.model(), c.data(), picture.camera, {},
                                      &picture.geoms);
    render_seconds_ += std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - start)
                           .count();
    ++renders_;
    if (k < c.module_count()) {
      picture.truth = c.TrueModulePose(k);
      picture.front_geom = c.model()->body_geomadr[c.pallet().modules[k]];
    }
    return pictures_.emplace(key, std::move(picture)).first->second;
  }

  static int FloorGeom() {
    return mj_name2id(cell_->model(), mjOBJ_GEOM, "floor");
  }

  static inline std::optional<Cell> cell_;
  static inline std::string load_error_;
  static inline PalletPrior prior_;
  static inline std::map<std::tuple<std::uint32_t, int, bool>, Picture>
      pictures_;
  static inline double render_seconds_ = 0.0;
  static inline int renders_ = 0;
};

// ------------------------------------------------------------------ prior

TEST_F(ModuleFinderTest, PriorIsTheDesign) {
  // models/pallet.xml: 30 modules 35 mm apart, leaning back 10 degrees,
  // glass facing the arm (-X), the front glass centre at (1.95, 0, 0.75),
  // the pallet's origin on the floor under it.
  EXPECT_EQ(prior_.module_count, 30);
  EXPECT_NEAR(prior_.pitch, 0.035, 1e-9);
  EXPECT_NEAR(prior_.DesignLean(), 10.0 * kDegree, 1e-6);
  const Vector3d normal = prior_.GlassNormal();
  EXPECT_NEAR(normal.x(), -std::cos(10.0 * kDegree), 1e-6);
  EXPECT_NEAR(normal.y(), 0.0, 1e-9);
  EXPECT_LT((prior_.FrontGlass().translation() - Vector3d(1.95, 0, 0.75))
                .norm(),
            1e-5);
  EXPECT_NEAR(prior_.floor_height, 0.0, 1e-9);
  // The module's +x is world -Y (to the right as seen from the arm).
  EXPECT_NEAR(prior_.front_module.linear().col(0).y(), -1.0, 1e-9);
}

// ------------------------------------------------------------------- crop

// The crop keeps every point on the front glass, at a full pallet and with
// one module left, and leaves out nearly all the floor.
TEST_F(ModuleFinderTest, CropKeepsTheGlassAndDropsTheFloor) {
  const cell::StackCrop crop = cell::CropFromPrior(prior_);
  for (int k : {0, 29}) {
    SCOPED_TRACE(testing::Message() << "k " << k);
    const Picture& picture = Take(1, k);
    const cell::PixelPoints cloud =
        cell::ImageToArmFrame(picture.image, picture.camera);
    int glass = 0, glass_kept = 0, floor = 0, floor_kept = 0;
    for (std::size_t j = 0; j < cloud.points.size(); ++j) {
      const int geom = picture.geoms[cloud.pixels[j]];
      const bool kept = crop.Contains(cloud.points[j]);
      if (geom == picture.front_geom) {
        ++glass;
        glass_kept += kept;
      } else if (geom == FloorGeom()) {
        ++floor;
        floor_kept += kept;
      }
    }
    std::printf("k %d: crop keeps %d of %d front-module points, %d of %d "
                "floor points\n",
                k, glass_kept, glass, floor_kept, floor);
    EXPECT_GT(glass, 40000);
    EXPECT_EQ(glass_kept, glass);
    EXPECT_GT(floor, 100000);
    EXPECT_LT(floor_kept, 0.01 * floor);
  }
}

// ------------------------------------------------------------ glass plane

// In the crop, with a full pallet and with the last module, the plane found
// is the front glass: it holds nearly every pixel of the glass, nearly all
// its points are the front module's (its glass, and its own top face just
// behind the glass's top edge, within the inlier distance: the crease), and
// it faces as the true glass. Which pixels are glass is decided on the exact
// image (within 1 mm of the true glass's plane); the fit runs on a noisy one.
TEST_F(ModuleFinderTest, PlaneIsTheGlassNotTheStackTopOrTheDeck) {
  const cell::StackCrop crop = cell::CropFromPrior(prior_);
  for (int k : {0, 29}) {
    SCOPED_TRACE(testing::Message() << "k " << k);
    const Picture& picture = Take(1, k);
    const Vector3d normal = picture.truth.linear().col(2);
    const Vector3d glass_centre =
        picture.truth * Vector3d(0, 0, 0.5 * cell::kModuleThickness);
    const cell::PixelPoints exact =
        cell::ImageToArmFrame(picture.image, picture.camera);
    std::vector<bool> is_glass(picture.image.size(), false);
    for (std::size_t j = 0; j < exact.points.size(); ++j) {
      is_glass[exact.pixels[j]] =
          picture.geoms[exact.pixels[j]] == picture.front_geom &&
          std::abs(normal.dot(exact.points[j] - glass_centre)) < 1e-3;
    }

    const DepthImage noisy = WithNoise(picture.image, 7);
    const cell::PixelPoints cloud =
        cell::ImageToArmFrame(noisy, picture.camera);
    cell::Points points;
    std::vector<int> pixels;
    for (int j : cell::CropIndices(cloud.points, crop)) {
      points.push_back(cloud.points[j]);
      pixels.push_back(cloud.pixels[j]);
    }
    const std::optional<cell::PlaneFitResult> plane =
        cell::FindGlassPlane(points, picture.camera, prior_);
    ASSERT_TRUE(plane.has_value());

    int front = 0, glass_in = 0, glass = 0;
    for (int i : plane->inliers) {
      front += picture.geoms[pixels[i]] == picture.front_geom;
      glass_in += is_glass[pixels[i]];
    }
    for (int pixel : pixels) glass += is_glass[pixel];
    const double angle =
        std::acos(std::min(1.0, plane->plane.normal.dot(normal)));
    std::printf("k %d: %zu inliers, %d on the front module, %d on its glass, "
                "which shows %d points; %.4f deg off its normal\n",
                k, plane->inliers.size(), front, glass_in, glass,
                angle / kDegree);
    EXPECT_GT(front, 0.99 * plane->inliers.size());
    EXPECT_GT(glass_in, 0.99 * glass);
    EXPECT_LT(angle, kFirstPlaneAngle);
    // The noise model is right: distances in units of each point's own
    // noise have a spread of about 1.
    EXPECT_NEAR(plane->normalised_rms, 1.0, 0.1);
  }
}

// On the whole picture, uncropped, an unconstrained RANSAC finds the floor,
// the biggest plane in sight. Told which way the glass faces, it finds the
// glass even there. (The first plane: see kFirstPlaneAngle.)
TEST_F(ModuleFinderTest, DirectionAloneSinglesOutTheGlassFromTheFloor) {
  const Picture& picture = Take(1, 0);
  const cell::Points points =
      cell::ImageToArmFrame(picture.image, picture.camera).points;

  cell::RansacOptions free;
  free.inlier_distance = 0.01;
  const std::optional<cell::RansacResult> floor =
      cell::FitPlaneRansac(points, free);
  ASSERT_TRUE(floor.has_value());
  EXPECT_GT(std::abs(floor->plane.normal.z()), std::cos(1.0 * kDegree));

  const std::optional<cell::PlaneFitResult> glass =
      cell::FindGlassPlane(points, picture.camera, prior_);
  ASSERT_TRUE(glass.has_value());
  EXPECT_LT(std::acos(std::min(
                1.0, glass->plane.normal.dot(picture.truth.linear().col(2)))),
            kFirstPlaneAngle);
}

// ---------------------------------------------------------------- flatten

TEST_F(ModuleFinderTest, FlatteningFrameAndRoundTrip) {
  // The design's glass plane: x along the design's +x, y up the lean.
  cell::GlassPlane plane;
  plane.normal = prior_.GlassNormal();
  plane.point = prior_.FrontGlass().translation();
  const Vector3d near = plane.point + Vector3d(0.3, -0.2, 0.1);
  const cell::PlaneFrame frame = cell::FlatteningFrame(plane, near);

  EXPECT_NEAR(frame.x.norm(), 1.0, 1e-12);
  EXPECT_NEAR(frame.y.norm(), 1.0, 1e-12);
  EXPECT_NEAR(frame.x.dot(frame.y), 0.0, 1e-12);
  EXPECT_LT((frame.x.cross(frame.y) - frame.normal).norm(), 1e-12);
  EXPECT_NEAR(frame.x.z(), 0.0, 1e-12);  // level
  EXPECT_GT(frame.y.z(), 0.9);            // up the lean
  EXPECT_LT((frame.x - prior_.front_module.linear().col(0)).norm(), 1e-9);
  EXPECT_LT((frame.y - prior_.front_module.linear().col(1)).norm(), 1e-9);
  EXPECT_NEAR(plane.SignedDistance(frame.origin), 0.0, 1e-12);

  std::mt19937 rng(3);
  std::uniform_real_distribution<double> spread(-2.0, 2.0);
  for (int trial = 0; trial < 100; ++trial) {
    const Vector2d q(spread(rng), spread(rng));
    EXPECT_LT((frame.Flatten(frame.Lift(q)) - q).norm(), 1e-12);
    const Vector3d p = near + Vector3d(spread(rng), spread(rng), spread(rng));
    // Lifting a flattened point puts it on the plane at right angles.
    const Vector3d foot = frame.Lift(frame.Flatten(p));
    EXPECT_NEAR(plane.SignedDistance(foot), 0.0, 1e-12);
    EXPECT_LT((p - foot).cross(plane.normal).norm(), 1e-12);
  }
}

TEST_F(ModuleFinderTest, PoseFromOutlineFollowsTheConvention) {
  cell::GlassPlane plane;
  plane.normal = prior_.GlassNormal();
  plane.point = prior_.FrontGlass().translation();
  const cell::PlaneFrame frame = cell::FlatteningFrame(plane, plane.point);

  // The design's own outline gives the design's pose.
  cell::OrientedRectangle outline;
  outline.length = cell::kModuleLength;
  outline.width = cell::kModuleWidth;
  const Isometry3d pose =
      cell::ModulePoseFromOutline(frame, outline, cell::kModuleThickness);
  EXPECT_TRUE(pose.isApprox(prior_.front_module, 1e-9));

  // Turned and moved in the plane: turned about the normal, the centre moved
  // along the frame's axes.
  outline.axis = Vector2d(std::cos(0.01), std::sin(0.01));
  outline.centre = Vector2d(0.02, -0.03);
  const Isometry3d moved =
      cell::ModulePoseFromOutline(frame, outline, cell::kModuleThickness);
  const PoseError e = ErrorOf(moved, prior_.front_module);
  EXPECT_NEAR(e.rotation, 0.01, 1e-12);
  EXPECT_NEAR(e.face, 0.0, 1e-12);
  EXPECT_LT((e.position - Vector3d(0.02, -0.03, 0.0)).norm(), 1e-12);
  EXPECT_NEAR(moved.linear().determinant(), 1.0, 1e-12);
}

// ------------------------------------------------------------- accuracy

TEST_F(ModuleFinderTest, ExactPicturesAreRecoveredWithinTheStatedError) {
  for (std::uint32_t seed : kSeeds) {
    for (int k : kDepths) {
      SCOPED_TRACE(testing::Message() << "seed " << seed << " k " << k);
      const Picture& picture = Take(seed, k);
      FinderDiagnostics why;
      const std::optional<ModuleEstimate> estimate = cell::FindFrontModule(
          picture.image, picture.camera, prior_, {}, &why);
      ASSERT_TRUE(estimate.has_value()) << why.detail;
      const PoseError e = ErrorOf(estimate->pose, picture.truth);
      EXPECT_LT(e.position.lpNorm<Eigen::Infinity>(), kExactPosition)
          << e.position.transpose() * 1e3 << " mm";
      EXPECT_LT(e.face, kExactAngle);
      EXPECT_LT(std::abs(e.rotation), kExactAngle);
      // Both long edges came from the creases, and the outline is the
      // module's size.
      EXPECT_TRUE(why.top.found);
      EXPECT_TRUE(why.bottom.found);
      EXPECT_NEAR(estimate->width, cell::kModuleWidth, kExactPosition);
      // The length rests on the ends alone: worst seen 0.4 mm off.
      EXPECT_NEAR(estimate->length, cell::kModuleLength, 1.0 * kMillimetre);
    }
  }
  std::printf("render: %.0f ms per image (%d images so far)\n",
              1e3 * render_seconds_ / renders_, renders_);
}

TEST_F(ModuleFinderTest, NoisyPicturesAreRecoveredWithinTheStatedError) {
  Vector3d worst = Vector3d::Zero();
  double worst_face = 0.0, worst_rotation = 0.0;
  double fit_seconds = 0.0;
  int fits = 0;
  for (std::uint32_t seed : kSeeds) {
    for (int k : kDepths) {
      SCOPED_TRACE(testing::Message() << "seed " << seed << " k " << k);
      const Picture& picture = Take(seed, k);
      const DepthImage noisy = WithNoise(picture.image, 1000 * seed + k);
      FinderDiagnostics why;
      const std::optional<ModuleEstimate> estimate =
          cell::FindFrontModule(noisy, picture.camera, prior_, {}, &why);
      ASSERT_TRUE(estimate.has_value()) << why.detail;
      fit_seconds += why.seconds_total;
      ++fits;
      const PoseError e = ErrorOf(estimate->pose, picture.truth);
      EXPECT_LT(std::abs(e.position.z()), kNoisyNormal);
      EXPECT_LT(e.position.head<2>().lpNorm<Eigen::Infinity>(), kNoisyInPlane)
          << e.position.transpose() * 1e3 << " mm";
      EXPECT_LT(e.face, kNoisyFace);
      EXPECT_LT(std::abs(e.rotation), kNoisyRotation);
      EXPECT_TRUE(why.top.found);
      EXPECT_TRUE(why.bottom.found);
      EXPECT_EQ(why.view.on_glass, why.view.pixels);
      worst = worst.cwiseMax(e.position.cwiseAbs());
      worst_face = std::max(worst_face, e.face);
      worst_rotation = std::max(worst_rotation, std::abs(e.rotation));
    }
  }
  std::printf("noisy: worst %.3f mm along the long side, %.3f up the lean, "
              "%.3f along the normal; %.4f deg face, %.4f deg about the "
              "normal\n",
              worst.x() / kMillimetre, worst.y() / kMillimetre,
              worst.z() / kMillimetre, worst_face / kDegree,
              worst_rotation / kDegree);
  std::printf("fit: %.1f ms per image\n", 1e3 * fit_seconds / fits);
}

// A wrong calibration moves every point by the same rigid motion, and the
// estimate with it: nothing in one picture can tell. The answer from the
// wrong camera is the answer from the right one moved by that motion.
TEST_F(ModuleFinderTest, CalibrationErrorPassesStraightThrough) {
  // Two fits a picture: two of the pallets are enough.
  double worst_position = 0.0, worst_angle = 0.0;
  for (std::uint32_t seed : {kSeeds[0], kSeeds[1]}) {
    for (int k : kDepths) {
      SCOPED_TRACE(testing::Message() << "seed " << seed << " k " << k);
      const Picture& picture = Take(seed, k);
      const DepthImage noisy = WithNoise(picture.image, 1000 * seed + k);
      const cell::CameraModel wrong =
          cell::Perturbed(picture.camera, kCalibrationTurn, kCalibrationShift);
      const std::optional<ModuleEstimate> right =
          cell::FindFrontModule(noisy, picture.camera, prior_);
      const std::optional<ModuleEstimate> moved =
          cell::FindFrontModule(noisy, wrong, prior_);
      ASSERT_TRUE(right.has_value());
      ASSERT_TRUE(moved.has_value());

      const Isometry3d motion = wrong.pose * picture.camera.pose.inverse();
      const PoseError e = ErrorOf(moved->pose, motion * right->pose);
      EXPECT_LT(e.position.norm(), kPassThroughPosition)
          << e.position.transpose() * 1e3 << " mm";
      EXPECT_LT(e.face, kPassThroughAngle);
      EXPECT_LT(std::abs(e.rotation), kPassThroughAngle);
      worst_position = std::max(worst_position, e.position.norm());
      worst_angle = std::max({worst_angle, e.face, std::abs(e.rotation)});

      // And against the truth it is off by about the motion itself: the
      // calibration error is the error.
      const PoseError expected = ErrorOf(motion * picture.truth, picture.truth);
      const PoseError actual = ErrorOf(moved->pose, picture.truth);
      EXPECT_GT(expected.position.norm(), 3.0 * kMillimetre);
      EXPECT_LT((actual.position - expected.position).norm(),
                kNoisyInPlane + kPassThroughPosition);
      EXPECT_NEAR(actual.face, expected.face, kNoisyFace);
    }
  }
  std::printf("calibration error: the answer moved as the camera did, to "
              "within %.3f mm and %.4f deg\n",
              worst_position / kMillimetre, worst_angle / kDegree);
}

// ------------------------------------------------------------- refusals

// With the arm at arm_home its tool stands in front of the glass. The
// finder refuses: the tool cuts a long side so that no rectangle with four
// square sides is left, or the module-sized one left is too wide, or the
// rectangle is found but a large part of it does not read the glass. Over
// 100 pictures (print_perception --arm-home) all were refused: 25 with no
// rectangle (all at k = 0), 4 the wrong size, 71 the view blocked (14 to 35%
// of the glass's pixels read something in front of it).
TEST_F(ModuleFinderTest, RefusesWithTheArmInTheWay) {
  for (int k : {0, 10}) {
    SCOPED_TRACE(testing::Message() << "k " << k);
    const Picture& picture = Take(1, k, /*arm_home=*/true);
    FinderDiagnostics why;
    EXPECT_FALSE(cell::FindFrontModule(WithNoise(picture.image, 5),
                                       picture.camera, prior_, {}, &why)
                     .has_value());
    std::printf("arm at home, k %d: refused, %s: %s\n", k,
                cell::FinderFailureName(why.failure), why.detail.c_str());
    EXPECT_TRUE(why.failure == FinderFailure::kViewBlocked ||
                why.failure == FinderFailure::kNoRectangle ||
                why.failure == FinderFailure::kSizeMismatch)
        << cell::FinderFailureName(why.failure);
    if (why.failure == FinderFailure::kViewBlocked) {
      EXPECT_GT(why.view.in_front, 0.1 * why.view.pixels);
    }
  }
}

// With every module gone, the biggest plane facing the arm is the back rest,
// 2.5 m wide and 1.0 m tall: not a module.
TEST_F(ModuleFinderTest, RefusesWithNoModuleLeft) {
  const Picture& picture = Take(1, cell_->module_count());
  FinderDiagnostics why;
  EXPECT_FALSE(cell::FindFrontModule(WithNoise(picture.image, 6),
                                     picture.camera, prior_, {}, &why)
                   .has_value());
  EXPECT_EQ(why.failure, FinderFailure::kSizeMismatch) << why.detail;
}

// A module of a size the pallet was not designed for is refused, whichever
// side is wrong.
TEST_F(ModuleFinderTest, RefusesARectangleOfTheWrongSize) {
  const Picture& picture = Take(2, 0);
  const DepthImage noisy = WithNoise(picture.image, 8);
  PalletPrior shorter = prior_;
  shorter.module_length = 2.0;
  FinderDiagnostics why;
  EXPECT_FALSE(cell::FindFrontModule(noisy, picture.camera, shorter, {}, &why)
                   .has_value());
  EXPECT_EQ(why.failure, FinderFailure::kSizeMismatch) << why.detail;

  PalletPrior narrower = prior_;
  narrower.module_width = 1.0;
  EXPECT_FALSE(cell::FindFrontModule(noisy, picture.camera, narrower, {}, &why)
                   .has_value());
  EXPECT_EQ(why.failure, FinderFailure::kSizeMismatch) << why.detail;
}

// A pallet designed to lean 3 degrees further back than this one does.
TEST_F(ModuleFinderTest, RefusesAPlaneThatDoesNotLeanAsDesigned) {
  const Picture& picture = Take(2, 0);
  PalletPrior steeper = prior_;
  steeper.front_module.linear() =
      steeper.front_module.linear() *
      Eigen::AngleAxisd(-3.0 * kDegree, Vector3d::UnitX()).toRotationMatrix();
  ASSERT_NEAR(steeper.DesignLean(), 13.0 * kDegree, 1e-6);
  FinderDiagnostics why;
  EXPECT_FALSE(cell::FindFrontModule(picture.image, picture.camera, steeper,
                                     {}, &why)
                   .has_value());
  EXPECT_EQ(why.failure, FinderFailure::kWrongLean) << why.detail;
}

TEST_F(ModuleFinderTest, RefusesBadInput) {
  const Picture& picture = Take(2, 0);
  FinderDiagnostics why;
  EXPECT_FALSE(cell::FindFrontModule(DepthImage{}, picture.camera, prior_, {},
                                     &why)
                   .has_value());
  EXPECT_EQ(why.failure, FinderFailure::kBadInput);

  // An image with nothing in it.
  DepthImage blank = picture.image;
  for (float& z : blank.depth) z = std::nanf("");
  EXPECT_FALSE(
      cell::FindFrontModule(blank, picture.camera, prior_, {}, &why)
          .has_value());
  EXPECT_EQ(why.failure, FinderFailure::kTooFewPoints);
}

// ---------------------------------------------------------- determinism

TEST_F(ModuleFinderTest, SamePictureGivesTheSameAnswer) {
  const Picture& picture = Take(3, 10);
  const DepthImage noisy = WithNoise(picture.image, 9);
  const std::optional<ModuleEstimate> first =
      cell::FindFrontModule(noisy, picture.camera, prior_);
  const std::optional<ModuleEstimate> second =
      cell::FindFrontModule(noisy, picture.camera, prior_);
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->pose.matrix(), second->pose.matrix());
  EXPECT_EQ(first->diagnostics.plane_inliers,
            second->diagnostics.plane_inliers);
}

}  // namespace
