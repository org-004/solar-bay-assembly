// Checks the check after pickup (src/held_module_finder.h) on the rig,
// models/check_scene.xml: module_0 held under the tool by the suction weld
// at seeded grip offsets (up to 10 mm and 1.5 degrees in each of the six
// components), check_camera on the floor looking up. The grip offset
// recovered from one depth image against the truth, without noise, with the
// camera's noise and with a calibration error; no size or centre bias from
// the edges that would move the centre; the frame algebra; the refusals;
// determinism; and what the stage is for: a place computed with the design
// grip misses by the grip offset, one computed with the measured grip lands
// within the measurement's error.
//
//   bazel test //:held_module_finder_test
//   bazel test --config=opt //:held_module_finder_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test.
//
// A shot (grip, settle, render) takes about 40 ms optimized; a fit about
// 30 ms optimized and some 80 times longer unoptimized. So each shot is
// taken once, without noise, and kept; noise is added to copies
// (AddDepthNoise), and a calibration error is a different camera model
// handed to the finder for the same image; the noisy fits are kept too and
// shared. About 40 fits in all.
//
// The truth (Suction::HeldPoseInTip, through CheckShot) is used here to
// score; the finder never sees it.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/check_rig.h"
#include "src/depth_camera.h"
#include "src/held_module_finder.h"
#include "src/module_finder.h"

namespace {

using cell::CheckRig;
using cell::CheckShot;
using cell::DepthImage;
using cell::HeldFinderDiagnostics;
using cell::HeldFinderFailure;
using cell::HeldModuleEstimate;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Six = Eigen::Matrix<double, 6, 1>;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kMillimetre = 0.001;

// The grip offsets drawn: each shift within +-10 mm, each turn within +-1.5
// degrees, from the design grip.
constexpr double kOffsetPosition = 10.0 * kMillimetre;
constexpr double kOffsetAngle = 1.5 * kDegree;
constexpr int kShots = 12;

// The stated error of the recovered module-in-tip against the truth, from
// print_check over 100 seeds (seeds 1 to 100), with the worst seen there in
// the comments. In the true module's frame: x the long side, y the short
// side, z the normal (vertical at the check pose); turns about each.
//
// Without noise: in the plane within 0.3 mm (worst seen 0.065 mm along x,
// 0.034 along y), along the normal and in tilt nothing measurable (under
// 0.0001 mm and 0.0001 degrees), about the vertical within 0.02 degrees
// (worst 0.005).
constexpr double kExactInPlane = 0.3 * kMillimetre;
constexpr double kExactNormal = 0.01 * kMillimetre;
constexpr double kExactTilt = 0.001 * kDegree;
constexpr double kExactYaw = 0.02 * kDegree;

// With the camera's noise (depth noise, flying and dropped pixels at the
// edges, millimetre quantisation): in the plane within 0.6 mm (worst seen
// 0.31 mm along x, 0.16 along y; RMS 0.11 and 0.064), along the normal
// within 0.05 mm (worst 0.012), tilts within 0.005 degrees (worst 0.0016),
// about the vertical within 0.04 degrees (worst 0.018, RMS 0.006).
constexpr double kNoisyInPlane = 0.6 * kMillimetre;
constexpr double kNoisyNormal = 0.05 * kMillimetre;
constexpr double kNoisyTilt = 0.005 * kDegree;
constexpr double kNoisyYaw = 0.04 * kDegree;

// With noise the edges lose pixels: about half of each edge's pixels are
// dropped or fly off the plane, so each side moves in by about half a
// pixel and the module reads about one pixel (3.4 mm) short both ways
// (measured: 3.0 to 3.9 mm, mean 3.4, the same with the rig's roof on,
// when the edge pixels fly, or off, when they only drop out). Opposite
// edges lose alike, so the centre does not move: its mean error over 100
// seeds was 0.005 mm along x and -0.016 mm along y.
constexpr double kNoisyShortLow = 2.5 * kMillimetre;
constexpr double kNoisyShortHigh = 4.5 * kMillimetre;
constexpr double kCentreBias = 0.1 * kMillimetre;  // |mean| over the shots

// A calibration error moves every point by one rigid motion, so the
// measured module moves by that motion and by nothing else: the answer
// from the wrong camera within this of the motion applied to the answer
// from the right one, for the same image.
constexpr double kPassThroughPosition = 0.05 * kMillimetre;
constexpr double kPassThroughAngle = 0.002 * kDegree;

// The calibration error used: the camera thought 1 mm off along its own X
// and turned 0.05 degrees (0.87 mrad) about it. At 1.4 m that moves the
// module by about 1.2 mm along its short side and 1 mm along its long side,
// and tips it 0.05 degrees.
const Vector3d kCalibrationShift(1.0 * kMillimetre, 0.0, 0.0);
const Vector3d kCalibrationTurn(0.05 * kDegree, 0.0, 0.0);

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

// Expects each of the six error components within its bound.
void ExpectWithin(const Six& e, double in_plane, double normal, double tilt,
                  double yaw) {
  EXPECT_LT(std::abs(e[0]), in_plane) << "x " << e[0] / kMillimetre << " mm";
  EXPECT_LT(std::abs(e[1]), in_plane) << "y " << e[1] / kMillimetre << " mm";
  EXPECT_LT(std::abs(e[2]), normal) << "z " << e[2] / kMillimetre << " mm";
  EXPECT_LT(std::abs(e[3]), tilt) << "about x " << e[3] / kDegree << " deg";
  EXPECT_LT(std::abs(e[4]), tilt) << "about y " << e[4] / kDegree << " deg";
  EXPECT_LT(std::abs(e[5]), yaw) << "about z " << e[5] / kDegree << " deg";
}

// Worst and root mean square of each component, printed.
struct Spread {
  Six worst = Six::Zero();
  Six squares = Six::Zero();
  Six sum = Six::Zero();
  int n = 0;
  void Add(const Six& e) {
    worst = worst.cwiseMax(e.cwiseAbs());
    squares += e.cwiseProduct(e);
    sum += e;
    ++n;
  }
  void Print(const char* name) const {
    const Six rms = (squares / std::max(n, 1)).cwiseSqrt();
    std::printf("%s over %d shots: worst / RMS  x %.3f / %.3f  y %.3f / %.3f"
                "  z %.4f / %.4f mm;  about x %.4f / %.4f  y %.4f / %.4f  "
                "z %.4f / %.4f deg\n",
                name, n, worst[0] / kMillimetre, rms[0] / kMillimetre,
                worst[1] / kMillimetre, rms[1] / kMillimetre,
                worst[2] / kMillimetre, rms[2] / kMillimetre,
                worst[3] / kDegree, rms[3] / kDegree, worst[4] / kDegree,
                rms[4] / kDegree, worst[5] / kDegree, rms[5] / kDegree);
  }
};

class HeldModuleFinderTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const char* path = std::getenv("MODEL_PATH");
    if (path == nullptr) {
      load_error_ = "MODEL_PATH is not set";
      return;
    }
    std::optional<CheckRig> loaded = CheckRig::Load(path, &load_error_);
    if (!loaded) return;
    rig_.emplace(std::move(*loaded));
  }

  static void TearDownTestSuite() {
    noisy_.clear();
    shots_.clear();
    rig_.reset();
  }

  void SetUp() override { ASSERT_TRUE(rig_.has_value()) << load_error_; }

  // The tool tip of shot `seed`: the nominal one, moved by up to 1 cm and
  // turned by up to half a degree about the vertical, as an arm that is not
  // exactly at its check pose holds it. The expected pose follows the tool.
  static Isometry3d TipFor(std::uint32_t seed) {
    const Six jiggle = cell::PoseError6(
        cell::DrawGripOffset(seed + 100000u, 0.01, 0.5 * kDegree),
        Isometry3d::Identity());
    Isometry3d tip = rig_->nominal_tip();
    tip.translation() += jiggle.head<3>();
    tip.linear() =
        Eigen::AngleAxisd(jiggle[5], Vector3d::UnitZ()).toRotationMatrix() *
        tip.linear();
    return tip;
  }

  // The shot for grip offset `seed`, taken on first use, with the roof on
  // or off.
  static const CheckShot& Take(std::uint32_t seed, bool roof = true) {
    const auto key = std::make_pair(seed, roof);
    auto found = shots_.find(key);
    if (found != shots_.end()) return found->second;
    rig_->SetRoof(roof);
    std::string error;
    std::optional<CheckShot> shot = rig_->Take(
        TipFor(seed),
        cell::DrawGripOffset(seed, kOffsetPosition, kOffsetAngle), &error);
    rig_->SetRoof(true);
    EXPECT_TRUE(shot.has_value()) << error;
    if (!shot) shot.emplace();
    return shots_.emplace(key, std::move(*shot)).first->second;
  }

  static Isometry3d Expected(const CheckShot& shot) {
    return shot.tip * cell::DesignModuleInTip();
  }

  // The grip offset recovered from `image` of `shot`, as module in tip.
  static std::optional<Isometry3d> Recover(
      const CheckShot& shot, const DepthImage& image,
      const cell::CameraModel& camera, HeldFinderDiagnostics* why = nullptr,
      std::optional<HeldModuleEstimate>* estimate = nullptr) {
    std::optional<HeldModuleEstimate> found =
        cell::FindHeldModule(image, camera, Expected(shot), {}, why);
    if (estimate) *estimate = found;
    if (!found) return std::nullopt;
    return cell::GripOffsetFromCheck(found->pose, shot.tip);
  }

  // FindHeldModule on shot `seed` (roof on) with the camera's noise, seeded
  // 1000 + seed, computed on first use: shared by the tests that need it.
  static const std::optional<HeldModuleEstimate>& Noisy(std::uint32_t seed) {
    auto found = noisy_.find(seed);
    if (found != noisy_.end()) return found->second;
    const CheckShot& shot = Take(seed);
    return noisy_
        .emplace(seed, cell::FindHeldModule(WithNoise(shot.image, 1000 + seed),
                                            shot.camera, Expected(shot)))
        .first->second;
  }

  static inline std::optional<CheckRig> rig_;
  static inline std::string load_error_;
  static inline std::map<std::pair<std::uint32_t, bool>, CheckShot> shots_;
  static inline std::map<std::uint32_t, std::optional<HeldModuleEstimate>>
      noisy_;
};

// ------------------------------------------------------------ the scene

// The rig as designed: the camera looks straight up, its rows turned 5
// degrees from the module's long side; the module at the design grip hangs
// with its underside 1.40 m above the lens; and it is in view with room for
// the tolerances.
TEST_F(HeldModuleFinderTest, SceneAsDesigned) {
  const cell::CameraModel camera = rig_->camera();
  EXPECT_EQ(camera.width, 848);
  EXPECT_EQ(camera.height, 480);
  const Vector3d look = -camera.pose.linear().col(2);
  EXPECT_GT(look.z(), std::cos(0.01 * kDegree));
  const Vector3d rows = camera.pose.linear().col(0);
  EXPECT_NEAR(std::atan2(rows.y(), rows.x()), 5.0 * kDegree, 1e-6);

  const Isometry3d design = rig_->nominal_tip() * cell::DesignModuleInTip();
  // As models/check_scene.xml builds module_0.
  EXPECT_LT((design.translation() - Vector3d(0, 0, 1.5675)).norm(), 1e-9);
  EXPECT_TRUE(design.linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  const double distance =
      design.translation().z() - 0.5 * cell::kModuleThickness -
      camera.pose.translation().z();
  EXPECT_NEAR(distance, 1.40, 1e-9);

  const CheckShot& shot = Take(1);
  int on_module = 0;
  const int module_geom = mj_name2id(&rig_->model(), mjOBJ_GEOM, "module_0");
  for (int g : shot.geoms) on_module += g == module_geom;
  std::printf("check camera: %.1f px focal, %.2f mm per pixel at the "
              "underside 1.40 m up; %d pixels on the underside; render "
              "%.0f ms\n",
              camera.focal, 1e3 * distance / camera.focal, on_module,
              1e3 * shot.render_seconds);
  EXPECT_GT(on_module, 200000);
  EXPECT_NEAR(1e3 * distance / camera.focal, 3.44, 0.01);
}

// The weld holds the module where the rig gripped it: the truth is the
// design grip times the drawn offset, to a hundredth of a millimetre (the
// weld's sag under 30 kg is about 0.004 mm).
TEST_F(HeldModuleFinderTest, RigHoldsTheDrawnOffset) {
  for (std::uint32_t seed = 1; seed <= 3; ++seed) {
    const CheckShot& shot = Take(seed);
    const Isometry3d meant =
        cell::DesignModuleInTip() *
        cell::DrawGripOffset(seed, kOffsetPosition, kOffsetAngle);
    const Six e = cell::PoseError6(shot.truth_in_tip, meant);
    EXPECT_LT(e.head<3>().norm(), 0.01 * kMillimetre);
    EXPECT_LT(e.tail<3>().norm(), 0.001 * kDegree);
  }
}

// ----------------------------------------------------- the frame algebra

TEST_F(HeldModuleFinderTest, AlgebraRoundTripsExactly) {
  for (std::uint32_t seed = 1; seed <= 20; ++seed) {
    // Any tip pose and any grip, far from the design.
    Isometry3d tip = cell::DrawGripOffset(seed + 7, 2.0, std::numbers::pi);
    const Isometry3d grip =
        cell::DrawGripOffset(seed + 70, 0.5, std::numbers::pi / 2);
    const Isometry3d module = tip * grip;
    EXPECT_TRUE(cell::GripOffsetFromCheck(module, tip).isApprox(grip, 1e-12));
    EXPECT_TRUE(cell::CorrectedTipTarget(module, grip).isApprox(tip, 1e-12));
    // The corrected target puts the module, not the tip, on the target.
    const Isometry3d target =
        cell::DrawGripOffset(seed + 700, 3.0, std::numbers::pi);
    EXPECT_TRUE((cell::CorrectedTipTarget(target, grip) * grip)
                    .isApprox(target, 1e-12));
  }
}

// The plan's example: a module gripped 5 mm to one side sends the tool tip
// 5 mm to the other; one gripped turned sends the tip turned back, about
// the module's centre.
TEST_F(HeldModuleFinderTest, CorrectionMovesTheTipTheOtherWay) {
  Isometry3d target = Isometry3d::Identity();  // flat on the tube, +x along X
  target.translation() = Vector3d(-2.0, 0.0, 0.8175);
  const Isometry3d design = cell::DesignModuleInTip();
  const Isometry3d plain = cell::CorrectedTipTarget(target, design);
  // Without an offset: the tip on the middle of the glass, pointing down.
  EXPECT_LT((plain.translation() - Vector3d(-2.0, 0.0, 0.835)).norm(), 1e-12);
  EXPECT_NEAR(plain.linear().col(2).z(), -1.0, 1e-12);

  // Held 5 mm toward the module's +y (left, seen from above with +x ahead).
  const Isometry3d left =
      design * cell::GripOffset(Vector3d(0, 0.005, 0), Vector3d::Zero());
  const Vector3d moved =
      cell::CorrectedTipTarget(target, left).translation() -
      plain.translation();
  EXPECT_LT((moved - Vector3d(0, -0.005, 0)).norm(), 1e-12);

  // Held turned 0.5 degrees about the vertical: the tip turns 0.5 degrees
  // back, and since the turn is about the module's centre, which is right
  // under the tip, the tip does not move.
  const Isometry3d turned =
      design * cell::GripOffset(Vector3d::Zero(),
                                Vector3d(0, 0, 0.5 * kDegree));
  const Isometry3d tip = cell::CorrectedTipTarget(target, turned);
  const Eigen::AngleAxisd turn(plain.linear().transpose() * tip.linear());
  EXPECT_NEAR(turn.angle(), 0.5 * kDegree, 1e-12);
  EXPECT_NEAR(std::abs((plain.linear() * turn.axis()).z()), 1.0, 1e-9);
  EXPECT_LT((tip.translation() - plain.translation()).norm(), 1e-12);
}

// -------------------------------------------------------------- accuracy

TEST_F(HeldModuleFinderTest, ExactPicturesAreRecoveredWithinTheStatedError) {
  Spread spread;
  for (std::uint32_t seed = 1; seed <= kShots; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const CheckShot& shot = Take(seed);
    HeldFinderDiagnostics why;
    std::optional<HeldModuleEstimate> estimate;
    const std::optional<Isometry3d> grip =
        Recover(shot, shot.image, shot.camera, &why, &estimate);
    ASSERT_TRUE(grip.has_value()) << why.detail;
    const Six e = cell::PoseError6(*grip, shot.truth_in_tip);
    ExpectWithin(e, kExactInPlane, kExactNormal, kExactTilt, kExactYaw);
    spread.Add(e);
    // No bias in the size either, without noise.
    EXPECT_NEAR(estimate->length, cell::kModuleLength, 0.2 * kMillimetre);
    EXPECT_NEAR(estimate->width, cell::kModuleWidth, 0.2 * kMillimetre);
    EXPECT_EQ(why.view.on_glass, why.view.pixels);
  }
  spread.Print("exact");
}

// With noise, on the roof's picture and without it: within the stated
// error; the module reads about a pixel short both ways, and its centre
// does not move.
TEST_F(HeldModuleFinderTest, NoisyPicturesAreRecoveredAndTheCentreIsUnbiased) {
  for (bool roof : {true, false}) {
    SCOPED_TRACE(testing::Message() << "roof " << roof);
    Spread spread;
    double short_length = 0.0, short_width = 0.0, fit_seconds = 0.0;
    const int shots = roof ? kShots : kShots / 2;
    for (std::uint32_t seed = 1; seed <= static_cast<std::uint32_t>(shots);
         ++seed) {
      SCOPED_TRACE(testing::Message() << "seed " << seed);
      const CheckShot& shot = Take(seed, roof);
      HeldFinderDiagnostics why;
      std::optional<HeldModuleEstimate> estimate;
      if (roof) {
        estimate = Noisy(seed);
        if (estimate) why = estimate->diagnostics;
      } else {
        Recover(shot, WithNoise(shot.image, 1000 + seed), shot.camera, &why,
                &estimate);
      }
      ASSERT_TRUE(estimate.has_value()) << why.detail;
      const Six e = cell::PoseError6(
          cell::GripOffsetFromCheck(estimate->pose, shot.tip),
          shot.truth_in_tip);
      ExpectWithin(e, kNoisyInPlane, kNoisyNormal, kNoisyTilt, kNoisyYaw);
      spread.Add(e);
      const double length = cell::kModuleLength - estimate->length;
      const double width = cell::kModuleWidth - estimate->width;
      EXPECT_GT(length, kNoisyShortLow);
      EXPECT_LT(length, kNoisyShortHigh);
      EXPECT_GT(width, kNoisyShortLow);
      EXPECT_LT(width, kNoisyShortHigh);
      short_length += length;
      short_width += width;
      fit_seconds += why.seconds_total;
    }
    spread.Print(roof ? "noisy, roof on " : "noisy, roof off");
    const Six mean = spread.sum / spread.n;
    std::printf("  reads %.2f mm short along the length and %.2f mm across "
                "on average; the centre's mean error %.3f mm along x, %.3f "
                "mm along y; fit %.0f ms\n",
                short_length / shots / kMillimetre,
                short_width / shots / kMillimetre, mean[0] / kMillimetre,
                mean[1] / kMillimetre, 1e3 * fit_seconds / shots);
    // Half the shortening would be 1.7 mm; the centre stays put.
    EXPECT_LT(std::abs(mean[0]), kCentreBias);
    EXPECT_LT(std::abs(mean[1]), kCentreBias);
  }
}

// A wrong calibration moves the measured module by the camera's error, as
// predicted: the answer from the wrong camera is the answer from the right
// one moved by wrong.pose * right.pose^-1, and the recovered grip is off by
// that motion, seen from the tip. Unlike the pallet camera's error, which
// the check takes out, this one goes straight into the placement.
TEST_F(HeldModuleFinderTest, CalibrationErrorShiftsAsPredicted) {
  double worst_position = 0.0, worst_angle = 0.0;
  for (std::uint32_t seed = 1; seed <= kShots / 2; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const CheckShot& shot = Take(seed);
    const DepthImage noisy = WithNoise(shot.image, 1000 + seed);
    const cell::CameraModel wrong =
        cell::Perturbed(shot.camera, kCalibrationTurn, kCalibrationShift);
    const std::optional<HeldModuleEstimate>& right = Noisy(seed);
    std::optional<HeldModuleEstimate> moved;
    ASSERT_TRUE(right.has_value());
    ASSERT_TRUE(Recover(shot, noisy, wrong, nullptr, &moved));

    const Isometry3d motion = wrong.pose * shot.camera.pose.inverse();
    const Six pass = cell::PoseError6(moved->pose, motion * right->pose);
    EXPECT_LT(pass.head<3>().norm(), kPassThroughPosition)
        << pass.head<3>().transpose() / kMillimetre << " mm";
    EXPECT_LT(pass.tail<3>().norm(), kPassThroughAngle);
    worst_position = std::max(worst_position, pass.head<3>().norm());
    worst_angle = std::max(worst_angle, pass.tail<3>().norm());

    // Against the truth, the grip is off by the motion's effect on the
    // module, to within the noisy error.
    const Six predicted =
        cell::PoseError6(motion * shot.truth_world, shot.truth_world);
    const Six actual = cell::PoseError6(
        cell::GripOffsetFromCheck(moved->pose, shot.tip), shot.truth_in_tip);
    EXPECT_GT(predicted.head<2>().norm(), 1.0 * kMillimetre);
    EXPECT_LT((actual.head<2>() - predicted.head<2>()).norm(), kNoisyInPlane);
    EXPECT_NEAR(actual[2], predicted[2], kNoisyNormal + kPassThroughPosition);
    EXPECT_LT((actual.tail<3>() - predicted.tail<3>()).norm(),
              kNoisyYaw + kPassThroughAngle);
    if (seed == 1) {
      std::printf("calibration 1 mm and 0.05 deg about the camera's X: the "
                  "module moves %.2f mm along x, %.2f along y, %.3f along z, "
                  "and tips %.4f deg; recovered %.2f, %.2f, %.3f mm\n",
                  predicted[0] / kMillimetre, predicted[1] / kMillimetre,
                  predicted[2] / kMillimetre,
                  predicted.tail<3>().norm() / kDegree,
                  actual[0] / kMillimetre, actual[1] / kMillimetre,
                  actual[2] / kMillimetre);
    }
  }
  std::printf("calibration error: the answer moved as the camera did, to "
              "within %.4f mm and %.5f deg\n",
              worst_position / kMillimetre, worst_angle / kDegree);
}

// ------------------------------------------------------------- refusals

// Nothing on the tool: the camera sees the tool's plate and cups, a third
// of the pixels a module would cover.
TEST_F(HeldModuleFinderTest, RefusesWithNoModuleOnTheTool) {
  const CheckShot shot = rig_->TakeEmpty(rig_->nominal_tip());
  HeldFinderDiagnostics why;
  EXPECT_FALSE(cell::FindHeldModule(WithNoise(shot.image, 3), shot.camera,
                                    Expected(shot), {}, &why)
                   .has_value());
  std::printf("no module: %s: %s\n", cell::HeldFinderFailureName(why.failure),
              why.detail.c_str());
  EXPECT_EQ(why.failure, HeldFinderFailure::kTooFewPoints) << why.detail;
  EXPECT_GT(why.points_in_crop, 0);  // the tool's plate and cups
}

// The arm stopped 1.3 m off its check pose: half the module is out of the
// picture. The expected pose, from the same tool, says so before anything
// is looked at.
TEST_F(HeldModuleFinderTest, RefusesAModuleHalfOutOfView) {
  Isometry3d tip = rig_->nominal_tip();
  tip.translation().x() += 1.3;
  std::string error;
  const std::optional<CheckShot> shot =
      rig_->Take(tip, Isometry3d::Identity(), &error);
  ASSERT_TRUE(shot.has_value()) << error;
  HeldFinderDiagnostics why;
  EXPECT_FALSE(cell::FindHeldModule(shot->image, shot->camera,
                                    Expected(*shot), {}, &why)
                   .has_value());
  std::printf("half out of view: %s: %s\n",
              cell::HeldFinderFailureName(why.failure), why.detail.c_str());
  EXPECT_EQ(why.failure, HeldFinderFailure::kNotInView) << why.detail;
}

// The module held 0.35 m along its length from where the control code
// thinks: the crop, round the expected pose, cuts it short. And held 8 cm
// off: found, whole, but further off than a grip can be: it has slipped.
TEST_F(HeldModuleFinderTest, RefusesAModuleNotWhereTheToolShouldHoldIt) {
  std::string error;
  {
    const std::optional<CheckShot> shot = rig_->Take(
        rig_->nominal_tip(),
        cell::GripOffset(Vector3d(0.35, 0, 0), Vector3d::Zero()), &error);
    ASSERT_TRUE(shot.has_value()) << error;
    HeldFinderDiagnostics why;
    EXPECT_FALSE(cell::FindHeldModule(WithNoise(shot->image, 4),
                                      shot->camera, Expected(*shot), {}, &why)
                     .has_value());
    std::printf("0.35 m off: %s: %s\n",
                cell::HeldFinderFailureName(why.failure), why.detail.c_str());
    EXPECT_TRUE(why.failure == HeldFinderFailure::kSizeMismatch ||
                why.failure == HeldFinderFailure::kNoRectangle)
        << cell::HeldFinderFailureName(why.failure) << ": " << why.detail;
  }
  {
    const std::optional<CheckShot> shot = rig_->Take(
        rig_->nominal_tip(),
        cell::GripOffset(Vector3d(0.08, 0, 0), Vector3d::Zero()), &error);
    ASSERT_TRUE(shot.has_value()) << error;
    HeldFinderDiagnostics why;
    EXPECT_FALSE(cell::FindHeldModule(WithNoise(shot->image, 5),
                                      shot->camera, Expected(*shot), {}, &why)
                     .has_value());
    std::printf("8 cm off: %s: %s\n",
                cell::HeldFinderFailureName(why.failure), why.detail.c_str());
    EXPECT_EQ(why.failure, HeldFinderFailure::kFarFromExpected) << why.detail;
    EXPECT_NEAR(why.offset_from_expected.x(), 0.08, 1.0 * kMillimetre);
  }
}

// A module of another product, 2.0 m x 1.0 m (the rig's module_1), held
// where the design module would be: refused as the wrong size. And the
// right module with the stage told the wrong width. (Told the wrong length,
// the crop, sized from that length, would cut the module's ends itself,
// and the refusal could as well be that a cut end is not square.)
TEST_F(HeldModuleFinderTest, RefusesARectangleOfTheWrongSize) {
  std::string error;
  const std::optional<CheckShot> other = rig_->Take(
      rig_->nominal_tip(), Isometry3d::Identity(), &error, 0.2, /*module=*/1);
  ASSERT_TRUE(other.has_value()) << error;
  HeldFinderDiagnostics why;
  EXPECT_FALSE(cell::FindHeldModule(WithNoise(other->image, 6), other->camera,
                                    Expected(*other), {}, &why)
                   .has_value());
  EXPECT_EQ(why.failure, HeldFinderFailure::kSizeMismatch) << why.detail;
  std::printf("a 2.0 x 1.0 m module: %s: %s\n",
              cell::HeldFinderFailureName(why.failure), why.detail.c_str());

  const CheckShot& shot = Take(2);
  cell::HeldFinderOptions narrower;
  narrower.module_width = 1.0;
  EXPECT_FALSE(cell::FindHeldModule(WithNoise(shot.image, 6), shot.camera,
                                    Expected(shot), narrower, &why)
                   .has_value());
  EXPECT_EQ(why.failure, HeldFinderFailure::kSizeMismatch) << why.detail;
}

// Something between the camera and the underside (a patch of the picture
// nearer than the module), and a hole (a patch with no reading).
TEST_F(HeldModuleFinderTest, RefusesABlockedView) {
  const CheckShot& shot = Take(3);
  for (bool hole : {false, true}) {
    SCOPED_TRACE(testing::Message() << (hole ? "hole" : "in front"));
    DepthImage image = WithNoise(shot.image, 7);
    for (int v = 210; v < 270; ++v) {
      for (int u = 394; u < 454; ++u) {
        image.at(u, v) = hole ? std::nanf("") : 1.0f;
      }
    }
    HeldFinderDiagnostics why;
    EXPECT_FALSE(cell::FindHeldModule(image, shot.camera, Expected(shot), {},
                                      &why)
                     .has_value());
    EXPECT_EQ(why.failure, HeldFinderFailure::kViewBlocked) << why.detail;
    EXPECT_GE(hole ? why.view.missing : why.view.in_front, 3600);
    std::printf("%s: %s\n", hole ? "hole" : "in front", why.detail.c_str());
  }
}

TEST_F(HeldModuleFinderTest, RefusesBadInput) {
  const CheckShot& shot = Take(1);
  HeldFinderDiagnostics why;
  EXPECT_FALSE(cell::FindHeldModule(DepthImage{}, shot.camera, Expected(shot),
                                    {}, &why)
                   .has_value());
  EXPECT_EQ(why.failure, HeldFinderFailure::kBadInput);
  // An expected pose facing away from the camera: the module upside down.
  Isometry3d flipped = Expected(shot);
  flipped.linear() = flipped.linear() *
                     Eigen::AngleAxisd(std::numbers::pi, Vector3d::UnitX())
                         .toRotationMatrix();
  EXPECT_FALSE(cell::FindHeldModule(shot.image, shot.camera, flipped, {}, &why)
                   .has_value());
  EXPECT_EQ(why.failure, HeldFinderFailure::kNotInView);
}

// ---------------------------------------------------------- determinism

TEST_F(HeldModuleFinderTest, SamePictureGivesTheSameAnswer) {
  const CheckShot& shot = Take(4);
  const DepthImage noisy = WithNoise(shot.image, 9);
  const std::optional<HeldModuleEstimate> first =
      cell::FindHeldModule(noisy, shot.camera, Expected(shot));
  const std::optional<HeldModuleEstimate> second =
      cell::FindHeldModule(noisy, shot.camera, Expected(shot));
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->pose.matrix(), second->pose.matrix());
  EXPECT_EQ(first->diagnostics.plane_inliers,
            second->diagnostics.plane_inliers);

  // And the same grip taken again gives the same picture.
  std::string error;
  const std::optional<CheckShot> again = rig_->Take(
      TipFor(4), cell::DrawGripOffset(4, kOffsetPosition, kOffsetAngle),
      &error);
  ASSERT_TRUE(again.has_value()) << error;
  EXPECT_EQ(again->truth_in_tip.matrix(), shot.truth_in_tip.matrix());
  ASSERT_EQ(again->image.depth.size(), shot.image.depth.size());
  EXPECT_TRUE(std::equal(again->image.depth.begin(), again->image.depth.end(),
                         shot.image.depth.begin(), [](float a, float b) {
                           return (std::isnan(a) && std::isnan(b)) || a == b;
                         }));
}

// ------------------------------------------------------- what it is for

// The place, computed from kinematics (no set-down simulated): the module
// is to lie flat on the tube at (-2.0, 0, 0.8175), long side along X. A tip
// target from the design grip lands the module off by exactly the grip
// offset; one from the measured grip lands it within the measurement's
// error.
TEST_F(HeldModuleFinderTest, PlacementWithAndWithoutTheCheck) {
  Isometry3d target = Isometry3d::Identity();
  target.translation() = Vector3d(-2.0, 0.0, 0.8175);
  Spread without, with;
  for (std::uint32_t seed = 1; seed <= kShots; ++seed) {
    SCOPED_TRACE(testing::Message() << "seed " << seed);
    const CheckShot& shot = Take(seed);
    const std::optional<HeldModuleEstimate>& estimate = Noisy(seed);
    ASSERT_TRUE(estimate.has_value());
    const std::optional<Isometry3d> measured =
        cell::GripOffsetFromCheck(estimate->pose, shot.tip);

    // Where the module lands for a tip target: tip * truly held offset.
    const Isometry3d blind =
        cell::CorrectedTipTarget(target, cell::DesignModuleInTip());
    const Isometry3d checked = cell::CorrectedTipTarget(target, *measured);
    const Six miss = cell::PoseError6(blind * shot.truth_in_tip, target);
    const Six hit = cell::PoseError6(checked * shot.truth_in_tip, target);

    // Without the check the miss is the grip offset itself.
    const Six offset =
        cell::PoseError6(shot.truth_in_tip, cell::DesignModuleInTip());
    EXPECT_LT((miss - offset).norm(), 1e-9);
    // With it, the measurement's error.
    ExpectWithin(hit, kNoisyInPlane, kNoisyNormal, kNoisyTilt, kNoisyYaw);
    without.Add(miss);
    with.Add(hit);
  }
  without.Print("placement without the check");
  with.Print("placement with the check   ");
  // The offsets drawn are millimetres; the check takes them to tenths.
  EXPECT_GT(without.worst.head<2>().maxCoeff(), 5.0 * kMillimetre);
}

}  // namespace
