// Checks the check camera in the real cell (models/cell.xml, check_camera on
// check_camera_stand): it stands where the cell file says, and with the arm
// at the check pose and a module on the tool the check after pickup
// (src/held_module_finder.h) recovers where the module sits on the tool from
// Cell::TakeCheckImage, with the pallet camera's post and boom somewhere
// above and the arm itself around. Held at the design grip the offset is
// zero; displaced by a known amount, the displacement; with nothing held
// the finder refuses.
//
//   bazel test --config=opt //:check_camera_cell_test --test_output=all
//
// The scene's path comes from MODEL_PATH, set by BUILD.bazel. The module is
// put on the tool by setting its free joint (Cell::TeleportModule) and
// calling mj_forward: no physics runs, so the truth is exact.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/camera_model.h"
#include "src/cell.h"
#include "src/check_rig.h"
#include "src/held_module_finder.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/pallet.h"

namespace {

using cell::Cell;
using Eigen::Isometry3d;
using Eigen::Vector3d;

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr double kGripGap = 0.002;
constexpr double kMillimetre = 0.001;

class CheckCameraCellTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr);
    std::string error;
    cell_ = Cell::Load(path, &error);
    ASSERT_TRUE(cell_.has_value()) << error;
    ASSERT_TRUE(cell_->Reset(1, cell::Randomization::None()));
    Eigen::VectorXd q(6);
    q << -85.00, -90.81, 81.68, 0.0, 99.13, 90.0;
    q *= kDegree;
    check_pose_ = q;
    cell_->PutArm(q);
    cell::DataPtr scratch(mj_makeData(cell_->model()));
    tip_ = cell::TipPoseAt(*cell_->model(), scratch.get(), cell_->arm(), q);
    expected_ = tip_ * cell::DesignModuleInTip(cell::kModuleThickness, kGripGap);
  }

  // The finder on the cell's check image with module_0 held at the design
  // grip times `offset`; the answer as the error against `offset`.
  static std::optional<Eigen::Matrix<double, 6, 1>> Recover(
      const Isometry3d& offset, cell::HeldFinderDiagnostics* why = nullptr) {
    cell_->TeleportModule(0, expected_ * offset);
    const cell::DepthImage image = cell_->TakeCheckImage();
    const std::optional<cell::HeldModuleEstimate> found = cell::FindHeldModule(
        image, cell_->CalibratedCheckCamera(), expected_, {}, why);
    if (!found) return std::nullopt;
    const Isometry3d measured = cell::GripOffsetFromCheck(found->pose, tip_);
    const Isometry3d truth =
        cell::DesignModuleInTip(cell::kModuleThickness, kGripGap) * offset;
    return cell::PoseError6(measured, truth);
  }

  static inline std::optional<Cell> cell_;
  static inline Eigen::VectorXd check_pose_;
  static inline Isometry3d tip_ = Isometry3d::Identity();
  static inline Isometry3d expected_ = Isometry3d::Identity();
};

void ExpectWithin(const Eigen::Matrix<double, 6, 1>& error) {
  std::printf("  error: %.4f %.4f %.4f mm, %.4f %.4f %.4f deg\n",
              error[0] * 1e3, error[1] * 1e3, error[2] * 1e3,
              error[3] / kDegree, error[4] / kDegree, error[5] / kDegree);
  EXPECT_LT(error.head<3>().norm(), 0.5 * kMillimetre);
  EXPECT_LT(error.tail<3>().norm(), 0.03 * kDegree);
}

TEST_F(CheckCameraCellTest, CameraIsWhereTheCellFileSaysAndLooksUp) {
  EXPECT_GE(cell_->check_camera_id(), 0);
  const cell::CameraModel camera = cell_->CheckCamera();
  EXPECT_NEAR(camera.pose.translation().x(), 0.1307, 1e-9);
  EXPECT_NEAR(camera.pose.translation().y(), 1.4943, 1e-9);
  EXPECT_NEAR(camera.pose.translation().z(), 0.15, 1e-9);
  // Its own -Z, the way it looks, is world +Z; its +X is world +X.
  EXPECT_NEAR((camera.pose.linear() * -Vector3d::UnitZ()).z(), 1.0, 1e-9);
  EXPECT_NEAR((camera.pose.linear() * Vector3d::UnitX()).x(), 1.0, 1e-9);
  EXPECT_EQ(camera.width, 848);
  EXPECT_EQ(camera.height, 480);
  EXPECT_NEAR(camera.fovy / kDegree, 61.0, 1e-9);
  // Exact calibration by default.
  EXPECT_TRUE(cell_->CalibratedCheckCamera().pose.isApprox(camera.pose));

  // Where the module's underside is, relative to the lens: straight above,
  // about 1.4 m, and the camera's rows turned from the module's long side.
  const Vector3d centre = expected_.translation();
  const Vector3d x = expected_.linear().col(0);
  std::printf("check pose: tip (%.4f %.4f %.4f), module centre (%.4f %.4f %.4f)"
              ", long side at %.2f deg from camera +X\n",
              tip_.translation().x(), tip_.translation().y(),
              tip_.translation().z(), centre.x(), centre.y(), centre.z(),
              std::atan2(x.y(), x.x()) / kDegree);
  EXPECT_NEAR(centre.x(), 0.1307, 2e-3);
  EXPECT_NEAR(centre.y(), 1.4943, 2e-3);
}

TEST_F(CheckCameraCellTest, HeldAtTheDesignGripRecoversZeroOffset) {
  cell::HeldFinderDiagnostics why;
  const auto error = Recover(Isometry3d::Identity(), &why);
  ASSERT_TRUE(error.has_value()) << cell::HeldFinderFailureName(why.failure)
                                 << ": " << why.detail;
  std::printf("design grip:\n");
  ExpectWithin(*error);
}

TEST_F(CheckCameraCellTest, DisplacedModuleRecoversTheDisplacement) {
  const Isometry3d offset = cell::GripOffset(
      Vector3d(4.0, -3.0, 2.0) * kMillimetre,
      Vector3d(0.3, -0.4, 1.0) * kDegree);  // about 1.1 degrees in all
  cell::HeldFinderDiagnostics why;
  const auto error = Recover(offset, &why);
  ASSERT_TRUE(error.has_value()) << cell::HeldFinderFailureName(why.failure)
                                 << ": " << why.detail;
  std::printf("displaced 4 -3 2 mm, 0.3 -0.4 1.0 deg:\n");
  ExpectWithin(*error);
}

TEST_F(CheckCameraCellTest, NothingHeldIsRefused) {
  // Module 0 on the floor out of view, as parking leaves it.
  cell_->ParkModule(0);
  const cell::DepthImage image = cell_->TakeCheckImage();
  cell::HeldFinderDiagnostics why;
  EXPECT_FALSE(cell::FindHeldModule(image, cell_->CalibratedCheckCamera(),
                                    expected_, {}, &why)
                   .has_value());
  std::printf("nothing held: %s: %s\n", cell::HeldFinderFailureName(why.failure),
              why.detail.c_str());
  EXPECT_NE(why.failure, cell::HeldFinderFailure::kNone);
}

}  // namespace
