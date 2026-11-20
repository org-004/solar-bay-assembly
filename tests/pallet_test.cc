// Checks the pallet scene: the modules are where and what they should be,
// the stack stands still when left alone, wherever the pallet is put, and
// the front module comes off without dragging the next one along.
//
//   bazel test //:pallet_test
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test.
//
// Every check of stillness starts after kSettleTime: the modules are built
// touching but not yet pressed together, and the stack takes a moment to
// take up its own weight. Positions and angles are then compared with where
// each module was at the end of settling.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/mj_handles.h"
#include "src/pallet.h"

namespace {

constexpr int kModuleCount = 30;
constexpr double kPi = 3.14159265358979323846;
constexpr double kLean = 10.0 * kPi / 180.0;  // rad, the pallet's tilt
constexpr double kSettleTime = 1.0;           // s

// Still means: no module moves further than this from where it settled, at
// any instant...
constexpr double kStillDistance = 0.1e-3;  // m
constexpr double kStillAngle = 0.1e-3;     // rad
// ...neither half of the watch moves it further than this, so the stack is
// not creeping at a steady rate either...
constexpr double kDriftPerHalf = 0.02e-3;  // m
// ...and at the end it is hardly moving.
constexpr double kEndSpeed = 0.01e-3;  // m/s
constexpr double kEndSpin = 0.1e-3;    // rad/s

// What a watch of the stack saw: the largest value over the modules watched.
struct Stillness {
  double distance = 0;     // furthest from the start, at any instant
  double angle = 0;        // largest turn from the start, at any instant
  double first_half = 0;   // start to halfway
  double second_half = 0;  // halfway to end
  double speed = 0;        // at the end
  double spin = 0;         // at the end
};

// Prints a Stillness on failure, in millimetres and milliradians.
std::string Describe(const Stillness& s) {
  return "moved " + std::to_string(1e3 * s.distance) + " mm, turned " +
         std::to_string(1e3 * s.angle) + " mrad, halves " +
         std::to_string(1e3 * s.first_half) + " / " +
         std::to_string(1e3 * s.second_half) + " mm, end " +
         std::to_string(1e3 * s.speed) + " mm/s " +
         std::to_string(1e3 * s.spin) + " mrad/s";
}

class PalletTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";

    std::string error;
    model_ = cell::LoadModel(path, &error);
    ASSERT_TRUE(model_) << "could not load " << path << ": " << error;
    data_ = cell::MakeData(*model_);
    ASSERT_TRUE(data_);

    std::optional<cell::PalletBodies> bodies = cell::FindPallet(*model_);
    ASSERT_TRUE(bodies) << "no pallet with modules in " << path;
    bodies_ = *bodies;
    mj_forward(model_.get(), data_.get());
  }

  void StepFor(double seconds) {
    const double end = data_->time + seconds;
    while (data_->time < end) {
      Push();
      mj_step(model_.get(), data_.get());
    }
  }

  int count() const { return static_cast<int>(bodies_.modules.size()); }

  cell::WorldPose Pose(int i) const {
    return cell::ModulePose(*data_, bodies_, i);
  }

  std::vector<cell::WorldPose> Poses() const {
    std::vector<cell::WorldPose> poses;
    for (int i = 0; i < count(); ++i) poses.push_back(Pose(i));
    return poses;
  }

  // Linear and angular speed of module i, in m/s and rad/s.
  std::pair<double, double> Speed(int i) const {
    mjtNum velocity[6];  // angular, then linear; world axes
    mj_objectVelocity(model_.get(), data_.get(), mjOBJ_BODY,
                      bodies_.modules[i], velocity, /*flg_local=*/0);
    return {Eigen::Vector3d(velocity[3], velocity[4], velocity[5]).norm(),
            Eigen::Vector3d(velocity[0], velocity[1], velocity[2]).norm()};
  }

  // Simulates for `seconds` and reports how much modules `first` onward
  // moved, compared with where they were at the start.
  Stillness Watch(double seconds, int first = 0) {
    const std::vector<cell::WorldPose> start = Poses();
    std::vector<cell::WorldPose> halfway;
    Stillness seen;
    const double middle = data_->time + seconds / 2;
    const double end = data_->time + seconds;
    while (data_->time < end) {
      Push();
      mj_step(model_.get(), data_.get());
      for (int i = first; i < count(); ++i) {
        const cell::WorldPose now = Pose(i);
        seen.distance = std::max(
            seen.distance, (now.position - start[i].position).norm());
        seen.angle = std::max(
            seen.angle, now.orientation.angularDistance(start[i].orientation));
      }
      if (halfway.empty() && data_->time >= middle) halfway = Poses();
    }
    for (int i = first; i < count(); ++i) {
      const cell::WorldPose now = Pose(i);
      seen.first_half = std::max(
          seen.first_half, (halfway[i].position - start[i].position).norm());
      seen.second_half = std::max(
          seen.second_half, (now.position - halfway[i].position).norm());
      const auto [speed, spin] = Speed(i);
      seen.speed = std::max(seen.speed, speed);
      seen.spin = std::max(seen.spin, spin);
    }
    return seen;
  }

  void ExpectStill(const Stillness& seen) {
    EXPECT_LT(seen.distance, kStillDistance) << Describe(seen);
    EXPECT_LT(seen.angle, kStillAngle) << Describe(seen);
    EXPECT_LT(seen.first_half, kDriftPerHalf) << Describe(seen);
    EXPECT_LT(seen.second_half, kDriftPerHalf) << Describe(seen);
    EXPECT_LT(seen.speed, kEndSpeed) << Describe(seen);
    EXPECT_LT(seen.spin, kEndSpin) << Describe(seen);
  }

  // A stand-in for a suction tool on an arm: while `holding_` is set,
  // module_0 is pulled toward `hold_position_` and `hold_orientation_` by a
  // stiff spring with a damper, applied as an outside force each step, and
  // its weight is carried. Only module_0 is touched: anything else that
  // moves is moved by the contacts.
  void Push() {
    if (!holding_) return;
    const int body = bodies_.modules[0];
    const cell::WorldPose now = Pose(0);
    mjtNum velocity[6];  // angular, then linear; world axes
    mj_objectVelocity(model_.get(), data_.get(), mjOBJ_BODY, body, velocity, 0);
    const Eigen::Vector3d linear(velocity[3], velocity[4], velocity[5]);
    const Eigen::Vector3d angular(velocity[0], velocity[1], velocity[2]);
    const Eigen::Vector3d gravity(model_->opt.gravity[0], model_->opt.gravity[1],
                                  model_->opt.gravity[2]);

    // About 5 Hz, critically damped, for a 30 kg module: stiff enough to
    // track a slow move, soft enough for a 2 ms step.
    const double mass = model_->body_mass[body];
    const double omega = 2 * kPi * 5;
    const Eigen::Vector3d force =
        mass * (omega * omega * (hold_position_ - now.position) -
                2 * omega * linear - gravity);
    const Eigen::AngleAxisd error(hold_orientation_ * now.orientation.conjugate());
    const double inertia = 15;  // kg m^2, a little above the largest axis
    const Eigen::Vector3d torque =
        inertia * (omega * omega * error.angle() * error.axis() -
                   2 * omega * angular);

    mjtNum* applied = data_->xfrc_applied + 6 * body;
    for (int k = 0; k < 3; ++k) {
      applied[k] = force[k];
      applied[3 + k] = torque[k];
    }
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  cell::PalletBodies bodies_;

  bool holding_ = false;
  Eigen::Vector3d hold_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond hold_orientation_ = Eigen::Quaterniond::Identity();
};

TEST_F(PalletTest, ThirtyModulesNamedInOrderFromTheFront) {
  ASSERT_EQ(count(), kModuleCount);
  for (int i = 0; i < count(); ++i) {
    const char* name = mj_id2name(model_.get(), mjOBJ_BODY, bodies_.modules[i]);
    ASSERT_NE(name, nullptr);
    EXPECT_EQ(std::string(name), "module_" + std::to_string(i));
  }

  // The pallet holds still in the physics and has no joints.
  EXPECT_GE(model_->body_mocapid[bodies_.pallet], 0);
  EXPECT_EQ(model_->body_jntnum[bodies_.pallet], 0);

  // Front to back: each module's glass is further from the arm than the
  // one before.
  for (int i = 1; i < count(); ++i) {
    EXPECT_GT(cell::ModuleGlass(*data_, bodies_, i).centre.x(),
              cell::ModuleGlass(*data_, bodies_, i - 1).centre.x())
        << "module_" << i;
  }
}

TEST_F(PalletTest, EachModuleIsOneBoxOfTheRightSizeAndMass) {
  for (int i = 0; i < count(); ++i) {
    const int body = bodies_.modules[i];
    EXPECT_NEAR(model_->body_mass[body], cell::kModuleMass, 1e-9) << i;
    ASSERT_EQ(model_->body_geomnum[body], 1) << i;
    const int geom = model_->body_geomadr[body];
    EXPECT_EQ(model_->geom_type[geom], mjGEOM_BOX) << i;
    // Box sizes are half-lengths. The box sits at the body's origin with
    // the body's axes, so the glass is exactly at z = +thickness / 2.
    EXPECT_DOUBLE_EQ(2 * model_->geom_size[3 * geom], cell::kModuleLength);
    EXPECT_DOUBLE_EQ(2 * model_->geom_size[3 * geom + 1], cell::kModuleWidth);
    EXPECT_DOUBLE_EQ(2 * model_->geom_size[3 * geom + 2],
                     cell::kModuleThickness);
    for (int k = 0; k < 3; ++k) {
      EXPECT_EQ(model_->geom_pos[3 * geom + k], 0.0) << i;
    }
    EXPECT_EQ(model_->geom_quat[4 * geom], 1.0) << i;
  }
}

TEST_F(PalletTest, StackLeansBackWithGlassTowardTheArm) {
  StepFor(kSettleTime);

  for (int i = 0; i < count(); ++i) {
    const cell::WorldPose pose = Pose(i);
    const Eigen::Matrix3d axes = pose.orientation.toRotationMatrix();
    const Eigen::Vector3d long_side = axes.col(0);
    const Eigen::Vector3d up = axes.col(1);
    const Eigen::Vector3d glass = axes.col(2);

    // The long side is level and runs across, along world Y.
    EXPECT_NEAR(long_side.dot(-Eigen::Vector3d::UnitY()), 1.0, 1e-6) << i;
    // The short side leans back, away from the arm, by the tilt.
    EXPECT_NEAR(std::acos(up.z()), kLean, 1e-3) << i;  // 0.06 degrees
    EXPECT_GT(up.x(), 0) << i;
    // The glass faces the arm, tipped up by the same angle.
    EXPECT_NEAR(glass.x(), -std::cos(kLean), 1e-3) << i;
    EXPECT_NEAR(glass.z(), std::sin(kLean), 1e-3) << i;
  }

  // The front glass centre rests where pallet.xml puts it: 0.75 m straight
  // above the pallet origin, wherever the file puts the pallet. And the
  // faces are one module thickness apart.
  const cell::GlassFace front = cell::ModuleGlass(*data_, bodies_, 0);
  const Eigen::Vector3d expected_front =
      bodies_.nominal_pallet.position + Eigen::Vector3d(0, 0, 0.75);
  EXPECT_LT((front.centre - expected_front).norm(), 1e-3);
  EXPECT_GT(front.normal.dot(-Eigen::Vector3d::UnitX()), 0.98);
  for (int i = 1; i < count(); ++i) {
    const cell::GlassFace ahead = cell::ModuleGlass(*data_, bodies_, i - 1);
    const cell::GlassFace face = cell::ModuleGlass(*data_, bodies_, i);
    EXPECT_NEAR((ahead.centre - face.centre).dot(ahead.normal),
                cell::kModuleThickness, 0.1e-3)
        << "between module_" << i - 1 << " and module_" << i;
  }
}

TEST_F(PalletTest, UntouchedStackStaysStillForThirtySeconds) {
  StepFor(kSettleTime);
  const Stillness seen = Watch(30.0);
  ExpectStill(seen);
  std::printf("untouched, 30 s: %s\n", Describe(seen).c_str());
}

TEST_F(PalletTest, FrontModuleComesOffWithoutDraggingTheNext) {
  StepFor(kSettleTime);
  const cell::WorldPose start = Pose(0);
  const std::vector<cell::WorldPose> before = Poses();
  const Eigen::Matrix3d axes = start.orientation.toRotationMatrix();
  const Eigen::Vector3d up = axes.col(1);     // up along the glass
  const Eigen::Vector3d glass = axes.col(2);  // out of the glass

  // Lift 10 mm up along the glass, then pull 0.3 m straight off the face,
  // each move eased in and out, then hold it there.
  const auto ease = [](double f) {
    f = std::clamp(f, 0.0, 1.0);
    return f * f * (3 - 2 * f);
  };
  const double lift = 0.010, lift_time = 0.5;
  const double pull = 0.300, pull_time = 1.0;
  holding_ = true;
  hold_orientation_ = start.orientation;
  double worst_drag = 0;  // furthest module_1 got from where it was
  const double begin = data_->time;
  while (data_->time < begin + lift_time + pull_time + 0.5) {
    const double t = data_->time - begin;
    hold_position_ = start.position + lift * ease(t / lift_time) * up +
                     pull * ease((t - lift_time) / pull_time) * glass;
    Push();
    mj_step(model_.get(), data_.get());
    worst_drag = std::max(worst_drag,
                          (Pose(1).position - before[1].position).norm());
  }

  // module_0 is off the stack, its back well clear of module_1's glass...
  const cell::GlassFace next = cell::ModuleGlass(*data_, bodies_, 1);
  const double clearance =
      (Pose(0).position - next.centre).dot(next.normal) -
      cell::kModuleThickness / 2;
  EXPECT_GT(clearance, 0.25);
  // ...and module_1 barely noticed.
  EXPECT_LT(worst_drag, 0.5e-3);

  // What is left stands still, and where it stood before.
  const Stillness seen = Watch(5.0, /*first=*/1);
  ExpectStill(seen);
  for (int i = 1; i < count(); ++i) {
    EXPECT_LT((Pose(i).position - before[i].position).norm(), 0.5e-3)
        << "module_" << i;
  }
  std::printf("pick: module_1 dragged %.4f mm, module_0 clear by %.3f m; "
              "after: %s\n",
              1e3 * worst_drag, clearance, Describe(seen).c_str());
}

// Settles the stack at the nominal pose and at a few others, and checks it
// stands still at each, the same way relative to the pallet.
TEST_F(PalletTest, MovedPalletStillGivesAStillStack) {
  const int deck = mj_name2id(model_.get(), mjOBJ_GEOM, "pallet_deck");
  ASSERT_GE(deck, 0);
  const Eigen::Vector3d deck_home(data_->geom_xpos[3 * deck],
                                  data_->geom_xpos[3 * deck + 1],
                                  data_->geom_xpos[3 * deck + 2]);
  StepFor(kSettleTime);
  const cell::WorldPose pallet_home = cell::PalletPose(*data_, bodies_);
  const std::vector<cell::WorldPose> home = Poses();

  struct Offset {
    double x, y, yaw;
  };
  for (const Offset& offset : {Offset{0.05, -0.04, 0.05},
                               Offset{-0.03, 0.06, -0.08}}) {
    SCOPED_TRACE("offset " + std::to_string(offset.x) + ", " +
                 std::to_string(offset.y) + ", yaw " +
                 std::to_string(offset.yaw));
    mj_resetData(model_.get(), data_.get());
    cell::PlacePallet(*model_, data_.get(), bodies_, offset.x, offset.y,
                      offset.yaw);

    // The pallet went where it was told, and its deck went with it.
    const Eigen::Quaterniond turn(
        Eigen::AngleAxisd(offset.yaw, Eigen::Vector3d::UnitZ()));
    const cell::WorldPose pallet = cell::PalletPose(*data_, bodies_);
    EXPECT_LT((pallet.position - pallet_home.position -
               Eigen::Vector3d(offset.x, offset.y, 0))
                  .norm(),
              1e-12);
    EXPECT_LT(pallet.orientation.angularDistance(turn * pallet_home.orientation),
              1e-12);
    const Eigen::Vector3d deck_at(data_->geom_xpos[3 * deck],
                                  data_->geom_xpos[3 * deck + 1],
                                  data_->geom_xpos[3 * deck + 2]);
    EXPECT_LT((deck_at - (pallet.position +
                          turn * (deck_home - pallet_home.position)))
                  .norm(),
              1e-9);

    StepFor(kSettleTime);
    const Stillness seen = Watch(10.0);
    ExpectStill(seen);
    std::printf("moved %.2f, %.2f m, %.2f rad, 10 s: %s\n", offset.x, offset.y,
                offset.yaw, Describe(seen).c_str());

    // Each module rests where it rested on the unmoved pallet, carried
    // along with the pallet.
    for (int i = 0; i < count(); ++i) {
      const Eigen::Vector3d expected =
          pallet.position + turn * (home[i].position - pallet_home.position);
      EXPECT_LT((Pose(i).position - expected).norm(), 0.05e-3)
          << "module_" << i;
      EXPECT_LT(Pose(i).orientation.angularDistance(turn * home[i].orientation),
                0.05e-3)
          << "module_" << i;
    }
  }
}

TEST_F(PalletTest, ResetPutsPalletAndModulesBack) {
  const std::vector<cell::WorldPose> built = Poses();
  const cell::WorldPose pallet_built = cell::PalletPose(*data_, bodies_);
  cell::PlacePallet(*model_, data_.get(), bodies_, 0.1, 0.1, 0.1);
  mj_resetData(model_.get(), data_.get());
  mj_forward(model_.get(), data_.get());
  EXPECT_LT((cell::PalletPose(*data_, bodies_).position - pallet_built.position)
                .norm(),
            1e-12);
  for (int i = 0; i < count(); ++i) {
    EXPECT_LT((Pose(i).position - built[i].position).norm(), 1e-12) << i;
  }
}

}  // namespace
