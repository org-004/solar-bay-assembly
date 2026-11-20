// Checks the suction grip on the test bench, models/suction_scene.xml: the
// tool rides on a carriage of slides and hinges instead of the arm.
//
//   bazel test //:suction_test
//
// A grip must be refused, with the right reason, when the tool is not on the
// glass; must pick the module the tool is on and no other; must hold the
// module at the pose it had at the grip, misalignment included, through the
// whole carry and over a long hold; and a release must let the module go and
// settle on the rest. The carriage is steered along S-curve joint moves.
//
// The numbers measured (sag, creep, peak acceleration) are printed in the
// test log, bazel-testlogs/suction_test/test.log.
//
// The model's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/joint_move.h"
#include "src/mj_handles.h"
#include "src/suction.h"

namespace {

using cell::GripFailure;
using cell::JointLimits;
using cell::JointMove;
using cell::Pose;
using cell::Suction;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegree = kPi / 180.0;

// The carriage's joints, in the order of a joint vector here.
constexpr std::array<const char*, 5> kRigJoints = {"rig_x", "rig_y", "rig_z",
                                                   "rig_yaw", "rig_pitch"};
enum RigJoint { kX = 0, kY, kZ, kYaw, kPitch };

// Module size, from the contract.
constexpr double kHalfLength = 1.15;
constexpr double kHalfWidth = 0.55;
constexpr double kHalfThickness = 0.0175;

// The rails in the scene: two along Y, tops at z = 0.3.
constexpr double kRailTop = 0.3;
constexpr double kRailNearX = -1.1;
constexpr double kRailFarX = -1.9;
constexpr double kRailHalfWidth = 0.04;

// How far a held module may drift from the pose it had at the grip, at any
// moment of a carry with accelerations of a few m/s^2. The weld gives about
// 0.07 mm and 0.02 degrees at the worst moment of the carry here, and far
// less held still; the tolerance is a few times that.
constexpr double kHeldPositionTolerance = 0.00025;      // m
constexpr double kHeldAngleTolerance = 0.05 * kDegree;  // rad

// Moves for positioning, and a hard one for shaking a held module.
JointLimits Limits(double velocity, double acceleration, double jerk) {
  JointLimits limits;
  limits.velocity = VectorXd::Constant(5, velocity);
  limits.acceleration = VectorXd::Constant(5, acceleration);
  limits.jerk = VectorXd::Constant(5, jerk);
  return limits;
}
const JointLimits kGentle = Limits(0.5, 2.0, 20.0);
const JointLimits kBrisk = Limits(1.5, 5.0, 60.0);

double AngleBetween(const Quaterniond& a, const Quaterniond& b) {
  return a.angularDistance(b);
}

// The worst difference seen between a held module's pose and its pose at
// the grip.
struct Drift {
  double position = 0.0;  // m
  double angle = 0.0;     // rad

  void Add(const Pose& now, const Pose& at_grip) {
    position = std::max(position, (now.position - at_grip.position).norm());
    angle = std::max(angle, AngleBetween(now.rotation, at_grip.rotation));
  }
};

class SuctionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";

    std::string error;
    model_ = cell::LoadModel(path, &error);
    ASSERT_TRUE(model_) << "could not load " << path << ": " << error;
    data_ = cell::MakeData(*model_);
    ASSERT_TRUE(data_);

    std::optional<Suction> suction = Suction::Create(*model_, {}, &error);
    ASSERT_TRUE(suction.has_value()) << error;
    suction_.emplace(std::move(*suction));

    for (int i = 0; i < 5; ++i) {
      const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, kRigJoints[i]);
      const int actuator =
          mj_name2id(model_.get(), mjOBJ_ACTUATOR, kRigJoints[i]);
      ASSERT_GE(joint, 0) << kRigJoints[i];
      ASSERT_GE(actuator, 0) << kRigJoints[i];
      qpos_address_[i] = model_->jnt_qposadr[joint];
      actuator_[i] = actuator;
    }
    for (int i = 0; i < 3; ++i) {
      const std::string name = "module_" + std::to_string(i);
      module_[i] = mj_name2id(model_.get(), mjOBJ_BODY, name.c_str());
      ASSERT_GE(module_[i], 0) << name;
    }
    tip_ = mj_name2id(model_.get(), mjOBJ_SITE, "tool_tip");
    weld_ = mj_name2id(model_.get(), mjOBJ_EQUALITY, "suction");
    ASSERT_GE(weld_, 0);
    const int pitch_body = mj_name2id(model_.get(), mjOBJ_BODY, "rig_pitch");
    ASSERT_GE(tip_, 0);
    ASSERT_GE(pitch_body, 0);

    // With every joint at 0 the tool points along +X. Where its tip is then,
    // relative to the point the hinges turn about, gives the length used to
    // aim the tool.
    mj_forward(model_.get(), data_.get());
    pivot_ = BodyPosition(pitch_body);
    tip_reach_ = (SitePosition(tip_) - pivot_).norm();

    // Let the modules settle against the backrest and each other.
    rig_target_ = VectorXd::Zero(5);
    StepFor(1.0);
  }

  Vector3d BodyPosition(int body) const {
    return Vector3d(data_->xpos + 3 * body);
  }
  Quaterniond BodyRotation(int body) const {
    const mjtNum* q = data_->xquat + 4 * body;
    return Quaterniond(q[0], q[1], q[2], q[3]);
  }
  Vector3d SitePosition(int site) const {
    return Vector3d(data_->site_xpos + 3 * site);
  }
  Vector3d ModuleVelocity(int body) const {
    const int joint = model_->body_jntadr[body];
    return Vector3d(data_->qvel + model_->jnt_dofadr[joint]);
  }

  // Carriage joint angles that put tool_tip at `tip` with the tool's +Z
  // along `direction`, then turned further by the given yaw and pitch.
  VectorXd Aim(const Vector3d& tip, const Vector3d& direction,
               double extra_yaw = 0.0, double extra_pitch = 0.0) const {
    const Vector3d d = direction.normalized();
    VectorXd q(5);
    q[kYaw] = std::atan2(d.y(), d.x()) + extra_yaw;
    q[kPitch] = std::asin(-d.z()) + extra_pitch;
    const Vector3d aimed(std::cos(q[kPitch]) * std::cos(q[kYaw]),
                         std::cos(q[kPitch]) * std::sin(q[kYaw]),
                         -std::sin(q[kPitch]));
    q.head<3>() = tip - pivot_ - tip_reach_ * aimed;
    return q;
  }

  // A point on a module's glass, `gap` in front of it, at (along, across)
  // from the centre along the module's long and short sides; and the
  // direction into the glass.
  Vector3d FacePoint(int body, double along, double across, double gap) const {
    const Matrix3d r = BodyRotation(body).toRotationMatrix();
    return BodyPosition(body) + along * r.col(0) + across * r.col(1) +
           (kHalfThickness + gap) * r.col(2);
  }
  Vector3d IntoGlass(int body) const {
    return -(BodyRotation(body) * Vector3d::UnitZ());
  }

  // Puts the carriage at `q` at once, at rest, with the servos holding it
  // there. For checks that need no motion.
  void PlaceRig(const VectorXd& q) {
    for (int i = 0; i < 5; ++i) {
      data_->qpos[qpos_address_[i]] = q[i];
      data_->ctrl[actuator_[i]] = q[i];
    }
    for (int i = 0; i < model_->nv; ++i) data_->qvel[i] = 0.0;
    rig_target_ = q;
    mj_forward(model_.get(), data_.get());
  }

  // One step of physics, with the servos on rig_target_, and a check that
  // the state stayed finite.
  void Step() {
    for (int i = 0; i < 5; ++i) data_->ctrl[actuator_[i]] = rig_target_[i];
    mj_step(model_.get(), data_.get());
    for (int i = 0; i < model_->nq; ++i) {
      ASSERT_TRUE(std::isfinite(data_->qpos[i])) << "qpos[" << i << "]";
    }
    for (int i = 0; i < model_->nv; ++i) {
      ASSERT_TRUE(std::isfinite(data_->qvel[i])) << "qvel[" << i << "]";
    }
    Observe();
  }

  void StepFor(double seconds) {
    const double end = data_->time + seconds;
    while (data_->time < end) {
      Step();
      if (HasFatalFailure()) return;
    }
  }

  // Drives the carriage from where it is told to be to `goal` along an
  // S-curve joint move, then lets it settle for `settle` seconds.
  void Move(const VectorXd& goal, const JointLimits& limits,
            double settle = 0.3) {
    std::optional<JointMove> move =
        JointMove::Plan(rig_target_, goal, limits, cell::ProfileShape::kSCurve);
    ASSERT_TRUE(move.has_value());
    const double start = data_->time;
    while (data_->time - start < move->duration()) {
      rig_target_ = move->At(data_->time - start).position;
      Step();
      if (HasFatalFailure()) return;
    }
    rig_target_ = goal;
    StepFor(settle);
  }

  // Called after every step: while a module is held, how far it is from its
  // pose at the grip, and how hard it is being accelerated.
  void Observe() {
    const std::optional<int> held = suction_->holding();
    if (!held) {
      last_velocity_.reset();
      return;
    }
    mj_kinematics(model_.get(), data_.get());
    drift_.Add(*suction_->HeldPoseInTip(*data_),
               *suction_->HeldPoseInTipAtGrip());
    // Acceleration as the change of velocity over 10 steps, which smooths
    // out the solver's step-to-step jitter.
    if (++steps_since_velocity_ < 10 && last_velocity_) return;
    const Vector3d velocity = ModuleVelocity(*held);
    if (last_velocity_) {
      peak_acceleration_ =
          std::max(peak_acceleration_, (velocity - *last_velocity_).norm() /
                                           (10 * model_->opt.timestep));
    }
    last_velocity_ = velocity;
    steps_since_velocity_ = 0;
  }

  // Grips and checks that it was `body` that got gripped.
  void GripExpecting(int body) {
    GripFailure failure;
    const std::optional<int> gripped =
        suction_->Grip(model_.get(), data_.get(), &failure);
    ASSERT_TRUE(gripped.has_value()) << failure.message;
    ASSERT_EQ(*gripped, body);
    drift_ = Drift();
    peak_acceleration_ = 0.0;
  }

  // A free module's kinetic plus potential energy. Its frame is its
  // principal frame, so the inertia is diagonal in it, and MuJoCo gives a
  // free joint's angular velocity in that frame.
  double Energy(int body) const {
    // From qpos and qvel, not xpos: after mj_step, xpos is a step behind.
    const int joint = model_->body_jntadr[body];
    const int dof = model_->jnt_dofadr[joint];
    const double height = data_->qpos[model_->jnt_qposadr[joint] + 2];
    const Vector3d v(data_->qvel + dof);
    const Vector3d w(data_->qvel + dof + 3);
    const Vector3d inertia(model_->body_inertia + 3 * body);
    const double mass = model_->body_mass[body];
    return 0.5 * mass * v.squaredNorm() +
           0.5 * inertia.dot(w.cwiseProduct(w)) +
           mass * -model_->opt.gravity[2] * height;
  }

  // The lowest point of a module's underside that is over a rail: where it
  // will first touch when lowered. Sampled every centimetre.
  double LowestOverRails(int body) const {
    const Matrix3d r = BodyRotation(body).toRotationMatrix();
    const Vector3d centre = BodyPosition(body);
    double lowest = INFINITY;
    for (double a = -kHalfLength; a <= kHalfLength; a += 0.01) {
      for (double b = -kHalfWidth; b <= kHalfWidth; b += 0.01) {
        for (double c : {-kHalfThickness, kHalfThickness}) {
          const Vector3d p = centre + r * Vector3d(a, b, c);
          const bool over_rail =
              std::abs(p.x() - kRailNearX) <= kRailHalfWidth ||
              std::abs(p.x() - kRailFarX) <= kRailHalfWidth;
          if (over_rail) lowest = std::min(lowest, p.z());
        }
      }
    }
    return lowest;
  }

  // Carries the held module from where it was gripped, standing at the
  // stack, to the rest, laid flat with its centre at `rest_y`, and lets go.
  // On the way: a lift, a pull back, a turn to flat, a quarter turn about
  // the vertical and back, and a shake with brisk moves.
  void CarryToRestAndRelease(double rest_y) {
    // Lift off the floor, pull straight back along the glass's normal, rise.
    VectorXd q = rig_target_;
    q[kZ] += 0.03;
    Move(q, kGentle);
    q[kX] -= 0.5;
    Move(q, kGentle);
    q[kZ] = 0.35;
    Move(q, kGentle);

    // Turn flat, glass up. The tool keeps whatever yaw it gripped with.
    q[kPitch] = kPi / 2.0;
    Move(q, kGentle);

    // Well clear of the stack and the rails, swing a quarter turn about the
    // vertical and back, then shake.
    q[kX] = -0.3;
    Move(q, kGentle);
    VectorXd turned = q;
    turned[kYaw] += kPi / 2.0;
    Move(turned, kBrisk, 0.0);
    Move(q, kBrisk, 0.0);
    for (int i = 0; i < 2; ++i) {
      VectorXd shaken = q;
      shaken[kX] += 0.3;
      shaken[kY] -= 0.3;
      shaken[kZ] += 0.15;
      Move(shaken, kBrisk, 0.0);
      Move(q, kBrisk, 0.0);
    }
    if (HasFatalFailure()) return;

    // Over the rest, square to the rails, a little above them.
    q[kX] = -1.5;
    q[kY] = rest_y;
    q[kYaw] = 0.0;
    Move(q, kGentle);
    const int held = *suction_->holding();
    q[kZ] -= LowestOverRails(held) - (kRailTop + 0.03);
    Move(q, kGentle);
    // Slowly down until the first point over a rail is 1 mm above it. A
    // module held tilted, as the misaligned one is, then rocks flat onto
    // the rails once let go: an arm would square it first, from what its
    // camera saw at the pick.
    q[kZ] -= LowestOverRails(held) - (kRailTop + 0.001);
    Move(q, Limits(0.05, 0.5, 5.0), 0.5);
    if (HasFatalFailure()) return;

    // Let go, and watch the module for a moment while the tool stays put.
    std::printf("carry of body %d: worst drift %.6f mm, %.6f deg; "
                "peak acceleration %.2f m/s^2\n",
                held, drift_.position * 1e3, drift_.angle / kDegree,
                peak_acceleration_);
    EXPECT_LT(drift_.position, kHeldPositionTolerance);
    EXPECT_LT(drift_.angle, kHeldAngleTolerance);
    EXPECT_GT(peak_acceleration_, 3.0);

    const Vector3d before = BodyPosition(held);
    suction_->Release(data_.get());
    EXPECT_FALSE(suction_->holding().has_value());
    EXPECT_EQ(data_->eq_active[weld_], 0);
    // Once let go, nothing but gravity and the rails acts on the module: it
    // may drop or rock onto the rails, but its energy must not rise. A
    // weld that let go with a jerk would show as a gain.
    const double energy_at_release = Energy(held);
    double gain = 0.0;
    const double start = data_->time;
    while (data_->time - start < 0.5) {
      Step();
      if (HasFatalFailure()) return;
      gain = std::max(gain, Energy(held) - energy_at_release);
    }
    std::printf("release of body %d: largest energy gain %.5f J\n", held,
                gain);
    EXPECT_LT(gain, 0.02) << "the module was thrown when let go";

    // Take the tool away and let the module settle.
    q[kZ] += 0.2;
    Move(q, kGentle, 2.0);
    if (HasFatalFailure()) return;

    const Vector3d after = BodyPosition(held);
    const Vector3d up = BodyRotation(held) * Vector3d::UnitZ();
    std::printf("release of body %d: moved %.3f mm sideways, %.3f mm down\n",
                held, (after - before).head<2>().norm() * 1e3,
                (before.z() - after.z()) * 1e3);
    EXPECT_LT((after - before).head<2>().norm(), 0.002);
    EXPECT_NEAR(LowestOverRails(held), kRailTop, 0.002)
        << "not resting on the rails";
    EXPECT_GT(up.z(), std::cos(1.0 * kDegree)) << "not flat, glass up";
    EXPECT_LT(ModuleVelocity(held).norm(), 0.002) << "still moving";
    EXPECT_GT(SitePosition(tip_).z() - after.z(), 0.15)
        << "the module came up with the tool";
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  std::optional<Suction> suction_;

  std::array<int, 5> qpos_address_{};
  std::array<int, 5> actuator_{};
  std::array<int, 3> module_{};
  int tip_ = -1;
  int weld_ = -1;
  Vector3d pivot_;
  double tip_reach_ = 0.0;

  VectorXd rig_target_;
  Drift drift_;
  double peak_acceleration_ = 0.0;
  std::optional<Vector3d> last_velocity_;
  int steps_since_velocity_ = 0;
};

TEST_F(SuctionTest, FindsToolWeldAndModules) {
  EXPECT_EQ(suction_->modules().size(), 3u);
  EXPECT_FALSE(suction_->holding().has_value());
  EXPECT_FALSE(suction_->HeldPoseInTip(*data_).has_value());

  cell::SuctionOptions options;
  options.weld = "no_such_weld";
  std::string error;
  EXPECT_FALSE(Suction::Create(*model_, options, &error).has_value());
  EXPECT_NE(error.find("no_such_weld"), std::string::npos) << error;
}

TEST_F(SuctionTest, GripFailsAwayFromTheGlass) {
  // 10 cm in front of module_0's glass, square to it.
  PlaceRig(Aim(FacePoint(module_[0], 0.0, 0.0, 0.10), IntoGlass(module_[0])));
  GripFailure failure;
  EXPECT_FALSE(suction_->Grip(model_.get(), data_.get(), &failure));
  EXPECT_EQ(failure.kind, GripFailure::Kind::kTooFar) << failure.message;
  EXPECT_NE(failure.message.find("module_0"), std::string::npos)
      << failure.message;
  EXPECT_FALSE(suction_->holding().has_value());
  EXPECT_EQ(data_->eq_active[weld_], 0);

  // Just beyond the limit.
  PlaceRig(Aim(FacePoint(module_[0], 0.0, 0.0, 0.011), IntoGlass(module_[0])));
  EXPECT_FALSE(suction_->Grip(model_.get(), data_.get(), &failure));
  EXPECT_EQ(failure.kind, GripFailure::Kind::kTooFar) << failure.message;

  // With the carriage parked, nowhere near anything.
  PlaceRig(VectorXd::Zero(5));
  EXPECT_FALSE(suction_->Grip(model_.get(), data_.get(), &failure));
  EXPECT_EQ(failure.kind, GripFailure::Kind::kTooFar) << failure.message;
}

TEST_F(SuctionTest, GripFailsWhenTiltedBeyondTolerance) {
  const Vector3d tip = FacePoint(module_[0], 0.0, 0.0, 0.002);
  GripFailure failure;

  // 5 degrees off square, about either hinge.
  PlaceRig(Aim(tip, IntoGlass(module_[0]), 5.0 * kDegree, 0.0));
  EXPECT_FALSE(suction_->Grip(model_.get(), data_.get(), &failure));
  EXPECT_EQ(failure.kind, GripFailure::Kind::kTooTilted) << failure.message;

  PlaceRig(Aim(tip, IntoGlass(module_[0]), 0.0, -5.0 * kDegree));
  EXPECT_FALSE(suction_->Grip(model_.get(), data_.get(), &failure));
  EXPECT_EQ(failure.kind, GripFailure::Kind::kTooTilted) << failure.message;
  EXPECT_FALSE(suction_->holding().has_value());

  // 2 degrees off is within tolerance.
  PlaceRig(Aim(tip, IntoGlass(module_[0]), 2.0 * kDegree, 0.0));
  EXPECT_EQ(suction_->Grip(model_.get(), data_.get(), &failure), module_[0])
      << failure.message;
}

TEST_F(SuctionTest, GripFailsOffTheEdgeOfTheGlass) {
  GripFailure failure;

  // In the plane of the glass, 5 cm beyond the end of the long side, and
  // 5 cm beyond the top edge.
  const std::pair<double, double> beyond[] = {{kHalfLength + 0.05, 0.0},
                                              {0.0, kHalfWidth + 0.05}};
  for (const auto& [along, across] : beyond) {
    PlaceRig(Aim(FacePoint(module_[0], along, across, 0.0),
                 IntoGlass(module_[0])));
    EXPECT_FALSE(suction_->Grip(model_.get(), data_.get(), &failure));
    EXPECT_EQ(failure.kind, GripFailure::Kind::kOffFace) << failure.message;
    EXPECT_NE(failure.message.find("module_0"), std::string::npos)
        << failure.message;
  }

  // 2 cm inside the corner is still on the glass.
  PlaceRig(Aim(
      FacePoint(module_[0], kHalfLength - 0.02, kHalfWidth - 0.02, 0.0),
      IntoGlass(module_[0])));
  EXPECT_EQ(suction_->Grip(model_.get(), data_.get(), &failure), module_[0])
      << failure.message;
}

TEST_F(SuctionTest, GripPicksTheModuleTheToolIsOn) {
  GripFailure failure;

  // module_0 stands right in front of module_1: the front one is taken.
  PlaceRig(Aim(FacePoint(module_[0], 0.0, 0.0, 0.003), IntoGlass(module_[0])));
  EXPECT_EQ(suction_->Grip(model_.get(), data_.get(), &failure), module_[0])
      << failure.message;
  EXPECT_EQ(suction_->holding(), module_[0]);
  EXPECT_EQ(model_->eq_obj2id[weld_], module_[0]);
  EXPECT_EQ(data_->eq_active[weld_], 1);

  // A second grip while holding is refused and changes nothing.
  EXPECT_FALSE(suction_->Grip(model_.get(), data_.get(), &failure));
  EXPECT_EQ(failure.kind, GripFailure::Kind::kAlreadyHolding);
  EXPECT_EQ(suction_->holding(), module_[0]);

  suction_->Release(data_.get());
  EXPECT_FALSE(suction_->holding().has_value());
  EXPECT_EQ(data_->eq_active[weld_], 0);

  // module_2, off to the side.
  PlaceRig(Aim(FacePoint(module_[2], -0.3, 0.1, 0.0), IntoGlass(module_[2])));
  EXPECT_EQ(suction_->Grip(model_.get(), data_.get(), &failure), module_[2])
      << failure.message;
  EXPECT_EQ(model_->eq_obj2id[weld_], module_[2]);
}

// The arm arrives a few millimetres and about a degree off. The grip must
// keep that offset, not square it away, all the way to the rest. Then a
// second module goes through the same cycle.
TEST_F(SuctionTest, CarryKeepsTheGripPoseAndReleaseIsClean) {
  // 4 mm along the long side, -3 mm along the short side, and turned by
  // 0.5 degrees about the vertical and 0.85 degrees about the horizontal:
  // about 1 degree off square in all. The tip stops 8.5 mm from the
  // glass, so that the nearest cup, tipped closer by the tilt, stays clear.
  const double gap = 0.0085;
  const double yaw = 0.5 * kDegree;
  const double pitch = -0.85 * kDegree;
  const Vector3d tip = FacePoint(module_[0], 0.004, -0.003, gap);
  const Vector3d into = IntoGlass(module_[0]);

  // Come in along the glass's normal from 6 cm out.
  Move(Aim(tip - 0.06 * into, into, yaw, pitch), kGentle);
  Move(Aim(tip, into, yaw, pitch), kGentle, 0.5);
  ASSERT_FALSE(HasFatalFailure());
  ASSERT_LT(ModuleVelocity(module_[0]).norm(), 1e-3) << "module not at rest";
  GripExpecting(module_[0]);
  ASSERT_FALSE(HasFatalFailure());

  // The pose held is the pose found: the offset and the tilt are kept.
  const Pose at_grip = *suction_->HeldPoseInTipAtGrip();
  const Vector3d module_z = at_grip.rotation * Vector3d::UnitZ();
  const double tilt = std::acos(std::clamp(-module_z.z(), -1.0, 1.0));
  std::printf("misaligned grip: module centre at (%.2f, %.2f, %.2f) mm "
              "in the tip frame, tilted %.3f deg\n",
              at_grip.position.x() * 1e3, at_grip.position.y() * 1e3,
              at_grip.position.z() * 1e3, tilt / kDegree);
  EXPECT_NEAR(tilt, std::hypot(yaw, pitch), 0.1 * kDegree);
  EXPECT_GT(at_grip.position.head<2>().norm(), 0.003);

  CarryToRestAndRelease(-1.25);
  ASSERT_FALSE(HasFatalFailure());
  const Vector3d first_resting = BodyPosition(module_[0]);

  // module_1, now at the front of the stack, square this time.
  const Vector3d tip_1 = FacePoint(module_[1], 0.0, 0.0, 0.003);
  const Vector3d into_1 = IntoGlass(module_[1]);
  Move(Aim(tip_1 - 0.06 * into_1, into_1), kGentle);
  Move(Aim(tip_1, into_1), kGentle, 0.5);
  ASSERT_FALSE(HasFatalFailure());
  GripExpecting(module_[1]);
  ASSERT_FALSE(HasFatalFailure());
  CarryToRestAndRelease(1.25);
  ASSERT_FALSE(HasFatalFailure());

  // The first module was not disturbed by the second.
  EXPECT_LT((BodyPosition(module_[0]) - first_resting).norm(), 0.001);
}

// A module held standing up has its whole weight pulling along the cups;
// held flat, straight off them. Either way it must stay where the grip put
// it: a small fixed sag from the weld's softness, and no creep with time.
TEST_F(SuctionTest, LongHoldDoesNotCreep) {
  const Vector3d tip = FacePoint(module_[0], 0.0, 0.0, 0.003);
  const Vector3d into = IntoGlass(module_[0]);
  Move(Aim(tip - 0.06 * into, into), kGentle);
  Move(Aim(tip, into), kGentle, 0.5);
  ASSERT_FALSE(HasFatalFailure());
  GripExpecting(module_[0]);
  ASSERT_FALSE(HasFatalFailure());
  const Pose at_grip = *suction_->HeldPoseInTipAtGrip();

  VectorXd q = rig_target_;
  q[kZ] += 0.03;
  Move(q, kGentle);
  q[kX] -= 0.5;
  Move(q, kGentle, 1.0);
  ASSERT_FALSE(HasFatalFailure());

  // Standing up. 1 s in, the sag is all there is; 60 s later, no more.
  auto difference = [&](const Pose& a, const Pose& b) {
    return std::pair{(a.position - b.position).norm(),
                     AngleBetween(a.rotation, b.rotation)};
  };
  mj_kinematics(model_.get(), data_.get());
  const Pose standing_start = *suction_->HeldPoseInTip(*data_);
  StepFor(60.0);
  ASSERT_FALSE(HasFatalFailure());
  mj_kinematics(model_.get(), data_.get());
  const Pose standing_end = *suction_->HeldPoseInTip(*data_);
  const auto [standing_sag, standing_sag_angle] =
      difference(standing_start, at_grip);
  const auto [standing_creep, standing_creep_angle] =
      difference(standing_end, standing_start);

  // Flat, hanging below the tool.
  q[kZ] = 0.35;
  Move(q, kGentle);
  q[kPitch] = kPi / 2.0;
  Move(q, kGentle, 1.0);
  ASSERT_FALSE(HasFatalFailure());
  mj_kinematics(model_.get(), data_.get());
  const Pose flat_start = *suction_->HeldPoseInTip(*data_);
  StepFor(30.0);
  ASSERT_FALSE(HasFatalFailure());
  mj_kinematics(model_.get(), data_.get());
  const Pose flat_end = *suction_->HeldPoseInTip(*data_);
  const auto [flat_sag, flat_sag_angle] = difference(flat_start, at_grip);
  const auto [flat_creep, flat_creep_angle] = difference(flat_end, flat_start);

  std::printf("standing: sag %.6f mm %.6f deg, creep over 60 s %.6f mm "
              "%.6f deg\n",
              standing_sag * 1e3, standing_sag_angle / kDegree,
              standing_creep * 1e3, standing_creep_angle / kDegree);
  std::printf("flat: sag %.6f mm %.6f deg, creep over 30 s %.6f mm %.6f deg\n",
              flat_sag * 1e3, flat_sag_angle / kDegree, flat_creep * 1e3,
              flat_creep_angle / kDegree);

  EXPECT_LT(standing_sag, kHeldPositionTolerance);
  EXPECT_LT(standing_sag_angle, kHeldAngleTolerance);
  EXPECT_LT(flat_sag, kHeldPositionTolerance);
  EXPECT_LT(flat_sag_angle, kHeldAngleTolerance);
  // Creep: a micrometre over a minute would already be a trend.
  EXPECT_LT(standing_creep, 1e-6);
  EXPECT_LT(flat_creep, 1e-6);
  EXPECT_LT(standing_creep_angle, 1e-4 * kDegree);
  EXPECT_LT(flat_creep_angle, 1e-4 * kDegree);
}

}  // namespace
