// Checks the arm following planned moves in the arm's own scene: it ends on
// the goal, keeps close to the plan on the way, stays within the vendor's
// speed, acceleration and torque limits with and without a module at the
// tool, joins moves without a jump and refuses what it cannot do. Also
// checks FindArm and TipPose from src/arm.h.
//
//   bazel test //:arm_follower_test
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/arm_scene.xml). Every test starts
// from the keyframe arm_home.
//
// Measured numbers are printed as they are found, so a run shows them:
//   bazel test //:arm_follower_test --test_output=all

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/arm_follower.h"
#include "src/joint_move.h"
#include "src/mj_handles.h"

namespace {

using cell::Arm;
using cell::ArmFollower;
using cell::JointLimits;
using cell::JointMove;
using cell::MoveOptions;
using cell::ProfileShape;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr int kJoints = Arm::kJoints;
constexpr double kHalfPi = 1.5707963267948966;
constexpr double kPi = 3.141592653589793;

// A module (as in src/pallet.h): 2.3 m x 1.1 m x 35 mm, 30 kg.
constexpr double kModuleLength = 2.3;
constexpr double kModuleWidth = 1.1;
constexpr double kModuleThickness = 0.035;
constexpr double kModuleMass = 30.0;

// Where the cups meet the glass, along the tool's +Z (models/tool.xml).
constexpr double kTipOut = 0.30;

// Measured speeds and accelerations may pass the limits by this fraction:
// the servo overshoots the plan by a hair where the S-curve's jerk changes.
constexpr double kLimitAllowance = 0.02;

// The largest tracking error allowed while moving, rad. The worst seen at
// full speed is under 0.15 mrad, on joint 4.
constexpr double kTrackingTolerance = 3e-4;

// The largest torque allowed at the default speed, as a fraction of the
// limit. The worst seen is 0.91, on joint 2 (see FullSpeedCanSaturate...).
constexpr double kTorqueCeiling = 0.95;

const VectorXd kHome{{0, -kHalfPi, kHalfPi, 0, kHalfPi, 0}};

// About a radian on every joint: forward and down, wrist rolled and bent.
const VectorXd kBigMove = kHome + VectorXd{{1.0, 0.6, -1.0, 1.0, -1.0, 1.0}};

// Home turned half way round on joint 1.
const VectorXd kTurned{{kPi, -kHalfPi, kHalfPi, 0, kHalfPi, 0}};

// Near full reach with the tool down, on either side.
const VectorXd kReachLeft{{-1.5, -0.3, 0.3, 0, kHalfPi, 0}};
const VectorXd kReachRight{{1.5, -0.3, 0.3, 0, kHalfPi, 0}};

// The hardest move found for joint 2: from full reach with the tool
// pointing straight out (as when facing the pallet's stack of upright
// modules), lifting the upper arm upright while turning 3 rad on joint 1.
// Gravity alone takes about 70% of joint 2's torque at the start; the lift,
// and the arm being flung outward as joint 1 turns, add the rest.
const VectorXd kOutAtFullReach{{3.0, -0.0873, 0.15, 0, 0, 0}};
const VectorXd kUpright{{0, -kHalfPi, 0.15, 0, 0, 0}};

// Loads the scene. With `module` true, a module is fixed to the tool as if
// held: a body with no joint, a child of the tool body, so MuJoCo welds it
// rigidly to the tool and the arm's inertia and gravity include it, the
// same as raising the tool body's own mass and inertia. Adding a body lets
// MuJoCo combine the inertias, so the test does not have to do it by hand.
// The module is a uniform plate with its glass on the cups, centred on the
// tool tip, long side along the tool's X.
cell::ModelPtr LoadScene(bool module, std::string* error) {
  const char* path = std::getenv("MODEL_PATH");
  if (path == nullptr) {
    *error = "MODEL_PATH is not set";
    return nullptr;
  }
  char message[1000] = "";
  mjSpec* spec = mj_parseXML(path, nullptr, message, sizeof(message));
  if (spec == nullptr) {
    *error = message;
    return nullptr;
  }
  if (module) {
    mjsBody* tool = mjs_findBody(spec, "tool");
    if (tool == nullptr) {
      *error = "no body tool";
      mj_deleteSpec(spec);
      return nullptr;
    }
    mjsBody* plate = mjs_addBody(tool, nullptr);
    mjs_setName(plate->element, "held_module");
    const double m = kModuleMass, a = kModuleLength, b = kModuleWidth,
                 t = kModuleThickness;
    plate->mass = m;
    plate->ipos[0] = 0.0;
    plate->ipos[1] = 0.0;
    plate->ipos[2] = kTipOut + t / 2;
    plate->inertia[0] = m / 12 * (b * b + t * t);  // 3.0 kg m^2
    plate->inertia[1] = m / 12 * (a * a + t * t);  // 13.2
    plate->inertia[2] = m / 12 * (a * a + b * b);  // 16.3
    plate->explicitinertial = 1;
  }
  mjModel* model = mj_compile(spec, nullptr);
  if (model == nullptr) *error = mjs_getError(spec);
  mj_deleteSpec(spec);
  return cell::ModelPtr(model);
}

// The arm's limits played slower by `speed`, as MoveOptions::speed does.
JointLimits Scaled(JointLimits limits, double speed) {
  limits.velocity *= speed;
  limits.acceleration *= speed * speed;
  limits.jerk *= speed * speed * speed;
  return limits;
}

std::string Format(const VectorXd& v, double scale = 1.0) {
  std::string out;
  char buffer[32];
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    std::snprintf(buffer, sizeof(buffer), " %8.4f", v[i] * scale);
    out += buffer;
  }
  return out;
}

// Distance from point p to the segment from a to b.
double DistanceToSegment(const Vector3d& p, const Vector3d& a,
                         const Vector3d& b) {
  const Vector3d ab = b - a;
  const double s = std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
  return (p - (a + s * ab)).norm();
}

// What a stretch of simulation saw. Peaks are of magnitudes; speed,
// acceleration and torque are fractions of each joint's limit.
struct Watch {
  VectorXd tracking = VectorXd::Zero(kJoints);  // rad, while a plan runs
  VectorXd speed = VectorXd::Zero(kJoints);
  VectorXd acceleration = VectorXd::Zero(kJoints);
  VectorXd torque = VectorXd::Zero(kJoints);
  VectorXd command_step = VectorXd::Zero(kJoints);  // largest tick-to-tick
  double range_margin = std::numeric_limits<double>::infinity();
  double done_at = -1.0;           // first time Done(), or -1
  bool done_while_moving = false;  // Done() before the plan ended
  bool touched = false;            // any contact at all
};

class ArmFollowerTest : public ::testing::Test {
 protected:
  void SetUp() override { Load(false); }

  // (Re)loads the scene, with or without a held module, and resets it to
  // arm_home.
  void Load(bool module) {
    std::string error;
    model_ = LoadScene(module, &error);
    ASSERT_TRUE(model_) << error;
    data_ = cell::MakeData(*model_);
    ASSERT_TRUE(data_);
    std::optional<Arm> arm = cell::FindArm(*model_);
    ASSERT_TRUE(arm.has_value());
    arm_ = *arm;
    Reset();
  }

  void Reset() {
    const int home = mj_name2id(model_.get(), mjOBJ_KEY, "arm_home");
    ASSERT_GE(home, 0);
    mj_resetDataKeyframe(model_.get(), data_.get(), home);
    mj_forward(model_.get(), data_.get());
  }

  VectorXd Angles() const { return cell::JointAngles(arm_, *data_); }
  VectorXd Speeds() const { return cell::JointSpeeds(arm_, *data_); }

  // One control tick: the follower writes the commands, then one step.
  // Adds what it saw to `watch`.
  void Tick(ArmFollower& follower, Watch& watch) {
    const VectorXd before = Speeds();
    const bool had_command = follower.Commanded().size() == kJoints;
    const VectorXd last_command =
        had_command ? follower.Commanded() : VectorXd::Zero(kJoints);

    follower.Update(*model_, data_.get());
    if (had_command) {
      watch.command_step = watch.command_step.cwiseMax(
          (follower.Commanded() - last_command).cwiseAbs());
    }
    mj_step(model_.get(), data_.get());

    const VectorXd q = Angles();
    const VectorXd v = Speeds();
    // The acceleration over the step, from the change of speed. (MuJoCo's
    // qacc is the explicit acceleration, before the implicit integrator
    // accounts for the stiff servo's damping, and overstates it.)
    const VectorXd a = (v - before) / model_->opt.timestep;
    watch.speed = watch.speed.cwiseMax(
        v.cwiseAbs().cwiseQuotient(arm_.limits.velocity));
    watch.acceleration = watch.acceleration.cwiseMax(
        a.cwiseAbs().cwiseQuotient(arm_.limits.acceleration));
    for (int i = 0; i < kJoints; ++i) {
      watch.torque[i] =
          std::max(watch.torque[i],
                   std::abs(data_->actuator_force[arm_.motor[i]]) /
                       arm_.torque_limit[i]);
      watch.range_margin = std::min(
          {watch.range_margin, q[i] - arm_.lower[i], arm_.upper[i] - q[i]});
    }
    if (follower.TimeLeft(*data_) > 0.0) {
      watch.tracking = watch.tracking.cwiseMax(
          follower.TrackingError(*data_).cwiseAbs());
    }
    const bool done = follower.Done(*data_);
    if (done && follower.TimeLeft(*data_) > 0.0) watch.done_while_moving = true;
    if (done && watch.done_at < 0.0) watch.done_at = data_->time;
    if (data_->ncon > 0) watch.touched = true;
  }

  // Ticks until the plan has ended and then `extra` seconds more.
  Watch RunPast(ArmFollower& follower, double extra) {
    Watch watch;
    const double end = data_->time + follower.TimeLeft(*data_) + extra;
    while (data_->time < end) Tick(follower, watch);
    return watch;
  }

  // Goes from home to `pose` at a gentle speed and lets it settle, so a
  // measured move can start there.
  void GoTo(ArmFollower& follower, const VectorXd& pose) {
    ASSERT_TRUE(follower.MoveTo(pose, *data_, {.speed = 0.5}));
    RunPast(follower, 0.5);
    ASSERT_TRUE(follower.Done(*data_));
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  Arm arm_;
};

// ------------------------------------------------------------ src/arm.h

TEST_F(ArmFollowerTest, FindArmNamesTheArmsEntries) {
  for (int i = 0; i < kJoints; ++i) {
    const std::string n = std::to_string(i + 1);
    const int joint =
        mj_name2id(model_.get(), mjOBJ_JOINT, ("arm_joint_" + n).c_str());
    const int motor =
        mj_name2id(model_.get(), mjOBJ_ACTUATOR, ("arm_motor_" + n).c_str());
    ASSERT_GE(joint, 0);
    ASSERT_GE(motor, 0);
    EXPECT_EQ(arm_.qpos[i], model_->jnt_qposadr[joint]);
    EXPECT_EQ(arm_.dof[i], model_->jnt_dofadr[joint]);
    EXPECT_EQ(arm_.motor[i], motor);
    EXPECT_EQ(arm_.lower[i], model_->jnt_range[2 * joint]);
    EXPECT_EQ(arm_.upper[i], model_->jnt_range[2 * joint + 1]);
    EXPECT_EQ(arm_.torque_limit[i],
              model_->actuator_forcerange[2 * motor + 1]);

    // lag = (servo damping kv + joint damping) / kp. In MuJoCo's arrays a
    // position servo stores kp as gainprm[0] and -kv as biasprm[2].
    const double kp = model_->actuator_gainprm[mjNGAIN * motor];
    const double kv = -model_->actuator_biasprm[mjNBIAS * motor + 2];
    EXPECT_GT(kv, 0.0);
    EXPECT_NEAR(arm_.lag[i], (kv + model_->dof_damping[arm_.dof[i]]) / kp,
                1e-15);
  }
  EXPECT_EQ(arm_.tool_body, mj_name2id(model_.get(), mjOBJ_BODY, "tool"));
  EXPECT_EQ(arm_.tip_site, mj_name2id(model_.get(), mjOBJ_SITE, "tool_tip"));

  // models/arm.xml: 0.035 s on joint 1, 0.018 to 0.025 s on the others.
  EXPECT_NEAR(arm_.lag[0], 0.035, 0.001);
  for (int i = 1; i < kJoints; ++i) {
    EXPECT_GT(arm_.lag[i], 0.017) << "joint " << i + 1;
    EXPECT_LT(arm_.lag[i], 0.026) << "joint " << i + 1;
  }
  std::printf("lag, s:%s\n", Format(arm_.lag).c_str());
}

TEST_F(ArmFollowerTest, FindArmRefusesAModelWithoutTheArm) {
  char error[1000] = "";
  mjSpec* spec = mj_parseXMLString(
      "<mujoco><worldbody><body><joint name='arm_joint_1'/>"
      "<geom size='0.1'/></body></worldbody>"
      "<actuator><position name='arm_motor_1' joint='arm_joint_1'/>"
      "</actuator></mujoco>",
      nullptr, error, sizeof(error));
  ASSERT_NE(spec, nullptr) << error;
  cell::ModelPtr model(mj_compile(spec, nullptr));
  mj_deleteSpec(spec);
  ASSERT_TRUE(model);
  EXPECT_FALSE(cell::FindArm(*model).has_value());
}

TEST_F(ArmFollowerTest, TipPoseIsTheToolTipSite) {
  const Isometry3d tip = cell::TipPose(arm_, *data_);
  for (int r = 0; r < 3; ++r) {
    EXPECT_EQ(tip.translation()[r], data_->site_xpos[3 * arm_.tip_site + r]);
    for (int c = 0; c < 3; ++c) {
      EXPECT_EQ(tip.linear()(r, c),
                data_->site_xmat[9 * arm_.tip_site + 3 * r + c]);
    }
  }
  // At home the flange faces the floor at (1.550, 0, 1.695) (arm_test), so
  // the tip is 0.30 m below it, pointing down.
  EXPECT_TRUE(tip.translation().isApprox(Vector3d(1.55, 0, 1.695 - kTipOut),
                                         1e-9));
  EXPECT_TRUE(
      (tip.linear() * Vector3d::UnitZ()).isApprox(-Vector3d::UnitZ(), 1e-9));
}

// ------------------------------------------------------- the follower

TEST_F(ArmFollowerTest, FirstUseAdoptsTheCommandedPose) {
  // Before any call, and after the first, the follower holds what ctrl
  // says: home, from the keyframe.
  ArmFollower follower(arm_);
  EXPECT_EQ(follower.Commanded().size(), 0);
  EXPECT_EQ(follower.Planned(*data_).position, kHome);
  EXPECT_EQ(follower.Goal(*data_), kHome);
  EXPECT_EQ(follower.TimeLeft(*data_), 0.0);

  // ctrl, not the measured angles: put the arm a little off what it is
  // told, and the follower still holds home and brings the arm back.
  data_->qpos[arm_.qpos[0]] += 0.002;
  mj_forward(model_.get(), data_.get());
  Watch watch;
  Tick(follower, watch);
  EXPECT_EQ(follower.Goal(*data_), kHome);
  // The command is home plus the model feedforward's few tenths of a mrad.
  EXPECT_LT((follower.Commanded() - kHome).cwiseAbs().maxCoeff(), 1e-3);
  for (int i = 0; i < 500; ++i) Tick(follower, watch);
  EXPECT_LT((Angles() - kHome).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_LT(watch.command_step.maxCoeff(), 1e-3);
  EXPECT_TRUE(follower.Done(*data_));
}

TEST_F(ArmFollowerTest, LargeMoveAtFullSpeedEndsOnGoalAndKeepsToThePlan) {
  ArmFollower follower(arm_);
  ASSERT_TRUE(follower.MoveTo(kBigMove, *data_, {.speed = 1.0}));
  const double duration = follower.TimeLeft(*data_);
  const double end = data_->time + duration;
  EXPECT_FALSE(follower.Done(*data_));

  const Watch watch = RunPast(follower, 1.0);
  EXPECT_FALSE(watch.touched);
  EXPECT_FALSE(watch.done_while_moving);
  EXPECT_GT(watch.range_margin, 0.0);
  for (int i = 0; i < kJoints; ++i) {
    EXPECT_LT(watch.tracking[i], kTrackingTolerance) << "joint " << i + 1;
  }
  ASSERT_GE(watch.done_at, 0.0);
  const double settle = watch.done_at - end;
  EXPECT_LT(settle, 0.05);
  const double end_error = (Angles() - kBigMove).cwiseAbs().maxCoeff();
  EXPECT_LT(end_error, 1e-6);
  EXPECT_LT(Speeds().cwiseAbs().maxCoeff(), 1e-6);

  std::printf(
      "big move at full speed: %.3f s, Done %.3f s after the plan ends, "
      "end error %.1e rad\n  tracking, mrad:%s\n",
      duration, settle, end_error, Format(watch.tracking, 1e3).c_str());
}

TEST_F(ArmFollowerTest, LagCompensationMatters) {
  // The same move with the lead term switched off. A joint then runs `lag`
  // seconds behind the plan, so at joint 1's peak planned speed it trails
  // by about lag * speed.
  const JointMove move =
      *JointMove::Plan(kHome, kBigMove, arm_.limits, ProfileShape::kSCurve);
  double peak_speed = 0.0;
  for (int k = 0; k <= 1000; ++k) {
    peak_speed = std::max(
        peak_speed, std::abs(move.At(move.duration() * k / 1000).velocity[0]));
  }

  Watch with, without;
  for (bool compensate : {true, false}) {
    Reset();
    ArmFollower follower(arm_, {.lag_compensation = compensate});
    ASSERT_TRUE(follower.MoveTo(kBigMove, *data_, {.speed = 1.0}));
    (compensate ? with : without) = RunPast(follower, 0.5);
  }
  std::printf("tracking with lag compensation, mrad:   %s\n",
              Format(with.tracking, 1e3).c_str());
  std::printf("tracking without lag compensation, mrad:%s\n",
              Format(without.tracking, 1e3).c_str());

  EXPECT_NEAR(without.tracking[0], arm_.lag[0] * peak_speed,
              0.2 * arm_.lag[0] * peak_speed);
  for (int i = 0; i < kJoints; ++i) {
    EXPECT_GT(without.tracking[i], 20 * with.tracking[i]) << "joint " << i + 1;
  }
}

TEST_F(ArmFollowerTest, ModelFeedforwardRemovesMostTrackingErrorAndTheSag) {
  // The same move without the model feedforward: the servo then makes the
  // torque for the acceleration and against gravity only from an angle
  // error. That error is still under a mrad, but the feedforward removes
  // nearly all of it, and the sag at rest with it.
  Watch with, without;
  double sag_with = 0.0, sag_without = 0.0;
  for (bool feedforward : {true, false}) {
    Reset();
    ArmFollower follower(arm_, {.model_feedforward = feedforward});
    ASSERT_TRUE(follower.MoveTo(kBigMove, *data_, {.speed = 1.0}));
    (feedforward ? with : without) = RunPast(follower, 1.0);
    (feedforward ? sag_with : sag_without) =
        (Angles() - kBigMove).cwiseAbs().maxCoeff();
  }
  std::printf("tracking with model feedforward, mrad:   %s; sag %.1e rad\n",
              Format(with.tracking, 1e3).c_str(), sag_with);
  std::printf("tracking without model feedforward, mrad:%s; sag %.1e rad\n",
              Format(without.tracking, 1e3).c_str(), sag_without);
  EXPECT_GT(without.tracking.maxCoeff(), 3 * with.tracking.maxCoeff());
  EXPECT_GT(sag_without, 1e-4);
  EXPECT_LT(sag_with, 1e-6);
}

// Large moves, each from a settled start, with and without a module held.
class LimitsTest : public ArmFollowerTest,
                   public ::testing::WithParamInterface<bool> {
 protected:
  void SetUp() override { Load(GetParam()); }
};

TEST_P(LimitsTest, SpeedsAccelerationsAndTorquesStayWithinLimits) {
  struct Case {
    const char* name;
    VectorXd from, to;
  };
  const std::vector<Case> cases = {
      {"home to big move", kHome, kBigMove},
      {"home to turned", kHome, kTurned},
      {"reach left to right", kReachLeft, kReachRight},
      {"reach right to big move", kReachRight, kBigMove},
      {"out at full reach to upright", kOutAtFullReach, kUpright},
      {"upright to out at full reach", kUpright, kOutAtFullReach},
  };
  // Full speed only without a module: with one, full speed can saturate
  // joint 2 (FullSpeedCanSaturate... below), which is why the default is
  // slower.
  std::vector<double> speeds = {MoveOptions::kDefaultSpeed};
  if (!GetParam()) speeds.push_back(1.0);
  for (double speed : speeds) {
    for (const Case& c : cases) {
      SCOPED_TRACE(c.name);
      Reset();
      ArmFollower follower(arm_);
      if (c.from != kHome) GoTo(follower, c.from);
      ASSERT_TRUE(follower.MoveTo(c.to, *data_, {.speed = speed}));
      const double duration = follower.TimeLeft(*data_);
      const Watch watch = RunPast(follower, 0.5);

      std::printf(
          "%s, speed %.2f, %s: %.2f s\n  torque/limit:%s\n  "
          "speed/limit: %s\n  accel/limit: %s\n  tracking, mrad:%s\n",
          GetParam() ? "with module" : "no module", speed, c.name, duration,
          Format(watch.torque).c_str(), Format(watch.speed).c_str(),
          Format(watch.acceleration).c_str(),
          Format(watch.tracking, 1e3).c_str());

      EXPECT_FALSE(watch.touched);
      EXPECT_GT(watch.range_margin, 0.0);
      EXPECT_TRUE(follower.Done(*data_));
      EXPECT_LT((Angles() - c.to).cwiseAbs().maxCoeff(), 1e-6);
      for (int i = 0; i < kJoints; ++i) {
        SCOPED_TRACE("joint " + std::to_string(i + 1));
        EXPECT_LE(watch.speed[i], speed * (1 + kLimitAllowance));
        EXPECT_LE(watch.acceleration[i],
                  speed * speed * (1 + kLimitAllowance));
        EXPECT_LT(watch.tracking[i], kTrackingTolerance);
        EXPECT_LT(watch.torque[i], kTorqueCeiling);
      }
    }
  }
}

INSTANTIATE_TEST_SUITE_P(Payload, LimitsTest, ::testing::Bool(),
                         [](const ::testing::TestParamInfo<bool>& info) {
                           return info.param ? "WithModule" : "NoModule";
                         });

TEST_F(ArmFollowerTest, FullSpeedCanSaturateJoint2WithAModuleAtFullReach) {
  // Why the default speed is below 1: this move with a module held drives
  // joint 2 to its torque limit at full speed, and stays clear of it at the
  // default speed.
  Load(true);
  double peak[2] = {0.0, 0.0};
  for (int k = 0; k < 2; ++k) {
    Reset();
    ArmFollower follower(arm_);
    GoTo(follower, kOutAtFullReach);
    const double speed = k == 0 ? 1.0 : MoveOptions::kDefaultSpeed;
    ASSERT_TRUE(follower.MoveTo(kUpright, *data_, {.speed = speed}));
    peak[k] = RunPast(follower, 0.2).torque[1];
  }
  std::printf(
      "joint 2 torque/limit, out at full reach to upright with a module: "
      "%.3f at full speed, %.3f at %.2f\n",
      peak[0], peak[1], MoveOptions::kDefaultSpeed);
  EXPECT_GT(peak[0], 0.99);
  EXPECT_LT(peak[1], kTorqueCeiling);
}

TEST_F(ArmFollowerTest, SuccessiveMovesJoinWithoutAJump) {
  // Each move is started the moment the last one's plan ends, before the
  // arm has settled. The command never jumps: from one tick to the next it
  // changes by no more than the fastest joint moves in a tick, plus the
  // lead term's change.
  ArmFollower follower(arm_);
  Watch watch;
  for (const VectorXd& goal : {kBigMove, kReachLeft, kTurned, kHome}) {
    std::string why;
    ASSERT_TRUE(follower.MoveTo(goal, *data_, {}, &why)) << why;
    while (follower.TimeLeft(*data_) > 0.0) Tick(follower, watch);
  }
  for (int i = 0; i < 250; ++i) Tick(follower, watch);
  EXPECT_TRUE(follower.Done(*data_));

  const double dt = model_->opt.timestep;
  for (int i = 0; i < kJoints; ++i) {
    const double bound = (arm_.limits.velocity[i] +
                          arm_.lag[i] * arm_.limits.acceleration[i]) *
                             dt * 1.1 +
                         1e-5;
    EXPECT_LT(watch.command_step[i], bound) << "joint " << i + 1;
  }
  std::printf("largest command change per tick, mrad:%s\n",
              Format(watch.command_step, 1e3).c_str());
}

TEST_F(ArmFollowerTest, MoveToWhileMovingIsRefusedAndTheMoveCarriesOn) {
  ArmFollower follower(arm_);
  ASSERT_TRUE(follower.MoveTo(kBigMove, *data_));
  const double end = data_->time + follower.TimeLeft(*data_);
  Watch watch;
  while (data_->time < end / 2) Tick(follower, watch);

  std::string why;
  EXPECT_FALSE(follower.MoveTo(kTurned, *data_, {}, &why));
  EXPECT_FALSE(why.empty());
  EXPECT_FALSE(follower.MoveThrough({kTurned}, *data_, {}, &why));
  EXPECT_EQ(follower.Goal(*data_), kBigMove);
  EXPECT_NEAR(data_->time + follower.TimeLeft(*data_), end, 1e-9);

  // The first move ends where it was going; then a new one is accepted.
  RunPast(follower, 0.2);
  EXPECT_TRUE(follower.Done(*data_));
  EXPECT_LT((Angles() - kBigMove).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_TRUE(follower.MoveTo(kTurned, *data_, {}, &why)) << why;
}

TEST_F(ArmFollowerTest, BadGoalsAreRefused) {
  ArmFollower follower(arm_);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();

  std::vector<VectorXd> bad = {VectorXd::Zero(5), VectorXd::Zero(7)};
  for (int i = 0; i < kJoints; ++i) {
    for (double value :
         {nan, inf, -inf, arm_.lower[i] - 1e-6, arm_.upper[i] + 1e-6}) {
      VectorXd goal = kHome;
      goal[i] = value;
      bad.push_back(goal);
    }
  }
  for (const VectorXd& goal : bad) {
    std::string why;
    EXPECT_FALSE(follower.MoveTo(goal, *data_, {}, &why)) << Format(goal);
    EXPECT_FALSE(why.empty());
  }
  for (double speed : {0.0, -0.5, 1.01, nan}) {
    std::string why;
    EXPECT_FALSE(follower.MoveTo(kBigMove, *data_, {.speed = speed}, &why))
        << speed;
    EXPECT_FALSE(why.empty());
  }
  std::string why;
  EXPECT_FALSE(follower.MoveThrough({}, *data_, {}, &why));
  // One bad waypoint refuses them all.
  EXPECT_FALSE(follower.MoveThrough({kBigMove, bad.back(), kTurned}, *data_,
                                    {}, &why));
  std::printf("for example: %s\n", why.c_str());

  // Nothing was planned; the arm stays at home.
  EXPECT_EQ(follower.TimeLeft(*data_), 0.0);
  EXPECT_EQ(follower.Goal(*data_), kHome);
  RunPast(follower, 0.5);
  EXPECT_LT((Angles() - kHome).cwiseAbs().maxCoeff(), 1e-6);

  // The edges of the ranges themselves are fine.
  VectorXd edge = kHome;
  edge[0] = arm_.upper[0];
  EXPECT_TRUE(follower.MoveTo(edge, *data_, {}, &why)) << why;
}

TEST_F(ArmFollowerTest, DoneOnlyOnceArrivedAndHoldingStaysPut) {
  ArmFollower follower(arm_);
  ASSERT_TRUE(follower.MoveTo(kReachLeft, *data_));
  const double end = data_->time + follower.TimeLeft(*data_);
  Watch watch = RunPast(follower, 0.0);
  EXPECT_FALSE(watch.done_while_moving);
  for (int i = 0; i < 100 && !follower.Done(*data_); ++i) {
    Tick(follower, watch);
  }
  ASSERT_TRUE(follower.Done(*data_));
  std::printf("Done %.3f s after the plan ended, %.1e rad from the goal\n",
              data_->time - end,
              (Angles() - kReachLeft).cwiseAbs().maxCoeff());

  // Done allows up to kArrivedAngle; the arm goes on settling to far less
  // within a few tenths of a second, then holds for five seconds.
  for (int i = 0; i < 100; ++i) Tick(follower, watch);
  Watch hold;
  double drift = 0.0;
  for (int i = 0; i < 2500; ++i) {
    Tick(follower, hold);
    drift = std::max(drift, (Angles() - kReachLeft).cwiseAbs().maxCoeff());
  }
  EXPECT_TRUE(follower.Done(*data_));
  EXPECT_LT(drift, 1e-6);
  EXPECT_LT(hold.speed.maxCoeff(), 1e-5);

  // Hold takes the measured angles and keeps them, five seconds more.
  follower.Hold(*data_);
  const VectorXd held = Angles();
  EXPECT_EQ(follower.Goal(*data_), held);
  for (int i = 0; i < 2500; ++i) Tick(follower, hold);
  EXPECT_LT((Angles() - held).cwiseAbs().maxCoeff(), 1e-6);
  EXPECT_TRUE(follower.Done(*data_));
}

TEST_F(ArmFollowerTest, HoldOnAMovingArmStopsIt) {
  // Not how to stop in normal use, but defined: the arm brakes hard and
  // ends where it was when Hold was called.
  ArmFollower follower(arm_);
  ASSERT_TRUE(follower.MoveTo(kTurned, *data_));
  Watch watch;
  for (int i = 0; i < 600; ++i) Tick(follower, watch);
  follower.Hold(*data_);
  EXPECT_EQ(follower.TimeLeft(*data_), 0.0);
  const VectorXd held = follower.Goal(*data_);
  for (int i = 0; i < 1500; ++i) Tick(follower, watch);
  EXPECT_TRUE(follower.Done(*data_));
  EXPECT_LT((Angles() - held).cwiseAbs().maxCoeff(), 1e-6);
}

TEST_F(ArmFollowerTest, WaypointsAreVisitedInTurn) {
  ArmFollower follower(arm_);
  const std::vector<VectorXd> waypoints = {kReachLeft, kBigMove, kTurned};
  std::string why;
  ASSERT_TRUE(follower.MoveThrough(waypoints, *data_, {}, &why)) << why;

  // The whole run takes as long as the legs one after another.
  const JointLimits limits = Scaled(arm_.limits, MoveOptions::kDefaultSpeed);
  double total = 0.0;
  VectorXd from = kHome;
  for (const VectorXd& to : waypoints) {
    total += JointMove::Plan(from, to, limits, ProfileShape::kSCurve)
                 ->duration();
    from = to;
  }
  EXPECT_NEAR(follower.TimeLeft(*data_), total, 1e-9);
  EXPECT_EQ(follower.Goal(*data_), waypoints.back());

  // The arm passes within a hair of each waypoint, in turn.
  std::vector<double> closest(waypoints.size(), 1e9);
  size_t next = 0;
  Watch watch;
  while (follower.TimeLeft(*data_) > 0.0) {
    Tick(follower, watch);
    for (size_t k = next; k < waypoints.size(); ++k) {
      closest[k] = std::min(
          closest[k], (Angles() - waypoints[k]).cwiseAbs().maxCoeff());
    }
    if (next < waypoints.size() && closest[next] < 1e-4) ++next;
  }
  EXPECT_EQ(next, waypoints.size());
  RunPast(follower, 0.2);
  EXPECT_TRUE(follower.Done(*data_));
  EXPECT_LT(watch.tracking.maxCoeff(), kTrackingTolerance);
}

TEST_F(ArmFollowerTest, WorksWhenTheArmIsNotFirstInTheModel) {
  // A model with another jointed body and motor ahead of the arm, so the
  // arm's joints start at qpos[1] and its motors at ctrl[1]. The follower
  // must touch only the arm's entries.
  const std::string path = std::getenv("MODEL_PATH");
  const std::string models_dir = path.substr(0, path.rfind('/') + 1);
  const std::string xml =
      "<mujoco><compiler angle='radian' autolimits='true' meshdir='" +
      models_dir +
      "'/><option timestep='0.002' integrator='implicitfast'/>"
      "<worldbody><body name='spinner' pos='5 5 1'><joint name='spin'/>"
      "<geom size='0.1' mass='1'/></body>"
      "<frame name='arm_mount'/></worldbody>"
      "<actuator><position name='spinner_motor' joint='spin' kp='10'/>"
      "</actuator></mujoco>";
  char error[1000] = "";
  mjSpec* parent =
      mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(parent, nullptr) << error;
  mjSpec* child = mj_parseXML(path.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(child, nullptr) << error;
  mjsElement* attached =
      mjs_attach(mjs_findFrame(parent, "arm_mount")->element,
                 mjs_findBody(child, "arm_base")->element, "", "");
  ASSERT_NE(attached, nullptr) << mjs_getError(parent);
  cell::ModelPtr model(mj_compile(parent, nullptr));
  const std::string compile_error = mjs_getError(parent);
  mj_deleteSpec(parent);
  mj_deleteSpec(child);
  ASSERT_TRUE(model) << compile_error;

  std::optional<Arm> arm = cell::FindArm(*model);
  ASSERT_TRUE(arm.has_value());
  EXPECT_EQ(arm->qpos[0], 1);
  EXPECT_EQ(arm->motor[0], 1);

  cell::DataPtr data = cell::MakeData(*model);
  for (int i = 0; i < kJoints; ++i) {
    data->qpos[arm->qpos[i]] = kHome[i];
    data->ctrl[arm->motor[i]] = kHome[i];
  }
  data->ctrl[0] = 0.7;  // the spinner's own command
  mj_forward(model.get(), data.get());

  ArmFollower follower(*arm);
  ASSERT_TRUE(follower.MoveTo(kBigMove, *data));
  const double end = data->time + follower.TimeLeft(*data) + 0.5;
  while (data->time < end) {
    follower.Update(*model, data.get());
    mj_step(model.get(), data.get());
  }
  EXPECT_TRUE(follower.Done(*data));
  EXPECT_LT((cell::JointAngles(*arm, *data) - kBigMove).cwiseAbs().maxCoeff(),
            1e-6);
  EXPECT_EQ(data->ctrl[0], 0.7);
}

// ------------------------------------------- tool-tip path of short moves

// Joint angles near `q` that put the tool tip at `target`, by Newton's
// method on the tip's Jacobian. Good enough for nearby targets; for the
// test only.
VectorXd SolveNear(const mjModel& model, mjData* data, const Arm& arm,
                   VectorXd q, const Isometry3d& target) {
  std::vector<mjtNum> jacp(3 * model.nv), jacr(3 * model.nv);
  for (int iteration = 0; iteration < 200; ++iteration) {
    for (int i = 0; i < kJoints; ++i) data->qpos[arm.qpos[i]] = q[i];
    mj_kinematics(&model, data);
    mj_comPos(&model, data);
    const Isometry3d tip = cell::TipPose(arm, *data);
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target.translation() - tip.translation();
    const Eigen::AngleAxisd turn(target.linear() * tip.linear().transpose());
    error.tail<3>() = turn.angle() * turn.axis();
    if (error.norm() < 1e-13) break;
    mj_jacSite(&model, data, jacp.data(), jacr.data(), arm.tip_site);
    Eigen::Matrix<double, 6, 6> jacobian;
    for (int r = 0; r < 3; ++r) {
      for (int i = 0; i < kJoints; ++i) {
        jacobian(r, i) = jacp[r * model.nv + arm.dof[i]];
        jacobian(r + 3, i) = jacr[r * model.nv + arm.dof[i]];
      }
    }
    // A short step at a time: a full Newton step can overshoot far.
    VectorXd step = jacobian.colPivHouseholderQr().solve(error);
    const double largest = step.cwiseAbs().maxCoeff();
    if (largest > 0.1) step *= 0.1 / largest;
    q += step;
  }
  return q;
}

TEST_F(ArmFollowerTest, ShortJointMovesKeepTheTipNearTheStraightLine) {
  // A joint move is a straight line in joint space, which is a curve at the
  // tool tip. For moves between two tip poses a short distance apart with
  // the same orientation, this measures how far the tip strays from the
  // straight line between them: on the plan (kinematics alone) and as the
  // arm really moves, at full speed.
  cell::DataPtr scratch = cell::MakeData(*model_);
  const std::vector<VectorXd> starts = {
      kHome,                             // tool down
      // Tool near level toward +X, the wrist bent a little to keep clear of
      // the wrist's straight-through singularity.
      VectorXd{{0, -1.2, 1.5, 0, -0.3, 0}},
      // Out to the side (+Y), upper arm leaning forward, tool down.
      VectorXd{{-1.5, -0.8, 0.8, 0, kHalfPi, 0}},
  };
  const std::vector<Vector3d> directions = {
      Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
      Vector3d(1, 1, 1).normalized()};

  const char* names[] = {"home, tool down", "tool near level",
                         "out to the side"};
  for (double length : {0.01, 0.03, 0.10}) {
    double worst_plan = 0.0, worst_real = 0.0, worst_turn = 0.0;
    double longest = 0.0;
    for (size_t n = 0; n < starts.size(); ++n) {
      const VectorXd& qa = starts[n];
      double plan_here = 0.0;
      for (const Vector3d& direction : directions) {
        for (int i = 0; i < kJoints; ++i) scratch->qpos[arm_.qpos[i]] = qa[i];
        mj_kinematics(model_.get(), scratch.get());
        const Isometry3d a = cell::TipPose(arm_, *scratch);
        Isometry3d b = a;
        b.translation() += length * direction;
        const VectorXd qb = SolveNear(*model_, scratch.get(), arm_, qa, b);
        mj_kinematics(model_.get(), scratch.get());
        ASSERT_LT((cell::TipPose(arm_, *scratch).translation() -
                   b.translation())
                      .norm(),
                  1e-9)
            << "no solution for " << length << " m along"
            << Format(direction) << " from" << Format(qa);

        // On the plan: how far off the line, and how far the tool turns
        // away from its orientation at both ends.
        const JointMove move =
            *JointMove::Plan(qa, qb, arm_.limits, ProfileShape::kSCurve);
        longest = std::max(longest, move.duration());
        for (int k = 0; k <= 200; ++k) {
          const VectorXd q = move.At(move.duration() * k / 200).position;
          for (int i = 0; i < kJoints; ++i) scratch->qpos[arm_.qpos[i]] = q[i];
          mj_kinematics(model_.get(), scratch.get());
          const Isometry3d tip = cell::TipPose(arm_, *scratch);
          plan_here = std::max(
              plan_here, DistanceToSegment(tip.translation(), a.translation(),
                                           b.translation()));
          worst_turn = std::max(
              worst_turn,
              Eigen::AngleAxisd(tip.linear() * a.linear().transpose()).angle());
        }

        // As the arm moves.
        Reset();
        ArmFollower follower(arm_);
        if (qa != kHome) GoTo(follower, qa);
        std::string why;
        ASSERT_TRUE(follower.MoveTo(qb, *data_, {.speed = 1.0}, &why)) << why;
        Watch watch;
        const double end = data_->time + follower.TimeLeft(*data_) + 0.3;
        while (data_->time < end) {
          Tick(follower, watch);
          mj_kinematics(model_.get(), data_.get());
          worst_real = std::max(
              worst_real,
              DistanceToSegment(cell::TipPose(arm_, *data_).translation(),
                                a.translation(), b.translation()));
        }
      }
      std::printf("  %3.0f mm from %s: up to %.3f mm off the line\n",
                  length * 1e3, names[n], plan_here * 1e3);
      worst_plan = std::max(worst_plan, plan_here);
    }
    std::printf(
        "joint move over %.0f mm: tip strays from the straight line by up to "
        "%.3f mm on the plan, %.3f mm as the arm moves; the tool turns by up "
        "to %.3f mrad; each move takes up to %.2f s at full speed\n",
        length * 1e3, worst_plan * 1e3, worst_real * 1e3, worst_turn * 1e3,
        longest);
    if (length == 0.03) {
      EXPECT_LT(worst_plan, 3e-4);
      EXPECT_LT(worst_real, 3e-4);
    }
  }
}

}  // namespace
