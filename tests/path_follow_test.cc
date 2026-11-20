// Checks the arm following blended trajectories (ArmFollower::MoveAlong and
// Follow, src/path_timing.h) in the arm's own scene: through several
// waypoints in one motion, with and without a module at the tool, within the
// same speed, acceleration, torque and tracking bounds as
// tests/arm_follower_test.cc, and clearly quicker than stopping at every
// waypoint; and a straight tool-tip line solved every centimetre, run as one
// motion, against stopping at every centimetre.
//
//   bazel test //:path_follow_test
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/arm_scene.xml). Every test starts
// from the keyframe arm_home. To see the measured numbers:
//   bazel test //:path_follow_test --test_output=all

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
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/arm_follower.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/path_timing.h"

namespace {

using cell::Arm;
using cell::ArmFollower;
using cell::JointLimits;
using cell::MoveOptions;
using cell::PathTimingOptions;
using cell::Trajectory;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr int kJoints = Arm::kJoints;
constexpr double kHalfPi = 1.5707963267948966;

// A module (as in src/pallet.h): 2.3 m x 1.1 m x 35 mm, 30 kg.
constexpr double kModuleLength = 2.3;
constexpr double kModuleWidth = 1.1;
constexpr double kModuleThickness = 0.035;
constexpr double kModuleMass = 30.0;

// Where the cups meet the glass, along the tool's +Z (models/tool.xml).
constexpr double kTipOut = 0.30;

// The same bounds as tests/arm_follower_test.cc: measured speeds and
// accelerations may pass the limits by this fraction, the tracking error
// stays under this while moving, and torques under this fraction of their
// limits at the default speed.
constexpr double kLimitAllowance = 0.02;
constexpr double kTrackingTolerance = 3e-4;
constexpr double kTorqueCeiling = 0.95;

const VectorXd kHome{{0, -kHalfPi, kHalfPi, 0, kHalfPi, 0}};

// A planner-like path: from home round to the other side with the upper arm
// leaning out, the wrist turning as it goes. Its corners turn by 10 to 25
// degrees.
const std::vector<VectorXd> kSweep = {
    kHome + VectorXd{{0.35, 0.10, -0.10, 0.10, -0.05, 0.10}},
    kHome + VectorXd{{0.80, 0.30, -0.20, 0.25, -0.15, 0.25}},
    kHome + VectorXd{{1.30, 0.45, -0.25, 0.45, -0.30, 0.45}},
    kHome + VectorXd{{1.80, 0.50, -0.20, 0.60, -0.40, 0.70}},
    kHome + VectorXd{{2.30, 0.40, -0.05, 0.70, -0.45, 0.90}},
    kHome + VectorXd{{2.70, 0.20, 0.10, 0.75, -0.45, 1.00}},
};

// The same as tests/arm_follower_test.cc: with `module` true, a module is
// fixed to the tool as if held, a body with no joint, a child of the tool
// body, so the arm's inertia and gravity include it.
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
    plate->inertia[0] = m / 12 * (b * b + t * t);
    plate->inertia[1] = m / 12 * (a * a + t * t);
    plate->inertia[2] = m / 12 * (a * a + b * b);
    plate->explicitinertial = 1;
  }
  mjModel* model = mj_compile(spec, nullptr);
  if (model == nullptr) *error = mjs_getError(spec);
  mj_deleteSpec(spec);
  return cell::ModelPtr(model);
}

std::string Format(const VectorXd& v, double scale = 1.0) {
  std::string out;
  char buffer[32];
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    std::snprintf(buffer, sizeof(buffer), " %7.3f", v[i] * scale);
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

// What a stretch of simulation saw, as in tests/arm_follower_test.cc.
// Peaks are of magnitudes; speed, acceleration and torque are fractions of
// each joint's limit.
struct Watch {
  VectorXd tracking = VectorXd::Zero(kJoints);  // rad, while a plan runs
  VectorXd speed = VectorXd::Zero(kJoints);
  VectorXd acceleration = VectorXd::Zero(kJoints);
  VectorXd torque = VectorXd::Zero(kJoints);
  VectorXd command_step = VectorXd::Zero(kJoints);  // largest tick-to-tick
  double range_margin = std::numeric_limits<double>::infinity();
  double done_at = -1.0;  // first time Done(), or -1
  bool touched = false;   // any contact at all
};

class PathFollowTest : public ::testing::Test {
 protected:
  void SetUp() override { Load(false); }

  void Load(bool module) {
    std::string error;
    model_ = LoadScene(module, &error);
    ASSERT_TRUE(model_) << error;
    data_ = cell::MakeData(*model_);
    scratch_ = cell::MakeData(*model_);
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
    // The acceleration over the step, from the change of speed (MuJoCo's
    // qacc overstates it; see tests/arm_follower_test.cc).
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
    if (follower.Done(*data_) && watch.done_at < 0.0) {
      watch.done_at = data_->time;
    }
    if (data_->ncon > 0) watch.touched = true;
  }

  // Ticks until the plan has ended and then `extra` seconds more. Calls
  // `each` after every tick.
  template <typename Each>
  Watch RunPast(ArmFollower& follower, double extra, Each each) {
    Watch watch;
    const double end = data_->time + follower.TimeLeft(*data_) + extra;
    while (data_->time < end) {
      Tick(follower, watch);
      each();
    }
    return watch;
  }
  Watch RunPast(ArmFollower& follower, double extra) {
    return RunPast(follower, extra, [] {});
  }

  // Goes from home to `pose` gently and lets it settle.
  void GoTo(ArmFollower& follower, const VectorXd& pose) {
    ASSERT_TRUE(follower.MoveTo(pose, *data_, {.speed = 0.5}));
    RunPast(follower, 0.5);
    ASSERT_TRUE(follower.Done(*data_));
  }

  // Checks a run against the bounds of tests/arm_follower_test.cc at
  // `speed`, and that it ended on `goal`.
  void ExpectWithinBounds(const Watch& watch, const ArmFollower& follower,
                          const VectorXd& goal, double speed) {
    EXPECT_FALSE(watch.touched);
    EXPECT_GT(watch.range_margin, 0.0);
    EXPECT_TRUE(follower.Done(*data_));
    EXPECT_LT((Angles() - goal).cwiseAbs().maxCoeff(), 1e-6);
    const double dt = model_->opt.timestep;
    for (int i = 0; i < kJoints; ++i) {
      SCOPED_TRACE("joint " + std::to_string(i + 1));
      EXPECT_LE(watch.speed[i], speed * (1 + kLimitAllowance));
      EXPECT_LE(watch.acceleration[i], speed * speed * (1 + kLimitAllowance));
      EXPECT_LT(watch.tracking[i], kTrackingTolerance);
      EXPECT_LT(watch.torque[i], kTorqueCeiling);
      // The command never jumps, from the move before to this one or along
      // it: per tick no more than the fastest the joint moves plus the
      // lead term's change (the bound of SuccessiveMovesJoinWithoutAJump).
      EXPECT_LT(watch.command_step[i],
                (arm_.limits.velocity[i] +
                 arm_.lag[i] * arm_.limits.acceleration[i]) *
                        dt * 1.1 +
                    1e-5);
    }
  }

  Isometry3d Tip() const { return cell::TipPose(arm_, *data_); }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  cell::DataPtr scratch_;
  Arm arm_;
};

// Several waypoints in one motion, with and without a module held.
class PayloadTest : public PathFollowTest,
                    public ::testing::WithParamInterface<bool> {
 protected:
  void SetUp() override { Load(GetParam()); }
};

// kSweep with points added along each leg, no two more than `spacing`
// apart, as a planner's path is when resampled for checking.
std::vector<VectorXd> Resampled(double spacing) {
  std::vector<VectorXd> out;
  VectorXd from = kHome;
  for (const VectorXd& to : kSweep) {
    const int parts =
        static_cast<int>(std::ceil((to - from).norm() / spacing));
    for (int k = 1; k <= parts; ++k) {
      out.push_back(from + (to - from) * (static_cast<double>(k) / parts));
    }
    from = to;
  }
  return out;
}

TEST_P(PayloadTest, MoveAlongIsWithinTheLimitsAndQuickerThanStopping) {
  const double speed = MoveOptions::kDefaultSpeed;
  struct Run {
    double planned = 0.0;  // s, the plan's duration
    double done = 0.0;     // s, until Done()
    double off = 0.0;      // rad, the plan's farthest from the polyline
    Watch watch;
  };
  // From home through `waypoints`, stopping at each or in one motion.
  auto run = [&](const std::vector<VectorXd>& waypoints, bool blended,
                 double deviation) {
    Run result;
    Reset();
    ArmFollower follower(arm_);
    std::string why;
    const double start = data_->time;
    if (blended) {
      EXPECT_TRUE(follower.MoveAlong(waypoints, *data_,
                                     {.max_deviation = deviation}, &why))
          << why;
    } else {
      EXPECT_TRUE(follower.MoveThrough(waypoints, *data_, {}, &why)) << why;
    }
    result.planned = follower.TimeLeft(*data_);
    result.watch = RunPast(follower, 0.5, [&] {
      // How far the plan is from the straight lines between waypoints.
      const VectorXd planned = follower.Planned(*data_).position;
      double closest = std::numeric_limits<double>::infinity();
      VectorXd from = kHome;
      for (const VectorXd& to : waypoints) {
        const VectorXd ab = to - from;
        const double x =
            std::clamp((planned - from).dot(ab) / ab.squaredNorm(), 0.0, 1.0);
        closest = std::min(closest, (planned - from - x * ab).norm());
        from = to;
      }
      result.off = std::max(result.off, closest);
    });
    EXPECT_GE(result.watch.done_at, 0.0);
    result.done = result.watch.done_at - start;
    ExpectWithinBounds(result.watch, follower, waypoints.back(), speed);
    EXPECT_LE(result.off, (blended ? deviation : 0.0) + 1e-9);
    return result;
  };
  auto print = [&](const char* name, const Run& r) {
    std::printf(
        "%s, %s: planned %.2f s, Done after %.2f s, %.2e rad off the lines\n"
        "  torque/limit:%s\n  speed/limit: %s\n  accel/limit: %s\n"
        "  tracking, mrad:%s\n",
        GetParam() ? "with module" : "no module", name, r.planned, r.done,
        r.off, Format(r.watch.torque).c_str(), Format(r.watch.speed).c_str(),
        Format(r.watch.acceleration).c_str(),
        Format(r.watch.tracking, 1e3).c_str());
  };

  // The corners alone. Within the default 2 mrad a corner of 10 to 25
  // degrees must be taken at a few tenths of a radian per second, so little
  // is saved; a looser deviation saves more.
  const double loose = 0.01;
  const Run through = run(kSweep, false, 0.0);
  const Run along = run(kSweep, true, PathTimingOptions::kDefaultMaxDeviation);
  const Run along_loose = run(kSweep, true, loose);
  print("6 corners, MoveThrough", through);
  print("6 corners, MoveAlong 0.002 rad", along);
  print("6 corners, MoveAlong 0.01 rad", along_loose);
  EXPECT_LT(along.done, through.done);
  EXPECT_LT(along_loose.done, along.done);

  // The same path resampled every 0.1 rad: MoveThrough stops at every
  // point, MoveAlong only slows at the corners.
  const std::vector<VectorXd> dense = Resampled(0.1);
  const Run dense_through = run(dense, false, 0.0);
  const Run dense_along =
      run(dense, true, PathTimingOptions::kDefaultMaxDeviation);
  std::printf("resampled to %zu waypoints:\n", dense.size());
  print("resampled, MoveThrough", dense_through);
  print("resampled, MoveAlong 0.002 rad", dense_along);
  EXPECT_LT(dense_along.done, 0.5 * dense_through.done);
  EXPECT_LT(dense_along.done, 1.05 * along.done);
}

INSTANTIATE_TEST_SUITE_P(Payload, PayloadTest, ::testing::Bool(),
                         [](const ::testing::TestParamInfo<bool>& info) {
                           return info.param ? "WithModule" : "NoModule";
                         });

TEST_F(PathFollowTest, FollowRunsATrajectoryFromTheCommandedPose) {
  std::vector<VectorXd> path = {kHome};
  path.insert(path.end(), kSweep.begin(), kSweep.end());
  const Trajectory trajectory =
      *Trajectory::Plan(path, arm_.limits, {.speed = 0.9});
  ArmFollower follower(arm_);
  std::string why;
  ASSERT_TRUE(follower.Follow(trajectory, *data_, &why)) << why;
  EXPECT_NEAR(follower.TimeLeft(*data_), trajectory.duration(), 1e-12);
  EXPECT_EQ(follower.Goal(*data_), kSweep.back());
  // The plan is the trajectory, shifted to start now.
  for (double t : {0.0, 0.7, 2.0}) {
    data_->time += t;
    EXPECT_EQ(follower.Planned(*data_).position, trajectory.At(t).position);
    data_->time -= t;
  }
  // While it runs, nothing else is accepted.
  Watch watch;
  for (int k = 0; k < 100; ++k) Tick(follower, watch);
  EXPECT_FALSE(follower.Follow(trajectory, *data_, &why));
  EXPECT_FALSE(follower.MoveAlong({kHome}, *data_, {}, &why));
  EXPECT_FALSE(follower.MoveTo(kHome, *data_, {}, &why));
  watch = RunPast(follower, 0.5);
  ExpectWithinBounds(watch, follower, kSweep.back(), 0.9);

  // A trajectory that does not start where the arm is commanded to be is
  // refused: the command would jump.
  EXPECT_FALSE(follower.Follow(trajectory, *data_, &why));
  std::printf("for example: %s\n", why.c_str());
  // And after it, a move from where it ended is fine.
  EXPECT_TRUE(follower.MoveTo(kHome, *data_, {}, &why)) << why;
}

TEST_F(PathFollowTest, MoveAlongRefusesWhatMoveThroughRefuses) {
  ArmFollower follower(arm_);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  std::string why;
  EXPECT_FALSE(follower.MoveAlong({}, *data_, {}, &why));
  VectorXd outside = kSweep[1];
  outside[1] = arm_.upper[1] + 0.01;
  EXPECT_FALSE(follower.MoveAlong({kSweep[0], outside}, *data_, {}, &why));
  EXPECT_FALSE(why.empty());
  EXPECT_FALSE(follower.MoveAlong({VectorXd::Zero(5)}, *data_, {}, &why));
  VectorXd not_finite = kSweep[0];
  not_finite[3] = nan;
  EXPECT_FALSE(follower.MoveAlong({not_finite}, *data_, {}, &why));
  for (double speed : {0.0, 1.5, nan}) {
    EXPECT_FALSE(follower.MoveAlong(kSweep, *data_, {.speed = speed}, &why));
  }
  for (double deviation : {-0.001, nan}) {
    EXPECT_FALSE(follower.MoveAlong(kSweep, *data_,
                                    {.max_deviation = deviation}, &why));
  }
  std::printf("for example: %s\n", why.c_str());
  // Nothing was planned; the arm stays at home.
  EXPECT_EQ(follower.TimeLeft(*data_), 0.0);
  EXPECT_EQ(follower.Goal(*data_), kHome);
  // One waypoint is a single move to it, as good as MoveTo.
  EXPECT_TRUE(follower.MoveAlong({kSweep[2]}, *data_, {}, &why)) << why;
  ArmFollower other(arm_);
  ASSERT_TRUE(other.MoveTo(kSweep[2], *data_));
  EXPECT_NEAR(follower.TimeLeft(*data_), other.TimeLeft(*data_),
              1e-9 * other.TimeLeft(*data_));
}

// ------------------------------------------------- a straight tool-tip line

// What one way of running a line did.
struct LineRun {
  double planned = 0.0;      // s
  double done = 0.0;         // s, until Done()
  double plan_off = 0.0;     // m, the planned tip's farthest from the line
  double real_off = 0.0;     // m, the measured tip's
  Watch watch;
};

// The tool tip 0.3 m along a straight line, from two starting poses, with
// and without a module: solved every centimetre by the IK, then run (a) as
// today, one rest-to-rest joint move per centimetre (MoveThrough), and (b)
// as one blended trajectory (MoveAlong) at two deviations. Also (c), one
// joint move straight from end to end, for reference: quickest, but a
// straight line in joint space is a curve at the tip.
TEST_F(PathFollowTest, StraightToolLineAsOneMotion) {
  struct Line {
    const char* name;
    VectorXd start;
    Vector3d along;  // m
  };
  const std::vector<Line> lines = {
      // Tool near level toward +X, pulled straight back 0.3 m toward the
      // base, as from the face of a module.
      {"pull back, tool level", VectorXd{{0, -1.2, 1.5, 0, -0.3, 0}},
       Vector3d(-0.3, 0, 0)},
      // From home, tool down, straight up 0.3 m.
      {"lift, tool down", kHome, Vector3d(0, 0, 0.3)},
  };
  const double tight = 2e-4;  // rad
  for (bool module : {false, true}) {
    Load(module);
    for (const Line& line : lines) {
      SCOPED_TRACE(line.name);
      const Isometry3d from =
          cell::TipPoseAt(*model_, scratch_.get(), arm_, line.start);
      Isometry3d to = from;
      to.translation() += line.along;
      const std::vector<Isometry3d> poses = cell::StraightLine(from, to, 0.01);
      ASSERT_EQ(poses.size(), 31u);
      cell::IkPathReport report;
      const std::optional<std::vector<VectorXd>> path = cell::SolveIkPath(
          *model_, scratch_.get(), arm_, line.start, poses, 0.05, {}, &report);
      ASSERT_TRUE(path) << "waypoint " << report.failed_waypoint;

      // Runs one way of doing it from a settled start.
      enum class Way { kThrough, kAlong, kAlongTight, kOneMove };
      auto run = [&](Way way) {
        LineRun result;
        Reset();
        ArmFollower follower(arm_);
        if (line.start != kHome) GoTo(follower, line.start);
        std::string why;
        bool ok = false;
        switch (way) {
          case Way::kThrough:
            ok = follower.MoveThrough(*path, *data_, {}, &why);
            break;
          case Way::kAlong:
            ok = follower.MoveAlong(*path, *data_, {}, &why);
            break;
          case Way::kAlongTight:
            ok = follower.MoveAlong(*path, *data_, {.max_deviation = tight},
                                    &why);
            break;
          case Way::kOneMove:
            ok = follower.MoveTo(path->back(), *data_, {}, &why);
            break;
        }
        EXPECT_TRUE(ok) << why;
        const double start = data_->time;
        result.planned = follower.TimeLeft(*data_);
        result.watch = RunPast(follower, 0.5, [&] {
          const VectorXd planned = follower.Planned(*data_).position;
          result.plan_off = std::max(
              result.plan_off,
              DistanceToSegment(
                  cell::TipPoseAt(*model_, scratch_.get(), arm_, planned)
                      .translation(),
                  from.translation(), to.translation()));
          mj_kinematics(model_.get(), data_.get());
          result.real_off = std::max(
              result.real_off,
              DistanceToSegment(Tip().translation(), from.translation(),
                                to.translation()));
        });
        result.done = result.watch.done_at - start;
        ExpectWithinBounds(result.watch, follower, path->back(),
                           MoveOptions::kDefaultSpeed);
        return result;
      };
      const LineRun through = run(Way::kThrough);
      const LineRun along = run(Way::kAlong);
      const LineRun along_tight = run(Way::kAlongTight);
      const LineRun one = run(Way::kOneMove);

      std::printf("%s, %s, 0.3 m in 31 waypoints:\n", line.name,
                  module ? "with module" : "no module");
      for (const auto& [name, r] :
           {std::pair<const char*, const LineRun&>{
                "stop every cm (MoveThrough)  ", through},
            {"MoveAlong, 0.002 rad         ", along},
            {"MoveAlong, 0.0002 rad        ", along_tight},
            {"one joint move, end to end   ", one}}) {
        std::printf(
            "  %s planned %.2f s, Done %.2f s; tip off the line %.3f mm "
            "planned, %.3f mm measured; torque/limit up to %.2f, tracking up "
            "to %.3f mrad\n",
            name, r.planned, r.done, r.plan_off * 1e3, r.real_off * 1e3,
            r.watch.torque.maxCoeff(), r.watch.tracking.maxCoeff() * 1e3);
      }
      EXPECT_LT(along.done, 0.3 * through.done);
      EXPECT_LT(along_tight.done, 0.4 * through.done);
      // 0.2 mrad of joint-space deviation is a fraction of a millimetre at
      // the tip.
      EXPECT_LT(along_tight.real_off, 0.5e-3);
      EXPECT_LT(along.real_off, 3e-3);
    }
  }
}

}  // namespace
