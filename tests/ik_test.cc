// Checks the inverse kinematics: that the angles it returns put the tool tip
// where it was asked to be, that it finds the solution near its seed, that
// it fails cleanly where there is no answer or the arm is singular, and that
// it leaves everything but the arm's angles in its scratch mjData alone.
// Also checks the poses the cell needs, with the real arm, and prints their
// joint angles and how far each joint is from its limits.
//
//   bazel test //:ik_test
//   bazel test --config=opt //:ik_test --test_output=all   (timings)
//
// The scenes' paths come from the MODEL_PATH (models/arm_scene.xml) and
// IK_SCENE_PATH (models/ik_scene.xml) environment variables, which
// BUILD.bazel sets for this test.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/ik.h"
#include "src/mj_handles.h"

namespace {

using cell::Arm;
using cell::IkOptions;
using cell::IkPathReport;
using cell::IkReport;
using cell::IkStatus;
using cell::SolveIk;
using cell::SolveIkPath;
using cell::TipPoseAt;
using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;
constexpr double kMillimetre = 0.001;
constexpr double kHalfPi = kPi / 2.0;

// ------------------------------------------------------------- the layout
// Where the cell's pallet, modules and conveyor are. These mirror
// models/pallet.xml (the pallet and its stack) and src/conveyor.h (where a
// module is put down); when the layout changes there, change it here, in
// this one place. Every cell pose in this file is built from these.

// The pallet's origin: on the floor, under the front module's glass centre.
const Vector3d kPalletOrigin(1.95, 0.0, 0.0);
// module_0's glass centre. module_0 is the front module, nearest the arm.
const Vector3d kModule0Glass(1.95, 0.0, 0.75);
constexpr int kModuleCount = 30;
// Each next module's glass is one module thickness further into the stack,
// straight along the glass normal (they stand touching).
constexpr double kModulePitch = 0.035;
// The stack leans back 10 degrees: the glass's outward normal points toward
// the arm and a little up.
const double kLean = 10.0 * kDegree;
const Vector3d kGlassNormal(-std::cos(kLean), 0.0, std::sin(kLean));
// Up the lean, in the glass: the direction a gripped module is lifted.
const Vector3d kUpTheGlass(std::sin(kLean), 0.0, std::cos(kLean));
// A standing module's +x, its long side: level, along world -Y.
const Vector3d kModuleLongSide(0.0, -1.0, 0.0);
// How far off nominal the pallet may be set down: this much either way in
// x and in y, and this much turned either way about the vertical through
// its origin.
constexpr double kPalletOffset = 0.03;
const double kPalletYaw = 3.0 * kDegree;

// Over the conveyor: the tool tip pointing straight down, the module's +x
// (and the tool's X) along world +X, as Conveyor::PlacementPose has it.
// That pose puts the module's centre at (-2.0, 0, 0.8175) and its glass, where
// the tip is at release, at z = 0.835.
const Vector3d kPlaceTip(-2.0, 0.0, 0.835);
const Vector3d kPlaceLongSide(1.0, 0.0, 0.0);

// One module's glass, wherever the pallet was set down.
struct Glass {
  Vector3d centre;
  Vector3d normal;     // out of the glass
  Vector3d long_side;  // the module's +x
  Vector3d up;         // up the lean, in the glass
};

// module_<index>'s glass with the pallet moved by (dx, dy) and turned by
// `yaw` about the vertical through its origin, as src/pallet.h's
// PlacePallet does.
Glass ModuleGlass(int index, double dx = 0.0, double dy = 0.0,
                  double yaw = 0.0) {
  const Eigen::Matrix3d turn =
      Eigen::AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix();
  const Vector3d nominal =
      kModule0Glass - index * kModulePitch * kGlassNormal;
  Glass g;
  g.centre = turn * (nominal - kPalletOrigin) + kPalletOrigin +
             Vector3d(dx, dy, 0.0);
  g.normal = turn * kGlassNormal;
  g.long_side = turn * kModuleLongSide;
  g.up = turn * kUpTheGlass;
  return g;
}

// The faces the reachability checks use: the front and back of the stack.
const Vector3d kNearFace = ModuleGlass(0).centre;
const Vector3d kFarFace = ModuleGlass(kModuleCount - 1).centre;

// Gripping a standing module at `centre`, the tool's X along the module's
// +x (x_sign = +1) or the other way (-1).
Isometry3d FacePose(const Vector3d& centre, double x_sign) {
  return *cell::GripPose(centre, kGlassNormal, x_sign * kModuleLongSide);
}

// Over the conveyor, the tool's X along the module's +x there (x_sign = +1)
// or the other way.
Isometry3d PlacePose(double x_sign) {
  return *cell::GripPose(kPlaceTip, Vector3d::UnitZ(),
                         x_sign * kPlaceLongSide);
}

// The solver's default tolerances: 0.1 mm and 0.01 degrees.
const IkOptions kDefaults;

// KUKA's home with the tool facing down, as in the arm_home keyframe.
VectorXd Home() {
  VectorXd q(Arm::kJoints);
  q << 0, -kHalfPi, kHalfPi, 0, kHalfPi, 0;
  return q;
}

// A pose from where its origin is and where its Z and X axes point. X is
// made square to Z; Y completes a right-handed frame.
Isometry3d PoseFrom(const Vector3d& position, const Vector3d& z,
                    const Vector3d& x) {
  const Vector3d zn = z.normalized();
  const Vector3d xn = (x - x.dot(zn) * zn).normalized();
  Isometry3d pose = Isometry3d::Identity();
  pose.linear().col(0) = xn;
  pose.linear().col(1) = zn.cross(xn);
  pose.linear().col(2) = zn;
  pose.translation() = position;
  return pose;
}

double PositionError(const Isometry3d& a, const Isometry3d& b) {
  return (a.translation() - b.translation()).norm();
}

double AngleError(const Isometry3d& a, const Isometry3d& b) {
  return Eigen::AngleAxisd(a.linear() * b.linear().transpose()).angle();
}

bool WithinRanges(const Arm& arm, const VectorXd& q) {
  return (q.array() >= arm.lower.array()).all() &&
         (q.array() <= arm.upper.array()).all();
}

std::string Degrees(const VectorXd& q) {
  std::string s;
  char buffer[32];
  for (int i = 0; i < q.size(); ++i) {
    std::snprintf(buffer, sizeof(buffer), "%s%8.2f", i ? " " : "", q[i] / kDegree);
    s += buffer;
  }
  return s;
}

// For each joint, how far it is from the nearer end of its range.
VectorXd LimitMargins(const Arm& arm, const VectorXd& q) {
  return (q - arm.lower).cwiseMin(arm.upper - q);
}

// Uniformly random angles over every joint's whole range.
VectorXd RandomAngles(const Arm& arm, std::mt19937& rng) {
  VectorXd q(Arm::kJoints);
  for (int i = 0; i < Arm::kJoints; ++i) {
    q[i] = std::uniform_real_distribution<double>(arm.lower[i], arm.upper[i])(rng);
  }
  return q;
}

VectorXd Perturbed(const VectorXd& q, double size, std::mt19937& rng) {
  std::uniform_real_distribution<double> d(-size, size);
  VectorXd p = q;
  for (int i = 0; i < p.size(); ++i) p[i] += d(rng);
  return p;
}

const char* StatusName(IkStatus status) {
  switch (status) {
    case IkStatus::kConverged: return "converged";
    case IkStatus::kBadInput: return "bad input";
    case IkStatus::kIterationLimit: return "iteration limit";
    case IkStatus::kStalled: return "stalled";
    case IkStatus::kTooFarFromSeed: return "too far from seed";
  }
  return "?";
}

class IkTest : public ::testing::Test {
 protected:
  void SetUp() override { Load("MODEL_PATH"); }

  void Load(const char* variable) {
    const char* path = std::getenv(variable);
    ASSERT_NE(path, nullptr) << variable << " is not set";
    std::string error;
    model_ = cell::LoadModel(path, &error);
    ASSERT_TRUE(model_) << "could not load " << path << ": " << error;
    data_ = cell::MakeData(*model_);
    ASSERT_TRUE(data_);
    std::optional<Arm> arm = cell::FindArm(*model_);
    ASSERT_TRUE(arm);
    arm_ = *arm;
  }

  Isometry3d Tip(const VectorXd& q) {
    return TipPoseAt(*model_, data_.get(), arm_, q);
  }

  std::optional<VectorXd> Solve(const Isometry3d& target, const VectorXd& seed,
                                IkReport* report = nullptr,
                                const IkOptions& options = {}) {
    return SolveIk(*model_, data_.get(), arm_, target, seed, options, report);
  }

  // A returned answer must be finite, within the ranges, and put the tip on
  // the target within the solver's tolerances.
  void ExpectReaches(const VectorXd& q, const Isometry3d& target) {
    ASSERT_EQ(q.size(), Arm::kJoints);
    EXPECT_TRUE(q.allFinite());
    EXPECT_TRUE(WithinRanges(arm_, q)) << Degrees(q);
    const Isometry3d tip = Tip(q);
    EXPECT_LE(PositionError(tip, target), kDefaults.position_tolerance);
    EXPECT_LE(AngleError(tip, target), kDefaults.orientation_tolerance);
  }

  // Where the wrist centre (joints 4, 5 and 6 cross there) is, at `q`.
  Vector3d WristCentre(const VectorXd& q) {
    Tip(q);
    const int body = mj_name2id(model_.get(), mjOBJ_BODY, "arm_link_5");
    return Vector3d(data_->xpos[3 * body], data_->xpos[3 * body + 1],
                    data_->xpos[3 * body + 2]);
  }

  Vector3d Joint2(const VectorXd& q) {
    Tip(q);
    const int body = mj_name2id(model_.get(), mjOBJ_BODY, "arm_link_2");
    return Vector3d(data_->xpos[3 * body], data_->xpos[3 * body + 1],
                    data_->xpos[3 * body + 2]);
  }

  // Joint 3's angle with the arm stretched straight: the one that puts the
  // wrist centre furthest from joint 2. Found by scanning, since the
  // forearm's 0.115 m offset puts it a few degrees off zero. Joints 1 and 2
  // do not change it.
  double StretchedJoint3() {
    VectorXd q = Home();
    double best = 0.0;
    double best_q3 = 0.0;
    for (double q3 = -0.3; q3 <= 0.3; q3 += 1e-4) {
      q[2] = q3;
      const double reach = (WristCentre(q) - Joint2(q)).norm();
      if (reach > best) {
        best = reach;
        best_q3 = q3;
      }
    }
    return best_q3;
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  Arm arm_;
};

// Random poses the arm can reach, each from random angles over the full
// ranges, solved from a seed up to 0.1 rad off on every joint. The pose must
// be reproduced within the tolerances, every time.
//
// The angles must also come back, except near one of the arm's three
// singularities, where two different sets of angles a few degrees apart
// give the same pose and a seed 0.1 rad off can be nearer the other:
//   elbow     joint 3 near the stretched angle; the elbow bent a little the
//             other way gives the same wrist position (elbow up / down).
//   wrist     joint 5 near zero; joints 4 and 6 turn about one axis, so
//             they can trade angle, or the wrist can flip (joint 5 the
//             other sign).
//   shoulder  the wrist centre near joint 1's axis; joint 1 and the wrist
//             can trade angle.
// Away from those, "come back" means within 0.01 rad: the tolerances fix
// the pose to 0.1 mm and 0.01 deg, and in a poorly conditioned pose that
// leaves the angles free by more than the tolerance itself.
TEST_F(IkTest, RoundTripsFromNearbySeeds) {
  std::mt19937 rng(7);
  constexpr int kTrials = 1000;
  constexpr double kSameAngles = 0.01;
  const double stretched_q3 = StretchedJoint3();
  int other_solution = 0;
  int total_iterations = 0;
  int most_iterations = 0;
  double worst_position = 0.0;
  double worst_angle = 0.0;
  double worst_mismatch = 0.0;
  for (int trial = 0; trial < kTrials; ++trial) {
    const VectorXd truth = RandomAngles(arm_, rng);
    const Isometry3d target = Tip(truth);
    const VectorXd seed = Perturbed(truth, 0.1, rng);

    IkReport report;
    const std::optional<VectorXd> q = Solve(target, seed, &report);
    ASSERT_TRUE(q) << "trial " << trial << ": " << StatusName(report.status)
                   << ", " << report.position_error / kMillimetre << " mm, "
                   << report.orientation_error / kDegree << " deg, truth "
                   << Degrees(truth);
    ExpectReaches(*q, target);
    total_iterations += report.iterations;
    most_iterations = std::max(most_iterations, report.iterations);
    const Isometry3d tip = Tip(*q);
    worst_position = std::max(worst_position, PositionError(tip, target));
    worst_angle = std::max(worst_angle, AngleError(tip, target));

    const double mismatch = (*q - truth).cwiseAbs().maxCoeff();
    if (mismatch <= kSameAngles) {
      worst_mismatch = std::max(worst_mismatch, mismatch);
      continue;
    }
    ++other_solution;
    const double elbow = std::abs(truth[2] - stretched_q3);
    const double wrist = std::abs(truth[4]);
    const double shoulder = WristCentre(truth).head<2>().norm();
    EXPECT_TRUE(elbow < 0.15 || wrist < 0.15 || shoulder < 0.25)
        << "trial " << trial << " found other angles away from the "
        << "singularities (elbow " << elbow << " rad, wrist " << wrist
        << " rad, shoulder " << shoulder << " m): truth " << Degrees(truth)
        << ", found " << Degrees(*q);
  }
  std::printf(
      "round trips: %d/%d solved, worst %.2e mm %.2e deg, iterations mean "
      "%.1f max %d; %d ended on other angles near a singularity; the rest "
      "match the true angles within %.2e rad\n",
      kTrials, kTrials, worst_position / kMillimetre, worst_angle / kDegree,
      static_cast<double>(total_iterations) / kTrials, most_iterations,
      other_solution, worst_mismatch);
}

// From the home pose to random reachable poses. Damped least squares is a
// local method: from far away it can walk into a joint limit, or toward a
// branch of solutions that is out of range, and stall. So not every pose is
// found from home; the rate is reported and a floor asserted. Every answer
// returned must still be right.
//
// The floor: over the 1000 poses here, 900 were found (see the printed
// line). Every failure seen ended with a joint pressed against its limit,
// most often joint 2 at -140 degrees or joint 1 at +-185: the straight
// downhill route from home ran into a limit before the target. Random
// angles over the full ranges include many contorted poses (joints 4 and 6
// far round, the arm reaching back over itself) that a planner would never
// ask for from home; it would seed with joint 1 turned toward the target,
// as CellPoses does. The floor of 0.85 leaves room for small changes in the
// solver without letting it get worse in a way that matters.
TEST_F(IkTest, FarSeedsFromHome) {
  std::mt19937 rng(11);
  constexpr int kTrials = 1000;
  int solved = 0;
  int total_iterations = 0;
  int most_iterations = 0;
  int stalled = 0;
  int limit = 0;
  for (int trial = 0; trial < kTrials; ++trial) {
    const Isometry3d target = Tip(RandomAngles(arm_, rng));
    IkReport report;
    const std::optional<VectorXd> q = Solve(target, Home(), &report);
    if (q) {
      ++solved;
      ExpectReaches(*q, target);
      total_iterations += report.iterations;
      most_iterations = std::max(most_iterations, report.iterations);
    } else {
      EXPECT_TRUE(report.angles.allFinite());
      EXPECT_TRUE(WithinRanges(arm_, report.angles));
      stalled += report.status == IkStatus::kStalled;
      limit += report.status == IkStatus::kIterationLimit;
    }
  }
  const double rate = static_cast<double>(solved) / kTrials;
  std::printf(
      "from home: %d/%d solved (%.1f%%), iterations mean %.1f max %d over "
      "those solved; failures: %d stalled, %d at the iteration limit\n",
      solved, kTrials, 100.0 * rate,
      solved ? static_cast<double>(total_iterations) / solved : 0.0,
      most_iterations, stalled, limit);
  EXPECT_GE(rate, 0.85);
}

// The same tool pose with the wrist the other way round: joint 4 half a
// turn further, joint 5 bent the other way, joint 6 half a turn further.
// The half turns are taken in whichever direction keeps the joint in range
// and nearer zero. Every pose away from joint 5 = 0 has these two wrists.
VectorXd FlippedWrist(const Arm& arm, const VectorXd& q) {
  VectorXd f = q;
  f[4] = -q[4];
  for (int i : {3, 5}) {
    f[i] = q[i] + (q[i] > 0.0 ? -kPi : kPi);
    if (f[i] < arm.lower[i] || f[i] > arm.upper[i]) f[i] = q[i] - (f[i] - q[i]);
  }
  return f;
}

// The poses the pick and place needs, with the real arm and tool, for both
// choices of the tool's X axis (joint 6 half a turn apart). Each is solved
// from home with joint 1 turned to face the target, which is how a planner
// would seed it; then 0.3 m back from the face along its normal, seeded
// from the face's answer. Prints the angles and the margin to the limits,
// for the answer found and for the same pose with the wrist flipped.
TEST_F(IkTest, CellPoses) {
  struct Case {
    const char* name;
    Isometry3d target;
    bool face;
  };
  const std::vector<Case> cases = {
      {"near face, X -Y", FacePose(kNearFace, +1), true},
      {"near face, X +Y", FacePose(kNearFace, -1), true},
      {"far face,  X -Y", FacePose(kFarFace, +1), true},
      {"far face,  X +Y", FacePose(kFarFace, -1), true},
      {"place,     X +X", PlacePose(+1), false},
      {"place,     X -X", PlacePose(-1), false},
  };

  // The IK knows nothing of collisions; this says which geoms of the arm
  // and tool would touch at `q`, using the collision hulls (a little fatter
  // than the real links) and the floor. Nothing else is in this scene.
  const auto contacts = [&](const VectorXd& q) {
    Tip(q);
    mj_forward(model_.get(), data_.get());
    std::string list;
    for (int i = 0; i < data_->ncon; ++i) {
      const mjContact& c = data_->contact[i];
      list += std::string(i ? ", " : "") +
              mj_id2name(model_.get(), mjOBJ_GEOM, c.geom[0]) + "/" +
              mj_id2name(model_.get(), mjOBJ_GEOM, c.geom[1]);
    }
    return list.empty() ? std::string("none") : list;
  };

  const auto print = [&](const char* label, const VectorXd& q) {
    const VectorXd margin = LimitMargins(arm_, q);
    std::printf("    %-22s angles %s  contacts: %s\n", label,
                Degrees(q).c_str(), contacts(q).c_str());
    std::printf("    %-22s margin %s  smallest %.1f (joint %d)\n", "",
                Degrees(margin).c_str(), margin.minCoeff() / kDegree,
                static_cast<int>(std::min_element(margin.data(),
                                                  margin.data() + 6) -
                                 margin.data()) +
                    1);
  };
  // The flipped wrist must put the tip exactly where the answer did.
  const auto print_flipped = [&](const VectorXd& q) {
    const VectorXd f = FlippedWrist(arm_, q);
    const Isometry3d tip = Tip(q);
    const Isometry3d flipped = Tip(f);
    EXPECT_LE(PositionError(flipped, tip), 1e-9);
    EXPECT_LE(AngleError(flipped, tip), 1e-9);
    if (WithinRanges(arm_, f)) {
      print("  wrist flipped", f);
    } else {
      std::printf("    %-22s out of range: %s\n", "  wrist flipped",
                  Degrees(f).c_str());
    }
  };

  std::printf("cell poses; angles and margins in degrees, joints 1 to 6\n");
  for (const Case& c : cases) {
    std::printf("  %s\n", c.name);
    VectorXd seed = Home();
    // Joint 1 is positive clockwise seen from above.
    seed[0] = -std::atan2(c.target.translation().y(),
                          c.target.translation().x());
    seed[0] = std::clamp(seed[0], arm_.lower[0], arm_.upper[0]);

    IkReport from_home;
    Solve(c.target, Home(), &from_home);
    std::printf("    from home itself: %s after %d iterations\n",
                StatusName(from_home.status), from_home.iterations);

    IkReport report;
    const std::optional<VectorXd> q = Solve(c.target, seed, &report);
    if (!q) {
      std::printf("    NOT FOUND from home turned to %.1f deg (%s, %.1f mm, "
                  "%.2f deg off, closest %s)\n",
                  seed[0] / kDegree, StatusName(report.status),
                  report.position_error / kMillimetre,
                  report.orientation_error / kDegree,
                  Degrees(report.angles).c_str());
      ADD_FAILURE() << c.name << " not reached";
      continue;
    }
    ExpectReaches(*q, c.target);
    print("pose", *q);
    print_flipped(*q);
    if (!c.face) continue;

    Isometry3d back = c.target;
    back.translation() += 0.3 * kGlassNormal;
    IkReport back_report;
    const std::optional<VectorXd> b = Solve(back, *q, &back_report);
    if (!b) {
      std::printf("    0.3 m back: NOT FOUND (%s, %.1f mm, %.2f deg off)\n",
                  StatusName(back_report.status),
                  back_report.position_error / kMillimetre,
                  back_report.orientation_error / kDegree);
      ADD_FAILURE() << c.name << ": 0.3 m back not reached";
      continue;
    }
    ExpectReaches(*b, back);
    print("0.3 m back", *b);
    print_flipped(*b);
  }
}

// Targets no joint angles reach return nothing, from several seeds, and the
// report's last guess is finite and within the ranges.
TEST_F(IkTest, UnreachableTargetsReturnNothing) {
  struct Case {
    const char* name;
    Isometry3d target;
  };
  const std::vector<Case> cases = {
      // The wrist centre reaches about 2.7 m from the base axis, the tool
      // tip 0.5 m more.
      {"4 m out", PoseFrom(Vector3d(4.0, 0, 1.0), Vector3d(1, 0, 0),
                           Vector3d(0, 1, 0))},
      {"far overhead", PoseFrom(Vector3d(0.5, 0, 5.0), Vector3d(0, 0, 1),
                                Vector3d(1, 0, 0))},
      // In the base casting, 0.3 m above the floor on joint 1's axis, tool
      // pointing down. The wrist centre would be on the axis 0.515 m
      // higher: 0.33 m behind joint 2 and 0.17 m above it, 0.372 m away.
      // Folding the forearm back that close takes joint 3 to about 162
      // degrees (in range), but that fold leaves the wrist centre about 93
      // degrees round from the upper arm, so the upper arm would have to
      // point down and back, below level, where joint 2 cannot put it;
      // folding the other way, joint 3 stops at 120 degrees. The solver
      // knows nothing of collisions: this fails on geometry alone.
      {"inside the base", PoseFrom(Vector3d(0, 0, 0.3), Vector3d(0, 0, -1),
                                   Vector3d(1, 0, 0))},
      // 1.5 m below the floor under the base. Joint 2 cannot lower the
      // upper arm below level, so the tip gets nowhere near.
      {"under the base", PoseFrom(Vector3d(0, 0, -1.5), Vector3d(0, 0, -1),
                                  Vector3d(1, 0, 0))},
  };
  std::mt19937 rng(3);
  std::vector<VectorXd> seeds = {Home()};
  for (int i = 0; i < 20; ++i) seeds.push_back(RandomAngles(arm_, rng));
  for (const Case& c : cases) {
    for (const VectorXd& seed : seeds) {
      IkReport report;
      EXPECT_FALSE(Solve(c.target, seed, &report)) << c.name;
      EXPECT_NE(report.status, IkStatus::kConverged);
      EXPECT_TRUE(report.angles.allFinite()) << c.name;
      EXPECT_TRUE(WithinRanges(arm_, report.angles)) << c.name;
      EXPECT_TRUE(std::isfinite(report.position_error));
      EXPECT_GT(report.position_error, 0.05) << c.name;
    }
    IkReport report;
    Solve(c.target, Home(), &report);
    std::printf("unreachable %s: %s after %d iterations, closest %.3f m and "
                "%.2f deg off\n",
                c.name, StatusName(report.status), report.iterations,
                report.position_error, report.orientation_error / kDegree);
  }
}

// At a singularity the Jacobian loses a direction. The solver must still
// return either a right answer near the seed, or nothing, with finite angles
// in range either way: never NaN, never a wild swing.
TEST_F(IkTest, SingularPosesGiveFiniteAnswersOrCleanFailures) {
  std::mt19937 rng(13);

  // Arm stretched straight.
  VectorXd stretched(Arm::kJoints);
  stretched << 0.3, -0.6, StretchedJoint3(), 0.4, 0.8, 0.2;
  std::printf("stretched: joint 3 at %.3f deg, wrist centre %.4f m from "
              "joint 2\n",
              stretched[2] / kDegree,
              (WristCentre(stretched) - Joint2(stretched)).norm());

  // Wrist singular: joint 5 at zero, joints 4 and 6 on one axis.
  VectorXd wrist(Arm::kJoints);
  wrist << -0.4, -1.2, 1.0, 0.7, 0.0, -0.5;

  const std::vector<std::pair<const char*, VectorXd>> singular_poses = {
      {"stretched", stretched}, {"wrist", wrist}};
  for (const auto& [name, singular] : singular_poses) {
    const Isometry3d target = Tip(singular);
    int solved = 0;
    double worst_move = 0.0;
    for (int i = 0; i < 50; ++i) {
      const VectorXd seed =
          i == 0 ? singular : Perturbed(singular, 0.05, rng);
      IkReport report;
      const std::optional<VectorXd> q = Solve(target, seed, &report);
      EXPECT_TRUE(report.angles.allFinite());
      EXPECT_TRUE(WithinRanges(arm_, report.angles));
      worst_move = std::max(worst_move, report.distance_from_seed);
      if (q) {
        ++solved;
        ExpectReaches(*q, target);
      }
      // "Wild" would be a joint swinging far from a seed 0.05 rad off.
      EXPECT_LT(report.distance_from_seed, 0.5);
    }
    std::printf("singular %s: solved from %d of 50 nearby seeds, largest "
                "joint move from the seed %.3f rad\n",
                name, solved, worst_move);
  }

  // Seeded exactly at the wrist singularity, to a target that needs the
  // wrist to bend and roll: the first Jacobian is rank 5.
  {
    VectorXd goal = wrist;
    goal[3] += 0.3;
    goal[4] = 0.2;
    const Isometry3d target = Tip(goal);
    IkReport report;
    const std::optional<VectorXd> q = Solve(target, wrist, &report);
    ASSERT_TRUE(q) << StatusName(report.status);
    ExpectReaches(*q, target);
  }

  // Just beyond the stretched reach, along the stretched arm: no answer,
  // and the last guess still finite and in range.
  {
    Isometry3d target = Tip(stretched);
    const Vector3d out =
        (WristCentre(stretched) - Joint2(stretched)).normalized();
    target.translation() += 0.01 * out;
    IkReport report;
    EXPECT_FALSE(Solve(target, stretched, &report));
    EXPECT_TRUE(report.angles.allFinite());
    EXPECT_TRUE(WithinRanges(arm_, report.angles));
    EXPECT_LT(report.distance_from_seed, 0.5);
    std::printf("1 cm beyond stretched: %s, %.2f mm short, joints moved at "
                "most %.3f rad\n",
                StatusName(report.status), report.position_error / kMillimetre,
                report.distance_from_seed);
  }
}

// Answers stay within the ranges even when the seed is outside them and the
// target is right at a limit.
TEST_F(IkTest, JointRangesAreRespected) {
  std::mt19937 rng(17);
  for (int trial = 0; trial < 200; ++trial) {
    VectorXd truth = RandomAngles(arm_, rng);
    // Put one joint exactly on a limit, and the seed beyond it.
    const int joint = trial % Arm::kJoints;
    const bool upper = (trial / Arm::kJoints) % 2 == 0;
    truth[joint] = upper ? arm_.upper[joint] : arm_.lower[joint];
    VectorXd seed = Perturbed(truth, 0.05, rng);
    seed[joint] += upper ? 0.2 : -0.2;

    const Isometry3d target = Tip(truth);
    IkReport report;
    const std::optional<VectorXd> q = Solve(target, seed, &report);
    EXPECT_TRUE(WithinRanges(arm_, report.angles)) << Degrees(report.angles);
    if (q) ExpectReaches(*q, target);
  }
}

// Straight back from each face along its normal, 0.3 m in 1 cm steps: the
// lift-off and pull-away. Neighbouring answers must differ by little, and
// the change from step to step must itself change little (no sudden swing
// of joint 4 or 6, which is what passing near the wrist singularity looks
// like). The worst seen are 0.018 rad per step and 0.0004 rad change
// between steps, at the far face, where the arm is stretched out and joints
// 2 and 3 do most of the work. The bounds leave room for a layout change:
// with the pallet 0.45 m nearer the arm the near face needed 0.021 and
// 0.0006.
TEST_F(IkTest, StraightLinePathIsSmooth) {
  const std::vector<std::pair<Vector3d, bool>> cases = {
      {kNearFace, false}, {kNearFace, true}, {kFarFace, false}, {kFarFace, true}};
  for (const auto& [centre, flip] : cases) {
    const Isometry3d face = FacePose(centre, +1);
    std::optional<VectorXd> start = Solve(face, Home());
    ASSERT_TRUE(start);
    // From home the solver keeps joint 5 positive, which here needs joint 4
    // at half a turn; the other wrist has joint 4 at zero. Both are tried.
    if (flip) start = FlippedWrist(arm_, *start);

    Isometry3d back = face;
    back.translation() += 0.3 * kGlassNormal;
    const std::vector<Isometry3d> poses = cell::StraightLine(face, back, 0.01);
    ASSERT_EQ(poses.size(), 31u);
    IkPathReport report;
    const auto path = SolveIkPath(*model_, data_.get(), arm_, *start, poses,
                                  0.05, {}, &report);
    ASSERT_TRUE(path) << "waypoint " << report.failed_waypoint << ": "
                      << StatusName(report.waypoint.status);
    ASSERT_EQ(path->size(), poses.size());

    double largest = 0.0;
    double largest_change = 0.0;
    // Between waypoints the arm moves in joint space; halfway between two,
    // how far is the tip from the straight line?
    double largest_stray = 0.0;
    for (size_t k = 0; k < path->size(); ++k) {
      ExpectReaches((*path)[k], poses[k]);
      if (k == 0) continue;
      const Vector3d halfway =
          Tip(0.5 * ((*path)[k] + (*path)[k - 1])).translation() -
          face.translation();
      largest_stray = std::max(
          largest_stray,
          (halfway - halfway.dot(kGlassNormal) * kGlassNormal).norm());
      const VectorXd step = (*path)[k] - (*path)[k - 1];
      largest = std::max(largest, step.cwiseAbs().maxCoeff());
      if (k >= 2) {
        const VectorXd before = (*path)[k - 1] - (*path)[k - 2];
        largest_change =
            std::max(largest_change, (step - before).cwiseAbs().maxCoeff());
      }
    }
    std::printf("straight back from (%.2f, %.2f, %.2f)%s: largest joint step "
                "%.4f rad (%.3f deg) per cm, largest change between steps "
                "%.5f rad, tip strays %.4f mm off the line between "
                "waypoints; start %s, end %s\n",
                centre.x(), centre.y(), centre.z(),
                flip ? ", wrist flipped" : "", largest, largest / kDegree,
                largest_change, largest_stray / kMillimetre,
                Degrees(path->front()).c_str(), Degrees(path->back()).c_str());
    EXPECT_LT(largest, 0.03);
    EXPECT_LT(largest_change, 0.002);
    EXPECT_LT(largest_stray, 0.2 * kMillimetre);
    EXPECT_DOUBLE_EQ(report.largest_step, largest);
  }
}

// A path fails, and says where, when a waypoint is out of reach or the
// joints would have to jump between neighbours.
TEST_F(IkTest, PathReportsTheFailingWaypoint) {
  const Isometry3d face = FacePose(kFarFace, +1);
  const std::optional<VectorXd> start = Solve(face, Home());
  ASSERT_TRUE(start);
  std::vector<Isometry3d> poses;
  for (int k = 0; k < 5; ++k) {
    Isometry3d pose = face;
    pose.translation() += 0.01 * k * kGlassNormal;
    poses.push_back(pose);
  }

  // A tiny allowed step: the second waypoint already needs more.
  IkPathReport report;
  EXPECT_FALSE(SolveIkPath(*model_, data_.get(), arm_, *start, poses, 1e-4,
                           {}, &report));
  EXPECT_EQ(report.failed_waypoint, 1);
  EXPECT_EQ(report.waypoint.status, IkStatus::kTooFarFromSeed);

  // An unreachable waypoint in the middle.
  poses[3].translation() = Vector3d(4.0, 0, 1.0);
  EXPECT_FALSE(SolveIkPath(*model_, data_.get(), arm_, *start, poses, 0.05,
                           {}, &report));
  EXPECT_EQ(report.failed_waypoint, 3);
}

// Every module the arm will pick, wherever the pallet is set down within its
// tolerance: grip, lift 0.03 m up the glass, pull straight back 0.4 m along
// the glass normal. Checked for the front, middle and back modules, at the
// nominal pallet pose and at the eight extremes (x, y and yaw each at +-).
// Each must solve as a straight-line path on the wrist with joint 4 near
// zero, without joint 5 changing sign (which would be a wrist flip on the
// way), and with at least 15 degrees to every joint limit throughout.
//
// The 15 degrees is the margin the layout was chosen for: with the pallet
// at 1.95 m, the tightest are joint 3 folding at the end of module_0's
// pull-back and joint 2 near its -5 degree end when reaching module_29.
TEST_F(IkTest, EveryModuleCanBePickedFromThePallet) {
  constexpr double kRequiredMargin = 15.0 * kDegree;
  constexpr double kLift = 0.03;
  constexpr double kPullBack = 0.4;
  constexpr double kSpacing = 0.01;

  struct Placement {
    double dx, dy, yaw;
  };
  std::vector<Placement> placements = {{0.0, 0.0, 0.0}};
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 1.0}) {
      for (double sw : {-1.0, 1.0}) {
        placements.push_back(
            {sx * kPalletOffset, sy * kPalletOffset, sw * kPalletYaw});
      }
    }
  }

  double smallest = 1e9;
  std::string smallest_where;
  for (int module : {0, 15, kModuleCount - 1}) {
    double module_smallest = 1e9;
    std::string module_where;
    for (const Placement& p : placements) {
      char name[96];
      std::snprintf(name, sizeof(name),
                    "module_%d, pallet at x %+.2f y %+.2f yaw %+.0f deg",
                    module, p.dx, p.dy, p.yaw / kDegree);
      const Glass glass = ModuleGlass(module, p.dx, p.dy, p.yaw);
      const std::optional<Isometry3d> grip =
          cell::GripPose(glass.centre, glass.normal, glass.long_side);
      ASSERT_TRUE(grip) << name;

      // Seed: home, turned to face the module.
      VectorXd seed = Home();
      seed[0] = -std::atan2(glass.centre.y(), glass.centre.x());
      std::optional<VectorXd> q = Solve(*grip, seed);
      ASSERT_TRUE(q) << name << ": grip not reached";
      // Onto the wrist with joint 4 near zero.
      if (std::abs((*q)[3]) > kHalfPi) q = FlippedWrist(arm_, *q);
      ASSERT_TRUE(WithinRanges(arm_, *q)) << name;
      ASSERT_LT(std::abs((*q)[3]), kHalfPi) << name;

      Isometry3d lifted = *grip;
      lifted.translation() += kLift * glass.up;
      Isometry3d back = lifted;
      back.translation() += kPullBack * glass.normal;
      std::vector<Isometry3d> poses = cell::StraightLine(*grip, lifted, kSpacing);
      const std::vector<Isometry3d> pull =
          cell::StraightLine(lifted, back, kSpacing);
      poses.insert(poses.end(), pull.begin() + 1, pull.end());
      const int lift_end = static_cast<int>(poses.size() - pull.size());

      IkPathReport report;
      const auto path =
          SolveIkPath(*model_, data_.get(), arm_, *q, poses, 0.05, {}, &report);
      ASSERT_TRUE(path) << name << ": waypoint " << report.failed_waypoint
                        << " of " << poses.size() << ", "
                        << StatusName(report.waypoint.status);

      for (int k = 0; k < static_cast<int>(path->size()); ++k) {
        const VectorXd& a = (*path)[k];
        ExpectReaches(a, poses[k]);
        EXPECT_LT(std::abs(a[3]), kHalfPi) << name << ", waypoint " << k;
        EXPECT_GT(a[4] * (*q)[4], 0.0)
            << name << ": joint 5 changed sign at waypoint " << k;
        const VectorXd margin = LimitMargins(arm_, a);
        int joint = 0;
        const double m = margin.minCoeff(&joint);
        EXPECT_GE(m, kRequiredMargin)
            << name << ", waypoint " << k << ", joint " << joint + 1;
        if (m < module_smallest) {
          module_smallest = m;
          char where[160];
          std::snprintf(
              where, sizeof(where), "joint %d, %s, %s", joint + 1, name,
              k == 0 ? "at the grip"
              : k <= lift_end
                  ? "in the lift"
                  : (std::to_string(static_cast<int>(
                         std::lround((k - lift_end) * kSpacing * 100))) +
                     " cm back")
                        .c_str());
          module_where = where;
        }
      }
    }
    std::printf("pick: module_%d smallest margin %.1f deg (%s)\n", module,
                module_smallest / kDegree, module_where.c_str());
    if (module_smallest < smallest) {
      smallest = module_smallest;
      smallest_where = module_where;
    }
  }
  std::printf("pick: smallest margin over all %.1f deg (%s)\n",
              smallest / kDegree, smallest_where.c_str());
}

// An answer far from the seed is refused when asked, and reported.
TEST_F(IkTest, FarFromSeedIsRefusedWhenAsked) {
  const Isometry3d place = PlacePose(+1);
  IkOptions options;
  options.max_distance_from_seed = 0.5;
  IkReport report;
  EXPECT_FALSE(Solve(place, Home(), &report, options));
  EXPECT_EQ(report.status, IkStatus::kTooFarFromSeed);
  EXPECT_GT(report.distance_from_seed, 0.5);
  EXPECT_LE(report.position_error, kDefaults.position_tolerance);
}

TEST_F(IkTest, BadInputIsRefused) {
  const Isometry3d target = Tip(Home());
  IkReport report;
  EXPECT_FALSE(Solve(target, VectorXd::Zero(5), &report));
  EXPECT_EQ(report.status, IkStatus::kBadInput);

  VectorXd nan_seed = Home();
  nan_seed[2] = std::nan("");
  EXPECT_FALSE(Solve(target, nan_seed, &report));
  EXPECT_EQ(report.status, IkStatus::kBadInput);

  Isometry3d nan_target = target;
  nan_target.translation().x() = std::nan("");
  EXPECT_FALSE(Solve(nan_target, Home(), &report));
  EXPECT_EQ(report.status, IkStatus::kBadInput);

  IkOptions options;
  options.damping = 0.0;
  EXPECT_FALSE(Solve(target, Home(), &report, options));
  EXPECT_EQ(report.status, IkStatus::kBadInput);
}

// The same scene with a free-floating box defined before the arm, so the
// box has qpos 0 to 6 and the arm's joints come after.
class IkSceneTest : public IkTest {
 protected:
  void SetUp() override { Load("IK_SCENE_PATH"); }
};

TEST_F(IkSceneTest, ArmJointsAreNotFirst) {
  EXPECT_EQ(model_->nq, 7 + Arm::kJoints);
  EXPECT_EQ(model_->nv, 6 + Arm::kJoints);
  EXPECT_EQ(arm_.qpos[0], 7);
  EXPECT_EQ(arm_.dof[0], 6);
}

TEST_F(IkSceneTest, RoundTrips) {
  std::mt19937 rng(19);
  for (int trial = 0; trial < 200; ++trial) {
    const VectorXd truth = RandomAngles(arm_, rng);
    const Isometry3d target = Tip(truth);
    const std::optional<VectorXd> q = Solve(target, Perturbed(truth, 0.1, rng));
    ASSERT_TRUE(q) << "trial " << trial;
    ExpectReaches(*q, target);
  }
}

// The solver writes only the arm's six qpos entries. Everything else the
// scratch data holds that a simulation would care about (the box's pose,
// every speed, every control) is exactly as it was.
TEST_F(IkSceneTest, LeavesTheRestOfScratchAlone) {
  std::mt19937 rng(23);
  std::uniform_real_distribution<double> any(-1.0, 1.0);
  for (int i = 0; i < model_->nq; ++i) data_->qpos[i] = any(rng);
  for (int i = 0; i < model_->nv; ++i) data_->qvel[i] = any(rng);
  for (int i = 0; i < model_->nu; ++i) data_->ctrl[i] = any(rng);
  for (int i = 0; i < model_->na; ++i) data_->act[i] = any(rng);
  const std::vector<mjtNum> qpos(data_->qpos, data_->qpos + model_->nq);
  const std::vector<mjtNum> qvel(data_->qvel, data_->qvel + model_->nv);
  const std::vector<mjtNum> ctrl(data_->ctrl, data_->ctrl + model_->nu);
  const std::vector<mjtNum> act(data_->act, data_->act + model_->na);
  const mjtNum time = data_->time;

  for (int trial = 0; trial < 20; ++trial) {
    const Isometry3d target = Tip(RandomAngles(arm_, rng));
    Solve(target, Home());
  }

  std::vector<bool> is_arm(model_->nq, false);
  for (int i = 0; i < Arm::kJoints; ++i) is_arm[arm_.qpos[i]] = true;
  for (int i = 0; i < model_->nq; ++i) {
    if (!is_arm[i]) EXPECT_EQ(data_->qpos[i], qpos[i]) << "qpos " << i;
  }
  for (int i = 0; i < model_->nv; ++i) EXPECT_EQ(data_->qvel[i], qvel[i]);
  for (int i = 0; i < model_->nu; ++i) EXPECT_EQ(data_->ctrl[i], ctrl[i]);
  for (int i = 0; i < model_->na; ++i) EXPECT_EQ(data_->act[i], act[i]);
  EXPECT_EQ(data_->time, time);
}

// The same question gives the same answer, bit for bit: again on the same
// scratch data after other solves, on fresh scratch data, and in the scene
// without the box.
TEST_F(IkSceneTest, IsDeterministic) {
  std::mt19937 rng(29);
  const VectorXd truth = RandomAngles(arm_, rng);
  const Isometry3d target = Tip(truth);
  const VectorXd seed = Home();

  const std::optional<VectorXd> first = Solve(target, seed);
  ASSERT_TRUE(first);
  for (int i = 0; i < 5; ++i) Solve(Tip(RandomAngles(arm_, rng)), Home());
  const std::optional<VectorXd> again = Solve(target, seed);
  ASSERT_TRUE(again);
  EXPECT_EQ(*first, *again);

  cell::DataPtr fresh = cell::MakeData(*model_);
  const std::optional<VectorXd> on_fresh =
      SolveIk(*model_, fresh.get(), arm_, target, seed);
  ASSERT_TRUE(on_fresh);
  EXPECT_EQ(*first, *on_fresh);

  // The arm alone: the same arm at the same place, so the same answer. Not
  // necessarily to the last bit, since MuJoCo's sums run over a different
  // set of bodies.
  const char* path = std::getenv("MODEL_PATH");
  ASSERT_NE(path, nullptr);
  cell::ModelPtr alone = cell::LoadModel(path);
  ASSERT_TRUE(alone);
  cell::DataPtr alone_data = cell::MakeData(*alone);
  const std::optional<Arm> alone_arm = cell::FindArm(*alone);
  ASSERT_TRUE(alone_arm);
  const std::optional<VectorXd> in_alone =
      SolveIk(*alone, alone_data.get(), *alone_arm, target, seed);
  ASSERT_TRUE(in_alone);
  EXPECT_LT((*first - *in_alone).cwiseAbs().maxCoeff(), 1e-9);
}

// How long a solve takes. Only printed; time it in an optimized build:
//   bazel test --config=opt //:ik_test --test_output=all
TEST_F(IkTest, TimePerSolve) {
  std::mt19937 rng(31);
  constexpr int kTrials = 500;
  std::vector<Isometry3d> targets;
  std::vector<VectorXd> near_seeds;
  for (int i = 0; i < kTrials; ++i) {
    const VectorXd truth = RandomAngles(arm_, rng);
    targets.push_back(Tip(truth));
    near_seeds.push_back(Perturbed(truth, 0.1, rng));
  }
  using Clock = std::chrono::steady_clock;
  int iterations = 0;
  auto t0 = Clock::now();
  for (int i = 0; i < kTrials; ++i) {
    IkReport report;
    Solve(targets[i], near_seeds[i], &report);
    iterations += report.iterations;
  }
  auto t1 = Clock::now();
  const double near_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count() / kTrials;
  const double per_iteration_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count() / iterations;
  t0 = Clock::now();
  for (int i = 0; i < kTrials; ++i) Solve(targets[i], Home());
  t1 = Clock::now();
  const double far_us =
      std::chrono::duration<double, std::micro>(t1 - t0).count() / kTrials;
  std::printf("time per solve: %.1f us from a seed 0.1 rad off, %.1f us from "
              "home (successes and failures); %.2f us per iteration\n",
              near_us, far_us, per_iteration_us);
}

// GripPose: tip on the face centre, +Z into the face, X along the long
// side, a right-handed frame; a long side that is not square to the normal
// is squared; inputs that give no frame give nothing.
TEST(GripPoseTest, BuildsTheToolFrame) {
  const Vector3d centre(1.0, 2.0, 0.5);
  const Vector3d normal = 2.0 * kGlassNormal;  // length does not matter
  const std::optional<Isometry3d> grip =
      cell::GripPose(centre, normal, kModuleLongSide);
  ASSERT_TRUE(grip);
  EXPECT_TRUE(grip->translation().isApprox(centre));
  EXPECT_TRUE(grip->linear().col(2).isApprox(-kGlassNormal));
  EXPECT_TRUE(grip->linear().col(0).isApprox(kModuleLongSide));
  EXPECT_TRUE((grip->linear().transpose() * grip->linear())
                  .isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_NEAR(grip->linear().determinant(), 1.0, 1e-12);

  // A long side tilted out of the face is squared into it.
  const std::optional<Isometry3d> tilted =
      cell::GripPose(centre, kGlassNormal, kModuleLongSide + 0.3 * kGlassNormal);
  ASSERT_TRUE(tilted);
  EXPECT_TRUE(tilted->linear().isApprox(grip->linear()));

  // The other sign: the same Z, X and Y reversed (half a turn about Z).
  const std::optional<Isometry3d> other =
      cell::GripPose(centre, kGlassNormal, -kModuleLongSide);
  ASSERT_TRUE(other);
  EXPECT_TRUE(other->linear().col(2).isApprox(grip->linear().col(2)));
  EXPECT_TRUE(other->linear().col(0).isApprox(-grip->linear().col(0)));
  EXPECT_TRUE(other->linear().col(1).isApprox(-grip->linear().col(1)));

  // The place pose: glass facing up means the tool points down.
  const Isometry3d place = PlacePose(+1);
  EXPECT_TRUE(place.linear().col(2).isApprox(-Vector3d::UnitZ()));
  EXPECT_TRUE(place.linear().col(0).isApprox(Vector3d::UnitX()));

  EXPECT_FALSE(cell::GripPose(centre, Vector3d::Zero(), kModuleLongSide));
  EXPECT_FALSE(cell::GripPose(centre, kGlassNormal, kGlassNormal));
  EXPECT_FALSE(cell::GripPose(centre, kGlassNormal, Vector3d::Zero()));
  EXPECT_FALSE(cell::GripPose(Vector3d(std::nan(""), 0, 0), kGlassNormal,
                              kModuleLongSide));
}

// StraightLine: both ends included exactly, evenly spaced within both
// limits, orientation turned about one axis at a steady rate.
TEST(StraightLineTest, EvenlySpacedPoses) {
  const Isometry3d from = *cell::GripPose(Vector3d(1, 0, 1), Vector3d::UnitX(),
                                          Vector3d::UnitY());
  Isometry3d to = Isometry3d::Identity();
  to.translation() = Vector3d(1.3, 0.1, 0.9);
  to.linear() = from.linear() *
                Eigen::AngleAxisd(0.5, Vector3d(1, 1, 0).normalized())
                    .toRotationMatrix();

  const std::vector<Isometry3d> line = cell::StraightLine(from, to, 0.01);
  const double distance = (to.translation() - from.translation()).norm();
  // 0.332 m in pieces of at most 1 cm needs 34; 0.5 rad in pieces of at
  // most 1 degree needs 29. The distance sets the count.
  const int pieces = static_cast<int>(line.size()) - 1;
  EXPECT_EQ(pieces, static_cast<int>(std::ceil(distance / 0.01)));
  EXPECT_TRUE(line.front().isApprox(from));
  EXPECT_TRUE(line.back().isApprox(to));
  for (int k = 1; k <= pieces; ++k) {
    const Isometry3d& a = line[k - 1];
    const Isometry3d& b = line[k];
    EXPECT_NEAR(PositionError(a, b), distance / pieces, 1e-12);
    EXPECT_NEAR(AngleError(a, b), 0.5 / pieces, 1e-9);
    // On the line.
    const Vector3d along = (to.translation() - from.translation()).normalized();
    const Vector3d off = b.translation() - from.translation();
    EXPECT_LT((off - off.dot(along) * along).norm(), 1e-12);
  }

  // A pure turn is split by angle; a coarse spacing still gives both ends.
  Isometry3d turned = from;
  turned.linear() =
      from.linear() * Eigen::AngleAxisd(0.1, Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_EQ(cell::StraightLine(from, turned, 0.01).size(),
            static_cast<size_t>(std::ceil(0.1 / kDegree)) + 1);
  EXPECT_EQ(cell::StraightLine(from, to, 10.0, 10.0).size(), 2u);
  EXPECT_EQ(cell::StraightLine(from, from, 0.01).size(), 2u);
  // An exact multiple does not gain a piece through rounding.
  Isometry3d back = from;
  back.translation() += 0.3 * kGlassNormal;
  EXPECT_EQ(cell::StraightLine(from, back, 0.01).size(), 31u);

  EXPECT_TRUE(cell::StraightLine(from, to, 0.0).empty());
  EXPECT_TRUE(cell::StraightLine(from, to, 0.01, -1.0).empty());
}

}  // namespace
