// Checks the whole cell: the parts come together without clashing, the
// standing stack stays put while nothing touches it with everything present,
// a trial's scene is the same for the same seed, the depth camera sees the
// whole front glass, nothing touches the arm, and the scorer, parking and
// swapping the tube do what src/cell.h and src/score.h say.
//
//   bazel test //:cell_test
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/cell.xml).
//
// Simulated time is what this test spends: the 30-module stack runs at about
// twice real time unoptimized. Each Reset costs Cell::kSettleTime (1 s), and
// the tests use about 30 s in all.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
#include "src/cell.h"
#include "src/conveyor.h"
#include "src/pallet.h"
#include "src/score.h"

namespace {

using cell::Cell;
using cell::ModuleState;

constexpr double kPi = 3.14159265358979323846;

// The stack's stillness, as tests/pallet_test.cc demands of the stack alone.
// No module moves further than this from where it settled, at any instant...
constexpr double kStillDistance = 0.1e-3;  // m
constexpr double kStillAngle = 0.1e-3;     // rad
// ...neither half of the watch moves it further than this...
constexpr double kDriftPerHalf = 0.02e-3;  // m
// ...and at the end it is hardly moving.
constexpr double kEndSpeed = 0.01e-3;  // m/s
constexpr double kEndSpin = 0.1e-3;    // rad/s

// The arm holds its pose within this, every joint, at every instant.
constexpr double kArmHold = 1e-3;  // rad

// Body names, for messages.
std::string BodyName(const mjModel& model, int body) {
  const char* name = mj_id2name(&model, mjOBJ_BODY, body);
  return name != nullptr ? name : "body " + std::to_string(body);
}

bool IsModule(const mjModel& model, int body) {
  return BodyName(model, body).rfind("module_", 0) == 0;
}

class CellTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";
    std::string error;
    std::optional<Cell> loaded = Cell::Load(path, &error);
    ASSERT_TRUE(loaded) << error;
    cell_.emplace(std::move(*loaded));
  }

  Cell& cell() { return *cell_; }
  const mjModel& model() { return *cell_->model(); }
  mjData* data() { return cell_->data(); }

  // Every contact as "body - body", for messages.
  std::vector<std::pair<int, int>> ContactBodies() {
    std::vector<std::pair<int, int>> pairs;
    for (int c = 0; c < data()->ncon; ++c) {
      pairs.push_back({model().geom_bodyid[data()->contact[c].geom[0]],
                       model().geom_bodyid[data()->contact[c].geom[1]]});
    }
    return pairs;
  }

  // Fails for every contact other than module on module or module on the
  // pallet. Returns how many contacts there are.
  int ExpectOnlyStackContacts() {
    mj_forward(cell().model(), data());
    for (const auto& [a, b] : ContactBodies()) {
      const bool stack =
          (IsModule(model(), a) || a == cell().pallet().pallet) &&
          (IsModule(model(), b) || b == cell().pallet().pallet) &&
          (IsModule(model(), a) || IsModule(model(), b));
      EXPECT_TRUE(stack) << BodyName(model(), a) << " touches "
                         << BodyName(model(), b);
    }
    return data()->ncon;
  }

  // The arm's bodies: everything under arm_base, the tool included.
  std::vector<int> ArmBodies() {
    std::vector<int> bodies;
    const int base = mj_name2id(&model(), mjOBJ_BODY, "arm_base");
    for (int b = 0; b < model().nbody; ++b) {
      for (int up = b; up > 0; up = model().body_parentid[up]) {
        if (up == base) {
          bodies.push_back(b);
          break;
        }
      }
    }
    return bodies;
  }

  // The smallest distance from any of the arm's colliding geoms to any
  // colliding geom that is not the arm's, up to 1 m. Pairs MuJoCo never
  // collides are left out: two geoms both fixed to the world, such as the
  // base standing on the floor.
  double ArmClearance(std::string* between = nullptr) {
    const std::vector<int> arm = ArmBodies();
    auto in_arm = [&](int geom) {
      return std::find(arm.begin(), arm.end(), model().geom_bodyid[geom]) !=
             arm.end();
    };
    auto collides = [&](int geom) {
      return model().geom_contype[geom] != 0 ||
             model().geom_conaffinity[geom] != 0;
    };
    double nearest = 1.0;
    for (int g = 0; g < model().ngeom; ++g) {
      if (!in_arm(g) || !collides(g)) continue;
      for (int h = 0; h < model().ngeom; ++h) {
        if (in_arm(h) || !collides(h)) continue;
        if (model().body_weldid[model().geom_bodyid[g]] == 0 &&
            model().body_weldid[model().geom_bodyid[h]] == 0) {
          continue;
        }
        const double d =
            mj_geomDistance(cell().model(), data(), g, h, 1.0, nullptr);
        if (d < nearest) {
          nearest = d;
          if (between != nullptr) {
            *between = std::string(mj_id2name(&model(), mjOBJ_GEOM, g)) +
                       " and " + BodyName(model(), model().geom_bodyid[h]);
          }
        }
      }
    }
    return nearest;
  }

  std::optional<Cell> cell_;
};

// ------------------------------------------------------------ the parts

TEST_F(CellTest, LoadsWithEveryPartAndTheRightCounts) {
  EXPECT_EQ(cell::Arm::kJoints, 6);
  EXPECT_EQ(cell().module_count(), 30);
  EXPECT_EQ(cell::Conveyor::kSlotCount, 10);

  // The model really has 10 pairs of rails: rail_a0 .. rail_a9.
  int rails = 0;
  for (int g = 0; g < model().ngeom; ++g) {
    const char* name = mj_id2name(&model(), mjOBJ_GEOM, g);
    if (name != nullptr && std::string(name).rfind("rail_a", 0) == 0) ++rails;
  }
  EXPECT_EQ(rails, cell::Conveyor::kSlotCount);

  // The arm comes first, so its keyframes line up with its joints and
  // motors, and the rest of each keyframe is the model as built.
  for (int j = 0; j < cell::Arm::kJoints; ++j) {
    EXPECT_EQ(cell().arm().qpos[j], j);
    EXPECT_EQ(cell().arm().motor[j], j);
  }
  for (const char* key_name : {"arm_home", "arm_clear"}) {
    const int key = mj_name2id(&model(), mjOBJ_KEY, key_name);
    ASSERT_GE(key, 0) << key_name;
    for (int k = cell::Arm::kJoints; k < model().nq; ++k) {
      EXPECT_EQ(model().key_qpos[key * model().nq + k], model().qpos0[k])
          << key_name << " qpos[" << k << "]";
    }
  }
  // Keyframe 0, the one the viewer starts from, is arm_home.
  EXPECT_EQ(mj_name2id(&model(), mjOBJ_KEY, "arm_home"), 0);

  // Every module the suction can grip is one of the pallet's.
  for (int body : cell().suction().modules()) {
    EXPECT_NE(std::find(cell().pallet().modules.begin(),
                        cell().pallet().modules.end(), body),
              cell().pallet().modules.end())
        << BodyName(model(), body);
  }

  const cell::CameraModel camera = cell().DepthCamera();
  EXPECT_EQ(camera.width, 640);
  EXPECT_EQ(camera.height, 480);
  EXPECT_GE(cell().depth_camera_id(), 0);
}

TEST(CellLoadTest, MissingFileOrPartSaysWhat) {
  std::string error;
  EXPECT_FALSE(Cell::Load("no/such/cell.xml", &error));
  EXPECT_NE(error.find("could not load"), std::string::npos) << error;

  // The arm alone: loads, but has no pallet.
  const char* arm_scene = std::getenv("ARM_SCENE_PATH");
  ASSERT_NE(arm_scene, nullptr) << "ARM_SCENE_PATH is not set";
  EXPECT_FALSE(Cell::Load(arm_scene, &error));
  EXPECT_NE(error.find("no pallet"), std::string::npos) << error;
}

// ------------------------------------------------------------ at rest

// The stage's "the standing stack stays put while untouched", now with the
// arm, the conveyor and the suction present.
TEST_F(CellTest, NominalResetIsAtRestForTenSeconds) {
  ASSERT_TRUE(cell().Reset(1, cell::Randomization::None()));
  EXPECT_NEAR(data()->time, Cell::kSettleTime, 1e-9);

  const int n = cell().module_count();
  std::vector<Eigen::Isometry3d> start, halfway;
  for (int i = 0; i < n; ++i) start.push_back(cell().TrueModulePose(i));
  const double tube_start = cell().conveyor().TrueSlidePosition(*data());

  double moved = 0, turned = 0, arm_off = 0, tube_off = 0;
  const double watch = 10.0;
  const long steps = std::lround(watch / model().opt.timestep);
  for (long k = 0; k < steps; ++k) {
    cell().Step();
    mj_kinematics(cell().model(), data());  // poses of the state now
    for (int i = 0; i < n; ++i) {
      const Eigen::Isometry3d now = cell().TrueModulePose(i);
      moved = std::max(moved, (now.translation() - start[i].translation()).norm());
      turned = std::max(turned, Eigen::AngleAxisd(start[i].rotation().transpose() *
                                                  now.rotation())
                                    .angle());
    }
    const Eigen::VectorXd q = cell::JointAngles(cell().arm(), *data());
    arm_off = std::max(arm_off, (q - cell().arm_home()).cwiseAbs().maxCoeff());
    tube_off = std::max(tube_off,
                        std::abs(cell().conveyor().TrueSlidePosition(*data()) -
                                 tube_start));
    if (k == steps / 2) {
      for (int i = 0; i < n; ++i) halfway.push_back(cell().TrueModulePose(i));
    }
  }
  mj_forward(cell().model(), data());

  double first_half = 0, second_half = 0, speed = 0, spin = 0;
  for (int i = 0; i < n; ++i) {
    const Eigen::Isometry3d now = cell().TrueModulePose(i);
    first_half = std::max(
        first_half, (halfway[i].translation() - start[i].translation()).norm());
    second_half = std::max(
        second_half, (now.translation() - halfway[i].translation()).norm());
    const Eigen::Vector2d v = cell().TrueModuleSpeedOnTube(i);
    speed = std::max(speed, v[0]);
    spin = std::max(spin, v[1]);
  }
  EXPECT_LT(moved, kStillDistance);
  EXPECT_LT(turned, kStillAngle);
  EXPECT_LT(first_half, kDriftPerHalf);
  EXPECT_LT(second_half, kDriftPerHalf);
  EXPECT_LT(speed, kEndSpeed);
  EXPECT_LT(spin, kEndSpin);
  EXPECT_LT(arm_off, kArmHold);
  EXPECT_LT(tube_off, 1e-6);
  EXPECT_LT(std::abs(cell().conveyor().SensorVelocity(*data())), 1e-6);
  std::printf("nominal, 10 s: stack moved %.5f mm, turned %.5f mrad, halves "
              "%.5f / %.5f mm, end %.5f mm/s %.5f mrad/s; arm off home "
              "%.4f mrad; tube moved %.2g m\n",
              1e3 * moved, 1e3 * turned, 1e3 * first_half, 1e3 * second_half,
              1e3 * speed, 1e3 * spin, 1e3 * arm_off, tube_off);
}

// ------------------------------------------------------------ scenes

TEST_F(CellTest, ResetIsRepeatableForASeedAndDiffersBetweenSeeds) {
  struct Snapshot {
    std::vector<mjtNum> qpos, qvel, ctrl, mocap;
    double time = 0;
    double stop_error = 0;
  };
  auto take = [&] {
    Snapshot s;
    s.qpos.assign(data()->qpos, data()->qpos + model().nq);
    s.qvel.assign(data()->qvel, data()->qvel + model().nv);
    s.ctrl.assign(data()->ctrl, data()->ctrl + model().nu);
    s.mocap.assign(data()->mocap_pos, data()->mocap_pos + 3 * model().nmocap);
    s.time = data()->time;
    // The first conveyor step draws its stopping error from the seed.
    EXPECT_TRUE(cell().conveyor().StartStep(*data()));
    s.stop_error = cell().conveyor().true_last_stop_error();
    return s;
  };

  cell().Reset(7);
  const cell::TrueScene first_scene = cell().true_scene();
  const Snapshot first = take();
  cell().Reset(8);
  const Snapshot other = take();
  cell().Reset(7);
  const Snapshot again = take();

  // Bit for bit: exact comparison of every number.
  EXPECT_EQ(first.qpos, again.qpos);
  EXPECT_EQ(first.qvel, again.qvel);
  EXPECT_EQ(first.ctrl, again.ctrl);
  EXPECT_EQ(first.mocap, again.mocap);
  EXPECT_EQ(first.time, again.time);
  EXPECT_EQ(first.stop_error, again.stop_error);
  EXPECT_EQ(cell().true_scene().pallet_x, first_scene.pallet_x);
  EXPECT_EQ(cell().true_scene().pallet_yaw, first_scene.pallet_yaw);

  EXPECT_NE(first.mocap, other.mocap);
  EXPECT_NE(first.qpos, other.qpos);
  EXPECT_NE(first.stop_error, other.stop_error);
}

TEST_F(CellTest, DrawnPalletOffsetsStayWithinTheirRanges) {
  const cell::Randomization ranges;
  EXPECT_DOUBLE_EQ(ranges.pallet_xy, 0.03);
  EXPECT_NEAR(ranges.pallet_yaw, 3.0 * kPi / 180.0, 1e-12);

  // Many seeds: always inside, and spread out to near both ends.
  double low_x = 1, high_x = -1, low_y = 1, high_y = -1, low_yaw = 1,
         high_yaw = -1;
  for (std::uint32_t seed = 0; seed < 20000; ++seed) {
    const cell::TrueScene s = Cell::DrawScene(seed, ranges);
    ASSERT_LE(std::abs(s.pallet_x), ranges.pallet_xy) << seed;
    ASSERT_LE(std::abs(s.pallet_y), ranges.pallet_xy) << seed;
    ASSERT_LE(std::abs(s.pallet_yaw), ranges.pallet_yaw) << seed;
    low_x = std::min(low_x, s.pallet_x);
    high_x = std::max(high_x, s.pallet_x);
    low_y = std::min(low_y, s.pallet_y);
    high_y = std::max(high_y, s.pallet_y);
    low_yaw = std::min(low_yaw, s.pallet_yaw);
    high_yaw = std::max(high_yaw, s.pallet_yaw);
  }
  EXPECT_LT(low_x, -0.99 * ranges.pallet_xy);
  EXPECT_GT(high_x, 0.99 * ranges.pallet_xy);
  EXPECT_LT(low_y, -0.99 * ranges.pallet_xy);
  EXPECT_GT(high_y, 0.99 * ranges.pallet_xy);
  EXPECT_LT(low_yaw, -0.99 * ranges.pallet_yaw);
  EXPECT_GT(high_yaw, 0.99 * ranges.pallet_yaw);

  // A range that makes no sense is refused before anything changes.
  EXPECT_FALSE(cell().Reset(1, {-0.01, 0.0, 0.0}));
  EXPECT_FALSE(cell().Reset(1, {0.0, std::nan(""), 0.0}));
  EXPECT_EQ(data()->time, 0.0);

  // No randomization: nominal exactly.
  const cell::TrueScene none =
      Cell::DrawScene(123, cell::Randomization::None());
  EXPECT_EQ(none.pallet_x, 0.0);
  EXPECT_EQ(none.pallet_y, 0.0);
  EXPECT_EQ(none.pallet_yaw, 0.0);
}

// A randomized Reset: the pallet really is where the draw says, and the
// stack on it is at rest.
TEST_F(CellTest, RandomizedResetPutsThePalletWhereDrawnAndIsAtRest) {
  const std::uint32_t seed = 12345;
  cell().Reset(seed);
  const cell::TrueScene drawn = Cell::DrawScene(seed, {});
  EXPECT_EQ(cell().true_scene().pallet_x, drawn.pallet_x);
  EXPECT_EQ(cell().true_scene().pallet_y, drawn.pallet_y);
  EXPECT_EQ(cell().true_scene().pallet_yaw, drawn.pallet_yaw);

  const cell::WorldPose pallet = cell::PalletPose(*data(), cell().pallet());
  const cell::WorldPose& nominal = cell().pallet().nominal_pallet;
  EXPECT_NEAR(pallet.position.x() - nominal.position.x(), drawn.pallet_x, 1e-12);
  EXPECT_NEAR(pallet.position.y() - nominal.position.y(), drawn.pallet_y, 1e-12);
  const Eigen::AngleAxisd turn(pallet.orientation * nominal.orientation.conjugate());
  EXPECT_NEAR(turn.angle() * turn.axis().z(), drawn.pallet_yaw, 1e-9);

  cell().StepFor(3.0);
  mj_forward(cell().model(), data());
  const cell::StackDisturbance seen = cell::MeasureStackDisturbance(cell());
  EXPECT_EQ(seen.modules, 30);
  EXPECT_LT(seen.worst_displacement, kStillDistance) << cell::Describe(seen);
  EXPECT_LT(seen.worst_rotation, kStillAngle) << cell::Describe(seen);
  for (int i = 0; i < cell().module_count(); ++i) {
    const Eigen::Vector2d v = cell().TrueModuleSpeedOnTube(i);
    EXPECT_LT(v[0], kEndSpeed) << "module_" << i;
    EXPECT_LT(v[1], kEndSpin) << "module_" << i;
  }
  std::printf("seed %u: pallet %+.4f %+.4f m %+.3f deg; 3 s later %s\n", seed,
              drawn.pallet_x, drawn.pallet_y, drawn.pallet_yaw * 180 / kPi,
              cell::Describe(seen).c_str());
}

// ------------------------------------------------------------ the camera

// What the camera sees of one module's glass, without rendering.
struct View {
  double margin = 1e9;  // nearest any corner comes to the image's edge, px
  int blocked = 0;      // rays that hit something else first
  int rays = 0;
  std::string first_blocker;
};

// Projects the four corners of module i's glass and a grid of points on it
// through the camera, and casts a ray from the camera to each grid point.
View LookAt(Cell& cell, int i) {
  const cell::CameraModel camera = cell.DepthCamera();
  const Eigen::Isometry3d module = cell.TrueModulePose(i);
  const double half_x = cell::kModuleLength / 2;
  const double half_y = cell::kModuleWidth / 2;
  const double glass = cell::kModuleThickness / 2;
  View view;
  auto margin_of = [&](const Eigen::Vector3d& point) {
    const std::optional<Eigen::Vector2d> pixel = cell::Project(camera, point);
    if (!pixel) return -1e9;  // behind the camera
    return std::min({pixel->x(), camera.width - pixel->x(), pixel->y(),
                     camera.height - pixel->y()});
  };
  for (double a : {-1.0, 1.0}) {
    for (double b : {-1.0, 1.0}) {
      view.margin = std::min(
          view.margin,
          margin_of(module * Eigen::Vector3d(a * half_x, b * half_y, glass)));
    }
  }

  // A 9 x 5 grid, its outer points 1 cm in from the edges so that no ray
  // only grazes the glass.
  const int body = cell.pallet().modules[i];
  const Eigen::Vector3d from = camera.pose.translation();
  for (int u = 0; u < 9; ++u) {
    for (int v = 0; v < 5; ++v) {
      const Eigen::Vector3d point =
          module * Eigen::Vector3d((half_x - 0.01) * (u / 4.0 - 1.0),
                                   (half_y - 0.01) * (v / 2.0 - 1.0), glass);
      view.margin = std::min(view.margin, margin_of(point));
      const Eigen::Vector3d direction = (point - from).normalized();
      int geom = -1;
      // All geom groups (the hidden collision hulls too, which are fatter
      // than what is drawn), static geoms included, nothing excluded.
      const mjtNum distance =
          mj_ray(cell.model(), cell.data(), from.data(), direction.data(),
                 nullptr, 1, -1, &geom, nullptr);
      ++view.rays;
      const bool hit_it = geom >= 0 && cell.model()->geom_bodyid[geom] == body &&
                          std::abs(distance - (point - from).norm()) < 1e-3;
      if (!hit_it) {
        ++view.blocked;
        if (view.first_blocker.empty()) {
          view.first_blocker =
              geom < 0 ? "nothing"
                       : BodyName(*cell.model(), cell.model()->geom_bodyid[geom]);
        }
      }
    }
  }
  return view;
}

// The whole glass face of the front module is in the picture, with margin,
// and in plain sight with the arm at arm_clear: with a full pallet and with
// only the last module left, at the nominal pose and at every corner of the
// randomization's range.
TEST_F(CellTest, CameraSeesTheWholeFrontGlassWithTheArmClear) {
  cell().Reset(0, cell::Randomization::None());
  cell().PutArm(cell().arm_clear());

  const cell::Randomization range;
  double worst_margin = 1e9;
  std::string worst_case;
  for (double x : {-1.0, 0.0, 1.0}) {
    for (double y : {-1.0, 0.0, 1.0}) {
      for (double yaw : {-1.0, 0.0, 1.0}) {
        for (bool last_only : {false, true}) {
          // Back on the pallet, all of them, at this offset.
          cell::PlacePallet(model(), data(), cell().pallet(), x * range.pallet_xy,
                            y * range.pallet_xy, yaw * range.pallet_yaw);
          int front = 0;
          if (last_only) {
            front = cell().module_count() - 1;
            for (int i = 0; i < front; ++i) cell().ParkModule(i);
          }
          const View view = LookAt(cell(), front);
          const std::string name =
              (last_only ? "module_29 alone" : "full pallet") +
              std::string(", offset ") + std::to_string(x) + " " +
              std::to_string(y) + " " + std::to_string(yaw);
          EXPECT_GT(view.margin, 20.0) << name;
          EXPECT_EQ(view.blocked, 0)
              << name << ": " << view.blocked << " of " << view.rays
              << " rays blocked, first by " << view.first_blocker;
          if (view.margin < worst_margin) {
            worst_margin = view.margin;
            worst_case = name;
          }
        }
      }
    }
  }
  std::printf("camera: worst corner %.1f px from the image edge (%s)\n",
              worst_margin, worst_case.c_str());

  // The check can fail: at arm_home the tool hangs in front of the stack.
  cell::PlacePallet(model(), data(), cell().pallet(), 0, 0, 0);
  cell().PutArm(cell().arm_home());
  const View home = LookAt(cell(), 0);
  EXPECT_GT(home.blocked, 0);
  std::printf("camera at arm_home: %d of %d rays blocked, first by %s\n",
              home.blocked, home.rays, home.first_blocker.c_str());

  // Parked modules are nowhere in the picture: each lies wholly beyond one
  // of the four planes that bound the camera's view.
  const cell::CameraModel camera = cell().DepthCamera();
  const Eigen::Isometry3d to_camera = camera.pose.inverse();
  for (int i = 0; i < cell().module_count(); ++i) {
    bool outside = false;
    // Each plane through the camera's centre, as (normal . p > 0 is out):
    // left, right, top and bottom edges of the image.
    const std::array<Eigen::Vector3d, 4> planes = {
        Eigen::Vector3d(-camera.focal, 0, -camera.cx),
        Eigen::Vector3d(camera.focal, 0, -(camera.width - camera.cx)),
        Eigen::Vector3d(0, camera.focal, -camera.cy),
        Eigen::Vector3d(0, -camera.focal, -(camera.height - camera.cy))};
    for (const Eigen::Vector3d& normal : planes) {
      bool all_out = true;
      for (int corner = 0; corner < 8; ++corner) {
        const Eigen::Vector3d local(
            (corner & 1 ? 1 : -1) * cell::kModuleLength / 2,
            (corner & 2 ? 1 : -1) * cell::kModuleWidth / 2,
            (corner & 4 ? 1 : -1) * cell::kModuleThickness / 2);
        const Eigen::Vector3d p = to_camera * (cell::ParkingPose(i) * local);
        if (normal.dot(p) <= 0) all_out = false;
      }
      outside = outside || all_out;
    }
    EXPECT_TRUE(outside) << "parking spot " << i << " is in the picture";
  }
}

// ------------------------------------------------------------ contacts

TEST_F(CellTest, NothingTouchesTheArmAtHomeOrClear) {
  cell().Reset(0, cell::Randomization::None());
  const int at_home = ExpectOnlyStackContacts();
  std::string home_pair, clear_pair;
  const double home_clearance = ArmClearance(&home_pair);
  EXPECT_GT(home_clearance, 0.0);

  cell().PutArm(cell().arm_clear());
  cell().StepFor(0.5);
  const int at_clear = ExpectOnlyStackContacts();
  const double clear_clearance = ArmClearance(&clear_pair);
  EXPECT_GT(clear_clearance, 0.0);
  const Eigen::VectorXd q = cell::JointAngles(cell().arm(), *data());
  EXPECT_LT((q - cell().arm_clear()).cwiseAbs().maxCoeff(), kArmHold);

  std::printf("contacts at arm_home %d, at arm_clear %d, all in the stack; "
              "arm's nearest approach to anything: %.3f m at home (%s), "
              "%.3f m clear (%s)\n",
              at_home, at_clear, home_clearance, home_pair.c_str(),
              clear_clearance, clear_pair.c_str());
}

// The camera's post, boom and housing are out of the arm's reach: over a few
// thousand joint angles drawn across the whole of every joint's range, no
// part of the arm or the tool comes near them. Kinematics only.
TEST_F(CellTest, ArmCannotReachTheCameraOrItsPost) {
  const int post = mj_name2id(&model(), mjOBJ_BODY, "camera_post");
  ASSERT_GE(post, 0);
  std::vector<int> post_geoms, arm_geoms;
  const std::vector<int> arm = ArmBodies();
  for (int g = 0; g < model().ngeom; ++g) {
    const int body = model().geom_bodyid[g];
    if (body == post) post_geoms.push_back(g);
    // Only what collides: the visual meshes lie inside the collision hulls.
    if (std::find(arm.begin(), arm.end(), body) != arm.end() &&
        model().geom_contype[g] != 0) {
      arm_geoms.push_back(g);
    }
  }
  ASSERT_EQ(post_geoms.size(), 4u);

  std::mt19937 random(5);
  double nearest = 10.0;
  for (int sample = 0; sample < 3000; ++sample) {
    for (int j = 0; j < cell::Arm::kJoints; ++j) {
      const double unit = (random() + 0.5) / 4294967296.0;
      data()->qpos[cell().arm().qpos[j]] =
          cell().arm().lower[j] +
          unit * (cell().arm().upper[j] - cell().arm().lower[j]);
    }
    mj_kinematics(cell().model(), data());
    for (int g : arm_geoms) {
      for (int h : post_geoms) {
        nearest = std::min(
            nearest, mj_geomDistance(cell().model(), data(), g, h, 10.0, nullptr));
      }
    }
  }
  EXPECT_GT(nearest, 0.2);
  std::printf("arm to camera post, nearest over 3000 random poses: %.3f m\n",
              nearest);
}

// ------------------------------------------------------------ the scorer

TEST_F(CellTest, ScorerReturnsAKnownPlacementExactly) {
  cell().Reset(0, cell::Randomization::None());

  struct Case {
    int module, slot;
    double along, across, height, yaw, tilt;
    double expected_yaw;
  };
  // Lying flat on the rails, a little off; turned almost half a turn, which
  // is the same rectangle; floating a little and tilted. Every other slot,
  // so that turned modules do not reach into their neighbours' (a 2.3 m
  // module turned by 0.03 rad sweeps 35 mm at its ends, more than the 20 mm
  // gap).
  const std::vector<Case> cases = {
      {0, 0, 0.012, -0.007, 0.0, 0.02, 0.0, 0.02},
      {1, 2, -0.004, 0.015, 0.0, kPi - 0.03, 0.0, -0.03},
      {2, 4, 0.0, 0.0, 0.0, -kPi + 0.01, 0.0, 0.01},
      {3, 6, 0.002, 0.001, 0.004, 0.05, 0.01, 0.05},
  };
  for (const Case& c : cases) {
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation() = Eigen::Vector3d(c.across, c.along, c.height);
    // Turned about the vertical, then tipped about the module's long side.
    offset.linear() =
        (Eigen::AngleAxisd(c.yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(c.tilt, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    cell().TeleportModule(
        c.module, cell().conveyor().TrueSlotPose(*data(), c.slot) * offset);
    EXPECT_EQ(cell().module_state(c.module), ModuleState::kReleased);
    ASSERT_EQ(cell().TruePlacedSlot(c.module), c.slot);

    const std::optional<cell::PlacementError> e =
        cell::MeasurePlacement(cell(), c.module);
    ASSERT_TRUE(e);
    EXPECT_EQ(e->slot, c.slot);
    EXPECT_NEAR(e->along_tube, c.along, 1e-12);
    EXPECT_NEAR(e->across_tube, c.across, 1e-12);
    EXPECT_NEAR(e->height, c.height, 1e-12);
    EXPECT_NEAR(e->yaw, c.expected_yaw, 1e-12);
    EXPECT_NEAR(e->tilt, c.tilt, 1e-9);
    EXPECT_TRUE(e->at_rest);  // teleported at rest
    std::printf("%s\n", cell::Describe(*e).c_str());
  }

  // The floating one away; the three lying on the rails stay as they were.
  cell().ParkModule(3);
  cell().StepFor(0.5);
  mj_forward(cell().model(), data());
  for (int k = 0; k < 3; ++k) {
    const Case& c = cases[k];
    const cell::PlacementError e = cell::MeasurePlacement(cell(), c.module, c.slot);
    EXPECT_TRUE(e.at_rest) << cell::Describe(e);
    EXPECT_NEAR(e.along_tube, c.along, 0.1e-3) << cell::Describe(e);
    EXPECT_NEAR(e.across_tube, c.across, 0.1e-3) << cell::Describe(e);
    EXPECT_NEAR(e.height, 0.0, 0.1e-3) << cell::Describe(e);
    EXPECT_NEAR(e.yaw, c.expected_yaw, 0.1e-3) << cell::Describe(e);
    EXPECT_LT(e.tilt, 0.1e-3) << cell::Describe(e);
  }

  // A module not let go over the tube is not scored against any slot.
  EXPECT_FALSE(cell::MeasurePlacement(cell(), 10));
}

TEST_F(CellTest, StackDisturbanceIsZeroUntouchedAndSeesAPush) {
  cell().Reset(0, cell::Randomization::None());
  cell().StepFor(0.5);
  mj_forward(cell().model(), data());
  cell::StackDisturbance d = cell::MeasureStackDisturbance(cell());
  EXPECT_EQ(d.modules, 30);
  EXPECT_LT(d.worst_displacement, 0.01e-3) << cell::Describe(d);
  EXPECT_LT(d.worst_rotation, 0.01e-3) << cell::Describe(d);

  // The front module taken away: it no longer counts, and the rest have not
  // moved.
  cell().TeleportModule(0, cell().conveyor().TrueSlotPose(*data(), 0));
  d = cell::MeasureStackDisturbance(cell());
  EXPECT_EQ(d.modules, 29);
  EXPECT_LT(d.worst_displacement, 0.01e-3) << cell::Describe(d);

  // Shove module_1, now in front, sideways for a tenth of a second.
  const int body = cell().pallet().modules[1];
  data()->xfrc_applied[6 * body + 1] = -300.0;  // N, along world -Y
  cell().StepFor(0.1);
  data()->xfrc_applied[6 * body + 1] = 0.0;
  cell().StepFor(0.5);
  mj_forward(cell().model(), data());
  d = cell::MeasureStackDisturbance(cell());
  EXPECT_GT(d.worst_displacement, 5e-3) << cell::Describe(d);
  EXPECT_EQ(d.worst_displacement_module, 1) << cell::Describe(d);
  std::printf("after a push: %s\n", cell::Describe(d).c_str());

  // A new baseline starts from where things are now.
  cell().RebaselineStack();
  d = cell::MeasureStackDisturbance(cell());
  EXPECT_LT(d.worst_displacement, 1e-12);
  EXPECT_LT(d.worst_rotation, 1e-12);
}

// ------------------------------------------------------------ carrying away

TEST_F(CellTest, GripReleaseAndParkKeepTheBookkeeping) {
  cell().Reset(0, cell::Randomization::None());

  // Put module_0 under the tool at home, glass up and 2 mm below the cups,
  // and grip it. Its long side along Y, so that it stays clear of the upper
  // arm (it would reach x = 0.4 along X).
  const Eigen::Isometry3d tip = cell::TipPose(cell().arm(), *data());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() =
      Eigen::AngleAxisd(kPi / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.translation() =
      tip.translation() - Eigen::Vector3d(0, 0, 0.002 + cell::kModuleThickness / 2);
  cell().TeleportModule(0, pose);
  EXPECT_FALSE(cell().TruePlacedSlot(0));  // not over the tube
  cell::GripFailure failure;
  const std::optional<int> gripped = cell().Grip(&failure);
  ASSERT_EQ(gripped, 0) << failure.message;
  EXPECT_EQ(cell().module_state(0), ModuleState::kHeld);
  cell().StepFor(0.2);
  EXPECT_EQ(cell().module_state(0), ModuleState::kHeld);
  // Held up by the arm, which sagged a little under the weight.
  mj_forward(cell().model(), data());
  EXPECT_LT((cell::TipPose(cell().arm(), *data()).translation() -
             tip.translation())
                .norm(),
            5e-3);
  EXPECT_LT((cell().TrueModulePose(0).translation() - pose.translation()).norm(),
            5e-3);
  EXPECT_EQ(cell().Release(), 0);
  EXPECT_EQ(cell().module_state(0), ModuleState::kReleased);
  EXPECT_FALSE(cell().suction().holding());
  EXPECT_FALSE(cell().Release());  // nothing held now

  // Parked before it has fallen anywhere.
  cell().ParkModule(0);
  EXPECT_EQ(cell().module_state(0), ModuleState::kParked);

  // Nothing under the tool now: no grip.
  EXPECT_FALSE(cell().Grip(&failure));
  EXPECT_EQ(failure.kind, cell::GripFailure::Kind::kTooFar) << failure.message;

  // A Reset puts every module back on the pallet and lets go of anything
  // held, weld and bookkeeping both.
  cell().TeleportModule(1, pose);
  ASSERT_EQ(cell().Grip(&failure), 1) << failure.message;
  const int weld = mj_name2id(&model(), mjOBJ_EQUALITY, "suction");
  ASSERT_GE(weld, 0);
  EXPECT_TRUE(data()->eq_active[weld]);
  cell().Reset(0, cell::Randomization::None());
  for (int i = 0; i < cell().module_count(); ++i) {
    EXPECT_EQ(cell().module_state(i), ModuleState::kOnPallet) << i;
  }
  EXPECT_FALSE(cell().suction().holding());
  EXPECT_FALSE(data()->eq_active[weld]);
}

TEST_F(CellTest, SwapTubeParksWhatIsOnTheTubeAndStartsEmpty) {
  cell().Reset(3);  // randomized, so the conveyor stops a little off

  // Lay module_0 in slot 0, step the conveyor, lay module_1 in slot 1.
  cell().TeleportModule(0, cell().conveyor().TrueSlotPose(*data(), 0));
  ASSERT_TRUE(cell().conveyor().StartStep(*data()));
  cell().Step();
  EXPECT_FALSE(cell().SwapTube());  // moving: refused, nothing parked
  EXPECT_EQ(cell().module_state(0), ModuleState::kReleased);
  double waited = 0;
  while (cell().conveyor().Moving(*data()) && waited < 10.0) {
    cell().StepFor(0.1);
    waited += 0.1;
  }
  ASSERT_FALSE(cell().conveyor().Moving(*data()));
  mj_forward(cell().model(), data());

  // module_0 rode along in its slot.
  const cell::PlacementError rode = cell::MeasurePlacement(cell(), 0, 0);
  EXPECT_LT(std::hypot(rode.along_tube, rode.across_tube), 1e-3)
      << cell::Describe(rode);
  EXPECT_TRUE(rode.at_rest) << cell::Describe(rode);
  std::printf("after one step (stop error %+.2f mm): %s\n",
              1e3 * cell().conveyor().true_last_stop_error(),
              cell::Describe(rode).c_str());

  cell().TeleportModule(1, cell().conveyor().TrueSlotPose(*data(), 1));
  EXPECT_EQ(cell().TruePlacedSlot(1), 1);
  // And one straight from the pallet to the parking, by hand.
  cell().ParkModule(5);

  ASSERT_TRUE(cell().SwapTube());
  for (int i : {0, 1, 5}) {
    EXPECT_EQ(cell().module_state(i), ModuleState::kParked) << i;
    EXPECT_FALSE(cell().TruePlacedSlot(i)) << i;
  }
  EXPECT_EQ(cell().conveyor().steps(), 0);
  EXPECT_EQ(cell().conveyor().tubes_completed(), 1);
  EXPECT_EQ(cell().conveyor().TrueSlidePosition(*data()), 0.0);
  EXPECT_EQ(cell().conveyor().SensorPosition(*data()), 0.0);

  // Nothing lies on the tube: no module's centre is over its rails.
  const Eigen::Isometry3d to_tube = cell().conveyor().TrueTubePose(*data()).inverse();
  for (int i = 0; i < cell().module_count(); ++i) {
    const Eigen::Vector3d c = to_tube * cell().TrueModulePose(i).translation();
    const bool over_rails = std::abs(c.x()) < 1.15 + 1.0 && c.y() > -1.0 &&
                            c.y() < cell::Conveyor::kSlotCount * 1.12 &&
                            std::abs(c.z()) < 0.5;
    EXPECT_FALSE(over_rails) << "module_" << i;
  }

  // The parked modules come to rest where they were put, on the floor and
  // touching nothing else.
  cell().StepFor(1.0);
  mj_forward(cell().model(), data());
  const int floor = mj_name2id(&model(), mjOBJ_GEOM, "floor");
  for (int i : {0, 1, 5}) {
    const int body = cell().pallet().modules[i];
    const Eigen::Isometry3d now = cell().TrueModulePose(i);
    EXPECT_LT((now.translation() - cell::ParkingPose(i).translation()).norm(),
              0.5e-3)
        << i;
    const Eigen::Vector2d v = cell().TrueModuleSpeedOnTube(i);
    // The tube is still, so speed on the tube is speed.
    EXPECT_LT(v[0], 1e-4) << i;
    EXPECT_LT(v[1], 1e-3) << i;
    int touches = 0;
    for (int c = 0; c < data()->ncon; ++c) {
      const mjContact& contact = data()->contact[c];
      const int a = contact.geom[0], b = contact.geom[1];
      if (model().geom_bodyid[a] != body && model().geom_bodyid[b] != body) {
        continue;
      }
      ++touches;
      EXPECT_TRUE(a == floor || b == floor)
          << "parked module_" << i << " touches "
          << BodyName(model(), model().geom_bodyid[a == floor ? b : a]);
    }
    EXPECT_GT(touches, 0) << "module_" << i << " is not on the floor";
  }
}

}  // namespace
