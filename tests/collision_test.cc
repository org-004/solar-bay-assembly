// Checks the collision test for the planner (src/collision.h): free poses
// are free and the simulated state is left alone; poses that collide are
// caught and name the right pair; the scripted pick and place's own path is
// free with the right allowances, and agrees with cycle_check on the
// simulated cycle; the margin; segments; the fast private model against a
// plain copy; obstacle updates; the lever arms; and the time per query.
//
//   bazel test //:collision_test
//   bazel test --config=opt //:collision_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/cell.xml). Every scene is the
// nominal one: Reset(1, Randomization::None()).
//
// Most of the time goes to simulating one nominal cycle (about 20 s of the
// cell, at about real time unoptimized) and to the thousands of random
// configurations that compare the fast model with the plain one. The
// numbers measured (clearances along the cycle, microseconds per query)
// are printed; read them with --test_output=all, and the times only from
// the optimized build.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
#include "src/cell.h"
#include "src/collision.h"
#include "src/cycle_check.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/pallet.h"
#include "src/pick_place.h"

namespace {

using cell::Cell;
using cell::CheckOptions;
using cell::CollisionChecker;
using cell::CollisionOptions;
using cell::CollisionReport;
using cell::PickPlace;
using Step = cell::PickPlaceStep;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;

Eigen::VectorXd Degrees(double a, double b, double c, double d, double e,
                        double f) {
  Eigen::VectorXd q(6);
  q << a, b, c, d, e, f;
  return q * kDegree;
}

std::string BodyName(const mjModel& model, int body) {
  const char* name = mj_id2name(&model, mjOBJ_BODY, body);
  return name != nullptr ? name : "body " + std::to_string(body);
}

std::string GeomName(const mjModel& model, int geom) {
  const char* name = mj_id2name(&model, mjOBJ_GEOM, geom);
  return name != nullptr ? name : "";
}

bool IsLink(const mjModel& model, int body) {
  return BodyName(model, body).rfind("arm_", 0) == 0;
}

// Every pair of a report, for messages.
std::string Pairs(const CollisionReport& report) {
  std::string text;
  for (const cell::CollidingPair& p : report.pairs) {
    char line[200];
    std::snprintf(line, sizeof(line), "\n  %s  %.1f mm", p.between.c_str(),
                  1e3 * p.distance);
    text += line;
  }
  return text.empty() ? " (none)" : text;
}

Eigen::Isometry3d FromPose(const cell::Pose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = pose.position;
  result.linear() = pose.rotation.toRotationMatrix();
  return result;
}

// Where body `body` is in `data`.
Eigen::Isometry3d BodyPose(const mjData& data, int body) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(data.xpos[3 * body], data.xpos[3 * body + 1],
                                       data.xpos[3 * body + 2]);
  pose.linear() = Eigen::Quaterniond(data.xquat[4 * body], data.xquat[4 * body + 1],
                                     data.xquat[4 * body + 2],
                                     data.xquat[4 * body + 3])
                      .toRotationMatrix();
  return pose;
}

class CollisionTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";
    std::string error;
    std::optional<Cell> loaded = Cell::Load(path, &error);
    ASSERT_TRUE(loaded) << error;
    cell_.emplace(std::move(*loaded));
    ASSERT_TRUE(cell_->Reset(1, cell::Randomization::None()));

    // The scripted cycle for module_0, all of it planned (starting at the
    // set-down plans the place side too); nothing is simulated.
    script_.emplace(*cell_);
    ASSERT_TRUE(script_->Start(0, Step::kSetDown)) << script_->outcome().reason;

    // The grip as designed: the module where it stands, the tool at the
    // touch pose, 2 mm in front of its glass. The weld holds what it finds
    // at the grip; this is what it finds when the arm arrives exactly.
    const Eigen::VectorXd home = cell_->arm_home();
    cell_->PutArm(Waypoints(Step::kTouchGlass).back());
    grip_ = cell::TipPose(cell_->arm(), *cell_->data()).inverse() *
            cell_->TrueModulePose(0);
    cell_->PutArm(home);
  }

  Cell& cell() { return *cell_; }
  const mjModel& model() { return *cell_->model(); }
  const std::vector<Eigen::VectorXd>& Waypoints(Step step) {
    return *script_->Waypoints(step);
  }

  // A checker for the cell as it stands now.
  CollisionChecker MakeChecker(const CollisionOptions& options = {}) {
    std::string error;
    std::optional<CollisionChecker> checker =
        CollisionChecker::Create(model(), options, &error);
    EXPECT_TRUE(checker) << error;
    checker->SetScene(*cell_->data());
    return std::move(*checker);
  }

  // Joint angles putting the tool tip at `target`, from `seed`.
  Eigen::VectorXd Solve(const Eigen::Isometry3d& target,
                        const Eigen::VectorXd& seed) {
    cell::DataPtr scratch = cell::MakeData(model());
    cell::IkReport report;
    std::optional<Eigen::VectorXd> q = cell::SolveIk(
        model(), scratch.get(), cell_->arm(), target, seed, {}, &report);
    EXPECT_TRUE(q) << "IK: " << report.position_error << " m short";
    return q.value_or(seed);
  }

  std::optional<Cell> cell_;
  std::optional<PickPlace> script_;
  Eigen::Isometry3d grip_ = Eigen::Isometry3d::Identity();
};

// ------------------------------------------------------------- free poses

TEST_F(CollisionTest, HomeAndClearAreFreeAndTheLiveStateIsUntouched) {
  const mjData& live = *cell().data();
  const std::vector<unsigned char> buffer(
      static_cast<const unsigned char*>(live.buffer),
      static_cast<const unsigned char*>(live.buffer) + live.nbuffer);
  const std::vector<unsigned char> arena(
      static_cast<const unsigned char*>(live.arena),
      static_cast<const unsigned char*>(live.arena) + live.narena);
  const double time = live.time;
  const int ncon = live.ncon;

  CollisionChecker checker = MakeChecker();
  for (const char* name : {"arm_home", "arm_clear"}) {
    const Eigen::VectorXd& q =
        name[4] == 'h' ? cell().arm_home() : cell().arm_clear();
    std::optional<CollisionReport> report =
        checker.Check(q, {.all_pairs = true, .clearance = true});
    ASSERT_TRUE(report);
    EXPECT_TRUE(report->free) << name << Pairs(*report);
    EXPECT_TRUE(checker.IsFree(q)) << name;
    std::printf("%s: clearance %.1f mm (%s), self %.1f mm (%s)\n", name,
                1e3 * *report->clearance, report->clearance_between.c_str(),
                1e3 * *report->self_clearance,
                report->self_clearance_between.c_str());
    EXPECT_GT(*report->clearance, checker.margin()) << name;
  }
  // Everything else a planner does, against the live state.
  ASSERT_TRUE(checker.SetHeldModule(0, grip_));
  std::mt19937 rng(5);
  for (int i = 0; i < 200; ++i) {
    Eigen::VectorXd q(6);
    for (int j = 0; j < 6; ++j) {
      q[j] = std::uniform_real_distribution<double>(
          cell().arm().lower[j], cell().arm().upper[j])(rng);
    }
    checker.Check(q, {.all_pairs = true, .clearance = i % 10 == 0});
  }
  checker.SegmentIsFree(cell().arm_home(), cell().arm_clear(), 0.01);
  checker.ClearHeldModule();
  checker.SetScene(live);

  EXPECT_EQ(std::memcmp(buffer.data(), live.buffer, buffer.size()), 0)
      << "the live mjData's buffer changed";
  EXPECT_EQ(std::memcmp(arena.data(), live.arena, arena.size()), 0)
      << "the live mjData's arena changed";
  EXPECT_EQ(live.time, time);
  EXPECT_EQ(live.ncon, ncon);
}

TEST_F(CollisionTest, BadInputAndTheApi) {
  CollisionChecker checker = MakeChecker();
  EXPECT_FALSE(checker.Check(Eigen::VectorXd::Zero(5)));
  EXPECT_FALSE(checker.IsFree(Eigen::VectorXd::Zero(5)));
  Eigen::VectorXd nan = cell().arm_home();
  nan[2] = std::nan("");
  EXPECT_FALSE(checker.Check(nan));

  // Outside a joint's range: not free, and nothing was checked.
  Eigen::VectorXd beyond = cell().arm_home();
  beyond[1] = cell().arm().upper[1] + 0.01;
  std::optional<CollisionReport> report = checker.Check(beyond);
  ASSERT_TRUE(report);
  EXPECT_FALSE(report->in_range);
  EXPECT_FALSE(report->free);

  EXPECT_FALSE(checker.SetMargins(-0.01, 0.0));
  EXPECT_FALSE(checker.SetMargins(0.01, 0.03));
  EXPECT_FALSE(checker.SetMargins(std::nan(""), 0.0));
  EXPECT_DOUBLE_EQ(checker.margin(), 0.02);
  EXPECT_FALSE(checker.AllowContact("tool", "no_such_body"));
  EXPECT_FALSE(checker.SetHeldModule(30, grip_));
  EXPECT_FALSE(checker.SetHeldModule(-1, grip_));

  cell::SegmentReport segment;
  EXPECT_FALSE(checker.SegmentIsFree(cell().arm_home(), cell().arm_clear(),
                                     0.0, &segment));
  EXPECT_EQ(segment.samples, 0);

  // Allowed pairs: saved, cleared and restored.
  ASSERT_TRUE(checker.AllowContact("module_0", "tool"));
  ASSERT_TRUE(checker.AllowContact("tool", "module_0"));  // the same pair
  ASSERT_EQ(checker.allowed_contacts().size(), 1u);
  const std::vector<std::pair<int, int>> saved = checker.allowed_contacts();
  checker.ClearAllowedContacts();
  EXPECT_TRUE(checker.allowed_contacts().empty());
  EXPECT_TRUE(checker.SetAllowedContacts(saved));
  EXPECT_EQ(checker.allowed_contacts(), saved);

  // A model where some geoms never meet cannot be filtered by bits.
  cell::ModelPtr odd(mj_copyModel(nullptr, &model()));
  const int floor = mj_name2id(odd.get(), mjOBJ_GEOM, "floor");
  odd->geom_contype[floor] = 2;
  odd->geom_conaffinity[floor] = 2;
  std::string error;
  EXPECT_FALSE(CollisionChecker::Create(*odd, {}, &error));
  EXPECT_NE(error.find("floor"), std::string::npos) << error;
  EXPECT_TRUE(CollisionChecker::Create(*odd, {.skip_obstacle_pairs = false}));
}

// ------------------------------------------------------ poses that collide

TEST_F(CollisionTest, ToolPushedIntoTheStack) {
  CollisionChecker checker = MakeChecker();
  // The grip pose pushed on into the glass: tool +Z points into the face.
  auto pushed = [&](double depth) {
    Eigen::Isometry3d target = script_->grip_tip();
    target.translation() += depth * target.linear().col(2);
    return Solve(target, Waypoints(Step::kTouchGlass).back());
  };
  const int tool = cell().arm().tool_body;
  const int module_0 = cell().pallet().modules[0];
  const int module_1 = cell().pallet().modules[1];

  // 30 mm: through most of module_0 (35 mm thick, 2 mm in front of the
  // tip) and within the margin of module_1's glass.
  std::optional<CollisionReport> report =
      checker.Check(pushed(0.03), {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_FALSE(report->free);
  EXPECT_EQ(report->pairs[0].body1, tool) << Pairs(*report);
  EXPECT_EQ(report->pairs[0].body2, module_0) << Pairs(*report);
  for (const cell::CollidingPair& p : report->pairs) {
    EXPECT_EQ(p.body1, tool) << p.between;
    EXPECT_TRUE(p.body2 == module_0 || p.body2 == module_1) << p.between;
  }

  // 10 mm in: only module_0; allowing the tool on it frees the pose.
  const Eigen::VectorXd q = pushed(0.01);
  report = checker.Check(q, {.all_pairs = true});
  ASSERT_FALSE(report->free);
  for (const cell::CollidingPair& p : report->pairs) {
    EXPECT_EQ(p.body2, module_0) << p.between;
  }
  ASSERT_TRUE(checker.AllowContact("tool", "module_0"));
  EXPECT_TRUE(checker.IsFree(q)) << Pairs(*checker.Check(q, {.all_pairs = true}));
}

TEST_F(CollisionTest, ArmIntoTheFloor) {
  CollisionChecker checker = MakeChecker();
  // Facing the open floor on -Y, the upper arm nearly level and the
  // forearm pointing straight down: the wrist is 0.36 m below the floor.
  const Eigen::VectorXd q = Degrees(90, -5, 90, 0, 0, 0);
  std::optional<CollisionReport> report = checker.Check(q, {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_FALSE(report->free);
  const int forearm = mj_name2id(&model(), mjOBJ_BODY, "arm_link_4");
  bool forearm_floor = false;
  for (const cell::CollidingPair& p : report->pairs) {
    EXPECT_EQ(GeomName(model(), p.geom2), "floor") << p.between;
    forearm_floor = forearm_floor || p.body1 == forearm;
  }
  EXPECT_TRUE(forearm_floor) << Pairs(*report);
}

TEST_F(CollisionTest, ArmFoldedIntoItself) {
  CollisionChecker checker = MakeChecker();
  const int link_1 = mj_name2id(&model(), mjOBJ_BODY, "arm_link_1");
  const int link_3 = mj_name2id(&model(), mjOBJ_BODY, "arm_link_3");

  // The upper arm leaning 131 degrees back (9 inside its range) and the
  // forearm bent the other way to 1 degree inside its range: the hull of
  // the elbow's housing (link 3) comes down 7 mm into the turning column's
  // (link 1). The wrist bent up keeps the tool off the floor. Links only.
  const Eigen::VectorXd folded = Degrees(0, -131, -119, 0, 110, 0);
  std::optional<CollisionReport> report =
      checker.Check(folded, {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_TRUE(report->in_range);
  ASSERT_FALSE(report->free);
  bool link_1_3 = false;
  for (const cell::CollidingPair& p : report->pairs) {
    EXPECT_TRUE(IsLink(model(), p.body1) && IsLink(model(), p.body2))
        << p.between;
    link_1_3 = link_1_3 || (p.body1 == link_1 && p.body2 == link_3) ||
               (p.body1 == link_3 && p.body2 == link_1);
  }
  EXPECT_TRUE(link_1_3) << Pairs(*report);

  // Upper arm up, forearm folded back 150 degrees beside it, wrist bent
  // toward the arm: the tool's plate in the upper arm.
  const Eigen::VectorXd tucked = Degrees(0, -90, 150, 0, 90, 0);
  report = checker.Check(tucked, {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_FALSE(report->free);
  EXPECT_EQ(report->pairs[0].body1, cell().arm().tool_body) << Pairs(*report);
  for (const cell::CollidingPair& p : report->pairs) {
    EXPECT_TRUE(IsLink(model(), p.body2)) << p.between;
  }
}

TEST_F(CollisionTest, ToolIntoTheRails) {
  CollisionChecker checker = MakeChecker();
  // Tool straight down, plate along rail_b00 (slot 0's +Y rail, along
  // world X at y = 0.4), cups 20 mm below its top.
  const Eigen::Isometry3d target =
      *cell::GripPose(Eigen::Vector3d(-2.0, 0.4, 0.78), Eigen::Vector3d::UnitZ(),
                      Eigen::Vector3d::UnitX());
  const Eigen::VectorXd q = Solve(target, Waypoints(Step::kSetDown).back());
  std::optional<CollisionReport> report = checker.Check(q, {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_FALSE(report->free);
  EXPECT_EQ(report->pairs[0].body1, cell().arm().tool_body) << Pairs(*report);
  EXPECT_EQ(GeomName(model(), report->pairs[0].geom2), "rail_b00")
      << Pairs(*report);
  for (const cell::CollidingPair& p : report->pairs) {
    EXPECT_EQ(BodyName(model(), p.body2), "tube") << p.between;
  }
  // The tube run on to near the end of its travel: nothing of it is left
  // at the placement spot, and the bed is 15 cm below the cups.
  ASSERT_TRUE(checker.SetTubeSlide(-11.0));
  EXPECT_TRUE(checker.IsFree(q)) << Pairs(*checker.Check(q, {.all_pairs = true}));
}

TEST_F(CollisionTest, HeldModuleIntoTheCameraPost) {
  CollisionChecker checker = MakeChecker();
  // Stretched out level toward the post on -Y, tool down, the module's long
  // side pointing at the post: its far end is 3.85 m out, the post 3.725 m.
  const Eigen::VectorXd q = Degrees(90, -5, 0, 0, 90, 0);
  EXPECT_TRUE(checker.IsFree(q)) << "bare arm"
                                 << Pairs(*checker.Check(q, {.all_pairs = true}));
  ASSERT_TRUE(checker.SetHeldModule(0, grip_));
  std::optional<CollisionReport> report = checker.Check(q, {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_FALSE(report->free);
  EXPECT_EQ(report->pairs[0].body1, cell().pallet().modules[0]) << Pairs(*report);
  EXPECT_EQ(GeomName(model(), report->pairs[0].geom2), "camera_post_column")
      << Pairs(*report);
}

TEST_F(CollisionTest, HeldModuleCornerIntoTheFloor) {
  CollisionChecker checker = MakeChecker();
  // Over the open floor on +Y, the tip 0.7 m up, the tool tilted 40
  // degrees about its Y: the module's end dips 0.74 m, the plate's 0.39 m.
  Eigen::Isometry3d target =
      *cell::GripPose(Eigen::Vector3d(0.3, 2.0, 0.7), Eigen::Vector3d::UnitZ(),
                      Eigen::Vector3d::UnitX());
  target.linear() =
      target.linear() * Eigen::AngleAxisd(40 * kDegree, Eigen::Vector3d::UnitY());
  Eigen::VectorXd seed = Waypoints(Step::kFlatten).back();
  seed[0] = -kPi / 2;
  const Eigen::VectorXd q = Solve(target, seed);

  EXPECT_TRUE(checker.IsFree(q)) << "bare arm"
                                 << Pairs(*checker.Check(q, {.all_pairs = true}));
  ASSERT_TRUE(checker.SetHeldModule(0, grip_));
  std::optional<CollisionReport> report = checker.Check(q, {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_FALSE(report->free);
  ASSERT_EQ(report->pairs.size(), 1u) << Pairs(*report);
  EXPECT_EQ(report->pairs[0].body1, cell().pallet().modules[0]);
  EXPECT_EQ(GeomName(model(), report->pairs[0].geom2), "floor");
}

TEST_F(CollisionTest, HeldModuleIntoTheStackWhileTheBareArmIsFree) {
  CollisionChecker checker = MakeChecker();
  // Flat over the front of the stack, glass up, long side along the
  // pallet, the tip only 1.0 m up: the module's underside is below the
  // stack's top edges (1.29 m) and reaches over them.
  const Eigen::Isometry3d target =
      *cell::GripPose(Eigen::Vector3d(1.6, 0.0, 1.0), Eigen::Vector3d::UnitZ(),
                      -Eigen::Vector3d::UnitY());
  const Eigen::VectorXd q = Solve(target, Waypoints(Step::kFlatten).back());

  std::optional<CollisionReport> bare = checker.Check(q, {.clearance = true});
  ASSERT_TRUE(bare);
  EXPECT_TRUE(bare->free) << Pairs(*bare);
  std::printf("bare arm over the stack: clearance %.1f mm (%s)\n",
              1e3 * *bare->clearance, bare->clearance_between.c_str());

  ASSERT_TRUE(checker.SetHeldModule(0, grip_));
  std::optional<CollisionReport> report = checker.Check(q, {.all_pairs = true});
  ASSERT_TRUE(report);
  ASSERT_FALSE(report->free);
  for (const cell::CollidingPair& p : report->pairs) {
    EXPECT_EQ(p.body1, cell().pallet().modules[0]) << p.between;
    const std::string other = BodyName(model(), p.body2);
    EXPECT_TRUE(other.rfind("module_", 0) == 0 || other == "pallet") << p.between;
  }
  // Let go again: the module is back in the stack and the bare arm free.
  checker.ClearHeldModule();
  EXPECT_TRUE(checker.IsFree(q));
}

// ----------------------------------------------------- the scripted cycle

// The script's waypoints at nominal, each leg checked as the straight
// joint-space segment the follower moves along, with what each step is
// meant to touch allowed: bare arm up to the touch (the cups on module_0's
// glass allowed there), module_0 held from the lift (its neighbour and the
// deck allowed while it leaves the stack), the rails allowed at the
// set-down, and module_0 lying on the rails afterwards (the cups on it
// allowed while retreating).
TEST_F(CollisionTest, ScriptedCycleIsFreeWithItsAllowances) {
  CollisionChecker checker = MakeChecker();
  const double resolution = 0.01;  // m, as the planner's default
  struct Phase {
    Step step;
    bool held;
    std::vector<std::pair<std::string, std::string>> allowed;
  };
  const std::vector<Phase> phases = {
      {Step::kToStandoff, false, {}},
      {Step::kApproach, false, {}},
      {Step::kTouchGlass, false, {{"tool", "module_0"}}},
      {Step::kLift, true, {{"module_0", "module_1"}, {"module_0", "pallet"}}},
      {Step::kPullBack, true, {{"module_0", "module_1"}, {"module_0", "pallet"}}},
      {Step::kClear, true, {}},
      {Step::kFlatten, true, {}},
      {Step::kSwing, true, {}},
      {Step::kTurn, true, {}},
      {Step::kLower, true, {}},
      {Step::kSetDown, true, {{"module_0", "tube"}}},
      {Step::kRetreat, false, {{"tool", "module_0"}}},
      {Step::kReturn, false, {}},
  };
  double world = 1.0, self = 1.0;
  std::string world_where, self_where;
  Eigen::VectorXd at = cell().arm_home();
  int samples = 0;
  for (const Phase& phase : phases) {
    if (phase.held && !checker.held_module()) {
      ASSERT_TRUE(checker.SetHeldModule(0, grip_));
    }
    if (!phase.held && checker.held_module()) {
      // Let go where the set-down left it: the last query put it there.
      const Eigen::Isometry3d placed =
          BodyPose(checker.data(), cell().pallet().modules[0]);
      checker.ClearHeldModule();
      ASSERT_TRUE(checker.SetModulePose(0, placed));
    }
    checker.ClearAllowedContacts();
    for (const auto& [a, b] : phase.allowed) ASSERT_TRUE(checker.AllowContact(a, b));

    double phase_world = 1.0, phase_self = 1.0;
    std::string phase_where;
    for (const Eigen::VectorXd& to : Waypoints(phase.step)) {
      cell::SegmentReport segment;
      const double step = checker.SegmentStep(at, to, resolution);
      EXPECT_TRUE(checker.SegmentIsFree(at, to, step, &segment))
          << cell::StepName(phase.step) << ": hits at "
          << segment.hit_fraction << Pairs(segment.hit_report);
      samples += segment.samples;
      // The clearances at the same samples.
      const int n = std::max(1, segment.samples - 1);
      for (int k = 0; k <= n; ++k) {
        const Eigen::VectorXd q = at + (to - at) * (static_cast<double>(k) / n);
        std::optional<CollisionReport> r = checker.Check(q, {.clearance = true});
        if (*r->clearance < phase_world) {
          phase_world = *r->clearance;
          phase_where = r->clearance_between;
        }
        if (*r->clearance < world) {
          world = *r->clearance;
          world_where = std::string(cell::StepName(phase.step)) + ": " +
                        r->clearance_between;
        }
        phase_self = std::min(phase_self, *r->self_clearance);
        if (*r->self_clearance < self) {
          self = *r->self_clearance;
          self_where = std::string(cell::StepName(phase.step)) + ": " +
                       r->self_clearance_between;
        }
      }
      at = to;
    }
    std::printf("  %-12s %s  clearance %6.1f mm (%s), self %6.1f mm\n",
                cell::StepName(phase.step), phase.held ? "held" : "bare",
                1e3 * phase_world, phase_where.c_str(), 1e3 * phase_self);
  }
  std::printf("scripted cycle, %d samples at %.0f mm: smallest clearance "
              "%.1f mm (%s), self %.1f mm (%s)\n",
              samples, 1e3 * resolution, 1e3 * world, world_where.c_str(),
              1e3 * self, self_where.c_str());
  EXPECT_GE(world, checker.margin());
}

// The nominal cycle simulated, checked by cycle_check from the contacts the
// physics made, and by the checker (no margin, the truth for the scene and
// the held pose) every 10 steps with what cycle_check allows. Both must
// find nothing.
TEST_F(CollisionTest, SimulatedCycleAgreesWithCycleCheck) {
  CollisionChecker checker = MakeChecker({.margin = 0.0});
  PickPlace script(cell());
  cell::CycleCheck check(cell(), {.clearance_every = 0});
  ASSERT_TRUE(script.Start(0));
  const int module_0 = cell().pallet().modules[0];
  const int tool = cell().arm().tool_body;
  auto between = [](Step s, Step first, Step last) {
    return static_cast<int>(s) >= static_cast<int>(first) &&
           static_cast<int>(s) <= static_cast<int>(last);
  };
  int queries = 0, collisions = 0;
  std::string first_collision;
  const double until = cell().data()->time + 60.0;
  for (long n = 1; !script.Done() && cell().data()->time < until; ++n) {
    script.Update();
    cell().Step();
    const Step step = script.step();
    check.Observe(step, 0);
    if (n % 10 != 0) continue;

    checker.SetScene(*cell().data());
    if (std::optional<cell::Pose> held =
            cell().suction().HeldPoseInTip(*cell().data())) {
      checker.SetHeldModule(0, FromPose(*held));
    } else {
      checker.ClearHeldModule();
    }
    checker.ClearAllowedContacts();
    if (between(step, Step::kTouchGlass, Step::kRetreat)) {
      checker.AllowContact(tool, module_0);
    }
    if (static_cast<int>(step) <= static_cast<int>(Step::kPullBack)) {
      checker.AllowContact(module_0, cell().pallet().pallet);
      for (int k = 1; k < cell().module_count(); ++k) {
        checker.AllowContact(module_0, cell().pallet().modules[k]);
      }
    }
    if (between(step, Step::kSetDown, Step::kDone)) {
      checker.AllowContact("module_0", "tube");
    }
    ++queries;
    std::optional<CollisionReport> report =
        checker.Check(cell::JointAngles(cell().arm(), *cell().data()));
    if (!report->free && collisions++ == 0) {
      first_collision = std::string(cell::StepName(step)) + Pairs(*report);
    }
  }
  ASSERT_EQ(script.outcome().status, cell::PickPlaceOutcome::Status::kPlaced)
      << script.outcome().reason;
  EXPECT_EQ(check.forbidden_count(), 0);
  EXPECT_EQ(collisions, 0) << "first in " << first_collision;
  std::printf("simulated cycle: %d checks, %d collisions; cycle_check: %ld "
              "forbidden contacts\n",
              queries, collisions, check.forbidden_count());
}

// ------------------------------------------------------------------ margin

TEST_F(CollisionTest, MarginFlipsAtTheMeasuredClearance) {
  // The end of the approach: the cups 30 mm in front of module_0's glass.
  const Eigen::VectorXd q = Waypoints(Step::kApproach).back();

  // Measured independently: the cell's own model, a fresh state, every
  // geom of the arm and tool against every other colliding geom.
  cell::DataPtr data = cell::MakeData(model());
  mju_copy(data->qpos, cell().data()->qpos, model().nq);
  mju_copy(data->mocap_pos, cell().data()->mocap_pos, 3 * model().nmocap);
  mju_copy(data->mocap_quat, cell().data()->mocap_quat, 4 * model().nmocap);
  for (int j = 0; j < 6; ++j) data->qpos[cell().arm().qpos[j]] = q[j];
  mj_kinematics(&model(), data.get());
  const int base = mj_name2id(&model(), mjOBJ_BODY, "arm_base");
  auto arm_geom = [&](int g) {
    for (int b = model().geom_bodyid[g]; b > 0; b = model().body_parentid[b]) {
      if (b == base) return true;
    }
    return false;
  };
  double clearance = 1.0;
  int nearest = -1;
  for (int g = 0; g < model().ngeom; ++g) {
    if (!arm_geom(g) || model().geom_contype[g] == 0) continue;
    for (int h = 0; h < model().ngeom; ++h) {
      if (arm_geom(h) || model().geom_contype[h] == 0) continue;
      // The base stands on the floor; MuJoCo never pairs the two.
      if (model().geom_bodyid[g] == base && GeomName(model(), h) == "floor") {
        continue;
      }
      const double d = mj_geomDistance(&model(), data.get(), g, h, 1.0, nullptr);
      if (d < clearance) {
        clearance = d;
        nearest = h;
      }
    }
  }
  ASSERT_GE(nearest, 0);
  ASSERT_EQ(model().geom_bodyid[nearest], cell().pallet().modules[0]);
  std::printf("approach end: clearance %.4f mm to module_0\n", 1e3 * clearance);

  CollisionChecker checker = MakeChecker();
  for (double off : {1e-3, 1e-4}) {
    ASSERT_TRUE(checker.SetMargins(clearance - off, 0.0));
    EXPECT_TRUE(checker.IsFree(q)) << "margin " << clearance - off;
    ASSERT_TRUE(checker.SetMargins(clearance + off, 0.0));
    std::optional<CollisionReport> report = checker.Check(q, {.all_pairs = true});
    ASSERT_FALSE(report->free) << "margin " << clearance + off;
    EXPECT_EQ(report->pairs[0].body1, cell().arm().tool_body);
    EXPECT_EQ(report->pairs[0].body2, cell().pallet().modules[0]);
    EXPECT_NEAR(report->pairs[0].distance, clearance, 1e-6);
  }
  // Check's own clearance agrees.
  std::optional<CollisionReport> report = checker.Check(q, {.clearance = true});
  EXPECT_NEAR(*report->clearance, clearance, 1e-9);
}

// ----------------------------------------------------------------- segments

TEST_F(CollisionTest, SegmentFindsThePostBetweenFreeEnds) {
  CollisionChecker checker = MakeChecker({.margin = 0.0});
  ASSERT_TRUE(checker.SetHeldModule(0, grip_));
  // Stretched out toward -Y with the module's long side radial (as in
  // HeldModuleIntoTheCameraPost), joint 1 swung from 17 degrees short of
  // the post to 17 degrees past it.
  const Eigen::VectorXd from = Degrees(73, -5, 0, 0, 90, 0);
  const Eigen::VectorXd to = Degrees(107, -5, 0, 0, 90, 0);
  ASSERT_TRUE(checker.IsFree(from));
  ASSERT_TRUE(checker.IsFree(to));

  cell::SegmentReport report;
  const double step = checker.SegmentStep(from, to, 0.005);
  EXPECT_FALSE(checker.SegmentIsFree(from, to, step, &report));
  ASSERT_FALSE(report.hit_report.pairs.empty());
  EXPECT_EQ(report.hit_report.pairs[0].body1, cell().pallet().modules[0]);
  EXPECT_EQ(GeomName(model(), report.hit_report.pairs[0].geom2),
            "camera_post_column");
  ASSERT_GE(report.free_fraction, 0.0);
  const Eigen::VectorXd last_free = from + report.free_fraction * (to - from);
  EXPECT_TRUE(checker.IsFree(last_free));

  // Where it should be: the module's leading long edge, half its width from
  // its centre line (which runs out through joint 1's axis), meets the
  // post's nearest corner (x = 0.075 m, y = -3.725 m). Joint 1 turns from
  // +X toward -Y.
  const double corner_range = std::hypot(0.075, 3.725);
  const double corner_bearing = std::atan2(3.725, 0.075);
  const double expected =
      corner_bearing - std::asin(0.5 * cell::kModuleWidth / corner_range);
  std::printf("segment past the post: step %.2f mrad, hit at sample %d, "
              "joint 1 = %.2f deg, expected %.2f deg\n",
              1e3 * step, report.samples, report.hit[0] / kDegree,
              expected / kDegree);
  // Within a step (1 mrad, 0.06 degrees) and a little for the module not
  // hanging exactly radially.
  EXPECT_NEAR(report.hit[0], expected, 0.15 * kDegree);
}

// ------------------------------------------------- fast against plain model

// The private model with obstacle pairs switched off against an unmodified
// copy (plain), over random configurations across the joint ranges and
// around the scripted path, bare and with module_0 held, at no margin and
// at the default margin. Every answer, every colliding pair and its
// distance must be the same.
TEST_F(CollisionTest, FastModelGivesThePlainModelsAnswers) {
  std::mt19937 rng(17);
  std::vector<Eigen::VectorXd> configurations;
  const cell::Arm& arm = cell().arm();
  for (int i = 0; i < 1000; ++i) {
    Eigen::VectorXd q(6);
    for (int j = 0; j < 6; ++j) {
      q[j] = std::uniform_real_distribution<double>(arm.lower[j], arm.upper[j])(rng);
    }
    configurations.push_back(q);
  }
  std::vector<Eigen::VectorXd> path;
  for (Step s : {Step::kToStandoff, Step::kApproach, Step::kTouchGlass,
                 Step::kLift, Step::kPullBack, Step::kClear, Step::kFlatten,
                 Step::kSwing, Step::kTurn, Step::kLower, Step::kSetDown}) {
    for (const Eigen::VectorXd& q : Waypoints(s)) path.push_back(q);
  }
  for (int i = 0; i < 1000; ++i) {
    Eigen::VectorXd q = path[i % path.size()];
    for (int j = 0; j < 6; ++j) {
      q[j] = std::clamp(q[j] + std::normal_distribution<double>(0.0, 0.15)(rng),
                        arm.lower[j], arm.upper[j]);
    }
    configurations.push_back(q);
  }

  for (double margin : {0.0, 0.02}) {
    for (bool held : {false, true}) {
      CollisionChecker fast = MakeChecker({.margin = margin});
      CollisionChecker plain =
          MakeChecker({.margin = margin, .skip_obstacle_pairs = false});
      if (held) {
        ASSERT_TRUE(fast.SetHeldModule(0, grip_));
        ASSERT_TRUE(plain.SetHeldModule(0, grip_));
      }
      int colliding = 0, differ = 0;
      long fast_contacts = 0, plain_contacts = 0;
      for (const Eigen::VectorXd& q : configurations) {
        const CollisionReport a = *fast.Check(q, {.all_pairs = true});
        fast_contacts += fast.data().ncon;
        const CollisionReport b = *plain.Check(q, {.all_pairs = true});
        plain_contacts += plain.data().ncon;
        colliding += !a.free;
        bool same = a.free == b.free && a.pairs.size() == b.pairs.size();
        for (size_t k = 0; same && k < a.pairs.size(); ++k) {
          same = a.pairs[k].geom1 == b.pairs[k].geom1 &&
                 a.pairs[k].geom2 == b.pairs[k].geom2 &&
                 a.pairs[k].distance == b.pairs[k].distance;
        }
        if (!same && differ++ < 3) {
          ADD_FAILURE() << "q " << q.transpose() << "\nfast:" << Pairs(a)
                        << "\nplain:" << Pairs(b);
        }
        EXPECT_EQ(fast.IsFree(q), a.free);
      }
      EXPECT_EQ(differ, 0);
      std::printf("margin %2.0f mm, %s: %zu configurations, %d colliding, "
                  "%d differ; contacts MuJoCo made per query: fast %.1f, "
                  "plain %.1f\n",
                  1e3 * margin, held ? "held" : "bare", configurations.size(),
                  colliding, differ,
                  static_cast<double>(fast_contacts) / configurations.size(),
                  static_cast<double>(plain_contacts) / configurations.size());
    }
  }
}

// --------------------------------------------------------- obstacle updates

TEST_F(CollisionTest, ObstacleUpdates) {
  // A small margin, so that module_1's glass, 35 mm behind module_0's,
  // stays clear of a tool pushed 15 mm into module_0.
  CollisionChecker checker = MakeChecker({.margin = 0.005});
  Eigen::Isometry3d target = script_->grip_tip();
  target.translation() += 0.015 * target.linear().col(2);
  const Eigen::VectorXd q = Solve(target, Waypoints(Step::kTouchGlass).back());
  std::optional<CollisionReport> report = checker.Check(q, {.all_pairs = true});
  ASSERT_FALSE(report->free);
  EXPECT_EQ(report->pairs[0].body2, cell().pallet().modules[0]) << Pairs(*report);

  // module_0 carried away; the planner is told by a new scene.
  const Eigen::Isometry3d where = cell().TrueModulePose(0);
  cell().ParkModule(0);
  EXPECT_FALSE(checker.IsFree(q)) << "the checker must not see the change "
                                     "before it is told";
  checker.SetScene(*cell().data());
  report = checker.Check(q, {.all_pairs = true, .clearance = true});
  EXPECT_TRUE(report->free) << Pairs(*report);
  std::printf("module_0 parked: clearance %.1f mm (%s)\n",
              1e3 * *report->clearance, report->clearance_between.c_str());

  // An estimate put back explicitly.
  ASSERT_TRUE(checker.SetModulePose(0, where));
  EXPECT_FALSE(checker.IsFree(q));

  // The pallet with its whole stack estimated 0.5 m further away: free.
  Eigen::Isometry3d pallet = Eigen::Isometry3d::Identity();
  pallet.translation() = cell().pallet().nominal_pallet.position;
  pallet.linear() = cell().pallet().nominal_pallet.orientation.toRotationMatrix();
  pallet.translation().x() += 0.5;
  ASSERT_TRUE(checker.SetPalletPose(pallet));
  for (int i = 1; i < cell().module_count(); ++i) {
    Eigen::Isometry3d m = cell().TrueModulePose(i);
    m.translation().x() += 0.5;
    ASSERT_TRUE(checker.SetModulePose(i, m));
  }
  EXPECT_FALSE(checker.IsFree(q)) << "module_0 was left where it stood";
  Eigen::Isometry3d moved = where;
  moved.translation().x() += 0.5;
  ASSERT_TRUE(checker.SetModulePose(0, moved));
  EXPECT_TRUE(checker.IsFree(q)) << Pairs(*checker.Check(q, {.all_pairs = true}));
}

// ------------------------------------------------------------- the details

// The held module is placed by hand, body and geom only, instead of a
// second mj_kinematics; it must land exactly where mj_kinematics puts it,
// and where the grip says.
TEST_F(CollisionTest, HeldModuleIsPlacedAsKinematicsPlacesIt) {
  CollisionChecker checker = MakeChecker();
  ASSERT_TRUE(checker.SetHeldModule(3, grip_));
  checker.Check(Waypoints(Step::kSwing).back());
  const mjModel& m = checker.model();
  cell::DataPtr data = cell::MakeData(m);
  mju_copy(data->qpos, checker.data().qpos, m.nq);
  mju_copy(data->mocap_pos, checker.data().mocap_pos, 3 * m.nmocap);
  mju_copy(data->mocap_quat, checker.data().mocap_quat, 4 * m.nmocap);
  mj_kinematics(&m, data.get());
  const int body = cell().pallet().modules[3];
  const int geom = m.body_geomadr[body];
  const mjData& d = checker.data();
  for (int k = 0; k < 3; ++k) {
    EXPECT_NEAR(d.xpos[3 * body + k], data->xpos[3 * body + k], 1e-12);
    EXPECT_NEAR(d.xipos[3 * body + k], data->xipos[3 * body + k], 1e-12);
    EXPECT_NEAR(d.geom_xpos[3 * geom + k], data->geom_xpos[3 * geom + k], 1e-12);
  }
  for (int k = 0; k < 9; ++k) {
    EXPECT_NEAR(d.xmat[9 * body + k], data->xmat[9 * body + k], 1e-12);
    EXPECT_NEAR(d.ximat[9 * body + k], data->ximat[9 * body + k], 1e-12);
    EXPECT_NEAR(d.geom_xmat[9 * geom + k], data->geom_xmat[9 * geom + k], 1e-12);
  }
  const Eigen::Isometry3d expected = cell::TipPose(cell().arm(), *data) * grip_;
  EXPECT_LT((expected.translation() - BodyPose(d, body).translation()).norm(),
            1e-12);
}

// No point of the arm, the tool or the held module moves further along a
// straight segment than SweepBound says. Measured on the corners of the
// tool's plate and of the held module, which are the farthest points.
TEST_F(CollisionTest, LeverArmsBoundHowFarPointsMove) {
  CollisionChecker checker = MakeChecker();
  ASSERT_TRUE(checker.SetHeldModule(0, grip_));
  const mjModel& m = checker.model();
  const int plate = mj_name2id(&m, mjOBJ_GEOM, "tool_plate");
  const int module = m.body_geomadr[cell().pallet().modules[0]];
  auto corners = [&](int g, std::vector<Eigen::Vector3d>* points) {
    const mjData& d = checker.data();
    const Eigen::Vector3d centre(d.geom_xpos[3 * g], d.geom_xpos[3 * g + 1],
                                 d.geom_xpos[3 * g + 2]);
    const Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> axes(
        d.geom_xmat + 9 * g);
    const Eigen::Vector3d size(m.geom_size[3 * g], m.geom_size[3 * g + 1],
                               m.geom_size[3 * g + 2]);
    for (int i = 0; i < 8; ++i) {
      const Eigen::Vector3d sign(i & 1 ? 1 : -1, i & 2 ? 1 : -1, i & 4 ? 1 : -1);
      points->push_back(centre + axes * sign.cwiseProduct(size));
    }
  };
  std::mt19937 rng(23);
  const cell::Arm& arm = cell().arm();
  double worst = 0.0;  // the largest distance moved over its bound
  for (int i = 0; i < 100; ++i) {
    Eigen::VectorXd from(6), to(6);
    for (int j = 0; j < 6; ++j) {
      from[j] = std::uniform_real_distribution<double>(arm.lower[j], arm.upper[j])(rng);
      to[j] = std::clamp(
          from[j] + std::uniform_real_distribution<double>(-0.3, 0.3)(rng),
          arm.lower[j], arm.upper[j]);
    }
    // The path length of each corner, summed over fine steps.
    std::vector<Eigen::Vector3d> previous;
    std::vector<double> travelled(16, 0.0);
    for (int k = 0; k <= 100; ++k) {
      checker.Check(from + (to - from) * (k / 100.0));
      std::vector<Eigen::Vector3d> now;
      corners(plate, &now);
      corners(module, &now);
      for (size_t c = 0; k > 0 && c < now.size(); ++c) {
        travelled[c] += (now[c] - previous[c]).norm();
      }
      previous = now;
    }
    const double bound = checker.SweepBound(from, to);
    for (double t : travelled) {
      EXPECT_LE(t, bound);
      worst = std::max(worst, t / bound);
    }
  }
  const CollisionChecker bare_checker = MakeChecker();
  const Eigen::VectorXd bare = bare_checker.LeverArms();
  const Eigen::VectorXd held = checker.LeverArms();
  std::printf("lever arms, m/rad: bare %.2f %.2f %.2f %.2f %.2f %.2f; held "
              "%.2f %.2f %.2f %.2f %.2f %.2f; the farthest corner moved %.0f%% "
              "of the bound\n",
              bare[0], bare[1], bare[2], bare[3], bare[4], bare[5], held[0],
              held[1], held[2], held[3], held[4], held[5], 100 * worst);
  std::printf("suggested step for 10 mm: bare %.2f mrad, held %.2f mrad\n",
              1e3 * bare_checker.SuggestStep(0.01), 1e3 * checker.SuggestStep(0.01));
  EXPECT_NEAR(checker.SuggestStep(0.01), 0.01 / held.sum(), 1e-15);
}

// --------------------------------------------------------------------- time

// Microseconds per query; read them from the optimized build. Two sets of
// configurations: uniform across the joint ranges (most collide with
// something) and the scripted path at 1 cm (all free, many near the
// stack): its pick with the bare arm, its carry with the module held.
TEST_F(CollisionTest, TimePerQuery) {
  std::mt19937 rng(29);
  std::vector<Eigen::VectorXd> uniform;
  const cell::Arm& arm = cell().arm();
  for (int i = 0; i < 2000; ++i) {
    Eigen::VectorXd q(6);
    for (int j = 0; j < 6; ++j) {
      q[j] = std::uniform_real_distribution<double>(arm.lower[j], arm.upper[j])(rng);
    }
    uniform.push_back(q);
  }
  std::vector<Eigen::VectorXd> pick, carry;
  {
    CollisionChecker checker = MakeChecker();
    Eigen::VectorXd at = cell().arm_home();
    for (Step s : {Step::kToStandoff, Step::kApproach, Step::kTouchGlass,
                   Step::kLift, Step::kPullBack, Step::kClear, Step::kFlatten,
                   Step::kSwing, Step::kTurn, Step::kLower}) {
      std::vector<Eigen::VectorXd>& list =
          static_cast<int>(s) <= static_cast<int>(Step::kTouchGlass) ? pick : carry;
      for (const Eigen::VectorXd& to : Waypoints(s)) {
        const double step = checker.SegmentStep(at, to, 0.01);
        const int n = std::max(
            1, static_cast<int>(std::ceil((to - at).cwiseAbs().maxCoeff() / step)));
        for (int k = 0; k < n; ++k) {
          list.push_back(at + (to - at) * (static_cast<double>(k) / n));
        }
        at = to;
      }
    }
  }
  auto time = [&](CollisionChecker& checker,
                  const std::vector<Eigen::VectorXd>& qs,
                  const CheckOptions* options) {
    int free = 0;
    const auto start = std::chrono::steady_clock::now();
    for (int repeat = 0; repeat < 3; ++repeat) {
      for (const Eigen::VectorXd& q : qs) {
        free += options != nullptr ? checker.Check(q, *options)->free
                                   : checker.IsFree(q);
      }
    }
    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - start)
                          .count();
    return std::make_pair(us / (3.0 * qs.size()), free / 3);
  };
  const CheckOptions clearance{.clearance = true};
  for (bool skip : {true, false}) {
    for (bool held : {false, true}) {
      CollisionChecker checker = MakeChecker({.skip_obstacle_pairs = skip});
      if (held) {
        ASSERT_TRUE(checker.SetHeldModule(0, grip_));
      }
      const std::vector<Eigen::VectorXd>& path = held ? carry : pick;
      const auto [u, uf] = time(checker, uniform, nullptr);
      const auto [p, pf] = time(checker, path, nullptr);
      const auto [c, cf] = time(checker, path, &clearance);
      std::printf("%s, %s: IsFree %6.1f us uniform (%4d of %zu free), %6.1f us "
                  "along the %s (%d of %zu free); Check with clearance %7.1f us\n",
                  skip ? "fast " : "plain", held ? "held" : "bare", u, uf,
                  uniform.size(), p, held ? "carry" : "pick ", pf, path.size(), c);
    }
  }
  // What the kinematics alone costs, on the same model.
  CollisionChecker checker = MakeChecker();
  cell::DataPtr data = cell::MakeData(checker.model());
  mju_copy(data->qpos, checker.data().qpos, checker.model().nq);
  const auto start = std::chrono::steady_clock::now();
  for (int repeat = 0; repeat < 3; ++repeat) {
    for (const Eigen::VectorXd& q : uniform) {
      for (int j = 0; j < 6; ++j) data->qpos[arm.qpos[j]] = q[j];
      mj_kinematics(&checker.model(), data.get());
    }
  }
  std::printf("mj_kinematics alone: %.1f us\n",
              std::chrono::duration<double, std::micro>(
                  std::chrono::steady_clock::now() - start)
                      .count() /
                  (3.0 * uniform.size()));
}

}  // namespace
