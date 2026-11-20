// Checks the conveyor: the model has the parts and names the other code
// relies on, keeps clear of the arm, and agrees with the layout in
// src/conveyor.h; a step moves the tube by one step plus the injected
// stopping error, and stops without overshoot; the move stays within its
// limits; modules on the rails ride along without sliding; stopping errors
// do not add up from step to step; a full tube can be swapped for an empty
// one; and the same seed repeats the same run.
//
//   bazel test //:conveyor_test
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test. The tests print what they measured.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/conveyor.h"
#include "src/mj_handles.h"

namespace {

using cell::Conveyor;
using cell::ConveyorSettings;

constexpr int kTestModules = 3;

// What one step looked like, measured every physics step.
struct StepRecord {
  double duration = 0.0;      // from StartStep until Moving() turned false
  double peak_speed = 0.0;    // |slide velocity|, m/s
  double peak_acceleration = 0.0;  // |slide acceleration|, m/s^2
  double overshoot = 0.0;     // furthest past the stop, in the direction of
                              // travel, m; 0 if never past
  double peak_slip = 0.0;     // furthest any test module got from where it
                              // was on the tube at the start of the step, m
  double peak_module_acceleration = 0.0;  // of any test module along Y
};

class ConveyorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";
    path_ = path;

    std::string error;
    model_ = cell::LoadModel(path_, &error);
    ASSERT_TRUE(model_) << "could not load " << path_ << ": " << error;
    data_ = cell::MakeData(*model_);
    ASSERT_TRUE(data_);
    mj_forward(model_.get(), data_.get());

    for (int i = 0; i < kTestModules; ++i) {
      const std::string name = "test_module_" + std::to_string(i);
      modules_.push_back(mj_name2id(model_.get(), mjOBJ_BODY, name.c_str()));
      ASSERT_GE(modules_.back(), 0) << name;
    }
  }

  // A conveyor for this model; fails the test if it cannot be found.
  Conveyor MakeConveyor(const ConveyorSettings& settings = {},
                        unsigned seed = 1) {
    std::optional<Conveyor> conveyor =
        Conveyor::Find(*model_, settings, seed);
    EXPECT_TRUE(conveyor.has_value());
    return *conveyor;
  }

  // One control tick: setpoint, then physics.
  void Tick(const Conveyor& conveyor) {
    conveyor.Update(data_.get());
    mj_step(model_.get(), data_.get());
  }

  void RunFor(const Conveyor& conveyor, double seconds) {
    const double end = data_->time + seconds;
    while (data_->time < end) Tick(conveyor);
  }

  // Lets the modules come to rest on the rails before measuring anything.
  void Settle(const Conveyor& conveyor) { RunFor(conveyor, 0.5); }

  // Test module i's centre in the tube's frame.
  Eigen::Vector3d ModuleOnTube(const Conveyor& conveyor, int i) const {
    const Eigen::Vector3d world =
        Eigen::Map<const Eigen::Vector3d>(data_->xpos + 3 * modules_[i]);
    return conveyor.TrueTubePose(*data_).inverse() * world;
  }

  // The velocity coordinate of test module i's motion along world Y. A free
  // joint's first three velocity coordinates are its linear velocity in the
  // world frame.
  int ModuleYDof(int i) const { return model_->body_dofadr[modules_[i]] + 1; }

  // Starts a step and runs until the conveyor says it has stopped.
  StepRecord RunStep(Conveyor& conveyor) {
    StepRecord record;
    const double start_time = data_->time;
    std::vector<Eigen::Vector3d> start_on_tube;
    for (int i = 0; i < kTestModules; ++i) {
      start_on_tube.push_back(ModuleOnTube(conveyor, i));
    }

    EXPECT_TRUE(conveyor.StartStep(*data_));
    const double stop = conveyor.true_stop_position();
    const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, "conveyor_slide");
    const int dof = model_->jnt_dofadr[joint];

    const double timeout = 3.0 * conveyor.StepDuration() + 5.0;
    while (conveyor.Moving(*data_) && data_->time < start_time + timeout) {
      // Acceleration as the change in velocity over the step. Not qacc: with
      // the implicit integrator, qacc is the acceleration before the servo's
      // damping is accounted for implicitly, and reads higher than what the
      // state actually does.
      const double dt = model_->opt.timestep;
      const double speed_before = data_->qvel[dof];
      std::vector<double> module_speed_before;
      for (int i = 0; i < kTestModules; ++i) {
        module_speed_before.push_back(data_->qvel[ModuleYDof(i)]);
      }
      Tick(conveyor);
      record.peak_speed =
          std::max(record.peak_speed, std::abs(data_->qvel[dof]));
      record.peak_acceleration =
          std::max(record.peak_acceleration,
                   std::abs(data_->qvel[dof] - speed_before) / dt);
      for (int i = 0; i < kTestModules; ++i) {
        record.peak_module_acceleration = std::max(
            record.peak_module_acceleration,
            std::abs(data_->qvel[ModuleYDof(i)] - module_speed_before[i]) / dt);
      }
      const double past = Conveyor::kTravelDirection *
                          (conveyor.TrueSlidePosition(*data_) - stop);
      record.overshoot = std::max(record.overshoot, past);
      for (int i = 0; i < kTestModules; ++i) {
        const double slip =
            (ModuleOnTube(conveyor, i) - start_on_tube[i]).norm();
        record.peak_slip = std::max(record.peak_slip, slip);
      }
    }
    record.duration = data_->time - start_time;
    EXPECT_FALSE(conveyor.Moving(*data_)) << "did not stop within " << timeout
                                          << " s";
    return record;
  }

  // Moves the test modules off the tube onto the floor, as the cell does
  // before a tube swap.
  void CarryModulesAway() {
    for (int i = 0; i < kTestModules; ++i) {
      const int joint = model_->body_jntadr[modules_[i]];
      double* qpos = data_->qpos + model_->jnt_qposadr[joint];
      const double parked[7] = {3.0, 3.0 * i, Conveyor::kModuleThickness / 2,
                                1.0, 0.0, 0.0, 0.0};
      std::copy(parked, parked + 7, qpos);
      std::fill_n(data_->qvel + model_->jnt_dofadr[joint], 6, 0.0);
    }
    mj_forward(model_.get(), data_.get());
  }

  struct FullTube {
    double worst_error = 0.0;  // furthest any stop was from nominal, m
    double peak_slip = 0.0;    // largest StepRecord::peak_slip
  };

  // Steps until the tube is full, letting the servo finish each stop, and
  // checks that every stop is its nominal position plus its own error.
  FullTube FillTube(Conveyor& conveyor) {
    FullTube run;
    for (int n = conveyor.steps(); n < Conveyor::kSlotCount; ++n) {
      const StepRecord record = RunStep(conveyor);
      RunFor(conveyor, 0.5);
      const double nominal = Conveyor::NominalSlidePosition(n + 1);
      EXPECT_EQ(conveyor.nominal_position(), nominal);
      EXPECT_NEAR(conveyor.TrueSlidePosition(*data_),
                  nominal + conveyor.true_last_stop_error(), 1e-4)
          << "step " << n + 1;
      EXPECT_NEAR(conveyor.SensorPosition(*data_),
                  conveyor.TrueSlidePosition(*data_), 1e-6);
      run.worst_error = std::max(
          run.worst_error,
          std::abs(conveyor.TrueSlidePosition(*data_) - nominal));
      run.peak_slip = std::max(run.peak_slip, record.peak_slip);
    }
    EXPECT_EQ(conveyor.steps(), Conveyor::kSlotCount);
    return run;
  }

  std::string path_;
  cell::ModelPtr model_;
  cell::DataPtr data_;
  std::vector<int> modules_;
};

// -------------------------------------------------------------- the model

TEST_F(ConveyorTest, ModelHasTheNamedParts) {
  const int conveyor = mj_name2id(model_.get(), mjOBJ_BODY, "conveyor");
  const int tube = mj_name2id(model_.get(), mjOBJ_BODY, "tube");
  const int slide = mj_name2id(model_.get(), mjOBJ_JOINT, "conveyor_slide");
  const int motor = mj_name2id(model_.get(), mjOBJ_ACTUATOR, "conveyor_motor");
  const int sensor =
      mj_name2id(model_.get(), mjOBJ_SENSOR, "conveyor_position");
  ASSERT_GE(conveyor, 0);
  ASSERT_GE(tube, 0);
  ASSERT_GE(slide, 0);
  ASSERT_GE(motor, 0);
  ASSERT_GE(sensor, 0);

  // The bed is fixed; the tube has exactly the one slide joint, along Y.
  EXPECT_EQ(model_->body_jntnum[conveyor], 0);
  EXPECT_EQ(model_->body_jntnum[tube], 1);
  EXPECT_EQ(model_->jnt_bodyid[slide], tube);
  EXPECT_EQ(model_->jnt_type[slide], mjJNT_SLIDE);
  EXPECT_TRUE(Eigen::Map<const Eigen::Vector3d>(model_->jnt_axis + 3 * slide)
                  .isApprox(Eigen::Vector3d::UnitY()));

  // A position servo: gain on position, ctrl in metres.
  EXPECT_EQ(model_->actuator_trnid[2 * motor], slide);
  EXPECT_GT(model_->actuator_gainprm[mjNGAIN * motor], 0.0);

  EXPECT_TRUE(Conveyor::Find(*model_).has_value());
}

TEST_F(ConveyorTest, FindFailsWithoutAConveyor) {
  // common.xml by itself is a valid model with only the floor in it.
  const std::string common =
      path_.substr(0, path_.find_last_of('/') + 1) + "common.xml";
  std::string error;
  cell::ModelPtr empty = cell::LoadModel(common, &error);
  ASSERT_TRUE(empty) << error;
  EXPECT_FALSE(Conveyor::Find(*empty).has_value());
}

TEST_F(ConveyorTest, FindRejectsUnusableSettings) {
  ConveyorSettings settings;
  settings.limits.acceleration = 0.0;
  EXPECT_FALSE(Conveyor::Find(*model_, settings).has_value());
  settings = {};
  settings.stop_error_stddev = -0.001;
  EXPECT_FALSE(Conveyor::Find(*model_, settings).has_value());
}

TEST_F(ConveyorTest, EveryConveyorNameHasTheConveyorPrefix) {
  const int conveyor = mj_name2id(model_.get(), mjOBJ_BODY, "conveyor");
  const int tube = mj_name2id(model_.get(), mjOBJ_BODY, "tube");
  for (int g = 0; g < model_->ngeom; ++g) {
    const int body = model_->geom_bodyid[g];
    if (body != conveyor && body != tube) continue;
    const std::string name = mj_id2name(model_.get(), mjOBJ_GEOM, g);
    EXPECT_TRUE(name.starts_with("conveyor_") || name.starts_with("tube") ||
                name.starts_with("rail_"))
        << name;
  }
}

TEST_F(ConveyorTest, RailsMatchTheSlotLayout) {
  const int tube = mj_name2id(model_.get(), mjOBJ_BODY, "tube");
  int rails = 0;
  // For each slot, the rails' y in the tube frame.
  std::vector<std::vector<double>> rail_y(Conveyor::kSlotCount);
  for (int g = 0; g < model_->ngeom; ++g) {
    const char* name = mj_id2name(model_.get(), mjOBJ_GEOM, g);
    if (name == nullptr || !std::string(name).starts_with("rail_")) continue;
    ++rails;
    EXPECT_EQ(model_->geom_bodyid[g], tube) << name;
    EXPECT_EQ(model_->geom_type[g], mjGEOM_BOX) << name;

    const double* size = model_->geom_size + 3 * g;
    const double* pos = model_->geom_pos + 3 * g;
    // Top surface on the tube frame's z = 0; across the tube; centred on it.
    EXPECT_NEAR(pos[2] + size[2], 0.0, 1e-9) << name;
    EXPECT_NEAR(pos[0], 0.0, 1e-9) << name;
    EXPECT_GT(size[0], size[1]) << name << " should run along X";

    const int slot =
        static_cast<int>(std::lround(pos[1] / Conveyor::kStepLength));
    ASSERT_GE(slot, 0) << name;
    ASSERT_LT(slot, Conveyor::kSlotCount) << name;
    rail_y[slot].push_back(pos[1]);

    // The module that lies on it overhangs it at both ends, but by less than
    // a third of its length, and the rail stops short of the arm.
    EXPECT_LT(size[0], Conveyor::kModuleLength / 2.0) << name;
    EXPECT_GT(size[0], Conveyor::kModuleLength / 3.0) << name;
  }
  EXPECT_EQ(rails, 2 * Conveyor::kSlotCount);
  // Named as models/conveyor.xml says.
  EXPECT_GE(mj_name2id(model_.get(), mjOBJ_GEOM, "rail_a00"), 0);
  EXPECT_GE(mj_name2id(model_.get(), mjOBJ_GEOM, "rail_b09"), 0);

  // Two rails per slot, symmetric about the slot's centre and under the
  // module, far enough apart to hold it flat.
  for (int slot = 0; slot < Conveyor::kSlotCount; ++slot) {
    ASSERT_EQ(rail_y[slot].size(), 2u) << "slot " << slot;
    const double centre = Conveyor::SlotPoseOnTube(slot).translation().y();
    const double a = std::min(rail_y[slot][0], rail_y[slot][1]) - centre;
    const double b = std::max(rail_y[slot][0], rail_y[slot][1]) - centre;
    EXPECT_NEAR(a, -b, 1e-9) << "slot " << slot;
    EXPECT_GT(b, Conveyor::kModuleWidth / 4.0) << "slot " << slot;
    EXPECT_LT(b, Conveyor::kModuleWidth / 2.0) << "slot " << slot;
  }

  // The tube's frame in the model is where src/conveyor.h says it is.
  const Eigen::Isometry3d tube_pose = Conveyor::SlotPose(0, 0.0) *
                                      Conveyor::SlotPoseOnTube(0).inverse();
  const Eigen::Isometry3d model_tube =
      Conveyor::Find(*model_)->TrueTubePose(*data_);
  EXPECT_TRUE(model_tube.isApprox(tube_pose, 1e-12));
}

TEST_F(ConveyorTest, PlacementPoseIsTheSameAfterEveryStep) {
  const Eigen::Isometry3d placement = Conveyor::PlacementPose();
  EXPECT_TRUE(placement.translation().isApprox(
      Eigen::Vector3d(-2.0, 0.0, 0.80 + 0.0175), 1e-12));
  EXPECT_TRUE(placement.linear().isIdentity(1e-12));
  for (int n = 0; n < Conveyor::kSlotCount; ++n) {
    const Eigen::Isometry3d pose =
        Conveyor::SlotPose(n, Conveyor::NominalSlidePosition(n));
    EXPECT_TRUE(pose.isApprox(placement, 1e-12)) << "step " << n;
  }
  // Placed modules leave toward -Y; empty slots wait toward +Y.
  EXPECT_LT(Conveyor::SlotPose(0, Conveyor::NominalSlidePosition(1))
                .translation()
                .y(),
            0.0);
  EXPECT_GT(Conveyor::SlotPose(1, Conveyor::NominalSlidePosition(0))
                .translation()
                .y(),
            0.0);
}

TEST_F(ConveyorTest, SlideRangeHoldsOneTubeAndNoMore) {
  const int slide = mj_name2id(model_.get(), mjOBJ_JOINT, "conveyor_slide");
  const double low = model_->jnt_range[2 * slide];
  const double high = model_->jnt_range[2 * slide + 1];
  const double full = Conveyor::NominalSlidePosition(Conveyor::kSlotCount);
  const double over = Conveyor::NominalSlidePosition(Conveyor::kSlotCount + 1);
  EXPECT_LE(low, full - 0.1);  // a full tube plus room for the error
  EXPECT_GT(low, over);        // but not one step more
  EXPECT_LE(high, 1.0);
  EXPECT_GE(high, 0.0);
}

TEST_F(ConveyorTest, TubeHoldsEveryModuleWherePlaced) {
  // In the tube frame, each slot's module footprint is over the tube, from
  // the first slot's back edge to the last slot's front edge.
  const int tube_geom = mj_name2id(model_.get(), mjOBJ_GEOM, "tube");
  ASSERT_GE(tube_geom, 0);
  const double centre = model_->geom_pos[3 * tube_geom + 1];
  const double half = model_->geom_size[3 * tube_geom + 1];
  const double first = Conveyor::SlotPoseOnTube(0).translation().y() -
                       Conveyor::kModuleWidth / 2;
  const double last =
      Conveyor::SlotPoseOnTube(Conveyor::kSlotCount - 1).translation().y() +
      Conveyor::kModuleWidth / 2;
  EXPECT_LE(centre - half, first);
  EXPECT_GE(centre + half, last);
}

TEST_F(ConveyorTest, KeepsClearOfTheArm) {
  // Nothing of the conveyor or tube comes closer to the arm than x = -0.7,
  // wherever the tube is along its travel (the slide only moves it along Y).
  const int conveyor = mj_name2id(model_.get(), mjOBJ_BODY, "conveyor");
  const int tube = mj_name2id(model_.get(), mjOBJ_BODY, "tube");
  double nearest = -1e9;
  for (int g = 0; g < model_->ngeom; ++g) {
    const int body = model_->geom_bodyid[g];
    if (body != conveyor && body != tube) continue;
    // geom_aabb is centre and half-size in the geom's own frame.
    const Eigen::Map<const Eigen::Vector3d> centre(model_->geom_aabb + 6 * g);
    const Eigen::Map<const Eigen::Vector3d> half(model_->geom_aabb + 6 * g + 3);
    const Eigen::Map<const Eigen::Vector3d> pos(data_->geom_xpos + 3 * g);
    const Eigen::Matrix3d rot =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            data_->geom_xmat + 9 * g);
    // The largest x of the box's eight corners.
    const double max_x = (pos + rot * centre + rot.cwiseAbs() * half).x();
    nearest = std::max(nearest, max_x);
  }
  std::printf("conveyor and tube reach x = %.3f\n", nearest);
  EXPECT_LE(nearest, -0.7);
}

// ------------------------------------------------------------- the moves

TEST_F(ConveyorTest, OneStepMovesByStepLengthPlusTheInjectedError) {
  Conveyor conveyor = MakeConveyor({}, /*seed=*/7);
  Settle(conveyor);
  const double before = conveyor.TrueSlidePosition(*data_);

  const StepRecord record = RunStep(conveyor);
  RunFor(conveyor, 1.0);  // the servo's last fraction of a millimetre

  const double error = conveyor.true_last_stop_error();
  EXPECT_NE(error, 0.0);
  EXPECT_LT(std::abs(error), 5 * 0.002);  // default standard deviation 2 mm
  const double moved = conveyor.TrueSlidePosition(*data_) - before;
  EXPECT_NEAR(moved, -Conveyor::kStepLength + error, 1e-5);
  // The encoder reports where the tube really is, not where it should be.
  EXPECT_NEAR(conveyor.SensorPosition(*data_),
              conveyor.TrueSlidePosition(*data_), 1e-6);
  EXPECT_NEAR(conveyor.SensorPosition(*data_), conveyor.true_stop_position(),
              1e-5);
  EXPECT_NEAR(conveyor.nominal_position(), -Conveyor::kStepLength, 1e-12);
  EXPECT_EQ(conveyor.steps(), 1);
  std::printf("injected error %+.4f m, moved %.4f m, step took %.3f s\n", error,
              moved, record.duration);
}

TEST_F(ConveyorTest, WithoutErrorItStopsOnTargetWithoutOvershoot) {
  ConveyorSettings settings;
  settings.stop_error_stddev = 0.0;
  Conveyor conveyor = MakeConveyor(settings);
  Settle(conveyor);

  const StepRecord record = RunStep(conveyor);
  const double at_in_position = std::abs(
      conveyor.TrueSlidePosition(*data_) - conveyor.nominal_position());
  RunFor(conveyor, 1.0);
  const double settled = std::abs(conveyor.TrueSlidePosition(*data_) -
                                  conveyor.nominal_position());

  EXPECT_EQ(conveyor.true_stop_position(), conveyor.nominal_position());
  // When Moving() turns false the tube is within the in-position tolerance;
  // a second later it is there to a hundredth of a millimetre.
  EXPECT_LT(at_in_position, settings.in_position_tolerance);
  EXPECT_LT(settled, 1e-5);
  // The servo is overdamped: it creeps up on the stop and never passes it
  // by more than a tenth of a millimetre.
  EXPECT_LT(record.overshoot, 1e-4);
  // Done within a second of the setpoint finishing.
  EXPECT_LT(record.duration, conveyor.StepDuration() + 1.0);
  std::printf(
      "setpoint %.3f s, in position after %.3f s; off by %.2e m then, "
      "%.2e m a second later; overshoot %.2e m\n",
      conveyor.StepDuration(), record.duration, at_in_position, settled,
      record.overshoot);
}

TEST_F(ConveyorTest, MoveStaysWithinItsLimits) {
  const ConveyorSettings settings;
  Conveyor conveyor = MakeConveyor(settings);
  Settle(conveyor);

  // The setpoint itself, sampled the way Update() feeds it.
  double peak_setpoint_speed = 0.0;
  double peak_setpoint_acceleration = 0.0;
  {
    Conveyor probe = conveyor;
    const int motor =
        mj_name2id(model_.get(), mjOBJ_ACTUATOR, "conveyor_motor");
    cell::DataPtr scratch = cell::MakeData(*model_);
    scratch->time = data_->time;
    ASSERT_TRUE(probe.StartStep(*scratch));
    const double dt = model_->opt.timestep;
    std::vector<double> setpoints;
    for (double t = 0; t < probe.StepDuration() + 0.1; t += dt) {
      scratch->time = data_->time + t;
      probe.Update(scratch.get());
      setpoints.push_back(scratch->ctrl[motor]);
    }
    for (size_t i = 2; i < setpoints.size(); ++i) {
      const double v = (setpoints[i] - setpoints[i - 1]) / dt;
      const double a =
          (setpoints[i] - 2 * setpoints[i - 1] + setpoints[i - 2]) / (dt * dt);
      peak_setpoint_speed = std::max(peak_setpoint_speed, std::abs(v));
      peak_setpoint_acceleration =
          std::max(peak_setpoint_acceleration, std::abs(a));
    }
  }
  EXPECT_LE(peak_setpoint_speed, settings.limits.velocity * 1.001);
  EXPECT_LE(peak_setpoint_acceleration, settings.limits.acceleration * 1.01);

  // The tube, which follows the setpoint through the servo. The servo is
  // overdamped, so it smooths the setpoint and never raises its peaks; 1%
  // allows for the integrator.
  const StepRecord record = RunStep(conveyor);
  EXPECT_LE(record.peak_speed, settings.limits.velocity * 1.01);
  EXPECT_LE(record.peak_acceleration, settings.limits.acceleration * 1.01);
  // And it does get going: the limits are not trivially met.
  EXPECT_GT(record.peak_speed, settings.limits.velocity * 0.9);
  std::printf(
      "setpoint peaks %.3f m/s, %.3f m/s^2; tube peaks %.3f m/s, %.3f m/s^2; "
      "modules peak %.3f m/s^2\n",
      peak_setpoint_speed, peak_setpoint_acceleration, record.peak_speed,
      record.peak_acceleration, record.peak_module_acceleration);
}

TEST_F(ConveyorTest, ModulesRideAlongWithoutSliding) {
  Conveyor conveyor = MakeConveyor();
  Settle(conveyor);
  std::vector<Eigen::Vector3d> before;
  for (int i = 0; i < kTestModules; ++i) {
    before.push_back(ModuleOnTube(conveyor, i));
    // Settled onto the rails where they were put.
    const Eigen::Vector3d slot = Conveyor::SlotPoseOnTube(i).translation();
    EXPECT_LT((before[i] - slot).norm(), 1e-3) << "module " << i;
  }

  const StepRecord record = RunStep(conveyor);
  // Moved with the tube to within 1.5 mm at every instant. Contacts in
  // MuJoCo are slightly soft, so a module lags a little while the tube
  // accelerates and catches up when it stops: this is that, not sliding, as
  // the net slip below shows.
  EXPECT_LT(record.peak_slip, 1.5e-3);
  // The modules accelerate no harder than the tube's limit, about a fifth
  // of the 0.5 g that friction 0.5 can hold.
  EXPECT_LE(record.peak_module_acceleration,
            ConveyorSettings{}.limits.acceleration * 1.01);

  std::vector<Eigen::Vector3d> stopped;
  for (int i = 0; i < kTestModules; ++i) {
    stopped.push_back(ModuleOnTube(conveyor, i));
  }
  // Where it ended up on the tube, against where it started.
  double net_slip = 0.0;
  for (int i = 0; i < kTestModules; ++i) {
    net_slip = std::max(net_slip, (stopped[i] - before[i]).norm());
  }
  EXPECT_LT(net_slip, 1e-4);

  // And stays put once the tube has stopped.
  RunFor(conveyor, 2.0);
  double creep = 0.0;
  for (int i = 0; i < kTestModules; ++i) {
    creep = std::max(creep, (ModuleOnTube(conveyor, i) - stopped[i]).norm());
    // Still flat, glass up.
    const Eigen::Vector3d up =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
            data_->xmat + 9 * modules_[i])
            .col(2);
    EXPECT_GT(up.z(), std::cos(0.001)) << "module " << i;
  }
  EXPECT_LT(creep, 1e-4);
  std::printf(
      "peak slip during the step %.2e m, net %.2e m; creep after it %.2e m; "
      "module peak acceleration %.3f m/s^2\n",
      record.peak_slip, net_slip, creep, record.peak_module_acceleration);
}

TEST_F(ConveyorTest, StopErrorsAreIndependentAndDoNotAddUp) {
  Conveyor conveyor = MakeConveyor({}, /*seed=*/11);
  Settle(conveyor);

  const FullTube run = FillTube(conveyor);
  // Each stop is off its own nominal position by its own error alone, so
  // the worst over a tube stays within a few standard deviations of 2 mm,
  // instead of growing as sqrt(steps) would.
  EXPECT_GT(run.worst_error, 0.0);
  EXPECT_LT(run.worst_error, 5 * 0.002);
  EXPECT_NEAR(conveyor.TrueSlidePosition(*data_),
              Conveyor::NominalSlidePosition(Conveyor::kSlotCount) +
                  conveyor.true_last_stop_error(),
              1e-4);
  // The modules came all the way, still on their slots.
  EXPECT_LT(run.peak_slip, 1.5e-3);
  for (int i = 0; i < kTestModules; ++i) {
    const Eigen::Vector3d slot = Conveyor::SlotPoseOnTube(i).translation();
    EXPECT_LT((ModuleOnTube(conveyor, i) - slot).norm(), 2e-3)
        << "module " << i;
  }
  std::printf("%d steps: worst stop %.4f m off nominal, last %+.4f m\n",
              Conveyor::kSlotCount, run.worst_error,
              conveyor.true_last_stop_error());
}

TEST_F(ConveyorTest, FullTubeIsSwappedForAnEmptyOne) {
  Conveyor conveyor = MakeConveyor({}, /*seed=*/5);
  Settle(conveyor);
  EXPECT_EQ(conveyor.tubes_completed(), 0);
  FillTube(conveyor);

  // The tube is full: an 11th step would run off the end of the range.
  EXPECT_FALSE(conveyor.StartStep(*data_));
  EXPECT_EQ(conveyor.steps(), Conveyor::kSlotCount);

  CarryModulesAway();
  ASSERT_TRUE(conveyor.SwapTube(data_.get()));
  EXPECT_EQ(conveyor.tubes_completed(), 1);
  EXPECT_EQ(conveyor.steps(), 0);
  EXPECT_EQ(conveyor.nominal_position(), 0.0);
  EXPECT_EQ(conveyor.TrueSlidePosition(*data_), 0.0);
  EXPECT_EQ(conveyor.SensorPosition(*data_), 0.0);
  EXPECT_EQ(conveyor.SensorVelocity(*data_), 0.0);
  EXPECT_FALSE(conveyor.Moving(*data_));
  EXPECT_TRUE(conveyor.TrueSlotPose(*data_, 0).isApprox(
      Conveyor::PlacementPose(), 1e-12));

  // And stays there: the motor was told the same, so nothing lurches.
  double drift = 0.0;
  const double end = data_->time + 2.0;
  while (data_->time < end) {
    Tick(conveyor);
    drift = std::max(drift, std::abs(conveyor.SensorPosition(*data_)));
  }
  EXPECT_LT(drift, 1e-9);
  EXPECT_FALSE(conveyor.Moving(*data_));

  // The new tube takes a full load of steps too.
  FillTube(conveyor);
  EXPECT_FALSE(conveyor.StartStep(*data_));
  EXPECT_EQ(conveyor.steps(), Conveyor::kSlotCount);
  EXPECT_EQ(conveyor.tubes_completed(), 1);
}

TEST_F(ConveyorTest, NoNewStepWhileMoving) {
  Conveyor conveyor = MakeConveyor();
  Settle(conveyor);
  ASSERT_TRUE(conveyor.StartStep(*data_));
  Tick(conveyor);
  EXPECT_TRUE(conveyor.Moving(*data_));
  EXPECT_FALSE(conveyor.StartStep(*data_));
  EXPECT_EQ(conveyor.steps(), 1);
  EXPECT_FALSE(conveyor.SwapTube(data_.get()));
  EXPECT_EQ(conveyor.tubes_completed(), 0);
  EXPECT_EQ(conveyor.steps(), 1);
}

TEST_F(ConveyorTest, SameSeedGivesTheSameErrors) {
  auto errors = [&](unsigned seed) {
    Conveyor conveyor = MakeConveyor({}, seed);
    cell::DataPtr data = cell::MakeData(*model_);
    std::vector<double> drawn;
    for (int n = 0; n < 5; ++n) {
      EXPECT_TRUE(conveyor.StartStep(*data));
      drawn.push_back(conveyor.true_last_stop_error());
      // Jump past the move rather than simulate it: Moving() then only
      // waits on the encoder, which the scratch state sets to the stop.
      data->time += conveyor.StepDuration() + 1.0;
      data->sensordata[model_->sensor_adr[mj_name2id(
          model_.get(), mjOBJ_SENSOR, "conveyor_position")]] =
          conveyor.true_stop_position();
    }
    return drawn;
  };
  const std::vector<double> first = errors(42);
  EXPECT_EQ(first, errors(42));
  EXPECT_NE(first, errors(43));

  // And the whole simulated run repeats exactly.
  auto final_position = [&](unsigned seed) {
    Conveyor conveyor = MakeConveyor({}, seed);
    cell::DataPtr data = cell::MakeData(*model_);
    for (int n = 0; n < 2; ++n) {
      EXPECT_TRUE(conveyor.StartStep(*data));
      const double end = data->time + conveyor.StepDuration() + 0.5;
      while (data->time < end) {
        conveyor.Update(data.get());
        mj_step(model_.get(), data.get());
      }
    }
    return Eigen::Vector2d(conveyor.TrueSlidePosition(*data),
                           data->xpos[3 * modules_[0] + 1]);
  };
  EXPECT_EQ(final_position(42), final_position(42));
}

}  // namespace
