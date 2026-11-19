// Checks that the toolchain works end to end: the model loads, the physics
// steps, and the state stays sane.
//
//   bazel test //:step_test
//
// The model's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test.

#include <cmath>
#include <cstdlib>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/mj_handles.h"

namespace {

// A fixture: SetUp runs before each TEST_F below, so every test starts from a
// freshly loaded model and its initial state.
class StepTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";

    std::string error;
    model_ = cell::LoadModel(path, &error);
    ASSERT_TRUE(model_) << "could not load " << path << ": " << error;

    data_ = cell::MakeData(*model_);
    ASSERT_TRUE(data_);
  }

  void StepFor(double seconds) {
    const double end = data_->time + seconds;
    while (data_->time < end) {
      mj_step(model_.get(), data_.get());
    }
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
};

TEST_F(StepTest, ArmHasSixJointsAndSixMotors) {
  EXPECT_EQ(model_->njnt, 6);
  EXPECT_EQ(model_->nq, 6);  // position coordinates
  EXPECT_EQ(model_->nv, 6);  // velocity coordinates
  EXPECT_EQ(model_->nu, 6);  // motor commands
}

TEST_F(StepTest, OneStepAdvancesTimeByTheTimestep) {
  ASSERT_EQ(data_->time, 0.0);
  mj_step(model_.get(), data_.get());
  EXPECT_DOUBLE_EQ(data_->time, model_->opt.timestep);
}

TEST_F(StepTest, StateStaysFiniteAndWithinJointLimits) {
  StepFor(1.0);
  EXPECT_NEAR(data_->time, 1.0, model_->opt.timestep);

  for (int i = 0; i < model_->nq; ++i) {
    EXPECT_TRUE(std::isfinite(data_->qpos[i])) << "qpos[" << i << "]";
  }
  for (int i = 0; i < model_->nv; ++i) {
    EXPECT_TRUE(std::isfinite(data_->qvel[i])) << "qvel[" << i << "]";
  }

  // jnt_range holds (lower, upper) for each joint that has a limit. Limits
  // in MuJoCo are soft, so a small overshoot is allowed.
  const double slack = 0.01;  // rad
  for (int joint = 0; joint < model_->njnt; ++joint) {
    if (!model_->jnt_limited[joint]) continue;
    const double angle = data_->qpos[model_->jnt_qposadr[joint]];
    EXPECT_GE(angle, model_->jnt_range[2 * joint] - slack) << "joint " << joint;
    EXPECT_LE(angle, model_->jnt_range[2 * joint + 1] + slack)
        << "joint " << joint;
  }
}

TEST_F(StepTest, SameStartGivesSameResult) {
  // The gate in CI depends on this: a failed trial must replay identically.
  cell::DataPtr other = cell::MakeData(*model_);
  for (int i = 0; i < 200; ++i) {
    mj_step(model_.get(), data_.get());
    mj_step(model_.get(), other.get());
  }
  for (int i = 0; i < model_->nq; ++i) {
    EXPECT_EQ(data_->qpos[i], other->qpos[i]) << "qpos[" << i << "]";
  }
}

TEST(LoadModelTest, MissingFileGivesEmptyHandleAndMessage) {
  std::string error;
  cell::ModelPtr model = cell::LoadModel("no/such/file.xml", &error);
  EXPECT_FALSE(model);
  EXPECT_FALSE(error.empty());
}

}  // namespace
