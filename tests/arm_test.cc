// Checks the KUKA arm model against the vendor's numbers and against what the
// rest of the cell relies on: the names, the joint ranges and torque limits,
// where the flange is, and that the motors hold a pose.
//
//   bazel test //:arm_test
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test (models/arm_scene.xml).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/mj_handles.h"

namespace {

constexpr int kJoints = 6;
constexpr double kHalfPi = 1.5707963267948966;

// From the vendor URDF (kuka_quantec_support, kr210_r2700_2_macro.xacro):
// joint ranges in radians and torque limits ("effort") in N m.
struct VendorJoint {
  double lower;
  double upper;
  double effort;
};
constexpr std::array<VendorJoint, kJoints> kVendor = {{
    {-3.2288591161895095, 3.2288591161895095, 7290.0},
    {-2.443460952792061, -0.08726646259971647, 8910.0},
    {-2.0943951023931953, 2.9321531433504737, 6988.84717},
    {-6.1086523819801535, 6.1086523819801535, 2274.29199},
    {-2.181661564992912, 2.181661564992912, 1398.30298},
    {-6.1086523819801535, 6.1086523819801535, 781.554626},
}};

// KUKA's home with the tool facing down, as in the arm_home keyframe.
constexpr std::array<double, kJoints> kHome = {0, -kHalfPi, kHalfPi,
                                               0, kHalfPi,  0};

// Stretched out toward +X with the tool facing down: as far out as the
// joint ranges allow while keeping joint 2 clear of its limit (-5 degrees),
// so that the motor, not the limit, carries the weight.
constexpr std::array<double, kJoints> kFullReach = {0, -0.1, 0.1,
                                                    0, kHalfPi, 0};

std::string JointName(int i) { return "arm_joint_" + std::to_string(i + 1); }
std::string MotorName(int i) { return "arm_motor_" + std::to_string(i + 1); }

class ArmTest : public ::testing::Test {
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

  int Id(mjtObj type, const std::string& name) const {
    return mj_name2id(model_.get(), type, name.c_str());
  }

  // Puts the joints at `q` and works out where every body is, without
  // simulating.
  void PlaceAt(const std::array<double, kJoints>& q) {
    for (int i = 0; i < kJoints; ++i) {
      data_->qpos[model_->jnt_qposadr[Id(mjOBJ_JOINT, JointName(i))]] = q[i];
    }
    mj_kinematics(model_.get(), data_.get());
  }

  // Starts at rest at `q`, commands `q`, and simulates for `seconds`.
  void HoldFor(const std::array<double, kJoints>& q, double seconds) {
    mj_resetData(model_.get(), data_.get());
    for (int i = 0; i < kJoints; ++i) {
      data_->qpos[model_->jnt_qposadr[Id(mjOBJ_JOINT, JointName(i))]] = q[i];
      data_->ctrl[Id(mjOBJ_ACTUATOR, MotorName(i))] = q[i];
    }
    const double end = data_->time + seconds;
    while (data_->time < end) {
      mj_step(model_.get(), data_.get());
    }
  }

  // Largest |angle - q| over the six joints.
  double LargestError(const std::array<double, kJoints>& q) const {
    double largest = 0.0;
    for (int i = 0; i < kJoints; ++i) {
      const int adr = model_->jnt_qposadr[Id(mjOBJ_JOINT, JointName(i))];
      largest = std::max(largest, std::abs(data_->qpos[adr] - q[i]));
    }
    return largest;
  }

  double LargestSpeed() const {
    double largest = 0.0;
    for (int i = 0; i < model_->nv; ++i) {
      largest = std::max(largest, std::abs(data_->qvel[i]));
    }
    return largest;
  }

  void ExpectFinite() const {
    for (int i = 0; i < model_->nq; ++i) {
      EXPECT_TRUE(std::isfinite(data_->qpos[i])) << "qpos[" << i << "]";
    }
    for (int i = 0; i < model_->nv; ++i) {
      EXPECT_TRUE(std::isfinite(data_->qvel[i])) << "qvel[" << i << "]";
    }
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
};

TEST_F(ArmTest, SixJointsAndSixMotorsWithTheAgreedNames) {
  EXPECT_EQ(model_->njnt, kJoints);
  EXPECT_EQ(model_->nu, kJoints);

  for (int i = 0; i < kJoints; ++i) {
    const int joint = Id(mjOBJ_JOINT, JointName(i));
    ASSERT_GE(joint, 0) << JointName(i);
    EXPECT_EQ(model_->jnt_type[joint], mjJNT_HINGE) << JointName(i);

    // Joint i moves link i.
    const std::string link = "arm_link_" + std::to_string(i + 1);
    EXPECT_EQ(model_->jnt_bodyid[joint], Id(mjOBJ_BODY, link)) << link;

    // Motor i drives joint i.
    const int motor = Id(mjOBJ_ACTUATOR, MotorName(i));
    ASSERT_GE(motor, 0) << MotorName(i);
    EXPECT_EQ(model_->actuator_trntype[motor], mjTRN_JOINT);
    EXPECT_EQ(model_->actuator_trnid[2 * motor], joint) << MotorName(i);
  }

  for (const char* name : {"arm_base", "arm_flange", "tool"}) {
    EXPECT_GE(Id(mjOBJ_BODY, name), 0) << name;
  }
  EXPECT_GE(Id(mjOBJ_SITE, "tool_tip"), 0);
  EXPECT_GE(Id(mjOBJ_KEY, "arm_home"), 0);

  // The tool is bolted to the flange.
  EXPECT_EQ(model_->body_parentid[Id(mjOBJ_BODY, "tool")],
            Id(mjOBJ_BODY, "arm_flange"));
}

TEST_F(ArmTest, JointRangesAndTorqueLimitsAreTheVendors) {
  for (int i = 0; i < kJoints; ++i) {
    const int joint = Id(mjOBJ_JOINT, JointName(i));
    const int motor = Id(mjOBJ_ACTUATOR, MotorName(i));
    ASSERT_GE(joint, 0);
    ASSERT_GE(motor, 0);

    EXPECT_TRUE(model_->jnt_limited[joint]) << JointName(i);
    EXPECT_NEAR(model_->jnt_range[2 * joint], kVendor[i].lower, 1e-7);
    EXPECT_NEAR(model_->jnt_range[2 * joint + 1], kVendor[i].upper, 1e-7);

    // A motor is only ever asked for an angle the joint can reach.
    EXPECT_TRUE(model_->actuator_ctrllimited[motor]) << MotorName(i);
    EXPECT_NEAR(model_->actuator_ctrlrange[2 * motor], kVendor[i].lower, 1e-7);
    EXPECT_NEAR(model_->actuator_ctrlrange[2 * motor + 1], kVendor[i].upper,
                1e-7);

    EXPECT_TRUE(model_->actuator_forcelimited[motor]) << MotorName(i);
    EXPECT_NEAR(model_->actuator_forcerange[2 * motor], -kVendor[i].effort,
                1e-6);
    EXPECT_NEAR(model_->actuator_forcerange[2 * motor + 1], kVendor[i].effort,
                1e-6);
  }
}

TEST_F(ArmTest, FlangeIsWhereTheVendorKinematicsPutIt) {
  // Expected numbers: the URDF's joint origins chained together. At zero the
  // arm is stretched out level toward +X: joint 2 is 0.330 m forward and
  // 0.645 m up, the upper arm is 1.150 m, the forearm is offset 0.115 m up
  // and is 1.220 m long to the wrist centre, and the flange is 0.215 m
  // beyond that. So the flange is at x = 0.33 + 1.15 + 1.22 + 0.215.
  const int flange = Id(mjOBJ_BODY, "arm_flange");
  const int wrist = Id(mjOBJ_BODY, "arm_link_5");
  ASSERT_GE(flange, 0);
  ASSERT_GE(wrist, 0);
  const double tol = 1e-9;

  PlaceAt({0, 0, 0, 0, 0, 0});
  EXPECT_NEAR(data_->xpos[3 * wrist + 0], 2.700, tol);
  EXPECT_NEAR(data_->xpos[3 * wrist + 1], 0.0, tol);
  EXPECT_NEAR(data_->xpos[3 * wrist + 2], 0.760, tol);
  EXPECT_NEAR(data_->xpos[3 * flange + 0], 2.915, tol);
  EXPECT_NEAR(data_->xpos[3 * flange + 1], 0.0, tol);
  EXPECT_NEAR(data_->xpos[3 * flange + 2], 0.760, tol);
  // The flange's +Z (third column of xmat) points out of the flange: +X.
  EXPECT_NEAR(data_->xmat[9 * flange + 2], 1.0, tol);
  EXPECT_NEAR(data_->xmat[9 * flange + 5], 0.0, tol);
  EXPECT_NEAR(data_->xmat[9 * flange + 8], 0.0, tol);

  // At home the upper arm is upright and the forearm level: the wrist
  // centre is 0.33 + 1.22 forward and 0.645 + 1.15 + 0.115 up, and the
  // flange faces the floor 0.215 m below it.
  PlaceAt(kHome);
  EXPECT_NEAR(data_->xpos[3 * flange + 0], 1.550, tol);
  EXPECT_NEAR(data_->xpos[3 * flange + 1], 0.0, tol);
  EXPECT_NEAR(data_->xpos[3 * flange + 2], 1.695, tol);
  EXPECT_NEAR(data_->xmat[9 * flange + 8], -1.0, tol);
}

TEST_F(ArmTest, JointSignsAreKukas) {
  const int flange = Id(mjOBJ_BODY, "arm_flange");

  // Joint 1 positive turns the arm clockwise seen from above: toward -Y.
  PlaceAt({0.1, -kHalfPi, kHalfPi, 0, 0, 0});
  EXPECT_LT(data_->xpos[3 * flange + 1], -0.1);

  // Joint 2 more negative lifts the upper arm, so the flange rises.
  PlaceAt({0, -0.5, 0, 0, 0, 0});
  const double lower = data_->xpos[3 * flange + 2];
  PlaceAt({0, -0.6, 0, 0, 0, 0});
  EXPECT_GT(data_->xpos[3 * flange + 2], lower);
}

TEST_F(ArmTest, HomeKeyframeMatchesItsControls) {
  const int key = Id(mjOBJ_KEY, "arm_home");
  ASSERT_GE(key, 0);
  for (int i = 0; i < kJoints; ++i) {
    EXPECT_DOUBLE_EQ(model_->key_qpos[key * model_->nq + i], kHome[i]);
    EXPECT_DOUBLE_EQ(model_->key_ctrl[key * model_->nu + i], kHome[i]);
  }
}

TEST_F(ArmTest, HoldsHomeForFiveSeconds) {
  mj_resetDataKeyframe(model_.get(), data_.get(), Id(mjOBJ_KEY, "arm_home"));
  double largest = 0.0;
  while (data_->time < 5.0) {
    mj_step(model_.get(), data_.get());
    largest = std::max(largest, LargestError(kHome));
  }
  ExpectFinite();
  // Gravity bends the joints by a fraction of a milliradian; 1 mrad at the
  // 1.7 m from joint 2 to the flange is under 2 mm.
  EXPECT_LT(largest, 1e-3);
  EXPECT_LT(LargestSpeed(), 1e-6);  // settled, not ringing
}

TEST_F(ArmTest, HoldsAThirtyKilogramModuleAtFullReach) {
  // Hang a 30 kg module on the tool by making the tool's body heavier, with
  // the extra mass just past the cups. Only the weight and where it acts
  // matter for how far the arm sags.
  const int tool = Id(mjOBJ_BODY, "tool");
  const int tip = Id(mjOBJ_SITE, "tool_tip");
  ASSERT_GE(tool, 0);
  ASSERT_GE(tip, 0);
  ASSERT_EQ(model_->site_bodyid[tip], tool);
  const double module_kg = 30.0;
  const double tool_kg = model_->body_mass[tool];
  for (int k = 0; k < 3; ++k) {
    const double at = model_->site_pos[3 * tip + k] + (k == 2 ? 0.02 : 0.0);
    model_->body_ipos[3 * tool + k] =
        (tool_kg * model_->body_ipos[3 * tool + k] + module_kg * at) /
        (tool_kg + module_kg);
  }
  model_->body_mass[tool] = tool_kg + module_kg;

  // Where the tool tip would be with no sag.
  PlaceAt(kFullReach);
  std::array<double, 3> wanted;
  for (int k = 0; k < 3; ++k) wanted[k] = data_->site_xpos[3 * tip + k];
  ASSERT_GT(std::hypot(wanted[0], wanted[1]), 2.6);  // really at full reach

  HoldFor(kFullReach, 5.0);
  ExpectFinite();
  EXPECT_LT(LargestError(kFullReach), 1e-3);
  EXPECT_LT(LargestSpeed(), 1e-6);
  const double sag =
      std::hypot(data_->site_xpos[3 * tip + 0] - wanted[0],
                 data_->site_xpos[3 * tip + 1] - wanted[1],
                 data_->site_xpos[3 * tip + 2] - wanted[2]);
  EXPECT_LT(sag, 0.005);  // metres; measured about 1.5 mm
}

TEST_F(ArmTest, MassIsPlausible) {
  // KUKA gives about 1070 kg for the KR 210 R2700-2. The tool is not
  // counted here.
  double arm_kg = 0.0;
  for (const char* name : {"arm_base", "arm_link_1", "arm_link_2",
                           "arm_link_3", "arm_link_4", "arm_link_5",
                           "arm_link_6", "arm_flange"}) {
    const int body = Id(mjOBJ_BODY, name);
    ASSERT_GE(body, 0) << name;
    arm_kg += model_->body_mass[body];
  }
  EXPECT_GT(arm_kg, 1000.0);
  EXPECT_LT(arm_kg, 1150.0);
}

TEST_F(ArmTest, StateStaysFiniteFromTheDefaultStart) {
  // The default start is all joints zero with all commands zero, which is
  // what the viewer shows. Joint 2's zero is 5 degrees outside its range,
  // so the arm is pushed back inside it and then held stretched out.
  mj_resetData(model_.get(), data_.get());
  while (data_->time < 3.0) {
    mj_step(model_.get(), data_.get());
  }
  ExpectFinite();
  const int joint_2 = Id(mjOBJ_JOINT, JointName(1));
  EXPECT_LT(data_->qpos[model_->jnt_qposadr[joint_2]],
            kVendor[1].upper + 0.01);
  EXPECT_LT(LargestSpeed(), 1e-3);
}

}  // namespace
