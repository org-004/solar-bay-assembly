// Checks the flat rack: the model has every member that src/flat_rack.h
// lists, each a box of the header's size at the header's place, and nothing
// else on the frame; the two envelope sites agree with the header; there is
// no post, side, roof or end frame, so nothing rises above the deck; the
// footprint is the 40 ft outside; the derived figures are the ones
// docs/bay-hardware.md states; and every member is inside the transport
// envelope. The 20 ft racks of base "two_flat_racks" (fr::twenty) have their
// own figures, places and envelopes checked in the same way, without a model
// file: src/layout_model.cc draws them.
//
//   bazel test //:flat_rack_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test.

#include <algorithm>
#include <cstdlib>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/flat_rack.h"
#include "src/mj_handles.h"

namespace {

namespace fr = cell::flat_rack;
using fr::Box;

constexpr double kTol = 1e-9;

class FlatRackTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const char* path = std::getenv("MODEL_PATH");
    ASSERT_NE(path, nullptr) << "MODEL_PATH is not set";
    std::string error;
    model_ = cell::LoadModel(path, &error);
    ASSERT_TRUE(model_) << "could not load " << path << ": " << error;
    data_ = cell::MakeData(*model_);
    ASSERT_TRUE(data_);
    mj_forward(model_.get(), data_.get());
    body_ = mj_name2id(model_.get(), mjOBJ_BODY, "flat_rack_frame");
    ASSERT_GE(body_, 0) << "no body named flat_rack_frame";
  }

  std::vector<int> GeomsOfFrame() const {
    std::vector<int> out;
    for (int g = 0; g < model_->ngeom; ++g) {
      if (model_->geom_bodyid[g] == body_) out.push_back(g);
    }
    return out;
  }

  std::vector<int> SitesOfFrame() const {
    std::vector<int> out;
    for (int s = 0; s < model_->nsite; ++s) {
      if (model_->site_bodyid[s] == body_) out.push_back(s);
    }
    return out;
  }

  static void ExpectBox(const double* pos, const double* size,
                        const Box& box) {
    for (int i = 0; i < 3; ++i) {
      EXPECT_NEAR(pos[i], box.centre[i], kTol) << box.name << " centre " << i;
      EXPECT_NEAR(size[i], box.half_size[i], kTol)
          << box.name << " half size " << i;
    }
  }

  void ExpectSite(const Box& box) {
    const int s = mj_name2id(model_.get(), mjOBJ_SITE, box.name.c_str());
    ASSERT_GE(s, 0) << "no site " << box.name;
    EXPECT_EQ(model_->site_bodyid[s], body_) << box.name;
    ExpectBox(model_->site_pos + 3 * s, model_->site_size + 3 * s, box);
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  int body_ = -1;
};

TEST_F(FlatRackTest, EveryMemberIsInTheModelAndNothingElse) {
  const std::vector<Box> members = fr::Members();
  std::set<int> seen;
  for (const Box& member : members) {
    const int g = mj_name2id(model_.get(), mjOBJ_GEOM, member.name.c_str());
    ASSERT_GE(g, 0) << "no geom " << member.name;
    EXPECT_EQ(model_->geom_bodyid[g], body_) << member.name;
    EXPECT_EQ(model_->geom_type[g], mjGEOM_BOX) << member.name;
    ExpectBox(model_->geom_pos + 3 * g, model_->geom_size + 3 * g, member);
    seen.insert(g);
  }
  const std::vector<int> geoms = GeomsOfFrame();
  EXPECT_EQ(geoms.size(), members.size());
  for (int g : geoms) EXPECT_TRUE(seen.count(g)) << "extra geom " << g;
}

TEST_F(FlatRackTest, NamesAndCollisions) {
  for (int g : GeomsOfFrame()) {
    const char* name = mj_id2name(model_.get(), mjOBJ_GEOM, g);
    ASSERT_NE(name, nullptr) << "unnamed geom " << g;
    EXPECT_EQ(std::string(name).rfind("flat_rack_", 0), 0u) << name;
    EXPECT_FALSE(model_->geom_contype[g] == 0 && model_->geom_conaffinity[g] == 0)
        << name << " collides with nothing";
  }
  const std::vector<int> sites = SitesOfFrame();
  EXPECT_EQ(sites.size(), 2u);
  for (int s : sites) {
    const char* name = mj_id2name(model_.get(), mjOBJ_SITE, s);
    ASSERT_NE(name, nullptr) << "unnamed site " << s;
    EXPECT_EQ(std::string(name).rfind("flat_rack_", 0), 0u) << name;
  }
}

TEST_F(FlatRackTest, TwoEnvelopeSites) {
  ExpectSite(fr::TransportEnvelope());
  ExpectSite(fr::CellEnvelope());
}

TEST_F(FlatRackTest, NothingRisesAboveTheDeck) {
  double deck_top = -1.0;
  for (const Box& member : fr::Members()) {
    const double top = member.centre.z() + member.half_size.z();
    EXPECT_LE(top, fr::kDeckHeight + kTol) << member.name;
    if (member.name == "flat_rack_deck") deck_top = top;
  }
  EXPECT_NEAR(deck_top, fr::kDeckHeight, kTol);
  EXPECT_NEAR(fr::kDeckHeight, 0.648, kTol);
}

TEST_F(FlatRackTest, FootprintIsTheFortyFootOutside) {
  Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d hi = Eigen::Vector3d::Constant(-1e9);
  for (const Box& member : fr::Members()) {
    lo = lo.cwiseMin(member.centre - member.half_size);
    hi = hi.cwiseMax(member.centre + member.half_size);
  }
  EXPECT_NEAR(lo.x(), -1.219, kTol);
  EXPECT_NEAR(hi.x(), 1.219, kTol);
  EXPECT_NEAR(lo.y(), -6.096, kTol);
  EXPECT_NEAR(hi.y(), 6.096, kTol);
  EXPECT_NEAR(hi.x() - lo.x(), 2.438, kTol);
  EXPECT_NEAR(hi.y() - lo.y(), 12.192, kTol);
  EXPECT_NEAR(lo.z(), 0.0, kTol);
}

TEST_F(FlatRackTest, DerivedFigures) {
  EXPECT_NEAR(fr::kTransitMassLimitKg, 30000.0 * 0.45359237, 1.0);
  EXPECT_NEAR(fr::kTransitMassLimitKg, 13608.0, kTol);
  EXPECT_NEAR(fr::kStowedHeightOnChassis, 1.219 + 2.896, kTol);
  EXPECT_NEAR(fr::kStowedHeightOnChassis, 4.115, kTol);
}

TEST_F(FlatRackTest, EveryMemberIsInsideTheTransportEnvelope) {
  const Box envelope = fr::TransportEnvelope();
  for (const Box& member : fr::Members()) {
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_GE(member.centre[axis] - member.half_size[axis],
                envelope.centre[axis] - envelope.half_size[axis] - kTol)
          << member.name << " axis " << axis;
      EXPECT_LE(member.centre[axis] + member.half_size[axis],
                envelope.centre[axis] + envelope.half_size[axis] + kTol)
          << member.name << " axis " << axis;
    }
  }
}

// The 20 ft flat racks of base "two_flat_racks" (fr::twenty).
TEST(TwentyFootRacks, FiguresAreTheSourcesAndTheDerivedOnes) {
  namespace tw = fr::twenty;
  EXPECT_NEAR(tw::kLength, 6.058, kTol);
  EXPECT_NEAR(tw::kWidth, 2.438, kTol);
  EXPECT_NEAR(tw::kDeckHeight, 0.370, kTol);
  EXPECT_NEAR(tw::kHeight, 2.896, kTol);
  EXPECT_NEAR(tw::kTareMass, 2900.0, kTol);
  EXPECT_NEAR(tw::kTransitMassLimit, 13608.0, kTol);
  EXPECT_NEAR(tw::kTransitMassLimit, 30000.0 * 0.45359237, 1.0);
  EXPECT_NEAR(tw::PairHalfWidth(0.025), 2.4505, kTol);
}

TEST(TwentyFootRacks, EachRackIsWhereTheFrameSaysAndInsideItsEnvelope) {
  namespace tw = fr::twenty;
  for (bool b : {false, true}) {
    const Eigen::Vector2d c = tw::RackCentre(b);
    EXPECT_NEAR(c.x(), (b ? 1.0 : -1.0) * (0.0125 + 1.219), kTol);
    EXPECT_NEAR(c.y(), 0.0, kTol);
    const std::vector<Box> members = tw::Members(b);
    EXPECT_EQ(members.size(), 5u);
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e9);
    Eigen::Vector3d hi = Eigen::Vector3d::Constant(-1e9);
    const Box env = tw::TransportEnvelope(b);
    for (const Box& m : members) {
      lo = lo.cwiseMin(m.centre - m.half_size);
      hi = hi.cwiseMax(m.centre + m.half_size);
      EXPECT_LE(m.centre.z() + m.half_size.z(), tw::kDeckHeight + kTol) << m.name;
      for (int axis = 0; axis < 3; ++axis) {
        EXPECT_GE(m.centre[axis] - m.half_size[axis],
                  env.centre[axis] - env.half_size[axis] - kTol) << m.name;
        EXPECT_LE(m.centre[axis] + m.half_size[axis],
                  env.centre[axis] + env.half_size[axis] + kTol) << m.name;
      }
    }
    EXPECT_NEAR(hi.x() - lo.x(), 2.438, kTol);
    EXPECT_NEAR(hi.y() - lo.y(), 6.058, kTol);
    EXPECT_NEAR(lo.z(), 0.0, kTol);
    // The long sides face each other across the nominal gap.
    EXPECT_NEAR(b ? lo.x() : hi.x(), b ? 0.0125 : -0.0125, kTol);
    const Box above = tw::CellEnvelope(b);
    EXPECT_NEAR(above.centre.z() - above.half_size.z(), tw::kDeckHeight, kTol);
    EXPECT_NEAR(above.centre.z() + above.half_size.z(), 2.896, kTol);
  }
}

TEST(TwentyFootRacks, RackBMovesByItsShiftAndRackAStays) {
  namespace tw = fr::twenty;
  EXPECT_NEAR(tw::RackCentre(true, 0.025, 0.02, -0.03).x(), 1.2315 + 0.02, kTol);
  EXPECT_NEAR(tw::RackCentre(true, 0.025, 0.02, -0.03).y(), -0.03, kTol);
  EXPECT_NEAR(tw::RackCentre(false, 0.025, 0.02, -0.03).x(), -1.2315, kTol);
  EXPECT_NEAR(tw::Members(true, 0.025, 0.0, 0.025)[0].centre.y(), 0.025, kTol);
}

}  // namespace
