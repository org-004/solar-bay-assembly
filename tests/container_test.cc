// Checks the container: the model has every member and both envelopes that
// src/container.h lists, each a box of the header's size at the header's
// place; the members lie inside the transport envelope and reach it; none
// of them stands in the inside envelope or in the two side openings; the
// header's derived figures are the ones docs/bay-hardware.md states; and in
// the scene a finished bay passes through the -X side opening without
// touching a member, with the clearance at each end the document gives.
//
//   bazel test //:container_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test. The tests print what they measured.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/bay_hardware.h"
#include "src/container.h"
#include "src/mj_handles.h"

namespace {

namespace container = cell::container;
using container::Box;

// An axis-aligned box given by two corners.
struct Extent {
  Eigen::Vector3d min = Eigen::Vector3d::Zero();
  Eigen::Vector3d max = Eigen::Vector3d::Zero();
};

Extent ExtentOf(const Box& box) {
  return {box.centre - box.half_size, box.centre + box.half_size};
}

// How far two extents overlap along the axis where they overlap least;
// zero or less when they are apart or only touch.
double Overlap(const Extent& a, const Extent& b) {
  double least = 1e9;
  for (int axis = 0; axis < 3; ++axis) {
    least = std::min(least, std::min(a.max[axis], b.max[axis]) -
                                std::max(a.min[axis], b.min[axis]));
  }
  return least;
}

class ContainerTest : public ::testing::Test {
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
    container_body_ = mj_name2id(model_.get(), mjOBJ_BODY, "container");
    ASSERT_GE(container_body_, 0) << "no body named container";
  }

  // A point of the world in the container's frame.
  Eigen::Vector3d InContainer(const double* world) const {
    const Eigen::Map<const Eigen::Vector3d> origin(
        data_->xpos + 3 * container_body_);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
        rotation(data_->xmat + 9 * container_body_);
    return rotation.transpose() *
           (Eigen::Map<const Eigen::Vector3d>(world) - origin);
  }

  // True if `body` is the body named `root` or below it.
  bool IsUnder(int body, const char* root) const {
    const int root_id = mj_name2id(model_.get(), mjOBJ_BODY, root);
    while (body > 0) {
      if (body == root_id) return true;
      body = model_->body_parentid[body];
    }
    return false;
  }

  // The extent of a geom in the container's frame. Right for boxes and for
  // cylinders whose axes lie along the frame's, which is all these models
  // have; the test of the bay checks that for its own geoms.
  Extent GeomExtent(int geom) const {
    const mjModel* m = model_.get();
    const Eigen::Vector3d centre = InContainer(data_->geom_xpos + 3 * geom);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
        geom_rotation(data_->geom_xmat + 9 * geom);
    const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
        body_rotation(data_->xmat + 9 * container_body_);
    const Eigen::Matrix3d rotation = body_rotation.transpose() * geom_rotation;
    Eigen::Vector3d half(m->geom_size[3 * geom], m->geom_size[3 * geom + 1],
                         m->geom_size[3 * geom + 2]);
    if (m->geom_type[geom] == mjGEOM_CYLINDER) {
      half = Eigen::Vector3d(half[0], half[0], half[1]);
    }
    const Eigen::Vector3d reach = rotation.cwiseAbs() * half;
    return {centre - reach, centre + reach};
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  int container_body_ = -1;
};

TEST_F(ContainerTest, ModelHasEveryMemberOfTheHeader) {
  const mjModel* m = model_.get();
  const std::vector<Box> members = container::Members();
  EXPECT_EQ(members.size(), 15u);
  double worst = 0.0;
  for (const Box& member : members) {
    const int geom = mj_name2id(m, mjOBJ_GEOM, member.name.c_str());
    ASSERT_GE(geom, 0) << "no geom named " << member.name;
    EXPECT_EQ(m->geom_bodyid[geom], container_body_) << member.name;
    EXPECT_EQ(m->geom_type[geom], mjGEOM_BOX) << member.name;
    // A member collides: it takes part in contacts.
    EXPECT_NE(m->geom_contype[geom], 0) << member.name;
    EXPECT_NE(m->geom_conaffinity[geom], 0) << member.name;
    const Extent extent = GeomExtent(geom);
    const Extent expected = ExtentOf(member);
    const double error = std::max((extent.min - expected.min).cwiseAbs().maxCoeff(),
                                  (extent.max - expected.max).cwiseAbs().maxCoeff());
    worst = std::max(worst, error);
    EXPECT_LT(error, 1e-9) << member.name;
  }
  // The container's body holds these geoms and no other, and every name of
  // the part starts with container_.
  int geoms = 0;
  for (int geom = 0; geom < m->ngeom; ++geom) {
    if (!IsUnder(m->geom_bodyid[geom], "container")) continue;
    ++geoms;
    const char* name = mj_id2name(m, mjOBJ_GEOM, geom);
    ASSERT_NE(name, nullptr);
    EXPECT_EQ(std::strncmp(name, "container_", 10), 0) << name;
  }
  EXPECT_EQ(geoms, 15);
  // The container is structure only: nothing in it moves.
  for (int joint = 0; joint < m->njnt; ++joint) {
    EXPECT_FALSE(IsUnder(m->jnt_bodyid[joint], "container"));
  }
  std::printf("members found: %d of %zu, largest difference from the header "
              "%.3g m\n", geoms, members.size(), worst);
}

TEST_F(ContainerTest, ModelHasBothEnvelopes) {
  const mjModel* m = model_.get();
  for (const Box& envelope :
       {container::TransportEnvelope(), container::InsideEnvelope()}) {
    const int site = mj_name2id(m, mjOBJ_SITE, envelope.name.c_str());
    ASSERT_GE(site, 0) << "no site named " << envelope.name;
    EXPECT_EQ(m->site_bodyid[site], container_body_);
    EXPECT_EQ(m->site_type[site], mjGEOM_BOX);
    const Eigen::Vector3d centre = InContainer(data_->site_xpos + 3 * site);
    const Eigen::Map<const Eigen::Vector3d> half(m->site_size + 3 * site);
    EXPECT_LT((centre - envelope.centre).norm(), 1e-9) << envelope.name;
    EXPECT_LT((half - envelope.half_size).norm(), 1e-9) << envelope.name;
    std::printf("%s: %.3f x %.3f x %.3f m\n", envelope.name.c_str(),
                2.0 * half[0], 2.0 * half[1], 2.0 * half[2]);
  }
  // The transport envelope is the standard's outside of a 40 ft high cube.
  const Box transport = container::TransportEnvelope();
  EXPECT_DOUBLE_EQ(2.0 * transport.half_size[0], 2.438);
  EXPECT_DOUBLE_EQ(2.0 * transport.half_size[1], 12.192);
  EXPECT_DOUBLE_EQ(2.0 * transport.half_size[2], 2.896);
  EXPECT_DOUBLE_EQ(transport.centre[2] - transport.half_size[2], 0.0);
}

TEST_F(ContainerTest, MembersLieInsideTheTransportEnvelope) {
  const Extent envelope = ExtentOf(container::TransportEnvelope());
  Extent hull{Eigen::Vector3d::Constant(1e9), Eigen::Vector3d::Constant(-1e9)};
  for (const Box& member : container::Members()) {
    const int geom = mj_name2id(model_.get(), mjOBJ_GEOM, member.name.c_str());
    ASSERT_GE(geom, 0);
    const Extent extent = GeomExtent(geom);
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_GE(extent.min[axis], envelope.min[axis] - 1e-9) << member.name;
      EXPECT_LE(extent.max[axis], envelope.max[axis] + 1e-9) << member.name;
    }
    hull.min = hull.min.cwiseMin(extent.min);
    hull.max = hull.max.cwiseMax(extent.max);
  }
  // The posts reach every face of the envelope: the members fill the
  // standard's outside and do not stop short of it.
  EXPECT_LT((hull.min - envelope.min).cwiseAbs().maxCoeff(), 1e-9);
  EXPECT_LT((hull.max - envelope.max).cwiseAbs().maxCoeff(), 1e-9);
  std::printf("members span %.3f x %.3f x %.3f m\n", hull.max[0] - hull.min[0],
              hull.max[1] - hull.min[1], hull.max[2] - hull.min[2]);
}

TEST_F(ContainerTest, InsideEnvelopeAndSideOpeningsAreClear) {
  const Extent inside = ExtentOf(container::InsideEnvelope());
  // A side opening as a slab: from the inside envelope's side out to the
  // container's outside, between the posts, from the floor to the top side
  // rail.
  const double hx = container::kWidth / 2.0;
  Extent openings[2];
  for (int side = 0; side < 2; ++side) {
    openings[side].min = Eigen::Vector3d(
        side == 0 ? -hx : inside.max[0], -container::kSideOpeningLength / 2.0,
        container::kFloorHeight);
    openings[side].max = Eigen::Vector3d(
        side == 0 ? inside.min[0] : hx, container::kSideOpeningLength / 2.0,
        container::kFloorHeight + container::kSideOpeningHeight);
  }
  double worst = -1e9;
  for (const Box& member : container::Members()) {
    const int geom = mj_name2id(model_.get(), mjOBJ_GEOM, member.name.c_str());
    ASSERT_GE(geom, 0);
    const Extent extent = GeomExtent(geom);
    for (const Extent& space : {inside, openings[0], openings[1]}) {
      const double overlap = Overlap(extent, space);
      worst = std::max(worst, overlap);
      EXPECT_LE(overlap, 1e-9) << member.name << " stands in a clear space";
    }
  }
  std::printf("deepest a member reaches into a clear space: %.3g m\n", worst);
}

TEST_F(ContainerTest, HeaderGivesTheDocumentsFigures) {
  // docs/bay-hardware.md, "The container this project assumes".
  EXPECT_NEAR(container::kSideOpeningLength, 11.830, 1e-9);
  EXPECT_NEAR(container::kSideOpeningHeight, 2.240, 1e-9);
  EXPECT_DOUBLE_EQ(container::kRoofOpeningLength, 0.0);
  EXPECT_NEAR(container::kInsideLength, 11.830, 1e-9);
  EXPECT_NEAR(container::kInsideWidth, 2.330, 1e-9);
  EXPECT_NEAR(container::kInsideHeight, 2.240, 1e-9);
  EXPECT_NEAR(container::kFloorHeight, 0.450, 1e-9);
  EXPECT_NEAR(container::kFloorHeight + container::kChassisDeckHeight, 1.669,
              1e-9);
  EXPECT_NEAR(container::kTopRailUnderside, 2.690, 1e-9);
  EXPECT_NEAR(container::kTareMass, 6600.0, 1e-9);
  EXPECT_NEAR(container::kTransportGrossMass, 23800.0, 1e-9);
  EXPECT_NEAR(container::kCellMassBudget, 17200.0, 1e-9);
  // A bay is wider than the inside envelope, which is why it leaves
  // through a side, and it is shorter than the side opening.
  EXPECT_GT(cell::bay::kBayWidth, container::kInsideWidth);
  EXPECT_LT(cell::bay::kBayLength, container::kSideOpeningLength);
  EXPECT_NEAR((container::kSideOpeningLength - cell::bay::kBayLength) / 2.0,
              0.035, 1e-9);
}

TEST_F(ContainerTest, BayPassesThroughTheSideOpening) {
  const mjModel* m = model_.get();
  ASSERT_GE(mj_name2id(m, mjOBJ_BODY, "bay"), 0) << "the scene has no bay";
  // The bay's geoms, in the container's frame.
  std::vector<Extent> bay;
  Extent hull{Eigen::Vector3d::Constant(1e9), Eigen::Vector3d::Constant(-1e9)};
  for (int geom = 0; geom < m->ngeom; ++geom) {
    if (!IsUnder(m->geom_bodyid[geom], "bay")) continue;
    bay.push_back(GeomExtent(geom));
    hull.min = hull.min.cwiseMin(bay.back().min);
    hull.max = hull.max.cwiseMax(bay.back().max);
  }
  ASSERT_FALSE(bay.empty());
  // It crosses the plane of the -X side, part inside and part outside.
  const double side = -container::kWidth / 2.0;
  EXPECT_LT(hull.min[0], side);
  EXPECT_GT(hull.max[0], side);
  // It fits the opening: between the posts and between floor and top rail.
  const double end_clearance =
      std::min(hull.min[1] + container::kSideOpeningLength / 2.0,
               container::kSideOpeningLength / 2.0 - hull.max[1]);
  EXPECT_NEAR(end_clearance, 0.035, 1e-6);
  EXPECT_GT(hull.min[2], container::kFloorHeight);
  EXPECT_LT(hull.max[2], container::kTopRailUnderside);
  // And no geom of the bay overlaps a member of the container.
  double closest = -1e9;
  for (const Box& member : container::Members()) {
    const int geom = mj_name2id(m, mjOBJ_GEOM, member.name.c_str());
    ASSERT_GE(geom, 0);
    const Extent extent = GeomExtent(geom);
    for (const Extent& part : bay) {
      const double overlap = Overlap(extent, part);
      closest = std::max(closest, overlap);
      EXPECT_LE(overlap, 0.0) << member.name << " overlaps the bay";
    }
  }
  std::printf("bay geoms: %zu, bay spans x %.3f to %.3f, y %.3f to %.3f, "
              "z %.3f to %.3f m in the container's frame\n",
              bay.size(), hull.min[0], hull.max[0], hull.min[1], hull.max[1],
              hull.min[2], hull.max[2]);
  std::printf("clearance to the posts at each end: %.4f m; nearest member "
              "is %.4f m from the bay\n", end_clearance, -closest);
}

}  // namespace
