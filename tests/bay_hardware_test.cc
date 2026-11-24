// Checks that the model of one bay, models/bay.xml, and src/bay_hardware.h
// describe the same bay:
//   - every site of Fastenings(), Locators() and Holds() is in the model, on
//     the body the header names, at the header's position and orientation in
//     the tube's frame (body "bay"), and there are no other such sites;
//   - the two sites of each fastening lie on the tool's axis, the distance
//     between them along it is the header's stack, and a locator's tab is
//     over its hole;
//   - the model has as many rails, modules and fastenings as the header and
//     the document say;
//   - every rail and module stands at its pose of the header, each module has
//     its laminate and its frame, and the tube has the header's radius;
//   - nothing moves: no joint, no actuator, no equality, and every name in
//     the bay starts with "bay_";
//   - the bay, each rail and each module weigh what the header says;
//   - the box round all the bay's geoms is the header's BayBoxMin/BayBoxMax;
//   - no two parts overlap. MuJoCo makes no contacts between bodies fixed to
//     the world, so this test gives every part a slide joint in a copy of the
//     model, and shows with a module moved 1 mm down that an overlap is seen.
//
//   bazel test //:bay_hardware_test --test_output=all
//
// The scene's path comes from the MODEL_PATH environment variable, which
// BUILD.bazel sets for this test. The tests print what they measured.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <regex>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/bay_hardware.h"
#include "src/mj_handles.h"

namespace {

namespace bay = cell::bay;

constexpr double kPositionTolerance = 1e-6;  // m
constexpr double kAngleTolerance = 1e-6;     // rad
constexpr double kSizeTolerance = 1e-9;      // m
constexpr double kMassTolerance = 0.01;      // relative
constexpr double kBoxTolerance = 1e-3;       // m
// The deepest overlap of two parts that is still taken as touching.
constexpr double kMaxPenetration = 1e-4;     // m
// The control's module is moved this far down, so its overlap with its
// rails must be seen as deeper than half of it.
constexpr double kControlShift = 1e-3;       // m
constexpr double kControlDepth = 5e-4;       // m

bool StartsWith(const std::string& s, const std::string& prefix) {
  return s.compare(0, prefix.size(), prefix) == 0;
}

std::string Name(const mjModel& m, mjtObj type, int id) {
  const char* name = mj_id2name(&m, type, id);
  return name == nullptr ? std::string() : std::string(name);
}

Eigen::Isometry3d MakePose(const mjtNum* pos, const mjtNum* xmat) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(xmat);
  pose.translation() = Eigen::Vector3d(pos[0], pos[1], pos[2]);
  return pose;
}

// The angle of the rotation that takes `a` to `b`.
double RotationAngle(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  return Eigen::AngleAxisd(a.transpose() * b).angle();
}

// The angle between two directions.
double DirectionAngle(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

// True if body `body` is `root` or below it.
bool InSubtree(const mjModel& m, int body, int root) {
  while (body != 0) {
    if (body == root) return true;
    body = m.body_parentid[body];
  }
  return root == 0;
}

// Builds the scene again from its file with a slide joint along Z on every
// body of the tube, the rails and the modules, so that MuJoCo makes contacts
// between them. Moves module 0 down by `module_0_drop` first. Returns an
// empty handle and writes the reason to `error` on failure.
cell::ModelPtr LoadWithSlides(const std::string& path, double module_0_drop,
                              int* joints, std::string* error) {
  char message[1000] = "";
  mjSpec* spec = mj_parseXML(path.c_str(), nullptr, message, sizeof(message));
  if (spec == nullptr) {
    *error = message;
    return nullptr;
  }
  mjsBody* world = mjs_findBody(spec, "world");
  std::vector<mjsBody*> bodies;
  for (mjsElement* e = mjs_firstChild(world, mjOBJ_BODY, 1); e != nullptr;
       e = mjs_nextChild(world, e, 1)) {
    const std::string name = mjs_getString(mjs_getName(e));
    if (StartsWith(name, "bay_tube") || StartsWith(name, "bay_rail_") ||
        StartsWith(name, "bay_module_")) {
      bodies.push_back(mjs_asBody(e));
    }
  }
  for (mjsBody* body : bodies) {
    mjsJoint* joint = mjs_addJoint(body, nullptr);
    joint->type = mjJNT_SLIDE;
    joint->axis[0] = 0.0;
    joint->axis[1] = 0.0;
    joint->axis[2] = 1.0;
  }
  *joints = static_cast<int>(bodies.size());
  if (module_0_drop != 0.0) {
    mjsBody* module = mjs_findBody(spec, bay::ModuleName(0).c_str());
    if (module == nullptr) {
      *error = "no body " + bay::ModuleName(0);
      mj_deleteSpec(spec);
      return nullptr;
    }
    module->pos[2] -= module_0_drop;
  }
  cell::ModelPtr model(mj_compile(spec, nullptr));
  if (!model) *error = mjs_getError(spec);
  mj_deleteSpec(spec);
  return model;
}

class BayHardwareTest : public ::testing::Test {
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

    bay_ = mj_name2id(model_.get(), mjOBJ_BODY, "bay");
    ASSERT_GE(bay_, 0) << "no body bay";
    bay_inverse_ = MakePose(data_->xpos + 3 * bay_, data_->xmat + 9 * bay_).inverse();
  }

  int Id(mjtObj type, const std::string& name) const {
    return mj_name2id(model_.get(), type, name.c_str());
  }

  // Poses in the tube's frame, the frame of body "bay".
  Eigen::Isometry3d SitePose(int site) const {
    return bay_inverse_ *
           MakePose(data_->site_xpos + 3 * site, data_->site_xmat + 9 * site);
  }
  Eigen::Isometry3d BodyPose(int body) const {
    return bay_inverse_ *
           MakePose(data_->xpos + 3 * body, data_->xmat + 9 * body);
  }
  Eigen::Isometry3d GeomPose(int geom) const {
    return bay_inverse_ *
           MakePose(data_->geom_xpos + 3 * geom, data_->geom_xmat + 9 * geom);
  }

  // The bodies below "bay", not "bay" itself.
  std::vector<int> BayBodies() const {
    std::vector<int> bodies;
    for (int b = 0; b < model_->nbody; ++b) {
      if (b != bay_ && InSubtree(*model_, b, bay_)) bodies.push_back(b);
    }
    return bodies;
  }

  std::string path_;
  cell::ModelPtr model_;
  cell::DataPtr data_;
  int bay_ = -1;
  Eigen::Isometry3d bay_inverse_ = Eigen::Isometry3d::Identity();
};

// Largest errors of a set of sites against the header.
struct SiteErrors {
  int found = 0;
  double position = 0.0;
  double angle = 0.0;
};

TEST_F(BayHardwareTest, Sites) {
  // Checks site `name` on body `body` against `expected`.
  auto check = [&](const std::string& name, const std::string& body,
                   const Eigen::Isometry3d& expected, SiteErrors* errors) {
    const int site = Id(mjOBJ_SITE, name);
    EXPECT_GE(site, 0) << "no site " << name;
    if (site < 0) return -1;
    ++errors->found;
    EXPECT_EQ(Name(*model_, mjOBJ_BODY, model_->site_bodyid[site]), body)
        << name;
    const Eigen::Isometry3d pose = SitePose(site);
    const double position = (pose.translation() - expected.translation()).norm();
    const double angle = RotationAngle(pose.linear(), expected.linear());
    EXPECT_LE(position, kPositionTolerance) << name;
    EXPECT_LE(angle, kAngleTolerance) << name;
    errors->position = std::max(errors->position, position);
    errors->angle = std::max(errors->angle, angle);
    return site;
  };

  SiteErrors fasten, locate, hold;
  std::set<std::string> expected_fasten, expected_locate, expected_hold;
  for (const bay::Fastening& f : bay::Fastenings()) {
    check(f.name + "_a", f.part_a, f.pose_a, &fasten);
    check(f.name + "_b", f.part_b, f.pose_b, &fasten);
    expected_fasten.insert({f.name + "_a", f.name + "_b"});
  }
  for (const bay::Locator& l : bay::Locators()) {
    check(l.name + "_a", l.part_a, l.pose_a, &locate);
    check(l.name + "_b", l.part_b, l.pose_b, &locate);
    expected_locate.insert({l.name + "_a", l.name + "_b"});
  }
  double hold_size_error = 0.0;
  for (const bay::Hold& h : bay::Holds()) {
    const int site = check(h.name, bay::TubeName(), h.pose, &hold);
    expected_hold.insert(h.name);
    if (site < 0) continue;
    EXPECT_EQ(model_->site_type[site], mjGEOM_CYLINDER) << h.name;
    const double error =
        std::max(std::abs(model_->site_size[3 * site] - h.radius),
                 std::abs(model_->site_size[3 * site + 1] - h.half_length));
    EXPECT_LE(error, kSizeTolerance) << h.name;
    hold_size_error = std::max(hold_size_error, error);
  }

  // No sites of these kinds beyond the header's.
  int fasten_in_model = 0, locate_in_model = 0, hold_in_model = 0;
  for (int s = 0; s < model_->nsite; ++s) {
    const std::string name = Name(*model_, mjOBJ_SITE, s);
    if (StartsWith(name, "bay_fasten_")) {
      ++fasten_in_model;
      EXPECT_TRUE(expected_fasten.count(name)) << "unexpected site " << name;
    } else if (StartsWith(name, "bay_locate_")) {
      ++locate_in_model;
      EXPECT_TRUE(expected_locate.count(name)) << "unexpected site " << name;
    } else if (StartsWith(name, "bay_hold_")) {
      ++hold_in_model;
      EXPECT_TRUE(expected_hold.count(name)) << "unexpected site " << name;
    }
  }
  EXPECT_EQ(fasten_in_model, 102);
  EXPECT_EQ(locate_in_model, 22);
  EXPECT_EQ(hold_in_model, 12);
  EXPECT_EQ(fasten.found, 2 * bay::kFasteningCount);
  EXPECT_EQ(locate.found, 2 * bay::kLocatorCount);
  EXPECT_EQ(hold.found, bay::kHoldCount);

  std::printf("fastening sites: %d of the header's found, %d named bay_fasten_ "
              "in the model; largest error %.3g m, %.3g rad\n",
              fasten.found, fasten_in_model, fasten.position, fasten.angle);
  std::printf("locator sites:   %d of the header's found, %d named bay_locate_ "
              "in the model; largest error %.3g m, %.3g rad\n",
              locate.found, locate_in_model, locate.position, locate.angle);
  std::printf("hold sites:      %d of the header's found, %d named bay_hold_ "
              "in the model; largest error %.3g m, %.3g rad, size %.3g m\n",
              hold.found, hold_in_model, hold.position, hold.angle,
              hold_size_error);
}

TEST_F(BayHardwareTest, Pairs) {
  // Measures the two sites `a` and `b` against `axis`: the angle between
  // their z axes and from a's z axis to `axis`, and b's offset from a across
  // and along `axis`.
  struct Pair {
    double parallel = 0.0;
    double to_axis = 0.0;
    double across = 0.0;
    double along = 0.0;
  };
  auto measure = [&](const std::string& name,
                     const Eigen::Vector3d& axis, Pair* pair) {
    const int a = Id(mjOBJ_SITE, name + "_a");
    const int b = Id(mjOBJ_SITE, name + "_b");
    EXPECT_GE(a, 0) << name;
    EXPECT_GE(b, 0) << name;
    if (a < 0 || b < 0) return false;
    const Eigen::Isometry3d pa = SitePose(a);
    const Eigen::Isometry3d pb = SitePose(b);
    const Eigen::Vector3d za = pa.linear().col(2);
    const Eigen::Vector3d zb = pb.linear().col(2);
    pair->parallel = DirectionAngle(za, zb);
    pair->to_axis = DirectionAngle(za, axis);
    const Eigen::Vector3d d = pb.translation() - pa.translation();
    pair->along = d.dot(axis);
    pair->across = (d - pair->along * axis).norm();
    return true;
  };

  int fastenings = 0;
  double worst_parallel = 0, worst_across = 0, worst_stack = 0;
  for (const bay::Fastening& f : bay::Fastenings()) {
    Pair p;
    if (!measure(f.name, f.tool_axis, &p)) continue;
    ++fastenings;
    EXPECT_LE(p.parallel, kAngleTolerance) << f.name;
    EXPECT_LE(p.to_axis, kAngleTolerance) << f.name;
    EXPECT_LE(p.across, f.clearance) << f.name;
    EXPECT_LT(p.across, kPositionTolerance) << f.name;
    EXPECT_NEAR(p.along, f.stack, kPositionTolerance) << f.name;
    worst_parallel = std::max({worst_parallel, p.parallel, p.to_axis});
    worst_across = std::max(worst_across, p.across);
    worst_stack = std::max(worst_stack, std::abs(p.along - f.stack));
  }
  int locators = 0;
  double locator_parallel = 0, locator_across = 0;
  for (const bay::Locator& l : bay::Locators()) {
    Pair p;
    if (!measure(l.name, -Eigen::Vector3d::UnitZ(), &p)) continue;
    ++locators;
    EXPECT_LE(p.parallel, kAngleTolerance) << l.name;
    EXPECT_LE(p.to_axis, kAngleTolerance) << l.name;
    EXPECT_LE(p.across, l.clearance) << l.name;
    locator_parallel = std::max({locator_parallel, p.parallel, p.to_axis});
    locator_across = std::max(locator_across, p.across);
  }
  EXPECT_EQ(fastenings, bay::kFasteningCount);
  EXPECT_EQ(locators, bay::kLocatorCount);
  std::printf("fastening pairs %d: axes within %.3g rad, across %.3g m, "
              "stack error %.3g m\n",
              fastenings, worst_parallel, worst_across, worst_stack);
  std::printf("locator pairs %d: axes within %.3g rad, across %.3g m\n",
              locators, locator_parallel, locator_across);
}

TEST_F(BayHardwareTest, Counts) {
  const std::regex rail(R"(bay_rail_\d+)");
  const std::regex module(R"(bay_module_\d+)");
  const std::regex rail_fastening(R"(bay_fasten_rail_\d+_\d+_a)");
  const std::regex module_fastening(R"(bay_fasten_module_\d+_\d+_a)");
  int rails = 0, modules = 0, rail_fastenings = 0, module_fastenings = 0;
  for (int b = 0; b < model_->nbody; ++b) {
    const std::string name = Name(*model_, mjOBJ_BODY, b);
    if (std::regex_match(name, rail)) ++rails;
    if (std::regex_match(name, module)) ++modules;
  }
  for (int s = 0; s < model_->nsite; ++s) {
    const std::string name = Name(*model_, mjOBJ_SITE, s);
    if (std::regex_match(name, rail_fastening)) ++rail_fastenings;
    if (std::regex_match(name, module_fastening)) ++module_fastenings;
  }
  const int fastenings = rail_fastenings + module_fastenings;
  std::printf("model: rails %d, modules %d, fastenings rail %d, module %d, "
              "all %d\n",
              rails, modules, rail_fastenings, module_fastenings, fastenings);

  EXPECT_EQ(rails, bay::kRailCount);
  EXPECT_EQ(modules, bay::kModuleCount);
  EXPECT_EQ(rail_fastenings, bay::kRailFasteningCount);
  EXPECT_EQ(module_fastenings, bay::kModuleFasteningCount);
  EXPECT_EQ(fastenings, bay::kFasteningCount);
  // The document's numbers.
  EXPECT_EQ(rails, 11);
  EXPECT_EQ(modules, 10);
  EXPECT_EQ(rail_fastenings, 11);
  EXPECT_EQ(module_fastenings, 40);
  EXPECT_EQ(fastenings, 51);
}

TEST_F(BayHardwareTest, Bodies) {
  double position = 0.0, angle = 0.0;
  auto check = [&](const std::string& name, const Eigen::Isometry3d& expected) {
    const int body = Id(mjOBJ_BODY, name);
    EXPECT_GE(body, 0) << "no body " << name;
    if (body < 0) return -1;
    const Eigen::Isometry3d pose = BodyPose(body);
    const double p = (pose.translation() - expected.translation()).norm();
    const double a = RotationAngle(pose.linear(), expected.linear());
    EXPECT_LE(p, kPositionTolerance) << name;
    EXPECT_LE(a, kAngleTolerance) << name;
    position = std::max(position, p);
    angle = std::max(angle, a);
    return body;
  };
  for (int k = 0; k < bay::kRailCount; ++k) check(bay::RailName(k), bay::RailPose(k));
  int fewest_frame_geoms = 1 << 30;
  for (int m = 0; m < bay::kModuleCount; ++m) {
    const std::string name = bay::ModuleName(m);
    const int body = check(name, bay::ModulePose(m));
    if (body < 0) continue;
    const int laminate = Id(mjOBJ_GEOM, name + "_laminate");
    EXPECT_GE(laminate, 0) << "no laminate on " << name;
    if (laminate >= 0) {
      EXPECT_EQ(model_->geom_bodyid[laminate], body);
    }
    int frame_geoms = 0;
    for (int g = 0; g < model_->ngeom; ++g) {
      if (model_->geom_bodyid[g] == body &&
          StartsWith(Name(*model_, mjOBJ_GEOM, g), name + "_frame_")) {
        ++frame_geoms;
      }
    }
    EXPECT_GE(frame_geoms, 4) << name;
    fewest_frame_geoms = std::min(fewest_frame_geoms, frame_geoms);
  }
  const int tube = Id(mjOBJ_GEOM, bay::TubeName());
  ASSERT_GE(tube, 0) << "no geom " << bay::TubeName();
  EXPECT_EQ(model_->geom_type[tube], mjGEOM_CYLINDER);
  EXPECT_NEAR(model_->geom_size[3 * tube], bay::kTubeRadius, kSizeTolerance);
  std::printf("rails and modules: largest pose error %.3g m, %.3g rad; "
              "fewest frame geoms on a module %d; tube radius %.4f m\n",
              position, angle, fewest_frame_geoms, model_->geom_size[3 * tube]);
}

TEST_F(BayHardwareTest, NoJoints) {
  EXPECT_EQ(model_->nq, 0);
  EXPECT_EQ(model_->nu, 0);
  EXPECT_EQ(model_->neq, 0);
  int bodies = 0, geoms = 0, sites = 0;
  for (int b : BayBodies()) {
    ++bodies;
    const std::string name = Name(*model_, mjOBJ_BODY, b);
    EXPECT_TRUE(StartsWith(name, "bay_")) << "body " << b << " '" << name << "'";
  }
  for (int g = 0; g < model_->ngeom; ++g) {
    if (!InSubtree(*model_, model_->geom_bodyid[g], bay_)) continue;
    ++geoms;
    const std::string name = Name(*model_, mjOBJ_GEOM, g);
    EXPECT_TRUE(StartsWith(name, "bay_")) << "geom " << g << " '" << name << "'";
  }
  for (int s = 0; s < model_->nsite; ++s) {
    if (!InSubtree(*model_, model_->site_bodyid[s], bay_)) continue;
    ++sites;
    const std::string name = Name(*model_, mjOBJ_SITE, s);
    EXPECT_TRUE(StartsWith(name, "bay_")) << "site " << s << " '" << name << "'";
  }
  std::printf("nq %d, nu %d, neq %d; names checked: %d bodies, %d geoms, "
              "%d sites\n",
              static_cast<int>(model_->nq), static_cast<int>(model_->nu),
              static_cast<int>(model_->neq), bodies, geoms, sites);
}

TEST_F(BayHardwareTest, Mass) {
  double total = model_->body_mass[bay_];
  for (int b : BayBodies()) total += model_->body_mass[b];
  EXPECT_NEAR(total, bay::kBayMass, kMassTolerance * bay::kBayMass);
  double rail_min = 1e9, rail_max = 0, module_min = 1e9, module_max = 0;
  for (int k = 0; k < bay::kRailCount; ++k) {
    const int body = Id(mjOBJ_BODY, bay::RailName(k));
    ASSERT_GE(body, 0);
    const double mass = model_->body_mass[body];
    EXPECT_NEAR(mass, bay::kRailMass, kMassTolerance * bay::kRailMass);
    rail_min = std::min(rail_min, mass);
    rail_max = std::max(rail_max, mass);
  }
  for (int m = 0; m < bay::kModuleCount; ++m) {
    const int body = Id(mjOBJ_BODY, bay::ModuleName(m));
    ASSERT_GE(body, 0);
    const double mass = model_->body_mass[body];
    EXPECT_NEAR(mass, bay::kModuleMass, kMassTolerance * bay::kModuleMass);
    module_min = std::min(module_min, mass);
    module_max = std::max(module_max, mass);
  }
  std::printf("mass: bay %.3f kg (header %.3f), rails %.4f to %.4f kg, "
              "modules %.4f to %.4f kg\n",
              total, bay::kBayMass, rail_min, rail_max, module_min, module_max);
}

TEST_F(BayHardwareTest, BoundingBox) {
  Eigen::Vector3d low = Eigen::Vector3d::Constant(1e9);
  Eigen::Vector3d high = Eigen::Vector3d::Constant(-1e9);
  int geoms = 0;
  for (int g = 0; g < model_->ngeom; ++g) {
    if (!InSubtree(*model_, model_->geom_bodyid[g], bay_)) continue;
    const Eigen::Isometry3d pose = GeomPose(g);
    const Eigen::Matrix3d& r = pose.linear();
    const mjtNum* size = model_->geom_size + 3 * g;
    Eigen::Vector3d half;
    if (model_->geom_type[g] == mjGEOM_BOX) {
      half = r.cwiseAbs() * Eigen::Vector3d(size[0], size[1], size[2]);
    } else if (model_->geom_type[g] == mjGEOM_CYLINDER) {
      // A disc of the radius at each end of the axis: along each direction
      // the axis's part of the half length plus the radius times the rest.
      for (int j = 0; j < 3; ++j) {
        const double c = std::abs(r(j, 2));
        half[j] = c * size[1] + size[0] * std::sqrt(std::max(0.0, 1.0 - c * c));
      }
    } else {
      ADD_FAILURE() << "geom " << Name(*model_, mjOBJ_GEOM, g)
                    << " is neither a box nor a cylinder";
      continue;
    }
    ++geoms;
    low = low.cwiseMin(pose.translation() - half);
    high = high.cwiseMax(pose.translation() + half);
  }
  const Eigen::Vector3d expected_low = bay::BayBoxMin();
  const Eigen::Vector3d expected_high = bay::BayBoxMax();
  const double error = std::max((low - expected_low).cwiseAbs().maxCoeff(),
                                (high - expected_high).cwiseAbs().maxCoeff());
  for (int j = 0; j < 3; ++j) {
    EXPECT_NEAR(low[j], expected_low[j], kBoxTolerance) << "axis " << j;
    EXPECT_NEAR(high[j], expected_high[j], kBoxTolerance) << "axis " << j;
  }
  std::printf("box of %d geoms: %.4f %.4f %.4f to %.4f %.4f %.4f m, "
              "largest error %.3g m\n",
              geoms, low.x(), low.y(), low.z(), high.x(), high.y(), high.z(),
              error);
}

TEST_F(BayHardwareTest, NoOverlap) {
  // As loaded, every part is fixed to the world, so MuJoCo makes no
  // contacts at all, overlapping or not: this check alone would prove
  // nothing.
  EXPECT_EQ(data_->ncon, 0);

  std::string error;
  int joints = 0;
  cell::ModelPtr model = LoadWithSlides(path_, 0.0, &joints, &error);
  ASSERT_TRUE(model) << error;
  EXPECT_EQ(joints, 1 + bay::kRailCount + bay::kModuleCount);
  cell::DataPtr data = cell::MakeData(*model);
  ASSERT_TRUE(data);
  mj_forward(model.get(), data.get());

  const int floor = mj_name2id(model.get(), mjOBJ_GEOM, "floor");
  int between_parts = 0;
  double deepest = 0.0;
  std::string deepest_pair = "none";
  for (int i = 0; i < data->ncon; ++i) {
    const mjContact& c = data->contact[i];
    if (c.geom[0] == floor || c.geom[1] == floor) continue;
    const int b1 = model->geom_bodyid[c.geom[0]];
    const int b2 = model->geom_bodyid[c.geom[1]];
    const std::string n1 = Name(*model, mjOBJ_BODY, b1);
    const std::string n2 = Name(*model, mjOBJ_BODY, b2);
    if (b1 == b2 || !StartsWith(n1, "bay_") || !StartsWith(n2, "bay_")) continue;
    ++between_parts;
    const std::string pair = Name(*model, mjOBJ_GEOM, c.geom[0]) + " / " +
                             Name(*model, mjOBJ_GEOM, c.geom[1]);
    EXPECT_GE(c.dist, -kMaxPenetration) << pair;
    if (c.dist < deepest) {
      deepest = c.dist;
      deepest_pair = pair;
    }
  }
  EXPECT_GT(data->ncon, 0);
  EXPECT_GT(between_parts, 0);
  std::printf("with %d slide joints: ncon %d, %d between two parts, deepest "
              "penetration %.3g m (%s)\n",
              joints, data->ncon, between_parts, -deepest, deepest_pair.c_str());

  // The control: module 0 moved down into its rails must be seen.
  cell::ModelPtr control = LoadWithSlides(path_, kControlShift, &joints, &error);
  ASSERT_TRUE(control) << error;
  cell::DataPtr control_data = cell::MakeData(*control);
  ASSERT_TRUE(control_data);
  mj_forward(control.get(), control_data.get());
  const int module_0 =
      mj_name2id(control.get(), mjOBJ_BODY, bay::ModuleName(0).c_str());
  ASSERT_GE(module_0, 0);
  double control_deepest = 0.0;
  for (int i = 0; i < control_data->ncon; ++i) {
    const mjContact& c = control_data->contact[i];
    const int b1 = control->geom_bodyid[c.geom[0]];
    const int b2 = control->geom_bodyid[c.geom[1]];
    const int other = b1 == module_0 ? b2 : (b2 == module_0 ? b1 : -1);
    if (other < 0 || !StartsWith(Name(*control, mjOBJ_BODY, other), "bay_rail_")) {
      continue;
    }
    control_deepest = std::min(control_deepest, c.dist);
  }
  EXPECT_LT(control_deepest, -kControlDepth);
  std::printf("control, module 0 moved %.4f m down: ncon %d, deepest "
              "penetration with a rail %.3g m\n",
              kControlShift, control_data->ncon, -control_deepest);
}

}  // namespace
