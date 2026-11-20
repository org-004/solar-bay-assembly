#include "src/collision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/mj_handles.h"
#include "src/pallet.h"

namespace cell {
namespace {

// A query outside a joint's range by less than this still counts as in it,
// so that the ends of the ranges themselves can be checked.
constexpr double kRangeSlack = 1e-9;  // rad

// The contact bits of the private model when obstacle pairs are skipped.
// MuJoCo tests a pair of geoms only if the contype of one shares a bit with
// the conaffinity of the other. One bit per role; each role's conaffinity
// names the roles it is to be tested against:
//   link      against everything
//   tool      against links and obstacles, not the held module
//   held      against links and obstacles, not the tool
//   obstacle  against links, tool and held module, not other obstacles
constexpr int kLinkBit = 1;
constexpr int kToolBit = 2;
constexpr int kHeldBit = 4;
constexpr int kObstacleBit = 8;

bool Collides(int contype, int conaffinity, int other_contype,
              int other_conaffinity) {
  return (contype & other_conaffinity) != 0 ||
         (other_contype & conaffinity) != 0;
}

Eigen::Vector3d Vec3(const mjtNum* v) { return {v[0], v[1], v[2]}; }

}  // namespace

std::optional<CollisionChecker> CollisionChecker::Create(
    const mjModel& model, const CollisionOptions& options, std::string* error) {
  auto fail = [&](const std::string& why) -> std::optional<CollisionChecker> {
    if (error != nullptr) *error = why;
    return std::nullopt;
  };
  std::optional<Arm> arm = FindArm(model);
  if (!arm) return fail("no arm in the model (see FindArm in src/arm.h)");
  if (model.npair > 0) {
    return fail("the model has explicit contact pairs, whose own margins "
                "this checker does not set");
  }
  if (model.nflex > 0) return fail("the model has flexes; not supported");

  CollisionChecker checker;
  checker.arm_ = *arm;
  checker.model_ = ModelPtr(mj_copyModel(nullptr, &model));
  if (!checker.model_) return fail("mj_copyModel failed");
  mjModel& m = *checker.model_;

  // Settings that would change or switch off what mj_collision reports.
  // Overriding replaces every margin with one global value; sleeping skips
  // bodies at rest, which in a scratch state means whatever was asleep last;
  // speculative contacts look ahead by the bodies' velocities, which a
  // configuration does not have.
  m.opt.enableflags &= ~(mjENBL_OVERRIDE | mjENBL_SLEEP | mjENBL_SPECULATIVE);
  m.opt.disableflags &= ~(mjDSBL_CONTACT | mjDSBL_CONSTRAINT);

  checker.data_ = MakeData(m);
  if (!checker.data_) return fail("mj_makeData failed");

  // The arm's root: the body under the world that holds joint 1.
  int root = 0;
  for (int j = 0; j < m.njnt; ++j) {
    if (m.jnt_qposadr[j] == checker.arm_.qpos[0]) root = m.jnt_bodyid[j];
  }
  while (m.body_parentid[root] != 0) root = m.body_parentid[root];

  auto under = [&](int body, int top) {
    for (int b = body; b > 0; b = m.body_parentid[b]) {
      if (b == top) return true;
    }
    return false;
  };
  checker.role_.assign(m.nbody, Role::kObstacle);
  for (int b = 1; b < m.nbody; ++b) {
    if (under(b, checker.arm_.tool_body)) {
      checker.role_[b] = Role::kTool;
    } else if (under(b, root)) {
      checker.role_[b] = Role::kLink;
    }
  }

  if (std::optional<PalletBodies> pallet = FindPallet(m)) {
    checker.modules_ = pallet->modules;
    checker.pallet_mocap_ = pallet->pallet_mocap;
  }
  const int slide = mj_name2id(&m, mjOBJ_JOINT, "conveyor_slide");
  if (slide >= 0 && m.jnt_type[slide] == mjJNT_SLIDE) {
    checker.slide_qpos_ = m.jnt_qposadr[slide];
  }
  checker.is_arm_qpos_.assign(m.nq, false);
  for (int j = 0; j < Arm::kJoints; ++j) {
    checker.is_arm_qpos_[checker.arm_.qpos[j]] = true;
  }

  checker.contype_.assign(m.geom_contype, m.geom_contype + m.ngeom);
  checker.conaffinity_.assign(m.geom_conaffinity,
                              m.geom_conaffinity + m.ngeom);
  checker.allowed_.assign(static_cast<size_t>(m.nbody) * m.nbody, 0);

  // Skipping obstacle pairs by contact bits gives MuJoCo's own answer only
  // if, as loaded, every geom that can move (the arm's, the tool's, any
  // module's) collides with every other colliding geom: the new bits make
  // each such pair collide, whatever the old ones said.
  if (options.skip_obstacle_pairs) {
    std::vector<bool> module_body(m.nbody, false);
    for (int b : checker.modules_) module_body[b] = true;
    auto collidable = [&](int g) {
      return checker.contype_[g] != 0 || checker.conaffinity_[g] != 0;
    };
    for (int g = 0; g < m.ngeom; ++g) {
      const int b = m.geom_bodyid[g];
      if (!collidable(g) || (checker.role_[b] == Role::kObstacle &&
                             !module_body[b])) {
        continue;
      }
      for (int h = 0; h < m.ngeom; ++h) {
        if (h == g || !collidable(h)) continue;
        if (!Collides(checker.contype_[g], checker.conaffinity_[g],
                      checker.contype_[h], checker.conaffinity_[h])) {
          return fail("geoms " + checker.PairName(g, h) +
                      " never collide by their contact bits; "
                      "skip_obstacle_pairs cannot keep that");
        }
      }
    }
  }

  checker.options_ = options;
  if (!checker.SetMargins(options.margin, options.self_margin)) {
    return fail("margins out of range: margin " +
                std::to_string(options.margin) + ", self_margin " +
                std::to_string(options.self_margin) +
                " (both at least 0, self_margin at most 2 * margin)");
  }

  // Lever arms. A hinge turns every point of the bodies below it about its
  // axis; a point's distance from the axis is at most its distance from
  // the joint's anchor, and that is at most the sum of the fixed offsets on
  // the way down the tree (each body's offset from its parent, each joint's
  // anchor from its body, twice: to the anchor and back out from it) plus
  // the geom's offset in its body and its bounding sphere. Configuration
  // independent, so a bound for every pose.
  checker.lever_ = Eigen::VectorXd::Zero(Arm::kJoints);
  checker.lever_to_tool_ = Eigen::VectorXd::Zero(Arm::kJoints);
  for (int j = 0; j < Arm::kJoints; ++j) {
    int joint = -1;
    for (int k = 0; k < m.njnt; ++k) {
      if (m.jnt_qposadr[k] == checker.arm_.qpos[j]) joint = k;
    }
    const int top = m.jnt_bodyid[joint];
    const double anchor = Vec3(m.jnt_pos + 3 * joint).norm();
    // From joint j's anchor down to body b's origin.
    auto reach_to = [&](int b) {
      double reach = anchor;
      for (; b != top; b = m.body_parentid[b]) {
        reach += Vec3(m.body_pos + 3 * b).norm();
        for (int k = m.body_jntadr[b];
             k >= 0 && k < m.body_jntadr[b] + m.body_jntnum[b]; ++k) {
          reach += 2.0 * Vec3(m.jnt_pos + 3 * k).norm();
        }
      }
      return reach;
    };
    for (int g = 0; g < m.ngeom; ++g) {
      const int b = m.geom_bodyid[g];
      if (checker.role_[b] == Role::kObstacle || !under(b, top)) continue;
      if (checker.contype_[g] == 0 && checker.conaffinity_[g] == 0) continue;
      checker.lever_[j] =
          std::max(checker.lever_[j], reach_to(b) +
                                          Vec3(m.geom_pos + 3 * g).norm() +
                                          m.geom_rbound[g]);
    }
    checker.lever_to_tool_[j] = reach_to(checker.arm_.tool_body);
  }
  return checker;
}

// ------------------------------------------------------------------- scene

void CollisionChecker::SetScene(const mjData& world_state) {
  const mjModel& m = *model_;
  for (int i = 0; i < m.nq; ++i) {
    if (!is_arm_qpos_[i]) data_->qpos[i] = world_state.qpos[i];
  }
  mju_copy(data_->mocap_pos, world_state.mocap_pos, 3 * m.nmocap);
  mju_copy(data_->mocap_quat, world_state.mocap_quat, 4 * m.nmocap);
  if (held_) {
    const int adr = m.jnt_qposadr[m.body_jntadr[modules_[*held_]]];
    held_scene_qpos_.assign(world_state.qpos + adr, world_state.qpos + adr + 7);
  }
}

bool CollisionChecker::SetModulePose(int i, const Eigen::Isometry3d& pose) {
  if (i < 0 || i >= static_cast<int>(modules_.size())) return false;
  const mjModel& m = *model_;
  const int adr = m.jnt_qposadr[m.body_jntadr[modules_[i]]];
  const Eigen::Quaterniond q(pose.rotation());
  const double qpos[7] = {pose.translation().x(), pose.translation().y(),
                          pose.translation().z(), q.w(), q.x(), q.y(), q.z()};
  std::copy(qpos, qpos + 7, data_->qpos + adr);
  if (held_ == i) held_scene_qpos_.assign(qpos, qpos + 7);
  return true;
}

bool CollisionChecker::SetPalletPose(const Eigen::Isometry3d& pose) {
  if (pallet_mocap_ < 0) return false;
  const Eigen::Quaterniond q(pose.rotation());
  mjtNum* pos = data_->mocap_pos + 3 * pallet_mocap_;
  mjtNum* quat = data_->mocap_quat + 4 * pallet_mocap_;
  pos[0] = pose.translation().x();
  pos[1] = pose.translation().y();
  pos[2] = pose.translation().z();
  quat[0] = q.w();
  quat[1] = q.x();
  quat[2] = q.y();
  quat[3] = q.z();
  return true;
}

bool CollisionChecker::SetTubeSlide(double position) {
  if (slide_qpos_ < 0) return false;
  data_->qpos[slide_qpos_] = position;
  return true;
}

// ------------------------------------------------------------- held module

bool CollisionChecker::SetHeldModule(int i,
                                     const Eigen::Isometry3d& module_in_tip) {
  if (i < 0 || i >= static_cast<int>(modules_.size())) return false;
  if (held_ && *held_ != i) ClearHeldModule();
  if (!held_) {
    const mjModel& m = *model_;
    const int adr = m.jnt_qposadr[m.body_jntadr[modules_[i]]];
    held_scene_qpos_.assign(data_->qpos + adr, data_->qpos + adr + 7);
    role_[modules_[i]] = Role::kHeld;
    held_ = i;
    ApplyFilters();
    distance_pairs_valid_ = false;
  }
  held_in_tip_ = module_in_tip;
  return true;
}

void CollisionChecker::ClearHeldModule() {
  if (!held_) return;
  const mjModel& m = *model_;
  const int body = modules_[*held_];
  const int adr = m.jnt_qposadr[m.body_jntadr[body]];
  std::copy(held_scene_qpos_.begin(), held_scene_qpos_.end(),
            data_->qpos + adr);
  role_[body] = Role::kObstacle;
  held_.reset();
  ApplyFilters();
  distance_pairs_valid_ = false;
}

// -------------------------------------------------------- allowed contacts

bool CollisionChecker::AllowContact(int body_a, int body_b) {
  const int n = model_->nbody;
  if (body_a < 0 || body_a >= n || body_b < 0 || body_b >= n) return false;
  if (body_a > body_b) std::swap(body_a, body_b);
  if (!allowed_[body_a * n + body_b]) {
    allowed_[body_a * n + body_b] = 1;
    allowed_[body_b * n + body_a] = 1;
    allowed_pairs_.push_back({body_a, body_b});
    distance_pairs_valid_ = false;
  }
  return true;
}

bool CollisionChecker::AllowContact(const std::string& body_a,
                                    const std::string& body_b) {
  return AllowContact(mj_name2id(model_.get(), mjOBJ_BODY, body_a.c_str()),
                      mj_name2id(model_.get(), mjOBJ_BODY, body_b.c_str()));
}

void CollisionChecker::ClearAllowedContacts() {
  std::fill(allowed_.begin(), allowed_.end(), 0);
  allowed_pairs_.clear();
  distance_pairs_valid_ = false;
}

bool CollisionChecker::SetAllowedContacts(
    const std::vector<std::pair<int, int>>& pairs) {
  const int n = model_->nbody;
  for (const auto& [a, b] : pairs) {
    if (a < 0 || a >= n || b < 0 || b >= n) return false;
  }
  ClearAllowedContacts();
  for (const auto& [a, b] : pairs) AllowContact(a, b);
  return true;
}

// ----------------------------------------------------------------- margins

bool CollisionChecker::SetMargins(double margin, double self_margin) {
  if (!std::isfinite(margin) || !std::isfinite(self_margin) || margin < 0.0 ||
      self_margin < 0.0 || self_margin > 2.0 * margin) {
    return false;
  }
  options_.margin = margin;
  options_.self_margin = self_margin;
  ApplyFilters();
  return true;
}

void CollisionChecker::ApplyFilters() {
  mjModel& m = *model_;
  for (int g = 0; g < m.ngeom; ++g) {
    // Gaps make MuJoCo report contacts beyond the margin that hold no
    // force; none here, so a contact means closer than the margin.
    m.geom_gap[g] = 0.0;
    if (contype_[g] == 0 && conaffinity_[g] == 0) {
      m.geom_margin[g] = 0.0;
      continue;
    }
    const Role role = role_[m.geom_bodyid[g]];
    // Shared out so that a pair's margins add up to the one wanted: two
    // moving geoms self_margin, a moving geom and an obstacle margin.
    m.geom_margin[g] = role == Role::kObstacle
                           ? options_.margin - 0.5 * options_.self_margin
                           : 0.5 * options_.self_margin;
    if (!options_.skip_obstacle_pairs) continue;
    switch (role) {
      case Role::kLink:
        m.geom_contype[g] = kLinkBit;
        m.geom_conaffinity[g] = kLinkBit | kToolBit | kHeldBit | kObstacleBit;
        break;
      case Role::kTool:
        m.geom_contype[g] = kToolBit;
        m.geom_conaffinity[g] = kLinkBit | kObstacleBit;
        break;
      case Role::kHeld:
        m.geom_contype[g] = kHeldBit;
        m.geom_conaffinity[g] = kLinkBit | kObstacleBit;
        break;
      case Role::kObstacle:
        m.geom_contype[g] = kObstacleBit;
        m.geom_conaffinity[g] = kLinkBit | kToolBit | kHeldBit;
        break;
    }
  }
  // Each body's summary of its geoms, which the broad and mid phases test
  // first; as mj_setConst computes it.
  for (int b = 0; b < m.nbody; ++b) {
    int contype = 0;
    int conaffinity = 0;
    mjtNum margin = 0.0;
    for (int g = m.body_geomadr[b]; g < m.body_geomadr[b] + m.body_geomnum[b];
         ++g) {
      contype |= m.geom_contype[g];
      conaffinity |= m.geom_conaffinity[g];
      margin = std::max(margin, m.geom_margin[g] + m.geom_gap[g]);
    }
    m.body_contype[b] = contype;
    m.body_conaffinity[b] = conaffinity;
    m.body_margin[b] = margin;
  }
}

// ----------------------------------------------------------------- queries

bool CollisionChecker::Counts(int g, int h) const {
  const int a = model_->geom_bodyid[g];
  const int b = model_->geom_bodyid[h];
  const Role ra = role_[a];
  const Role rb = role_[b];
  if (ra == Role::kObstacle && rb == Role::kObstacle) return false;
  if ((ra == Role::kTool && rb == Role::kHeld) ||
      (ra == Role::kHeld && rb == Role::kTool)) {
    return false;
  }
  return !allowed_[a * model_->nbody + b];
}

void CollisionChecker::Pose(const Eigen::VectorXd& angles) {
  const mjModel& m = *model_;
  mjData* d = data_.get();
  for (int j = 0; j < Arm::kJoints; ++j) d->qpos[arm_.qpos[j]] = angles[j];
  mj_kinematics(&m, d);
  if (!held_) return;

  // The held module, placed by the tool tip just computed. Its qpos is
  // written too, so that the scratch state stays consistent, but rather
  // than a second mj_kinematics for the whole model only this one body is
  // brought up to date: exactly what mj_kinematics does for a body on a
  // free joint (frame from qpos, then its inertial frame and its geoms from
  // their fixed offsets).
  const int site = arm_.tip_site;
  Eigen::Isometry3d tip = Eigen::Isometry3d::Identity();
  tip.translation() = Vec3(d->site_xpos + 3 * site);
  tip.linear() =
      Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>>(
          d->site_xmat + 9 * site);
  const Eigen::Isometry3d pose = tip * held_in_tip_;
  const Eigen::Quaterniond q(pose.rotation());

  const int body = modules_[*held_];
  const int adr = m.jnt_qposadr[m.body_jntadr[body]];
  mjtNum* qpos = d->qpos + adr;
  qpos[0] = pose.translation().x();
  qpos[1] = pose.translation().y();
  qpos[2] = pose.translation().z();
  qpos[3] = q.w();
  qpos[4] = q.x();
  qpos[5] = q.y();
  qpos[6] = q.z();

  mju_copy3(d->xpos + 3 * body, qpos);
  mju_copy4(d->xquat + 4 * body, qpos + 3);
  mju_normalize4(d->xquat + 4 * body);
  mju_quat2Mat(d->xmat + 9 * body, d->xquat + 4 * body);
  mj_local2Global(d, d->xipos + 3 * body, d->ximat + 9 * body,
                  m.body_ipos + 3 * body, m.body_iquat + 4 * body, body,
                  m.body_sameframe[body]);
  mju_mulQuat(d->xiquat + 4 * body, d->xquat + 4 * body,
              m.body_iquat + 4 * body);
  for (int g = m.body_geomadr[body];
       g < m.body_geomadr[body] + m.body_geomnum[body]; ++g) {
    mj_local2Global(d, d->geom_xpos + 3 * g, d->geom_xmat + 9 * g,
                    m.geom_pos + 3 * g, m.geom_quat + 4 * g, body,
                    m.geom_sameframe[g]);
    mju_mulQuat(d->geom_xquat + 4 * g, d->xquat + 4 * body,
                m.geom_quat + 4 * g);
  }
}

CollisionReport CollisionChecker::Query(const Eigen::VectorXd& angles,
                                        const CheckOptions& options,
                                        bool names) {
  CollisionReport report;
  for (int j = 0; j < Arm::kJoints; ++j) {
    if (angles[j] < arm_.lower[j] - kRangeSlack ||
        angles[j] > arm_.upper[j] + kRangeSlack) {
      report.in_range = false;
      report.free = false;
      return report;
    }
  }
  ++queries_;
  const mjModel& m = *model_;
  mjData* d = data_.get();
  Pose(angles);
  mj_collision(&m, d);

  for (int c = 0; c < d->ncon; ++c) {
    const mjContact& contact = d->contact[c];
    int g = contact.geom[0];
    int h = contact.geom[1];
    if (!Counts(g, h)) continue;
    // MuJoCo reports a pair closer than its margin; the check is repeated
    // so that the rule does not rest on that.
    if (contact.dist >= m.geom_margin[g] + m.geom_margin[h]) continue;
    report.free = false;
    if (!Moving(m.geom_bodyid[g])) std::swap(g, h);
    auto same = std::find_if(
        report.pairs.begin(), report.pairs.end(),
        [&](const CollidingPair& p) { return p.geom1 == g && p.geom2 == h; });
    if (same != report.pairs.end()) {
      same->distance = std::min(same->distance, contact.dist);
      continue;
    }
    report.pairs.push_back({g, h, m.geom_bodyid[g], m.geom_bodyid[h],
                            contact.dist, ""});
    if (!options.all_pairs) break;
  }
  // Deepest first; ties by geom, so that the order does not depend on the
  // order MuJoCo found the contacts in.
  std::sort(report.pairs.begin(), report.pairs.end(),
            [](const CollidingPair& a, const CollidingPair& b) {
              if (a.distance != b.distance) return a.distance < b.distance;
              if (a.geom1 != b.geom1) return a.geom1 < b.geom1;
              return a.geom2 < b.geom2;
            });
  if (names) {
    for (CollidingPair& p : report.pairs) {
      p.between = PairName(p.geom1, p.geom2);
    }
  }

  if (options.clearance) {
    if (!distance_pairs_valid_) BuildDistancePairs();
    double world = options.clearance_range;
    double self = options.clearance_range;
    int world_g = -1, world_h = -1, self_g = -1, self_h = -1;
    for (const auto& [g, h] : distance_pairs_) {
      const bool among = Moving(m.geom_bodyid[h]);
      double& best = among ? self : world;
      // Bounding spheres first: a pair whose spheres are further apart than
      // the best so far cannot beat it. A plane (the floor) has none; its
      // distance to the other geom's centre stands in.
      const Eigen::Vector3d pg = Vec3(d->geom_xpos + 3 * g);
      const Eigen::Vector3d ph = Vec3(d->geom_xpos + 3 * h);
      double lower = 0.0;
      if (m.geom_type[h] == mjGEOM_PLANE) {
        const Eigen::Vector3d normal(d->geom_xmat[9 * h + 2],
                                     d->geom_xmat[9 * h + 5],
                                     d->geom_xmat[9 * h + 8]);
        lower = normal.dot(pg - ph) - m.geom_rbound[g];
      } else {
        lower = (pg - ph).norm() - m.geom_rbound[g] - m.geom_rbound[h];
      }
      if (lower > best) continue;
      const double distance = mj_geomDistance(&m, d, g, h, best, nullptr);
      if (distance < best) {
        best = distance;
        (among ? self_g : world_g) = g;
        (among ? self_h : world_h) = h;
      }
    }
    report.clearance = world;
    report.self_clearance = self;
    if (world_g >= 0) report.clearance_between = PairName(world_g, world_h);
    if (self_g >= 0) report.self_clearance_between = PairName(self_g, self_h);
  }
  return report;
}

bool CollisionChecker::IsFree(const Eigen::VectorXd& angles) {
  if (angles.size() != Arm::kJoints || !angles.allFinite()) return false;
  // Without names: building the strings would be most of the cost of a
  // colliding query.
  return Query(angles, {}, false).free;
}

std::optional<CollisionReport> CollisionChecker::Check(
    const Eigen::VectorXd& angles, const CheckOptions& options) {
  if (angles.size() != Arm::kJoints || !angles.allFinite()) return std::nullopt;
  return Query(angles, options, true);
}

bool CollisionChecker::SegmentIsFree(const Eigen::VectorXd& from,
                                     const Eigen::VectorXd& to, double step,
                                     SegmentReport* report) {
  SegmentReport local;
  SegmentReport& r = report != nullptr ? *report : local;
  r = SegmentReport{};
  if (from.size() != Arm::kJoints || to.size() != Arm::kJoints ||
      !from.allFinite() || !to.allFinite() || !(step > 0.0)) {
    r.free = false;
    r.hit_fraction = 0.0;
    r.free_fraction = -1.0;
    return false;
  }
  const Eigen::VectorXd delta = to - from;
  const double steps = std::ceil(delta.cwiseAbs().maxCoeff() / step);
  const int n = std::max(1, static_cast<int>(std::min(steps, 1e9)));
  for (int k = 0; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    const Eigen::VectorXd q = k == n ? to : Eigen::VectorXd(from + t * delta);
    ++r.samples;
    CollisionReport check = Query(q, {}, true);
    if (!check.free) {
      r.free = false;
      r.hit_fraction = t;
      r.free_fraction = k > 0 ? static_cast<double>(k - 1) / n : -1.0;
      r.hit = q;
      r.hit_report = std::move(check);
      return false;
    }
  }
  return true;
}

// -------------------------------------------------------------- resolution

double CollisionChecker::HeldLever() const {
  if (!held_) return 0.0;
  const mjModel& m = *model_;
  const int body = modules_[*held_];
  double extent = 0.0;
  for (int g = m.body_geomadr[body];
       g < m.body_geomadr[body] + m.body_geomnum[body]; ++g) {
    extent = std::max(extent,
                      Vec3(m.geom_pos + 3 * g).norm() + m.geom_rbound[g]);
  }
  return Vec3(m.site_pos + 3 * arm_.tip_site).norm() +
         held_in_tip_.translation().norm() + extent;
}

Eigen::VectorXd CollisionChecker::LeverArms() const {
  if (!held_) return lever_;
  const double held = HeldLever();
  Eigen::VectorXd lever = lever_;
  for (int j = 0; j < Arm::kJoints; ++j) {
    lever[j] = std::max(lever[j], lever_to_tool_[j] + held);
  }
  return lever;
}

double CollisionChecker::SweepBound(const Eigen::VectorXd& from,
                                    const Eigen::VectorXd& to) const {
  return (to - from).cwiseAbs().dot(LeverArms());
}

double CollisionChecker::SuggestStep(double resolution) const {
  return resolution / LeverArms().sum();
}

double CollisionChecker::SegmentStep(const Eigen::VectorXd& from,
                                     const Eigen::VectorXd& to,
                                     double resolution) const {
  const double sweep = SweepBound(from, to);
  if (!(sweep > 0.0)) return std::numeric_limits<double>::infinity();
  return (to - from).cwiseAbs().maxCoeff() * resolution / sweep;
}

// ---------------------------------------------------------------- helpers

void CollisionChecker::BuildDistancePairs() {
  const mjModel& m = *model_;
  distance_pairs_.clear();
  auto collidable = [&](int g) {
    return contype_[g] != 0 || conaffinity_[g] != 0;
  };
  // MuJoCo's own filters on a pair of bodies (mj_collision's broad phase):
  // the same rigid body, two bodies without degrees of freedom, a body and
  // its parent, and the pairs excluded in the model.
  const bool filter_parent = !(m.opt.disableflags & mjDSBL_FILTERPARENT);
  auto tested = [&](int a, int b) {
    const int wa = m.body_weldid[a];
    const int wb = m.body_weldid[b];
    if (wa == wb) return false;
    if (m.body_dofnum[wa] == 0 && m.body_dofnum[wb] == 0) return false;
    const int pa = m.body_weldid[m.body_parentid[wa]];
    const int pb = m.body_weldid[m.body_parentid[wb]];
    if (filter_parent && wa != 0 && wb != 0 && (wa == pb || wb == pa)) {
      return false;
    }
    const unsigned int signature =
        (static_cast<unsigned int>(std::min(a, b)) << 16) +
        static_cast<unsigned int>(std::max(a, b));
    for (int e = 0; e < m.nexclude; ++e) {
      if (static_cast<unsigned int>(m.exclude_signature[e]) == signature) {
        return false;
      }
    }
    return true;
  };
  for (int g = 0; g < m.ngeom; ++g) {
    if (!collidable(g) || !Moving(m.geom_bodyid[g])) continue;
    for (int h = 0; h < m.ngeom; ++h) {
      if (h == g || !collidable(h)) continue;
      // Each pair of moving geoms once.
      if (Moving(m.geom_bodyid[h]) && h < g) continue;
      if (!Collides(contype_[g], conaffinity_[g], contype_[h],
                    conaffinity_[h])) {
        continue;
      }
      if (!tested(m.geom_bodyid[g], m.geom_bodyid[h]) || !Counts(g, h)) {
        continue;
      }
      distance_pairs_.push_back({g, h});
    }
  }
  distance_pairs_valid_ = true;
}

std::string CollisionChecker::PairName(int g, int h) const {
  auto body_name = [&](int body) -> std::string {
    const char* name = mj_id2name(model_.get(), mjOBJ_BODY, body);
    return name != nullptr ? name : "body " + std::to_string(body);
  };
  auto geom_name = [&](int geom) -> std::string {
    const char* name = mj_id2name(model_.get(), mjOBJ_GEOM, geom);
    return name != nullptr ? name
                           : body_name(model_->geom_bodyid[geom]) + " geom";
  };
  return geom_name(g) + " (" + body_name(model_->geom_bodyid[g]) + ") - " +
         geom_name(h) + " (" + body_name(model_->geom_bodyid[h]) + ")";
}

}  // namespace cell
