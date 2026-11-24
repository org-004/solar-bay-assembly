// The checks on a layout. See src/layout_check.h.

#include "src/layout_check.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm_catalog.h"
#include "src/bay_hardware.h"
#include "src/container.h"
#include "src/mj_handles.h"
#include "src/portable_random.h"

namespace cell::layout {
namespace {

using Vec = Eigen::Vector3d;
using Q6 = std::array<double, 6>;  // radians

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kFar = 1.0;              // m, the most a distance looks.
constexpr double kPinMagazineMass = 0.5;  // kg, ours.
constexpr double kShort = 0.005;          // A fastening tool stops this short.
constexpr double kPosTol = 1e-4;          // m
constexpr double kAngTol = 1e-3;          // rad
constexpr double kOrientWeight = 0.5;     // m per rad in the IK's error.

std::string Fmt(const char* format, ...) {
  char buffer[4096];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

Vec V(const Vec3& a) { return {a[0], a[1], a[2]}; }

bool Wanted(const std::vector<std::string>& list, const std::string& what) {
  return list.empty() ||
         std::find(list.begin(), list.end(), what) != list.end();
}

bool Mentions(std::string text, const std::string& word) {
  for (char& c : text) c = static_cast<char>(std::tolower(c));
  return text.find(word) != std::string::npos;
}

std::string GeomName(const mjModel& m, int g) {
  if (g < 0) return "nothing";
  const char* name = mj_id2name(&m, mjOBJ_GEOM, g);
  if (name != nullptr && name[0] != '\0') return name;
  const char* body = mj_id2name(&m, mjOBJ_BODY, m.geom_bodyid[g]);
  return std::string(body != nullptr ? body : "body") + "/geom" +
         std::to_string(g);
}

// ------------------------------------------------------------ bounding boxes
struct Aabb {
  Vec c = Vec::Zero();
  Vec h = Vec::Zero();
  bool plane = false;
};

Aabb WorldAabb(const mjModel& m, const mjData& d, int g) {
  Aabb box;
  if (m.geom_type[g] == mjGEOM_PLANE) {
    box.plane = true;
    return box;
  }
  const mjtNum* a = m.geom_aabb + 6 * g;
  Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> r(
      d.geom_xmat + 9 * g);
  const Vec p(d.geom_xpos[3 * g], d.geom_xpos[3 * g + 1],
              d.geom_xpos[3 * g + 2]);
  box.c = p + r * Vec(a[0], a[1], a[2]);
  box.h = r.cwiseAbs() * Vec(a[3], a[4], a[5]);
  return box;
}

double AabbGap(const Aabb& a, const Aabb& b) {
  if (a.plane || b.plane) return 0.0;
  const Vec gap = ((a.c - b.c).cwiseAbs() - a.h - b.h).cwiseMax(0.0);
  return gap.norm();
}

// The exact extent of a geom in the world, for transport.
void ExactExtent(const mjModel& m, const mjData& d, int g, Vec* lo, Vec* hi) {
  Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> r(
      d.geom_xmat + 9 * g);
  const Vec p(d.geom_xpos[3 * g], d.geom_xpos[3 * g + 1],
              d.geom_xpos[3 * g + 2]);
  *lo = Vec::Constant(std::numeric_limits<double>::infinity());
  *hi = -*lo;
  auto add = [&](const Vec& local) {
    const Vec w = p + r * local;
    *lo = lo->cwiseMin(w);
    *hi = hi->cwiseMax(w);
  };
  if (m.geom_type[g] == mjGEOM_MESH && m.geom_dataid[g] >= 0) {
    const int mesh = m.geom_dataid[g];
    const int adr = m.mesh_vertadr[mesh];
    for (int v = 0; v < m.mesh_vertnum[mesh]; ++v) {
      const float* x = m.mesh_vert + 3 * (adr + v);
      add(Vec(x[0], x[1], x[2]));
    }
    return;
  }
  Vec c = Vec::Zero(), h;
  if (m.geom_type[g] == mjGEOM_BOX) {
    h = Vec(m.geom_size[3 * g], m.geom_size[3 * g + 1], m.geom_size[3 * g + 2]);
  } else {
    const mjtNum* a = m.geom_aabb + 6 * g;
    c = Vec(a[0], a[1], a[2]);
    h = Vec(a[3], a[4], a[5]);
  }
  for (int i = 0; i < 8; ++i) {
    add(c + Vec(i & 1 ? h.x() : -h.x(), i & 2 ? h.y() : -h.y(),
                i & 4 ? h.z() : -h.z()));
  }
}

// ------------------------------------------------------------ distances
// Groups of what moves; an obstacle can be skipped by some of them.
enum Group : unsigned {
  kLinks = 1,
  kCarriage = 2,
  kToolBody = 4,
  kToolLast = 8,
  kHeld = 16,
  kProbe = 32,
};

struct Obstacle {
  int geom = -1;
  unsigned skip = 0;
};

struct Nearest {
  double distance = kFar;
  int moving = -1;
  int obstacle = -1;
};

std::string PairText(const mjModel& m, const Nearest& n) {
  if (n.moving < 0) return "nothing within 1 m";
  return GeomName(m, n.moving) + " to " + GeomName(m, n.obstacle);
}

// The least distance between the moving geoms (with their groups) and the
// obstacles, as of the data's geom poses.
Nearest LeastDistance(const mjModel& m, mjData* d,
                      const std::vector<std::pair<int, unsigned>>& moving,
                      const std::vector<Obstacle>& obstacles,
                      double far = kFar) {
  std::vector<Aabb> mb(moving.size()), ob(obstacles.size());
  for (size_t i = 0; i < moving.size(); ++i) {
    mb[i] = WorldAabb(m, *d, moving[i].first);
  }
  for (size_t j = 0; j < obstacles.size(); ++j) {
    ob[j] = WorldAabb(m, *d, obstacles[j].geom);
  }
  struct Pair {
    double gap;
    int i, j;
  };
  std::vector<Pair> pairs;
  for (size_t i = 0; i < moving.size(); ++i) {
    for (size_t j = 0; j < obstacles.size(); ++j) {
      if (obstacles[j].skip & moving[i].second) continue;
      if (moving[i].first == obstacles[j].geom) continue;
      const double gap = AabbGap(mb[i], ob[j]);
      if (gap < far) pairs.push_back({gap, static_cast<int>(i),
                                      static_cast<int>(j)});
    }
  }
  std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
    if (a.gap != b.gap) return a.gap < b.gap;
    if (a.i != b.i) return a.i < b.i;
    return a.j < b.j;
  });
  Nearest best;
  best.distance = far;
  for (const Pair& p : pairs) {
    if (p.gap >= best.distance) break;
    const int g = moving[p.i].first;
    const int h = obstacles[p.j].geom;
    const double dist = mj_geomDistance(&m, d, g, h, best.distance, nullptr);
    if (dist < best.distance) {
      best.distance = dist;
      best.moving = g;
      best.obstacle = h;
    }
  }
  return best;
}

// ------------------------------------------------------------ the arms
struct ArmCtx {
  const ArmUse* use = nullptr;
  const ArmModel* cat = nullptr;
  const ArmHandle* h = nullptr;
  Q6 lo{}, hi{};    // Where the solver may put each joint.
  Q6 dlo{}, dhi{};  // The datasheet's range.
  Q6 wait{};
  double wait_track = 0.0;
  std::vector<Q6> seeds;
  std::optional<Q6> last;
};

Q6 ToRad(const std::array<double, 6>& deg) {
  Q6 q;
  for (int i = 0; i < 6; ++i) q[i] = deg[i] * kDeg;
  return q;
}
std::array<double, 6> ToDeg(const Q6& q) {
  std::array<double, 6> deg;
  for (int i = 0; i < 6; ++i) deg[i] = q[i] / kDeg;
  return deg;
}

struct Target {
  Vec p = Vec::Zero();
  Vec z = Vec::UnitZ();
  bool has_x = false;
  Vec x = Vec::UnitX();
};

struct IkResult {
  bool ok = false;
  Q6 q{};
  double perr = 1e9;
  double aerr = 1e9;
};

// A pose of an arm.
struct Pose {
  Q6 q{};
  double track = 0.0;
};

// What the reach and clearance checks found at one point.
struct PointResult {
  bool solved = false;   // A pose was looked for.
  bool reached = false;
  bool clear = false;
  Pose pose;
  double margin_deg = 0.0;
  std::string margin_joint;
  double perr = 0.0;
  std::string track_note;
  Nearest nearest;
  std::string scene_note;
  std::string why;  // When there is no pose: why not.
};

// ------------------------------------------------------------ the checker
class Checker {
 public:
  Checker(const Layout& layout, const CheckOptions& options)
      : L_(layout), opt_(options) {}

  bool Init(const ModelPaths& paths, std::string* error);

  // Everything of the reach and clearance checks, for the work kinds wanted.
  void SolveAll();

  std::vector<CheckRow> TransportRows(const ModelPaths& paths);
  std::vector<CheckRow> HoldRows();
  std::vector<CheckRow> LoadRows();
  std::vector<CheckRow> ReachRows();
  std::vector<CheckRow> ClearanceRows();
  std::vector<CheckRow> WaysRows();
  std::vector<CheckRow> TimeRows(std::vector<BayStep>* steps, double* bay);

  // The state for a picture.
  bool State(const std::string& kind, int index, LayoutState* state,
             std::string* error);

 private:
  struct Built {
    std::vector<int> rails;
    std::vector<int> modules;
    int pallet = 36;  // Modules left on the pallet: the last `pallet` taken.
  };
  struct Job {  // One point of one kind, as solved.
    WorkPoint point;
    int arm = -1;
    std::string tool;
    std::string held;  // "module", "rail", "pin magazine" or "".
    PointResult result;
    std::vector<Built> scenes;  // One or two; the worse counts.
    std::set<int> held_skip;
    std::set<int> last_skip;
    std::string error;  // No arm, no tool, no work.
  };

  int ArmIndex(const std::string& name) const {
    for (size_t i = 0; i < arms_.size(); ++i) {
      if (arms_[i].use->name == name) return static_cast<int>(i);
    }
    return -1;
  }
  void AllAtWait();
  // True if the geom is fixed to the arm's carriage: no joint between its
  // body and the track's slide.
  bool RidesCarriage(const ArmCtx& a, int g) const {
    const mjModel& m = *model_;
    if (a.h->track_qpos < 0) return false;
    for (int b = m.geom_bodyid[g]; b > 0; b = m.body_parentid[b]) {
      for (int k = 0; k < m.body_jntnum[b]; ++k) {
        const int j = m.body_jntadr[b] + k;
        return m.jnt_qposadr[j] == a.h->track_qpos;
      }
    }
    return false;
  }
  void SetPose(const ArmCtx& a, const Pose& p);
  IkResult Solve(const ArmCtx& a, int site, const Target& t, double track,
                 const Q6& seed);
  std::vector<std::pair<double, std::string>> TrackCandidates(
      const ArmCtx& a, const WorkPoint& p, int count) const;
  std::vector<Obstacle> Scene(int active, const std::string& tool,
                              const Built& b, const std::set<int>& held_skip,
                              const std::set<int>& last_skip) const;
  std::vector<std::pair<int, unsigned>> Moving(int active,
                                               const std::string& tool,
                                               const std::string& held) const;
  Nearest PoseClearance(Job& job);
  void SolveJob(Job& job, int count);
  void SetUpJob(Job& job);
  Nearest FreeSpace(const Job& job, std::string* which);
  std::string Key(const Job& job) const;
  std::vector<std::pair<std::string, int>> StepOrder() const;
  double MoveTime(const ArmCtx& a, const Pose& from, const Pose& to) const;
  void Margin(const ArmCtx& a, const Q6& q, double* margin,
              std::string* joint) const;

  const Layout& L_;
  CheckOptions opt_;
  ModelPtr model_;
  DataPtr data_;
  LayoutHandles H_;
  std::vector<ArmCtx> arms_;
  // Solved points by kind, in index order.
  std::map<std::string, std::vector<Job>> jobs_;
  std::map<std::string, std::vector<std::string>> kind_errors_;
  bool solved_ = false;
};

bool Checker::Init(const ModelPaths& paths, std::string* error) {
  LayoutState everything;
  everything.everything = true;
  model_ = LoadLayoutModel(L_, everything, paths, error);
  if (!model_) return false;
  data_ = MakeData(*model_);
  if (model_->nkey > 0) mj_resetDataKeyframe(model_.get(), data_.get(), 0);
  H_ = FindHandles(*model_, L_);
  if (H_.arms.size() != L_.arms.size()) {
    *error = Fmt("the model has %d arms, the layout %d",
                 static_cast<int>(H_.arms.size()),
                 static_cast<int>(L_.arms.size()));
    return false;
  }
  std::mt19937 rng(20261124u);
  for (size_t i = 0; i < L_.arms.size(); ++i) {
    ArmCtx a;
    a.use = &L_.arms[i];
    a.cat = FindArmModel(a.use->model);
    a.h = &H_.arms[i];
    if (a.cat == nullptr) {
      *error = "arm " + a.use->name + ": model " + a.use->model +
               " is not in the catalogue";
      return false;
    }
    for (int j = 0; j < 6; ++j) {
      a.dlo[j] = a.cat->joint_lo_deg[j] * kDeg;
      a.dhi[j] = a.cat->joint_hi_deg[j] * kDeg;
      a.lo[j] = a.dlo[j] + opt_.joint_margin_deg * kDeg;
      a.hi[j] = a.dhi[j] - opt_.joint_margin_deg * kDeg;
      const int jnt = [&] {
        for (int k = 0; k < model_->njnt; ++k) {
          if (model_->jnt_qposadr[k] == a.h->qpos[j]) return k;
        }
        return -1;
      }();
      if (jnt >= 0 && model_->jnt_limited[jnt]) {
        a.lo[j] = std::max(a.lo[j], model_->jnt_range[2 * jnt]);
        a.hi[j] = std::min(a.hi[j], model_->jnt_range[2 * jnt + 1]);
      }
    }
    a.wait = ToRad(a.use->wait_joints_deg);
    a.wait_track = a.use->wait_track;
    // The wait pose first, then the fixed draws: one draw per statement.
    a.seeds.push_back(a.wait);
    for (int s = 0; s < opt_.ik_seeds; ++s) {
      Q6 q;
      for (int j = 0; j < 6; ++j) {
        const double u = UniformUnit(rng);
        q[j] = a.lo[j] + u * (a.hi[j] - a.lo[j]);
      }
      a.seeds.push_back(q);
    }
    arms_.push_back(a);
  }
  return true;
}

void Checker::SetPose(const ArmCtx& a, const Pose& p) {
  for (int j = 0; j < 6; ++j) data_->qpos[a.h->qpos[j]] = p.q[j];
  if (a.h->track_qpos >= 0) data_->qpos[a.h->track_qpos] = p.track;
}

void Checker::AllAtWait() {
  for (const ArmCtx& a : arms_) SetPose(a, {a.wait, a.wait_track});
}

// Damped least squares over six joints, the carriage at `track`.
IkResult Checker::Solve(const ArmCtx& a, int site, const Target& t,
                        double track, const Q6& seed) {
  const mjModel& m = *model_;
  mjData* d = data_.get();
  IkResult out;
  Q6 q = seed;
  for (int j = 0; j < 6; ++j) q[j] = std::clamp(q[j], a.lo[j], a.hi[j]);
  if (a.h->track_qpos >= 0) d->qpos[a.h->track_qpos] = track;
  std::vector<mjtNum> jacp(3 * m.nv), jacr(3 * m.nv);
  Eigen::Matrix3d rd;
  if (t.has_x) {
    rd.col(0) = t.x;
    rd.col(2) = t.z;
    rd.col(1) = t.z.cross(t.x);
  }
  auto eval = [&](const Q6& qq, Eigen::Matrix<double, 6, 1>* e,
                  Eigen::Matrix<double, 6, 6>* jac, double* perr,
                  double* aerr) {
    for (int j = 0; j < 6; ++j) d->qpos[a.h->qpos[j]] = qq[j];
    mj_kinematics(&m, d);
    mj_comPos(&m, d);
    const Vec p(d->site_xpos[3 * site], d->site_xpos[3 * site + 1],
                d->site_xpos[3 * site + 2]);
    Eigen::Map<const Eigen::Matrix<mjtNum, 3, 3, Eigen::RowMajor>> rc(
        d->site_xmat + 9 * site);
    const Vec ep = t.p - p;
    Vec er;
    Eigen::Matrix3d proj = Eigen::Matrix3d::Identity();
    if (t.has_x) {
      const Eigen::AngleAxisd aa(Eigen::Matrix3d(rd * rc.transpose()));
      er = aa.axis() * aa.angle();
      *aerr = std::abs(aa.angle());
    } else {
      const Vec zc = rc.col(2);
      const Vec cr = zc.cross(t.z);
      const double ang = std::atan2(cr.norm(), zc.dot(t.z));
      Vec axis = cr;
      if (axis.norm() < 1e-12) {
        axis = std::abs(zc.x()) < 0.9 ? zc.cross(Vec::UnitX())
                                      : zc.cross(Vec::UnitY());
      }
      er = axis.normalized() * ang;
      *aerr = std::abs(ang);
      proj -= zc * zc.transpose();
    }
    *perr = ep.norm();
    e->head<3>() = ep;
    e->tail<3>() = kOrientWeight * er;
    mj_jacSite(&m, d, jacp.data(), jacr.data(), site);
    for (int j = 0; j < 6; ++j) {
      const int c = a.h->dof[j];
      Vec jp(jacp[c], jacp[m.nv + c], jacp[2 * m.nv + c]);
      Vec jr(jacr[c], jacr[m.nv + c], jacr[2 * m.nv + c]);
      jac->block<3, 1>(0, j) = jp;
      jac->block<3, 1>(3, j) = kOrientWeight * (proj * jr);
    }
  };
  Eigen::Matrix<double, 6, 1> e, en;
  Eigen::Matrix<double, 6, 6> jac, jacn;
  double perr, aerr, perrn, aerrn;
  eval(q, &e, &jac, &perr, &aerr);
  double cost = e.squaredNorm();
  double lambda = 0.01;
  double cost_mark = cost;
  int mark = 0;
  for (int it = 0; it < 300; ++it) {
    if (perr < kPosTol && aerr < kAngTol) break;
    const Eigen::Matrix<double, 6, 6> A =
        jac * jac.transpose() +
        lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 6, 1> dq = jac.transpose() * A.ldlt().solve(e);
    const double big = dq.cwiseAbs().maxCoeff();
    if (big > 0.25) dq *= 0.25 / big;
    Q6 qn;
    for (int j = 0; j < 6; ++j) {
      qn[j] = std::clamp(q[j] + dq[j], a.lo[j], a.hi[j]);
    }
    eval(qn, &en, &jacn, &perrn, &aerrn);
    const double costn = en.squaredNorm();
    if (costn < cost) {
      q = qn;
      e = en;
      jac = jacn;
      perr = perrn;
      aerr = aerrn;
      cost = costn;
      lambda = std::max(1e-3, lambda * 0.5);
    } else {
      lambda *= 5.0;
      if (lambda > 1e3) break;
    }
    if (it - mark >= 30) {
      if (cost > 0.5 * cost_mark && perr > 10 * kPosTol) break;  // stalled
      cost_mark = cost;
      mark = it;
    }
  }
  // Leave the data at the result.
  eval(q, &e, &jac, &perr, &aerr);
  out.q = q;
  out.perr = perr;
  out.aerr = aerr;
  out.ok = perr < kPosTol && aerr < kAngTol;
  return out;
}

std::vector<std::pair<double, std::string>> Checker::TrackCandidates(
    const ArmCtx& a, const WorkPoint& p, int count) const {
  std::vector<std::pair<double, std::string>> out;
  if (!a.use->track) {
    out.push_back({0.0, "no track"});
    return out;
  }
  const Track& tr = *a.use->track;
  auto stop = a.use->stops.find(p.kind);
  if (stop != a.use->stops.end() && !stop->second.empty()) {
    const std::vector<double>& v = stop->second;
    if (v.size() == 1) {
      out.push_back({v[0], Fmt("track %.3f (the stop)", v[0])});
    } else if (static_cast<int>(v.size()) == count && p.index < count) {
      out.push_back({v[p.index], Fmt("track %.3f (stop %d)", v[p.index],
                                     p.index)});
    }
    return out;  // Empty when the stops do not fit: the caller says so.
  }
  const Vec dir = V(tr.direction);
  const double s0 = (p.pos - V(a.use->base_pos)).dot(dir) / dir.squaredNorm();
  for (double off : {0.0, 0.15, -0.15, 0.30, -0.30, 0.45, -0.45, 0.60, -0.60}) {
    const double s = std::clamp(s0 + off, tr.lo, tr.hi);
    bool seen = false;
    for (const auto& c : out) seen |= std::abs(c.first - s) < 1e-9;
    if (seen) continue;
    out.push_back({s, Fmt("track %.3f (opposite %+.2f%s)", s, off,
                          std::abs(s - s0 - off) > 1e-9 ? ", clamped" : "")});
  }
  return out;
}

std::vector<Obstacle> Checker::Scene(int active, const std::string& tool,
                                     const Built& b,
                                     const std::set<int>& held_skip,
                                     const std::set<int>& last_skip) const {
  std::vector<Obstacle> out;
  auto add = [&](const std::vector<int>& geoms, unsigned skip = 0) {
    for (int g : geoms) {
      unsigned s = skip;
      if (held_skip.count(g)) s |= kHeld;
      if (last_skip.count(g)) s |= kToolLast;
      out.push_back({g, s});
    }
  };
  add(H_.container_geoms);
  add(H_.holder_geoms);
  add(H_.tube_geoms);
  for (int k : b.rails) {
    if (k < static_cast<int>(H_.rail_geoms.size())) add(H_.rail_geoms[k]);
  }
  for (int k : b.modules) {
    if (k < static_cast<int>(H_.module_geoms.size())) add(H_.module_geoms[k]);
  }
  add(H_.pallet_base_geoms);
  // pallet_module_geoms[j] is the j-th module taken.
  const int pallet_n = static_cast<int>(H_.pallet_module_geoms.size());
  for (int k = std::max(0, pallet_n - b.pallet); k < pallet_n; ++k) {
    add(H_.pallet_module_geoms[k]);
  }
  for (const auto& s : H_.stock_geoms) add(s.second);
  for (const auto& s : H_.item_geoms) add(s.second);
  for (size_t i = 0; i < arms_.size(); ++i) {
    const ArmCtx& a = arms_[i];
    const bool me = static_cast<int>(i) == active;
    add(a.h->beam_geoms, me ? kCarriage : 0u);
    const std::string on = me ? tool : a.use->first_tool;
    if (!me) {
      add(a.h->link_geoms);
      add(a.h->carriage_geoms);
      auto t = a.h->tool_geoms.find(on);
      if (t != a.h->tool_geoms.end()) add(t->second);
    }
    // The arm's own waiting tools ride on its carriage.
    for (const auto& w : a.h->waiting_tool_geoms) {
      if (w.first != on) add(w.second, me ? kCarriage : 0u);
    }
  }
  return out;
}

std::vector<std::pair<int, unsigned>> Checker::Moving(
    int active, const std::string& tool, const std::string& held) const {
  std::vector<std::pair<int, unsigned>> out;
  const ArmCtx& a = arms_[active];
  // Link geoms fixed to the carriage (the base) count as the carriage.
  for (int g : a.h->link_geoms) {
    out.push_back({g, RidesCarriage(a, g) ? kCarriage : kLinks});
  }
  for (int g : a.h->carriage_geoms) out.push_back({g, kCarriage});
  auto t = a.h->tool_geoms.find(tool);
  if (t != a.h->tool_geoms.end()) {
    for (size_t i = 0; i < t->second.size(); ++i) {
      out.push_back({t->second[i],
                     i + 1 == t->second.size() ? kToolLast : kToolBody});
    }
  }
  if (!held.empty()) {
    auto h = a.h->held_geoms.find(held);
    if (h != a.h->held_geoms.end()) {
      for (int g : h->second) out.push_back({g, kHeld});
    }
  }
  return out;
}

// The kinds' parts.
std::string HeldFor(const std::string& kind) {
  if (kind == "rail_take" || kind == "rail_set") return "rail";
  if (kind == "module_take" || kind == "module_set") return "module";
  if (kind == "pins_take") return "pin magazine";
  return "";
}

void Checker::SetUpJob(Job& job) {
  const WorkPoint& p = job.point;
  const std::string& kind = p.kind;
  const Work* w = FindWork(L_, kind);
  if (w == nullptr) {
    job.error = "the layout gives no work " + kind;
    return;
  }
  job.arm = ArmIndex(w->arm);
  job.tool = w->tool;
  if (job.arm < 0) {
    job.error = "no arm " + w->arm;
    return;
  }
  const ArmCtx& a = arms_[job.arm];
  if (FindTool(*a.use, w->tool) == nullptr ||
      a.h->work_site.find(w->tool) == a.h->work_site.end()) {
    job.error = "arm " + w->arm + " has no tool " + w->tool;
    return;
  }
  job.held = HeldFor(kind);
  const int R = bay::kRailCount, M = bay::kModuleCount;
  const int pallet_total = L_.pallet.modules;
  auto range = [](int n) {
    std::vector<int> v;
    for (int i = 0; i < n; ++i) v.push_back(i);
    return v;
  };
  auto ins = [](std::set<int>* s, const std::vector<int>& g) {
    s->insert(g.begin(), g.end());
  };
  auto rail = [&](int k) -> const std::vector<int>& {
    static const std::vector<int> none;
    return k >= 0 && k < static_cast<int>(H_.rail_geoms.size())
               ? H_.rail_geoms[k]
               : none;
  };
  auto module = [&](int k) -> const std::vector<int>& {
    static const std::vector<int> none;
    return k >= 0 && k < static_cast<int>(H_.module_geoms.size())
               ? H_.module_geoms[k]
               : none;
  };
  Built b;
  b.pallet = pallet_total;
  if (kind == "rail_take" || kind == "rail_set") {
    b.rails = range(p.index);
    if (kind == "rail_set") ins(&job.held_skip, H_.tube_geoms);
    if (kind == "rail_take") {
      auto s = H_.stock_geoms.find("rails");
      if (s != H_.stock_geoms.end()) ins(&job.held_skip, s->second);
    }
    job.scenes.push_back(b);
  } else if (kind == "strap_bolt") {
    b.rails = L_.rail_order == "each_rail_then_its_bolt" ? range(p.index + 1)
                                                         : range(R);
    ins(&job.last_skip, rail(p.index));
    // The bolt joins the strap's ears 18 mm under the tube, so the socket
    // on it is nearer the tube than the clearance by the bay's design.
    ins(&job.last_skip, H_.tube_geoms);
    job.scenes.push_back(b);
  } else if (kind == "module_take") {
    b.rails = range(R);
    b.modules = range(std::min(p.index, M));
    const int n = p.pallet_modules;
    b.pallet = n - 1;  // Module j itself is the one held.
    // It touches module j+1 behind (or below) it, and the deck it stands on
    // (flat: only the last one).
    const int j = p.index;
    if (j + 1 < static_cast<int>(H_.pallet_module_geoms.size())) {
      ins(&job.held_skip, H_.pallet_module_geoms[j + 1]);
    }
    if (L_.pallet.stands != "flat" || n == 1) {
      ins(&job.held_skip, H_.pallet_base_geoms);
    }
    job.scenes.push_back(b);
  } else if (kind == "module_set") {
    b.rails = range(R);
    b.modules = range(p.index);
    b.pallet = pallet_total - p.index - 1;
    ins(&job.held_skip, rail(p.index));
    ins(&job.held_skip, rail(p.index + 1));
    ins(&job.held_skip, module(p.index - 1));
    job.scenes.push_back(b);
  } else if (kind == "locate" || kind == "pin" || kind == "swage") {
    b.rails = range(R);
    b.modules = range(p.module + 1);
    b.pallet = pallet_total - p.module - 1;
    // The nose goes into the holes of the rail and the module it joins.
    ins(&job.last_skip, rail(p.rail));
    ins(&job.last_skip, module(p.module));
    job.scenes.push_back(b);
    if (p.module + 1 < M) {
      b.modules = range(p.module + 2);
      job.scenes.push_back(b);
    }
  } else if (kind == "pins_take") {
    b.rails = range(R);
    auto s = H_.stock_geoms.find("pins");
    if (s != H_.stock_geoms.end()) ins(&job.held_skip, s->second);
    job.scenes.push_back(b);
  }
}

Nearest Checker::PoseClearance(Job& job) {
  // The data holds the active arm at its pose and the others at wait.
  const auto moving = Moving(job.arm, job.tool, job.held);
  Nearest worst;
  worst.distance = kFar + 1.0;
  int which = 0;
  for (size_t s = 0; s < job.scenes.size(); ++s) {
    const auto obstacles =
        Scene(job.arm, job.tool, job.scenes[s], job.held_skip, job.last_skip);
    const Nearest n = LeastDistance(*model_, data_.get(), moving, obstacles);
    if (n.distance < worst.distance) {
      worst = n;
      which = static_cast<int>(s);
    }
  }
  job.result.scene_note =
      job.scenes.size() > 1
          ? (which == 1 ? "worse with the next module on"
                        : "worse before the next module")
          : "";
  return worst;
}

void Checker::Margin(const ArmCtx& a, const Q6& q, double* margin,
                     std::string* joint) const {
  *margin = 1e9;
  for (int j = 0; j < 6; ++j) {
    const double mj = std::min(q[j] - a.dlo[j], a.dhi[j] - q[j]) / kDeg;
    if (mj < *margin) {
      *margin = mj;
      *joint = a.cat->joint_names[j];
    }
  }
}

void Checker::SolveJob(Job& job, int count) {
  PointResult& r = job.result;
  r.solved = true;
  if (!job.error.empty()) {
    r.why = job.error;
    return;
  }
  ArmCtx& a = arms_[job.arm];
  const int site = a.h->work_site.at(job.tool);
  const auto tracks = TrackCandidates(a, job.point, count);
  if (tracks.empty()) {
    r.why = Fmt("the layout's stops for %s do not fit its %d points",
                job.point.kind.c_str(), count);
    return;
  }
  std::vector<Target> targets;
  Target t;
  t.p = job.point.pos;
  t.z = job.point.z.normalized();
  t.has_x = job.point.has_x;
  if (t.has_x) {
    t.x = (job.point.x - job.point.x.dot(t.z) * t.z).normalized();
    targets.push_back(t);
    t.x = -t.x;
    targets.push_back(t);
  } else {
    targets.push_back(t);
  }
  std::vector<Q6> seeds;
  if (a.last) seeds.push_back(*a.last);
  seeds.insert(seeds.end(), a.seeds.begin(), a.seeds.end());
  bool have = false;
  double best_perr = 1e9;
  for (const auto& [track, note] : tracks) {
    for (const Q6& seed : seeds) {
      for (const Target& tg : targets) {
        AllAtWait();
        const IkResult ik = Solve(a, site, tg, track, seed);
        if (!ik.ok) {
          if (!r.reached && ik.perr < best_perr) {
            best_perr = ik.perr;
            r.perr = ik.perr;
            r.track_note = note;
            r.pose = {ik.q, track};
          }
          continue;
        }
        const Nearest n = PoseClearance(job);
        const bool clear = n.distance >= opt_.clearance_m;
        if (!have || n.distance > r.nearest.distance) {
          have = true;
          r.reached = true;
          r.clear = clear;
          r.pose = {ik.q, track};
          r.perr = ik.perr;
          r.track_note = note;
          r.nearest = n;
        }
        if (clear) {
          a.last = ik.q;
          // Leave the data at the chosen pose for what follows.
          AllAtWait();
          SetPose(a, r.pose);
          mj_kinematics(model_.get(), data_.get());
          PoseClearance(job);
          Margin(a, r.pose.q, &r.margin_deg, &r.margin_joint);
          return;
        }
      }
    }
  }
  if (r.reached) {
    a.last = r.pose.q;
    AllAtWait();
    SetPose(a, r.pose);
    mj_kinematics(model_.get(), data_.get());
    r.nearest = PoseClearance(job);
    Margin(a, r.pose.q, &r.margin_deg, &r.margin_joint);
  } else {
    Margin(a, r.pose.q, &r.margin_deg, &r.margin_joint);
    r.why = Fmt("not reached: %.3f m short at best", best_perr);
  }
}

std::string Checker::Key(const Job& job) const {
  const WorkPoint& p = job.point;
  return Fmt("%d|%s|%.6f,%.6f,%.6f|%.6f,%.6f,%.6f|%d|%.6f,%.6f,%.6f|%s",
             job.arm, job.tool.c_str(), p.pos.x(), p.pos.y(), p.pos.z(),
             p.z.x(), p.z.y(), p.z.z(), p.has_x ? 1 : 0, p.x.x(), p.x.y(),
             p.x.z(), job.held.c_str());
}

// The order in which points are solved: the bay's steps, then the takes
// the bay does not use.
std::vector<std::pair<std::string, int>> Checker::StepOrder() const {
  std::vector<std::pair<std::string, int>> out;
  const int R = bay::kRailCount, M = bay::kModuleCount;
  const bool each = L_.rail_order == "each_rail_then_its_bolt";
  for (int k = 0; k < R; ++k) {
    out.push_back({"rail_take", k});
    out.push_back({"rail_set", k});
    if (each) out.push_back({"strap_bolt", k});
  }
  if (!each) {
    for (int k = 0; k < R; ++k) out.push_back({"strap_bolt", k});
  }
  // The module arm's work, then the pin arm's (each arm's chain of seeds
  // is its own, so their interleaving does not matter).
  for (int m = 0; m < M; ++m) {
    out.push_back({"module_take", m});
    out.push_back({"module_set", m});
  }
  out.push_back({"pins_take", 0});
  for (int m = 0; m < M; ++m) {
    for (int i : bay::kLocateOrder) {
      out.push_back({"locate", 4 * m + i});
      out.push_back({"pin", 4 * m + i});
    }
    for (int i : bay::kSwageOrder) out.push_back({"swage", 4 * m + i});
  }
  for (int j = M; j < L_.pallet.modules; ++j) out.push_back({"module_take", j});
  return out;
}

void Checker::SolveAll() {
  if (solved_) return;
  solved_ = true;
  for (const std::string& kind : WorkKinds()) {
    if (!Wanted(opt_.work_kinds, kind)) continue;
    std::vector<Job>& list = jobs_[kind];
    for (const WorkPoint& p : WorkPoints(L_, kind)) {
      Job job;
      job.point = p;
      SetUpJob(job);
      list.push_back(job);
    }
    if (list.empty()) {
      kind_errors_[kind].push_back(
          kind == "rail_take" ? "the layout has no stock \"rails\""
                              : kind == "pins_take"
                                    ? "the layout has no stock \"pins\""
                                    : "no points");
    }
  }
  std::map<std::string, std::pair<std::string, int>> done;  // key -> job
  for (const auto& [kind, index] : StepOrder()) {
    auto it = jobs_.find(kind);
    if (it == jobs_.end() || index >= static_cast<int>(it->second.size())) {
      continue;
    }
    Job& job = it->second[index];
    if (job.result.solved) continue;
    const int count = static_cast<int>(it->second.size());
    const std::string key = job.error.empty() ? Key(job) : "";
    auto same = key.empty() ? done.end() : done.find(key);
    if (same != done.end()) {
      const Job& other = jobs_[same->second.first][same->second.second];
      job.result = other.result;
      // The clearance is the same pose's, but against this kind's scenes.
      if (job.result.reached) {
        AllAtWait();
        SetPose(arms_[job.arm], job.result.pose);
        mj_kinematics(model_.get(), data_.get());
        job.result.nearest = PoseClearance(job);
        job.result.clear = job.result.nearest.distance >= opt_.clearance_m;
      }
      continue;
    }
    SolveJob(job, count);
    if (!key.empty()) done[key] = {kind, index};
  }
}

// ------------------------------------------------------------ rows
CheckRow Row(const std::string& check, const std::string& item,
             const std::string& arm, const std::string& tool, double figure,
             const std::string& unit, double limit, bool pass,
             const std::string& detail) {
  return {check, item, arm, tool, figure, unit, limit, pass, detail};
}

std::vector<CheckRow> Checker::ReachRows() {
  std::vector<CheckRow> rows;
  for (const std::string& kind : WorkKinds()) {
    auto e = kind_errors_.find(kind);
    if (e != kind_errors_.end()) {
      for (const std::string& why : e->second) {
        rows.push_back(Row("reach", kind, "", "", 0.0, "deg",
                           opt_.joint_margin_deg, false, why));
      }
    }
    auto it = jobs_.find(kind);
    if (it == jobs_.end()) continue;
    for (const Job& job : it->second) {
      const PointResult& r = job.result;
      const std::string arm = job.arm >= 0 ? arms_[job.arm].use->name : "";
      std::string detail;
      if (r.reached) {
        detail = Fmt("least margin at %s; %s", r.margin_joint.c_str(),
                     r.track_note.c_str());
      } else {
        detail = r.why + (r.track_note.empty() ? "" : "; " + r.track_note);
      }
      rows.push_back(Row("reach", job.point.item, arm, job.tool,
                         r.reached ? r.margin_deg : 0.0, "deg",
                         opt_.joint_margin_deg,
                         r.reached && r.margin_deg >= opt_.joint_margin_deg - 1e-9,
                         detail));
    }
  }
  return rows;
}

Nearest Checker::FreeSpace(const Job& job, std::string* which) {
  const WorkPoint& p = job.point;
  const mjModel& m = *model_;
  mjData* d = data_.get();
  AllAtWait();
  if (job.arm >= 0 && job.result.reached) {
    SetPose(arms_[job.arm], job.result.pose);
  }
  // Point a and the probe's orientation.
  Eigen::Isometry3d at;
  std::vector<int> probes;
  std::set<int> joined;
  auto ins = [&](const std::vector<int>& g) { joined.insert(g.begin(), g.end()); };
  const std::vector<bay::Fastening> all = bay::Fastenings();
  if (p.kind == "strap_bolt") {
    at = all[p.index].pose_a;
    probes = {H_.probe_bolt_geom};
    if (p.index < static_cast<int>(H_.rail_geoms.size())) {
      ins(H_.rail_geoms[p.index]);
    }
    ins(H_.tube_geoms);
  } else {
    at = all[bay::kRailFasteningCount + p.index].pose_a;
    probes = p.kind == "pin" ? std::vector<int>{H_.probe_pin_geom}
                             : std::vector<int>{H_.probe_locate_below_geom,
                                                H_.probe_locate_above_geom};
    if (p.rail < static_cast<int>(H_.rail_geoms.size())) {
      ins(H_.rail_geoms[p.rail]);
    }
    if (p.module < static_cast<int>(H_.module_geoms.size())) {
      ins(H_.module_geoms[p.module]);
    }
  }
  Nearest worst;
  worst.distance = kFar + 1.0;
  if (H_.probe_mocap < 0) {
    *which = "the model has no probe";
    worst.distance = -1.0;
    return worst;
  }
  const Vec pos = BayPoint(L_, at.translation());
  const Eigen::Matrix3d rot = at.linear();
  mjtNum mat[9], quat[4];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) mat[3 * r + c] = rot(r, c);
  }
  mju_mat2Quat(quat, mat);
  for (int i = 0; i < 3; ++i) d->mocap_pos[3 * H_.probe_mocap + i] = pos[i];
  for (int i = 0; i < 4; ++i) d->mocap_quat[4 * H_.probe_mocap + i] = quat[i];
  mj_kinematics(&m, d);
  std::vector<std::pair<int, unsigned>> moving;
  for (int g : probes) {
    if (g >= 0) moving.push_back({g, kProbe});
  }
  if (moving.empty()) {
    *which = "the model has no such probe cylinder";
    worst.distance = -1.0;
    return worst;
  }
  int scene_index = 0;
  for (size_t s = 0; s < job.scenes.size(); ++s) {
    // Everything there, the active arm at its pose too, except its tool and
    // the joined parts.
    std::vector<Obstacle> obstacles =
        Scene(job.arm >= 0 ? job.arm : 0, job.tool, job.scenes[s], {}, {});
    if (job.arm >= 0) {
      const ArmCtx& a = arms_[job.arm];
      for (int g : a.h->link_geoms) obstacles.push_back({g, 0});
      for (int g : a.h->carriage_geoms) obstacles.push_back({g, 0});
    }
    for (Obstacle& o : obstacles) {
      if (joined.count(o.geom)) o.skip |= kProbe;
    }
    const Nearest n = LeastDistance(m, d, moving, obstacles);
    if (n.distance < worst.distance) {
      worst = n;
      scene_index = static_cast<int>(s);
    }
  }
  *which = job.scenes.size() > 1 && scene_index == 1
               ? "with the next module on"
               : "";
  // Put the probe away.
  for (int i = 0; i < 3; ++i) d->mocap_pos[3 * H_.probe_mocap + i] = 0.0;
  d->mocap_pos[3 * H_.probe_mocap + 2] = -10.0;
  return worst;
}

std::vector<CheckRow> Checker::ClearanceRows() {
  std::vector<CheckRow> rows;
  const mjModel& m = *model_;
  for (const std::string& kind : WorkKinds()) {
    auto it = jobs_.find(kind);
    if (it == jobs_.end()) continue;
    for (Job& job : it->second) {
      const PointResult& r = job.result;
      const std::string arm = job.arm >= 0 ? arms_[job.arm].use->name : "";
      if (!r.reached) {
        rows.push_back(Row("clearance", job.point.item, arm, job.tool, 0.0,
                           "m", opt_.clearance_m, false,
                           "no pose: " + (r.why.empty() ? "not solved" : r.why)));
      } else {
        std::string detail = PairText(m, r.nearest);
        if (!r.scene_note.empty() && job.scenes.size() > 1) {
          detail += "; " + r.scene_note;
        }
        rows.push_back(Row("clearance", job.point.item, arm, job.tool,
                           r.nearest.distance, "m", opt_.clearance_m,
                           r.nearest.distance >= opt_.clearance_m, detail));
      }
    }
    if (kind == "strap_bolt" || kind == "pin" || kind == "locate") {
      for (Job& job : it->second) {
        std::string which;
        const Nearest n = FreeSpace(job, &which);
        std::string detail =
            n.distance < 0.0 ? which : PairText(m, n);
        if (n.distance >= 0.0 && !which.empty()) detail += "; " + which;
        if (!job.result.reached) detail += "; the arm at its wait pose (no pose)";
        const std::string name =
            kind == "strap_bolt"
                ? Fmt("free space bolt %d", job.point.index)
                : Fmt("free space %s %d/%d", kind.c_str(), job.point.module,
                      job.point.hole);
        rows.push_back(Row("clearance", name,
                           job.arm >= 0 ? arms_[job.arm].use->name : "",
                           job.tool, n.distance, "m", 0.0, n.distance > 0.0,
                           detail));
      }
    }
  }
  return rows;
}

std::vector<CheckRow> Checker::TransportRows(const ModelPaths& paths) {
  std::vector<CheckRow> rows;
  LayoutState stowed;
  stowed.stowed = true;
  std::string error;
  ModelPtr model = LoadLayoutModel(L_, stowed, paths, &error);
  const container::Box env = container::InsideEnvelope();
  const Vec elo = env.centre - env.half_size;
  const Vec ehi = env.centre + env.half_size;
  if (!model) {
    for (const char* item : {"stowed x", "stowed y", "stowed top",
                             "stowed bottom"}) {
      rows.push_back(Row("transport", item, "", "", 0.0, "m", 0.0, false,
                         "no stowed model: " + error));
    }
  } else {
    DataPtr data = MakeData(*model);
    if (model->nkey > 0) mj_resetDataKeyframe(model.get(), data.get(), 0);
    mj_kinematics(model.get(), data.get());
    const LayoutHandles h = FindHandles(*model, L_);
    std::set<int> container(h.container_geoms.begin(), h.container_geoms.end());
    double fx = 1e9, fy = 1e9, ftop = 1e9, fbot = 1e9;
    int gx = -1, gy = -1, gtop = -1, gbot = -1;
    for (int g = 0; g < model->ngeom; ++g) {
      if (container.count(g)) continue;
      if (model->geom_type[g] == mjGEOM_PLANE) continue;
      if (model->geom_bodyid[g] == 0 && model->geom_rgba[4 * g + 3] == 0.0) {
        continue;  // An invisible marker of the world.
      }
      Vec lo, hi;
      ExactExtent(*model, *data, g, &lo, &hi);
      const double x = std::min(ehi.x() - hi.x(), lo.x() - elo.x());
      const double y = std::min(ehi.y() - hi.y(), lo.y() - elo.y());
      const double top = ehi.z() - hi.z();
      const double bot = lo.z() - elo.z();
      if (x < fx) { fx = x; gx = g; }
      if (y < fy) { fy = y; gy = g; }
      if (top < ftop) { ftop = top; gtop = g; }
      if (bot < fbot) { fbot = bot; gbot = g; }
    }
    auto row = [&](const char* item, double f, int g, const char* what) {
      rows.push_back(Row("transport", item, "", "", f, "m", 0.0, f >= -1e-6,
                         Fmt("worst %s, %s", GeomName(*model, g).c_str(), what)));
    };
    row("stowed x", fx, gx, Fmt("inside +/-%.3f", ehi.x()).c_str());
    row("stowed y", fy, gy, Fmt("inside +/-%.3f", ehi.y()).c_str());
    row("stowed top", ftop, gtop, Fmt("under %.3f", ehi.z()).c_str());
    row("stowed bottom", fbot, gbot, Fmt("above %.3f", elo.z()).c_str());
  }
  // Mass.
  double arms = 0.0, tracks = 0.0, tools = 0.0, items = 0.0;
  for (const ArmUse& a : L_.arms) {
    const ArmModel* c = FindArmModel(a.model);
    if (c != nullptr) arms += c->mass_kg;
    if (a.track) tracks += a.track->mass_kg;
    for (const Tool& t : a.tools) tools += t.mass_kg;
  }
  const double holder = L_.holder.travels_inside ? L_.holder.mass_kg : 0.0;
  for (const Item& i : L_.items) {
    if (i.travels_inside) items += i.mass_kg;
  }
  const double total = arms + tracks + tools + holder + items;
  rows.push_back(Row("transport", "mass", "", "", total, "kg",
                     container::kCellMassBudget,
                     total <= container::kCellMassBudget,
                     Fmt("arms %.0f, tracks %.0f, tools %.1f, holder %.0f, "
                         "items inside %.0f",
                         arms, tracks, tools, holder, items)));
  return rows;
}

std::vector<CheckRow> Checker::HoldRows() {
  std::vector<CheckRow> rows;
  const std::vector<bay::Hold> holds = bay::Holds();
  int inner = 0;
  for (const Grip& g : L_.holder.grips) {
    const bool in = g.hold >= 0 && g.hold < bay::kHoldCount;
    const std::string site = "bay_hold_" + std::to_string(g.hold);
    const bool modelled =
        model_ && mj_name2id(model_.get(), mjOBJ_SITE, site.c_str()) >= 0;
    const double stretch = in ? 2.0 * holds[g.hold].half_length : 0.0;
    if (in && g.hold > 0 && g.hold < bay::kHoldCount - 1) ++inner;
    rows.push_back(Row(
        "hold", "grip " + site, "", "", g.length, "m", stretch,
        in && modelled && g.length > 0.0 && g.length <= stretch + 1e-9,
        !in ? "no such stretch"
            : !modelled ? "the model has no site " + site
                        : Fmt("stretch %.3f long", stretch)));
  }
  rows.push_back(Row("hold", "grips not only the ends", "", "", inner, "",
                     1.0, inner >= 1,
                     Fmt("%d of %d grips on stretches between rails", inner,
                         static_cast<int>(L_.holder.grips.size()))));
  const int pin = L_.holder.angle_pin_hold;
  rows.push_back(Row("hold", "angle pin", "", "", pin, "", 0.0,
                     pin == 0 || pin == bay::kHoldCount - 1,
                     "an end stretch: bay_hold_0 or bay_hold_" +
                         std::to_string(bay::kHoldCount - 1)));
  rows.push_back(Row("hold", "moment", "", "", L_.holder.resists_nm, "N m",
                     bay::kHoldMoment, L_.holder.resists_nm >= bay::kHoldMoment,
                     "about the tube's axis, at least bay::kHoldMoment"));
  const double limit = bay::kHoldAngle / kDeg;
  rows.push_back(Row("hold", "angle", "", "", L_.holder.holds_within_deg,
                     "deg", limit, L_.holder.holds_within_deg <= limit + 1e-9,
                     "the holes kept this close to straight up, at most "
                     "bay::kHoldAngle"));
  return rows;
}

std::vector<CheckRow> Checker::LoadRows() {
  std::vector<CheckRow> rows;
  for (const ArmUse& a : L_.arms) {
    const ArmModel* c = FindArmModel(a.model);
    for (const Tool& t : a.tools) {
      double part = 0.0;
      std::string what = "nothing";
      for (const Work& w : L_.work) {
        if (w.arm != a.name || w.tool != t.name) continue;
        const std::string held = HeldFor(w.kind);
        double mass = 0.0;
        if (held == "module") mass = bay::kModuleMass;
        if (held == "rail") mass = bay::kRailMass;
        if (held == "pin magazine" || w.kind == "pin" || w.kind == "locate" ||
            w.kind == "swage") {
          mass = kPinMagazineMass;
          if (part < mass) what = "a pin magazine (ours)";
        }
        if (mass > part) {
          part = mass;
          if (held == "module") what = "a module";
          if (held == "rail") what = "a rail with its strap";
        }
      }
      const double total = t.mass_kg + part;
      const double payload = c != nullptr ? c->payload_kg : 0.0;
      rows.push_back(Row("load", a.name + " " + t.name, a.name, t.name, total,
                         "kg", payload, c != nullptr && total <= payload,
                         Fmt("tool %.2f + %s %.2f; rated payload of %s",
                             t.mass_kg, what.c_str(), part,
                             c != nullptr ? c->maker.c_str() : "?")));
    }
  }
  return rows;
}

std::vector<CheckRow> Checker::WaysRows() {
  std::vector<CheckRow> rows;
  const mjModel& m = *model_;
  mjData* d = data_.get();
  AllAtWait();
  if (H_.probe_mocap >= 0) d->mocap_pos[3 * H_.probe_mocap + 2] = -10.0;
  mj_kinematics(&m, d);
  // Obstacles: everything but the bay, every arm at wait.
  std::vector<Obstacle> obstacles;
  auto add = [&](const std::vector<int>& g) {
    for (int x : g) obstacles.push_back({x, 0});
  };
  add(H_.container_geoms);
  const size_t holder_from = obstacles.size();
  add(H_.holder_geoms);
  const size_t holder_to = obstacles.size();
  add(H_.pallet_base_geoms);
  for (const auto& p : H_.pallet_module_geoms) add(p);
  for (const auto& s : H_.stock_geoms) add(s.second);
  for (const Item& item : L_.items) {
    if (Mentions(item.does, "tube") || Mentions(item.does, "bay")) continue;
    auto it = H_.item_geoms.find(item.name);
    if (it != H_.item_geoms.end()) add(it->second);
  }
  for (const ArmCtx& a : arms_) {
    add(a.h->beam_geoms);
    add(a.h->link_geoms);
    add(a.h->carriage_geoms);
    auto t = a.h->tool_geoms.find(a.use->first_tool);
    if (t != a.h->tool_geoms.end()) add(t->second);
    for (const auto& w : a.h->waiting_tool_geoms) {
      if (w.first != a.use->first_tool) add(w.second);
    }
  }
  struct Sweep {
    std::string item;
    const Way* way;
    std::vector<int> geoms;
  };
  std::vector<int> bay = H_.tube_geoms;
  for (const auto& r : H_.rail_geoms) bay.insert(bay.end(), r.begin(), r.end());
  for (const auto& r : H_.module_geoms) {
    bay.insert(bay.end(), r.begin(), r.end());
  }
  const std::vector<Sweep> sweeps = {{"tube in", &L_.tube_in, H_.tube_geoms},
                                     {"bay out", &L_.bay_out, bay}};
  for (const Sweep& s : sweeps) {
    std::vector<std::pair<int, unsigned>> moving;
    for (int g : s.geoms) moving.push_back({g, kHeld});
    std::vector<Vec> orig;
    for (int g : s.geoms) {
      orig.push_back(Vec(d->geom_xpos[3 * g], d->geom_xpos[3 * g + 1],
                         d->geom_xpos[3 * g + 2]));
    }
    // The holder's geoms the moving set rests in.
    std::vector<Obstacle> rest = obstacles;
    for (size_t j = holder_from; j < holder_to; ++j) {
      std::vector<Obstacle> one = {obstacles[j]};
      if (LeastDistance(m, d, moving, one).distance < opt_.clearance_m) {
        rest[j].skip |= kHeld;
      }
    }
    // The samples.
    std::vector<Vec> samples;
    const std::vector<Vec3>& path = s.way->path;
    if (path.empty()) samples.push_back(Vec::Zero());
    for (size_t i = 0; i < path.size(); ++i) {
      if (i == 0) {
        samples.push_back(V(path[0]));
        continue;
      }
      const Vec a = V(path[i - 1]), b = V(path[i]);
      const int n = std::max(
          1, static_cast<int>(std::ceil((b - a).norm() / opt_.sweep_step_m)));
      for (int k = 1; k <= n; ++k) samples.push_back(a + (b - a) * k / n);
    }
    Nearest worst;
    worst.distance = kFar + 1.0;
    Vec worst_at = Vec::Zero();
    for (const Vec& o : samples) {
      for (size_t i = 0; i < s.geoms.size(); ++i) {
        for (int c = 0; c < 3; ++c) {
          d->geom_xpos[3 * s.geoms[i] + c] = orig[i][c] + o[c];
        }
      }
      const bool above = std::hypot(o.x(), o.y()) < 1e-9 && o.z() >= -1e-9;
      const Nearest n =
          LeastDistance(m, d, moving, above ? rest : obstacles);
      if (n.distance < worst.distance) {
        worst = n;
        worst_at = o;
      }
    }
    for (size_t i = 0; i < s.geoms.size(); ++i) {
      for (int c = 0; c < 3; ++c) d->geom_xpos[3 * s.geoms[i] + c] = orig[i][c];
    }
    if (s.geoms.empty()) worst.distance = 0.0;
    rows.push_back(Row(
        "ways", s.item, "", "", std::min(worst.distance, kFar), "m",
        opt_.clearance_m, !s.geoms.empty() && worst.distance >= opt_.clearance_m,
        s.geoms.empty()
            ? "the model has no tube"
            : Fmt("%s at offset (%.3f, %.3f, %.3f); %d samples, at most "
                  "%.3f m apart",
                  PairText(m, worst).c_str(), worst_at.x(), worst_at.y(),
                  worst_at.z(), static_cast<int>(samples.size()),
                  opt_.sweep_step_m)));
  }
  mj_kinematics(&m, d);
  return rows;
}

double Checker::MoveTime(const ArmCtx& a, const Pose& from,
                         const Pose& to) const {
  std::array<double, 6> change;
  bool any = false;
  for (int j = 0; j < 6; ++j) {
    change[j] = (to.q[j] - from.q[j]) / kDeg;
    any |= std::abs(change[j]) > 1e-9;
  }
  double t = JointMoveTime(change, a.cat->joint_speed_deg_s);
  if (a.use->track) {
    const double dist = std::abs(to.track - from.track) *
                        V(a.use->track->direction).norm();
    if (dist > 1e-9) any = true;
    t = std::max(t, TrackMoveTime(dist, a.use->track->speed,
                                  a.use->track->accel));
  }
  return any ? t + kSettleS : 0.0;
}

std::vector<CheckRow> Checker::TimeRows(std::vector<BayStep>* steps,
                                        double* bay_time) {
  std::vector<CheckRow> rows;
  steps->clear();
  struct ArmNow {
    Pose pose;
    std::string tool;
    std::string at = "wait";
    double busy = 0.0;
  };
  std::vector<ArmNow> now(arms_.size());
  for (size_t i = 0; i < arms_.size(); ++i) {
    now[i].pose = {arms_[i].wait, arms_[i].wait_track};
    now[i].tool = arms_[i].use->first_tool;
  }
  bool all_poses = true;
  std::vector<std::string> missing;
  auto push = [&](const std::string& what, const std::string& arm,
                  const std::string& tool, const std::string& part,
                  const std::string& from, const std::string& to,
                  double t) {
    BayStep s;
    s.number = static_cast<int>(steps->size()) + 1;
    s.what = what;
    s.arm = arm;
    s.tool = tool;
    s.part = part;
    s.from = from;
    s.to = to;
    s.time_s = t;
    steps->push_back(s);
    return t;
  };
  // One working step of an arm; returns its time.
  auto work = [&](const std::string& kind, int index, const std::string& what,
                  const std::string& part) -> double {
    const Work* w = FindWork(L_, kind);
    if (w == nullptr) {
      all_poses = false;
      missing.push_back(kind + ": no work");
      return 0.0;
    }
    const int ai = ArmIndex(w->arm);
    if (ai < 0) {
      all_poses = false;
      missing.push_back(kind + ": no arm " + w->arm);
      return 0.0;
    }
    ArmNow& n = now[ai];
    double t = 0.0;
    if (n.tool != w->tool) {
      const double tc = arms_[ai].use->tool_change_s;
      t += push("tool change", w->arm, w->tool, "", n.tool, w->tool, tc);
      n.tool = w->tool;
    }
    auto it = jobs_.find(kind);
    const Job* job = it != jobs_.end() &&
                             index < static_cast<int>(it->second.size())
                         ? &it->second[index]
                         : nullptr;
    double move = 0.0;
    std::string to = job != nullptr ? job->point.item : kind;
    if (job != nullptr && job->result.reached) {
      move = MoveTime(arms_[ai], n.pose, job->result.pose);
      n.pose = job->result.pose;
    } else {
      all_poses = false;
      missing.push_back(to);
      to += " (no pose)";
    }
    t += push(what, w->arm, w->tool, part, n.at, to, move + w->process_s);
    n.at = job != nullptr ? job->point.item : kind;
    n.busy += t;
    return t;
  };
  const int R = bay::kRailCount, M = bay::kModuleCount;
  double total = 0.0;
  const double tube_in = push("tube in", "", "", "tube", "outside", "holder",
                              L_.tube_in.time_s);
  const double clamp = push("clamp", "", "", "tube", "", "holder",
                            L_.holder.clamp_s);
  // The rail phase. Each step's time goes to the setting arm's rail k
  // (set[k]) or the bolting arm's (bolt[k]); a take goes to the arm that
  // takes (a third arm counts with the setting arm).
  const bool each = L_.rail_order == "each_rail_then_its_bolt";
  const Work* wrail = FindWork(L_, "rail_set");
  const Work* wbolt = FindWork(L_, "strap_bolt");
  const Work* wtake = FindWork(L_, "rail_take");
  const bool one_rail_arm =
      wrail == nullptr || wbolt == nullptr || wrail->arm == wbolt->arm;
  const bool take_by_bolter = !one_rail_arm && wtake != nullptr &&
                              wtake->arm == wbolt->arm;
  std::vector<double> set(R, 0.0), bolt(R, 0.0);
  auto bolt_rail = [&](int k) {
    bolt[k] += work("strap_bolt", k, Fmt("bolt rail %d", k),
                    Fmt("strap bolt %d", k));
  };
  for (int k = 0; k < R; ++k) {
    (take_by_bolter ? bolt : set)[k] +=
        work("rail_take", k, Fmt("take rail %d", k), Fmt("rail %d", k));
    set[k] += work("rail_set", k, Fmt("set rail %d", k), Fmt("rail %d", k));
    if (each) bolt_rail(k);
  }
  if (!each) {
    for (int k = 0; k < R; ++k) bolt_rail(k);
  }
  double sum_set = 0.0, sum_bolt = 0.0;
  for (int k = 0; k < R; ++k) {
    sum_set += set[k];
    sum_bolt += bolt[k];
  }
  const double rails =
      one_rail_arm ? sum_set + sum_bolt : SideBySide(set, bolt);
  // The module phase.
  std::vector<double> mod(M, 0.0), pin(M, 0.0);
  const Work* wset = FindWork(L_, "module_set");
  const Work* wpin = FindWork(L_, "pin");
  const bool one_arm =
      wset != nullptr && wpin != nullptr && wset->arm == wpin->arm;
  double pins_take = 0.0;
  for (int m = 0; m < M; ++m) {
    mod[m] += work("module_take", m, Fmt("take module %d", m),
                   Fmt("pallet module %d", m));
    mod[m] += work("module_set", m, Fmt("set module %d", m),
                   Fmt("module %d", m));
    if (m == 0) {
      pins_take = work("pins_take", 0, "take a pin magazine", "pin magazine");
    }
    for (int i : bay::kLocateOrder) {
      pin[m] += work("locate", 4 * m + i, Fmt("locate %d/%d", m, i),
                     Fmt("module %d hole %d", m, i));
      pin[m] += work("pin", 4 * m + i, Fmt("pin %d/%d", m, i),
                     Fmt("pin %d/%d", m, i));
    }
    for (int i : bay::kSwageOrder) {
      pin[m] += work("swage", 4 * m + i, Fmt("swage %d/%d", m, i),
                     Fmt("collar %d/%d", m, i));
    }
  }
  pin[0] += pins_take;
  double sum_mod = 0.0, sum_pin = 0.0;
  for (int m = 0; m < M; ++m) {
    sum_mod += mod[m];
    sum_pin += pin[m];
  }
  const double modules = one_arm ? sum_mod + sum_pin : SideBySide(mod, pin);
  // The end.
  const double release = push("release", "", "", "bay", "holder", "",
                              L_.holder.release_s);
  const double out = push("bay out", "", "", "bay", "holder", "outside",
                          L_.bay_out.time_s);
  double back = 0.0;
  for (size_t i = 0; i < arms_.size(); ++i) {
    ArmNow& n = now[i];
    double t = 0.0;
    const Pose wait{arms_[i].wait, arms_[i].wait_track};
    const double move = MoveTime(arms_[i], n.pose, wait);
    if (move > 0.0) {
      t += push("back to wait", arms_[i].use->name, n.tool, "", n.at, "wait",
                move);
    }
    if (n.tool != arms_[i].use->first_tool) {
      t += push("tool change", arms_[i].use->name, arms_[i].use->first_tool,
                "", n.tool, arms_[i].use->first_tool,
                arms_[i].use->tool_change_s);
    }
    n.busy += t;
    back = std::max(back, t);
  }
  const double end = std::max(release + out, back);
  total = tube_in + clamp + rails + modules + end;
  *bay_time = total;
  const std::string why =
      all_poses ? ""
                : Fmt("%d steps have no pose, first %s",
                      static_cast<int>(missing.size()),
                      missing.empty() ? "?" : missing[0].c_str());
  rows.push_back(Row(
      "time", "bay", "", "", total, "s", 0.0, all_poses,
      (all_poses ? std::string() : why + "; ") +
          Fmt("tube in %.1f + clamp %.1f + rails %.1f (setting %.1f, "
              "bolting %.1f%s) + modules %.1f (module arm %.1f, pin arm "
              "%.1f%s) + end %.1f",
              tube_in, clamp, rails, sum_set, sum_bolt,
              one_rail_arm ? ", one arm" : "", modules, sum_mod, sum_pin,
              one_arm ? ", one arm" : "", end)));
  for (size_t i = 0; i < arms_.size(); ++i) {
    rows.push_back(Row("time", "busy " + arms_[i].use->name,
                       arms_[i].use->name, "", now[i].busy, "s", 0.0,
                       all_poses, "the sum of its steps"));
  }
  return rows;
}

bool Checker::State(const std::string& kind, int index, LayoutState* state,
                    std::string* error) {
  SolveAll();
  auto it = jobs_.find(kind);
  if (it == jobs_.end() || index < 0 ||
      index >= static_cast<int>(it->second.size())) {
    *error = "no point " + kind + " " + std::to_string(index);
    return false;
  }
  const Job& job = it->second[index];
  if (!job.result.reached) {
    *error = job.point.item + ": " + job.result.why;
    return false;
  }
  LayoutState s;
  const Built& b = job.scenes.front();
  s.rails = static_cast<int>(b.rails.size());
  s.modules = static_cast<int>(b.modules.size());
  s.pallet_modules = kind == "module_take" ? job.point.pallet_modules : b.pallet;
  const ArmCtx& a = arms_[job.arm];
  s.tool[a.use->name] = job.tool;
  s.held[a.use->name] =
      job.held == "module" || job.held == "rail" ? job.held : "";
  s.joints_deg[a.use->name] = ToDeg(job.result.pose.q);
  if (a.use->track) s.track[a.use->name] = job.result.pose.track;
  *state = s;
  return true;
}

}  // namespace

// ---------------------------------------------------------------- public
std::vector<WorkPoint> WorkPoints(const Layout& layout,
                                  const std::string& kind) {
  std::vector<WorkPoint> out;
  const std::vector<bay::Fastening> all = bay::Fastenings();
  const Vec down(0.0, 0.0, -1.0);
  if (kind == "rail_take" || kind == "pins_take") {
    const Stock* s = FindStock(layout, kind == "rail_take" ? "rails" : "pins");
    if (s == nullptr) return out;
    const int n = kind == "rail_take" ? bay::kRailCount : 1;
    for (int k = 0; k < n; ++k) {
      WorkPoint p;
      p.kind = kind;
      p.index = k;
      p.item = Fmt("%s %d", kind.c_str(), k);
      p.pos = V(s->pick_pos);
      p.z = V(s->pick_axis).normalized();
      if (kind == "rail_take") p.rail = k;
      out.push_back(p);
    }
  } else if (kind == "rail_set") {
    for (int k = 0; k < bay::kRailCount; ++k) {
      WorkPoint p;
      p.kind = kind;
      p.index = k;
      p.item = Fmt("rail_set %d", k);
      p.pos = BayPoint(layout, bay::RailPose(k).translation() +
                                   Vec(0.0, 0.0, bay::kRailTopHeight));
      p.z = down;
      p.has_x = true;
      p.x = Vec::UnitX();
      p.rail = k;
      out.push_back(p);
    }
  } else if (kind == "strap_bolt") {
    for (int k = 0; k < bay::kRailCount; ++k) {
      const bay::Fastening& f = all[k];
      WorkPoint p;
      p.kind = kind;
      p.index = k;
      p.item = Fmt("strap_bolt %d", k);
      p.pos = BayPoint(layout, f.pose_a.translation() - kShort * f.tool_axis);
      p.z = f.tool_axis;
      p.rail = k;
      out.push_back(p);
    }
  } else if (kind == "module_take") {
    const double yaw = layout.pallet.yaw_deg * kDeg;
    for (int j = 0; j < layout.pallet.modules; ++j) {
      const int n = layout.pallet.modules - j;
      // The pallet's frame: x along the modules' long side, y along the
      // stack, turned by yaw about z.
      const Vec px(std::cos(yaw), std::sin(yaw), 0.0);
      const Vec py(-std::sin(yaw), std::cos(yaw), 0.0);
      const double deck = layout.pallet.pos[2] + layout.pallet.base_height;
      WorkPoint p;
      p.kind = kind;
      p.index = j;
      p.item = Fmt("module_take %d", j);
      if (layout.pallet.stands == "flat") {
        p.pos = V(layout.pallet.pos) + Vec(0.0, 0.0, -layout.pallet.pos[2]);
        p.pos.z() = deck + (layout.pallet.modules - 1 - j) * layout.pallet.pitch +
                    bay::kModuleThickness;
        p.z = down;
      } else {
        // On its long edge, glass toward -y: the centre of the glass face.
        const double y =
            (j - (layout.pallet.modules - 1) / 2.0) * layout.pallet.pitch -
            bay::kModuleThickness / 2.0;
        p.pos = V(layout.pallet.pos) + y * py;
        p.pos.z() = deck + bay::kModuleWidth / 2.0;
        p.z = py;
      }
      p.has_x = true;
      p.x = px;
      p.pallet_modules = n;
      out.push_back(p);
    }
  } else if (kind == "module_set") {
    for (int m = 0; m < bay::kModuleCount; ++m) {
      WorkPoint p;
      p.kind = kind;
      p.index = m;
      p.item = Fmt("module_set %d", m);
      p.pos = BayPoint(layout, bay::ModulePose(m).translation() +
                                   Vec(0.0, 0.0, bay::kModuleThickness / 2.0));
      p.z = down;
      p.has_x = true;
      p.x = Vec::UnitX();
      p.module = m;
      out.push_back(p);
    }
  } else if (kind == "locate" || kind == "pin" || kind == "swage") {
    for (int m = 0; m < bay::kModuleCount; ++m) {
      for (int i = 0; i < bay::kPinsPerModule; ++i) {
        const bay::Fastening& f =
            all[bay::kRailFasteningCount + m * bay::kPinsPerModule + i];
        WorkPoint p;
        p.kind = kind;
        p.index = m * bay::kPinsPerModule + i;
        p.item = Fmt("%s %d/%d", kind.c_str(), m, i);
        p.pos =
            BayPoint(layout, f.pose_a.translation() - kShort * f.tool_axis);
        p.z = f.tool_axis;
        p.module = m;
        p.hole = i;
        p.rail = m + i / 2;
        out.push_back(p);
      }
    }
  }
  return out;
}

double JointMoveTime(const std::array<double, 6>& change_deg,
                     const std::array<double, 6>& speed_deg_s) {
  double t = 0.0;
  for (int j = 0; j < 6; ++j) {
    if (speed_deg_s[j] > 0.0) {
      t = std::max(t, std::abs(change_deg[j]) / speed_deg_s[j]);
    }
  }
  return kAccelFactor * t;
}

double SideBySide(const std::vector<double>& lead,
                  const std::vector<double>& follow) {
  if (lead.empty() || follow.empty()) return 0.0;
  double sum_lead = 0.0, sum_follow = 0.0;
  for (double t : lead) sum_lead += t;
  for (double t : follow) sum_follow += t;
  return std::max(sum_lead + follow.back(), lead.front() + sum_follow);
}

double TrackMoveTime(double distance, double speed, double accel) {
  if (distance <= 0.0) return 0.0;
  if (speed <= 0.0 || accel <= 0.0) return std::numeric_limits<double>::infinity();
  return distance / speed + speed / accel;
}

std::string TimeModelText() {
  return Fmt(
      "A move of an arm from one pose to the next: %.1f x the largest over "
      "its six joints of |change| / (the datasheet's greatest speed of that "
      "joint), the factor for speeding up and slowing down; its carriage "
      "moves at the same time and takes distance / speed + speed / "
      "acceleration of the track; the longer of the two counts, then %.1f s "
      "to settle. A move with no change takes nothing. Moves go straight "
      "from the pose at one point to the pose at the next: approaches, "
      "retreats and the paths between are not modelled or checked.\n"
      "A working step is its move from the arm's last pose and the layout's "
      "process time for the kind. A tool change takes the arm's "
      "tool_change_s; the move to where the tools wait is not counted. "
      "Every arm starts the bay at its wait pose with its first tool.\n"
      "The bay: tube in + clamp + the rail phase + the module phase + the "
      "end. In the rail phase, when one arm sets the rails and another "
      "drives the strap bolts, the bolting arm works one rail behind: "
      "max(the setting arm's steps + the bolting arm's steps on the last "
      "rail, the setting arm's steps on the first rail + the bolting arm's "
      "steps), each rail's take counted with the arm that takes; when one "
      "arm does both, the plain sum in the layout's rail_order. In the "
      "module phase the module arm and the pin arm work side by side the "
      "same way, the pin arm one module behind: max(the module arm's steps "
      "+ the pin arm's steps on the last module, the module arm's steps on "
      "the first module + the pin arm's steps), the magazine taken with the "
      "first module's; if one arm does both, the plain sum. An arm's steps "
      "include its tool changes. The end: release + bay out, or the longest of "
      "the arms' moves back to their wait poses and first tools if that is "
      "longer.",
      kAccelFactor, kSettleS);
}

CheckReport CheckLayout(const Layout& layout, const ModelPaths& paths,
                        const CheckOptions& options) {
  CheckReport report;
  report.time_model = TimeModelText();
  Checker checker(layout, options);
  std::string error;
  const bool ok = checker.Init(paths, &error);
  auto want = [&](const char* kind) { return Wanted(options.checks, kind); };
  if (want("transport")) {
    for (CheckRow& r : checker.TransportRows(paths)) report.rows.push_back(r);
  }
  if (!ok) {
    report.error = error;
    for (const char* kind :
         {"hold", "load", "reach", "clearance", "ways", "time"}) {
      if (want(kind)) {
        report.rows.push_back(Row(kind, "model", "", "", 0.0, "", 0.0, false,
                                  "no model: " + error));
      }
    }
    report.pass = false;
    return report;
  }
  if (want("hold")) {
    for (CheckRow& r : checker.HoldRows()) report.rows.push_back(r);
  }
  if (want("load")) {
    for (CheckRow& r : checker.LoadRows()) report.rows.push_back(r);
  }
  const bool time = want("time") && options.work_kinds.empty();
  if (want("reach") || want("clearance") || time) checker.SolveAll();
  if (want("reach")) {
    for (CheckRow& r : checker.ReachRows()) report.rows.push_back(r);
  }
  if (want("clearance")) {
    for (CheckRow& r : checker.ClearanceRows()) report.rows.push_back(r);
  }
  if (want("ways")) {
    for (CheckRow& r : checker.WaysRows()) report.rows.push_back(r);
  }
  if (time) {
    for (CheckRow& r : checker.TimeRows(&report.steps, &report.bay_time_s)) {
      report.rows.push_back(r);
    }
  }
  report.pass = !report.rows.empty();
  for (const CheckRow& r : report.rows) report.pass &= r.pass;
  return report;
}

bool StateAt(const Layout& layout, const ModelPaths& paths,
             const std::string& kind, int index, LayoutState* state,
             std::string* error) {
  CheckOptions options;
  Checker checker(layout, options);
  std::string e;
  if (!checker.Init(paths, &e)) {
    if (error != nullptr) *error = e;
    return false;
  }
  std::string why;
  const bool ok = checker.State(kind, index, state, &why);
  if (!ok && error != nullptr) *error = why;
  return ok;
}

}  // namespace cell::layout
