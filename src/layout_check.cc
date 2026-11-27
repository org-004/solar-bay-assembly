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
#include <mutex>
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
#include "src/layout_rows.h"
#include "src/layout_scene.h"
#include "src/layout_stage_rows.h"
#include "src/layout_stages.h"
#include "src/mj_handles.h"
#include "src/portable_random.h"

namespace cell::layout {
namespace {

using Vec = Eigen::Vector3d;
using Q6 = std::array<double, 6>;  // radians

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kFar = kSceneFar;        // m, the most a distance looks.
constexpr double kPinMagazineMass = 0.5;  // kg, ours.
constexpr double kShort = 0.005;          // A fastening tool stops this short.
constexpr double kPosTol = 1e-4;          // m
constexpr double kAngTol = 1e-3;          // rad
constexpr double kOrientWeight = 0.5;     // m per rad in the IK's error.
constexpr double kBoltFreeLength = bay::kStrapBoltFreeLength;  // m
constexpr double kLegStep = 0.005;        // m, the most between two samples.
constexpr double kLegJumpDeg = 10.0;      // The most a joint turns between them.

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

// ------------------------------------------------------------ the arms
// The checker's own state of an arm: the scene's arm, the seeds of the
// inverse kinematics and the arm's last solution.
struct ArmCtx {
  const SceneArm* s = nullptr;
  std::vector<Q6> seeds;
  std::optional<Q6> last;
};

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

using Pose = ArmPose;

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

// What the walk along one leg (approach or retreat) found.
struct LegResult {
  bool walked = false;
  bool reached = false;      // Every sample, the far pose included.
  Q6 far{};                  // The far pose (the last sample reached).
  double length = 0.0;       // m
  int samples = 0;
  double figure = kFar;
  Nearest nearest;
  bool by_design = false;    // The figure's pair is by design.
  bool at_far = false;       // The figure is the far pose's.
  double at_m = 0.0;         // Where along the leg the figure is.
  double margin_deg = 1e9;   // Least joint margin over the leg.
  std::string margin_joint;
  std::string held;
  std::string why;           // Not walked or not reached: why.
  std::vector<Q6> path;      // The work pose, then every sample reached.
};

// ------------------------------------------------------------ the checker
class Checker {
 public:
  Checker(const Layout& layout, const CheckOptions& options)
      : L_(layout), opt_(options) {}

  bool Init(const ModelPaths& paths, std::string* error);

  // Everything of the reach and clearance checks, for the work kinds wanted,
  // and the legs of every point whose kind has an approach.
  void SolveAll();

  std::vector<CheckRow> TransportRows(const ModelPaths& paths);
  std::vector<CheckRow> HoldRows();
  std::vector<CheckRow> LoadRows();
  std::vector<CheckRow> ReachRows();
  std::vector<CheckRow> ClearanceRows();
  std::vector<CheckRow> WaysRows();
  std::vector<CheckRow> TimeRows(std::vector<BayStep>* steps, double* bay);
  std::vector<CheckRow> ApproachRows();
  std::vector<SolvedPoint> Points() const;
  // The tool-change poses, solved on the first call.
  std::vector<SolvedToolChange> ToolChanges();
  // The flat rack's rows of the tool-change poses: reach, clearance.
  std::vector<CheckRow> ToolChangeRows(const std::string& check);
  const std::vector<std::pair<std::string, double>>& phases() const {
    return phases_;
  }
  std::vector<PostureChanges> postures() const {
    return OnFlatRacks(L_) ? postures_ : std::vector<PostureChanges>();
  }
  // The flat rack's rows of the wait poses: reach, clearance.
  std::vector<CheckRow> WaitPoseRows(const std::string& check);

  // The state for a picture.
  bool State(const std::string& kind, int index, LayoutState* state,
             std::string* error);

 private:
  struct Job {  // One point of one kind, as solved.
    WorkPoint point;
    StepScene sc;  // Who, with what, what is there, what is by design.
    PointResult result;
    LegResult legs[2];  // Approach, retreat.
    bool legs_done = false;  // Walked for the chosen pose already.
    bool in_scope = true;    // A point of the stage checked (CheckOptions::stage).
  };

  const mjModel& M() const { return scene_->model(); }
  const LayoutHandles& H() const { return scene_->handles(); }
  const ArmUse& Use(int arm) const { return *arms_[arm].s->use; }
  void AllAtWait() { scene_->AllAtWait(data_.get()); }
  void SetPose(int arm, const Pose& p) { scene_->SetPose(data_.get(), arm, p); }
  IkResult Solve(int arm, int site, const Target& t, double track,
                 const Q6& seed);
  std::vector<std::pair<double, std::string>> TrackCandidates(
      int arm, const WorkPoint& p, int count, std::string* why) const;
  CheckRow ToolInFreeSpace(const std::string& kind, const ArmUse& arm,
                           const Tool& tool) const;
  Nearest PoseClearance(Job& job);
  void SolveJob(Job& job, int count);
  void SolvePosture(Job& job, int arm, int site,
                    const std::vector<std::pair<double, std::string>>& tracks,
                    const std::vector<Target>& targets);
  // The same pose with every joint the whole turns nearest the anchor that
  // keep it inside its range.
  Q6 Unwind(int arm, const Q6& q, const Q6& anchor) const;
  // The largest over the joints of |change| / the datasheet's speed, s.
  double PostureSeconds(int arm, const Q6& from, const Q6& to) const;
  // Walks one leg of the job's pose (job.result) into job.legs[leg].
  void WalkLeg(Job& job, int leg, const Approach& rule);
  bool LegPasses(const LegResult& r) const {
    return r.walked && r.why.empty() && r.reached &&
           r.figure >= opt_.clearance_m;
  }
  Nearest FreeSpace(const Job& job, std::string* which);
  // The scene's AtWorkPoint, with the bay as built in staged work
  // (CheckOptions::stage): the earlier stages' rails and modules on, the
  // later stages' off.
  StepScene At(const WorkPoint& p, Leg leg) const;
  bool InScope(const std::string& kind, int index) const;
  // The bay as built when the stage starts and when it ends (the whole
  // bay's bare tube and full bay without stages).
  Built StageStart() const;
  Built StageEnd() const;
  std::string Key(const Job& job) const;
  std::vector<std::pair<std::string, int>> StepOrder() const;
  double MoveTime(int arm, const Pose& from, const Pose& to) const;
  void Margin(int arm, const Q6& q, double* margin, std::string* joint) const;

  const Layout& L_;
  CheckOptions opt_;
  std::unique_ptr<LayoutScene> scene_;
  DataPtr data_;
  std::vector<ArmCtx> arms_;
  // Solved points by kind, in index order.
  std::map<std::string, std::vector<Job>> jobs_;
  std::map<std::string, std::vector<std::string>> kind_errors_;
  bool solved_ = false;
  std::vector<std::pair<std::string, double>> phases_;
  std::vector<PostureChanges> postures_;
  struct ToolChange {
    int arm = -1;
    bool reached = false;
    Pose pose;
    Vec flange = Vec::Zero();  // Where the rule puts the flange.
    double margin_deg = 0.0;
    std::string margin_joint;
    Nearest nearest;           // The worst over its tools.
    std::string worst_tool;
    std::string why;
  };
  std::vector<ToolChange> tool_changes_;
  bool tool_changes_solved_ = false;
  void SolveToolChanges();
};

bool Checker::Init(const ModelPaths& paths, std::string* error) {
  scene_ = LayoutScene::Load(L_, paths, opt_.joint_margin_deg, error);
  if (!scene_) return false;
  data_ = scene_->MakeData();
  std::mt19937 rng(20261124u);
  for (int i = 0; i < scene_->ArmCount(); ++i) {
    ArmCtx a;
    a.s = &scene_->arm(i);
    // The wait pose first, then the fixed draws: one draw per statement.
    a.seeds.push_back(a.s->wait);
    for (int s = 0; s < opt_.ik_seeds; ++s) {
      Q6 q;
      for (int j = 0; j < 6; ++j) {
        const double u = UniformUnit(rng);
        q[j] = a.s->lo[j] + u * (a.s->hi[j] - a.s->lo[j]);
      }
      a.seeds.push_back(q);
    }
    arms_.push_back(a);
  }
  return true;
}

// Damped least squares over six joints, the carriage at `track`.
IkResult Checker::Solve(int arm, int site, const Target& t, double track,
                        const Q6& seed) {
  const mjModel& m = M();
  const SceneArm& a = *arms_[arm].s;
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
    int arm, const WorkPoint& p, int count, std::string* why) const {
  const ArmUse& use = Use(arm);
  std::vector<std::pair<double, std::string>> out;
  if (!use.track) {
    out.push_back({0.0, "no track"});
    return out;
  }
  const Track& tr = *use.track;
  auto stop = use.stops.find(p.kind);
  if (stop != use.stops.end() && !stop->second.empty()) {
    const std::vector<double>& v = stop->second;
    const double value = v.size() == 1 ? v[0]
                         : p.index < static_cast<int>(v.size()) ? v[p.index]
                                                                : 0.0;
    if ((v.size() == 1 || static_cast<int>(v.size()) == count) &&
        (value < tr.lo - 1e-9 || value > tr.hi + 1e-9)) {
      *why = Fmt("stop %.3f outside the track's %.3f..%.3f", value, tr.lo,
                 tr.hi);
      return {};
    }
    if (v.size() == 1) {
      out.push_back({v[0], Fmt("track %.3f (the stop)", v[0])});
    } else if (static_cast<int>(v.size()) == count && p.index < count) {
      out.push_back({v[p.index], Fmt("track %.3f (stop %d)", v[p.index],
                                     p.index)});
    }
    return out;  // Empty when the stops do not fit: the caller says so.
  }
  const Vec dir = V(tr.direction);
  const double s0 = (p.pos - V(use.base_pos)).dot(dir) / dir.squaredNorm();
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

bool Has(const std::vector<int>& v, int x) {
  return std::find(v.begin(), v.end(), x) != v.end();
}

StepScene Checker::At(const WorkPoint& p, Leg leg) const {
  StepScene sc = scene_->AtWorkPoint(p, leg);
  if (opt_.stage) {
    BuiltInStage(*opt_.stage, PointCount(L_, "module_take"), &sc);
  }
  return sc;
}

bool Checker::InScope(const std::string& kind, int index) const {
  if (!opt_.stage) return true;
  auto it = opt_.stage->points.find(kind);
  return it != opt_.stage->points.end() && Has(it->second, index);
}

Built Checker::StageStart() const {
  return BuiltAtStageStart(opt_.stage, PointCount(L_, "module_take"));
}

Built Checker::StageEnd() const {
  return BuiltAtStageEnd(opt_.stage, PointCount(L_, "module_take"));
}

Nearest Checker::PoseClearance(Job& job) {
  // The data holds the active arm at its pose and the others at wait.
  const StepScene& sc = job.sc;
  const auto moving = scene_->Moving(sc.arm, sc.tool, sc.held);
  Nearest worst;
  worst.distance = kFar + 1.0;
  int which = 0;
  for (size_t s = 0; s < sc.built.size(); ++s) {
    const auto obstacles =
        scene_->Obstacles(sc.arm, sc.tool, sc.built[s], sc.pairs);
    const Nearest n = scene_->LeastDistance(data_.get(), moving, obstacles);
    if (n.distance < worst.distance) {
      worst = n;
      which = static_cast<int>(s);
    }
  }
  job.result.scene_note =
      sc.built.size() > 1
          ? (which == 1 ? "worse with the next module on"
                        : "worse before the next module")
          : "";
  return worst;
}

void Checker::Margin(int arm, const Q6& q, double* margin,
                     std::string* joint) const {
  const SceneArm& a = *arms_[arm].s;
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
  if (!job.sc.error.empty()) {
    r.why = job.sc.error;
    return;
  }
  const int ai = job.sc.arm;
  ArmCtx& a = arms_[ai];
  const int site = a.s->h->work_site.at(job.sc.tool);
  std::string why;
  const auto tracks = TrackCandidates(ai, job.point, count, &why);
  if (tracks.empty()) {
    r.why = !why.empty()
                ? why
                : Fmt("the layout's stops for %s do not fit its %d points",
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
  if (!L_.approaches.empty()) {
    SolvePosture(job, ai, site, tracks, targets);
    return;
  }
  std::vector<Q6> seeds;
  if (a.last) seeds.push_back(*a.last);
  seeds.insert(seeds.end(), a.seeds.begin(), a.seeds.end());
  bool have = false;
  double best_perr = 1e9;
  // With a rule for the kind, a candidate wins only if both its legs pass
  // as well; the first reached and clear one is kept, with its legs, in case
  // none does.
  const Approach* rule = FindApproach(L_, job.point.kind);
  std::optional<PointResult> first_clear;
  LegResult first_legs[2];
  for (const auto& [track, note] : tracks) {
    for (const Q6& seed : seeds) {
      for (const Target& tg : targets) {
        AllAtWait();
        const IkResult ik = Solve(ai, site, tg, track, seed);
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
        if (!first_clear && (!have || n.distance > r.nearest.distance)) {
          have = true;
          r.reached = true;
          r.clear = clear;
          r.pose = {ik.q, track};
          r.perr = ik.perr;
          r.track_note = note;
          r.nearest = n;
        }
        if (!clear) continue;
        if (rule != nullptr) {
          PointResult candidate = r;
          candidate.reached = true;
          candidate.clear = true;
          candidate.pose = {ik.q, track};
          candidate.perr = ik.perr;
          candidate.track_note = note;
          candidate.nearest = n;
          const PointResult keep = r;
          r = candidate;
          WalkLeg(job, 0, *rule);
          WalkLeg(job, 1, *rule);
          const bool legs = LegPasses(job.legs[0]) && LegPasses(job.legs[1]);
          if (!legs) {
            r = keep;
            if (!first_clear) {
              first_clear = candidate;
              first_legs[0] = job.legs[0];
              first_legs[1] = job.legs[1];
            }
            continue;
          }
          job.legs_done = true;
        }
        a.last = ik.q;
        // Leave the data at the chosen pose for what follows.
        AllAtWait();
        SetPose(ai, r.pose);
        mj_kinematics(&M(), data_.get());
        PoseClearance(job);
        Margin(ai, r.pose.q, &r.margin_deg, &r.margin_joint);
        return;
      }
    }
  }
  if (first_clear) {
    // Today's choice, the first reached and clear pose; its legs fail.
    r = *first_clear;
    job.legs[0] = first_legs[0];
    job.legs[1] = first_legs[1];
    job.legs_done = true;
    a.last = r.pose.q;
    AllAtWait();
    SetPose(ai, r.pose);
    mj_kinematics(&M(), data_.get());
    PoseClearance(job);
    Margin(ai, r.pose.q, &r.margin_deg, &r.margin_joint);
    return;
  }
  if (r.reached) {
    a.last = r.pose.q;
    AllAtWait();
    SetPose(ai, r.pose);
    mj_kinematics(&M(), data_.get());
    r.nearest = PoseClearance(job);
    Margin(ai, r.pose.q, &r.margin_deg, &r.margin_joint);
  } else {
    Margin(ai, r.pose.q, &r.margin_deg, &r.margin_joint);
    r.why = Fmt("not reached: %.3f m short at best", best_perr);
  }
}

Q6 Checker::Unwind(int arm, const Q6& q, const Q6& anchor) const {
  const SceneArm& a = *arms_[arm].s;
  Q6 out = q;
  for (int j = 0; j < 6; ++j) {
    for (int k = -3; k <= 3; ++k) {
      const double v = q[j] + 2.0 * std::numbers::pi * k;
      if (v < a.lo[j] - 1e-12 || v > a.hi[j] + 1e-12) continue;
      if (std::abs(v - anchor[j]) < std::abs(out[j] - anchor[j])) out[j] = v;
    }
  }
  return out;
}

double Checker::PostureSeconds(int arm, const Q6& from, const Q6& to) const {
  const SceneArm& a = *arms_[arm].s;
  double t = 0.0;
  for (int j = 0; j < 6; ++j) {
    const double speed = a.cat->joint_speed_deg_s[j];
    if (speed > 0.0) t = std::max(t, std::abs(to[j] - from[j]) / kDeg / speed);
  }
  return t;
}

// The pose choice of a layout with approach rules: every seed's solution,
// its windings brought nearest the arm's anchor, tried nearest first.
void Checker::SolvePosture(
    Job& job, int ai, int site,
    const std::vector<std::pair<double, std::string>>& tracks,
    const std::vector<Target>& targets) {
  PointResult& r = job.result;
  ArmCtx& a = arms_[ai];
  const Q6 anchor = a.last ? *a.last : a.s->wait;
  std::vector<Q6> seeds = {anchor};
  seeds.insert(seeds.end(), a.seeds.begin(), a.seeds.end());
  const Approach* rule = FindApproach(L_, job.point.kind);
  std::optional<PointResult> first_clear;
  LegResult first_legs[2];
  bool have = false;
  double best_perr = 1e9;
  auto settle = [&](const PointResult& chosen) {
    r = chosen;
    a.last = r.pose.q;
    AllAtWait();
    SetPose(ai, r.pose);
    mj_kinematics(&M(), data_.get());
    r.nearest = PoseClearance(job);
    Margin(ai, r.pose.q, &r.margin_deg, &r.margin_joint);
  };
  for (const auto& [track, note] : tracks) {
    struct Candidate {
      Q6 q;
      double perr;
      double seconds;
    };
    std::vector<Candidate> found;
    for (const Q6& seed : seeds) {
      for (const Target& tg : targets) {
        AllAtWait();
        const IkResult ik = Solve(ai, site, tg, track, seed);
        if (!ik.ok) {
          if (!r.reached && ik.perr < best_perr) {
            best_perr = ik.perr;
            r.perr = ik.perr;
            r.track_note = note;
            r.pose = {ik.q, track};
          }
          continue;
        }
        const Q6 q = Unwind(ai, ik.q, anchor);
        bool seen = false;
        for (const Candidate& c : found) {
          bool same = true;
          for (int j = 0; j < 6; ++j) {
            same = same && std::abs(c.q[j] - q[j]) < 0.5 * kDeg;
          }
          seen = seen || same;
        }
        if (!seen) found.push_back({q, ik.perr, PostureSeconds(ai, anchor, q)});
      }
    }
    std::stable_sort(found.begin(), found.end(),
                     [](const Candidate& x, const Candidate& y) {
                       return x.seconds < y.seconds;
                     });
    for (const Candidate& c : found) {
      double margin;
      std::string joint;
      Margin(ai, c.q, &margin, &joint);
      if (margin < opt_.joint_margin_deg - 1e-9) continue;
      AllAtWait();
      SetPose(ai, {c.q, track});
      mj_kinematics(&M(), data_.get());
      const Nearest n = PoseClearance(job);
      const bool clear = n.distance >= opt_.clearance_m;
      PointResult candidate = r;
      candidate.reached = true;
      candidate.clear = clear;
      candidate.pose = {c.q, track};
      candidate.perr = c.perr;
      candidate.track_note = note;
      candidate.nearest = n;
      candidate.why.clear();
      if (!first_clear && (!have || n.distance > r.nearest.distance)) {
        have = true;
        r = candidate;
      }
      if (!clear) continue;
      if (rule != nullptr) {
        const PointResult keep = r;
        r = candidate;
        WalkLeg(job, 0, *rule);
        WalkLeg(job, 1, *rule);
        const bool legs = LegPasses(job.legs[0]) && LegPasses(job.legs[1]);
        r = keep;
        if (!legs) {
          if (!first_clear) {
            first_clear = candidate;
            first_legs[0] = job.legs[0];
            first_legs[1] = job.legs[1];
          }
          continue;
        }
        job.legs_done = true;
      }
      settle(candidate);
      return;
    }
  }
  if (first_clear) {
    job.legs[0] = first_legs[0];
    job.legs[1] = first_legs[1];
    job.legs_done = true;
    settle(*first_clear);
    return;
  }
  if (r.reached) {
    settle(r);
  } else {
    Margin(ai, r.pose.q, &r.margin_deg, &r.margin_joint);
    r.why = Fmt("not reached: %.3f m short at best", best_perr);
  }
}

// True if the nearest pair is one of the step's pairs by design.
bool ByDesign(const DesignPairs& p, const std::vector<MovingGeom>& moving,
              const Nearest& n) {
  if (n.moving < 0) return false;
  unsigned group = 0;
  for (const MovingGeom& g : moving) {
    if (g.geom == n.moving) group = g.group;
  }
  const int o = n.obstacle;
  if ((group & kHeld) && (p.held_skip.count(o) || p.held_near.count(o))) {
    return true;
  }
  if ((group & kToolLast) && p.last_near.count(o)) return true;
  if ((group & (kToolBody | kToolLast)) && p.tool_near.count(o)) return true;
  return false;
}

// Walks one leg of a reached work point: from the work pose straight to the
// approach (leg 0) or retreat (leg 1) pose, at most kLegStep apart, each
// sample solved from the one before with the carriage held at the work
// pose's track value.
void Checker::WalkLeg(Job& job, int leg, const Approach& rule) {
  LegResult& r = job.legs[leg];
  r = LegResult();
  r.walked = true;
  const PointResult& w = job.result;
  const StepScene sc =
      At(job.point, leg == 0 ? Leg::kApproach : Leg::kRetreat);
  r.held = sc.held;
  if (!sc.error.empty() || !w.reached) {
    r.why = "no work pose: " +
            (!sc.error.empty() ? sc.error
                               : (w.why.empty() ? "not solved" : w.why));
    return;
  }
  const int ai = sc.arm;
  const int site = arms_[ai].s->h->work_site.at(sc.tool);
  const std::string& frame = leg == 0 ? rule.approach_frame : rule.retreat_frame;
  const Vec3& dir3 = leg == 0 ? rule.approach_dir : rule.retreat_dir;
  r.length = leg == 0 ? rule.approach_m : rule.retreat_m;
  r.far = w.pose.q;
  mjData* d = data_.get();
  AllAtWait();
  SetPose(ai, w.pose);
  mj_kinematics(&M(), d);
  const Vec p0(d->site_xpos[3 * site], d->site_xpos[3 * site + 1],
               d->site_xpos[3 * site + 2]);
  Eigen::Matrix3d r0;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) r0(i, j) = d->site_xmat[9 * site + 3 * i + j];
  }
  Vec dir = V(dir3);
  if (frame == "tool") {
    dir = r0 * dir;
  } else if (frame != "world") {
    r.why = "the rule's frame \"" + frame + "\" is not \"tool\" or \"world\"";
    return;
  }
  if (dir.norm() < 1e-9) {
    r.why = "the rule's direction is zero";
    return;
  }
  dir.normalize();
  const int n = std::max(
      1, static_cast<int>(std::ceil(r.length / kLegStep - 1e-9)));
  r.samples = n;
  const auto moving = scene_->Moving(ai, sc.tool, sc.held);
  // Along the move the pairs that touch by design count as near: they may
  // close to zero but not go below it. At the far pose every pair counts.
  DesignPairs along = sc.pairs;
  along.held_near.insert(along.held_skip.begin(), along.held_skip.end());
  along.held_skip.clear();
  // At the work pose (sample 0) the pairs that touch there by design are
  // not counted: the held part on what it is set on or taken from, and the
  // tool on the part it is about to take or has just let go.
  std::vector<std::vector<Obstacle>> at_work, on_way, at_far;
  for (const Built& b : sc.built) {
    on_way.push_back(scene_->Obstacles(ai, sc.tool, b, along));
    std::vector<Obstacle> work = scene_->Obstacles(ai, sc.tool, b, sc.pairs);
    for (Obstacle& o : work) {
      if (sc.pairs.tool_near.count(o.geom)) o.near &= ~(kToolBody | kToolLast);
    }
    at_work.push_back(work);
    at_far.push_back(scene_->Obstacles(ai, sc.tool, b, DesignPairs()));
  }
  Target t;
  t.has_x = true;
  t.x = r0.col(0);
  t.z = r0.col(2);
  Q6 q = w.pose.q;
  r.reached = true;
  // Sample 0 is the work pose itself, in the leg's own scene: the data is
  // there already.
  for (int i = 0; i <= n; ++i) {
    const double s = r.length * i / n;
    IkResult ik;
    ik.ok = true;
    ik.q = q;
    if (i > 0) {
      t.p = p0 + s * dir;
      ik = Solve(ai, site, t, w.pose.track, q);
    }
    double margin;
    std::string joint;
    Margin(ai, ik.q, &margin, &joint);
    if (!ik.ok || margin < opt_.joint_margin_deg - 1e-9) {
      r.reached = false;
      std::string at_limit;
      const SceneArm& a = *arms_[ai].s;
      for (int j = 0; j < 6; ++j) {
        if (std::abs(ik.q[j] - a.lo[j]) < 1e-9 ||
            std::abs(ik.q[j] - a.hi[j]) < 1e-9) {
          at_limit = a.cat->joint_names[j];
          break;
        }
      }
      r.why = Fmt("not reached at %.1f of %.1f mm: ", s * 1000.0,
                  r.length * 1000.0) +
              (!ik.ok ? Fmt("%.2f mm and %.3f deg off", ik.perr * 1000.0,
                            ik.aerr / kDeg) +
                            (at_limit.empty()
                                 ? std::string()
                                 : "; joint " + at_limit + " held at its margin")
                      : "joint " + joint + Fmt(" inside its margin (%.1f deg)",
                                               margin));
      r.at_m = s;
      return;
    }
    // The arm stays on the work pose's branch: no joint jumps.
    for (int j = 0; j < 6; ++j) {
      const double jump = std::abs(ik.q[j] - q[j]) / kDeg;
      if (jump > kLegJumpDeg) {
        r.reached = false;
        r.why = Fmt("joint %s jumps %.1f deg at %.1f mm: another branch",
                    arms_[ai].s->cat->joint_names[j].c_str(), jump,
                    s * 1000.0);
        r.at_m = s;
        return;
      }
    }
    q = ik.q;
    r.path.push_back(q);
    if (margin < r.margin_deg) {
      r.margin_deg = margin;
      r.margin_joint = joint;
    }
    // The data is at the sample: Solve leaves it there.
    const bool far = i == n;
    for (size_t b = 0; b < sc.built.size(); ++b) {
      const Nearest nd =
          scene_->LeastDistance(d, moving, far ? at_far[b]
                                           : i == 0 ? at_work[b]
                                                    : on_way[b]);
      if (nd.distance < r.figure || r.nearest.moving < 0) {
        r.figure = nd.distance;
        r.nearest = nd;
        r.at_m = s;
        r.at_far = far;
        r.by_design = far ? ByDesign(sc.pairs, moving, nd) : nd.near;
      }
    }
  }
  r.far = q;
}

// Points with one key share one pose: the same arm, tool, target and part,
// and the same rule of approach (the pose is chosen by its legs too).
std::string Checker::Key(const Job& job) const {
  const WorkPoint& p = job.point;
  std::string key =
      Fmt("%d|%s|%.6f,%.6f,%.6f|%.6f,%.6f,%.6f|%d|%.6f,%.6f,%.6f|%s",
          job.sc.arm, job.sc.tool.c_str(), p.pos.x(), p.pos.y(), p.pos.z(),
          p.z.x(), p.z.y(), p.z.z(), p.has_x ? 1 : 0, p.x.x(), p.x.y(),
          p.x.z(), job.sc.held.c_str());
  if (const Approach* a = FindApproach(L_, p.kind)) {
    key += Fmt("|%s %.6f,%.6f,%.6f %.6f|%s %.6f,%.6f,%.6f %.6f",
               a->approach_frame.c_str(), a->approach_dir[0],
               a->approach_dir[1], a->approach_dir[2], a->approach_m,
               a->retreat_frame.c_str(), a->retreat_dir[0], a->retreat_dir[1],
               a->retreat_dir[2], a->retreat_m);
  }
  return key;
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
  // The module arms' work, then the pin arms' (each arm's chain of seeds
  // is its own, so their interleaving does not matter).
  for (int m = 0; m < M; ++m) {
    out.push_back({"module_take", m});
    out.push_back({"module_set", m});
  }
  const int magazines = PointCount(L_, "pins_take");
  for (int i = 0; i < magazines; ++i) out.push_back({"pins_take", i});
  for (int m = 0; m < M; ++m) {
    for (int i : bay::kLocateOrder) {
      out.push_back({"locate", 4 * m + i});
      out.push_back({"pin", 4 * m + i});
    }
    for (int i : bay::kSwageOrder) out.push_back({"swage", 4 * m + i});
  }
  const int takes = PointCount(L_, "module_take");
  for (int j = M; j < takes; ++j) out.push_back({"module_take", j});
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
      job.in_scope = InScope(kind, p.index);
      job.sc = At(p, Leg::kWork);
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
    if (job.result.solved || !job.in_scope) continue;
    const int count = static_cast<int>(it->second.size());
    const std::string key = job.sc.error.empty() ? Key(job) : "";
    auto same = key.empty() ? done.end() : done.find(key);
    if (same != done.end()) {
      const Job& other = jobs_[same->second.first][same->second.second];
      job.result = other.result;
      // The clearance is the same pose's, but against this kind's scenes.
      if (job.result.reached) {
        AllAtWait();
        SetPose(job.sc.arm, job.result.pose);
        mj_kinematics(&M(), data_.get());
        job.result.nearest = PoseClearance(job);
        job.result.clear = job.result.nearest.distance >= opt_.clearance_m;
      }
      continue;
    }
    SolveJob(job, count);
    if (!key.empty()) done[key] = {kind, index};
  }
  // The legs, for the kinds that have an approach.
  for (auto& [kind, list] : jobs_) {
    const Approach* rule = FindApproach(L_, kind);
    if (rule == nullptr) continue;
    for (Job& job : list) {
      if (job.legs_done || !job.in_scope) continue;
      WalkLeg(job, 0, *rule);
      WalkLeg(job, 1, *rule);
    }
  }
}

// ------------------------------------------------------------ rows
CheckRow Row(const std::string& check, const std::string& item,
             const std::string& arm, const std::string& tool, double figure,
             const std::string& unit, double limit, bool pass,
             const std::string& detail) {
  return {check, item, arm, tool, figure, unit, limit, pass, detail};
}

// How far a track value is inside the track's range (negative: outside).
double InsideTrack(const Track& t, double s) {
  return std::min(s - t.lo, t.hi - s);
}

std::vector<CheckRow> Checker::ReachRows() {
  std::vector<CheckRow> rows;
  // Where each arm waits must be on its track.
  for (const ArmCtx& a : arms_) {
    const ArmUse& use = *a.s->use;
    if (!use.track) continue;
    const double in = InsideTrack(*use.track, use.wait_track);
    rows.push_back(Row("reach", "wait track " + use.name, use.name, "", in,
                       "m", 0.0, in >= -1e-9,
                       Fmt("wait_track %.3f on the track's %.3f..%.3f",
                           use.wait_track, use.track->lo, use.track->hi)));
  }
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
      if (!job.in_scope) continue;
      const PointResult& r = job.result;
      const std::string arm = job.sc.arm >= 0 ? Use(job.sc.arm).name : "";
      std::string detail;
      if (r.reached) {
        detail = Fmt("least margin at %s; %s", r.margin_joint.c_str(),
                     r.track_note.c_str());
      } else {
        detail = r.why + (r.track_note.empty() ? "" : "; " + r.track_note);
      }
      rows.push_back(Row("reach", job.point.item, arm, job.sc.tool,
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
  const mjModel& m = M();
  const LayoutHandles& H = this->H();
  mjData* d = data_.get();
  const int arm = job.sc.arm;
  AllAtWait();
  if (arm >= 0 && job.result.reached) SetPose(arm, job.result.pose);
  // Point a and the probe's orientation.
  Eigen::Isometry3d at;
  std::vector<int> probes;
  std::set<int> joined;
  auto ins = [&](const std::vector<int>& g) { joined.insert(g.begin(), g.end()); };
  const std::vector<bay::Fastening> all = bay::Fastenings();
  if (p.kind == "strap_bolt") {
    at = all[p.index].pose_a;
    probes = {H.probe_bolt_geom};
    if (p.index < static_cast<int>(H.rail_geoms.size())) {
      ins(H.rail_geoms[p.index]);
    }
    ins(H.tube_geoms);
  } else {
    at = all[bay::kRailFasteningCount + p.index].pose_a;
    probes = p.kind == "pin" ? std::vector<int>{H.probe_pin_geom}
                             : std::vector<int>{H.probe_locate_below_geom,
                                                H.probe_locate_above_geom};
    if (p.rail < static_cast<int>(H.rail_geoms.size())) {
      ins(H.rail_geoms[p.rail]);
    }
    if (p.module < static_cast<int>(H.module_geoms.size())) {
      ins(H.module_geoms[p.module]);
    }
  }
  Nearest worst;
  worst.distance = kFar + 1.0;
  if (H.probe_mocap < 0) {
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
  for (int i = 0; i < 3; ++i) d->mocap_pos[3 * H.probe_mocap + i] = pos[i];
  for (int i = 0; i < 4; ++i) d->mocap_quat[4 * H.probe_mocap + i] = quat[i];
  mj_kinematics(&m, d);
  std::vector<MovingGeom> moving;
  for (int g : probes) {
    if (g >= 0) moving.push_back({g, kProbe});
  }
  if (moving.empty()) {
    *which = "the model has no such probe cylinder";
    worst.distance = -1.0;
    return worst;
  }
  int scene_index = 0;
  for (size_t s = 0; s < job.sc.built.size(); ++s) {
    // Everything there, the active arm at its pose too, except its tool and
    // the joined parts.
    std::vector<Obstacle> obstacles = scene_->Obstacles(
        arm >= 0 ? arm : 0, job.sc.tool, job.sc.built[s], DesignPairs());
    if (arm >= 0) {
      const ArmHandle& h = *arms_[arm].s->h;
      for (int g : h.link_geoms) obstacles.push_back({g, 0});
      for (int g : h.carriage_geoms) obstacles.push_back({g, 0});
    }
    for (Obstacle& o : obstacles) {
      if (joined.count(o.geom)) o.skip |= kProbe;
    }
    const Nearest n = scene_->LeastDistance(d, moving, obstacles);
    if (n.distance < worst.distance) {
      worst = n;
      scene_index = static_cast<int>(s);
    }
  }
  *which = job.sc.built.size() > 1 && scene_index == 1
               ? "with the next module on"
               : "";
  // Put the probe away.
  for (int i = 0; i < 3; ++i) d->mocap_pos[3 * H.probe_mocap + i] = 0.0;
  d->mocap_pos[3 * H.probe_mocap + 2] = -10.0;
  return worst;
}

std::vector<CheckRow> Checker::ClearanceRows() {
  std::vector<CheckRow> rows;
  for (const std::string& kind : WorkKinds()) {
    auto it = jobs_.find(kind);
    if (it == jobs_.end()) continue;
    for (Job& job : it->second) {
      if (!job.in_scope) continue;
      const PointResult& r = job.result;
      const std::string arm = job.sc.arm >= 0 ? Use(job.sc.arm).name : "";
      if (!r.reached) {
        rows.push_back(Row("clearance", job.point.item, arm, job.sc.tool, 0.0,
                           "m", opt_.clearance_m, false,
                           "no pose: " + (r.why.empty() ? "not solved" : r.why)));
      } else {
        std::string detail = scene_->PairText(r.nearest);
        if (!r.scene_note.empty() && job.sc.built.size() > 1) {
          detail += "; " + r.scene_note;
        }
        rows.push_back(Row("clearance", job.point.item, arm, job.sc.tool,
                           r.nearest.distance, "m", opt_.clearance_m,
                           r.nearest.distance >= opt_.clearance_m, detail));
      }
    }
    if (kind == "strap_bolt" || kind == "pin" || kind == "locate") {
      for (Job& job : it->second) {
        if (!job.in_scope) continue;
        std::string which;
        const Nearest n = FreeSpace(job, &which);
        std::string detail =
            n.distance < 0.0 ? which : scene_->PairText(n);
        if (n.distance >= 0.0 && !which.empty()) detail += "; " + which;
        if (!job.result.reached) detail += "; the arm at its wait pose (no pose)";
        const std::string name =
            kind == "strap_bolt"
                ? Fmt("free space bolt %d", job.point.index)
                : Fmt("free space %s %d/%d", kind.c_str(), job.point.module,
                      job.point.hole);
        rows.push_back(Row("clearance", name,
                           job.sc.arm >= 0 ? Use(job.sc.arm).name : "",
                           job.sc.tool, n.distance, "m", 0.0, n.distance > 0.0,
                           detail));
      }
    }
    // The tool's own boxes within the free space's radius: one row for each
    // arm and tool the kind is given to.
    if (kind == "strap_bolt" || kind == "pin" || kind == "swage" ||
        kind == "locate") {
      std::set<std::pair<std::string, std::string>> seen;
      for (const Work& w : L_.work) {
        if (w.kind != kind || !seen.insert({w.arm, w.tool}).second) continue;
        const ArmUse* arm = FindArm(L_, w.arm);
        const Tool* tool = arm != nullptr ? FindTool(*arm, w.tool) : nullptr;
        if (tool != nullptr) rows.push_back(ToolInFreeSpace(kind, *arm, *tool));
      }
    }
  }
  return rows;
}

CheckRow Checker::ToolInFreeSpace(const std::string& kind, const ArmUse& arm,
                                  const Tool& tool) const {
  double length = 0.0, radius = 0.0;
  if (kind == "strap_bolt") {
    length = kBoltFreeLength;
    radius = bay::kStrapBoltFreeRadius;
  } else if (kind == "locate") {
    length = bay::kLocateFreeLengthBelow;
    radius = bay::kLocateFreeRadiusBelow;
  } else {
    length = bay::kPinToolLength;
    radius = bay::kPinFreeRadius;
  }
  // The free space: from the working point (z = tool.length) back along the
  // tool's z for `length`.
  const double z_lo = tool.length - length, z_hi = tool.length;
  double worst = radius;
  std::string worst_box = "no box within it";
  for (const Box& b : tool.boxes) {
    const double lo = b.pos[2] - b.size[2] / 2.0;
    const double hi = b.pos[2] + b.size[2] / 2.0;
    if (hi <= z_lo + 1e-12 || lo >= z_hi - 1e-12) continue;
    const double across = std::hypot(std::abs(b.pos[0]) + b.size[0] / 2.0,
                                     std::abs(b.pos[1]) + b.size[1] / 2.0);
    if (radius - across < worst) {
      worst = radius - across;
      worst_box = Fmt("box %s reaches %.4f from the axis", b.name.c_str(),
                      across);
    }
  }
  return Row("clearance", "tool in free space " + kind + " " + tool.name,
             arm.name, tool.name, worst, "m", 0.0, worst >= -1e-12,
             Fmt("free radius %.3f over the %.3f behind the working point; %s",
                 radius, length, worst_box.c_str()));
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
    std::set<int> base(h.base_geoms.begin(), h.base_geoms.end());
    auto name = [&](int g) -> std::string {
      if (g < 0) return "nothing";
      const char* n = mj_id2name(model.get(), mjOBJ_GEOM, g);
      if (n != nullptr && n[0] != '\0') return n;
      const char* body = mj_id2name(model.get(), mjOBJ_BODY, model->geom_bodyid[g]);
      return std::string(body != nullptr ? body : "body") + "/geom" +
             std::to_string(g);
    };
    double fx = 1e9, fy = 1e9, ftop = 1e9, fbot = 1e9;
    int gx = -1, gy = -1, gtop = -1, gbot = -1;
    for (int g = 0; g < model->ngeom; ++g) {
      if (base.count(g)) continue;
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
                         Fmt("worst %s, %s", name(g).c_str(), what)));
    };
    row("stowed x", fx, gx, Fmt("inside +/-%.3f", ehi.x()).c_str());
    row("stowed y", fy, gy, Fmt("inside +/-%.3f", ehi.y()).c_str());
    row("stowed top", ftop, gtop, Fmt("under %.3f", ehi.z()).c_str());
    row("stowed bottom", fbot, gbot, Fmt("above %.3f", elo.z()).c_str());
  }
  // Where each arm is stowed must be on its track.
  for (const ArmUse& a : L_.arms) {
    if (!a.track) continue;
    const double in = InsideTrack(*a.track, a.stow_track);
    rows.push_back(Row("transport", "stow track " + a.name, a.name, "", in,
                       "m", 0.0, in >= -1e-9,
                       Fmt("stow_track %.3f on the track's %.3f..%.3f",
                           a.stow_track, a.track->lo, a.track->hi)));
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
        scene_ && mj_name2id(&M(), mjOBJ_SITE, site.c_str()) >= 0;
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
        const std::string held = HeldAtWork(w.kind);
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
      bool pass = c != nullptr && total <= payload;
      std::string detail = Fmt("tool %.2f + %s %.2f; rated payload of %s",
                               t.mass_kg, what.c_str(), part,
                               c != nullptr ? c->maker.c_str() : "?");
      if (OnFlatRacks(L_) && c != nullptr) {
        // The load's centre of mass in the flange's frame: the tool's mass
        // at the volume-weighted centre of its boxes (ours: uniform
        // density), the part's at the working point.
        Vec centre = Vec::Zero();
        double volume = 0.0;
        for (const Box& b : t.boxes) {
          const double v = b.size[0] * b.size[1] * b.size[2];
          centre += v * V(b.pos);
          volume += v;
        }
        if (volume > 0.0) centre /= volume;
        const Vec com =
            total > 0.0
                ? Vec((t.mass_kg * centre + part * Vec(0.0, 0.0, t.length)) /
                      total)
                : Vec::Zero();
        const double axial = std::abs(com.z());
        const double lateral = std::hypot(com.x(), com.y());
        const double ra = c->payload_centre_axial_m;
        const double rl = c->payload_centre_lateral_m;
        if (ra > 0.0) pass = pass && axial <= ra + 1e-12;
        if (rl > 0.0) pass = pass && lateral <= rl + 1e-12;
        detail += Fmt("; centre of mass %.3f m along the flange's axis "
                      "(rated to %s), %.3f m across (rated to %s)",
                      axial, ra > 0.0 ? Fmt("%.3f", ra).c_str() : "none stated",
                      lateral,
                      rl > 0.0 ? Fmt("%.3f", rl).c_str() : "none stated");
      }
      rows.push_back(Row("load", a.name + " " + t.name, a.name, t.name, total,
                         "kg", payload, pass, detail));
    }
  }
  return rows;
}

std::vector<CheckRow> Checker::WaysRows() {
  std::vector<CheckRow> rows;
  const mjModel& m = M();
  const LayoutHandles& H = this->H();
  mjData* d = data_.get();
  AllAtWait();
  if (H.probe_mocap >= 0) d->mocap_pos[3 * H.probe_mocap + 2] = -10.0;
  mj_kinematics(&m, d);
  // What moves and what is there along each way: the scene's. In staged
  // work the tube comes in at the first stage and the bay leaves at the
  // last.
  const StageScope* st = opt_.stage ? &*opt_.stage : nullptr;
  const double c = opt_.clearance_m, step = opt_.sweep_step_m;
  if (st == nullptr || st->first) {
    rows.push_back(WayRow(*scene_, d, "tube in", L_, WayKind::kTubeIn,
                          L_.tube_in.path, st != nullptr, c, step));
  }
  if (st != nullptr) {
    rows.push_back(TubeAtRestRow(*scene_, d, L_, *st, StageEnd(), c, step));
  }
  if (st != nullptr && !st->last) {
    rows.push_back(TransferRow(*scene_, d, *st, StageEnd(), c, step));
  }
  if (st == nullptr || st->last) {
    rows.push_back(WayRow(*scene_, d, "bay out", L_, WayKind::kBayOut,
                          L_.bay_out.path, st != nullptr, c, step));
  }
  mj_kinematics(&m, d);
  return rows;
}

double Checker::MoveTime(int arm, const Pose& from, const Pose& to) const {
  return ModelMoveSeconds(*scene_, arm, from, to);
}

using Piece = TimePiece;

// A phase's figures.
struct Phase {
  double time = 0.0;
  double lead_sum = 0.0;    // Every arm's leading pieces.
  double follow_sum = 0.0;  // Every arm's following pieces.
  int arms = 0;             // Arms that have a piece.
  int lead_arms = 0;
  int follow_arms = 0;
};

// The time model of one phase of `units` units (see TimeModelText).
Phase PhaseTime(const std::vector<Piece>& pieces, int units, int arm_count) {
  Phase ph;
  // Each arm's time on each unit, its leading and its following pieces
  // apart; a magazine is added after the unit's other pieces.
  std::vector<std::vector<double>> lead(arm_count, std::vector<double>(units)),
      follow(arm_count, std::vector<double>(units)),
      magazine(arm_count, std::vector<double>(units));
  std::vector<bool> has_lead(arm_count), has_follow(arm_count);
  for (const Piece& p : pieces) {
    if (p.arm < 0) continue;
    if (p.lead) {
      lead[p.arm][p.unit] += p.t;
      has_lead[p.arm] = true;
    } else if (p.magazine) {
      magazine[p.arm][p.unit] += p.t;
      has_follow[p.arm] = true;
    } else {
      follow[p.arm][p.unit] += p.t;
      has_follow[p.arm] = true;
    }
  }
  int lead_arm = -1, follow_arm = -1;
  for (int a = 0; a < arm_count; ++a) {
    for (int k = 0; k < units; ++k) follow[a][k] += magazine[a][k];
    ph.arms += has_lead[a] || has_follow[a] ? 1 : 0;
    if (has_lead[a]) {
      ++ph.lead_arms;
      lead_arm = a;
    }
    if (has_follow[a]) {
      ++ph.follow_arms;
      follow_arm = a;
    }
  }
  for (int k = 0; k < units; ++k) {
    for (int a = 0; a < arm_count; ++a) ph.lead_sum += lead[a][k];
  }
  for (int k = 0; k < units; ++k) {
    for (int a = 0; a < arm_count; ++a) ph.follow_sum += follow[a][k];
  }
  if (ph.arms <= 1) {
    // One arm does it all: the plain sum.
    ph.time = ph.lead_sum + ph.follow_sum;
  } else if (ph.arms == 2 && ph.lead_arms == 1 && ph.follow_arms == 1 &&
             lead_arm != follow_arm) {
    ph.time = SideBySide(lead[lead_arm], follow[follow_arm]);
  } else {
    // Each arm does its pieces in the order of the steps; a following piece
    // on unit k starts when every leading piece of unit k has ended, and a
    // set of unit k when every leading piece of unit k-1 has ended.
    std::vector<double> free(arm_count, 0.0), lead_end(units, 0.0);
    for (const Piece& p : pieces) {
      if (p.arm < 0) continue;
      double start = free[p.arm];
      if (!p.lead) start = std::max(start, lead_end[p.unit]);
      if (p.set && p.unit > 0) start = std::max(start, lead_end[p.unit - 1]);
      const double end = start + p.t;
      free[p.arm] = end;
      if (p.lead) lead_end[p.unit] = std::max(lead_end[p.unit], end);
    }
    for (double f : free) ph.time = std::max(ph.time, f);
  }
  return ph;
}

}  // namespace

double ModelMoveSeconds(const LayoutScene& scene, int arm, const ArmPose& from,
                        const ArmPose& to) {
  const SceneArm& a = scene.arm(arm);
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

double PhaseEnd(const std::vector<TimePiece>& pieces, int units,
                int arm_count) {
  return PhaseTime(pieces, units, arm_count).time;
}

namespace {

std::string ArmsNote(const Phase& ph) {
  if (ph.arms == 1) return ", one arm";
  if (ph.arms > 2) return Fmt(", %d arms", ph.arms);
  return "";
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
    std::vector<int> magazines;  // Its pins_take points not yet taken.
    Q6 posture{};                // Its last pose and what it was.
    std::string posture_at = "wait";
  };
  const int arm_count = static_cast<int>(arms_.size());
  std::vector<ArmNow> now(arm_count);
  for (int i = 0; i < arm_count; ++i) {
    now[i].pose = {arms_[i].s->wait, arms_[i].s->wait_track};
    now[i].tool = Use(i).first_tool;
    now[i].posture = arms_[i].s->wait;
  }
  const bool postures = OnFlatRacks(L_);
  if (postures) SolveToolChanges();
  postures_.clear();
  for (int i = 0; i < arm_count; ++i) postures_.push_back({Use(i).name});
  auto posture = [&](int ai, const Q6& q, const std::string& at) {
    if (!postures) return;
    ArmNow& n = now[ai];
    const double sec = PostureSeconds(ai, n.posture, q);
    PostureChanges& p = postures_[ai];
    if (sec > p.largest_s) {
      p.largest_s = sec;
      p.from = n.posture_at;
      p.to = at;
    }
    if (sec > 2.0) ++p.above_2s;
    n.posture = q;
    n.posture_at = at;
  };
  auto tool_change_pose = [&](int ai) -> const ToolChange* {
    for (const ToolChange& tc : tool_changes_) {
      if (tc.arm == ai && tc.reached) return &tc;
    }
    return nullptr;
  };
  // In staged work only the stage's steps (CheckOptions::stage).
  const StageScope* st = opt_.stage ? &*opt_.stage : nullptr;
  for (int i = 0; i < PointCount(L_, "pins_take"); ++i) {
    if (!InScope("pins_take", i)) continue;
    const Work* w = FindWork(L_, "pins_take", i);
    const int ai = w != nullptr ? scene_->ArmIndex(w->arm) : -1;
    if (ai >= 0) now[ai].magazines.push_back(i);
  }
  bool all_poses = true;
  std::vector<std::string> missing;
  std::string phase_now = "tube in";
  auto push = [&](const std::string& what, const std::string& arm,
                  const std::string& tool, const std::string& part,
                  const std::string& from, const std::string& to,
                  double t, const std::string& kind, int index = -1,
                  const std::string& tool_after = "") {
    BayStep s;
    s.kind = kind;
    s.index = index;
    s.phase = phase_now;
    s.tool_after = tool_after;
    if (st != nullptr) s.stage = st->name;
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
  // One working step of an arm; returns its time and sets *arm to the arm.
  auto work = [&](const std::string& kind, int index, const std::string& what,
                  const std::string& part, int* arm) -> double {
    *arm = -1;
    const Work* w = FindWork(L_, kind, index);
    if (w == nullptr) {
      all_poses = false;
      missing.push_back(kind + ": no work");
      return 0.0;
    }
    const int ai = scene_->ArmIndex(w->arm);
    if (ai < 0) {
      all_poses = false;
      missing.push_back(kind + ": no arm " + w->arm);
      return 0.0;
    }
    *arm = ai;
    ArmNow& n = now[ai];
    double t = 0.0;
    if (n.tool != w->tool) {
      const double tc = Use(ai).tool_change_s;
      t += push("tool change", w->arm, w->tool, "", n.tool, w->tool, tc,
                "tool_change", -1, w->tool);
      n.tool = w->tool;
      if (const ToolChange* pose = tool_change_pose(ai)) {
        posture(ai, pose->pose.q, "tool change");
      }
    }
    auto it = jobs_.find(kind);
    const Job* job = it != jobs_.end() &&
                             index < static_cast<int>(it->second.size())
                         ? &it->second[index]
                         : nullptr;
    double move = 0.0;
    std::string to = job != nullptr ? job->point.item : kind;
    if (job != nullptr && job->result.reached) {
      move = MoveTime(ai, n.pose, job->result.pose);
      n.pose = job->result.pose;
      posture(ai, job->result.pose.q, job->point.item);
    } else {
      all_poses = false;
      missing.push_back(to);
      to += " (no pose)";
    }
    t += push(what, w->arm, w->tool, part, n.at, to, move + w->process_s, kind,
              index);
    n.at = job != nullptr ? job->point.item : kind;
    n.busy += t;
    return t;
  };
  const int R = bay::kRailCount, M = bay::kModuleCount;
  double total = 0.0;
  const bool first = st == nullptr || st->first;
  const bool last = st == nullptr || st->last;
  const double tube_in =
      first ? push("tube in", "", "", "tube", "outside", "holder",
                   L_.tube_in.time_s, "tube_in")
            : 0.0;
  phase_now = "clamp";
  const double clamp = first ? push("clamp", "", "", "tube", "", "holder",
                                    L_.holder.clamp_s, "clamp")
                             : 0.0;
  phase_now = "rails";
  // The rail phase: each rail's take and set lead, its bolt follows.
  const bool each = L_.rail_order == "each_rail_then_its_bolt";
  std::vector<Piece> rail_pieces;
  auto piece = [&](std::vector<Piece>* list, const std::string& kind,
                   int index, int unit, bool lead, const std::string& what,
                   const std::string& part) {
    Piece p;
    p.unit = unit;
    p.lead = lead;
    p.set = kind == "rail_set" || kind == "module_set";
    p.t = work(kind, index, what, part, &p.arm);
    list->push_back(p);
  };
  auto bolt_rail = [&](int k, int unit) {
    piece(&rail_pieces, "strap_bolt", k, unit, false, Fmt("bolt rail %d", k),
          Fmt("strap bolt %d", k));
  };
  // The rails worked here, each a unit of the phase.
  std::vector<int> rails_here;
  for (int k = 0; k < R; ++k) {
    if (st == nullptr || Has(st->rails, k)) rails_here.push_back(k);
  }
  const int rail_units = static_cast<int>(rails_here.size());
  for (int u = 0; u < rail_units; ++u) {
    const int k = rails_here[u];
    piece(&rail_pieces, "rail_take", k, u, true, Fmt("take rail %d", k),
          Fmt("rail %d", k));
    piece(&rail_pieces, "rail_set", k, u, true, Fmt("set rail %d", k),
          Fmt("rail %d", k));
    if (each) bolt_rail(k, u);
  }
  if (!each) {
    for (int u = 0; u < rail_units; ++u) bolt_rail(rails_here[u], u);
  }
  const Phase rails = PhaseTime(rail_pieces, rail_units, arm_count);
  phase_now = "modules";
  // The module phase: each module's take and set lead, its holes follow;
  // an arm takes its magazine before its first hole.
  std::vector<Piece> module_pieces;
  auto hole = [&](const std::string& kind, int m, int unit, int i,
                  const std::string& what, const std::string& part) {
    const Work* w = FindWork(L_, kind, 4 * m + i);
    const int ai = w != nullptr ? scene_->ArmIndex(w->arm) : -1;
    if (ai >= 0) {
      for (int point : now[ai].magazines) {
        Piece p;
        p.unit = unit;
        p.magazine = true;
        p.t = work("pins_take", point, "take a pin magazine", "pin magazine",
                   &p.arm);
        module_pieces.push_back(p);
      }
      now[ai].magazines.clear();
    }
    piece(&module_pieces, kind, 4 * m + i, unit, false, what, part);
  };
  // In the layout's order of modules (src/layout.h, WorkOrder): the k-th
  // take is pallet module k, set as bay module order[k]; the time model's
  // unit is the place k in that order.
  // In staged work the order is the stage's (src/layout_stages.h) and the
  // unit the place among the stage's modules.
  const std::vector<int> order = ModuleOrder(L_);
  std::vector<int> takes_here;
  for (int k = 0; k < M; ++k) {
    if (st == nullptr || Has(st->modules, order[k])) takes_here.push_back(k);
  }
  const int module_units = static_cast<int>(takes_here.size());
  for (int u = 0; u < module_units; ++u) {
    const int k = takes_here[u];
    const int m = order[k];
    piece(&module_pieces, "module_take", k, u, true, Fmt("take module %d", m),
          Fmt("pallet module %d", k));
    piece(&module_pieces, "module_set", m, u, true, Fmt("set module %d", m),
          Fmt("module %d", m));
    // An arm that has a magazine but no hole takes it after the first set.
    if (u == 0) {
      const std::vector<std::string> holers =
          ArmsDoing(L_, {"locate", "pin", "swage"});
      for (int ai = 0; ai < arm_count; ++ai) {
        if (std::find(holers.begin(), holers.end(), Use(ai).name) !=
            holers.end()) {
          continue;
        }
        for (int point : now[ai].magazines) {
          Piece p;
          p.unit = 0;
          p.magazine = true;
          p.t = work("pins_take", point, "take a pin magazine", "pin magazine",
                     &p.arm);
          module_pieces.push_back(p);
        }
        now[ai].magazines.clear();
      }
    }
    for (int i : bay::kLocateOrder) {
      hole("locate", m, u, i, Fmt("locate %d/%d", m, i),
           Fmt("module %d hole %d", m, i));
      hole("pin", m, u, i, Fmt("pin %d/%d", m, i), Fmt("pin %d/%d", m, i));
    }
    for (int i : bay::kSwageOrder) {
      hole("swage", m, u, i, Fmt("swage %d/%d", m, i),
           Fmt("collar %d/%d", m, i));
    }
  }
  const Phase modules = PhaseTime(module_pieces, module_units, arm_count);
  phase_now = "end";
  // The end.
  const double release = last ? push("release", "", "", "bay", "holder", "",
                                     L_.holder.release_s, "release")
                              : 0.0;
  const double out = last ? push("bay out", "", "", "bay", "holder",
                                 "outside", L_.bay_out.time_s, "bay_out")
                          : 0.0;
  double back = 0.0;
  for (int i = 0; i < arm_count; ++i) {
    ArmNow& n = now[i];
    const ArmUse& use = Use(i);
    double t = 0.0;
    const Pose wait{arms_[i].s->wait, arms_[i].s->wait_track};
    const double move = MoveTime(i, n.pose, wait);
    if (move > 0.0) {
      t += push("back to wait", use.name, n.tool, "", n.at, "wait", move,
                "to_wait");
    }
    if (n.tool != use.first_tool) {
      if (const ToolChange* pose = tool_change_pose(i)) {
        posture(i, pose->pose.q, "tool change");
      }
      t += push("tool change", use.name, use.first_tool, "", n.tool,
                use.first_tool, use.tool_change_s, "tool_change", -1,
                use.first_tool);
    }
    posture(i, arms_[i].s->wait, "wait");
    n.busy += t;
    back = std::max(back, t);
  }
  const double end = std::max(release + out, back);
  total = tube_in + clamp + rails.time + modules.time + end;
  phases_ = {{"tube in", tube_in},
             {"clamp", clamp},
             {"rails", rails.time},
             {"modules", modules.time},
             {"end", end}};
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
              "bolting %.1f%s) + modules %.1f (module arm%s %.1f, pin arm%s "
              "%.1f%s) + end %.1f",
              tube_in, clamp, rails.time, rails.lead_sum, rails.follow_sum,
              ArmsNote(rails).c_str(), modules.time,
              modules.lead_arms > 1 ? "s" : "", modules.lead_sum,
              modules.follow_arms > 1 ? "s" : "", modules.follow_sum,
              ArmsNote(modules).c_str(), end)));
  for (int i = 0; i < arm_count; ++i) {
    rows.push_back(Row("time", "busy " + Use(i).name, Use(i).name, "",
                       now[i].busy, "s", 0.0, all_poses,
                       "the sum of its steps"));
  }
  return rows;
}

std::vector<CheckRow> Checker::ApproachRows() {
  std::vector<CheckRow> rows;
  for (const std::string& kind : WorkKinds()) {
    auto it = jobs_.find(kind);
    if (it == jobs_.end() ||
        std::none_of(it->second.begin(), it->second.end(),
                     [](const Job& j) { return j.in_scope; })) {
      continue;
    }
    if (FindApproach(L_, kind) == nullptr) {
      rows.push_back(Row("approach", kind, "", "", 0.0, "m", opt_.clearance_m,
                         false, "the layout gives no approach for " + kind));
      continue;
    }
    for (const Job& job : it->second) {
      if (!job.in_scope) continue;
      const std::string arm = job.sc.arm >= 0 ? Use(job.sc.arm).name : "";
      for (int leg = 0; leg < 2; ++leg) {
        const LegResult& r = job.legs[leg];
        const char* name = leg == 0 ? "approach" : "retreat";
        const std::string item = job.point.item + " " + name;
        if (!r.walked || !r.why.empty()) {
          rows.push_back(Row("approach", item, arm, job.sc.tool, 0.0, "m",
                             opt_.clearance_m, false,
                             r.walked ? r.why : "not walked"));
          continue;
        }
        std::string where =
            r.at_far ? Fmt("at the %s pose", name)
                     : Fmt("at %.1f mm along", r.at_m * 1000.0);
        std::string pair = scene_->PairText(r.nearest);
        if (r.by_design && r.at_far && !r.nearest.near) {
          pair += " (by design at the work point)";
        }
        rows.push_back(Row(
            "approach", item, arm, job.sc.tool, r.figure, "m",
            opt_.clearance_m, r.reached && r.figure >= opt_.clearance_m,
            Fmt("%s %s; least joint margin %.1f deg at %s; %d samples over "
                "%.1f mm",
                pair.c_str(), where.c_str(), r.margin_deg,
                r.margin_joint.c_str(), r.samples, r.length * 1000.0)));
      }
    }
  }
  return rows;
}

// The tool-change pose of each arm with more than one tool (see the header).
void Checker::SolveToolChanges() {
  if (tool_changes_solved_) return;
  tool_changes_solved_ = true;
  const mjModel& m = M();
  mjData* d = data_.get();
  for (int ai = 0; ai < static_cast<int>(arms_.size()); ++ai) {
    const ArmUse& use = Use(ai);
    if (use.tools.size() < 2) continue;
    ToolChange tc;
    tc.arm = ai;
    const SceneArm& sa = *arms_[ai].s;
    // Over the middle of the waiting places, kToolChangeHeight above the
    // highest top, in the arm's base frame.
    Vec lo = Vec::Constant(1e9), hi = Vec::Constant(-1e9);
    for (const Tool& t : use.tools) {
      const Vec c = V(t.waits_at), h = V(t.wait_size) / 2.0;
      lo = lo.cwiseMin(c - h);
      hi = hi.cwiseMax(c + h);
    }
    const Vec local((lo.x() + hi.x()) / 2.0, (lo.y() + hi.y()) / 2.0,
                    hi.z() + kToolChangeHeight);
    AllAtWait();
    mj_kinematics(&m, d);
    const std::string frame = use.name + (use.track ? "_carriage" : "_mount");
    const int body = mj_name2id(&m, mjOBJ_BODY, frame.c_str());
    if (body < 0 || sa.h->flange_site < 0) {
      tc.why = "the model has no body " + frame + " or no flange site";
      tool_changes_.push_back(tc);
      continue;
    }
    Eigen::Matrix3d r;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) r(i, j) = d->xmat[9 * body + 3 * i + j];
    }
    tc.flange = Vec(d->xpos[3 * body], d->xpos[3 * body + 1],
                    d->xpos[3 * body + 2]) +
                r * local;
    Target t;
    t.p = tc.flange;
    t.z = -Vec::UnitZ();
    std::vector<Q6> seeds = arms_[ai].seeds;  // The wait pose first.
    const Built built = StageEnd();
    bool have = false;
    double best_perr = 1e9;
    // Any spin: the solver's own, then eight fixed ones.
    std::vector<Target> targets = {t};
    for (int k = 0; k < 8; ++k) {
      Target ts = t;
      ts.has_x = true;
      const double a = k * std::numbers::pi / 4.0;
      ts.x = Vec(std::cos(a), std::sin(a), 0.0);
      targets.push_back(ts);
    }
    // The worst over its tools on the flange, the arm at q; true if clear.
    auto evaluate = [&](const Q6& q) {
      AllAtWait();
      SetPose(ai, {q, sa.wait_track});
      mj_kinematics(&m, d);
      Nearest worst;
      worst.distance = kFar + 1.0;
      std::string worst_tool;
      for (const Tool& tool : use.tools) {
        const Nearest n = scene_->LeastDistance(
            d, scene_->Moving(ai, tool.name, ""),
            scene_->Obstacles(ai, tool.name, built, DesignPairs()));
        if (n.distance < worst.distance) {
          worst = n;
          worst_tool = tool.name;
        }
      }
      if (!have || worst.distance > tc.nearest.distance) {
        have = true;
        tc.reached = true;
        tc.pose = {q, sa.wait_track};
        tc.nearest = worst;
        tc.worst_tool = worst_tool;
      }
      return worst.distance >= opt_.clearance_m;
    };
    if (!L_.approaches.empty()) {
      // Every solution, its windings nearest the wait pose, nearest first.
      const Q6 anchor = sa.wait;
      std::vector<std::pair<double, Q6>> found;
      for (const Target& tg : targets) {
        for (const Q6& seed : seeds) {
          AllAtWait();
          const IkResult ik =
              Solve(ai, sa.h->flange_site, tg, sa.wait_track, seed);
          if (!ik.ok) {
            best_perr = std::min(best_perr, ik.perr);
            continue;
          }
          const Q6 q = Unwind(ai, ik.q, anchor);
          bool seen = false;
          for (const auto& f : found) {
            bool same = true;
            for (int j = 0; j < 6; ++j) {
              same = same && std::abs(f.second[j] - q[j]) < 0.5 * kDeg;
            }
            seen = seen || same;
          }
          if (!seen) found.push_back({PostureSeconds(ai, anchor, q), q});
        }
      }
      std::stable_sort(found.begin(), found.end(),
                       [](const auto& x, const auto& y) {
                         return x.first < y.first;
                       });
      for (const auto& f : found) {
        double margin;
        std::string joint;
        Margin(ai, f.second, &margin, &joint);
        if (margin < opt_.joint_margin_deg - 1e-9) continue;
        if (evaluate(f.second)) break;
      }
    } else {
      bool clear = false;
      for (const Target& tg : targets) {
        if (clear) break;
        for (const Q6& seed : seeds) {
          AllAtWait();
          const IkResult ik =
              Solve(ai, sa.h->flange_site, tg, sa.wait_track, seed);
          if (!ik.ok) {
            best_perr = std::min(best_perr, ik.perr);
            continue;
          }
          if (evaluate(ik.q)) {  // Clear: it wins.
            clear = true;
            break;
          }
        }
      }
    }
    if (have) {
      Margin(ai, tc.pose.q, &tc.margin_deg, &tc.margin_joint);
    } else {
      tc.why = Fmt("not reached: %.3f m short at best", best_perr);
    }
    tool_changes_.push_back(tc);
  }
  AllAtWait();
  mj_kinematics(&m, d);
}

std::vector<CheckRow> Checker::WaitPoseRows(const std::string& check) {
  std::vector<CheckRow> rows;
  if (!OnFlatRacks(L_)) return rows;
  const mjModel& m = M();
  mjData* d = data_.get();
  const Built bare = StageStart();
  const Built full = StageEnd();
  for (int ai = 0; ai < static_cast<int>(arms_.size()); ++ai) {
    const ArmUse& use = Use(ai);
    const SceneArm& sa = *arms_[ai].s;
    if (check == "reach") {
      double margin;
      std::string joint;
      Margin(ai, sa.wait, &margin, &joint);
      rows.push_back(Row("reach", "wait pose " + use.name, use.name, "",
                         margin, "deg", opt_.joint_margin_deg,
                         margin >= opt_.joint_margin_deg - 1e-9,
                         "least margin at " + joint));
      continue;
    }
    AllAtWait();
    mj_kinematics(&m, d);
    const auto moving = scene_->Moving(ai, use.first_tool, "");
    const Nearest a = scene_->LeastDistance(
        d, moving, scene_->Obstacles(ai, use.first_tool, bare, DesignPairs()));
    const Nearest b = scene_->LeastDistance(
        d, moving, scene_->Obstacles(ai, use.first_tool, full, DesignPairs()));
    const Nearest& worst = b.distance < a.distance ? b : a;
    rows.push_back(Row("clearance", "wait pose " + use.name, use.name,
                       use.first_tool, worst.distance, "m", opt_.clearance_m,
                       worst.distance >= opt_.clearance_m,
                       scene_->PairText(worst) +
                           (b.distance < a.distance
                                ? (opt_.stage ? "; worse at the stage's end"
                                              : "; worse with all rails and "
                                                "modules on")
                                : (opt_.stage ? "; worse at the stage's start"
                                              : "; worse with the bare tube"))));
  }
  return rows;
}

std::vector<SolvedToolChange> Checker::ToolChanges() {
  SolveToolChanges();
  std::vector<SolvedToolChange> out;
  for (const ToolChange& tc : tool_changes_) {
    SolvedToolChange s;
    s.arm = Use(tc.arm).name;
    s.reached = tc.reached;
    s.deg = ToDeg(tc.pose.q);
    out.push_back(s);
  }
  return out;
}

std::vector<CheckRow> Checker::ToolChangeRows(const std::string& check) {
  std::vector<CheckRow> rows;
  if (!OnFlatRacks(L_)) return rows;
  SolveToolChanges();
  for (const ToolChange& tc : tool_changes_) {
    const std::string arm = Use(tc.arm).name;
    const std::string where =
        Fmt("flange at (%.3f, %.3f, %.3f), z down, track %.3f", tc.flange.x(),
            tc.flange.y(), tc.flange.z(), arms_[tc.arm].s->wait_track);
    if (check == "reach") {
      rows.push_back(Row("reach", "tools " + arm, arm, "",
                         tc.reached ? tc.margin_deg : 0.0, "deg",
                         opt_.joint_margin_deg,
                         tc.reached &&
                             tc.margin_deg >= opt_.joint_margin_deg - 1e-9,
                         tc.reached ? Fmt("least margin at %s; ",
                                          tc.margin_joint.c_str()) +
                                          where
                                    : tc.why + "; " + where));
    } else {
      rows.push_back(Row(
          "clearance", "tools " + arm, arm, tc.worst_tool,
          tc.reached ? tc.nearest.distance : 0.0, "m", opt_.clearance_m,
          tc.reached && tc.nearest.distance >= opt_.clearance_m,
          tc.reached ? scene_->PairText(tc.nearest) + "; worst with " +
                           tc.worst_tool + " on the flange; " +
                           (opt_.stage ? "the bay as at the stage's end"
                                       : "all rails and modules on")
                     : "no pose: " + tc.why));
    }
  }
  return rows;
}

std::vector<SolvedPoint> Checker::Points() const {
  std::vector<SolvedPoint> out;
  if (!solved_) return out;
  for (const std::string& kind : WorkKinds()) {
    auto it = jobs_.find(kind);
    if (it == jobs_.end()) continue;
    for (const Job& job : it->second) {
      if (!job.in_scope) continue;
      SolvedPoint p;
      p.kind = kind;
      if (opt_.stage) p.stage = opt_.stage->name;
      p.index = job.point.index;
      p.item = job.point.item;
      p.arm = job.sc.arm >= 0 ? Use(job.sc.arm).name : "";
      p.tool = job.sc.tool;
      p.held_work = job.sc.held;
      p.held_approach = job.legs[0].walked ? job.legs[0].held : job.sc.held;
      p.held_retreat = job.legs[1].walked ? job.legs[1].held : job.sc.held;
      if (!job.legs[0].walked && job.sc.error.empty()) {
        p.held_approach = At(job.point, Leg::kApproach).held;
        p.held_retreat = At(job.point, Leg::kRetreat).held;
      }
      p.track = job.result.pose.track;
      p.work_reached = job.result.reached;
      p.work_deg = ToDeg(job.result.pose.q);
      const LegResult& a = job.legs[0];
      const LegResult& r = job.legs[1];
      p.approach_reached = a.walked && a.why.empty() && a.reached;
      p.retreat_reached = r.walked && r.why.empty() && r.reached;
      if (a.walked) p.approach_deg = ToDeg(a.far);
      if (r.walked) p.retreat_deg = ToDeg(r.far);
      if (p.approach_reached) {
        for (const Q6& q : a.path) p.approach_leg_deg.push_back(ToDeg(q));
      }
      if (p.retreat_reached) {
        for (const Q6& q : r.path) p.retreat_leg_deg.push_back(ToDeg(q));
      }
      if (const Approach* rule = FindApproach(L_, kind)) {
        p.approach_length_m = rule->approach_m;
        p.retreat_length_m = rule->retreat_m;
      }
      out.push_back(p);
    }
  }
  return out;
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
  const Built& b = job.sc.built.front();
  s.rails = static_cast<int>(b.rails.size());
  s.modules = static_cast<int>(b.modules.size());
  s.pallet_modules =
      kind == "module_take" ? job.point.pallet_modules : b.on_pallets;
  const ArmUse& a = Use(job.sc.arm);
  s.tool[a.name] = job.sc.tool;
  s.held[a.name] =
      job.sc.held == "module" || job.sc.held == "rail" ? job.sc.held : "";
  s.joints_deg[a.name] = ToDeg(job.result.pose.q);
  if (a.track) s.track[a.name] = job.result.pose.track;
  *state = s;
  return true;
}

}  // namespace

// ---------------------------------------------------------------- public
const std::vector<std::string>& CheckKinds() {
  static const std::vector<std::string> kinds = {
      "transport", "hold", "load",    "reach", "clearance", "ways",
      "time",      "allowed", "port", "approach", "gap"};
  return kinds;
}

std::vector<WorkPoint> WorkPoints(const Layout& layout,
                                  const std::string& kind) {
  std::vector<WorkPoint> out;
  const std::vector<bay::Fastening> all = bay::Fastenings();
  const Vec down(0.0, 0.0, -1.0);
  if (kind == "rail_take" || kind == "pins_take") {
    std::vector<const Stock*> stocks;
    if (kind == "rail_take") {
      const Stock* s = FindStock(layout, "rails");
      for (int k = 0; s != nullptr && k < bay::kRailCount; ++k) {
        stocks.push_back(s);
      }
    } else {
      stocks = PinStocks(layout);
    }
    for (size_t k = 0; k < stocks.size(); ++k) {
      const Stock* s = stocks[k];
      WorkPoint p;
      p.kind = kind;
      p.index = static_cast<int>(k);
      p.item = Fmt("%s %d", kind.c_str(), p.index);
      p.pos = V(s->pick_pos);
      p.z = V(s->pick_axis).normalized();
      if (kind == "rail_take") p.rail = p.index;
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
    int index = 0;
    for (size_t pi = 0; pi < layout.pallets.size(); ++pi) {
      const Pallet& pallet = layout.pallets[pi];
      const double yaw = pallet.yaw_deg * kDeg;
      for (int j = 0; j < pallet.modules; ++j, ++index) {
        const int n = pallet.modules - j;
        // The pallet's frame: x along the modules' long side, y along the
        // stack, turned by yaw about z.
        const Vec px(std::cos(yaw), std::sin(yaw), 0.0);
        const Vec py(-std::sin(yaw), std::cos(yaw), 0.0);
        const double deck = pallet.pos[2] + pallet.base_height;
        WorkPoint p;
        p.kind = kind;
        p.index = index;
        p.item = Fmt("module_take %d", index);
        if (pallet.stands == "flat") {
          p.pos = V(pallet.pos) + Vec(0.0, 0.0, -pallet.pos[2]);
          p.pos.z() = deck + (pallet.modules - 1 - j) * pallet.pitch +
                      bay::kModuleThickness;
          p.z = down;
        } else {
          // On its long edge, glass toward -y: the centre of the glass face.
          const double y = (j - (pallet.modules - 1) / 2.0) * pallet.pitch -
                           bay::kModuleThickness / 2.0;
          p.pos = V(pallet.pos) + y * py;
          p.pos.z() = deck + bay::kModuleWidth / 2.0;
          p.z = py;
        }
        p.has_x = true;
        p.x = px;
        p.pallet = static_cast<int>(pi);
        p.pallet_modules = n;
        out.push_back(p);
      }
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
  // The follower starts unit k once the leader has finished it and it has
  // finished unit k-1: the end is the largest over k of the leader's units
  // 0..k and the follower's units k..end.
  const size_t n = std::min(lead.size(), follow.size());
  double best = 0.0, before = 0.0, after = 0.0;
  for (size_t k = 0; k < follow.size(); ++k) after += follow[k];
  for (size_t k = 0; k < n; ++k) {
    before += lead[k];
    best = std::max(best, before + after);
    after -= follow[k];
  }
  return best;
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
      "from the pose at one point to the pose at the next: the approaches "
      "and retreats are checked by the approach rows but not timed, and the "
      "paths between are not modelled or checked.\n"
      "A working step is its move from the arm's last pose and the layout's "
      "process time for the kind. A tool change takes the arm's "
      "tool_change_s; the move to where the tools wait is not counted. "
      "Every arm starts the bay at its wait pose with its first tool, and "
      "each step is done by the arm the layout's division of the kind's "
      "points gives it.\n"
      "The bay: tube in + clamp + the rail phase + the module phase + the "
      "end. A phase is a row of units (the ten rails; the nine modules). "
      "Each unit has a leading piece, its take and its set, by the arms "
      "that have them, and following pieces: a rail's strap bolt; a "
      "module's locating, pinning and swaging of its four holes, an arm's "
      "pin magazine counted with the module before whose holes it takes "
      "it. Each arm does its pieces in the order of the steps (the layout's "
      "rail_order for the rails); a following piece on unit k starts when "
      "every leading piece of unit k has ended and its arm has ended its "
      "pieces before it; a set of unit k starts when every leading piece of "
      "unit k-1 has ended (the rails and the modules go on in order); and "
      "the phase ends when the last arm ends. With "
      "one leading and one following arm this is the following arm one "
      "unit behind: the largest over k of (the leading arm's pieces on "
      "units 0..k + the following arm's on units k..end); when one arm "
      "does the whole phase, the plain sum. An arm's pieces include its "
      "tool changes. The time has no limit: its row passes when every step "
      "has a pose. The end: release + bay out, or the longest of the arms' "
      "moves back to their wait poses and first tools if that is longer.",
      kAccelFactor, kSettleS);
}

CheckReport CheckAtOnePlace(const Layout& layout, const ModelPaths& paths,
                            const CheckOptions& options) {
  // The stages run side by side (src/layout_stages.cc): models are loaded
  // one at a time.
  static std::mutex loading;
  CheckReport report;
  report.time_model = TimeModelText();
  // The margins: on the flat rack the larger of the options' and the
  // layout's.
  CheckOptions opt = options;
  if (OnFlatRacks(layout)) {
    opt.clearance_m = std::max(opt.clearance_m, layout.margins.clearance_m);
    opt.joint_margin_deg =
        std::max(opt.joint_margin_deg, layout.margins.joint_margin_deg);
  }
  report.clearance_m = opt.clearance_m;
  report.joint_margin_deg = opt.joint_margin_deg;
  Checker checker(layout, opt);
  std::string error;
  std::unique_lock<std::mutex> lock(loading);
  const bool ok = checker.Init(paths, &error);
  auto want = [&](const char* kind) { return Wanted(opt.checks, kind); };
  auto append = [&](const std::vector<CheckRow>& rows) {
    report.rows.insert(report.rows.end(), rows.begin(), rows.end());
  };
  if (want("transport")) {
    append(OnFlatRacks(layout) ? FlatRackTransportRows(layout, paths)
                               : checker.TransportRows(paths));
  }
  // PortRows loads models too; it comes later in the output.
  std::vector<CheckRow> ports;
  if (want("port")) ports = PortRows(layout, paths);
  lock.unlock();
  if (!ok) {
    report.error = error;
    for (const char* kind :
         {"hold", "load", "reach", "clearance", "ways", "time"}) {
      if (want(kind)) {
        report.rows.push_back(Row(kind, "model", "", "", 0.0, "", 0.0, false,
                                  "no model: " + error));
      }
    }
    if (want("allowed")) append(AllowedRows(layout));
    append(ports);
    if (want("approach")) {
      report.rows.push_back(Row("approach", "model", "", "", 0.0, "", 0.0,
                                false, "no model: " + error));
    }
    report.pass = false;
    return report;
  }
  if (want("hold")) append(checker.HoldRows());
  if (want("load")) append(checker.LoadRows());
  const bool time = want("time") && opt.work_kinds.empty();
  const bool poses =
      want("reach") || want("clearance") || time || want("approach");
  if (poses) checker.SolveAll();
  if (want("reach")) {
    append(checker.ReachRows());
    append(checker.ToolChangeRows("reach"));
    append(checker.WaitPoseRows("reach"));
  }
  if (want("clearance")) {
    append(checker.ClearanceRows());
    append(checker.ToolChangeRows("clearance"));
    append(checker.WaitPoseRows("clearance"));
  }
  if (want("ways")) append(checker.WaysRows());
  if (time) append(checker.TimeRows(&report.steps, &report.bay_time_s));
  if (want("allowed")) append(AllowedRows(layout));
  append(ports);
  if (want("approach")) append(checker.ApproachRows());
  if (poses) report.points = checker.Points();
  report.tool_changes = checker.ToolChanges();
  report.phases = checker.phases();
  report.postures = checker.postures();
  report.pass = !report.rows.empty();
  for (const CheckRow& r : report.rows) report.pass &= r.pass;
  return report;
}

bool StateAt(const Layout& layout, const ModelPaths& paths,
             const std::string& kind, int index, LayoutState* state,
             std::string* error) {
  CheckOptions options;
  options.checks = {"reach"};
  if (OnFlatRacks(layout)) {
    options.clearance_m =
        std::max(options.clearance_m, layout.margins.clearance_m);
    options.joint_margin_deg =
        std::max(options.joint_margin_deg, layout.margins.joint_margin_deg);
  }
  // Staged work: the point's stage, with the tube at its place.
  const int stage = layout.staging ? StageOfPoint(layout, kind, index) : -1;
  const Layout at = stage >= 0 ? StageLayout(layout, stage) : layout;
  if (stage >= 0) options.stage = ScopeOfStage(layout, stage);
  Checker checker(at, options);
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

// ------------------------------------------------- PairQuery's kinematics
// (src/layout_scene.h). Here, because they read poses out of an mjData the
// planner poses itself, as the checker does.

// mj_kinematics1's work for body i (MuJoCo 4.0.0,
// src/engine/engine_core_smooth.c, sleep off, a body with hinges and
// slides only, not a mocap body): the same steps through MuJoCo's public
// functions, which do the same arithmetic as the inline ones it uses there.
void PlaceBody(const mjModel& m, mjData* d, int i) {
  mjtNum xpos[3] = {}, xquat[4] = {};
  const int pid = m.body_parentid[i];
  const mjtNum* bodypos = m.body_pos + 3 * i;
  const mjtNum* bodyquat = m.body_quat + 4 * i;
  if (pid) {
    mju_mulMatVec3(xpos, d->xmat + 9 * pid, bodypos);
    mju_addTo3(xpos, d->xpos + 3 * pid);
    mju_mulQuat(xquat, d->xquat + 4 * pid, bodyquat);
  } else {
    mju_copy3(xpos, bodypos);
    mju_copy4(xquat, bodyquat);
  }
  for (int j = 0; j < m.body_jntnum[i]; ++j) {
    const int jid = m.body_jntadr[i] + j;
    const int qadr = m.jnt_qposadr[jid];
    mjtNum xanchor[3] = {}, xaxis[3] = {};
    mju_rotVecQuat(xaxis, m.jnt_axis + 3 * jid, xquat);
    mju_rotVecQuat(xanchor, m.jnt_pos + 3 * jid, xquat);
    mju_addTo3(xanchor, xpos);
    if (m.jnt_type[jid] == mjJNT_SLIDE) {
      mju_addToScl3(xpos, xaxis, d->qpos[qadr] - m.qpos0[qadr]);
    } else {
      mjtNum qloc[4] = {};
      mju_axisAngle2Quat(qloc, m.jnt_axis + 3 * jid, d->qpos[qadr] - m.qpos0[qadr]);
      mju_mulQuat(xquat, xquat, qloc);
      mjtNum vec[3] = {};
      mju_rotVecQuat(vec, m.jnt_pos + 3 * jid, xquat);
      mju_sub3(xpos, xanchor, vec);
    }
    mju_copy3(d->xanchor + 3 * jid, xanchor);
    mju_copy3(d->xaxis + 3 * jid, xaxis);
  }
  mju_normalize4(xquat);
  mju_copy4(d->xquat + 4 * i, xquat);
  mju_copy3(d->xpos + 3 * i, xpos);
  mju_quat2Mat(d->xmat + 9 * i, xquat);
}

// mj_local2GlobalQuat (engine_core_util.c), which mj_kinematics2 uses for
// inertial frames, geoms and sites: the position and matrix by the public
// mj_local2Global, which calls it, and the quaternion as it makes it.
void PlaceFrame(mjData* d, mjtNum* xpos, mjtNum* xquat, mjtNum* xmat,
                   const mjtNum* pos, const mjtNum* quat, int body,
                   mjtByte sameframe) {
  mj_local2Global(d, xpos, xmat, pos, quat, body, sameframe);
  switch (sameframe) {
    case mjSAMEFRAME_NONE:
      mju_mulQuat(xquat, d->xquat + 4 * body, quat);
      break;
    case mjSAMEFRAME_BODY:
    case mjSAMEFRAME_BODYROT:
      mju_copy4(xquat, d->xquat + 4 * body);
      break;
    default:
      mju_copy4(xquat, d->xiquat + 4 * body);
      break;
  }
}

// Whether PlaceBody does for body b what mj_kinematics1 does: hinges and
// slides only, no mocap, and no joint whose position is a zero with a sign
// (mju_rotVecQuat returns +0 for a zero vector, the inline one keeps -0).
bool Placeable(const mjModel& m, int b) {
  if (m.body_mocapid[b] >= 0) return false;
  for (int j = m.body_jntadr[b]; j < m.body_jntadr[b] + m.body_jntnum[b]; ++j) {
    if (m.jnt_type[j] != mjJNT_HINGE && m.jnt_type[j] != mjJNT_SLIDE) return false;
    for (int k = 0; k < 3; ++k) {
      if (m.jnt_pos[3 * j + k] == 0.0 && std::signbit(m.jnt_pos[3 * j + k])) return false;
    }
  }
  return true;
}

}  // namespace cell::layout
