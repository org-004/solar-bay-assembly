// R16: the time model of src/layout_check.h against this repository's own
// planner (src/rrt.h, src/path_timing.h) for a few of a layout's moves.
//
//   bazel run --config=opt //:print_layout_moves -- below
//
// One CSV row per move, consecutive steps of one bay (check_layout --steps):
// the time model's figure (JointMoveTime, TrackMoveTime, kSettleS of
// src/layout_check.h), whether the straight joint-space line between the two
// poses is free, RRT-Connect and ShortenPath on the layout's model, and the
// shortened path timed by Trajectory::Plan. Then # lines: the figures used and
// the summary. Fixed seeds: the same output every run.
//
// The test of a configuration. The model is the state at the START of the
// move (StateAt of the move's first point; for an empty return the held part
// is put down on the bay), the other arm at its wait pose. A configuration
// (six joints in radians, and the track in metres as a seventh coordinate
// when the two poses differ in it) is free when every geom of the moving set
// (the arm's links, its carriage, the tool on the flange, the held part) is
// at least kClearance from every other geom there. An end pose that is not
// free is backed off by 0.05, 0.075 or 0.10 m along the tool's -z, else
// straight up, else half and half (damped least squares on the six joints, the tool's orientation kept) and the plan
// starts or ends there; the straight back-off is a segment of the timed path
// that the planner did not check. Only if that does not free both ends are
// the pairs that touch by design at the move's two ends (src/layout_check.h's
// list, given per move below) left out, along the whole move, and the ends
// tried again. Leaving them out along the whole move would hide, say, a pin
// tool's nose passing through the rail it works under.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include "src/arm_catalog.h"
#include "src/joint_move.h"
#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/path_timing.h"
#include "src/rrt.h"

namespace {

using namespace cell::layout;
using cell::JointPath;

constexpr double kClearance = 0.010;     // m
constexpr double kToFullSpeedS = 0.3;    // ours: joints reach full speed in 0.3 s
constexpr double kJerkRiseS = 0.1;       // ours: acceleration builds in 0.1 s
constexpr double kMarginDeg = 5.0;       // as CheckOptions::joint_margin_deg
constexpr double kDeg = M_PI / 180.0;

// Moving-set groups, for leaving pairs out.
constexpr unsigned kLinks = 1, kCarriage = 2, kToolBody = 4, kToolLast = 8,
                   kHeld = 16;

struct MoveSpec {
  std::string from_kind;
  int from_index;
  std::string to_kind;
  int to_index;
  bool empty_return = false;  // module_set m -> module_take m+1, gripper empty.
};

std::string Num(double v, int digits = 2) {
  char b[64];
  std::snprintf(b, sizeof(b), "%.*f", digits, v);
  return b;
}

std::string Csv(const std::string& s) {
  if (s.find_first_of(",\"\n") == std::string::npos) return s;
  std::string out = "\"";
  for (char c : s) {
    if (c == '"') out += '"';
    out += c;
  }
  return out + "\"";
}

// One move's world: the model, its data, and the collision test.
class MoveWorld {
 public:
  MoveWorld(const mjModel* m, mjData* d) : m_(m), d_(d) {}

  void AddMoving(int g, unsigned group) { moving_.push_back({g, group}); }
  void AddObstacle(int g) { obstacles_.insert(g); }
  void RemoveObstacle(int g) { obstacles_.erase(g); }
  void Skip(int g, unsigned groups) { skip_[g] |= groups; }
  void Finish() {
    for (const auto& mv : moving_) obstacles_.erase(mv.first);
    obs_.assign(obstacles_.begin(), obstacles_.end());
  }

  // Configuration layout: six joints (rad), then the track (m) if used.
  std::array<int, 6> qpos{};
  int track_qpos = -1;
  bool use_track = false;
  double fixed_track = 0.0;

  void Set(const Eigen::VectorXd& c) {
    for (int j = 0; j < 6; ++j) d_->qpos[qpos[j]] = c[j];
    if (track_qpos >= 0) {
      d_->qpos[track_qpos] = use_track ? c[6] : fixed_track;
    }
    mj_kinematics(m_, d_);
  }

  // Least distance (capped at `cap`) and the pair, at the current data.
  double Least(double cap, int* gm, int* go) const {
    double best = cap;
    for (const auto& mv : moving_) {
      Eigen::Vector3d cm, hm;
      Box(mv.first, &cm, &hm);
      for (int h : obs_) {
        auto it = skip_.find(h);
        if (it != skip_.end() && (it->second & mv.second)) continue;
        Eigen::Vector3d co, ho;
        Box(h, &co, &ho);
        const Eigen::Vector3d gap =
            ((cm - co).cwiseAbs() - hm - ho).cwiseMax(0.0);
        if (gap.norm() >= best) continue;
        const double dist = mj_geomDistance(m_, d_, mv.first, h, best, nullptr);
        if (dist < best) {
          best = dist;
          if (gm) *gm = mv.first;
          if (go) *go = h;
        }
      }
    }
    return best;
  }

  bool Free(const Eigen::VectorXd& c) {
    Set(c);
    return Least(kClearance, nullptr, nullptr) >= kClearance;
  }

  std::string Why(const Eigen::VectorXd& c) {
    Set(c);
    int a = -1, b = -1;
    const double dist = Least(kClearance, &a, &b);
    if (dist >= kClearance) return "";
    auto name = [&](int g) {
      const char* n = mj_id2name(m_, mjOBJ_GEOM, g);
      return std::string(n != nullptr ? n : ("geom " + std::to_string(g)).c_str());
    };
    return name(a) + " to " + name(b) + " " + Num(dist * 1000.0, 1) + " mm";
  }

 private:
  void Box(int g, Eigen::Vector3d* c, Eigen::Vector3d* h) const {
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r(
        d_->geom_xmat + 9 * g);
    const Eigen::Vector3d lc(m_->geom_aabb + 6 * g);
    const Eigen::Vector3d lh(m_->geom_aabb + 6 * g + 3);
    *c = Eigen::Vector3d(d_->geom_xpos + 3 * g) + r * lc;
    *h = r.cwiseAbs() * lh;
  }

  const mjModel* m_;
  mjData* d_;
  std::vector<std::pair<int, unsigned>> moving_;
  std::set<int> obstacles_;
  std::vector<int> obs_;
  std::map<int, unsigned> skip_;
};

// The pose backed off by `dist` along the tool's -z, orientation kept.
std::optional<Eigen::VectorXd> BackOff(const mjModel* m, mjData* d,
                                       MoveWorld& w, int site,
                                       const std::array<int, 6>& dof,
                                       const Eigen::VectorXd& c, double dist,
                                       int how,
                                       const Eigen::VectorXd& lo,
                                       const Eigen::VectorXd& hi) {
  w.Set(c);
  mj_comPos(m, d);
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> r0(
      d->site_xmat + 9 * site);
  const Eigen::Matrix3d rt = r0;
  // how 0: along the tool's -z; 1: straight up; 2: half way between.
  Eigen::Vector3d dir = -rt.col(2);
  if (how == 1) dir = Eigen::Vector3d::UnitZ();
  if (how == 2) dir = (dir + Eigen::Vector3d::UnitZ()).normalized();
  const Eigen::Vector3d pt = Eigen::Vector3d(d->site_xpos + 3 * site) + dist * dir;
  Eigen::VectorXd q = c;
  std::vector<mjtNum> jp(3 * m->nv), jr(3 * m->nv);
  for (int it = 0; it < 200; ++it) {
    w.Set(q);
    mj_comPos(m, d);
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rr(
        d->site_xmat + 9 * site);
    Eigen::Matrix<double, 6, 1> e;
    e.head<3>() = pt - Eigen::Vector3d(d->site_xpos + 3 * site);
    Eigen::Vector3d rot = Eigen::Vector3d::Zero();
    for (int k = 0; k < 3; ++k) rot += 0.5 * Eigen::Vector3d(rr.col(k)).cross(rt.col(k));
    e.tail<3>() = rot;
    if (e.head<3>().norm() < 1e-5 && rot.norm() < 1e-5) return q;
    mj_jacSite(m, d, jp.data(), jr.data(), site);
    Eigen::Matrix<double, 6, 6> J;
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 3; ++k) {
        J(k, j) = jp[k * m->nv + dof[j]];
        J(3 + k, j) = jr[k * m->nv + dof[j]];
      }
    }
    const Eigen::Matrix<double, 6, 6> a =
        J * J.transpose() + 1e-4 * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> dq = J.transpose() * a.ldlt().solve(e);
    for (int j = 0; j < 6; ++j) q[j] = std::clamp(q[j] + dq[j], lo[j], hi[j]);
  }
  return std::nullopt;
}

struct Row {
  std::string move, arm, from, to;
  double model_s = 0.0;
  bool straight_free = false;
  bool found = false;
  int waypoints = 0;
  long long queries = 0;
  double planned_s = 0.0;
  double ratio = 0.0;
  std::string note;
};

}  // namespace

// The planner's budget and seed: 5000 iterations and seed 1 unless the
// command line says otherwise (--iterations N, --seed S after the name).
int g_iterations = 5000;
int g_seed = 1;

int main(int argc, char** argv) {
  for (int i = 2; i + 1 < argc; i += 2) {
    if (std::string(argv[i]) == "--iterations") g_iterations = std::atoi(argv[i + 1]);
    if (std::string(argv[i]) == "--seed") g_seed = std::atoi(argv[i + 1]);
  }
  if (argc < 2) {
    std::fprintf(stderr, "usage: print_layout_moves <layout> [--iterations N] [--seed S]\n");
    return 2;
  }
  const char* dir_env = std::getenv("LAYOUT_DIR");
  const std::string dir = dir_env && dir_env[0] ? dir_env : "layouts";
  std::string error;
  std::optional<Layout> layout =
      LoadLayout(dir + "/" + argv[1] + ".json", &error);
  if (!layout) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 2;
  }
  const ModelPaths paths = ModelPaths::FromEnvironment();

  const std::vector<MoveSpec> moves = {
      {"module_take", 0, "module_set", 0},
      {"module_take", 4, "module_set", 4},
      {"module_take", 8, "module_set", 8},
      {"module_set", 3, "module_take", 4, true},
      {"rail_take", 5, "rail_set", 5},
      {"pin", 0, "locate", 2},
      {"strap_bolt", 3, "strap_bolt", 4},
  };

  std::vector<Row> rows;
  for (const MoveSpec& mv : moves) {
    Row row;
    row.from = mv.from_kind + " " + std::to_string(mv.from_index);
    row.to = mv.to_kind + " " + std::to_string(mv.to_index);
    row.move = row.from + " -> " + row.to;
    LayoutState s0, s1;
    if (!StateAt(*layout, paths, mv.from_kind, mv.from_index, &s0, &error) ||
        !StateAt(*layout, paths, mv.to_kind, mv.to_index, &s1, &error)) {
      row.note = "no pose: " + error;
      rows.push_back(row);
      continue;
    }
    const Work* work = FindWork(*layout, mv.from_kind);
    const ArmUse* use = work ? FindArm(*layout, work->arm) : nullptr;
    const ArmModel* cat = use ? FindArmModel(use->model) : nullptr;
    if (cat == nullptr) {
      row.note = "no arm for " + mv.from_kind;
      rows.push_back(row);
      continue;
    }
    row.arm = use->name;
    const std::string tool = s0.tool[use->name];
    if (mv.empty_return) {
      // The module is down on the bay; the gripper goes back empty.
      s0.held[use->name] = "";
      s0.modules = mv.from_index + 1;
    }
    const std::string held = s0.held[use->name];

    // The time model's figure, as the checker computes a move.
    const std::array<double, 6> qa = s0.joints_deg[use->name];
    const std::array<double, 6> qb = s1.joints_deg[use->name];
    std::array<double, 6> change{};
    for (int j = 0; j < 6; ++j) change[j] = qb[j] - qa[j];
    const double ta = s0.track.count(use->name) ? s0.track[use->name] : 0.0;
    const double tb = s1.track.count(use->name) ? s1.track[use->name] : 0.0;
    double model_t = JointMoveTime(change, cat->joint_speed_deg_s);
    const bool track_moves = use->track && std::abs(tb - ta) > 1e-9;
    if (use->track) {
      const double dist = std::abs(tb - ta) *
                          Eigen::Vector3d(use->track->direction[0],
                                          use->track->direction[1],
                                          use->track->direction[2])
                              .norm();
      model_t = std::max(model_t, TrackMoveTime(dist, use->track->speed,
                                                use->track->accel));
    }
    row.model_s = model_t + kSettleS;

    // The model of the start state.
    cell::ModelPtr model = LoadLayoutModel(*layout, s0, paths, &error);
    if (!model) {
      row.note = "model: " + error;
      rows.push_back(row);
      continue;
    }
    mjData* d = mj_makeData(model.get());
    mj_resetDataKeyframe(model.get(), d, 0);
    const LayoutHandles H = FindHandles(*model, *layout);
    const ArmHandle* ah = nullptr;
    for (const ArmHandle& a : H.arms) {
      if (a.name == use->name) ah = &a;
    }
    std::string designed = "none";
    auto make_world = [&](bool leave_out) {
    MoveWorld w(model.get(), d);
    w.qpos = ah->qpos;
    w.track_qpos = ah->track_qpos;
    w.use_track = track_moves;
    w.fixed_track = ta;

    // Moving set. A link geom with no arm joint between it and the track
    // rides the carriage.
    auto rides = [&](int g) {
      for (int b = model->geom_bodyid[g]; b > 0; b = model->body_parentid[b]) {
        if (model->body_jntnum[b] > 0) {
          const int adr = model->jnt_qposadr[model->body_jntadr[b]];
          return std::find(ah->qpos.begin(), ah->qpos.end(), adr) ==
                 ah->qpos.end();
        }
      }
      return true;
    };
    for (int g : ah->link_geoms) w.AddMoving(g, rides(g) ? kCarriage : kLinks);
    for (int g : ah->carriage_geoms) w.AddMoving(g, kCarriage);
    if (auto t = ah->tool_geoms.find(tool); t != ah->tool_geoms.end()) {
      for (size_t i = 0; i < t->second.size(); ++i) {
        w.AddMoving(t->second[i],
                    i + 1 == t->second.size() ? kToolLast : kToolBody);
      }
    }
    if (!held.empty()) {
      if (auto h = ah->held_geoms.find(held); h != ah->held_geoms.end()) {
        for (int g : h->second) w.AddMoving(g, kHeld);
      }
    }
    // Everything else that is there.
    auto add = [&](const std::vector<int>& gs) {
      for (int g : gs) w.AddObstacle(g);
    };
    add(H.container_geoms);
    add(H.holder_geoms);
    add(H.tube_geoms);
    for (int k = 0; k < s0.rails && k < (int)H.rail_geoms.size(); ++k) add(H.rail_geoms[k]);
    for (int k = 0; k < s0.modules && k < (int)H.module_geoms.size(); ++k) add(H.module_geoms[k]);
    add(H.pallet_base_geoms);
    for (const auto& p : H.pallet_module_geoms) add(p);
    for (const auto& s : H.stock_geoms) add(s.second);
    for (const auto& s : H.item_geoms) add(s.second);
    for (const ArmHandle& a : H.arms) {
      add(a.beam_geoms);
      for (const auto& t : a.waiting_tool_geoms) add(t.second);
      if (&a == ah) {
        for (int g : a.beam_geoms) w.Skip(g, kCarriage);
        for (const auto& t : a.waiting_tool_geoms) {
          for (int g : t.second) w.Skip(g, kCarriage);
        }
        continue;
      }
      add(a.link_geoms);
      add(a.carriage_geoms);
      for (const auto& t : a.tool_geoms) add(t.second);
    }
    // The pairs that touch by design at the two ends.
    auto skip = [&](const std::vector<int>& gs, unsigned groups) {
      if (!leave_out) return;
      for (int g : gs) w.Skip(g, groups);
    };
    auto rail = [&](int k) -> std::vector<int> {
      return k >= 0 && k < (int)H.rail_geoms.size() ? H.rail_geoms[k]
                                                    : std::vector<int>{};
    };
    auto module = [&](int k) -> std::vector<int> {
      return k >= 0 && k < (int)H.module_geoms.size() ? H.module_geoms[k]
                                                      : std::vector<int>{};
    };
    auto pallet = [&](int j) -> std::vector<int> {
      return j >= 0 && j < (int)H.pallet_module_geoms.size()
                 ? H.pallet_module_geoms[j]
                 : std::vector<int>{};
    };
    std::string what;
    if (mv.from_kind == "module_take") {
      const int j = mv.from_index;
      for (int g : pallet(j)) w.RemoveObstacle(g);  // It is the held one.
      skip(pallet(j + 1), kHeld);
      skip(H.pallet_base_geoms, kHeld);
      skip(rail(j), kHeld);
      skip(rail(j + 1), kHeld);
      skip(module(j - 1), kHeld);
      what = "held module vs pallet module behind and deck; vs rails m, m+1 and module m-1";
    } else if (mv.empty_return) {
      skip(module(mv.from_index), kToolBody | kToolLast);
      skip(pallet(mv.to_index), kToolBody | kToolLast);
      what = "gripper vs the module it let go and the pallet module it takes";
    } else if (mv.from_kind == "rail_take") {
      if (auto s = H.stock_geoms.find("rails"); s != H.stock_geoms.end()) {
        skip(s->second, kHeld);
      }
      skip(H.tube_geoms, kHeld);
      what = "held rail vs its stock and the tube";
    } else if (mv.from_kind == "strap_bolt") {
      skip(rail(mv.from_index), kToolLast);
      skip(rail(mv.to_index), kToolLast);
      skip(H.tube_geoms, kToolLast);
      what = "socket vs rails k, k+1 and the tube";
    } else if (mv.from_kind == "pin") {
      const int m = mv.from_index / 4;
      skip(rail(m), kToolLast);
      skip(rail(m + 1), kToolLast);
      skip(module(m), kToolLast);
      what = "pin tool's nose vs rails m, m+1 and module m";
    }
    w.Finish();
    if (leave_out) designed = what;
    return w;
    };
    MoveWorld w = make_world(false);

    // Bounds, weights, limits.
    const int n = track_moves ? 7 : 6;
    Eigen::VectorXd lo(n), hi(n), weights(n), start(n), goal(n);
    const bool big = cat->key == "kr210_r2700_2";
    const std::array<double, 6> wk = {4.5, 4.2, 3.0, 1.8, 1.8, 1.3};
    const std::array<double, 6> wu = {1.8, 1.6, 1.0, 0.6, 0.5, 0.4};
    cell::JointLimits limits;
    limits.velocity.resize(n);
    limits.acceleration.resize(n);
    limits.jerk.resize(n);
    for (int j = 0; j < 6; ++j) {
      lo[j] = (cat->joint_lo_deg[j] + kMarginDeg) * kDeg;
      hi[j] = (cat->joint_hi_deg[j] - kMarginDeg) * kDeg;
      const int jid = [&] {
        for (int k = 0; k < model->njnt; ++k) {
          if (model->jnt_qposadr[k] == ah->qpos[j]) return k;
        }
        return -1;
      }();
      if (jid >= 0 && model->jnt_limited[jid]) {
        lo[j] = std::max(lo[j], model->jnt_range[2 * jid]);
        hi[j] = std::min(hi[j], model->jnt_range[2 * jid + 1]);
      }
      weights[j] = big ? wk[j] : wu[j];
      start[j] = qa[j] * kDeg;
      goal[j] = qb[j] * kDeg;
      limits.velocity[j] = cat->joint_speed_deg_s[j] * kDeg;
      limits.acceleration[j] = limits.velocity[j] / kToFullSpeedS;
      limits.jerk[j] = limits.acceleration[j] / kJerkRiseS;
    }
    if (track_moves) {
      lo[6] = use->track->lo;
      hi[6] = use->track->hi;
      weights[6] = 1.0;
      start[6] = ta;
      goal[6] = tb;
      limits.velocity[6] = use->track->speed;
      limits.acceleration[6] = use->track->accel;
      limits.jerk[6] = use->track->accel / kJerkRiseS;
    }
    for (int j = 0; j < n; ++j) {  // The poses themselves are inside.
      lo[j] = std::min(lo[j], std::min(start[j], goal[j]));
      hi[j] = std::max(hi[j], std::max(start[j], goal[j]));
    }

    cell::IsFree is_free = [&](const Eigen::VectorXd& c) { return w.Free(c); };
    long long queries = 0;
    // End poses: free, or backed off along the tool's -z.
    const int site = ah->work_site.count(tool) ? ah->work_site.at(tool) : -1;
    std::string notes;
    auto end_pose = [&](const Eigen::VectorXd& c, const char* which,
                        std::optional<Eigen::VectorXd>* backed) -> bool {
      ++queries;
      if (w.Free(c)) return true;
      const std::string why = w.Why(c);
      static const char* kHow[] = {"along the tool's -z", "straight up",
                                   "half up, half along the tool's -z"};
      for (int how = 0; how < 3; ++how) {
        for (double dist : {0.05, 0.075, 0.10}) {
          if (site < 0) break;
          std::optional<Eigen::VectorXd> b = BackOff(
              model.get(), d, w, site, ah->dof, c, dist, how, lo, hi);
          ++queries;
          if (b && w.Free(*b)) {
            *backed = b;
            notes += std::string(which) + " not free (" + why +
                     "), backed off " + Num(dist * 1000.0, 0) + " mm " +
                     kHow[how] + "; ";
            return true;
          }
        }
      }
      notes += std::string(which) + " not free (" + why +
               ") even backed off 100 mm three ways; ";
      return false;
    };
    std::optional<Eigen::VectorXd> bs, bg;
    bool ok_s = end_pose(start, "start", &bs);
    bool ok_g = end_pose(goal, "goal", &bg);
    if (!ok_s || !ok_g) {
      // Backing off was not enough: leave the designed pairs out.
      w = make_world(true);
      bs.reset();
      bg.reset();
      notes += "so the designed pairs are left out; ";
      ok_s = end_pose(start, "start", &bs);
      ok_g = end_pose(goal, "goal", &bg);
    }
    const Eigen::VectorXd ps = bs ? *bs : start;
    const Eigen::VectorXd pg = bg ? *bg : goal;

    const double resolution = 0.01;  // m of motion between checked points
    std::int64_t straight_q = 0;
    row.straight_free =
        ok_s && ok_g &&
        cell::EdgeIsFree(ps, pg, is_free, resolution, weights,
                         cell::EdgeOrder::kBisection, &straight_q);
    queries += straight_q;
    if (!row.straight_free && ok_s && ok_g) {
      // Where the straight line is blocked: the first sample, coarse to fine.
      for (int k = 1; k < 64; ++k) {
        const double f = k / 64.0;
        const Eigen::VectorXd c = ps + f * (pg - ps);
        const std::string why = w.Why(c);
        if (!why.empty()) {
          notes += "straight line blocked at " + Num(f, 2) + ": " + why + "; ";
          break;
        }
      }
    }

    if (ok_s && ok_g) {
      cell::RrtOptions ro;
      ro.step_size = big ? 1.0 : 0.4;
      ro.edge_resolution = resolution;
      ro.weights = weights;
      ro.max_iterations = g_iterations;
      ro.max_seconds = 600.0;  // The iterations bind, for repeatable results.
      ro.seed = g_seed;
      cell::RrtReport rr;
      std::optional<JointPath> path =
          cell::PlanRrtConnect(ps, pg, lo, hi, is_free, ro, &rr);
      queries += rr.collision_queries;
      if (path) {
        cell::ShortenOptions so;
        so.edge_resolution = resolution;
        so.weights = weights;
        so.max_seconds = 600.0;
        so.seed = g_seed;
        cell::ShortenReport sr;
        std::optional<JointPath> shorter =
            cell::ShortenPath(*path, is_free, so, &sr);
        queries += sr.collision_queries;
        JointPath p = shorter ? *shorter : *path;
        row.found = true;
        row.waypoints = static_cast<int>(p.size());
        if (bs) p.insert(p.begin(), start);
        if (bg) p.push_back(goal);
        std::optional<cell::Trajectory> tr = cell::Trajectory::Plan(p, limits);
        if (tr) {
          row.planned_s = tr->duration() + kSettleS;
          row.ratio = row.planned_s / row.model_s;
        } else {
          notes += "path could not be timed; ";
        }
      } else {
        notes += std::string("planner: ") + cell::ToString(rr.status) + "; ";
      }
    }
    row.queries = queries;
    row.note = notes + "designed pairs left out: " + designed;
    rows.push_back(row);
    mj_deleteData(d);
  }

  std::printf(
      "move,arm,model_s,straight_free,planned,waypoints,queries,planned_s,"
      "ratio,note\n");
  std::vector<double> ratios;
  std::string unplanned;
  for (const Row& r : rows) {
    std::printf("%s,%s,%s,%s,%s,%d,%lld,%s,%s,%s\n", Csv(r.move).c_str(),
                r.arm.c_str(), Num(r.model_s).c_str(),
                r.straight_free ? "yes" : "no", r.found ? "found" : "not found",
                r.waypoints, r.queries,
                r.found ? Num(r.planned_s).c_str() : "",
                r.found ? Num(r.ratio, 3).c_str() : "", Csv(r.note).c_str());
    if (r.found && r.ratio > 0) {
      ratios.push_back(r.ratio);
    } else {
      unplanned += r.move + "; ";
    }
  }
  std::printf("# clearance %.3f m between the moving set (links, carriage, tool on the flange, held part) and everything else there\n", kClearance);
  std::printf("# state: the bay as built at the START of each move, the other arm at its wait pose; an end pose not free is backed off 50 to 100 mm (along the tool's -z, else straight up, else half and half); only where that is not enough are the pairs that touch by design left out, along the whole move (see note)\n");
  std::printf("# held part: a fixed part of the moving set, against everything but the designed pairs; not checked against the arm's own links\n");
  std::printf("# timing (ours): joint acceleration = datasheet speed / %.1f s, jerk = acceleration / %.1f s; track: the layout's speed and acceleration, jerk = acceleration / %.1f s\n", kToFullSpeedS, kJerkRiseS, kJerkRiseS);
  std::printf("# planned_s = Trajectory::Plan duration (max_deviation 0.002) + %.1f s settle, as the model adds; ratio = planned_s / model_s\n", kSettleS);
  std::printf("# planner: RRT-Connect seed %d, %d iterations, edge resolution 0.01 m (weighted), ShortenPath 200 tries, same seed; waypoints after shortening, back-off poses not counted; queries include end-pose and straight-line checks\n", g_seed, g_iterations);
  if (!ratios.empty()) {
    std::vector<double> s = ratios;
    std::sort(s.begin(), s.end());
    const size_t k = s.size();
    const double median =
        k % 2 ? s[k / 2] : 0.5 * (s[k / 2 - 1] + s[k / 2]);
    std::printf("# ratio over %zu moves: median %.3f, range %.3f to %.3f\n", k,
                median, s.front(), s.back());
  }
  std::printf("# not planned: %s\n", unplanned.empty() ? "none" : unplanned.c_str());
  return 0;
}
