// One module's cycle of a layout's committed plan, run in physics. See
// src/physics_run.h.

#include "src/physics_run.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <map>
#include <numbers>
#include <set>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/arm.h"
#include "src/arm_follower.h"
#include "src/bay_hardware.h"
#include "src/layout_check.h"
#include "src/layout_scene.h"
#include "src/mj_handles.h"
#include "src/suction.h"

namespace cell::layout {
namespace {

constexpr double kRadToDeg = 180.0 / std::numbers::pi;

std::string Fmt(const char* format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

// A rigid pose in the world.
struct Frame {
  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
};

Frame BodyFrame(const mjData& d, int body) {
  Frame f;
  f.p = Eigen::Map<const Eigen::Vector3d>(d.xpos + 3 * body);
  f.r = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d.xmat + 9 * body);
  return f;
}

Frame SiteFrame(const mjData& d, int site) {
  Frame f;
  f.p = Eigen::Map<const Eigen::Vector3d>(d.site_xpos + 3 * site);
  f.r = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(d.site_xmat + 9 * site);
  return f;
}

// The distance and the angle between two frames, in mm and degrees.
std::pair<double, double> FrameError(const Frame& a, const Frame& b) {
  const Eigen::AngleAxisd turn(Eigen::Matrix3d(a.r.transpose() * b.r));
  return {(a.p - b.p).norm() * 1e3, std::abs(turn.angle()) * kRadToDeg};
}

// The same for a module, whose shape and holes are the same turned half
// round about its normal (src/bay_model.cc: every geom of side a has its
// twin on side b, and the four holes lie symmetric about the centre): a
// module held or set that way round is where the plan has it. The smaller
// of the two errors.
std::pair<double, double> ModuleError(const Frame& a, const Frame& b) {
  Frame turned = a;
  turned.r = a.r * Eigen::Matrix3d(Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitZ()));
  const std::pair<double, double> one = FrameError(a, b);
  const std::pair<double, double> two = FrameError(turned, b);
  return two.second < one.second ? two : one;
}

// Moves every geom of `geoms` by the motion that takes body frame `from`
// to `to`, in the data's world poses (after kinematics).
void MoveGeoms(mjData* d, const std::vector<int>& geoms, const Frame& from,
               const Frame& to) {
  const Eigen::Matrix3d r = to.r * from.r.transpose();
  const Eigen::Vector3d t = to.p - r * from.p;
  for (int g : geoms) {
    Eigen::Map<Eigen::Vector3d> p(d->geom_xpos + 3 * g);
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> m(d->geom_xmat + 9 * g);
    const Eigen::Vector3d np = r * Eigen::Vector3d(p) + t;
    const Eigen::Matrix3d nm = r * Eigen::Matrix3d(m);
    p = np;
    m = nm;
  }
}

double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

void Widen(PhysicsFigures* into, const PhysicsFigures& f) {
  into->joint_error_mrad = std::max(into->joint_error_mrad, f.joint_error_mrad);
  into->track_error_mm = std::max(into->track_error_mm, f.track_error_mm);
  if (f.frame_error_mm > into->frame_error_mm) into->frame_error_mm = f.frame_error_mm;
  into->frame_error_deg = std::max(into->frame_error_deg, f.frame_error_deg);
  into->frame_is_module = into->frame_is_module || f.frame_is_module;
  if (f.clearance_m < into->clearance_m) {
    into->clearance_m = f.clearance_m;
    into->clearance_pair = f.clearance_pair;
    into->clearance_t = f.clearance_t;
  }
  into->plan_clearance_m = std::min(into->plan_clearance_m, f.plan_clearance_m);
  for (int j = 0; j < 7; ++j) {
    into->torque_ratio[j] = std::max(into->torque_ratio[j], f.torque_ratio[j]);
  }
}

}  // namespace

struct PhysicsRun::Impl {
  const Layout* layout = nullptr;
  const Plan* plan = nullptr;
  PhysicsOptions options;
  PhysicsResult result;

  // The checker's world: its report, scene, player and a data to measure in.
  CheckReport report;
  std::unique_ptr<LayoutScene> scene;
  std::unique_ptr<PlanPlayer> player;
  DataPtr measure;
  int arm = -1;            // Index of the module arm.
  std::string tool;        // On its flange through the two steps.

  // The physics world.
  ModelPtr model;
  DataPtr data;
  LayoutHandles handles;   // Of the physics model.
  std::optional<Arm> follower_arm;
  std::unique_ptr<ArmFollower> follower;
  std::optional<Suction> suction;
  int track_motor = -1;
  double track_lag = 0.0;
  int module_body = -1;    // The free module, in the physics model.
  int pallet_weld = -1;
  int work_site = -1;      // The tool's working point, in the physics model.

  // In the checker's model.
  int scene_held_body = -1;     // The held module's body.
  int scene_set_body = -1;      // The module's body in the bay.
  int scene_work_site = -1;

  // Plan time.
  double start = 0.0, end = 0.0, grip_t = 0.0, release_t = 0.0;
  bool following = false, gripped = false, released = false;
  bool ended = false, failed = false;
  long tick = 0;
  long sample_every = 1;
  long settle_ticks = 0, hold_ticks = 0, run_ticks = 0;

  // Figures by segment index of the module arm.
  std::map<int, PhysicsFigures> by_segment;
  std::vector<double> torque_limit;  // Six joints, then the track.
  Eigen::VectorXd acceleration, torque;

  double PlanAt(long k) const {
    const double tau = static_cast<double>(k) * options.timestep;
    const double hold = static_cast<double>(hold_ticks) * options.timestep;
    return Clamp(start + (tau - hold), start, end);
  }

  // The plan's configuration of the module arm at plan time t, clamped to
  // the two steps.
  Config ConfigOf(double t) const {
    return player->At(Clamp(t, start, end)).arms[arm].config;
  }

  // Angles, speeds and accelerations of the seven axes at plan time t.
  void Sample(double t, Eigen::VectorXd* q, Eigen::VectorXd* v, Eigen::VectorXd* a) const {
    const Config c0 = ConfigOf(t - kDifference);
    const Config c1 = ConfigOf(t);
    const Config c2 = ConfigOf(t + kDifference);
    q->resize(7);
    v->resize(7);
    a->resize(7);
    for (int j = 0; j < 7; ++j) {
      (*q)[j] = c1[j];
      (*v)[j] = (c2[j] - c0[j]) / (2.0 * kDifference);
      (*a)[j] = (c2[j] - 2.0 * c1[j] + c0[j]) / (kDifference * kDifference);
    }
  }

  void Fail(const std::string& why) {
    if (!failed) result.error = why;
    failed = true;
    ended = true;
  }

  // Every other arm where the plan has it at plan time t, at rest.
  void PoseOthers(double t) {
    const WorldAt w = player->At(t);
    for (int i = 0; i < static_cast<int>(handles.arms.size()); ++i) {
      if (i == arm) continue;
      const ArmHandle& h = handles.arms[i];
      for (int j = 0; j < 6; ++j) {
        data->qpos[h.qpos[j]] = w.arms[i].config[j];
        data->qvel[h.dof[j]] = 0.0;
      }
      if (h.track_qpos >= 0) {
        data->qpos[h.track_qpos] = w.arms[i].config[6];
        data->qvel[h.track_dof] = 0.0;
      }
    }
  }

  // The track's command at plan time t, with the arm's planned
  // accelerations in the feedforward.
  void DriveTrack(double t, bool moving) {
    const ArmHandle& h = handles.arms[arm];
    Eigen::VectorXd q, v, a;
    Sample(t, &q, &v, &a);
    if (!moving) {
      v.setZero();
      a.setZero();
    }
    acceleration.setZero(model->nv);
    torque.setZero(model->nv);
    for (int j = 0; j < 6; ++j) acceleration[h.dof[j]] = a[j];
    acceleration[h.track_dof] = a[6];
    mj_mulM(model.get(), data.get(), torque.data(), acceleration.data());
    const double kp = model->actuator_gainprm[mjNGAIN * track_motor];
    data->ctrl[track_motor] = q[6] + track_lag * v[6] +
                              (torque[h.track_dof] + data->qfrc_bias[h.track_dof]) / kp;
    follower->SetCarrier({{h.track_dof, a[6]}});
  }

  // The checker's figures at plan time t: in physics, then in the plan.
  void Measure(double t, PhysicsFigures* f) {
    WorldAt w = player->At(t);
    const std::string name = layout->arms[arm].name;
    auto least = [&](const std::vector<Closest>& all, std::string* pair) {
      double best = kSceneFar;
      for (const Closest& c : all) {
        if (c.arm != name && c.other_arm != name) continue;
        if (c.distance < best) {
          best = c.distance;
          if (pair != nullptr) {
            *pair = c.a + " to " + c.b + (c.by_design ? " (near by design, but into each other)" : "");
          }
        }
      }
      return best;
    };
    // The plan's world; its kinematics stay in `measure` for the frames.
    const std::vector<Closest> planned =
        InstantDistances(*scene, report, *player, w, measure.get());
    f->plan_clearance_m = least(planned, nullptr);
    const bool held = w.arms[arm].held == "module";
    Frame planned_frame, physics_frame;
    if (held) {
      planned_frame = BodyFrame(*measure, scene_held_body);
      physics_frame = BodyFrame(*data, module_body);
    } else {
      planned_frame = SiteFrame(*measure, scene_work_site);
      physics_frame = SiteFrame(*data, work_site);
    }
    const auto [mm, deg] = held ? ModuleError(planned_frame, physics_frame)
                                : FrameError(planned_frame, physics_frame);
    f->frame_error_mm = mm;
    f->frame_error_deg = deg;
    f->frame_is_module = held;
    // The physics world: the arm at its measured joints and track, the
    // module where the physics has it.
    const ArmHandle& h = handles.arms[arm];
    for (int j = 0; j < 6; ++j) w.arms[arm].config[j] = data->qpos[h.qpos[j]];
    w.arms[arm].config[6] = data->qpos[h.track_qpos];
    const Frame module = BodyFrame(*data, module_body);
    const Built built = player->BuiltAt(w);
    const bool set = std::find(built.modules.begin(), built.modules.end(),
                               result.module) != built.modules.end();
    const LayoutHandles& H = scene->handles();
    auto adjust = [&](mjData* d) {
      if (held) {
        MoveGeoms(d, H.arms[arm].held_geoms.at("module"), BodyFrame(*d, scene_held_body),
                  module);
      } else if (set) {
        MoveGeoms(d, H.module_geoms[result.module], BodyFrame(*d, scene_set_body), module);
      }
    };
    std::string pair;
    const std::vector<Closest> physics =
        InstantDistances(*scene, report, *player, w, measure.get(), adjust);
    f->clearance_m = least(physics, &pair);
    f->clearance_pair = pair;
    f->clearance_t = t;
  }

  PhysicsFigures& SegmentAt(double t) {
    const WorldAt w = player->At(t);
    const int index = w.arms[arm].segment;
    auto it = by_segment.find(index);
    if (it == by_segment.end()) {
      PhysicsFigures f;
      const Segment& s = plan->arms[arm].segments[index];
      f.step = s.step;
      f.kind = s.kind;
      f.t0 = s.t0;
      f.t1 = s.t1;
      it = by_segment.emplace(index, f).first;
    }
    return it->second;
  }

  bool StepOnce();
};

bool PhysicsRun::Impl::StepOnce() {
  if (ended) return false;
  const long total = hold_ticks + run_ticks + settle_ticks;
  if (tick >= total) {
    ended = true;
    return false;
  }
  const double t = PlanAt(tick);
  const bool in_steps = tick >= hold_ticks && tick < hold_ticks + run_ticks;
  // The plan starts once the hold is over.
  if (tick == hold_ticks && !following) {
    TimedPath path;
    path.seconds = end - start;
    const Impl* self = this;
    path.at = [self](double s) {
      Eigen::VectorXd q, v, a;
      self->Sample(self->start + s, &q, &v, &a);
      JointSample out;
      out.position = q.head(6);
      out.velocity = v.head(6);
      out.acceleration = a.head(6);
      out.jerk = Eigen::VectorXd::Zero(6);
      return out;
    };
    std::string why;
    if (!follower->Follow(path, *data, &why)) {
      Fail("the follower refused the plan: " + why);
      return false;
    }
    following = true;
  }
  // The grip and the release, at the end of the work segments.
  if (in_steps && !gripped && t >= grip_t) {
    data->eq_active[pallet_weld] = 0;
    GripFailure failure;
    if (!suction->Grip(model.get(), data.get(), &failure)) {
      Fail("the grip failed: " + failure.message);
      return false;
    }
    gripped = true;
    const std::optional<Pose> at = suction->HeldPoseInTipAtGrip();
    if (at) {
      result.grip_offset_mm =
          std::abs(at->position.z() - bay::kModuleThickness / 2.0) * 1e3;
    }
  }
  if (in_steps && gripped && !released && t >= release_t) {
    suction->Release(data.get());
    released = true;
  }
  PoseOthers(t);
  DriveTrack(t, in_steps);
  follower->Update(*model, data.get());
  mj_step(model.get(), data.get());
  ++tick;
  for (int i = 0; i < model->nv; ++i) {
    if (!std::isfinite(data->qacc[i])) {
      Fail(Fmt("the physics diverged at plan time %.3f s", t));
      return false;
    }
  }
  if (!in_steps && tick <= hold_ticks) return true;
  // What the step did: errors every tick, the checker's figures every
  // sample.
  const double now = PlanAt(tick);
  PhysicsFigures f;
  const Eigen::VectorXd error = follower->TrackingError(*data);
  f.joint_error_mrad = error.cwiseAbs().maxCoeff() * 1e3;
  Eigen::VectorXd q, v, a;
  Sample(now, &q, &v, &a);
  const ArmHandle& h = handles.arms[arm];
  f.track_error_mm = std::abs(data->qpos[h.track_qpos] - q[6]) * 1e3;
  for (int j = 0; j < 6; ++j) {
    f.torque_ratio[j] = std::abs(data->actuator_force[follower_arm->motor[j]]) / torque_limit[j];
  }
  f.torque_ratio[6] = std::abs(data->actuator_force[track_motor]) / torque_limit[6];
  if ((tick - hold_ticks) % sample_every == 0) {
    // mj_step leaves the frames of the state before its integration; the
    // figures are of the state after it.
    mj_kinematics(model.get(), data.get());
    Measure(now, &f);
  }
  if (tick > hold_ticks + run_ticks) {
    Widen(&result.settle, f);
  } else {
    Widen(&SegmentAt(std::min(now, end - 1e-9)), f);
  }
  return true;
}

PhysicsRun::PhysicsRun() : impl_(std::make_unique<Impl>()) {}
PhysicsRun::~PhysicsRun() = default;

std::unique_ptr<PhysicsRun> PhysicsRun::Create(const Layout& layout, const Plan& plan,
                                               const ModelPaths& paths,
                                               const PhysicsOptions& options,
                                               std::string* error) {
  auto fail = [&](const std::string& what) {
    if (error != nullptr) *error = what;
    return nullptr;
  };
  std::unique_ptr<PhysicsRun> run(new PhysicsRun());
  Impl& r = *run->impl_;
  r.layout = &layout;
  r.plan = &plan;
  r.options = options;
  r.result.layout = layout.name;
  r.result.cycle_s = plan.bay_planned_s;
  if (!(options.timestep > 0.0) || !(options.sample_s >= options.timestep)) {
    return fail("the timestep must be positive and no more than sample_s");
  }

  // The checker's world.
  r.report = CheckLayout(layout, paths);
  if (!r.report.error.empty()) return fail(r.report.error);
  std::string why;
  r.scene = LayoutScene::Load(layout, paths, r.report.joint_margin_deg, &why);
  if (!r.scene) return fail(why);
  r.player = std::make_unique<PlanPlayer>(*r.scene, r.report, plan);
  r.measure = r.scene->MakeData();

  // The take and the set.
  std::map<int, const BayStep*> steps;
  for (const BayStep& s : r.report.steps) steps[s.number] = &s;
  std::map<int, const StepTime*> times;
  for (const StepTime& s : plan.steps) times[s.number] = &s;
  // The set step that follows each take of the same arm.
  std::vector<std::pair<const BayStep*, const BayStep*>> cycles;
  for (const BayStep& s : r.report.steps) {
    if (s.kind != "module_take") continue;
    for (const BayStep& n : r.report.steps) {
      if (n.number > s.number && n.arm == s.arm && n.kind == "module_set") {
        cycles.emplace_back(&s, &n);
        break;
      }
    }
  }
  if (cycles.empty()) return fail("the plan has no module take followed by a set");
  const BayStep* take = nullptr;
  const BayStep* set = nullptr;
  if (options.take >= 0) {
    for (const auto& [t, s] : cycles) {
      if (t->index == options.take) {
        take = t;
        set = s;
      }
    }
    if (take == nullptr) return fail(Fmt("no module_take %d in the plan", options.take));
  } else {
    // The hardest carry: the move of the set step that holds the module
    // and took the most queries to find.
    std::int64_t most = -1;
    for (const auto& [t, s] : cycles) {
      for (const ArmPlan& a : plan.arms) {
        for (const Segment& g : a.segments) {
          if (g.step == s->number && g.kind == "move" && g.held == "module" &&
              g.queries > most) {
            most = g.queries;
            take = t;
            set = s;
          }
        }
      }
    }
    if (take == nullptr) return fail("no carry of a module in the plan");
  }
  r.arm = r.scene->ArmIndex(take->arm);
  if (r.arm < 0) return fail("no arm " + take->arm);
  if (!times.count(take->number) || !times.count(set->number)) {
    return fail("the plan has no times for the take and the set");
  }
  r.result.arm = take->arm;
  r.result.take = take->index;
  r.result.module = set->index;
  r.result.take_step = take->number;
  r.result.set_step = set->number;
  r.start = times[take->number]->start;
  r.end = times[set->number]->end;
  for (const Segment& g : plan.arms[r.arm].segments) {
    if (g.kind != "work") continue;
    if (g.step == take->number) r.grip_t = g.t1;
    if (g.step == set->number) r.release_t = g.t1;
  }
  if (!(r.grip_t > r.start && r.release_t > r.grip_t && r.release_t < r.end)) {
    return fail("the take's and the set's work segments are not where they should be");
  }

  // The state at the take's start.
  const WorldAt w0 = r.player->At(r.start);
  r.tool = w0.arms[r.arm].tool;
  const Built built = r.player->BuiltAt(w0);
  LayoutState state;
  state.rails = static_cast<int>(built.rails.size());
  {
    std::set<int> on(built.modules.begin(), built.modules.end());
    int n = 0;
    while (on.count(n)) ++n;
    state.modules = n;  // The contiguous ones; the others are not drawn.
  }
  if (!layout.pallets.empty()) {
    state.pallet_modules = layout.pallets[0].modules - take->index;
  }
  for (int i = 0; i < r.scene->ArmCount(); ++i) {
    const std::string& name = layout.arms[i].name;
    const ArmAt& a = w0.arms[i];
    if (a.held == "module" || a.held == "rail") {
      return fail("arm " + name + " holds a " + a.held + " at the take's start");
    }
    state.tool[name] = a.tool;
    std::array<double, 6> deg{};
    for (int j = 0; j < 6; ++j) deg[j] = a.config[j] * kRadToDeg;
    state.joints_deg[name] = deg;
    state.track[name] = a.config[6];
  }
  state.physics_arm = take->arm;
  state.carried_take = take->index;
  r.model = LoadLayoutModel(layout, state, paths, &why);
  if (!r.model) return fail(why);
  mjModel* m = r.model.get();
  r.handles = FindHandles(*m, layout);
  const ArmHandle& h = r.handles.arms[r.arm];
  if (h.track_qpos < 0) return fail("the module arm has no track");

  // Timestep and contacts (src/physics_run.h).
  m->opt.timestep = options.timestep;
  m->opt.cone = mjCONE_ELLIPTIC;
  r.module_body = mj_name2id(m, mjOBJ_BODY, "carried_module");
  r.pallet_weld = mj_name2id(m, mjOBJ_EQUALITY, "pallet_hold");
  if (r.module_body < 0 || r.pallet_weld < 0) return fail("the physics model has no carried module");
  std::set<int> rails;
  if (r.result.module + 1 >= static_cast<int>(r.handles.rail_geoms.size())) {
    return fail("the rails of the module are not in the model");
  }
  for (int k : {r.result.module, r.result.module + 1}) {
    rails.insert(r.handles.rail_geoms[k].begin(), r.handles.rail_geoms[k].end());
  }
  for (int g = 0; g < m->ngeom; ++g) {
    m->geom_contype[g] = 0;
    m->geom_conaffinity[g] = 0;
    const bool module = m->geom_bodyid[g] == r.module_body;
    if (module) m->geom_contype[g] = 2;
    if (rails.count(g)) m->geom_conaffinity[g] = 2;
    if (module || rails.count(g)) {
      m->geom_condim[g] = 3;
      m->geom_solref[mjNREF * g] = 4.0 * options.timestep;
      m->geom_solref[mjNREF * g + 1] = 1.0;
      m->geom_solimp[mjNIMP * g] = 0.95;
      m->geom_solimp[mjNIMP * g + 1] = 0.99;
      m->geom_solimp[mjNIMP * g + 2] = 0.0005;
      m->geom_friction[3 * g] = bay::kLocateFriction;
      m->geom_margin[g] = 0.0;
      m->geom_gap[g] = 0.0;
    }
  }

  // The arm, the follower, the suction.
  const std::string prefix = take->arm + "_";
  r.follower_arm = FindArm(*m, ArmNames{prefix, prefix + "tool_mount",
                                        prefix + "work_" + r.tool});
  if (!r.follower_arm) return fail("the physics model's arm has no motors where expected");
  r.track_motor = mj_name2id(m, mjOBJ_ACTUATOR, (prefix + "track_motor").c_str());
  if (r.track_motor < 0) return fail("no track motor");
  r.track_lag = (-m->actuator_biasprm[mjNBIAS * r.track_motor + 2] + m->dof_damping[h.track_dof]) /
                m->actuator_gainprm[mjNGAIN * r.track_motor];
  r.work_site = r.follower_arm->tip_site;
  for (int j = 0; j < 6; ++j) r.torque_limit.push_back(r.follower_arm->torque_limit[j]);
  r.torque_limit.push_back(m->actuator_forcerange[2 * r.track_motor + 1]);
  SuctionOptions so;
  so.tool_body = prefix + "tool_mount";
  so.tip_site = prefix + "work_" + r.tool;
  so.weld = "suction";
  so.module_prefix = "carried_module";
  so.module_size = Eigen::Vector3d(bay::kModuleLength, bay::kModuleWidth, bay::kModuleThickness);
  r.suction = Suction::Create(*m, so, &why);
  if (!r.suction) return fail("suction: " + why);

  // In the checker's model.
  const LayoutHandles& H = r.scene->handles();
  auto held = H.arms[r.arm].held_geoms.find("module");
  if (held == H.arms[r.arm].held_geoms.end() || held->second.empty()) {
    return fail("the checker's model has no held module");
  }
  r.scene_held_body = r.scene->model().geom_bodyid[held->second.front()];
  if (r.result.module >= static_cast<int>(H.module_geoms.size()) ||
      H.module_geoms[r.result.module].empty()) {
    return fail("the checker's model has no module " + std::to_string(r.result.module));
  }
  r.scene_set_body = r.scene->model().geom_bodyid[H.module_geoms[r.result.module].front()];
  auto site = H.arms[r.arm].work_site.find(r.tool);
  if (site == H.arms[r.arm].work_site.end()) return fail("no work site for " + r.tool);
  r.scene_work_site = site->second;

  // The start: keyframe 0 holds every arm at the take's start.
  r.data = MakeData(*m);
  mj_resetDataKeyframe(m, r.data.get(), 0);
  for (int j = 0; j < 6; ++j) r.data->ctrl[r.follower_arm->motor[j]] = r.data->qpos[h.qpos[j]];
  r.data->ctrl[r.track_motor] = r.data->qpos[h.track_qpos];
  mj_forward(m, r.data.get());
  // Exactly the plan's angles, not their round trip through degrees.
  const Config c0 = r.ConfigOf(r.start);
  for (int j = 0; j < 6; ++j) r.data->qpos[h.qpos[j]] = c0[j];
  r.data->qpos[h.track_qpos] = c0[6];
  mj_forward(m, r.data.get());
  r.follower = std::make_unique<ArmFollower>(*r.follower_arm);
  r.follower->Hold(*r.data);
  r.hold_ticks = std::lround(options.hold_s / options.timestep);
  r.run_ticks = std::lround((r.end - r.start) / options.timestep);
  r.settle_ticks = std::lround(options.settle_s / options.timestep);
  r.sample_every = std::max(1L, std::lround(options.sample_s / options.timestep));
  return run;
}

const mjModel& PhysicsRun::model() const { return *impl_->model; }
mjData* PhysicsRun::data() { return impl_->data.get(); }
double PhysicsRun::PlanTime() const { return impl_->PlanAt(impl_->tick); }
bool PhysicsRun::Step() { return impl_->StepOnce(); }
bool PhysicsRun::Ended() const { return impl_->ended; }

PhysicsResult PhysicsRun::Finish() {
  Impl& r = *impl_;
  while (r.StepOnce()) {
  }
  PhysicsResult& out = r.result;
  if (r.failed) return out;
  if (!r.gripped || !r.released) {
    out.error = "the run ended before the module was gripped and let go";
    return out;
  }
  // The segments in time order, and the two steps.
  PhysicsFigures take, set;
  take.step = out.take_step;
  set.step = out.set_step;
  take.kind = set.kind = "step";
  for (const auto& [index, f] : r.by_segment) {
    out.segments.push_back(f);
    PhysicsFigures& into = f.step == out.take_step ? take : set;
    Widen(&into, f);
  }
  for (const StepTime& s : r.plan->steps) {
    if (s.number == take.step) {
      take.t0 = s.start;
      take.t1 = s.end;
    }
    if (s.number == set.step) {
      set.t0 = s.start;
      set.t1 = s.end;
    }
  }
  out.steps = {take, set};
  out.settle.step = out.set_step;
  out.settle.t0 = out.settle.t1 = r.end;

  // The module on its rails, against its designed place.
  mj_kinematics(&r.scene->model(), r.measure.get());
  const Frame designed = BodyFrame(*r.measure, r.scene_set_body);
  const Frame physics = BodyFrame(*r.data, r.module_body);
  const Eigen::Vector3d dp = designed.r.transpose() * (physics.p - designed.p);
  const Eigen::Matrix3d dr = designed.r.transpose() * physics.r;
  out.set_down.across_mm = dp.x() * 1e3;
  out.set_down.along_mm = dp.y() * 1e3;
  out.set_down.height_mm = dp.z() * 1e3;
  out.set_down.offset_mm = std::hypot(dp.x(), dp.y()) * 1e3;
  // Modulo a half turn, for the module's symmetry (ModuleError).
  double yaw = std::atan2(dr(1, 0), dr(0, 0));
  if (yaw > std::numbers::pi / 2.0) yaw -= std::numbers::pi;
  if (yaw <= -std::numbers::pi / 2.0) yaw += std::numbers::pi;
  out.set_down.yaw_deg = yaw * kRadToDeg;

  // The record.
  PhysicsRecord& rec = out.record;
  rec.layout = r.layout->name;
  rec.module = out.module;
  rec.take = out.take;
  rec.clearance_m = r.report.clearance_m;
  rec.gate_offset_mm = bay::kGateOffset * 1e3;
  rec.gate_yaw_deg = bay::kGateYaw * kRadToDeg;
  auto round = [](double v, double unit) { return std::round(v / unit) * unit; };
  rec.set_down.offset_mm = round(out.set_down.offset_mm, 1e-4);
  rec.set_down.across_mm = round(out.set_down.across_mm, 1e-4);
  rec.set_down.along_mm = round(out.set_down.along_mm, 1e-4);
  rec.set_down.yaw_deg = round(out.set_down.yaw_deg, 1e-4);
  rec.set_down.height_mm = round(out.set_down.height_mm, 1e-4);
  for (const PhysicsFigures& f : out.steps) {
    PhysicsStep s;
    s.number = f.step;
    for (const StepTime& t : r.plan->steps) {
      if (t.number == f.step) {
        s.what = t.what;
        s.arm = t.arm;
        s.start_s = t.start;
        s.end_s = t.end;
      }
    }
    s.planned_s = s.end_s - s.start_s;
    s.max_joint_error_mrad = round(f.joint_error_mrad, 1e-4);
    s.max_frame_error_mm = round(f.frame_error_mm, 1e-4);
    s.max_frame_error_deg = round(f.frame_error_deg, 1e-4);
    s.least_clearance_m = round(f.clearance_m, 1e-6);
    s.least_clearance_pair = f.clearance_pair;
    s.least_clearance_t = round(f.clearance_t, 1e-6);
    s.plan_least_clearance_m = round(f.plan_clearance_m, 1e-6);
    rec.steps.push_back(s);
  }
  for (PhysicsStep& s : rec.steps) s.counted = StepCounts(rec, s);
  rec.counted_s = CountedSeconds(rec);
  out.ok = true;
  return out;
}

PhysicsResult RunPhysics(const Layout& layout, const Plan& plan,
                         const ModelPaths& paths, const PhysicsOptions& options,
                         const std::string& layout_sha256,
                         const std::string& plan_sha256) {
  std::string error;
  std::unique_ptr<PhysicsRun> run = PhysicsRun::Create(layout, plan, paths, options, &error);
  if (!run) {
    PhysicsResult r;
    r.error = error;
    return r;
  }
  PhysicsResult r = run->Finish();
  r.record.layout_sha256 = layout_sha256;
  r.record.plan_sha256 = plan_sha256;
  r.record.command = "bazel run --config=opt //:run_physics -- " + layout.name + " --write";
  return r;
}

std::string PhysicsText(const PhysicsResult& r) {
  if (!r.ok) return "run_physics: " + r.error + "\n";
  std::string t;
  const PhysicsRecord& rec = r.record;
  t += Fmt("layout %s, arm %s: module %d (module_take %d), steps %d and %d\n",
           r.layout.c_str(), r.arm.c_str(), r.module, r.take, r.take_step, r.set_step);
  t += Fmt("plan %s, layout %s\n", rec.plan_sha256.substr(0, 12).c_str(),
           rec.layout_sha256.substr(0, 12).c_str());
  t += Fmt("grip: tool tip %.4f mm from the glass\n", r.grip_offset_mm);
  t += "\nsegments (plan time; errors against the plan; clearance in physics and in the plan)\n";
  t += Fmt("%5s %-9s %10s %10s %9s %9s %9s %9s %10s %10s  %s\n", "step", "kind", "t0_s",
           "t1_s", "joint_mrad", "track_mm", "frame_mm", "frame_deg", "phys_m", "plan_m",
           "least pair in physics");
  auto row = [&](const PhysicsFigures& f, const char* kind) {
    return Fmt("%5d %-9s %10.3f %10.3f %9.4f %9.4f %9.4f %9.4f %10.6f %10.6f  %s%s\n", f.step,
               kind, f.t0, f.t1, f.joint_error_mrad, f.track_error_mm, f.frame_error_mm,
               f.frame_error_deg, f.clearance_m, f.plan_clearance_m, f.clearance_pair.c_str(),
               f.frame_is_module ? " [module frame]" : "");
  };
  for (const PhysicsFigures& f : r.segments) t += row(f, f.kind.c_str());
  t += row(r.settle, "settle");
  t += "\npeak torque over its limit (joints 1-6, track)\n";
  for (const PhysicsFigures& f : r.steps) {
    t += Fmt("  step %d:", f.step);
    for (int j = 0; j < 7; ++j) t += Fmt(" %.3f", f.torque_ratio[j]);
    t += "\n";
  }
  t += "\nsteps\n";
  for (size_t i = 0; i < rec.steps.size(); ++i) {
    const PhysicsStep& s = rec.steps[i];
    t += Fmt("  step %d (%s, %s): %.6f to %.6f s, planned %.6f s\n", s.number, s.what.c_str(),
             s.arm.c_str(), s.start_s, s.end_s, s.planned_s);
    t += Fmt("    largest joint tracking error %.4f mrad, track %.4f mm\n",
             s.max_joint_error_mrad, r.steps[i].track_error_mm);
    t += Fmt("    largest Cartesian error %.4f mm, %.4f deg\n", s.max_frame_error_mm,
             s.max_frame_error_deg);
    t += Fmt("    least clearance in physics %.6f m at %.3f s (%s); the plan's %.6f m; "
             "against %.3f m\n",
             s.least_clearance_m, s.least_clearance_t, s.least_clearance_pair.c_str(),
             s.plan_least_clearance_m, rec.clearance_m);
  }
  t += Fmt("\nset-down after release and settling: offset %.4f mm (across %.4f, along %.4f), "
           "yaw %.4f deg, height %.4f mm; the gate %.1f mm and %.2f deg\n",
           rec.set_down.offset_mm, rec.set_down.across_mm, rec.set_down.along_mm,
           rec.set_down.yaw_deg, rec.set_down.height_mm, rec.gate_offset_mm, rec.gate_yaw_deg);
  t += "\nthe rule: a step counts when its least clearance in physics is at least clearance_m "
       "and the module ends within the gate\n";
  for (const PhysicsStep& s : rec.steps) {
    t += Fmt("  step %d: %s\n", s.number, s.counted ? "counts" : "does not count");
  }
  t += Fmt("counted %.6f s of cycle_s %.6f s: physics_share %.6f\n", rec.counted_s, r.cycle_s,
           r.cycle_s > 0.0 ? rec.counted_s / r.cycle_s : 0.0);
  return t;
}

}  // namespace cell::layout
