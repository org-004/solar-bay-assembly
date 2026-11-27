// The torque a timed move of the module arm demands. See src/layout_torque.h.

#include "src/layout_torque.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace cell::layout {

double TorqueDemand::Worst() const { return *std::max_element(ratio.begin(), ratio.end()); }

int TorqueDemand::WorstAxis() const {
  return static_cast<int>(std::max_element(ratio.begin(), ratio.end()) - ratio.begin());
}

std::string ModuleArm(const Layout& layout) {
  const std::vector<std::string> arms = ArmsDoing(layout, {"module_set"});
  return arms.empty() ? std::string() : arms.front();
}

std::optional<Trajectory> MoveTrajectory(const std::vector<Config>& path,
                                         const ArmMotion& motion, const PlanFigures& f) {
  if (path.empty()) return std::nullopt;
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.reserve(path.size());
  for (const Config& c : path) {
    Eigen::VectorXd v(7);
    for (int j = 0; j < 7; ++j) v[j] = c[j];
    waypoints.push_back(v);
  }
  PathTimingOptions options;
  options.max_deviation = f.max_deviation;
  return Trajectory::Plan(waypoints, motion.limits, options);
}

namespace {

void Widen(TorqueDemand* into, const std::array<double, 7>& r) {
  for (int j = 0; j < 7; ++j) into->ratio[j] = std::max(into->ratio[j], r[j]);
}

Config FromVector(const Eigen::VectorXd& v, double factor) {
  Config c{};
  for (int j = 0; j < 7; ++j) c[j] = v[j] * factor;
  return c;
}

}  // namespace

ArmDynamics::~ArmDynamics() = default;

std::unique_ptr<ArmDynamics> ArmDynamics::Create(const Layout& layout, const ModelPaths& paths,
                                                 const std::string& arm, std::string* error) {
  std::unique_ptr<ArmDynamics> d(new ArmDynamics());
  d->layout_ = &layout;
  d->paths_ = paths;
  d->arm_ = arm;
  const ArmUse* use = nullptr;
  for (const ArmUse& a : layout.arms) {
    if (a.name == arm) use = &a;
  }
  if (use == nullptr) {
    if (error != nullptr) *error = "no arm \"" + arm + "\"";
    return nullptr;
  }
  if (!use->track) {
    if (error != nullptr) *error = "arm " + arm + " has no track";
    return nullptr;
  }
  // The limits from the model with the arm's first tool.
  Model* m = d->Get(use->first_tool, "", error);
  if (m == nullptr) return nullptr;
  const mjModel& model = *m->model;
  for (int j = 0; j < 7; ++j) {
    d->limits_[j] = 0.0;
    for (int a = 0; a < model.nu; ++a) {
      if (model.actuator_trntype[a] != mjTRN_JOINT) continue;
      const int joint = model.actuator_trnid[2 * a];
      if (model.jnt_dofadr[joint] != m->dof[j]) continue;
      d->limits_[j] = model.actuator_forcerange[2 * a + 1] * std::abs(model.actuator_gear[6 * a]);
    }
    if (!(d->limits_[j] > 0.0)) {
      if (error != nullptr) *error = "arm " + arm + ": an axis has no motor with a force limit";
      return nullptr;
    }
  }
  return d;
}

ArmDynamics::Model* ArmDynamics::Get(const std::string& tool, const std::string& held,
                                     std::string* error) {
  const auto key = std::make_pair(tool, held);
  auto it = models_.find(key);
  if (it != models_.end()) return &it->second;
  LayoutState state;
  if (!layout_->pallets.empty()) state.pallet_modules = layout_->pallets[0].modules;
  state.physics_arm = arm_;
  state.carried_take = 0;
  state.tool[arm_] = tool;
  state.held[arm_] = held;
  std::string why;
  Model m;
  m.model = LoadLayoutModel(*layout_, state, paths_, &why);
  if (!m.model) {
    if (error != nullptr) *error = "the physics model: " + why;
    return nullptr;
  }
  m.model->opt.disableflags |= mjDSBL_CONSTRAINT | mjDSBL_CONTACT;
  m.data = MakeData(*m.model);
  mj_resetDataKeyframe(m.model.get(), m.data.get(), 0);
  const LayoutHandles h = FindHandles(*m.model, *layout_);
  bool found = false;
  for (const ArmHandle& a : h.arms) {
    if (a.name != arm_) continue;
    found = true;
    for (int j = 0; j < 6; ++j) {
      m.qpos[j] = a.qpos[j];
      m.dof[j] = a.dof[j];
    }
    m.qpos[6] = a.track_qpos;
    m.dof[6] = a.track_dof;
  }
  if (!found || m.qpos[6] < 0) {
    if (error != nullptr) *error = "the physics model has no arm " + arm_ + " on a track";
    return nullptr;
  }
  return &models_.emplace(key, std::move(m)).first->second;
}

bool ArmDynamics::Ratio(const std::string& tool, const std::string& held, const Config& q,
                        const Config& v, const Config& a, std::array<double, 7>* ratio,
                        std::string* error) {
  Model* m = Get(tool, held, error);
  if (m == nullptr) return false;
  const mjModel* model = m->model.get();
  mjData* d = m->data.get();
  mju_zero(d->qvel, model->nv);
  mju_zero(d->qacc, model->nv);
  for (int j = 0; j < 7; ++j) {
    d->qpos[m->qpos[j]] = q[j];
    d->qvel[m->dof[j]] = v[j];
    d->qacc[m->dof[j]] = a[j];
  }
  mj_inverse(model, d);
  for (int j = 0; j < 7; ++j) {
    (*ratio)[j] = std::abs(d->qfrc_inverse[m->dof[j]]) / limits_[j];
  }
  return true;
}

bool ArmDynamics::AlongMove(const Trajectory& trajectory, double time_scale,
                            const std::string& tool, const std::string& held,
                            TorqueDemand* demand, std::string* error) {
  *demand = TorqueDemand{};
  const double total = trajectory.duration();
  const int n = std::max(1, static_cast<int>(std::ceil(total / kTorqueSampleS)));
  for (int i = 0; i <= n; ++i) {
    const double t = std::min(total, static_cast<double>(i) * kTorqueSampleS);
    const JointSample s = trajectory.At(t);
    std::array<double, 7> r{};
    if (!Ratio(tool, held, FromVector(s.position, 1.0), FromVector(s.velocity, 1.0 / time_scale),
               FromVector(s.acceleration, 1.0 / (time_scale * time_scale)), &r, error)) {
      return false;
    }
    Widen(demand, r);
  }
  return true;
}

bool ArmDynamics::AlongSegment(const Segment& segment, const ArmMotion& motion,
                               const PlanFigures& f, TorqueDemand* demand, std::string* error) {
  *demand = TorqueDemand{};
  if (segment.path.empty()) return true;
  if (segment.kind == "move") {
    const std::optional<Trajectory> t = MoveTrajectory(segment.path, motion, f);
    if (!t) {
      if (error != nullptr) *error = "a move that cannot be timed";
      return false;
    }
    return AlongMove(*t, segment.time_scale, segment.tool, segment.held, demand, error);
  }
  const Config zero{};
  if (segment.kind != "approach" && segment.kind != "retreat") {
    std::array<double, 7> r{};
    if (!Ratio(segment.tool, segment.held, segment.path.front(), zero, zero, &r, error)) {
      return false;
    }
    Widen(demand, r);
    return true;
  }
  // A leg: central differences of the plan's own law, as the physics run.
  const double h = kTorqueDifference;
  const double span = segment.t1 - segment.t0;
  const int n = std::max(1, static_cast<int>(std::ceil(span / kTorqueSampleS)));
  for (int i = 0; i <= n; ++i) {
    const double t = segment.t0 + std::min(span, static_cast<double>(i) * kTorqueSampleS);
    const Config c0 = ConfigAt(segment, motion, f, t - h);
    const Config c1 = ConfigAt(segment, motion, f, t);
    const Config c2 = ConfigAt(segment, motion, f, t + h);
    Config v{}, a{};
    for (int j = 0; j < 7; ++j) {
      v[j] = (c2[j] - c0[j]) / (2.0 * h);
      a[j] = (c2[j] - 2.0 * c1[j] + c0[j]) / (h * h);
    }
    std::array<double, 7> r{};
    if (!Ratio(segment.tool, segment.held, c1, v, a, &r, error)) return false;
    Widen(demand, r);
  }
  return true;
}

double TorqueTimeScale(ArmDynamics& dynamics, const Trajectory& trajectory,
                       const std::string& tool, const std::string& held, double fraction,
                       TorqueDemand* demand, std::string* error) {
  double scale = 1.0;
  for (int round = 0; round <= kMaxTorqueRounds; ++round) {
    if (!dynamics.AlongMove(trajectory, scale, tool, held, demand, error)) return -1.0;
    const double worst = demand->Worst() / fraction;
    if (worst <= 1.0) return scale;
    const double grown = scale * std::sqrt(worst);
    scale = std::ceil(grown / kTorqueScaleStep) * kTorqueScaleStep;
  }
  if (error != nullptr) {
    *error = "the move demands more than the fraction of a limit after " +
             std::to_string(kMaxTorqueRounds) + " slowings";
  }
  return -1.0;
}

}  // namespace cell::layout
