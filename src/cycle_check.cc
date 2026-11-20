#include "src/cycle_check.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/cell.h"
#include "src/pick_place.h"

namespace cell {
namespace {

constexpr double kDegree = std::numbers::pi / 180.0;
constexpr int kMaxKept = 50;
constexpr double kFar = 1.0;  // m, the most mj_geomDistance is asked to look

bool Collides(const mjModel& model, int geom) {
  return model.geom_contype[geom] != 0 || model.geom_conaffinity[geom] != 0;
}

// Steps compared by their order in the cycle.
bool Between(PickPlaceStep step, PickPlaceStep first, PickPlaceStep last) {
  return static_cast<int>(step) >= static_cast<int>(first) &&
         static_cast<int>(step) <= static_cast<int>(last);
}

}  // namespace

EstimateError EstimateErrorOf(const Eigen::Isometry3d& estimate,
                              const Eigen::Isometry3d& truth) {
  EstimateError e;
  const Eigen::Vector3d d =
      truth.linear().transpose() * (estimate.translation() - truth.translation());
  e.long_side = d.x();
  e.up_lean = d.y();
  e.normal = d.z();
  const Eigen::Matrix3d r = truth.linear().transpose() * estimate.linear();
  e.face_angle = std::acos(std::clamp(r(2, 2), -1.0, 1.0));
  e.in_plane = std::atan2(r(1, 0), r(0, 0));
  return e;
}

const char* ClearanceName(Clearance clearance) {
  switch (clearance) {
    case Clearance::kModuleArm:
      return "held module - arm links";
    case Clearance::kModuleStack:
      return "held module - pallet and stack (after pull-back)";
    case Clearance::kModuleConveyor:
      return "held module - conveyor, tube, rails (carry)";
    case Clearance::kModulePost:
      return "held module - camera post and boom";
    case Clearance::kModuleFloor:
      return "held module - floor";
    case Clearance::kArmStack:
      return "arm and tool - pallet and other modules";
    case Clearance::kArmConveyor:
      return "arm and tool - conveyor, tube, modules on it";
    case Clearance::kArmPost:
      return "arm and tool - camera post and boom";
    case Clearance::kArmFloor:
      return "arm links and tool - floor";
    case Clearance::kModuleCheckStand:
      return "held module - check camera stand";
    case Clearance::kArmCheckStand:
      return "arm and tool - check camera stand";
    case Clearance::kCount:
      break;
  }
  return "?";
}

CycleCheck::CycleCheck(const Cell& cell, const CycleCheckOptions& options)
    : cell_(cell), options_(options) {
  const mjModel& model = *cell.model();
  tool_ = cell.arm().tool_body;
  conveyor_ = mj_name2id(&model, mjOBJ_BODY, "conveyor");
  tube_ = mj_name2id(&model, mjOBJ_BODY, "tube");
  post_ = mj_name2id(&model, mjOBJ_BODY, "camera_post");
  check_stand_ = mj_name2id(&model, mjOBJ_BODY, "check_camera_stand");
  const int base = mj_name2id(&model, mjOBJ_BODY, "arm_base");
  is_arm_.assign(model.nbody, false);
  for (int b = 0; b < model.nbody; ++b) {
    if (b == tool_) continue;
    for (int up = b; up > 0; up = model.body_parentid[up]) {
      if (up == tool_) break;
      if (up == base) {
        is_arm_[b] = true;
        break;
      }
    }
  }
  module_of_body_.assign(model.nbody, -1);
  for (int i = 0; i < cell.module_count(); ++i) {
    module_of_body_[cell.pallet().modules[i]] = i;
  }
  peak_torque_ = Eigen::VectorXd::Zero(Arm::kJoints);
  peak_torque_holding_ = Eigen::VectorXd::Zero(Arm::kJoints);
  peak_torque_by_step_ = Eigen::MatrixXd::Zero(kSteps, Arm::kJoints);
  smallest_margin_ = Eigen::VectorXd::Constant(Arm::kJoints, 1e9);
}

std::string CycleCheck::GeomName(int geom) const {
  const char* name = mj_id2name(cell_.model(), mjOBJ_GEOM, geom);
  if (name != nullptr) return name;
  return BodyName(cell_.model()->geom_bodyid[geom]) + " geom";
}

std::string CycleCheck::BodyName(int body) const {
  const char* name = mj_id2name(cell_.model(), mjOBJ_BODY, body);
  return name != nullptr ? name : "body " + std::to_string(body);
}

CycleCheck::Role CycleCheck::RoleOf(int body, int module) const {
  if (body == 0) return Role::kFloor;
  if (body == tool_) return Role::kTool;
  if (is_arm_[body]) return Role::kArm;
  if (body == cell_.pallet().pallet) return Role::kStack;
  if (body == tube_) return Role::kOnTube;
  if (body == conveyor_) return Role::kBed;
  if (body == post_) return Role::kPost;
  if (body == check_stand_) return Role::kCheckStand;
  const int i = module_of_body_[body];
  if (i >= 0) {
    // A parked module (carried away with a full tube) lies far off on the
    // floor and is nothing to the cycle any more.
    if (cell_.module_state(i) == ModuleState::kParked) return Role::kOther;
    if (i == module) return Role::kTarget;
    switch (cell_.module_state(i)) {
      case ModuleState::kOnPallet:
        return Role::kStack;
      case ModuleState::kReleased:
        return Role::kOnTube;
      default:
        return Role::kOther;
    }
  }
  return Role::kOther;
}

bool CycleCheck::Allowed(Role a, Role b, PickPlaceStep step) const {
  auto moving = [](Role r) {
    return r == Role::kArm || r == Role::kTool || r == Role::kTarget;
  };
  if (!moving(a) && !moving(b)) return true;  // not the cycle's business
  if (b == Role::kTarget) std::swap(a, b);
  if (a == Role::kTarget) {
    if (b == Role::kTool) {
      return Between(step, PickPlaceStep::kTouchGlass, PickPlaceStep::kRetreat) ||
             step == PickPlaceStep::kFailed;
    }
    if (b == Role::kStack) {
      // Standing in the stack until it has been pulled clear; or left
      // there after a failed grip.
      return static_cast<int>(step) <=
                 static_cast<int>(PickPlaceStep::kPullBack) ||
             step == PickPlaceStep::kFailed;
    }
    if (b == Role::kOnTube) {
      return Between(step, PickPlaceStep::kSetDown, PickPlaceStep::kDone);
    }
  }
  return false;
}

void CycleCheck::Observe(PickPlaceStep step, int module) {
  const mjModel& model = *cell_.model();
  const mjData& data = *cell_.data();
  const Arm& arm = cell_.arm();
  ++steps_;

  // Contacts.
  for (int c = 0; c < data.ncon; ++c) {
    const mjContact& contact = data.contact[c];
    // A contact MuJoCo found but holds no force for (beyond the margin) is
    // not a touch.
    if (contact.dist > 0.0) continue;
    const int a = model.geom_bodyid[contact.geom[0]];
    const int b = model.geom_bodyid[contact.geom[1]];
    if (Allowed(RoleOf(a, module), RoleOf(b, module), step)) continue;
    ++forbidden_count_;
    if (static_cast<int>(forbidden_.size()) < kMaxKept) {
      forbidden_.push_back({data.time, step,
                            GeomName(contact.geom[0]) + " (" + BodyName(a) +
                                ") - " + GeomName(contact.geom[1]) + " (" +
                                BodyName(b) + ")"});
    }
  }

  // Motors and joints.
  const bool holding = cell_.suction().holding().has_value();
  bool saturated = false;
  bool out_of_range = false;
  for (int j = 0; j < Arm::kJoints; ++j) {
    const double fraction =
        std::abs(data.actuator_force[arm.motor[j]]) / arm.torque_limit[j];
    peak_torque_[j] = std::max(peak_torque_[j], fraction);
    double& in_step = peak_torque_by_step_(static_cast<int>(step), j);
    in_step = std::max(in_step, fraction);
    if (holding) {
      peak_torque_holding_[j] = std::max(peak_torque_holding_[j], fraction);
    }
    saturated = saturated || fraction >= 1.0 - 1e-9;
    const double q = data.qpos[arm.qpos[j]];
    const double margin = std::min(q - arm.lower[j], arm.upper[j] - q);
    smallest_margin_[j] = std::min(smallest_margin_[j], margin);
    out_of_range = out_of_range || margin < -options_.range_slack;
  }
  saturated_steps_ += saturated;
  out_of_range_steps_ += out_of_range;

  bool finite = true;
  for (int i = 0; i < model.nq; ++i) finite = finite && std::isfinite(data.qpos[i]);
  for (int i = 0; i < model.nv; ++i) finite = finite && std::isfinite(data.qvel[i]);
  for (int i = 0; i < model.nu; ++i) finite = finite && std::isfinite(data.ctrl[i]);
  non_finite_steps_ += !finite;

  if (options_.clearance_every > 0 && steps_ % options_.clearance_every == 0 &&
      finite) {
    MeasureClearances(step, module);
  }
}

void CycleCheck::MeasureClearances(PickPlaceStep step, int module) {
  const mjModel& model = *cell_.model();
  const mjData& data = *cell_.data();

  // The colliding geoms of each role, now.
  std::vector<int> arm_geoms, tool_geoms, target_geoms, stack_geoms,
      conveyor_geoms, post_geoms, stand_geoms, floor_geoms;
  for (int g = 0; g < model.ngeom; ++g) {
    if (!Collides(model, g)) continue;
    const int body = model.geom_bodyid[g];
    switch (RoleOf(body, module)) {
      case Role::kArm:
        arm_geoms.push_back(g);
        break;
      case Role::kTool:
        tool_geoms.push_back(g);
        break;
      case Role::kTarget:
        target_geoms.push_back(g);
        break;
      case Role::kStack:
        stack_geoms.push_back(g);
        break;
      case Role::kOnTube:
      case Role::kBed:
        conveyor_geoms.push_back(g);
        break;
      case Role::kPost:
        post_geoms.push_back(g);
        break;
      case Role::kCheckStand:
        stand_geoms.push_back(g);
        break;
      case Role::kFloor:
        floor_geoms.push_back(g);
        break;
      case Role::kOther:
        break;
    }
  }
  std::vector<int> arm_and_tool = arm_geoms;
  arm_and_tool.insert(arm_and_tool.end(), tool_geoms.begin(), tool_geoms.end());
  // The base stands on the floor and link 1 turns just above it; neither
  // tells anything about the floor clearance of a move.
  std::vector<int> moving_arm;
  const int base = mj_name2id(&model, mjOBJ_BODY, "arm_base");
  const int link_1 = mj_name2id(&model, mjOBJ_BODY, "arm_link_1");
  for (int g : arm_and_tool) {
    const int body = model.geom_bodyid[g];
    if (body != base && body != link_1) moving_arm.push_back(g);
  }

  auto measure = [&](Clearance which, const std::vector<int>& first,
                     const std::vector<int>& second) {
    ClosestApproach& best = closest_[static_cast<int>(which)];
    ClosestApproach& in_step =
        closest_by_step_[static_cast<int>(which)][static_cast<int>(step)];
    best.measured = true;
    in_step.measured = true;
    for (int g : first) {
      for (int h : second) {
        // Bounding spheres first: a pair whose spheres are further apart
        // than the best so far cannot beat it. Planes have no sphere.
        if (model.geom_type[g] != mjGEOM_PLANE &&
            model.geom_type[h] != mjGEOM_PLANE) {
          const Eigen::Vector3d between =
              Eigen::Map<const Eigen::Vector3d>(data.geom_xpos + 3 * g) -
              Eigen::Map<const Eigen::Vector3d>(data.geom_xpos + 3 * h);
          if (between.norm() - model.geom_rbound[g] - model.geom_rbound[h] >
              std::min(in_step.distance, kFar)) {
            continue;
          }
        }
        // mj_geomDistance takes a writable mjData only for its scratch
        // stack; it leaves the state as it was.
        const double d = mj_geomDistance(
            cell_.model(), const_cast<mjData*>(cell_.data()), g, h, kFar,
            nullptr);
        if (d < in_step.distance) {
          in_step.distance = d;
          in_step.step = step;
          in_step.between = GeomName(g) + " - " + GeomName(h);
        }
        if (d < best.distance) best = in_step;
      }
    }
  };

  const bool held = cell_.suction().holding().has_value() &&
                    !target_geoms.empty();
  if (held) {
    measure(Clearance::kModuleArm, target_geoms, arm_geoms);
    measure(Clearance::kModulePost, target_geoms, post_geoms);
    measure(Clearance::kModuleCheckStand, target_geoms, stand_geoms);
    measure(Clearance::kModuleFloor, target_geoms, floor_geoms);
    if (static_cast<int>(step) >= static_cast<int>(PickPlaceStep::kClear)) {
      measure(Clearance::kModuleStack, target_geoms, stack_geoms);
    }
    if (static_cast<int>(step) <=
        static_cast<int>(PickPlaceStep::kWaitForConveyor)) {
      measure(Clearance::kModuleConveyor, target_geoms, conveyor_geoms);
    }
  }
  measure(Clearance::kArmStack, arm_and_tool, stack_geoms);
  measure(Clearance::kArmConveyor, arm_and_tool, conveyor_geoms);
  measure(Clearance::kArmPost, arm_and_tool, post_geoms);
  measure(Clearance::kArmCheckStand, arm_and_tool, stand_geoms);
  measure(Clearance::kArmFloor, moving_arm, floor_geoms);
}

std::string CycleCheck::Report(bool by_step) const {
  std::string text;
  char line[400];
  std::snprintf(line, sizeof(line),
                "forbidden contacts: %ld (pair-steps)%s\n", forbidden_count_,
                forbidden_.empty() ? "" : ", first:");
  text += line;
  for (int i = 0; i < std::min<int>(5, forbidden_.size()); ++i) {
    std::snprintf(line, sizeof(line), "  t %.3f s, %s: %s\n",
                  forbidden_[i].time, StepName(forbidden_[i].step),
                  forbidden_[i].between.c_str());
    text += line;
  }
  text += "closest approaches:\n";
  for (int c = 0; c < static_cast<int>(Clearance::kCount); ++c) {
    const ClosestApproach& a = closest_[c];
    if (!a.measured) continue;
    std::snprintf(line, sizeof(line), "  %-48s %7.1f mm  in %s (%s)\n",
                  ClearanceName(static_cast<Clearance>(c)), 1e3 * a.distance,
                  StepName(a.step), a.between.c_str());
    text += line;
    if (!by_step) continue;
    for (int s = 0; s < kSteps; ++s) {
      const ClosestApproach& b = closest_by_step_[c][s];
      if (!b.measured || b.distance >= kFar) continue;
      std::snprintf(line, sizeof(line), "      %-14s %7.1f mm  (%s)\n",
                    StepName(static_cast<PickPlaceStep>(s)), 1e3 * b.distance,
                    b.between.c_str());
      text += line;
    }
  }
  auto row = [&](const char* label, const Eigen::VectorXd& v, double scale) {
    std::snprintf(line, sizeof(line), "%-26s %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
                  label, scale * v[0], scale * v[1], scale * v[2], scale * v[3],
                  scale * v[4], scale * v[5]);
    text += line;
  };
  row("peak torque / limit", peak_torque_, 1.0);
  row("  with a module held", peak_torque_holding_, 1.0);
  if (by_step) {
    for (int s = 0; s < kSteps; ++s) {
      const Eigen::VectorXd p = peak_torque_by_step_.row(s).transpose();
      if (p.maxCoeff() <= 0.0) continue;
      const std::string label =
          std::string("    in ") + StepName(static_cast<PickPlaceStep>(s));
      row(label.c_str(), p, 1.0);
    }
  }
  std::snprintf(line, sizeof(line),
                "smallest range margin, deg %6.1f %6.1f %6.1f %6.1f %6.1f "
                "%6.1f\n",
                smallest_margin_[0] / kDegree, smallest_margin_[1] / kDegree,
                smallest_margin_[2] / kDegree, smallest_margin_[3] / kDegree,
                smallest_margin_[4] / kDegree, smallest_margin_[5] / kDegree);
  text += line;
  std::snprintf(line, sizeof(line),
                "saturated steps %ld, out-of-range steps %ld, non-finite "
                "steps %ld\n",
                saturated_steps_, out_of_range_steps_, non_finite_steps_);
  text += line;
  return text;
}

}  // namespace cell
