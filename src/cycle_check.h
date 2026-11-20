// Watching a pick-and-place cycle from the truth: what touched what, how
// close things came, how hard the motors worked, and whether the state
// stayed sane. For tests and for the runner's report, never for control
// code: it reads every body's pose and every contact in the simulation.
//
// Call Observe once after every Cell::Step, with the script's current step
// and the module it is picking (src/pick_place.h).
//
// Contacts. Every contact in mjData::contact is sorted by the two bodies
// it is between. Contacts among things the cycle does not touch (the stack
// standing on the pallet, modules lying on the tube) are none of its
// business and are ignored. The rest are allowed only where the cycle means
// them to be:
//   - the tool's cups on the picked module, from touching the glass until
//     the tool has retreated;
//   - the picked module against its neighbour in the stack or the pallet's
//     deck, while it is still in the stack: until the pull-back is done;
//   - the picked module on the tube's rails, from the set-down on.
// Anything else touching the arm, the tool or the picked module is
// forbidden and recorded.
//
// Clearances. Every `clearance_every` steps, the smallest distance between
// a few groups of geoms (mj_geomDistance, up to 1 m) is measured and the
// smallest seen kept. Pairs that are meant to touch at that moment (as
// above) are left out.

#ifndef SRC_CYCLE_CHECK_H_
#define SRC_CYCLE_CHECK_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/cell.h"
#include "src/pick_place.h"

namespace cell {

// One kind of clearance kept by CycleCheck.
enum class Clearance {
  kModuleArm,       // the held module to the arm's links (not the tool)
  kModuleStack,     // the held module to the pallet and the modules on it,
                    // once it has been pulled back
  kModuleConveyor,  // the held module to the conveyor's bed, tube and
                    // rails, until the set-down
  kModulePost,      // the held module to the camera's post, boom, housing
  kModuleFloor,     // the held module to the floor
  kArmStack,        // the arm and tool to the pallet and the modules on it
                    // other than the one being picked
  kArmConveyor,     // the arm and tool to the conveyor, its tube and rails
                    // and the modules lying there
  kArmPost,         // the arm and tool to the camera's post
  kArmFloor,        // the arm's moving links and tool to the floor
  kCount,
};

const char* ClearanceName(Clearance clearance);

struct ClosestApproach {
  double distance = 1.0;  // m; 1.0 means nothing came within 1 m
  PickPlaceStep step = PickPlaceStep::kIdle;  // where it happened
  std::string between;  // "geom - geom"
  bool measured = false;  // the group was watched at least once
};

struct ForbiddenContact {
  double time = 0.0;
  PickPlaceStep step = PickPlaceStep::kIdle;
  std::string between;  // "body - body"
};

struct CycleCheckOptions {
  int clearance_every = 10;  // steps; 0 turns the clearances off
  // How far past a joint's range counts as out of range. MuJoCo's limits
  // are soft, so a joint pressed against one goes a hair past it.
  double range_slack = 1e-3;  // rad
};

// How far an estimate of a module's pose is from where the module truly
// is, in the TRUE module's own frame: its centre's offset along the long
// side (x), up the lean (y) and along the glass's normal (z), the angle
// between the two glass normals, and the turn about the normal (the long
// sides' angle seen square to the glass). Truth: for tests and the runner.
struct EstimateError {
  double long_side = 0.0;  // m
  double up_lean = 0.0;    // m
  double normal = 0.0;     // m, + toward the arm
  double face_angle = 0.0;  // rad
  double in_plane = 0.0;    // rad, counterclockwise seen from the arm
};
EstimateError EstimateErrorOf(const Eigen::Isometry3d& estimate,
                              const Eigen::Isometry3d& truth);

class CycleCheck {
 public:
  explicit CycleCheck(const Cell& cell, const CycleCheckOptions& options = {});

  // Looks at the state after a Cell::Step. `module` is the module being
  // picked in this cycle (-1 if none).
  void Observe(PickPlaceStep step, int module);

  // Contacts that should not have happened, first ones first (at most 50
  // are kept; forbidden_count() counts them all, one per pair per step).
  const std::vector<ForbiddenContact>& forbidden() const { return forbidden_; }
  long forbidden_count() const { return forbidden_count_; }

  const ClosestApproach& closest(Clearance clearance) const {
    return closest_[static_cast<int>(clearance)];
  }
  // The same, kept separately for each step of the cycle.
  const ClosestApproach& closest(Clearance clearance,
                                 PickPlaceStep step) const {
    return closest_by_step_[static_cast<int>(clearance)]
                           [static_cast<int>(step)];
  }

  // The largest |motor force| / limit seen for each joint: overall, and
  // while the suction held a module. Saturated means at the limit.
  const Eigen::VectorXd& peak_torque() const { return peak_torque_; }
  const Eigen::VectorXd& peak_torque_holding() const {
    return peak_torque_holding_;
  }
  // The same in one step of the cycle.
  Eigen::VectorXd peak_torque(PickPlaceStep step) const {
    return peak_torque_by_step_.row(static_cast<int>(step)).transpose();
  }
  long saturated_steps() const { return saturated_steps_; }

  // Joints seen outside their range (beyond range_slack), in steps.
  long out_of_range_steps() const { return out_of_range_steps_; }
  // The smallest margin to a range end seen, per joint, rad.
  const Eigen::VectorXd& smallest_margin() const { return smallest_margin_; }

  // Steps where qpos, qvel or ctrl held a value that is not finite.
  long non_finite_steps() const { return non_finite_steps_; }

  // Everything above in a few lines, millimetres and degrees. With
  // `by_step`, also the closest approach of each kind in each step.
  std::string Report(bool by_step = false) const;

 private:
  // Role of a body in the current step.
  enum class Role {
    kArm,         // the arm's links, base included
    kTool,        // the suction tool
    kTarget,      // the module being picked
    kStack,       // the pallet, or another module standing on it
    kOnTube,      // the tube with its rails, or a module let go (on it)
    kBed,         // the conveyor's beams, rollers and legs
    kPost,        // the camera's post and everything on it
    kFloor,       // the world: the floor
    kOther,
  };
  Role RoleOf(int body, int module) const;
  bool Allowed(Role a, Role b, PickPlaceStep step) const;
  void MeasureClearances(PickPlaceStep step, int module);
  std::string GeomName(int geom) const;
  std::string BodyName(int body) const;

  const Cell& cell_;
  CycleCheckOptions options_;
  int tool_ = -1;
  int conveyor_ = -1;
  int tube_ = -1;
  int post_ = -1;
  std::vector<bool> is_arm_;  // per body: under arm_base, tool excluded
  std::vector<int> module_of_body_;  // per body: module index or -1
  long steps_ = 0;

  std::vector<ForbiddenContact> forbidden_;
  long forbidden_count_ = 0;
  ClosestApproach closest_[static_cast<int>(Clearance::kCount)];
  static constexpr int kSteps = static_cast<int>(PickPlaceStep::kFailed) + 1;
  ClosestApproach closest_by_step_[static_cast<int>(Clearance::kCount)]
                                  [kSteps];
  Eigen::VectorXd peak_torque_;
  Eigen::VectorXd peak_torque_holding_;
  Eigen::MatrixXd peak_torque_by_step_;  // kSteps x joints
  long saturated_steps_ = 0;
  long out_of_range_steps_ = 0;
  Eigen::VectorXd smallest_margin_;
  long non_finite_steps_ = 0;
};

}  // namespace cell

#endif  // SRC_CYCLE_CHECK_H_
