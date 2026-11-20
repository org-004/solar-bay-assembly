#include "src/pick_place.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <numbers>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/arm_follower.h"
#include "src/camera_model.h"
#include "src/cell.h"
#include "src/collision.h"
#include "src/conveyor.h"
#include "src/depth_camera.h"
#include "src/held_module_finder.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/module_finder.h"
#include "src/motion_planner.h"
#include "src/pallet.h"
#include "src/suction.h"

// What this file may read, and does: the design poses in PalletBodies
// (nominal_pallet, nominal_modules), the conveyor's fixed layout and its
// encoder (Conveyor::PlacementPose, SlotPose, SensorPosition, Moving,
// steps), the arm's keyframes, the joint angles through the follower, and
// the camera: Cell::TakeDepthImage (the picture) and Cell::CalibratedCamera
// (where the calibration says the camera is). Tool poses come from the IK's TipPoseAt on the scratch mjData,
// never from the simulated one. Nothing here reads a module's or the
// pallet's pose out of the simulation, or calls a True... accessor. The
// planned carry's scene is built from the same: the estimate (or the
// design), what the script did itself, ParkingPose (where the cell carries
// a swapped tube's modules) and the encoder; the planner's own model
// supplies only what is fixed by design.

namespace cell {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;

Eigen::VectorXd Degrees(double j1, double j2, double j3, double j4, double j5,
                        double j6) {
  Eigen::VectorXd q(Arm::kJoints);
  q << j1, j2, j3, j4, j5, j6;
  return q * kDegree;
}

// Seeds for the two far solves. The IK walks from its seed to the nearest
// answer, so the seed picks the branch: elbow up, and the wrist with joint 4
// near 0.
//
// At the pallet: square to the glass, the tool's X along the module's +x
// (world -Y), joint 5 bent up (negative). This is the answer the IK finds
// for module_0's face with joint 4 = 0 (tests/ik_test.cc, CellPoses:
// -74.9, 128.7, 0, -43.8, 90 degrees), whose straight pull-back keeps every
// joint at least 15 degrees inside its range for every module. Joint 1 is
// turned toward the target before solving.
const Eigen::VectorXd kPickSeed = Degrees(0, -78, 132, 0, -44, 90);

// Over the conveyor: tool pointing straight down, its X along world +X, so
// the module's +x lies along +X as Conveyor::PlacementPose has it. Joint 1
// at -180 degrees: turned half a turn counterclockwise seen from above, the
// way round over +Y. +180 would be the same pose reached round -Y, past the
// camera's post. Both are 5 degrees inside the 185-degree limit; the script
// always uses -180 and so always swings the same way.
const Eigen::VectorXd kPlaceSeed = Degrees(-180, -65, 85, 0, 70, 0);

// Flat over the pallet, tool straight down: the answer for the nominal
// pallet (tests/pick_place_test.cc's plan), joint 4 at 0 and joint 5 bent
// down. The flatten turns the wrist from bent up (joint 5 about -30
// degrees, at the clear pose) to bent down through joint 5 = 0, the wrist's
// singular line. Seeded from the clear pose, the IK must cross it, and with
// the pallet turned a degree or two (the tool's X then a little off the
// arm's plane, joint 4 no longer exactly 0) it can come out on the other
// side with joint 4 at 180 and joint 5 bent up the other way: the same tool
// pose, but a joint move there swings the wrist half a turn with the module
// on it. Seeding from this pose keeps the answer on the intended branch.
const Eigen::VectorXd kOverPalletSeed = Degrees(0, -90.6, 80.9, 0, 99.8, 90);

// Joint 1 must be at least this far round (negative) at the slot, or the IK
// found the other way round.
constexpr double kSwungJoint1 = -0.5 * kPi;

// The arm counts as at the look pose with every joint within this of it.
constexpr double kAtLookPose = 0.01;  // rad

// The joint-4 = 0 wrist at the pallet: joint 4 within this of 0.
constexpr double kWristBranch = 45.0 * kDegree;

Eigen::Isometry3d FromWorldPose(const WorldPose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = pose.position;
  result.linear() = pose.orientation.normalized().toRotationMatrix();
  return result;
}

// The pose a fraction t of the way from `from` to `to`: straight line for
// the position, the shorter way round for the orientation.
Eigen::Isometry3d Interpolate(const Eigen::Isometry3d& from,
                              const Eigen::Isometry3d& to, double t) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
      (1.0 - t) * from.translation() + t * to.translation();
  const Eigen::Quaterniond a(from.linear());
  const Eigen::Quaterniond b(to.linear());
  pose.linear() = a.slerp(t, b).toRotationMatrix();
  return pose;
}

// `pose` turned about the vertical through its own origin so that its X,
// seen from above, lies square to the line from the arm's base axis (the
// world's Z axis) out to it, keeping X's sense. A tool pose so turned has
// its X along joint 5's axis with joint 4 at zero, the arm "facing" it.
Eigen::Isometry3d SquaredToArm(const Eigen::Isometry3d& pose) {
  const Eigen::Vector2d bearing = pose.translation().head<2>().normalized();
  Eigen::Vector2d x = pose.linear().col(0).head<2>();
  Eigen::Vector2d square(-bearing.y(), bearing.x());
  if (square.dot(x) < 0.0) square = -square;
  const double angle = std::atan2(x.x() * square.y() - x.y() * square.x(),
                                  x.dot(square));
  Eigen::Isometry3d turned = pose;
  turned.linear() =
      Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
      pose.linear();
  return turned;
}

// `pose` moved by `offset`, world axes; orientation unchanged.
Eigen::Isometry3d Shifted(const Eigen::Isometry3d& pose,
                          const Eigen::Vector3d& offset) {
  Eigen::Isometry3d moved = pose;
  moved.translation() += offset;
  return moved;
}

const char* IkStatusName(IkStatus status) {
  switch (status) {
    case IkStatus::kConverged:
      return "converged";
    case IkStatus::kBadInput:
      return "bad input";
    case IkStatus::kIterationLimit:
      return "iteration limit";
    case IkStatus::kStalled:
      return "stalled";
    case IkStatus::kTooFarFromSeed:
      return "jumped branch";
  }
  return "?";
}

std::string Describe(const IkReport& report) {
  char text[160];
  std::snprintf(text, sizeof(text), "%s, %.1f mm and %.2f deg short",
                IkStatusName(report.status), 1e3 * report.position_error,
                report.orientation_error / kDegree);
  return text;
}

}  // namespace

const char* StepName(PickPlaceStep step) {
  switch (step) {
    case PickPlaceStep::kIdle:
      return "idle";
    case PickPlaceStep::kLook:
      return "look";
    case PickPlaceStep::kToStandoff:
      return "to standoff";
    case PickPlaceStep::kApproach:
      return "approach";
    case PickPlaceStep::kTouchGlass:
      return "touch glass";
    case PickPlaceStep::kGrip:
      return "grip";
    case PickPlaceStep::kLift:
      return "lift";
    case PickPlaceStep::kPullBack:
      return "pull back";
    case PickPlaceStep::kClear:
      return "clear";
    case PickPlaceStep::kFlatten:
      return "flatten";
    case PickPlaceStep::kSwing:
      return "swing";
    case PickPlaceStep::kTurn:
      return "turn";
    case PickPlaceStep::kCarry:
      return "carry";
    case PickPlaceStep::kWaitForConveyor:
      return "wait conveyor";
    case PickPlaceStep::kLower:
      return "lower";
    case PickPlaceStep::kSetDown:
      return "set down";
    case PickPlaceStep::kRelease:
      return "release";
    case PickPlaceStep::kRetreat:
      return "retreat";
    case PickPlaceStep::kConveyor:
      return "conveyor";
    case PickPlaceStep::kLookAhead:
      return "look ahead";
    case PickPlaceStep::kReturn:
      return "return";
    case PickPlaceStep::kDone:
      return "done";
    case PickPlaceStep::kFailed:
      return "failed";
  }
  return "?";
}

const char* CarryModeName(CarryMode mode) {
  switch (mode) {
    case CarryMode::kScripted:
      return "scripted";
    case CarryMode::kPlanned:
      return "planned";
  }
  return "?";
}

const char* CheckStatusName(CheckReport::Status status) {
  switch (status) {
    case CheckReport::Status::kOff:
      return "off";
    case CheckReport::Status::kNotRun:
      return "not run";
    case CheckReport::Status::kChecked:
      return "checked";
    case CheckReport::Status::kRefused:
      return "refused";
    case CheckReport::Status::kSlipped:
      return "slipped";
  }
  return "?";
}

PlanScene CarryScene(const PalletBodies& design, const CarrySceneInput& input) {
  PlanScene scene;
  scene.pallet = input.pallet_shift * FromWorldPose(design.nominal_pallet);
  const int count = static_cast<int>(design.nominal_modules.size());
  for (int i = 0; i < count; ++i) {
    if (i == input.held) continue;
    const KnownModule known = i < static_cast<int>(input.modules.size())
                                  ? input.modules[i]
                                  : KnownModule{};
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    switch (known.place) {
      case ModulePlace::kOnTube:
        // Lying in its slot, which the tube has carried along since.
        pose = Conveyor::SlotPose(known.slot, input.tube_slide);
        break;
      case ModulePlace::kParked:
      case ModulePlace::kUnaccounted:
        pose = ParkingPose(i);
        break;
      case ModulePlace::kLetGo:
        // Where the tool let it go, carried along by the tube since.
        pose = Conveyor::SlotPose(known.slot, input.tube_slide) *
               Conveyor::SlotPose(known.slot, known.slide).inverse() *
               known.pose;
        break;
      case ModulePlace::kOnStack:
      case ModulePlace::kHeld:  // held once, by a cycle that failed: unknown
        // Behind the module carried: still standing where the estimate puts
        // the stack. In front of it with no record: taken off before this
        // script knew (Cell::ParkModule), so carried away.
        pose = i > input.held ? input.pallet_shift *
                                    FromWorldPose(design.nominal_modules[i])
                              : ParkingPose(i);
        break;
    }
    scene.modules.emplace_back(i, pose);
  }
  scene.tube_slide = input.tube_slide;
  if (input.held >= 0) {
    scene.held_module = input.held;
    scene.module_in_tip =
        input.module_in_tip
            ? *input.module_in_tip
            : DesignModuleInTip(kModuleThickness, input.grip_gap);
  }
  return scene;
}

PickPlace::PickPlace(Cell& cell, const PickPlaceOptions& options)
    : cell_(cell),
      options_(options),
      follower_(cell.arm()),
      scratch_(MakeData(*cell.model())),
      last_time_(cell.data()->time),
      prior_(PriorFromDesign(cell.pallet())),
      look_pose_(cell.arm_clear()),
      known_(cell.module_count()) {
  // arm_clear's joint 1 is +pi, the same pose as -pi; the script keeps to
  // the -pi side, reached round +Y, like the place.
  if (look_pose_[0] > 0.0) look_pose_[0] -= 2.0 * kPi;
}

std::optional<Eigen::VectorXd> PickPlace::SolvePose(
    const Eigen::Isometry3d& target, const Eigen::VectorXd& seed,
    std::string* reason) {
  IkReport report;
  std::optional<Eigen::VectorXd> q = SolveIk(
      *cell_.model(), scratch_.get(), cell_.arm(), target, seed, {}, &report);
  if (!q) *reason = Describe(report);
  return q;
}

std::optional<std::vector<Eigen::VectorXd>> PickPlace::SolveLine(
    const Eigen::VectorXd& start, const Eigen::Isometry3d& from,
    const Eigen::Isometry3d& to, const std::vector<double>& stops,
    std::string* reason) {
  const double length = (to.translation() - from.translation()).norm();
  std::vector<Eigen::VectorXd> kept;
  Eigen::VectorXd q = start;
  double done = 0.0;
  for (double stop : stops) {
    const double t0 = length > 0.0 ? done / length : 0.0;
    const double t1 = length > 0.0 ? std::min(stop / length, 1.0) : 1.0;
    const std::vector<Eigen::Isometry3d> poses =
        StraightLine(Interpolate(from, to, t0), Interpolate(from, to, t1),
                     options_.line_spacing);
    IkPathReport report;
    std::optional<std::vector<Eigen::VectorXd>> path =
        SolveIkPath(*cell_.model(), scratch_.get(), cell_.arm(), q, poses,
                    options_.max_joint_step, {}, &report);
    if (!path || path->empty()) {
      char text[200];
      std::snprintf(text, sizeof(text), "at %.0f mm of %.0f mm: %s",
                    1e3 * (done + (stop - done) * report.failed_waypoint /
                                      std::max<double>(1, poses.size() - 1)),
                    1e3 * length, Describe(report.waypoint).c_str());
      *reason = text;
      return std::nullopt;
    }
    if (options_.blend_lines) {
      // Every waypoint but the first, which is where the arm already is.
      kept.insert(kept.end(), path->begin() + 1, path->end());
    } else {
      kept.push_back(path->back());
    }
    q = path->back();
    done = stop;
  }
  if (kept.empty()) kept.push_back(q);  // a line of no length
  return kept;
}

PickPlace::Face PickPlace::DesignFace(int module) const {
  // Module k's glass as designed: the front face of the stack once modules
  // 0 .. k-1 have been taken, k thicknesses behind module_0's.
  const Eigen::Isometry3d design =
      FromWorldPose(cell_.pallet().nominal_modules[module]);
  Face face;
  face.normal = design.linear().col(2);
  face.long_side = design.linear().col(0);
  face.up = design.linear().col(1);
  face.glass = design.translation() + 0.5 * kModuleThickness * face.normal;
  return face;
}

PickPlace::Face PickPlace::EstimatedFace(const ModuleEstimate& estimate,
                                         int module) const {
  Face face;
  face.glass = estimate.glass_centre;
  face.normal = estimate.glass_normal.normalized();
  // The estimate's axes are those of the module's own frame, as the
  // design's are.
  face.long_side = estimate.pose.linear().col(0);
  face.up = estimate.pose.linear().col(1);
  // The pallet as the estimate places it: the design's pose of the module
  // the pick count expects, carried onto the estimate. The pallet is one
  // rigid piece with its stack, so the same motion carries every design
  // pose near it. Only the carry's over-the-pallet pose uses it.
  const int k = std::clamp(module, 0, cell_.module_count() - 1);
  face.pallet_shift =
      estimate.pose *
      FromWorldPose(cell_.pallet().nominal_modules[k]).inverse();
  return face;
}

bool PickPlace::PlanPick(const Face& face, std::string* reason,
                         PickPlaceStep* step) {
  const PalletBodies& pallet = cell_.pallet();
  const Eigen::Vector3d& normal = face.normal;
  const Eigen::Vector3d& long_side = face.long_side;
  const Eigen::Vector3d& up = face.up;

  // The tool square to the glass, its X along the module's +x, the tip
  // `distance` in front of the glass centre. In camera mode these are the
  // estimate's glass and axes, so every pick-side pose below, the lift
  // along the glass and the pull-back along its normal included, follows
  // the module wherever the pallet was set down.
  auto facing = [&](double distance) {
    return *GripPose(face.glass + distance * normal, normal, long_side);
  };
  const Eigen::Isometry3d standoff = facing(options_.standoff);
  const Eigen::Isometry3d slow_from =
      facing(std::max(options_.approach_slow, options_.grip_gap));
  grip_tip_ = facing(options_.grip_gap);
  const Eigen::Isometry3d lifted =
      Shifted(grip_tip_, options_.lift * up + options_.lift_back * normal);
  const double lift_length = std::hypot(options_.lift, options_.lift_back);
  const Eigen::Isometry3d pulled =
      Shifted(lifted, options_.pull_back * normal);
  // The clear pose also turns the module about the vertical until it
  // squarely faces the arm (SquaredToArm). With the pallet set down turned,
  // the module, and so the tool, is turned with it, and the wrist holds it
  // with joint 4 off zero (14 degrees for a pallet turned 2.8 degrees and
  // shifted 3 cm). The flatten that follows swings joint 5 through about
  // 130 degrees; with joint 4 off zero that is no longer a turn about the
  // module's long side, and its 1.15 m ends swept sideways into the next
  // module. Squared to the arm, joint 4 is zero and the flatten turns the
  // module about its own long side, as at nominal. The turn (up to about 4
  // degrees: the pallet's 3 and the bearing's 1) is spread along the 0.45 m
  // back-off, so the module's ends move toward the stack by at most an
  // eighth of how far the tool has backed off.
  const Eigen::Isometry3d clear = SquaredToArm(
      Shifted(lifted, (options_.clear_back - options_.grip_gap -
                       options_.lift_back) * normal +
                          options_.clear_up * up));

  // Flat over the pallet: tool straight down, its X still along the
  // module's +x, which keeps the module's long side along the pallet. The
  // point is given in the pallet's frame, and moves with the pallet as
  // estimated, so the module turned flat keeps the same distance from the
  // stack wherever the pallet stands.
  const Eigen::Isometry3d nominal_pallet = FromWorldPose(pallet.nominal_pallet);
  const Eigen::Vector3d down_face = Eigen::Vector3d::UnitZ();  // glass up
  // Squared to the arm, like the clear pose, so the flatten is a turn of
  // joint 5 alone.
  const Eigen::Isometry3d over_pallet = SquaredToArm(*GripPose(
      face.pallet_shift * nominal_pallet * options_.over_pallet, down_face,
      long_side));
  // Ready stays where it was designed: 0.8 m from the stack, a few
  // centimetres of pallet offset do not matter there, and the arm comes
  // back to the same pose every cycle.
  const Eigen::Vector3d design_long_side =
      FromWorldPose(pallet.nominal_modules[0]).linear().col(0);
  const Eigen::Isometry3d ready =
      *GripPose(nominal_pallet * options_.ready, down_face, design_long_side);

  // Over the slot, from the fixed placement pose (the encoder's correction
  // of a few millimetres is left to the lowering). First with the long
  // side along the tube: the pallet's long-side direction carried half a
  // turn about the vertical by joint 1, as the swing does.
  const Eigen::Isometry3d place = Conveyor::PlacementPose();
  const Eigen::Vector3d place_up = place.linear().col(2);
  const Eigen::Vector3d release_glass =
      place.translation() +
      (0.5 * kModuleThickness + options_.release_clearance) * place_up;
  // The tube's cross direction away from the arm: from the base's axis
  // out through the placement spot, level.
  Eigen::Vector3d outward = place.translation();
  outward.z() = 0.0;
  outward.normalize();
  const Eigen::Vector3d over_slot_tip =
      release_glass + (options_.grip_gap + options_.turn_height) * place_up +
      options_.turn_outboard * outward;
  Eigen::Vector3d level_long_side = long_side;
  level_long_side.z() = 0.0;
  const Eigen::Vector3d swung_long_side =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitZ()) *
      level_long_side.normalized();
  const Eigen::Isometry3d over_slot_along =
      *GripPose(over_slot_tip, place_up, swung_long_side);

  auto fail = [&](PickPlaceStep s, const std::string& why) {
    *step = s;
    *reason = std::string("no IK solution for ") + StepName(s) + ": " + why;
    return false;
  };
  std::string why;

  // The standoff: seeded with joint 1 turned toward the target. Joint 1 is
  // positive clockwise seen from above, so it is minus the bearing.
  Eigen::VectorXd seed = kPickSeed;
  seed[0] = std::clamp(-std::atan2(standoff.translation().y(),
                                   standoff.translation().x()),
                       cell_.arm().lower[0], cell_.arm().upper[0]);
  std::optional<Eigen::VectorXd> q_standoff = SolvePose(standoff, seed, &why);
  if (!q_standoff) return fail(PickPlaceStep::kToStandoff, why);
  if (std::abs((*q_standoff)[3]) > kWristBranch) {
    return fail(PickPlaceStep::kToStandoff, "the wrist came out flipped");
  }

  // Straight in: the long part at speed_empty, then the slow short one.
  const double fast =
      (slow_from.translation() - standoff.translation()).norm();
  std::optional<std::vector<Eigen::VectorXd>> line =
      SolveLine(*q_standoff, standoff, slow_from, {fast}, &why);
  if (!line) return fail(PickPlaceStep::kApproach, why);
  approach_ = *line;
  const double slow = (grip_tip_.translation() - slow_from.translation()).norm();
  line = SolveLine(approach_.back(), slow_from, grip_tip_, {slow}, &why);
  if (!line) return fail(PickPlaceStep::kTouchGlass, why);
  touch_ = *line;

  line = SolveLine(touch_.back(), grip_tip_, lifted, {lift_length}, &why);
  if (!line) return fail(PickPlaceStep::kLift, why);
  lift_ = *line;

  line = SolveLine(lift_.back(), lifted, pulled, {options_.pull_back}, &why);
  if (!line) return fail(PickPlaceStep::kPullBack, why);
  pull_back_ = *line;

  // Solved along its line so the answer stays on this branch.
  const double clear_length =
      (clear.translation() - pulled.translation()).norm();
  line = SolveLine(pull_back_.back(), pulled, clear, {clear_length}, &why);
  if (!line) return fail(PickPlaceStep::kClear, why);
  clear_ = *line;

  // Flat over the pallet, seeded on the intended wrist branch (see
  // kOverPalletSeed), joint 1 turned toward it and joint 6 as at the clear
  // pose: the tool's X lies along joint 5's axis in both, so joint 6 hardly
  // changes.
  Eigen::VectorXd flat_seed = kOverPalletSeed;
  flat_seed[0] = std::clamp(-std::atan2(over_pallet.translation().y(),
                                        over_pallet.translation().x()),
                            cell_.arm().lower[0], cell_.arm().upper[0]);
  flat_seed[5] = clear_.back()[5];
  std::optional<Eigen::VectorXd> q_over_pallet =
      SolvePose(over_pallet, flat_seed, &why);
  if (!q_over_pallet) return fail(PickPlaceStep::kFlatten, why);
  if (std::abs((*q_over_pallet)[3]) > kWristBranch) {
    return fail(PickPlaceStep::kFlatten, "the wrist came out flipped");
  }
  flatten_ = {*q_over_pallet};

  // The ready pose has nearly the same orientation as over_pallet and lies
  // near it, so it is seeded from there.
  std::optional<Eigen::VectorXd> q_ready = SolvePose(ready, *q_over_pallet, &why);
  if (!q_ready) return fail(PickPlaceStep::kToStandoff, "ready pose: " + why);
  if (std::abs((*q_ready)[3]) > kWristBranch) {
    return fail(PickPlaceStep::kToStandoff, "ready pose: the wrist came out "
                                            "flipped");
  }
  to_standoff_ = {*q_ready, *q_standoff};
  return_ = {*q_ready};

  // Over the slot, long side along the tube: kPlaceSeed puts joint 1 round
  // the +Y side; joint 6 as over the pallet, since the tool keeps its
  // place relative to the arm through the swing.
  Eigen::VectorXd place_seed = kPlaceSeed;
  place_seed[5] = (*q_over_pallet)[5];
  std::optional<Eigen::VectorXd> q_over_slot =
      SolvePose(over_slot_along, place_seed, &why);
  if (!q_over_slot) return fail(PickPlaceStep::kSwing, why);
  if ((*q_over_slot)[0] > kSwungJoint1) {
    return fail(PickPlaceStep::kSwing, "joint 1 came out the wrong way round");
  }
  swing_ = {*q_over_slot};

  // The quarter turn by joint 6, whichever way ends nearer joint 6 = 0.
  // The tool's X then lies along world +X or -X; take the exact one.
  Eigen::VectorXd turned = *q_over_slot;
  turned[5] += (turned[5] > 0.0 ? -0.5 : 0.5) * kPi;
  const Eigen::Isometry3d at_turned =
      TipPoseAt(*cell_.model(), scratch_.get(), cell_.arm(), turned);
  place_x_sign_ = at_turned.linear().col(0).dot(place.linear().col(0)) >= 0.0
                      ? 1.0
                      : -1.0;
  const Eigen::Isometry3d over_slot_across = *GripPose(
      over_slot_tip, place_up, place_x_sign_ * place.linear().col(0));
  std::optional<Eigen::VectorXd> q_turned =
      SolvePose(over_slot_across, turned, &why);
  if (!q_turned) return fail(PickPlaceStep::kTurn, why);
  turn_ = {*q_turned};
  return true;
}

bool PickPlace::PlanPlace(std::string* reason) {
  // Where the slot really is, as far as the encoder says: the slot's pose
  // for the slide position it reads now. This takes out the conveyor's
  // stopping error.
  //
  // What the place side assumes about the grip: that the module hangs from
  // the tool exactly as the pick planned it, its glass grip_gap below the
  // tip, centred, square and with its long side along the tool's X. In
  // blind mode that is wrong by however far the pallet is off its design;
  // in camera mode by the error of the estimate the pick aimed at plus how
  // far the approach strayed from it. The script cannot see either, and
  // lays the module down wrong by just that much; the scorer shows it.
  // With the check after pickup the grip has been measured (held_in_tip_),
  // and the tip is sent where that grip puts the MODULE on its release pose
  // (CorrectedTipTarget): the module, not the tool, lands on the slot.
  const Conveyor& conveyor = cell_.conveyor();
  const Eigen::Isometry3d slot = Conveyor::SlotPose(
      outcome_.slot, conveyor.SensorPosition(*cell_.data()));
  const Eigen::Vector3d up = slot.linear().col(2);
  const Eigen::Vector3d glass =
      slot.translation() +
      (0.5 * kModuleThickness + options_.release_clearance) * up;
  // With the design grip the module hangs grip_gap below the tip (the weld
  // kept the gap it was gripped with), so the tip is that much above where
  // the glass goes.
  const Eigen::Isometry3d design_release = *GripPose(
      glass + options_.grip_gap * up, up, place_x_sign_ * slot.linear().col(0));
  release_module_ =
      design_release * DesignModuleInTip(kModuleThickness, options_.grip_gap);
  const Eigen::Isometry3d release =
      held_in_tip_ ? CorrectedTipTarget(*release_module_, *held_in_tip_)
                   : design_release;
  release_tip_ = release;

  // Where the arm is: the end of the turn, as planned. In from there at
  // the same height to over the slot, then down.
  const Eigen::Isometry3d turned =
      TipPoseAt(*cell_.model(), scratch_.get(), cell_.arm(), turn_.back());
  const Eigen::Isometry3d above = Shifted(release, options_.set_down_slow * up);
  Eigen::Isometry3d over = above;
  over.translation().z() = turned.translation().z();

  std::string why;
  const double in = (over.translation() - turned.translation()).norm();
  std::vector<double> stops;
  const int legs = std::max(1, options_.in_legs);
  for (int i = 1; i <= legs; ++i) stops.push_back(in * i / legs);
  std::optional<std::vector<Eigen::VectorXd>> line =
      SolveLine(turn_.back(), turned, over, stops, &why);
  if (!line) {
    *reason = std::string("no IK solution for lower (in): ") + why;
    return false;
  }
  lower_ = *line;
  const double drop = (above.translation() - over.translation()).norm();
  line = SolveLine(lower_.back(), over, above, {drop}, &why);
  if (!line) {
    *reason = std::string("no IK solution for lower (down): ") + why;
    return false;
  }
  lower_.insert(lower_.end(), line->begin(), line->end());
  line = SolveLine(lower_.back(), above, release, {options_.set_down_slow},
                   &why);
  if (!line) {
    *reason = std::string("no IK solution for set down: ") + why;
    return false;
  }
  set_down_ = *line;
  const Eigen::Isometry3d up_high = Shifted(release, options_.retreat * up);
  line = SolveLine(set_down_.back(), release, up_high,
                   {options_.retreat_short, options_.retreat}, &why);
  if (!line) {
    *reason = std::string("no IK solution for retreat: ") + why;
    return false;
  }
  retreat_ = *line;
  return true;
}

void PickPlace::set_known_module(int i, const KnownModule& known) {
  NoticeReset();
  if (i >= 0 && i < static_cast<int>(known_.size())) known_[i] = known;
}

bool PickPlace::NoticeReset() {
  const double now = cell_.data()->time;
  const bool reset = now < last_time_;
  if (reset) {
    // The cell was Reset: the old plan is in the old time, the last
    // cycle's picture is of another scene, and every module stands on the
    // pallet again.
    follower_ = ArmFollower(cell_.arm());
    look_ahead_.reset();
    known_.assign(cell_.module_count(), KnownModule{});
  }
  last_time_ = now;
  return reset;
}

Eigen::Isometry3d PickPlace::GripInTip() const {
  return held_in_tip_ ? *held_in_tip_
                      : DesignModuleInTip(kModuleThickness, options_.grip_gap);
}

Eigen::Isometry3d PickPlace::PalletShift(int held) const {
  // Camera mode: the stack where the estimate the pick aimed at puts it,
  // carried from the design pose of the module actually held.
  if (options_.sensing == SensingMode::kCamera && estimate_) {
    return EstimatedFace(*estimate_, held).pallet_shift;
  }
  return Eigen::Isometry3d::Identity();
}

PlanScene PickPlace::CarryPlanScene(int held) const {
  CarrySceneInput input;
  input.held = held;
  input.grip_gap = options_.grip_gap;
  input.module_in_tip = held_in_tip_;
  input.pallet_shift = PalletShift(held);
  input.modules = known_;
  // The encoder; while the tube still moves, where it is headed (its
  // nominal stop: the drive's own stopping error is a few millimetres).
  const Conveyor& conveyor = cell_.conveyor();
  const mjData& data = *cell_.data();
  input.tube_slide = conveyor.Moving(data)
                         ? Conveyor::NominalSlidePosition(conveyor.steps())
                         : conveyor.SensorPosition(data);
  return CarryScene(cell_.pallet(), input);
}

bool PickPlace::PlanCarry(std::string* reason) {
  CarryReport& report = outcome_.carry;
  report.attempted = true;
  report.found = false;
  report.legs.clear();
  report.clearance = 0.0;
  report.clearance_between.clear();
  report.trajectory_seconds = 0.0;
  carry_legs_.clear();
  carry_.clear();
  const auto begin = std::chrono::steady_clock::now();
  const bool found = [&]() {
    if (clear_.empty() || turn_.empty()) {
      *reason = "the pick side is not planned";
      return false;
    }
    if (!checker_) {
      std::string error;
      checker_ = CollisionChecker::Create(*cell_.model(), {}, &error);
      if (!checker_) {
        *reason = "no collision checker: " + error;
        return false;
      }
    }
    const PlanScene scene = CarryPlanScene(outcome_.module);
    report.tube_moving = cell_.conveyor().Moving(*cell_.data());

    // Where each leg ends, and how long the arm holds still there: to the
    // end of the turn; with the check after pickup, first to the check
    // pose, holding still there for the picture. Every leg with the grip
    // the script has (the design's, until measured).
    struct LegEnd {
      Eigen::VectorXd q;
      double dwell = 0.0;
    };
    std::vector<LegEnd> ends;
    check_leg_.reset();
    if (options_.check_after_pickup) {
      check_leg_ = ends.size();
      ends.push_back({options_.check_pose, options_.check_dwell});
    }
    ends.push_back({turn_.back(), 0.0});

    Eigen::VectorXd from = clear_.back();
    for (std::size_t leg = 0; leg < ends.size(); ++leg) {
      MotionPlanOptions plan = options_.planner;
      plan.speed = options_.speed_carrying;
      // Each leg its own seeds, all from the one fixed base seed.
      plan.base_seed =
          options_.planner.base_seed + 1000u * static_cast<std::uint32_t>(leg);
      MotionPlanReport leg_report;
      std::optional<TimedMove> move =
          PlanArmMove(*checker_, scene, from, ends[leg].q, cell_.arm().limits,
                      plan, &leg_report);
      report.legs.push_back(leg_report);
      if (!move) {
        *reason = std::string(ToString(leg_report.status)) + ": " +
                  leg_report.reason;
        if (ends.size() > 1) {
          *reason = "leg " + std::to_string(leg + 1) + " of " +
                    std::to_string(ends.size()) + ": " + *reason;
        }
        return false;
      }
      carry_legs_.push_back({std::move(*move), ends[leg].dwell});
      from = ends[leg].q;
    }
    JoinCarryLegs();
    return true;
  }();
  report.plan_seconds = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - begin)
                            .count();
  report.found = found;
  if (!found) {
    carry_legs_.clear();
    carry_.clear();
  }
  return found;
}

void PickPlace::JoinCarryLegs() {
  CarryReport& report = outcome_.carry;
  carry_.clear();
  report.trajectory_seconds = 0.0;
  for (std::size_t leg = 0; leg < carry_legs_.size(); ++leg) {
    const TimedMove& move = carry_legs_[leg].move;
    // The legs' reports, in order; a replanned leg's report replaces the
    // first plan's.
    const MotionPlanReport& leg_report = report.legs[leg];
    if (leg == 0 || leg_report.clearance < report.clearance) {
      report.clearance = leg_report.clearance;
      report.clearance_between = leg_report.clearance_between;
    }
    report.trajectory_seconds += move.trajectory.duration();
    carry_.insert(carry_.end(), move.path.begin() + (leg == 0 ? 0 : 1),
                  move.path.end());
  }
}

bool PickPlace::RecheckLeg(std::size_t leg, std::string* reason) {
  CheckReport& check = outcome_.check;
  const MotionPlanOptions& planner = options_.planner;
  const double spacing = options_.recheck_spacing;
  // The planner's budget (src/motion_planner.h) with the edge term for
  // samples `spacing` apart: every configuration along the path is then
  // within spacing / 2 of a checked one, and the corners and the tracking
  // add the same as for the planned path. A path whose samples all keep
  // these margins keeps required_margin when followed, as a planned one
  // does.
  const double budget =
      0.5 * spacing + planner.corner_allowance + planner.tracking_allowance;
  check.recheck_margin = planner.required_margin + budget;
  check.recheck_self_margin = planner.required_self_margin + budget;
  const auto begin = std::chrono::steady_clock::now();
  // The checker still holds the scene the legs were planned in (PlanArmMove
  // left it so), and nothing in it has moved since: the stack's estimate is
  // the same, and the tube stands still from before lift until the set
  // down is over (if it was still moving when the carry was planned, the
  // scene has it at its nominal stop, as the plan did). Only the grip
  // changes: the measured one goes on the tool.
  bool clear = checker_->SetHeldModule(outcome_.module, GripInTip());
  const double old_margin = checker_->margin();
  const double old_self = checker_->self_margin();
  if (clear) {
    clear = checker_->SetMargins(check.recheck_margin,
                                 check.recheck_self_margin);
  }
  check.recheck_hit.clear();
  if (clear) {
    const JointPath samples =
        DenseSamples(*checker_, carry_legs_[leg].move.path, spacing);
    check.recheck_samples = static_cast<int>(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const std::optional<CollisionReport> hit = checker_->Check(samples[i]);
      if (!hit || !hit->free) {
        clear = false;
        char text[300];
        std::snprintf(text, sizeof(text), "sample %zu of %zu: %s", i + 1,
                      samples.size(),
                      !hit ? "not a configuration"
                      : !hit->in_range ? "out of the joints' range"
                      : hit->pairs.empty()
                          ? "too close"
                          : hit->pairs.front().between.c_str());
        check.recheck_hit = text;
        break;
      }
    }
  } else {
    check.recheck_hit = "the measured grip could not be put on the tool";
  }
  checker_->SetMargins(old_margin, old_self);
  check.rechecked = true;
  check.recheck_clear = clear;
  check.recheck_seconds = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - begin)
                              .count();
  if (clear) return true;

  // Replanned from where the leg starts (the arm holds there) to where it
  // ends, in the scene as it is now, with the measured grip; its own seeds.
  const auto replan_begin = std::chrono::steady_clock::now();
  const PlanScene scene = CarryPlanScene(outcome_.module);
  MotionPlanOptions plan = planner;
  plan.speed = options_.speed_carrying;
  plan.base_seed = planner.base_seed + 1000u * static_cast<std::uint32_t>(leg) +
                   500u;
  const TimedMove& old = carry_legs_[leg].move;
  std::optional<TimedMove> move =
      PlanArmMove(*checker_, scene, old.path.front(), old.path.back(),
                  cell_.arm().limits, plan, &check.replan);
  check.replanned = true;
  check.replan_seconds = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - replan_begin)
                             .count();
  if (!move) {
    *reason = std::string("leg ") + std::to_string(leg + 1) +
              " replanned with the measured grip: " +
              ToString(check.replan.status) + ": " + check.replan.reason;
    return false;
  }
  carry_legs_[leg].move = std::move(*move);
  outcome_.carry.legs[leg] = check.replan;
  JoinCarryLegs();
  return true;
}

PickPlace::CheckNext PickPlace::RunCheck() {
  CheckReport& check = outcome_.check;
  CheckRecord record;
  record.time = cell_.data()->time;
  record.attempt = check.pictures + 1;
  // Where the arm says it is, not where it was told to be: the expected
  // pose and the tip the grip is measured against both come from these.
  record.arm = JointAngles(cell_.arm(), *cell_.data());
  // The vacuum switch at the picture: whether anything is on the tool.
  record.vacuum_holds = cell_.VacuumHolds();
  record.tip = TipPoseAt(*cell_.model(), scratch_.get(), cell_.arm(),
                         record.arm);
  const Eigen::Isometry3d design_in_tip =
      DesignModuleInTip(kModuleThickness, options_.grip_gap);
  record.expected = record.tip * design_in_tip;
  const auto begin = std::chrono::steady_clock::now();
  const DepthImage image = cell_.TakeCheckImage();
  record.render_seconds = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - begin)
                              .count();
  // The check camera as calibrated: where the control code believes it is.
  record.estimate =
      FindHeldModule(image, cell_.CalibratedCheckCamera(), record.expected,
                     options_.held_finder, &record.diagnostics);
  if (record.estimate) {
    record.module_in_tip = GripOffsetFromCheck(record.estimate->pose,
                                               record.tip);
  }
  ++check.pictures;
  check.vacuum_holds = record.vacuum_holds;
  check.finder_seconds += record.diagnostics.seconds_total;
  check.render_seconds += record.render_seconds;
  checks_.push_back(record);
  if (check_hook_) check_hook_(checks_.back());

  const HeldFinderDiagnostics& d = record.diagnostics;
  char text[600];
  // The vacuum lost: nothing on the tool to place, whatever the picture
  // shows (a module still falling may be found, or seen far from where it
  // should be). The finder's answer goes into the reason.
  auto vacuum_lost = [&]() {
    check.status = CheckReport::Status::kRefused;
    Fail(std::string("the check after pickup: no module on the tool: the "
                     "vacuum is lost (dropped or never picked); picture ") +
         std::to_string(check.pictures) + ": " +
         (record.estimate ? std::string("the finder saw a module anyway")
                          : std::string(HeldFinderFailureName(d.failure)) +
                                ": " + d.detail));
    return CheckNext::kFailed;
  };
  if (record.estimate && !record.vacuum_holds) return vacuum_lost();
  if (record.estimate) {
    held_in_tip_ = *record.module_in_tip;
    check.status = CheckReport::Status::kChecked;
    check.retaken = check.pictures > 1;
    check.module_in_tip = *held_in_tip_;
    const Eigen::Isometry3d offset = design_in_tip.inverse() * *held_in_tip_;
    check.offset_shift = offset.translation();
    const Eigen::AngleAxisd turn(offset.linear());
    check.offset_rotation = turn.angle() * turn.axis();
    // Leg 2 was planned with the design grip; checked again with the
    // measured one, replanned only if that fails.
    std::string why;
    const std::size_t next = *check_leg_ + 1;
    if (next < carry_legs_.size() && !RecheckLeg(next, &why)) {
      Fail("the check after pickup: " + why);
      return CheckNext::kFailed;
    }
    char leg[300] = "";
    if (next < carry_legs_.size()) {
      if (check.replanned) {
        std::snprintf(leg, sizeof(leg),
                      "; leg %zu not clear with it (%s), replanned in %.0f ms",
                      next + 1, check.recheck_hit.c_str(),
                      1e3 * check.replan_seconds);
      } else {
        std::snprintf(leg, sizeof(leg),
                      "; leg %zu clear with it at %.0f mm (%d samples, %.0f "
                      "ms)",
                      next + 1, 1e3 * check.recheck_margin,
                      check.recheck_samples, 1e3 * check.recheck_seconds);
      }
    }
    char retaken[200] = "";
    if (check.retaken) {
      std::snprintf(retaken, sizeof(retaken),
                    "; RETAKEN, picture 1 refused: %s",
                    HeldFinderFailureName(check.failure));
    }
    std::snprintf(text, sizeof(text),
                  "check: grip off the design by %+.2f %+.2f %+.2f mm, %.3f "
                  "deg (picture %d, finder %.0f ms)%s%s",
                  1e3 * check.offset_shift.x(), 1e3 * check.offset_shift.y(),
                  1e3 * check.offset_shift.z(),
                  check.offset_rotation.norm() / kDegree, check.pictures,
                  1e3 * d.seconds_total, retaken, leg);
    check_note_ = text;
    return CheckNext::kGoOn;
  }
  check.failure = d.failure;
  check.failure_detail = d.detail;
  if (d.failure == HeldFinderFailure::kFarFromExpected) {
    if (!record.vacuum_holds) return vacuum_lost();
    // src/held_module_finder.h, step 9: slipped on the cups; stop.
    check.status = CheckReport::Status::kSlipped;
    Fail(std::string("the check after pickup: the module has slipped on the "
                     "cups (the vacuum holds it): ") +
         d.detail);
    return CheckNext::kFailed;
  }
  // One more picture after another dwell, whatever the vacuum says: a
  // second look costs one dwell, and the reason below then rests on both.
  if (check.pictures < 2) return CheckNext::kRetake;
  if (!record.vacuum_holds) return vacuum_lost();
  // Refused twice with a module on the tool: the camera's fault, or one the
  // finder names. Nothing is placed uncorrected.
  check.status = CheckReport::Status::kRefused;
  if (d.failure == HeldFinderFailure::kTooFewPoints) {
    std::snprintf(text, sizeof(text),
                  "the check after pickup: check camera fault: the vacuum "
                  "holds a module but the camera sees too few points (%d of "
                  "%.0f expected, %.0f needed; %d in picture 1)",
                  d.points_in_crop, d.expected_pixels,
                  options_.held_finder.min_point_fraction * d.expected_pixels,
                  checks_.front().diagnostics.points_in_crop);
    Fail(text);
  } else {
    Fail(std::string("the check after pickup: refused twice, the vacuum "
                     "holding a module: ") +
         HeldFinderFailureName(d.failure) + ": " + d.detail);
  }
  return CheckNext::kFailed;
}

void PickPlace::BeginCarry() {
  Enter(PickPlaceStep::kCarry);
  CarryReport& report = outcome_.carry;
  if (!report.attempted) {
    // Started later than lift: planned now, with nothing to run alongside.
    std::string why;
    if (!PlanCarry(&why)) carry_failure_ = why;
  } else {
    // Planned at the start of lift: it ran alongside these moves.
    report.overlap_seconds = 0.0;
    for (const StepRecord& r : records_) {
      if (r.step == PickPlaceStep::kLift || r.step == PickPlaceStep::kPullBack ||
          r.step == PickPlaceStep::kClear) {
        report.overlap_seconds += r.duration();
      }
    }
  }
  if (!report.found) {
    Fail("no planned carry: " + carry_failure_);
    return;
  }
  char text[400];
  char when[160];
  if (report.overlap_seconds > 0.0) {
    std::snprintf(when, sizeof(when),
                  "during lift, pull back and clear (%.2f s): %s",
                  report.overlap_seconds,
                  report.fits() ? "fits" : "does NOT fit");
  } else {
    std::snprintf(when, sizeof(when), "when the carry began");
  }
  std::snprintf(text, sizeof(text),
                "planned in %.0f ms %s; %zu leg%s, %zu waypoints, %.2f s; "
                "clearance %.0f mm (%s)%s",
                1e3 * report.plan_seconds, when, carry_legs_.size(),
                carry_legs_.size() == 1 ? "" : "s", carry_.size(),
                report.trajectory_seconds, 1e3 * report.clearance,
                report.clearance_between.c_str(),
                report.tube_moving ? "; tube still moving when planned" : "");
  carry_note_ = text;
  carry_leg_ = 0;
  carry_dwelling_ = false;
  BeginCarryLeg();
}

void PickPlace::BeginCarryLeg() {
  const CarryLeg& leg = carry_legs_[carry_leg_];
  planned_.push_back(
      {PickPlaceStep::kCarry, leg.move.path, options_.speed_carrying});
  std::string reason;
  if (!follower_.Follow(leg.move.trajectory, *cell_.data(), &reason)) {
    Fail("the follower refused the planned carry: " + reason);
    return;
  }
  plan_end_ = cell_.data()->time + follower_.TimeLeft(*cell_.data());
}

bool PickPlace::CarryFinished() {
  const double now = cell_.data()->time;
  if (carry_dwelling_) {
    if (now < carry_dwell_end_) return false;
    carry_dwelling_ = false;
    if (check_leg_ && carry_leg_ == *check_leg_) {
      // The end of the dwell at the check pose: the picture.
      switch (RunCheck()) {
        case CheckNext::kFailed:
          return false;  // the cycle has failed, holding here
        case CheckNext::kRetake:
          // One more dwell, then the picture again.
          carry_dwelling_ = true;
          carry_dwell_end_ = now + options_.check_dwell;
          return false;
        case CheckNext::kGoOn:
          break;
      }
    }
  } else {
    if (MoveFinished(false) != Progress::kFinished) return false;
    const double dwell = carry_legs_[carry_leg_].dwell;
    if (dwell > 0.0 || (check_leg_ && carry_leg_ == *check_leg_)) {
      carry_dwelling_ = true;
      carry_dwell_end_ = now + dwell;
      return false;
    }
  }
  if (carry_leg_ + 1 >= carry_legs_.size()) return true;
  ++carry_leg_;
  BeginCarryLeg();
  return false;
}

bool PickPlace::Start(int module, PickPlaceStep first) {
  if (step_ != PickPlaceStep::kIdle && !Done()) return false;
  const mjData& data = *cell_.data();
  NoticeReset();
  const bool camera = options_.sensing == SensingMode::kCamera;
  if (first == PickPlaceStep::kIdle) {
    first = camera ? PickPlaceStep::kLook : PickPlaceStep::kToStandoff;
  }
  if (!camera && first == PickPlaceStep::kLook) first = PickPlaceStep::kToStandoff;

  outcome_ = PickPlaceOutcome{};
  outcome_.status = PickPlaceOutcome::Status::kRunning;
  outcome_.module = module;
  outcome_.slot = cell_.conveyor().steps();
  outcome_.start_time = data.time;
  outcome_.carry.mode = options_.carry;
  outcome_.check.status = options_.check_after_pickup
                              ? CheckReport::Status::kNotRun
                              : CheckReport::Status::kOff;
  carry_legs_.clear();
  carry_.clear();
  carry_failure_.clear();
  carry_note_.clear();
  carry_leg_ = 0;
  carry_dwelling_ = false;
  check_leg_.reset();
  checks_.clear();
  held_in_tip_.reset();
  check_note_.clear();
  planned_.clear();
  records_.clear();
  looks_.clear();
  estimate_.reset();
  release_tip_.reset();
  release_module_.reset();
  set_down_blocked_ = false;
  paused_ = false;
  step_ = PickPlaceStep::kIdle;

  if (module < 0 || module >= cell_.module_count()) {
    Fail("no module_" + std::to_string(module) + " on the pallet");
    return false;
  }
  if (outcome_.slot >= Conveyor::kSlotCount) {
    Fail("the tube is full");
    return false;
  }
  if (static_cast<int>(first) < static_cast<int>(PickPlaceStep::kLook) ||
      static_cast<int>(first) > static_cast<int>(PickPlaceStep::kReturn)) {
    Fail(std::string("cannot start at ") + StepName(first));
    return false;
  }
  const bool planned_carry = options_.carry == CarryMode::kPlanned;
  if (options_.check_after_pickup) {
    if (!planned_carry) {
      Fail("the check after pickup needs the planned carry: the check pose "
           "is a leg end of the planned carry, and the scripted one has no "
           "place to stop over the check camera");
      return false;
    }
    if (options_.check_pose.size() != Arm::kJoints ||
        !options_.check_pose.allFinite() || !(options_.check_dwell >= 0.0) ||
        !(options_.recheck_spacing > 0.0)) {
      Fail("the check after pickup's options are out of range (check_pose "
           "six finite angles, check_dwell >= 0, recheck_spacing > 0)");
      return false;
    }
  }
  const bool scripted_carry_step = first == PickPlaceStep::kFlatten ||
                                   first == PickPlaceStep::kSwing ||
                                   first == PickPlaceStep::kTurn;
  if ((planned_carry && scripted_carry_step) ||
      (!planned_carry && first == PickPlaceStep::kCarry)) {
    Fail(std::string("cannot start at ") + StepName(first) + " with the " +
         CarryModeName(options_.carry) + " carry");
    return false;
  }
  // In camera mode the pick side is planned once the look step has an
  // estimate; started after it, from the design.
  if (first != PickPlaceStep::kLook) {
    std::string reason;
    PickPlaceStep failed = PickPlaceStep::kIdle;
    if (!PlanPick(DesignFace(module), &reason, &failed)) {
      step_ = failed;
      Fail(reason);
      return false;
    }
    // Starting later in the cycle (a test's constructed state): the place
    // side is planned on entering `lower`, so a start after that plans it
    // now.
    if (static_cast<int>(first) > static_cast<int>(PickPlaceStep::kLower) &&
        !PlanPlace(&reason)) {
      Fail(reason);
      return false;
    }
  }
  BeginStep(first);
  return !Done();
}

const std::vector<Eigen::VectorXd>* PickPlace::Waypoints(
    PickPlaceStep step) const {
  switch (step) {
    case PickPlaceStep::kLook:
      return &look_;
    case PickPlaceStep::kToStandoff:
      return &to_standoff_;
    case PickPlaceStep::kApproach:
      return &approach_;
    case PickPlaceStep::kTouchGlass:
      return &touch_;
    case PickPlaceStep::kLift:
      return &lift_;
    case PickPlaceStep::kPullBack:
      return &pull_back_;
    case PickPlaceStep::kClear:
      return &clear_;
    case PickPlaceStep::kFlatten:
      return &flatten_;
    case PickPlaceStep::kSwing:
      return &swing_;
    case PickPlaceStep::kTurn:
      return &turn_;
    case PickPlaceStep::kCarry:
      return &carry_;
    case PickPlaceStep::kLower:
      return &lower_;
    case PickPlaceStep::kSetDown:
      return &set_down_;
    case PickPlaceStep::kRetreat:
      return &retreat_;
    case PickPlaceStep::kReturn:
      return &return_;
    default:
      return nullptr;
  }
}

void PickPlace::Resume() {
  if (!paused_) return;
  paused_ = false;
  BeginStep(pending_);
}

void PickPlace::Enter(PickPlaceStep step) {
  step_ = step;
  step_start_ = cell_.data()->time;
  if (step != PickPlaceStep::kDone && step != PickPlaceStep::kFailed) {
    records_.push_back({step, step_start_, step_start_, false, ""});
  }
}

void PickPlace::Close(const std::string& result) {
  if (records_.empty() || records_.back().step != step_ ||
      records_.back().finished) {
    return;
  }
  records_.back().end = cell_.data()->time;
  records_.back().finished = true;
  records_.back().result = result;
}

void PickPlace::Finish(const std::string& result) {
  Close(result);
  if (step_ == PickPlaceStep::kTurn || step_ == PickPlaceStep::kCarry) {
    // The carry has ended: from the end of clear to here.
    double seconds = 0.0;
    for (const StepRecord& r : records_) {
      if (r.step == PickPlaceStep::kFlatten || r.step == PickPlaceStep::kSwing ||
          r.step == PickPlaceStep::kTurn || r.step == PickPlaceStep::kCarry) {
        seconds += r.duration();
      }
    }
    outcome_.carry.seconds = seconds;
  }
  const PickPlaceStep next = Next(step_);
  if (pause_after_ && *pause_after_ == step_) {
    paused_ = true;
    pending_ = next;
    return;
  }
  BeginStep(next);
}

void PickPlace::BeginMove(PickPlaceStep step,
                          const std::vector<Eigen::VectorXd>& waypoints,
                          double speed, std::optional<double> deviation) {
  Enter(step);
  planned_.push_back({step, waypoints, speed});
  MoveOptions move;
  move.speed = speed;
  std::string reason;
  bool started = false;
  if (deviation && options_.blend_lines) {
    move.max_deviation = *deviation;
    started = follower_.MoveAlong(waypoints, *cell_.data(), move, &reason);
  } else {
    started = follower_.MoveThrough(waypoints, *cell_.data(), move, &reason);
  }
  if (!started) {
    Fail("the follower refused the move: " + reason);
    return;
  }
  plan_end_ = cell_.data()->time + follower_.TimeLeft(*cell_.data());
}

std::string PickPlace::Shortfall(double* distance) const {
  const mjData& data = *cell_.data();
  const Eigen::VectorXd measured = JointAngles(cell_.arm(), data);
  // The end of the current step's move as planned (the follower's own goal
  // is where it holds once stopped).
  const Eigen::VectorXd goal = planned_.empty()
                                   ? follower_.Goal(data)
                                   : planned_.back().waypoints.back();
  const Eigen::Vector3d at =
      TipPoseAt(*cell_.model(), scratch_.get(), cell_.arm(), measured)
          .translation();
  const Eigen::Vector3d want =
      TipPoseAt(*cell_.model(), scratch_.get(), cell_.arm(), goal)
          .translation();
  const Eigen::VectorXd error = measured - goal;
  Eigen::Index joint = 0;
  error.cwiseAbs().maxCoeff(&joint);
  if (distance != nullptr) *distance = (want - at).norm();
  char text[200];
  std::snprintf(text, sizeof(text),
                "the tool tip stopped %.1f mm short (joint %d %.1f mrad off)",
                1e3 * (want - at).norm(), static_cast<int>(joint) + 1,
                1e3 * error[joint]);
  return text;
}

PickPlace::Progress PickPlace::MoveFinished(bool may_be_blocked) {
  const mjData& data = *cell_.data();
  // Held up on the way: a joint far behind the plan means the arm has run
  // into something. Only the joint angles are needed to see it, and it
  // works the same for a blended move as for one that stops at each
  // waypoint: the plan says where the arm should be at every instant.
  const bool held_up =
      follower_.TrackingError(data).cwiseAbs().maxCoeff() >
      options_.blocked_tracking_error;
  if (!held_up) {
    if (follower_.TimeLeft(data) > 0.0) return Progress::kRunning;
    if (follower_.Done(data)) return Progress::kFinished;
    if (data.time - plan_end_ < options_.arrive_timeout) {
      return Progress::kRunning;
    }
  }
  if (may_be_blocked) {
    // Stop the move here, so the arm does not press on to its goal.
    follower_.Hold(data);
    return Progress::kBlocked;
  }
  Fail(std::string("blocked in ") + StepName(step_) + ": " + Shortfall());
  return Progress::kRunning;
}

void PickPlace::Fail(const std::string& reason) {
  Close("failed: " + reason);
  outcome_.status = PickPlaceOutcome::Status::kFailed;
  outcome_.reason = reason;
  outcome_.failed_step = step_;
  outcome_.end_time = cell_.data()->time;
  // Stop pushing: hold the measured angles, so an arm stopped against
  // something does not go on pressing into it at full torque.
  follower_.Hold(*cell_.data());
  Enter(PickPlaceStep::kFailed);
}

void PickPlace::Update() {
  mjData* data = cell_.data();
  // The clock went back: the cell was Reset under us. The old plan is in
  // the old time, so start again with a fresh follower, holding the
  // commands Reset left (home).
  if (NoticeReset() && !Done() && step_ != PickPlaceStep::kIdle) {
    Fail("the cell was reset during the cycle");
  }
  if (!Done() && !paused_ && step_ != PickPlaceStep::kIdle) Advance();
  follower_.Update(*cell_.model(), data);
}

bool PickPlace::Skipped(PickPlaceStep step) const {
  const bool camera = options_.sensing == SensingMode::kCamera;
  const bool planned = options_.carry == CarryMode::kPlanned;
  switch (step) {
    case PickPlaceStep::kLook:
    case PickPlaceStep::kLookAhead:
      return !camera;
    case PickPlaceStep::kFlatten:
    case PickPlaceStep::kSwing:
    case PickPlaceStep::kTurn:
      return planned;
    case PickPlaceStep::kCarry:
      return !planned;
    default:
      return false;
  }
}

PickPlaceStep PickPlace::Next(PickPlaceStep step) const {
  PickPlaceStep next = step;
  do {
    next = static_cast<PickPlaceStep>(static_cast<int>(next) + 1);
  } while (Skipped(next));
  return next;
}

bool PickPlace::set_pause_after(std::optional<PickPlaceStep> step) {
  if (step && (*step == PickPlaceStep::kIdle || *step == PickPlaceStep::kDone ||
               *step == PickPlaceStep::kFailed || Skipped(*step))) {
    return false;
  }
  pause_after_ = step;
  return true;
}

LookRecord PickPlace::TakeLook(PickPlaceStep step) {
  LookRecord look;
  look.time = cell_.data()->time;
  look.step = step;
  look.arm = JointAngles(cell_.arm(), *cell_.data());
  const auto begin = std::chrono::steady_clock::now();
  const DepthImage image = cell_.TakeDepthImage();
  look.render_seconds = std::chrono::duration<double>(
                            std::chrono::steady_clock::now() - begin)
                            .count();
  // The camera as calibrated: where the control code believes it is.
  look.estimate = FindFrontModule(image, cell_.CalibratedCamera(), prior_,
                                  options_.finder, &look.diagnostics);
  return look;
}

void PickPlace::UseLook(const LookRecord& look) {
  const FinderDiagnostics& d = look.diagnostics;
  char text[400];
  if (look.estimate) {
    estimate_ = look.estimate;
    // The sanity check against the face the pick count expects: how far
    // along the design's glass normal the estimate is from module k's
    // design glass, against what the pallet's placement tolerance can
    // explain there (a shift of up to sqrt(2) times the tolerance, and the
    // turn about the pallet's vertical axis times the glass's distance from
    // it).
    const Face design = DesignFace(outcome_.module);
    const double off = (estimate_->glass_centre - design.glass).dot(design.normal);
    Eigen::Vector3d from_axis = design.glass - prior_.pallet_origin;
    from_axis.z() = 0.0;
    const double explained =
        std::sqrt(2.0) * prior_.placement_tolerance +
        prior_.placement_yaw_tolerance * from_axis.norm() +
        options_.face_check_margin;
    std::snprintf(text, sizeof(text),
                  "%sfound module, %+.1f mm from module_%d's design face "
                  "along its normal%s (finder %.0f ms)",
                  look_note_.c_str(), 1e3 * off, outcome_.module,
                  std::abs(off) > explained
                      ? ", MORE than the pallet's tolerance explains: another "
                        "module in front?"
                      : "",
                  1e3 * d.seconds_total);
    std::string reason;
    PickPlaceStep failed = PickPlaceStep::kIdle;
    if (!PlanPick(EstimatedFace(*estimate_, outcome_.module), &reason,
                  &failed)) {
      Fail(reason);
      return;
    }
    Finish(text);
    return;
  }
  // Where the picture was taken from. Away from the look pose the arm may
  // be in the picture, and then any step of the finder can be the one that
  // refuses: at arm_home the tool hides part of the glass's outline and
  // the rectangle fit refuses before the view check is reached. So a
  // refusal there earns one more picture from the look pose, where nothing
  // of the arm is in the view; a refusal from the look pose is the scene's.
  const bool at_look_pose =
      (look.arm - look_pose_).cwiseAbs().maxCoeff() < kAtLookPose;
  if (!look_moved_ &&
      (d.failure == FinderFailure::kViewBlocked || !at_look_pose)) {
    look_moved_ = true;
    look_moving_ = true;
    look_note_ = std::string(FinderFailureName(d.failure)) +
                 " where the arm stood, taken again from the look pose: ";
    look_ = {look_pose_};
    planned_.push_back({PickPlaceStep::kLook, look_, options_.speed_empty});
    MoveOptions move;
    move.speed = options_.speed_empty;
    std::string reason;
    if (!follower_.MoveThrough(look_, *cell_.data(), move, &reason)) {
      Fail("the follower refused the move to the look pose: " + reason);
      return;
    }
    plan_end_ = cell_.data()->time + follower_.TimeLeft(*cell_.data());
    return;
  }
  std::snprintf(text, sizeof(text), "%sthe finder refused: %s: %s",
                look_note_.c_str(), FinderFailureName(d.failure),
                d.detail.c_str());
  Fail(text);
}

// What each step does when it begins. Steps that move start their move;
// the others act at once.
void PickPlace::BeginStep(PickPlaceStep step) {
  mjData* data = cell_.data();
  switch (step) {
    case PickPlaceStep::kLook:
      Enter(step);
      look_moving_ = false;
      look_moved_ = false;
      look_note_.clear();
      look_.clear();
      if (look_ahead_) {
        // Taken at the end of the last cycle, for this one.
        const LookRecord look = *look_ahead_;
        look_ahead_.reset();
        look_note_ = "picture from the last cycle's look ahead: ";
        looks_.push_back(look);
        UseLook(look);
        return;
      }
      // Otherwise a picture from where the arm stands, once it has held
      // still for look_time.
      look_hold_start_ = data->time;
      return;
    case PickPlaceStep::kToStandoff:
      BeginMove(step, to_standoff_, options_.speed_empty);
      return;
    case PickPlaceStep::kApproach:
      BeginMove(step, approach_, options_.speed_empty, options_.deviation_far);
      return;
    case PickPlaceStep::kTouchGlass:
      BeginMove(step, touch_, options_.speed_touching, options_.deviation_near);
      return;
    case PickPlaceStep::kGrip: {
      Enter(step);
      grip_note_.clear();
      GripFailure failure;
      const std::optional<int> gripped = cell_.Grip(&failure);
      if (!gripped) {
        Fail("grip refused: " + failure.message);
      } else if (*gripped != outcome_.module) {
        if (options_.sensing == SensingMode::kCamera) {
          // The camera found the front face, whichever module that is; the
          // grip says which one it was. Picked is picked: carry on, and say
          // so.
          grip_note_ = "gripped module_" + std::to_string(*gripped) +
                       ", where the pick count expected module_" +
                       std::to_string(outcome_.module);
          outcome_.module = *gripped;
        } else {
          // Whatever was gripped stays held; the cycle stops here.
          Fail("gripped module_" + std::to_string(*gripped) +
               ", not module_" + std::to_string(outcome_.module));
        }
      }
      if (gripped) set_known_module(*gripped, {ModulePlace::kHeld, -1});
      return;
    }
    case PickPlaceStep::kLift:
      // The grip has held, and the carry's start (the end of clear) is
      // known: plan the carry now, so that on a real cell it is ready by
      // the end of clear. A failure shows in the carry step.
      if (options_.carry == CarryMode::kPlanned) {
        std::string why;
        if (!PlanCarry(&why)) carry_failure_ = why;
      }
      BeginMove(step, lift_, options_.speed_lift_off, options_.deviation_near);
      return;
    case PickPlaceStep::kPullBack:
      BeginMove(step, pull_back_, options_.speed_lift_off,
                options_.deviation_near);
      return;
    case PickPlaceStep::kClear:
      BeginMove(step, clear_, options_.speed_carrying, options_.deviation_far);
      return;
    case PickPlaceStep::kFlatten:
    case PickPlaceStep::kSwing:
    case PickPlaceStep::kTurn:
      BeginMove(step, *Waypoints(step), options_.speed_carrying);
      return;
    case PickPlaceStep::kCarry:
      BeginCarry();
      return;
    case PickPlaceStep::kWaitForConveyor:
      Enter(step);
      return;
    case PickPlaceStep::kLower: {
      // Planned only now: the slot's pose comes from the encoder, read
      // once the tube has stopped.
      std::string reason;
      if (!PlanPlace(&reason)) {
        Enter(step);
        Fail(reason);
        return;
      }
      BeginMove(step, lower_, options_.speed_carrying, options_.deviation_far);
      return;
    }
    case PickPlaceStep::kSetDown:
      BeginMove(step, set_down_, options_.speed_touching,
                options_.deviation_near);
      return;
    case PickPlaceStep::kRelease:
      Enter(step);
      cell_.Release();
      if (!set_down_blocked_) {
        // Let go over its slot as planned: the cell knows where it lies
        // from now on.
        set_known_module(outcome_.module,
                         {ModulePlace::kOnTube, outcome_.slot});
      } else {
        // Let go where the blocked set-down stopped: the module is where
        // the tool held it then (the joint angles and the grip the script
        // used), resting on the rails, but not confirmed in its slot.
        KnownModule let_go;
        let_go.place = ModulePlace::kLetGo;
        let_go.slot = outcome_.slot;
        let_go.pose = TipPoseAt(*cell_.model(), scratch_.get(), cell_.arm(),
                                JointAngles(cell_.arm(), *data)) *
                      GripInTip();
        let_go.slide = cell_.conveyor().SensorPosition(*data);
        set_known_module(outcome_.module, let_go);
        outcome_.placement_unconfirmed = true;
      }
      return;
    case PickPlaceStep::kRetreat:
      BeginMove(step, retreat_, options_.speed_empty, options_.deviation_near);
      return;
    case PickPlaceStep::kReturn:
      BeginMove(step, return_, options_.speed_empty);
      return;
    case PickPlaceStep::kConveyor: {
      Enter(step);
      if (placed_hook_) placed_hook_(outcome_.module, outcome_.slot);
      // The conveyor has stood still since the wait before the lowering.
      // After the tube's last slot, an empty tube replaces it; otherwise it
      // moves on one step, and the next slot arrives while the arm goes
      // back to ready.
      const bool moved = outcome_.slot >= Conveyor::kSlotCount - 1
                             ? cell_.SwapTube()
                             : cell_.conveyor().StartStep(*data);
      if (!moved) {
        Fail("the conveyor would not move on");
      } else if (outcome_.slot >= Conveyor::kSlotCount - 1) {
        // The full tube went away with its modules (Cell::SwapTube parks
        // them).
        // One let go where a blocked set-down stopped went with it only if
        // it lay on the tube, which the script cannot confirm.
        for (KnownModule& known : known_) {
          if (known.place == ModulePlace::kOnTube) {
            known = {ModulePlace::kParked, -1};
          } else if (known.place == ModulePlace::kLetGo) {
            known = {ModulePlace::kUnaccounted, -1};
          }
        }
      }
      return;
    }
    case PickPlaceStep::kLookAhead:
      Enter(step);
      look_hold_start_ = data->time;
      return;
    case PickPlaceStep::kDone:
      outcome_.status = PickPlaceOutcome::Status::kPlaced;
      outcome_.end_time = data->time;
      Enter(step);
      return;
    case PickPlaceStep::kIdle:
    case PickPlaceStep::kFailed:
      return;
  }
}

// Whether the current step has finished, and with what result. May fail
// the cycle.
void PickPlace::Advance() {
  const mjData& data = *cell_.data();
  const double now = data.time;
  std::string result = "done";
  switch (step_) {
    case PickPlaceStep::kLook:
      if (look_moving_) {
        if (MoveFinished(false) != Progress::kFinished) return;
        look_moving_ = false;
        look_hold_start_ = now;
        return;
      }
      if (now - look_hold_start_ < options_.look_time) return;
      looks_.push_back(TakeLook(PickPlaceStep::kLook));
      UseLook(looks_.back());
      return;  // UseLook finishes the step, or fails it, or moves
    case PickPlaceStep::kTouchGlass: {
      // If the glass is nearer than planned, or turned, the cups meet it
      // early and the arm stops short. A little short is harmless, and the
      // grip decides; further short would press the module into the stack.
      const Progress p = MoveFinished(true);
      if (p == Progress::kRunning) return;
      if (p == Progress::kBlocked) {
        double short_by = 0.0;
        const std::string how = Shortfall(&short_by);
        if (short_by > options_.touch_tolerance) {
          Fail("blocked in touch glass: " + how);
          return;
        }
        result = "met the glass early: " + how;
      }
      break;
    }
    case PickPlaceStep::kSetDown: {
      // If the module hangs lower than planned, it lands on the rails
      // before the end and the arm stops short, pressing. Let go anyway.
      const Progress p = MoveFinished(true);
      if (p == Progress::kRunning) return;
      if (p == Progress::kBlocked) {
        set_down_blocked_ = true;
        result = "blocked, let go (placement unconfirmed): " + Shortfall();
      }
      break;
    }
    case PickPlaceStep::kCarry:
      if (!CarryFinished()) return;
      result = carry_note_;
      if (!check_note_.empty()) result += "; " + check_note_;
      break;
    case PickPlaceStep::kToStandoff:
    case PickPlaceStep::kApproach:
    case PickPlaceStep::kLift:
    case PickPlaceStep::kPullBack:
    case PickPlaceStep::kClear:
    case PickPlaceStep::kFlatten:
    case PickPlaceStep::kSwing:
    case PickPlaceStep::kTurn:
    case PickPlaceStep::kLower:
    case PickPlaceStep::kRetreat:
    case PickPlaceStep::kReturn:
      // A move that is held up fails the cycle: the arm has run into
      // something it was not meant to touch.
      if (MoveFinished(false) != Progress::kFinished) return;
      break;
    case PickPlaceStep::kGrip:
      if (now - step_start_ < options_.grip_dwell) return;
      result = "held module_" + std::to_string(outcome_.module);
      if (!grip_note_.empty()) result += " (" + grip_note_ + ")";
      break;
    case PickPlaceStep::kWaitForConveyor:
      if (cell_.conveyor().Moving(data)) {
        if (now - step_start_ > options_.conveyor_timeout) {
          Fail("the conveyor did not stop");
        }
        return;
      }
      break;
    case PickPlaceStep::kRelease:
      if (now - step_start_ < options_.release_dwell) return;
      break;
    case PickPlaceStep::kConveyor:
      result = outcome_.slot >= Conveyor::kSlotCount - 1 ? "tube swapped"
                                                         : "step started";
      break;
    case PickPlaceStep::kLookAhead: {
      if (now - look_hold_start_ < options_.look_time) return;
      looks_.push_back(TakeLook(PickPlaceStep::kLookAhead));
      look_ahead_ = looks_.back();
      const FinderDiagnostics& d = look_ahead_->diagnostics;
      char text[300];
      if (look_ahead_->estimate) {
        std::snprintf(text, sizeof(text),
                      "picture for the next pick: module found (finder %.0f "
                      "ms)",
                      1e3 * d.seconds_total);
      } else {
        std::snprintf(text, sizeof(text),
                      "picture for the next pick: %s (%s)",
                      FinderFailureName(d.failure), d.detail.c_str());
      }
      result = text;
      break;
    }
    case PickPlaceStep::kIdle:
    case PickPlaceStep::kDone:
    case PickPlaceStep::kFailed:
      return;
  }
  Finish(result);
}

}  // namespace cell
