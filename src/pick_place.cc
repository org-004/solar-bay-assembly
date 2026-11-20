#include "src/pick_place.h"

#include <algorithm>
#include <cmath>
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
#include "src/cell.h"
#include "src/conveyor.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/pallet.h"
#include "src/suction.h"

// What this file may read, and does: the design poses in PalletBodies
// (nominal_pallet, nominal_modules), the conveyor's fixed layout and its
// encoder (Conveyor::PlacementPose, SlotPose, SensorPosition, Moving,
// steps), the arm's keyframe arm_home, and the joint angles through the
// follower. Tool poses come from the IK's TipPoseAt on the scratch mjData,
// never from the simulated one. Nothing here reads a module's or the
// pallet's pose out of the simulation, or calls a True... accessor.

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

// Joint 1 must be at least this far round (negative) at the slot, or the IK
// found the other way round.
constexpr double kSwungJoint1 = -0.5 * kPi;

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
    case PickPlaceStep::kReturn:
      return "return";
    case PickPlaceStep::kDone:
      return "done";
    case PickPlaceStep::kFailed:
      return "failed";
  }
  return "?";
}

PickPlace::PickPlace(Cell& cell, const PickPlaceOptions& options)
    : cell_(cell),
      options_(options),
      follower_(cell.arm()),
      scratch_(MakeData(*cell.model())),
      last_time_(cell.data()->time) {}

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
  std::vector<Eigen::VectorXd> ends;
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
    q = path->back();
    ends.push_back(q);
    done = stop;
  }
  return ends;
}

bool PickPlace::PlanPick(int module, std::string* reason, PickPlaceStep* step) {
  const PalletBodies& pallet = cell_.pallet();
  // Module k's glass as designed: the front face of the stack once modules
  // 0 .. k-1 have been taken, k thicknesses behind module_0's.
  const Eigen::Isometry3d design = FromWorldPose(pallet.nominal_modules[module]);
  const Eigen::Vector3d normal = design.linear().col(2);     // out of the glass
  const Eigen::Vector3d long_side = design.linear().col(0);  // module's +x
  const Eigen::Vector3d up = design.linear().col(1);         // up the lean
  const Eigen::Vector3d glass =
      design.translation() + 0.5 * kModuleThickness * normal;

  // The tool square to the glass, its X along the module's +x, the tip
  // `distance` in front of the glass centre.
  auto facing = [&](double distance) {
    return *GripPose(glass + distance * normal, normal, long_side);
  };
  const Eigen::Isometry3d standoff = facing(options_.standoff);
  grip_tip_ = facing(options_.grip_gap);
  const Eigen::Isometry3d lifted =
      Shifted(grip_tip_, options_.lift * up + options_.lift_back * normal);
  const double lift_length = std::hypot(options_.lift, options_.lift_back);
  const Eigen::Isometry3d pulled =
      Shifted(lifted, options_.pull_back * normal);
  const Eigen::Isometry3d clear =
      Shifted(lifted, (options_.clear_back - options_.grip_gap -
                       options_.lift_back) * normal +
                          options_.clear_up * up);

  // Flat over the pallet: tool straight down, its X still along the
  // module's +x, which keeps the module's long side along the pallet.
  const Eigen::Isometry3d nominal_pallet = FromWorldPose(pallet.nominal_pallet);
  const Eigen::Vector3d down_face = Eigen::Vector3d::UnitZ();  // glass up
  const Eigen::Isometry3d over_pallet =
      *GripPose(nominal_pallet * options_.over_pallet, down_face, long_side);
  const Eigen::Isometry3d ready =
      *GripPose(nominal_pallet * options_.ready, down_face, long_side);

  // High over the slot, from the fixed placement pose (the encoder's
  // correction of a few millimetres is left to the lowering). First with
  // the long side along the tube: the pallet's long-side direction carried
  // half a turn about the vertical by joint 1, as the swing does.
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
  const Eigen::Vector3d swung_long_side =
      Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitZ()) * long_side;
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

  // Straight in: a long leg, then the slow short one.
  const double in = options_.standoff - options_.grip_gap;
  const double fast = std::max(0.0, options_.standoff - options_.approach_slow);
  std::optional<std::vector<Eigen::VectorXd>> line =
      SolveLine(*q_standoff, standoff, grip_tip_, {fast, in}, &why);
  if (!line) return fail(PickPlaceStep::kApproach, why);
  approach_ = {(*line)[0]};
  touch_ = {(*line)[1]};

  line = SolveLine(touch_.back(), grip_tip_, lifted, {lift_length}, &why);
  if (!line) return fail(PickPlaceStep::kLift, why);
  lift_ = *line;

  line = SolveLine(lift_.back(), lifted, pulled, {options_.pull_back}, &why);
  if (!line) return fail(PickPlaceStep::kPullBack, why);
  pull_back_ = *line;

  // One leg, solved along its line so the answer stays on this branch.
  const double clear_length =
      (clear.translation() - pulled.translation()).norm();
  line = SolveLine(pull_back_.back(), pulled, clear, {clear_length}, &why);
  if (!line) return fail(PickPlaceStep::kClear, why);
  clear_ = *line;

  // Flat over the pallet, seeded from the clear pose so the wrist stays as
  // it is (joint 4 near 0; joint 5 goes from bent up to bent down). The
  // tool's X lies along joint 5's axis in both, so joint 6 hardly changes.
  std::optional<Eigen::VectorXd> q_over_pallet =
      SolvePose(over_pallet, clear_.back(), &why);
  if (!q_over_pallet) return fail(PickPlaceStep::kFlatten, why);
  flatten_ = {*q_over_pallet};

  // The ready pose has the same orientation as over_pallet and lies near
  // it, so it is seeded from there.
  std::optional<Eigen::VectorXd> q_ready = SolvePose(ready, *q_over_pallet, &why);
  if (!q_ready) return fail(PickPlaceStep::kToStandoff, "ready pose: " + why);
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
  const Conveyor& conveyor = cell_.conveyor();
  const Eigen::Isometry3d slot = Conveyor::SlotPose(
      outcome_.slot, conveyor.SensorPosition(*cell_.data()));
  const Eigen::Vector3d up = slot.linear().col(2);
  const Eigen::Vector3d glass =
      slot.translation() +
      (0.5 * kModuleThickness + options_.release_clearance) * up;
  // The module hangs grip_gap below the tip (the weld kept the gap it was
  // gripped with), so the tip is that much above where the glass goes.
  const Eigen::Isometry3d release = *GripPose(
      glass + options_.grip_gap * up, up, place_x_sign_ * slot.linear().col(0));
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
  lower_.push_back(line->back());
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

bool PickPlace::Start(int module, PickPlaceStep first) {
  if (step_ != PickPlaceStep::kIdle && !Done()) return false;
  const mjData& data = *cell_.data();
  if (data.time < last_time_) follower_ = ArmFollower(cell_.arm());
  last_time_ = data.time;

  outcome_ = PickPlaceOutcome{};
  outcome_.status = PickPlaceOutcome::Status::kRunning;
  outcome_.module = module;
  outcome_.slot = cell_.conveyor().steps();
  outcome_.start_time = data.time;
  planned_.clear();
  records_.clear();
  release_tip_.reset();
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
  if (static_cast<int>(first) < static_cast<int>(PickPlaceStep::kToStandoff) ||
      static_cast<int>(first) > static_cast<int>(PickPlaceStep::kReturn)) {
    Fail(std::string("cannot start at ") + StepName(first));
    return false;
  }
  std::string reason;
  PickPlaceStep failed = PickPlaceStep::kIdle;
  if (!PlanPick(module, &reason, &failed)) {
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
  BeginStep(first);
  return !Done();
}

const std::vector<Eigen::VectorXd>* PickPlace::Waypoints(
    PickPlaceStep step) const {
  switch (step) {
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

void PickPlace::BeginMove(PickPlaceStep step,
                          const std::vector<Eigen::VectorXd>& waypoints,
                          double speed) {
  Enter(step);
  planned_.push_back({step, waypoints, speed});
  MoveOptions move;
  move.speed = speed;
  std::string reason;
  if (!follower_.MoveThrough(waypoints, *cell_.data(), move, &reason)) {
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
  // into something. Only the joint angles are needed to see it.
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
  if (data->time < last_time_) {
    follower_ = ArmFollower(cell_.arm());
    if (!Done() && step_ != PickPlaceStep::kIdle) {
      Fail("the cell was reset during the cycle");
    }
  }
  last_time_ = data->time;
  if (!Done() && !paused_ && step_ != PickPlaceStep::kIdle) Advance();
  follower_.Update(*cell_.model(), data);
}

PickPlaceStep PickPlace::NextStep(PickPlaceStep step) {
  return static_cast<PickPlaceStep>(static_cast<int>(step) + 1);
}

// What each step does when it begins. Steps that move start their move;
// the others act at once.
void PickPlace::BeginStep(PickPlaceStep step) {
  mjData* data = cell_.data();
  switch (step) {
    case PickPlaceStep::kToStandoff:
    case PickPlaceStep::kApproach:
      BeginMove(step, *Waypoints(step), options_.speed_empty);
      return;
    case PickPlaceStep::kTouchGlass:
      BeginMove(step, touch_, options_.speed_touching);
      return;
    case PickPlaceStep::kGrip: {
      Enter(step);
      GripFailure failure;
      const std::optional<int> gripped = cell_.Grip(&failure);
      if (!gripped) {
        Fail("grip refused: " + failure.message);
      } else if (*gripped != outcome_.module) {
        // Whatever was gripped stays held; the cycle stops here.
        Fail("gripped module_" + std::to_string(*gripped) + ", not module_" +
             std::to_string(outcome_.module));
      }
      return;
    }
    case PickPlaceStep::kLift:
    case PickPlaceStep::kPullBack:
      BeginMove(step, *Waypoints(step), options_.speed_lift_off);
      return;
    case PickPlaceStep::kClear:
    case PickPlaceStep::kFlatten:
    case PickPlaceStep::kSwing:
    case PickPlaceStep::kTurn:
      BeginMove(step, *Waypoints(step), options_.speed_carrying);
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
      BeginMove(step, lower_, options_.speed_carrying);
      return;
    }
    case PickPlaceStep::kSetDown:
      BeginMove(step, set_down_, options_.speed_touching);
      return;
    case PickPlaceStep::kRelease:
      Enter(step);
      cell_.Release();
      return;
    case PickPlaceStep::kRetreat:
    case PickPlaceStep::kReturn:
      BeginMove(step, *Waypoints(step), options_.speed_empty);
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
      if (!moved) Fail("the conveyor would not move on");
      return;
    }
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
    case PickPlaceStep::kSetDown: {
      // If the module hangs lower than designed, it lands on the rails
      // before the end and the arm stops short, pressing. Let go anyway.
      const Progress p = MoveFinished(true);
      if (p == Progress::kRunning) return;
      if (p == Progress::kBlocked) result = "blocked, let go: " + Shortfall();
      break;
    }
    case PickPlaceStep::kTouchGlass: {
      // If the glass is nearer than designed, or turned, the cups meet it
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
    case PickPlaceStep::kIdle:
    case PickPlaceStep::kDone:
    case PickPlaceStep::kFailed:
      return;
  }
  Close(result);
  const PickPlaceStep next = NextStep(step_);
  if (pause_after_ && *pause_after_ == step_) {
    paused_ = true;
    pending_ = next;
    return;
  }
  BeginStep(next);
}

}  // namespace cell
