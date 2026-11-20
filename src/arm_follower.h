// Drives the arm's motors along planned joint moves.
//
// Each of the arm's motors is a position servo: its ctrl is the angle the
// joint should be at. Stepping ctrl straight to a far goal saturates the
// motor and the arm overshoots (see the actuator notes in models/arm.xml).
// ArmFollower instead plans a JointMove from where the arm is commanded to
// be to the goal, within the arm's limits, and every control tick writes the
// motors' commands from that plan at the current simulation time.
//
// Use, once per control tick:
//
//   follower.Update(*model, data);   // writes the six commands
//   mj_step(model, data);
//
// and, whenever the arm should go somewhere and the last move has ended,
// follower.MoveTo(goal, *data). Done(*data) says when it has got there.
//
// Planned and measured. The follower keeps its own idea of where the arm
// should be: the plan. Moves start from the plan, not from the measured
// angles, so one move takes over exactly where the last one ended and the
// command never jumps. The measured angles differ from the plan by a little
// while moving; TrackingError reports the difference.
//
// First use. A new ArmFollower has no plan. The first call that is given the
// state (MoveTo, MoveThrough or Update) takes the motors' current commands,
// mjData::ctrl, as the pose to hold. After a reset to the arm_home keyframe
// that is home, and the arm stays where it was told to be. (ctrl must then
// hold plain angles, as a keyframe's does; Hold is the way to start from the
// measured angles instead.) The plan is kept in simulation time, so after
// resetting mjData, which sets the time back to zero, make a new
// ArmFollower.
//
// Angles are radians in the arm's joint order (see src/arm.h); times are
// simulation seconds, mjData::time.

#ifndef SRC_ARM_FOLLOWER_H_
#define SRC_ARM_FOLLOWER_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/joint_move.h"

namespace cell {

struct MoveOptions {
  // The default speed. Full speed can saturate joint 2's motor when a
  // module is held at full reach (tests/arm_follower_test.cc,
  // FullSpeedCanSaturate...): over a few hundred moves near full reach with
  // a 30 kg module, the worst peak torque on joint 2 was 1.00 of its limit
  // at full speed, 0.96 at 0.95 and 0.91 at 0.9. 0.9 leaves about a tenth
  // in hand for what the model gets wrong, and costs about a tenth more
  // time. Without a module, full speed stayed under 0.84 of every limit in
  // every move tried.
  static constexpr double kDefaultSpeed = 0.9;

  // Fraction of full speed, in (0, 1]. The move is the full-speed move
  // played slower: velocity limits are scaled by `speed`, acceleration
  // limits by speed^2 and jerk limits by speed^3, so the move has the same
  // shape, stretched in time by 1 / speed.
  double speed = kDefaultSpeed;

  ProfileShape shape = ProfileShape::kSCurve;
};

// What goes into the command besides the planned angle. Both are on in use;
// they can be switched off to measure what each is worth.
struct FollowerSettings {
  // Add lag * planned speed, so the joint does not trail the plan by `lag`
  // seconds of motion (src/arm.h). Without it joint 1 runs about 0.07 rad
  // behind at its full speed of 2 rad/s (0.035 s of motion); with it the
  // remaining error is what the model feedforward below deals with.
  bool lag_compensation = true;

  // Add the torque the model says the planned motion needs, divided by the
  // motor's gain: the arm's inertia times the planned acceleration, plus
  // gravity and the Coriolis and centrifugal torques at the measured state.
  // The servo otherwise produces that torque only from an angle error, which
  // shows as up to 1.3 mrad of tracking error and up to 0.6 mrad of sag
  // under gravity at rest. With it the tracking error at full speed is
  // about a tenth of a mrad and the sag is gone.
  //
  // It knows only what is in the arm's own body tree. A module held by the
  // suction weld is a separate body, so its weight is not included and
  // still sags the arm: by about 0.1 mrad on joints 2 and 3, and with the
  // tool level, 0.5 mrad on joint 5 (the module hangs 0.3 m out on the
  // wrist's weaker motor). That is about half a millimetre at the tip.
  bool model_feedforward = true;
};

class ArmFollower {
 public:
  // Done() needs every joint within this of the goal and slower than
  // kArrivedSpeed. 1 mrad is about 2.7 mm at the tool tip at full reach.
  // The arm itself settles to far less within a few milliseconds of the
  // plan's end (see the tests), but a module held by the weld sags it by up
  // to about 0.5 mrad (see model_feedforward), and this leaves room for it.
  static constexpr double kArrivedAngle = 1e-3;  // rad
  static constexpr double kArrivedSpeed = 1e-2;  // rad/s

  // Keeps a copy of `arm`. Works in any model that holds the arm: it reads
  // and writes only the entries `arm` names.
  explicit ArmFollower(const Arm& arm, const FollowerSettings& settings = {});

  // Plans a move from the commanded pose to `goal`, starting at data.time.
  // Returns false, does nothing and, if `reason` is not null, says why, if
  // `goal` has the wrong length, is not finite or is outside a joint's
  // range, if options.speed is not in (0, 1], or if a move is still running.
  //
  // A running move is never replaced or extended: a MoveTo before its plan
  // has ended (TimeLeft() > 0) is refused. Use MoveThrough to go through
  // several poses in one go. Once the plan has ended a new move may start at
  // once, before the arm has quite settled; it starts from the old goal, so
  // the command does not jump.
  bool MoveTo(const Eigen::VectorXd& goal, const mjData& data,
              const MoveOptions& options = {}, std::string* reason = nullptr);

  // Plans moves through every waypoint in turn, starting at data.time. The
  // arm comes to rest at each (only for an instant: the next leg starts at
  // once). All waypoints are checked before anything is planned: if one is
  // refused, nothing is. Same refusals as MoveTo, plus an empty list.
  bool MoveThrough(const std::vector<Eigen::VectorXd>& waypoints,
                   const mjData& data, const MoveOptions& options = {},
                   std::string* reason = nullptr);

  // Drops any plan and holds the measured angles from now on. Meant for an
  // arm at rest. On a moving arm it is an emergency stop: the command stops
  // dead and the motors brake at full torque.
  void Hold(const mjData& data);

  // Writes the six motor commands for data.time. Call every control tick,
  // before mj_step. The model feedforward uses the inertia and bias forces
  // MuJoCo worked out at the last step; before the first step they are zero,
  // which only matters if a move starts at that very instant.
  //
  // MuJoCo clamps each command to its joint's range, so close to the end of
  // a range the lead terms are cut off and the joint trails the plan a
  // little more there.
  void Update(const mjModel& model, mjData* data);

  // The plan has ended, and the arm has really arrived: every joint within
  // kArrivedAngle of the goal and slower than kArrivedSpeed.
  bool Done(const mjData& data) const;

  // Seconds until the plan ends; 0 once it has, or if there is none.
  double TimeLeft(const mjData& data) const;

  // Where the plan has the arm at data.time: angles, speeds, accelerations
  // and jerks. At rest on the goal when no move is running.
  JointSample Planned(const mjData& data) const;

  // Where the arm is going: the last waypoint of the plan, or the held pose.
  Eigen::VectorXd Goal(const mjData& data) const;

  // The six commands Update last wrote, lead terms included. Empty before
  // the first Update.
  const Eigen::VectorXd& Commanded() const { return commanded_; }

  // Measured angles minus planned angles, at data.time.
  Eigen::VectorXd TrackingError(const mjData& data) const;

  const Arm& arm() const { return arm_; }

 private:
  // One leg of a plan, starting at `start_time` in simulation time.
  struct Leg {
    JointMove move;
    double start_time;
  };

  // Takes ctrl as the held pose if nothing has been adopted yet.
  void Adopt(const mjData& data);

  // The pose held once the plan has ended: adopted, set by Hold, or the
  // end of the last plan. Read from ctrl if nothing has been adopted yet.
  Eigen::VectorXd HeldPose(const mjData& data) const;

  // Why `goal` cannot be a goal, or empty if it can.
  std::string CheckGoal(const Eigen::VectorXd& goal) const;

  Arm arm_;
  FollowerSettings settings_;
  bool adopted_ = false;
  Eigen::VectorXd held_;   // valid once adopted_
  std::vector<Leg> legs_;  // the plan, in time order; empty if none
  Eigen::VectorXd commanded_;

  // model.nv long, for the model feedforward; kept to avoid allocating
  // every tick.
  Eigen::VectorXd acceleration_;
  Eigen::VectorXd torque_;
};

}  // namespace cell

#endif  // SRC_ARM_FOLLOWER_H_
