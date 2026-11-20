// The conveyor that carries the torque tube past the arm, one step at a time.
//
// The tube lies along world Y on a conveyor on the -X side of the arm, with
// mounting rails across it, a pair for each module. The arm lays a module on
// the pair of rails at the placement spot; then the conveyor moves the tube
// toward -Y by one step, a module's short side plus the gap between modules,
// and the next empty pair arrives at the same spot. models/conveyor.xml is
// the model; the numbers below must agree with it, and
// tests/conveyor_test.cc checks that they do.
//
// A real conveyor does not stop exactly where it is told. Each move here gets
// a random stopping error, drawn when the move starts and added to its
// target, so the tube really does stop off the nominal spot. The drive is a
// servo with an encoder and goes to absolute positions: after n steps it aims
// for NominalSlidePosition(n), wherever the last stop really was. So the
// errors are independent from stop to stop and do not add up.
//
// A tube holds kSlotCount modules. When it is full, the cell carries the
// modules away and calls SwapTube, which puts an empty tube in its place: the
// slide goes back to the start and the step count to zero.
//
// Two kinds of value come out of this class, and they are kept apart by name:
//
//   Sensor...   What the cell's control code may use: the conveyor's
//               encoder, as MuJoCo's sensors report it. Moving() is also
//               based only on these.
//   True...     What really happened, including the stopping error. Only
//               for the code that sets up scenes and the code that scores a
//               run. Control code that reads these is cheating.
//
// nominal_position() is neither: it is where a naive controller that counts
// steps would think the tube is.

#ifndef SRC_CONVEYOR_H_
#define SRC_CONVEYOR_H_

#include <optional>
#include <random>

#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/profile.h"

namespace cell {

struct ConveyorSettings {
  // Limits for the move. The tube carries modules by friction alone, so the
  // acceleration has to stay well under friction times g: 1 m/s^2 needs a
  // friction of only 0.1, against about 0.5 for aluminium on steel. With
  // these one step takes about 3 s.
  Limits limits = {.velocity = 0.5, .acceleration = 1.0, .jerk = 4.0};

  // Standard deviation of the stopping error of each move, metres. Zero
  // turns the error off.
  double stop_error_stddev = 0.002;

  // Moving() turns false once the setpoint has reached its end and the
  // encoder reads within this of the stop, and slower than in_position_speed.
  double in_position_tolerance = 0.0005;  // m
  double in_position_speed = 0.001;       // m/s
};

class Conveyor {
 public:
  // ------------------------------------------------------------ the layout
  // A module, as it lies on the tube: long side along world X, short side
  // along world Y, glass up.
  static constexpr double kModuleLength = 2.3;
  static constexpr double kModuleWidth = 1.1;
  static constexpr double kModuleThickness = 0.035;

  // Space between neighbouring modules on the tube. Not yet settled in the
  // project notes; 2 cm is a guess at what the clamps between modules need.
  static constexpr double kModuleGap = 0.02;

  // How far one step moves the tube, and the distance between slots.
  static constexpr double kStepLength = kModuleWidth + kModuleGap;

  // Slots on a tube, as on a real torque tube section. A pallet's 30
  // modules fill three tubes.
  static constexpr int kSlotCount = 10;

  // The tube moves toward -Y: placed modules leave toward -Y, empty rails
  // arrive from +Y. Slot k is at tube-frame y = +k * kStepLength.
  static constexpr double kTravelDirection = -1.0;

  // The tube's axis, and the height of the rails' top surface.
  static constexpr double kTubeAxisX = -2.0;
  static constexpr double kRailTopHeight = 0.80;

  // Where a module is put down: its centre and orientation in the world.
  // Centre (-2.0, 0, 0.8175); body axes along the world's, so +x (long side)
  // along X and +z (out of the glass) up. Fixed: the same for every step.
  static Eigen::Isometry3d PlacementPose();

  // Where slot `slot`'s module sits in the world when the slide joint is at
  // `slide_position`. Pass NominalSlidePosition(n) to get the pose a naive
  // controller expects after n steps; SlotPose(n, NominalSlidePosition(n))
  // is PlacementPose().
  static Eigen::Isometry3d SlotPose(int slot, double slide_position);

  // Where slot `slot`'s module sits in the tube's frame (body "tube": origin
  // at the centre of slot 0 on the rails' top surface, axes along the
  // world's).
  static Eigen::Isometry3d SlotPoseOnTube(int slot);

  // The slide position after `steps` steps, if every one were exact.
  static double NominalSlidePosition(int steps);

  // --------------------------------------------------------------- the drive
  // Finds conveyor_slide, conveyor_motor, conveyor_position,
  // conveyor_velocity and the body tube in `model`. Returns nothing if one is
  // missing or of the wrong kind, or a setting is not usable. `seed` seeds
  // the stopping errors, so the same seed gives the same run. Keeps a
  // pointer to `model`, which must outlive the Conveyor.
  static std::optional<Conveyor> Find(const mjModel& model,
                                      const ConveyorSettings& settings = {},
                                      std::mt19937::result_type seed = 1);

  // Starts a move of one step, beginning at data.time: from where the last
  // move really stopped to NominalSlidePosition(steps() + 1) plus a newly
  // drawn stopping error. Returns false and does nothing if the conveyor is
  // still moving, or if the step would leave the slide's range: after
  // kSlotCount steps the tube is full and has to be swapped.
  bool StartStep(const mjData& data);

  // Replaces the tube with an empty one: puts the slide back at its start
  // with zero velocity, sets the motor's ctrl to match so the tube does not
  // lurch, sets steps() to zero, adds one to tubes_completed(), and runs
  // mj_forward so the sensors and poses read the new state at once. Returns
  // false and does nothing if the conveyor is still moving.
  //
  // The tube is swapped, not the modules on it: anything lying on the rails
  // would be thrown along with the slide. The caller must carry the modules
  // away BEFORE calling this.
  bool SwapTube(mjData* data);

  // Tubes swapped out so far.
  int tubes_completed() const { return tubes_completed_; }

  // Writes the setpoint for data.time into the motor's ctrl. Call every
  // control tick, before mj_step, whether or not a move is running: between
  // moves it holds the tube where it stopped.
  void Update(mjData* data) const;

  // True from StartStep until the setpoint has finished and the encoder says
  // the tube is at rest at the stop.
  bool Moving(const mjData& data) const;

  // How long a step's setpoint takes, seconds. The tube settles a little
  // after.
  double StepDuration() const;

  // Steps started on the current tube.
  int steps() const { return steps_; }

  // Where a controller that only counts steps would think the slide is:
  // NominalSlidePosition(steps()).
  double nominal_position() const { return NominalSlidePosition(steps_); }

  // --------------------------------------------------------------- sensor
  // The encoder: the slide's position, metres, and speed, metres per second.
  // As of the last mj_step or mj_forward.
  double SensorPosition(const mjData& data) const;
  double SensorVelocity(const mjData& data) const;

  // ---------------------------------------------- truth: scenes and scoring
  // Where the current or last move really ends, stopping error included.
  double true_stop_position() const { return target_; }

  // The stopping error of the current or last move: how far its stop is from
  // its nominal position. Zero before the first move on a tube.
  double true_last_stop_error() const { return last_error_; }

  // The slide joint's position, from the simulation's state.
  double TrueSlidePosition(const mjData& data) const;

  // The tube's frame in the world.
  Eigen::Isometry3d TrueTubePose(const mjData& data) const;

  // Where slot `slot`'s module should be in the world, given where the tube
  // really is. A scorer compares a module's pose with this.
  Eigen::Isometry3d TrueSlotPose(const mjData& data, int slot) const;

 private:
  Conveyor() = default;

  ConveyorSettings settings_;
  std::mt19937 random_;
  const mjModel* model_ = nullptr;

  // Indices into the model and state.
  int actuator_ = -1;
  int qpos_address_ = -1;
  int dof_address_ = -1;
  int position_sensor_address_ = -1;
  int velocity_sensor_address_ = -1;
  int tube_body_ = -1;
  bool range_limited_ = false;
  double range_low_ = 0.0;
  double range_high_ = 0.0;
  double start_position_ = 0.0;  // the slide's position in the model's qpos0

  // The current or last move. Before the first, no profile: hold at target_.
  std::optional<Profile> move_;
  double move_start_time_ = 0.0;
  double move_start_position_ = 0.0;
  double target_ = 0.0;
  double last_error_ = 0.0;
  int steps_ = 0;
  int tubes_completed_ = 0;
};

}  // namespace cell

#endif  // SRC_CONVEYOR_H_
