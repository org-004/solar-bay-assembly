// The whole cell, loaded and ready to run: the one object that owns the model
// and the state, finds every part in them, and sets up a scene for each trial.
//
// models/cell.xml puts the parts together: the arm and its suction tool
// (src/arm.h, src/suction.h), the pallet and its modules (src/pallet.h), the
// conveyor and its tube (src/conveyor.h), and a fixed depth camera. This class
// holds the found parts so that nothing else has to look them up again.
//
// A trial goes:
//
//   std::optional<Cell> cell = Cell::Load("models/cell.xml", &error);
//   cell->Reset(seed);             // arm at home, pallet a little off, at rest
//   ...                            // control code writes the arm's ctrl
//   cell->Step();                  // once per physics step
//   cell->Grip(); cell->Release(); // the suction, with the bookkeeping
//   cell->conveyor().StartStep(*cell->data());
//   cell->SwapTube();              // when a tube is full
//
// What control code may read, and what it may not. A real cell knows its
// joint angles, its conveyor's encoder, its camera's picture and the fixed
// pose of its camera; it does not know where the modules or the pallet truly
// are. Here everything is in mjData, so the line is drawn by name:
//
//   Allowed      JointAngles (src/arm.h), conveyor().Sensor..., the camera
//                (DepthCamera: calibration, fixed), what the control code did
//                itself (Grip's result, steps()).
//   True...      What really is: where modules and the pallet are, what was
//                drawn at Reset, which slot a module landed in. For setting up
//                scenes, for tests and for the scorer (src/score.h) only.
//                Control code that reads these is cheating.
//
// data() is writable because control code has to write the arm's ctrl. It
// must not read the modules' or the pallet's state out of it.
//
// Poses are Eigen::Isometry3d. The parts use their own small types
// (WorldPose in pallet.h, Pose in suction.h); this file converts at the
// boundary and leaves them as they are.

#ifndef SRC_CELL_H_
#define SRC_CELL_H_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/conveyor.h"
#include "src/mj_handles.h"
#include "src/pallet.h"
#include "src/suction.h"

namespace cell {

// How far each trial's scene may be off nominal. Reset draws each value
// uniformly between minus and plus the figure given. All zero gives the
// nominal scene.
struct Randomization {
  // The pallet is set down a few centimetres and a few degrees off where it
  // should be: in x and in y, and turned about the vertical through its
  // origin (the floor under module_0's glass centre).
  double pallet_xy = 0.03;              // m
  double pallet_yaw = 0.05235987755983; // rad, 3 degrees

  // Standard deviation of each conveyor stop's error (see
  // ConveyorSettings::stop_error_stddev). Not uniform: the conveyor draws it
  // from a normal distribution itself.
  double conveyor_stop_error = 0.002;  // m

  // No randomness at all.
  static Randomization None() { return {0.0, 0.0, 0.0}; }
};

// What Reset drew for one seed. Truth: control code must not read it.
struct TrueScene {
  std::uint32_t seed = 0;
  double pallet_x = 0.0;    // m, pallet moved from nominal along world X
  double pallet_y = 0.0;    // m, along world Y
  double pallet_yaw = 0.0;  // rad, turned about the vertical
  std::uint32_t conveyor_seed = 0;  // seeds the conveyor's stopping errors
};

// Where each module is in the bookkeeping. Changed only by the calls named.
enum class ModuleState {
  kOnPallet,  // standing in the stack since Reset
  kHeld,      // on the suction tool (Grip)
  kReleased,  // let go (Release, or TeleportModule); normally on the tube
  kParked,    // carried away (ParkModule, SwapTube)
};

// The depth camera as a pinhole: where it is and how it maps the world onto
// its pixels. Fixed, like a calibrated real camera, so control code may use
// it.
//
// Frame: MuJoCo's camera convention. The camera looks along its own -Z; +X is
// to the right in the image and +Y up. Pixels: u to the right and v down,
// from the top-left corner of the image, so the top-left pixel covers
// [0, 1) x [0, 1) and its centre is (0.5, 0.5). mjr_readPixels returns rows
// bottom first; flip them to get this v.
struct CameraModel {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // camera in world
  int width = 0;        // pixels
  int height = 0;
  double fovy = 0.0;    // rad, vertical field of view
  double focal = 0.0;   // pixels, the same along u and v
  double cx = 0.0;      // principal point, pixels: the image centre
  double cy = 0.0;
};

// Where a world point lands in the image, or nothing if it is behind the
// camera (or level with it). The result may lie outside the image; check
// against width and height.
std::optional<Eigen::Vector2d> Project(const CameraModel& camera,
                                       const Eigen::Vector3d& world);

// Where module i is put by ParkModule: flat on the floor, glass up, long
// side along X, in a grid of 5 along X by 6 along Y far off in +Y, x 0.85 to
// 13.55 and y 11.45 to 19.55 for the 30 modules, 0.3 m apart. Nothing else
// in the cell is there, and it is well outside the camera's picture.
Eigen::Isometry3d ParkingPose(int i);

class Cell {
 public:
  // How long Reset lets the physics run before handing over, seconds. The
  // stack is built touching but not pressed together and takes up its weight
  // within a few tenths of a second (src/pallet.h); the arm takes up its own
  // sag in under a tenth. After this every module moves less than 0.1 mm in
  // the next 10 s (tests/cell_test.cc).
  static constexpr double kSettleTime = 1.0;

  // Loads the cell from `path` (models/cell.xml) and finds every part: the
  // arm, the pallet and its modules, the conveyor, the suction weld, the
  // depth camera and the keyframes arm_home and arm_clear. Returns nothing,
  // and says what is missing in `error` if it is not null, if loading fails
  // or a part is not there. The state is as built, not yet Reset.
  static std::optional<Cell> Load(const std::string& path,
                                  std::string* error = nullptr,
                                  const ConveyorSettings& conveyor = {});

  // Sets up a new trial. Clears the state (time back to 0), puts the arm at
  // arm_home with its motors commanded there too, draws the pallet's offset
  // from `seed` (see DrawScene) and places the pallet and its stack there,
  // starts a fresh conveyor with its stopping errors seeded from `seed`,
  // lets go of anything the suction held, marks every module as on the
  // pallet, and runs kSettleTime seconds of physics so everything is at rest.
  // Then records where the stack stands, for StackDisturbance.
  //
  // The same seed and randomization give the same state, bit for bit.
  // Returns false and changes nothing if a figure in `randomization` is
  // negative or not finite.
  bool Reset(std::uint32_t seed, const Randomization& randomization = {});

  // What Reset draws for `seed`, without touching any state. Reset uses
  // exactly this. The draws use std::mt19937, whose sequence the C++ standard
  // fixes, and no library distribution, so they are the same on every
  // platform.
  static TrueScene DrawScene(std::uint32_t seed,
                             const Randomization& randomization);

  // One physics step: the conveyor writes its setpoint, then mj_step. The
  // arm's motors are left as the control code last set them.
  void Step();

  // Steps until `seconds` more of simulated time have passed.
  void StepFor(double seconds);

  // ----------------------------------------------------------- the suction
  // Grips the module whose glass the tool tip is on (Suction::Grip) and marks
  // it held. Returns its index i (module_<i>), or nothing, with the reason
  // in `failure` if it is not null.
  std::optional<int> Grip(GripFailure* failure = nullptr);

  // Lets the held module go and marks it released. If it is then over the
  // tube, it is recorded as lying in the nearest slot (see TruePlacedSlot).
  // Returns the module's index, or nothing if nothing was held.
  std::optional<int> Release();

  // ------------------------------------------------ carrying modules away
  // Teleports module i to ParkingPose(i), at rest, and marks it parked. If
  // the tool holds it, lets it go first. Runs mj_forward.
  void ParkModule(int i);

  // Swaps the full tube for an empty one: parks every module recorded as
  // lying on the tube (TruePlacedSlot has a value), then calls
  // Conveyor::SwapTube, which puts the slide back at its start. Returns false
  // and does nothing if the conveyor is still moving.
  //
  // "Lying on the tube" is the record made when a module was let go
  // (Release or TeleportModule): its centre was then above the rails of a
  // slot. A module let go anywhere else is not on the tube and stays where it
  // is.
  bool SwapTube();

  // -------------------------------------------- setting up scenes and tests
  // Teleports the arm to joint angles `q`, commands its motors there, at
  // rest, and runs mj_forward. For tests and for setting up scenes; control
  // code moves the arm through ctrl.
  void PutArm(const Eigen::VectorXd& q);

  // Teleports module i to `pose`, at rest, runs mj_forward, and marks it
  // released exactly as Release would (so it is recorded in a slot if it is
  // over the tube's rails). For tests: a module laid in its slot this way
  // scores as if the arm had put it there.
  void TeleportModule(int i, const Eigen::Isometry3d& pose);

  // Records where every module stands now as the stack's baseline, for
  // StackDisturbance. Reset does this after settling; call it after a pick
  // to measure only what the next pick disturbs.
  void RebaselineStack();

  // ---------------------------------------------------------------- access
  mjModel* model() { return model_.get(); }
  const mjModel* model() const { return model_.get(); }
  mjData* data() { return data_.get(); }
  const mjData* data() const { return data_.get(); }

  const Arm& arm() const { return arm_; }
  const PalletBodies& pallet() const { return pallet_; }
  Conveyor& conveyor() { return *conveyor_; }
  const Conveyor& conveyor() const { return *conveyor_; }
  // Read only: grip and release through Grip and Release, which keep the
  // bookkeeping. Its HeldPose... accessors are truth.
  const Suction& suction() const { return *suction_; }

  int module_count() const { return static_cast<int>(pallet_.modules.size()); }
  ModuleState module_state(int i) const { return state_.at(i); }

  // The arm's two named poses, from the keyframes.
  const Eigen::VectorXd& arm_home() const { return arm_home_; }
  const Eigen::VectorXd& arm_clear() const { return arm_clear_; }

  // The depth camera, as of the last mj_forward or mj_step (it never moves).
  CameraModel DepthCamera() const;
  int depth_camera_id() const { return camera_; }

  // ------------------------------------------------------------------ truth
  // For setting up scenes, tests and scoring only.

  // What the last Reset drew.
  const TrueScene& true_scene() const { return scene_; }

  // Where module i's centre and axes truly are now (body frame: +x the long
  // side, +y the short side, +z out of the glass).
  Eigen::Isometry3d TrueModulePose(int i) const;

  // Module i's linear speed (m/s) and turning speed (rad/s) relative to the
  // tube, which is the world while the conveyor stands still. As of the last
  // mj_forward; after mj_step it is the speed at the start of that step.
  Eigen::Vector2d TrueModuleSpeedOnTube(int i) const;

  // The slot module i was recorded in when it was let go, or nothing if it
  // was not let go over the tube, or has since been parked.
  std::optional<int> TruePlacedSlot(int i) const { return placed_slot_.at(i); }

  // Where every module stood at the last Reset or RebaselineStack.
  const std::vector<Eigen::Isometry3d>& true_stack_baseline() const {
    return stack_baseline_;
  }

 private:
  Cell() = default;

  // Marks module i released and records its slot from where it truly is.
  void RecordRelease(int i);

  // Module index for a body id, or nothing.
  std::optional<int> ModuleIndexOf(int body) const;

  ModelPtr model_;
  DataPtr data_;
  Arm arm_;
  PalletBodies pallet_;
  ConveyorSettings conveyor_settings_;
  std::optional<Conveyor> conveyor_;
  std::optional<Suction> suction_;
  int tube_ = -1;    // body id
  int camera_ = -1;  // camera id
  Eigen::VectorXd arm_home_;  // joint angles, also the motor commands
  Eigen::VectorXd arm_clear_;

  TrueScene scene_;
  std::vector<ModuleState> state_;
  std::vector<std::optional<int>> placed_slot_;
  std::vector<Eigen::Isometry3d> stack_baseline_;
};

}  // namespace cell

#endif  // SRC_CELL_H_
