#include "src/cell.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/camera_model.h"
#include "src/conveyor.h"
#include "src/depth_camera.h"
#include "src/mj_handles.h"
#include "src/pallet.h"
#include "src/suction.h"

namespace cell {
namespace {

// The parking grid: columns along X, rows along Y. A module lying flat is
// 2.3 m along X and 1.1 m along Y; 0.3 m between neighbours.
constexpr int kParkingColumns = 5;
constexpr double kParkingFirstX = 2.0;
constexpr double kParkingFirstY = 12.0;
constexpr double kParkingPitchX = 2.6;
constexpr double kParkingPitchY = 1.4;

// A module let go counts as lying on the tube if its centre is within the
// rails' length across the tube (they run 1.0 m each way from its axis),
// along it within half a step of one of the tube's slots, and no more than
// this above the rails' top surface, or a little below it.
constexpr double kRailHalfLength = 1.0;     // m, as in models/conveyor.xml
constexpr double kOnTubeAbove = 0.5;        // m
constexpr double kOnTubeBelow = 0.05;       // m

// A uniform draw in [-range, range] from one 32-bit output of the generator.
// std::uniform_real_distribution is not used: the standard leaves its
// algorithm to each library, and a seed must give the same scene everywhere.
double Uniform(std::mt19937& random, double range) {
  if (range == 0.0) {
    random();  // keep the sequence the same whatever the ranges
    return 0.0;
  }
  // (k + 0.5) / 2^32 is strictly between 0 and 1.
  const double unit = (static_cast<double>(random()) + 0.5) / 4294967296.0;
  return range * (2.0 * unit - 1.0);
}

Eigen::Isometry3d FromWorldPose(const WorldPose& pose) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = pose.position;
  result.linear() = pose.orientation.normalized().toRotationMatrix();
  return result;
}

// Writes `pose` into a free joint's seven position numbers and zeroes its
// six velocities, and the solver's memory of them.
void SetFreeJoint(const mjModel& model, mjData* data, int body,
                  const Eigen::Isometry3d& pose) {
  const int joint = model.body_jntadr[body];
  mjtNum* qpos = data->qpos + model.jnt_qposadr[joint];
  const Eigen::Quaterniond q(pose.rotation());
  qpos[0] = pose.translation().x();
  qpos[1] = pose.translation().y();
  qpos[2] = pose.translation().z();
  // MuJoCo's order is w, x, y, z.
  qpos[3] = q.w();
  qpos[4] = q.x();
  qpos[5] = q.y();
  qpos[6] = q.z();
  const int dof = model.jnt_dofadr[joint];
  for (int k = 0; k < 6; ++k) {
    data->qvel[dof + k] = 0.0;
    data->qacc_warmstart[dof + k] = 0.0;
  }
}

// The six joint angles a keyframe gives the arm, or nothing if there is no
// keyframe of that name.
std::optional<Eigen::VectorXd> KeyAngles(const mjModel& model, const Arm& arm,
                                         const char* name) {
  const int key = mj_name2id(&model, mjOBJ_KEY, name);
  if (key < 0) return std::nullopt;
  Eigen::VectorXd q(Arm::kJoints);
  for (int j = 0; j < Arm::kJoints; ++j) {
    q[j] = model.key_qpos[key * model.nq + arm.qpos[j]];
  }
  return q;
}

}  // namespace

Eigen::Isometry3d ParkingPose(int i) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
      Eigen::Vector3d(kParkingFirstX + kParkingPitchX * (i % kParkingColumns),
                      kParkingFirstY + kParkingPitchY * (i / kParkingColumns),
                      kModuleThickness / 2.0);
  return pose;
}

std::optional<Cell> Cell::Load(const std::string& path, std::string* error,
                               const ConveyorSettings& conveyor) {
  auto fail = [&](const std::string& message) -> std::optional<Cell> {
    if (error != nullptr) *error = message;
    return std::nullopt;
  };

  Cell cell;
  std::string load_error;
  cell.model_ = LoadModel(path, &load_error);
  if (!cell.model_) return fail("could not load " + path + ": " + load_error);
  cell.data_ = MakeData(*cell.model_);
  if (!cell.data_) return fail("could not allocate the state for " + path);
  const mjModel& model = *cell.model_;

  std::optional<Arm> arm = FindArm(model);
  if (!arm) {
    return fail("no arm: arm_joint_1..6, arm_motor_1..6, tool and tool_tip "
                "(include arm.xml)");
  }
  cell.arm_ = *arm;

  std::optional<PalletBodies> pallet = FindPallet(model);
  if (!pallet) {
    return fail("no pallet: mocap body pallet and bodies module_0.. "
                "(include pallet.xml)");
  }
  cell.pallet_ = *pallet;

  cell.conveyor_settings_ = conveyor;
  cell.conveyor_ = Conveyor::Find(model, conveyor);
  if (!cell.conveyor_) {
    return fail("no conveyor, or unusable conveyor settings: conveyor_slide, "
                "conveyor_motor, conveyor_position, conveyor_velocity, tube "
                "(include conveyor.xml)");
  }

  std::string suction_error;
  cell.suction_ = Suction::Create(model, {}, &suction_error);
  if (!cell.suction_) return fail("no suction: " + suction_error);
  // Suction finds modules by name prefix and the pallet by number; they have
  // to be the same bodies or the bookkeeping would not line up.
  if (cell.suction_->modules().size() != cell.pallet_.modules.size()) {
    return fail("the suction and the pallet see different modules");
  }

  cell.tube_ = mj_name2id(&model, mjOBJ_BODY, "tube");  // Find checked it

  cell.camera_ = mj_name2id(&model, mjOBJ_CAMERA, "depth_camera");
  if (cell.camera_ < 0) return fail("no camera named depth_camera");
  if (!CameraModelFromMujoco(model, *cell.data_, cell.camera_)) {
    return fail("depth_camera is not a perspective camera with square pixels");
  }

  cell.check_camera_ = mj_name2id(&model, mjOBJ_CAMERA, "check_camera");
  if (cell.check_camera_ < 0) return fail("no camera named check_camera");
  if (!CameraModelFromMujoco(model, *cell.data_, cell.check_camera_)) {
    return fail("check_camera is not a perspective camera with square pixels");
  }

  std::optional<Eigen::VectorXd> home = KeyAngles(model, cell.arm_, "arm_home");
  std::optional<Eigen::VectorXd> clear =
      KeyAngles(model, cell.arm_, "arm_clear");
  if (!home || !clear) return fail("no keyframe arm_home or arm_clear");
  cell.arm_home_ = *home;
  cell.arm_clear_ = *clear;

  const int n = cell.module_count();
  cell.state_.assign(n, ModuleState::kOnPallet);
  cell.placed_slot_.assign(n, std::nullopt);
  mj_forward(cell.model_.get(), cell.data_.get());
  cell.RebaselineStack();
  return cell;
}

TrueScene Cell::DrawScene(std::uint32_t seed,
                          const Randomization& randomization) {
  std::mt19937 random(seed);
  TrueScene scene;
  scene.seed = seed;
  // Always four draws in this order, so changing one range leaves the
  // others' values as they were.
  scene.pallet_x = Uniform(random, randomization.pallet_xy);
  scene.pallet_y = Uniform(random, randomization.pallet_xy);
  scene.pallet_yaw = Uniform(random, randomization.pallet_yaw);
  scene.conveyor_seed = random();
  // The camera's draws come after the four above, so adding them left every
  // earlier seed's scene as it was. The calibration error's directions:
  // uniform over the sphere, from a uniform height and a uniform angle
  // round (Archimedes' hat-box theorem).
  auto direction = [&random]() {
    const double z = Uniform(random, 1.0);
    const double angle = Uniform(random, std::numbers::pi);
    const double r = std::sqrt(std::max(0.0, 1.0 - z * z));
    return Eigen::Vector3d(r * std::cos(angle), r * std::sin(angle), z);
  };
  scene.camera_shift = randomization.camera_shift * direction();
  scene.camera_turn = randomization.camera_turn * direction();
  scene.camera_noise = randomization.camera_noise;
  scene.camera_noise_seed = random();
  // The check camera's draws come last, for the same reason.
  scene.check_camera_shift = randomization.check_camera_shift * direction();
  scene.check_camera_turn = randomization.check_camera_turn * direction();
  scene.check_noise_seed = random();
  return scene;
}

bool Cell::Reset(std::uint32_t seed, const Randomization& randomization) {
  for (double figure : {randomization.pallet_xy, randomization.pallet_yaw,
                        randomization.conveyor_stop_error,
                        randomization.camera_shift, randomization.camera_turn,
                        randomization.check_camera_shift,
                        randomization.check_camera_turn}) {
    if (!std::isfinite(figure) || figure < 0.0) return false;
  }
  mjModel* model = model_.get();
  mjData* data = data_.get();

  // mj_resetData switches the suction weld off without the Suction object
  // knowing; Release brings it back in step.
  mj_resetData(model, data);
  suction_->Release(data);

  // The arm at home, its motors told to hold it there. The keyframe's ctrl
  // is read through the motors' own indices.
  const int home = mj_name2id(model, mjOBJ_KEY, "arm_home");
  for (int j = 0; j < Arm::kJoints; ++j) {
    data->qpos[arm_.qpos[j]] = model->key_qpos[home * model->nq + arm_.qpos[j]];
    data->ctrl[arm_.motor[j]] = model->key_ctrl[home * model->nu + arm_.motor[j]];
  }

  scene_ = DrawScene(seed, randomization);
  pictures_ = 0;
  check_pictures_ = 0;
  // Also runs mj_forward.
  PlacePallet(*model, data, pallet_, scene_.pallet_x, scene_.pallet_y,
              scene_.pallet_yaw);

  // A fresh conveyor: no steps, holding the slide at its start, its stopping
  // errors drawn from this trial's seed. Load checked that Find succeeds
  // with these settings on this model, and a stopping error that is finite
  // and not negative is always usable.
  ConveyorSettings settings = conveyor_settings_;
  settings.stop_error_stddev = randomization.conveyor_stop_error;
  conveyor_ = Conveyor::Find(*model, settings, scene_.conveyor_seed);

  state_.assign(module_count(), ModuleState::kOnPallet);
  placed_slot_.assign(module_count(), std::nullopt);

  StepFor(kSettleTime);
  // mj_step leaves positions computed before its last integration; bring
  // them up to date with the state so the baseline is where things are.
  mj_forward(model, data);
  RebaselineStack();
  return true;
}

void Cell::Step() {
  conveyor_->Update(data_.get());
  mj_step(model_.get(), data_.get());
}

void Cell::StepFor(double seconds) {
  // Counted in steps, not by comparing times, so that rounding in the sum of
  // timesteps cannot add or drop one.
  const long steps = std::lround(seconds / model_->opt.timestep);
  for (long k = 0; k < steps; ++k) Step();
}

std::optional<int> Cell::Grip(GripFailure* failure) {
  const std::optional<int> body =
      suction_->Grip(model_.get(), data_.get(), failure);
  if (!body) return std::nullopt;
  const std::optional<int> i = ModuleIndexOf(*body);
  if (i) {
    state_[*i] = ModuleState::kHeld;
    placed_slot_[*i].reset();
  }
  return i;
}

std::optional<int> Cell::Release() {
  const std::optional<int> body = suction_->holding();
  if (!body) return std::nullopt;
  suction_->Release(data_.get());
  const std::optional<int> i = ModuleIndexOf(*body);
  if (i) {
    mj_kinematics(model_.get(), data_.get());  // where it is now
    RecordRelease(*i);
  }
  return i;
}

void Cell::ParkModule(int i) {
  const int body = pallet_.modules.at(i);
  if (suction_->holding() == body) suction_->Release(data_.get());
  SetFreeJoint(*model_, data_.get(), body, ParkingPose(i));
  state_[i] = ModuleState::kParked;
  placed_slot_[i].reset();
  mj_forward(model_.get(), data_.get());
}

bool Cell::SwapTube() {
  if (conveyor_->Moving(*data_)) return false;
  for (int i = 0; i < module_count(); ++i) {
    if (placed_slot_[i]) ParkModule(i);
  }
  return conveyor_->SwapTube(data_.get());
}

void Cell::PutArm(const Eigen::VectorXd& q) {
  for (int j = 0; j < Arm::kJoints; ++j) {
    data_->qpos[arm_.qpos[j]] = q[j];
    data_->qvel[arm_.dof[j]] = 0.0;
    data_->qacc_warmstart[arm_.dof[j]] = 0.0;
    data_->ctrl[arm_.motor[j]] = q[j];
  }
  mj_forward(model_.get(), data_.get());
}

void Cell::TeleportModule(int i, const Eigen::Isometry3d& pose) {
  const int body = pallet_.modules.at(i);
  if (suction_->holding() == body) suction_->Release(data_.get());
  SetFreeJoint(*model_, data_.get(), body, pose);
  mj_forward(model_.get(), data_.get());
  RecordRelease(i);
}

void Cell::RebaselineStack() {
  stack_baseline_.clear();
  for (int i = 0; i < module_count(); ++i) {
    stack_baseline_.push_back(TrueModulePose(i));
  }
}

CameraModel Cell::DepthCamera() const {
  // Load checked that this camera can be described.
  return *CameraModelFromMujoco(*model_, *data_, camera_);
}

DepthImage Cell::TakeDepthImage() {
  // After mj_step the positions are from before its last integration.
  mj_kinematics(model_.get(), data_.get());
  DepthOptions options;
  options.noise.depth_noise = scene_.camera_noise;
  options.noise.edge_effects = scene_.camera_noise;
  options.noise.quantize = scene_.camera_noise;
  options.noise.seed = scene_.camera_noise_seed + static_cast<std::uint32_t>(pictures_);
  ++pictures_;
  return RenderDepth(*model_, data_.get(), DepthCamera(), options);
}

CameraModel Cell::CheckCamera() const {
  return *CameraModelFromMujoco(*model_, *data_, check_camera_);
}

DepthImage Cell::TakeCheckImage() {
  mj_kinematics(model_.get(), data_.get());
  DepthOptions options;
  options.noise.depth_noise = scene_.camera_noise;
  options.noise.edge_effects = scene_.camera_noise;
  options.noise.quantize = scene_.camera_noise;
  options.noise.seed =
      scene_.check_noise_seed + static_cast<std::uint32_t>(check_pictures_);
  ++check_pictures_;
  return RenderDepth(*model_, data_.get(), CheckCamera(), options);
}

CameraModel Cell::CalibratedCheckCamera() const {
  return Perturbed(CheckCamera(), scene_.check_camera_turn,
                   scene_.check_camera_shift);
}

CameraModel Cell::CalibratedCamera() const {
  return Perturbed(DepthCamera(), scene_.camera_turn, scene_.camera_shift);
}

Eigen::Isometry3d Cell::TrueModulePose(int i) const {
  return FromWorldPose(ModulePose(*data_, pallet_, i));
}

Eigen::Vector2d Cell::TrueModuleSpeedOnTube(int i) const {
  mjtNum module[6];  // angular, then linear; world axes
  mj_objectVelocity(model_.get(), data_.get(), mjOBJ_BODY,
                    pallet_.modules.at(i), module, /*flg_local=*/0);
  // The tube slides without turning, so its velocity is the same everywhere
  // on it.
  mjtNum tube[6];
  mj_objectVelocity(model_.get(), data_.get(), mjOBJ_BODY, tube_, tube,
                    /*flg_local=*/0);
  const Eigen::Vector3d linear(module[3] - tube[3], module[4] - tube[4],
                               module[5] - tube[5]);
  const Eigen::Vector3d angular(module[0], module[1], module[2]);
  return Eigen::Vector2d(linear.norm(), angular.norm());
}

void Cell::RecordRelease(int i) {
  state_[i] = ModuleState::kReleased;
  placed_slot_[i].reset();

  // Where its centre is in the tube's frame, whose y runs along the tube
  // with slot k at +k steps.
  const Eigen::Vector3d centre =
      conveyor_->TrueTubePose(*data_).inverse() *
      TrueModulePose(i).translation();
  const double along = -Conveyor::kTravelDirection * centre.y();
  const long slot = std::lround(along / Conveyor::kStepLength);
  if (slot < 0 || slot >= Conveyor::kSlotCount) return;
  if (std::abs(centre.x()) > kRailHalfLength) return;
  const double height = centre.z() - kModuleThickness / 2.0;
  if (height > kOnTubeAbove || height < -kOnTubeBelow) return;
  placed_slot_[i] = static_cast<int>(slot);
}

std::optional<int> Cell::ModuleIndexOf(int body) const {
  for (int i = 0; i < module_count(); ++i) {
    if (pallet_.modules[i] == body) return i;
  }
  return std::nullopt;
}

}  // namespace cell
