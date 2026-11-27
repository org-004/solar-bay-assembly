// The torque a timed move of the module arm demands of its motors, and the
// time scale at which a move stays within a fraction of them (run 045).
//
// The planner times every move from the datasheet's joint speeds and
// accelerations (src/path_timing.h, ArmMotion) and knows neither the payload
// nor the motors. In physics (src/physics_run.h) the chosen layout's module
// arm runs joint 1 at its torque limit. ArmDynamics measures, before any
// physics run, what the plan's timing asks of the motors: inverse dynamics
// (mj_inverse) of the module arm along the timed path.
//
// The model. LoadLayoutModel in the physics state (src/layout_model.h): the
// module arm with models/arm.xml's links, armature, damping and torque
// limits, its track with its servo's force limit (kTrackForce), the tool on
// its flange with the tool's mass, and, while the segment holds a module,
// the held module of the checker's model, 32.4 kg, rigid at the tool's
// working point (the physics run's suction weld holds it there, within the
// grip's offset of a few mm). One model per tool and held part, loaded once.
// Constraints and contacts are switched off in the private copies, so that
// qfrc_inverse is M qacc + bias - passive: the force each axis must give
// for the planned angle, speed and acceleration, its damping included. The
// other arms' trees share no axis with the module arm's and do not enter.
//
// The demand. For each instant, |qfrc_inverse| over the motor's limit
// (actuator_forcerange times the gear), for the six joints and then the
// track. A move's trajectory is sampled every kTorqueSampleS of its own
// time and at its end, with the trajectory's own speeds and accelerations;
// at time scale k the trajectory q(t / k) has speeds v / k and
// accelerations a / k^2. A leg (approach or retreat) is sampled as the
// physics run follows it: central differences of ConfigAt over
// kDifference (src/physics_run.h's figure, the same 1 ms). A work segment
// stands still: gravity alone, once.
//
// The bound (the layout's torque_limit_scale, src/layout.h). A move whose
// worst ratio r exceeds the fraction is played at k * sqrt(r / fraction),
// and again until it holds or kMaxTorqueRounds are spent; the scale is
// rounded up to kTorqueScaleStep so that the plan file holds it exactly.
// Gravity does not scale: a move whose demand at rest already exceeds the
// fraction cannot be helped, and TorqueTimeScale says so.

#ifndef SRC_LAYOUT_TORQUE_H_
#define SRC_LAYOUT_TORQUE_H_

#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/mj_handles.h"
#include "src/path_timing.h"

namespace cell::layout {

inline constexpr double kTorqueSampleS = 0.005;  // s of a move's own time.
inline constexpr double kTorqueDifference = 1e-3;  // s, as physics_run's kDifference.
inline constexpr int kMaxTorqueRounds = 20;
inline constexpr double kTorqueScaleStep = 1e-4;

// The peak demand over limit of a stretch of motion: six joints, then the
// track.
struct TorqueDemand {
  std::array<double, 7> ratio{};
  double Worst() const;
  int WorstAxis() const;  // 0..5 the joints, 6 the track.
};

// The arm that sets the modules ("" if none): the one the bound applies to.
std::string ModuleArm(const Layout& layout);

// A move's trajectory as the plan times it (src/layout_replay.cc's own).
std::optional<Trajectory> MoveTrajectory(const std::vector<Config>& path,
                                         const ArmMotion& motion, const PlanFigures& f);

class ArmDynamics {
 public:
  // Nullptr with *error set on a fault (no such arm, not a KUKA).
  static std::unique_ptr<ArmDynamics> Create(const Layout& layout, const ModelPaths& paths,
                                             const std::string& arm, std::string* error);
  ~ArmDynamics();
  ArmDynamics(const ArmDynamics&) = delete;
  ArmDynamics& operator=(const ArmDynamics&) = delete;
  ArmDynamics(ArmDynamics&&) = delete;
  ArmDynamics& operator=(ArmDynamics&&) = delete;

  // The limits: six joints in N m, then the track in N.
  const std::array<double, 7>& limits() const { return limits_; }
  // |force| over limit at one instant. False with *error on a fault.
  bool Ratio(const std::string& tool, const std::string& held, const Config& q,
             const Config& v, const Config& a, std::array<double, 7>* ratio,
             std::string* error);
  // Along a move's trajectory played time_scale slower.
  bool AlongMove(const Trajectory& trajectory, double time_scale, const std::string& tool,
                 const std::string& held, TorqueDemand* demand, std::string* error);
  // Along any segment of a plan (a move at its own time_scale).
  bool AlongSegment(const Segment& segment, const ArmMotion& motion, const PlanFigures& f,
                    TorqueDemand* demand, std::string* error);

 private:
  ArmDynamics() = default;
  struct Model {
    ModelPtr model;
    DataPtr data;
    std::array<int, 7> qpos{}, dof{};
  };
  Model* Get(const std::string& tool, const std::string& held, std::string* error);

  const Layout* layout_ = nullptr;
  ModelPaths paths_;
  std::string arm_;
  std::array<double, 7> limits_{};
  std::map<std::pair<std::string, std::string>, Model> models_;
};

// The time scale, at least 1, at which the move demands at most `fraction`
// of every limit, and the demand there. 1 when it holds already. Negative
// with *error set when it cannot be timed or does not hold after
// kMaxTorqueRounds (gravity alone over the fraction).
double TorqueTimeScale(ArmDynamics& dynamics, const Trajectory& trajectory,
                       const std::string& tool, const std::string& held, double fraction,
                       TorqueDemand* demand, std::string* error);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_TORQUE_H_
