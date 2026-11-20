// The suction grip: holding a module on the tool, and letting it go.
//
// MuJoCo has no suction cup. The stand-in is a single weld constraint, named
// "suction" in models/suction.xml, between the tool and whichever module is
// being held. While nothing is held the weld is switched off. Grip checks
// that tool_tip is on a module's glass, then writes into the model's copy of
// the weld which module it binds and the pose the module has relative to the
// tool at that moment, and switches it on. From then on the module moves with
// the tool, keeping that relative pose, until Release switches the weld off.
//
// Capturing the pose as it is matters: the arm arrives a few millimetres and
// a degree or so off, the module stays held with exactly that offset, and the
// placement error measured afterwards includes it. Nothing is snapped square.
//
// The weld lives in the model (mjModel), not the state: Grip writes
// eq_obj2id and eq_data, and switches the weld on in mjData.eq_active. Every
// mjData made from the same model therefore shares the choice of module.
// mj_resetData switches the weld off (eq_active goes back to the model's
// "off"), so call Release, or make a new Suction, after a reset.

#ifndef SRC_SUCTION_H_
#define SRC_SUCTION_H_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

namespace cell {

struct SuctionOptions {
  // Names in the model. The defaults are those of models/tool.xml,
  // models/suction.xml and the modules of the cell.
  std::string tool_body = "tool";
  std::string tip_site = "tool_tip";
  std::string weld = "suction";
  std::string module_prefix = "module_";  // every body whose name starts so

  // A module's full size: long side (its x), short side (y), thickness (z).
  // The glass is the +z face, at z = +size.z() / 2 in the module's frame.
  Eigen::Vector3d module_size{2.3, 1.1, 0.035};

  // How close to the glass tool_tip must be: its distance from the plane of
  // the glass, in front of it or pressed behind it, at most this. Real cups
  // on bellows reach out about this far.
  double max_distance = 0.010;  // m

  // How square the tool must be: the angle between the tool's +Z and the
  // direction into the glass (the module's -z), at most this.
  double max_angle = 0.0524;  // rad, 3 degrees

  // How far inside the outline of the glass tool_tip must be. 0 means
  // anywhere on the face; a positive value keeps it that far from every edge.
  double edge_margin = 0.0;  // m
};

// Where one frame is relative to another: position and orientation of the
// child frame, expressed in the parent frame.
struct Pose {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
};

// Why a grip did not happen.
struct GripFailure {
  enum class Kind {
    kAlreadyHolding,  // a module is held already; release it first
    kTooFar,          // tool_tip is not within max_distance of any glass
    kOffFace,         // in the plane of the glass, but outside its outline
    kTooTilted,       // on the glass, but the tool is not square to it
  };
  Kind kind = Kind::kTooFar;
  std::string message;  // the same in words, with the numbers
};

class Suction {
 public:
  // Finds the tool, its tip, the weld and the modules by the names in
  // `options`. Returns nothing, and says why in `error` if it is not null,
  // when one is missing or the weld is not a weld between bodies whose first
  // body is the tool.
  static std::optional<Suction> Create(const mjModel& model,
                                       const SuctionOptions& options = {},
                                       std::string* error = nullptr);

  // Grips the module whose glass tool_tip is on: within max_distance of the
  // plane of the glass, inside its outline (less edge_margin), and with the
  // tool within max_angle of square. If several qualify, the one whose
  // glass is nearest. Holds it at the pose it has relative to the tool right
  // now. Returns the gripped module's body id. Returns nothing if no module
  // qualifies, or one is held already, and says why in `failure` if it is not
  // null; about the module whose glass is nearest tool_tip.
  //
  // Recomputes the positions in `data` from data->qpos first (mj_kinematics),
  // so it may be called straight after mj_step.
  std::optional<int> Grip(mjModel* model, mjData* data,
                          GripFailure* failure = nullptr);

  // Lets the held module go: switches the weld off. Nothing happens if
  // nothing is held.
  void Release(mjData* data);

  // The body id of the module held, or nothing.
  std::optional<int> holding() const { return held_; }

  // Body ids of the modules found, in the model's order.
  const std::vector<int>& modules() const { return modules_; }

  // ------------------------------------------------------------------ truth
  // For tests and the scorer only. Control code must not use these: they
  // read the module's true pose from the simulation, which a real cell does
  // not know.

  // The held module's frame relative to the tool_tip frame, now, from the
  // positions in `data` as last computed (by mj_forward or mj_step).
  std::optional<Pose> HeldPoseInTip(const mjData& data) const;

  // The same at the moment of the grip: the pose the weld holds.
  std::optional<Pose> HeldPoseInTipAtGrip() const;

 private:
  Suction(const SuctionOptions& options, int tool, int tip, int weld,
          std::vector<int> modules)
      : options_(options),
        tool_(tool),
        tip_(tip),
        weld_(weld),
        modules_(std::move(modules)) {}

  SuctionOptions options_;
  int tool_ = -1;  // body id
  int tip_ = -1;   // site id
  int weld_ = -1;  // equality id
  std::vector<int> modules_;

  std::optional<int> held_;
  Pose held_at_grip_;
};

}  // namespace cell

#endif  // SRC_SUCTION_H_
