// One module's cycle of a layout's committed plan, run in physics.
//
// The plan (src/layout_plan.h) is kinematic: every pose of every arm at every
// instant, checked for clearance, but never driven. PhysicsRun drives the
// module arm of one module's take and set through the plan's own timing in
// MuJoCo, with its real masses and position servos, and measures how closely
// it follows and how near it comes to everything else.
//
// The model. LoadLayoutModel in the physics state (src/layout_model.h,
// Physics): the module arm keeps models/arm.xml's six position servos, its
// track gets a servo, and the module it is about to take is a free body of
// 32.4 kg, held on its pallet by a weld until the grip. The bay is built as
// the plan has it when the take starts (rails, and the modules set before,
// as far as the builder's contiguous count draws them; they only show).
// Only the carried module touches anything: its geoms against the two rails
// it is set on, with the contact settings of tests/locate_physics_test.cc
// scaled to this timestep (solref twice the timestep's four, solimp 0.95 0.99
// 0.0005, elliptic cones, friction kLocateFriction). Nothing else is given
// contacts: what the plan keeps 15 mm apart is measured below, not left to
// collide. The other arms stand where the plan has them at each instant
// (their qpos written every step, at rest): kinematic, as the spec of run
// 019 allows.
//
// The run, in physics time from 0:
//   - hold_s at the take's first pose, the plan's clock stopped, so that the
//     servos' answer to gravity has settled before the plan starts; not
//     counted;
//   - the take step and the set step of the plan, end to end, in the plan's
//     time: from the take step's start (its move from where the arm last
//     was) to the set step's end (its retreat). ArmFollower (src/
//     arm_follower.h) follows a TimedPath whose angles are the plan's
//     (PlanPlayer::At, the very function the verifier and the viewer play)
//     and whose speeds and accelerations are central differences of them
//     over kDifference; its lag compensation and model feedforward are on.
//     The track's servo is driven the same way here: planned position, plus
//     lag times planned speed, plus the force the model says the planned
//     accelerations of all seven axes need (the mass matrix's row of the
//     track, and gravity and bias forces), over its gain; and the follower
//     is given the track's planned acceleration (SetCarrier), which is where
//     most of joint 1's torque comes from;
//   - at the end of the take's work segment the pallet's weld is switched
//     off and Suction::Grip (src/suction.h) binds the module at its pose as
//     found; at the end of the set's work segment Suction::Release lets it
//     go, as the plan's own rule says things change (at the END of a work
//     segment);
//   - settle_s more at the set's retreat pose, then the module's pose on its
//     rails is taken.
//
// The figures, for every segment and every step of the two:
//   - the largest joint tracking error, measured minus planned, in mrad, and
//     the track's in mm;
//   - the largest Cartesian error against the plan of the module's frame
//     while it is held, and otherwise of the tool's working point: in mm and
//     degrees. The plan's is the same frame in the checker's model at the
//     planned configuration. A module is the same turned half round about
//     its normal (its shape and its four holes are symmetric), and the plan
//     takes some modules from the pallet that way round, so its error is
//     the smaller of the two, as is the set-down's yaw below;
//   - the least distance in physics between the module arm with its
//     carriage, tool and held part and everything else (the bay as built,
//     the base, the pallets, the stocks, the items and every other arm), as
//     the verifier measures it: InstantDistances of src/layout_plan.h on the
//     checker's own model (capsules turned into cylinders), with the arm at
//     its measured joints and track, and the module's geoms moved to where
//     the physics has the module, held or set. The same function over the
//     plan's own world at the same instants gives the plan's figure beside
//     it. Sampled every sample_s of plan time;
//   - the peak torque of each motor over its limit.
// And for the run: the module's pose on its two rails after release and
// settling, against its designed place (the checker's module of that
// index): offset in the rails' plane, across and along the tube, yaw about
// the rails' normal, and height. The rails are fixed where the bay puts
// them, so "from the rails as they lie" is from their designed place.
//
// The same run gives the same output on one machine, byte for byte: one
// thread, fixed steps, no clock.

#ifndef SRC_PHYSICS_RUN_H_
#define SRC_PHYSICS_RUN_H_

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/physics_record.h"

namespace cell::layout {

struct PhysicsOptions {
  // The module_take index whose take and set are run; -1: the take before
  // the set whose carry (the move that holds the module) the planner needed
  // the most collision queries to find, the hardest carry of the plan.
  int take = -1;
  double timestep = 0.0005;  // s
  double hold_s = 1.0;       // Before the plan's clock starts.
  double settle_s = 2.0;     // After the set's retreat.
  double sample_s = 0.01;    // Clearance and frame error, in plan time.
};

// Central differences of the plan's angles, for speed and acceleration.
inline constexpr double kDifference = 1e-3;  // s

// The figures of one segment (or, summed up, of a step).
struct PhysicsFigures {
  int step = 0;
  std::string kind;              // The segment's kind, or "step".
  double t0 = 0.0, t1 = 0.0;     // Plan time.
  double joint_error_mrad = 0.0;
  double track_error_mm = 0.0;
  double frame_error_mm = 0.0;
  double frame_error_deg = 0.0;
  bool frame_is_module = false;  // Else the tool's working point.
  double clearance_m = kSceneFar;
  std::string clearance_pair;
  double clearance_t = 0.0;
  double plan_clearance_m = kSceneFar;
  std::array<double, 7> torque_ratio{};  // Six joints, then the track.
};

struct PhysicsResult {
  bool ok = false;
  std::string error;
  std::string layout, arm;
  int take = -1, module = -1;
  int take_step = 0, set_step = 0;
  std::vector<PhysicsFigures> segments;  // In time order.
  std::vector<PhysicsFigures> steps;     // The take, then the set.
  PhysicsFigures settle;                 // After the set's retreat.
  SetDown set_down;
  double grip_offset_mm = 0.0;           // Tool tip to glass at the grip.
  PhysicsRecord record;                  // The record of these figures.
  double cycle_s = 0.0;                  // The plan's bay_planned_s.
};

class PhysicsRun {
 public:
  // Loads everything and puts the arm at the take's first pose. Returns
  // nullptr and sets *error on a fault.
  static std::unique_ptr<PhysicsRun> Create(const Layout& layout, const Plan& plan,
                                            const ModelPaths& paths,
                                            const PhysicsOptions& options,
                                            std::string* error);
  ~PhysicsRun();
  PhysicsRun(const PhysicsRun&) = delete;
  PhysicsRun& operator=(const PhysicsRun&) = delete;
  PhysicsRun(PhysicsRun&&) = delete;
  PhysicsRun& operator=(PhysicsRun&&) = delete;

  // The physics model and its state, for a viewer.
  const mjModel& model() const;
  mjData* data();
  // The plan time of the current state.
  double PlanTime() const;
  // One control tick and one physics step, with the measurements due.
  // False once the run has ended or failed.
  bool Step();
  bool Ended() const;
  // Runs to the end and returns the figures. Also after some Steps.
  PhysicsResult Finish();

 private:
  PhysicsRun();
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// The whole run, as //:run_physics does it, and its record.
// layout_sha256 and plan_sha256 are those of the files, for the record.
PhysicsResult RunPhysics(const Layout& layout, const Plan& plan,
                         const ModelPaths& paths, const PhysicsOptions& options,
                         const std::string& layout_sha256,
                         const std::string& plan_sha256);

// The figures as text, the same for the same result.
std::string PhysicsText(const PhysicsResult& result);

}  // namespace cell::layout

#endif  // SRC_PHYSICS_RUN_H_
