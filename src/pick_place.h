// A scripted, blind pick and place: take one module off the front of the
// stack, carry it round to the conveyor, lay it on the rails of the empty
// slot, and move the tube on.
//
// Two ways of knowing where the module is (PickPlaceOptions::sensing):
//
//   Blind    No camera. The script assumes the pallet stands exactly where it
//            was designed to (PalletBodies::nominal_modules, the DESIGN
//            poses, not where the modules truly are), and that each pick
//            moves the front face back by one module thickness, which is
//            what those design poses say: module k's glass is k thicknesses
//            behind module_0's. The baseline: what blindness costs.
//   Camera   One depth picture per pick (Cell::TakeDepthImage), turned into
//            the front module's pose by src/module_finder.h with the camera
//            the control code believes in (Cell::CalibratedCamera). Every
//            tool pose of the pick side is then built from that estimate
//            instead of the design.
//
// What the script may read while running is what a real cell knows: the
// arm's joint angles (through the follower), the conveyor's encoder, the
// camera's picture and its calibration, and what it did itself (Grip's
// result). It never reads where a module or the pallet truly is.
// pick_place.cc does not call any True... accessor and reads no body pose
// out of the simulation; that is what makes the score of a run (src/score.h)
// a fair measure of what each way of sensing costs.
//
// One cycle, in the order the steps run (StepName gives each its name):
//
//   look          camera only. The picture of the stack, and the front
//                 module's pose from it. The arm must be out of the camera's
//                 view. Usually the picture was already taken at the end of
//                 the cycle before (look ahead, below), and this step only
//                 uses it. Otherwise (the first cycle, after a Reset) a
//                 picture is taken where the arm stands. If the finder
//                 refuses it, the arm goes to the look pose (arm_clear,
//                 reached round +Y, where nothing of it is in the view) and
//                 the picture is taken again, once: away from the look pose
//                 the arm may be in the picture, and then any step of the
//                 finder can be the one that refuses (at arm_home, whose
//                 tool hangs in front of the stack, it is the rectangle
//                 fit). A refusal from the look pose (no module there, the
//                 wrong size: an empty or wrong pallet) ends the cycle with
//                 the finder's reason, the arm holding still. Then the pick
//                 is planned from the estimate.
//   to standoff   joint moves from where the arm is (home, or ready after
//                 the cycle before) through `ready`, well back from the
//                 stack, to a pose square to module k's nominal glass,
//                 `standoff` in front of it
//   approach      straight in to `approach_slow` from the glass
//   touch glass   straight in, slowly, to `grip_gap` from the glass
//   grip          Cell::Grip, then wait for the vacuum (`grip_dwell`)
//   lift          straight up along the glass by `lift` (backing off
//                 `lift_back`), so the bottom edge leaves the deck instead
//                 of dragging on it
//   pull back     straight back along the glass normal by `pull_back`, so
//                 the back of the module leaves the next module's glass
//   clear         back and up, away from the stack and the deck
//   flatten       joint move: the module turned glass up and raised above
//                 the stack (`over_pallet`)
//   swing         joint move: joint 1 turned half a turn round the +Y side,
//                 the module still flat and long side along the tube, low
//                 over the rails beyond the slot (`turn_outboard`,
//                 `turn_height`)
//   turn          joint 6 alone turns the module a quarter turn, long side
//                 across the tube
//   wait conveyor until the tube has stopped; then the slot's pose is read
//                 from the conveyor's encoder and the set-down planned
//   lower         straight in toward the arm to above the slot, then
//                 straight down to `set_down_slow` above the release pose
//   set down      straight down, slowly, to `release_clearance` above the
//                 rails
//   release       Cell::Release, then wait (`release_dwell`)
//   retreat       straight up by `retreat`
//   conveyor      calls the placed hook (a scorer measures here), then
//                 starts the conveyor's step, or swaps the tube if this was
//                 its last slot
//   look ahead    camera only. The arm holds still at the end of the
//                 retreat, over the tube on the far side of the base from
//                 the pallet and below the camera's lines of sight to the
//                 stack, and the picture for the NEXT pick is taken there,
//                 while the tube moves: no extra arm motion, and only
//                 look_time added. The next cycle's look step uses it.
//   return        joint move back to `ready`, while the tube moves
//
// The straight moves are lists of joint waypoints solved by the IK
// (src/ik.h) every centimetre along the straight line. With blend_lines (the
// default) the follower runs the whole list in one motion
// (ArmFollower::MoveAlong), rounding the corners between waypoints within a
// set joint-space distance: deviation_near (tip within about 0.06 mm of the
// line) where the tool is at the glass or the module at the rails,
// deviation_far elsewhere. Without it, as at first, only the ends of a few
// legs are kept and the follower stops at each; a joint move strays from
// the straight line between its ends by about 2.2e-4 * L^2 mm for a leg L mm
// long, so the legs are short where straightness matters.
//
// Use, once per physics step, with a Cell that has just been Reset:
//
//   PickPlace script(cell);
//   script.Start(0);
//   while (!script.Done()) {
//     script.Update();   // writes the arm's ctrl, runs the steps
//     cell.Step();
//   }
//   script.outcome();    // placed, or failed and why
//
// Start the next module once Done. The script keeps one ArmFollower across
// cycles, whose plan is kept in simulation time; after another Cell::Reset
// it notices the clock went back and starts a fresh follower, but making a
// new PickPlace is clearer.
//
// Units: metres, radians and simulated seconds. Poses are in the world
// frame.

#ifndef SRC_PICK_PLACE_H_
#define SRC_PICK_PLACE_H_

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/arm_follower.h"
#include "src/cell.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/module_finder.h"

namespace cell {

// How the script knows where the module to pick is; see the top of the file.
enum class SensingMode { kBlind, kCamera };

// Every distance, speed and wait the script uses. The defaults are the ones
// tests/pick_place_test.cc measures the cycle with.
struct PickPlaceOptions {
  // ---------------------------------------------------------- sensing
  // Blind by default: the baseline, and what the stage-3 tests measure.
  SensingMode sensing = SensingMode::kBlind;
  // How long the arm holds still for a picture: the camera's exposure and
  // the readout and processing (the finder itself takes about 12 ms
  // optimized). The picture is taken at the end of this time.
  double look_time = 0.3;  // s
  // The finder's settings (src/module_finder.h); its defaults are the ones
  // it was measured with.
  FinderOptions finder;
  // The estimate is checked against the face the pick count expects (module
  // k's design glass): it may be off along the glass's normal by what the
  // pallet's placement tolerance allows (the shift, and the turn about the
  // pallet's vertical axis), plus this. Half a module thickness: more than
  // that and it may be another module. Only noted in the look step's
  // record, never acted on: the finder sees the front face, wherever it is.
  double face_check_margin = 0.5 * kModuleThickness;  // m

  // ------------------------------------------------------------ the pick
  // The ready pose, where a cycle starts its approach and where it returns
  // to: tool pointing straight down, its X along the pallet (as for the
  // grip), the tip at this point in the NOMINAL pallet's frame (origin on
  // the floor under module_0's glass centre, axes along the world's). The
  // default is 0.8 m toward the arm from module_0's glass and 1.3 m up.
  // At arm_home the cups are only 0.11 m above the stack's top edges and
  // the plate reaches out over them, so turning the tool from there
  // toward the glass would swing the cups into module_0's top edge. From
  // ready the wrist, which every point of the tool is within 0.75 m of,
  // stays about 0.9 m from the stack while the tool turns.
  Eigen::Vector3d ready{-0.80, 0.0, 1.30};  // m, nominal pallet frame
  // The joint move from ready ends this far in front of the glass, square
  // to it. From there on the tool only moves straight along the normal, so
  // nothing but the cups comes near the glass.
  double standoff = 0.35;  // m
  // The straight approach comes in at speed_empty to this distance from the
  // glass, then at speed_touching for the rest. A 28 mm leg strays by about
  // 0.2 mm from its line; the 320 mm leg before it by up to about 2 cm,
  // which is harmless that far out.
  double approach_slow = 0.03;  // m
  // Where the approach stops: the tool tip this far in front of the glass.
  // The suction grips within 10 mm (src/suction.h). The cups should not
  // push the free-standing module: when the tool is slightly tilted the end
  // cups touch first and push it. 2 mm leaves room for the arm's tracking
  // error (about 0.3 mm at the tip) and the stack's settling (0.2 mm).
  // The weld then holds the module this far from the tip, and the set-down
  // allows for it.
  double grip_gap = 0.002;  // m
  // After the grip the module is lifted straight up along its glass by this
  // much, so its bottom edge leaves the deck. Pulling it back without
  // lifting would drag that edge over the deck (about 174 N of friction)
  // and tip the module.
  double lift = 0.03;  // m
  // ...while backing off this much along the glass's normal. Once the
  // module's weight moves from the deck onto the weld, the arm sags a
  // millimetre or two, which the follower's model does not know of (the
  // module is not part of the arm's body tree); with the glass leaning
  // back 10 degrees, a vertical sag of d pushes the module sin(10 deg) * d
  // into whatever is behind it. A free-standing module behind gives way;
  // the pallet's back rest, behind module_29, does not: lifting module_29
  // straight along the glass pressed it on the back rest with up to 2.6 kN
  // and took joint 2 to 0.90 of its torque limit. Backing off 2 mm brought
  // that to 0.75, 4 mm to 0.71 (module_28, with a module behind it: 0.61;
  // the peak of module_29's cycle is then 0.74, in the clear move). The
  // bottom edge rises off the deck from the start, so backing off does not
  // drag it; module_0's lift-off disturbed the stack by 0.08 mm without
  // backing off and by 0.002 mm with it.
  double lift_back = 0.004;  // m
  // Then pulled straight back along the glass's normal by this much, in one
  // short leg, so the module's back leaves the next module's glass without
  // rubbing on it (a 50 mm leg strays by about 0.55 mm).
  double pull_back = 0.05;  // m
  // Then moved in one long leg to this far in front of the glass's plane
  // and this much further up the glass. The bottom edge rises away from the
  // deck (the deck is square to the glass) while the module backs off far
  // enough that turning it flat next does not swing its top edge into the
  // stack: the top edge swings toward the pallet by up to half the
  // module's short side, 0.55 m, as it turns.
  double clear_back = 0.45;  // m
  double clear_up = 0.15;    // m

  // ----------------------------------------------------------- the carry
  // The carry is two hand-chosen poses, joined by joint moves.
  //
  // over_pallet: the tool tip pointing straight down, the module flat under
  // it, glass up, its long side square to the arm (about world Y), the tip
  // at this point given in the pallet's frame (origin on the floor under
  // module_0's glass centre, axes along the world's): the NOMINAL pallet in
  // blind mode, the pallet as the estimate places it in camera mode, so
  // the module turned flat keeps its distance from the stack wherever the
  // pallet was set down. The default is 0.45 m toward the arm from
  // module_0's glass and 1.6 m up, so the module's underside is about
  // 0.27 m above the stack's highest edge (module_0's top edge, 1.29 m),
  // and the arm, reaching only 1.5 m out, holds its forearm well above the
  // module's inner edge.
  Eigen::Vector3d over_pallet{-0.45, 0.0, 1.60};  // m, pallet frame
  // The place side. The swing from over_pallet turns joint 1 half a turn
  // round the +Y side (the camera's post stands on -Y) and keeps the
  // module's long side along the arm's circle, as it was at the pallet, so
  // its inner edge stays well outside the arm. It ends with the module flat
  // over the slot's rails, but `turn_outboard` further from the arm than
  // the slot and `turn_height` above the release pose. There joint 6 alone
  // turns it a quarter turn, long side across the tube, and then it comes
  // in toward the arm, low, to above the slot.
  //
  // Why out there and low: the upper arm's collision hull (models/arm.xml)
  // is wide, reaching about 0.27 m out of the arm's plane toward the tool
  // and 0.4 m to one side, and at the place the arm reaches 2 m out with
  // the upper arm leaning toward the tool, so the hull's outer face is at
  // about 0.7 m from the base's axis at rail height and 0.9 m at 1.3 m.
  // Lying across the tube, the module's inner end is 0.85 m from that axis,
  // and during the quarter turn its edge crosses the arm's plane up to
  // 1.27 m in from the tip. Turning right over the slot hit the upper arm
  // at any height; turning 0.35 m further out keeps that edge outside the
  // hull, and coming in low keeps the inner end below the part of the hull
  // that leans out.
  double turn_outboard = 0.35;  // m, along the tube's cross direction
  double turn_height = 0.10;    // m, above the release pose
  // Without blend_lines: the way in, from the turn to above the slot at
  // turn_height, is cut into this many straight legs (each 0.35 m / legs
  // long; 0.175 m strays by about 7 mm), so the module, only centimetres
  // above the rails, does not sag onto them in the middle of one long leg.
  // With blend_lines the way in is one straight line, followed within
  // deviation_far.
  int in_legs = 2;

  // ------------------------------------------------------------- the place
  // The lowering comes straight down at speed_carrying to this height above
  // the release pose, then at speed_touching for the rest (a 40 mm leg
  // strays by about 0.35 mm).
  double set_down_slow = 0.04;  // m
  // Where the module is let go: its underside this far above the rails'
  // top surface. A module released tilted, or pressed onto the rails, rocks
  // and shifts as it settles; 1 mm lets it drop flat.
  double release_clearance = 0.001;  // m
  // After the release the tool goes straight up by `retreat`. Without
  // blend_lines in two legs, the first retreat_short long, so the cups
  // leave the glass along its normal.
  double retreat_short = 0.03;  // m
  double retreat = 0.25;        // m

  // ---------------------------------------------------------------- speeds
  // Fractions of full speed for the follower (MoveOptions::speed). Empty,
  // the follower's default, which keeps every motor under 0.84 of its
  // limit. Carrying, a little slower: the held module hangs on the weld, so
  // the follower's model of the arm does not include it, and joint 2 is the
  // motor nearest its limit when the module is held far out. Touching, for
  // the short legs where the tool or the module meets something.
  double speed_empty = MoveOptions::kDefaultSpeed;
  double speed_carrying = 0.9;
  double speed_touching = 0.5;
  // For the lift and the pull-back, while the module's back still rubs on
  // the next module's glass and its bottom edge is just off the deck.
  double speed_lift_off = 0.25;

  // ------------------------------------------------------------------ waits
  // Suction takes a moment to build; the arm waits this long after Grip
  // before lifting, and this long after Release before leaving.
  double grip_dwell = 0.2;     // s
  double release_dwell = 0.3;  // s
  // A move counts as finished when the follower's plan has ended and the
  // arm has arrived (ArmFollower::Done). If it has not arrived this long
  // after the plan ended, something holds it: the cycle fails, except where
  // the tool is meant to meet something. In the set-down the module meets
  // the rails: the arm stops where it is and lets go. In touch glass the
  // cups meet the glass: if the tip stopped within touch_tolerance of its
  // goal the grip is tried there, otherwise the cycle fails.
  double arrive_timeout = 1.0;  // s
  // A move is also held up as soon as any joint falls this far behind the
  // follower's plan. Following a plan the arm stays within about 0.1 mrad,
  // and a held module sags it by up to 0.5 mrad more (src/arm_follower.h);
  // a tool pressing on glass the cups met early bends the wrist by tens of
  // mrad within a fraction of a second. Seeing it from the joint angles
  // stops the push about a millimetre in, instead of pressing the stack
  // for the whole arrive_timeout.
  double blocked_tracking_error = 3e-3;  // rad
  // How far short of the touch pose the tool may stop, having met the
  // glass early, and still grip. The cups press the glass by about this
  // much at most, which is harmless; any more would push the free-standing
  // module into the stack.
  double touch_tolerance = 0.002;  // m
  // The longest the script waits for the conveyor to stop before laying a
  // module down. One step takes about 3 s.
  double conveyor_timeout = 15.0;  // s

  // -------------------------------------------------------------------- IK
  // Straight moves are solved every `line_spacing` along the line, each
  // pose seeded from the one before, so the solutions stay on one branch;
  // only the legs' ends become the follower's waypoints. No joint may turn
  // more than `max_joint_step` between neighbours (the pull-back needs
  // about 0.02 rad per centimetre).
  double line_spacing = 0.01;    // m
  double max_joint_step = 0.05;  // rad

  // Run straight moves as one blended motion through every IK waypoint
  // (ArmFollower::MoveAlong) instead of stopping at the ends of a few legs.
  bool blend_lines = true;
  // How far a blended move may stray from the polyline through its
  // waypoints, in radians of joint-space distance (MoveOptions::
  // max_deviation). Near: where the tool is on the glass or the module just
  // above the rails or just off the stack (touch glass, lift, pull back,
  // set down, retreat): on 0.3 m lines solved every centimetre the tip kept
  // within about 0.06 mm of the line (src/path_timing.h). Far: approach,
  // clear and lower, with centimetres to spare: about 0.5 mm at the tip,
  // and at most about 15 mm anywhere on the arm and module in the worst
  // case.
  double deviation_near = 2e-4;  // rad
  double deviation_far = PathTimingOptions::kDefaultMaxDeviation;  // rad
};

// The script's steps, in the order they run.
enum class PickPlaceStep {
  kIdle,            // not started
  kLook,            // camera only
  kToStandoff,
  kApproach,
  kTouchGlass,
  kGrip,
  kLift,
  kPullBack,
  kClear,
  kFlatten,
  kSwing,
  kTurn,
  kWaitForConveyor,
  kLower,
  kSetDown,
  kRelease,
  kRetreat,
  kConveyor,
  kLookAhead,       // camera only
  kReturn,
  kDone,            // placed, and back at ready
  kFailed,          // stopped; the arm holds where it is
};

// A short name for each step, for display: "touch glass", "swing", ...
const char* StepName(PickPlaceStep step);

// How a cycle ended, or that it has not yet.
struct PickPlaceOutcome {
  enum class Status { kNotStarted, kRunning, kPlaced, kFailed };
  Status status = Status::kNotStarted;
  int module = -1;  // module_<module> was picked
  int slot = -1;    // ...for this slot of the tube
  // For kFailed: what went wrong, in words with the numbers, and in which
  // step. Empty otherwise.
  std::string reason;
  PickPlaceStep failed_step = PickPlaceStep::kIdle;
  // Simulation time at Start, and when the cycle ended (kPlaced: back at
  // home; kFailed: when it failed).
  double start_time = 0.0;
  double end_time = 0.0;
};

// One move the script planned: the step it belongs to, its joint
// waypoints and the speed it is played at. The record of what the script
// commanded, for tests and for display.
struct PlannedMove {
  PickPlaceStep step = PickPlaceStep::kIdle;
  std::vector<Eigen::VectorXd> waypoints;
  double speed = 0.0;
};

// One picture and what the finder made of it.
struct LookRecord {
  double time = 0.0;            // simulation time it was taken
  PickPlaceStep step = PickPlaceStep::kIdle;  // look or look ahead
  Eigen::VectorXd arm;          // joint angles it was taken from
  // The estimate, or nothing if the finder refused; then diagnostics.failure
  // and diagnostics.detail say why.
  std::optional<ModuleEstimate> estimate;
  FinderDiagnostics diagnostics;
  double render_seconds = 0.0;  // computer time to take the picture
};

// What became of one step of a cycle: when it ran, in simulation seconds,
// and how it ended.
struct StepRecord {
  PickPlaceStep step = PickPlaceStep::kIdle;
  double start = 0.0;
  double end = 0.0;      // == start while the step runs
  bool finished = false;  // it ended, well or not
  // "done"; for some steps what was done ("held module_0", "step
  // started", "blocked, let go: ..."); "failed: <reason>" for the step the
  // cycle failed in.
  std::string result;
  double duration() const { return end - start; }
};

class PickPlace {
 public:
  // Called once per placement, after the module has been let go and the
  // tool has retreated, before the conveyor is told to move: the place for
  // a scorer to measure the placement. `module` and `slot` are the ones
  // the script meant. The hook belongs to the caller, which may read the
  // truth; the script itself does not.
  using PlacedHook = std::function<void(int module, int slot)>;

  // Keeps a reference to `cell`, which must outlive this object. The arm
  // should be at rest where its motors are commanded, as after Reset.
  explicit PickPlace(Cell& cell, const PickPlaceOptions& options = {});

  void set_placed_hook(PlacedHook hook) { placed_hook_ = std::move(hook); }

  // Starts a cycle for module_<module>: plans every pose of the pick and the
  // carry from the design geometry and starts the first move, at the
  // current simulation time. The slot is the one in front of the arm by
  // the script's own count: Conveyor::steps(). Returns false if a cycle is
  // still running (nothing changes), or if the module index is out of
  // range, the tube is full or the IK finds no solution for a pose; then
  // the cycle has failed at once and outcome() says why.
  //
  // `first` starts the cycle later on, for tests that build the state a
  // step expects instead of running every step before it (for example the
  // module already held over the slot, with Cell::PutArm and Grip). The
  // whole cycle is still planned; the arm must already be where the step
  // before `first` leaves it (Waypoints(that step).back()), and the module
  // as that step leaves it.
  //
  // In camera mode the cycle starts with the look step, and `module` is
  // only the module the pick count expects (for the estimate's sanity
  // check): the module actually picked is the one the grip reports, and
  // outcome().module says which. Started later than the look step, nothing
  // has been seen, and the pick side is planned from the design as in
  // blind mode. kIdle means the first step of the mode.
  bool Start(int module, PickPlaceStep first = PickPlaceStep::kIdle);

  // Stops the cycle when `step` has finished, before the next one begins,
  // until Resume. The arm holds its pose meanwhile, and Update must still be
  // called. For tests that look at the state a step leaves behind. Applies
  // to every later cycle too, until cleared with std::nullopt.
  void set_pause_after(std::optional<PickPlaceStep> step) {
    pause_after_ = step;
  }
  bool paused() const { return paused_; }
  // Goes on with the next step.
  void Resume();

  // Runs the script for the current simulation time and writes the arm's
  // motor commands. Call once before every Cell::Step, also when idle or
  // done, so the arm keeps holding its pose.
  void Update();

  // The cycle has ended: placed and back at home, or failed.
  bool Done() const {
    return step_ == PickPlaceStep::kDone || step_ == PickPlaceStep::kFailed;
  }

  PickPlaceStep step() const { return step_; }
  const char* step_name() const { return StepName(step_); }
  const PickPlaceOutcome& outcome() const { return outcome_; }

  // Every move planned for the current cycle so far, in order. The set-down
  // is planned only once the conveyor has stopped, so its moves appear
  // then.
  const std::vector<PlannedMove>& planned() const { return planned_; }

  // Each step of the current cycle so far, in order: when, how long, and
  // how it ended.
  const std::vector<StepRecord>& records() const { return records_; }

  // The joint waypoints planned for a step of the current cycle, or null
  // for a step that does not move the arm. Planned at Start, except lower,
  // set down and retreat, which are planned when lower begins (empty until
  // then).
  const std::vector<Eigen::VectorXd>* Waypoints(PickPlaceStep step) const;

  // The tool-tip poses the script aims for, for display and tests: where it
  // grips, and where it lets go (set once the set-down is planned).
  const Eigen::Isometry3d& grip_tip() const { return grip_tip_; }
  const std::optional<Eigen::Isometry3d>& release_tip() const {
    return release_tip_;
  }

  // Which way round the module goes on the tube: +1 if its +x ends up
  // along world +X (as Conveyor::PlacementPose has it), -1 if along -X.
  // Both are the same rectangle on the rails; the script takes the one its
  // quarter turn reaches with joint 6 nearer zero.
  double place_x_sign() const { return place_x_sign_; }

  const PickPlaceOptions& options() const { return options_; }
  const ArmFollower& follower() const { return follower_; }

  // Camera mode: the pictures of the current cycle, in order (the look
  // step's one or two, and the look ahead's), and the estimate the pick
  // side was planned from (nothing in blind mode, or before the look step
  // has succeeded).
  const std::vector<LookRecord>& looks() const { return looks_; }
  const std::optional<ModuleEstimate>& estimate() const { return estimate_; }
  // The joint angles of the look pose: arm_clear, with joint 1 at -pi so
  // that it is reached round +Y like everything else.
  const Eigen::VectorXd& look_pose() const { return look_pose_; }

 private:
  // The face the pick side aims at, from the design or from an estimate.
  struct Face {
    Eigen::Vector3d glass;      // centre of the glass
    Eigen::Vector3d normal;     // out of the glass, unit
    Eigen::Vector3d long_side;  // the module's +x, unit
    Eigen::Vector3d up;         // the module's +y, up the lean, unit
    // Where the pallet is taken to be: its nominal pose moved as the
    // estimate says the module moved (identity when from the design).
    Eigen::Isometry3d pallet_shift = Eigen::Isometry3d::Identity();
  };
  Face DesignFace(int module) const;
  Face EstimatedFace(const ModuleEstimate& estimate, int module) const;

  // Plans the pick and the carry for `face`. On failure returns false with
  // the reason and the step it was for.
  bool PlanPick(const Face& face, std::string* reason, PickPlaceStep* step);
  // Plans the set-down from the conveyor's encoder.
  bool PlanPlace(std::string* reason);

  // Solves the straight line from `from` to `to` (tip poses), the arm
  // starting at `start` (its angles at `from`), and returns the angles at
  // each of `stops`, distances from `from` along the line in increasing
  // order, the last being the whole length. With blend_lines, every
  // waypoint solved along the line instead (all but `from`'s), for
  // MoveAlong.
  std::optional<std::vector<Eigen::VectorXd>> SolveLine(
      const Eigen::VectorXd& start, const Eigen::Isometry3d& from,
      const Eigen::Isometry3d& to, const std::vector<double>& stops,
      std::string* reason);

  // Camera mode: takes a picture now and runs the finder on it.
  LookRecord TakeLook(PickPlaceStep step);
  // The look step's verdict on a picture: plans the pick from its estimate
  // and ends the step, or moves to the look pose to take it again, or
  // fails the cycle.
  void UseLook(const LookRecord& look);
  // The step after `step` in this mode (blind skips the look steps).
  PickPlaceStep Next(PickPlaceStep step) const;
  // Solves one pose from `seed`; on failure says why in `reason`.
  std::optional<Eigen::VectorXd> SolvePose(const Eigen::Isometry3d& target,
                                           const Eigen::VectorXd& seed,
                                           std::string* reason);

  // Makes `step` the current step and opens its record.
  void Enter(PickPlaceStep step);
  // Closes the current step's record with `result`.
  void Close(const std::string& result);
  // Closes the current step with `result` and begins the next, or pauses
  // before it.
  void Finish(const std::string& result);
  // Begins `step`: starts its move, or does what it does.
  void BeginStep(PickPlaceStep step);
  // Enters `step` and starts its move through `waypoints` at `speed`:
  // stopping at each, or, if `deviation` is set and blend_lines is on, in
  // one blended motion within that joint-space distance.
  void BeginMove(PickPlaceStep step, const std::vector<Eigen::VectorXd>& waypoints,
                 double speed, std::optional<double> deviation = std::nullopt);
  // Whether the current move has ended. If the arm has not arrived
  // arrive_timeout after the plan ended, it is blocked: that fails the
  // cycle (and returns kRunning), unless `may_be_blocked`.
  enum class Progress { kRunning, kFinished, kBlocked };
  Progress MoveFinished(bool may_be_blocked);
  // How far the tool tip is from the current move's goal, in words, and in
  // metres in `distance` if it is not null.
  std::string Shortfall(double* distance = nullptr) const;
  void Fail(const std::string& reason);

  // The state machine: one call per Update.
  void Advance();

  Cell& cell_;
  PickPlaceOptions options_;
  ArmFollower follower_;
  // For the IK, never the simulated state. Mutable: TipPoseAt writes into
  // it to answer a question about joint angles.
  mutable DataPtr scratch_;
  PlacedHook placed_hook_;
  double last_time_ = 0.0;

  PickPlaceStep step_ = PickPlaceStep::kIdle;
  double step_start_ = 0.0;
  double plan_end_ = 0.0;  // when the current move's plan ends
  PickPlaceOutcome outcome_;
  std::vector<PlannedMove> planned_;
  std::vector<StepRecord> records_;
  std::optional<PickPlaceStep> pause_after_;
  bool paused_ = false;
  PickPlaceStep pending_ = PickPlaceStep::kIdle;  // to begin on Resume

  // The current cycle's plan, joint angles. Each list is one step's
  // waypoints.
  // Camera mode.
  PalletPrior prior_;
  Eigen::VectorXd look_pose_;
  std::vector<Eigen::VectorXd> look_;     // the move to the look pose
  std::vector<LookRecord> looks_;         // this cycle's pictures
  std::optional<LookRecord> look_ahead_;  // taken last cycle, for this one
  std::optional<ModuleEstimate> estimate_;
  // Inside the look step: moving to the look pose; then holding still for
  // the picture since look_hold_start_.
  bool look_moving_ = false;
  bool look_moved_ = false;  // the retake from the look pose has begun
  double look_hold_start_ = 0.0;
  std::string look_note_;    // what the look step found, for its record
  std::string grip_note_;    // a grip of another module than expected

  std::vector<Eigen::VectorXd> to_standoff_, approach_, touch_, lift_,
      pull_back_, clear_, flatten_, swing_, turn_, lower_, set_down_,
      retreat_, return_;
  // The tool's X at the place, along world X: +1 or -1, whichever the
  // quarter turn reaches with joint 6 nearer zero. The module is the same
  // rectangle either way round.
  double place_x_sign_ = 1.0;
  Eigen::Isometry3d grip_tip_ = Eigen::Isometry3d::Identity();
  std::optional<Eigen::Isometry3d> release_tip_;
};

}  // namespace cell

#endif  // SRC_PICK_PLACE_H_
