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
//   carry         planned carry only (PickPlaceOptions::carry), in place of
//                 flatten, swing and turn: one collision-free move from the
//                 end of clear to where the turn ends, planned by
//                 src/motion_planner.h (PlanArmMove) and run with
//                 ArmFollower::Follow. See "The planned carry" below.
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
// The planned carry (CarryMode::kPlanned). The scripted carry is three
// hand-chosen joint moves; the planned one is a single path found by the
// motion planner between the same two ends:
//   start  the end of clear. Not the end of pull back: there the module is
//          still within the planner's 45 mm planning margin of the stack.
//   goal   the end of the scripted turn (module flat, long side across the
//          tube, turn_outboard beyond the slot, turn_height above the
//          release pose, joint 1 at -180 degrees, round +Y), so wait
//          conveyor, lower and set down follow unchanged.
// The planner's scene is built only from what the cell knows (CarryScene):
// the pallet and the modules still on it where the stack's estimate puts
// them (the design in blind mode), the modules the cell already took off
// where it put them (their slots on the current tube, from the conveyor's
// encoder, or ParkingPose once their tube was swapped away), the tube's
// slide from the encoder, and the held module at the design grip
// (DesignModuleInTip with grip_gap). Everything else in the planner's
// model (the conveyor's bed, the camera's post and stands, the floor) is
// fixed by design and stays as the model was built.
//
// It is planned as soon as the grip has held, at the start of lift: the
// start is known then, and on a real cell the planning (about 0.3 s of
// computer time, optimized) runs while the arm lifts, pulls back and clears.
// Here the simulation waits for it; the wall time it took, the planner's
// report and whether it fits in the simulated time of those three moves
// are kept in PickPlaceOutcome::carry and in the carry step's record. If
// no path is found the cycle fails in the carry step with the planner's
// reason, the module held at the end of clear; there is no fallback to the
// scripted carry. The planner's base seed is fixed (PickPlaceOptions::
// planner), so a cycle is reproducible from the cell's seed.
//
// The carry is a list of legs (CarryLeg), each a timed move and a dwell at
// its end. Without the check after pickup there is one leg and no dwell;
// with it, two (below).
//
// The check after pickup (PickPlaceOptions::check_after_pickup; planned
// carry only, Start refuses it with the scripted one). The grip holds the
// module wherever it was on the tool at the grip, so an error in the pallet
// camera's estimate (its calibration above all) would go straight into the
// placement. With the check the carry runs in two legs, both planned at the
// start of lift with the design grip:
//   leg 1  the end of clear to the check pose (check_pose: the module flat,
//          glass up, over the check camera on the floor), then a dwell of
//          check_dwell; at its end the check camera's picture is taken
//          (Cell::TakeCheckImage) and src/held_module_finder.h's
//          FindHeldModule run on it with the calibrated check camera and
//          the expected pose TipPoseAt(the joint angles the arm REPORTS
//          then) times DesignModuleInTip(thickness, grip_gap).
//   leg 2  the check pose to the end of the turn, as the single leg was.
// From the module's measured pose and the tip's, GripOffsetFromCheck gives
// where the module sits on the tool. From then on the script uses that
// grip: leg 2 is checked again with the measured module on the tool
// (DenseSamples along its path, at the margin that keeps the planner's
// required_margin for samples that dense, see RecheckLeg in the .cc) and
// replanned with it only if that check fails; the planner's scene
// (CarryPlanScene) holds the measured grip; and the place is planned with
// CorrectedTipTarget, so that lower and set down put the MODULE, not the
// tool, on the slot. All of it is kept in PickPlaceOutcome::check.
//
// A refusal by the finder. Nothing is ever placed uncorrected: a check that
// is refused fails the cycle in the carry, the arm holding still at the
// check pose, and the reason says whether the pickup failed or the camera
// did, from the suction tool's vacuum switch (Cell::VacuumHolds, read at
// every picture). kFarFromExpected means the module has slipped on the
// cups (src/held_module_finder.h, step 9: "the cycle should stop"): it fails
// at once (CheckReport::Status::kSlipped). Any other refusal earns one more
// picture after another check_dwell. If that one is accepted, the cycle
// carries on corrected and counts as retaken (CheckReport::retaken), so a
// flaky camera shows as a number. If it is refused too, the cycle fails
// (kRefused): with the vacuum lost, "no module on the tool: the vacuum is
// lost (dropped or never picked)"; with it holding and kTooFewPoints, "check
// camera fault: the vacuum holds a module but the camera sees too few
// points (n of m expected ...)"; with it holding and any other refusal, the
// finder's reason and that the vacuum holds. A picture taken with the
// vacuum lost fails the cycle whatever the finder made of it, except that a
// refused first picture is still retaken: a module that is not on the tool
// cannot be placed, and a slip seen while it falls is no slip.
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
// cycles, whose plan is kept in simulation time, and its records of where
// each module is (known_modules). One rule for a Cell::Reset: the script
// notices it when the clock is behind the last time it saw, at the next
// Start, Update or set_known_module, whichever comes first, and then starts
// a fresh follower, drops the look-ahead picture and wipes every record
// (every module on the stack again), before doing what was asked. So
// records set with set_known_module after a Reset survive into Start, and
// records from before a Reset never do. A Reset the script cannot see (it
// has seen no time later than the settled clock since the Reset before)
// is not noticed: make a new PickPlace after a Reset to be sure.
//
// Units: metres, radians and simulated seconds. Poses are in the world
// frame.

#ifndef SRC_PICK_PLACE_H_
#define SRC_PICK_PLACE_H_

#include <cstddef>
#include <functional>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/arm_follower.h"
#include "src/cell.h"
#include "src/collision.h"
#include "src/held_module_finder.h"
#include "src/ik.h"
#include "src/mj_handles.h"
#include "src/module_finder.h"
#include "src/motion_planner.h"
#include "src/pallet.h"

namespace cell {

// How the script knows where the module to pick is; see the top of the file.
enum class SensingMode { kBlind, kCamera };

// How the module goes from the end of clear to over the slot: the three
// scripted joint moves (flatten, swing, turn), or one planned move (carry).
// See the top of the file.
enum class CarryMode { kScripted, kPlanned };
const char* CarryModeName(CarryMode mode);  // "scripted", "planned"

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

  // Scripted by default: the stage-3 to stage-5 carry, which the existing
  // tests measure. Planned: see the top of the file.
  CarryMode carry = CarryMode::kScripted;
  // The planned carry's planner settings (src/motion_planner.h), its
  // defaults those the planner was measured with. Its base_seed is used as
  // it is for every cycle, so the same cell seed gives the same carry. Its
  // speed is not used: the carry runs at speed_carrying, as the scripted one
  // does.
  MotionPlanOptions planner;

  // ------------------------------------------------ the check after pickup
  // Off by default. On, the planned carry goes through check_pose and the
  // grip is measured there (see the top of the file); Start refuses it with
  // the scripted carry.
  bool check_after_pickup = false;
  // The check pose, joints 1 to 6: the module flat, glass up, its centre
  // straight above the check camera (src/check_placement.h's study; the
  // camera stands at (0.1307, 1.4943) on the floor, tests/
  // check_camera_cell_test.cc). Joint 1 at -85 degrees: on the way round +Y.
  Eigen::VectorXd check_pose = [] {
    Eigen::VectorXd q(6);
    q << -85.00, -90.81, 81.68, 0.0, 99.13, 90.0;
    return Eigen::VectorXd(q * (std::numbers::pi / 180.0));
  }();
  // How long the arm holds still at the check pose before the picture (the
  // arm settling and one exposure); a retake after a refusal waits as long
  // again.
  double check_dwell = 0.2;  // s
  // The held-module finder's settings; its defaults are the ones it was
  // measured with.
  HeldFinderOptions held_finder;
  // Leg 2 is checked again with the measured grip at configurations no
  // further apart than this (m of motion, DenseSamples), and so at the
  // margin required_margin + recheck_spacing / 2 + corner_allowance +
  // tracking_allowance (35 mm with the defaults; self pairs the same over
  // required_self_margin): the planner's own budget (src/motion_planner.h)
  // with the edge term for samples this dense, so a path that passes keeps
  // the planner's required_margin as surely as a planned one.
  double recheck_spacing = 0.01;  // m

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
  kCarry,           // planned carry only, in place of flatten .. turn; with
                    // the check after pickup, its picture too
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

// The carry of one cycle: how long it took and, when planned, how the
// planning went.
struct CarryReport {
  CarryMode mode = CarryMode::kScripted;
  // Simulated seconds from the end of clear to the end of the turn (or of
  // the planned carry), dwells included. Zero until the carry has ended.
  double seconds = 0.0;

  // ------------------------------------------------------- planned only
  bool attempted = false;  // the planner was run this cycle
  bool found = false;      // ...and every leg was found and timed
  // Wall-clock seconds the planning took (scene, search, shortening,
  // timing, all legs), and the planner's report of each leg in order (the
  // failing one last). Empty when not attempted.
  double plan_seconds = 0.0;
  std::vector<MotionPlanReport> legs;
  // The simulated time the planning ran alongside: the lift, pull back and
  // clear moves, if it was planned at the start of lift (zero if it was
  // planned when the carry began, for a cycle started later than lift).
  double overlap_seconds = 0.0;
  bool fits() const { return attempted && plan_seconds <= overlap_seconds; }
  // Of the found path: the smallest clearance to an obstacle any leg's
  // report measured (m), and between what; the trajectories' simulated
  // duration in all (dwells not included).
  double clearance = 0.0;
  std::string clearance_between;
  double trajectory_seconds = 0.0;
  // The tube was still moving when the carry was planned: the scene put it
  // where the conveyor was headed (its nominal stop) instead of where the
  // encoder read.
  bool tube_moving = false;
};

// One picture of the check after pickup and what the finder made of it.
struct CheckRecord {
  double time = 0.0;      // simulation time it was taken
  int attempt = 1;        // 1, or 2 for the retake after a refusal
  Eigen::VectorXd arm;    // the joint angles the arm reported then
  bool vacuum_holds = false;  // the vacuum switch then (Cell::VacuumHolds)
  // The tool tip's pose from those angles (TipPoseAt), and the module's
  // expected pose: tip times DesignModuleInTip(thickness, grip_gap).
  Eigen::Isometry3d tip = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  // The finder's answer, or nothing (diagnostics.failure says why).
  std::optional<HeldModuleEstimate> estimate;
  HeldFinderDiagnostics diagnostics;
  // With an answer: the module's pose in the tool tip's frame
  // (GripOffsetFromCheck).
  std::optional<Eigen::Isometry3d> module_in_tip;
  double render_seconds = 0.0;  // computer time to take the picture
};

// The check after pickup of one cycle.
struct CheckReport {
  enum class Status {
    kOff,        // not asked for
    kNotRun,     // asked for, but the carry has not reached it (or the
                 // cycle started after the carry, or failed before it)
    kChecked,    // measured; the place is corrected
    kRefused,    // refused twice, or the vacuum lost: the cycle failed
    kSlipped,    // the finder says the module slipped: the cycle failed
  };
  Status status = Status::kOff;
  int pictures = 0;
  // kChecked on the second picture, the first refused.
  bool retaken = false;
  // The suction tool's vacuum switch (Cell::VacuumHolds) at the last
  // picture.
  bool vacuum_holds = false;
  // kRefused, kSlipped: the last refusal; kChecked with retaken: the first
  // picture's.
  HeldFinderFailure failure = HeldFinderFailure::kNone;
  std::string failure_detail;
  // Computer time: the finder (all pictures), and rendering them.
  double finder_seconds = 0.0;
  double render_seconds = 0.0;

  // ------------------------------------------------------ kChecked only
  // The measured grip (module in tip), and how far it is from the design
  // grip, in the design module's frame: the shift along its x, y, z (m)
  // and the rotation vector (rad) of design^-1 * measured.
  Eigen::Isometry3d module_in_tip = Eigen::Isometry3d::Identity();
  Eigen::Vector3d offset_shift = Eigen::Vector3d::Zero();
  Eigen::Vector3d offset_rotation = Eigen::Vector3d::Zero();
  // Leg 2 checked again with the measured grip: at what margins, how many
  // configurations, whether all were clear, and the wall time it took.
  bool rechecked = false;
  bool recheck_clear = false;
  double recheck_margin = 0.0;       // m
  double recheck_self_margin = 0.0;  // m
  int recheck_samples = 0;
  double recheck_seconds = 0.0;
  std::string recheck_hit;  // the first configuration found too close
  // Leg 2 replanned with the measured grip (only when the recheck failed),
  // and the wall time and the planner's report of that.
  bool replanned = false;
  double replan_seconds = 0.0;
  MotionPlanReport replan;
};
const char* CheckStatusName(CheckReport::Status status);  // "checked", ...

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
  CarryReport carry;
  CheckReport check;
  // The set down was blocked (the module met the rails early, "blocked, let
  // go") and the tool let go where it stopped: the cycle counts as placed,
  // but the script cannot say the module lies in its slot. It is recorded as
  // ModulePlace::kLetGo.
  bool placement_unconfirmed = false;
};

// Where the cell takes a module to be, from what it did itself.
enum class ModulePlace {
  kOnStack,  // not taken by this script (see CarryScene for which count)
  kHeld,     // gripped by this script
  kOnTube,   // let go by this script in `slot` of the current tube
  kParked,   // was on a tube that this script swapped away
  // Let go where a blocked set-down stopped, over `slot` of the current
  // tube: at `pose` (the module's centre and axes in the world, from the
  // joint angles at the release and the grip the script used), the tube's
  // slide reading `slide` then. It met the rails, so it is taken to rest on
  // the tube and move with it; but not to lie squarely in its slot.
  kLetGo,
  // Was kLetGo on a tube this script swapped away: it went with the tube if
  // it lay on it, which the script cannot confirm.
  kUnaccounted,
};
struct KnownModule {
  ModulePlace place = ModulePlace::kOnStack;
  int slot = -1;  // kOnTube and kLetGo
  // kLetGo only.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  double slide = 0.0;
};

// What the planner's scene for a carry is built from: all of it is what a
// real cell knows.
struct CarrySceneInput {
  int held = -1;  // the module on the tool; -1 for none
  // The design grip's gap: the module hangs at DesignModuleInTip(
  // kModuleThickness, grip_gap) in the tool tip's frame...
  double grip_gap = 0.0;
  // ...unless the grip has been measured (the check after pickup): then
  // the module hangs here in the tool tip's frame.
  std::optional<Eigen::Isometry3d> module_in_tip;
  // Where the stack is taken to be: the motion that carries the design's
  // pallet and stack onto the estimate (identity in blind mode).
  Eigen::Isometry3d pallet_shift = Eigen::Isometry3d::Identity();
  // Every module of the pallet by index: where the cell put it.
  std::vector<KnownModule> modules;
  // The conveyor's slide (Conveyor::SensorPosition, or its nominal stop).
  double tube_slide = 0.0;
};

// The planner's scene for a carry, from `input` and the design (`design`'s
// nominal poses, Conveyor::SlotPose, ParkingPose):
//   pallet       pallet_shift times the design pallet;
//   modules      every module but the held one: on the stack, pallet_shift
//                times its design pose, if its index is above the held
//                one's (one in front of the module being carried, with no
//                record of where it went, is taken to have been carried
//                away, at ParkingPose, as Cell::ParkModule puts it); on
//                the tube, its slot's pose at tube_slide; parked, at
//                ParkingPose; let go (kLetGo), at its pose carried along
//                by the tube from the slide it was let go at to
//                tube_slide; unaccounted, at ParkingPose, where the cell
//                puts a swapped tube's modules (the scene must put it
//                somewhere, and nothing better is known);
//   tube_slide   as given;
//   held_module, module_in_tip  the held module at the design grip, or at
//                the measured one if given.
// world stays null, so the planner's checker keeps the model's own state
// for everything not named here: what is fixed by design. No contact is
// allowed.
PlanScene CarryScene(const PalletBodies& design, const CarrySceneInput& input);

// One leg of a planned carry: the timed move, and how long the arm holds
// still at its end before the next leg (or before the carry ends).
struct CarryLeg {
  TimedMove move;
  double dwell = 0.0;  // s
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
  // Called right after each check picture has been taken and the finder
  // run, before anything moves: the place for a scorer to compare the
  // measured grip with the truth. Also the caller's, as PlacedHook.
  using CheckHook = std::function<void(const CheckRecord& record)>;

  // Keeps a reference to `cell`, which must outlive this object. The arm
  // should be at rest where its motors are commanded, as after Reset.
  explicit PickPlace(Cell& cell, const PickPlaceOptions& options = {});

  void set_placed_hook(PlacedHook hook) { placed_hook_ = std::move(hook); }
  void set_check_hook(CheckHook hook) { check_hook_ = std::move(hook); }

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
  //
  // With the planned carry, flatten, swing and turn are not steps of the
  // cycle and cannot be `first`; with the scripted one, carry cannot.
  // Started at carry, the carry is planned when it begins, from the end of
  // clear, for `module` as held. With check_after_pickup and the scripted
  // carry the cycle fails at once ("the check after pickup needs the
  // planned carry"); started after the carry, nothing is checked and the
  // place is planned with the design grip (CheckReport::Status::kNotRun).
  bool Start(int module, PickPlaceStep first = PickPlaceStep::kIdle);

  // Stops the cycle when `step` has finished, before the next one begins,
  // until Resume. The arm holds its pose meanwhile, and Update must still be
  // called. For tests that look at the state a step leaves behind. Applies
  // to every later cycle too, until cleared with std::nullopt. Returns false
  // and changes nothing for a step that never finishes in this script's
  // modes (idle, done, failed; the look steps in blind mode; flatten, swing
  // and turn with the planned carry; carry with the scripted one), which
  // would otherwise never pause.
  bool set_pause_after(std::optional<PickPlaceStep> step);
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
  // then), and carry, the planned carry's path (every leg's, joined; its
  // first point the end of clear), planned when lift begins. Flatten, swing
  // and turn are planned in either carry mode: the turn's end is the
  // planned carry's goal.
  const std::vector<Eigen::VectorXd>* Waypoints(PickPlaceStep step) const;

  // The tool-tip poses the script aims for, for display and tests: where it
  // grips, and where it lets go (set once the set-down is planned; with a
  // measured grip, the corrected tip: CorrectedTipTarget of the module's
  // release pose, release_module()).
  const Eigen::Isometry3d& grip_tip() const { return grip_tip_; }
  const std::optional<Eigen::Isometry3d>& release_tip() const {
    return release_tip_;
  }
  // Where the script means the MODULE to be when it is let go: flat, its
  // underside release_clearance above the slot's rails, centred on the slot
  // (set with release_tip).
  const std::optional<Eigen::Isometry3d>& release_module() const {
    return release_module_;
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

  // ----------------------------------------------------- the planned carry
  // Where the script takes each module to be: what it gripped, let go on
  // the tube and swapped away since it was made (or since the cell was
  // last Reset). Every module starts on the stack.
  const std::vector<KnownModule>& known_modules() const { return known_; }
  // Tells the script where module i is, for a scene it did not make itself
  // (cycles run by an earlier script, a test's constructed state). An index
  // out of range is ignored. Notices a Reset first (see the top of the
  // file), so a record set after a Reset is kept.
  void set_known_module(int i, const KnownModule& known);
  // The planner's scene for carrying module `held` now (CarryScene), from
  // what the script knows: the stack where the current estimate puts it
  // (camera mode, once the look step has an estimate) or as designed, the
  // modules it put on the tube or swapped away, the conveyor's encoder
  // (its nominal stop while it still moves), and the grip as measured by
  // this cycle's check, once it has been, or the design grip.
  PlanScene CarryPlanScene(int held) const;
  // The current cycle's planned carry, leg by leg; empty until planned.
  const std::vector<CarryLeg>& carry_legs() const { return carry_legs_; }

  // ---------------------------------------------- the check after pickup
  // This cycle's check pictures, in order, and the grip it measured
  // (nothing until measured, or if it was not).
  const std::vector<CheckRecord>& checks() const { return checks_; }
  const std::optional<Eigen::Isometry3d>& measured_grip() const {
    return held_in_tip_;
  }

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
  // Plans the carry from the end of clear to the end of the turn for the
  // module held (outcome_.module), fills outcome_.carry and carry_legs_.
  // On failure returns false with the planner's reason.
  bool PlanCarry(std::string* reason);
  // Begins the carry step: plans it if that has not been done, then starts
  // its first leg, or fails the cycle.
  void BeginCarry();
  // Starts following leg carry_leg_.
  void BeginCarryLeg();
  // The carry step's progress: legs, dwells, the check. Returns true once
  // it has ended.
  bool CarryFinished();
  // The check after pickup at the end of the check leg's dwell: the
  // picture, the finder, and what follows (see the top of the file).
  enum class CheckNext { kGoOn, kRetake, kFailed };
  CheckNext RunCheck();
  // Checks leg `leg` of the carry again with the grip measured (see
  // PickPlaceOptions::recheck_spacing), and replans it if that fails; on a
  // failed replan returns false with the reason.
  bool RecheckLeg(std::size_t leg, std::string* reason);
  // Joins the legs' paths into carry_ and sums their figures into
  // outcome_.carry.
  void JoinCarryLegs();
  // The grip the script holds the module with: measured, or the design's.
  Eigen::Isometry3d GripInTip() const;
  // A Reset of the cell, seen by the clock going back (see the top of the
  // file): a fresh follower, no look-ahead picture, every record wiped.
  // Returns true if one was seen.
  bool NoticeReset();
  // Whether `step` is not a step of a cycle in this script's modes.
  bool Skipped(PickPlaceStep step) const;
  // The motion carrying the design's stack onto the estimate, for `held`.
  Eigen::Isometry3d PalletShift(int held) const;

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
  CheckHook check_hook_;
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
  std::optional<Eigen::Isometry3d> release_module_;
  // The set down of this cycle stopped short against the rails.
  bool set_down_blocked_ = false;

  // The planned carry. The checker is made on the first plan.
  std::optional<CollisionChecker> checker_;
  std::vector<KnownModule> known_;
  std::vector<CarryLeg> carry_legs_;
  std::vector<Eigen::VectorXd> carry_;  // every leg's path, joined
  std::string carry_failure_;           // why the plan failed
  std::size_t carry_leg_ = 0;           // the leg running
  bool carry_dwelling_ = false;         // holding at the end of carry_leg_
  double carry_dwell_end_ = 0.0;
  std::string carry_note_;              // for the carry step's record

  // The check after pickup: the leg whose dwell ends in the picture (none
  // without the check), this cycle's pictures, the grip measured (from
  // then on the script's grip), and the check's note for the carry record.
  std::optional<std::size_t> check_leg_;
  std::vector<CheckRecord> checks_;
  std::optional<Eigen::Isometry3d> held_in_tip_;
  std::string check_note_;
};

}  // namespace cell

#endif  // SRC_PICK_PLACE_H_
