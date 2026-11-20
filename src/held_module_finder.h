// The check after pickup: where the module really sits on the suction tool,
// from one depth image taken from below.
//
// Why it is needed. The grip (src/suction.h) holds the module at whatever
// pose it had relative to the tool at the moment of the grip. That pose is
// off from the design's by the pallet camera's error (most of all its
// calibration: about 4.4 mm at the module per milliradian), the arm's
// approach error and any push from the cups touching first. The place is
// planned for the design's grip, so all of that offset lands in the
// placement. Measuring the module once it is on the tool takes it out.
//
// How. On the way to the conveyor the arm holds the module flat, glass up,
// the tool on top, over a second depth camera standing on the floor and
// looking straight up, and pauses. The camera sees the module's underside
// (its back, the module's -z face) whole: nothing of the arm or the tool is
// below it, and beyond its edges the camera sees things far higher up, or
// nothing. So all four edges are steps in depth. The pose comes out of the
// picture in these steps, the same ones as for the pallet
// (src/module_finder.h), reusing its pieces:
//
//   1. In view    The expected underside, grown by the tolerances, must lie
//                 inside the image and face the camera; otherwise the arm is
//                 not where it should be, and nothing is looked at.
//
//   2. Points     Every pixel with a reading becomes a point in the world,
//                 through the camera's pose (ImageToArmFrame). An error in
//                 that pose (calibration) moves the answer rigidly by the
//                 same motion; nothing in one picture can see it.
//
//   3. Crop       Only points where the underside can be: a slab round the
//                 expected underside, as long and wide as the module plus
//                 what the position and angle tolerances can move a corner
//                 by, and as thick as they can tip it. Fewer points in it
//                 than half the pixels the underside should cover is no
//                 module: the tool alone shows its plate and cups, about a
//                 third of that.
//
//   4. Plane      FindGlassPlane on the crop, constrained to planes facing
//                 within a few degrees of the expected underside's normal
//                 (toward the camera). It reads only that direction from the
//                 prior it is given, so the prior here is "the underside as
//                 if it were a glass": the expected module turned half a turn
//                 about its long side. Then fitted again to the inside only,
//                 3 cm in from the outline, away from the edge pixels.
//
//   5. Flatten    The plane's points in 2D, with x along the expected long
//                 side (+x of the expected module) and y = normal x x. That
//                 is the module's -y, because the normal (toward the camera)
//                 is the module's -z: seen from below the module is mirrored.
//
//   6. Outline    FitRectangle (src/rectangle_fit.h) for the 2.3 x 1.1 m
//                 rectangle. Every side is a depth step, so its dot counting
//                 corrects the pixel grid on all four, and the dropped and
//                 flying pixels there, which are alike on opposite sides,
//                 shorten the module without moving its centre. Size check.
//
//   7. Clear view Every pixel inside the outline must read the underside
//                 (CheckView): nothing in the way, no holes.
//
//   8. Pose       ModulePoseFromOutline gives the frame of a module whose
//                 glass is the face seen: origin half a thickness behind it,
//                 +z toward the camera. The real module is that frame turned
//                 half a turn about its long side: same x, y and z reversed,
//                 and the same origin, which is half a thickness ABOVE the
//                 underside, toward the glass. The convention is the usual
//                 one: origin at the centre, +x long side, +y short side, +z
//                 out of the glass.
//
//   9. Sanity     The answer must be within the tolerances of the expected
//                 pose. A module much further off has slipped on the cups,
//                 and the cycle should stop.
//
// Then the frame algebra (GripOffsetFromCheck, CorrectedTipTarget at the
// bottom): the measured pose and the tool tip's pose, which the arm's joint
// angles give, make the module's pose in the tool tip's frame; and from that,
// where the tool tip has to go for the MODULE to land where it should.
//
// Nothing here reads the truth: only the image, the camera model it is
// handed (perhaps miscalibrated), the expected pose and the design sizes.
//
// Units: metres and radians. World frame: +Z up, the arm's base at the origin.

#ifndef SRC_HELD_MODULE_FINDER_H_
#define SRC_HELD_MODULE_FINDER_H_

#include <numbers>
#include <optional>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/camera_model.h"
#include "src/depth_camera.h"
#include "src/module_finder.h"
#include "src/pallet.h"
#include "src/rectangle_fit.h"

namespace cell {

struct HeldFinderOptions {
  // The module's design size.
  double module_length = kModuleLength;
  double module_width = kModuleWidth;
  double module_thickness = kModuleThickness;

  // How far the module may be from the expected pose and still be looked
  // for: its centre within position_tolerance in every direction, and turned
  // within angle_tolerance about every axis. The grip offsets this stage is
  // for are millimetres and a degree or so; a module further off than this
  // has slipped (step 9 refuses it).
  double position_tolerance = 0.05;                        // m
  double angle_tolerance = 3.0 * std::numbers::pi / 180.0;  // rad

  // The crop: the tolerances' reach at the corners, plus this.
  double crop_extra = 0.03;  // m

  // Refuse (no module) if the crop holds fewer points than this fraction of
  // the pixels the expected underside covers in the image.
  double min_point_fraction = 0.5;

  // The expected underside's corners, grown by the crop's margin, must
  // project at least this many pixels inside the image.
  double image_margin = 2.0;  // pixels

  // The plane. Defaults as for the pallet, except that RANSAC only looks
  // at planes within 6 degrees of the expected underside's direction (the
  // angle tolerance, twice), since nothing else in the crop faces down.
  PlaneOptions plane = [] {
    PlaneOptions p;
    p.max_normal_angle = 6.0 * std::numbers::pi / 180.0;
    return p;
  }();

  // FitRectangle's, with its size tolerance of 3 cm.
  RectangleFitOptions rectangle;

  // Before the final fit of the plane, points this close to the outline are
  // left out, so edge pixels do not tip it.
  double interior_margin = 0.03;  // m

  ViewOptions view;
};

enum class HeldFinderFailure {
  kNone,
  kBadInput,         // image and camera do not match, or an empty image
  kNotInView,        // the expected underside is not wholly in the picture
  kTooFewPoints,     // too few points where the underside can be: no module
  kNoPlane,          // no plane facing the camera as the underside should
  kWrongTilt,        // a plane, but tipped further than the tolerance
  kNoRectangle,      // FitRectangle refused: a side missing or not square
  kSizeMismatch,     // a rectangle, but not the module's size
  kViewBlocked,      // something between the camera and the underside, or
                     // holes in it
  kFarFromExpected,  // a module, but further from the expected pose than
                     // the tolerances: it has slipped on the cups
};

// A name for logs, such as "not in view".
const char* HeldFinderFailureName(HeldFinderFailure failure);

// What happened at each step, kept whether the answer is given or refused.
struct HeldFinderDiagnostics {
  HeldFinderFailure failure = HeldFinderFailure::kNone;
  std::string detail;  // one sentence, with the numbers that failed

  // Pixels the expected underside covers in the image (its outline's area
  // in pixels), and points left after each step.
  double expected_pixels = 0.0;
  int pixels_read = 0;
  int points_in_crop = 0;
  int plane_inliers = 0;
  int interior_points = 0;
  int rectangle_dots = 0;

  // The plane: its tilt from the expected underside's direction (first and
  // final fit), and the scatter of the inside points about it.
  double first_tilt = 0.0;  // rad
  double tilt = 0.0;        // rad
  double plane_rms = 0.0;   // m
  double plane_normalised_rms = 0.0;

  // The rectangle as FitRectangle gave it: size, each side's residual and
  // how far its dot counting moved it outward, and the dot spacing.
  double length = 0.0, width = 0.0;  // m
  double side_rms[4] = {0, 0, 0, 0};          // m
  double edge_correction[4] = {0, 0, 0, 0};   // m
  double spacing = 0.0;                       // m

  ViewCheck view;

  // How far the answer is from the expected pose, in the expected module's
  // frame.
  Eigen::Vector3d offset_from_expected = Eigen::Vector3d::Zero();  // m
  double angle_from_expected = 0.0;  // rad, the whole turn

  // Seconds spent: points and crop, plane, outline, view check, total.
  double seconds_points = 0.0, seconds_plane = 0.0, seconds_outline = 0.0,
         seconds_view = 0.0, seconds_total = 0.0;
};

struct HeldModuleEstimate {
  // The module's frame in the world: centre, +x long side, +y short side,
  // +z out of the glass (up, at the check pose).
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  // The centre of the underside, and the direction out of it, toward the
  // camera.
  Eigen::Vector3d underside_centre = Eigen::Vector3d::Zero();
  Eigen::Vector3d underside_normal = -Eigen::Vector3d::UnitZ();
  double length = 0.0;  // m, as measured
  double width = 0.0;
  // One standard deviation of each part, from the scatter of the points
  // used; statistical only (no calibration error, nothing that is the same
  // in every picture). In the module's own frame: along_normal is along its
  // z, along_length its x, up_lean its y (the short side here: the field's
  // name is the pallet's), face_direction the tilt of its z, and
  // in_plane_rotation the turn about its z. Over 100 noisy pictures on the
  // rig (print_check --noise) the errors over these came out at 0.45 to 0.9
  // root mean square: honest to cautious, except along the normal, where
  // the image's millimetre quantisation adds a few micrometres the fit does
  // not see (1.6); none of it matters for placement.
  PoseUncertainty sigma;
  HeldFinderDiagnostics diagnostics;
};

// The held module's pose in the world from one depth image of its underside,
// or nothing, with the reason in `diagnostics` (if not null; filled in also
// on success), if the picture does not show it clearly: see the steps at
// the top. `expected_module_pose_in_world` is where the control code thinks
// the module is: the tool tip's pose from the arm's joint angles, times the
// design grip offset (DesignModuleInTip). The same image, camera, expected
// pose and options give the same answer.
std::optional<HeldModuleEstimate> FindHeldModule(
    const DepthImage& image, const CameraModel& camera,
    const Eigen::Isometry3d& expected_module_pose_in_world,
    const HeldFinderOptions& options = {},
    HeldFinderDiagnostics* diagnostics = nullptr);

// ------------------------------------------------------- the frame algebra
//
// A pose "B in A" is the transform that takes coordinates in frame B to
// coordinates in frame A. Chaining reads right to left like the words:
//   module in world = tip in world * module in tip.
// Each function below is that one line solved for a different factor.

// The design's grip: the module's frame in the tool tip's frame when the
// cups hold the middle of its glass squarely, the glass `gap` in front of the
// tip (the pick's grip gap; 0 for cups pressed on the glass). The tip's +Z
// points into the glass, so the module's centre is gap + thickness / 2 along
// the tip's +Z; the tool's X runs along the module's +x (the pick builds it
// so, see GripPose in src/ik.h), and the module's +z, out of the glass, is
// the tip's -Z: half a turn about X.
Eigen::Isometry3d DesignModuleInTip(double thickness = kModuleThickness,
                                    double gap = 0.0);

// Where the module sits on the tool, measured: module in tip = (tip in
// world)^-1 * (module in world). `measured_module_pose_world` is
// FindHeldModule's answer, and `tool_tip_pose_world` the tool tip's pose
// from the arm's joint angles at the moment of the picture (forward
// kinematics; TipPoseAt in src/ik.h). Exact frame algebra, nothing more.
Eigen::Isometry3d GripOffsetFromCheck(
    const Eigen::Isometry3d& measured_module_pose_world,
    const Eigen::Isometry3d& tool_tip_pose_world);

// Where the tool tip must go so that the MODULE lands on
// `desired_module_pose_world`, given where it sits on the tool:
//   tip in world = (module in world) * (module in tip)^-1.
// The plan's "cup target = module target combined with the inverse of the
// measured cup-to-module offset". In words: a module held 5 mm to one side
// of the design sends the tip 5 mm to the other side, and one held turned
// half a degree turns the tip half a degree back, about the module's centre
// (not about the tip), so that the module, not the tool, ends on target.
// With DesignModuleInTip as the offset this is the uncorrected place.
Eigen::Isometry3d CorrectedTipTarget(
    const Eigen::Isometry3d& desired_module_pose_world,
    const Eigen::Isometry3d& measured_module_in_tip);

}  // namespace cell

#endif  // SRC_HELD_MODULE_FINDER_H_
