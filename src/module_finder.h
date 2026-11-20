// Finding the front module from one depth image: where its glass is and
// which way it faces, so that the arm can put its suction tool on it.
//
// The depth camera hangs 4 m up above the arm's base and looks down at the
// pallet. The modules stand on edge, leaning back 10 degrees, glass toward
// the arm. The front one is the one to pick. Its pose comes out of one
// picture in these steps, each a function below that can be tried alone:
//
//   1. Points       Every pixel with a reading becomes a 3D point in the
//                   world, through the camera's pose. The world here is the
//                   arm's base frame: the arm stands at the world origin,
//                   unturned, so "camera in the world" IS the camera-to-arm
//                   transform, and an error in it (a slightly wrong
//                   calibration) moves every point, and so the answer, by
//                   the same rigid motion. Nothing in one picture can see or
//                   undo that; it passes straight through.
//
//   2. Crop         Only points where the stack can be are kept: a box
//                   round every place the front glass can be, from the
//                   design (nominal pallet pose, how far a pallet may be set
//                   down off it, how deep the stack is, the module's size),
//                   plus a margin, and above the floor. Never from where the
//                   modules truly are.
//
//   3. Glass plane  RANSAC (src/plane_fit.h) on the cropped points, but only
//                   over planes facing within a few degrees of the design's
//                   glass direction. In the crop the stack's top (thirty
//                   module edges side by side) and the pallet deck are each
//                   as big as the glass, and both are square to it; the
//                   direction check means they are never even counted.
//                   A point belongs to the glass if it is within 3 standard
//                   deviations of the camera's depth noise of the plane,
//                   worked out for that point's own depth and viewing angle.
//
//   4. Flatten      The glass's points are put onto the plane, in 2D, with
//                   x along the level direction in the plane (the long side,
//                   by design) and y up the lean.
//
//   5. Outline      The 2.3 x 1.1 m rectangle in the flattened dots
//                   (src/rectangle_fit.h), checked for size. Then the top
//                   and bottom edges are measured again from the surfaces
//                   that meet the glass there (see "Creases" below).
//
//   6. Clear view   Every pixel inside the outline must read the glass. If
//                   something stands in front of it (the arm) or the glass
//                   has holes, the answer is refused.
//
//   7. Pose         The module's frame: origin at the module's centre, half
//                   a thickness behind the glass; +z out of the glass toward
//                   the arm; +x along the long side, pointing the way the
//                   design's +x does (world -Y, to the right as seen from the
//                   arm), which settles the half-turn ambiguity of a
//                   rectangle; +y = z cross x, up the lean.
//
// Creases. With modules behind it, the glass's top edge runs straight into
// the stack's top surface and its bottom edge into the deck: both square to
// the glass, so the depth does not step there, it only bends. Points on those
// surfaces within the inlier distance of the glass's plane count as glass
// (about one row of pixels at each edge), and flattened at right angles onto
// the plane they land exactly ON the edge line, since their surface is square
// to the plane. They do not push the outline outward by their depth, but
// they pile extra dots on the edge, which FitRectangle's dot counting reads
// as more glass: its top edge comes out about 7 mm too high and its bottom
// edge 9 to 10 mm too low (measured, the same with or without noise and at
// every stack depth), so its width is 16 to 18 mm too big and its centre
// about 1.2 mm low. They also tip the plane: those at the top lie behind it,
// those at the bottom in front, which leans it back about 0.02 degrees
// further. Both are dealt with:
//   - the long edges are measured from the creases instead. The points a
//     little beyond the inlier distance on the module's own top face (behind
//     the glass) and on the deck (in front of it) also all flatten onto the
//     edge lines, whatever their depth. A straight line through the few
//     hundred of them along each edge, with one slope for both, gives each
//     edge and the module's turn in its plane to a fraction of a millimetre
//     and a hundredth of a degree, free of the pixel grid. With only the
//     last module left its top edge is a step, but the module's own 35 mm
//     top face is still seen, so the same works.
//   - the plane is fitted again to the glass's inside only, 3 cm in from the
//     outline, away from the creases and the ends' flying pixels.
// The left and right ends are depth steps, measured by FitRectangle alone;
// its dot counting takes care of the pixel grid and of the dropped and
// flying pixels there, which are the same at both ends and so shorten the
// module by a few millimetres without moving its centre. With only the last
// module left, the back rest shows beside its ends 35 mm behind the glass:
// about nine noise standard deviations, so never on the glass's plane.
//
// Nothing here reads the truth: it sees only the depth image, the camera
// model it is handed and the design (PalletPrior).
//
// Units: metres and radians. World frame: +Z up, the arm's base at the origin.

#ifndef SRC_MODULE_FINDER_H_
#define SRC_MODULE_FINDER_H_

#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "src/camera_model.h"
#include "src/depth_camera.h"
#include "src/pallet.h"
#include "src/plane_fit.h"
#include "src/rectangle_fit.h"

namespace cell {

// How far a pallet may be set down off its nominal place, as the cell's
// Randomization defaults have it (src/cell.h): up to this much in each of x
// and y, and turned up to this much about the vertical. Design figures: the
// forklift's tolerance, which perception may know.
inline constexpr double kPlacementTolerance = 0.03;  // m
inline constexpr double kPlacementYawTolerance =
    3.0 * std::numbers::pi / 180.0;  // rad

// What perception knows before it looks: the design of the pallet and stack,
// and how far off a pallet may be set down. Nothing in it comes from where
// things truly are.
struct PalletPrior {
  // Where module_0 was designed to stand: its centre and axes (+x long side,
  // +y up the lean, +z out of the glass).
  Eigen::Isometry3d front_module = Eigen::Isometry3d::Identity();

  // How many modules a full pallet holds, and how far the front glass moves
  // back, along the glass's normal, with each module taken: one thickness.
  int module_count = 0;
  double pitch = kModuleThickness;

  double module_length = kModuleLength;
  double module_width = kModuleWidth;
  double module_thickness = kModuleThickness;

  // The pallet turns about the vertical line through this point when it is
  // set down turned (the floor under module_0's glass centre).
  Eigen::Vector3d pallet_origin = Eigen::Vector3d::Zero();
  double placement_tolerance = kPlacementTolerance;
  double placement_yaw_tolerance = kPlacementYawTolerance;

  // The floor's height in the world.
  double floor_height = 0.0;

  // module_0's glass as designed: the frame at the centre of its glass face,
  // with the module's axes.
  Eigen::Isometry3d FrontGlass() const;
  // Out of the design's glass, toward the arm.
  Eigen::Vector3d GlassNormal() const;
  // The design's glass leans back by this much (10 degrees).
  double DesignLean() const;
};

// The prior from the design poses FindPallet read out of the model as built
// (PalletBodies::nominal_pallet and nominal_modules): those are where the
// pallet and modules were designed to be, not where they are.
PalletPrior PriorFromDesign(
    const PalletBodies& bodies,
    double placement_tolerance = kPlacementTolerance,
    double placement_yaw_tolerance = kPlacementYawTolerance);

// ------------------------------------------------------------ 1. points

// Every pixel with a reading, as a point in the arm's base frame, with the
// pixel it came from. The arm's base is the world origin, so this is
// PointsWithPixels in the world frame: `camera.pose` is the camera-to-arm
// transform. Hand in the calibrated camera, not the true one; the two
// differ when the calibration is off, and the estimate then moves with it.
PixelPoints ImageToArmFrame(const DepthImage& image, const CameraModel& camera);

// ------------------------------------------------------------- 2. crop

// A box in the frame of the design's front glass (see
// PalletPrior::FrontGlass), and a height above the floor.
struct StackCrop {
  Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();  // box in world
  Eigen::Vector3d low = Eigen::Vector3d::Zero();   // corners, in `frame`
  Eigen::Vector3d high = Eigen::Vector3d::Zero();
  double min_height = 0.0;  // world z

  bool Contains(const Eigen::Vector3d& world) const;
};

// The box holding every place the front glass can be, plus the surfaces just
// round it that the outline step looks at. In the design's front glass
// frame: x across the full length, y across the full width, z from the
// front glass to where the last module's glass is, each widened by a margin
// that covers the pallet being set down off nominal: the shift
// (placement_tolerance in x and in y, so sqrt(2) times it), plus the turn
// (placement_yaw_tolerance times the farthest any glass corner is from the
// line it turns about), plus `extra` for everything the design does not
// say (the stack settling, a bowed module). About 0.18 m for this cell.
// And above the floor by `floor_clearance`, so that most of the floor round
// the pallet is left out however the box is tilted. Keep it small: the deck
// slopes down away from the arm, and the last module's glass ends only
// 28 mm above the floor, with the deck in front of it lower still. (What
// floor is left is dropped by the glass plane's direction check.)
StackCrop CropFromPrior(const PalletPrior& prior, double extra = 0.05,
                        double floor_clearance = 0.01);

// Indices of the points inside the crop, in order.
std::vector<int> CropIndices(const Points& points, const StackCrop& crop);

// ------------------------------------------------------ 3. glass plane

// The depth camera's noise as its datasheet states it: a reading at depth z
// has standard deviation a + b z^2 along the viewing axis. The defaults are
// the camera's (DepthNoise in src/depth_camera.h). Perception uses it only
// to decide how far off the plane a point may be and still be on it.
struct NoiseSpec {
  double a = DepthNoise{}.sigma_a;  // m
  double b = DepthNoise{}.sigma_b;  // 1/m

  double Sigma(double depth) const { return a + b * depth * depth; }
};

// The standard deviation of a point's distance from a plane with unit normal
// `normal`, given its depth noise. A depth error e moves the point along its
// own ray (direction `ray` from the camera, unit) by e / cos(ray, viewing
// axis), and of that move only the part along the normal takes it off the
// plane. At this cell's 45 degree view of the glass that is about 0.75 of
// the depth noise: 2.8 mm at 4 m, 3.8 mm at 4.9 m.
double OffPlaneSigma(const NoiseSpec& noise, const CameraModel& camera,
                     const Eigen::Vector3d& point,
                     const Eigen::Vector3d& normal);

// A plane facing the arm.
struct GlassPlane {
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();  // unit, out of the glass
  Eigen::Vector3d point = Eigen::Vector3d::Zero();    // any point on it

  // Positive in front of the glass (toward the arm), negative behind it.
  double SignedDistance(const Eigen::Vector3d& p) const {
    return normal.dot(p - point);
  }
};

struct PlaneFitResult {
  GlassPlane plane;
  // Indices into the points given, of those within their own inlier
  // distance (inlier_sigmas times OffPlaneSigma) of `plane`.
  std::vector<int> inliers;
  int ransac_inliers = 0;    // supporters of the RANSAC winner, all points
  double ransac_distance = 0.0;  // the fixed inlier distance RANSAC used, m
  double min_inlier_distance = 0.0;  // per-point limits used, m
  double max_inlier_distance = 0.0;
  // Root mean square distance of the inliers from the plane, m, and the same
  // in units of each point's own noise (about 1 when the noise model is
  // right and the glass is flat).
  double rms_error = 0.0;
  double normalised_rms = 0.0;
};

struct PlaneOptions {
  NoiseSpec noise;

  // A point is on the plane if within this many of its own off-plane
  // standard deviations. Three keeps 99.7% of the glass and leaves out the
  // back rest 35 mm behind the last module (9 standard deviations there).
  double inlier_sigmas = 3.0;

  // RANSAC only considers planes within this angle of the design's glass
  // direction. The pallet may be turned up to 3 degrees; a plane through
  // three noisy points is a degree or so off even on the glass; the stack's
  // top and the deck are 80 degrees off, the floor 80, the stand's front
  // face 10. Eight degrees keeps the first and drops the rest.
  double max_normal_angle = 8.0 * std::numbers::pi / 180.0;

  int ransac_iterations = 1000;
  // RANSAC only has to tell the planes apart, and a sample of the points
  // does that as well as all of them: it is given every k-th point, k
  // chosen to give at most this many. Its winner is then refined on all of
  // them. The crop holds about 100,000; 25,000 makes RANSAC four times
  // quicker and still gives the glass over 10,000 supporters.
  int ransac_points = 25000;
  // Fewer supporters than this is no glass at all: the last module alone
  // shows over 40,000 pixels.
  int min_inliers = 5000;
  std::uint32_t seed = 1;
};

// The front glass's plane among `points` (world, already cropped).
//
// RANSAC runs with one fixed inlier distance, inlier_sigmas times the
// off-plane noise of the deepest point given (the most generous any point
// needs), over planes facing within max_normal_angle of prior.GlassNormal().
// The winner is then refined: points within their own inlier distance are
// kept and the plane is fitted again by least squares, each point weighted
// by one over its noise variance, until the set settles. Returns nothing,
// with a reason in `detail` if not null, if no such plane has min_inliers
// supporters.
std::optional<PlaneFitResult> FindGlassPlane(const Points& points,
                                             const CameraModel& camera,
                                             const PalletPrior& prior,
                                             const PlaneOptions& options = {},
                                             std::string* detail = nullptr);

// The weighted least-squares plane through `points[indices]`, each weighted
// by one over its own off-plane variance for `normal_hint` (a plane facing
// about that way), with the normal turned to face the same way as the hint.
// Also gives the standard errors of the fit: of the plane's position along
// its normal at the weighted centroid, and of its tilt about the two
// in-plane axes, scaled by how well the points actually fit.
struct WeightedPlane {
  GlassPlane plane;
  double sigma_offset = 0.0;  // m
  double sigma_tilt = 0.0;    // rad, both tilts together
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
};
std::optional<WeightedPlane> FitWeightedPlane(
    const Points& points, const std::vector<int>& indices,
    const CameraModel& camera, const NoiseSpec& noise,
    const Eigen::Vector3d& normal_hint);

// -------------------------------------------------------- 4. flatten

// Two axes in the plane and an origin on it.
//   x  level: up cross normal, so along the module's long side by design,
//      and pointing the way the design's +x does when the normal faces the
//      arm.
//   y  up the lean: normal cross x.
// with normal = x cross y. Well defined for any plane that is not level.
struct PlaneFrame {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  Eigen::Vector3d x = Eigen::Vector3d::UnitX();
  Eigen::Vector3d y = Eigen::Vector3d::UnitY();
  Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();

  // At right angles onto the plane: the 2D coordinates of the foot of the
  // perpendicular from `p`.
  Eigen::Vector2d Flatten(const Eigen::Vector3d& p) const;
  // The point on the plane at 2D coordinates `q`. Lift(Flatten(p)) is p
  // moved onto the plane at right angles.
  Eigen::Vector3d Lift(const Eigen::Vector2d& q) const;
};

// The frame on `plane` whose origin is the foot of the perpendicular from
// `near` (the design's glass centre, so the numbers stay small).
PlaneFrame FlatteningFrame(
    const GlassPlane& plane, const Eigen::Vector3d& near,
    const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ());

// points[indices], flattened.
Points2d Flatten(const Points& points, const std::vector<int>& indices,
                 const PlaneFrame& frame);

// --------------------------------------------------------- 5. outline

// One of the two long edges measured from the surface meeting the glass
// there (see "Creases" at the top). In the rectangle's own frame (s along
// its axis from its centre, t across it), the edge is the line
// t = offset + slope s.
struct CreaseEdge {
  bool found = false;
  int points = 0;       // band points on the line
  double offset = 0.0;  // m
  double slope = 0.0;
  double rms = 0.0;     // m, of the band points about the line
  // How far the edge is from the rectangle fit's own edge, outward positive.
  double shift = 0.0;   // m
};

struct OutlineOptions {
  RectangleFitOptions rectangle;

  // Measure the long edges from the creases. False keeps the rectangle fit's
  // own edges, for comparison.
  bool use_creases = true;
  // Band points: within this of the rectangle fit's edge across it...
  double crease_window = 0.025;  // m
  // ... beyond the point's own inlier distance from the glass's plane, but
  // no farther than this: inside the module's own 35 mm thick top face, so
  // that only it counts with the last module left (behind it, the next
  // thing is the floor); and on the deck in front.
  double crease_depth = 0.030;  // m
  // ... and not within this fraction of the length of either end, where the
  // corners are.
  double crease_end_margin = 0.10;
  // Fewer than this and the rectangle fit's own edge is kept.
  int min_crease_points = 150;
  // Band points more than this many robust standard deviations off the
  // first line are dropped and the line fitted again.
  double crease_reject_sigmas = 3.0;
};

struct ModuleOutline {
  // In the flattening frame, canonical (see OrientedRectangle): axis along
  // +x of the frame, so along the design's +x.
  OrientedRectangle rectangle;
  // What FitRectangle gave, before the creases.
  RectangleFit fit;
  CreaseEdge top;     // the +Across() side: up the lean
  CreaseEdge bottom;  // the -Across() side
  // Standard errors, m and rad: of the centre along the axis and across it,
  // and of the axis's direction, from the scatter of the edge dots and band
  // points, plus an allowance for the pixel grid where FitRectangle's edges
  // are used (see SideSigma in the .cc).
  double sigma_along = 0.0;
  double sigma_across = 0.0;
  double sigma_angle = 0.0;
};

// Why the outline step refused.
struct OutlineRefusal {
  RectangleRefusal rectangle;  // FitRectangle's reason, if it refused
  bool width_mismatch = false; // the creases gave the wrong width
  std::string detail;
};

// The module's outline. `points` are the cropped world points and `glass`
// their plane's inliers; `frame` the flattening frame on that plane. The
// rectangle is fitted to the flattened inliers, and its long edges are then
// measured again from the crease bands among all of `points`. Refuses if
// FitRectangle does, or if the creases' width differs from the expected by
// more than the rectangle's size tolerance.
std::optional<ModuleOutline> FitModuleOutline(
    const Points& points, const PlaneFitResult& glass, const PlaneFrame& frame,
    const CameraModel& camera, const PalletPrior& prior,
    const PlaneOptions& plane_options, const OutlineOptions& options = {},
    OutlineRefusal* refusal = nullptr);

// ------------------------------------------------------- 6. clear view

// What the pixels whose rays meet the glass's plane inside the outline
// (shrunk by `margin`) read.
struct ViewCheck {
  int pixels = 0;    // rays meeting the plane inside the shrunk outline
  int on_glass = 0;  // read within `sigmas` of their off-plane noise
  int in_front = 0;  // read something nearer: something in the way
  int behind = 0;    // read something farther: a hole in the glass
  int missing = 0;   // read nothing
  double BlockedFraction() const {
    return pixels == 0 ? 1.0
                       : static_cast<double>(pixels - on_glass) / pixels;
  }
};

struct ViewOptions {
  // Ignore this much round the edge, where pixels see part glass and part
  // background.
  double margin = 0.03;  // m
  // A reading counts as the glass within this many off-plane standard
  // deviations: far wider than the inlier distance, so that the noise's
  // tail is not taken for an obstacle.
  double sigmas = 6.0;
  // Refuse if more than this fraction of the pixels do not read the glass.
  // A clear view gives none (the noise model drops pixels only at depth
  // edges); the arm's tool in front gives tens of percent.
  double max_blocked_fraction = 0.01;
};

ViewCheck CheckView(const DepthImage& image, const CameraModel& camera,
                    const GlassPlane& glass, const PlaneFrame& frame,
                    const OrientedRectangle& outline, const NoiseSpec& noise,
                    const ViewOptions& options = {});

// ------------------------------------------------------------ 7. pose

// The module's pose from its outline on the glass's plane: see step 7 at the
// top. `frame` must be on the plane of the glass, its normal out of it.
Eigen::Isometry3d ModulePoseFromOutline(const PlaneFrame& frame,
                                        const OrientedRectangle& outline,
                                        double thickness);

// ------------------------------------------------------ all together

struct FinderOptions {
  PlaneOptions plane;
  OutlineOptions outline;
  ViewOptions view;

  // Crop margins; see CropFromPrior.
  double crop_extra = 0.05;
  double floor_clearance = 0.01;

  // Refuse a glass plane whose lean differs from the design's by more than
  // this: the module is not standing as a module on this pallet stands.
  double max_lean_error = 1.5 * std::numbers::pi / 180.0;

  // Before the final fit of the plane, points this close to the outline are
  // left out, so the crease points (on the stack's top and the deck, which
  // sit within the inlier distance behind and in front of the plane at its
  // edges) do not tip it, nor do edge pixels.
  double interior_margin = 0.03;  // m
};

enum class FinderFailure {
  kNone,
  kBadInput,       // image and camera do not match, or an empty image
  kTooFewPoints,   // too few points where the stack can be
  kNoPlane,        // no plane facing the way the glass does
  kWrongLean,      // a plane, but not leaning as the glass does
  kNoRectangle,    // FitRectangle refused: a side missing or not square
  kSizeMismatch,   // a rectangle, but not the module's size
  kViewBlocked,    // something in front of the glass, or holes in it
};

// A name for logs, such as "view blocked".
const char* FinderFailureName(FinderFailure failure);

// What happened at each step, kept whether the answer is given or refused.
struct FinderDiagnostics {
  FinderFailure failure = FinderFailure::kNone;
  std::string detail;  // one sentence, with the numbers that failed

  // Points left after each step.
  int pixels_read = 0;     // pixels with a reading
  int points_in_crop = 0;
  int ransac_inliers = 0;
  int plane_inliers = 0;   // after the per-point refinement
  int interior_points = 0; // used for the final plane
  int rectangle_dots = 0;  // kept by FitRectangle's largest clump

  // The plane.
  double ransac_distance = 0.0;   // m
  double min_inlier_distance = 0.0, max_inlier_distance = 0.0;  // m
  double plane_rms = 0.0;         // m, of the interior points
  double plane_normalised_rms = 0.0;
  // The lean of the plane, and how far it is from the design's, for the
  // first plane (all inliers, crease points among them), which the lean
  // check uses, and for the final one (the glass's inside only). The
  // difference is the tilt the crease points give: about 0.02 degrees.
  double first_lean = 0.0;        // rad
  double first_lean_error = 0.0;  // rad
  double lean = 0.0;              // rad
  double lean_error = 0.0;        // rad

  // The rectangle: FitRectangle's own size and edge residuals, and the
  // final size.
  double fit_length = 0.0, fit_width = 0.0;  // m
  double side_rms[4] = {0, 0, 0, 0};         // m, FitRectangle's sides
  double spacing = 0.0;                      // m, between dots
  double length = 0.0, width = 0.0;          // m, final
  CreaseEdge top, bottom;

  ViewCheck view;

  // Seconds spent: points and crop, plane, outline, view check, total.
  double seconds_points = 0.0, seconds_plane = 0.0, seconds_outline = 0.0,
         seconds_view = 0.0, seconds_total = 0.0;
};

// One standard deviation of each part of the answer, from the scatter of
// the points used. Statistical only: a calibration error is not in it (one
// picture cannot see it; see step 1), nor anything else that is the same in
// every picture.
struct PoseUncertainty {
  double along_normal = 0.0;  // m
  double along_length = 0.0;  // m, along the long side
  double up_lean = 0.0;       // m, along the short side
  double face_direction = 0.0;      // rad, the normal's direction
  double in_plane_rotation = 0.0;   // rad, about the normal
};

struct ModuleEstimate {
  // The front module's frame in the world (the arm's base frame): centre of
  // the module, axes as in step 7 at the top. The same convention as
  // Cell::TrueModulePose, so the two compare directly.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  // The centre of its glass face, and the direction out of it, toward the
  // arm: what the tool aims at.
  Eigen::Vector3d glass_centre = Eigen::Vector3d::Zero();
  Eigen::Vector3d glass_normal = Eigen::Vector3d::UnitZ();
  double length = 0.0;  // m, as measured
  double width = 0.0;
  PoseUncertainty sigma;
  FinderDiagnostics diagnostics;
};

// The front module's pose from one depth image, or nothing if the picture
// does not show one clearly enough to pick: no glass plane facing the arm,
// one leaning wrongly, no module-sized rectangle on it, or something in
// front of it. `diagnostics`, if not null, receives what happened, also on
// refusal. The same image, camera and options give the same answer.
std::optional<ModuleEstimate> FindFrontModule(
    const DepthImage& image, const CameraModel& camera,
    const PalletPrior& prior, const FinderOptions& options = {},
    FinderDiagnostics* diagnostics = nullptr);

}  // namespace cell

#endif  // SRC_MODULE_FINDER_H_
