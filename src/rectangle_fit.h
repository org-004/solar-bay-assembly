// Rectangle fitting: finding the module's outline in a flat cloud of dots.
//
// The plane fit (src/plane_fit.h) picks out the points on the module's face.
// Flattened onto that plane they become 2D dots, one per camera pixel, that
// fill a rectangle 2.3 m by 1.1 m. This file finds that rectangle: its centre,
// which way its long side points, and its size. The steps, each of which is
// also offered on its own:
//
//   LargestCluster    Drops stray dots, from other surfaces that happen to
//                     cross the plane, by keeping the largest connected clump.
//
//   MinAreaRectangle  The smallest rectangle, at any rotation, that holds all
//                     the dots. Quick and never wildly wrong, but it rests on
//                     the few outermost dots only, so one ragged pixel tips
//                     or shifts it. Used as the first guess.
//
//   RefineRectangle   Fits a straight line to each of the four sides through
//                     the outermost dot of every short strip along that side,
//                     a hundred or so per long side, and intersects the
//                     lines for the corners. Averaging many edge dots instead
//                     of trusting the most extreme few is what makes it more
//                     accurate than the first guess.
//
//   FitRectangle      All of the above, then a check against the known module
//                     size. A mismatch refuses the pick.
//
// Dots are pixel centres, so the outermost dots sit up to one dot spacing
// inside the true edge. RefineRectangle corrects for that by counting dots:
// see RefineOptions::correct_edge_bias.
//
// Units are the caller's; metres in this repository. Angles are radians. The
// 2D frame is whatever the caller flattened onto; "anticlockwise" means from
// its +x axis toward its +y axis.

#ifndef SRC_RECTANGLE_FIT_H_
#define SRC_RECTANGLE_FIT_H_

#include <array>
#include <cstdint>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace cell {

using Points2d = std::vector<Eigen::Vector2d>;

// A rectangle at any rotation.
//
// A rectangle looks the same turned half a turn, so its long side has two
// directions. Every function here returns the canonical one: `axis` has a
// positive x component, or, for a long side exactly along y, a positive y
// component. Angle() is then in (-pi/2, pi/2]. A long side close to the y
// axis can come back as nearly +y one time and nearly -y the next; see
// FitRectangle.
struct OrientedRectangle {
  Eigen::Vector2d centre = Eigen::Vector2d::Zero();

  // Unit vector along the length.
  Eigen::Vector2d axis = Eigen::Vector2d::UnitX();

  // Along `axis`, and across it. Canonical rectangles have length >= width.
  double length = 0.0;
  double width = 0.0;

  // `axis` turned a quarter turn anticlockwise.
  Eigen::Vector2d Across() const { return {-axis.y(), axis.x()}; }

  // Of `axis` from the +x axis, anticlockwise.
  double Angle() const;

  // Anticlockwise, starting from centre - length/2 axis - width/2 Across().
  // Side k of a fit runs from corner k to corner k + 1: sides 0 and 2 are
  // the long sides, 0 on the -Across() side; 1 and 3 the short sides, 1 at
  // the +axis end.
  std::array<Eigen::Vector2d, 4> Corners() const;
};

// The same rectangle with length >= width and `axis` in its canonical
// direction.
OrientedRectangle Canonical(const OrientedRectangle& rectangle);

// The typical distance between neighbouring dots: the median distance from a
// dot to its nearest neighbour. On a camera's pixel grid that is the smaller
// of the two spacings. Points with a coordinate that is not finite are
// ignored, and so are repeated points. Returns nothing if fewer than two
// distinct points have a neighbour nearby.
std::optional<double> DotSpacing(const Points2d& points);

// Indices, in increasing order, of the points in the largest connected clump.
//
// The plane is cut into square cells of side `cell_size`, and dots in the
// same or touching cells (sideways or diagonally) are connected. So two dots
// closer than `cell_size` are always connected, and dots farther apart than
// 2 sqrt(2) cell_size never directly. Choose `cell_size` about twice the
// larger dot spacing: then a missing dot or two at the edge does not cut the
// face in two, and a stray more than about 6 spacings from the face is
// dropped. On the cell's grid of 6 by 9 mm that is 18 to 20 mm; three times
// DotSpacing() gives it, since that is the smaller spacing.
//
// Points with a coordinate that is not finite belong to no clump. Of two
// clumps of the same size, the one holding the lowest index wins. Returns
// nothing for no points or a `cell_size` that is not positive and finite.
std::vector<int> LargestCluster(const Points2d& points, double cell_size);

// The convex hull: the corners of the smallest convex polygon holding every
// point, anticlockwise, starting from the point with the smallest x (and of
// those the smallest y). Repeated points and points along a hull edge are
// left out. Fewer than three distinct points, or points all in one line,
// give the distinct points or the line's two ends. Points with a coordinate
// that is not finite are ignored.
Points2d ConvexHull(const Points2d& points);

// The smallest-area rectangle at any rotation that holds all of `points`,
// canonical. Returns nothing if the points have no area: fewer than three
// distinct ones, or all in one line.
//
// One side of the smallest rectangle always lies along an edge of the convex
// hull, so only as many rotations as the hull has edges need trying. For
// each, the rectangle is fixed by the hull corners farthest forward, farthest
// back and farthest from that edge. Rotating calipers: as the edge moves on
// round the hull, each of those three corners can only move on too, never
// back, so they are found by stepping three markers forward as the edge
// turns, and the whole search is one trip round the hull.
std::optional<OrientedRectangle> MinAreaRectangle(const Points2d& points);

// Why a fit was refused.
enum class RectangleFitFailure {
  kNone,
  kBadOptions,     // an option or argument out of range
  kNonFinite,      // an input coordinate is not a number or infinite
  kTooFewPoints,   // fewer dots than options.min_points
  kDegenerate,     // the dots have no area: all in one line, or one point
  kSideMissing,    // a side has too few edge dots on a straight line
  kNotSquare,      // a side's direction disagrees with the other three
  kSizeMismatch,   // not the expected size, within tolerance
};

// A name for logs, such as "size mismatch".
const char* FailureName(RectangleFitFailure failure);

struct RectangleRefusal {
  RectangleFitFailure reason = RectangleFitFailure::kNone;
  // One sentence, with the numbers that failed.
  std::string detail;
};

struct RefineOptions {
  // The typical distance between neighbouring dots, as DotSpacing() gives.
  // Zero to work it out from the dots. Every other length used inside is a
  // multiple of it: strips along a side are 3 spacings wide, an edge dot
  // more than 2.5 spacings off its side's line is an outlier, and the dots
  // counted for the edge correction lie 4 to 12 spacings inside the edge.
  double spacing = 0.0;

  // Rounds of: find edge dots against the current rectangle, fit the sides,
  // move the rectangle. The first round can start a few degrees and
  // centimetres off; after three the answer no longer changes.
  int iterations = 3;

  // Dots are pixel centres. The outermost dots of a side sit between zero
  // and one dot spacing inside the true edge: on average half a spacing for
  // a grid whose rows run along the edge, and a less predictable amount for
  // a slanted one. A rectangle through the outermost dots therefore comes
  // out up to two spacings short.
  //
  // The correction counts dots instead. Every dot stands for the same patch
  // of face, so the number of dots between a line inside the face and the
  // edge, divided by how many dots there are per unit of depth (read off a
  // band deeper inside), is how far that line is from the edge. This is
  // right on average whatever the grid's slant, and it needs no knowledge
  // of the grid, so a spacing that changes across the face (perspective)
  // does no harm. The band is weighted smoothly rather than cut sharply, so
  // where rows of dots happen to fall relative to it does not matter.
  //
  // What counting cannot fix: on a grid whose rows run exactly along an
  // edge, the edge can be anywhere in the gap beyond the last row, so the
  // answer can be off by up to half a row spacing in either direction (zero
  // on average). Dots missing along the edge move it inward by their share.
  //
  // False keeps the line through the outermost dots, for comparison.
  bool correct_edge_bias = true;

  // Fit one direction for all four sides, opposite sides parallel and
  // adjacent ones square, and report a true rectangle. False keeps each
  // side's own line and reports the four corners where they meet; centre,
  // axis and size are then averages.
  bool square_corners = true;

  // Each end of every side, this fraction of its length, is left out of the
  // search for edge dots: near a corner the outermost dot of a strip may
  // belong to the other side.
  double end_margin = 0.05;

  // A side needs at least this many edge dots on its line, and at least
  // this fraction of its strips, or it counts as missing. A side can be
  // partly hidden: of the strips along it, the line is fitted to the
  // outermost group of edge dots that lie in a straight line and has this
  // much support, so a bite out of most of a side still leaves its true
  // edge to be found from what is left.
  int min_side_dots = 8;
  double min_side_support = 0.2;

  // A side whose own direction differs from the shared one by more than
  // this, plus four times its own standard error, is not square with the
  // others and is not used. Long sides come out within a few hundredths of a
  // degree; short or partly hidden ones within a few tenths.
  double max_angle_error = std::numbers::pi / 180.0;

  // Positive values let one missing or unsquare side be placed from the
  // opposite side at this distance: known_length apart for the short sides,
  // known_width for the long ones. Zero refuses such a fit instead.
  double known_length = 0.0;
  double known_width = 0.0;

  // Seeds the random choices of the robust line fits, so the same input
  // gives the same answer.
  std::uint32_t seed = 1;
};

// One side of a fitted rectangle.
struct SideFit {
  // Its two corners, in anticlockwise order, and the unit normal pointing
  // out of the rectangle.
  Eigen::Vector2d start = Eigen::Vector2d::Zero();
  Eigen::Vector2d end = Eigen::Vector2d::Zero();
  Eigen::Vector2d outward = Eigen::Vector2d::Zero();

  int strips = 0;        // strips searched along the side
  int edge_dots = 0;     // strips that held a dot on this side
  int dots_used = 0;     // edge dots on the fitted line
  double support = 0.0;  // dots_used / strips

  // Root mean square distance of the edge dots used from the side's own
  // line. It measures the ragged edge, not the fit's error, which is far
  // smaller.
  double rms_residual = 0.0;

  // The side's own direction minus the rectangle's, anticlockwise.
  double angle_error = 0.0;

  // How far the side was moved outward from the line through its edge dots
  // by the correction described at RefineOptions::correct_edge_bias.
  double edge_correction = 0.0;

  // Placed from the opposite side and the known size, not from its own dots.
  bool from_known_size = false;
};

struct RectangleFit {
  // Canonical: see OrientedRectangle for which of the two directions along
  // the long side `axis` is.
  OrientedRectangle rectangle;

  // As OrientedRectangle::Corners(). With square_corners off, where the
  // sides' own lines meet.
  std::array<Eigen::Vector2d, 4> corners;

  // In the order of OrientedRectangle::Corners(): 0 and 2 long, 1 and 3
  // short.
  std::array<SideFit, 4> sides;

  // The rectangle the refinement started from: the smallest-area rectangle
  // in FitRectangle, the caller's in RefineRectangle.
  OrientedRectangle first_guess;

  double spacing = 0.0;    // the dot spacing used
  int dots_given = 0;      // points passed in
  int dots_kept = 0;       // in the largest clump; all of them in Refine
  double rms_residual = 0.0;  // over every side's edge dots used

  // One side was placed from the known size; see SideFit::from_known_size.
  bool used_known_size = false;
};

// The rectangle `first_guess` is close to, fitted to the edges of `points`.
//
// Each round, along each side of the current rectangle, the side's length
// (less the end margins) is cut into strips 3 spacings wide, and in each
// strip the dot farthest out on that side, among the dots in the rectangle's
// half nearer the side, is that strip's edge dot. One per strip, so a clump
// of dots on a ragged stretch cannot outvote the rest, dots deep inside
// never take part, and a bad dot spoils only its own strip. A line is fitted
// through the edge dots by RANSAC (random pairs; the line with most dots
// within 2.5 spacings wins) and then least squares on its supporters, so
// strips whose edge dot is a stray, a flying pixel or the boundary of
// something in front are left out. Then the four directions are combined
// into one, each side weighted by how tightly its edge dots follow its line,
// the rectangle is turned to it, and each side is moved to where the dot
// count puts its edge (see RefineOptions::correct_edge_bias).
//
// Every point is used: call LargestCluster first. Returns nothing, saying
// why in `refusal` if given, for a coordinate that is not finite, a bad
// option or first guess, a side missing or not square (unless the known
// size may stand in), or a result with no area.
std::optional<RectangleFit> RefineRectangle(
    const Points2d& points, const OrientedRectangle& first_guess,
    const RefineOptions& options = {}, RectangleRefusal* refusal = nullptr);

struct RectangleFitOptions {
  RefineOptions refine;

  // For LargestCluster. Zero for three times the dot spacing.
  double cluster_cell_size = 0.0;

  // Fewer dots than this, before or after dropping strays, are refused. A
  // module seen by the cell's camera gives tens of thousands.
  int min_points = 500;

  // The fitted length and width must each be within this of the expected
  // ones. Three centimetres: the fit's own error is at most about one dot
  // spacing (9 mm on the cell's grid, 18 mm on a grid twice as coarse, and
  // only on a grid whose rows run along a side), and modules are made to
  // within a few millimetres. A wrong object is off by far more: a module
  // turned on edge, two modules seen as one, or a part of one hidden by
  // something straight.
  double size_tolerance = 0.03;

  // Let one side that is missing or not square, as when one end is hidden
  // behind something with an irregular outline, be placed from the opposite
  // side at the expected distance. The size check then applies to the other
  // dimension only, and the fit is still refused if more than a few dots lie
  // beyond the placed side. Off by default: a pick should rest on all four
  // edges having been seen.
  bool allow_missing_side = false;
};

// The module's rectangle in `points`: strays dropped (LargestCluster), first
// guess (MinAreaRectangle), refined (RefineRectangle), and checked against
// the expected length and width.
//
// Returns nothing, saying why in `refusal` if given, for: a bad argument;
// any coordinate that is not finite (the dots come from plane inliers, which
// are always finite, so one that is not means something upstream is
// broken); too few dots; dots in a line; a side missing or not square; or a
// size off by more than options.size_tolerance.
//
// What a partly hidden module does:
//   - A bite out of a corner or a side, leaving at least min_side_support of
//     every side visible, is handled: each side is fitted where it is seen,
//     and a hidden corner is where its two sides' lines meet.
//   - A whole end hidden behind something with a straight edge parallel to
//     it looks exactly like a shorter module and is refused as a size
//     mismatch: nothing in the dots says which end is missing.
//   - A whole end hidden behind something with an irregular or slanted
//     outline is refused (side missing or not square), unless
//     allow_missing_side is set, in which case that end is placed from the
//     other at the expected length.
//
// For a caller turning this into a pose: the long side's direction is the
// canonical one (see OrientedRectangle), so with the long side close to the
// 2D y axis it can flip by half a turn from one view to the next. Choose the
// 2D frame so that the long side is expected along x, or settle the
// direction with something only 3D knows, such as which way is up.
std::optional<RectangleFit> FitRectangle(
    const Points2d& points, double expected_length, double expected_width,
    const RectangleFitOptions& options = {},
    RectangleRefusal* refusal = nullptr);

}  // namespace cell

#endif  // SRC_RECTANGLE_FIT_H_
