// The depth camera's picture: a depth image of the cell, made by casting one
// ray through every pixel, an optional model of a real camera's errors, and
// the conversion from depth image to 3D points.
//
// A depth image is a picture whose pixels hold distances instead of colours.
// The usual convention, used here, is that a pixel holds the depth of what
// it sees: how far the point is in front of the camera, measured along the
// camera's viewing axis (its -Z), not along the pixel's own ray. Depth is
// the z of the pinhole model in src/camera_model.h. The distance along the
// ray, the range, is longer by 1 / cos(angle off the axis): at this camera's
// corners, 26 degrees off axis horizontally and 20 vertically, a range of
// 4.00 m is a depth of only 3.42 m. Depth makes going back to 3D a single
// multiplication: the point of pixel (u, v) at depth z is
// z * PointAtUnitDepth(camera, u, v) in the camera's frame. A flat wall
// square to the camera has the same depth at every pixel, but not the same
// range.
//
// How the image is made: one ray per pixel centre, through MuJoCo's
// mj_multiRay, against the geoms the camera sees. Not by rendering with
// OpenGL. Ray casting needs no OpenGL or EGL context and no display, so it
// runs the same in tests, in CI and on a headless machine, and gives exact
// geometry in double precision. A GPU depth render of the same scene (the
// depth buffer read back with mjr_readPixels and linearised with the near
// and far planes) would agree on what is seen and where: the same pinhole,
// the same occlusion, the same meshes (both use the triangles, not convex
// hulls). It would differ in:
//   - precision: the depth buffer is 24 or 32 bit non-linear in depth, so a
//     GPU depth image at 4 m is quantised at the millimetre level or finer
//     depending on the near plane, where rays are exact;
//   - sampling: the GPU decides coverage per pixel centre too, but with
//     anti-aliasing (multisampling) it averages edge pixels into a blend of
//     the two surfaces, which a real camera also does in its own way;
//   - what is drawn: the renderer shows geom groups by mjvOption, and also
//     draws things rays never hit, such as sites, tendons and decorations if
//     they are switched on; rays skip fully transparent geoms;
//   - speed: a GPU renders the whole image in a few milliseconds, where
//     the 307 200 rays of a 640 x 480 image take about 0.2 s optimized
//     (--config=opt) and about 1 s in the default unoptimized build
//     (tests/depth_camera_test.cc prints the figure). Fine for one picture
//     per pick; too slow to render every physics step.
//
// What the camera sees (DepthOptions::geom_groups): groups 0, 1 and 2, the
// groups MuJoCo's viewer draws by default. The floor, the pallet (its deck,
// back rest and the stand under it), the modules, the conveyor, the tube,
// the tool and the camera's post are all in group 0; the arm's drawn meshes
// are in group 2. Group 3 holds the arm's collision meshes, which are hidden
// in the viewer and are rougher and a little fatter than the drawn links: a
// real camera sees the drawn shape, so group 3 is left out. Groups 4 and 5
// are unused. mj_multiRay intersects a mesh geom as its true triangles
// (through the mesh's bounding volume tree), not as the convex hull the
// collision code uses, so the arm's visual meshes appear with their real
// hollows.
//
// Perception code may use everything here but RenderDepth's `true_geom`
// output, which says which geom each pixel saw. That is truth, for tests.
// RenderDepth itself reads the geometry through rays: that is the sensor.

#ifndef SRC_DEPTH_CAMERA_H_
#define SRC_DEPTH_CAMERA_H_

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/plane_fit.h"

namespace cell {

// A depth image: width x height values, row by row from the top-left pixel,
// so the pixel in column u and row v (counted down) is depth[v * width + u].
// Each value is the depth along the camera's viewing axis in metres, or NaN
// where the camera has no reading: nothing hit, beyond max_depth, or dropped
// by the noise model.
struct DepthImage {
  int width = 0;
  int height = 0;
  std::vector<float> depth;

  int index(int u, int v) const { return v * width + u; }
  float at(int u, int v) const { return depth[index(u, v)]; }
  float& at(int u, int v) { return depth[index(u, v)]; }
  bool has_reading(int u, int v) const { return std::isfinite(at(u, v)); }
  int size() const { return width * height; }
  // How many pixels have a reading.
  int CountReadings() const;
};

// A real camera's errors, each switched on by itself. All are off by default,
// so the image is exact. The random draws come from `seed` and std::mt19937
// (whose sequence the C++ standard fixes) through this file's own uniform
// and normal draws, so the same seed and image give the same noisy image.
// Each effect draws from its own stream, so switching one on or off does not
// change the others' draws.
//
// The order: edge effects first, on the exact image; then depth noise on
// every reading; then quantisation.
//
// The numbers are assumptions chosen to look like an industrial
// structured-light or stereo camera working at 3 to 5 m, not taken from any
// datasheet.
struct DepthNoise {
  // (a) Depth noise: each reading moves along the viewing axis by a draw
  // from a normal distribution with standard deviation a + b z^2 at depth z.
  // Stereo and structured-light cameras measure depth from a disparity, the
  // shift of a pattern between two viewpoints; the shift shrinks as 1 / z,
  // so a fixed error in measuring it grows as z^2. The constant a stands for
  // the error that does not shrink close up (surface texture, matching
  // jitter). With these: 1.0 mm at 1 m, 2.0 mm at 2.5 m, 3.3 mm at 3.8 m
  // (about where the front glass is from this camera) and 4.4 mm at 4.5 m.
  bool depth_noise = false;
  double sigma_a = 0.8e-3;   // m
  double sigma_b = 0.18e-3;  // 1/m, times z^2 in m^2

  // (b) Edge effects. Where the depth jumps between neighbouring pixels, a
  // real camera's pixel sees part of each surface. Stereo matching then
  // fails (no reading), and time-of-flight and structured light often
  // return a depth somewhere in between, a "flying pixel" floating in the
  // air between the two surfaces. A pixel is on an edge if one of its four
  // neighbours (left, right, up, down) has a depth differing from its own by
  // more than edge_threshold, or has no reading. Each edge pixel is hit with
  // probability edge_probability; a hit pixel is dropped (NaN) with
  // probability edge_dropout, and otherwise given depth
  // t * own + (1 - t) * neighbour with t uniform in (0, 1), the neighbour
  // being the one differing most. A pixel whose only edge is a neighbour
  // without a reading can only be dropped.
  //
  // Both sides of an edge are affected, so the module's edges, which the
  // next stage measures, come out ragged by about a pixel.
  //
  // The threshold of 2 cm sits between the steepest smooth surface in this
  // camera's view (neighbouring pixels at most about 8 mm apart in depth, on
  // the floor far beyond the pallet) and the smallest real step at a front
  // glass's edge (about 4.5 cm, from module_29's glass to the back rest
  // showing beside it). A front glass's top and bottom edges with modules
  // behind it are creases, not steps, and are left alone, as a real camera
  // would.
  bool edge_effects = false;
  double edge_threshold = 0.02;    // m, between neighbours' depths
  double edge_probability = 0.5;   // that an edge pixel is affected
  double edge_dropout = 0.5;       // that an affected pixel is dropped

  // (c) Quantisation: readings rounded to the nearest multiple of `quantum`,
  // as an image stored as whole millimetres is. (A stereo camera's own
  // steps are in disparity and so grow with depth; a fixed step is the
  // simpler model, and at 1 mm it is below the depth noise anyway.)
  bool quantize = false;
  double quantum = 1e-3;  // m

  std::uint32_t seed = 1;
};

struct DepthOptions {
  // Readings deeper than this, along the viewing axis, are NaN. Rays are
  // cast no further than needed to find them. A camera's working range
  // ends somewhere; 6 m is a usual upper limit for a structured-light
  // camera and leaves the whole stack, at 3.5 to 4.5 m, well inside. The
  // top rows of this camera's image see the floor beyond the pallet at up
  // to about 7 m and read nothing.
  double max_depth = 6.0;  // m

  // Which geom groups the camera sees: 1 to see the group, 0 not. See the
  // comment at the top of this file.
  std::array<mjtByte, mjNGROUP> geom_groups = {1, 1, 1, 0, 0, 0};

  // Geoms of this body are not seen; -1 for none. The camera's housing is
  // wholly behind its lens in models/cell.xml, so nothing needs excluding.
  int exclude_body = -1;

  DepthNoise noise;
};

// The depth image seen by `camera`, cast in `model` as `data` has it now.
// One ray per pixel centre, from the camera's centre; a pixel's depth is the
// distance to the first surface hit, times the cosine of the ray's angle
// off the viewing axis. Then the noise, if any is switched on.
//
// The pose comes from `camera`, not from `data`, so an image can be taken
// from anywhere; for the cell's own camera pass Cell::DepthCamera().
//
// What must be up to date in `data`: the geoms' and bodies' positions
// (geom_xpos, geom_xmat, xipos, ximat), which mj_kinematics computes, and so
// does mj_forward. After mj_step they are from before the step's last
// integration; call mj_forward (or mj_kinematics) first for the image of the
// current state. `data` is not const only because mj_multiRay takes its
// scratch memory from data's stack (mj_markStack / mj_freeStack), which it
// gives back before returning; no state and no computed quantity changes.
// (It would also mark bounding volumes for drawing if the model's
// visual/global/bvactive were set; it is off in this repository's models.)
//
// If `true_geom` is not null it receives, for every pixel, the id of the
// geom its ray hit, or -1 for none or beyond max_depth, before any noise.
// Truth: for tests only.
DepthImage RenderDepth(const mjModel& model, mjData* data,
                       const CameraModel& camera,
                       const DepthOptions& options = {},
                       std::vector<int>* true_geom = nullptr);

// Applies `noise` to `image`, as RenderDepth does after casting. Exposed so
// that one exact image can be given many noises without casting again.
void AddDepthNoise(DepthImage* image, const DepthNoise& noise);

// Which frame PointsFromDepth gives its points in.
enum class PointFrame {
  kWorld,   // through camera.pose: what the cell's planning works in
  kCamera,  // the camera's own frame: -Z forward, +X right, +Y up
};

// Points with the pixel each came from: points[k] was seen by pixel
// pixels[k], which is v * width + u.
struct PixelPoints {
  Points points;
  std::vector<int> pixels;
};

// The 3D point of every pixel that has a reading, in row order; pixels
// without one are skipped, so the cloud holds only real points (the plane
// fit would ignore NaN points, but other steps need not). Use the
// PixelPoints form to map a point back to its pixel.
//
// Each point is depth * PointAtUnitDepth(camera, u + 0.5, v + 0.5), moved
// into the world by camera.pose for kWorld. The image must be camera's size.
PixelPoints PointsWithPixels(const DepthImage& image, const CameraModel& camera,
                             PointFrame frame = PointFrame::kWorld);
Points PointsFromDepth(const DepthImage& image, const CameraModel& camera,
                       PointFrame frame = PointFrame::kWorld);

}  // namespace cell

#endif  // SRC_DEPTH_CAMERA_H_
