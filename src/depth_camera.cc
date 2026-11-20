#include "src/depth_camera.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/plane_fit.h"

namespace cell {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr float kNoReading = std::numeric_limits<float>::quiet_NaN();

// Uniform and normal draws from std::mt19937. Library distributions are not
// used: the standard leaves their algorithms to each library, and a seed
// must give the same image everywhere.
class Draws {
 public:
  explicit Draws(std::uint32_t seed) : random_(seed) {}

  // Strictly between 0 and 1: (k + 0.5) / 2^32 for one 32-bit output k.
  double Unit() {
    return (static_cast<double>(random_()) + 0.5) / 4294967296.0;
  }

  // Standard normal, by the Box-Muller transform: two uniform draws give
  // two independent normals, the second kept for the next call.
  double Normal() {
    if (has_spare_) {
      has_spare_ = false;
      return spare_;
    }
    const double radius = std::sqrt(-2.0 * std::log(Unit()));
    const double angle = 2.0 * kPi * Unit();
    spare_ = radius * std::sin(angle);
    has_spare_ = true;
    return radius * std::cos(angle);
  }

 private:
  std::mt19937 random_;
  double spare_ = 0.0;
  bool has_spare_ = false;
};

// The flying pixels and dropouts at depth edges, on `image` as it is.
void AddEdgeEffects(DepthImage* image, const DepthNoise& noise,
                    std::uint32_t seed) {
  const DepthImage exact = *image;  // decide every pixel on the exact image
  Draws draws(seed);
  for (int v = 0; v < exact.height; ++v) {
    for (int u = 0; u < exact.width; ++u) {
      // Three draws for every pixel, used or not, so that pixel k always
      // uses the same draws whatever the image holds.
      const double hit = draws.Unit();
      const double drop = draws.Unit();
      const double t = draws.Unit();
      if (!exact.has_reading(u, v)) continue;
      const float own = exact.at(u, v);

      bool edge = false;
      bool can_mix = false;
      float other = own;  // the neighbour differing most
      const int du[4] = {-1, 1, 0, 0};
      const int dv[4] = {0, 0, -1, 1};
      for (int k = 0; k < 4; ++k) {
        const int nu = u + du[k];
        const int nv = v + dv[k];
        if (nu < 0 || nu >= exact.width || nv < 0 || nv >= exact.height) {
          continue;  // the image's border is not an edge in the scene
        }
        if (!exact.has_reading(nu, nv)) {
          edge = true;
          continue;
        }
        const float neighbour = exact.at(nu, nv);
        if (std::abs(neighbour - own) > noise.edge_threshold) {
          edge = true;
          if (!can_mix || std::abs(neighbour - own) > std::abs(other - own)) {
            other = neighbour;
          }
          can_mix = true;
        }
      }
      if (!edge || hit >= noise.edge_probability) continue;
      if (!can_mix || drop < noise.edge_dropout) {
        image->at(u, v) = kNoReading;
      } else {
        image->at(u, v) = static_cast<float>(t * own + (1.0 - t) * other);
      }
    }
  }
}

}  // namespace

int DepthImage::CountReadings() const {
  return static_cast<int>(
      std::count_if(depth.begin(), depth.end(),
                    [](float z) { return std::isfinite(z); }));
}

DepthImage RenderDepth(const mjModel& model, mjData* data,
                       const CameraModel& camera, const DepthOptions& options,
                       std::vector<int>* true_geom) {
  DepthImage image;
  image.width = camera.width;
  image.height = camera.height;
  const int n = image.size();
  image.depth.assign(n, kNoReading);
  if (true_geom != nullptr) true_geom->assign(n, -1);
  if (n <= 0) return image;

  // One ray per pixel centre, in the world. `per_depth` is each ray's length
  // per metre of depth, 1 / cos(angle off the axis): range = depth *
  // per_depth.
  std::vector<mjtNum> directions(3 * n);
  std::vector<double> per_depth(n);
  double longest = 1.0;
  for (int v = 0; v < image.height; ++v) {
    for (int u = 0; u < image.width; ++u) {
      const int k = image.index(u, v);
      const Eigen::Vector3d unit_depth =
          PointAtUnitDepth(camera, u + 0.5, v + 0.5);
      per_depth[k] = unit_depth.norm();
      longest = std::max(longest, per_depth[k]);
      const Eigen::Vector3d ray =
          camera.pose.linear() * (unit_depth / per_depth[k]);
      directions[3 * k + 0] = ray.x();
      directions[3 * k + 1] = ray.y();
      directions[3 * k + 2] = ray.z();
    }
  }

  // mj_multiRay leaves out geoms whose bounding sphere lies wholly beyond
  // the cutoff, a range along the ray; it does not stop a ray there, so a
  // hit beyond can still come back and is checked below.
  const double cutoff = std::isfinite(options.max_depth)
                            ? options.max_depth * longest
                            : mjMAXVAL;
  const Eigen::Vector3d origin = camera.pose.translation();
  std::vector<int> geom(n, -1);
  std::vector<mjtNum> range(n, -1.0);
  mj_multiRay(&model, data, origin.data(), directions.data(),
              options.geom_groups.data(), /*flg_static=*/1,
              options.exclude_body, geom.data(), range.data(),
              /*normal=*/nullptr, n, cutoff);

  for (int k = 0; k < n; ++k) {
    if (range[k] < 0.0) continue;  // -1: the ray hit nothing
    const double depth = range[k] / per_depth[k];
    if (!(depth <= options.max_depth)) continue;
    image.depth[k] = static_cast<float>(depth);
    if (true_geom != nullptr) (*true_geom)[k] = geom[k];
  }

  AddDepthNoise(&image, options.noise);
  return image;
}

void AddDepthNoise(DepthImage* image, const DepthNoise& noise) {
  // One stream per effect, each seeded from the one seed.
  std::mt19937 seeds(noise.seed);
  const std::uint32_t edge_seed = seeds();
  const std::uint32_t depth_seed = seeds();

  if (noise.edge_effects) AddEdgeEffects(image, noise, edge_seed);

  if (noise.depth_noise) {
    Draws draws(depth_seed);
    for (float& z : image->depth) {
      // Drawn for every pixel, so pixel k always uses the same draw.
      const double step = draws.Normal();
      if (!std::isfinite(z)) continue;
      const double sigma = noise.sigma_a + noise.sigma_b * z * z;
      z = static_cast<float>(z + sigma * step);
    }
  }

  if (noise.quantize && noise.quantum > 0.0) {
    for (float& z : image->depth) {
      if (!std::isfinite(z)) continue;
      z = static_cast<float>(std::round(z / noise.quantum) * noise.quantum);
    }
  }
}

PixelPoints PointsWithPixels(const DepthImage& image, const CameraModel& camera,
                             PointFrame frame) {
  PixelPoints result;
  for (int v = 0; v < image.height; ++v) {
    for (int u = 0; u < image.width; ++u) {
      if (!image.has_reading(u, v)) continue;
      const Eigen::Vector3d in_camera =
          static_cast<double>(image.at(u, v)) *
          PointAtUnitDepth(camera, u + 0.5, v + 0.5);
      result.points.push_back(frame == PointFrame::kWorld
                                  ? Eigen::Vector3d(camera.pose * in_camera)
                                  : in_camera);
      result.pixels.push_back(image.index(u, v));
    }
  }
  return result;
}

Points PointsFromDepth(const DepthImage& image, const CameraModel& camera,
                       PointFrame frame) {
  return PointsWithPixels(image, camera, frame).points;
}

}  // namespace cell
