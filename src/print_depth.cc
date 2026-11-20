// Takes one depth image of the cell, as the perception stage will: resets the
// cell, puts the arm at arm_clear so that it is out of the picture, casts
// the image from the depth camera, and prints what it holds.
//
//   bazel run //:print_depth
//   bazel run //:print_depth -- [--seed N] [--nominal] [--noise]
//                               [--last] [--pgm FILE] [--pgm8 FILE] [--points]
//
//   --seed N     the scene's seed (default 1)
//   --nominal    the pallet exactly where it was designed
//   --noise      switch on every effect of the noise model (src/depth_camera.h)
//   --last       carry modules 0 to 28 away first, leaving only module_29
//   --pgm FILE   write the image as a 16-bit PGM, each pixel the depth in
//                whole millimetres, 0 where there is no reading
//   --pgm8 FILE  write it as an 8-bit PGM for viewers that cannot read 16
//                bits: the nearest reading black, the farthest white, no
//                reading also white
//   --points     print the points as CSV, x,y,z,u,v: world metres, then the
//                pixel's column and row
//
// Without --pgm, --pgm8 or --points it prints summary statistics: how many
// pixels read something, the depth range, what the pixels see (from the
// truth, as only a simulation can), and how long the image took.
// bazel run starts the program in its runfiles directory, so give FILE as
// an absolute path.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <mujoco/mujoco.h>

#include "src/camera_model.h"
#include "src/cell.h"
#include "src/depth_camera.h"

namespace {

// A part's name for the statistics: module_<i> for modules, other bodies by
// their own name.
std::string BodyOfGeom(const mjModel& model, int geom) {
  if (geom < 0) return "(no reading)";
  const char* name = mj_id2name(&model, mjOBJ_BODY, model.geom_bodyid[geom]);
  if (name == nullptr) {
    const char* geom_name = mj_id2name(&model, mjOBJ_GEOM, geom);
    return geom_name != nullptr ? geom_name : "world";
  }
  return name;
}

bool WritePgm(const std::string& path, const cell::DepthImage& image,
              bool eight_bit) {
  std::FILE* file = std::fopen(path.c_str(), "wb");
  if (file == nullptr) return false;
  float nearest = 1e30f;
  float farthest = 0.0f;
  for (float z : image.depth) {
    if (!std::isfinite(z)) continue;
    nearest = std::min(nearest, z);
    farthest = std::max(farthest, z);
  }
  std::fprintf(file, "P5\n%d %d\n%d\n", image.width, image.height,
               eight_bit ? 255 : 65535);
  for (float z : image.depth) {
    if (eight_bit) {
      unsigned char grey = 255;
      if (std::isfinite(z) && farthest > nearest) {
        grey = static_cast<unsigned char>(
            std::lround(254.0 * (z - nearest) / (farthest - nearest)));
      }
      std::fputc(grey, file);
    } else {
      // PGM's 16-bit samples are big-endian.
      const long mm = std::isfinite(z) ? std::lround(z * 1000.0) : 0;
      const unsigned value =
          static_cast<unsigned>(std::clamp(mm, 0L, 65535L));
      std::fputc(static_cast<int>(value >> 8), file);
      std::fputc(static_cast<int>(value & 0xff), file);
    }
  }
  return std::fclose(file) == 0;
}

}  // namespace

int main(int argc, char** argv) {
  const char* usage =
      "usage: %s <cell.xml> [--seed N] [--nominal] [--noise] [--last] "
      "[--pgm FILE] [--pgm8 FILE] [--points]\n";
  if (argc < 2) {
    std::fprintf(stderr, usage, argv[0]);
    return 1;
  }
  std::uint32_t seed = 1;
  bool nominal = false;
  bool noise = false;
  bool last = false;
  bool points = false;
  std::string pgm;
  std::string pgm8;
  for (int i = 2; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--nominal") == 0) {
      nominal = true;
    } else if (std::strcmp(argv[i], "--noise") == 0) {
      noise = true;
    } else if (std::strcmp(argv[i], "--last") == 0) {
      last = true;
    } else if (std::strcmp(argv[i], "--points") == 0) {
      points = true;
    } else if (std::strcmp(argv[i], "--seed") == 0 && has_value) {
      seed = static_cast<std::uint32_t>(std::strtoul(argv[++i], nullptr, 10));
    } else if (std::strcmp(argv[i], "--pgm") == 0 && has_value) {
      pgm = argv[++i];
    } else if (std::strcmp(argv[i], "--pgm8") == 0 && has_value) {
      pgm8 = argv[++i];
    } else {
      std::fprintf(stderr, usage, argv[0]);
      return 1;
    }
  }

  std::string error;
  std::optional<cell::Cell> loaded = cell::Cell::Load(argv[1], &error);
  if (!loaded) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  cell::Cell& c = *loaded;
  c.Reset(seed, nominal ? cell::Randomization::None() : cell::Randomization());
  if (last) {
    for (int i = 0; i + 1 < c.module_count(); ++i) c.ParkModule(i);
  }
  c.PutArm(c.arm_clear());  // also runs mj_forward

  const cell::CameraModel camera = c.DepthCamera();
  cell::DepthOptions options;
  if (noise) {
    options.noise.depth_noise = true;
    options.noise.edge_effects = true;
    options.noise.quantize = true;
    options.noise.seed = seed;
  }
  std::vector<int> geoms;
  const auto start = std::chrono::steady_clock::now();
  const cell::DepthImage image =
      cell::RenderDepth(*c.model(), c.data(), camera, options, &geoms);
  const double seconds = std::chrono::duration<double>(
                             std::chrono::steady_clock::now() - start)
                             .count();

  bool wrote = false;
  for (const auto& [path, eight_bit] :
       {std::pair{pgm, false}, std::pair{pgm8, true}}) {
    if (path.empty()) continue;
    if (!WritePgm(path, image, eight_bit)) {
      std::fprintf(stderr, "could not write %s\n", path.c_str());
      return 1;
    }
    std::fprintf(stderr, "wrote %s\n", path.c_str());
    wrote = true;
  }

  if (points) {
    const cell::PixelPoints cloud = cell::PointsWithPixels(image, camera);
    std::printf("x,y,z,u,v\n");
    for (std::size_t k = 0; k < cloud.points.size(); ++k) {
      const Eigen::Vector3d& p = cloud.points[k];
      std::printf("%.5f,%.5f,%.5f,%d,%d\n", p.x(), p.y(), p.z(),
                  cloud.pixels[k] % image.width, cloud.pixels[k] / image.width);
    }
    return 0;
  }
  if (wrote) return 0;

  float nearest = 1e30f;
  float farthest = 0.0f;
  for (float z : image.depth) {
    if (!std::isfinite(z)) continue;
    nearest = std::min(nearest, z);
    farthest = std::max(farthest, z);
  }
  std::printf("depth image %d x %d, seed %u%s%s%s\n", image.width,
              image.height, seed, nominal ? ", nominal" : "",
              noise ? ", noise on" : "", last ? ", only module_29 left" : "");
  std::printf("  cast in %.1f ms\n", seconds * 1e3);
  std::printf("  %d of %d pixels read a depth, from %.3f to %.3f m\n",
              image.CountReadings(), image.size(), nearest, farthest);

  // What the pixels see, from the truth.
  std::map<std::string, std::pair<int, std::pair<float, float>>> parts;
  for (int k = 0; k < image.size(); ++k) {
    std::string name = BodyOfGeom(*c.model(), geoms[k]);
    if (name.rfind("module_", 0) == 0 && name != "module_0" &&
        name != "module_29") {
      name = "module_* (others)";
    }
    auto [it, fresh] = parts.try_emplace(name, 0, std::pair{1e30f, 0.0f});
    ++it->second.first;
    if (geoms[k] >= 0) {
      it->second.second.first = std::min(it->second.second.first, image.depth[k]);
      it->second.second.second =
          std::max(it->second.second.second, image.depth[k]);
    }
  }
  std::printf("  what the pixels see (truth), pixels and depth range:\n");
  for (const auto& [name, entry] : parts) {
    if (name == "(no reading)") {
      std::printf("    %-20s %7d\n", name.c_str(), entry.first);
    } else {
      std::printf("    %-20s %7d   %.3f to %.3f m\n", name.c_str(),
                  entry.first, entry.second.first, entry.second.second);
    }
  }
  return 0;
}
