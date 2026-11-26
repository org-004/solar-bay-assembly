// The sweep of a moving set against what is there (src/layout_sweep.h).

#include "src/layout_sweep.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include <Eigen/Core>

namespace cell::layout {
namespace {

using Vec = Eigen::Vector3d;

// The tube's geoms, in addition to their group: the by-design geoms skip and
// are near this alone (not the rails and modules on the tube).
constexpr unsigned kTubeOnly = 64;

std::string Fmt(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  std::vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  return buffer;
}

Vec V(const Vec3& a) { return {a[0], a[1], a[2]}; }

}  // namespace

CheckRow SweepRow(const LayoutScene& scene, mjData* d, const SweepSpec& spec) {
  std::vector<MovingGeom> moving = spec.moving;
  std::vector<int> geoms;
  std::vector<MovingGeom> tube;
  for (MovingGeom& g : moving) {
    geoms.push_back(g.geom);
    if (spec.tube.count(g.geom)) {
      g.group |= kTubeOnly;
      tube.push_back(g);
    }
  }
  std::vector<Vec> orig;
  for (int g : geoms) {
    orig.push_back(Vec(d->geom_xpos[3 * g], d->geom_xpos[3 * g + 1],
                       d->geom_xpos[3 * g + 2]));
  }
  auto place = [&](const Vec& o) {
    for (size_t i = 0; i < geoms.size(); ++i) {
      for (int c = 0; c < 3; ++c) {
        d->geom_xpos[3 * geoms[i] + c] = orig[i][c] + o[c];
      }
    }
  };
  // The offsets where a geom counts as near the tube or the holder: in
  // place, and at the path's ends.
  std::vector<Vec> ends = {Vec::Zero()};
  if (!spec.path.empty()) {
    ends.push_back(V(spec.path.front()));
    ends.push_back(V(spec.path.back()));
  }
  // The holder's geoms the moving set rests in (with `slide`, those the
  // tube rests in at either end of the path).
  std::vector<MovingGeom> slider;
  for (const MovingGeom& g : moving) {
    if (!spec.slide || g.group == kHeld || (g.group & kTubeOnly)) {
      slider.push_back(g);
    }
  }
  std::vector<Obstacle> all = spec.obstacles;
  std::vector<Obstacle> rest = spec.obstacles;
  struct Design {
    int geom;
    double distance;
  };
  std::vector<Design> designs;
  for (size_t j = 0; j < all.size(); ++j) {
    const bool holder = spec.holder.count(all[j].geom) > 0;
    const bool unit = spec.by_design.count(all[j].geom) > 0;
    if (!holder && !unit) continue;
    const std::vector<Obstacle> one = {all[j]};
    const std::vector<MovingGeom>& against = unit ? tube : slider;
    double least = 1.0;
    bool near = false;
    for (size_t e = 0; e < ends.size(); ++e) {
      // The holder lifts out of at the first offset only, unless it slides.
      if (holder && !spec.slide && e > 0) break;
      place(ends[e]);
      const double dist = scene.LeastDistance(d, against, one).distance;
      least = std::min(least, dist);
      near = near || dist < spec.clearance_m;
    }
    place(Vec::Zero());
    if (!near) continue;
    if (unit) {
      // Near by design, at every offset: only going into it counts.
      all[j].skip |= kTubeOnly;
      all[j].near |= kTubeOnly;
      rest[j].skip |= kTubeOnly;
      rest[j].near |= kTubeOnly;
      designs.push_back({all[j].geom, least});
    } else {
      rest[j].skip |= kHeld;
      // Along a transfer the tube slides in them: near by design, so they
      // count against it only below zero, never not at all.
      if (spec.slide) rest[j].near |= kHeld;
    }
  }
  // The samples.
  std::vector<Vec> samples;
  if (spec.path.empty()) samples.push_back(Vec::Zero());
  for (size_t i = 0; i < spec.path.size(); ++i) {
    if (i == 0) {
      samples.push_back(V(spec.path[0]));
      continue;
    }
    const Vec a = V(spec.path[i - 1]), b = V(spec.path[i]);
    const int n = std::max(
        1, static_cast<int>(std::ceil((b - a).norm() / spec.step_m)));
    for (int k = 1; k <= n; ++k) samples.push_back(a + (b - a) * k / n);
  }
  Nearest worst;
  worst.distance = kSceneFar + 1.0;
  Vec worst_at = Vec::Zero();
  for (const Vec& o : samples) {
    place(o);
    const bool above =
        spec.slide || (std::hypot(o.x(), o.y()) < 1e-9 && o.z() >= -1e-9);
    const Nearest n = scene.LeastDistance(d, moving, above ? rest : all);
    if (n.distance < worst.distance) {
      worst = n;
      worst_at = o;
    }
  }
  place(Vec::Zero());
  if (geoms.empty()) worst.distance = 0.0;
  std::string detail =
      geoms.empty()
          ? "the model has no tube"
          : Fmt("%s at offset (%.3f, %.3f, %.3f); %d samples, at most "
                "%.3f m apart",
                scene.PairText(worst).c_str(), worst_at.x(), worst_at.y(),
                worst_at.z(), static_cast<int>(samples.size()), spec.step_m);
  if (!designs.empty()) {
    detail += "; near the tube by design, counted only below zero:";
    for (const Design& g : designs) {
      detail += Fmt(" %s (%.4f m)", scene.GeomName(g.geom).c_str(), g.distance);
    }
  }
  const double figure = std::min(worst.distance, kSceneFar);
  return {"ways",
          spec.item,
          "",
          "",
          figure,
          "m",
          spec.clearance_m,
          !geoms.empty() && worst.distance >= spec.clearance_m,
          detail};
}

}  // namespace cell::layout
