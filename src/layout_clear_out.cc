#include "src/layout_clear_out.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "src/layout_bays.h"
#include "src/layout_plan_stages.h"

namespace cell::layout {

ArmsPassing ArmsPassBay(const LayoutScene& scene, mjData* d, double out_t0, double span,
                        double until, double sample_s, double clearance_m,
                        const ArmInstantFn& at, bool first_only, double far) {
  ArmsPassing out;
  if (!(sample_s > 0.0) || !(span > 0.0)) return out;
  until = std::min(until, out_t0 + span);
  const Layout& L = scene.layout();
  const LayoutHandles& H = scene.handles();
  std::vector<int> bay(H.tube_geoms.begin(), H.tube_geoms.end());
  for (const auto& r : H.rail_geoms) bay.insert(bay.end(), r.begin(), r.end());
  for (const auto& m : H.module_geoms) bay.insert(bay.end(), m.begin(), m.end());
  std::vector<Obstacle> obstacles;
  obstacles.reserve(bay.size());
  for (int g : bay) obstacles.push_back({g, 0u, 0u});
  // The samples from the bay's start and those of the plan's clock (the
  // verifier's) while the bay is on its way.
  std::vector<double> times;
  for (int k = 0; out_t0 + k * sample_s < until; ++k) times.push_back(out_t0 + k * sample_s);
  for (long k = static_cast<long>(std::ceil(out_t0 / sample_s)); k * sample_s < until; ++k) {
    times.push_back(static_cast<double>(k) * sample_s);
  }
  std::sort(times.begin(), times.end());
  // The bay leaves from the last stage's place (src/layout_plan_stages.h).
  const Eigen::Vector3d from = StageOffset(L, StageCount(L) - 1);
  const mjModel& m = scene.model();
  std::map<std::tuple<int, std::string, std::string>, std::vector<MovingGeom>> sets;
  for (double t : times) {
    const Eigen::Vector3d off = WayOffsetAt(L.bay_out.path, WayFraction(out_t0, span, t)) + from;
    for (int i = 0; i < scene.ArmCount(); ++i) {
      const ArmInstant a = at(i, t);
      if (a.at_wait) continue;
      ArmPose p;
      for (int j = 0; j < 6; ++j) p.q[j] = a.config[j];
      p.track = a.config[6];
      scene.SetPose(d, i, p);
      mj_kinematics(&m, d);
      for (int g : bay) {
        for (int c = 0; c < 3; ++c) d->geom_xpos[3 * g + c] += off[c];
      }
      auto key = std::make_tuple(i, a.tool, a.held);
      auto it = sets.find(key);
      if (it == sets.end()) it = sets.emplace(key, scene.Moving(i, a.tool, a.held)).first;
      const Nearest n = scene.LeastDistance(d, it->second, obstacles,
                                            first_only ? clearance_m : std::max(far, clearance_m));
      ++out.samples;
      if (n.distance < out.least_m) {
        out.least_m = n.distance;
        out.least_t = t;
        out.least_arm = i;
        out.least = n;
      }
      if (n.distance < clearance_m && out.clear) {
        out.clear = false;
        out.t = t;
        out.arm = i;
        out.nearest = n;
      }
      if (!out.clear && first_only) break;
    }
    if (!out.clear && first_only) break;
  }
  scene.AllAtWait(d);
  mj_kinematics(&m, d);
  return out;
}

std::string ArmsPassingText(const LayoutScene& scene, const ArmsPassing& p) {
  if (p.clear || p.arm < 0) return "";
  char buf[512];
  std::snprintf(buf, sizeof buf, "arm %s at %.2f s: %s to %s at %.4f m from the bay going out",
                scene.arm(p.arm).use->name.c_str(), p.t,
                scene.GeomName(p.nearest.moving).c_str(),
                scene.GeomName(p.nearest.obstacle).c_str(), p.nearest.distance);
  return buf;
}

}  // namespace cell::layout
