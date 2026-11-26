// The sweep of a moving set against what is there, the one function behind
// the "ways" rows of src/layout_check.h: the tube in, each move from stage to
// stage, the bay out, and the tube at rest at each stage
// (src/layout_stage_rows.h).
//
// SweepRow moves the set along a path, in steps of at most step_m, and gives
// the least distance of any moving geom to any obstacle (LayoutScene::
// LeastDistance), limit clearance_m. Two kinds of pair are near by design and
// count only when they go below zero, never as a distance: the holder's geoms
// the tube rests or slides in (SweepSpec::holder), and the by-design geoms
// of units that carry, guide or grip the tube (SweepSpec::by_design), each
// only against the tube's own geoms (SweepSpec::tube) and only when within
// clearance_m of them in their place or at an end of the path. Every other
// pair is held to clearance_m. The row's detail names the by-design pairs.
//
// Geoms that travel (SweepSpec::travels) move with the set by the sweep's
// offset and are not measured against it: they are the carriage that draws
// it. They are measured, moved, against every other obstacle at
// clearance_m. Used for the outfeed unit's head along "bay out", the reading
// of the layout's bay_out text (the head pins the tube's angle at the plain
// end, lifts the bay out of the saddle and takes it out); not for the tube in
// or a transfer, where the boxes stay put, the stricter reading.

#ifndef SRC_LAYOUT_SWEEP_H_
#define SRC_LAYOUT_SWEEP_H_

#include <set>
#include <string>
#include <vector>


#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_scene.h"

namespace cell::layout {

struct SweepSpec {
  std::string item;                // The row's item.
  std::vector<MovingGeom> moving;  // Groups as LayoutScene::WayMoving's.
  std::vector<Obstacle> obstacles;
  std::vector<Vec3> path;          // Offsets from the place; empty: at rest.
  // The tube slides in the holder's geoms near it at either end (counted
  // only below zero all along); else they are left out while the offset is
  // straight up (lifting out of a saddle).
  bool slide = false;
  std::set<int> holder;
  std::set<int> tube;       // The moving geoms that are the tube.
  std::set<int> by_design;  // Geoms of units that carry, guide or grip it.
  std::set<int> travels;    // Obstacle geoms that move with the set.
  double clearance_m = 0.0;
  double step_m = 0.025;
};

// The row (check "ways") of the sweep; `d` holds the model's poses and is
// left as it was.
CheckRow SweepRow(const LayoutScene& scene, mjData* d, const SweepSpec& spec);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_SWEEP_H_
