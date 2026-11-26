// The "ways" rows of a layout, with the infeed and outfeed units measured:
// the tube in, the bay out, each move from stage to stage, and the tube at
// rest at each stage. The rules are the opening comment of
// src/layout_check.h ("Stages"); this header holds what makes the rows.
//
// The units. An item that carries the tube along a way (LayoutScene::
// WayCarriers: the infeed unit for the tube in, the outfeed unit for the bay
// out) used to be left out of the way's obstacles, and a transfer left out
// the carriers of both ways. Now every item is measured, the moving tube
// with the rails and modules on it against all its boxes, at clearance_m.
// The exception is only what the unit does to the tube by design: a box of a
// carrier of the way that lies within clearance_m of the tube in its place or
// at an end of the path (the bed under it, a jaw that closes on it, a jaw it
// slides through open) is near by design against the tube alone, and is
// counted only below zero (touching or in it), never as a distance. A box of
// a carrier that is not near the tube at either end, a box of any other
// item, and every pair of a rail or a module with a unit, are held to
// clearance_m. In a transfer the carriers of both ways are the by-design
// units (the infeed unit holds the neck, the outfeed unit carries the built
// part). The row's detail names each by-design box and how near it is.
//
// The tube at rest (TubeAtRestRow), at each stage of a staged layout: the
// tube with the rails and modules on it at the stage's end (those at its
// start are among them), at the stage's place, against every box of every
// item, limit clearance_m. By design near the tube, counted only below
// zero, are the grips of the unit that pins the tube's angle at that stage
// (the stage's angle_pin_by item: its boxes named "jaw..." and "head"),
// when within clearance_m of the tube: the jaws that grip it, and at stage 1
// the stage-2 jaws 2 mm beside it, open for the tube to pass through. All
// other boxes, the unit's bed too, are held to clearance_m.

#ifndef SRC_LAYOUT_STAGE_ROWS_H_
#define SRC_LAYOUT_STAGE_ROWS_H_

#include <string>
#include <vector>


#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_scene.h"

namespace cell::layout {

// "tube in" or "bay out": the way's moving set (WayMoving) swept along
// `path` against what is there; with `units`, the way's carriers too, as
// above; without it (a layout with no staging) as the scene's WayObstacles
// has it, the carriers left out.
CheckRow WayRow(const LayoutScene& scene, mjData* d, const std::string& item,
                WayKind way, const std::vector<Vec3>& path, bool units,
                double clearance_m, double step_m);

// "stage <name> to <next>": the tube and the bay as built at the stage's
// end (`end`), swept along +Y by stage.to_next_m.
CheckRow TransferRow(const LayoutScene& scene, mjData* d,
                     const StageScope& stage, const Built& end,
                     double clearance_m, double step_m);

// "stage <name>: tube at rest", for the stage's layout `stage_layout`
// (StageLayout, whose staging is the whole layout's).
CheckRow TubeAtRestRow(const LayoutScene& scene, mjData* d,
                       const Layout& stage_layout, const StageScope& stage,
                       const Built& end, double clearance_m, double step_m);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_STAGE_ROWS_H_
