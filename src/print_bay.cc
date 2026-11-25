// Prints the bay's fastening points, as src/bay_hardware.h has them, or the
// bay's counts, size and mass and the figures of the locating move, or the
// locating move's capture table.
//
//   bazel run --config=opt //:print_bay
//   bazel run --config=opt //:print_bay -- --summary
//   bazel run --config=opt //:print_bay -- --capture
//
// Without an argument: CSV, one row for each fastening in the order of
// cell::bay::Fastenings(): its name without the _a / _b of its sites, its
// kind, the two parts it joins, point a in the tube's frame (metres), the
// way the tool travels, and the free radius the tool needs (metres).
// With --summary: key,value lines, every value computed from the header's
// constants: the counts, size and mass, then the locating move
// (docs/bay-hardware.md, "Locating a module before pinning"): the nose, the
// gate's figures, the worst hole errors and the least opening over what the
// gate accepts, the capture range, where a hole is once the nose is in, the
// order of the holes, the slides and the forces. Lengths in millimetres
// (_mm), angles in degrees (_deg), forces in newtons (_n).
// With --capture: CSV, the capture table: for three placements the gate
// accepts (3 mm straight across the tube at the yaw that makes hole 0
// worst, 3 mm straight along, 3 mm at 45 degrees, each at the yaw sign that
// leaves the least opening), one row per hole 0..3 at the hole tolerance
// that makes that hole worst: its error, the opening, whether the tip
// enters, where the hole is once the nose is fully in and whether that is
// inside the pin's play. The model, models/bay.xml, is not read.

#include <cmath>
#include <cstdio>
#include <cstring>

#include <Eigen/Core>

#include "src/bay_hardware.h"

namespace {

// A value printed with `decimals` decimals, without a minus sign on zero.
double NoNegativeZero(double value, int decimals) {
  double scale = 1.0;
  for (int i = 0; i < decimals; ++i) scale *= 10.0;
  return (value * scale > -0.5 && value * scale < 0.5) ? 0.0 : value;
}

void PrintFastenings() {
  std::printf("name,kind,part_a,part_b,x,y,z,ax,ay,az,free_radius\n");
  for (const cell::bay::Fastening& f : cell::bay::Fastenings()) {
    const Eigen::Vector3d a = f.pose_a.translation();
    std::printf("%s,%s,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f\n",
                f.name.c_str(), cell::bay::KindName(f.kind), f.part_a.c_str(),
                f.part_b.c_str(), NoNegativeZero(a.x(), 6),
                NoNegativeZero(a.y(), 6), NoNegativeZero(a.z(), 6),
                NoNegativeZero(f.tool_axis.x(), 6),
                NoNegativeZero(f.tool_axis.y(), 6),
                NoNegativeZero(f.tool_axis.z(), 6), f.free_radius);
  }
}

void PrintSummary() {
  using namespace cell::bay;
  std::printf("modules,%d\n", kModuleCount);
  std::printf("rails,%d\n", kRailCount);
  std::printf("fastenings_rail,%d\n", kRailFasteningCount);
  std::printf("fastenings_module,%d\n", kModuleFasteningCount);
  std::printf("fastenings,%d\n", kFasteningCount);
  std::printf("locating_holes,%d\n", kLocatorCount);
  std::printf("holds,%d\n", kHoldCount);
  std::printf("length_m,%.3f\n", kBayLength);
  std::printf("width_m,%.3f\n", kBayWidth);
  std::printf("height_m,%.3f\n", kBayHeight);
  std::printf("mass_kg,%.1f\n", kBayMass);
  std::printf("module_pitch_m,%.3f\n", kModulePitch);
  std::printf("module_gap_m,%.3f\n", kModuleGap);

  // The locating move.
  const double mm = 1000.0;
  const double deg = 180.0 / 3.14159265358979323846;
  const LocatingDims d;
  std::printf("nose_tip_mm,%.2f\n", kNoseTipDiameter * mm);
  std::printf("nose_diameter_mm,%.2f\n", kNoseDiameter * mm);
  std::printf("nose_taper_length_mm,%.2f\n", kNoseTaperLength * mm);
  std::printf("nose_half_angle_deg,%.2f\n", NoseHalfAngle(d) * deg);
  std::printf("nose_land_mm,%.2f\n", kNoseLandLength * mm);
  std::printf("nose_length_mm,%.2f\n", kNoseLength * mm);
  std::printf("nose_over_travel_mm,%.2f\n", kNoseOverTravel * mm);
  std::printf("nose_rise_above_flange_mm,%.2f\n", kNoseRise * mm);
  std::printf("free_above_flange_mm,%.2f\n", kFreeAboveFlange * mm);
  std::printf("nose_clearance_to_laminate_mm,%.2f\n",
              (kFreeAboveFlange - kNoseRise) * mm);
  std::printf("free_below_radius_mm,%.1f\n", kLocateFreeRadiusBelow * mm);
  std::printf("free_below_length_mm,%.1f\n", kLocateFreeLengthBelow * mm);
  std::printf("free_above_radius_mm,%.1f\n", kLocateFreeRadiusAbove * mm);
  std::printf("free_above_height_mm,%.1f\n", kLocateFreeHeightAbove * mm);
  // What bounds the free space: the hat's web inside, along the tube, and
  // the frame's upright part above the flange.
  std::printf("free_below_to_web_mm,%.1f\n",
              (kRailTopWidth / 2.0 - kRailSheet - kModuleGap / 2.0 -
               kHoleFromLongEdge) * mm);
  std::printf("free_above_to_upright_mm,%.1f\n",
              (kHoleFromLongEdge - kFrameUprightWidth) * mm);
  std::printf("gate_offset_mm,%.2f\n", kGateOffset * mm);
  std::printf("gate_yaw_deg,%.2f\n", kGateYaw * deg);
  std::printf("hole_tolerance_across_mm,%.2f\n", kHoleToleranceAcrossTube * mm);
  std::printf("hole_tolerance_along_mm,%.2f\n", kHoleToleranceAlongTube * mm);
  // The opening's formula: 7.05 - the distance of the two slots' centre
  // lines, whose half straight parts are 1.50 across and 2.95 along.
  const Eigen::Vector2d half_segments =
      HalfSegment(d.rail_slot_across, d.rail_slot_along) +
      HalfSegment(d.frame_slot_across, d.frame_slot_along);
  std::printf("slots_half_widths_mm,%.2f\n", HalfWidths(d) * mm);
  std::printf("slots_half_segment_across_mm,%.2f\n", half_segments.x() * mm);
  std::printf("slots_half_segment_along_mm,%.2f\n", half_segments.y() * mm);
  const GateWorst w = WorstOverGate();
  std::printf("worst_hole_error_across_mm,%.3f\n", w.error_across * mm);
  std::printf("worst_hole_error_along_mm,%.3f\n", w.error_along * mm);
  std::printf("least_opening_mm,%.3f\n", w.opening * mm);
  std::printf("least_opening_offset_direction_deg,%.1f\n",
              NoNegativeZero(w.direction_deg, 1));
  std::printf("least_opening_dx_mm,%.3f\n", NoNegativeZero(w.placement.dx * mm, 3));
  std::printf("least_opening_dy_mm,%.3f\n", NoNegativeZero(w.placement.dy * mm, 3));
  std::printf("least_opening_yaw_deg,%.3f\n", w.placement.yaw * deg);
  std::printf("least_opening_hole,%d\n", w.hole);
  std::printf("least_opening_hole_error_across_mm,%.3f\n", w.error.x() * mm);
  std::printf("least_opening_hole_error_along_mm,%.3f\n", w.error.y() * mm);
  std::printf("tip_needs_mm,%.3f\n", TipNeeds(d) * mm);
  std::printf("tip_enters,%s\n", w.opening >= TipNeeds(d) ? "yes" : "no");
  std::printf("tip_margin_mm,%.3f\n", (w.opening - TipNeeds(d)) * mm);
  // Aiming the tip at the rail's slot from below, and the loop its centre
  // makes round the inside of the slot when the flange stops it.
  const double aim_across = (kRailSlotAcrossTube - d.tip) / 2.0;
  const double aim_along = (kRailSlotAlongTube - d.tip) / 2.0;
  std::printf("tip_aim_across_mm,%.2f\n", aim_across * mm);
  std::printf("tip_aim_along_mm,%.2f\n", aim_along * mm);
  std::printf("tip_loop_mm,%.1f\n",
              (4.0 * (aim_along - aim_across) +
               2.0 * 3.14159265358979323846 * aim_across) * mm);
  const Eigen::Vector2d capture = CaptureLimit(d);
  std::printf("capture_hole_error_across_mm,%.3f\n", capture.x() * mm);
  std::printf("capture_hole_error_along_mm,%.3f\n", capture.y() * mm);
  std::printf("capture_corner_radius_mm,%.3f\n", GapFor(TipNeeds(d), d) * mm);
  std::printf("capture_offset_at_gate_yaw_mm,%.3f\n", CaptureOffsetAtYaw() * mm);
  std::printf("capture_yaw_at_gate_offset_deg,%.4f\n",
              CaptureYawAtOffset() * deg);
  std::printf("capture_covers_gate,%s\n", CapturesGate() ? "yes" : "no");
  // The capture takes the rails' slots at their nominal places. From the
  // tube's nominal frame the rail's own play would add along the tube.
  std::printf("rail_play_along_mm,%.3f\n", kRailPlayAlongTube * mm);
  std::printf("rail_play_yaw_deg,%.3f\n", kRailPlayYaw * deg);
  std::printf("rail_play_yaw_along_at_hole_mm,%.3f\n",
              kRailPlayYaw * kHolePairSpacing / 2.0 * mm);
  std::printf("rail_play_along_at_hole_mm,%.3f\n", RailPlayAlongAtHole() * mm);
  const double tube_frame = HoleErrorAlongFromTubeFrame(d);
  std::printf("hole_error_along_from_tube_frame_mm,%.3f\n", tube_frame * mm);
  std::printf("capture_along_exceeded_from_tube_frame,%s\n",
              tube_frame > CaptureLimit(d).y() ? "yes" : "no");
  const Eigen::Vector2d leaves = NoseLeaves(d);
  const Eigen::Vector2d play = PinPlay(d);
  std::printf("nose_in_hole_across_mm,%.3f\n", leaves.x() * mm);
  std::printf("nose_in_hole_along_mm,%.3f\n", leaves.y() * mm);
  std::printf("nose_in_corner_radius_mm,%.3f\n", GapFor(d.nose, d) * mm);
  std::printf("pin_play_across_mm,%.3f\n", play.x() * mm);
  std::printf("pin_play_along_mm,%.3f\n", play.y() * mm);
  std::printf("pin_play_corner_radius_mm,%.3f\n", GapFor(d.pin, d) * mm);
  std::printf("pin_play_margin_across_mm,%.3f\n", (play.x() - leaves.x()) * mm);
  std::printf("pin_play_margin_along_mm,%.3f\n", (play.y() - leaves.y()) * mm);
  std::printf("locate_order,%d %d %d %d\n", kLocateOrder[0], kLocateOrder[1],
              kLocateOrder[2], kLocateOrder[3]);
  std::printf("swage_order,%d %d %d %d\n", kSwageOrder[0], kSwageOrder[1],
              kSwageOrder[2], kSwageOrder[3]);
  const Eigen::Vector3d slide = WorstFirstSlide();
  std::printf("first_hole_slide_across_mm,%.3f\n", slide.x() * mm);
  std::printf("first_hole_slide_along_mm,%.3f\n", slide.y() * mm);
  std::printf("first_hole_slide_mm,%.3f\n", slide.z() * mm);
  // The turn's pivot: the middle of the other rail, from the module's centre.
  std::printf("turn_pivot_from_centre_mm,%.1f\n",
              (kModuleWidth / 2.0 + kModuleGap / 2.0) * mm);
  for (LocateMotion m : {LocateMotion::kShift, LocateMotion::kTurn}) {
    const LaterHoles l = LaterHolesAfterFirst(m);
    const char* n = MotionName(m);
    std::printf("later_hole_error_across_%s_mm,%.3f\n", n, l.error_across * mm);
    std::printf("later_hole_error_along_%s_mm,%.3f\n", n, l.error_along * mm);
    std::printf("later_hole_least_opening_%s_mm,%.3f\n", n, l.opening * mm);
    std::printf("later_holes_captured_%s,%s\n", n, l.captured ? "yes" : "no");
  }
  std::printf("friction,%.2f\n", d.friction);
  std::printf("module_weight_n,%.1f\n", ModuleWeight());
  std::printf("slide_force_n,%.1f\n", SlideForce(d));
  std::printf("turn_force_n,%.1f\n", TurnForce(d));
  std::printf("lift_per_push,%.3f\n", LiftPerPush(d));
  std::printf("nose_push_n,%.1f\n", NosePush(d));
  std::printf("nose_lift_n,%.1f\n", NoseLift(d));
  std::printf("half_module_weight_n,%.1f\n", ModuleWeight() / 2.0);
  std::printf("friction_lift_limit,%.3f\n", LiftLimitFriction(d));
  std::printf("friction_lift_margin,%.3f\n", LiftLimitFriction(d) - d.friction);
  std::printf("nose_thrust_n,%.1f\n", NoseThrust(d));
  std::printf("nose_tool_rated_thrust_n,%.1f\n", kNoseToolThrust);
}

// The three placements of the capture table, each with its rows.
void PrintCapture() {
  using namespace cell::bay;
  const double mm = 1000.0;
  const double deg = 180.0 / 3.14159265358979323846;
  const LocatingDims d;
  std::printf("placement,dx_mm,dy_mm,yaw_deg,hole,sign_across,sign_along,"
              "error_across_mm,error_along_mm,opening_mm,tip_enters,"
              "located_across_mm,located_along_mm,inside_pin_play\n");
  struct Case {
    const char* name;
    double direction;  // radians
  };
  const double pi = 3.14159265358979323846;
  const Case cases[] = {{"across", 0.0}, {"along", pi / 2.0}, {"diagonal", pi / 4.0}};
  for (const Case& c : cases) {
    const double dx = c.direction == pi / 2.0 ? 0.0 : kGateOffset * std::cos(c.direction);
    const double dy = c.direction == 0.0 ? 0.0 : kGateOffset * std::sin(c.direction);
    // The yaw sign that leaves the least opening at hole 0 (across) or at
    // any hole (the others); the tolerance worst for each hole.
    auto worst_signs = [&](const ModulePlacement& p, int hole, int* sa, int* sl) {
      double least = 1e9;
      for (int a : {1, -1}) {
        for (int l : {1, -1}) {
          const Eigen::Vector2d e = HoleError(p, hole, a, l, d);
          const double o = Opening(e.x(), e.y(), d);
          if (o < least - 1e-12) {
            least = o;
            *sa = a;
            *sl = l;
          }
        }
      }
      return least;
    };
    ModulePlacement best{dx, dy, kGateYaw};
    double best_opening = 1e9;
    for (int ys : {1, -1}) {
      const ModulePlacement p{dx, dy, ys * kGateYaw};
      double least = 1e9;
      for (int hole = 0; hole < 4; ++hole) {
        if (c.direction == 0.0 && hole != 0) continue;
        int sa = 1, sl = 1;
        least = std::min(least, worst_signs(p, hole, &sa, &sl));
      }
      if (least < best_opening - 1e-12) {
        best_opening = least;
        best = p;
      }
    }
    for (int hole = 0; hole < 4; ++hole) {
      int sa = 1, sl = 1;
      worst_signs(best, hole, &sa, &sl);
      const Eigen::Vector2d e = HoleError(best, hole, sa, sl, d);
      const double o = Opening(e.x(), e.y(), d);
      const Eigen::Vector2d at = LocatedHole(e.x(), e.y(), d);
      std::printf("%s,%.3f,%.3f,%.3f,%d,%+d,%+d,%.3f,%.3f,%.3f,%s,%.3f,%.3f,%s\n",
                  c.name, NoNegativeZero(best.dx * mm, 3),
                  NoNegativeZero(best.dy * mm, 3), best.yaw * deg, hole, sa, sl,
                  NoNegativeZero(e.x() * mm, 3), NoNegativeZero(e.y() * mm, 3),
                  o * mm, TipEnters(e.x(), e.y(), d) ? "yes" : "no",
                  NoNegativeZero(at.x() * mm, 3), NoNegativeZero(at.y() * mm, 3),
                  InsidePinPlay(at.x(), at.y(), d) ? "yes" : "no");
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 1) {
    PrintFastenings();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--summary") == 0) {
    PrintSummary();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--capture") == 0) {
    PrintCapture();
    return 0;
  }
  std::fprintf(stderr, "usage: %s [--summary | --capture]\n", argv[0]);
  return 1;
}
