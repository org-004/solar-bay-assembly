// Margins of a committed plan that the verifier does not report, worked out
// from the plan's timed paths and the layout's tracks without planning or
// measuring anything (run 051):
//
//   - track ends: for each arm on a track, the least slack of its track
//     value to the track's lo and to its hi over the whole plan;
//   - carriage gaps: for each pair of arms whose tracks run on the same
//     line, the least distance between their carriages' positions along
//     that line over the plan's time.
//
// Two tracks run on the same line when they ride the same beam: BeamArm
// (src/layout.h) gives the same arm for both, that is, one names the other
// in shares_beam_with or both name the same arm. The position of a carriage
// on its line is the arm's base origin at s = 0 projected on the track's
// direction plus the track value s, so the distance between two carriages is
// the distance between those positions. Its contact distance is the sum of
// the two carriage boxes' half extents along the line, centre offsets
// included: below it the boxes overlap along the line.
//
// Both are sampled from the plan's player (src/layout_plan.h, PlanPlayer) on
// a grid of sample_s and at every segment's start and end, where an arm
// comes to rest. The torque demand of the module arm bay by bay, which
// //:torque_demand prints with --all-bays, is TorqueByBay below.

#ifndef SRC_LAYOUT_MARGINS_H_
#define SRC_LAYOUT_MARGINS_H_

#include <string>
#include <utility>
#include <vector>

#include "src/layout.h"
#include "src/layout_plan.h"
#include "src/layout_torque.h"

namespace cell::layout {

// One end of one arm's track.
struct TrackEnd {
  std::string arm;
  std::string end;         // "lo" or "hi".
  double limit = 0.0;      // The end's track value, m.
  double slack = 0.0;      // Least distance of the track value to it, m; negative past it.
  double value = 0.0;      // The track value where the slack is least, m.
  int step = 0;            // The arm's step at that instant.
  double t = 0.0;          // The first instant where it is least, s.
};

// One pair of carriages on the same line.
struct CarriageGap {
  std::string a, b;        // The arms, in the layout's order.
  double distance = 0.0;   // Least |position of a - position of b| along the line, m.
  double contact = 0.0;    // The distance at which the carriage boxes touch, m.
  int step_a = 0, step_b = 0;  // Each arm's step at that instant.
  double t = 0.0;          // The first instant where it is least, s.
};

struct TrackMargins {
  std::vector<TrackEnd> ends;     // Tightest first (by slack, then the layout's order).
  std::vector<CarriageGap> gaps;  // Tightest first (by distance - contact).
  int samples = 0;                // Instants looked at.
};

// Every pair of arms whose tracks ride the same beam, as indices into
// layout.arms, in the layout's order (i < j).
std::vector<std::pair<int, int>> ArmsOnOneLine(const Layout& layout);

// The track ends and carriage gaps of the plan the player plays. The player
// must be of `layout`'s scene. sample_s > 0.
TrackMargins TrackMarginsOf(const Layout& layout, const PlanPlayer& player,
                            double sample_s);

// The module arm's moves of one bay against its motors' limits.
struct BayTorque {
  int bay = 0;
  int moves = 0;
  int over_one = 0;
  int over_scale = 0;      // Over the layout's torque_limit_scale (1.0 without it).
  TorqueDemand worst;      // Of the bay's worst move.
  int worst_step = 0;
};

// One BayTorque for each of the plan's bays, from bay 0, for the moves of
// arm `arm`; `scale` is the fraction counted in over_scale. Returns false
// and sets *error when a segment cannot be evaluated.
bool TorqueByBay(const Plan& plan, ArmDynamics* dynamics,
                 const ArmMotion& motion, const std::string& arm, double scale,
                 std::vector<BayTorque>* bays, std::string* error);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_MARGINS_H_
