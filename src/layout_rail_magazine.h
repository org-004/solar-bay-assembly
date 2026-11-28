// A rail magazine on the tool that sets the rails (run 056): the layout's
// opt-in field "rail_magazine" of a tool (src/layout.h, Tool, and
// src/layout.cc, ReadTool), and the counts the checker, the planner and the
// verifier take from it.
//
// Without the field every rail has a take of its own: the arm that sets the
// rails fetches each one from the stock "rails" and sets it, ten round trips
// a bay. With it, the tool carries a magazine that holds `holds` rails. At a
// take (the work rail_take, at the stock's pick point as before) the stock
// loads the magazine: rails k .. k + n - 1 for n = min(holds, rails left),
// at the work's process_s for each rail. The arm then sets those rails one
// after another, with no take between: after a set's retreat the magazine
// presents its next rail in the clamp, so the arm holds a rail again on its
// move to the next set. The bay's steps hold a take only for the first rail
// of each load (rail k with k % holds == 0); the checker's rows still solve
// every rail_take point, as they do for the module takes a bay does not use.
//
// The magazine's boxes join the tool's (before its last box) and its empty
// mass joins the tool's mass when the layout is read, so that every sweep of
// the tool sweeps it and every torque of the tool counts it. Its rails are
// counted on the moves that hold a rail: the held rail stands for a full
// magazine (MagazineRailsMass), the most it carries, on every rail move.
//
// The planner and the verifier count the rails left in each arm's
// magazine: a take fills it, a set uses one, and a set with none left is a
// fault (a plan that sets more rails than the magazine holds is refused).

#ifndef SRC_LAYOUT_RAIL_MAGAZINE_H_
#define SRC_LAYOUT_RAIL_MAGAZINE_H_

#include <string>

#include "src/layout.h"

namespace cell::layout {

// The rails one take loads: the holds of the rail_set work's tool's
// magazine, 1 when it has none.
int RailsPerTake(const Layout& layout);

// Whether rail k (0 .. kRailCount - 1) has a take among the bay's steps.
bool RailHasTake(const Layout& layout, int rail);

// The rails the take of rail k loads: min(RailsPerTake, rails from k on).
int RailsLoadedAt(const Layout& layout, int rail);

// The seconds the work of `kind` point `index` takes: the work's process_s,
// times RailsLoadedAt for a rail_take into a magazine.
double WorkSeconds(const Layout& layout, const Work& work, const std::string& kind,
                   int index);

// The mass of the rails a held rail stands for: RailsPerTake rails.
double MagazineRailsMass(const Layout& layout);

// For the body that mounts a held part on the tool (src/layout_model.cc):
// with a magazine and part "rail", an <inertial> of the other rails' mass,
// MagazineRailsMass less one rail, at the magazine's middle behind the
// plate (tool_length / 2 back from the working point), with no geoms (the
// magazine's boxes are the tool's); "" otherwise.
std::string MagazineRailsInertialXml(const Layout& layout, const std::string& part,
                                     double tool_length);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_RAIL_MAGAZINE_H_
