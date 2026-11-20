// Made-up worlds with known answers, for testing the planner in src/rrt.h
// before the real cell's collision check exists, and for showing what it
// does.
//
// The 2D worlds are a point in the square [0, 10] x [0, 10] among circles
// and boxes. They are easy to draw and to reason about: for most of them the
// shortest path is worked out by hand below, so the planner's and the
// shortener's lengths can be compared with it. Their sizes are chosen to be
// like the arm's joint space in radians: a few units across, obstacles a
// few tenths thick, so the planner's default options suit them.
//
// The 6D world is a box with the arm's joint ranges and a few forbidden
// regions shaped like the cell's obstacles in joint space; see ArmLikeWorld.
//
// Obstacles are open sets: a configuration exactly on an obstacle's
// boundary is free. That makes the hand-worked shortest paths, which run
// along boundaries and round corners, free themselves.
//
// Each world can also say whether a configuration is free of its obstacles
// shrunk by a margin. The planner checks an edge only at points spaced
// `edge_resolution` apart, so it promises that every point of the edge is
// within half that (measured as a sum of joint changes) of a point found
// free; a corner may be clipped by less. The tests check paths densely
// against obstacles shrunk by that half, which is the promise exactly.

#ifndef SRC_RRT_WORLDS_H_
#define SRC_RRT_WORLDS_H_

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "src/rrt.h"

namespace cell {

struct Circle {
  Eigen::Vector2d centre = Eigen::Vector2d::Zero();
  double radius = 1.0;
};

// An axis-aligned box between two opposite corners, in any number of
// dimensions.
struct Box {
  Eigen::VectorXd low;
  Eigen::VectorXd high;
};

struct ToyWorld {
  std::string name;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  Eigen::VectorXd start;
  Eigen::VectorXd goal;
  IsFree is_free;

  // Whether q is free of the obstacles shrunk by `margin` on every side
  // (each bounding line or plane moved inward by margin). With margin 0,
  // the same as is_free.
  std::function<bool(const Eigen::VectorXd& q, double margin)> free_by;

  // The length of the shortest free path from start to goal, unweighted.
  // Infinite if there is none; not a number if not worked out.
  double shortest = 0.0;

  // The obstacles of a 2D world, for drawing. Empty for the 6D world, whose
  // obstacles are not boxes.
  std::vector<Circle> circles;
  std::vector<Box> boxes;
};

// A 2D world with the given obstacles; its is_free is true inside the
// square and outside every obstacle.
ToyWorld MakeWorld2d(std::string name, std::vector<Circle> circles,
                     std::vector<Box> boxes);

// No obstacles. Start (1, 1), goal (9, 8): the shortest path is the
// straight line.
ToyWorld OpenWorld();

// One circle of radius 2 at the centre, exactly between start (1, 1) and
// goal (9, 9). The shortest path runs straight to the circle, round its
// arc and straight on, either side: two tangents and an arc.
ToyWorld CircleWorld();

// A wall across the square at x 4.5 to 5.5, with a gap `gap` wide whose
// lower edge is at y = 6. Start (1, 1) and goal (9, 1) are on either side,
// low down, so the path must climb to the gap: the shortest goes straight
// to the gap's lower near corner, along the gap's lower edge and straight
// down to the goal.
ToyWorld GapWorld(double gap = 0.5);

// The same wall, with the goal in the middle of the gap.
ToyWorld GoalInGapWorld(double gap = 0.5);

// The start inside a cup whose mouth faces away from the goal: walls left,
// right and between the start (5, 5) and the goal (5, 9), open at the
// bottom. Any tree from the start has to find its way out of the mouth and
// round; a single tree is pulled toward the goal and into the cup's top.
ToyWorld BugTrapWorld();

// The goal (8, 8) inside a closed square ring of boxes. There is no path,
// and the planner must say so by running out of iterations, not by hanging.
ToyWorld EnclosedGoalWorld();

// Six joints with the ranges of the cell's arm (models/arm.xml), and
// forbidden regions like the cell's obstacles, as the carried module would
// make them in joint space:
//
//   post      Joint 1 within 0.35 rad of the middle swings the module past
//             the camera post. It clears only lifted and flat: joint 2 in
//             [-1.9, -1.5], joint 3 in [0.2, 0.7] and |joint 5| < 0.6. That
//             window is about 0.5% of the joints' ranges across: a narrow
//             passage the path must thread.
//   stack     Joint 1 on the pallet side (-2.2 to -0.35) with joint 2 above
//             -0.9 reaches forward into the stack.
//   conveyor  Joint 1 on the conveyor side (0.35 to 2.2) with joint 2 plus
//             joint 3 above -0.5 holds the module too low over the tube; and
//             joint 5 above 1.5 there tips its edge into the rails.
//   floor     Joint 2 plus joint 3 below -3.0 anywhere folds the arm into
//             the floor.
//
// Start on the pallet side, goal on the conveyor side, with joints 4 to 6
// turned differently at each end; the straight line between them runs
// through the post. The shortest path is not known.
ToyWorld ArmLikeWorld();

// The 2D worlds above that have a path, for tests that run over all of
// them.
std::vector<ToyWorld> SolvableWorlds2d();

}  // namespace cell

#endif  // SRC_RRT_WORLDS_H_
