#include "src/rrt_worlds.h"

#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace cell {
namespace {

Eigen::VectorXd Vec(std::initializer_list<double> values) {
  Eigen::VectorXd v(values.size());
  int i = 0;
  for (double x : values) v[i++] = x;
  return v;
}

Box Box2(double x0, double y0, double x1, double y1) {
  return {Vec({x0, y0}), Vec({x1, y1})};
}

bool StrictlyInside(const Box& box, const Eigen::VectorXd& q, double margin) {
  return (q.array() > box.low.array() + margin).all() &&
         (q.array() < box.high.array() - margin).all();
}

void SetFree(ToyWorld& world,
             std::function<bool(const Eigen::VectorXd&, double)> free_by) {
  world.free_by = free_by;
  world.is_free = [free_by](const Eigen::VectorXd& q) {
    return free_by(q, 0.0);
  };
}

double Hypot(double x, double y) { return std::sqrt(x * x + y * y); }

// The wall of GapWorld and GoalInGapWorld.
std::vector<Box> Wall(double gap) {
  return {Box2(4.5, 0.0, 5.5, 6.0), Box2(4.5, 6.0 + gap, 5.5, 10.0)};
}

}  // namespace

ToyWorld MakeWorld2d(std::string name, std::vector<Circle> circles,
                     std::vector<Box> boxes) {
  ToyWorld world;
  world.name = std::move(name);
  world.lower = Vec({0.0, 0.0});
  world.upper = Vec({10.0, 10.0});
  world.circles = std::move(circles);
  world.boxes = std::move(boxes);
  SetFree(world, [lower = world.lower, upper = world.upper,
                  circles = world.circles,
                  boxes = world.boxes](const Eigen::VectorXd& q, double m) {
    if ((q.array() < lower.array()).any() || (q.array() > upper.array()).any())
      return false;
    for (const Circle& c : circles) {
      const double r = c.radius - m;
      if (r > 0.0 && (q.head<2>() - c.centre).squaredNorm() < r * r) {
        return false;
      }
    }
    for (const Box& b : boxes) {
      if (StrictlyInside(b, q, m)) return false;
    }
    return true;
  });
  return world;
}

ToyWorld OpenWorld() {
  ToyWorld world = MakeWorld2d("open", {}, {});
  world.start = Vec({1.0, 1.0});
  world.goal = Vec({9.0, 8.0});
  world.shortest = (world.goal - world.start).norm();
  return world;
}

ToyWorld CircleWorld() {
  const double r = 2.0;
  ToyWorld world =
      MakeWorld2d("circle", {{Eigen::Vector2d(5.0, 5.0), r}}, {});
  world.start = Vec({1.0, 1.0});
  world.goal = Vec({9.0, 9.0});
  // From a point at distance d from the centre, the tangent to the circle
  // is sqrt(d^2 - r^2) long and touches it at an angle acos(r / d) from the
  // line to the centre. Start and goal are opposite each other, half a
  // turn apart, so the arc between the two tangent points is
  // pi - 2 acos(r / d).
  const double d = Hypot(4.0, 4.0);
  const double tangent = std::sqrt(d * d - r * r);
  const double arc = r * (std::numbers::pi - 2.0 * std::acos(r / d));
  world.shortest = 2.0 * tangent + arc;  // about 12.03
  return world;
}

ToyWorld GapWorld(double gap) {
  ToyWorld world = MakeWorld2d("gap", {}, Wall(gap));
  world.start = Vec({1.0, 1.0});
  world.goal = Vec({9.0, 1.0});
  // (1, 1) to (4.5, 6), along the gap's lower edge to (5.5, 6), down to
  // (9, 1).
  world.shortest = 2.0 * Hypot(3.5, 5.0) + 1.0;  // about 13.21
  return world;
}

ToyWorld GoalInGapWorld(double gap) {
  ToyWorld world = MakeWorld2d("goal_in_gap", {}, Wall(gap));
  world.start = Vec({1.0, 1.0});
  world.goal = Vec({5.0, 6.0 + gap / 2.0});
  // (1, 1) to the gap's lower near corner (4.5, 6), then to the goal.
  world.shortest = Hypot(3.5, 5.0) + Hypot(0.5, gap / 2.0);
  return world;
}

ToyWorld BugTrapWorld() {
  ToyWorld world = MakeWorld2d("bug_trap", {},
                               {Box2(3.0, 3.0, 3.3, 7.0),    // left
                                Box2(6.7, 3.0, 7.0, 7.0),    // right
                                Box2(3.0, 6.7, 7.0, 7.0)});  // top
  world.start = Vec({5.0, 5.0});
  world.goal = Vec({5.0, 9.0});
  // Out of the mouth round the bottom of the left wall, (3.3, 3) and
  // (3, 3), up its outside to the top wall's corner (3, 7), and straight to
  // the goal. The right side is the mirror image, the same length.
  world.shortest = Hypot(1.7, 2.0) + 0.3 + 4.0 + Hypot(2.0, 2.0);  // ~9.75
  return world;
}

ToyWorld EnclosedGoalWorld() {
  ToyWorld world = MakeWorld2d("enclosed_goal", {},
                               {Box2(6.5, 6.5, 9.5, 7.2),    // bottom
                                Box2(6.5, 8.8, 9.5, 9.5),    // top
                                Box2(6.5, 6.5, 7.2, 9.5),    // left
                                Box2(8.8, 6.5, 9.5, 9.5)});  // right
  world.start = Vec({1.0, 1.0});
  world.goal = Vec({8.0, 8.0});
  world.shortest = std::numeric_limits<double>::infinity();
  return world;
}

ToyWorld ArmLikeWorld() {
  ToyWorld world;
  world.name = "arm_like";
  // models/arm.xml's joint ranges, rounded inward.
  world.lower = Vec({-3.22, -2.44, -2.09, -6.10, -2.18, -6.10});
  world.upper = Vec({3.22, -0.09, 2.93, 6.10, 2.18, 6.10});
  world.start = Vec({-1.5, -1.2, 0.0, 0.0, 1.0, 0.0});
  world.goal = Vec({1.5, -1.0, -0.5, 2.0, -1.0, 3.0});
  world.shortest = std::numeric_limits<double>::quiet_NaN();
  // Each region is a few inequalities. Shrinking it by m moves every one
  // of them inward by m: the region's own bounds tighten, and the post's
  // window, which is a hole in it, grows.
  SetFree(world, [lower = world.lower,
                  upper = world.upper](const Eigen::VectorXd& q, double m) {
    if ((q.array() < lower.array()).any() || (q.array() > upper.array()).any())
      return false;
    const double j1 = q[0], j2 = q[1], j3 = q[2], j5 = q[4];
    // floor
    if (j2 + j3 < -3.0 - m) return false;
    // post, except through the window
    if (std::abs(j1) < 0.35 - m) {
      const bool window = j2 > -1.9 - m && j2 < -1.5 + m && j3 > 0.2 - m &&
                          j3 < 0.7 + m && std::abs(j5) < 0.6 + m;
      if (!window) return false;
    }
    // stack
    if (j1 > -2.2 + m && j1 < -0.35 - m && j2 > -0.9 + m) return false;
    // conveyor
    if (j1 > 0.35 + m && j1 < 2.2 - m &&
        (j2 + j3 > -0.5 + m || j5 > 1.5 + m)) {
      return false;
    }
    return true;
  });
  return world;
}

std::vector<ToyWorld> SolvableWorlds2d() {
  return {OpenWorld(), CircleWorld(), GapWorld(), GoalInGapWorld(),
          BugTrapWorld()};
}

}  // namespace cell
