// A joint move: every joint goes from its start angle to its goal angle, and
// all of them start and finish together.
//
// One Profile runs from 0 to 1 and says what fraction of the move is done at
// each instant. Every joint follows that same fraction:
//
//   q(t) = start + (goal - start) * fraction(t)
//
// so the arm traces a straight line in joint space. The profile's limits are
// chosen so that no joint exceeds its own: the joint that would take longest
// sets the pace, and the others move slower than they could.

#ifndef SRC_JOINT_MOVE_H_
#define SRC_JOINT_MOVE_H_

#include <optional>

#include <Eigen/Core>

#include "src/profile.h"

namespace cell {

// Largest allowed magnitudes, one entry per joint. All must be positive and
// finite.
struct JointLimits {
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;  // Not used by the trapezoid; may be left empty.
};

enum class ProfileShape { kTrapezoid, kSCurve };

// The state of every joint at one instant.
struct JointSample {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd acceleration;
  Eigen::VectorXd jerk;
};

class JointMove {
 public:
  // Shortest synchronized rest-to-rest move from `start` to `goal`. Returns
  // nothing if the vectors differ in length, or hold a value that is not
  // finite, or a limit is not positive.
  static std::optional<JointMove> Plan(const Eigen::VectorXd& start,
                                       const Eigen::VectorXd& goal,
                                       const JointLimits& limits,
                                       ProfileShape shape);

  // How long the move takes. Zero if start and goal are the same.
  double duration() const { return fraction_.duration(); }

  // The state at time `t`. Before 0 it is `start` at rest; after duration()
  // it is `goal` at rest.
  JointSample At(double t) const;

 private:
  JointMove(const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
            const Profile& fraction)
      : start_(start), goal_(goal), delta_(goal - start), fraction_(fraction) {}

  Eigen::VectorXd start_;
  Eigen::VectorXd goal_;
  Eigen::VectorXd delta_;  // goal - start
  Profile fraction_;       // 0 at the start of the move, 1 at the end
};

}  // namespace cell

#endif  // SRC_JOINT_MOVE_H_
