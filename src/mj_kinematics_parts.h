// The two passes of MuJoCo's mj_kinematics that a planner needs apart.
//
// mj_kinematics (MuJoCo 4.0.0, src/engine/engine_core_smooth.c) places
// every body (mj_kinematics1), then every body's inertial frame, every
// geom and every site (mj_kinematics2, through mj_local2GlobalQuat). A move
// planner changes only one arm's joints between queries, so after the
// first query only that arm's frames need placing again. These are
// MuJoCo's own functions, not copies, so what they place is what
// mj_kinematics places, to the bit. MuJoCo builds them with hidden
// visibility and leaves them out of its public header: the library is
// linked statically into every program and test, where they can be
// reached, and hands them on under these names.

#ifndef SRC_MJ_KINEMATICS_PARTS_H_
#define SRC_MJ_KINEMATICS_PARTS_H_

#include <mujoco/mujoco.h>

namespace cell {

// mj_kinematics1: every body's xpos, xquat and xmat, and every joint's
// anchor and axis.
void KinematicsBodies(const mjModel& m, mjData* d);

// mj_kinematics2's step for one frame: the frame at `pos`, `quat` in body
// `body`'s frame, into world coordinates.
void LocalToGlobal(mjData* d, mjtNum* xpos, mjtNum* xquat, mjtNum* xmat,
                   const mjtNum* pos, const mjtNum* quat, int body,
                   mjtByte sameframe);

}  // namespace cell

#endif  // SRC_MJ_KINEMATICS_PARTS_H_
