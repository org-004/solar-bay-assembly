// See src/mj_kinematics_parts.h.

#include "src/mj_kinematics_parts.h"

// MuJoCo 4.0.0: src/engine/engine_core_smooth.h and engine_core_util.h.
extern "C" {
void mj_kinematics1(const mjModel* m, mjData* d);
void mj_local2GlobalQuat(mjData* d, mjtNum* xpos, mjtNum* xquat, mjtNum* xmat,
                         const mjtNum pos[3], const mjtNum quat[4], int body,
                         mjtByte sameframe);
}

namespace cell {

void KinematicsBodies(const mjModel& m, mjData* d) { mj_kinematics1(&m, d); }

void LocalToGlobal(mjData* d, mjtNum* xpos, mjtNum* xquat, mjtNum* xmat,
                   const mjtNum* pos, const mjtNum* quat, int body,
                   mjtByte sameframe) {
  mj_local2GlobalQuat(d, xpos, xquat, xmat, pos, quat, body, sameframe);
}

}  // namespace cell
