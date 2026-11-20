// The placement study for the check after pickup in the real cell: where
// check_camera should stand on the floor, and the joint angles the arm
// should hold the module at above it (the check pose). Part of print_check
// (print_check --study); reads models/cell.xml, never writes it.
//
// The module at the check pose: flat, glass up, its centre straight above
// the camera's lens by the rig's 1.40 m plus half a thickness, its long side
// along the arm's circle as the swing carries it (the module's +x is
// (sin a, -cos a, 0) with the arm facing bearing a, joint 1 = -a). The
// camera: as on the rig (models/check_scene.xml), looking straight up, its
// rows turned 5 degrees from the module's long side.
//
// The search, over the bearing a (joint 1 = -a), the distance r of the
// module's centre from the base's axis and the underside's height h:
//   - inverse kinematics for the tool tip that holds the module there at
//     the design grip (2 mm gap), seeded from the scripted swing's straight
//     joint move at the same joint 1, so the answer is on the swing's branch;
//   - every joint at least `min_joint_margin` inside its range;
//   - the arm, tool and held module at least `margin` from everything (the
//     motion planner's planning margin), and `self_margin` among themselves,
//     by src/collision.h with module_0 held and the rest of the stack, the
//     pallet and the conveyor where they were built;
//   - the camera's view clear: rays from the lens to points just inside the
//     underside's edges must meet module_0 first, and rays to points beyond
//     its edges, out to the finder's tolerances, nothing below the module;
//     and the view's pyramid, from the lens to the underside grown by the
//     tolerances, must keep out of where the tube's rails and the modules
//     laid on it can be (x below -0.80 m, up to 0.9 m high), whichever slot
//     is at the place, and of the pallet;
//   - the camera at least 0.9 m from the base's axis.
// Ranked by the time it adds to the carry: the scripted swing (over_pallet
// to over the slot, one joint move) cut in two at the check pose, and the
// planner's carry (start and goal as its report gives them) the same, both
// as S-curve joint moves at the carrying speed, plus a dwell for the
// picture.
//
// Then, if the model given has a camera named check_camera (a scratch copy
// of the cell with the snippet added), the check pose is taken from where
// that camera is and verified in full: the camera's housing among the
// obstacles for the collision check of the check pose and of the joint
// moves into and out of it (and how near the housing every carry comes);
// and module_0 gripped at the check pose at a few offsets by the real
// suction weld, the cell's physics run so the arm sags under it, the
// camera's picture rendered with noise, FindHeldModule and the recovered
// grip offset against the truth.

#ifndef SRC_CHECK_PLACEMENT_H_
#define SRC_CHECK_PLACEMENT_H_

#include <string>

namespace cell {

struct PlacementStudyOptions {
  double margin = 0.045;       // m, to the world
  double self_margin = 0.025;  // m, among the arm's links and the module
  double min_joint_margin = 15.0;  // degrees
  // The pause at the check pose: the arm settling, one exposure, the fit.
  double dwell = 0.3;   // s
  double speed = 0.9;   // fraction of full speed, as the carry
  int verify_shots = 5;
};

// Prints the study to stdout; returns 0 on success, 1 if the model cannot
// be loaded or no check pose qualifies.
int RunPlacementStudy(const std::string& cell_path,
                      const PlacementStudyOptions& options = {});

}  // namespace cell

#endif  // SRC_CHECK_PLACEMENT_H_
