#include "src/check_placement.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <map>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mujoco/mujoco.h>

#include "src/arm.h"
#include "src/camera_model.h"
#include "src/check_rig.h"
#include "src/collision.h"
#include "src/depth_camera.h"
#include "src/held_module_finder.h"
#include "src/ik.h"
#include "src/joint_move.h"
#include "src/mj_handles.h"
#include "src/motion_planner.h"
#include "src/pallet.h"
#include "src/path_timing.h"
#include "src/rrt.h"
#include "src/suction.h"

namespace cell {
namespace {

using Eigen::Isometry3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kPi = std::numbers::pi;
constexpr double kDegree = kPi / 180.0;

// As on the rig (models/check_scene.xml): the underside 1.40 m above the
// lens, the camera's rows turned 5 degrees from the module's long side.
constexpr double kCameraDistance = 1.40;
constexpr double kCameraTurn = 5.0 * kDegree;
// The pick's grip gap (PickPlaceOptions::grip_gap): the glass this far in
// front of the tool tip.
constexpr double kGripGap = 0.002;
// How far the finder's tolerances move a corner of the underside
// (HeldFinderOptions: 5 cm plus 3 degrees at the corner).
constexpr double kReach = 0.05 + 3.0 * kDegree * 1.2741;

// Where the rails and the modules laid on the tube can be: x below this, up
// to kConveyorTop high (the modules lie across the tube, x from -3.15 to
// -0.85 m, their glass at 0.835 m; the rails end at x = -1.0).
constexpr double kConveyorEdge = -0.80;
constexpr double kConveyorTop = 0.90;
// The pallet's stand, deck and stack, wherever it is set down: x beyond
// this, |y| within kPalletHalfWidth, up to kPalletTop.
constexpr double kPalletEdge = 1.60;
constexpr double kPalletHalfWidth = 1.45;
constexpr double kPalletTop = 1.35;

VectorXd Degrees(double a, double b, double c, double d, double e, double f) {
  VectorXd q(6);
  q << a, b, c, d, e, f;
  return q * kDegree;
}

// The planner's carry, start and goal as its report gives them: the end of
// the scripted clear (module standing, backed off the stack) and over the
// slot with the long side along the tube.
const VectorXd kPlannerStart = Degrees(0, -107, 144, 0, -27, 90);
const VectorXd kPlannerGoal = Degrees(-180, -46, 53, 0, 84, 0);
// Seeds for the scripted swing's ends (src/pick_place.cc's own).
const VectorXd kOverPalletSeed = Degrees(0, -90.6, 80.9, 0, 99.8, 90);
const VectorXd kPlaceSeed = Degrees(-180, -65, 85, 0, 70, 0);

Isometry3d Frame(const Vector3d& x, const Vector3d& z, const Vector3d& at) {
  Isometry3d pose = Isometry3d::Identity();
  pose.linear().col(0) = x.normalized();
  pose.linear().col(2) = z.normalized();
  pose.linear().col(1) = pose.linear().col(2).cross(pose.linear().col(0));
  pose.translation() = at;
  return pose;
}

// The module flat, glass up, centre at bearing a and distance r from the
// base's axis, its underside at height h, its +x along the arm's circle as
// the swing carries it.
Isometry3d ModuleAt(double a, double r, double h) {
  return Frame(Vector3d(std::sin(a), -std::cos(a), 0.0), Vector3d::UnitZ(),
               Vector3d(r * std::cos(a), r * std::sin(a),
                        h + 0.5 * kModuleThickness));
}

Isometry3d DesignGrip() { return DesignModuleInTip(kModuleThickness, kGripGap); }

Isometry3d TipFor(const Isometry3d& module) {
  return CorrectedTipTarget(module, DesignGrip());
}

// The camera under `module`: looking straight up, its +X (the image's rows)
// the module's +x turned kCameraTurn about the vertical.
Isometry3d CameraFor(const Isometry3d& module) {
  const Vector3d x = Eigen::AngleAxisd(kCameraTurn, Vector3d::UnitZ()) *
                     module.linear().col(0);
  return Frame(x, -Vector3d::UnitZ(),
               module.translation() -
                   (0.5 * kModuleThickness + kCameraDistance) *
                       Vector3d::UnitZ());
}

// And back: the module the camera at `camera` is placed for.
Isometry3d ModuleFor(const Isometry3d& camera) {
  const Vector3d x = Eigen::AngleAxisd(-kCameraTurn, Vector3d::UnitZ()) *
                     camera.linear().col(0);
  Vector3d level = x;
  level.z() = 0.0;
  return Frame(level, Vector3d::UnitZ(),
               camera.translation() +
                   (0.5 * kModuleThickness + kCameraDistance) *
                       Vector3d::UnitZ());
}

JointLimits Scaled(const JointLimits& limits, double speed) {
  JointLimits scaled = limits;
  scaled.velocity *= speed;
  scaled.acceleration *= speed * speed;
  scaled.jerk *= speed * speed * speed;
  return scaled;
}

double MoveTime(const VectorXd& a, const VectorXd& b,
                const JointLimits& limits) {
  const std::optional<JointMove> move =
      JointMove::Plan(a, b, limits, ProfileShape::kSCurve);
  return move ? move->duration() : 1e9;
}

double JointMargin(const Arm& arm, const VectorXd& q, int* joint) {
  double worst = 1e9;
  for (int i = 0; i < Arm::kJoints; ++i) {
    const double m = std::min(q[i] - arm.lower[i], arm.upper[i] - q[i]);
    if (m < worst) {
      worst = m;
      *joint = i;
    }
  }
  return worst;
}

std::string Join(const VectorXd& q, double scale, const char* format) {
  std::string out;
  char buffer[64];
  for (int i = 0; i < q.size(); ++i) {
    std::snprintf(buffer, sizeof(buffer), format, q[i] * scale);
    out += (i ? ", " : "") + std::string(buffer);
  }
  return out;
}

// The four corners of the underside grown by `grow`, in the world.
std::array<Vector3d, 4> UndersideCorners(const Isometry3d& module,
                                         double grow) {
  const double hx = 0.5 * kModuleLength + grow;
  const double hy = 0.5 * kModuleWidth + grow;
  const double z = -0.5 * kModuleThickness;
  return {module * Vector3d(-hx, -hy, z), module * Vector3d(hx, -hy, z),
          module * Vector3d(hx, hy, z), module * Vector3d(-hx, hy, z)};
}

// Whether the view's pyramid, from the lens to the underside grown by the
// finder's reach, keeps out of the conveyor's and the pallet's spaces.
bool PyramidClear(const Isometry3d& module, const Vector3d& lens,
                  std::string* why) {
  const std::array<Vector3d, 4> corners = UndersideCorners(module, kReach);
  const double top = corners[0].z();
  for (double z = lens.z(); z <= top; z += 0.02) {
    const double s = (z - lens.z()) / (top - lens.z());
    double x_low = 1e9, x_high = -1e9, y_low = 1e9, y_high = -1e9;
    for (const Vector3d& c : corners) {
      const Vector3d p = lens + s * (c - lens);
      x_low = std::min(x_low, p.x());
      x_high = std::max(x_high, p.x());
      y_low = std::min(y_low, p.y());
      y_high = std::max(y_high, p.y());
    }
    if (z <= kConveyorTop && x_low < kConveyorEdge) {
      *why = "view over the tube's rails or the modules on it";
      return false;
    }
    if (z <= kPalletTop && x_high > kPalletEdge &&
        y_low < kPalletHalfWidth && y_high > -kPalletHalfWidth) {
      *why = "view crosses the pallet";
      return false;
    }
  }
  return true;
}

// Rays from the lens: to points 2 cm inside the underside's edges they must
// meet `module_geom` first, there; to points beyond its edges, out to the
// finder's reach, nothing short of the underside's plane. `data` is posed
// with the module held.
bool RaysClear(const mjModel& model, const mjData& data, int module_geom,
               const Isometry3d& module, const Vector3d& lens,
               std::string* why) {
  const mjtByte groups[mjNGROUP] = {1, 1, 1, 0, 0, 0};
  const double z = -0.5 * kModuleThickness;
  auto cast = [&](const Vector3d& p, int* geom) {
    const Vector3d d = p - lens;
    mjtNum from[3] = {lens.x(), lens.y(), lens.z()};
    mjtNum vec[3] = {d.x(), d.y(), d.z()};
    return mj_ray(&model, &data, from, vec, groups, 1, -1, geom, nullptr);
  };
  for (double s : {-1.0, -0.5, 0.0, 0.5, 1.0}) {
    for (double t : {-1.0, 0.0, 1.0}) {
      const Vector3d p =
          module * Vector3d(s * (0.5 * kModuleLength - 0.02),
                            t * (0.5 * kModuleWidth - 0.02), z);
      int geom = -1;
      const double x = cast(p, &geom);
      if (geom != module_geom || std::abs(x - 1.0) > 0.01) {
        *why = std::string("the underside is hidden by ") +
               (geom >= 0 ? mj_id2name(&model, mjOBJ_GEOM, geom) : "nothing");
        return false;
      }
    }
  }
  for (double s : {-1.0, 0.0, 1.0}) {
    for (double t : {-1.0, 0.0, 1.0}) {
      if (s == 0.0 && t == 0.0) continue;
      const Vector3d p =
          module * Vector3d(s * (0.5 * kModuleLength + kReach),
                            t * (0.5 * kModuleWidth + kReach), z);
      int geom = -1;
      const double x = cast(p, &geom);
      if (geom >= 0 && x < 0.99) {
        *why = std::string("beyond the edges the view is cut by ") +
               mj_id2name(&model, mjOBJ_GEOM, geom);
        return false;
      }
    }
  }
  return true;
}

struct Candidate {
  double bearing = 0.0, r = 0.0, h = 0.0;
  Isometry3d module = Isometry3d::Identity();
  Isometry3d tip = Isometry3d::Identity();
  Isometry3d camera = Isometry3d::Identity();
  VectorXd q;
  double clearance = 0.0, self_clearance = 0.0;
  std::string clearance_between, self_between;
  double joint_margin = 0.0;
  int tight_joint = -1;
  double scripted_detour = 0.0, planner_detour = 0.0;
};

// Everything the study works with.
struct Setup {
  ModelPtr model;
  DataPtr data;
  DataPtr scratch;
  Arm arm;
  std::optional<CollisionChecker> checker;
  int module_body = -1;
  int module_geom = -1;
  VectorXd over_pallet, over_slot;  // the scripted swing's ends
  JointLimits limits;
};

// The candidate for the module held at `module` (flat, glass up), or
// nothing with the reason.
std::optional<Candidate> Evaluate(Setup& s, const Isometry3d& module,
                                  const PlacementStudyOptions& options,
                                  std::string* why) {
  Candidate c;
  c.module = module;
  const double a = std::atan2(module.translation().y(), module.translation().x());
  c.bearing = a;
  c.r = module.translation().head<2>().norm();
  c.h = module.translation().z() - 0.5 * kModuleThickness;
  c.tip = TipFor(c.module);
  c.camera = CameraFor(c.module);
  const Vector3d lens = c.camera.translation();
  if (lens.head<2>().norm() < 0.9) {
    *why = "camera within 0.9 m of the base's axis";
    return std::nullopt;
  }
  if (!PyramidClear(c.module, lens, why)) return std::nullopt;

  // Seeded from the scripted swing at the same joint 1.
  const double lambda = std::clamp(a / kPi, 0.0, 1.0);
  const VectorXd seed =
      s.over_pallet + lambda * (s.over_slot - s.over_pallet);
  IkReport report;
  const std::optional<VectorXd> q =
      SolveIk(*s.model, s.scratch.get(), s.arm, c.tip, seed, {}, &report);
  if (!q) {
    *why = "no IK solution";
    return std::nullopt;
  }
  if (std::abs((*q)[3]) > 45.0 * kDegree) {
    *why = "wrist flipped";
    return std::nullopt;
  }
  c.q = *q;
  c.joint_margin = JointMargin(s.arm, c.q, &c.tight_joint);
  if (c.joint_margin < options.min_joint_margin * kDegree) {
    *why = "a joint near its limit";
    return std::nullopt;
  }
  CheckOptions check;
  check.clearance = true;
  const std::optional<CollisionReport> collision = s.checker->Check(c.q, check);
  if (!collision || !collision->free) {
    *why = "collides within the margin";
    return std::nullopt;
  }
  c.clearance = collision->clearance.value_or(1.0);
  c.clearance_between = collision->clearance_between;
  c.self_clearance = collision->self_clearance.value_or(1.0);
  c.self_between = collision->self_clearance_between;
  if (!RaysClear(s.checker->model(), s.checker->data(), s.module_geom,
                 c.module, lens, why)) {
    return std::nullopt;
  }
  c.scripted_detour = MoveTime(s.over_pallet, c.q, s.limits) +
                      MoveTime(c.q, s.over_slot, s.limits) -
                      MoveTime(s.over_pallet, s.over_slot, s.limits) +
                      options.dwell;
  c.planner_detour = MoveTime(kPlannerStart, c.q, s.limits) +
                     MoveTime(c.q, kPlannerGoal, s.limits) -
                     MoveTime(kPlannerStart, kPlannerGoal, s.limits) +
                     options.dwell;
  return c;
}

void PrintCandidate(const Candidate& c) {
  std::printf("  bearing %5.1f deg (joint 1 %6.1f), r %.2f m, underside "
              "%.2f m: detour %.2f s scripted, %.2f s planner's ends (straight); "
              "clearance "
              "%.0f mm (%s), self %.0f mm; joint margin %.0f deg (joint %d)\n",
              c.bearing / kDegree, -c.bearing / kDegree, c.r, c.h,
              c.scripted_detour, c.planner_detour, 1e3 * c.clearance,
              c.clearance_between.c_str(), 1e3 * c.self_clearance,
              c.joint_margin / kDegree, c.tight_joint + 1);
}

// The XML for the camera and its stand, for models/cell.xml.
void PrintSnippet(const Isometry3d& camera) {
  const Vector3d p = camera.translation();
  const Vector3d x = camera.linear().col(0);
  const Vector3d y = camera.linear().col(1);
  const double lens = p.z();
  // Housing below the lens, wholly behind it; a short post under it.
  const double housing_half = 0.06;
  const double housing_centre = lens - 0.02 - housing_half;
  const double post_top = housing_centre - housing_half;
  std::printf("\n    <!--\n"
              "      The check camera (src/held_module_finder.h): on the floor, "
              "looking straight\n"
              "      up at the module held over it at the check pose, its "
              "underside %.2f m\n"
              "      above the lens. Its rows are turned 5 degrees from the "
              "module's long side.\n"
              "      fovy 61 degrees, 848 x 480: 3.44 mm per pixel at the "
              "underside. The housing\n"
              "      is below the lens, wholly behind it. All of it collides.\n"
              "    -->\n",
              kCameraDistance);
  std::printf("    <body name=\"check_camera_stand\" pos=\"%.4f %.4f 0\">\n",
              p.x(), p.y());
  std::printf("      <geom name=\"check_camera_base\" type=\"box\" "
              "size=\"0.12 0.12 0.01\" pos=\"0 0 0.01\" xyaxes=\"%.7g %.7g 0 "
              "%.7g %.7g 0\" rgba=\"0.35 0.35 0.38 1\"/>\n",
              x.x(), x.y(), -x.y(), x.x());
  if (post_top > 0.03) {
    std::printf("      <geom name=\"check_camera_post\" type=\"box\" "
                "size=\"0.03 0.03 %.4f\" pos=\"0 0 %.4f\" rgba=\"0.35 0.35 "
                "0.38 1\"/>\n",
                0.5 * (post_top - 0.02), 0.5 * (post_top + 0.02));
  }
  std::printf("      <geom name=\"check_camera_housing\" type=\"box\" "
              "size=\"0.06 0.045 %.3g\" pos=\"0 0 %.4g\" xyaxes=\"%.7g %.7g 0 "
              "%.7g %.7g 0\" rgba=\"0.1 0.1 0.1 1\"/>\n",
              housing_half, housing_centre, x.x(), x.y(), -x.y(), x.x());
  std::printf("      <camera name=\"check_camera\" pos=\"0 0 %.4f\" "
              "xyaxes=\"%.7g %.7g %.7g %.7g %.7g %.7g\"\n"
              "              fovy=\"61\" resolution=\"848 480\"/>\n"
              "    </body>\n\n",
              lens, x.x(), x.y(), x.z(), y.x(), y.y(), y.z());
}

// The joint angles' pose in `data`: the arm's joints written, module_0
// (free joint at `module_qpos`) put at tip * DesignGrip(), kinematics.
void PoseWithModule(const mjModel& model, mjData* data, const Arm& arm,
                    int module_qpos, const VectorXd& q) {
  for (int i = 0; i < Arm::kJoints; ++i) data->qpos[arm.qpos[i]] = q[i];
  mj_kinematics(&model, data);
  const Isometry3d module = TipPose(arm, *data) * DesignGrip();
  const Eigen::Quaterniond rotation(module.linear());
  mjtNum* p = data->qpos + module_qpos;
  for (int i = 0; i < 3; ++i) p[i] = module.translation()[i];
  p[3] = rotation.w();
  p[4] = rotation.x();
  p[5] = rotation.y();
  p[6] = rotation.z();
  mj_kinematics(&model, data);
}

// The verification, with check_camera in the model.
int Verify(Setup& s, int camera_id, const std::string& cell_path,
           const PlacementStudyOptions& options) {
  const mjModel& m = *s.model;
  mj_forward(&m, s.data.get());
  const std::optional<CameraModel> camera =
      CameraModelFromMujoco(m, *s.data, camera_id);
  if (!camera) {
    std::printf("check_camera cannot be read as a pinhole\n");
    return 1;
  }
  const Isometry3d module = ModuleFor(camera->pose);
  const double bearing =
      std::atan2(module.translation().y(), module.translation().x());
  const double r = module.translation().head<2>().norm();
  const double h = module.translation().z() - 0.5 * kModuleThickness;
  // The module's long side along the arm's circle, as the swing has it?
  const double off_circle = std::asin(std::clamp(
      module.linear().col(0).dot(Vector3d(std::cos(bearing),
                                          std::sin(bearing), 0.0)),
      -1.0, 1.0));
  std::printf("\n== verification with check_camera in %s\n", cell_path.c_str());
  std::printf("camera at (%.3f, %.3f, %.3f), %d x %d, fovy %.1f deg; module "
              "for it at bearing %.1f deg, r %.3f m, underside %.3f m\n",
              camera->pose.translation().x(), camera->pose.translation().y(),
              camera->pose.translation().z(), camera->width, camera->height,
              camera->fovy / kDegree, bearing / kDegree, r, h);
  std::printf("its long side %.2f deg off the arm's circle\n",
              off_circle / kDegree);
  std::string why;
  const std::optional<Candidate> c = Evaluate(s, module, options, &why);
  if (!c) {
    std::printf("the check pose for this camera fails: %s\n", why.c_str());
    return 1;
  }
  std::printf("check pose, with the camera's stand among the obstacles:\n");
  PrintCandidate(*c);

  // The moves into and out of the check pose, and the carries without it:
  // free at the margin? and how near the stand comes.
  std::vector<int> stand_geoms;
  for (int g = 0; g < m.ngeom; ++g) {
    const char* name = mj_id2name(&m, mjOBJ_GEOM, g);
    if (name && std::string(name).rfind("check_camera", 0) == 0) {
      stand_geoms.push_back(g);
    }
  }
  std::vector<int> moving;
  const int arm_base = mj_name2id(&m, mjOBJ_BODY, "arm_base");
  for (int g = 0; g < m.ngeom; ++g) {
    if (m.geom_contype[g] == 0 && m.geom_conaffinity[g] == 0) continue;
    int b = m.geom_bodyid[g];
    bool mine = b == s.module_body;
    while (!mine && b > 0) {
      if (b == arm_base) mine = true;
      b = m.body_parentid[b];
    }
    if (mine) moving.push_back(g);
  }
  DataPtr posed = MakeData(m);
  const int module_qpos = m.jnt_qposadr[m.body_jntadr[s.module_body]];
  struct Leg {
    const char* name;
    VectorXd from, to;
  };
  // The nearest any of `path`'s configurations brings the arm, tool or
  // module to the camera's stand.
  auto nearest_stand = [&](const JointPath& path) {
    double nearest = 1e9;
    for (const VectorXd& q : path) {
      PoseWithModule(m, posed.get(), s.arm, module_qpos, q);
      for (int a : stand_geoms) {
        for (int g : moving) {
          nearest = std::min(
              nearest, mj_geomDistance(&m, posed.get(), a, g, 5.0, nullptr));
        }
      }
    }
    return nearest;
  };
  std::printf("the script's moves (straight joint moves), at the planning "
              "margin:\n");
  const std::vector<Leg> legs = {
      {"flatten, end of clear -> over_pallet", kPlannerStart, s.over_pallet},
      {"swing, over_pallet -> over the slot", s.over_pallet, s.over_slot},
      {"over_pallet -> check pose", s.over_pallet, c->q},
      {"check pose -> over the slot", c->q, s.over_slot},
  };
  for (const Leg& leg : legs) {
    SegmentReport segment;
    const double step = s.checker->SegmentStep(leg.from, leg.to, 0.01);
    const bool free =
        s.checker->SegmentIsFree(leg.from, leg.to, step, &segment);
    JointPath samples;
    for (int k = 0; k <= 200; ++k) {
      samples.push_back(leg.from + (leg.to - leg.from) * (k / 200.0));
    }
    const double nearest = nearest_stand(samples);
    std::string hit;
    if (!free && !segment.hit_report.pairs.empty()) {
      char buffer[64];
      std::snprintf(buffer, sizeof(buffer), " at %.0f%%: ",
                    100.0 * segment.hit_fraction);
      hit = buffer + segment.hit_report.pairs.front().between;
    }
    std::printf("  %-46s %4.2f s, %s at %.0f mm%s; nearest the camera's "
                "stand %.2f m\n",
                leg.name, MoveTime(leg.from, leg.to, s.limits),
                free ? "free" : "NOT free", 1e3 * s.checker->margin(),
                hit.c_str(), nearest);
  }

  // The planner's carry, planned as the cell plans it (src/motion_planner.h,
  // its defaults: 20 mm required, 45 mm planned), straight to the goal and
  // through the check pose.
  std::printf("the planner's carry (PlanArmMove, from its start to its "
              "goal):\n");
  PlanScene scene;
  scene.world = s.data.get();
  scene.held_module = 0;
  scene.module_in_tip = DesignGrip();
  auto plan = [&](const char* name, const VectorXd& from, const VectorXd& to,
                  double* seconds) {
    MotionPlanReport report;
    const std::optional<TimedMove> move = PlanArmMove(
        *s.checker, scene, from, to, s.arm.limits, {}, &report);
    if (!move) {
      std::printf("  %-46s not found: %s\n", name, report.reason.c_str());
      *seconds = 1e9;
      return;
    }
    *seconds = move->trajectory.duration();
    std::printf("  %-46s %4.2f s, clearance %.0f mm (%s); nearest the "
                "camera's stand %.2f m\n",
                name, *seconds, 1e3 * report.clearance,
                report.clearance_between.c_str(),
                nearest_stand(DenseSamples(*s.checker, move->path, 0.05)));
  };
  double direct = 0.0, in = 0.0, out = 0.0;
  plan("start -> goal", kPlannerStart, kPlannerGoal, &direct);
  plan("start -> check pose", kPlannerStart, c->q, &in);
  plan("check pose -> goal", c->q, kPlannerGoal, &out);
  std::printf("  through the check pose: %.2f + %.2f + %.2f s dwell = %.2f s "
              "against %.2f s: %+.2f s\n",
              in, out, options.dwell, in + out + options.dwell, direct,
              in + out + options.dwell - direct);
  std::printf("  the script: flatten %.2f + swing %.2f = %.2f s; through the "
              "check pose %.2f + %.2f + %.2f + %.2f s dwell = %.2f s: %+.2f "
              "s\n",
              MoveTime(kPlannerStart, s.over_pallet, s.limits),
              MoveTime(s.over_pallet, s.over_slot, s.limits),
              MoveTime(kPlannerStart, s.over_pallet, s.limits) +
                  MoveTime(s.over_pallet, s.over_slot, s.limits),
              MoveTime(kPlannerStart, s.over_pallet, s.limits),
              MoveTime(s.over_pallet, c->q, s.limits),
              MoveTime(c->q, s.over_slot, s.limits), options.dwell,
              MoveTime(kPlannerStart, s.over_pallet, s.limits) +
                  MoveTime(s.over_pallet, c->q, s.limits) +
                  MoveTime(c->q, s.over_slot, s.limits) + options.dwell,
              c->scripted_detour);

  // The picture, in the cell, with the real grip and the arm's own sag.
  ModelPtr model = LoadModel(cell_path);
  DataPtr data = MakeData(*model);
  std::string error;
  // As on the rig: the grip offsets drawn reach 10 mm along the normal, on
  // top of the 2 mm gap.
  SuctionOptions grip_options;
  grip_options.max_distance = 0.015;
  std::optional<Suction> suction =
      Suction::Create(*model, grip_options, &error);
  if (!suction) {
    std::printf("no suction: %s\n", error.c_str());
    return 1;
  }
  std::printf("pictures in the cell (noise on, arm sagging under the module, "
              "expected pose from the joint angles):\n");
  int failures = 0;
  for (int k = 1; k <= options.verify_shots; ++k) {
    mj_resetData(model.get(), data.get());
    suction->Release(data.get());
    for (int i = 0; i < Arm::kJoints; ++i) {
      data->qpos[s.arm.qpos[i]] = c->q[i];
      data->ctrl[s.arm.motor[i]] = c->q[i];
    }
    mj_kinematics(model.get(), data.get());
    const Isometry3d offset =
        DrawGripOffset(static_cast<std::uint32_t>(k), 0.01, 1.5 * kDegree);
    const Isometry3d held = TipPose(s.arm, *data) * DesignGrip() * offset;
    const Eigen::Quaterniond rotation(held.linear());
    mjtNum* p = data->qpos + module_qpos;
    for (int i = 0; i < 3; ++i) p[i] = held.translation()[i];
    p[3] = rotation.w();
    p[4] = rotation.x();
    p[5] = rotation.y();
    p[6] = rotation.z();
    mj_forward(model.get(), data.get());
    GripFailure grip;
    if (!suction->Grip(model.get(), data.get(), &grip)) {
      std::printf("  shot %d: no grip: %s\n", k, grip.message.c_str());
      ++failures;
      continue;
    }
    while (data->time < 0.5) mj_step(model.get(), data.get());
    mj_forward(model.get(), data.get());
    // What the control code knows: the joint angles, and from them the tip.
    const Isometry3d tip = TipPose(s.arm, *data);
    const Pose truth_pose = *suction->HeldPoseInTip(*data);
    Isometry3d truth = Isometry3d::Identity();
    truth.translation() = truth_pose.position;
    truth.linear() = truth_pose.rotation.toRotationMatrix();

    const CameraModel cam = *CameraModelFromMujoco(*model, *data, camera_id);
    DepthOptions depth;
    depth.noise.depth_noise = true;
    depth.noise.edge_effects = true;
    depth.noise.quantize = true;
    depth.noise.seed = 1000u + k;
    const auto start = std::chrono::steady_clock::now();
    const DepthImage image = RenderDepth(*model, data.get(), cam, depth);
    const double render = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - start)
                              .count();
    HeldFinderDiagnostics d;
    const std::optional<HeldModuleEstimate> estimate =
        FindHeldModule(image, cam, tip * DesignGrip(), {}, &d);
    const double sag =
        (tip.translation() - TipPoseAt(*model, s.scratch.get(), s.arm, c->q)
                                 .translation())
            .norm();
    if (!estimate) {
      std::printf("  shot %d: refused, %s: %s\n", k,
                  HeldFinderFailureName(d.failure), d.detail.c_str());
      ++failures;
      continue;
    }
    const Eigen::Matrix<double, 6, 1> e =
        PoseError6(GripOffsetFromCheck(estimate->pose, tip), truth);
    std::printf("  shot %d: error x %.3f y %.3f z %.3f mm, about z %.4f deg; "
                "%d points, view %d/%d on the underside; tip sagged %.2f mm "
                "from the commanded pose; render %.0f ms, fit %.0f ms\n",
                k, 1e3 * e[0], 1e3 * e[1], 1e3 * e[2], e[5] / kDegree,
                d.interior_points, d.view.on_glass, d.view.pixels, 1e3 * sag,
                1e3 * render, 1e3 * d.seconds_total);
  }
  return failures == 0 ? 0 : 1;
}

}  // namespace

int RunPlacementStudy(const std::string& cell_path,
                      const PlacementStudyOptions& options) {
  Setup s;
  std::string error;
  s.model = LoadModel(cell_path, &error);
  if (!s.model) {
    std::printf("could not load %s: %s\n", cell_path.c_str(), error.c_str());
    return 1;
  }
  const mjModel& m = *s.model;
  s.data = MakeData(m);
  s.scratch = MakeData(m);
  mj_forward(&m, s.data.get());
  const std::optional<Arm> arm = FindArm(m);
  if (!arm) {
    std::printf("no arm in %s\n", cell_path.c_str());
    return 1;
  }
  s.arm = *arm;
  s.limits = Scaled(s.arm.limits, options.speed);
  s.module_body = mj_name2id(&m, mjOBJ_BODY, "module_0");
  s.module_geom = s.module_body >= 0 ? m.body_geomadr[s.module_body] : -1;
  CollisionOptions collision;
  collision.margin = options.margin;
  collision.self_margin = options.self_margin;
  std::optional<CollisionChecker> checker =
      CollisionChecker::Create(m, collision, &error);
  if (!checker || s.module_body < 0) {
    std::printf("no collision checker: %s\n", error.c_str());
    return 1;
  }
  s.checker.emplace(std::move(*checker));
  s.checker->SetScene(*s.data);
  s.checker->SetHeldModule(0, DesignGrip());
  // Link 1 turns on the base 0.23 m above the floor whatever the angles;
  // left in, that distance would be every pose's clearance.
  s.checker->AllowContact("arm_link_1", "world");

  // The scripted swing's ends, as src/pick_place.cc builds them for the
  // nominal pallet: over_pallet, the tip at (-0.45, 0, 1.60) in the pallet's
  // frame (origin on the floor under module_0's glass, (1.95, 0, 0)), tool
  // down, its X along the module's +x (world -Y); and over the slot, the tip
  // 0.35 m outboard of the placement spot and 0.10 m above the release,
  // its X along +Y.
  const Isometry3d over_pallet = *GripPose(
      Vector3d(1.50, 0.0, 1.60), Vector3d::UnitZ(), -Vector3d::UnitY());
  const Isometry3d over_slot = *GripPose(
      Vector3d(-2.35, 0.0, 0.8175 + 0.0175 + 0.001 + kGripGap + 0.10),
      Vector3d::UnitZ(), Vector3d::UnitY());
  VectorXd place_seed = kPlaceSeed;
  const std::optional<VectorXd> q_over_pallet = SolveIk(
      m, s.scratch.get(), s.arm, over_pallet, kOverPalletSeed);
  if (q_over_pallet) place_seed[5] = (*q_over_pallet)[5];
  const std::optional<VectorXd> q_over_slot =
      SolveIk(m, s.scratch.get(), s.arm, over_slot, place_seed);
  if (!q_over_pallet || !q_over_slot) {
    std::printf("no IK for the scripted swing's ends\n");
    return 1;
  }
  s.over_pallet = *q_over_pallet;
  s.over_slot = *q_over_slot;
  std::printf("scripted swing: over_pallet (%s) deg -> over the slot (%s) "
              "deg, %.2f s at speed %.2f\n",
              Join(s.over_pallet, 1 / kDegree, "%.1f").c_str(),
              Join(s.over_slot, 1 / kDegree, "%.1f").c_str(),
              MoveTime(s.over_pallet, s.over_slot, s.limits), options.speed);
  std::printf("planner carry: start (%s) deg -> goal (%s) deg, %.2f s as one "
              "straight joint move\n",
              Join(kPlannerStart, 1 / kDegree, "%.0f").c_str(),
              Join(kPlannerGoal, 1 / kDegree, "%.0f").c_str(),
              MoveTime(kPlannerStart, kPlannerGoal, s.limits));

  const int camera_id = mj_name2id(&m, mjOBJ_CAMERA, "check_camera");
  if (camera_id >= 0) return Verify(s, camera_id, cell_path, options);

  // The search.
  std::vector<Candidate> found;
  std::map<std::string, int> rejected;
  int tried = 0;
  for (double h : {1.55, 1.65, 1.75}) {
    for (double a = 20.0; a <= 170.0 + 1e-9; a += 5.0) {
      for (double r = 1.30; r <= 2.10 + 1e-9; r += 0.05) {
        ++tried;
        std::string why;
        std::optional<Candidate> c =
            Evaluate(s, ModuleAt(a * kDegree, r, h), options, &why);
        if (c) {
          found.push_back(*c);
        } else {
          ++rejected[why];
        }
      }
    }
  }
  std::printf("\n%d candidate check poses (bearing 20 to 170 deg, r 1.30 to "
              "2.10 m, underside 1.55, 1.65, 1.75 m), %zu qualify; "
              "rejected:\n",
              tried, found.size());
  for (const auto& [reason, count] : rejected) {
    std::printf("  %5d  %s\n", count, reason.c_str());
  }
  if (found.empty()) return 1;
  std::sort(found.begin(), found.end(),
            [](const Candidate& a, const Candidate& b) {
              return a.scripted_detour + a.planner_detour <
                     b.scripted_detour + b.planner_detour;
            });
  std::printf("\nthe best at each bearing:\n");
  for (double a = 20.0; a <= 170.0 + 1e-9; a += 5.0) {
    for (const Candidate& c : found) {
      if (std::abs(c.bearing / kDegree - a) < 1e-6) {
        PrintCandidate(c);
        break;
      }
    }
  }
  std::printf("\nbest 15 by the two detours together:\n");
  for (int i = 0; i < std::min<int>(15, found.size()); ++i) {
    PrintCandidate(found[i]);
  }
  for (double h : {1.55, 1.65, 1.75}) {
    for (const Candidate& c : found) {
      if (std::abs(c.h - h) < 1e-9) {
        std::printf("best at underside %.2f m:\n", h);
        PrintCandidate(c);
        break;
      }
    }
  }

  const Candidate& best = found.front();
  std::printf("\nrecommended check pose:\n");
  PrintCandidate(best);
  std::printf("  joint angles (%s) deg\n",
              Join(best.q, 1 / kDegree, "%.2f").c_str());
  std::printf("  joint angles (%s) rad\n",
              Join(best.q, 1.0, "%.5f").c_str());
  const Eigen::Quaterniond tq(best.tip.linear());
  std::printf("  tool tip at (%.4f, %.4f, %.4f), quat (w x y z) %.6f %.6f "
              "%.6f %.6f, pointing down, its X along the module's +x (%.4f, "
              "%.4f, 0)\n",
              best.tip.translation().x(), best.tip.translation().y(),
              best.tip.translation().z(), tq.w(), tq.x(), tq.y(), tq.z(),
              best.module.linear()(0, 0), best.module.linear()(1, 0));
  std::printf("  module centre at (%.4f, %.4f, %.4f), underside %.3f m\n",
              best.module.translation().x(), best.module.translation().y(),
              best.module.translation().z(), best.h);
  std::printf("  times: over_pallet -> check %.2f s, check -> over the slot "
              "%.2f s, against %.2f s straight; planner legs %.2f + %.2f "
              "against %.2f s; dwell %.2f s\n",
              MoveTime(s.over_pallet, best.q, s.limits),
              MoveTime(best.q, s.over_slot, s.limits),
              MoveTime(s.over_pallet, s.over_slot, s.limits),
              MoveTime(kPlannerStart, best.q, s.limits),
              MoveTime(best.q, kPlannerGoal, s.limits),
              MoveTime(kPlannerStart, kPlannerGoal, s.limits), options.dwell);
  std::printf("  or with the check pose in place of over_pallet as the "
              "flatten's goal: start -> check -> over the slot %.2f + %.2f s "
              "against start -> over_pallet -> over the slot %.2f + %.2f s\n",
              MoveTime(kPlannerStart, best.q, s.limits),
              MoveTime(best.q, s.over_slot, s.limits),
              MoveTime(kPlannerStart, s.over_pallet, s.limits),
              MoveTime(s.over_pallet, s.over_slot, s.limits));
  std::printf("  camera lens at (%.4f, %.4f, %.4f)\n",
              best.camera.translation().x(), best.camera.translation().y(),
              best.camera.translation().z());
  PrintSnippet(best.camera);
  return 0;
}

}  // namespace cell
