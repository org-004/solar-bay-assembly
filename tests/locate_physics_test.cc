// R8: the locating move on one module, in physics.
//
// docs/bay-hardware.md, "Locating a module before pinning", designs a tapered
// nose that rises from below through a rail's slot into the frame's slot and
// slides the module until a pin can go in, hole by hole in the order 0, 2, 3,
// 1. This test builds that scene in MuJoCo and runs it from several
// placements the gate accepts, the worst of R6 among them.
//
// The scene, made as text here:
//   - the tube and the two rails of one module from src/bay_model.h's
//     emitters, at the bay's spacing (rails 1.144 m apart), fixed to the
//     world. Their geoms are drawn and carry no contacts: the emitted rail
//     top is a solid sheet without slots.
//   - the module from the same emitter as a FREE body (its nine geoms give
//     the 32.4 kg and the inertia), with no contacts on those geoms either.
//   - what touches: each rail's top over its full 460 x 100 mm, 2.0 mm thick,
//     and each of the module's long flanges over the rails' reach, 28 mm wide
//     and 1.7 mm thick, each built from boxes that leave the slots open:
//     7.1 x 13 in the rail, 10 x 7 in the frame. The slots are RECTANGLES,
//     not the document's obrounds; that leaves more opening at the corners
//     and changes nothing on the straight sides, where all four holes of the
//     cases below are worked.
//   - four noses, one under each hole, each a body on three slide joints:
//     X and Y with a spring of 300 N/m and damping (the floating mount) and
//     Z driven by a position actuator whose force is limited to the tool's
//     rated 300 N (kNoseToolThrust). No arm. The nose is the document's: a
//     ball tip 2.0 across, a taper 8.6 long to 6.6, a land 6.0 long (one
//     convex mesh of three rings, with a sphere for the tip).
//
// What is not the document's move:
//   - the tip's search loop. Each nose's float is centred (its spring's rest)
//     under the opening the two slots leave, computed from the module's true
//     pose once it has settled on its rails; the test reads that pose, as a
//     test may. The nose then rises straight up.
//   - the loose pins. A nose that is in stays in while the next one rises: it
//     stands for the pin, and at 6.6 mm it is stricter than the pin's 6.35.
//   - the holes' own tolerance. The model's frame slots are at their nominal
//     places. One case starts 3.5 mm across, beyond the gate's 3.0: its hole
//     0 is 4.935 mm out, the error the worst of R6 has once the 0.5 mm of
//     hole tolerance is against it.
//
// Contacts, after models/pallet.xml's reasoning: a timestep of 0.25 ms and
// solref 0.001 1 (four timesteps: the 2 mm sheets want a contact that gives
// microns, not tenths), solimp 0.95 0.99 0.0005, elliptic cones so that the
// friction of 0.61 (kLocateFriction) is a circle, not a pyramid, condim 3,
// margin and gap zero. The noses touch only the slot pieces; the module only
// the rails and the noses.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/bay_hardware.h"
#include "src/bay_model.h"
#include "src/mj_handles.h"

namespace {

namespace bay = cell::bay;

constexpr double kPi = 3.14159265358979323846;
constexpr double kTimestep = 0.00025;

// The world is the tube's frame shifted along Y so that the module's centre
// is at the origin: rails at Y = -/+0.572, rail tops at Z = 0.124.
constexpr double kFrameY = -(bay::kFirstRailY + bay::kModulePitch / 2.0);
constexpr double kRailTop = bay::kRailTopHeight;
constexpr double kModuleZ = kRailTop + bay::kModuleThickness / 2.0;

// Contact bits: rails 1, module 2, noses 4.
constexpr char kRailContact[] = "contype=\"1\" conaffinity=\"6\"";
constexpr char kModuleContact[] = "contype=\"2\" conaffinity=\"5\"";
constexpr char kNoseContact[] = "contype=\"4\" conaffinity=\"0\"";

// Z of the nose body's origin (the land's upper end) at the start, with the
// tip's top 1 mm under the rail sheet, and fully in, the land 1 mm above the
// flange's top face.
constexpr double kNoseTipAboveOrigin =
    bay::kNoseTaperLength + bay::kNoseTipDiameter / 2.0;
constexpr double kNoseStartZ =
    kRailTop - bay::kRailSheet - 0.001 - kNoseTipAboveOrigin;
constexpr double kNoseInZ =
    kRailTop + bay::kFlangeThickness + bay::kNoseOverTravel;

std::string Num(double v) {
  char b[32];
  std::snprintf(b, sizeof(b), "%.6g", v);
  return b;
}

// A box from its extents.
std::string Box(const std::string& name, double x0, double x1, double y0,
                double y1, double z0, double z1, const char* contact,
                bool massless) {
  return "<geom name=\"" + name + "\" type=\"box\" " + contact +
         " pos=\"" + Num((x0 + x1) / 2) + " " + Num((y0 + y1) / 2) + " " +
         Num((z0 + z1) / 2) + "\" size=\"" + Num((x1 - x0) / 2) + " " +
         Num((y1 - y0) / 2) + " " + Num((z1 - z0) / 2) + "\"" +
         (massless ? " mass=\"0\"" : "") + " rgba=\"0.9 0.6 0.2 1\"/>\n";
}

// A plate over [x0, x1] x [y0, y1] x [z0, z1] with rectangular openings of
// half sizes (hx, hy) at (holes_x[i], hole_y), the holes sorted by X.
std::string SlottedPlate(const std::string& name, double x0, double x1,
                         double y0, double y1, double z0, double z1,
                         const std::vector<double>& holes_x, double hole_y,
                         double hx, double hy, const char* contact,
                         bool massless) {
  std::string out;
  double x = x0;
  int k = 0;
  for (double hxc : holes_x) {
    out += Box(name + "_" + std::to_string(k++), x, hxc - hx, y0, y1, z0, z1,
               contact, massless);
    out += Box(name + "_" + std::to_string(k++), hxc - hx, hxc + hx, y0,
               hole_y - hy, z0, z1, contact, massless);
    out += Box(name + "_" + std::to_string(k++), hxc - hx, hxc + hx,
               hole_y + hy, y1, z0, z1, contact, massless);
    x = hxc + hx;
  }
  out += Box(name + "_" + std::to_string(k++), x, x1, y0, y1, z0, z1, contact,
             massless);
  return out;
}

// The nose as a convex mesh: the land (two rings of 6.6) and the taper to a
// ring of the tip's 2.0, the origin at the land's upper end.
std::string NoseMesh() {
  std::string v;
  const int n = 24;
  const double r_full = bay::kNoseDiameter / 2.0;
  const double r_tip = bay::kNoseTipDiameter / 2.0;
  const std::array<std::pair<double, double>, 3> rings = {
      std::pair{-bay::kNoseLandLength, r_full}, std::pair{0.0, r_full},
      std::pair{bay::kNoseTaperLength, r_tip}};
  for (const auto& [z, r] : rings) {
    for (int i = 0; i < n; ++i) {
      const double a = 2.0 * kPi * i / n;
      v += Num(r * std::cos(a)) + " " + Num(r * std::sin(a)) + " " + Num(z) +
           "  ";
    }
  }
  return "<mesh name=\"nose\" vertex=\"" + v + "\"/>\n";
}

std::string SceneXml() {
  std::string xml =
      "<mujoco model=\"locate\">\n"
      "<option timestep=\"" + Num(kTimestep) +
      "\" cone=\"elliptic\" integrator=\"implicitfast\"/>\n"
      "<default>\n"
      "  <geom condim=\"3\" friction=\"" + Num(bay::kLocateFriction) +
      " 0.005 0.0001\" solref=\"0.001 1\" solimp=\"0.95 0.99 0.0005\""
      " margin=\"0\" gap=\"0\"/>\n"
      "  <default class=\"bay_steel\"><geom type=\"box\" contype=\"0\""
      " conaffinity=\"0\" rgba=\"0.62 0.64 0.66 0.3\"/></default>\n"
      "  <default class=\"bay_aluminium\"><geom type=\"box\" contype=\"0\""
      " conaffinity=\"0\" rgba=\"0.8 0.82 0.85 0.3\"/></default>\n"
      "  <default class=\"bay_laminate\"><geom type=\"box\" contype=\"0\""
      " conaffinity=\"0\" rgba=\"0.08 0.12 0.32 0.3\"/></default>\n"
      "  <default class=\"bay_site_a\"><site size=\"0.004\"/></default>\n"
      "  <default class=\"bay_site_b\"><site size=\"0.004\"/></default>\n"
      "  <default class=\"bay_site_locate\"><site size=\"0.004\"/></default>\n"
      "  <default class=\"bay_site_hold\"><site type=\"cylinder\""
      " group=\"3\"/></default>\n"
      "</default>\n"
      "<asset>\n" + NoseMesh() + "</asset>\n"
      "<worldbody>\n"
      "<frame pos=\"0 " + Num(kFrameY) + " 0\">\n";
  xml += bay::TubeBodyXml();
  const double rail_y0 = bay::kFirstRailY;
  const double rail_y1 = bay::kFirstRailY + bay::kModulePitch;
  xml += bay::RailBodyXml("rail_a", {0.0, rail_y0, 0.0});
  xml += bay::RailBodyXml("rail_b", {0.0, rail_y1, 0.0});
  std::string module = bay::ModuleBodyXml(
      "module", {0.0, rail_y0 + bay::kModulePitch / 2.0, kModuleZ});
  // The free joint goes after the body's opening line; the contact geoms of
  // the flanges before its end. Both in the module's frame.
  const size_t first_line = module.find('\n') + 1;
  module.insert(first_line, "<freejoint name=\"module_free\"/>\n");
  std::string flanges;
  const double hy = bay::HolePlace(2).y();  // 0.548
  const double half_w = bay::kModuleWidth / 2.0;
  const double fz0 = -bay::kModuleThickness / 2.0;
  const double fz1 = fz0 + bay::kFlangeThickness;
  for (int side = 0; side < 2; ++side) {
    const double s = side == 0 ? -1.0 : 1.0;
    const double e0 = s * half_w;
    const double e1 = s * (half_w - bay::kFlangeWidthLongSide);
    flanges += SlottedPlate(
        std::string("module_flange_") + (side == 0 ? "a" : "b"), -0.26, 0.26,
        std::min(e0, e1), std::max(e0, e1), fz0, fz1, {-0.2, 0.2}, s * hy,
        bay::kModuleSlotAcrossTube / 2.0, bay::kModuleSlotAlongTube / 2.0,
        kModuleContact, true);
  }
  module.insert(module.rfind("</body>"), flanges);
  xml += module;
  xml += "</frame>\n";
  // The rails' tops with their slots, in the world.
  for (int side = 0; side < 2; ++side) {
    const double s = side == 0 ? -1.0 : 1.0;
    const double yr = s * bay::kModulePitch / 2.0;
    xml += SlottedPlate(std::string("rail_top_") + (side == 0 ? "a" : "b"),
                        -bay::kRailLength / 2.0, bay::kRailLength / 2.0,
                        yr - bay::kRailTopWidth / 2.0,
                        yr + bay::kRailTopWidth / 2.0,
                        kRailTop - bay::kRailSheet, kRailTop, {-0.2, 0.2},
                        s * hy, bay::kRailSlotAcrossTube / 2.0,
                        bay::kRailSlotAlongTube / 2.0, kRailContact, false);
  }
  // The noses, placed for each case by the test (body_pos).
  for (int h = 0; h < 4; ++h) {
    const std::string n = "nose_" + std::to_string(h);
    const Eigen::Vector2d p = bay::HolePlace(h);
    xml += "<body name=\"" + n + "\" pos=\"" + Num(p.x()) + " " + Num(p.y()) +
           " " + Num(kNoseStartZ) + "\">\n"
           "  <inertial pos=\"0 0 0\" mass=\"0.5\" diaginertia=\"1e-4 1e-4 "
           "1e-4\"/>\n"
           "  <joint name=\"" + n + "_x\" type=\"slide\" axis=\"1 0 0\""
           " stiffness=\"300\" damping=\"20\" limited=\"true\""
           " range=\"-0.006 0.006\"/>\n"
           "  <joint name=\"" + n + "_y\" type=\"slide\" axis=\"0 1 0\""
           " stiffness=\"300\" damping=\"20\" limited=\"true\""
           " range=\"-0.008 0.008\"/>\n"
           "  <joint name=\"" + n + "_z\" type=\"slide\" axis=\"0 0 1\""
           " damping=\"500\" limited=\"true\" range=\"-0.005 0.03\"/>\n"
           "  <geom name=\"" + n + "_body\" type=\"mesh\" mesh=\"nose\" " +
           kNoseContact + " rgba=\"0.2 0.2 0.2 1\"/>\n"
           "  <geom name=\"" + n + "_tip\" type=\"sphere\" size=\"" +
           Num(bay::kNoseTipDiameter / 2.0) + "\" pos=\"0 0 " +
           Num(bay::kNoseTaperLength) + "\" " + kNoseContact +
           " rgba=\"0.2 0.2 0.2 1\"/>\n"
           "</body>\n";
  }
  xml += "</worldbody>\n<actuator>\n";
  for (int h = 0; h < 4; ++h) {
    const std::string n = "nose_" + std::to_string(h);
    xml += "<position name=\"" + n + "_lift\" joint=\"" + n +
           "_z\" kp=\"300000\" forcelimited=\"true\" forcerange=\"-" +
           Num(bay::kNoseToolThrust) + " " + Num(bay::kNoseToolThrust) +
           "\"/>\n";
  }
  xml += "</actuator>\n</mujoco>\n";
  return xml;
}

struct Start {
  const char* name;
  double dx, dy, yaw_deg;
};

struct Pose {
  Eigen::Vector3d pos;
  Eigen::Matrix3d rot;
};

class LocatePhysicsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string xml = SceneXml();
    char error[1000] = "";
    mjSpec* spec =
        mj_parseXMLString(xml.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;
    model_.reset(mj_compile(spec, nullptr));
    if (!model_) std::printf("%s\n", mjs_getError(spec));
    mj_deleteSpec(spec);
    ASSERT_TRUE(model_);
    data_ = cell::MakeData(*model_);
    module_body_ = mj_name2id(model_.get(), mjOBJ_BODY, "module");
    ASSERT_GE(module_body_, 0);
    for (int h = 0; h < 4; ++h) {
      const std::string n = "nose_" + std::to_string(h);
      nose_body_[h] = mj_name2id(model_.get(), mjOBJ_BODY, n.c_str());
      nose_act_[h] =
          mj_name2id(model_.get(), mjOBJ_ACTUATOR, (n + "_lift").c_str());
      const int jz = mj_name2id(model_.get(), mjOBJ_JOINT, (n + "_z").c_str());
      ASSERT_GE(jz, 0);
      nose_z_qpos_[h] = model_->jnt_qposadr[jz];
    }
  }

  Pose ModulePose() const {
    Pose p;
    for (int i = 0; i < 3; ++i) p.pos[i] = data_->xpos[3 * module_body_ + i];
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c)
        p.rot(r, c) = data_->xmat[9 * module_body_ + 3 * r + c];
    return p;
  }

  // Hole h's error: the frame slot's centre less the rail slot's, in the
  // rails' plane, metres.
  Eigen::Vector2d HoleErr(const Pose& p, int h) const {
    const Eigen::Vector2d place = bay::HolePlace(h);
    const Eigen::Vector3d w =
        p.pos + p.rot * Eigen::Vector3d(place.x(), place.y(), 0.0);
    return w.head<2>() - place;
  }
  double HoleZ(const Pose& p, int h) const {
    const Eigen::Vector2d place = bay::HolePlace(h);
    return (p.pos + p.rot * Eigen::Vector3d(place.x(), place.y(),
                                            -bay::kModuleThickness / 2.0))
        .z();
  }
  static double Yaw(const Pose& p) {
    return std::atan2(p.rot(1, 0), p.rot(0, 0));
  }

  cell::ModelPtr model_;
  cell::DataPtr data_;
  int module_body_ = -1;
  std::array<int, 4> nose_body_{};
  std::array<int, 4> nose_act_{};
  std::array<int, 4> nose_z_qpos_{};
};

TEST_F(LocatePhysicsTest, NosesBringEveryHoleIntoThePinsPlay) {
  const std::vector<Start> starts = {
      {"centred", 0.0, 0.0, 0.0},
      {"3.0 across, +0.15 deg (R6 worst, nominal holes)", 0.003, 0.0, 0.15},
      {"3.5 across, +0.15 deg (worst with hole tolerance)", 0.0035, 0.0, 0.15},
      {"3.0 along, +0.15 deg", 0.0, 0.003, 0.15},
      {"3.0 at 45 deg, -0.15 deg", 0.003 / std::sqrt(2.0),
       0.003 / std::sqrt(2.0), -0.15},
  };
  const int settle_steps = static_cast<int>(0.3 / kTimestep);
  const double rise_speed = 0.020;  // m/s
  const double rise = kNoseInZ - kNoseStartZ;
  const int rise_steps = static_cast<int>(rise / rise_speed / kTimestep);
  const int hold_steps = static_cast<int>(0.15 / kTimestep);

  for (const Start& start : starts) {
    SCOPED_TRACE(start.name);
    mj_resetData(model_.get(), data_.get());
    // Put the noses back under their rail slots before the module settles.
    for (int h = 0; h < 4; ++h) {
      const Eigen::Vector2d place = bay::HolePlace(h);
      model_->body_pos[3 * nose_body_[h] + 0] = place.x();
      model_->body_pos[3 * nose_body_[h] + 1] = place.y();
      data_->ctrl[nose_act_[h]] = 0.0;
    }
    const double yaw = start.yaw_deg * kPi / 180.0;
    data_->qpos[0] = start.dx;
    data_->qpos[1] = start.dy;
    data_->qpos[2] = kModuleZ + 0.0002;
    data_->qpos[3] = std::cos(yaw / 2.0);
    data_->qpos[4] = 0.0;
    data_->qpos[5] = 0.0;
    data_->qpos[6] = std::sin(yaw / 2.0);
    for (int i = 0; i < settle_steps; ++i) mj_step(model_.get(), data_.get());

    const Pose before = ModulePose();
    std::array<Eigen::Vector2d, 4> err_before;
    for (int h = 0; h < 4; ++h) err_before[h] = HoleErr(before, h);
    // Aim each float under the opening the two slots leave at the settled
    // pose (the true pose: this stands for the tip's search loop). Both
    // slots taken as rectangles; the opening is their overlap.
    for (int h = 0; h < 4; ++h) {
      const Eigen::Vector2d place = bay::HolePlace(h);
      const Eigen::Vector2d e = err_before[h];
      const double ax0 = std::max(-bay::kRailSlotAcrossTube / 2.0,
                                  e.x() - bay::kModuleSlotAcrossTube / 2.0);
      const double ax1 = std::min(bay::kRailSlotAcrossTube / 2.0,
                                  e.x() + bay::kModuleSlotAcrossTube / 2.0);
      const double ay0 = std::max(-bay::kRailSlotAlongTube / 2.0,
                                  e.y() - bay::kModuleSlotAlongTube / 2.0);
      const double ay1 = std::min(bay::kRailSlotAlongTube / 2.0,
                                  e.y() + bay::kModuleSlotAlongTube / 2.0);
      ASSERT_GT(ax1 - ax0, bay::TipNeeds()) << "hole " << h;
      ASSERT_GT(ay1 - ay0, bay::TipNeeds()) << "hole " << h;
      model_->body_pos[3 * nose_body_[h] + 0] = place.x() + (ax0 + ax1) / 2.0;
      model_->body_pos[3 * nose_body_[h] + 1] = place.y() + (ay0 + ay1) / 2.0;
    }
    mj_forward(model_.get(), data_.get());

    const double z_rest = before.pos.z();
    double max_force = 0.0;
    double max_lift = 0.0;
    std::array<double, 4> hole_z0;
    for (int h = 0; h < 4; ++h) hole_z0[h] = HoleZ(before, h);
    for (int h : bay::kLocateOrder) {
      for (int i = 0; i <= rise_steps + hold_steps; ++i) {
        data_->ctrl[nose_act_[h]] =
            rise * std::min(1.0, static_cast<double>(i) / rise_steps);
        mj_step(model_.get(), data_.get());
        for (int k = 0; k < 4; ++k)
          max_force =
              std::max(max_force, std::abs(data_->actuator_force[nose_act_[k]]));
        const Pose p = ModulePose();
        for (int k = 0; k < 4; ++k)
          max_lift = std::max(max_lift, HoleZ(p, k) - hole_z0[k]);
      }
      EXPECT_GT(data_->qpos[nose_z_qpos_[h]], rise - 0.0005)
          << "nose " << h << " did not get fully in";
    }

    const Pose after = ModulePose();
    std::printf("%s\n  hole  before (across, along) mm   after mm\n",
                start.name);
    for (int h = 0; h < 4; ++h) {
      const Eigen::Vector2d e = HoleErr(after, h);
      std::printf("  %d    %7.3f %7.3f            %7.3f %7.3f  %s\n", h,
                  1e3 * err_before[h].x(), 1e3 * err_before[h].y(),
                  1e3 * e.x(), 1e3 * e.y(),
                  bay::InsidePinPlay(e.x(), e.y()) ? "inside" : "OUTSIDE");
      EXPECT_TRUE(bay::InsidePinPlay(e.x(), e.y()))
          << "hole " << h << " at " << 1e3 * e.x() << ", " << 1e3 * e.y();
    }
    const double dz = after.pos.z() - z_rest;
    std::printf(
        "  yaw %.4f -> %.4f deg, centre dz %.4f mm, largest rise of a hole "
        "%.4f mm, largest nose force %.1f N (document: 248 N)\n",
        Yaw(before) * 180.0 / kPi, Yaw(after) * 180.0 / kPi, 1e3 * dz,
        1e3 * max_lift, max_force);
    EXPECT_LT(std::abs(dz), 0.0003);
    EXPECT_LT(max_lift, 0.0003);
    EXPECT_LT(std::abs(Yaw(after)), 0.25 * kPi / 180.0);
  }
}

}  // namespace
