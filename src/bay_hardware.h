// One bay of a single-axis solar tracker as it really is: a round torque
// tube section, the rails across it, the modules on the rails, and every
// point where one part is fastened to another.
//
// docs/bay-hardware.md is the source of every figure here: it says which are
// published, which are derived and which were designed in this project, and
// why. models/bay.xml is the same bay as a MuJoCo model, and
// tests/bay_hardware_test.cc checks that the model, this header and the
// document's counts agree. The cell (models/cell.xml, src/conveyor.h) does
// not use these parts yet: it still carries its old square tube.
//
// The bay. The tube is a round steel tube 124 mm across. Eleven rails sit
// across it, 1.144 m apart, and ten modules lie between them, each with its
// two long edges on two rails, so neighbouring modules share a rail. A rail
// is a hat section, open side down, whose webs straddle the tube. It is
// located by a tab that drops into a hole on top of the tube and held by a
// strap that passes under the tube and is closed by one M8 bolt. A module is
// held by four pins with swaged collars (lockbolts), two into each of its
// rails, through the slots its frame has 400 mm apart.
//
// The frame. Everything here is in the tube's frame: the origin is on the
// tube's axis at the plain (not swaged) end of the section, +Y runs along
// the tube to the swaged end, +Z points up through the locating holes, and
// +X is across the tube. Metres, kilograms, radians, newton metres.
// models/bay.xml places this frame in the world as its body "bay_frame";
// where it puts it is the scene's choice and not a figure of the bay.
//
// A fastening is two points, one on each of the two parts it joins, each at
// the centre of that part's hole, and an axis: the direction the tool
// travels to drive the fastener. Point a is on the part nearer the tool. The
// model has a site at each point, named <name>_a and <name>_b, whose z axis
// is the tool's axis. In the assembled bay the two points lie on one line
// along that axis, a little apart, because each is in its own part's sheet:
// a fastening goes in when the two points are within the fastener's play of
// each other across the axis, and the distance along it is the stack.
//
// The strap's bolt passes through the strap's two ears under the tube and
// not through the tube, so the tube has no hole for a rail's fastening.
// Point a of such a fastening is at the mouth of the hole in the ear on the
// tool's side, and point b is the point of the tube's frame that the bolt's
// axis must pass through: on the tube's vertical mid-plane, under it.
//
// Two more kinds of point are not fastenings. A locator is a rail's tab (a)
// and the hole on the tube it drops into (b). A hold is a stretch of the
// tube that something may grip while the bay is assembled; the document's
// section "Holding the tube" says what a holder must do.

#ifndef SRC_BAY_HARDWARE_H_
#define SRC_BAY_HARDWARE_H_

#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace cell::bay {

// ---------------------------------------------------------------- the tube
inline constexpr double kTubeRadius = 0.062;  // 124 mm across, published
inline constexpr double kTubeWall = 0.003;
inline constexpr double kTubeMassPerMetre = 8.968;
// From the plain end to the shoulder of the swaged neck.
inline constexpr double kTubeLength = 11.660;
// The neck slides into the next section's plain end.
inline constexpr double kNeckLength = 0.100;
inline constexpr double kNeckRadius = 0.059;
inline constexpr double kNeckMassPerMetre = 8.523;
// One section as it is shipped, and so the bay's length.
inline constexpr double kSectionLength = kTubeLength + kNeckLength;
inline constexpr double kTubeMass =
    kTubeMassPerMetre * kTubeLength + kNeckMassPerMetre * kNeckLength;
// The stretch at the plain end that the next section's neck fills.
inline constexpr double kJointLength = 0.100;

// -------------------------------------------------------------- the module
// A module lies with its long side across the tube (X), its short side
// along it (Y) and its glass up (Z).
inline constexpr double kModuleLength = 2.382;
inline constexpr double kModuleWidth = 1.134;
inline constexpr double kModuleThickness = 0.030;
inline constexpr double kModuleMass = 32.4;
inline constexpr int kModuleCount = 10;
inline constexpr double kModuleGap = 0.010;
inline constexpr double kModulePitch = kModuleWidth + kModuleGap;
// The frame's bottom flange, which rests on the rail and takes the pins.
inline constexpr double kFlangeWidthLongSide = 0.028;
inline constexpr double kFlangeWidthShortSide = 0.0116;
inline constexpr double kFlangeThickness = 0.0017;
// The upright part of a frame member, as the model draws it.
inline constexpr double kFrameUprightWidth = 0.010;
inline constexpr double kLaminateThickness = 0.005;
inline constexpr double kLaminateMass = 27.0;
inline constexpr double kFrameMass = kModuleMass - kLaminateMass;
// The tracker's holes: two on each long side, this far apart along the
// side, their centres this far in from the long edge. The slots are 10 mm
// along the long side and 7 mm across it.
inline constexpr double kHolePairSpacing = 0.400;
inline constexpr double kHoleFromLongEdge = 0.019;
inline constexpr double kModuleSlotAcrossTube = 0.010;
inline constexpr double kModuleSlotAlongTube = 0.007;

// ---------------------------------------------------------------- the rail
inline constexpr int kRailCount = kModuleCount + 1;
// The first rail's centre along the tube; the others follow at the pitch.
inline constexpr double kFirstRailY = 0.160;
inline constexpr double kRailLength = 0.460;    // across the tube
inline constexpr double kRailTopWidth = 0.100;  // along the tube
inline constexpr double kRailDepth = 0.080;
inline constexpr double kRailSheet = 0.002;
inline constexpr double kRailFootWidth = 0.010;
// The rail's top face, on which the modules rest, above the tube's axis.
inline constexpr double kRailTopHeight = 0.124;
// The webs are cut to arcs of this radius where they sit on the tube.
inline constexpr double kSaddleRadius = 0.0625;
// The model draws the saddle as a step: within this distance of the tube's
// mid-plane the webs end on top of the tube, outside it they reach down to
// the feet.
inline constexpr double kSaddleHalfWidth = 0.044;
// Rail, strap, the strap's two pins and the bolt together.
inline constexpr double kRailMass = 2.35;
// The rail's slots for the pins: 7.1 mm across the tube, 13 mm along it.
inline constexpr double kRailSlotAcrossTube = 0.0071;
inline constexpr double kRailSlotAlongTube = 0.013;
// The strap, drawn as a square U under the tube with a block for its ears.
inline constexpr double kStrapWidth = 0.040;  // along the tube
inline constexpr double kStrapThickness = 0.0025;
inline constexpr double kStrapEarHalfWidth = 0.010;  // across the tube
// The lowest point of the bay: the ears' underside, below the tube's axis.
inline constexpr double kStrapEarBottom = -0.095;

// ----------------------------------------------- fastening: rail to tube
// One M8 bolt through the strap's ears, along X, under the tube. The tool
// comes from +X and travels toward -X.
inline constexpr double kStrapBoltHeight = -0.080;
inline constexpr double kStrapBoltDiameter = 0.008;
inline constexpr double kStrapBoltHole = 0.009;
inline constexpr double kStrapBoltTorque = 17.6;  // N m
inline constexpr double kStrapBoltFreeRadius = 0.012;
// Play of the bolt in its hole, each way across its axis.
inline constexpr double kStrapBoltPlay =
    (kStrapBoltHole - kStrapBoltDiameter) / 2.0;

// --------------------------------------------- fastening: module to rail
// A 1/4 in pin with a swaged collar, along Z. The tool comes from below the
// rail's top, inside the hat, and travels up. It is not torqued.
inline constexpr double kPinDiameter = 0.00635;
inline constexpr int kPinsPerModule = 4;
inline constexpr double kPinFreeRadius = 0.013;
inline constexpr double kPinToolLength = 0.080;
// How far a module may be from its place for its pins to go in, each alone:
// across the tube, along it, and in yaw (docs/bay-hardware.md, Tolerances).
inline constexpr double kModulePlayAcrossTube =
    (kRailSlotAcrossTube - kPinDiameter) / 2.0 +
    (kModuleSlotAcrossTube - kPinDiameter) / 2.0;  // 2.20 mm
inline constexpr double kModulePlayAlongTube =
    (kRailSlotAlongTube - kPinDiameter) / 2.0 +
    (kModuleSlotAlongTube - kPinDiameter) / 2.0;  // 3.65 mm
inline constexpr double kModulePlayYaw =
    kModulePlayAcrossTube /
    ((kModuleWidth - 2.0 * kHoleFromLongEdge) / 2.0);  // 4.0 mrad

// ------------------------------------------------------------- the locator
// The rail's tab, 12 mm across the tube and 2 mm along it, in a round hole
// of 14 mm on top of the tube. The tab is on the web nearer the plain end.
inline constexpr double kLocatingHoleDiameter = 0.014;
inline constexpr double kTabWidth = 0.012;
inline constexpr double kTabThickness = 0.002;
inline constexpr double kTabLength = 0.006;
inline constexpr double kTabOffsetY = -0.049;
// Play of the tab in its hole: sqrt(7^2 - 1^2) - 6 and sqrt(7^2 - 6^2) - 1,
// in millimetres.
inline constexpr double kTabPlayAcrossTube = 0.000928;
inline constexpr double kTabPlayAlongTube = 0.002606;
// How far a rail in its hole and on its saddle can be from its place: along
// the tube, round it, and in yaw. Round the tube is the tab's play across at
// the tube's radius; the yaw is the saddle's, at most.
inline constexpr double kRailPlayAlongTube = kTabPlayAlongTube;
inline constexpr double kRailPlayRound = kTabPlayAcrossTube / kTubeRadius;
inline constexpr double kRailPlayYaw = 0.0144;

// ------------------------------------------------------- holding the tube
// How far the tube may be turned from its place before a rail's tab misses
// its hole (0.86 degrees), the angle a holder must keep, just inside it, and
// the moment about the tube's axis a holder must resist.
inline constexpr double kTubeTurnLimit = kRailPlayRound;
inline constexpr double kHoldMoment = 150.0;                    // N m
inline constexpr double kHoldAngle = 0.8 * 3.14159265358979323846 / 180.0;
// The largest moment assembly puts on the tube: a module resting on one end
// of its rails.
inline constexpr double kLargestAssemblyMoment =
    kModuleMass * 9.81 * kRailLength / 2.0;  // 73 N m

// ----------------------------------------------------------------- the bay
inline constexpr int kRailFasteningCount = kRailCount;
inline constexpr int kModuleFasteningCount = kModuleCount * kPinsPerModule;
inline constexpr int kFasteningCount =
    kRailFasteningCount + kModuleFasteningCount;
inline constexpr int kLocatorCount = kRailCount;
inline constexpr int kHoldCount = kRailCount + 1;
inline constexpr double kBayMass =
    kTubeMass + kRailCount * kRailMass + kModuleCount * kModuleMass;
// The bay's size along X, Y and Z.
inline constexpr double kBayWidth = kModuleLength;
inline constexpr double kBayLength = kSectionLength;
inline constexpr double kBayHeight =
    kRailTopHeight + kModuleThickness - kStrapEarBottom;

// The corners of the box that holds the whole bay, in the tube's frame.
inline Eigen::Vector3d BayBoxMin() {
  return {-kModuleLength / 2.0, 0.0, kStrapEarBottom};
}
inline Eigen::Vector3d BayBoxMax() {
  return {kModuleLength / 2.0, kSectionLength,
          kRailTopHeight + kModuleThickness};
}

// ------------------------------------------------------------------ poses
// The names of the model's bodies.
inline std::string TubeName() { return "bay_tube"; }
inline std::string RailName(int rail) {
  return "bay_rail_" + std::to_string(rail);
}
inline std::string ModuleName(int module) {
  return "bay_module_" + std::to_string(module);
}

// Where rail `rail` (0 .. kRailCount - 1) stands along the tube.
inline double RailY(int rail) { return kFirstRailY + rail * kModulePitch; }

// A rail's frame: its origin on the tube's axis at the rail's place, its
// axes along the tube's. The rail's long side is along X.
inline Eigen::Isometry3d RailPose(int rail) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.0, RailY(rail), 0.0);
  return pose;
}

// A module's frame: its origin at the module's centre, its axes along the
// tube's. Module `module` (0 .. kModuleCount - 1) lies on rails `module` and
// `module + 1`.
inline Eigen::Isometry3d ModulePose(int module) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() =
      Eigen::Vector3d(0.0, RailY(module) + kModulePitch / 2.0,
                      kRailTopHeight + kModuleThickness / 2.0);
  return pose;
}

// The orientation of a site whose z axis is the given direction. The three
// directions the bay needs are fixed here, so that the model and the header
// mean the same frame and not only the same axis:
//   +Z (a pin):        the tube's own axes;
//   -X (a strap bolt): x = +Z, y = +Y, z = -X  (quaternion w x y z =
//                      0.70710678 0 -0.70710678 0);
//   -Z (a locator):    x = +X, y = -Y, z = -Z  (quaternion 0 1 0 0);
//   +Y (a hold):       x = +X, y = -Z, z = +Y  (quaternion
//                      0.70710678 -0.70710678 0 0).
inline Eigen::Matrix3d FrameAlongPlusZ() { return Eigen::Matrix3d::Identity(); }
inline Eigen::Matrix3d FrameAlongMinusX() {
  Eigen::Matrix3d r;
  r.col(0) = Eigen::Vector3d(0.0, 0.0, 1.0);
  r.col(1) = Eigen::Vector3d(0.0, 1.0, 0.0);
  r.col(2) = Eigen::Vector3d(-1.0, 0.0, 0.0);
  return r;
}
inline Eigen::Matrix3d FrameAlongMinusZ() {
  Eigen::Matrix3d r;
  r.col(0) = Eigen::Vector3d(1.0, 0.0, 0.0);
  r.col(1) = Eigen::Vector3d(0.0, -1.0, 0.0);
  r.col(2) = Eigen::Vector3d(0.0, 0.0, -1.0);
  return r;
}
inline Eigen::Matrix3d FrameAlongPlusY() {
  Eigen::Matrix3d r;
  r.col(0) = Eigen::Vector3d(1.0, 0.0, 0.0);
  r.col(1) = Eigen::Vector3d(0.0, 0.0, -1.0);
  r.col(2) = Eigen::Vector3d(0.0, 1.0, 0.0);
  return r;
}

inline Eigen::Isometry3d MakePose(const Eigen::Matrix3d& rotation,
                                  const Eigen::Vector3d& position) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation;
  pose.translation() = position;
  return pose;
}

// -------------------------------------------------------------- fastenings
enum class FasteningKind { kRail, kModule };

// "rail" or "module", as the sites' names and print_bay's rows have it.
inline const char* KindName(FasteningKind kind) {
  return kind == FasteningKind::kRail ? "rail" : "module";
}

struct Fastening {
  // bay_fasten_<kind>_<part>_<i>: <part> is the number of the rail or of the
  // module that is being fastened, <i> counts that part's fastenings from 0.
  // The model's sites are name + "_a" and name + "_b".
  std::string name;
  FasteningKind kind = FasteningKind::kRail;
  // The bodies the two points are on: a on the part nearer the tool.
  std::string part_a;
  std::string part_b;
  // The two points in the tube's frame. Each pose's z axis is tool_axis.
  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  // Unit vector: the way the tool travels to drive the fastener.
  Eigen::Vector3d tool_axis = Eigen::Vector3d::UnitZ();
  // The free space the tool needs round the axis, as a radius.
  double free_radius = 0.0;
  // How far apart the two points may be across the axis for the fastener
  // still to go in: the smallest play of the fastener in its holes.
  double clearance = 0.0;
  // How far point b is from point a along the tool's axis in the assembled
  // bay.
  double stack = 0.0;
};

// Every fastening of the bay: first the eleven of kind rail, by rail, then
// the forty of kind module, by module and within a module in the order
// (rail `module`, X = -0.2), (rail `module`, X = +0.2),
// (rail `module + 1`, X = -0.2), (rail `module + 1`, X = +0.2).
inline std::vector<Fastening> Fastenings() {
  std::vector<Fastening> all;
  all.reserve(kFasteningCount);
  for (int rail = 0; rail < kRailCount; ++rail) {
    Fastening f;
    f.name = "bay_fasten_rail_" + std::to_string(rail) + "_0";
    f.kind = FasteningKind::kRail;
    f.part_a = RailName(rail);
    f.part_b = TubeName();
    f.tool_axis = Eigen::Vector3d(-1.0, 0.0, 0.0);
    f.pose_a = MakePose(FrameAlongMinusX(),
                        {kStrapEarHalfWidth, RailY(rail), kStrapBoltHeight});
    f.pose_b =
        MakePose(FrameAlongMinusX(), {0.0, RailY(rail), kStrapBoltHeight});
    f.free_radius = kStrapBoltFreeRadius;
    f.clearance = kStrapBoltPlay;
    f.stack = kStrapEarHalfWidth;
    all.push_back(f);
  }
  for (int module = 0; module < kModuleCount; ++module) {
    for (int i = 0; i < kPinsPerModule; ++i) {
      const int rail = module + i / 2;
      const double x = (i % 2 == 0 ? -0.5 : 0.5) * kHolePairSpacing;
      // The hole is kHoleFromLongEdge inside the module's long edge, and
      // the edge is half a gap from the rail's centre line.
      const double inset = kModuleGap / 2.0 + kHoleFromLongEdge;
      const double y = RailY(rail) + (i / 2 == 0 ? inset : -inset);
      Fastening f;
      f.name = "bay_fasten_module_" + std::to_string(module) + "_" +
               std::to_string(i);
      f.kind = FasteningKind::kModule;
      f.part_a = RailName(rail);
      f.part_b = ModuleName(module);
      f.tool_axis = Eigen::Vector3d(0.0, 0.0, 1.0);
      // Each point is in the middle of its own sheet: the rail's top below
      // the resting face, the frame's flange above it.
      f.pose_a = MakePose(FrameAlongPlusZ(),
                          {x, y, kRailTopHeight - kRailSheet / 2.0});
      f.pose_b = MakePose(FrameAlongPlusZ(),
                          {x, y, kRailTopHeight + kFlangeThickness / 2.0});
      f.free_radius = kPinFreeRadius;
      f.clearance = kModulePlayAcrossTube;
      f.stack = kRailSheet / 2.0 + kFlangeThickness / 2.0;
      all.push_back(f);
    }
  }
  return all;
}

// ---------------------------------------------------------------- locators
struct Locator {
  // bay_locate_<rail>; the model's sites are name + "_a" on the rail (the
  // tab) and name + "_b" on the tube (the hole). Their z axis is -Z, the way
  // the tab goes in.
  std::string name;
  std::string part_a;
  std::string part_b;
  Eigen::Isometry3d pose_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pose_b = Eigen::Isometry3d::Identity();
  // The tab's smaller play in its hole.
  double clearance = 0.0;
};

inline std::vector<Locator> Locators() {
  std::vector<Locator> all;
  all.reserve(kLocatorCount);
  for (int rail = 0; rail < kRailCount; ++rail) {
    Locator l;
    l.name = "bay_locate_" + std::to_string(rail);
    l.part_a = RailName(rail);
    l.part_b = TubeName();
    // The hole's centre at the tube's surface, on top.
    const Eigen::Vector3d at(0.0, RailY(rail) + kTabOffsetY, kTubeRadius);
    l.pose_a = MakePose(FrameAlongMinusZ(), at);
    l.pose_b = MakePose(FrameAlongMinusZ(), at);
    l.clearance = kTabPlayAcrossTube;
    all.push_back(l);
  }
  return all;
}

// ------------------------------------------------------------------- holds
struct Hold {
  // bay_hold_<i>: a site on the tube's body, a cylinder as long as the
  // stretch, its z axis along the tube (+Y).
  std::string name;
  // The middle of the stretch, on the tube's axis.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  // The tube's radius there, and half the stretch's length.
  double radius = 0.0;
  double half_length = 0.0;
  // True where modules cover the upper half, so that only the lower half of
  // the tube can be reached.
  bool lower_half_only = false;
};

// The twelve stretches a holder may grip: the plain end, the ten stretches
// between neighbouring rails, and the neck.
inline std::vector<Hold> Holds() {
  std::vector<Hold> all;
  all.reserve(kHoldCount);
  auto add = [&all](double y_from, double y_to, double radius, bool lower) {
    Hold h;
    h.name = "bay_hold_" + std::to_string(all.size());
    h.pose = MakePose(FrameAlongPlusY(), {0.0, (y_from + y_to) / 2.0, 0.0});
    h.radius = radius;
    h.half_length = (y_to - y_from) / 2.0;
    h.lower_half_only = lower;
    all.push_back(h);
  };
  add(0.0, kJointLength, kTubeRadius, false);
  for (int rail = 0; rail + 1 < kRailCount; ++rail) {
    add(RailY(rail) + kRailTopWidth / 2.0,
        RailY(rail + 1) - kRailTopWidth / 2.0, kTubeRadius, true);
  }
  add(kTubeLength, kSectionLength, kNeckRadius, false);
  return all;
}

}  // namespace cell::bay

#endif  // SRC_BAY_HARDWARE_H_
