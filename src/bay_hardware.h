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
// The bay. The tube is a round steel tube 124 mm across. A bay has
// kModuleCount modules (nine, by the owner's decision of 2026-11-24) and one
// rail more than that, ten, and the number of modules is the only count that
// is set: every other count, and the tube's length, follow from it. The rails
// sit across the tube 1.144 m apart, and a module lies between each two of
// them, with its two long edges on two rails, so neighbouring modules share
// a rail, and the two end rails carry one module each. A rail
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
//
// The locating move (docs/bay-hardware.md, "Locating a module before
// pinning"). By the owner's decision of 2026-11-24 a module is laid on its
// rails within the cell's placement tolerance (the gate: 3 mm in the rails'
// plane in any direction and 0.15 degrees of yaw, kGateOffset and kGateYaw)
// and then moved into its holes before it is pinned, because the gate
// accepts placements the pins alone do not (Tolerances). A hardened nose on
// a floating mount comes up from below, inside the hat, along a hole's axis:
// its ball tip finds the opening that the rail's slot and the frame's slot
// leave between them, by contact, and its taper then slides the module
// until the land is in both slots, which leaves the hole inside the pin's
// play. The nose leaves the hole, a pin is put in from above and left loose,
// and the next hole is located; the holes are taken in the order
// kLocateOrder and the collars swaged in kSwageOrder once all four pins are
// in. Both slots are read as obrounds (two half circles joined by straight
// sides), the cautious reading, so a slot is the set of points within half
// its width of a segment, and a round thing of diameter d passes through two
// slots when their two segments are no further apart than the sum of the
// slots' half widths less d (Opening). The functions after the constants
// compute the capture from a LocatingDims, which defaults to the constants,
// so that a test can ask the same of other slots and noses. Placements and
// hole errors are in the tube's frame, metres and radians, relative to a
// module's place: a hole's error is where its centre is less where it
// should be. Every figure of the move is `ours`: designed here.

#ifndef SRC_BAY_HARDWARE_H_
#define SRC_BAY_HARDWARE_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace cell::bay {

// ---------------------------------------------------------------- the tube
inline constexpr double kTubeRadius = 0.062;  // 124 mm across, published
inline constexpr double kTubeWall = 0.003;
inline constexpr double kTubeMassPerMetre = 8.968;
// The neck slides into the next section's plain end.
inline constexpr double kNeckLength = 0.100;
inline constexpr double kNeckRadius = 0.059;
inline constexpr double kNeckMassPerMetre = 8.523;
// The stretch at the plain end that the next section's neck fills.
inline constexpr double kJointLength = 0.100;

// -------------------------------------------------------------- the module
// A module lies with its long side across the tube (X), its short side
// along it (Y) and its glass up (Z).
inline constexpr double kModuleLength = 2.382;
inline constexpr double kModuleWidth = 1.134;
inline constexpr double kModuleThickness = 0.030;
inline constexpr double kModuleMass = 32.4;
// The one count that is set; the rails, the fastenings and the length of the
// tube follow from it.
inline constexpr int kModuleCount = 9;
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
inline constexpr double kRailLength = 0.460;    // across the tube
inline constexpr double kRailTopWidth = 0.100;  // along the tube
inline constexpr double kRailDepth = 0.080;
inline constexpr double kRailSheet = 0.002;
inline constexpr double kRailFootWidth = 0.010;
// How far a rail's feet reach beyond its centre along the tube: half the top's
// width and the foot, 60 mm.
inline constexpr double kRailEndMargin = kRailTopWidth / 2.0 + kRailFootWidth;
// The first rail's centre along the tube: the joint, then the rail's margin.
// The others follow at the module pitch.
inline constexpr double kFirstRailY = kJointLength + kRailEndMargin;
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

// ------------------------------------------------------ the section's length
// From the plain end to the shoulder of the swaged neck: the joint, the first
// rail, the other rails a pitch apart, and the last rail's margin. With the
// nine modules that is 10.516 m.
inline constexpr double kTubeLength =
    kFirstRailY + (kRailCount - 1) * kModulePitch + kRailEndMargin;
// One section as it is shipped, and so the bay's length: 10.616 m.
inline constexpr double kSectionLength = kTubeLength + kNeckLength;
inline constexpr double kTubeMass =
    kTubeMassPerMetre * kTubeLength + kNeckMassPerMetre * kNeckLength;

// ----------------------------------------------- fastening: rail to tube
// One M8 bolt through the strap's ears, along X, under the tube. The tool
// comes from +X and travels toward -X.
inline constexpr double kStrapBoltHeight = -0.080;
inline constexpr double kStrapBoltDiameter = 0.008;
inline constexpr double kStrapBoltHole = 0.009;
inline constexpr double kStrapBoltTorque = 17.6;  // N m
inline constexpr double kStrapBoltFreeRadius = 0.012;
// How far out from the ear's mouth, along the bolt's axis, that radius is
// kept free: the length of socket and extension a tool must have before its
// body begins. Ours; the reference gives none.
inline constexpr double kStrapBoltFreeLength = 0.100;
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

// Every fastening of the bay: first those of kind rail, one for each rail,
// then those of kind module, four for each module, by module and within a
// module in the order
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

// The stretches a holder may grip, kHoldCount of them: the plain end, one
// between each two neighbouring rails, and the neck.
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

// ------------------------------------------------------ the locating move
// The module's own hole tolerance: the two holes of a pair are 400 +/- 1
// apart and the two rows 1,096 +/- 2, so each hole can be this far from its
// place across and along the tube (Tolerances, derived).
inline constexpr double kHoleToleranceAcrossTube = 0.0005;
inline constexpr double kHoleToleranceAlongTube = 0.0010;

// What the gate accepts: an offset in the rails' plane of this much in any
// direction and this much yaw. They repeat cell::PlacementTolerance's
// `position` and `yaw` (src/trial_runner.h, the owner's figures); this
// header stands alone and does not include it.
inline constexpr double kGateOffset = 0.003;
inline constexpr double kGateYaw = 0.15 * 3.14159265358979323846 / 180.0;

// The free height above the frame's flange, under the laminate: the frame's
// height less the flange and the laminate, 23.3 mm.
inline constexpr double kFreeAboveFlange =
    kModuleThickness - kFlangeThickness - kLaminateThickness;

// The nose, hardened steel, on a mount that floats across and along (ours).
// A ball tip 2 mm across, small enough to find a slot of 7.1 x 13 from below
// with +/-2.55 across and +/-5.5 along to aim in.
inline constexpr double kNoseTipDiameter = 0.0020;
// The full diameter: more than the pin's 6.35, so a located hole is inside
// the pin's play with 0.25 to spare, and less than the frame slot's 7.0.
inline constexpr double kNoseDiameter = 0.0066;
// The taper from the tip's diameter to the full one. Its half angle,
// atan(2.3 / 8.6), about 15 degrees, is NoseHalfAngle().
inline constexpr double kNoseTaperLength = 0.0086;
// The straight land of the full diameter: longer than the two sheets (3.7)
// and the over-travel together.
inline constexpr double kNoseLandLength = 0.0060;
// Tip (its radius), taper and land: 15.6 mm.
inline constexpr double kNoseLength =
    kNoseTipDiameter / 2.0 + kNoseTaperLength + kNoseLandLength;
// "Fully in": the land's upper end this far above the flange's top face.
inline constexpr double kNoseOverTravel = 0.0010;
// How high the tip then stands above the flange's top face: 10.6 mm.
inline constexpr double kNoseRise =
    kNoseOverTravel + kNoseTaperLength + kNoseTipDiameter / 2.0;
// The tip enters when the opening is at least the tip and this.
inline constexpr double kNoseTipClearance = 0.0005;
// Static friction of the frame on the rail and of the nose on the slots'
// edges: the handbook figure for aluminium on mild steel, clean and dry,
// applied to anodised aluminium on galvanised steel (ours).
inline constexpr double kLocateFriction = 0.61;
// The rated thrust of the nose's tool along its axis, N. The tool bears on
// the rail; the arm carries the tool, not the thrust.
inline constexpr double kNoseToolThrust = 300.0;
// The free space the move needs round each hole's axis. Below the rail's
// top: the swaging tool's 13 and the 5.5 the tip's centre travels along the
// slot, rounded up, and the swaging tool's 80 of length.
inline constexpr double kLocateFreeRadiusBelow = 0.019;
inline constexpr double kLocateFreeLengthBelow = 0.080;
// Above the flange: the tip and the taper, which stay within the rail's
// slot's outline (6.5 from the axis), and kNoseRise.
inline constexpr double kLocateFreeRadiusAbove = 0.007;
inline constexpr double kLocateFreeHeightAbove = 0.011;
// The order in which a module's holes are located and pinned (pin index as
// in Fastenings()), and the order in which the collars are swaged once all
// four pins are in: the way back.
inline constexpr std::array<int, 4> kLocateOrder = {0, 2, 3, 1};
inline constexpr std::array<int, 4> kSwageOrder = {1, 3, 2, 0};

// The dimensions the capture arithmetic reads, the constants by default.
// Slots are obrounds: `across` is the size across the tube (X), `along`
// along it (Y).
struct LocatingDims {
  double rail_slot_across = kRailSlotAcrossTube;
  double rail_slot_along = kRailSlotAlongTube;
  double frame_slot_across = kModuleSlotAcrossTube;
  double frame_slot_along = kModuleSlotAlongTube;
  double pin = kPinDiameter;
  double tip = kNoseTipDiameter;
  double nose = kNoseDiameter;
  double taper_length = kNoseTaperLength;
  double clearance = kNoseTipClearance;
  double hole_tolerance_across = kHoleToleranceAcrossTube;
  double hole_tolerance_along = kHoleToleranceAlongTube;
  double friction = kLocateFriction;
};

// A module's placement: its centre's offset from its place, and its yaw.
struct ModulePlacement {
  double dx = 0.0;
  double dy = 0.0;
  double yaw = 0.0;
};

// Hole `hole`'s place in the module's frame: X = -/+200, Y = -/+548.
inline Eigen::Vector2d HolePlace(int hole) {
  return {(hole % 2 == 0 ? -0.5 : 0.5) * kHolePairSpacing,
          (hole / 2 == 0 ? -1.0 : 1.0) *
              (kModuleWidth / 2.0 - kHoleFromLongEdge)};
}

// Hole `hole`'s error for a placement, with its own tolerance taken at
// `sign_across` and `sign_along` (each -1, 0 or +1) of its limits:
// (dx - yaw y + tx, dy + yaw x + ty). The yaw is small (the gate's is 2.6
// mrad), so the turn is taken to first order, as the design has it; the
// second-order terms are under 0.003 mm.
inline Eigen::Vector2d HoleError(const ModulePlacement& p, int hole,
                                 int sign_across, int sign_along,
                                 const LocatingDims& d = {}) {
  const Eigen::Vector2d place = HolePlace(hole);
  return {p.dx - p.yaw * place.y() + sign_across * d.hole_tolerance_across,
          p.dy + p.yaw * place.x() + sign_along * d.hole_tolerance_along};
}
inline Eigen::Vector2d HoleError(double dx, double dy, double yaw, int hole,
                                 int sign_across, int sign_along,
                                 const LocatingDims& d = {}) {
  return HoleError(ModulePlacement{dx, dy, yaw}, hole, sign_across,
                   sign_along, d);
}

// Half the straight part of each slot's centre line, across and along: the
// rail's (0, 2.95 mm) and the frame's (1.5 mm, 0).
inline Eigen::Vector2d HalfSegment(double across, double along) {
  return across > along ? Eigen::Vector2d((across - along) / 2.0, 0.0)
                        : Eigen::Vector2d(0.0, (along - across) / 2.0);
}
// How far the two slots' centre lines are apart for a hole error (ex, ey):
// sqrt(max(0, |ex| - 1.5)^2 + max(0, |ey| - 2.95)^2) with the constants.
inline Eigen::Vector2d SegmentExcess(double ex, double ey,
                                     const LocatingDims& d = {}) {
  const Eigen::Vector2d h =
      HalfSegment(d.rail_slot_across, d.rail_slot_along) +
      HalfSegment(d.frame_slot_across, d.frame_slot_along);
  return {std::copysign(std::max(0.0, std::abs(ex) - h.x()), ex),
          std::copysign(std::max(0.0, std::abs(ey) - h.y()), ey)};
}
inline double SegmentGap(double ex, double ey, const LocatingDims& d = {}) {
  return SegmentExcess(ex, ey, d).norm();
}
// The slots' half widths together: 7.05 mm.
inline double HalfWidths(const LocatingDims& d = {}) {
  return (std::min(d.rail_slot_across, d.rail_slot_along) +
          std::min(d.frame_slot_across, d.frame_slot_along)) /
         2.0;
}

// The diameter of the largest round thing that passes through both slots
// at a hole error (ex, ey): min(7.0, 7.1, 7.05 - SegmentGap).
inline double Opening(double ex, double ey, const LocatingDims& d = {}) {
  return std::min({d.rail_slot_across, d.rail_slot_along, d.frame_slot_across,
                   d.frame_slot_along, HalfWidths(d) - SegmentGap(ex, ey, d)});
}

// The least opening the tip enters: tip and clearance, 2.5 mm.
inline double TipNeeds(const LocatingDims& d = {}) {
  return d.tip + d.clearance;
}
inline bool TipEnters(double ex, double ey, const LocatingDims& d = {}) {
  return Opening(ex, ey, d) >= TipNeeds(d);
}

// The largest SegmentGap at which a round thing of diameter `diameter`
// passes: 4.55 mm for the tip, 0.45 for the nose, 0.70 for the pin.
inline double GapFor(double diameter, const LocatingDims& d = {}) {
  return HalfWidths(d) - diameter;
}
// The largest hole error, across alone and along alone, that something of
// `diameter` passes: the capture (6.05, 7.50) for the tip and its
// clearance, what the nose leaves (1.95, 3.40), the pin's play (2.20,
// 3.65).
inline Eigen::Vector2d ErrorLimitFor(double diameter,
                                     const LocatingDims& d = {}) {
  const Eigen::Vector2d h =
      HalfSegment(d.rail_slot_across, d.rail_slot_along) +
      HalfSegment(d.frame_slot_across, d.frame_slot_along);
  return h + Eigen::Vector2d::Constant(GapFor(diameter, d));
}
inline Eigen::Vector2d CaptureLimit(const LocatingDims& d = {}) {
  return ErrorLimitFor(TipNeeds(d), d);
}
inline Eigen::Vector2d NoseLeaves(const LocatingDims& d = {}) {
  return ErrorLimitFor(d.nose, d);
}
inline Eigen::Vector2d PinPlay(const LocatingDims& d = {}) {
  return ErrorLimitFor(d.pin, d);
}

// True if the nose fits both slots and is not smaller than the pin, so that
// a hole it leaves is inside the pin's play.
inline bool NoseFits(const LocatingDims& d = {}) {
  return d.nose < std::min({d.rail_slot_across, d.rail_slot_along,
                            d.frame_slot_across, d.frame_slot_along}) &&
         d.nose >= d.pin && d.tip < d.nose;
}

// Where a hole with error (ex, ey) is once the nose is fully in: the
// nearest error at which the nose's full diameter passes both slots (the
// module slides no further than the nose makes it). An error the nose
// already passes is left as it is.
inline Eigen::Vector2d LocatedHole(double ex, double ey,
                                   const LocatingDims& d = {}) {
  const Eigen::Vector2d excess = SegmentExcess(ex, ey, d);
  const double gap = excess.norm();
  const double allowed = GapFor(d.nose, d);
  if (gap <= allowed) return {ex, ey};
  const Eigen::Vector2d keep = excess * (allowed / gap);
  return Eigen::Vector2d(ex, ey) - excess + keep;
}
// True if a hole error is inside the pin's play.
inline bool InsidePinPlay(double ex, double ey, const LocatingDims& d = {}) {
  return SegmentGap(ex, ey, d) <= GapFor(d.pin, d) + 1e-12;
}
// How far the module slides at a hole: its error less where it is located.
inline Eigen::Vector2d Slide(double ex, double ey,
                             const LocatingDims& d = {}) {
  return Eigen::Vector2d(ex, ey) - LocatedHole(ex, ey, d);
}

// The corners of what the gate accepts: placements at `offset` in
// directions 0, 360/steps, ... degrees (steps a multiple of 4 has the exact
// axes), yaw +/-`yaw`, and every combination of tolerance signs of each
// hole. A placement inside the disc is never worse: SegmentGap is convex in
// the hole error.
inline constexpr int kRimSteps = 3600;
inline ModulePlacement RimPlacement(double offset, double yaw, int step,
                                    int steps = kRimSteps) {
  const double angle = 2.0 * 3.14159265358979323846 * step / steps;
  // The exact axes without rounding.
  double c = std::cos(angle), s = std::sin(angle);
  if (4 * step % steps == 0) {
    const int quarter = (4 * step / steps) % 4;
    c = quarter == 0 ? 1.0 : (quarter == 2 ? -1.0 : 0.0);
    s = quarter == 1 ? 1.0 : (quarter == 3 ? -1.0 : 0.0);
  }
  return {offset * c, offset * s, yaw};
}

// The worst case of one hole over what the gate accepts.
struct GateWorst {
  // The least opening and where it occurs.
  double opening = 1e9;
  ModulePlacement placement;
  double direction_deg = 0.0;  // of the offset, from +X toward +Y
  int hole = 0;
  int sign_across = 0;
  int sign_along = 0;
  Eigen::Vector2d error = Eigen::Vector2d::Zero();
  // The largest hole error across and along, each over all cases.
  double error_across = 0.0;
  double error_along = 0.0;
};
inline GateWorst WorstOverGate(double offset = kGateOffset,
                               double yaw = kGateYaw,
                               const LocatingDims& d = {},
                               int steps = kRimSteps) {
  GateWorst w;
  for (int step = 0; step < steps; ++step) {
    for (int ys : {1, -1}) {
      const ModulePlacement p = RimPlacement(offset, ys * yaw, step, steps);
      for (int hole = 0; hole < 4; ++hole) {
        for (int sa : {1, -1}) {
          for (int sl : {1, -1}) {
            const Eigen::Vector2d e = HoleError(p, hole, sa, sl, d);
            w.error_across = std::max(w.error_across, std::abs(e.x()));
            w.error_along = std::max(w.error_along, std::abs(e.y()));
            const double o = Opening(e.x(), e.y(), d);
            if (o < w.opening - 1e-12) {
              w.opening = o;
              w.placement = p;
              w.direction_deg = 360.0 * step / steps;
              w.hole = hole;
              w.sign_across = sa;
              w.sign_along = sl;
              w.error = e;
            }
          }
        }
      }
    }
  }
  return w;
}
// True if the tip enters every hole of every placement the gate accepts.
inline bool CapturesGate(double offset = kGateOffset, double yaw = kGateYaw,
                         const LocatingDims& d = {}) {
  return WorstOverGate(offset, yaw, d).opening >= TipNeeds(d);
}

// The capture range in the gate's terms: the largest offset in any
// direction at yaw `yaw`, and the largest yaw at offset `offset`, with the
// hole tolerance against them, for which the tip still enters every hole.
// Found by halving, to well under a micrometre and a microradian.
inline double CaptureOffsetAtYaw(double yaw = kGateYaw,
                                 const LocatingDims& d = {},
                                 int steps = kRimSteps) {
  double lo = 0.0, hi = 0.05;
  if (WorstOverGate(lo, yaw, d, steps).opening < TipNeeds(d)) return 0.0;
  for (int i = 0; i < 40; ++i) {
    const double mid = (lo + hi) / 2.0;
    (WorstOverGate(mid, yaw, d, steps).opening >= TipNeeds(d) ? lo : hi) = mid;
  }
  return lo;
}
inline double CaptureYawAtOffset(double offset = kGateOffset,
                                 const LocatingDims& d = {},
                                 int steps = kRimSteps) {
  double lo = 0.0, hi = 0.05;
  if (WorstOverGate(offset, lo, d, steps).opening < TipNeeds(d)) return 0.0;
  for (int i = 0; i < 40; ++i) {
    const double mid = (lo + hi) / 2.0;
    (WorstOverGate(offset, mid, d, steps).opening >= TipNeeds(d) ? lo : hi) =
        mid;
  }
  return lo;
}

// How the module moves when the nose locates a hole on one rail.
enum class LocateMotion {
  // The whole module shifts.
  kShift,
  // The module turns about the middle of its other rail (that rail's
  // friction holds) until the hole's error across is taken up, and shifts
  // along the tube for what is left along.
  kTurn,
};
inline const char* MotionName(LocateMotion m) {
  return m == LocateMotion::kShift ? "shift" : "turn";
}

// The placement after the nose has located hole `hole` (its tolerance
// signs `sign_across`, `sign_along`) by `motion`: that hole's error is then
// LocatedHole of what it was.
inline ModulePlacement AfterLocating(const ModulePlacement& p, int hole,
                                     int sign_across, int sign_along,
                                     LocateMotion motion,
                                     const LocatingDims& d = {}) {
  const Eigen::Vector2d e = HoleError(p, hole, sign_across, sign_along, d);
  const Eigen::Vector2d target = LocatedHole(e.x(), e.y(), d);
  if (motion == LocateMotion::kShift) {
    const Eigen::Vector2d s = e - target;
    return {p.dx - s.x(), p.dy - s.y(), p.yaw};
  }
  // The pivot: the module's point over the middle of its other rail, at
  // `rail_y` from the module's centre. Turning by phi about it moves the
  // centre by phi x (rail_y, 0) and a hole at (x, y) by
  // phi x (rail_y - y, x), to first order; phi takes up the hole's error
  // across, and a shift along takes up what is left along.
  const Eigen::Vector2d place = HolePlace(hole);
  const double rail_y = (place.y() < 0.0 ? 1.0 : -1.0) *
                        (kModuleWidth / 2.0 + kModuleGap / 2.0);
  const double phi = (target.x() - e.x()) / (rail_y - place.y());
  const double along_left = target.y() - (e.y() + phi * place.x());
  return {p.dx + phi * rail_y, p.dy + along_left, p.yaw + phi};
}

// The worst of the holes not yet located once hole kLocateOrder[0] has
// been located by `motion`, over what the gate accepts.
struct LaterHoles {
  double error_across = 0.0;
  double error_along = 0.0;
  double opening = 1e9;  // the least
  ModulePlacement placement;  // where the least opening occurs
  int hole = 0;
  bool captured = true;
};
inline LaterHoles LaterHolesAfterFirst(LocateMotion motion,
                                       double offset = kGateOffset,
                                       double yaw = kGateYaw,
                                       const LocatingDims& d = {},
                                       int steps = kRimSteps) {
  LaterHoles w;
  const int first = kLocateOrder[0];
  for (int step = 0; step < steps; ++step) {
    for (int ys : {1, -1}) {
      const ModulePlacement p = RimPlacement(offset, ys * yaw, step, steps);
      for (int sa : {1, -1}) {
        for (int sl : {1, -1}) {
          const ModulePlacement q = AfterLocating(p, first, sa, sl, motion, d);
          for (int hole = 0; hole < 4; ++hole) {
            if (hole == first) continue;
            for (int ha : {1, -1}) {
              for (int hl : {1, -1}) {
                const Eigen::Vector2d e = HoleError(q, hole, ha, hl, d);
                w.error_across = std::max(w.error_across, std::abs(e.x()));
                w.error_along = std::max(w.error_along, std::abs(e.y()));
                const double o = Opening(e.x(), e.y(), d);
                if (o < w.opening) {
                  w.opening = o;
                  w.placement = p;
                  w.hole = hole;
                }
                if (o < TipNeeds(d)) w.captured = false;
              }
            }
          }
        }
      }
    }
  }
  return w;
}

// The largest slide of the module at hole kLocateOrder[0] over what the
// gate accepts: across, along and in all (not together).
inline Eigen::Vector3d WorstFirstSlide(double offset = kGateOffset,
                                       double yaw = kGateYaw,
                                       const LocatingDims& d = {},
                                       int steps = kRimSteps) {
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  for (int step = 0; step < steps; ++step) {
    for (int ys : {1, -1}) {
      const ModulePlacement p = RimPlacement(offset, ys * yaw, step, steps);
      for (int sa : {1, -1}) {
        for (int sl : {1, -1}) {
          const Eigen::Vector2d e = HoleError(p, kLocateOrder[0], sa, sl, d);
          const Eigen::Vector2d s = Slide(e.x(), e.y(), d);
          w = w.cwiseMax(Eigen::Vector3d(std::abs(s.x()), std::abs(s.y()),
                                         s.norm()));
        }
      }
    }
  }
  return w;
}

// The capture takes each rail's slot at its nominal place: the gate's
// offset is measured from the rails as they lie (what the cell senses;
// PlacementTolerance is "in the rails' plane"), so a rail's own play is not
// in a hole's error. If the gate were measured from the tube's nominal
// frame instead, a rail's play along the tube and its yaw (at the holes,
// 200 from the tube's axis) would add to a hole's error along the tube.
inline double RailPlayAlongAtHole() {
  return kRailPlayAlongTube + kRailPlayYaw * kHolePairSpacing / 2.0;
}
// The largest hole error along the tube over what the gate accepts, with
// the rail's play added: what it would be from the tube's nominal frame.
inline double HoleErrorAlongFromTubeFrame(const LocatingDims& d = {}) {
  return WorstOverGate(kGateOffset, kGateYaw, d).error_along +
         RailPlayAlongAtHole();
}

// The forces, newtons. The module's weight, 32.4 x 9.81.
inline double ModuleWeight() { return kModuleMass * 9.81; }
// The taper's half angle: atan(2.3 / 8.6).
inline double NoseHalfAngle(const LocatingDims& d = {}) {
  return std::atan((d.nose - d.tip) / 2.0 / d.taper_length);
}
// Sliding the whole module on its two rails, and turning it about its other
// rail (half its weight on the rail where it slides).
inline double SlideForce(const LocatingDims& d = {}) {
  return d.friction * ModuleWeight();
}
inline double TurnForce(const LocatingDims& d = {}) {
  return SlideForce(d) / 2.0;
}
// The taper lifts the frame's edge as it pushes it: lift = push x
// tan(half angle + atan(friction)).
inline double LiftPerPush(const LocatingDims& d = {}) {
  return std::tan(NoseHalfAngle(d) + std::atan(d.friction));
}
// The push across that slides the whole module, the lift taking weight off
// the rails: H = mu W / (1 + mu x LiftPerPush); and that lift.
inline double NosePush(const LocatingDims& d = {}) {
  return SlideForce(d) / (1.0 + d.friction * LiftPerPush(d));
}
inline double NoseLift(const LocatingDims& d = {}) {
  return NosePush(d) * LiftPerPush(d);
}
// The friction (the same on the rails and on the nose's faces) above which
// the nose would lift the frame's edge instead of sliding the module: where
// NoseLift reaches half the module's weight. NoseLift grows with the
// friction, so it is found by halving.
inline double LiftLimitFriction(const LocatingDims& d = {}) {
  LocatingDims t = d;
  double lo = 0.0, hi = 10.0;
  for (int i = 0; i < 60; ++i) {
    t.friction = (lo + hi) / 2.0;
    (NoseLift(t) < ModuleWeight() / 2.0 ? lo : hi) = t.friction;
  }
  return lo;
}
// The thrust along the nose's axis, an upper figure: wedged between the two
// slots, the rail's side is taken to push back as hard as the frame's, so
// the lift twice. Not a computed force.
inline double NoseThrust(const LocatingDims& d = {}) {
  return 2.0 * NoseLift(d);
}

}  // namespace cell::bay

#endif  // SRC_BAY_HARDWARE_H_
