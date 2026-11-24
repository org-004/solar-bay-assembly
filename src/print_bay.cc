// Prints the bay's fastening points, as src/bay_hardware.h has them, or the
// bay's counts, size and mass.
//
//   bazel run --config=opt //:print_bay
//   bazel run --config=opt //:print_bay -- --summary
//
// Without an argument: CSV, one row for each fastening in the order of
// cell::bay::Fastenings(): its name without the _a / _b of its sites, its
// kind, the two parts it joins, point a in the tube's frame (metres), the
// way the tool travels, and the free radius the tool needs (metres).
// With --summary: key,value lines, every value computed from the header's
// constants. The model, models/bay.xml, is not read.

#include <cstdio>
#include <cstring>

#include <Eigen/Core>

#include "src/bay_hardware.h"

namespace {

// A value printed with `decimals` decimals, without a minus sign on zero.
double NoNegativeZero(double value, int decimals) {
  double scale = 1.0;
  for (int i = 0; i < decimals; ++i) scale *= 10.0;
  return (value * scale > -0.5 && value * scale < 0.5) ? 0.0 : value;
}

void PrintFastenings() {
  std::printf("name,kind,part_a,part_b,x,y,z,ax,ay,az,free_radius\n");
  for (const cell::bay::Fastening& f : cell::bay::Fastenings()) {
    const Eigen::Vector3d a = f.pose_a.translation();
    std::printf("%s,%s,%s,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f\n",
                f.name.c_str(), cell::bay::KindName(f.kind), f.part_a.c_str(),
                f.part_b.c_str(), NoNegativeZero(a.x(), 6),
                NoNegativeZero(a.y(), 6), NoNegativeZero(a.z(), 6),
                NoNegativeZero(f.tool_axis.x(), 6),
                NoNegativeZero(f.tool_axis.y(), 6),
                NoNegativeZero(f.tool_axis.z(), 6), f.free_radius);
  }
}

void PrintSummary() {
  using namespace cell::bay;
  std::printf("modules,%d\n", kModuleCount);
  std::printf("rails,%d\n", kRailCount);
  std::printf("fastenings_rail,%d\n", kRailFasteningCount);
  std::printf("fastenings_module,%d\n", kModuleFasteningCount);
  std::printf("fastenings,%d\n", kFasteningCount);
  std::printf("locating_holes,%d\n", kLocatorCount);
  std::printf("holds,%d\n", kHoldCount);
  std::printf("length_m,%.3f\n", kBayLength);
  std::printf("width_m,%.3f\n", kBayWidth);
  std::printf("height_m,%.3f\n", kBayHeight);
  std::printf("mass_kg,%.1f\n", kBayMass);
  std::printf("module_pitch_m,%.3f\n", kModulePitch);
  std::printf("module_gap_m,%.3f\n", kModuleGap);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 1) {
    PrintFastenings();
    return 0;
  }
  if (argc == 2 && std::strcmp(argv[1], "--summary") == 0) {
    PrintSummary();
    return 0;
  }
  std::fprintf(stderr, "usage: %s [--summary]\n", argv[0]);
  return 1;
}
