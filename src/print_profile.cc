// Prints a trapezoid and an S-curve for the same move, one row per time step.
// The output is CSV, for a plotting tool. With --pretty it is a table of
// aligned columns, for reading in a terminal.
//
//   bazel run //:print_profile
//   bazel run //:print_profile -- --pretty
//   bazel run //:print_profile -- <distance> <velocity> <acceleration> <jerk>
//
// With no numbers the move is a quarter turn of one joint.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <vector>

#include "src/profile.h"

int main(int argc, char** argv) {
  double distance = 1.5708;  // rad
  cell::Limits limits;
  limits.velocity = 2.0;       // rad/s
  limits.acceleration = 5.0;   // rad/s^2
  limits.jerk = 50.0;          // rad/s^3

  // --pretty may appear anywhere. What is left must be no numbers or four.
  bool pretty = false;
  std::vector<const char*> numbers;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--pretty") == 0) {
      pretty = true;
    } else {
      numbers.push_back(argv[i]);
    }
  }

  if (numbers.size() == 4) {
    distance = std::atof(numbers[0]);
    limits.velocity = std::atof(numbers[1]);
    limits.acceleration = std::atof(numbers[2]);
    limits.jerk = std::atof(numbers[3]);
  } else if (!numbers.empty()) {
    std::fprintf(stderr,
                 "usage: %s [--pretty] "
                 "[<distance> <velocity> <acceleration> <jerk>]\n",
                 argv[0]);
    return 1;
  }

  const std::optional<cell::Profile> trapezoid = cell::Profile::Trapezoid(
      distance, limits.velocity, limits.acceleration);
  const std::optional<cell::Profile> s_curve =
      cell::Profile::SCurve(distance, limits);
  if (!trapezoid || !s_curve) {
    std::fprintf(stderr, "limits must be positive numbers\n");
    return 1;
  }

  std::fprintf(stderr, "trapezoid %.4f s, S-curve %.4f s\n",
               trapezoid->duration(), s_curve->duration());

  // The pretty table pads columns to fixed widths so they line up, with the
  // two shapes side by side and a bar between them.
  const char* row_format = "%.5f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n";
  if (pretty) {
    row_format = "%8.5f %11.6f %11.6f %11.6f | %11.6f %11.6f %11.6f %11.6f\n";
    std::printf("%8s %-35s | %s\n", "", "trapezoid", "S-curve");
    std::printf("%8s %11s %11s %11s | %11s %11s %11s %11s\n", "time", "pos",
                "vel", "acc", "pos", "vel", "acc", "jerk");
  } else {
    std::printf("time,trapezoid_position,trapezoid_velocity,"
                "trapezoid_acceleration,scurve_position,scurve_velocity,"
                "scurve_acceleration,scurve_jerk\n");
  }
  const int rows = 100;
  const double end = std::max(trapezoid->duration(), s_curve->duration());
  for (int i = 0; i <= rows; ++i) {
    const double t = end * i / rows;
    const cell::Sample a = trapezoid->At(t);
    const cell::Sample b = s_curve->At(t);
    std::printf(row_format, t, a.position, a.velocity, a.acceleration,
                b.position, b.velocity, b.acceleration, b.jerk);
  }
  return 0;
}
