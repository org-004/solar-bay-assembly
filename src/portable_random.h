// Random draws that are the same on every platform.
//
// std::mt19937's sequence of 32-bit outputs is fixed by the C++ standard, but
// the library distributions built on it (std::uniform_real_distribution,
// std::normal_distribution, std::uniform_int_distribution) are not: each
// standard library chooses its own algorithm, and libstdc++, libc++ and
// Microsoft's give different numbers for the same seed. The draws here are
// written out on the generator's raw outputs instead. Each one is, bit for
// bit, what libstdc++ (GCC 11 to 16) gives for the same call on the same
// generator, so code moved onto them draws exactly what it drew before on
// this repository's own toolchain.
//
// Not covered here, and as important: the order of draws. In
//
//     Vector2d(unit(rng), unit(rng))
//
// the C++ standard leaves the order in which the two arguments are
// evaluated to the compiler, and GCC evaluates them right to left on x86-64
// and left to right on 64-bit ARM: the same seed gives x and y swapped. Take
// each draw in a statement of its own.
//
// What stays platform-dependent: Normal() calls std::log, which the C++
// standard does not require to be correctly rounded, so two math libraries
// can differ in the last bit. That changes a scene by a part in 10^16, not
// which scene it is.

#ifndef SRC_PORTABLE_RANDOM_H_
#define SRC_PORTABLE_RANDOM_H_

#include <cmath>
#include <cstdint>
#include <random>

namespace cell {

// Uniform in [0, 1): two outputs, the first as the low 32 bits and the
// second as the high 32 bits of a 64-bit integer, rounded to the nearest
// double and divided by 2^64; a result that rounds up to 1 is drawn again.
// As std::generate_canonical<double, 53> on a 32-bit generator.
inline double UniformUnit(std::mt19937& rng) {
  for (;;) {
    const std::uint64_t low = rng();
    const std::uint64_t high = rng();
    const double unit = static_cast<double>(low | high << 32) * 0x1p-64;
    if (unit < 1.0) return unit;
  }
}

// Uniform in [a, b), as std::uniform_real_distribution<double>(a, b).
inline double UniformReal(std::mt19937& rng, double a, double b) {
  return UniformUnit(rng) * (b - a) + a;
}

// Uniform over the integers 0 to n - 1, n >= 1, by Lemire's multiply and
// shift with rejection, as std::uniform_int_distribution<int>(0, n - 1).
inline int UniformIndex(std::mt19937& rng, std::uint32_t n) {
  std::uint64_t product = static_cast<std::uint64_t>(rng()) * n;
  std::uint32_t low = static_cast<std::uint32_t>(product);
  if (low < n) {
    const std::uint32_t threshold = static_cast<std::uint32_t>(-n) % n;
    while (low < threshold) {
      product = static_cast<std::uint64_t>(rng()) * n;
      low = static_cast<std::uint32_t>(product);
    }
  }
  return static_cast<int>(product >> 32);
}

// Normal with the given mean and standard deviation, as
// std::normal_distribution<double>: Marsaglia's polar method, which turns
// one accepted pair of uniform draws into two normal draws and keeps the
// second for the next call. Like the library's, an object carries that
// spare; a fresh one starts a new pair.
class NormalDraw {
 public:
  explicit NormalDraw(double mean = 0.0, double stddev = 1.0)
      : mean_(mean), stddev_(stddev) {}

  double operator()(std::mt19937& rng) {
    double standard;
    if (has_spare_) {
      has_spare_ = false;
      standard = spare_;
    } else {
      double x, y, r2;
      do {
        x = 2.0 * UniformUnit(rng) - 1.0;
        y = 2.0 * UniformUnit(rng) - 1.0;
        r2 = x * x + y * y;
      } while (r2 > 1.0 || r2 == 0.0);
      const double scale = std::sqrt(-2 * std::log(r2) / r2);
      spare_ = x * scale;
      has_spare_ = true;
      standard = y * scale;
    }
    return standard * stddev_ + mean_;
  }

 private:
  double mean_;
  double stddev_;
  double spare_ = 0.0;
  bool has_spare_ = false;
};

}  // namespace cell

#endif  // SRC_PORTABLE_RANDOM_H_
