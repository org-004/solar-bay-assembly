// Prints models/bay.xml, as src/bay_model.h computes it from the constants of
// src/bay_hardware.h, to standard output and writes nothing else.
//
//   bazel run --config=opt //:gen_bay > models/bay.xml

#include <cstdio>
#include <string>

#include "src/bay_model.h"

int main() {
  const std::string text = cell::bay::BayModelXml();
  std::fwrite(text.data(), 1, text.size(), stdout);
  return 0;
}
