// score_layout: the scorecard of a layout (src/scorecard.h).
//
//   score_layout <name> [--json]   print the committed scorecard (scores/<name>.json),
//                                  or one computed afresh when the file is
//                                  missing or stale; exit 0 when every gate passes
//   score_layout <name> --write    compute it and write scores/<name>.json
//                                  into the source tree (BUILD_WORKSPACE_DIRECTORY)
//   score_layout --all [--write]   one table of every layout with a scorecard
//
// A layout without a committed plan has no scorecard: exit 2. The files come
// from LAYOUT_DIR, PLAN_DIR and SCORES_DIR (set by BUILD.bazel); the models
// from MODELS_DIR and UR10E_XML. Printing from a fresh file does not run the
// checker or the verifier; computing does (about 16 s a layout).
// The work is RunScoreLayout of src/scorecard.h, so that a test can run the
// program without starting a process.
#include <cstdio>
#include <string>
#include <vector>

#include "src/scorecard.h"

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  std::string out;
  const int code = cell::layout::RunScoreLayout(args, &out);
  std::fputs(out.c_str(), stdout);
  return code;
}
