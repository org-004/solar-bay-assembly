// score_layout: the scorecard of a layout (src/scorecard.h).
//
//   score_layout <name> [--json]   print the committed scorecard (scores/<name>.json),
//                                  or one computed afresh when the file is
//                                  missing or stale; exit 0 when every gate passes
//   score_layout <name> --write    compute it and write scores/<name>.json
//                                  into the source tree (BUILD_WORKSPACE_DIRECTORY)
//   score_layout --all [--write]   one table of every layout with a scorecard
//
// Without a name (and without --all), the chosen layout: the one
// versions/versions.json names (VERSIONS_JSON; src/layout.h, ChosenLayout);
// exit 2 when that file names none.
// A layout without a committed plan has no scorecard: exit 2. The files come
// from LAYOUT_DIR, PLAN_DIR and SCORES_DIR (set by BUILD.bazel); the models
// from MODELS_DIR and UR10E_XML. Printing from a fresh file does not run the
// checker or the verifier; computing does (about 16 s a layout).
// The work is RunScoreLayout of src/scorecard.h, so that a test can run the
// program without starting a process.
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/scorecard.h"

int main(int argc, char** argv) {
  std::vector<std::string> args(argv + 1, argv + argc);
  bool named = false;
  for (const std::string& a : args) named |= a.empty() || a[0] != '-' || a == "--all";
  if (!named) {
    std::string why;
    const std::optional<std::string> chosen =
        cell::layout::ChosenLayout(cell::layout::VersionsPath(), &why);
    if (!chosen) {
      std::fprintf(stderr, "score_layout: no layout is chosen: %s\n", why.c_str());
      return 2;
    }
    std::fprintf(stderr, "score_layout: the chosen layout, %s\n", chosen->c_str());
    args.insert(args.begin(), *chosen);
  }
  std::string out;
  const int code = cell::layout::RunScoreLayout(args, &out);
  std::fputs(out.c_str(), stdout);
  return code;
}
