// Checks a layout and prints one row for each check (src/layout_check.h).
//
//   bazel run --config=opt //:check_layout
//   bazel run --config=opt //:check_layout -- --list
//   bazel run --config=opt //:check_layout -- below
//   bazel run --config=opt //:check_layout -- below --summary
//   bazel run --config=opt //:check_layout -- below --steps
//   bazel run --config=opt //:check_layout -- below --pretty
//   bazel run --config=opt //:check_layout -- below --checks reach,clearance
//   bazel run --config=opt //:check_layout -- below --report-out PATH
//
// --list: one line per layout under LAYOUT_DIR, tab-separated: its name,
// "chosen" for the chosen one or "-", its title and its base ("container",
// "flat_rack" or "two_flat_racks"). The chosen layout is the one versions/versions.json names
// (VERSIONS_JSON; src/layout.h, ChosenLayout); exits 1 when that file names
// none or a layout that is not under LAYOUT_DIR.
// With no name (and no --list), the chosen layout: the one
// versions/versions.json names (src/layout.h, ChosenLayout); exits 2 when that
// file names none.
// <name>: the rows as CSV (check,item,arm,tool,figure,unit,limit,result,
// detail), then the bay's time, the time model and the margins used as #
// lines, and for a layout worked in stages one # line per stage (its name,
// how far the tube is moved, its rails, modules and grips, its time); exits
// 0 only when every row passes. --checks a,b,c: makes and
// prints only the rows of those kinds (src/layout_check.h, CheckKinds) and
// exits by them alone. --summary: one line per kind of check, then the bay's
// time and the margins. --steps: the steps of one bay as CSV. --pretty:
// aligned columns. --report-out PATH: prints nothing and writes the whole
// report to PATH as src/layout_report_file.h's file, with the layout file's
// SHA-256, and exits 0 when it is written, whether the rows pass or not (the
// tests that read the file check that); the rule check_report of
// BUILD.bazel makes each one-posture layout's report so. An unknown name,
// option or kind of check exits 2.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/layout_report_file.h"

namespace {

using cell::layout::CheckReport;
using cell::layout::CheckRow;

std::string LayoutDir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr && dir[0] != '\0' ? dir : "layouts";
}

// A field for CSV: quoted when it holds a comma or a quote.
std::string Csv(const std::string& s) {
  if (s.find_first_of(",\"\n") == std::string::npos) return s;
  std::string out = "\"";
  for (char c : s) {
    if (c == '"') out += '"';
    out += c;
  }
  return out + "\"";
}

std::string Num(double v) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.4f", v);
  std::string s = buffer;
  if (s == "-0.0000") s = "0.0000";
  return s;
}

void PrintTable(const std::vector<std::vector<std::string>>& rows,
                bool pretty) {
  std::vector<size_t> width;
  if (pretty) {
    for (const auto& r : rows) {
      if (width.size() < r.size()) width.resize(r.size(), 0);
      for (size_t i = 0; i < r.size(); ++i) {
        width[i] = std::max(width[i], r[i].size());
      }
    }
  }
  for (const auto& r : rows) {
    std::string line;
    for (size_t i = 0; i < r.size(); ++i) {
      if (pretty) {
        line += r[i];
        if (i + 1 < r.size()) line += std::string(width[i] - r[i].size() + 2, ' ');
      } else {
        line += (i > 0 ? "," : "") + Csv(r[i]);
      }
    }
    std::printf("%s\n", line.c_str());
  }
}

int List() {
  const std::string dir = LayoutDir();
  const std::vector<std::string> names = cell::layout::ListLayouts(dir);
  std::string why;
  const std::optional<std::string> chosen =
      cell::layout::ChosenLayout(cell::layout::VersionsPath(), &why);
  bool listed = false;
  for (const std::string& name : names) {
    std::string error;
    const auto layout =
        cell::layout::LoadLayout(dir + "/" + name + ".json", &error);
    if (!layout) {
      std::printf("%s\t?\t(does not load: %s)\n", name.c_str(), error.c_str());
      continue;
    }
    const bool is_chosen = chosen && *chosen == name;
    listed = listed || is_chosen;
    std::printf("%s\t%s\t%s\t%s\n", name.c_str(), is_chosen ? "chosen" : "-",
                layout->title.c_str(), layout->base.c_str());
  }
  if (!chosen) {
    std::fprintf(stderr, "no layout is chosen: %s\n", why.c_str());
    return 1;
  }
  if (!listed) {
    std::fprintf(stderr, "the chosen layout %s is not a layout that loads in %s\n",
                 chosen->c_str(), dir.c_str());
    return 1;
  }
  return 0;
}

// "a,b,c" as its parts; false if a part is empty or not a kind of check.
bool ParseChecks(const std::string& text, std::vector<std::string>* kinds,
                 std::string* bad) {
  const std::vector<std::string>& known = cell::layout::CheckKinds();
  size_t start = 0;
  while (true) {
    const size_t end = text.find(',', start);
    const std::string kind = text.substr(start, end - start);
    if (std::find(known.begin(), known.end(), kind) == known.end()) {
      *bad = kind;
      return false;
    }
    kinds->push_back(kind);
    if (end == std::string::npos) return true;
    start = end + 1;
  }
}

void PrintMargins(const CheckReport& report) {
  std::printf("# margins: clearance %.3f m, joint margin %.1f deg\n",
              report.clearance_m, report.joint_margin_deg);
  for (const auto& p : report.postures) {
    std::printf("# posture %s: largest change %.2f s between %s and %s; %d "
                "changes above 2.0 s\n",
                p.arm.c_str(), p.largest_s, p.from.c_str(), p.to.c_str(),
                p.above_2s);
  }
}

void PrintStages(const cell::layout::Layout& layout, const CheckReport& report) {
  if (!layout.staging) return;
  for (const cell::layout::Stage& st : layout.staging->stages) {
    std::string grips;
    for (const cell::layout::Grip& g : st.grips) {
      grips += (grips.empty() ? "" : ", ") + std::string("bay_hold_") +
               std::to_string(g.hold);
    }
    std::string time = "no time";
    for (const CheckRow& r : report.rows) {
      if (r.check == "time" && r.item == "stage " + st.name + ": bay") {
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "%.1f s", r.figure);
        time = buffer;
      }
    }
    std::printf("# stage %s: tube moved %+.3f m along Y; rails %s; modules "
                "%s; grips %s; %s\n",
                st.name.c_str(), st.tube_offset_m, st.rails.c_str(),
                st.modules.empty() ? "none" : st.modules.c_str(),
                grips.empty() ? "none" : grips.c_str(), time.c_str());
  }
}

const char kUsage[] =
    "usage: check_layout --list | [<name>] [--summary|--steps] [--pretty] "
    "[--checks kind,kind,...] | <name> --report-out PATH\n"
    "(without a name, the chosen layout of versions/versions.json)\n";

}  // namespace

int main(int argc, char** argv) {
  std::string name, report_out;
  bool steps = false, summary = false, pretty = false, list = false;
  cell::layout::CheckOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--checks") {
      std::string bad;
      if (i + 1 >= argc || !ParseChecks(argv[i + 1], &options.checks, &bad)) {
        std::string kinds;
        for (const std::string& k : cell::layout::CheckKinds()) {
          kinds += (kinds.empty() ? "" : ",") + k;
        }
        std::fprintf(stderr, "%s--checks takes kinds of check separated by "
                     "commas, of %s%s\n", kUsage, kinds.c_str(),
                     i + 1 >= argc ? "" : ("; not \"" + bad + "\"").c_str());
        return 2;
      }
      ++i;
    } else if (a == "--report-out") {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s--report-out takes a path\n", kUsage);
        return 2;
      }
      report_out = argv[++i];
    } else if (a == "--list") {
      list = true;
    } else if (a == "--steps") {
      steps = true;
    } else if (a == "--summary") {
      summary = true;
    } else if (a == "--pretty") {
      pretty = true;
    } else if (!a.empty() && a[0] == '-') {
      std::fprintf(stderr, "unknown option %s\n", a.c_str());
      return 2;
    } else {
      name = a;
    }
  }
  if (list) return List();
  if (name.empty() && !report_out.empty()) {
    std::fprintf(stderr, "%s--report-out needs a layout's name\n", kUsage);
    return 2;
  }
  if (name.empty()) {
    std::string why;
    const std::optional<std::string> chosen =
        cell::layout::ChosenLayout(cell::layout::VersionsPath(), &why);
    if (!chosen) {
      std::fprintf(stderr, "%sno layout is chosen: %s\n", kUsage, why.c_str());
      return 2;
    }
    name = *chosen;
    std::fprintf(stderr, "check_layout: the chosen layout, %s\n", name.c_str());
  }
  if (!report_out.empty() && !options.checks.empty()) {
    std::fprintf(stderr, "%s--report-out writes the whole report, without --checks\n", kUsage);
    return 2;
  }
  const std::string dir = LayoutDir();
  bool known = false;
  for (const std::string& n : cell::layout::ListLayouts(dir)) known |= n == name;
  if (!known) {
    std::fprintf(stderr, "no layout %s in %s (--list names them)\n",
                 name.c_str(), dir.c_str());
    return 2;
  }
  std::string error;
  const auto layout =
      cell::layout::LoadLayout(dir + "/" + name + ".json", &error);
  if (!layout) {
    std::fprintf(stderr, "layout %s does not load: %s\n", name.c_str(),
                 error.c_str());
    return 2;
  }
  const CheckReport report = cell::layout::CheckLayout(
      *layout, cell::layout::ModelPaths::FromEnvironment(), options);
  if (!report.error.empty()) {
    std::fprintf(stderr, "model: %s\n", report.error.c_str());
  }
  if (!report_out.empty()) {
    const std::string sha256 = cell::layout::Sha256OfFile(dir + "/" + name + ".json");
    std::ofstream out(report_out, std::ios::binary);
    out << cell::layout::CheckReportToJson(report, name, sha256);
    out.close();
    if (!out) {
      std::fprintf(stderr, "cannot write %s\n", report_out.c_str());
      return 2;
    }
    return 0;
  }
  if (steps) {
    std::vector<std::vector<std::string>> rows = {
        {"step", "what", "arm", "tool", "part", "from", "to", "time_s"}};
    for (const auto& s : report.steps) {
      char t[32];
      std::snprintf(t, sizeof(t), "%.2f", s.time_s);
      rows.push_back({std::to_string(s.number), s.what, s.arm, s.tool, s.part,
                      s.from, s.to, t});
    }
    PrintTable(rows, pretty);
    std::printf("# bay time %.1f s\n", report.bay_time_s);
  } else if (summary) {
    struct Kind {
      int rows = 0, passed = 0;
      double worst = 0.0;
      std::string worst_item, unit;
      bool any = false;
    };
    std::vector<std::string> order;
    std::map<std::string, Kind> kinds;
    for (const CheckRow& r : report.rows) {
      if (!kinds.count(r.check)) order.push_back(r.check);
      Kind& k = kinds[r.check];
      ++k.rows;
      k.passed += r.pass ? 1 : 0;
      // The worst figure: the least margin over its limit, by the sense of
      // the check (a time is a figure: the largest).
      const bool upper = r.check == "load" || r.check == "time" ||
                         (r.check == "hold" &&
                          (r.item == "angle" || r.item.rfind("grip ", 0) == 0)) ||
                         (r.check == "transport" && r.item == "mass");
      const double slack = upper ? r.limit - r.figure : r.figure - r.limit;
      const double score = r.check == "time" ? -r.figure : slack;
      if (!k.any || score < k.worst) {
        k.any = true;
        k.worst = score;
        k.worst_item = r.item + " " + Num(r.figure) + " " + r.unit;
      }
    }
    std::vector<std::vector<std::string>> rows = {
        {"check", "rows", "passed", "failed", "worst"}};
    for (const std::string& c : order) {
      const Kind& k = kinds[c];
      rows.push_back({c, std::to_string(k.rows), std::to_string(k.passed),
                      std::to_string(k.rows - k.passed), k.worst_item});
    }
    PrintTable(rows, pretty);
    std::printf("# bay time %.1f s; %s\n", report.bay_time_s,
                report.pass ? "pass" : "fail");
    PrintMargins(report);
    PrintStages(*layout, report);
  } else {
    std::vector<std::vector<std::string>> rows = {
        {"check", "item", "arm", "tool", "figure", "unit", "limit", "result",
         "detail"}};
    for (const CheckRow& r : report.rows) {
      rows.push_back({r.check, r.item, r.arm, r.tool, Num(r.figure), r.unit,
                      Num(r.limit), r.pass ? "pass" : "fail", r.detail});
    }
    PrintTable(rows, pretty);
    std::printf("# bay time %.1f s\n", report.bay_time_s);
    std::string text = report.time_model;
    size_t start = 0;
    while (start <= text.size()) {
      const size_t end = text.find('\n', start);
      std::printf("# %s\n", text.substr(start, end - start).c_str());
      if (end == std::string::npos) break;
      start = end + 1;
    }
    PrintMargins(report);
    PrintStages(*layout, report);
  }
  return report.pass ? 0 : 1;
}
