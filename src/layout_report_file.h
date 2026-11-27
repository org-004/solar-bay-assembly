// A checker's report (src/layout_check.h, CheckReport) as a file, so that a
// report that takes minutes to make is made once per layout and checker
// build and read by every test that needs it (run 039).
//
// The layouts with "one_posture_per_module" take 125 to 180 s each to check
// (the poses of every module's holes are solved for one posture, which
// solves the approach legs again and again). Their reports are made by a
// Bazel action each (bazel/check_report.bzl, rule check_report in
// BUILD.bazel): `check_layout <name> --report-out PATH` writes this file,
// the action's inputs are the layout's file, the model files and the
// check_layout program, and nothing else, so Bazel's caches keep the file
// across commits that change neither the layout nor the checker. The tests
// that need such a report (tests/plan_replay_staged_test.cc,
// tests/scorecard_test.cc) take the files as data and read them here.
//
// The file is JSON, 2-space indent, a final newline: "layout" (the layout's
// name), "layout_sha256" (Sha256OfFile of the layout's file that the report
// was made from) and "report", every field of CheckReport by its name in
// the header, in the header's order: rows, steps, points, tool_changes,
// phases (a list of [name, seconds]) and postures as lists of objects,
// joint values as lists of six numbers. A number is written so that it
// reads back to the same double, bit for bit (nlohmann::json writes 17
// significant digits); a number that is not finite (a figure with nothing
// to measure) is written as the string "inf", "-inf" or "nan". So a report
// read back is equal, field by field, to the one CheckLayout computed
// (tests/check_report_file_test.cc).
//
// ReadCheckReportFile refuses a file whose layout_sha256 is not the one
// given (the file was made from another version of the layout) or whose
// layout name is not the one given: the caller must not use a report of
// another layout.

#ifndef SRC_LAYOUT_REPORT_FILE_H_
#define SRC_LAYOUT_REPORT_FILE_H_

#include <optional>
#include <string>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"

namespace cell::layout {

// The file's text for the report of layout `name`, whose file has SHA-256
// `layout_sha256`.
std::string CheckReportToJson(const CheckReport& report, const std::string& name,
                              const std::string& layout_sha256);

// Reads the file's text back. Returns nothing and sets *error when the text
// is not such a file, or when its layout or layout_sha256 is not `name` or
// `layout_sha256`.
std::optional<CheckReport> CheckReportFromJson(const std::string& text,
                                               const std::string& name,
                                               const std::string& layout_sha256,
                                               std::string* error);

// The same for the file at `path`; a missing file is an error too.
std::optional<CheckReport> ReadCheckReportFile(const std::string& path,
                                               const std::string& name,
                                               const std::string& layout_sha256,
                                               std::string* error);

// The report a test uses for `layout`, whose file has SHA-256
// `layout_sha256`: for a layout with one_posture_per_module, the file
// <report_dir>/<layout.name>.json, read by ReadCheckReportFile (a missing
// file or one of another version of the layout is an error, so a new
// one-posture layout needs its check_report rule in BUILD.bazel); for any
// other layout CheckLayout's report, computed (seconds). Tests pass
// ReportDir().
std::optional<CheckReport> CheckReportOf(const Layout& layout,
                                         const std::string& layout_sha256,
                                         const ModelPaths& paths,
                                         const std::string& report_dir,
                                         std::string* error);

// CHECK_REPORT_DIR, else "check_reports".
std::string ReportDir();

}  // namespace cell::layout

#endif  // SRC_LAYOUT_REPORT_FILE_H_
