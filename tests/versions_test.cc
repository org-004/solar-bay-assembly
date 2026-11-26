// versions/versions.json: a registered version does not change (its layout
// and plan files keep the recorded sha256), the chosen layout is the layout of
// the newest chosen version (it is read there, src/layout.h ChosenLayout, and
// no layout file says it), a direction has at most one chosen version, ids
// run v01, v02, ..., and every recording
// exists within its size (R10, R11).

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_plan.h"
#include "src/scorecard.h"

namespace cell::layout {
namespace {

constexpr long long kRecordingMaxBytes = 2500000;
constexpr long long kEvolutionMaxBytes = 20000000;

std::string Env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

std::vector<Version> Versions() {
  std::string error;
  const std::optional<std::vector<Version>> versions =
      LoadVersions(Env("VERSIONS_JSON", "versions/versions.json"), &error);
  EXPECT_TRUE(versions.has_value()) << error;
  return versions.value_or(std::vector<Version>{});
}

// The size of a file, or -1 when there is none.
long long SizeOf(const std::string& path) {
  std::error_code ec;
  const auto size = std::filesystem::file_size(path, ec);
  return ec ? -1 : static_cast<long long>(size);
}

TEST(Versions, ARegisteredVersionDoesNotChange) {
  for (const Version& v : Versions()) {
    for (const std::optional<VersionFile>& file : {v.layout, v.plan}) {
      if (!file) continue;
      const std::string now = Sha256OfFile(file->path);
      EXPECT_EQ(now, file->sha256)
          << "versions/versions.json: " << v.id << " records " << file->path
          << " with sha256 " << file->sha256 << " but the file now has " << now
          << ". A registered version does not change: a change to the chosen "
             "layout or its plan adds a new layout file and a new version with "
             "tools/evolution/add.sh.";
    }
  }
}

TEST(Versions, TheChosenLayoutIsTheNewestChosenVersions) {
  const std::string dir = Env("LAYOUT_DIR", "layouts");
  const std::vector<Version> versions = Versions();
  const Version* newest = nullptr;
  std::map<std::string, int> chosen_in;
  for (const Version& v : versions) {
    if (v.status != "chosen") continue;
    ++chosen_in[v.direction];
    if (v.layout.has_value()) newest = &v;
  }
  for (const auto& [direction, count] : chosen_in) {
    EXPECT_EQ(count, 1) << direction << " has " << count
                        << " versions with status chosen; a chosen version "
                           "supersedes the direction's previous one";
  }
  ASSERT_NE(newest, nullptr) << "no version with status chosen records a layout";
  const std::string path = newest->layout->path;
  const std::string stem =
      path.substr(path.rfind('/') + 1, path.rfind('.') - path.rfind('/') - 1);
  std::string error;
  const std::optional<std::string> chosen =
      ChosenLayout(Env("VERSIONS_JSON", "versions/versions.json"), &error);
  ASSERT_TRUE(chosen.has_value()) << error;
  EXPECT_EQ(*chosen, stem) << newest->id << " records " << path;
  // The chosen layout is a layout that loads; LoadLayout refuses a file that
  // still has a "chosen" field, so every layout loading means none says it.
  for (const std::string& name : ListLayouts(dir)) {
    const std::optional<Layout> layout =
        LoadLayout(dir + "/" + name + ".json", &error);
    EXPECT_TRUE(layout.has_value()) << error;
  }
  const std::vector<std::string> names = ListLayouts(dir);
  EXPECT_NE(std::find(names.begin(), names.end(), *chosen), names.end())
      << "the chosen layout " << *chosen << " is not in " << dir;
}

TEST(Versions, IdsStatusesAndDirections) {
  const std::vector<Version> versions = Versions();
  ASSERT_FALSE(versions.empty());
  const std::set<std::string> statuses = {"chosen", "superseded", "candidate"};
  const std::set<std::string> directions = {
      "stand-in-cell", "one-40ft-flat-rack", "two-20ft-flat-racks"};
  for (size_t i = 0; i < versions.size(); ++i) {
    char id[32];
    std::snprintf(id, sizeof(id), "v%02zu", i + 1);
    EXPECT_EQ(versions[i].id, id) << "versions run v01, v02, ... in order";
    EXPECT_EQ(statuses.count(versions[i].status), 1u)
        << versions[i].id << " has status " << versions[i].status;
    EXPECT_EQ(directions.count(versions[i].direction), 1u)
        << versions[i].id << " has direction " << versions[i].direction;
  }
}

TEST(Versions, RecordingsExistWithinTheirSize) {
  for (const Version& v : Versions()) {
    ASSERT_FALSE(v.recording.empty()) << v.id << " names no recording";
    const long long size = SizeOf(v.recording);
    EXPECT_GE(size, 0) << v.id << ": " << v.recording << " does not exist";
    EXPECT_LE(size, kRecordingMaxBytes)
        << v.id << ": " << v.recording << " is larger than 2,500,000 bytes";
  }
}

TEST(Versions, TheEvolutionDirectoryIsSmall) {
  // Every file of docs/evolution/ (BUILD.bazel gives the test all of them),
  // not only those a version names.
  long long total = 0;
  int files = 0;
  std::error_code ec;
  for (const auto& entry :
       std::filesystem::directory_iterator("docs/evolution", ec)) {
    if (!entry.is_regular_file()) continue;
    ++files;
    total += std::max(0LL, SizeOf(entry.path().string()));
  }
  EXPECT_FALSE(ec) << "cannot list docs/evolution: " << ec.message();
  EXPECT_GT(files, 0) << "docs/evolution/ holds no recording";
  EXPECT_LE(total, kEvolutionMaxBytes)
      << "the files of docs/evolution/ together are larger than 20,000,000 bytes";
}

TEST(Versions, EveryScorecardExists) {
  for (const Version& v : Versions()) {
    if (v.scores.empty()) continue;
    EXPECT_GE(SizeOf(v.scores), 0)
        << v.id << " names the scorecard " << v.scores << ", which does not exist";
  }
}

}  // namespace
}  // namespace cell::layout
