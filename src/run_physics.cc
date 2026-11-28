// run_physics: one module's cycle of a layout's committed plan, in physics
// (src/physics_run.h), and its record (src/physics_record.h).
//
//   run_physics [<name>] [--take K]... [--rail K] [--bay 0] [--write] [--json]
//
// <name> is a layout with a committed plan, plans/<name>.json; without it,
// the chosen layout (versions/versions.json). --take K runs module_take K
// and the set that follows it; without it, the take before the plan's
// hardest carry. --take given more than once runs each of those takes, and
// --bay 0 every module take of bay 0 (the only bay the record counts: the
// later bays repeat its modules), each on its own from the plan's state at
// its start, and joins them as one record of several takes; since run 074
// --bay 0 also runs every rail set of the module arm in bay 0 (rail_set k,
// the rail carried at the gripper and set on the tube) and the record holds
// them beside the takes. --rail K runs the set of rail K alone. It prints the figures of every segment and step, the
// set-down and which steps count by the rule of docs/scorecard.md. --json
// prints the record instead; --write also writes it as physics/<name>.json
// into the source tree (BUILD_WORKSPACE_DIRECTORY), where //:score_layout
// reads it. Exit 0 when the run was made, 1 when it could not be, 2 on a
// wrong argument. Two runs on one machine print the same, byte for byte.
//
//   bazel run --config=opt //:run_physics
//   bazel run --config=opt //:run_physics -- flat_rack_3_pin_arms_ordered --write
//   bazel run --config=opt //:run_physics -- <name> --bay 0 --write

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_model.h"
#include "src/layout_plan.h"
#include "src/physics_record.h"
#include "src/physics_run.h"

namespace {

std::string Env(const char* name, const char* fallback) {
  const char* value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

int Usage() {
  std::fprintf(stderr,
               "usage: run_physics [<name>] [--take K]... [--rail K] [--bay 0] [--write] [--json]\n");
  return 2;
}

// Writes the record as physics/<name>.json into the source tree.
int WriteRecord(const std::string& name, const std::string& record) {
  const std::string root = Env("BUILD_WORKSPACE_DIRECTORY", ".");
  const std::filesystem::path dir = std::filesystem::path(root) / "physics";
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  const std::filesystem::path path = dir / (name + ".json");
  std::ofstream out(path, std::ios::binary);
  out << record;
  out.close();
  if (!out) {
    std::fprintf(stderr, "run_physics: cannot write %s\n", path.string().c_str());
    return 1;
  }
  std::fprintf(stderr, "wrote %s\n", path.string().c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace cell::layout;
  const std::vector<std::string> args(argv + 1, argv + argc);
  std::string name;
  bool write = false, json = false, bay = false;
  std::vector<int> takes;
  PhysicsOptions options;
  for (size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    if (a == "--write") {
      write = true;
    } else if (a == "--json") {
      json = true;
    } else if (a == "--take" && i + 1 < args.size()) {
      const std::string& k = args[++i];
      char* end = nullptr;
      const long take = std::strtol(k.c_str(), &end, 10);
      if (k.empty() || *end != '\0' || take < 0) return Usage();
      if (std::find(takes.begin(), takes.end(), take) != takes.end()) return Usage();
      takes.push_back(static_cast<int>(take));
    } else if (a == "--rail" && i + 1 < args.size() && options.rail < 0) {
      const std::string& k = args[++i];
      char* end = nullptr;
      const long rail = std::strtol(k.c_str(), &end, 10);
      if (k.empty() || *end != '\0' || rail < 0) return Usage();
      options.rail = static_cast<int>(rail);
    } else if (a == "--bay" && i + 1 < args.size()) {
      if (args[++i] != "0") {
        std::fprintf(stderr, "run_physics: only bay 0 is run; the later bays repeat its modules\n");
        return 2;
      }
      bay = true;
    } else if ((!a.empty() && a[0] == '-') || !name.empty()) {
      return Usage();
    } else {
      name = a;
    }
  }
  std::string error;
  if (name.empty()) {
    const std::optional<std::string> chosen = ChosenLayout(VersionsPath(), &error);
    if (!chosen) {
      std::fprintf(stderr, "run_physics: %s\n", error.c_str());
      return 1;
    }
    name = *chosen;
  }
  const std::string layout_path = Env("LAYOUT_DIR", "layouts") + "/" + name + ".json";
  const std::string plan_path = Env("PLAN_DIR", "plans") + "/" + name + ".json";
  const std::optional<Layout> layout = LoadLayout(layout_path, &error);
  if (!layout) {
    std::fprintf(stderr, "run_physics: %s\n", error.c_str());
    return 1;
  }
  std::ifstream in(plan_path);
  if (!in) {
    std::fprintf(stderr, "run_physics: no committed plan %s\n", plan_path.c_str());
    return 1;
  }
  std::stringstream text;
  text << in.rdbuf();
  const std::optional<Plan> plan = PlanFromJson(text.str(), &error);
  if (!plan) {
    std::fprintf(stderr, "run_physics: %s: %s\n", plan_path.c_str(), error.c_str());
    return 1;
  }
  const std::string layout_sha = Sha256OfFile(layout_path);
  if (plan->layout_sha256 != layout_sha) {
    std::fprintf(stderr, "run_physics: the plan was made for another version of %s\n",
                 layout_path.c_str());
    return 1;
  }
  if (bay && !takes.empty()) return Usage();
  if (options.rail >= 0 && (bay || !takes.empty())) return Usage();
  if (takes.size() == 1) options.take = takes.front();
  std::vector<int> rails;
  if (bay) {
    takes = BayTakes(*layout, ModelPaths::FromEnvironment(), &error);
    if (!takes.empty()) rails = BayRails(*layout, ModelPaths::FromEnvironment(), &error);
    if (takes.empty() || !error.empty()) {
      std::fprintf(stderr, "run_physics: %s\n", error.c_str());
      return 1;
    }
  }
  std::string record;
  if (takes.size() > 1) {
    std::string command = "bazel run --config=opt //:run_physics -- " + name;
    if (bay) {
      command += " --bay 0";
    } else {
      for (const int k : takes) command += " --take " + std::to_string(k);
    }
    command += " --write";
    const PhysicsBayResult result =
        RunPhysicsTakes(*layout, *plan, ModelPaths::FromEnvironment(), options, takes, rails,
                        command,
                        layout_sha, Sha256OfFile(plan_path));
    if (!result.ok) {
      std::fputs(PhysicsBayText(result).c_str(), stderr);
      return 1;
    }
    record = PhysicsRecordToJson(result.record);
    std::fputs(json ? record.c_str() : PhysicsBayText(result).c_str(), stdout);
    return write ? WriteRecord(name, record) : 0;
  }
  const PhysicsResult result = RunPhysics(*layout, *plan, ModelPaths::FromEnvironment(),
                                          options, layout_sha, Sha256OfFile(plan_path));
  if (!result.ok) {
    std::fputs(PhysicsText(result).c_str(), stderr);
    return 1;
  }
  record = PhysicsRecordToJson(result.record);
  std::fputs(json ? record.c_str() : PhysicsText(result).c_str(), stdout);
  return write ? WriteRecord(name, record) : 0;
}
