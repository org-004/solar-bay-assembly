// run_physics: one module's cycle of a layout's committed plan, in physics
// (src/physics_run.h), and its record (src/physics_record.h).
//
//   run_physics [<name>] [--take K] [--write] [--json]
//
// <name> is a layout with a committed plan, plans/<name>.json; without it,
// the chosen layout (versions/versions.json). --take K runs module_take K
// and the set that follows it; without it, the take before the plan's
// hardest carry. It prints the figures of every segment and step, the
// set-down and which steps count by the rule of docs/scorecard.md. --json
// prints the record instead; --write also writes it as physics/<name>.json
// into the source tree (BUILD_WORKSPACE_DIRECTORY), where //:score_layout
// reads it. Exit 0 when the run was made, 1 when it could not be, 2 on a
// wrong argument. Two runs on one machine print the same, byte for byte.
//
//   bazel run --config=opt //:run_physics
//   bazel run --config=opt //:run_physics -- flat_rack_3_pin_arms_ordered --write

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
  std::fprintf(stderr, "usage: run_physics [<name>] [--take K] [--write] [--json]\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace cell::layout;
  const std::vector<std::string> args(argv + 1, argv + argc);
  std::string name;
  bool write = false, json = false;
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
      options.take = static_cast<int>(take);
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
  const PhysicsResult result = RunPhysics(*layout, *plan, ModelPaths::FromEnvironment(),
                                          options, layout_sha, Sha256OfFile(plan_path));
  if (!result.ok) {
    std::fputs(PhysicsText(result).c_str(), stderr);
    return 1;
  }
  const std::string record = PhysicsRecordToJson(result.record);
  std::fputs(json ? record.c_str() : PhysicsText(result).c_str(), stdout);
  if (write) {
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
  }
  return 0;
}
