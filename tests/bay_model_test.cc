// Checks the generator of the bay's model, src/bay_model.h:
//   - the committed models/bay.xml is, byte for byte, what BayModelXml()
//     returns; when it is not, the message shows the first line that differs
//     and the command that writes the file again;
//   - the options: fewer rails or fewer modules give a model that MuJoCo
//     loads, with the bodies and the sites the options promise, and the
//     tube's sites stay whatever is on it;
//   - the default place of the bay's frame is where the container's -X side
//     is, as models/container_scene.xml says;
//   - the pieces: a module and a rail under names of their own load, with
//     geoms named after the body and without sites.
//
//   bazel test //:bay_model_test --test_output=all
//
// The paths of models/bay.xml and models/common.xml come from the environment
// variables BAY_XML and COMMON_XML, which BUILD.bazel sets for this test.

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "src/bay_hardware.h"
#include "src/bay_model.h"
#include "src/container.h"
#include "src/mj_handles.h"

namespace {

namespace bay = cell::bay;
namespace fs = std::filesystem;

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

std::string EnvPath(const char* name) {
  const char* value = std::getenv(name);
  EXPECT_NE(value, nullptr) << name << " is not set";
  return value == nullptr ? "" : fs::absolute(value).string();
}

std::vector<std::string> Lines(const std::string& text) {
  std::vector<std::string> lines;
  std::stringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) lines.push_back(line);
  return lines;
}

// Loads `bay_xml` as a part of a scene with common.xml, the way
// models/bay_scene.xml does, through files in the test's temporary directory.
cell::ModelPtr LoadBay(const std::string& bay_xml, const std::string& tag,
                       std::string* error) {
  const char* temp = std::getenv("TEST_TMPDIR");
  const fs::path dir = temp != nullptr ? temp : fs::temp_directory_path();
  const fs::path part = dir / (tag + "_part.xml");
  const fs::path scene = dir / (tag + "_scene.xml");
  std::ofstream(part) << bay_xml;
  std::ofstream(scene) << "<mujoco>\n  <include file=\"" << EnvPath("COMMON_XML")
                       << "\"/>\n  <include file=\"" << part.string()
                       << "\"/>\n</mujoco>\n";
  return cell::LoadModel(scene.string(), error);
}

int Count(const mjModel& m, mjtObj type, const std::string& prefix) {
  int found = 0;
  const int n = type == mjOBJ_BODY   ? m.nbody
                : type == mjOBJ_GEOM ? m.ngeom
                                     : m.nsite;
  for (int i = 0; i < n; ++i) {
    const char* name = mj_id2name(&m, type, i);
    if (name != nullptr && std::string(name).rfind(prefix, 0) == 0) ++found;
  }
  return found;
}

int Sites(const mjModel& m, const std::string& infix) {
  int found = 0;
  for (int i = 0; i < m.nsite; ++i) {
    const char* name = mj_id2name(&m, mjOBJ_SITE, i);
    if (name != nullptr && std::string(name).find(infix) != std::string::npos) {
      ++found;
    }
  }
  return found;
}

TEST(BayModelTest, CommittedFileIsWhatTheGeneratorPrints) {
  const std::string committed = ReadFile(EnvPath("BAY_XML"));
  const std::string generated = bay::BayModelXml();
  if (committed == generated) {
    SUCCEED();
    return;
  }
  const std::vector<std::string> a = Lines(committed);
  const std::vector<std::string> b = Lines(generated);
  size_t line = 0;
  while (line < a.size() && line < b.size() && a[line] == b[line]) ++line;
  const std::string in_file = line < a.size() ? a[line] : "(end of file)";
  const std::string in_generator = line < b.size() ? b[line] : "(end of output)";
  ADD_FAILURE() << "models/bay.xml is not what BayModelXml() prints. First "
                << "difference at line " << line + 1 << ":\n  file:      "
                << in_file << "\n  generator: " << in_generator
                << "\nWrite the file again with:\n  bazel run --config=opt "
                << "//:gen_bay > models/bay.xml";
}

TEST(BayModelTest, WholeBayHasTheHeadersCounts) {
  std::string error;
  const cell::ModelPtr model = LoadBay(bay::BayModelXml(), "whole", &error);
  ASSERT_TRUE(model) << error;
  EXPECT_EQ(Count(*model, mjOBJ_BODY, "bay_rail_"), bay::kRailCount);
  EXPECT_EQ(Count(*model, mjOBJ_BODY, "bay_module_"), bay::kModuleCount);
  EXPECT_EQ(Sites(*model, "bay_fasten_"), 2 * bay::kFasteningCount);
  EXPECT_EQ(Sites(*model, "bay_locate_"), 2 * bay::kLocatorCount);
  EXPECT_EQ(Sites(*model, "bay_hold_"), bay::kHoldCount);
}

TEST(BayModelTest, FewerRailsAndModulesGiveSmallerBays) {
  struct Case {
    int rails, modules;
  };
  for (const Case c : {Case{0, 0}, Case{1, 0}, Case{3, 0}, Case{3, 2},
                       Case{bay::kRailCount, 0}, Case{4, 3},
                       Case{bay::kRailCount, bay::kModuleCount}}) {
    SCOPED_TRACE("rails " + std::to_string(c.rails) + ", modules " +
                 std::to_string(c.modules));
    bay::BayModelOptions options;
    options.rails = c.rails;
    options.modules = c.modules;
    std::string error;
    const cell::ModelPtr model = LoadBay(
        bay::BayModelXml(options),
        "r" + std::to_string(c.rails) + "m" + std::to_string(c.modules),
        &error);
    ASSERT_TRUE(model) << error;
    EXPECT_EQ(Count(*model, mjOBJ_BODY, "bay_rail_"), c.rails);
    EXPECT_EQ(Count(*model, mjOBJ_BODY, "bay_module_"), c.modules);
    EXPECT_EQ(Count(*model, mjOBJ_BODY, "bay_tube"), 1);
    // World, frame, tube, and the parts.
    EXPECT_EQ(model->nbody, 3 + c.rails + c.modules);
    // Fifteen geoms to a rail, nine to a module, two for the tube.
    EXPECT_EQ(Count(*model, mjOBJ_GEOM, "bay_"), 2 + 15 * c.rails + 9 * c.modules);
    // The tube's sites stay: a _b for every strap bolt and every locator, and
    // the holds.
    EXPECT_EQ(Sites(*model, "bay_fasten_rail_") - c.rails, bay::kRailCount);
    EXPECT_EQ(Sites(*model, "bay_locate_") - c.rails, bay::kLocatorCount);
    EXPECT_EQ(Sites(*model, "bay_hold_"), bay::kHoldCount);
    // A module brings the _b sites of its pins.
    int module_b = 0;
    for (int i = 0; i < model->nsite; ++i) {
      const std::string name = mj_id2name(model.get(), mjOBJ_SITE, i);
      if (name.rfind("bay_fasten_module_", 0) == 0 &&
          name.substr(name.size() - 2) == "_b") {
        ++module_b;
      }
    }
    EXPECT_EQ(module_b, bay::kPinsPerModule * c.modules);
    // A rail carries its own: the bolt's, the tab's, and the _a of the pins
    // of every module of the bay that has one on that rail.
    int rail_a = 0;
    for (int rail = 0; rail < c.rails; ++rail) {
      rail_a += 2 + (rail > 0 ? 2 : 0) + (rail < bay::kModuleCount ? 2 : 0);
    }
    int a_sites = 0;
    for (int i = 0; i < model->nsite; ++i) {
      const std::string name = mj_id2name(model.get(), mjOBJ_SITE, i);
      if (name.size() > 2 && name.substr(name.size() - 2) == "_a") ++a_sites;
    }
    EXPECT_EQ(a_sites, rail_a);
  }
}

TEST(BayModelTest, WithoutCommentTheFileStartsAtTheModel) {
  bay::BayModelOptions options;
  options.comment = false;
  const std::string text = bay::BayModelXml(options);
  EXPECT_EQ(text.rfind("<mujoco>\n", 0), 0u);
  const std::string with = bay::BayModelXml();
  ASSERT_GT(with.size(), text.size());
  EXPECT_EQ(with.compare(with.size() - text.size(), text.size(), text), 0);
  std::string error;
  EXPECT_TRUE(LoadBay(text, "nocomment", &error)) << error;
}

TEST(BayModelTest, FramePositionIsWrittenAsGiven) {
  bay::BayModelOptions options;
  options.comment = false;
  options.frame_pos = {1.0, 2.0, 0.5};
  const std::string text = bay::BayModelXml(options);
  EXPECT_NE(text.find("<body name=\"bay_frame\" pos=\"1.000 2.000 0.500\">"),
            std::string::npos);
}

TEST(BayModelTest, DefaultFrameIsInThePlaneOfTheContainersSide) {
  const bay::BayModelOptions options;
  EXPECT_NEAR(options.frame_pos[0], -cell::container::kWidth / 2.0, 1e-9);
  EXPECT_NEAR(options.frame_pos[1], -bay::kSectionLength / 2.0, 1e-9);
}

TEST(BayModelTest, OptionsOutOfRangeAreRefused) {
  bay::BayModelOptions options;
  options.rails = bay::kRailCount + 1;
  EXPECT_THROW(bay::BayModelXml(options), std::invalid_argument);
  options = {};
  options.modules = -1;
  EXPECT_THROW(bay::BayModelXml(options), std::invalid_argument);
}

TEST(BayModelTest, PiecesTakeTheNameAndPositionGiven) {
  const std::string module =
      bay::ModuleBodyXml("pallet_module_7", {1.0, 2.0, 3.0});
  const std::string rail = bay::RailBodyXml("stock_rail_2", {-1.0, 0.0, 0.5});
  EXPECT_EQ(module.find("<site"), std::string::npos);
  EXPECT_EQ(rail.find("<site"), std::string::npos);
  EXPECT_NE(module.find("<body name=\"pallet_module_7\" pos=\"1 2 3\">"),
            std::string::npos);
  EXPECT_NE(module.find("name=\"pallet_module_7_laminate\""),
            std::string::npos);
  EXPECT_NE(rail.find("name=\"stock_rail_2_strap_ears\""), std::string::npos);
  // They load inside a world of their own.
  bay::BayModelOptions none;
  none.rails = 0;
  none.modules = 0;
  none.comment = false;
  std::string text = bay::BayModelXml(none);
  const std::string tail = "    </body>\n  </worldbody>\n</mujoco>\n";
  ASSERT_EQ(text.compare(text.size() - tail.size(), tail.size(), tail), 0);
  text.insert(text.size() - tail.size(), module + rail);
  std::string error;
  const cell::ModelPtr model = LoadBay(text, "pieces", &error);
  ASSERT_TRUE(model) << error;
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_BODY, "pallet_module_7"), 0);
  EXPECT_GE(mj_name2id(model.get(), mjOBJ_BODY, "stock_rail_2"), 0);
  // The mass is the header's.
  const int body = mj_name2id(model.get(), mjOBJ_BODY, "pallet_module_7");
  EXPECT_NEAR(model->body_mass[body], bay::kModuleMass, 1e-5);
  const int rail_body = mj_name2id(model.get(), mjOBJ_BODY, "stock_rail_2");
  EXPECT_NEAR(model->body_mass[rail_body], bay::kRailMass, 1e-5);
}

}  // namespace
