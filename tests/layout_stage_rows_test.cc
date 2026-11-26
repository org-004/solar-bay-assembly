// The ways rows of the staged two-rack layout that measure the infeed and
// outfeed units (src/layout_stage_rows.h): the rows exist and pass; a unit's
// bed raised into the tube's or the built part's path fails its way, naming
// the unit's box (the units used to be left out); an item box on the tube at
// rest, or a jaw 1 mm into it, fails "tube at rest" while the jaws 2 mm
// beside it by design pass; the gap variants rerun the port rows. Only the
// kinds the checks need are made ({"ways"}: no pose is solved).

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"

namespace cell::layout {
namespace {

const char* kName = "two_flat_racks_2_pin_arms";

Layout Load() {
  const char* dir = std::getenv("LAYOUT_DIR");
  std::string error;
  const std::optional<Layout> layout = LoadLayout(
      std::string(dir != nullptr ? dir : "layouts") + "/" + kName + ".json",
      &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout.value_or(Layout{});
}

CheckReport RunChecks(const Layout& layout, std::vector<std::string> checks) {
  CheckOptions options;
  options.checks = std::move(checks);
  return CheckLayout(layout, ModelPaths::FromEnvironment(), options);
}

const CheckRow* Find(const CheckReport& r, const std::string& check,
                     const std::string& item) {
  for (const CheckRow& row : r.rows) {
    if (row.check == check && row.item == item) return &row;
  }
  return nullptr;
}

Item* ItemOf(Layout* l, const std::string& name) {
  for (Item& i : l->items) {
    if (i.name == name) return &i;
  }
  return nullptr;
}

Box* BoxOf(Layout* l, const std::string& item, const std::string& box) {
  Item* i = ItemOf(l, item);
  if (i == nullptr) return nullptr;
  for (Box& b : i->boxes) {
    if (b.name == box) return &b;
  }
  return nullptr;
}

bool Contains(const std::string& s, const std::string& what) {
  return s.find(what) != std::string::npos;
}

TEST(LayoutStageRows, TheWaysRowsExistAndPass) {
  const CheckReport r = RunChecks(Load(), {"ways"});
  ASSERT_TRUE(r.error.empty()) << r.error;
  for (const char* item :
       {"tube in", "stage 1: tube at rest", "stage 1 to 2",
        "stage 2: tube at rest", "stage 2 to 3", "stage 3: tube at rest",
        "bay out"}) {
    const CheckRow* row = Find(r, "ways", item);
    ASSERT_NE(row, nullptr) << item;
    EXPECT_TRUE(row->pass) << item << ": " << row->detail;
  }
  EXPECT_EQ(r.rows.size(), 7u);
  // The jaws 2 mm beside the tube at stage 1 are named, near by design.
  const CheckRow* rest = Find(r, "ways", "stage 1: tube at rest");
  EXPECT_TRUE(Contains(rest->detail, "item_infeed_unit_jaw_s2_neg"))
      << rest->detail;
  EXPECT_TRUE(Contains(rest->detail, "item_infeed_unit_jaw_s1_pos"))
      << rest->detail;
  // The outfeed head draws the bay out with it.
  const CheckRow* out = Find(r, "ways", "bay out");
  EXPECT_TRUE(Contains(out->detail, "item_outfeed_unit_head moves with the bay"))
      << out->detail;
}

TEST(LayoutStageRows, AUnitsBedInTheWayFailsTheWay) {
  Layout l = Load();
  // The infeed bed raised 0.5 m into the tube coming in.
  Box* in = BoxOf(&l, "infeed_unit", "bed");
  ASSERT_NE(in, nullptr);
  in->size[2] += 0.5;
  in->pos[2] += 0.25;
  // The outfeed bed raised into the built part it carries.
  Box* out = BoxOf(&l, "outfeed_unit", "bed");
  ASSERT_NE(out, nullptr);
  out->size[2] += 0.6;
  out->pos[2] += 0.3;
  const CheckReport r = RunChecks(l, {"ways"});
  const CheckRow* tube_in = Find(r, "ways", "tube in");
  ASSERT_NE(tube_in, nullptr);
  EXPECT_FALSE(tube_in->pass) << tube_in->detail;
  EXPECT_TRUE(Contains(tube_in->detail, "item_infeed_unit_bed"))
      << tube_in->detail;
  const CheckRow* move = Find(r, "ways", "stage 2 to 3");
  ASSERT_NE(move, nullptr);
  EXPECT_FALSE(move->pass) << move->detail;
  EXPECT_TRUE(Contains(move->detail, "item_outfeed_unit_bed")) << move->detail;
}

TEST(LayoutStageRows, TheTubeAtRestIsHeldToEveryItemsBoxes) {
  const CheckRow* ok = nullptr;
  {
    const CheckReport r = RunChecks(Load(), {"ways"});
    ok = Find(r, "ways", "stage 1: tube at rest");
    ASSERT_NE(ok, nullptr);
    EXPECT_TRUE(ok->pass);
  }
  // A new box of an item on the tube at stage 2's place.
  Layout on = Load();
  Item* outfeed = ItemOf(&on, "outfeed_unit");
  ASSERT_NE(outfeed, nullptr);
  Box block;
  block.name = "block";
  block.size = {0.1, 0.1, 0.1};
  block.pos = {-0.85, -2.0, 1.75};
  outfeed->boxes.push_back(block);
  const CheckReport a = RunChecks(on, {"ways"});
  const CheckRow* rest2 = Find(a, "ways", "stage 2: tube at rest");
  ASSERT_NE(rest2, nullptr);
  EXPECT_FALSE(rest2->pass) << rest2->detail;
  EXPECT_LT(rest2->figure, 0.0);
  EXPECT_TRUE(Contains(rest2->detail, "item_outfeed_unit_block"))
      << rest2->detail;
  // A jaw 1 mm into the tube at stage 1 (2 mm beside it by design).
  Layout jaw = Load();
  Box* j = BoxOf(&jaw, "infeed_unit", "jaw_s1_neg");
  ASSERT_NE(j, nullptr);
  j->pos[0] += 0.003;
  const CheckReport b = RunChecks(jaw, {"ways"});
  const CheckRow* rest1 = Find(b, "ways", "stage 1: tube at rest");
  ASSERT_NE(rest1, nullptr);
  EXPECT_FALSE(rest1->pass) << rest1->detail;
  EXPECT_LT(rest1->figure, 0.0) << rest1->detail;
  EXPECT_TRUE(Contains(rest1->detail, "item_infeed_unit_jaw_s1_neg"))
      << rest1->detail;
}

// A gap variant reruns the ports on rack b moved: the gap row's detail names
// a port row with its nominal and its variant figure (rack b's module arm 25
// mm nearer the zone of the bays, at gap 0 mm), and every row passes. A port
// zone cannot be made to catch rack b without the zone reaching over the
// footprint, which fails the nominal "zone ... off the base" row first, so
// the failing side is the gap row's rule that any failing rerun row fails it
// (src/layout_stages.cc, GapRow), which the unit test of the ways rows above
// shows at work on the bay out.
TEST(LayoutStageRows, TheGapVariantsRerunThePorts) {
  const CheckReport r = RunChecks(Load(), {"port", "gap"});
  ASSERT_TRUE(r.error.empty()) << r.error;
  int nominal_ports = 0, gap_rows = 0;
  for (const CheckRow& row : r.rows) {
    if (row.check == "port") {
      ++nominal_ports;
      EXPECT_TRUE(row.pass) << row.item << ": " << row.detail;
    }
    if (row.check == "gap") {
      ++gap_rows;
      EXPECT_TRUE(row.pass) << row.item << ": " << row.detail;
    }
  }
  EXPECT_GT(nominal_ports, 30);
  EXPECT_EQ(gap_rows, 8);
  const CheckRow* gap0 = Find(r, "gap", "gap 0 mm");
  ASSERT_NE(gap0, nullptr);
  EXPECT_TRUE(Contains(gap0->detail,
                       "port stowed clear of zone bays_out 0.1200 -> 0.0950 m"))
      << gap0->detail;
}

}  // namespace
}  // namespace cell::layout
