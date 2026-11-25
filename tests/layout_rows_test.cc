// The allowed, port and flat rack transport rows (src/layout_rows.h): a
// layout that keeps every limit passes each row, and copies broken in memory,
// one thing at a time, fail the row that names it.

#include "src/layout_rows.h"

#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/flat_rack.h"
#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"

namespace cell::layout {
namespace {

std::string Dir() {
  const char* dir = std::getenv("LAYOUT_DIR");
  return dir != nullptr ? dir : "layouts";
}

ModelPaths Paths() { return ModelPaths::FromEnvironment(); }

Layout Load(const std::string& name) {
  std::string error;
  const std::optional<Layout> layout =
      LoadLayout(Dir() + "/" + name + ".json", &error);
  EXPECT_TRUE(layout.has_value()) << error;
  return layout.value_or(Layout{});
}

Layout FlatLayout() { return Load("flat_rack_3_pin_arms"); }

Port& PortOf(Layout& l, const std::string& name) {
  for (Port& p : l.ports) {
    if (p.name == name) return p;
  }
  ADD_FAILURE() << "no port " << name;
  return l.ports[0];
}

const CheckRow* Find(const std::vector<CheckRow>& rows,
                     const std::string& item) {
  for (const CheckRow& r : rows) {
    if (r.item == item) return &r;
  }
  return nullptr;
}

std::string Failures(const std::vector<CheckRow>& rows) {
  std::string out;
  for (const CheckRow& r : rows) {
    if (!r.pass) {
      out += r.check + " | " + r.item + " | " + std::to_string(r.figure) +
             " | " + r.detail + "\n";
    }
  }
  return out;
}

void ExpectFails(const std::vector<CheckRow>& rows, const std::string& item) {
  const CheckRow* r = Find(rows, item);
  ASSERT_NE(r, nullptr) << "no row " << item;
  EXPECT_FALSE(r->pass) << item << ": " << r->detail;
}

TEST(AllowedRows, ReferencePasses) {
  const Layout l = Load("below_rails_by_module_arm");
  const std::vector<CheckRow> rows = AllowedRows(l);
  EXPECT_TRUE(Failures(rows).empty()) << Failures(rows);
  ASSERT_NE(Find(rows, "module arms"), nullptr);
  EXPECT_EQ(Find(rows, "module arms")->figure, 1.0);
  EXPECT_NE(Find(rows, "module arm module_arm"), nullptr);
  EXPECT_NE(Find(rows, "pin arms"), nullptr);
  EXPECT_NE(Find(rows, "arms"), nullptr);
  EXPECT_NE(Find(rows, "pallet 0"), nullptr);
}

TEST(AllowedRows, ThirdModuleArmFails) {
  Layout l = Load("below_rails_by_module_arm");
  ArmUse extra = l.arms[1];
  extra.name = "third";
  l.arms.push_back(extra);
  Work w;
  w.kind = "module_set";
  w.arm = "third";
  w.tool = extra.first_tool;
  w.points = "0";
  l.work.push_back(w);
  l.work.push_back(w);
  // The small arm takes a point as well, so that three arms move modules.
  w.arm = "small_arm";
  w.tool = l.arms[1].first_tool;
  w.kind = "module_take";
  l.work.push_back(w);
  const std::vector<CheckRow> rows = AllowedRows(l);
  ExpectFails(rows, "module arms");
  EXPECT_EQ(Find(rows, "module arms")->figure, 3.0);
}

TEST(AllowedRows, SixArmsFails) {
  Layout l = Load("below_rails_by_module_arm");
  for (int i = 0; i < 4; ++i) {
    ArmUse extra = l.arms[1];
    extra.name = "extra" + std::to_string(i);
    l.arms.push_back(extra);
  }
  ExpectFails(AllowedRows(l), "arms");
}

TEST(AllowedRows, FlatPalletFails) {
  Layout l = Load("below_rails_by_module_arm");
  l.pallets[0].stands = "flat";
  ExpectFails(AllowedRows(l), "pallet 0");
}

TEST(AllowedRows, UnknownMakerFails) {
  Layout l = Load("below_rails_by_module_arm");
  for (ArmUse& a : l.arms) {
    if (a.name == "module_arm") a.model = "ur10e";
  }
  ExpectFails(AllowedRows(l), "module arm module_arm");
}

TEST(AllowedRows, FlatRackLayoutPasses) {
  const std::vector<CheckRow> rows = AllowedRows(FlatLayout());
  EXPECT_TRUE(Failures(rows).empty()) << Failures(rows);
  ASSERT_NE(Find(rows, "pin arms"), nullptr);
  EXPECT_EQ(Find(rows, "pin arms")->figure, 3.0);
  ASSERT_NE(Find(rows, "arms"), nullptr);
  EXPECT_EQ(Find(rows, "arms")->figure, 4.0);
}

TEST(PortRows, ReferencePasses) {
  const std::vector<CheckRow> rows = PortRows(FlatLayout(), Paths());
  EXPECT_TRUE(Failures(rows).empty()) << Failures(rows);
  for (const char* item :
       {"tubes and bays are two ports", "frame tubes_in on the outline",
        "zone pin_magazines off the base", "zones modules and rails apart",
        "zones tubes_in and bays_out apart", "pallet 0 in a zone",
        "stock rails in a zone", "item transfer_cart in a zone",
        "item pin_feeder in a zone", "way tube_in in a zone",
        "way bay_out in a zone", "stowed clear of zone bays_out"}) {
    EXPECT_NE(Find(rows, item), nullptr) << item;
  }
  EXPECT_EQ(Find(rows, "item pin_lane in a zone"), nullptr);
  // A thing on the ground shows its horizontal margin, not the ground's 0.
  const CheckRow* pallet = Find(rows, "pallet 0 in a zone");
  ASSERT_NE(pallet, nullptr);
  EXPECT_GT(pallet->figure, 0.0) << pallet->detail;
}

TEST(PortRows, PalletHalfOutOfItsZone) {
  Layout l = FlatLayout();
  l.pallets[0].pos[0] += 1.6;
  l.pallet = l.pallets[0];
  const std::vector<CheckRow> rows = PortRows(l, Paths());
  ExpectFails(rows, "pallet 0 in a zone");
  EXPECT_LT(Find(rows, "pallet 0 in a zone")->figure, 0.0);
}

TEST(PortRows, ZoneOverTheBase) {
  Layout l = FlatLayout();
  PortOf(l, "pin_magazines").zone.pos[1] = 5.0;
  ExpectFails(PortRows(l, Paths()), "zone pin_magazines off the base");
}

TEST(PortRows, ZonesOverlapping) {
  Layout l = FlatLayout();
  PortOf(l, "rails").zone = PortOf(l, "modules").zone;
  ExpectFails(PortRows(l, Paths()), "zones modules and rails apart");
}

TEST(PortRows, ShareNamedByOneOnly) {
  Layout l = FlatLayout();
  PortOf(l, "bays_out").shares_zone_with = "";
  ExpectFails(PortRows(l, Paths()), "zones tubes_in and bays_out apart");
}

TEST(PortRows, FrameOffTheOutline) {
  Layout l = FlatLayout();
  PortOf(l, "modules").frame_pos[0] += 0.1;
  ExpectFails(PortRows(l, Paths()), "frame modules on the outline");
}

TEST(PortRows, StockNamedByNoPort) {
  Layout l = FlatLayout();
  PortOf(l, "rails").hands_over.clear();
  const std::vector<CheckRow> rows = PortRows(l, Paths());
  ExpectFails(rows, "stock rails in a zone");
  EXPECT_EQ(Find(rows, "stock rails in a zone")->figure, -1.0);
}

TEST(PortRows, StockPickPointOutside) {
  Layout l = FlatLayout();
  for (Stock& s : l.stocks) {
    if (s.name == "rails") s.pick_pos[1] = 7.5;
  }
  ExpectFails(PortRows(l, Paths()), "stock rails in a zone");
}

TEST(PortRows, StowedCellInAZone) {
  Layout l = FlatLayout();
  PortOf(l, "tubes_in").zone.pos[0] = -1.0;
  PortOf(l, "bays_out").zone = PortOf(l, "tubes_in").zone;
  ExpectFails(PortRows(l, Paths()), "stowed clear of zone tubes_in");
}

TEST(PortRows, AThingOnTheGroundMayTouchTheZoneFoot) {
  // The cart stands on the ground; moving the zone's floor up to 0.1 m puts
  // the cart below the zone's underside, which fails.
  Layout l = FlatLayout();
  Port& p = PortOf(l, "pin_magazines");
  p.zone.pos[2] += 0.1;
  p.zone.size[2] -= 0.2;
  ExpectFails(PortRows(l, Paths()), "item pin_feeder in a zone");
}

TEST(FlatRackTransportRows, ReferencePasses) {
  const std::vector<CheckRow> rows = FlatRackTransportRows(FlatLayout(), Paths());
  EXPECT_TRUE(Failures(rows).empty()) << Failures(rows);
  for (const char* item :
       {"stowed x", "stowed y", "stowed top", "stowed bottom",
        "stow track module_arm", "mass", "on the deck holder",
        "on the deck beam module_arm", "on the deck beam pin_arm_a",
        "on the deck item pin_lane", "closed container x",
        "closed container y", "closed container top", "between end frames y"}) {
    EXPECT_NE(Find(rows, item), nullptr) << item;
  }
  for (const char* item : {"closed container x", "closed container y",
                           "closed container top", "between end frames y"}) {
    const CheckRow* r = Find(rows, item);
    ASSERT_NE(r, nullptr);
    EXPECT_TRUE(r->pass);
    EXPECT_EQ(r->limit, 0.0);
    EXPECT_EQ(r->detail.rfind("no limit: ", 0), 0u) << r->detail;
  }
  const CheckRow* x = Find(rows, "closed container x");
  EXPECT_LT(x->figure, 0.0);
  EXPECT_NE(x->detail.find("outside"), std::string::npos) << x->detail;
}

TEST(FlatRackTransportRows, HolderBoxLifted) {
  Layout l = FlatLayout();
  l.holder.boxes[0].pos[2] += 0.05;
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "on the deck holder");
  EXPECT_GE(Find(rows, "on the deck holder")->figure, 1.0);
}

TEST(FlatRackTransportRows, NestLiftedOffTheLane) {
  Layout l = FlatLayout();
  for (Item& i : l.items) {
    if (i.name == "pin_lane") i.boxes[1].pos[2] += 0.05;
  }
  ExpectFails(FlatRackTransportRows(l, Paths()), "on the deck item pin_lane");
}

TEST(FlatRackTransportRows, BeamWidenedPastTheEnvelope) {
  Layout l = FlatLayout();
  l.arms[0].track->beam.size[0] = 3.0;
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "stowed x");
  const CheckRow* closed = Find(rows, "closed container x");
  ASSERT_NE(closed, nullptr);
  EXPECT_TRUE(closed->pass);
  EXPECT_LT(closed->figure, 0.0);
}

TEST(FlatRackTransportRows, HeavyItem) {
  Layout l = FlatLayout();
  Item ballast;
  ballast.name = "ballast";
  ballast.travels_inside = true;
  ballast.mass_kg = 20000.0;
  l.items.push_back(ballast);
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "mass");
  ASSERT_NE(Find(rows, "on the deck item ballast"), nullptr);
  EXPECT_TRUE(Find(rows, "on the deck item ballast")->pass);
}

TEST(FlatRackTransportRows, StowTrackOutside) {
  Layout l = FlatLayout();
  l.arms[0].stow_track = 99.0;
  ExpectFails(FlatRackTransportRows(l, Paths()), "stow track module_arm");
}

TEST(FlatRackLayouts, AllThreePassEveryRow) {
  for (const char* name : {"flat_rack_1_pin_arm", "flat_rack_2_pin_arms",
                           "flat_rack_3_pin_arms"}) {
    const Layout l = Load(name);
    std::vector<CheckRow> rows = AllowedRows(l);
    for (const CheckRow& r : PortRows(l, Paths())) rows.push_back(r);
    for (const CheckRow& r : FlatRackTransportRows(l, Paths())) rows.push_back(r);
    EXPECT_GT(rows.size(), 30u) << name;
    EXPECT_TRUE(Failures(rows).empty()) << name << "\n" << Failures(rows);
  }
}

TEST(AllowedRows, FourPinArmsFail) {
  Layout l = FlatLayout();
  ArmUse extra = l.arms[3];
  extra.name = "pin_arm_d";
  l.arms.push_back(extra);
  Work w;
  w.kind = "locate";
  w.arm = "pin_arm_d";
  w.tool = extra.first_tool;
  w.points = "0";
  l.work.push_back(w);
  const std::vector<CheckRow> rows = AllowedRows(l);
  ExpectFails(rows, "pin arms");
  EXPECT_EQ(Find(rows, "pin arms")->figure, 4.0);
}

TEST(AllowedRows, NoModuleArmFails) {
  Layout l = FlatLayout();
  std::vector<Work> kept;
  for (const Work& w : l.work) {
    if (w.kind != "module_take" && w.kind != "module_set") kept.push_back(w);
  }
  l.work = kept;
  ExpectFails(AllowedRows(l), "module arms");
}

TEST(PortRows, BaysPortRemoved) {
  Layout l = FlatLayout();
  std::vector<Port> kept;
  for (const Port& p : l.ports) {
    if (p.name != "bays_out") kept.push_back(p);
  }
  l.ports = kept;
  PortOf(l, "tubes_in").shares_zone_with = "";
  ExpectFails(PortRows(l, Paths()), "tubes and bays are two ports");
}

TEST(PortRows, WaysOutsideTheirZone) {
  Layout l = FlatLayout();
  PortOf(l, "tubes_in").zone.size[1] = 2.0;
  PortOf(l, "bays_out").zone = PortOf(l, "tubes_in").zone;
  const std::vector<CheckRow> rows = PortRows(l, Paths());
  ExpectFails(rows, "way tube_in in a zone");
  ExpectFails(rows, "way bay_out in a zone");
}

TEST(PortRows, ThingNamedByTwoPorts) {
  Layout l = FlatLayout();
  PortOf(l, "rails").hands_over.push_back("pallet:0");
  const std::vector<CheckRow> rows = PortRows(l, Paths());
  ExpectFails(rows, "pallet 0 in a zone");
  EXPECT_EQ(Find(rows, "pallet 0 in a zone")->figure, -1.0);
}

TEST(PortRows, ZonesStackedOverTheSameGround) {
  Layout l = FlatLayout();
  Port& rails = PortOf(l, "rails");
  rails.zone.pos = PortOf(l, "modules").zone.pos;
  rails.zone.pos[2] += 5.0;
  ExpectFails(PortRows(l, Paths()), "zones modules and rails apart");
}

TEST(PortRows, FrameFacesAwayFromItsZone) {
  Layout l = FlatLayout();
  PortOf(l, "modules").zone.pos[0] = -2.86;
  ExpectFails(PortRows(l, Paths()), "frame modules faces its zone");
}

TEST(PortRows, PortHandsOverWhatDoesNotCross) {
  Layout l = FlatLayout();
  PortOf(l, "modules").hands_over.push_back("stock:rails");
  PortOf(l, "tubes_in").hands_over = {"item:transfer_cart"};
  const std::vector<CheckRow> rows = PortRows(l, Paths());
  ExpectFails(rows, "port modules hands over what crosses");
  ExpectFails(rows, "port tubes_in hands over what crosses");
}

TEST(FlatRackTransportRows, StowedYTopAndBottom) {
  Layout l = FlatLayout();
  l.holder.boxes[0].pos[1] = 7.0;
  ExpectFails(FlatRackTransportRows(l, Paths()), "stowed y");
  l = FlatLayout();
  l.holder.boxes[0].pos[2] += 3.0;
  ExpectFails(FlatRackTransportRows(l, Paths()), "stowed top");
  l = FlatLayout();
  l.holder.boxes[0].pos[2] -= 0.3;
  ExpectFails(FlatRackTransportRows(l, Paths()), "stowed bottom");
}

TEST(FlatRackTransportRows, BoxOutsideTheFootprintIsNotCarried) {
  Layout l = FlatLayout();
  l.holder.boxes[0].pos[1] = 7.0;
  ExpectFails(FlatRackTransportRows(l, Paths()), "on the deck holder");
}

TEST(FlatRackTransportRows, BeamLifted) {
  Layout l = FlatLayout();
  l.arms[0].track->beam.pos[2] += 0.05;
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "on the deck beam module_arm");
  ExpectFails(rows, "on the deck arm module_arm");
}

TEST(FlatRackTransportRows, ArmNotOnItsCarriage) {
  Layout l = FlatLayout();
  l.arms[1].base_pos[2] += 0.05;
  ExpectFails(FlatRackTransportRows(l, Paths()), "on the deck arm pin_arm_a");
  l = FlatLayout();
  l.arms[1].track->carriage.pos[1] = 20.0;
  ExpectFails(FlatRackTransportRows(l, Paths()), "on the deck arm pin_arm_a");
  l = FlatLayout();
  l.arms[1].track.reset();
  ExpectFails(FlatRackTransportRows(l, Paths()), "on the deck arm pin_arm_a");
}

TEST(FlatRackTransportRows, CarriageRunsOffTheBeamsEnd) {
  Layout l = FlatLayout();
  l.arms[1].track->hi += 3.0;
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "on the deck arm pin_arm_a");
  EXPECT_NE(Find(rows, "on the deck arm pin_arm_a")->detail.find("hi"),
            std::string::npos);
}

TEST(FlatRackTransportRows, NoLimitRowNamesEveryGeom) {
  Layout l = FlatLayout();
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  const CheckRow* x = Find(rows, "closed container x");
  ASSERT_NE(x, nullptr);
  EXPECT_EQ(x->detail.find('\n'), std::string::npos);
  // 14 geoms stand outside today; the detail has a name for each.
  size_t metres = 0;
  for (size_t at = x->detail.find(" m"); at != std::string::npos;
       at = x->detail.find(" m", at + 1)) {
    ++metres;
  }
  EXPECT_GT(metres, 5u) << x->detail;
}

}  // namespace
}  // namespace cell::layout
