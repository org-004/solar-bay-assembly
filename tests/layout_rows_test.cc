// The allowed, port and flat rack transport rows (src/layout_rows.h): a
// layout that keeps every limit passes each row, and copies broken in memory,
// one thing at a time, fail the row that names it. On two racks the rows are
// per rack (envelope and mass) and one for the gap; the four 40 ft layouts'
// rows are the same as before.

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
  EXPECT_NE(Find(rows, "module arm module_arm"), nullptr);
  EXPECT_EQ(Find(rows, "module arms"), nullptr);
  EXPECT_EQ(Find(rows, "pin arms"), nullptr);
  EXPECT_EQ(Find(rows, "arms"), nullptr);
  EXPECT_NE(Find(rows, "pallet 0"), nullptr);
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
  for (const char* item : {"module arms", "pin arms", "arms"}) {
    EXPECT_EQ(Find(rows, item), nullptr) << item;
  }
  for (const ArmUse& a : FlatLayout().arms) {
    EXPECT_NE(Find(rows, "maker " + a.name), nullptr) << a.name;
  }
}

TEST(AllowedRows, KukaArmPassesOnTwoFlatRacks) {
  Layout l = FlatLayout();
  l.base = "two_flat_racks";
  l.arms[0].model = "kr210_r2700_2";
  const std::vector<CheckRow> rows = AllowedRows(l);
  const CheckRow* r = Find(rows, "maker " + l.arms[0].name);
  ASSERT_NE(r, nullptr);
  EXPECT_TRUE(r->pass) << r->detail;
  EXPECT_EQ(r->figure, 1.0);
  EXPECT_EQ(r->limit, 1.0);
}

TEST(AllowedRows, Ur10eFailsOnTwoFlatRacks) {
  Layout l = FlatLayout();
  l.base = "two_flat_racks";
  l.arms[1].model = "ur10e";
  const std::vector<CheckRow> rows = AllowedRows(l);
  ExpectFails(rows, "maker " + l.arms[1].name);
  EXPECT_EQ(Find(rows, "maker " + l.arms[1].name)->figure, 0.0);
}

TEST(AllowedRows, Ur10ePassesOnTheFlatRack) {
  Layout l = FlatLayout();
  ASSERT_EQ(l.base, "flat_rack");
  l.arms[1].model = "ur10e";
  const std::vector<CheckRow> rows = AllowedRows(l);
  const CheckRow* r = Find(rows, "maker " + l.arms[1].name);
  ASSERT_NE(r, nullptr);
  EXPECT_TRUE(r->pass) << r->detail;
  EXPECT_EQ(r->figure, 0.0);
  EXPECT_EQ(r->limit, 0.0);
  EXPECT_NE(r->detail.find("no limit"), std::string::npos) << r->detail;
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
        "on the deck item pin_lane"}) {
    EXPECT_NE(Find(rows, item), nullptr) << item;
  }
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

// ------------------------------------------------------------ two racks
Layout TwoRacks() { return Load("two_flat_racks_2_pin_arms"); }

TEST(TwoRackRows, TheLayoutPassesTransportAllowedAndPort) {
  const Layout l = TwoRacks();
  EXPECT_TRUE(Failures(FlatRackTransportRows(l, Paths())).empty())
      << Failures(FlatRackTransportRows(l, Paths()));
  EXPECT_TRUE(Failures(AllowedRows(l)).empty()) << Failures(AllowedRows(l));
  // As the checker makes them (src/layout_check.h, "Stages"): the port rows
  // at the first stage, but the bay's way out from the last, where it leaves.
  std::vector<CheckRow> ports = PortRows(l, Paths());
  const std::vector<CheckRow> last =
      PortRows(AtStage(l, l.staging->stages.back()), Paths());
  for (CheckRow& r : ports) {
    if (r.item == "way bay_out in a zone") r = *Find(last, r.item);
  }
  EXPECT_TRUE(Failures(ports).empty()) << Failures(ports);
}

TEST(TwoRackRows, PerRackRowsReplaceTheFortyFootOnes) {
  const std::vector<CheckRow> rows = FlatRackTransportRows(TwoRacks(), Paths());
  for (const char* item : {"rack a envelope", "rack b envelope", "rack a mass",
                           "rack b mass", "nothing spans the gap"}) {
    EXPECT_NE(Find(rows, item), nullptr) << item;
  }
  for (const char* item : {"stowed x", "stowed y", "stowed top",
                           "stowed bottom", "mass"}) {
    EXPECT_EQ(Find(rows, item), nullptr) << item;
  }
  EXPECT_DOUBLE_EQ(Find(rows, "rack a mass")->limit, 13608.0);
  EXPECT_DOUBLE_EQ(Find(rows, "rack b mass")->limit, 13608.0);
}

TEST(TwoRackRows, MassIsSplitByRack) {
  const Layout l = TwoRacks();
  double a = 0.0, b = 0.0, all = 0.0;
  for (const MassPart& p : TransportMassParts(l)) {
    EXPECT_TRUE(p.rack == "a" || p.rack == "b") << p.name;
    // Stocks inside are infed parts and are not in the row (docs/scorecard.md).
    if (p.component == "stocks inside") continue;
    (p.rack == "a" ? a : b) += p.mass_kg;
    all += p.mass_kg;
  }
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  EXPECT_NEAR(Find(rows, "rack a mass")->figure, a, 1e-9);
  EXPECT_NEAR(Find(rows, "rack b mass")->figure, b, 1e-9);
  EXPECT_GT(a, 0.0);
  EXPECT_GT(b, 0.0);
  EXPECT_NEAR(a + b, all, 1e-9);
  // The module arm and its track are on rack b, the holder on rack a.
  EXPECT_GT(b, 2000.0);
  // A heavy item on rack a fails rack a's row only.
  Layout heavy = l;
  for (Item& i : heavy.items) {
    if (i.rack == "a") i.mass_kg = 20000.0;
  }
  const std::vector<CheckRow> h = FlatRackTransportRows(heavy, Paths());
  ExpectFails(h, "rack a mass");
  EXPECT_TRUE(Find(h, "rack b mass")->pass);
}

TEST(TwoRackRows, EnvelopeIsFromTheGroundWithAMarginOnEachSide) {
  const std::vector<CheckRow> rows = FlatRackTransportRows(TwoRacks(), Paths());
  const CheckRow* a = Find(rows, "rack a envelope");
  ASSERT_NE(a, nullptr);
  EXPECT_GT(a->figure, 0.0) << a->detail;  // Not pinned at the deck.
  for (const char* word : {"across", "along", "top"}) {
    EXPECT_NE(a->detail.find(word), std::string::npos) << a->detail;
  }
  EXPECT_NE(Find(rows, "rack a above the deck"), nullptr);
  EXPECT_TRUE(Find(rows, "rack b above the deck")->pass);
  // The visual meshes of an arm count: a KR 210 base pushed to the gap's edge
  // fails rack b's envelope.
  Layout l = TwoRacks();
  l.arms[0].base_pos[0] = 0.2;
  l.arms[0].track->beam.pos[0] = 0.2;
  ExpectFails(FlatRackTransportRows(l, Paths()), "nothing spans the gap");
  EXPECT_TRUE(Find(FlatRackTransportRows(l, Paths()), "rack b envelope")->figure <
              Find(rows, "rack b envelope")->figure);
}

TEST(TwoRackRows, OneRackLayoutsHaveNoRackInTheirParts) {
  for (const MassPart& p : TransportMassParts(FlatLayout())) {
    EXPECT_TRUE(p.rack.empty()) << p.name;
  }
}

TEST(TwoRackRows, AThingOutsideItsRacksEnvelopeFails) {
  Layout l = TwoRacks();
  // Rack a's holder moved past rack a's +Y end, still on the pair's deck line.
  for (Box& b : l.holder.boxes) b.pos[1] += 2.0;
  std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "rack a envelope");
  EXPECT_NE(Find(rows, "rack a envelope")->detail.find("holder"), std::string::npos);
  EXPECT_TRUE(Find(rows, "rack b envelope")->pass);
  // Taller than the envelope.
  l = TwoRacks();
  l.holder.boxes[0].size[2] += 3.0;
  l.holder.boxes[0].pos[2] += 1.5;
  rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "rack a envelope");
  // A thing of rack b standing over rack a's side fails rack b's row.
  l = TwoRacks();
  l.arms[0].track->beam.pos[0] -= 1.0;
  l.arms[0].base_pos[0] -= 1.0;
  rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "rack b envelope");
  ExpectFails(rows, "on the deck beam module_arm");
}

TEST(TwoRackRows, AThingOnTheWrongRackIsNotOnItsDeck) {
  Layout l = TwoRacks();
  l.holder.rack = "b";  // Its boxes are over rack a.
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "on the deck holder");
}

TEST(TwoRackRows, NothingSpansTheGap) {
  Layout l = TwoRacks();
  // Stretch the lane across the gap.
  for (Item& i : l.items) {
    if (i.name != "pin_lane") continue;
    i.boxes[0].size[0] = 0.5;
    i.boxes[0].pos[0] = 0.0;
  }
  std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "nothing spans the gap");
  EXPECT_LT(Find(rows, "nothing spans the gap")->figure, 0.0);
  EXPECT_NE(Find(rows, "nothing spans the gap")->detail.find("pin_lane"),
            std::string::npos);
  // With rack b moved to the gap's tolerance the band moves with it.
  const Layout wide = MovedRackB(TwoRacks(), 0.025, 0.025);
  rows = FlatRackTransportRows(wide, Paths());
  EXPECT_TRUE(Failures(rows).empty()) << Failures(rows);
}

TEST(TwoRackRows, PortsLieOnThePairsOutline) {
  Layout l = TwoRacks();
  PortOf(l, "modules").frame_pos[0] = 1.219;  // Where rack b's own outline is.
  ExpectFails(PortRows(l, Paths()), "frame modules on the outline");
  l = TwoRacks();
  PortOf(l, "tubes_in").frame_pos[1] = 3.5;  // Past the pair's +Y end.
  ExpectFails(PortRows(l, Paths()), "frame tubes_in on the outline");
  // A zone over the gap between the racks reaches over the footprint.
  l = TwoRacks();
  PortOf(l, "pin_magazines").zone.pos[1] -= 1.0;
  ExpectFails(PortRows(l, Paths()), "zone pin_magazines off the base");
}

TEST(TwoRackLoader, TheLayoutLoadsAsTwoRacks) {
  const Layout l = TwoRacks();
  EXPECT_EQ(l.base, "two_flat_racks");
  EXPECT_TRUE(OnFlatRacks(l));
  EXPECT_EQ(l.staging->stages.size(), 3u);
  for (const ArmUse& a : l.arms) EXPECT_FALSE(a.rack.empty()) << a.name;
}

}  // namespace
}  // namespace cell::layout
