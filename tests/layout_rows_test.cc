// The allowed, port and flat rack transport rows (src/layout_rows.h): a
// layout that keeps every limit passes each row, and copies broken in memory,
// one thing at a time, fail the row that names it.

#include "src/layout_rows.h"

#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "src/container.h"
#include "src/flat_rack.h"
#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"

namespace cell::layout {
namespace {

constexpr double kRaise = flat_rack::kDeckHeight - container::kFloorHeight;

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

Port MakePort(const std::string& name, const std::string& crosses,
              const Vec3& frame, const Vec3& lo, const Vec3& hi,
              std::vector<std::string> hands_over) {
  Port p;
  p.name = name;
  p.crosses = crosses;
  p.frame_pos = frame;
  p.zone.name = name;
  for (int a = 0; a < 3; ++a) {
    p.zone.size[a] = hi[a] - lo[a];
    p.zone.pos[a] = (hi[a] + lo[a]) / 2.0;
  }
  p.hands_over = std::move(hands_over);
  return p;
}

// The reference layout of run 011 with its base changed to the flat rack in
// memory: everything on the deck is raised by the deck's extra height, and
// ports are put around the pallet, the stocks and the transfer cart.
Layout FlatLayout() {
  Layout l = Load("below_rails_by_module_arm");
  l.base = "flat_rack";
  l.flat_rack.end_frames = "taken_off";
  l.tube.axis_z += kRaise;
  for (Box& b : l.holder.boxes) b.pos[2] += kRaise;
  for (ArmUse& a : l.arms) {
    a.base_pos[2] += kRaise;
    if (a.track) a.track->beam.pos[2] += kRaise;
  }
  for (Stock& s : l.stocks) {
    if (!s.outside) {
      for (Box& b : s.boxes) b.pos[2] += kRaise;
    }
  }
  for (Item& i : l.items) {
    if (i.travels_inside) {
      for (Box& b : i.boxes) b.pos[2] += kRaise;
    }
  }
  const double deck = flat_rack::kDeckHeight;
  l.ports = {
      MakePort("pallet_side", "modules", {1.219, 0.0, deck}, {1.3, -1.4, -0.1},
               {4.4, 1.4, 3.0}, {"pallet:0"}),
      MakePort("rail_rack", "rails", {1.219, 1.75, deck}, {1.3, 1.45, -0.1},
               {3.4, 2.2, 2.0}, {"stock:rails"}),
      MakePort("pin_rack", "pin_magazines", {0.15, 6.096, deck},
               {-0.4, 6.1, -0.1}, {0.6, 6.8, 2.0}, {"stock:pins"}),
      MakePort("tubes_in", "tubes", {-1.219, 0.0, deck}, {-5.5, -6.5, -0.1},
               {-1.25, 6.5, 2.5}, {"item:transfer_cart", "way:tube_in"}),
      MakePort("bays_out", "bays", {-1.219, 0.0, deck}, {-5.5, -6.5, -0.1},
               {-1.25, 6.5, 2.5}, {"way:bay_out"}),
  };
  l.ports[3].shares_zone_with = "bays_out";
  l.ports[4].shares_zone_with = "tubes_in";
  return l;
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

TEST(PortRows, ReferencePasses) {
  const std::vector<CheckRow> rows = PortRows(FlatLayout(), Paths());
  EXPECT_TRUE(Failures(rows).empty()) << Failures(rows);
  EXPECT_NE(Find(rows, "tubes and bays are two ports"), nullptr);
  EXPECT_NE(Find(rows, "frame tubes_in on the outline"), nullptr);
  EXPECT_NE(Find(rows, "zone pin_rack off the base"), nullptr);
  EXPECT_NE(Find(rows, "zones pallet_side and rail_rack apart"), nullptr);
  EXPECT_NE(Find(rows, "pallet 0 in a zone"), nullptr);
  EXPECT_NE(Find(rows, "stock pins in a zone"), nullptr);
  EXPECT_NE(Find(rows, "item transfer_cart in a zone"), nullptr);
  EXPECT_NE(Find(rows, "way tube_in in a zone"), nullptr);
  EXPECT_NE(Find(rows, "way bay_out in a zone"), nullptr);
  EXPECT_NE(Find(rows, "stowed clear of zone bays_out"), nullptr);
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
  l.ports[2].zone.pos[1] = 5.0;
  ExpectFails(PortRows(l, Paths()), "zone pin_rack off the base");
}

TEST(PortRows, ZonesOverlapping) {
  Layout l = FlatLayout();
  l.ports[1].zone.pos = {2.5, 0.5, 1.0};
  ExpectFails(PortRows(l, Paths()), "zones pallet_side and rail_rack apart");
}

TEST(PortRows, ShareNamedByOneOnly) {
  Layout l = FlatLayout();
  l.ports[4].shares_zone_with = "";
  ExpectFails(PortRows(l, Paths()), "zones tubes_in and bays_out apart");
}

TEST(PortRows, FrameOffTheOutline) {
  Layout l = FlatLayout();
  l.ports[0].frame_pos[0] += 0.1;
  ExpectFails(PortRows(l, Paths()), "frame pallet_side on the outline");
}

TEST(PortRows, StockNamedByNoPort) {
  Layout l = FlatLayout();
  l.ports[1].hands_over.clear();
  const std::vector<CheckRow> rows = PortRows(l, Paths());
  ExpectFails(rows, "stock rails in a zone");
  EXPECT_EQ(Find(rows, "stock rails in a zone")->figure, -1.0);
}

TEST(PortRows, StockPickPointOutside) {
  Layout l = FlatLayout();
  l.stocks[1].pick_pos[1] = 7.5;
  ExpectFails(PortRows(l, Paths()), "stock pins in a zone");
}

TEST(PortRows, StowedCellInAZone) {
  Layout l = FlatLayout();
  l.ports[3].zone.pos[0] = -1.0;
  l.ports[3].zone.size[0] = 2.0;
  l.ports[4].zone = l.ports[3].zone;
  ExpectFails(PortRows(l, Paths()), "stowed clear of zone tubes_in");
}

TEST(FlatRackTransportRows, ReferencePasses) {
  const std::vector<CheckRow> rows = FlatRackTransportRows(FlatLayout(), Paths());
  EXPECT_TRUE(Failures(rows).empty()) << Failures(rows);
  for (const char* item : {"stowed x", "stowed y", "stowed top",
                           "stowed bottom", "stow track module_arm", "mass",
                           "on the deck holder", "on the deck beam module_arm",
                           "closed container x", "closed container y",
                           "closed container top", "between end frames y"}) {
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
}

TEST(FlatRackTransportRows, HolderBoxLifted) {
  Layout l = FlatLayout();
  l.holder.boxes[0].pos[2] += 0.05;
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "on the deck holder");
  EXPECT_GE(Find(rows, "on the deck holder")->figure, 1.0);
}

TEST(FlatRackTransportRows, BeamWidenedPastTheEnvelope) {
  Layout l = FlatLayout();
  l.arms[0].track->beam.size[0] = 3.0;
  const std::vector<CheckRow> rows = FlatRackTransportRows(l, Paths());
  ExpectFails(rows, "stowed x");
  // The closed container's row names it and still passes.
  const CheckRow* closed = Find(rows, "closed container x");
  ASSERT_NE(closed, nullptr);
  EXPECT_TRUE(closed->pass);
  EXPECT_LT(closed->figure, 0.0);
  EXPECT_NE(closed->detail.find("outside"), std::string::npos) << closed->detail;
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

}  // namespace
}  // namespace cell::layout
