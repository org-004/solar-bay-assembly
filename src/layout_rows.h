// The rows of a layout's check that need no pose of an arm: what is
// allowed, the ports, and the transport rows of a layout on the flat rack.
//
// These are part of the checker (src/layout_check.h, whose opening comment
// lists every kind of row); CheckLayout calls them and //:check_layout
// prints them with the rest. They are in a file of their own because they
// read only the layout and its model at rest.
//
//   allowed    the limits the owner set on 2026-11-24 for the first cell, one
//              row each: that each arm that takes or sets modules is a KUKA
//              KR 210 R2700-2 or a smaller KUKA arm of src/arm_catalog.h, and
//              that every pallet is the maker's, 36 modules standing on their
//              long edge. There is no limit on the number of arms, as long as
//              the cell works and the stowed cell keeps to the weight limit
//              and the stow envelope. Since 2026-11-26, one row per arm, in
//              the layout's order, "maker <arm>": the arm's maker, 1 if KUKA
//              and 0 if not. It is a limit of 1 on every layout, whatever
//              its base (every arm is KUKA, the owner, 2026-11-26), except
//              the layouts that had a UR10e at 9d6efb2, named in one list in
//              layout_rows.cc (MakerUnlimited): there it is a row without a
//              limit that always passes, until a later spec replaces them.
//   port       a separate unit stands against the cell only at a port. Rows:
//              that tubes and bays are two ports; for each port, that its
//              frame is on the outline of the base's footprint (at the deck's
//              height; for two racks the pair's outer rectangle at the
//              nominal gap), that its zone's centre lies on the +x side of the
//              frame (the frame faces its zone), and that it hands over what
//              crosses it (tubes: way:tube_in; bays: way:bay_out; modules:
//              pallets only; rails: stock:rails; pin magazines: a stock of
//              pins or an item); that no zone reaches over the footprint;
//              that two zones are apart unless each says it shares the
//              other's, which only a port for tubes and a port for bays may,
//              with the same box; that every pallet, every stock and item
//              that does not travel, and the tube at the first offset of its
//              way in and the bay at the last offset of its way out, lie
//              wholly inside the zone of exactly one port that names them
//              (a stock's pick point too); and that in the stowed state no
//              geom of the cell is in a zone. Zones are boxes, but two are
//              apart or overlapping by their rectangles in the plane (x and
//              y) alone, so zones stacked over the same ground are not
//              apart. A pallet is always outside: the loader refuses one that
//              is not on the flat rack, and a holder that does not travel,
//              which it refuses too, is not looked at here. A thing standing
//              on the ground at the foot of its zone is not held by the
//              zone's underside; only a thing below it fails there.
//   transport  for a layout on the flat rack (src/flat_rack.h): stowed, every
//              geom of the cell inside the transport envelope across, along
//              and at the top, and above the deck; everything that travels
//              and does not move standing on the deck: the holder, each
//              arm's beam, the stocks and items that travel, each by its
//              boxes, a box being carried when it lies inside the base's
//              footprint and its underside is on the deck or it touches a
//              carried box of the same thing; each arm on the deck: with a
//              track, its carriage on its beam at both ends of the track (along the
//              track within the beam's length, across it only overlapping
//              the beam, its underside on the beam's top), and the base
//              on the carriage, without one, the base on the deck; each
//              arm's stow track value inside its track; the mass within what
//              the flat rack leaves for the cell. The row's detail names
//              every geom that stands outside.
//              On two racks (base "two_flat_racks"), in place of the 40 ft
//              envelope and mass rows: "rack a envelope" and "rack b
//              envelope", every stowed geom (visual meshes too) of what
//              stands on that rack inside that rack's 6.058 x 2.438 x 2.896 m
//              box from the ground at its place, the figure being the least
//              margin across, along and at the top, the detail giving each
//              with its geom; "rack a above the deck" and "rack b above the
//              deck", the lowest geom against the deck; "rack a mass" and "rack b mass": what stands on
//              it against 13,608 kg, stocks inside left out as infed parts; "nothing spans the gap": no stowed geom
//              of the cell lies in the band between the racks, the figure
//              the least distance of any from it. The rows on the deck keep
//              to the thing's own rack.
//
// The parts of the mass row are TransportMassParts below, so that the
// scorecard (src/scorecard.h) reads them from here and does not add them up
// a second time.
//
// src/layout_rows.cc says exactly what each row measures.

#ifndef SRC_LAYOUT_ROWS_H_
#define SRC_LAYOUT_ROWS_H_

#include <string>
#include <vector>

#include "src/layout.h"
#include "src/layout_check.h"
#include "src/layout_model.h"

namespace cell::layout {

// The `allowed` rows. Needs no model.
std::vector<CheckRow> AllowedRows(const Layout& layout);

// The `port` rows. Builds the layout's model itself, working and stowed; on
// a fault returns one failing row that says what could not be built.
std::vector<CheckRow> PortRows(const Layout& layout, const ModelPaths& paths);

// The `transport` rows of a layout whose base is "flat_rack" or
// "two_flat_racks", the rows without a limit included. Builds the stowed model itself.
std::vector<CheckRow> FlatRackTransportRows(const Layout& layout,
                                            const ModelPaths& paths);

// One thing that the transport row's mass adds up. `component` is the word
// of the row's detail it is summed under: "arms" (an arm by its catalogue
// mass, src/arm_catalog.h), "tracks" (an arm's track, beam and carriage),
// "tools" (each tool an arm has), "holder" (when it travels inside), "items
// inside" (each item that travels inside) or "stocks inside" (each stock
// that is not outside, by its mass_kg). `name` says which one: the arm's
// name, "<arm> track", "<arm> <tool>", "holder", or the item's or stock's
// name.
struct MassPart {
  std::string component;
  std::string name;
  double mass_kg = 0.0;
  // The rack ("a" or "b") the thing stands on, for a layout on two racks; ""
  // on every other base.
  std::string rack;
};

// The parts of the transport row's mass, in the order the row adds them:
// for each arm in the layout's order its arm, its track and its tools; then
// the holder, the items and the stocks. A part whose rule leaves it out (an
// arm model not in the catalogue, an item that does not travel, a stock
// outside) is not in the list. Needs no model.
std::vector<MassPart> TransportMassParts(const Layout& layout);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_ROWS_H_
