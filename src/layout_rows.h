// The rows of a layout's check that need no pose of an arm: what is
// allowed, the ports, and the transport rows of a layout on the flat rack.
//
// These are part of the checker (src/layout_check.h, whose opening comment
// lists every kind of row); CheckLayout calls them and //:check_layout
// prints them with the rest. They are in a file of their own because they
// read only the layout and its model at rest.
//
//   allowed    the limits the owner set on 2026-11-24 for the first cell, one
//              row each: how many arms take or set modules (at most two),
//              that each of them is a KUKA KR 210 R2700-2 or a smaller KUKA
//              arm of src/arm_catalog.h, how many arms locate, pin or swage
//              (at most three), how many arms in all (at most five), and that
//              every pallet is the maker's, 36 modules standing on their long
//              edge.
//   port       a separate unit stands against the cell only at a port. Rows:
//              that tubes and bays are two ports; for each port, that its
//              frame is on the outline of the base's footprint (at the deck's
//              height), that its zone's centre lies on the +x side of the
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
//              the flat rack leaves for the cell. Then, as rows without a
//              limit that always pass, how far the stowed cell is inside
//              (or, negative, outside) what a container that closes would
//              give it: the closed container's inside envelope standing on
//              the deck, and the length between a flat rack's end frames;
//              the row's detail names every geom that stands outside.
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

// The `transport` rows of a layout whose base is "flat_rack", the rows
// without a limit included. Builds the stowed model itself.
std::vector<CheckRow> FlatRackTransportRows(const Layout& layout,
                                            const ModelPaths& paths);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_ROWS_H_
