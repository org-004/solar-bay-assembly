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
//   port       a separate unit stands against the cell only at a port: every
//              pallet, every stock and item that does not travel, and the
//              tube and the bay at the outer end of their ways, lie wholly
//              inside the zone of one port that names them; no zone reaches
//              over the base's footprint; the zones of two ports are apart
//              unless each says it shares the other's; each port's frame is
//              on the base's outline; and in the stowed state nothing of the
//              cell is in a zone.
//   transport  for a layout on the flat rack (src/flat_rack.h): stowed, every
//              geom of the cell inside the transport envelope across, along
//              and at the top, and above the deck; everything that travels
//              and does not move standing on the deck, itself or through what
//              it touches; each arm's stow track value inside its track; the
//              mass within what the flat rack leaves for the cell. Then, as
//              rows without a limit that always pass, how far the stowed cell
//              is inside (or, negative, outside) what a container that closes
//              would give it: the closed container's inside envelope standing
//              on the deck, and the length between a flat rack's end frames;
//              the row's detail names what stands outside.
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
