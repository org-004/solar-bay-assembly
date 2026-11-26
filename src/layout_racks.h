// The reader of what a layout says about two flat racks and about the stages
// of a tube worked in sections: the block "two_flat_racks", the field "rack"
// of arms, the holder, stocks and items, and the block "staging"
// (src/layout.h gives the form). src/layout.cc reads everything else and
// calls ReadRacksAndStages once the rest of the layout is read; the two are
// in separate files only so that neither grows past the size the
// architecture's rules allow.
//
// What it refuses, each with the field's path: a "two_flat_racks" block whose
// gap is not within its own tolerance (0 <= gap_lo_m <= gap_m <= gap_hi_m)
// or whose misalignment is negative; on base "two_flat_racks", an arm, the
// holder, a stock that is not outside or an item that travels without a
// rack "a" or "b", and a rack on a pallet-side thing that stands outside (a
// stock outside, an item that does not travel); on any other base, any
// "rack" at all; a staging with no stage, two stages of one name, a rail or a
// module in no stage or in two, a module set at a stage before one of its
// rails, a grip of a stretch that does not exist, and an angle_pin_by that
// is not "holder" or an item of the layout.

#ifndef SRC_LAYOUT_RACKS_H_
#define SRC_LAYOUT_RACKS_H_

#include <nlohmann/json.hpp>

#include "src/layout.h"

namespace cell::layout {

// Reads the top-level object `top` of a layout file into *layout, whose other
// fields are read already. Throws std::runtime_error("<field>: <what>").
void ReadRacksAndStages(const nlohmann::json& top, Layout* layout);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_RACKS_H_
