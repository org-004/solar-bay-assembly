// The text of models/bay.xml, computed from the constants of
// src/bay_hardware.h, and the shapes it is made of, one part at a time.
//
// models/bay.xml is one bay of a single-axis tracker as a MuJoCo model: a
// round tube, the rails with their straps, the modules, and a site at every
// fastening, locator and hold. It was once written out by a script that is
// lost. BayModelXml() is that script again: every position, size and mass in
// the text follows from the header's constants, so that a change of the
// header (the number of modules, say) is a change of the model by running
// //:gen_bay and nothing else. tests/bay_model_test.cc fails when the
// committed file differs from this text by a single byte.
//
//   bazel run --config=opt //:gen_bay > models/bay.xml
//
// What the header does not hold is named in src/bay_model.cc: the colours,
// the sizes of the sites, the names of the classes, and the number of
// decimals in each kind of figure. Positions and sizes are written with the
// fewest digits that carry twelve significant ones, masses with six decimals
// (the tube's with three), as the file always had them.
//
// The whole bay is the default. A later piece builds the bay as it is at a
// step of its assembly, so the options say how much of it is on:
//   - rails: the first `rails` rails, each with its strap, are on the tube;
//   - modules: the first `modules` modules lie on them.
// The tube is always whole. Its sites stay when a part is absent, since the
// holes and the stretches are there before anything is put on: the _b sites
// of the rails' fastenings, the _b sites of the locators and the holds. A
// rail's own sites stay with the rail, including the _a sites of the pins
// for every module of the bay, on or off, since those are the points a module
// is placed against. A module's sites, the _b sites of its pins, go with the
// module. A rail that is not on has no sites at all.
//
// The same shapes make parts that stand elsewhere: RailBodyXml and
// ModuleBodyXml each return one body, at the position given, named as given
// (every geom's name starts with the body's name). A part made for another
// place, such as a module on a pallet or a rail in a stock, passes no index
// and has no sites, since the sites are named for a place in the bay.
//
// Body positions are in the frame of the body they are written in. For the
// bay that is the tube's frame, "bay_frame" in the file; here the frame is
// placed in the world at `frame_pos`, by default so that the tube lies in the
// plane of the container's -X side (half the container's outside width from
// the origin), the section is centred along Y on the origin, and the tube's
// axis is 1.2 m above the floor. That is the placing the file has always had.
//
// The comment that opens the file describes the finished bay, whatever the
// options say; `comment` = false leaves it out, for a model that is only a
// part of a larger one.

#ifndef SRC_BAY_MODEL_H_
#define SRC_BAY_MODEL_H_

#include <array>
#include <string>

#include "src/bay_hardware.h"

namespace cell::bay {

// Where the tube's frame stands in the world by default: the tube's axis in
// the plane of the container's -X side (half of its outside width, 2.438 m),
// 1.2 m above the floor.
inline constexpr double kDefaultFrameX = -1.219;
inline constexpr double kDefaultFrameHeight = 1.200;

struct BayModelOptions {
  // The origin of the tube's frame in the world. The section is centred along
  // Y on the world's origin.
  std::array<double, 3> frame_pos = {kDefaultFrameX, -kSectionLength / 2.0,
                                     kDefaultFrameHeight};
  // How many rails are on the tube, 0 to kRailCount; the first ones.
  int rails = kRailCount;
  // How many modules lie on them, 0 to kModuleCount; the first ones.
  int modules = kModuleCount;
  // Whether the long comment that opens the file is written.
  bool comment = true;
};

// The whole file. Throws std::invalid_argument if rails or modules is out of
// its range.
std::string BayModelXml(const BayModelOptions& options = {});

// The pieces. Each returns whole lines, indented as the bodies of bay_frame
// are, and none of them ends with a blank line.

// The body "bay_tube" with its two geoms and every site on the tube.
std::string TubeBodyXml();

// One rail with its strap, as a body of the given name at `pos`. With `index`
// 0 .. kRailCount - 1 it has the sites of that rail of the bay (the strap
// bolt's _a, the locator's _a, the _a of the pins that go through it);
// without, none.
//
// The strap is closed by default: a square U under the tube (_strap_neg,
// _strap_pos, _strap_bottom, _strap_ears), as on a rail that is on the tube.
// With `strap_open` it hangs open, as on a rail that is carried to the tube:
// its two ends are rolled round pins in the webs' notches and swing open to
// pass the tube (docs/bay-hardware.md, Rail, "Strap"). Each half is then one
// box swung outward about its pin until it lies level under the rail,
// _strap_open_neg and _strap_open_pos: from |x| = kTubeRadius outward by half
// the strap's developed length (0.174 m), kStrapWidth along the tube, 0.020 m
// thick, its top at the webs' lower edges. Nothing of the open strap is
// between the webs under the tube or below that slab. The rail's mass is the
// same either way. src/layout_model.cc draws a rail an arm holds open and the
// rails on the tube closed; models/bay.xml has only closed straps.
std::string RailBodyXml(const std::string& name,
                        const std::array<double, 3>& pos, int index = -1,
                        bool strap_open = false);

// One module, as a body of the given name at `pos`: nine geoms, and with
// `index` 0 .. kModuleCount - 1 the _b site of each of that module's pins.
std::string ModuleBodyXml(const std::string& name,
                          const std::array<double, 3>& pos, int index = -1);

}  // namespace cell::bay

#endif  // SRC_BAY_MODEL_H_
