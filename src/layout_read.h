// The readers that src/layout.cc and src/layout_read.cc share: the checks of
// a field's type (Fields, Number, Integer, Bool, String, Array), the paths of
// fields ("arms[1].track.beam"), the error they throw, and the readers of a
// layout's parts (a pallet, an arm, a port, ...). They are behind
// LoadLayout of src/layout.h and are not for use elsewhere: a caller loads a
// layout with LoadLayout.
//
// Every reader takes the JSON value and its path in the file and throws a
// LayoutError("<path>", "<what is wrong>") on a fault; LoadLayout turns it
// into "<file>: <path>: <what is wrong>". The readers are in two files only so
// that neither grows past the size the architecture's rules allow
// (tools/architecture/deps.json, "size"): src/layout.cc reads the top level
// (ReadLayout, where a new top-level field is read), src/layout_read.cc the
// parts (where a new field of a part of src/layout_parts.h is read).

#ifndef SRC_LAYOUT_READ_H_
#define SRC_LAYOUT_READ_H_

#include <cstddef>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "src/layout.h"

namespace cell::layout {

// A fault of a layout file at the field `field`.
struct LayoutError : std::runtime_error {
  LayoutError(const std::string& field, const std::string& what)
      : std::runtime_error(field + ": " + what) {}
};

// The name of a JSON value's type, for the messages ("array", "null").
std::string TypeName(const nlohmann::json& j);

// Checks that `j` is an object with exactly the fields `keys`, and may have
// those of `optional`, in any order.
void Fields(const nlohmann::json& j, const std::string& path,
            std::initializer_list<const char*> keys,
            std::initializer_list<const char*> optional = {});

// The path of the member `key` of the object at `path`, and of element `i`.
std::string Sub(const std::string& path, const char* key);
std::string At(const std::string& path, size_t i);

// The value at `path`, of the type of the name; any other type throws.
double Number(const nlohmann::json& j, const std::string& path);
int Integer(const nlohmann::json& j, const std::string& path);
bool Bool(const nlohmann::json& j, const std::string& path);
std::string String(const nlohmann::json& j, const std::string& path);
const nlohmann::json& Array(const nlohmann::json& j, const std::string& path);

// Refuses two items of `items` with one name.
template <typename T>
void Unique(const std::vector<T>& items, const std::string& path,
            const char* what) {
  std::set<std::string> seen;
  for (size_t i = 0; i < items.size(); ++i) {
    if (!seen.insert(items[i].name).second) {
      throw LayoutError(At(path, i) + ".name",
                        std::string(what) + " \"" + items[i].name +
                            "\" is used twice");
    }
  }
}

// The readers of the parts of src/layout_parts.h, each of one object.
ContainerState ReadContainer(const nlohmann::json& j, const std::string& path);
FlatRackState ReadFlatRack(const nlohmann::json& j, const std::string& path);
Margins ReadMargins(const nlohmann::json& j, const std::string& path);
Port ReadPort(const nlohmann::json& j, const std::string& path);
Approach ReadApproach(const nlohmann::json& j, const std::string& path);
TubePlace ReadTube(const nlohmann::json& j, const std::string& path);
Holder ReadHolder(const nlohmann::json& j, const std::string& path);
ArmUse ReadArm(const nlohmann::json& j, const std::string& path);
Pallet ReadPallet(const nlohmann::json& j, const std::string& path);
Stock ReadStock(const nlohmann::json& j, const std::string& path);
Way ReadWay(const nlohmann::json& j, const std::string& path);
Work ReadWork(const nlohmann::json& j, const std::string& path);
Item ReadItem(const nlohmann::json& j, const std::string& path);

// Reads a "points" string: "all", or numbers and ranges such as
// "0-11,14,20-23", both ends included, each number in 0 .. count - 1.
// Spaces around the numbers are allowed.
std::vector<int> ParsePoints(const std::string& text, int count,
                             const std::string& path);

// The text of a hands_over entry names something of the layout: a pallet
// place by index, a stock, an item, or one of the two ways.
void CheckHandsOver(const Layout& l, const std::string& h,
                    const std::string& path);

}  // namespace cell::layout

#endif  // SRC_LAYOUT_READ_H_
