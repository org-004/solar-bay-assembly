// See src/layout_rows.h. STUB written with the header; its owner replaces it.

#include "src/layout_rows.h"

namespace cell::layout {

std::vector<CheckRow> AllowedRows(const Layout&) { return {}; }

std::vector<CheckRow> PortRows(const Layout&, const ModelPaths&) { return {}; }

std::vector<CheckRow> FlatRackTransportRows(const Layout&, const ModelPaths&) {
  return {};
}

}  // namespace cell::layout
