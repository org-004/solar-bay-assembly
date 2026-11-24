// The checks on a layout. See src/layout_check.h.

#include "src/layout_check.h"

namespace cell::layout {

CheckReport CheckLayout(const Layout&, const ModelPaths&,
                        const CheckOptions&) {
  return {};
}

bool StateAt(const Layout&, const ModelPaths&, const std::string&, int,
             LayoutState*, std::string* error) {
  if (error != nullptr) *error = "not written yet";
  return false;
}

}  // namespace cell::layout
