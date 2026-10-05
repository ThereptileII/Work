#pragma once
#include "application/Brand.h"
#include <wx/string.h>

namespace opennav::ui {
// Existing automation and saved layout selectors remain stable. Native window
// titles and accessible labels use the public identity instead.
inline wxString BrandedSurfaceTitle(const wxString &selector) {
  return selector.StartsWith("OpenNav ")
      ? wxString(application::brand::Name) + " " + selector.Mid(8)
      : selector;
}
} // namespace opennav::ui
