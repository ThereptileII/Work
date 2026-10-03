#pragma once
#include <initializer_list>
#include <wx/fontenum.h>
#include <wx/string.h>

namespace opennav::integration {
// Explicit chart-label/chart-symbol-label CSS uses Segoe UI, independently of
// the UI's Segoe UI Variable Display stack. Empty means retain the template.
template <typename Available>
wxString SelectPrototypeChartTextFace(Available available) {
  for (const char* face : {"Segoe UI", "Arial"})
    if (available(face)) return wxString(face);
  return {};
}

inline wxString PrototypeChartTextFace() {
  // Installed faces are queried once per module, never while drawing a label.
  static const wxString face = SelectPrototypeChartTextFace(
      [](const char* name) { return wxFontEnumerator::IsValidFacename(name); });
  return face;
}
}  // namespace opennav::integration
