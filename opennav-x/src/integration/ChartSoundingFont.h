#pragma once

#include <cmath>
#include <wx/font.h>
#include <wx/fontenum.h>

namespace opennav::integration {
// The final .chart-depth override is 10 CSS px, inheriting this body stack.
// Preserve sounding ink/opacity and special symbols in their S-52 paths.
inline wxFont ChartSoundingFont(double sounding_scale, double content_scale) {
  static const wxString face = [] {
    for (const auto *candidate : {"Segoe UI Variable Display", "Segoe UI", "Arial"})
      if (wxFontEnumerator::IsValidFacename(candidate)) return wxString(candidate);
    return wxString("Arial");
  }();
  if (!std::isfinite(sounding_scale) || sounding_scale < 0.5 || sounding_scale > 2.0)
    sounding_scale = 1.0;
  if (!std::isfinite(content_scale) || content_scale <= 0.0)
    content_scale = 1.0;
  wxFont font(wxFontInfo(8).Family(wxFONTFAMILY_SWISS).FaceName(face));
  // 10 logical px = 7.5 pt at 96 DPI. wx retains native display-DPI handling;
  // include upstream's separate content scale once, as FontMgr normally does.
  font.SetFractionalPointSize(7.5 * sounding_scale * content_scale);
  return font;
}
}  // namespace opennav::integration
