#pragma once

#include <cmath>
#include <wx/font.h>

namespace opennav::integration {
// Input seam for isolated extracted painters, not the production configuration
// or renderer lifecycle. Existing fixtures retain their 100% baseline.
inline double chart_display_fixture_scale = 1.0;
inline double ChartDisplayScale() { return chart_display_fixture_scale; }
inline wxFont ScaleChartFont(wxFont font) {
  if (font.IsOk()) font.SetFractionalPointSize(
      font.GetFractionalPointSize() * chart_display_fixture_scale);
  return font;
}
template<class Canvas>
inline int ChartDisplayPixels(Canvas& canvas, int logical_pixels) {
  return static_cast<int>(std::lround(
      canvas.FromDIP(logical_pixels) * chart_display_fixture_scale));
}
}  // namespace opennav::integration
