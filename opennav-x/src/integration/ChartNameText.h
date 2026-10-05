#pragma once
#include "integration/ChartNameSpacing.h"
#include <wx/dc.h>
#include <wx/graphics.h>
#include <memory>

namespace opennav::integration {

#ifdef __WXMSW__
// Changes only this fresh translucent context when its backend is GDI+.
bool PrepareChartNameAlpha(wxGraphicsContext& context);
#endif

struct ChartNameTextRun {
  wxString text;
  wxArrayInt advances;
  std::vector<std::size_t> starts;
  double tracking = 0;
  int extra_width = 0;

  ChartNameTextRun(wxDC& dc, const wxString& value, double spacing) : text(value) {
    if (!std::isfinite(spacing) || spacing <= 0 || spacing > 64 || text.size() > 512)
      return;
    std::u32string points;
    for (const auto c : text) points.push_back(c.GetValue());
    starts = ChartNameClusterStarts(points);
    const double extra = ChartNameTrackingWidth(starts.size(), spacing);
    if (extra <= 0 || !dc.GetPartialTextExtents(text, advances) ||
        advances.size() != points.size()) {
      starts.clear();
      return;
    }
    tracking = spacing;
    extra_width = static_cast<int>(std::lround(extra));
  }

  template <typename Draw>
  void Paint(Draw draw, double x, double y) const {
    if (starts.empty()) { draw(text, x, y); return; }
    for (std::size_t i = 0; i < starts.size(); ++i) {
      const auto start = starts[i];
      const auto end = i + 1 < starts.size() ? starts[i + 1] : text.size();
      draw(text.Mid(start, end - start),
           x + (start ? advances[start - 1] : 0) + tracking * i, y);
    }
  }
  void DrawOpaque(wxDC& dc, int x, int y) const {
    Paint([&dc](const wxString& s, double a, double b) {
      dc.DrawText(s, static_cast<int>(std::lround(a)), static_cast<int>(std::lround(b)));
    }, x, y);
  }
  bool Draw(wxDC& dc, int x, int y, wxColour color, unsigned char opacity) const {
    if (opacity == 255) { DrawOpaque(dc, x, y); return true; }
    std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::CreateFromUnknownDC(dc));
    if (!gc) return false;
#ifdef __WXMSW__
    if (!PrepareChartNameAlpha(*gc)) return false;
#endif
    color.Set(color.Red(), color.Green(), color.Blue(), opacity);
    gc->SetFont(dc.GetFont(), color);
    Paint([&gc](const wxString& s, double a, double b) { gc->DrawText(s, a, b); }, x, y);
    return true;
  }
};

} // namespace opennav::integration
