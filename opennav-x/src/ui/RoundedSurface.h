#pragma once
#include "ui/Controls.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/nonownedwnd.h>
#include <wx/region.h>

namespace opennav::ui {
// Native shape, not a painted imitation of the chart behind a window. A region
// also avoids wxMSW's SetShape(path) helper, which adds its own light-grey border
// paint handler. The caller keeps ownership of focus, modality and dismissal.
class RoundedWindowShape {
public:
  void Update(wxNonOwnedWindow &window, int radius_dip) {
    const auto size = window.GetClientSize();
    const int radius = (std::max)(0, (std::min)(window.FromDIP(radius_dip),
        (std::min)(size.x, size.y) / 2));
    if (size.x <= 0 || size.y <= 0 || (size == size_ && radius == radius_)) return;
    wxRegion region;
    if (size.y > 2 * radius) region.Union(0, radius, size.x, size.y - 2 * radius);
    // Pixel-centre coverage of the circular corners. Work is O(radius), never
    // O(window area), and the same outline clips both paint and native input.
    for (int y = 0; y < radius; ++y) {
      const double dy = radius - y - .5;
      const int inset = static_cast<int>(std::ceil(radius -
          std::sqrt(radius * radius - dy * dy) - .5));
      region.Union(inset, y, size.x - 2 * inset, 1);
      region.Union(inset, size.y - y - 1, size.x - 2 * inset, 1);
    }
    if (window.SetShape(region)) { size_ = size; radius_ = radius; }
  }
private:
  wxSize size_;
  int radius_ = -1;
};

inline void PaintRoundedSurface(wxWindow &window, wxDC &dc, LightMode mode,
                                int radius_dip) {
  const auto c = Theme(mode);
  dc.SetBackground(wxBrush(Colour(c.background)));
  dc.Clear();
  const auto size = window.GetClientSize();
  const int stroke = (std::max)(1, window.FromDIP(1));
  if (size.x <= stroke || size.y <= stroke) return;
  const int radius = (std::max)(0, (std::min)(window.FromDIP(radius_dip),
      (std::min)(size.x, size.y) / 2));
  const double inset = stroke / 2.;
  std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::CreateFromUnknownDC(dc));
  if (gc) {
    gc->SetBrush(wxBrush(Colour(c.background)));
    gc->SetPen(wxPen(Colour(c.border), stroke));
    gc->DrawRoundedRectangle(inset, inset, size.x - stroke, size.y - stroke,
        (std::max)(0., radius - inset));
  } else {
    dc.SetBrush(wxBrush(Colour(c.background)));
    dc.SetPen(wxPen(Colour(c.border), stroke));
    dc.DrawRoundedRectangle(0, 0, (std::max)(0, size.x - 1),
        (std::max)(0, size.y - 1), radius);
  }
}

inline void BindRoundedSurface(wxNonOwnedWindow &window,
    std::function<LightMode()> light, int radius_dip) {
  auto shape = std::make_shared<RoundedWindowShape>();
  window.SetBackgroundStyle(wxBG_STYLE_PAINT);
  window.Bind(wxEVT_SIZE, [&window, shape, radius_dip](wxSizeEvent &event) {
    shape->Update(window, radius_dip); event.Skip();
  });
  window.Bind(wxEVT_SHOW, [&window, shape, radius_dip](wxShowEvent &event) {
    if (event.IsShown()) shape->Update(window, radius_dip);
    event.Skip();
  });
  window.Bind(wxEVT_PAINT, [&window, light, radius_dip](wxPaintEvent &) {
    wxAutoBufferedPaintDC dc(&window);
    PaintRoundedSurface(window, dc, light(), radius_dip);
  });
  shape->Update(window, radius_dip);
}
} // namespace opennav::ui
