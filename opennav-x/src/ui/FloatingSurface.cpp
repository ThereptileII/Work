#include "ui/FloatingSurface.h"
#include <wx/graphics.h>
#include <wx/dcbuffer.h>
#include <memory>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif

namespace opennav::ui {
#ifdef __WXGTK__
bool XNavFloatingSurface::Show(bool show) {
  show_requested_ = show;
  return wxFrame::Show(show);
}
#endif

void XNavFloatingSurface::Present(const wxPoint &screen) {
#ifdef __WXGTK__
  // A queued GTK map event can set wx's shown flag after an explicit Hide(),
  // although the native widget remains hidden. Reconcile that stale flag only
  // after a hide request: a normal deferred first show must stay pending.
  if (!show_requested_ && IsShown() && !gtk_widget_get_visible(GetHandle())) {
    wxFrame::Show(false);
    // GTK restores the old native origin on re-show; move to the fresh chart
    // target below only after this nonactivating remap.
    ShowWithoutActivating();
    Shape();
  }
#endif
  if (GetPosition() != screen) Move(screen);
  if (!IsShown()) {
    ShowWithoutActivating();
    Shape();
  }
  RestackNativeAboveOwner();
}

void XNavFloatingSurface::RestackAboveOwner() {
#ifdef __WXGTK__
  if (!IsShownOnScreen() || !gtk_widget_get_visible(GetHandle()) ||
      !gtk_widget_get_mapped(GetHandle())) return;
  RestackNativeAboveOwner();
#endif
}

void XNavFloatingSurface::RestackNativeAboveOwner() {
#ifdef __WXGTK__
  // Restack immediately above the owner without gtk_window_present(), focus
  // changes or a global always-on-top hint. X11 without a window manager must
  // also preserve this relationship. Other applications remain above the pair.
  auto *surface = gtk_widget_get_window(GetHandle());
  auto *owner = gtk_widget_get_window(GetParent()->GetHandle());
  if (surface && owner) gdk_window_restack(surface, owner, true);
#endif
}

void XNavFloatingSurface::SetChartToolsTheme(LightMode mode) {
  if (!chart_tools_paint_) {
    chart_tools_paint_ = true;
    radius_ = 11; // .map-tools, not the generic floating-control radius.
    SetBackgroundStyle(wxBG_STYLE_PAINT);
    Bind(wxEVT_PAINT, &XNavFloatingSurface::PaintChartTools, this);
    Shape();
  }
  const auto rgb = FloatingTheme(mode).surface;
  SetBackgroundColour(wxColour((rgb>>16)&255, (rgb>>8)&255, rgb&255));
  Refresh(false);
}

void XNavFloatingSurface::PaintChartTools(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  const auto surface = GetBackgroundColour();
  dc.SetBackground(wxBrush(surface));
  dc.Clear();
  const auto composite = [&](unsigned rgb, unsigned alpha) {
    const auto channel = [alpha](unsigned front, unsigned back) {
      return static_cast<unsigned char>((front * alpha + back * (255-alpha) + 127) / 255);
    };
    return wxColour(channel((rgb>>16)&255, surface.Red()),
                    channel((rgb>>8)&255, surface.Green()),
                    channel(rgb&255, surface.Blue()));
  };
  // Exact CSS #6b8b801c floating border and #68837730 map-tools separator.
  // Paint the full exposed client area so native theme chrome cannot supply
  // the pale GTK frame seen around Night controls.
  const auto size = GetClientSize();
  const double scale = GetDPIScaleFactor();
  std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
  if (gc) {
    gc->DisableOffset(); // Coordinates already place the 1px stroke on pixel centers.
    gc->SetPen(gc->CreatePen(wxGraphicsPenInfo(composite(0x6b8b80, 0x1c)).Width(scale)));
    gc->SetBrush(*wxTRANSPARENT_BRUSH);
    gc->DrawRoundedRectangle(.5*scale, .5*scale, size.x-scale, size.y-scale, 10.5*scale);
  }
  gc.reset(); // Finish graphics rendering before returning to the native DC.
  dc.SetPen(*wxTRANSPARENT_PEN);
  dc.SetBrush(wxBrush(composite(0x688377, 0x30)));
  // The existing 5px spacer is 2px margin + 1px rule + 2px margin.
  dc.DrawRectangle(FromDIP(4+2*44+2), (size.y-FromDIP(18))/2, FromDIP(1), FromDIP(18));
}

void XNavFloatingSurface::Shape() {
  const auto size = GetClientSize();
  auto *renderer = wxGraphicsRenderer::GetDefaultRenderer();
  if (!renderer || size.x <= 0 || size.y <= 0) return;
  auto path = renderer->CreatePath();
  path.AddRoundedRectangle(0, 0, size.x, size.y, FromDIP(radius_));
  SetShape(path);
}
} // namespace opennav::ui
