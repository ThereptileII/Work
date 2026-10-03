#include "ui/FloatingSurface.h"
#include <wx/graphics.h>
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

void XNavFloatingSurface::Shape() {
  const auto size = GetClientSize();
  auto *renderer = wxGraphicsRenderer::GetDefaultRenderer();
  if (!renderer || size.x <= 0 || size.y <= 0) return;
  auto path = renderer->CreatePath();
  path.AddRoundedRectangle(0, 0, size.x, size.y, FromDIP(radius_));
  SetShape(path);
}
} // namespace opennav::ui
