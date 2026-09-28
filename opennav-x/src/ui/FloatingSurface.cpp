#include "ui/FloatingSurface.h"
#include <wx/graphics.h>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif

namespace opennav::ui {
void XNavFloatingSurface::Present(const wxPoint &screen) {
  if (GetPosition() != screen) Move(screen);
  if (!IsShown()) {
    ShowWithoutActivating();
    Shape();
  }
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
