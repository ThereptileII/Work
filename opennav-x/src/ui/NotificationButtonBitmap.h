#pragma once

#include "ui/PrototypeIcons.h"
#include <wx/bmpbndl.h>

namespace opennav::ui {
// Match the shell's 44 DIP target in the upstream physical-pixel rectangle.
// FromDIP handles Windows DPI; ToPhys also covers DIP-based Retina/GTK ports.
inline wxSize NotificationButtonSize(const wxWindow &window) {
  const auto logical = window.FromDIP(wxSize(44, 44));
#ifdef wxHAS_DPI_INDEPENDENT_PIXELS
#if wxCHECK_VERSION(3, 1, 6)
  return window.ToPhys(logical);
#else
  const auto scale = window.GetContentScaleFactor();
  return wxSize(wxRound(logical.x * scale), wxRound(logical.y * scale));
#endif
#else
  return logical;
#endif
}

// Presentation only. The pinned NotificationButton owns placement, visibility,
// maximum severity, invalidation and clicks. Unknown upstream artwork falls back.
inline wxBitmap NotificationButtonBitmap(const wxString &icon, LightMode mode,
                                          const wxSize &size) {
  if (size.x <= 0 || size.y <= 0) return {};
  const auto colors = Theme(mode);
  std::uint32_t ink;
  if (icon == "notification-info-2") ink = NavigationContextInk(mode);
  else if (icon == "notification-warning-2") ink = colors.attention;
  else if (icon == "notification-critical-2") ink = colors.alarm;
  else return {};

  // .alert-button/.icon-btn: 44px target, 9px radius, centered 22px bell.
  // Rasterize this design box at the canvas DPI, retaining the full hit target.
  // Its --bg backing keeps all three severity inks legible over real charts.
  // No illustrative prototype badge/count is manufactured here.
  const auto svg = wxString::Format(
      "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"44\" height=\"44\" viewBox=\"0 0 44 44\">"
      "<rect width=\"44\" height=\"44\" rx=\"9\" fill=\"#%06x\"/>"
      "<g transform=\"translate(11 11) scale(.916666667)\">"
      "<path d=\"%s\" fill=\"none\" stroke=\"#%06x\" stroke-width=\"1.65\" "
      "stroke-linecap=\"round\" stroke-linejoin=\"round\"/></g></svg>",
      static_cast<unsigned int>(colors.background),
      wxString::FromUTF8(PrototypeIconPath(XNavIcon::Bell)),
      static_cast<unsigned int>(ink));
  const auto bundle = wxBitmapBundle::FromSVG(svg.utf8_str(), size);
  if (!bundle.IsOk()) return {};
  return bundle.GetBitmap(size);
}
} // namespace opennav::ui
