#include "integration/ChartNameText.h"
#if defined(__WXMSW__) && wxUSE_GRAPHICS_GDIPLUS
// wxWidgets' SDK wrapper keeps Windows/GDI+ macros out of shared S-52 headers.
#include <wx/msw/wrapgdip.h>
#endif

namespace opennav::integration {
bool PrepareChartNameAlpha(wxGraphicsContext& context) {
#if defined(__WXMSW__) && wxUSE_GRAPHICS_GDIPLUS
  if (context.GetRenderer() == wxGraphicsRenderer::GetGDIPlusRenderer()) {
    auto* graphics = static_cast<Gdiplus::Graphics*>(context.GetNativeContext());
    // System/ClearType smoothing can exceed the requested per-channel alpha
    // on dark backgrounds. Grayscale coverage preserves this brush's opacity.
    // This context belongs to one translucent run; opaque DC text is untouched.
    return graphics && graphics->SetTextRenderingHint(
        Gdiplus::TextRenderingHintAntiAliasGridFit) == Gdiplus::Ok;
  }
#endif
  return true;
}
} // namespace opennav::integration
