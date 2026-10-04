wxFont UiFontWeight(wxWindow& window, int pixels, int weight) {
  // Same ordered stack as the immutable HTML. Use installed fonts, never copy
  // proprietary Windows font files into the package or the Linux environment.
  static const wxString face = [] {
    for (const auto *candidate : {"Segoe UI Variable Display", "Segoe UI", "Arial"})
      if (wxFontEnumerator::IsValidFacename(candidate)) return wxString(candidate);
    return wxString("Arial"); // Same fontconfig fallback request as Chromium.
  }();
#ifdef __WXMSW__
  // wxMSW uses negative LOGFONT character height (the CSS em), scaled once.
  wxFont font(wxFontInfo(wxSize(0, window.FromDIP(pixels))).FaceName(face));
#else
  // wxGTK SetPixelSize fits the *line cell*, making every glyph too small.
  // Pango's point-size constructor expresses the em: 96 CSS px = 72 pt.
  // Pango applies the display resolution itself; do not scale it twice.
  (void)window;
  wxFont font(wxFontInfo(pixels * 72.0 / 96.0).FaceName(face));
#endif
  font.SetNumericWeight(weight);
  return font;
}
