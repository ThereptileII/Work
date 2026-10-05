bool DrawChartOverzoomWarning(ocpnDC &dc, ChartCanvas &canvas, int x, int y) {
  if (!wxIsMainThread() || !xnav_mode || !active || x < 0 || y < 0) return false;
  const auto mode = canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_NIGHT
      ? ui::LightMode::Night : canvas.GetColorScheme() == GLOBAL_COLOR_SCHEME_DUSK
      ? ui::LightMode::Dusk : ui::LightMode::Day;
  const auto theme = ui::Theme(mode);
  const wxFont font = ui::UiFontWeight(canvas, 12, 550);
  if (!font.IsOk()) return false;
  // Necessary one-line warning extension: the HTML has no OverZoom component.
  // Preserve its translated source wording, using final warning-callout roles.
  const wxString label = _("OverZoom");
  const auto old_font = dc.GetFont(); const auto old_ink = dc.GetTextForeground();
  const auto old_pen = dc.GetPen(); const auto old_brush = dc.GetBrush();
  struct Restore {
    ocpnDC &dc; wxFont font; wxColour ink; wxPen pen; wxBrush brush;
    ~Restore() { dc.SetFont(font); dc.SetTextForeground(ink);
                 dc.SetPen(pen); dc.SetBrush(brush); }
  } restore{dc, old_font, old_ink, old_pen, old_brush};
  // ocpnDC clamps measured widths to 500; use the full native extent for fit.
  wxClientDC measure(&canvas); measure.SetFont(font);
  int width = 0, height = 0; measure.GetTextExtent(label, &width, &height);
  const int padx = canvas.FromDIP(15), pady = canvas.FromDIP(13);
  const auto size = canvas.GetClientSize();
  // Measure the entire translation. Never elide or clip a warning to make it fit.
  if (width <= 0 || height <= 0 || x >= size.x || y >= size.y ||
      width > size.x - x - 2 * padx || height > size.y - y - 2 * pady)
    return false;
  const wxRect bounds(x, y, width + 2 * padx, height + 2 * pady);
  if (auto *native = dc.GetDC()) {
    wxCoord cx, cy, cw, ch;
    if (native->GetClippingBox(&cx, &cy, &cw, &ch) &&
        !wxRect(cx, cy, cw, ch).Contains(bounds)) return false;
  }
  const auto mix = [&](int alpha) {
    const auto ink = ui::Colour(ui::prototype_ink::warning);
    const auto base = ui::Colour(theme.background);
    return wxColour((ink.Red()*alpha + base.Red()*(255-alpha) + 127)/255,
                    (ink.Green()*alpha + base.Green()*(255-alpha) + 127)/255,
                    (ink.Blue()*alpha + base.Blue()*(255-alpha) + 127)/255);
  };
  const auto edge = mix(ui::prototype_ink::warning_border_alpha);
  dc.SetPen(wxPen(edge, canvas.FromDIP(1)));
  dc.SetBrush(wxBrush(mix(ui::prototype_ink::warning_callout_alpha)));
  dc.DrawRoundedRectangle(x, y, bounds.width - 1, bounds.height - 1,
                          canvas.FromDIP(8));
  dc.SetPen(wxPen(edge, canvas.FromDIP(2)));
  dc.DrawLine(x + canvas.FromDIP(1), y, x + canvas.FromDIP(1), y + bounds.height - 1);
  dc.SetFont(font);
  dc.SetTextForeground(ui::Colour(theme.attention));
  dc.DrawText(label, x + padx, y + pady);
  return true;
}
