#include "ui/NextTurnCard.h"
#include <memory>
#include <wx/bmpbndl.h>
#include <wx/dcbuffer.h>
#include <wx/graphics.h>

namespace opennav::ui {
namespace {
wxString W(const std::string &s) { return wxString::FromUTF8(s); }
// Prototype icons: turn ("M5 21V11a4 4 0 0 1 4-4h11m-6-5 6 5-6 5"), mirrored
// for port; a straight arrow and a pin for the remaining cases.
wxString IconSvg(const application::NextTurnView &view, std::uint32_t ink) {
  const char *path = view.direction ? "M5 21V11a4 4 0 0 1 4-4h11m-6-5 6 5-6 5"
                   : view.headline == "Arrival"
                       ? "M12 21s-7-6.2-7-11a7 7 0 0 1 14 0c0 4.8-7 11-7 11Zm0-8.5a2.5 2.5 0 1 0 0-5 2.5 2.5 0 0 0 0 5Z"
                       : "M12 21V4m-6 6 6-6 6 6";
  const wxString transform = view.direction < 0 ? " transform=\"matrix(-1 0 0 1 24 0)\"" : "";
  return wxString::Format(
      "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"24\" height=\"24\" viewBox=\"0 0 24 24\">"
      "<path%s d=\"%s\" fill=\"none\" stroke=\"#%06x\" stroke-width=\"1.9\" "
      "stroke-linecap=\"round\" stroke-linejoin=\"round\"/></svg>",
      transform, path, ink);
}
}  // namespace

XNavNextTurnCard::XNavNextTurnCard(wxWindow *parent, std::function<void()> open)
    : wxPanel(parent, wxID_ANY), open_(std::move(open)) {
  SetName("Next turn");
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  SetCursor(wxCursor(wxCURSOR_HAND));
  Bind(wxEVT_PAINT, &XNavNextTurnCard::Paint, this);
  Bind(wxEVT_LEFT_UP, [this](wxMouseEvent &) { if (open_) open_(); });
}

wxSize XNavNextTurnCard::DoGetBestClientSize() const {
  // .next-turn: min-width 304, padding 17/18, 48 px icon row.
  return FromDIP(wxSize(304, 82));
}

bool XNavNextTurnCard::Update(const application::NextTurnView &view, LightMode mode) {
  const bool same = mode == mode_ && view.visible == view_.visible &&
                    view.eyebrow == view_.eyebrow && view.headline == view_.headline &&
                    view.detail == view_.detail && view.turn_deg == view_.turn_deg &&
                    view.direction == view_.direction;
  view_ = view;
  mode_ = mode;
  if (!same) Refresh(false);
  return !same;
}

void XNavNextTurnCard::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  const auto floating = FloatingTheme(mode_);
  dc.SetBackground(wxBrush(Colour(floating.surface)));
  dc.Clear();
  std::unique_ptr<wxGraphicsContext> gc(wxGraphicsContext::Create(dc));
  if (!gc) return;
  const int pad_x = FromDIP(18);
  const int height = GetClientSize().y;
  // Turn icon tile: route ink, 10 px radius, 44x48.
  const wxSize tile = FromDIP(wxSize(44, 48));
  const int tile_y = (height - tile.y) / 2;
  gc->SetPen(*wxTRANSPARENT_PEN);
  gc->SetBrush(wxBrush(Colour(ActiveRouteInk(mode_))));
  gc->DrawRoundedRectangle(pad_x, tile_y, tile.x, tile.y, FromDIP(10));
  const int icon = FromDIP(24);
  const auto bitmap = wxBitmapBundle::FromSVG(IconSvg(view_, 0xF2F5EE).utf8_str(),
                                              wxSize(icon, icon)).GetBitmap(wxSize(icon, icon));
  if (bitmap.IsOk())
    gc->DrawBitmap(bitmap, pad_x + (tile.x - icon) / 2, tile_y + (tile.y - icon) / 2, icon, icon);
  const int text_x = pad_x + tile.x + FromDIP(14);
  const int chevron_w = FromDIP(18);
  const int text_w = GetClientSize().x - text_x - chevron_w - pad_x;
  // Eyebrow, headline ("Starboard 32°", the angle semibold), detail.
  gc->SetFont(UiFontWeight(*this, 9, 650), Colour(floating.secondary));
  gc->DrawText(wxControl::Ellipsize(W(view_.eyebrow), dc, wxELLIPSIZE_END, text_w),
               text_x, tile_y - FromDIP(2));
  gc->SetFont(UiFont(*this, 20), Colour(floating.primary));
  double hw = 0, hh = 0;
  gc->GetTextExtent(W(view_.headline), &hw, &hh);
  const int head_y = tile_y + FromDIP(11);
  gc->DrawText(W(view_.headline), text_x, head_y);
  if (view_.turn_deg) {
    gc->SetFont(UiFontWeight(*this, 20, 650), Colour(floating.primary));
    gc->DrawText(wxString::Format(wxString::FromUTF8("%d°"), *view_.turn_deg),
                 text_x + hw + FromDIP(6), head_y);
  }
  dc.SetFont(UiFont(*this, 10));
  gc->SetFont(UiFont(*this, 10), Colour(floating.secondary));
  gc->DrawText(wxControl::Ellipsize(W(view_.detail), dc, wxELLIPSIZE_END, text_w),
               text_x, tile_y + tile.y - FromDIP(13));
  gc->SetFont(UiFont(*this, 18), Colour(floating.secondary));
  gc->DrawText(wxString::FromUTF8("›"), GetClientSize().x - pad_x - FromDIP(8),
               (height - FromDIP(22)) / 2);
}
}  // namespace opennav::ui
