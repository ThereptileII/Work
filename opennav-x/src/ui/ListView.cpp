#include "ui/ListView.h"
#include "ui/PrototypeIcons.h"
#include <algorithm>
#include <wx/bmpbndl.h>
#include <wx/dcbuffer.h>

namespace opennav::ui {
XNavListView::XNavListView(wxWindow *parent)
    : wxControl(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                wxBORDER_NONE) {
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  SetName("Vessel traffic list");
  Bind(wxEVT_PAINT, &XNavListView::Paint, this);
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    ScrollPixels(0);
    e.Skip();
  });
  Bind(wxEVT_MOUSEWHEEL, [this](wxMouseEvent &e) {
    if (e.GetWheelDelta())
      ScrollPixels(-e.GetWheelRotation() * FromDIP(71) / e.GetWheelDelta());
  });
  EnableTouchEvents(wxTOUCH_VERTICAL_PAN_GESTURE);
  Bind(wxEVT_GESTURE_PAN, [this](wxPanGestureEvent &e) {
    pressed_.clear();
    ScrollPixels(-e.GetDelta().y);
  });
  Bind(wxEVT_MOTION, [this](wxMouseEvent &e) {
    const auto row = RowAt(e.GetPosition());
    if (row != hover_) {
      hover_ = row;
      Refresh(false);
    }
  });
  Bind(wxEVT_LEAVE_WINDOW, [this](wxMouseEvent &) {
    hover_ = -1;
    Refresh(false);
  });
  Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent &e) {
    const int row = RowAt(e.GetPosition());
    pressed_ = row >= 0 ? rows_[row].identity : "";
    SetFocus();
    if (!pressed_.empty())
      CaptureMouse();
  });
  Bind(wxEVT_LEFT_UP, [this](wxMouseEvent &e) {
    const auto pressed = pressed_;
    pressed_.clear();
    if (HasCapture())
      ReleaseMouse();
    const int row = RowAt(e.GetPosition());
    if (row >= 0 && !pressed.empty() && rows_[row].identity == pressed &&
        on_select) {
      const auto action = on_select;
      GetParent()->CallAfter([action, pressed] { action(pressed); });
    }
  });
  Bind(wxEVT_MOUSE_CAPTURE_LOST,
       [this](wxMouseCaptureLostEvent &) { pressed_.clear(); });
  Bind(wxEVT_KEY_DOWN, [this](wxKeyEvent &e) {
    if (e.GetKeyCode() == WXK_DOWN)
      ScrollPixels(FromDIP(71));
    else if (e.GetKeyCode() == WXK_UP)
      ScrollPixels(-FromDIP(71));
    else
      e.Skip();
  });
}
void XNavListView::Update(std::vector<XNavListRowData> rows, LightMode mode) {
  if (rows.size() > 2000)
    rows.clear();
  bool changed = light_ != mode || rows.size() != rows_.size();
  if (!changed)
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const auto &a = rows[i], &b = rows_[i];
      if (a.identity != b.identity || a.title != b.title ||
          a.subtitle != b.subtitle || a.metric != b.metric ||
          a.detail != b.detail || a.attention != b.attention ||
          a.stale != b.stale) {
        changed = true;
        break;
      }
    }
  if (!changed)
    return;
  rows_ = std::move(rows);
  light_ = mode;
  ScrollPixels(0);
}
void XNavListView::ScrollPixels(int delta) {
  const int maximum =
      (std::max)(0, int(rows_.size()) * FromDIP(71) - GetClientSize().y);
  offset_ = static_cast<int>(std::clamp<long long>(
      static_cast<long long>(offset_) + delta, 0, maximum));
  Refresh(false);
}
int XNavListView::RowAt(wxPoint position) const {
  if (!GetClientRect().Contains(position))
    return -1;
  const int row = (position.y + offset_) / FromDIP(71);
  return row >= 0 && row < int(rows_.size()) ? row : -1;
}
void XNavListView::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  XNavPainter p(*this, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  const int width = ToDIP(GetClientSize().x), row_height = FromDIP(71);
  const auto color = Colour(p.c.accent);
  const auto svg = wxString::Format(
      "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 24 24\"><path "
      "d=\"%s\" fill=\"none\" stroke=\"#%02x%02x%02x\" stroke-width=\"1.65\" "
      "stroke-linecap=\"round\" stroke-linejoin=\"round\"/></svg>",
      wxString::FromUTF8(PrototypeIconPath(XNavIcon::Traffic)), color.Red(),
      color.Green(), color.Blue());
  const auto icon =
      wxBitmapBundle::FromSVG(svg.utf8_str(), FromDIP(wxSize(22, 22)))
          .GetBitmap(FromDIP(wxSize(22, 22)));
  for (int i = offset_ / row_height; i < int(rows_.size()); ++i) {
    const int py = i * row_height - offset_;
    if (py >= GetClientSize().y)
      break;
    const int y = ToDIP(py);
    const auto &r = rows_[i];
    if (i == hover_ || r.attention) {
      dc.SetBrush(wxBrush(Colour(p.c.surface)));
      dc.SetPen(wxPen(Colour(p.c.border)));
      dc.DrawRoundedRectangle(0, py + FromDIP(1), GetClientSize().x - 1,
                              row_height - FromDIP(3), FromDIP(10));
    } else
      p.Rule(0, y + 70, width);
    if (icon.IsOk())
      dc.DrawBitmap(icon, FromDIP(14), py + FromDIP(24), true);
    p.Text(r.title, 49, y + 16, 12, r.stale ? p.c.muted : p.c.primary, true,
           width - 160);
    p.Text(r.subtitle, 49, y + 39, 9, p.c.muted, false, width - 140);
    p.Text(r.metric, width - 86, y + 16, 10,
           r.attention ? p.c.attention : p.c.primary, false, 80);
    p.Text(r.detail, width - 86, y + 36, 9, p.c.muted, false, 80);
  }
  if (rows_.empty())
    p.Text("No current AIS targets", 0, 20, 14, p.c.secondary, false, width);
}
} // namespace opennav::ui
