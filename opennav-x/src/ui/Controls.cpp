#include "ui/Controls.h"
#include "ui/PrototypeIcons.h"

#include <wx/dcbuffer.h>
#include <wx/graphics.h>
#include <wx/fontenum.h>
#include <wx/bmpbndl.h>
#include <wx/eventfilter.h>
#include <wx/sizer.h>
#include <wx/tokenzr.h>
#ifdef __WXMSW__
#include <wx/msw/wrapwin.h>
#include <dwrite.h>
#endif

#include <memory>
#include <cmath>

namespace opennav::ui {
namespace {
// Track input modality from events. wxGTK only supports modifier keys in
// wxGetKeyState; querying Tab asserts there. This observer never consumes input.
class FocusInput final : public wxEventFilter {
 public:
  FocusInput() { wxEvtHandler::AddFilter(this); }
  ~FocusInput() override { wxEvtHandler::RemoveFilter(this); }
  int FilterEvent(wxEvent &event) override {
    const auto type = event.GetEventType();
    if (type == wxEVT_KEY_DOWN || type == wxEVT_CHAR_HOOK)
      keyboard = true;
    else if (type == wxEVT_LEFT_DOWN || type == wxEVT_RIGHT_DOWN ||
             type == wxEVT_GESTURE_PAN)
      keyboard = false;
    return Event_Skip;
  }
  bool keyboard = false;
};
FocusInput &FocusModality() { static FocusInput input; return input; }
// Native hover windows use the OS palette, not our painted marine palette.
// Change only the owning XNav control: Legacy tooltips keep their normal state.
void ApplyHint(wxWindow &window, LightMode mode, const wxString &hint) {
  if (mode == LightMode::Day && !hint.empty()) {
    if (window.GetToolTipText() != hint) window.SetToolTip(hint);
  } else if (window.GetToolTip()) {
    window.UnsetToolTip();
  }
}
}  // namespace

XNavScroll::XNavScroll(wxWindow *parent)
    : wxScrolledWindow(parent, wxID_ANY, wxDefaultPosition, wxDefaultSize,
                       wxVSCROLL | wxBORDER_NONE) {
  SetScrollRate(0, FromDIP(24));
  ShowScrollbars(wxSHOW_SB_NEVER, wxSHOW_SB_NEVER);
  EnableScrollGesture(*this);
}

bool XNavScroll::Layout() {
  if (!GetSizer()) return wxScrolledWindow::Layout();
  // wx 3.2 ScrollLayout treats a hidden native scrollbar as disabled scrolling
  // and shrinks the sizer to the viewport. Keep full-height content when using
  // our explicit controls, otherwise quality labels and touch rows are clipped.
  const wxSize content(GetClientSize().x,
      std::max(GetVirtualSize().y, GetSizer()->GetMinSize().y));
  GetSizer()->SetDimension(CalcScrolledPosition(wxPoint(0, 0)), content);
  return true;
}

bool XNavScroll::CanScroll(int direction) const {
  int x, y, ux, uy;
  GetViewStart(&x, &y);
  GetScrollPixelsPerUnit(&ux, &uy);
  return direction < 0 ? y > 0
      : y * uy + GetClientSize().y < GetVirtualSize().y;
}

void XNavScroll::Step(int direction) {
  int x, y, ux, uy;
  GetViewStart(&x, &y);
  GetScrollPixelsPerUnit(&ux, &uy);
  if (uy > 0)
    Scroll(0, std::max(0, y + direction * std::max(1, GetClientSize().y * 2 / (3 * uy))));
}

void XNavScroll::Pan(wxPanGestureEvent &event) {
  if (event.IsGestureStart()) pan_remainder_ = 0;
  int x, y, ux, uy;
  GetViewStart(&x, &y);
  GetScrollPixelsPerUnit(&ux, &uy);
  if (uy <= 0) return;
  pan_remainder_ -= event.GetDelta().y;
  const int steps = pan_remainder_ / uy;
  pan_remainder_ %= uy;
  Scroll(0, std::max(0, y + steps));
}

void EnableScrollGesture(wxWindow &window) {
  window.EnableTouchEvents(wxTOUCH_VERTICAL_PAN_GESTURE);
  window.Bind(wxEVT_GESTURE_PAN, [&window](wxPanGestureEvent &event) {
    for (auto *parent = &window; parent; parent = parent->GetParent())
      if (auto *scroll = dynamic_cast<XNavScroll *>(parent)) {
        scroll->Pan(event);
        return;
      }
    event.Skip();
  });
}

bool ThemeWindowChrome(wxWindow &window, LightMode mode) {
#ifdef __WXMSW__
  // Documented DWM attributes; unsupported older Windows retain their native
  // caption. No undocumented ordinals, global theme or registry changes.
  const auto library = LoadLibraryExW(L"dwmapi.dll", nullptr, LOAD_LIBRARY_SEARCH_SYSTEM32);
  if (!library) return false;
  using SetAttribute = HRESULT(WINAPI *)(HWND, DWORD, LPCVOID, DWORD);
  const auto set = reinterpret_cast<SetAttribute>(GetProcAddress(library, "DwmSetWindowAttribute"));
  bool applied = false;
  if (set) {
    const auto c = Theme(mode);
    const BOOL dark = TRUE;
    const COLORREF caption = RGB((c.surface >> 16) & 255, (c.surface >> 8) & 255, c.surface & 255);
    const COLORREF text = RGB((c.secondary >> 16) & 255, (c.secondary >> 8) & 255, c.secondary & 255);
    const auto handle = static_cast<HWND>(window.GetHandle());
    set(handle, 20, &dark, sizeof(dark));
    const auto a = set(handle, 35, &caption, sizeof(caption));
    const auto b = set(handle, 36, &text, sizeof(text));
    applied = SUCCEEDED(a) && SUCCEEDED(b);
  }
  FreeLibrary(library);
  return applied;
#else
  return false;
#endif
}

wxColour Colour(std::uint32_t rgb) {
  return wxColour((rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255);
}

wxFont UiFont(wxWindow& window, int pixels, bool bold) {
  return UiFontWeight(window, pixels, bold ? 700 : 400);
}

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

double UiTextWidth(wxWindow& window, const wxString& text, int pixels, int weight) {
#ifdef __WXMSW__
  struct Release { void operator()(IUnknown *p) const { if(p) p->Release(); } };
  using Factory = std::unique_ptr<IDWriteFactory,Release>;
  static const Factory factory = [] {
    IDWriteFactory *value=nullptr;
    if (FAILED(DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED,__uuidof(IDWriteFactory),
                                  reinterpret_cast<IUnknown **>(&value)))) return Factory{};
    return Factory(value);
  }();
  if(factory) {
    IDWriteTextFormat *raw_format=nullptr;
    const auto face=UiFontWeight(window,pixels,weight).GetFaceName();
    if(SUCCEEDED(factory->CreateTextFormat(face.wc_str(),nullptr,
        static_cast<DWRITE_FONT_WEIGHT>(weight),DWRITE_FONT_STYLE_NORMAL,
        DWRITE_FONT_STRETCH_NORMAL,static_cast<float>(pixels),L"en-US",&raw_format))) {
      std::unique_ptr<IDWriteTextFormat,Release> format(raw_format);
      IDWriteTextLayout *raw_layout=nullptr;
      const auto wide=text.wc_str();
      if(SUCCEEDED(factory->CreateTextLayout(wide,static_cast<UINT32>(wcslen(wide)),
          format.get(),100000.f,1000.f,&raw_layout))) {
        std::unique_ptr<IDWriteTextLayout,Release> layout(raw_layout);
        DWRITE_TEXT_METRICS metrics{};
        if(SUCCEEDED(layout->GetMetrics(&metrics)))
          return metrics.widthIncludingTrailingWhitespace*window.FromDIP(1024)/1024.;
      }
    }
  }
#endif
  wxClientDC dc(&window);dc.SetFont(UiFontWeight(window,pixels,weight));
  return dc.GetTextExtent(text).x;
}

void XNavPainter::Text(wxString text, int x, int y, int size,
                       std::uint32_t color, bool bold, int width) {
  TextWeight(text, x, y, size, color, bold ? 700 : 400, width);
}
void XNavPainter::TextWeight(wxString text, int x, int y, int size,
                             std::uint32_t color, int weight, int width,
                             bool right) {
  dc_.SetFont(UiFontWeight(window_, size, weight));
  dc_.SetTextForeground(Colour(color));
  if (width > 0)
    text = wxControl::Ellipsize(text, dc_, wxELLIPSIZE_END, D(width));
  dc_.DrawText(text, D(x) + (right ? D(width) - dc_.GetTextExtent(text).x : 0), D(y));
}
void XNavPainter::TextTracked(wxString text, int x, int y, int size,
                              std::uint32_t color, int weight,
                              double tracking, int width) {
  dc_.SetFont(UiFontWeight(window_, size, weight));
  dc_.SetTextForeground(Colour(color));
  const double step = tracking * D(100) / 100.;
  const auto extent = [&](const wxString &s) {
    return dc_.GetTextExtent(s).x + step * std::max(0, static_cast<int>(s.length()) - 1);
  };
  if (width > 0 && extent(text) > D(width)) {
    const wxString ellipsis = wxString::FromUTF8("…");
    while (!text.empty() && extent(text + ellipsis) > D(width)) text.RemoveLast();
    text += ellipsis;
  }
  wxArrayInt advances;
  dc_.GetPartialTextExtents(text, advances);
  for (std::size_t i = 0; i < text.length(); ++i)
    dc_.DrawText(text.Mid(i, 1), D(x) + (i ? advances[i - 1] : 0) +
                 static_cast<int>(std::lround(step * i)), D(y));
}
void XNavPainter::Stat(const wxString &label, const wxString &value,
                       const wxString &unit, int x, int y, int width) {
  Text(label, x, y, 9, c.muted, false, width);
  TextWeight(value, x, y + 20, 23, c.primary, 450, width);
  const int extent = window_.ToDIP(dc_.GetTextExtent(value).x);
  if (extent + 4 < width)
    Text(unit, x + extent + 4, y + 32, 10, c.muted, false, width - extent - 4);
}
namespace {
wxColour Mix(std::uint32_t foreground, std::uint32_t background, int alpha) {
  const auto f = Colour(foreground), b = Colour(background);
  return wxColour((f.Red()*alpha + b.Red()*(255-alpha) + 127)/255,
                  (f.Green()*alpha + b.Green()*(255-alpha) + 127)/255,
                  (f.Blue()*alpha + b.Blue()*(255-alpha) + 127)/255);
}
} // namespace
int XNavPainter::Tag(const wxString &text, int x, int y, int maximum, bool attention,
                     bool active) {
  dc_.SetFont(UiFont(window_, 9));
  const int width = (std::min)(maximum, window_.ToDIP(dc_.GetTextExtent(text).x) + 18);
  dc_.SetPen(wxPen(attention
      ? Mix(prototype_ink::warning, c.background, prototype_ink::warning_border_alpha)
      : active ? Mix(prototype_ink::active, c.background, prototype_ink::active_border_alpha)
      : Colour(c.border)));
  dc_.SetBrush(wxBrush(attention
      ? Mix(prototype_ink::warning, c.background, prototype_ink::warning_tag_alpha)
      : active ? Mix(prototype_ink::active, c.background, prototype_ink::active_tag_alpha)
      : Colour(c.selected)));
  dc_.DrawRoundedRectangle(D(x), D(y), D(width), D(26), D(5));
  Text(text, x + 8, y + 6, 9, attention ? c.attention : active ? c.accent : c.secondary, false, width - 16);
  return width;
}
void XNavPainter::Callout(const wxString &title, const wxString &body,
                          int width, int height, bool attention) {
  const auto edge = attention
      ? Mix(prototype_ink::warning, c.background, prototype_ink::warning_border_alpha)
      : Colour(c.accent);
  dc_.SetPen(wxPen(edge));
  dc_.SetBrush(wxBrush(attention
      ? Mix(prototype_ink::warning, c.background, prototype_ink::warning_callout_alpha)
      : Colour(c.selected)));
  dc_.DrawRoundedRectangle(0, 0, D(width) - 1, D(height) - 1, D(8));
  dc_.SetPen(wxPen(edge, D(2)));
  dc_.DrawLine(D(1), 0, D(1), D(height));
  TextWeight(title, 17, 14, 12, attention ? c.attention : c.primary, 550, width - 34);
  dc_.SetFont(UiFont(window_, 12));
  wxStringTokenizer words(body, " ");
  wxString line;
  int y = 38;
  while (words.HasMoreTokens()) {
    const auto word = words.GetNextToken();
    const auto candidate = line.empty() ? word : line + " " + word;
    if (!line.empty() && dc_.GetTextExtent(candidate).x > D(width - 34)) {
      Text(line, 17, y, 12, c.secondary, false, width - 34);
      y += 20; line = word;
      if (y + 18 > height) return;
    } else line = candidate;
  }
  if (!line.empty()) Text(line, 17, y, 12, c.secondary, false, width - 34);
}
void XNavPainter::Card(int x, int y, int width, int height,
                       const wxString &title) {
  dc_.SetPen(wxPen(Colour(c.border)));
  dc_.SetBrush(wxBrush(Colour(c.surface)));
  dc_.DrawRoundedRectangle(D(x), D(y), D(width), D(height), D(spacing::panel_radius));
  Text(title, x + 20, y + 16, 12, c.secondary, false, width - 40);
}
void XNavPainter::Wrapped(const wxString &text, int x, int y, int size,
                          int line_height, int width, std::uint32_t color,
                          int maximum_lines) {
  if(width<=0 || maximum_lines<=0)return;
  dc_.SetFont(UiFont(window_,size));
  wxStringTokenizer words(text," ");wxString line;int row=0;
  while(words.HasMoreTokens()) {
    const auto word=words.GetNextToken();
    const auto candidate=line.empty()?word:line+" "+word;
    if(!line.empty() && dc_.GetTextExtent(candidate).x>D(width) && row+1<maximum_lines) {
      Text(line,x,y+row*line_height,size,color,false,width);
      ++row;line=word;
    } else line=candidate;
  }
  if(!line.empty())Text(line,x,y+row*line_height,size,color,false,width);
}
void XNavPainter::Rule(int x, int y, int width) {
  dc_.SetPen(wxPen(Colour(c.border)));
  dc_.DrawLine(D(x), D(y), D(x + width), D(y));
}

XNavButton::XNavButton(wxWindow* parent, wxWindowID id, const wxString& label,
                       const wxString& accessible_name)
    : wxControl(parent, id, wxDefaultPosition, wxDefaultSize, wxBORDER_NONE) {
  (void)FocusModality();
  SetLabel(label);
  SetName(accessible_name);
  SetHint(accessible_name);
  SetMinSize(FromDIP(wxSize(spacing::touch, spacing::touch)));
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  EnableTouchEvents(wxTOUCH_VERTICAL_PAN_GESTURE);
  Bind(wxEVT_GESTURE_PAN, [this](wxPanGestureEvent &e) {
    pressed_ = false;
    if (HasCapture()) ReleaseMouse();
    Refresh();
    for (auto *p = GetParent(); p; p = p->GetParent())
      if (auto *scroll = dynamic_cast<XNavScroll *>(p)) { scroll->Pan(e); return; }
    e.Skip();
  });
  Bind(wxEVT_PAINT, &XNavButton::Paint, this);
  Bind(wxEVT_ENTER_WINDOW, [this](wxMouseEvent &e) { hovered_ = true; Refresh(); e.Skip(); });
  Bind(wxEVT_LEAVE_WINDOW, [this](wxMouseEvent &e) { hovered_ = false; Refresh(); e.Skip(); });
  Bind(wxEVT_SET_FOCUS, [this](wxFocusEvent& e) {
    keyboard_focus_ = FocusModality().keyboard; Refresh(); e.Skip();
  });
  Bind(wxEVT_KILL_FOCUS, [this](wxFocusEvent& e) { pressed_ = false; Refresh(); e.Skip(); });
  Bind(wxEVT_LEFT_DOWN, [this](wxMouseEvent&) {
    if (!IsEnabled()) return;
    SetFocus();
    keyboard_focus_ = false;
    pressed_ = true;
    CaptureMouse();
    Refresh();
  });
  Bind(wxEVT_LEFT_UP, [this](wxMouseEvent& e) {
    const bool activate = pressed_ && GetClientRect().Contains(e.GetPosition());
    pressed_ = false;
    if (HasCapture()) ReleaseMouse();
    Refresh();
    if (activate) Activate();
  });
  Bind(wxEVT_MOUSE_CAPTURE_LOST, [this](wxMouseCaptureLostEvent&) {
    pressed_ = false;
    Refresh();
  });
  Bind(wxEVT_KEY_DOWN, [this](wxKeyEvent& e) {
    keyboard_focus_ = true;
    if (e.GetKeyCode() == WXK_SPACE || e.GetKeyCode() == WXK_RETURN) {
      pressed_ = IsEnabled();
      Refresh();
    } else e.Skip();
  });
  Bind(wxEVT_KEY_UP, [this](wxKeyEvent& e) {
    if (e.GetKeyCode() == WXK_SPACE || e.GetKeyCode() == WXK_RETURN) {
      const bool activate = pressed_;
      pressed_ = false;
      Refresh();
      if (activate) Activate();
    } else e.Skip();
  });
}

void XNavButton::SetLabel(const wxString &label) {
  if(GetLabel() == label) return;
  wxControl::SetLabel(label);
  Refresh();
}

void XNavButton::SetLightMode(LightMode mode) {
  if (mode_ != mode) {
    mode_ = mode;
    ApplyHint(*this, mode_, hint_);
    Refresh();
  }
}

void XNavButton::SetHint(const wxString &hint) {
  hint_ = hint;
  ApplyHint(*this, mode_, hint_);
}

void XNavButton::Activate() {
  if (!IsEnabled()) return;
  wxCommandEvent event(wxEVT_BUTTON, GetId());
  event.SetEventObject(this);
  // Queue dispatch so a restart callback cannot destroy the active input handler.
  wxPostEvent(this, event);
}

void XNavButton::SetSummary(const wxString &value, const wxString &detail) {
  if (summary_ && summary_value_ == value && summary_detail_ == detail) return;
  summary_ = true; summary_value_ = value; summary_detail_ = detail;
  SetName(GetLabel() + ": " + value + " / " + detail);
  Refresh();
}

void XNavButton::Paint(wxPaintEvent&) {
  wxAutoBufferedPaintDC dc(this);
  auto colors = Theme(mode_);
  if (floating_) {
    const auto floating = FloatingTheme(mode_);
    colors.background = colors.surface = colors.selected = floating.surface;
    colors.primary = colors.secondary = floating.primary;
    colors.muted = floating.secondary;
  }
  const auto size = GetClientSize();
  const auto background = GetParent()->GetBackgroundColour().IsOk()
      ? GetParent()->GetBackgroundColour() : Colour(colors.background);
  dc.SetBackground(wxBrush(background));
  dc.Clear();
  if(vessel_profile_) {
    dc.SetBrush(wxBrush(Colour(colors.selected)));
    dc.SetPen(HasFocus()&&keyboard_focus_ ? wxPen(Colour(colors.accent),FromDIP(1)) : *wxTRANSPARENT_PEN);
    dc.DrawEllipse(0,0,size.x,size.y);
    dc.SetFont(UiFont(*this,11));dc.SetTextForeground(Colour(colors.secondary));
    // No configured vessel identity exists in the current contract. Never
    // reproduce the reference's fictional boat initial or connected state.
    const wxString unknown=wxString::FromUTF8("—");const auto extent=dc.GetTextExtent(unknown);
    dc.DrawText(unknown,(size.x-extent.x)/2,(size.y-extent.y)/2);
    dc.SetPen(wxPen(Colour(colors.background),FromDIP(2)));
    dc.SetBrush(wxBrush(Colour(colors.muted)));
    dc.DrawEllipse(size.x-FromDIP(7),size.y-FromDIP(7),FromDIP(7),FromDIP(7));
    return;
  }
  const auto blend = [](wxColour foreground, wxColour back, double alpha) {
    return wxColour(static_cast<unsigned char>(foreground.Red()*alpha+back.Red()*(1-alpha)),
                    static_cast<unsigned char>(foreground.Green()*alpha+back.Green()*(1-alpha)),
                    static_cast<unsigned char>(foreground.Blue()*alpha+back.Blue()*(1-alpha)));
  };
  const auto opacity = [&](wxColour color) { return IsEnabled() ? color : blend(color, background, .38); };
  const auto semantic = role_ == ButtonRole::Critical ? colors.alarm : colors.accent;
  if (suite_link_) {
    XNavPainter p(*this, dc, mode_);
    const int width = ToDIP(size.x), height = ToDIP(size.y);
    const auto ink = IsEnabled() ? (hovered_ ? colors.accent : colors.primary) : colors.muted;
    dc.SetPen(*wxTRANSPARENT_PEN);
    dc.SetBrush(wxBrush(Colour(hovered_ && IsEnabled() ? colors.selected : colors.elevated)));
    dc.DrawRoundedRectangle(0, FromDIP((height-38)/2), FromDIP(35), FromDIP(38), FromDIP(9));
    const auto icon = wxString::Format(
      "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"24\" height=\"24\"><path d=\"%s\" fill=\"none\" stroke=\"#%06x\" stroke-width=\"1.65\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/></svg>",
      wxString::FromUTF8(PrototypeIconPath(icon_)), IsEnabled()?colors.accent:colors.muted);
    const auto bitmap = wxBitmapBundle::FromSVG(icon.utf8_str(), FromDIP(wxSize(19,19))).GetBitmap(FromDIP(wxSize(19,19)));
    if(bitmap.IsOk())dc.DrawBitmap(bitmap,FromDIP(8),FromDIP((height-19)/2),true);
    p.TextWeight(GetLabel(),47,16,13,ink,600,width-72);
    p.Text(suite_detail_,47,38,11,IsEnabled()?colors.secondary:colors.muted,false,width-72);
    p.Rule(0,height-1,width);
    dc.SetPen(wxPen(Colour(colors.muted),FromDIP(1)));
    dc.DrawLine(FromDIP(width-10),FromDIP(height/2-4),FromDIP(width-6),FromDIP(height/2));
    dc.DrawLine(FromDIP(width-6),FromDIP(height/2),FromDIP(width-10),FromDIP(height/2+4));
    if(HasFocus() && keyboard_focus_) {
      dc.SetBrush(*wxTRANSPARENT_BRUSH);dc.SetPen(wxPen(Colour(colors.accent)));
      dc.DrawRoundedRectangle(1,1,size.x-2,size.y-2,FromDIP(6));
    }
    return;
  }
  wxColour fill = navigation_item_ ? background : Colour(colors.surface);
  wxColour edge = Colour(colors.border);
  wxColour ink = Colour(colors.primary);
  if (settings_tab_) {
    fill = Colour(selected_ ? colors.accent : colors.surface);
    ink = Colour(selected_ ? colors.background : colors.muted);
  } else if (navigation_item_) {
    fill = selected_ ? blend(Colour(colors.accent), background, .05)
                    : hovered_ || pressed_ ? Colour(colors.surface) : background;
    ink = Colour(selected_ ? colors.accent : hovered_ ? colors.primary : colors.muted);
  } else if (role_ == ButtonRole::Primary) {
    fill = Colour(colors.accent); edge = fill; ink = Colour(colors.background);
  } else if (role_ == ButtonRole::Critical) {
    fill = blend(Colour(colors.alarm), background, .063);
    edge = blend(Colour(colors.alarm), background, .25); ink = Colour(colors.alarm);
  } else if (role_ == ButtonRole::Quiet) {
    fill = hovered_ || pressed_ || selected_ ? Colour(colors.selected) : background;
    ink = Colour(selected_ ? colors.accent : colors.secondary);
  } else if (role_ == ButtonRole::Segment) {
    fill = selected_ || hovered_ || pressed_ ? Colour(colors.selected) : background;
    ink = Colour(selected_ ? colors.primary : colors.muted);
  } else if (selected_ || pressed_) fill = Colour(colors.selected);
  if (hovered_ && IsEnabled() && !navigation_item_) {
    fill = wxColour(std::min(255, int(fill.Red()*1.08)),
                    std::min(255, int(fill.Green()*1.08)), std::min(255, int(fill.Blue()*1.08)));
  }
  dc.SetPen(settings_tab_ || navigation_item_ || role_ == ButtonRole::Quiet || role_ == ButtonRole::Segment ? *wxTRANSPARENT_PEN : wxPen(opacity(edge)));
  dc.SetBrush(wxBrush(opacity(fill)));
  dc.DrawRoundedRectangle(1, 1, size.x - 2, size.y - 2, FromDIP(navigation_item_ || summary_ ? 10 : role_ == ButtonRole::Segment ? 6 : spacing::control_radius));
  if (HasFocus() && keyboard_focus_) {
    dc.SetBrush(*wxTRANSPARENT_BRUSH);
    dc.SetPen(wxPen(Colour(semantic), FromDIP(2)));
    dc.DrawRoundedRectangle(FromDIP(2), FromDIP(2), size.x-FromDIP(4), size.y-FromDIP(4), FromDIP(8));
  }
  if (navigation_item_ && selected_) {
    dc.SetPen(wxPen(Colour(colors.accent), FromDIP(2)));
    dc.DrawLine(FromDIP(1), size.y/2-FromDIP(9), FromDIP(1), size.y/2+FromDIP(10));
  }
  const auto text_color = opacity(ink);
  dc.SetTextForeground(text_color);
  dc.SetFont(UiFontWeight(*this, settings_tab_ ? 11 : role_ == ButtonRole::Segment ? 10 : 12, settings_tab_ ? 400 : 500));
  if (settings_tab_) {
#if defined(__WXMSW__) && wxUSE_GRAPHICS_DIRECT2D
    // Match natural DirectWrite advances used by the HTML and tab layout.
    // GDI integer advances are wider and ellipsized three complete labels.
    // This context targets the existing buffered paint DC, not the chart.
    if (auto *renderer=wxGraphicsRenderer::GetDirect2DRenderer()) {
      std::unique_ptr<wxGraphicsContext> graphics(renderer->CreateContextFromUnknownDC(dc));
      if (graphics) {
        const auto font=graphics->CreateFont(11.*FromDIP(1024)/1024.,
            dc.GetFont().GetFaceName(),wxFONTFLAG_DEFAULT,text_color);
        if (!font.IsNull()) {
          graphics->SetFont(font);
          double width=0,height=0;
          graphics->GetTextExtent(GetLabel(),&width,&height);
          graphics->DrawText(GetLabel(),(size.x-width)/2.,(size.y-height)/2.);
          return;
        }
      }
    }
#endif
    // The measured fixed section names fit. Keep them legible if a graphics
    // device cannot initialize; the native visual gate still records fallback.
    const auto extent=dc.GetTextExtent(GetLabel());
    dc.DrawText(GetLabel(),(size.x-extent.x)/2,(size.y-extent.y)/2);
    return;
  }
  if (summary_) {
    XNavPainter p(*this, dc, mode_);
    const int width = ToDIP(size.x), height = ToDIP(size.y);
    const bool compact_pilot = height <= 74, short_helm = height <= 66;
    const int inset = short_helm ? 9 : compact_pilot ? 11 : 13;
    p.TextWeight(GetLabel().Upper(),inset,compact_pilot?8:12,8,colors.secondary,650,width-inset*2-8);
    p.Text(summary_value_,inset,short_helm?25:compact_pilot?26:31,short_helm?16:compact_pilot?18:20,colors.primary,false,width-inset*2);
    p.Text(summary_detail_,inset,short_helm?49:compact_pilot?52:61,short_helm?8:9,colors.muted,false,width-inset*2);
    dc.SetPen(wxPen(Colour(colors.muted),FromDIP(1)));
    dc.DrawLine(FromDIP(width-20),FromDIP(15),FromDIP(width-17),FromDIP(18));
    dc.DrawLine(FromDIP(width-17),FromDIP(18),FromDIP(width-20),FromDIP(21));
    return;
  }
  if (floating_ && icon_ == XNavIcon::Compass) {
    const int center = size.x / 2;
    const auto label = GetLabel();
    dc.SetFont(UiFontWeight(*this, 10, 600));
    const auto letter = label.Left(1);
    dc.DrawText(letter, center-dc.GetTextExtent(letter).x/2, FromDIP(12));
    if (std::isfinite(compass_rotation_)) {
      // Same two-tone 24x29 north arrow as the reference. Its rotation is a
      // copied canvas orientation, never a guessed vessel heading.
      const auto point = [&](double x, double y) {
        const double co=std::cos(compass_rotation_), si=std::sin(compass_rotation_);
        const double scale=FromDIP(1000)/1000.0;
        return wxPoint(center+int(std::lround(scale*(x*co-y*si))),
                       FromDIP(46)+int(std::lround(scale*(x*si+y*co))));
      };
      wxPoint left[]={point(0,-14.5),point(-10.8,14.5),point(0,7.54)};
      wxPoint right[]={point(0,-14.5),point(10.8,14.5),point(0,7.54)};
      dc.SetPen(*wxTRANSPARENT_PEN);dc.SetBrush(wxBrush(text_color));dc.DrawPolygon(3,left);
      dc.SetBrush(wxBrush(opacity(Colour(FloatingTheme(mode_).compass_light))));dc.DrawPolygon(3,right);
    }
    dc.SetFont(UiFont(*this,8));
    const auto caption=label+" up";
    dc.DrawText(caption,center-dc.GetTextExtent(caption).x/2,FromDIP(72));
    return;
  }
  if (icon_ != XNavIcon::None) {
    const int nav_height = ToDIP(size.y);
    const bool compact_nav = navigation_item_ && nav_height <= 51;
    const bool short_nav = navigation_item_ && nav_height <= 43;
    const int icon_px = inline_icon_ ? 17 : short_nav ? 19 : compact_nav ? 20 : icon_size_;
    const int icon_size = FromDIP(icon_px);
    const bool caption = !icon_only_ && !GetLabel().empty();
    const int content_height = short_nav ? 34 : compact_nav ? 37 : 40;
    const int y = caption && !inline_icon_ ? (size.y - FromDIP(content_height))/2 : (size.y-icon_size)/2;
    const auto svg = wxString::Format(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"24\" height=\"24\" viewBox=\"0 0 24 24\">"
        "<path d=\"%s\" fill=\"none\" stroke=\"#%02x%02x%02x\" stroke-width=\"1.65\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/></svg>",
        wxString::FromUTF8(PrototypeIconPath(icon_)), text_color.Red(), text_color.Green(), text_color.Blue());
    const auto bitmap = wxBitmapBundle::FromSVG(svg.utf8_str(), wxSize(icon_size, icon_size)).GetBitmap(wxSize(icon_size, icon_size));
    if (bitmap.IsOk()) dc.DrawBitmap(bitmap, inline_icon_ ? FromDIP(14) : (size.x-icon_size)/2, y, true);
    if (caption) {
      if (inline_icon_) {
        dc.SetFont(UiFont(*this,11));
        const auto label=wxControl::Ellipsize(GetLabel(),dc,wxELLIPSIZE_END,std::max(1,size.x-FromDIP(52)));
        dc.DrawText(label,FromDIP(40),(size.y-dc.GetTextExtent(label).y)/2);
        return;
      }
      dc.SetFont(UiFont(*this, compact_nav ? 9 : navigation_item_ ? 10 : 11));
      const auto label=wxControl::Ellipsize(GetLabel(),dc,wxELLIPSIZE_END,std::max(1,size.x-FromDIP(8)));
      dc.DrawText(label,(size.x-dc.GetTextExtent(label).x)/2,y+FromDIP(short_nav?22:compact_nav?25:29));
    }
    return;
  }
  const auto label=wxControl::Ellipsize(GetLabel(),dc,wxELLIPSIZE_END,std::max(1,size.x-FromDIP(20)));
  const auto extent = dc.GetTextExtent(label);
  dc.DrawText(label, (size.x - extent.x) / 2, (size.y - extent.y) / 2);
}

XNavDataValue::XNavDataValue(wxWindow* parent, const wxString& label,
                             const wxString& unit, int decimals)
    : wxPanel(parent, wxID_ANY), label_(label), unit_(unit), decimals_(decimals) {
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  SetMinSize(FromDIP(wxSize(120, 120)));
  SetName(label + " unavailable");
  EnableScrollGesture(*this);
  Bind(wxEVT_PAINT, &XNavDataValue::Paint, this);
}

XNavDataRail::XNavDataRail(wxWindow *parent) : wxPanel(parent, wxID_ANY) {
  SetSizer(new wxBoxSizer(wxVERTICAL));
}
bool XNavDataRail::Layout() {
  const bool result=wxPanel::Layout();
  const auto count=GetSizer()->GetItemCount();
  if(!count) return result;
  const auto size=GetClientSize();
  for(std::size_t i=0;i<count;++i) {
    const int top=int(std::lround(double(size.y)*i/count));
    const int bottom=int(std::lround(double(size.y)*(i+1)/count));
    if(auto *window=GetSizer()->GetItem(i)->GetWindow())
      window->SetSize(0,top,size.x,bottom-top);
  }
  return result;
}

void XNavDataValue::SetLightMode(LightMode mode) {
  if (mode_ != mode) {
    mode_ = mode;
    ApplyHint(*this, mode_, hint_);
    Refresh();
  }
}
void XNavDataValue::SetCompact(bool compact) {
  compact_ = compact;
  SetMinSize(FromDIP(wxSize(120, compact ? 72 : 120)));
  Refresh();
}

void XNavDataValue::SetReading(const vessel::Sample& sample, vessel::Time now) {
  const auto next = vessel::Assess(sample, now);
  const bool changed = reading_.value != next.value || reading_.quality != next.quality ||
      sample.source != sample_.source ||
      (!next.age || !reading_.age || next.age->count()/1000 != reading_.age->count()/1000);
  sample_ = sample;
  reading_ = next;
  const wxString source = sample.source.empty() ? "No source" : wxString::FromUTF8(sample.source);
  const wxString age = reading_.age
      ? wxString::Format("%.1f s old", reading_.age->count() / 1000.0) : "Age unavailable";
  hint_ = source + "\n" + age;
  ApplyHint(*this, mode_, hint_);
  SetName(label_ + ": " + (reading_.value ? wxString::Format("%.*f", decimals_, *reading_.value) : "Unavailable")
          + " " + unit_ + " / " + wxString::FromUTF8(vessel::QualityName(reading_.quality)));
  if(changed) Refresh();
}

void XNavDataValue::Paint(wxPaintEvent&) {
  wxAutoBufferedPaintDC dc(this);
  const auto colors = Theme(mode_);
  dc.SetBackground(wxBrush(Colour(compact_ ? colors.background : colors.surface)));
  dc.Clear();
  const bool narrow_rail = compact_ && ToDIP(GetClientSize().x) <= 156;
  const int x = FromDIP(compact_ ? (narrow_rail ? 14 : 19) : 12);
  if (compact_) {
    const int height = ToDIP(GetClientSize().y);
    const bool roomy = height >= 108;
    const bool medium = height >= 95;
    const int label_y = roomy ? 10 : medium ? 7 : 5;
    const int right = GetClientSize().x - FromDIP(narrow_rail ? 13 : 18);
    const int available = right - x;
    dc.SetPen(wxPen(Colour(colors.border)));
    dc.DrawLine(x, GetClientSize().y-1, right, GetClientSize().y-1);
    dc.SetFont(UiFont(*this,narrow_rail?9:11)); dc.SetTextForeground(Colour(colors.secondary));
    auto title=label_;
    if(title=="SOG") title="Speed over ground";
    if(title=="DEPTH") title="Depth / transducer";
    if(title=="APPARENT WIND") title="Apparent wind";
    if(title=="TRUE WIND") title="True wind";
    if(title=="SPEED OVER GROUND") title="Speed over ground";
    if(title=="HEADING") title="Heading";
    dc.DrawText(wxControl::Ellipsize(title,dc,wxELLIPSIZE_END,available),x,FromDIP(label_y));
    const bool stale=reading_.quality==vessel::Quality::Stale;
    const auto value=reading_.value?wxString::Format("%.*f",decimals_,*reading_.value):wxString::FromUTF8("—");
    int value_size = roomy ? 48 : medium ? 40 : 33;
    dc.SetFont(UiFont(*this,value_size));
    const int tracking=FromDIP(roomy ? -3 : -2);
    const auto tracked_width=[&]{return dc.GetTextExtent(value).x+tracking*(int(value.length())-1);};
    while(value_size > 24 && tracked_width() > available) {
      value_size -= 2; dc.SetFont(UiFont(*this,value_size));
    }
    dc.SetTextForeground(Colour(stale?colors.muted:colors.primary));
    const int value_y=FromDIP(label_y+(roomy?21:16));
    wxArrayInt advances; dc.GetPartialTextExtents(value,advances);
    for(std::size_t i=0;i<value.length();++i)
      dc.DrawText(value.Mid(i,1),x+(i?advances[i-1]:0)+tracking*int(i),value_y);
    const int unit_x=x+tracked_width()+FromDIP(6);
    dc.SetFont(UiFont(*this,11));dc.SetTextForeground(Colour(colors.secondary));
    auto unit=unit_;if(unit.StartsWith("m /"))unit="m";if(unit=="deg true")unit=wxString::FromUTF8("°T");
    const bool inline_unit=unit_x+dc.GetTextExtent(unit).x<=right;
    if(inline_unit)dc.DrawText(unit,unit_x,value_y+FromDIP(value_size-13));
    wxString status = wxString::FromUTF8(vessel::QualityName(reading_.quality));
    if(reading_.quality==vessel::Quality::Live && !sample_.source.empty())
      status=wxString::FromUTF8(sample_.source);
    dc.SetFont(UiFont(*this,8));dc.SetTextForeground(Colour(stale?colors.attention:colors.muted));
    // If a long numeric value fills the rail, retain its unit on the status
    // line. Never hide the unit or clip a valid heading at a narrower DPI.
    if(!inline_unit)status=unit+"  "+status;
    const auto age=reading_.age ? wxString::Format("%.1f s",reading_.age->count()/1000.0) : wxString{};
    const int age_width=dc.GetTextExtent(age).x;
    const int detail_y=GetClientSize().y-FromDIP(18);
    dc.DrawText(wxControl::Ellipsize(status,dc,wxELLIPSIZE_END,std::max(1,available-age_width-FromDIP(8))),x,detail_y);
    dc.DrawText(age,right-age_width,detail_y);
    return;
  }
  dc.SetPen(wxPen(Colour(colors.border)));
  dc.DrawLine(x, GetClientSize().y - 1, GetClientSize().x - x, GetClientSize().y - 1);
  dc.SetFont(UiFont(*this, 11, true));
  dc.SetTextForeground(Colour(colors.secondary));
  dc.DrawText(label_, x, FromDIP(12));
  const bool stale = reading_.quality == vessel::Quality::Stale;
  dc.SetFont(UiFont(*this, 32, true));
  dc.SetTextForeground(Colour(stale ? colors.muted : colors.primary));
  dc.DrawText(reading_.value ? wxString::Format("%.*f", decimals_, *reading_.value) : wxString::FromUTF8("—"),
              x, FromDIP(30));
  dc.SetFont(UiFont(*this, 13));
  dc.SetTextForeground(Colour(colors.secondary));
  dc.DrawText(unit_, x, FromDIP(69));
  wxString status;
  switch (reading_.quality) {
    case vessel::Quality::Live: status = "LIVE"; break;
    case vessel::Quality::Aging: status = "AGING"; break;
    case vessel::Quality::Stale: status = "STALE"; break;
    case vessel::Quality::Unavailable: status = "UNAVAILABLE"; break;
    case vessel::Quality::Estimated: status = "EST"; break;
    case vessel::Quality::Uncertain: status = "UNCERTAIN"; break;
  }
  if (reading_.age && reading_.quality != vessel::Quality::Live) {
    status += wxString::Format(" %.0fs", reading_.age->count() / 1000.0);
  }
  dc.SetFont(UiFont(*this, 10, true));
  dc.SetTextForeground(Colour(stale ? colors.attention : colors.muted));
  dc.DrawText(status, x, FromDIP(96));
}

}  // namespace opennav::ui
