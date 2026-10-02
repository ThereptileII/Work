#include "ui/Branding.h"
#include "ui/Drawer.h"
#include "ui/PrototypeGeometry.h"
#include "ui/ChoiceField.h"
#include "ui/DisplaySizing.h"
#include <wx/dcbuffer.h>
#include <wx/dialog.h>
#include <wx/frame.h>
#include <wx/graphics.h>
#include <wx/popupwin.h>
#include <wx/sizer.h>
#ifdef __WXGTK__
#include <gtk/gtk.h>
#endif

namespace opennav::ui {
XNavDrawer::XNavDrawer(wxWindow &owner, const wxString &name)
    : wxFrame(&owner, wxID_ANY, BrandedSurfaceTitle(name), wxDefaultPosition, wxDefaultSize,
              wxBORDER_NONE | wxFRAME_SHAPED | wxTAB_TRAVERSAL |
              wxFRAME_NO_TASKBAR | wxFRAME_FLOAT_ON_PARENT) {
  SetName(name);
  SetBackgroundStyle(wxBG_STYLE_PAINT);
  heading_ = new wxPanel(this, wxID_ANY);
  heading_->SetLabel(wxEmptyString);
  heading_->SetBackgroundStyle(wxBG_STYLE_PAINT);
  body_ = new XNavScroll(this);
  body_->SetLabel(wxEmptyString);
  content_ = new wxBoxSizer(wxVERTICAL);
  auto *padding = new wxBoxSizer(wxVERTICAL);
  padding->AddSpacer(FromDIP(20));
  padding->Add(content_, 1, wxEXPAND | wxLEFT | wxRIGHT, FromDIP(22));
  padding->AddSpacer(FromDIP(24));
  body_->SetSizer(padding);
  close_ = new XNavButton(heading_, wxID_ANY, "Close", "Close sheet");
  close_->SetRole(ButtonRole::Quiet);
  close_->SetIcon(XNavIcon::Close);
  close_->SetInlineIcon();
  back_ = new XNavButton(heading_, wxID_ANY, "Back", "Back");
  back_->SetRole(ButtonRole::Quiet);
  back_->SetIcon(XNavIcon::Back);
  back_->SetInlineIcon();
  back_->Hide();
  close_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) { Dismiss(); });
  back_->Bind(wxEVT_BUTTON, [this](wxCommandEvent &) {
    if (on_back)
      on_back();
  });
  Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &) { Dismiss(); });
  Bind(wxEVT_SIZE, [this](wxSizeEvent &e) {
    Arrange();
    e.Skip();
  });
  Bind(wxEVT_PAINT, &XNavDrawer::Paint, this);
  heading_->Bind(wxEVT_PAINT, &XNavDrawer::PaintHeading, this);
  wxEvtHandler::AddFilter(this);
  SetLight(light_);
}
XNavDrawer::~XNavDrawer() { wxEvtHandler::RemoveFilter(this); }
void XNavDrawer::Dismiss() {
  Hide();
  if (on_dismiss)
    on_dismiss();
}
void XNavDrawer::Present(const wxRect &workspace) {
  auto area = workspace;
  area.Deflate(FromDIP(prototype::drawer_gap), FromDIP(prototype::drawer_top));
  const auto viewport=GetParent()->ToDIP(GetParent()->GetClientSize());
  const auto layout=prototype::Desktop(viewport.x,viewport.y);
  const int scaled_wide = viewport.x<=760 ? 0 : interface_scale_ == 150 ? 480 :
      interface_scale_ == 125 ? 460 : layout.wide_drawer_width;
  const int logical_width=wide_?scaled_wide:layout.drawer_width;
  const int width = logical_width ? (std::min)(FromDIP(logical_width),area.width) : area.width;
  if (width < FromDIP(280) || area.height < FromDIP(260)) {
    Hide();
    return;
  }
  const wxRect desired(area.x + area.width - width, area.y, width, area.height);
  if (GetScreenRect() != desired)
    SetSize(desired);
  if (!IsShown()) {
    ShowWithoutActivating();
    Arrange();
    Raise();
  }
#ifdef __WXGTK__
  // Preserve owned-window stacking even on an X11 test desktop without a
  // window manager. Restacking does not activate or make the sheet topmost.
  auto *surface = gtk_widget_get_window(GetHandle());
  auto *owner = gtk_widget_get_window(GetParent()->GetHandle());
  if (surface && owner) gdk_window_restack(surface, owner, true);
#endif
}
void XNavDrawer::SetHeading(const wxString &eyebrow, const wxString &title,
                            bool back) {
  const bool changed =
      eyebrow_ != eyebrow || title_ != title || has_back_ != back;
  eyebrow_ = eyebrow;
  title_ = title;
  has_back_ = back;
  if (changed) {
    back_->Show(back);
    close_->Show(!back);
    Arrange();
    heading_->Refresh(false);
  }
}
void XNavDrawer::SetLight(LightMode mode) {
  light_ = mode;
  const auto c = Theme(mode);
  SetBackgroundColour(Colour(c.background));
  heading_->SetBackgroundColour(Colour(c.background));
  body_->SetBackgroundColour(Colour(c.background));
  close_->SetLightMode(mode);
  back_->SetLightMode(mode);
  Refresh();
}
void XNavDrawer::SetInterfaceScale(int percent) {
  if (!ValidInterfaceScale(percent)) return;
  if (interface_scale_ == percent) return;
  interface_scale_ = percent;
  // Only controls explicitly marked as prototype .btn equivalents resize;
  // icon targets, tabs, segments and actuator controls retain their contracts.
  const auto apply=[percent](const auto &self,wxWindow *window)->void {
    for(auto *child:window->GetChildren()) {
      if(auto *button=dynamic_cast<XNavButton *>(child))button->SetInterfaceScale(percent);
      if(auto *field=dynamic_cast<XNavChoiceField *>(child))field->SetInterfaceScale(percent);
      if(!child->IsTopLevel())self(self,child);
    }
  };
  apply(apply,this);
  body_->SetFont(UiFont(*body_,percent==125?15:14));
  body_->Layout();body_->FitInside();
}
void XNavDrawer::ClearBody() {
  content_->Clear(true);
  body_->Scroll(0, 0);
}
void XNavDrawer::Arrange() {
  const auto size = GetClientSize();
  const int head = FromDIP(has_back_ ? 139 : 89);
  heading_->SetSize(FromDIP(1), FromDIP(1), size.x - FromDIP(2), head);
  body_->SetSize(FromDIP(1), FromDIP(1) + head, size.x - FromDIP(2),
                 (std::max)(1, size.y - head - FromDIP(2)));
  back_->SetSize(FromDIP(8), FromDIP(8), FromDIP(94), FromDIP(44));
  close_->SetSize(size.x - FromDIP(96), FromDIP(18), FromDIP(86), FromDIP(44));
  body_->FitInside();
  body_->Layout();
  auto *renderer = wxGraphicsRenderer::GetDefaultRenderer();
  if (renderer && size.x > 0 && size.y > 0) {
    auto path = renderer->CreatePath();
    path.AddRoundedRectangle(0, 0, size.x, size.y,
                             FromDIP(prototype::drawer_radius));
    SetShape(path);
  }
}
void XNavDrawer::Paint(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(this);
  const auto c = Theme(light_);
  dc.SetBackground(wxBrush(Colour(c.background)));
  dc.Clear();
  dc.SetBrush(*wxTRANSPARENT_BRUSH);
  dc.SetPen(wxPen(Colour(c.border)));
  const auto s = GetClientSize();
#ifdef __WXMSW__
  // GDI excludes the right/bottom extent. Keep those strokes in the outer
  // pixel reserved by Arrange(), rather than underneath its child windows.
  dc.DrawRoundedRectangle(0, 0, s.x, s.y,
                          FromDIP(prototype::drawer_radius));
#else
  dc.DrawRoundedRectangle(0, 0, s.x - 1, s.y - 1,
                          FromDIP(prototype::drawer_radius));
#endif
}
void XNavDrawer::PaintHeading(wxPaintEvent &) {
  wxAutoBufferedPaintDC dc(heading_);
  XNavPainter p(*heading_, dc, light_);
  dc.SetBackground(wxBrush(Colour(p.c.background)));
  dc.Clear();
  const int offset = has_back_ ? 50 : 0,
            width = ToDIP(heading_->GetClientSize().x);
  p.TextTracked(eyebrow_, 22, 20 + offset, 9, p.c.accent, 650, 1.17,
                width - 44);
#ifdef __WXMSW__
  // Native29 title ink was 4px below the canonical Windows CSS line box.
  // Correct only this GDI title placement; header/body geometry is unchanged.
  constexpr int title_y = 35;
#else
  constexpr int title_y = 39;
#endif
  p.TextTracked(title_, 22, title_y + offset, 26, p.c.primary, 450, -1.0,
                width - (has_back_ ? 44 : 110));
  p.Rule(0, ToDIP(heading_->GetClientSize().y) - 1, width);
}
int XNavDrawer::FilterEvent(wxEvent &event) {
  if (!IsShownOnScreen() || !IsEnabled())
    return Event_Skip;
  // GTK may keep an owned drawer logically enabled while its modal child
  // blocks input. Escape belongs to that modal; it must not navigate the
  // underlying drawer or leave the password prompt orphaned.
  for (auto *window : wxTopLevelWindows)
    if (auto *dialog = dynamic_cast<wxDialog *>(window);
        dialog && dialog->IsModal())
      return Event_Skip;
  if (event.GetEventType() == wxEVT_CHAR_HOOK) {
    const auto *key = dynamic_cast<wxKeyEvent *>(&event);
    if (key && (key->GetKeyCode() == WXK_ESCAPE ||
                (key->AltDown() && key->GetKeyCode() == WXK_LEFT))) {
      for (auto *target=dynamic_cast<wxWindow *>(event.GetEventObject());target;
           target=target->GetParent())
        if (dynamic_cast<wxPopupTransientWindow *>(target)) return Event_Skip;
      // A choice popup owns Escape while open. Dismissing the underlying sheet
      // would discard the user's draft and leave the transient surface orphaned.
      for (auto *window : wxTopLevelWindows) {
        if (!dynamic_cast<wxPopupTransientWindow *>(window) || !window->IsShown()) continue;
        for (auto *parent=window->GetParent();parent;parent=parent->GetParent())
          if (parent==this) return Event_Skip;
      }
      if (has_back_ && on_back)
        CallAfter(on_back);
      else
        Dismiss();
      return Event_Processed;
    }
  }
  return Event_Skip;
}
} // namespace opennav::ui
