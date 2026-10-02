#pragma once
#include "ui/Branding.h"
#include "ui/Controls.h"
#include <wx/frame.h>

namespace opennav::ui {
// An owned native tool surface, rather than a sibling painted over a GL child.
// It stays above its owner (not other applications), is never added to the AUI
// perspective, and does not reparent or replace the OpenCPN chart canvas.
class XNavFloatingSurface final : public wxFrame {
public:
  XNavFloatingSurface(wxFrame &owner, const wxString &name, int radius = 12)
      : wxFrame(&owner, wxID_ANY, BrandedSurfaceTitle(name), wxDefaultPosition, wxDefaultSize,
                wxFRAME_NO_TASKBAR | wxFRAME_FLOAT_ON_PARENT | wxFRAME_SHAPED |
                    wxBORDER_NONE), radius_(radius) {
    SetName(name);
    Hide();
    Bind(wxEVT_SIZE, [this](wxSizeEvent &event) {
      Shape();
      event.Skip();
    });
    Bind(wxEVT_CLOSE_WINDOW, [this](wxCloseEvent &event) {
      // Alt-F4 on an owned control must use the application's normal close
      // path, not silently remove a chart control or trap keyboard users.
      wxCloseEvent close(wxEVT_CLOSE_WINDOW, GetParent()->GetId());
      close.SetCanVeto(event.CanVeto());
      wxPostEvent(GetParent(), close);
      if (event.CanVeto()) event.Veto();
    });
  }
  void Present(const wxPoint &screen);
#ifdef __WXGTK__
  bool Show(bool show = true) override;
#endif
private:
  void Shape();
  int radius_;
#ifdef __WXGTK__
  bool show_requested_ = false;
#endif
};
} // namespace opennav::ui
