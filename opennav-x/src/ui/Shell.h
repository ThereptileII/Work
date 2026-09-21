#pragma once

#include "ui/Controls.h"

#include <wx/aui/aui.h>
#include <wx/frame.h>
#include <wx/stattext.h>
#include <wx/timer.h>

#include <functional>
#include <vector>

namespace opennav::ui {

struct ShellActions {
  std::function<void()> zoom_in, zoom_out, follow, legacy;
  std::function<void(LightMode)> theme;
};

// Chrome only. The existing OpenCPN ChartCanvas stays in its existing AUI pane.
// Destroy before the owning frame's AUI manager is uninitialized.
class Shell final : public wxEvtHandler {
 public:
  Shell(wxFrame& frame, wxAuiManager& manager, ShellActions actions,
        LightMode mode, bool simulation);
  ~Shell() override;
  void UpdateState(const vessel::VesselState& state);

 private:
  wxPanel* MakePane(const wxString& name, wxAuiPaneInfo placement);
  XNavButton* Button(wxWindow* parent, const wxString& text,
                     const wxString& name, std::function<void()> callback);
  wxStaticText* Text(wxWindow* parent, const wxString& text, int size, bool bold = false);
  void ApplyTheme();
  void Tick();
  void ShowSystem();
  wxFrame& frame_;
  wxAuiManager& manager_;
  ShellActions actions_;
  LightMode mode_;
  vessel::VesselState state_;
  bool simulation_ = false;
  bool simulation_paused_ = false;
  wxTimer timer_;
  std::vector<wxPanel*> panes_;
  std::vector<XNavButton*> buttons_;
  std::vector<wxStaticText*> labels_;
  XNavDataValue *wind_, *depth_, *speed_, *course_;
  wxStaticText *clock_, *source_;
};

}  // namespace opennav::ui
