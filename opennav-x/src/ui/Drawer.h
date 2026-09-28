#pragma once
#include "ui/Controls.h"
#include <functional>
#include <wx/frame.h>
#include <wx/eventfilter.h>
#include <wx/sizer.h>

namespace opennav::ui {
// Prototype .drawer: an owned modeless sheet. It never reparents the chart,
// changes the AUI perspective or disables global navigation/alert controls.
class XNavDrawer : public wxFrame, public wxEventFilter {
public:
  XNavDrawer(wxWindow &owner, const wxString &name);
  ~XNavDrawer() override;
  void Present(const wxRect &workspace);
  void Dismiss();
  void SetHeading(const wxString &eyebrow, const wxString &title, bool back);
  void SetLight(LightMode mode);
  void ClearBody();
  int FilterEvent(wxEvent &event) override;
  std::function<void()> on_back, on_dismiss;

protected:
  XNavScroll *body_ = nullptr;
  wxBoxSizer *content_ = nullptr;
  LightMode light_ = LightMode::Day;

private:
  void Arrange();
  void Paint(wxPaintEvent &);
  void PaintHeading(wxPaintEvent &);
  wxPanel *heading_ = nullptr;
  XNavButton *back_ = nullptr, *close_ = nullptr;
  wxString eyebrow_, title_;
  bool has_back_ = false;
};
} // namespace opennav::ui
