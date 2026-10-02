#pragma once

#include "application/NavigationObjects.h"
#include "ui/Drawer.h"
#include <array>

namespace opennav::ui {
// Presentation reads copied native chart state. Only explicit human actions
// invoke callbacks; selections always come from the returned native readback.
class XNavChartPresentationDrawer final : public XNavDrawer {
public:
  XNavChartPresentationDrawer(wxWindow &, application::NavigationActions);
  void Open(const wxRect &, const application::ChartPresentationState &, LightMode);
  void Update(const application::ChartPresentationState &, LightMode);
  // Keeps the existing XNav / Standard palette preference separate from the
  // observed Vector / Raster chart format.
  std::function<void()> on_style_preferences;

private:
  void ChangeLayer(unsigned);
  void ChangeOrientation(application::ChartOrientation);
  void Accept(application::ChartPresentationResult);
  void PaintRows(wxPaintEvent &);
  void PaintFormat(wxPaintEvent &);
  void ReflowNotes();
  wxPanel *CopyBlock(int, std::function<void(XNavPainter &, int)>);
  application::NavigationActions actions_;
  application::ChartPresentationState state_;
  wxString feedback_;
  std::uint64_t command_generation_ = 0;
  bool rendered_ = false, style_available_ = false;
  wxPanel *format_ = nullptr, *rows_ = nullptr, *orientation_track_ = nullptr;
  wxPanel *notes_ = nullptr;
  std::vector<std::pair<wxString, int>> note_lines_;
  std::array<XNavButton *, 3> layers_{}, orientations_{};
  XNavButton *style_ = nullptr;
  std::vector<wxPanel *> copies_;
};
} // namespace opennav::ui
