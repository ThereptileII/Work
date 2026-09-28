#pragma once
#include "application/InstrumentView.h"
#include "ui/Controls.h"
#include <functional>

namespace opennav::ui {
// A native full-view body. Its parent owns scrolling; it never owns a canvas,
// marine input, route, or hardware adapter.
class XNavInstrumentPanel final : public wxPanel {
public:
  explicit XNavInstrumentPanel(wxWindow *parent);
  void Update(const vessel::VesselState &, const std::vector<std::string> &,
              vessel::Time now, LightMode);
  const application::InstrumentView &View() const { return view_; }
  std::vector<std::pair<wxString, wxRect>> Regions() const;
  std::vector<XNavButton *> Controls() const {
    return {close_, rail_, health_, configure_};
  }
  std::function<void()> on_close, on_rail, on_health, on_configure;

private:
  void Reflow();
  void Paint(wxPaintEvent &);
  void Wind(wxDC &, XNavPainter &, int x, int y, int width);
  void Reading(wxDC &, XNavPainter &, const application::InstrumentReading &,
               int x, int y, int width, bool tile);
  LightMode light_ = LightMode::Day;
  application::InstrumentView view_;
  XNavButton *close_, *rail_, *health_, *configure_;
};
} // namespace opennav::ui
