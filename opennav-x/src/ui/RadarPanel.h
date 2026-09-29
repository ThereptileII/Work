#pragma once
#include "adapters/Radar.h"
#include "ui/Controls.h"
#include <functional>

namespace opennav::ui {
// Native prototype composition over copied adapter status. No receive-image
// contract exists yet: this view cannot invent echoes or issue scanner
// commands.
class XNavRadarPanel final : public wxPanel {
public:
  explicit XNavRadarPanel(wxWindow *parent);
  void Update(const adapters::RadarState &, vessel::Time now, bool replay,
              LightMode);
  std::vector<std::pair<wxString, wxRect>> Regions() const;
  std::vector<XNavButton *> Controls() const {
    return {close_, active_, guard_, pause_, plugins_};
  }
  std::function<void()> on_close, on_plugins;

private:
  void Reflow();
  void Paint(wxPaintEvent &);
  void PaintControls(wxPaintEvent &);
  XNavScroll *controls_ = nullptr;
  wxPanel *control_body_ = nullptr;
  adapters::RadarState state_;
  bool replay_ = false, status_current_ = false, initialized_ = false;
  LightMode light_ = LightMode::Day;
  XNavButton *close_, *active_, *guard_, *pause_, *plugins_;
};
} // namespace opennav::ui
