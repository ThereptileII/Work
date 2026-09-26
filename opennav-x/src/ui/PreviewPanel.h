#pragma once
#include "smartnav/VesselEnergy.h"
#include "smartnav/Advisories.h"
#include "ui/Controls.h"
#include <vector>
#include <wx/scrolwin.h>

namespace opennav::ui {
enum class PreviewPage { Route, Energy, Diagnostics };
class PreviewPanel final : public XNavScroll {
public:
  explicit PreviewPanel(wxWindow *parent);
  void Update(PreviewPage page, LightMode mode,
              const vessel::VesselState &state, vessel::Time now,
              const smartnav::EnergyModel &model,
              const smartnav::EnergyPrediction &energy,
              const std::vector<std::string> &build_info,
              const smartnav::NavigationAdvice &advice = {});

private:
  void Paint(wxPaintEvent &);
  PreviewPage page_ = PreviewPage::Route;
  LightMode mode_ = LightMode::Day;
  vessel::VesselState state_;
  vessel::Time now_{};
  smartnav::EnergyModel model_;
  smartnav::EnergyPrediction energy_;
  std::vector<std::string> info_;
  smartnav::NavigationAdvice advice_;
};
} // namespace opennav::ui
