#pragma once
#include "application/PilotPresentation.h"
#include "ui/Drawer.h"
#include <array>

namespace opennav::ui {
struct PilotDrawerActions {
  std::function<void(adapters::PilotAction, double)> command;
  std::function<void(bool)> enable;
  std::function<void()> settings;
};
class XNavPilotDrawer final : public XNavDrawer {
public:
  XNavPilotDrawer(wxWindow &, PilotDrawerActions);
  void Update(const adapters::PilotView &, vessel::Time pilot_now,
              const vessel::VesselState &, vessel::Time vessel_now,
              bool permit_control, LightMode);
  const application::PilotPresentation &View() const { return view_; }

private:
  void Paint(wxPaintEvent &);
  void Arrange();
  void RefreshControls();
  bool Allowed(adapters::PilotAction) const;
  void Request(adapters::PilotAction, double);
  void Toggle();
  PilotDrawerActions actions_;
  application::PilotPresentation view_;
  std::optional<double> rudder_;
  wxPanel *panel_ = nullptr;
  std::array<XNavButton *, 4> course_{};
  std::array<XNavButton *, 4> modes_{};
  XNavButton *enable_ = nullptr, *settings_ = nullptr;
  bool simulated_ = false, queued_ = false;
};
} // namespace opennav::ui
