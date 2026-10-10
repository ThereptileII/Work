#pragma once
#include "application/NavigationObjects.h"
#include "application/PilotPresentation.h"
#include "ui/Drawer.h"
#include <array>

namespace opennav::ui {
struct PilotDrawerActions {
  std::function<void(adapters::PilotAction, double)> command;
  std::function<void(bool)> enable;
  // AutoTrack-style take-over: binds the pilot seen live on the OpenCPN NMEA
  // 2000 connection and records control permission. Called only after the
  // user has confirmed the enable sheet; the session is enabled afterwards.
  std::function<application::CommandResult()> take_control;
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
  bool CanTakeControl() const;
  PilotDrawerActions actions_;
  application::PilotPresentation view_;
  std::optional<double> rudder_;
  wxPanel *panel_ = nullptr;
  std::array<XNavButton *, 4> course_{};
  std::array<XNavButton *, 4> modes_{};
  XNavButton *enable_ = nullptr;
  bool simulated_ = false, queued_ = false;
  // Set after a confirmed take-over while the adapter verifies the new
  // binding; the session is enabled as soon as it does, or abandoned.
  std::optional<vessel::Time> enable_after_bind_;
  wxString notice_;
  wxString blocker_;
};
} // namespace opennav::ui
