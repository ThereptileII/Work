#pragma once
#include "application/SourceHealthView.h"
#include "application/NavigationObjects.h"

namespace opennav::application {
struct FooterView {
  std::string navigation_state = "NO POSITION";
  std::string position = "GPS POSITION UNAVAILABLE";
  std::string cog = "—", xte = "—";
  std::string health_source = "Vessel data", health_summary = "0 live signals";
  SignalState position_state = SignalState::Unavailable;
  SignalState cog_state = SignalState::Unavailable;
  SignalState health_state = SignalState::Unavailable;
  unsigned live_signals = 0, aging_signals = 0, stale_signals = 0;
  bool historical = false;
};
// Formats copied observations only. The health summary counts the named
// onboard measurements in SourceHealthView, never transport connections.
// Online AIS is deliberately excluded; its independent row remains in Health.
FooterView PresentFooter(const vessel::VesselState &, const AnchorState &,
                         const SourceHealthView &, vessel::Time now);
} // namespace opennav::application
