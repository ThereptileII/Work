#pragma once
#include "application/OnlineAis.h"
#include "adapters/Autopilot.h"
#include "vessel/SensorRegistry.h"
#include <vector>

namespace opennav::application {
enum class SignalState { Current, Aging, Stale, Estimated, Uncertain, Invalid, Unavailable };
struct HealthSignal {
  std::string id, title, status, source, measurement, note;
  SignalState state = SignalState::Unavailable;
  std::optional<vessel::Duration> age;
  std::optional<double> frequency_hz;
  std::optional<unsigned> priority;
  std::optional<vessel::Quantity> quantity;
};
struct SourceHealthView {
  std::vector<HealthSignal> signals;
  bool historical = false;
};
// Owned presentation only. No observation is refreshed and no connection or
// source-selection mutation is possible. Onboard and Internet AIS stay separate.
SourceHealthView PresentSourceHealth(const vessel::VesselState &,
    const std::vector<vessel::SourceHealth> &, const vessel::AisState &onboard,
    const OnlineAisState &, const adapters::PilotView &, vessel::Time now);
HealthSignal PresentPositionHealth(const vessel::Navigation &, vessel::Time now);
HealthSignal PresentSignalHealth(const vessel::Sample &, vessel::Time now);
} // namespace opennav::application
